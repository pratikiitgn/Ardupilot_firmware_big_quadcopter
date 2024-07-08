/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  simulate a slung payload
*/

#include "SIM_config.h"

#if AP_SIM_SLUNGPAYLOAD_ENABLED

#include "SIM_SlungPayload.h"
#include "SITL.h"
#include <stdio.h>
#include "SIM_Aircraft.h"
#include <AP_HAL_SITL/SITL_State.h>
#include <GCS_MAVLink/GCS.h>

using namespace SITL;

// SlungPayloadSim parameters
const AP_Param::GroupInfo SlungPayloadSim::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Slung Payload Sim enable/disable
    // @Description: Slung Payload Sim enable/disable
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("ENABLE",  1, SlungPayloadSim,  enable, 0),

    // @Param: WEIGHT
    // @DisplayName: Slung Payload weight
    // @Description: Slung Payload weight in kg
    // @Units: kg
    // @Range: 0 15
    // @User: Advanced
    AP_GROUPINFO("WEIGHT",  2, SlungPayloadSim,  weight_kg, 1.0),

    // @Param: LINELEN
    // @DisplayName: Slung Payload line length
    // @Description: Slung Payload line length in meters
    // @Units: m
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("LINELEN", 3, SlungPayloadSim,  line_length, 30.0),

    // @Param: DRAG
    // @DisplayName: Slung Payload drag coefficient
    // @Description: Slung Payload drag coefficient.  Higher values increase drag and slow the payload more quickly
    // @Units: m
    // @Range: 0 10
    // @User: Advanced
    AP_GROUPINFO("DRAG",    4, SlungPayloadSim,  drag_coef, 1),

    // @Param: SYSID
    // @DisplayName: Slung Payload MAVLink sysmtem ID
    // @Description: Slung Payload MAVLink system id to distinguish it from others on the same network
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("SYSID",   5, SlungPayloadSim,  sys_id, 2),

    AP_GROUPEND
};

// SlungPayloadSim handles interaction with main vehicle
SlungPayloadSim::SlungPayloadSim()
{
    AP_Param::setup_object_defaults(this, var_info);
}

// update the SlungPayloadSim's state using the vehicle's earth-frame position, velocity and acceleration
void SlungPayloadSim::update(const Vector3p& veh_pos, const Vector3f& veh_vel_ef, const Vector3f& veh_accel_ef)
{
    if (!enable) {
        return;
    }

    // initialise slung payload location
    const uint32_t now_us = AP_HAL::micros();
    if (!initialised) {
        // capture EKF origin
        auto *sitl = AP::sitl();
        const Location ekf_origin = sitl->state.home;
        if (ekf_origin.lat == 0 && ekf_origin.lng == 0) {
            return;
        }

        // more initialisation
        last_update_us = now_us;
        initialised = true;
    }

    // calculate dt and update slung payload
    const float dt = (now_us - last_update_us)*1.0e-6;
    last_update_us = now_us;
    update_payload(veh_pos, veh_vel_ef, veh_accel_ef, dt);

    // send payload location to GCS at 5hz
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_report_ms >= reporting_period_ms) {
        last_report_ms = now_ms;
        send_report();
    }
}

// get earth-frame forces on the vehicle from slung payload
// returns true on success and fills in forces_ef argument, false on failure
bool SlungPayloadSim::get_forces_on_vehicle(Vector3f& forces_ef) const
{
    if (!enable) {
        return false;
    }

    forces_ef = veh_forces_ef;
    return true;
}

// send a report to the vehicle control code over MAVLink
void SlungPayloadSim::send_report(void)
{
    if (!mavlink_connected && mav_socket.connect(target_address, target_port)) {
        ::printf("SlungPayloadSim connected to %s:%u\n", target_address, (unsigned)target_port);
        mavlink_connected = true;
    }
    if (!mavlink_connected) {
        return;
    }

    // get current time
    uint32_t now_ms = AP_HAL::millis();

    // send heartbeat at 1hz
    const uint8_t component_id = MAV_COMP_ID_USER11;
    if (now_ms - last_heartbeat_ms >= 1000) {
        last_heartbeat_ms = now_ms;

        const mavlink_heartbeat_t heartbeat{
            custom_mode: 0,
            type : MAV_TYPE_AIRSHIP,
            autopilot : MAV_AUTOPILOT_INVALID,
            base_mode: 0,
            system_status: 0,
            mavlink_version: 0,
        };

        mavlink_message_t msg;
        mavlink_msg_heartbeat_encode_status(
            sys_id.get(),
            component_id,
            &mav_status,
            &msg,
            &heartbeat);
        uint8_t buf[300];
        const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        mav_socket.send(buf, len);
    }

    // send a GLOBAL_POSITION_INT messages
    {
        Location payload_loc;
        int32_t alt_amsl_cm, alt_rel_cm;
        if (!get_payload_location(payload_loc) ||
            !payload_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, alt_amsl_cm) ||
            !payload_loc.get_alt_cm(Location::AltFrame::ABOVE_HOME, alt_rel_cm)) {
            return;
        }
        const mavlink_global_position_int_t global_position_int{
            time_boot_ms: now_ms,
            lat: payload_loc.lat,
            lon: payload_loc.lng,
            alt: alt_amsl_cm * 10,              // amsl alt in mm
            relative_alt: alt_rel_cm * 10,      // relative alt in mm
            vx: int16_t(velocity_NED.x * 100),  // velocity in cm/s
            vy: int16_t(velocity_NED.y * 100),  // velocity in cm/s
            vz: int16_t(velocity_NED.z * 100),  // velocity in cm/s
            hdg: 0                              // heading in centi-degrees
        };
        mavlink_message_t msg;
        mavlink_msg_global_position_int_encode_status(sys_id, component_id, &mav_status, &msg, &global_position_int);
        uint8_t buf[300];
        const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        if (len > 0) {
            mav_socket.send(buf, len);
        }
    }

    // send ATTITUDE so MissionPlanner can display orientation
    {
        const mavlink_attitude_t attitude{
            time_boot_ms: now_ms,
            roll: 0,
            pitch: 0,
            yaw: 0,         // heading in radians
            rollspeed: 0,
            pitchspeed: 0,
            yawspeed: 0
        };
        mavlink_message_t msg;
        mavlink_msg_attitude_encode_status(
                sys_id,
                component_id,
                &mav_status,
                &msg,
                &attitude);
        uint8_t buf[300];
        const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        if (len > 0) {
            mav_socket.send(buf, len);
        }
    }
}

// returns true on success and fills in payload_loc argument, false on failure
bool SlungPayloadSim::get_payload_location(Location& payload_loc) const
{
    // get EKF origin
    auto *sitl = AP::sitl();
    if (sitl == nullptr) {
        return false;
    }
    const Location ekf_origin = sitl->state.home;
    if (ekf_origin.lat == 0 && ekf_origin.lng == 0) {
        return false;
    }

    // calculate location
    payload_loc = ekf_origin;
    payload_loc.offset(position_NED);
    return true;
}

// update the slung payloads position, velocity, acceleration
// vehicle position, velocity and acceleration should be in earth-frame NED frame
void SlungPayloadSim::update_payload(const Vector3p& veh_pos, const Vector3f& veh_vel_ef, const Vector3f& veh_accel_ef, float dt)
{
    // how we calculate the payload's position, velocity and acceleration
    //   1. update the payload's position, velocity using the previous iterations acceleration
    //   2. check that the payload does not fall below the terrain
    //   3. check if the line is taught and that the payload does not move more than the line length from the vehicle
    //   4. calculate gravity and drag forces on the payload
    //   5. calculate the tension force between the payload and vehicle including force countering gravity, drag and centripetal force
    //   6. update the payload's acceleration using the sum of the above forces

    // initialise position_NED from vehicle position
    if (position_NED.is_zero()) {
        if (!veh_pos.is_zero()) {
            position_NED = veh_pos;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SlungPayload: initialised at %f %f %f", position_NED.x, position_NED.y, position_NED.z);
        }
        return;
    }

    // integrate previous iterations acceleration into velocity and position
    velocity_NED += accel_NED * dt;
    position_NED += (velocity_NED * dt).todouble();

    // calculate distance from payload to vehicle
    Vector3p payload_to_veh = veh_pos - position_NED;
    float payload_to_veh_length = payload_to_veh.length();

    // update landed state by checking if payload has dropped below terrain
    Location payload_loc;
    if (get_payload_location(payload_loc)) {
        int32_t alt_terrain_cm;
        bool landed_orig = landed;
        if (payload_loc.get_alt_cm(Location::AltFrame::ABOVE_TERRAIN, alt_terrain_cm)) {

            // landed if below terrain
            if (alt_terrain_cm <= 0) {
                landed = true;

                // raise payload to match terrain
                position_NED.z += (alt_terrain_cm * 0.01);

                // zero out velocity and acceleration in horizontal and downward direction
                velocity_NED.xy().zero();
                velocity_NED.z = MIN(velocity_NED.z, 0);
                accel_NED.xy().zero();
                accel_NED.z = MIN(accel_NED.z, 0);

                // zero out forces on vehicle
                veh_forces_ef.zero();
            }

            // not landed if above terrain
            if (landed && (alt_terrain_cm > 1)) {
                landed = false;
            }
        }

        // inform user if landed state has changed
        if (landed != landed_orig) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SlungPayload: %s", landed ? "landed" : "liftoff");
        }
    }

    // calculate forces of gravity
    Vector3f force_gravity_NED = Vector3f(0.0f, 0.0f, GRAVITY_MSS * weight_kg);

    // tension force on payload (resists gravity, drag, centripetal force)
    Vector3f tension_force_NED;

    // calculate drag force (0.5 * drag_coef * air_density * velocity^2 * surface area)
    Vector3f force_drag_NED;
    if (drag_coef > 0 && !velocity_NED.is_zero()) {
        const float air_density = 1.225;    // 1.225 kg/m^3 (standard sea-level density)
        const float surface_area_m2 = 0.07; // 30cm diameter sphere
        const float drag_force = 0.5 * drag_coef * air_density * velocity_NED.length_squared() * surface_area_m2;
        force_drag_NED = -velocity_NED.normalized() * drag_force;
    }

    // sanity check payload distance from vehicle and calculate tension force
    if (is_positive(payload_to_veh_length)) {

        // calculate unit vector from payload to vehicle
        const Vector3f payload_to_veh_norm = payload_to_veh.normalized().tofloat();

        // ensure payload is no more than line_length from vehicle
        if (payload_to_veh_length > line_length) {
            payload_to_veh *= (line_length / payload_to_veh_length);
            position_NED = veh_pos - payload_to_veh;
            line_is_taut = true;
        }

        // check if line becomes loose by more than 1cm
        if (payload_to_veh_length < line_length - 0.01) {
            line_is_taut = false;
        }

        // calculate tension forces when line is taut
        if (line_is_taut) {

            // tension resists gravity if vehicle is above payload
            if (is_negative(payload_to_veh_norm.z)) {
                tension_force_NED += -force_gravity_NED.projected(payload_to_veh_norm);
            }

            // tension resists vehicle acceleration
            // tension_force_NED += (-veh_accel_ef * weight_kg).projected(payload_to_veh_norm);

            // tension force resisting payload drag
            tension_force_NED += -force_drag_NED.projected(payload_to_veh_norm);

            // calculate centripetal force
            const Vector3f velocity_parallel = velocity_NED.projected(payload_to_veh_norm);
            const Vector3f velocity_perpendicular = velocity_NED - velocity_parallel;
            const float tension_force_centripetal = velocity_perpendicular.length_squared() * weight_kg / line_length;
            const Vector3f tension_force_centripetal_NED = payload_to_veh_norm * tension_force_centripetal;

            // add centripetal force to tension force
            tension_force_NED += tension_force_centripetal_NED;
        }
    }

    // force on vehicle is opposite to tension force on payload
    veh_forces_ef = -tension_force_NED;

    // convert force to acceleration (f=m*a => a=f/m)
    accel_NED = (force_gravity_NED + force_drag_NED + tension_force_NED) / weight_kg;

    // if slung payload is landed we zero out downward (e.g positive) acceleration
    if (landed) {
        accel_NED.z = MIN(accel_NED.z, 0);
        // should probably zero out forces_ef vertical component as well?
    }
}

#endif
