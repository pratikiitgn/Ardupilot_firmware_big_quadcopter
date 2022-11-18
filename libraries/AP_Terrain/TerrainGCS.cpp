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
  handle vehicle <-> GCS communications for terrain library
 */

#include "AP_Terrain.h"

#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <GCS_MAVLink/GCS.h>

#if AP_TERRAIN_AVAILABLE

#include <assert.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

/*
  request any missing 4x4 grids from a block, given a grid_cache
 */
bool AP_Terrain::request_missing(GCS_MAVLINK &link, struct grid_cache &gcache)
{
    if (!HAVE_PAYLOAD_SPACE(link.get_chan(), TERRAIN_REQUEST)) {
        return false;
    }

    struct grid_block &grid = gcache.grid;

    if (options.get() & uint16_t(Options::DisableDownload)) {
        return false;
    }

    if (grid.spacing != grid_spacing) {
        // an invalid grid
        return false;
    }

    // see if we are waiting for disk read
    if (gcache.state == GRID_CACHE_DISKWAIT) {
        // don't request data from the GCS till we know it's not on disk
        return false;
    }

    // see if it is fully populated
    if ((grid.bitmap & bitmap_mask) == bitmap_mask) {
        // it is fully populated, nothing to do
        return false;
    }

    /*
      ask the GCS to send a set of 4x4 grids
     */
    const mavlink_terrain_request_t packet {
        bitmap_mask & ~grid.bitmap,
        grid.lat,
        grid.lon,
        (uint16_t)grid_spacing
    };
    mavlink_msg_terrain_request_send_struct(link.get_chan(), &packet);
    last_request_time_ms[link.get_chan()] = AP_HAL::millis();

    return true;
}

/*
  request any missing 4x4 grids from a block
 */
bool AP_Terrain::request_missing(GCS_MAVLINK &link, const struct grid_info &info)
{
    // find the grid
    struct grid_cache &gcache = find_grid_cache(info);
    return request_missing(link, gcache);
}

/*
  send any pending cache requests
 */
bool AP_Terrain::send_cache_request(GCS_MAVLINK &link)
{
    for (uint16_t i=0; i<cache_size; i++) {
        if (cache[i].state >= GRID_CACHE_VALID) {
            if (request_missing(link, cache[i])) {
                return true;
            }
        }
    }
    return false;
}

/*
  send any pending terrain request to the GCS
 */
void AP_Terrain::send_request(GCS_MAVLINK &link)
{
    if (!allocate()) {
        // not enabled
        return;
    }

    // see if we need to schedule some disk IO
    schedule_disk_io();

    Location loc;
    if (!AP::ahrs().get_location(loc)) {
        // we don't know where we are. Send a report and request any cached blocks.
        // this allows for download of mission items when we have no GPS lock
        loc = {};
        send_terrain_report(link, loc, true);
        send_cache_request(link);
        return;
    }

    // always send a terrain report
    send_terrain_report(link, loc, true);

    // did we request recently?
    if (AP_HAL::millis() - last_request_time_ms[link.get_chan()] < 2000) {
        // too soon to request again
        return;
    }

    // request any missing 4x4 blocks in the current grid
    struct grid_info info;
    calculate_grid_info(loc, info);

    if (request_missing(link, info)) {
        return;
    }

    // check cache blocks that may have been setup by a TERRAIN_CHECK,
    // mission items, rally items, squares surrounding our current
    // location, favourite holiday destination, scripting, height
    // reference location, ....
    if (send_cache_request(link)) {
        return;
    }

    // request the current loc last to ensure it has highest last
    // access time
    if (request_missing(link, info)) {
        return;
    }
}

/*
  count bits in a uint64_t
*/
uint8_t AP_Terrain::bitcount64(uint64_t b) const
{
    return __builtin_popcount((unsigned)(b&0xFFFFFFFF)) + __builtin_popcount((unsigned)(b>>32));
}

/*
  get some statistics for TERRAIN_REPORT
*/
void AP_Terrain::get_statistics(uint16_t &pending, uint16_t &loaded) const
{
    pending = 0;
    loaded = 0;
    for (uint16_t i=0; i<cache_size; i++) {
        if (cache[i].grid.spacing != grid_spacing) {
            continue;
        }
        if (cache[i].state == GRID_CACHE_INVALID) {
            continue;
        }
        uint8_t maskbits = TERRAIN_GRID_BLOCK_MUL_X*TERRAIN_GRID_BLOCK_MUL_Y;
        if (cache[i].state == GRID_CACHE_DISKWAIT) {
            pending += maskbits;
            continue;
        }
        if (cache[i].state == GRID_CACHE_DIRTY) {
            // count dirty grids as a pending, so we know where there 
            // are disk writes pending
            pending++;
        }
        uint8_t bitcount = bitcount64(cache[i].grid.bitmap);
        pending += maskbits - bitcount;
        loaded += bitcount;
    }
}


/* 
   handle terrain messages from GCS
 */
void AP_Terrain::handle_message(GCS_MAVLINK &link, const mavlink_message_t &msg)
{
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_TERRAIN_DATA:
        return handle_terrain_data(msg);
    case MAVLINK_MSG_ID_TERRAIN_CHECK:
        return handle_terrain_check(link, msg);
    // default:
        // shouldn't have been called
    }
}


/* 
   send a TERRAIN_REPORT for a location
 */
void AP_Terrain::send_terrain_report(GCS_MAVLINK &link, const Location &loc, bool extrapolate)
{
    if (!HAVE_PAYLOAD_SPACE(link.get_chan(), TERRAIN_REPORT)) {
        return;
    }

    float terrain_height = 0;
    float home_terrain_height = 0;
    uint16_t spacing = 0;
    Location current_loc;
    const AP_AHRS &ahrs = AP::ahrs();
    if (ahrs.get_location(current_loc) &&
        height_amsl(ahrs.get_home(), home_terrain_height) &&
        height_amsl(loc, terrain_height)) {
        // non-zero spacing indicates we have data
        spacing = grid_spacing;
    } else if (extrapolate && have_current_loc_height) {
        // show the extrapolated height, so logs show what height is
        // being used for navigation
        terrain_height = last_current_loc_height;
    } else {
        // report terrain height if we can, but can't give current_height
        height_amsl(loc, terrain_height);
    }
    uint16_t pending, loaded;
    get_statistics(pending, loaded);

    float current_height = 0.0f;
    if (spacing == 0 && !(extrapolate && have_current_loc_height)) {
        current_height = 0;
    } else if (!current_loc.is_zero()) {
        int32_t height_above_home_cm = 0;
        UNUSED_RESULT(current_loc.get_alt_cm(Location::AltFrame::ABOVE_HOME, height_above_home_cm));
        current_height = height_above_home_cm * 0.01f;  // cm -> m
    }
    current_height += home_terrain_height - terrain_height;

    const mavlink_terrain_report_t packet {
        loc.lat,
        loc.lng,
        terrain_height,
        current_height,
        spacing,
        pending,
        loaded
    };

    mavlink_msg_terrain_report_send_struct(link.get_chan(), &packet);
}

/* 
   handle TERRAIN_CHECK messages from GCS
 */
void AP_Terrain::handle_terrain_check(GCS_MAVLINK &link, const mavlink_message_t &msg)
{
    mavlink_terrain_check_t packet;
    mavlink_msg_terrain_check_decode(&msg, &packet);
    Location loc;
    loc.lat = packet.lat;
    loc.lng = packet.lon;
    send_terrain_report(link, loc, false);
}

/* 
   handle TERRAIN_DATA messages from GCS
 */
void AP_Terrain::handle_terrain_data(const mavlink_message_t &msg)
{
    mavlink_terrain_data_t packet;
    mavlink_msg_terrain_data_decode(&msg, &packet);

    uint16_t i;
    for (i=0; i<cache_size; i++) {
        if (TERRAIN_LATLON_EQUAL(cache[i].grid.lat,packet.lat) &&
            TERRAIN_LATLON_EQUAL(cache[i].grid.lon,packet.lon) &&
            cache[i].grid.spacing == packet.grid_spacing &&
            grid_spacing == packet.grid_spacing &&
            packet.gridbit < 56) {
            break;
        }
    }
    if (i == cache_size) {
        // we don't have that grid, ignore data
        return;
    }
    struct grid_cache &gcache = cache[i];
    struct grid_block &grid = gcache.grid;
    uint8_t idx_x = (packet.gridbit / TERRAIN_GRID_BLOCK_MUL_Y) * TERRAIN_GRID_MAVLINK_SIZE;
    uint8_t idx_y = (packet.gridbit % TERRAIN_GRID_BLOCK_MUL_Y) * TERRAIN_GRID_MAVLINK_SIZE;
    ASSERT_RANGE(idx_x,0,(TERRAIN_GRID_BLOCK_MUL_X-1)*TERRAIN_GRID_MAVLINK_SIZE);
    ASSERT_RANGE(idx_y,0,(TERRAIN_GRID_BLOCK_MUL_Y-1)*TERRAIN_GRID_MAVLINK_SIZE);
    for (uint8_t x=0; x<TERRAIN_GRID_MAVLINK_SIZE; x++) {
        for (uint8_t y=0; y<TERRAIN_GRID_MAVLINK_SIZE; y++) {
            grid.height[idx_x+x][idx_y+y] = packet.data[x*TERRAIN_GRID_MAVLINK_SIZE+y];
        }
    }
    gcache.grid.bitmap |= ((uint64_t)1) << packet.gridbit;
    
    // mark dirty for disk IO
    gcache.state = GRID_CACHE_DIRTY;
    
#if TERRAIN_DEBUG
    hal.console->printf("Filled bit %u idx_x=%u idx_y=%u\n", 
                        (unsigned)packet.gridbit, (unsigned)idx_x, (unsigned)idx_y);
    if (gcache.grid.bitmap == bitmap_mask) {
        hal.console->printf("--lat=%12.7f --lon=%12.7f %u\n",
                            grid.lat*1.0e-7f,
                            grid.lon*1.0e-7f,
                            grid.height[0][0]);
        Location loc2;
        loc2.lat = grid.lat;
        loc2.lng = grid.lon;
        loc2.offset(28*grid_spacing, 32*grid_spacing);
        hal.console->printf("--lat=%12.7f --lon=%12.7f %u\n",
                            loc2.lat*1.0e-7f,
                            loc2.lng*1.0e-7f,
                            grid.height[27][31]);            
    }
#endif
    
    // see if we need to schedule some disk IO
    update();
}


#endif // AP_TERRAIN_AVAILABLE
