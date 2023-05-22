// TransmuterProtocol.c was generated by ProtoGen version 3.5.c

/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Oliver Walters / Currawong Engineering Pty Ltd
 */

#include "TransmuterProtocol.h"

/*!
 * \brief Lookup label for 'TransmuterModes' enum entry
 * 
 * \param value is the integer value of the enum entry
 * \return string label of the given entry
 */
const char* TransmuterModes_EnumLabel(int value)
{
    switch (value)
    {
    default:
        return "";
    case TRANSMUTER_MODE_STARTUP:
        return "TRANSMUTER_MODE_STARTUP";
    case TRANSMUTER_MODE_STANDBY:
        return "TRANSMUTER_MODE_STANDBY";
    case TRANSMUTER_MODE_PREFLIGHT:
        return "TRANSMUTER_MODE_PREFLIGHT";
    case TRANSMUTER_MODE_STARTING:
        return "TRANSMUTER_MODE_STARTING";
    case TRANSMUTER_MODE_WARMUP:
        return "TRANSMUTER_MODE_WARMUP";
    case TRANSMUTER_MODE_RPM:
        return "TRANSMUTER_MODE_RPM";
    case TRANSMUTER_MODE_CURRENT:
        return "TRANSMUTER_MODE_CURRENT";
    case TRANSMUTER_MODE_PWM:
        return "TRANSMUTER_MODE_PWM";
    case TRANSMUTER_MODE_NUM_MODES:
        return "TRANSMUTER_MODE_NUM_MODES";
    }
}

/*!
 * \brief Lookup title for 'TransmuterModes' enum entry
 * 
 * \param value is the integer value of the enum entry
 * \return string title of the given entry (comment if no title given)
 */
const char* TransmuterModes_EnumTitle(int value)
{
    switch (value)
    {
    default:
        return "";
    case TRANSMUTER_MODE_STARTUP:
        return "Initial powerup mode of the transmuter";
    case TRANSMUTER_MODE_STANDBY:
        return "Transmuter is in standby mode";
    case TRANSMUTER_MODE_PREFLIGHT:
        return "Transmuter is in preflight check mode";
    case TRANSMUTER_MODE_STARTING:
        return "Transmuter is attempting to start the engine";
    case TRANSMUTER_MODE_WARMUP:
        return "Transmuter is in warmup mode";
    case TRANSMUTER_MODE_RPM:
        return "Transmuter is running in RPM control mode";
    case TRANSMUTER_MODE_CURRENT:
        return "Transmuter is running in CURRENT control mode";
    case TRANSMUTER_MODE_PWM:
        return "Transmuter is running in PWM control mode";
    case TRANSMUTER_MODE_NUM_MODES:
        return "Number of transmuter operational modes";
    }
}


/*!
 * \brief Lookup label for 'TransmuterStartingStates' enum entry
 * 
 * \param value is the integer value of the enum entry
 * \return string label of the given entry
 */
const char* TransmuterStartingStates_EnumLabel(int value)
{
    switch (value)
    {
    default:
        return "";
    case TRANSMUTER_START_INIT:
        return "TRANSMUTER_START_INIT";
    case TRANSMUTER_START_PRIMING:
        return "TRANSMUTER_START_PRIMING";
    case TRANSMUTER_START_DISABLE_ECU:
        return "TRANSMUTER_START_DISABLE_ECU";
    case TRANSMUTER_START_CRANK_GEN:
        return "TRANSMUTER_START_CRANK_GEN";
    case TRANSMUTER_START_ENABLE_ECU:
        return "TRANSMUTER_START_ENABLE_ECU";
    case TRANSMUTER_START_FAILURE:
        return "TRANSMUTER_START_FAILURE";
    }
}

/*!
 * \brief Lookup title for 'TransmuterStartingStates' enum entry
 * 
 * \param value is the integer value of the enum entry
 * \return string title of the given entry (comment if no title given)
 */
const char* TransmuterStartingStates_EnumTitle(int value)
{
    switch (value)
    {
    default:
        return "";
    case TRANSMUTER_START_INIT:
        return "Initializing the starting routine";
    case TRANSMUTER_START_PRIMING:
        return "Priming engine / fuel pump / etc";
    case TRANSMUTER_START_DISABLE_ECU:
        return "Disabling ECU prior to engine cranking";
    case TRANSMUTER_START_CRANK_GEN:
        return "Spinning the generator";
    case TRANSMUTER_START_ENABLE_ECU:
        return "Enable ECU once the system is spinning";
    case TRANSMUTER_START_FAILURE:
        return "Starting failed";
    }
}


/*!
 * \brief Lookup label for 'TransmuterStandbyCause' enum entry
 * 
 * \param value is the integer value of the enum entry
 * \return string label of the given entry
 */
const char* TransmuterStandbyCause_EnumLabel(int value)
{
    switch (value)
    {
    default:
        return "";
    case TRANSMUTER_STANDBY_CAUSE_POWERUP:
        return "TRANSMUTER_STANDBY_CAUSE_POWERUP";
    case TRANSMUTER_STANDBY_CAUSE_HW_DISABLE:
        return "TRANSMUTER_STANDBY_CAUSE_HW_DISABLE";
    case TRANSMUTER_STANDBY_CAUSE_SW_DISABLE:
        return "TRANSMUTER_STANDBY_CAUSE_SW_DISABLE";
    case TRANSMUTER_STANDBY_CAUSE_STARTING_RETRIES:
        return "TRANSMUTER_STANDBY_CAUSE_STARTING_RETRIES";
    case TRANSMUTER_STANDBY_CAUSE_CMD:
        return "TRANSMUTER_STANDBY_CAUSE_CMD";
    case TRANSMUTER_STANDBY_CAUSE_CRITICAL_RUNNING_ERROR:
        return "TRANSMUTER_STANDBY_CAUSE_CRITICAL_RUNNING_ERROR";
    case TRANSMUTER_STANDBY_CAUSE_UNKNOWN_MODE:
        return "TRANSMUTER_STANDBY_CAUSE_UNKNOWN_MODE";
    }
}


/*!
 * \brief Lookup label for 'TransmuterFuelPumpMode' enum entry
 * 
 * \param value is the integer value of the enum entry
 * \return string label of the given entry
 */
const char* TransmuterFuelPumpMode_EnumLabel(int value)
{
    switch (value)
    {
    default:
        return "";
    case TRANSMUTER_PUMP_TRIPLEX:
        return "TRANSMUTER_PUMP_TRIPLEX";
    case TRANSMUTER_PUMP_ECU:
        return "TRANSMUTER_PUMP_ECU";
    }
}


/*!
 * \brief Lookup label for 'TransmuterPackets' enum entry
 * 
 * \param value is the integer value of the enum entry
 * \return string label of the given entry
 */
const char* TransmuterPackets_EnumLabel(int value)
{
    switch (value)
    {
    default:
        return "";
    case PKT_TRANSMUTER_STANDBY:
        return "PKT_TRANSMUTER_STANDBY";
    case PKT_TRANSMUTER_PREFLIGHT:
        return "PKT_TRANSMUTER_PREFLIGHT";
    case PKT_TRANSMUTER_WARMUP:
        return "PKT_TRANSMUTER_WARMUP";
    case PKT_TRANSMUTER_RUN:
        return "PKT_TRANSMUTER_RUN";
    case PKT_TRANSMUTER_PWM:
        return "PKT_TRANSMUTER_PWM";
    case PKT_TRANSMUTER_SYSTEM_CMD:
        return "PKT_TRANSMUTER_SYSTEM_CMD";
    case PKT_TRANSMUTER_TELEMETRY_STATUS:
        return "PKT_TRANSMUTER_TELEMETRY_STATUS";
    case PKT_TRANSMUTER_TELEMETRY_POWER:
        return "PKT_TRANSMUTER_TELEMETRY_POWER";
    case PKT_TRANSMUTER_TELEMETRY_SETPOINT:
        return "PKT_TRANSMUTER_TELEMETRY_SETPOINT";
    case PKT_TRANSMUTER_TELEMETRY_GEN:
        return "PKT_TRANSMUTER_TELEMETRY_GEN";
    case PKT_TRANSMUTER_TELEMETRY_CAPACITY:
        return "PKT_TRANSMUTER_TELEMETRY_CAPACITY";
    case PKT_TRANSMUTER_TELEMETRY_CTRL_LOOP:
        return "PKT_TRANSMUTER_TELEMETRY_CTRL_LOOP";
    case PKT_TRANSMUTER_TELEMETRY_APB:
        return "PKT_TRANSMUTER_TELEMETRY_APB";
    case PKT_TRANSMUTER_TELEMETRY_CONFIG:
        return "PKT_TRANSMUTER_TELEMETRY_CONFIG";
    case PKT_TRANSMUTER_WARNING_LEVELS:
        return "PKT_TRANSMUTER_WARNING_LEVELS";
    case PKT_TRANSMUTER_CTRL_LOOP_SETTINGS:
        return "PKT_TRANSMUTER_CTRL_LOOP_SETTINGS";
    case PKT_TRANSMUTER_BATTERY_SETTINGS:
        return "PKT_TRANSMUTER_BATTERY_SETTINGS";
    case PKT_TRANSMUTER_EFI_CONFIG:
        return "PKT_TRANSMUTER_EFI_CONFIG";
    case PKT_TRANSMUTER_GEN_CONFIG:
        return "PKT_TRANSMUTER_GEN_CONFIG";
    case PKT_TRANSMUTER_POWER_MAP:
        return "PKT_TRANSMUTER_POWER_MAP";
    case PKT_TRANSMUTER_STARTING_SETTINGS:
        return "PKT_TRANSMUTER_STARTING_SETTINGS";
    case PKT_TRANSMUTER_BULK_TRANSFER:
        return "PKT_TRANSMUTER_BULK_TRANSFER";
    }
}

// end of TransmuterProtocol.c
