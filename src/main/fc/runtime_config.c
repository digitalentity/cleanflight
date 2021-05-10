/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "common/utils.h"
#include "fc/runtime_config.h"

#include "io/beeper.h"

EXTENDED_FASTRAM uint32_t armingFlags = 0;
EXTENDED_FASTRAM uint32_t stateFlags = 0;
EXTENDED_FASTRAM uint32_t flightModeFlags = 0;

static EXTENDED_FASTRAM uint32_t enabledSensors = 0;

#if !defined(CLI_MINIMAL_VERBOSITY)
const char *armingDisableFlagNames[]= {
    "FS", "ANGLE", "CAL", "OVRLD", "NAV", "COMPASS",
    "ACC", "ARMSW", "HWFAIL", "BOXFS", "KILLSW", "RX",
    "THR", "CLI", "CMS", "OSD", "ROLL/PITCH", "AUTOTRIM", "OOM",
    "SETTINGFAIL", "PWMOUT", "NOPREARM", "DSHOTBEEPER"
};
#endif

const armingFlag_e armDisableReasonsChecklist[] = {
    ARMING_DISABLED_INVALID_SETTING,
    ARMING_DISABLED_HARDWARE_FAILURE,
    ARMING_DISABLED_PWM_OUTPUT_ERROR,
    ARMING_DISABLED_COMPASS_NOT_CALIBRATED,
    ARMING_DISABLED_ACCELEROMETER_NOT_CALIBRATED,
    ARMING_DISABLED_RC_LINK,
    ARMING_DISABLED_NAVIGATION_UNSAFE,
    ARMING_DISABLED_ARM_SWITCH,
    ARMING_DISABLED_BOXFAILSAFE,
    ARMING_DISABLED_BOXKILLSWITCH,
    ARMING_DISABLED_THROTTLE,
    ARMING_DISABLED_CLI,
    ARMING_DISABLED_CMS_MENU,
    ARMING_DISABLED_OSD_MENU,
    ARMING_DISABLED_ROLLPITCH_NOT_CENTERED,
    ARMING_DISABLED_SERVO_AUTOTRIM,
    ARMING_DISABLED_OOM,
    ARMING_DISABLED_NO_PREARM,
    ARMING_DISABLED_DSHOT_BEEPER
};

armingFlag_e isArmingDisabledReason(void)
{
    // Shortcut, if we don't block arming at all
    if (!isArmingDisabled()) {
        return 0;
    }

    armingFlag_e reasons = armingFlags & ARMING_DISABLED_ALL_FLAGS;

    // First check for "more important reasons"
    for (unsigned ii = 0; ii < ARRAYLEN(armDisableReasonsChecklist); ii++) {
        armingFlag_e flag = armDisableReasonsChecklist[ii];
        if (flag & reasons) {
            return flag;
        }
    }

    // Fallback, we ended up with a blocker flag not included in armDisableReasonsChecklist[]
    for (unsigned ii = 0; ii < sizeof(armingFlag_e) * 8; ii++) {
        armingFlag_e flag = 1u << ii;
        if (flag & reasons) {
            return flag;
        }
    }

    return 0;
}

/**
 * Enables the given flight mode.  A beep is sounded if the flight mode
 * has changed.  Returns the new 'flightModeFlags' value.
 */
uint32_t enableFlightMode(flightModeFlags_e mask)
{
    uint32_t oldVal = flightModeFlags;

    flightModeFlags |= (mask);
    if (flightModeFlags != oldVal)
        beeperConfirmationBeeps(1);
    return flightModeFlags;
}

/**
 * Disables the given flight mode.  A beep is sounded if the flight mode
 * has changed.  Returns the new 'flightModeFlags' value.
 */
uint32_t disableFlightMode(flightModeFlags_e mask)
{
    uint32_t oldVal = flightModeFlags;

    flightModeFlags &= ~(mask);
    if (flightModeFlags != oldVal)
        beeperConfirmationBeeps(1);
    return flightModeFlags;
}

bool sensors(uint32_t mask)
{
    return enabledSensors & mask;
}

void sensorsSet(uint32_t mask)
{
    enabledSensors |= mask;
}

void sensorsClear(uint32_t mask)
{
    enabledSensors &= ~(mask);
}

uint32_t sensorsMask(void)
{
    return enabledSensors;
}

flightModeForTelemetry_e getFlightModeForTelemetry(void)
{
    if (FLIGHT_MODE(FAILSAFE_MODE))
        return FLM_FAILSAFE;

    if (FLIGHT_MODE(MANUAL_MODE))
        return FLM_MANUAL;

    if (FLIGHT_MODE(NAV_LAUNCH_MODE))
        return FLM_LAUNCH;

    if (FLIGHT_MODE(NAV_RTH_MODE))
        return FLM_RTH;

    if (FLIGHT_MODE(NAV_POSHOLD_MODE))
        return FLM_POSITION_HOLD;

    if (FLIGHT_MODE(NAV_COURSE_HOLD_MODE) && FLIGHT_MODE(NAV_ALTHOLD_MODE))
        return FLM_CRUISE;

    if (FLIGHT_MODE(NAV_COURSE_HOLD_MODE))
        return FLM_COURSE_HOLD;

    if (FLIGHT_MODE(NAV_WP_MODE))
        return FLM_MISSION;

    if (FLIGHT_MODE(NAV_ALTHOLD_MODE))
        return FLM_ALTITUDE_HOLD;

    if (FLIGHT_MODE(ANGLE_MODE))
        return FLM_ANGLE;

    if (FLIGHT_MODE(HORIZON_MODE))
        return FLM_HORIZON;


    return STATE(AIRMODE_ACTIVE) ? FLM_ACRO_AIR : FLM_ACRO;
}
