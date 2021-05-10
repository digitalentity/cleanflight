/*
 * This file is part of INAV Project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Alternatively, the contents of this file may be used under the terms
 * of the GNU General Public License Version 3, as described below:
 *
 * This file is free software: you may copy, redistribute and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/.
 */

#include "platform.h"

FILE_COMPILE_FOR_SIZE

#ifdef USE_NAV

#include "build/debug.h"
#include "common/utils.h"
#include "fc/rc_controls.h"
#include "fc/config.h"
#include "flight/mixer.h"
#include "navigation/navigation.h"
#include "navigation/navigation_private.h"

static bool isYawAdjustmentValid = false;
static int32_t navHeadingError;

static void update2DPositionHeadingController(timeUs_t currentTimeUs, timeDelta_t deltaMicros)
{
    static timeUs_t previousTimeMonitoringUpdate;
    static int32_t previousHeadingError;
    static bool errorIsDecreasing;

    int32_t targetBearing = calculateBearingToDestination(&posControl.desiredState.pos);

    /*
     * Calculate NAV heading error
     * Units are centidegrees
     */
    navHeadingError = wrap_18000(targetBearing - posControl.actualState.yaw);

    // Slow error monitoring (2Hz rate)
    if ((currentTimeUs - previousTimeMonitoringUpdate) >= HZ2US(NAV_FW_CONTROL_MONITORING_RATE)) {
        // Check if error is decreasing over time
        errorIsDecreasing = (ABS(previousHeadingError) > ABS(navHeadingError));

        // Save values for next iteration
        previousHeadingError = navHeadingError;
        previousTimeMonitoringUpdate = currentTimeUs;
    }

    posControl.rcAdjustment[YAW] = processHeadingYawController(deltaMicros, navHeadingError, errorIsDecreasing);
}

void applyRoverBoatPositionController(timeUs_t currentTimeUs)
{
    static timeUs_t previousTimePositionUpdate;         // Occurs @ GPS update rate
    static timeUs_t previousTimeUpdate;                 // Occurs @ looptime rate

    const timeDeltaLarge_t deltaMicros = currentTimeUs - previousTimeUpdate;
    previousTimeUpdate = currentTimeUs;

    // If last position update was too long in the past - ignore it (likely restarting altitude controller)
    if (deltaMicros > MAX_POSITION_UPDATE_INTERVAL_US) {
        previousTimeUpdate = currentTimeUs;
        previousTimePositionUpdate = currentTimeUs;
        resetFixedWingPositionController();
        return;
    }

    // Apply controller only if position source is valid. In absence of valid pos sensor (GPS loss), we'd stick in forced ANGLE mode
    if ((posControl.flags.estPosStatus >= EST_USABLE)) {
        // If we have new position - update velocity and acceleration controllers
        if (posControl.flags.horizontalPositionDataNew) {
            const timeDeltaLarge_t deltaMicrosPositionUpdate = currentTimeUs - previousTimePositionUpdate;
            previousTimePositionUpdate = currentTimeUs;

            if (deltaMicrosPositionUpdate < MAX_POSITION_UPDATE_INTERVAL_US) {
                update2DPositionHeadingController(currentTimeUs, deltaMicrosPositionUpdate);
            } else {
                resetFixedWingPositionController();
            }

            // Indicate that information is no longer usable
            posControl.flags.horizontalPositionDataConsumed = 1;
        }

        isYawAdjustmentValid = true;
    }
    else {
        isYawAdjustmentValid = false;
    }
}

void applyRoverBoatPitchRollThrottleController(navigationFSMStateFlags_t navStateFlags, timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
    rcCommand[ROLL] = 0;
    rcCommand[PITCH] = 0;

    if (navStateFlags & NAV_CTL_POS) {

        if (navStateFlags & NAV_AUTO_WP_DONE) {
            /*
             * When WP mission is done, stop the motors
             */
            rcCommand[YAW] = 0;
            rcCommand[THROTTLE] = feature(FEATURE_REVERSIBLE_MOTORS) ? reversibleMotorsConfig()->neutral : motorConfig()->mincommand;
        } else {
            if (isYawAdjustmentValid) {
                rcCommand[YAW] = posControl.rcAdjustment[YAW];
            }

            rcCommand[THROTTLE] = constrain(navConfig()->fw.cruise_throttle, motorConfig()->mincommand, motorConfig()->maxthrottle);
        }
    }
}

void applyRoverBoatNavigationController(navigationFSMStateFlags_t navStateFlags, timeUs_t currentTimeUs)
{
    if (navStateFlags & NAV_CTL_EMERG) {
        rcCommand[ROLL] = 0;
        rcCommand[PITCH] = 0;
        rcCommand[YAW] = 0;
        rcCommand[THROTTLE] = failsafeConfig()->failsafe_throttle;
    } else if (navStateFlags & NAV_CTL_POS) {
        applyRoverBoatPositionController(currentTimeUs);
        applyRoverBoatPitchRollThrottleController(navStateFlags, currentTimeUs);
    }
}

#endif
