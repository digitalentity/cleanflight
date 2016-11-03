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
#include <math.h>
#include <string.h>

#include "platform.h"

#if defined(NAV)

#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"

#include "drivers/system.h"
#include "drivers/sensor.h"
#include "drivers/accgyro.h"

#include "sensors/sensors.h"
#include "sensors/rangefinder.h"
#include "sensors/barometer.h"
#include "sensors/acceleration.h"
#include "sensors/boardalignment.h"
#include "sensors/compass.h"

#include "io/gps.h"
#include "io/beeper.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/navigation_rewrite.h"
#include "flight/navigation_rewrite_private.h"

#include "fc/runtime_config.h"
#include "config/config.h"
#include "config/feature.h"


typedef struct FixedWingLaunchState_s {
    /* Launch detection */
    uint32_t launchDetectorPreviosUpdate;
    uint32_t launchDetectionTimeAccum;
    bool launchDetected;

    /* Launch progress */
    uint32_t launchStartedTime;
    bool launchFinished;
    bool motorControlAllowed;
} FixedWingLaunchState_t;

static FixedWingLaunchState_t   launchState;

#define COS_MAX_LAUNCH_ANGLE    0.70710678f         // cos(45), just to be safe
static void updateFixedWingLaunchDetector(const uint32_t currentTime)
{
    const bool isForwardAccelerationHigh = (imuAccelInBodyFrame.A[X] > posControl.navConfig->fw.launch_accel_thresh);
    const bool isAircraftAlmostLevel = (calculateCosTiltAngle() >= COS_MAX_LAUNCH_ANGLE);

    if (isForwardAccelerationHigh && isAircraftAlmostLevel) {
        launchState.launchDetectionTimeAccum += (currentTime - launchState.launchDetectorPreviosUpdate);
        launchState.launchDetectorPreviosUpdate = currentTime;
        if (launchState.launchDetectionTimeAccum >= MS2US((uint32_t)posControl.navConfig->fw.launch_time_thresh)) {
            launchState.launchDetected = true;
        }
    }
    else {
        launchState.launchDetectorPreviosUpdate = currentTime;
        launchState.launchDetectionTimeAccum = 0;
    }
}

void resetFixedWingLaunchController(const uint32_t currentTime)
{
    launchState.launchDetectorPreviosUpdate = currentTime;
    launchState.launchDetectionTimeAccum = 0;
    launchState.launchStartedTime = 0;
    launchState.launchDetected = false;
    launchState.launchFinished = false;
    launchState.motorControlAllowed = false;
}

bool isFixedWingLaunchDetected(void)
{
    return launchState.launchDetected;
}

void enableFixedWingLaunchController(const uint32_t currentTime)
{
    launchState.launchStartedTime = currentTime;
    launchState.motorControlAllowed = true;
}

bool isFixedWingLaunchFinishedOrAborted(void)
{
    return launchState.launchFinished;
}

void applyFixedWingLaunchController(const uint32_t currentTime)
{
    // Called at PID rate

    if (launchState.launchDetected) {
        // If launch detected we are in launch procedure - control airplane
        const float timeElapsedSinceLaunchMs = US2MS(currentTime - launchState.launchStartedTime);

        // If user moves the stick - finish the launch
        if ((ABS(rcCommand[ROLL]) > posControl.rcControlsConfig->pos_hold_deadband) || (ABS(rcCommand[PITCH]) > posControl.rcControlsConfig->pos_hold_deadband)) {
            launchState.launchFinished = true;
        }

        // Motor control enabled
        if (launchState.motorControlAllowed) {
            // Abort launch after a pre-set time
            if (timeElapsedSinceLaunchMs >= posControl.navConfig->fw.launch_timeout) {
                launchState.launchFinished = true;
            }

            // Control throttle
            if (timeElapsedSinceLaunchMs >= posControl.navConfig->fw.launch_motor_timer) {
                rcCommand[THROTTLE] = posControl.navConfig->fw.launch_throttle;
            }
            else {
                // Until motors are started don't use PID I-term
                pidResetErrorAccumulators();

                // Throttle control logic
                ENABLE_STATE(NAV_MOTOR_STOP_OR_IDLE);                       // If MOTOR_STOP is enabled mixer will keep motor stopped
                rcCommand[THROTTLE] = posControl.motorConfig->minthrottle;  // If MOTOR_STOP is disabled, motors will spin at minthrottle
            }
        }
    }
    else {
        // We are waiting for launch - update launch detector
        updateFixedWingLaunchDetector(currentTime);

        // Until motors are started don't use PID I-term
        pidResetErrorAccumulators();

        // Throttle control logic
        ENABLE_STATE(NAV_MOTOR_STOP_OR_IDLE);                       // If MOTOR_STOP is enabled mixer will keep motor stopped
        rcCommand[THROTTLE] = posControl.motorConfig->minthrottle;  // If MOTOR_STOP is disabled, motors will spin at minthrottle
    }

    // Control beeper
    if (!launchState.launchFinished) {
        beeper(BEEPER_LAUNCH_MODE_ENABLED);
    }

    // Lock out controls
    rcCommand[ROLL] = 0;
    rcCommand[PITCH] = pidAngleToRcCommand(-DEGREES_TO_DECIDEGREES(posControl.navConfig->fw.launch_climb_angle), posControl.pidProfile->max_angle_inclination[FD_PITCH]);
    rcCommand[YAW] = 0;
}

#endif
