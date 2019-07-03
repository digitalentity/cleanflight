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

#if defined(USE_NAV)

#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"

#include "config/feature.h"

#include "drivers/time.h"
#include "drivers/sensor.h"
#include "drivers/accgyro/accgyro.h"

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
#include "flight/mixer.h"

#include "fc/config.h"
#include "fc/rc_control.h"
#include "fc/runtime_config.h"

#include "navigation/navigation.h"
#include "navigation/navigation_private.h"


typedef struct FixedWingLaunchState_s {
    /* Launch detection */
    timeUs_t launchDetectorPreviousUpdate;
    timeUs_t launchDetectionTimeAccum;
    bool launchDetected;

    /* Launch progress */
    timeUs_t launchStartedTime;
    bool launchFinished;
    bool motorControlAllowed;
} FixedWingLaunchState_t;

static EXTENDED_FASTRAM FixedWingLaunchState_t   launchState;

#define COS_MAX_LAUNCH_ANGLE                0.70710678f                 // cos(45), just to be safe
#define SWING_LAUNCH_MIN_ROTATION_RATE      DEGREES_TO_RADIANS(100)     // expect minimum 100dps rotation rate
static void updateFixedWingLaunchDetector(timeUs_t currentTimeUs)
{
    const float swingVelocity = (fabsf(imuMeasuredRotationBF.z) > SWING_LAUNCH_MIN_ROTATION_RATE) ? (imuMeasuredAccelBF.y / imuMeasuredRotationBF.z) : 0;
    const bool isForwardAccelerationHigh = (imuMeasuredAccelBF.x > navConfig()->fw.launch_accel_thresh);
    const bool isAircraftAlmostLevel = (calculateCosTiltAngle() >= cos_approx(DEGREES_TO_RADIANS(navConfig()->fw.launch_max_angle)));

    const bool isBungeeLaunched = isForwardAccelerationHigh && isAircraftAlmostLevel;
    const bool isSwingLaunched = (swingVelocity > navConfig()->fw.launch_velocity_thresh) && (imuMeasuredAccelBF.x > 0);

    if (isBungeeLaunched || isSwingLaunched) {
        launchState.launchDetectionTimeAccum += (currentTimeUs - launchState.launchDetectorPreviousUpdate);
        launchState.launchDetectorPreviousUpdate = currentTimeUs;
        if (launchState.launchDetectionTimeAccum >= MS2US((uint32_t)navConfig()->fw.launch_time_thresh)) {
            launchState.launchDetected = true;
        }
    }
    else {
        launchState.launchDetectorPreviousUpdate = currentTimeUs;
        launchState.launchDetectionTimeAccum = 0;
    }
}

void resetFixedWingLaunchController(timeUs_t currentTimeUs)
{
    launchState.launchDetectorPreviousUpdate = currentTimeUs;
    launchState.launchDetectionTimeAccum = 0;
    launchState.launchStartedTime = 0;
    launchState.launchDetected = false;
    launchState.launchFinished = false;
    launchState.motorControlAllowed = false;
}

bool FAST_CODE isFixedWingLaunchDetected(void)
{
    return launchState.launchDetected;
}

void enableFixedWingLaunchController(timeUs_t currentTimeUs)
{
    launchState.launchStartedTime = currentTimeUs;
    launchState.motorControlAllowed = true;
}

bool FAST_CODE isFixedWingLaunchFinishedOrAborted(void)
{
    return launchState.launchFinished;
}

void abortFixedWingLaunch(void)
{
    launchState.launchFinished = true;
}

#define LAUNCH_MOTOR_IDLE_SPINUP_TIME 1500 //ms

static void applyFixedWingLaunchIdleLogic(rcCommand_t *controlOutput)
{
    // Until motors are started don't use PID I-term
    pidResetErrorAccumulators();

    // We're not flying yet, reset TPA filter
    pidResetTPAFilter();

    // Throttle control logic
    if (navConfig()->fw.launch_idle_throttle <= motorConfig()->minthrottle)
    {
        ENABLE_STATE(NAV_MOTOR_STOP_OR_IDLE);           // If MOTOR_STOP is enabled mixer will keep motor stopped
        controlOutput->throttle = RC_COMMAND_CENTER;    // If MOTOR_STOP is disabled, motors will spin at minthrottle
    }
    else
    {
        static float timeThrottleRaisedMs;
        if (calculateThrottleStatus() == THROTTLE_LOW)
        {
            timeThrottleRaisedMs = millis();
        }
        else
        {
            const float timeSinceMotorStartMs = MIN(millis() - timeThrottleRaisedMs, LAUNCH_MOTOR_IDLE_SPINUP_TIME);
            float thr = scaleRangef(timeSinceMotorStartMs,
                                                0.0f, LAUNCH_MOTOR_IDLE_SPINUP_TIME,
                                                motorConfig()->minthrottle, navConfig()->fw.launch_idle_throttle);
            controlOutput->throttle = rcCommandMapUnidirectionalPWMThrottle(thr);
        }
    }
}

static inline bool isFixedWingLaunchMaxAltitudeReached(void)
{
    return (navConfig()->fw.launch_max_altitude > 0) && (getEstimatedActualPosition(Z) >= navConfig()->fw.launch_max_altitude);
}
static inline bool isLaunchModeMinTimeElapsed(float timeSinceLaunchMs)
{
    return timeSinceLaunchMs > navConfig()->fw.launch_min_time;
}

static inline bool isLaunchModeMaxTimeElapsed(float timeSinceLaunchMs)
{
    return timeSinceLaunchMs >= navConfig()->fw.launch_timeout;
}

static inline bool isFixedWingLaunchCompleted(float timeSinceLaunchMs)
{
    return (isLaunchModeMaxTimeElapsed(timeSinceLaunchMs)) || ((isLaunchModeMinTimeElapsed(timeSinceLaunchMs)) && (areSticksDeflectedMoreThanPosHoldDeadband())) || isFixedWingLaunchMaxAltitudeReached();
}

void applyFixedWingLaunchController(timeUs_t currentTimeUs)
{
    // Called at PID rate

    rcCommand_t controlOutput;
    rcCommandCopy(&controlOutput, rcControlGetOutput());

    if (launchState.launchDetected) {
        bool applyLaunchIdleLogic = true;

        if (launchState.motorControlAllowed) {
            // If launch detected we are in launch procedure - control airplane
            const float timeElapsedSinceLaunchMs = US2MS(currentTimeUs - launchState.launchStartedTime);

            launchState.launchFinished = isFixedWingLaunchCompleted(timeElapsedSinceLaunchMs);

            // Motor control enabled
            if (timeElapsedSinceLaunchMs >= navConfig()->fw.launch_motor_timer) {
                // Don't apply idle logic anymore
                applyLaunchIdleLogic = false;

                // Increase throttle gradually over `launch_motor_spinup_time` milliseconds
                float thr;
                if (navConfig()->fw.launch_motor_spinup_time > 0) {
                    const float timeSinceMotorStartMs = constrainf(timeElapsedSinceLaunchMs - navConfig()->fw.launch_motor_timer, 0.0f, navConfig()->fw.launch_motor_spinup_time);
                    const uint16_t minIdleThrottle = MAX(motorConfig()->minthrottle, navConfig()->fw.launch_idle_throttle);
                    thr = scaleRangef(timeSinceMotorStartMs,
                                                      0.0f, navConfig()->fw.launch_motor_spinup_time,
                                                      minIdleThrottle, navConfig()->fw.launch_throttle);
                }
                else {
                    thr = navConfig()->fw.launch_throttle;
                }
                controlOutput.throttle = rcCommandMapUnidirectionalPWMThrottle(thr);
            }
        }

        if (applyLaunchIdleLogic) {
            // Launch idle logic - low throttle and zero out PIDs
            applyFixedWingLaunchIdleLogic(&controlOutput);
        }
    }
    else {
        // We are waiting for launch - update launch detector
        updateFixedWingLaunchDetector(currentTimeUs);

        // Launch idle logic - low throttle and zero out PIDs
        applyFixedWingLaunchIdleLogic(&controlOutput);
    }

    // Control beeper
    if (!launchState.launchFinished) {
        beeper(BEEPER_LAUNCH_MODE_ENABLED);
    }

    // Lock out controls
    controlOutput.roll = RC_COMMAND_CENTER;
    controlOutput.pitch = pidAngleToRcCommand(-DEGREES_TO_DECIDEGREES(navConfig()->fw.launch_climb_angle), pidProfile()->max_angle_inclination[FD_PITCH]);
    controlOutput.yaw = RC_COMMAND_CENTER;
    rcControlUpdateOutput(&controlOutput, RC_CONTROL_SOURCE_NAVIGATION);
}

#endif
