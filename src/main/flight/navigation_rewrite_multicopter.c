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

#include "build_config.h"
#include "platform.h"
#include "debug.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"

#include "drivers/system.h"
#include "drivers/sensor.h"
#include "drivers/accgyro.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/boardalignment.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/navigation_rewrite.h"
#include "flight/navigation_rewrite_private.h"

#include "config/runtime_config.h"
#include "config/config.h"

#if defined(NAV)

#define HOVER_ACCZ_THRESHOLD    5.0f    // cm/s/s
#define HOVER_THR_FILTER        0.025f


/*-----------------------------------------------------------
 * Backdoor to MW heading controller
 *-----------------------------------------------------------*/
extern int16_t magHold;

/*-----------------------------------------------------------
 * Altitude controller for multicopter aircraft
 *-----------------------------------------------------------*/
static int16_t altholdInitialThrottle;      // Throttle input when althold was activated
static int16_t rcCommandAdjustedThrottle;

static void updateAltitudeTargetFromRCInput_MC(void)
{
    // In some cases pilot has no control over flight direction
    if (!navCanAdjustAltitudeFromRCInput()) {
        posControl.flags.isAdjustingAltitude = false;
        return;
    }

    int16_t rcThrottleAdjustment = applyDeadband(rcCommand[THROTTLE] - altholdInitialThrottle, posControl.navConfig->alt_hold_deadband);
    if (rcThrottleAdjustment) {
        // set velocity proportional to stick movement
        float rcClimbRate = rcThrottleAdjustment * posControl.navConfig->max_manual_climb_rate / 500;
        updateAltitudeTargetFromClimbRate(rcClimbRate);
        posControl.flags.isAdjustingAltitude = true;
    }
    else {
        // Adjusting finished - reset desired position to stay exactly where pilot released the stick
        if (posControl.flags.isAdjustingAltitude) {
            updateAltitudeTargetFromClimbRate(0);
        }
        posControl.flags.isAdjustingAltitude = false;
    }
}

/* Calculate global altitude setpoint based on surface setpoint */
static void updateSurfaceTrackingAltitudeSetpoint_MC(void)
{
    /* If we have a surface offset target and a valid surface offset reading - recalculate altitude target */
    if (posControl.desiredState.surface > 0 && (posControl.actualState.surface > 0 && posControl.flags.hasValidSurfaceSensor)) {
        float surfaceOffsetError = posControl.desiredState.surface - posControl.actualState.surface;
        posControl.desiredState.pos.V.Z = posControl.actualState.pos.V.Z + surfaceOffsetError;
    }
}

// Position to velocity controller for Z axis
static void updateAltitudeVelocityController_MC(void)
{
    float altitudeError = posControl.desiredState.pos.V.Z - posControl.actualState.pos.V.Z;
    posControl.desiredState.vel.V.Z = altitudeError * posControl.pids.pos[Z].param.kP;
    posControl.desiredState.vel.V.Z = constrainf(posControl.desiredState.vel.V.Z, -300.0f, 300.0f); // hard limit velocity to +/- 3 m/s

#if defined(NAV_BLACKBOX)
    navDesiredVelocity[Z] = constrain(lrintf(posControl.desiredState.vel.V.Z), -32678, 32767);
    navTargetPosition[Z] = constrain(lrintf(posControl.desiredState.pos.V.Z), -32678, 32767);
#endif
}

static void updateAltitudeThrottleController_MC(uint32_t deltaMicros)
{
    static filterStatePt1_t throttleFilterState;

    // Calculate min and max throttle boundaries (to compensate for integral windup)
    int32_t thrAdjustmentMin = posControl.escAndServoConfig->minthrottle - altholdInitialThrottle;
    int32_t thrAdjustmentMax = posControl.escAndServoConfig->maxthrottle - altholdInitialThrottle;

    posControl.rcAdjustment[THROTTLE] = navPidApply2(posControl.desiredState.vel.V.Z, posControl.actualState.vel.V.Z, US2S(deltaMicros), &posControl.pids.vel[Z], thrAdjustmentMin, thrAdjustmentMax);
    posControl.rcAdjustment[THROTTLE] = filterApplyPt1(posControl.rcAdjustment[THROTTLE], &throttleFilterState, NAV_THROTTLE_CUTOFF_FREQENCY_HZ, US2S(deltaMicros));
    posControl.rcAdjustment[THROTTLE] = constrain(posControl.rcAdjustment[THROTTLE], thrAdjustmentMin, thrAdjustmentMax);
}

void setupMulticopterAltitudeController(void)
{
    if (posControl.navConfig->flags.use_midrc_for_althold) {
        altholdInitialThrottle = posControl.rxConfig->midrc;
    }
    else {
        altholdInitialThrottle = rcCommand[THROTTLE];
    }
}

void resetMulticopterAltitudeController()
{
    navPidReset(&posControl.pids.vel[Z]);
    posControl.rcAdjustment[THROTTLE] = 0;
}

void applyMulticopterAltitudeController(uint32_t currentTime)
{
    static navigationTimer_t targetPositionUpdateTimer; // Occurs @ POSITION_TARGET_UPDATE_RATE_HZ
    static uint32_t previousTimePositionUpdate;         // Occurs @ altitude sensor update rate (max MAX_ALTITUDE_UPDATE_RATE_HZ)
    static uint32_t previousTimeUpdate;                 // Occurs @ looptime rate

    uint32_t deltaMicros = currentTime - previousTimeUpdate;
    previousTimeUpdate = currentTime;

    // If last position update was too long in the past - ignore it (likely restarting altitude controller)
    if (deltaMicros > HZ2US(MIN_POSITION_UPDATE_RATE_HZ)) {
        resetTimer(&targetPositionUpdateTimer, currentTime);
        previousTimeUpdate = currentTime;
        previousTimePositionUpdate = currentTime;
        resetMulticopterAltitudeController();
        return;
    }

    // Update altitude target from RC input or RTL controller
    if (updateTimer(&targetPositionUpdateTimer, HZ2US(POSITION_TARGET_UPDATE_RATE_HZ), currentTime)) {
        if (navShouldApplyAutonomousLandingLogic()) {
            // Gradually reduce descent speed depending on actual altitude.
            if (posControl.actualState.pos.V.Z > (posControl.homePosition.pos.V.Z + 1000.0f)) {
                updateAltitudeTargetFromClimbRate(-200.0f);
            }
            else if (posControl.actualState.pos.V.Z > (posControl.homePosition.pos.V.Z + 250.0f)) {
                updateAltitudeTargetFromClimbRate(-100.0f);
            }
            else {
                updateAltitudeTargetFromClimbRate(-50.0f);
            }
        }

        updateAltitudeTargetFromRCInput_MC();
    }

    // If we have an update on vertical position data - update velocity and accel targets
    if (posControl.flags.verticalPositionNewData) {
        uint32_t deltaMicrosPositionUpdate = currentTime - previousTimePositionUpdate;
        previousTimePositionUpdate = currentTime;

        // Check if last correction was too log ago - ignore this update
        if (deltaMicrosPositionUpdate < HZ2US(MIN_POSITION_UPDATE_RATE_HZ)) {
            updateSurfaceTrackingAltitudeSetpoint_MC();
            updateAltitudeVelocityController_MC();
            updateAltitudeThrottleController_MC(deltaMicrosPositionUpdate);
        }
        else {
            // due to some glitch position update has not occurred in time, reset altitude controller
            resetMulticopterAltitudeController();
        }

        // Indicate that information is no longer usable
        posControl.flags.verticalPositionNewData = 0;
    }

    // Update throttle controller
    rcCommand[THROTTLE] = constrain(altholdInitialThrottle + posControl.rcAdjustment[THROTTLE], posControl.escAndServoConfig->minthrottle, posControl.escAndServoConfig->maxthrottle);

    // Save processed throttle for future use
    rcCommandAdjustedThrottle = rcCommand[THROTTLE];
}

/*-----------------------------------------------------------
 * Heading controller for multicopter aircraft
 *-----------------------------------------------------------*/
/*-----------------------------------------------------------
 * Adjusts desired heading from pilot's input
 *-----------------------------------------------------------*/
static void adjustHeadingFromRCInput_MC()
{
    // In some cases pilot has no control over flight direction
    if (!navCanAdjustHeadingFromRCInput()) {
        posControl.flags.isAdjustingHeading = false;
        return;
    }

    int16_t rcYawAdjustment = applyDeadband(rcCommand[YAW], posControl.navConfig->pos_hold_deadband);
    if (rcYawAdjustment && navShouldApplyPosHold()) {
        // Can only allow pilot to set the new heading if doing PH, during RTH copter will target itself to home
        posControl.desiredState.yaw = posControl.actualState.yaw;
        posControl.flags.isAdjustingHeading = true;
    }
    else {
        posControl.flags.isAdjustingHeading = false;
    }
}

void applyMulticopterHeadingController(void)
{
    if (posControl.flags.headingNewData) {
        adjustHeadingFromRCInput_MC();

        // Indicate that information is no longer usable
        posControl.flags.headingNewData = 0;
    }

    // Control yaw my magHold PID
    magHold = CENTIDEGREES_TO_DEGREES(posControl.desiredState.yaw);
}

void resetMulticopterHeadingController(void)
{
    magHold = CENTIDEGREES_TO_DEGREES(posControl.actualState.yaw);
}

/*-----------------------------------------------------------
 * XY-position controller for multicopter aircraft
 *-----------------------------------------------------------*/
static filterStatePt1_t mcPosControllerAccFilterStateX, mcPosControllerAccFilterStateY;
static float lastAccelTargetX = 0.0f, lastAccelTargetY = 0.0f;

void resetMulticopterPositionController(void)
{
    int axis;
    for (axis = 0; axis < 2; axis++) {
        navPidReset(&posControl.pids.vel[axis]);
        posControl.rcAdjustment[axis] = 0;
        mcPosControllerAccFilterStateX.state = 0.0f;
        mcPosControllerAccFilterStateY.state = 0.0f;
        lastAccelTargetX = 0.0f;
        lastAccelTargetY = 0.0f;
    }
}

static void updatePositionTargetFromRCInput_MC(void)
{
    // In some cases pilot has no control over flight direction
    if (!navCanAdjustHorizontalVelocityAndAttitudeFromRCInput()) {
        posControl.flags.isAdjustingPosition = false;
        return;
    }

    int16_t rcPitchAdjustment = applyDeadband(rcCommand[PITCH], posControl.navConfig->pos_hold_deadband);
    int16_t rcRollAdjustment = applyDeadband(rcCommand[ROLL], posControl.navConfig->pos_hold_deadband);

    if (rcPitchAdjustment || rcRollAdjustment) {
        float rcVelX = rcPitchAdjustment * posControl.navConfig->max_manual_speed / 500;
        float rcVelY = rcRollAdjustment * posControl.navConfig->max_manual_speed / 500;

        // Rotate these velocities from body frame to to earth frame
        float neuVelX = rcVelX * posControl.actualState.cosYaw - rcVelY * posControl.actualState.sinYaw;
        float neuVelY = rcVelX * posControl.actualState.sinYaw + rcVelY * posControl.actualState.cosYaw;

        // Calculate new position target, so Pos-to-Vel P-controller would yield desired velocity
        posControl.desiredState.pos.V.X = posControl.actualState.pos.V.X + (neuVelX / posControl.pids.pos[X].param.kP);
        posControl.desiredState.pos.V.Y = posControl.actualState.pos.V.Y + (neuVelY / posControl.pids.pos[Y].param.kP);

        posControl.flags.isAdjustingPosition = true;
    }
    else {
        // Adjusting finished - reset desired position to stay exactly where pilot released the stick
        if (posControl.flags.isAdjustingPosition) {
            posControl.desiredState.pos.V.X = posControl.actualState.pos.V.X;
            posControl.desiredState.pos.V.Y = posControl.actualState.pos.V.Y;
        }

        posControl.flags.isAdjustingPosition = false;
    }
}

static void updatePositionLeanAngleFromRCInput_MC()
{
    // In some cases pilot has no control over flight direction
    if (!navCanAdjustHorizontalVelocityAndAttitudeFromRCInput()) {
        posControl.flags.isAdjustingPosition = false;
        return;
    }

    int16_t rcRollAdjustment = applyDeadband(rcCommand[ROLL], posControl.navConfig->pos_hold_deadband);
    int16_t rcPitchAdjustment = applyDeadband(rcCommand[PITCH], posControl.navConfig->pos_hold_deadband);

    if (rcPitchAdjustment || rcRollAdjustment) {
        // Direct attitude control
        posControl.rcAdjustment[ROLL] = rcCommandToLeanAngle(rcCommand[ROLL]);
        posControl.rcAdjustment[PITCH] = rcCommandToLeanAngle(rcCommand[PITCH]);

        // If we are in position hold mode, so adjust poshold position
        if (navShouldApplyPosHold()) {
            posControl.desiredState.pos.V.X = posControl.actualState.pos.V.X;
            posControl.desiredState.pos.V.Y = posControl.actualState.pos.V.Y;
        }

        // When sticks are released we should restart velocity PIDs
        navPidReset(&posControl.pids.vel[X]);
        navPidReset(&posControl.pids.vel[Y]);

        posControl.flags.isAdjustingPosition = true;
    }
    else {
        posControl.flags.isAdjustingPosition = false;
    }
}

static void updatePositionVelocityController_MC(void)
{
    float posErrorX = posControl.desiredState.pos.V.X - posControl.actualState.pos.V.X;
    float posErrorY = posControl.desiredState.pos.V.Y - posControl.actualState.pos.V.Y;

    float newVelX = posErrorX * posControl.pids.pos[X].param.kP;
    float newVelY = posErrorY * posControl.pids.pos[Y].param.kP;
    float newVelTotal = sqrtf(sq(newVelX) + sq(newVelY));

    if (newVelTotal > posControl.navConfig->max_speed) {
        newVelX = posControl.navConfig->max_speed * (newVelX / newVelTotal);
        newVelY = posControl.navConfig->max_speed * (newVelY / newVelTotal);
    }

    posControl.desiredState.vel.V.X = newVelX;
    posControl.desiredState.vel.V.Y = newVelY;

#if defined(NAV_BLACKBOX)
    navDesiredVelocity[X] = constrain(lrintf(posControl.desiredState.vel.V.X), -32678, 32767);
    navDesiredVelocity[Y] = constrain(lrintf(posControl.desiredState.vel.V.Y), -32678, 32767);
    navTargetPosition[X] = constrain(lrintf(posControl.desiredState.pos.V.X), -32678, 32767);
    navTargetPosition[Y] = constrain(lrintf(posControl.desiredState.pos.V.Y), -32678, 32767);
#endif
}

static void updatePositionAccelController_MC(uint32_t deltaMicros, float maxAccelLimit)
{
    float velErrorX, velErrorY, newAccelX, newAccelY;

    // Calculate velocity error
    velErrorX = posControl.desiredState.vel.V.X - posControl.actualState.vel.V.X;
    velErrorY = posControl.desiredState.vel.V.Y - posControl.actualState.vel.V.Y;

    // Calculate XY-acceleration limit according to velocity error limit
    float accelLimitX, accelLimitY;
    float velErrorMagnitude = sqrtf(sq(velErrorX) + sq(velErrorY));
    if (velErrorMagnitude > 0.1f) {
        accelLimitX = maxAccelLimit / velErrorMagnitude * fabsf(velErrorX);
        accelLimitY = maxAccelLimit / velErrorMagnitude * fabsf(velErrorY);
    }
    else {
        accelLimitX = maxAccelLimit / 1.414213f;
        accelLimitY = accelLimitX;
    }

    // Apply additional jerk limiting of 1700 cm/s^3 (~100 deg/s), almost any copter should be able to achieve this rate
    // This will assure that we wont't saturate out LEVEL and RATE PID controller
    float maxAccelChange = US2S(deltaMicros) * 1700.0f;
    float accelLimitXMin = constrainf(lastAccelTargetX - maxAccelChange, -accelLimitX, +accelLimitX);
    float accelLimitXMax = constrainf(lastAccelTargetX + maxAccelChange, -accelLimitX, +accelLimitX);
    float accelLimitYMin = constrainf(lastAccelTargetY - maxAccelChange, -accelLimitY, +accelLimitY);
    float accelLimitYMax = constrainf(lastAccelTargetY + maxAccelChange, -accelLimitY, +accelLimitY);

    // TODO: Verify if we need jerk limiting after all

    // Apply PID with output limiting and I-term anti-windup
    // Pre-calculated accelLimit and the logic of navPidApply2 function guarantee that our newAccel won't exceed maxAccelLimit
    // Thus we don't need to do anything else with calculated acceleration
    newAccelX = navPidApply2(posControl.desiredState.vel.V.X, posControl.actualState.vel.V.X, US2S(deltaMicros), &posControl.pids.vel[X], accelLimitXMin, accelLimitXMax);
    newAccelY = navPidApply2(posControl.desiredState.vel.V.Y, posControl.actualState.vel.V.Y, US2S(deltaMicros), &posControl.pids.vel[Y], accelLimitYMin, accelLimitYMax);

    // Save last acceleration target
    lastAccelTargetX = newAccelX;
    lastAccelTargetY = newAccelY;

    // Apply LPF to jerk limited acceleration target
    float accelN = filterApplyPt1(newAccelX, &mcPosControllerAccFilterStateX, NAV_ACCEL_CUTOFF_FREQUENCY_HZ, US2S(deltaMicros));
    float accelE = filterApplyPt1(newAccelY, &mcPosControllerAccFilterStateY, NAV_ACCEL_CUTOFF_FREQUENCY_HZ, US2S(deltaMicros));

    // Rotate acceleration target into forward-right frame (aircraft)
    float accelForward = accelN * posControl.actualState.cosYaw + accelE * posControl.actualState.sinYaw;
    float accelRight = -accelN * posControl.actualState.sinYaw + accelE * posControl.actualState.cosYaw;

    // Calculate banking angles
    float desiredPitch = atan2_approx(accelForward, GRAVITY_CMSS);
    float desiredRoll = atan2_approx(accelRight * cos_approx(desiredPitch), GRAVITY_CMSS);

    posControl.rcAdjustment[ROLL] = constrain(RADIANS_TO_DECIDEGREES(desiredRoll), -NAV_ROLL_PITCH_MAX_DECIDEGREES, NAV_ROLL_PITCH_MAX_DECIDEGREES);
    posControl.rcAdjustment[PITCH] = constrain(RADIANS_TO_DECIDEGREES(desiredPitch), -NAV_ROLL_PITCH_MAX_DECIDEGREES, NAV_ROLL_PITCH_MAX_DECIDEGREES);
}

void applyMulticopterPositionController(uint32_t currentTime)
{
    static navigationTimer_t targetPositionUpdateTimer; // Occurs @ POSITION_TARGET_UPDATE_RATE_HZ
    static uint32_t previousTimePositionUpdate;         // Occurs @ GPS update rate
    static uint32_t previousTimeUpdate;                 // Occurs @ looptime rate
    bool forceGPSAttiMode = false;

    uint32_t deltaMicros = currentTime - previousTimeUpdate;
    previousTimeUpdate = currentTime;

    // If last position update was too long in the past - ignore it (likely restarting position controller)
    if (deltaMicros > HZ2US(MIN_POSITION_UPDATE_RATE_HZ)) {
        resetTimer(&targetPositionUpdateTimer, currentTime);
        previousTimeUpdate = currentTime;
        previousTimePositionUpdate = currentTime;
        resetMulticopterPositionController();
        return;
    }

    // Apply controller only if position source is valid. In absence of valid pos sensor (GPS loss), we'd stick in forced ANGLE mode
    // and pilots input would be passed thru to PID controller
    if (posControl.flags.hasValidPositionSensor) {
        // Update position target from RC input
        if (updateTimer(&targetPositionUpdateTimer, HZ2US(POSITION_TARGET_UPDATE_RATE_HZ), currentTime)) {
            if (posControl.navConfig->flags.user_control_mode == NAV_GPS_CRUISE && !forceGPSAttiMode) {
                updatePositionTargetFromRCInput_MC();
            }
        }

        // If we have new position - update velocity and acceleration controllers
        if (posControl.flags.horizontalPositionNewData) {
            uint32_t deltaMicrosPositionUpdate = currentTime - previousTimePositionUpdate;
            previousTimePositionUpdate = currentTime;

            if (deltaMicrosPositionUpdate < HZ2US(MIN_POSITION_UPDATE_RATE_HZ)) {
                updatePositionVelocityController_MC();

                if (navShouldApplyWaypoint() || navShouldApplyRTH()) {
                    // In case of waypoint navigation and RTH limit maximum acceleration to lower value
                    updatePositionAccelController_MC(deltaMicrosPositionUpdate, NAV_ACCEL_SLOW_XY_MAX);
                }
                else {
                    // In case of PH limit acceleration to some high value
                    updatePositionAccelController_MC(deltaMicrosPositionUpdate, NAV_ACCELERATION_XY_MAX);
                }
            }
            else {
                resetMulticopterPositionController();
            }

            // Indicate that information is no longer usable
            posControl.flags.horizontalPositionNewData = 0;
        }
    }
    else {
        /* No position data, disable automatic adjustment */
        posControl.rcAdjustment[PITCH] = 0;
        posControl.rcAdjustment[ROLL] = 0;

        /* Force GPS_ATTI mode */
        forceGPSAttiMode = true;
    }

    /* Process pilot input (only GPS_ATTI mode) */
    if (posControl.navConfig->flags.user_control_mode == NAV_GPS_ATTI || forceGPSAttiMode) {
        updatePositionLeanAngleFromRCInput_MC();
    }

    rcCommand[PITCH] = leanAngleToRcCommand(posControl.rcAdjustment[PITCH]);
    rcCommand[ROLL] = leanAngleToRcCommand(posControl.rcAdjustment[ROLL]);
}

/*-----------------------------------------------------------
 * Multicopter land detector
 *-----------------------------------------------------------*/
bool isMulticopterLandingDetected(uint32_t * landingTimer)
{
    uint32_t currentTime = micros();

    // Average climb rate should be low enough
    bool verticalMovement = fabsf(posControl.actualState.vel.V.Z) > 25.0f;

    // check if we are moving horizontally
    bool horizontalMovement = sqrtf(sq(posControl.actualState.vel.V.X) + sq(posControl.actualState.vel.V.Y)) > 100.0f;

    // Throttle should be low enough
    // We use rcCommandAdjustedThrottle to keep track of NAV corrected throttle (isLandingDetected is executed
    // from processRx() and rcCommand at that moment holds rc input, not adjusted values from NAV core)
    bool minimalThrust = rcCommandAdjustedThrottle <= (posControl.escAndServoConfig->minthrottle + (posControl.escAndServoConfig->maxthrottle - posControl.escAndServoConfig->minthrottle) * 0.25f);

    if (!minimalThrust || !navShouldApplyAutonomousLandingLogic() || !navShouldApplyAltHold() || verticalMovement || horizontalMovement) {
        *landingTimer = currentTime;
        return false;
    }
    else {
        return ((currentTime - *landingTimer) > (LAND_DETECTOR_TRIGGER_TIME_MS * 1000)) ? true : false;
    }
}

/*-----------------------------------------------------------
 * Multicopter emergency landing
 *-----------------------------------------------------------*/
void applyMulticopterEmergencyLandingController(void)
{
    rcCommand[ROLL] = 0;
    rcCommand[PITCH] = 0;
    rcCommand[YAW] = 0;
    rcCommand[THROTTLE] = 1300; // FIXME
}

#endif  // NAV
