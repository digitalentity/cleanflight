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

#include "platform.h"

#if defined(USE_NAV)

#include "build/build_config.h"
#include "build/debug.h"

#include "drivers/time.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/boardalignment.h"

#include "fc/config.h"
#include "fc/rc_control.h"
#include "fc/rc_curves.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/failsafe.h"
#include "flight/mixer.h"

#include "navigation/navigation.h"
#include "navigation/navigation_private.h"


/*-----------------------------------------------------------
 * Altitude controller for multicopter aircraft
 *-----------------------------------------------------------*/
static float rcCommandAdjustedThrottle;
static float altHoldThrottleRCZero;
static pt1Filter_t altholdThrottleFilterState;
static bool prepareForTakeoffOnReset = false;

// Position to velocity controller for Z axis
static void updateAltitudeVelocityController_MC(timeDelta_t deltaMicros)
{
    const float altitudeError = posControl.desiredState.pos.z - navGetCurrentActualPositionAndVelocity()->pos.z;
    float targetVel = altitudeError * posControl.pids.pos[Z].param.kP;

    // hard limit desired target velocity to max_climb_rate
    if (posControl.flags.isAdjustingAltitude) {
        targetVel = constrainf(targetVel, -navConfig()->general.max_manual_climb_rate, navConfig()->general.max_manual_climb_rate);
    }
    else {
        targetVel = constrainf(targetVel, -navConfig()->general.max_auto_climb_rate, navConfig()->general.max_auto_climb_rate);
    }

    posControl.pids.pos[Z].output_constrained = targetVel;

    // Limit max up/down acceleration target
    const float smallVelChange = US2S(deltaMicros) * (GRAVITY_CMSS * 0.1f);
    const float velTargetChange = targetVel - posControl.desiredState.vel.z;

    if (velTargetChange <= -smallVelChange) {
        // Large & Negative - acceleration is _down_. We can't reach more than -1G in any possible condition. Hard limit to 0.8G to stay safe
        // This should be safe enough for stability since we only reduce throttle
        const float maxVelDifference = US2S(deltaMicros) * (GRAVITY_CMSS * 0.8f);
        posControl.desiredState.vel.z = constrainf(targetVel, posControl.desiredState.vel.z - maxVelDifference, posControl.desiredState.vel.z + maxVelDifference);
    }
    else if (velTargetChange >= smallVelChange) {
        // Large and positive - acceleration is _up_. We are limited by thrust/weight ratio which is usually about 2:1 (hover around 50% throttle).
        // T/W ratio = 2 means we are able to reach 1G acceleration in "UP" direction. Hard limit to 0.5G to be on a safe side and avoid abrupt throttle changes
        const float maxVelDifference = US2S(deltaMicros) * (GRAVITY_CMSS * 0.5f);
        posControl.desiredState.vel.z = constrainf(targetVel, posControl.desiredState.vel.z - maxVelDifference, posControl.desiredState.vel.z + maxVelDifference);
    }
    else {
        // Small - desired acceleration is less than 0.1G. We should be safe setting velocity target directly - any platform should be able to satisfy this
        posControl.desiredState.vel.z = targetVel;
    }

#if defined(NAV_BLACKBOX)
    navDesiredVelocity[Z] = constrain(lrintf(posControl.desiredState.vel.z), -32678, 32767);
#endif
}

static void updateAltitudeThrottleController_MC(timeDelta_t deltaMicros)
{
    // Calculate min and max throttle boundaries (to compensate for integral windup)
    const int16_t thrAdjustmentMin = (int16_t)motorConfig()->minthrottle - (int16_t)navConfig()->mc.hover_throttle;
    const int16_t thrAdjustmentMax = (int16_t)motorConfig()->maxthrottle - (int16_t)navConfig()->mc.hover_throttle;

    posControl.rcAdjustment[THROTTLE] = navPidApply2(&posControl.pids.vel[Z], posControl.desiredState.vel.z, navGetCurrentActualPositionAndVelocity()->vel.z, US2S(deltaMicros), thrAdjustmentMin, thrAdjustmentMax, 0);

    posControl.rcAdjustment[THROTTLE] = pt1FilterApply4(&altholdThrottleFilterState, posControl.rcAdjustment[THROTTLE], NAV_THROTTLE_CUTOFF_FREQENCY_HZ, US2S(deltaMicros));
    posControl.rcAdjustment[THROTTLE] = constrain(posControl.rcAdjustment[THROTTLE], thrAdjustmentMin, thrAdjustmentMax);
}

bool adjustMulticopterAltitudeFromRCInput(void)
{
    float inputThr = rcControlGetInputAxis(THROTTLE);
    if (posControl.flags.isTerrainFollowEnabled) {
        const float altTarget = scaleRangef(inputThr, RC_COMMAND_MIN, RC_COMMAND_MAX, 0, navConfig()->general.max_terrain_follow_altitude);

        // In terrain follow mode we apply different logic for terrain control
        if (posControl.flags.estAglStatus == EST_TRUSTED && altTarget > 10.0f) {
            // We have solid terrain sensor signal - directly map throttle to altitude
            updateClimbRateToAltitudeController(0, ROC_TO_ALT_RESET);
            posControl.desiredState.pos.z = altTarget;
        }
        else {
            updateClimbRateToAltitudeController(-50.0f, ROC_TO_ALT_NORMAL);
        }

        // In surface tracking we always indicate that we're adjusting altitude
        return true;
    }
    else {
        const float altHoldDeadband = rcCommandConvertPWMDeadband(rcControlsConfig()->alt_hold_deadband);
        const float rcThrottleAdjustment = fapplyDeadbandf(inputThr - altHoldThrottleRCZero, altHoldDeadband);
        if (rcThrottleAdjustment) {
            // set velocity proportional to stick movement
            float rcClimbRate;

            // Make sure we can satisfy max_manual_climb_rate in both up and down directions
            if (rcThrottleAdjustment > 0) {
                // Scaling from altHoldThrottleRCZero to maxthrottle
                rcClimbRate = rcThrottleAdjustment * navConfig()->general.max_manual_climb_rate / (RC_COMMAND_MAX - altHoldThrottleRCZero - altHoldDeadband);
            }
            else {
                // Scaling from minthrottle to altHoldThrottleRCZero
                rcClimbRate = rcThrottleAdjustment * navConfig()->general.max_manual_climb_rate / (altHoldThrottleRCZero - RC_COMMAND_MIN - altHoldDeadband);
            }

            updateClimbRateToAltitudeController(rcClimbRate, ROC_TO_ALT_NORMAL);

            return true;
        }
        else {
            // Adjusting finished - reset desired position to stay exactly where pilot released the stick
            if (posControl.flags.isAdjustingAltitude) {
                updateClimbRateToAltitudeController(0, ROC_TO_ALT_RESET);
            }

            return false;
        }
    }
}

void setupMulticopterAltitudeController(void)
{
    const throttleStatus_e throttleStatus = calculateThrottleStatus();

    if (navConfig()->general.flags.use_thr_mid_for_althold) {
        altHoldThrottleRCZero = RC_COMMAND_CENTER;
    }
    else {
        // If throttle status is THROTTLE_LOW - use Thr Mid anyway
        if (throttleStatus == THROTTLE_LOW) {
            altHoldThrottleRCZero = RC_COMMAND_CENTER;
        }
        else {
            altHoldThrottleRCZero = rcControlGetInputAxis(THROTTLE);
        }
    }

    // Make sure we are able to satisfy the deadband
    float altHoldDeadband = rcCommandConvertPWMDeadband(rcControlsConfig()->alt_hold_deadband);
    // This used to be 10 PWM units (1000 range)
    float altHoldDeadbandMargin = 0.01f;
    float altHoldThrMin = RC_COMMAND_MIN + altHoldDeadband + altHoldDeadbandMargin;
    float altHoldThrMax = RC_COMMAND_MAX - altHoldDeadband - altHoldDeadbandMargin;
    altHoldThrottleRCZero = constrain(altHoldThrottleRCZero, altHoldThrMin, altHoldThrMax);

    // Force AH controller to initialize althold integral for pending takeoff on reset
    // Signal for that is low throttle _and_ low actual altitude
    if (throttleStatus == THROTTLE_LOW && fabsf(navGetCurrentActualPositionAndVelocity()->pos.z) <= 50.0f) {
        prepareForTakeoffOnReset = true;
    }
}

void resetMulticopterAltitudeController(void)
{
    const navEstimatedPosVel_t * posToUse = navGetCurrentActualPositionAndVelocity();

    navPidReset(&posControl.pids.vel[Z]);
    navPidReset(&posControl.pids.surface);
    posControl.rcAdjustment[THROTTLE] = 0;

    if (prepareForTakeoffOnReset) {
        /* If we are preparing for takeoff - start with lowset possible climb rate, adjust alt target and make sure throttle doesn't jump */
        posControl.desiredState.vel.z = -navConfig()->general.max_manual_climb_rate;
        posControl.desiredState.pos.z = posToUse->pos.z - (navConfig()->general.max_manual_climb_rate / posControl.pids.pos[Z].param.kP);
        posControl.pids.vel[Z].integrator = -500.0f;
        pt1FilterReset(&altholdThrottleFilterState, -500.0f);
        prepareForTakeoffOnReset = false;
    }
    else {
        posControl.desiredState.vel.z = posToUse->vel.z;   // Gradually transition from current climb
        pt1FilterReset(&altholdThrottleFilterState, 0.0f);
    }
}

static void applyMulticopterAltitudeController(timeUs_t currentTimeUs)
{
    static timeUs_t previousTimePositionUpdate;         // Occurs @ altitude sensor update rate (max MAX_ALTITUDE_UPDATE_RATE_HZ)
    static timeUs_t previousTimeUpdate;                 // Occurs @ looptime rate

    const timeDelta_t deltaMicros = currentTimeUs - previousTimeUpdate;
    rcCommand_t controlOutput;

    previousTimeUpdate = currentTimeUs;
    rcCommandCopy(&controlOutput, rcControlGetOutput());

    // If last position update was too long in the past - ignore it (likely restarting altitude controller)
    if (deltaMicros > HZ2US(MIN_POSITION_UPDATE_RATE_HZ)) {
        previousTimeUpdate = currentTimeUs;
        previousTimePositionUpdate = currentTimeUs;
        resetMulticopterAltitudeController();
        return;
    }

    // If we have an update on vertical position data - update velocity and accel targets
    if (posControl.flags.verticalPositionDataNew) {
        const timeDelta_t deltaMicrosPositionUpdate = currentTimeUs - previousTimePositionUpdate;
        previousTimePositionUpdate = currentTimeUs;

        // Check if last correction was too log ago - ignore this update
        if (deltaMicrosPositionUpdate < HZ2US(MIN_POSITION_UPDATE_RATE_HZ)) {
            updateAltitudeVelocityController_MC(deltaMicrosPositionUpdate);
            updateAltitudeThrottleController_MC(deltaMicrosPositionUpdate);
        }
        else {
            // due to some glitch position update has not occurred in time, reset altitude controller
            resetMulticopterAltitudeController();
        }

        // Indicate that information is no longer usable
        posControl.flags.verticalPositionDataConsumed = 1;
    }

    // Update throttle controller
    controlOutput.throttle = rcCommandMapUnidirectionalPWMThrottle((int16_t)navConfig()->mc.hover_throttle + posControl.rcAdjustment[THROTTLE]);

    // Save processed throttle for future use
    rcCommandAdjustedThrottle = controlOutput.throttle;

    rcControlUpdateOutput(&controlOutput, RC_CONTROL_SOURCE_NAVIGATION);
}

/*-----------------------------------------------------------
 * Adjusts desired heading from pilot's input
 *-----------------------------------------------------------*/
bool adjustMulticopterHeadingFromRCInput(void)
{
    if (rcControlGetInputAxisApplyingPosholdDeadband(YAW) != RC_COMMAND_CENTER) {
        // Can only allow pilot to set the new heading if doing PH, during RTH copter will target itself to home
        posControl.desiredState.yaw = posControl.actualState.yaw;

        return true;
    }
    return false;
}

/*-----------------------------------------------------------
 * XY-position controller for multicopter aircraft
 *-----------------------------------------------------------*/
static float lastAccelTargetX = 0.0f, lastAccelTargetY = 0.0f;

void resetMulticopterBrakingMode(void)
{
    DISABLE_STATE(NAV_CRUISE_BRAKING);
    DISABLE_STATE(NAV_CRUISE_BRAKING_BOOST);
    DISABLE_STATE(NAV_CRUISE_BRAKING_LOCKED);
}

static void processMulticopterBrakingMode(const bool isAdjusting)
{
#ifdef USE_MR_BRAKING_MODE
    static uint32_t brakingModeDisengageAt = 0;
    static uint32_t brakingBoostModeDisengageAt = 0;

    if (!(NAV_Status.state == MW_NAV_STATE_NONE || NAV_Status.state == MW_NAV_STATE_HOLD_INFINIT)) {
        resetMulticopterBrakingMode();
        return;
    }

    const bool brakingEntryAllowed =
        IS_RC_MODE_ACTIVE(BOXBRAKING) &&
        !STATE(NAV_CRUISE_BRAKING_LOCKED) &&
        posControl.actualState.velXY > navConfig()->mc.braking_speed_threshold &&
        !isAdjusting &&
        navConfig()->general.flags.user_control_mode == NAV_GPS_CRUISE &&
        navConfig()->mc.braking_speed_threshold > 0;


    /*
     * Case one, when we order to brake (sticks to the center) and we are moving above threshold
     * Speed is above 1m/s and sticks are centered
     * Extra condition: BRAKING flight mode has to be enabled
     */
    if (brakingEntryAllowed) {
        /*
         * Set currnt position and target position
         * Enabling NAV_CRUISE_BRAKING locks other routines from setting position!
         */
        setDesiredPosition(&navGetCurrentActualPositionAndVelocity()->pos, 0, NAV_POS_UPDATE_XY);

        ENABLE_STATE(NAV_CRUISE_BRAKING_LOCKED);
        ENABLE_STATE(NAV_CRUISE_BRAKING);

        //Set forced BRAKING disengage moment
        brakingModeDisengageAt = millis() + navConfig()->mc.braking_timeout;

        //If speed above threshold, start boost mode as well
        if (posControl.actualState.velXY > navConfig()->mc.braking_boost_speed_threshold) {
            ENABLE_STATE(NAV_CRUISE_BRAKING_BOOST);

            brakingBoostModeDisengageAt = millis() + navConfig()->mc.braking_boost_timeout;
        }

    }

    // We can enter braking only after user started to move the sticks
    if (STATE(NAV_CRUISE_BRAKING_LOCKED) && isAdjusting) {
        DISABLE_STATE(NAV_CRUISE_BRAKING_LOCKED);
    }

    /*
     * Case when speed dropped, disengage BREAKING_BOOST
     */
    if (
        STATE(NAV_CRUISE_BRAKING_BOOST) && (
            posControl.actualState.velXY <= navConfig()->mc.braking_boost_disengage_speed ||
            brakingBoostModeDisengageAt < millis()
    )) {
        DISABLE_STATE(NAV_CRUISE_BRAKING_BOOST);
    }

    /*
     * Case when we were braking but copter finally stopped or we started to move the sticks
     */
    if (STATE(NAV_CRUISE_BRAKING) && (
        posControl.actualState.velXY <= navConfig()->mc.braking_disengage_speed ||  //We stopped
        isAdjusting ||                                                              //Moved the sticks
        brakingModeDisengageAt < millis()                                           //Braking is done to timed disengage
    )) {
        DISABLE_STATE(NAV_CRUISE_BRAKING);
        DISABLE_STATE(NAV_CRUISE_BRAKING_BOOST);

        /*
         * When braking is done, store current position as desired one
         * We do not want to go back to the place where braking has started
         */
        setDesiredPosition(&navGetCurrentActualPositionAndVelocity()->pos, 0, NAV_POS_UPDATE_XY);
    }
#else
    UNUSED(isAdjusting);
#endif
}

void resetMulticopterPositionController(void)
{
    for (int axis = 0; axis < 2; axis++) {
        navPidReset(&posControl.pids.vel[axis]);
        posControl.rcAdjustment[axis] = 0;
        lastAccelTargetX = 0.0f;
        lastAccelTargetY = 0.0f;
    }
}

bool adjustMulticopterPositionFromRCInput(float rcPitchAdjustment, float rcRollAdjustment)
{
    bool isAdjusting = rcPitchAdjustment != 0 || rcRollAdjustment != 0;

    // Process braking mode
    processMulticopterBrakingMode(isAdjusting);

    // Actually change position
    if (isAdjusting) {
        // If mode is GPS_CRUISE, move target position, otherwise POS controller will passthru the RC input to ANGLE PID
        if (navConfig()->general.flags.user_control_mode == NAV_GPS_CRUISE) {
            const float rcVelX = rcPitchAdjustment * navConfig()->general.max_manual_speed;
            const float rcVelY = rcRollAdjustment * navConfig()->general.max_manual_speed;

            // Rotate these velocities from body frame to to earth frame
            const float neuVelX = rcVelX * posControl.actualState.cosYaw - rcVelY * posControl.actualState.sinYaw;
            const float neuVelY = rcVelX * posControl.actualState.sinYaw + rcVelY * posControl.actualState.cosYaw;

            // Calculate new position target, so Pos-to-Vel P-controller would yield desired velocity
            posControl.desiredState.pos.x = navGetCurrentActualPositionAndVelocity()->pos.x + (neuVelX / posControl.pids.pos[X].param.kP);
            posControl.desiredState.pos.y = navGetCurrentActualPositionAndVelocity()->pos.y + (neuVelY / posControl.pids.pos[Y].param.kP);
        }

        return true;
    }
    else {
        // Adjusting finished - reset desired position to stay exactly where pilot released the stick
        if (posControl.flags.isAdjustingPosition) {
            fpVector3_t stopPosition;
            calculateMulticopterInitialHoldPosition(&stopPosition);
            setDesiredPosition(&stopPosition, 0, NAV_POS_UPDATE_XY);
        }

        return false;
    }
}

static float getVelocityHeadingAttenuationFactor(void)
{
    // In WP mode scale velocity if heading is different from bearing
    if (navGetCurrentStateFlags() & NAV_AUTO_WP) {
        const int32_t headingError = constrain(wrap_18000(posControl.desiredState.yaw - posControl.actualState.yaw), -9000, 9000);
        const float velScaling = cos_approx(CENTIDEGREES_TO_RADIANS(headingError));

        return constrainf(velScaling * velScaling, 0.05f, 1.0f);
    } else {
        return 1.0f;
    }
}

static float getVelocityExpoAttenuationFactor(float velTotal, float velMax)
{
    // Calculate factor of how velocity with applied expo is different from unchanged velocity
    const float velScale = constrainf(velTotal / velMax, 0.01f, 1.0f);

    // navConfig()->max_speed * ((velScale * velScale * velScale) * posControl.posResponseExpo + velScale * (1 - posControl.posResponseExpo)) / velTotal;
    // ((velScale * velScale * velScale) * posControl.posResponseExpo + velScale * (1 - posControl.posResponseExpo)) / velScale
    // ((velScale * velScale) * posControl.posResponseExpo + (1 - posControl.posResponseExpo));
    return 1.0f - posControl.posResponseExpo * (1.0f - (velScale * velScale));  // x^3 expo factor
}

static void updatePositionVelocityController_MC(void)
{
    const float posErrorX = posControl.desiredState.pos.x - navGetCurrentActualPositionAndVelocity()->pos.x;
    const float posErrorY = posControl.desiredState.pos.y - navGetCurrentActualPositionAndVelocity()->pos.y;

    // Calculate target velocity
    float newVelX = posErrorX * posControl.pids.pos[X].param.kP;
    float newVelY = posErrorY * posControl.pids.pos[Y].param.kP;

    // Get max speed from generic NAV (waypoint specific), don't allow to move slower than 0.5 m/s
    const float maxSpeed = getActiveWaypointSpeed();

    // Scale velocity to respect max_speed
    float newVelTotal = sqrtf(sq(newVelX) + sq(newVelY));
    if (newVelTotal > maxSpeed) {
        newVelX = maxSpeed * (newVelX / newVelTotal);
        newVelY = maxSpeed * (newVelY / newVelTotal);
        newVelTotal = maxSpeed;
    }

    posControl.pids.pos[X].output_constrained = newVelX;
    posControl.pids.pos[Y].output_constrained = newVelY;

    // Apply expo & attenuation if heading in wrong direction - turn first, accelerate later (effective only in WP mode)
    const float velHeadFactor = getVelocityHeadingAttenuationFactor();
    const float velExpoFactor = getVelocityExpoAttenuationFactor(newVelTotal, maxSpeed);
    posControl.desiredState.vel.x = newVelX * velHeadFactor * velExpoFactor;
    posControl.desiredState.vel.y = newVelY * velHeadFactor * velExpoFactor;

#if defined(NAV_BLACKBOX)
    navDesiredVelocity[X] = constrain(lrintf(posControl.desiredState.vel.x), -32678, 32767);
    navDesiredVelocity[Y] = constrain(lrintf(posControl.desiredState.vel.y), -32678, 32767);
#endif
}

static void updatePositionAccelController_MC(timeDelta_t deltaMicros, float maxAccelLimit)
{

    // Calculate velocity error
    const float velErrorX = posControl.desiredState.vel.x - navGetCurrentActualPositionAndVelocity()->vel.x;
    const float velErrorY = posControl.desiredState.vel.y - navGetCurrentActualPositionAndVelocity()->vel.y;

    // Calculate XY-acceleration limit according to velocity error limit
    float accelLimitX, accelLimitY;
    const float velErrorMagnitude = sqrtf(sq(velErrorX) + sq(velErrorY));
    if (velErrorMagnitude > 0.1f) {
        accelLimitX = maxAccelLimit / velErrorMagnitude * fabsf(velErrorX);
        accelLimitY = maxAccelLimit / velErrorMagnitude * fabsf(velErrorY);
    } else {
        accelLimitX = maxAccelLimit / 1.414213f;
        accelLimitY = accelLimitX;
    }

    // Apply additional jerk limiting of 1700 cm/s^3 (~100 deg/s), almost any copter should be able to achieve this rate
    // This will assure that we wont't saturate out LEVEL and RATE PID controller

    float maxAccelChange = US2S(deltaMicros) * 1700.0f;
    //When braking, raise jerk limit even if we are not boosting acceleration
#ifdef USE_MR_BRAKING_MODE
    if (STATE(NAV_CRUISE_BRAKING)) {
        maxAccelChange = maxAccelChange * 2;
    }
#endif

    const float accelLimitXMin = constrainf(lastAccelTargetX - maxAccelChange, -accelLimitX, +accelLimitX);
    const float accelLimitXMax = constrainf(lastAccelTargetX + maxAccelChange, -accelLimitX, +accelLimitX);
    const float accelLimitYMin = constrainf(lastAccelTargetY - maxAccelChange, -accelLimitY, +accelLimitY);
    const float accelLimitYMax = constrainf(lastAccelTargetY + maxAccelChange, -accelLimitY, +accelLimitY);

    // TODO: Verify if we need jerk limiting after all

    // Apply PID with output limiting and I-term anti-windup
    // Pre-calculated accelLimit and the logic of navPidApply2 function guarantee that our newAccel won't exceed maxAccelLimit
    // Thus we don't need to do anything else with calculated acceleration
    float newAccelX = navPidApply2(&posControl.pids.vel[X], posControl.desiredState.vel.x, navGetCurrentActualPositionAndVelocity()->vel.x, US2S(deltaMicros), accelLimitXMin, accelLimitXMax, 0);
    float newAccelY = navPidApply2(&posControl.pids.vel[Y], posControl.desiredState.vel.y, navGetCurrentActualPositionAndVelocity()->vel.y, US2S(deltaMicros), accelLimitYMin, accelLimitYMax, 0);

    int32_t maxBankAngle = DEGREES_TO_DECIDEGREES(navConfig()->mc.max_bank_angle);

#ifdef USE_MR_BRAKING_MODE
    //Boost required accelerations
    if (STATE(NAV_CRUISE_BRAKING_BOOST) && navConfig()->mc.braking_boost_factor > 0) {
        const float rawBoostFactor = (float)navConfig()->mc.braking_boost_factor / 100.0f;

        //Scale boost factor according to speed
        const float boostFactor = constrainf(
            scaleRangef(
                posControl.actualState.velXY,
                navConfig()->mc.braking_boost_speed_threshold,
                navConfig()->general.max_manual_speed,
                0.0f,
                rawBoostFactor
            ),
            0.0f,
            rawBoostFactor
        );

        //Boost required acceleration for harder braking
        newAccelX = newAccelX * (1.0f + boostFactor);
        newAccelY = newAccelY * (1.0f + boostFactor);

        maxBankAngle = DEGREES_TO_DECIDEGREES(navConfig()->mc.braking_bank_angle);
    }
#endif

    // Save last acceleration target
    lastAccelTargetX = newAccelX;
    lastAccelTargetY = newAccelY;

    // Rotate acceleration target into forward-right frame (aircraft)
    const float accelForward = newAccelX * posControl.actualState.cosYaw + newAccelY * posControl.actualState.sinYaw;
    const float accelRight = -newAccelX * posControl.actualState.sinYaw + newAccelY * posControl.actualState.cosYaw;

    // Calculate banking angles
    const float desiredPitch = atan2_approx(accelForward, GRAVITY_CMSS);
    const float desiredRoll = atan2_approx(accelRight * cos_approx(desiredPitch), GRAVITY_CMSS);

    posControl.rcAdjustment[ROLL] = constrain(RADIANS_TO_DECIDEGREES(desiredRoll), -maxBankAngle, maxBankAngle);
    posControl.rcAdjustment[PITCH] = constrain(RADIANS_TO_DECIDEGREES(desiredPitch), -maxBankAngle, maxBankAngle);
}

static void applyMulticopterPositionController(timeUs_t currentTimeUs)
{
    static timeUs_t previousTimePositionUpdate;         // Occurs @ GPS update rate
    static timeUs_t previousTimeUpdate;                 // Occurs @ looptime rate

    const timeDelta_t deltaMicros = currentTimeUs - previousTimeUpdate;
    previousTimeUpdate = currentTimeUs;
    bool bypassPositionController;
    rcCommand_t controlOutput;

    // We should passthrough rcCommand is adjusting position in GPS_ATTI mode
    bypassPositionController = (navConfig()->general.flags.user_control_mode == NAV_GPS_ATTI) && posControl.flags.isAdjustingPosition;

    // If last call to controller was too long in the past - ignore it (likely restarting position controller)
    if (deltaMicros > HZ2US(MIN_POSITION_UPDATE_RATE_HZ)) {
        previousTimeUpdate = currentTimeUs;
        previousTimePositionUpdate = currentTimeUs;
        resetMulticopterPositionController();
        return;
    }

    // Apply controller only if position source is valid. In absence of valid pos sensor (GPS loss), we'd stick in forced ANGLE mode
    // and pilots input would be passed thru to PID controller
    if ((posControl.flags.estPosStatus >= EST_USABLE)) {
        // If we have new position - update velocity and acceleration controllers
        if (posControl.flags.horizontalPositionDataNew) {
            const timeDelta_t deltaMicrosPositionUpdate = currentTimeUs - previousTimePositionUpdate;
            previousTimePositionUpdate = currentTimeUs;

            if (!bypassPositionController) {
                // Update position controller
                if (deltaMicrosPositionUpdate < HZ2US(MIN_POSITION_UPDATE_RATE_HZ)) {
                    updatePositionVelocityController_MC();
                    updatePositionAccelController_MC(deltaMicrosPositionUpdate, NAV_ACCELERATION_XY_MAX);
                }
                else {
                    resetMulticopterPositionController();
                }
            }

            // Indicate that information is no longer usable
            posControl.flags.horizontalPositionDataConsumed = 1;
        }
    }
    else {
        /* No position data, disable automatic adjustment, rcCommand passthrough */
        posControl.rcAdjustment[PITCH] = 0;
        posControl.rcAdjustment[ROLL] = 0;
        bypassPositionController = true;
    }

    if (!bypassPositionController) {
        rcCommandCopy(&controlOutput, rcControlGetOutput());
        controlOutput.pitch = pidAngleToRcCommand(posControl.rcAdjustment[PITCH], pidProfile()->max_angle_inclination[FD_PITCH]);
        controlOutput.roll = pidAngleToRcCommand(posControl.rcAdjustment[ROLL], pidProfile()->max_angle_inclination[FD_ROLL]);
        rcControlUpdateOutput(&controlOutput, RC_CONTROL_SOURCE_NAVIGATION);
    }
}

/*-----------------------------------------------------------
 * Multicopter land detector
 *-----------------------------------------------------------*/
static timeUs_t landingTimer;
static timeUs_t landingDetectorStartedAt;
static int32_t landingThrSum;
static int32_t landingThrSamples;

void resetMulticopterLandingDetector(void)
{
    // FIXME: This function is called some time before isMulticopterLandingDetected is first called
    landingTimer = micros();
    landingDetectorStartedAt = 0; // ugly hack for now

    landingThrSum = 0;
    landingThrSamples = 0;
}

bool isMulticopterLandingDetected(void)
{
    const timeUs_t currentTimeUs = micros();

    // FIXME: Remove delay between resetMulticopterLandingDetector and first run of this function so this code isn't needed.
    if (landingDetectorStartedAt == 0) {
        landingDetectorStartedAt = currentTimeUs;
    }

    // Average climb rate should be low enough
    bool verticalMovement = fabsf(navGetCurrentActualPositionAndVelocity()->vel.z) > 25.0f;

    // check if we are moving horizontally
    bool horizontalMovement = posControl.actualState.velXY > 100.0f;

    // We have likely landed if throttle is 40PWM units below average descend throttle
    // We use rcCommandAdjustedThrottle to keep track of NAV corrected throttle (isLandingDetected is executed
    // from processRx() and rcCommand at that moment holds rc input, not adjusted values from NAV core)
    // Wait for 1 second so throttle has stabilized.
    bool isAtMinimalThrust = false;
    if (currentTimeUs - landingDetectorStartedAt > 1000 * 1000) {
        landingThrSamples += 1;
        landingThrSum += rcCommandAdjustedThrottle;
        isAtMinimalThrust = rcCommandAdjustedThrottle < (landingThrSum / landingThrSamples - 0.040f);
    }

    bool possibleLandingDetected = isAtMinimalThrust && !verticalMovement && !horizontalMovement;

    DEBUG_SET(DEBUG_NAV_LANDING_DETECTOR, 0, isAtMinimalThrust * 100 + !verticalMovement * 10 + !horizontalMovement * 1);
    DEBUG_SET(DEBUG_NAV_LANDING_DETECTOR, 1, (landingThrSamples == 0) ? 0 : (rcCommandAdjustedThrottle - (landingThrSum / landingThrSamples)));
    DEBUG_SET(DEBUG_NAV_LANDING_DETECTOR, 2, (currentTimeUs - landingTimer) / 1000);

    // If we have surface sensor (for example sonar) - use it to detect touchdown
    if ((posControl.flags.estAglStatus == EST_TRUSTED) && (posControl.actualState.agl.pos.z >= 0)) {
        // TODO: Come up with a clever way to let sonar increase detection performance, not just add extra safety.
        // TODO: Out of range sonar may give reading that looks like we landed, find a way to check if sonar is healthy.
        // surfaceMin is our ground reference. If we are less than 5cm above the ground - we are likely landed
        possibleLandingDetected = possibleLandingDetected && (posControl.actualState.agl.pos.z <= (posControl.actualState.surfaceMin + 5.0f));
    }

    if (!possibleLandingDetected) {
        landingTimer = currentTimeUs;
        return false;
    }
    else {
        return ((currentTimeUs - landingTimer) > (navConfig()->mc.auto_disarm_delay * 1000)) ? true : false;
    }
}

/*-----------------------------------------------------------
 * Multicopter emergency landing
 *-----------------------------------------------------------*/
static void applyMulticopterEmergencyLandingController(timeUs_t currentTimeUs)
{
    static timeUs_t previousTimeUpdate;
    static timeUs_t previousTimePositionUpdate;
    const timeDelta_t deltaMicros = currentTimeUs - previousTimeUpdate;
    rcCommand_t controlOutput;

    previousTimeUpdate = currentTimeUs;
    rcCommandCopy(&controlOutput, rcControlGetOutput());

    /* Attempt to stabilise */
    controlOutput.roll = RC_COMMAND_CENTER;
    controlOutput.pitch = RC_COMMAND_CENTER;
    controlOutput.yaw = RC_COMMAND_CENTER;

    if ((posControl.flags.estAltStatus >= EST_USABLE)) {
        // If last position update was too long in the past - ignore it (likely restarting altitude controller)
        if (deltaMicros > HZ2US(MIN_POSITION_UPDATE_RATE_HZ)) {
            previousTimeUpdate = currentTimeUs;
            previousTimePositionUpdate = currentTimeUs;
            resetMulticopterAltitudeController();
            return;
        }

        if (posControl.flags.verticalPositionDataNew) {
            const timeDelta_t deltaMicrosPositionUpdate = currentTimeUs - previousTimePositionUpdate;
            previousTimePositionUpdate = currentTimeUs;

            // Check if last correction was too log ago - ignore this update
            if (deltaMicrosPositionUpdate < HZ2US(MIN_POSITION_UPDATE_RATE_HZ)) {
                updateClimbRateToAltitudeController(-1.0f * navConfig()->general.emerg_descent_rate, ROC_TO_ALT_NORMAL);
                updateAltitudeVelocityController_MC(deltaMicrosPositionUpdate);
                updateAltitudeThrottleController_MC(deltaMicrosPositionUpdate);
            }
            else {
                // due to some glitch position update has not occurred in time, reset altitude controller
                resetMulticopterAltitudeController();
            }

            // Indicate that information is no longer usable
            posControl.flags.verticalPositionDataConsumed = 1;
        }

        // Update throttle controller
        controlOutput.throttle = rcCommandMapUnidirectionalPWMThrottle((int16_t)navConfig()->mc.hover_throttle + posControl.rcAdjustment[THROTTLE]);
    }
    else {
        /* Sensors has gone haywire, attempt to land regardless */
        if (failsafeConfig()->failsafe_procedure == FAILSAFE_PROCEDURE_DROP_IT) {
            controlOutput.throttle = RC_COMMAND_CENTER;
        }
        else {
            controlOutput.throttle = rcCommandMapUnidirectionalPWMThrottle(failsafeConfig()->failsafe_throttle);
        }
    }
    rcControlUpdateOutput(&controlOutput, RC_CONTROL_SOURCE_NAVIGATION);
}

/*-----------------------------------------------------------
 * Calculate loiter target based on current position and velocity
 *-----------------------------------------------------------*/
void calculateMulticopterInitialHoldPosition(fpVector3_t * pos)
{
    const float stoppingDistanceX = navGetCurrentActualPositionAndVelocity()->vel.x * posControl.posDecelerationTime;
    const float stoppingDistanceY = navGetCurrentActualPositionAndVelocity()->vel.y * posControl.posDecelerationTime;

    pos->x = navGetCurrentActualPositionAndVelocity()->pos.x + stoppingDistanceX;
    pos->y = navGetCurrentActualPositionAndVelocity()->pos.y + stoppingDistanceY;
}

void resetMulticopterHeadingController(void)
{
    updateHeadingHoldTarget(CENTIDEGREES_TO_DEGREES(posControl.actualState.yaw));
}

static void applyMulticopterHeadingController(void)
{
    updateHeadingHoldTarget(CENTIDEGREES_TO_DEGREES(posControl.desiredState.yaw));
}

void applyMulticopterNavigationController(navigationFSMStateFlags_t navStateFlags, timeUs_t currentTimeUs)
{
    if (navStateFlags & NAV_CTL_EMERG) {
        applyMulticopterEmergencyLandingController(currentTimeUs);
    }
    else {
        if (navStateFlags & NAV_CTL_ALT)
            applyMulticopterAltitudeController(currentTimeUs);

        if (navStateFlags & NAV_CTL_POS)
            applyMulticopterPositionController(currentTimeUs);

        if (navStateFlags & NAV_CTL_YAW)
            applyMulticopterHeadingController();
    }
}
#endif  // NAV
