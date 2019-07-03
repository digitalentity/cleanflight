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

#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"

#include "drivers/time.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/boardalignment.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/mixer.h"

#include "fc/config.h"
#include "fc/controlrate_profile.h"
#include "fc/rc_control.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "navigation/navigation.h"
#include "navigation/navigation_private.h"

#include "rx/rx.h"

// XXX: Whole file was using rcCommand to check for input, changed to actual inputs

// If we are going slower than NAV_FW_MIN_VEL_SPEED_BOOST - boost throttle to fight against the wind
#define NAV_FW_THROTTLE_SPEED_BOOST_GAIN        1.5f
#define NAV_FW_MIN_VEL_SPEED_BOOST              700.0f      // 7 m/s

#define NAV_FW_MANUAL_THR_INCREASE_THRESHOLD    0.95f       // 95%

// If this is enabled navigation won't be applied if velocity is below 3 m/s
//#define NAV_FW_LIMIT_MIN_FLY_VELOCITY

static bool isPitchAdjustmentValid = false;
static bool isRollAdjustmentValid = false;
static float throttleSpeedAdjustment = 0;
static bool isAutoThrottleManuallyIncreased = false;
static int32_t navHeadingError;


/*-----------------------------------------------------------
 * Altitude controller
 *-----------------------------------------------------------*/
void setupFixedWingAltitudeController(void)
{
    // TODO
}

void resetFixedWingAltitudeController(void)
{
    navPidReset(&posControl.pids.fw_alt);
    posControl.rcAdjustment[PITCH] = 0;
    isPitchAdjustmentValid = false;
    throttleSpeedAdjustment = 0;
}

bool adjustFixedWingAltitudeFromRCInput(void)
{
    float rcAdjustment = rcControlGetInputAxisApplyingPosholdDeadband(PITCH);

    if (rcAdjustment) {
        // set velocity proportional to stick movement
        float rcClimbRate = -rcAdjustment * navConfig()->general.max_manual_climb_rate;
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

// Position to velocity controller for Z axis
static void updateAltitudeVelocityAndPitchController_FW(timeDelta_t deltaMicros)
{
    static pt1Filter_t velzFilterState;

    // On a fixed wing we might not have a reliable climb rate source (if no BARO available), so we can't apply PID controller to
    // velocity error. We use PID controller on altitude error and calculate desired pitch angle

    // Update energies
    const float demSPE = (posControl.desiredState.pos.z / 100.0f) * GRAVITY_MSS;
    const float demSKE = 0.0f;

    const float estSPE = (navGetCurrentActualPositionAndVelocity()->pos.z / 100.0f) * GRAVITY_MSS;
    const float estSKE = 0.0f;

    // speedWeight controls balance between potential and kinetic energy used for pitch controller
    //  speedWeight = 1.0 : pitch will only control airspeed and won't control altitude
    //  speedWeight = 0.5 : pitch will be used to control both airspeed and altitude
    //  speedWeight = 0.0 : pitch will only control altitude
    const float speedWeight = 0.0f; // no speed sensing for now

    const float demSEB = demSPE * (1.0f - speedWeight) - demSKE * speedWeight;
    const float estSEB = estSPE * (1.0f - speedWeight) - estSKE * speedWeight;

    // SEB to pitch angle gain to account for airspeed (with respect to specified reference (tuning) speed
    const float pitchGainInv = 1.0f / 1.0f;

    // Here we use negative values for dive for better clarity
    const float maxClimbDeciDeg = DEGREES_TO_DECIDEGREES(navConfig()->fw.max_climb_angle);
    const float minDiveDeciDeg = -DEGREES_TO_DECIDEGREES(navConfig()->fw.max_dive_angle);

    // PID controller to translate energy balance error [J] into pitch angle [decideg]
    float targetPitchAngle = navPidApply3(&posControl.pids.fw_alt, demSEB, estSEB, US2S(deltaMicros), minDiveDeciDeg, maxClimbDeciDeg, 0, pitchGainInv);
    targetPitchAngle = pt1FilterApply4(&velzFilterState, targetPitchAngle, NAV_FW_PITCH_CUTOFF_FREQENCY_HZ, US2S(deltaMicros));

    // Reconstrain pitch angle ( >0 - climb, <0 - dive)
    targetPitchAngle = constrainf(targetPitchAngle, minDiveDeciDeg, maxClimbDeciDeg);
    posControl.rcAdjustment[PITCH] = targetPitchAngle;
}

void applyFixedWingAltitudeAndThrottleController(timeUs_t currentTimeUs)
{
    static timeUs_t previousTimePositionUpdate;         // Occurs @ altitude sensor update rate (max MAX_ALTITUDE_UPDATE_RATE_HZ)
    static timeUs_t previousTimeUpdate;                 // Occurs @ looptime rate

    const timeDelta_t deltaMicros = currentTimeUs - previousTimeUpdate;
    previousTimeUpdate = currentTimeUs;

    // If last time Z-controller was called is too far in the past - ignore it (likely restarting altitude controller)
    if (deltaMicros > HZ2US(MIN_POSITION_UPDATE_RATE_HZ)) {
        previousTimeUpdate = currentTimeUs;
        previousTimePositionUpdate = currentTimeUs;
        resetFixedWingAltitudeController();
        return;
    }

    if ((posControl.flags.estAltStatus >= EST_USABLE)) {
        if (posControl.flags.verticalPositionDataNew) {
            const timeDelta_t deltaMicrosPositionUpdate = currentTimeUs - previousTimePositionUpdate;
            previousTimePositionUpdate = currentTimeUs;

            // Check if last correction was too log ago - ignore this update
            if (deltaMicrosPositionUpdate < HZ2US(MIN_POSITION_UPDATE_RATE_HZ)) {
                updateAltitudeVelocityAndPitchController_FW(deltaMicrosPositionUpdate);
            }
            else {
                // due to some glitch position update has not occurred in time, reset altitude controller
                resetFixedWingAltitudeController();
            }

            // Indicate that information is no longer usable
            posControl.flags.verticalPositionDataConsumed = 1;
        }

        isPitchAdjustmentValid = true;
    }
    else {
        // No valid altitude sensor data, don't adjust pitch automatically, rcCommand[PITCH] is passed through to PID controller
        isPitchAdjustmentValid = false;
    }
}

/*-----------------------------------------------------------
 * Adjusts desired heading from pilot's input
 *-----------------------------------------------------------*/
bool adjustFixedWingHeadingFromRCInput(void)
{
    return rcControlGetInputAxisApplyingPosholdDeadband(YAW) != 0;
}

/*-----------------------------------------------------------
 * XY-position controller for multicopter aircraft
 *-----------------------------------------------------------*/
static fpVector3_t virtualDesiredPosition;
static pt1Filter_t fwPosControllerCorrectionFilterState;

void resetFixedWingPositionController(void)
{
    virtualDesiredPosition.x = 0;
    virtualDesiredPosition.y = 0;
    virtualDesiredPosition.z = 0;

    navPidReset(&posControl.pids.fw_nav);
    posControl.rcAdjustment[ROLL] = 0;
    isRollAdjustmentValid = false;

    pt1FilterReset(&fwPosControllerCorrectionFilterState, 0.0f);
}

// Returns 1 when loitering should turn to the right, -1
// when it should turn to the left.
static int8_t loiterDirection(void) {
    // Default to loitering to the right
    static int8_t loiterDirYaw = 1;
    int8_t dir = 1;
    switch (pidProfile()->loiter_direction) {
        case NAV_LOITER_LEFT:
            dir = -1;
            break;
        case NAV_LOITER_RIGHT:
        default:
            break;
        case NAV_LOITER_YAW:
        {
            // Require the stick to be either in the first 25% or the last 25% of the range
            float yaw = fapplyDeadbandf(rcControlGetInputAxis(YAW), RC_COMMAND_RANGE / 4.0f);
            if (yaw > 0) {
                // Turn to the right
                loiterDirYaw = 1;
            } else if (yaw < 0) {
                // Turn to the left
                loiterDirYaw = -1;
            }
            dir = loiterDirYaw;
            break;
        }
    }
    if (IS_RC_MODE_ACTIVE(BOXLOITERDIRCHN)) {
        dir *= -1;
    }
    return dir;
}

static void calculateVirtualPositionTarget_FW(float trackingPeriod)
{
    float posErrorX = posControl.desiredState.pos.x - navGetCurrentActualPositionAndVelocity()->pos.x;
    float posErrorY = posControl.desiredState.pos.y - navGetCurrentActualPositionAndVelocity()->pos.y;

    float distanceToActualTarget = sqrtf(sq(posErrorX) + sq(posErrorY));

    // Limit minimum forward velocity to 1 m/s
    float trackingDistance = trackingPeriod * MAX(posControl.actualState.velXY, 100.0f);

    // If angular visibility of a waypoint is less than 30deg, don't calculate circular loiter, go straight to the target
    #define TAN_15DEG    0.26795f
    bool needToCalculateCircularLoiter = isApproachingLastWaypoint()
                                            && (distanceToActualTarget <= (navConfig()->fw.loiter_radius / TAN_15DEG))
                                            && (distanceToActualTarget > 50.0f)
                                            && !FLIGHT_MODE(NAV_CRUISE_MODE);

    // Calculate virtual position for straight movement
    if (needToCalculateCircularLoiter) {
        // We are closing in on a waypoint, calculate circular loiter
        float loiterAngle = atan2_approx(-posErrorY, -posErrorX) + DEGREES_TO_RADIANS(loiterDirection() * 45.0f);

        float loiterTargetX = posControl.desiredState.pos.x + navConfig()->fw.loiter_radius * cos_approx(loiterAngle);
        float loiterTargetY = posControl.desiredState.pos.y + navConfig()->fw.loiter_radius * sin_approx(loiterAngle);

        // We have temporary loiter target. Recalculate distance and position error
        posErrorX = loiterTargetX - navGetCurrentActualPositionAndVelocity()->pos.x;
        posErrorY = loiterTargetY - navGetCurrentActualPositionAndVelocity()->pos.y;
        distanceToActualTarget = sqrtf(sq(posErrorX) + sq(posErrorY));
    }

    // Calculate virtual waypoint
    virtualDesiredPosition.x = navGetCurrentActualPositionAndVelocity()->pos.x + posErrorX * (trackingDistance / distanceToActualTarget);
    virtualDesiredPosition.y = navGetCurrentActualPositionAndVelocity()->pos.y + posErrorY * (trackingDistance / distanceToActualTarget);

    // Shift position according to pilot's ROLL input (up to max_manual_speed velocity)
    if (posControl.flags.isAdjustingPosition) {
        float rcRollAdjustment = rcControlGetInputAxisApplyingPosholdDeadband(ROLL);

        if (rcRollAdjustment) {
            float rcShiftY = rcRollAdjustment * navConfig()->general.max_manual_speed * trackingPeriod;

            // Rotate this target shift from body frame to to earth frame and apply to position target
            virtualDesiredPosition.x += -rcShiftY * posControl.actualState.sinYaw;
            virtualDesiredPosition.y +=  rcShiftY * posControl.actualState.cosYaw;
        }
    }
}

bool adjustFixedWingPositionFromRCInput(void)
{
    return rcControlGetInputAxisApplyingPosholdDeadband(ROLL) != 0;
}

static void updatePositionHeadingController_FW(timeUs_t currentTimeUs, timeDelta_t deltaMicros)
{
    static timeUs_t previousTimeMonitoringUpdate;
    static int32_t previousHeadingError;
    static bool errorIsDecreasing;
    static bool forceTurnDirection = false;

    // We have virtual position target, calculate heading error
    int32_t virtualTargetBearing = calculateBearingToDestination(&virtualDesiredPosition);

    // Calculate NAV heading error
    navHeadingError = wrap_18000(virtualTargetBearing - posControl.actualState.yaw);

    // Forced turn direction
    // If heading error is close to 180 deg we initiate forced turn and only disable it when heading error goes below 90 deg
    if (ABS(navHeadingError) > 17000) {
        forceTurnDirection = true;
    }
    else if (ABS(navHeadingError) < 9000 && forceTurnDirection) {
        forceTurnDirection = false;
    }

    // If forced turn direction flag is enabled we fix the sign of the direction
    if (forceTurnDirection) {
        navHeadingError = loiterDirection() * ABS(navHeadingError);
    }

    // Slow error monitoring (2Hz rate)
    if ((currentTimeUs - previousTimeMonitoringUpdate) >= HZ2US(NAV_FW_CONTROL_MONITORING_RATE)) {
        // Check if error is decreasing over time
        errorIsDecreasing = (ABS(previousHeadingError) > ABS(navHeadingError));

        // Save values for next iteration
        previousHeadingError = navHeadingError;
        previousTimeMonitoringUpdate = currentTimeUs;
    }

    // Only allow PID integrator to shrink if error is decreasing over time
    const pidControllerFlags_e pidFlags = PID_DTERM_FROM_ERROR | (errorIsDecreasing ? PID_SHRINK_INTEGRATOR : 0);

    // Input error in (deg*100), output roll angle (deg*100)
    float rollAdjustment = navPidApply2(&posControl.pids.fw_nav, posControl.actualState.yaw + navHeadingError, posControl.actualState.yaw, US2S(deltaMicros),
                                       -DEGREES_TO_CENTIDEGREES(navConfig()->fw.max_bank_angle),
                                        DEGREES_TO_CENTIDEGREES(navConfig()->fw.max_bank_angle),
                                        pidFlags);

    // Apply low-pass filter to prevent rapid correction
    rollAdjustment = pt1FilterApply4(&fwPosControllerCorrectionFilterState, rollAdjustment, NAV_FW_ROLL_CUTOFF_FREQUENCY_HZ, US2S(deltaMicros));

    // Convert rollAdjustment to decidegrees (rcAdjustment holds decidegrees)
    posControl.rcAdjustment[ROLL] = CENTIDEGREES_TO_DECIDEGREES(rollAdjustment);
}

void applyFixedWingPositionController(timeUs_t currentTimeUs)
{
    static timeUs_t previousTimePositionUpdate;         // Occurs @ GPS update rate
    static timeUs_t previousTimeUpdate;                 // Occurs @ looptime rate

    const timeDelta_t deltaMicros = currentTimeUs - previousTimeUpdate;
    previousTimeUpdate = currentTimeUs;

    // If last position update was too long in the past - ignore it (likely restarting altitude controller)
    if (deltaMicros > HZ2US(MIN_POSITION_UPDATE_RATE_HZ)) {
        previousTimeUpdate = currentTimeUs;
        previousTimePositionUpdate = currentTimeUs;
        resetFixedWingPositionController();
        return;
    }

    // Apply controller only if position source is valid. In absence of valid pos sensor (GPS loss), we'd stick in forced ANGLE mode
    if ((posControl.flags.estPosStatus >= EST_USABLE)) {
        // If we have new position - update velocity and acceleration controllers
        if (posControl.flags.horizontalPositionDataNew) {
            const timeDelta_t deltaMicrosPositionUpdate = currentTimeUs - previousTimePositionUpdate;
            previousTimePositionUpdate = currentTimeUs;

            if (deltaMicrosPositionUpdate < HZ2US(MIN_POSITION_UPDATE_RATE_HZ)) {
                // Calculate virtual position target at a distance of forwardVelocity * HZ2S(POSITION_TARGET_UPDATE_RATE_HZ)
                // Account for pilot's roll input (move position target left/right at max of max_manual_speed)
                // POSITION_TARGET_UPDATE_RATE_HZ should be chosen keeping in mind that position target shouldn't be reached until next pos update occurs
                // FIXME: verify the above
                calculateVirtualPositionTarget_FW(HZ2S(MIN_POSITION_UPDATE_RATE_HZ) * 2);

                updatePositionHeadingController_FW(currentTimeUs, deltaMicrosPositionUpdate);
            }
            else {
                resetFixedWingPositionController();
            }

            // Indicate that information is no longer usable
            posControl.flags.horizontalPositionDataConsumed = 1;
        }

        isRollAdjustmentValid = true;
    }
    else {
        // No valid pos sensor data, don't adjust pitch automatically, rcCommand[ROLL] is passed through to PID controller
        isRollAdjustmentValid = false;
    }
}

int16_t applyFixedWingMinSpeedController(timeUs_t currentTimeUs)
{
    static timeUs_t previousTimePositionUpdate;         // Occurs @ GPS update rate
    static timeUs_t previousTimeUpdate;                 // Occurs @ looptime rate

    const timeDelta_t deltaMicros = currentTimeUs - previousTimeUpdate;
    previousTimeUpdate = currentTimeUs;

    // If last position update was too long in the past - ignore it (likely restarting altitude controller)
    if (deltaMicros > HZ2US(MIN_POSITION_UPDATE_RATE_HZ)) {
        previousTimeUpdate = currentTimeUs;
        previousTimePositionUpdate = currentTimeUs;
        throttleSpeedAdjustment = 0;
        return 0;
    }

    // Apply controller only if position source is valid
    if ((posControl.flags.estPosStatus >= EST_USABLE)) {
        // If we have new position - update velocity and acceleration controllers
        if (posControl.flags.horizontalPositionDataNew) {
            const timeDelta_t deltaMicrosPositionUpdate = currentTimeUs - previousTimePositionUpdate;
            previousTimePositionUpdate = currentTimeUs;

            if (deltaMicrosPositionUpdate < HZ2US(MIN_POSITION_UPDATE_RATE_HZ)) {
                float velThrottleBoost = (NAV_FW_MIN_VEL_SPEED_BOOST - posControl.actualState.velXY) * NAV_FW_THROTTLE_SPEED_BOOST_GAIN * US2S(deltaMicrosPositionUpdate);

                // If we are in the deadband of 50cm/s - don't update speed boost
                if (fabsf(posControl.actualState.velXY - NAV_FW_MIN_VEL_SPEED_BOOST) > 50) {
                    throttleSpeedAdjustment += velThrottleBoost;
                }

                throttleSpeedAdjustment = constrainf(throttleSpeedAdjustment, 0.0f, 500.0f);
            }
            else {
                throttleSpeedAdjustment = 0;
            }

            // Indicate that information is no longer usable
            posControl.flags.horizontalPositionDataConsumed = 1;
        }
    }
    else {
        // No valid pos sensor data, we can't calculate speed
        throttleSpeedAdjustment = 0;
    }

    return throttleSpeedAdjustment;
}

int16_t fixedWingPitchToThrottleCorrection(int16_t pitch)
{
    return DECIDEGREES_TO_DEGREES(pitch) * navConfig()->fw.pitch_to_throttle;
}

void applyFixedWingPitchRollThrottleController(navigationFSMStateFlags_t navStateFlags, timeUs_t currentTimeUs)
{
    rcCommand_t controlOutput;

    rcCommandCopy(&controlOutput, rcControlGetOutput());

    if (isRollAdjustmentValid && (navStateFlags & NAV_CTL_POS)) {
        // ROLL >0 right, <0 left
        int16_t rollCorrection = constrain(posControl.rcAdjustment[ROLL], -DEGREES_TO_DECIDEGREES(navConfig()->fw.max_bank_angle), DEGREES_TO_DECIDEGREES(navConfig()->fw.max_bank_angle));
        controlOutput.roll = pidAngleToRcCommand(rollCorrection, pidProfile()->max_angle_inclination[FD_ROLL]);
    }

    if (isPitchAdjustmentValid && (navStateFlags & NAV_CTL_ALT)) {
        // PITCH >0 dive, <0 climb
        int16_t pitchCorrection = constrain(posControl.rcAdjustment[PITCH], -DEGREES_TO_DECIDEGREES(navConfig()->fw.max_dive_angle), DEGREES_TO_DECIDEGREES(navConfig()->fw.max_climb_angle));
        controlOutput.pitch = -pidAngleToRcCommand(pitchCorrection, pidProfile()->max_angle_inclination[FD_PITCH]);
        int16_t throttleCorrection = fixedWingPitchToThrottleCorrection(pitchCorrection);

        int16_t minThrottleCorrection = navConfig()->fw.min_throttle - navConfig()->fw.cruise_throttle;
        int16_t maxThrottleCorrection = navConfig()->fw.max_throttle - navConfig()->fw.cruise_throttle;

#ifdef NAV_FIXED_WING_LANDING
        if (navStateFlags & NAV_CTL_LAND) {
            // During LAND we do not allow to raise THROTTLE when nose is up to reduce speed
            throttleCorrection = constrain(throttleCorrection, minThrottleCorrection, 0);
        } else {
#endif
            throttleCorrection = constrain(throttleCorrection, minThrottleCorrection, maxThrottleCorrection);
#ifdef NAV_FIXED_WING_LANDING
        }
#endif

        // Speed controller - only apply in POS mode when NOT NAV_CTL_LAND
        if ((navStateFlags & NAV_CTL_POS) && !(navStateFlags & NAV_CTL_LAND)) {
            throttleCorrection += applyFixedWingMinSpeedController(currentTimeUs);
            throttleCorrection = constrain(throttleCorrection, minThrottleCorrection, maxThrottleCorrection);
        }

        float correctedThrottleValue = rcCommandMapUnidirectionalPWMThrottle(constrain(navConfig()->fw.cruise_throttle + throttleCorrection, navConfig()->fw.min_throttle, navConfig()->fw.max_throttle));
        float cruiseThr = rcCommandMapUnidirectionalPWMThrottle(navConfig()->fw.cruise_throttle);

        // Manual throttle increase
        if (navConfig()->fw.allow_manual_thr_increase && !FLIGHT_MODE(FAILSAFE_MODE)) {
            // Check if THR is >= NAV_FW_MANUAL_THR_INCREASE_THRESHOLD
            if (rcControlGetInputAxis(THROTTLE) >= RC_COMMAND_MAX * NAV_FW_MANUAL_THR_INCREASE_THRESHOLD) {
                correctedThrottleValue = RC_COMMAND_MAX;
            } else {
                correctedThrottleValue += MAX(0, correctedThrottleValue - cruiseThr);
            }
            isAutoThrottleManuallyIncreased = correctedThrottleValue > cruiseThr;
        } else {
            isAutoThrottleManuallyIncreased = false;
        }

        // Constrain throttle to zero or positive values
        controlOutput.throttle = constrain(correctedThrottleValue, RC_COMMAND_CENTER, RC_COMMAND_MAX);
    }

#ifdef NAV_FIXED_WING_LANDING
    /*
     * Then altitude is below landing slowdown min. altitude, enable final approach procedure
     * TODO refactor conditions in this metod if logic is proven to be correct
     */
    if (navStateFlags & NAV_CTL_LAND) {
        if ( ((posControl.flags.estAltStatus >= EST_USABLE) && (navGetCurrentActualPositionAndVelocity()->pos.z <= navConfig()->general.land_slowdown_minalt)) ||
             ((posControl.flags.estAglStatus == EST_TRUSTED) && (posControl.actualState.agl.pos.z <= navConfig()->general.land_slowdown_minalt)) ) {

            // Set motor to min. throttle and stop it when MOTOR_STOP feature is enabled
            // XXX: Should we deprecate the NAV_MOTOR_STOP_OR_IDLE flag?
            controlOutput.throttle = 0;
            ENABLE_STATE(NAV_MOTOR_STOP_OR_IDLE);

            // Stabilize ROLL axis on 0 degrees banking regardless of loiter radius and position
            controlOutput.roll = 0;

            // Stabilize PITCH angle into shallow dive as specified by the nav_fw_land_dive_angle setting (default value is 2 - defined in navigation.c).
            controlOutput.pitch = pidAngleToRcCommand(DEGREES_TO_DECIDEGREES(navConfig()->fw.land_dive_angle), pidProfile()->max_angle_inclination[FD_PITCH]);
        }
    }
#endif
    rcControlUpdateOutput(&controlOutput, RC_CONTROL_SOURCE_NAVIGATION);
}

bool isFixedWingAutoThrottleManuallyIncreased()
{
    return isAutoThrottleManuallyIncreased;
}

/*-----------------------------------------------------------
 * FixedWing land detector
 *-----------------------------------------------------------*/
static timeUs_t landingTimerUs;

void resetFixedWingLandingDetector(void)
{
    landingTimerUs = micros();
}

bool isFixedWingLandingDetected(void)
{
    timeUs_t currentTimeUs = micros();

    landingTimerUs = currentTimeUs;
    return false;
}

/*-----------------------------------------------------------
 * FixedWing emergency landing
 *-----------------------------------------------------------*/
void applyFixedWingEmergencyLandingController(void)
{
    // FIXME: Use altitude controller if available (similar to MC code)
    rcCommand_t controlOutput;

    rcCommandCopy(&controlOutput, rcControlGetOutput());
    controlOutput.roll = pidAngleToRcCommand(failsafeConfig()->failsafe_fw_roll_angle, pidProfile()->max_angle_inclination[FD_ROLL]);
    controlOutput.pitch = pidAngleToRcCommand(failsafeConfig()->failsafe_fw_pitch_angle, pidProfile()->max_angle_inclination[FD_PITCH]);
    controlOutput.yaw = -pidRateToRcCommand(failsafeConfig()->failsafe_fw_yaw_rate, currentControlRateProfile->stabilized.rates[FD_YAW]);
    controlOutput.throttle = rcCommandMapUnidirectionalPWMThrottle(failsafeConfig()->failsafe_throttle);
    rcControlUpdateOutput(&controlOutput, RC_CONTROL_SOURCE_NAVIGATION);
}

/*-----------------------------------------------------------
 * Calculate loiter target based on current position and velocity
 *-----------------------------------------------------------*/
void calculateFixedWingInitialHoldPosition(fpVector3_t * pos)
{
    // TODO: stub, this should account for velocity and target loiter radius
    *pos = navGetCurrentActualPositionAndVelocity()->pos;
}

void resetFixedWingHeadingController(void)
{
    updateHeadingHoldTarget(CENTIDEGREES_TO_DEGREES(posControl.actualState.yaw));
}

void applyFixedWingNavigationController(navigationFSMStateFlags_t navStateFlags, timeUs_t currentTimeUs)
{
    rcCommand_t controlOutput;

    if (navStateFlags & NAV_CTL_LAUNCH) {
        applyFixedWingLaunchController(currentTimeUs);
    }
    else if (navStateFlags & NAV_CTL_EMERG) {
        applyFixedWingEmergencyLandingController();
    }
    else {
#ifdef NAV_FW_LIMIT_MIN_FLY_VELOCITY
        // Don't apply anything if ground speed is too low (<3m/s)
        if (posControl.actualState.velXY > 300) {
#else
        if (true) {
#endif
            if (navStateFlags & NAV_CTL_ALT) {
                if (getMotorStatus() == MOTOR_STOPPED_USER) {
                    // Motor has been stopped by user. Update target altitude and bypass navigation pitch/throttle control
                    resetFixedWingAltitudeController();
                    setDesiredPosition(&navGetCurrentActualPositionAndVelocity()->pos, posControl.actualState.yaw, NAV_POS_UPDATE_Z);
                } else
                    applyFixedWingAltitudeAndThrottleController(currentTimeUs);
            }

            if (navStateFlags & NAV_CTL_POS)
                applyFixedWingPositionController(currentTimeUs);

        } else {
            posControl.rcAdjustment[PITCH] = 0;
            posControl.rcAdjustment[ROLL] = 0;
        }

        if (FLIGHT_MODE(NAV_CRUISE_MODE) && posControl.flags.isAdjustingPosition) {
            rcCommandCopy(&controlOutput, rcControlGetOutput());
            controlOutput.roll = rcControlGetInputAxisApplyingPosholdDeadband(ROLL);
            rcControlUpdateOutput(&controlOutput, RC_CONTROL_SOURCE_NAVIGATION);
        }

        //if (navStateFlags & NAV_CTL_YAW)
        if ((navStateFlags & NAV_CTL_ALT) || (navStateFlags & NAV_CTL_POS))
            applyFixedWingPitchRollThrottleController(navStateFlags, currentTimeUs);
    }
}

int32_t navigationGetHeadingError(void)
{
    return navHeadingError;
}

#endif  // NAV
