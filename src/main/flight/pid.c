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

#include <platform.h>

#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"
#include "common/utils.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "fc/config.h"
#include "fc/controlrate_profile.h"
#include "fc/rc_command.h"
#include "fc/rc_control.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/mixer.h"

#include "io/gps.h"

#include "navigation/navigation.h"

#include "rx/rx.h"

#include "sensors/sensors.h"
#include "sensors/gyro.h"
#include "sensors/acceleration.h"
#include "sensors/compass.h"
#include "sensors/pitotmeter.h"


typedef struct {
    float kP;   // Proportional gain
    float kI;   // Integral gain
    float kD;   // Derivative gain
    float kFF;  // Feed-forward gain
    float kT;   // Back-calculation tracking gain

    float gyroRate;
    float rateTarget;

    // Buffer for derivative calculation
#define PID_GYRO_RATE_BUF_LENGTH 5
    float gyroRateBuf[PID_GYRO_RATE_BUF_LENGTH];
    firFilter_t gyroRateFilter;

    // Rate integrator
    float errorGyroIf;
    float errorGyroIfLimit;

    // Used for ANGLE filtering (PT1, we don't need super-sharpness here)
    pt1Filter_t angleFilterState;

    // Rate filtering
    rateLimitFilter_t axisAccelFilter;
    pt1Filter_t ptermLpfState;
    biquadFilter_t deltaLpfState;

    // Dterm notch filtering
#ifdef USE_DTERM_NOTCH
    biquadFilter_t deltaNotchFilter;
#endif
    float stickPosition;

#ifdef USE_D_BOOST
    float previousRateTarget;
    float previousRateGyro;
    pt1Filter_t dBoostLpf;
    biquadFilter_t dBoostGyroLpf;
#endif
} pidState_t;

#ifdef USE_DTERM_NOTCH
STATIC_FASTRAM filterApplyFnPtr notchFilterApplyFn;
#endif

extern float dT;

STATIC_FASTRAM bool pidFiltersConfigured = false;
FASTRAM float headingHoldCosZLimit;
FASTRAM int16_t headingHoldTarget;
STATIC_FASTRAM pt1Filter_t headingHoldRateFilter;
STATIC_FASTRAM pt1Filter_t fixedWingTpaFilter;

// Thrust PID Attenuation factor. 0.0f means fully attenuated, 1.0f no attenuation is applied
STATIC_FASTRAM bool pidGainsUpdateRequired;
FASTRAM int16_t axisPID[FLIGHT_DYNAMICS_INDEX_COUNT];

#ifdef USE_BLACKBOX
int32_t axisPID_P[FLIGHT_DYNAMICS_INDEX_COUNT], axisPID_I[FLIGHT_DYNAMICS_INDEX_COUNT], axisPID_D[FLIGHT_DYNAMICS_INDEX_COUNT], axisPID_Setpoint[FLIGHT_DYNAMICS_INDEX_COUNT];
#endif

STATIC_FASTRAM pidState_t pidState[FLIGHT_DYNAMICS_INDEX_COUNT];

static EXTENDED_FASTRAM pt1Filter_t windupLpf[XYZ_AXIS_COUNT];
static EXTENDED_FASTRAM uint8_t itermRelax;
static EXTENDED_FASTRAM uint8_t itermRelaxType;
static EXTENDED_FASTRAM float itermRelaxSetpointThreshold;

#define D_BOOST_GYRO_LPF_HZ 80    // Biquad lowpass input cutoff to peak D around propwash frequencies
#define D_BOOST_LPF_HZ 10         // PT1 lowpass cutoff to smooth the boost effect

#ifdef USE_D_BOOST
static EXTENDED_FASTRAM float dBoostFactor;
static EXTENDED_FASTRAM float dBoostMaxAtAlleceleration;
#endif

PG_REGISTER_PROFILE_WITH_RESET_TEMPLATE(pidProfile_t, pidProfile, PG_PID_PROFILE, 9);

PG_RESET_TEMPLATE(pidProfile_t, pidProfile,
        .bank_mc = {
            .pid = {
                [PID_ROLL] =    { 40, 30, 23, 0 },
                [PID_PITCH] =   { 40, 30, 23, 0 },
                [PID_YAW] =     { 85, 45, 0, 0 },
                [PID_LEVEL] = {
                    .P = 20,    // Self-level strength
                    .I = 15,    // Self-leveing low-pass frequency (0 - disabled)
                    .D = 75,    // 75% horizon strength
                    .FF = 0,
                },
                [PID_HEADING] = { 60, 0, 0, 0 },
                [PID_POS_XY] = {
                    .P = 65,   // NAV_POS_XY_P * 100
                    .I = 0,
                    .D = 0,
                    .FF = 0,
                },
                [PID_VEL_XY] = {
                    .P = 40,   // NAV_VEL_XY_P * 20
                    .I = 15,   // NAV_VEL_XY_I * 100
                    .D = 100,  // NAV_VEL_XY_D * 100
                    .FF = 40,  // NAV_VEL_XY_D * 100
                },
                [PID_POS_Z] = {
                    .P = 50,    // NAV_POS_Z_P * 100
                    .I = 0,     // not used
                    .D = 0,     // not used
                    .FF = 0,
                },
                [PID_VEL_Z] = {
                    .P = 100,   // NAV_VEL_Z_P * 66.7
                    .I = 50,    // NAV_VEL_Z_I * 20
                    .D = 10,    // NAV_VEL_Z_D * 100
                    .FF = 0,
                }
            }
        },

        .bank_fw = {
            .pid = {
                [PID_ROLL] =    { 5, 7, 0, 50 },
                [PID_PITCH] =   { 5, 7, 0, 50 },
                [PID_YAW] =     { 6, 10, 0, 60 },
                [PID_LEVEL] = {
                    .P = 20,    // Self-level strength
                    .I = 5,     // Self-leveing low-pass frequency (0 - disabled)
                    .D = 75,    // 75% horizon strength
                    .FF = 0,
                },
                [PID_HEADING] = { 60, 0, 0, 0 },
                [PID_POS_Z] = {
                    .P = 40,    // FW_POS_Z_P * 10
                    .I = 5,     // FW_POS_Z_I * 10
                    .D = 10,    // FW_POS_Z_D * 10
                    .FF = 0,
                },
                [PID_POS_XY] = {
                    .P = 75,    // FW_POS_XY_P * 100
                    .I = 5,     // FW_POS_XY_I * 100
                    .D = 8,     // FW_POS_XY_D * 100
                    .FF = 0,
                }
            }
        },

        .dterm_soft_notch_hz = 0,
        .dterm_soft_notch_cutoff = 1,
        .dterm_lpf_hz = 40,
        .yaw_lpf_hz = 30,
        .dterm_setpoint_weight = 1.0f,
        .use_dterm_fir_filter = 1,

        .itermWindupPointPercent = 50,       // Percent

        .axisAccelerationLimitYaw = 10000,       // dps/s
        .axisAccelerationLimitRollPitch = 0,     // dps/s

        .yaw_p_limit = YAW_P_LIMIT_DEFAULT,

        .heading_hold_rate_limit = HEADING_HOLD_RATE_LIMIT_DEFAULT,

        .max_angle_inclination[FD_ROLL] = 300,    // 30 degrees
        .max_angle_inclination[FD_PITCH] = 300,    // 30 degrees
        .pidSumLimit = PID_SUM_LIMIT_DEFAULT,

        .fixedWingItermThrowLimit = FW_ITERM_THROW_LIMIT_DEFAULT,
        .fixedWingReferenceAirspeed = 1000,
        .fixedWingCoordinatedYawGain = 1.0f,
        .fixedWingItermLimitOnStickPosition = 0.5f,

        .loiter_direction = NAV_LOITER_RIGHT,
        .navVelXyDTermLpfHz = NAV_ACCEL_CUTOFF_FREQUENCY_HZ,
        .iterm_relax_type = ITERM_RELAX_SETPOINT,
        .iterm_relax_cutoff = MC_ITERM_RELAX_CUTOFF_DEFAULT,
        .iterm_relax = ITERM_RELAX_OFF,
        .dBoostFactor = 1.0f,
        .dBoostMaxAtAlleceleration = 7500.0f,
        .dBoostGyroDeltaLpfHz = D_BOOST_GYRO_LPF_HZ,
);

void pidInit(void)
{
    pidResetTPAFilter();

    // Calculate max overall tilt (max pitch + max roll combined) as a limit to heading hold
    headingHoldCosZLimit = cos_approx(DECIDEGREES_TO_RADIANS(pidProfile()->max_angle_inclination[FD_ROLL])) *
                           cos_approx(DECIDEGREES_TO_RADIANS(pidProfile()->max_angle_inclination[FD_PITCH]));

    pidGainsUpdateRequired = false;

    itermRelax = pidProfile()->iterm_relax;
    itermRelaxType = pidProfile()->iterm_relax_type;
    itermRelaxSetpointThreshold = MC_ITERM_RELAX_SETPOINT_THRESHOLD * MC_ITERM_RELAX_CUTOFF_DEFAULT / pidProfile()->iterm_relax_cutoff;

#ifdef USE_D_BOOST
    dBoostFactor = pidProfile()->dBoostFactor;
    dBoostMaxAtAlleceleration = pidProfile()->dBoostMaxAtAlleceleration;
#endif

}

bool pidInitFilters(void)
{
    const uint32_t refreshRate = getLooptime();

    if (refreshRate == 0) {
        return false;
    }

    static float dtermCoeffs[PID_GYRO_RATE_BUF_LENGTH];

    if (pidProfile()->use_dterm_fir_filter) {
        // Calculate derivative using 5-point noise-robust differentiators without time delay (one-sided or forward filters)
        // by Pavel Holoborodko, see http://www.holoborodko.com/pavel/numerical-methods/numerical-derivative/smooth-low-noise-differentiators/
        // h[0] = 5/8, h[-1] = 1/4, h[-2] = -1, h[-3] = -1/4, h[-4] = 3/8
        dtermCoeffs[0] = 5.0f/8;
        dtermCoeffs[1] = 2.0f/8;
        dtermCoeffs[2] = -8.0f/8;
        dtermCoeffs[3] = -2.0f/8;
        dtermCoeffs[4] = 3.0f/8;
    } else {
        //simple d(t) - d(t-1) differentiator 
        dtermCoeffs[0] = 1.0f;
        dtermCoeffs[1] = -1.0f;
        dtermCoeffs[2] = 0.0f;
        dtermCoeffs[3] = 0.0f;
        dtermCoeffs[4] = 0.0f;
    }

    for (int axis = 0; axis < 3; ++ axis) {
        firFilterInit(&pidState[axis].gyroRateFilter, pidState[axis].gyroRateBuf, PID_GYRO_RATE_BUF_LENGTH, dtermCoeffs);
    }

#ifdef USE_DTERM_NOTCH
    notchFilterApplyFn = nullFilterApply;
    if (pidProfile()->dterm_soft_notch_hz != 0) {
        notchFilterApplyFn = (filterApplyFnPtr)biquadFilterApply;
        for (int axis = 0; axis < 3; ++ axis) {
            biquadFilterInitNotch(&pidState[axis].deltaNotchFilter, refreshRate, pidProfile()->dterm_soft_notch_hz, pidProfile()->dterm_soft_notch_cutoff);
        }
    }
#endif

    // Init other filters
    for (int axis = 0; axis < 3; ++ axis) {
        biquadFilterInitLPF(&pidState[axis].deltaLpfState, pidProfile()->dterm_lpf_hz, refreshRate);
    }

    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
        pt1FilterInit(&windupLpf[i], pidProfile()->iterm_relax_cutoff, refreshRate * 1e-6f);
    }

#ifdef USE_D_BOOST
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        biquadFilterInitLPF(&pidState[axis].dBoostGyroLpf, pidProfile()->dBoostGyroDeltaLpfHz, getLooptime());
    }
#endif

    pidFiltersConfigured = true;

    return true;
}

void pidResetTPAFilter(void)
{
    if (STATE(FIXED_WING) && currentControlRateProfile->throttle.fixedWingTauMs > 0) {
        pt1FilterInitRC(&fixedWingTpaFilter, currentControlRateProfile->throttle.fixedWingTauMs * 1e-3f, getLooptime() * 1e-6f);
        pt1FilterReset(&fixedWingTpaFilter, motorConfig()->minthrottle);
    }
}

void pidResetErrorAccumulators(void)
{
    // Reset R/P/Y integrator
    for (int axis = 0; axis < 3; axis++) {
        pidState[axis].errorGyroIf = 0.0f;
        pidState[axis].errorGyroIfLimit = 0.0f;
    }
}

static float pidRcCommandToAngle(float stick, int16_t maxInclination)
{
    // No need to constrain here, it's already constrained by
    // the input system
    return scaleRangef(stick, RC_COMMAND_MIN, RC_COMMAND_MAX, (float) -maxInclination, (float) maxInclination);
}

float pidAngleToRcCommand(float angleDeciDegrees, int16_t maxInclination)
{
    angleDeciDegrees = constrainf(angleDeciDegrees, (float) -maxInclination, (float) maxInclination);
    return scaleRangef((float) angleDeciDegrees, (float) -maxInclination, (float) maxInclination, RC_COMMAND_MIN, RC_COMMAND_MAX);
}

/*
Map stick positions to desired rotatrion rate in given axis.
Rotation rate in dps at full stick deflection is defined by axis rate measured in dps/10
Rate 20 means 200dps at full stick deflection
*/
float pidRateToRcCommand(float rateDPS, uint8_t rate)
{
    const float maxRateDPS = rate * 10.0f;
    return scaleRangef(rateDPS, -maxRateDPS, maxRateDPS, RC_COMMAND_MIN, RC_COMMAND_MAX);
}

static float pidRcCommandToRate(float stick, uint8_t rate)
{
    const float maxRateDPS = rate * 10.0f;
    return scaleRangef(stick, RC_COMMAND_MIN, RC_COMMAND_MAX, -maxRateDPS, maxRateDPS);
}

static float calculateFixedWingTPAFactor(uint16_t throttle)
{
    float tpaFactor;

    // tpa_rate is amount of curve TPA applied to PIDs
    // tpa_breakpoint for fixed wing is cruise throttle value (value at which PIDs were tuned)
    if (currentControlRateProfile->throttle.dynPID != 0 && currentControlRateProfile->throttle.pa_breakpoint > motorConfig()->minthrottle) {
        if (throttle > motorConfig()->minthrottle) {
            // Calculate TPA according to throttle
            tpaFactor = 0.5f + ((float)(currentControlRateProfile->throttle.pa_breakpoint - motorConfig()->minthrottle) / (throttle - motorConfig()->minthrottle) / 2.0f);

            // Limit to [0.5; 2] range
            tpaFactor = constrainf(tpaFactor, 0.5f, 2.0f);
        }
        else {
            tpaFactor = 2.0f;
        }

        // Attenuate TPA curve according to configured amount
        tpaFactor = 1.0f + (tpaFactor - 1.0f) * (currentControlRateProfile->throttle.dynPID / 100.0f);
    }
    else {
        tpaFactor = 1.0f;
    }

    return tpaFactor;
}

static float calculateMultirotorTPAFactor(void)
{
    float tpaFactor;

    // TPA should be updated only when TPA is actually set
    float thr = fabsf(rcControlGetOutput()->throttle);
    float paBreakpoint = rcCommandMapUnidirectionalPWMThrottle(currentControlRateProfile->throttle.pa_breakpoint);
    if (currentControlRateProfile->throttle.dynPID == 0 || thr < paBreakpoint) {
        tpaFactor = 1.0f;
    } else if (thr < RC_COMMAND_MAX) {
        tpaFactor = (100 - (uint16_t)currentControlRateProfile->throttle.dynPID * (thr - paBreakpoint) / (RC_COMMAND_MAX - paBreakpoint)) / 100.0f;
    } else {
        tpaFactor = (100 - currentControlRateProfile->throttle.dynPID) / 100.0f;
    }

    return tpaFactor;
}

void schedulePidGainsUpdate(void)
{
    pidGainsUpdateRequired = true;
}

void FAST_CODE NOINLINE updatePIDCoefficients(void)
{
    STATIC_FASTRAM float prevThrottle = 0;

    const rcCommand_t *controlOutput = rcControlGetOutput();
    float thr = controlOutput->throttle;
    // Check if throttle changed. Different logic for fixed wing vs multirotor
    if (STATE(FIXED_WING) && (currentControlRateProfile->throttle.fixedWingTauMs > 0)) {
        float filteredThrottle = pt1FilterApply3(&fixedWingTpaFilter, thr, dT);
        if (filteredThrottle != prevThrottle) {
            prevThrottle = filteredThrottle;
            pidGainsUpdateRequired = true;
        }
    }
    else {
        if (thr != prevThrottle) {
            prevThrottle = thr;
            pidGainsUpdateRequired = true;
        }
    }

    /*
     * Compute stick position in range of [-1.0f : 1.0f] without deadband and expo
     */
    for (int axis = ROLL; axis < THROTTLE; axis++) {
        pidState[axis].stickPosition = controlOutput->axes[axis];
    }

    // If nothing changed - don't waste time recalculating coefficients
    if (!pidGainsUpdateRequired) {
        return;
    }

    const float tpaFactor = STATE(FIXED_WING) ? calculateFixedWingTPAFactor(prevThrottle) : calculateMultirotorTPAFactor();

    // PID coefficients can be update only with THROTTLE and TPA or inflight PID adjustments
    //TODO: Next step would be to update those only at THROTTLE or inflight adjustments change
    for (int axis = 0; axis < 3; axis++) {
        if (STATE(FIXED_WING)) {
            // Airplanes - scale all PIDs according to TPA
            pidState[axis].kP  = pidBank()->pid[axis].P / FP_PID_RATE_P_MULTIPLIER  * tpaFactor;
            pidState[axis].kI  = pidBank()->pid[axis].I / FP_PID_RATE_I_MULTIPLIER  * tpaFactor;
            pidState[axis].kD  = 0.0f;
            pidState[axis].kFF = pidBank()->pid[axis].FF / FP_PID_RATE_FF_MULTIPLIER * tpaFactor;
            pidState[axis].kT  = 0.0f;
        }
        else {
            const float axisTPA = (axis == FD_YAW) ? 1.0f : tpaFactor;
            pidState[axis].kP  = pidBank()->pid[axis].P / FP_PID_RATE_P_MULTIPLIER * axisTPA;
            pidState[axis].kI  = pidBank()->pid[axis].I / FP_PID_RATE_I_MULTIPLIER;
            pidState[axis].kD  = pidBank()->pid[axis].D / FP_PID_RATE_D_MULTIPLIER * axisTPA;
            pidState[axis].kFF = 0.0f;

            // Tracking anti-windup requires P/I/D to be all defined which is only true for MC
            if ((pidBank()->pid[axis].P != 0) && (pidBank()->pid[axis].I != 0)) {
                pidState[axis].kT = 2.0f / ((pidState[axis].kP / pidState[axis].kI) + (pidState[axis].kD / pidState[axis].kP));
            } else {
                pidState[axis].kT = 0;
            }
        }
    }

    pidGainsUpdateRequired = false;
}

static float calcHorizonRateMagnitude(void)
{
    // Figure out the raw stick positions
    const rcCommand_t *input = rcControlGetInput();
    const float pitchDeflection = fabsf(input->pitch);

    const float rollDeflection = fabsf(input->roll);
    const float mostDeflectedStickPos = MAX(pitchDeflection, rollDeflection);
    const float modeTransitionStickPos = constrain(pidBank()->pid[PID_LEVEL].D, 0, 100) / 100.0f;

    float horizonRateMagnitude;

    // Calculate transition point according to stick deflection
    if (mostDeflectedStickPos <= modeTransitionStickPos) {
        horizonRateMagnitude = mostDeflectedStickPos / modeTransitionStickPos;
    }
    else {
        horizonRateMagnitude = 1.0f;
    }

    return horizonRateMagnitude;
}

static void pidLevel(pidState_t *pidState, flight_dynamics_index_t axis, float horizonRateMagnitude)
{
    const rcCommand_t *controlOutput = rcControlGetOutput();
    // This is ROLL/PITCH, run ANGLE/HORIZON controllers
    float angleTarget = pidRcCommandToAngle(controlOutput->axes[axis], pidProfile()->max_angle_inclination[axis]);

    // Automatically pitch down if the throttle is manually controlled and reduced bellow cruise throttle
    if ((axis == FD_PITCH) && STATE(FIXED_WING) && FLIGHT_MODE(ANGLE_MODE) && !navigationIsControllingThrottle()) {
        // XXX: Should we do this for negative THR?
        float thr = controlOutput->throttle;
        float cruiseThr = rcCommandMapUnidirectionalPWMThrottle(navConfig()->fw.cruise_throttle);
        angleTarget += scaleRangef(MAX(0, cruiseThr - thr), 0, cruiseThr - RC_COMMAND_CENTER, 0, mixerConfig()->fwMinThrottleDownPitchAngle);
    }

    const float angleErrorDeg = DECIDEGREES_TO_DEGREES(angleTarget - attitude.raw[axis]);

    float angleRateTarget = constrainf(angleErrorDeg * (pidBank()->pid[PID_LEVEL].P / FP_PID_LEVEL_P_MULTIPLIER), -currentControlRateProfile->stabilized.rates[axis] * 10.0f, currentControlRateProfile->stabilized.rates[axis] * 10.0f);

    // Apply simple LPF to angleRateTarget to make response less jerky
    // Ideas behind this:
    //  1) Attitude is updated at gyro rate, rateTarget for ANGLE mode is calculated from attitude
    //  2) If this rateTarget is passed directly into gyro-base PID controller this effectively doubles the rateError.
    //     D-term that is calculated from error tend to amplify this even more. Moreover, this tend to respond to every
    //     slightest change in attitude making self-leveling jittery
    //  3) Lowering LEVEL P can make the effects of (2) less visible, but this also slows down self-leveling.
    //  4) Human pilot response to attitude change in RATE mode is fairly slow and smooth, human pilot doesn't
    //     compensate for each slightest change
    //  5) (2) and (4) lead to a simple idea of adding a low-pass filter on rateTarget for ANGLE mode damping
    //     response to rapid attitude changes and smoothing out self-leveling reaction
    if (pidBank()->pid[PID_LEVEL].I) {
        // I8[PIDLEVEL] is filter cutoff frequency (Hz). Practical values of filtering frequency is 5-10 Hz
        angleRateTarget = pt1FilterApply4(&pidState->angleFilterState, angleRateTarget, pidBank()->pid[PID_LEVEL].I, dT);
    }

    // P[LEVEL] defines self-leveling strength (both for ANGLE and HORIZON modes)
    if (FLIGHT_MODE(HORIZON_MODE)) {
        pidState->rateTarget = (1.0f - horizonRateMagnitude) * angleRateTarget + horizonRateMagnitude * pidState->rateTarget;
    } else {
        pidState->rateTarget = angleRateTarget;
    }
}

/* Apply angular acceleration limit to rate target to limit extreme stick inputs to respect physical capabilities of the machine */
static void FAST_CODE pidApplySetpointRateLimiting(pidState_t *pidState, flight_dynamics_index_t axis)
{
    const uint32_t axisAccelLimit = (axis == FD_YAW) ? pidProfile()->axisAccelerationLimitYaw : pidProfile()->axisAccelerationLimitRollPitch;

    if (axisAccelLimit > AXIS_ACCEL_MIN_LIMIT) {
        pidState->rateTarget = rateLimitFilterApply4(&pidState->axisAccelFilter, pidState->rateTarget, (float)axisAccelLimit, dT);
    }
}

bool isFixedWingItermLimitActive(float stickPosition)
{
    /*
     * Iterm anti windup whould be active only when pilot controls the rotation
     * velocity directly, not when ANGLE or HORIZON are used
     */
    if (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE)) {
        return false;
    }

    return fabsf(stickPosition) > pidProfile()->fixedWingItermLimitOnStickPosition;
}

static void FAST_CODE pidApplyFixedWingRateController(pidState_t *pidState, flight_dynamics_index_t axis)
{
    const float rateError = pidState->rateTarget - pidState->gyroRate;

    // Calculate new P-term and FF-term
    float newPTerm = rateError * pidState->kP;
    float newFFTerm = pidState->rateTarget * pidState->kFF;

    // Additional P-term LPF on YAW axis
    if (axis == FD_YAW && pidProfile()->yaw_lpf_hz) {
        newPTerm = pt1FilterApply4(&pidState->ptermLpfState, newPTerm, pidProfile()->yaw_lpf_hz, dT);
    }

    // Calculate integral
    pidState->errorGyroIf += rateError * pidState->kI * dT;

    if (STATE(ANTI_WINDUP) || isFixedWingItermLimitActive(pidState->stickPosition)) {
        pidState->errorGyroIf = constrainf(pidState->errorGyroIf, -pidState->errorGyroIfLimit, pidState->errorGyroIfLimit);
    } else {
        pidState->errorGyroIfLimit = fabsf(pidState->errorGyroIf);
    }

    if (pidProfile()->fixedWingItermThrowLimit != 0) {
        pidState->errorGyroIf = constrainf(pidState->errorGyroIf, -pidProfile()->fixedWingItermThrowLimit, pidProfile()->fixedWingItermThrowLimit);
    }

#ifdef USE_AUTOTUNE_FIXED_WING
    if (FLIGHT_MODE(AUTO_TUNE) && !FLIGHT_MODE(MANUAL_MODE)) {
        autotuneFixedWingUpdate(axis, pidState->rateTarget, pidState->gyroRate, newPTerm + newFFTerm);
    }
#endif

    axisPID[axis] = constrainf(newPTerm + newFFTerm + pidState->errorGyroIf, -pidProfile()->pidSumLimit, +pidProfile()->pidSumLimit);

#ifdef USE_BLACKBOX
    axisPID_P[axis] = newPTerm;
    axisPID_I[axis] = pidState->errorGyroIf;
    axisPID_D[axis] = newFFTerm;
    axisPID_Setpoint[axis] = pidState->rateTarget;
#endif
}

static void FAST_CODE applyItermRelax(const int axis, const float gyroRate, float currentPidSetpoint, float *itermErrorRate)
{
    const float setpointLpf = pt1FilterApply(&windupLpf[axis], currentPidSetpoint);
    const float setpointHpf = fabsf(currentPidSetpoint - setpointLpf);

    if (itermRelax) {
        if (axis < FD_YAW || itermRelax == ITERM_RELAX_RPY) {

            const float itermRelaxFactor = MAX(0, 1 - setpointHpf / itermRelaxSetpointThreshold);

            if (itermRelaxType == ITERM_RELAX_SETPOINT) {
                *itermErrorRate *= itermRelaxFactor;
            } else if (itermRelaxType == ITERM_RELAX_GYRO ) {
                *itermErrorRate = fapplyDeadbandf(setpointLpf - gyroRate, setpointHpf);
            } else {
                *itermErrorRate = 0.0f;
            }

            if (axis == FD_ROLL) {
                DEBUG_SET(DEBUG_ITERM_RELAX, 0, lrintf(setpointHpf));
                DEBUG_SET(DEBUG_ITERM_RELAX, 1, lrintf(itermRelaxFactor * 100.0f));
                DEBUG_SET(DEBUG_ITERM_RELAX, 2, lrintf(*itermErrorRate));
            }
        }
    }
}
#ifdef USE_D_BOOST
static float FAST_CODE applyDBoost(pidState_t *pidState, flight_dynamics_index_t axis) {
    
    float dBoost = 1.0f;
    
    if (dBoostFactor > 1) {
        const float dBoostGyroDelta = (pidState->gyroRate - pidState->previousRateGyro) / dT;
        const float dBoostGyroAcceleration = fabsf(biquadFilterApply(&pidState->dBoostGyroLpf, dBoostGyroDelta));
        const float dBoostRateAcceleration = fabsf((pidState->rateTarget - pidState->previousRateTarget) / dT);
        
        const float acceleration = MAX(dBoostGyroAcceleration, dBoostRateAcceleration);
        dBoost = scaleRangef(acceleration, 0.0f, dBoostMaxAtAlleceleration, 1.0f, dBoostFactor);
        dBoost = pt1FilterApply4(&pidState->dBoostLpf, dBoost, D_BOOST_LPF_HZ, dT);
        dBoost = constrainf(dBoost, 1.0f, dBoostFactor);

        if (axis == FD_ROLL) {
            DEBUG_SET(DEBUG_D_BOOST, 0, dBoostGyroAcceleration);
            DEBUG_SET(DEBUG_D_BOOST, 1, dBoostRateAcceleration);
            DEBUG_SET(DEBUG_D_BOOST, 2, dBoost * 100);
        } else if (axis == FD_PITCH) {
            DEBUG_SET(DEBUG_D_BOOST, 3, dBoost * 100);
        }

        pidState->previousRateTarget = pidState->rateTarget;
        pidState->previousRateGyro = pidState->gyroRate;
    } 

    return dBoost;
}
#else 
static float applyDBoost(pidState_t *pidState, flight_dynamics_index_t axis) {
    UNUSED(pidState);
    UNUSED(axis);
    return 1.0f;
}
#endif

static void FAST_CODE pidApplyMulticopterRateController(pidState_t *pidState, flight_dynamics_index_t axis)
{
    const float rateError = pidState->rateTarget - pidState->gyroRate;

    // Calculate new P-term
    float newPTerm = rateError * pidState->kP;
    // Constrain YAW by yaw_p_limit value if not servo driven (in that case servo limits apply)
    if (axis == FD_YAW && (getMotorCount() >= 4 && pidProfile()->yaw_p_limit)) {
        newPTerm = constrain(newPTerm, -pidProfile()->yaw_p_limit, pidProfile()->yaw_p_limit);
    }

    // Additional P-term LPF on YAW axis
    if (axis == FD_YAW && pidProfile()->yaw_lpf_hz) {
        newPTerm = pt1FilterApply4(&pidState->ptermLpfState, newPTerm, pidProfile()->yaw_lpf_hz, dT);
    }

    // Calculate new D-term
    float newDTerm;
    if (pidBank()->pid[axis].D == 0) {
        // optimisation for when D8 is zero, often used by YAW axis
        newDTerm = 0;
    } else {
        // Calculate delta for Dterm calculation. Apply filters before derivative to minimize effects of dterm kick
        float deltaFiltered = pidProfile()->dterm_setpoint_weight * pidState->rateTarget - pidState->gyroRate;

#ifdef USE_DTERM_NOTCH
        // Apply D-term notch
        deltaFiltered = notchFilterApplyFn(&pidState->deltaNotchFilter, deltaFiltered);
#endif

        // Apply additional lowpass
        if (pidProfile()->dterm_lpf_hz) {
            deltaFiltered = biquadFilterApply(&pidState->deltaLpfState, deltaFiltered);
        }

        firFilterUpdate(&pidState->gyroRateFilter, deltaFiltered);
        newDTerm = firFilterApply(&pidState->gyroRateFilter);

        // Calculate derivative
        newDTerm =  newDTerm * (pidState->kD / dT) * applyDBoost(pidState, axis);

        // Additionally constrain D
        newDTerm = constrainf(newDTerm, -300.0f, 300.0f);
    }

    // TODO: Get feedback from mixer on available correction range for each axis
    const float newOutput = newPTerm + newDTerm + pidState->errorGyroIf;
    const float newOutputLimited = constrainf(newOutput, -pidProfile()->pidSumLimit, +pidProfile()->pidSumLimit);

    // Prevent strong Iterm accumulation during stick inputs
    const float motorItermWindupPoint = 1.0f - (pidProfile()->itermWindupPointPercent / 100.0f);
    const float antiWindupScaler = constrainf((1.0f - getMotorMixRange()) / motorItermWindupPoint, 0.0f, 1.0f);

    float itermErrorRate = rateError;
    applyItermRelax(axis, pidState->gyroRate, pidState->rateTarget, &itermErrorRate);

    pidState->errorGyroIf += (itermErrorRate * pidState->kI * antiWindupScaler * dT)
                             + ((newOutputLimited - newOutput) * pidState->kT * antiWindupScaler * dT);

    // Don't grow I-term if motors are at their limit
    if (STATE(ANTI_WINDUP) || mixerIsOutputSaturated()) {
        pidState->errorGyroIf = constrainf(pidState->errorGyroIf, -pidState->errorGyroIfLimit, pidState->errorGyroIfLimit);
    } else {
        pidState->errorGyroIfLimit = fabsf(pidState->errorGyroIf);
    }

    axisPID[axis] = newOutputLimited;

#ifdef USE_BLACKBOX
    axisPID_P[axis] = newPTerm;
    axisPID_I[axis] = pidState->errorGyroIf;
    axisPID_D[axis] = newDTerm;
    axisPID_Setpoint[axis] = pidState->rateTarget;
#endif
}

void updateHeadingHoldTarget(int16_t heading)
{
    headingHoldTarget = heading;
}

void resetHeadingHoldTarget(int16_t heading)
{
    updateHeadingHoldTarget(heading);
    pt1FilterReset(&headingHoldRateFilter, 0.0f);
}

int16_t getHeadingHoldTarget() {
    return headingHoldTarget;
}

static uint8_t getHeadingHoldState(void)
{
    // Don't apply heading hold if overall tilt is greater than maximum angle inclination
    if (calculateCosTiltAngle() < headingHoldCosZLimit) {
        return HEADING_HOLD_DISABLED;
    }

#if defined(USE_NAV)
    int navHeadingState = navigationGetHeadingControlState();
    // NAV will prevent MAG_MODE from activating, but require heading control
    if (navHeadingState != NAV_HEADING_CONTROL_NONE) {
        // Apply maghold only if heading control is in auto mode
        if (navHeadingState == NAV_HEADING_CONTROL_AUTO) {
            return HEADING_HOLD_ENABLED;
        }
    }
    else
#endif
    if (rcControlGetInput()->yaw == 0 && FLIGHT_MODE(HEADING_MODE)) {
        return HEADING_HOLD_ENABLED;
    } else {
        return HEADING_HOLD_UPDATE_HEADING;
    }

    return HEADING_HOLD_UPDATE_HEADING;
}

/*
 * HEADING_HOLD P Controller returns desired rotation rate in dps to be fed to Rate controller
 */
float pidHeadingHold(void)
{
    float headingHoldRate;

    int16_t error = DECIDEGREES_TO_DEGREES(attitude.values.yaw) - headingHoldTarget;

    /*
     * Convert absolute error into relative to current heading
     */
    if (error <= -180) {
        error += 360;
    }

    if (error >= +180) {
        error -= 360;
    }

    /*
        New MAG_HOLD controller work slightly different that previous one.
        Old one mapped error to rotation speed in following way:
            - on rate 0 it gave about 0.5dps for each degree of error
            - error 0 = rotation speed of 0dps
            - error 180 = rotation speed of 96 degrees per second
            - output
            - that gives about 2 seconds to correct any error, no matter how big. Of course, usually more because of inertia.
        That was making him quite "soft" for small changes and rapid for big ones that started to appear
        when iNav introduced real RTH and WAYPOINT that might require rapid heading changes.

        New approach uses modified principle:
            - manual yaw rate is not used. MAG_HOLD is decoupled from manual input settings
            - instead, mag_hold_rate_limit is used. It defines max rotation speed in dps that MAG_HOLD controller can require from RateController
            - computed rotation speed is capped at -mag_hold_rate_limit and mag_hold_rate_limit
            - Default mag_hold_rate_limit = 40dps and default MAG_HOLD P-gain is 40
            - With those values, maximum rotation speed will be required from Rate Controller when error is greater that 30 degrees
            - For smaller error, required rate will be proportional.
            - It uses LPF filter set at 2Hz to additionally smoothen out any rapid changes
            - That makes correction of smaller errors stronger, and those of big errors softer

        This make looks as very slow rotation rate, but please remember this is automatic mode.
        Manual override with YAW input when MAG_HOLD is enabled will still use "manual" rates, not MAG_HOLD rates.
        Highest possible correction is 180 degrees and it will take more less 4.5 seconds. It is even more than sufficient
        to run RTH or WAYPOINT missions. My favourite rate range here is 20dps - 30dps that gives nice and smooth turns.

        Correction for small errors is much faster now. For example, old contrioller for 2deg errors required 1dps (correction in 2 seconds).
        New controller for 2deg error requires 2,6dps. 4dps for 3deg and so on up until mag_hold_rate_limit is reached.
    */

    headingHoldRate = error * pidBank()->pid[PID_HEADING].P / 30.0f;
    headingHoldRate = constrainf(headingHoldRate, -pidProfile()->heading_hold_rate_limit, pidProfile()->heading_hold_rate_limit);
    headingHoldRate = pt1FilterApply4(&headingHoldRateFilter, headingHoldRate, HEADING_HOLD_ERROR_LPF_FREQ, dT);

    return headingHoldRate;
}

/*
 * TURN ASSISTANT mode is an assisted mode to do a Yaw rotation on a ground plane, allowing one-stick turn in RATE more
 * and keeping ROLL and PITCH attitude though the turn.
 */
static void pidTurnAssistant(pidState_t *pidState)
{
    fpVector3_t targetRates;
    targetRates.x = 0.0f;
    targetRates.y = 0.0f;

    if (STATE(FIXED_WING)) {
        if (calculateCosTiltAngle() >= 0.173648f) {
            // Ideal banked turn follow the equations:
            //      forward_vel^2 / radius = Gravity * tan(roll_angle)
            //      yaw_rate = forward_vel / radius
            // If we solve for roll angle we get:
            //      tan(roll_angle) = forward_vel * yaw_rate / Gravity
            // If we solve for yaw rate we get:
            //      yaw_rate = tan(roll_angle) * Gravity / forward_vel

#if defined(USE_PITOT)
            float airspeedForCoordinatedTurn = sensors(SENSOR_PITOT) ?
                    pitot.airSpeed :
                    pidProfile()->fixedWingReferenceAirspeed;
#else
            float airspeedForCoordinatedTurn = pidProfile()->fixedWingReferenceAirspeed;
#endif

            // Constrain to somewhat sane limits - 10km/h - 216km/h
            airspeedForCoordinatedTurn = constrainf(airspeedForCoordinatedTurn, 300, 6000);

            // Calculate rate of turn in Earth frame according to FAA's Pilot's Handbook of Aeronautical Knowledge
            float bankAngle = DECIDEGREES_TO_RADIANS(attitude.values.roll);
            float coordinatedTurnRateEarthFrame = GRAVITY_CMSS * tan_approx(-bankAngle) / airspeedForCoordinatedTurn;

            targetRates.z = RADIANS_TO_DEGREES(coordinatedTurnRateEarthFrame);
        }
        else {
            // Don't allow coordinated turn calculation if airplane is in hard bank or steep climb/dive
            return;
        }
    }
    else {
        targetRates.z = pidState[YAW].rateTarget;
    }

    // Transform calculated rate offsets into body frame and apply
    imuTransformVectorEarthToBody(&targetRates);

    // Add in roll and pitch
    pidState[ROLL].rateTarget = constrainf(pidState[ROLL].rateTarget + targetRates.x, -currentControlRateProfile->stabilized.rates[ROLL] * 10.0f, currentControlRateProfile->stabilized.rates[ROLL] * 10.0f);
    pidState[PITCH].rateTarget = constrainf(pidState[PITCH].rateTarget + targetRates.y, -currentControlRateProfile->stabilized.rates[PITCH] * 10.0f, currentControlRateProfile->stabilized.rates[PITCH] * 10.0f);

    // Replace YAW on quads - add it in on airplanes
    if (STATE(FIXED_WING)) {
        pidState[YAW].rateTarget = constrainf(pidState[YAW].rateTarget + targetRates.z * pidProfile()->fixedWingCoordinatedYawGain, -currentControlRateProfile->stabilized.rates[YAW] * 10.0f, currentControlRateProfile->stabilized.rates[YAW] * 10.0f);
    }
    else {
        pidState[YAW].rateTarget = constrainf(targetRates.z, -currentControlRateProfile->stabilized.rates[YAW] * 10.0f, currentControlRateProfile->stabilized.rates[YAW] * 10.0f);
    }
}

static void pidApplyFpvCameraAngleMix(pidState_t *pidState, uint8_t fpvCameraAngle)
{
    static uint8_t lastFpvCamAngleDegrees = 0;
    static float cosCameraAngle = 1.0;
    static float sinCameraAngle = 0.0;

    if (lastFpvCamAngleDegrees != fpvCameraAngle) {
        lastFpvCamAngleDegrees = fpvCameraAngle;
        cosCameraAngle = cos_approx(DEGREES_TO_RADIANS(fpvCameraAngle));
        sinCameraAngle = sin_approx(DEGREES_TO_RADIANS(fpvCameraAngle));
    }

    // Rotate roll/yaw command from camera-frame coordinate system to body-frame coordinate system
    const float rollRate = pidState[ROLL].rateTarget;
    const float yawRate = pidState[YAW].rateTarget;
    pidState[ROLL].rateTarget = constrainf(rollRate * cosCameraAngle -  yawRate * sinCameraAngle, -GYRO_SATURATION_LIMIT, GYRO_SATURATION_LIMIT);
    pidState[YAW].rateTarget = constrainf(yawRate * cosCameraAngle + rollRate * sinCameraAngle, -GYRO_SATURATION_LIMIT, GYRO_SATURATION_LIMIT);
}

void FAST_CODE pidController(void)
{
    if (!pidFiltersConfigured) {
        return;
    }

    bool canUseFpvCameraMix = true;
    uint8_t headingHoldState = getHeadingHoldState();

    if (headingHoldState == HEADING_HOLD_UPDATE_HEADING) {
        updateHeadingHoldTarget(DECIDEGREES_TO_DEGREES(attitude.values.yaw));
    }

    const rcCommand_t *controlOutput = rcControlGetOutput();

    for (int axis = ROLL; axis <= YAW; axis++) {
        // Step 1: Calculate gyro rates
        pidState[axis].gyroRate = gyro.gyroADCf[axis];

        // Step 2: Read target
        float rateTarget;

        if (axis == FD_YAW && headingHoldState == HEADING_HOLD_ENABLED) {
            rateTarget = pidHeadingHold();
        } else {
            rateTarget = pidRcCommandToRate(controlOutput->axes[axis], currentControlRateProfile->stabilized.rates[axis]);
        }

        // Limit desired rate to something gyro can measure reliably
        pidState[axis].rateTarget = constrainf(rateTarget, -GYRO_SATURATION_LIMIT, +GYRO_SATURATION_LIMIT);
    }

    // Step 3: Run control for ANGLE_MODE, HORIZON_MODE, and HEADING_LOCK
    if (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE)) {
        const float horizonRateMagnitude = calcHorizonRateMagnitude();
        pidLevel(&pidState[FD_ROLL], FD_ROLL, horizonRateMagnitude);
        pidLevel(&pidState[FD_PITCH], FD_PITCH, horizonRateMagnitude);
        canUseFpvCameraMix = false;     // FPVANGLEMIX is incompatible with ANGLE/HORIZON
    }

    if (FLIGHT_MODE(TURN_ASSISTANT) || navigationRequiresTurnAssistance()) {
        pidTurnAssistant(pidState);
        canUseFpvCameraMix = false;     // FPVANGLEMIX is incompatible with TURN_ASSISTANT
    }

    if (canUseFpvCameraMix && IS_RC_MODE_ACTIVE(BOXFPVANGLEMIX) && currentControlRateProfile->misc.fpvCamAngleDegrees) {
        pidApplyFpvCameraAngleMix(pidState, currentControlRateProfile->misc.fpvCamAngleDegrees);
    }

    // Apply setpoint rate of change limits
    for (int axis = 0; axis < 3; axis++) {
        pidApplySetpointRateLimiting(&pidState[axis], axis);
    }

    // Step 4: Run gyro-driven control
    for (int axis = 0; axis < 3; axis++) {
        // Apply PID setpoint controller
        if (STATE(FIXED_WING)) {
            pidApplyFixedWingRateController(&pidState[axis], axis);
        }
        else {
            pidApplyMulticopterRateController(&pidState[axis], axis);
        }
    }
}

pidType_e pidIndexGetType(pidIndex_e pidIndex)
{
    if (STATE(FIXED_WING)) {
        // FW specific
        if (pidIndex == PID_ROLL || pidIndex == PID_PITCH || pidIndex == PID_YAW) {
            return PID_TYPE_PIFF;
        }
        if (pidIndex == PID_VEL_XY || pidIndex == PID_VEL_Z) {
            return PID_TYPE_NONE;
        }
    }
    // Common
    if (pidIndex == PID_SURFACE) {
        return PID_TYPE_NONE;
    }
    return PID_TYPE_PID;
}
