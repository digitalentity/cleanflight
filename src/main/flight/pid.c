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

FILE_COMPILE_FOR_SPEED

#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"
#include "common/utils.h"
#include "common/fp_pid.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "fc/config.h"
#include "fc/controlrate_profile.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"
#include "fc/settings.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/rpm_filter.h"
#include "flight/secondary_imu.h"
#include "flight/kalman.h"
#include "flight/smith_predictor.h"

#include "io/gps.h"

#include "navigation/navigation.h"

#include "rx/rx.h"

#include "sensors/sensors.h"
#include "sensors/gyro.h"
#include "sensors/acceleration.h"
#include "sensors/compass.h"
#include "sensors/pitotmeter.h"

#include "scheduler/scheduler.h"

#include "programming/logic_condition.h"

typedef struct {
    float kP;   // Proportional gain
    float kI;   // Integral gain
    float kD;   // Derivative gain
    float kFF;  // Feed-forward gain
    float kCD;  // Control Derivative
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
    filter_t dtermLpfState;
    filter_t dtermLpf2State;

    float stickPosition;

    float previousRateTarget;
    float previousRateGyro;

#ifdef USE_D_BOOST
    pt1Filter_t dBoostLpf;
    biquadFilter_t dBoostGyroLpf;
#endif
    uint16_t pidSumLimit;
    filterApply4FnPtr ptermFilterApplyFn;
    bool itermLimitActive;
    bool itermFreezeActive;

    biquadFilter_t rateTargetFilter;

    smithPredictor_t smithPredictor;
} pidState_t;

STATIC_FASTRAM bool pidFiltersConfigured = false;
static EXTENDED_FASTRAM float headingHoldCosZLimit;
static EXTENDED_FASTRAM int16_t headingHoldTarget;
static EXTENDED_FASTRAM pt1Filter_t headingHoldRateFilter;
static EXTENDED_FASTRAM pt1Filter_t fixedWingTpaFilter;

// Thrust PID Attenuation factor. 0.0f means fully attenuated, 1.0f no attenuation is applied
STATIC_FASTRAM bool pidGainsUpdateRequired;
FASTRAM int16_t axisPID[FLIGHT_DYNAMICS_INDEX_COUNT];

#ifdef USE_BLACKBOX
int32_t axisPID_P[FLIGHT_DYNAMICS_INDEX_COUNT], axisPID_I[FLIGHT_DYNAMICS_INDEX_COUNT], axisPID_D[FLIGHT_DYNAMICS_INDEX_COUNT], axisPID_Setpoint[FLIGHT_DYNAMICS_INDEX_COUNT];
#endif

static EXTENDED_FASTRAM pidState_t pidState[FLIGHT_DYNAMICS_INDEX_COUNT];
static EXTENDED_FASTRAM pt1Filter_t windupLpf[XYZ_AXIS_COUNT];
static EXTENDED_FASTRAM uint8_t itermRelax;

#ifdef USE_ANTIGRAVITY
static EXTENDED_FASTRAM pt1Filter_t antigravityThrottleLpf;
static EXTENDED_FASTRAM float antigravityThrottleHpf;
static EXTENDED_FASTRAM float antigravityGain;
static EXTENDED_FASTRAM float antigravityAccelerator;
#endif

#define D_BOOST_GYRO_LPF_HZ 80    // Biquad lowpass input cutoff to peak D around propwash frequencies
#define D_BOOST_LPF_HZ 10         // PT1 lowpass cutoff to smooth the boost effect

#ifdef USE_D_BOOST
static EXTENDED_FASTRAM float dBoostFactor;
static EXTENDED_FASTRAM float dBoostMaxAtAlleceleration;
#endif

static EXTENDED_FASTRAM uint8_t yawLpfHz;
static EXTENDED_FASTRAM float motorItermWindupPoint;
static EXTENDED_FASTRAM float antiWindupScaler;
#ifdef USE_ANTIGRAVITY
static EXTENDED_FASTRAM float iTermAntigravityGain;
#endif
static EXTENDED_FASTRAM uint8_t usedPidControllerType;

typedef void (*pidControllerFnPtr)(pidState_t *pidState, flight_dynamics_index_t axis, float dT);
static EXTENDED_FASTRAM pidControllerFnPtr pidControllerApplyFn;
static EXTENDED_FASTRAM filterApplyFnPtr dTermLpfFilterApplyFn;
static EXTENDED_FASTRAM filterApplyFnPtr dTermLpf2FilterApplyFn;
static EXTENDED_FASTRAM bool levelingEnabled = false;

#define FIXED_WING_LEVEL_TRIM_MAX_ANGLE 10.0f // Max angle auto trimming can demand
#define FIXED_WING_LEVEL_TRIM_DIVIDER 500.0f
#define FIXED_WING_LEVEL_TRIM_MULTIPLIER 1.0f / FIXED_WING_LEVEL_TRIM_DIVIDER
#define FIXED_WING_LEVEL_TRIM_CONTROLLER_LIMIT FIXED_WING_LEVEL_TRIM_DIVIDER * FIXED_WING_LEVEL_TRIM_MAX_ANGLE

static EXTENDED_FASTRAM float fixedWingLevelTrim;
static EXTENDED_FASTRAM pidController_t fixedWingLevelTrimController;

PG_REGISTER_PROFILE_WITH_RESET_TEMPLATE(pidProfile_t, pidProfile, PG_PID_PROFILE, 2);

PG_RESET_TEMPLATE(pidProfile_t, pidProfile,
        .bank_mc = {
            .pid = {
                [PID_ROLL] =  { SETTING_MC_P_ROLL_DEFAULT, SETTING_MC_I_ROLL_DEFAULT, SETTING_MC_D_ROLL_DEFAULT, SETTING_MC_CD_ROLL_DEFAULT },
                [PID_PITCH] = { SETTING_MC_P_PITCH_DEFAULT, SETTING_MC_I_PITCH_DEFAULT, SETTING_MC_D_PITCH_DEFAULT, SETTING_MC_CD_PITCH_DEFAULT },
                [PID_YAW] =   { SETTING_MC_P_YAW_DEFAULT, SETTING_MC_I_YAW_DEFAULT, SETTING_MC_D_YAW_DEFAULT, SETTING_MC_CD_YAW_DEFAULT },
                [PID_LEVEL] = {
                    .P = SETTING_MC_P_LEVEL_DEFAULT,          // Self-level strength
                    .I = SETTING_MC_I_LEVEL_DEFAULT,          // Self-leveing low-pass frequency (0 - disabled)
                    .D = SETTING_MC_D_LEVEL_DEFAULT,          // 75% horizon strength
                    .FF = 0,
                },
                [PID_HEADING] = { SETTING_NAV_MC_HEADING_P_DEFAULT, 0, 0, 0 },
                [PID_POS_XY] = {
                    .P = SETTING_NAV_MC_POS_XY_P_DEFAULT,     // NAV_POS_XY_P * 100
                    .I = 0,
                    .D = 0,
                    .FF = 0,
                },
                [PID_VEL_XY] = {
                    .P = SETTING_NAV_MC_VEL_XY_P_DEFAULT,     // NAV_VEL_XY_P * 20
                    .I = SETTING_NAV_MC_VEL_XY_I_DEFAULT,     // NAV_VEL_XY_I * 100
                    .D = SETTING_NAV_MC_VEL_XY_D_DEFAULT,     // NAV_VEL_XY_D * 100
                    .FF = SETTING_NAV_MC_VEL_XY_FF_DEFAULT,   // NAV_VEL_XY_D * 100
                },
                [PID_POS_Z] = {
                    .P = SETTING_NAV_MC_POS_Z_P_DEFAULT,      // NAV_POS_Z_P * 100
                    .I = 0,                                   // not used
                    .D = 0,                                   // not used
                    .FF = 0,
                },
                [PID_VEL_Z] = {
                    .P = SETTING_NAV_MC_VEL_Z_P_DEFAULT,      // NAV_VEL_Z_P * 66.7
                    .I = SETTING_NAV_MC_VEL_Z_I_DEFAULT,      // NAV_VEL_Z_I * 20
                    .D = SETTING_NAV_MC_VEL_Z_D_DEFAULT,      // NAV_VEL_Z_D * 100
                    .FF = 0,
                },
                [PID_POS_HEADING] = {
                    .P = 0,
                    .I = 0,
                    .D = 0,
                    .FF = 0
                }
            }
        },

        .bank_fw = {
            .pid = {
                [PID_ROLL] =  { SETTING_FW_P_ROLL_DEFAULT, SETTING_FW_I_ROLL_DEFAULT, 0, SETTING_FW_FF_ROLL_DEFAULT },
                [PID_PITCH] =  { SETTING_FW_P_PITCH_DEFAULT, SETTING_FW_I_PITCH_DEFAULT, 0, SETTING_FW_FF_PITCH_DEFAULT },
                [PID_YAW] =  { SETTING_FW_P_YAW_DEFAULT, SETTING_FW_I_YAW_DEFAULT, 0, SETTING_FW_FF_YAW_DEFAULT },
                [PID_LEVEL] = {
                    .P = SETTING_FW_P_LEVEL_DEFAULT,          // Self-level strength
                    .I = SETTING_FW_I_LEVEL_DEFAULT,          // Self-leveing low-pass frequency (0 - disabled)
                    .D = SETTING_FW_D_LEVEL_DEFAULT,          // 75% horizon strength
                    .FF = 0,
                },
                [PID_HEADING] = { SETTING_NAV_FW_HEADING_P_DEFAULT, 0, 0, 0 },
                [PID_POS_Z] = {
                    .P = SETTING_NAV_FW_POS_Z_P_DEFAULT,      // FW_POS_Z_P * 10
                    .I = SETTING_NAV_FW_POS_Z_I_DEFAULT,      // FW_POS_Z_I * 10
                    .D = SETTING_NAV_FW_POS_Z_D_DEFAULT,      // FW_POS_Z_D * 10
                    .FF = 0,
                },
                [PID_POS_XY] = {
                    .P = SETTING_NAV_FW_POS_XY_P_DEFAULT,     // FW_POS_XY_P * 100
                    .I = SETTING_NAV_FW_POS_XY_I_DEFAULT,     // FW_POS_XY_I * 100
                    .D = SETTING_NAV_FW_POS_XY_D_DEFAULT,     // FW_POS_XY_D * 100
                    .FF = 0,
                },
                [PID_POS_HEADING] = {
                    .P = SETTING_NAV_FW_POS_HDG_P_DEFAULT,
                    .I = SETTING_NAV_FW_POS_HDG_I_DEFAULT,
                    .D = SETTING_NAV_FW_POS_HDG_D_DEFAULT,
                    .FF = 0
                }
            }
        },

        .dterm_lpf_type = SETTING_DTERM_LPF_TYPE_DEFAULT,
        .dterm_lpf_hz = SETTING_DTERM_LPF_HZ_DEFAULT,
        .dterm_lpf2_type = SETTING_DTERM_LPF2_TYPE_DEFAULT,
        .dterm_lpf2_hz = SETTING_DTERM_LPF2_HZ_DEFAULT,
        .yaw_lpf_hz = SETTING_YAW_LPF_HZ_DEFAULT,

        .itermWindupPointPercent = SETTING_ITERM_WINDUP_DEFAULT,

        .axisAccelerationLimitYaw = SETTING_RATE_ACCEL_LIMIT_YAW_DEFAULT,
        .axisAccelerationLimitRollPitch = SETTING_RATE_ACCEL_LIMIT_ROLL_PITCH_DEFAULT,

        .heading_hold_rate_limit = SETTING_HEADING_HOLD_RATE_LIMIT_DEFAULT,

        .max_angle_inclination[FD_ROLL] = SETTING_MAX_ANGLE_INCLINATION_RLL_DEFAULT,
        .max_angle_inclination[FD_PITCH] = SETTING_MAX_ANGLE_INCLINATION_PIT_DEFAULT,
        .pidSumLimit = SETTING_PIDSUM_LIMIT_DEFAULT,
        .pidSumLimitYaw = SETTING_PIDSUM_LIMIT_YAW_DEFAULT,

        .fixedWingItermThrowLimit = SETTING_FW_ITERM_THROW_LIMIT_DEFAULT,
        .fixedWingReferenceAirspeed = SETTING_FW_REFERENCE_AIRSPEED_DEFAULT,
        .fixedWingCoordinatedYawGain = SETTING_FW_TURN_ASSIST_YAW_GAIN_DEFAULT,
        .fixedWingCoordinatedPitchGain = SETTING_FW_TURN_ASSIST_PITCH_GAIN_DEFAULT,
        .fixedWingItermLimitOnStickPosition = SETTING_FW_ITERM_LIMIT_STICK_POSITION_DEFAULT,
        .fixedWingYawItermBankFreeze = SETTING_FW_YAW_ITERM_FREEZE_BANK_ANGLE_DEFAULT,

        .loiter_direction = SETTING_FW_LOITER_DIRECTION_DEFAULT,
        .navVelXyDTermLpfHz = SETTING_NAV_MC_VEL_XY_DTERM_LPF_HZ_DEFAULT,
        .navVelXyDtermAttenuation = SETTING_NAV_MC_VEL_XY_DTERM_ATTENUATION_DEFAULT,
        .navVelXyDtermAttenuationStart = SETTING_NAV_MC_VEL_XY_DTERM_ATTENUATION_START_DEFAULT,
        .navVelXyDtermAttenuationEnd = SETTING_NAV_MC_VEL_XY_DTERM_ATTENUATION_END_DEFAULT,
        .iterm_relax_cutoff = SETTING_MC_ITERM_RELAX_CUTOFF_DEFAULT,
        .iterm_relax = SETTING_MC_ITERM_RELAX_DEFAULT,

#ifdef USE_D_BOOST
        .dBoostFactor = SETTING_D_BOOST_FACTOR_DEFAULT,
        .dBoostMaxAtAlleceleration = SETTING_D_BOOST_MAX_AT_ACCELERATION_DEFAULT,
        .dBoostGyroDeltaLpfHz = SETTING_D_BOOST_GYRO_DELTA_LPF_HZ_DEFAULT,
#endif

#ifdef USE_ANTIGRAVITY
        .antigravityGain = SETTING_ANTIGRAVITY_GAIN_DEFAULT,
        .antigravityAccelerator = SETTING_ANTIGRAVITY_ACCELERATOR_DEFAULT,
        .antigravityCutoff = SETTING_ANTIGRAVITY_CUTOFF_LPF_HZ_DEFAULT,
#endif

        .pidControllerType = SETTING_PID_TYPE_DEFAULT,
        .navFwPosHdgPidsumLimit = SETTING_NAV_FW_POS_HDG_PIDSUM_LIMIT_DEFAULT,
        .controlDerivativeLpfHz = SETTING_MC_CD_LPF_HZ_DEFAULT,

#ifdef USE_GYRO_KALMAN
        .kalman_q = SETTING_SETPOINT_KALMAN_Q_DEFAULT,
        .kalman_w = SETTING_SETPOINT_KALMAN_W_DEFAULT,
        .kalman_sharpness = SETTING_SETPOINT_KALMAN_SHARPNESS_DEFAULT,
        .kalmanEnabled = SETTING_SETPOINT_KALMAN_ENABLED_DEFAULT,
#endif

        .fixedWingLevelTrim = SETTING_FW_LEVEL_PITCH_TRIM_DEFAULT,
        .fixedWingLevelTrimGain = SETTING_FW_LEVEL_PITCH_GAIN_DEFAULT,
        .fixedWingLevelTrimDeadband = SETTING_FW_LEVEL_PITCH_DEADBAND_DEFAULT,

#ifdef USE_SMITH_PREDICTOR
        .smithPredictorStrength = SETTING_SMITH_PREDICTOR_STRENGTH_DEFAULT,
        .smithPredictorDelay = SETTING_SMITH_PREDICTOR_DELAY_DEFAULT,
        .smithPredictorFilterHz = SETTING_SMITH_PREDICTOR_LPF_HZ_DEFAULT,
#endif
);

bool pidInitFilters(void)
{
    const uint32_t refreshRate = getLooptime();

    if (refreshRate == 0) {
        return false;
    }

    // Init other filters
    if (pidProfile()->dterm_lpf_hz) {
        for (int axis = 0; axis < 3; ++ axis) {
            if (pidProfile()->dterm_lpf_type == FILTER_PT1) {
                pt1FilterInit(&pidState[axis].dtermLpfState.pt1, pidProfile()->dterm_lpf_hz, refreshRate * 1e-6f);
            } else {
                biquadFilterInitLPF(&pidState[axis].dtermLpfState.biquad, pidProfile()->dterm_lpf_hz, refreshRate);
            }
        }
    }

    // Init other filters
    if (pidProfile()->dterm_lpf2_hz) {
        for (int axis = 0; axis < 3; ++ axis) {
            if (pidProfile()->dterm_lpf2_type == FILTER_PT1) {
                pt1FilterInit(&pidState[axis].dtermLpf2State.pt1, pidProfile()->dterm_lpf2_hz, refreshRate * 1e-6f);
            } else {
                biquadFilterInitLPF(&pidState[axis].dtermLpf2State.biquad, pidProfile()->dterm_lpf2_hz, refreshRate);
            }
        }
    }

    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
        pt1FilterInit(&windupLpf[i], pidProfile()->iterm_relax_cutoff, refreshRate * 1e-6f);
    }

#ifdef USE_ANTIGRAVITY
    pt1FilterInit(&antigravityThrottleLpf, pidProfile()->antigravityCutoff, TASK_PERIOD_HZ(TASK_AUX_RATE_HZ) * 1e-6f);
#endif

#ifdef USE_D_BOOST
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        biquadFilterInitLPF(&pidState[axis].dBoostGyroLpf, pidProfile()->dBoostGyroDeltaLpfHz, getLooptime());
    }
#endif

    if (pidProfile()->controlDerivativeLpfHz) {
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            biquadFilterInitLPF(&pidState[axis].rateTargetFilter, pidProfile()->controlDerivativeLpfHz, getLooptime());
        }
    }

#ifdef USE_SMITH_PREDICTOR
    smithPredictorInit(
        &pidState[FD_ROLL].smithPredictor, 
        pidProfile()->smithPredictorDelay, 
        pidProfile()->smithPredictorStrength, 
        pidProfile()->smithPredictorFilterHz, 
        getLooptime()
    );
    smithPredictorInit(
        &pidState[FD_PITCH].smithPredictor, 
        pidProfile()->smithPredictorDelay, 
        pidProfile()->smithPredictorStrength, 
        pidProfile()->smithPredictorFilterHz, 
        getLooptime()
    );
    smithPredictorInit(
        &pidState[FD_YAW].smithPredictor, 
        pidProfile()->smithPredictorDelay, 
        pidProfile()->smithPredictorStrength, 
        pidProfile()->smithPredictorFilterHz, 
        getLooptime()
    );
#endif

    pidFiltersConfigured = true;

    return true;
}

void pidResetTPAFilter(void)
{
    if (usedPidControllerType == PID_TYPE_PIFF && currentControlRateProfile->throttle.fixedWingTauMs > 0) {
        pt1FilterInitRC(&fixedWingTpaFilter, currentControlRateProfile->throttle.fixedWingTauMs * 1e-3f, TASK_PERIOD_HZ(TASK_AUX_RATE_HZ) * 1e-6f);
        pt1FilterReset(&fixedWingTpaFilter, getThrottleIdleValue());
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

void pidReduceErrorAccumulators(int8_t delta, uint8_t axis) 
{
    pidState[axis].errorGyroIf -= delta;
    pidState[axis].errorGyroIfLimit -= delta;
}

float getTotalRateTarget(void)
{
    return sqrtf(sq(pidState[FD_ROLL].rateTarget) + sq(pidState[FD_PITCH].rateTarget) + sq(pidState[FD_YAW].rateTarget));
}

float getAxisIterm(uint8_t axis) 
{
    return pidState[axis].errorGyroIf;
}

static float pidRcCommandToAngle(int16_t stick, int16_t maxInclination)
{
    stick = constrain(stick, -500, 500);
    return scaleRangef((float) stick, -500.0f, 500.0f, (float) -maxInclination, (float) maxInclination);
}

int16_t pidAngleToRcCommand(float angleDeciDegrees, int16_t maxInclination)
{
    angleDeciDegrees = constrainf(angleDeciDegrees, (float) -maxInclination, (float) maxInclination);
    return scaleRangef((float) angleDeciDegrees, (float) -maxInclination, (float) maxInclination, -500.0f, 500.0f);
}

/*
Map stick positions to desired rotatrion rate in given axis.
Rotation rate in dps at full stick deflection is defined by axis rate measured in dps/10
Rate 20 means 200dps at full stick deflection
*/
float pidRateToRcCommand(float rateDPS, uint8_t rate)
{
    const float maxRateDPS = rate * 10.0f;
    return scaleRangef(rateDPS, -maxRateDPS, maxRateDPS, -500.0f, 500.0f);
}

float pidRcCommandToRate(int16_t stick, uint8_t rate)
{
    const float maxRateDPS = rate * 10.0f;
    return scaleRangef((float) stick, -500.0f, 500.0f, -maxRateDPS, maxRateDPS);
}

static float calculateFixedWingTPAFactor(uint16_t throttle)
{
    float tpaFactor;

    // tpa_rate is amount of curve TPA applied to PIDs
    // tpa_breakpoint for fixed wing is cruise throttle value (value at which PIDs were tuned)
    if (currentControlRateProfile->throttle.dynPID != 0 && currentControlRateProfile->throttle.pa_breakpoint > getThrottleIdleValue() && !FLIGHT_MODE(AUTO_TUNE) && ARMING_FLAG(ARMED)) {
        if (throttle > getThrottleIdleValue()) {
            // Calculate TPA according to throttle
            tpaFactor = 0.5f + ((float)(currentControlRateProfile->throttle.pa_breakpoint - getThrottleIdleValue()) / (throttle - getThrottleIdleValue()) / 2.0f);

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
    if (currentControlRateProfile->throttle.dynPID == 0 || rcCommand[THROTTLE] < currentControlRateProfile->throttle.pa_breakpoint) {
        tpaFactor = 1.0f;
    } else if (rcCommand[THROTTLE] < motorConfig()->maxthrottle) {
        tpaFactor = (100 - (uint16_t)currentControlRateProfile->throttle.dynPID * (rcCommand[THROTTLE] - currentControlRateProfile->throttle.pa_breakpoint) / (float)(motorConfig()->maxthrottle - currentControlRateProfile->throttle.pa_breakpoint)) / 100.0f;
    } else {
        tpaFactor = (100 - currentControlRateProfile->throttle.dynPID) / 100.0f;
    }

    return tpaFactor;
}

void schedulePidGainsUpdate(void)
{
    pidGainsUpdateRequired = true;
}

void updatePIDCoefficients()
{
    STATIC_FASTRAM uint16_t prevThrottle = 0;

    // Check if throttle changed. Different logic for fixed wing vs multirotor
    if (usedPidControllerType == PID_TYPE_PIFF && (currentControlRateProfile->throttle.fixedWingTauMs > 0)) {
        uint16_t filteredThrottle = pt1FilterApply(&fixedWingTpaFilter, rcCommand[THROTTLE]);
        if (filteredThrottle != prevThrottle) {
            prevThrottle = filteredThrottle;
            pidGainsUpdateRequired = true;
        }
    }
    else {
        if (rcCommand[THROTTLE] != prevThrottle) {
            prevThrottle = rcCommand[THROTTLE];
            pidGainsUpdateRequired = true;
        }
    }

#ifdef USE_ANTIGRAVITY
    if (usedPidControllerType == PID_TYPE_PID) {
        antigravityThrottleHpf = rcCommand[THROTTLE] - pt1FilterApply(&antigravityThrottleLpf, rcCommand[THROTTLE]);
        iTermAntigravityGain = scaleRangef(fabsf(antigravityThrottleHpf) * antigravityAccelerator, 0.0f, 1000.0f, 1.0f, antigravityGain);    
    }
#endif

    /*
     * Compute stick position in range of [-1.0f : 1.0f] without deadband and expo
     */
    for (int axis = 0; axis < 3; axis++) {
        pidState[axis].stickPosition = constrain(rxGetChannelValue(axis) - PWM_RANGE_MIDDLE, -500, 500) / 500.0f;
    }

    // If nothing changed - don't waste time recalculating coefficients
    if (!pidGainsUpdateRequired) {
        return;
    }

    const float tpaFactor = usedPidControllerType == PID_TYPE_PIFF ? calculateFixedWingTPAFactor(prevThrottle) : calculateMultirotorTPAFactor();

    // PID coefficients can be update only with THROTTLE and TPA or inflight PID adjustments
    //TODO: Next step would be to update those only at THROTTLE or inflight adjustments change
    for (int axis = 0; axis < 3; axis++) {
        if (usedPidControllerType == PID_TYPE_PIFF) {
            // Airplanes - scale all PIDs according to TPA
            pidState[axis].kP  = pidBank()->pid[axis].P / FP_PID_RATE_P_MULTIPLIER  * tpaFactor;
            pidState[axis].kI  = pidBank()->pid[axis].I / FP_PID_RATE_I_MULTIPLIER  * tpaFactor;
            pidState[axis].kD  = pidBank()->pid[axis].D / FP_PID_RATE_D_MULTIPLIER * tpaFactor;
            pidState[axis].kFF = pidBank()->pid[axis].FF / FP_PID_RATE_FF_MULTIPLIER * tpaFactor;
            pidState[axis].kCD = 0.0f;
            pidState[axis].kT  = 0.0f;
        }
        else {
            const float axisTPA = (axis == FD_YAW) ? 1.0f : tpaFactor;
            pidState[axis].kP  = pidBank()->pid[axis].P / FP_PID_RATE_P_MULTIPLIER * axisTPA;
            pidState[axis].kI  = pidBank()->pid[axis].I / FP_PID_RATE_I_MULTIPLIER;
            pidState[axis].kD  = pidBank()->pid[axis].D / FP_PID_RATE_D_MULTIPLIER * axisTPA;
            pidState[axis].kCD = pidBank()->pid[axis].FF / FP_PID_RATE_D_FF_MULTIPLIER * axisTPA;
            pidState[axis].kFF = 0.0f;

            // Tracking anti-windup requires P/I/D to be all defined which is only true for MC
            if ((pidBank()->pid[axis].P != 0) && (pidBank()->pid[axis].I != 0) && (usedPidControllerType == PID_TYPE_PID)) {
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
    const int32_t stickPosAil = ABS(getRcStickDeflection(FD_ROLL));
    const int32_t stickPosEle = ABS(getRcStickDeflection(FD_PITCH));
    const float mostDeflectedStickPos = constrain(MAX(stickPosAil, stickPosEle), 0, 500) / 500.0f;
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

static void pidLevel(pidState_t *pidState, flight_dynamics_index_t axis, float horizonRateMagnitude, float dT)
{
    // This is ROLL/PITCH, run ANGLE/HORIZON controllers
    float angleTarget = pidRcCommandToAngle(rcCommand[axis], pidProfile()->max_angle_inclination[axis]);

    // Automatically pitch down if the throttle is manually controlled and reduced bellow cruise throttle
    if ((axis == FD_PITCH) && STATE(AIRPLANE) && FLIGHT_MODE(ANGLE_MODE) && !navigationIsControllingThrottle()) {
        angleTarget += scaleRange(MAX(0, navConfig()->fw.cruise_throttle - rcCommand[THROTTLE]), 0, navConfig()->fw.cruise_throttle - PWM_RANGE_MIN, 0, mixerConfig()->fwMinThrottleDownPitchAngle);        
    }

    //PITCH trim applied by a AutoLevel flight mode and manual pitch trimming
    if (axis == FD_PITCH && STATE(AIRPLANE)) {
        DEBUG_SET(DEBUG_AUTOLEVEL, 0, angleTarget * 10);
        DEBUG_SET(DEBUG_AUTOLEVEL, 1, fixedWingLevelTrim * 10);
        DEBUG_SET(DEBUG_AUTOLEVEL, 2, getEstimatedActualVelocity(Z));

        /* 
         * fixedWingLevelTrim has opposite sign to rcCommand.
         * Positive rcCommand means nose should point downwards
         * Negative rcCommand mean nose should point upwards
         * This is counter intuitive and a natural way suggests that + should mean UP
         * This is why fixedWingLevelTrim has opposite sign to rcCommand
         * Positive fixedWingLevelTrim means nose should point upwards
         * Negative fixedWingLevelTrim means nose should point downwards
         */
        angleTarget -= fixedWingLevelTrim;   
        DEBUG_SET(DEBUG_AUTOLEVEL, 3, angleTarget * 10);
    }

        //PITCH trim applied by a AutoLevel flight mode and manual pitch trimming
    if (axis == FD_PITCH && STATE(AIRPLANE)) {
        /* 
         * fixedWingLevelTrim has opposite sign to rcCommand.
         * Positive rcCommand means nose should point downwards
         * Negative rcCommand mean nose should point upwards
         * This is counter intuitive and a natural way suggests that + should mean UP
         * This is why fixedWingLevelTrim has opposite sign to rcCommand
         * Positive fixedWingLevelTrim means nose should point upwards
         * Negative fixedWingLevelTrim means nose should point downwards
         */
        angleTarget -= DEGREES_TO_DECIDEGREES(fixedWingLevelTrim);   
    }

#ifdef USE_SECONDARY_IMU
    float actual;
    if (secondaryImuState.active && secondaryImuConfig()->useForStabilized) {
        if (axis == FD_ROLL) {
            actual = secondaryImuState.eulerAngles.values.roll;
        } else {
            actual = secondaryImuState.eulerAngles.values.pitch;
        }
    } else {
        actual = attitude.raw[axis];
    }

    const float angleErrorDeg = DECIDEGREES_TO_DEGREES(angleTarget - actual);
#else
    const float angleErrorDeg = DECIDEGREES_TO_DEGREES(angleTarget - attitude.raw[axis]);
#endif

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
static void pidApplySetpointRateLimiting(pidState_t *pidState, flight_dynamics_index_t axis, float dT)
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
    if (levelingEnabled) {
        return false;
    }

    return fabsf(stickPosition) > pidProfile()->fixedWingItermLimitOnStickPosition;
}

static float pTermProcess(pidState_t *pidState, float rateError, float dT) {
    float newPTerm = rateError * pidState->kP;

    return pidState->ptermFilterApplyFn(&pidState->ptermLpfState, newPTerm, yawLpfHz, dT);
}

#ifdef USE_D_BOOST
static float FAST_CODE applyDBoost(pidState_t *pidState, float dT) {
    
    float dBoost = 1.0f;
    
    if (dBoostFactor > 1) {
        const float dBoostGyroDelta = (pidState->gyroRate - pidState->previousRateGyro) / dT;
        const float dBoostGyroAcceleration = fabsf(biquadFilterApply(&pidState->dBoostGyroLpf, dBoostGyroDelta));
        const float dBoostRateAcceleration = fabsf((pidState->rateTarget - pidState->previousRateTarget) / dT);
        
        const float acceleration = MAX(dBoostGyroAcceleration, dBoostRateAcceleration);
        dBoost = scaleRangef(acceleration, 0.0f, dBoostMaxAtAlleceleration, 1.0f, dBoostFactor);
        dBoost = pt1FilterApply4(&pidState->dBoostLpf, dBoost, D_BOOST_LPF_HZ, dT);
        dBoost = constrainf(dBoost, 1.0f, dBoostFactor);
    } 

    return dBoost;
}
#else 
static float applyDBoost(pidState_t *pidState, float dT) {
    UNUSED(pidState);
    UNUSED(dT);
    return 1.0f;
}
#endif

static float dTermProcess(pidState_t *pidState, float dT) {
    // Calculate new D-term
    float newDTerm = 0;
    if (pidState->kD == 0) {
        // optimisation for when D is zero, often used by YAW axis
        newDTerm = 0;
    } else {
        float delta = pidState->previousRateGyro - pidState->gyroRate;

        delta = dTermLpfFilterApplyFn((filter_t *) &pidState->dtermLpfState, delta);
        delta = dTermLpf2FilterApplyFn((filter_t *) &pidState->dtermLpf2State, delta);

        // Calculate derivative
        newDTerm =  delta * (pidState->kD / dT) * applyDBoost(pidState, dT);
    }
    return(newDTerm);
}

static void applyItermLimiting(pidState_t *pidState) {
    if (pidState->itermLimitActive) {
        pidState->errorGyroIf = constrainf(pidState->errorGyroIf, -pidState->errorGyroIfLimit, pidState->errorGyroIfLimit);
    } else 
    {
        pidState->errorGyroIfLimit = fabsf(pidState->errorGyroIf);
    }
}

static void nullRateController(pidState_t *pidState, flight_dynamics_index_t axis, float dT) {
    UNUSED(pidState);
    UNUSED(axis);
    UNUSED(dT);
}

static void NOINLINE pidApplyFixedWingRateController(pidState_t *pidState, flight_dynamics_index_t axis, float dT)
{
    const float rateError = pidState->rateTarget - pidState->gyroRate;
    const float newPTerm = pTermProcess(pidState, rateError, dT);
    const float newDTerm = dTermProcess(pidState, dT);
    const float newFFTerm = pidState->rateTarget * pidState->kFF;

    DEBUG_SET(DEBUG_FW_D, axis, newDTerm);
    /*
     * Integral should be updated only if axis Iterm is not frozen
     */
    if (!pidState->itermFreezeActive) {
        pidState->errorGyroIf += rateError * pidState->kI * dT;
    }

    applyItermLimiting(pidState);

    if (pidProfile()->fixedWingItermThrowLimit != 0) {
        pidState->errorGyroIf = constrainf(pidState->errorGyroIf, -pidProfile()->fixedWingItermThrowLimit, pidProfile()->fixedWingItermThrowLimit);
    }

    axisPID[axis] = constrainf(newPTerm + newFFTerm + pidState->errorGyroIf, -pidState->pidSumLimit, +pidState->pidSumLimit);

#ifdef USE_AUTOTUNE_FIXED_WING
    if (FLIGHT_MODE(AUTO_TUNE) && !FLIGHT_MODE(MANUAL_MODE)) {
        autotuneFixedWingUpdate(axis, pidState->rateTarget, pidState->gyroRate, constrainf(newPTerm + newFFTerm, -pidState->pidSumLimit, +pidState->pidSumLimit));
    }
#endif

#ifdef USE_BLACKBOX
    axisPID_P[axis] = newPTerm;
    axisPID_I[axis] = pidState->errorGyroIf;
    axisPID_D[axis] = newDTerm;
    axisPID_Setpoint[axis] = pidState->rateTarget;
#endif

    pidState->previousRateGyro = pidState->gyroRate;

}

static float FAST_CODE applyItermRelax(const int axis, float currentPidSetpoint, float itermErrorRate)
{
    if (itermRelax) {
        if (axis < FD_YAW || itermRelax == ITERM_RELAX_RPY) {

            const float setpointLpf = pt1FilterApply(&windupLpf[axis], currentPidSetpoint);
            const float setpointHpf = fabsf(currentPidSetpoint - setpointLpf);
            
            const float itermRelaxFactor = MAX(0, 1 - setpointHpf / MC_ITERM_RELAX_SETPOINT_THRESHOLD);
            return itermErrorRate * itermRelaxFactor;
        }
    }

    return itermErrorRate;
}

static void FAST_CODE NOINLINE pidApplyMulticopterRateController(pidState_t *pidState, flight_dynamics_index_t axis, float dT)
{
    const float rateError = pidState->rateTarget - pidState->gyroRate;
    const float newPTerm = pTermProcess(pidState, rateError, dT);
    const float newDTerm = dTermProcess(pidState, dT);

    const float rateTargetDelta = pidState->rateTarget - pidState->previousRateTarget;
    const float rateTargetDeltaFiltered = biquadFilterApply(&pidState->rateTargetFilter, rateTargetDelta);

    /*
     * Compute Control Derivative
     * CD is enabled only when ANGLE and HORIZON modes are not enabled!
     */
    float newCDTerm;
    if (levelingEnabled) {
        newCDTerm = 0.0F;
    } else {
        newCDTerm = rateTargetDeltaFiltered * (pidState->kCD / dT);
    }
    DEBUG_SET(DEBUG_CD, axis, newCDTerm);

    // TODO: Get feedback from mixer on available correction range for each axis
    const float newOutput = newPTerm + newDTerm + pidState->errorGyroIf + newCDTerm;
    const float newOutputLimited = constrainf(newOutput, -pidState->pidSumLimit, +pidState->pidSumLimit);

    float itermErrorRate = applyItermRelax(axis, pidState->rateTarget, rateError);

#ifdef USE_ANTIGRAVITY
    itermErrorRate *= iTermAntigravityGain;
#endif

    pidState->errorGyroIf += (itermErrorRate * pidState->kI * antiWindupScaler * dT)
                             + ((newOutputLimited - newOutput) * pidState->kT * antiWindupScaler * dT);

    // Don't grow I-term if motors are at their limit
    applyItermLimiting(pidState);

    axisPID[axis] = newOutputLimited;

#ifdef USE_BLACKBOX
    axisPID_P[axis] = newPTerm;
    axisPID_I[axis] = pidState->errorGyroIf;
    axisPID_D[axis] = newDTerm;
    axisPID_Setpoint[axis] = pidState->rateTarget;
#endif

    pidState->previousRateTarget = pidState->rateTarget;
    pidState->previousRateGyro = pidState->gyroRate;
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
    if (ABS(rcCommand[YAW]) == 0 && FLIGHT_MODE(HEADING_MODE)) {
        return HEADING_HOLD_ENABLED;
    } else {
        return HEADING_HOLD_UPDATE_HEADING;
    }

    return HEADING_HOLD_UPDATE_HEADING;
}

/*
 * HEADING_HOLD P Controller returns desired rotation rate in dps to be fed to Rate controller
 */
float pidHeadingHold(float dT)
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
static void NOINLINE pidTurnAssistant(pidState_t *pidState, float bankAngleTarget, float pitchAngleTarget)
{
    fpVector3_t targetRates;
    targetRates.x = 0.0f;
    targetRates.y = 0.0f;

    if (STATE(AIRPLANE)) {
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
            bankAngleTarget = constrainf(bankAngleTarget, -DEGREES_TO_RADIANS(60), DEGREES_TO_RADIANS(60));
            float turnRatePitchAdjustmentFactor = cos_approx(fabsf(pitchAngleTarget));
            float coordinatedTurnRateEarthFrame = GRAVITY_CMSS * tan_approx(-bankAngleTarget) / airspeedForCoordinatedTurn * turnRatePitchAdjustmentFactor;

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
    pidState[PITCH].rateTarget = constrainf(pidState[PITCH].rateTarget + targetRates.y * pidProfile()->fixedWingCoordinatedPitchGain, -currentControlRateProfile->stabilized.rates[PITCH] * 10.0f, currentControlRateProfile->stabilized.rates[PITCH] * 10.0f);

    // Replace YAW on quads - add it in on airplanes
    if (STATE(AIRPLANE)) {
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

void checkItermLimitingActive(pidState_t *pidState)
{
    bool shouldActivate;
    if (usedPidControllerType == PID_TYPE_PIFF) {
        shouldActivate = isFixedWingItermLimitActive(pidState->stickPosition);
    } else 
    {
        shouldActivate = mixerIsOutputSaturated();
    }

    pidState->itermLimitActive = STATE(ANTI_WINDUP) || shouldActivate; 
}

void checkItermFreezingActive(pidState_t *pidState, flight_dynamics_index_t axis)
{
    if (usedPidControllerType == PID_TYPE_PIFF && pidProfile()->fixedWingYawItermBankFreeze != 0 && axis == FD_YAW) {
        // Do not allow yaw I-term to grow when bank angle is too large
        float bankAngle = DECIDEGREES_TO_DEGREES(attitude.values.roll);
        if (fabsf(bankAngle) > pidProfile()->fixedWingYawItermBankFreeze && !(FLIGHT_MODE(AUTO_TUNE) || FLIGHT_MODE(TURN_ASSISTANT) || navigationRequiresTurnAssistance())){
            pidState->itermFreezeActive = true;
        } else
        {
            pidState->itermFreezeActive = false;
        }
    } else
    {
        pidState->itermFreezeActive = false;
    }
    
}

void FAST_CODE pidController(float dT)
{
    if (!pidFiltersConfigured) {
        return;
    }

    bool canUseFpvCameraMix = true;
    uint8_t headingHoldState = getHeadingHoldState();

    if (headingHoldState == HEADING_HOLD_UPDATE_HEADING) {
        updateHeadingHoldTarget(DECIDEGREES_TO_DEGREES(attitude.values.yaw));
    }

    for (int axis = 0; axis < 3; axis++) {
        // Step 1: Calculate gyro rates
        pidState[axis].gyroRate = gyro.gyroADCf[axis];

        // Step 2: Read target
        float rateTarget;

        if (axis == FD_YAW && headingHoldState == HEADING_HOLD_ENABLED) {
            rateTarget = pidHeadingHold(dT);
        } else {
#ifdef USE_PROGRAMMING_FRAMEWORK
            rateTarget = pidRcCommandToRate(getRcCommandOverride(rcCommand, axis), currentControlRateProfile->stabilized.rates[axis]);
#else 
            rateTarget = pidRcCommandToRate(rcCommand[axis], currentControlRateProfile->stabilized.rates[axis]);
#endif
        }

        // Limit desired rate to something gyro can measure reliably
        pidState[axis].rateTarget = constrainf(rateTarget, -GYRO_SATURATION_LIMIT, +GYRO_SATURATION_LIMIT);

#ifdef USE_GYRO_KALMAN
        if (pidProfile()->kalmanEnabled) {
            pidState[axis].gyroRate = gyroKalmanUpdate(axis, pidState[axis].gyroRate, pidState[axis].rateTarget);
        }
#endif

#ifdef USE_SMITH_PREDICTOR
        DEBUG_SET(DEBUG_SMITH_PREDICTOR, axis, pidState[axis].gyroRate);
        pidState[axis].gyroRate = applySmithPredictor(axis, &pidState[axis].smithPredictor, pidState[axis].gyroRate);
        DEBUG_SET(DEBUG_SMITH_PREDICTOR, axis + 3, pidState[axis].gyroRate);
#endif

        DEBUG_SET(DEBUG_PID_MEASUREMENT, axis, pidState[axis].gyroRate);
    }

    // Step 3: Run control for ANGLE_MODE, HORIZON_MODE, and HEADING_LOCK
    if (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE)) {
        const float horizonRateMagnitude = calcHorizonRateMagnitude();
        pidLevel(&pidState[FD_ROLL], FD_ROLL, horizonRateMagnitude, dT);
        pidLevel(&pidState[FD_PITCH], FD_PITCH, horizonRateMagnitude, dT);
        canUseFpvCameraMix = false;     // FPVANGLEMIX is incompatible with ANGLE/HORIZON
        levelingEnabled = true;
    } else {
        levelingEnabled = false;
    }

    if ((FLIGHT_MODE(TURN_ASSISTANT) || navigationRequiresTurnAssistance()) && (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE) || navigationRequiresTurnAssistance())) {
        float bankAngleTarget = DECIDEGREES_TO_RADIANS(pidRcCommandToAngle(rcCommand[FD_ROLL], pidProfile()->max_angle_inclination[FD_ROLL]));
        float pitchAngleTarget = DECIDEGREES_TO_RADIANS(pidRcCommandToAngle(rcCommand[FD_PITCH], pidProfile()->max_angle_inclination[FD_PITCH]));
        pidTurnAssistant(pidState, bankAngleTarget, pitchAngleTarget);
        canUseFpvCameraMix = false;     // FPVANGLEMIX is incompatible with TURN_ASSISTANT
    }

    if (canUseFpvCameraMix && IS_RC_MODE_ACTIVE(BOXFPVANGLEMIX) && currentControlRateProfile->misc.fpvCamAngleDegrees) {
        pidApplyFpvCameraAngleMix(pidState, currentControlRateProfile->misc.fpvCamAngleDegrees);
    }

    // Prevent strong Iterm accumulation during stick inputs
    antiWindupScaler = constrainf((1.0f - getMotorMixRange()) / motorItermWindupPoint, 0.0f, 1.0f);

    for (int axis = 0; axis < 3; axis++) {
        // Apply setpoint rate of change limits
        pidApplySetpointRateLimiting(&pidState[axis], axis, dT);
        
        // Step 4: Run gyro-driven control
        checkItermLimitingActive(&pidState[axis]);
        checkItermFreezingActive(&pidState[axis], axis);

        pidControllerApplyFn(&pidState[axis], axis, dT);
    }
}

pidType_e pidIndexGetType(pidIndex_e pidIndex)
{
    if (pidIndex == PID_ROLL || pidIndex == PID_PITCH || pidIndex == PID_YAW) {
        return usedPidControllerType;    
    }
    if (STATE(AIRPLANE) || STATE(ROVER) || STATE(BOAT)) {
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

void pidInit(void)
{
    // Calculate max overall tilt (max pitch + max roll combined) as a limit to heading hold
    headingHoldCosZLimit = cos_approx(DECIDEGREES_TO_RADIANS(pidProfile()->max_angle_inclination[FD_ROLL])) *
                           cos_approx(DECIDEGREES_TO_RADIANS(pidProfile()->max_angle_inclination[FD_PITCH]));

    pidGainsUpdateRequired = false;

    itermRelax = pidProfile()->iterm_relax;

    yawLpfHz = pidProfile()->yaw_lpf_hz;
    motorItermWindupPoint = 1.0f - (pidProfile()->itermWindupPointPercent / 100.0f);

#ifdef USE_D_BOOST
    dBoostFactor = pidProfile()->dBoostFactor;
    dBoostMaxAtAlleceleration = pidProfile()->dBoostMaxAtAlleceleration;
#endif

#ifdef USE_ANTIGRAVITY
    antigravityGain = pidProfile()->antigravityGain;
    antigravityAccelerator = pidProfile()->antigravityAccelerator;
#endif
    
    for (uint8_t axis = FD_ROLL; axis <= FD_YAW; axis++) {
        if (axis == FD_YAW) {
            pidState[axis].pidSumLimit = pidProfile()->pidSumLimitYaw;
            if (yawLpfHz) {
                pidState[axis].ptermFilterApplyFn = (filterApply4FnPtr) pt1FilterApply4;
            } else {
                pidState[axis].ptermFilterApplyFn = (filterApply4FnPtr) nullFilterApply4;
            }
        } else {
            pidState[axis].pidSumLimit = pidProfile()->pidSumLimit;
            pidState[axis].ptermFilterApplyFn = (filterApply4FnPtr) nullFilterApply4;
        }
    }

    if (pidProfile()->pidControllerType == PID_TYPE_AUTO) {
        if (
            mixerConfig()->platformType == PLATFORM_AIRPLANE || 
            mixerConfig()->platformType == PLATFORM_BOAT ||
            mixerConfig()->platformType == PLATFORM_ROVER
        ) {
            usedPidControllerType = PID_TYPE_PIFF;
        } else {
            usedPidControllerType = PID_TYPE_PID;
        }
    } else {
        usedPidControllerType = pidProfile()->pidControllerType;
    }

    dTermLpfFilterApplyFn = nullFilterApply;
    if (pidProfile()->dterm_lpf_hz) {
        if (pidProfile()->dterm_lpf_type == FILTER_PT1) {
            dTermLpfFilterApplyFn = (filterApplyFnPtr) pt1FilterApply;
        } else {
            dTermLpfFilterApplyFn = (filterApplyFnPtr) biquadFilterApply;
        }
    }

    dTermLpf2FilterApplyFn = nullFilterApply;
    if (pidProfile()->dterm_lpf2_hz) {
        if (pidProfile()->dterm_lpf2_type == FILTER_PT1) {
            dTermLpf2FilterApplyFn = (filterApplyFnPtr) pt1FilterApply;
        } else {
            dTermLpf2FilterApplyFn = (filterApplyFnPtr) biquadFilterApply;
        }
    }

    if (usedPidControllerType == PID_TYPE_PIFF) {
        pidControllerApplyFn = pidApplyFixedWingRateController;
    } else if (usedPidControllerType == PID_TYPE_PID) {
        pidControllerApplyFn = pidApplyMulticopterRateController;
    } else {
        pidControllerApplyFn = nullRateController;
    }

    pidResetTPAFilter();
#ifdef USE_GYRO_KALMAN
    if (pidProfile()->kalmanEnabled) {
        gyroKalmanInitialize(pidProfile()->kalman_q, pidProfile()->kalman_w, pidProfile()->kalman_sharpness);
    }
#endif

    fixedWingLevelTrim = pidProfile()->fixedWingLevelTrim;

    navPidInit(
        &fixedWingLevelTrimController,
        0.0f,
        (float)pidProfile()->fixedWingLevelTrimGain / 100000.0f,
        0.0f,
        0.0f,
        2.0f
    );

}

const pidBank_t * pidBank(void) { 
    return usedPidControllerType == PID_TYPE_PIFF ? &pidProfile()->bank_fw : &pidProfile()->bank_mc; 
}
pidBank_t * pidBankMutable(void) { 
    return usedPidControllerType == PID_TYPE_PIFF ? &pidProfileMutable()->bank_fw : &pidProfileMutable()->bank_mc;
}

void updateFixedWingLevelTrim(timeUs_t currentTimeUs)
{
    if (!STATE(AIRPLANE)) {
        return;
    }

    static timeUs_t previousUpdateTimeUs;
    static bool previousArmingState;
    const float dT = US2S(currentTimeUs - previousUpdateTimeUs);

    /*
     * On every ARM reset the controller
     */
    if (ARMING_FLAG(ARMED) && !previousArmingState) {
        navPidReset(&fixedWingLevelTrimController);
    } 

    /*
     * On disarm update the default value
     */
    if (!ARMING_FLAG(ARMED) && previousArmingState) {
        pidProfileMutable()->fixedWingLevelTrim = constrainf(fixedWingLevelTrim, -FIXED_WING_LEVEL_TRIM_MAX_ANGLE, FIXED_WING_LEVEL_TRIM_MAX_ANGLE);
    } 

    /*
     * Prepare flags for the PID controller
     */
    pidControllerFlags_e flags = PID_LIMIT_INTEGRATOR;

    //Iterm should freeze when pitch stick is deflected
    if (
        !IS_RC_MODE_ACTIVE(BOXAUTOLEVEL) ||
        rxGetChannelValue(PITCH) > (PWM_RANGE_MIDDLE + pidProfile()->fixedWingLevelTrimDeadband) ||
        rxGetChannelValue(PITCH) < (PWM_RANGE_MIDDLE - pidProfile()->fixedWingLevelTrimDeadband) ||
        (!FLIGHT_MODE(ANGLE_MODE) && !FLIGHT_MODE(HORIZON_MODE)) ||
        navigationIsControllingAltitude()
    ) {
        flags |= PID_FREEZE_INTEGRATOR;
    }

    const float output = navPidApply3(
        &fixedWingLevelTrimController,
        0,  //Setpoint is always 0 as we try to keep level flight
        getEstimatedActualVelocity(Z),
        dT,
        -FIXED_WING_LEVEL_TRIM_CONTROLLER_LIMIT,
        FIXED_WING_LEVEL_TRIM_CONTROLLER_LIMIT,
        flags,
        1.0f,
        1.0f
    );

    DEBUG_SET(DEBUG_AUTOLEVEL, 4, output);
    fixedWingLevelTrim = pidProfile()->fixedWingLevelTrim + (output * FIXED_WING_LEVEL_TRIM_MULTIPLIER);

    previousArmingState = !!ARMING_FLAG(ARMED);
}
