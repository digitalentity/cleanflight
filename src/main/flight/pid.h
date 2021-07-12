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

#pragma once

#include "config/parameter_group.h"
#include "fc/runtime_config.h"
#include "common/time.h"

#define GYRO_SATURATION_LIMIT       1800        // 1800dps
#define PID_SUM_LIMIT_MIN           100
#define PID_SUM_LIMIT_MAX           1000
#define PID_SUM_LIMIT_DEFAULT       500
#define PID_SUM_LIMIT_YAW_DEFAULT   400

#define HEADING_HOLD_RATE_LIMIT_MIN 10
#define HEADING_HOLD_RATE_LIMIT_MAX 250
#define HEADING_HOLD_RATE_LIMIT_DEFAULT 90

#define FW_ITERM_THROW_LIMIT_DEFAULT 165
#define FW_ITERM_THROW_LIMIT_MIN 0
#define FW_ITERM_THROW_LIMIT_MAX 500

#define AXIS_ACCEL_MIN_LIMIT        50

#define HEADING_HOLD_ERROR_LPF_FREQ 2

/*
FP-PID has been rescaled to match LuxFloat (and MWRewrite) from Cleanflight 1.13
*/
#define FP_PID_RATE_FF_MULTIPLIER   31.0f
#define FP_PID_RATE_P_MULTIPLIER    31.0f
#define FP_PID_RATE_I_MULTIPLIER    4.0f
#define FP_PID_RATE_D_MULTIPLIER    1905.0f
#define FP_PID_RATE_D_FF_MULTIPLIER   7270.0f
#define FP_PID_LEVEL_P_MULTIPLIER   6.56f       // Level P gain units is [1/sec] and angle error is [deg] => [deg/s]
#define FP_PID_YAWHOLD_P_MULTIPLIER 80.0f

#define MC_ITERM_RELAX_SETPOINT_THRESHOLD 40.0f
#define MC_ITERM_RELAX_CUTOFF_DEFAULT 15

#define ANTI_GRAVITY_THROTTLE_FILTER_CUTOFF 15  // The anti gravity throttle highpass filter cutoff

#define FIXED_WING_LEVEL_TRIM_DEADBAND_DEFAULT 5

#define TASK_AUX_RATE_HZ   100 //In Hz

typedef enum {
    /* PID              MC      FW  */
    PID_ROLL,       //   +       +
    PID_PITCH,      //   +       +
    PID_YAW,        //   +       +
    PID_POS_Z,      //   +       +
    PID_POS_XY,     //   +       +
    PID_VEL_XY,     //   +       n/a
    PID_SURFACE,    //   n/a     n/a
    PID_LEVEL,      //   +       +
    PID_HEADING,    //   +       +
    PID_VEL_Z,      //   +       n/a
    PID_POS_HEADING,//   n/a     +
    PID_ITEM_COUNT
} pidIndex_e;

// TODO(agh): PIDFF
typedef enum {
    PID_TYPE_NONE = 0,  // Not used in the current platform/mixer/configuration
    PID_TYPE_PID,   // Uses P, I and D terms
    PID_TYPE_PIFF,  // Uses P, I, D and FF
    PID_TYPE_AUTO,  // Autodetect
} pidType_e;

typedef struct pid8_s {
    uint16_t P;
    uint16_t I;
    uint16_t D;
    uint16_t FF;
} pid8_t;

typedef struct pidBank_s {
    pid8_t  pid[PID_ITEM_COUNT];
} pidBank_t;

typedef enum {
    ITERM_RELAX_OFF = 0,
    ITERM_RELAX_RP,
    ITERM_RELAX_RPY
} itermRelax_e;

typedef struct pidProfile_s {
    uint8_t pidControllerType;
    pidBank_t bank_fw;
    pidBank_t bank_mc;

    uint8_t dterm_lpf_type;                 // Dterm LPF type: PT1, BIQUAD
    uint16_t dterm_lpf_hz;                  
    
    uint8_t dterm_lpf2_type;                // Dterm LPF type: PT1, BIQUAD
    uint16_t dterm_lpf2_hz;                 
    
    uint8_t yaw_lpf_hz;

    uint8_t heading_hold_rate_limit;        // Maximum rotation rate HEADING_HOLD mode can feed to yaw rate PID controller

    uint8_t itermWindupPointPercent;        // Experimental ITerm windup threshold, percent of motor saturation

    uint32_t axisAccelerationLimitYaw;          // Max rate of change of yaw angular rate setpoint (deg/s^2 = dps/s)
    uint32_t axisAccelerationLimitRollPitch;    // Max rate of change of roll/pitch angular rate setpoint (deg/s^2 = dps/s)

    int16_t max_angle_inclination[ANGLE_INDEX_COUNT];       // Max possible inclination (roll and pitch axis separately

    uint16_t pidSumLimit;
    uint16_t pidSumLimitYaw;

    // Airplane-specific parameters
    uint16_t    fixedWingItermThrowLimit;
    float       fixedWingReferenceAirspeed;     // Reference tuning airspeed for the airplane - the speed for which PID gains are tuned
    float       fixedWingCoordinatedYawGain;    // This is the gain of the yaw rate required to keep the yaw rate consistent with the turn rate for a coordinated turn.
    float       fixedWingCoordinatedPitchGain;    // This is the gain of the pitch rate to keep the pitch angle constant during coordinated turns.
    float       fixedWingItermLimitOnStickPosition;   //Do not allow Iterm to grow when stick position is above this point
    uint16_t    fixedWingYawItermBankFreeze;       // Freeze yaw Iterm when bank angle is more than this many degrees

    uint8_t     loiter_direction;               // Direction of loitering center point on right wing (clockwise - as before), or center point on left wing (counterclockwise)
    float       navVelXyDTermLpfHz;
    uint8_t navVelXyDtermAttenuation;       // VEL_XY dynamic Dterm scale: Dterm will be attenuatedby this value (in percent) when UAV is traveling with more than navVelXyDtermAttenuationStart percents of max velocity
    uint8_t navVelXyDtermAttenuationStart;  // VEL_XY dynamic Dterm scale: Dterm attenuation will begin at this percent of max velocity
    uint8_t navVelXyDtermAttenuationEnd;    // VEL_XY dynamic Dterm scale: Dterm will be fully attenuated at this percent of max velocity
    uint8_t iterm_relax_cutoff;             // This cutoff frequency specifies a low pass filter which predicts average response of the quad to setpoint
    uint8_t iterm_relax;                    // Enable iterm suppression during stick input

#ifdef USE_D_BOOST
    float dBoostMin;
    float dBoostMax;
    float dBoostMaxAtAlleceleration;
    uint8_t dBoostGyroDeltaLpfHz;
#endif

#ifdef USE_ANTIGRAVITY
    float antigravityGain;
    float antigravityAccelerator;
    uint8_t antigravityCutoff;
#endif

    uint16_t navFwPosHdgPidsumLimit;
    uint8_t controlDerivativeLpfHz;

    float fixedWingLevelTrim;
    float fixedWingLevelTrimGain;
#ifdef USE_SMITH_PREDICTOR
    float smithPredictorStrength;
    float smithPredictorDelay;
    uint16_t smithPredictorFilterHz;
#endif
} pidProfile_t;

typedef struct pidAutotuneConfig_s {
    uint16_t    fw_detect_time;             // Time [ms] to detect sustained undershoot or overshoot
    uint8_t     fw_min_stick;               // Minimum stick input required to update rates and gains
    uint8_t     fw_ff_to_p_gain;            // FF to P gain (strength relationship) [%]
    uint8_t     fw_p_to_d_gain;             // P to D gain (strength relationship) [%]
    uint16_t    fw_ff_to_i_time_constant;   // FF to I time (defines time for I to reach the same level of response as FF) [ms]
    uint8_t     fw_rate_adjustment;         // Adjust rate settings during autotune?
    uint8_t     fw_max_rate_deflection;     // Percentage of max mixer output used for calculating the rates
} pidAutotuneConfig_t;

typedef enum {
    FIXED,
    LIMIT,
    AUTO,
} fw_autotune_rate_adjustment_e;

PG_DECLARE_PROFILE(pidProfile_t, pidProfile);
PG_DECLARE(pidAutotuneConfig_t, pidAutotuneConfig);

const pidBank_t * pidBank(void);
pidBank_t * pidBankMutable(void);

extern int16_t axisPID[];
extern int32_t axisPID_P[], axisPID_I[], axisPID_D[], axisPID_Setpoint[];

void pidInit(void);
bool pidInitFilters(void);
void pidResetErrorAccumulators(void);
void pidReduceErrorAccumulators(int8_t delta, uint8_t axis);
float getAxisIterm(uint8_t axis);
float getTotalRateTarget(void);
void pidResetTPAFilter(void);

struct controlRateConfig_s;
struct motorConfig_s;
struct rxConfig_s;

void schedulePidGainsUpdate(void);
void updatePIDCoefficients(void);
void pidController(float dT);

float pidRateToRcCommand(float rateDPS, uint8_t rate);
int16_t pidAngleToRcCommand(float angleDeciDegrees, int16_t maxInclination);

enum {
    HEADING_HOLD_DISABLED = 0,
    HEADING_HOLD_UPDATE_HEADING,
    HEADING_HOLD_ENABLED
};

void updateHeadingHoldTarget(int16_t heading);
void resetHeadingHoldTarget(int16_t heading);
int16_t getHeadingHoldTarget(void);

void autotuneUpdateState(void);
void autotuneFixedWingUpdate(const flight_dynamics_index_t axis, float desiredRateDps, float reachedRateDps, float pidOutput);

pidType_e pidIndexGetType(pidIndex_e pidIndex);

void updateFixedWingLevelTrim(timeUs_t currentTimeUs);
float getFixedWingLevelTrim(void);
