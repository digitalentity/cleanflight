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
#include <string.h>

#include "platform.h"

#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"
#include "common/utils.h"

#include "config/feature.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/pwm_output.h"
#include "drivers/pwm_mapping.h"
#include "drivers/time.h"

#include "fc/config.h"
#include "fc/rc_control.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/servos.h"

#include "navigation/navigation.h"

#include "rx/rx.h"

#include "sensors/battery.h"

FASTRAM int16_t motor[MAX_SUPPORTED_MOTORS];
// This is used by the configurator for motor testing.
FASTRAM int16_t motor_disarmed[MAX_SUPPORTED_MOTORS];

static float motorMixRange;
static float mixerScale = 1.0f;
static EXTENDED_FASTRAM motorMixer_t currentMixer[MAX_SUPPORTED_MOTORS];
static EXTENDED_FASTRAM uint8_t motorCount = 0;

// Rate limiting
static float motorMaxInc;
static float motorMaxDec;
static uint16_t motorRange;
static float motorPrevious[MAX_SUPPORTED_MOTORS];

PG_REGISTER_WITH_RESET_TEMPLATE(flight3DConfig_t, flight3DConfig, PG_MOTOR_3D_CONFIG, 0);

PG_RESET_TEMPLATE(flight3DConfig_t, flight3DConfig,
    .deadband3d_low = 1406,
    .deadband3d_high = 1514,
    .neutral3d = 1460
);

PG_REGISTER_WITH_RESET_TEMPLATE(mixerConfig_t, mixerConfig, PG_MIXER_CONFIG, 1);

PG_RESET_TEMPLATE(mixerConfig_t, mixerConfig,
    .yaw_motor_direction = 1,
    .yaw_jump_prevention_limit = 200,
    .platformType = PLATFORM_MULTIROTOR,
    .hasFlaps = false,
    .appliedMixerPreset = -1, //This flag is not available in CLI and used by Configurator only
    .fwMinThrottleDownPitchAngle = 0
);

#ifdef BRUSHED_MOTORS
#define DEFAULT_PWM_PROTOCOL    PWM_TYPE_BRUSHED
#define DEFAULT_PWM_RATE        16000
#define DEFAULT_MIN_THROTTLE    1000
#else
#define DEFAULT_PWM_PROTOCOL    PWM_TYPE_STANDARD
#define DEFAULT_PWM_RATE        400
#define DEFAULT_MIN_THROTTLE    1150
#endif

PG_REGISTER_WITH_RESET_TEMPLATE(motorConfig_t, motorConfig, PG_MOTOR_CONFIG, 3);

PG_RESET_TEMPLATE(motorConfig_t, motorConfig,
    .minthrottle = DEFAULT_MIN_THROTTLE,
    .motorPwmProtocol = DEFAULT_PWM_PROTOCOL,
    .motorPwmRate = DEFAULT_PWM_RATE,
    .maxthrottle = 1850,
    .mincommand = 1000,
    .motorAccelTimeMs = 0,
    .motorDecelTimeMs = 0,
    .digitalIdleOffsetValue = 450,   // Same scale as in Betaflight
    .motorBidirectionalReverse = 0,
);

PG_REGISTER_ARRAY(motorMixer_t, MAX_SUPPORTED_MOTORS, primaryMotorMixer, PG_MOTOR_MIXER, 0);

static void computeMotorCount(void)
{
    motorCount = 0;
    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        // check if done
        if (primaryMotorMixer(i)->throttle == 0.0f) {
            break;
        }
        motorCount++;
    }
}

uint8_t FAST_CODE NOINLINE getMotorCount(void) {
    return motorCount;
}

float getMotorMixRange(void)
{
    return motorMixRange;
}

bool mixerIsOutputSaturated(void)
{
    return motorMixRange >= 1.0f;
}

bool mixerCanReverseMotors(void)
{
    return feature(FEATURE_BIDIR_MOTORS);
}

bool mixerIsReversingMotors(void)
{
    return mixerCanReverseMotors() && rcControlGetOutputAxis(THROTTLE) < 0;
}

void mixerUpdateStateFlags(void)
{
    // set flag that we're on something with wings
    if (mixerConfig()->platformType == PLATFORM_AIRPLANE) {
        ENABLE_STATE(FIXED_WING);
    } else if (mixerConfig()->platformType == PLATFORM_HELICOPTER) {
        DISABLE_STATE(FIXED_WING);
    } else {
        DISABLE_STATE(FIXED_WING);
    }

    if (mixerConfig()->hasFlaps) {
        ENABLE_STATE(FLAPERON_AVAILABLE);
    } else {
        DISABLE_STATE(FLAPERON_AVAILABLE);
    }
}

static float mixerCalculateRateLimiterStep(uint16_t motorRange, uint16_t timeMs)
{
    if (timeMs == 0) {
        // No time limit, allow full range on a single step
        return PWM_RANGE_MAX - PWM_RANGE_MIN;
    }
    return motorRange / (timeMs * 1e-3f);
}

void mixerInit(void)
{
    computeMotorCount();
    loadPrimaryMotorMixer();
    // TODO: We can probably get rid of this
    // in 3D mode, mixer gain has to be halved
    if (feature(FEATURE_BIDIR_MOTORS)) {
        mixerScale = 0.5f;
    }
    motorRange = motorConfig()->maxthrottle - motorConfig()->minthrottle;
    motorMaxInc = mixerCalculateRateLimiterStep(motorRange, motorConfig()->motorAccelTimeMs);
    motorMaxDec = mixerCalculateRateLimiterStep(motorRange, motorConfig()->motorDecelTimeMs);

    mixerResetDisarmedMotors();
}

static int16_t mixerMotorStoppedPWMValue(void)
{
    if (feature(FEATURE_BIDIR_MOTORS)) {
        return flight3DConfig()->neutral3d;
    }
    return motorConfig()->mincommand;
}

static int16_t mixerMotorMinimumSpinPWMValue(void)
{
    if (feature(FEATURE_BIDIR_MOTORS)) {
        return flight3DConfig()->deadband3d_high;
    }
    return motorConfig()->minthrottle;
}

void mixerResetDisarmedMotors(void)
{
    // set disarmed motor values
    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        motor_disarmed[i] = mixerMotorStoppedPWMValue();
    }
}

void FAST_CODE NOINLINE writeMotors(void)
{
    for (int i = 0; i < motorCount; i++) {
        uint16_t motorValue;

#ifdef USE_DSHOT
        // If we use DSHOT we need to convert motorValue to DSHOT ranges
        if (isMotorProtocolDigital()) {
            const float dshotMinThrottleOffset = (DSHOT_MAX_THROTTLE - DSHOT_MIN_THROTTLE) / 10000.0f * motorConfig()->digitalIdleOffsetValue;

            if (feature(FEATURE_BIDIR_MOTORS)) {
                if (motor[i] >= motorConfig()->minthrottle && motor[i] <= flight3DConfig()->deadband3d_low) {
                    motorValue = scaleRangef(motor[i], motorConfig()->minthrottle, flight3DConfig()->deadband3d_low, DSHOT_3D_DEADBAND_LOW, dshotMinThrottleOffset + DSHOT_MIN_THROTTLE);
                    motorValue = constrain(motorValue, DSHOT_MIN_THROTTLE, DSHOT_3D_DEADBAND_LOW);
                }
                else if (motor[i] >= flight3DConfig()->deadband3d_high && motor[i] <= motorConfig()->maxthrottle) {
                    motorValue = scaleRangef(motor[i], flight3DConfig()->deadband3d_high, motorConfig()->maxthrottle, dshotMinThrottleOffset + DSHOT_3D_DEADBAND_HIGH, DSHOT_MAX_THROTTLE);
                    motorValue = constrain(motorValue, DSHOT_3D_DEADBAND_HIGH, DSHOT_MAX_THROTTLE);
                }
                else {
                    motorValue = DSHOT_DISARM_COMMAND;
                }
            }
            else {
                if (motor[i] < motorConfig()->minthrottle) {    // motor disarmed
                    motorValue = DSHOT_DISARM_COMMAND;
                }
                else {
                    motorValue = scaleRangef(motor[i], motorConfig()->minthrottle, motorConfig()->maxthrottle, (dshotMinThrottleOffset + DSHOT_MIN_THROTTLE), DSHOT_MAX_THROTTLE);
                    motorValue = constrain(motorValue, (dshotMinThrottleOffset + DSHOT_MIN_THROTTLE), DSHOT_MAX_THROTTLE);
                }
            }
        }
        else {
            motorValue = motor[i];
        }
#else
        // We don't define USE_DSHOT
        motorValue = motor[i];
#endif

        pwmWriteMotor(i, motorValue);
    }
}

void writeAllMotors(int16_t mc)
{
    // Sends commands to all motors
    for (int i = 0; i < motorCount; i++) {
        motor[i] = mc;
    }
    writeMotors();
}

void stopMotors(void)
{
    writeAllMotors(mixerMotorStoppedPWMValue());

    delay(50); // give the timers and ESCs a chance to react.
}

void stopPwmAllMotors(void)
{
    pwmShutdownPulsesForAllMotors(motorCount);
}

static void applyMotorRateLimiting(const float dT)
{
    // Calculate max motor step
    const float maxInc = motorMaxInc * dT;
    const float maxDec = motorMaxDec * dT;

    if (feature(FEATURE_BIDIR_MOTORS)) {
        for (int i = 0; i < motorCount; i++) {
            if (motorPrevious[i] >= flight3DConfig()->neutral3d) {
                // Higher values increase motor speed
                motorPrevious[i] = constrainf(motor[i], motorPrevious[i] - maxDec, motorPrevious[i] + maxInc);
            } else {
                // Lower values increase motor speed
                motorPrevious[i] = constrainf(motor[i], motorPrevious[i] - maxInc, motorPrevious[i] + maxDec);
            }
            if (motorPrevious[i] > flight3DConfig()->deadband3d_low && motorPrevious[i] < flight3DConfig()->deadband3d_high) {
                if (motor[i] > flight3DConfig()->deadband3d_low && motor[i] < flight3DConfig()->deadband3d_high) {
                    motorPrevious[i] = motor[i];
                } else {
                    if (motor[i] <= flight3DConfig()->deadband3d_low) {
                        motorPrevious[i] = flight3DConfig()->deadband3d_low;
                    } else { // motor[i] >= flight3DConfig()->deadband3d_high
                        motorPrevious[i] = flight3DConfig()->deadband3d_high;
                    }
                }
            }
        }
        // FIXME: Don't apply rate limiting in 3D mode
        for (int i = 0; i < motorCount; i++) {
            motorPrevious[i] = motor[i];
        }
    }
    else {
        for (int i = 0; i < motorCount; i++) {
            // Apply motor rate limiting
            motorPrevious[i] = constrainf(motor[i], motorPrevious[i] - maxDec, motorPrevious[i] + maxInc);

            // Handle throttle below min_throttle (motor start/stop)
            if (motorPrevious[i] < motorConfig()->minthrottle) {
                if (motor[i] < motorConfig()->minthrottle) {
                    motorPrevious[i] = motor[i];
                }
                else {
                    motorPrevious[i] = motorConfig()->minthrottle;
                }
            }
        }
    }

    // Update motor values
    for (int i = 0; i < motorCount; i++) {
        motor[i] = motorPrevious[i];
    }
}

void FAST_CODE NOINLINE mixTable(const float dT)
{
    int16_t input[3];   // RPY, range [-500:+500]
    // Allow direct stick input to motors in passthrough mode on airplanes
    if (STATE(FIXED_WING) && FLIGHT_MODE(MANUAL_MODE)) {
        // Direct passthru from RX
        input[ROLL] = rcControlGetOutputAxis(ROLL) * 500.0f;
        input[PITCH] = rcControlGetOutputAxis(PITCH) * 500.0f;
        input[YAW] = rcControlGetOutputAxis(YAW) * 500.0f;
    }
    else {
        input[ROLL] = axisPID[ROLL];
        input[PITCH] = axisPID[PITCH];
        input[YAW] = axisPID[YAW];

        if (motorCount >= 4 && mixerConfig()->yaw_jump_prevention_limit < YAW_JUMP_PREVENTION_LIMIT_HIGH) {
            // prevent "yaw jump" during yaw correction
            int16_t yawDelta = fabsf(rcControlGetOutputAxis(YAW)) * 500.0f;
            int16_t yawMin = -mixerConfig()->yaw_jump_prevention_limit - yawDelta;
            int16_t yawMax = mixerConfig()->yaw_jump_prevention_limit + yawDelta;
            input[YAW] = constrain(input[YAW], yawMin, yawMax);
        }
    }

    // Initial mixer concept by bdoiron74 reused and optimized for Air Mode
    int16_t rpyMix[MAX_SUPPORTED_MOTORS];
    int16_t rpyMixMax = 0; // assumption: symetrical about zero.
    int16_t rpyMixMin = 0;

    // motors for non-servo mixes
    for (int i = 0; i < motorCount; i++) {
        rpyMix[i] =
            (input[PITCH] * currentMixer[i].pitch +
            input[ROLL] * currentMixer[i].roll +
            -mixerConfig()->yaw_motor_direction * input[YAW] * currentMixer[i].yaw) * mixerScale;

        if (rpyMix[i] > rpyMixMax) rpyMixMax = rpyMix[i];
        if (rpyMix[i] < rpyMixMin) rpyMixMin = rpyMix[i];
    }

    int16_t rpyMixRange = rpyMixMax - rpyMixMin;
    int16_t throttleRange, throttleCommand;
    int16_t throttleMin, throttleMax;

    // Store the last throttle direction for deadband transitions
    static float throttlePreviousOutput = 0;

    // Find min and max throttle based on condition.
    float throttleOutput = rcControlGetOutputAxis(THROTTLE);

    if (mixerCanReverseMotors()) {
        if (motorConfig()->motorBidirectionalReverse) {
            throttleOutput = -throttleOutput;
        }
        if (throttleOutput > 0 ||
            (throttleOutput == 0 && throttlePreviousOutput > 0) ||
            (throttleOutput == 0 && throttlePreviousOutput == 0 && !motorConfig()->motorBidirectionalReverse)) { // Positive handling

            throttleMin = flight3DConfig()->deadband3d_high;
            throttleMax = motorConfig()->maxthrottle;

            throttleCommand = throttleMin + (throttleMax - throttleMin) * throttleOutput;
        } else {
            throttleMin = motorConfig()->minthrottle;
            throttleMax = flight3DConfig()->deadband3d_low;

            throttleCommand = throttleMin + (throttleMax - throttleMin) * (RC_COMMAND_MAX + throttleOutput);
        }
    } else {
        // Clip to [0, 1]
        throttleOutput = constrainf(throttleOutput, RC_COMMAND_CENTER, RC_COMMAND_MAX);

        throttleMin = motorConfig()->minthrottle;
        throttleMax = motorConfig()->maxthrottle;

        // Map to [minthrottle, maxthrottle]
        throttleCommand = throttleMin + (throttleMax - throttleMin) * throttleOutput;
    }

    // Throttle compensation based on battery voltage
    if (feature(FEATURE_THR_VBAT_COMP) && feature(FEATURE_VBAT) && isAmperageConfigured()) {
        throttleCommand = MIN(throttleMin + (throttleCommand - throttleMin) * calculateThrottleCompensationFactor(), throttleMax);
    }

    throttlePreviousOutput = throttleOutput;

    throttleRange = throttleMax - throttleMin;

    #define THROTTLE_CLIPPING_FACTOR    0.33f
    motorMixRange = (float)rpyMixRange / (float)throttleRange;
    uint16_t clippedThrottleMin;
    uint16_t clippedThrottleMax;
    if (motorMixRange > 1.0f) {
        for (int i = 0; i < motorCount; i++) {
            rpyMix[i] /= motorMixRange;
        }

        // Allow some clipping on edges to soften correction response
        clippedThrottleMin = throttleMin + (throttleRange / 2) - (throttleRange * THROTTLE_CLIPPING_FACTOR / 2);
        clippedThrottleMax = throttleMin + (throttleRange / 2) + (throttleRange * THROTTLE_CLIPPING_FACTOR / 2);
    } else {
        clippedThrottleMin = MIN(throttleMin + (rpyMixRange / 2), throttleMin + (throttleRange / 2) - (throttleRange * THROTTLE_CLIPPING_FACTOR / 2));
        clippedThrottleMax = MAX(throttleMax - (rpyMixRange / 2), throttleMin + (throttleRange / 2) + (throttleRange * THROTTLE_CLIPPING_FACTOR / 2));
    }

    // Now add in the desired throttle, but keep in a range that doesn't clip adjusted
    // roll/pitch/yaw. This could move throttle down, but also up for those low throttle flips.
    if (ARMING_FLAG(ARMED)) {
        for (int i = 0; i < motorCount; i++) {
            motor[i] = rpyMix[i] + constrain(throttleCommand * currentMixer[i].throttle, clippedThrottleMin, clippedThrottleMax);

            int16_t low = throttleMin;
            if (failsafeIsActive()) {
                low = motorConfig()->mincommand;
            }
            motor[i] = constrain(motor[i], low, throttleMax);

            // Motor stop handling
            if (ARMING_FLAG(ARMED) && (getMotorStatus() != MOTOR_RUNNING)) {
                throttlePreviousOutput = 0;
                motor[i] = feature(FEATURE_MOTOR_STOP) ? mixerMotorStoppedPWMValue() : mixerMotorMinimumSpinPWMValue();
            }
        }
    } else {
        throttlePreviousOutput = 0;
        for (int i = 0; i < motorCount; i++) {
            motor[i] = motor_disarmed[i];
        }
    }

    /* Apply motor acceleration/deceleration limit */
    applyMotorRateLimiting(dT);
}

motorStatus_e getMotorStatus(void)
{
    if (failsafeRequiresMotorStop() || (!failsafeIsActive() && STATE(NAV_MOTOR_STOP_OR_IDLE))) {
        return MOTOR_STOPPED_AUTO;
    }

    if (rcControlGetInput()->throttle == 0) {
        if ((STATE(FIXED_WING) || !STATE(AIRMODE_ACTIVE)) && (!(navigationIsFlyingAutonomousMode() && navConfig()->general.flags.auto_overrides_motor_stop)) && (!failsafeIsActive())) {
            return MOTOR_STOPPED_USER;
        }
    }

    return MOTOR_RUNNING;
}

void loadPrimaryMotorMixer(void) {
    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        currentMixer[i] = *primaryMotorMixer(i);
    }
}
