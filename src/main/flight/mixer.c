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
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/servos.h"

#include "navigation/navigation.h"

#include "rx/rx.h"

#include "sensors/battery.h"


//#define MIXER_DEBUG

static uint8_t motorCount;

int16_t motor[MAX_SUPPORTED_MOTORS];
int16_t motor_disarmed[MAX_SUPPORTED_MOTORS];
static float motorMixRange;

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
    .appliedMixerPreset = -1 //This flag is not available in CLI and used by Configurator only
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

PG_REGISTER_WITH_RESET_TEMPLATE(motorConfig_t, motorConfig, PG_MOTOR_CONFIG, 2);

PG_RESET_TEMPLATE(motorConfig_t, motorConfig,
    .minthrottle = DEFAULT_MIN_THROTTLE,
    .motorPwmProtocol = DEFAULT_PWM_PROTOCOL,
    .motorPwmRate = DEFAULT_PWM_RATE,
    .maxthrottle = 1850,
    .mincommand = 1000,
    .motorAccelTimeMs = 0,
    .motorDecelTimeMs = 0,
    .throttleVBatCompensation = false
);

static motorMixer_t currentMixer[MAX_SUPPORTED_MOTORS];

PG_REGISTER_ARRAY(motorMixer_t, MAX_SUPPORTED_MOTORS, customMotorMixer, PG_MOTOR_MIXER, 0);

uint8_t getMotorCount(void)
{
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

void mixerUpdateStateFlags(void)
{
    // set flag that we're on something with wings
    if (mixerConfig()->platformType == PLATFORM_AIRPLANE) {
        ENABLE_STATE(FIXED_WING);
        DISABLE_STATE(HELICOPTER);
    } else if (mixerConfig()->platformType == PLATFORM_HELICOPTER) {
        DISABLE_STATE(FIXED_WING);
        ENABLE_STATE(HELICOPTER);
    } else {
        DISABLE_STATE(FIXED_WING);
        DISABLE_STATE(HELICOPTER);
    }

    if (mixerConfig()->hasFlaps) {
        ENABLE_STATE(FLAPERON_AVAILABLE);
    } else {
        DISABLE_STATE(FLAPERON_AVAILABLE);
    }
}

void mixerUsePWMIOConfiguration(void)
{
    motorCount = 0;

    // load custom mixer into currentMixer
    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        // check if done
        if (customMotorMixer(i)->throttle == 0.0f)
            break;
        currentMixer[i] = *customMotorMixer(i);
        motorCount++;
    }
    
    // in 3D mode, mixer gain has to be halved
    if (feature(FEATURE_3D)) {
        if (motorCount > 1) {
            for (int i = 0; i < motorCount; i++) {
                currentMixer[i].pitch *= 0.5f;
                currentMixer[i].roll *= 0.5f;
                currentMixer[i].yaw *= 0.5f;
            }
        }
    }

    mixerResetDisarmedMotors();
}

void mixerResetDisarmedMotors(void)
{
    // set disarmed motor values
    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        motor_disarmed[i] = feature(FEATURE_3D) ? flight3DConfig()->neutral3d : motorConfig()->mincommand;
    }
}

void writeMotors(void)
{
    for (int i = 0; i < motorCount; i++) {
        pwmWriteMotor(i, motor[i]);
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
    writeAllMotors(feature(FEATURE_3D) ? flight3DConfig()->neutral3d : motorConfig()->mincommand);

    delay(50); // give the timers and ESCs a chance to react.
}

void stopPwmAllMotors(void)
{
    pwmShutdownPulsesForAllMotors(motorCount);
}

static void applyMotorRateLimiting(const float dT)
{
    static float motorPrevious[MAX_SUPPORTED_MOTORS] = { 0 };

    if (feature(FEATURE_3D)) {
        // FIXME: Don't apply rate limiting in 3D mode
        for (int i = 0; i < motorCount; i++) {
            motorPrevious[i] = motor[i];
        }
    }
    else {
        // Calculate max motor step
        const uint16_t motorRange = motorConfig()->maxthrottle - motorConfig()->minthrottle;
        const float motorMaxInc = (motorConfig()->motorAccelTimeMs == 0) ? 2000 : motorRange * dT / (motorConfig()->motorAccelTimeMs * 1e-3f);
        const float motorMaxDec = (motorConfig()->motorDecelTimeMs == 0) ? 2000 : motorRange * dT / (motorConfig()->motorDecelTimeMs * 1e-3f);

        for (int i = 0; i < motorCount; i++) {
            // Apply motor rate limiting
            motorPrevious[i] = constrainf(motor[i], motorPrevious[i] - motorMaxDec, motorPrevious[i] + motorMaxInc);

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

typedef enum {
    THROTTLE_3D_POSITIVE,
    THROTTLE_3D_NEGATIVE
} throttle3DStatus_t;

void mixTable(const float dT)
{
    float mixInput[3];   // RPY, range [-1;1]

    // Allow direct stick input to motors in passthrough mode on airplanes
    if (STATE(FIXED_WING) && FLIGHT_MODE(MANUAL_MODE)) {
        // Direct passthru from RX - use stick to prevent ANY code from messing up with passthrough mode
        mixInput[ROLL] = rcCmd.stick[ROLL];
        mixInput[PITCH] = rcCmd.stick[PITCH];
        mixInput[YAW] = rcCmd.stick[YAW];
    }
    else {
        mixInput[ROLL] = axisPID[ROLL] / PID_MIXER_SCALING;
        mixInput[PITCH] = axisPID[PITCH] / PID_MIXER_SCALING;
        mixInput[YAW] = axisPID[YAW] / PID_MIXER_SCALING;

        if (motorCount >= 4 && mixerConfig()->yaw_jump_prevention_limit < YAW_JUMP_PREVENTION_LIMIT_HIGH) {
            // prevent "yaw jump" during yaw correction
            const float yawLimit = mixerConfig()->yaw_jump_prevention_limit / PID_MIXER_SCALING + ABS(rcCmd.command[YAW]);
            mixInput[YAW] = constrainf(mixInput[YAW], -yawLimit, yawLimit);
        }
    }

    // Initial mixer concept by bdoiron74 reused and optimized for Air Mode
    float rpyMix[MAX_SUPPORTED_MOTORS];
    float rpyMixMax = 0; // assumption: symetrical about zero.
    float rpyMixMin = 0;

    // motors for non-servo mixes
    for (int i = 0; i < motorCount; i++) {
        rpyMix[i] =
            mixInput[PITCH] * currentMixer[i].pitch +
            mixInput[ROLL] * currentMixer[i].roll +
            -mixerConfig()->yaw_motor_direction * mixInput[YAW] * currentMixer[i].yaw;

        if (rpyMix[i] > rpyMixMax) rpyMixMax = rpyMix[i];
        if (rpyMix[i] < rpyMixMin) rpyMixMin = rpyMix[i];
    }

    float rpyMixRange = rpyMixMax - rpyMixMin;
    float throttleRange, throttleCommand;
    float throttleMin;
    float throttleMax;
    static float throttlePrevious = 0;   // Store the last throttle direction for deadband transitions
    throttle3DStatus_t throttle3DStatus;

    // Find min and max throttle based on condition.
    if (feature(FEATURE_3D)) {
        if (!ARMING_FLAG(ARMED)) {
            throttlePrevious = 0;
        }

        // In 3D mode we have to mind the throttle change direction and switch between positive and negative ranges
        const float throttleDeadbandValue = rcControlsConfig()->deadband3d_throttle / 500.0f;   // Translate from RC raw to fractions
        if (rcCmd.command[THROTTLE] <= -throttleDeadbandValue) { // Out of band handling
            throttle3DStatus = THROTTLE_3D_NEGATIVE;
            throttleMin = -1.0f;
            throttleMax = 0.0f;
            throttlePrevious = rcCmd.command[THROTTLE];
            throttleCommand = rcCmd.command[THROTTLE];
        } else if (rcCmd.command[THROTTLE] >= throttleDeadbandValue) { // Positive handling
            throttle3DStatus = THROTTLE_3D_POSITIVE;
            throttleMin = 0.0f;
            throttleMax = 1.0f;
            throttlePrevious = rcCmd.command[THROTTLE];
            throttleCommand = rcCmd.command[THROTTLE];
        } else if (throttlePrevious <= -throttleDeadbandValue)  { // Deadband handling from negative to positive
            throttle3DStatus = THROTTLE_3D_NEGATIVE;
            throttleMin = -1.0f;
            throttleMax = 0.0f;
            throttleCommand = 0.0f;
        } else {  // Deadband handling from positive to negative
            throttle3DStatus = THROTTLE_3D_POSITIVE;
            throttleMin = 0.0f;
            throttleMax = 1.0f;
            throttleCommand = 0.0f;
        }
    } else {
        // In non-3D mode we only use positive throttle range
        throttleCommand = rcCmd.command[THROTTLE];
        throttleMin = 0.0f;
        throttleMax = 1.0f;

        // Throttle compensation based on battery voltage
        if (motorConfig()->throttleVBatCompensation && STATE(FIXED_WING) && isAmperageConfigured() && feature(FEATURE_VBAT)) {
            throttleCommand = MIN(throttleCommand * calculateThrottleCompensationFactor(), throttleMax);
        }
    }

    throttleRange = throttleMax - throttleMin;

    #define THROTTLE_CLIPPING_FACTOR    0.33f
    motorMixRange = (float)rpyMixRange / (float)throttleRange;
    if (motorMixRange > 1.0f) {
        for (int i = 0; i < motorCount; i++) {
            rpyMix[i] /= motorMixRange;
        }

        // Allow some clipping on edges to soften correction response
        throttleMin = throttleMin + (throttleRange / 2) - (throttleRange * THROTTLE_CLIPPING_FACTOR / 2);
        throttleMax = throttleMin + (throttleRange / 2) + (throttleRange * THROTTLE_CLIPPING_FACTOR / 2);
    } else {
        throttleMin = MIN(throttleMin + (rpyMixRange / 2), throttleMin + (throttleRange / 2) - (throttleRange * THROTTLE_CLIPPING_FACTOR / 2));
        throttleMax = MAX(throttleMax - (rpyMixRange / 2), throttleMin + (throttleRange / 2) + (throttleRange * THROTTLE_CLIPPING_FACTOR / 2));
    }

    // Now add in the desired throttle, but keep in a range that doesn't clip adjusted
    // roll/pitch/yaw. This could move throttle down, but also up for those low throttle flips.
    if (ARMING_FLAG(ARMED)) {
        for (int i = 0; i < motorCount; i++) {
            float motorOutput = rpyMix[i] + constrainf(throttleCommand * currentMixer[i].throttle, throttleMin, throttleMax);

            // Map from virtual [-1;1] values to PWM command to be sent to motors
            if (failsafeIsActive()) {
                motorOutput = constrainf(motorOutput, 0.0f, 1.0f);
                motor[i] = scaleRangef(motorOutput, 0.0, 1.0f, motorConfig()->minthrottle, motorConfig()->maxthrottle);
            } else if (feature(FEATURE_3D)) {
                if (throttle3DStatus == THROTTLE_3D_POSITIVE) {
                    motorOutput = constrainf(motorOutput, 0.0f, 1.0f);
                    motor[i] = scaleRangef(motorOutput, 0.0, 1.0f, flight3DConfig()->deadband3d_high, motorConfig()->maxthrottle);
                }
                else {
                    motorOutput = constrainf(motorOutput, -1.0f, 0.0f);
                    motor[i] = scaleRangef(motorOutput, -1.0, 0.0f, motorConfig()->minthrottle, flight3DConfig()->deadband3d_low);
                }
            }
            else {
                motorOutput = constrainf(motorOutput, 0.0f, 1.0f);
                motor[i] = scaleRangef(motorOutput, 0.0, 1.0f, motorConfig()->minthrottle, motorConfig()->maxthrottle);
            }

            // Motor stop handling
            if (feature(FEATURE_MOTOR_STOP) && ARMING_FLAG(ARMED)) {
                bool failsafeMotorStop = failsafeRequiresMotorStop();
                bool navMotorStop = !failsafeIsActive() && STATE(NAV_MOTOR_STOP_OR_IDLE);
                bool userMotorStop = !navigationIsFlyingAutonomousMode() && !failsafeIsActive() && (rcData[THROTTLE] < rxConfig()->mincheck);
                if (failsafeMotorStop || navMotorStop || userMotorStop) {
                    if (feature(FEATURE_3D)) {
                        motor[i] = PWM_RANGE_MIDDLE;
                    }
                    else {
                        motor[i] = motorConfig()->mincommand;
                    }
                }
            }
        }
    } else {
        for (int i = 0; i < motorCount; i++) {
            motor[i] = motor_disarmed[i];
        }
    }

    /* Apply motor acceleration/deceleration limit */
    applyMotorRateLimiting(dT);
}
