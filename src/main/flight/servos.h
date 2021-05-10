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
#include "programming/logic_condition.h"

#define MAX_SUPPORTED_SERVOS 16

// These must be consecutive
typedef enum {
    INPUT_STABILIZED_ROLL           = 0,
    INPUT_STABILIZED_PITCH          = 1,
    INPUT_STABILIZED_YAW            = 2,
    INPUT_STABILIZED_THROTTLE       = 3,
    INPUT_RC_ROLL                   = 4,
    INPUT_RC_PITCH                  = 5,
    INPUT_RC_YAW                    = 6,
    INPUT_RC_THROTTLE               = 7,
    INPUT_RC_CH5                    = 8,
    INPUT_RC_CH6                    = 9,
    INPUT_RC_CH7                    = 10,
    INPUT_RC_CH8                    = 11,
    INPUT_GIMBAL_PITCH              = 12,
    INPUT_GIMBAL_ROLL               = 13,
    INPUT_FEATURE_FLAPS             = 14,
    INPUT_RC_CH9                    = 15,
    INPUT_RC_CH10                   = 16,
    INPUT_RC_CH11                   = 17,
    INPUT_RC_CH12                   = 18,
    INPUT_RC_CH13                   = 19,
    INPUT_RC_CH14                   = 20,
    INPUT_RC_CH15                   = 21,
    INPUT_RC_CH16                   = 22,
    INPUT_STABILIZED_ROLL_PLUS      = 23,
    INPUT_STABILIZED_ROLL_MINUS     = 24,
    INPUT_STABILIZED_PITCH_PLUS     = 25,
    INPUT_STABILIZED_PITCH_MINUS    = 26,
    INPUT_STABILIZED_YAW_PLUS       = 27,
    INPUT_STABILIZED_YAW_MINUS      = 28,
    INPUT_MAX                       = 29,
    INPUT_GVAR_0                    = 30,
    INPUT_GVAR_1                    = 31,
    INPUT_GVAR_2                    = 32,
    INPUT_GVAR_3                    = 33,
    INPUT_GVAR_4                    = 34,
    INPUT_GVAR_5                    = 35,
    INPUT_GVAR_6                    = 36,
    INPUT_GVAR_7                    = 37,

    INPUT_SOURCE_COUNT
} inputSource_e;

// target servo channels
typedef enum {
    SERVO_GIMBAL_PITCH = 0,
    SERVO_GIMBAL_ROLL = 1,
    SERVO_ELEVATOR = 2,
    SERVO_FLAPPERON_1 = 3,
    SERVO_FLAPPERON_2 = 4,
    SERVO_RUDDER = 5,

    SERVO_BICOPTER_LEFT = 4,
    SERVO_BICOPTER_RIGHT = 5,

    SERVO_DUALCOPTER_LEFT = 4,
    SERVO_DUALCOPTER_RIGHT = 5,

    SERVO_SINGLECOPTER_1 = 3,
    SERVO_SINGLECOPTER_2 = 4,
    SERVO_SINGLECOPTER_3 = 5,
    SERVO_SINGLECOPTER_4 = 6,

} servoIndex_e; // FIXME rename to servoChannel_e

#define SERVO_PLANE_INDEX_MIN SERVO_ELEVATOR
#define SERVO_PLANE_INDEX_MAX SERVO_RUDDER

#define SERVO_DUALCOPTER_INDEX_MIN SERVO_DUALCOPTER_LEFT
#define SERVO_DUALCOPTER_INDEX_MAX SERVO_DUALCOPTER_RIGHT

#define SERVO_SINGLECOPTER_INDEX_MIN SERVO_SINGLECOPTER_1
#define SERVO_SINGLECOPTER_INDEX_MAX SERVO_SINGLECOPTER_4

#define SERVO_FLAPPERONS_MIN SERVO_FLAPPERON_1
#define SERVO_FLAPPERONS_MAX SERVO_FLAPPERON_2

#define FLAPERON_THROW_DEFAULT 200
#define FLAPERON_THROW_MIN 50
#define FLAPERON_THROW_MAX 450

typedef struct servoMixer_s {
    uint8_t targetChannel;                  // servo that receives the output of the rule
    uint8_t inputSource;                    // input channel for this rule
    int16_t rate;                           // range [-1000;+1000] ; can be used to adjust a rate 0-1000% and a direction
    uint8_t speed;                          // reduces the speed of the rule, 0=unlimited speed
#ifdef USE_PROGRAMMING_FRAMEWORK
    int8_t conditionId;
#endif
} servoMixer_t;

#define MAX_SERVO_RULES (2 * MAX_SUPPORTED_SERVOS)
#define MAX_SERVO_SPEED UINT8_MAX
#define SERVO_OUTPUT_MAX 2500
#define SERVO_OUTPUT_MIN 500

PG_DECLARE_ARRAY(servoMixer_t, MAX_SERVO_RULES, customServoMixers);

typedef struct servoParam_s {
    int16_t min;                            // servo min
    int16_t max;                            // servo max
    int16_t middle;                         // servo middle
    int8_t rate;                            // range [-125;+125] ; can be used to adjust a rate 0-125% and a direction
} servoParam_t;

PG_DECLARE_ARRAY(servoParam_t, MAX_SUPPORTED_SERVOS, servoParams);

typedef struct servoConfig_s {
    // PWM values, in milliseconds, common range is 1000-2000 (1ms to 2ms)
    uint16_t servoCenterPulse;              // This is the value for servos when they should be in the middle. e.g. 1500.
    uint16_t servoPwmRate;                  // The update rate of servo outputs (50-498Hz)
    int16_t servo_lowpass_freq;             // lowpass servo filter frequency selection; 1/1000ths of loop freq
    uint16_t flaperon_throw_offset;
    uint8_t servo_protocol;                 // See servoProtocolType_e
    uint8_t tri_unarmed_servo;              // send tail servo correction pulses even when unarmed
    uint8_t servo_autotrim_rotation_limit;  // Max rotation for servo midpoints to be updated
    uint8_t servo_autotrim_iterm_threshold; // How much of the Iterm is carried over to the servo midpoints on each update
} servoConfig_t;

PG_DECLARE(servoConfig_t, servoConfig);

typedef struct servoMetadata_s {
    float scaleMax;
    float scaleMin;
} servoMetadata_t;

extern int16_t servo[MAX_SUPPORTED_SERVOS];

bool isServoOutputEnabled(void);
void setServoOutputEnabled(bool flag);
bool isMixerUsingServos(void);
void writeServos(void);
void loadCustomServoMixer(void);
void servoMixer(float dT);
void servoComputeScalingFactors(uint8_t servoIndex);
void servosInit(void);
int getServoCount(void);
