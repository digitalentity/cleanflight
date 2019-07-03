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

#if defined(TARGET_MOTOR_COUNT)
#define MAX_SUPPORTED_MOTORS TARGET_MOTOR_COUNT
#else
#define MAX_SUPPORTED_MOTORS 12
#endif

#define YAW_JUMP_PREVENTION_LIMIT_LOW 80
#define YAW_JUMP_PREVENTION_LIMIT_HIGH 500

#define FW_MIN_THROTTLE_DOWN_PITCH_ANGLE_MAX 450

// Digital protocol has fixed values
#define DSHOT_DISARM_COMMAND      0
#define DSHOT_MIN_THROTTLE       48
#define DSHOT_MAX_THROTTLE     2047
#define DSHOT_3D_DEADBAND_LOW  1047
#define DSHOT_3D_DEADBAND_HIGH 1048

typedef enum {
    PLATFORM_MULTIROTOR     = 0,
    PLATFORM_AIRPLANE       = 1,
    PLATFORM_HELICOPTER     = 2,
    PLATFORM_TRICOPTER      = 3,
    PLATFORM_ROVER          = 4,
    PLATFORM_BOAT           = 5,
    PLATFORM_OTHER          = 6
} flyingPlatformType_e;

typedef struct motorAxisCorrectionLimits_s {
    int16_t min;
    int16_t max;
} motorAxisCorrectionLimits_t;

// Custom mixer data per motor
typedef struct motorMixer_s {
    float throttle;
    float roll;
    float pitch;
    float yaw;
} motorMixer_t;

PG_DECLARE_ARRAY(motorMixer_t, MAX_SUPPORTED_MOTORS, primaryMotorMixer);

typedef struct mixerConfig_s {
    int8_t yaw_motor_direction;
    uint16_t yaw_jump_prevention_limit;      // make limit configurable (original fixed value was 100)
    uint8_t platformType;
    bool hasFlaps;
    int16_t appliedMixerPreset;
    uint16_t fwMinThrottleDownPitchAngle;
} mixerConfig_t;

PG_DECLARE(mixerConfig_t, mixerConfig);

typedef struct flight3DConfig_s {
    uint16_t deadband3d_low;                // min 3d value
    uint16_t deadband3d_high;               // max 3d value
    uint16_t neutral3d;                     // center 3d value
} flight3DConfig_t;

PG_DECLARE(flight3DConfig_t, flight3DConfig);

typedef struct motorConfig_s {
    // PWM values, in milliseconds, common range is 1000-2000 (1ms to 2ms)
    uint16_t minthrottle;                   // Set the minimum throttle command sent to the ESC (Electronic Speed Controller). This is the minimum value that allow motors to run at a idle speed.
    uint16_t maxthrottle;                   // This is the maximum value for the ESCs at full power this value can be increased up to 2000
    uint16_t mincommand;                    // This is the value for the ESCs when they are not armed. In some cases, this value must be lowered down to 900 for some specific ESCs
    uint16_t motorPwmRate;                  // The update rate of motor outputs (50-498Hz)
    uint8_t  motorPwmProtocol;
    uint16_t motorAccelTimeMs;              // Time limit for motor to accelerate from 0 to 100% throttle [ms]
    uint16_t motorDecelTimeMs;              // Time limit for motor to decelerate from 0 to 100% throttle [ms]
    uint16_t digitalIdleOffsetValue;
    uint8_t  motorBidirectionalReverse;     // Use reverse THR values when in bidirectional mode
} motorConfig_t;

PG_DECLARE(motorConfig_t, motorConfig);

typedef enum {
    MOTOR_STOPPED_USER,
    MOTOR_STOPPED_AUTO,
    MOTOR_RUNNING
} motorStatus_e;

extern int16_t motor[MAX_SUPPORTED_MOTORS];
extern int16_t motor_disarmed[MAX_SUPPORTED_MOTORS];

uint8_t getMotorCount(void);
float getMotorMixRange(void);
bool mixerIsOutputSaturated(void);
// Returns wether reversing motor direction is possible
// (e.g. bidirectional ESC or PWM protocol supports momentarily
// reversing motor direction).
bool mixerCanReverseMotors(void);
// Returns wether the mixer is currently driving the motors
// in reverse.
bool mixerIsReversingMotors(void);
motorStatus_e getMotorStatus(void);

void writeAllMotors(int16_t mc);
void mixerInit(void);
void mixerUpdateStateFlags(void);
void mixerResetDisarmedMotors(void);
void mixTable(const float dT);
void writeMotors(void);
void processServoAutotrim(void);
void stopMotors(void);
void stopPwmAllMotors(void);

void loadPrimaryMotorMixer(void);
