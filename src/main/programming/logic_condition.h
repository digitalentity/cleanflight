/*
 * This file is part of INAV Project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Alternatively, the contents of this file may be used under the terms
 * of the GNU General Public License Version 3, as described below:
 *
 * This file is free software: you may copy, redistribute and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/.
 */
#pragma once

#include "config/parameter_group.h"
#include "common/time.h"

#define MAX_LOGIC_CONDITIONS 32

typedef enum {
    LOGIC_CONDITION_TRUE                        = 0,
    LOGIC_CONDITION_EQUAL                       = 1,
    LOGIC_CONDITION_GREATER_THAN                = 2,
    LOGIC_CONDITION_LOWER_THAN                  = 3,
    LOGIC_CONDITION_LOW                         = 4,
    LOGIC_CONDITION_MID                         = 5,
    LOGIC_CONDITION_HIGH                        = 6,
    LOGIC_CONDITION_AND                         = 7,
    LOGIC_CONDITION_OR                          = 8,
    LOGIC_CONDITION_XOR                         = 9,
    LOGIC_CONDITION_NAND                        = 10,
    LOGIC_CONDITION_NOR                         = 11,
    LOGIC_CONDITION_NOT                         = 12,
    LOGIC_CONDITION_STICKY                      = 13,
    LOGIC_CONDITION_ADD                         = 14,
    LOGIC_CONDITION_SUB                         = 15,
    LOGIC_CONDITION_MUL                         = 16,
    LOGIC_CONDITION_DIV                         = 17,
    LOGIC_CONDITION_GVAR_SET                    = 18,
    LOGIC_CONDITION_GVAR_INC                    = 19,
    LOGIC_CONDITION_GVAR_DEC                    = 20,
    LOGIC_CONDITION_PORT_SET                    = 21,
    LOGIC_CONDITION_OVERRIDE_ARMING_SAFETY      = 22,
    LOGIC_CONDITION_OVERRIDE_THROTTLE_SCALE     = 23,
    LOGIC_CONDITION_SWAP_ROLL_YAW               = 24,
    LOGIC_CONDITION_SET_VTX_POWER_LEVEL         = 25,
    LOGIC_CONDITION_INVERT_ROLL                 = 26,
    LOGIC_CONDITION_INVERT_PITCH                = 27,
    LOGIC_CONDITION_INVERT_YAW                  = 28,
    LOGIC_CONDITION_OVERRIDE_THROTTLE           = 29,
    LOGIC_CONDITION_SET_VTX_BAND                = 30,
    LOGIC_CONDITION_SET_VTX_CHANNEL             = 31,
    LOGIC_CONDITION_SET_OSD_LAYOUT              = 32,
    LOGIC_CONDITION_SIN                         = 33,
    LOGIC_CONDITION_COS                         = 34,
    LOGIC_CONDITION_TAN                         = 35,
    LOGIC_CONDITION_MAP_INPUT                   = 36,
    LOGIC_CONDITION_MAP_OUTPUT                  = 37,
    LOGIC_CONDITION_RC_CHANNEL_OVERRIDE         = 38,
    LOGIC_CONDITION_SET_HEADING_TARGET          = 39,
    LOGIC_CONDITION_MODULUS                     = 40,
    LOGIC_CONDITION_LAST                        = 41,
} logicOperation_e;

typedef enum logicOperandType_s {
    LOGIC_CONDITION_OPERAND_TYPE_VALUE = 0,
    LOGIC_CONDITION_OPERAND_TYPE_RC_CHANNEL,
    LOGIC_CONDITION_OPERAND_TYPE_FLIGHT,
    LOGIC_CONDITION_OPERAND_TYPE_FLIGHT_MODE,
    LOGIC_CONDITION_OPERAND_TYPE_LC,    // Result of different LC and LC operand
    LOGIC_CONDITION_OPERAND_TYPE_GVAR,  // Value from a global variable
    LOGIC_CONDITION_OPERAND_TYPE_PID,  // Value from a Programming PID
    LOGIC_CONDITION_OPERAND_TYPE_LAST
} logicOperandType_e;

typedef enum {
    LOGIC_CONDITION_OPERAND_FLIGHT_ARM_TIMER = 0, // in s                   // 0
    LOGIC_CONDITION_OPERAND_FLIGHT_HOME_DISTANCE, //in m                    // 1
    LOGIC_CONDITION_OPERAND_FLIGHT_TRIP_DISTANCE, //in m                    // 2
    LOGIC_CONDITION_OPERAND_FLIGHT_RSSI,                                    // 3
    LOGIC_CONDITION_OPERAND_FLIGHT_VBAT, // Volt / 10                       // 4
    LOGIC_CONDITION_OPERAND_FLIGHT_CELL_VOLTAGE, // Volt / 10               // 5
    LOGIC_CONDITION_OPERAND_FLIGHT_CURRENT, // Amp / 100                    // 6
    LOGIC_CONDITION_OPERAND_FLIGHT_MAH_DRAWN, // mAh                        // 7
    LOGIC_CONDITION_OPERAND_FLIGHT_GPS_SATS,                                // 8
    LOGIC_CONDITION_OPERAND_FLIGHT_GROUD_SPEED, // cm/s                     // 9
    LOGIC_CONDITION_OPERAND_FLIGHT_3D_SPEED, // cm/s                        // 10
    LOGIC_CONDITION_OPERAND_FLIGHT_AIR_SPEED, // cm/s                       // 11
    LOGIC_CONDITION_OPERAND_FLIGHT_ALTITUDE, // cm                          // 12
    LOGIC_CONDITION_OPERAND_FLIGHT_VERTICAL_SPEED, // cm/s                  // 13
    LOGIC_CONDITION_OPERAND_FLIGHT_TROTTLE_POS, // %                        // 14
    LOGIC_CONDITION_OPERAND_FLIGHT_ATTITUDE_ROLL, // deg                    // 15
    LOGIC_CONDITION_OPERAND_FLIGHT_ATTITUDE_PITCH, // deg                   // 16
    LOGIC_CONDITION_OPERAND_FLIGHT_IS_ARMED, // 0/1                         // 17
    LOGIC_CONDITION_OPERAND_FLIGHT_IS_AUTOLAUNCH, // 0/1                    // 18
    LOGIC_CONDITION_OPERAND_FLIGHT_IS_ALTITUDE_CONTROL, // 0/1              // 19
    LOGIC_CONDITION_OPERAND_FLIGHT_IS_POSITION_CONTROL, // 0/1              // 20
    LOGIC_CONDITION_OPERAND_FLIGHT_IS_EMERGENCY_LANDING, // 0/1             // 21
    LOGIC_CONDITION_OPERAND_FLIGHT_IS_RTH, // 0/1                           // 22
    LOGIC_CONDITION_OPERAND_FLIGHT_IS_WP, // 0/1                            // 23
    LOGIC_CONDITION_OPERAND_FLIGHT_IS_LANDING, // 0/1                       // 24
    LOGIC_CONDITION_OPERAND_FLIGHT_IS_FAILSAFE, // 0/1                      // 25
    LOGIC_CONDITION_OPERAND_FLIGHT_STABILIZED_ROLL,                         // 26
    LOGIC_CONDITION_OPERAND_FLIGHT_STABILIZED_PITCH,                        // 27
    LOGIC_CONDITION_OPERAND_FLIGHT_STABILIZED_YAW,                          // 28
    LOGIC_CONDITION_OPERAND_FLIGHT_WAYPOINT_INDEX,                          // 29
    LOGIC_CONDITION_OPERAND_FLIGHT_WAYPOINT_ACTION,                         // 30
    LOGIC_CONDITION_OPERAND_FLIGHT_3D_HOME_DISTANCE,                        // 31
    LOGIC_CONDITION_OPERAND_FLIGHT_CRSF_LQ,                                 // 32
    LOGIC_CONDITION_OPERAND_FLIGHT_CRSF_SNR,                                // 33
    LOGIC_CONDITION_OPERAND_FLIGHT_GPS_VALID, // 0/1                        // 34

} logicFlightOperands_e;

typedef enum {
    LOGIC_CONDITION_OPERAND_FLIGHT_MODE_FAILSAFE,                           // 0
    LOGIC_CONDITION_OPERAND_FLIGHT_MODE_MANUAL,                             // 1
    LOGIC_CONDITION_OPERAND_FLIGHT_MODE_RTH,                                // 2
    LOGIC_CONDITION_OPERAND_FLIGHT_MODE_POSHOLD,                            // 3
    LOGIC_CONDITION_OPERAND_FLIGHT_MODE_CRUISE,                             // 4
    LOGIC_CONDITION_OPERAND_FLIGHT_MODE_ALTHOLD,                            // 5
    LOGIC_CONDITION_OPERAND_FLIGHT_MODE_ANGLE,                              // 6
    LOGIC_CONDITION_OPERAND_FLIGHT_MODE_HORIZON,                            // 7
    LOGIC_CONDITION_OPERAND_FLIGHT_MODE_AIR,                                // 8
    LOGIC_CONDITION_OPERAND_FLIGHT_MODE_USER1,                              // 9
    LOGIC_CONDITION_OPERAND_FLIGHT_MODE_USER2,                              // 10
    LOGIC_CONDITION_OPERAND_FLIGHT_MODE_COURSE_HOLD,                        // 11
} logicFlightModeOperands_e;

typedef enum {
    LOGIC_CONDITION_GLOBAL_FLAG_OVERRIDE_ARMING_SAFETY = (1 << 0),
    LOGIC_CONDITION_GLOBAL_FLAG_OVERRIDE_THROTTLE_SCALE = (1 << 1),
    LOGIC_CONDITION_GLOBAL_FLAG_OVERRIDE_SWAP_ROLL_YAW = (1 << 2),
    LOGIC_CONDITION_GLOBAL_FLAG_OVERRIDE_INVERT_ROLL = (1 << 3),
    LOGIC_CONDITION_GLOBAL_FLAG_OVERRIDE_INVERT_PITCH = (1 << 4),
    LOGIC_CONDITION_GLOBAL_FLAG_OVERRIDE_INVERT_YAW = (1 << 5),
    LOGIC_CONDITION_GLOBAL_FLAG_OVERRIDE_THROTTLE = (1 << 6),
    LOGIC_CONDITION_GLOBAL_FLAG_OVERRIDE_OSD_LAYOUT = (1 << 7),
    LOGIC_CONDITION_GLOBAL_FLAG_OVERRIDE_RC_CHANNEL = (1 << 8),
} logicConditionsGlobalFlags_t;

typedef enum {
    LOGIC_CONDITION_FLAG_LATCH      = 1 << 0,
} logicConditionFlags_e;

typedef struct logicOperand_s {
    logicOperandType_e type;
    int32_t value;
} logicOperand_t;

typedef struct logicCondition_s {
    uint8_t enabled;
    int8_t activatorId;
    logicOperation_e operation;
    logicOperand_t operandA;
    logicOperand_t operandB;
    uint8_t flags;
} logicCondition_t;

PG_DECLARE_ARRAY(logicCondition_t, MAX_LOGIC_CONDITIONS, logicConditions);

typedef struct logicConditionState_s {
    int value;
    uint8_t flags;
} logicConditionState_t;

typedef struct rcChannelOverride_s {
    uint8_t active;
    int value;
} rcChannelOverride_t;

extern int logicConditionValuesByType[LOGIC_CONDITION_LAST];
extern uint64_t logicConditionsGlobalFlags;

#define LOGIC_CONDITION_GLOBAL_FLAG_DISABLE(mask) (logicConditionsGlobalFlags &= ~(mask))
#define LOGIC_CONDITION_GLOBAL_FLAG_ENABLE(mask) (logicConditionsGlobalFlags |= (mask))
#define LOGIC_CONDITION_GLOBAL_FLAG(mask) (logicConditionsGlobalFlags & (mask))

void logicConditionProcess(uint8_t i);

int logicConditionGetOperandValue(logicOperandType_e type, int operand);

int logicConditionGetValue(int8_t conditionId);
void logicConditionUpdateTask(timeUs_t currentTimeUs);
void logicConditionReset(void);

float getThrottleScale(float globalThrottleScale);
int16_t getRcCommandOverride(int16_t command[], uint8_t axis);
int16_t getRcChannelOverride(uint8_t channel, int16_t originalValue);
