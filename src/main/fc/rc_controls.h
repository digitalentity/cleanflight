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

#define AIRMODE_THROTTLE_THRESHOLD 1300

typedef enum rc_alias {
    ROLL = 0,
    PITCH,
    YAW,
    THROTTLE,
    AUX1,
    AUX2,
    AUX3,
    AUX4,
    AUX5,
    AUX6,
    AUX7,
    AUX8,
    AUX9,
    AUX10,
    AUX11,
    AUX12
} rc_alias_e;

typedef enum {
    THROTTLE_LOW = 0,
    THROTTLE_HIGH
} throttleStatus_e;

typedef enum {
    THROTTLE_STATUS_TYPE_RC = 0,
    THROTTLE_STATUS_TYPE_COMMAND
} throttleStatusType_e;

typedef enum {
    NOT_CENTERED = 0,
    CENTERED
} rollPitchStatus_e;

typedef enum {
    STICK_CENTER = 0,
    THROTTLE_THRESHOLD,
    STICK_CENTER_ONCE
} airmodeHandlingType_e;

typedef enum {
    ROL_LO = (1 << (2 * ROLL)),
    ROL_CE = (3 << (2 * ROLL)),
    ROL_HI = (2 << (2 * ROLL)),

    PIT_LO = (1 << (2 * PITCH)),
    PIT_CE = (3 << (2 * PITCH)),
    PIT_HI = (2 << (2 * PITCH)),

    YAW_LO = (1 << (2 * YAW)),
    YAW_CE = (3 << (2 * YAW)),
    YAW_HI = (2 << (2 * YAW)),

    THR_LO = (1 << (2 * THROTTLE)),
    THR_CE = (3 << (2 * THROTTLE)),
    THR_HI = (2 << (2 * THROTTLE))
} stickPositions_e;

extern int16_t rcCommand[4];

typedef struct rcControlsConfig_s {
    uint8_t deadband;                       // introduce a deadband around the stick center for pitch and roll axis. Must be greater than zero.
    uint8_t yaw_deadband;                   // introduce a deadband around the stick center for yaw axis. Must be greater than zero.
    uint8_t pos_hold_deadband;              // Adds ability to adjust the Hold-position when moving the sticks (assisted mode)
    uint8_t alt_hold_deadband;              // Defines the neutral zone of throttle stick during altitude hold
    uint16_t mid_throttle_deadband;           // default throttle deadband from MIDRC
    uint8_t airmodeHandlingType;            // Defaults to ANTI_WINDUP triggered at sticks centered
    uint16_t airmodeThrottleThreshold;      // Throttle threshold for airmode initial activation
} rcControlsConfig_t;

PG_DECLARE(rcControlsConfig_t, rcControlsConfig);

typedef struct armingConfig_s {
    uint8_t fixed_wing_auto_arm;            // Auto-arm fixed wing aircraft on throttle up and never disarm
    uint8_t disarm_kill_switch;             // allow disarm via AUX switch regardless of throttle value
    uint16_t switchDisarmDelayMs;           // additional delay between ARM box going off and actual disarm
    uint16_t prearmTimeoutMs;               // duration for which Prearm being activated is valid. after this, Prearm needs to be reset. 0 means Prearm does not timeout.
} armingConfig_t;

PG_DECLARE(armingConfig_t, armingConfig);

stickPositions_e getRcStickPositions(void);
bool checkStickPosition(stickPositions_e stickPos);

bool areSticksInApModePosition(uint16_t ap_mode);
bool areSticksDeflectedMoreThanPosHoldDeadband(void);
throttleStatus_e calculateThrottleStatus(throttleStatusType_e type);
rollPitchStatus_e calculateRollPitchCenterStatus(void);
void processRcStickPositions(throttleStatus_e throttleStatus);

int32_t getRcStickDeflection(int32_t axis);
