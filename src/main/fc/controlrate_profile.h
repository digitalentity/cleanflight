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

#include <stdint.h>

#include "config/parameter_group.h"

#define MAX_CONTROL_RATE_PROFILE_COUNT 3

typedef struct controlRateConfig_s {

    struct {
        uint8_t rcMid8;
        uint8_t rcExpo8;
        uint8_t dynPID;
        uint16_t pa_breakpoint;                // Breakpoint where TPA is activated
        uint16_t fixedWingTauMs;               // Time constant of airplane TPA PT1-filter
    } throttle;

    struct {
        uint8_t rcExpo8;
        uint8_t rcYawExpo8;
        uint8_t rates[3];
    } stabilized;

    struct {
        uint8_t rcExpo8;
        uint8_t rcYawExpo8;
        uint8_t rates[3];
    } manual;

    struct {
        uint8_t fpvCamAngleDegrees;             // Camera angle to treat as "forward" base axis in ACRO (Roll and Yaw sticks will command rotation considering this axis)
    } misc;

} controlRateConfig_t;

PG_DECLARE_ARRAY(controlRateConfig_t, MAX_CONTROL_RATE_PROFILE_COUNT, controlRateProfiles);

extern const controlRateConfig_t *currentControlRateProfile;

void setControlRateProfile(uint8_t profileIndex);
void changeControlRateProfile(uint8_t profileIndex);
void activateControlRateConfig(void);
uint8_t getCurrentControlRateProfile(void);
