/*
 * This file is part of iNav
 *
 * iNav free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * iNav distributed in the hope that it
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

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

#ifdef USE_RATE_DYNAMICS
    struct {
        uint8_t sensitivityCenter;
        uint8_t sensitivityEnd;
        uint8_t correctionCenter;
        uint8_t correctionEnd;
        uint8_t weightCenter;
        uint8_t weightEnd;
    } rateDynamics;
#endif

} controlRateConfig_t;
