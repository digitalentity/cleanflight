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

#include <common/time.h>

#include "config/parameter_group.h"

#if defined(USE_POWER_LIMITS)

typedef struct {
    uint16_t continuousCurrent;         // dA
    uint16_t burstCurrent;              // dA
    uint16_t burstCurrentTime;          // ds
    uint16_t burstCurrentFalldownTime;  // ds

#ifdef USE_ADC
    uint16_t continuousPower;           // dW
    uint16_t burstPower;                // dW
    uint16_t burstPowerTime;            // ds
    uint16_t burstPowerFalldownTime;    // ds
#endif

    uint16_t piP;
    uint16_t piI;

    float attnFilterCutoff;             // Hz
} powerLimitsConfig_t;

PG_DECLARE(powerLimitsConfig_t, powerLimitsConfig);

void powerLimiterInit(void);
void currentLimiterUpdate(timeDelta_t timeDelta);
void powerLimiterUpdate(timeDelta_t timeDelta);
void powerLimiterApply(int16_t *throttleCommand);
bool powerLimiterIsLimiting(void);
bool powerLimiterIsLimitingCurrent(void);
float powerLimiterGetRemainingBurstTime(void);      // returns seconds
uint16_t powerLimiterGetActiveCurrentLimit(void);   // returns cA
#ifdef USE_ADC
uint16_t powerLimiterGetActivePowerLimit(void);     // returns cW
bool powerLimiterIsLimitingPower(void);
#endif

#endif
