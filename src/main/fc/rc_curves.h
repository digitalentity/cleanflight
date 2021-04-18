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

#include "fc/controlrate_profile.h"

void generateRCThrottleCurve(const controlRateConfig_t *controlRateConfig);
void generateLinThrottleCurve(void);

int16_t rcLookup(int16_t stickDeflection, uint8_t expo);
uint16_t rcLookupThrottle(uint16_t throttle);
uint16_t linLookupThrottle(uint16_t throttle);
uint16_t rcLookupThrottleMid(void);
