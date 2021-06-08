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


#include <stdint.h>
#include "common/time.h"
#include "fc/runtime_config.h"

typedef struct adsbVehicle_s{
    uint32_t icao;
    int32_t  alt;
    uint16_t cog;
    uint16_t dir;
    uint32_t dist;     
    uint8_t  ttl; 
} adsbVehicle_t;

extern void adsbNewVehicle(uint32_t avicao, int32_t avlat, int32_t avlon, int32_t avalt);
extern void adsbExpiry(void);
extern adsbVehicle_t adsbNearest;










