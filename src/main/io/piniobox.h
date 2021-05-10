/*
 * This file is part of Cleanflight, Betaflight and INAV
 *
 * Cleanflight, Betaflight and INAV are free software. You can 
 * redistribute this software and/or modify this software under 
 * the terms of the GNU General Public License as published by 
 * the Free Software Foundation, either version 3 of the License, 
 * or (at your option) any later version.
 *
 * Cleanflight, Betaflight and INAV are distributed in the hope that 
 * they will be useful, but WITHOUT ANY WARRANTY; without even the 
 * implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR 
 * PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "common/time.h"
#include "drivers/pinio.h"


#define BOX_PERMANENT_ID_USER1      47
#define BOX_PERMANENT_ID_USER2      48
#define BOX_PERMANENT_ID_NONE       255       // A permanent ID for no box mode


typedef struct pinioBoxConfig_s {
    uint8_t permanentId[PINIO_COUNT];
} pinioBoxConfig_t;

PG_DECLARE(pinioBoxConfig_t, pinioBoxConfig);

void pinioBoxInit(void);
void pinioBoxUpdate(void);
