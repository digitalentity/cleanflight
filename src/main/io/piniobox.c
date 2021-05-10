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

#include <stdint.h>
#include <stdbool.h>

#include "platform.h"

#ifdef USE_PINIOBOX

#include "build/debug.h"
#include "common/utils.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "fc/fc_msp.h"
#include "fc/fc_msp_box.h"
#include "fc/settings.h"

#include "io/piniobox.h"


PG_REGISTER_WITH_RESET_TEMPLATE(pinioBoxConfig_t, pinioBoxConfig, PG_PINIOBOX_CONFIG, 1);

PG_RESET_TEMPLATE(pinioBoxConfig_t, pinioBoxConfig,
    { SETTING_PINIO_BOX1_DEFAULT, SETTING_PINIO_BOX2_DEFAULT, SETTING_PINIO_BOX3_DEFAULT, SETTING_PINIO_BOX4_DEFAULT }
);

typedef struct pinioBoxRuntimeConfig_s {
    uint8_t boxId[PINIO_COUNT];
} pinioBoxRuntimeConfig_t;

static pinioBoxRuntimeConfig_t pinioBoxRuntimeConfig;

void pinioBoxInit(void)
{
    // Convert permanentId to boxId_e
    for (int i = 0; i < PINIO_COUNT; i++) {
        const box_t *box = findBoxByPermanentId(pinioBoxConfig()->permanentId[i]);

        pinioBoxRuntimeConfig.boxId[i] = box ? box->boxId : BOXID_NONE;
    }
}

void pinioBoxUpdate(void)
{
    for (int i = 0; i < PINIO_COUNT; i++) {
        if (pinioBoxRuntimeConfig.boxId[i] != BOXID_NONE) {
            pinioSet(i, IS_RC_MODE_ACTIVE(pinioBoxRuntimeConfig.boxId[i]));
        }
    }
}

#endif
