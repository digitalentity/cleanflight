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

#include <platform.h>
#include "config/config_master.h"
#include "flight/mixer.h"
#include "rx/rx.h"
#include "io/serial.h"
#include "telemetry/telemetry.h"
#include "sensors/compass.h"

void targetConfiguration(void)
{
    compassConfigMutable()->mag_align = CW90_DEG;

    serialConfigMutable()->portConfigs[1].functionMask = FUNCTION_MSP;
    serialConfigMutable()->portConfigs[1].msp_baudrateIndex = BAUD_57600;

    serialConfigMutable()->portConfigs[2].functionMask = FUNCTION_GPS;
    serialConfigMutable()->portConfigs[2].gps_baudrateIndex = BAUD_57600;

    serialConfigMutable()->portConfigs[4].functionMask = FUNCTION_VTX_TRAMP;
    serialConfigMutable()->portConfigs[4].peripheral_baudrateIndex = BAUD_115200;

    serialConfigMutable()->portConfigs[5].functionMask = FUNCTION_RX_SERIAL;
    rxConfigMutable()->receiverType = RX_TYPE_SERIAL;
    rxConfigMutable()->serialrx_provider = SERIALRX_CRSF;

    serialConfigMutable()->portConfigs[6].functionMask = FUNCTION_TELEMETRY_SMARTPORT;

    mixerConfigMutable()->platformType = PLATFORM_AIRPLANE;
}
