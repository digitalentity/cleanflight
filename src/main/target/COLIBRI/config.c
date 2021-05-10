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

#include "platform.h"

#include "config/feature.h"
#include "fc/config.h"
#include "flight/mixer.h"
#include "io/serial.h"
#include "rx/rx.h"
#include "sensors/compass.h"

void targetConfiguration(void)
{
    rxConfigMutable()->serialrx_provider = SERIALRX_SBUS;
    rxConfigMutable()->receiverType = RX_TYPE_SERIAL;
    serialConfigMutable()->portConfigs[2].functionMask = FUNCTION_RX_SERIAL;
#ifdef QUANTON
    compassConfigMutable()->mag_align = CW90_DEG;
#endif
}
