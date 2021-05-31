/*
 * This file is part of INAV.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Alternatively, the contents of this file may be used under the terms
 * of the GNU General Public License Version 3, as described below:
 *
 * This file is free software: you may copy, redistribute and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/.
 *
 * @author Konstantin Sharlaimov (ksharlaimov@inavflight.com)
  */

#pragma once

#include "msp/msp.h"
#include "msp/msp_serial.h"

#include "config/parameter_group.h"

#if defined(USE_DJI_HD_OSD)

#define DJI_API_VERSION_MAJOR           1
#define DJI_API_VERSION_MINOR           42

#define DJI_MSP_API_VERSION             1       // INAV: Implemented     | DSI: ???             | 
#define DJI_MSP_FC_VARIANT              2       // INAV: Implemented     | DSI: ???             | 
#define DJI_MSP_FC_VERSION              3       // INAV: Implemented     | DSI: ???             | 
#define DJI_MSP_NAME                    10      // INAV: Implemented     | DSI: Implemented     | For OSD 'Craft Name'
#define DJI_MSP_OSD_CONFIG              84      // INAV: Implemented     | DSI: Implemented     | OSD item count + positions
#define DJI_MSP_FILTER_CONFIG           92      // INAV: Not implemented | DSI: Implemented     |
#define DJI_MSP_PID_ADVANCED            94      // INAV: Not implemented | DSI: Implemented     |
#define DJI_MSP_STATUS                  101     // INAV: Implemented     | DSI: Implemented     | For OSD ‘armingTime’, Flight controller arming status
#define DJI_MSP_RC                      105     // INAV: Implemented     | DSI: Implemented     |
#define DJI_MSP_RAW_GPS                 106     // INAV: Implemented     | DSI: Implemented     | For OSD ‘GPS Sats’ + coordinates
#define DJI_MSP_COMP_GPS                107     // INAV: Implemented     | DSI: Not implemented | GPS direction to home & distance to home
#define DJI_MSP_ATTITUDE                108     // INAV: Implemented     | DSI: Implemented     | For OSD ‘Angle: roll & pitch’
#define DJI_MSP_ALTITUDE                109     // INAV: Implemented     | DSI: Implemented     | For OSD ‘Numerical Vario’
#define DJI_MSP_ANALOG                  110     // INAV: Implemented     | DSI: Implemented     | For OSD ‘RSSI Value’, For OSD ‘Battery voltage’ etc
#define DJI_MSP_RC_TUNING               111     // INAV: Not implemented | DSI: Implemented     |
#define DJI_MSP_PID                     112     // INAV: Implemented     | DSI: Implemented     | For OSD ‘PID roll, yaw, pitch'
#define DJI_MSP_BATTERY_STATE           130     // INAV: Implemented     | DSI: Implemented     | For OSD ‘Battery current mAh drawn’ etc
#define DJI_MSP_ESC_SENSOR_DATA         134     // INAV: Implemented     | DSI: Implemented     | For OSD ‘ESC temperature’
#define DJI_MSP_STATUS_EX               150     // INAV: Implemented     | DSI: Implemented     | For OSD ‘Fly mode', For OSD ‘Disarmed’
#define DJI_MSP_RTC                     247     // INAV: Implemented     | DSI: Implemented     | For OSD ‘RTC date time’

#define DJI_MSP_SET_FILTER_CONFIG       93
#define DJI_MSP_SET_PID_ADVANCED        95
#define DJI_MSP_SET_PID                 202
#define DJI_MSP_SET_RC_TUNING           204

enum djiOsdTempSource_e {
    DJI_OSD_TEMP_ESC    = 0,
    DJI_OSD_TEMP_CORE   = 1,
    DJI_OSD_TEMP_BARO   = 2
};

enum djiOsdProtoWorkarounds_e {
    DJI_OSD_USE_NON_STANDARD_MSP_ESC_SENSOR_DATA    = 1 << 0,
};

typedef struct djiOsdConfig_s {
    uint8_t use_name_for_messages;
    uint8_t esc_temperature_source;
    uint8_t proto_workarounds;
} djiOsdConfig_t;

PG_DECLARE(djiOsdConfig_t, djiOsdConfig);

void djiOsdSerialInit(void);
void djiOsdSerialProcess(void);

#endif
