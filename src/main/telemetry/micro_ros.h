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

#include "common/time.h"

#define TELEMETRY_MICRO_ROS_PORT_MODE MODE_RXTX

#define TELEMETRY_MICRO_ROS_GCS_MAC_ADDRESS_BUFF_LENGTH 32

typedef enum {
    MICRO_ROS_STATE_CONNECT,
    MICRO_ROS_STATE_RUN,
    MICRO_ROS_STATE_FINIALIZE,
} microRosState;
    

void initMicroRosTelemetry(void);
void handleMicroRosTelemetry(timeUs_t currentTimeUs);
void checkMicroRosTelemetryState(void);
