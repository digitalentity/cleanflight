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

#define SIM_MIN_TRANSMIT_INTERVAL 10
#define SIM_DEFAULT_TRANSMIT_INTERVAL 60
#define SIM_N_TX_FLAGS 5
#define SIM_DEFAULT_TX_FLAGS "f"
#define SIM_PIN "0000"

typedef enum  {
    SIM_TX_FLAG                 = (1 << 0),
    SIM_TX_FLAG_FAILSAFE        = (1 << 1),
    SIM_TX_FLAG_GPS             = (1 << 2),
    SIM_TX_FLAG_ACC             = (1 << 3),
    SIM_TX_FLAG_LOW_ALT         = (1 << 4),
    SIM_TX_FLAG_RESPONSE        = (1 << 5)
} simTxFlags_e;


void handleSimTelemetry(void);
void initSimTelemetry(void);
void checkSimTelemetryState(void);
