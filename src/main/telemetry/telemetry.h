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

/*
 * telemetry.h
 *
 *  Created on: 6 Apr 2014
 *      Author: Hydra
 */

#pragma once

#include "common/time.h"

#include "config/parameter_group.h"

#include "io/serial.h"


typedef enum {
    FRSKY_FORMAT_DMS = 0,
    FRSKY_FORMAT_NMEA
} frskyGpsCoordFormat_e;

typedef enum {
    FRSKY_UNIT_METRICS = 0,
    FRSKY_UNIT_IMPERIALS
} frskyUnit_e;

typedef enum {
    LTM_RATE_NORMAL,
    LTM_RATE_MEDIUM,
    LTM_RATE_SLOW
} ltmUpdateRate_e;

typedef enum {
    SMARTPORT_FUEL_UNIT_PERCENT,
    SMARTPORT_FUEL_UNIT_MAH,
    SMARTPORT_FUEL_UNIT_MWH
} smartportFuelUnit_e;

typedef struct telemetryConfig_s {
    float gpsNoFixLatitude;
    float gpsNoFixLongitude;
    uint8_t telemetry_switch;               // Use aux channel to change serial output & baudrate( MSP / Telemetry ). It disables automatic switching to Telemetry when armed.
    uint8_t telemetry_inverted;             // Flip the default inversion of the protocol - Same as serialrx_inverted in rx.c, but for telemetry.
    frskyGpsCoordFormat_e frsky_coordinate_format;
    frskyUnit_e frsky_unit;
    uint8_t frsky_vfas_precision;
    uint8_t frsky_pitch_roll;
    uint8_t report_cell_voltage;
    uint8_t hottAlarmSoundInterval;
    uint8_t halfDuplex;
    smartportFuelUnit_e smartportFuelUnit;
    uint8_t ibusTelemetryType;
    uint8_t ltmUpdateRate;

#ifdef USE_TELEMETRY_SIM
    int16_t simLowAltitude;
    char simGroundStationNumber[16];
    char simPin[8];
    uint16_t simTransmitInterval;
    uint8_t simTransmitFlags;

    uint16_t accEventThresholdHigh;
    uint16_t accEventThresholdLow;
    uint16_t accEventThresholdNegX;
#endif
    struct {
        uint8_t extended_status_rate;
        uint8_t rc_channels_rate;
        uint8_t position_rate;
        uint8_t extra1_rate;
        uint8_t extra2_rate;
        uint8_t extra3_rate;
        uint8_t version;
    } mavlink;
} telemetryConfig_t;

PG_DECLARE(telemetryConfig_t, telemetryConfig);

#define TELEMETRY_SHAREABLE_PORT_FUNCTIONS_MASK (FUNCTION_TELEMETRY_FRSKY | FUNCTION_TELEMETRY_LTM | FUNCTION_TELEMETRY_IBUS)
extern serialPort_t *telemetrySharedPort;

void telemetryInit(void);
bool telemetryCheckRxPortShared(const serialPortConfig_t *portConfig);

void telemetryCheckState(void);
void telemetryProcess(timeUs_t currentTimeUs);

bool telemetryDetermineEnabledState(portSharing_e portSharing);
