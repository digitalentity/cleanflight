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

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#ifdef USE_TELEMETRY

#include "common/utils.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/serial.h"

#include "fc/config.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"
#include "fc/settings.h"

#include "io/serial.h"

#include "rx/rx.h"

#include "telemetry/telemetry.h"
#include "telemetry/frsky_d.h"
#include "telemetry/hott.h"
#include "telemetry/smartport.h"
#include "telemetry/ltm.h"
#include "telemetry/mavlink.h"
#include "telemetry/jetiexbus.h"
#include "telemetry/ibus.h"
#include "telemetry/crsf.h"
#include "telemetry/srxl.h"
#include "telemetry/sim.h"
#include "telemetry/ghst.h"


PG_REGISTER_WITH_RESET_TEMPLATE(telemetryConfig_t, telemetryConfig, PG_TELEMETRY_CONFIG, 5);

PG_RESET_TEMPLATE(telemetryConfig_t, telemetryConfig,
    .gpsNoFixLatitude = SETTING_FRSKY_DEFAULT_LATITUDE_DEFAULT,
    .gpsNoFixLongitude = SETTING_FRSKY_DEFAULT_LONGITUDE_DEFAULT,
    .telemetry_switch = SETTING_TELEMETRY_SWITCH_DEFAULT,
    .telemetry_inverted = SETTING_TELEMETRY_INVERTED_DEFAULT,
    .frsky_coordinate_format = SETTING_FRSKY_COORDINATES_FORMAT_DEFAULT,
    .frsky_unit = SETTING_FRSKY_UNIT_DEFAULT,
    .frsky_vfas_precision = SETTING_FRSKY_VFAS_PRECISION_DEFAULT,
    .frsky_pitch_roll = SETTING_FRSKY_PITCH_ROLL_DEFAULT,
    .report_cell_voltage = SETTING_REPORT_CELL_VOLTAGE_DEFAULT,
    .hottAlarmSoundInterval = SETTING_HOTT_ALARM_SOUND_INTERVAL_DEFAULT,
    .halfDuplex = SETTING_TELEMETRY_HALFDUPLEX_DEFAULT,
    .smartportFuelUnit = SETTING_SMARTPORT_FUEL_UNIT_DEFAULT,
    .ibusTelemetryType = SETTING_IBUS_TELEMETRY_TYPE_DEFAULT,
    .ltmUpdateRate = SETTING_LTM_UPDATE_RATE_DEFAULT,

#ifdef USE_TELEMETRY_SIM
    .simTransmitInterval = SETTING_SIM_TRANSMIT_INTERVAL_DEFAULT,
    .simTransmitFlags = SETTING_SIM_TRANSMIT_FLAGS_DEFAULT,
    .simLowAltitude = SETTING_SIM_LOW_ALTITUDE_DEFAULT,
    .simPin = SETTING_SIM_PIN_DEFAULT,
    .simGroundStationNumber = SETTING_SIM_GROUND_STATION_NUMBER_DEFAULT,

    .accEventThresholdHigh = SETTING_ACC_EVENT_THRESHOLD_HIGH_DEFAULT,
    .accEventThresholdLow = SETTING_ACC_EVENT_THRESHOLD_LOW_DEFAULT,
    .accEventThresholdNegX = SETTING_ACC_EVENT_THRESHOLD_NEG_X_DEFAULT,
#endif

    .mavlink = {
        .extended_status_rate = SETTING_MAVLINK_EXT_STATUS_RATE_DEFAULT,
        .rc_channels_rate = SETTING_MAVLINK_RC_CHAN_RATE_DEFAULT,
        .position_rate = SETTING_MAVLINK_POS_RATE_DEFAULT,
        .extra1_rate = SETTING_MAVLINK_EXTRA1_RATE_DEFAULT,
        .extra2_rate = SETTING_MAVLINK_EXTRA2_RATE_DEFAULT,
        .extra3_rate = SETTING_MAVLINK_EXTRA3_RATE_DEFAULT,
        .version = SETTING_MAVLINK_VERSION_DEFAULT
    }
);

void telemetryInit(void)
{
#if defined(USE_TELEMETRY_FRSKY)
    initFrSkyTelemetry();
#endif

#if defined(USE_TELEMETRY_HOTT)
    initHoTTTelemetry();
#endif

#if defined(USE_TELEMETRY_SMARTPORT)
    initSmartPortTelemetry();
#endif

#if defined(USE_TELEMETRY_LTM)
    initLtmTelemetry();
#endif

#if defined(USE_TELEMETRY_MAVLINK)
    initMAVLinkTelemetry();
#endif

#if defined(TELEMETRY_JETIEXBUS)
    initJetiExBusTelemetry();
#endif

#if defined(USE_TELEMETRY_IBUS)
    initIbusTelemetry();
#endif

#if defined(USE_TELEMETRY_SIM)
    initSimTelemetry();
#endif

#if defined(USE_SERIALRX_CRSF) && defined(USE_TELEMETRY_CRSF)
    initCrsfTelemetry();
#endif

#ifdef USE_TELEMETRY_SRXL
    initSrxlTelemetry();
#endif

#ifdef USE_TELEMETRY_GHST
    initGhstTelemetry();
#endif

    telemetryCheckState();
}

bool telemetryDetermineEnabledState(portSharing_e portSharing)
{
    bool enabled = portSharing == PORTSHARING_NOT_SHARED;

    if (portSharing == PORTSHARING_SHARED) {
        if (telemetryConfig()->telemetry_switch)
            enabled = IS_RC_MODE_ACTIVE(BOXTELEMETRY);
        else
            enabled = ARMING_FLAG(ARMED);
    }

    return enabled;
}

bool telemetryCheckRxPortShared(const serialPortConfig_t *portConfig)
{
    return portConfig->functionMask & FUNCTION_RX_SERIAL && portConfig->functionMask & TELEMETRY_SHAREABLE_PORT_FUNCTIONS_MASK;
}

serialPort_t *telemetrySharedPort = NULL;

void telemetryCheckState(void)
{
#if defined(USE_TELEMETRY_FRSKY)
    checkFrSkyTelemetryState();
#endif

#if defined(USE_TELEMETRY_HOTT)
    checkHoTTTelemetryState();
#endif

#if defined(USE_TELEMETRY_SMARTPORT)
    checkSmartPortTelemetryState();
#endif

#if defined(USE_TELEMETRY_LTM)
    checkLtmTelemetryState();
#endif

#if defined(USE_TELEMETRY_MAVLINK)
    checkMAVLinkTelemetryState();
#endif

#if defined(TELEMETRY_JETIEXBUS)
    checkJetiExBusTelemetryState();
#endif

#if defined(USE_TELEMETRY_IBUS)
    checkIbusTelemetryState();
#endif

#if defined(USE_TELEMETRY_SIM)
    checkSimTelemetryState();
#endif

#if defined(USE_SERIALRX_CRSF) && defined(USE_TELEMETRY_CRSF)
    checkCrsfTelemetryState();
#endif

#ifdef USE_TELEMETRY_SRXL
    checkSrxlTelemetryState();
#endif
#ifdef USE_TELEMETRY_GHST
    checkGhstTelemetryState();
#endif
}

void telemetryProcess(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs); // since not used by all the telemetry protocols

#if defined(USE_TELEMETRY_FRSKY)
    handleFrSkyTelemetry();
#endif

#if defined(USE_TELEMETRY_HOTT)
    handleHoTTTelemetry(currentTimeUs);
#endif

#if defined(USE_TELEMETRY_SMARTPORT)
    handleSmartPortTelemetry();
#endif

#if defined(USE_TELEMETRY_LTM)
    handleLtmTelemetry();
#endif

#if defined(USE_TELEMETRY_MAVLINK)
    handleMAVLinkTelemetry(currentTimeUs);
#endif

#if defined(TELEMETRY_JETIEXBUS)
    handleJetiExBusTelemetry();
#endif

#if defined(USE_SERIALRX_IBUS) && defined(USE_TELEMETRY_IBUS)
    handleIbusTelemetry();
#endif

#if defined(USE_TELEMETRY_SIM)
    handleSimTelemetry();
#endif

#if defined(USE_SERIALRX_CRSF) && defined(USE_TELEMETRY_CRSF)
    handleCrsfTelemetry(currentTimeUs);
#endif

#ifdef USE_TELEMETRY_SRXL
    handleSrxlTelemetry(currentTimeUs);
#endif
#ifdef USE_TELEMETRY_GHST
    handleGhstTelemetry(currentTimeUs);
#endif
}

#endif
