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
#include "telemetry/ibus_shared.h"

#if defined(USE_TELEMETRY) && defined(USE_TELEMETRY_IBUS)

#include "common/maths.h"
#include "common/axis.h"

#include "config/feature.h"

#include "drivers/serial.h"

#include "fc/config.h"
#include "fc/fc_core.h"
#include "fc/rc_control.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "io/serial.h"

#include "sensors/barometer.h"
#include "sensors/temperature.h"
#include "sensors/acceleration.h"
#include "sensors/battery.h"
#include "sensors/sensors.h"
#include "sensors/pitotmeter.h"

#include "scheduler/scheduler.h"

#include "flight/imu.h"
#include "flight/failsafe.h"

#include "io/gps.h"

#include "navigation/navigation.h"

#include "telemetry/ibus.h"
#include "telemetry/telemetry.h"

#define IBUS_TEMPERATURE_OFFSET (0x0190)

typedef uint8_t ibusAddress_t;

typedef enum {
    IBUS_COMMAND_DISCOVER_SENSOR      = 0x80,
    IBUS_COMMAND_SENSOR_TYPE          = 0x90,
    IBUS_COMMAND_MEASUREMENT          = 0xA0
} ibusCommand_e;

typedef struct IBUS_SENSOR {
    uint8_t type;
    uint8_t size;
    uint8_t value;
} IBUS_SENSOR;
static IBUS_SENSOR SENSOR_ADDRESS_TYPE_LOOKUP[] = {
    {.type = IBUS_MEAS_TYPE_INTERNAL_VOLTAGE, .size = 2, .value = IBUS_MEAS_VALUE_NONE },             // Address 0, sensor 1, not usable since it is reserved for internal voltage
    {.type = IBUS_MEAS_TYPE_EXTERNAL_VOLTAGE, .size = 2, .value = IBUS_MEAS_VALUE_EXTERNAL_VOLTAGE }, // Address 1 ,sensor 2, VBAT
    {.type = IBUS_MEAS_TYPE_TEMPERATURE,      .size = 2, .value = IBUS_MEAS_VALUE_TEMPERATURE },      // Address 2, sensor 3, Baro/Gyro Temp
    {.type = IBUS_MEAS_TYPE_RPM,              .size = 2, .value = IBUS_MEAS_VALUE_STATUS },           // Address 3, sensor 4, Status AS RPM
    {.type = IBUS_MEAS_TYPE_RPM,              .size = 2, .value = IBUS_MEAS_VALUE_ACC_Z },            // Address 4, sensor 5, MAG_COURSE in deg AS RPM
    {.type = IBUS_MEAS_TYPE_EXTERNAL_VOLTAGE, .size = 2, .value = IBUS_MEAS_VALUE_CURRENT },          // Address 5, sensor 6, Current in A AS ExtV
    {.type = IBUS_MEAS_TYPE_EXTERNAL_VOLTAGE, .size = 2, .value = IBUS_MEAS_VALUE_ALT },              // Address 6, sensor 7, Baro Alt in cm AS ExtV
    {.type = IBUS_MEAS_TYPE_RPM,              .size = 2, .value = IBUS_MEAS_VALUE_HEADING },          // Address 7, sensor 8, HOME_DIR in deg AS RPM
    {.type = IBUS_MEAS_TYPE_RPM,              .size = 2, .value = IBUS_MEAS_VALUE_DIST },             // Address 8, sensor 9, HOME_DIST in m AS RPM
    {.type = IBUS_MEAS_TYPE_RPM,              .size = 2, .value = IBUS_MEAS_VALUE_COG },              // Address 9, sensor 10,GPS_COURSE in deg AS RPM
    {.type = IBUS_MEAS_TYPE_RPM,              .size = 2, .value = IBUS_MEAS_VALUE_GALT },             // Address 10,sensor 11,GPS_ALT in m AS RPM (ALT m)
    {.type = IBUS_MEAS_TYPE_RPM,              .size = 2, .value = IBUS_MEAS_VALUE_GPS_LAT2 },         // Address 11,sensor 12,GPS_LAT2 AS RPM 5678 (-12.3456789 N)
    {.type = IBUS_MEAS_TYPE_RPM,              .size = 2, .value = IBUS_MEAS_VALUE_GPS_LON2 },         // Address 12,sensor 13,GPS_LON2 AS RPM 6789 (-123.4567890 E)
    {.type = IBUS_MEAS_TYPE_EXTERNAL_VOLTAGE, .size = 2, .value = IBUS_MEAS_VALUE_GPS_LAT1 },         // Address 13,sensor 14,GPS_LAT1 AS ExtV -12.45 (-12.3456789 N)
    {.type = IBUS_MEAS_TYPE_EXTERNAL_VOLTAGE, .size = 2, .value = IBUS_MEAS_VALUE_GPS_LON1 },         // Address 14,sensor 15,GPS_LON1 AS ExtV -123.45 (-123.4567890 E)
    {.type = IBUS_MEAS_TYPE_RPM,              .size = 2, .value = IBUS_MEAS_VALUE_SPE }               // Address 15,sensor 16,GPS_SPEED in km/h AS RPM (SPE km\h)
};

static serialPort_t *ibusSerialPort = NULL;

static uint8_t transmitIbusPacket(uint8_t ibusPacket[static IBUS_MIN_LEN], size_t packetLength) {
    uint16_t checksum = ibusCalculateChecksum(ibusPacket, packetLength);
    ibusPacket[packetLength - IBUS_CHECKSUM_SIZE] = (checksum & 0xFF);
    ibusPacket[packetLength - IBUS_CHECKSUM_SIZE + 1] = (checksum >> 8);
    for (size_t i = 0; i < packetLength; i++) {
        serialWrite(ibusSerialPort, ibusPacket[i]);
    }
    return packetLength;
}

static uint8_t sendIbusCommand(ibusAddress_t address) {
    uint8_t sendBuffer[] = { 0x04, IBUS_COMMAND_DISCOVER_SENSOR | address, 0x00, 0x00 };
    return transmitIbusPacket(sendBuffer, sizeof sendBuffer);
}

static uint8_t sendIbusSensorType(ibusAddress_t address) {
    uint8_t sendBuffer[] = { 0x06, IBUS_COMMAND_SENSOR_TYPE | address, SENSOR_ADDRESS_TYPE_LOOKUP[address].type, SENSOR_ADDRESS_TYPE_LOOKUP[address].size, 0x0, 0x0 };
    return transmitIbusPacket(sendBuffer, sizeof sendBuffer);
}

static uint8_t sendIbusMeasurement2(ibusAddress_t address, uint16_t measurement) {
    uint8_t sendBuffer[] = { 0x06, IBUS_COMMAND_MEASUREMENT | address, measurement & 0xFF, (measurement >> 8) & 0xFF, 0x0, 0x0 };
    return transmitIbusPacket(sendBuffer, sizeof sendBuffer);
}

static uint8_t sendIbusMeasurement4(ibusAddress_t address, int32_t measurement) {
    uint8_t sendBuffer[] = { 0x08, IBUS_COMMAND_MEASUREMENT | address,
        measurement & 0xFF, (measurement >> 8) & 0xFF, (measurement >> 16) & 0xFF, (measurement >> 24) & 0xFF,
        0x0, 0x0 };
    return transmitIbusPacket(sendBuffer, sizeof sendBuffer);
}

static bool isCommand(ibusCommand_e expected, uint8_t ibusPacket[static IBUS_MIN_LEN]) {
    return (ibusPacket[1] & 0xF0) == expected;
}

static ibusAddress_t getAddress(uint8_t ibusPacket[static IBUS_MIN_LEN]) {
    return (ibusPacket[1] & 0x0F);
}

// MANUAL, ACRO, ANGLE, HRZN, ALTHOLD, POSHOLD, RTH, WP, LAUNCH, FAILSAFE
static uint8_t flightModeToIBusTelemetryMode1[FLM_COUNT] = { 0, 1, 3, 2, 5, 6, 7, 4, 8, 9 };
static uint8_t flightModeToIBusTelemetryMode2[FLM_COUNT] = { 5, 1, 0, 7, 2, 8, 6, 3, 4, 9 };
static uint8_t dispatchMeasurementRequest(ibusAddress_t address) {
#if defined(USE_GPS)
    uint8_t fix = 0;
    if (sensors(SENSOR_GPS)) {
        if (gpsSol.fixType == GPS_NO_FIX) fix = 1;
        else if (gpsSol.fixType == GPS_FIX_2D) fix = 2;
        else if (gpsSol.fixType == GPS_FIX_3D) fix = 3;
    }
#endif
    if (SENSOR_ADDRESS_TYPE_LOOKUP[address].value == IBUS_MEAS_VALUE_TEMPERATURE) { //BARO_TEMP\GYRO_TEMP
        int16_t temperature;
        const bool temp_valid = sensors(SENSOR_BARO) ? getBaroTemperature(&temperature) : getIMUTemperature(&temperature);
        if (!temp_valid || (temperature < -400)) temperature = -400; // Minimum reported temperature is -40°C
        return sendIbusMeasurement2(address, (uint16_t)(temperature  + IBUS_TEMPERATURE_OFFSET));
    } else if (SENSOR_ADDRESS_TYPE_LOOKUP[address].value == IBUS_MEAS_VALUE_RPM) {
        return sendIbusMeasurement2(address, (uint16_t) fabsf(rcControlGetOutput()->throttle * 1000.0f));
    } else if (SENSOR_ADDRESS_TYPE_LOOKUP[address].value == IBUS_MEAS_VALUE_EXTERNAL_VOLTAGE) { //VBAT
        if (telemetryConfig()->report_cell_voltage) {
            return sendIbusMeasurement2(address, getBatteryAverageCellVoltage());
        } else {
            return sendIbusMeasurement2(address, getBatteryVoltage());
        }
    } else if (SENSOR_ADDRESS_TYPE_LOOKUP[address].value == IBUS_MEAS_VALUE_CURRENT) { //CURR in 10*mA, 1 = 10 mA
        if (isAmperageConfigured()) return sendIbusMeasurement2(address, (uint16_t) getAmperage()); //int32_t
        else return sendIbusMeasurement2(address, 0);
    } else if (SENSOR_ADDRESS_TYPE_LOOKUP[address].value == IBUS_MEAS_VALUE_FUEL) { //capacity in mAh
        if (isAmperageConfigured()) return sendIbusMeasurement2(address, (uint16_t) getMAhDrawn()); //int32_t
        else return sendIbusMeasurement2(address, 0);
    } else if (SENSOR_ADDRESS_TYPE_LOOKUP[address].value == IBUS_MEAS_VALUE_CLIMB) {
        return sendIbusMeasurement2(address, (int16_t) (getEstimatedActualVelocity(Z))); //
    } else if (SENSOR_ADDRESS_TYPE_LOOKUP[address].value == IBUS_MEAS_VALUE_ACC_Z) { //MAG_COURSE 0-360*, 0=north
        return sendIbusMeasurement2(address, (uint16_t) (attitude.values.yaw * 10)); //in ddeg -> cdeg, 1ddeg = 10cdeg
    } else if (SENSOR_ADDRESS_TYPE_LOOKUP[address].value == IBUS_MEAS_VALUE_ACC_Y) { //PITCH in
        return sendIbusMeasurement2(address, (uint16_t) (-attitude.values.pitch * 10)); //in ddeg -> cdeg, 1ddeg = 10cdeg
    } else if (SENSOR_ADDRESS_TYPE_LOOKUP[address].value == IBUS_MEAS_VALUE_ACC_X) { //ROLL in
        return sendIbusMeasurement2(address, (uint16_t) (attitude.values.roll * 10)); //in ddeg -> cdeg, 1ddeg = 10cdeg
    } else if (SENSOR_ADDRESS_TYPE_LOOKUP[address].value == IBUS_MEAS_VALUE_VSPEED) { //Speed cm/s
#ifdef USE_PITOT
        if (sensors(SENSOR_PITOT)) return sendIbusMeasurement2(address, (uint16_t) (pitot.airSpeed)); //int32_t
        else
#endif
        return sendIbusMeasurement2(address, 0);
    } else if (SENSOR_ADDRESS_TYPE_LOOKUP[address].value == IBUS_MEAS_VALUE_ARMED) { //motorArmed
        if ((telemetryConfig()->ibusTelemetryType & 0x7F) < 8) {
            return sendIbusMeasurement2(address, ARMING_FLAG(ARMED) ? 0 : 1);
        } else {
            return sendIbusMeasurement2(address, ARMING_FLAG(ARMED) ? 1 : 0);
        }
    } else if (SENSOR_ADDRESS_TYPE_LOOKUP[address].value == IBUS_MEAS_VALUE_MODE) {
        uint16_t flightMode = flightModeToIBusTelemetryMode2[getFlightModeForTelemetry()];
        return sendIbusMeasurement2(address, flightMode);
    } else if (SENSOR_ADDRESS_TYPE_LOOKUP[address].value == IBUS_MEAS_VALUE_PRES) { //PRESSURE in dPa -> 9876 is 987.6 hPa
        if (sensors(SENSOR_BARO)) return sendIbusMeasurement2(address, (int16_t) (baro.baroPressure / 10)); //int32_t
        else return sendIbusMeasurement2(address, 0);
    } else if (SENSOR_ADDRESS_TYPE_LOOKUP[address].value == IBUS_MEAS_VALUE_ALT) { //BARO_ALT in cm => m
        if (sensors(SENSOR_BARO)) return sendIbusMeasurement2(address, (uint16_t) baro.BaroAlt); //int32_t
        else return sendIbusMeasurement2(address, 0);
    } else if (SENSOR_ADDRESS_TYPE_LOOKUP[address].value == IBUS_MEAS_VALUE_ALT4) { //BARO_ALT //In cm => m
        if (sensors(SENSOR_BARO)) return sendIbusMeasurement4(address, (int32_t) baro.BaroAlt); //int32_t
        else return sendIbusMeasurement4(address, 0);
    } else if (SENSOR_ADDRESS_TYPE_LOOKUP[address].value == IBUS_MEAS_VALUE_STATUS) { //STATUS sat num AS #0, FIX AS 0, HDOP AS 0, Mode AS 0
        uint16_t status = flightModeToIBusTelemetryMode1[getFlightModeForTelemetry()];
#if defined(USE_GPS)
        if (sensors(SENSOR_GPS)) {
            status += gpsSol.numSat * 1000;
            status += fix * 100;
            if (STATE(GPS_FIX_HOME)) status += 500;
            status += constrain(gpsSol.hdop / 1000, 0, 9) * 10;
        }
#endif
        return sendIbusMeasurement2(address, status);
    } else if (SENSOR_ADDRESS_TYPE_LOOKUP[address].value == IBUS_MEAS_VALUE_HEADING) { //HOME_DIR 0-360deg
#if defined(USE_GPS)
        if (sensors(SENSOR_GPS)) return sendIbusMeasurement2(address, (uint16_t) GPS_directionToHome); else //int16_t
#endif
        return sendIbusMeasurement2(address, 0);
    } else if (SENSOR_ADDRESS_TYPE_LOOKUP[address].value == IBUS_MEAS_VALUE_DIST) { //HOME_DIST in m
#if defined(USE_GPS)
        if (sensors(SENSOR_GPS)) return sendIbusMeasurement2(address, (uint16_t) GPS_distanceToHome); else //uint16_t
#endif
        return sendIbusMeasurement2(address, 0);
    } else if (SENSOR_ADDRESS_TYPE_LOOKUP[address].value == IBUS_MEAS_VALUE_SPE) { //GPS_SPEED in cm/s => km/h, 1cm/s = 0.036 km/h
#if defined(USE_GPS)
        if (sensors(SENSOR_GPS)) return sendIbusMeasurement2(address, (uint16_t) gpsSol.groundSpeed * 36 / 100); else //int16_t
#endif
        return sendIbusMeasurement2(address, 0);
    } else if (SENSOR_ADDRESS_TYPE_LOOKUP[address].value == IBUS_MEAS_VALUE_SPEED) {//SPEED in cm/s
#if defined(USE_GPS)
        if (sensors(SENSOR_GPS)) return sendIbusMeasurement2(address, (uint16_t) gpsSol.groundSpeed); //int16_t
#endif
        return sendIbusMeasurement2(address, 0);
    } else if (SENSOR_ADDRESS_TYPE_LOOKUP[address].value == IBUS_MEAS_VALUE_COG) { //GPS_COURSE (0-360deg, 0=north)
#if defined(USE_GPS)
        if (sensors(SENSOR_GPS)) return sendIbusMeasurement2(address, (uint16_t) (gpsSol.groundCourse / 10)); else //int16_t
#endif
        return sendIbusMeasurement2(address, 0);
    } else if (SENSOR_ADDRESS_TYPE_LOOKUP[address].value == IBUS_MEAS_VALUE_GPS_STATUS) { //GPS_STATUS fix sat
#if defined(USE_GPS)
        if (sensors(SENSOR_GPS)) return sendIbusMeasurement2(address, (((uint16_t)fix)<<8) + gpsSol.numSat); else //uint8_t, uint8_t
#endif
        return sendIbusMeasurement2(address, 0);
    } else if (SENSOR_ADDRESS_TYPE_LOOKUP[address].value == IBUS_MEAS_VALUE_GPS_LAT) { //4byte
#if defined(USE_GPS)
        if (sensors(SENSOR_GPS)) return sendIbusMeasurement4(address, (int32_t)gpsSol.llh.lat); else //int32_t
#endif
        return sendIbusMeasurement4(address, 0);
    } else if (SENSOR_ADDRESS_TYPE_LOOKUP[address].value == IBUS_MEAS_VALUE_GPS_LON) { //4byte
#if defined(USE_GPS)
        if (sensors(SENSOR_GPS)) return sendIbusMeasurement4(address, (int32_t)gpsSol.llh.lon); else //int32_t
#endif
        return sendIbusMeasurement4(address, 0);
    } else if (SENSOR_ADDRESS_TYPE_LOOKUP[address].value == IBUS_MEAS_VALUE_GPS_LAT1) { //GPS_LAT1 //Lattitude * 1e+7
#if defined(USE_GPS)
        if (sensors(SENSOR_GPS)) return sendIbusMeasurement2(address, (uint16_t) (gpsSol.llh.lat / 100000)); else
#endif
        return sendIbusMeasurement2(address, 0);
    } else if (SENSOR_ADDRESS_TYPE_LOOKUP[address].value == IBUS_MEAS_VALUE_GPS_LON1) { //GPS_LON1 //Longitude * 1e+7
#if defined(USE_GPS)
        if (sensors(SENSOR_GPS)) return sendIbusMeasurement2(address, (uint16_t) (gpsSol.llh.lon / 100000)); else
#endif
        return sendIbusMeasurement2(address, 0);
    } else if (SENSOR_ADDRESS_TYPE_LOOKUP[address].value == IBUS_MEAS_VALUE_GPS_LAT2) { //GPS_LAT2 //Lattitude * 1e+7
#if defined(USE_GPS)
        if (sensors(SENSOR_GPS)) return sendIbusMeasurement2(address, (uint16_t) ((gpsSol.llh.lat % 100000)/10));
#endif
        return sendIbusMeasurement2(address, 0);
    } else if (SENSOR_ADDRESS_TYPE_LOOKUP[address].value == IBUS_MEAS_VALUE_GPS_LON2) { //GPS_LON2 //Longitude * 1e+7
#if defined(USE_GPS)
        if (sensors(SENSOR_GPS)) return sendIbusMeasurement2(address, (uint16_t) ((gpsSol.llh.lon % 100000)/10)); else
#endif
        return sendIbusMeasurement2(address, 0);
    } else if (SENSOR_ADDRESS_TYPE_LOOKUP[address].value == IBUS_MEAS_VALUE_GALT4) { //GPS_ALT //In cm => m
#if defined(USE_GPS)
        if (sensors(SENSOR_GPS)) return sendIbusMeasurement4(address, (int32_t) (gpsSol.llh.alt)); else //int32_t
#endif
        return sendIbusMeasurement4(address, 0);
    } else if (SENSOR_ADDRESS_TYPE_LOOKUP[address].value == IBUS_MEAS_VALUE_GALT) { //GPS_ALT //In cm => m
#if defined(USE_GPS)
        if (sensors(SENSOR_GPS)) return sendIbusMeasurement2(address, (uint16_t) (gpsSol.llh.alt / 100)); else //int32_t
#endif
        return sendIbusMeasurement2(address, 0);
    }
    else return 0;
}

uint8_t respondToIbusRequest(uint8_t ibusPacket[static IBUS_RX_BUF_LEN]) {
    ibusAddress_t returnAddress = getAddress(ibusPacket);
    if (returnAddress < sizeof SENSOR_ADDRESS_TYPE_LOOKUP) {
        if (isCommand(IBUS_COMMAND_DISCOVER_SENSOR, ibusPacket)) {
            return sendIbusCommand(returnAddress);
        } else if (isCommand(IBUS_COMMAND_SENSOR_TYPE, ibusPacket)) {
            return sendIbusSensorType(returnAddress);
        } else if (isCommand(IBUS_COMMAND_MEASUREMENT, ibusPacket)) {
            return dispatchMeasurementRequest(returnAddress);
        }
    }
    return 0;
}

void initSharedIbusTelemetry(serialPort_t *port) {
    ibusSerialPort = port;
}

void changeTypeIbusTelemetry(uint8_t id, uint8_t type, uint8_t value) {
    SENSOR_ADDRESS_TYPE_LOOKUP[id].type = type;
    SENSOR_ADDRESS_TYPE_LOOKUP[id].value = value;
    if (value == IBUS_MEAS_VALUE_GPS) SENSOR_ADDRESS_TYPE_LOOKUP[id].size = 14;
    else if (value == IBUS_MEAS_VALUE_GPS_LAT || value == IBUS_MEAS_VALUE_GPS_LON || value == IBUS_MEAS_VALUE_ALT4 || value == IBUS_MEAS_VALUE_GALT4)
      SENSOR_ADDRESS_TYPE_LOOKUP[id].size = 4;
    else SENSOR_ADDRESS_TYPE_LOOKUP[id].size = 2;
}

#endif //defined(USE_TELEMETRY) && defined(USE_TELEMETRY_IBUS)
