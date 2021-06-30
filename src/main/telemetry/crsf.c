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
#include <string.h>

#include "platform.h"
FILE_COMPILE_FOR_SPEED

#if defined(USE_TELEMETRY) && defined(USE_SERIALRX_CRSF) && defined(USE_TELEMETRY_CRSF)

#include "build/atomic.h"
#include "build/build_config.h"
#include "build/version.h"

#include "common/axis.h"
#include "common/crc.h"
#include "common/streambuf.h"
#include "common/time.h"
#include "common/utils.h"
#include "common/printf.h"

#include "config/feature.h"

#include "drivers/serial.h"
#include "drivers/time.h"
#include "drivers/nvic.h"

#include "fc/config.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"

#include "io/gps.h"
#include "io/serial.h"

#include "navigation/navigation.h"

#include "rx/crsf.h"
#include "rx/rx.h"

#include "sensors/battery.h"
#include "sensors/sensors.h"

#include "telemetry/crsf.h"
#include "telemetry/telemetry.h"
#include "telemetry/msp_shared.h"


#define CRSF_CYCLETIME_US                   100000  // 100ms, 10 Hz
#define CRSF_DEVICEINFO_VERSION             0x01
// According to TBS: "CRSF over serial should always use a sync byte at the beginning of each frame.
// To get better performance it's recommended to use the sync byte 0xC8 to get better performance"
//
// Digitalentity: Using frame address byte as a sync field looks somewhat hacky to me, but seems it's needed to get CRSF working properly
#define CRSF_DEVICEINFO_PARAMETER_COUNT     0

#define CRSF_MSP_BUFFER_SIZE 96
#define CRSF_MSP_LENGTH_OFFSET 1

static uint8_t crsfCrc;
static bool crsfTelemetryEnabled;
static bool deviceInfoReplyPending;
static uint8_t crsfFrame[CRSF_FRAME_SIZE_MAX];

#if defined(USE_MSP_OVER_TELEMETRY)
typedef struct mspBuffer_s {
    uint8_t bytes[CRSF_MSP_BUFFER_SIZE];
    int len;
} mspBuffer_t;

static mspBuffer_t mspRxBuffer;

void initCrsfMspBuffer(void)
{
    mspRxBuffer.len = 0;
}

bool bufferCrsfMspFrame(uint8_t *frameStart, int frameLength)
{
    if (mspRxBuffer.len + CRSF_MSP_LENGTH_OFFSET + frameLength > CRSF_MSP_BUFFER_SIZE) {
        return false;
    } else {
        uint8_t *p = mspRxBuffer.bytes + mspRxBuffer.len;
        *p++ = frameLength;
        memcpy(p, frameStart, frameLength);
        mspRxBuffer.len += CRSF_MSP_LENGTH_OFFSET + frameLength;
        return true;
    }
}

bool handleCrsfMspFrameBuffer(uint8_t payloadSize, mspResponseFnPtr responseFn)
{
    bool requestHandled = false;
    if (!mspRxBuffer.len) {
        return false;
    }
    int pos = 0;
    while (true) {
        const int mspFrameLength = mspRxBuffer.bytes[pos];
        if (handleMspFrame(&mspRxBuffer.bytes[CRSF_MSP_LENGTH_OFFSET + pos], mspFrameLength)) {
            requestHandled |= sendMspReply(payloadSize, responseFn);
        }
        pos += CRSF_MSP_LENGTH_OFFSET + mspFrameLength;
        ATOMIC_BLOCK(NVIC_PRIO_SERIALUART) {
            if (pos >= mspRxBuffer.len) {
                mspRxBuffer.len = 0;
                return requestHandled;
            }
        }
    }
    return requestHandled;
}
#endif

static void crsfInitializeFrame(sbuf_t *dst)
{
    crsfCrc = 0;
    dst->ptr = crsfFrame;
    dst->end = ARRAYEND(crsfFrame);

    sbufWriteU8(dst, CRSF_TELEMETRY_SYNC_BYTE);
}

static void crsfSerialize8(sbuf_t *dst, uint8_t v)
{
    sbufWriteU8(dst, v);
    crsfCrc = crc8_dvb_s2(crsfCrc, v);
}

static void crsfSerialize16(sbuf_t *dst, uint16_t v)
{
    // Use BigEndian format
    crsfSerialize8(dst,  (v >> 8));
    crsfSerialize8(dst, (uint8_t)v);
}

static void crsfSerialize32(sbuf_t *dst, uint32_t v)
{
    // Use BigEndian format
    crsfSerialize8(dst, (v >> 24));
    crsfSerialize8(dst, (v >> 16));
    crsfSerialize8(dst, (v >> 8));
    crsfSerialize8(dst, (uint8_t)v);
}

static void crsfSerializeData(sbuf_t *dst, const uint8_t *data, int len)
{
    for (int ii = 0; ii< len; ++ii) {
        crsfSerialize8(dst, data[ii]);
    }
}

static void crsfFinalize(sbuf_t *dst)
{
    sbufWriteU8(dst, crsfCrc);
    sbufSwitchToReader(dst, crsfFrame);
    // write the telemetry frame to the receiver.
    crsfRxWriteTelemetryData(sbufPtr(dst), sbufBytesRemaining(dst));
}

static int crsfFinalizeBuf(sbuf_t *dst, uint8_t *frame)
{
    sbufWriteU8(dst, crsfCrc);
    sbufSwitchToReader(dst, crsfFrame);
    const int frameSize = sbufBytesRemaining(dst);
    for (int ii = 0; sbufBytesRemaining(dst); ++ii) {
        frame[ii] = sbufReadU8(dst);
    }
    return frameSize;
}

/*
CRSF frame has the structure:
<Device address> <Frame length> <Type> <Payload> <CRC>
Device address: (uint8_t)
Frame length:   length in  bytes including Type (uint8_t)
Type:           (uint8_t)
CRC:            (uint8_t), crc of <Type> and <Payload>
*/

/*
0x02 GPS
Payload:
int32_t     Latitude ( degree / 10`000`000 )
int32_t     Longitude (degree / 10`000`000 )
uint16_t    Groundspeed ( km/h / 10 )
uint16_t    GPS heading ( degree / 100 )
uint16      Altitude ( meter ­1000m offset )
uint8_t     Satellites in use ( counter )
*/
static void crsfFrameGps(sbuf_t *dst)
{
    // use sbufWrite since CRC does not include frame length
    sbufWriteU8(dst, CRSF_FRAME_GPS_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE_CRC);
    crsfSerialize8(dst, CRSF_FRAMETYPE_GPS);
    crsfSerialize32(dst, gpsSol.llh.lat); // CRSF and betaflight use same units for degrees
    crsfSerialize32(dst, gpsSol.llh.lon);
    crsfSerialize16(dst, (gpsSol.groundSpeed * 36 + 50) / 100); // gpsSol.groundSpeed is in cm/s
    crsfSerialize16(dst, DECIDEGREES_TO_CENTIDEGREES(gpsSol.groundCourse)); // gpsSol.groundCourse is 0.1 degrees, need 0.01 deg
    const uint16_t altitude = (getEstimatedActualPosition(Z) / 100) + 1000;
    crsfSerialize16(dst, altitude);
    crsfSerialize8(dst, gpsSol.numSat);
}

/*
0x07 Vario sensor
Payload:
int16      Vertical speed ( cm/s )
*/
static void crsfFrameVarioSensor(sbuf_t *dst)
{
    // use sbufWrite since CRC does not include frame length
    sbufWriteU8(dst, CRSF_FRAME_VARIO_SENSOR_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE_CRC);
    crsfSerialize8(dst, CRSF_FRAMETYPE_VARIO_SENSOR);
    crsfSerialize16(dst, lrintf(getEstimatedActualVelocity(Z)));
}

/*
0x08 Battery sensor
Payload:
uint16_t    Voltage ( mV * 100 )
uint16_t    Current ( mA * 100 )
uint24_t    Capacity ( mAh )
uint8_t     Battery remaining ( percent )
*/
static void crsfFrameBatterySensor(sbuf_t *dst)
{
    // use sbufWrite since CRC does not include frame length
    sbufWriteU8(dst, CRSF_FRAME_BATTERY_SENSOR_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE_CRC);
    crsfSerialize8(dst, CRSF_FRAMETYPE_BATTERY_SENSOR);
    if (telemetryConfig()->report_cell_voltage) {
        crsfSerialize16(dst, getBatteryAverageCellVoltage() / 10);
    } else {
        crsfSerialize16(dst, getBatteryVoltage() / 10); // vbat is in units of 0.01V
    }
    crsfSerialize16(dst, getAmperage() / 10);
    const uint8_t batteryRemainingPercentage = calculateBatteryPercentage();
    crsfSerialize8(dst, (getMAhDrawn() >> 16));
    crsfSerialize8(dst, (getMAhDrawn() >> 8));
    crsfSerialize8(dst, (uint8_t)getMAhDrawn());
    crsfSerialize8(dst, batteryRemainingPercentage);
}

typedef enum {
    CRSF_ACTIVE_ANTENNA1 = 0,
    CRSF_ACTIVE_ANTENNA2 = 1
} crsfActiveAntenna_e;

typedef enum {
    CRSF_RF_MODE_4_HZ = 0,
    CRSF_RF_MODE_50_HZ = 1,
    CRSF_RF_MODE_150_HZ = 2
} crsrRfMode_e;

typedef enum {
    CRSF_RF_POWER_0_mW = 0,
    CRSF_RF_POWER_10_mW = 1,
    CRSF_RF_POWER_25_mW = 2,
    CRSF_RF_POWER_100_mW = 3,
    CRSF_RF_POWER_500_mW = 4,
    CRSF_RF_POWER_1000_mW = 5,
    CRSF_RF_POWER_2000_mW = 6,
    CRSF_RF_POWER_250_mW = 7
} crsrRfPower_e;

/*
0x1E Attitude
Payload:
int16_t     Pitch angle ( rad / 10000 )
int16_t     Roll angle ( rad / 10000 )
int16_t     Yaw angle ( rad / 10000 )
*/

#define DECIDEGREES_TO_RADIANS10000(angle) ((int16_t)(1000.0f * (angle) * RAD))

static void crsfFrameAttitude(sbuf_t *dst)
{
     sbufWriteU8(dst, CRSF_FRAME_ATTITUDE_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE_CRC);
     crsfSerialize8(dst, CRSF_FRAMETYPE_ATTITUDE);
     crsfSerialize16(dst, DECIDEGREES_TO_RADIANS10000(attitude.values.pitch));
     crsfSerialize16(dst, DECIDEGREES_TO_RADIANS10000(attitude.values.roll));
     crsfSerialize16(dst, DECIDEGREES_TO_RADIANS10000(attitude.values.yaw));
}

/*
0x21 Flight mode text based
Payload:
char[]      Flight mode ( Null­terminated string )
*/
static void crsfFrameFlightMode(sbuf_t *dst)
{
    // just do "OK" for the moment as a placeholder
    // write zero for frame length, since we don't know it yet
    uint8_t *lengthPtr = sbufPtr(dst);
    sbufWriteU8(dst, 0);
    crsfSerialize8(dst, CRSF_FRAMETYPE_FLIGHT_MODE);

    // use same logic as OSD, so telemetry displays same flight text as OSD when armed
    const char *flightMode = "OK";
    if (ARMING_FLAG(ARMED)) {
        if (STATE(AIRMODE_ACTIVE)) {
            flightMode = "AIR";
        } else {
            flightMode = "ACRO";
        }
        if (FLIGHT_MODE(FAILSAFE_MODE)) {
            flightMode = "!FS!";
        } else if (ARMING_FLAG(ARMED) && IS_RC_MODE_ACTIVE(BOXHOMERESET) && !FLIGHT_MODE(NAV_RTH_MODE) && !FLIGHT_MODE(NAV_WP_MODE)) {
            flightMode = "HRST";
        } else if (FLIGHT_MODE(MANUAL_MODE)) {
            flightMode = "MANU";
        } else if (FLIGHT_MODE(NAV_RTH_MODE)) {
            flightMode = "RTH";
        } else if (FLIGHT_MODE(NAV_POSHOLD_MODE)) {
            flightMode = "HOLD";
        } else if (FLIGHT_MODE(NAV_COURSE_HOLD_MODE) && FLIGHT_MODE(NAV_ALTHOLD_MODE)) {
            flightMode = "CRUZ";
        } else if (FLIGHT_MODE(NAV_COURSE_HOLD_MODE)) {
            flightMode = "CRSH";
        } else if (FLIGHT_MODE(NAV_ALTHOLD_MODE)) {
            flightMode = "AH";
        } else if (FLIGHT_MODE(NAV_WP_MODE)) {
            flightMode = "WP";
        } else if (FLIGHT_MODE(ANGLE_MODE)) {
            flightMode = "ANGL";
        } else if (FLIGHT_MODE(HORIZON_MODE)) {
            flightMode = "HOR";
        }
#ifdef USE_GPS
    } else if (feature(FEATURE_GPS) && navConfig()->general.flags.extra_arming_safety && (!STATE(GPS_FIX) || !STATE(GPS_FIX_HOME))) {
        flightMode = "WAIT"; // Waiting for GPS lock
#endif
    } else if (isArmingDisabled()) {
        flightMode = "!ERR";
    }

    crsfSerializeData(dst, (const uint8_t*)flightMode, strlen(flightMode));
    crsfSerialize8(dst, 0); // zero terminator for string
    // write in the length
    *lengthPtr = sbufPtr(dst) - lengthPtr;
}

/*
0x29 Device Info
Payload:
uint8_t     Destination
uint8_t     Origin
char[]      Device Name ( Null terminated string )
uint32_t    Null Bytes
uint32_t    Null Bytes
uint32_t    Null Bytes
uint8_t     255 (Max MSP Parameter)
uint8_t     0x01 (Parameter version 1)
*/
static void crsfFrameDeviceInfo(sbuf_t *dst)
{
    char buff[30];
    tfp_sprintf(buff, "%s %s: %s", FC_FIRMWARE_NAME, FC_VERSION_STRING, TARGET_BOARD_IDENTIFIER);

    uint8_t *lengthPtr = sbufPtr(dst);
    sbufWriteU8(dst, 0);
    crsfSerialize8(dst, CRSF_FRAMETYPE_DEVICE_INFO);
    crsfSerialize8(dst, CRSF_ADDRESS_RADIO_TRANSMITTER);
    crsfSerialize8(dst, CRSF_ADDRESS_FLIGHT_CONTROLLER);
    crsfSerializeData(dst, (const uint8_t*)buff, strlen(buff));
    crsfSerialize8(dst, 0); // zero terminator for string
	for (unsigned int ii=0; ii<12; ii++) {
        crsfSerialize8(dst, 0x00);
    }
    crsfSerialize8(dst, CRSF_DEVICEINFO_PARAMETER_COUNT);
    crsfSerialize8(dst, CRSF_DEVICEINFO_VERSION);
    *lengthPtr = sbufPtr(dst) - lengthPtr;
}

#define BV(x)  (1 << (x)) // bit value

// schedule array to decide how often each type of frame is sent
typedef enum {
    CRSF_FRAME_START_INDEX = 0,
    CRSF_FRAME_ATTITUDE_INDEX = CRSF_FRAME_START_INDEX,
    CRSF_FRAME_BATTERY_SENSOR_INDEX,
    CRSF_FRAME_FLIGHT_MODE_INDEX,
    CRSF_FRAME_GPS_INDEX,
    CRSF_FRAME_VARIO_SENSOR_INDEX,
    CRSF_SCHEDULE_COUNT_MAX
} crsfFrameTypeIndex_e;

static uint8_t crsfScheduleCount;
static uint8_t crsfSchedule[CRSF_SCHEDULE_COUNT_MAX];

#if defined(USE_MSP_OVER_TELEMETRY)

static bool mspReplyPending;

void crsfScheduleMspResponse(void)
{
    mspReplyPending = true;
}

void crsfSendMspResponse(uint8_t *payload)
{
    sbuf_t crsfPayloadBuf;
    sbuf_t *dst = &crsfPayloadBuf;

    crsfInitializeFrame(dst);
    sbufWriteU8(dst, CRSF_FRAME_TX_MSP_FRAME_SIZE + CRSF_FRAME_LENGTH_EXT_TYPE_CRC);
    crsfSerialize8(dst, CRSF_FRAMETYPE_MSP_RESP);
    crsfSerialize8(dst, CRSF_ADDRESS_RADIO_TRANSMITTER);
    crsfSerialize8(dst, CRSF_ADDRESS_FLIGHT_CONTROLLER);
    crsfSerializeData(dst, (const uint8_t*)payload, CRSF_FRAME_TX_MSP_FRAME_SIZE);
    crsfFinalize(dst);
}
#endif

static void processCrsf(void)
{
    static uint8_t crsfScheduleIndex = 0;
    const uint8_t currentSchedule = crsfSchedule[crsfScheduleIndex];

    sbuf_t crsfPayloadBuf;
    sbuf_t *dst = &crsfPayloadBuf;

    if (currentSchedule & BV(CRSF_FRAME_ATTITUDE_INDEX)) {
        crsfInitializeFrame(dst);
        crsfFrameAttitude(dst);
        crsfFinalize(dst);
    }
    if (currentSchedule & BV(CRSF_FRAME_BATTERY_SENSOR_INDEX)) {
        crsfInitializeFrame(dst);
        crsfFrameBatterySensor(dst);
        crsfFinalize(dst);
    }
    if (currentSchedule & BV(CRSF_FRAME_FLIGHT_MODE_INDEX)) {
        crsfInitializeFrame(dst);
        crsfFrameFlightMode(dst);
        crsfFinalize(dst);
    }
#ifdef USE_GPS
    if (currentSchedule & BV(CRSF_FRAME_GPS_INDEX)) {
        crsfInitializeFrame(dst);
        crsfFrameGps(dst);
        crsfFinalize(dst);
    }
#endif
#if defined(USE_BARO) || defined(USE_GPS)
    if (currentSchedule & BV(CRSF_FRAME_VARIO_SENSOR_INDEX)) {
        crsfInitializeFrame(dst);
        crsfFrameVarioSensor(dst);
        crsfFinalize(dst);
    }
#endif
    crsfScheduleIndex = (crsfScheduleIndex + 1) % crsfScheduleCount;
}

void crsfScheduleDeviceInfoResponse(void)
{
    deviceInfoReplyPending = true;
}

void initCrsfTelemetry(void)
{
    // check if there is a serial port open for CRSF telemetry (ie opened by the CRSF RX)
    // and feature is enabled, if so, set CRSF telemetry enabled
    crsfTelemetryEnabled = crsfRxIsActive();

    deviceInfoReplyPending = false;
#if defined(USE_MSP_OVER_TELEMETRY)
    mspReplyPending = false;
#endif

    int index = 0;
    crsfSchedule[index++] = BV(CRSF_FRAME_ATTITUDE_INDEX);
    crsfSchedule[index++] = BV(CRSF_FRAME_BATTERY_SENSOR_INDEX);
    crsfSchedule[index++] = BV(CRSF_FRAME_FLIGHT_MODE_INDEX);
#ifdef USE_GPS
    if (feature(FEATURE_GPS)) {
        crsfSchedule[index++] = BV(CRSF_FRAME_GPS_INDEX);
    }
#endif
#if defined(USE_BARO) || defined(USE_GPS)
    if (sensors(SENSOR_BARO) || (STATE(FIXED_WING_LEGACY) && feature(FEATURE_GPS))) {
        crsfSchedule[index++] = BV(CRSF_FRAME_VARIO_SENSOR_INDEX);
    }
#endif
    crsfScheduleCount = (uint8_t)index;
}

bool checkCrsfTelemetryState(void)
{
    return crsfTelemetryEnabled;
}

/*
 * Called periodically by the scheduler
 */
void handleCrsfTelemetry(timeUs_t currentTimeUs)
{
    static uint32_t crsfLastCycleTime;

    if (!crsfTelemetryEnabled) {
        return;
    }
    // Give the receiver a chance to send any outstanding telemetry data.
    // This needs to be done at high frequency, to enable the RX to send the telemetry frame
    // in between the RX frames.
    crsfRxSendTelemetryData();

    // Send ad-hoc response frames as soon as possible
#if defined(USE_MSP_OVER_TELEMETRY)
    if (mspReplyPending) {
        mspReplyPending = handleCrsfMspFrameBuffer(CRSF_FRAME_TX_MSP_FRAME_SIZE, &crsfSendMspResponse);
        crsfLastCycleTime = currentTimeUs; // reset telemetry timing due to ad-hoc request
        return;
    }
#endif

    if (deviceInfoReplyPending) {
        sbuf_t crsfPayloadBuf;
        sbuf_t *dst = &crsfPayloadBuf;
        crsfInitializeFrame(dst);
        crsfFrameDeviceInfo(dst);
        crsfFinalize(dst);
        deviceInfoReplyPending = false;
        crsfLastCycleTime = currentTimeUs; // reset telemetry timing due to ad-hoc request
        return;
    }

    // Actual telemetry data only needs to be sent at a low frequency, ie 10Hz
    // Spread out scheduled frames evenly so each frame is sent at the same frequency.
    if (currentTimeUs >= crsfLastCycleTime + (CRSF_CYCLETIME_US / crsfScheduleCount)) {
        crsfLastCycleTime = currentTimeUs;
        processCrsf();
    }
}

int getCrsfFrame(uint8_t *frame, crsfFrameType_e frameType)
{
    sbuf_t crsfFrameBuf;
    sbuf_t *sbuf = &crsfFrameBuf;

    crsfInitializeFrame(sbuf);
    switch (frameType) {
    default:
    case CRSF_FRAMETYPE_ATTITUDE:
        crsfFrameAttitude(sbuf);
        break;
    case CRSF_FRAMETYPE_BATTERY_SENSOR:
        crsfFrameBatterySensor(sbuf);
        break;
    case CRSF_FRAMETYPE_FLIGHT_MODE:
        crsfFrameFlightMode(sbuf);
        break;
#if defined(USE_GPS)
    case CRSF_FRAMETYPE_GPS:
        crsfFrameGps(sbuf);
        break;
#endif
    case CRSF_FRAMETYPE_VARIO_SENSOR:
        crsfFrameVarioSensor(sbuf);
        break;
    }
    const int frameSize = crsfFinalizeBuf(sbuf, frame);
    return frameSize;
}
#endif
