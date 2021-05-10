/*
 * This file is part of iNav.
 *
 * iNav is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * iNav is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with iNav.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"
FILE_COMPILE_FOR_SPEED

#if defined(USE_SMARTPORT_MASTER)

#include "build/debug.h"

#include "common/utils.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/serial.h"
#include "drivers/time.h"

#include "fc/settings.h"

#include "io/serial.h"
#include "io/smartport_master.h"

#include "rx/frsky_crc.h"

enum {
    PRIM_DISCARD_FRAME = 0x00,
    PRIM_DATA_FRAME = 0x10,
    PRIM_WORKING_STATE = 0x20,
    PRIM_IDLE_STATE = 0x21,
    PRIM_CONFIG_READ = 0x30,
    PRIM_CONFIG_WRITE = 0x31,
    PRIM_CONFIG_RESPONSE = 0x32,
    PRIM_DIAG_READ = 0X40,
    PRIM_DIAG_WRITE = 0X41,
    PRIM_ENABLE_APP_NODE = 0x70,
    PRIM_DISABLE_APP_NODE = 0x71,
};

enum
{
    DATAID_SPEED      = 0x0830,
    DATAID_VFAS       = 0x0210,
    DATAID_CURRENT    = 0x0200,
    DATAID_RPM        = 0x050F,
    DATAID_ALTITUDE   = 0x0100,
    DATAID_FUEL       = 0x0600,
    DATAID_ADC1       = 0xF102,
    DATAID_ADC2       = 0xF103,
    DATAID_LATLONG    = 0x0800,
    DATAID_CAP_USED   = 0x0600,
    DATAID_VARIO      = 0x0110,
    DATAID_CELLS      = 0x0300,
    DATAID_CELLS_LAST = 0x030F,
    DATAID_HEADING    = 0x0840,
    DATAID_FPV        = 0x0450,
    DATAID_PITCH      = 0x0430,
    DATAID_ROLL       = 0x0440,
    DATAID_ACCX       = 0x0700,
    DATAID_ACCY       = 0x0710,
    DATAID_ACCZ       = 0x0720,
    DATAID_T1         = 0x0400,
    DATAID_T2         = 0x0410,
    DATAID_HOME_DIST  = 0x0420,
    DATAID_GPS_ALT    = 0x0820,
    DATAID_ASPD       = 0x0A00,
    DATAID_A3         = 0x0900,
    DATAID_A4         = 0x0910,
    DATAID_VS600      = 0x0E10
};

#define SMARTPORT_BAUD 57600
#define SMARTPORT_UART_MODE MODE_RXTX

#define SMARTPORT_PHYID_MAX 0x1B
#define SMARTPORT_PHYID_COUNT (SMARTPORT_PHYID_MAX + 1)

#define SMARTPORT_POLLING_INTERVAL 12 // ms

#define SMARTPORT_FRAME_START 0x7E
#define SMARTPORT_BYTESTUFFING_MARKER 0x7D
#define SMARTPORT_BYTESTUFFING_XOR_VALUE 0x20

#define SMARTPORT_SENSOR_DATA_TIMEOUT 500 // ms

#define SMARTPORT_FORWARD_REQUESTS_MAX 10

typedef enum {
    PT_ACTIVE_ID,
    PT_INACTIVE_ID
} pollType_e;

typedef struct smartPortMasterFrame_s {
    uint8_t magic;
    uint8_t phyID;
    smartPortPayload_t payload;
    uint8_t crc;
} PACKED smartportFrame_t;

typedef union {
    smartportFrame_t frame;
    uint8_t bytes[sizeof(smartportFrame_t)];
} smartportFrameBuffer_u;

typedef struct {
    cellsData_t cells;
    timeUs_t cellsTimestamp;
    vs600Data_t vs600;
    timeUs_t vs600Timestamp;
    int32_t altitude;
    timeUs_t altitudeTimestamp;
    int16_t vario;
    timeUs_t varioTimestamp;
} smartportSensorsData_t;

typedef struct {
    uint8_t phyID;
    smartPortPayload_t payload;
} smartportForwardData_t;

PG_REGISTER_WITH_RESET_TEMPLATE(smartportMasterConfig_t, smartportMasterConfig, PG_SMARTPORT_MASTER_CONFIG, 0);

PG_RESET_TEMPLATE(smartportMasterConfig_t, smartportMasterConfig,
    .halfDuplex = SETTING_SMARTPORT_MASTER_HALFDUPLEX_DEFAULT,
    .inverted = SETTING_SMARTPORT_MASTER_INVERTED_DEFAULT
);

static serialPort_t *smartportMasterSerialPort = NULL;
static serialPortConfig_t *portConfig;
static int8_t currentPolledPhyID = -1;
static int8_t forcedPolledPhyID = -1;
static uint8_t rxBufferLen = 0;

static uint32_t activePhyIDs = 0;
static smartPortPayload_t sensorPayloadCache[SMARTPORT_PHYID_COUNT];

static uint8_t forwardRequestsStart = 0;
static uint8_t forwardRequestsEnd = 0;
static smartportForwardData_t forwardRequests[SMARTPORT_FORWARD_REQUESTS_MAX]; // Forward requests' circular buffer

static uint8_t forwardResponsesCount = 0;
static smartportForwardData_t forwardResponses[SMARTPORT_FORWARD_REQUESTS_MAX]; // Forward responses' buffer

static smartportSensorsData_t sensorsData;


bool smartportMasterInit(void)
{
    portConfig = findSerialPortConfig(FUNCTION_TELEMETRY_SMARTPORT_MASTER);
    if (!portConfig) {
        return false;
    }

    portOptions_t portOptions = (smartportMasterConfig()->halfDuplex ? SERIAL_BIDIR : SERIAL_UNIDIR) | (smartportMasterConfig()->inverted ? SERIAL_NOT_INVERTED : SERIAL_INVERTED);
    smartportMasterSerialPort = openSerialPort(portConfig->identifier, FUNCTION_TELEMETRY_SMARTPORT_MASTER, NULL, NULL, SMARTPORT_BAUD, SMARTPORT_UART_MODE, portOptions);

    memset(&sensorsData, 0, sizeof(sensorsData));

    return true;
}

static void phyIDSetActive(uint8_t phyID, bool active)
{
    uint32_t mask = 1 << phyID;
    if (active) {
        activePhyIDs |= mask;
    } else {
        activePhyIDs &= ~mask;
    }
}

static uint32_t phyIDAllActiveMask(void)
{
    uint32_t mask = 0;
    for (uint8_t i = 0; i < SMARTPORT_PHYID_COUNT; ++i) {
        mask |= 1 << i;
    }
    return mask;
}

static int8_t phyIDNext(uint8_t start, bool active, bool loop)
{
    for (uint8_t i = start; i < ((loop ? start : 0) + SMARTPORT_PHYID_COUNT); ++i) {
        uint8_t phyID = i % SMARTPORT_PHYID_COUNT;
        uint32_t mask = 1 << phyID;
        uint32_t phyIDMasked = activePhyIDs & mask;
        if ((active && phyIDMasked) || !(active || phyIDMasked)) {
            return phyID;
        }
    }
    return -1;
}

static bool phyIDIsActive(uint8_t id)
{
    return !!(activePhyIDs & (1 << id));
}

static bool phyIDNoneActive(void)
{
    return activePhyIDs == 0;
}

static bool phyIDAllActive(void)
{
    static uint32_t allActiveMask = 0;

    if (!allActiveMask) {
        allActiveMask = phyIDAllActiveMask();
    }

    return !!((activePhyIDs & allActiveMask) == allActiveMask);
}

static bool phyIDAnyActive(void)
{
    return !!activePhyIDs;
}

static void smartportMasterSendByte(uint8_t byte)
{
    serialWrite(smartportMasterSerialPort, byte);
}

static void smartportMasterPhyIDFillCheckBits(uint8_t *phyIDByte)
{
    *phyIDByte |= (GET_BIT(*phyIDByte, 0) ^ GET_BIT(*phyIDByte, 1) ^ GET_BIT(*phyIDByte, 2)) << 5;
    *phyIDByte |= (GET_BIT(*phyIDByte, 2) ^ GET_BIT(*phyIDByte, 3) ^ GET_BIT(*phyIDByte, 4)) << 6;
    *phyIDByte |= (GET_BIT(*phyIDByte, 0) ^ GET_BIT(*phyIDByte, 2) ^ GET_BIT(*phyIDByte, 4)) << 7;
}

static void smartportMasterPoll(void)
{
    static pollType_e nextPollType = PT_INACTIVE_ID;
    static uint8_t nextActivePhyID = 0, nextInactivePhyID = 0;
    uint8_t phyIDToPoll;

    if (currentPolledPhyID != -1) {
        // currentPolledPhyID hasn't been reset by smartportMasterReceive so we didn't get valid data for this PhyID (inactive)
        phyIDSetActive(currentPolledPhyID, false);
    }

    if (forcedPolledPhyID != -1) {

        phyIDToPoll = forcedPolledPhyID;
        forcedPolledPhyID = -1;

    } else {

        if (phyIDNoneActive()) {
            nextPollType = PT_INACTIVE_ID;
        } else if (phyIDAllActive()) {
            nextPollType = PT_ACTIVE_ID;
        }

        switch (nextPollType) {

            case PT_ACTIVE_ID: {
                int8_t activePhyIDToPoll = phyIDNext(nextActivePhyID, true, false);
                if (activePhyIDToPoll == -1) {
                    nextActivePhyID = 0;
                    nextPollType = PT_INACTIVE_ID;
                } else {
                    phyIDToPoll = activePhyIDToPoll;
                    if (phyIDToPoll == SMARTPORT_PHYID_MAX) {
                        nextActivePhyID = 0;
                        if (!phyIDAllActive()) {
                            nextPollType = PT_INACTIVE_ID;
                        }
                    } else {
                        nextActivePhyID = phyIDToPoll + 1;
                    }
                    break;
                }
                FALLTHROUGH;
            }

            case PT_INACTIVE_ID: {
                phyIDToPoll = phyIDNext(nextInactivePhyID, false, true);
                nextInactivePhyID = (phyIDToPoll == SMARTPORT_PHYID_MAX ? 0 : phyIDToPoll + 1);
                if (phyIDAnyActive()) {
                    nextPollType = PT_ACTIVE_ID;
                }
                break;
            }

            default: // should not happen
                return;

        }

    }

    currentPolledPhyID = phyIDToPoll;
    smartportMasterPhyIDFillCheckBits(&phyIDToPoll);

    // poll
    smartportMasterSendByte(SMARTPORT_FRAME_START);
    smartportMasterSendByte(phyIDToPoll);

    rxBufferLen = 0; // discard incomplete frames received during previous poll
}

static void smartportMasterForwardNextPayload(void)
{
    smartportForwardData_t *request = forwardRequests + forwardRequestsStart;

    /*forcedPolledPhyID = request->phyID; // force next poll to the request's phyID          XXX disabled, doesn't seem necessary */

    smartportMasterPhyIDFillCheckBits(&request->phyID);
    smartportMasterSendByte(SMARTPORT_FRAME_START);
    smartportMasterSendByte(request->phyID);

    uint16_t checksum = 0;
    uint8_t *payloadPtr = (uint8_t *)&request->payload;
    for (uint8_t i = 0; i < sizeof(request->payload); ++i) {
        uint8_t c = *payloadPtr++;
        if ((c == SMARTPORT_FRAME_START) || (c == SMARTPORT_BYTESTUFFING_MARKER)) {
            smartportMasterSendByte(SMARTPORT_BYTESTUFFING_MARKER);
            smartportMasterSendByte(c ^ SMARTPORT_BYTESTUFFING_XOR_VALUE);
        } else {
            smartportMasterSendByte(c);
        }
        checksum += c;
    }
    checksum = 0xff - ((checksum & 0xff) + (checksum >> 8));
    smartportMasterSendByte(checksum);

    forwardRequestsStart = (forwardRequestsStart + 1) % SMARTPORT_FORWARD_REQUESTS_MAX;
}

static void decodeCellsData(uint32_t sdata)
{
    uint8_t voltageStartIndex = sdata & 0xF;
    uint8_t count = sdata >> 4 & 0xF;
    uint16_t voltage1 = (sdata >> 8 & 0xFFF) * 2;
    uint16_t voltage2 = (sdata >> 20 & 0xFFF) * 2;
    if ((voltageStartIndex <= 4) && (count <= 6)) { // sanity check
        cellsData_t *cd = &sensorsData.cells;
        cd->count = count;
        cd->voltage[voltageStartIndex] = voltage1;
        cd->voltage[voltageStartIndex+1] = voltage2;

        DEBUG_SET(DEBUG_SPM_CELLS, 0, cd->count);
        for (uint8_t i = 0; i < 6; ++i) {
            DEBUG_SET(DEBUG_SPM_CELLS, i+1, cd->voltage[i]);
        }
    }
}

static void decodeVS600Data(uint32_t sdata)
{
    vs600Data_t *vs600 = &sensorsData.vs600;
    vs600->power = (sdata >> 8) & 0xFF;
    vs600->band = (sdata >> 16) & 0xFF;
    vs600->channel = (sdata >> 24) & 0xFF;
    DEBUG_SET(DEBUG_SPM_VS600, 0, sdata);
    DEBUG_SET(DEBUG_SPM_VS600, 1, vs600->channel);
    DEBUG_SET(DEBUG_SPM_VS600, 2, vs600->band);
    DEBUG_SET(DEBUG_SPM_VS600, 3, vs600->power);
}

static void decodeAltitudeData(uint32_t sdata)
{
    sensorsData.altitude = sdata * 5; // cm
    DEBUG_SET(DEBUG_SPM_VARIO, 0, sensorsData.altitude);
}

static void decodeVarioData(uint32_t sdata)
{
    sensorsData.vario = sdata * 2; // mm/s
    DEBUG_SET(DEBUG_SPM_VARIO, 1, sensorsData.vario);
}

static void processSensorPayload(smartPortPayload_t *payload, timeUs_t currentTimeUs)
{
    switch (payload->valueId) {
        case DATAID_CELLS:
            decodeCellsData(payload->data);
            sensorsData.cellsTimestamp = currentTimeUs;
            break;

        case DATAID_VS600:
            decodeVS600Data(payload->data);
            sensorsData.vs600Timestamp = currentTimeUs;
            break;

        case DATAID_ALTITUDE:
            decodeAltitudeData(payload->data);
            sensorsData.altitudeTimestamp = currentTimeUs;
            break;

        case DATAID_VARIO:
            decodeVarioData(payload->data);
            sensorsData.varioTimestamp = currentTimeUs;
            break;
    }
    sensorPayloadCache[currentPolledPhyID] = *payload;
}

static void processPayload(smartPortPayload_t *payload, timeUs_t currentTimeUs)
{
    switch (payload->frameId) {

        case PRIM_DISCARD_FRAME:
            break;

        case PRIM_DATA_FRAME:
            processSensorPayload(payload, currentTimeUs);
            break;

        default:
            if (forwardResponsesCount < SMARTPORT_FORWARD_REQUESTS_MAX) {
                smartportForwardData_t *response = forwardResponses + forwardResponsesCount;
                response->phyID = currentPolledPhyID;
                response->payload = *payload;
                forwardResponsesCount += 1;
            }
            break;
    }
}

static void smartportMasterReceive(timeUs_t currentTimeUs)
{
    static smartportFrameBuffer_u buffer;
    static bool processByteStuffing = false;

    if (!rxBufferLen) {
        processByteStuffing = false;
    }

    while (serialRxBytesWaiting(smartportMasterSerialPort)) {

        uint8_t c = serialRead(smartportMasterSerialPort);

        if (currentPolledPhyID == -1) { // We did not poll a sensor or a packet has already been received and processed
            continue;
        }

        if (processByteStuffing) {
            c ^= SMARTPORT_BYTESTUFFING_XOR_VALUE;
            processByteStuffing = false;
        } else if (c == SMARTPORT_BYTESTUFFING_MARKER) {
            processByteStuffing = true;
            continue;
        }

        buffer.bytes[rxBufferLen] = c;
        rxBufferLen += 1;

        if (rxBufferLen == sizeof(buffer)) { // frame complete

            uint8_t calcCRC = frskyCheckSum((uint8_t *)&buffer.frame.payload, sizeof(buffer.frame.payload));

            if (buffer.frame.crc == calcCRC) {
                phyIDSetActive(currentPolledPhyID, true);
                processPayload(&buffer.frame.payload, currentTimeUs);
            }

            currentPolledPhyID = -1; // previously polled PhyID has answered, not expecting more data until next poll
            rxBufferLen = 0; // reset buffer

        }

    }
}

bool smartportMasterGetSensorPayload(uint8_t phyID, smartPortPayload_t *payload)
{
    if ((phyID > SMARTPORT_PHYID_MAX) || !phyIDIsActive(phyID)) {
        return false;
    }

    *payload = sensorPayloadCache[phyID];
    return true;
}

bool smartportMasterHasForwardResponse(uint8_t phyID)
{
    for (uint8_t i = 0; i < forwardResponsesCount; ++i) {
        smartportForwardData_t *response = forwardResponses + i;
        if (response->phyID == phyID) {
            return true;
        }
    }

    return false;
}

bool smartportMasterNextForwardResponse(uint8_t phyID, smartPortPayload_t *payload)
{
    for (uint8_t i = 0; i < forwardResponsesCount; ++i) {
        smartportForwardData_t *response = forwardResponses + i;
        if (response->phyID == phyID) {
            *payload = response->payload;
            forwardResponsesCount -= 1;
            memmove(response, response + 1, (forwardResponsesCount - i) * sizeof(*response));
            return true;
        }
    }

    return false;
}

static uint8_t forwardRequestCount(void)
{
    if (forwardRequestsStart > forwardRequestsEnd) {
        return SMARTPORT_FORWARD_REQUESTS_MAX - forwardRequestsStart + forwardRequestsEnd;
    } else {
        return forwardRequestsEnd - forwardRequestsStart;
    }
}

bool smartportMasterForward(uint8_t phyID, smartPortPayload_t *payload)
{
    if (forwardRequestCount() == SMARTPORT_FORWARD_REQUESTS_MAX) {
        return false;
    }

    smartportForwardData_t *request = forwardRequests + forwardRequestsEnd;
    request->phyID = phyID;
    request->payload = *payload;
    forwardRequestsEnd = (forwardRequestsEnd + 1) % SMARTPORT_FORWARD_REQUESTS_MAX;
    return true;
}

void smartportMasterHandle(timeUs_t currentTimeUs)
{
    static timeUs_t pollTimestamp = 0;

    if (!smartportMasterSerialPort) {
        return;
    }

    if (!pollTimestamp || (cmpTimeUs(currentTimeUs, pollTimestamp) > MS2US(SMARTPORT_POLLING_INTERVAL))) {
        if (forwardRequestCount() && (forcedPolledPhyID == -1)) { // forward next payload if there is one in queue and we are not waiting from the response of the previous one
            smartportMasterForwardNextPayload();
        } else {
            smartportMasterPoll();
        }
        pollTimestamp = currentTimeUs;
    } else {
        smartportMasterReceive(currentTimeUs);
    }
}

cellsData_t *smartportMasterGetCellsData(void)
{
    if (micros() - sensorsData.cellsTimestamp > SMARTPORT_SENSOR_DATA_TIMEOUT) {
        return NULL;
    }

    return &sensorsData.cells;
}

vs600Data_t *smartportMasterGetVS600Data(void)
{
    if (micros() - sensorsData.vs600Timestamp > SMARTPORT_SENSOR_DATA_TIMEOUT) {
        return NULL;
    }

    return &sensorsData.vs600;
}

bool smartportMasterPhyIDIsActive(uint8_t phyID)
{
    return phyIDIsActive(phyID);
}

int8_t smartportMasterStripPhyIDCheckBits(uint8_t phyID)
{
    uint8_t smartportPhyID = phyID & 0x1F;
    uint8_t phyIDCheck = smartportPhyID;
    smartportMasterPhyIDFillCheckBits(&phyIDCheck);
    return phyID == phyIDCheck ? smartportPhyID : -1;
}

#endif
