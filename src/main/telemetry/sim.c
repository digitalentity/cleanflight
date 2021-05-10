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

#include "platform.h"

#if defined(USE_TELEMETRY) && defined(USE_TELEMETRY_SIM)

#include <string.h>

#include "common/printf.h"
#include "common/olc.h"

#include "drivers/time.h"

#include "fc/fc_core.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"

#include "io/gps.h"
#include "io/serial.h"

#include "navigation/navigation.h"

#include "sensors/battery.h"
#include "sensors/sensors.h"

#include "common/string_light.h"
#include "common/typeconversion.h"

#include "telemetry/sim.h"
#include "telemetry/telemetry.h"

#define SIM_AT_COMMAND_MAX_SIZE 255
#define SIM_RESPONSE_BUFFER_SIZE 255
#define SIM_CYCLE_MS 5000                       // wait between sim command cycles
#define SIM_AT_COMMAND_DELAY_MS 3000
#define SIM_AT_COMMAND_DELAY_MIN_MS 500
#define SIM_STARTUP_DELAY_MS 10000
#define SIM_SMS_COMMAND_RTH "RTH"
#define SIM_LOW_ALT_WARNING_MODES (NAV_ALTHOLD_MODE | NAV_RTH_MODE | NAV_WP_MODE | FAILSAFE_MODE)

#define SIM_RESPONSE_CODE_OK    ('O' << 24 | 'K' << 16)
#define SIM_RESPONSE_CODE_ERROR ('E' << 24 | 'R' << 16 | 'R' << 8 | 'O')
#define SIM_RESPONSE_CODE_RING  ('R' << 24 | 'I' << 16 | 'N' << 8 | 'G')
#define SIM_RESPONSE_CODE_CLIP  ('C' << 24 | 'L' << 16 | 'I' << 8 | 'P')
#define SIM_RESPONSE_CODE_CREG  ('C' << 24 | 'R' << 16 | 'E' << 8 | 'G')
#define SIM_RESPONSE_CODE_CSQ   ('C' << 24 | 'S' << 16 | 'Q' << 8 | ':')
#define SIM_RESPONSE_CODE_CMT   ('C' << 24 | 'M' << 16 | 'T' << 8 | ':')


typedef enum  {
    SIM_MODULE_NOT_DETECTED = 0,
    SIM_MODULE_NOT_REGISTERED,
    SIM_MODULE_REGISTERED,
} simModuleState_e;

typedef enum  {
    SIM_STATE_INIT = 0,
    SIM_STATE_INIT2,
    SIM_STATE_INIT_ENTER_PIN,
    SIM_STATE_SET_MODES,
    SIM_STATE_SEND_SMS,
    SIM_STATE_SEND_SMS_ENTER_MESSAGE
} simTelemetryState_e;

typedef enum  {
    SIM_AT_OK = 0,
    SIM_AT_ERROR,
    SIM_AT_WAITING_FOR_RESPONSE
} simATCommandState_e;

typedef enum  {
    SIM_READSTATE_RESPONSE = 0,
    SIM_READSTATE_SMS,
    SIM_READSTATE_SKIP
} simReadState_e;

typedef enum  {
    SIM_TX_NO = 0,
    SIM_TX_FS,
    SIM_TX
} simTransmissionState_e;

typedef enum {
    ACC_EVENT_NONE = 0,
    ACC_EVENT_HIGH,
    ACC_EVENT_LOW,
    ACC_EVENT_NEG_X
} accEvent_t;


static serialPort_t *simPort;
static serialPortConfig_t *portConfig;
static bool simEnabled = false;

static uint8_t atCommand[SIM_AT_COMMAND_MAX_SIZE];
static int simTelemetryState = SIM_STATE_INIT;
static timeMs_t sim_t_stateChange = 0;
static uint8_t simResponse[SIM_RESPONSE_BUFFER_SIZE + 1];
static int atCommandStatus = SIM_AT_OK;
static bool simWaitAfterResponse = false;
static uint8_t readState = SIM_READSTATE_RESPONSE;
static timeMs_t t_lastMessageSent = 0;
static uint8_t lastMessageTriggeredBy = 0;
static uint8_t simModuleState = SIM_MODULE_NOT_DETECTED;

static int simRssi;
static uint8_t accEvent = ACC_EVENT_NONE;
static char* accEventDescriptions[] = { "", "HIT! ", "DROP ", "HIT " };
static char* modeDescriptions[] = { "MAN", "ACR", "AIR", "ANG", "HOR", "ALH", "POS", "RTH", "WP", "CRS", "LAU", "FS" };
static const char gpsFixIndicators[] = { '!', '*', ' ' };


// XXX UNUSED
#if 0
static bool isGroundStationNumberDefined(void) {
    return telemetryConfig()->simGroundStationNumber[0] != '\0';
}
#endif

static bool checkGroundStationNumber(uint8_t* rv)
{
    int i;
    const char* gsn = telemetryConfig()->simGroundStationNumber;

    int digitsToCheck = strlen((char*)gsn);
    if (gsn[0] == '+') {
        digitsToCheck -= 5;        // ignore country code (max 4 digits)
    } else if (gsn[0] == '0') { // ignore trunk prefixes: '0', '8', 01', '02', '06'
        digitsToCheck--;
        if (gsn[1] == '1' || gsn[1] == '2' || gsn[1] == '6') {
            digitsToCheck--;
        }
    } else if (gsn[0] == '8') {
        digitsToCheck--;
    }

    for (i = 0; i < 16 && *gsn != '\0'; i++) gsn++;
    if (i == 0)
        return false;
    for (i = 0; i < 16 && *rv != '\"'; i++) rv++;

    gsn--; rv--;
    for (i = 0; i < digitsToCheck; i++) {
        if (*rv != *gsn) return false;
        gsn--; rv--;
    }
    return true;
}


static void readOriginatingNumber(uint8_t* rv)
{
    int i;
    char* gsn = telemetryConfigMutable()->simGroundStationNumber;
    if (gsn[0] != '\0')
        return;
    for (i = 0; i < 15 && rv[i] != '\"'; i++)
         gsn[i] = rv[i];
    gsn[i] = '\0';
}

static void readTransmitFlags(const uint8_t* fs)
{
    int i;

    uint8_t transmitFlags = 0;
    for (i = 0; i < SIM_N_TX_FLAGS && fs[i] != '\0'; i++) {
        switch (fs[i]) {
            case 'T': case 't':
            transmitFlags |= SIM_TX_FLAG;
            break;
            case 'F': case 'f':
            transmitFlags |= SIM_TX_FLAG_FAILSAFE;
            break;
            case 'G': case 'g':
            transmitFlags |= SIM_TX_FLAG_GPS;
            break;
            case 'L': case 'l':
            transmitFlags |= SIM_TX_FLAG_LOW_ALT;
            break;
            case 'A': case 'a':
            transmitFlags |= SIM_TX_FLAG_ACC;
            break;
        }
    }

    telemetryConfigMutable()->simTransmitFlags = transmitFlags;
}

static void requestSendSMS(uint8_t trigger)
{
    lastMessageTriggeredBy = trigger;
    if (simTelemetryState == SIM_STATE_SEND_SMS_ENTER_MESSAGE)
        return; // sending right now, don't reissue AT command
    simTelemetryState = SIM_STATE_SEND_SMS;
    if (atCommandStatus != SIM_AT_WAITING_FOR_RESPONSE)
        sim_t_stateChange = 0; // send immediately
}

static void readSMS(void)
{
    if (sl_strcasecmp((char*)simResponse, SIM_SMS_COMMAND_RTH) == 0) {
        if (getStateOfForcedRTH() == RTH_IDLE) {
            activateForcedRTH();
        } else {
            abortForcedRTH();
        }
    } else {
        readTransmitFlags(simResponse);
    }
    requestSendSMS(SIM_TX_FLAG_RESPONSE);
}

static void readSimResponse(void)
{
    if (readState == SIM_READSTATE_SKIP) {
        readState = SIM_READSTATE_RESPONSE;
        return;
    } else if (readState == SIM_READSTATE_SMS) {
        readSMS();
        readState = SIM_READSTATE_RESPONSE;
        return;
    }

    uint8_t* resp = simResponse;
    uint32_t responseCode = 0;
    if (simResponse[0] == '+') {
        resp++;
    }
    responseCode = *resp++;
    responseCode <<= 8; responseCode |= *resp++;
    responseCode <<= 8; responseCode |= *resp++;
    responseCode <<= 8; responseCode |= *resp++;

    if (responseCode == SIM_RESPONSE_CODE_OK) {
        // OK
        atCommandStatus = SIM_AT_OK;
        if (!simWaitAfterResponse) {
            sim_t_stateChange = millis() + SIM_AT_COMMAND_DELAY_MIN_MS;
        }
        return;
    } else if (responseCode == SIM_RESPONSE_CODE_ERROR) {
        // ERROR
        atCommandStatus = SIM_AT_ERROR;
        if (!simWaitAfterResponse) {
            sim_t_stateChange = millis() + SIM_AT_COMMAND_DELAY_MIN_MS;
        }
        return;
    } else if (responseCode == SIM_RESPONSE_CODE_RING) {
        // RING
    } else if (responseCode == SIM_RESPONSE_CODE_CSQ) {
        // +CSQ: 26,0
        simRssi = fastA2I((char*)&simResponse[6]);
    } else if (responseCode == SIM_RESPONSE_CODE_CLIP) {
        // we always get this after a RING when a call is incoming
        // +CLIP: "+3581234567"
        readOriginatingNumber(&simResponse[8]);
        if (checkGroundStationNumber(&simResponse[8])) {
            requestSendSMS(SIM_TX_FLAG_RESPONSE);
        }
    } else if (responseCode == SIM_RESPONSE_CODE_CREG) {
        // +CREG: 0,1
        if (simResponse[9] == '1' || simResponse[9] == '5') {
            simModuleState = SIM_MODULE_REGISTERED;
        } else {
            simModuleState = SIM_MODULE_NOT_REGISTERED;
        }        
    } else if (responseCode == SIM_RESPONSE_CODE_CMT) {
        // +CMT: <oa>,[<alpha>],<scts>[,<tooa>,<fo>,<pid>,<dcs>,<sca>,<tosca>,<length>]<CR><LF><data>
        // +CMT: "+3581234567","","19/02/12,14:57:24+08"
        readOriginatingNumber(&simResponse[7]);
        if (checkGroundStationNumber(&simResponse[7])) {
            readState = SIM_READSTATE_SMS; // next simResponse line will be SMS content
        } else {
            readState = SIM_READSTATE_SKIP; // skip SMS content
        }
    }
}

static int16_t getAltitudeMeters(void)
{
#if defined(USE_NAV)
    return getEstimatedActualPosition(Z) / 100;
#else
    return sensors(SENSOR_GPS) ? gpsSol.llh.alt / 100 : 0;
#endif
}

static void transmit(void)
{
    timeMs_t timeSinceMsg = millis() - t_lastMessageSent;
    uint8_t triggers = SIM_TX_FLAG;
    uint32_t accSq = sq(imuMeasuredAccelBF.x) + sq(imuMeasuredAccelBF.y) + sq(imuMeasuredAccelBF.z);

    if (telemetryConfig()->accEventThresholdHigh > 0 && accSq > sq(telemetryConfig()->accEventThresholdHigh)) {
        triggers |= SIM_TX_FLAG_ACC;
        accEvent = ACC_EVENT_HIGH;
    } else if (accSq < sq(telemetryConfig()->accEventThresholdLow)) {
        triggers |= SIM_TX_FLAG_ACC;
        accEvent = ACC_EVENT_LOW;
    } else if (telemetryConfig()->accEventThresholdNegX > 0 && imuMeasuredAccelBF.x < -telemetryConfig()->accEventThresholdNegX) {
        triggers |= SIM_TX_FLAG_ACC;
        accEvent = ACC_EVENT_NEG_X;
    }

    if ((lastMessageTriggeredBy & SIM_TX_FLAG_ACC) && timeSinceMsg < 2000)
        accEvent = ACC_EVENT_NONE;

    if (FLIGHT_MODE(FAILSAFE_MODE))
        triggers |= SIM_TX_FLAG_FAILSAFE;
    if (!navigationPositionEstimateIsHealthy())
        triggers |= SIM_TX_FLAG_GPS;
    if (gpsSol.fixType != GPS_NO_FIX && FLIGHT_MODE(SIM_LOW_ALT_WARNING_MODES) && getAltitudeMeters() < telemetryConfig()->simLowAltitude)
        triggers |= SIM_TX_FLAG_LOW_ALT;

    triggers &= telemetryConfig()->simTransmitFlags;

    if (!triggers)
        return;
    if (!ARMING_FLAG(WAS_EVER_ARMED))
        return;

    if ((triggers & ~lastMessageTriggeredBy) // if new trigger activated after last msg, don't wait
        || timeSinceMsg > 1000 * MAX(SIM_MIN_TRANSMIT_INTERVAL, telemetryConfig()->simTransmitInterval)) {
        requestSendSMS(triggers);
    }
}

static void sendATCommand(const char* command)
{
    atCommandStatus = SIM_AT_WAITING_FOR_RESPONSE;
    uint8_t len = MIN((uint8_t)strlen(command), SIM_AT_COMMAND_MAX_SIZE);
    serialWriteBuf(simPort, (const uint8_t*) command, len);
}

static void sendSMS(void)
{
    char pluscode_url[20];
    int16_t groundSpeed = 0;
    uint16_t vbat = getBatteryVoltage();
    int16_t amps = isAmperageConfigured() ? getAmperage() / 10 : 0; // 1 = 100 milliamps
    uint16_t avgSpeed = lrintf(10 * calculateAverageSpeed());
    uint32_t now = millis();

    memset(pluscode_url, 0, sizeof(pluscode_url));

    if (sensors(SENSOR_GPS) && STATE(GPS_FIX)) {
        groundSpeed = gpsSol.groundSpeed / 100;

        char buf[20];
        olc_encode(gpsSol.llh.lat, gpsSol.llh.lon, 11, buf, sizeof(buf));

        // URLencode plus code (replace plus sign with %2B)
        for (char *in = buf, *out = pluscode_url; *in; ) {
            if (*in == '+') {
                in++;
                *out++ = '%';
                *out++ = '2';
                *out++ = 'B';
            }
            else {
                *out++ = *in++;
            }
        }
    }

    // \x1a sends msg, \x1b cancels
    uint8_t len = tfp_sprintf((char*)atCommand, "%s%d.%02dV %d.%dA ALT:%d SPD:%d/%d.%d DIS:%lu/%lu HDG:%d SAT:%d%c SIG:%d %s https://maps.google.com/?q=%s\x1a",
        accEventDescriptions[accEvent],
        vbat / 100, vbat % 100,
        amps / 10, amps % 10,
        getAltitudeMeters(),
        groundSpeed, avgSpeed / 10, avgSpeed % 10,
        GPS_distanceToHome, getTotalTravelDistance() / 100,
        DECIDEGREES_TO_DEGREES(attitude.values.yaw),
        gpsSol.numSat, gpsFixIndicators[gpsSol.fixType],
        simRssi,
        getStateOfForcedRTH() == RTH_IDLE ? modeDescriptions[getFlightModeForTelemetry()] : "RTH",
        pluscode_url);

    serialWriteBuf(simPort, atCommand, len);
    t_lastMessageSent = now;
    accEvent = ACC_EVENT_NONE;
    atCommandStatus = SIM_AT_WAITING_FOR_RESPONSE;
}

void handleSimTelemetry()
{
    static uint16_t simResponseIndex = 0;
    uint32_t now = millis();

    if (!simEnabled)
        return;
    if (!simPort)
        return;

    while (serialRxBytesWaiting(simPort) > 0) {
        uint8_t c = serialRead(simPort);
        if (c == '\n' || simResponseIndex == SIM_RESPONSE_BUFFER_SIZE) {
            simResponse[simResponseIndex] = '\0';
            if (simResponseIndex > 0) simResponseIndex--;
            if (simResponse[simResponseIndex] == '\r') simResponse[simResponseIndex] = '\0';
            simResponseIndex = 0; //data ok
            readSimResponse();
            break;
        } else {
            simResponse[simResponseIndex] = c;
            simResponseIndex++;
        }
    }

    transmit();

    if (now < sim_t_stateChange)
        return;

    sim_t_stateChange = now + SIM_AT_COMMAND_DELAY_MS;  // by default, if OK or ERROR not received, wait this long
    simWaitAfterResponse = false;                       // by default, if OK or ERROR received, go to next state immediately.
    switch (simTelemetryState) {
        case SIM_STATE_INIT:
        sendATCommand("AT\r");
        simTelemetryState = SIM_STATE_INIT2;
        break;
        case SIM_STATE_INIT2:
        sendATCommand("ATE0\r");
        simTelemetryState = SIM_STATE_INIT_ENTER_PIN;
        break;
        case SIM_STATE_INIT_ENTER_PIN:
        sendATCommand("AT+CPIN=");
        sendATCommand((char*)telemetryConfig()->simPin);        
        sendATCommand("\r");
        simTelemetryState = SIM_STATE_SET_MODES;
        break;
        case SIM_STATE_SET_MODES:
        sendATCommand("AT+CMGF=1;+CNMI=3,2;+CLIP=1;+CSQ\r");
        simTelemetryState = SIM_STATE_INIT;
        sim_t_stateChange = now + SIM_CYCLE_MS;
        simWaitAfterResponse = true;
        break;
        case SIM_STATE_SEND_SMS:
        sendATCommand("AT+CMGS=\"");
        sendATCommand((char*)telemetryConfig()->simGroundStationNumber);
        sendATCommand("\"\r");
        simTelemetryState = SIM_STATE_SEND_SMS_ENTER_MESSAGE;
        sim_t_stateChange = now + 100;
        break;
        case SIM_STATE_SEND_SMS_ENTER_MESSAGE:
        sendSMS();
        simTelemetryState = SIM_STATE_INIT;
        sim_t_stateChange = now + SIM_CYCLE_MS;
        simWaitAfterResponse = true;
        break;
    }
}

// XXX UNUSED
#if 0
static void freeSimTelemetryPort(void)
{
    closeSerialPort(simPort);
    simPort = NULL;
    simEnabled = false;
}
#endif

void initSimTelemetry(void)
{
    portConfig = findSerialPortConfig(FUNCTION_TELEMETRY_SIM);
}

static void configureSimTelemetryPort(void)
{
    if (!portConfig) {
        return;
    }
    baudRate_e baudRateIndex = portConfig->telemetry_baudrateIndex;
    simPort = openSerialPort(portConfig->identifier, FUNCTION_TELEMETRY_SIM, NULL, NULL,
        baudRates[baudRateIndex], MODE_RXTX, SERIAL_NOT_INVERTED);

    if (!simPort) {
        return;
    }

    sim_t_stateChange = millis() + SIM_STARTUP_DELAY_MS;
    simTelemetryState = SIM_STATE_INIT;
    readState = SIM_READSTATE_RESPONSE;
    simEnabled = true;
}

void checkSimTelemetryState(void)
{
    if (simEnabled) {
        return;
    }
    configureSimTelemetryPort();
}

#endif
