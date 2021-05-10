/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/* Created by jflyper */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>
#include <math.h>

#include "platform.h"

#if defined(USE_VTX_SMARTAUDIO) && defined(USE_VTX_CONTROL)

#include "build/debug.h"

#include "cms/cms.h"

#include "common/log.h"
#include "common/maths.h"
#include "common/printf.h"
#include "common/utils.h"
#include "common/typeconversion.h"

#include "drivers/time.h"
#include "drivers/vtx_common.h"

#include "fc/settings.h"

#include "io/serial.h"
#include "io/vtx.h"
#include "io/vtx_control.h"
#include "io/vtx_smartaudio.h"
#include "io/vtx_string.h"


// Timing parameters
// Note that vtxSAProcess() is normally called at 200ms interval
#define SMARTAUDIO_CMD_TIMEOUT       120    // Time until the command is considered lost
#define SMARTAUDIO_POLLING_INTERVAL  150    // Minimum time between state polling
#define SMARTAUDIO_POLLING_WINDOW   1000    // Time window after command polling for state change

static serialPort_t *smartAudioSerialPort = NULL;

uint8_t saPowerCount = VTX_SMARTAUDIO_DEFAULT_POWER_COUNT;
const char * saPowerNames[VTX_SMARTAUDIO_MAX_POWER_COUNT + 1] = {
    "----", "25  ", "200 ", "500 ", "800 ", "    "
};

// Save powerlevels reported from SA 2.1 devices here
char sa21PowerNames[VTX_SMARTAUDIO_MAX_POWER_COUNT][5];

static const vtxVTable_t saVTable;    // Forward
static vtxDevice_t vtxSmartAudio = {
    .vTable = &saVTable,
    .capability.bandCount = VTX_SMARTAUDIO_BAND_COUNT,
    .capability.channelCount = VTX_SMARTAUDIO_CHANNEL_COUNT,
    .capability.powerCount = VTX_SMARTAUDIO_MAX_POWER_COUNT,
    .capability.bandNames = (char **)vtx58BandNames,
    .capability.channelNames = (char **)vtx58ChannelNames,
    .capability.powerNames = (char**)saPowerNames
};

// SmartAudio command and response codes
enum {
    SA_CMD_NONE = 0x00,
    SA_CMD_GET_SETTINGS = 0x01,
    SA_CMD_SET_POWER,
    SA_CMD_SET_CHAN,
    SA_CMD_SET_FREQ,
    SA_CMD_SET_MODE,
    SA_CMD_GET_SETTINGS_V2 = 0x09,       // Response only
    SA_CMD_GET_SETTINGS_V21 = 0x11,
} smartAudioCommand_e;

// This is not a good design; can't distinguish command from response this way.
#define SACMD(cmd) (((cmd) << 1) | 1)


#define SA_IS_PITMODE(n) ((n) & SA_MODE_GET_PITMODE)
#define SA_IS_PIRMODE(n) (((n) & SA_MODE_GET_PITMODE) && ((n) & SA_MODE_GET_IN_RANGE_PITMODE))
#define SA_IS_PORMODE(n) (((n) & SA_MODE_GET_PITMODE) && ((n) & SA_MODE_GET_OUT_RANGE_PITMODE))


// convert between 'saDevice.channel' and band/channel values
#define SA_DEVICE_CHVAL_TO_BAND(val) ((val) / (uint8_t)8)
#define SA_DEVICE_CHVAL_TO_CHANNEL(val) ((val) % (uint8_t)8)
#define SA_BANDCHAN_TO_DEVICE_CHVAL(band, channel) ((band) * (uint8_t)8 + (channel))


// Statistical counters, for user side trouble shooting.

smartAudioStat_t saStat = {
    .pktsent = 0,
    .pktrcvd = 0,
    .badpre = 0,
    .badlen = 0,
    .crc = 0,
    .ooopresp = 0,
    .badcode = 0,
};

// Fill table with standard values for SA 1.0 and 2.0
saPowerTable_t saPowerTable[VTX_SMARTAUDIO_MAX_POWER_COUNT] = {
    {  25,   7 },
    { 200,  16 },
    { 500,  25 },
    { 800,  40 },
    {   0,   0 } // Placeholder
};

// Last received device ('hard') states

smartAudioDevice_t saDevice = {
    .version = SA_UNKNOWN,
    .channel = SETTING_VTX_CHANNEL_DEFAULT,
    .power = SETTING_VTX_POWER_DEFAULT,
    .mode = 0,
    .freq = 0,
    .orfreq = 0,
    .willBootIntoPitMode = false
};

static smartAudioDevice_t saDevicePrev = {
    .version = 0,
};

// XXX Possible compliance problem here. Need LOCK/UNLOCK menu?
static uint8_t saLockMode = SA_MODE_SET_UNLOCK; // saCms variable?

// XXX Should be configurable by user?
bool saDeferred = true; // saCms variable?

// Receive frame reassembly buffer
#define SA_MAX_RCVLEN 21
static uint8_t sa_rbuf[SA_MAX_RCVLEN+4]; // XXX delete 4 byte guard

//
// CRC8 computations
//

#define POLYGEN 0xd5

static uint8_t CRC8(const uint8_t *data, const int8_t len)
{
    uint8_t crc = 0; /* start with 0 so first byte can be 'xored' in */
    uint8_t currByte;

    for (int i = 0 ; i < len ; i++) {
        currByte = data[i];

        crc ^= currByte; /* XOR-in the next input byte */

        for (int i = 0; i < 8; i++) {
            if ((crc & 0x80) != 0) {
                crc = (uint8_t)((crc << 1) ^ POLYGEN);
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}


static void saPrintSettings(void)
{
    LOG_D(VTX, "Current status: version: %d", saDevice.version);
    LOG_D(VTX, "  mode(0x%x): fmode=%s", saDevice.mode,  (saDevice.mode & 1) ? "freq" : "chan");
    LOG_D(VTX, " pit=%s ", (saDevice.mode & 2) ? "on " : "off");
    LOG_D(VTX, " inb=%s", (saDevice.mode & 4) ? "on " : "off");
    LOG_D(VTX, " outb=%s", (saDevice.mode & 8) ? "on " : "off");
    LOG_D(VTX, " lock=%s", (saDevice.mode & 16) ? "unlocked" : "locked");
    LOG_D(VTX, " deferred=%s", (saDevice.mode & 32) ? "on" : "off");
    LOG_D(VTX, "  channel: %d ", saDevice.channel);
    LOG_D(VTX, "freq: %d ", saDevice.freq);
    LOG_D(VTX, "power: %d ", saDevice.power);
    LOG_D(VTX, "pitfreq: %d ", saDevice.orfreq);
    LOG_D(VTX, "BootIntoPitMode: %s", saDevice.willBootIntoPitMode ? "yes" : "no");
}

int saDacToPowerIndex(int dac)
{
    for (int idx = saPowerCount - 1 ; idx >= 0 ; idx--) {
        if (saPowerTable[idx].dbi <= dac) {
            return idx;
        }
    }
    return 0;
}

int saDbiToMw(uint16_t dbi) {

    uint16_t mw = (uint16_t)pow(10.0, dbi / 10.0);

    if (dbi > 14) {
        // For powers greater than 25mW round up to a multiple of 50 to match expectations
        mw = 50 * ((mw + 25) / 50);
     }

    return mw;
}

//
// Autobauding
//

#define SMARTBAUD_MIN 4800
#define SMARTBAUD_MAX 4950
uint16_t sa_smartbaud = SMARTBAUD_MIN;
static int sa_adjdir = 1; // -1=going down, 1=going up
static int sa_baudstep = 50;

static void saAutobaud(void)
{
    if (saStat.pktsent < 10) {
        // Not enough samples collected
        return;
    }

#if 0
    LOG_D(VTX, "autobaud: %d rcvd %d/%d (%d)",
        sa_smartbaud, saStat.pktrcvd, saStat.pktsent, ((saStat.pktrcvd * 100) / saStat.pktsent)));
#endif

    if (((saStat.pktrcvd * 100) / saStat.pktsent) >= 70) {
        // This is okay
        saStat.pktsent = 0; // Should be more moderate?
        saStat.pktrcvd = 0;
        return;
    }

    LOG_D(VTX, "autobaud: adjusting");

    if ((sa_adjdir == 1) && (sa_smartbaud == SMARTBAUD_MAX)) {
       sa_adjdir = -1;
       LOG_D(VTX, "autobaud: now going down");
    } else if ((sa_adjdir == -1 && sa_smartbaud == SMARTBAUD_MIN)) {
       sa_adjdir = 1;
       LOG_D(VTX, "autobaud: now going up");
    }

    sa_smartbaud += sa_baudstep * sa_adjdir;

    LOG_D(VTX, "autobaud: %d", sa_smartbaud);

    smartAudioSerialPort->vTable->serialSetBaudRate(smartAudioSerialPort, sa_smartbaud);

    saStat.pktsent = 0;
    saStat.pktrcvd = 0;
}

// Transport level variables

static timeUs_t sa_lastTransmissionMs = 0;
static uint8_t sa_outstanding = SA_CMD_NONE; // Outstanding command
static uint8_t sa_osbuf[32]; // Outstanding comamnd frame for retransmission
static int sa_oslen;         // And associate length

static void saProcessResponse(uint8_t *buf, int len)
{
    uint8_t resp = buf[0];

    if (resp == sa_outstanding) {
        sa_outstanding = SA_CMD_NONE;
    } else if ((resp == SA_CMD_GET_SETTINGS_V2 || resp == SA_CMD_GET_SETTINGS_V21) && (sa_outstanding == SA_CMD_GET_SETTINGS)) {
        sa_outstanding = SA_CMD_NONE;
    } else {
        saStat.ooopresp++;
        LOG_D(VTX, "processResponse: outstanding %d got %d", sa_outstanding, resp);
    }

    switch (resp) {
    case SA_CMD_GET_SETTINGS_V21: // Version 2.1 Get Settings
    case SA_CMD_GET_SETTINGS_V2: // Version 2 Get Settings
    case SA_CMD_GET_SETTINGS:    // Version 1 Get Settings
        if (len < 7) {
            break;
        }

        // From spec: "Bit 7-3 is holding the Smart audio version where 0 is V1, 1 is V2, 2 is V2.1"
        // saDevice.version = 0 means unknown, 1 means Smart audio V1, 2 means Smart audio V2 and 3 means Smart audio V2.1
        saDevice.version = (buf[0] == SA_CMD_GET_SETTINGS) ? 1 : ((buf[0] == SA_CMD_GET_SETTINGS_V2) ? 2 : 3);
        saDevice.channel = buf[2];
        uint8_t rawPowerValue = buf[3];
        saDevice.mode = buf[4];
        saDevice.freq = (buf[5] << 8) | buf[6];

        // read pir and por flags to detect if the device will boot into pitmode.
        // note that "quit pitmode without unsetting the pitmode flag" clears pir and por flags but the device will still boot into pitmode.
        // therefore we ignore the pir and por flags while the device is not in pitmode
        // actually, this is the whole reason the variable saDevice.willBootIntoPitMode exists.
        // otherwise we could use saDevice.mode directly
        if (saDevice.mode & SA_MODE_GET_PITMODE) {
            bool newBootMode = (saDevice.mode & SA_MODE_GET_IN_RANGE_PITMODE) || (saDevice.mode & SA_MODE_GET_OUT_RANGE_PITMODE);
            if (newBootMode != saDevice.willBootIntoPitMode) {
                LOG_D(VTX, "saProcessResponse: willBootIntoPitMode is now %s\r\n", newBootMode  ? "true" : "false");
            }
            saDevice.willBootIntoPitMode = newBootMode;
        }

        if(saDevice.version == SA_2_1) {
            //read dbm based power levels
            if(len < 10) { //current power level in dbm field missing or power level length field missing or zero power levels reported
                LOG_D(VTX, "processResponse: V2.1 vtx didn't report any power levels\r\n");
                break;
            }
            saPowerCount = constrain((int8_t)buf[8], 0, VTX_SMARTAUDIO_MAX_POWER_COUNT);
            vtxSmartAudio.capability.powerCount = saPowerCount;
            //SmartAudio seems to report buf[8] + 1 power levels, but one of them is zero.
            //zero is indeed a valid power level to set the vtx to, but it activates pit mode.
            //crucially, after sending 0 dbm, the vtx does NOT report its power level to be 0 dbm.
            //instead, it reports whatever value was set previously and it reports to be in pit mode.
            //for this reason, zero shouldn't be used as a normal power level in iNav.
            for (int8_t i = 0; i < saPowerCount; i++ ) {
                saPowerTable[i].dbi = buf[9 + i + 1]; //+ 1 to skip the first power level, as mentioned above
                saPowerTable[i].mW = saDbiToMw(saPowerTable[i].dbi);
                if (i <= VTX_SMARTAUDIO_MAX_POWER_COUNT) {
                    char strbuf[5];
                    itoa(saPowerTable[i].mW, strbuf, 10);
                    strcpy(sa21PowerNames[i], strbuf);
                    saPowerNames[i + 1] = sa21PowerNames[i];
                }
            }

            LOG_D(VTX, "processResponse: %d power values: %d, %d, %d, %d\r\n",
                    saPowerCount, saPowerTable[0].dbi, saPowerTable[1].dbi,
                    saPowerTable[2].dbi, saPowerTable[3].dbi);
            //LOG_D(VTX, "processResponse: V2.1 received vtx power value %d\r\n",buf[7]);
            rawPowerValue = buf[7];

            saDevice.power = 0; //set to unknown power level if the reported one doesnt match any of the known ones
            LOG_D(VTX, "processResponse: rawPowerValue is %d, legacy power is %d\r\n", rawPowerValue, buf[3]);
            for (int8_t i = 0; i < saPowerCount; i++) {
                if (rawPowerValue == saPowerTable[i].dbi) {
                    saDevice.power = i + 1;
                }
            }
        } else {
            saDevice.power = rawPowerValue + 1;
        }

        DEBUG_SET(DEBUG_SMARTAUDIO, 0, saDevice.version * 100 + saDevice.mode);
        DEBUG_SET(DEBUG_SMARTAUDIO, 1, saDevice.channel);
        DEBUG_SET(DEBUG_SMARTAUDIO, 2, saDevice.freq);
        DEBUG_SET(DEBUG_SMARTAUDIO, 3, saDevice.power);
        break;

    case SA_CMD_SET_POWER: // Set Power
        break;

    case SA_CMD_SET_CHAN: // Set Channel
        break;

    case SA_CMD_SET_FREQ: // Set Frequency
        if (len < 5) {
            break;
        }

        const uint16_t freq = (buf[2] << 8)|buf[3];

        if (freq & SA_FREQ_GETPIT) {
            saDevice.orfreq = freq & SA_FREQ_MASK;
            LOG_D(VTX, "saProcessResponse: GETPIT freq %d", saDevice.orfreq);
        } else if (freq & SA_FREQ_SETPIT) {
            saDevice.orfreq = freq & SA_FREQ_MASK;
            LOG_D(VTX, "saProcessResponse: SETPIT freq %d", saDevice.orfreq);
        } else {
            saDevice.freq = freq;
            LOG_D(VTX, "saProcessResponse: SETFREQ freq %d", freq);
        }
        break;

    case SA_CMD_SET_MODE: // Set Mode
        LOG_D(VTX, "saProcessResponse: SET_MODE 0x%x, (pir %s, por %s, pitdsbl %s, %s)\r\n",
            buf[2], (buf[2] & 1) ? "on" : "off", (buf[2] & 2) ? "on" : "off", (buf[3] & 4) ? "on" : "off",
            (buf[4] & 8) ? "unlocked" : "locked");
        break;

    default:
        saStat.badcode++;
        return;
    }

    if (memcmp(&saDevice, &saDevicePrev, sizeof(smartAudioDevice_t))) {
        // Debug
        saPrintSettings();
    }

    saDevicePrev = saDevice;
}

//
// Datalink
//

static void saReceiveFramer(uint8_t c)
{

    static enum saFramerState_e {
        S_WAITPRE1, // Waiting for preamble 1 (0xAA)
        S_WAITPRE2, // Waiting for preamble 2 (0x55)
        S_WAITRESP, // Waiting for response code
        S_WAITLEN,  // Waiting for length
        S_DATA,     // Receiving data
        S_WAITCRC,  // Waiting for CRC
    } state = S_WAITPRE1;

    static int len;
    static int dlen;

    switch (state) {
    case S_WAITPRE1:
        if (c == 0xAA) {
            state = S_WAITPRE2;
        } else {
            state = S_WAITPRE1; // Don't need this (no change)
        }
        break;

    case S_WAITPRE2:
        if (c == 0x55) {
            state = S_WAITRESP;
        } else {
            saStat.badpre++;
            state = S_WAITPRE1;
        }
        break;

    case S_WAITRESP:
        sa_rbuf[0] = c;
        state = S_WAITLEN;
        break;

    case S_WAITLEN:
        sa_rbuf[1] = c;
        len = c;

        if (len > SA_MAX_RCVLEN - 2) {
            saStat.badlen++;
            state = S_WAITPRE1;
        } else if (len == 0) {
            state = S_WAITCRC;
        } else {
            dlen = 0;
            state = S_DATA;
        }
        break;

    case S_DATA:
        // XXX Should check buffer overflow (-> saerr_overflow)
        sa_rbuf[2 + dlen] = c;
        if (++dlen == len) {
            state = S_WAITCRC;
        }
        break;

    case S_WAITCRC:
        if (CRC8(sa_rbuf, 2 + len) == c) {
            // Got a response
            saProcessResponse(sa_rbuf, len + 2);
            saStat.pktrcvd++;
        } else if (sa_rbuf[0] & 1) {
            // Command echo
            // XXX There is an exceptional case (V2 response)
            // XXX Should check crc in the command format?
        } else {
            saStat.crc++;
        }
        state = S_WAITPRE1;
        break;
    }
}

static void saSendFrame(uint8_t *buf, int len)
{
    switch (smartAudioSerialPort->identifier) {
        case SERIAL_PORT_SOFTSERIAL1:
        case SERIAL_PORT_SOFTSERIAL2:
            break;
        default:
            serialWrite(smartAudioSerialPort, 0x00); // Generate 1st start bit
            break;
    }

    for (int i = 0 ; i < len ; i++) {
        serialWrite(smartAudioSerialPort, buf[i]);
    }

    // XXX: Workaround for early AKK SAudio-enabled VTX bug,
    // shouldn't cause any problems with VTX with properly
    // implemented SAudio.
	//Update: causes problem with new AKK AIO camera connected to SoftUART
    if (vtxConfig()->smartAudioEarlyAkkWorkaroundEnable) serialWrite(smartAudioSerialPort, 0x00);

    sa_lastTransmissionMs = millis();
    saStat.pktsent++;
}

/*
 * Retransmission and command queuing
 *
 *   The transport level support includes retransmission on response timeout
 * and command queueing.
 *
 * Resend buffer:
 *   The smartaudio returns response for valid command frames in no less
 * than 60msec, which we can't wait. So there's a need for a resend buffer.
 *
 * Command queueing:
 *   The driver autonomously sends GetSettings command for auto-bauding,
 * asynchronous to user initiated commands; commands issued while another
 * command is outstanding must be queued for later processing.
 *   The queueing also handles the case in which multiple commands are
 * required to implement a user level command.
 */

// Retransmission

static void saResendCmd(void)
{
    saSendFrame(sa_osbuf, sa_oslen);
}

static void saSendCmd(uint8_t *buf, int len)
{
    for (int i = 0 ; i < len ; i++) {
        sa_osbuf[i] = buf[i];
    }

    sa_oslen = len;
    sa_outstanding = (buf[2] >> 1);

    saSendFrame(sa_osbuf, sa_oslen);
}

// Command queue management

typedef struct saCmdQueue_s {
    uint8_t *buf;
    int len;
} saCmdQueue_t;

#define SA_QSIZE 6     // 1 heartbeat (GetSettings) + 2 commands + 1 slack
static saCmdQueue_t sa_queue[SA_QSIZE];
static uint8_t sa_qhead = 0;
static uint8_t sa_qtail = 0;

static bool saQueueEmpty(void)
{
    return sa_qhead == sa_qtail;
}

static bool saQueueFull(void)
{
    return ((sa_qhead + 1) % SA_QSIZE) == sa_qtail;
}

static void saQueueCmd(uint8_t *buf, int len)
{
    if (saQueueFull()) {
         return;
    }

    sa_queue[sa_qhead].buf = buf;
    sa_queue[sa_qhead].len = len;
    sa_qhead = (sa_qhead + 1) % SA_QSIZE;
}

static void saSendQueue(void)
{
    if (saQueueEmpty()) {
         return;
    }

    saSendCmd(sa_queue[sa_qtail].buf, sa_queue[sa_qtail].len);
    sa_qtail = (sa_qtail + 1) % SA_QSIZE;
}

// Individual commands

static void saGetSettings(void)
{
    static uint8_t bufGetSettings[5] = {0xAA, 0x55, SACMD(SA_CMD_GET_SETTINGS), 0x00, 0x9F};

    LOG_D(VTX, "smartAudioGetSettings\r\n");
    saQueueCmd(bufGetSettings, 5);
}

void saSetFreq(uint16_t freq)
{
    static uint8_t buf[7] = { 0xAA, 0x55, SACMD(SA_CMD_SET_FREQ), 2 };
    static uint8_t switchBuf[7];

    if (freq & SA_FREQ_GETPIT) {
        LOG_D(VTX, "smartAudioSetFreq: GETPIT");
    } else if (freq & SA_FREQ_SETPIT) {
        LOG_D(VTX, "smartAudioSetFreq: SETPIT %d", freq & SA_FREQ_MASK);
    } else {
        LOG_D(VTX, "smartAudioSetFreq: SET %d", freq);
    }

    buf[4] = (freq >> 8) & 0xff;
    buf[5] = freq & 0xff;
    buf[6] = CRC8(buf, 6);

    // Need to work around apparent SmartAudio bug when going from 'channel'
    // to 'user-freq' mode, where the set-freq command will fail if the freq
    // value is unchanged from the previous 'user-freq' mode
    if ((saDevice.mode & SA_MODE_GET_FREQ_BY_FREQ) == 0 && freq == saDevice.freq) {
        memcpy(&switchBuf, &buf, sizeof(buf));
        const uint16_t switchFreq = freq + ((freq == VTX_SMARTAUDIO_MAX_FREQUENCY_MHZ) ? -1 : 1);
        switchBuf[4] = (switchFreq >> 8);
        switchBuf[5] = switchFreq & 0xff;
        switchBuf[6] = CRC8(switchBuf, 6);

        saQueueCmd(switchBuf, 7);
    }

    saQueueCmd(buf, 7);
}

void saSetPitFreq(uint16_t freq)
{
    saSetFreq(freq | SA_FREQ_SETPIT);
}

#if 0
static void saGetPitFreq(void)
{
    saDoDevSetFreq(SA_FREQ_GETPIT);
}
#endif

static bool saValidateBandAndChannel(uint8_t band, uint8_t channel)
{
    return (band >= VTX_SMARTAUDIO_MIN_BAND && band <= VTX_SMARTAUDIO_MAX_BAND &&
            channel >= VTX_SMARTAUDIO_MIN_CHANNEL && channel <= VTX_SMARTAUDIO_MAX_CHANNEL);
}

void saSetBandAndChannel(uint8_t band, uint8_t channel)
{
    static uint8_t buf[6] = { 0xAA, 0x55, SACMD(SA_CMD_SET_CHAN), 1 };

    buf[4] = SA_BANDCHAN_TO_DEVICE_CHVAL(band, channel);
    buf[5] = CRC8(buf, 5);
    LOG_D(VTX, "vtxSASetBandAndChannel set index band %d channel %d value sent 0x%x\r\n", band, channel, buf[4]);

    //this will clear saDevice.mode & SA_MODE_GET_FREQ_BY_FREQ
    saQueueCmd(buf, 6);
}

void saSetMode(int mode)
{
    static uint8_t buf[6] = { 0xAA, 0x55, SACMD(SA_CMD_SET_MODE), 1 };

    buf[4] = (mode & 0x3f) | saLockMode;
    if (saDevice.version >= SA_2_1 && (mode & SA_MODE_CLR_PITMODE) &&
        ((mode & SA_MODE_SET_IN_RANGE_PITMODE) || (mode & SA_MODE_SET_OUT_RANGE_PITMODE))) {
        saDevice.willBootIntoPitMode = true;//quit pitmode without unsetting flag.
        //the response will just say pit=off but the device will still go into pitmode on reboot.
        //therefore we have to memorize this change here.
    }
    LOG_D(VTX, "saSetMode(0x%x): pir=%s por=%s pitdsbl=%s %s\r\n", mode, (mode & 1) ? "on " : "off", (mode & 2) ? "on " : "off",
            (mode & 4)? "on " : "off", (mode & 8) ? "locked" : "unlocked");

    buf[5] = CRC8(buf, 5);
    saQueueCmd(buf, 6);
}

bool vtxSmartAudioInit(void)
{
    serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_VTX_SMARTAUDIO);
    if (portConfig) {
        portOptions_t portOptions = SERIAL_BIDIR_NOPULL;
        portOptions = portOptions | (vtxConfig()->halfDuplex ? SERIAL_BIDIR | SERIAL_BIDIR_PP : SERIAL_UNIDIR);
        smartAudioSerialPort = openSerialPort(portConfig->identifier, FUNCTION_VTX_SMARTAUDIO, NULL, NULL, 4800, MODE_RXTX, portOptions);
    }

    if (!smartAudioSerialPort) {
        return false;
    }

    vtxCommonSetDevice(&vtxSmartAudio);

    return true;
}

#define SA_INITPHASE_START         0
#define SA_INITPHASE_WAIT_SETTINGS 1 // SA_CMD_GET_SETTINGS was sent and waiting for reply.
#define SA_INITPHASE_WAIT_PITFREQ  2 // SA_FREQ_GETPIT sent and waiting for reply.
#define SA_INITPHASE_DONE          3

static void vtxSAProcess(vtxDevice_t *vtxDevice, timeUs_t currentTimeUs)
{
    UNUSED(vtxDevice);
    UNUSED(currentTimeUs);

    static char initPhase = SA_INITPHASE_START;

    if (smartAudioSerialPort == NULL) {
        return;
    }

    while (serialRxBytesWaiting(smartAudioSerialPort) > 0) {
        uint8_t c = serialRead(smartAudioSerialPort);
        saReceiveFramer((uint16_t)c);
    }

    // Re-evaluate baudrate after each frame reception
    saAutobaud();

    switch (initPhase) {
    case SA_INITPHASE_START:
        saGetSettings();
        //saSendQueue();
        initPhase = SA_INITPHASE_WAIT_SETTINGS;
        break;

    case SA_INITPHASE_WAIT_SETTINGS:
        // Don't send SA_FREQ_GETPIT to V1 device; it act as plain SA_CMD_SET_FREQ,
        // and put the device into user frequency mode with uninitialized freq.
        if (saDevice.version) {
            if (saDevice.version == SA_2_0) {
                saSetFreq(SA_FREQ_GETPIT);
                initPhase = SA_INITPHASE_WAIT_PITFREQ;
            } else {
                initPhase = SA_INITPHASE_DONE;
            }
            if (saDevice.version >= SA_2_0 ) {
                //did the device boot up in pit mode on its own?
                saDevice.willBootIntoPitMode = (saDevice.mode & SA_MODE_GET_PITMODE) ? true : false;
                LOG_D(VTX, "sainit: willBootIntoPitMode is %s\r\n", saDevice.willBootIntoPitMode ? "true" : "false");
            }
        }
        break;

    case SA_INITPHASE_WAIT_PITFREQ:
        if (saDevice.orfreq) {
            initPhase = SA_INITPHASE_DONE;
        }
        break;

    case SA_INITPHASE_DONE:
        break;
    }

    // Command queue control

    timeMs_t nowMs = millis();             // Don't substitute with "currentTimeUs / 1000"; sa_lastTransmissionMs is based on millis().
    static timeMs_t lastCommandSentMs = 0; // Last non-GET_SETTINGS sent

    if ((sa_outstanding != SA_CMD_NONE) && (nowMs - sa_lastTransmissionMs > SMARTAUDIO_CMD_TIMEOUT)) {
        // Last command timed out
        // LOG_D(VTX, "process: resending 0x%x", sa_outstanding);
        // XXX Todo: Resend termination and possible offline transition
        saResendCmd();
    lastCommandSentMs = nowMs;
    } else if (!saQueueEmpty()) {
        // Command pending. Send it.
        // LOG_D(VTX, "process: sending queue");
        saSendQueue();
        lastCommandSentMs = nowMs;
    } else if ((nowMs - lastCommandSentMs < SMARTAUDIO_POLLING_WINDOW) && (nowMs - sa_lastTransmissionMs >= SMARTAUDIO_POLLING_INTERVAL)) {
    //LOG_D(VTX, "process: sending status change polling");
    saGetSettings();
    saSendQueue();
    }
}

// Interface to common VTX API

vtxDevType_e vtxSAGetDeviceType(const vtxDevice_t *vtxDevice)
{
    UNUSED(vtxDevice);
    return VTXDEV_SMARTAUDIO;
}

static bool vtxSAIsReady(const vtxDevice_t *vtxDevice)
{
    return vtxDevice != NULL && !(saDevice.power == 0);
    //wait until power reading exists
}

void vtxSASetBandAndChannel(vtxDevice_t *vtxDevice, uint8_t band, uint8_t channel)
{
    UNUSED(vtxDevice);
    if (saValidateBandAndChannel(band, channel)) {
        saSetBandAndChannel(band - 1, channel - 1);
    }
}
 static void vtxSASetPowerByIndex(vtxDevice_t *vtxDevice, uint8_t index)
{
    static uint8_t buf[6] = { 0xAA, 0x55, SACMD(SA_CMD_SET_POWER), 1 };

    if (!vtxSAIsReady(vtxDevice)) {
        return;
    }

    if (index == 0) {
        LOG_D(VTX, "SmartAudio doesn't support power off");
        return;
    }

    if (index > saPowerCount) {
        LOG_D(VTX, "Invalid power level");
        return;
    }

    LOG_D(VTX, "saSetPowerByIndex: index %d, value %d\r\n", index, buf[4]);

    index--;
    switch (saDevice.version) {
        case SA_1_0:
            buf[4] = saPowerTable[index].dbi;
            break;
        case SA_2_0:
            buf[4] = index;
            break;
        case SA_2_1:
            buf[4] = saPowerTable[index].dbi;
            buf[4] |= 128; //set MSB to indicate set power by dbm
            break;
        default:
            break;
    }

    buf[5] = CRC8(buf, 5);
    saQueueCmd(buf, 6);
}

static void vtxSASetPitMode(vtxDevice_t *vtxDevice, uint8_t onoff)
{
    if (!vtxSAIsReady(vtxDevice) || saDevice.version < SA_1_0) {
        return;
    }

    if (onoff && saDevice.version < SA_2_1) {
        // Smart Audio prior to V2.1 can not turn pit mode on by software.
        return;
    }

    if (saDevice.version >= SA_2_1 && !saDevice.willBootIntoPitMode) {
        if (onoff) {
            // enable pitmode using SET_POWER command with 0 dbm.
            // This enables pitmode without causing the device to boot into pitmode next power-up
            static uint8_t buf[6] = { 0xAA, 0x55, SACMD(SA_CMD_SET_POWER), 1 };
            buf[4] = 0 | 128;
            buf[5] = CRC8(buf, 5);
            saQueueCmd(buf, 6);
            LOG_D(VTX, "vtxSASetPitMode: set power to 0 dbm\r\n");
        } else {
            saSetMode(SA_MODE_CLR_PITMODE);
            LOG_D(VTX, "vtxSASetPitMode: clear pitmode permanently");
        }
       return;
    }

    uint8_t newMode = onoff ? 0 : SA_MODE_CLR_PITMODE;

    if (saDevice.mode & SA_MODE_GET_OUT_RANGE_PITMODE) {
        newMode |= SA_MODE_SET_OUT_RANGE_PITMODE;
    }

    if ((saDevice.mode & SA_MODE_GET_IN_RANGE_PITMODE) || (onoff && newMode == 0)) {
        // ensure when turning on pit mode that pit mode gets actually enabled
        newMode |= SA_MODE_SET_IN_RANGE_PITMODE;
    }
    LOG_D(VTX, "vtxSASetPitMode %s with stored mode 0x%x por %s, pir %s, newMode 0x%x\r\n", onoff ? "on" : "off", saDevice.mode,
            (saDevice.mode & SA_MODE_GET_OUT_RANGE_PITMODE) ? "on" : "off",
            (saDevice.mode & SA_MODE_GET_IN_RANGE_PITMODE) ? "on" : "off" , newMode);


    saSetMode(newMode);

    return;
}

static bool vtxSAGetBandAndChannel(const vtxDevice_t *vtxDevice, uint8_t *pBand, uint8_t *pChannel)
{
    if (!vtxSAIsReady(vtxDevice)) {
        return false;
    }

    // if in user-freq mode then report band as zero
    *pBand = (saDevice.mode & SA_MODE_GET_FREQ_BY_FREQ) ? 0 :
        (SA_DEVICE_CHVAL_TO_BAND(saDevice.channel) + 1);
    *pChannel = SA_DEVICE_CHVAL_TO_CHANNEL(saDevice.channel) + 1;

    return true;
}

static bool vtxSAGetPowerIndex(const vtxDevice_t *vtxDevice, uint8_t *pIndex)
{
    if (!vtxSAIsReady(vtxDevice)) {
        return false;
    }

    *pIndex = ((saDevice.version == SA_1_0) ? saDacToPowerIndex(saDevice.power) : saDevice.power);
    return true;
}

static bool vtxSAGetPitMode(const vtxDevice_t *vtxDevice, uint8_t *pOnOff)
{
    if (!(vtxSAIsReady(vtxDevice) && (saDevice.version < SA_2_0))) {
        return false;
    }

    *pOnOff = (saDevice.mode & SA_MODE_GET_PITMODE) ? 1 : 0;
    return true;
}

static bool vtxSAGetFreq(const vtxDevice_t *vtxDevice, uint16_t *pFreq)
{
    if (!vtxSAIsReady(vtxDevice)) {
        return false;
    }

    // if not in user-freq mode then convert band/chan to frequency
    *pFreq = (saDevice.mode & SA_MODE_GET_FREQ_BY_FREQ) ? saDevice.freq :
        vtx58_Bandchan2Freq(SA_DEVICE_CHVAL_TO_BAND(saDevice.channel) + 1,
        SA_DEVICE_CHVAL_TO_CHANNEL(saDevice.channel) + 1);
    return true;
}

static bool vtxSAGetPower(const vtxDevice_t *vtxDevice, uint8_t *pIndex, uint16_t *pPowerMw)
{
    uint8_t powerIndex;

    if (!vtxSAGetPowerIndex(vtxDevice, &powerIndex)) {
        return false;
    }

    *pIndex = powerIndex;
    *pPowerMw = (powerIndex > 0) ? saPowerTable[powerIndex - 1].mW : 0;
    return true;
}

static bool vtxSAGetOsdInfo(const  vtxDevice_t *vtxDevice, vtxDeviceOsdInfo_t * pOsdInfo)
{
    uint8_t powerIndex;
    uint16_t powerMw;
    uint16_t freq;
    uint8_t band, channel;

    if (!vtxSAGetBandAndChannel(vtxDevice, &band, &channel)) {
        return false;
    }

    if (!vtxSAGetFreq(vtxDevice, &freq)) {
        return false;
    }

    if (!vtxSAGetPower(vtxDevice, &powerIndex, &powerMw)) {
        return false;
    }

    pOsdInfo->band = band;
    pOsdInfo->channel = channel;
    pOsdInfo->frequency = freq;
    pOsdInfo->powerIndex = powerIndex;
    pOsdInfo->powerMilliwatt = powerMw;
    pOsdInfo->bandLetter = vtx58BandNames[band][0];
    pOsdInfo->bandName = vtx58BandNames[band];
    pOsdInfo->channelName = vtx58ChannelNames[channel];
    pOsdInfo->powerIndexLetter = '0' + powerIndex;
    return true;
}

static const vtxVTable_t saVTable = {
    .process = vtxSAProcess,
    .getDeviceType = vtxSAGetDeviceType,
    .isReady = vtxSAIsReady,
    .setBandAndChannel = vtxSASetBandAndChannel,
    .setPowerByIndex = vtxSASetPowerByIndex,
    .setPitMode = vtxSASetPitMode,
    .getBandAndChannel = vtxSAGetBandAndChannel,
    .getPowerIndex = vtxSAGetPowerIndex,
    .getPitMode = vtxSAGetPitMode,
    .getFrequency = vtxSAGetFreq,
    .getPower = vtxSAGetPower,
    .getOsdInfo = vtxSAGetOsdInfo,
};


#endif // VTX_SMARTAUDIO
