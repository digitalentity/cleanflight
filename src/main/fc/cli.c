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
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <ctype.h>

#include "platform.h"

uint8_t cliMode = 0;

#include "blackbox/blackbox.h"

#include "build/assert.h"
#include "build/build_config.h"
#include "build/version.h"

#include "common/axis.h"
#include "common/color.h"
#include "common/maths.h"
#include "common/printf.h"
#include "common/string_light.h"
#include "common/memory.h"
#include "common/time.h"
#include "common/typeconversion.h"
#include "common/fp_pid.h"
#include "programming/global_variables.h"
#include "programming/pid.h"

#include "config/config_eeprom.h"
#include "config/feature.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/pwm_mapping.h"
#include "drivers/buf_writer.h"
#include "drivers/bus_i2c.h"
#include "drivers/compass/compass.h"
#include "drivers/flash.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/osd_symbols.h"
#include "drivers/persistent.h"
#include "drivers/rx_pwm.h"
#include "drivers/sdcard/sdcard.h"
#include "drivers/sensor.h"
#include "drivers/serial.h"
#include "drivers/stack_check.h"
#include "drivers/system.h"
#include "drivers/time.h"
#include "drivers/usb_msc.h"
#include "drivers/vtx_common.h"

#include "fc/fc_core.h"
#include "fc/cli.h"
#include "fc/config.h"
#include "fc/controlrate_profile.h"
#include "fc/rc_adjustments.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"
#include "fc/settings.h"

#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/servos.h"
#include "flight/secondary_imu.h"

#include "io/asyncfatfs/asyncfatfs.h"
#include "io/beeper.h"
#include "io/flashfs.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/osd.h"
#include "io/serial.h"

#include "fc/fc_msp_box.h"

#include "navigation/navigation.h"
#include "navigation/navigation_private.h"

#include "rx/rx.h"
#include "rx/spektrum.h"
#include "rx/eleres.h"
#include "rx/srxl2.h"

#include "scheduler/scheduler.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"
#include "sensors/compass.h"
#include "sensors/diagnostics.h"
#include "sensors/gyro.h"
#include "sensors/pitotmeter.h"
#include "sensors/rangefinder.h"
#include "sensors/opflow.h"
#include "sensors/sensors.h"
#include "sensors/temperature.h"

#include "telemetry/frsky_d.h"
#include "telemetry/telemetry.h"
#include "build/debug.h"

#if MCU_FLASH_SIZE > 128
#define PLAY_SOUND
#endif

extern timeDelta_t cycleTime; // FIXME dependency on mw.c
extern uint8_t detectedSensors[SENSOR_INDEX_COUNT];

static serialPort_t *cliPort;

static bufWriter_t *cliWriter;
static uint8_t cliWriteBuffer[sizeof(*cliWriter) + 128];

static char cliBuffer[64];
static uint32_t bufferIndex = 0;

#if defined(USE_ASSERT)
static void cliAssert(char *cmdline);
#endif

#ifdef USE_CLI_BATCH
static bool commandBatchActive = false;
static bool commandBatchError = false;
#endif

// sync this with features_e
static const char * const featureNames[] = {
    "THR_VBAT_COMP", "VBAT", "TX_PROF_SEL", "BAT_PROF_AUTOSWITCH", "MOTOR_STOP",
    "", "SOFTSERIAL", "GPS", "RPM_FILTERS",
    "", "TELEMETRY", "CURRENT_METER", "REVERSIBLE_MOTORS", "",
    "", "RSSI_ADC", "LED_STRIP", "DASHBOARD", "",
    "BLACKBOX", "", "TRANSPONDER", "AIRMODE",
    "SUPEREXPO", "VTX", "", "", "", "PWM_OUTPUT_ENABLE",
    "OSD", "FW_LAUNCH", "FW_AUTOTRIM", NULL
};

/* Sensor names (used in lookup tables for *_hardware settings and in status command output) */
// sync with gyroSensor_e
static const char * const gyroNames[] = { "NONE", "AUTO", "MPU6050", "L3G4200D", "MPU3050", "L3GD20", "MPU6000", "MPU6500", "MPU9250", "BMI160", "ICM20689", "FAKE"};

// sync this with sensors_e
static const char * const sensorTypeNames[] = {
    "GYRO", "ACC", "BARO", "MAG", "RANGEFINDER", "PITOT", "OPFLOW", "GPS", "GPS+MAG", NULL
};

#define SENSOR_NAMES_MASK (SENSOR_GYRO | SENSOR_ACC | SENSOR_BARO | SENSOR_MAG | SENSOR_RANGEFINDER | SENSOR_PITOT | SENSOR_OPFLOW)

static const char * const hardwareSensorStatusNames[] = {
    "NONE", "OK", "UNAVAILABLE", "FAILING"
};

static const char * const *sensorHardwareNames[] = {
        gyroNames,
        table_acc_hardware,
#ifdef USE_BARO
        table_baro_hardware,
#else
        NULL,
#endif
#ifdef USE_MAG
        table_mag_hardware,
#else
        NULL,
#endif
#ifdef USE_RANGEFINDER
        table_rangefinder_hardware,
#else
        NULL,
#endif
#ifdef USE_PITOT
        table_pitot_hardware,
#else
        NULL,
#endif
#ifdef USE_OPFLOW
        table_opflow_hardware,
#else
        NULL,
#endif
};

static void cliPrint(const char *str)
{
    while (*str) {
        bufWriterAppend(cliWriter, *str++);
    }
}

static void cliPrintLinefeed(void)
{
    cliPrint("\r\n");
}

static void cliPrintLine(const char *str)
{
    cliPrint(str);
    cliPrintLinefeed();
}

static void cliPrintError(const char *str)
{
    cliPrint("### ERROR: ");
    cliPrint(str);
#ifdef USE_CLI_BATCH
    if (commandBatchActive) {
        commandBatchError = true;
    }
#endif
}

static void cliPrintErrorLine(const char *str)
{
    cliPrint("### ERROR: ");
    cliPrintLine(str);
#ifdef USE_CLI_BATCH
    if (commandBatchActive) {
        commandBatchError = true;
    }
#endif
}

#ifdef CLI_MINIMAL_VERBOSITY
#define cliPrintHashLine(str)
#else
static void cliPrintHashLine(const char *str)
{
    cliPrint("\r\n# ");
    cliPrintLine(str);
}
#endif

static void cliPutp(void *p, char ch)
{
    bufWriterAppend(p, ch);
}

typedef enum {
    DUMP_MASTER = (1 << 0),
    DUMP_PROFILE = (1 << 1),
    DUMP_BATTERY_PROFILE = (1 << 2),
    DUMP_RATES = (1 << 3),
    DUMP_ALL = (1 << 4),
    DO_DIFF = (1 << 5),
    SHOW_DEFAULTS = (1 << 6),
    HIDE_UNUSED = (1 << 7)
} dumpFlags_e;

static void cliPrintfva(const char *format, va_list va)
{
    tfp_format(cliWriter, cliPutp, format, va);
    bufWriterFlush(cliWriter);
}

static void cliPrintLinefva(const char *format, va_list va)
{
    tfp_format(cliWriter, cliPutp, format, va);
    bufWriterFlush(cliWriter);
    cliPrintLinefeed();
}

static bool cliDumpPrintLinef(uint8_t dumpMask, bool equalsDefault, const char *format, ...)
{
    if (!((dumpMask & DO_DIFF) && equalsDefault)) {
        va_list va;
        va_start(va, format);
        cliPrintLinefva(format, va);
        va_end(va);
        return true;
    } else {
        return false;
    }
}

static void cliWrite(uint8_t ch)
{
    bufWriterAppend(cliWriter, ch);
}

static bool cliDefaultPrintLinef(uint8_t dumpMask, bool equalsDefault, const char *format, ...)
{
    if ((dumpMask & SHOW_DEFAULTS) && !equalsDefault) {
        cliWrite('#');

        va_list va;
        va_start(va, format);
        cliPrintLinefva(format, va);
        va_end(va);
        return true;
    } else {
        return false;
    }
}

static void cliPrintf(const char *format, ...)
{
    va_list va;
    va_start(va, format);
    cliPrintfva(format, va);
    va_end(va);
}


static void cliPrintLinef(const char *format, ...)
{
    va_list va;
    va_start(va, format);
    cliPrintLinefva(format, va);
    va_end(va);
}

static void cliPrintErrorVa(const char *format, va_list va)
{
    cliPrint("### ERROR: ");
    cliPrintfva(format, va);
    va_end(va);

#ifdef USE_CLI_BATCH
    if (commandBatchActive) {
        commandBatchError = true;
    }
#endif
}

static void cliPrintErrorLinef(const char *format, ...)
{
    va_list va;
    va_start(va, format);
    cliPrintErrorVa(format, va);
    cliPrintLinefeed();
}

static void printValuePointer(const setting_t *var, const void *valuePointer, uint32_t full)
{
    int32_t value = 0;
    char buf[SETTING_MAX_NAME_LENGTH];

    switch (SETTING_TYPE(var)) {
    case VAR_UINT8:
        value = *(uint8_t *)valuePointer;
        break;

    case VAR_INT8:
        value = *(int8_t *)valuePointer;
        break;

    case VAR_UINT16:
        value = *(uint16_t *)valuePointer;
        break;

    case VAR_INT16:
        value = *(int16_t *)valuePointer;
        break;

    case VAR_UINT32:
        value = *(uint32_t *)valuePointer;
        break;

    case VAR_FLOAT:
        cliPrintf("%s", ftoa(*(float *)valuePointer, buf));
        if (full) {
            if (SETTING_MODE(var) == MODE_DIRECT) {
                cliPrintf(" %s", ftoa((float)settingGetMin(var), buf));
                cliPrintf(" %s", ftoa((float)settingGetMax(var), buf));
            }
        }
        return; // return from case for float only

    case VAR_STRING:
        cliPrintf("%s", (const char *)valuePointer);
        return;
    }

    switch (SETTING_MODE(var)) {
    case MODE_DIRECT:
        if (SETTING_TYPE(var) == VAR_UINT32)
            cliPrintf("%u", value);
        else
            cliPrintf("%d", value);
        if (full) {
            if (SETTING_MODE(var) == MODE_DIRECT) {
                cliPrintf(" %d %u", settingGetMin(var), settingGetMax(var));
            }
        }
        break;
    case MODE_LOOKUP:
    {
        const char *name = settingLookupValueName(var, value);
        if (name) {
            cliPrintf(name);
        } else {
            settingGetName(var, buf);
            cliPrintErrorLinef("VALUE %d OUT OF RANGE FOR %s", (int)value, buf);
        }
        break;
    }
    }
}

static bool valuePtrEqualsDefault(const setting_t *value, const void *ptr, const void *ptrDefault)
{
    bool result = false;
    switch (SETTING_TYPE(value)) {
    case VAR_UINT8:
        result = *(uint8_t *)ptr == *(uint8_t *)ptrDefault;
        break;

    case VAR_INT8:
        result = *(int8_t *)ptr == *(int8_t *)ptrDefault;
        break;

    case VAR_UINT16:
        result = *(uint16_t *)ptr == *(uint16_t *)ptrDefault;
        break;

    case VAR_INT16:
        result = *(int16_t *)ptr == *(int16_t *)ptrDefault;
        break;

    case VAR_UINT32:
        result = *(uint32_t *)ptr == *(uint32_t *)ptrDefault;
        break;

    case VAR_FLOAT:
        result = *(float *)ptr == *(float *)ptrDefault;
        break;

    case VAR_STRING:
        result = strncmp(ptr, ptrDefault, settingGetStringMaxLength(value) + 1) == 0;
        break;
    }
    return result;
}

static void dumpPgValue(const setting_t *value, uint8_t dumpMask)
{
    char name[SETTING_MAX_NAME_LENGTH];
    const char *format = "set %s = ";
    const char *defaultFormat = "#set %s = ";
    // During a dump, the PGs have been backed up to their "copy"
    // regions and the actual values have been reset to its
    // defaults. This means that settingGetValuePointer() will
    // return the default value while settingGetCopyValuePointer()
    // will return the actual value.
    const void *valuePointer = settingGetCopyValuePointer(value);
    const void *defaultValuePointer = settingGetValuePointer(value);
    const bool equalsDefault = valuePtrEqualsDefault(value, valuePointer, defaultValuePointer);
    if (((dumpMask & DO_DIFF) == 0) || !equalsDefault) {
        settingGetName(value, name);
        if (dumpMask & SHOW_DEFAULTS && !equalsDefault) {
            cliPrintf(defaultFormat, name);
            printValuePointer(value, defaultValuePointer, 0);
            cliPrintLinefeed();
        }
        cliPrintf(format, name);
        printValuePointer(value, valuePointer, 0);
        cliPrintLinefeed();
    }
}

static void dumpAllValues(uint16_t valueSection, uint8_t dumpMask)
{
    for (unsigned i = 0; i < SETTINGS_TABLE_COUNT; i++) {
        const setting_t *value = settingGet(i);
        bufWriterFlush(cliWriter);
        if (SETTING_SECTION(value) == valueSection) {
            dumpPgValue(value, dumpMask);
        }
    }
}

static void cliPrintVar(const setting_t *var, uint32_t full)
{
    const void *ptr = settingGetValuePointer(var);

    printValuePointer(var, ptr, full);
}

static void cliPrintVarRange(const setting_t *var)
{
    switch (SETTING_MODE(var)) {
    case MODE_DIRECT:
        if (SETTING_TYPE(var) == VAR_STRING) {
           cliPrintLinef("Max. length: %u", settingGetStringMaxLength(var));
           break;
        }
        cliPrintLinef("Allowed range: %d - %u", settingGetMin(var), settingGetMax(var));
        break;
    case MODE_LOOKUP:
    {
        const lookupTableEntry_t *tableEntry = settingLookupTable(var);
        cliPrint("Allowed values:");
        for (uint32_t i = 0; i < tableEntry->valueCount ; i++) {
            if (i > 0)
                cliPrint(",");
            cliPrintf(" %s", tableEntry->values[i]);
        }
        cliPrintLinefeed();
    }
        break;
    }
}

typedef union {
    uint32_t uint_value;
    int32_t int_value;
    float float_value;
} int_float_value_t;

static void cliSetIntFloatVar(const setting_t *var, const int_float_value_t value)
{
    void *ptr = settingGetValuePointer(var);

    switch (SETTING_TYPE(var)) {
    case VAR_UINT8:
    case VAR_INT8:
        *(int8_t *)ptr = value.int_value;
        break;

    case VAR_UINT16:
    case VAR_INT16:
        *(int16_t *)ptr = value.int_value;
        break;

    case VAR_UINT32:
        *(uint32_t *)ptr = value.uint_value;
        break;

    case VAR_FLOAT:
        *(float *)ptr = (float)value.float_value;
        break;

    case VAR_STRING:
        // Handled by cliSet directly
        break;
    }
}

static void cliPrompt(void)
{
    cliPrint("\r\n# ");
    bufWriterFlush(cliWriter);
}

static void cliShowParseError(void)
{
    cliPrintErrorLinef("Parse error");
}

static void cliShowArgumentRangeError(char *name, int min, int max)
{
    cliPrintErrorLinef("%s must be between %d and %d", name, min, max);
}

static const char *nextArg(const char *currentArg)
{
    const char *ptr = strchr(currentArg, ' ');
    while (ptr && *ptr == ' ') {
        ptr++;
    }

    return ptr;
}

static const char *processChannelRangeArgs(const char *ptr, channelRange_t *range, uint8_t *validArgumentCount)
{
    for (uint32_t argIndex = 0; argIndex < 2; argIndex++) {
        ptr = nextArg(ptr);
        if (ptr) {
            int val = fastA2I(ptr);
            val = CHANNEL_VALUE_TO_STEP(val);
            if (val >= MIN_MODE_RANGE_STEP && val <= MAX_MODE_RANGE_STEP) {
                if (argIndex == 0) {
                    range->startStep = val;
                } else {
                    range->endStep = val;
                }
                (*validArgumentCount)++;
            }
        }
    }

    return ptr;
}

// Check if a string's length is zero
static bool isEmpty(const char *string)
{
    return (string == NULL || *string == '\0') ? true : false;
}

#if defined(USE_ASSERT)
static void cliAssert(char *cmdline)
{
    UNUSED(cmdline);

    if (assertFailureLine) {
        if (assertFailureFile) {
            cliPrintErrorLinef("Assertion failed at line %d, file %s", assertFailureLine, assertFailureFile);
        }
        else {
            cliPrintErrorLinef("Assertion failed at line %d", assertFailureLine);
        }
#ifdef USE_CLI_BATCH
        if (commandBatchActive) {
            commandBatchError = true;
        }
#endif
    }
    else {
        cliPrintLine("No assert() failed");
    }
}
#endif

static void printAux(uint8_t dumpMask, const modeActivationCondition_t *modeActivationConditions, const modeActivationCondition_t *defaultModeActivationConditions)
{
    const char *format = "aux %u %u %u %u %u";
    // print out aux channel settings
    for (uint32_t i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
        const modeActivationCondition_t *mac = &modeActivationConditions[i];
        bool equalsDefault = false;
        if (defaultModeActivationConditions) {
            const modeActivationCondition_t *macDefault = &defaultModeActivationConditions[i];
            equalsDefault = mac->modeId == macDefault->modeId
                && mac->auxChannelIndex == macDefault->auxChannelIndex
                && mac->range.startStep == macDefault->range.startStep
                && mac->range.endStep == macDefault->range.endStep;
            const box_t *box = findBoxByActiveBoxId(macDefault->modeId);
            cliDefaultPrintLinef(dumpMask, equalsDefault, format,
                i,
                box->permanentId,
                macDefault->auxChannelIndex,
                MODE_STEP_TO_CHANNEL_VALUE(macDefault->range.startStep),
                MODE_STEP_TO_CHANNEL_VALUE(macDefault->range.endStep)
            );
        }
        const box_t *box = findBoxByActiveBoxId(mac->modeId);
        cliDumpPrintLinef(dumpMask, equalsDefault, format,
            i,
            box->permanentId,
            mac->auxChannelIndex,
            MODE_STEP_TO_CHANNEL_VALUE(mac->range.startStep),
            MODE_STEP_TO_CHANNEL_VALUE(mac->range.endStep)
        );
    }
}

static void cliAux(char *cmdline)
{
    int i, val = 0;
    const char *ptr;

    if (isEmpty(cmdline)) {
        printAux(DUMP_MASTER, modeActivationConditions(0), NULL);
    } else {
        ptr = cmdline;
        i = fastA2I(ptr++);
        if (i < MAX_MODE_ACTIVATION_CONDITION_COUNT) {
            modeActivationCondition_t *mac = modeActivationConditionsMutable(i);
            uint8_t validArgumentCount = 0;
            ptr = nextArg(ptr);
            if (ptr) {
                val = fastA2I(ptr);
                if (val >= 0) {
                    const box_t *box = findBoxByPermanentId(val);
                    if (box != NULL) {
                        mac->modeId = box->boxId;
                        validArgumentCount++;
                    }
                }
            }
            ptr = nextArg(ptr);
            if (ptr) {
                val = fastA2I(ptr);
                if (val >= 0 && val < MAX_AUX_CHANNEL_COUNT) {
                    mac->auxChannelIndex = val;
                    validArgumentCount++;
                }
            }
            ptr = processChannelRangeArgs(ptr, &mac->range, &validArgumentCount);

            if (validArgumentCount != 4) {
                memset(mac, 0, sizeof(modeActivationCondition_t));
            }
        } else {
            cliShowArgumentRangeError("index", 0, MAX_MODE_ACTIVATION_CONDITION_COUNT - 1);
        }
    }
}

static void printSerial(uint8_t dumpMask, const serialConfig_t *serialConfig, const serialConfig_t *serialConfigDefault)
{
    const char *format = "serial %d %d %ld %ld %ld %ld";
    for (uint32_t i = 0; i < SERIAL_PORT_COUNT; i++) {
        if (!serialIsPortAvailable(serialConfig->portConfigs[i].identifier)) {
            continue;
        };
        bool equalsDefault = false;
        if (serialConfigDefault) {
            equalsDefault = serialConfig->portConfigs[i].identifier == serialConfigDefault->portConfigs[i].identifier
                && serialConfig->portConfigs[i].functionMask == serialConfigDefault->portConfigs[i].functionMask
                && serialConfig->portConfigs[i].msp_baudrateIndex == serialConfigDefault->portConfigs[i].msp_baudrateIndex
                && serialConfig->portConfigs[i].gps_baudrateIndex == serialConfigDefault->portConfigs[i].gps_baudrateIndex
                && serialConfig->portConfigs[i].telemetry_baudrateIndex == serialConfigDefault->portConfigs[i].telemetry_baudrateIndex
                && serialConfig->portConfigs[i].peripheral_baudrateIndex == serialConfigDefault->portConfigs[i].peripheral_baudrateIndex;
            cliDefaultPrintLinef(dumpMask, equalsDefault, format,
                serialConfigDefault->portConfigs[i].identifier,
                serialConfigDefault->portConfigs[i].functionMask,
                baudRates[serialConfigDefault->portConfigs[i].msp_baudrateIndex],
                baudRates[serialConfigDefault->portConfigs[i].gps_baudrateIndex],
                baudRates[serialConfigDefault->portConfigs[i].telemetry_baudrateIndex],
                baudRates[serialConfigDefault->portConfigs[i].peripheral_baudrateIndex]
            );
        }
        cliDumpPrintLinef(dumpMask, equalsDefault, format,
            serialConfig->portConfigs[i].identifier,
            serialConfig->portConfigs[i].functionMask,
            baudRates[serialConfig->portConfigs[i].msp_baudrateIndex],
            baudRates[serialConfig->portConfigs[i].gps_baudrateIndex],
            baudRates[serialConfig->portConfigs[i].telemetry_baudrateIndex],
            baudRates[serialConfig->portConfigs[i].peripheral_baudrateIndex]
            );
    }
}

static void cliSerial(char *cmdline)
{
    if (isEmpty(cmdline)) {
        printSerial(DUMP_MASTER, serialConfig(), NULL);
        return;
    }
    serialPortConfig_t portConfig;

    serialPortConfig_t *currentConfig;

    uint8_t validArgumentCount = 0;

    const char *ptr = cmdline;

    int val = fastA2I(ptr++);
    currentConfig = serialFindPortConfiguration(val);
    if (!currentConfig) {
        // Invalid port ID
        cliPrintErrorLinef("Invalid port ID %d", val);
        return;
    }
    memcpy(&portConfig, currentConfig, sizeof(portConfig));
    validArgumentCount++;

    ptr = nextArg(ptr);
    if (ptr) {
        switch (*ptr) {
            case '+':
                // Add function
                ptr++;
                val = fastA2I(ptr);
                portConfig.functionMask |= (1 << val);
                break;
            case '-':
                // Remove function
                ptr++;
                val = fastA2I(ptr);
                portConfig.functionMask &= 0xFFFFFFFF ^ (1 << val);
                break;
            default:
                // Set functions
                val = fastA2I(ptr);
                portConfig.functionMask = val & 0xFFFFFFFF;
                break;
        }
        validArgumentCount++;
    }

    for (int i = 0; i < 4; i ++) {
        ptr = nextArg(ptr);
        if (!ptr) {
            break;
        }

        val = fastA2I(ptr);

        uint8_t baudRateIndex = lookupBaudRateIndex(val);
        if (baudRates[baudRateIndex] != (uint32_t) val) {
            break;
        }

        switch (i) {
        case 0:
            baudRateIndex = constrain(baudRateIndex, BAUD_MIN, BAUD_MAX);
            portConfig.msp_baudrateIndex = baudRateIndex;
            break;
        case 1:
            baudRateIndex = constrain(baudRateIndex, BAUD_MIN, BAUD_MAX);
            portConfig.gps_baudrateIndex = baudRateIndex;
            break;
        case 2:
            baudRateIndex = constrain(baudRateIndex, BAUD_MIN, BAUD_MAX);
            portConfig.telemetry_baudrateIndex = baudRateIndex;
            break;
        case 3:
            baudRateIndex = constrain(baudRateIndex, BAUD_MIN, BAUD_MAX);
            portConfig.peripheral_baudrateIndex = baudRateIndex;
            break;
        }

        validArgumentCount++;
    }

    if (validArgumentCount < 2) {
        cliShowParseError();
        return;
    }

    memcpy(currentConfig, &portConfig, sizeof(portConfig));
}

#ifdef USE_SERIAL_PASSTHROUGH
static void cliSerialPassthrough(char *cmdline)
{
    char * saveptr;

    if (isEmpty(cmdline)) {
        cliShowParseError();
        return;
    }

    int id = -1;
    uint32_t baud = 0;
    unsigned mode = 0;
    char* tok = strtok_r(cmdline, " ", &saveptr);
    int index = 0;

    while (tok != NULL) {
        switch (index) {
            case 0:
                id = fastA2I(tok);
                break;
            case 1:
                baud = fastA2I(tok);
                break;
            case 2:
                if (strstr(tok, "rx") || strstr(tok, "RX"))
                    mode |= MODE_RX;
                if (strstr(tok, "tx") || strstr(tok, "TX"))
                    mode |= MODE_TX;
                break;
        }
        index++;
        tok = strtok_r(NULL, " ", &saveptr);
    }

    serialPort_t *passThroughPort;
    serialPortUsage_t *passThroughPortUsage = findSerialPortUsageByIdentifier(id);
    if (!passThroughPortUsage || passThroughPortUsage->serialPort == NULL) {
        if (!baud) {
            tfp_printf("Port %d is closed, must specify baud.\r\n", id);
            return;
        }
        if (!mode)
            mode = MODE_RXTX;

        passThroughPort = openSerialPort(id, FUNCTION_NONE, NULL, NULL,
                                         baud, mode,
                                         SERIAL_NOT_INVERTED);
        if (!passThroughPort) {
            tfp_printf("Port %d could not be opened.\r\n", id);
            return;
        }
        tfp_printf("Port %d opened, baud = %u.\r\n", id, (unsigned)baud);
    } else {
        passThroughPort = passThroughPortUsage->serialPort;
        // If the user supplied a mode, override the port's mode, otherwise
        // leave the mode unchanged. serialPassthrough() handles one-way ports.
        tfp_printf("Port %d already open.\r\n", id);
        if (mode && passThroughPort->mode != mode) {
            tfp_printf("Adjusting mode from %d to %d.\r\n",
                   passThroughPort->mode, mode);
            serialSetMode(passThroughPort, mode);
        }
        // If this port has a rx callback associated we need to remove it now.
        // Otherwise no data will be pushed in the serial port buffer!
        if (passThroughPort->rxCallback) {
            tfp_printf("Removing rxCallback\r\n");
            passThroughPort->rxCallback = 0;
        }
    }

    tfp_printf("Forwarding data to %d, power cycle to exit.\r\n", id);

    serialPassthrough(cliPort, passThroughPort, NULL, NULL);
}
#endif

static void printAdjustmentRange(uint8_t dumpMask, const adjustmentRange_t *adjustmentRanges, const adjustmentRange_t *defaultAdjustmentRanges)
{
    const char *format = "adjrange %u %u %u %u %u %u %u";
    // print out adjustment ranges channel settings
    for (uint32_t i = 0; i < MAX_ADJUSTMENT_RANGE_COUNT; i++) {
        const adjustmentRange_t *ar = &adjustmentRanges[i];
        bool equalsDefault = false;
        if (defaultAdjustmentRanges) {
            const adjustmentRange_t *arDefault = &defaultAdjustmentRanges[i];
            equalsDefault = ar->auxChannelIndex == arDefault->auxChannelIndex
                && ar->range.startStep == arDefault->range.startStep
                && ar->range.endStep == arDefault->range.endStep
                && ar->adjustmentFunction == arDefault->adjustmentFunction
                && ar->auxSwitchChannelIndex == arDefault->auxSwitchChannelIndex
                && ar->adjustmentIndex == arDefault->adjustmentIndex;
            cliDefaultPrintLinef(dumpMask, equalsDefault, format,
                i,
                arDefault->adjustmentIndex,
                arDefault->auxChannelIndex,
                MODE_STEP_TO_CHANNEL_VALUE(arDefault->range.startStep),
                MODE_STEP_TO_CHANNEL_VALUE(arDefault->range.endStep),
                arDefault->adjustmentFunction,
                arDefault->auxSwitchChannelIndex
            );
        }
        cliDumpPrintLinef(dumpMask, equalsDefault, format,
            i,
            ar->adjustmentIndex,
            ar->auxChannelIndex,
            MODE_STEP_TO_CHANNEL_VALUE(ar->range.startStep),
            MODE_STEP_TO_CHANNEL_VALUE(ar->range.endStep),
            ar->adjustmentFunction,
            ar->auxSwitchChannelIndex
        );
    }
}

static void cliAdjustmentRange(char *cmdline)
{
    int i, val = 0;
    const char *ptr;

    if (isEmpty(cmdline)) {
        printAdjustmentRange(DUMP_MASTER, adjustmentRanges(0), NULL);
    } else {
        ptr = cmdline;
        i = fastA2I(ptr++);
        if (i < MAX_ADJUSTMENT_RANGE_COUNT) {
            adjustmentRange_t *ar = adjustmentRangesMutable(i);
            uint8_t validArgumentCount = 0;

            ptr = nextArg(ptr);
            if (ptr) {
                val = fastA2I(ptr);
                if (val >= 0 && val < MAX_SIMULTANEOUS_ADJUSTMENT_COUNT) {
                    ar->adjustmentIndex = val;
                    validArgumentCount++;
                }
            }
            ptr = nextArg(ptr);
            if (ptr) {
                val = fastA2I(ptr);
                if (val >= 0 && val < MAX_AUX_CHANNEL_COUNT) {
                    ar->auxChannelIndex = val;
                    validArgumentCount++;
                }
            }

            ptr = processChannelRangeArgs(ptr, &ar->range, &validArgumentCount);

            ptr = nextArg(ptr);
            if (ptr) {
                val = fastA2I(ptr);
                if (val >= 0 && val < ADJUSTMENT_FUNCTION_COUNT) {
                    ar->adjustmentFunction = val;
                    validArgumentCount++;
                }
            }
            ptr = nextArg(ptr);
            if (ptr) {
                val = fastA2I(ptr);
                if (val >= 0 && val < MAX_AUX_CHANNEL_COUNT) {
                    ar->auxSwitchChannelIndex = val;
                    validArgumentCount++;
                }
            }

            if (validArgumentCount != 6) {
                memset(ar, 0, sizeof(adjustmentRange_t));
                cliShowParseError();
            }
        } else {
            cliShowArgumentRangeError("index", 0, MAX_ADJUSTMENT_RANGE_COUNT - 1);
        }
    }
}

static void printMotorMix(uint8_t dumpMask, const motorMixer_t *primaryMotorMixer, const motorMixer_t *defaultprimaryMotorMixer)
{
    const char *format = "mmix %d %s %s %s %s";
    char buf0[FTOA_BUFFER_SIZE];
    char buf1[FTOA_BUFFER_SIZE];
    char buf2[FTOA_BUFFER_SIZE];
    char buf3[FTOA_BUFFER_SIZE];
    for (uint32_t i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        if (primaryMotorMixer[i].throttle == 0.0f)
            break;
        const float thr = primaryMotorMixer[i].throttle;
        const float roll = primaryMotorMixer[i].roll;
        const float pitch = primaryMotorMixer[i].pitch;
        const float yaw = primaryMotorMixer[i].yaw;
        bool equalsDefault = false;
        if (defaultprimaryMotorMixer) {
            const float thrDefault = defaultprimaryMotorMixer[i].throttle;
            const float rollDefault = defaultprimaryMotorMixer[i].roll;
            const float pitchDefault = defaultprimaryMotorMixer[i].pitch;
            const float yawDefault = defaultprimaryMotorMixer[i].yaw;
            const bool equalsDefault = thr == thrDefault && roll == rollDefault && pitch == pitchDefault && yaw == yawDefault;

            cliDefaultPrintLinef(dumpMask, equalsDefault, format,
                i,
                ftoa(thrDefault, buf0),
                ftoa(rollDefault, buf1),
                ftoa(pitchDefault, buf2),
                ftoa(yawDefault, buf3));
        }
        cliDumpPrintLinef(dumpMask, equalsDefault, format,
            i,
            ftoa(thr, buf0),
            ftoa(roll, buf1),
            ftoa(pitch, buf2),
            ftoa(yaw, buf3));
    }
}

static void cliMotorMix(char *cmdline)
{
    int check = 0;
    const char *ptr;

    if (isEmpty(cmdline)) {
        printMotorMix(DUMP_MASTER, primaryMotorMixer(0), NULL);
    } else if (sl_strncasecmp(cmdline, "reset", 5) == 0) {
        // erase custom mixer
        for (uint32_t i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
            primaryMotorMixerMutable(i)->throttle = 0.0f;
        }
    } else {
        ptr = cmdline;
        uint32_t i = fastA2I(ptr); // get motor number
        if (i < MAX_SUPPORTED_MOTORS) {
            ptr = nextArg(ptr);
            if (ptr) {
                primaryMotorMixerMutable(i)->throttle = fastA2F(ptr);
                check++;
            }
            ptr = nextArg(ptr);
            if (ptr) {
                primaryMotorMixerMutable(i)->roll = fastA2F(ptr);
                check++;
            }
            ptr = nextArg(ptr);
            if (ptr) {
                primaryMotorMixerMutable(i)->pitch = fastA2F(ptr);
                check++;
            }
            ptr = nextArg(ptr);
            if (ptr) {
                primaryMotorMixerMutable(i)->yaw = fastA2F(ptr);
                check++;
            }
            if (check != 4) {
                cliShowParseError();
            } else {
                printMotorMix(DUMP_MASTER, primaryMotorMixer(0), NULL);
            }
        } else {
            cliShowArgumentRangeError("index", 0, MAX_SUPPORTED_MOTORS - 1);
        }
    }
}

static void printRxRange(uint8_t dumpMask, const rxChannelRangeConfig_t *channelRangeConfigs, const rxChannelRangeConfig_t *defaultChannelRangeConfigs)
{
    const char *format = "rxrange %u %u %u";
    for (uint32_t i = 0; i < NON_AUX_CHANNEL_COUNT; i++) {
        bool equalsDefault = false;
        if (defaultChannelRangeConfigs) {
            equalsDefault = channelRangeConfigs[i].min == defaultChannelRangeConfigs[i].min
                && channelRangeConfigs[i].max == defaultChannelRangeConfigs[i].max;
            cliDefaultPrintLinef(dumpMask, equalsDefault, format,
                i,
                defaultChannelRangeConfigs[i].min,
                defaultChannelRangeConfigs[i].max
            );
        }
        cliDumpPrintLinef(dumpMask, equalsDefault, format,
            i,
            channelRangeConfigs[i].min,
            channelRangeConfigs[i].max
        );
    }
}

static void cliRxRange(char *cmdline)
{
    int i, validArgumentCount = 0;
    const char *ptr;

    if (isEmpty(cmdline)) {
        printRxRange(DUMP_MASTER, rxChannelRangeConfigs(0), NULL);
    } else if (sl_strcasecmp(cmdline, "reset") == 0) {
        resetAllRxChannelRangeConfigurations();
    } else {
        ptr = cmdline;
        i = fastA2I(ptr);
        if (i >= 0 && i < NON_AUX_CHANNEL_COUNT) {
            int rangeMin, rangeMax;

            ptr = nextArg(ptr);
            if (ptr) {
                rangeMin = fastA2I(ptr);
                validArgumentCount++;
            }

            ptr = nextArg(ptr);
            if (ptr) {
                rangeMax = fastA2I(ptr);
                validArgumentCount++;
            }

            if (validArgumentCount != 2) {
                cliShowParseError();
            } else if (rangeMin < PWM_PULSE_MIN || rangeMin > PWM_PULSE_MAX || rangeMax < PWM_PULSE_MIN || rangeMax > PWM_PULSE_MAX) {
                cliShowParseError();
            } else {
                rxChannelRangeConfig_t *channelRangeConfig = rxChannelRangeConfigsMutable(i);
                channelRangeConfig->min = rangeMin;
                channelRangeConfig->max = rangeMax;
            }
        } else {
            cliShowArgumentRangeError("channel", 0, NON_AUX_CHANNEL_COUNT - 1);
        }
    }
}

#ifdef USE_TEMPERATURE_SENSOR
static void printTempSensor(uint8_t dumpMask, const tempSensorConfig_t *tempSensorConfigs, const tempSensorConfig_t *defaultTempSensorConfigs)
{
    const char *format = "temp_sensor %u %u %s %d %d %u %s";
    for (uint8_t i = 0; i < MAX_TEMP_SENSORS; i++) {
        bool equalsDefault = false;
        char label[5], hex_address[17];
        strncpy(label, tempSensorConfigs[i].label, TEMPERATURE_LABEL_LEN);
        label[4] = '\0';
        tempSensorAddressToString(tempSensorConfigs[i].address, hex_address);
        if (defaultTempSensorConfigs) {
            equalsDefault = tempSensorConfigs[i].type == defaultTempSensorConfigs[i].type
                && tempSensorConfigs[i].address == defaultTempSensorConfigs[i].address
                && tempSensorConfigs[i].osdSymbol == defaultTempSensorConfigs[i].osdSymbol
                && !memcmp(tempSensorConfigs[i].label, defaultTempSensorConfigs[i].label, TEMPERATURE_LABEL_LEN)
                && tempSensorConfigs[i].alarm_min == defaultTempSensorConfigs[i].alarm_min
                && tempSensorConfigs[i].alarm_max == defaultTempSensorConfigs[i].alarm_max;
            cliDefaultPrintLinef(dumpMask, equalsDefault, format,
                i,
                defaultTempSensorConfigs[i].type,
                "0",
                defaultTempSensorConfigs[i].alarm_min,
                defaultTempSensorConfigs[i].alarm_max,
                0,
                ""
            );
        }
        cliDumpPrintLinef(dumpMask, equalsDefault, format,
            i,
            tempSensorConfigs[i].type,
            hex_address,
            tempSensorConfigs[i].alarm_min,
            tempSensorConfigs[i].alarm_max,
            tempSensorConfigs[i].osdSymbol,
            label
        );
    }
}

static void cliTempSensor(char *cmdline)
{
    if (isEmpty(cmdline)) {
        printTempSensor(DUMP_MASTER, tempSensorConfig(0), NULL);
    } else if (sl_strcasecmp(cmdline, "reset") == 0) {
        resetTempSensorConfig();
    } else {
        int16_t i;
        const char *ptr = cmdline, *label;
        int16_t type=0, alarm_min=0, alarm_max=0;
        bool addressValid = false;
        uint64_t address;
        int8_t osdSymbol=0;
        uint8_t validArgumentCount = 0;
        i = fastA2I(ptr);
        if (i >= 0 && i < MAX_TEMP_SENSORS) {

            ptr = nextArg(ptr);
            if (ptr) {
                type = fastA2I(ptr);
                validArgumentCount++;
            }

            ptr = nextArg(ptr);
            if (ptr) {
                addressValid = tempSensorStringToAddress(ptr, &address);
                validArgumentCount++;
            }

            ptr = nextArg(ptr);
            if (ptr) {
                alarm_min = fastA2I(ptr);
                validArgumentCount++;
            }

            ptr = nextArg(ptr);
            if (ptr) {
                alarm_max = fastA2I(ptr);
                validArgumentCount++;
            }

            ptr = nextArg(ptr);
            if (ptr) {
                osdSymbol = fastA2I(ptr);
                validArgumentCount++;
            }

            label = nextArg(ptr);
            if (label)
                ++validArgumentCount;
            else
                label = "";

            if (validArgumentCount < 4) {
                cliShowParseError();
            } else if (type < 0 || type > TEMP_SENSOR_DS18B20 || alarm_min < -550 || alarm_min > 1250 || alarm_max < -550 || alarm_max > 1250 || osdSymbol < 0 || osdSymbol > TEMP_SENSOR_SYM_COUNT || strlen(label) > TEMPERATURE_LABEL_LEN || !addressValid) {
                cliShowParseError();
            } else {
                tempSensorConfig_t *sensorConfig = tempSensorConfigMutable(i);
                sensorConfig->type = type;
                sensorConfig->address = address;
                sensorConfig->alarm_min = alarm_min;
                sensorConfig->alarm_max = alarm_max;
                sensorConfig->osdSymbol = osdSymbol;
                for (uint8_t index = 0; index < TEMPERATURE_LABEL_LEN; ++index) {
                    sensorConfig->label[index] = toupper(label[index]);
                    if (label[index] == '\0') break;
                }
            }
        } else {
            cliShowArgumentRangeError("sensor index", 0, MAX_TEMP_SENSORS - 1);
        }
    }
}
#endif

#if defined(USE_SAFE_HOME)
static void printSafeHomes(uint8_t dumpMask, const navSafeHome_t *navSafeHome, const navSafeHome_t *defaultSafeHome)
{
    const char *format = "safehome %u %u %d %d"; // uint8_t enabled, int32_t lat; int32_t lon
    for (uint8_t i = 0; i < MAX_SAFE_HOMES; i++) {
        bool equalsDefault = false;
        if (defaultSafeHome) {
            equalsDefault = navSafeHome[i].enabled == defaultSafeHome[i].enabled
               && navSafeHome[i].lat == defaultSafeHome[i].lat
               && navSafeHome[i].lon == defaultSafeHome[i].lon;
            cliDefaultPrintLinef(dumpMask, equalsDefault, format, i,
                defaultSafeHome[i].enabled, defaultSafeHome[i].lat, defaultSafeHome[i].lon);
        }
        cliDumpPrintLinef(dumpMask, equalsDefault, format, i,
            navSafeHome[i].enabled, navSafeHome[i].lat, navSafeHome[i].lon);
    }
}

static void cliSafeHomes(char *cmdline)
{
    if (isEmpty(cmdline)) {
        printSafeHomes(DUMP_MASTER, safeHomeConfig(0), NULL);
    } else if (sl_strcasecmp(cmdline, "reset") == 0) {
        resetSafeHomes();
    } else {
        int32_t lat=0, lon=0;
        bool enabled=false;
        uint8_t validArgumentCount = 0;
        const char *ptr = cmdline;
        int8_t i = fastA2I(ptr);
        if (i < 0 || i >= MAX_SAFE_HOMES) {
             cliShowArgumentRangeError("safehome index", 0, MAX_SAFE_HOMES - 1);
        } else {
            if ((ptr = nextArg(ptr))) {
                enabled = fastA2I(ptr);
                validArgumentCount++;
            }
            if ((ptr = nextArg(ptr))) {
                lat = fastA2I(ptr);
                validArgumentCount++;
            }
            if ((ptr = nextArg(ptr))) {
                lon = fastA2I(ptr);
                validArgumentCount++;
            }
            if ((ptr = nextArg(ptr))) {
                // check for too many arguments
                validArgumentCount++;
            }
            if (validArgumentCount != 3) {
                cliShowParseError();
            } else {
                safeHomeConfigMutable(i)->enabled = enabled;
                safeHomeConfigMutable(i)->lat = lat;
                safeHomeConfigMutable(i)->lon = lon;
            }
        }
    }
}

#endif
#if defined(USE_NAV) && defined(NAV_NON_VOLATILE_WAYPOINT_STORAGE) && defined(NAV_NON_VOLATILE_WAYPOINT_CLI)
static void printWaypoints(uint8_t dumpMask, const navWaypoint_t *navWaypoint, const navWaypoint_t *defaultNavWaypoint)
{
    cliPrintLinef("#wp %d %svalid", posControl.waypointCount, posControl.waypointListValid ? "" : "in"); //int8_t bool
    const char *format = "wp %u %u %d %d %d %d %d %d %u"; //uint8_t action; int32_t lat; int32_t lon; int32_t alt; int16_t p1 int16_t p2 int16_t p3; uint8_t flag
    for (uint8_t i = 0; i < NAV_MAX_WAYPOINTS; i++) {
        bool equalsDefault = false;
        if (defaultNavWaypoint) {
            equalsDefault = navWaypoint[i].action == defaultNavWaypoint[i].action
                && navWaypoint[i].lat == defaultNavWaypoint[i].lat
                && navWaypoint[i].lon == defaultNavWaypoint[i].lon
                && navWaypoint[i].alt == defaultNavWaypoint[i].alt
                && navWaypoint[i].p1 == defaultNavWaypoint[i].p1
                && navWaypoint[i].p2 == defaultNavWaypoint[i].p2
                && navWaypoint[i].p3 == defaultNavWaypoint[i].p3
                && navWaypoint[i].flag == defaultNavWaypoint[i].flag;
            cliDefaultPrintLinef(dumpMask, equalsDefault, format,
                i,
                defaultNavWaypoint[i].action,
                defaultNavWaypoint[i].lat,
                defaultNavWaypoint[i].lon,
                defaultNavWaypoint[i].alt,
                defaultNavWaypoint[i].p1,
                defaultNavWaypoint[i].p2,
                defaultNavWaypoint[i].p3,
                defaultNavWaypoint[i].flag
            );
        }
        cliDumpPrintLinef(dumpMask, equalsDefault, format,
            i,
            navWaypoint[i].action,
            navWaypoint[i].lat,
            navWaypoint[i].lon,
            navWaypoint[i].alt,
            navWaypoint[i].p1,
            navWaypoint[i].p2,
            navWaypoint[i].p3,
            navWaypoint[i].flag
        );
    }
}

static void cliWaypoints(char *cmdline)
{
    static int8_t multiMissionWPCounter = 0;

    if (isEmpty(cmdline)) {
        printWaypoints(DUMP_MASTER, posControl.waypointList, NULL);
    } else if (sl_strcasecmp(cmdline, "reset") == 0) {
        resetWaypointList();
    } else if (sl_strcasecmp(cmdline, "load") == 0) {
        loadNonVolatileWaypointList();
    } else if (sl_strcasecmp(cmdline, "save") == 0) {
        posControl.waypointListValid = false;
        for (int i = 0; i < NAV_MAX_WAYPOINTS; i++) {
            if (!(posControl.waypointList[i].action == NAV_WP_ACTION_WAYPOINT || posControl.waypointList[i].action == NAV_WP_ACTION_JUMP || posControl.waypointList[i].action == NAV_WP_ACTION_RTH || posControl.waypointList[i].action == NAV_WP_ACTION_HOLD_TIME || posControl.waypointList[i].action == NAV_WP_ACTION_LAND || posControl.waypointList[i].action == NAV_WP_ACTION_SET_POI || posControl.waypointList[i].action == NAV_WP_ACTION_SET_HEAD)) break;
            if (posControl.waypointList[i].flag == NAV_WP_FLAG_LAST) {
                if (posControl.multiMissionCount == 1) {
                    posControl.waypointCount = i + 1;
                    posControl.waypointListValid = true;
                    multiMissionWPCounter = 0;
                    posControl.multiMissionCount = 0;
                    navConfigMutable()->general.waypoint_multi_mission_index = 1;    // reset selected mission to 1 when new entries loaded
                    break;
                } else {
                    posControl.multiMissionCount -= 1;
                }
            }
        }
        if (posControl.waypointListValid) {
            saveNonVolatileWaypointList();
        } else {
            cliShowParseError();
        }
    } else {
        int16_t i, p1=0,p2=0,p3=0,tmp=0;
        uint8_t action=0, flag=0;
        int32_t lat=0, lon=0, alt=0;
        uint8_t validArgumentCount = 0;
        const char *ptr = cmdline;
        i = fastA2I(ptr);
        if (i + multiMissionWPCounter >= 0 && i + multiMissionWPCounter < NAV_MAX_WAYPOINTS) {
            ptr = nextArg(ptr);
            if (ptr) {
                action = fastA2I(ptr);
                validArgumentCount++;
            }
            ptr = nextArg(ptr);
            if (ptr) {
                lat = fastA2I(ptr);
                validArgumentCount++;
            }
            ptr = nextArg(ptr);
            if (ptr) {
                lon = fastA2I(ptr);
                validArgumentCount++;
            }
            ptr = nextArg(ptr);
            if (ptr) {
                alt = fastA2I(ptr);
                validArgumentCount++;
            }
            ptr = nextArg(ptr);
            if (ptr) {
                p1 = fastA2I(ptr);
                validArgumentCount++;
            }
            ptr = nextArg(ptr);
            if (ptr) {
                tmp = fastA2I(ptr);
                validArgumentCount++;
            }
                /* We support pre-2.5 6 values (... p1,flags) or
                 *  2.5 and later, 8 values (... p1,p2,p3,flags)
                 */
            ptr = nextArg(ptr);
            if (ptr) {
                p2 = tmp;
                p3 = fastA2I(ptr);
                validArgumentCount++;
                ptr = nextArg(ptr);
                 if (ptr) {
                    flag = fastA2I(ptr);
                    validArgumentCount++;
                }
            } else {
                flag = tmp;
            }

            if (!(validArgumentCount == 6 || validArgumentCount == 8)) {
                cliShowParseError();
            } else if (!(action == 0 || action == NAV_WP_ACTION_WAYPOINT || (action == NAV_WP_ACTION_RTH && i > 0) || action == NAV_WP_ACTION_JUMP || action == NAV_WP_ACTION_HOLD_TIME || action == NAV_WP_ACTION_LAND || action == NAV_WP_ACTION_SET_POI || action == NAV_WP_ACTION_SET_HEAD) || !(flag == 0 || flag == NAV_WP_FLAG_LAST || flag == NAV_WP_FLAG_HOME)) {
                cliShowParseError();
            } else {
                if (i + multiMissionWPCounter == 0) {
                    posControl.multiMissionCount = 0;
                }

                posControl.waypointList[i + multiMissionWPCounter].action = action;
                posControl.waypointList[i + multiMissionWPCounter].lat = lat;
                posControl.waypointList[i + multiMissionWPCounter].lon = lon;
                posControl.waypointList[i + multiMissionWPCounter].alt = alt;
                posControl.waypointList[i + multiMissionWPCounter].p1 = p1;
                posControl.waypointList[i + multiMissionWPCounter].p2 = p2;
                posControl.waypointList[i + multiMissionWPCounter].p3 = p3;
                posControl.waypointList[i + multiMissionWPCounter].flag = flag;

                // Process WP entries made up of multiple successive WP missions (multiple NAV_WP_FLAG_LAST entries)
                // Individial missions loaded at runtime, mission selected nav_waypoint_multi_mission_index
                if (flag == NAV_WP_FLAG_LAST) {
                    multiMissionWPCounter += i + 1;
                    posControl.multiMissionCount += 1;
                }
            }
        } else {
            cliShowArgumentRangeError("wp index", 0, NAV_MAX_WAYPOINTS - 1);
        }
    }
}

#endif

#ifdef USE_LED_STRIP
static void printLed(uint8_t dumpMask, const ledConfig_t *ledConfigs, const ledConfig_t *defaultLedConfigs)
{
    const char *format = "led %u %s";
    char ledConfigBuffer[20];
    char ledConfigDefaultBuffer[20];
    for (uint32_t i = 0; i < LED_MAX_STRIP_LENGTH; i++) {
        ledConfig_t ledConfig = ledConfigs[i];
        generateLedConfig(&ledConfig, ledConfigBuffer, sizeof(ledConfigBuffer));
        bool equalsDefault = false;
        if (defaultLedConfigs) {
            ledConfig_t ledConfigDefault = defaultLedConfigs[i];
            equalsDefault = ledConfig == ledConfigDefault;
            generateLedConfig(&ledConfigDefault, ledConfigDefaultBuffer, sizeof(ledConfigDefaultBuffer));
            cliDefaultPrintLinef(dumpMask, equalsDefault, format, i, ledConfigDefaultBuffer);
        }
        cliDumpPrintLinef(dumpMask, equalsDefault, format, i, ledConfigBuffer);
    }
}

static void cliLed(char *cmdline)
{
    int i;
    const char *ptr;

    if (isEmpty(cmdline)) {
        printLed(DUMP_MASTER, ledStripConfig()->ledConfigs, NULL);
    } else {
        ptr = cmdline;
        i = fastA2I(ptr);
        if (i < LED_MAX_STRIP_LENGTH) {
            ptr = nextArg(cmdline);
            if (!parseLedStripConfig(i, ptr)) {
                cliShowParseError();
            }
        } else {
            cliShowArgumentRangeError("index", 0, LED_MAX_STRIP_LENGTH - 1);
        }
    }
}

static void printColor(uint8_t dumpMask, const hsvColor_t *colors, const hsvColor_t *defaultColors)
{
    const char *format = "color %u %d,%u,%u";
    for (uint32_t i = 0; i < LED_CONFIGURABLE_COLOR_COUNT; i++) {
        const hsvColor_t *color = &colors[i];
        bool equalsDefault = false;
        if (defaultColors) {
            const hsvColor_t *colorDefault = &defaultColors[i];
            equalsDefault = color->h == colorDefault->h
                && color->s == colorDefault->s
                && color->v == colorDefault->v;
            cliDefaultPrintLinef(dumpMask, equalsDefault, format, i,colorDefault->h, colorDefault->s, colorDefault->v);
        }
        cliDumpPrintLinef(dumpMask, equalsDefault, format, i, color->h, color->s, color->v);
    }
}

static void cliColor(char *cmdline)
{
    if (isEmpty(cmdline)) {
        printColor(DUMP_MASTER, ledStripConfig()->colors, NULL);
    } else {
        const char *ptr = cmdline;
        const int i = fastA2I(ptr);
        if (i < LED_CONFIGURABLE_COLOR_COUNT) {
            ptr = nextArg(cmdline);
            if (!parseColor(i, ptr)) {
                cliShowParseError();
            }
        } else {
            cliShowArgumentRangeError("index", 0, LED_CONFIGURABLE_COLOR_COUNT - 1);
        }
    }
}

static void printModeColor(uint8_t dumpMask, const ledStripConfig_t *ledStripConfig, const ledStripConfig_t *defaultLedStripConfig)
{
    const char *format = "mode_color %u %u %u";
    for (uint32_t i = 0; i < LED_MODE_COUNT; i++) {
        for (uint32_t j = 0; j < LED_DIRECTION_COUNT; j++) {
            int colorIndex = ledStripConfig->modeColors[i].color[j];
            bool equalsDefault = false;
            if (defaultLedStripConfig) {
                int colorIndexDefault = defaultLedStripConfig->modeColors[i].color[j];
                equalsDefault = colorIndex == colorIndexDefault;
                cliDefaultPrintLinef(dumpMask, equalsDefault, format, i, j, colorIndexDefault);
            }
            cliDumpPrintLinef(dumpMask, equalsDefault, format, i, j, colorIndex);
        }
    }

    for (uint32_t j = 0; j < LED_SPECIAL_COLOR_COUNT; j++) {
        const int colorIndex = ledStripConfig->specialColors.color[j];
        bool equalsDefault = false;
        if (defaultLedStripConfig) {
            const int colorIndexDefault = defaultLedStripConfig->specialColors.color[j];
            equalsDefault = colorIndex == colorIndexDefault;
            cliDefaultPrintLinef(dumpMask, equalsDefault, format, LED_SPECIAL, j, colorIndexDefault);
        }
        cliDumpPrintLinef(dumpMask, equalsDefault, format, LED_SPECIAL, j, colorIndex);
    }
}

static void cliModeColor(char *cmdline)
{
    char * saveptr;

    if (isEmpty(cmdline)) {
        printModeColor(DUMP_MASTER, ledStripConfig(), NULL);
    } else {
        enum {MODE = 0, FUNCTION, COLOR, ARGS_COUNT};
        int args[ARGS_COUNT];
        int argNo = 0;
        const char* ptr = strtok_r(cmdline, " ", &saveptr);
        while (ptr && argNo < ARGS_COUNT) {
            args[argNo++] = fastA2I(ptr);
            ptr = strtok_r(NULL, " ", &saveptr);
        }

        if (ptr != NULL || argNo != ARGS_COUNT) {
            cliShowParseError();
            return;
        }

        int modeIdx  = args[MODE];
        int funIdx = args[FUNCTION];
        int color = args[COLOR];
        if (!setModeColor(modeIdx, funIdx, color)) {
            cliShowParseError();
            return;
        }
        // values are validated
        cliPrintLinef("mode_color %u %u %u", modeIdx, funIdx, color);
    }
}
#endif

static void printServo(uint8_t dumpMask, const servoParam_t *servoParam, const servoParam_t *defaultServoParam)
{
    // print out servo settings
    const char *format = "servo %u %d %d %d %d";
    for (uint32_t i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        const servoParam_t *servoConf = &servoParam[i];
        bool equalsDefault = false;
        if (defaultServoParam) {
            const servoParam_t *servoConfDefault = &defaultServoParam[i];
            equalsDefault = servoConf->min == servoConfDefault->min
                && servoConf->max == servoConfDefault->max
                && servoConf->middle == servoConfDefault->middle
                && servoConf->rate == servoConfDefault->rate;
            cliDefaultPrintLinef(dumpMask, equalsDefault, format,
                i,
                servoConfDefault->min,
                servoConfDefault->max,
                servoConfDefault->middle,
                servoConfDefault->rate
            );
        }
        cliDumpPrintLinef(dumpMask, equalsDefault, format,
            i,
            servoConf->min,
            servoConf->max,
            servoConf->middle,
            servoConf->rate
        );
    }
}

static void cliServo(char *cmdline)
{
    enum { SERVO_ARGUMENT_COUNT = 5 };
    int16_t arguments[SERVO_ARGUMENT_COUNT];

    servoParam_t *servo;

    int i;
    const char *ptr;

    if (isEmpty(cmdline)) {
        printServo(DUMP_MASTER, servoParams(0), NULL);
    } else {
        int validArgumentCount = 0;

        ptr = cmdline;

        // Command line is integers (possibly negative) separated by spaces, no other characters allowed.

        // If command line doesn't fit the format, don't modify the config
        while (*ptr) {
            if (*ptr == '-' || (*ptr >= '0' && *ptr <= '9')) {
                if (validArgumentCount >= SERVO_ARGUMENT_COUNT) {
                    cliShowParseError();
                    return;
                }

                arguments[validArgumentCount++] = fastA2I(ptr);

                do {
                    ptr++;
                } while (*ptr >= '0' && *ptr <= '9');
            } else if (*ptr == ' ') {
                ptr++;
            } else {
                cliShowParseError();
                return;
            }
        }

        enum {INDEX = 0, MIN, MAX, MIDDLE, RATE};

        i = arguments[INDEX];

        // Check we got the right number of args and the servo index is correct (don't validate the other values)
        if (validArgumentCount != SERVO_ARGUMENT_COUNT || i < 0 || i >= MAX_SUPPORTED_SERVOS) {
            cliShowParseError();
            return;
        }

        servo = servoParamsMutable(i);

        if (
            arguments[MIN] < SERVO_OUTPUT_MIN || arguments[MIN] > SERVO_OUTPUT_MAX ||
            arguments[MAX] < SERVO_OUTPUT_MIN || arguments[MAX] > SERVO_OUTPUT_MAX ||
            arguments[MIDDLE] < arguments[MIN] || arguments[MIDDLE] > arguments[MAX] ||
            arguments[MIN] > arguments[MAX] || arguments[MAX] < arguments[MIN] ||
            arguments[RATE] < -125 || arguments[RATE] > 125
        ) {
            cliShowParseError();
            return;
        }

        servo->min = arguments[MIN];
        servo->max = arguments[MAX];
        servo->middle = arguments[MIDDLE];
        servo->rate = arguments[RATE];
    }
}

static void printServoMix(uint8_t dumpMask, const servoMixer_t *customServoMixers, const servoMixer_t *defaultCustomServoMixers)
{
    const char *format = "smix %d %d %d %d %d %d";
    for (uint32_t i = 0; i < MAX_SERVO_RULES; i++) {
        const servoMixer_t customServoMixer = customServoMixers[i];
        if (customServoMixer.rate == 0) {
            break;
        }

        bool equalsDefault = false;
        if (defaultCustomServoMixers) {
            servoMixer_t customServoMixerDefault = defaultCustomServoMixers[i];
            equalsDefault = customServoMixer.targetChannel == customServoMixerDefault.targetChannel
                && customServoMixer.inputSource == customServoMixerDefault.inputSource
                && customServoMixer.rate == customServoMixerDefault.rate
                && customServoMixer.speed == customServoMixerDefault.speed
            #ifdef USE_PROGRAMMING_FRAMEWORK
                && customServoMixer.conditionId == customServoMixerDefault.conditionId
            #endif
            ;

            cliDefaultPrintLinef(dumpMask, equalsDefault, format,
                i,
                customServoMixerDefault.targetChannel,
                customServoMixerDefault.inputSource,
                customServoMixerDefault.rate,
                customServoMixerDefault.speed,
            #ifdef USE_PROGRAMMING_FRAMEWORK
                customServoMixer.conditionId
            #else
                0
            #endif
            );
        }
        cliDumpPrintLinef(dumpMask, equalsDefault, format,
            i,
            customServoMixer.targetChannel,
            customServoMixer.inputSource,
            customServoMixer.rate,
            customServoMixer.speed,
        #ifdef USE_PROGRAMMING_FRAMEWORK
            customServoMixer.conditionId
        #else
            0
        #endif
        );
    }
}

static void cliServoMix(char *cmdline)
{
    char * saveptr;
    int args[6], check = 0;
    uint8_t len = strlen(cmdline);

    if (len == 0) {
        printServoMix(DUMP_MASTER, customServoMixers(0), NULL);
    } else if (sl_strncasecmp(cmdline, "reset", 5) == 0) {
        // erase custom mixer
        pgResetCopy(customServoMixersMutable(0), PG_SERVO_MIXER);
    } else {
        enum {RULE = 0, TARGET, INPUT, RATE, SPEED, CONDITION, ARGS_COUNT};
        char *ptr = strtok_r(cmdline, " ", &saveptr);
        args[CONDITION] = -1;
        while (ptr != NULL && check < ARGS_COUNT) {
            args[check++] = fastA2I(ptr);
            ptr = strtok_r(NULL, " ", &saveptr);
        }

        if (ptr != NULL || (check < ARGS_COUNT - 1)) {
            cliShowParseError();
            return;
        }

        int32_t i = args[RULE];
        if (
            i >= 0 && i < MAX_SERVO_RULES &&
            args[TARGET] >= 0 && args[TARGET] < MAX_SUPPORTED_SERVOS &&
            args[INPUT] >= 0 && args[INPUT] < INPUT_SOURCE_COUNT &&
            args[RATE] >= -1000 && args[RATE] <= 1000 &&
            args[SPEED] >= 0 && args[SPEED] <= MAX_SERVO_SPEED &&
            args[CONDITION] >= -1 && args[CONDITION] < MAX_LOGIC_CONDITIONS
        ) {
            customServoMixersMutable(i)->targetChannel = args[TARGET];
            customServoMixersMutable(i)->inputSource = args[INPUT];
            customServoMixersMutable(i)->rate = args[RATE];
            customServoMixersMutable(i)->speed = args[SPEED];
        #ifdef USE_PROGRAMMING_FRAMEWORK
            customServoMixersMutable(i)->conditionId = args[CONDITION];
        #endif
            cliServoMix("");
        } else {
            cliShowParseError();
        }
    }
}

#ifdef USE_PROGRAMMING_FRAMEWORK

static void printLogic(uint8_t dumpMask, const logicCondition_t *logicConditions, const logicCondition_t *defaultLogicConditions)
{
    const char *format = "logic %d %d %d %d %d %d %d %d %d";
    for (uint32_t i = 0; i < MAX_LOGIC_CONDITIONS; i++) {
        const logicCondition_t logic = logicConditions[i];

        bool equalsDefault = false;
        if (defaultLogicConditions) {
            logicCondition_t defaultValue = defaultLogicConditions[i];
            equalsDefault =
                logic.enabled == defaultValue.enabled &&
                logic.activatorId == defaultValue.activatorId &&
                logic.operation == defaultValue.operation &&
                logic.operandA.type == defaultValue.operandA.type &&
                logic.operandA.value == defaultValue.operandA.value &&
                logic.operandB.type == defaultValue.operandB.type &&
                logic.operandB.value == defaultValue.operandB.value &&
                logic.flags == defaultValue.flags;

            cliDefaultPrintLinef(dumpMask, equalsDefault, format,
                i,
                logic.enabled,
                logic.activatorId,
                logic.operation,
                logic.operandA.type,
                logic.operandA.value,
                logic.operandB.type,
                logic.operandB.value,
                logic.flags
            );
        }
        cliDumpPrintLinef(dumpMask, equalsDefault, format,
            i,
            logic.enabled,
            logic.activatorId,
            logic.operation,
            logic.operandA.type,
            logic.operandA.value,
            logic.operandB.type,
            logic.operandB.value,
            logic.flags
        );
    }
}

static void cliLogic(char *cmdline) {
    char * saveptr;
    int args[9], check = 0;
    uint8_t len = strlen(cmdline);

    if (len == 0) {
        printLogic(DUMP_MASTER, logicConditions(0), NULL);
    } else if (sl_strncasecmp(cmdline, "reset", 5) == 0) {
        pgResetCopy(logicConditionsMutable(0), PG_LOGIC_CONDITIONS);
    } else {
        enum {
            INDEX = 0,
            ENABLED,
            ACTIVATOR_ID,
            OPERATION,
            OPERAND_A_TYPE,
            OPERAND_A_VALUE,
            OPERAND_B_TYPE,
            OPERAND_B_VALUE,
            FLAGS,
            ARGS_COUNT
            };
        char *ptr = strtok_r(cmdline, " ", &saveptr);
        while (ptr != NULL && check < ARGS_COUNT) {
            args[check++] = fastA2I(ptr);
            ptr = strtok_r(NULL, " ", &saveptr);
        }

        if (ptr != NULL || check != ARGS_COUNT) {
            cliShowParseError();
            return;
        }

        int32_t i = args[INDEX];
        if (
            i >= 0 && i < MAX_LOGIC_CONDITIONS &&
            args[ENABLED] >= 0 && args[ENABLED] <= 1 &&
            args[ACTIVATOR_ID] >= -1 && args[ACTIVATOR_ID] < MAX_LOGIC_CONDITIONS &&
            args[OPERATION] >= 0 && args[OPERATION] < LOGIC_CONDITION_LAST &&
            args[OPERAND_A_TYPE] >= 0 && args[OPERAND_A_TYPE] < LOGIC_CONDITION_OPERAND_TYPE_LAST &&
            args[OPERAND_A_VALUE] >= -1000000 && args[OPERAND_A_VALUE] <= 1000000 &&
            args[OPERAND_B_TYPE] >= 0 && args[OPERAND_B_TYPE] < LOGIC_CONDITION_OPERAND_TYPE_LAST &&
            args[OPERAND_B_VALUE] >= -1000000 && args[OPERAND_B_VALUE] <= 1000000 &&
            args[FLAGS] >= 0 && args[FLAGS] <= 255

        ) {
            logicConditionsMutable(i)->enabled = args[ENABLED];
            logicConditionsMutable(i)->activatorId = args[ACTIVATOR_ID];
            logicConditionsMutable(i)->operation = args[OPERATION];
            logicConditionsMutable(i)->operandA.type = args[OPERAND_A_TYPE];
            logicConditionsMutable(i)->operandA.value = args[OPERAND_A_VALUE];
            logicConditionsMutable(i)->operandB.type = args[OPERAND_B_TYPE];
            logicConditionsMutable(i)->operandB.value = args[OPERAND_B_VALUE];
            logicConditionsMutable(i)->flags = args[FLAGS];

            cliLogic("");
        } else {
            cliShowParseError();
        }
    }
}

static void printGvar(uint8_t dumpMask, const globalVariableConfig_t *gvars, const globalVariableConfig_t *defaultGvars)
{
    const char *format = "gvar %d %d %d %d";
    for (uint32_t i = 0; i < MAX_GLOBAL_VARIABLES; i++) {
        const globalVariableConfig_t gvar = gvars[i];

        bool equalsDefault = false;
        if (defaultGvars) {
            globalVariableConfig_t defaultValue = defaultGvars[i];
            equalsDefault =
                gvar.defaultValue == defaultValue.defaultValue &&
                gvar.min == defaultValue.min &&
                gvar.max == defaultValue.max;

            cliDefaultPrintLinef(dumpMask, equalsDefault, format,
                i,
                gvar.defaultValue,
                gvar.min,
                gvar.max
            );
        }
        cliDumpPrintLinef(dumpMask, equalsDefault, format,
            i,
            gvar.defaultValue,
            gvar.min,
            gvar.max
        );
    }
}

static void cliGvar(char *cmdline) {
    char * saveptr;
    int args[4], check = 0;
    uint8_t len = strlen(cmdline);

    if (len == 0) {
        printGvar(DUMP_MASTER, globalVariableConfigs(0), NULL);
    } else if (sl_strncasecmp(cmdline, "reset", 5) == 0) {
        pgResetCopy(globalVariableConfigsMutable(0), PG_GLOBAL_VARIABLE_CONFIG);
    } else {
        enum {
            INDEX = 0,
            DEFAULT,
            MIN,
            MAX,
            ARGS_COUNT
            };
        char *ptr = strtok_r(cmdline, " ", &saveptr);
        while (ptr != NULL && check < ARGS_COUNT) {
            args[check++] = fastA2I(ptr);
            ptr = strtok_r(NULL, " ", &saveptr);
        }

        if (ptr != NULL || check != ARGS_COUNT) {
            cliShowParseError();
            return;
        }

        int32_t i = args[INDEX];
        if (
            i >= 0 && i < MAX_GLOBAL_VARIABLES &&
            args[DEFAULT] >= INT32_MIN && args[DEFAULT] <= INT32_MAX &&
            args[MIN] >= INT32_MIN && args[MIN] <= INT32_MAX &&
            args[MAX] >= INT32_MIN && args[MAX] <= INT32_MAX
        ) {
            globalVariableConfigsMutable(i)->defaultValue = args[DEFAULT];
            globalVariableConfigsMutable(i)->min = args[MIN];
            globalVariableConfigsMutable(i)->max = args[MAX];

            cliGvar("");
        } else {
            cliShowParseError();
        }
    }
}

static void printPid(uint8_t dumpMask, const programmingPid_t *programmingPids, const programmingPid_t *defaultProgrammingPids)
{
    const char *format = "pid %d %d %d %d %d %d %d %d %d %d";
    for (uint32_t i = 0; i < MAX_PROGRAMMING_PID_COUNT; i++) {
        const programmingPid_t pid = programmingPids[i];

        bool equalsDefault = false;
        if (defaultProgrammingPids) {
            programmingPid_t defaultValue = defaultProgrammingPids[i];
            equalsDefault =
                pid.enabled == defaultValue.enabled &&
                pid.setpoint.type == defaultValue.setpoint.type &&
                pid.setpoint.value == defaultValue.setpoint.value &&
                pid.measurement.type == defaultValue.measurement.type &&
                pid.measurement.value == defaultValue.measurement.value &&
                pid.gains.P == defaultValue.gains.P &&
                pid.gains.I == defaultValue.gains.I &&
                pid.gains.D == defaultValue.gains.D &&
                pid.gains.FF == defaultValue.gains.FF;

            cliDefaultPrintLinef(dumpMask, equalsDefault, format,
                i,
                pid.enabled,
                pid.setpoint.type,
                pid.setpoint.value,
                pid.measurement.type,
                pid.measurement.value,
                pid.gains.P,
                pid.gains.I,
                pid.gains.D,
                pid.gains.FF
            );
        }
        cliDumpPrintLinef(dumpMask, equalsDefault, format,
            i,
            pid.enabled,
            pid.setpoint.type,
            pid.setpoint.value,
            pid.measurement.type,
            pid.measurement.value,
            pid.gains.P,
            pid.gains.I,
            pid.gains.D,
            pid.gains.FF
        );
    }
}

static void cliPid(char *cmdline) {
    char * saveptr;
    int args[10], check = 0;
    uint8_t len = strlen(cmdline);

    if (len == 0) {
        printPid(DUMP_MASTER, programmingPids(0), NULL);
    } else if (sl_strncasecmp(cmdline, "reset", 5) == 0) {
        pgResetCopy(programmingPidsMutable(0), PG_LOGIC_CONDITIONS);
    } else {
        enum {
            INDEX = 0,
            ENABLED,
            SETPOINT_TYPE,
            SETPOINT_VALUE,
            MEASUREMENT_TYPE,
            MEASUREMENT_VALUE,
            P_GAIN,
            I_GAIN,
            D_GAIN,
            FF_GAIN,
            ARGS_COUNT
            };
        char *ptr = strtok_r(cmdline, " ", &saveptr);
        while (ptr != NULL && check < ARGS_COUNT) {
            args[check++] = fastA2I(ptr);
            ptr = strtok_r(NULL, " ", &saveptr);
        }

        if (ptr != NULL || check != ARGS_COUNT) {
            cliShowParseError();
            return;
        }

        int32_t i = args[INDEX];
        if (
            i >= 0 && i < MAX_PROGRAMMING_PID_COUNT &&
            args[ENABLED] >= 0 && args[ENABLED] <= 1 &&
            args[SETPOINT_TYPE] >= 0 && args[SETPOINT_TYPE] < LOGIC_CONDITION_OPERAND_TYPE_LAST &&
            args[SETPOINT_VALUE] >= -1000000 && args[SETPOINT_VALUE] <= 1000000 &&
            args[MEASUREMENT_TYPE] >= 0 && args[MEASUREMENT_TYPE] < LOGIC_CONDITION_OPERAND_TYPE_LAST &&
            args[MEASUREMENT_VALUE] >= -1000000 && args[MEASUREMENT_VALUE] <= 1000000 &&
            args[P_GAIN] >= 0 && args[P_GAIN] <= INT16_MAX &&
            args[I_GAIN] >= 0 && args[I_GAIN] <= INT16_MAX &&
            args[D_GAIN] >= 0 && args[D_GAIN] <= INT16_MAX &&
            args[FF_GAIN] >= 0 && args[FF_GAIN] <= INT16_MAX
        ) {
            programmingPidsMutable(i)->enabled = args[ENABLED];
            programmingPidsMutable(i)->setpoint.type = args[SETPOINT_TYPE];
            programmingPidsMutable(i)->setpoint.value = args[SETPOINT_VALUE];
            programmingPidsMutable(i)->measurement.type = args[MEASUREMENT_TYPE];
            programmingPidsMutable(i)->measurement.value = args[MEASUREMENT_VALUE];
            programmingPidsMutable(i)->gains.P = args[P_GAIN];
            programmingPidsMutable(i)->gains.I = args[I_GAIN];
            programmingPidsMutable(i)->gains.D = args[D_GAIN];
            programmingPidsMutable(i)->gains.FF = args[FF_GAIN];

            cliPid("");
        } else {
            cliShowParseError();
        }
    }
}

#endif

#ifdef USE_SDCARD

static void cliWriteBytes(const uint8_t *buffer, int count)
{
    while (count > 0) {
        cliWrite(*buffer);
        buffer++;
        count--;
    }
}

static void cliSdInfo(char *cmdline)
{
    UNUSED(cmdline);

    cliPrint("SD card: ");

    if (!sdcard_isInserted()) {
        cliPrintLine("None inserted");
        return;
    }

    if (!sdcard_isInitialized()) {
        cliPrintLine("Startup failed");
        return;
    }

    const sdcardMetadata_t *metadata = sdcard_getMetadata();

    cliPrintf("Manufacturer 0x%x, %ukB, %02d/%04d, v%d.%d, '",
        metadata->manufacturerID,
        metadata->numBlocks / 2, /* One block is half a kB */
        metadata->productionMonth,
        metadata->productionYear,
        metadata->productRevisionMajor,
        metadata->productRevisionMinor
    );

    cliWriteBytes((uint8_t*)metadata->productName, sizeof(metadata->productName));

    cliPrint("'\r\n" "Filesystem: ");

    switch (afatfs_getFilesystemState()) {
        case AFATFS_FILESYSTEM_STATE_READY:
            cliPrint("Ready");
        break;
        case AFATFS_FILESYSTEM_STATE_INITIALIZATION:
            cliPrint("Initializing");
        break;
        case AFATFS_FILESYSTEM_STATE_UNKNOWN:
        case AFATFS_FILESYSTEM_STATE_FATAL:
            cliPrint("Fatal");

            switch (afatfs_getLastError()) {
                case AFATFS_ERROR_BAD_MBR:
                    cliPrint(" - no FAT MBR partitions");
                break;
                case AFATFS_ERROR_BAD_FILESYSTEM_HEADER:
                    cliPrint(" - bad FAT header");
                break;
                case AFATFS_ERROR_GENERIC:
                case AFATFS_ERROR_NONE:
                    ; // Nothing more detailed to print
                break;
            }
        break;
    }
    cliPrintLinefeed();
}

#endif

#ifdef USE_FLASHFS

static void cliFlashInfo(char *cmdline)
{
    UNUSED(cmdline);

    const flashGeometry_t *layout = flashGetGeometry();

    cliPrintLinef("Flash sectors=%u, sectorSize=%u, pagesPerSector=%u, pageSize=%u, totalSize=%u",
            layout->sectors, layout->sectorSize, layout->pagesPerSector, layout->pageSize, layout->totalSize);

    for (uint8_t index = 0; index < FLASH_MAX_PARTITIONS; index++) {
        const flashPartition_t *partition;
        if (index == 0) {
            cliPrintLine("Paritions:");
        }
        partition = flashPartitionFindByIndex(index);
        if (!partition) {
            break;
        }
        cliPrintLinef("  %d: %s %u %u", index, flashPartitionGetTypeName(partition->type), partition->startSector, partition->endSector);
    }
#ifdef USE_FLASHFS
    const flashPartition_t *flashPartition = flashPartitionFindByType(FLASH_PARTITION_TYPE_FLASHFS);

    cliPrintLinef("FlashFS size=%u, usedSize=%u",
            FLASH_PARTITION_SECTOR_COUNT(flashPartition) * layout->sectorSize,
            flashfsGetOffset()
    );
#endif
}

static void cliFlashErase(char *cmdline)
{
    UNUSED(cmdline);

    cliPrintLine("Erasing...");
    flashfsEraseCompletely();

    while (!flashIsReady()) {
        delay(100);
    }

    cliPrintLine("Done.");
}

#ifdef USE_FLASH_TOOLS

static void cliFlashWrite(char *cmdline)
{
    const uint32_t address = fastA2I(cmdline);
    const char *text = strchr(cmdline, ' ');

    if (!text) {
        cliShowParseError();
    } else {
        flashfsSeekAbs(address);
        flashfsWrite((uint8_t*)text, strlen(text), true);
        flashfsFlushSync();

        cliPrintLinef("Wrote %u bytes at %u.", strlen(text), address);
    }
}

static void cliFlashRead(char *cmdline)
{
    uint32_t address = fastA2I(cmdline);

    const char *nextArg = strchr(cmdline, ' ');

    if (!nextArg) {
        cliShowParseError();
    } else {
        uint32_t length = fastA2I(nextArg);

        cliPrintLinef("Reading %u bytes at %u:", length, address);

        uint8_t buffer[32];
        while (length > 0) {
            int bytesRead = flashfsReadAbs(address, buffer, length < sizeof(buffer) ? length : sizeof(buffer));

            for (int i = 0; i < bytesRead; i++) {
                cliWrite(buffer[i]);
            }

            length -= bytesRead;
            address += bytesRead;

            if (bytesRead == 0) {
                //Assume we reached the end of the volume or something fatal happened
                break;
            }
        }
        cliPrintLinefeed();
    }
}

#endif
#endif

#ifdef USE_OSD
static void printOsdLayout(uint8_t dumpMask, const osdLayoutsConfig_t *config, const osdLayoutsConfig_t *configDefault, int layout, int item)
{
    // "<layout> <item> <col> <row> <visible>"
    const char *format = "osd_layout %d %d %d %d %c";
    for (int ii = 0; ii < OSD_LAYOUT_COUNT; ii++) {
        if (layout >= 0 && layout != ii) {
            continue;
        }
        const uint16_t *layoutItems = config->item_pos[ii];
        const uint16_t *defaultLayoutItems = configDefault->item_pos[ii];
        for (int jj = 0; jj < OSD_ITEM_COUNT; jj++) {
            if (item >= 0 && item != jj) {
                continue;
            }
            bool equalsDefault = layoutItems[jj] == defaultLayoutItems[jj];
            cliDefaultPrintLinef(dumpMask, equalsDefault, format,
                ii, jj,
                OSD_X(defaultLayoutItems[jj]),
                OSD_Y(defaultLayoutItems[jj]),
                OSD_VISIBLE(defaultLayoutItems[jj]) ? 'V' : 'H');

            cliDumpPrintLinef(dumpMask, equalsDefault, format,
                ii, jj,
                OSD_X(layoutItems[jj]),
                OSD_Y(layoutItems[jj]),
                OSD_VISIBLE(layoutItems[jj]) ? 'V' : 'H');
        }
    }
}

static void cliOsdLayout(char *cmdline)
{
    char * saveptr;

    int layout = -1;
    int item = -1;
    int col = 0;
    int row = 0;
    bool visible = false;
    char *tok = strtok_r(cmdline, " ", &saveptr);

    int ii;

    for (ii = 0; tok != NULL; ii++, tok = strtok_r(NULL, " ", &saveptr)) {
        switch (ii) {
            case 0:
                layout = fastA2I(tok);
                if (layout < 0 || layout >= OSD_LAYOUT_COUNT) {
                    cliShowParseError();
                    return;
                }
                break;
            case 1:
                item = fastA2I(tok);
                if (item < 0 || item >= OSD_ITEM_COUNT) {
                    cliShowParseError();
                    return;
                }
                break;
            case 2:
                col = fastA2I(tok);
                if (col < 0 || col > OSD_X(OSD_POS_MAX)) {
                    cliShowParseError();
                    return;
                }
                break;
            case 3:
                row = fastA2I(tok);
                if (row < 0 || row > OSD_Y(OSD_POS_MAX)) {
                    cliShowParseError();
                    return;
                }
                break;
            case 4:
                switch (*tok) {
                    case 'H':
                        visible = false;
                        break;
                    case 'V':
                        visible = true;
                        break;
                    default:
                        cliShowParseError();
                        return;
                }
                break;
            default:
                cliShowParseError();
                return;
        }
    }

    switch (ii) {
        case 0:
            FALLTHROUGH;
        case 1:
            FALLTHROUGH;
        case 2:
            // No args, or just layout or layout and item. If any of them not provided,
            // it will be the -1 that we used during initialization, so printOsdLayout()
            // won't use them for filtering.
            printOsdLayout(DUMP_MASTER, osdLayoutsConfig(), osdLayoutsConfig(), layout, item);
            break;
        case 4:
            // No visibility provided. Keep the previous one.
            visible = OSD_VISIBLE(osdLayoutsConfig()->item_pos[layout][item]);
            FALLTHROUGH;
        case 5:
            // Layout, item, pos and visibility. Set the item.
            osdLayoutsConfigMutable()->item_pos[layout][item] = OSD_POS(col, row) | (visible ? OSD_VISIBLE_FLAG : 0);
            break;
        default:
            // Unhandled
            cliShowParseError();
            return;
    }
}

#endif

static void printFeature(uint8_t dumpMask, const featureConfig_t *featureConfig, const featureConfig_t *featureConfigDefault)
{
    uint32_t mask = featureConfig->enabledFeatures;
    uint32_t defaultMask = featureConfigDefault->enabledFeatures;
    for (uint32_t i = 0; ; i++) { // disable all feature first
        if (featureNames[i] == NULL)
            break;
        if (featureNames[i][0] == '\0')
            continue;
        const char *format = "feature -%s";
        cliDefaultPrintLinef(dumpMask, (defaultMask | ~mask) & (1 << i), format, featureNames[i]);
        cliDumpPrintLinef(dumpMask, (~defaultMask | mask) & (1 << i), format, featureNames[i]);
    }
    for (uint32_t i = 0; ; i++) {  // reenable what we want.
        if (featureNames[i] == NULL)
            break;
        if (featureNames[i][0] == '\0')
            continue;
        const char *format = "feature %s";
        if (defaultMask & (1 << i)) {
            cliDefaultPrintLinef(dumpMask, (~defaultMask | mask) & (1 << i), format, featureNames[i]);
        }
        if (mask & (1 << i)) {
            cliDumpPrintLinef(dumpMask, (defaultMask | ~mask) & (1 << i), format, featureNames[i]);
        }
    }
}

static void cliFeature(char *cmdline)
{
    uint32_t len = strlen(cmdline);
    uint32_t mask = featureMask();

    if (len == 0) {
        cliPrint("Enabled: ");
        for (uint32_t i = 0; ; i++) {
            if (featureNames[i] == NULL)
                break;
            if (featureNames[i][0] == '\0')
                continue;
            if (mask & (1 << i))
                cliPrintf("%s ", featureNames[i]);
        }
        cliPrintLinefeed();
    } else if (sl_strncasecmp(cmdline, "list", len) == 0) {
        cliPrint("Available: ");
        for (uint32_t i = 0; ; i++) {
            if (featureNames[i] == NULL)
                break;
            if (featureNames[i][0] == '\0')
                continue;
            cliPrintf("%s ", featureNames[i]);
        }
        cliPrintLinefeed();
        return;
    } else {
        bool remove = false;
        if (cmdline[0] == '-') {
            // remove feature
            remove = true;
            cmdline++; // skip over -
            len--;
        }

        for (uint32_t i = 0; ; i++) {
            if (featureNames[i] == NULL) {
                cliPrintErrorLine("Invalid name");
                break;
            }

            if (sl_strncasecmp(cmdline, featureNames[i], len) == 0) {

                mask = 1 << i;
#ifndef USE_GPS
                if (mask & FEATURE_GPS) {
                    cliPrintErrorLine("unavailable");
                    break;
                }
#endif
                if (remove) {
                    featureClear(mask);
                    cliPrint("Disabled");
                } else {
                    featureSet(mask);
                    cliPrint("Enabled");
                }
                cliPrintLinef(" %s", featureNames[i]);
                break;
            }
        }
    }
}

#if defined(BEEPER) || defined(USE_DSHOT)
static void printBeeper(uint8_t dumpMask, const beeperConfig_t *beeperConfig, const beeperConfig_t *beeperConfigDefault)
{
    const uint8_t beeperCount = beeperTableEntryCount();
    const uint32_t mask = beeperConfig->beeper_off_flags;
    const uint32_t defaultMask = beeperConfigDefault->beeper_off_flags;
    for (int i = 0; i < beeperCount - 2; i++) {
        const char *formatOff = "beeper -%s";
        const char *formatOn = "beeper %s";
        cliDefaultPrintLinef(dumpMask, ~(mask ^ defaultMask) & (1 << i), mask & (1 << i) ? formatOn : formatOff, beeperNameForTableIndex(i));
        cliDumpPrintLinef(dumpMask, ~(mask ^ defaultMask) & (1 << i), mask & (1 << i) ? formatOff : formatOn, beeperNameForTableIndex(i));
    }
}

static void cliBeeper(char *cmdline)
{
    uint32_t len = strlen(cmdline);
    uint8_t beeperCount = beeperTableEntryCount();
    uint32_t mask = getBeeperOffMask();

    if (len == 0) {
        cliPrintf("Disabled:");
        for (int32_t i = 0; ; i++) {
            if (i == beeperCount - 2){
                if (mask == 0)
                    cliPrint("  none");
                break;
            }
            if (mask & (1 << (beeperModeForTableIndex(i) - 1)))
                cliPrintf("  %s", beeperNameForTableIndex(i));
        }
        cliPrintLinefeed();
    } else if (sl_strncasecmp(cmdline, "list", len) == 0) {
        cliPrint("Available:");
        for (uint32_t i = 0; i < beeperCount; i++)
            cliPrintf("  %s", beeperNameForTableIndex(i));
        cliPrintLinefeed();
        return;
    } else {
        bool remove = false;
        if (cmdline[0] == '-') {
            remove = true;     // this is for beeper OFF condition
            cmdline++;
            len--;
        }

        for (uint32_t i = 0; ; i++) {
            if (i == beeperCount) {
                cliPrintErrorLine("Invalid name");
                break;
            }
            if (sl_strncasecmp(cmdline, beeperNameForTableIndex(i), len) == 0) {
                if (remove) { // beeper off
                    if (i == BEEPER_ALL-1)
                        beeperOffSetAll(beeperCount-2);
                    else
                        if (i == BEEPER_PREFERENCE-1)
                            setBeeperOffMask(getPreferredBeeperOffMask());
                        else {
                            mask = 1 << (beeperModeForTableIndex(i) - 1);
                            beeperOffSet(mask);
                        }
                    cliPrint("Disabled");
                }
                else { // beeper on
                    if (i == BEEPER_ALL-1)
                        beeperOffClearAll();
                    else
                        if (i == BEEPER_PREFERENCE-1)
                            setPreferredBeeperOffMask(getBeeperOffMask());
                        else {
                            mask = 1 << (beeperModeForTableIndex(i) - 1);
                            beeperOffClear(mask);
                        }
                    cliPrint("Enabled");
                }
            cliPrintLinef(" %s", beeperNameForTableIndex(i));
            break;
            }
        }
    }
}
#endif

static void printMap(uint8_t dumpMask, const rxConfig_t *rxConfig, const rxConfig_t *defaultRxConfig)
{
    bool equalsDefault = true;
    char buf[16];
    char bufDefault[16];
    uint32_t i;

    for (i = 0; i < MAX_MAPPABLE_RX_INPUTS; i++) {
        buf[i] = bufDefault[i] = 0;
    }

    for (i = 0; i < MAX_MAPPABLE_RX_INPUTS; i++) {
        buf[rxConfig->rcmap[i]] = rcChannelLetters[i];
        if (defaultRxConfig) {
            bufDefault[defaultRxConfig->rcmap[i]] = rcChannelLetters[i];
            equalsDefault = equalsDefault && (rxConfig->rcmap[i] == defaultRxConfig->rcmap[i]);
        }
    }
    buf[i] = '\0';

    const char *formatMap = "map %s";
    cliDefaultPrintLinef(dumpMask, equalsDefault, formatMap, bufDefault);
    cliDumpPrintLinef(dumpMask, equalsDefault, formatMap, buf);
}

static void cliMap(char *cmdline)
{
    uint32_t len;
    char out[5];

    len = strlen(cmdline);

    if (len == 4) {
        // uppercase it
        for (uint32_t i = 0; i < 4; i++)
            cmdline[i] = sl_toupper((unsigned char)cmdline[i]);
        for (uint32_t i = 0; i < 4; i++) {
            if (strchr(rcChannelLetters, cmdline[i]) && !strchr(cmdline + i + 1, cmdline[i]))
                continue;
            cliShowParseError();
            return;
        }
        parseRcChannels(cmdline);
    } else if (len != 0)
        cliShowParseError();
    cliPrint("Map: ");
    uint32_t i;
    for (i = 0; i < 4; i++)
        out[rxConfig()->rcmap[i]] = rcChannelLetters[i];
    out[i] = '\0';
    cliPrintLinef("%s", out);
}

static const char *checkCommand(const char *cmdLine, const char *command)
{
    if (!sl_strncasecmp(cmdLine, command, strlen(command))   // command names match
        && !sl_isalnum((unsigned)cmdLine[strlen(command)])) {   // next characted in bufffer is not alphanumeric (command is correctly terminated)
        return cmdLine + strlen(command) + 1;
    } else {
        return 0;
    }
}

static void cliRebootEx(bool bootLoader)
{
    cliPrint("\r\nRebooting");
    bufWriterFlush(cliWriter);
    waitForSerialPortToFinishTransmitting(cliPort);

    fcReboot(bootLoader);
}

static void cliReboot(void)
{
    cliRebootEx(false);
}

static void cliDfu(char *cmdline)
{
    UNUSED(cmdline);
#ifndef CLI_MINIMAL_VERBOSITY
    cliPrint("\r\nRestarting in DFU mode");
#endif
    cliRebootEx(true);
}

#ifdef USE_RX_ELERES
static void cliEleresBind(char *cmdline)
{
    UNUSED(cmdline);

    if (!(rxConfig()->receiverType == RX_TYPE_SPI && rxConfig()->rx_spi_protocol == RFM22_ELERES)) {
        cliPrintLine("Eleres not active. Please enable feature ELERES and restart IMU");
        return;
    }

    cliPrintLine("Waiting for correct bind signature....");
    bufWriterFlush(cliWriter);
    if (eleresBind()) {
        cliPrintLine("Bind timeout!");
    } else {
        cliPrintLine("Bind OK!\r\nPlease restart your transmitter.");
    }
}
#endif // USE_RX_ELERES

#if defined(USE_RX_SPI) || defined (USE_SERIALRX_SRXL2)
void cliRxBind(char *cmdline){
    UNUSED(cmdline);
    if (rxConfig()->receiverType == RX_TYPE_SERIAL) {
        switch (rxConfig()->serialrx_provider) {
        default:
            cliPrint("Not supported.");
            break;
#if defined(USE_SERIALRX_SRXL2)
        case SERIALRX_SRXL2:
            srxl2Bind();
            cliPrint("Binding SRXL2 receiver...");
            break;
#endif
        }
    }
#if defined(USE_RX_SPI)
    else if (rxConfig()->receiverType == RX_TYPE_SPI) {
        switch (rxConfig()->rx_spi_protocol) {
        default:
            cliPrint("Not supported.");
            break;
        }

    }
#endif
}
#endif

static void cliExit(char *cmdline)
{
    UNUSED(cmdline);

#ifndef CLI_MINIMAL_VERBOSITY
    cliPrintLine("\r\nLeaving CLI mode, unsaved changes lost.");
#endif
    bufWriterFlush(cliWriter);

    *cliBuffer = '\0';
    bufferIndex = 0;
    cliMode = 0;
    // incase a motor was left running during motortest, clear it here
    mixerResetDisarmedMotors();
    cliReboot();

    cliWriter = NULL;
}

#ifdef USE_GPS
static void cliGpsPassthrough(char *cmdline)
{
    UNUSED(cmdline);

    gpsEnablePassthrough(cliPort);
}
#endif

static void cliMotor(char *cmdline)
{
    int motor_index = 0;
    int motor_value = 0;
    int index = 0;
    char *pch = NULL;
    char *saveptr;

    if (isEmpty(cmdline)) {
        cliShowParseError();

        return;
    }

    pch = strtok_r(cmdline, " ", &saveptr);
    while (pch != NULL) {
        switch (index) {
            case 0:
                motor_index = fastA2I(pch);
                break;
            case 1:
                motor_value = fastA2I(pch);
                break;
        }
        index++;
        pch = strtok_r(NULL, " ", &saveptr);
    }

    if (motor_index < 0 || motor_index >= MAX_SUPPORTED_MOTORS) {
        cliShowArgumentRangeError("index", 0, MAX_SUPPORTED_MOTORS - 1);
        return;
    }

    if (index == 2) {
        if (motor_value < PWM_RANGE_MIN || motor_value > PWM_RANGE_MAX) {
            cliShowArgumentRangeError("value", 1000, 2000);
            return;
        } else {
            motor_disarmed[motor_index] = motor_value;
        }
    }

    cliPrintLinef("motor %d: %d", motor_index, motor_disarmed[motor_index]);
}

#ifdef PLAY_SOUND
static void cliPlaySound(char *cmdline)
{
    int i;
    const char *name;
    static int lastSoundIdx = -1;

    if (isEmpty(cmdline)) {
        i = lastSoundIdx + 1;     //next sound index
        if ((name=beeperNameForTableIndex(i)) == NULL) {
            while (true) {   //no name for index; try next one
                if (++i >= beeperTableEntryCount())
                    i = 0;   //if end then wrap around to first entry
                if ((name=beeperNameForTableIndex(i)) != NULL)
                    break;   //if name OK then play sound below
                if (i == lastSoundIdx + 1) {     //prevent infinite loop
                    cliPrintLine("Error playing sound");
                    return;
                }
            }
        }
    } else {       //index value was given
        i = fastA2I(cmdline);
        if ((name=beeperNameForTableIndex(i)) == NULL) {
            cliPrintLinef("No sound for index %d", i);
            return;
        }
    }
    lastSoundIdx = i;
    beeperSilence();
    cliPrintLinef("Playing sound %d: %s", i, name);
    beeper(beeperModeForTableIndex(i));
}
#endif

static void cliProfile(char *cmdline)
{
    // CLI profile index is 1-based
    if (isEmpty(cmdline)) {
        cliPrintLinef("profile %d", getConfigProfile() + 1);
        return;
    } else {
        const int i = fastA2I(cmdline) - 1;
        if (i >= 0 && i < MAX_PROFILE_COUNT) {
            setConfigProfileAndWriteEEPROM(i);
            cliProfile("");
        }
    }
}

static void cliDumpProfile(uint8_t profileIndex, uint8_t dumpMask)
{
    if (profileIndex >= MAX_PROFILE_COUNT) {
        // Faulty values
        return;
    }
    setConfigProfile(profileIndex);
    cliPrintHashLine("profile");
    cliPrintLinef("profile %d\r\n", getConfigProfile() + 1);
    dumpAllValues(PROFILE_VALUE, dumpMask);
    dumpAllValues(CONTROL_RATE_VALUE, dumpMask);
}

static void cliBatteryProfile(char *cmdline)
{
    // CLI profile index is 1-based
    if (isEmpty(cmdline)) {
        cliPrintLinef("battery_profile %d", getConfigBatteryProfile() + 1);
        return;
    } else {
        const int i = fastA2I(cmdline) - 1;
        if (i >= 0 && i < MAX_PROFILE_COUNT) {
            setConfigBatteryProfileAndWriteEEPROM(i);
            cliBatteryProfile("");
        }
    }
}

static void cliDumpBatteryProfile(uint8_t profileIndex, uint8_t dumpMask)
{
    if (profileIndex >= MAX_BATTERY_PROFILE_COUNT) {
        // Faulty values
        return;
    }
    setConfigBatteryProfile(profileIndex);
    cliPrintHashLine("battery_profile");
    cliPrintLinef("battery_profile %d\r\n", getConfigBatteryProfile() + 1);
    dumpAllValues(BATTERY_CONFIG_VALUE, dumpMask);
}

#ifdef USE_CLI_BATCH
static void cliPrintCommandBatchWarning(const char *warning)
{
    cliPrintErrorLinef("ERRORS WERE DETECTED - PLEASE REVIEW BEFORE CONTINUING");
    if (warning) {
        cliPrintErrorLinef(warning);
    }
}

static void resetCommandBatch(void)
{
    commandBatchActive = false;
    commandBatchError = false;
}

static void cliBatch(char *cmdline)
{
    if (strncasecmp(cmdline, "start", 5) == 0) {
        if (!commandBatchActive) {
            commandBatchActive = true;
            commandBatchError = false;
        }
        cliPrintLine("Command batch started");
    } else if (strncasecmp(cmdline, "end", 3) == 0) {
        if (commandBatchActive && commandBatchError) {
            cliPrintCommandBatchWarning(NULL);
        } else {
            cliPrintLine("Command batch ended");
        }
        resetCommandBatch();
    } else {
        cliPrintErrorLinef("Invalid option");
    }
}
#endif

#ifdef USE_SECONDARY_IMU

static void printImu2Status(void)
{
    cliPrintLinef("Secondary IMU active: %d", secondaryImuState.active);
    cliPrintLine("Calibration status:");
    cliPrintLinef("Sys: %d", secondaryImuState.calibrationStatus.sys);
    cliPrintLinef("Gyro: %d", secondaryImuState.calibrationStatus.gyr);
    cliPrintLinef("Acc: %d", secondaryImuState.calibrationStatus.acc);
    cliPrintLinef("Mag: %d", secondaryImuState.calibrationStatus.mag);
    cliPrintLine("Calibration gains:");

    cliPrintLinef(
        "Gyro: %d %d %d",
        secondaryImuConfig()->calibrationOffsetGyro[X],
        secondaryImuConfig()->calibrationOffsetGyro[Y],
        secondaryImuConfig()->calibrationOffsetGyro[Z]
    );
    cliPrintLinef(
        "Acc: %d %d %d",
        secondaryImuConfig()->calibrationOffsetAcc[X],
        secondaryImuConfig()->calibrationOffsetAcc[Y],
        secondaryImuConfig()->calibrationOffsetAcc[Z]
    );
    cliPrintLinef(
        "Mag: %d %d %d",
        secondaryImuConfig()->calibrationOffsetMag[X],
        secondaryImuConfig()->calibrationOffsetMag[Y],
        secondaryImuConfig()->calibrationOffsetMag[Z]
    );
    cliPrintLinef(
        "Radius: %d %d",
        secondaryImuConfig()->calibrationRadiusAcc,
        secondaryImuConfig()->calibrationRadiusMag
    );
}

static void cliImu2(char *cmdline)
{
    if (sl_strcasecmp(cmdline, "fetch") == 0) {
        secondaryImuFetchCalibration();
        printImu2Status();
    } else {
        printImu2Status();
    }
}

#endif

static void cliSave(char *cmdline)
{
    UNUSED(cmdline);

#ifdef USE_CLI_BATCH
    if (commandBatchActive && commandBatchError) {
        cliPrintCommandBatchWarning("PLEASE FIX ERRORS THEN 'SAVE'");
        resetCommandBatch();
        return;
    }
#endif

    cliPrint("Saving");
    //copyCurrentProfileToProfileSlot(getConfigProfile();
    writeEEPROM();
    cliReboot();
}

static void cliDefaults(char *cmdline)
{
    UNUSED(cmdline);

    cliPrint("Resetting to defaults");
    resetEEPROM();

#ifdef USE_CLI_BATCH
    commandBatchError = false;
#endif

    if (!checkCommand(cmdline, "noreboot"))
        cliReboot();
}

static void cliGet(char *cmdline)
{
    const setting_t *val;
    int matchedCommands = 0;
    char name[SETTING_MAX_NAME_LENGTH];

    while(*cmdline == ' ') ++cmdline; // ignore spaces

    for (uint32_t i = 0; i < SETTINGS_TABLE_COUNT; i++) {
        val = settingGet(i);
        if (settingNameContains(val, name, cmdline)) {
            cliPrintf("%s = ", name);
            cliPrintVar(val, 0);
            cliPrintLinefeed();
            cliPrintVarRange(val);
            cliPrintLinefeed();

            matchedCommands++;
        }
    }


    if (matchedCommands) {
        return;
    }

    cliPrintErrorLine("Invalid name");
}

static void cliSet(char *cmdline)
{
    uint32_t len;
    const setting_t *val;
    char *eqptr = NULL;
    char name[SETTING_MAX_NAME_LENGTH];

    while(*cmdline == ' ') ++cmdline; // ignore spaces

    len = strlen(cmdline);

    if (len == 0 || (len == 1 && cmdline[0] == '*')) {
        cliPrintLine("Current settings:");
        for (uint32_t i = 0; i < SETTINGS_TABLE_COUNT; i++) {
            val = settingGet(i);
            settingGetName(val, name);
            cliPrintf("%s = ", name);
            cliPrintVar(val, len); // when len is 1 (when * is passed as argument), it will print min/max values as well, for gui
            cliPrintLinefeed();
        }
    } else if ((eqptr = strstr(cmdline, "=")) != NULL) {
        // has equals

        char *lastNonSpaceCharacter = eqptr;
        while (*(lastNonSpaceCharacter - 1) == ' ') {
            lastNonSpaceCharacter--;
        }
        uint8_t variableNameLength = lastNonSpaceCharacter - cmdline;

        // skip the '=' and any ' ' characters
        eqptr++;
        while (*(eqptr) == ' ') {
            eqptr++;
        }

        for (uint32_t i = 0; i < SETTINGS_TABLE_COUNT; i++) {
            val = settingGet(i);
            // ensure exact match when setting to prevent setting variables with shorter names
            if (settingNameIsExactMatch(val, name, cmdline, variableNameLength)) {
                const setting_type_e type = SETTING_TYPE(val);
                if (type == VAR_STRING) {
                    settingSetString(val, eqptr, strlen(eqptr));
                    return;
                }
                const setting_mode_e mode = SETTING_MODE(val);
                bool changeValue = false;
                int_float_value_t tmp = {0};
                switch (mode) {
                case MODE_DIRECT: {
                        if (*eqptr != 0 && strspn(eqptr, "0123456789.+-") == strlen(eqptr)) {
                            float valuef = fastA2F(eqptr);
                            // note: compare float values
                            if (valuef >= (float)settingGetMin(val) && valuef <= (float)settingGetMax(val)) {

                                if (type == VAR_FLOAT)
                                    tmp.float_value = valuef;
                                else if (type == VAR_UINT32)
                                    tmp.uint_value = fastA2UL(eqptr);
                                else
                                    tmp.int_value = fastA2I(eqptr);

                                changeValue = true;
                            }
                        }
                    }
                    break;
                case MODE_LOOKUP: {
                        const lookupTableEntry_t *tableEntry = settingLookupTable(val);
                        bool matched = false;
                        for (uint32_t tableValueIndex = 0; tableValueIndex < tableEntry->valueCount && !matched; tableValueIndex++) {
                            matched = sl_strcasecmp(tableEntry->values[tableValueIndex], eqptr) == 0;

                            if (matched) {
                                tmp.int_value = tableValueIndex;
                                changeValue = true;
                            }
                        }
                    }
                    break;
                }

                if (changeValue) {
                    cliSetIntFloatVar(val, tmp);

                    cliPrintf("%s set to ", name);
                    cliPrintVar(val, 0);
                } else {
                    cliPrintError("Invalid value. ");
                    cliPrintVarRange(val);
                    cliPrintLinefeed();
                }

                return;
            }
        }
        cliPrintErrorLine("Invalid name");
    } else {
        // no equals, check for matching variables.
        cliGet(cmdline);
    }
}

static const char * getBatteryStateString(void)
{
    static const char * const batteryStateStrings[] = {"OK", "WARNING", "CRITICAL", "NOT PRESENT"};

    return batteryStateStrings[getBatteryState()];
}

static void cliStatus(char *cmdline)
{
    UNUSED(cmdline);

    char buf[MAX(FORMATTED_DATE_TIME_BUFSIZE, SETTING_MAX_NAME_LENGTH)];
    dateTime_t dt;

    cliPrintLinef("System Uptime: %d seconds", millis() / 1000);
    rtcGetDateTime(&dt);
    dateTimeFormatLocal(buf, &dt);
    cliPrintLinef("Current Time: %s", buf);
    cliPrintLinef("Voltage: %d.%02dV (%dS battery - %s)", getBatteryVoltage() / 100, getBatteryVoltage() % 100, getBatteryCellCount(), getBatteryStateString());
    cliPrintf("CPU Clock=%dMHz", (SystemCoreClock / 1000000));

    const uint32_t detectedSensorsMask = sensorsMask();

    for (int i = 0; i < SENSOR_INDEX_COUNT; i++) {

        const uint32_t mask = (1 << i);
        if ((detectedSensorsMask & mask) && (mask & SENSOR_NAMES_MASK)) {
            const int sensorHardwareIndex = detectedSensors[i];
            if (sensorHardwareNames[i]) {
                const char *sensorHardware = sensorHardwareNames[i][sensorHardwareIndex];
                cliPrintf(", %s=%s", sensorTypeNames[i], sensorHardware);
            }
        }
    }
    cliPrintLinefeed();

    cliPrintLine("STM32 system clocks:");
#if defined(USE_HAL_DRIVER)
    cliPrintLinef("  SYSCLK = %d MHz", HAL_RCC_GetSysClockFreq() / 1000000);
    cliPrintLinef("  HCLK   = %d MHz", HAL_RCC_GetHCLKFreq() / 1000000);
    cliPrintLinef("  PCLK1  = %d MHz", HAL_RCC_GetPCLK1Freq() / 1000000);
    cliPrintLinef("  PCLK2  = %d MHz", HAL_RCC_GetPCLK2Freq() / 1000000);
#else
    RCC_ClocksTypeDef clocks;
    RCC_GetClocksFreq(&clocks);
    cliPrintLinef("  SYSCLK = %d MHz", clocks.SYSCLK_Frequency / 1000000);
    cliPrintLinef("  HCLK   = %d MHz", clocks.HCLK_Frequency / 1000000);
    cliPrintLinef("  PCLK1  = %d MHz", clocks.PCLK1_Frequency / 1000000);
    cliPrintLinef("  PCLK2  = %d MHz", clocks.PCLK2_Frequency / 1000000);
#endif

    cliPrintLinef("Sensor status: GYRO=%s, ACC=%s, MAG=%s, BARO=%s, RANGEFINDER=%s, OPFLOW=%s, GPS=%s",
        hardwareSensorStatusNames[getHwGyroStatus()],
        hardwareSensorStatusNames[getHwAccelerometerStatus()],
        hardwareSensorStatusNames[getHwCompassStatus()],
        hardwareSensorStatusNames[getHwBarometerStatus()],
        hardwareSensorStatusNames[getHwRangefinderStatus()],
        hardwareSensorStatusNames[getHwOpticalFlowStatus()],
        hardwareSensorStatusNames[getHwGPSStatus()]
    );

#ifdef USE_SDCARD
    cliSdInfo(NULL);
#endif
#ifdef USE_I2C
    const uint16_t i2cErrorCounter = i2cGetErrorCounter();
#else
    const uint16_t i2cErrorCounter = 0;
#endif

#ifdef STACK_CHECK
    cliPrintf("Stack used: %d, ", stackUsedSize());
#endif
    cliPrintLinef("Stack size: %d, Stack address: 0x%x, Heap available: %d", stackTotalSize(), stackHighMem(), memGetAvailableBytes());

    cliPrintLinef("I2C Errors: %d, config size: %d, max available config: %d", i2cErrorCounter, getEEPROMConfigSize(), &__config_end - &__config_start);

#ifdef USE_ADC
    static char * adcFunctions[] = { "BATTERY", "RSSI", "CURRENT", "AIRSPEED" };
    cliPrintLine("ADC channel usage:");
    for (int i = 0; i < ADC_FUNCTION_COUNT; i++) {
        cliPrintf("  %8s :", adcFunctions[i]);

        cliPrint(" configured = ");
        if (adcChannelConfig()->adcFunctionChannel[i] == ADC_CHN_NONE) {
            cliPrint("none");
        }
        else {
            cliPrintf("ADC %d", adcChannelConfig()->adcFunctionChannel[i]);
        }

        cliPrint(", used = ");
        if (adcGetFunctionChannelAllocation(i) == ADC_CHN_NONE) {
            cliPrintLine("none");
        }
        else {
            cliPrintLinef("ADC %d", adcGetFunctionChannelAllocation(i));
        }
    }
#endif

    cliPrintf("System load: %d", averageSystemLoadPercent);
    const timeDelta_t pidTaskDeltaTime = getTaskDeltaTime(TASK_PID);
    const int pidRate = pidTaskDeltaTime == 0 ? 0 : (int)(1000000.0f / ((float)pidTaskDeltaTime));
    const int rxRate = getTaskDeltaTime(TASK_RX) == 0 ? 0 : (int)(1000000.0f / ((float)getTaskDeltaTime(TASK_RX)));
    const int systemRate = getTaskDeltaTime(TASK_SYSTEM) == 0 ? 0 : (int)(1000000.0f / ((float)getTaskDeltaTime(TASK_SYSTEM)));
    cliPrintLinef(", cycle time: %d, PID rate: %d, RX rate: %d, System rate: %d",  (uint16_t)cycleTime, pidRate, rxRate, systemRate);
#if !defined(CLI_MINIMAL_VERBOSITY)
    cliPrint("Arming disabled flags:");
    uint32_t flags = armingFlags & ARMING_DISABLED_ALL_FLAGS;
    while (flags) {
        int bitpos = ffs(flags) - 1;
        flags &= ~(1 << bitpos);
	if (bitpos > 6) cliPrintf(" %s", armingDisableFlagNames[bitpos - 7]);
    }
    cliPrintLinefeed();
    if (armingFlags & ARMING_DISABLED_INVALID_SETTING) {
        unsigned invalidIndex;
        if (!settingsValidate(&invalidIndex)) {
            settingGetName(settingGet(invalidIndex), buf);
            cliPrintErrorLinef("Invalid setting: %s", buf);
        }
    }
#else
    cliPrintLinef("Arming disabled flags: 0x%lx", armingFlags & ARMING_DISABLED_ALL_FLAGS);
#endif

#if defined(USE_VTX_CONTROL) && !defined(CLI_MINIMAL_VERBOSITY)
    cliPrint("VTX: ");

    if (vtxCommonDeviceIsReady(vtxCommonDevice())) {
        vtxDeviceOsdInfo_t osdInfo;
        vtxCommonGetOsdInfo(vtxCommonDevice(), &osdInfo);
        cliPrintf("band: %c, chan: %s, power: %c", osdInfo.bandLetter, osdInfo.channelName, osdInfo.powerIndexLetter);

        if (osdInfo.powerMilliwatt) {
            cliPrintf(" (%d mW)", osdInfo.powerMilliwatt);
        }

        if (osdInfo.frequency) {
            cliPrintf(", freq: %d MHz", osdInfo.frequency);
        }
    }
    else {
        cliPrint("not detected");
    }

    cliPrintLinefeed();
#endif

    // If we are blocked by PWM init - provide more information
    if (getPwmInitError() != PWM_INIT_ERROR_NONE) {
        cliPrintLinef("PWM output init error: %s", getPwmInitErrorMessage());
    }
}

#ifndef SKIP_TASK_STATISTICS
static void cliTasks(char *cmdline)
{
    UNUSED(cmdline);
    int maxLoadSum = 0;
    int averageLoadSum = 0;
    cfCheckFuncInfo_t checkFuncInfo;

    cliPrintLinef("Task list         rate/hz  max/us  avg/us maxload avgload     total/ms");
    for (cfTaskId_e taskId = 0; taskId < TASK_COUNT; taskId++) {
        cfTaskInfo_t taskInfo;
        getTaskInfo(taskId, &taskInfo);
        if (taskInfo.isEnabled) {
            const int taskFrequency = taskInfo.latestDeltaTime == 0 ? 0 : (int)(1000000.0f / ((float)taskInfo.latestDeltaTime));
            const int maxLoad = (taskInfo.maxExecutionTime * taskFrequency + 5000) / 1000;
            const int averageLoad = (taskInfo.averageExecutionTime * taskFrequency + 5000) / 1000;
            if (taskId != TASK_SERIAL) {
                maxLoadSum += maxLoad;
                averageLoadSum += averageLoad;
            }
            cliPrintLinef("%2d - %12s  %6d   %5d   %5d %4d.%1d%% %4d.%1d%%  %8d",
                    taskId, taskInfo.taskName, taskFrequency, (uint32_t)taskInfo.maxExecutionTime, (uint32_t)taskInfo.averageExecutionTime,
                    maxLoad/10, maxLoad%10, averageLoad/10, averageLoad%10, (uint32_t)taskInfo.totalExecutionTime / 1000);
        }
    }
    getCheckFuncInfo(&checkFuncInfo);
    cliPrintLinef("Task check function %13d %7d %25d", (uint32_t)checkFuncInfo.maxExecutionTime, (uint32_t)checkFuncInfo.averageExecutionTime, (uint32_t)checkFuncInfo.totalExecutionTime / 1000);
    cliPrintLinef("Total (excluding SERIAL) %21d.%1d%% %4d.%1d%%", maxLoadSum/10, maxLoadSum%10, averageLoadSum/10, averageLoadSum%10);
}
#endif

static void cliVersion(char *cmdline)
{
    UNUSED(cmdline);

    cliPrintLinef("# %s/%s %s %s / %s (%s)",
        FC_FIRMWARE_NAME,
        targetName,
        FC_VERSION_STRING,
        buildDate,
        buildTime,
        shortGitRevision
    );
    cliPrintLinef("# GCC-%s",
        compilerVersion
    );
}

static void cliMemory(char *cmdline)
{
    UNUSED(cmdline);
    cliPrintLinef("Dynamic memory usage:");
    for (unsigned i = 0; i < OWNER_TOTAL_COUNT; i++) {
        const char * owner = ownerNames[i];
        const uint32_t memUsed = memGetUsedBytesByOwner(i);

        if (memUsed) {
            cliPrintLinef("%s : %d bytes", owner, memUsed);
        }
    }
}

#if !defined(SKIP_TASK_STATISTICS) && !defined(SKIP_CLI_RESOURCES)
static void cliResource(char *cmdline)
{
    UNUSED(cmdline);
    cliPrintLinef("IO:\r\n----------------------");
    for (unsigned i = 0; i < DEFIO_IO_USED_COUNT; i++) {
        const char* owner;
        owner = ownerNames[ioRecs[i].owner];

        const char* resource;
        resource = resourceNames[ioRecs[i].resource];

        if (ioRecs[i].index > 0) {
            cliPrintLinef("%c%02d: %s%d %s", IO_GPIOPortIdx(ioRecs + i) + 'A', IO_GPIOPinIdx(ioRecs + i), owner, ioRecs[i].index, resource);
        } else {
            cliPrintLinef("%c%02d: %s %s", IO_GPIOPortIdx(ioRecs + i) + 'A', IO_GPIOPinIdx(ioRecs + i), owner, resource);
        }
    }
}
#endif

static void backupConfigs(void)
{
    // make copies of configs to do differencing
    PG_FOREACH(pg) {
        if (pgIsProfile(pg)) {
            memcpy(pg->copy, pg->address, pgSize(pg) * MAX_PROFILE_COUNT);
        } else {
            memcpy(pg->copy, pg->address, pgSize(pg));
        }
    }
}

static void restoreConfigs(void)
{
    PG_FOREACH(pg) {
        if (pgIsProfile(pg)) {
            memcpy(pg->address, pg->copy, pgSize(pg) * MAX_PROFILE_COUNT);
        } else {
            memcpy(pg->address, pg->copy, pgSize(pg));
        }
    }
}

static void printConfig(const char *cmdline, bool doDiff)
{
    uint8_t dumpMask = DUMP_MASTER;
    const char *options;
    if ((options = checkCommand(cmdline, "master"))) {
        dumpMask = DUMP_MASTER; // only
    } else if ((options = checkCommand(cmdline, "profile"))) {
        dumpMask = DUMP_PROFILE; // only
    } else if ((options = checkCommand(cmdline, "battery_profile"))) {
        dumpMask = DUMP_BATTERY_PROFILE; // only
    } else if ((options = checkCommand(cmdline, "all"))) {
        dumpMask = DUMP_ALL;   // all profiles and rates
    } else {
        options = cmdline;
    }

    if (doDiff) {
        dumpMask = dumpMask | DO_DIFF;
    }

    const int currentProfileIndexSave = getConfigProfile();
    const int currentBatteryProfileIndexSave = getConfigBatteryProfile();
    backupConfigs();
    // reset all configs to defaults to do differencing
    resetConfigs();
    // restore the profile indices, since they should not be reset for proper comparison
    setConfigProfile(currentProfileIndexSave);
    setConfigBatteryProfile(currentBatteryProfileIndexSave);

    if (checkCommand(options, "showdefaults")) {
        dumpMask = dumpMask | SHOW_DEFAULTS;   // add default values as comments for changed values
    }

#ifdef USE_CLI_BATCH
    bool batchModeEnabled = false;
#endif

    if ((dumpMask & DUMP_MASTER) || (dumpMask & DUMP_ALL)) {
        cliPrintHashLine("version");
        cliVersion(NULL);

#ifdef USE_CLI_BATCH
        cliPrintHashLine("start the command batch");
        cliPrintLine("batch start");
        batchModeEnabled = true;
#endif

        if ((dumpMask & (DUMP_ALL | DO_DIFF)) == (DUMP_ALL | DO_DIFF)) {
#ifndef CLI_MINIMAL_VERBOSITY
            cliPrintHashLine("reset configuration to default settings\r\ndefaults noreboot");
#else
            cliPrintLinef("defaults noreboot");
#endif
        }

        cliPrintHashLine("resources");
        //printResource(dumpMask, &defaultConfig);

        cliPrintHashLine("mixer");
        cliDumpPrintLinef(dumpMask, primaryMotorMixer_CopyArray[0].throttle == 0.0f, "\r\nmmix reset\r\n");

        printMotorMix(dumpMask, primaryMotorMixer_CopyArray, primaryMotorMixer(0));

        // print custom servo mixer if exists
        cliPrintHashLine("servo mix");
        cliDumpPrintLinef(dumpMask, customServoMixers_CopyArray[0].rate == 0, "smix reset\r\n");
        printServoMix(dumpMask, customServoMixers_CopyArray, customServoMixers(0));

        // print servo parameters
        cliPrintHashLine("servo");
        printServo(dumpMask, servoParams_CopyArray, servoParams(0));

#if defined(USE_SAFE_HOME)
        cliPrintHashLine("safehome");
        printSafeHomes(dumpMask, safeHomeConfig_CopyArray, safeHomeConfig(0));
#endif
#ifdef USE_PROGRAMMING_FRAMEWORK
        cliPrintHashLine("logic");
        printLogic(dumpMask, logicConditions_CopyArray, logicConditions(0));

        cliPrintHashLine("gvar");
        printGvar(dumpMask, globalVariableConfigs_CopyArray, globalVariableConfigs(0));

        cliPrintHashLine("pid");
        printPid(dumpMask, programmingPids_CopyArray, programmingPids(0));
#endif

        cliPrintHashLine("feature");
        printFeature(dumpMask, &featureConfig_Copy, featureConfig());

#if defined(BEEPER) || defined(USE_DSHOT)
        cliPrintHashLine("beeper");
        printBeeper(dumpMask, &beeperConfig_Copy, beeperConfig());
#endif

        cliPrintHashLine("map");
        printMap(dumpMask, &rxConfig_Copy, rxConfig());

        cliPrintHashLine("serial");
        printSerial(dumpMask, &serialConfig_Copy, serialConfig());

#ifdef USE_LED_STRIP
        cliPrintHashLine("led");
        printLed(dumpMask, ledStripConfig_Copy.ledConfigs, ledStripConfig()->ledConfigs);

        cliPrintHashLine("color");
        printColor(dumpMask, ledStripConfig_Copy.colors, ledStripConfig()->colors);

        cliPrintHashLine("mode_color");
        printModeColor(dumpMask, &ledStripConfig_Copy, ledStripConfig());
#endif

        cliPrintHashLine("aux");
        printAux(dumpMask, modeActivationConditions_CopyArray, modeActivationConditions(0));

        cliPrintHashLine("adjrange");
        printAdjustmentRange(dumpMask, adjustmentRanges_CopyArray, adjustmentRanges(0));

        cliPrintHashLine("rxrange");
        printRxRange(dumpMask, rxChannelRangeConfigs_CopyArray, rxChannelRangeConfigs(0));

#ifdef USE_TEMPERATURE_SENSOR
        cliPrintHashLine("temp_sensor");
        printTempSensor(dumpMask, tempSensorConfig_CopyArray, tempSensorConfig(0));
#endif

#if defined(USE_NAV) && defined(NAV_NON_VOLATILE_WAYPOINT_STORAGE) && defined(NAV_NON_VOLATILE_WAYPOINT_CLI)
        cliPrintHashLine("wp");
        printWaypoints(dumpMask, posControl.waypointList, nonVolatileWaypointList(0));
#endif

#ifdef USE_OSD
        cliPrintHashLine("osd_layout");
        printOsdLayout(dumpMask, &osdLayoutsConfig_Copy, osdLayoutsConfig(), -1, -1);
#endif

        cliPrintHashLine("master");
        dumpAllValues(MASTER_VALUE, dumpMask);

        if (dumpMask & DUMP_ALL) {
            // dump all profiles
            const int currentProfileIndexSave = getConfigProfile();
            const int currentBatteryProfileIndexSave = getConfigBatteryProfile();
            for (int ii = 0; ii < MAX_PROFILE_COUNT; ++ii) {
                cliDumpProfile(ii, dumpMask);
            }
            for (int ii = 0; ii < MAX_BATTERY_PROFILE_COUNT; ++ii) {
                cliDumpBatteryProfile(ii, dumpMask);
            }
            setConfigProfile(currentProfileIndexSave);
            setConfigBatteryProfile(currentBatteryProfileIndexSave);

            cliPrintHashLine("restore original profile selection");
            cliPrintLinef("profile %d", currentProfileIndexSave + 1);
            cliPrintLinef("battery_profile %d", currentBatteryProfileIndexSave + 1);

            cliPrintHashLine("save configuration\r\nsave");
#ifdef USE_CLI_BATCH
            batchModeEnabled = false;
#endif
        } else {
            // dump just the current profiles
            cliDumpProfile(getConfigProfile(), dumpMask);
            cliDumpBatteryProfile(getConfigBatteryProfile(), dumpMask);
        }
    }

    if (dumpMask & DUMP_PROFILE) {
        cliDumpProfile(getConfigProfile(), dumpMask);
    }

    if (dumpMask & DUMP_BATTERY_PROFILE) {
        cliDumpBatteryProfile(getConfigBatteryProfile(), dumpMask);
    }

#ifdef USE_CLI_BATCH
    if (batchModeEnabled) {
        cliPrintHashLine("end the command batch");
        cliPrintLine("batch end");
    }
#endif

    // restore configs from copies
    restoreConfigs();
}

static void cliDump(char *cmdline)
{
    printConfig(cmdline, false);
}

static void cliDiff(char *cmdline)
{
    printConfig(cmdline, true);
}

#ifdef USE_USB_MSC
static void cliMsc(char *cmdline)
{
    UNUSED(cmdline);

    if (false
#ifdef USE_SDCARD
        || sdcard_isFunctional()
#endif
#ifdef USE_FLASHFS
        || flashfsGetSize() > 0
#endif
    ) {
        cliPrintHashLine("restarting in mass storage mode");
        cliPrint("\r\nRebooting");
        bufWriterFlush(cliWriter);
        delay(1000);
        waitForSerialPortToFinishTransmitting(cliPort);
        stopPwmAllMotors();
        systemResetRequest(RESET_MSC_REQUEST);
    } else {
        cliPrint("\r\nStorage not present or failed to initialize!");
        bufWriterFlush(cliWriter);
    }
}
#endif


typedef struct {
    const char *name;
#ifndef SKIP_CLI_COMMAND_HELP
    const char *description;
    const char *args;
#endif
    void (*func)(char *cmdline);
} clicmd_t;

#ifndef SKIP_CLI_COMMAND_HELP
#define CLI_COMMAND_DEF(name, description, args, method) \
{ \
    name , \
    description , \
    args , \
    method \
}
#else
#define CLI_COMMAND_DEF(name, description, args, method) \
{ \
    name, \
    method \
}
#endif

static void cliHelp(char *cmdline);

// should be sorted a..z for bsearch()
const clicmd_t cmdTable[] = {
    CLI_COMMAND_DEF("adjrange", "configure adjustment ranges", NULL, cliAdjustmentRange),
#if defined(USE_ASSERT)
    CLI_COMMAND_DEF("assert", "", NULL, cliAssert),
#endif
    CLI_COMMAND_DEF("aux", "configure modes", NULL, cliAux),
#ifdef USE_CLI_BATCH
    CLI_COMMAND_DEF("batch", "start or end a batch of commands", "start | end", cliBatch),
#endif
#if defined(BEEPER) || defined(USE_DSHOT)
    CLI_COMMAND_DEF("beeper", "turn on/off beeper", "list\r\n"
            "\t<+|->[name]", cliBeeper),
#endif
#if defined(USE_RX_SPI) || defined (USE_SERIALRX_SRXL2)
    CLI_COMMAND_DEF("bind_rx", "initiate binding for RX SPI or SRXL2", NULL, cliRxBind),
#endif
#if defined(USE_BOOTLOG)
    CLI_COMMAND_DEF("bootlog", "show boot events", NULL, cliBootlog),
#endif
#ifdef USE_LED_STRIP
    CLI_COMMAND_DEF("color", "configure colors", NULL, cliColor),
    CLI_COMMAND_DEF("mode_color", "configure mode and special colors", NULL, cliModeColor),
#endif
    CLI_COMMAND_DEF("defaults", "reset to defaults and reboot", NULL, cliDefaults),
    CLI_COMMAND_DEF("dfu", "DFU mode on reboot", NULL, cliDfu),
    CLI_COMMAND_DEF("diff", "list configuration changes from default",
        "[master|battery_profile|profile|rates|all] {showdefaults}", cliDiff),
    CLI_COMMAND_DEF("dump", "dump configuration",
        "[master|battery_profile|profile|rates|all] {showdefaults}", cliDump),
#ifdef USE_RX_ELERES
    CLI_COMMAND_DEF("eleres_bind", NULL, NULL, cliEleresBind),
#endif // USE_RX_ELERES
    CLI_COMMAND_DEF("exit", NULL, NULL, cliExit),
    CLI_COMMAND_DEF("feature", "configure features",
        "list\r\n"
        "\t<+|->[name]", cliFeature),
#ifdef USE_FLASHFS
    CLI_COMMAND_DEF("flash_erase", "erase flash chip", NULL, cliFlashErase),
    CLI_COMMAND_DEF("flash_info", "show flash chip info", NULL, cliFlashInfo),
#ifdef USE_FLASH_TOOLS
    CLI_COMMAND_DEF("flash_read", NULL, "<length> <address>", cliFlashRead),
    CLI_COMMAND_DEF("flash_write", NULL, "<address> <message>", cliFlashWrite),
#endif
#endif
    CLI_COMMAND_DEF("get", "get variable value", "[name]", cliGet),
#ifdef USE_GPS
    CLI_COMMAND_DEF("gpspassthrough", "passthrough gps to serial", NULL, cliGpsPassthrough),
#endif
    CLI_COMMAND_DEF("help", NULL, NULL, cliHelp),
#ifdef USE_LED_STRIP
    CLI_COMMAND_DEF("led", "configure leds", NULL, cliLed),
#endif
    CLI_COMMAND_DEF("map", "configure rc channel order", "[<map>]", cliMap),
    CLI_COMMAND_DEF("memory", "view memory usage", NULL, cliMemory),
    CLI_COMMAND_DEF("mmix", "custom motor mixer", NULL, cliMotorMix),
    CLI_COMMAND_DEF("motor",  "get/set motor", "<index> [<value>]", cliMotor),
#ifdef USE_USB_MSC
    CLI_COMMAND_DEF("msc", "switch into msc mode", NULL, cliMsc),
#endif
#ifdef PLAY_SOUND
    CLI_COMMAND_DEF("play_sound", NULL, "[<index>]\r\n", cliPlaySound),
#endif
    CLI_COMMAND_DEF("profile", "change profile",
        "[<index>]", cliProfile),
    CLI_COMMAND_DEF("battery_profile", "change battery profile",
        "[<index>]", cliBatteryProfile),
#if !defined(SKIP_TASK_STATISTICS) && !defined(SKIP_CLI_RESOURCES)
    CLI_COMMAND_DEF("resource", "view currently used resources", NULL, cliResource),
#endif
    CLI_COMMAND_DEF("rxrange", "configure rx channel ranges", NULL, cliRxRange),
#ifdef USE_SECONDARY_IMU
    CLI_COMMAND_DEF("imu2", "Secondary IMU", NULL, cliImu2),
#endif
#if defined(USE_SAFE_HOME)
    CLI_COMMAND_DEF("safehome", "safe home list", NULL, cliSafeHomes),
#endif
    CLI_COMMAND_DEF("save", "save and reboot", NULL, cliSave),
    CLI_COMMAND_DEF("serial", "configure serial ports", NULL, cliSerial),
#ifdef USE_SERIAL_PASSTHROUGH
    CLI_COMMAND_DEF("serialpassthrough", "passthrough serial data to port", "<id> [baud] [mode] : passthrough to serial", cliSerialPassthrough),
#endif
    CLI_COMMAND_DEF("servo", "configure servos", NULL, cliServo),
#ifdef USE_PROGRAMMING_FRAMEWORK
    CLI_COMMAND_DEF("logic", "configure logic conditions",
        "<rule> <enabled> <activatorId> <operation> <operand A type> <operand A value> <operand B type> <operand B value> <flags>\r\n"
        "\treset\r\n", cliLogic),

    CLI_COMMAND_DEF("gvar", "configure global variables",
        "<gvar> <default> <min> <max>\r\n"
        "\treset\r\n", cliGvar),

    CLI_COMMAND_DEF("pid", "configurable PID controllers",
        "<#> <enabled> <setpoint type> <setpoint value> <measurement type> <measurement value> <P gain> <I gain> <D gain> <FF gain>\r\n"
        "\treset\r\n", cliPid),
#endif
    CLI_COMMAND_DEF("set", "change setting", "[<name>=<value>]", cliSet),
    CLI_COMMAND_DEF("smix", "servo mixer",
        "<rule> <servo> <source> <rate> <speed> <conditionId>\r\n"
        "\treset\r\n", cliServoMix),
#ifdef USE_SDCARD
    CLI_COMMAND_DEF("sd_info", "sdcard info", NULL, cliSdInfo),
#endif
    CLI_COMMAND_DEF("status", "show status", NULL, cliStatus),
#ifndef SKIP_TASK_STATISTICS
    CLI_COMMAND_DEF("tasks", "show task stats", NULL, cliTasks),
#endif
#ifdef USE_TEMPERATURE_SENSOR
    CLI_COMMAND_DEF("temp_sensor", "change temp sensor settings", NULL, cliTempSensor),
#endif
    CLI_COMMAND_DEF("version", "show version", NULL, cliVersion),
#if defined(USE_NAV) && defined(NAV_NON_VOLATILE_WAYPOINT_STORAGE) && defined(NAV_NON_VOLATILE_WAYPOINT_CLI)
    CLI_COMMAND_DEF("wp", "waypoint list", NULL, cliWaypoints),
#endif
#ifdef USE_OSD
    CLI_COMMAND_DEF("osd_layout", "get or set the layout of OSD items", "[<layout> [<item> [<col> <row> [<visible>]]]]", cliOsdLayout),
#endif
};

static void cliHelp(char *cmdline)
{
    UNUSED(cmdline);

    for (uint32_t i = 0; i < ARRAYLEN(cmdTable); i++) {
        cliPrint(cmdTable[i].name);
#ifndef SKIP_CLI_COMMAND_HELP
        if (cmdTable[i].description) {
            cliPrintf(" - %s", cmdTable[i].description);
        }
        if (cmdTable[i].args) {
            cliPrintf("\r\n\t%s", cmdTable[i].args);
        }
#endif
        cliPrintLinefeed();
    }
}

void cliProcess(void)
{
    if (!cliWriter) {
        return;
    }

    // Be a little bit tricky.  Flush the last inputs buffer, if any.
    bufWriterFlush(cliWriter);

    while (serialRxBytesWaiting(cliPort)) {
        uint8_t c = serialRead(cliPort);
        if (c == '\t' || c == '?') {
            // do tab completion
            const clicmd_t *cmd, *pstart = NULL, *pend = NULL;
            uint32_t i = bufferIndex;
            for (cmd = cmdTable; cmd < cmdTable + ARRAYLEN(cmdTable); cmd++) {
                if (bufferIndex && (sl_strncasecmp(cliBuffer, cmd->name, bufferIndex) != 0))
                    continue;
                if (!pstart)
                    pstart = cmd;
                pend = cmd;
            }
            if (pstart) {    /* Buffer matches one or more commands */
                for (; ; bufferIndex++) {
                    if (pstart->name[bufferIndex] != pend->name[bufferIndex])
                        break;
                    if (!pstart->name[bufferIndex] && bufferIndex < sizeof(cliBuffer) - 2) {
                        /* Unambiguous -- append a space */
                        cliBuffer[bufferIndex++] = ' ';
                        cliBuffer[bufferIndex] = '\0';
                        break;
                    }
                    cliBuffer[bufferIndex] = pstart->name[bufferIndex];
                }
            }
            if (!bufferIndex || pstart != pend) {
                /* Print list of ambiguous matches */
                cliPrint("\r\033[K");
                for (cmd = pstart; cmd <= pend; cmd++) {
                    cliPrint(cmd->name);
                    cliWrite('\t');
                }
                cliPrompt();
                i = 0;    /* Redraw prompt */
            }
            for (; i < bufferIndex; i++)
                cliWrite(cliBuffer[i]);
        } else if (!bufferIndex && c == 4) {   // CTRL-D
            cliExit(cliBuffer);
            return;
        } else if (c == 12) {                  // NewPage / CTRL-L
            // clear screen
            cliPrint("\033[2J\033[1;1H");
            cliPrompt();
        } else if (bufferIndex && (c == '\n' || c == '\r')) {
            // enter pressed
            cliPrintLinefeed();

            // Strip comment starting with # from line
            char *p = cliBuffer;
            p = strchr(p, '#');
            if (NULL != p) {
                bufferIndex = (uint32_t)(p - cliBuffer);
            }

            // Strip trailing whitespace
            while (bufferIndex > 0 && cliBuffer[bufferIndex - 1] == ' ') {
                bufferIndex--;
            }

            // Process non-empty lines
            if (bufferIndex > 0) {
                cliBuffer[bufferIndex] = 0; // null terminate

                const clicmd_t *cmd;
                for (cmd = cmdTable; cmd < cmdTable + ARRAYLEN(cmdTable); cmd++) {
                    if (!sl_strncasecmp(cliBuffer, cmd->name, strlen(cmd->name))   // command names match
                       && !sl_isalnum((unsigned)cliBuffer[strlen(cmd->name)]))    // next characted in bufffer is not alphanumeric (command is correctly terminated)
                        break;
                }
                if (cmd < cmdTable + ARRAYLEN(cmdTable))
                    cmd->func(cliBuffer + strlen(cmd->name) + 1);
                else
                    cliPrintError("Unknown command, try 'help'");
                bufferIndex = 0;
            }

            memset(cliBuffer, 0, sizeof(cliBuffer));

            // 'exit' will reset this flag, so we don't need to print prompt again
            if (!cliMode)
                return;

            cliPrompt();
        } else if (c == 127) {
            // backspace
            if (bufferIndex) {
                cliBuffer[--bufferIndex] = 0;
                cliPrint("\010 \010");
            }
        } else if (bufferIndex < sizeof(cliBuffer) && c >= 32 && c <= 126) {
            if (!bufferIndex && c == ' ')
                continue; // Ignore leading spaces
            cliBuffer[bufferIndex++] = c;
            cliWrite(c);
        }
    }
}

void cliEnter(serialPort_t *serialPort)
{
    if (cliMode) {
        return;
    }

    cliMode = 1;
    cliPort = serialPort;
    setPrintfSerialPort(cliPort);
    cliWriter = bufWriterInit(cliWriteBuffer, sizeof(cliWriteBuffer), (bufWrite_t)serialWriteBufShim, serialPort);

#ifndef CLI_MINIMAL_VERBOSITY
    cliPrintLine("\r\nEntering CLI Mode, type 'exit' to return, or 'help'");
#else
    cliPrintLine("\r\nCLI");
#endif
    cliPrompt();

#ifdef USE_CLI_BATCH
    resetCommandBatch();
#endif

    ENABLE_ARMING_FLAG(ARMING_DISABLED_CLI);
}

void cliInit(const serialConfig_t *serialConfig)
{
    UNUSED(serialConfig);
}
