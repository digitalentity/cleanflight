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

#include "config/parameter_group.h"
#include "drivers/time.h"

#ifndef VBAT_SCALE_DEFAULT
#define VBAT_SCALE_DEFAULT 1100
#endif
#define VBAT_SCALE_MIN 0
#define VBAT_SCALE_MAX 65535

#ifndef CURRENT_METER_SCALE
#define CURRENT_METER_SCALE 400 // for Allegro ACS758LCB-100U (40mV/A)
#endif

#ifndef CURRENT_METER_OFFSET
#define CURRENT_METER_OFFSET 0
#endif

#ifndef MAX_BATTERY_PROFILE_COUNT
#define MAX_BATTERY_PROFILE_COUNT 3
#endif

typedef enum {
    CURRENT_SENSOR_NONE = 0,
    CURRENT_SENSOR_ADC,
    CURRENT_SENSOR_VIRTUAL,
    CURRENT_SENSOR_ESC,
    CURRENT_SENSOR_MAX = CURRENT_SENSOR_VIRTUAL
} currentSensor_e;

typedef enum {
    VOLTAGE_SENSOR_NONE = 0,
    VOLTAGE_SENSOR_ADC,
    VOLTAGE_SENSOR_ESC
} voltageSensor_e;

typedef enum {
    BAT_CAPACITY_UNIT_MAH,
    BAT_CAPACITY_UNIT_MWH,
} batCapacityUnit_e;

typedef struct {
  uint8_t profile_index;
  uint16_t max_voltage;
} profile_comp_t;

typedef enum {
    BAT_VOLTAGE_RAW,
    BAT_VOLTAGE_SAG_COMP
} batVoltageSource_e;

typedef struct batteryMetersConfig_s {

#ifdef USE_ADC
    struct {
        uint16_t scale;
        voltageSensor_e type;
    } voltage;
#endif

    struct {
        int16_t scale;          // scale the current sensor output voltage to milliamps. Value in 1/10th mV/A
        int16_t offset;         // offset of the current sensor in millivolt steps
        currentSensor_e type;   // type of current meter used, either ADC or virtual
    } current;

    batVoltageSource_e voltageSource;

    uint32_t cruise_power;      // power drawn by the motor(s) at cruise throttle/speed (cW)
    uint16_t idle_power;        // power drawn by the system when the motor(s) are stopped (cW)
    uint8_t rth_energy_margin;  // Energy that should be left after RTH (%), used for remaining time/distance before RTH

    float throttle_compensation_weight;

} batteryMetersConfig_t;

typedef struct batteryProfile_s {

#ifdef USE_ADC
    uint8_t cells;

    struct {
        uint16_t cellDetect;    // maximum voltage per cell, used for auto-detecting battery cell count in 0.01V units, default is 430 (4.3V)
        uint16_t cellMax;       // maximum voltage per cell, used for auto-detecting battery voltage in 0.01V units, default is 421 (4.21V)
        uint16_t cellMin;       // minimum voltage per cell, this triggers battery critical alarm, in 0.01V units, default is 330 (3.3V)
        uint16_t cellWarning;   // warning voltage per cell, this triggers battery warning alarm, in 0.01V units, default is 350 (3.5V)
    } voltage;
#endif

    struct {
        uint32_t value;         // mAh or mWh (see capacity.unit)
        uint32_t warning;       // mAh or mWh (see capacity.unit)
        uint32_t critical;      // mAh or mWh (see capacity.unit)
        batCapacityUnit_e unit; // Describes unit of capacity.value, capacity.warning and capacity.critical
    } capacity;

} batteryProfile_t;

PG_DECLARE(batteryMetersConfig_t, batteryMetersConfig);
PG_DECLARE_ARRAY(batteryProfile_t, MAX_BATTERY_PROFILE_COUNT, batteryProfiles);

extern const batteryProfile_t *currentBatteryProfile;

typedef enum {
    BATTERY_OK = 0,
    BATTERY_WARNING,
    BATTERY_CRITICAL,
    BATTERY_NOT_PRESENT
} batteryState_e;


uint16_t batteryAdcToVoltage(uint16_t src);
batteryState_e getBatteryState(void);
bool batteryWasFullWhenPluggedIn(void);
bool batteryUsesCapacityThresholds(void);
void batteryInit(void);
void setBatteryProfile(uint8_t profileIndex);
void activateBatteryProfile(void);
void batteryDisableProfileAutoswitch(void);

bool isBatteryVoltageConfigured(void);
bool isPowerSupplyImpedanceValid(void);
uint16_t getBatteryVoltage(void);
uint16_t getBatteryRawVoltage(void);
uint16_t getBatterySagCompensatedVoltage(void);
uint16_t getBatteryWarningVoltage(void);
uint8_t getBatteryCellCount(void);
uint16_t getBatteryRawAverageCellVoltage(void);
uint16_t getBatteryAverageCellVoltage(void);
uint16_t getBatterySagCompensatedAverageCellVoltage(void);
uint32_t getBatteryRemainingCapacity(void);
uint16_t getPowerSupplyImpedance(void);

bool isAmperageConfigured(void);
int16_t getAmperage(void);
int16_t getAmperageSample(void);
int32_t getPower(void);
int32_t getMAhDrawn(void);
int32_t getMWhDrawn(void);

#ifdef USE_ADC
void batteryUpdate(timeUs_t timeDelta);
void sagCompensatedVBatUpdate(timeUs_t currentTime, timeUs_t timeDelta);
void powerMeterUpdate(timeUs_t timeDelta);
uint16_t getVBatSample(void);
#endif

void currentMeterUpdate(timeUs_t timeDelta);

uint8_t calculateBatteryPercentage(void);
float calculateThrottleCompensationFactor(void);
int32_t calculateAveragePower(void);
int32_t calculateAverageEfficiency(void);
int32_t heatLossesCompensatedPower(int32_t power);
