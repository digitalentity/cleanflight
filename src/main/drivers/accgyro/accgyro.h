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

#include "platform.h"
#include "common/axis.h"
#include "drivers/exti.h"
#include "drivers/sensor.h"
#include "scheduler/scheduler.h"

#define GYRO_LPF_256HZ      0
#define GYRO_LPF_188HZ      1
#define GYRO_LPF_98HZ       2
#define GYRO_LPF_42HZ       3
#define GYRO_LPF_20HZ       4
#define GYRO_LPF_10HZ       5
#define GYRO_LPF_5HZ        6
#define GYRO_LPF_NONE       7

typedef struct {
    uint8_t gyroLpf;
    uint16_t gyroRateHz;
    uint8_t gyroConfigValues[2];
} gyroFilterAndRateConfig_t;

typedef struct gyroDev_s {
    busDevice_t * busDev;
    sensorGyroInitFuncPtr initFn;                       // initialize function
    sensorGyroReadFuncPtr readFn;                       // read 3 axis data function
    sensorGyroReadDataFuncPtr temperatureFn;            // read temperature if available
    sensorGyroUpdateFuncPtr updateFn;
    extiCallbackRec_t exti;
    schdSemaphore_t dataReadySemaphore;
    float scale;                                        // scalefactor
    int16_t gyroADCRaw[XYZ_AXIS_COUNT];
    int16_t gyroZero[XYZ_AXIS_COUNT];
    uint8_t imuSensorToUse;
    uint8_t lpf;                                        // Configuration value: Hardware LPF setting
    uint32_t requestedSampleIntervalUs;                 // Requested sample interval
    uint32_t sampleRateIntervalUs;                      // Gyro driver should set this to actual sampling rate as signaled by IRQ
    sensor_align_e gyroAlign;
} gyroDev_t;

typedef struct accDev_s {
    busDevice_t * busDev;
    sensorAccInitFuncPtr initFn;                        // initialize function
    sensorAccReadFuncPtr readFn;                        // read 3 axis data function
    uint16_t acc_1G;
    int16_t ADCRaw[XYZ_AXIS_COUNT];
    uint8_t imuSensorToUse;
    sensor_align_e accAlign;
} accDev_t;

const gyroFilterAndRateConfig_t * chooseGyroConfig(uint8_t desiredLpf, uint16_t desiredRateHz, const gyroFilterAndRateConfig_t * configs, int count);
void gyroIntExtiInit(struct gyroDev_s *gyro);
bool gyroCheckDataReady(struct gyroDev_s *gyro);
