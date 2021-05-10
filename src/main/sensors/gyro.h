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

#include "common/axis.h"
#include "common/maths.h"
#include "common/vector.h"
#include "common/time.h"
#include "config/parameter_group.h"
#include "drivers/sensor.h"

typedef enum {
    GYRO_NONE = 0,
    GYRO_AUTODETECT,
    GYRO_MPU6050,
    GYRO_L3G4200D,
    GYRO_MPU3050,
    GYRO_L3GD20,
    GYRO_MPU6000,
    GYRO_MPU6500,
    GYRO_MPU9250,
    GYRO_BMI160,
    GYRO_ICM20689,
    GYRO_BMI088,
    GYRO_FAKE
} gyroSensor_e;

typedef enum {
    DYN_NOTCH_RANGE_HIGH = 0,
    DYN_NOTCH_RANGE_MEDIUM,
    DYN_NOTCH_RANGE_LOW
} dynamicFilterRange_e;

#define DYN_NOTCH_RANGE_HZ_HIGH 2000
#define DYN_NOTCH_RANGE_HZ_MEDIUM 1333
#define DYN_NOTCH_RANGE_HZ_LOW 1000

typedef struct gyro_s {
    bool initialized;
    uint32_t targetLooptime;
    float gyroADCf[XYZ_AXIS_COUNT];
} gyro_t;

extern gyro_t gyro;

typedef struct gyroConfig_s {
    sensor_align_e gyro_align;              // gyro alignment
    uint8_t  gyroMovementCalibrationThreshold; // people keep forgetting that moving model while init results in wrong gyro offsets. and then they never reset gyro. so this is now on by default.
    uint16_t looptime;                      // imu loop time in us
    uint8_t  gyro_lpf;                      // gyro LPF setting - values are driver specific, in case of invalid number, a reasonable default ~30-40HZ is chosen.
    uint8_t  gyro_anti_aliasing_lpf_hz;
    uint8_t  gyro_anti_aliasing_lpf_type;
#ifdef USE_DUAL_GYRO
    uint8_t  gyro_to_use;
#endif
    uint16_t gyro_notch_hz;
    uint16_t gyro_notch_cutoff;
    uint16_t gyro_main_lpf_hz;
    uint8_t gyro_main_lpf_type;
    uint8_t useDynamicLpf;
    uint16_t gyroDynamicLpfMinHz;
    uint16_t gyroDynamicLpfMaxHz;
    uint8_t gyroDynamicLpfCurveExpo;
#ifdef USE_DYNAMIC_FILTERS
    uint8_t dynamicGyroNotchRange;
    uint16_t dynamicGyroNotchQ;
    uint16_t dynamicGyroNotchMinHz;
    uint8_t dynamicGyroNotchEnabled;
#endif
#ifdef USE_ALPHA_BETA_GAMMA_FILTER
    float alphaBetaGammaAlpha;
    float alphaBetaGammaBoost;
    float alphaBetaGammaHalfLife;
#endif
} gyroConfig_t;

PG_DECLARE(gyroConfig_t, gyroConfig);

bool gyroInit(void);
void gyroGetMeasuredRotationRate(fpVector3_t *imuMeasuredRotationBF);
void gyroUpdate(void);
void gyroFilter(void);
void gyroStartCalibration(void);
bool gyroIsCalibrationComplete(void);
bool gyroReadTemperature(void);
int16_t gyroGetTemperature(void);
int16_t gyroRateDps(int axis);
void gyroUpdateDynamicLpf(float cutoffFreq);
