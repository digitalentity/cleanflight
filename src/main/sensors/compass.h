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
#include "common/time.h"

#include "config/parameter_group.h"

#include "drivers/compass/compass.h"

#include "sensors/sensors.h"

// Type of magnetometer used/detected
typedef enum {
    MAG_NONE = 0,
    MAG_AUTODETECT = 1,
    MAG_HMC5883 = 2,
    MAG_AK8975 = 3,
    MAG_GPS = 4,
    MAG_MAG3110 = 5,
    MAG_AK8963 = 6,
    MAG_IST8310 = 7,
    MAG_QMC5883 = 8,
    MAG_MPU9250 = 9,
    MAG_IST8308 = 10,
    MAG_LIS3MDL = 11,
    MAG_MSP = 12,
    MAG_RM3100 = 13,
    MAG_FAKE = 14,
    MAG_MAX = MAG_FAKE
} magSensor_e;

typedef struct mag_s {
    magDev_t dev;
    int32_t magADC[XYZ_AXIS_COUNT];
} mag_t;

extern mag_t mag;

#ifdef USE_MAG

typedef struct compassConfig_s {
    int16_t mag_declination;                // Get your magnetic declination from here : http://magnetic-declination.com/
                                            // For example, -6deg 37min, = -637 Japan, format is [sign]dddmm (degreesminutes) default is zero.
    sensor_align_e mag_align;               // on-board mag alignment. Ignored if externally aligned via *DeciDegrees.
    uint8_t mag_hardware;                   // Which mag hardware to use on boards with more than one device
    flightDynamicsTrims_t magZero;
    int16_t magGain[XYZ_AXIS_COUNT];
#ifdef USE_DUAL_MAG
    uint8_t mag_to_use;
#endif
    uint8_t magCalibrationTimeLimit;        // Time for compass calibration (seconds)
    int16_t rollDeciDegrees;                // Alignment for external mag on the roll (X) axis (0.1deg)
    int16_t pitchDeciDegrees;               // Alignment for external mag on the pitch (Y) axis (0.1deg)
    int16_t yawDeciDegrees;                 // Alignment for external mag on the yaw (Z) axis (0.1deg)
} compassConfig_t;

PG_DECLARE(compassConfig_t, compassConfig);

bool compassDetect(magDev_t *dev, magSensor_e magHardwareToUse);
bool compassInit(void);
void compassUpdate(timeUs_t currentTimeUs);
bool compassIsReady(void);
bool compassIsHealthy(void);

#endif
