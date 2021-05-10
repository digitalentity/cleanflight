/*
 * This file is part of INAV.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Alternatively, the contents of this file may be used under the terms
 * of the GNU General Public License Version 3, as described below:
 *
 * This file is free software: you may copy, redistribute and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/.
 */
#pragma once

#include "common/vector.h"

#define BNO055_ADDR_PWR_MODE 0x3E
#define BNO055_ADDR_OPR_MODE 0x3D
#define BNO055_ADDR_CALIB_STAT 0x35

#define BNO055_PWR_MODE_NORMAL  0x00
#define BNO055_OPR_MODE_CONFIG  0x00
#define BNO055_OPR_MODE_NDOF    0x0C

#define BNO055_ADDR_EUL_YAW_LSB 0x1A
#define BNO055_ADDR_EUL_YAW_MSB 0x1B
#define BNO055_ADDR_EUL_ROLL_LSB 0x1C
#define BNO055_ADDR_EUL_ROLL_MSB 0x1D
#define BNO055_ADDR_EUL_PITCH_LSB 0x1E
#define BNO055_ADDR_EUL_PITCH_MSB 0x1F

#define BNO055_ADDR_MAG_RADIUS_MSB 0x6A 
#define BNO055_ADDR_MAG_RADIUS_LSB 0x69
#define BNO055_ADDR_ACC_RADIUS_MSB 0x68
#define BNO055_ADDR_ACC_RADIUS_LSB 0x67

#define BNO055_ADDR_GYR_OFFSET_Z_MSB 0x66
#define BNO055_ADDR_GYR_OFFSET_Z_LSB 0x65
#define BNO055_ADDR_GYR_OFFSET_Y_MSB 0x64
#define BNO055_ADDR_GYR_OFFSET_Y_LSB 0x63
#define BNO055_ADDR_GYR_OFFSET_X_MSB 0x62
#define BNO055_ADDR_GYR_OFFSET_X_LSB 0x61

#define BNO055_ADDR_MAG_OFFSET_Z_MSB 0x60
#define BNO055_ADDR_MAG_OFFSET_Z_LSB 0x5F
#define BNO055_ADDR_MAG_OFFSET_Y_MSB 0x5E
#define BNO055_ADDR_MAG_OFFSET_Y_LSB 0x5D
#define BNO055_ADDR_MAG_OFFSET_X_MSB 0x5C
#define BNO055_ADDR_MAG_OFFSET_X_LSB 0x5B

#define BNO055_ADDR_ACC_OFFSET_Z_MSB 0x5A
#define BNO055_ADDR_ACC_OFFSET_Z_LSB 0x59
#define BNO055_ADDR_ACC_OFFSET_Y_MSB 0x58
#define BNO055_ADDR_ACC_OFFSET_Y_LSB 0x57
#define BNO055_ADDR_ACC_OFFSET_X_MSB 0x56
#define BNO055_ADDR_ACC_OFFSET_X_LSB 0x55

typedef struct {
    uint8_t sys;
    uint8_t gyr;
    uint8_t acc;
    uint8_t mag;
} bno055CalibStat_t;

typedef enum {
    ACC = 0,
    MAG = 1,
    GYRO = 2
} bno055Sensor_e;

typedef struct {
    int16_t radius[2];
    int16_t offset[3][3];
} bno055CalibrationData_t;

bool bno055Init(bno055CalibrationData_t calibrationData, bool setCalibration);
fpVector3_t bno055GetEurlerAngles(void);
void bno055FetchEulerAngles(int16_t * buffer);
bno055CalibStat_t bno055GetCalibStat(void);
bno055CalibrationData_t bno055GetCalibrationData(void);