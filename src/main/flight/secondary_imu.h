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

#include "config/parameter_group.h"
#include "common/time.h"
#include "sensors/sensors.h"
#include "drivers/accgyro/accgyro_bno055.h"
#include "drivers/accgyro/accgyro_bno055_serial.h"

typedef enum {
    SECONDARY_IMU_NONE          = 0,
    SECONDARY_IMU_BNO055        = 1,
    SECONDARY_IMU_BNO055_SERIAL = 2,
} secondaryImuType_e;

typedef struct secondaryImuConfig_s {
    uint8_t hardwareType;
    int16_t rollDeciDegrees;
    int16_t pitchDeciDegrees;
    int16_t yawDeciDegrees;
    uint8_t useForOsdHeading;
    uint8_t useForOsdAHI;
    uint8_t useForStabilized;
    int16_t calibrationOffsetGyro[3];
    int16_t calibrationOffsetMag[3];
    int16_t calibrationOffsetAcc[3];
    int16_t calibrationRadiusAcc;
    int16_t calibrationRadiusMag;
} secondaryImuConfig_t;

typedef struct secondaryImuState_s {
    flightDynamicsTrims_t eulerAngles;
    bno055CalibStat_t calibrationStatus;
    uint8_t active;
    float magDeclination;
} secondaryImuState_t;

extern secondaryImuState_t secondaryImuState;

PG_DECLARE(secondaryImuConfig_t, secondaryImuConfig);

void secondaryImuProcess(void);
void secondaryImuInit(void);
void taskSecondaryImu(timeUs_t currentTimeUs);
void secondaryImuFetchCalibration(void);
void secondaryImuSetMagneticDeclination(float declination);