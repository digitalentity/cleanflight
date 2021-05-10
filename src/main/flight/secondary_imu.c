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

#include "stdint.h"

#include "build/debug.h"

#include "common/utils.h"
#include "common/axis.h"

#include "build/debug.h"
#include "scheduler/scheduler.h"
#include "config/parameter_group_ids.h"

#include "drivers/sensor.h"
#include "drivers/accgyro/accgyro_bno055.h"
#include "drivers/accgyro/accgyro_bno055_serial.h"

#include "fc/settings.h"

#include "flight/secondary_imu.h"

#include "sensors/boardalignment.h"
#include "sensors/compass.h"

PG_REGISTER_WITH_RESET_FN(secondaryImuConfig_t, secondaryImuConfig, PG_SECONDARY_IMU, 1);

EXTENDED_FASTRAM secondaryImuState_t secondaryImuState;

void pgResetFn_secondaryImuConfig(secondaryImuConfig_t *instance)
{
    instance->hardwareType = SECONDARY_IMU_NONE,
    instance->rollDeciDegrees = 0;
    instance->pitchDeciDegrees = 0;
    instance->yawDeciDegrees = 0;
    instance->useForOsdHeading = 0;
    instance->useForOsdAHI = 0;
    instance->useForStabilized = 0;

    for (uint8_t i = 0; i < 3; i++)
    {
        instance->calibrationOffsetGyro[i] = 0;
        instance->calibrationOffsetMag[i] = 0;
        instance->calibrationOffsetAcc[i] = 0;
    }
    instance->calibrationRadiusAcc = 0;
    instance->calibrationRadiusMag = 0;
}

void secondaryImuInit(void)
{
    secondaryImuState.active = false;
    // Create magnetic declination matrix
#ifdef USE_MAG
    const int deg = compassConfig()->mag_declination / 100;
    const int min = compassConfig()->mag_declination % 100;
#else
    const int deg = 0;
    const int min = 0;
#endif

    secondaryImuSetMagneticDeclination(deg + min / 60.0f);

    bno055CalibrationData_t calibrationData;
        calibrationData.offset[ACC][X] = secondaryImuConfig()->calibrationOffsetAcc[X];
        calibrationData.offset[ACC][Y] = secondaryImuConfig()->calibrationOffsetAcc[Y];
        calibrationData.offset[ACC][Z] = secondaryImuConfig()->calibrationOffsetAcc[Z];
        calibrationData.offset[MAG][X] = secondaryImuConfig()->calibrationOffsetMag[X];
        calibrationData.offset[MAG][Y] = secondaryImuConfig()->calibrationOffsetMag[Y];
        calibrationData.offset[MAG][Z] = secondaryImuConfig()->calibrationOffsetMag[Z];
        calibrationData.offset[GYRO][X] = secondaryImuConfig()->calibrationOffsetGyro[X];
        calibrationData.offset[GYRO][Y] = secondaryImuConfig()->calibrationOffsetGyro[Y];
        calibrationData.offset[GYRO][Z] = secondaryImuConfig()->calibrationOffsetGyro[Z];
        calibrationData.radius[ACC] = secondaryImuConfig()->calibrationRadiusAcc;
        calibrationData.radius[MAG] = secondaryImuConfig()->calibrationRadiusMag;

    if (secondaryImuConfig()->hardwareType == SECONDARY_IMU_BNO055) {
        secondaryImuState.active = bno055Init(calibrationData, (secondaryImuConfig()->calibrationRadiusAcc && secondaryImuConfig()->calibrationRadiusMag));

        if (secondaryImuState.active) {
            rescheduleTask(TASK_SECONDARY_IMU, TASK_PERIOD_HZ(10));
        }

    } else if (secondaryImuConfig()->hardwareType == SECONDARY_IMU_BNO055_SERIAL) {
        secondaryImuState.active = bno055SerialInit(calibrationData, (secondaryImuConfig()->calibrationRadiusAcc && secondaryImuConfig()->calibrationRadiusMag));

        if (secondaryImuState.active) {
            rescheduleTask(TASK_SECONDARY_IMU, TASK_PERIOD_HZ(50));
        }
    }

    if (!secondaryImuState.active) {
        secondaryImuConfigMutable()->hardwareType = SECONDARY_IMU_NONE;
    }

}

void secondaryImuProcess(void) {
    //Apply mag declination
    secondaryImuState.eulerAngles.raw[2] += secondaryImuState.magDeclination;

    //TODO this way of rotating a vector makes no sense, something simpler have to be developed
    const fpVector3_t v = {
        .x = secondaryImuState.eulerAngles.raw[0],
        .y = secondaryImuState.eulerAngles.raw[1],
        .z = secondaryImuState.eulerAngles.raw[2],
    };

    fpVector3_t rotated;

    fp_angles_t imuAngles = {
        .angles.roll = DECIDEGREES_TO_RADIANS(secondaryImuConfig()->rollDeciDegrees),
        .angles.pitch = DECIDEGREES_TO_RADIANS(secondaryImuConfig()->pitchDeciDegrees),
        .angles.yaw = DECIDEGREES_TO_RADIANS(secondaryImuConfig()->yawDeciDegrees),
    };
    fpMat3_t rotationMatrix;
    rotationMatrixFromAngles(&rotationMatrix, &imuAngles);
    rotationMatrixRotateVector(&rotated, &v, &rotationMatrix);
    rotated.z = ((int32_t)(rotated.z + secondaryImuConfig()->yawDeciDegrees)) % 3600;

    secondaryImuState.eulerAngles.values.roll = rotated.x;
    secondaryImuState.eulerAngles.values.pitch = rotated.y;
    secondaryImuState.eulerAngles.values.yaw = rotated.z;

    DEBUG_SET(DEBUG_IMU2, 0, secondaryImuState.eulerAngles.values.roll);
    DEBUG_SET(DEBUG_IMU2, 1, secondaryImuState.eulerAngles.values.pitch);
    DEBUG_SET(DEBUG_IMU2, 2, secondaryImuState.eulerAngles.values.yaw);

    DEBUG_SET(DEBUG_IMU2, 3, secondaryImuState.calibrationStatus.mag);
    DEBUG_SET(DEBUG_IMU2, 4, secondaryImuState.calibrationStatus.gyr);
    DEBUG_SET(DEBUG_IMU2, 5, secondaryImuState.calibrationStatus.acc);
    DEBUG_SET(DEBUG_IMU2, 6, secondaryImuState.magDeclination);
}

void taskSecondaryImu(timeUs_t currentTimeUs)
{
    static uint8_t tick = 0;
    tick++;

    if (!secondaryImuState.active)
    {
        return;
    }
    /*
     * Secondary IMU works in decidegrees
     */
    UNUSED(currentTimeUs);

    if (secondaryImuConfig()->hardwareType == SECONDARY_IMU_BNO055) {
        bno055FetchEulerAngles(secondaryImuState.eulerAngles.raw);
        secondaryImuProcess();

        /*
        * Every 2 seconds fetch current calibration state
        */
        if (tick == 20)
        {
            secondaryImuState.calibrationStatus = bno055GetCalibStat();
            tick = 0;
        }
    } else if (secondaryImuConfig()->hardwareType == SECONDARY_IMU_BNO055_SERIAL) {
        /*
         * Every 2 seconds fetch current calibration state
         */
        if (tick == 100)
        {
            bno055SerialGetCalibStat();
            tick = 0;
        } else {
            bno055SerialFetchEulerAngles();
        }
    }
}

void secondaryImuFetchCalibration(void) {
    bno055CalibrationData_t calibrationData;

    if (secondaryImuConfig()->hardwareType == SECONDARY_IMU_BNO055) {
        calibrationData = bno055GetCalibrationData();
    } else if (secondaryImuConfig()->hardwareType == SECONDARY_IMU_BNO055_SERIAL) {
        calibrationData = bno055SerialGetCalibrationData();
    } else {
        return;
    }

    secondaryImuConfigMutable()->calibrationOffsetAcc[X] = calibrationData.offset[ACC][X];
    secondaryImuConfigMutable()->calibrationOffsetAcc[Y] = calibrationData.offset[ACC][Y];
    secondaryImuConfigMutable()->calibrationOffsetAcc[Z] = calibrationData.offset[ACC][Z];

    secondaryImuConfigMutable()->calibrationOffsetMag[X] = calibrationData.offset[MAG][X];
    secondaryImuConfigMutable()->calibrationOffsetMag[Y] = calibrationData.offset[MAG][Y];
    secondaryImuConfigMutable()->calibrationOffsetMag[Z] = calibrationData.offset[MAG][Z];

    secondaryImuConfigMutable()->calibrationOffsetGyro[X] = calibrationData.offset[GYRO][X];
    secondaryImuConfigMutable()->calibrationOffsetGyro[Y] = calibrationData.offset[GYRO][Y];
    secondaryImuConfigMutable()->calibrationOffsetGyro[Z] = calibrationData.offset[GYRO][Z];

    secondaryImuConfigMutable()->calibrationRadiusAcc = calibrationData.radius[ACC];
    secondaryImuConfigMutable()->calibrationRadiusMag = calibrationData.radius[MAG];
}

void secondaryImuSetMagneticDeclination(float declination) { //Incoming units are degrees
    secondaryImuState.magDeclination = declination * 10.0f; //Internally declination is stored in decidegrees
}
