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

#include <stdbool.h>
#include <stdint.h>
#include "drivers/bus.h"
#include "drivers/time.h"
#include "build/debug.h"
#include "common/vector.h"
#include "drivers/accgyro/accgyro_bno055.h"

#ifdef USE_IMU_BNO055

static busDevice_t *busDev;

static bool deviceDetect(busDevice_t *busDev)
{
    for (int retry = 0; retry < 5; retry++)
    {
        uint8_t sig;

        delay(150);

        bool ack = busRead(busDev, 0x00, &sig);
        if (ack)
        {
            return true;
        }
    };

    return false;
}

static void bno055SetMode(uint8_t mode) 
{
    busWrite(busDev, BNO055_ADDR_OPR_MODE, mode);
    delay(25);
}

static void bno055SetCalibrationData(bno055CalibrationData_t data) 
{
    uint8_t buf[12];

    //Prepare gains
    //We do not restore gyro offsets, they are quickly calibrated at startup
    uint8_t bufferBit = 0;
    for (uint8_t sensorIndex = 0; sensorIndex < 2; sensorIndex++)
    {
        for (uint8_t axisIndex = 0; axisIndex < 3; axisIndex++)
        {
            buf[bufferBit] = (uint8_t)(data.offset[sensorIndex][axisIndex] & 0xff);
            buf[bufferBit + 1] = (uint8_t)((data.offset[sensorIndex][axisIndex] >> 8 ) & 0xff);
            bufferBit += 2;
        }
    }

    busWriteBuf(busDev, BNO055_ADDR_ACC_OFFSET_X_LSB, buf, 12);

    //Prepare radius
    buf[0] = (uint8_t)(data.radius[ACC] & 0xff);
    buf[1] = (uint8_t)((data.radius[ACC] >> 8 ) & 0xff);
    buf[2] = (uint8_t)(data.radius[MAG] & 0xff);
    buf[3] = (uint8_t)((data.radius[MAG] >> 8 ) & 0xff);

    //Write to the device
    busWriteBuf(busDev, BNO055_ADDR_ACC_RADIUS_LSB, buf, 4);
}

bool bno055Init(bno055CalibrationData_t calibrationData, bool setCalibration)
{
    busDev = busDeviceInit(BUSTYPE_I2C, DEVHW_BNO055, 0, 0);
    if (busDev == NULL)
    {
        return false;
    }

    if (!deviceDetect(busDev))
    {
        busDeviceDeInit(busDev);
        return false;
    }

    busWrite(busDev, BNO055_ADDR_PWR_MODE, BNO055_PWR_MODE_NORMAL); //Set power mode NORMAL
    delay(25);
    if (setCalibration) {
        bno055SetMode(BNO055_OPR_MODE_CONFIG);
        bno055SetCalibrationData(calibrationData);
    }
    bno055SetMode(BNO055_OPR_MODE_NDOF);

    return true;
}

void bno055FetchEulerAngles(int16_t *buffer)
{
    uint8_t buf[6];
    busReadBuf(busDev, BNO055_ADDR_EUL_YAW_LSB, buf, 6);

    buffer[0] = ((int16_t)((buf[3] << 8) | buf[2])) / 1.6f;
    buffer[1] = ((int16_t)((buf[5] << 8) | buf[4])) / -1.6f; //Pitch has to be reversed to match INAV notation
    buffer[2] = ((int16_t)((buf[1] << 8) | buf[0])) / 1.6f;
}

fpVector3_t bno055GetEurlerAngles(void)
{
    fpVector3_t eurlerAngles;

    uint8_t buf[6];
    busReadBuf(busDev, BNO055_ADDR_EUL_YAW_LSB, buf, 6);

    eurlerAngles.x = ((int16_t)((buf[3] << 8) | buf[2])) / 16;
    eurlerAngles.y = ((int16_t)((buf[5] << 8) | buf[4])) / 16;
    eurlerAngles.z = ((int16_t)((buf[1] << 8) | buf[0])) / 16;

    return eurlerAngles;
}

bno055CalibStat_t bno055GetCalibStat(void)
{
    bno055CalibStat_t stats;
    uint8_t buf;

    busRead(busDev, BNO055_ADDR_CALIB_STAT, &buf);

    stats.mag = buf & 0b00000011;
    stats.acc = (buf >> 2) & 0b00000011;
    stats.gyr = (buf >> 4) & 0b00000011;
    stats.sys = (buf >> 6) & 0b00000011;

    return stats;
}

bno055CalibrationData_t bno055GetCalibrationData(void)
{
    bno055CalibrationData_t data;
    uint8_t buf[22];

    bno055SetMode(BNO055_OPR_MODE_CONFIG);

    busReadBuf(busDev, BNO055_ADDR_ACC_OFFSET_X_LSB, buf, 22);

    bno055SetMode(BNO055_OPR_MODE_NDOF);

    uint8_t bufferBit = 0;
    for (uint8_t sensorIndex = 0; sensorIndex < 3; sensorIndex++)
    {
        for (uint8_t axisIndex = 0; axisIndex < 3; axisIndex++)
        {
            data.offset[sensorIndex][axisIndex] = (int16_t)((buf[bufferBit + 1] << 8) | buf[bufferBit]);
            bufferBit += 2;
        }
    }

    data.radius[ACC] = (int16_t)((buf[19] << 8) | buf[18]);
    data.radius[MAG] = (int16_t)((buf[21] << 8) | buf[20]);

    return data;
}

#endif