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
#include <string.h>
#include <stdint.h>

#include "platform.h"

#include "common/maths.h"

#include "drivers/system.h"
#include "drivers/time.h"
#include "drivers/exti.h"
#include "drivers/bus_i2c.h"

#include "drivers/sensor.h"
#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/accgyro/accgyro_mpu3050.h"

#ifdef USE_GYRO_MPU3050

// MPU3050, Standard address 0x68
#define MPU3050_ADDRESS         0x68

// Bits
#define MPU3050_FS_SEL_2000DPS  0x18
#define MPU3050_DLPF_10HZ       0x05
#define MPU3050_DLPF_20HZ       0x04
#define MPU3050_DLPF_42HZ       0x03
#define MPU3050_DLPF_98HZ       0x02
#define MPU3050_DLPF_188HZ      0x01
#define MPU3050_DLPF_256HZ      0x00

#define MPU3050_SMPLRT_DIV      0x15
#define MPU3050_DLPF_FS_SYNC    0x16
#define MPU3050_INT_CFG         0x17
#define MPU3050_TEMP_OUT        0x1B
#define MPU3050_GYRO_OUT        0x1D
#define MPU3050_USER_CTRL       0x3D
#define MPU3050_PWR_MGM         0x3E

#define MPU3050_USER_RESET      0x01
#define MPU3050_CLK_SEL_PLL_GX  0x01

#define MPU_INQUIRY_MASK        0x7E

static void mpu3050Init(gyroDev_t *gyro)
{
    bool ack;
    busDevice_t * busDev = gyro->busDev;
    const gyroFilterAndRateConfig_t * config = mpuChooseGyroConfig(gyro->lpf, 1000000 / gyro->requestedSampleIntervalUs);
    gyro->sampleRateIntervalUs = 1000000 / config->gyroRateHz;

    delay(25); // datasheet page 13 says 20ms. other stuff could have been running meanwhile. but we'll be safe

    ack = busWrite(busDev, MPU3050_SMPLRT_DIV, config->gyroConfigValues[1]);
    if (!ack) {
        failureMode(FAILURE_ACC_INIT);
    }

    busWrite(busDev, MPU3050_DLPF_FS_SYNC, MPU3050_FS_SEL_2000DPS | config->gyroConfigValues[0]);
    busWrite(busDev, MPU3050_INT_CFG, 0);
    busWrite(busDev, MPU3050_USER_CTRL, MPU3050_USER_RESET);
    busWrite(busDev, MPU3050_PWR_MGM, MPU3050_CLK_SEL_PLL_GX);
}

static bool mpu3050GyroRead(gyroDev_t *gyro)
{
    uint8_t data[6];

    const bool ack = busReadBuf(gyro->busDev, MPU3050_GYRO_OUT, data, 6);
    if (!ack) {
        return false;
    }

    gyro->gyroADCRaw[X] = (int16_t)((data[0] << 8) | data[1]);
    gyro->gyroADCRaw[Y] = (int16_t)((data[2] << 8) | data[3]);
    gyro->gyroADCRaw[Z] = (int16_t)((data[4] << 8) | data[5]);

    return true;
}

static bool deviceDetect(busDevice_t * busDev)
{
    busSetSpeed(busDev, BUS_SPEED_INITIALIZATION);

    for (int retry = 0; retry < 5; retry++) {
        uint8_t inquiryResult;

        delay(150);

        bool ack = busRead(busDev, MPU_RA_WHO_AM_I_LEGACY, &inquiryResult);
        inquiryResult &= MPU_INQUIRY_MASK;
        if (ack && inquiryResult == MPUx0x0_WHO_AM_I_CONST) {
            return true;
        }
    };

    return false;
}

bool mpu3050Detect(gyroDev_t *gyro)
{
    gyro->busDev = busDeviceInit(BUSTYPE_ANY, DEVHW_MPU3050, gyro->imuSensorToUse, OWNER_MPU);
    if (gyro->busDev == NULL) {
        return false;
    }

    if (!deviceDetect(gyro->busDev)) {
        busDeviceDeInit(gyro->busDev);
        return false;
    }

    gyro->initFn = mpu3050Init;
    gyro->readFn = mpu3050GyroRead;
    gyro->scale = 1.0f / 16.4f;     // 16.4 dps/lsb scalefactor

    return true;
}

#endif
