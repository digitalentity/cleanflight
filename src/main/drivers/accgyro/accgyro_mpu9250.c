/*
 * This file is part of INAV.
 *
 * INAV is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * INAV is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with INAV.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#include "common/axis.h"
#include "common/maths.h"

#include "drivers/system.h"
#include "drivers/time.h"
#include "drivers/exti.h"

#include "drivers/sensor.h"
#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/accgyro/accgyro_mpu9250.h"

#if defined(USE_GYRO_MPU9250) || defined(USE_ACC_MPU9250)

#define MPU9250_BIT_RESET                   (0x80)
#define MPU9250_BIT_INT_ANYRD_2CLEAR        (1 << 4)
#define MPU9250_BIT_BYPASS_EN               (1 << 0)
#define MPU9250_BIT_I2C_IF_DIS              (1 << 4)
#define MPU9250_BIT_RAW_RDY_EN              (0x01)

static void mpu9250AccInit(accDev_t *acc)
{
    acc->acc_1G = 512 * 4;
}

bool mpu9250AccDetect(accDev_t *acc)
{
    acc->busDev = busDeviceOpen(BUSTYPE_ANY, DEVHW_MPU9250, acc->imuSensorToUse);
    if (acc->busDev == NULL) {
        return false;
    }

    mpuContextData_t * ctx = busDeviceGetScratchpadMemory(acc->busDev);
    if (ctx->chipMagicNumber != 0x9250) {
        return false;
    }

    acc->initFn = mpu9250AccInit;
    acc->readFn = mpuAccReadScratchpad;

    return true;
}

static void mpu9250AccAndGyroInit(gyroDev_t *gyro)
{
    busDevice_t * dev = gyro->busDev;
    const gyroFilterAndRateConfig_t * config = mpuChooseGyroConfig(gyro->lpf, 1000000 / gyro->requestedSampleIntervalUs);
    gyro->sampleRateIntervalUs = 1000000 / config->gyroRateHz;

    gyroIntExtiInit(gyro);

    busSetSpeed(dev, BUS_SPEED_INITIALIZATION);

    busWrite(dev, MPU_RA_PWR_MGMT_1, MPU9250_BIT_RESET);
    delay(100);

    busWrite(dev, MPU_RA_SIGNAL_PATH_RESET, 0x07);      // BIT_GYRO | BIT_ACC | BIT_TEMP
    delay(100);

    busWrite(dev, MPU_RA_PWR_MGMT_1, 0);
    delay(100);

    busWrite(dev, MPU_RA_PWR_MGMT_1, INV_CLK_PLL);
    delay(15);

    busWrite(dev, MPU_RA_GYRO_CONFIG, INV_FSR_2000DPS << 3 | FCB_DISABLED);
    delay(15);

    busWrite(dev, MPU_RA_ACCEL_CONFIG, INV_FSR_16G << 3);
    delay(15);

    busWrite(dev, MPU_RA_CONFIG, config->gyroConfigValues[0]);
    delay(15);

    busWrite(dev, MPU_RA_SMPLRT_DIV, config->gyroConfigValues[1]);
    delay(100);

    // Data ready interrupt configuration
    busWrite(dev, MPU_RA_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 1 << 4 | 0 << 3 | 0 << 2 | 1 << 1 | 0 << 0);  // INT_ANYRD_2CLEAR, BYPASS_EN
    delay(15);

#ifdef USE_MPU_DATA_READY_SIGNAL
    busWrite(dev, MPU_RA_INT_ENABLE, MPU_RF_DATA_RDY_EN);
    delay(15);
#endif

    busSetSpeed(dev, BUS_SPEED_FAST);
}

static bool mpu9250DeviceDetect(busDevice_t * dev)
{
    uint8_t tmp;
    uint8_t attemptsRemaining = 5;

    busSetSpeed(dev, BUS_SPEED_INITIALIZATION);

    busWrite(dev, MPU_RA_PWR_MGMT_1, MPU9250_BIT_RESET);

    do {
        delay(150);

        busRead(dev, MPU_RA_WHO_AM_I, &tmp);

        switch (tmp) {
            case MPU9250_WHO_AM_I_CONST:
                // Compatible chip detected
                return true;

            default:
                // Retry detection
                break;
        }

        if (!attemptsRemaining) {
            return false;
        }
    } while (attemptsRemaining--);

    return false;
}

bool mpu9250GyroDetect(gyroDev_t *gyro)
{
    gyro->busDev = busDeviceInit(BUSTYPE_ANY, DEVHW_MPU9250, gyro->imuSensorToUse, OWNER_MPU);
    if (gyro->busDev == NULL) {
        return false;
    }

    if (!mpu9250DeviceDetect(gyro->busDev)) {
        busDeviceDeInit(gyro->busDev);
        return false;
    }

    // Magic number for ACC detection to indicate that we have detected MPU6000 gyro
    mpuContextData_t * ctx = busDeviceGetScratchpadMemory(gyro->busDev);
    ctx->chipMagicNumber = 0x9250;

    gyro->initFn = mpu9250AccAndGyroInit;
    gyro->readFn = mpuGyroReadScratchpad;
    gyro->temperatureFn = mpuTemperatureReadScratchpad;
    gyro->scale = 1.0f / 16.4f;     // 16.4 dps/lsb scalefactor

    return true;
}

#endif
