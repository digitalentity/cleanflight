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

/*
 * Authors:
 * Dominic Clifton - Cleanflight implementation
 * John Ihlein - Initial FF32 code
 * Konstantin Sharlaimov - busDevice refactoring
*/

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"

#include "drivers/system.h"
#include "drivers/time.h"
#include "drivers/io.h"
#include "drivers/exti.h"
#include "drivers/bus.h"

#include "drivers/sensor.h"
#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/accgyro/accgyro_mpu6000.h"

#if (defined(USE_GYRO_MPU6000) || defined(USE_ACC_MPU6000))

// Bits
#define BIT_H_RESET                 0x80
#define MPU_CLK_SEL_PLLGYROX        0x01
#define MPU_CLK_SEL_PLLGYROZ        0x03
#define BIT_I2C_IF_DIS              0x10
#define BIT_GYRO                    3
#define BIT_ACC                     2
#define BIT_TEMP                    1

// Product ID Description for MPU6000
// high 4 bits low 4 bits
// Product Name Product Revision
#define MPU6000ES_REV_C4 0x14
#define MPU6000ES_REV_C5 0x15
#define MPU6000ES_REV_D6 0x16
#define MPU6000ES_REV_D7 0x17
#define MPU6000ES_REV_D8 0x18
#define MPU6000_REV_C4 0x54
#define MPU6000_REV_C5 0x55
#define MPU6000_REV_D6 0x56
#define MPU6000_REV_D7 0x57
#define MPU6000_REV_D8 0x58
#define MPU6000_REV_D9 0x59
#define MPU6000_REV_D10 0x5A

static void mpu6000AccAndGyroInit(gyroDev_t *gyro)
{
    busDevice_t * busDev = gyro->busDev;
    const gyroFilterAndRateConfig_t * config = mpuChooseGyroConfig(gyro->lpf, 1000000 / gyro->requestedSampleIntervalUs);
    gyro->sampleRateIntervalUs = 1000000 / config->gyroRateHz;

    gyroIntExtiInit(gyro);

    busSetSpeed(busDev, BUS_SPEED_INITIALIZATION);

    // Device Reset
    busWrite(busDev, MPU_RA_PWR_MGMT_1, BIT_H_RESET);
    delay(150);

    busWrite(busDev, MPU_RA_SIGNAL_PATH_RESET, BIT_GYRO | BIT_ACC | BIT_TEMP);
    delay(150);

    // Clock Source PPL with Z axis gyro reference
    busWrite(busDev, MPU_RA_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ);
    delayMicroseconds(15);

    // Disable Primary I2C Interface
    busWrite(busDev, MPU_RA_USER_CTRL, BIT_I2C_IF_DIS);
    delayMicroseconds(15);

    busWrite(busDev, MPU_RA_PWR_MGMT_2, 0x00);
    delayMicroseconds(15);

    // Accel Sample Rate 1kHz
    // Gyroscope Output Rate =  1kHz when the DLPF is enabled
    busWrite(busDev, MPU_RA_SMPLRT_DIV, config->gyroConfigValues[1]);
    delayMicroseconds(15);

    // Gyro +/- 2000 DPS Full Scale
    busWrite(busDev, MPU_RA_GYRO_CONFIG, INV_FSR_2000DPS << 3);
    delayMicroseconds(15);

    // Accel +/- 16 G Full Scale
    busWrite(busDev, MPU_RA_ACCEL_CONFIG, INV_FSR_16G << 3);
    delayMicroseconds(15);

    busWrite(busDev, MPU_RA_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 1 << 4 | 0 << 3 | 0 << 2 | 0 << 1 | 0 << 0);  // INT_ANYRD_2CLEAR
    delayMicroseconds(15);

#ifdef USE_MPU_DATA_READY_SIGNAL
    busWrite(busDev, MPU_RA_INT_ENABLE, MPU_RF_DATA_RDY_EN);
    delayMicroseconds(15);
#endif

    // Accel and Gyro DLPF Setting
    busWrite(busDev, MPU_RA_CONFIG, config->gyroConfigValues[0]);
    delayMicroseconds(1);

    busSetSpeed(busDev, BUS_SPEED_FAST);

    mpuGyroRead(gyro);

    if (((int8_t)gyro->gyroADCRaw[1]) == -1 && ((int8_t)gyro->gyroADCRaw[0]) == -1) {
        failureMode(FAILURE_GYRO_INIT_FAILED);
    }
}

static void mpu6000AccInit(accDev_t *acc)
{
    acc->acc_1G = 512 * 4;
}

bool mpu6000AccDetect(accDev_t *acc)
{
    acc->busDev = busDeviceOpen(BUSTYPE_ANY, DEVHW_MPU6000, acc->imuSensorToUse);
    if (acc->busDev == NULL) {
        return false;
    }

    mpuContextData_t * ctx = busDeviceGetScratchpadMemory(acc->busDev);
    if (ctx->chipMagicNumber != 0x6860) {
        return false;
    }

    acc->initFn = mpu6000AccInit;
    acc->readFn = mpuAccReadScratchpad;

    return true;
}

static bool mpu6000DeviceDetect(busDevice_t * busDev)
{
    uint8_t in;
    uint8_t attemptsRemaining = 5;

    busSetSpeed(busDev, BUS_SPEED_INITIALIZATION);

    busWrite(busDev, MPU_RA_PWR_MGMT_1, BIT_H_RESET);

    do {
        delay(150);

        busRead(busDev, MPU_RA_WHO_AM_I, &in);
        if (in == MPU6000_WHO_AM_I_CONST) {
            break;
        }
        if (!attemptsRemaining) {
            return false;
        }
    } while (attemptsRemaining--);

    busRead(busDev, MPU_RA_PRODUCT_ID, &in);

    /* look for a product ID we recognise */
    switch (in) {
        case MPU6000ES_REV_C4:
        case MPU6000ES_REV_C5:
        case MPU6000_REV_C4:
        case MPU6000_REV_C5:
        case MPU6000ES_REV_D6:
        case MPU6000ES_REV_D7:
        case MPU6000ES_REV_D8:
        case MPU6000_REV_D6:
        case MPU6000_REV_D7:
        case MPU6000_REV_D8:
        case MPU6000_REV_D9:
        case MPU6000_REV_D10:
            return true;
    }

    return false;
}

bool mpu6000GyroDetect(gyroDev_t *gyro)
{
    gyro->busDev = busDeviceInit(BUSTYPE_ANY, DEVHW_MPU6000, gyro->imuSensorToUse, OWNER_MPU);
    if (gyro->busDev == NULL) {
        return false;
    }

    if (!mpu6000DeviceDetect(gyro->busDev)) {
        busDeviceDeInit(gyro->busDev);
        return false;
    }

    // Magic number for ACC detection to indicate that we have detected MPU6000 gyro
    mpuContextData_t * ctx = busDeviceGetScratchpadMemory(gyro->busDev);
    ctx->chipMagicNumber = 0x6860;

    gyro->initFn = mpu6000AccAndGyroInit;
    gyro->readFn = mpuGyroReadScratchpad;
    gyro->temperatureFn = mpuTemperatureReadScratchpad;
    gyro->scale = 1.0f / 16.4f;     // 16.4 dps/lsb scalefactor

    return true;
}

#endif