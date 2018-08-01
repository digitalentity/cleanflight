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

#include <stdint.h>

#include <platform.h>
#include "drivers/io.h"
#include "drivers/bus.h"
#include "drivers/pwm_mapping.h"
#include "drivers/timer.h"

/* GYRO */
BUSDEV_REGISTER_SPI_TAG(busdev_mpu6000,     DEVHW_MPU6000,          MPU6000_SPI_BUS,    MPU6000_CS_PIN,     MPU6000_EXTI_PIN,       0,  DEVFLAGS_NONE);

BUSDEV_REGISTER_I2C(    busdev_ms5611,      DEVHW_MS5611,           BARO_I2C_BUS,       0x77,               NONE,                       DEVFLAGS_USE_RAW_REGISTERS);

BUSDEV_REGISTER_I2C(    busdev_eagle_tree,  DEVHW_PITOT_EAGLE_TREE, EAGLE_TREE_I2C_BUS, 0x4D,               NONE,                       DEVFLAGS_USE_RAW_REGISTERS);

BUSDEV_REGISTER_I2C_TAG(busdev_hmc5883_int, DEVHW_HMC5883,          MAG_I2C_BUS_INT,    0x1E,               NONE,                   0,  DEVFLAGS_NONE);

BUSDEV_REGISTER_I2C_TAG(busdev_hmc5883,     DEVHW_HMC5883,          MAG_I2C_BUS_EXT,    0x1E,               NONE,                   1,  DEVFLAGS_NONE);
BUSDEV_REGISTER_I2C_TAG(busdev_qmc5883,     DEVHW_QMC5883,          MAG_I2C_BUS_EXT,    0x0D,               NONE,                   1,  DEVFLAGS_NONE);
BUSDEV_REGISTER_I2C_TAG(busdev_mag3110,     DEVHW_MAG3110,          MAG_I2C_BUS_EXT,    0x0E,               NONE,                   1,  DEVFLAGS_NONE);

BUSDEV_REGISTER_SPI    (busdev_m25p16,      DEVHW_M25P16,           M25P16_SPI_BUS,     M25P16_CS_PIN,      NONE,                       DEVFLAGS_NONE);

/* TIMERS */
const timerHardware_t timerHardware[] = {
    { TIM3,  IO_TAG(PB0),  TIM_Channel_3, 1, IOCFG_AF_PP_PD, GPIO_AF_TIM3,  TIM_USE_MC_MOTOR | TIM_USE_FW_MOTOR }, // S1_OUT
    { TIM3,  IO_TAG(PB1),  TIM_Channel_4, 1, IOCFG_AF_PP_PD, GPIO_AF_TIM3,  TIM_USE_MC_MOTOR | TIM_USE_FW_MOTOR }, // S2_OUT
    { TIM9,  IO_TAG(PA3),  TIM_Channel_2, 1, IOCFG_AF_PP_PD, GPIO_AF_TIM9,  TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO }, // S3_OUT
    { TIM2,  IO_TAG(PA2),  TIM_Channel_3, 1, IOCFG_AF_PP_PD, GPIO_AF_TIM2,  TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO }, // S4_OUT
    { TIM5,  IO_TAG(PA1),  TIM_Channel_2, 1, IOCFG_AF_PP_PD, GPIO_AF_TIM5,  TIM_USE_MC_MOTOR | TIM_USE_MC_SERVO | TIM_USE_FW_SERVO | TIM_USE_ANY }, // S5_OUT - GPIO_PartialRemap_TIM3
    { TIM5,  IO_TAG(PA0),  TIM_Channel_1, 1, IOCFG_AF_PP_PD, GPIO_AF_TIM5,  TIM_USE_MC_MOTOR | TIM_USE_MC_SERVO | TIM_USE_FW_SERVO }, // S6_OUT

    { TIM12, IO_TAG(PB14), TIM_Channel_1, 0, IOCFG_AF_PP_PD, GPIO_AF_TIM12, TIM_USE_PWM | TIM_USE_PPM },            // PWM1 - PPM (5th pin on FlexiIO port)
    { TIM12, IO_TAG(PB15), TIM_Channel_2, 0, IOCFG_AF_PP_PD, GPIO_AF_TIM12, TIM_USE_PWM },                          // PWM2 - S2_IN - GPIO_PartialRemap_TIM3
    { TIM8,  IO_TAG(PC6),  TIM_Channel_1, 0, IOCFG_AF_PP_PD, GPIO_AF_TIM8,  TIM_USE_PWM | TIM_USE_FW_SERVO },       // PWM3 - S3_IN
    { TIM8,  IO_TAG(PC7),  TIM_Channel_2, 0, IOCFG_AF_PP_PD, GPIO_AF_TIM8,  TIM_USE_PWM | TIM_USE_FW_SERVO },       // PWM4 - S4_IN
    { TIM8,  IO_TAG(PC8),  TIM_Channel_3, 0, IOCFG_AF_PP_PD, GPIO_AF_TIM8,  TIM_USE_PWM | TIM_USE_FW_SERVO },       // PWM5 - S5_IN
    { TIM8,  IO_TAG(PC9),  TIM_Channel_4, 0, IOCFG_AF_PP_PD, GPIO_AF_TIM8,  TIM_USE_PWM | TIM_USE_FW_SERVO },       // PWM6 - S6_IN
};

const int timerHardwareCount = sizeof(timerHardware) / sizeof(timerHardware[0]);
