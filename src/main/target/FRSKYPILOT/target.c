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

#include <stdint.h>

#include "platform.h"

#include "drivers/bus.h"
#include "drivers/io.h"
#include "drivers/pwm_mapping.h"
#include "drivers/timer.h"
#include "drivers/pinio.h"
#include "drivers/sensor.h"

BUSDEV_REGISTER_SPI_TAG(busdev_mpu6500,     DEVHW_MPU6500,      MPU6500_SPI_BUS,    MPU6500_CS_PIN,     MPU6500_EXTI_PIN,       0,  DEVFLAGS_NONE,  IMU_MPU6500_ALIGN);
BUSDEV_REGISTER_SPI_TAG(busdev_mpu6000,     DEVHW_MPU6000,      MPU6000_SPI_BUS,    MPU6000_CS_PIN,     MPU6000_EXTI_PIN,       1,  DEVFLAGS_NONE,  IMU_MPU6000_ALIGN);

const timerHardware_t timerHardware[] = {
    DEF_TIM(TIM3,  CH3, PB0,  TIM_USE_MC_SERVO | TIM_USE_FW_MOTOR, 0, 0), // S1
    DEF_TIM(TIM3,  CH4, PB1,  TIM_USE_MC_SERVO | TIM_USE_FW_MOTOR, 0, 0), // S2
    DEF_TIM(TIM1,  CH1, PE9,  TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO, 0, 0), // S3
    DEF_TIM(TIM1,  CH2, PE11, TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO, 0, 0), // S4
    DEF_TIM(TIM1,  CH3, PE13, TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO, 0, 0), // S5
    DEF_TIM(TIM1,  CH4, PE14, TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO, 0, 0), // S6
    DEF_TIM(TIM4,  CH2, PB7,  TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO, 0, 0), // S7
    DEF_TIM(TIM4,  CH3, PB8,  TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO, 0, 0), // S8
    DEF_TIM(TIM4,  CH4, PB9,  TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO, 0, 0), // S9
#ifdef FRSKYPILOT_LED
    DEF_TIM(TIM2,  CH2, PA1,  TIM_USE_LED | TIM_USE_LED, 0, 0), // S10 now LED, S11 & S12 UART 3 only
#else
    DEF_TIM(TIM2,  CH2, PA1,  TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO, 0, 0), // S10
    DEF_TIM(TIM2,  CH3, PB10, TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO, 0, 0), // S11
    DEF_TIM(TIM2,  CH4, PB11, TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO, 0, 0), // S12
#endif

    DEF_TIM(TIM5, CH1,  PA0,  TIM_USE_BEEPER, 0, 0),    // beeper
};

const int timerHardwareCount = sizeof(timerHardware) / sizeof(timerHardware[0]);
