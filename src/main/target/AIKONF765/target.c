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

BUSDEV_REGISTER_SPI_TAG(busdev_imu0,    DEVHW_MPU6000,      MPU6000_SPI_BUS,    MPU6000_CS_PIN,     MPU6000_EXTI_PIN,       0,  DEVFLAGS_NONE,  IMU_MPU6000_ALIGN);
BUSDEV_REGISTER_SPI_TAG(busdev_imu1,    DEVHW_MPU6500,      MPU6500_SPI_BUS,    MPU6500_CS_PIN,     MPU6500_EXTI_PIN,       1,  DEVFLAGS_NONE,  IMU_MPU6500_ALIGN);

const timerHardware_t timerHardware[] = {
    DEF_TIM(TIM2,  CH1, PA0,   TIM_USE_MC_MOTOR | TIM_USE_FW_MOTOR, 0, 0), // TIM2_CH1, TIM5_CH1
    DEF_TIM(TIM2,  CH2, PA1,   TIM_USE_MC_MOTOR | TIM_USE_FW_MOTOR, 0, 0), // TIM2_CH2, TIM5_CH2
    DEF_TIM(TIM5,  CH3, PA2,   TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO, 0, 0), // TIM2_CH3, TIM5_CH3, TIM9_CH1
    DEF_TIM(TIM5,  CH4, PA3,   TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO, 0, 0), // TIM2_CH4, TIM5_CH4, TIM9_CH2
    DEF_TIM(TIM3,  CH3, PB0,   TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO, 0, 0), // TIM1_CH2N, TIM3_CH3, TIM8_CH2N
    DEF_TIM(TIM3,  CH4, PB1,   TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO, 0, 0), // TIM1_CH3N, TIM3_CH4, TIM8_CH3N
    DEF_TIM(TIM4,  CH1, PD12,  TIM_USE_MC_SERVO | TIM_USE_FW_SERVO, 0, 0), // TIM4_CH1
    DEF_TIM(TIM4,  CH2, PD13,  TIM_USE_MC_SERVO | TIM_USE_FW_SERVO, 0, 0), // TIM4_CH2
    DEF_TIM(TIM4,  CH3, PD14,  TIM_USE_MC_SERVO | TIM_USE_FW_SERVO, 0, 0), // TIM4_CH3
    DEF_TIM(TIM4,  CH4, PD15,  TIM_USE_MC_SERVO | TIM_USE_FW_SERVO, 0, 0), // TIM4_CH4

    DEF_TIM(TIM1,  CH4, PE14,  TIM_USE_LED                        , 0, 0), // TIM1_CH1
};

const int timerHardwareCount = sizeof(timerHardware) / sizeof(timerHardware[0]);
