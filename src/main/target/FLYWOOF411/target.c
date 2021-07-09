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
#include <platform.h>
#include "drivers/io.h"
#include "drivers/dma.h"
#include "drivers/timer.h"
#include "drivers/bus.h"
#include "drivers/pwm_mapping.h"


const timerHardware_t timerHardware[] = {
    DEF_TIM(TIM9, CH1, PA2,   TIM_USE_PPM,   0, 0), // PPM IN
#ifdef FLYWOOF411_V2
    DEF_TIM(TIM1,  CH1, PA8,  TIM_USE_MC_MOTOR | TIM_USE_FW_MOTOR, 0, 1),      // S1 - D(2,1)
    DEF_TIM(TIM2,  CH2, PB3,  TIM_USE_MC_MOTOR | TIM_USE_FW_MOTOR, 0, 0),      // S2 - D(1,6)
    DEF_TIM(TIM2,  CH3, PB10, TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO, 0, 0),      // S3 - D(1,1)
    DEF_TIM(TIM2,  CH1, PA15, TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO, 0, 0),      // S4 - D(1,5)
    DEF_TIM(TIM4,  CH1, PB6,  TIM_USE_ANY, 0, 0),      // SOFTSERIAL_1_TX_PIN - D(1,0)
    DEF_TIM(TIM4,  CH2, PB7,  TIM_USE_ANY, 0, 0),      // SOFTSERIAL_1_RX_PIN - D(1,3)
    DEF_TIM(TIM3,  CH3, PB0,  TIM_USE_ANY, 0, 0), //  RSSI_ADC_CHANNEL 1-7
    DEF_TIM(TIM3,  CH1, PB4,  TIM_USE_ANY, 0, 0), //   1-4
    DEF_TIM(TIM5,  CH1, PA0,  TIM_USE_LED, 0, 0), //  LED    1,2

#elif defined(FLYWOOF411HEX)
    DEF_TIM(TIM1,  CH1, PA8,  TIM_USE_MC_MOTOR | TIM_USE_FW_MOTOR, 0, 1),      // S1 - D(2,1)
    DEF_TIM(TIM2,  CH2, PB3,  TIM_USE_MC_MOTOR | TIM_USE_FW_MOTOR, 0, 0),      // S2 - D(1,6)
    DEF_TIM(TIM2,  CH3, PB10, TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO, 0, 0),      // S3 - D(1,1)
    DEF_TIM(TIM2,  CH1, PA15, TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO, 0, 0),      // S4 - D(1,5)  
    DEF_TIM(TIM4,  CH1, PB6,  TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO, 0, 0),      // S5 - D(1,0) 
    DEF_TIM(TIM4,  CH2, PB7,  TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO, 0, 0),      // S6 - D(1,3)
    DEF_TIM(TIM3,  CH3, PB0,  TIM_USE_ANY,   0, 0), //  SOFTSERIAL_1_RX_PIN 1-7
    DEF_TIM(TIM3,  CH1, PB4,  TIM_USE_ANY,   0, 0), //  SOFTSERIAL_1_TX_PIN 1-4
    DEF_TIM(TIM5,  CH1, PA0,  TIM_USE_LED,   0, 0), //  LED    1,2

#else
    DEF_TIM(TIM1, CH1, PA8,  TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO,  0, 1), // S1_OUT 2,1
    DEF_TIM(TIM1, CH2, PA9,  TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO,  0, 1), // S2_OUT 2,2
    DEF_TIM(TIM1, CH3, PA10, TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO,  0, 0), // S3_OUT 2,6
    DEF_TIM(TIM3, CH3, PB0,  TIM_USE_MC_MOTOR | TIM_USE_FW_MOTOR,  0, 0), // S4_OUT 1,7

    DEF_TIM(TIM3, CH4, PB1,  TIM_USE_ANY,   0, 0), //  RSSI   1,2
    DEF_TIM(TIM5, CH4, PA3,  TIM_USE_ANY,   0, 1), //  RX2    1,0
    DEF_TIM(TIM2, CH1, PA15, TIM_USE_LED,   0, 0), //  LED    1,5
#endif
};

const int timerHardwareCount = sizeof(timerHardware) / sizeof(timerHardware[0]);
