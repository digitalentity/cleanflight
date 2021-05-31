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
#include "drivers/bus.h"
#include "drivers/io.h"
#include "drivers/pwm_mapping.h"
#include "drivers/timer.h"
#include "drivers/pinio.h"

const timerHardware_t timerHardware[] = {
    DEF_TIM(TIM3,  CH1, PB4,  TIM_USE_MC_MOTOR | TIM_USE_FW_MOTOR, 0, 0), // S1  D(1,4,5)
    DEF_TIM(TIM3,  CH2, PB5,  TIM_USE_MC_MOTOR | TIM_USE_FW_MOTOR, 0, 0), // S2  D(1,5,5)
    DEF_TIM(TIM1,  CH1, PA8,  TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO, 0, 1), // S3  D(2,1,6)
    DEF_TIM(TIM1,  CH2, PA9,  TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO, 0, 1), // S4  D(2,2,6)
    DEF_TIM(TIM1,  CH3, PA10, TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO, 0, 1), // S5  D(2,6,6)
#ifndef MATEKF411SE_SS2_CH6
    DEF_TIM(TIM4,  CH3, PB8,  TIM_USE_MC_SERVO | TIM_USE_FW_SERVO, 0, 0), // S6  D(1,7,2)

    DEF_TIM(TIM5,  CH3, PA2,  TIM_USE_ANY, 0, 0), //TX2 pad - softserial_tx2
#else
    DEF_TIM(TIM4,  CH3, PB8,  TIM_USE_ANY, 0, 0), //CH6 pad - softserial 2
#endif

    DEF_TIM(TIM11, CH1, PB9,  TIM_USE_ANY, 0, 0), //ST1 pad -softserial_tx1    

    DEF_TIM(TIM9,  CH2, PA3,  TIM_USE_PPM, 0, 0), //RX2 Pad -PPM
#ifndef MATEKF411SE_PINIO
    DEF_TIM(TIM2,  CH3, PB10, TIM_USE_ANY, 0, 0), // softserial_rx1 - LED 2812   D(1,1,3)
#endif

};

const int timerHardwareCount = sizeof(timerHardware) / sizeof(timerHardware[0]);
