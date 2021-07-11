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

#include "platform.h"

#include "drivers/bus.h"
#include "drivers/io.h"
#include "drivers/pwm_mapping.h"
#include "drivers/timer.h"

const timerHardware_t timerHardware[] = {
    DEF_TIM(TIM9, CH2, PA3, TIM_USE_PPM,                         0, 0),     // PPM

    DEF_TIM(TIM3, CH1, PB4, TIM_USE_MC_MOTOR | TIM_USE_FW_MOTOR, 0, 0),     // S1 D(1, 4, 5)
    DEF_TIM(TIM3, CH2, PB5, TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO, 0, 0),     // S2 D(2, 3, 7)
    DEF_TIM(TIM3, CH3, PB0, TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO, 0, 0),     // S3 D(2, 4, 7)
    DEF_TIM(TIM3, CH4, PB1, TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO, 0, 0),     // S4 D(2, 7, 7)
    DEF_TIM(TIM4, CH1, PB6, TIM_USE_MC_MOTOR | TIM_USE_FW_MOTOR, 0, 0),     // S5 DMA1_ST2
    DEF_TIM(TIM4, CH2, PB7, TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO, 0, 0),     // S6 DMA2_ST6

    DEF_TIM(TIM1, CH1, PA8, TIM_USE_LED, 1, 0),                            // LED STRIP  D(1, 5, 3)

};

const int timerHardwareCount = sizeof(timerHardware) / sizeof(timerHardware[0]);
