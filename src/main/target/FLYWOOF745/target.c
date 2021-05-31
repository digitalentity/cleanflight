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

#include <platform.h>
#include "drivers/io.h"
#include "drivers/pwm_mapping.h"
#include "drivers/timer.h"
#include "drivers/bus.h"

const timerHardware_t timerHardware[] = {
    DEF_TIM(TIM1, CH3, PE13, TIM_USE_PPM,                                               0, 1), // PPM, DMA2_ST6

    DEF_TIM(TIM3, CH3, PB0,  TIM_USE_MC_MOTOR | TIM_USE_FW_MOTOR,                       0, 0), // M1 , DMA1_ST7
    DEF_TIM(TIM3, CH4, PB1,  TIM_USE_MC_MOTOR | TIM_USE_FW_MOTOR,                       0, 0), // M2 , DMA1_ST2
    DEF_TIM(TIM1, CH1, PE9,  TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO,                       0, 2), // M3 , DMA2_ST2
    DEF_TIM(TIM1, CH2, PE11, TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO,                       0, 1), // M4 , DMA2_ST4
    DEF_TIM(TIM8, CH4, PC9,  TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO | TIM_USE_MC_SERVO,    0, 0), // M5 , DMA2_ST7
    DEF_TIM(TIM5, CH4, PA3,  TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO | TIM_USE_MC_SERVO,    0, 0), // M6 , DMA1_ST1
    DEF_TIM(TIM3, CH1, PB4,  TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO | TIM_USE_MC_SERVO,    0, 0), // M7 , DMA1_ST4
    DEF_TIM(TIM3, CH2, PB5,  TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO | TIM_USE_MC_SERVO,    0, 0), // M8 , DMA1_ST5
    DEF_TIM(TIM4, CH1, PD12, TIM_USE_LED,                                               0, 0), // LED_STRIP, DMA1_ST0
};

const int timerHardwareCount = sizeof(timerHardware) / sizeof(timerHardware[0]);
