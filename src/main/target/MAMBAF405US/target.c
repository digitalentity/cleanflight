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
#include "drivers/io.h"
#include "drivers/timer.h"
#include "drivers/pwm_mapping.h"
#include "drivers/bus.h"

const timerHardware_t timerHardware[] = {
    DEF_TIM(TIM11,  CH1,  PB9,   TIM_USE_PPM,       0, 0 ),     // PPM IN

#ifdef MAMBAF405US_I2C
    DEF_TIM(TIM1,   CH2,  PA9,   TIM_USE_MC_MOTOR | TIM_USE_FW_MOTOR,  0, 1 ),     // S1 pin A9: DMA2 Stream 6 Channel 0
    DEF_TIM(TIM1,   CH1,  PA8,   TIM_USE_MC_MOTOR | TIM_USE_FW_MOTOR,  0, 1 ),     // S2 pin A8: DMA2 Stream 6 Channel 0
    DEF_TIM(TIM8,   CH4,  PC9,   TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO,  0, 0 ),     // S3 pin C9: DMA2 Stream 7 Channel 7
    DEF_TIM(TIM8,   CH3,  PC8,   TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO,  0, 1 ),     // S4 pin C8: DMA2 Stream 2 Channel 0
#else
    DEF_TIM(TIM1,   CH2,  PA9,   TIM_USE_MC_MOTOR | TIM_USE_FW_MOTOR,  0, 1 ),     // S4_OUT – D(2, 2, 6)
    DEF_TIM(TIM1,   CH1,  PA8,   TIM_USE_MC_MOTOR | TIM_USE_FW_MOTOR,  0, 1 ),     // S3_OUT – D(2, 1, 6)
    DEF_TIM(TIM8,   CH4,  PC9,   TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO,  0, 0 ),     // S2_OUT – D(2, 7, 7)
    DEF_TIM(TIM8,   CH3,  PC8,   TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO,  0, 0 ),     // S1_OUT – D(2, 4, 7)
#endif

    DEF_TIM(TIM2,   CH2,  PB3,   TIM_USE_LED,       0, 0 ),     // LED_STRIP – D(1, 6, 3)
};

const int timerHardwareCount = sizeof(timerHardware) / sizeof(timerHardware[0]);
