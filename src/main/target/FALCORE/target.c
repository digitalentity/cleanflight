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
#include "drivers/pwm_mapping.h"
#include "drivers/timer.h"

const uint16_t multiPPM[] = {
    PWM7  | (MAP_TO_PPM_INPUT << 8), // PPM input

    PWM1  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM8
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM8
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM8
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM8
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM3
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM3
    0xFFFF
};

const uint16_t multiPWM[] = {
    PWM1  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM8
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM8
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM8
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM8
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM3
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM3
    0xFFFF
};

const uint16_t airPPM[] = {
    // TODO
    0xFFFF
};

const uint16_t airPWM[] = {
    // TODO
    0xFFFF
};


const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    // MOTOR outputs
    { TIM8,  IO_TAG(PC6),  TIM_Channel_1, TIM8_CC_IRQn,            1, IOCFG_AF_PP, GPIO_AF_4}, // PWM1 - PC6 - TIM8_CH1
    { TIM8,  IO_TAG(PC7),  TIM_Channel_2, TIM8_CC_IRQn,            1, IOCFG_AF_PP, GPIO_AF_4}, // PWM2 - PC7 - TIM8_CH2
    { TIM8,  IO_TAG(PC8),  TIM_Channel_3, TIM8_CC_IRQn,            1, IOCFG_AF_PP, GPIO_AF_4}, // PWM3 - PC8 - TIM8_CH3
    { TIM8,  IO_TAG(PC9),  TIM_Channel_4, TIM8_CC_IRQn,            1, IOCFG_AF_PP, GPIO_AF_4}, // PWM4 - PC9 - TIM8_CH4

    // Additional servo outputs
    { TIM3,  IO_TAG(PA4),  TIM_Channel_2, TIM3_IRQn,               0, IOCFG_AF_PP, GPIO_AF_2}, // PWM5 - PA4  - TIM3_CH2
    { TIM3,  IO_TAG(PB1),  TIM_Channel_4, TIM3_IRQn,               0, IOCFG_AF_PP, GPIO_AF_2}, // PWM6 - PB1  - TIM3_CH4

    // PPM PORT - Also USART2 RX (AF5)
    { TIM2,  IO_TAG(PA3),  TIM_Channel_4, TIM2_IRQn,               0, IOCFG_AF_PP, GPIO_AF_1}, // PPM - PA3  - TIM2_CH4

    // LED_STRIP
    { TIM1,  IO_TAG(PA8),  TIM_Channel_1, TIM1_CC_IRQn,            0, IOCFG_AF_PP, GPIO_AF_6},  // GPIO_TIMER / LED_STRIP

    // PWM_BUZZER
    { TIM16, IO_TAG(PB4),  TIM_Channel_1, TIM1_CC_IRQn,            0, IOCFG_AF_PP, GPIO_AF_1},  // BUZZER - PB4 - TIM16_CH1N
};

