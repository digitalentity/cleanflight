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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "common/utils.h"

#include "drivers/io.h"
#include "drivers/rcc.h"
#include "drivers/time.h"
#include "drivers/nvic.h"
#include "drivers/timer.h"
#include "drivers/timer_impl.h"

#include "stm32h7xx.h"

const timerDef_t timerDefinitions[HARDWARE_TIMER_DEFINITION_COUNT] = {
    [0] = { .tim = TIM1,  .rcc = RCC_APB2(TIM1),   .irq = TIM1_CC_IRQn},
    [1] = { .tim = TIM2,  .rcc = RCC_APB1L(TIM2),  .irq = TIM2_IRQn},
    [2] = { .tim = TIM3,  .rcc = RCC_APB1L(TIM3),  .irq = TIM3_IRQn},
    [3] = { .tim = TIM4,  .rcc = RCC_APB1L(TIM4),  .irq = TIM4_IRQn},
    [4] = { .tim = TIM5,  .rcc = RCC_APB1L(TIM5),  .irq = TIM5_IRQn},
    [5] = { .tim = TIM6,  .rcc = RCC_APB1L(TIM6),  .irq = 0},
    [6] = { .tim = TIM7,  .rcc = RCC_APB1L(TIM7),  .irq = 0},
    [7] = { .tim = TIM8,  .rcc = RCC_APB2(TIM8),   .irq = TIM8_CC_IRQn},
    [8] = { .tim = TIM12, .rcc = RCC_APB1L(TIM12), .irq = TIM8_BRK_TIM12_IRQn},
    [9] = { .tim = TIM13, .rcc = RCC_APB1L(TIM13), .irq = TIM8_UP_TIM13_IRQn},
    [10] = { .tim = TIM14, .rcc = RCC_APB1L(TIM14), .irq = TIM8_TRG_COM_TIM14_IRQn},
    [11] = { .tim = TIM15, .rcc = RCC_APB2(TIM15),  .irq = TIM15_IRQn},
    [12] = { .tim = TIM16, .rcc = RCC_APB2(TIM16),  .irq = TIM16_IRQn},
    [13] = { .tim = TIM17, .rcc = RCC_APB2(TIM17),  .irq = TIM17_IRQn},
};

uint32_t timerClock(TIM_TypeDef *tim)
{
    int timpre;
    uint32_t pclk;
    uint32_t ppre;

    // Implement the table:
    // RM0433 (Rev 6) Table 52.
    // RM0455 (Rev 3) Table 55.
    // "Ratio between clock timer and pclk"
    // (Tables are the same, just D2 or CD difference)

#if defined(STM32H743xx) || defined(STM32H750xx) || defined(STM32H723xx) || defined(STM32H725xx)
#define PERIPH_PRESCALER(bus) ((RCC->D2CFGR & RCC_D2CFGR_D2PPRE ## bus) >> RCC_D2CFGR_D2PPRE ## bus ## _Pos)
#elif defined(STM32H7A3xx) || defined(STM32H7A3xxQ)
#define PERIPH_PRESCALER(bus) ((RCC->CDCFGR2 & RCC_CDCFGR2_CDPPRE ## bus) >> RCC_CDCFGR2_CDPPRE ## bus ## _Pos)
#else
#error Unknown MCU type
#endif

    if (tim == TIM1 || tim == TIM8 || tim == TIM15 || tim == TIM16 || tim == TIM17) {
        // Timers on APB2
        pclk = HAL_RCC_GetPCLK2Freq();
        ppre = PERIPH_PRESCALER(2);
    } else {
        // Timers on APB1
        pclk = HAL_RCC_GetPCLK1Freq();
        ppre = PERIPH_PRESCALER(1);
    }

    timpre = (RCC->CFGR & RCC_CFGR_TIMPRE) ? 1 : 0;

    int index = (timpre << 3) | ppre;

    static uint8_t periphToKernel[16] = { // The mutiplier table
        1, 1, 1, 1, 2, 2, 2, 2, // TIMPRE = 0
        1, 1, 1, 1, 2, 4, 4, 4  // TIMPRE = 1
    };

    return pclk * periphToKernel[index];

#undef PERIPH_PRESCALER
}

_TIM_IRQ_HANDLER(TIM1_CC_IRQHandler, 1);
_TIM_IRQ_HANDLER(TIM2_IRQHandler, 2);
_TIM_IRQ_HANDLER(TIM3_IRQHandler, 3);
_TIM_IRQ_HANDLER(TIM4_IRQHandler, 4);
_TIM_IRQ_HANDLER(TIM5_IRQHandler, 5);
_TIM_IRQ_HANDLER(TIM8_CC_IRQHandler, 8);
_TIM_IRQ_HANDLER(TIM8_BRK_TIM12_IRQHandler, 12);
_TIM_IRQ_HANDLER(TIM15_IRQHandler, 15);
_TIM_IRQ_HANDLER(TIM16_IRQHandler, 16);
_TIM_IRQ_HANDLER(TIM17_IRQHandler, 17);
_TIM_IRQ_HANDLER2(TIM8_UP_TIM13_IRQHandler, 8, 13);
_TIM_IRQ_HANDLER2(TIM8_TRG_COM_TIM14_IRQHandler, 8, 14);
