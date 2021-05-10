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

#include "build/atomic.h"

#include "common/utils.h"

#include "drivers/io.h"
#include "drivers/rcc.h"
#include "drivers/time.h"
#include "drivers/nvic.h"
#include "drivers/timer.h"
#include "drivers/timer_impl.h"

#include "stm32f4xx.h"

const timerDef_t timerDefinitions[HARDWARE_TIMER_DEFINITION_COUNT] = {
#if defined(TIM1)
    [0] = { .tim = TIM1,  .rcc = RCC_APB2(TIM1),  .irq = TIM1_CC_IRQn, .secondIrq = TIM1_UP_TIM10_IRQn },
#endif

#if defined(TIM2)
    [1] = { .tim = TIM2,  .rcc = RCC_APB1(TIM2),  .irq = TIM2_IRQn},
#endif

#if defined(TIM3)
    [2] = { .tim = TIM3,  .rcc = RCC_APB1(TIM3),  .irq = TIM3_IRQn},
#endif

#if defined(TIM4)
    [3] = { .tim = TIM4,  .rcc = RCC_APB1(TIM4),  .irq = TIM4_IRQn},
#endif

#if defined(TIM5)
    [4] = { .tim = TIM5,  .rcc = RCC_APB1(TIM5),  .irq = TIM5_IRQn},
#endif

#if defined(TIM6)
    [5] = { .tim = TIM6,  .rcc = RCC_APB1(TIM6),  .irq = 0},
#endif

#if defined(TIM7)
    [6] = { .tim = TIM7,  .rcc = RCC_APB1(TIM7),  .irq = 0},
#endif

#if defined(TIM8) && !defined(STM32F411xE)
#if defined(STM32F446xx)
    [7] = { .tim = TIM8,  .rcc = RCC_APB2(TIM8),  .irq = 0 },
#else
    [7] = { .tim = TIM8,  .rcc = RCC_APB2(TIM8),  .irq = TIM8_CC_IRQn, .secondIrq = TIM8_UP_TIM13_IRQn },
#endif
#endif

#if defined(TIM9)
    [8] = { .tim = TIM9,  .rcc = RCC_APB2(TIM9),  .irq = TIM1_BRK_TIM9_IRQn},
#endif

#if defined(TIM10)
    [9] = { .tim = TIM10, .rcc = RCC_APB2(TIM10), .irq = TIM1_UP_TIM10_IRQn},
#endif

#if defined(TIM11)
    [10] = { .tim = TIM11, .rcc = RCC_APB2(TIM11), .irq = TIM1_TRG_COM_TIM11_IRQn},
#endif

#if defined(TIM12) && !defined(STM32F411xE)
    [11] = { .tim = TIM12, .rcc = RCC_APB1(TIM12), .irq = TIM8_BRK_TIM12_IRQn},
#endif

#if defined(TIM13) && !defined(STM32F411xE)
    [12] = { .tim = TIM13, .rcc = RCC_APB1(TIM13), .irq = TIM8_UP_TIM13_IRQn},
#endif

#if defined(TIM14) && !defined(STM32F411xE)
    [13] = { .tim = TIM14, .rcc = RCC_APB1(TIM14), .irq = TIM8_TRG_COM_TIM14_IRQn},
#endif
};

uint32_t timerClock(TIM_TypeDef *tim)
{
#if defined (STM32F411xE)
    UNUSED(tim);
    return SystemCoreClock;
#elif defined (STM32F40_41xxx) || defined (STM32F427_437xx) || defined (STM32F446xx)
    if (tim == TIM1 || tim == TIM8 || tim == TIM9 || tim == TIM10 || tim == TIM11) {
        return SystemCoreClock;
    } else {
        return SystemCoreClock / 2;
    }
#else
    #error "No timer clock defined correctly for the MCU"
#endif
}

// Timer IRQ handlers
_TIM_IRQ_HANDLER(TIM1_CC_IRQHandler, 1);
_TIM_IRQ_HANDLER2(TIM1_BRK_TIM9_IRQHandler, 1, 9);
_TIM_IRQ_HANDLER2(TIM1_UP_TIM10_IRQHandler, 1, 10);
_TIM_IRQ_HANDLER2(TIM1_TRG_COM_TIM11_IRQHandler, 1, 11);
_TIM_IRQ_HANDLER(TIM2_IRQHandler, 2);
_TIM_IRQ_HANDLER(TIM3_IRQHandler, 3);
_TIM_IRQ_HANDLER(TIM4_IRQHandler, 4);
_TIM_IRQ_HANDLER(TIM5_IRQHandler, 5);

#if defined(TIM8) && !defined(STM32F411xE)
_TIM_IRQ_HANDLER(TIM8_CC_IRQHandler, 8);
_TIM_IRQ_HANDLER(TIM8_UP_IRQHandler, 8);
#endif

#if defined(TIM12) && !defined(STM32F411xE)
_TIM_IRQ_HANDLER2(TIM8_BRK_TIM12_IRQHandler, 8, 12);
#endif

#if defined(TIM13) && !defined(STM32F411xE)
_TIM_IRQ_HANDLER2(TIM8_UP_TIM13_IRQHandler, 8, 13);
#endif

#if defined(TIM14) && !defined(STM32F411xE)
_TIM_IRQ_HANDLER2(TIM8_TRG_COM_TIM14_IRQHandler, 8, 14);
#endif