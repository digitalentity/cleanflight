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

#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/persistent.h"
#include "drivers/exti.h"
#include "drivers/nvic.h"
#include "drivers/system.h"

void forcedSystemResetWithoutDisablingCaches(void)
{
    persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_NONE);
    __disable_irq();
    NVIC_SystemReset();    
}

void enableGPIOPowerUsageAndNoiseReductions(void)
{
	__HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_D2SRAM1_CLK_ENABLE();
    __HAL_RCC_D2SRAM2_CLK_ENABLE();
    __HAL_RCC_D2SRAM3_CLK_ENABLE();
}

bool isMPUSoftReset(void)
{
    if (cachedRccCsrValue & RCC_RSR_SFTRSTF)
        return true;
    else
        return false;
}

uint32_t systemBootloaderAddress(void)
{
#if defined(STM32H743xx) || defined(STM32H750xx) || defined(STM32H723xx) || defined(STM32H725xx)
    return 0x1ff09800;
#else
#error Unknown MCU
#endif
}

void systemClockSetup(uint8_t cpuUnderclock)
{
    (void)cpuUnderclock;
    // This is a stub
}

void systemInit(void)
{
    checkForBootLoaderRequest();

    // Configure NVIC preempt/priority groups
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITY_GROUPING);

    // cache RCC->CSR value to use it in isMPUSoftreset() and others
    cachedRccCsrValue = RCC->CSR;

    enableGPIOPowerUsageAndNoiseReductions();

    // Init cycle counter
    cycleCounterInit();

    // SysTick
    HAL_SYSTICK_Config(SystemCoreClock / 1000);
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
}
