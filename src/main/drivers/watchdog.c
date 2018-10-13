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
 *
 * @author Alberto Garcia Hierro <alberto@garciahierro.com>
 */

#include "platform.h"

#if defined(USE_WATCHDOG)

#include "common/time.h"

#include "drivers/watchdog.h"

#define LSI_FREQ 32000 // 32khz
#define RLR_BITS 12
#define RLR_MASK ((1 << RLR_BITS)-1)

#define IWDG_PR_DIV256  (6)
#define IWDG_PR_DIV128  (5)
#define IWDG_PR_DIV64   (4)
#define IWDG_PR_DIV32   (3)
#define IWDG_PR_DIV16   (2)
#define IWDG_PR_DIV8    (1)
#define IWDG_PR_DIV4    (0)

#define IWDG_KR_RESET			0xAAAA
#define IWDG_KR_UNLOCK			0x5555
#define IWDG_KR_START			0xCCCC

typedef struct watchdogState_s {
    timeMs_t timeout;
    bool suspended;
} watchdogState_t;

static watchdogState_t state;

static bool watchdogPrescalerIsBusy(void)
{
    return IWDG->SR & IWDG_SR_PVU;
}

static bool watchdogReloadIsBusy(void)
{
    return IWDG->SR & IWDG_SR_RVU;
}

void watchdogSetTimeout(timeMs_t ms)
{
    if (ms <= 0) {
        ms = 500;
    }

    uint32_t pr;
    if (ms > 16384) {
        pr = IWDG_PR_DIV256;
    } else if (ms > 8192) {
        pr = IWDG_PR_DIV128;
    } else if (ms > 4096) {
        pr = IWDG_PR_DIV64;
    } else if (ms > 2048) {
        pr = IWDG_PR_DIV32;
    } else if (ms > 1024) {
        pr = IWDG_PR_DIV16;
    } else if (ms > 512) {
        pr = IWDG_PR_DIV8;
    } else {
        pr = IWDG_PR_DIV4;
    }

    uint32_t clk = LSI_FREQ / (0x04 << pr);
    uint32_t rlr = ms * clk / 1000UL - 1;

    while (watchdogPrescalerIsBusy());
    // enable write to PR, RLR
    IWDG->KR = IWDG_KR_UNLOCK;
    // Set prescaler
    IWDG->PR = pr;

    while (watchdogReloadIsBusy());
    // Set count
    IWDG->RLR = rlr & RLR_MASK;
    // Reset the watchdog
    IWDG->KR = IWDG_KR_RESET;
    state.timeout = ms;
}

void watchdogInit(void)
{
    // LSI enable, used by IWDG
    RCC->CSR |= RCC_CSR_LSION;
    // wait until LSI is ready
    while ((RCC->CSR & RCC_CSR_LSIRDY) == 0);

    // Start the watchdog
    IWDG->KR = IWDG_KR_START;
    // Set default timeout
    watchdogSetTimeout(0);
}

void watchdogRestart(void)
{
    IWDG->KR = IWDG_KR_RESET;
}

bool watchdogSuspend(void)
{
    if (state.timeout > 0 && !state.suspended) {
        // Since we can't suspend the watchdog, we set a big timeout
        watchdogSetTimeout(60 * 1000);
        state.suspended = true;
        return true;
    }
    return false;
}

bool watchdogResume(void)
{
    if (state.timeout > 0 && state.suspended) {
        watchdogRestart();
        watchdogSetTimeout(state.timeout);
        state.suspended = false;
        return true;
    }
    return false;
}


bool watchdogBarked(void)
{
    return (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) == SET);
}

#endif
