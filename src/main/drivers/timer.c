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
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#include "build/atomic.h"

#include "common/log.h"
#include "common/memory.h"
#include "common/utils.h"

#include "drivers/io.h"
#include "drivers/rcc.h"
#include "drivers/time.h"
#include "drivers/nvic.h"

#include "drivers/timer.h"
#include "drivers/timer_impl.h"

timHardwareContext_t * timerCtx[HARDWARE_TIMER_DEFINITION_COUNT];

// return index of timer in timer table. Lowest timer has index 0
uint8_t lookupTimerIndex(const TIM_TypeDef *tim)
{
    int i;

    // let gcc do the work, switch should be quite optimized
    for (i = 0; i < HARDWARE_TIMER_DEFINITION_COUNT; i++) {
        if (tim == timerDefinitions[i].tim) {
            return i;
        }
    }

    // make sure final index is out of range
    return ~1;
}

void timerConfigBase(TCH_t * tch, uint16_t period, uint32_t hz)
{
    if (tch == NULL) {
        return;
    }

    impl_timerConfigBase(tch, period, hz);
}

// old interface for PWM inputs. It should be replaced
void timerConfigure(TCH_t * tch, uint16_t period, uint32_t hz)
{
    if (tch == NULL) {
        return;
    }

    impl_timerConfigBase(tch, period, hz);
    impl_timerNVICConfigure(tch, NVIC_PRIO_TIMER);
    impl_enableTimer(tch);
}

TCH_t * timerGetTCH(const timerHardware_t * timHw)
{
    const int timerIndex = lookupTimerIndex(timHw->tim);
    
    if (timerIndex >= HARDWARE_TIMER_DEFINITION_COUNT) {
        LOG_E(TIMER, "Can't find hardware timer definition");
        return NULL;
    }

    // If timer context does not exist - allocate memory
    if (timerCtx[timerIndex] == NULL) {
        timerCtx[timerIndex] = memAllocate(sizeof(timHardwareContext_t), OWNER_TIMER);
        
        // Check for OOM
        if (timerCtx[timerIndex] == NULL) {
            LOG_E(TIMER, "Can't allocate TCH object");
            return NULL;
        }

        // Initialize parent object
        memset(timerCtx[timerIndex], 0, sizeof(timHardwareContext_t));
        timerCtx[timerIndex]->timDef = &timerDefinitions[timerIndex];
        timerCtx[timerIndex]->ch[0].timCtx = timerCtx[timerIndex];
        timerCtx[timerIndex]->ch[1].timCtx = timerCtx[timerIndex];
        timerCtx[timerIndex]->ch[2].timCtx = timerCtx[timerIndex];
        timerCtx[timerIndex]->ch[3].timCtx = timerCtx[timerIndex];

        // Implementation-specific init
        impl_timerInitContext(timerCtx[timerIndex]);
    }

    // Initialize timer channel object
    timerCtx[timerIndex]->ch[timHw->channelIndex].timHw = timHw;
    timerCtx[timerIndex]->ch[timHw->channelIndex].dma = NULL;
    timerCtx[timerIndex]->ch[timHw->channelIndex].cb = NULL;
    timerCtx[timerIndex]->ch[timHw->channelIndex].dmaState = TCH_DMA_IDLE;

    return &timerCtx[timerIndex]->ch[timHw->channelIndex];
}

// config edge and overflow callback for channel. Try to avoid overflowCallback, it is a bit expensive
void timerChInitCallbacks(timerCallbacks_t * cb, void * callbackParam, timerCallbackFn * edgeCallback, timerCallbackFn * overflowCallback)
{
    cb->callbackParam = callbackParam;
    cb->callbackEdge = edgeCallback;
    cb->callbackOvr = overflowCallback;
}

void timerChConfigCallbacks(TCH_t * tch, timerCallbacks_t * cb)
{
    if (tch == NULL) {
        return;
    }

    if (cb->callbackEdge == NULL) {
        impl_timerDisableIT(tch, TIM_IT_CCx(tch->timHw->channelIndex));
    }
    
    if (cb->callbackOvr == NULL) {
        impl_timerDisableIT(tch, IMPL_TIM_IT_UPDATE_INTERRUPT);
    }

    tch->cb = cb;

    if (cb->callbackEdge) {
        impl_timerEnableIT(tch, TIM_IT_CCx(tch->timHw->channelIndex));
    }

    if (cb->callbackOvr) {
        impl_timerEnableIT(tch, IMPL_TIM_IT_UPDATE_INTERRUPT);
    }
}

// Configure input captupre
void timerChConfigIC(TCH_t * tch, bool polarityRising, unsigned inputFilterSamples)
{
    impl_timerChConfigIC(tch, polarityRising, inputFilterSamples);
}

uint16_t timerGetPeriod(TCH_t * tch)
{
    return tch->timHw->tim->ARR;
}

void timerInit(void)
{
    memset(timerCtx, 0, sizeof (timerCtx));

    /* enable the timer peripherals */
    for (int i = 0; i < timerHardwareCount; i++) {
        unsigned timer = lookupTimerIndex(timerHardware[i].tim);
        RCC_ClockCmd(timerDefinitions[timer].rcc, ENABLE);
    }

    /* Before 2.0 timer outputs were initialized to IOCFG_AF_PP_PD even if not used */
    /* To keep compatibility make sure all timer output pins are mapped to INPUT with weak pull-down */
    for (int i = 0; i < timerHardwareCount; i++) {
        const timerHardware_t *timerHardwarePtr = &timerHardware[i];
        IOConfigGPIO(IOGetByTag(timerHardwarePtr->tag), IOCFG_IPD);
    }
}

const timerHardware_t * timerGetByTag(ioTag_t tag, timerUsageFlag_e flag)
{
    if (!tag) {
        return NULL;
    }

    for (int i = 0; i < timerHardwareCount; i++) {
        if (timerHardware[i].tag == tag) {
            if (timerHardware[i].usageFlags & flag || flag == 0) {
                return &timerHardware[i];
            }
        }
    }
    return NULL;
}

const timerHardware_t * timerGetByUsageFlag(timerUsageFlag_e flag)
{
    for (int i = 0; i < timerHardwareCount; i++) {
        if (timerHardware[i].usageFlags & flag) {
            return &timerHardware[i];
        }
    }
    return NULL;
}

void timerPWMConfigChannel(TCH_t * tch, uint16_t value)
{
    impl_timerPWMConfigChannel(tch, value);
}

void timerEnable(TCH_t * tch)
{
    impl_enableTimer(tch);
}

void timerPWMStart(TCH_t * tch)
{
    impl_timerPWMStart(tch);
}

volatile timCCR_t *timerCCR(TCH_t * tch)
{
    return impl_timerCCR(tch);
}

void timerChCaptureEnable(TCH_t * tch)
{
    impl_timerChCaptureCompareEnable(tch, true);
}

void timerChCaptureDisable(TCH_t * tch)
{
    impl_timerChCaptureCompareEnable(tch, false);
}

uint32_t timerGetBaseClock(TCH_t * tch)
{
    return timerGetBaseClockHW(tch->timHw);
}

uint32_t timerGetBaseClockHW(const timerHardware_t * timHw)
{
    return timerClock(timHw->tim);
}

bool timerPWMConfigChannelDMA(TCH_t * tch, void * dmaBuffer, uint8_t dmaBufferElementSize, uint32_t dmaBufferElementCount)
{
    return impl_timerPWMConfigChannelDMA(tch, dmaBuffer, dmaBufferElementSize, dmaBufferElementCount);
}

void timerPWMPrepareDMA(TCH_t * tch, uint32_t dmaBufferElementCount)
{
    impl_timerPWMPrepareDMA(tch, dmaBufferElementCount);
}

void timerPWMStartDMA(TCH_t * tch)
{
    impl_timerPWMStartDMA(tch);
}

void timerPWMStopDMA(TCH_t * tch)
{
    impl_timerPWMStopDMA(tch);
}

bool timerPWMDMAInProgress(TCH_t * tch)
{
    return tch->dmaState != TCH_DMA_IDLE;
}
