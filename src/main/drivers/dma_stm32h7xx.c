/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <platform.h>

#include "common/utils.h"
#include "drivers/nvic.h"
#include "drivers/dma.h"
#include "drivers/rcc.h"

/*
 * DMA descriptors.
 */
static dmaChannelDescriptor_t dmaDescriptors[] = {
    [0]  = DEFINE_DMA_CHANNEL(1, 0,  0), // DMA1_ST0
    [1]  = DEFINE_DMA_CHANNEL(1, 1,  6), // DMA1_ST1
    [2]  = DEFINE_DMA_CHANNEL(1, 2, 16), // DMA1_ST2
    [3]  = DEFINE_DMA_CHANNEL(1, 3, 22), // DMA1_ST3
    [4]  = DEFINE_DMA_CHANNEL(1, 4, 32), // DMA1_ST4
    [5]  = DEFINE_DMA_CHANNEL(1, 5, 38), // DMA1_ST5
    [6]  = DEFINE_DMA_CHANNEL(1, 6, 48), // DMA1_ST6
    [7]  = DEFINE_DMA_CHANNEL(1, 7, 54), // DMA1_ST7

    [8]  = DEFINE_DMA_CHANNEL(2, 0,  0), // DMA2_ST0
    [9]  = DEFINE_DMA_CHANNEL(2, 1,  6), // DMA2_ST1
    [10] = DEFINE_DMA_CHANNEL(2, 2, 16), // DMA2_ST2
    [11] = DEFINE_DMA_CHANNEL(2, 3, 22), // DMA2_ST3
    [12] = DEFINE_DMA_CHANNEL(2, 4, 32), // DMA2_ST4
    [13] = DEFINE_DMA_CHANNEL(2, 5, 38), // DMA2_ST5
    [14] = DEFINE_DMA_CHANNEL(2, 6, 48), // DMA2_ST6
    [15] = DEFINE_DMA_CHANNEL(2, 7, 54), // DMA2_ST7
};

/*
 * DMA IRQ Handlers
 */
DEFINE_DMA_IRQ_HANDLER(1, 0, 0) // DMA1_ST0
DEFINE_DMA_IRQ_HANDLER(1, 1, 1) // DMA1_ST1
DEFINE_DMA_IRQ_HANDLER(1, 2, 2) // DMA1_ST2
DEFINE_DMA_IRQ_HANDLER(1, 3, 3) // DMA1_ST3
DEFINE_DMA_IRQ_HANDLER(1, 4, 4) // DMA1_ST4
DEFINE_DMA_IRQ_HANDLER(1, 5, 5) // DMA1_ST5
DEFINE_DMA_IRQ_HANDLER(1, 6, 6) // DMA1_ST6
DEFINE_DMA_IRQ_HANDLER(1, 7, 7) // DMA1_ST7
DEFINE_DMA_IRQ_HANDLER(2, 0, 8) // DMA2_ST0
DEFINE_DMA_IRQ_HANDLER(2, 1, 9) // DMA2_ST1
DEFINE_DMA_IRQ_HANDLER(2, 2, 10) // DMA2_ST2
DEFINE_DMA_IRQ_HANDLER(2, 3, 11) // DMA2_ST3
DEFINE_DMA_IRQ_HANDLER(2, 4, 12) // DMA2_ST4
DEFINE_DMA_IRQ_HANDLER(2, 5, 13) // DMA2_ST5
DEFINE_DMA_IRQ_HANDLER(2, 6, 14) // DMA2_ST6
DEFINE_DMA_IRQ_HANDLER(2, 7, 15) // DMA2_ST7

DMA_t dmaGetByTag(dmaTag_t tag)
{
    for (unsigned i = 0; i < ARRAYLEN(dmaDescriptors); i++) {
        if (DMATAG_GET_DMA(dmaDescriptors[i].tag) == DMATAG_GET_DMA(tag) && DMATAG_GET_STREAM(dmaDescriptors[i].tag) == DMATAG_GET_STREAM(tag)) {
            return (DMA_t)&dmaDescriptors[i];
        }
    }

    return (DMA_t) NULL;
}

void dmaEnableClock(DMA_t dma)
{
    if (dma->dma == DMA1) {
        RCC_ClockCmd(RCC_AHB1(DMA1), ENABLE);
    }
    else {
        RCC_ClockCmd(RCC_AHB1(DMA2), ENABLE);
    }
}

resourceOwner_e dmaGetOwner(DMA_t dma)
{
    return dma->owner;
}

void dmaInit(DMA_t dma, resourceOwner_e owner, uint8_t resourceIndex)
{
    dmaEnableClock(dma);
    dma->owner = owner;
    dma->resourceIndex = resourceIndex;
}

void dmaSetHandler(DMA_t dma, dmaCallbackHandlerFuncPtr callback, uint32_t priority, uint32_t userParam)
{
    dmaEnableClock(dma);

    dma->irqHandlerCallback = callback;
    dma->userParam = userParam;

    HAL_NVIC_SetPriority(dma->irqNumber, priority, 0);
    HAL_NVIC_EnableIRQ(dma->irqNumber);
}

DMA_t dmaGetByRef(const DMA_Stream_TypeDef* ref)
{
    for (unsigned i = 0; i < (sizeof(dmaDescriptors) / sizeof(dmaDescriptors[0])); i++) {
        if (ref == dmaDescriptors[i].ref) {
            return &dmaDescriptors[i];
        }
    }

    return NULL;
}
