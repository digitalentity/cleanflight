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

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "drivers/io_types.h"
#include "drivers/dma.h"
#include "drivers/rcc_types.h"
#include "drivers/timer_def.h"

#define CC_CHANNELS_PER_TIMER       4   // TIM_Channel_1..4

typedef uint16_t captureCompare_t;        // 16 bit on both 103 and 303, just register access must be 32bit sometimes (use timCCR_t)

#if defined(STM32F3) || defined(STM32F4) || defined(STM32F7) || defined(STM32H7)
typedef uint32_t timCCR_t;
typedef uint32_t timCCER_t;
typedef uint32_t timSR_t;
typedef uint32_t timCNT_t;
#elif defined(UNIT_TEST)
typedef uint32_t timCCR_t;
typedef uint32_t timCCER_t;
typedef uint32_t timSR_t;
typedef uint32_t timCNT_t;
#else
#error "Unknown CPU defined"
#endif

#if defined(STM32F3)
#define HARDWARE_TIMER_DEFINITION_COUNT 17
#elif defined(STM32F4)
#define HARDWARE_TIMER_DEFINITION_COUNT 14
#elif defined(STM32F7)
#define HARDWARE_TIMER_DEFINITION_COUNT 14
#elif defined(STM32H7)
#define HARDWARE_TIMER_DEFINITION_COUNT 14
#else
#error "Unknown CPU defined"
#endif

typedef struct timerDef_s {
    TIM_TypeDef   * tim;
    rccPeriphTag_t  rcc;
    uint8_t         irq;
    uint8_t         secondIrq;
} timerDef_t;

typedef enum {
    TIM_USE_ANY             = 0,
    TIM_USE_PPM             = (1 << 0),
    TIM_USE_PWM             = (1 << 1),
    TIM_USE_MC_MOTOR        = (1 << 2),     // Multicopter motor output
    TIM_USE_MC_SERVO        = (1 << 3),     // Multicopter servo output (i.e. TRI)
    TIM_USE_MC_CHNFW        = (1 << 4),     // Deprecated and not used after removal of CHANNEL_FORWARDING feature
    TIM_USE_FW_MOTOR        = (1 << 5),
    TIM_USE_FW_SERVO        = (1 << 6),
    TIM_USE_LED             = (1 << 24),
    TIM_USE_BEEPER          = (1 << 25),
} timerUsageFlag_e;

// TCH hardware definition (listed in target.c)
typedef struct timerHardware_s {
    TIM_TypeDef *tim;
    ioTag_t tag;
    uint8_t channelIndex;
    uint8_t output;
    ioConfig_t ioMode;
    uint8_t alternateFunction;
    uint32_t usageFlags;
    dmaTag_t dmaTag;
} timerHardware_t;

enum {
    TIMER_OUTPUT_NONE = 0x00,
    TIMER_OUTPUT_INVERTED = 0x02,
    TIMER_OUTPUT_N_CHANNEL= 0x04
};

typedef enum {
    TCH_DMA_IDLE = 0,
    TCH_DMA_READY,
    TCH_DMA_ACTIVE,
} tchDmaState_e;

// Some forward declarations for types
struct TCH_s;
struct timHardwareContext_s;

// Timer generic callback
typedef void timerCallbackFn(struct TCH_s * tch, uint32_t value);

typedef struct timerCallbacks_s {
    void *            callbackParam;
    timerCallbackFn * callbackEdge;
    timerCallbackFn * callbackOvr;
} timerCallbacks_t;

// Run-time TCH (Timer CHannel) context
typedef struct TCH_s {
    struct timHardwareContext_s *   timCtx;         // Run-time initialized to parent timer
    const timerHardware_t *         timHw;          // Link to timerHardware_t definition (target-specific)
    const timerCallbacks_t *        cb;
    DMA_t                           dma;            // Timer channel DMA handle
    volatile tchDmaState_e          dmaState;
    void *                          dmaBuffer;
} TCH_t;

// Run-time timer context (dynamically allocated), includes 4x TCH
typedef struct timHardwareContext_s {
    const timerDef_t *  timDef;
#ifdef USE_HAL_DRIVER
    TIM_HandleTypeDef * timHandle;
#endif
    TCH_t               ch[CC_CHANNELS_PER_TIMER];
} timHardwareContext_t;

// Per MCU timer definitions
extern timHardwareContext_t * timerCtx[HARDWARE_TIMER_DEFINITION_COUNT];
extern const timerDef_t timerDefinitions[HARDWARE_TIMER_DEFINITION_COUNT];

// Per target timer output definitions
extern const timerHardware_t timerHardware[];
extern const int timerHardwareCount;

typedef enum {
    TYPE_FREE,
    TYPE_PWMINPUT,
    TYPE_PPMINPUT,
    TYPE_PWMOUTPUT_MOTOR,
    TYPE_PWMOUTPUT_FAST,
    TYPE_PWMOUTPUT_SERVO,
    TYPE_SOFTSERIAL_RX,
    TYPE_SOFTSERIAL_TX,
    TYPE_SOFTSERIAL_RXTX,        // bidirectional pin for softserial
    TYPE_SOFTSERIAL_AUXTIMER,    // timer channel is used for softserial. No IO function on pin
    TYPE_ADC,
    TYPE_SERIAL_RX,
    TYPE_SERIAL_TX,
    TYPE_SERIAL_RXTX,
    TYPE_TIMER
} channelType_t;

uint32_t timerClock(TIM_TypeDef *tim);
uint32_t timerGetBaseClockHW(const timerHardware_t * timHw);

const timerHardware_t * timerGetByUsageFlag(timerUsageFlag_e flag);
const timerHardware_t * timerGetByTag(ioTag_t tag, timerUsageFlag_e flag);
TCH_t * timerGetTCH(const timerHardware_t * timHw);

uint32_t timerGetBaseClock(TCH_t * tch);
void timerConfigure(TCH_t * tch, uint16_t period, uint32_t hz);  // This interface should be replaced.

void timerChInitCallbacks(timerCallbacks_t * cb, void * callbackParam, timerCallbackFn * edgeCallback, timerCallbackFn * overflowCallback);
void timerChConfigIC(TCH_t * tch, bool polarityRising, unsigned inputFilterSamples);
void timerChConfigCallbacks(TCH_t * tch, timerCallbacks_t * cb);
void timerChCaptureEnable(TCH_t * tch);
void timerChCaptureDisable(TCH_t * tch);

void timerInit(void);
void timerStart(void);

void timerConfigBase(TCH_t * tch, uint16_t period, uint32_t hz);  // TODO - just for migration
uint16_t timerGetPeriod(TCH_t * tch);

void timerEnable(TCH_t * tch);
void timerPWMConfigChannel(TCH_t * tch, uint16_t value);
void timerPWMStart(TCH_t * tch);

// dmaBufferElementSize is the size in bytes of each element in the memory
// buffer. 1, 2 or 4 are the only valid values.
// dmaBufferElementCount is the number of elements in the buffer
bool timerPWMConfigChannelDMA(TCH_t * tch, void * dmaBuffer, uint8_t dmaBufferElementSize, uint32_t dmaBufferElementCount);
void timerPWMPrepareDMA(TCH_t * tch, uint32_t dmaBufferElementCount);
void timerPWMStartDMA(TCH_t * tch);
void timerPWMStopDMA(TCH_t * tch);
bool timerPWMDMAInProgress(TCH_t * tch);

volatile timCCR_t *timerCCR(TCH_t * tch);

uint16_t timerGetPrescalerByDesiredMhz(TIM_TypeDef *tim, uint16_t mhz);
