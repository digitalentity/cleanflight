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

/*
 * Cleanflight (or Baseflight): original
 * jflyper: Mono-timer and single-wire half-duplex
 * jflyper: Ported to iNav
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#if defined(USE_SOFTSERIAL1) || defined(USE_SOFTSERIAL2)

#include "build/build_config.h"
#include "build/atomic.h"

#include "build/debug.h"

#include "common/utils.h"

#include "drivers/nvic.h"
#include "drivers/io.h"
#include "drivers/timer.h"

#include "serial.h"
#include "serial_softserial.h"

#include "fc/config.h" //!!TODO remove this dependency

#define RX_TOTAL_BITS 10
#define TX_TOTAL_BITS 10

#if defined(USE_SOFTSERIAL1) && defined(USE_SOFTSERIAL2)
#define MAX_SOFTSERIAL_PORTS 2
#else
#define MAX_SOFTSERIAL_PORTS 1
#endif

typedef enum {
    TIMER_MODE_SINGLE,
    TIMER_MODE_DUAL,
} timerMode_e;

#define ICPOLARITY_RISING true
#define ICPOLARITY_FALLING false

typedef struct softSerial_s {
    serialPort_t     port;

    IO_t rxIO;
    IO_t txIO;

    TCH_t *           tch;
    TCH_t *           exTch;

    timerCallbacks_t  tchCallbacks;
    timerCallbacks_t  exTchCallbacks;

    volatile uint8_t rxBuffer[SOFTSERIAL_BUFFER_SIZE];
    volatile uint8_t txBuffer[SOFTSERIAL_BUFFER_SIZE];

    uint8_t          isSearchingForStartBit;
    uint8_t          rxBitIndex;
    uint8_t          rxLastLeadingEdgeAtBitIndex;
    uint8_t          rxEdge;
    uint8_t          rxActive;

    uint8_t          isTransmittingData;
    int8_t           bitsLeftToTransmit;

    uint16_t         internalTxBuffer;  // includes start and stop bits
    uint16_t         internalRxBuffer;  // includes start and stop bits

    uint16_t         transmissionErrors;
    uint16_t         receiveErrors;

    uint8_t          softSerialPortIndex;
    timerMode_e      timerMode;
} softSerial_t;

static const struct serialPortVTable softSerialVTable; // Forward

static softSerial_t softSerialPorts[MAX_SOFTSERIAL_PORTS];

void onSerialTimerOverflow(TCH_t * tch, uint32_t capture);
void onSerialRxPinChange(TCH_t * tch, uint32_t capture);

static void setTxSignal(softSerial_t *softSerial, uint8_t state)
{
    if (softSerial->port.options & SERIAL_INVERTED) {
        state = !state;
    }

    if (state) {
        IOHi(softSerial->txIO);
    } else {
        IOLo(softSerial->txIO);
    }
}

static void serialEnableCC(softSerial_t *softSerial)
{
    timerChCaptureEnable(softSerial->tch);
}

static void serialInputPortActivate(softSerial_t *softSerial)
{
    if (softSerial->port.options & SERIAL_INVERTED) {
        const uint8_t pinConfig = (softSerial->port.options & SERIAL_BIDIR_NOPULL) ? IOCFG_AF_PP : IOCFG_AF_PP_PD;
        IOConfigGPIOAF(softSerial->rxIO, pinConfig, softSerial->tch->timHw->alternateFunction);
    } else {
        const uint8_t pinConfig = (softSerial->port.options & SERIAL_BIDIR_NOPULL) ? IOCFG_AF_PP : IOCFG_AF_PP_UP;
        IOConfigGPIOAF(softSerial->rxIO, pinConfig, softSerial->tch->timHw->alternateFunction);
    }

    softSerial->rxActive = true;
    softSerial->isSearchingForStartBit = true;
    softSerial->rxBitIndex = 0;

    // Enable input capture
    serialEnableCC(softSerial);
}

static void serialInputPortDeActivate(softSerial_t *softSerial)
{
    // Disable input capture
    timerChCaptureDisable(softSerial->tch);
    IOConfigGPIOAF(softSerial->rxIO, IOCFG_IN_FLOATING, softSerial->tch->timHw->alternateFunction);
    softSerial->rxActive = false;
}

static void serialOutputPortActivate(softSerial_t *softSerial)
{
    if (softSerial->exTch)
        IOConfigGPIOAF(softSerial->txIO, IOCFG_OUT_PP, softSerial->exTch->timHw->alternateFunction);
    else
        IOConfigGPIO(softSerial->txIO, IOCFG_OUT_PP);
}

static void serialOutputPortDeActivate(softSerial_t *softSerial)
{
    if (softSerial->exTch)
        IOConfigGPIOAF(softSerial->txIO, IOCFG_IN_FLOATING, softSerial->exTch->timHw->alternateFunction);
    else
        IOConfigGPIO(softSerial->txIO, IOCFG_IN_FLOATING);
}

static bool isTimerPeriodTooLarge(uint32_t timerPeriod)
{
    return timerPeriod > 0xFFFF;
}

static void serialTimerConfigureTimebase(TCH_t * tch, uint32_t baud)
{
    uint32_t baseClock = timerClock(tch->timHw->tim);
    uint32_t clock = baseClock;
    uint32_t timerPeriod;

    do {
        timerPeriod = clock / baud;
        if (isTimerPeriodTooLarge(timerPeriod)) {
            if (clock > 1) {
                clock = clock / 2;   // this is wrong - mhz stays the same ... This will double baudrate until ok (but minimum baudrate is < 1200)
            } else {
                // TODO unable to continue, unable to determine clock and timerPeriods for the given baud
            }

        }
    } while (isTimerPeriodTooLarge(timerPeriod));

    timerConfigure(tch, timerPeriod, baseClock);
}

static void resetBuffers(softSerial_t *softSerial)
{
    softSerial->port.rxBufferSize = SOFTSERIAL_BUFFER_SIZE;
    softSerial->port.rxBuffer = softSerial->rxBuffer;
    softSerial->port.rxBufferTail = 0;
    softSerial->port.rxBufferHead = 0;

    softSerial->port.txBuffer = softSerial->txBuffer;
    softSerial->port.txBufferSize = SOFTSERIAL_BUFFER_SIZE;
    softSerial->port.txBufferTail = 0;
    softSerial->port.txBufferHead = 0;
}

serialPort_t *openSoftSerial(softSerialPortIndex_e portIndex, serialReceiveCallbackPtr rxCallback, void *rxCallbackData, uint32_t baud, portMode_t mode, portOptions_t options)
{
    softSerial_t *softSerial = &(softSerialPorts[portIndex]);
    ioTag_t tagRx;
    ioTag_t tagTx;

#ifdef USE_SOFTSERIAL1
    if (portIndex == SOFTSERIAL1) {
        tagRx = IO_TAG(SOFTSERIAL_1_RX_PIN);
        tagTx = IO_TAG(SOFTSERIAL_1_TX_PIN);
    }
#endif

#ifdef USE_SOFTSERIAL2
    if (portIndex == SOFTSERIAL2) {
        tagRx = IO_TAG(SOFTSERIAL_2_RX_PIN);
        tagTx = IO_TAG(SOFTSERIAL_2_TX_PIN);
    }
#endif

    const timerHardware_t *timerRx = timerGetByTag(tagRx, TIM_USE_ANY);
    const timerHardware_t *timerTx = timerGetByTag(tagTx, TIM_USE_ANY);

    IO_t rxIO = IOGetByTag(tagRx);
    IO_t txIO = IOGetByTag(tagTx);

	if (tagRx == tagTx) {
		if ((mode & MODE_RX) && (mode & MODE_TX)) {
			options |= SERIAL_BIDIR;
		}
	}
    
    if (options & SERIAL_BIDIR) {
        // If RX and TX pins are both assigned, we CAN use either with a timer.
        // However, for consistency with hardware UARTs, we only use TX pin,
        // and this pin must have a timer, and it should not be N-Channel.
        if (!timerTx || (timerTx->output & TIMER_OUTPUT_N_CHANNEL)) {
            return NULL;
        }

        softSerial->tch = timerGetTCH(timerTx);
        if (!softSerial->tch) {
            return NULL;
        }

        softSerial->txIO = txIO;
        softSerial->rxIO = txIO;
        IOInit(txIO, OWNER_SOFTSERIAL, RESOURCE_UART_TXRX, RESOURCE_INDEX(portIndex));
    } else {
        if (mode & MODE_RX) {
            // Need a pin & a timer on RX. Channel should not be N-Channel.
            if (!timerRx || (timerRx->output & TIMER_OUTPUT_N_CHANNEL)) {
                return NULL;
            }

            softSerial->tch = timerGetTCH(timerRx);
            if (!softSerial->tch) {
                return NULL;
            }

            softSerial->rxIO = rxIO;
            IOInit(rxIO, OWNER_SOFTSERIAL, RESOURCE_UART_RX, RESOURCE_INDEX(portIndex));
        }

        if (mode & MODE_TX) {
            // Need a pin on TX
            if (!tagTx) {
                return NULL;
            }

            softSerial->txIO = txIO;

            if (!(mode & MODE_RX)) {
                // TX Simplex, must have a timer
                if (!timerTx) {
                    return NULL;
                }

                softSerial->tch = timerGetTCH(timerTx);
                if (!softSerial->tch) {
                    return NULL;
                }
            } else {
                // Duplex
                softSerial->exTch = timerGetTCH(timerTx);
                if (!softSerial->tch) {
                    return NULL;
                }
            }
            IOInit(txIO, OWNER_SOFTSERIAL, RESOURCE_UART_TX, RESOURCE_INDEX(portIndex));
        }
    }

    softSerial->port.vTable = &softSerialVTable;
    softSerial->port.baudRate = baud;
    softSerial->port.mode = mode;
    softSerial->port.options = options;
    softSerial->port.rxCallback = rxCallback;
    softSerial->port.rxCallbackData = rxCallbackData;

    resetBuffers(softSerial);

    softSerial->softSerialPortIndex = portIndex;

    softSerial->transmissionErrors = 0;
    softSerial->receiveErrors = 0;

    softSerial->rxActive = false;
    softSerial->isTransmittingData = false;

    // Configure master timer (on RX); time base and input capture
    serialTimerConfigureTimebase(softSerial->tch, baud);
    timerChConfigIC(softSerial->tch, options & SERIAL_INVERTED, 0);

    // Configure bit clock interrupt & handler.
    // If we have an extra timer (on TX), it is initialized and configured
    // for overflow interrupt.
    // Receiver input capture is configured when input is activated.
    if ((mode & MODE_TX) && softSerial->exTch && softSerial->exTch->timHw->tim != softSerial->tch->timHw->tim) {
        softSerial->timerMode = TIMER_MODE_DUAL;
        serialTimerConfigureTimebase(softSerial->exTch, baud);

        timerChInitCallbacks(&softSerial->tchCallbacks, softSerial, onSerialRxPinChange, NULL);
        timerChInitCallbacks(&softSerial->exTchCallbacks, softSerial, NULL, onSerialTimerOverflow);

        timerChConfigCallbacks(softSerial->tch, &softSerial->tchCallbacks);
        timerChConfigCallbacks(softSerial->exTch, &softSerial->exTchCallbacks);
    } else {
        softSerial->timerMode = TIMER_MODE_SINGLE;
        timerChInitCallbacks(&softSerial->tchCallbacks, softSerial, onSerialRxPinChange, onSerialTimerOverflow);
        timerChConfigCallbacks(softSerial->tch, &softSerial->tchCallbacks);
    }

    if (!(options & SERIAL_BIDIR)) {
        serialOutputPortActivate(softSerial);
        setTxSignal(softSerial, ENABLE);
    }

    serialInputPortActivate(softSerial);

    return &softSerial->port;
}


/*
 * Serial Engine
 */

void processTxState(softSerial_t *softSerial)
{
    uint8_t mask;

    if (!softSerial->isTransmittingData) {
        if (isSoftSerialTransmitBufferEmpty((serialPort_t *)softSerial)) {
            // Transmit buffer empty.
            // Start listening if not already in if half-duplex
            if (!softSerial->rxActive && softSerial->port.options & SERIAL_BIDIR) {
                serialOutputPortDeActivate(softSerial);
                serialInputPortActivate(softSerial);
            }
            return;
        }

        // data to send
        uint8_t byteToSend = softSerial->port.txBuffer[softSerial->port.txBufferTail++];
        if (softSerial->port.txBufferTail >= softSerial->port.txBufferSize) {
            softSerial->port.txBufferTail = 0;
        }

        // build internal buffer, MSB = Stop Bit (1) + data bits (MSB to LSB) + start bit(0) LSB
        softSerial->internalTxBuffer = (1 << (TX_TOTAL_BITS - 1)) | (byteToSend << 1);
        softSerial->bitsLeftToTransmit = TX_TOTAL_BITS;
        softSerial->isTransmittingData = true;

        if (softSerial->rxActive && (softSerial->port.options & SERIAL_BIDIR)) {
            // Half-duplex: Deactivate receiver, activate transmitter
            serialInputPortDeActivate(softSerial);
            serialOutputPortActivate(softSerial);
        }

        return;
    }

    if (softSerial->bitsLeftToTransmit) {
        mask = softSerial->internalTxBuffer & 1;
        softSerial->internalTxBuffer >>= 1;

        setTxSignal(softSerial, mask);
        softSerial->bitsLeftToTransmit--;
        return;
    }

    softSerial->isTransmittingData = false;
}

enum {
    TRAILING,
    LEADING
};

void applyChangedBits(softSerial_t *softSerial)
{
    if (softSerial->rxEdge == TRAILING) {
        uint8_t bitToSet;
        for (bitToSet = softSerial->rxLastLeadingEdgeAtBitIndex; bitToSet < softSerial->rxBitIndex; bitToSet++) {
            softSerial->internalRxBuffer |= 1 << bitToSet;
        }
    }
}

void prepareForNextRxByte(softSerial_t *softSerial)
{
    // prepare for next byte
    softSerial->rxBitIndex = 0;
    softSerial->isSearchingForStartBit = true;
    if (softSerial->rxEdge == LEADING) {
        softSerial->rxEdge = TRAILING;
        timerChConfigIC(softSerial->tch, softSerial->port.options & SERIAL_INVERTED, 0);
        serialEnableCC(softSerial);
    }
}

#define STOP_BIT_MASK (1 << 0)
#define START_BIT_MASK (1 << (RX_TOTAL_BITS - 1))

void extractAndStoreRxByte(softSerial_t *softSerial)
{
    if ((softSerial->port.mode & MODE_RX) == 0) {
        return;
    }

    uint8_t haveStartBit = (softSerial->internalRxBuffer & START_BIT_MASK) == 0;
    uint8_t haveStopBit = (softSerial->internalRxBuffer & STOP_BIT_MASK) == 1;

    if (!haveStartBit || !haveStopBit) {
        softSerial->receiveErrors++;
        return;
    }

    uint8_t rxByte = (softSerial->internalRxBuffer >> 1) & 0xFF;

    if (softSerial->port.rxCallback) {
        softSerial->port.rxCallback(rxByte, softSerial->port.rxCallbackData);
    } else {
        softSerial->port.rxBuffer[softSerial->port.rxBufferHead] = rxByte;
        softSerial->port.rxBufferHead = (softSerial->port.rxBufferHead + 1) % softSerial->port.rxBufferSize;
    }
}

void processRxState(softSerial_t *softSerial)
{
    if (softSerial->isSearchingForStartBit) {
        return;
    }

    softSerial->rxBitIndex++;

    if (softSerial->rxBitIndex == RX_TOTAL_BITS - 1) {
        applyChangedBits(softSerial);
        return;
    }

    if (softSerial->rxBitIndex == RX_TOTAL_BITS) {

        if (softSerial->rxEdge == TRAILING) {
            softSerial->internalRxBuffer |= STOP_BIT_MASK;
        }

        extractAndStoreRxByte(softSerial);
        prepareForNextRxByte(softSerial);
    }
}

void onSerialTimerOverflow(TCH_t * tch, uint32_t capture)
{
    UNUSED(capture);

    softSerial_t * self = (softSerial_t *)tch->cb->callbackParam;

    if (self->port.mode & MODE_TX)
        processTxState(self);

    if (self->port.mode & MODE_RX)
        processRxState(self);
}

void onSerialRxPinChange(TCH_t * tch, uint32_t capture)
{
    UNUSED(capture);

    softSerial_t * self = (softSerial_t *)tch->cb->callbackParam;
    bool inverted = self->port.options & SERIAL_INVERTED;

    if ((self->port.mode & MODE_RX) == 0) {
        return;
    }

    if (self->isSearchingForStartBit) {
        // Synchronize the bit timing so that it will interrupt at the center
        // of the bit period.

#ifdef USE_HAL_DRIVER
        __HAL_TIM_SET_COUNTER(self->tch->timCtx->timHandle, __HAL_TIM_GET_AUTORELOAD(self->tch->timCtx->timHandle) / 2);
#else
        TIM_SetCounter(self->tch->timHw->tim, timerGetPeriod(self->tch) / 2);
#endif

        // For a mono-timer full duplex configuration, this may clobber the
        // transmission because the next callback to the onSerialTimerOverflow
        // will happen too early causing transmission errors.
        // For a dual-timer configuration, there is no problem.

        if ((self->timerMode != TIMER_MODE_DUAL) && self->isTransmittingData) {
            self->transmissionErrors++;
        }

        timerChConfigIC(self->tch, !inverted, 0);
#ifdef STM32F7
        serialEnableCC(self);
#endif
        self->rxEdge = LEADING;

        self->rxBitIndex = 0;
        self->rxLastLeadingEdgeAtBitIndex = 0;
        self->internalRxBuffer = 0;
        self->isSearchingForStartBit = false;
        return;
    }

    if (self->rxEdge == LEADING) {
        self->rxLastLeadingEdgeAtBitIndex = self->rxBitIndex;
    }

    applyChangedBits(self);

    if (self->rxEdge == TRAILING) {
        self->rxEdge = LEADING;
        timerChConfigIC(self->tch, !inverted, 0);
    } else {
        self->rxEdge = TRAILING;
        timerChConfigIC(self->tch, inverted, 0);
    }
#ifdef STM32F7
    serialEnableCC(self);
#endif
}


/*
 * Standard serial driver API
 */

uint32_t softSerialRxBytesWaiting(const serialPort_t *instance)
{
    if ((instance->mode & MODE_RX) == 0) {
        return 0;
    }

    softSerial_t *s = (softSerial_t *)instance;

    return (s->port.rxBufferHead - s->port.rxBufferTail) & (s->port.rxBufferSize - 1);
}

uint32_t softSerialTxBytesFree(const serialPort_t *instance)
{
    if ((instance->mode & MODE_TX) == 0) {
        return 0;
    }

    softSerial_t *s = (softSerial_t *)instance;

    uint8_t bytesUsed = (s->port.txBufferHead - s->port.txBufferTail) & (s->port.txBufferSize - 1);

    return (s->port.txBufferSize - 1) - bytesUsed;
}

uint8_t softSerialReadByte(serialPort_t *instance)
{
    uint8_t ch;

    if ((instance->mode & MODE_RX) == 0) {
        return 0;
    }

    if (softSerialRxBytesWaiting(instance) == 0) {
        return 0;
    }

    ch = instance->rxBuffer[instance->rxBufferTail];
    instance->rxBufferTail = (instance->rxBufferTail + 1) % instance->rxBufferSize;
    return ch;
}

void softSerialWriteByte(serialPort_t *s, uint8_t ch)
{
    if ((s->mode & MODE_TX) == 0) {
        return;
    }

    s->txBuffer[s->txBufferHead] = ch;
    s->txBufferHead = (s->txBufferHead + 1) % s->txBufferSize;
}

void softSerialSetBaudRate(serialPort_t *s, uint32_t baudRate)
{
    softSerial_t *softSerial = (softSerial_t *)s;

    softSerial->port.baudRate = baudRate;

    serialTimerConfigureTimebase(softSerial->tch, baudRate);
}

void softSerialSetMode(serialPort_t *instance, portMode_t mode)
{
    instance->mode = mode;
}

bool isSoftSerialTransmitBufferEmpty(const serialPort_t *instance)
{
    return instance->txBufferHead == instance->txBufferTail;
}

static const struct serialPortVTable softSerialVTable = {
    .serialWrite = softSerialWriteByte,
    .serialTotalRxWaiting = softSerialRxBytesWaiting,
    .serialTotalTxFree = softSerialTxBytesFree,
    .serialRead = softSerialReadByte,
    .serialSetBaudRate = softSerialSetBaudRate,
    .isSerialTransmitBufferEmpty = isSoftSerialTransmitBufferEmpty,
    .setMode = softSerialSetMode,
    .isConnected = NULL,
    .writeBuf = NULL,
    .beginWrite = NULL,
    .endWrite = NULL,
    .isIdle = NULL,
};

#endif
