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

#include "drivers/time.h"
#include "drivers/io.h"
#include "rcc.h"
#include "drivers/nvic.h"

#include "serial.h"
#include "serial_uart.h"
#include "serial_uart_impl.h"

#define UART_RX_BUFFER_SIZE UART1_RX_BUFFER_SIZE
#define UART_TX_BUFFER_SIZE UART1_TX_BUFFER_SIZE

typedef struct uartDevice_s {
    USART_TypeDef* dev;
    uartPort_t port;
    ioTag_t rx;
    ioTag_t tx;
    volatile uint8_t rxBuffer[UART_RX_BUFFER_SIZE];
    volatile uint8_t txBuffer[UART_TX_BUFFER_SIZE];
    rccPeriphTag_t rcc;
    uint8_t af_rx;
    uint8_t af_tx;
    uint8_t irq;
} uartDevice_t;

#define UART_PIN_AF_HELPER(uart, pin)  CONCAT4(UART_PIN_AF_UART, uart, _, pin)

#ifdef USE_UART1
#define UART_PIN_AF_UART1_PA9       GPIO_AF7_USART1
#define UART_PIN_AF_UART1_PA10      GPIO_AF7_USART1
#define UART_PIN_AF_UART1_PB6       GPIO_AF4_USART1
#define UART_PIN_AF_UART1_PB7       GPIO_AF4_USART1
#define UART_PIN_AF_UART1_PB14      GPIO_AF4_USART1
#define UART_PIN_AF_UART1_PB15      GPIO_AF4_USART1

static uartDevice_t uart1 =
{
    .dev = USART1,
    .rx = IO_TAG(UART1_RX_PIN),
    .tx = IO_TAG(UART1_TX_PIN),
    .af_rx = UART_PIN_AF_HELPER(1, UART1_RX_PIN),
    .af_tx = UART_PIN_AF_HELPER(1, UART1_TX_PIN),
    .rcc = RCC_APB2(USART1),
    .irq = USART1_IRQn,
};
#endif

#ifdef USE_UART2
static uartDevice_t uart2 =
{
    .dev = USART2,
    .rx = IO_TAG(UART2_RX_PIN),
    .tx = IO_TAG(UART2_TX_PIN),
    .af_rx = GPIO_AF7_USART2,
    .af_tx = GPIO_AF7_USART2,
    .rcc = RCC_APB1L(USART2),
    .irq = USART2_IRQn,
};
#endif

#ifdef USE_UART3
static uartDevice_t uart3 =
{
    .dev = USART3,
    .rx = IO_TAG(UART3_RX_PIN),
    .tx = IO_TAG(UART3_TX_PIN),
    .af_rx = GPIO_AF7_USART3,
    .af_tx = GPIO_AF7_USART3,
    .rcc = RCC_APB1L(USART3),
    .irq = USART3_IRQn,
};
#endif

#ifdef USE_UART4
#define UART_PIN_AF_UART4_PA0       GPIO_AF8_UART4
#define UART_PIN_AF_UART4_PA1       GPIO_AF8_UART4
#define UART_PIN_AF_UART4_PA11      GPIO_AF6_UART4
#define UART_PIN_AF_UART4_PA12      GPIO_AF6_UART4
#define UART_PIN_AF_UART4_PB8       GPIO_AF8_UART4
#define UART_PIN_AF_UART4_PB9       GPIO_AF8_UART4
#define UART_PIN_AF_UART4_PC10      GPIO_AF8_UART4
#define UART_PIN_AF_UART4_PC11      GPIO_AF8_UART4
#define UART_PIN_AF_UART4_PD0       GPIO_AF8_UART4
#define UART_PIN_AF_UART4_PD1       GPIO_AF8_UART4

static uartDevice_t uart4 =
{
    .dev = UART4,
    .rx = IO_TAG(UART4_RX_PIN),
    .tx = IO_TAG(UART4_TX_PIN),
    .af_rx = UART_PIN_AF_HELPER(4, UART4_RX_PIN),
    .af_tx = UART_PIN_AF_HELPER(4, UART4_TX_PIN),
    .rcc = RCC_APB1L(UART4),
    .irq = UART4_IRQn,
};
#endif

#ifdef USE_UART5
#define UART_PIN_AF_UART5_PB5       GPIO_AF14_UART5
#define UART_PIN_AF_UART5_PB6       GPIO_AF14_UART5
#define UART_PIN_AF_UART5_PB12      GPIO_AF14_UART5
#define UART_PIN_AF_UART5_PB13      GPIO_AF14_UART5
#define UART_PIN_AF_UART5_PD2       GPIO_AF8_UART5
#define UART_PIN_AF_UART5_PC12      GPIO_AF8_UART5

static uartDevice_t uart5 =
{
    .dev = UART5,
    .rx = IO_TAG(UART5_RX_PIN),
    .tx = IO_TAG(UART5_TX_PIN),
    .af_rx = UART_PIN_AF_HELPER(5, UART5_RX_PIN),
    .af_tx = UART_PIN_AF_HELPER(5, UART5_TX_PIN),
    .rcc = RCC_APB1L(UART5),
    .irq = UART5_IRQn,
};
#endif

#ifdef USE_UART6
static uartDevice_t uart6 =
{
    .dev = USART6,
    .rx = IO_TAG(UART6_RX_PIN),
    .tx = IO_TAG(UART6_TX_PIN),
    .af_rx = GPIO_AF7_USART6,
    .af_tx = GPIO_AF7_USART6,
    .rcc = RCC_APB2(USART6),
    .irq = USART6_IRQn,
};
#endif

#ifdef USE_UART7
#define UART_PIN_AF_UART7_PA8       GPIO_AF11_UART7
#define UART_PIN_AF_UART7_PA15      GPIO_AF11_UART7
#define UART_PIN_AF_UART7_PB3       GPIO_AF11_UART7
#define UART_PIN_AF_UART7_PB4       GPIO_AF11_UART7
#define UART_PIN_AF_UART7_PE7       GPIO_AF7_UART7
#define UART_PIN_AF_UART7_PE8       GPIO_AF7_UART7
#define UART_PIN_AF_UART7_PF6       GPIO_AF7_UART7
#define UART_PIN_AF_UART7_PF7       GPIO_AF7_UART7

static uartDevice_t uart7 =
{
    .dev = UART7,
    .rx = IO_TAG(UART7_RX_PIN),
    .tx = IO_TAG(UART7_TX_PIN),
    .af_rx = UART_PIN_AF_HELPER(7, UART7_RX_PIN),
    .af_tx = UART_PIN_AF_HELPER(7, UART7_TX_PIN),
    .rcc = RCC_APB1L(UART7),
    .irq = UART7_IRQn,
};
#endif

#ifdef USE_UART8
static uartDevice_t uart8 =
{
    .dev = UART8,
    .rx = IO_TAG(UART8_RX_PIN),
    .tx = IO_TAG(UART8_TX_PIN),
    .af_rx = GPIO_AF8_UART8,
    .af_tx = GPIO_AF8_UART8,
    .rcc = RCC_APB1L(UART8),
    .irq = UART8_IRQn,
};
#endif



static uartDevice_t* uartHardwareMap[] = {
#ifdef USE_UART1
    &uart1,
#else
    NULL,
#endif
#ifdef USE_UART2
    &uart2,
#else
    NULL,
#endif
#ifdef USE_UART3
    &uart3,
#else
    NULL,
#endif
#ifdef USE_UART4
    &uart4,
#else
    NULL,
#endif
#ifdef USE_UART5
    &uart5,
#else
    NULL,
#endif
#ifdef USE_UART6
    &uart6,
#else
    NULL,
#endif
#ifdef USE_UART7
    &uart7,
#else
    NULL,
#endif
#ifdef USE_UART8
    &uart8,
#else
    NULL,
#endif
};

void uartGetPortPins(UARTDevice_e device, serialPortPins_t * pins)
{
    uartDevice_t *uart = uartHardwareMap[device];

    if (uart) {
        pins->txPin = uart->tx;
        pins->rxPin = uart->rx;
    }
    else {
        pins->txPin = IO_TAG(NONE);
        pins->rxPin = IO_TAG(NONE);
    }
}

void uartIrqHandler(uartPort_t *s)
{
    UART_HandleTypeDef *huart = &s->Handle;
    /* UART in mode Receiver ---------------------------------------------------*/
    if ((__HAL_UART_GET_IT(huart, UART_IT_RXNE) != RESET)) {
        uint8_t rbyte = (uint8_t)(huart->Instance->RDR & (uint8_t) 0xff);

        if (s->port.rxCallback) {
            s->port.rxCallback(rbyte, s->port.rxCallbackData);
        } else {
            s->port.rxBuffer[s->port.rxBufferHead] = rbyte;
            s->port.rxBufferHead = (s->port.rxBufferHead + 1) % s->port.rxBufferSize;
        }
        CLEAR_BIT(huart->Instance->CR1, (USART_CR1_PEIE));

        /* Disable the UART Error Interrupt: (Frame error, noise error, overrun error) */
        CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);

        __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);
    }

    /* UART parity error interrupt occurred -------------------------------------*/
    if ((__HAL_UART_GET_IT(huart, UART_IT_PE) != RESET)) {
        __HAL_UART_CLEAR_IT(huart, UART_CLEAR_PEF);
    }

    /* UART frame error interrupt occurred --------------------------------------*/
    if ((__HAL_UART_GET_IT(huart, UART_IT_FE) != RESET)) {
        __HAL_UART_CLEAR_IT(huart, UART_CLEAR_FEF);
    }

    /* UART noise error interrupt occurred --------------------------------------*/
    if ((__HAL_UART_GET_IT(huart, UART_IT_NE) != RESET)) {
        __HAL_UART_CLEAR_IT(huart, UART_CLEAR_NEF);
    }

    /* UART Over-Run interrupt occurred -----------------------------------------*/
    if ((__HAL_UART_GET_IT(huart, UART_IT_ORE) != RESET)) {
        __HAL_UART_CLEAR_IT(huart, UART_CLEAR_OREF);
    }

    /* UART in mode Transmitter ------------------------------------------------*/
    if (__HAL_UART_GET_IT(huart, UART_IT_TXE) != RESET) {
        /* Check that a Tx process is ongoing */
        if (huart->gState != HAL_UART_STATE_BUSY_TX) {
            if (s->port.txBufferTail == s->port.txBufferHead) {
                huart->TxXferCount = 0;
                /* Disable the UART Transmit Data Register Empty Interrupt */
                CLEAR_BIT(huart->Instance->CR1, USART_CR1_TXEIE);
            } else {
                if ((huart->Init.WordLength == UART_WORDLENGTH_9B) && (huart->Init.Parity == UART_PARITY_NONE)) {
                    huart->Instance->TDR = (((uint16_t) s->port.txBuffer[s->port.txBufferTail]) & (uint16_t) 0x01FFU);
                } else {
                    huart->Instance->TDR = (uint8_t)(s->port.txBuffer[s->port.txBufferTail]);
                }
                s->port.txBufferTail = (s->port.txBufferTail + 1) % s->port.txBufferSize;
            }
        }
    }

    /* UART in mode Transmitter (transmission end) -----------------------------*/
    if ((__HAL_UART_GET_IT(huart, UART_IT_TC) != RESET)) {
        HAL_UART_IRQHandler(huart);
    }
}

uartPort_t *serialUART(UARTDevice_e device, uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    uartPort_t *s;

    uartDevice_t *uart = uartHardwareMap[device];
    if (!uart) return NULL;

    s = &(uart->port);
    s->port.vTable = uartVTable;

    s->port.baudRate = baudRate;

    s->port.rxBuffer = uart->rxBuffer;
    s->port.txBuffer = uart->txBuffer;
    s->port.rxBufferSize = sizeof(uart->rxBuffer);
    s->port.txBufferSize = sizeof(uart->txBuffer);

    s->USARTx = uart->dev;

    s->Handle.Instance = uart->dev;

    if (uart->rcc) {
        RCC_ClockCmd(uart->rcc, ENABLE);
    }

    IO_t tx = IOGetByTag(uart->tx);
    IO_t rx = IOGetByTag(uart->rx);

    if (options & SERIAL_BIDIR) {
        IOInit(tx, OWNER_SERIAL, RESOURCE_UART_TXRX, RESOURCE_INDEX(device));
        IOConfigGPIOAF(tx, IOCFG_AF_PP, uart->af_tx);
    }
    else {
        if (mode & MODE_TX) {
            IOInit(tx, OWNER_SERIAL, RESOURCE_UART_TX, RESOURCE_INDEX(device));
            IOConfigGPIOAF(tx, IOCFG_AF_PP, uart->af_tx);
        }

        if (mode & MODE_RX) {
            IOInit(rx, OWNER_SERIAL, RESOURCE_UART_RX, RESOURCE_INDEX(device));
            IOConfigGPIOAF(rx, IOCFG_AF_PP, uart->af_rx);
        }
    }

    HAL_NVIC_SetPriority(uart->irq, NVIC_PRIO_SERIALUART, 0);
    HAL_NVIC_EnableIRQ(uart->irq);

    return s;
}

#ifdef USE_UART1
uartPort_t *serialUART1(uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    return serialUART(UARTDEV_1, baudRate, mode, options);
}

// USART1 Rx/Tx IRQ Handler
void USART1_IRQHandler(void)
{
    uartPort_t *s = &(uartHardwareMap[UARTDEV_1]->port);
    uartIrqHandler(s);
}
#endif

#ifdef USE_UART2
uartPort_t *serialUART2(uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    return serialUART(UARTDEV_2, baudRate, mode, options);
}

// USART2 Rx/Tx IRQ Handler
void USART2_IRQHandler(void)
{
    uartPort_t *s = &(uartHardwareMap[UARTDEV_2]->port);
    uartIrqHandler(s);
}
#endif

#ifdef USE_UART3
uartPort_t *serialUART3(uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    return serialUART(UARTDEV_3, baudRate, mode, options);
}

// USART3 Rx/Tx IRQ Handler
void USART3_IRQHandler(void)
{
    uartPort_t *s = &(uartHardwareMap[UARTDEV_3]->port);
    uartIrqHandler(s);
}
#endif

#ifdef USE_UART4
uartPort_t *serialUART4(uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    return serialUART(UARTDEV_4, baudRate, mode, options);
}

// UART4 Rx/Tx IRQ Handler
void UART4_IRQHandler(void)
{
    uartPort_t *s = &(uartHardwareMap[UARTDEV_4]->port);
    uartIrqHandler(s);
}
#endif

#ifdef USE_UART5
uartPort_t *serialUART5(uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    return serialUART(UARTDEV_5, baudRate, mode, options);
}

// UART5 Rx/Tx IRQ Handler
void UART5_IRQHandler(void)
{
    uartPort_t *s = &(uartHardwareMap[UARTDEV_5]->port);
    uartIrqHandler(s);
}
#endif

#ifdef USE_UART6
uartPort_t *serialUART6(uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    return serialUART(UARTDEV_6, baudRate, mode, options);
}

// USART6 Rx/Tx IRQ Handler
void USART6_IRQHandler(void)
{
    uartPort_t *s = &(uartHardwareMap[UARTDEV_6]->port);
    uartIrqHandler(s);
}
#endif

#ifdef USE_UART7
uartPort_t *serialUART7(uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    return serialUART(UARTDEV_7, baudRate, mode, options);
}

// UART7 Rx/Tx IRQ Handler
void UART7_IRQHandler(void)
{
    uartPort_t *s = &(uartHardwareMap[UARTDEV_7]->port);
    uartIrqHandler(s);
}
#endif

#ifdef USE_UART8
uartPort_t *serialUART8(uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    return serialUART(UARTDEV_8, baudRate, mode, options);
}

// UART8 Rx/Tx IRQ Handler
void UART8_IRQHandler(void)
{
    uartPort_t *s = &(uartHardwareMap[UARTDEV_8]->port);
    uartIrqHandler(s);
}
#endif