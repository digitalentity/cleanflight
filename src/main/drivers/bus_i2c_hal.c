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

#include <platform.h>

#include "drivers/io.h"
#include "drivers/time.h"

#include "drivers/bus_i2c.h"
#include "drivers/nvic.h"
#include "io_impl.h"
#include "rcc.h"

#if !defined(SOFT_I2C) && defined(USE_I2C)

#define CLOCKSPEED 800000    // i2c clockspeed 400kHz default (conform specs), 800kHz  and  1200kHz (Betaflight default)

static void i2cUnstick(IO_t scl, IO_t sda);

#if defined(USE_I2C_PULLUP)
#define IOCFG_I2C IO_CONFIG(GPIO_MODE_AF_OD, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_PULLUP)
#else
#define IOCFG_I2C IOCFG_AF_OD
#endif

#ifndef I2C1_SCL
#define I2C1_SCL PB6
#endif

#ifndef I2C1_SDA
#define I2C1_SDA PB7
#endif

#ifndef I2C2_SCL
#define I2C2_SCL PB10
#endif
#ifndef I2C2_SDA
#define I2C2_SDA PB11
#endif

#ifndef I2C3_SCL
#define I2C3_SCL PA8
#endif
#ifndef I2C3_SDA
#define I2C3_SDA PB4
#endif

#if defined(USE_I2C_DEVICE_4)
#ifndef I2C4_SCL
#define I2C4_SCL PD12
#endif
#ifndef I2C4_SDA
#define I2C4_SDA PD13
#endif
#endif

static i2cDevice_t i2cHardwareMap[I2CDEV_COUNT] = {

#if defined(STM32F7)
    { .dev = I2C1, .scl = IO_TAG(I2C1_SCL), .sda = IO_TAG(I2C1_SDA), .rcc = RCC_APB1(I2C1), .speed = I2C_SPEED_400KHZ, .ev_irq = I2C1_EV_IRQn, .er_irq = I2C1_ER_IRQn, .af = GPIO_AF4_I2C1 },
    { .dev = I2C2, .scl = IO_TAG(I2C2_SCL), .sda = IO_TAG(I2C2_SDA), .rcc = RCC_APB1(I2C2), .speed = I2C_SPEED_400KHZ, .ev_irq = I2C2_EV_IRQn, .er_irq = I2C2_ER_IRQn, .af = GPIO_AF4_I2C2 },
    { .dev = I2C3, .scl = IO_TAG(I2C3_SCL), .sda = IO_TAG(I2C3_SDA), .rcc = RCC_APB1(I2C3), .speed = I2C_SPEED_400KHZ, .ev_irq = I2C3_EV_IRQn, .er_irq = I2C3_ER_IRQn, .af = GPIO_AF4_I2C3 },
#if defined(USE_I2C_DEVICE_4)
    { .dev = I2C4, .scl = IO_TAG(I2C4_SCL), .sda = IO_TAG(I2C4_SDA), .rcc = RCC_APB1(I2C4), .speed = I2C_SPEED_400KHZ, .ev_irq = I2C4_EV_IRQn, .er_irq = I2C4_ER_IRQn, .af = GPIO_AF4_I2C4 }
#endif
#elif defined(STM32H7)
    { .dev = I2C1, .scl = IO_TAG(I2C1_SCL), .sda = IO_TAG(I2C1_SDA), .rcc = RCC_APB1L(I2C1), .speed = I2C_SPEED_400KHZ, .ev_irq = I2C1_EV_IRQn, .er_irq = I2C1_ER_IRQn, .af = GPIO_AF4_I2C1 },
    { .dev = I2C2, .scl = IO_TAG(I2C2_SCL), .sda = IO_TAG(I2C2_SDA), .rcc = RCC_APB1L(I2C2), .speed = I2C_SPEED_400KHZ, .ev_irq = I2C2_EV_IRQn, .er_irq = I2C2_ER_IRQn, .af = GPIO_AF4_I2C2 },
    { .dev = I2C3, .scl = IO_TAG(I2C3_SCL), .sda = IO_TAG(I2C3_SDA), .rcc = RCC_APB1L(I2C3), .speed = I2C_SPEED_400KHZ, .ev_irq = I2C3_EV_IRQn, .er_irq = I2C3_ER_IRQn, .af = GPIO_AF4_I2C3 },
#if defined(USE_I2C_DEVICE_4)
    { .dev = I2C4, .scl = IO_TAG(I2C4_SCL), .sda = IO_TAG(I2C4_SDA), .rcc = RCC_APB1L(I2C4), .speed = I2C_SPEED_400KHZ, .ev_irq = I2C4_EV_IRQn, .er_irq = I2C4_ER_IRQn, .af = GPIO_AF4_I2C4 }
#endif
#endif
};

static volatile uint16_t i2cErrorCount = 0;

// Note that I2C_TIMEOUT is in us, while the HAL
// functions expect the timeout to be in ticks.
// Since we're setting up the ticks a 1khz, each
// tick equals 1ms.
#define I2C_DEFAULT_TIMEOUT     (I2C_TIMEOUT / 1000)

typedef struct {
    bool initialised;
    I2C_HandleTypeDef handle;
} i2cState_t;

static i2cState_t i2cState[I2CDEV_COUNT];

void i2cSetSpeed(uint8_t speed)
{
    for (unsigned int i = 0; i < sizeof(i2cHardwareMap) / sizeof(i2cHardwareMap[0]); i++) {
        i2cHardwareMap[i].speed = speed;
    }
}

void I2C1_ER_IRQHandler(void)
{
    HAL_I2C_ER_IRQHandler(&i2cState[I2CDEV_1].handle);
}

void I2C1_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&i2cState[I2CDEV_1].handle);
}

void I2C2_ER_IRQHandler(void)
{
    HAL_I2C_ER_IRQHandler(&i2cState[I2CDEV_2].handle);
}

void I2C2_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&i2cState[I2CDEV_2].handle);
}

void I2C3_ER_IRQHandler(void)
{
    HAL_I2C_ER_IRQHandler(&i2cState[I2CDEV_3].handle);
}

void I2C3_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&i2cState[I2CDEV_3].handle);
}

#ifdef USE_I2C_DEVICE_4
void I2C4_ER_IRQHandler(void)
{
    HAL_I2C_ER_IRQHandler(&i2cState[I2CDEV_4].handle);
}

void I2C4_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&i2cState[I2CDEV_4].handle);
}
#endif


static bool i2cHandleHardwareFailure(I2CDevice device)
{
    i2cErrorCount++;
    i2cInit(device);
    return false;
}

bool i2cWriteBuffer(I2CDevice device, uint8_t addr_, uint8_t reg_, uint8_t len_, const uint8_t *data, bool allowRawAccess)
{
    if (device == I2CINVALID)
        return false;

    i2cState_t * state = &(i2cState[device]);

    if (!state->initialised)
        return false;

    HAL_StatusTypeDef status;

    if ((reg_ == 0xFF || len_ == 0) && allowRawAccess) {
        status = HAL_I2C_Master_Transmit(&state->handle, addr_ << 1, (uint8_t *)data, len_, I2C_DEFAULT_TIMEOUT);
    }
    else {
        status = HAL_I2C_Mem_Write(&state->handle, addr_ << 1, reg_, I2C_MEMADD_SIZE_8BIT, (uint8_t *)data, len_, I2C_DEFAULT_TIMEOUT);
    }

    if (status != HAL_OK)
        return i2cHandleHardwareFailure(device);

    return true;
}

bool i2cWrite(I2CDevice device, uint8_t addr_, uint8_t reg_, uint8_t data, bool allowRawAccess)
{
    return i2cWriteBuffer(device, addr_, reg_, 1, &data, allowRawAccess);
}

bool i2cRead(I2CDevice device, uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t* buf, bool allowRawAccess)
{
    if (device == I2CINVALID)
        return false;

    i2cState_t * state = &(i2cState[device]);

    if (!state->initialised)
        return false;

    HAL_StatusTypeDef status;

    if (reg_ == 0xFF && allowRawAccess) {
        status = HAL_I2C_Master_Receive(&state->handle, addr_ << 1,buf, len, I2C_DEFAULT_TIMEOUT);
    }
    else {
        status = HAL_I2C_Mem_Read(&state->handle, addr_ << 1, reg_, I2C_MEMADD_SIZE_8BIT,buf, len, I2C_DEFAULT_TIMEOUT);
    }

    if (status != HAL_OK)
        return i2cHandleHardwareFailure(device);

    return true;
}

/*
 * Compute SCLDEL, SDADEL, SCLH and SCLL for TIMINGR register according to reference manuals.
 */
static void i2cClockComputeRaw(uint32_t pclkFreq, int i2cFreqKhz, int presc, int dfcoeff,
                       uint8_t *scldel, uint8_t *sdadel, uint16_t *sclh, uint16_t *scll)
{
    // Values from I2C-SMBus specification
    uint16_t trmax;      // Raise time (max)
    uint16_t tfmax;      // Fall time (max)
    uint8_t  tsuDATmin;  // SDA setup time (min)
    uint8_t  thdDATmin;  // SDA hold time (min)

    // Silicon specific values, from datasheet
    uint8_t  tAFmin;     // Analog filter delay (min)
    //uint8_t  tAFmax;     // Analog filter delay (max)

    // Actual (estimated) values
    uint16_t tr = 100;   // Raise time
    uint16_t tf = 100;   // Fall time
    uint8_t  tAF = 70;   // Analog filter delay

    if (i2cFreqKhz > 400) {
        // Fm+ (Fast mode plus)
        trmax = 120;
        tfmax = 120;
        tsuDATmin = 50;
        thdDATmin = 0;
    } else {
        // Fm (Fast mode)
        trmax = 300;
        tfmax = 300;
        tsuDATmin = 100;
        thdDATmin = 0;
    }

    tAFmin = 50;
    //tAFmax = 90;  // Unused

    // Convert pclkFreq into nsec
    float tI2cclk = 1000000000.0f / pclkFreq;

    // Convert target i2cFreq into cycle time (nsec)
    float tSCL = 1000000.0f / i2cFreqKhz;

    uint32_t SCLDELmin = (trmax + tsuDATmin)/((presc + 1) * tI2cclk) - 1;

    uint32_t SDADELmin = (tfmax + thdDATmin - tAFmin - ((dfcoeff + 3) * tI2cclk)) / ((presc + 1) * tI2cclk);

    float tsync1 = tf + tAF + dfcoeff * tI2cclk + 3 * tI2cclk;
    float tsync2 = tr + tAF + dfcoeff * tI2cclk + 3 * tI2cclk;

    float tSCLHL = tSCL - tsync1 - tsync2;
    float SCLHL = tSCLHL / ((presc + 1) * tI2cclk) - 1;

    uint32_t SCLH = SCLHL / 4.75;  // STM32CubeMX seems to use a value like this
    uint32_t SCLL = (uint32_t)(SCLHL + 0.5f) - SCLH;

    *scldel = SCLDELmin;
    *sdadel = SDADELmin;
    *sclh = SCLH - 1;
    *scll = SCLL - 1;
}

static uint32_t i2cClockTIMINGR(uint32_t pclkFreq, int i2cFreqKhz, int dfcoeff)
{
#define TIMINGR(presc, scldel, sdadel, sclh, scll) \
    ((presc << 28)|(scldel << 20)|(sdadel << 16)|(sclh << 8)|(scll << 0))

    uint8_t scldel;
    uint8_t sdadel;
    uint16_t sclh;
    uint16_t scll;

    for (int presc = 1; presc < 15; presc++) {
        i2cClockComputeRaw(pclkFreq, i2cFreqKhz, presc, dfcoeff, &scldel, &sdadel, &sclh, &scll);

        // If all fields are not overflowing, return TIMINGR.
        // Otherwise, increase prescaler and try again.
        if ((scldel < 16) && (sdadel < 16) && (sclh < 256) && (scll < 256)) {
            return TIMINGR(presc, scldel, sdadel, sclh, scll);
        }
    }
    return 0; // Shouldn't reach here
}

void i2cInit(I2CDevice device)
{
    i2cDevice_t * hardware = &(i2cHardwareMap[device]);
    i2cState_t * state = &(i2cState[device]);
    I2C_HandleTypeDef * pHandle = &state->handle;


    if (hardware->dev == NULL)
        return;

/*
    if (state->initialised)
        return;
*/

    // Enable RCC
    RCC_ClockCmd(hardware->rcc, ENABLE);

    IO_t scl = IOGetByTag(hardware->scl);
    IO_t sda = IOGetByTag(hardware->sda);

    IOInit(scl, OWNER_I2C, RESOURCE_I2C_SCL, RESOURCE_INDEX(device));
    IOInit(sda, OWNER_I2C, RESOURCE_I2C_SDA, RESOURCE_INDEX(device));

    i2cUnstick(scl, sda);

    // Init pins
    IOConfigGPIOAF(scl, IOCFG_I2C, hardware->af);
    IOConfigGPIOAF(sda, IOCFG_I2C, hardware->af);

    // Init I2C peripheral
    pHandle->Instance = hardware->dev;

/*
    switch (i2c->speed) {
        case I2C_SPEED_400KHZ:
        default:
            pHandle->Init.Timing = 0x00A01B5B;  // 400kHz, Rise 100ns, Fall 10ns    0x00500B6A
            break;

        case I2C_SPEED_800KHZ:
            pHandle->Init.Timing = 0x00401B1B;  // 800khz, Rise 40, Fall 4
            break;

        case I2C_SPEED_100KHZ:
            pHandle->Init.Timing = 0x60100544;  // 100kHz, Rise 100ns, Fall 10ns
            break;

        case I2C_SPEED_200KHZ:
            pHandle->Init.Timing = 0x00A01EDF;  // 200kHz, Rise 100ns, Fall 10ns
            break;
    }
*/

    // Compute TIMINGR value based on peripheral clock for this device instance

    uint32_t i2cPclk;

#if defined(STM32F7) || defined(STM32G4)
    // F7 Clock source configured in startup/system_stm32f7xx.c as:
    //   I2C1234 : PCLK1
    // G4 Clock source configured in startup/system_stm32g4xx.c as:
    //   I2C1234 : PCLK1
    i2cPclk = HAL_RCC_GetPCLK1Freq();
#elif defined(STM32H7)
    // Clock sources configured in startup/system_stm32h7xx.c as:
    //   I2C123 : D2PCLK1 (rcc_pclk1 for APB1)
    //   I2C4   : D3PCLK1 (rcc_pclk4 for APB4)
    i2cPclk = (hardware->dev == I2C4) ? HAL_RCCEx_GetD3PCLK1Freq() : HAL_RCC_GetPCLK1Freq();
#else
    #error Unknown MCU type
#endif

    switch (hardware->speed) {
        case I2C_SPEED_400KHZ:
        default:
            pHandle->Init.Timing = i2cClockTIMINGR(i2cPclk, 400, 0);    // 400kHz, Rise 100ns, Fall 10ns    0x00500B6A
            break;

        case I2C_SPEED_800KHZ:
            pHandle->Init.Timing = i2cClockTIMINGR(i2cPclk, 800, 0);    // 800khz, Rise 40, Fall 4
            break;

        case I2C_SPEED_100KHZ:
            pHandle->Init.Timing = i2cClockTIMINGR(i2cPclk, 100, 0);    // 100kHz, Rise 100ns, Fall 10ns
            break;

        case I2C_SPEED_200KHZ:
            pHandle->Init.Timing = i2cClockTIMINGR(i2cPclk, 200, 0);    // 200kHz, Rise 100ns, Fall 10ns
            break;
    }

    pHandle->Init.OwnAddress1     = 0x0;
    pHandle->Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    pHandle->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    pHandle->Init.OwnAddress2     = 0x0;
    pHandle->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    pHandle->Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;


    HAL_I2C_Init(pHandle);
    /* Enable the Analog I2C Filter */
    HAL_I2CEx_ConfigAnalogFilter(pHandle, I2C_ANALOGFILTER_ENABLE);

    HAL_NVIC_SetPriority(hardware->er_irq, NVIC_PRIO_I2C_ER, 0);
    HAL_NVIC_EnableIRQ(hardware->er_irq);

    HAL_NVIC_SetPriority(hardware->ev_irq, NVIC_PRIO_I2C_EV, 0);
    HAL_NVIC_EnableIRQ(hardware->ev_irq);

    state->initialised = true;
}

uint16_t i2cGetErrorCounter(void)
{
    return i2cErrorCount;
}

static void i2cUnstick(IO_t scl, IO_t sda)
{
    int i;

    IOHi(scl);
    IOHi(sda);

    IOConfigGPIO(scl, IOCFG_OUT_OD);
    IOConfigGPIO(sda, IOCFG_OUT_OD);

    // Analog Devices AN-686
    // We need 9 clock pulses + STOP condition
    for (i = 0; i < 9; i++) {
        // Wait for any clock stretching to finish
        int timeout = 100;
        while (!IORead(scl) && timeout) {
            delayMicroseconds(5);
            timeout--;
        }

        // Pull low
        IOLo(scl); // Set bus low
        delayMicroseconds(5);
        IOHi(scl); // Set bus high
        delayMicroseconds(5);
    }

    // Generate a stop condition in case there was none
    IOLo(scl);
    delayMicroseconds(5);
    IOLo(sda);
    delayMicroseconds(5);

    IOHi(scl); // Set bus scl high
    delayMicroseconds(5);
    IOHi(sda); // Set bus sda high
}

#endif
