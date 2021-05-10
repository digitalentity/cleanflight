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
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/resource.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_spi.h"

// FIXME: Hack until we rework SPI driver
#define BUS_SPI1    SPIDEV_1
#define BUS_SPI2    SPIDEV_2
#define BUS_SPI3    SPIDEV_3
#define BUS_SPI4    SPIDEV_4

#define BUS_I2C1            I2CDEV_1
#define BUS_I2C2            I2CDEV_2
#define BUS_I2C3            I2CDEV_3
#define BUS_I2C4            I2CDEV_4
#define BUS_I2C_EMULATED    I2CINVALID

#define BUS_SCRATCHPAD_MEMORY_SIZE      (20)

typedef enum {
    BUS_SPEED_INITIALIZATION = 0,
    BUS_SPEED_SLOW           = 1,
    BUS_SPEED_STANDARD       = 2,
    BUS_SPEED_FAST           = 3,
    BUS_SPEED_ULTRAFAST      = 4
} busSpeed_e;

typedef enum {
    BUSTYPE_ANY  = 0,
    BUSTYPE_NONE = 0,
    BUSTYPE_I2C  = 1,
    BUSTYPE_SPI  = 2,
    BUSTYPE_SDIO = 3,
} busType_e;

typedef enum {
    BUSINDEX_1  = 0,
    BUSINDEX_2  = 1,
    BUSINDEX_3  = 2,
    BUSINDEX_4  = 3
} busIndex_e;

/* Ultimately all hardware descriptors will go to target definition files.
 * Driver code will merely query for it's HW descriptor and initialize it */
typedef enum {
    DEVHW_NONE = 0,

    /* Dedicated ACC chips */
    DEVHW_BMA280,
    DEVHW_ADXL345,
    DEVHW_MMA8452,
    DEVHW_LSM303DLHC,

    /* Dedicated GYRO chips */
    DEVHW_L3GD20,
    DEVHW_L3G4200,

    /* Combined ACC/GYRO chips */
    DEVHW_MPU3050,
    DEVHW_MPU6000,
    DEVHW_MPU6050,
    DEVHW_MPU6500,
    DEVHW_BMI160,
    DEVHW_BMI088_GYRO,
    DEVHW_BMI088_ACC,
    DEVHW_ICM20689,

    /* Combined ACC/GYRO/MAG chips */
    DEVHW_MPU9250,

    /* Barometer chips */
    DEVHW_BMP085,
    DEVHW_BMP280,
    DEVHW_MS5611,
    DEVHW_MS5607,
    DEVHW_LPS25H,
    DEVHW_SPL06,
    DEVHW_BMP388,
    DEVHW_DPS310,

    /* Compass chips */
    DEVHW_HMC5883,
    DEVHW_AK8963,
    DEVHW_AK8975,
    DEVHW_IST8310_0,
    DEVHW_IST8310_1,
    DEVHW_IST8308,
    DEVHW_QMC5883,
    DEVHW_MAG3110,
    DEVHW_LIS3MDL,
    DEVHW_RM3100,

    /* Temp sensor chips */
    DEVHW_LM75_0,
    DEVHW_LM75_1,
    DEVHW_LM75_2,
    DEVHW_LM75_3,
    DEVHW_LM75_4,
    DEVHW_LM75_5,
    DEVHW_LM75_6,
    DEVHW_LM75_7,

    /* 1-wire interface chips */
    DEVHW_DS2482,

    /* OSD chips */
    DEVHW_MAX7456,

    /* Rangefinder modules */
    DEVHW_SRF10,
    DEVHW_HCSR04_I2C,   // DIY-style adapter
    DEVHW_VL53L0X,
    DEVHW_VL53L1X,
    DEVHW_US42,

    /* Other hardware */
    DEVHW_MS4525,       // Pitot meter
    DEVHW_PCA9685,      // PWM output device
    DEVHW_M25P16,       // SPI NOR flash
    DEVHW_UG2864,       // I2C OLED display
    DEVHW_SDCARD,       // Generic SD-Card
    DEVHW_IRLOCK,       // IR-Lock visual positioning hardware
    DEVHW_PCF8574,      // 8-bit I/O expander
    DEVHW_BNO055,       // BNO055 IMU
} devHardwareType_e;

typedef enum {
    DEVFLAGS_NONE                       = 0,
    DEVFLAGS_USE_RAW_REGISTERS          = (1 << 0),     // Don't manipulate MSB for R/W selection (SPI), allow using 0xFF register to raw i2c reads/writes

    // SPI-only
    DEVFLAGS_USE_MANUAL_DEVICE_SELECT   = (1 << 1),     // (SPI only) Don't automatically select/deselect device
    DEVFLAGS_SPI_MODE_0                 = (1 << 2),     // (SPI only) Use CPOL=0/CPHA=0 (if unset MODE3 is used - CPOL=1/CPHA=1)
} deviceFlags_e;

typedef struct busDeviceDescriptor_s {
    void *              devicePtr;
    busType_e           busType;
    devHardwareType_e   devHwType;
    uint8_t             flags;
    uint8_t             tag;
    uint8_t             param;      // Driver-specific parameter
    union {
#ifdef USE_SPI
        struct {
            SPIDevice   spiBus;
            ioTag_t     csnPin;
        } spi;
#endif
#ifdef USE_I2C
        struct {
            I2CDevice   i2cBus;
            uint8_t     address;
        } i2c;
#endif
    } busdev;
    ioTag_t irqPin;
} busDeviceDescriptor_t;

typedef struct busDevice_s {
    const busDeviceDescriptor_t * descriptorPtr;
    busType_e busType;              // Copy of busType to avoid additional pointer dereferencing
    uint32_t flags;                 // Copy of flags
    uint32_t param;                 // Copy of param
    union {
#ifdef USE_SPI
        struct {
            SPIDevice spiBus;       // SPI bus ID
            IO_t csnPin;            // IO for CS# pin
        } spi;
#endif
#ifdef USE_I2C
        struct {
            I2CDevice i2cBus;       // I2C bus ID
            uint8_t address;        // I2C bus device address
        } i2c;
#endif
    } busdev;
    IO_t irqPin;                    // Device IRQ pin. Bus system will only assign IO_t object to this var. Initialization is up to device driver
    uint32_t * scratchpad;          // Memory where device driver can store persistent data. Zeroed out when initializing the device
                                    // for the first time. Useful when once device is shared between several sensors
                                    // (like MPU/ICM acc-gyro sensors)
} busDevice_t;

#ifdef __APPLE__
extern const busDeviceDescriptor_t __busdev_registry_start[] __asm("section$start$__DATA$__busdev_registry");
extern const busDeviceDescriptor_t __busdev_registry_end[] __asm("section$end$__DATA$__busdev_registry");
#define BUSDEV_REGISTER_ATTRIBUTES __attribute__ ((section("__DATA,__busdev_registry"), used, aligned(4)))
#else
extern const busDeviceDescriptor_t __busdev_registry_start[];
extern const busDeviceDescriptor_t __busdev_registry_end[];
#define BUSDEV_REGISTER_ATTRIBUTES __attribute__ ((section(".busdev_registry"), used, aligned(4)))
#endif

#ifdef USE_SPI
#define BUSDEV_REGISTER_SPI_F(_name, _devHw, _spiBus, _csnPin, _irqPin, _tag, _flags, _param)   \
    extern const busDeviceDescriptor_t _name ## _registry;                                      \
    static busDevice_t _name ## _memory;                                                        \
    const busDeviceDescriptor_t _name ## _registry BUSDEV_REGISTER_ATTRIBUTES = {               \
        .devicePtr = (void *) & _name ## _memory,                                               \
        .busType = BUSTYPE_SPI,                                                                 \
        .devHwType = _devHw,                                                                    \
        .flags = _flags,                                                                        \
        .tag = _tag,                                                                            \
        .param = _param,                                                                        \
        .busdev.spi = {                                                                         \
            .spiBus = _spiBus,                                                                  \
            .csnPin = IO_TAG(_csnPin)                                                           \
        },                                                                                      \
        .irqPin = IO_TAG(_irqPin)                                                               \
    };                                                                                          \
    struct _dummy                                                                               \
    /**/

#define BUSDEV_REGISTER_SPI(_name, _devHw, _spiBus, _csnPin, _irqPin, _flags, _param)           \
    BUSDEV_REGISTER_SPI_F(_name, _devHw, _spiBus, _csnPin, _irqPin, 0, _flags, _param)

#define BUSDEV_REGISTER_SPI_TAG(_name, _devHw, _spiBus, _csnPin, _irqPin, _tag, _flags, _param) \
    BUSDEV_REGISTER_SPI_F(_name, _devHw, _spiBus, _csnPin, _irqPin, _tag, _flags, _param)
#else
#define BUSDEV_REGISTER_SPI(_name, _devHw, _spiBus, _csnPin, _irqPin, _flags, _param)               // NO-OP
#define BUSDEV_REGISTER_SPI_TAG(_name, _devHw, _spiBus, _csnPin, _irqPin, _tag, _flags, _param)     // NO-OP
#endif

#ifdef USE_I2C
#define BUSDEV_REGISTER_I2C_F(_name, _devHw, _i2cBus, _devAddr, _irqPin, _tag, _flags, _param)  \
    extern const busDeviceDescriptor_t _name ## _registry;                                      \
    static busDevice_t _name ## _memory;                                                        \
    const busDeviceDescriptor_t _name ## _registry BUSDEV_REGISTER_ATTRIBUTES = {               \
        .devicePtr = (void *) & _name ## _memory,                                               \
        .busType = BUSTYPE_I2C,                                                                 \
        .devHwType = _devHw,                                                                    \
        .flags = _flags,                                                                        \
        .tag = _tag,                                                                            \
        .param = _param,                                                                        \
        .busdev.i2c = {                                                                         \
            .i2cBus = _i2cBus,                                                                  \
            .address = _devAddr                                                                 \
        },                                                                                      \
        .irqPin = IO_TAG(_irqPin)                                                               \
    };                                                                                          \
    struct _dummy                                                                               \
    /**/

#define BUSDEV_REGISTER_I2C(_name, _devHw, _i2cBus, _devAddr, _irqPin, _flags, _param)          \
    BUSDEV_REGISTER_I2C_F(_name, _devHw, _i2cBus, _devAddr, _irqPin, 0, _flags, _param)

#define BUSDEV_REGISTER_I2C_TAG(_name, _devHw, _i2cBus, _devAddr, _irqPin, _tag, _flags, _param)\
    BUSDEV_REGISTER_I2C_F(_name, _devHw, _i2cBus, _devAddr, _irqPin, _tag, _flags, _param)
#else
#define BUSDEV_REGISTER_I2C(_name, _devHw, _i2cBus, _devAddr, _irqPin, _flags, _param)              // NO-OP
#define BUSDEV_REGISTER_I2C_TAG(_name, _devHw, _i2cBus, _devAddr, _irqPin, _tag, _flags, _param)    // NO-OP
#endif

// busTransfer and busTransferMultiple are supported only on full-duplex SPI bus
typedef struct busTransferDescriptor_s {
    uint8_t *       rxBuf;
    const uint8_t * txBuf;
    uint32_t        length;
} busTransferDescriptor_t;

/* Internal abstraction function */
bool i2cBusWriteBuffer(const busDevice_t * dev, uint8_t reg, const uint8_t * data, uint8_t length);
bool i2cBusWriteRegister(const busDevice_t * dev, uint8_t reg, uint8_t data);
bool i2cBusReadBuffer(const busDevice_t * dev, uint8_t reg, uint8_t * data, uint8_t length);
bool i2cBusReadRegister(const busDevice_t * dev, uint8_t reg, uint8_t * data);

bool spiBusInitHost(const busDevice_t * dev);
bool spiBusIsBusy(const busDevice_t * dev);
void spiBusSetSpeed(const busDevice_t * dev, busSpeed_e speed);
bool spiBusTransfer(const busDevice_t * dev, uint8_t * rxBuf, const uint8_t * txBuf, int length);
bool spiBusTransferMultiple(const busDevice_t * dev, busTransferDescriptor_t * dsc, int count);
bool spiBusWriteBuffer(const busDevice_t * dev, uint8_t reg, const uint8_t * data, uint8_t length);
bool spiBusWriteRegister(const busDevice_t * dev, uint8_t reg, uint8_t data);
bool spiBusReadBuffer(const busDevice_t * dev, uint8_t reg, uint8_t * data, uint8_t length);
bool spiBusReadRegister(const busDevice_t * dev, uint8_t reg, uint8_t * data);
void spiBusSelectDevice(const busDevice_t * dev);
void spiBusDeselectDevice(const busDevice_t * dev);

/* Pre-initialize all known device descriptors to make sure hardware state is consistent and known
 * Initialize bus hardware */
void busInit(void);

/* Finds a device in registry. First matching device is returned. Also performs the low-level initialization of the hardware (CS line for SPI) */
busDevice_t * busDeviceInit(busType_e bus, devHardwareType_e hw, uint8_t tag, resourceOwner_e owner);
busDevice_t * busDeviceOpen(busType_e bus, devHardwareType_e hw, uint8_t tag);
void busDeviceDeInit(busDevice_t * dev);

uint32_t busDeviceReadScratchpad(const busDevice_t * dev);
void busDeviceWriteScratchpad(busDevice_t * dev, uint32_t value);
void * busDeviceGetScratchpadMemory(const busDevice_t * dev);

void busSetSpeed(const busDevice_t * dev, busSpeed_e speed);

/* Select/Deselect device will allow code to do something during device transfer or do transfer in chunks over some time */
void busSelectDevice(const busDevice_t * dev);
void busDeselectDevice(const busDevice_t * dev);

bool busWriteBuf(const busDevice_t * busdev, uint8_t reg, const uint8_t * data, uint8_t length);
bool busReadBuf(const busDevice_t * busdev, uint8_t reg, uint8_t * data, uint8_t length);
bool busRead(const busDevice_t * busdev, uint8_t reg, uint8_t * data);
bool busWrite(const busDevice_t * busdev, uint8_t reg, uint8_t data);

bool busTransfer(const busDevice_t * dev, uint8_t * rxBuf, const uint8_t * txBuf, int length);
bool busTransferMultiple(const busDevice_t * dev, busTransferDescriptor_t * buffers, int count);

bool busIsBusy(const busDevice_t * dev);
