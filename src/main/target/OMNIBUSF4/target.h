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

//Same target as OMNIBUSF4PRO with LED strip in M5
#ifdef OMNIBUSF4PRO_LEDSTRIPM5
#define OMNIBUSF4PRO
#endif
//Same target as OMNIBUSF4V3 with softserial in M5 and M6
#if defined(OMNIBUSF4V3_S6_SS) || defined(OMNIBUSF4V3_S5S6_SS) || defined(OMNIBUSF4V3_S5_S6_2SS)
#define OMNIBUSF4V3
#endif

#ifdef OMNIBUSF4PRO
#define TARGET_BOARD_IDENTIFIER "OBSD"
#elif defined(OMNIBUSF4V3)
#define TARGET_BOARD_IDENTIFIER "OB43"
#elif defined(DYSF4PRO)
#define TARGET_BOARD_IDENTIFIER "DYS4"
#elif defined(DYSF4PROV2)
#define TARGET_BOARD_IDENTIFIER "DY42"
#else
#define TARGET_BOARD_IDENTIFIER "OBF4"
#endif

#if defined(DYSF4PRO) || defined(DYSF4PROV2)
#define USBD_PRODUCT_STRING "DysF4Pro"
#else
#define USBD_PRODUCT_STRING "Omnibus F4"
#endif

#define LED0                    PB5

#define BEEPER                  PB4
#define BEEPER_INVERTED

#if defined(DYSF4PROV2)
#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C1_SCL PB8
#define I2C1_SDA PB9
#define I2C_EXT_BUS BUS_I2C1
#else
#define USE_I2C
#define USE_I2C_DEVICE_2
#define I2C_DEVICE_2_SHARES_UART3
#define I2C_EXT_BUS BUS_I2C2
#endif

#define UG2864_I2C_BUS I2C_EXT_BUS

// MPU6000 interrupts
#define USE_EXTI
#define GYRO_INT_EXTI            PC4
#define USE_MPU_DATA_READY_SIGNAL

#define MPU6000_CS_PIN          PA4
#define MPU6000_SPI_BUS         BUS_SPI1

#if defined(OMNIBUSF4PRO) || defined(OMNIBUSF4V3)
  #define USE_IMU_MPU6000
  #define IMU_MPU6000_ALIGN       CW270_DEG
#else
  #define USE_IMU_MPU6000
  #define IMU_MPU6000_ALIGN       CW180_DEG
#endif

// Support for OMNIBUS F4 PRO CORNER - it has ICM20608 instead of MPU6000
#if defined(OMNIBUSF4PRO) || defined(OMNIBUSF4V3)
  #define MPU6500_CS_PIN          MPU6000_CS_PIN
  #define MPU6500_SPI_BUS         MPU6000_SPI_BUS
  #define USE_IMU_MPU6500
  #define IMU_MPU6500_ALIGN       IMU_MPU6000_ALIGN
#endif

#define USE_MAG
#define MAG_I2C_BUS             I2C_EXT_BUS
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
#define USE_MAG_IST8310
#define USE_MAG_IST8308
#define USE_MAG_MAG3110
#define USE_MAG_LIS3MDL
#define USE_MAG_AK8975

#define TEMPERATURE_I2C_BUS     I2C_EXT_BUS
#define BNO055_I2C_BUS          I2C_EXT_BUS

#define USE_BARO

#if defined(OMNIBUSF4PRO) || defined(OMNIBUSF4V3)
  #define USE_BARO_BMP280
  #define BMP280_SPI_BUS        BUS_SPI3
  #define BMP280_CS_PIN         PB3 // v1
  // Support external barometers
  #define BARO_I2C_BUS          I2C_EXT_BUS
  #define USE_BARO_BMP085
  #define USE_BARO_MS5611
#else
  #define BARO_I2C_BUS          I2C_EXT_BUS
  #define USE_BARO_BMP085
  #define USE_BARO_BMP280
  #define USE_BARO_MS5611
#endif

#define PITOT_I2C_BUS           I2C_EXT_BUS

#define USE_RANGEFINDER
#define RANGEFINDER_I2C_BUS     I2C_EXT_BUS

#define USE_VCP
#define VBUS_SENSING_PIN        PC5
#define VBUS_SENSING_ENABLED

#if defined(OMNIBUSF4PRO) || defined(OMNIBUSF4V3)
#define USE_UART_INVERTER
#endif

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9
#define UART1_AHB1_PERIPHERALS  RCC_AHB1Periph_DMA2
#if defined(OMNIBUSF4PRO)
#define INVERTER_PIN_UART1_RX PC0 // PC0 has never been used as inverter control on genuine OMNIBUS F4 variants, but leave it as is since some clones actually implement it.
#endif

#define USE_UART3
#define UART3_RX_PIN            PB11
#define UART3_TX_PIN            PB10

#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6
#if defined(OMNIBUSF4V3)
  #define INVERTER_PIN_UART6_RX PC8
  #define INVERTER_PIN_UART6_TX PC9
#endif

#if defined(OMNIBUSF4V3) && !(defined(OMNIBUSF4V3_S6_SS) || defined(OMNIBUSF4V3_S5S6_SS) || defined(OMNIBUSF4V3_S5_S6_2SS))
#define USE_SOFTSERIAL1
#define SOFTSERIAL_1_RX_PIN     PC6     // shared with UART6 TX
#define SOFTSERIAL_1_TX_PIN     PC6     // shared with UART6 TX

#define SERIAL_PORT_COUNT       5       // VCP, USART1, USART3, USART6, SOFTSERIAL1

#elif defined(OMNIBUSF4V3_S6_SS)        // one softserial on S6
#define USE_SOFTSERIAL1
#define SOFTSERIAL_1_RX_PIN     PA8     // S6 output
#define SOFTSERIAL_1_TX_PIN     PA8     // S6 output

#define SERIAL_PORT_COUNT       5       // VCP, USART1, USART3, USART6, SOFTSERIAL1

#elif defined(OMNIBUSF4V3_S5S6_SS)      // one softserial on S5/RX S6/TX
#define USE_SOFTSERIAL1
#define SOFTSERIAL_1_RX_PIN     PA1     // S5 output
#define SOFTSERIAL_1_TX_PIN     PA8     // S6 output

#define SERIAL_PORT_COUNT       5       // VCP, USART1, USART3, USART6, SOFTSERIAL1

#elif defined(OMNIBUSF4V3_S5_S6_2SS)    // two softserials, one on S5 and one on S6
#define USE_SOFTSERIAL1
#define SOFTSERIAL_1_RX_PIN     PA1     // S5 output
#define SOFTSERIAL_1_TX_PIN     PA1     // S5 output

#define USE_SOFTSERIAL2
#define SOFTSERIAL_2_RX_PIN     PA8     // S6 output
#define SOFTSERIAL_2_TX_PIN     PA8     // S6 output

#define SERIAL_PORT_COUNT       6       // VCP, USART1, USART3, USART6, SOFTSERIAL1, SOFTSERIAL2

#else                                   // One softserial on versions other than OMNIBUSF4V3
#define USE_SOFTSERIAL1
#define SOFTSERIAL_1_RX_PIN     PC8     // pad labelled CH5 on OMNIBUSF4PRO
#define SOFTSERIAL_1_TX_PIN     PC9     // pad labelled CH6 on OMNIBUSF4PRO

#define SERIAL_PORT_COUNT       5       // VCP, USART1, USART3, USART6, SOFTSERIAL1
#endif

#define USE_SPI

#define USE_SPI_DEVICE_1

#if defined(OMNIBUSF4PRO) || defined(OMNIBUSF4V3)
  #define USE_SPI_DEVICE_2
  #define SPI2_NSS_PIN          PB12
  #define SPI2_SCK_PIN          PB13
  #define SPI2_MISO_PIN         PB14
  #define SPI2_MOSI_PIN         PB15
#endif

#define USE_SPI_DEVICE_3
#if defined(OMNIBUSF4PRO) || defined(OMNIBUSF4V3)
  #define SPI3_NSS_PIN          PA15
#else
  #define SPI3_NSS_PIN          PB3
#endif
#define SPI3_SCK_PIN            PC10
#define SPI3_MISO_PIN           PC11
#define SPI3_MOSI_PIN           PC12

#define USE_OSD
#define USE_MAX7456
#define MAX7456_SPI_BUS         BUS_SPI3
#define MAX7456_CS_PIN          PA15

#if defined(OMNIBUSF4PRO) || defined(OMNIBUSF4V3)
  #define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT
  #define USE_SDCARD
  #define USE_SDCARD_SPI

  #define SDCARD_SPI_BUS        BUS_SPI2
  #define SDCARD_CS_PIN         SPI2_NSS_PIN

  #define SDCARD_DETECT_PIN     PB7
  #define SDCARD_DETECT_INVERTED
#else
  #define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT
  #define M25P16_CS_PIN           SPI3_NSS_PIN
  #define M25P16_SPI_BUS          BUS_SPI3
  #define USE_FLASHFS
  #define USE_FLASH_M25P16
#endif

#define USE_ADC
#define ADC_CHANNEL_1_PIN               PC1
#define ADC_CHANNEL_2_PIN               PC2

#ifdef DYSF4PRO
    #define ADC_CHANNEL_3_PIN               PC3
#else
    #define ADC_CHANNEL_3_PIN               PA0
#endif

#define CURRENT_METER_ADC_CHANNEL       ADC_CHN_1
#define VBAT_ADC_CHANNEL                ADC_CHN_2
#define RSSI_ADC_CHANNEL                ADC_CHN_3

#define SENSORS_SET (SENSOR_ACC|SENSOR_MAG|SENSOR_BARO)

#define USE_LED_STRIP
#if (defined(OMNIBUSF4PRO) || defined(OMNIBUSF4V3)) && !defined(OMNIBUSF4PRO_LEDSTRIPM5)
  #define WS2811_PIN                   PB6
#else
  #define WS2811_PIN                   PA1
#endif

#define DEFAULT_RX_TYPE         RX_TYPE_PPM
#define DISABLE_RX_PWM_FEATURE
#define DEFAULT_FEATURES        (FEATURE_TX_PROF_SEL | FEATURE_BLACKBOX | FEATURE_VBAT | FEATURE_OSD)

#define USE_SPEKTRUM_BIND
#define BIND_PIN                PB11 // USART3 RX

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

// Number of available PWM outputs
#define MAX_PWM_OUTPUT_PORTS    6
#define TARGET_MOTOR_COUNT      6
#define USE_DSHOT
#define USE_ESC_SENSOR

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         0xffff

#ifdef OMNIBUSF4PRO
#define CURRENT_METER_SCALE   265
#endif

#define PCA9685_I2C_BUS         I2C_EXT_BUS
