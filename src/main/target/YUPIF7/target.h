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
#define TARGET_BOARD_IDENTIFIER "YPF7"
#define USBD_PRODUCT_STRING     "YUPIF7"

#define LED0                    PB4

//define camera control
#define CAMERA_CONTROL_PIN      PB7

#define BEEPER                  PB14
#define BEEPER_PWM
#define BEEPER_INVERTED
#define BEEPER_PWM_FREQUENCY    3150

// Gyro interrupt
#define USE_EXTI
#define GYRO_INT_EXTI           PC4
#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW

// ICM 20689
#define USE_IMU_ICM20689
#define IMU_ICM20689_ALIGN      CW90_DEG
#define ICM20689_CS_PIN         SPI1_NSS_PIN
#define ICM20689_SPI_BUS        BUS_SPI1

#define USE_MAG
#define MAG_I2C_BUS             BUS_I2C1
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
#define TEMPERATURE_I2C_BUS     BUS_I2C1
#define BNO055_I2C_BUS          BUS_I2C1

#define USE_BARO
#define BARO_I2C_BUS            BUS_I2C1
#define USE_BARO_MS5611
#define USE_BARO_BMP280

// Serial ports
#define USB_IO
#define USE_VCP
#define VBUS_SENSING_PIN        PA8
#define VBUS_SENSING_ENABLED

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9

#define USE_UART3
#define UART3_RX_PIN            PB11
#define UART3_TX_PIN            PB10

#define USE_UART5
#define UART5_RX_PIN            PD2
#define UART5_TX_PIN            PC12

#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6

#define USE_SOFTSERIAL1
#define SOFTSERIAL_1_RX_PIN     PB0
#define SOFTSERIAL_1_TX_PIN     PB1

#define SERIAL_PORT_COUNT       6 //VCP, USART1, USART3, USART5, USART6

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_HARDWARE 0

#define UG2864_I2C_BUS          BUS_I2C1
#define USE_DASHBOARD
#define USE_OLED_UG2864
#define DASHBOARD_ARMED_BITMAP

#define USE_PITOT
#define PITOT_I2C_BUS           BUS_I2C1

//SPI ports
#define USE_SPI

#define USE_SPI_DEVICE_1
#define SPI1_NSS_PIN            PA4
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define USE_SPI_DEVICE_3
#define SPI3_NSS_PIN            PA15
#define SPI3_SCK_PIN            PC10
#define SPI3_MISO_PIN           PC11
#define SPI3_MOSI_PIN           PB5

// I2C Port
#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C1_SCL                PB8
#define I2C1_SDA                PB9
#define I2C_DEVICE              (I2CDEV_1)

// OSD
#define USE_OSD
#define USE_MAX7456
#define MAX7456_SPI_BUS         BUS_SPI1
#define MAX7456_CS_PIN          PA14

// Dataflash
#define M25P16_CS_PIN           SPI3_NSS_PIN
#define M25P16_SPI_BUS          BUS_SPI3
#define USE_FLASHFS
#define USE_FLASH_M25P16
#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT

// ADC inputs
#define USE_ADC
#define ADC_CHANNEL_1_PIN       PC0
#define ADC_CHANNEL_2_PIN       PC1
#define ADC_CHANNEL_3_PIN       PC2
#define RSSI_ADC_CHANNEL                ADC_CHN_1
#define VBAT_ADC_CHANNEL                ADC_CHN_2
#define CURRENT_METER_ADC_CHANNEL       ADC_CHN_3
#define CURRENT_METER_SCALE             235

// LED Strip can run off Pin 5 (PB1) of the motor outputs
#define USE_LED_STRIP
#define WS2811_PIN                      PB1

// DSHOT
#define USE_DSHOT
#define USE_ESC_SENSOR

// Default configuration
#define DEFAULT_RX_TYPE         RX_TYPE_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define SERIALRX_UART           SERIAL_PORT_USART6
#define TELEMETRY_UART          SERIAL_PORT_USART1
#define DEFAULT_FEATURES        (FEATURE_TELEMETRY | FEATURE_OSD)

// Target IO and timers
#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define MAX_PWM_OUTPUT_PORTS    9
#define TARGET_MOTOR_COUNT      6

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))

