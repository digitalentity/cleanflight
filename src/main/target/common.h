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

#if defined(STM32F7) || defined(STM32H7)
#define USE_ITCM_RAM
#endif

#ifdef USE_ITCM_RAM
#define FAST_CODE                   __attribute__((section(".tcm_code")))
#define NOINLINE                    __attribute__((noinline))
#else
#define FAST_CODE
#define NOINLINE
#endif

#if defined(STM32F3)
#define DYNAMIC_HEAP_SIZE   1024
#else
#define DYNAMIC_HEAP_SIZE   2048
#endif

#define I2C1_OVERCLOCK false
#define I2C2_OVERCLOCK false
#define USE_I2C_PULLUP          // Enable built-in pullups on all boards in case external ones are too week

#define USE_RX_PPM
#define USE_SERIAL_RX
#define USE_SERIALRX_SPEKTRUM   // Cheap and fairly common protocol
#define USE_SERIALRX_SBUS       // Very common protocol
#define USE_SERIALRX_IBUS       // Cheap FlySky & Turnigy receivers
#define USE_SERIALRX_FPORT
#define USE_SERIALRX_FPORT2

#define COMMON_DEFAULT_FEATURES (FEATURE_TX_PROF_SEL)

#if defined(STM32F3)
#define USE_UNDERCLOCK
//save flash for F3 targets
#define CLI_MINIMAL_VERBOSITY
#define SKIP_CLI_COMMAND_HELP
#define SKIP_CLI_RESOURCES
#endif

#if defined(STM32F4) || defined(STM32F7)
#define USE_SERVO_SBUS
#endif

#define USE_ADC_AVERAGING
#define USE_64BIT_TIME
#define USE_BLACKBOX
#define USE_GPS
#define USE_GPS_PROTO_UBLOX
#define USE_GPS_PROTO_MSP
#define USE_NAV
#define USE_TELEMETRY
#define USE_TELEMETRY_LTM
#define USE_TELEMETRY_FRSKY

#if defined(STM_FAST_TARGET)
#define SCHEDULER_DELAY_LIMIT           10
#else
#define SCHEDULER_DELAY_LIMIT           100
#endif

#if (MCU_FLASH_SIZE > 256)
#define USE_MR_BRAKING_MODE
#define USE_PITOT
#define USE_PITOT_ADC
#define USE_PITOT_VIRTUAL

#define USE_ALPHA_BETA_GAMMA_FILTER
#define USE_DYNAMIC_FILTERS
#define USE_GYRO_KALMAN
#define USE_SMITH_PREDICTOR
#define USE_RATE_DYNAMICS
#define USE_EXTENDED_CMS_MENUS
#define USE_HOTT_TEXTMODE

// NAZA GPS support for F4+ only
#define USE_GPS_PROTO_NAZA

// Allow default rangefinders
#define USE_RANGEFINDER
#define USE_RANGEFINDER_MSP
#define USE_RANGEFINDER_BENEWAKE
#define USE_RANGEFINDER_VL53L0X
#define USE_RANGEFINDER_VL53L1X
#define USE_RANGEFINDER_HCSR04_I2C
#define USE_RANGEFINDER_US42

// Allow default optic flow boards
#define USE_OPFLOW
#define USE_OPFLOW_CXOF
#define USE_OPFLOW_MSP

// Allow default airspeed sensors
#define USE_PITOT
#define USE_PITOT_MS4525
#define USE_PITOT_MSP

#define USE_1WIRE
#define USE_1WIRE_DS2482

#define USE_TEMPERATURE_SENSOR
#define USE_TEMPERATURE_LM75
#define USE_TEMPERATURE_DS18B20

#define USE_MSP_DISPLAYPORT
#define USE_DASHBOARD
#define DASHBOARD_ARMED_BITMAP
#define USE_OLED_UG2864

#define USE_PWM_DRIVER_PCA9685

#define USE_OSD
#define USE_FRSKYOSD
#define USE_DJI_HD_OSD
#define USE_SMARTPORT_MASTER

#define NAV_NON_VOLATILE_WAYPOINT_CLI

#define NAV_AUTO_MAG_DECLINATION_PRECISE

#define USE_D_BOOST
#define USE_ANTIGRAVITY

#define USE_I2C_IO_EXPANDER

#define USE_GPS_PROTO_NMEA
#define USE_GPS_PROTO_MTK

#define USE_TELEMETRY_SIM
#define USE_TELEMETRY_HOTT
#define USE_TELEMETRY_MAVLINK
#define USE_MSP_OVER_TELEMETRY

#define USE_SERIALRX_SRXL2     // Spektrum SRXL2 protocol
#define USE_SERIALRX_SUMD
#define USE_SERIALRX_SUMH
#define USE_SERIALRX_XBUS
#define USE_SERIALRX_JETIEXBUS
#define USE_SERIALRX_MAVLINK
#define USE_TELEMETRY_SRXL
#define USE_SPEKTRUM_CMS_TELEMETRY
//#define USE_SPEKTRUM_VTX_CONTROL //Some functions from betaflight still not implemented
#define USE_SPEKTRUM_VTX_TELEMETRY

#define USE_VTX_COMMON

#define USE_SERIALRX_GHST
#define USE_TELEMETRY_GHST

#define USE_SECONDARY_IMU
#define USE_IMU_BNO055

#define USE_POWER_LIMITS

#else // MCU_FLASH_SIZE < 256
#define LOG_LEVEL_MAXIMUM LOG_LEVEL_ERROR
#endif

#if (MCU_FLASH_SIZE > 128)
#define NAV_FIXED_WING_LANDING
#define USE_SAFE_HOME
#define USE_AUTOTUNE_FIXED_WING
#define USE_LOG
#define USE_STATS
#define USE_CMS
#define CMS_MENU_OSD
#define NAV_GPS_GLITCH_DETECTION
#define NAV_NON_VOLATILE_WAYPOINT_STORAGE
#define USE_TELEMETRY_IBUS
#define USE_TELEMETRY_SMARTPORT
#define USE_TELEMETRY_CRSF
// These are rather exotic serial protocols
#define USE_RX_MSP
//#define USE_MSP_RC_OVERRIDE
#define USE_SERIALRX_CRSF
#define USE_PWM_SERVO_DRIVER
#define USE_SERIAL_PASSTHROUGH
#define NAV_MAX_WAYPOINTS       60
#define USE_RCDEVICE

//Enable VTX control
#define USE_VTX_CONTROL
#define USE_VTX_SMARTAUDIO
#define USE_VTX_TRAMP
#define USE_VTX_FFPV

#ifndef STM32F3 //F3 series does not have enoug RAM to support logic conditions
#define USE_PROGRAMMING_FRAMEWORK
#define USE_CLI_BATCH
#endif

//Enable DST calculations
#define RTC_AUTOMATIC_DST
// Wind estimator
#define USE_WIND_ESTIMATOR

#else // MCU_FLASH_SIZE < 128

#define SKIP_TASK_STATISTICS

#endif
