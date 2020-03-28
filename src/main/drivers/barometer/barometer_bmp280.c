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
#include "build/build_config.h"
#include "build/debug.h"
#include "common/utils.h"

#include "drivers/time.h"
#include "drivers/io.h"
#include "drivers/bus.h"
#include "drivers/barometer/barometer.h"
#include "drivers/barometer/barometer_bmp280.h"

#if defined(USE_BARO_BMP280)

// BMP280, address 0x76

typedef struct bmp280_calib_param_s {
    uint16_t dig_T1; /* calibration T1 data */
    int16_t dig_T2; /* calibration T2 data */
    int16_t dig_T3; /* calibration T3 data */
    uint16_t dig_P1; /* calibration P1 data */
    int16_t dig_P2; /* calibration P2 data */
    int16_t dig_P3; /* calibration P3 data */
    int16_t dig_P4; /* calibration P4 data */
    int16_t dig_P5; /* calibration P5 data */
    int16_t dig_P6; /* calibration P6 data */
    int16_t dig_P7; /* calibration P7 data */
    int16_t dig_P8; /* calibration P8 data */
    int16_t dig_P9; /* calibration P9 data */
    int32_t t_fine; /* calibration t_fine data */
} bmp280_calib_param_t;

STATIC_UNIT_TESTED bmp280_calib_param_t bmp280_cal;
// uncompensated pressure and temperature
int32_t bmp280_up = 0;
int32_t bmp280_ut = 0;

static bool bmp280_start_ut(baroDev_t * baro)
{
    UNUSED(baro);
    return true;
}

static bool bmp280_get_ut(baroDev_t * baro)
{
    UNUSED(baro);
    return true;
}

static bool bmp280_start_up(baroDev_t * baro)
{
    // start measurement
    // set oversampling + power mode (forced), and start sampling
    busWrite(baro->busDev, BMP280_CTRL_MEAS_REG, BMP280_MODE);
    return true;
}

static bool bmp280_get_up(baroDev_t * baro)
{
    uint8_t data[BMP280_DATA_FRAME_SIZE];

    //error free measurements
    static int32_t bmp280_up_valid;
    static int32_t bmp280_ut_valid;

    //read data from sensor
    bool ack = busReadBuf(baro->busDev, BMP280_PRESSURE_MSB_REG, data, BMP280_DATA_FRAME_SIZE);

    //check if pressure and temperature readings are valid, otherwise use previous measurements from the moment
    if (ack) {
        bmp280_up = (int32_t)((((uint32_t)(data[0])) << 12) | (((uint32_t)(data[1])) << 4) | ((uint32_t)data[2] >> 4));
        bmp280_ut = (int32_t)((((uint32_t)(data[3])) << 12) | (((uint32_t)(data[4])) << 4) | ((uint32_t)data[5] >> 4));
        bmp280_up_valid = bmp280_up;
        bmp280_ut_valid = bmp280_ut;
    }
    else {
        //assign previous valid measurements
        bmp280_up = bmp280_up_valid;
        bmp280_ut = bmp280_ut_valid;
    }

    return ack;
}

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of "5123" equals 51.23 DegC
// t_fine carries fine temperature as global value
static int32_t bmp280_compensate_T(int32_t adc_T)
{
    int32_t var1, var2, T;

    var1 = ((((adc_T >> 3) - ((int32_t)bmp280_cal.dig_T1 << 1))) * ((int32_t)bmp280_cal.dig_T2)) >> 11;
    var2  = (((((adc_T >> 4) - ((int32_t)bmp280_cal.dig_T1)) * ((adc_T >> 4) - ((int32_t)bmp280_cal.dig_T1))) >> 12) * ((int32_t)bmp280_cal.dig_T3)) >> 14;
    bmp280_cal.t_fine = var1 + var2;
    T = (bmp280_cal.t_fine * 5 + 128) >> 8;

    return T;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of "24674867" represents 24674867/256 = 96386.2 Pa = 963.862 hPa
static uint32_t bmp280_compensate_P(int32_t adc_P)
{
    int64_t var1, var2, p;
    var1 = ((int64_t)bmp280_cal.t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)bmp280_cal.dig_P6;
    var2 = var2 + ((var1*(int64_t)bmp280_cal.dig_P5) << 17);
    var2 = var2 + (((int64_t)bmp280_cal.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)bmp280_cal.dig_P3) >> 8) + ((var1 * (int64_t)bmp280_cal.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)bmp280_cal.dig_P1) >> 33;
    if (var1 == 0) {
        return 0;
    }

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)bmp280_cal.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)bmp280_cal.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)bmp280_cal.dig_P7) << 4);
    return (uint32_t)p;
}

STATIC_UNIT_TESTED bool bmp280_calculate(baroDev_t * baro, int32_t * pressure, int32_t * temperature)
{
    UNUSED(baro);

    int32_t t = bmp280_compensate_T(bmp280_ut);
    uint32_t p = bmp280_compensate_P(bmp280_up);

    if (pressure) {
        *pressure = (int32_t)(p / 256);
    }

    if (temperature) {
        *temperature = t;
    }

    return true;
}

#define DETECTION_MAX_RETRY_COUNT   5
static bool deviceDetect(busDevice_t * busDev)
{
    for (int retry = 0; retry < DETECTION_MAX_RETRY_COUNT; retry++) {
        uint8_t chipId = 0;

        delay(100);

        bool ack = busRead(busDev, BMP280_CHIP_ID_REG, &chipId);

        if (ack && (chipId == BMP280_DEFAULT_CHIP_ID || chipId == BME280_DEFAULT_CHIP_ID)) {
            return true;
        }
    };

    return false;
}

bool bmp280Detect(baroDev_t *baro)
{
    baro->busDev = busDeviceInit(BUSTYPE_ANY, DEVHW_BMP280, 0, OWNER_BARO);
    if (baro->busDev == NULL) {
        return false;
    }

    busSetSpeed(baro->busDev, BUS_SPEED_STANDARD);

    if (!deviceDetect(baro->busDev)) {
        busDeviceDeInit(baro->busDev);
        return false;
    }

    // read calibration
    busReadBuf(baro->busDev, BMP280_TEMPERATURE_CALIB_DIG_T1_LSB_REG, (uint8_t *)&bmp280_cal, 24);

    //set filter setting
    busWrite(baro->busDev, BMP280_CONFIG_REG, BMP280_FILTER);

    // set oversampling + power mode (forced), and start sampling
    busWrite(baro->busDev, BMP280_CTRL_MEAS_REG, BMP280_MODE);

    baro->ut_delay = 0;
    baro->get_ut = bmp280_get_ut;
    baro->start_ut = bmp280_start_ut;

    baro->up_delay = ((T_INIT_MAX + T_MEASURE_PER_OSRS_MAX * (((1 << BMP280_TEMPERATURE_OSR) >> 1) + ((1 << BMP280_PRESSURE_OSR) >> 1)) + (BMP280_PRESSURE_OSR ? T_SETUP_PRESSURE_MAX : 0) + 15) / 16) * 1000;
    baro->start_up = bmp280_start_up;
    baro->get_up = bmp280_get_up;

    baro->calculate = bmp280_calculate;

    return true;
}

#endif
