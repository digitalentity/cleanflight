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

#include <stdint.h>
#include <stdbool.h>

#include <limits.h>
#include <algorithm>

extern "C" {
    #include <platform.h>

    #include "build/build_config.h"
    #include "build/debug.h"
    #include "common/axis.h"
    #include "common/maths.h"
    #include "common/calibration.h"
    #include "common/utils.h"
    #include "drivers/accgyro/accgyro_fake.h"
    #include "drivers/logging_codes.h"
    #include "io/beeper.h"
    #include "scheduler/scheduler.h"
    #include "sensors/gyro.h"
    #include "sensors/acceleration.h"
    #include "sensors/sensors.h"
    #include "fc/rc_controls.h"
    #include "flight/mixer.h"

    extern zeroCalibrationVector_t gyroCalibration;
    extern gyroDev_t gyroDev[];

    STATIC_UNIT_TESTED gyroSensor_e gyroDetect(gyroDev_t *dev, gyroSensor_e gyroHardware);
    STATIC_UNIT_TESTED void performGyroCalibration(gyroDev_t *dev, zeroCalibrationVector_t *gyroCalibration);
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

TEST(SensorGyro, Detect)
{
    const gyroSensor_e detected = gyroDetect(&gyroDev[0], GYRO_AUTODETECT);
    EXPECT_EQ(GYRO_FAKE, detected);
}

TEST(SensorGyro, Init)
{
    const bool initialised = gyroInit();
    EXPECT_EQ(true, initialised);
    EXPECT_EQ(GYRO_FAKE, detectedSensors[SENSOR_INDEX_GYRO]);
}

TEST(SensorGyro, Read)
{
    gyroInit();
    EXPECT_EQ(GYRO_FAKE, detectedSensors[SENSOR_INDEX_GYRO]);
    fakeGyroSet(5, 6, 7);
    const bool read = gyroDev[0].readFn(&gyroDev[0]);
    EXPECT_EQ(true, read);
    EXPECT_EQ(5, gyroDev[0].gyroADCRaw[X]);
    EXPECT_EQ(6, gyroDev[0].gyroADCRaw[Y]);
    EXPECT_EQ(7, gyroDev[0].gyroADCRaw[Z]);
}

TEST(SensorGyro, Calibrate)
{
    gyroInit();
    gyroStartCalibration();
    fakeGyroSet(5, 6, 7);
    const bool read = gyroDev[0].readFn(&gyroDev[0]);
    EXPECT_EQ(true, read);
    EXPECT_EQ(5, gyroDev[0].gyroADCRaw[X]);
    EXPECT_EQ(6, gyroDev[0].gyroADCRaw[Y]);
    EXPECT_EQ(7, gyroDev[0].gyroADCRaw[Z]);
    gyroDev[0].gyroZero[X] = 8;
    gyroDev[0].gyroZero[Y] = 9;
    gyroDev[0].gyroZero[Z] = 10;
    performGyroCalibration(&gyroDev[0], &gyroCalibration);
    EXPECT_EQ(0, gyroDev[0].gyroZero[X]);
    EXPECT_EQ(0, gyroDev[0].gyroZero[Y]);
    EXPECT_EQ(0, gyroDev[0].gyroZero[Z]);
    EXPECT_EQ(false, gyroIsCalibrationComplete());
    while (!gyroIsCalibrationComplete()) {
        performGyroCalibration(&gyroDev[0], &gyroCalibration);
    }
    EXPECT_EQ(5, gyroDev[0].gyroZero[X]);
    EXPECT_EQ(6, gyroDev[0].gyroZero[Y]);
    EXPECT_EQ(7, gyroDev[0].gyroZero[Z]);
}

TEST(SensorGyro, Update)
{
    gyroInit();
    gyroStartCalibration();
    EXPECT_EQ(false, gyroIsCalibrationComplete());
    gyroInit();
    fakeGyroSet(5, 6, 7);
    gyroUpdate();
    EXPECT_EQ(false, gyroIsCalibrationComplete());
    while (!gyroIsCalibrationComplete()) {
        gyroUpdate();
    }
    EXPECT_EQ(true, gyroIsCalibrationComplete());
    EXPECT_EQ(5, gyroDev[0].gyroZero[X]);
    EXPECT_EQ(6, gyroDev[0].gyroZero[Y]);
    EXPECT_EQ(7, gyroDev[0].gyroZero[Z]);
    EXPECT_FLOAT_EQ(0, gyro.gyroADCf[X]);
    EXPECT_FLOAT_EQ(0, gyro.gyroADCf[Y]);
    EXPECT_FLOAT_EQ(0, gyro.gyroADCf[Z]);
    gyroUpdate();
    // expect zero values since gyro is calibrated
    EXPECT_FLOAT_EQ(0, gyro.gyroADCf[X]);
    EXPECT_FLOAT_EQ(0, gyro.gyroADCf[Y]);
    EXPECT_FLOAT_EQ(0, gyro.gyroADCf[Z]);
    fakeGyroSet(15, 26, 97);
    gyroUpdate();
    EXPECT_FLOAT_EQ(10 * gyroDev[0].scale, gyro.gyroADCf[X]); // gyroADCf values are scaled
    EXPECT_FLOAT_EQ(20 * gyroDev[0].scale, gyro.gyroADCf[Y]);
    EXPECT_FLOAT_EQ(90 * gyroDev[0].scale, gyro.gyroADCf[Z]);
}


// STUBS

extern "C" {
static timeMs_t milliTime = 0;

timeMs_t millis(void) {return milliTime++;}
uint32_t micros(void) {return 0;}
void beeper(beeperMode_e) {}
uint8_t detectedSensors[] = { GYRO_NONE, ACC_NONE };
timeDelta_t getLooptime(void) {return gyro.targetLooptime;}
timeDelta_t getGyroLooptime(void) {return gyro.targetLooptime;}
void sensorsSet(uint32_t) {}
void schedulerResetTaskStatistics(cfTaskId_e) {}
}
