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
#include <stdlib.h>
#include <stdint.h>

#include "platform.h"

#include "cms/cms.h"

#include "common/axis.h"
#include "common/color.h"
#include "common/utils.h"
#include "programming/programming_task.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/compass/compass.h"
#include "drivers/sensor.h"
#include "drivers/serial.h"
#include "drivers/stack_check.h"
#include "drivers/pwm_mapping.h"

#include "fc/cli.h"
#include "fc/config.h"
#include "fc/fc_core.h"
#include "fc/fc_msp.h"
#include "fc/fc_tasks.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/dynamic_lpf.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/power_limits.h"
#include "flight/rpm_filter.h"
#include "flight/secondary_imu.h"
#include "flight/servos.h"
#include "flight/wind_estimator.h"

#include "navigation/navigation.h"

#include "io/beeper.h"
#include "io/lights.h"
#include "io/dashboard.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/osd.h"
#include "io/pwmdriver_i2c.h"
#include "io/serial.h"
#include "io/rcdevice_cam.h"
#include "io/smartport_master.h"
#include "io/vtx.h"
#include "io/osd_dji_hd.h"
#include "io/servo_sbus.h"

#include "msp/msp_serial.h"

#include "rx/rx.h"
#include "rx/eleres.h"
#include "rx/rx_spi.h"

#include "scheduler/scheduler.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/temperature.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"
#include "sensors/irlock.h"
#include "sensors/pitotmeter.h"
#include "sensors/rangefinder.h"
#include "sensors/opflow.h"

#include "telemetry/telemetry.h"

#include "config/feature.h"

void taskHandleSerial(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
    // in cli mode, all serial stuff goes to here. enter cli mode by sending #
    if (cliMode) {
        cliProcess();
    }

    // Allow MSP processing even if in CLI mode
    mspSerialProcess(ARMING_FLAG(ARMED) ? MSP_SKIP_NON_MSP_DATA : MSP_EVALUATE_NON_MSP_DATA, mspFcProcessCommand);

#if defined(USE_DJI_HD_OSD)
    // DJI OSD uses a special flavour of MSP (subset of Betaflight 4.1.1 MSP) - process as part of serial task
    djiOsdSerialProcess();
#endif
}

void taskUpdateBattery(timeUs_t currentTimeUs)
{
    static timeUs_t batMonitoringLastServiced = 0;
    timeDelta_t BatMonitoringTimeSinceLastServiced = cmpTimeUs(currentTimeUs, batMonitoringLastServiced);

    if (isAmperageConfigured()) {
        currentMeterUpdate(BatMonitoringTimeSinceLastServiced);
#ifdef USE_POWER_LIMITS
        currentLimiterUpdate(BatMonitoringTimeSinceLastServiced);
#endif
    }

#ifdef USE_ADC
    if (feature(FEATURE_VBAT)) {
        batteryUpdate(BatMonitoringTimeSinceLastServiced);
    }

    if (feature(FEATURE_VBAT) && isAmperageConfigured()) {
        powerMeterUpdate(BatMonitoringTimeSinceLastServiced);
        sagCompensatedVBatUpdate(currentTimeUs, BatMonitoringTimeSinceLastServiced);
#if defined(USE_POWER_LIMITS) && defined(USE_ADC)
        powerLimiterUpdate(BatMonitoringTimeSinceLastServiced);
#endif
    }
#endif

    batMonitoringLastServiced = currentTimeUs;
}

void taskUpdateTemperature(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
    temperatureUpdate();
}

#ifdef USE_GPS
void taskProcessGPS(timeUs_t currentTimeUs)
{
    // if GPS feature is enabled, gpsThread() will be called at some intervals to check for stuck
    // hardware, wrong baud rates, init GPS if needed, etc. Don't use SENSOR_GPS here as gpsThread() can and will
    // change this based on available hardware
    if (feature(FEATURE_GPS)) {
        if (gpsUpdate()) {
#ifdef USE_WIND_ESTIMATOR
            updateWindEstimator(currentTimeUs);
#endif
        }
    }

    if (sensors(SENSOR_GPS)) {
        updateGpsIndicator(currentTimeUs);
    }
}
#endif

#ifdef USE_MAG
void taskUpdateCompass(timeUs_t currentTimeUs)
{
    if (sensors(SENSOR_MAG)) {
        compassUpdate(currentTimeUs);
    }
}
#endif

#ifdef USE_BARO
void taskUpdateBaro(timeUs_t currentTimeUs)
{
    if (!sensors(SENSOR_BARO)) {
        return;
    }

    const uint32_t newDeadline = baroUpdate();
    if (newDeadline != 0) {
        rescheduleTask(TASK_SELF, newDeadline);
    }

    updatePositionEstimator_BaroTopic(currentTimeUs);
}
#endif

#ifdef USE_PITOT
void taskUpdatePitot(timeUs_t currentTimeUs)
{
    if (!sensors(SENSOR_PITOT)) {
        return;
    }

    pitotUpdate();
    updatePositionEstimator_PitotTopic(currentTimeUs);
}
#endif

#ifdef USE_RANGEFINDER
void taskUpdateRangefinder(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    if (!sensors(SENSOR_RANGEFINDER))
        return;

    // Update and adjust task to update at required rate
    const uint32_t newDeadline = rangefinderUpdate();
    if (newDeadline != 0) {
        rescheduleTask(TASK_SELF, newDeadline);
    }

    /*
     * Process raw rangefinder readout
     */
    if (rangefinderProcess(calculateCosTiltAngle())) {
        updatePositionEstimator_SurfaceTopic(currentTimeUs, rangefinderGetLatestAltitude());
    }
}
#endif

#if defined(USE_NAV) && defined(USE_IRLOCK)
void taskUpdateIrlock(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
    irlockUpdate();
}
#endif

#ifdef USE_OPFLOW
void taskUpdateOpticalFlow(timeUs_t currentTimeUs)
{
    if (!sensors(SENSOR_OPFLOW))
        return;

    opflowUpdate(currentTimeUs);
    updatePositionEstimator_OpticalFlowTopic(currentTimeUs);
}
#endif

#ifdef USE_DASHBOARD
void taskDashboardUpdate(timeUs_t currentTimeUs)
{
    if (feature(FEATURE_DASHBOARD)) {
        dashboardUpdate(currentTimeUs);
    }
}
#endif

#ifdef USE_TELEMETRY
void taskTelemetry(timeUs_t currentTimeUs)
{
    telemetryCheckState();

    if (!cliMode && feature(FEATURE_TELEMETRY)) {
        telemetryProcess(currentTimeUs);
    }
}
#endif

#if defined(USE_SMARTPORT_MASTER)
void taskSmartportMaster(timeUs_t currentTimeUs)
{
    smartportMasterHandle(currentTimeUs);
}
#endif

#ifdef USE_LED_STRIP
void taskLedStrip(timeUs_t currentTimeUs)
{
    if (feature(FEATURE_LED_STRIP)) {
        ledStripUpdate(currentTimeUs);
    }
}
#endif

void taskSyncServoDriver(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

#if defined(USE_SERVO_SBUS)
    sbusServoSendUpdate();
#endif

#ifdef USE_PWM_SERVO_DRIVER
    pwmDriverSync();
#endif
}

#ifdef USE_OSD
void taskUpdateOsd(timeUs_t currentTimeUs)
{
    if (feature(FEATURE_OSD)) {
        osdUpdate(currentTimeUs);
    }
}
#endif

void taskUpdateAux(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
    updatePIDCoefficients();
    dynamicLpfGyroTask();
}

void fcTasksInit(void)
{
    schedulerInit();

    rescheduleTask(TASK_PID, getLooptime());
    setTaskEnabled(TASK_PID, true);

    rescheduleTask(TASK_GYRO, getGyroLooptime());
    setTaskEnabled(TASK_GYRO, true);

    setTaskEnabled(TASK_AUX, true);

    setTaskEnabled(TASK_SERIAL, true);
#if defined(BEEPER) || defined(USE_DSHOT)
    setTaskEnabled(TASK_BEEPER, true);
#endif
#ifdef USE_LIGHTS
    setTaskEnabled(TASK_LIGHTS, true);
#endif
    setTaskEnabled(TASK_BATTERY, feature(FEATURE_VBAT) || isAmperageConfigured());
    setTaskEnabled(TASK_TEMPERATURE, true);
    setTaskEnabled(TASK_RX, true);
#ifdef USE_GPS
    setTaskEnabled(TASK_GPS, feature(FEATURE_GPS));
#endif
#ifdef USE_MAG
    setTaskEnabled(TASK_COMPASS, sensors(SENSOR_MAG));
#if defined(USE_MAG_MPU9250)
    // fixme temporary solution for AK6983 via slave I2C on MPU9250
    rescheduleTask(TASK_COMPASS, TASK_PERIOD_HZ(40));
#endif
#endif
#ifdef USE_BARO
    setTaskEnabled(TASK_BARO, sensors(SENSOR_BARO));
#endif
#ifdef USE_PITOT
    setTaskEnabled(TASK_PITOT, sensors(SENSOR_PITOT));
#endif
#ifdef USE_RANGEFINDER
    setTaskEnabled(TASK_RANGEFINDER, sensors(SENSOR_RANGEFINDER));
#endif
#ifdef USE_DASHBOARD
    setTaskEnabled(TASK_DASHBOARD, feature(FEATURE_DASHBOARD));
#endif
#ifdef USE_TELEMETRY
    setTaskEnabled(TASK_TELEMETRY, feature(FEATURE_TELEMETRY));
#endif
#ifdef USE_LED_STRIP
    setTaskEnabled(TASK_LEDSTRIP, feature(FEATURE_LED_STRIP));
#endif
#ifdef STACK_CHECK
    setTaskEnabled(TASK_STACK_CHECK, true);
#endif
#if defined(USE_PWM_SERVO_DRIVER) || defined(USE_SERVO_SBUS)
    setTaskEnabled(TASK_PWMDRIVER, (servoConfig()->servo_protocol == SERVO_TYPE_SERVO_DRIVER) || (servoConfig()->servo_protocol == SERVO_TYPE_SBUS) || (servoConfig()->servo_protocol == SERVO_TYPE_SBUS_PWM));
#endif
#ifdef USE_CMS
#ifdef USE_MSP_DISPLAYPORT
    setTaskEnabled(TASK_CMS, true);
#else
    setTaskEnabled(TASK_CMS, feature(FEATURE_OSD) || feature(FEATURE_DASHBOARD));
#endif
#endif
#ifdef USE_OPFLOW
    setTaskEnabled(TASK_OPFLOW, sensors(SENSOR_OPFLOW));
#endif
#ifdef USE_VTX_CONTROL
#if defined(USE_VTX_SMARTAUDIO) || defined(USE_VTX_TRAMP)
    setTaskEnabled(TASK_VTXCTRL, true);
#endif
#endif
#ifdef USE_RCDEVICE
    setTaskEnabled(TASK_RCDEVICE, rcdeviceIsEnabled());
#endif
#ifdef USE_PROGRAMMING_FRAMEWORK
    setTaskEnabled(TASK_PROGRAMMING_FRAMEWORK, true);
#endif
#ifdef USE_IRLOCK
    setTaskEnabled(TASK_IRLOCK, irlockHasBeenDetected());
#endif
#if defined(USE_SMARTPORT_MASTER)
    setTaskEnabled(TASK_SMARTPORT_MASTER, true);
#endif
#ifdef USE_SECONDARY_IMU
    setTaskEnabled(TASK_SECONDARY_IMU, secondaryImuConfig()->hardwareType != SECONDARY_IMU_NONE && secondaryImuState.active);
#endif
}

cfTask_t cfTasks[TASK_COUNT] = {
    [TASK_SYSTEM] = {
        .taskName = "SYSTEM",
        .taskFunc = taskSystem,
        .desiredPeriod = TASK_PERIOD_HZ(10),              // run every 100 ms, 10Hz
        .staticPriority = TASK_PRIORITY_HIGH,
    },
    [TASK_PID] = {
        .taskName = "PID",
        .taskFunc = taskMainPidLoop,
        .desiredPeriod = TASK_PERIOD_US(1000),
        .staticPriority = TASK_PRIORITY_REALTIME,
    },
    [TASK_GYRO] = {
        .taskName = "GYRO",
        .taskFunc = taskGyro,
        .desiredPeriod = TASK_PERIOD_US(TASK_GYRO_LOOPTIME),
        .staticPriority = TASK_PRIORITY_REALTIME,
    },
    [TASK_SERIAL] = {
        .taskName = "SERIAL",
        .taskFunc = taskHandleSerial,
        .desiredPeriod = TASK_PERIOD_HZ(100),     // 100 Hz should be enough to flush up to 115 bytes @ 115200 baud
        .staticPriority = TASK_PRIORITY_LOW,
    },

#if defined(BEEPER) || defined(USE_DSHOT)
    [TASK_BEEPER] = {
        .taskName = "BEEPER",
        .taskFunc = beeperUpdate,
        .desiredPeriod = TASK_PERIOD_HZ(100),     // 100 Hz
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },
#endif

#ifdef USE_LIGHTS
    [TASK_LIGHTS] = {
        .taskName = "LIGHTS",
        .taskFunc = lightsUpdate,
        .desiredPeriod = TASK_PERIOD_HZ(100),     // 100 Hz
        .staticPriority = TASK_PRIORITY_LOW,
    },
#endif

    [TASK_BATTERY] = {
        .taskName = "BATTERY",
        .taskFunc = taskUpdateBattery,
        .desiredPeriod = TASK_PERIOD_HZ(50),      // 50 Hz
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },

    [TASK_TEMPERATURE] = {
        .taskName = "TEMPERATURE",
        .taskFunc = taskUpdateTemperature,
        .desiredPeriod = TASK_PERIOD_HZ(100),     // 100 Hz
        .staticPriority = TASK_PRIORITY_LOW,
    },

    [TASK_RX] = {
        .taskName = "RX",
        .checkFunc = taskUpdateRxCheck,
        .taskFunc = taskUpdateRxMain,
        .desiredPeriod = TASK_PERIOD_HZ(50),      // If event-based scheduling doesn't work, fallback to periodic scheduling
        .staticPriority = TASK_PRIORITY_HIGH,
    },

#ifdef USE_GPS
    [TASK_GPS] = {
        .taskName = "GPS",
        .taskFunc = taskProcessGPS,
        .desiredPeriod = TASK_PERIOD_HZ(50),      // GPS usually don't go raster than 10Hz
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },
#endif

#ifdef USE_MAG
    [TASK_COMPASS] = {
        .taskName = "COMPASS",
        .taskFunc = taskUpdateCompass,
        .desiredPeriod = TASK_PERIOD_HZ(10),      // Compass is updated at 10 Hz
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },
#endif

#ifdef USE_BARO
    [TASK_BARO] = {
        .taskName = "BARO",
        .taskFunc = taskUpdateBaro,
        .desiredPeriod = TASK_PERIOD_HZ(20),
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },
#endif

#ifdef USE_PITOT
    [TASK_PITOT] = {
        .taskName = "PITOT",
        .taskFunc = taskUpdatePitot,
        .desiredPeriod = TASK_PERIOD_HZ(100),
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },
#endif

#ifdef USE_RANGEFINDER
    [TASK_RANGEFINDER] = {
        .taskName = "RANGEFINDER",
        .taskFunc = taskUpdateRangefinder,
        .desiredPeriod = TASK_PERIOD_MS(70),
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },
#endif

#ifdef USE_IRLOCK
    [TASK_IRLOCK] = {
        .taskName = "IRLOCK",
        .taskFunc = taskUpdateIrlock,
        .desiredPeriod = TASK_PERIOD_HZ(100),
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },
#endif

#ifdef USE_DASHBOARD
    [TASK_DASHBOARD] = {
        .taskName = "DASHBOARD",
        .taskFunc = taskDashboardUpdate,
        .desiredPeriod = TASK_PERIOD_HZ(10),
        .staticPriority = TASK_PRIORITY_LOW,
    },
#endif

#ifdef USE_TELEMETRY
    [TASK_TELEMETRY] = {
        .taskName = "TELEMETRY",
        .taskFunc = taskTelemetry,
        .desiredPeriod = TASK_PERIOD_HZ(500),         // 500 Hz
        .staticPriority = TASK_PRIORITY_IDLE,
    },
#endif

#if defined(USE_SMARTPORT_MASTER)
    [TASK_SMARTPORT_MASTER] = {
        .taskName = "SPORT MASTER",
        .taskFunc = taskSmartportMaster,
        .desiredPeriod = TASK_PERIOD_HZ(500),         // 500 Hz
        .staticPriority = TASK_PRIORITY_IDLE,
    },
#endif

#ifdef USE_LED_STRIP
    [TASK_LEDSTRIP] = {
        .taskName = "LEDSTRIP",
        .taskFunc = taskLedStrip,
        .desiredPeriod = TASK_PERIOD_HZ(100),         // 100 Hz
        .staticPriority = TASK_PRIORITY_IDLE,
    },
#endif

#if defined(USE_PWM_SERVO_DRIVER) || defined(USE_SERVO_SBUS)
    [TASK_PWMDRIVER] = {
        .taskName = "SERVOS",
        .taskFunc = taskSyncServoDriver,
        .desiredPeriod = TASK_PERIOD_HZ(200),         // 200 Hz
        .staticPriority = TASK_PRIORITY_HIGH,
    },
#endif

#ifdef STACK_CHECK
    [TASK_STACK_CHECK] = {
        .taskName = "STACKCHECK",
        .taskFunc = taskStackCheck,
        .desiredPeriod = TASK_PERIOD_HZ(10),          // 10 Hz
        .staticPriority = TASK_PRIORITY_IDLE,
    },
#endif

#ifdef USE_OSD
    [TASK_OSD] = {
        .taskName = "OSD",
        .taskFunc = taskUpdateOsd,
        .desiredPeriod = TASK_PERIOD_HZ(250),
        .staticPriority = TASK_PRIORITY_LOW,
    },
#endif

#ifdef USE_CMS
    [TASK_CMS] = {
        .taskName = "CMS",
        .taskFunc = cmsHandler,
        .desiredPeriod = TASK_PERIOD_HZ(50),
        .staticPriority = TASK_PRIORITY_LOW,
    },
#endif

#ifdef USE_OPFLOW
    [TASK_OPFLOW] = {
        .taskName = "OPFLOW",
        .taskFunc = taskUpdateOpticalFlow,
        .desiredPeriod = TASK_PERIOD_HZ(100),   // I2C/SPI sensor will work at higher rate and accumulate, UART sensor will work at lower rate w/o accumulation
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },
#endif

#ifdef USE_RCDEVICE
    [TASK_RCDEVICE] = {
        .taskName = "RCDEVICE",
        .taskFunc = rcdeviceUpdate,
        .desiredPeriod = TASK_PERIOD_HZ(10),        // 10 Hz, 100ms
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },
#endif

#if defined(USE_VTX_CONTROL)
    [TASK_VTXCTRL] = {
        .taskName = "VTXCTRL",
        .taskFunc = vtxUpdate,
        .desiredPeriod = TASK_PERIOD_HZ(5),          // 5Hz @200msec
        .staticPriority = TASK_PRIORITY_IDLE,
    },
#endif
#ifdef USE_PROGRAMMING_FRAMEWORK
    [TASK_PROGRAMMING_FRAMEWORK] = {
        .taskName = "PROGRAMMING",
        .taskFunc = programmingFrameworkUpdateTask,
        .desiredPeriod = TASK_PERIOD_HZ(10),          // 10Hz @100msec
        .staticPriority = TASK_PRIORITY_IDLE,
    },
#endif
#ifdef USE_SECONDARY_IMU
    [TASK_SECONDARY_IMU] = {
        .taskName = "IMU2",
        .taskFunc = taskSecondaryImu,
        .desiredPeriod = TASK_PERIOD_HZ(10),          // 10Hz @100msec
        .staticPriority = TASK_PRIORITY_IDLE,
    },
#endif
#ifdef USE_RPM_FILTER
    [TASK_RPM_FILTER] = {
        .taskName = "RPM",
        .taskFunc = rpmFilterUpdateTask,
        .desiredPeriod = TASK_PERIOD_HZ(RPM_FILTER_UPDATE_RATE_HZ),          // 300Hz @3,33ms
        .staticPriority = TASK_PRIORITY_LOW,
    },
#endif
    [TASK_AUX] = {
        .taskName = "AUX",
        .taskFunc = taskUpdateAux,
        .desiredPeriod = TASK_PERIOD_HZ(TASK_AUX_RATE_HZ),          // 100Hz @10ms
        .staticPriority = TASK_PRIORITY_HIGH,
    },
};
