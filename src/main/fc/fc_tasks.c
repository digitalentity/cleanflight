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
#include "common/logic_condition.h"
#include "common/global_functions.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/compass/compass.h"
#include "drivers/sensor.h"
#include "drivers/serial.h"
#include "drivers/stack_check.h"
#include "drivers/pwm_output.h"

#include "fc/cli.h"
#include "fc/config.h"
#include "fc/fc_core.h"
#include "fc/fc_msp.h"
#include "fc/fc_tasks.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/wind_estimator.h"
#include "flight/rpm_filter.h"

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
#include "io/vtx.h"
#include "io/osd_dji_hd.h"

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
#include "sensors/pitotmeter.h"
#include "sensors/rangefinder.h"
#include "sensors/opflow.h"

#include "telemetry/telemetry.h"

#include "config/feature.h"

#include "uav_interconnect/uav_interconnect.h"

TASK(taskHandleSerial)
{
    taskBegin();

    while(1) {
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

        taskYield();
    }

    taskEnd();
}

#ifdef USE_BARO
TASK(taskBaroUpdate)
{
    taskBegin();

    while(sensors(SENSOR_BARO)) {
        const timeUs_t newDeadline = baroUpdate();
        updatePositionEstimator_BaroTopic(currentTimeUs);

        if (newDeadline != 0) {
            taskDelayUs(newDeadline);
        }
        else {
            taskYield();
        }
    }

    taskEnd();
}
#endif

TASK(taskBatteryUpdate)
{
    taskBegin();

    static schdTimer_t batTimer;
    taskTimerInit(&batTimer, HZ2US(50));

    while(1) {
        taskWaitTimer(&batTimer);
        timeUs_t batDeltaUs = taskTimerGetLastInterval(&batTimer);

        if (isAmperageConfigured()) {
            currentMeterUpdate(batDeltaUs);
        }

#ifdef USE_ADC
        if (feature(FEATURE_VBAT)) {
            batteryUpdate(batDeltaUs);
        }

        if (feature(FEATURE_VBAT) && isAmperageConfigured()) {
            powerMeterUpdate(batDeltaUs);
            sagCompensatedVBatUpdate(currentTimeUs, batDeltaUs);
        }
#endif
    }

    taskEnd();
}

TASK(taskUpdateTemp)
{
    taskBegin();

    static schdTimer_t tempTimer;
    taskTimerInit(&tempTimer, HZ2US(100));

    while(1) {
        taskWaitTimer(&tempTimer);
        temperatureUpdate();
    }

    taskEnd();
}

#ifdef USE_GPS
TASK(taskProcessGPS)
{
    taskBegin();

    static schdTimer_t gpsTimer;
    taskTimerInit(&gpsTimer, HZ2US(50));

    while(1) {
        taskWaitTimer(&gpsTimer);

        if (gpsUpdate()) {
#ifdef USE_WIND_ESTIMATOR
            updateWindEstimator(currentTimeUs);
#endif
        }

        if (sensors(SENSOR_GPS)) {
            updateGpsIndicator(currentTimeUs);
        }
    }

    taskEnd();
}
#endif

#ifdef USE_MAG
TASK(taskUpdateCompass)
{
    taskBegin();

    static schdTimer_t magTimer;

    // fixme temporary solution for AK6983 via slave I2C on MPU9250
    if (detectedSensors[SENSOR_INDEX_MAG] == MAG_MPU9250) {
        taskTimerInit(&magTimer, HZ2US(40));
    }
    else {
        taskTimerInit(&magTimer, HZ2US(10));
    }

    while(1) {
        taskWaitTimer(&magTimer);
        compassUpdate(currentTimeUs);
    }

    taskEnd();
}
#endif

/*
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

    // Process raw rangefinder readout
    if (rangefinderProcess(calculateCosTiltAngle())) {
        updatePositionEstimator_SurfaceTopic(currentTimeUs, rangefinderGetLatestAltitude());
    }
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

#ifdef USE_LED_STRIP
void taskLedStrip(timeUs_t currentTimeUs)
{
    if (feature(FEATURE_LED_STRIP)) {
        ledStripUpdate(currentTimeUs);
    }
}
#endif

#ifdef USE_PWM_SERVO_DRIVER
void taskSyncPwmDriver(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    if (feature(FEATURE_PWM_SERVO_DRIVER)) {
        pwmDriverSync();
    }
}
#endif
*/

void fcTasksInit(void)
{
    taskCreate(taskGyro,                    "GYRO",         TASK_PRIORITY_HIGHEST,      NULL);
    taskCreate(taskAHRS,                    "AHRS",         TASK_PRIORITY_HIGHEST,      NULL);
    taskCreate(taskMainPidLoop,             "PID",          TASK_PRIORITY_HIGHEST,      NULL);
    taskCreate(taskHandleSerial,            "SERIAL",       TASK_PRIORITY_LOW,          NULL);

#ifdef BEEPER
    taskCreate(taskBeeper,                  "BEEPER",       TASK_PRIORITY_LOWEST,       NULL);
#endif

#ifdef USE_SDCARD
    taskCreate(taskRealtimeSDCard,          "SDCARD",       TASK_PRIORITY_HIGHEST,      NULL);
#endif

#ifdef USE_DSHOT
    if (isMotorProtocolDigital()) {
        taskCreate(taskRealtimeMotorUpdate, "MOTOR/DSHOT",  TASK_PRIORITY_HIGHEST,      NULL);
    }
#endif

#ifdef USE_BARO
    if (sensors(SENSOR_BARO)) {
        taskCreate(taskBaroUpdate,          "BARO",         TASK_PRIORITY_LOW,          NULL);
    }
#endif

    if (feature(FEATURE_VBAT) || isAmperageConfigured()) {
        taskCreate(taskBatteryUpdate,       "BATTERY",      TASK_PRIORITY_LOW,          NULL);
    }

    taskCreate(taskUpdateRx,                "RX_UPDATE",    TASK_PRIORITY_MEDIUM,       NULL);
    taskCreate(taskProcessRx,               "RX_PROCESS",   TASK_PRIORITY_LOW,          NULL);

    taskCreate(taskUpdateTemp,              "TEMPERATURE",  TASK_PRIORITY_LOW,          NULL);

#ifdef USE_GPS
    if (feature(FEATURE_GPS)) {
        taskCreate(taskProcessGPS,          "GPS",          TASK_PRIORITY_MEDIUM,       NULL);
    }
#endif

#ifdef USE_MAG
    if (sensors(SENSOR_MAG)) {
        taskCreate(taskUpdateCompass,       "COMPASS",      TASK_PRIORITY_MEDIUM,       NULL);
    }
#endif

#ifdef USE_OSD
    if (feature(FEATURE_OSD) && (osdGetDisplayPort() != NULL)) {
        taskCreate(taskUpdateOsd,           "OSD",          TASK_PRIORITY_LOW,          NULL);
    }
#endif


#ifdef USE_LIGHTS
    taskCreate(taskLights,                  "LIGHTS",       TASK_PRIORITY_LOW,          NULL);
    lightsUpdate
    setTaskEnabled(TASK_LIGHTS, true);
#endif
}

/*
void fcTasksInit(void)
{
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
#ifdef USE_PWM_SERVO_DRIVER
    setTaskEnabled(TASK_PWMDRIVER, feature(FEATURE_PWM_SERVO_DRIVER));
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
#ifdef USE_UAV_INTERCONNECT
    setTaskEnabled(TASK_UAV_INTERCONNECT, uavInterconnectBusIsInitialized());
#endif
#ifdef USE_RCDEVICE
    setTaskEnabled(TASK_RCDEVICE, rcdeviceIsEnabled());
#endif
#ifdef USE_LOGIC_CONDITIONS
    setTaskEnabled(TASK_LOGIC_CONDITIONS, true);
#endif
#ifdef USE_GLOBAL_FUNCTIONS
    setTaskEnabled(TASK_GLOBAL_FUNCTIONS, true);
#endif
}

cfTask_t cfTasks[TASK_COUNT] = {
    [TASK_SYSTEM] = {
        .taskName = "SYSTEM",
        .taskFunc = taskSystem,
        .desiredPeriod = TASK_PERIOD_HZ(10),              // run every 100 ms, 10Hz
        .staticPriority = TASK_PRIORITY_HIGH,
    },
    [TASK_GYROPID] = {
        .taskName = "GYRO/PID",
        .taskFunc = taskMainPidLoop,
        .desiredPeriod = TASK_PERIOD_US(1000),
        .staticPriority = TASK_PRIORITY_REALTIME,
    },
    [TASK_SERIAL] = {
        .taskName = "SERIAL",
        .taskFunc = taskHandleSerial,
        .desiredPeriod = TASK_PERIOD_HZ(100),     // 100 Hz should be enough to flush up to 115 bytes @ 115200 baud
        .staticPriority = TASK_PRIORITY_LOW,
    },

#ifdef BEEPER
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

#ifdef USE_LED_STRIP
    [TASK_LEDSTRIP] = {
        .taskName = "LEDSTRIP",
        .taskFunc = taskLedStrip,
        .desiredPeriod = TASK_PERIOD_HZ(100),         // 100 Hz
        .staticPriority = TASK_PRIORITY_IDLE,
    },
#endif

#ifdef USE_PWM_SERVO_DRIVER
    [TASK_PWMDRIVER] = {
        .taskName = "PWMDRIVER",
        .taskFunc = taskSyncPwmDriver,
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
        .desiredPeriod = TASK_PERIOD_HZ(100),   // I2C/SPI sensor will work at higher rate and accumulate, UIB/UART sensor will work at lower rate w/o accumulation
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },
#endif

#ifdef USE_UAV_INTERCONNECT
    [TASK_UAV_INTERCONNECT] = {
        .taskName = "UIB",
        .taskFunc = uavInterconnectBusTask,
        .desiredPeriod = 1000000 / 500,          // 500 Hz
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
#ifdef USE_LOGIC_CONDITIONS
    [TASK_LOGIC_CONDITIONS] = {
        .taskName = "LOGIC",
        .taskFunc = logicConditionUpdateTask,
        .desiredPeriod = TASK_PERIOD_HZ(10),          // 10Hz @100msec
        .staticPriority = TASK_PRIORITY_IDLE,
    },
#endif
#ifdef USE_GLOBAL_FUNCTIONS
    [TASK_GLOBAL_FUNCTIONS] = {
        .taskName = "G_FNK",
        .taskFunc = globalFunctionsUpdateTask,
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
};
*/