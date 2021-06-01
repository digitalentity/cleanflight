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
#include <string.h>

#include "platform.h"

#if defined(USE_TELEMETRY) && defined(USE_TELEMETRY_MICRO_ROS)

#include "build/build_config.h"
#include "build/debug.h"
#include "common/log.h"
#include "common/utils.h"

#include "config/feature.h"

#include "drivers/serial.h"
#include "drivers/time.h"

#include "telemetry/micro_ros.h"
#include "telemetry/telemetry.h"

#include <rmw_microros/rmw_microros.h>
#include <uxr/client/transport.h>

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#include <rosidl_runtime_c/primitives_sequence_functions.h>
#include <rosidl_runtime_c/string_functions.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <builtin_interfaces/msg/time.h>



static serialPort_t* microRosPort = NULL;
static serialPortConfig_t* portConfig;

static bool microRosTelemetryEnabled = false;
static portSharing_e microRosPortSharing;

#define RCCHECK(fn)                    \
    {                                  \
        rcl_ret_t temp_rc = fn;        \
        if ((temp_rc != RCL_RET_OK)) { \
            return false;              \
        }                              \
    }

#define RCSOFTCHECK(fn)                  \
    {                                    \
        rcl_ret_t temp_rc = fn;          \
        if ((temp_rc != RCL_RET_OK)) { } \
    }

static microRosState micro_ros_state = MICRO_ROS_STATE_CONNECT;

// adjust to the sum of subscriptions, timers, services, clients and guard conditions.
// do not include the number of nodes and publishers.
static unsigned int handler_count = 1;

static rcl_timer_t timer_timesync;

static rclc_executor_t executor;
static rclc_support_t support;
static rcl_allocator_t allocator;
static rcl_node_options_t node_ops;
static rcl_node_t node;
static timeMs_t last_timesync_ms = 0;
static timeMs_t last_creation_ms = 0;

// publisher
static rcl_publisher_t publisher_talker;
static rcl_timer_t timer_talker;
static std_msgs__msg__String msg_talker;

#define CONNECTION_ATTEMPT_MS 1000 // too small intervals may cause connection issues
#define MICRO_ROS_TIMESYNC_TIMEOUT_MS 5000
#define MICRO_ROS_TIMESYNC_COUNT 5
#define MICRO_ROS_NAMESPACE ""
#define MICRO_ROS_NODE_NAME "inav_node"

// override clock_gettime syscall
int clock_gettime(clockid_t unused, struct timespec* tp)
{
    (void)unused;
    tp->tv_sec = millis() / MILLISECS_PER_SEC;
    tp->tv_nsec = (micros() % USECS_PER_SEC) * MILLISECS_PER_SEC;
    return 0;
}

/* static void set_stamp(builtin_interfaces__msg__Time* stamp) */
/* { */
/*     int64_t nanos = rmw_uros_epoch_nanos(); */
/*     stamp->nanosec = nanos % (USECS_PER_SEC * 1000); */
/*     stamp->sec = RCL_NS_TO_S(nanos); */
/* } */

static bool transportOpen(struct uxrCustomTransport* transport)
{
    (void)transport;

    if (microRosPort == NULL) {
        return false;
    }

    return true;
}

static bool transportClose(struct uxrCustomTransport* transport)
{
    (void)transport;

    return true;
}

static size_t transportWrite(struct uxrCustomTransport* transport, const uint8_t* buf, size_t len, uint8_t* errcode)
{
    (void)transport;
    (void)errcode;

    // block on initial connection only, its required to use best effort streams during creation
    if (micro_ros_state == MICRO_ROS_STATE_RUN && !isSerialTransmitBufferEmpty(microRosPort) && (serialTxBytesFree(microRosPort) < len)) {
        // We are allowed to send out the response if
        //  a) TX buffer is completely empty
        //  b) Response fits into TX buffer
        // sending to non empty buffer will cause the fc to hang until all buffer is transmitted

        LOG_W(MICRO_ROS, "micro_ros: throwing serial buffer size %d to avoid blocking", len);
        *errcode = 1;
        return 0;
    }

    serialWriteBuf(microRosPort, buf, (int)len);
    *errcode = 0;
    return len;
}

static size_t transportRead(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* errcode)
{
    (void)errcode;
    (void)transport;

    timeMs_t start_time_ms = millis();
    size_t readed = 0;

    do {
        while ((readed < len) && serialRxBytesWaiting(microRosPort)) {
            buf[readed++] = serialRead(microRosPort);
        }
    } while ((readed < len) && (((millis()) - start_time_ms) < (timeMs_t)timeout));

    return readed;
}

void initMicroRosTelemetry(void)
{
    portConfig = findSerialPortConfig(FUNCTION_TELEMETRY_MICRO_ROS);
    microRosPortSharing = determinePortSharing(portConfig, FUNCTION_TELEMETRY_MICRO_ROS);
}

static void freeMicroRosTelemetryPort(void)
{
    closeSerialPort(microRosPort);
    microRosPort = NULL;
    microRosTelemetryEnabled = false;
}

static void configureMicroRosTelemetryPort(void)
{
    if (!portConfig) {
        return;
    }

    baudRate_e baudRateIndex = portConfig->telemetry_baudrateIndex;
    if (baudRateIndex == BAUD_AUTO) {
        baudRateIndex = BAUD_115200;
    }

    microRosPort = openSerialPort(portConfig->identifier, FUNCTION_TELEMETRY_MICRO_ROS, NULL, NULL, baudRates[baudRateIndex], MODE_RXTX, SERIAL_NOT_INVERTED);

    if (!microRosPort) {
        return;
    }

    rmw_uros_set_custom_transport(
        true,
        NULL,
        transportOpen,
        transportClose,
        transportWrite,
        transportRead);

    memset(&msg_talker, 0, sizeof(std_msgs__msg__String));
    rosidl_runtime_c__String__assign(&msg_talker.data, "miau");
    handler_count++;

    microRosTelemetryEnabled = true;
}

void checkMicroRosTelemetryState(void)
{
    bool newTelemetryEnabledValue = telemetryDetermineEnabledState(microRosPortSharing);

    if (newTelemetryEnabledValue == microRosTelemetryEnabled) {
        return;
    }

    if (newTelemetryEnabledValue) {
        configureMicroRosTelemetryPort();
    } else {
        freeMicroRosTelemetryPort();
    }
}

static void callbackTimesync(rcl_timer_t* timer, int64_t last_call_time)
{
    (void)last_call_time;
    if (timer != NULL) {
        rmw_uros_sync_session(0);
    }
}

static void publishTalkerCallback(rcl_timer_t* timer, int64_t last_call_time)
{
    (void)last_call_time;

    if (timer != NULL) {
        strcpy(msg_talker.data.data, "biau");

        RCSOFTCHECK(rcl_publish(&publisher_talker, &msg_talker, NULL));
    }
}

static rcl_ret_t createPublisher(
    rclc_support_t* support,
    rclc_executor_t* executor,
    rcl_node_t* node,
    rcl_timer_t* timer,
    rcl_publisher_t* publisher,
    const rosidl_message_type_support_t* type_support,
    const uint16_t frequency,
    const char* topic_name,
    const rcl_timer_callback_t callback)
{
    rcl_ret_t ret = RCL_RET_OK;

    // do nothing for 0 frequency
    if (frequency == 0) {
        return ret;
    }

    ret = rclc_timer_init_default(
        timer,
        support,
        RCL_MS_TO_NS(MILLISECS_PER_SEC / frequency),
        callback);

    if (RCL_RET_OK != ret) {
        return ret;
    }

    ret = rclc_publisher_init_best_effort(
        publisher,
        node,
        type_support,
        topic_name);

    if (RCL_RET_OK != ret) {
        return ret;
    }

    ret = rclc_executor_add_timer(executor, timer);

    if (RCL_RET_OK != ret) {
        return ret;
    }

    return RCL_RET_OK;
}

static bool createEntities(void)
{
    allocator = rcl_get_default_allocator();

    //create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    RCSOFTCHECK(rmw_uros_sync_session(0));

    node_ops = rcl_node_get_default_options();
    node_ops.domain_id = 0;
    // create node
    RCCHECK(rclc_node_init_with_options(&node, MICRO_ROS_NODE_NAME, MICRO_ROS_NAMESPACE, &support, &node_ops));

    // create executor
    executor = rclc_executor_get_zero_initialized_executor();

    RCCHECK(rclc_executor_init(&executor, &support.context, handler_count, &allocator));

    // timesync
    RCCHECK(rclc_timer_init_default(
                &timer_timesync,
                &support,
                RCL_MS_TO_NS(MICRO_ROS_TIMESYNC_TIMEOUT_MS / MICRO_ROS_TIMESYNC_COUNT),
                callbackTimesync));
    RCCHECK(rclc_executor_add_timer(&executor, &timer_timesync));

    RCCHECK(createPublisher(
                &support,
                &executor,
                &node,
                &timer_talker,
                &publisher_talker,
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
                10, // hz
                "talker",
                publishTalkerCallback));

    return true;
}

static void destroyEntities()
{
    RCSOFTCHECK(rcl_timer_fini(&timer_timesync));

    RCSOFTCHECK(rcl_publisher_fini(&publisher_talker, &node));
    RCSOFTCHECK(rcl_timer_fini(&timer_talker));

    RCSOFTCHECK(rcl_node_options_fini(&node_ops));
    RCSOFTCHECK(rcl_node_fini(&node));
    RCSOFTCHECK(rclc_executor_fini(&executor));
    rclc_support_fini(&support);
}

void handleMicroRosTelemetry(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    if (!microRosTelemetryEnabled) {
        return;
    }

    switch(micro_ros_state) {
    case MICRO_ROS_STATE_CONNECT:
        if (last_creation_ms + CONNECTION_ATTEMPT_MS > millis()) {
            break;
        }

        last_creation_ms = millis();
        if (createEntities()) {
            // successfully connected
            micro_ros_state = MICRO_ROS_STATE_RUN;
            last_timesync_ms = millis();
        } else {
            micro_ros_state = MICRO_ROS_STATE_FINIALIZE;
        }
        break;
    case MICRO_ROS_STATE_RUN:
        if (rmw_uros_epoch_synchronized()) {
            last_timesync_ms = millis();
        }

        if (last_timesync_ms + MICRO_ROS_TIMESYNC_TIMEOUT_MS < millis()) {
            micro_ros_state = MICRO_ROS_STATE_FINIALIZE;
            break;
        }

        // spin is needed only for timer, subscribers and services
        // the timeout should be relative to the fc_tasks loop, in our case its 2ms for 500hz
        // timeout bellow 1ms will cause the handlers not to be called
        rcl_ret_t ret = rclc_executor_spin_some(&executor, 0);
        if (RCL_RET_OK != ret) {
            LOG_E(MICRO_ROS, "micro_ros: ret code is %ld", ret);
            micro_ros_state = MICRO_ROS_STATE_FINIALIZE;
        }
        break;
    case MICRO_ROS_STATE_FINIALIZE:
        destroyEntities();
        micro_ros_state = MICRO_ROS_STATE_CONNECT;
        break;
    }
}

#endif
