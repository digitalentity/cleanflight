#include "stdbool.h"
#include "stdint.h"
#include "stdlib.h"

#include "platform.h"

#include "common/utils.h"

#include "drivers/time.h"
#include "drivers/lights_io.h"

#include "rx/rx.h"

#include "fc/config.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "scheduler/scheduler.h"

#include "config/feature.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "io/lights.h"

#ifdef USE_LIGHTS

PG_REGISTER_WITH_RESET_TEMPLATE(lightsConfig_t, lightsConfig, PG_LIGHTS_CONFIG, 0);

PG_RESET_TEMPLATE(lightsConfig_t, lightsConfig,
        .failsafe = {
            .enabled = SETTING_FAILSAFE_LIGHTS_DEFAULT,
            .flash_period = SETTING_FAILSAFE_LIGHTS_FLASH_PERIOD_DEFAULT,
            .flash_on_time = SETTING_FAILSAFE_LIGHTS_FLASH_ON_TIME_DEFAULT
        }
);

static bool lights_on = false;
static timeUs_t last_status_change = 0;

static void lightsSetStatus(bool status, timeUs_t currentTimeUs)
{
    if (status != lights_on) {
        lights_on = status;
        lightsHardwareSetStatus(status);
        last_status_change = currentTimeUs;
    }
}

/*
 * Lights handler function to be called periodically in loop. Updates lights
 * state via time schedule.
 */
void lightsUpdate(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
    if (lightsConfig()->failsafe.enabled && FLIGHT_MODE(FAILSAFE_MODE) && ARMING_FLAG(WAS_EVER_ARMED)) {
        if (lightsConfig()->failsafe.flash_period <= lightsConfig()->failsafe.flash_on_time) {
            lightsSetStatus(true, currentTimeUs);
        } else {
            if (lights_on) {
                if (currentTimeUs - last_status_change > lightsConfig()->failsafe.flash_on_time * 1000)
                    lightsSetStatus(false, currentTimeUs);
            } else {
                if (currentTimeUs - last_status_change > (uint32_t)(lightsConfig()->failsafe.flash_period - lightsConfig()->failsafe.flash_on_time) * 1000)
                    lightsSetStatus(true, currentTimeUs);
            }
        }
    } else
        lightsSetStatus(IS_RC_MODE_ACTIVE(BOXLIGHTS), currentTimeUs);
}

void lightsInit(void)
{
    lightsHardwareInit();
}

#endif /* USE_LIGHTS */
