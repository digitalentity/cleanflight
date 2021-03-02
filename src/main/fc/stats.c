
#include "platform.h"

#ifdef USE_STATS

#include "fc/stats.h"

#include "sensors/battery.h"

#include "drivers/time.h"
#include "navigation/navigation.h"

#include "fc/config.h"
#include "fc/runtime_config.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "io/beeper.h"


#define MIN_FLIGHT_TIME_TO_RECORD_STATS_S 10    //prevent recording stats for that short "flights" [s]
#define STATS_SAVE_DELAY_MS              500    //let disarming complete and save stats after this time


PG_REGISTER_WITH_RESET_TEMPLATE(statsConfig_t, statsConfig, PG_STATS_CONFIG, 1);

PG_RESET_TEMPLATE(statsConfig_t, statsConfig,
    .stats_enabled = 0,
    .stats_total_time = 0,
    .stats_total_dist = 0,
#ifdef USE_ADC
    .stats_total_energy = 0
#endif
);

static uint32_t arm_millis;
static uint32_t arm_distance_cm;
static uint32_t save_pending_millis;  // 0 = none

#ifdef USE_ADC
static uint32_t arm_mWhDrawn;
static uint32_t flyingEnergy; // energy drawn during flying up to last disarm (ARMED) mWh

uint32_t getFlyingEnergy() {
    return flyingEnergy;
}
#endif

void statsOnArm(void)
{
    arm_millis      = millis();
    arm_distance_cm = getTotalTravelDistance();
#ifdef USE_ADC
    arm_mWhDrawn    = getMWhDrawn();
#endif
}

void statsOnDisarm(void)
{
    if (statsConfig()->stats_enabled) {
        uint32_t dt = (millis() - arm_millis) / 1000;
        if (dt >= MIN_FLIGHT_TIME_TO_RECORD_STATS_S) {
            statsConfigMutable()->stats_total_time += dt;   //[s]
            statsConfigMutable()->stats_total_dist += (getTotalTravelDistance() - arm_distance_cm) / 100;   //[m]
#ifdef USE_ADC
            if (feature(FEATURE_VBAT) && isAmperageConfigured()) {
                const uint32_t energy = getMWhDrawn() - arm_mWhDrawn;
                statsConfigMutable()->stats_total_energy += energy;
                flyingEnergy += energy;
            }
#endif
            /* signal that stats need to be saved but don't execute time consuming flash operation
               now - let the disarming process complete and then execute the actual save */
            save_pending_millis = millis();
        }
    }
}

void statsOnLoop(void)
{
    /* check for pending flash write */
    if (save_pending_millis && millis()-save_pending_millis > STATS_SAVE_DELAY_MS) {
        if (ARMING_FLAG(ARMED)) {
            /* re-armed - don't save! */
        }
        else {
            if (isConfigDirty()) {
                /* There are some adjustments made in the configuration and we don't want
                   to implicitly save it... We can't currently save part of the configuration,
                   so we simply don't execute the stats save operation at all.
                   This will result in lost stats *if* rc-adjustments were used during the flight */
            }
            else {
                writeEEPROM();
                /* repeat disarming beep indicating the stats save is complete */
                beeper(BEEPER_DISARMING);
            }
        }
        save_pending_millis = 0;
    }
}

#endif
