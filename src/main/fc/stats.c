
#include "platform.h"

#ifdef USE_STATS

#include "fc/settings.h"
#include "fc/stats.h"

#include "sensors/battery.h"

#include "drivers/time.h"
#include "navigation/navigation.h"

#include "fc/config.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#define MIN_FLIGHT_TIME_TO_RECORD_STATS_S 10    //prevent recording stats for that short "flights" [s]


PG_REGISTER_WITH_RESET_TEMPLATE(statsConfig_t, statsConfig, PG_STATS_CONFIG, 1);

PG_RESET_TEMPLATE(statsConfig_t, statsConfig,
    .stats_enabled = SETTING_STATS_DEFAULT,
    .stats_total_time = SETTING_STATS_TOTAL_TIME_DEFAULT,
    .stats_total_dist = SETTING_STATS_TOTAL_DIST_DEFAULT,
#ifdef USE_ADC
    .stats_total_energy = SETTING_STATS_TOTAL_ENERGY_DEFAULT
#endif
);

static uint32_t arm_millis;
static uint32_t arm_distance_cm;

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
            writeEEPROM();
        }
    }
}

#endif
