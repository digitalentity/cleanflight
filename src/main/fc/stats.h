
#pragma once
#ifdef USE_STATS

typedef struct statsConfig_s {
    uint32_t stats_total_time; // [s]
    uint32_t stats_total_dist; // [m]
#ifdef USE_ADC
    uint32_t stats_total_energy; // deciWatt hour (x0.1Wh)
#endif
    uint8_t  stats_enabled;
} statsConfig_t;

uint32_t getFlyingEnergy(void);
void statsOnArm(void);
void statsOnDisarm(void);
void statsOnLoop(void);

#else

#define statsOnArm()    do {} while (0)
#define statsOnDisarm() do {} while (0)
#define statsOnLoop()   do {} while (0)

#endif
