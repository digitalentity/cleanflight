#include <stdbool.h>
#include <stdint.h>
#include "drivers/io_pca9685.h"

#include "config/config.h"
#include "config/runtime_config.h"

#define PWM_DRIVER_IMPLEMENTATION_COUNT 1

static bool driverEnabled = false;
static uint8_t driverImplementationIndex = 0;

typedef struct {
    bool (*initFunction)(void);
    void (*writeFunction)(uint8_t servoIndex, uint16_t off);
    void (*setFrequencyFunction)(float freq);
    void (*syncFunction)(void);
} pwmDriverDriver_t;

pwmDriverDriver_t pwmDrivers[PWM_DRIVER_IMPLEMENTATION_COUNT] = {
    [0] = {
        .initFunction = pca9685Initialize,
        .writeFunction = pca9685setServoPulse,
        .setFrequencyFunction = pca9685setPWMFreq,
        .syncFunction = pca9685sync
    }
};

bool isPwmDriverEnabled() {
    return driverEnabled;
}

void pwmDriverSetPulse(uint8_t servoIndex, uint16_t length) {
    (pwmDrivers[driverImplementationIndex].writeFunction)(servoIndex, length);
}

void pwmDriverInitialize(void) {
    driverEnabled = (pwmDrivers[driverImplementationIndex].initFunction)();

    if (!driverEnabled) {
        featureClear(FEATURE_PWM_SERVO_DRIVER);
    }

}

void pwmDriverSync(void) {
    (pwmDrivers[driverImplementationIndex].syncFunction)();
}
