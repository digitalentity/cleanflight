/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "platform.h"

#ifdef USE_CAMERA_CONTROL

#include "camera_control.h"
#include "io.h"
#include "math.h"
#include "nvic.h"
#include "pwm_output.h"
#include "time.h"
#include "config/parameter_group_ids.h"
#include "pwm_mapping.h"


#if defined(STM32F40_41xxx)
#define CAMERA_CONTROL_TIMER_HZ   MHZ_TO_HZ(84)
#elif defined(STM32F7)
#define CAMERA_CONTROL_TIMER_HZ   MHZ_TO_HZ(216)
#else
#define CAMERA_CONTROL_TIMER_HZ   MHZ_TO_HZ(72)
#endif

#define CAMERA_CONTROL_PWM_RESOLUTION   128
#define CAMERA_CONTROL_SOFT_PWM_RESOLUTION 448

#ifdef CURRENT_TARGET_CPU_VOLTAGE
#define ADC_VOLTAGE CURRENT_TARGET_CPU_VOLTAGE
#else
#define ADC_VOLTAGE 3.3f
#endif

#if !defined(STM32F411xE) && !defined(STM32F7) && !defined(STM32H7)
#define CAMERA_CONTROL_SOFTWARE_PWM_AVAILABLE
#include "build/atomic.h"
#endif

#define CAMERA_CONTROL_HARDWARE_PWM_AVAILABLE
#include "timer.h"

#ifdef USE_OSD
#include "osd.h"
#endif

PG_REGISTER_WITH_RESET_FN(cameraControlConfig_t, cameraControlConfig, PG_CAMERA_CONTROL_CONFIG, 0);


void pgResetFn_cameraControlConfig(cameraControlConfig_t *cameraControlConfig)
{
    cameraControlConfig->mode = CAMERA_CONTROL_MODE_HARDWARE_PWM;
    cameraControlConfig->refVoltage = 330;
    cameraControlConfig->keyDelayMs = 180;
    cameraControlConfig->internalResistance = 470;
    cameraControlConfig->inverted = 0;   // Output is inverted externally
}

static struct {
    bool enabled;
    IO_t io;
    volatile timCCR_t *ccr;         // Shortcut for timer CCR register
    TCH_t *channel;
    uint32_t period;
    uint8_t inverted;
} cameraControlRuntime;

static uint32_t endTimeMillis;

void cameraControlInit(void)
{
    cameraControlRuntime.inverted = cameraControlConfig()->inverted;
    cameraControlRuntime.io = IOGetByTag(IO_TAG(CAMERA_CONTROL_PIN));
    IOInit(cameraControlRuntime.io, OWNER_CAMERA_CONTROL, RESOURCE_OUTPUT, 0);

    const timerHardware_t *timerHardware = timerGetByTag(IO_TAG(CAMERA_CONTROL_PIN), TIM_USE_ANY);
    
    if (!timerHardware) {
       	return;
    }

    IOConfigGPIOAF(cameraControlRuntime.io, IOCFG_AF_PP, timerHardware->alternateFunction);
        
    cameraControlRuntime.channel = timerGetTCH(timerHardware);

    timerConfigBase(cameraControlRuntime.channel, CAMERA_CONTROL_PWM_RESOLUTION, CAMERA_CONTROL_TIMER_HZ);
    timerPWMConfigChannel(cameraControlRuntime.channel, 0);
    timerPWMStart(cameraControlRuntime.channel);

    timerEnable(cameraControlRuntime.channel);

    cameraControlRuntime.ccr = timerCCR(cameraControlRuntime.channel);

    cameraControlRuntime.period = CAMERA_CONTROL_PWM_RESOLUTION;
    *cameraControlRuntime.ccr = cameraControlRuntime.period;
    cameraControlRuntime.enabled = true;
}

void cameraControlProcess(uint32_t currentTimeUs)
{
    if (endTimeMillis && currentTimeUs >= 1000 * endTimeMillis) {
        if (CAMERA_CONTROL_MODE_HARDWARE_PWM == cameraControlConfig()->mode) {
            *cameraControlRuntime.ccr = cameraControlRuntime.period;
        } else if (CAMERA_CONTROL_MODE_SOFTWARE_PWM == cameraControlConfig()->mode) {

        }

        endTimeMillis = 0;
    }
}

static const int buttonResistanceValues[] = { 45000, 27000, 15000, 6810, 0 };

static float calculateKeyPressVoltage(const cameraControlKey_e key)
{
    const int buttonResistance = buttonResistanceValues[key];
    return 1.0e-2f * cameraControlConfig()->refVoltage * buttonResistance / (100 * cameraControlConfig()->internalResistance + buttonResistance);
}

#if defined(CAMERA_CONTROL_HARDWARE_PWM_AVAILABLE) || defined(CAMERA_CONTROL_SOFTWARE_PWM_AVAILABLE)
static float calculatePWMDutyCycle(const cameraControlKey_e key)
{
    const float voltage = calculateKeyPressVoltage(key);

    return voltage / ADC_VOLTAGE;
}
#endif

void cameraControlKeyPress(cameraControlKey_e key, uint32_t holdDurationMs)
{
    if (!cameraControlRuntime.enabled)
        return;

    if (key >= CAMERA_CONTROL_KEYS_COUNT)
        return;

#if defined(CAMERA_CONTROL_HARDWARE_PWM_AVAILABLE) || defined(CAMERA_CONTROL_SOFTWARE_PWM_AVAILABLE)
    const float dutyCycle = calculatePWMDutyCycle(key);
#else
    (void) holdDurationMs;
#endif

#ifdef USE_OSD
    // Force OSD timeout so we are alone on the display.
//    resumeRefreshAt = 0;
#endif

    if (CAMERA_CONTROL_MODE_HARDWARE_PWM == cameraControlConfig()->mode) {
#ifdef CAMERA_CONTROL_HARDWARE_PWM_AVAILABLE
        *cameraControlRuntime.ccr = lrintf(dutyCycle * cameraControlRuntime.period);
        endTimeMillis = millis() + cameraControlConfig()->keyDelayMs + holdDurationMs;
#endif
    } else if (CAMERA_CONTROL_MODE_SOFTWARE_PWM == cameraControlConfig()->mode) {
    } else if (CAMERA_CONTROL_MODE_DAC == cameraControlConfig()->mode) {
        // @todo not yet implemented
    }
}

#endif
