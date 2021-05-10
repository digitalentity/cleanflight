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

#include <platform.h>

#include "common/axis.h"

#include "drivers/pwm_esc_detect.h"
#include "drivers/pwm_mapping.h"
#include "drivers/pwm_output.h"

#include "fc/controlrate_profile.h"
#include "fc/rc_controls.h"

#include "flight/failsafe.h"
#include "flight/mixer.h"
#include "flight/pid.h"

#include "io/serial.h"

#include "rx/rx.h"

#include "sensors/battery.h"

#ifdef BRUSHED_MOTORS_PWM_RATE
#undef BRUSHED_MOTORS_PWM_RATE
#endif

#define BRUSHED_MOTORS_PWM_RATE 32000           // 32kHz

// alternative defaults settings for BlueJayF4 targets
void targetConfiguration(void)
{
    // alternative defaults settings for ALIENFLIGHTF1 and ALIENFLIGHTF3 targets
    serialConfigMutable()->portConfigs[2].functionMask = FUNCTION_RX_SERIAL;
    batteryMetersConfigMutable()->voltage.scale = 200;
    rxConfigMutable()->spektrum_sat_bind = 5;
    rxConfigMutable()->spektrum_sat_bind_autoreset = 1;

    if (hardwareMotorType == MOTOR_BRUSHED) {
        motorConfigMutable()->motorPwmProtocol = PWM_TYPE_BRUSHED;
        motorConfigMutable()->motorPwmRate = BRUSHED_MOTORS_PWM_RATE;
    }

    pidProfileMutable()->bank_mc.pid[ROLL].P = 36;
    pidProfileMutable()->bank_mc.pid[PITCH].P = 36;
    failsafeConfigMutable()->failsafe_delay = 2;
    failsafeConfigMutable()->failsafe_off_delay = 0;
    parseRcChannels("TAER1234");

    *primaryMotorMixerMutable(0) = (motorMixer_t){ 1.0f, -0.414178f,  1.0f, -1.0f };    // REAR_R
    *primaryMotorMixerMutable(1) = (motorMixer_t){ 1.0f, -0.414178f, -1.0f,  1.0f };    // FRONT_R
    *primaryMotorMixerMutable(2) = (motorMixer_t){ 1.0f,  0.414178f,  1.0f,  1.0f };    // REAR_L
    *primaryMotorMixerMutable(3) = (motorMixer_t){ 1.0f,  0.414178f, -1.0f, -1.0f };    // FRONT_L
    *primaryMotorMixerMutable(4) = (motorMixer_t){ 1.0f, -1.0f, -0.414178f, -1.0f };    // MIDFRONT_R
    *primaryMotorMixerMutable(5) = (motorMixer_t){ 1.0f,  1.0f, -0.414178f,  1.0f };    // MIDFRONT_L
    *primaryMotorMixerMutable(6) = (motorMixer_t){ 1.0f, -1.0f,  0.414178f,  1.0f };    // MIDREAR_R
    *primaryMotorMixerMutable(7) = (motorMixer_t){ 1.0f,  1.0f,  0.414178f, -1.0f };    // MIDREAR_L
}
