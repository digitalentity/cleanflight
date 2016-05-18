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

#include <math.h>

#include "platform.h"

#include "build_config.h"

#include "common/axis.h"
#include "common/maths.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "config/config.h"
#include "config/runtime_config.h"
#include "config/feature.h"

#include "drivers/system.h"
#include "drivers/sensor.h"
#include "drivers/accgyro.h"

#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/sensors.h"
#include "sensors/gyro.h"
#include "sensors/acceleration.h"

#include "rx/rx.h"

#include "io/gps.h"
#include "io/beeper.h"
#include "io/rc_curves.h"
#include "io/rate_profile.h"

#include "io/display.h"

#include "flight/pid.h"
#include "flight/navigation_rewrite.h"
#include "flight/failsafe.h"

#include "mw.h"

#define AIRMODE_DEADBAND 25

PG_REGISTER(armingConfig_t, armingConfig, PG_ARMING_CONFIG, 0);

PG_REGISTER_PROFILE(rcControlsConfig_t, rcControlsConfig, PG_RC_CONTROLS_CONFIG, 0);
PG_REGISTER_PROFILE(modeActivationProfile_t, modeActivationProfile, PG_MODE_ACTIVATION_PROFILE, 0);

// true if arming is done via the sticks (as opposed to a switch)
static bool isUsingSticksToArm = true;

#ifdef NAV
// true if pilot has any of GPS modes configured
static bool isUsingNAVModes = false;
#endif

int16_t rcCommand[4];           // interval [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW

uint32_t rcModeActivationMask; // one bit per mode defined in boxId_e

bool isUsingSticksForArming(void)
{
    return isUsingSticksToArm;
}

#if defined(NAV)
bool isUsingNavigationModes(void)
{
    return isUsingNAVModes;
}
#endif

bool areSticksInApModePosition(uint16_t ap_mode)
{
    return ABS(rcCommand[ROLL]) < ap_mode && ABS(rcCommand[PITCH]) < ap_mode;
}

throttleStatus_e calculateThrottleStatus(rxConfig_t *rxConfig, uint16_t deadband3d_throttle)
{
    if (feature(FEATURE_3D) && (rcData[THROTTLE] > (rxConfig->midrc - deadband3d_throttle) && rcData[THROTTLE] < (rxConfig->midrc + deadband3d_throttle)))
        return THROTTLE_LOW;
    else if (!feature(FEATURE_3D) && (rcData[THROTTLE] < rxConfig->mincheck))
        return THROTTLE_LOW;

    return THROTTLE_HIGH;
}

rollPitchStatus_e calculateRollPitchCenterStatus(rxConfig_t *rxConfig)
{
    if (((rcData[PITCH] < (rxConfig->midrc + AIRMODE_DEADBAND)) && (rcData[PITCH] > (rxConfig->midrc -AIRMODE_DEADBAND)))
            && ((rcData[ROLL] < (rxConfig->midrc + AIRMODE_DEADBAND)) && (rcData[ROLL] > (rxConfig->midrc -AIRMODE_DEADBAND))))
        return CENTERED;

    return NOT_CENTERED;
}

void processRcStickPositions(rxConfig_t *rxConfig, throttleStatus_e throttleStatus, bool disarm_kill_switch)
{
    static uint8_t rcDelayCommand;      // this indicates the number of time (multiple of RC measurement at 50Hz) the sticks must be maintained to run or switch off motors
    static uint8_t rcSticks;            // this hold sticks position for command combos
    uint8_t stTmp = 0;
    int i;

    // ------------------ STICKS COMMAND HANDLER --------------------
    // checking sticks positions
    for (i = 0; i < 4; i++) {
        stTmp >>= 2;
        if (rcData[i] > rxConfig->mincheck)
            stTmp |= 0x80;  // check for MIN
        if (rcData[i] < rxConfig->maxcheck)
            stTmp |= 0x40;  // check for MAX
    }
    if (stTmp == rcSticks) {
        if (rcDelayCommand < 250)
            rcDelayCommand++;
    } else
        rcDelayCommand = 0;
    rcSticks = stTmp;

    // perform actions
    if (!isUsingSticksToArm) {

        if (IS_RC_MODE_ACTIVE(BOXARM)) {
            // Arming via ARM BOX
            if (throttleStatus == THROTTLE_LOW) {
                if (ARMING_FLAG(OK_TO_ARM)) {
                    mwArm();
                }
            }
        } else {
            // Disarming via ARM BOX

            if (ARMING_FLAG(ARMED) && rxIsReceivingSignal() && !failsafeIsActive()  ) {
                if (disarm_kill_switch) {
                    mwDisarm();
                } else if (throttleStatus == THROTTLE_LOW) {
                    mwDisarm();
                }
            }
        }
    }

    if (rcDelayCommand != 20) {
        return;
    }

    if (isUsingSticksToArm) {
        // Disarm on throttle down + yaw
        if (rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_CE) {
            if (ARMING_FLAG(ARMED))
                mwDisarm();
            else {
                beeper(BEEPER_DISARM_REPEAT);    // sound tone while stick held
                rcDelayCommand = 0;              // reset so disarm tone will repeat
            }
        }
    }

    if (ARMING_FLAG(ARMED)) {
        // actions during armed
        return;
    }

    // actions during not armed
    i = 0;

    if (rcSticks == THR_LO + YAW_LO + PIT_LO + ROL_CE) {
        // GYRO calibration
        gyroSetCalibrationCycles(CALIBRATING_GYRO_CYCLES);
        return;
    }

    // Multiple configuration profiles
    if (rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_LO)          // ROLL left  -> Profile 1
        i = 1;
    else if (rcSticks == THR_LO + YAW_LO + PIT_HI + ROL_CE)     // PITCH up   -> Profile 2
        i = 2;
    else if (rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_HI)     // ROLL right -> Profile 3
        i = 3;
    if (i) {
        changeProfile(i - 1);
        beeperConfirmationBeeps(i);
        return;
    }

    if (rcSticks == THR_LO + YAW_LO + PIT_LO + ROL_HI) {
        saveConfigAndNotify();
    }

    if (isUsingSticksToArm) {

        if (rcSticks == THR_LO + YAW_HI + PIT_CE + ROL_CE) {
            // Arm via YAW
            mwArm();
            return;
        }
    }

    if (rcSticks == THR_HI + YAW_LO + PIT_LO + ROL_CE) {
        // Calibrating Acc
        accSetCalibrationCycles(CALIBRATING_ACC_CYCLES);
        return;
    }


    if (rcSticks == THR_HI + YAW_HI + PIT_LO + ROL_CE) {
        // Calibrating Mag
        ENABLE_STATE(CALIBRATE_MAG);
        return;
    }


    // Accelerometer Trim
    if (rcSticks == THR_HI + YAW_CE + PIT_HI + ROL_CE) {
        applyAndSaveBoardAlignmentDelta(0, -2);
        rcDelayCommand = 10;
        return;
    } else if (rcSticks == THR_HI + YAW_CE + PIT_LO + ROL_CE) {
        applyAndSaveBoardAlignmentDelta(0, 2);
        rcDelayCommand = 10;
        return;
    } else if (rcSticks == THR_HI + YAW_CE + PIT_CE + ROL_HI) {
        applyAndSaveBoardAlignmentDelta(-2, 0);
        rcDelayCommand = 10;
        return;
    } else if (rcSticks == THR_HI + YAW_CE + PIT_CE + ROL_LO) {
        applyAndSaveBoardAlignmentDelta(2, 0);
        rcDelayCommand = 10;
        return;
    }
}

bool isModeActivationConditionPresent(modeActivationCondition_t *modeActivationConditions, boxId_e modeId)
{
    uint8_t index;

    for (index = 0; index < MAX_MODE_ACTIVATION_CONDITION_COUNT; index++) {
        modeActivationCondition_t *modeActivationCondition = &modeActivationConditions[index];
        
        if (modeActivationCondition->modeId == modeId && IS_RANGE_USABLE(&modeActivationCondition->range)) {
            return true;
        }
    }
    
    return false;
}

bool isRangeActive(uint8_t auxChannelIndex, channelRange_t *range)
{
    if (!IS_RANGE_USABLE(range)) {
        return false;
    }

    uint16_t channelValue = constrain(rcData[auxChannelIndex + NON_AUX_CHANNEL_COUNT], CHANNEL_RANGE_MIN, CHANNEL_RANGE_MAX - 1);
    return (channelValue >= 900 + (range->startStep * 25) &&
            channelValue < 900 + (range->endStep * 25));
}

void updateActivatedModes(modeActivationCondition_t *modeActivationConditions)
{
    rcModeActivationMask = 0;

    uint8_t index;

    for (index = 0; index < MAX_MODE_ACTIVATION_CONDITION_COUNT; index++) {
        modeActivationCondition_t *modeActivationCondition = &modeActivationConditions[index];

        if (isRangeActive(modeActivationCondition->auxChannelIndex, &modeActivationCondition->range)) {
            ACTIVATE_RC_MODE(modeActivationCondition->modeId);
        }
    }
}

int32_t getRcStickDeflection(int32_t axis, uint16_t midrc) {
    return MIN(ABS(rcData[axis] - midrc), 500);
}

void useRcControlsConfig(modeActivationCondition_t *modeActivationConditions)
{
    isUsingSticksToArm = !isModeActivationConditionPresent(modeActivationConditions, BOXARM);

#ifdef NAV
    isUsingNAVModes = isModeActivationConditionPresent(modeActivationConditions, BOXNAVPOSHOLD) ||
                        isModeActivationConditionPresent(modeActivationConditions, BOXNAVRTH) ||
                        isModeActivationConditionPresent(modeActivationConditions, BOXNAVWP);
#endif
}

