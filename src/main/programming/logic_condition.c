/*
 * This file is part of INAV Project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Alternatively, the contents of this file may be used under the terms
 * of the GNU General Public License Version 3, as described below:
 *
 * This file is free software: you may copy, redistribute and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/.
 */

#include <stdbool.h>

#include "config/config_reset.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "programming/logic_condition.h"
#include "programming/global_variables.h"
#include "programming/pid.h"
#include "common/utils.h"
#include "rx/rx.h"
#include "common/maths.h"
#include "fc/fc_core.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"
#include "fc/rc_modes.h"
#include "navigation/navigation.h"
#include "sensors/battery.h"
#include "sensors/pitotmeter.h"
#include "flight/imu.h"
#include "flight/pid.h"
#include "drivers/io_port_expander.h"
#include "io/osd_common.h"
#include "sensors/diagnostics.h"

#include "navigation/navigation.h"
#include "navigation/navigation_private.h"

#include "io/vtx.h"
#include "drivers/vtx_common.h"

PG_REGISTER_ARRAY_WITH_RESET_FN(logicCondition_t, MAX_LOGIC_CONDITIONS, logicConditions, PG_LOGIC_CONDITIONS, 2);

EXTENDED_FASTRAM uint64_t logicConditionsGlobalFlags;
EXTENDED_FASTRAM int logicConditionValuesByType[LOGIC_CONDITION_LAST];
EXTENDED_FASTRAM rcChannelOverride_t rcChannelOverrides[MAX_SUPPORTED_RC_CHANNEL_COUNT];

void pgResetFn_logicConditions(logicCondition_t *instance)
{
    for (int i = 0; i < MAX_LOGIC_CONDITIONS; i++) {
        RESET_CONFIG(logicCondition_t, &instance[i],
            .enabled = 0,
            .activatorId = -1,
            .operation = 0,
            .operandA = {
                .type = LOGIC_CONDITION_OPERAND_TYPE_VALUE,
                .value = 0
            },
            .operandB = {
                .type = LOGIC_CONDITION_OPERAND_TYPE_VALUE,
                .value = 0
            },
            .flags = 0
        );
    }
}

logicConditionState_t logicConditionStates[MAX_LOGIC_CONDITIONS];

static int logicConditionCompute(
    int currentVaue,
    logicOperation_e operation,
    int operandA,
    int operandB
) {
    int temporaryValue;
    vtxDeviceCapability_t vtxDeviceCapability;

    switch (operation) {

        case LOGIC_CONDITION_TRUE:
            return true;
            break;

        case LOGIC_CONDITION_EQUAL:
            return operandA == operandB;
            break;

        case LOGIC_CONDITION_GREATER_THAN:
            return operandA > operandB;
            break;

        case LOGIC_CONDITION_LOWER_THAN:
            return operandA < operandB;
            break;

        case LOGIC_CONDITION_LOW:
            return operandA < 1333;
            break;

        case LOGIC_CONDITION_MID:
            return operandA >= 1333 && operandA <= 1666;
            break;

        case LOGIC_CONDITION_HIGH:
            return operandA > 1666;
            break;

        case LOGIC_CONDITION_AND:
            return (operandA && operandB);
            break;

        case LOGIC_CONDITION_OR:
            return (operandA || operandB);
            break;

        case LOGIC_CONDITION_XOR:
            return (operandA != operandB);
            break;

        case LOGIC_CONDITION_NAND:
            return !(operandA && operandB);
            break;

        case LOGIC_CONDITION_NOR:
            return !(operandA || operandB);
            break; 

        case LOGIC_CONDITION_NOT:
            return !operandA;
            break;

        case LOGIC_CONDITION_STICKY:
            // Operand A is activation operator
            if (operandA) {
                return true;
            }
            //Operand B is deactivation operator
            if (operandB) {
                return false;
            }

            //When both operands are not met, keep current value 
            return currentVaue;
            break;

        case LOGIC_CONDITION_GVAR_SET:
            gvSet(operandA, operandB);
            return operandB;
            break;
        
        case LOGIC_CONDITION_GVAR_INC:
            temporaryValue = gvGet(operandA) + operandB;
            gvSet(operandA, temporaryValue);
            return temporaryValue;
            break;

        case LOGIC_CONDITION_GVAR_DEC:
            temporaryValue = gvGet(operandA) - operandB;
            gvSet(operandA, temporaryValue);
            return temporaryValue;
            break;

        case LOGIC_CONDITION_ADD:
            return constrain(operandA + operandB, INT16_MIN, INT16_MAX);
            break;

        case LOGIC_CONDITION_SUB:
            return constrain(operandA - operandB, INT16_MIN, INT16_MAX);
            break;

        case LOGIC_CONDITION_MUL:
            return constrain(operandA * operandB, INT16_MIN, INT16_MAX);
            break;

        case LOGIC_CONDITION_DIV:
            if (operandB != 0) {
                return constrain(operandA / operandB, INT16_MIN, INT16_MAX);
            } else {
                return operandA;
            }
            break;
        
        case LOGIC_CONDITION_OVERRIDE_ARMING_SAFETY:
            LOGIC_CONDITION_GLOBAL_FLAG_ENABLE(LOGIC_CONDITION_GLOBAL_FLAG_OVERRIDE_ARMING_SAFETY);
            return true;
            break;

        case LOGIC_CONDITION_OVERRIDE_THROTTLE_SCALE:
            logicConditionValuesByType[LOGIC_CONDITION_OVERRIDE_THROTTLE_SCALE] = operandA;
            LOGIC_CONDITION_GLOBAL_FLAG_ENABLE(LOGIC_CONDITION_GLOBAL_FLAG_OVERRIDE_THROTTLE_SCALE);
            return true;
            break;

        case LOGIC_CONDITION_SWAP_ROLL_YAW:
            LOGIC_CONDITION_GLOBAL_FLAG_ENABLE(LOGIC_CONDITION_GLOBAL_FLAG_OVERRIDE_SWAP_ROLL_YAW);
            return true;
            break;

        case LOGIC_CONDITION_SET_VTX_POWER_LEVEL:
#if defined(USE_VTX_SMARTAUDIO) || defined(USE_VTX_TRAMP)
            if (
                logicConditionValuesByType[LOGIC_CONDITION_SET_VTX_POWER_LEVEL] != operandA && 
                vtxCommonGetDeviceCapability(vtxCommonDevice(), &vtxDeviceCapability)
            ) {
                logicConditionValuesByType[LOGIC_CONDITION_SET_VTX_POWER_LEVEL] = constrain(operandA, VTX_SETTINGS_MIN_POWER, vtxDeviceCapability.powerCount);
                vtxSettingsConfigMutable()->power = logicConditionValuesByType[LOGIC_CONDITION_SET_VTX_POWER_LEVEL];
                return logicConditionValuesByType[LOGIC_CONDITION_SET_VTX_POWER_LEVEL];
            } else {
                return false;
            }
            break;
#else
            return false;
#endif

        case LOGIC_CONDITION_SET_VTX_BAND:
            if (
                logicConditionValuesByType[LOGIC_CONDITION_SET_VTX_BAND] != operandA &&
                vtxCommonGetDeviceCapability(vtxCommonDevice(), &vtxDeviceCapability)
            ) {
                logicConditionValuesByType[LOGIC_CONDITION_SET_VTX_BAND] = constrain(operandA, VTX_SETTINGS_MIN_BAND, VTX_SETTINGS_MAX_BAND);
                vtxSettingsConfigMutable()->band = logicConditionValuesByType[LOGIC_CONDITION_SET_VTX_BAND];
                return logicConditionValuesByType[LOGIC_CONDITION_SET_VTX_BAND];
            } else {
                return false;
            }
            break;
        case LOGIC_CONDITION_SET_VTX_CHANNEL:
            if (
                logicConditionValuesByType[LOGIC_CONDITION_SET_VTX_CHANNEL] != operandA &&
                vtxCommonGetDeviceCapability(vtxCommonDevice(), &vtxDeviceCapability)
            ) {
                logicConditionValuesByType[LOGIC_CONDITION_SET_VTX_CHANNEL] = constrain(operandA, VTX_SETTINGS_MIN_CHANNEL, VTX_SETTINGS_MAX_CHANNEL);
                vtxSettingsConfigMutable()->channel = logicConditionValuesByType[LOGIC_CONDITION_SET_VTX_CHANNEL];
                return logicConditionValuesByType[LOGIC_CONDITION_SET_VTX_CHANNEL];
            } else {
                return false;
            }
            break;
        
        case LOGIC_CONDITION_INVERT_ROLL:
            LOGIC_CONDITION_GLOBAL_FLAG_ENABLE(LOGIC_CONDITION_GLOBAL_FLAG_OVERRIDE_INVERT_ROLL);
            return true;
            break;

        case LOGIC_CONDITION_INVERT_PITCH:
            LOGIC_CONDITION_GLOBAL_FLAG_ENABLE(LOGIC_CONDITION_GLOBAL_FLAG_OVERRIDE_INVERT_PITCH);
            return true;
            break;
        
        case LOGIC_CONDITION_INVERT_YAW:
            LOGIC_CONDITION_GLOBAL_FLAG_ENABLE(LOGIC_CONDITION_GLOBAL_FLAG_OVERRIDE_INVERT_YAW);
            return true;
            break;
        
        case LOGIC_CONDITION_OVERRIDE_THROTTLE:
            logicConditionValuesByType[LOGIC_CONDITION_OVERRIDE_THROTTLE] = operandA;
            LOGIC_CONDITION_GLOBAL_FLAG_ENABLE(LOGIC_CONDITION_GLOBAL_FLAG_OVERRIDE_THROTTLE);
            return operandA;
            break;
        
        case LOGIC_CONDITION_SET_OSD_LAYOUT:
            logicConditionValuesByType[LOGIC_CONDITION_SET_OSD_LAYOUT] = operandA;
            LOGIC_CONDITION_GLOBAL_FLAG_ENABLE(LOGIC_CONDITION_GLOBAL_FLAG_OVERRIDE_OSD_LAYOUT);
            return operandA;
            break;

#ifdef USE_I2C_IO_EXPANDER
        case LOGIC_CONDITION_PORT_SET:
            ioPortExpanderSet((uint8_t)operandA, (uint8_t)operandB);
            return operandB;
            break;
#endif

        case LOGIC_CONDITION_SIN:
            temporaryValue = (operandB == 0) ? 500 : operandB;
            return sin_approx(DEGREES_TO_RADIANS(operandA)) * temporaryValue; 
            break;
    
        case LOGIC_CONDITION_COS:
            temporaryValue = (operandB == 0) ? 500 : operandB;
            return cos_approx(DEGREES_TO_RADIANS(operandA)) * temporaryValue; 
            break;
        break;
    
        case LOGIC_CONDITION_TAN:
            temporaryValue = (operandB == 0) ? 500 : operandB;
            return tan_approx(DEGREES_TO_RADIANS(operandA)) * temporaryValue; 
        break;
    
        case LOGIC_CONDITION_MAP_INPUT:
            return scaleRange(constrain(operandA, 0, operandB), 0, operandB, 0, 1000);
        break;
    
        case LOGIC_CONDITION_MAP_OUTPUT:
            return scaleRange(constrain(operandA, 0, 1000), 0, 1000, 0, operandB);
        break;

        case LOGIC_CONDITION_RC_CHANNEL_OVERRIDE:
            temporaryValue = constrain(operandA - 1, 0, MAX_SUPPORTED_RC_CHANNEL_COUNT - 1);
            rcChannelOverrides[temporaryValue].active = true;
            rcChannelOverrides[temporaryValue].value = constrain(operandB, PWM_RANGE_MIN, PWM_RANGE_MAX);
            LOGIC_CONDITION_GLOBAL_FLAG_ENABLE(LOGIC_CONDITION_GLOBAL_FLAG_OVERRIDE_RC_CHANNEL);
            return true;
        break;

        case LOGIC_CONDITION_SET_HEADING_TARGET:
            temporaryValue = CENTIDEGREES_TO_DEGREES(wrap_36000(DEGREES_TO_CENTIDEGREES(operandA)));
            updateHeadingHoldTarget(temporaryValue);
            return temporaryValue;
        break;

        case LOGIC_CONDITION_MODULUS:
            if (operandB != 0) {
                return constrain(operandA % operandB, INT16_MIN, INT16_MAX);
            } else {
                return operandA;
            }
            break;

        default:
            return false;
            break; 
    }
}

void logicConditionProcess(uint8_t i) {

    const int activatorValue = logicConditionGetValue(logicConditions(i)->activatorId);

    if (logicConditions(i)->enabled && activatorValue) {
        
        /*
         * Process condition only when latch flag is not set
         * Latched LCs can only go from OFF to ON, not the other way
         */
        if (!(logicConditionStates[i].flags & LOGIC_CONDITION_FLAG_LATCH)) {
            const int operandAValue = logicConditionGetOperandValue(logicConditions(i)->operandA.type, logicConditions(i)->operandA.value);
            const int operandBValue = logicConditionGetOperandValue(logicConditions(i)->operandB.type, logicConditions(i)->operandB.value);
            const int newValue = logicConditionCompute(
                logicConditionStates[i].value, 
                logicConditions(i)->operation, 
                operandAValue, 
                operandBValue
            );
        
            logicConditionStates[i].value = newValue;

            /*
             * if value evaluates as true, put a latch on logic condition
             */
            if (logicConditions(i)->flags & LOGIC_CONDITION_FLAG_LATCH && newValue) {
                logicConditionStates[i].flags |= LOGIC_CONDITION_FLAG_LATCH;
            }
        }
    } else {
        logicConditionStates[i].value = false;
    }
}

static int logicConditionGetFlightOperandValue(int operand) {

    switch (operand) {

        case LOGIC_CONDITION_OPERAND_FLIGHT_ARM_TIMER: // in s
            return constrain((uint32_t)getFlightTime(), 0, INT16_MAX);
            break;
        
        case LOGIC_CONDITION_OPERAND_FLIGHT_HOME_DISTANCE: //in m
            return constrain(GPS_distanceToHome, 0, INT16_MAX);
            break;
        
        case LOGIC_CONDITION_OPERAND_FLIGHT_TRIP_DISTANCE: //in m
            return constrain(getTotalTravelDistance() / 100, 0, INT16_MAX);
            break;

        case LOGIC_CONDITION_OPERAND_FLIGHT_RSSI:
            return constrain(getRSSI() * 100 / RSSI_MAX_VALUE, 0, 99);
            break;
        
        case LOGIC_CONDITION_OPERAND_FLIGHT_VBAT: // V / 100
            return getBatteryVoltage();
            break;

        case LOGIC_CONDITION_OPERAND_FLIGHT_CELL_VOLTAGE: // V / 10
            return getBatteryAverageCellVoltage();
            break;
        case LOGIC_CONDITION_OPERAND_FLIGHT_CURRENT: // Amp / 100
            return getAmperage();
            break;
        
        case LOGIC_CONDITION_OPERAND_FLIGHT_MAH_DRAWN: // mAh
            return getMAhDrawn();
            break;

        case LOGIC_CONDITION_OPERAND_FLIGHT_GPS_SATS:
            if (getHwGPSStatus() == HW_SENSOR_UNAVAILABLE || getHwGPSStatus() == HW_SENSOR_UNHEALTHY) {
                return 0;
            } else {
                return gpsSol.numSat;
            }
            break;
            
        case LOGIC_CONDITION_OPERAND_FLIGHT_GPS_VALID: // 0/1
            return STATE(GPS_FIX) ? 1 : 0;
            break;

        case LOGIC_CONDITION_OPERAND_FLIGHT_GROUD_SPEED: // cm/s
            return gpsSol.groundSpeed;
            break;

        //FIXME align with osdGet3DSpeed
        case LOGIC_CONDITION_OPERAND_FLIGHT_3D_SPEED: // cm/s
            return osdGet3DSpeed();
            break;

        case LOGIC_CONDITION_OPERAND_FLIGHT_AIR_SPEED: // cm/s
        #ifdef USE_PITOT
            return constrain(pitot.airSpeed, 0, INT16_MAX);
        #else
            return false;
        #endif
            break;

        case LOGIC_CONDITION_OPERAND_FLIGHT_ALTITUDE: // cm
            return constrain(getEstimatedActualPosition(Z), INT16_MIN, INT16_MAX);
            break;

        case LOGIC_CONDITION_OPERAND_FLIGHT_VERTICAL_SPEED: // cm/s
            return constrain(getEstimatedActualVelocity(Z), INT16_MIN, INT16_MAX);
            break;

        case LOGIC_CONDITION_OPERAND_FLIGHT_TROTTLE_POS: // %
            return (constrain(rcCommand[THROTTLE], PWM_RANGE_MIN, PWM_RANGE_MAX) - PWM_RANGE_MIN) * 100 / (PWM_RANGE_MAX - PWM_RANGE_MIN);
            break;

        case LOGIC_CONDITION_OPERAND_FLIGHT_ATTITUDE_ROLL: // deg
            return constrain(attitude.values.roll / 10, -180, 180);
            break;

        case LOGIC_CONDITION_OPERAND_FLIGHT_ATTITUDE_PITCH: // deg
            return constrain(attitude.values.pitch / 10, -180, 180);
            break;

        case LOGIC_CONDITION_OPERAND_FLIGHT_IS_ARMED: // 0/1
            return ARMING_FLAG(ARMED) ? 1 : 0;
            break;

        case LOGIC_CONDITION_OPERAND_FLIGHT_IS_AUTOLAUNCH: // 0/1
            return (navGetCurrentStateFlags() & NAV_CTL_LAUNCH) ? 1 : 0;
            break; 
        
        case LOGIC_CONDITION_OPERAND_FLIGHT_IS_ALTITUDE_CONTROL: // 0/1
            return (navGetCurrentStateFlags() & NAV_CTL_ALT) ? 1 : 0;
            break; 

        case LOGIC_CONDITION_OPERAND_FLIGHT_IS_POSITION_CONTROL: // 0/1
            return (navGetCurrentStateFlags() & NAV_CTL_POS) ? 1 : 0;
            break; 

        case LOGIC_CONDITION_OPERAND_FLIGHT_IS_EMERGENCY_LANDING: // 0/1
            return (navGetCurrentStateFlags() & NAV_CTL_EMERG) ? 1 : 0;
            break;

        case LOGIC_CONDITION_OPERAND_FLIGHT_IS_RTH: // 0/1
            return (navGetCurrentStateFlags() & NAV_AUTO_RTH) ? 1 : 0;
            break; 

        case LOGIC_CONDITION_OPERAND_FLIGHT_IS_WP: // 0/1
            return (navGetCurrentStateFlags() & NAV_AUTO_WP) ? 1 : 0;
            break; 

        case LOGIC_CONDITION_OPERAND_FLIGHT_IS_LANDING: // 0/1
            return (navGetCurrentStateFlags() & NAV_CTL_LAND) ? 1 : 0;
            break;

        case LOGIC_CONDITION_OPERAND_FLIGHT_IS_FAILSAFE: // 0/1
            return (failsafePhase() != FAILSAFE_IDLE) ? 1 : 0;
            break;
        
        case LOGIC_CONDITION_OPERAND_FLIGHT_STABILIZED_YAW: // 
            return axisPID[YAW];
            break;
        
        case LOGIC_CONDITION_OPERAND_FLIGHT_STABILIZED_ROLL: // 
            return axisPID[ROLL];
            break;
        
        case LOGIC_CONDITION_OPERAND_FLIGHT_STABILIZED_PITCH: // 
            return axisPID[PITCH];
            break;

        case LOGIC_CONDITION_OPERAND_FLIGHT_WAYPOINT_INDEX:
            return NAV_Status.activeWpNumber;
            break;

        case LOGIC_CONDITION_OPERAND_FLIGHT_WAYPOINT_ACTION:
            return NAV_Status.activeWpAction;
            break;

        case LOGIC_CONDITION_OPERAND_FLIGHT_3D_HOME_DISTANCE: //in m
            return constrain(sqrtf(sq(GPS_distanceToHome) + sq(getEstimatedActualPosition(Z)/100)), 0, INT16_MAX);
            break;

        case LOGIC_CONDITION_OPERAND_FLIGHT_CRSF_LQ:
        #ifdef USE_SERIALRX_CRSF
            return rxLinkStatistics.uplinkLQ;
        #else
            return 0;
        #endif
            break;

        case LOGIC_CONDITION_OPERAND_FLIGHT_CRSF_SNR:
        #ifdef USE_SERIALRX_CRSF
            return rxLinkStatistics.uplinkSNR;
        #else
            return 0;
        #endif
            break;

        default:
            return 0;
            break;
    }
}

static int logicConditionGetFlightModeOperandValue(int operand) {

    switch (operand) {

        case LOGIC_CONDITION_OPERAND_FLIGHT_MODE_FAILSAFE:
            return (bool) FLIGHT_MODE(FAILSAFE_MODE);
            break;

        case LOGIC_CONDITION_OPERAND_FLIGHT_MODE_MANUAL:
            return (bool) FLIGHT_MODE(MANUAL_MODE);
            break;

        case LOGIC_CONDITION_OPERAND_FLIGHT_MODE_RTH:
            return (bool) FLIGHT_MODE(NAV_RTH_MODE);
            break;

        case LOGIC_CONDITION_OPERAND_FLIGHT_MODE_POSHOLD:
            return (bool) FLIGHT_MODE(NAV_POSHOLD_MODE);
            break;

        case LOGIC_CONDITION_OPERAND_FLIGHT_MODE_COURSE_HOLD:
            return (bool) FLIGHT_MODE(NAV_COURSE_HOLD_MODE);
            break;

        case LOGIC_CONDITION_OPERAND_FLIGHT_MODE_CRUISE:
            return (bool)(FLIGHT_MODE(NAV_COURSE_HOLD_MODE) && FLIGHT_MODE(NAV_ALTHOLD_MODE));
            break;

        case LOGIC_CONDITION_OPERAND_FLIGHT_MODE_ALTHOLD:
            return (bool) FLIGHT_MODE(NAV_ALTHOLD_MODE);
            break;

        case LOGIC_CONDITION_OPERAND_FLIGHT_MODE_ANGLE:
            return (bool) FLIGHT_MODE(ANGLE_MODE);
            break;

        case LOGIC_CONDITION_OPERAND_FLIGHT_MODE_HORIZON:
            return (bool) FLIGHT_MODE(HORIZON_MODE);
            break;

        case LOGIC_CONDITION_OPERAND_FLIGHT_MODE_AIR:
            return (bool) FLIGHT_MODE(AIRMODE_ACTIVE);
            break;

        case LOGIC_CONDITION_OPERAND_FLIGHT_MODE_USER1:
            return IS_RC_MODE_ACTIVE(BOXUSER1);
            break;

        case LOGIC_CONDITION_OPERAND_FLIGHT_MODE_USER2:
            return IS_RC_MODE_ACTIVE(BOXUSER2);
            break;

        default:
            return 0;
            break;
    }
}

int logicConditionGetOperandValue(logicOperandType_e type, int operand) {
    int retVal = 0;

    switch (type) {

        case LOGIC_CONDITION_OPERAND_TYPE_VALUE:
            retVal = operand;
            break;

        case LOGIC_CONDITION_OPERAND_TYPE_RC_CHANNEL:
            //Extract RC channel raw value
            if (operand >= 1 && operand <= 16) {
                retVal = rxGetChannelValue(operand - 1);
            } 
            break;

        case LOGIC_CONDITION_OPERAND_TYPE_FLIGHT:
            retVal = logicConditionGetFlightOperandValue(operand);
            break;

        case LOGIC_CONDITION_OPERAND_TYPE_FLIGHT_MODE:
            retVal = logicConditionGetFlightModeOperandValue(operand);
            break;

        case LOGIC_CONDITION_OPERAND_TYPE_LC:
            if (operand >= 0 && operand < MAX_LOGIC_CONDITIONS) {
                retVal = logicConditionGetValue(operand);
            }
            break;

        case LOGIC_CONDITION_OPERAND_TYPE_GVAR:
            if (operand >= 0 && operand < MAX_GLOBAL_VARIABLES) {
                retVal = gvGet(operand);
            }
            break;

        case LOGIC_CONDITION_OPERAND_TYPE_PID:
            if (operand >= 0 && operand < MAX_PROGRAMMING_PID_COUNT) {
                retVal = programmingPidGetOutput(operand);
            }
            break;

        default:
            break;
    }

    return retVal;
}

/*
 * conditionId == -1 is always evaluated as true
 */ 
int logicConditionGetValue(int8_t conditionId) {
    if (conditionId >= 0) {
        return logicConditionStates[conditionId].value;
    } else {
        return true;
    }
}

void logicConditionUpdateTask(timeUs_t currentTimeUs) {
    UNUSED(currentTimeUs);

    //Disable all flags
    logicConditionsGlobalFlags = 0;

    for (uint8_t i = 0; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
        rcChannelOverrides[i].active = false;
    }

    for (uint8_t i = 0; i < MAX_LOGIC_CONDITIONS; i++) {
        logicConditionProcess(i);
    }

#ifdef USE_I2C_IO_EXPANDER
    ioPortExpanderSync();
#endif
}

void logicConditionReset(void) {
    for (uint8_t i = 0; i < MAX_LOGIC_CONDITIONS; i++) {
        logicConditionStates[i].value = 0;
        logicConditionStates[i].flags = 0;
    }
}

float NOINLINE getThrottleScale(float globalThrottleScale) {
    if (LOGIC_CONDITION_GLOBAL_FLAG(LOGIC_CONDITION_GLOBAL_FLAG_OVERRIDE_THROTTLE_SCALE)) {
        return constrainf(logicConditionValuesByType[LOGIC_CONDITION_OVERRIDE_THROTTLE_SCALE] / 100.0f, 0.0f, 1.0f);
    } else {
        return globalThrottleScale;
    }
}

int16_t getRcCommandOverride(int16_t command[], uint8_t axis) {
    int16_t outputValue = command[axis];

    if (LOGIC_CONDITION_GLOBAL_FLAG(LOGIC_CONDITION_GLOBAL_FLAG_OVERRIDE_SWAP_ROLL_YAW) && axis == FD_ROLL) {
        outputValue = command[FD_YAW];
    } else if (LOGIC_CONDITION_GLOBAL_FLAG(LOGIC_CONDITION_GLOBAL_FLAG_OVERRIDE_SWAP_ROLL_YAW) && axis == FD_YAW) {
        outputValue = command[FD_ROLL];
    }

    if (LOGIC_CONDITION_GLOBAL_FLAG(LOGIC_CONDITION_GLOBAL_FLAG_OVERRIDE_INVERT_ROLL) && axis == FD_ROLL) {
        outputValue *= -1;
    }
    if (LOGIC_CONDITION_GLOBAL_FLAG(LOGIC_CONDITION_GLOBAL_FLAG_OVERRIDE_INVERT_PITCH) && axis == FD_PITCH) {
        outputValue *= -1;
    }
    if (LOGIC_CONDITION_GLOBAL_FLAG(LOGIC_CONDITION_GLOBAL_FLAG_OVERRIDE_INVERT_YAW) && axis == FD_YAW) {
        outputValue *= -1;
    }

    return outputValue;
}

int16_t getRcChannelOverride(uint8_t channel, int16_t originalValue) {
    if (rcChannelOverrides[channel].active) {
        return rcChannelOverrides[channel].value;
    } else {
        return originalValue;
    }
}
