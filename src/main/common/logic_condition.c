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

#include "common/logic_condition.h"
#include "common/maths.h"
#include "common/utils.h"

#include "fc/fc_core.h"
#include "fc/rc_control.h"

#include "flight/imu.h"

#include "navigation/navigation.h"

#include "rx/rx.h"

#include "sensors/battery.h"
#include "sensors/pitotmeter.h"

PG_REGISTER_ARRAY(logicCondition_t, MAX_LOGIC_CONDITIONS, logicConditions, PG_LOGIC_CONDITIONS, 0);

void pgResetFn_logicConditions(logicCondition_t *instance)
{
    for (int i = 0; i < MAX_LOGIC_CONDITIONS; i++) {
        RESET_CONFIG(logicCondition_t, &instance[i],
            .enabled = 0,
            .operation = LOGIC_CONDITION_TRUE,
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

void logicConditionProcess(uint8_t i) {

    if (logicConditions(i)->enabled) {
        const int operandAValue = logicConditionGetOperandValue(logicConditions(i)->operandA.type, logicConditions(i)->operandA.value);
        const int operandBValue = logicConditionGetOperandValue(logicConditions(i)->operandB.type, logicConditions(i)->operandB.value);
        logicConditionStates[i].value = logicConditionCompute(logicConditions(i)->operation, operandAValue, operandBValue);
    } else {
        logicConditionStates[i].value = false;
    }
}

int logicConditionCompute(
    logicOperation_e operation,
    int operandA,
    int operandB
) {
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

        default:
            return false;
            break; 
    }
}

static int logicConditionGetFlightOperandValue(int operand) {

    switch (operand) {

        case LOGIC_CONDITION_OPERAND_FLIGHT_ARM_TIMER: // in s
            return constrain((uint32_t)getFlightTime(), 0, 32767);
            break;
        
        case LOGIC_CONDITION_OPERAND_FLIGHT_HOME_DISTANCE: //in m
            return constrain(GPS_distanceToHome, 0, 32767);
            break;
        
        case LOGIC_CONDITION_OPERAND_FLIGHT_TRIP_DISTANCE: //in m
            return constrain(getTotalTravelDistance() / 100, 0, 32767);
            break;

        case LOGIC_CONDITION_OPERAND_FLIGHT_RSSI:
            return constrain(getRSSI() * 100 / RSSI_MAX_VALUE, 0, 99);
            break;
        
        case LOGIC_CONDITION_OPERAND_FLIGHT_VBAT: // V / 10
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
            return gpsSol.numSat;
            break;

        case LOGIC_CONDITION_OPERAND_FLIGHT_GROUD_SPEED: // cm/s
            return gpsSol.groundSpeed;
            break;

        //FIXME align with osdGet3DSpeed
        case LOGIC_CONDITION_OPERAND_FLIGHT_3D_SPEED: // cm/s
            return (int) sqrtf(sq(gpsSol.groundSpeed) + sq((int)getEstimatedActualVelocity(Z)));
            break;

        case LOGIC_CONDITION_OPERAND_FLIGHT_AIR_SPEED: // cm/s
        #ifdef USE_PITOT
            return constrain(pitot.airSpeed, 0, 32767);
        #else
            return false;
        #endif
            break;

        case LOGIC_CONDITION_OPERAND_FLIGHT_ALTITUDE: // cm
            return constrain(getEstimatedActualPosition(Z), -32678, 32767);
            break;

        case LOGIC_CONDITION_OPERAND_FLIGHT_VERTICAL_SPEED: // cm/s
            return constrain(getEstimatedActualVelocity(Z), 0, 32767);
            break;

        case LOGIC_CONDITION_OPERAND_FLIGHT_TROTTLE_POS: // %
            return fabsf(rcControlGetOutput()->throttle) * 100;
            break;

        case LOGIC_CONDITION_OPERAND_FLIGHT_ATTITUDE_ROLL: // deg
            return constrain(attitude.values.roll / 10, -180, 180);
            break;

        case LOGIC_CONDITION_OPERAND_FLIGHT_ATTITUDE_PITCH: // deg
            return constrain(attitude.values.pitch / 10, -180, 180);
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

        case LOGIC_CONDITION_OPERAND_TYPE_LC:
            if (operand >= 0 && operand < MAX_LOGIC_CONDITIONS) {
                retVal = logicConditionGetValue(operand);
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
    for (uint8_t i = 0; i < MAX_LOGIC_CONDITIONS; i++) {
        logicConditionProcess(i);
    }
}
