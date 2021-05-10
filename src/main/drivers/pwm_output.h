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

#pragma once

#include "drivers/io_types.h"
#include "drivers/time.h"

typedef enum {
    DSHOT_CMD_SPIN_DIRECTION_NORMAL = 20,
    DSHOT_CMD_SPIN_DIRECTION_REVERSED = 21,
} dshotCommands_e;

typedef struct {
    dshotCommands_e cmd;
    int remainingRepeats;
} currentExecutingCommand_t;

void pwmRequestMotorTelemetry(int motorIndex);

ioTag_t pwmGetMotorPinTag(int motorIndex);

void pwmWriteMotor(uint8_t index, uint16_t value);
void pwmShutdownPulsesForAllMotors(uint8_t motorCount);
void pwmCompleteMotorUpdate(void);
bool isMotorProtocolDigital(void);
bool isMotorProtocolDshot(void);

void pwmWriteServo(uint8_t index, uint16_t value);

void pwmDisableMotors(void);
void pwmEnableMotors(void);
struct timerHardware_s;

void pwmMotorPreconfigure(void);
bool pwmMotorConfig(const struct timerHardware_s *timerHardware, uint8_t motorIndex, bool enableOutput);

void pwmServoPreconfigure(void);
bool pwmServoConfig(const struct timerHardware_s *timerHardware, uint8_t servoIndex, uint16_t servoPwmRate, uint16_t servoCenterPulse, bool enableOutput);
void pwmWriteBeeper(bool onoffBeep);
void beeperPwmInit(ioTag_t tag, uint16_t frequency);

void sendDShotCommand(dshotCommands_e cmd);
void initDShotCommands(void);