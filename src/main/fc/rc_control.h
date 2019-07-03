/*
 * This file is part of INAV.
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
 *
 * @author Alberto Garcia Hierro <alberto@garciahierro.com>
 */

#pragma once

#include "fc/rc_command.h"
#include "fc/rc_controls.h"

typedef enum {
    RC_CONTROL_SOURCE_KEEP = 0, // Keep the current source
    RC_CONTROL_SOURCE_RX,
    RC_CONTROL_SOURCE_HEADFREE,
    RC_CONTROL_SOURCE_FAILSAFE,
    RC_CONTROL_SOURCE_NAVIGATION,
} rcControlSource_e;

typedef enum {
    RC_CONTROL_BIDIR_THR_USER,  // User controls wether THR is bidir or not
    RC_CONTROL_BIDIR_THR_ON,    // THR is forced to bidir
    RC_CONTROL_BIDIR_THR_OFF,   // THR is forced to unidir
} rcControlBidirThrottleMode_e;

typedef struct rcControl_s {
    rcCommand_t input; // Input from user
    rcCommand_t output; // Output might be set by multiple subsystems, see rcCommandOutputSource_e
    rcControlSource_e source; // Source that set the output
} rcControl_t;

void rcControlUpdateFromRX(void);
const rcCommand_t *rcControlGetInput(void);
float rcControlGetInputAxis(rc_alias_e axis);
float rcControlGetInputAxisApplyingPosholdDeadband(rc_alias_e axis);
rcControlBidirThrottleMode_e rcControlGetBidirThrottleMode(void);
void rcControlSetBidirThrottleMode(rcControlBidirThrottleMode_e mode);
const rcCommand_t *rcControlGetOutput(void);
float rcControlGetOutputAxis(rc_alias_e axis);
void rcControlUpdateOutput(const rcCommand_t *cmd, rcControlSource_e source);
