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

#include <stdbool.h>

// Initializes the watchdog, typically called during system startup.
// Note that once enabled, the watchdog can't be disabled. This function
// can be called multiple times and it will always set the watchdog
// interval to 500ms.
void watchdogInit(void);

// Restart the watchdog counter.
void watchdogRestart(void);

// Momentarily suspend the watchdog. If the watchdog isn't running
// (either not initialized or already suspended) this function does
// nothing. Returns true iff the WD was suspended by this call.
bool watchdogSuspend(void);

// Resume the watchdog after suspending it via watchdogSuspend().
// If the watchdog wasn't previously suspended, this function does
// nothing. Returns true iff the WD was resumed by this call.
bool watchdogResume(void);

// Returns true iff the system was restarted
// due to a watchdog reset. This MUST be called
// before systemInit() (which resets the flags by
// calling RCC_ClearFlag()). Otherwise it will always
// return false.
bool watchdogBarked(void);