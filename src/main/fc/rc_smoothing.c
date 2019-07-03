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
 */

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>

#include "platform.h"

#include "blackbox/blackbox.h"

#include "build/debug.h"

#include "common/maths.h"
#include "common/filter.h"

#include "drivers/time.h"

#include "rx/rx.h"

#include "fc/config.h"
#include "fc/fc_core.h"
#include "fc/rc_command.h"
#include "fc/rc_controls.h"
#include "fc/rc_smoothing.h"
#include "fc/runtime_config.h"

#include "flight/mixer.h"

static biquadFilter_t rcSmoothFilter[RC_COMMAND_AXES_COUNT];
static rcCommand_t rcUnfilteredInput;

static void rcInterpolationInit(int rcFilterFrequency)
{
    for (int stick = 0; stick < RC_COMMAND_AXES_COUNT; stick++) {
        biquadFilterInitLPF(&rcSmoothFilter[stick], rcFilterFrequency, getLooptime());
    }
}

bool rcInterpolationApply(rcCommand_t *dst, const rcCommand_t *src, bool isRXDataNew)
{
    static bool initDone = false;
    static float initFilterFrequency = 0;

    if (isRXDataNew) {
        if (!initDone || (initFilterFrequency != rxConfig()->rcFilterFrequency)) {
            rcInterpolationInit(rxConfig()->rcFilterFrequency);
            initFilterFrequency = rxConfig()->rcFilterFrequency;
            initDone = true;
        }

        for (int stick = 0; stick < RC_COMMAND_AXES_COUNT; stick++) {
            rcUnfilteredInput.axes[stick] = src->axes[stick];
        }
    }

    // Don't filter if not initialized
    if (!initDone) {
        return false;
    }

    for (int stick = 0; stick < RC_COMMAND_AXES_COUNT; stick++) {
        dst->axes[stick] = biquadFilterApply(&rcSmoothFilter[stick], rcUnfilteredInput.axes[stick]);
    }
    return true;
}

























