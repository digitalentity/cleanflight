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
 * 
 * This code is a derivative of work done in EmuFlight Project https://github.com/emuflight/EmuFlight
 *  
 */

#include "platform.h"
#ifdef USE_SMITH_PREDICTOR

#include <stdbool.h>
#include "common/axis.h"
#include "common/utils.h"
#include "flight/smith_predictor.h"
#include "build/debug.h"

FUNCTION_COMPILE_FOR_SPEED
float applySmithPredictor(uint8_t axis, smithPredictor_t *predictor, float sample) {
    UNUSED(axis);
    if (predictor->enabled) {
        predictor->data[predictor->idx] = sample;

        predictor->idx++;
        if (predictor->idx > predictor->samples) {
            predictor->idx = 0;
        }

        // filter the delayed data to help reduce the overall noise this prediction adds
        float delayed = pt1FilterApply(&predictor->smithPredictorFilter, predictor->data[predictor->idx]);
        float delayCompensatedSample = predictor->smithPredictorStrength * (sample - delayed);

        sample += delayCompensatedSample;
    }
    return sample;
}

FUNCTION_COMPILE_FOR_SIZE
void smithPredictorInit(smithPredictor_t *predictor, float delay, float strength, uint16_t filterLpfHz, uint32_t looptime) {
    if (delay > 0.1) {
        predictor->enabled = true;
        predictor->samples = (delay * 1000) / looptime;
        predictor->idx = 0;
        predictor->smithPredictorStrength = strength;
        pt1FilterInit(&predictor->smithPredictorFilter, filterLpfHz, looptime * 1e-6f);
    } else {
        predictor->enabled = false;
    }
}

#endif