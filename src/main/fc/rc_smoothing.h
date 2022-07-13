/*
 * This file is part of Rotorflight.
 *
 * Rotorflight is free software. You can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Rotorflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software. If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "pg/rx.h"


#define RC_SMOOTHING_FACTOR_MIN 0
#define RC_SMOOTHING_FACTOR_MAX 250


uint16_t rcSmoothingGetCutoffFreq(void);
uint16_t rcSmoothingGetRxFrameTime(void);

void rcSmoothingFilterInit(void);
void rcSmoothingFilterUpdate(int currentRxRefreshRate);
float rcSmoothingFilterApply(int axis, float input);


static inline bool rcSmoothingAutoCalculate(void)
{
    return (rxConfig()->rc_smoothing_cutoff == 0);
}

static inline bool rcSmoothingInitializationComplete(void)
{
    return true;
}

