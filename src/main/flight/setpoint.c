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

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#include "build/debug.h"

#include "common/axis.h"

#include "config/config.h"
#include "config/feature.h"

#include "flight/pid.h"

#include "fc/rc.h"

#include "setpoint.h"


#define SP_SMOOTHING_FILTER_MIN_HZ             5
#define SP_SMOOTHING_FILTER_MAX_HZ           250

#define SP_SMOOTHING_RX_RATE_MIN_US          950
#define SP_SMOOTHING_RX_RATE_MAX_US        65000

#define SP_SMOOTHING_RX_RATE_AVERAGING       100


typedef struct
{
    float accelLimit[4];
    float limitedSp[4];
    float setpoint[4];

    pt3Filter_t filter[4];

    uint16_t styleCutoff;
    uint16_t activeCutoff;
    uint16_t averageFrameCount;

    float currentFrameTimeUs;
    float averageFrameTimeUs;

} setpointFilter_t;

static FAST_DATA_ZERO_INIT setpointFilter_t spFilter;


float getSetpoint(int axis)
{
    return spFilter.setpoint[axis];
}

uint16_t setpointFilterGetCutoffFreq(void)
{
    return spFilter.activeCutoff;
}

uint16_t setpointFilterGetRxFrameTime(void)
{
    return rxIsReceivingSignal() ? lrintf(spFilter.averageFrameTimeUs) : 0;
}


static inline float setpointAutoSmoothingCutoff(float frameTimeUs, uint8_t autoSmoothnessFactor)
{
    float cutoff = 0;

    if (frameTimeUs > 0) {
        float factor = 1.5f / (1.0f + (autoSmoothnessFactor / 10.0f));
        cutoff = factor * (1.0f / (frameTimeUs * 1e-6f));
    }

    return cutoff;
}

static inline void setpointFilterSetCutoffs(float frameTimeUs)
{
    float cutoff = setpointAutoSmoothingCutoff(frameTimeUs, currentControlRateProfile->rates_smoothness);

    cutoff = MIN(spFilter.styleCutoff, cutoff);
    cutoff = constrain(cutoff, SP_SMOOTHING_FILTER_MIN_HZ, SP_SMOOTHING_FILTER_MAX_HZ);

    if (spFilter.activeCutoff != cutoff) {
        const float gain = pt3FilterGain(cutoff, pidGetDT());
        for (int i = 0; i < 4; i++) {
            pt3FilterUpdateCutoff(&spFilter.filter[i], gain);
        }
        spFilter.activeCutoff = cutoff;
    }
}

FAST_CODE void setpointFilterUpdate(int currentRxRateUs)
{
    if (spFilter.averageFrameCount >= SP_SMOOTHING_RX_RATE_AVERAGING) {
        if (rxIsReceivingSignal() && currentRxRateUs > SP_SMOOTHING_RX_RATE_MIN_US && currentRxRateUs < SP_SMOOTHING_RX_RATE_MAX_US) {
            spFilter.currentFrameTimeUs = constrainf(currentRxRateUs, 0.75f * spFilter.averageFrameTimeUs, 1.25f * spFilter.averageFrameTimeUs);
        } else {
            spFilter.currentFrameTimeUs = spFilter.averageFrameTimeUs;
        }
    }
    else {
        spFilter.currentFrameTimeUs = currentRxRateUs;
        spFilter.averageFrameCount++;
    }

    spFilter.averageFrameTimeUs += (spFilter.currentFrameTimeUs - spFilter.averageFrameTimeUs) / spFilter.averageFrameCount;

    setpointFilterSetCutoffs(spFilter.averageFrameTimeUs);

    DEBUG(SETPOINT, 4, currentRxRateUs);
    DEBUG(SETPOINT, 5, spFilter.currentFrameTimeUs);
    DEBUG(SETPOINT, 6, spFilter.averageFrameTimeUs);
    DEBUG(SETPOINT, 7, spFilter.activeCutoff);
}


INIT_CODE void setpointFilterInitProfile(void)
{
    for (int i = 0; i < 4; i++) {
        spFilter.accelLimit[i] = 10.0f * currentControlRateProfile->accel_limit[i] * pidGetDT();
    }

    spFilter.styleCutoff  = 1000.0f / constrain(currentControlRateProfile->rates_response, 1, 250);
    spFilter.activeCutoff = constrain(spFilter.styleCutoff, SP_SMOOTHING_FILTER_MIN_HZ, SP_SMOOTHING_FILTER_MAX_HZ);
}

INIT_CODE void setpointFilterInit(void)
{
    spFilter.currentFrameTimeUs = 10;
    spFilter.averageFrameTimeUs = 10;

    spFilter.averageFrameCount = 0;

    setpointFilterInitProfile();

    const float gain = pt3FilterGain(spFilter.activeCutoff, pidGetDT());
    for (int i = 0; i < 4; i++) {
        pt3FilterInit(&spFilter.filter[i], gain);
    }
}


FAST_CODE void processSetpoint(void)
{
    for (int axis = 0; axis < 4; axis++) {
        float setpoint = getRawSetpoint(axis);
        DEBUG_AXIS(SETPOINT, axis, 0, setpoint);

        setpoint = spFilter.limitedSp[axis] = slew_limit(spFilter.limitedSp[axis], setpoint, spFilter.accelLimit[axis]);
        DEBUG_AXIS(SETPOINT, axis, 1, setpoint);

        setpoint = pt3FilterApply(&spFilter.filter[axis], setpoint);
        DEBUG_AXIS(SETPOINT, axis, 2, setpoint);

        spFilter.setpoint[axis] = setpoint;
    }
}

