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
#include "common/utils.h"

#include "config/config.h"
#include "config/feature.h"

#ifdef USE_RC_SMOOTHING_FILTER

#include "fc/runtime_config.h"
#include "fc/core.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/rc_rates.h"

#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/pid.h"
#include "flight/gps_rescue.h"
#include "flight/pid_init.h"

#include "pg/rx.h"

#include "rx/rx.h"

#include "sensors/battery.h"
#include "sensors/gyro.h"

#include "rc_smoothing.h"


#define RC_SMOOTHING_CUTOFF_MIN_HZ              10      // Minimum rc smoothing cutoff frequency
#define RC_SMOOTHING_CUTOFF_MAX_HZ              250     // Maximum rc smoothing cutoff frequency

#define RC_SMOOTHING_STARTUP_DELAY_MS           5000    // Time to wait after power to let the PID loop stabilize before starting average frame rate calculation

#define RC_SMOOTHING_RX_RATE_MIN_US             950     // 0.950ms to fit 1kHz without an issue
#define RC_SMOOTHING_RX_RATE_MAX_US             62500   // 62.5ms or 16hz
#define RC_SMOOTHING_RX_RATE_AVERAGE            100


typedef struct
{
    pt3Filter_t filter[4];

    bool dynamicCutoffs;
    uint16_t cutoffFreq;
    
    float currentFrameTimeUs;
    float averageFrameTimeUs;
    float averageFrameTimeAlpha;

} rcSmoothingFilter_t;

static FAST_DATA_ZERO_INIT rcSmoothingFilter_t rcSmoothing;


uint16_t rcSmoothingGetCutoffFreq(void)
{
    return rcSmoothing.cutoffFreq;
}

uint16_t rcSmoothingGetRxFrameTime(void)
{
    return rxIsReceivingSignal() ? lrintf(rcSmoothing.averageFrameTimeUs) : 0;
}


static inline int calcAutoSmoothingCutoff(float frameTimeUs, uint8_t autoSmoothnessFactor)
{
    float cutoff = 0;

    if (frameTimeUs > 0) {
        // factor = 1.5f / (1.0f + (autoSmoothnessFactor / 10.0f));
        // cutoff = factor * rxFrameRate = factor * (1.0f / (avgRxFrameTimeUs * 1e-6f));
        cutoff = 15000000 / (frameTimeUs * (10 + autoSmoothnessFactor));
    }

    return constrain(cutoff, RC_SMOOTHING_CUTOFF_MIN_HZ, RC_SMOOTHING_CUTOFF_MAX_HZ);
}

static void rcSmoothingSetFilterCutoffs(float frameTimeUs)
{
    uint16_t cutoff = calcAutoSmoothingCutoff(frameTimeUs, rxConfig()->rc_smoothing_factor);

    if (rcSmoothing.cutoffFreq != cutoff) {
        const float gain = pt3FilterGain(cutoff, pidGetDT());
        for (int i = 0; i < 4; i++) {
            pt3FilterUpdateCutoff(&rcSmoothing.filter[i], gain);
        }
        rcSmoothing.cutoffFreq = cutoff;
    }
}


INIT_CODE void rcSmoothingFilterInit(void)
{
    if (rxConfig()->rc_smoothing_mode) {

        rcSmoothing.dynamicCutoffs = rcSmoothingAutoCalculate();
        rcSmoothing.cutoffFreq = MAX(RC_SMOOTHING_CUTOFF_MIN_HZ, rxConfig()->rc_smoothing_cutoff);

        rcSmoothing.currentFrameTimeUs = 10;
        rcSmoothing.averageFrameTimeUs = 10;
        rcSmoothing.averageFrameTimeAlpha = 1.0f / RC_SMOOTHING_RX_RATE_AVERAGE;

        const float gain = pt3FilterGain(rcSmoothing.cutoffFreq, pidGetDT());

        for (int i = 0; i < 4; i++) {
            pt3FilterInit(&rcSmoothing.filter[i], gain);
        }
    }
}

FAST_CODE void rcSmoothingFilterUpdate(int currentRxRateUs)
{
    if (rcSmoothing.dynamicCutoffs) {
        const timeMs_t currentTimeMs = millis();

        if (currentTimeMs > RC_SMOOTHING_STARTUP_DELAY_MS) {
            if (rxIsReceivingSignal() && currentRxRateUs > RC_SMOOTHING_RX_RATE_MIN_US && currentRxRateUs < RC_SMOOTHING_RX_RATE_MAX_US) {
                rcSmoothing.currentFrameTimeUs = constrainf(currentRxRateUs, 0.75f * rcSmoothing.averageFrameTimeUs, 1.25f * rcSmoothing.averageFrameTimeUs);
            } else {
                rcSmoothing.currentFrameTimeUs = rcSmoothing.averageFrameTimeUs;
            }
        }
        else {
            rcSmoothing.currentFrameTimeUs = currentRxRateUs;
        }

        rcSmoothing.averageFrameTimeUs += (rcSmoothing.currentFrameTimeUs - rcSmoothing.averageFrameTimeUs) * rcSmoothing.averageFrameTimeAlpha;

        rcSmoothingSetFilterCutoffs(rcSmoothing.averageFrameTimeUs);

        DEBUG(RC_SMOOTHING_RATE, 0, currentRxRateUs);
        DEBUG(RC_SMOOTHING_RATE, 1, rcSmoothing.currentFrameTimeUs);
        DEBUG(RC_SMOOTHING_RATE, 2, rcSmoothing.averageFrameTimeUs);
        DEBUG(RC_SMOOTHING_RATE, 3, rcSmoothing.cutoffFreq);
    }
}

FAST_CODE float rcSmoothingFilterApply(int axis, float input)
{
    float output = input;

    // RPYC primary controls only
    if (axis < 4) {
        output = pt3FilterApply(&rcSmoothing.filter[axis], input);

        DEBUG_AXIS(RC_SMOOTHING, axis, 1, input * 1000);
        DEBUG_AXIS(RC_SMOOTHING, axis, 2, output * 1000);
    }

    return output;
}


FAST_CODE float rcSmoothingDeltaFilterApply(int axis, float delta)
{
    UNUSED(axis);
    return delta;
}

#endif
