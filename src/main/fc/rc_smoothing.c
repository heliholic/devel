/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
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


#define RC_SMOOTHING_CUTOFF_MIN_HZ              15    // Minimum rc smoothing cutoff frequency
#define RC_SMOOTHING_FILTER_STARTUP_DELAY_MS    5000  // Time to wait after power to let the PID loop stabilize before starting average frame rate calculation
#define RC_SMOOTHING_FILTER_TRAINING_SAMPLES    50    // Number of rx frame rate samples to average during initial training
#define RC_SMOOTHING_FILTER_RETRAINING_SAMPLES  20    // Number of rx frame rate samples to average during frame rate changes
#define RC_SMOOTHING_FILTER_TRAINING_DELAY_MS   1000  // Additional time to wait after receiving first valid rx frame before initial training starts
#define RC_SMOOTHING_FILTER_RETRAINING_DELAY_MS 2000  // Guard time to wait after retraining to prevent retraining again too quickly
#define RC_SMOOTHING_RX_RATE_CHANGE_PERCENT     20    // Look for samples varying this much from the current detected frame rate to initiate retraining
#define RC_SMOOTHING_SP_DELTA_INITIAL_HZ        100   // The value to use for "auto"


static FAST_DATA_ZERO_INIT rcSmoothingFilter_t rcSmoothing;


static void rcSmoothingResetAccumulation(void)
{
    rcSmoothing.training.sum = 0;
    rcSmoothing.training.count = 0;
    rcSmoothing.training.min = UINT16_MAX;
    rcSmoothing.training.max = 0;
}

static FAST_CODE bool rcSmoothingAccumulateSample(int rxFrameTimeUs)
{
    rcSmoothing.training.sum += rxFrameTimeUs;
    rcSmoothing.training.count++;
    rcSmoothing.training.max = MAX(rcSmoothing.training.max, rxFrameTimeUs);
    rcSmoothing.training.min = MIN(rcSmoothing.training.min, rxFrameTimeUs);

    // if we've collected enough samples then calculate the average and reset the accumulation
    const int sampleLimit = (rcSmoothing.filterInitialized) ? RC_SMOOTHING_FILTER_RETRAINING_SAMPLES : RC_SMOOTHING_FILTER_TRAINING_SAMPLES;

    if (rcSmoothing.training.count >= sampleLimit) {
        rcSmoothing.training.sum = rcSmoothing.training.sum - rcSmoothing.training.min - rcSmoothing.training.max; // Throw out high and low samples
        rcSmoothing.averageFrameTimeUs = lrintf(rcSmoothing.training.sum / (rcSmoothing.training.count - 2));
        rcSmoothingResetAccumulation();
        return true;
    }

    return false;
}

static inline int calcAutoSmoothingCutoff(int avgRxFrameTimeUs, uint8_t autoSmoothnessFactor)
{
    float cutoff = 0;

    if (avgRxFrameTimeUs > 0) {
        const float cutoffFactor = 1.5f / (1.0f + (autoSmoothnessFactor / 10.0f));
        cutoff = (1e6f / avgRxFrameTimeUs) * cutoffFactor;
    }
    
    return MAX(RC_SMOOTHING_CUTOFF_MIN_HZ, lrintf(cutoff));
}


static FAST_CODE void rcSmoothingSetFilterCutoffs(void)
{
    const float dT = pidGetDT();
    uint16_t cutoff;

    if (rcSmoothing.setpointCutoffSetting == 0) {
        cutoff = calcAutoSmoothingCutoff(rcSmoothing.averageFrameTimeUs, rcSmoothing.autoSmoothnessFactorSetpoint);
        if (rcSmoothing.setpointCutoffFrequency != cutoff) {
            float gain = pt3FilterGain(rcSmoothing.setpointCutoffFrequency, dT);
            for (int i = 0; i < 4; i++)
                pt3FilterUpdateCutoff(&rcSmoothing.setpointFilter[i], gain);
            rcSmoothing.setpointCutoffFrequency = cutoff;
        }
    }

    if (rcSmoothing.throttleCutoffSetting == 0) {
        cutoff = calcAutoSmoothingCutoff(rcSmoothing.averageFrameTimeUs, rcSmoothing.autoSmoothnessFactorThrottle);
        if (rcSmoothing.throttleCutoffFrequency != cutoff) {
            float gain = pt3FilterGain(rcSmoothing.throttleCutoffFrequency, dT);
            pt3FilterUpdateCutoff(&rcSmoothing.setpointFilter[4], gain);
            rcSmoothing.throttleCutoffFrequency = cutoff;
        }
    }

    if (rcSmoothing.setpointDeltaCutoffSetting == 0) {
        cutoff = calcAutoSmoothingCutoff(rcSmoothing.averageFrameTimeUs, rcSmoothing.autoSmoothnessFactorSetpoint);
        if (rcSmoothing.setpointDeltaCutoffFrequency != cutoff) {
            float gain = pt3FilterGain(rcSmoothing.setpointDeltaCutoffFrequency, dT);
            for (int i = 0; i < 3; i++)
                pt3FilterUpdateCutoff(&rcSmoothing.setpointDeltaFilter[i], gain);
            rcSmoothing.setpointDeltaCutoffFrequency = cutoff;
        }
    }
}


FAST_CODE bool rcSmoothingAutoCalculate(void)
{
    return ((rcSmoothing.setpointCutoffSetting == 0) || 
            (rcSmoothing.setpointDeltaCutoffSetting == 0) || 
            (rcSmoothing.throttleCutoffSetting == 0));
}


INIT_CODE void rcSmoothingFilterInit(void)
{
    rcSmoothing.filterInitialized = false;

    rcSmoothing.averageFrameTimeUs = 0;
    rcSmoothing.autoSmoothnessFactorSetpoint = rxConfig()->rc_smoothing_auto_factor_rpy;
    rcSmoothing.autoSmoothnessFactorThrottle = rxConfig()->rc_smoothing_auto_factor_throttle;
    rcSmoothing.setpointCutoffSetting = rxConfig()->rc_smoothing_setpoint_cutoff;
    rcSmoothing.throttleCutoffSetting = rxConfig()->rc_smoothing_throttle_cutoff;
    rcSmoothing.setpointDeltaCutoffSetting = rxConfig()->rc_smoothing_setpoint_delta_cutoff;

    rcSmoothing.setpointCutoffFrequency = (rcSmoothing.setpointCutoffSetting) ?
        rcSmoothing.setpointCutoffSetting : RC_SMOOTHING_CUTOFF_MIN_HZ;
    rcSmoothing.throttleCutoffFrequency = (rcSmoothing.throttleCutoffSetting) ?
        rcSmoothing.throttleCutoffSetting : RC_SMOOTHING_CUTOFF_MIN_HZ;
    rcSmoothing.setpointDeltaCutoffFrequency = (rcSmoothing.setpointDeltaCutoffSetting) ?
        rcSmoothing.setpointDeltaCutoffSetting : RC_SMOOTHING_SP_DELTA_INITIAL_HZ;

    rcSmoothing.debugAxis = rxConfig()->rc_smoothing_debug_axis;
    
    rcSmoothingResetAccumulation();

    const float dT = pidGetDT();

    for (int i = 0; i < 4; i++)
        pt3FilterInit(&rcSmoothing.setpointFilter[i], pt3FilterGain(rcSmoothing.setpointCutoffFrequency, dT));

    for (int i = 0; i < 3; i++)
        pt3FilterInit(&rcSmoothing.setpointDeltaFilter[i], pt3FilterGain(rcSmoothing.setpointDeltaCutoffFrequency, dT));

    pt3FilterInit(&rcSmoothing.setpointFilter[4], pt3FilterGain(rcSmoothing.throttleCutoffFrequency, dT));

    if (rxConfig()->rc_smoothing_mode) {
        rcSmoothing.calculateCutoffs = rcSmoothingAutoCalculate();
        if (!rcSmoothing.calculateCutoffs) {
            rcSmoothingSetFilterCutoffs();
            rcSmoothing.filterInitialized = true;
        }
    }
}


FAST_CODE void rcSmoothingFilterUpdate(bool isRxRateValid, uint16_t currentRxRefreshRate)
{
    // for auto calculated filters we need to examine each rx frame interval
    if (rcSmoothing.calculateCutoffs) {
        const timeMs_t currentTimeMs = millis();
        int sampleState = 0;

        // If the filter cutoffs in auto mode, and we have good rx data, then determine the average rx frame rate
        // and use that to calculate the filter cutoff frequencies
        if ((currentTimeMs > RC_SMOOTHING_FILTER_STARTUP_DELAY_MS) && (gyro.targetLooptime > 0)) {
            if (rxIsReceivingSignal() && isRxRateValid) {

                // set the guard time expiration if it's not set
                if (rcSmoothing.validRxFrameTimeMs == 0) {
                    rcSmoothing.validRxFrameTimeMs = currentTimeMs + (rcSmoothing.filterInitialized ? RC_SMOOTHING_FILTER_RETRAINING_DELAY_MS : RC_SMOOTHING_FILTER_TRAINING_DELAY_MS);
                } else {
                    sampleState = 1;
                }

                // if the guard time has expired then process the rx frame time
                if (currentTimeMs > rcSmoothing.validRxFrameTimeMs) {
                    sampleState = 2;
                    bool accumulateSample = true;

                    // During initial training process all samples.
                    // During retraining check samples to determine if they vary by more than the limit percentage.
                    if (rcSmoothing.filterInitialized) {
                        const float percentChange = (ABS(currentRxRefreshRate - rcSmoothing.averageFrameTimeUs) / (float)rcSmoothing.averageFrameTimeUs) * 100;
                        if (percentChange < RC_SMOOTHING_RX_RATE_CHANGE_PERCENT) {
                            // We received a sample that wasn't more than the limit percent so reset the accumulation
                            // During retraining we need a contiguous block of samples that are all significantly different than the current average
                            rcSmoothingResetAccumulation();
                            accumulateSample = false;
                        }
                    }

                    // accumlate the sample into the average
                    if (accumulateSample) {
                        if (rcSmoothingAccumulateSample(currentRxRefreshRate)) {
                            // the required number of samples were collected so set the filter cutoffs, but only if smoothing is active
                            if (rxConfig()->rc_smoothing_mode) {
                                rcSmoothingSetFilterCutoffs();
                                rcSmoothing.filterInitialized = true;
                            }
                            rcSmoothing.validRxFrameTimeMs = 0;
                        }
                    }

                }
            } else {
                // we have either stopped receiving rx samples (failsafe?) or the sample time is unreasonable so reset the accumulation
                rcSmoothingResetAccumulation();
            }
        }

        // rx frame rate training blackbox debugging
        DEBUG_SET(DEBUG_RC_SMOOTHING_RATE, 0, currentRxRefreshRate);              // log each rx frame interval
        DEBUG_SET(DEBUG_RC_SMOOTHING_RATE, 1, rcSmoothing.training.count);    // log the training step count
        DEBUG_SET(DEBUG_RC_SMOOTHING_RATE, 2, rcSmoothing.averageFrameTimeUs);// the current calculated average
        DEBUG_SET(DEBUG_RC_SMOOTHING_RATE, 3, sampleState);                       // indicates whether guard time is active
    }
}

FAST_CODE float rcSmoothingFilterApply(int axis, float rawSetpoint)
{
    float newSetpoint = pt3FilterApply(&rcSmoothing.setpointFilter[axis], rawSetpoint);

    if (axis == rcSmoothing.debugAxis) {
        DEBUG_SET(DEBUG_RC_SMOOTHING, 1, lrintf(rawSetpoint * 100.0f));
        DEBUG_SET(DEBUG_RC_SMOOTHING, 2, lrintf(newSetpoint * 100.0f));
    }

    return newSetpoint;
}


FAST_CODE float rcSmoothingDeltaFilterApply(int axis, float rawDelta)
{
    float newDelta = pt3FilterApply(&rcSmoothing.setpointDeltaFilter[axis], rawDelta);

    if (axis == rcSmoothing.debugAxis) {
        DEBUG_SET(DEBUG_RC_SMOOTHING, 3, lrintf(rawDelta * 100.0f));
        DEBUG_SET(DEBUG_RC_SMOOTHING, 4, lrintf(newDelta * 100.0f));
    }

    return newDelta;
}


bool rcSmoothingInitializationComplete(void) {
    return rcSmoothing.filterInitialized;
}

rcSmoothingFilter_t *getRcSmoothingData(void)
{
    return &rcSmoothing;
}

#endif
