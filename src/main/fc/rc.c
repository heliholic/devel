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

#include "fc/controlrate_profile.h"
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

#include "rc.h"


static float rawSetpoint[4];
static float setpointRate[4];
static float rcDeflection[4], rcDeflectionAbs[4];
static uint16_t currentRxRefreshRate;
static bool isRxDataNew = false;
static bool isRxRateValid = false;
static float rcCommandDivider = 500.0f;
static float rcCommandYawDivider = 500.0f;

#ifdef USE_RC_SMOOTHING_FILTER
#define RC_SMOOTHING_CUTOFF_MIN_HZ              15    // Minimum rc smoothing cutoff frequency
#define RC_SMOOTHING_FILTER_STARTUP_DELAY_MS    5000  // Time to wait after power to let the PID loop stabilize before starting average frame rate calculation
#define RC_SMOOTHING_FILTER_TRAINING_SAMPLES    50    // Number of rx frame rate samples to average during initial training
#define RC_SMOOTHING_FILTER_RETRAINING_SAMPLES  20    // Number of rx frame rate samples to average during frame rate changes
#define RC_SMOOTHING_FILTER_TRAINING_DELAY_MS   1000  // Additional time to wait after receiving first valid rx frame before initial training starts
#define RC_SMOOTHING_FILTER_RETRAINING_DELAY_MS 2000  // Guard time to wait after retraining to prevent retraining again too quickly
#define RC_SMOOTHING_RX_RATE_CHANGE_PERCENT     20    // Look for samples varying this much from the current detected frame rate to initiate retraining
#define RC_SMOOTHING_SP_DELTA_INITIAL_HZ        100   // The value to use for "auto"

static FAST_DATA_ZERO_INIT rcSmoothingFilter_t rcSmoothingData;
static float rcDeflectionSmoothed[3];

#endif // USE_RC_SMOOTHING_FILTER

#define RC_RX_RATE_MIN_US                       950   // 0.950ms to fit 1kHz without an issue
#define RC_RX_RATE_MAX_US                       65500 // 65.5ms or 15.26hz

float getSetpointRate(int axis)
{
#ifdef USE_RC_SMOOTHING_FILTER
    return setpointRate[axis];
#else
    return rawSetpoint[axis];
#endif
}

float getRcDeflection(int axis)
{
#ifdef USE_RC_SMOOTHING_FILTER
    return rcDeflectionSmoothed[axis];
#else
    return rcDeflection[axis];
#endif
}

float getRcDeflectionAbs(int axis)
{
    return rcDeflectionAbs[axis];
}

void updateRcRefreshRate(timeUs_t currentTimeUs)
{
    static timeUs_t lastRxTimeUs;

    timeDelta_t frameAgeUs;
    timeDelta_t frameDeltaUs = rxGetFrameDelta(&frameAgeUs);

    if (!frameDeltaUs || cmpTimeUs(currentTimeUs, lastRxTimeUs) <= frameAgeUs) {
        frameDeltaUs = cmpTimeUs(currentTimeUs, lastRxTimeUs); // calculate a delta here if not supplied by the protocol
    }

    DEBUG_SET(DEBUG_RX_TIMING, 0, MIN(frameDeltaUs / 10, INT16_MAX));
    DEBUG_SET(DEBUG_RX_TIMING, 1, MIN(frameAgeUs / 10, INT16_MAX));

    lastRxTimeUs = currentTimeUs;
    isRxRateValid = (frameDeltaUs >= RC_RX_RATE_MIN_US && frameDeltaUs <= RC_RX_RATE_MAX_US);
    currentRxRefreshRate = constrain(frameDeltaUs, RC_RX_RATE_MIN_US, RC_RX_RATE_MAX_US);
}

uint16_t getCurrentRxRefreshRate(void)
{
    return currentRxRefreshRate;
}

#ifdef USE_RC_SMOOTHING_FILTER
// Determine a cutoff frequency based on smoothness factor and calculated average rx frame time
FAST_CODE_NOINLINE int calcAutoSmoothingCutoff(int avgRxFrameTimeUs, uint8_t autoSmoothnessFactor)
{
    if (avgRxFrameTimeUs > 0) {
        const float cutoffFactor = 1.5f / (1.0f + (autoSmoothnessFactor / 10.0f));
        float cutoff = (1 / (avgRxFrameTimeUs * 1e-6f));  // link frequency
        cutoff = cutoff * cutoffFactor;
        return lrintf(cutoff);
    } else {
        return 0;
    }
}

// Initialize or update the filters base on either the manually selected cutoff, or
// the auto-calculated cutoff frequency based on detected rx frame rate.
FAST_CODE_NOINLINE void rcSmoothingSetFilterCutoffs(rcSmoothingFilter_t *smoothingData)
{
    const float dT = pidGetDT();
    uint16_t oldCutoff = smoothingData->setpointCutoffFrequency;

    if (smoothingData->setpointCutoffSetting == 0) {
        smoothingData->setpointCutoffFrequency = MAX(RC_SMOOTHING_CUTOFF_MIN_HZ, calcAutoSmoothingCutoff(smoothingData->averageFrameTimeUs, smoothingData->autoSmoothnessFactorSetpoint));
    }
    if (smoothingData->throttleCutoffSetting == 0) {
        smoothingData->throttleCutoffFrequency = MAX(RC_SMOOTHING_CUTOFF_MIN_HZ, calcAutoSmoothingCutoff(smoothingData->averageFrameTimeUs, smoothingData->autoSmoothnessFactorThrottle));
    }

    // initialize or update the Setpoint filter
    if ((smoothingData->setpointCutoffFrequency != oldCutoff) || !smoothingData->filterInitialized) {
        for (int i = 0; i < PRIMARY_CHANNEL_COUNT; i++) {
            if (i < THROTTLE) { // Throttle handled by smoothing rcCommand
                if (!smoothingData->filterInitialized) {
                    pt3FilterInit(&smoothingData->filter[i], pt3FilterGain(smoothingData->setpointCutoffFrequency, dT));
                } else {
                    pt3FilterUpdateCutoff(&smoothingData->filter[i], pt3FilterGain(smoothingData->setpointCutoffFrequency, dT));
                }
            } else {
                if (!smoothingData->filterInitialized) {
                    pt3FilterInit(&smoothingData->filter[i], pt3FilterGain(smoothingData->throttleCutoffFrequency, dT));
                } else {
                    pt3FilterUpdateCutoff(&smoothingData->filter[i], pt3FilterGain(smoothingData->throttleCutoffFrequency, dT));
                }
            }
        }

        // initialize or update the Level filter
        for (int i = FD_ROLL; i < FD_YAW; i++) {
            if (!smoothingData->filterInitialized) {
                pt3FilterInit(&smoothingData->filterDeflection[i], pt3FilterGain(smoothingData->setpointCutoffFrequency, dT));
            } else {
                pt3FilterUpdateCutoff(&smoothingData->filterDeflection[i], pt3FilterGain(smoothingData->setpointCutoffFrequency, dT));
            }
        }
    }

    // update or initialize the Setpoint delta filter
    oldCutoff = smoothingData->setpointDeltaCutoffFrequency;
    if (smoothingData->setpointDeltaCutoffSetting == 0) {
        smoothingData->setpointDeltaCutoffFrequency = MAX(RC_SMOOTHING_CUTOFF_MIN_HZ, calcAutoSmoothingCutoff(smoothingData->averageFrameTimeUs, smoothingData->autoSmoothnessFactorSetpoint));
    }
    if (!smoothingData->filterInitialized) {
        if (smoothingData->setpointDeltaCutoffFrequency > 0) {
            for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
                pt3FilterInit(&smoothingData->setpointDeltaFilter[axis], pt3FilterGain(smoothingData->setpointDeltaCutoffFrequency, dT));
            }
        }
    } else if (smoothingData->setpointDeltaCutoffFrequency != oldCutoff) {
        if (smoothingData->setpointDeltaCutoffFrequency > 0) {
            for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
                pt3FilterUpdateCutoff(&smoothingData->setpointDeltaFilter[axis], pt3FilterGain(smoothingData->setpointDeltaCutoffFrequency, dT));
            }
        }
    }
}

FAST_CODE_NOINLINE void rcSmoothingResetAccumulation(rcSmoothingFilter_t *smoothingData)
{
    smoothingData->training.sum = 0;
    smoothingData->training.count = 0;
    smoothingData->training.min = UINT16_MAX;
    smoothingData->training.max = 0;
}

// Accumulate the rx frame time samples. Once we've collected enough samples calculate the
// average and return true.
static FAST_CODE bool rcSmoothingAccumulateSample(rcSmoothingFilter_t *smoothingData, int rxFrameTimeUs)
{
    smoothingData->training.sum += rxFrameTimeUs;
    smoothingData->training.count++;
    smoothingData->training.max = MAX(smoothingData->training.max, rxFrameTimeUs);
    smoothingData->training.min = MIN(smoothingData->training.min, rxFrameTimeUs);

    // if we've collected enough samples then calculate the average and reset the accumulation
    const int sampleLimit = (rcSmoothingData.filterInitialized) ? RC_SMOOTHING_FILTER_RETRAINING_SAMPLES : RC_SMOOTHING_FILTER_TRAINING_SAMPLES;
    if (smoothingData->training.count >= sampleLimit) {
        smoothingData->training.sum = smoothingData->training.sum - smoothingData->training.min - smoothingData->training.max; // Throw out high and low samples
        smoothingData->averageFrameTimeUs = lrintf(smoothingData->training.sum / (smoothingData->training.count - 2));
        rcSmoothingResetAccumulation(smoothingData);
        return true;
    }
    return false;
}

// Determine if we need to caclulate filter cutoffs. If not then we can avoid
// examining the rx frame times completely
FAST_CODE_NOINLINE bool rcSmoothingAutoCalculate(void)
{
    // if any rc smoothing cutoff is 0 (auto) then we need to calculate cutoffs
    if ((rcSmoothingData.setpointCutoffSetting == 0) || (rcSmoothingData.setpointDeltaCutoffSetting == 0) || (rcSmoothingData.throttleCutoffSetting == 0)) {
        return true;
    }
    return false;
}

static FAST_CODE void processRcSmoothingFilter(void)
{
    static FAST_DATA_ZERO_INIT float rxDataToSmooth[5];
    static FAST_DATA_ZERO_INIT bool initialized;
    static FAST_DATA_ZERO_INIT timeMs_t validRxFrameTimeMs;
    static FAST_DATA_ZERO_INIT bool calculateCutoffs;

    // first call initialization
    if (!initialized) {
        initialized = true;
        rcSmoothingData.filterInitialized = false;
        rcSmoothingData.averageFrameTimeUs = 0;
        rcSmoothingData.autoSmoothnessFactorSetpoint = rxConfig()->rc_smoothing_auto_factor_rpy;
        rcSmoothingData.autoSmoothnessFactorThrottle = rxConfig()->rc_smoothing_auto_factor_throttle;
        rcSmoothingData.debugAxis = rxConfig()->rc_smoothing_debug_axis;
        rcSmoothingData.setpointCutoffSetting = rxConfig()->rc_smoothing_setpoint_cutoff;
        rcSmoothingData.throttleCutoffSetting = rxConfig()->rc_smoothing_throttle_cutoff;
        rcSmoothingData.setpointDeltaCutoffSetting = rxConfig()->rc_smoothing_setpoint_delta_cutoff;
        rcSmoothingResetAccumulation(&rcSmoothingData);
        rcSmoothingData.setpointCutoffFrequency = rcSmoothingData.setpointCutoffSetting;
        rcSmoothingData.throttleCutoffFrequency = rcSmoothingData.throttleCutoffSetting;
        if (rcSmoothingData.setpointDeltaCutoffSetting == 0) {
            // calculate and use an initial derivative cutoff until the RC interval is known
            const float cutoffFactor = 1.5f / (1.0f + (rcSmoothingData.autoSmoothnessFactorSetpoint / 10.0f));
            float ffCutoff = RC_SMOOTHING_SP_DELTA_INITIAL_HZ * cutoffFactor;
            rcSmoothingData.setpointDeltaCutoffFrequency = lrintf(ffCutoff);
        } else {
            rcSmoothingData.setpointDeltaCutoffFrequency = rcSmoothingData.setpointDeltaCutoffSetting;
        }

        if (rxConfig()->rc_smoothing_mode) {
            calculateCutoffs = rcSmoothingAutoCalculate();

            // if we don't need to calculate cutoffs dynamically then the filters can be initialized now
            if (!calculateCutoffs) {
                rcSmoothingSetFilterCutoffs(&rcSmoothingData);
                rcSmoothingData.filterInitialized = true;
            }
        }
    }

    if (isRxDataNew) {
        // for auto calculated filters we need to examine each rx frame interval
        if (calculateCutoffs) {
            const timeMs_t currentTimeMs = millis();
            int sampleState = 0;

            // If the filter cutoffs in auto mode, and we have good rx data, then determine the average rx frame rate
            // and use that to calculate the filter cutoff frequencies
            if ((currentTimeMs > RC_SMOOTHING_FILTER_STARTUP_DELAY_MS) && (gyro.targetLooptime > 0)) { // skip during FC initialization
                if (rxIsReceivingSignal() && isRxRateValid) {

                    // set the guard time expiration if it's not set
                    if (validRxFrameTimeMs == 0) {
                        validRxFrameTimeMs = currentTimeMs + (rcSmoothingData.filterInitialized ? RC_SMOOTHING_FILTER_RETRAINING_DELAY_MS : RC_SMOOTHING_FILTER_TRAINING_DELAY_MS);
                    } else {
                        sampleState = 1;
                    }

                    // if the guard time has expired then process the rx frame time
                    if (currentTimeMs > validRxFrameTimeMs) {
                        sampleState = 2;
                        bool accumulateSample = true;

                        // During initial training process all samples.
                        // During retraining check samples to determine if they vary by more than the limit percentage.
                        if (rcSmoothingData.filterInitialized) {
                            const float percentChange = (ABS(currentRxRefreshRate - rcSmoothingData.averageFrameTimeUs) / (float)rcSmoothingData.averageFrameTimeUs) * 100;
                            if (percentChange < RC_SMOOTHING_RX_RATE_CHANGE_PERCENT) {
                                // We received a sample that wasn't more than the limit percent so reset the accumulation
                                // During retraining we need a contiguous block of samples that are all significantly different than the current average
                                rcSmoothingResetAccumulation(&rcSmoothingData);
                                accumulateSample = false;
                            }
                        }

                        // accumlate the sample into the average
                        if (accumulateSample) {
                            if (rcSmoothingAccumulateSample(&rcSmoothingData, currentRxRefreshRate)) {
                                // the required number of samples were collected so set the filter cutoffs, but only if smoothing is active
                                if (rxConfig()->rc_smoothing_mode) {
                                    rcSmoothingSetFilterCutoffs(&rcSmoothingData);
                                    rcSmoothingData.filterInitialized = true;
                                }
                                validRxFrameTimeMs = 0;
                            }
                        }

                    }
                } else {
                    // we have either stopped receiving rx samples (failsafe?) or the sample time is unreasonable so reset the accumulation
                    rcSmoothingResetAccumulation(&rcSmoothingData);
                }
            }

            // rx frame rate training blackbox debugging
            DEBUG_SET(DEBUG_RC_SMOOTHING_RATE, 0, currentRxRefreshRate);              // log each rx frame interval
            DEBUG_SET(DEBUG_RC_SMOOTHING_RATE, 1, rcSmoothingData.training.count);    // log the training step count
            DEBUG_SET(DEBUG_RC_SMOOTHING_RATE, 2, rcSmoothingData.averageFrameTimeUs);// the current calculated average
            DEBUG_SET(DEBUG_RC_SMOOTHING_RATE, 3, sampleState);                       // indicates whether guard time is active
        }
        // Get new values to be smoothed
        for (int i = 0; i < 4; i++) {
            rxDataToSmooth[i] = rawSetpoint[i];
            DEBUG_SET(DEBUG_RC_INTERPOLATION, i, lrintf(rxDataToSmooth[i]));
        }
        rxDataToSmooth[THROTTLE] = rcCommand[THROTTLE];
    }

    if (rcSmoothingData.filterInitialized && (debugMode == DEBUG_RC_SMOOTHING)) {
        // after training has completed then log the raw rc channel and the calculated
        // average rx frame rate that was used to calculate the automatic filter cutoffs
        DEBUG_SET(DEBUG_RC_SMOOTHING, 3, rcSmoothingData.averageFrameTimeUs);
    }

    // each pid loop, apply the last received channel value to the filter, if initialised - thanks @klutvott
    if (rcSmoothingData.filterInitialized) {
        for (int i = 0; i < 4; i++) {
            setpointRate[i] = pt3FilterApply(&rcSmoothingData.filter[i], rxDataToSmooth[i]);
        }
        rcCommand[THROTTLE] = pt3FilterApply(&rcSmoothingData.filter[THROTTLE], rxDataToSmooth[THROTTLE]);
    } else {
        // If filter isn't initialized yet, as in smoothing off, use the actual unsmoothed rx channel data
        for (int i = 0; i < 4; i++) {
            setpointRate[i] = rxDataToSmooth[i];
        }
        rcCommand[THROTTLE] = rxDataToSmooth[THROTTLE];
    }

    // for ANGLE and HORIZON, smooth rcDeflection on pitch and roll to avoid setpoint steps
    bool smoothingNeeded = (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE)) && rcSmoothingData.filterInitialized;
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        if (smoothingNeeded && axis < FD_YAW) {
            rcDeflectionSmoothed[axis] = pt3FilterApply(&rcSmoothingData.filterDeflection[axis], rcDeflection[axis]);
        } else {
            rcDeflectionSmoothed[axis] = rcDeflection[axis];
        }
    }

}

FAST_CODE float rcSmoothingApplySetpointDeltaFilter(int axis, float pidSetpointDelta)
{
    float ret = pidSetpointDelta;
    if (axis == rcSmoothingData.debugAxis) {
        DEBUG_SET(DEBUG_RC_SMOOTHING, 1, lrintf(pidSetpointDelta * 100.0f));
    }
    if (rcSmoothingData.filterInitialized) {
        ret = pt3FilterApply(&rcSmoothingData.setpointDeltaFilter[axis], pidSetpointDelta);
        if (axis == rcSmoothingData.debugAxis) {
            DEBUG_SET(DEBUG_RC_SMOOTHING, 2, lrintf(ret * 100.0f));
        }
    }
    return ret;
}

#endif // USE_RC_SMOOTHING_FILTER

FAST_CODE void processRcCommand(void)
{
    if (isRxDataNew) {
        for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
            float angleRate;
            
#ifdef USE_GPS_RESCUE
            if ((axis == FD_YAW) && FLIGHT_MODE(GPS_RESCUE_MODE)) {
                // If GPS Rescue is active then override the setpointRate used in the
                // pid controller with the value calculated from the desired heading logic.
                angleRate = gpsRescueGetYawRate();
                // Treat the stick input as centered to avoid any stick deflection base modifications (like acceleration limit)
                rcDeflection[axis] = 0;
                rcDeflectionAbs[axis] = 0;
            } else
#endif
            {
                // scale rcCommandf to range [-1.0, 1.0]
                float rcCommandf;
                if (axis == FD_YAW) {
                    rcCommandf = rcCommand[axis] / rcCommandYawDivider;
                } else {
                    rcCommandf = rcCommand[axis] / rcCommandDivider;
                }

                rcDeflection[axis] = rcCommandf;
                const float rcCommandfAbs = fabsf(rcCommandf);
                rcDeflectionAbs[axis] = rcCommandfAbs;

                angleRate = applyRates(axis, rcCommandf, rcCommandfAbs);

            }
            rawSetpoint[axis] = constrainf(angleRate, -1.0f * currentControlRateProfile->rate_limit[axis], 1.0f * currentControlRateProfile->rate_limit[axis]);
            DEBUG_SET(DEBUG_ANGLERATE, axis, angleRate);
        }
    }

#ifdef USE_RC_SMOOTHING_FILTER
    processRcSmoothingFilter();
#endif

    isRxDataNew = false;
}

FAST_CODE_NOINLINE void updateRcCommands(void)
{
    isRxDataNew = true;

    for (int axis = 0; axis < 4; axis++) {
        float tmp = MIN(ABS(rcData[axis] - rxConfig()->midrc), 500);
        if (axis == ROLL || axis == PITCH) {
            if (tmp > rcControlsConfig()->deadband) {
                tmp -= rcControlsConfig()->deadband;
            } else {
                tmp = 0;
            }
            rcCommand[axis] = tmp;
        } else if (axis == YAW) {
            if (tmp > rcControlsConfig()->yaw_deadband) {
                tmp -= rcControlsConfig()->yaw_deadband;
            } else {
                tmp = 0;
            }
            rcCommand[axis] = tmp * -GET_DIRECTION(rcControlsConfig()->yaw_control_reversed);
        } else {
            rcCommand[axis] = tmp;
        }
        if (rcData[axis] < rxConfig()->midrc) {
            rcCommand[axis] = -rcCommand[axis];
        }
    }

    rcCommand[THROTTLE] = constrain(rcData[THROTTLE], PWM_RANGE_MIN, PWM_RANGE_MAX) - PWM_RANGE_MIN;
}

void resetYawAxis(void)
{
    rcCommand[YAW] = 0;
    setpointRate[YAW] = 0;
}

void initRcProcessing(void)
{
    rcCommandDivider = 500.0f - rcControlsConfig()->deadband;
    rcCommandYawDivider = 500.0f - rcControlsConfig()->yaw_deadband;
}

// send rc smoothing details to blackbox
#ifdef USE_RC_SMOOTHING_FILTER
rcSmoothingFilter_t *getRcSmoothingData(void)
{
    return &rcSmoothingData;
}

bool rcSmoothingInitializationComplete(void) {
    return rcSmoothingData.filterInitialized;
}
#endif // USE_RC_SMOOTHING_FILTER
