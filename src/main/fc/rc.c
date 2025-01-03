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
#include <stdlib.h>
#include <math.h>

#include "platform.h"

#include "build/debug.h"

#include "common/utils.h"
#include "common/vector.h"

#include "config/config.h"
#include "config/feature.h"

#include "fc/controlrate_profile.h"
#include "fc/core.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/gps_rescue.h"
#include "flight/pid.h"

#include "pg/rx.h"
#include "rx/rx.h"

#include "sensors/battery.h"
#include "sensors/gyro.h"

#include "rc.h"

#define RX_INTERVAL_MIN_US     800 // 0.800ms to fit 1kHz without an issue often 1khz rc comes in at 880us or so
#define RX_INTERVAL_MAX_US   65500 // 65.5ms or 15.26hz

typedef float (applyRatesFn)(const int axis, float rcCommandf, const float rcCommandfAbs);
// note that rcCommand[] is an external float

static float rawSetpoint[XYZ_AXIS_COUNT];

static float setpointRate[3], rcDeflection[3], rcDeflectionAbs[3]; // deflection range -1 to 1
static float maxRcDeflectionAbs;

static applyRatesFn *applyRates;

static uint16_t currentRxIntervalUs;  // packet interval in microseconds, constrained to above range
static uint16_t previousRxIntervalUs; // previous packet interval in microseconds
static float currentRxRateHz;         // packet interval in Hz, constrained as above
static float smoothedRxRateHz = 100.0f;

static bool isRxDataNew = false;
static bool isRxRateValid = false;
static float rcCommandDivider = 500.0f;
static float rcCommandYawDivider = 500.0f;

enum {
    ROLL_FLAG = 1 << ROLL,
    PITCH_FLAG = 1 << PITCH,
    YAW_FLAG = 1 << YAW,
    THROTTLE_FLAG = 1 << THROTTLE,
};

float getSetpointRate(int axis)
{
    return rawSetpoint[axis];
}

static float maxRcRate[3];
float getMaxRcRate(int axis)
{
    return maxRcRate[axis];
}

float getRcDeflection(int axis)
{
    return rcDeflection[axis];
}

float getRcDeflectionRaw(int axis)
{
    return rcDeflection[axis];
}

float getRcDeflectionAbs(int axis)
{
    return rcDeflectionAbs[axis];
}

float getMaxRcDeflectionAbs(void)
{
    return maxRcDeflectionAbs;
}

#ifndef THROTTLE_LOOKUP_LENGTH
# define THROTTLE_LOOKUP_LENGTH 12
#endif
static int16_t lookupThrottleRC[THROTTLE_LOOKUP_LENGTH];    // lookup table for expo & mid THROTTLE

static int16_t rcLookupThrottle(int32_t tmp)
{
    // tmp is 0…PWM_RANGE
    // Spread that range evenly over THROTTLE_LOOKUP_LENGTH-1 steps
    const int32_t steps  = THROTTLE_LOOKUP_LENGTH - 1;
    const int32_t scaled = tmp * steps;                // 0…PWM_RANGE*steps
    const int32_t idx    = scaled / PWM_RANGE;         // 0…steps
    const int32_t rem    = scaled % PWM_RANGE;         // for interpolation

    // If index goes outside the valid range, clamp it
    if (idx >= steps) {
        return lookupThrottleRC[steps];
    } else if (idx < 0) {
        return lookupThrottleRC[0];
    }

    // Otherwise linearly interpolate between lookupThrottleRC[idx] and [idx+1]
    return scaleRange(rem, 0, PWM_RANGE, lookupThrottleRC[idx], lookupThrottleRC[idx + 1]);
}

#define SETPOINT_RATE_LIMIT_MIN -1998.0f
#define SETPOINT_RATE_LIMIT_MAX 1998.0f
STATIC_ASSERT(CONTROL_RATE_CONFIG_RATE_LIMIT_MAX <= (uint16_t)SETPOINT_RATE_LIMIT_MAX, CONTROL_RATE_CONFIG_RATE_LIMIT_MAX_too_large);

#define RC_RATE_INCREMENTAL 14.54f

static float applyBetaflightRates(const int axis, float rcCommandf, const float rcCommandfAbs)
{
    if (currentControlRateProfile->rcExpo[axis]) {
        const float expof = currentControlRateProfile->rcExpo[axis] / 100.0f;
        rcCommandf = rcCommandf * power3(rcCommandfAbs) * expof + rcCommandf * (1 - expof);
    }

    float rcRate = currentControlRateProfile->rcRates[axis] / 100.0f;
    if (rcRate > 2.0f) {
        rcRate += RC_RATE_INCREMENTAL * (rcRate - 2.0f);
    }
    float angleRate = 200.0f * rcRate * rcCommandf;
    if (currentControlRateProfile->rates[axis]) {
        const float rcSuperfactor = 1.0f / (constrainf(1.0f - (rcCommandfAbs * (currentControlRateProfile->rates[axis] / 100.0f)), 0.01f, 1.00f));
        angleRate *= rcSuperfactor;
    }

    return angleRate;
}

static float applyRaceFlightRates(const int axis, float rcCommandf, const float rcCommandfAbs)
{
    // -1.0 to 1.0 ranged and curved
    rcCommandf = ((1.0f + 0.01f * currentControlRateProfile->rcExpo[axis] * (rcCommandf * rcCommandf - 1.0f)) * rcCommandf);
    // convert to -2000 to 2000 range using acro+ modifier
    float angleRate = 10.0f * currentControlRateProfile->rcRates[axis] * rcCommandf;
    angleRate = angleRate * (1 + rcCommandfAbs * (float)currentControlRateProfile->rates[axis] * 0.01f);

    return angleRate;
}

static float applyKissRates(const int axis, float rcCommandf, const float rcCommandfAbs)
{
    const float rcCurvef = currentControlRateProfile->rcExpo[axis] / 100.0f;

    float kissRpyUseRates = 1.0f / (constrainf(1.0f - (rcCommandfAbs * (currentControlRateProfile->rates[axis] / 100.0f)), 0.01f, 1.00f));
    float kissRcCommandf = (power3(rcCommandf) * rcCurvef + rcCommandf * (1 - rcCurvef)) * (currentControlRateProfile->rcRates[axis] / 1000.0f);
    float kissAngle = constrainf(((2000.0f * kissRpyUseRates) * kissRcCommandf), SETPOINT_RATE_LIMIT_MIN, SETPOINT_RATE_LIMIT_MAX);

    return kissAngle;
}

static float applyActualRates(const int axis, float rcCommandf, const float rcCommandfAbs)
{
    float expof = currentControlRateProfile->rcExpo[axis] / 100.0f;
    expof = rcCommandfAbs * (power5(rcCommandf) * expof + rcCommandf * (1 - expof));

    const float centerSensitivity = currentControlRateProfile->rcRates[axis] * 10.0f;
    const float stickMovement = MAX(0, currentControlRateProfile->rates[axis] * 10.0f - centerSensitivity);
    const float angleRate = rcCommandf * centerSensitivity + stickMovement * expof;

    return angleRate;
}

static float applyQuickRates(const int axis, float rcCommandf, const float rcCommandfAbs)
{
    const uint16_t rcRate = currentControlRateProfile->rcRates[axis] * 2;
    const uint16_t maxDPS = MAX(currentControlRateProfile->rates[axis] * 10, rcRate);
    const float expof = currentControlRateProfile->rcExpo[axis] / 100.0f;
    const float superFactorConfig = ((float)maxDPS / rcRate - 1) / ((float)maxDPS / rcRate);

    float curve;
    float superFactor;
    float angleRate;

    if (currentControlRateProfile->quickRatesRcExpo) {
        curve = power3(rcCommandf) * expof + rcCommandf * (1 - expof);
        superFactor = 1.0f / (constrainf(1.0f - (rcCommandfAbs * superFactorConfig), 0.01f, 1.00f));
        angleRate = constrainf(curve * rcRate * superFactor, SETPOINT_RATE_LIMIT_MIN, SETPOINT_RATE_LIMIT_MAX);
    } else {
        curve = power3(rcCommandfAbs) * expof + rcCommandfAbs * (1 - expof);
        superFactor = 1.0f / (constrainf(1.0f - (curve * superFactorConfig), 0.01f, 1.00f));
        angleRate = constrainf(rcCommandf * rcRate * superFactor, SETPOINT_RATE_LIMIT_MIN, SETPOINT_RATE_LIMIT_MAX);
    }

    return angleRate;
}

void updateRcRefreshRate(timeUs_t currentTimeUs, bool rxReceivingSignal)
{
    // this function runs from processRx in core.c
    // rxReceivingSignal is true:
    // - every time a new frame is detected,
    // - if we stop getting data, at the expiry of RXLOSS_TRIGGER_INTERVAL since the last good frame
    // - if that interval is exceeded and still no data, every RX_FRAME_RECHECK_INTERVAL, until a new frame is detected
    static timeUs_t lastRxTimeUs = 0;
    timeDelta_t delta = 0;

    if (rxReceivingSignal) { // true while receiving data and until RXLOSS_TRIGGER_INTERVAL expires, otherwise false
        previousRxIntervalUs = currentRxIntervalUs;
        // use driver rx time if available, current time otherwise
        const timeUs_t rxTime = rxRuntimeState.lastRcFrameTimeUs ? rxRuntimeState.lastRcFrameTimeUs : currentTimeUs;

        if (lastRxTimeUs) {  // report delta only if previous time is available
            delta = cmpTimeUs(rxTime, lastRxTimeUs);
        }
        lastRxTimeUs = rxTime;
        DEBUG_SET(DEBUG_RX_TIMING, 1, rxTime / 100);  // packet time stamp in tenths of ms
    } else {
        if (lastRxTimeUs) {
            // no packet received, use current time for delta
            delta = cmpTimeUs(currentTimeUs, lastRxTimeUs);
        }
    }

    // constrain to a frequency range no lower than about 15Hz and up to about 1000Hz
    // these intervals and rates will be used for RCSmoothing, Feedforward, etc.
    currentRxIntervalUs = constrain(delta, RX_INTERVAL_MIN_US, RX_INTERVAL_MAX_US);
    currentRxRateHz = 1e6f / currentRxIntervalUs;
    isRxRateValid = delta == currentRxIntervalUs; // delta is not constrained, therefore not outside limits

    DEBUG_SET(DEBUG_RX_TIMING, 0, MIN(delta / 10, INT16_MAX));   // packet interval in hundredths of ms
    DEBUG_SET(DEBUG_RX_TIMING, 2, isRxRateValid);
    DEBUG_SET(DEBUG_RX_TIMING, 3, MIN(currentRxIntervalUs / 10, INT16_MAX));  // constrained packet interval, tenths of ms
    DEBUG_SET(DEBUG_RX_TIMING, 4, lrintf(currentRxRateHz));
    // temporary debugs
#ifdef USE_RX_LINK_QUALITY_INFO
    DEBUG_SET(DEBUG_RX_TIMING, 6, rxGetLinkQualityPercent());    // raw link quality value
#endif
    DEBUG_SET(DEBUG_RX_TIMING, 7, isRxReceivingSignal());        // flag to initiate RXLOSS signal and Stage 1 values
}

// currently only used in the CLI
float getCurrentRxRateHz(void)
{
    return smoothedRxRateHz;
}

bool getRxRateValid(void)
{
    return isRxRateValid;
}

bool shouldUpdateSmoothing(void)
{
    static int validCount = 0;
    static int outlierCount = 0;
    static const float smoothingFactor = 0.1f;       // Low pass smoothing factor to smooth valid RxRate values
    static int8_t prevOutlierSign = 0;               // -1 for negative, +1 for positive

    if (isRxReceivingSignal() && isRxRateValid) {
        float deltaRateHz = currentRxRateHz - smoothedRxRateHz;
        bool isOutlier = fabsf(deltaRateHz) > (smoothedRxRateHz * 0.2f);

        if (isOutlier) {
            const int8_t currentSign = (deltaRateHz < 0.0f) ? -1 : 1;
            if (outlierCount == 0) {
                prevOutlierSign = currentSign;
                outlierCount++;
            } else {
                if (currentSign != prevOutlierSign) {
                    // Reset outlier count if outlier sign reverses, as often happens at 1000Hz
                    // with a true change, all new packet delta will have the same sign.
                    outlierCount = 0;
                    prevOutlierSign = currentSign;
                } else {
                    outlierCount++;
                }
            }
            validCount = 0;
        } else {
            // First-order smoothing toward new value for non-outliers
            smoothedRxRateHz += smoothingFactor * (currentRxRateHz - smoothedRxRateHz);
            validCount++;
            outlierCount = 0;
        }

        if (validCount >= 3) {
            validCount = 0;
            // indicate that filter cutoffs should be updated to smoothedRxRateHz
            return true;
        }
        if (outlierCount >= 3) {
            // Link rate likely changed — snap smoothing accumulator to current value
            smoothedRxRateHz = currentRxRateHz;
            outlierCount = 0;
        }
    } else {
        // Signal lost or invalid widths, reset counts, but hold last stable and smoothed values
        validCount = 0;
        outlierCount = 0;
    }
    return false;
}

FAST_CODE void processRcCommand(void)
{
    if (isRxDataNew) {
        maxRcDeflectionAbs = 0.0f;

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
                maxRcDeflectionAbs = fmaxf(maxRcDeflectionAbs, rcCommandfAbs);

                angleRate = applyRates(axis, rcCommandf, rcCommandfAbs);
            }

            rawSetpoint[axis] = constrainf(angleRate, -1.0f * currentControlRateProfile->rate_limit[axis], 1.0f * currentControlRateProfile->rate_limit[axis]);
            DEBUG_SET(DEBUG_ANGLERATE, axis, angleRate);

            // log the smoothed Rx Rate from non-outliers, this will not show the steps every three valid packets
            DEBUG_SET(DEBUG_RX_TIMING, 5, lrintf(smoothedRxRateHz));
        }
    }

    isRxDataNew = false;
}

FAST_CODE_NOINLINE void updateRcCommands(void)
{
    isRxDataNew = true;

    for (int axis = 0; axis < 3; axis++) {
        float rc = constrainf(rcData[axis] - rxConfig()->midrc, -500.0f, 500.0f); // -500 to 500
        float rcDeadband = 0;
        if (axis == ROLL || axis == PITCH) {
            rcDeadband = rcControlsConfig()->deadband;
        } else {
            rcDeadband  = rcControlsConfig()->yaw_deadband;
            rc = -rc;  // Yaw direction reversed
        }
        rcCommand[axis] = fapplyDeadband(rc, rcDeadband);
    }

    int32_t tmp;
    tmp = constrain(rcData[THROTTLE], rxConfig()->mincheck, PWM_RANGE_MAX);
    tmp = (uint32_t)(tmp - rxConfig()->mincheck) * PWM_RANGE_MIN / (PWM_RANGE_MAX - rxConfig()->mincheck);

    if (getLowVoltageCutoff()->enabled) {
        tmp = tmp * getLowVoltageCutoff()->percentage / 100;
    }

    rcCommand[THROTTLE] = rcLookupThrottle(tmp);
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

    switch (currentControlRateProfile->rates_type) {
    case RATES_TYPE_BETAFLIGHT:
    default:
        applyRates = applyBetaflightRates;
        break;
    case RATES_TYPE_RACEFLIGHT:
        applyRates = applyRaceFlightRates;
        break;
    case RATES_TYPE_KISS:
        applyRates = applyKissRates;
        break;
    case RATES_TYPE_ACTUAL:
        applyRates = applyActualRates;
        break;
    case RATES_TYPE_QUICK:
        applyRates = applyQuickRates;
        break;
    }

    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
        maxRcRate[i] = applyRates(i, 1.0f, 1.0f);
    }
}
