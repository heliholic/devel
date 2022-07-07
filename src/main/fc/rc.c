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
#include "common/maths.h"

#include "config/config.h"
#include "config/feature.h"

#include "fc/runtime_config.h"
#include "fc/core.h"

#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/rc_rates.h"
#include "fc/rc_smoothing.h"

#include "pg/rx.h"

#include "rx/rx.h"

#include "rc.h"


#define RC_RX_RATE_MIN_US       950    // 0.950ms to fit 1kHz without an issue
#define RC_RX_RATE_MAX_US       65500  // 65.5ms or 15.26hz


FAST_DATA_ZERO_INIT float rcCommand[5];                  // -500..+500 for RPYC and 0..1000 for THROTTLE
FAST_DATA_ZERO_INIT float rcDeflection[5];               // -1..1 for RPYC, 0..1 for THROTTLE

static FAST_DATA_ZERO_INIT float rawSetpoint[4];
static FAST_DATA_ZERO_INIT float smoothSetpoint[4];

static FAST_DATA_ZERO_INIT float rcDivider[4]   = { 500, 500, 500, 500 };
static FAST_DATA_ZERO_INIT float rcDeadband[4]  = {   0,   0,   0,   0 };

static FAST_DATA_ZERO_INIT bool isRxRateValid;

static FAST_DATA_ZERO_INIT timeUs_t lastRxTimeUs;
static FAST_DATA_ZERO_INIT uint16_t currentRxRefreshRate;


void resetYawAxis(void)
{
    rcCommand[YAW] = 0;
    rcDeflection[YAW] = 0;
    rawSetpoint[YAW] =  0;
    smoothSetpoint[YAW] = 0;
}

float getRawSetpoint(int axis)
{
    return rawSetpoint[axis];
}

float getRcSetpoint(int axis)
{
    return smoothSetpoint[axis];
}

float getRcDeflection(int axis)
{
    return rcDeflection[axis];
}


uint16_t getCurrentRxRefreshRate(void)
{
    return currentRxRefreshRate;
}

void updateRcRefreshRate(timeUs_t currentTimeUs)
{
    timeDelta_t frameAgeUs;
    timeDelta_t frameDeltaUs = rxGetFrameDelta(&frameAgeUs);
    timeDelta_t localDeltaUs = cmpTimeUs(currentTimeUs, lastRxTimeUs);

    if (frameDeltaUs == 0 || localDeltaUs <= frameAgeUs) {
        frameDeltaUs = localDeltaUs;
    }

    DEBUG_SET(DEBUG_RX_TIMING, 0, MIN(frameDeltaUs / 10, INT16_MAX));
    DEBUG_SET(DEBUG_RX_TIMING, 1, MIN(localDeltaUs / 10, INT16_MAX));
    DEBUG_SET(DEBUG_RX_TIMING, 2, MIN(frameAgeUs / 10, INT16_MAX));

    isRxRateValid = (frameDeltaUs >= RC_RX_RATE_MIN_US && frameDeltaUs <= RC_RX_RATE_MAX_US);
    currentRxRefreshRate = constrain(frameDeltaUs, RC_RX_RATE_MIN_US, RC_RX_RATE_MAX_US);

    lastRxTimeUs = currentTimeUs;
}


static inline float deadband(float x, float deadband)
{
    if (x > deadband)
        return x - deadband;
    else if (x < -deadband)
        return x + deadband;
    else
        return 0;
}

FAST_CODE void updateRcCommands(void)
{
#ifdef USE_RC_SMOOTHING_FILTER
    rcSmoothingFilterUpdate(isRxRateValid, currentRxRefreshRate);
#endif

    // rcData => rcCommand => rcDeflection
    for (int axis = 0; axis < 4; axis++) {
        float data = rcData[axis] - rxConfig()->midrc;
        data = deadband(data, rcDeadband[axis]);
        data = constrainf(data, -500, 500);
        rcCommand[axis] = data;
        rcDeflection[axis] = data / rcDivider[axis];
    }

    rcCommand[YAW] *= -GET_DIRECTION(rcControlsConfig()->yaw_control_reversed);

    // rcDeflection => rawSetpoint
    for (int axis = 0; axis < 4; axis++) {
        rawSetpoint[axis] = applyRatesCurve(axis, rcDeflection[axis]);
        DEBUG_SET(DEBUG_ANGLERATE, axis, rawSetpoint[axis]);
    }

    // FIXME
    rcCommand[THROTTLE] = constrain(rcData[THROTTLE], PWM_RANGE_MIN, PWM_RANGE_MAX) - PWM_RANGE_MIN;
    rcDeflection[THROTTLE] = rcCommand[THROTTLE] / PWM_RANGE;
}

FAST_CODE void processRcCommand(void)
{
    // rawSetpoint => smoothSetpoint
    for (int axis = 0; axis < 4; axis++) {
#ifdef USE_RC_SMOOTHING_FILTER
        smoothSetpoint[axis] = rcSmoothingFilterApply(axis, rawSetpoint[axis]);
#else
        smoothSetpoint[axis] = rawSetpoint[axis];
#endif
    }
}

INIT_CODE void initRcProcessing(void)
{
    rcDivider[0] = 500 - rcControlsConfig()->deadband;
    rcDivider[1] = 500 - rcControlsConfig()->deadband;
    rcDivider[2] = 500 - rcControlsConfig()->yaw_deadband;

    rcDeadband[0] = rcControlsConfig()->deadband;
    rcDeadband[1] = rcControlsConfig()->deadband;
    rcDeadband[2] = rcControlsConfig()->yaw_deadband;

#ifdef USE_RC_SMOOTHING_FILTER
    rcSmoothingFilterInit();
#endif
}

