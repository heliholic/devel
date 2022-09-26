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

#include "flight/pid.h"
#include "flight/setpoint.h"

#include "pg/rx.h"

#include "rx/rx.h"

#include "rc.h"


#define RX_REFRESH_RATE_MIN_US          950
#define RX_REFRESH_RATE_MAX_US        65000

#define RX_REFRESH_RATE_AVERAGING       100


FAST_DATA_ZERO_INIT float rcCommand[5];                  // -500..+500 for RPYC and 0..1000 for THROTTLE
FAST_DATA_ZERO_INIT float rcDeflection[5];               // -1..1 for RPYC, 0..1 for THROTTLE

static FAST_DATA_ZERO_INIT float rawSetpoint[4];

static FAST_DATA_ZERO_INIT float rcDivider[4];
static FAST_DATA_ZERO_INIT float rcDeadband[4];

static FAST_DATA_ZERO_INIT uint16_t currentRxRefreshRate;
static FAST_DATA_ZERO_INIT float    averageRxRefreshRate;
static FAST_DATA_ZERO_INIT uint16_t averageLength;
static FAST_DATA_ZERO_INIT timeUs_t lastRxTimeUs;


void resetYawAxis(void)
{
    rcCommand[YAW] = 0;
    rcDeflection[YAW] = 0;
    rawSetpoint[YAW] =  0;
}

float getRawSetpoint(int axis)
{
    return rawSetpoint[axis];
}

float getRcDeflection(int axis)
{
    return rcDeflection[axis];
}


uint16_t getCurrentRxRefreshRate(void)
{
    return currentRxRefreshRate;
}

float getAverageRxRefreshRate(void)
{
    return averageRxRefreshRate;
}


void updateRcRefreshRate(timeUs_t currentTimeUs)
{
    timeDelta_t frameAgeUs = 0;
    timeDelta_t frameDeltaUs = rxGetFrameDelta(&frameAgeUs);
    timeDelta_t localDeltaUs = cmpTimeUs(currentTimeUs, lastRxTimeUs);

    DEBUG(RX_TIMING, 4, frameDeltaUs);
    DEBUG(RX_TIMING, 5, localDeltaUs);
    DEBUG(RX_TIMING, 6, frameAgeUs);

    if (frameDeltaUs == 0 || localDeltaUs <= frameAgeUs) {
        frameDeltaUs = localDeltaUs;
    }

    currentRxRefreshRate = frameDeltaUs;
    lastRxTimeUs = currentTimeUs;

    float currentRateUs = frameDeltaUs;

    if (averageLength >= RX_REFRESH_RATE_AVERAGING) {
        if (rxIsReceivingSignal() && currentRxRefreshRate > RX_REFRESH_RATE_MIN_US && currentRxRefreshRate < RX_REFRESH_RATE_MAX_US) {
            currentRateUs = constrainf(currentRateUs, 0.75f * averageRxRefreshRate, 1.25f * averageRxRefreshRate);
        } else {
            currentRateUs = averageRxRefreshRate;
        }
    }
    else {
        averageLength++;
    }

    averageRxRefreshRate += (currentRateUs - averageRxRefreshRate) / averageLength;

    DEBUG(RX_TIMING, 0, averageRxRefreshRate);
    DEBUG(RX_TIMING, 1, currentRxRefreshRate);
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
    float data;

    setpointFilterUpdate(currentRxRefreshRate);

    // rcData => rcCommand => rcDeflection
    for (int axis = 0; axis < 4; axis++) {
        data = rcData[axis] - rxConfig()->midrc;
        data = deadband(data, rcDeadband[axis]);
        data = constrainf(data, -500, 500);
        rcCommand[axis] = data;
        rcDeflection[axis] = data / rcDivider[axis];
        DEBUG(RC_COMMAND, axis, data);
    }

    if (rcControlsConfig()->yaw_control_reversed)
        rcCommand[YAW] = -rcCommand[YAW];

    // rcDeflection => rawSetpoint
    for (int axis = 0; axis < 4; axis++) {
        data = applyRatesCurve(axis, rcDeflection[axis]);
        rawSetpoint[axis] = data;
        DEBUG(RC_SETPOINT, axis, data);
    }

    // RF FIXME
    data = constrain(rcData[THROTTLE], PWM_RANGE_MIN, PWM_RANGE_MAX) - PWM_RANGE_MIN;
    rcCommand[THROTTLE] = data;
    rcDeflection[THROTTLE] = data / PWM_RANGE;
}

INIT_CODE void initRcProcessing(void)
{
    rcDivider[0] = 500 - rcControlsConfig()->deadband;
    rcDivider[1] = 500 - rcControlsConfig()->deadband;
    rcDivider[2] = 500 - rcControlsConfig()->yaw_deadband;
    rcDivider[3] = 500;

    rcDeadband[0] = rcControlsConfig()->deadband;
    rcDeadband[1] = rcControlsConfig()->deadband;
    rcDeadband[2] = rcControlsConfig()->yaw_deadband;
    rcDeadband[3] = 0;
}

