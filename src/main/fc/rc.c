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
#include "fc/rc_smoothing.h"

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


#define RC_RX_RATE_MIN_US       950    // 0.950ms to fit 1kHz without an issue
#define RC_RX_RATE_MAX_US       65500  // 65.5ms or 15.26hz


FAST_DATA_ZERO_INIT float rcCommand[5];                  // -500..+500 for RPYC and 0..1000 for THROTTLE
FAST_DATA_ZERO_INIT float rcDeflection[5];               // -1..1 for RPYC, 0..1 for THROTTLE

static FAST_DATA_ZERO_INIT float rcDivider[4];
static FAST_DATA_ZERO_INIT float rcDeadband[4];

static FAST_DATA_ZERO_INIT float rawSetpoint[4];
static FAST_DATA_ZERO_INIT float smoothSetpoint[4];

static FAST_DATA_ZERO_INIT bool isRxDataNew = false;
static FAST_DATA_ZERO_INIT bool isRxRateValid = false;

static FAST_DATA_ZERO_INIT uint16_t currentRxRefreshRate;


void resetYawAxis(void)
{
    rcCommand[YAW] = 0;
    smoothSetpoint[YAW] = 0;
}

float getRawSetpoint(int axis)
{
    return rawSetpoint[axis];
}

float getSetpointRate(int axis)
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


FAST_CODE void processRcCommand(void)
{
#ifdef USE_RC_SMOOTHING_FILTER
    rcSmoothingFilterUpdate(isRxDataNew, isRxRateValid, currentRxRefreshRate);

    for (int axis = 0; axis < 4; axis++) {
        smoothSetpoint[axis] = rcSmoothingFilterApply(axis, rawSetpoint[axis]);
    }
#else
    for (int axis = 0; axis < 4; axis++) {
        smoothSetpoint[axis] = rawSetpoint[axis];
    }
#endif

    isRxDataNew = false;
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
    isRxDataNew = true;

    for (int axis = 0; axis < 4; axis++) {
        float data = rcData[axis] - rxConfig()->midrc;
        rcCommand[axis] = constrainf(deadband(data, rcDeadband[axis]), -500, 500);
        rcDeflection[axis] = rcCommand[axis] / rcDivider[axis];
    }

    rcCommand[YAW] *= -GET_DIRECTION(rcControlsConfig()->yaw_control_reversed);

    // FIXME
    rcCommand[THROTTLE] = constrain(rcData[THROTTLE], PWM_RANGE_MIN, PWM_RANGE_MAX) - PWM_RANGE_MIN;

    for (int axis = 0; axis < 4; axis++) {
        rawSetpoint[axis] = applyRatesCurve(axis, rcDeflection[axis]);
        DEBUG_SET(DEBUG_ANGLERATE, axis, rawSetpoint[axis]);
    }
}

INIT_CODE void initRcProcessing(void)
{
    rcDivider[0] = 500.0f - rcControlsConfig()->deadband;
    rcDivider[1] = 500.0f - rcControlsConfig()->deadband;
    rcDivider[2] = 500.0f - rcControlsConfig()->yaw_deadband;
    rcDivider[3] = 500.0f;

    rcDeadband[0] = rcControlsConfig()->deadband;
    rcDeadband[1] = rcControlsConfig()->deadband;
    rcDeadband[2] = rcControlsConfig()->yaw_deadband;
    rcDeadband[3] = 0;
}

