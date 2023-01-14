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
#include "common/maths.h"

#include "config/config.h"
#include "config/feature.h"

#include "flight/pid.h"
#include "flight/position.h"
#include "flight/governor.h"

#include "fc/runtime_config.h"
#include "fc/rc.h"

#include "setpoint.h"


#define SP_SMOOTHING_FILTER_MIN_HZ             5
#define SP_SMOOTHING_FILTER_MAX_HZ           500


typedef struct
{
    float deflection[4];
    float setpoint[4];
    float limited[4];

    float accelLimit[4];
    float ringLimit;

    float maximum[4];
    float deviation[4];
    float maxGainUp;
    float maxGainDown;

    pt3Filter_t filter[4];
    pt2Filter_t dcFilter[4];

    uint16_t smoothCutoff;
    uint16_t activeCutoff;

} setpointData_t;

static FAST_DATA_ZERO_INIT setpointData_t sp;


float getSetpoint(int axis)
{
    return sp.setpoint[axis];
}

float getDeflection(int axis)
{
    return sp.deflection[axis];
}

uint16_t setpointFilterGetCutoffFreq(void)
{
    return sp.activeCutoff;
}


static float setpointAutoSmoothingCutoff(float frameTimeUs, uint8_t autoSmoothnessFactor)
{
    float cutoff = 0;

    if (frameTimeUs > 0) {
        float factor = 1.5f / (1.0f + (autoSmoothnessFactor / 10.0f));
        cutoff = factor * (1.0f / (frameTimeUs * 1e-6f));
    }

    return cutoff;
}

void setpointUpdateTiming(float frameTimeUs)
{
    float cutoff = setpointAutoSmoothingCutoff(frameTimeUs, rxConfig()->rx_smoothness);

    DEBUG(SETPOINT, 5, cutoff);

    cutoff = MIN(sp.smoothCutoff, cutoff);
    cutoff = constrain(cutoff, SP_SMOOTHING_FILTER_MIN_HZ, SP_SMOOTHING_FILTER_MAX_HZ);

    DEBUG(SETPOINT, 6, cutoff);

    if (sp.activeCutoff != cutoff) {
        const float gain = pt3FilterGain(cutoff, pidGetDT());
        for (int i = 0; i < 4; i++) {
            pt3FilterUpdateCutoff(&sp.filter[i], gain);
        }
        sp.activeCutoff = cutoff;
    }

    DEBUG(SETPOINT, 7, frameTimeUs);
}

INIT_CODE void setpointInitProfile(void)
{
    sp.ringLimit = 1.0f / (1.4142135623f - currentControlRateProfile->cyclic_ring * 0.004142135623f);

    for (int i = 0; i < 4; i++) {
        sp.accelLimit[i] = 10.0f * currentControlRateProfile->accel_limit[i] * pidGetDT();
    }

    sp.smoothCutoff = 1000.0f / constrain(currentControlRateProfile->rates_smoothness, 1, 250);
    sp.activeCutoff = constrain(sp.smoothCutoff, SP_SMOOTHING_FILTER_MIN_HZ, SP_SMOOTHING_FILTER_MAX_HZ);
}

INIT_CODE void setpointInit(void)
{
    float gain;

    setpointInitProfile();

    gain = pt3FilterGain(sp.activeCutoff, pidGetDT());
    for (int i = 0; i < 4; i++) {
        pt3FilterInit(&sp.filter[i], gain);
    }

    gain = pt2FilterGain(0.1f, pidGetDT());
    for (int i = 0; i < 4; i++) {
        pt2FilterInit(&sp.dcFilter[i], gain);
    }

    sp.maxGainUp    = pt1FilterGain(10, pidGetDT());
    sp.maxGainDown  = sp.maxGainUp / 25;
}

void setpointUpdate(void)
{
    for (int axis = 0; axis < 4; axis++) {
        float deflection, delta;

        deflection = sp.deflection[axis] = getRcDeflection(axis);
        DEBUG_AXIS(SETPOINT, axis, 0, deflection * 1000);

        delta = sq(deflection)- sp.maximum[axis];
        sp.maximum[axis] += delta * ((delta > 0) ? sp.maxGainUp : sp.maxGainDown);

        deflection -= pt2FilterApply(&sp.dcFilter[axis], deflection);
        delta = sq(deflection)- sp.deviation[axis];
        sp.deviation[axis] += delta * ((delta > 0) ? sp.maxGainUp : sp.maxGainDown);

        DEBUG_AXIS(SETPOINT, axis, 6, sp.maximum[axis]);
        DEBUG_AXIS(SETPOINT, axis, 7, sp.deviation[axis]);

    }

    DEBUG(AIRBORNE, 0, sqrtf(sp.deviation[FD_ROLL]) * 1000);
    DEBUG(AIRBORNE, 1, sqrtf(sp.deviation[FD_PITCH]) * 1000);
    DEBUG(AIRBORNE, 2, sqrtf(sp.maximum[FD_COLL]) * 1000);
    DEBUG(AIRBORNE, 3, isAirborne() ? 1 : 0);

    const float R = sp.deflection[FD_ROLL]  * sp.ringLimit;
    const float P = sp.deflection[FD_PITCH] * sp.ringLimit;
    const float C = sqrtf(sq(R) + sq(P));

    if (C > 1.0f) {
        sp.deflection[FD_ROLL]  /= C;
        sp.deflection[FD_PITCH] /= C;
    }

    DEBUG_AXIS(SETPOINT, FD_ROLL, 1, sp.deflection[FD_ROLL] * 1000);
    DEBUG_AXIS(SETPOINT, FD_PITCH, 1, sp.deflection[FD_PITCH] * 1000);

    for (int axis = 0; axis < 4; axis++) {
        float SP = sp.limited[axis] = slewLimit(sp.limited[axis], sp.deflection[axis], sp.accelLimit[axis]);
        DEBUG_AXIS(SETPOINT, axis, 2, SP * 1000);

        SP = sp.deflection[axis] = pt3FilterApply(&sp.filter[axis], SP);
        DEBUG_AXIS(SETPOINT, axis, 3, SP * 1000);

        SP = sp.setpoint[axis] = applyRatesCurve(axis, SP);
        DEBUG_AXIS(SETPOINT, axis, 4, SP);
    }
}

bool isAirborne(void)
{
    return (
        ARMING_FLAG(ARMED) &&
        isSpooledUp() && (
            sp.maximum[FD_COLL] > 0.025f ||
            sp.deviation[FD_ROLL] > 0.01f ||
            sp.deviation[FD_PITCH] > 0.01f ||
            getAltitude() > 2.0f ||
            FLIGHT_MODE(RESCUE_MODE | GPS_RESCUE_MODE | FAILSAFE_MODE)
        )
    );
}
