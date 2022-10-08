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
#include <string.h>
#include <math.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "common/filter.h"
#include "common/maths.h"

#include "config/feature.h"
#include "config/config.h"

#include "drivers/time.h"

#include "sensors/acceleration.h"

#include "fc/runtime_config.h"
#include "fc/rc_controls.h"

#include "flight/rescue.h"
#include "flight/pid.h"
#include "flight/imu.h"


enum {
    RSTATE_OFF = 0,
    RSTATE_PULL_UP,
    RSTATE_FLIP_OVER,
    RSTATE_CLIMB,
    RSTATE_ALT_HOLD,
    RSTATE_POS_HOLD,
    RSTATE_EXIT,
    RSTATE_DONE,
};

typedef struct {

    uint8_t     mode;
    uint8_t     flip;

    uint8_t     state;
    timeMs_t    stateEntryTime;

    int32_t     pullUpTime;
    int32_t     climbTime;
    int32_t     flipTime;
    int32_t     exitTime;

    float       levelGain;
    float       flipGain;

    float       pullUpCollective;
    float       climbCollective;

    float       maxRate;
    float       maxAccel;

    float       setpoint[4];
    float       prevSetpoint[4];

} rescueState_t;

static FAST_DATA_ZERO_INIT rescueState_t rescue;


//// Internal functions

static inline void rescueChangeState(uint8_t newState)
{
    rescue.state = newState;
    rescue.stateEntryTime = millis();
}

static inline long rescueStateTime(void)
{
    return cmp32(millis(), rescue.stateEntryTime);
}

static inline bool rescueActive(void)
{
    return FLIGHT_MODE(RESCUE_MODE);
}

void rescueApplyStabilisation(void)
{
    const rollAndPitchTrims_t *trim = &accelerometerConfig()->accelerometerTrims;

    const int roll = attitude.values.roll - trim->values.roll;
    const int pitch = attitude.values.pitch - trim->values.pitch;

    int Rerror = 0;
    int Perror = 0;

    if (roll > 900) {
        Perror = pitch;
        Rerror = 1800 - roll;
    }
    else if (roll < -900) {
        Perror = pitch;
        Rerror = -1800 - roll;
    }
    else {
        Perror = -pitch;
        Rerror = -roll;
    }

    // Avoid "gimbal lock"
    if (pitch > 750 || pitch < -750) {
        Rerror = 0;
    }

    rescue.setpoint[FD_PITCH] = constrainf(Perror * rescue.levelGain, -rescue.maxRate, rescue.maxRate);
    rescue.setpoint[FD_ROLL] = constrainf(Rerror * rescue.levelGain, -rescue.maxRate, rescue.maxRate);
    rescue.setpoint[FD_YAW] = 0;
}

static void rescuePullUp(void)
{
    rescueApplyStabilisation();

    const float ct = getCosTiltAngle();

    rescue.setpoint[FD_COLL] = rescue.pullUpCollective * copysignf(ct*ct, ct);
}

static inline bool rescuePullUpDone(void)
{
    return (rescueStateTime() > rescue.pullUpTime);
}

static inline void rescueFlipOver(void)
{
    const float ct = getCosTiltAngle();

    rescue.setpoint[FD_COLL] = rescue.pullUpCollective * copysignf(ct*ct, ct);
}

static inline bool rescueFlipDone(void)
{
    return (getCosTiltAngle() > 0.95f);
}

static inline bool rescueFlipTimeout(void)
{
    return (rescueStateTime() > rescue.flipTime);
}

static void rescueClimb(void)
{
    rescueApplyStabilisation();

    const float ct = getCosTiltAngle();

    rescue.setpoint[FD_COLL] = rescue.climbCollective * copysignf(ct*ct, ct);
}

static inline bool rescueClimbDone(void)
{
    return (rescueStateTime() > rescue.climbTime);
}

static void rescueExitRamp(void)
{
}

static bool rescueExitRampDone(void)
{
    return (rescueStateTime() > rescue.exitTime);
}

static void rescueUpdateState(void)
{
    // Handle DISARM separately for SAFETY!
    if (!ARMING_FLAG(ARMED)) {
        rescueChangeState(RSTATE_OFF);
    }
    else {
        switch (rescue.state)
        {
            case RSTATE_OFF:
                if (rescueActive()) {
                    rescueChangeState(RSTATE_PULL_UP);
                    rescuePullUp();
                }
                break;

            case RSTATE_PULL_UP:
                rescuePullUp();
                if (!rescueActive())
                    rescueChangeState(RSTATE_EXIT);
                else if (rescuePullUpDone()) {
                    if (rescue.flip)
                        rescueChangeState(RSTATE_FLIP_OVER);
                    else
                        rescueChangeState(RSTATE_CLIMB);
                }
                break;

            case RSTATE_FLIP_OVER:
                rescueFlipOver();
                // Flip can't be interrupted
                if (rescueFlipDone()) {
                    if (!rescueActive())
                        rescueChangeState(RSTATE_EXIT);
                    else
                        rescueChangeState(RSTATE_CLIMB);
                }
                else if (rescueFlipTimeout()) {
                    rescueChangeState(RSTATE_EXIT);
                }
                break;

            case RSTATE_CLIMB:
                rescueClimb();
                if (!rescueActive())
                    rescueChangeState(RSTATE_EXIT);
                else if (rescueClimbDone())
                    rescueChangeState(RSTATE_EXIT);
                break;

            case RSTATE_EXIT:
                rescueExitRamp();
                if (rescueActive())
                    rescueChangeState(RSTATE_PULL_UP);
                else if (rescueExitRampDone())
                    rescueChangeState(RSTATE_OFF);
                break;

            case RSTATE_DONE:
                if (!rescueActive())
                    rescueChangeState(RSTATE_OFF);
                break;
        }
    }

    for (int i=0; i<4; i++) {
        rescue.setpoint[i] = slewLimit(rescue.prevSetpoint[i], rescue.setpoint[i], rescue.maxAccel);
        rescue.prevSetpoint[i] = rescue.setpoint[i];
    }
}


//// Interface functions

void rescueUpdate(void)
{
    if (rescue.mode)
        rescueUpdateState();

    DEBUG(RESCUE, 0, attitude.values.roll);
    DEBUG(RESCUE, 1, attitude.values.pitch);
    DEBUG(RESCUE, 2, attitude.values.yaw);
    DEBUG(RESCUE, 3, getCosTiltAngle() * 1000);

    DEBUG(RESCUE, 4, rescue.setpoint[0]);
    DEBUG(RESCUE, 5, rescue.setpoint[1]);
    DEBUG(RESCUE, 6, rescue.setpoint[2]);
    DEBUG(RESCUE, 7, rescue.setpoint[3]);
}

float rescueApply(uint8_t axis, float setpoint)
{
    if (rescue.state != RSTATE_OFF && rescue.state != RSTATE_DONE)
        return rescue.setpoint[axis];

    return setpoint;
}

void rescueInitProfile(const pidProfile_t *pidProfile)
{
    rescue.mode = pidProfile->rescue.mode;
    rescue.flip = pidProfile->rescue.flip_mode;

    rescue.levelGain = pidProfile->rescue.level_gain;
    rescue.flipGain = pidProfile->rescue.flip_gain;

    rescue.pullUpTime = pidProfile->rescue.pull_up_time * 100000;
    rescue.climbTime = pidProfile->rescue.climb_time * 100000;
    rescue.flipTime = pidProfile->rescue.flip_time * 100000;
    rescue.exitTime = pidProfile->rescue.exit_time * 100000;

    rescue.pullUpCollective = pidProfile->rescue.pull_up_collective / 1000.0f;
    rescue.climbCollective = pidProfile->rescue.climb_collective / 1000.0f;

    rescue.maxRate = pidProfile->rescue.max_roll_rate;
}
