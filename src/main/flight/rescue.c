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
    RSTATE_PULLUP,
    RSTATE_FLIP,
    RSTATE_CLIMB,
    RSTATE_HOVER,
    RSTATE_EXIT,
};

typedef struct {

    uint8_t         mode;
    uint8_t         flip;

    uint8_t         state;
    timeMs_t        stateEntryTime;

    timeDelta_t     pullUpTime;
    timeDelta_t     climbTime;
    timeDelta_t     flipTime;
    timeDelta_t     exitTime;

    float           levelGain;
    float           flipGain;

    float           pullUpCollective;
    float           climbCollective;
    float           hoverCollective;

    float           maxRate;
    float           maxAccel;

    float           setpoint[4];
    float           prevSetpoint[4];

} rescueState_t;

static FAST_DATA_ZERO_INIT rescueState_t rescue;


//// Internal functions

static inline void rescueChangeState(uint8_t newState)
{
    rescue.state = newState;
    rescue.stateEntryTime = millis();
}

static inline timeDelta_t rescueStateTime(void)
{
    return cmp32(millis(), rescue.stateEntryTime);
}

static inline bool rescueActive(void)
{
    return FLIGHT_MODE(RESCUE_MODE);
}

static inline float rescueSetpoint(uint8_t axis, float setpoint)
{
    if (rescue.state == RSTATE_OFF) {
        rescue.prevSetpoint[axis] = rescue.setpoint[axis] = setpoint;
    }
    else if (rescue.state == RSTATE_EXIT) {
        float alpha = (float)rescueStateTime() / (float)rescue.exitTime;
        setpoint = alpha * setpoint + (1.0f - alpha) * rescue.setpoint[axis];
    }
    else {
        setpoint = rescue.setpoint[axis];
    }

    return setpoint;
}

static inline bool rescueIsInverted(void)
{
    return getCosTiltAngle() < 0;
}

static inline bool rescueIsLeveled(void)
{
    return fabsf(getCosTiltAngle()) > 0.866f; // less than 30deg error from level
}

static void rescueApplyLimits(void)
{
    // Rate limit is for RPY
    for (int i=0; i<3; i++) {
        rescue.setpoint[i] = constrainf(rescue.setpoint[i], -rescue.maxRate, rescue.maxRate);
    }

    for (int i=0; i<4; i++) {
        rescue.setpoint[i] = slewLimit(rescue.prevSetpoint[i], rescue.setpoint[i], rescue.maxAccel);
        rescue.prevSetpoint[i] = rescue.setpoint[i];
    }
}

static void rescueApplyStabilisation(void)
{
    const rollAndPitchTrims_t *trim = &accelerometerConfig()->accelerometerTrims;

    const int roll = attitude.values.roll - trim->values.roll;
    const int pitch = attitude.values.pitch - trim->values.pitch;

    int rError = -pitch;
    int pError = -roll;

    // Inverted
    if (roll > 900) {
        pError = pitch;
        rError = 1800 - roll;
    }
    else if (roll < -900) {
        pError = pitch;
        rError = -1800 - roll;
    }

    // Avoid "gimbal lock"
    if (pitch > 750 || pitch < -750) {
        rError = 0;
    }

    rescue.setpoint[FD_PITCH] = pError * rescue.levelGain;
    rescue.setpoint[FD_ROLL] = rError * rescue.levelGain;
    rescue.setpoint[FD_YAW] = 0;
}

static void rescueApplyFlip(void)
{
    const rollAndPitchTrims_t *trim = &accelerometerConfig()->accelerometerTrims;

    int rError = attitude.values.roll - trim->values.roll;
    int pError = attitude.values.pitch - trim->values.pitch;

    // Avoid "gimbal lock"
    if (pError > 750 || pError < -750) {
        rError = 0;
    }

    rescue.setpoint[FD_PITCH] = -pError * rescue.flipGain;
    rescue.setpoint[FD_ROLL] = -rError * rescue.flipGain;
    rescue.setpoint[FD_YAW] = 0;
}

static void rescueApplyCollective(float collective)
{
    const float ct = getCosTiltAngle();
    rescue.setpoint[FD_COLL] = collective * copysignf(ct*ct, ct);
}

static void rescuePullUp(void)
{
    rescueApplyStabilisation();
    rescueApplyCollective(rescue.pullUpCollective);
    rescueApplyLimits();
}

static inline bool rescuePullUpDone(void)
{
    return (rescueStateTime() > rescue.pullUpTime);
}

static void rescueFlipOver(void)
{
    rescueApplyFlip();
    rescueApplyCollective(rescue.pullUpCollective);
    rescueApplyLimits();
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
    rescueApplyCollective(rescue.climbCollective);
    rescueApplyLimits();
}

static inline bool rescueClimbDone(void)
{
    return (rescueStateTime() > rescue.climbTime);
}

static void rescueHover(void)
{
    rescueApplyStabilisation();
    rescueApplyCollective(rescue.hoverCollective);
    rescueApplyLimits();
}

static inline bool rescueSlowExitDone(void)
{
    return (rescueStateTime() > rescue.exitTime);
}


static void rescueUpdateState(void)
{
    // Handle DISARM separately
    if (!ARMING_FLAG(ARMED)) {
        rescueChangeState(RSTATE_OFF);
    }
    else {
        switch (rescue.state)
        {
            case RSTATE_OFF:
                if (rescueActive()) {
                    rescueChangeState(RSTATE_PULLUP);
                    rescuePullUp();
                }
                break;

            case RSTATE_PULLUP:
                rescuePullUp();
                if (!rescueActive())
                    rescueChangeState(RSTATE_EXIT);
                else if (rescuePullUpDone()) {
                    if (rescueIsLeveled()) {
                        if (rescue.flip && rescueIsInverted())
                            rescueChangeState(RSTATE_FLIP);
                        else
                            rescueChangeState(RSTATE_CLIMB);
                    }
                    else {
                        rescueChangeState(RSTATE_EXIT);
                    }
                }
                break;

            case RSTATE_FLIP:
                rescueFlipOver();
                if (rescueFlipDone()) {
                    if (!rescueActive())
                        rescueChangeState(RSTATE_EXIT);
                    else
                        rescueChangeState(RSTATE_CLIMB);
                }
                else if (rescueFlipTimeout()) {
                    if (rescueIsLeveled())
                        rescueChangeState(RSTATE_CLIMB);
                    else
                        rescueChangeState(RSTATE_EXIT);
                }
                break;

            case RSTATE_CLIMB:
                rescueClimb();
                if (!rescueActive())
                    rescueChangeState(RSTATE_EXIT);
                else if (rescueClimbDone())
                    rescueChangeState(RSTATE_HOVER);
                break;

            case RSTATE_HOVER:
                rescueHover();
                if (!rescueActive())
                    rescueChangeState(RSTATE_EXIT);
                break;

            case RSTATE_EXIT:
                if (rescueActive())
                    rescueChangeState(RSTATE_PULLUP);
                else if (rescueSlowExitDone())
                    rescueChangeState(RSTATE_OFF);
                break;
        }
    }
}


//// Interface functions

uint8_t getRescueState(void)
{
    return rescue.state;
}

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
    if (rescue.mode)
        setpoint = rescueSetpoint(axis, setpoint);

    return setpoint;
}

void INIT_CODE rescueInitProfile(const pidProfile_t *pidProfile)
{
    rescue.mode = pidProfile->rescue.mode;
    rescue.flip = pidProfile->rescue.flip_mode;

    rescue.levelGain = pidProfile->rescue.level_gain / 250.0f;
    rescue.flipGain = pidProfile->rescue.flip_gain / 250.0f;

    rescue.pullUpTime = pidProfile->rescue.pull_up_time * 100;
    rescue.climbTime = pidProfile->rescue.climb_time * 100;
    rescue.flipTime = pidProfile->rescue.flip_time * 100;
    rescue.exitTime = pidProfile->rescue.exit_time * 100;

    rescue.pullUpCollective = pidProfile->rescue.pull_up_collective;
    rescue.climbCollective = pidProfile->rescue.climb_collective;
    rescue.hoverCollective = pidProfile->rescue.hover_collective;

    rescue.maxRate = pidProfile->rescue.max_rate;
    rescue.maxAccel = pidProfile->rescue.max_accel * pidGetDT();
}
