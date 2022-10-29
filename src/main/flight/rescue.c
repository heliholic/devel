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

#include "fc/runtime_config.h"
#include "fc/rc_controls.h"

#include "flight/rescue.h"
#include "flight/pid.h"


enum {
    RSTATE_OFF = 0,
    RSTATE_PULL_UP,
    RSTATE_FLIP_OVER,
    RSTATE_CLIMB,
    RSTATE_ALT_HOLD,
    RSTATE_POS_HOLD,
    RSTATE_EXIT,
};

typedef struct {

    uint8_t     mode;
    uint8_t     flip;

    uint8_t     state;
    timeMs_t    stateEntryTime;

    int32_t     pullUpTime;
    int32_t     climbTime;
    int32_t     exitTime;

    float       pullUpCollective;
    float       climbCollective;

    float       setpoint[4];

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

static void rescuePullUp(void)
{

}

static bool rescuePullUpDone(void)
{
    if (rescueStateTime() > rescue.pullUpTime)
        return true;

    return false;
}

static void rescueFlipOver(void)
{

}

static bool rescueFlipDone(void)
{
    return false;
}

static void rescueClimb(void)
{

}

static bool rescueClimbDone(void)
{
    if (rescueStateTime() > rescue.climbTime)
        return true;

    return false;
}

static void rescueExitRamp(void)
{

}

static bool rescueExitRampDone(void)
{
    if (rescueStateTime() > rescue.exitTime)
        return true;

    return false;
}

#if 0

float pidRescueCollective(void)
{
    float collective = rescueCollective;

    // Initial rescue with boost
    if (rescueInverted)
        collective = constrainf(collective + rescueBoost, 0, 1);

    // attitude.values.roll/pitch = 0 when level, 1800 when fully inverted (decidegrees)
    const float absRoll = fabsf(attitude.values.roll / 900.0f);
    const float absPitch = fabsf(attitude.values.pitch / 900.0f);

    // Pitch is +90/-90 at straight down and straight up. Convert it so that level = 1.0
    const float pitchCurrentInclination = 1.0f - absPitch;

    // Roll is +90/-90 when sideways, and +180/-180 when inverted
    const float rollCurrentInclination = (absRoll < 1.0f) ?  1.0f - absRoll : -1.0f + absRoll;

    // Smaller of the two
    const float vertCurrentInclination = MIN(pitchCurrentInclination, rollCurrentInclination);

    // Add more pitch as the heli approaches level
    collective *= vertCurrentInclination * vertCurrentInclination;

    // We're closer to inverted. Use negative collective pitch
    if (absRoll > 1.0f)
        collective = -collective;

    return collective;
}

float calcRescueErrorAngle(int axis)
{
    const rollAndPitchTrims_t *angleTrim = &accelerometerConfig()->accelerometerTrims;
    const float roll = (attitude.raw[FD_ROLL] - angleTrim->raw[FD_ROLL]) / 10.0f;
    const float angle = (attitude.raw[axis] - angleTrim->raw[axis]) / 10.0f;

    float error = 0;

    if (roll > 90 && rescueInverted) {
        // Rolled right closer to inverted, continue to roll right to inverted (+180 degrees)
        if (axis == FD_PITCH) {
            error = angle;
        } else if (axis == FD_ROLL) {
            error = 180.0f - angle;
        }
    } else if (roll < -90 && rescueInverted) {
        // Rolled left closer to inverted, continue to roll left to inverted (-180 degrees)
        if (axis == FD_PITCH) {
            error = angle;
        } else if (axis == FD_ROLL) {
            error = -180.0f - angle;
        }
    } else {
        // We're rolled left or right between -90 and 90, and thus are closer to up-right
        if (axis == FD_PITCH) {
            error = -angle;
        } else if (axis == FD_ROLL) {
            error = -angle;
        }
    }

    return error;
}

#endif


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
        }
    }
}


//// Interface functions

float rescueApply(uint8_t axis, float setpoint)
{
    if (rescue.state)
        return rescue.setpoint[axis];

    return setpoint;
}

void rescueUpdate(void)
{
    if (rescue.mode)
        rescueUpdateState();
}

void rescueInitProfile(const pidProfile_t *pidProfile)
{
    rescue.mode = pidProfile->rescue.mode;
    rescue.flip = pidProfile->rescue.flip_mode;

    rescue.pullUpTime = pidProfile->rescue.pull_up_time * 100000;
    rescue.climbTime = pidProfile->rescue.climb_time * 100000;
    rescue.exitTime = 2000000;

    rescue.pullUpCollective = pidProfile->rescue.pull_up_collective / 1000.0f;
    rescue.climbCollective = pidProfile->rescue.climb_collective / 1000.0f;
}

void rescueInit(const pidProfile_t *pidProfile)
{
    rescueInitProfile(pidProfile);
}
