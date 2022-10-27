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


typedef struct {

    uint8_t     mode;

    uint8_t     state;
    timeMs_t    stateEntryTime;

    int32_t     pullUpTime;
    float       pullUpCollective;

    int32_t     climbTime;
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

static void rescueStabiliseToLevel(void)
{

}

static void rescueFlipUpright(void)
{

}

static void rescueClimb(void)
{

}

static void rescueExitSlew(void)
{

}

static void rescueUpdateState(void)
{
    // Handle DISARM separately for SAFETY!
    if (!ARMING_FLAG(ARMED)) {
        rescueChangeState(RESCUE_STATE_OFF);
    }
    else {
        switch (rescue.state)
        {
            case RESCUE_STATE_OFF:
                if (FLIGHT_MODE(RESCUE_MODE))
                    rescueChangeState(RESCUE_PULL_UP);
                break;

            case RESCUE_PULL_UP:
                if (!FLIGHT_MODE(RESCUE_MODE))
                    rescueChangeState(RESCUE_EXIT);
                else
                    rescueStabiliseToLevel();
                break;

            case RESCUE_FLIP_OVER:
                rescueFlipUpright();
                break;

            case RESCUE_CLIMB:
                if (!FLIGHT_MODE(RESCUE_MODE))
                    rescueChangeState(RESCUE_EXIT);
                else if (rescueStateTime() > rescue.climbTime)
                    rescueChangeState(RESCUE_EXIT);
                else
                    rescueClimb();
                break;

            case RESCUE_EXIT:
                if (FLIGHT_MODE(RESCUE_MODE)) {
                    // TODO check how long time
                    rescueChangeState(RESCUE_PULL_UP);
                } else if (rescueStateTime() > RESCUE_EXIT_TIME) {
                    rescueChangeState(RESCUE_STATE_OFF);
                } else {
                    rescueExitSlew();
                }
                break;
        }
    }
}



//// Interface functions

float rescueApply(uint8_t axis, float setpoint)
{
    UNUSED(axis);
    return setpoint;
}

void rescueUpdate(void)
{
    if (rescue.mode) {
        rescueUpdateState();
    }
}

void rescueInitProfile(const pidProfile_t *pidProfile)
{
    UNUSED(pidProfile);
}

void rescueInit(const pidProfile_t *pidProfile)
{
    UNUSED(pidProfile);
}
