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

#include "common/maths.h"
#include "common/axis.h"
#include "common/time.h"

#include "config/feature.h"
#include "config/config.h"

#include "flight/pid.h"
#include "flight/wiggle.h"


typedef struct {
    int         action;
    int         param;
    int         state;
    timeUs_t    start;
    float       axis[4];
} wiggleState_t;

static wiggleState_t wgl = { 0, };


float wiggleGetAxis(int axis)
{
    return wgl.axis[axis];
}

bool wiggleActive(void)
{
    return (wgl.action != WIGGLE_OFF);
}

void wiggleTrigger(int action, int param)
{
    wgl.action = action;
    wgl.param = param;
    wgl.state = 0;
}

static inline void wiggleResetAxis(void)
{
    wgl.axis[0] = wgl.axis[1] = wgl.axis[2] = wgl.axis[3] = 0;
}

static void wiggleSetState(timeUs_t currentTimeUs, uint8_t state)
{
    wgl.state = state;
    wgl.start = currentTimeUs;
}

static timeDelta_t wiggleStateTime(timeUs_t currentTimeUs)
{
    return cmpTimeUs(currentTimeUs, wgl.start) / 1000;
}

static void wiggleStopAction(timeUs_t __unused currentTimeUs)
{
    wiggleResetAxis();
    wgl.action = 0;
    wgl.state = 0;
    wgl.start = 0;
}


static void wiggleActionReady(timeUs_t currentTimeUs)
{
    const int circle_period = 1200;
    const float level = pidGetCollective();
    const float delta = 0.5f;

    switch (wgl.state)
    {
        case 0:
            wiggleResetAxis();
            wiggleSetState(currentTimeUs, 1);
            FALLTHROUGH;

        case 1:
        {
            timeDelta_t time = wiggleStateTime(currentTimeUs);
            float angle = time * M_2PIf / circle_period;

            wgl.axis[FD_COLL] = level;
            wgl.axis[FD_ROLL] = sin_approx(angle) * delta;
            wgl.axis[FD_PITCH] = cos_approx(angle) * delta;

            if (time > circle_period) {
                wiggleResetAxis();
                wiggleSetState(currentTimeUs, 2);
            }

            break;
        }

        case 2:
        {
            timeDelta_t time = wiggleStateTime(currentTimeUs);

            if (time < 100)
                wgl.axis[FD_COLL] = level;
            else if (time < 200)
                wgl.axis[FD_COLL] = level - delta;
            else if (time < 300)
                wgl.axis[FD_COLL] = level + delta;
            else
                wiggleStopAction(currentTimeUs);

            break;
        }

        default:
            wiggleStopAction(currentTimeUs);
            break;
    }
}

static void wiggleActionArmed(timeUs_t currentTimeUs)
{
    const float delta = 0.5f;
    const float level = pidGetCollective();
    const timeDelta_t period = 50;

    switch (wgl.state)
    {
        case 0:
            wiggleResetAxis();
            wiggleSetState(currentTimeUs, 1);
            FALLTHROUGH;

        case 1:
        {
            timeDelta_t time = wiggleStateTime(currentTimeUs);

            if (time < period)
                wgl.axis[FD_COLL] = level - delta;
            else if (time < 2 * period)
                wgl.axis[FD_COLL] = level;
            else if (time < 3 * period)
                wgl.axis[FD_COLL] = level - delta;
            else
                wiggleStopAction(currentTimeUs);
            break;
        }

        default:
            wiggleStopAction(currentTimeUs);
            break;
    }
}

static void wiggleActionError(timeUs_t currentTimeUs)
{
    float level = 0.666f;

    switch (wgl.state)
    {
        case 0:
            wiggleResetAxis();
            wiggleSetState(currentTimeUs, 1);
            FALLTHROUGH;

        case 1:
        {
            timeDelta_t time = wiggleStateTime(currentTimeUs);
            int freq = time / 50;

            wgl.axis[FD_COLL] = (freq & 1) ? level : -level;

            if (time > wgl.param)
                wiggleStopAction(currentTimeUs);

            break;
        }

        default:
            wiggleStopAction(currentTimeUs);
            break;
    }
}

static void wiggleActionBuzzer(timeUs_t currentTimeUs)
{
    float level = 0.5f;

    switch (wgl.state)
    {
        case 0:
            wiggleResetAxis();
            wiggleSetState(currentTimeUs, 1);
            FALLTHROUGH;

        case 1:
        {
            timeDelta_t time = wiggleStateTime(currentTimeUs);
            int step = time / 500;
            int freq = time / 50;

            if (step & 1)
                wgl.axis[FD_COLL] = (freq & 1) ? level : -level;
            else
                wgl.axis[FD_COLL] = 0;

            if (time > wgl.param)
                wiggleStopAction(currentTimeUs);

            break;
        }

        default:
            wiggleStopAction(currentTimeUs);
            break;
    }
}


void wiggleUpdate(timeUs_t currentTimeUs)
{
    if (wgl.action) {
        switch (wgl.action)
        {
            case WIGGLE_READY:
                wiggleActionReady(currentTimeUs);
                break;
            case WIGGLE_ARMED:
                wiggleActionArmed(currentTimeUs);
                break;
            case WIGGLE_ERROR:
                wiggleActionError(currentTimeUs);
                break;
            case WIGGLE_BUZZER:
                wiggleActionBuzzer(currentTimeUs);
                break;
            default:
                wiggleStopAction(currentTimeUs);
                break;
        }
    }
}

void wiggleInit(void)
{
    /* NADA */
}
