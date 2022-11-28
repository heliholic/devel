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
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#ifdef USE_SERVOS

#include "build/build_config.h"

#include "common/maths.h"

#include "config/config.h"
#include "config/config_reset.h"

#include "fc/runtime_config.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "drivers/pwm_output.h"

#include "flight/servos.h"
#include "flight/mixer.h"


PG_REGISTER_WITH_RESET_FN(servoConfig_t, servoConfig, PG_SERVO_CONFIG, 0);

void pgResetFn_servoConfig(servoConfig_t *servoConfig)
{
    servoConfig->dev.servoPwmRate = DEFAULT_SERVO_UPDATE;

    for (unsigned i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        servoConfig->dev.ioTags[i] = timerioTagGetByUsage(TIM_USE_SERVO, i);
    }
}

PG_REGISTER_ARRAY_WITH_RESET_FN(servoParam_t, MAX_SUPPORTED_SERVOS, servoParams, PG_SERVO_PARAMS, 0);

void pgResetFn_servoParams(servoParam_t *instance)
{
    for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        RESET_CONFIG(servoParam_t, &instance[i],
            .servo_flags = 0,
            .servo_trim = 0,
            .high_trim = 0,
            .low_trim = 0,
            .high_limit = 0,
            .low_limit = 0,
        );
    }
}


typedef struct {
    uint16_t    center;
    uint16_t    rate;
    int16_t     max;
    int16_t     min;
    bool        linear;
} servoData_t;

static FAST_DATA_ZERO_INIT servoData_t  servos[MAX_SUPPORTED_SERVOS];

static FAST_DATA_ZERO_INIT float        servoOutput[MAX_SUPPORTED_SERVOS];
static FAST_DATA_ZERO_INIT int16_t      servoOverride[MAX_SUPPORTED_SERVOS];
static FAST_DATA_ZERO_INIT uint8_t      servoCount;


uint8_t getServoCount(void)
{
    return servoCount;
}

int16_t getServoOutput(uint8_t servo)
{
    return lrintf(servoOutput[servo]);
}

bool hasServoOverride(uint8_t servo)
{
    return (servoOverride[servo] >= SERVO_OVERRIDE_MIN && servoOverride[servo] <= SERVO_OVERRIDE_MAX);
}

int16_t getServoOverride(uint8_t servo)
{
    return servoOverride[servo];
}

int16_t setServoOverride(uint8_t servo, int16_t value)
{
    return servoOverride[servo] = value;
}

void INIT_CODE servoInitConfig(void)
{
    for (int i = 0; i < servoCount; i++) {
        const servoParam_t *param = servoParams(i);
        servoData_t *servo = &servos[i];

        uint16_t rate = 0, mid = 0, max = 0, min = 0;

        switch (param->servo_flags & SERVO_FLAGS_TYPE) {
            case SERVO_TYPE_NORMAL:
                rate = 500;
                mid = 1520 + param->servo_trim * 2;
                max = rate + param->high_limit * 2;
                min = rate + param->low_limit * 2;
                break;
            case SERVO_TYPE_NARROW:
                rate = 250;
                mid = 760 + param->servo_trim;
                max = rate + param->high_limit;
                min = rate + param->low_limit;
                break;
        }

        servo->center = mid;

        if (param->servo_flags & SERVO_FLAGS_REVERSED) {
            servo->rate = -rate;
            servo->max =   min;
            servo->min =  -max;
        } else {
            servo->rate = rate;
            servo->max =  max;
            servo->min = -min;
        }

        servo->linear = (param->servo_flags & SERVO_FLAGS_LINEAR);

        servoOutput[i] = mid;
        servoOverride[i] = SERVO_OVERRIDE_OFF;
    }
}

void INIT_CODE servoInit(void)
{
    const ioTag_t *ioTags = servoConfig()->dev.ioTags;

    for (servoCount = 0;
         servoCount < MAX_SUPPORTED_SERVOS && ioTags[servoCount] != IO_TAG_NONE;
         servoCount++);

    servoDevInit(&servoConfig()->dev, servoCount);

    servoInitConfig();
}

static inline float limitTravel(uint8_t servo, float output, float min, float max)
{
    if (output > max) {
        mixerSaturateServoOutput(servo);
        return max;
    } else if (output < min) {
        mixerSaturateServoOutput(servo);
        return min;
    }
    return output;
}

#ifdef USE_SERVO_GEOMETRY_CORRECTION
static inline float geometryCorrection(float angle)
{
    // 1.0 == 50° without correction
    float height = constrainf(angle * 0.7660444431f, -1, 1);

    // Scale 50° in rad => 1.0
    float rotation = asin_approx(height) * 1.14591559026f;

    return rotation;
}
#endif

void FAST_CODE servoUpdate(void)
{
    for (int i = 0; i < servoCount; i++)
    {
        servoData_t *servo = &servos[i];
        float output = mixerGetServoOutput(i);

#ifdef USE_SERVO_GEOMETRY_CORRECTION
        if (!servo->linear)
            output = geometryCorrection(output);
#endif

        if (!ARMING_FLAG(ARMED) && hasServoOverride(i))
            output = getServoOverride(i) / 1000.0f;

        output = limitTravel(i, servo->rate * output, servo->min, servo->max);
        output = servo->center + output;

        servoOutput[i]= output;

        pwmWriteServo(i, output);
    }
}

#endif
