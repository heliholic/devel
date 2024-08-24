/*
 * This file is part of Rotorflight.
 *
 * Rotorflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Rotorflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>
#include <string.h>

#include "platform.h"

#ifdef USE_SERVOS

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "pg/servo.h"

#include "config/config.h"
#include "config/config_reset.h"

#include "drivers/io.h"


PG_REGISTER_WITH_RESET_FN(servoConfig_t, servoConfig, PG_SERVO_CONFIG, 0);

PG_REGISTER_ARRAY_WITH_RESET_FN(servoParam_t, MAX_SUPPORTED_SERVOS, servoParams, PG_SERVO_PARAMS, 0);

void pgResetFn_servoConfig(servoConfig_t *servoConfig)
{
    servoConfig->dev.servoCenterPulse = 1500;
    servoConfig->dev.servoPwmRate = 50;

#ifdef SERVO1_PIN
    servoConfig->dev.ioTags[0] = IO_TAG(SERVO1_PIN);
#endif
#ifdef SERVO2_PIN
    servoConfig->dev.ioTags[1] = IO_TAG(SERVO2_PIN);
#endif
#ifdef SERVO3_PIN
    servoConfig->dev.ioTags[2] = IO_TAG(SERVO3_PIN);
#endif
#ifdef SERVO4_PIN
    servoConfig->dev.ioTags[3] = IO_TAG(SERVO4_PIN);
#endif
#ifdef SERVO5_PIN
    servoConfig->dev.ioTags[4] = IO_TAG(SERVO5_PIN);
#endif
#ifdef SERVO6_PIN
    servoConfig->dev.ioTags[5] = IO_TAG(SERVO6_PIN);
#endif
#ifdef SERVO7_PIN
    servoConfig->dev.ioTags[6] = IO_TAG(SERVO7_PIN);
#endif
#ifdef SERVO8_PIN
    servoConfig->dev.ioTags[7] = IO_TAG(SERVO8_PIN);
#endif
}

void pgResetFn_servoParams(servoParam_t *instance)
{
    for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        RESET_CONFIG(servoParam_t, &instance[i],
            .min = DEFAULT_SERVO_MIN,
            .max = DEFAULT_SERVO_MAX,
            .middle = DEFAULT_SERVO_MIDDLE,
            .rate = 100,
        );
    }
}

#endif // USE_SERVOS


