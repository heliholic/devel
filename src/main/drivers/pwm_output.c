/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
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

#include "platform.h"

#include "pg/motor.h"
#include "config/feature.h"
#include "drivers/pwm_output.h"

#if defined(USE_PWM_OUTPUT) && defined(USE_MOTOR)

FAST_DATA_ZERO_INIT pwmOutputPort_t pwmMotors[MAX_SUPPORTED_MOTORS];
FAST_DATA_ZERO_INIT uint8_t pwmMotorCount;

void analogInitEndpoints(__unused const motorConfig_t *motorConfig, float outputLimit, float *outputLow, float *outputHigh, __unused float *disarm)
{
    *outputLow = motorConfig->mincommand;
    *outputHigh = motorConfig->maxthrottle - ((motorConfig->maxthrottle - motorConfig->mincommand) * (1 - outputLimit));
}

IO_t pwmGetMotorIO(unsigned index)
{
    if (index >= pwmMotorCount) {
        return IO_NONE;
    }
    return pwmMotors[index].io;
}

bool pwmIsMotorEnabled(unsigned index)
{
    return pwmMotors[index].enabled;
}

bool pwmEnableMotors(void)
{
    /* check motors can be enabled */
    return pwmMotorCount > 0;
}

#endif
