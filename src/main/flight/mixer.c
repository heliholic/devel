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
#include <ctype.h>
#include <math.h>

#include "platform.h"

#include "build/build_config.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"

#include "config/config.h"
#include "config/config_reset.h"

#include "fc/runtime_config.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/rc.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/governor.h"

#include "rx/rx.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "sensors/gyro.h"


PG_REGISTER_WITH_RESET_TEMPLATE(mixerConfig_t, mixerConfig, PG_GENERIC_MIXER_CONFIG, 0);

PG_RESET_TEMPLATE(mixerConfig_t, mixerConfig,
    .mixer_type = MIXER_TYPE_NONE,
    .mixer_flags = 0,
    .main_rotor_dir = DIR_CW,
    .tail_rotor_mode = TAIL_MODE_VARIABLE,
    .tail_motor_idle = 0,
    .swash_ring = 0,
    .swash_phase = 0,
    .coll_correction = 0,
);

PG_REGISTER_ARRAY(mixerRule_t, MIXER_RULE_COUNT, mixerRules, PG_GENERIC_MIXER_RULES, 0);

PG_REGISTER_ARRAY_WITH_RESET_FN(mixerInput_t, MIXER_INPUT_COUNT, mixerInputs, PG_GENERIC_MIXER_INPUTS, 0);

void pgResetFn_mixerInputs(mixerInput_t *input)
{
    for (int i = MIXER_IN_STABILIZED_ROLL; i <= MIXER_IN_STABILIZED_THROTTLE; i++) {
        input[i].rate =  1000;
        input[i].min  = -1000;
        input[i].max  =  1000;
    }
}

typedef struct {

    mixerRule_t     rules[MIXER_RULE_COUNT];

    float           input[MIXER_INPUT_COUNT];
    float           output[MIXER_OUTPUT_COUNT];

    int16_t         override[MIXER_INPUT_COUNT];
    uint32_t        outputMap[MIXER_OUTPUT_COUNT];
    uint16_t        saturation[MIXER_INPUT_COUNT];

    float           collectiveCorrection;

    float           tailMotorIdle;
    int8_t          tailMotorDirection;

    float           cyclicTotal;
    float           cyclicLimit;

    float           phaseSin;
    float           phaseCos;

} mixerData_t;

static FAST_DATA_ZERO_INIT mixerData_t mixer;


#ifdef USE_MIXER_HISTORY

// History lengh is 1024 samples (must be power of 2)
#define MIXER_HISTORY_TIME   (1<<10)
#define MIXER_HISTORY_MASK   (MIXER_HISTORY_TIME-1)

static float mixerInputHistory[4][MIXER_HISTORY_TIME];

static FAST_DATA_ZERO_INIT uint16_t historyIndex;

float mixerGetInputHistory(uint8_t i, uint16_t delay)
{
    return mixerInputHistory[i][(historyIndex - delay) & MIXER_HISTORY_MASK];
}

static inline void mixerHistoryUpdate(void)
{
    historyIndex = (historyIndex + 1) & MIXER_HISTORY_MASK;

    mixerInputHistory[FD_ROLL][historyIndex]   = mixer.input[MIXER_IN_STABILIZED_ROLL];
    mixerInputHistory[FD_PITCH][historyIndex]  = mixer.input[MIXER_IN_STABILIZED_PITCH];
    mixerInputHistory[FD_YAW][historyIndex]    = mixer.input[MIXER_IN_STABILIZED_YAW];
    mixerInputHistory[FD_COLL][historyIndex]   = mixer.input[MIXER_IN_STABILIZED_COLLECTIVE];
}

#endif /* USE_MIXER_HISTORY */

float mixerGetOutput(uint8_t i)
{
    return mixer.output[i];
}

float mixerGetServoOutput(uint8_t i)
{
    return mixer.output[MIXER_SERVO_OFFSET + i];
}

float mixerGetMotorOutput(uint8_t i)
{
    return mixer.output[MIXER_MOTOR_OFFSET + i];
}

static inline void mixerSetServoOutput(uint8_t i, float value)
{
    mixer.output[MIXER_SERVO_OFFSET + i] = value;
}

static inline void mixerSetMotorOutput(uint8_t i, float value)
{
    mixer.output[MIXER_MOTOR_OFFSET + i] = value;
}

static inline void mixerSetOutputMap(uint8_t input, uint8_t output)
{
    mixer.outputMap[output] |= BIT(input);
}

static inline void mixerSetServoOutputMap(uint8_t input, uint8_t servo)
{
    mixer.outputMap[MIXER_SERVO_OFFSET + servo] |= BIT(input);
}

static inline void mixerSetMotorOutputMap(uint8_t input, uint8_t motor)
{
    mixer.outputMap[MIXER_MOTOR_OFFSET + motor] |= BIT(input);
}

bool mixerSaturated(uint8_t index)
{
    return (mixer.saturation[index] > 0);
}

void mixerSaturateInput(uint8_t index)
{
    mixer.saturation[index] = MIXER_SATURATION_TIME;
}

void mixerSaturateOutput(uint8_t index)
{
    for (int i = 1; i < MIXER_INPUT_COUNT; i++) {
        if (mixer.outputMap[index] & BIT(i)) {
            mixerSaturateInput(i);
        }
    }
}

int16_t mixerGetOverride(uint8_t i)
{
    return mixer.override[i];
}

int16_t mixerSetOverride(uint8_t i, int16_t value)
{
    return mixer.override[i] = value;
}

float getCyclicDeflection(void)
{
    return mixer.cyclicTotal;
}

float getCollectiveDeflection(void)
{
    return mixer.input[MIXER_IN_STABILIZED_COLLECTIVE];
}

float getYawDeflection(void)
{
    return mixer.input[MIXER_IN_STABILIZED_YAW];
}

float mixerGetInput(uint8_t i)
{
    return mixer.input[i];
}

static void mixerSetInput(int index, float value)
{
    const mixerInput_t *in = mixerInputs(index);

    // Check override only if not armed
    if (!ARMING_FLAG(ARMED)) {
        if (mixer.override[index] >= MIXER_OVERRIDE_MIN && mixer.override[index] <= MIXER_OVERRIDE_MAX)
            value = mixer.override[index] / 1000.0f;
    }

    // Input limits
    const float imin = in->min / 1000.0f;
    const float imax = in->max / 1000.0f;

    // Constrain and saturate
    if (value > imax) {
        mixer.input[index] = imax;
        mixerSaturateInput(index);
    }
    else if (value < imin) {
        mixer.input[index] = imin;
        mixerSaturateInput(index);
    }
    else {
        mixer.input[index] = value;
    }
}

static void mixerCyclicUpdate(void)
{
    // Swash phasing
    if (mixer.phaseSin != 0)
    {
        float SR = mixer.input[MIXER_IN_STABILIZED_ROLL];
        float SP = mixer.input[MIXER_IN_STABILIZED_PITCH];

        mixer.input[MIXER_IN_STABILIZED_PITCH] = SP * mixer.phaseCos - SR * mixer.phaseSin;
        mixer.input[MIXER_IN_STABILIZED_ROLL]  = SP * mixer.phaseSin + SR * mixer.phaseCos;
    }

    // Swashring enabled
    if (mixer.cyclicLimit > 0)
    {
        float SR = mixer.input[MIXER_IN_STABILIZED_ROLL];
        float SP = mixer.input[MIXER_IN_STABILIZED_PITCH];

        const mixerInput_t *mixR = mixerInputs(MIXER_IN_STABILIZED_ROLL);
        const mixerInput_t *mixP = mixerInputs(MIXER_IN_STABILIZED_PITCH);

        // Assume min<0 and max>0 for cyclic & pitch
        const float maxR = abs((SR < 0) ? mixR->min : mixR->max) / 1000.0f;
        const float maxP = abs((SP < 0) ? mixP->min : mixP->max) / 1000.0f;

        // Stretch the limits to the unit circle
        SR /= fmaxf(maxR, 0.001f) * mixer.cyclicLimit;
        SP /= fmaxf(maxP, 0.001f) * mixer.cyclicLimit;

        // Stretched cyclic deflection
        const float cyclic = sqrtf(sq(SR) + sq(SP));

        // Cyclic limits reached - scale back
        if (cyclic > 1.0f)
        {
            mixerSaturateInput(MIXER_IN_STABILIZED_ROLL);
            mixerSaturateInput(MIXER_IN_STABILIZED_PITCH);

            mixer.input[MIXER_IN_STABILIZED_ROLL]  /= cyclic;
            mixer.input[MIXER_IN_STABILIZED_PITCH] /= cyclic;
        }
    }

    // Total cyclic deflection
    mixer.cyclicTotal = sqrtf(sq(mixer.input[MIXER_IN_STABILIZED_ROLL]) +
                        sq(mixer.input[MIXER_IN_STABILIZED_PITCH]));
}

static void mixerCollectiveUpdate(void)
{
    // Headspeed fluctuation ratio
    const float ratio = constrainf(getHeadSpeedRatio(), 0.90f, 1.25f);

    // Collective correction - TODO: CHECK
    const float corr = mixer.collectiveCorrection * (1.0f / (ratio * ratio) - 1.0f) + 1.0f;

    // Apply correction
    mixer.input[MIXER_IN_STABILIZED_COLLECTIVE] *= corr;
}

static void mixerUpdateMotorizedTail(void)
{
    // Motorized tail control
    if (mixerIsTailMode(TAIL_MODE_MOTORIZED)) {
        // Yaw input value - positive is against torque
        const float yaw = mixer.input[MIXER_IN_STABILIZED_YAW] * mixerRotationSign();

        // Thrust linearization
        float throttle = sqrtf(fmaxf(yaw,0));

        // Apply minimum throttle
        throttle = fmaxf(throttle, mixer.tailMotorIdle);

        // Slow spoolup
        if (!isSpooledUp()) {
            if (mixer.input[MIXER_IN_STABILIZED_THROTTLE] < 0.01f)
                throttle = 0;
            else if (mixer.input[MIXER_IN_STABILIZED_THROTTLE] < 0.20f)
                throttle *= mixer.input[MIXER_IN_STABILIZED_THROTTLE] / 0.20f;
        }

        // Yaw is now tail motor throttle
        mixer.input[MIXER_IN_STABILIZED_YAW] = throttle;
    }
    else if (mixerIsTailMode(TAIL_MODE_BIDIRECTIONAL)) {
        // Yaw input value - positive is against torque
        const float yaw = mixer.input[MIXER_IN_STABILIZED_YAW] * mixerRotationSign();

        // Thrust linearization
        float throttle = copysignf(sqrtf(fabsf(yaw)),yaw);

        // Apply minimum throttle
        if (throttle > -mixer.tailMotorIdle && throttle < mixer.tailMotorIdle)
            throttle = mixer.tailMotorDirection * mixer.tailMotorIdle;

        // Slow spoolup
        if (!isSpooledUp()) {
            if (mixer.input[MIXER_IN_STABILIZED_THROTTLE] < 0.01f)
                throttle = 0;
            else if (mixer.input[MIXER_IN_STABILIZED_THROTTLE] < 0.20f)
                throttle *= mixer.input[MIXER_IN_STABILIZED_THROTTLE] / 0.20f;
        }

        // Direction sign
        mixer.tailMotorDirection = (throttle < 0) ? -1 : 1;

        // Yaw is now tail motor throttle
        mixer.input[MIXER_IN_STABILIZED_YAW] = throttle;
    }
}

#define inputVal(NAME) ( mixer.input[MIXER_IN_STABILIZED_##NAME] * \
    mixerInputs(MIXER_IN_STABILIZED_##NAME)->rate / \
    ((mixerConfig()->mixer_flags & BIT(NAME)) ? -1000.0f : 1000.0f) )

static void mixerUpdateBasic(void)
{
    if (mixerConfig()->mixer_type)
    {
        const float SR = inputVal(ROLL);
        const float SP = inputVal(PITCH);
        const float SY = inputVal(YAW);
        const float SC = inputVal(COLLECTIVE);
        const float ST = inputVal(THROTTLE);

        switch (mixerConfig()->mixer_type) {
            case MIXER_TYPE_120:
                mixerSetServoOutput(0, 0.5f * SC - SP);
                mixerSetServoOutput(1, 0.5f * SC + 0.86602540f * SR + 0.5f * SP);
                mixerSetServoOutput(2, 0.5f * SC - 0.86602540f * SR + 0.5f * SP);
                break;

            case MIXER_TYPE_135:
                mixerSetServoOutput(0, 0.5f * SC - SP);
                mixerSetServoOutput(1, 0.5f * SC + 0.70710678f * SR + 0.70710678f * SP);
                mixerSetServoOutput(2, 0.5f * SC - 0.70710678f * SR + 0.70710678f * SP);
                break;

            case MIXER_TYPE_140:
                mixerSetServoOutput(0, 0.5f * SC - SP);
                mixerSetServoOutput(1, 0.5f * SC + 0.64278760f * SR + 0.7660444f * SP);
                mixerSetServoOutput(2, 0.5f * SC - 0.64278760f * SR + 0.7660444f * SP);
                break;

            case MIXER_TYPE_90L:
                mixerSetServoOutput(0, -SP);
                mixerSetServoOutput(1, -SR);
                break;

            case MIXER_TYPE_90V:
                mixerSetServoOutput(0,  0.70710678f * SR - 0.70710678f * SP);
                mixerSetServoOutput(1, -0.70710678f * SR - 0.70710678f * SP);
                break;

            default:
                mixerSetServoOutput(0, SP);
                mixerSetServoOutput(1, SR);
                mixerSetServoOutput(2, SC);
                break;
        }

        mixerSetMotorOutput(0, ST);

        if (mixerMotorizedTail())
            mixerSetMotorOutput(1, SY);
        else
            mixerSetServoOutput(3, SY);
    }
}

static void mixerUpdateRules(void)
{
    for (int i = 0; i < MIXER_RULE_COUNT; i++) {
        if (mixer.rules[i].oper) {
            uint8_t src = mixer.rules[i].input;
            uint8_t dst = mixer.rules[i].output;
            float   val = mixer.input[src] * mixerInputs(src)->rate / 1000.0f;
            float   out = (mixer.rules[i].offset + mixer.rules[i].weight * val) / 1000.0f;

            switch (mixer.rules[i].oper)
            {
                case MIXER_OP_SET:
                    mixer.output[dst] = out;
                    break;
                case MIXER_OP_ADD:
                    mixer.output[dst] += out;
                    break;
                case MIXER_OP_MUL:
                    mixer.output[dst] *= out;
                    break;
            }
        }
    }
}

static void mixerUpdateInputs(void)
{
    // Flight Dynamics
    mixerSetInput(MIXER_IN_RC_COMMAND_ROLL, getRcDeflection(ROLL));
    mixerSetInput(MIXER_IN_RC_COMMAND_PITCH, getRcDeflection(PITCH));
    mixerSetInput(MIXER_IN_RC_COMMAND_YAW, getRcDeflection(YAW));
    mixerSetInput(MIXER_IN_RC_COMMAND_COLLECTIVE, getRcDeflection(COLLECTIVE));

    // Throttle input
    mixerSetInput(MIXER_IN_RC_COMMAND_THROTTLE, getRcDeflection(THROTTLE));

    // RC channels
    for (int i = 0; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++)
        mixerSetInput(MIXER_IN_RC_CHANNEL_ROLL + i, (rcData[i] - rxConfig()->midrc) / 500);

    // Stabilised inputs
    mixerSetInput(MIXER_IN_STABILIZED_ROLL, pidGetOutput(PID_ROLL));
    mixerSetInput(MIXER_IN_STABILIZED_PITCH, pidGetOutput(PID_PITCH));
    mixerSetInput(MIXER_IN_STABILIZED_YAW, pidGetOutput(PID_YAW));
    mixerSetInput(MIXER_IN_STABILIZED_COLLECTIVE, pidGetCollective());

    // Calculate cyclic
    mixerCyclicUpdate();

    // Calculate collective
    mixerCollectiveUpdate();

    // Update governor sub-mixer
    governorUpdate();

    // Update throttle from governor
    mixerSetInput(MIXER_IN_STABILIZED_THROTTLE, getGovernorOutput());

    // Update motorized tail
    if (mixerMotorizedTail())
        mixerUpdateMotorizedTail();

#ifdef USE_MIXER_HISTORY
    // Update history
    mixerHistoryUpdate();
#endif
}

void mixerUpdate(void)
{
    // Reset saturation
    for (int i = 0; i < MIXER_INPUT_COUNT; i++) {
        if (mixer.saturation[i])
            mixer.saturation[i]--;
    }

    // Reset mixer outputs
    for (int i = 0; i < MIXER_OUTPUT_COUNT; i++) {
        mixer.output[i] = 0;
    }

    // Fetch input values
    mixerUpdateInputs();

    // Evaluate hard-coded mixer
    mixerUpdateBasic();

    // Evaluate rule-based mixer
    mixerUpdateRules();
}

void INIT_CODE mixerInitConfig(void)
{
    mixer.tailMotorIdle = mixerConfig()->tail_motor_idle / 1000.0f;

    if (mixerConfig()->swash_phase) {
        const float angle = DECIDEGREES_TO_RADIANS(mixerConfig()->swash_phase);
        mixer.phaseSin = sin_approx(angle);
        mixer.phaseCos = cos_approx(angle);
    }
    else {
        mixer.phaseSin = 0;
        mixer.phaseCos = 1;
    }

    if (mixerConfig()->swash_ring)
        mixer.cyclicLimit = 1.4142135623f - mixerConfig()->swash_ring * 0.004142135623f;
    else
        mixer.cyclicLimit = 0;

    mixer.collectiveCorrection = mixerConfig()->coll_correction / 100.0f;
}

void INIT_CODE mixerInit(void)
{
    for (int i = 0; i < MIXER_OUTPUT_COUNT; i++) {
        mixer.output[i] = 0;
        mixer.outputMap[i] = 0;
    }

    for (int i = 1; i < MIXER_INPUT_COUNT; i++) {
        mixer.override[i] = MIXER_OVERRIDE_OFF;
    }

    switch (mixerConfig()->mixer_type) {
        case MIXER_TYPE_120:
        case MIXER_TYPE_135:
        case MIXER_TYPE_140:
            mixerSetServoOutputMap(MIXER_IN_STABILIZED_COLLECTIVE, 0);
            mixerSetServoOutputMap(MIXER_IN_STABILIZED_COLLECTIVE, 1);
            mixerSetServoOutputMap(MIXER_IN_STABILIZED_COLLECTIVE, 2);
            mixerSetServoOutputMap(MIXER_IN_STABILIZED_PITCH, 0);
            mixerSetServoOutputMap(MIXER_IN_STABILIZED_PITCH, 1);
            mixerSetServoOutputMap(MIXER_IN_STABILIZED_PITCH, 2);
            mixerSetServoOutputMap(MIXER_IN_STABILIZED_ROLL, 1);
            mixerSetServoOutputMap(MIXER_IN_STABILIZED_ROLL, 2);
            break;

        case MIXER_TYPE_90L:
            mixerSetServoOutputMap(MIXER_IN_STABILIZED_PITCH, 0);
            mixerSetServoOutputMap(MIXER_IN_STABILIZED_ROLL, 1);
            break;

        case MIXER_TYPE_90V:
            mixerSetServoOutputMap(MIXER_IN_STABILIZED_PITCH, 0);
            mixerSetServoOutputMap(MIXER_IN_STABILIZED_PITCH, 1);
            mixerSetServoOutputMap(MIXER_IN_STABILIZED_ROLL, 0);
            mixerSetServoOutputMap(MIXER_IN_STABILIZED_ROLL, 1);
            break;

        case MIXER_TYPE_THRU:
            mixerSetServoOutputMap(MIXER_IN_STABILIZED_COLLECTIVE, 0);
            mixerSetServoOutputMap(MIXER_IN_STABILIZED_PITCH, 1);
            mixerSetServoOutputMap(MIXER_IN_STABILIZED_ROLL, 2);
            break;
    }

    mixerSetMotorOutputMap(MIXER_IN_STABILIZED_THROTTLE, 0);

    if (mixerMotorizedTail())
        mixerSetMotorOutputMap(MIXER_IN_STABILIZED_YAW, 1);
    else
        mixerSetServoOutputMap(MIXER_IN_STABILIZED_YAW, 3);

    for (int i = 0; i < MIXER_RULE_COUNT; i++)
    {
        const mixerRule_t *rule = mixerRules(i);

        if (rule->oper) {
            mixer.rules[i].oper    = constrain(rule->oper, 0, MIXER_OP_COUNT - 1);
            mixer.rules[i].input   = constrain(rule->input, 0, MIXER_INPUT_COUNT - 1);
            mixer.rules[i].output  = constrain(rule->output, 0, MIXER_OUTPUT_COUNT - 1);
            mixer.rules[i].offset  = constrain(rule->offset, MIXER_INPUT_MIN, MIXER_INPUT_MAX);
            mixer.rules[i].weight  = constrain(rule->weight, MIXER_WEIGHT_MIN, MIXER_WEIGHT_MAX);

            switch (mixer.rules[i].oper)
            {
                case MIXER_OP_SET:
                    mixer.outputMap[mixer.rules[i].output] = BIT(mixer.rules[i].input);
                    break;
                case MIXER_OP_ADD:
                case MIXER_OP_MUL:
                    mixer.outputMap[mixer.rules[i].output] |= BIT(mixer.rules[i].input);
                    break;
            }
        }
    }

    mixerInitConfig();
}
