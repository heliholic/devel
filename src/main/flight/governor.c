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
#include "fc/rc.h"

#include "sensors/battery.h"
#include "sensors/gyro.h"

#include "rx/rx.h"

#include "flight/governor.h"
#include "flight/mixer.h"
#include "flight/pid.h"


// Headspeed quality levels
#define GOV_HS_DETECT_DELAY             200
#define GOV_HS_DETECT_RATIO             0.05f

// RPM glitch ratio
#define GOV_HS_GLITCH_DELTA             0.25f
#define GOV_HS_GLITCH_LIMIT             2.0f

// Lost headspeed levels
#define GOV_HS_INVALID_RATIO            0.01f
#define GOV_HS_INVALID_THROTTLE         0.10f

// Nominal battery cell voltage
#define GOV_NOMINAL_CELL_VOLTAGE        3.70f

// Dynamic min throttle limit (%)
#define GOV_DYN_MIN_THROTTLE_LIMIT      0.80f

// Motor constant K filter
#define GOV_MOTOR_K_CUTOFF              0.10f

// Auto bailout timeout if no RPM signal
#define GOV_BAILOUT_TIMEOUT             1000

// Approx throttle headroom
#define GOV_THROTTLE_HEADROOM           1.25f


//// Internal Data

typedef struct {

    // Governor type
    govMode_e       govMode;

    // Governor features
    bool            usePassthrough;
    bool            useDynMinThrottle;
    bool            useThreePosThrottle;
    bool            useHsAdjustment;
    bool            usePidSpoolup;
    bool            useVoltageComp;
    bool            useFallbackPrecomp;
    bool            useDirectPrecomp;
    bool            useAutoRotation;
    bool            usePrecomp;
    bool            useTTA;

    // State machine
    govState_e      state;
    timeMs_t        stateEntryTime;

    // Headspeed spoolup active
    bool            hsSpoolupActive;

    // Output throttle
    float           throttleOutput;

    // Input throttle
    float           throttleInput;
    bool            throttleInputOff;
    float           throttlePrevInput;

    // Fallback base throttle
    float           baseThrottle;

    // Idle throttle level
    float           idleThrottle;

    // Autorotation throttle level
    float           autoThrottle;

    // Handover level
    float           handoverThrottle;

    // Startup/spoolup throttle levels
    float           minSpoolupThrottle;
    float           maxSpoolupThrottle;

    // Active throttle limits
    float           minActiveThrottle;
    float           maxActiveThrottle;

    // Current headspeed
    float           currentHeadSpeed;

    // Headspeed config
    float           fullHeadSpeed;
    float           targetHeadSpeed;
    float           requestedHeadSpeed;

    // Proportial headspeeds
    float           requestRatio;
    float           fullHeadSpeedRatio;

    // Main gear ratio (motor/head)
    float           mainGearRatio;

    // RPM Signal Flags
    bool            motorRPMGood;

    // RPM filter & detection
    filter_t        motorRPMFilter;
    float           motorRPMGlitchDelta;
    float           motorRPMGlitchLimit;
    uint32_t        motorRPMDetectTime;

    // Battery voltage
    float           motorVoltage;
    filter_t        motorVoltageFilter;

    // Nominal battery voltage
    float           nominalVoltage;

    // Voltage compensation gain
    float           voltageCompGain;

    // Motor constant (approx HS at full throttle)
    float           motorHSK;
    ewma1Filter_t   motorHSKFilter;

    // Dynamic min throttle limit
    float           dynMinLevel;  // TDB remove

    // PID terms
    float           P;
    float           I;
    float           C;
    float           D;
    float           F;
    float           pidSum;

    // Differentiator with bandwidth limiter
    difFilter_t     differentiator;

    // PID Gains
    float           K;
    float           Kp;
    float           Ki;
    float           Kd;
    float           Kf;

    // PID Limits
    float           LP;
    float           lp;
    float           Li;
    float           Ld;
    float           Lf;

    // Feedforward
    float           yawWeight;
    float           cyclicWeight;
    float           collectiveWeight;
    uint8_t         collectiveCurve;
    filter_t        precompFilter;

    // Tail Torque Assist
    float           TTAAdd;
    float           ttaGain;
    float           ttaLimit;
    filter_t        TTAFilter;

    // Autorotation
    long            autoMinEntry;
    long            autoTimeout;

    // Timeouts
    long            throttleHoldTimeout;

    // Throttle change rates
    float           throttleStartupRate;
    float           throttleSpoolupRate;
    float           throttleRecoveryRate;
    float           throttleTrackingRate;

} govData_t;

static FAST_DATA_ZERO_INIT govData_t gov;


//// Access functions

int getGovernorState(void)
{
    return gov.state;
}

float getGovernorOutput(void)
{
    return gov.throttleOutput;
}

float getTTAIncrease(void)
{
    return gov.TTAAdd;
}

float getFullHeadSpeedRatio(void)
{
    if (gov.govMode > GOV_MODE_EXTERNAL) {
        switch (gov.state)
        {
            case GOV_STATE_ACTIVE:
            case GOV_STATE_RECOVERY:
            case GOV_STATE_SPOOLUP:
            case GOV_STATE_THROTTLE_HOLD:
            case GOV_STATE_FALLBACK:
            case GOV_STATE_AUTOROTATION:
            case GOV_STATE_BAILOUT:
                return gov.fullHeadSpeedRatio;

            default:
                return 1.0f;
        }
    }

    return 1.0f;
}

float getSpoolUpRatio(void)
{
    if (!ARMING_FLAG(ARMED))
        return 0;

    if (gov.govMode ) {
        switch (gov.state)
        {
            case GOV_STATE_THROTTLE_OFF:
            case GOV_STATE_THROTTLE_HOLD:
            case GOV_STATE_THROTTLE_IDLE:
            case GOV_STATE_AUTOROTATION:
                return 0;

            case GOV_STATE_ACTIVE:
            case GOV_STATE_RECOVERY:
            case GOV_STATE_FALLBACK:
            case GOV_STATE_BAILOUT:
            case GOV_STATE_PASSTHRU:
                return 1.0f;

            case GOV_STATE_SPOOLUP:
                return gov.requestRatio;
        }
        return 0;
    }
    else {
        return 1.0f;
    }

    return 0;
}

bool isSpooledUp(void)
{
    if (!ARMING_FLAG(ARMED))
        return false;

    if (gov.govMode ) {
        switch (gov.state)
        {
            case GOV_STATE_ACTIVE:
            case GOV_STATE_AUTOROTATION:
            case GOV_STATE_BAILOUT:
            case GOV_STATE_PASSTHRU:
                return true;

            case GOV_STATE_RECOVERY:
            case GOV_STATE_SPOOLUP:
            case GOV_STATE_FALLBACK:
                return (gov.throttleOutput > 0.333f);

            case GOV_STATE_THROTTLE_OFF:
            case GOV_STATE_THROTTLE_HOLD:
            case GOV_STATE_THROTTLE_IDLE:
                return false;
        }
        return false;
    }
    else {
        return (gov.throttleOutput > 0.333f);
    }

    return false;
}


//// Internal functions

static void govDebugStats(void)
{
    DEBUG(GOVERNOR, 0, gov.requestedHeadSpeed);
    DEBUG(GOVERNOR, 1, gov.targetHeadSpeed);
    DEBUG(GOVERNOR, 2, gov.voltageCompGain * 1000);
    DEBUG(GOVERNOR, 3, gov.pidSum * 1000);
    DEBUG(GOVERNOR, 4, gov.P * 1000);
    DEBUG(GOVERNOR, 5, gov.I * 1000);
    DEBUG(GOVERNOR, 6, gov.D * 1000);
    DEBUG(GOVERNOR, 7, gov.F * 1000);
}

static inline bool isAutorotation(void)
{
    return IS_RC_MODE_ACTIVE(BOXAUTOROTATION) || gov.useAutoRotation;
}

static inline bool isGovPassthrough(void)
{
    return IS_RC_MODE_ACTIVE(BOXGOVPASSTHRU) || gov.usePassthrough;
}

static inline bool isForcedRPMError(void)
{
    return IS_RC_MODE_ACTIVE(BOXGOVRPMERROR);
}

static inline void govChangeState(govState_e futureState)
{
    if (gov.state != futureState) {
        gov.state = futureState;
        gov.stateEntryTime = millis();
    }
}

static inline long govStateTime(void)
{
    return cmp32(millis(),gov.stateEntryTime);
}

static void govGetInputThrottle(void)
{
    bool throloff = (getThrottleStatus() == THROTTLE_LOW);
    float throttle = getThrottle();

    if (gov.useThreePosThrottle) {
        if (throttle < 0.333f) {
            throttle = 0;
            throloff = true;
        }
        else if (throttle < 0.666f) {
            throttle = gov.idleThrottle;
        }
        else {
            throttle = 1.0f;
        }
    }

    if (gov.useHsAdjustment && throttle > gov.handoverThrottle) {
        gov.requestedHeadSpeed = throttle * gov.fullHeadSpeed;
        gov.throttleInput = 1.0f;
    }
    else {
        gov.throttleInput = throttle;
    }

    gov.throttleInputOff = throloff;
}

static float precompCurve(float angle, uint8_t curve)
{
    float drag;

    angle = fabsf(angle);

    switch (curve) {
        case 0:
            drag = angle;
            break;
        case 1:
            drag = angle * sqrtf(angle);
            break;
        case 2:
            drag = angle * angle;
            break;
        case 3:
            drag = angle * angle * sqrtf(angle);
            break;
        case 4:
            drag = angle * angle * angle;
            break;
        default:
            drag = 0;
            break;
    }

    return drag;
}

static void govDataUpdate(void)
{
    // Calculate effective throttle
    govGetInputThrottle();

    // Assume motor[0]
    const float motorRPM = getMotorRawRPMf(0);

    // RPM signal is noisy - filtering is required
    const float filteredRPM = filterApply(&gov.motorRPMFilter, motorRPM);

    // Calculate headspeed from filtered motor speed
    gov.currentHeadSpeed = filteredRPM * gov.mainGearRatio;

    // Calculate current HS vs FullHS ratio
    gov.fullHeadSpeedRatio = gov.currentHeadSpeed / gov.fullHeadSpeed;

    // Detect stuck motor / startup problem
    const bool rpmError = ((gov.fullHeadSpeedRatio < GOV_HS_INVALID_RATIO || motorRPM < 10) && gov.throttleOutput > GOV_HS_INVALID_THROTTLE);

    // Detect RPM glitches
    const bool rpmGlitch = (fabsf(motorRPM - filteredRPM) > gov.motorRPMGlitchDelta || motorRPM > gov.motorRPMGlitchLimit);

    // Error cases
    const bool noErrors = !rpmError && !rpmGlitch && !isForcedRPMError();

    // Evaluate RPM signal quality
    if (noErrors && motorRPM > 0 && gov.fullHeadSpeedRatio > GOV_HS_DETECT_RATIO) {
        if (!gov.motorRPMDetectTime) {
            gov.motorRPMDetectTime = millis();
        }
    }
    else {
        if (gov.motorRPMDetectTime) {
            gov.motorRPMDetectTime = 0;
        }
    }

    // Headspeed is present if RPM is stable long enough
    gov.motorRPMGood = (gov.motorRPMDetectTime && cmp32(millis(),gov.motorRPMDetectTime) > GOV_HS_DETECT_DELAY);

    // Battery voltage required
    if (gov.useVoltageComp) {
        const float Vnom = getBatteryCellCount() * GOV_NOMINAL_CELL_VOLTAGE;
        const float Vbat = getBatteryVoltageMeter()->sample * 0.001f;

        if (Vnom > 2 && Vbat > 2) {
            gov.motorVoltage = filterApply(&gov.motorVoltageFilter, Vbat);
            gov.nominalVoltage = Vnom;
            if (gov.motorVoltage > 1)
                gov.voltageCompGain = constrainf(gov.nominalVoltage / gov.motorVoltage, 0.80f, 1.2f);
            else
                gov.voltageCompGain = 1;
        }
    }

    // Calculate request ratio (HS or throttle)
    if (gov.govMode == GOV_MODE_EXTERNAL) {
        if (gov.throttleInput > 0)
            gov.requestRatio = gov.throttleOutput / gov.throttleInput;
        else
            gov.requestRatio = 0;
    }
    else {
        if (gov.motorRPMGood)
            gov.requestRatio = gov.currentHeadSpeed / gov.requestedHeadSpeed;
        else
            gov.requestRatio = 0;
    }

    // All precomps and feedforwards
    if (gov.usePrecomp) {
        if (gov.useDirectPrecomp) {
            // Use throttle input directly as F-term
            gov.F = gov.throttleInput;
        }
        else {
            // Calculate feedforward from collective deflection
            const float collectiveFF = gov.collectiveWeight * precompCurve(getCollectiveDeflectionAbs(), gov.collectiveCurve);

            // Calculate feedforward from cyclic deflection
            const float cyclicFF = gov.cyclicWeight * getCyclicDeflection();

            // Calculate feedforward from yaw deflection
            const float yawFF = gov.yawWeight * getYawDeflectionAbs();

            // Total feedforward / precomp
            float totalFF = collectiveFF + cyclicFF + yawFF;

            // Filtered FeedForward
            totalFF = filterApply(&gov.precompFilter, totalFF);

            // F-term
            gov.F = gov.K * gov.Kf * totalFF;
        }
    }
    else {
        gov.F = 0;
    }

    // Tail Torque Assist
    if (gov.useTTA) {
        float YAW = mixerGetInput(MIXER_IN_STABILIZED_YAW);
        float TTA = filterApply(&gov.TTAFilter, YAW) * getSpoolUpRatio() * gov.ttaGain;
        float headroom = 0;

        if (gov.govMode  == GOV_MODE_ELECTRIC)
            headroom = 2 * fmaxf(1.0f + gov.ttaLimit - gov.fullHeadSpeedRatio, 0);
        else
            headroom = gov.ttaLimit;

        gov.TTAAdd = constrainf(TTA, 0, headroom);

        DEBUG(TTA, 0, YAW * 1000);
        DEBUG(TTA, 1, TTA * 1000);
        DEBUG(TTA, 2, headroom * 1000);
        DEBUG(TTA, 3, gov.TTAAdd * 1000);

        DEBUG(TTA, 4, gov.P * 1000);
        DEBUG(TTA, 5, gov.I * 1000);
        DEBUG(TTA, 6, gov.pidSum * 1000);
        DEBUG(TTA, 7, gov.targetHeadSpeed);
    }
    else {
        gov.TTAAdd = 0;
    }

    // Normalized RPM error
    const float newError = (gov.targetHeadSpeed - gov.currentHeadSpeed) / gov.fullHeadSpeed + gov.TTAAdd;
    const float newDiff = difFilterApply(&gov.differentiator, newError);

    // Update PIDF terms
    gov.P = gov.K * gov.Kp * newError;
    gov.C = gov.K * gov.Ki * newError * pidGetDT();
    gov.D = gov.K * gov.Kd * newDiff;
}


static void govOutputThrottleControl(float throttle, float min, float max, float rate)
{
    // Limit input value range
    throttle = constrainf(throttle, min, max);

    // Limit rate
    gov.throttleOutput = slewLimit(gov.throttleOutput, throttle, rate);
}

static inline void govThrottleSlewControl(float rate, float min, float max)
{
    // Set output throttle
    govOutputThrottleControl(gov.throttleInput, min, max, rate);

    // Update headspeed target
    gov.targetHeadSpeed = gov.currentHeadSpeed;
}

static void govSpoolupInit(void)
{
    gov.hsSpoolupActive = false;
}

static void govSpoolupControl(float rate, float min, float max)
{
    if (gov.hsSpoolupActive)
    {
        // PID limits
        gov.P = constrainf(gov.P, -gov.lp, gov.LP);
        gov.I = constrainf(gov.I, 0, gov.Li);
        gov.D = 0;
        gov.F = 0;

        // Governor PI sum
        gov.pidSum = gov.P + gov.I + gov.C;

        // Throttle value
        const float throttle = gov.pidSum;

        // Apply gov.C if output not saturated
        if (!((throttle > max && gov.C > 0) || (throttle < min && gov.C < 0)))
            gov.I += gov.C;

        // Set governor output throttle
        govOutputThrottleControl(throttle, min, max, rate);

        // Update headspeed target
        gov.targetHeadSpeed = slewLimit(gov.targetHeadSpeed, gov.requestedHeadSpeed, rate * gov.fullHeadSpeed * GOV_THROTTLE_HEADROOM);
    }
    else
    {
        // Throttle ramp up
        govThrottleSlewControl(rate, min, max);

        // Move to HS rampup, if applicable
        if (gov.usePidSpoolup && gov.throttleInput > gov.handoverThrottle && gov.throttleOutput > gov.handoverThrottle)
        {
            // Update gov.I from current throttle
            gov.I = gov.throttleOutput;

            // Now HS control active
            gov.hsSpoolupActive = true;
        }
    }
}

static void govPIDInit(void)
{
    // Expected PID output
    float pidTarget = gov.throttleOutput / gov.voltageCompGain;

    // PID limits
    gov.P = constrainf(gov.P, -gov.lp, gov.LP);
    gov.D = constrainf(gov.D, -gov.Ld, gov.Ld);
    gov.F = constrainf(gov.F,       0, gov.Lf);

    // Use gov.I to reach the target
    gov.I = pidTarget - (gov.P + gov.D + gov.F);
}

static void govPIDControl(float rate, float min, float max)
{
    DEBUG(GOV_HSK, 0, gov.requestedHeadSpeed);
    DEBUG(GOV_HSK, 1, gov.targetHeadSpeed);
    DEBUG(GOV_HSK, 2, gov.currentHeadSpeed);
    DEBUG(GOV_HSK, 3, gov.pidSum * 1000);

    // Update motor constant
    if (gov.fullHeadSpeedRatio > 0.25f && gov.I > 0.25f) {
        const float HSK = gov.currentHeadSpeed / (gov.I * gov.voltageCompGain);
        gov.motorHSK = ewma1FilterApply(&gov.motorHSKFilter, HSK);
        DEBUG(GOV_HSK, 4, HSK);
    }

    DEBUG(GOV_HSK, 5, gov.motorHSK);

    // Dynamic min throttle
    if (gov.useDynMinThrottle) {
        if (gov.motorHSK > gov.fullHeadSpeed) {
            const float throttleEst = gov.targetHeadSpeed / gov.motorHSK;
            min = fmaxf(min, throttleEst * gov.dynMinLevel); // GOV_DYN_MIN_THROTTLE_LIMIT
            DEBUG(GOV_HSK, 6, throttleEst);
            DEBUG(GOV_HSK, 7, min);
        }
    }

    // PID limits
    gov.P = constrainf(gov.P, -gov.lp, gov.LP);
    gov.I = constrainf(gov.I,       0, gov.Li);
    gov.D = constrainf(gov.D, -gov.Ld, gov.Ld);
    gov.F = constrainf(gov.F,       0, gov.Lf);

    // Governor PIDF sum
    gov.pidSum = gov.P + gov.I + gov.C + gov.D + gov.F;

    // Generate throttle signal
    const float throttle = gov.pidSum * gov.voltageCompGain;

    // Apply gov.C if throttle not saturated
    if (!((throttle > max && gov.C > 0) || (throttle < min && gov.C < 0)))
        gov.I += gov.C;

    // Set output
    govOutputThrottleControl(throttle, min, max, rate);

    // Estimate HS change speed
    const float hsRate = fmaxf(gov.fullHeadSpeed * GOV_THROTTLE_HEADROOM, gov.motorHSK) * rate;

    // Update headspeed target
    gov.targetHeadSpeed = slewLimit(gov.targetHeadSpeed, gov.requestedHeadSpeed, hsRate);
}

static void govFallbackControl(float rate, float min, float max)
{
    // I-term is fixed
    gov.I = gov.baseThrottle;

    // Precomp enabled
    if (gov.useFallbackPrecomp)
        gov.F = constrainf(gov.F, 0, gov.Lf);
    else
        gov.F = 0;

    // Fallback "PID sum"
    gov.pidSum = gov.I + gov.F;

    // Generate throttle signal
    const float throttle = gov.pidSum * gov.voltageCompGain;

    // Set output
    govOutputThrottleControl(throttle, min, max, rate);
}

static void govRecoveryInit(void)
{
    // Headspeed is still reasonably high. Motor constant has been acquired.
    //  => calculate estimated throttle for the target headspeed
    if (gov.motorRPMGood && gov.fullHeadSpeedRatio > 0.25f && gov.motorHSK > gov.fullHeadSpeed) {
        gov.throttleOutput = fminf(gov.targetHeadSpeed / gov.motorHSK, gov.requestRatio) * 0.95f;
    }
}


/*
 * External throttle control (Ext.Gov)
 */

static void govUpdateExternalThrottle(void)
{
    float throttle = 0;

    switch (gov.state) {
        case GOV_STATE_THROTTLE_OFF:
            throttle = 0;
            break;
        case GOV_STATE_THROTTLE_HOLD:
        case GOV_STATE_THROTTLE_IDLE:
            throttle = slewUpLimit(gov.throttlePrevInput, gov.throttleInput, gov.throttleStartupRate);
            break;
        case GOV_STATE_SPOOLUP:
            throttle = slewUpLimit(gov.throttlePrevInput, gov.throttleInput, gov.throttleSpoolupRate);
            break;
        case GOV_STATE_ACTIVE:
        case GOV_STATE_FALLBACK:
        case GOV_STATE_AUTOROTATION:
            throttle = slewLimit(gov.throttlePrevInput, gov.throttleInput, gov.throttleTrackingRate);
            break;
        case GOV_STATE_RECOVERY:
        case GOV_STATE_BAILOUT:
            throttle = slewUpLimit(gov.throttlePrevInput, gov.throttleInput, gov.throttleRecoveryRate);
            break;
        case GOV_STATE_PASSTHRU:
            throttle = gov.throttleInput;
            break;
        default:
            break;
    }

    gov.throttlePrevInput = throttle;

    if (gov.state == GOV_STATE_ACTIVE && gov.useTTA) {
        throttle += throttle * gov.TTAAdd;
    }

    gov.throttleOutput = throttle;
}

static void govUpdateExternalState(void)
{
    // Handle DISARM separately for SAFETY!
    if (!ARMING_FLAG(ARMED)) {
        govChangeState(GOV_STATE_THROTTLE_OFF);
    }
    else {
        switch (gov.state)
        {
            // Throttle is OFF
            case GOV_STATE_THROTTLE_OFF:
                if (isGovPassthrough())
                    govChangeState(GOV_STATE_PASSTHRU);
                else if (!gov.throttleInputOff)
                    govChangeState(GOV_STATE_THROTTLE_IDLE);
                break;

            // Throttle is IDLE, follow with a limited ramupup rate.
            //  -- If NO throttle, move to THROTTLE_OFF
            //  -- if throttle > handover, move to SPOOLUP
            case GOV_STATE_THROTTLE_IDLE:
                if (isGovPassthrough())
                    govChangeState(GOV_STATE_PASSTHRU);
                else if (gov.throttleInputOff)
                    govChangeState(GOV_STATE_THROTTLE_OFF);
                else if (gov.throttleOutput > gov.handoverThrottle)
                    govChangeState(GOV_STATE_SPOOLUP);
                break;

            // Throttle is low. If it is a mistake, give a chance to recover.
            //  -- If throttle returns, move to RECOVERY
            //  -- When timer expires, move to OFF
            case GOV_STATE_THROTTLE_HOLD:
                if (gov.throttleInput > gov.handoverThrottle)
                    govChangeState(GOV_STATE_RECOVERY);
                else if (govStateTime() > gov.throttleHoldTimeout) {
                    if (gov.throttleInputOff)
                        govChangeState(GOV_STATE_THROTTLE_OFF);
                    else
                        govChangeState(GOV_STATE_THROTTLE_IDLE);
                }
                break;

            // Follow the throttle, with a limited ramp up rate.
            //  -- If NO throttle, move to THROTTLE_OFF
            //  -- If 0% < throttle < handover, stay in spoolup
            //  -- Once throttle > handover and not ramping up, move to ACTIVE
            case GOV_STATE_SPOOLUP:
                if (gov.throttleInputOff)
                    govChangeState(GOV_STATE_THROTTLE_OFF);
                else if (gov.throttleInput < gov.handoverThrottle)
                    govChangeState(GOV_STATE_THROTTLE_IDLE);
                else if (gov.throttleOutput >= gov.throttleInput)
                    govChangeState(GOV_STATE_ACTIVE);
                break;

            // Follow the throttle without ramp limits.
            //  -- If NO throttle, move to THROTTLE_HOLD
            //  -- If throttle < handover, move to AUTO or SPOOLING_UP
            case GOV_STATE_ACTIVE:
                if (gov.throttleInputOff)
                    govChangeState(GOV_STATE_THROTTLE_HOLD);
                else if (gov.throttleInput < gov.handoverThrottle) {
                    if (isAutorotation())
                        govChangeState(GOV_STATE_AUTOROTATION);
                    else
                        govChangeState(GOV_STATE_THROTTLE_HOLD);
                }
                break;

            // Follow the throttle, with a high(er) ramp rate.
            //  -- Once throttle is not ramping up any more, move to ACTIVE
            //  -- If throttle < handover, move to IDLE
            case GOV_STATE_RECOVERY:
                if (gov.throttleInputOff)
                    govChangeState(GOV_STATE_THROTTLE_OFF);
                else if (gov.throttleInput < gov.handoverThrottle)
                    govChangeState(GOV_STATE_THROTTLE_IDLE);
                else if (gov.throttleOutput >= gov.throttleInput)
                    govChangeState(GOV_STATE_ACTIVE);
                break;

            // Follow throttle with high(er) ramp rate.
            //  -- If throttle is > handover, move to BAILOUT
            //  -- If timer expires, move to IDLE
            case GOV_STATE_AUTOROTATION:
                if (gov.throttleInputOff)
                    govChangeState(GOV_STATE_THROTTLE_HOLD);
                else if (gov.throttleInput > gov.handoverThrottle)
                    govChangeState(GOV_STATE_BAILOUT);
                else if (!isAutorotation())
                    govChangeState(GOV_STATE_THROTTLE_IDLE);
                break;

            // Follow the throttle, with a high(er) ramp rate.
            //  -- Once throttle is not ramping up any more, move to ACTIVE
            //  -- If throttle < handover, move back to AUTO
            case GOV_STATE_BAILOUT:
                if (gov.throttleInputOff)
                    govChangeState(GOV_STATE_THROTTLE_HOLD);
                else if (gov.throttleInput < gov.handoverThrottle)
                    govChangeState(GOV_STATE_AUTOROTATION);
                else if (gov.throttleOutput >= gov.throttleInput)
                    govChangeState(GOV_STATE_ACTIVE);
                break;

            // Passthrough: Direct throttle to output
            //  -- If governor enabled, move back to approriate state
            case GOV_STATE_PASSTHRU:
                if (!isGovPassthrough()) {
                    if (gov.throttleInputOff)
                        govChangeState(GOV_STATE_THROTTLE_OFF);
                    else if (gov.throttleInput < gov.handoverThrottle)
                        govChangeState(GOV_STATE_THROTTLE_IDLE);
                    else
                        govChangeState(GOV_STATE_SPOOLUP);
                }
                break;

            // Should not be here
            default:
                govChangeState(GOV_STATE_THROTTLE_OFF);
                break;
        }
    }

    // Get throttle
    govUpdateExternalThrottle();
}


/*
 * Electric motor speed control
 */

static inline void govEnterActiveState(void)
{
    govChangeState(GOV_STATE_ACTIVE);
    govPIDInit();
}

static inline void govEnterSpoolupState(void)
{
    govChangeState(GOV_STATE_SPOOLUP);
    govSpoolupInit();
}

static inline void govEnterRecoveryState(void)
{
    govChangeState(GOV_STATE_RECOVERY);
    govRecoveryInit();
}

static inline void govEnterBailoutState(void)
{
    govChangeState(GOV_STATE_BAILOUT);
    govRecoveryInit();
}

static void govUpdateGovernedThrottle(void)
{
    switch (gov.state) {
        case GOV_STATE_THROTTLE_OFF:
            gov.targetHeadSpeed = 0;
            gov.throttleOutput = 0;
            break;
        case GOV_STATE_THROTTLE_IDLE:
            govThrottleSlewControl(gov.throttleStartupRate, gov.idleThrottle, gov.handoverThrottle);
            break;
        case GOV_STATE_THROTTLE_HOLD:
            if (gov.throttleInputOff)
                gov.throttleOutput = 0;
            else
                govThrottleSlewControl(gov.throttleTrackingRate, gov.idleThrottle, gov.handoverThrottle);
            break;
        case GOV_STATE_SPOOLUP:
            govSpoolupControl(gov.throttleSpoolupRate, gov.minSpoolupThrottle, gov.maxSpoolupThrottle);
            break;
        case GOV_STATE_ACTIVE:
            govPIDControl(gov.throttleTrackingRate, gov.minActiveThrottle, gov.maxActiveThrottle);
            break;
        case GOV_STATE_FALLBACK:
            govFallbackControl(gov.throttleTrackingRate, gov.minActiveThrottle, gov.maxActiveThrottle);
            break;
        case GOV_STATE_AUTOROTATION:
            govThrottleSlewControl(gov.throttleTrackingRate, gov.autoThrottle, gov.maxActiveThrottle);
            break;
        case GOV_STATE_RECOVERY:
        case GOV_STATE_BAILOUT:
            govThrottleSlewControl(gov.throttleRecoveryRate, gov.minSpoolupThrottle, gov.maxSpoolupThrottle);
            break;
        case GOV_STATE_PASSTHRU:
            gov.targetHeadSpeed = 0;
            gov.throttleOutput = gov.throttleInput;
            break;
        default:
            break;
    }
}

static void govUpdateGovernedState(void)
{
    // Handle DISARM separately for SAFETY!
    if (!ARMING_FLAG(ARMED)) {
        govChangeState(GOV_STATE_THROTTLE_OFF);
    }
    else {
        switch (gov.state)
        {
            // Throttle is OFF
            case GOV_STATE_THROTTLE_OFF:
                if (isGovPassthrough())
                    govChangeState(GOV_STATE_PASSTHRU);
                else if (!gov.throttleInputOff)
                    govChangeState(GOV_STATE_THROTTLE_IDLE);
                break;

            // Throttle is IDLE, follow with a limited startup rate
            //  -- If NO throttle, move to THROTTLE_OFF
            //  -- if throttle > handover and stable RPM, move to SPOOLUP
            case GOV_STATE_THROTTLE_IDLE:
                if (isGovPassthrough())
                    govChangeState(GOV_STATE_PASSTHRU);
                else if (gov.throttleInputOff)
                    govChangeState(GOV_STATE_THROTTLE_OFF);
                else if (gov.throttleInput > gov.handoverThrottle && gov.motorRPMGood)
                    govEnterSpoolupState();
                break;

            // Throttle is moved from high to low. If it is a mistake, give a chance to recover
            //  -- When throttle returns high, move to RECOVERY
            //  -- When timer expires, move to OFF/IDLE
            case GOV_STATE_THROTTLE_HOLD:
                if (gov.throttleInput > gov.handoverThrottle && gov.motorRPMGood)
                    govEnterRecoveryState();
                else if (govStateTime() > gov.throttleHoldTimeout) {
                    if (gov.throttleInputOff)
                        govChangeState(GOV_STATE_THROTTLE_OFF);
                    else
                        govChangeState(GOV_STATE_THROTTLE_IDLE);
                }
                break;

            // Ramp up throttle until target is reached
            //  -- Once 99% headspeed reached, move to ACTIVE
            //  -- If throttle reaches 95% before headspeed target, also move to ACTIVE
            //  -- If throttle < handover, move back to IDLE
            //  -- If no headspeed detected, move to IDLE
            //  -- If NO throttle, move to THROTTLE_OFF
            case GOV_STATE_SPOOLUP:
                if (gov.throttleInputOff)
                    govChangeState(GOV_STATE_THROTTLE_OFF);
                else if (gov.throttleInput < gov.handoverThrottle)
                    govChangeState(GOV_STATE_THROTTLE_IDLE);
                else if (!gov.motorRPMGood)
                    govChangeState(GOV_STATE_THROTTLE_IDLE);
                else if (gov.currentHeadSpeed > gov.requestedHeadSpeed * 0.99f || gov.throttleOutput > gov.maxSpoolupThrottle * 0.95f)
                    govEnterActiveState();
                break;

            // Governor active, maintain headspeed
            //  -- If NO throttle, move to THROTTLE_HOLD
            //  -- If no headspeed signal, move to FALLBACK
            //  -- If throttle < handover, move to AUTOROTATION or LOW
            case GOV_STATE_ACTIVE:
                if (gov.throttleInputOff)
                    govChangeState(GOV_STATE_THROTTLE_HOLD);
                else if (!gov.motorRPMGood)
                    govChangeState(GOV_STATE_FALLBACK);
                else if (gov.throttleInput < gov.handoverThrottle) {
                    if (isAutorotation())
                        govChangeState(GOV_STATE_AUTOROTATION);
                    else
                        govChangeState(GOV_STATE_THROTTLE_HOLD);
                }
                break;

            // Fallback: No headspeed signal. Use curves.
            //  -- If NO throttle, move to LOW
            //  -- If headspeed recovers, move to RECOVERY or IDLE
            //  -- When timer expires, move to OFF
            case GOV_STATE_FALLBACK:
                if (gov.throttleInputOff)
                    govChangeState(GOV_STATE_THROTTLE_HOLD);
                else if (gov.throttleInput < gov.handoverThrottle)
                    govChangeState(GOV_STATE_THROTTLE_HOLD);
                else if (gov.motorRPMGood)
                    govEnterRecoveryState();
                break;

            // Ramp up throttle until target headspeed is reached, with fast(er) ramp up.
            //  -- If NO throttle, move to THROTTLE_OFF
            //  -- If no headspeed detected, move to IDLE
            //  -- If throttle reaches 95%, move to ACTIVE
            //  -- Once 99% headspeed reached, move to ACTIVE
            case GOV_STATE_RECOVERY:
                if (gov.throttleInputOff)
                    govChangeState(GOV_STATE_THROTTLE_OFF);
                else if (gov.throttleInput < gov.handoverThrottle)
                    govChangeState(GOV_STATE_THROTTLE_IDLE);
                else if (!gov.motorRPMGood)
                    govChangeState(GOV_STATE_THROTTLE_IDLE);
                else if (gov.currentHeadSpeed > gov.requestedHeadSpeed * 0.99f || gov.throttleOutput > gov.maxSpoolupThrottle * 0.95f)
                    govEnterActiveState();
                break;

            // Throttle passthrough with ramp up limit
            //  -- If NO throttle, move to THROTTLE_OFF
            //  -- If throttle > handover, move to BAILOUT
            //  -- If timer expires, move to IDLE
            //  -- Map throttle to motor output
            case GOV_STATE_AUTOROTATION:
                if (gov.throttleInputOff)
                    govChangeState(GOV_STATE_THROTTLE_HOLD);
                else if (gov.throttleInput > gov.handoverThrottle && gov.motorRPMGood)
                    govEnterBailoutState();
                else if (!isAutorotation())
                    govChangeState(GOV_STATE_THROTTLE_IDLE);
                break;

            // Ramp up throttle until target headspeed is reached, with fast(er) ramp up.
            //  -- If NO throttle, move to THROTTLE_HOLD
            //  -- If no headspeed detected, move back to AUTOROTATION
            //  -- If throttle < handover, move back to AUTOROTATION
            //  -- If throttle reaches 95%, move to ACTIVE
            //  -- Once 99% headspeed reached, move to ACTIVE
            case GOV_STATE_BAILOUT:
                if (gov.throttleInputOff)
                    govChangeState(GOV_STATE_THROTTLE_HOLD);
                else if (gov.throttleInput < gov.handoverThrottle)
                    govChangeState(GOV_STATE_AUTOROTATION);
                else if (!gov.motorRPMGood)
                    govChangeState(GOV_STATE_AUTOROTATION);
                else if (gov.currentHeadSpeed > gov.requestedHeadSpeed * 0.99f || gov.throttleOutput > gov.maxSpoolupThrottle * 0.95f)
                    govEnterActiveState();
                break;

            // Governor deactivated: Direct throttle to output
            //  -- If governor enabled, move back to approriate state
            case GOV_STATE_PASSTHRU:
                if (!isGovPassthrough()) {
                    if (gov.throttleInputOff)
                        govChangeState(GOV_STATE_THROTTLE_OFF);
                    else if (gov.throttleInput < gov.handoverThrottle)
                        govChangeState(GOV_STATE_THROTTLE_IDLE);
                    else
                        govEnterSpoolupState();
                }
                break;

            // Should not be here
            default:
                govChangeState(GOV_STATE_THROTTLE_OFF);
                break;
        }
    }

    // Update throttle and HS target
    govUpdateGovernedThrottle();
}


//// Interface functions

void governorUpdate(void)
{
    // Governor is active
    if (gov.govMode)
    {
        // Update internal data
        govDataUpdate();

        // Run state machine
        if (gov.govMode == GOV_MODE_EXTERNAL)
            govUpdateExternalState();
        else
            govUpdateGovernedState();

        // Set debug
        govDebugStats();
    }
    else
    {
        // Straight passthrough
        gov.throttleOutput = gov.throttleInput;
    }
}

void INIT_CODE governorInitProfile(const pidProfile_t *pidProfile)
{
    if (gov.govMode)
    {
        gov.usePassthrough = (pidProfile->governor.flags & BIT(GOV_FLAG_PASSTHRU));
        gov.useDynMinThrottle = (pidProfile->governor.flags & BIT(GOV_FLAG_DYN_MIN_THROTTLE)) && !gov.usePassthrough;
        gov.useThreePosThrottle = (pidProfile->governor.flags & BIT(GOV_FLAG_3POS_THROTTLE)) && !gov.usePassthrough;
        gov.usePidSpoolup = (pidProfile->governor.flags & BIT(GOV_FLAG_PID_SPOOLUP)) && !gov.usePassthrough;
        gov.useHsAdjustment = (pidProfile->governor.flags & BIT(GOV_FLAG_HS_ON_THROTTLE)) && !gov.useThreePosThrottle && gov.usePidSpoolup;
        gov.useVoltageComp = (pidProfile->governor.flags & BIT(GOV_FLAG_VOLTAGE_COMP)) && (gov.govMode == GOV_MODE_ELECTRIC) && (getBatteryVoltageSource() == VOLTAGE_METER_ADC);
        gov.useDirectPrecomp = (pidProfile->governor.flags & BIT(GOV_FLAG_DIRECT_PRECOMP)) && gov.usePrecomp && !gov.useHsAdjustment && !gov.usePassthrough;
        gov.useFallbackPrecomp = (pidProfile->governor.flags & BIT(GOV_FLAG_FALLBACK_PRECOMP)) && gov.usePrecomp && !gov.usePassthrough;
        gov.useAutoRotation = (pidProfile->governor.flags & BIT(GOV_FLAG_AUTOROTATION)) && !gov.usePassthrough;
        gov.usePrecomp = (pidProfile->governor.f_gain > 0) && !gov.usePassthrough;

        gov.K  = pidProfile->governor.gain / 100.0f;
        gov.Kp = pidProfile->governor.p_gain / 10.0f;
        gov.Ki = pidProfile->governor.i_gain / 10.0f;
        gov.Kd = pidProfile->governor.d_gain / 1000.0f;
        gov.Kf = pidProfile->governor.f_gain / 100.0f;

        gov.LP = pidProfile->governor.p_limits[0] / 100.0f;
        gov.lp = (pidProfile->governor.p_limits[1] ? pidProfile->governor.p_limits[1] : pidProfile->governor.p_limits[0]) / 100.0f;
        gov.Li = pidProfile->governor.i_limit / 100.0f;
        gov.Ld = pidProfile->governor.d_limit / 100.0f;
        gov.Lf = pidProfile->governor.f_limit / 100.0f;

        gov.idleThrottle = pidProfile->governor.idle_throttle / 100.0f;
        gov.baseThrottle = pidProfile->governor.base_throttle / 100.0f;
        gov.autoThrottle = pidProfile->governor.auto_throttle / 100.0f;

        gov.minActiveThrottle = pidProfile->governor.min_throttle / 100.0f;
        gov.maxActiveThrottle = pidProfile->governor.max_throttle / 100.0f;

        gov.minSpoolupThrottle = gov.idleThrottle;
        gov.maxSpoolupThrottle = gov.maxActiveThrottle;

        if (pidProfile->governor.tta_gain) {
            gov.ttaGain = mixerRotationSign() * pidProfile->governor.tta_gain / -125.0f;
            gov.ttaLimit = pidProfile->governor.tta_limit / 100.0f;
            gov.useTTA = true;

            if (gov.govMode == GOV_MODE_ELECTRIC)
                gov.ttaGain /= gov.K * gov.Kp;
        }
        else {
            gov.ttaGain = 0;
            gov.ttaLimit = 0;
            gov.useTTA = false;
        }

        gov.voltageCompGain = 1;

        gov.dynMinLevel = pidProfile->governor.dyn_min_level / 100.0f; // TDB remove

        gov.yawWeight = pidProfile->governor.yaw_weight / 100.0f;
        gov.cyclicWeight = pidProfile->governor.cyclic_weight / 100.0f;
        gov.collectiveWeight = pidProfile->governor.collective_weight / 100.0f;
        gov.collectiveCurve = pidProfile->governor.collective_curve;

        gov.fullHeadSpeed = constrainf(pidProfile->governor.headspeed, 100, 50000);
        gov.fullHeadSpeedRatio = 1;

        gov.requestedHeadSpeed = gov.fullHeadSpeed;

        gov.motorRPMGlitchDelta = (gov.fullHeadSpeed / gov.mainGearRatio) * GOV_HS_GLITCH_DELTA;
        gov.motorRPMGlitchLimit = (gov.fullHeadSpeed / gov.mainGearRatio) * GOV_HS_GLITCH_LIMIT;
    }
}

static inline float govCalcRate(uint16_t param, uint16_t min, uint16_t max)
{
    if (param)
        return 10 * pidGetDT() / constrain(param,min,max);
    else
        return 0;
}

void INIT_CODE governorInit(const pidProfile_t *pidProfile)
{
    if (getMotorCount() > 0)
    {
        gov.state = GOV_STATE_THROTTLE_OFF;

        gov.govMode = governorConfig()->gov_mode;

        switch (gov.govMode)
        {
            case GOV_MODE_EXTERNAL:
                break;
            case GOV_MODE_ELECTRIC:
            case GOV_MODE_NITRO:
                if (!isMotorFastRpmSourceActive(0)) {
                    setArmingDisabled(ARMING_DISABLED_GOVERNOR);
                    setArmingDisabled(ARMING_DISABLED_RPM_SIGNAL);
                    gov.govMode = GOV_MODE_NONE;
                }
                break;
            default:
                gov.govMode = GOV_MODE_NONE;
                break;
        }

        if (gov.govMode)
        {
            gov.mainGearRatio = getMainGearRatio();

            gov.throttleStartupRate  = govCalcRate(governorConfig()->gov_startup_time, 1, 600);
            gov.throttleSpoolupRate  = govCalcRate(governorConfig()->gov_spoolup_time, 1, 600);
            gov.throttleTrackingRate = govCalcRate(governorConfig()->gov_tracking_time, 1, 100);
            gov.throttleRecoveryRate = govCalcRate(governorConfig()->gov_recovery_time, 1, 100);

            gov.throttleHoldTimeout  = governorConfig()->gov_throttle_hold_timeout * 100;

            gov.handoverThrottle = constrain(governorConfig()->gov_handover_throttle, 1, 100) / 100.0f;

            difFilterInit(&gov.differentiator, governorConfig()->gov_d_cutoff / 10.0f, gyro.targetRateHz);

            ewma1FilterInit(&gov.motorHSKFilter, GOV_MOTOR_K_CUTOFF, gyro.targetRateHz);

            lowpassFilterInit(&gov.motorVoltageFilter, LPF_PT2, governorConfig()->gov_pwr_filter, gyro.targetRateHz, 0);
            lowpassFilterInit(&gov.motorRPMFilter, LPF_PT2, governorConfig()->gov_rpm_filter, gyro.targetRateHz, 0);
            lowpassFilterInit(&gov.TTAFilter, LPF_PT2, governorConfig()->gov_tta_filter, gyro.targetRateHz, 0);
            lowpassFilterInit(&gov.precompFilter, LPF_PT2, governorConfig()->gov_ff_filter, gyro.targetRateHz, 0);

            governorInitProfile(pidProfile);
        }
    }
}
