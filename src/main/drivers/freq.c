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

#if defined(USE_FREQ_SENSOR)

#include "build/build_config.h"
#include "build/debug.h"

#include "common/utils.h"

#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/time.h"
#include "drivers/timer.h"
#include "drivers/dshot.h"
#include "drivers/freq.h"

#include "pg/freq.h"


// Accepted frequence range
#define FREQ_RANGE_MIN        10.0
#define FREQ_RANGE_MAX        5000.0

// Prescaler limits
#define FREQ_PRESCALER_MIN    0x0001
#define FREQ_PRESCALER_MAX    0x1000

// Prescaler shift points
#define FREQ_SHIFT_MIN        0x1000
#define FREQ_SHIFT_MAX        0x4000

// Period init value
#define FREQ_PERIOD_INIT      0x2000

// Freq filtering coefficient - 6 is best for 3-phase motor
#define FREQ_FILTER_COEFF     6

// Maximum number of overflow failures
#define FREQ_MAX_FAILURES     3

// Timeout for missing signal in 32bit mode [s]
#define FREQ_TIMEOUT          0.2f

// Input signal max deviation from average 75%..150%
#define FREQ_PERIOD_MIN(p)    ((p)*3/4)
#define FREQ_PERIOD_MAX(p)    ((p)*3/2)


#define FILTER_UPDATE(_var,_value,_coef) \
    ((_var) += ((_value)-(_var))/(_coef))

#define UPDATE_FREQ_FILTER(_input,_freq) \
    FILTER_UPDATE((_input)->freq, _freq, FREQ_FILTER_COEFF)

#define UPDATE_PERIOD_FILTER(_input,_period) \
    FILTER_UPDATE((_input)->period, (int32_t)_period, (_input)->percoef)


typedef struct {

    bool enabled;
    bool timer32;

    float freq;
    float clock;

    int32_t  period;
    int32_t  percoef;
    uint32_t capture;
    uint32_t prescaler;

    uint32_t failures;
    uint32_t overflows;


    timerCCHandlerRec_t edgeCb;
    timerOvrHandlerRec_t overflowCb;

    const timerHardware_t *timerHardware;

} freqInputPort_t;

static FAST_DATA_ZERO_INIT freqInputPort_t freqInputPorts[FREQ_SENSOR_PORT_COUNT];


/*
 * Set the base clock to a frequency that gives a reading in range
 * RANGE_MIN..RANGE_MAX [0x1000..0x4000]. This gives enough resolution,
 * while allowing the signal to change four times slower or faster in one cycle.
 *
 * Also, set the period filter coefficient so that it allows very quick change
 * on low frequencies, but slower change on higher. This is needed because
 * electric motors have lots of torque on low speeds, especially when starting up.
 * We need to be able to adjust to the startup quickly enough.
 */

static const uint8_t perCoeffs[16] = {
     4,  4,  4,  4,
     4,  4,  4,  4,
     4,  6,  8, 16,
    24, 32, 32, 32,
};

static void freqSetBaseClock(freqInputPort_t *input, uint32_t prescaler)
{
    TIM_TypeDef *tim = input->timerHardware->tim;

    input->prescaler = prescaler;
    input->percoef = perCoeffs[(__builtin_clz(prescaler) - 15)];
    input->clock = (float)timerClock(tim) / prescaler;

    tim->PSC = prescaler - 1;
    tim->ARR = 0xffffffff;
    tim->EGR = TIM_EGR_UG;
}

static FAST_CODE void freqOverflowCallback16(timerOvrHandlerRec_t *cbRec, captureCompare_t capture)
{
    UNUSED(capture);
    freqInputPort_t *input = container_of(cbRec, freqInputPort_t, overflowCb);

    input->overflows++;

    // Two overflows means no signal for a whole period
    if (input->overflows > 1) {

        input->failures++;

        // Reset after too many dead periods
        if (input->failures > FREQ_MAX_FAILURES) {
            input->freq = 0;
            input->period = FREQ_PERIOD_INIT;
            input->failures = 0;

            freqSetBaseClock(input, FREQ_PRESCALER_MAX);
        }

        input->overflows = 0;
        input->capture = 0;
    }
}

static FAST_CODE void freqEdgeCallback16(timerCCHandlerRec_t *cbRec, captureCompare_t capture)
{
    freqInputPort_t *input = container_of(cbRec, freqInputPort_t, edgeCb);

    if (input->enabled) {
        if (input->capture) {
            // Must use uint16 here because of wraparound
            uint16_t period = capture - input->capture;
            float freq = 0;

            UPDATE_PERIOD_FILTER(input, period);

            // Signal conditioning. Update freq filter only if period within acceptable range.
            if (period > FREQ_PERIOD_MIN(input->period) && period < FREQ_PERIOD_MAX(input->period)) {
                freq = input->clock / period;
                if (freq > FREQ_RANGE_MIN && freq < FREQ_RANGE_MAX) {
                    UPDATE_FREQ_FILTER(input, freq);
                }
            }

            const uint8_t index = input - freqInputPorts;
            DEBUG_AXIS(FREQ_SENSOR, index, 0, input->freq * 1000);
            DEBUG_AXIS(FREQ_SENSOR, index, 1, freq * 1000);
            DEBUG_AXIS(FREQ_SENSOR, index, 2, capture);
            DEBUG_AXIS(FREQ_SENSOR, index, 3, period);
            DEBUG_AXIS(FREQ_SENSOR, index, 4, input->period);
            DEBUG_AXIS(FREQ_SENSOR, index, 5, input->percoef);
            DEBUG_AXIS(FREQ_SENSOR, index, 6, 32 - __builtin_clz(input->prescaler));

            // Filtered period out of range. Change prescaler.
            if (input->period < FREQ_SHIFT_MIN && input->prescaler > FREQ_PRESCALER_MIN) {
                freqSetBaseClock(input, input->prescaler >> 1);
                input->period <<= 1;
                capture = 0;
            }
            else if (input->period > FREQ_SHIFT_MAX && input->prescaler < FREQ_PRESCALER_MAX) {
                freqSetBaseClock(input, input->prescaler << 1);
                input->period >>= 1;
                capture = 0;
            }

            input->failures = 0;
        }

        input->overflows = 0;
        input->capture = capture;
    }
}

static FAST_CODE void freqEdgeCallback32(timerCCHandlerRec_t *cbRec, captureCompare_t capture16)
{
    UNUSED(capture16);

    freqInputPort_t *input = container_of(cbRec, freqInputPort_t, edgeCb);

    if (input->enabled) {
        uint32_t capture = *timerChCCR(input->timerHardware);

        if (input->capture) {
            uint32_t period = capture - input->capture;

            float freq = input->clock / period;

            if (freq > FREQ_RANGE_MIN && freq < FREQ_RANGE_MAX) {
                UPDATE_FREQ_FILTER(input, freq);
            }

            const uint8_t index = input - freqInputPorts;
            DEBUG_AXIS(FREQ_SENSOR, index, 0, input->freq * 1000);
            DEBUG_AXIS(FREQ_SENSOR, index, 1, freq * 1000);
            DEBUG_AXIS(FREQ_SENSOR, index, 2, capture);
            DEBUG_AXIS(FREQ_SENSOR, index, 3, period);
        }

        input->capture = capture;
    }
}

#if defined(USE_HAL_DRIVER)
void freqICConfig(const timerHardware_t *timer, bool rising, uint16_t filter)
{
    TIM_HandleTypeDef *handle = timerFindTimerHandle(timer->tim);

    if (handle) {
        TIM_IC_InitTypeDef sInitStructure;
        memset(&sInitStructure, 0, sizeof(sInitStructure));

        sInitStructure.ICPolarity = rising ? TIM_ICPOLARITY_RISING : TIM_ICPOLARITY_FALLING;
        sInitStructure.ICSelection = TIM_ICSELECTION_DIRECTTI;
        sInitStructure.ICPrescaler = TIM_ICPSC_DIV1;
        sInitStructure.ICFilter = filter;

        HAL_TIM_IC_ConfigChannel(handle, &sInitStructure, timer->channel);
        HAL_TIM_IC_Start_IT(handle, timer->channel);
    }
}
#else
void freqICConfig(const timerHardware_t *timer, bool rising, uint16_t filter)
{
    TIM_ICInitTypeDef sInitStructure;

    TIM_ICStructInit(&sInitStructure);
    sInitStructure.TIM_Channel = timer->channel;
    sInitStructure.TIM_ICPolarity = rising ? TIM_ICPolarity_Rising : TIM_ICPolarity_Falling;
    sInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    sInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    sInitStructure.TIM_ICFilter = filter;

    TIM_ICInit(timer->tim, &sInitStructure);
}
#endif

void freqInit(const freqConfig_t *freqConfig)
{
    for (int port = 0; port < FREQ_SENSOR_PORT_COUNT; port++) {
        freqInputPort_t *input = &freqInputPorts[port];
        const timerHardware_t *timer = timerAllocate(freqConfig->ioTag[port], OWNER_FREQ, RESOURCE_INDEX(port));
        if (timer) {
            input->timerHardware = timer;
            input->enabled = true;
            input->timer32 = (timer->tim == TIM2 || timer->tim == TIM5);
            input->overflows = 0;
            input->failures = 0;
            input->capture = 0;
            input->period = FREQ_PERIOD_INIT;
            input->freq = 0;

            IO_t io = IOGetByTag(freqConfig->ioTag[port]);
            IOInit(io, OWNER_FREQ, RESOURCE_INDEX(port));
            IOConfigGPIOAF(io, IOCFG_AF_PP_PD, timer->alternateFunction);

            timerConfigure(timer, 0, timerClock(timer->tim));

            if (input->timer32) {
                timerChCCHandlerInit(&input->edgeCb, freqEdgeCallback32);
                timerChConfigCallbacks(timer, &input->edgeCb, NULL);
            }
            else {
                timerChCCHandlerInit(&input->edgeCb, freqEdgeCallback16);
                timerChOvrHandlerInit(&input->overflowCb, freqOverflowCallback16);
                timerChConfigCallbacks(timer, &input->edgeCb, &input->overflowCb);
            }

            freqICConfig(timer, true, 4);
            freqSetBaseClock(input, (input->timer32) ? 1 : FREQ_PRESCALER_MAX);
        }
    }
}

void freqUpdate(void)
{
    for (int port = 0; port < FREQ_SENSOR_PORT_COUNT; port++) {
        freqInputPort_t *input = &freqInputPorts[port];
        if (input->enabled && input->timer32 && input->capture) {
            uint32_t counter = input->timerHardware->tim->CNT;
            float time = (counter - input->capture) / input->clock;
            if (time > FREQ_TIMEOUT) {
                input->freq = 0;
                input->capture = 0;
            }
        }
    }
}

// RTFL: The freq sensor number MUST match the motor number.
// The resource configuration should reflect this requirement.

float freqRead(uint8_t port)
{
    if (port < FREQ_SENSOR_PORT_COUNT) {
        return freqInputPorts[port].freq;
    }
    return 0.0f;
}

uint16_t getFreqSensorRPM(uint8_t port)
{
    if (port < FREQ_SENSOR_PORT_COUNT) {
        // Return eRPM/100 as expected by RPM filter, msp, etc.
        return (uint16_t) (freqInputPorts[port].freq * 60.0f / 100.0f);
    }
    return 0;
}

bool isFreqSensorPortInitialized(uint8_t port)
{
    if (port < FREQ_SENSOR_PORT_COUNT) {
        return freqInputPorts[port].enabled;
    }
    return false;
}

// Now, return true if at least one sensor is enabled
bool isFreqSensorInitialized(void)
{
    for (int port = 0; port < FREQ_SENSOR_PORT_COUNT; port++) {
        if (freqInputPorts[port].enabled) {
            return true;
        }
    }
    return false;
}

#endif
