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
#define FREQ_RANGE_MIN        10
#define FREQ_RANGE_MAX        10000

// Prescaler limits
#define FREQ_PRESCALER_MIN    0x0001
#define FREQ_PRESCALER_MAX    0x1000

// Prescaler shift points
#define FREQ_SHIFT_MIN        0x2000
#define FREQ_SHIFT_MAX        0x8000

// Period filter coef
#define FREQ_PERIOD_FILTER    8

// Frequency filter coef
#define FREQ_FILTER_COEF      0.16666666f

// Timeout for missing signal [125ms]
#define FREQ_TIMEOUT(clk)     ((clk) / 8)


typedef struct {

    bool enabled;
    bool timer32;

    float freq;
    float clock;

    uint32_t pcoef;
    uint32_t period;
    uint32_t capture;
    uint32_t timeout;
    uint32_t overflows;

    uint32_t prescaler;
    uint32_t presc_min;
    uint32_t presc_max;

    timerCCHandlerRec_t edgeCb;
    timerOvrHandlerRec_t overflowCb;

    const timerHardware_t *timerHardware;

} freqInput_t;

static FAST_DATA_ZERO_INIT freqInput_t freqInputs[FREQ_SENSOR_PORT_COUNT];


/*
 * Set the base clock to a frequency that gives a reading in range
 * RANGE_MIN..RANGE_MAX [0x2000..0x8000]. This gives enough resolution,
 * while allowing the signal to change four times slower or faster in one cycle.
 */

static void freqSetBaseClock(freqInput_t *input, uint32_t prescaler)
{
    TIM_TypeDef *tim = input->timerHardware->tim;

    input->prescaler = prescaler;
    input->capture = 0;

    tim->PSC = prescaler - 1;
    tim->ARR = 0xffffffff;
    tim->EGR = TIM_EGR_UG;
}

static FAST_CODE void freqOverflowCallback16(timerOvrHandlerRec_t *cbRec, captureCompare_t capture)
{
    UNUSED(capture);
    freqInput_t *input = container_of(cbRec, freqInput_t, overflowCb);

    input->overflows++;

    // Two overflows means no signal for a whole period
    if (input->overflows > 1) {
        if (input->prescaler < input->presc_max) {
            freqSetBaseClock(input, input->prescaler << 1);
        }
        else {
            input->freq = 0;
        }

        input->overflows = 0;
        input->capture = 0;
    }
}

static FAST_CODE void freqEdgeCallback16(timerCCHandlerRec_t *cbRec, captureCompare_t capture)
{
    freqInput_t *input = container_of(cbRec, freqInput_t, edgeCb);

    if (input->enabled) {
        if (input->capture) {
            // Must use uint16 here because of wrap-around
            const uint16_t delta = capture - input->capture;
            const uint32_t period = input->prescaler * delta;

            // Update period counter
            if (input->pcoef < FREQ_PERIOD_FILTER)
                input->pcoef++;

            // Update period filter
            input->period += (period - input->period) / input->pcoef;

            // Calculate frequency
            const float freq = input->clock / period;

            // Update freq filter
            if (freq > FREQ_RANGE_MIN && freq < FREQ_RANGE_MAX) {
                input->freq += (freq - input->freq) * FREQ_FILTER_COEF;
            }

            const uint8_t index = input - freqInputs;
            DEBUG_AXIS(FREQ_SENSOR, index, 0, input->freq * 1000);
            DEBUG_AXIS(FREQ_SENSOR, index, 1, freq * 1000);
            DEBUG_AXIS(FREQ_SENSOR, index, 2, period);
            DEBUG_AXIS(FREQ_SENSOR, index, 3, input->period);
            DEBUG_AXIS(FREQ_SENSOR, index, 4, 32 - __builtin_clz(input->prescaler));

            // Filtered period out of range. Change prescaler.
            if (input->period < FREQ_SHIFT_MIN && input->prescaler > input->presc_min) {
                freqSetBaseClock(input, input->prescaler >> 1);
                input->pcoef >>= 1;
                capture = 0;
            }
            else if (input->period > FREQ_SHIFT_MAX && input->prescaler < input->presc_max) {
                freqSetBaseClock(input, input->prescaler << 1);
                input->pcoef >>= 1;
                capture = 0;
            }
        }

        input->overflows = 0;
        input->capture = capture;
    }
}

static FAST_CODE void freqEdgeCallback32(timerCCHandlerRec_t *cbRec, captureCompare_t capture16)
{
    UNUSED(capture16);

    freqInput_t *input = container_of(cbRec, freqInput_t, edgeCb);

    if (input->enabled) {
        const uint32_t capture = *timerChCCR(input->timerHardware);

        if (input->capture) {
            const uint32_t period = capture - input->capture;

            // Calculate frequency
            const float freq = input->clock / period;

            // Update freq filter
            if (freq > FREQ_RANGE_MIN && freq < FREQ_RANGE_MAX) {
                input->freq += (freq - input->freq) * FREQ_FILTER_COEF;
            }

            const uint8_t index = input - freqInputs;
            DEBUG_AXIS(FREQ_SENSOR, index, 0, input->freq * 1000);
            DEBUG_AXIS(FREQ_SENSOR, index, 1, freq * 1000);
            DEBUG_AXIS(FREQ_SENSOR, index, 2, period);
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
        const timerHardware_t *timer = timerAllocate(freqConfig->ioTag[port], OWNER_FREQ, RESOURCE_INDEX(port));
        if (timer) {
            freqInput_t *input = &freqInputs[port];
            uint32_t clock = timerClock(timer->tim);
            uint32_t presc, count;

            input->timerHardware = timer;
            input->enabled = true;
            input->timer32 = (timer->tim == TIM2 || timer->tim == TIM5);
            input->timeout = FREQ_TIMEOUT(clock);
            input->clock = clock;
            input->freq = 0;
            input->pcoef = 0;
            input->period = 0;
            input->capture = 0;
            input->overflows = 0;

            presc = FREQ_PRESCALER_MIN;
            count = clock / FREQ_RANGE_MAX;
            while (count > presc * 32768 && presc < FREQ_PRESCALER_MAX)
                presc = presc << 1;
            input->presc_min = presc;

            presc = FREQ_PRESCALER_MIN;
            count = input->timeout / 65536;
            while (count > presc && presc < FREQ_PRESCALER_MAX)
                presc = presc << 1;
            input->presc_max = presc;

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
            freqSetBaseClock(input, (input->timer32) ? 1 : input->presc_max);
        }
    }
}

void freqUpdate(void)
{
    for (int port = 0; port < FREQ_SENSOR_PORT_COUNT; port++) {
        freqInput_t *input = &freqInputs[port];
        if (input->enabled && input->timer32 && input->capture) {
            uint32_t counter = input->timerHardware->tim->CNT;
            uint32_t delta = counter - input->capture;
            if (delta > input->timeout) {
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
        return freqInputs[port].freq;
    }
    return 0.0f;
}

uint16_t getFreqSensorRPM(uint8_t port)
{
    if (port < FREQ_SENSOR_PORT_COUNT) {
        // Return eRPM/100 as expected by RPM filter, msp, etc.
        return (uint16_t) (freqInputs[port].freq * 60.0f / 100.0f);
    }
    return 0;
}

bool isFreqSensorPortInitialized(uint8_t port)
{
    if (port < FREQ_SENSOR_PORT_COUNT) {
        return freqInputs[port].enabled;
    }
    return false;
}

// Now, return true if at least one sensor is enabled
bool isFreqSensorInitialized(void)
{
    for (int port = 0; port < FREQ_SENSOR_PORT_COUNT; port++) {
        if (freqInputs[port].enabled) {
            return true;
        }
    }
    return false;
}

#endif
