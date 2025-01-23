/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include "platform.h"

#include "build/build_config.h"

#include "common/filter.h"
#include "common/maths.h"
#include "common/utils.h"

#include "config/config.h"
#include "config/config_reset.h"

#include "drivers/adc.h"

#include "flight/mixer.h"
#include "flight/pid.h"

#include "sensors/adcinternal.h"
#include "sensors/battery.h"
#include "sensors/esc_sensor.h"

#include "voltage.h"

const char * const voltageMeterSourceNames[VOLTAGE_METER_COUNT] = {
    "NONE", "ADC", "ESC"
};

const uint8_t voltageMeterIds[] = {
    VOLTAGE_METER_ID_BATTERY_1,
#ifdef ADC_POWER_12V
    VOLTAGE_METER_ID_12V_1,
#endif
#ifdef ADC_POWER_9V
    VOLTAGE_METER_ID_9V_1,
#endif
#ifdef ADC_POWER_5V
    VOLTAGE_METER_ID_5V_1,
#endif
#ifdef USE_ESC_SENSOR
    VOLTAGE_METER_ID_ESC_COMBINED_1,
    VOLTAGE_METER_ID_ESC_MOTOR_1,
    VOLTAGE_METER_ID_ESC_MOTOR_2,
    VOLTAGE_METER_ID_ESC_MOTOR_3,
    VOLTAGE_METER_ID_ESC_MOTOR_4,
    VOLTAGE_METER_ID_ESC_MOTOR_5,
    VOLTAGE_METER_ID_ESC_MOTOR_6,
    VOLTAGE_METER_ID_ESC_MOTOR_7,
    VOLTAGE_METER_ID_ESC_MOTOR_8,
    VOLTAGE_METER_ID_ESC_MOTOR_9,
    VOLTAGE_METER_ID_ESC_MOTOR_10,
    VOLTAGE_METER_ID_ESC_MOTOR_11,
    VOLTAGE_METER_ID_ESC_MOTOR_12,
#endif
};

const uint8_t supportedVoltageMeterCount = ARRAYLEN(voltageMeterIds);

//
// ADC/ESC shared
//

void voltageMeterReset(voltageMeter_t *meter)
{
    meter->displayFiltered = 0;
    meter->voltageStablePrevFiltered = 0;
    meter->voltageStableLastUpdate = 0;
    meter->voltageStableBits = 0;
    meter->unfiltered = 0;
}
//
// ADC
//


typedef struct voltageMeterADCState_s {
    uint16_t voltageDisplayFiltered;         // battery voltage in 0.01V steps (filtered)
    uint16_t voltageUnfiltered;       // battery voltage in 0.01V steps (unfiltered)
    pt1Filter_t displayFilter;
} voltageMeterADCState_t;

voltageMeterADCState_t voltageMeterADCStates[MAX_VOLTAGE_SENSOR_ADC];

voltageMeterADCState_t *getVoltageMeterADC(uint8_t index)
{
    return &voltageMeterADCStates[index];
}

static const uint8_t voltageMeterAdcChannelMap[] = {
    ADC_BATTERY,
#ifdef ADC_POWER_12V
    ADC_POWER_12V,
#endif
#ifdef ADC_POWER_9V
    ADC_POWER_9V,
#endif
#ifdef ADC_POWER_5V
    ADC_POWER_5V,
#endif
};

STATIC_UNIT_TESTED uint16_t voltageAdcToVoltage(const uint16_t src, const voltageSensorADCConfig_t *config)
{
    // calculate battery voltage based on ADC reading
    // result is Vbatt in 0.01V steps. 3.3V = ADC Vref, 0xFFF = 12bit adc, 110 = 10:1 voltage divider (10k:1k) * 100 for 0.01V
    return ((((uint32_t)src * config->vbatscale * getVrefMv() / 10 + (0xFFF * 5)) / (0xFFF * config->vbatresdivval)) / config->vbatresdivmultiplier);
}

void voltageMeterADCRefresh(void)
{
    for (uint8_t i = 0; i < MAX_VOLTAGE_SENSOR_ADC && i < ARRAYLEN(voltageMeterAdcChannelMap); i++) {
        voltageMeterADCState_t *state = &voltageMeterADCStates[i];
#ifdef USE_ADC
        // store the battery voltage with some other recent battery voltage readings

        const voltageSensorADCConfig_t *config = voltageSensorADCConfig(i);

        uint8_t channel = voltageMeterAdcChannelMap[i];
        uint16_t rawSample = adcGetChannel(channel);
        uint16_t filteredDisplaySample = pt1FilterApply(&state->displayFilter, rawSample);

        // always calculate the latest voltage, see getLatestVoltage() which does the calculation on demand.
        state->voltageDisplayFiltered = voltageAdcToVoltage(filteredDisplaySample, config);
        state->voltageUnfiltered = voltageAdcToVoltage(rawSample, config);
#else
        UNUSED(voltageAdcToVoltage);

        state->voltageDisplayFiltered = 0;
        state->voltageUnfiltered = 0;
#endif
    }
}

void voltageMeterADCRead(voltageSensorADC_e adcChannel, voltageMeter_t *voltageMeter)
{
    voltageMeterADCState_t *state = &voltageMeterADCStates[adcChannel];

    voltageMeter->displayFiltered = state->voltageDisplayFiltered;
    voltageMeter->unfiltered = state->voltageUnfiltered;
}

void voltageMeterADCInit(void)
{
    for (uint8_t i = 0; i < MAX_VOLTAGE_SENSOR_ADC && i < ARRAYLEN(voltageMeterAdcChannelMap); i++) {
        // store the battery voltage with some other recent battery voltage readings

        voltageMeterADCState_t *state = &voltageMeterADCStates[i];
        memset(state, 0, sizeof(voltageMeterADCState_t));

        pt1FilterInit(&state->displayFilter, pt1FilterGain(GET_BATTERY_LPF_FREQUENCY(batteryConfig()->vbatDisplayLpfPeriod), HZ_TO_INTERVAL(VOLTAGE_TASK_FREQ_HZ)));
    }
}

void voltageMeterGenericInit(void)
{
}

//
// ESC
//

#ifdef USE_ESC_SENSOR
typedef struct voltageMeterESCState_s {
    uint16_t voltageDisplayFiltered;         // battery voltage in 0.01V steps (filtered)
    uint16_t voltageUnfiltered;       // battery voltage in 0.01V steps (unfiltered)
    pt1Filter_t displayFilter;
} voltageMeterESCState_t;

static voltageMeterESCState_t voltageMeterESCState;
#endif

void voltageMeterESCInit(void)
{
#ifdef USE_ESC_SENSOR
    memset(&voltageMeterESCState, 0, sizeof(voltageMeterESCState_t));
    pt1FilterInit(&voltageMeterESCState.displayFilter, pt1FilterGain(GET_BATTERY_LPF_FREQUENCY(batteryConfig()->vbatDisplayLpfPeriod), HZ_TO_INTERVAL(VOLTAGE_TASK_FREQ_HZ)));
#endif
}

void voltageMeterESCRefresh(void)
{
#ifdef USE_ESC_SENSOR
    escSensorData_t *escData = getEscSensorData(ESC_SENSOR_COMBINED);
    if (escData) {
        voltageMeterESCState.voltageUnfiltered = escData->dataAge <= ESC_BATTERY_AGE_MAX ? escData->voltage : 0;
        voltageMeterESCState.voltageDisplayFiltered = pt1FilterApply(&voltageMeterESCState.displayFilter, voltageMeterESCState.voltageUnfiltered);
    }
#endif
}

void voltageMeterESCReadMotor(uint8_t motorNumber, voltageMeter_t *voltageMeter)
{
#ifndef USE_ESC_SENSOR
    UNUSED(motorNumber);
    voltageMeterReset(voltageMeter);
#else
    escSensorData_t *escData = getEscSensorData(motorNumber);
    if (escData) {
        voltageMeter->unfiltered = escData->dataAge <= ESC_BATTERY_AGE_MAX ? escData->voltage : 0;
        voltageMeter->displayFiltered = voltageMeter->unfiltered; // no filtering for ESC motors currently.
    } else {
        voltageMeterReset(voltageMeter);
    }

#endif
}

void voltageMeterESCReadCombined(voltageMeter_t *voltageMeter)
{
#ifndef USE_ESC_SENSOR
    voltageMeterReset(voltageMeter);
#else
    voltageMeter->displayFiltered = voltageMeterESCState.voltageDisplayFiltered;
    voltageMeter->unfiltered = voltageMeterESCState.voltageUnfiltered;
#endif
}

//
// API for using voltage meters using IDs
//
// This API is used by MSP, for configuration/status.
//

// the order of these much match the indexes in voltageSensorADC_e
const uint8_t voltageMeterADCtoIDMap[MAX_VOLTAGE_SENSOR_ADC] = {
    VOLTAGE_METER_ID_BATTERY_1,
#ifdef ADC_POWER_12V
    VOLTAGE_METER_ID_12V_1,
#endif
#ifdef ADC_POWER_9V
    VOLTAGE_METER_ID_9V_1,
#endif
#ifdef ADC_POWER_5V
    VOLTAGE_METER_ID_5V_1,
#endif
};

void voltageMeterRead(voltageMeterId_e id, voltageMeter_t *meter)
{
    if (id == VOLTAGE_METER_ID_BATTERY_1) {
        voltageMeterADCRead(VOLTAGE_SENSOR_ADC_VBAT, meter);
    } else
#ifdef ADC_POWER_12V
    if (id == VOLTAGE_METER_ID_12V_1) {
        voltageMeterADCRead(VOLTAGE_SENSOR_ADC_12V, meter);
    } else
#endif
#ifdef ADC_POWER_9V
    if (id == VOLTAGE_METER_ID_9V_1) {
        voltageMeterADCRead(VOLTAGE_SENSOR_ADC_9V, meter);
    } else
#endif
#ifdef ADC_POWER_5V
    if (id == VOLTAGE_METER_ID_5V_1) {
        voltageMeterADCRead(VOLTAGE_SENSOR_ADC_5V, meter);
    } else
#endif
#ifdef USE_ESC_SENSOR
    if (id == VOLTAGE_METER_ID_ESC_COMBINED_1) {
        voltageMeterESCReadCombined(meter);
    } else
    if (id >= VOLTAGE_METER_ID_ESC_MOTOR_1 && id <= VOLTAGE_METER_ID_ESC_MOTOR_20 ) {
        int motor = id - VOLTAGE_METER_ID_ESC_MOTOR_1;
        voltageMeterESCReadMotor(motor, meter);
    } else
#endif
    {
        voltageMeterReset(meter);
    }
}

// update voltageStableBits
// new 1 bit (= stable) is shifted in every VOLTAGE_STABLE_UPDATE_MS
// when difference is larger than VOLTAGE_STABLE_MAX_DELTA, LSB of shift register is
//   reset to 0 (logical AND) and voltageStablePrevFiltered is updated with new voltage value
void voltageStableUpdate(voltageMeter_t* vm)
{
    const uint32_t now = millis();
    // test voltage on each call
    if (abs(vm->voltageStablePrevFiltered - vm->displayFiltered) > VOLTAGE_STABLE_MAX_DELTA) {
        // reset stable voltage reference
        vm->voltageStablePrevFiltered = vm->displayFiltered;
        vm->voltageStableBits &= ~BIT(0);  // voltage threshold exceeded in this period, clear LSB/BIT(0)
    }
    if (cmp32(now, vm->voltageStableLastUpdate) >= VOLTAGE_STABLE_TICK_MS) {
        vm->voltageStableBits = (vm->voltageStableBits << 1) | BIT(0);  // start with 'stable' state
        vm->voltageStableLastUpdate = now;
    }
}

// voltage is stable when it was within VOLTAGE_STABLE_MAX_DELTA for 10 update periods
bool voltageIsStable(voltageMeter_t* vm)
{
    return popcount(vm->voltageStableBits & (BIT(VOLTAGE_STABLE_BITS_TOTAL + 1) - 1)) >= VOLTAGE_STABLE_BITS_THRESHOLD;
}
