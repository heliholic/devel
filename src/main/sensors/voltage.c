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

#include "stdbool.h"
#include "stdint.h"
#include "string.h"

#include "platform.h"

#include "build/build_config.h"

#include "common/filter.h"
#include "common/maths.h"

#include "config/config.h"
#include "config/config_reset.h"

#include "drivers/adc.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "sensors/battery.h"
#include "sensors/esc_sensor.h"
#include "sensors/adcinternal.h"

#include "voltage.h"


/** Voltage ADC Sensors **/

#ifndef VOLTAGE_SCALE_DEFAULT
#define VOLTAGE_SCALE_DEFAULT 110
#endif

#ifndef VOLTAGE_DIVIDER_DEFAULT
#define VOLTAGE_DIVIDER_DEFAULT 10
#endif

#ifndef VOLTAGE_MULTIPLIER_DEFAULT
#define VOLTAGE_MULTIPLIER_DEFAULT 1
#endif

PG_REGISTER_ARRAY_WITH_RESET_FN(voltageSensorADCConfig_t, MAX_VOLTAGE_SENSOR_ADC, voltageSensorADCConfig, PG_VOLTAGE_SENSOR_ADC_CONFIG, 0);

void pgResetFn_voltageSensorADCConfig(voltageSensorADCConfig_t *instance)
{
    for (int i = 0; i < MAX_VOLTAGE_SENSOR_ADC; i++) {
        RESET_CONFIG(voltageSensorADCConfig_t, &instance[i],
            .scale = VOLTAGE_SCALE_DEFAULT,
            .resdivval = VOLTAGE_DIVIDER_DEFAULT,
            .resdivmul = VOLTAGE_MULTIPLIER_DEFAULT,
        );
    }
}


// Voltage sensor ID to ADC Channel map
static const uint8_t voltageSensorAdcChannelMap[MAX_VOLTAGE_SENSOR_ADC] = {
    [VOLTAGE_SENSOR_ADC_BAT] = ADC_BATTERY,
    [VOLTAGE_SENSOR_ADC_BEC] = ADC_VBEC,
    [VOLTAGE_SENSOR_ADC_BUS] = ADC_VBUS,
    [VOLTAGE_SENSOR_ADC_EXT] = ADC_VEXT,
};

typedef struct {
    uint32_t filtered;
    uint32_t unfiltered;
    filter_t filter;
} voltageSensorState_t;

voltageSensorState_t voltageADCSensors[MAX_VOLTAGE_SENSOR_ADC];


static uint32_t voltageSensorADCtoVoltage(int sensor, uint16_t adc)
{
    const voltageSensorADCConfig_t *config = voltageSensorADCConfig(sensor);

    const uint32_t divisor = constrain(config->resdivmul, VOLTAGE_MULTIPLIER_MIN, VOLTAGE_MULTIPLIER_MAX) *
                             constrain(config->resdivval, VOLTAGE_DIVIDER_MIN, VOLTAGE_DIVIDER_MAX);
    const uint32_t scale = config->scale * getVrefMv();

    // calculate battery voltage based on ADC reading in 1mV steps
    return (adc * scale) / (0xfff * divisor);
}

void voltageSensorADCRefresh(void)
{
    for (uint8_t i = 0; i < MAX_VOLTAGE_SENSOR_ADC && i < ARRAYLEN(voltageSensorAdcChannelMap); i++) {
        voltageSensorState_t *state = &voltageADCSensors[i];
#ifdef USE_ADC
        uint8_t channel = voltageSensorAdcChannelMap[i];
        uint16_t sample = adcGetChannel(channel);
        uint32_t voltage = voltageSensorADCtoVoltage(i, sample);
        uint32_t filtered = filterApply(&state->filter, voltage);

        state->filtered = filtered;
        state->unfiltered = voltage;
#else
        UNUSED(voltageSensorADCtoVoltage);
        state->filtered = 0;
        state->unfiltered = 0;
#endif
    }
}

void voltageSensorADCRead(voltageSensorADC_e adcChannel, voltageMeter_t *voltageMeter)
{
    voltageSensorState_t *state = &voltageADCSensors[adcChannel];

    voltageMeter->filtered = state->filtered / 10;  // FIXME
    voltageMeter->unfiltered = state->unfiltered / 10;
}

void voltageSensorADCInit(void)
{
    memset(voltageADCSensors, 0, sizeof(voltageADCSensors));

    for (unsigned i = 0; i < MAX_VOLTAGE_SENSOR_ADC && i < ARRAYLEN(voltageSensorAdcChannelMap); i++) {
        lowpassFilterInit(&voltageADCSensors[i].filter, LPF_BESSEL, GET_BATTERY_LPF_FREQUENCY(batteryConfig()->vbatLpfPeriod), batteryConfig()->vbatUpdateHz, 0);
    }
}


/** Voltage ESC Sensors **/

#ifdef USE_ESC_SENSOR
static voltageSensorState_t voltageESCSensor;
#endif

void voltageMeterESCReadMotor(uint8_t motorNumber, voltageMeter_t *voltageMeter)
{
#ifdef USE_ESC_SENSOR
    const escSensorData_t *escData = getEscSensorData(motorNumber);
    if (escData) {
        const uint32_t voltage = (escData->dataAge <= ESC_BATTERY_AGE_MAX) ? escData->voltage : 0;
        voltageMeter->unfiltered = voltage;
        voltageMeter->filtered = voltage;
    } else {
        voltageMeterReset(voltageMeter);
    }
#else
    UNUSED(motorNumber);
    voltageMeterReset(voltageMeter);
#endif
}

void voltageMeterESCReadCombined(voltageMeter_t *voltageMeter)
{
#ifdef USE_ESC_SENSOR
    voltageMeter->filtered = voltageESCSensor.filtered;
    voltageMeter->unfiltered = voltageESCSensor.unfiltered;
#else
    voltageMeterReset(voltageMeter);
#endif
}

void voltageMeterESCRefresh(void)
{
#ifdef USE_ESC_SENSOR
    const escSensorData_t *escData = getEscSensorData(ESC_SENSOR_COMBINED);
    if (escData) {
        const uint32_t voltage = (escData->dataAge <= ESC_BATTERY_AGE_MAX) ? escData->voltage : 0;
        voltageESCSensor.unfiltered = voltage;
        voltageESCSensor.filtered = filterApply(&voltageESCSensor.filter, voltage);
    }
#endif
}

void voltageMeterESCInit(void)
{
#ifdef USE_ESC_SENSOR
    memset(&voltageESCSensor, 0, sizeof(voltageESCSensor));
    lowpassFilterInit(&voltageESCSensor.filter, LPF_BESSEL, GET_BATTERY_LPF_FREQUENCY(batteryConfig()->vbatLpfPeriod), batteryConfig()->vbatUpdateHz, 0);
#endif
}


//
// API for using voltage meters using IDs
//
// This API is used by MSP, for configuration/status.
//

const uint8_t voltageMeterIds[] = {
    VOLTAGE_METER_ID_BATTERY,
    VOLTAGE_METER_ID_BEC,
    VOLTAGE_METER_ID_BUS,
    VOLTAGE_METER_ID_EXT,
#ifdef USE_ESC_SENSOR
    VOLTAGE_METER_ID_ESC_COMBINED,
    VOLTAGE_METER_ID_ESC_1,
    VOLTAGE_METER_ID_ESC_2,
    VOLTAGE_METER_ID_ESC_3,
    VOLTAGE_METER_ID_ESC_4,
#endif
};

const uint8_t voltageMeterCount = ARRAYLEN(voltageMeterIds);

const uint8_t voltageSensorToMeterMap[MAX_VOLTAGE_SENSOR_ADC] = {
    [VOLTAGE_SENSOR_ADC_BAT] = VOLTAGE_METER_ID_BATTERY,
    [VOLTAGE_SENSOR_ADC_BEC] = VOLTAGE_METER_ID_BEC,
    [VOLTAGE_SENSOR_ADC_BUS] = VOLTAGE_METER_ID_BUS,
    [VOLTAGE_SENSOR_ADC_EXT] = VOLTAGE_METER_ID_EXT,
};

void voltageMeterRead(voltageMeterId_e id, voltageMeter_t *meter)
{
    if (id == VOLTAGE_METER_ID_BATTERY) {
        voltageSensorADCRead(VOLTAGE_SENSOR_ADC_BAT, meter);
    } else
    if (id == VOLTAGE_METER_ID_BEC) {
        voltageSensorADCRead(VOLTAGE_SENSOR_ADC_BEC, meter);
    } else
    if (id == VOLTAGE_METER_ID_BUS) {
        voltageSensorADCRead(VOLTAGE_SENSOR_ADC_BUS, meter);
    } else
    if (id == VOLTAGE_METER_ID_EXT) {
        voltageSensorADCRead(VOLTAGE_SENSOR_ADC_EXT, meter);
    } else
#ifdef USE_ESC_SENSOR
    if (id == VOLTAGE_METER_ID_ESC_COMBINED) {
        voltageMeterESCReadCombined(meter);
    } else
    if (id >= VOLTAGE_METER_ID_ESC_1 && id <= VOLTAGE_METER_ID_ESC_4) {
        int motor = id - VOLTAGE_METER_ID_ESC_1;
        voltageMeterESCReadMotor(motor, meter);
    } else
#endif
    {
        voltageMeterReset(meter);
    }
}

void voltageMeterReset(voltageMeter_t *meter)
{
    meter->filtered = 0;
    meter->unfiltered = 0;
}
