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
#include "common/utils.h"
#include "common/filter.h"

#include "config/config.h"
#include "config/config_reset.h"

#include "drivers/adc.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "sensors/adcinternal.h"
#include "sensors/battery.h"
#include "sensors/esc_sensor.h"

#include "current.h"


/** Temperature Sensors **/

#ifndef TEMP_METER_SCALE_DEFAULT
#define TEMP_METER_SCALE_DEFAULT 400
#endif

#ifndef TEMP_METER_OFFSET_DEFAULT
#define TEMP_METER_OFFSET_DEFAULT 0
#endif

#ifndef TEMP_METER_CUTOFF_DEFAULT
#define TEMP_METER_CUTOFF_DEFAULT 25
#endif

PG_REGISTER_ARRAY_WITH_RESET_FN(tempSensorADCConfig_t, MAX_TEMP_SENSOR_ADC, tempSensorADCConfig, PG_TEMP_SENSOR_ADC_CONFIG, 0);

void pgResetFn_tempSensorADCConfig(tempSensorADCConfig_t *instance)
{
    for (int i = 0; i < MAX_TEMP_SENSOR_ADC; i++) {
        RESET_CONFIG(tempSensorADCConfig_t, &instance[i],
            .scale = TEMP_METER_SCALE_DEFAULT,
            .offset = TEMP_METER_OFFSET_DEFAULT,
            .cutoff = TEMP_METER_CUTOFF_DEFAULT,
        );
    }
}


/** Internal state **/

typedef struct {
    bool enabled;
    int16_t sample;             // 0.1C
    int16_t temperature;
    filter_t filter;
} tempSensorState_t;


/** Temperature ADC Sensors **/

#ifdef USE_ADC

static tempSensorState_t tempADCSensors[MAX_TEMP_SENSOR_ADC];

// Temperature sensor ID to ADC Channel map
static const uint8_t tempSensorAdcChannelMap[MAX_TEMP_SENSOR_ADC] = {
    [TEMP_SENSOR_ADC_EXT] = ADC_TEMP,
};

static float tempSensorADCToTemperature(int sensor, const uint16_t src)
{
    const tempSensorADCConfig_t *config = tempSensorADCConfig(sensor);

    // y=x/m+b m is scale in (mV/10A) and b is offset in (mA)
    float voltage = src * getVrefMv() / 0.4095f;
    float temperature = config->scale ? (voltage / config->scale + config->offset) : 0;

    return temperature;
}
#endif

void tempSensorADCRefresh(timeUs_t currentTimeUs)
{
#ifdef USE_ADC
    static timeUs_t lastServiced = 0;

    const timeDelta_t udpateDelta = cmp32(currentTimeUs, lastServiced);
    lastServiced = currentTimeUs;

    for (unsigned i = 0; i < MAX_TEMP_SENSOR_ADC; i++) {
        tempSensorState_t * state = &tempADCSensors[i];

        const uint8_t channel = tempSensorAdcChannelMap[i];
        state->enabled = adcIsEnabled(channel);

        if (state->enabled) {
            const uint16_t sample = adcGetChannel(channel);
            const float temp = tempSensorADCToTemperature(i, sample);

            state->sample = temp;
            state->temperature = filterApply(&state->filter, temp);

            DEBUG_AXIS(TEMP_SENSOR, i, 0, sample);
            DEBUG_AXIS(TEMP_SENSOR, i, 1, temperature);
        }
        else {
            state->sample = 0;
            state->temperature = 0;
        }
    }
#else
    UNUSED(currentTimeUs);
#endif
}

bool tempSensorADCRead(tempSensorADC_e sensor, tempMeter_t *meter)
{
#ifdef USE_ADC
    tempSensorState_t * state = &tempADCSensors[sensor];

    if (state->enabled) {
        meter->sample = state->sample;
        meter->temperature = state->temperature;
    }

    return state->enabled;
#else
    UNUSED(sensor);
    tempMeterReset(meter);
    return false;
#endif
}

void tempSensorADCInit(void)
{
#ifdef USE_ADC
    memset(&tempADCSensors, 0, sizeof(tempADCSensors));

    for (unsigned i = 0; i < MAX_TEMP_SENSOR_ADC; i++) {
        lowpassFilterInit(&tempADCSensors[i].filter, LPF_BESSEL,
            tempSensorADCConfig(i)->cutoff,
            batteryConfig()->ibatUpdateHz, 0);
    }
#endif
}


/** Temperature ESC Sensors **/

#ifdef USE_ESC_SENSOR

tempSensorState_t tempESCSensor;

bool tempSensorESCReadMotor(uint8_t motorNumber, tempMeter_t *meter)
{
    escSensorData_t *escData = getEscSensorData(motorNumber);

    if (escData && escData->age <= ESC_BATTERY_AGE_MAX) {
        meter->temperature = meter->sample = escData->temperature;
        return true;
    } else {
        tempMeterReset(meter);
    }

    return false;
}

bool tempSensorESCReadTotal(tempMeter_t *meter)
{
    tempSensorState_t * state = &currentESCSensor;

    meter->sample = state->sample;
    meter->temperature = state->temperature;

    return state->enabled;
}

void tempSensorESCRefresh(void)
{
    escSensorData_t *escData = getEscSensorData(ESC_SENSOR_COMBINED);
    tempSensorState_t * state = &currentESCSensor;

    if (escData && escData->age <= ESC_BATTERY_AGE_MAX) {
        const uint32_t temp = escData->temperature + escSensorConfig()->temperature_offset;
        state->sample = temp;
        state->temperature = filterApply(&state->filter, temp);
        state->enabled = true;
    }
    else {
        state->sample = 0;
        state->temperature = 0;
    }
}

void tempSensorESCInit(void)
{
    memset(&currentESCSensor, 0, sizeof(currentESCSensor));

    lowpassFilterInit(&currentESCSensor.filter, LPF_BESSEL,
        escSensorConfig()->filter_cutoff,
        batteryConfig()->ibatUpdateHz, 0);
}

#endif


//
// API for temperature meters using IDs
//
// This API is used by MSP, for configuration/status.
//

const uint8_t tempMeterIds[] = {
    TEMP_METER_ID_EXT,
#ifdef USE_ESC_SENSOR
    TEMP_METER_ID_ESC_COMBINED,
    TEMP_METER_ID_ESC_1,
    TEMP_METER_ID_ESC_2,
    TEMP_METER_ID_ESC_3,
    TEMP_METER_ID_ESC_4,
#endif
};

const uint8_t tempMeterCount = ARRAYLEN(tempMeterIds);

const uint8_t tempSensorToMeterMap[MAX_TEMP_SENSOR_ADC] = {
    [TEMP_SENSOR_ADC_BAT] = TEMP_METER_ID_BATTERY,
};

bool tempMeterRead(tempMeterId_e id, tempMeter_t *meter)
{
    switch (id) {
        case TEMP_METER_ID_EXT:
            return tempSensorADCRead(TEMP_SENSOR_ADC_EXT, meter);
#ifdef USE_ESC_SENSOR
        case TEMP_METER_ID_ESC_COMBINED:
            return tempSensorESCReadTotal(meter);
        case TEMP_METER_ID_ESC_1:
        case TEMP_METER_ID_ESC_2:
        case TEMP_METER_ID_ESC_3:
        case TEMP_METER_ID_ESC_4:
            return tempSensorESCReadMotor(id - TEMP_METER_ID_ESC_1, meter);
#endif
        default:
            tempMeterReset(meter);
    }

    return false;
}

void tempMeterReset(tempMeter_t *meter)
{
    memset(meter, 0, sizeof(tempMeter_t));
}

