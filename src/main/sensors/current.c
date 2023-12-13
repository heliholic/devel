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
#include "build/debug.h"

#include "common/maths.h"
#include "common/utils.h"
#include "common/filter.h"

#include "drivers/adc.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "sensors/adcinternal.h"
#include "sensors/battery.h"
#include "sensors/esc_sensor.h"

#include "current.h"


/** Current Sensors **/

#ifndef CURRENT_METER_SCALE_DEFAULT
#define CURRENT_METER_SCALE_DEFAULT 400
#endif

#ifndef CURRENT_METER_OFFSET_DEFAULT
#define CURRENT_METER_OFFSET_DEFAULT 0
#endif

PG_REGISTER_WITH_RESET_TEMPLATE(currentSensorADCConfig_t, currentSensorADCConfig, PG_CURRENT_SENSOR_ADC_CONFIG, 0);

PG_RESET_TEMPLATE(currentSensorADCConfig_t, currentSensorADCConfig,
    .scale = CURRENT_METER_SCALE_DEFAULT,
    .offset = CURRENT_METER_OFFSET_DEFAULT,
);


typedef struct {
    float latest;
    float current;
    float capacity;
    filter_t filter;
} currentSensorState_t;


/** Current ADC Sensors **/

currentSensorState_t currentSensorADCState;

static float currentSensorADCToCurrent(const uint16_t src)
{
    const currentSensorADCConfig_t *config = currentSensorADCConfig();

    // y=x/m+b m is scale in (mV/10A) and b is offset in (mA)
    float voltage = 10.0f * src * getVrefMv() / 4095.0f;
    float current = config->scale ? (voltage / config->scale) : 0;

    current += config->offset / 100.0f;

    DEBUG_SET(DEBUG_CURRENT_SENSOR, 0, voltage * 100);
    DEBUG_SET(DEBUG_CURRENT_SENSOR, 1, current * 100);

    return current;
}

void currentSensorADCRefresh(int32_t lastUpdateAt)
{
#ifdef USE_ADC
    currentSensorState_t * state = &currentSensorADCState;

    const uint16_t adc = adcGetChannel(ADC_CURRENT);
    const float current = currentSensorADCToCurrent(adc);

    state->latest = current;
    state->current = filterApply(&state->filter, current);
    state->capacity += current * lastUpdateAt / 3600e3f;
#else
    UNUSED(lastUpdateAt);
    state->latest = 0;
    state->current = 0;
    state->capacity = 0;
#endif
}

void currentSensorADCRead(currentMeter_t *meter)
{
    currentSensorState_t * state = &currentSensorADCState;

    meter->latestf = state->latest;
    meter->currentf = state->current;
    meter->capacityf = state->capacity;

    meter->latest = state->latest * 100;
    meter->current = state->current * 100;
    meter->capacity = state->capacity * 1000;

    DEBUG_SET(DEBUG_CURRENT_SENSOR, 2, meter->latest);
    DEBUG_SET(DEBUG_CURRENT_SENSOR, 3, meter->capacity);
}

void currentSensorADCInit(void)
{
    memset(&currentSensorADCState, 0, sizeof(currentSensorADCState));

    lowpassFilterInit(&currentSensorADCState.filter, LPF_BESSEL,
        GET_BATTERY_LPF_FREQUENCY(batteryConfig()->ibatLpfPeriod), batteryConfig()->ibatUpdateHz, 0);
}


/** Current ESC Sensors **/

#ifdef USE_ESC_SENSOR

currentSensorState_t currentSensorESCState;

void currentSensorESCRefresh(int32_t lastUpdateAt)
{
    UNUSED(lastUpdateAt);

    currentSensorState_t * state = &currentSensorESCState;
    escSensorData_t *escData = getEscSensorData(ESC_SENSOR_COMBINED);

    if (escData && escData->dataAge <= ESC_BATTERY_AGE_MAX) {
        const uint32_t current = escData->current + escSensorConfig()->current_offset;
        state->current = state->latest = current / 100.0f;
        state->capacity = (escData->consumption / 100.0f) + (escSensorConfig()->current_offset * millis() / 3600.0f);
    } else {
        state->latest = 0;
        state->current = 0;
    }
}

void currentSensorESCReadTotal(currentMeter_t *meter)
{
    currentSensorState_t * state = &currentSensorESCState;

    meter->latestf = state->latest;
    meter->currentf = state->current;
    meter->capacityf = state->capacity;

    meter->latest = meter->latestf * 100;
    meter->current = meter->currentf * 100;
    meter->capacity = state->capacity * 1000;
}

void currentSensorESCReadMotor(uint8_t motorNumber, currentMeter_t *meter)
{
    escSensorData_t *escData = getEscSensorData(motorNumber);

    if (escData && escData->dataAge <= ESC_BATTERY_AGE_MAX) {
        meter->current = meter->latest = escData->current;
        meter->currentf = meter->latestf = meter->latest / 100.0f;
        meter->capacity = escData->consumption;
        meter->capacityf = meter->capacity / 1000.0f;
    } else {
        currentMeterReset(meter);
    }
}

void currentSensorESCInit(void)
{
    memset(&currentSensorESCState, 0, sizeof(currentSensorESCState));
}

#endif


//
// API for current meters using IDs
//
// This API is used by MSP, for configuration/status.
//

const uint8_t currentMeterIds[] = {
    CURRENT_METER_ID_BATTERY,
#ifdef USE_ESC_SENSOR
    CURRENT_METER_ID_ESC_COMBINED,
    CURRENT_METER_ID_ESC_1,
    CURRENT_METER_ID_ESC_2,
    CURRENT_METER_ID_ESC_3,
    CURRENT_METER_ID_ESC_4,
#endif
};

const uint8_t supportedCurrentMeterCount = ARRAYLEN(currentMeterIds);

void currentMeterRead(currentMeterId_e id, currentMeter_t *meter)
{
    if (id == CURRENT_METER_ID_BATTERY) {
        currentSensorADCRead(meter);
    }
#ifdef USE_ESC_SENSOR
    else if (id == CURRENT_METER_ID_ESC_COMBINED) {
        currentSensorESCReadTotal(meter);
    }
    else if (id >= CURRENT_METER_ID_ESC_1 && id <= CURRENT_METER_ID_ESC_4 ) {
        int motor = id - CURRENT_METER_ID_ESC_1;
        currentSensorESCReadMotor(motor, meter);
    }
#endif
    else {
        currentMeterReset(meter);
    }
}

void currentMeterReset(currentMeter_t *meter)
{
    memset(meter, 0, sizeof(currentMeter_t));
}

