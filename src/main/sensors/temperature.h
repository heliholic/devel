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

#pragma once

#include "platform.h"

#include "common/time.h"
#include "temperature_ids.h"

typedef enum {
    TEMP_SENSOR_ADC_BAT = 0,
    MAX_TEMP_SENSOR_ADC
} tempSensorADC_e;

typedef struct {
    int16_t scale;               // scale the current sensor output voltage to milliamps. Value in mV/10A
    int16_t offset;              // offset of the current sensor in mA
    uint8_t cutoff;              // filter cutoff frequency in Hz
} tempSensorADCConfig_t;

PG_DECLARE_ARRAY(tempSensorADCConfig_t, MAX_TEMP_SENSOR_ADC, tempSensorADCConfig);


typedef enum {
    TEMP_SENSOR_TYPE_NONE = 0,
    TEMP_SENSOR_TYPE_ADC,
    TEMP_SENSOR_TYPE_ESC,
} tempSensorType_e;

typedef struct {
    int16_t sample;
    int16_t temperature;
} tempMeter_t;


//
// Current Sensor API
//

void tempSensorADCInit(void);
void tempSensorADCRefresh(timeUs_t currentTimeUs);
bool tempSensorADCRead(tempSensorADC_e sensor, tempMeter_t *meter);

void tempSensorESCInit(void);
void tempSensorESCRefresh(void);
bool tempSensorESCReadTotal(tempMeter_t *meter);
bool tempSensorESCReadMotor(uint8_t motorNumber, tempMeter_t *meter);


//
// Current Meter API
//

extern const uint8_t tempSensorToMeterMap[MAX_TEMP_SENSOR_ADC];
extern const uint8_t tempMeterIds[];
extern const uint8_t tempMeterCount;

bool tempMeterRead(tempMeterId_e id, tempMeter_t *tempMeter);
void tempMeterReset(tempMeter_t *meter);

