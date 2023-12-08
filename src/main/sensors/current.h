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
#include "current_ids.h"

#ifndef CURRENT_TASK_FREQ_HZ
#define CURRENT_TASK_FREQ_HZ 50
#endif

#define CURRENT_METER_ID_ADC_COUNT 1
#define CURRENT_METER_ID_ESC_COUNT 4


typedef struct currentSensorADCConfig_s {
    int16_t scale;              // scale the current sensor output voltage to milliamps. Value in mV/10A
    int16_t offset;             // offset of the current sensor in mA
} currentSensorADCConfig_t;

PG_DECLARE(currentSensorADCConfig_t, currentSensorADCConfig);


typedef enum {
    CURRENT_SENSOR_NONE = 0,
    CURRENT_SENSOR_ADC,
    CURRENT_SENSOR_ESC,
    CURRENT_SENSOR_MSP
} currentSensor_e;

typedef struct {
    int32_t amperage;             // current in 1mA steps
    int32_t amperageLatest;
    int32_t mAhDrawn;             // mAh drawn from the battery since start
} currentMeter_t;


//
// Current Sensor API
//

void currentSensorADCInit(void);
void currentSensorADCRefresh(int32_t lastUpdateAt);
void currentSensorADCRead(currentMeter_t *meter);

void currentSensorESCInit(void);
void currentSensorESCRefresh(int32_t lastUpdateAt);
void currentSensorESCReadCombined(currentMeter_t *meter);
void currentSensorESCReadMotor(uint8_t motorNumber, currentMeter_t *meter);


//
// Current Meter API
//

extern const uint8_t supportedCurrentMeterCount;
extern const uint8_t currentMeterIds[];

void currentMeterRead(currentMeterId_e id, currentMeter_t *currentMeter);
void currentMeterReset(currentMeter_t *meter);

