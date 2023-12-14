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

#pragma once

#include "platform.h"

#include "voltage_ids.h"

#ifndef VOLTAGE_TASK_FREQ_HZ
#define VOLTAGE_TASK_FREQ_HZ 50
#endif

#define VOLTAGE_SCALE_MIN 0
#define VOLTAGE_SCALE_MAX 255

#define VOLTAGE_DIVIDER_MIN 1
#define VOLTAGE_DIVIDER_MAX 255

#define VOLTAGE_MULTIPLIER_MIN 1
#define VOLTAGE_MULTIPLIER_MAX 255

typedef enum {
    VOLTAGE_SENSOR_ADC_BAT = 0,
    VOLTAGE_SENSOR_ADC_BEC = 1,
    VOLTAGE_SENSOR_ADC_BUS = 2,
    VOLTAGE_SENSOR_ADC_EXT = 3,
    MAX_VOLTAGE_SENSOR_ADC
} voltageSensorADC_e;

typedef struct {
    uint8_t scale;                      // adjust this to match battery voltage to reported value
    uint8_t resdivval;                  // resistor divider R2 (default NAZE 10(K))
    uint8_t resdivmul;                  // multiplier for scale (e.g. 2.5:1 ratio with multiplier of 4 can use '100' instead of '25' in ratio) to get better precision
} voltageSensorADCConfig_t;

PG_DECLARE_ARRAY(voltageSensorADCConfig_t, MAX_VOLTAGE_SENSOR_ADC, voltageSensorADCConfig);


typedef enum {
    VOLTAGE_SENSOR_TYPE_NONE = 0,
    VOLTAGE_SENSOR_TYPE_ADC,
    VOLTAGE_SENSOR_TYPE_ESC,
} voltageSensorType_e;

typedef struct voltageMeter_s {
    uint32_t sample;
    uint32_t voltage;
} voltageMeter_t;


//
// Voltage Sensor API
//

void voltageSensorADCInit(void);
void voltageSensorADCRefresh(void);
bool voltageSensorADCRead(voltageSensorADC_e sensor, voltageMeter_t *voltageMeter);

void voltageSensorESCInit(void);
void voltageSensorESCRefresh(void);
bool voltageSensorESCReadTotal(voltageMeter_t *voltageMeter);
bool voltageSensorESCReadMotor(uint8_t motor, voltageMeter_t *voltageMeter);


//
// Voltage Meter API
//

extern const uint8_t voltageSensorToMeterMap[MAX_VOLTAGE_SENSOR_ADC];
extern const uint8_t voltageMeterIds[];
extern const uint8_t voltageMeterCount;

bool voltageMeterRead(voltageMeterId_e id, voltageMeter_t *voltageMeter);
void voltageMeterReset(voltageMeter_t *voltageMeter);
