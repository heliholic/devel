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

#include "pg/pg.h"
#include "pg/telemetry.h"

typedef enum
{
    TELEM_NONE = 0,

    TELEM_MODEL_ID,

    TELEM_BATTERY,
    TELEM_BATTERY_VOLTAGE,
    TELEM_BATTERY_CURRENT,
    TELEM_BATTERY_CONSUMPTION,
    TELEM_BATTERY_CHARGE_LEVEL,
    TELEM_BATTERY_TEMPERATURE,
    TELEM_BATTERY_CELLS,

    TELEM_ESC1_GROUP,
    TELEM_ESC1_VOLTAGE,
    TELEM_ESC1_CURRENT,
    TELEM_ESC1_ERPM,
    TELEM_ESC1_POWER,
    TELEM_ESC1_THROTTLE,
    TELEM_ESC1_TEMP1,
    TELEM_ESC1_TEMP2,
    TELEM_ESC1_BEC_VOLTAGE,
    TELEM_ESC1_BEC_CURRENT,
    TELEM_ESC1_ERRORS,
    TELEM_ESC1_STATUS,

    TELEM_ESC2_GROUP,
    TELEM_ESC2_VOLTAGE,
    TELEM_ESC2_CURRENT,
    TELEM_ESC2_ERPM,
    TELEM_ESC2_POWER,
    TELEM_ESC2_THROTTLE,
    TELEM_ESC2_TEMP1,
    TELEM_ESC2_TEMP2,
    TELEM_ESC2_BEC_VOLTAGE,
    TELEM_ESC2_BEC_CURRENT,
    TELEM_ESC2_ERRORS,
    TELEM_ESC2_STATUS,

    TELEM_BEC_VOLTAGE,
    TELEM_BUS_VOLTAGE,
    TELEM_MCU_VOLTAGE,

    TELEM_BEC_CURRENT,
    TELEM_BUS_CURRENT,
    TELEM_MCU_CURRENT,

    TELEM_BEC_TEMP,
    TELEM_ESC_TEMP,
    TELEM_MCU_TEMP,
    TELEM_MOTOR_TEMP,

    TELEM_EXT1_TEMP,
    TELEM_EXT2_TEMP,
    TELEM_EXT3_TEMP,
    TELEM_EXT4_TEMP,

    TELEM_ALTITUDE,
    TELEM_VARIOMETER,

    TELEM_HEADSPEED,
    TELEM_TAILSPEED,

    TELEM_ATTITUDE_PITCH,
    TELEM_ATTITUDE_ROLL,
    TELEM_ATTITUDE_YAW,

    TELEM_ACCEL_X,
    TELEM_ACCEL_Y,
    TELEM_ACCEL_Z,

    TELEM_GPS_GROUP,
    TELEM_GPS_HEADING,
    TELEM_GPS_LATITUDE,
    TELEM_GPS_LONGITUDE,
    TELEM_GPS_ALTITUDE,
    TELEM_GPS_DISTANCE,
    TELEM_GPS_GROUNDSPEED,
    TELEM_GPS_SAT_COUNT,
    TELEM_GPS_DATE_TIME,

    TELEM_FC_CPU_GROUP,
    TELEM_FC_CPU_LOAD,
    TELEM_FC_SYS_LOAD,
    TELEM_FC_RT_LOAD,
    TELEM_FC_UPTIME,

    TELEM_FLIGHT_MODE,
    TELEM_ARMING_FLAGS,
    TELEM_GOVERNOR_STATE,

    TELEM_PROFILE_GORUP,
    TELEM_PID_PROFILE,
    TELEM_RATES_PROFILE,
    TELEM_BATTERY_PROFILE,
    TELEM_LED_PROFILE,

    TELEM_ADJFUNC,

    TELEM_SENSOR_COUNT,

    /* Compatibility values */
    SENSOR_VOLTAGE         = TELEM_BATTERY_VOLTAGE,
    SENSOR_CURRENT         = TELEM_BATTERY_CURRENT,
    SENSOR_FUEL            = TELEM_BATTERY_CHARGE_LEVEL,
    SENSOR_MODE            = TELEM_FLIGHT_MODE,
    SENSOR_ACC_X           = TELEM_ACCEL_X,
    SENSOR_ACC_Y           = TELEM_ACCEL_Y,
    SENSOR_ACC_Z           = TELEM_ACCEL_Z,
    SENSOR_PITCH           = TELEM_ATTITUDE_PITCH,
    SENSOR_ROLL            = TELEM_ATTITUDE_ROLL,
    SENSOR_HEADING         = TELEM_ATTITUDE_YAW,
    SENSOR_ALTITUDE        = TELEM_ALTITUDE,
    SENSOR_VARIO           = TELEM_VARIOMETER,
    SENSOR_LAT_LONG        = TELEM_GPS_LATITUDE,
    SENSOR_GROUND_SPEED    = TELEM_GPS_GROUNDSPEED,
    SENSOR_DISTANCE        = TELEM_GPS_DISTANCE,
    SENSOR_TEMPERATURE     = TELEM_ESC_TEMP,
    SENSOR_CAP_USED        = TELEM_BATTERY_CONSUMPTION,
    SENSOR_ADJUSTMENT      = TELEM_ADJFUNC,
    SENSOR_GOV_MODE        = TELEM_GOVERNOR_STATE,

    ESC_SENSOR_CURRENT     = TELEM_ESC1_CURRENT,
    ESC_SENSOR_VOLTAGE     = TELEM_ESC1_VOLTAGE,
    ESC_SENSOR_RPM         = TELEM_ESC1_ERPM,
    ESC_SENSOR_TEMPERATURE = TELEM_ESC1_TEMP1,

} sensor_e;


typedef int (*tlmValue_f)(void);

typedef struct {
    sensor_e            sensor_index;
    uint16_t            sensor_code;
    const char *        sensor_name;

    int                 min_delay;
    int                 max_delay;

    uint8_t             length;

    tlmValue_f          value;

} telemetrySensor_t;


const telemetrySensor_t * telemetryGetSensor(sensor_e sensor_id);
const telemetrySensor_t * telemetryGetSensorCode(uint16_t sensor_code);

inline int telemetryGetSensorValue(sensor_e sensor_id)
{
    return telemetryGetSensor(sensor_id)->value();
}

bool telemetryIsSensorEnabled(sensor_e sensor_id);
