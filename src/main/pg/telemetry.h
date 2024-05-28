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

#include "common/unit.h"

#include "telemetry/ibus_shared.h"

#include "pg/pg.h"


typedef enum {
    FRSKY_FORMAT_DMS = 0,
    FRSKY_FORMAT_NMEA
} frskyGpsCoordFormat_e;

typedef enum
{
    TELEM_NONE              = 0,

    TELEM_HEADSPEED,
    TELEM_TAILSPEED,

    TELEM_BATTERY_GROUP,
    TELEM_BATTERY_VOLTAGE,
    TELEM_BATTERY_CURRENT,
    TELEM_BATTERY_CONSUMPTION,
    TELEM_BATTERY_CHARGE_LEVEL,

    TELEM_CELLS_GROUP,
    TELEM_CELL1_VOLTAGE,
    TELEM_CELL2_VOLTAGE,
    TELEM_CELL3_VOLTAGE,
    TELEM_CELL4_VOLTAGE,
    TELEM_CELL5_VOLTAGE,
    TELEM_CELL6_VOLTAGE,
    TELEM_CELL7_VOLTAGE,
    TELEM_CELL8_VOLTAGE,
    TELEM_CELL9_VOLTAGE,
    TELEM_CELL10_VOLTAGE,
    TELEM_CELL11_VOLTAGE,
    TELEM_CELL12_VOLTAGE,
    TELEM_CELL13_VOLTAGE,
    TELEM_CELL14_VOLTAGE,
    TELEM_CELL15_VOLTAGE,
    TELEM_CELL16_VOLTAGE,

    TELEM_CELLS_MIN,
    TELEM_CELLS_MAX,

    TELEM_BEC_VOLTAGE,
    TELEM_BEC_CURRENT,

    TELEM_BUS_VOLTAGE,
    TELEM_BUS_CURRENT,

    TELEM_MCU_VOLTAGE,
    TELEM_MCU_CURRENT,

    TELEM_BUFFER_VOLTAGE,
    TELEM_BUFFER_CURRENT,

    TELEM_AIR_TEMPERATURE,
    TELEM_ESC_TEMPERATURE,
    TELEM_BEC_TEMPERATURE,
    TELEM_MCU_TEMPERATURE,
    TELEM_MOTOR_TEMPERATURE,
    TELEM_BATTERY_TEMPERATURE,

    TELEM_ALTITUDE,
    TELEM_VARIOMETER,

    TELEM_ACCEL_GROUP,
    TELEM_ACCEL_X,
    TELEM_ACCEL_Y,
    TELEM_ACCEL_Z,

    TELEM_ATTITUDE_GROUP,
    TELEM_ATTITUDE_PITCH,
    TELEM_ATTITUDE_ROLL,
    TELEM_ATTITUDE_YAW,

    TELEM_GPS_GROUP,
    TELEM_GPS_HEADING,
    TELEM_GPS_LATITUDE,
    TELEM_GPS_LONGITUDE,
    TELEM_GPS_ALTITUDE,
    TELEM_GPS_DISTANCE,
    TELEM_GPS_GROUNDSPEED,
    TELEM_GPS_SAT_COUNT,
    TELEM_GPS_DATE_TIME,

    TELEM_AIRSPEED,
    TELEM_RANGER_ALT,

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
    TELEM_ESC2_STATUS,

    TELEM_FC_CPU_LOAD,
    TELEM_FC_SYS_LOAD,
    TELEM_FC_RT_LOAD,
    TELEM_FC_UPTIME,

    TELEM_FLIGHT_MODE,
    TELEM_ARMING_FLAGS,
    TELEM_GOVERNOR_STATE,

    TELEM_ADJFUNC_GROUP,
    TELEM_ADJFUNC_ID,
    TELEM_ADJFUNC_VALUE,

    TELEM_PID_PROFILE,
    TELEM_RATES_PROFILE,
    TELEM_BATTERY_PROFILE,
    TELEM_LED_PROFILE,

    TELEM_ADC1,
    TELEM_ADC2,
    TELEM_ADC3,
    TELEM_ADC4,

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
    SENSOR_TEMPERATURE     = TELEM_ESC_TEMPERATURE,
    SENSOR_CAP_USED        = TELEM_BATTERY_CONSUMPTION,
    SENSOR_ADJUSTMENT      = TELEM_ADJFUNC_GROUP,
    SENSOR_GOV_MODE        = TELEM_GOVERNOR_STATE,

    ESC_SENSOR_CURRENT     = TELEM_ESC1_CURRENT,
    ESC_SENSOR_VOLTAGE     = TELEM_ESC1_VOLTAGE,
    ESC_SENSOR_RPM         = TELEM_ESC1_ERPM,
    ESC_SENSOR_TEMPERATURE = TELEM_ESC1_TEMP1,

} sensor_e;

#define TELEM_SENSOR_COUNT 40

typedef struct telemetryConfig_s {
    int16_t gpsNoFixLatitude;
    int16_t gpsNoFixLongitude;
    uint8_t telemetry_inverted;
    uint8_t halfDuplex;
    uint8_t frsky_coordinate_format;
    uint8_t frsky_unit;
    uint8_t frsky_vfas_precision;
    uint8_t hottAlarmSoundInterval;
    uint8_t pidValuesAsTelemetry;
    uint8_t report_cell_voltage;
    uint16_t mavlink_mah_as_heading_divisor;
    uint8_t flysky_sensors[IBUS_SENSOR_COUNT];
    uint16_t telemetry_sensors[TELEM_SENSOR_COUNT];
} telemetryConfig_t;

PG_DECLARE(telemetryConfig_t, telemetryConfig);

