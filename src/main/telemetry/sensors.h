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
    TELEM_NONE                              = 0,

    TELEM_MODEL_ID                          = 0x0001,

    TELEM_BATTERY                           = 0x0010,
    TELEM_BATTERY_VOLTAGE                   = 0x0011,
    TELEM_BATTERY_CURRENT                   = 0x0012,
    TELEM_BATTERY_CONSUMPTION               = 0x0013,
    TELEM_BATTERY_CHARGE_LEVEL              = 0x0014,
    TELEM_BATTERY_TEMPERATURE               = 0x0015,

    TELEM_CELL1_VOLTAGE                     = 0x0020,
    TELEM_CELL2_VOLTAGE                     = 0x0021,
    TELEM_CELL3_VOLTAGE                     = 0x0022,
    TELEM_CELL4_VOLTAGE                     = 0x0023,
    TELEM_CELL5_VOLTAGE                     = 0x0024,
    TELEM_CELL6_VOLTAGE                     = 0x0025,
    TELEM_CELL7_VOLTAGE                     = 0x0026,
    TELEM_CELL8_VOLTAGE                     = 0x0027,
    TELEM_CELL9_VOLTAGE                     = 0x0028,
    TELEM_CELL10_VOLTAGE                    = 0x0029,
    TELEM_CELL11_VOLTAGE                    = 0x002A,
    TELEM_CELL12_VOLTAGE                    = 0x002B,
    TELEM_CELL13_VOLTAGE                    = 0x002C,
    TELEM_CELL14_VOLTAGE                    = 0x002D,
    TELEM_CELL15_VOLTAGE                    = 0x002E,
    TELEM_CELL16_VOLTAGE                    = 0x002F,

    TELEM_ESC1_GROUP                        = 0x0040,
    TELEM_ESC1_VOLTAGE                      = 0x0041,
    TELEM_ESC1_CURRENT                      = 0x0042,
    TELEM_ESC1_ERPM                         = 0x0043,
    TELEM_ESC1_POWER                        = 0x0044,
    TELEM_ESC1_THROTTLE                     = 0x0045,
    TELEM_ESC1_TEMP1                        = 0x0046,
    TELEM_ESC1_TEMP2                        = 0x0047,
    TELEM_ESC1_BEC_VOLTAGE                  = 0x0048,
    TELEM_ESC1_BEC_CURRENT                  = 0x0049,
    TELEM_ESC1_ERRORS                       = 0x004E,
    TELEM_ESC1_STATUS                       = 0x004F,

    TELEM_ESC2_GROUP                        = 0x0050,
    TELEM_ESC2_VOLTAGE                      = 0x0051,
    TELEM_ESC2_CURRENT                      = 0x0052,
    TELEM_ESC2_ERPM                         = 0x0053,
    TELEM_ESC2_POWER                        = 0x0054,
    TELEM_ESC2_THROTTLE                     = 0x0055,
    TELEM_ESC2_TEMP1                        = 0x0056,
    TELEM_ESC2_TEMP2                        = 0x0057,
    TELEM_ESC2_BEC_VOLTAGE                  = 0x0058,
    TELEM_ESC2_BEC_CURRENT                  = 0x0059,
    TELEM_ESC2_ERRORS                       = 0x005E,
    TELEM_ESC2_STATUS                       = 0x005F,

    TELEM_ESC3_GROUP                        = 0x0060,
    TELEM_ESC3_VOLTAGE                      = 0x0061,
    TELEM_ESC3_CURRENT                      = 0x0062,
    TELEM_ESC3_ERPM                         = 0x0063,
    TELEM_ESC3_POWER                        = 0x0064,
    TELEM_ESC3_THROTTLE                     = 0x0065,
    TELEM_ESC3_TEMP1                        = 0x0066,
    TELEM_ESC3_TEMP2                        = 0x0067,
    TELEM_ESC3_BEC_VOLTAGE                  = 0x0068,
    TELEM_ESC3_BEC_CURRENT                  = 0x0069,
    TELEM_ESC3_ERRORS                       = 0x006E,
    TELEM_ESC3_STATUS                       = 0x006F,

    TELEM_ESC4_GROUP                        = 0x0070,
    TELEM_ESC4_VOLTAGE                      = 0x0071,
    TELEM_ESC4_CURRENT                      = 0x0072,
    TELEM_ESC4_ERPM                         = 0x0073,
    TELEM_ESC4_POWER                        = 0x0074,
    TELEM_ESC4_THROTTLE                     = 0x0075,
    TELEM_ESC4_TEMP1                        = 0x0076,
    TELEM_ESC4_TEMP2                        = 0x0077,
    TELEM_ESC4_BEC_VOLTAGE                  = 0x0078,
    TELEM_ESC4_BEC_CURRENT                  = 0x0079,
    TELEM_ESC4_ERRORS                       = 0x007E,
    TELEM_ESC4_STATUS                       = 0x007F,

    TELEM_BEC_VOLTAGE                       = 0x0080,
    TELEM_BUS_VOLTAGE                       = 0x0081,
    TELEM_MCU_VOLTAGE                       = 0x0082,

    TELEM_BEC_CURRENT                       = 0x0090,
    TELEM_BUS_CURRENT                       = 0x0091,
    TELEM_MCU_CURRENT                       = 0x0092,

    TELEM_BEC_TEMPERATURE                   = 0x00A0,
    TELEM_ESC_TEMPERATURE                   = 0x00A1,
    TELEM_MCU_TEMPERATURE                   = 0x00A2,
    TELEM_MOTOR_TEMPERATURE                 = 0x00A3,

    TELEM_EXT1_TEMPERATURE                  = 0x00AA,
    TELEM_EXT2_TEMPERATURE                  = 0x00AB,
    TELEM_EXT3_TEMPERATURE                  = 0x00AC,
    TELEM_EXT4_TEMPERATURE                  = 0x00AD,

    TELEM_AIRSPEED                          = 0x00B0,
    TELEM_ALTITUDE                          = 0x00B1,
    TELEM_VARIOMETER                        = 0x00B2,

    TELEM_HEADSPEED                         = 0x00C0,
    TELEM_TAILSPEED                         = 0x00C1,

    TELEM_ATTITUDE_PITCH                    = 0x0100,
    TELEM_ATTITUDE_ROLL                     = 0x0101,
    TELEM_ATTITUDE_YAW                      = 0x0102,

    TELEM_ACCEL_X                           = 0x0110,
    TELEM_ACCEL_Y                           = 0x0111,
    TELEM_ACCEL_Z                           = 0x0112,

    TELEM_GPS_GROUP                         = 0x0200,
    TELEM_GPS_HEADING                       = 0x0201,
    TELEM_GPS_LATITUDE                      = 0x0202,
    TELEM_GPS_LONGITUDE                     = 0x0203,
    TELEM_GPS_ALTITUDE                      = 0x0204,
    TELEM_GPS_DISTANCE                      = 0x0205,
    TELEM_GPS_GROUNDSPEED                   = 0x0206,
    TELEM_GPS_SAT_COUNT                     = 0x020E,
    TELEM_GPS_DATE_TIME                     = 0x020F,

    TELEM_FC_CPU_GROUP                      = 0x0220,
    TELEM_FC_CPU_LOAD                       = 0x0221,
    TELEM_FC_SYS_LOAD                       = 0x0222,
    TELEM_FC_RT_LOAD                        = 0x0223,
    TELEM_FC_UPTIME                         = 0x022F,

    TELEM_FLIGHT_MODE                       = 0x0231,
    TELEM_ARMING_FLAGS                      = 0x0232,
    TELEM_GOVERNOR_STATE                    = 0x0233,

    TELEM_PROFILE_GORUP                     = 0x0240,
    TELEM_PID_PROFILE                       = 0x0241,
    TELEM_RATES_PROFILE                     = 0x0242,
    TELEM_BATTERY_PROFILE                   = 0x0243,
    TELEM_LED_PROFILE                       = 0x0244,

    TELEM_ADJFUNC                           = 0x0250,

    TELEM_ADC1                              = 0x0801,
    TELEM_ADC2                              = 0x0802,
    TELEM_ADC3                              = 0x0803,
    TELEM_ADC4                              = 0x0804,

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
    SENSOR_TEMPERATURE     = TELEM_ESC_TEMPERATURE,
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
    sensor_e            sensor_id;
    const char *        sensor_name;

    int                 min_delay;
    int                 max_delay;

    int                 length;

    tlmValue_f          value;

} telemetrySensor_t;


const telemetrySensor_t * telemetryGetSensor(sensor_e sensor);

inline int telemetryGetSensorValue(sensor_e sensor)
{
    return telemetryGetSensor(sensor)->value();
}

bool telemetryIsSensorEnabled(sensor_e sensor_id);
