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

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "pg/pg.h"
#include "pg/telemetry.h"

#include "common/streambuf.h"

#include "flight/motors.h"
#include "flight/servos.h"


/** Custom telemetry sensor types **/

typedef enum
{
    TELEM_NONE = 0,

    TELEM_HEARTBEAT,

    TELEM_BATTERY,
    TELEM_BATTERY_VOLTAGE,
    TELEM_BATTERY_CURRENT,
    TELEM_BATTERY_CONSUMPTION,
    TELEM_BATTERY_CHARGE_LEVEL,
    TELEM_BATTERY_TEMPERATURE,
    TELEM_BATTERY_CELL_COUNT,
    TELEM_BATTERY_CELL_VOLTAGES,

    TELEM_ROLL_CONTROL,
    TELEM_PITCH_CONTROL,
    TELEM_YAW_CONTROL,
    TELEM_COLLECTIVE_CONTROL,
    TELEM_THROTTLE_CONTROL,

    TELEM_ESC1_DATA,
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

    TELEM_ESC2_DATA,
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

    TELEM_ESC_VOLTAGE,
    TELEM_BEC_VOLTAGE,
    TELEM_BUS_VOLTAGE,
    TELEM_MCU_VOLTAGE,

    TELEM_ESC_CURRENT,
    TELEM_BEC_CURRENT,
    TELEM_BUS_CURRENT,
    TELEM_MCU_CURRENT,

    TELEM_ESC_TEMP,
    TELEM_BEC_TEMP,
    TELEM_MCU_TEMP,
    TELEM_AIR_TEMP,
    TELEM_MOTOR_TEMP,
    TELEM_EXHAUST_TEMP,

    TELEM_ALTITUDE,
    TELEM_VARIOMETER,

    TELEM_HEADSPEED,
    TELEM_TAILSPEED,
    TELEM_MOTOR_RPM,
    TELEM_TRANS_RPM,

    TELEM_ATTITUDE,
    TELEM_ATTITUDE_PITCH,
    TELEM_ATTITUDE_ROLL,
    TELEM_ATTITUDE_YAW,

    TELEM_ACCEL,
    TELEM_ACCEL_X,
    TELEM_ACCEL_Y,
    TELEM_ACCEL_Z,

    TELEM_GPS,
    TELEM_GPS_SATS,
    TELEM_GPS_PDOP,
    TELEM_GPS_HDOP,
    TELEM_GPS_VDOP,
    TELEM_GPS_COORD,
    TELEM_GPS_ALTITUDE,
    TELEM_GPS_HEADING,
    TELEM_GPS_GROUNDSPEED,
    TELEM_GPS_HOME_DISTANCE,
    TELEM_GPS_HOME_DIRECTION,
    TELEM_GPS_DATE_TIME,

    TELEM_LOAD,
    TELEM_CPU_LOAD,
    TELEM_SYS_LOAD,
    TELEM_RT_LOAD,

    TELEM_MODEL_ID,
    TELEM_FLIGHT_MODE,
    TELEM_ARMING_FLAGS,
    TELEM_RESCUE_STATE,
    TELEM_GOVERNOR_STATE,

    TELEM_PROFILES,
    TELEM_PID_PROFILE,
    TELEM_RATES_PROFILE,
    TELEM_BATTERY_PROFILE,
    TELEM_LED_PROFILE,
    TELEM_OSD_PROFILE,

    TELEM_ADJFUNC,

    TELEM_SENSOR_COUNT,

} sensor_id_e;


typedef struct telemetrySensor_s telemetrySensor_t;

typedef void (*telemetryEncode_f)(sbuf_t *buf, telemetrySensor_t *sensor);

typedef uint16_t sensor_code_t;

struct telemetrySensor_s {

    sensor_id_e             telid;
    sensor_code_t           tcode;

    uint16_t                min_interval;
    uint16_t                max_interval;

    bool                    active;
    bool                    update;
    int                     bucket;
    int                     value;

    telemetryEncode_f       encode;
};


int telemetrySensorValue(sensor_id_e id);
bool telemetrySensorActive(sensor_id_e id);


/** Legacy sensors **/

typedef enum {
    SENSOR_VOLTAGE         = BIT(0),
    SENSOR_CURRENT         = BIT(1),
    SENSOR_FUEL            = BIT(2),
    SENSOR_MODE            = BIT(3),
    SENSOR_ACC_X           = BIT(4),
    SENSOR_ACC_Y           = BIT(5),
    SENSOR_ACC_Z           = BIT(6),
    SENSOR_PITCH           = BIT(7),
    SENSOR_ROLL            = BIT(8),
    SENSOR_HEADING         = BIT(9),
    SENSOR_ALTITUDE        = BIT(10),
    SENSOR_VARIO           = BIT(11),
    SENSOR_LAT_LONG        = BIT(12),
    SENSOR_GROUND_SPEED    = BIT(13),
    SENSOR_DISTANCE        = BIT(14),
    ESC_SENSOR_CURRENT     = BIT(15),
    ESC_SENSOR_VOLTAGE     = BIT(16),
    ESC_SENSOR_RPM         = BIT(17),
    ESC_SENSOR_TEMPERATURE = BIT(18),
    SENSOR_TEMPERATURE     = BIT(19),
    SENSOR_CAP_USED        = BIT(20),
    SENSOR_ADJUSTMENT      = BIT(21),
    SENSOR_GOV_MODE        = BIT(22),
} sensor_e;

sensor_e telemetryGetLegacySensor(sensor_id_e sensor_id);

void legacySensorInit(void);

bool telemetryIsSensorEnabled(uint32_t sensor_bits);
