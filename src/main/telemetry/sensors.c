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

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_TELEMETRY

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/telemetry.h"

#include "telemetry/sensors.h"


sensor_e telemetrySensorGetLegacy(sensor_id_e sensor_id)
{
    switch (sensor_id)
    {
        case TELEM_BATTERY_VOLTAGE:
            return SENSOR_VOLTAGE;
        case TELEM_BATTERY_CURRENT:
            return SENSOR_CURRENT;
        case TELEM_BATTERY_CONSUMPTION:
            return SENSOR_CAP_USED;
        case TELEM_BATTERY_CHARGE_LEVEL:
            return SENSOR_FUEL;
        case TELEM_FLIGHT_MODE:
            return SENSOR_MODE;
        case TELEM_ACCEL_X:
            return SENSOR_ACC_X;
        case TELEM_ACCEL_Y:
            return SENSOR_ACC_Y;
        case TELEM_ACCEL_Z:
            return SENSOR_ACC_Z;
        case TELEM_ATTITUDE_PITCH:
            return SENSOR_PITCH;
        case TELEM_ATTITUDE_ROLL:
            return SENSOR_ROLL;
        case TELEM_ATTITUDE_YAW:
            return SENSOR_HEADING;
        case TELEM_ALTITUDE:
            return SENSOR_ALTITUDE;
        case TELEM_VARIOMETER:
            return SENSOR_VARIO;
        case TELEM_GPS_LATITUDE:
            return SENSOR_LAT_LONG;
        case TELEM_GPS_LONGITUDE:
            return SENSOR_LAT_LONG;
        case TELEM_GPS_GROUNDSPEED:
            return SENSOR_GROUND_SPEED;
        case TELEM_GPS_DISTANCE:
            return SENSOR_DISTANCE;
        case TELEM_MCU_TEMP:
            return SENSOR_TEMPERATURE;
        case TELEM_ADJFUNC:
            return SENSOR_ADJUSTMENT;
        case TELEM_GOVERNOR_STATE:
            return SENSOR_GOV_MODE;
        case TELEM_ESC_CURRENT:
            return ESC_SENSOR_CURRENT;
        case TELEM_ESC_VOLTAGE:
            return ESC_SENSOR_VOLTAGE;
        case TELEM_MOTOR_RPM:
            return ESC_SENSOR_RPM;
        case TELEM_ESC_TEMP:
            return ESC_SENSOR_TEMPERATURE;
        default:
            return 0;
    }

    return 0;
}


/** Legacy sensors **/

static uint32_t telemetry_legacy_sensors = 0;

bool telemetryIsSensorEnabled(uint32_t sensor_bits)
{
    return (telemetry_legacy_sensors & sensor_bits);
}

void INIT_CODE legacySensorInit(void)
{
    telemetry_legacy_sensors = 0;

    for (int i = 0; i < TELEM_SENSOR_SLOT_COUNT; i++) {
        sensor_id_e id = telemetryConfig()->telemetry_sensors[i];
        if (id) {
            telemetry_legacy_sensors |= telemetrySensorGetLegacy(id);
        }
    }
}

#endif /* USE_TELEMETRY */
