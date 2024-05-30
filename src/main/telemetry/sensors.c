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

#include "platform.h"

#ifdef USE_TELEMETRY

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/telemetry.h"

#include "telemetry/sensors.h"

#include "flight/motors.h"
#include "flight/servos.h"


static int getNull(void)
{
    return 0;
}


#define TLM_SENSOR(NAME, DESC, MIND, MAXD, SIZE, GETV) \
    [TELEM_##NAME] = { TELEM_##NAME, DESC, (MIND), (MAXD), (SIZE), (GETV) }

const telemetrySensor_t telemetry_sensors[TELEM_SENSOR_COUNT] =
{
    TLM_SENSOR(NONE,           "NONE",    0,    0,    0,    getNull),
    TLM_SENSOR(HEADSPEED,      "HSPD",   40, 1000,    2,    getHeadSpeed),
    TLM_SENSOR(TAILSPEED,      "TSPD",   40, 1000,    2,    getTailSpeed),
};

const telemetrySensor_t * telemetryGetSensor(sensor_e sensor)
{
    return &telemetry_sensors[sensor];
}


bool telemetryIsSensorEnabled(sensor_e sensor_id)
{
    for (int i = 0; i < TELEM_SENSOR_SLOT_COUNT; i++) {
        if (telemetryConfig()->telemetry_sensors[i] == sensor_id)
            return true;
    }
    return false;
}


#endif /* USE_TELEMETRY */
