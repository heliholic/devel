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


#define TLM_SENSOR(NAME, DESC, CODE, MIND, MAXD, SIZE, GETV) \
    [TELEM_##NAME] = { TELEM_##NAME, (CODE), (DESC), (MIND), (MAXD), (SIZE), (GETV) }

const telemetrySensor_t telemetry_sensors[TELEM_SENSOR_COUNT] =
{
    TLM_SENSOR(MODEL_ID,                "ID  ",     0x0001,     0,    0,    0,    getNull),

    TLM_SENSOR(BATTERY,                 "Batt",     0x0010,     0,    0,    0,    getNull),
    TLM_SENSOR(BATTERY_VOLTAGE,         "Vbat",     0x0011,     0,    0,    0,    getNull),
    TLM_SENSOR(BATTERY_CURRENT,         "Curr",     0x0012,     0,    0,    0,    getNull),
    TLM_SENSOR(BATTERY_CONSUMPTION,     "mAh ",     0x0013,     0,    0,    0,    getNull),
    TLM_SENSOR(BATTERY_CHARGE_LEVEL,    "Bat%",     0x0014,     0,    0,    0,    getNull),
    TLM_SENSOR(BATTERY_TEMPERATURE,     "Tbat",     0x0015,     0,    0,    0,    getNull),

    TLM_SENSOR(BATTERY_CELLS,           "Cell",     0x0040,   100, 1000,    2,    getNull),

    TLM_SENSOR(ESC1_VOLTAGE,            "Vol1",     0x0041,   100, 1000,    2,    getNull),
    TLM_SENSOR(ESC1_CURRENT,            "Cur1",     0x0042,   100, 1000,    2,    getNull),
    TLM_SENSOR(ESC1_ERPM,               "RPM1",     0x0043,   100, 1000,    4,    getNull),
    TLM_SENSOR(ESC1_POWER,              "Pow1",     0x0044,   100, 1000,    2,    getNull),
    TLM_SENSOR(ESC1_THROTTLE,           "Thr1",     0x0045,   100, 1000,    2,    getNull),
    TLM_SENSOR(ESC1_TEMP1,              "TeE1",     0x0046,   100, 1000,    1,    getNull),
    TLM_SENSOR(ESC1_TEMP2,              "TeB1",     0x0047,   100, 1000,    1,    getNull),
    TLM_SENSOR(ESC1_BEC_VOLTAGE,        "VoB1",     0x0048,   100, 1000,    2,    getNull),
    TLM_SENSOR(ESC1_BEC_CURRENT,        "CuB1",     0x0049,   100, 1000,    2,    getNull),
    TLM_SENSOR(ESC1_ERRORS,             "Err1",     0x004E,   100, 1000,    4,    getNull),
    TLM_SENSOR(ESC1_STATUS,             "Est1",     0x004F,   100, 1000,    4,    getNull),

    TLM_SENSOR(BEC_VOLTAGE,             "Vbec",     0x0080,   100, 1000,    2,    getNull),
    TLM_SENSOR(BUS_VOLTAGE,             "Vbus",     0x0081,   100, 1000,    2,    getNull),
    TLM_SENSOR(MCU_VOLTAGE,             "Vmcu",     0x0082,   100, 1000,    2,    getNull),

    TLM_SENSOR(BEC_CURRENT,             "Ibec",     0x0090,   100, 1000,    2,    getNull),
    TLM_SENSOR(BUS_CURRENT,             "Ibus",     0x0091,   100, 1000,    2,    getNull),
    TLM_SENSOR(MCU_CURRENT,             "Imcu",     0x0092,   100, 1000,    2,    getNull),

    TLM_SENSOR(BEC_TEMP,                "Tbec",     0x00A0,   100, 1000,    1,    getNull),
    TLM_SENSOR(ESC_TEMP,                "Tesc",     0x00A1,   100, 1000,    1,    getNull),
    TLM_SENSOR(MCU_TEMP,                "Tmcu",     0x00A2,   100, 1000,    1,    getNull),
    TLM_SENSOR(MOTOR_TEMP,              "Tmtr",     0x00A3,   100, 1000,    1,    getNull),

    TLM_SENSOR(ALTITUDE,                "Alt",      0x00B1,   100, 1000,    3,    getNull),
    TLM_SENSOR(VARIOMETER,              "Vari",     0x00B2,   100, 1000,    2,    getNull),

    TLM_SENSOR(HEADSPEED,               "HSpd",     0x00C0,    40, 1000,    2,    getHeadSpeed),
    TLM_SENSOR(TAILSPEED,               "TSpd",     0x00C1,    40, 1000,    2,    getTailSpeed),

    TLM_SENSOR(ATTITUDE_PITCH,          "AttP",     0x0100,   100, 1000,    2,    getNull),
    TLM_SENSOR(ATTITUDE_ROLL,           "AttR",     0x0101,   100, 1000,    2,    getNull),
    TLM_SENSOR(ATTITUDE_YAW,            "AttY",     0x0102,   100, 1000,    2,    getNull),

    TLM_SENSOR(ACCEL_X,                 "AccX",     0x0110,   100, 1000,    2,    getNull),
    TLM_SENSOR(ACCEL_Y,                 "AccY",     0x0111,   100, 1000,    2,    getNull),
    TLM_SENSOR(ACCEL_Z,                 "AccZ",     0x0112,   100, 1000,    2,    getNull),

};


const telemetrySensor_t * telemetryGetSensor(sensor_e sensor_id)
{
    return &telemetry_sensors[sensor_id];
}

const telemetrySensor_t * telemetryGetSensorCode(uint16_t sensor_code)
{
    for (unsigned i = 0; i < ARRAYLEN(telemetry_sensors); i++) {
        const telemetrySensor_t * sensor = &telemetry_sensors[i];
        if (sensor->sensor_code == sensor_code)
            return sensor;
    }

    return NULL;
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
