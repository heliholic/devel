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


#define STATIC

STATIC int getNil(void)
{
    return 0;
}


STATIC int sensorEncodeNil(sbuf_t *buf, telemetryValue_t value)
{
    UNUSED(value);
    sbufWriteU8(buf, 0);
    return 1;
}


STATIC int sensorEncodeU8(sbuf_t *buf, telemetryValue_t value)
{
    sbufWriteU8(buf, 1);
    sbufWriteU8(buf, value.u32);
    return 2;
}

STATIC int sensorEncodeS8(sbuf_t *buf, telemetryValue_t value)
{
    sbufWriteU8(buf, 1);
    sbufWriteS8(buf, value.s32);
    return 2;
}

STATIC int sensorEncodeU16(sbuf_t *buf, telemetryValue_t value)
{
    sbufWriteU8(buf, 2);
    sbufWriteU16BE(buf, value.u32);
    return 3;
}

STATIC int sensorEncodeS16(sbuf_t *buf, telemetryValue_t value)
{
    sbufWriteU8(buf, 2);
    sbufWriteS16BE(buf, value.s32);
    return 3;
}

STATIC int sensorEncodeU24(sbuf_t *buf, telemetryValue_t value)
{
    sbufWriteU8(buf, 3);
    sbufWriteU24BE(buf, value.u32);
    return 4;
}

STATIC int sensorEncodeS24(sbuf_t *buf, telemetryValue_t value)
{
    sbufWriteU8(buf, 3);
    sbufWriteS24BE(buf, value.s32);
    return 4;
}

STATIC int sensorEncodeU32(sbuf_t *buf, telemetryValue_t value)
{
    sbufWriteU8(buf, 4);
    sbufWriteU32BE(buf, value.u32);
    return 5;
}

STATIC int sensorEncodeS32(sbuf_t *buf, telemetryValue_t value)
{
    sbufWriteU8(buf, 4);
    sbufWriteS32BE(buf, value.s32);
    return 5;
}


#define TLM_SENSOR(NAME, DESC, CODE, MIND, MAXD, ENCODER, GETF) \
    [TELEM_##NAME] = { \
        .sensor_index = TELEM_##NAME, \
        .sensor_code = (CODE), \
        .sensor_name = (DESC), \
        .min_delay = (MIND), \
        .max_delay = (MAXD), \
        .value = (telemetryFunction_f)(GETF), \
        .encode = sensorEncode##ENCODER, \
    }

const telemetrySensor_t telemetry_sensors[TELEM_SENSOR_COUNT] =
{
    TLM_SENSOR(MODEL_ID,                "ID  ",     0x0001,     0,     0,    U16,    getNil),

    TLM_SENSOR(BATTERY_GROUP,           "Batt",     0x0010,     0,     0,    Nil,    getNil),
    TLM_SENSOR(BATTERY_VOLTAGE,         "Vbat",     0x0011,     0,     0,    Nil,    getNil),
    TLM_SENSOR(BATTERY_CURRENT,         "Curr",     0x0012,     0,     0,    Nil,    getNil),
    TLM_SENSOR(BATTERY_CONSUMPTION,     "mAh ",     0x0013,     0,     0,    Nil,    getNil),
    TLM_SENSOR(BATTERY_CHARGE_LEVEL,    "Bat%",     0x0014,     0,     0,    Nil,    getNil),
    TLM_SENSOR(BATTERY_TEMPERATURE,     "Tbat",     0x0015,     0,     0,    Nil,    getNil),

    TLM_SENSOR(BATTERY_CELLS,           "Cell",     0x0040,   100,  1000,    Nil,    getNil),

    TLM_SENSOR(ESC1_VOLTAGE,            "Vol1",     0x0041,   100,  1000,    U16,    getNil),
    TLM_SENSOR(ESC1_CURRENT,            "Cur1",     0x0042,   100,  1000,    U16,    getNil),
    TLM_SENSOR(ESC1_ERPM,               "RPM1",     0x0043,   100,  1000,    U16,    getNil),
    TLM_SENSOR(ESC1_POWER,              "Pow1",     0x0044,   100,  1000,    U16,    getNil),
    TLM_SENSOR(ESC1_THROTTLE,           "Thr1",     0x0045,   100,  1000,    U8,     getNil),
    TLM_SENSOR(ESC1_TEMP1,              "TeE1",     0x0046,   100,  1000,    U8,     getNil),
    TLM_SENSOR(ESC1_TEMP2,              "TeB1",     0x0047,   100,  1000,    U8,     getNil),
    TLM_SENSOR(ESC1_BEC_VOLTAGE,        "VoB1",     0x0048,   100,  1000,    U16,    getNil),
    TLM_SENSOR(ESC1_BEC_CURRENT,        "CuB1",     0x0049,   100,  1000,    U16,    getNil),
    TLM_SENSOR(ESC1_ERRORS,             "Err1",     0x004E,   100,  1000,    U32,    getNil),
    TLM_SENSOR(ESC1_STATUS,             "Est1",     0x004F,   100,  1000,    U32,    getNil),

    TLM_SENSOR(BEC_VOLTAGE,             "Vbec",     0x0080,   100,  1000,    U16,    getNil),
    TLM_SENSOR(BUS_VOLTAGE,             "Vbus",     0x0081,   100,  1000,    U16,    getNil),
    TLM_SENSOR(MCU_VOLTAGE,             "Vmcu",     0x0082,   100,  1000,    U16,    getNil),

    TLM_SENSOR(BEC_CURRENT,             "Ibec",     0x0090,   100,  1000,    U16,    getNil),
    TLM_SENSOR(BUS_CURRENT,             "Ibus",     0x0091,   100,  1000,    U16,    getNil),
    TLM_SENSOR(MCU_CURRENT,             "Imcu",     0x0092,   100,  1000,    U16,    getNil),

    TLM_SENSOR(BEC_TEMP,                "Tbec",     0x00A0,   100,  1000,    U8,     getNil),
    TLM_SENSOR(ESC_TEMP,                "Tesc",     0x00A1,   100,  1000,    U8,     getNil),
    TLM_SENSOR(MCU_TEMP,                "Tmcu",     0x00A2,   100,  1000,    U8,     getNil),
    TLM_SENSOR(MOTOR_TEMP,              "Tmtr",     0x00A3,   100,  1000,    U8,     getNil),

    TLM_SENSOR(ALTITUDE,                "Alt",      0x00B1,   100,  1000,    S24,    getNil),
    TLM_SENSOR(VARIOMETER,              "Vari",     0x00B2,   100,  1000,    S16,    getNil),

    TLM_SENSOR(HEADSPEED,               "HSpd",     0x00C0,   100,  1000,    U16,    getHeadSpeed),
    TLM_SENSOR(TAILSPEED,               "TSpd",     0x00C1,   100,  1000,    U16,    getTailSpeed),

    TLM_SENSOR(ATTITUDE_PITCH,          "AttP",     0x0100,   100,  1000,    S16,    getNil),
    TLM_SENSOR(ATTITUDE_ROLL,           "AttR",     0x0101,   100,  1000,    S16,    getNil),
    TLM_SENSOR(ATTITUDE_YAW,            "AttY",     0x0102,   100,  1000,    S16,    getNil),

    TLM_SENSOR(ACCEL_X,                 "AccX",     0x0110,   100,  1000,    S16,    getNil),
    TLM_SENSOR(ACCEL_Y,                 "AccY",     0x0111,   100,  1000,    S16,    getNil),
    TLM_SENSOR(ACCEL_Z,                 "AccZ",     0x0112,   100,  1000,    S16,    getNil),
};


const telemetrySensor_t * telemetryGetSensor(sensor_id_e sensor_id)
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

sensor_e telemetrySensorBit2Id(sensor_e sensor_bit)
{
    switch (sensor_bit)
    {
        case SENSOR_VOLTAGE:
            return TELEM_BATTERY_VOLTAGE;
        case SENSOR_CURRENT:
            return TELEM_BATTERY_CURRENT;
        case SENSOR_FUEL:
            return TELEM_BATTERY_CHARGE_LEVEL;
        case SENSOR_MODE:
            return TELEM_FLIGHT_MODE;
        case SENSOR_ACC_X:
            return TELEM_ACCEL_X;
        case SENSOR_ACC_Y:
            return TELEM_ACCEL_Y;
        case SENSOR_ACC_Z:
            return TELEM_ACCEL_Z;
        case SENSOR_PITCH:
            return TELEM_ATTITUDE_PITCH;
        case SENSOR_ROLL:
            return TELEM_ATTITUDE_ROLL;
        case SENSOR_HEADING:
            return TELEM_ATTITUDE_YAW;
        case SENSOR_ALTITUDE:
            return TELEM_ALTITUDE;
        case SENSOR_VARIO:
            return TELEM_VARIOMETER;
        case SENSOR_LAT_LONG:
            return TELEM_GPS_LATITUDE;
        case SENSOR_GROUND_SPEED:
            return TELEM_GPS_GROUNDSPEED;
        case SENSOR_DISTANCE:
            return TELEM_GPS_DISTANCE;
        case SENSOR_TEMPERATURE:
            return TELEM_ESC_TEMP;
        case SENSOR_CAP_USED:
            return TELEM_BATTERY_CONSUMPTION;
        case SENSOR_ADJUSTMENT:
            return TELEM_ADJFUNC;
        case SENSOR_GOV_MODE:
            return TELEM_GOVERNOR_STATE;
        case ESC_SENSOR_CURRENT:
            return TELEM_ESC1_CURRENT;
        case ESC_SENSOR_VOLTAGE:
            return TELEM_ESC1_VOLTAGE;
        case ESC_SENSOR_RPM:
            return TELEM_ESC1_ERPM;
        case ESC_SENSOR_TEMPERATURE:
            return TELEM_ESC1_TEMP1;
        default:
            return TELEM_NONE;
    }

    return TELEM_NONE;
}

static uint32_t telemetry_legacy_sensors = 0;

bool telemetryIsSensorEnabled(uint32_t sensor_bits)
{
    return (telemetry_legacy_sensors & sensor_bits);
}


#endif /* USE_TELEMETRY */
