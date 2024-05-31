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
        .index = TELEM_##NAME, \
        .code = (CODE), \
        .name = (DESC), \
        .min_delay = (MIND), \
        .max_delay = (MAXD), \
        .value = (telemetryFunction_f)(GETF), \
        .encode = sensorEncode##ENCODER, \
    }

const telemetrySensor_t telemetry_sensors[TELEM_SENSOR_COUNT] =
{
    TLM_SENSOR(HEARTBEAT,               "BEAT",     0x0000,   100,   100,    Nil,    getNil),

    TLM_SENSOR(MODEL_ID,                "ID  ",     0x0001,  1000,  1000,    U16,    getNil),

    TLM_SENSOR(BATTERY,                 "Batt",     0x0010,  1000,  1000,    Nil,    getNil),
    TLM_SENSOR(BATTERY_VOLTAGE,         "Vbat",     0x0011,   100,  1000,    Nil,    getNil),
    TLM_SENSOR(BATTERY_CURRENT,         "Curr",     0x0012,   100,  1000,    Nil,    getNil),
    TLM_SENSOR(BATTERY_CONSUMPTION,     "mAh ",     0x0013,   100,  1000,    Nil,    getNil),
    TLM_SENSOR(BATTERY_CHARGE_LEVEL,    "Bat%",     0x0014,   100,  1000,    Nil,    getNil),
    TLM_SENSOR(BATTERY_TEMPERATURE,     "Tbat",     0x0015,   100,  1000,    Nil,    getNil),

    TLM_SENSOR(BATTERY_CELLS,           "Cell",     0x0040,   100,  1000,    Nil,    getNil),

    TLM_SENSOR(ESC1_DATA,               "ESC1",     0x0040,   100,  1000,    Nil,    getNil),
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

    TLM_SENSOR(ESC_VOLTAGE,             "Vesc",     0x0080,   100,  1000,    U16,    getNil),
    TLM_SENSOR(BEC_VOLTAGE,             "Vbec",     0x0081,   100,  1000,    U16,    getNil),
    TLM_SENSOR(BUS_VOLTAGE,             "Vbus",     0x0082,   100,  1000,    U16,    getNil),
    TLM_SENSOR(MCU_VOLTAGE,             "Vmcu",     0x0083,   100,  1000,    U16,    getNil),

    TLM_SENSOR(ESC_CURRENT,             "Iesc",     0x0090,   100,  1000,    U16,    getNil),
    TLM_SENSOR(BEC_CURRENT,             "Ibec",     0x0091,   100,  1000,    U16,    getNil),
    TLM_SENSOR(BUS_CURRENT,             "Ibus",     0x0092,   100,  1000,    U16,    getNil),
    TLM_SENSOR(MCU_CURRENT,             "Imcu",     0x0093,   100,  1000,    U16,    getNil),

    TLM_SENSOR(ESC_TEMP,                "Tesc",     0x00A0,   100,  1000,    U8,     getNil),
    TLM_SENSOR(BEC_TEMP,                "Tbec",     0x00A1,   100,  1000,    U8,     getNil),
    TLM_SENSOR(MCU_TEMP,                "Tmcu",     0x00A3,   100,  1000,    U8,     getNil),
    TLM_SENSOR(AIR_TEMP,                "Tair",     0x00A4,   100,  1000,    U8,     getNil),
    TLM_SENSOR(MOTOR_TEMP,              "Tmtr",     0x00A5,   100,  1000,    U8,     getNil),

    TLM_SENSOR(ALTITUDE,                "Alt",      0x00B1,   100,  1000,    S24,    getNil),
    TLM_SENSOR(VARIOMETER,              "Vari",     0x00B2,   100,  1000,    S16,    getNil),

    TLM_SENSOR(HEADSPEED,               "HSpd",     0x00C0,   100,  1000,    U16,    getHeadSpeed),
    TLM_SENSOR(TAILSPEED,               "TSpd",     0x00C1,   100,  1000,    U16,    getTailSpeed),
    TLM_SENSOR(MOTOR_RPM,               "Mrpm",     0x00C2,   100,  1000,    U16,    getNil),

    TLM_SENSOR(ATTITUDE,                "Att",      0x0100,   100,  1000,    Nil,    getNil),
    TLM_SENSOR(ATTITUDE_PITCH,          "AttP",     0x0101,   100,  1000,    S16,    getNil),
    TLM_SENSOR(ATTITUDE_ROLL,           "AttR",     0x0102,   100,  1000,    S16,    getNil),
    TLM_SENSOR(ATTITUDE_YAW,            "AttY",     0x0103,   100,  1000,    S16,    getNil),

    TLM_SENSOR(ACCEL,                   "AccX",     0x0110,   100,  1000,    Nil,    getNil),
    TLM_SENSOR(ACCEL_X,                 "AccX",     0x0111,   100,  1000,    S16,    getNil),
    TLM_SENSOR(ACCEL_Y,                 "AccY",     0x0112,   100,  1000,    S16,    getNil),
    TLM_SENSOR(ACCEL_Z,                 "AccZ",     0x0113,   100,  1000,    S16,    getNil),

    TLM_SENSOR(GPS,                     "GPS ",     0x0120,   100,  1000,    Nil,    getNil),
    TLM_SENSOR(GPS_SAT_COUNT,           "Gsat",     0x0121,   100,  1000,    Nil,    getNil),
    TLM_SENSOR(GPS_HEADING,             "GHdg",     0x0122,   100,  1000,    Nil,    getNil),
    TLM_SENSOR(GPS_LATITUDE,            "Glat",     0x0123,   100,  1000,    Nil,    getNil),
    TLM_SENSOR(GPS_LONGITUDE,           "Glng",     0x0124,   100,  1000,    Nil,    getNil),
    TLM_SENSOR(GPS_ALTITUDE,            "Galt",     0x0125,   100,  1000,    Nil,    getNil),
    TLM_SENSOR(GPS_DISTANCE,            "Gdis",     0x0126,   100,  1000,    Nil,    getNil),
    TLM_SENSOR(GPS_GROUNDSPEED,         "Gspd",     0x0127,   100,  1000,    Nil,    getNil),
    TLM_SENSOR(GPS_DATE_TIME,           "Gtim",     0x012F,   100,  1000,    Nil,    getNil),

    TLM_SENSOR(FC,                      "FC ",      0x0140,   100,  1000,    Nil,    getNil),
    TLM_SENSOR(FC_UPTIME,               "UpTm",     0x0141,   100,  1000,    Nil,    getNil),
    TLM_SENSOR(FC_CPU_LOAD,             "CPU*",     0x0142,   100,  1000,    Nil,    getNil),
    TLM_SENSOR(FC_SYS_LOAD,             "SYS*",     0x0143,   100,  1000,    Nil,    getNil),
    TLM_SENSOR(FC_RT_LOAD,              "RT*",      0x0144,   100,  1000,    Nil,    getNil),

};


const telemetrySensor_t * telemetryGetSensor(sensor_id_e sensor_id)
{
    return &telemetry_sensors[sensor_id];
}

const telemetrySensor_t * telemetryGetSensorCode(uint16_t sensor_code)
{
    for (unsigned i = 0; i < ARRAYLEN(telemetry_sensors); i++) {
        const telemetrySensor_t * sensor = &telemetry_sensors[i];
        if (sensor->code == sensor_code)
            return sensor;
    }

    return NULL;
}


sensor_e telemetrySensorId2Bit(sensor_id_e sensor_id)
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

#endif /* USE_TELEMETRY */
