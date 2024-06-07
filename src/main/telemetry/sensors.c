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

#include "sensors/battery.h"
#include "sensors/voltage.h"
#include "sensors/current.h"
#include "sensors/esc_sensor.h"
#include "sensors/adcinternal.h"

#include "flight/position.h"

#include "scheduler/scheduler.h"

#include "telemetry/sensors.h"


/** Sensor functions **/

int telemetrySensorValue(sensor_id_e id)
{
    voltageMeter_t voltage;
    currentMeter_t current;

    switch (id) {
        case TELEM_NONE:
            return 0;

        case TELEM_HEARTBEAT:
            return 0;

        case TELEM_MODEL_ID:
            return 0;

        case TELEM_BATTERY:
            return 0;
        case TELEM_BATTERY_CELL_COUNT:
            return getBatteryCellCount();
        case TELEM_BATTERY_VOLTAGE:
            return getBatteryVoltage();
        case TELEM_BATTERY_CURRENT:
            return getBatteryCurrent();
        case TELEM_BATTERY_CONSUMPTION:
            return getBatteryCapacityUsed();
        case TELEM_BATTERY_CHARGE_LEVEL:
            return calculateBatteryPercentageRemaining();
        case TELEM_BATTERY_TEMPERATURE:
            return 0;
        case TELEM_BATTERY_CELL_VOLTAGES:
            return 0;

        case TELEM_ESC1_DATA:
        case TELEM_ESC1_VOLTAGE:
        case TELEM_ESC1_CURRENT:
        case TELEM_ESC1_ERPM:
        case TELEM_ESC1_POWER:
        case TELEM_ESC1_THROTTLE:
        case TELEM_ESC1_TEMP1:
        case TELEM_ESC1_TEMP2:
        case TELEM_ESC1_BEC_VOLTAGE:
        case TELEM_ESC1_BEC_CURRENT:
        case TELEM_ESC1_ERRORS:
        case TELEM_ESC1_STATUS:
            return 0;

        case TELEM_ESC2_DATA:
        case TELEM_ESC2_VOLTAGE:
        case TELEM_ESC2_CURRENT:
        case TELEM_ESC2_ERPM:
        case TELEM_ESC2_POWER:
        case TELEM_ESC2_THROTTLE:
        case TELEM_ESC2_TEMP1:
        case TELEM_ESC2_TEMP2:
        case TELEM_ESC2_BEC_VOLTAGE:
        case TELEM_ESC2_BEC_CURRENT:
        case TELEM_ESC2_ERRORS:
        case TELEM_ESC2_STATUS:
            return 0;

        case TELEM_ESC_VOLTAGE:
            return voltageMeterRead(VOLTAGE_METER_ID_ESC_COMBINED, &voltage) ? voltage.voltage : 0;
        case TELEM_BEC_VOLTAGE:
            return voltageMeterRead(VOLTAGE_METER_ID_BEC, &voltage) ? voltage.voltage : 0;
        case TELEM_BUS_VOLTAGE:
            return voltageMeterRead(VOLTAGE_METER_ID_BUS, &voltage) ? voltage.voltage : 0;
        case TELEM_MCU_VOLTAGE:
            return voltageMeterRead(VOLTAGE_METER_ID_MCU, &voltage) ? voltage.voltage : 0;

        case TELEM_ESC_CURRENT:
            return currentMeterRead(CURRENT_METER_ID_ESC_COMBINED, &current) ? current.current : 0;
        case TELEM_BEC_CURRENT:
            return currentMeterRead(CURRENT_METER_ID_BEC, &current) ? current.current : 0;
        case TELEM_BUS_CURRENT:
            return currentMeterRead(CURRENT_METER_ID_BUS, &current) ? current.current : 0;
        case TELEM_MCU_CURRENT:
            return currentMeterRead(CURRENT_METER_ID_MCU, &current) ? current.current : 0;

        case TELEM_MCU_TEMP:
            return getCoreTemperatureCelsius();

        case TELEM_ESC_TEMP:
        case TELEM_BEC_TEMP:
        case TELEM_AIR_TEMP:
        case TELEM_MOTOR_TEMP:
        case TELEM_EXHAUST_TEMP:
            return 0;

        case TELEM_ALTITUDE:
            return getEstimatedAltitudeCm();
        case TELEM_VARIOMETER:
            return getEstimatedVarioCms();

        case TELEM_HEADSPEED:
            return getHeadSpeed();
        case TELEM_TAILSPEED:
            return getTailSpeed();

        case TELEM_MOTOR_RPM:
            return 0;
        case TELEM_TRANS_RPM:
            return 0;

        case TELEM_ATTITUDE:
        case TELEM_ATTITUDE_PITCH:
        case TELEM_ATTITUDE_ROLL:
        case TELEM_ATTITUDE_YAW:
            return 0;

        case TELEM_ACCEL:
        case TELEM_ACCEL_X:
        case TELEM_ACCEL_Y:
        case TELEM_ACCEL_Z:
            return 0;

        case TELEM_GPS:
        case TELEM_GPS_SATS:
        case TELEM_GPS_COORD:
        case TELEM_GPS_HEADING:
        case TELEM_GPS_ALTITUDE:
        case TELEM_GPS_DISTANCE:
        case TELEM_GPS_GROUNDSPEED:
        case TELEM_GPS_DATE_TIME:
            return 0;

        case TELEM_FC:
            return 0;

        case TELEM_FC_UPTIME:
            return millis();

        case TELEM_FC_CPU_LOAD:
            return getAverageCPULoadPercent();
        case TELEM_FC_SYS_LOAD:
            return getAverageSystemLoadPercent();
        case TELEM_FC_RT_LOAD:
            return getMaxRealTimeLoadPercent();

        case TELEM_FLIGHT_MODE:
        case TELEM_ARMING_FLAGS:
        case TELEM_RESCUE_STATE:
        case TELEM_GOVERNOR_STATE:
            return 0;

        case TELEM_PROFILES:
        case TELEM_PID_PROFILE:
            return getCurrentPidProfileIndex();
        case TELEM_RATES_PROFILE:
            return getCurrentControlRateProfileIndex();
        case TELEM_BATTERY_PROFILE:
        case TELEM_LED_PROFILE:
            return 0;

        case TELEM_ADJFUNC:
            return 0;

        default:
            return 0;
    }

    return 0;
}


bool telemetrySensorActive(sensor_id_e id)
{
    switch (id) {
        case TELEM_NONE:
            return false;

        case TELEM_HEARTBEAT:
            return false;

        case TELEM_MODEL_ID:

        case TELEM_BATTERY:
        case TELEM_BATTERY_VOLTAGE:
        case TELEM_BATTERY_CURRENT:
        case TELEM_BATTERY_CONSUMPTION:
        case TELEM_BATTERY_CHARGE_LEVEL:
        case TELEM_BATTERY_TEMPERATURE:
        case TELEM_BATTERY_CELL_VOLTAGES:
        case TELEM_BATTERY_CELL_COUNT:

        case TELEM_ESC1_DATA:
        case TELEM_ESC1_VOLTAGE:
        case TELEM_ESC1_CURRENT:
        case TELEM_ESC1_ERPM:
        case TELEM_ESC1_POWER:
        case TELEM_ESC1_THROTTLE:
        case TELEM_ESC1_TEMP1:
        case TELEM_ESC1_TEMP2:
        case TELEM_ESC1_BEC_VOLTAGE:
        case TELEM_ESC1_BEC_CURRENT:
        case TELEM_ESC1_ERRORS:
        case TELEM_ESC1_STATUS:

        case TELEM_ESC2_DATA:
        case TELEM_ESC2_VOLTAGE:
        case TELEM_ESC2_CURRENT:
        case TELEM_ESC2_ERPM:
        case TELEM_ESC2_POWER:
        case TELEM_ESC2_THROTTLE:
        case TELEM_ESC2_TEMP1:
        case TELEM_ESC2_TEMP2:
        case TELEM_ESC2_BEC_VOLTAGE:
        case TELEM_ESC2_BEC_CURRENT:
        case TELEM_ESC2_ERRORS:
        case TELEM_ESC2_STATUS:

        case TELEM_ESC_VOLTAGE:
        case TELEM_BEC_VOLTAGE:
        case TELEM_BUS_VOLTAGE:
        case TELEM_MCU_VOLTAGE:

        case TELEM_ESC_CURRENT:
        case TELEM_BEC_CURRENT:
        case TELEM_BUS_CURRENT:
        case TELEM_MCU_CURRENT:

        case TELEM_ESC_TEMP:
        case TELEM_BEC_TEMP:
        case TELEM_MCU_TEMP:
        case TELEM_AIR_TEMP:
        case TELEM_MOTOR_TEMP:
        case TELEM_EXHAUST_TEMP:

        case TELEM_ALTITUDE:
        case TELEM_VARIOMETER:

        case TELEM_HEADSPEED:
        case TELEM_TAILSPEED:
        case TELEM_MOTOR_RPM:
        case TELEM_TRANS_RPM:

        case TELEM_ATTITUDE:
        case TELEM_ATTITUDE_PITCH:
        case TELEM_ATTITUDE_ROLL:
        case TELEM_ATTITUDE_YAW:

        case TELEM_ACCEL:
        case TELEM_ACCEL_X:
        case TELEM_ACCEL_Y:
        case TELEM_ACCEL_Z:

        case TELEM_GPS:
        case TELEM_GPS_SATS:
        case TELEM_GPS_COORD:
        case TELEM_GPS_HEADING:
        case TELEM_GPS_ALTITUDE:
        case TELEM_GPS_DISTANCE:
        case TELEM_GPS_GROUNDSPEED:
        case TELEM_GPS_DATE_TIME:

        case TELEM_FC:
        case TELEM_FC_UPTIME:
        case TELEM_FC_CPU_LOAD:
        case TELEM_FC_SYS_LOAD:
        case TELEM_FC_RT_LOAD:

        case TELEM_FLIGHT_MODE:
        case TELEM_ARMING_FLAGS:
        case TELEM_RESCUE_STATE:
        case TELEM_GOVERNOR_STATE:

        case TELEM_PROFILES:
        case TELEM_PID_PROFILE:
        case TELEM_RATES_PROFILE:
        case TELEM_BATTERY_PROFILE:
        case TELEM_LED_PROFILE:

        case TELEM_ADJFUNC:
            return true;

        case TELEM_SENSOR_COUNT:
            return false;
    }

    return false;
}


/** Legacy sensors **/

static uint32_t telemetry_legacy_sensors = 0;


sensor_e telemetryGetLegacySensor(sensor_id_e sensor_id)
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
        case TELEM_GPS_COORD:
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
            telemetry_legacy_sensors |= telemetryGetLegacySensor(id);
        }
    }
}

#endif /* USE_TELEMETRY */
