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

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#if defined(USE_ESC_SENSOR)

#include "config/feature.h"
#include "config/config.h"

#include "build/debug.h"

#include "blackbox/blackbox.h"

#include "common/time.h"
#include "common/crc.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/motor.h"

#include "common/maths.h"
#include "common/utils.h"

#include "drivers/timer.h"
#include "drivers/motor.h"
#include "drivers/dshot.h"
#include "drivers/dshot_dpwm.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"

#include "flight/mixer.h"

#include "io/serial.h"

#include "esc_sensor.h"



PG_REGISTER_WITH_RESET_TEMPLATE(escSensorConfig_t, escSensorConfig, PG_ESC_SENSOR_CONFIG, 0);

PG_RESET_TEMPLATE(escSensorConfig_t, escSensorConfig,
        .protocol = ESC_SENSOR_PROTO_NONE,
        .halfDuplex = 0,
        .update_hz = ESC_SENSOR_TASK_FREQ_HZ,
        .current_offset = 0,
        .hw4_current_offset = 15,
        .hw4_current_gain = 100,
        .hw4_voltage_gain = 110,
);


enum {
    DEBUG_ESC_1_RPM = 0,
    DEBUG_ESC_1_TEMP,
    DEBUG_ESC_1_VOLTAGE,
    DEBUG_ESC_1_CURRENT,
    DEBUG_ESC_2_RPM,
    DEBUG_ESC_2_TEMP,
    DEBUG_ESC_2_VOLTAGE,
    DEBUG_ESC_2_CURRENT,
};

enum {
    DEBUG_DATA_RPM = 0,
    DEBUG_DATA_PWM,
    DEBUG_DATA_TEMP,
    DEBUG_DATA_VOLTAGE,
    DEBUG_DATA_CURRENT,
    DEBUG_DATA_CAPACITY,
    DEBUG_DATA_EXTRA,
    DEBUG_DATA_AGE,
};

enum {
    DEBUG_FRAME_BYTE_COUNT = 0,
    DEBUG_FRAME_FRAME_COUNT,
    DEBUG_FRAME_SYNC_COUNT,
    DEBUG_FRAME_SYNC_ERRORS,
    DEBUG_FRAME_CRC_ERRORS,
    DEBUG_FRAME_TIMEOUTS,
    DEBUG_FRAME_BUFFER,
};

#define TELEMETRY_BUFFER_SIZE    40

static serialPort_t *escSensorPort = NULL;

static escSensorData_t escSensorData[MAX_SUPPORTED_MOTORS];
static escSensorData_t escSensorDataCombined;
static bool combinedNeedsUpdate = true;

static timeUs_t dataUpdateUs = 0;

static uint32_t totalByteCount = 0;
static uint32_t totalFrameCount = 0;
static uint32_t totalTimeoutCount = 0;
static uint32_t totalCrcErrorCount = 0;
static uint32_t totalSyncErrorCount = 0;

static uint8_t buffer[TELEMETRY_BUFFER_SIZE] = { 0, };

static uint8_t *bufferPtr = NULL;

static volatile uint8_t bufferSize = 0;
static volatile uint8_t bufferPos = 0;

static uint8_t  readBytes = 0;
static uint32_t syncCount = 0;


uint8_t getNumberEscBytesRead(void)
{
    return bufferPos;
}

bool isEscSensorActive(void)
{
    return escSensorPort != NULL;
}

uint16_t getEscSensorRPM(uint8_t motorNumber)
{
    return escSensorData[motorNumber].rpm;
}

static void combinedDataUpdate(void)
{
    const int motorCount = getMotorCount();

    if (combinedNeedsUpdate && motorCount > 0) {
        escSensorDataCombined.dataAge = 0;
        escSensorDataCombined.temperature = 0;
        escSensorDataCombined.voltage = 0;
        escSensorDataCombined.current = 0;
        escSensorDataCombined.consumption = 0;
        escSensorDataCombined.rpm = 0;

        for (int i = 0; i < motorCount; i++) {
            escSensorDataCombined.dataAge = MAX(escSensorDataCombined.dataAge, escSensorData[i].dataAge);
            escSensorDataCombined.temperature = MAX(escSensorDataCombined.temperature, escSensorData[i].temperature);
            escSensorDataCombined.voltage += escSensorData[i].voltage;
            escSensorDataCombined.current += escSensorData[i].current;
            escSensorDataCombined.consumption += escSensorData[i].consumption;
            escSensorDataCombined.rpm += escSensorData[i].rpm;
        }

        escSensorDataCombined.voltage = escSensorDataCombined.voltage / motorCount;
        escSensorDataCombined.rpm = escSensorDataCombined.rpm / motorCount;

        combinedNeedsUpdate = false;
    }
}

escSensorData_t * getEscSensorData(uint8_t motorNumber)
{
    if (featureIsEnabled(FEATURE_ESC_SENSOR)) {
        if (escSensorConfig()->protocol == ESC_SENSOR_PROTO_KISS) {
            if (motorNumber < getMotorCount()) {
                return &escSensorData[motorNumber];
            }
            else if (motorNumber == ESC_SENSOR_COMBINED) {
                combinedDataUpdate();
                return &escSensorDataCombined;
            }
        }
        else if (escSensorConfig()->protocol == ESC_SENSOR_PROTO_HW4 ||
                 escSensorConfig()->protocol == ESC_SENSOR_PROTO_HW5 ||
                 escSensorConfig()->protocol == ESC_SENSOR_PROTO_SCORPION ||
                 escSensorConfig()->protocol == ESC_SENSOR_PROTO_KONTRONIK ||
                 escSensorConfig()->protocol == ESC_SENSOR_PROTO_OMPHOBBY ||
                 escSensorConfig()->protocol == ESC_SENSOR_PROTO_ZTW ||
                 escSensorConfig()->protocol == ESC_SENSOR_PROTO_APD) {
            if (motorNumber == 0 || motorNumber == ESC_SENSOR_COMBINED)
                return &escSensorData[0];
        }
    }

    return NULL;
}

static FAST_CODE void escSensorDataReceive(uint16_t c, void *data)
{
    UNUSED(data);

    totalByteCount++;

    if (bufferPos < bufferSize) {
        bufferPtr[bufferPos++] = c;
    }
}


/*
 * KISS ESC TELEMETRY PROTOCOL
 * ---------------------------
 *
 * One packet is ten 8-bit bytes sent with 115200 bps.
 *
 * Byte 0:      Temperature
 * Byte 1,2:    Voltage
 * Byte 3,4:    Current
 * Byte 5,6:    Consumption
 * Byte 7,8:    RPM
 * Byte 9:      CRC8
 *
 */

#define KISS_ESC_BOOTTIME    5000            // 5 seconds
#define KISS_REQ_TIMEOUT     100             // 100 ms (data transfer takes only 900us)
#define ESC_FRAME_SIZE       10

enum {
    ESC_FRAME_PENDING = 0,
    ESC_FRAME_COMPLETE = 1,
    ESC_FRAME_FAILED = 2,
};

enum {
    ESC_TRIGGER_STARTUP = 0,
    ESC_TRIGGER_PENDING = 1,
};

static uint32_t escTriggerTimestamp;
static uint8_t escTriggerState = ESC_TRIGGER_STARTUP;

static uint8_t currentEsc = 0;


void startEscDataRead(uint8_t *frameBuffer, uint8_t frameLength)
{
    bufferPos = 0;
    bufferPtr = frameBuffer;
    bufferSize = frameLength;
}

static void increaseDataAge(void)
{
    if (escSensorData[currentEsc].dataAge < ESC_DATA_INVALID) {
        escSensorData[currentEsc].dataAge++;
        combinedNeedsUpdate = true;
    }

    DEBUG_AXIS(ESC_SENSOR_DATA, currentEsc, DEBUG_DATA_AGE, escSensorData[currentEsc].dataAge);
}

static void selectNextMotor(void)
{
    if (++currentEsc >= getMotorCount())
        currentEsc = 0;
}

static void setTelemetryReqeust(timeMs_t currentTimeMs)
{
    startEscDataRead(buffer, ESC_FRAME_SIZE);
    getMotorDmaOutput(currentEsc)->protocolControl.requestTelemetry = true;

    escTriggerState = ESC_TRIGGER_PENDING;
    escTriggerTimestamp = currentTimeMs;
}

static uint8_t decodeTelemetryFrame(void)
{
    // First, check the variables that can change in the interrupt
    if (bufferPos < bufferSize)
        return ESC_FRAME_PENDING;

    // Verify CRC8 checksum
    uint16_t chksum = crc8_kiss_update(0, buffer, ESC_FRAME_SIZE - 1);
    uint16_t tlmsum = buffer[ESC_FRAME_SIZE - 1];

    if (chksum == tlmsum) {
        uint16_t temp = buffer[0];
        uint16_t volt = buffer[1] << 8 | buffer[2];
        uint16_t curr = buffer[3] << 8 | buffer[4];
        uint16_t capa = buffer[5] << 8 | buffer[6];
        uint16_t erpm = buffer[7] << 8 | buffer[8];

        escSensorData[currentEsc].dataAge = 0;
        escSensorData[currentEsc].temperature = temp;
        escSensorData[currentEsc].voltage = volt;
        escSensorData[currentEsc].current = curr;
        escSensorData[currentEsc].consumption = capa;
        escSensorData[currentEsc].rpm = erpm;

        combinedNeedsUpdate = true;

        totalFrameCount++;

        if (currentEsc == 0) {
            DEBUG(ESC_SENSOR, DEBUG_ESC_1_RPM, erpm * 100);
            DEBUG(ESC_SENSOR, DEBUG_ESC_1_TEMP, temp * 10);
            DEBUG(ESC_SENSOR, DEBUG_ESC_1_VOLTAGE, volt);
            DEBUG(ESC_SENSOR, DEBUG_ESC_1_CURRENT, curr);
        }
        else if (currentEsc == 1) {
            DEBUG(ESC_SENSOR, DEBUG_ESC_2_RPM, erpm * 100);
            DEBUG(ESC_SENSOR, DEBUG_ESC_2_TEMP, temp * 10);
            DEBUG(ESC_SENSOR, DEBUG_ESC_2_VOLTAGE, volt);
            DEBUG(ESC_SENSOR, DEBUG_ESC_2_CURRENT, curr);
        }

        if (currentEsc == debugAxis) {
            DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_RPM, erpm);
            DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_TEMP, temp);
            DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_VOLTAGE, volt);
            DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_CURRENT, curr);
            DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_CAPACITY, capa);
            DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_AGE, 0);
        }

        return ESC_FRAME_COMPLETE;
    }

    totalCrcErrorCount++;

    return ESC_FRAME_FAILED;
}

static void kissSensorProcess(timeUs_t currentTimeUs)
{
    const timeMs_t currentTimeMs = currentTimeUs / 1000;

    switch (escTriggerState) {
        case ESC_TRIGGER_STARTUP:
            if (currentTimeMs >= KISS_ESC_BOOTTIME) {
                setTelemetryReqeust(currentTimeMs);
            }
            break;

        case ESC_TRIGGER_PENDING:
            if (currentTimeMs < escTriggerTimestamp + KISS_REQ_TIMEOUT) {
                uint8_t state = decodeTelemetryFrame();
                switch (state) {
                    case ESC_FRAME_COMPLETE:
                        selectNextMotor();
                        setTelemetryReqeust(currentTimeMs);
                        break;
                    case ESC_FRAME_FAILED:
                        increaseDataAge();
                        selectNextMotor();
                        setTelemetryReqeust(currentTimeMs);
                        break;
                    case ESC_FRAME_PENDING:
                        break;
                }
            }
            else {
                increaseDataAge();
                selectNextMotor();
                setTelemetryReqeust(currentTimeMs);

                totalTimeoutCount++;
            }
            break;
    }
}


/*
 * Calculate temperature from an NTC resistor divider ADC reading
 *
 * Let
 *     Rᵣ = Reference R
 *     Rₙ = NTC nominal R
 *     Tₙ = NTC nominal temp (25°C)
 *     Tₖ = 0°C in Kelvin = 273.15K
 *     β  = NTC beta
 *
 * and
 *
 *     T₁ = Tₖ + Tₙ = 298.15K
 *
 * Then
 *
 *          x         Rᵣ
 *  R = ―――――――――― ⋅ ――――
 *       4096 - x     Rₙ
 *
 *
 *             1
 *  T = ――――――――――――――― - Tₖ
 *       ln(R)/β + 1/T₁
 *
 *
 * Simplify:
 *
 *            1
 *  T = ―――――――――――――― - Tₖ
 *        γ⋅ln(S) + δ
 *
 * Where
 *          x
 *  S = ――――――――――
 *       4096 - x
 *
 *  γ = 1 / β
 *
 *  δ = γ⋅ln(Rᵣ/Rₙ) + 1/T₁
 *
 */

static float calcTempNTC(uint16_t adc, float gamma, float delta)
{
    const float X = constrainf(adc, 1, 4095);
    const float R = X / (4096 - X);
    const float A = logf(R) * gamma + delta;
    const float T = 1 / A - 273.15f;

    return T;
}


/*
 * Hobbywing V4 telemetry
 *
 * Credit to:       https://github.com/dgatf/msrc/
 *
 * Byte 0:          Sync 0x9B
 * Byte 1,2,3:      Packet counter
 * Byte 4,5:        Throttle
 * Byte 6,7:        PWM
 * Byte 8,9,10:     RPM
 * Byte 11,12:      Voltage
 * Byte 13,14:      Current
 * Byte 15,16:      Temperature (FETs)
 * Byte 17,18:      Temperature (BEC)
 *
 *
 * Voltage Gain:
 *   3-6S  (LV):    gain = 110
 *   3-8S  (LVv2):  gain = 154
 *   5-12s (HV):    gain = 210
 *
 * Current Gain:
 *   60A:           gain = 60
 *   80A:           gain = 78
 *   100A:          gain = 90
 *   120A:          gain = 100
 *   130A:          gain = 113
 *   150A:          gain = 129
 *   160A:          gain = 137
 *   200A:          gain = 169
 *
 */

/*
 * HW v4 Temp sensor design:
 *
 *  β  = 3950
 *  Tₙ = 25°C
 *  Rᵣ = 10k
 *  Rₙ = 47k
 *
 *  γ = 1 / β = 0.0002531…
 *
 *  δ = γ⋅ln(Rᵣ/Rₙ) + 1/T₁ = γ⋅ln(10/47) + 1/298.15 = 0.002962…
 */

#define HW4_GAMMA   0.00025316455696f
#define HW4_DELTA   0.00296226896087f

#define calcTempHW(adc)  calcTempNTC(adc, HW4_GAMMA, HW4_DELTA)

#define HW4_V_REF            3.3f
#define HW4_CURRENT_SHUNT    250e-6f
#define HW4_ADC_RESOLUTION   4096

static float calcVoltHW(uint16_t voltRaw)
{
    return voltRaw * (HW4_V_REF / HW4_ADC_RESOLUTION) * (escSensorConfig()->hw4_voltage_gain / 10.0f);
}

static float calcCurrHW(uint16_t currentRaw)
{
    if (currentRaw > escSensorConfig()->hw4_current_offset) {
        return (currentRaw - escSensorConfig()->hw4_current_offset) *
            (HW4_V_REF / (HW4_ADC_RESOLUTION * HW4_CURRENT_SHUNT * escSensorConfig()->hw4_current_gain / 10.0f));
    }

    return 0;
}

static void frameSyncError(void)
{
    readBytes = 0;
    syncCount = 0;

    totalSyncErrorCount++;
}

static void frameTimeoutError(void)
{
    readBytes = 0;
    syncCount = 0;

    totalTimeoutCount++;
}

static bool processHW4TelemetryStream(uint8_t dataByte)
{
    totalByteCount++;

    buffer[readBytes++] = dataByte;

    if (readBytes == 1) {
        if (dataByte == 0x9B) {
            syncCount++;
        }
        else if (dataByte == 0xB9) {
            readBytes = 0;
            syncCount++;
        }
        else {
            frameSyncError();
        }
    }
    else if (readBytes == 12) {
        if (buffer[1] == 0x9B) {
            readBytes = 0;
        }
    }
    else if (readBytes == 19) {
        readBytes = 0;
        return true;
    }

    return false;
}

static void hw4SensorProcess(timeUs_t currentTimeUs)
{
    // check for any available bytes in the rx buffer
    while (serialRxBytesWaiting(escSensorPort)) {
        if (processHW4TelemetryStream(serialRead(escSensorPort))) {
            if (buffer[4] < 4 && buffer[6] < 4 && buffer[8] < 4 &&
                buffer[11] < 0x10 && buffer[13] < 0x10 && buffer[15] < 0x10 && buffer[17] < 0x10) {

                //uint32_t cnt = buffer[1] << 16 | buffer[2] << 8 | buffer[3];
                uint16_t thr = buffer[4] << 8 | buffer[5];
                uint16_t pwm = buffer[6] << 8 | buffer[7];
                uint32_t rpm = buffer[8] << 16 | buffer[9] << 8 | buffer[10];

                float voltage = calcVoltHW(buffer[11] << 8 | buffer[12]);
                float current = calcCurrHW(buffer[13] << 8 | buffer[14]);
                float tempFET = calcTempHW(buffer[15] << 8 | buffer[16]);
                float tempBEC = calcTempHW(buffer[17] << 8 | buffer[18]);

                escSensorData[0].dataAge = 0;
                escSensorData[0].temperature = lrintf(tempFET);
                escSensorData[0].voltage = lrintf(voltage * 100);
                escSensorData[0].current = lrintf(current * 100);
                escSensorData[0].rpm = rpm / 100;

                // Hobbywing reports the last current reading when the motor stops.
                // That's completely useless, so set it to zero.
                if (rpm < 100 || thr < 50) {
                    escSensorData[0].current = 0.0f;
                }

                DEBUG(ESC_SENSOR, DEBUG_ESC_1_RPM, rpm);
                DEBUG(ESC_SENSOR, DEBUG_ESC_1_TEMP, lrintf(tempFET * 10));
                DEBUG(ESC_SENSOR, DEBUG_ESC_1_VOLTAGE, lrintf(voltage * 100));
                DEBUG(ESC_SENSOR, DEBUG_ESC_1_CURRENT, lrintf(current * 100));

                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_RPM, rpm);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_PWM, pwm);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_TEMP, lrintf(tempFET * 10));
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_VOLTAGE, lrintf(voltage * 100));
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_CURRENT, lrintf(current * 100));
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_EXTRA, lrintf(tempBEC * 10));
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_AGE, 0);

                dataUpdateUs = currentTimeUs;

                totalFrameCount++;
            }
            else {
                totalCrcErrorCount++;
            }
        }
    }

    // Increment data age counter if no updates in 500ms
    if (cmp32(currentTimeUs, dataUpdateUs) > 500000) {
        increaseDataAge();
        frameTimeoutError();
        dataUpdateUs = currentTimeUs;
    }
}


/*
 * Hobbywing V5 Telemetry
 *
 *    - Serial protocol 115200,8N1
 *    - Little-Endian fields
 *    - Frame length over data (23)
 *    - CRC16-MODBUS (poly 0x8005, init 0xffff)
 *    - Fault code bits:
 *         0:  Motor locked protection
 *         1:  Over-temp protection
 *         2:  Input throttle error at startup
 *         3:  Throttle signal lost
 *         4:  Over-current error
 *         5:  Low-voltage error
 *         6:  Input-voltage error
 *         7:  Motor connection error
 *
 * Byte 0-5:        Sync header 0xFE 0x01 0x00 0x03 0x30 0x5C
 * Byte 6:          Data frame length (23)
 * Byte 7-8:        Data type 0x06 0x00
 * Byte 9:          Throttle value in %
 * Byte 10-11:      Unknown
 * Byte 12:         Fault code
 * Byte 13-14:      RPM in 10rpm steps
 * Byte 15-16:      Voltage in 0.1V
 * Byte 17-18:      Current in 0.1A
 * Byte 19:         ESC Temperature in °C
 * Byte 20:         BEC Temperature in °C
 * Byte 21:         Motor Temperature in °C
 * Byte 22:         BEC Voltage in 0.1V
 * Byte 23:         BEC Current in 0.1A
 * Byte 24-29:      Unused 0xFF
 * Byte 30-31:      CRC16 MODBUS
 *
 */

static uint16_t calculateCRC16_MODBUS(const uint8_t *ptr, size_t len)
{
    uint16_t crc = ~0;

    while (len--) {
        crc ^= *ptr++;
        for (int i = 0; i < 8; i++)
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
    }

    return crc;
}

static bool processHW5TelemetryStream(uint8_t dataByte)
{
    totalByteCount++;

    buffer[readBytes++] = dataByte;

    if (readBytes == 1) {
        if (dataByte != 0xFE)
            frameSyncError();
    }
    else if (readBytes == 2) {
        if (dataByte != 0x01)
            frameSyncError();
    }
    else if (readBytes == 3) {
        if (dataByte != 0x00)
            frameSyncError();
    }
    else if (readBytes == 4) {
        if (dataByte != 0x03)
            frameSyncError();
    }
    else if (readBytes == 5) {
        if (dataByte != 0x30)
            frameSyncError();
    }
    else if (readBytes == 6) {
        if (dataByte != 0x5C)
            frameSyncError();
    }
    else if (readBytes == 7) {
        if (dataByte != 0x17)
            frameSyncError();
        else
            syncCount++;
    }
    else if (readBytes == 32) {
        readBytes = 0;
        if (syncCount > 3)
            return true;
    }

    return false;
}

static void hw5SensorProcess(timeUs_t currentTimeUs)
{
    // check for any available bytes in the rx buffer
    while (serialRxBytesWaiting(escSensorPort)) {
        if (processHW5TelemetryStream(serialRead(escSensorPort))) {
            uint16_t crc = buffer[31] << 8 | buffer[30];

            if (calculateCRC16_MODBUS(buffer, 30) == crc) {
                uint32_t rpm = buffer[14] << 8 | buffer[13];
                uint16_t power = buffer[9];
                uint16_t voltage = buffer[16] << 8 | buffer[15];
                uint16_t current = buffer[18] << 8 | buffer[17];
                uint16_t tempFET = buffer[19];
                uint16_t tempBEC = buffer[20];

                escSensorData[0].dataAge = 0;
                escSensorData[0].temperature = tempFET;
                escSensorData[0].voltage = voltage * 10;
                escSensorData[0].current = current * 10;
                escSensorData[0].rpm = rpm / 10;
                escSensorData[0].consumption = 0;

                DEBUG(ESC_SENSOR, DEBUG_ESC_1_RPM, rpm * 10);
                DEBUG(ESC_SENSOR, DEBUG_ESC_1_TEMP, tempFET * 10);
                DEBUG(ESC_SENSOR, DEBUG_ESC_1_VOLTAGE, voltage * 10);
                DEBUG(ESC_SENSOR, DEBUG_ESC_1_CURRENT, current * 10);

                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_RPM, rpm);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_PWM, power);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_TEMP, tempFET);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_VOLTAGE, voltage);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_CURRENT, current);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_CAPACITY, 0);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_EXTRA, tempBEC);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_AGE, 0);

                dataUpdateUs = currentTimeUs;

                totalFrameCount++;
            }
            else {
                totalCrcErrorCount++;
            }
        }
    }

    // Increment data age counter if no updates in 250ms
    if (cmp32(currentTimeUs, dataUpdateUs) > 250000) {
        increaseDataAge();
        frameTimeoutError();
        dataUpdateUs = currentTimeUs;
    }
}


/*
 * Kontronik Telemetry V4
 *
 *    - Serial protocol is 115200,8E1
 *    - Little-Endian fields
 *    - CRC32
 *    - Error flags:
 *         0:  Undervoltage on the battery
 *         1:  Overvoltage on the battery
 *         2:  Overcurrent error
 *         3:  Overcurrent warning
 *         4:  Temperature warning
 *         5:  Temperature error
 *         6:  BEC under-voltage error
 *         7:  BEC over-voltage error
 *         8:  BEC over-current error
 *         9:  BEC temperature error
 *         10: Switch-off by rudder movement
 *         11: Capacity limit reached
 *         12: Operational error
 *         13: Operational warning
 *         14: Self-test error
 *         15: EEPROM error
 *         16: Watchdog error
 *         17: Programming is still permitted
 *         18: Battery limit reached
 *         19: Current limit reached
 *         20: ESC temperature limit reached
 *         21: BEC temperature limit reached
 *         22: ESC current limit reached
 *         23: Capacity limit reached
 *
 * Byte 0-3:        Sync 0x4B 0x4F 0x44 0x4C "KODL"
 * Byte 4-7:        RPM
 * Byte 8-9:        Battery voltage in 10mV
 * Byte 10-11:      Battery current in 0.1A
 * Byte 12-13:      Motor current average in 0.1A
 * Byte 14-15:      Motor current peak in 0.1A
 * Byte 16-17:      Capacity in mAh
 * Byte 18-19:      BEC current in mA
 * Byte 20-21:      BEC Voltage n mV
 * Byte 22-23:      PWM in us
 * Byte 24:         Throttle % (-100..100)
 * Byte 25:         Output throttle 0..100%
 * Byte 26:         FET temperature -128..127°C
 * Byte 27:         BEC temperature -128..127°C
 * Byte 28-31:      Error Flags
 * Byte 32:         Operational condition
 * Byte 33:         Timing 0..30
 * Byte 34-37:      CRC32
 *
 */

static uint32_t calculateCRC32(const uint8_t *ptr, size_t len)
{
    uint32_t crc = 0xFFFFFFFF;

    while (len--) {
        crc ^= *ptr++;
        for (int i = 0; i < 8; i++)
            crc = (crc & 1) ? (crc >> 1) ^ 0xEDB88320 : (crc >> 1);
    }

    return ~crc;
}

static bool processKontronikTelemetryStream(uint8_t dataByte)
{
    totalByteCount++;

    buffer[readBytes++] = dataByte;

    if (readBytes == 1) {
        if (dataByte != 0x4B)
            frameSyncError();
    }
    else if (readBytes == 2) {
        if (dataByte != 0x4F)
            frameSyncError();
    }
    else if (readBytes == 3) {
        if (dataByte != 0x44)
            frameSyncError();
    }
    else if (readBytes == 4) {
        if (dataByte != 0x4C)
            frameSyncError();
        else
            syncCount++;
    }
    else if (readBytes == 38) {
        readBytes = 0;
        if (syncCount > 3)
            return true;
    }

    return false;
}

static void kontronikSensorProcess(timeUs_t currentTimeUs)
{
    // check for any available bytes in the rx buffer
    while (serialRxBytesWaiting(escSensorPort)) {
        if (processKontronikTelemetryStream(serialRead(escSensorPort))) {
            uint32_t crc = buffer[37] << 24 | buffer[36] << 16 | buffer[35] << 8 | buffer[34];

            if (calculateCRC32(buffer, 34) == crc) {
                uint32_t rpm = buffer[7] << 24 | buffer[6] << 16 | buffer[5] << 8 | buffer[4];
                uint16_t pwm = buffer[23] << 8 | buffer[22];
                uint16_t voltage = buffer[9] << 8 | buffer[8];
                uint16_t current = buffer[11] << 8 | buffer[10];
                uint16_t capacity = buffer[17] << 8 | buffer[16];
                uint16_t tempFET = buffer[26];
                uint16_t tempBEC = buffer[27];

                escSensorData[0].dataAge = 0;
                escSensorData[0].temperature = tempFET;
                escSensorData[0].voltage = voltage;
                escSensorData[0].current = current;
                escSensorData[0].rpm = rpm / 100;
                escSensorData[0].consumption = capacity;

                DEBUG(ESC_SENSOR, DEBUG_ESC_1_RPM, rpm);
                DEBUG(ESC_SENSOR, DEBUG_ESC_1_TEMP, tempFET * 10);
                DEBUG(ESC_SENSOR, DEBUG_ESC_1_VOLTAGE, voltage);
                DEBUG(ESC_SENSOR, DEBUG_ESC_1_CURRENT, current);

                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_RPM, rpm);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_PWM, pwm);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_TEMP, tempFET);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_VOLTAGE, voltage);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_CURRENT, current);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_CAPACITY, capacity);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_EXTRA, tempBEC);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_AGE, 0);

                dataUpdateUs = currentTimeUs;

                totalFrameCount++;
            }
            else {
                totalCrcErrorCount++;
            }
        }
    }

    // Increment data age counter if no updates in 250ms
    if (cmp32(currentTimeUs, dataUpdateUs) > 250000) {
        increaseDataAge();
        frameTimeoutError();
        dataUpdateUs = currentTimeUs;
    }
}


/*
 * OMP Hobby M4 Telemetry
 *
 *    - Serial protocol is 115200,8N1
 *    - Frame length includes header and CRC
 *    - Big-Endian fields
 *    - Status Code bits:
 *         0:  Short-circuit protection
 *         1:  Motor connection error
 *         2:  Throttle signal lost
 *         3:  Throttle signal >0 on startup error
 *         4:  Low voltage protection
 *         5:  Temperature protection
 *         6:  Startup protection
 *         7:  Current protection
 *         8:  Throttle signal error
 *        12:  Battery voltage error
 *
 * Byte 0:          Start Flag 0xdd
 * Byte 1:          Protocol version 0x01
 * Byte 2:          Frame lenght (32 for v1)
 * Byte 3-4:        Battery voltage in 0.1V
 * Byte 5-6:        Battery current in 0.1V
 * Byte 7:          Input Throttle in %
 * Byte 8-9:        RPM in 10rpm steps
 * Byte 10:         ESC Temperature
 * Byte 11:         Motor Temperature
 * Byte 12:         PWM Throttle in %
 * Byte 13-14:      Status Code
 * Byte 15-16:      Capacity mAh
 * Byte 17-31:      Unused / Zeros
 *
 */

static bool processOMPTelemetryStream(uint8_t dataByte)
{
    totalByteCount++;

    buffer[readBytes++] = dataByte;

    if (readBytes == 1) {
        if (dataByte != 0xDD)
            frameSyncError();
        else
            syncCount++;
    }
    else if (readBytes == 32) {
        readBytes = 0;
        if (syncCount > 3)
            return true;
    }

    return false;
}

static void ompSensorProcess(timeUs_t currentTimeUs)
{
    // check for any available bytes in the rx buffer
    while (serialRxBytesWaiting(escSensorPort)) {
        if (processOMPTelemetryStream(serialRead(escSensorPort))) {
            // Make sure this is OMP M4 ESC
            if (buffer[1] == 0x01 && buffer[2] == 0x20 && buffer[11] == 0 && buffer[18] == 0 && buffer[20] == 0) {
                uint16_t rpm = buffer[8] << 8 | buffer[9];
                uint16_t pwm = buffer[12];
                uint16_t temp = buffer[10];
                uint16_t voltage = buffer[3] << 8 | buffer[4];
                uint16_t current = buffer[5] << 8 | buffer[6];
                uint16_t capacity = buffer[15] << 8 | buffer[16];
                uint16_t status = buffer[13] << 8 | buffer[14];

                escSensorData[0].dataAge = 0;
                escSensorData[0].temperature = temp;
                escSensorData[0].voltage = voltage * 10;
                escSensorData[0].current = current * 10;
                escSensorData[0].rpm = rpm / 10;
                escSensorData[0].consumption = 0; // capacity; // FIXME bogus value

                DEBUG(ESC_SENSOR, DEBUG_ESC_1_RPM, rpm * 10);
                DEBUG(ESC_SENSOR, DEBUG_ESC_1_TEMP, temp * 10);
                DEBUG(ESC_SENSOR, DEBUG_ESC_1_VOLTAGE, voltage * 10);
                DEBUG(ESC_SENSOR, DEBUG_ESC_1_CURRENT, current * 10);

                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_RPM, rpm);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_PWM, pwm);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_TEMP, temp);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_VOLTAGE, voltage);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_CURRENT, current);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_CAPACITY, capacity);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_EXTRA, status);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_AGE, 0);

                dataUpdateUs = currentTimeUs;

                totalFrameCount++;
            }
            else {
                totalCrcErrorCount++;
            }
        }
    }

    // Increment data age counter if no updates in 250ms
    if (cmp32(currentTimeUs, dataUpdateUs) > 250000) {
        increaseDataAge();
        frameTimeoutError();
        dataUpdateUs = currentTimeUs;
    }
}


/*
 * ZTW Telemetry
 *
 *    - Serial protocol is 115200,8N1
 *    - Frame length includes header and CRC
 *    - Big-Endian fields
 *    - Checksum (unknown)
 *    - Status Code bits:
 *         0:  Short-circuit protection
 *         1:  Motor connection error
 *         2:  Throttle signal lost
 *         3:  Throttle signal >0 on startup error
 *         4:  Low voltage protection
 *         5:  Temperature protection
 *         6:  Startup protection
 *         7:  Current protection
 *         8:  Throttle signal error
 *         9:  UART throttle error
 *        10:  UART throttle lost
 *        11:  CAN throttle lost
 *        12:  Battery voltage error
 *
 * Byte 0:          Start Flag 0xdd
 * Byte 1:          Protocol version 0x01
 * Byte 2:          Frame lenght (32 for v1)
 * Byte 3-4:        Battery voltage in 0.1V
 * Byte 5-6:        Battery current in 0.1V
 * Byte 7:          Input Throttle in %
 * Byte 8-9:        RPM in 10rpm steps
 * Byte 10:         ESC Temperature
 * Byte 11:         Motor Temperature
 * Byte 12:         PWM Throttle in %
 * Byte 13-14:      Status Code
 * Byte 15-16:      Capacity mAh
 * Byte 17:         Serial Throttle input (unused)
 * Byte 18:         CAN Throttle input (unused)
 * Byte 19:         BEC Voltage
 * Byte 20-29:      Unused
 * Byte 30-31:      Checksum
 *
 */

static bool processZTWTelemetryStream(uint8_t dataByte)
{
    totalByteCount++;

    buffer[readBytes++] = dataByte;

    if (readBytes == 1) {
        if (dataByte != 0xDD)
            frameSyncError();
        else
            syncCount++;
    }
    else if (readBytes == 32) {
        readBytes = 0;
        if (syncCount > 3)
            return true;
    }

    return false;
}

static void ztwSensorProcess(timeUs_t currentTimeUs)
{
    // check for any available bytes in the rx buffer
    while (serialRxBytesWaiting(escSensorPort)) {
        if (processZTWTelemetryStream(serialRead(escSensorPort))) {
            if (buffer[1] == 0x01 && buffer[2] == 0x20) {
                uint16_t rpm = buffer[8] << 8 | buffer[9];
                uint16_t temp = buffer[10];
                uint16_t power = buffer[12];
                uint16_t voltage = buffer[3] << 8 | buffer[4];
                uint16_t current = buffer[5] << 8 | buffer[6];
                uint16_t capacity = buffer[15] << 8 | buffer[16];
                uint16_t status = buffer[13] << 8 | buffer[14];

                escSensorData[0].dataAge = 0;
                escSensorData[0].temperature = temp;
                escSensorData[0].voltage = voltage * 10;
                escSensorData[0].current = current * 10;
                escSensorData[0].rpm = rpm / 10;
                escSensorData[0].consumption = capacity;

                DEBUG(ESC_SENSOR, DEBUG_ESC_1_RPM, rpm * 10);
                DEBUG(ESC_SENSOR, DEBUG_ESC_1_TEMP, temp * 10);
                DEBUG(ESC_SENSOR, DEBUG_ESC_1_VOLTAGE, voltage * 10);
                DEBUG(ESC_SENSOR, DEBUG_ESC_1_CURRENT, current * 10);

                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_RPM, rpm);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_PWM, power);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_TEMP, temp);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_VOLTAGE, voltage);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_CURRENT, current);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_CAPACITY, capacity);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_EXTRA, status);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_AGE, 0);

                dataUpdateUs = currentTimeUs;

                totalFrameCount++;
            }
            else {
                totalCrcErrorCount++;
            }
        }
    }

    // Increment data age counter if no updates in 250ms
    if (cmp32(currentTimeUs, dataUpdateUs) > 250000) {
        increaseDataAge();
        frameTimeoutError();
        dataUpdateUs = currentTimeUs;
    }
}


/*
 * Scorpion Unsolicited Telemetry
 *
 *    - ESC must be set to "Unsolicited mode"
 *    - Serial protocol is 38400,8N1
 *    - Little-Endian fields
 *    - CRC16-CCITT
 *    - Error Code bits:
 *         0:  N/A
 *         1:  BEC voltage error
 *         2:  Temperature error
 *         3:  Consumption error
 *         4:  Input voltage error
 *         5:  Current error
 *         6:  N/A
 *         7:  Throttle error
 *
 * Byte 0:          Header Sync 0x55
 * Byte 1:          Message format version (0x00)
 * Byte 2:          Message Length incl. header and CRC (22)
 * Byte 3:          Device ID
 * Byte 4-6:        Timestamp ms
 * Byte 7:          Input throttle in 0.5%
 * Byte 8-9:        Current in 0.1A
 * Byte 10-11:      Voltage in 0.1V
 * Byte 12-13:      Consumption in mAh
 * Byte 14:         Temperature in °C
 * Byte 15:         Output Power in 0.5%
 * Byte 16:         BEC voltage in 0.1V
 * Byte 17-18:      RPM in 5rpm steps
 * Byte 19:         Error code
 * Byte 20-21:      CRC16 CCITT
 *
 */

static uint16_t calculateCRC16_CCITT(const uint8_t *ptr, size_t len)
{
    uint16_t crc = 0;

    while (len--) {
        crc ^= *ptr++;
        for (int i = 0; i < 8; i++)
            crc = (crc & 1) ? (crc >> 1) ^ 0x8408 : (crc >> 1);
    }

    return crc;
}

static bool processUNCTelemetryStream(uint8_t dataByte)
{
    totalByteCount++;

    buffer[readBytes++] = dataByte;

    if (readBytes == 1) {
        if (dataByte != 0x55)
            frameSyncError();
    }
    else if (readBytes == 2) {
        if (dataByte != 0x00)  // Proto v0
            frameSyncError();
    }
    else if (readBytes == 3) {
        if (dataByte != 22)  // Message v0 is 22 bytes
            frameSyncError();
        else
            syncCount++;
    }
    else if (readBytes == 22) {
        readBytes = 0;
        if (syncCount > 3)
            return true;
    }

    return false;
}

static void uncSensorProcess(timeUs_t currentTimeUs)
{
    // check for any available bytes in the rx buffer
    while (serialRxBytesWaiting(escSensorPort)) {
        if (processUNCTelemetryStream(serialRead(escSensorPort))) {
            uint16_t crc = buffer[21] << 8 | buffer[20];

            if (calculateCRC16_CCITT(buffer, 20) == crc) {
                uint16_t rpm = buffer[18] << 8 | buffer[17];
                uint16_t temp = buffer[14];
                uint16_t power = buffer[15];
                uint16_t voltage = buffer[11] << 8 | buffer[10];
                uint16_t current = buffer[9] << 8 | buffer[8];
                uint16_t capacity = buffer[13] << 8 | buffer[12];
                uint16_t status = buffer[19];

                escSensorData[0].dataAge = 0;
                escSensorData[0].temperature = temp;
                escSensorData[0].voltage = voltage * 10;
                escSensorData[0].current = current * 10;
                escSensorData[0].rpm = rpm / 20;
                escSensorData[0].consumption = capacity;

                DEBUG(ESC_SENSOR, DEBUG_ESC_1_RPM, rpm * 5);
                DEBUG(ESC_SENSOR, DEBUG_ESC_1_TEMP, temp * 10);
                DEBUG(ESC_SENSOR, DEBUG_ESC_1_VOLTAGE, voltage * 10);
                DEBUG(ESC_SENSOR, DEBUG_ESC_1_CURRENT, current * 10);

                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_RPM, rpm);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_PWM, power);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_TEMP, temp);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_VOLTAGE, voltage);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_CURRENT, current);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_CAPACITY, capacity);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_EXTRA, status);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_AGE, 0);

                dataUpdateUs = currentTimeUs;

                totalFrameCount++;
            }
            else {
                totalCrcErrorCount++;
            }
        }
    }

    // Increment data age counter if no updates in 500ms
    if (cmp32(currentTimeUs, dataUpdateUs) > 500000) {
        increaseDataAge();
        frameTimeoutError();
        dataUpdateUs = currentTimeUs;
    }
}


/*
 * Advanced Power Drives UART Telemetry
 *
 *    - Serial protocol is 115200,8N1
 *    - Little-Endian fields
 *    - Fletcher16 checksum
 *    - Status Flags:
 *         0:  Motor started
 *         1:  Motor saturation
 *         2:  Over-temperature
 *         3:  Over-voltage
 *         4:  Under-voltage
 *         5:  Startup error
 *         6:  Unused
 *         7:  Unused
 *
 * Byte 0-1:        Sync 0xFFFF
 * Byte 2-3:        Voltage in 10mV steps
 * Byte 4-5:        Temperature (raw ADC)
 * Byte 6-7:        Current in 80mA steps
 * Byte 8-9:        Unused
 * Byte 10-13:      ERPM
 * Byte 14-15:      Throttle in 0.1%
 * Byte 16-17:      Motor Duty Cycle in 0.1%
 * Byte 18:         Status flags
 * Byte 19:         Unused
 * Byte 20-21:      Checksum
 *
 */

/*
 *  β  = 3455
 *  Rᵣ = 10k
 *  Rₙ = 10k
 *  Tₙ = 25°C
 *
 *  γ = 1 / β = 0.0002894…
 *
 *  δ = γ⋅ln(Rᵣ/Rₙ) + 1/T₁ = 1/298.15 = 0.0033540…
 */
#define APD_GAMMA  0.0002894356005f
#define APD_DELTA  0.0033540164346f

#define calcTempAPD(value)   calcTempNTC(value, APD_GAMMA, APD_DELTA)

static uint16_t calculateFletcher16(const uint8_t *ptr, size_t len)
{
    uint16_t s0 = 0;
    uint16_t s1 = 0;

    while (len--) {
        s0 = (s0 + *ptr++) % 255;
        s1 = (s1 + s0) % 255;
    }
    return s1 << 8 | s0;
}

static bool processAPDTelemetryStream(uint8_t dataByte)
{
    totalByteCount++;

    buffer[readBytes++] = dataByte;

    if (readBytes == 1) {
        if (dataByte != 0xFF)
            frameSyncError();
    }
    else if (readBytes == 2) {
        if (dataByte != 0xFF)
            frameSyncError();
        else
            syncCount++;
    }
    else if (readBytes == 22) {
        readBytes = 0;
        if (syncCount > 3)
            return true;
    }

    return false;
}

static void apdSensorProcess(timeUs_t currentTimeUs)
{
    // check for any available bytes in the rx buffer
    while (serialRxBytesWaiting(escSensorPort)) {
        if (processAPDTelemetryStream(serialRead(escSensorPort))) {
            uint16_t crc = buffer[21] << 8 | buffer[20];

            if (calculateFletcher16(buffer + 2, 18) == crc) {
                uint16_t rpm = buffer[13] << 24 | buffer[12] << 16 | buffer[11] << 8 | buffer[10];
                uint16_t traw = buffer[3] << 8 | buffer[2];
                uint16_t power = buffer[17] << 8 | buffer[16];
                uint16_t voltage = buffer[1] << 8 | buffer[0];
                uint16_t current = buffer[5] << 8 | buffer[4];
                uint16_t status = buffer[18];

                float temp = calcTempAPD(traw);

                escSensorData[0].dataAge = 0;
                escSensorData[0].temperature = lrintf(temp);
                escSensorData[0].voltage = voltage;
                escSensorData[0].current = current * 8;
                escSensorData[0].rpm = rpm / 100;
                escSensorData[0].consumption = 0;

                DEBUG(ESC_SENSOR, DEBUG_ESC_1_RPM, rpm);
                DEBUG(ESC_SENSOR, DEBUG_ESC_1_TEMP, lrintf(temp * 10));
                DEBUG(ESC_SENSOR, DEBUG_ESC_1_VOLTAGE, voltage);
                DEBUG(ESC_SENSOR, DEBUG_ESC_1_CURRENT, current * 8);

                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_RPM, rpm);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_PWM, power);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_TEMP, traw);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_VOLTAGE, voltage);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_CURRENT, current);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_EXTRA, status);
                DEBUG(ESC_SENSOR_DATA, DEBUG_DATA_AGE, 0);

                dataUpdateUs = currentTimeUs;

                totalFrameCount++;
            }
            else {
                totalCrcErrorCount++;
            }
        }
    }

    // Increment data age counter if no updates in 500ms
    if (cmp32(currentTimeUs, dataUpdateUs) > 500000) {
        increaseDataAge();
        frameTimeoutError();
        dataUpdateUs = currentTimeUs;
    }
}


/*
 * Raw Telemetry Data Recorder
 */

static void recordSensorProcess(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    // check for any available bytes in the rx buffer
    while (serialRxBytesWaiting(escSensorPort) && readBytes < 32) {
        totalByteCount++;
        buffer[readBytes++] = serialRead(escSensorPort);
    }

    if (readBytes > 0) {
        blackboxLogCustomData(buffer, readBytes);
        totalFrameCount++;
        readBytes = 0;
    }
}


void escSensorProcess(timeUs_t currentTimeUs)
{
    if (escSensorPort && motorIsEnabled()) {
        switch (escSensorConfig()->protocol) {
            case ESC_SENSOR_PROTO_KISS:
                kissSensorProcess(currentTimeUs);
                break;
            case ESC_SENSOR_PROTO_HW4:
                hw4SensorProcess(currentTimeUs);
                break;
            case ESC_SENSOR_PROTO_HW5:
                hw5SensorProcess(currentTimeUs);
                break;
            case ESC_SENSOR_PROTO_SCORPION:
                uncSensorProcess(currentTimeUs);
                break;
            case ESC_SENSOR_PROTO_KONTRONIK:
                kontronikSensorProcess(currentTimeUs);
                break;
            case ESC_SENSOR_PROTO_OMPHOBBY:
                ompSensorProcess(currentTimeUs);
                break;
            case ESC_SENSOR_PROTO_ZTW:
                ztwSensorProcess(currentTimeUs);
                break;
            case ESC_SENSOR_PROTO_APD:
                apdSensorProcess(currentTimeUs);
                break;
            case ESC_SENSOR_PROTO_RECORD:
                recordSensorProcess(currentTimeUs);
                break;
        }

        DEBUG(ESC_SENSOR_FRAME, DEBUG_FRAME_BYTE_COUNT, totalByteCount);
        DEBUG(ESC_SENSOR_FRAME, DEBUG_FRAME_FRAME_COUNT, totalFrameCount);
        DEBUG(ESC_SENSOR_FRAME, DEBUG_FRAME_SYNC_COUNT, syncCount);
        DEBUG(ESC_SENSOR_FRAME, DEBUG_FRAME_SYNC_ERRORS, totalSyncErrorCount);
        DEBUG(ESC_SENSOR_FRAME, DEBUG_FRAME_CRC_ERRORS, totalCrcErrorCount);
        DEBUG(ESC_SENSOR_FRAME, DEBUG_FRAME_TIMEOUTS, totalTimeoutCount);
        DEBUG(ESC_SENSOR_FRAME, DEBUG_FRAME_BUFFER, readBytes);
    }
}

bool INIT_CODE escSensorInit(void)
{
    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_ESC_SENSOR);
    serialReceiveCallbackPtr callback = NULL;
    portOptions_e options = 0;
    uint32_t baudrate = 0;

    if (!portConfig) {
        return false;
    }

    options = SERIAL_STOPBITS_1 | SERIAL_PARITY_NO | SERIAL_NOT_INVERTED | (escSensorConfig()->halfDuplex ? SERIAL_BIDIR : 0);

    switch (escSensorConfig()->protocol) {
        case ESC_SENSOR_PROTO_KISS:
            callback = escSensorDataReceive;
            baudrate = 115200;
            break;
        case ESC_SENSOR_PROTO_HW4:
            baudrate = 19200;
            break;
        case ESC_SENSOR_PROTO_SCORPION:
            baudrate = 38400;
            break;
        case ESC_SENSOR_PROTO_KONTRONIK:
            baudrate = 115200;
            options |= SERIAL_PARITY_EVEN;
            break;
        case ESC_SENSOR_PROTO_OMPHOBBY:
        case ESC_SENSOR_PROTO_ZTW:
        case ESC_SENSOR_PROTO_HW5:
        case ESC_SENSOR_PROTO_APD:
            baudrate = 115200;
            break;
        case ESC_SENSOR_PROTO_RECORD:
            baudrate = baudRates[portConfig->telemetry_baudrateIndex];
            break;
    }

    if (baudrate) {
        escSensorPort = openSerialPort(portConfig->identifier, FUNCTION_ESC_SENSOR, callback, NULL, baudrate, MODE_RX, options);
    }

    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        escSensorData[i].dataAge = ESC_DATA_INVALID;
    }

    return (escSensorPort != NULL);
}

#endif
