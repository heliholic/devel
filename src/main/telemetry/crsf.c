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
#include <string.h>

#include "platform.h"

#ifdef USE_TELEMETRY_CRSF

#include "build/atomic.h"
#include "build/build_config.h"
#include "build/version.h"

#include "cms/cms.h"

#include "config/config.h"
#include "config/feature.h"

#include "common/crc.h"
#include "common/maths.h"
#include "common/printf.h"
#include "common/streambuf.h"
#include "common/time.h"
#include "common/utils.h"

#include "drivers/nvic.h"
#include "drivers/persistent.h"

#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/position.h"
#include "flight/governor.h"

#include "io/displayport_crsf.h"
#include "io/gps.h"
#include "io/serial.h"
#include "io/ledstrip.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "rx/crsf.h"
#include "rx/crsf_protocol.h"

#include "sensors/battery.h"
#include "sensors/sensors.h"

#include "telemetry/telemetry.h"
#include "telemetry/msp_shared.h"

#include "crsf.h"


#define CRSF_CYCLETIME_US                   10000

#define CRSF_DEVICEINFO_VERSION             0x01
#define CRSF_DEVICEINFO_PARAMETER_COUNT     0

#define CRSF_MSP_BUFFER_SIZE                96
#define CRSF_MSP_LENGTH_OFFSET              1

static bool crsfTelemetryEnabled;
static bool deviceInfoReplyPending;

static sbuf_t crsfSbuf;
static uint8_t crsfFrame[CRSF_FRAME_SIZE_MAX + 32];


#if defined(USE_CRSF_V3)

#define CRSF_TELEMETRY_FRAME_INTERVAL_MAX_US 20000      // 20ms

static bool isCrsfV3Running = false;

typedef struct {
    bool hasPendingReply;
    bool isNewSpeedValid;
    uint8_t portID:3;
    uint8_t index;
    uint32_t confirmationTime;
} crsfSpeedControl_t;

static crsfSpeedControl_t crsfSpeed = {0};

uint32_t getCrsfCachedBaudrate(void)
{
    uint32_t crsfCachedBaudrate = persistentObjectRead(PERSISTENT_OBJECT_SERIALRX_BAUD);
    // check if valid first. return default baudrate if not
    for (unsigned i = 0; i < BAUD_COUNT; i++) {
        if (crsfCachedBaudrate == baudRates[i] && baudRates[i] >= CRSF_BAUDRATE) {
            return crsfCachedBaudrate;
        }
    }
    return CRSF_BAUDRATE;
}

bool checkCrsfCustomizedSpeed(void)
{
    return crsfSpeed.index < BAUD_COUNT;
}

uint32_t getCrsfDesiredSpeed(void)
{
    return checkCrsfCustomizedSpeed() ? baudRates[crsfSpeed.index] : CRSF_BAUDRATE;
}

void setCrsfDefaultSpeed(void)
{
    crsfSpeed.hasPendingReply = false;
    crsfSpeed.isNewSpeedValid = false;
    crsfSpeed.confirmationTime = 0;
    crsfSpeed.index = BAUD_COUNT;
    isCrsfV3Running = false;
    crsfRxUpdateBaudrate(getCrsfDesiredSpeed());
}

bool crsfBaudNegotiationInProgress(void)
{
    return crsfSpeed.hasPendingReply || crsfSpeed.isNewSpeedValid;
}

#endif /* USE_CRSF_V3 */


#if defined(USE_MSP_OVER_TELEMETRY)

typedef struct mspBuffer_s {
    uint8_t bytes[CRSF_MSP_BUFFER_SIZE];
    int len;
} mspBuffer_t;

static mspBuffer_t mspRxBuffer;

void initCrsfMspBuffer(void)
{
    mspRxBuffer.len = 0;
}

bool bufferCrsfMspFrame(uint8_t *frameStart, int frameLength)
{
    if (mspRxBuffer.len + CRSF_MSP_LENGTH_OFFSET + frameLength > CRSF_MSP_BUFFER_SIZE) {
        return false;
    } else {
        uint8_t *p = mspRxBuffer.bytes + mspRxBuffer.len;
        *p++ = frameLength;
        memcpy(p, frameStart, frameLength);
        mspRxBuffer.len += CRSF_MSP_LENGTH_OFFSET + frameLength;
        return true;
    }
}

bool handleCrsfMspFrameBuffer(mspResponseFnPtr responseFn)
{
    static bool replyPending = false;

    if (replyPending) {
        if (crsfRxIsTelemetryBufEmpty()) {
            replyPending = sendMspReply(CRSF_FRAME_TX_MSP_FRAME_SIZE, responseFn);
        }
        return replyPending;
    }

    if (!mspRxBuffer.len) {
        return false;
    }

    int pos = 0;
    while (true) {
        const uint8_t mspFrameLength = mspRxBuffer.bytes[pos];
        if (handleMspFrame(&mspRxBuffer.bytes[CRSF_MSP_LENGTH_OFFSET + pos], mspFrameLength, NULL)) {
            if (crsfRxIsTelemetryBufEmpty()) {
                replyPending = sendMspReply(CRSF_FRAME_TX_MSP_FRAME_SIZE, responseFn);
            } else {
                replyPending = true;
            }
        }
        pos += CRSF_MSP_LENGTH_OFFSET + mspFrameLength;
        ATOMIC_BLOCK(NVIC_PRIO_SERIALUART1) {
            if (pos >= mspRxBuffer.len) {
                mspRxBuffer.len = 0;
                return replyPending;
            }
        }
    }

    return replyPending;
}
#endif /* USE_MSP_OVER_TELEMETRY */


static sbuf_t * crsfInitializeSbuf(void)
{
    sbuf_t * dst = &crsfSbuf;

    dst->ptr = crsfFrame;
    dst->end = crsfFrame + CRSF_FRAME_SIZE_MAX;

    sbufWriteU8(dst, CRSF_SYNC_BYTE);
    sbufWriteU8(dst, CRSF_FRAME_LENGTH_TYPE_CRC);  // placeholder

    return dst;
}

static void crsfFinalizeSbuf(sbuf_t *dst)
{
    // Frame length including CRC
    const size_t frameLength = sbufPtr(dst) - crsfFrame + 2;

    if (frameLength <= CRSF_FRAME_SIZE_MAX)
    {
        // Set frame length into the placeholder
        crsfFrame[1] = frameLength - 3;

        // CRC does not include device address and frame length
        crc8_dvb_s2_sbuf_append(dst, &crsfFrame[2]);

        // Write the telemetry frame to the receiver
        crsfRxWriteTelemetryData(crsfFrame, frameLength);
    }
}

/*
CRSF frame has the structure:
<Device address> <Frame length> <Type> <Payload> <CRC>
Device address: (uint8_t)
Frame length:   length in  bytes including Type (uint8_t)
Type:           (uint8_t)
CRC:            (uint8_t), crc of <Type> and <Payload>
*/

/*
0x02 GPS
Payload:
int32_t     Latitude ( degree / 10`000`000 )
int32_t     Longitude (degree / 10`000`000 )
uint16_t    Groundspeed ( km/h / 10 )
uint16_t    GPS heading ( degree / 100 )
uint16      Altitude ( meter ­1000m offset )
uint8_t     Satellites in use ( counter )
*/

void crsfFrameGps(sbuf_t *dst)
{
    sbufWriteU8(dst, CRSF_FRAMETYPE_GPS);
    sbufWriteS32BE(dst, gpsSol.llh.lat);
    sbufWriteS32BE(dst, gpsSol.llh.lon);
    sbufWriteU16BE(dst, (gpsSol.groundSpeed * 36 + 50) / 100); // cm/s
    sbufWriteU16BE(dst, gpsSol.groundCourse * 10); // degrees * 10
    sbufWriteU16BE(dst, getEstimatedAltitudeCm() / 100 + 1000);
    sbufWriteU8(dst, gpsSol.numSat);
}

/*
0x08 Battery sensor
Payload:
uint16_t    Voltage (100mV steps)
uint16_t    Current (100mA steps)
uint24_t    Fuel (drawn mAh)
uint8_t     Battery remaining (percent)
*/
void crsfFrameBatterySensor(sbuf_t *dst)
{
    sbufWriteU8(dst, CRSF_FRAMETYPE_BATTERY_SENSOR);
    sbufWriteU16BE(dst, getLegacyBatteryVoltage());
    sbufWriteU16BE(dst, getLegacyBatteryCurrent());
    sbufWriteU24BE(dst, getBatteryCapacityUsed());
    sbufWriteU8(dst, calculateBatteryPercentageRemaining());
}

/*
0x0B Heartbeat
Payload:
int16_t    Origin Device address
*/
void crsfFrameHeartbeat(sbuf_t *dst)
{
    sbufWriteU8(dst, CRSF_FRAMETYPE_HEARTBEAT);
    sbufWriteU16BE(dst, CRSF_ADDRESS_FLIGHT_CONTROLLER);
}

typedef enum {
    CRSF_ACTIVE_ANTENNA1 = 0,
    CRSF_ACTIVE_ANTENNA2 = 1
} crsfActiveAntenna_e;

typedef enum {
    CRSF_RF_MODE_4_HZ = 0,
    CRSF_RF_MODE_50_HZ = 1,
    CRSF_RF_MODE_150_HZ = 2
} crsrRfMode_e;

typedef enum {
    CRSF_RF_POWER_0_mW = 0,
    CRSF_RF_POWER_10_mW = 1,
    CRSF_RF_POWER_25_mW = 2,
    CRSF_RF_POWER_100_mW = 3,
    CRSF_RF_POWER_500_mW = 4,
    CRSF_RF_POWER_1000_mW = 5,
    CRSF_RF_POWER_2000_mW = 6,
    CRSF_RF_POWER_250_mW = 7,
    CRSF_RF_POWER_50_mW = 8
} crsrRfPower_e;

/*
0x1E Attitude
Payload:
int16_t     Pitch angle (rad / 10000)
int16_t     Roll angle (rad / 10000)
int16_t     Yaw angle (rad / 10000)
*/

// convert angle in decidegree to radians/10000 with +/-180 degree range
static int16_t decidegrees2Radians10000(int16_t angle_decidegree)
{
    while (angle_decidegree > 1800)
        angle_decidegree -= 3600;
    while (angle_decidegree < -1800)
        angle_decidegree += 3600;
    return RAD * 1000 * angle_decidegree;
}

// fill dst buffer with crsf-attitude telemetry frame
void crsfFrameAttitude(sbuf_t *dst)
{
    sbufWriteU8(dst, CRSF_FRAMETYPE_ATTITUDE);
    sbufWriteS16BE(dst, decidegrees2Radians10000(attitude.values.pitch));
    sbufWriteS16BE(dst, decidegrees2Radians10000(attitude.values.roll));
    sbufWriteS16BE(dst, decidegrees2Radians10000(attitude.values.yaw));
}

/*
0x21 Flight mode text based
Payload:
char[]      Flight mode (Null terminated string)
*/

static const char * govStateNames[] = {
    "OFF",
    "IDLE",
    "SPOOLUP",
    "RECOVERY",
    "ACTIVE",
    "THR-OFF",
    "LOST-HS",
    "AUTOROT",
    "BAILOUT",
};

static void crsfGovernorInfo(char *buf)
{
    // Modes that are only relevant when disarmed
    if (!ARMING_FLAG(ARMED)) {
        if (isArmingDisabled())
            strcpy(buf, "DISABLED");
        else
            strcpy(buf, "DISARMED");
    }
    else {
        strcpy(buf, govStateNames[getGovernorState()]);
    }
}

void crsfFrameFlightMode(sbuf_t *dst)
{
    char buff[32] = { 0, };

    crsfGovernorInfo(buff);

    sbufWriteU8(dst, CRSF_FRAMETYPE_FLIGHT_MODE);
    sbufWriteStringWithZeroTerminator(dst, buff);
}

/*
0x29 Device Info
Payload:
uint8_t     Destination
uint8_t     Origin
char[]      Device Name (Null terminated string)
uint32_t    Null Bytes
uint32_t    Null Bytes
uint32_t    Null Bytes
uint8_t     255 (Max MSP Parameter)
uint8_t     0x01 (Parameter version 1)
*/
void crsfFrameDeviceInfo(sbuf_t *dst)
{
    char buff[30];

    tfp_sprintf(buff, "%s %s: %s", FC_FIRMWARE_NAME, FC_VERSION_STRING, systemConfig()->boardIdentifier);

    sbufWriteU8(dst, CRSF_FRAMETYPE_DEVICE_INFO);
    sbufWriteU8(dst, CRSF_ADDRESS_RADIO_TRANSMITTER);
    sbufWriteU8(dst, CRSF_ADDRESS_FLIGHT_CONTROLLER);
    sbufWriteStringWithZeroTerminator(dst, buff);
    sbufWriteU32(dst, 0);
    sbufWriteU32(dst, 0);
    sbufWriteU32(dst, 0);
    sbufWriteU8(dst, CRSF_DEVICEINFO_PARAMETER_COUNT);
    sbufWriteU8(dst, CRSF_DEVICEINFO_VERSION);
}

/*
0x88 Rotorflight telemetry
Payload:
uint16_t    Sensor id
uint8_t     Data length
data        Sensor data
...
*/

void crsfFrameRotorflightTelemetryHeader(sbuf_t *dst)
{
    sbufWriteU8(dst, CRSF_FRAMETYPE_RF_TELEM);
}

void crsfFrameRotorflightTelemetrySensor(sbuf_t *dst, const telemetrySensor_t * sensor, telemetryValue_t value)
{
    sbufWriteU16BE(dst, sensor->code);
    sensor->encode(dst, value);
}


#if defined(USE_CRSF_V3)
void crsfFrameSpeedNegotiationResponse(sbuf_t *dst, bool reply)
{
    uint8_t *start = sbufPtr(dst);
    sbufWriteU8(dst, CRSF_FRAMETYPE_COMMAND);
    sbufWriteU8(dst, CRSF_ADDRESS_CRSF_RECEIVER);
    sbufWriteU8(dst, CRSF_ADDRESS_FLIGHT_CONTROLLER);
    sbufWriteU8(dst, CRSF_COMMAND_SUBCMD_GENERAL);
    sbufWriteU8(dst, CRSF_COMMAND_SUBCMD_GENERAL_CRSF_SPEED_RESPONSE);
    sbufWriteU8(dst, crsfSpeed.portID);
    sbufWriteU8(dst, reply);
    crc8_poly_0xba_sbuf_append(dst, start);
}

static void crsfProcessSpeedNegotiationCmd(uint8_t *frameStart)
{
    uint32_t newBaudrate = frameStart[2] << 24 | frameStart[3] << 16 | frameStart[4] << 8 | frameStart[5];
    uint8_t ii = 0;
    for (ii = 0; ii < BAUD_COUNT; ++ii) {
        if (newBaudrate == baudRates[ii]) {
            break;
        }
    }
    crsfSpeed.portID = frameStart[1];
    crsfSpeed.index = ii;
}

void crsfScheduleSpeedNegotiationResponse(void)
{
    crsfSpeed.hasPendingReply = true;
    crsfSpeed.isNewSpeedValid = false;
}

void speedNegotiationProcess(timeUs_t currentTimeUs)
{
    if (crsfSpeed.hasPendingReply) {
        bool found = (crsfSpeed.index < BAUD_COUNT) && crsfRxUseNegotiatedBaud();
        sbuf_t *dst = crsfInitializeSbuf();
        crsfFrameSpeedNegotiationResponse(dst, found);
        crsfRxSendTelemetryData(); // prevent overwriting previous data
        crsfFinalizeSbuf(dst);
        crsfRxSendTelemetryData();
        crsfSpeed.hasPendingReply = false;
        crsfSpeed.isNewSpeedValid = found;
        crsfSpeed.confirmationTime = currentTimeUs;
    } else if (crsfSpeed.isNewSpeedValid) {
        if (cmpTimeUs(currentTimeUs, crsfSpeed.confirmationTime) >= 4000) {
            // delay 4ms before applying the new baudrate
            crsfRxUpdateBaudrate(getCrsfDesiredSpeed());
            crsfSpeed.isNewSpeedValid = false;
            isCrsfV3Running = true;
        }
    } else if (!featureIsEnabled(FEATURE_TELEMETRY) && crsfRxUseNegotiatedBaud()) {
        // Send heartbeat if telemetry is disabled to allow RX to detect baud rate mismatches
        sbuf_t *dst = crsfInitializeSbuf();
        crsfFrameHeartbeat(dst);
        crsfRxSendTelemetryData(); // prevent overwriting previous data
        crsfFinalizeSbuf(dst);
        crsfRxSendTelemetryData();
    }
}
#endif

#if defined(USE_CRSF_CMS_TELEMETRY)

#define CRSF_DISPLAYPORT_MAX_CHUNK_LENGTH   50
#define CRSF_DISPLAYPORT_BATCH_MAX          0x3F
#define CRSF_DISPLAYPORT_FIRST_CHUNK_MASK   0x80
#define CRSF_DISPLAYPORT_LAST_CHUNK_MASK    0x40
#define CRSF_DISPLAYPORT_SANITIZE_MASK      0x60
#define CRSF_RLE_CHAR_REPEATED_MASK         0x80
#define CRSF_RLE_MAX_RUN_LENGTH             256
#define CRSF_RLE_BATCH_SIZE                 2

static uint16_t getRunLength(const void *start, const void *end)
{
    uint8_t *cursor = (uint8_t*)start;
    uint8_t c = *cursor;
    size_t runLength = 0;
    for (; cursor != end; cursor++) {
        if (*cursor == c) {
            runLength++;
        } else {
            break;
        }
    }
    return runLength;
}

static void cRleEncodeStream(sbuf_t *source, sbuf_t *dest, uint8_t maxDestLen)
{
    const uint8_t *destEnd = sbufPtr(dest) + maxDestLen;
    while (sbufBytesRemaining(source) && (sbufPtr(dest) < destEnd)) {
        const uint8_t destRemaining = destEnd - sbufPtr(dest);
        const uint8_t *srcPtr = sbufPtr(source);
        const uint16_t runLength = getRunLength(srcPtr, source->end);
        uint8_t c = *srcPtr;
        if (runLength > 1) {
            c |=  CRSF_RLE_CHAR_REPEATED_MASK;
            const uint8_t fullBatches = (runLength / CRSF_RLE_MAX_RUN_LENGTH);
            const uint8_t remainder = (runLength % CRSF_RLE_MAX_RUN_LENGTH);
            const uint8_t totalBatches = fullBatches + (remainder ? 1 : 0);
            if (destRemaining >= totalBatches * CRSF_RLE_BATCH_SIZE) {
                for (unsigned int i = 1; i <= totalBatches; i++) {
                    const uint8_t batchLength = (i < totalBatches) ? CRSF_RLE_MAX_RUN_LENGTH : remainder;
                    sbufWriteU8(dest, c);
                    sbufWriteU8(dest, batchLength);
                }
                sbufAdvance(source, runLength);
            } else {
                break;
            }
        }
        else if (destRemaining >= runLength) {
            sbufWriteU8(dest, c);
            sbufAdvance(source, runLength);
        }
    }
}

static void crsfFrameDisplayPortChunk(sbuf_t *dst, sbuf_t *src, uint8_t batchId, uint8_t idx)
{
    sbufWriteU8(dst, CRSF_FRAMETYPE_DISPLAYPORT_CMD);
    sbufWriteU8(dst, CRSF_ADDRESS_RADIO_TRANSMITTER);
    sbufWriteU8(dst, CRSF_ADDRESS_FLIGHT_CONTROLLER);
    sbufWriteU8(dst, CRSF_DISPLAYPORT_SUBCMD_UPDATE);
    uint8_t *metaPtr = sbufPtr(dst);
    sbufWriteU8(dst, batchId);
    sbufWriteU8(dst, idx);
    cRleEncodeStream(src, dst, CRSF_DISPLAYPORT_MAX_CHUNK_LENGTH);
    if (idx == 0)
        *metaPtr |= CRSF_DISPLAYPORT_FIRST_CHUNK_MASK;
    if (!sbufBytesRemaining(src))
        *metaPtr |= CRSF_DISPLAYPORT_LAST_CHUNK_MASK;
}

static void crsfFrameDisplayPortClear(sbuf_t *dst)
{
    sbufWriteU8(dst, CRSF_FRAMETYPE_DISPLAYPORT_CMD);
    sbufWriteU8(dst, CRSF_ADDRESS_RADIO_TRANSMITTER);
    sbufWriteU8(dst, CRSF_ADDRESS_FLIGHT_CONTROLLER);
    sbufWriteU8(dst, CRSF_DISPLAYPORT_SUBCMD_CLEAR);
}

#endif /* USE_CRSF_CMS_TELEMETRY */

#if defined(USE_MSP_OVER_TELEMETRY)

static bool mspReplyPending;
static uint8_t mspRequestOriginID = 0; // Saved origin of the MSP request

void crsfScheduleMspResponse(uint8_t requestOriginID)
{
    mspReplyPending = true;
    mspRequestOriginID = requestOriginID;
}

// sends MSP response chunk over CRSF. Must be of type mspResponseFnPtr
static void crsfSendMspResponse(uint8_t *payload, const uint8_t payloadSize)
{
    sbuf_t *dst = crsfInitializeSbuf();
    sbufWriteU8(dst, CRSF_FRAMETYPE_MSP_RESP);
    sbufWriteU8(dst, mspRequestOriginID);
    sbufWriteU8(dst, CRSF_ADDRESS_FLIGHT_CONTROLLER);
    sbufWriteData(dst, payload, payloadSize);
    crsfFinalizeSbuf(dst);
}

#endif /* USE_MSP_OVER_TELEMETRY */


static void processCrsfTelemetry(void)
{
    if (crsfRxIsTelemetryBufEmpty()) {
        telemetrySlot_t *slot = telemetryScheduleNext();
        if (slot) {
            sbuf_t *dst = crsfInitializeSbuf();
            switch (slot->sensor->code) {
                case TELEM_ATTITUDE:
                    crsfFrameAttitude(dst);
                    break;
                case TELEM_BATTERY:
                    crsfFrameBatterySensor(dst);
                    break;
                case TELEM_FLIGHT_MODE:
                    crsfFrameFlightMode(dst);
                    break;
#ifdef USE_GPS
                case TELEM_GPS:
                    crsfFrameGps(dst);
                    break;
#endif
                case TELEM_HEARTBEAT:
                default:
                    crsfFrameHeartbeat(dst);
                    break;
            }
            telemetryScheduleCommit(slot);
            crsfFinalizeSbuf(dst);
        }
    }
}

static void processRotorflightTelemetry(void)
{
    if (crsfRxIsTelemetryBufEmpty()) {
        sbuf_t *dst = crsfInitializeSbuf();
        crsfFrameRotorflightTelemetryHeader(dst);
        while (sbufBytesRemaining(dst) >= 6) {
            telemetrySlot_t *slot = telemetryScheduleNext();
            if (slot) {
                uint8_t *ptr = sbufPtr(dst);
                crsfFrameRotorflightTelemetrySensor(dst, slot->sensor, slot->value);
                if (sbufBytesRemaining(dst) >= 2)
                    telemetryScheduleCommit(slot);
                else
                    sbufJump(dst, ptr);
            }
        }
        crsfFinalizeSbuf(dst);
    }
}


void crsfScheduleDeviceInfoResponse(void)
{
    deviceInfoReplyPending = true;
}

bool checkCrsfTelemetryState(void)
{
    return crsfTelemetryEnabled;
}

void initCrsfTelemetry(void)
{
    // check if there is a serial port open for CRSF telemetry (ie opened by the CRSF RX)
    // and feature is enabled, if so, set CRSF telemetry enabled
    crsfTelemetryEnabled = crsfRxIsActive();

    if (crsfTelemetryEnabled) {
        deviceInfoReplyPending = false;
#if defined(USE_MSP_OVER_TELEMETRY)
        mspReplyPending = false;
#endif
#if defined(USE_CRSF_CMS_TELEMETRY)
        crsfDisplayportRegister();
#endif
    }
}

#if defined(USE_CRSF_CMS_TELEMETRY)
void crsfProcessDisplayPortCmd(uint8_t *frameStart)
{
    uint8_t cmd = *frameStart;
    switch (cmd) {
        case CRSF_DISPLAYPORT_SUBCMD_OPEN:;
            const uint8_t rows = *(frameStart + CRSF_DISPLAYPORT_OPEN_ROWS_OFFSET);
            const uint8_t cols = *(frameStart + CRSF_DISPLAYPORT_OPEN_COLS_OFFSET);
            crsfDisplayPortSetDimensions(rows, cols);
            crsfDisplayPortMenuOpen();
            break;
        case CRSF_DISPLAYPORT_SUBCMD_CLOSE:
            crsfDisplayPortMenuExit();
            break;
        case CRSF_DISPLAYPORT_SUBCMD_POLL:
            crsfDisplayPortRefresh();
            break;
        default:
            break;
    }

}
#endif

#if defined(USE_CRSF_V3)
void crsfProcessCommand(uint8_t *frameStart)
{
    uint8_t cmd = frameStart[0];
    uint8_t sub = frameStart[1];

    if (cmd == CRSF_COMMAND_SUBCMD_GENERAL && sub ==  CRSF_COMMAND_SUBCMD_GENERAL_CRSF_SPEED_PROPOSAL) {
        crsfProcessSpeedNegotiationCmd(&frameStart[1]);
        crsfScheduleSpeedNegotiationResponse();
    }
}
#endif

/*
 * Called periodically by the scheduler
 */
void handleCrsfTelemetry(timeUs_t currentTimeUs)
{
    static uint32_t crsfLastCycleTime;

    if (!crsfTelemetryEnabled)
        return;

#if defined(USE_CRSF_V3)
    if (crsfBaudNegotiationInProgress())
        return;
#endif

    // Give the receiver a chance to send any outstanding telemetry data.
    // This needs to be done at high frequency, to enable the RX to send the telemetry frame
    // in between the RX frames.
    crsfRxSendTelemetryData();

    // Send ad-hoc response frames as soon as possible
#if defined(USE_MSP_OVER_TELEMETRY)
    if (mspReplyPending) {
        mspReplyPending = handleCrsfMspFrameBuffer(&crsfSendMspResponse);
        crsfLastCycleTime = currentTimeUs; // reset telemetry timing due to ad-hoc request
        return;
    }
#endif

    if (deviceInfoReplyPending) {
        sbuf_t *dst = crsfInitializeSbuf();
        crsfFrameDeviceInfo(dst);
        crsfFinalizeSbuf(dst);
        deviceInfoReplyPending = false;
        crsfLastCycleTime = currentTimeUs; // reset telemetry timing due to ad-hoc request
        return;
    }

#if defined(USE_CRSF_CMS_TELEMETRY)
    if (crsfDisplayPortScreen()->reset) {
        crsfDisplayPortScreen()->reset = false;
        sbuf_t *dst = crsfInitializeSbuf();
        crsfFrameDisplayPortClear(dst);
        crsfFinalizeSbuf(dst);
        crsfLastCycleTime = currentTimeUs;
        return;
    }
    static uint8_t displayPortBatchId = 0;
    if (crsfDisplayPortIsReady() && crsfDisplayPortScreen()->updated) {
        crsfDisplayPortScreen()->updated = false;
        uint16_t screenSize = crsfDisplayPortScreen()->rows * crsfDisplayPortScreen()->cols;
        uint8_t *srcStart = (uint8_t*)crsfDisplayPortScreen()->buffer;
        uint8_t *srcEnd = (uint8_t*)(crsfDisplayPortScreen()->buffer + screenSize);
        sbuf_t displayPortSbuf;
        sbuf_t *src = sbufInit(&displayPortSbuf, srcStart, srcEnd);
        displayPortBatchId = (displayPortBatchId  + 1) % CRSF_DISPLAYPORT_BATCH_MAX;
        uint8_t i = 0;
        while (sbufBytesRemaining(src)) {
            sbuf_t *dst = crsfInitializeSbuf();
            crsfFrameDisplayPortChunk(dst, src, displayPortBatchId, i);
            crsfFinalizeSbuf(dst);
            crsfRxSendTelemetryData();
            i++;
        }
        crsfLastCycleTime = currentTimeUs;
        return;
    }
#endif

    // Actual telemetry data only needs to be sent at a low frequency, ie 10Hz
    // Spread out scheduled frames evenly so each frame is sent at the same frequency.
    if (currentTimeUs >= crsfLastCycleTime + CRSF_CYCLETIME_US) {
        crsfLastCycleTime = currentTimeUs;
        if (1)
            processCrsfTelemetry();
        else
            processRotorflightTelemetry();
        crsfRxSendTelemetryData();
    }
}

#if defined(USE_RX_EXPRESSLRS)

static int crsfFinalizeSbufBuf(sbuf_t *dst, uint8_t *frame)
{
    // frame size including CRC
    const size_t frameSize = sbufPtr(dst) - crsfFrame + 2;

    // Set frame length into the placeholder
    crsfFrame[1] = frameSize - 3;

    // frame CRC
    crc8_dvb_s2_sbuf_append(dst, &crsfFrame[2]); // start at byte 2, since CRC does not include device address and frame length

    // Copy data to the frame
    memcpy(frame, crsfFrame, frameSize);

    return frameSize;
}

int getCrsfFrame(uint8_t *frame, crsfFrameType_e frameType)
{
    sbuf_t *dst = crsfInitializeSbuf();

    switch (frameType) {
        default:
        case CRSF_FRAMETYPE_ATTITUDE:
            crsfFrameAttitude(dst);
            break;
        case CRSF_FRAMETYPE_BATTERY_SENSOR:
            crsfFrameBatterySensor(dst);
            break;
        case CRSF_FRAMETYPE_FLIGHT_MODE:
            crsfFrameFlightMode(dst);
            break;
#if defined(USE_GPS)
        case CRSF_FRAMETYPE_GPS:
            crsfFrameGps(dst);
            break;
#endif
#if defined(USE_MSP_OVER_TELEMETRY)
        case CRSF_FRAMETYPE_DEVICE_INFO:
            crsfFrameDeviceInfo(dst);
            break;
#endif
    }

    return crsfFinalizeSbufBuf(dst, frame);
}

#if defined(USE_MSP_OVER_TELEMETRY)
int getCrsfMspFrame(uint8_t *frame, uint8_t *payload, const uint8_t payloadSize)
{
    sbuf_t *dst = crsfInitializeSbuf();

    sbufWriteU8(dst, CRSF_FRAMETYPE_MSP_RESP);
    sbufWriteU8(dst, CRSF_ADDRESS_RADIO_TRANSMITTER);
    sbufWriteU8(dst, CRSF_ADDRESS_FLIGHT_CONTROLLER);
    sbufWriteData(dst, payload, payloadSize);

    return crsfFinalizeSbufBuf(dst, frame);
}
#endif /* USE_MSP_OVER_TELEMETRY */
#endif /* USE_RX_EXPRESSLRS */

#endif /* USE_TELEMETRY_CRSF */
