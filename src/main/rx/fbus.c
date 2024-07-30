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
#include <string.h>

#include "platform.h"

#ifdef USE_SERIALRX_FBUS

#include "build/debug.h"
#include "build/dprintf.h"

#include "common/maths.h"
#include "common/utils.h"

#include "drivers/time.h"

#include "io/serial.h"

#ifdef MSP_FIRMWARE_UPDATE
#include "fc/firmware_update.h"
#endif

#ifdef USE_TELEMETRY
#include "telemetry/telemetry.h"
#include "telemetry/smartport.h"
#endif

#include "pg/rx.h"

#include "rx/frsky_crc.h"
#include "rx/rx.h"
#include "rx/sbus_channels.h"
#include "rx/fbus.h"


#define FBUS_MIN_TELEMETRY_RESPONSE_DELAY_US            500
#define FBUS_MAX_TELEMETRY_RESPONSE_DELAY_US            3000

#define FBUS_OTA_MAX_RESPONSE_TIME_US_DEFAULT           200
#define FBUS_OTA_MIN_RESPONSE_DELAY_US_DEFAULT          50

#define FBUS_MAX_TELEMETRY_AGE_US                       500000
#define FBUS_FC_COMMON_ID                               0x1B
#define FBUS_FC_MSP_ID                                  0x0D

#define FBUS_BAUDRATE                                   460800
#define FBUS_PORT_OPTIONS                               (SERIAL_STOPBITS_1 | SERIAL_PARITY_NO)

#define FBUS_RX_TIMEOUT                                 120 // µs

#define FBUS_CONTROL_FRAME_LENGTH                       24
#define FBUS_OTA_DATA_FRAME_LENGTH                      32
#define FBUS_DOWNLINK_FRAME_LENGTH                      8
#define FBUS_UPLINK_FRAME_LENGTH                        8

#define FBUS_OTA_DATA_FRAME_BYTES                       32


enum {
    DEBUG_FBUS_FRAME_INTERVAL = 0,
    DEBUG_FBUS_FRAME_ERRORS,
    DEBUG_FBUS_FRAME_LAST_ERROR,
    DEBUG_FBUS_TELEMETRY_INTERVAL,
};

enum {
    DEBUG_FBUS_NO_ERROR = 0,
    DEBUG_FBUS_ERROR_TIMEOUT,
    DEBUG_FBUS_ERROR_OVERSIZE,
    DEBUG_FBUS_ERROR_SIZE,
    DEBUG_FBUS_ERROR_CHECKSUM,
    DEBUG_FBUS_ERROR_PHYID_CRC,
    DEBUG_FBUS_ERROR_TYPE,
    DEBUG_FBUS_ERROR_TYPE_SIZE,
    DEBUG_FBUS_ERROR_OTA_BAD_ADDRESS,
};

enum {
    FBUS_CONTROL_FRAME_TYPE_OTA_START   = 0xF0,
    FBUS_CONTROL_FRAME_TYPE_OTA_DATA    = 0xF1,
    FBUS_CONTROL_FRAME_TYPE_OTA_STOP    = 0xF2,
    FBUS_CONTROL_FRAME_TYPE_CHANNELS    = 0xFF,
};

enum {
    FBUS_FRAME_TYPE_CONTROL,
    FBUS_FRAME_TYPE_DOWNLINK,
};

typedef enum {
    FS_CONTROL_FRAME_START,
    FS_CONTROL_FRAME_TYPE,
    FS_CONTROL_FRAME_DATA,
    FS_DOWNLINK_FRAME_START,
    FS_DOWNLINK_FRAME_DATA,
} frame_state_e;

enum {
    FBUS_FRAME_ID_NULL      = 0x00,
    FBUS_FRAME_ID_DATA      = 0x10,
    FBUS_FRAME_ID_READ      = 0x30,
    FBUS_FRAME_ID_WRITE     = 0x31,
    FBUS_FRAME_ID_RESPONSE  = 0x32,
    FBUS_FRAME_ID_OTA_START = 0xF0,
    FBUS_FRAME_ID_OTA_DATA  = 0xF1,
    FBUS_FRAME_ID_OTA_STOP  = 0xF2
};

typedef struct {
    sbusChannels_t channels;
    uint8_t rssi;
} __packed fbusChannelData_t;

typedef struct {
    uint8_t data[FBUS_OTA_DATA_FRAME_BYTES];
} __packed fbusOTAData_t;

typedef struct {
    uint8_t phyID;
    smartPortPayload_t telemetryData;
} __packed fbusDownlinkData_t;

typedef struct {
    uint8_t type;
    union {
        fbusChannelData_t   rc;
        fbusOTAData_t       ota;
    };
} __packed fbusControlFrame_t;

typedef struct {
    uint8_t type;
    union {
        fbusControlFrame_t control;
        fbusDownlinkData_t downlink;
    };
} __packed fbusFrame_t;

// RX frames ring buffer
#define NUM_RX_BUFFERS 15

typedef struct fbusBuffer_s {
    uint8_t data[sizeof(fbusFrame_t) + sizeof(uint8_t)];    // +CRC
    uint8_t length;
} fbusBuffer_t;

typedef struct {
    uint32_t size;
    uint8_t  crc;
} __packed firmwareUpdateHeader_t;

static volatile fbusBuffer_t rxBuffer[NUM_RX_BUFFERS];

static volatile uint8_t rxBufferWriteIndex = 0;
static volatile uint8_t rxBufferReadIndex = 0;

static serialPort_t *fbusPort;

#ifdef USE_TELEMETRY_SMARTPORT
static bool telemetryEnabled = false;

static smartPortPayload_t *mspPayload = NULL;
static volatile timeUs_t lastTelemetryFrameReceivedUs;

static volatile bool clearToSend = false;
static volatile bool sendNullFrame = false;

static uint8_t downlinkPhyID;

static const smartPortPayload_t emptySmartPortFrame = INIT_ZERO;

static smartPortPayload_t *otaResponsePayload = NULL;
static bool otaMode = false;
static bool otaDataNeedsProcessing = false;
static uint16_t otaMinResponseDelay = FBUS_OTA_MIN_RESPONSE_DELAY_US_DEFAULT;
static uint16_t otaMaxResponseTime = FBUS_OTA_MAX_RESPONSE_TIME_US_DEFAULT;
static uint32_t otaDataAddress;
static uint8_t otaDataBuffer[FBUS_OTA_DATA_FRAME_BYTES];
static timeUs_t otaFrameEndTimestamp = 0;

static bool firmwareUpdateError = false;
#ifdef MSP_FIRMWARE_UPDATE
static uint8_t firmwareUpdateCRC;
static timeUs_t readyToUpdateFirmwareTimestamp = 0;
#endif
#endif

static volatile uint16_t frameErrors = 0;

static void reportFrameError(uint8_t errorReason)
{
    frameErrors++;

    DEBUG_SET(DEBUG_FPORT, DEBUG_FBUS_FRAME_ERRORS, frameErrors);
    DEBUG_SET(DEBUG_FPORT, DEBUG_FBUS_FRAME_LAST_ERROR, errorReason);
}

static void clearWriteBuffer(void)
{
    rxBuffer[rxBufferWriteIndex].length = 0;
}

static bool nextWriteBuffer(void)
{
    const uint8_t nextWriteIndex = (rxBufferWriteIndex + 1) % NUM_RX_BUFFERS;

    if (nextWriteIndex != rxBufferReadIndex) {
        rxBufferWriteIndex = nextWriteIndex;
        clearWriteBuffer();
        return true;
    } else {
        clearWriteBuffer();
        return false;
    }
}

static uint8_t writeBuffer(uint8_t byte)
{
    volatile uint8_t * const buffer = rxBuffer[rxBufferWriteIndex].data;
    volatile uint8_t * const buflen = &rxBuffer[rxBufferWriteIndex].length;
    buffer[*buflen] = byte;
    *buflen += 1;
    return *buflen;
}

// UART RX ISR
static void fbusDataReceive(uint16_t byte, void *callback_data)
{
    UNUSED(callback_data);

    static volatile frame_state_e state = FS_CONTROL_FRAME_START;
    static volatile timeUs_t lastRxByteTimestamp = 0;
    static unsigned controlFrameSize;

    const timeUs_t currentTimeUs = microsISR();
    const timeUs_t timeSincePreviousRxByte = cmpTimeUs(currentTimeUs, lastRxByteTimestamp);

    lastRxByteTimestamp = currentTimeUs;

    //dprintf("0x%02X ", byte);

#ifdef USE_TELEMETRY_SMARTPORT
    clearToSend = false;
#endif

    if (state != FS_CONTROL_FRAME_START && timeSincePreviousRxByte > FBUS_RX_TIMEOUT) {
        state = FS_CONTROL_FRAME_START;
    }

    switch (state)
    {
        case FS_CONTROL_FRAME_START:
            if (byte == FBUS_CONTROL_FRAME_LENGTH || byte == FBUS_OTA_DATA_FRAME_LENGTH) {
                clearWriteBuffer();
                writeBuffer(FBUS_FRAME_TYPE_CONTROL);
                state = FS_CONTROL_FRAME_TYPE;
            }
            break;

        case FS_CONTROL_FRAME_TYPE:
            if (byte == FBUS_CONTROL_FRAME_TYPE_CHANNELS || byte == FBUS_CONTROL_FRAME_TYPE_OTA_DATA ||
                byte == FBUS_CONTROL_FRAME_TYPE_OTA_START || byte == FBUS_CONTROL_FRAME_TYPE_OTA_STOP) {
                writeBuffer(byte);
                controlFrameSize = (byte == FBUS_CONTROL_FRAME_TYPE_OTA_DATA ? FBUS_OTA_DATA_FRAME_LENGTH : FBUS_CONTROL_FRAME_LENGTH) + 2; // +2 = General frame type + CRC
                state = FS_CONTROL_FRAME_DATA;
            } else {
                state = FS_CONTROL_FRAME_START;
            }
            break;

        case FS_CONTROL_FRAME_DATA: {
            if (writeBuffer(byte) > controlFrameSize) {
                nextWriteBuffer();
                state = FS_DOWNLINK_FRAME_START;
            }
            break;
        }

        case FS_DOWNLINK_FRAME_START:
            if (byte == FBUS_DOWNLINK_FRAME_LENGTH) {
                writeBuffer(FBUS_FRAME_TYPE_DOWNLINK);
                state = FS_DOWNLINK_FRAME_DATA;
            } else {
                state = FS_CONTROL_FRAME_START;
            }
            break;

        case FS_DOWNLINK_FRAME_DATA:
            if (writeBuffer(byte) > (FBUS_DOWNLINK_FRAME_LENGTH + 1)) {
#ifdef USE_TELEMETRY_SMARTPORT
                if ((rxBuffer[rxBufferWriteIndex].data[2] >= FBUS_FRAME_ID_OTA_START) && (rxBuffer[rxBufferWriteIndex].data[2] <= FBUS_FRAME_ID_OTA_STOP)) {
                    otaFrameEndTimestamp = currentTimeUs;
                }
                lastTelemetryFrameReceivedUs = currentTimeUs;
#endif
                nextWriteBuffer();
                state = FS_CONTROL_FRAME_START;
            }
            break;

        default:
            state = FS_CONTROL_FRAME_START;
            break;
    }
}

#ifdef USE_TELEMETRY_SMARTPORT
static void writeUplinkFramePhyID(uint8_t phyID, const smartPortPayload_t *payload)
{
    serialWrite(fbusPort, FBUS_UPLINK_FRAME_LENGTH);
    serialWrite(fbusPort, phyID);

    uint16_t checksum = 0;
    frskyCheckSumStep(&checksum, phyID);

    uint8_t *data = (uint8_t *)payload;
    for (unsigned i = 0; i < sizeof(smartPortPayload_t); ++i, ++data) {
        serialWrite(fbusPort, *data);
        frskyCheckSumStep(&checksum, *data);
    }

    frskyCheckSumFini(&checksum);
    serialWrite(fbusPort, checksum);
}

static void writeUplinkFrame(const smartPortPayload_t *payload)
{
    writeUplinkFramePhyID(FBUS_FC_COMMON_ID, payload);
}
#endif

static uint8_t fbusFrameStatus(rxRuntimeState_t *rxRuntimeState)
{
#ifdef USE_TELEMETRY_SMARTPORT
    static smartPortPayload_t payloadBuffer;
    static bool hasTelemetryRequest = false;
    static uint32_t otaPrevDataAddress;
    static bool otaJustStarted = false;
    static bool otaGotData = false;
#endif
    static timeUs_t frameReceivedTimestamp = 0;

    uint8_t result = RX_FRAME_PENDING;

    timeUs_t currentTimeUs = micros();

    if (rxBufferReadIndex != rxBufferWriteIndex) {

        volatile uint8_t *buffer = rxBuffer[rxBufferReadIndex].data;
        uint8_t buflen = rxBuffer[rxBufferReadIndex].length;

        fbusFrame_t *frame = (fbusFrame_t *)buffer;

        if (!frskyCheckSumIsGood((uint8_t *)(buffer + 1), buflen - 1)) {
            reportFrameError(DEBUG_FBUS_ERROR_CHECKSUM);
        }
        else {
            switch (frame->type) {
                case FBUS_FRAME_TYPE_CONTROL:
                    switch (frame->control.type) {
                        case FBUS_CONTROL_FRAME_TYPE_CHANNELS:
                            result = sbusChannelsDecode(rxRuntimeState, &frame->control.rc.channels);
                            // TODO lqTrackerSet(rxRuntimeState->lqTracker, scaleRange(frame->control.rc.rssi, 0, 100, 0, RSSI_MAX_VALUE));
                            frameReceivedTimestamp = currentTimeUs;
#ifdef USE_TELEMETRY_SMARTPORT
                            otaMode = false;
#endif
                            break;

#ifdef USE_TELEMETRY_SMARTPORT
                        case FBUS_CONTROL_FRAME_TYPE_OTA_START:
                        {
                            uint8_t otaMinResponseDelayByte = frame->control.ota.data[0];
                            if ((otaMinResponseDelayByte > 0) && (otaMinResponseDelayByte <= 4)) {
                                otaMinResponseDelay = otaMinResponseDelayByte * 100;
                            } else {
                                otaMinResponseDelay = FBUS_OTA_MIN_RESPONSE_DELAY_US_DEFAULT;
                            }

                            uint8_t otaMaxResponseTimeByte = frame->control.ota.data[1];
                            if (otaMaxResponseTimeByte > 0) {
                                otaMaxResponseTime = otaMaxResponseTimeByte * 100;
                            } else {
                                otaMaxResponseTime = FBUS_OTA_MAX_RESPONSE_TIME_US_DEFAULT;
                            }

                            otaMode = true;
                            break;
                        }

                        case FBUS_CONTROL_FRAME_TYPE_OTA_DATA:
                        {
                            if (otaMode) {
                                memcpy(otaDataBuffer, frame->control.ota.data, sizeof(otaDataBuffer));
                                otaGotData = true;
                            } else {
                                reportFrameError(DEBUG_FBUS_ERROR_TYPE);
                            }
                            break;
                        }

                        case FBUS_CONTROL_FRAME_TYPE_OTA_STOP:
                        {
                            if (!otaMode) {
                                reportFrameError(DEBUG_FBUS_ERROR_TYPE);
                            }
                            break;
                        }
#endif

                        default:
                            reportFrameError(DEBUG_FBUS_ERROR_TYPE);
                            break;

                    }
                    break;

                case FBUS_FRAME_TYPE_DOWNLINK:
#ifdef USE_TELEMETRY_SMARTPORT
                    if (!telemetryEnabled) {
                        break;
                    }

                    downlinkPhyID = frame->downlink.phyID;
                    const uint8_t frameId = frame->downlink.telemetryData.frameId;

                    switch (frameId) {

                        case FBUS_FRAME_ID_NULL:
                            hasTelemetryRequest = true;
                            sendNullFrame = true;
                            break;

                        case FBUS_FRAME_ID_DATA:
                            hasTelemetryRequest = true;
                            break;

                        case FBUS_FRAME_ID_OTA_START:
                        case FBUS_FRAME_ID_OTA_DATA:
                        case FBUS_FRAME_ID_OTA_STOP:
                            switch (frameId) {
                                case FBUS_FRAME_ID_OTA_START:
                                    if (otaMode) {
                                        otaJustStarted = true;
                                        otaPrevDataAddress = 0;
                                        hasTelemetryRequest = true;
                                        otaDataNeedsProcessing = false;
                                        firmwareUpdateError = false;
                                    } else {
                                        reportFrameError(DEBUG_FBUS_ERROR_TYPE);
                                    }
                                    break;

                                case FBUS_FRAME_ID_OTA_DATA:
                                    if (otaMode) {
                                        otaDataAddress = frame->downlink.telemetryData.data;
                                        if (otaGotData && (otaDataAddress == (otaJustStarted ? 0 : otaPrevDataAddress + FBUS_OTA_DATA_FRAME_BYTES))) { // check that we got a control frame with data and check address
                                            otaPrevDataAddress = otaDataAddress;
                                            otaGotData = false;
                                            otaDataNeedsProcessing = true;
                                        }
                                        hasTelemetryRequest = true;
                                        otaJustStarted = false;
                                    } else {
                                        reportFrameError(DEBUG_FBUS_ERROR_TYPE);
                                    }
                                    break;

                                case FBUS_FRAME_ID_OTA_STOP:
                                    if (otaMode) {
                                        hasTelemetryRequest = true;
                                    } else {
                                        reportFrameError(DEBUG_FBUS_ERROR_TYPE);
                                    }
                                    break;
                            }
                            if (hasTelemetryRequest) {
                                memcpy(&payloadBuffer, &frame->downlink.telemetryData, sizeof(payloadBuffer));
                                otaResponsePayload = &payloadBuffer;
                            }
                            break;

                        default:
                            if ((frameId == FBUS_FRAME_ID_READ) && (downlinkPhyID == FBUS_FC_MSP_ID)) {
                                memcpy(&payloadBuffer, &frame->downlink.telemetryData, sizeof(payloadBuffer));
                                mspPayload = &payloadBuffer;
                                hasTelemetryRequest = true;
                            } else if (downlinkPhyID != FBUS_FC_COMMON_ID) {
#ifdef USE_SMARTPORT_MASTER
                                int8_t smartportPhyID = smartportMasterStripPhyIDCheckBits(downlinkPhyID);
                                if (smartportPhyID != -1) {
                                    smartportMasterForward(smartportPhyID, &frame->downlink.telemetryData);
                                    hasTelemetryRequest = true;
                                }
#endif
                            }
                            break;

                    }
#endif
                    break;

                default:
                    reportFrameError(DEBUG_FBUS_ERROR_TYPE);
                    break;

            }

        }

        rxBufferReadIndex = (rxBufferReadIndex + 1) % NUM_RX_BUFFERS;
    }

#ifdef USE_TELEMETRY_SMARTPORT
    if (((mspPayload || hasTelemetryRequest) && cmpTimeUs(currentTimeUs, lastTelemetryFrameReceivedUs) >= FBUS_MIN_TELEMETRY_RESPONSE_DELAY_US) ||
        (otaResponsePayload && cmpTimeUs(currentTimeUs, lastTelemetryFrameReceivedUs) >= otaMinResponseDelay)) {
        hasTelemetryRequest = false;
        clearToSend = true;
        result |= RX_FRAME_PROCESSING_REQUIRED;
    }
#endif

    if (frameReceivedTimestamp && cmpTimeUs(currentTimeUs, frameReceivedTimestamp) > FBUS_MAX_TELEMETRY_AGE_US) {
        // TODO lqTrackerSet(rxRuntimeState->lqTracker, 0);
        frameReceivedTimestamp = 0;
    }

#ifdef MSP_FIRMWARE_UPDATE
    if (readyToUpdateFirmwareTimestamp && cmpTimeUs(currentTimeUs, readyToUpdateFirmwareTimestamp) > 2000000) {
        readyToUpdateFirmwareTimestamp = 0;
        firmwareUpdateExec(firmwareUpdateCRC);
    }
#endif

    return result;
}

static bool fbusProcessFrame(const rxRuntimeState_t *rxRuntimeState)
{
    UNUSED(rxRuntimeState);

#ifdef USE_TELEMETRY_SMARTPORT

    timeUs_t currentTimeUs = micros();
    if (cmpTimeUs(currentTimeUs, lastTelemetryFrameReceivedUs) > FBUS_MAX_TELEMETRY_RESPONSE_DELAY_US) {
       clearToSend = false;
    }

    if (clearToSend) {
        if (otaResponsePayload) {
            switch (otaResponsePayload->frameId) {
                case FBUS_FRAME_ID_OTA_DATA: {
                    if (otaDataNeedsProcessing && !firmwareUpdateError) { // We have got fresh data
#ifdef MSP_FIRMWARE_UPDATE
                        static uint32_t firmwareUpdateSize;
                        uint32_t receivedSize = otaDataAddress - FBUS_OTA_DATA_FRAME_BYTES;
                        if (otaDataAddress == 0) {
                            static firmwareUpdateHeader_t *header = (firmwareUpdateHeader_t *)otaDataBuffer;
                            firmwareUpdateSize = header->size;
                            firmwareUpdateCRC = header->crc;
                            firmwareUpdateError = !firmwareUpdatePrepare(firmwareUpdateSize);
                        } else if (receivedSize < firmwareUpdateSize) {
                            uint8_t firmwareDataSize = MIN((uint8_t)FBUS_OTA_DATA_FRAME_BYTES, firmwareUpdateSize - receivedSize);
                            firmwareUpdateError = !firmwareUpdateStore(otaDataBuffer, firmwareDataSize);
                        }
#else
                        firmwareUpdateError = true;
#endif
                    }
                    otaDataNeedsProcessing = false;
                    break;
                }

                case FBUS_FRAME_ID_OTA_STOP:
                    otaMode = false;
#ifdef MSP_FIRMWARE_UPDATE
                    readyToUpdateFirmwareTimestamp = currentTimeUs;
#endif
                    break;
            }

            timeDelta_t otaResponseTime = cmpTimeUs(micros(), otaFrameEndTimestamp);
            if (!firmwareUpdateError && (otaResponseTime <= otaMaxResponseTime)) { // We can answer in time (firmwareUpdateStore can take time because it might need to erase flash)
                writeUplinkFramePhyID(downlinkPhyID, otaResponsePayload);
            }

            otaResponsePayload = NULL;
            clearToSend = false;

        } else if ((downlinkPhyID == FBUS_FC_COMMON_ID) || (downlinkPhyID == FBUS_FC_MSP_ID)) {
            if ((downlinkPhyID == FBUS_FC_MSP_ID) && !mspPayload) {
                clearToSend = false;
            } else if (!sendNullFrame) {
                processSmartPortTelemetry(mspPayload, &clearToSend, NULL);
                mspPayload = NULL;
            }

        } else {
#ifdef USE_SMARTPORT_MASTER
            int8_t smartportPhyID = smartportMasterStripPhyIDCheckBits(downlinkPhyID);

            if (smartportPhyID != -1) {
                if (sendNullFrame) {
                    if (!smartportMasterPhyIDIsActive(smartportPhyID)) { // send null frame only if the sensor is active
                        clearToSend = false;
                    }
                } else {
                    smartPortPayload_t forwardPayload;
                    if (smartportMasterNextForwardResponse(smartportPhyID, &forwardPayload) || smartportMasterGetSensorPayload(smartportPhyID, &forwardPayload)) {
                        writeUplinkFramePhyID(downlinkPhyID, &forwardPayload);
                    }
                    clearToSend = false; // either we answered or the sensor is not active, do not send null frame
                }
            } else {
                clearToSend = false;
            }
#else
            clearToSend = false;
#endif
        }

        if (clearToSend) {
            writeUplinkFramePhyID(downlinkPhyID, &emptySmartPortFrame);
            clearToSend = false;
        }

        sendNullFrame = false;
    }
#endif

    return true;
}


bool fbusRxInit(const rxConfig_t *rxConfig, rxRuntimeState_t *rxRuntimeState)
{
    static uint16_t sbusChannelData[SBUS_MAX_CHANNEL];

    initDebugSerial(SERIAL_PORT_USART6, 921600);

    rxRuntimeState->channelData = sbusChannelData;
    sbusChannelsInit(rxConfig, rxRuntimeState);

    rxRuntimeState->channelCount = SBUS_MAX_CHANNEL;
    rxRuntimeState->rxRefreshRate = 4000;

    rxRuntimeState->rcFrameStatusFn = fbusFrameStatus;
    rxRuntimeState->rcProcessFrameFn = fbusProcessFrame;
    rxRuntimeState->rcFrameTimeUsFn = rxFrameTimeUs;

    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RX_SERIAL);
    if (!portConfig) {
        return false;
    }

    fbusPort = openSerialPort(portConfig->identifier,
        FUNCTION_RX_SERIAL,
        fbusDataReceive,
        NULL,
        FBUS_BAUDRATE,
        MODE_RXTX,
        FBUS_PORT_OPTIONS |
        (rxConfig->serialrx_inverted ? SERIAL_INVERTED : SERIAL_NOT_INVERTED) |
        (rxConfig->halfDuplex ? SERIAL_BIDIR : SERIAL_UNIDIR) |
        (rxConfig->pinSwap ? SERIAL_PINSWAP : SERIAL_NOSWAP)
    );

    if (fbusPort) {
#ifdef USE_TELEMETRY_SMARTPORT
        telemetryEnabled = initSmartPortTelemetryExternal(writeUplinkFrame);
#endif

        dprintf("FBUS init done\r\n");
    }

    return fbusPort != NULL;
}

#endif
