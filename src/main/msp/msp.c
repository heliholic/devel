/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <limits.h>
#include <ctype.h>

#include "platform.h"

#include "blackbox/blackbox.h"
#include "blackbox/blackbox_io.h"

#include "build/build_config.h"
#include "build/debug.h"
#include "build/version.h"

#include "cli/cli.h"

#include "common/axis.h"
#include "common/bitarray.h"
#include "common/color.h"
#include "common/huffman.h"
#include "common/maths.h"
#include "common/streambuf.h"
#include "common/utils.h"

#include "config/config.h"
#include "config/config_eeprom.h"
#include "config/feature.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_spi.h"
#include "drivers/camera_control.h"
#include "drivers/compass/compass.h"
#include "drivers/display.h"
#include "drivers/dshot.h"
#include "drivers/dshot_command.h"
#include "drivers/flash.h"
#include "drivers/io.h"
#include "drivers/motor.h"
#include "drivers/osd.h"
#include "drivers/pwm_output.h"
#include "drivers/sdcard.h"
#include "drivers/serial.h"
#include "drivers/serial_escserial.h"
#include "drivers/system.h"
#include "drivers/usb_msc.h"
#include "drivers/vtx_common.h"
#include "drivers/vtx_table.h"
#include "drivers/freq.h"

#include "fc/board_info.h"
#include "fc/rc_rates.h"
#include "fc/core.h"
#include "fc/dispatch.h"
#include "fc/rc.h"
#include "fc/rc_adjustments.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/failsafe.h"
#include "flight/gps_rescue.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/position.h"
#include "flight/rpm_filter.h"
#include "flight/servos.h"

#include "io/asyncfatfs/asyncfatfs.h"
#include "io/beeper.h"
#include "io/flashfs.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/serial.h"
#include "io/serial_4way.h"
#include "io/servos.h"
#include "io/usb_msc.h"
#include "io/vtx_control.h"
#include "io/vtx.h"

#include "msp/msp_box.h"
#include "msp/msp_protocol.h"
#include "msp/msp_protocol_v2_betaflight.h"
#include "msp/msp_protocol_v2_common.h"
#include "msp/msp_serial.h"

#include "osd/osd.h"
#include "osd/osd_elements.h"
#include "osd/osd_warnings.h"

#include "pg/beeper.h"
#include "pg/board.h"
#include "pg/dyn_notch.h"
#include "pg/gyrodev.h"
#include "pg/motor.h"
#include "pg/rx.h"
#include "pg/rx_spi.h"
#include "pg/usb.h"
#include "pg/vcd.h"
#include "pg/vtx_table.h"

#include "rx/rx.h"
#include "rx/rx_bind.h"
#include "rx/msp.h"

#include "scheduler/scheduler.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"
#include "sensors/compass.h"
#include "sensors/esc_sensor.h"
#include "sensors/gyro.h"
#include "sensors/gyro_init.h"
#include "sensors/rangefinder.h"

#include "telemetry/msp_shared.h"
#include "telemetry/telemetry.h"

#ifdef USE_HARDWARE_REVISION_DETECTION
#include "hardware_revision.h"
#endif

#include "msp.h"


static const char * const flightControllerIdentifier = FC_FIRMWARE_IDENTIFIER; // 4 UPPER CASE alpha numeric characters that identify the flight controller.

enum {
    MSP_REBOOT_FIRMWARE = 0,
    MSP_REBOOT_BOOTLOADER_ROM,
    MSP_REBOOT_MSC,
    MSP_REBOOT_MSC_UTC,
    MSP_REBOOT_BOOTLOADER_FLASH,
    MSP_REBOOT_COUNT,
};

static uint8_t rebootMode;

typedef enum {
    MSP_SDCARD_STATE_NOT_PRESENT = 0,
    MSP_SDCARD_STATE_FATAL       = 1,
    MSP_SDCARD_STATE_CARD_INIT   = 2,
    MSP_SDCARD_STATE_FS_INIT     = 3,
    MSP_SDCARD_STATE_READY       = 4
} mspSDCardState_e;

typedef enum {
    MSP_SDCARD_FLAG_SUPPORTED   = 1
} mspSDCardFlags_e;

typedef enum {
    MSP_FLASHFS_FLAG_READY       = 1,
    MSP_FLASHFS_FLAG_SUPPORTED  = 2
} mspFlashFsFlags_e;

typedef enum {
    MSP_PASSTHROUGH_ESC_SIMONK = PROTOCOL_SIMONK,
    MSP_PASSTHROUGH_ESC_BLHELI = PROTOCOL_BLHELI,
    MSP_PASSTHROUGH_ESC_KISS = PROTOCOL_KISS,
    MSP_PASSTHROUGH_ESC_KISSALL = PROTOCOL_KISSALL,
    MSP_PASSTHROUGH_ESC_CASTLE = PROTOCOL_CASTLE,

    MSP_PASSTHROUGH_SERIAL_ID = 0xFD,
    MSP_PASSTHROUGH_SERIAL_FUNCTION_ID = 0xFE,

    MSP_PASSTHROUGH_ESC_4WAY = 0xFF,
} mspPassthroughType_e;

#define RATEPROFILE_MASK (1 << 7)

#define RTC_NOT_SUPPORTED 0xff

typedef enum {
    DEFAULTS_TYPE_BASE = 0,
    DEFAULTS_TYPE_CUSTOM,
} defaultsType_e;

#ifdef USE_VTX_TABLE
static bool vtxTableNeedsInit = false;
#endif

static int mspDescriptor = 0;

mspDescriptor_t mspDescriptorAlloc(void)
{
    return (mspDescriptor_t)mspDescriptor++;
}

static uint32_t mspArmingDisableFlags = 0;

static void mspArmingDisableByDescriptor(mspDescriptor_t desc)
{
    mspArmingDisableFlags |= (1 << desc);
}

static void mspArmingEnableByDescriptor(mspDescriptor_t desc)
{
    mspArmingDisableFlags &= ~(1 << desc);
}

static bool mspIsMspArmingEnabled(void)
{
    return mspArmingDisableFlags == 0;
}

#define MSP_PASSTHROUGH_ESC_4WAY 0xff

static uint8_t mspPassthroughMode;
static uint8_t mspPassthroughArgument;

#ifdef USE_ESCSERIAL
static void mspEscPassthroughFn(serialPort_t *serialPort)
{
    escEnablePassthrough(serialPort, &motorConfig()->dev, mspPassthroughArgument, mspPassthroughMode);
}
#endif

static serialPort_t *mspFindPassthroughSerialPort(void)
{
    serialPortUsage_t *portUsage = NULL;

    switch (mspPassthroughMode) {
    case MSP_PASSTHROUGH_SERIAL_ID:
    {
        portUsage = findSerialPortUsageByIdentifier(mspPassthroughArgument);
        break;
    }
    case MSP_PASSTHROUGH_SERIAL_FUNCTION_ID:
    {
        const serialPortConfig_t *portConfig = findSerialPortConfig(1 << mspPassthroughArgument);
        if (portConfig) {
            portUsage = findSerialPortUsageByIdentifier(portConfig->identifier);
        }
        break;
    }
    }
    return portUsage ? portUsage->serialPort : NULL;
}

static void mspSerialPassthroughFn(serialPort_t *serialPort)
{
    serialPort_t *passthroughPort = mspFindPassthroughSerialPort();
    if (passthroughPort && serialPort) {
        serialPassthrough(passthroughPort, serialPort, NULL, NULL);
    }
}

static void mspFcSetPassthroughCommand(sbuf_t *dst, sbuf_t *src, mspPostProcessFnPtr *mspPostProcessFn)
{
    const unsigned int dataSize = sbufBytesRemaining(src);
    if (dataSize == 0) {
        // Legacy format
        mspPassthroughMode = MSP_PASSTHROUGH_ESC_4WAY;
    } else {
        mspPassthroughMode = sbufReadU8(src);
        mspPassthroughArgument = sbufReadU8(src);
    }

    switch (mspPassthroughMode) {
    case MSP_PASSTHROUGH_SERIAL_ID:
    case MSP_PASSTHROUGH_SERIAL_FUNCTION_ID:
        if (mspFindPassthroughSerialPort()) {
            if (mspPostProcessFn) {
                *mspPostProcessFn = mspSerialPassthroughFn;
            }
            sbufWriteU8(dst, 1);
        } else {
            sbufWriteU8(dst, 0);
        }
        break;
#ifdef USE_SERIAL_4WAY_BLHELI_INTERFACE
    case MSP_PASSTHROUGH_ESC_4WAY:
        // get channel number
        // switch all motor lines HI
        // reply with the count of ESC found
        sbufWriteU8(dst, esc4wayInit());

        if (mspPostProcessFn) {
            *mspPostProcessFn = esc4wayProcess;
        }
        break;

#ifdef USE_ESCSERIAL
    case MSP_PASSTHROUGH_ESC_SIMONK:
    case MSP_PASSTHROUGH_ESC_BLHELI:
    case MSP_PASSTHROUGH_ESC_KISS:
    case MSP_PASSTHROUGH_ESC_KISSALL:
    case MSP_PASSTHROUGH_ESC_CASTLE:
        if (mspPassthroughArgument < getMotorCount() || (mspPassthroughMode == MSP_PASSTHROUGH_ESC_KISS && mspPassthroughArgument == ALL_MOTORS)) {
            sbufWriteU8(dst, 1);

            if (mspPostProcessFn) {
                *mspPostProcessFn = mspEscPassthroughFn;
            }

            break;
        }
        FALLTHROUGH;
#endif // USE_ESCSERIAL
#endif //USE_SERIAL_4WAY_BLHELI_INTERFACE
    default:
        sbufWriteU8(dst, 0);
    }
}

// TODO: Remove the pragma once this is called from unconditional code
#pragma GCC diagnostic ignored "-Wunused-function"
static void configRebootUpdateCheckU8(uint8_t *parm, uint8_t value)
{
    if (*parm != value) {
        setRebootRequired();
    }
    *parm = value;
}
#pragma GCC diagnostic pop

static void mspRebootFn(serialPort_t *serialPort)
{
    UNUSED(serialPort);

    motorShutdown();

    switch (rebootMode) {
    case MSP_REBOOT_FIRMWARE:
        systemReset();

        break;
    case MSP_REBOOT_BOOTLOADER_ROM:
        systemResetToBootloader(BOOTLOADER_REQUEST_ROM);

        break;
#if defined(USE_USB_MSC)
    case MSP_REBOOT_MSC:
    case MSP_REBOOT_MSC_UTC: {
#ifdef USE_RTC_TIME
        const int16_t timezoneOffsetMinutes = (rebootMode == MSP_REBOOT_MSC) ? timeConfig()->tz_offsetMinutes : 0;
        systemResetToMsc(timezoneOffsetMinutes);
#else
        systemResetToMsc(0);
#endif
        }
        break;
#endif
#if defined(USE_FLASH_BOOT_LOADER)
    case MSP_REBOOT_BOOTLOADER_FLASH:
        systemResetToBootloader(BOOTLOADER_REQUEST_FLASH);

        break;
#endif
    default:

        return;
    }

    // control should never return here.
    while (true) ;
}

#define MSP_DISPATCH_DELAY_US 1000000

void mspReboot(dispatchEntry_t* self)
{
    UNUSED(self);

    if (ARMING_FLAG(ARMED)) {
        return;
    }

    mspRebootFn(NULL);
}

dispatchEntry_t mspRebootEntry =
{
    mspReboot, 0, NULL, false
};

void writeReadEeprom(dispatchEntry_t* self)
{
    UNUSED(self);

    if (ARMING_FLAG(ARMED)) {
        return;
    }

    writeEEPROM();
    readEEPROM();

#ifdef USE_VTX_TABLE
    if (vtxTableNeedsInit) {
        vtxTableNeedsInit = false;
        vtxTableInit();  // Reinitialize and refresh the in-memory copies
    }
#endif
}

dispatchEntry_t writeReadEepromEntry =
{
    writeReadEeprom, 0, NULL, false
};

static void serializeSDCardSummaryReply(sbuf_t *dst)
{
    uint8_t flags = 0;
    uint8_t state = 0;
    uint8_t lastError = 0;
    uint32_t freeSpace = 0;
    uint32_t totalSpace = 0;

#if defined(USE_SDCARD)
    if (sdcardConfig()->mode != SDCARD_MODE_NONE) {
        flags = MSP_SDCARD_FLAG_SUPPORTED;

        // Merge the card and filesystem states together
        if (!sdcard_isInserted()) {
            state = MSP_SDCARD_STATE_NOT_PRESENT;
        } else if (!sdcard_isFunctional()) {
            state = MSP_SDCARD_STATE_FATAL;
        } else {
            switch (afatfs_getFilesystemState()) {
            case AFATFS_FILESYSTEM_STATE_READY:
                state = MSP_SDCARD_STATE_READY;
                break;

            case AFATFS_FILESYSTEM_STATE_INITIALIZATION:
             if (sdcard_isInitialized()) {
                 state = MSP_SDCARD_STATE_FS_INIT;
             } else {
                 state = MSP_SDCARD_STATE_CARD_INIT;
             }
             break;

            case AFATFS_FILESYSTEM_STATE_FATAL:
            case AFATFS_FILESYSTEM_STATE_UNKNOWN:
            default:
                state = MSP_SDCARD_STATE_FATAL;
                break;
            }
        }

        lastError = afatfs_getLastError();
        // Write free space and total space in kilobytes
        if (state == MSP_SDCARD_STATE_READY) {
            freeSpace = afatfs_getContiguousFreeSpace() / 1024;
            totalSpace = sdcard_getMetadata()->numBlocks / 2;
        }
    }
#endif

    sbufWriteU8(dst, flags);
    sbufWriteU8(dst, state);
    sbufWriteU8(dst, lastError);
    sbufWriteU32(dst, freeSpace);
    sbufWriteU32(dst, totalSpace);
}

static void serializeDataflashSummaryReply(sbuf_t *dst)
{
#ifdef USE_FLASHFS
    if (flashfsIsSupported()) {
        uint8_t flags = MSP_FLASHFS_FLAG_SUPPORTED;
        flags |= (flashfsIsReady() ? MSP_FLASHFS_FLAG_READY : 0);

        const flashPartition_t *flashPartition = flashPartitionFindByType(FLASH_PARTITION_TYPE_FLASHFS);

        sbufWriteU8(dst, flags);
        sbufWriteU32(dst, FLASH_PARTITION_SECTOR_COUNT(flashPartition));
        sbufWriteU32(dst, flashfsGetSize());
        sbufWriteU32(dst, flashfsGetOffset()); // Effectively the current number of bytes stored on the volume
    } else
#endif

    // FlashFS is not configured or valid device is not detected
    {
        sbufWriteU8(dst, 0);
        sbufWriteU32(dst, 0);
        sbufWriteU32(dst, 0);
        sbufWriteU32(dst, 0);
    }
}

#ifdef USE_FLASHFS
enum compressionType_e {
    NO_COMPRESSION,
    HUFFMAN
};

static void serializeDataflashReadReply(sbuf_t *dst, uint32_t address, const uint16_t size, bool useLegacyFormat, bool allowCompression)
{
    STATIC_ASSERT(MSP_PORT_DATAFLASH_INFO_SIZE >= 16, MSP_PORT_DATAFLASH_INFO_SIZE_invalid);

    uint16_t readLen = size;
    const int bytesRemainingInBuf = sbufBytesRemaining(dst) - MSP_PORT_DATAFLASH_INFO_SIZE;
    if (readLen > bytesRemainingInBuf) {
        readLen = bytesRemainingInBuf;
    }
    // size will be lower than that requested if we reach end of volume
    const uint32_t flashfsSize = flashfsGetSize();
    if (readLen > flashfsSize - address) {
        // truncate the request
        readLen = flashfsSize - address;
    }
    sbufWriteU32(dst, address);

    // legacy format does not support compression
#ifdef USE_HUFFMAN
    const uint8_t compressionMethod = (!allowCompression || useLegacyFormat) ? NO_COMPRESSION : HUFFMAN;
#else
    const uint8_t compressionMethod = NO_COMPRESSION;
    UNUSED(allowCompression);
#endif

    if (compressionMethod == NO_COMPRESSION) {

        uint16_t *readLenPtr = (uint16_t *)sbufPtr(dst);
        if (!useLegacyFormat) {
            // new format supports variable read lengths
            sbufWriteU16(dst, readLen);
            sbufWriteU8(dst, 0); // placeholder for compression format
        }

        const int bytesRead = flashfsReadAbs(address, sbufPtr(dst), readLen);

        if (!useLegacyFormat) {
            // update the 'read length' with the actual amount read from flash.
            *readLenPtr = bytesRead;
        }

        sbufAdvance(dst, bytesRead);

        if (useLegacyFormat) {
            // pad the buffer with zeros
            for (int i = bytesRead; i < size; i++) {
                sbufWriteU8(dst, 0);
            }
        }
    } else {
#ifdef USE_HUFFMAN
        // compress in 256-byte chunks
        const uint16_t READ_BUFFER_SIZE = 256;
        // This may be DMAable, so make it cache aligned
        __attribute__ ((aligned(32))) uint8_t readBuffer[READ_BUFFER_SIZE];

        huffmanState_t state = {
            .bytesWritten = 0,
            .outByte = sbufPtr(dst) + sizeof(uint16_t) + sizeof(uint8_t) + HUFFMAN_INFO_SIZE,
            .outBufLen = readLen,
            .outBit = 0x80,
        };
        *state.outByte = 0;

        uint16_t bytesReadTotal = 0;
        // read until output buffer overflows or flash is exhausted
        while (state.bytesWritten < state.outBufLen && address + bytesReadTotal < flashfsSize) {
            const int bytesRead = flashfsReadAbs(address + bytesReadTotal, readBuffer,
                MIN(sizeof(readBuffer), flashfsSize - address - bytesReadTotal));

            const int status = huffmanEncodeBufStreaming(&state, readBuffer, bytesRead, huffmanTable);
            if (status == -1) {
                // overflow
                break;
            }

            bytesReadTotal += bytesRead;
        }

        if (state.outBit != 0x80) {
            ++state.bytesWritten;
        }

        // header
        sbufWriteU16(dst, HUFFMAN_INFO_SIZE + state.bytesWritten);
        sbufWriteU8(dst, compressionMethod);
        // payload
        sbufWriteU16(dst, bytesReadTotal);
        sbufAdvance(dst, state.bytesWritten);
#endif
    }
}
#endif // USE_FLASHFS

/*
 * Returns true if the command was processd, false otherwise.
 * May set mspPostProcessFunc to a function to be called once the command has been processed
 */
static bool mspCommonProcessOutCommand(int16_t cmdMSP, sbuf_t *dst, mspPostProcessFnPtr *mspPostProcessFn)
{
    UNUSED(mspPostProcessFn);

    switch (cmdMSP) {
    case MSP_API_VERSION:
        sbufWriteU8(dst, MSP_PROTOCOL_VERSION);
        sbufWriteU8(dst, API_VERSION_MAJOR);
        sbufWriteU8(dst, API_VERSION_MINOR);
        break;

    case MSP_FC_VARIANT:
        sbufWriteData(dst, flightControllerIdentifier, FLIGHT_CONTROLLER_IDENTIFIER_LENGTH);
        break;

    case MSP_FC_VERSION:
        sbufWriteU8(dst, FC_VERSION_MAJOR);
        sbufWriteU8(dst, FC_VERSION_MINOR);
        sbufWriteU8(dst, FC_VERSION_PATCH_LEVEL);
        break;

    case MSP_BOARD_INFO:
    {
        sbufWriteData(dst, systemConfig()->boardIdentifier, BOARD_IDENTIFIER_LENGTH);
#ifdef USE_HARDWARE_REVISION_DETECTION
        sbufWriteU16(dst, hardwareRevision);
#else
        sbufWriteU16(dst, 0); // No other build targets currently have hardware revision detection.
#endif
#if defined(USE_MAX7456)
        sbufWriteU8(dst, 2);  // 2 == FC with MAX7456
#else
        sbufWriteU8(dst, 0);  // 0 == FC
#endif

        // Target capabilities (uint8)
#define TARGET_HAS_VCP 0
#define TARGET_HAS_SOFTSERIAL 1
#define TARGET_IS_UNIFIED 2
#define TARGET_HAS_FLASH_BOOTLOADER 3
#define TARGET_SUPPORTS_CUSTOM_DEFAULTS 4
#define TARGET_HAS_CUSTOM_DEFAULTS 5
#define TARGET_SUPPORTS_RX_BIND 6

        uint8_t targetCapabilities = 0;
#ifdef USE_VCP
        targetCapabilities |= BIT(TARGET_HAS_VCP);
#endif
#if defined(USE_SOFTSERIAL1) || defined(USE_SOFTSERIAL2)
        targetCapabilities |= BIT(TARGET_HAS_SOFTSERIAL);
#endif
#if defined(USE_UNIFIED_TARGET)
        targetCapabilities |= BIT(TARGET_IS_UNIFIED);
#endif
#if defined(USE_FLASH_BOOT_LOADER)
        targetCapabilities |= BIT(TARGET_HAS_FLASH_BOOTLOADER);
#endif
#if defined(USE_CUSTOM_DEFAULTS)
        targetCapabilities |= BIT(TARGET_SUPPORTS_CUSTOM_DEFAULTS);
        if (hasCustomDefaults()) {
            targetCapabilities |= BIT(TARGET_HAS_CUSTOM_DEFAULTS);
        }
#endif
#if defined(USE_RX_BIND)
        if (getRxBindSupported()) {
            targetCapabilities |= BIT(TARGET_SUPPORTS_RX_BIND);
        }
#endif

        sbufWriteU8(dst, targetCapabilities);

        // Target name with explicit length
        sbufWriteU8(dst, strlen(targetName));
        sbufWriteData(dst, targetName, strlen(targetName));

#if defined(USE_BOARD_INFO)
        // Board name with explicit length
        char *value = getBoardName();
        sbufWriteU8(dst, strlen(value));
        sbufWriteString(dst, value);

        // Manufacturer id with explicit length
        value = getManufacturerId();
        sbufWriteU8(dst, strlen(value));
        sbufWriteString(dst, value);
#else
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);
#endif

#if defined(USE_SIGNATURE)
        // Signature
        sbufWriteData(dst, getSignature(), SIGNATURE_LENGTH);
#else
        uint8_t emptySignature[SIGNATURE_LENGTH];
        memset(emptySignature, 0, sizeof(emptySignature));
        sbufWriteData(dst, &emptySignature, sizeof(emptySignature));
#endif

        sbufWriteU8(dst, getMcuTypeId());

        // Added in API version 1.42
        sbufWriteU8(dst, systemConfig()->configurationState);

        // Added in API version 1.43
        sbufWriteU16(dst, gyro.sampleRateHz); // informational so the configurator can display the correct gyro/pid frequencies in the drop-down

        // Configuration warnings / problems (uint32_t)
#define PROBLEM_ACC_NEEDS_CALIBRATION 0
#define PROBLEM_MOTOR_PROTOCOL_DISABLED 1

        uint32_t configurationProblems = 0;

#if defined(USE_ACC)
        if (!accHasBeenCalibrated()) {
            configurationProblems |= BIT(PROBLEM_ACC_NEEDS_CALIBRATION);
        }
#endif

        if (!checkMotorProtocolEnabled(&motorConfig()->dev)) {
            configurationProblems |= BIT(PROBLEM_MOTOR_PROTOCOL_DISABLED);
        }

        sbufWriteU32(dst, configurationProblems);

        // Added in MSP API 1.44
#if defined(USE_SPI)
        sbufWriteU8(dst, spiGetRegisteredDeviceCount());
#else
        sbufWriteU8(dst, 0);
#endif
#if defined(USE_I2C)
        sbufWriteU8(dst, i2cGetRegisteredDeviceCount());
#else
        sbufWriteU8(dst, 0);
#endif

        break;
    }

    case MSP_BUILD_INFO:
        sbufWriteData(dst, buildDate, BUILD_DATE_LENGTH);
        sbufWriteData(dst, buildTime, BUILD_TIME_LENGTH);
        sbufWriteData(dst, shortGitRevision, GIT_SHORT_REVISION_LENGTH);
        const char *version = FC_VERSION_STRING;
        sbufWriteU8(dst, strlen(version));
        sbufWriteString(dst, version);
        break;

    case MSP_ANALOG:
        sbufWriteU8(dst, (uint8_t)constrain(getLegacyBatteryVoltage(), 0, 255));
        sbufWriteU16(dst, (uint16_t)constrain(getMAhDrawn(), 0, 0xFFFF)); // milliamp hours drawn from battery
        sbufWriteU16(dst, getRssi());
        sbufWriteU16(dst, (int16_t)constrain(getAmperage(), -0x8000, 0x7FFF)); // send current in 0.01 A steps, range is -320A to 320A
        sbufWriteU16(dst, getBatteryVoltage());
        break;

    case MSP_DEBUG:
        for (int i = 0; i < DEBUG_VALUE_COUNT; i++) {
            // RF TODO sbufWriteU32(dst, debug[i]);
            sbufWriteU16(dst, debug[i]);
        }
        break;

    case MSP_UID:
        sbufWriteU32(dst, U_ID_0);
        sbufWriteU32(dst, U_ID_1);
        sbufWriteU32(dst, U_ID_2);
        break;

    case MSP_FEATURE_CONFIG:
        sbufWriteU32(dst, featureConfig()->enabledFeatures);
        break;

#ifdef USE_BEEPER
    case MSP_BEEPER_CONFIG:
        sbufWriteU32(dst, beeperConfig()->beeper_off_flags);
        sbufWriteU8(dst, beeperConfig()->dshotBeaconTone);
        sbufWriteU32(dst, beeperConfig()->dshotBeaconOffFlags);
        break;
#endif

    case MSP_BATTERY_STATE: {
        // battery characteristics
        sbufWriteU8(dst, (uint8_t)constrain(getBatteryCellCount(), 0, 255)); // 0 indicates battery not detected.
        sbufWriteU16(dst, batteryConfig()->batteryCapacity); // in mAh

        // battery state
        sbufWriteU8(dst, (uint8_t)constrain(getLegacyBatteryVoltage(), 0, 255)); // in 0.1V steps
        sbufWriteU16(dst, (uint16_t)constrain(getMAhDrawn(), 0, 0xFFFF)); // milliamp hours drawn from battery
        sbufWriteU16(dst, (int16_t)constrain(getAmperage(), -0x8000, 0x7FFF)); // send current in 0.01 A steps, range is -320A to 320A

        // battery alerts
        sbufWriteU8(dst, (uint8_t)getBatteryState());

        sbufWriteU16(dst, getBatteryVoltage()); // in 0.01V steps
        break;
    }

    case MSP_VOLTAGE_METERS: {
        // write out id and voltage meter values, once for each meter we support
        uint8_t count = supportedVoltageMeterCount;
#ifdef USE_ESC_SENSOR
        count -= VOLTAGE_METER_ID_ESC_COUNT - getMotorCount();
#endif

        for (int i = 0; i < count; i++) {

            voltageMeter_t meter;
            uint8_t id = (uint8_t)voltageMeterIds[i];
            voltageMeterRead(id, &meter);

            sbufWriteU8(dst, id);
            sbufWriteU8(dst, (uint8_t)constrain((meter.displayFiltered + 5) / 10, 0, 255));
        }
        break;
    }

    case MSP_CURRENT_METERS: {
        // write out id and current meter values, once for each meter we support
        uint8_t count = supportedCurrentMeterCount;
#ifdef USE_ESC_SENSOR
        count -= VOLTAGE_METER_ID_ESC_COUNT - getMotorCount();
#endif
        for (int i = 0; i < count; i++) {

            currentMeter_t meter;
            uint8_t id = (uint8_t)currentMeterIds[i];
            currentMeterRead(id, &meter);

            sbufWriteU8(dst, id);
            sbufWriteU16(dst, (uint16_t)constrain(meter.mAhDrawn, 0, 0xFFFF)); // milliamp hours drawn from battery
            sbufWriteU16(dst, (uint16_t)constrain(meter.amperage * 10, 0, 0xFFFF)); // send amperage in 0.001 A steps (mA). Negative range is truncated to zero
        }
        break;
    }

    case MSP_VOLTAGE_METER_CONFIG:
        {
            // by using a sensor type and a sub-frame length it's possible to configure any type of voltage meter,
            // e.g. an i2c/spi/can sensor or any sensor not built directly into the FC such as ESC/RX/SPort/SBus that has
            // different configuration requirements.
            STATIC_ASSERT(VOLTAGE_SENSOR_ADC_VBAT == 0, VOLTAGE_SENSOR_ADC_VBAT_incorrect); // VOLTAGE_SENSOR_ADC_VBAT should be the first index
            sbufWriteU8(dst, MAX_VOLTAGE_SENSOR_ADC); // voltage meters in payload
            for (int i = VOLTAGE_SENSOR_ADC_VBAT; i < MAX_VOLTAGE_SENSOR_ADC; i++) {
                const uint8_t adcSensorSubframeLength = 1 + 1 + 1 + 1 + 1; // length of id, type, vbatscale, vbatresdivval, vbatresdivmultipler, in bytes
                sbufWriteU8(dst, adcSensorSubframeLength); // ADC sensor sub-frame length

                sbufWriteU8(dst, voltageMeterADCtoIDMap[i]); // id of the sensor
                sbufWriteU8(dst, VOLTAGE_SENSOR_TYPE_ADC_RESISTOR_DIVIDER); // indicate the type of sensor that the next part of the payload is for

                sbufWriteU8(dst, voltageSensorADCConfig(i)->vbatscale);
                sbufWriteU8(dst, voltageSensorADCConfig(i)->vbatresdivval);
                sbufWriteU8(dst, voltageSensorADCConfig(i)->vbatresdivmultiplier);
            }
            // if we had any other voltage sensors, this is where we would output any needed configuration
        }

        break;
    case MSP_CURRENT_METER_CONFIG: {
        sbufWriteU8(dst, 1);
        sbufWriteU8(dst, 1 + 1 + 2 + 2); // length of id, type, scale, offset, in bytes
        sbufWriteU8(dst, CURRENT_METER_ID_BATTERY_1); // the id of the meter
        sbufWriteU8(dst, CURRENT_SENSOR_ADC); // indicate the type of sensor that the next part of the payload is for
        sbufWriteU16(dst, currentSensorADCConfig()->scale);
        sbufWriteU16(dst, currentSensorADCConfig()->offset);

        // if we had any other current sensors, this is where we would output any needed configuration
        break;
    }

    case MSP_BATTERY_CONFIG:
        sbufWriteU8(dst, (batteryConfig()->vbatmincellvoltage + 5) / 10);
        sbufWriteU8(dst, (batteryConfig()->vbatmaxcellvoltage + 5) / 10);
        sbufWriteU8(dst, (batteryConfig()->vbatwarningcellvoltage + 5) / 10);
        sbufWriteU16(dst, batteryConfig()->batteryCapacity);
        sbufWriteU8(dst, batteryConfig()->voltageMeterSource);
        sbufWriteU8(dst, batteryConfig()->currentMeterSource);
        sbufWriteU16(dst, batteryConfig()->vbatmincellvoltage);
        sbufWriteU16(dst, batteryConfig()->vbatmaxcellvoltage);
        sbufWriteU16(dst, batteryConfig()->vbatwarningcellvoltage);
        break;

    case MSP_OSD_CONFIG: {
#define OSD_FLAGS_OSD_FEATURE           (1 << 0)
//#define OSD_FLAGS_OSD_SLAVE             (1 << 1)
#define OSD_FLAGS_RESERVED_1            (1 << 2)
#define OSD_FLAGS_OSD_HARDWARE_FRSKYOSD (1 << 3)
#define OSD_FLAGS_OSD_HARDWARE_MAX_7456 (1 << 4)
#define OSD_FLAGS_OSD_DEVICE_DETECTED   (1 << 5)

        uint8_t osdFlags = 0;
#if defined(USE_OSD)
        osdFlags |= OSD_FLAGS_OSD_FEATURE;

        osdDisplayPortDevice_e deviceType;
        displayPort_t *osdDisplayPort = osdGetDisplayPort(&deviceType);
        bool displayIsReady = osdDisplayPort && displayCheckReady(osdDisplayPort, true);
        switch (deviceType) {
        case OSD_DISPLAYPORT_DEVICE_MAX7456:
            osdFlags |= OSD_FLAGS_OSD_HARDWARE_MAX_7456;
            if (displayIsReady) {
                osdFlags |= OSD_FLAGS_OSD_DEVICE_DETECTED;
            }

            break;
        case OSD_DISPLAYPORT_DEVICE_FRSKYOSD:
            osdFlags |= OSD_FLAGS_OSD_HARDWARE_FRSKYOSD;
            if (displayIsReady) {
                osdFlags |= OSD_FLAGS_OSD_DEVICE_DETECTED;
            }

            break;
        default:
            break;
        }
#endif
        sbufWriteU8(dst, osdFlags);

#ifdef USE_MAX7456
        // send video system (AUTO/PAL/NTSC)
        sbufWriteU8(dst, vcdProfile()->video_system);
#else
        sbufWriteU8(dst, 0);
#endif

#ifdef USE_OSD
        // OSD specific, not applicable to OSD slaves.

        // Configuration
        sbufWriteU8(dst, osdConfig()->units);

        // Alarms
        sbufWriteU8(dst, osdConfig()->rssi_alarm);
        sbufWriteU16(dst, osdConfig()->cap_alarm);

        // Reuse old timer alarm (U16) as OSD_ITEM_COUNT
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, OSD_ITEM_COUNT);

        sbufWriteU16(dst, osdConfig()->alt_alarm);

        // Element position and visibility
        for (int i = 0; i < OSD_ITEM_COUNT; i++) {
            sbufWriteU16(dst, osdElementConfig()->item_pos[i]);
        }

        // Post flight statistics
        sbufWriteU8(dst, OSD_STAT_COUNT);
        for (int i = 0; i < OSD_STAT_COUNT; i++ ) {
            sbufWriteU8(dst, osdStatGetState(i));
        }

        // Timers
        sbufWriteU8(dst, OSD_TIMER_COUNT);
        for (int i = 0; i < OSD_TIMER_COUNT; i++) {
            sbufWriteU16(dst, osdConfig()->timers[i]);
        }

        // Enabled warnings
        // Send low word first for backwards compatibility (API < 1.41)
        sbufWriteU16(dst, (uint16_t)(osdConfig()->enabledWarnings & 0xFFFF));
        // API >= 1.41
        // Send the warnings count and 32bit enabled warnings flags.
        // Add currently active OSD profile (0 indicates OSD profiles not available).
        // Add OSD stick overlay mode (0 indicates OSD stick overlay not available).
        sbufWriteU8(dst, OSD_WARNING_COUNT);
        sbufWriteU32(dst, osdConfig()->enabledWarnings);

#ifdef USE_OSD_PROFILES
        sbufWriteU8(dst, OSD_PROFILE_COUNT);            // available profiles
        sbufWriteU8(dst, osdConfig()->osdProfileIndex); // selected profile
#else
        // If the feature is not available there is only 1 profile and it's always selected
        sbufWriteU8(dst, 1);
        sbufWriteU8(dst, 1);
#endif // USE_OSD_PROFILES

#ifdef USE_OSD_STICK_OVERLAY
        sbufWriteU8(dst, osdConfig()->overlay_radio_mode);
#else
        sbufWriteU8(dst, 0);
#endif // USE_OSD_STICK_OVERLAY

        // API >= 1.43
        // Add the camera frame element width/height
        sbufWriteU8(dst, osdConfig()->camera_frame_width);
        sbufWriteU8(dst, osdConfig()->camera_frame_height);

#endif // USE_OSD
        break;
    }

    default:
        return false;
    }
    return true;
}

static bool mspProcessOutCommand(int16_t cmdMSP, sbuf_t *dst)
{
    bool unsupportedCommand = false;

    switch (cmdMSP) {
    case MSP_STATUS:
        {
            sbufWriteU16(dst, getTaskDeltaTimeUs(TASK_PID));
            sbufWriteU16(dst, getTaskDeltaTimeUs(TASK_GYRO));

            sbufWriteU16(dst, sensors(SENSOR_ACC) << 0 |
                              sensors(SENSOR_BARO) << 1 |
                              sensors(SENSOR_MAG) << 2 |
                              sensors(SENSOR_GPS) << 3 |
                              sensors(SENSOR_RANGEFINDER) << 4 |
                              sensors(SENSOR_GYRO) << 5);

            boxBitmask_t flightModeFlags;
            packFlightModeFlags(&flightModeFlags);
            sbufWriteData(dst, &flightModeFlags, 4);

            sbufWriteU8(dst, 0); // compat: profile number

            sbufWriteU16(dst, getAverageSystemLoadPercent());
            sbufWriteU16(dst, 0); // getAverageCPULoadPercent());

            sbufWriteU8(dst, 0); // compat: extra flight mode flags count

            sbufWriteU8(dst, ARMING_DISABLE_FLAGS_COUNT);
            sbufWriteU32(dst, getArmingDisableFlags());

            sbufWriteU8(dst, getRebootRequired());
            sbufWriteU8(dst, systemConfig()->configurationState);

            sbufWriteU8(dst, getCurrentPidProfileIndex());
            sbufWriteU8(dst, PID_PROFILE_COUNT);
            sbufWriteU8(dst, getCurrentControlRateProfileIndex());
            sbufWriteU8(dst, CONTROL_RATE_PROFILE_COUNT);

            sbufWriteU8(dst, getMotorCount());
#ifdef USE_SERVOS
            sbufWriteU8(dst, getServoCount());
#else
            sbufWriteU8(dst, 0);
#endif
            sbufWriteU8(dst, getGyroDetectionFlags());
        }
        break;

    case MSP_RAW_IMU:
        {
#if defined(USE_ACC)
            // Hack scale due to choice of units for sensor data in multiwii

            uint8_t scale;
            if (acc.dev.acc_1G > 512 * 4) {
                scale = 8;
            } else if (acc.dev.acc_1G > 512 * 2) {
                scale = 4;
            } else if (acc.dev.acc_1G >= 512) {
                scale = 2;
            } else {
                scale = 1;
            }
#endif

            for (int i = 0; i < 3; i++) {
#if defined(USE_ACC)
                sbufWriteU16(dst, lrintf(acc.accADC[i] / scale));
#else
                sbufWriteU16(dst, 0);
#endif
            }
            for (int i = 0; i < 3; i++) {
                sbufWriteU16(dst, gyroRateDps(i));
            }
            for (int i = 0; i < 3; i++) {
#if defined(USE_MAG)
                sbufWriteU16(dst, lrintf(mag.magADC[i]));
#else
                sbufWriteU16(dst, 0);
#endif
            }
        }
        break;

    case MSP_NAME:
        {
            const int nameLen = strlen(pilotConfig()->name);
            for (int i = 0; i < nameLen; i++) {
                sbufWriteU8(dst, pilotConfig()->name[i]);
            }
        }
        break;

#ifdef USE_SERVOS
    case MSP_SERVO:
        for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
            sbufWriteU16(dst, getServoOutput(i));
        }
        break;

    case MSP_SERVO_CONFIGURATIONS:
        for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
            sbufWriteU16(dst, servoParams(i)->mid);
            sbufWriteU16(dst, servoParams(i)->min);
            sbufWriteU16(dst, servoParams(i)->max);
            sbufWriteU16(dst, servoParams(i)->rate);
            sbufWriteU16(dst, servoParams(i)->trim);
            sbufWriteU16(dst, servoParams(i)->speed);
        }
        break;

    case MSP_SERVO_OVERRIDE:
        for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
            sbufWriteU16(dst, getServoOverride(i));
        }
        break;
#endif

    case MSP_MOTOR:
        for (int i = 0; i < 8; i++) {
#ifdef USE_MOTOR
            if (i < getMotorCount() && motorIsEnabled() && motorIsMotorEnabled(i)) {
                const int16_t throttle = getMotorOutput(i);
                if (throttle < 0)
                    sbufWriteU16(dst, throttle - 1000);
                else
                    sbufWriteU16(dst, throttle + 1000); // compat: BLHeli
            }
            else
#endif
                sbufWriteU16(dst, 0); // zero means motor disabled
        }
        break;

    case MSP_MOTOR_OVERRIDE:
        for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
#ifdef USE_MOTOR
            if (i < getMotorCount())
                sbufWriteU16(dst, getMotorOverride(i));
            else
#endif
                sbufWriteU16(dst, 0);
        }
        break;

    case MSP_MOTOR_TELEMETRY:
        sbufWriteU8(dst, getMotorCount());
        for (unsigned i = 0; i < getMotorCount(); i++) {
            uint32_t motorRpm = 0;
            uint16_t errorRatio = 0;
            uint8_t escTemperature = 0;  // degrees celcius
            uint16_t escVoltage = 0;     // 0.01V per unit
            uint16_t escCurrent = 0;     // 0.01A per unit
            uint16_t escConsumption = 0; // mAh

#ifdef USE_ESC_SENSOR
            if (featureIsEnabled(FEATURE_ESC_SENSOR)) {
                escSensorData_t *escData = getEscSensorData(i);
                motorRpm = calcMotorRPM(i,escData->rpm);
                escTemperature = escData->temperature;
                escVoltage = escData->voltage;
                escCurrent = escData->current;
                escConsumption = escData->consumption;
            }
#endif

#ifdef USE_DSHOT_TELEMETRY
            if (motorConfig()->dev.useDshotTelemetry) {
                if (isDshotMotorTelemetryActive(i)) {
                    motorRpm = calcMotorRPM(i, getDshotTelemetry(i));
#ifdef USE_DSHOT_TELEMETRY_STATS
                    errorRatio = getDshotTelemetryMotorInvalidPercent(i);
#endif
                }
            }
#endif

#ifdef USE_FREQ_SENSOR
            if (featureIsEnabled(FEATURE_FREQ_SENSOR) && isFreqSensorPortInitialized(i)) {
                motorRpm = calcMotorRPM(i, getFreqSensorRPM(i));
            }
#endif

            if (isMotorRpmSourceActive(i))
                motorRpm = getMotorRPM(i);

            sbufWriteU32(dst, motorRpm);
            sbufWriteU16(dst, errorRatio);
            sbufWriteU8(dst, escTemperature);
            sbufWriteU16(dst, escVoltage);
            sbufWriteU16(dst, escCurrent);
            sbufWriteU16(dst, escConsumption);
        }
        break;

#ifdef USE_VTX_COMMON
    case MSP2_GET_VTX_DEVICE_STATUS:
        {
            const vtxDevice_t *vtxDevice = vtxCommonDevice();
            vtxCommonSerializeDeviceStatus(vtxDevice, dst);
        }
        break;
#endif

#ifdef USE_OSD
    case MSP2_GET_OSD_WARNINGS:
        {
            bool isBlinking;
            uint8_t displayAttr;
            char warningsBuffer[OSD_FORMAT_MESSAGE_BUFFER_SIZE];

            renderOsdWarning(warningsBuffer, &isBlinking, &displayAttr);
            const uint8_t warningsLen = strlen(warningsBuffer);

            if (isBlinking) {
                displayAttr |= DISPLAYPORT_ATTR_BLINK;
            }
            sbufWriteU8(dst, displayAttr);  // see displayPortAttr_e
            sbufWriteU8(dst, warningsLen);  // length byte followed by the actual characters
            for (unsigned i = 0; i < warningsLen; i++) {
                sbufWriteU8(dst, warningsBuffer[i]);
            }
            break;
        }
#endif

    case MSP_RC:
        for (int i = 0; i < rxRuntimeState.channelCount; i++) {
            sbufWriteU16(dst, rcData[i]);
        }
        break;

    case MSP_ATTITUDE:
        sbufWriteU16(dst, attitude.values.roll);
        sbufWriteU16(dst, attitude.values.pitch);
        sbufWriteU16(dst, DECIDEGREES_TO_DEGREES(attitude.values.yaw));
        break;

    case MSP_ALTITUDE:
        sbufWriteU32(dst, getEstimatedAltitudeCm());
#ifdef USE_VARIO
        sbufWriteU16(dst, getEstimatedVario());
#else
        sbufWriteU16(dst, 0);
#endif
        break;

    case MSP_SONAR_ALTITUDE:
#if defined(USE_RANGEFINDER)
        sbufWriteU32(dst, rangefinderGetLatestAltitude());
#else
        sbufWriteU32(dst, 0);
#endif
        break;

    case MSP_BOARD_ALIGNMENT_CONFIG:
        sbufWriteU16(dst, boardAlignment()->rollDegrees);
        sbufWriteU16(dst, boardAlignment()->pitchDegrees);
        sbufWriteU16(dst, boardAlignment()->yawDegrees);
        break;

    case MSP_DEBUG_CONFIG:
        sbufWriteU8(dst, DEBUG_COUNT);
        sbufWriteU8(dst, systemConfig()->debug_mode);
        sbufWriteU8(dst, systemConfig()->debug_axis);
        break;

    case MSP_ARMING_CONFIG:
        sbufWriteU8(dst, armingConfig()->auto_disarm_delay);
        break;

    case MSP_RC_TUNING:
        sbufWriteU8(dst, currentControlRateProfile->rates_type);
        for (int i = 0; i < 3; i++) {
            sbufWriteU8(dst, currentControlRateProfile->rcRates[i]);
            sbufWriteU8(dst, currentControlRateProfile->rcExpo[i]);
            sbufWriteU8(dst, currentControlRateProfile->rates[i]);
            sbufWriteU16(dst, currentControlRateProfile->rate_limit[i]);
        }
        break;

    case MSP_PID:
        for (int i = 0; i < PID_AXIS_COUNT; i++) {
            sbufWriteU16(dst, currentPidProfile->pid[i].P);
            sbufWriteU16(dst, currentPidProfile->pid[i].I);
            sbufWriteU16(dst, currentPidProfile->pid[i].D);
            sbufWriteU16(dst, currentPidProfile->pid[i].F);
        }
        break;

    case MSP_MODE_RANGES:
        for (int i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
            const modeActivationCondition_t *mac = modeActivationConditions(i);
            const box_t *box = findBoxByBoxId(mac->modeId);
            sbufWriteU8(dst, box->permanentId);
            sbufWriteU8(dst, mac->auxChannelIndex);
            sbufWriteU8(dst, mac->range.startStep);
            sbufWriteU8(dst, mac->range.endStep);
        }
        break;

    case MSP_MODE_RANGES_EXTRA:
        sbufWriteU8(dst, MAX_MODE_ACTIVATION_CONDITION_COUNT);          // prepend number of EXTRAs array elements

        for (int i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
            const modeActivationCondition_t *mac = modeActivationConditions(i);
            const box_t *box = findBoxByBoxId(mac->modeId);
            const box_t *linkedBox = findBoxByBoxId(mac->linkedTo);
            sbufWriteU8(dst, box->permanentId);     // each element is aligned with MODE_RANGES by the permanentId
            sbufWriteU8(dst, mac->modeLogic);
            sbufWriteU8(dst, linkedBox->permanentId);
        }
        break;

    case MSP_ADJUSTMENT_RANGES:
        break;

    case MSP_MOTOR_CONFIG:
        sbufWriteU16(dst, motorConfig()->minthrottle);
        sbufWriteU16(dst, motorConfig()->maxthrottle);
        sbufWriteU16(dst, motorConfig()->mincommand);

        sbufWriteU8(dst, getMotorCount()); // compat: BLHeliSuite
        sbufWriteU8(dst, motorConfig()->motorPoleCount[0]); // compat: BLHeliSuite

#ifdef USE_DSHOT_TELEMETRY
        sbufWriteU8(dst, motorConfig()->dev.useDshotTelemetry);
#else
        sbufWriteU8(dst, 0);
#endif
        sbufWriteU8(dst, motorConfig()->dev.motorPwmProtocol);
        sbufWriteU16(dst, motorConfig()->dev.motorPwmRate);
        sbufWriteU8(dst, motorConfig()->dev.motorPwmInversion);
        sbufWriteU8(dst, motorConfig()->dev.useUnsyncedPwm);

        for (int i = 0; i < 4; i++)
            sbufWriteU8(dst, motorConfig()->motorPoleCount[i]);
        for (int i = 0; i < 4; i++)
            sbufWriteU8(dst, motorConfig()->motorRpmLpf[i]);

        sbufWriteU16(dst, motorConfig()->mainRotorGearRatio[0]);
        sbufWriteU16(dst, motorConfig()->mainRotorGearRatio[1]);
        sbufWriteU16(dst, motorConfig()->tailRotorGearRatio[0]);
        sbufWriteU16(dst, motorConfig()->tailRotorGearRatio[1]);
        break;

#if defined(USE_ESC_SENSOR)
    // Deprecated in favor of MSP_MOTOR_TELEMETY as of API version 1.42
    case MSP_ESC_SENSOR_DATA:
        if (featureIsEnabled(FEATURE_ESC_SENSOR)) {
            sbufWriteU8(dst, getMotorCount());
            for (int i = 0; i < getMotorCount(); i++) {
                const escSensorData_t *escData = getEscSensorData(i);
                sbufWriteU8(dst, escData->temperature);
                sbufWriteU16(dst, escData->rpm);
            }
        } else {
            unsupportedCommand = true;
        }

        break;
#endif

#ifdef USE_GPS
    case MSP_GPS_CONFIG:
        sbufWriteU8(dst, gpsConfig()->provider);
        sbufWriteU8(dst, gpsConfig()->sbasMode);
        sbufWriteU8(dst, gpsConfig()->autoConfig);
        sbufWriteU8(dst, gpsConfig()->autoBaud);
        // Added in API version 1.43
        sbufWriteU8(dst, gpsConfig()->gps_set_home_point_once);
        sbufWriteU8(dst, gpsConfig()->gps_ublox_use_galileo);
        break;

    case MSP_RAW_GPS:
        sbufWriteU8(dst, STATE(GPS_FIX));
        sbufWriteU8(dst, gpsSol.numSat);
        sbufWriteU32(dst, gpsSol.llh.lat);
        sbufWriteU32(dst, gpsSol.llh.lon);
        sbufWriteU16(dst, (uint16_t)constrain(gpsSol.llh.altCm / 100, 0, UINT16_MAX)); // alt changed from 1m to 0.01m per lsb since MSP API 1.39 by RTH. To maintain backwards compatibility compensate to 1m per lsb in MSP again.
        sbufWriteU16(dst, gpsSol.groundSpeed);
        sbufWriteU16(dst, gpsSol.groundCourse);
        // Added in API version 1.44
        sbufWriteU16(dst, gpsSol.hdop);
        break;

    case MSP_COMP_GPS:
        sbufWriteU16(dst, GPS_distanceToHome);
        sbufWriteU16(dst, GPS_directionToHome);
        sbufWriteU8(dst, GPS_update & 1);
        break;

    case MSP_GPSSVINFO:
        sbufWriteU8(dst, GPS_numCh);
       for (int i = 0; i < GPS_numCh; i++) {
           sbufWriteU8(dst, GPS_svinfo_chn[i]);
           sbufWriteU8(dst, GPS_svinfo_svid[i]);
           sbufWriteU8(dst, GPS_svinfo_quality[i]);
           sbufWriteU8(dst, GPS_svinfo_cno[i]);
       }
        break;

#ifdef USE_GPS_RESCUE
    case MSP_GPS_RESCUE:
        sbufWriteU16(dst, gpsRescueConfig()->angle);
        sbufWriteU16(dst, gpsRescueConfig()->initialAltitudeM);
        sbufWriteU16(dst, gpsRescueConfig()->descentDistanceM);
        sbufWriteU16(dst, gpsRescueConfig()->rescueGroundspeed);
        sbufWriteU16(dst, gpsRescueConfig()->throttleMin);
        sbufWriteU16(dst, gpsRescueConfig()->throttleMax);
        sbufWriteU16(dst, gpsRescueConfig()->throttleHover);
        sbufWriteU8(dst,  gpsRescueConfig()->sanityChecks);
        sbufWriteU8(dst,  gpsRescueConfig()->minSats);
        // Added in API version 1.43
        sbufWriteU16(dst, gpsRescueConfig()->ascendRate);
        sbufWriteU16(dst, gpsRescueConfig()->descendRate);
        sbufWriteU8(dst, gpsRescueConfig()->allowArmingWithoutFix);
        sbufWriteU8(dst, gpsRescueConfig()->altitudeMode);
        // Added in API version 1.44
        sbufWriteU16(dst, gpsRescueConfig()->minRescueDth);
        break;

    case MSP_GPS_RESCUE_PIDS:
        sbufWriteU16(dst, gpsRescueConfig()->throttleP);
        sbufWriteU16(dst, gpsRescueConfig()->throttleI);
        sbufWriteU16(dst, gpsRescueConfig()->throttleD);
        sbufWriteU16(dst, gpsRescueConfig()->velP);
        sbufWriteU16(dst, gpsRescueConfig()->velI);
        sbufWriteU16(dst, gpsRescueConfig()->velD);
        sbufWriteU16(dst, gpsRescueConfig()->yawP);
        break;
#endif
#endif

#if defined(USE_ACC)
    case MSP_ACC_TRIM:
        sbufWriteU16(dst, accelerometerConfig()->accelerometerTrims.values.pitch);
        sbufWriteU16(dst, accelerometerConfig()->accelerometerTrims.values.roll);

        break;
#endif
    case MSP_MIXER_CONFIG:
        sbufWriteU8(dst, mixerConfig()->main_rotor_dir);
        sbufWriteU8(dst, mixerConfig()->tail_rotor_mode);
        sbufWriteU8(dst, mixerConfig()->tail_motor_idle);
        sbufWriteU8(dst, mixerConfig()->swash_ring);
        sbufWriteU8(dst, mixerConfig()->swash_phase);
        break;

    case MSP_MIXER_INPUTS:
        for (int i = 0; i < MIXER_INPUT_COUNT; i++) {
          sbufWriteU16(dst, mixerInputs(i)->rate);
          sbufWriteU16(dst, mixerInputs(i)->min);
          sbufWriteU16(dst, mixerInputs(i)->max);
        }
        break;

    case MSP_MIXER_RULES:
        for (int i = 0; i < MIXER_RULE_COUNT; i++) {
          sbufWriteU32(dst, 0); // RF TODO remove me
          sbufWriteU8(dst, mixerRules(i)->oper);
          sbufWriteU8(dst, mixerRules(i)->input);
          sbufWriteU8(dst, mixerRules(i)->output);
          sbufWriteU16(dst, mixerRules(i)->offset);
          sbufWriteU16(dst, mixerRules(i)->weight);
        }
        break;

    case MSP_MIXER_OVERRIDE:
        for (int i = 0; i < MIXER_INPUT_COUNT; i++) {
            sbufWriteU16(dst, mixerGetOverride(i));
        }
        break;

    case MSP_RX_CONFIG:
        sbufWriteU8(dst, rxConfig()->serialrx_provider);
        sbufWriteU8(dst, rxConfig()->serialrx_inverted);
        sbufWriteU8(dst, rxConfig()->halfDuplex);
        sbufWriteU16(dst, rxConfig()->maxcheck);
        sbufWriteU16(dst, rxConfig()->midrc);
        sbufWriteU16(dst, rxConfig()->mincheck);
        sbufWriteU16(dst, rxConfig()->rx_min_usec);
        sbufWriteU16(dst, rxConfig()->rx_max_usec);
        sbufWriteU8(dst, 0); // RF TODO rxConfig()->rcInterpolation
        sbufWriteU8(dst, 0); // RF TODO rxConfig()->rcInterpolationInterval
        sbufWriteU8(dst, 0); // RF TODO rxConfig()->rcInterpolationChannels
        sbufWriteU8(dst, 0); // RF TODO rxConfig()->rc_smoothing_mode
        sbufWriteU8(dst, 0); // RF TODO rxConfig()->rc_smoothing_input_type
        sbufWriteU8(dst, 0); // RF TODO  rxConfig()->rc_smoothing_input_cutoff
        sbufWriteU8(dst, 0); // RF TODO  rxConfig()->rc_smoothing_derivative_type
        sbufWriteU8(dst, 0); // RF TODO rxConfig()->rc_smoothing_derivative_cutoff
        sbufWriteU8(dst, 0); // RF TODO rxConfig()->rc_smoothing_factor
#ifdef USE_RX_SPI
        sbufWriteU8(dst, rxSpiConfig()->rx_spi_protocol);
        sbufWriteU32(dst, rxSpiConfig()->rx_spi_id);
        sbufWriteU8(dst, rxSpiConfig()->rx_spi_rf_channel_count);
#else
        sbufWriteU8(dst, 0);
        sbufWriteU32(dst, 0);
        sbufWriteU8(dst, 0);
#endif
        break;

    case MSP_FAILSAFE_CONFIG:
        sbufWriteU8(dst, failsafeConfig()->failsafe_delay);
        sbufWriteU8(dst, failsafeConfig()->failsafe_off_delay);
        sbufWriteU16(dst, failsafeConfig()->failsafe_throttle);
        sbufWriteU8(dst, failsafeConfig()->failsafe_switch_mode);
        sbufWriteU16(dst, failsafeConfig()->failsafe_throttle_low_delay);
        sbufWriteU8(dst, failsafeConfig()->failsafe_procedure);
        break;

    case MSP_RXFAIL_CONFIG:
        for (int i = 0; i < rxRuntimeState.channelCount; i++) {
            sbufWriteU8(dst, rxFailsafeChannelConfigs(i)->mode);
            sbufWriteU16(dst, RXFAIL_STEP_TO_CHANNEL_VALUE(rxFailsafeChannelConfigs(i)->step));
        }
        break;

    case MSP_RSSI_CONFIG:
        sbufWriteU8(dst, rxConfig()->rssi_channel);
        sbufWriteU8(dst, rxConfig()->rssi_scale);
        sbufWriteU8(dst, rxConfig()->rssi_invert);
        sbufWriteU8(dst, rxConfig()->rssi_offset);
        break;

    case MSP_RX_MAP:
        sbufWriteData(dst, rxConfig()->rcmap, RX_MAPPABLE_CHANNEL_COUNT);
        break;

    case MSP_CF_SERIAL_CONFIG:
        for (int i = 0; i < SERIAL_PORT_COUNT; i++) {
            if (!serialIsPortAvailable(serialConfig()->portConfigs[i].identifier)) {
                continue;
            };
            sbufWriteU8(dst, serialConfig()->portConfigs[i].identifier);
            sbufWriteU16(dst, serialConfig()->portConfigs[i].functionMask);
            sbufWriteU8(dst, serialConfig()->portConfigs[i].msp_baudrateIndex);
            sbufWriteU8(dst, serialConfig()->portConfigs[i].gps_baudrateIndex);
            sbufWriteU8(dst, serialConfig()->portConfigs[i].telemetry_baudrateIndex);
            sbufWriteU8(dst, serialConfig()->portConfigs[i].blackbox_baudrateIndex);
        }
        break;
    case MSP2_COMMON_SERIAL_CONFIG: {
        uint8_t count = 0;
        for (int i = 0; i < SERIAL_PORT_COUNT; i++) {
            if (serialIsPortAvailable(serialConfig()->portConfigs[i].identifier)) {
                count++;
            }
        }
        sbufWriteU8(dst, count);
        for (int i = 0; i < SERIAL_PORT_COUNT; i++) {
            if (!serialIsPortAvailable(serialConfig()->portConfigs[i].identifier)) {
                continue;
            };
            sbufWriteU8(dst, serialConfig()->portConfigs[i].identifier);
            sbufWriteU32(dst, serialConfig()->portConfigs[i].functionMask);
            sbufWriteU8(dst, serialConfig()->portConfigs[i].msp_baudrateIndex);
            sbufWriteU8(dst, serialConfig()->portConfigs[i].gps_baudrateIndex);
            sbufWriteU8(dst, serialConfig()->portConfigs[i].telemetry_baudrateIndex);
            sbufWriteU8(dst, serialConfig()->portConfigs[i].blackbox_baudrateIndex);
        }
        break;
    }

#ifdef USE_LED_STRIP_STATUS_MODE
    case MSP_LED_COLORS:
        for (int i = 0; i < LED_CONFIGURABLE_COLOR_COUNT; i++) {
            const hsvColor_t *color = &ledStripStatusModeConfig()->colors[i];
            sbufWriteU16(dst, color->h);
            sbufWriteU8(dst, color->s);
            sbufWriteU8(dst, color->v);
        }
        break;
#endif

#ifdef USE_LED_STRIP
    case MSP_LED_STRIP_CONFIG:
        for (int i = 0; i < LED_MAX_STRIP_LENGTH; i++) {
#ifdef USE_LED_STRIP_STATUS_MODE
            const ledConfig_t *ledConfig = &ledStripStatusModeConfig()->ledConfigs[i];
            sbufWriteU32(dst, *ledConfig);
#else
            sbufWriteU32(dst, 0);
#endif
        }

        // API 1.41 - add indicator for advanced profile support and the current profile selection
        // 0 = basic ledstrip available
        // 1 = advanced ledstrip available
#ifdef USE_LED_STRIP_STATUS_MODE
        sbufWriteU8(dst, 1);   // advanced ledstrip available
#else
        sbufWriteU8(dst, 0);   // only simple ledstrip available
#endif
        sbufWriteU8(dst, ledStripConfig()->ledstrip_profile);
        break;
#endif

#ifdef USE_LED_STRIP_STATUS_MODE
    case MSP_LED_STRIP_MODECOLOR:
        for (int i = 0; i < LED_MODE_COUNT; i++) {
            for (int j = 0; j < LED_DIRECTION_COUNT; j++) {
                sbufWriteU8(dst, i);
                sbufWriteU8(dst, j);
                sbufWriteU8(dst, ledStripStatusModeConfig()->modeColors[i].color[j]);
            }
        }

        for (int j = 0; j < LED_SPECIAL_COLOR_COUNT; j++) {
            sbufWriteU8(dst, LED_MODE_COUNT);
            sbufWriteU8(dst, j);
            sbufWriteU8(dst, ledStripStatusModeConfig()->specialColors.color[j]);
        }

        sbufWriteU8(dst, LED_AUX_CHANNEL);
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, ledStripStatusModeConfig()->ledstrip_aux_channel);
        break;
#endif

    case MSP_DATAFLASH_SUMMARY:
        serializeDataflashSummaryReply(dst);
        break;

    case MSP_BLACKBOX_CONFIG:
#ifdef USE_BLACKBOX
        sbufWriteU8(dst, 1); //Blackbox supported
        sbufWriteU8(dst, blackboxConfig()->device);
        sbufWriteU8(dst, 1); // Rate numerator, not used anymore
        sbufWriteU8(dst, blackboxGetRateDenom());
        sbufWriteU16(dst, blackboxGetPRatio());
        sbufWriteU8(dst, blackboxConfig()->sample_rate);
#else
        sbufWriteU8(dst, 0); // Blackbox not supported
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);
        sbufWriteU16(dst, 0);
        sbufWriteU8(dst, 0);
#endif
        break;

    case MSP_SDCARD_SUMMARY:
        serializeSDCardSummaryReply(dst);
        break;

    case MSP_RC_DEADBAND:
        sbufWriteU8(dst, rcControlsConfig()->deadband);
        sbufWriteU8(dst, rcControlsConfig()->yaw_deadband);
        sbufWriteU8(dst, rcControlsConfig()->alt_hold_deadband);
        sbufWriteU16(dst, 0); // was flight3DConfig()->deadband3d_throttle
        break;


    case MSP_SENSOR_ALIGNMENT: {
        uint8_t gyroAlignment;
#ifdef USE_MULTI_GYRO
        switch (gyroConfig()->gyro_to_use) {
        case GYRO_CONFIG_USE_GYRO_2:
            gyroAlignment = gyroDeviceConfig(1)->alignment;
            break;
        case GYRO_CONFIG_USE_GYRO_BOTH:
            // for dual-gyro in "BOTH" mode we only read/write gyro 0
        default:
            gyroAlignment = gyroDeviceConfig(0)->alignment;
            break;
        }
#else
        gyroAlignment = gyroDeviceConfig(0)->alignment;
#endif
        sbufWriteU8(dst, gyroAlignment);
        sbufWriteU8(dst, gyroAlignment);  // Starting with 4.0 gyro and acc alignment are the same
#if defined(USE_MAG)
        sbufWriteU8(dst, compassConfig()->mag_alignment);
#else
        sbufWriteU8(dst, 0);
#endif

        // API 1.41 - Add multi-gyro indicator, selected gyro, and support for separate gyro 1 & 2 alignment
        sbufWriteU8(dst, getGyroDetectionFlags());
#ifdef USE_MULTI_GYRO
        sbufWriteU8(dst, gyroConfig()->gyro_to_use);
        sbufWriteU8(dst, gyroDeviceConfig(0)->alignment);
        sbufWriteU8(dst, gyroDeviceConfig(1)->alignment);
#else
        sbufWriteU8(dst, GYRO_CONFIG_USE_GYRO_1);
        sbufWriteU8(dst, gyroDeviceConfig(0)->alignment);
        sbufWriteU8(dst, ALIGN_DEFAULT);
#endif

        break;
    }
    case MSP_ADVANCED_CONFIG:
        sbufWriteU8(dst, 1); // compat: gyro denom
        sbufWriteU8(dst, pidConfig()->pid_process_denom);
        sbufWriteU8(dst, motorConfig()->dev.useUnsyncedPwm);
        sbufWriteU8(dst, motorConfig()->dev.motorPwmProtocol);
        sbufWriteU16(dst, motorConfig()->dev.motorPwmRate);
        sbufWriteU16(dst, servoConfig()->dev.servoPwmRate);
        sbufWriteU32(dst, 0); // compat: deprecated
        sbufWriteU32(dst, 0);
        sbufWriteU32(dst, 0);
        break;

    case MSP_FILTER_CONFIG :
        sbufWriteU8(dst, gyroConfig()->gyro_lpf1_static_hz);
        sbufWriteU16(dst, gyroConfig()->dterm_lpf1_static_hz);
        sbufWriteU16(dst, 0); // was currentPidProfile->yaw_lowpass_hz
        sbufWriteU16(dst, gyroConfig()->gyro_soft_notch_hz_1);
        sbufWriteU16(dst, gyroConfig()->gyro_soft_notch_cutoff_1);
        sbufWriteU16(dst, gyroConfig()->dterm_notch_hz);
        sbufWriteU16(dst, gyroConfig()->dterm_notch_cutoff);
        sbufWriteU16(dst, gyroConfig()->gyro_soft_notch_hz_2);
        sbufWriteU16(dst, gyroConfig()->gyro_soft_notch_cutoff_2);
        sbufWriteU8(dst, gyroConfig()->dterm_lpf1_type);
        sbufWriteU8(dst, gyroConfig()->gyro_hardware_lpf);
        sbufWriteU8(dst, 0); // DEPRECATED: gyro_32khz_hardware_lpf
        sbufWriteU16(dst, gyroConfig()->gyro_lpf1_static_hz);
        sbufWriteU16(dst, gyroConfig()->gyro_lpf2_static_hz);
        sbufWriteU8(dst, gyroConfig()->gyro_lpf1_type);
        sbufWriteU8(dst, gyroConfig()->gyro_lpf2_type);
        sbufWriteU16(dst, gyroConfig()->dterm_lpf2_static_hz);
        // Added in MSP API 1.41
        sbufWriteU8(dst, gyroConfig()->dterm_lpf2_type);
#if defined(USE_DYN_LPF)
        sbufWriteU16(dst, gyroConfig()->gyro_lpf1_dyn_min_hz);
        sbufWriteU16(dst, gyroConfig()->gyro_lpf1_dyn_max_hz);
        sbufWriteU16(dst, gyroConfig()->dterm_lpf1_dyn_min_hz);
        sbufWriteU16(dst, gyroConfig()->dterm_lpf1_dyn_max_hz);
#else
        sbufWriteU16(dst, 0);
        sbufWriteU16(dst, 0);
        sbufWriteU16(dst, 0);
        sbufWriteU16(dst, 0);
#endif
        // Added in MSP API 1.42
#if defined(USE_DYN_NOTCH_FILTER)
        sbufWriteU8(dst, 0);  // DEPRECATED 1.43: dyn_notch_range
        sbufWriteU8(dst, 0);  // DEPRECATED 1.44: dyn_notch_width_percent
        sbufWriteU16(dst, dynNotchConfig()->dyn_notch_q);
        sbufWriteU16(dst, dynNotchConfig()->dyn_notch_min_hz);
#else
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);
        sbufWriteU16(dst, 0);
        sbufWriteU16(dst, 0);
#endif
        sbufWriteU8(dst, 0); // was rpmFilterConfig()->rpm_filter_harmonics
        sbufWriteU8(dst, 0); // was rpmFilterConfig()->rpm_filter_min_hz
#if defined(USE_DYN_NOTCH_FILTER)
        // Added in MSP API 1.43
        sbufWriteU16(dst, dynNotchConfig()->dyn_notch_max_hz);
#else
        sbufWriteU16(dst, 0);
#endif
        sbufWriteU8(dst, 0); // was currentPidProfile->dterm_lpf1_dyn_expo
#if defined(USE_DYN_NOTCH_FILTER)
        sbufWriteU8(dst, dynNotchConfig()->dyn_notch_count);
#else
        sbufWriteU8(dst, 0);
#endif

        break;
    case MSP_PID_ADVANCED:
        sbufWriteU16(dst, 0);
        sbufWriteU16(dst, 0);
        sbufWriteU16(dst, 0); // was pidProfile.yaw_p_limit
        sbufWriteU8(dst, 0); // reserved
        sbufWriteU8(dst, 0); // was vbatPidCompensation
        sbufWriteU8(dst, 0); // was currentPidProfile->feedforward_transition
        sbufWriteU8(dst, 0); // was low byte of currentPidProfile->dtermSetpointWeight
        sbufWriteU8(dst, 0); // reserved
        sbufWriteU8(dst, 0); // reserved
        sbufWriteU8(dst, 0); // reserved
        sbufWriteU16(dst, 0); // was currentPidProfile->rateAccelLimit
        sbufWriteU16(dst, 0); // was currentPidProfile->yawRateAccelLimit
        sbufWriteU8(dst, 0); // was currentPidProfile->levelAngleLimit
        sbufWriteU8(dst, 0); // was pidProfile.levelSensitivity
        sbufWriteU16(dst, 0); // was currentPidProfile->itermThrottleThreshold
        sbufWriteU16(dst, 0); // was currentPidProfile->itermAcceleratorGain
        sbufWriteU16(dst, 0); // was currentPidProfile->dtermSetpointWeight
        sbufWriteU8(dst, 0); // was currentPidProfile->iterm_rotation
        sbufWriteU8(dst, 0); // was currentPidProfile->smart_feedforward
        sbufWriteU8(dst, 0); // was currentPidProfile->iterm_relax
        sbufWriteU8(dst, 0); // was urrentPidProfile->iterm_relax_type
        sbufWriteU8(dst, 0); // was currentPidProfile->abs_control_gain
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0); // was currentPidProfile->acro_trainer_angle_limit
        sbufWriteU16(dst, currentPidProfile->pid[PID_ROLL].F);
        sbufWriteU16(dst, currentPidProfile->pid[PID_PITCH].F);
        sbufWriteU16(dst, currentPidProfile->pid[PID_YAW].F);

        sbufWriteU8(dst, 0); // was currentPidProfile->antiGravityMode
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0); // was currentPidProfile->iterm_relax_cutoff
        // Added in MSP API 1.43
        sbufWriteU8(dst, 0); // was currentPidProfile->motor_output_limit
        sbufWriteU8(dst, 0); // was currentPidProfile->auto_profile_cell_count
        sbufWriteU8(dst, 0); // was currentPidProfile->dyn_idle_min_rpm

        // Added in MSP API 1.44
        sbufWriteU8(dst, 0); // was currentPidProfile->feedforward_averaging
        sbufWriteU8(dst, 0); // was currentPidProfile->feedforward_smooth_factor
        sbufWriteU8(dst, 0); // was currentPidProfile->feedforward_boost
        sbufWriteU8(dst, 0); // was currentPidProfile->feedforward_max_rate_limit
        sbufWriteU8(dst, 0); // was currentPidProfile->feedforward_jitter_factor
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);
        break;
    case MSP_SENSOR_CONFIG:
#if defined(USE_ACC)
        sbufWriteU8(dst, accelerometerConfig()->acc_hardware);
#else
        sbufWriteU8(dst, 0);
#endif
#ifdef USE_BARO
        sbufWriteU8(dst, barometerConfig()->baro_hardware);
#else
        sbufWriteU8(dst, BARO_NONE);
#endif
#ifdef USE_MAG
        sbufWriteU8(dst, compassConfig()->mag_hardware);
#else
        sbufWriteU8(dst, MAG_NONE);
#endif
        break;

#if defined(USE_VTX_COMMON)
    case MSP_VTX_CONFIG:
        {
            const vtxDevice_t *vtxDevice = vtxCommonDevice();
            unsigned vtxStatus = 0;
            vtxDevType_e vtxType = VTXDEV_UNKNOWN;
            uint8_t deviceIsReady = 0;
            if (vtxDevice) {
                vtxCommonGetStatus(vtxDevice, &vtxStatus);
                vtxType = vtxCommonGetDeviceType(vtxDevice);
                deviceIsReady = vtxCommonDeviceIsReady(vtxDevice) ? 1 : 0;
            }
            sbufWriteU8(dst, vtxType);
            sbufWriteU8(dst, vtxSettingsConfig()->band);
            sbufWriteU8(dst, vtxSettingsConfig()->channel);
            sbufWriteU8(dst, vtxSettingsConfig()->power);
            sbufWriteU8(dst, (vtxStatus & VTX_STATUS_PIT_MODE) ? 1 : 0);
            sbufWriteU16(dst, vtxSettingsConfig()->freq);
            sbufWriteU8(dst, deviceIsReady);
            sbufWriteU8(dst, vtxSettingsConfig()->lowPowerDisarm);

            // API version 1.42
            sbufWriteU16(dst, vtxSettingsConfig()->pitModeFreq);
#ifdef USE_VTX_TABLE
            sbufWriteU8(dst, 1);   // vtxtable is available
            sbufWriteU8(dst, vtxTableConfig()->bands);
            sbufWriteU8(dst, vtxTableConfig()->channels);
            sbufWriteU8(dst, vtxTableConfig()->powerLevels);
#else
            sbufWriteU8(dst, 0);
            sbufWriteU8(dst, 0);
            sbufWriteU8(dst, 0);
            sbufWriteU8(dst, 0);
#endif

        }
        break;
#endif

    case MSP_TX_INFO:
        sbufWriteU8(dst, rssiSource);
        uint8_t rtcDateTimeIsSet = 0;
#ifdef USE_RTC_TIME
        dateTime_t dt;
        if (rtcGetDateTime(&dt)) {
            rtcDateTimeIsSet = 1;
        }
#else
        rtcDateTimeIsSet = RTC_NOT_SUPPORTED;
#endif
        sbufWriteU8(dst, rtcDateTimeIsSet);

        break;
#ifdef USE_RTC_TIME
    case MSP_RTC:
        {
            dateTime_t dt;
            if (rtcGetDateTime(&dt)) {
                sbufWriteU16(dst, dt.year);
                sbufWriteU8(dst, dt.month);
                sbufWriteU8(dst, dt.day);
                sbufWriteU8(dst, dt.hours);
                sbufWriteU8(dst, dt.minutes);
                sbufWriteU8(dst, dt.seconds);
                sbufWriteU16(dst, dt.millis);
            }
        }

        break;
#endif
    default:
        unsupportedCommand = true;
    }
    return !unsupportedCommand;
}

static mspResult_e mspFcProcessOutCommandWithArg(mspDescriptor_t srcDesc, int16_t cmdMSP, sbuf_t *src, sbuf_t *dst, mspPostProcessFnPtr *mspPostProcessFn)
{

    switch (cmdMSP) {
    case MSP_BOXNAMES:
        {
            const int page = sbufBytesRemaining(src) ? sbufReadU8(src) : 0;
            serializeBoxReply(dst, page, &serializeBoxNameFn);
        }
        break;
    case MSP_BOXIDS:
        {
            const int page = sbufBytesRemaining(src) ? sbufReadU8(src) : 0;
            serializeBoxReply(dst, page, &serializeBoxPermanentIdFn);
        }
        break;
    case MSP_REBOOT:
        if (sbufBytesRemaining(src)) {
            rebootMode = sbufReadU8(src);

            if (rebootMode >= MSP_REBOOT_COUNT
#if !defined(USE_USB_MSC)
                || rebootMode == MSP_REBOOT_MSC || rebootMode == MSP_REBOOT_MSC_UTC
#endif
                ) {
                return MSP_RESULT_ERROR;
            }
        } else {
            rebootMode = MSP_REBOOT_FIRMWARE;
        }

        sbufWriteU8(dst, rebootMode);

#if defined(USE_USB_MSC)
        if (rebootMode == MSP_REBOOT_MSC) {
            if (mscCheckFilesystemReady()) {
                sbufWriteU8(dst, 1);
            } else {
                sbufWriteU8(dst, 0);

                return MSP_RESULT_ACK;
            }
        }
#endif

#if defined(USE_MSP_OVER_TELEMETRY)
        if (featureIsEnabled(FEATURE_RX_SPI) && srcDesc == getMspTelemetryDescriptor()) {
            dispatchAdd(&mspRebootEntry, MSP_DISPATCH_DELAY_US);
        } else
#endif
        if (mspPostProcessFn) {
            *mspPostProcessFn = mspRebootFn;
        }

        break;
    case MSP_MULTIPLE_MSP:
        {
            uint8_t maxMSPs = 0;
            if (sbufBytesRemaining(src) == 0) {
                return MSP_RESULT_ERROR;
            }
            int bytesRemaining = sbufBytesRemaining(dst) - 1; // need to keep one byte for checksum
            mspPacket_t packetIn, packetOut;
            sbufInit(&packetIn.buf, src->end, src->end);
            uint8_t* resetInputPtr = src->ptr;
            while (sbufBytesRemaining(src) && bytesRemaining > 0) {
                uint8_t newMSP = sbufReadU8(src);
                sbufInit(&packetOut.buf, dst->ptr, dst->end);
                packetIn.cmd = newMSP;
                mspFcProcessCommand(srcDesc, &packetIn, &packetOut, NULL);
                uint8_t mspSize = sbufPtr(&packetOut.buf) - dst->ptr;
                mspSize++; // need to add length information for each MSP
                bytesRemaining -= mspSize;
                if (bytesRemaining >= 0) {
                    maxMSPs++;
                }
            }
            src->ptr = resetInputPtr;
            sbufInit(&packetOut.buf, dst->ptr, dst->end);
            for (int i = 0; i < maxMSPs; i++) {
                uint8_t* sizePtr = sbufPtr(&packetOut.buf);
                sbufWriteU8(&packetOut.buf, 0); // dummy
                packetIn.cmd = sbufReadU8(src);
                mspFcProcessCommand(srcDesc, &packetIn, &packetOut, NULL);
                (*sizePtr) = sbufPtr(&packetOut.buf) - (sizePtr + 1);
            }
            dst->ptr = packetOut.buf.ptr;
        }
        break;

#ifdef USE_VTX_TABLE
    case MSP_VTXTABLE_BAND:
        {
            const uint8_t band = sbufBytesRemaining(src) ? sbufReadU8(src) : 0;
            if (band > 0 && band <= VTX_TABLE_MAX_BANDS) {
                sbufWriteU8(dst, band);  // band number (same as request)
                sbufWriteU8(dst, VTX_TABLE_BAND_NAME_LENGTH); // band name length
                for (int i = 0; i < VTX_TABLE_BAND_NAME_LENGTH; i++) { // band name bytes
                    sbufWriteU8(dst, vtxTableConfig()->bandNames[band - 1][i]);
                }
                sbufWriteU8(dst, vtxTableConfig()->bandLetters[band - 1]); // band letter
                sbufWriteU8(dst, vtxTableConfig()->isFactoryBand[band - 1]); // CUSTOM = 0; FACTORY = 1
                sbufWriteU8(dst, vtxTableConfig()->channels); // number of channel frequencies to follow
                for (int i = 0; i < vtxTableConfig()->channels; i++) { // the frequency for each channel
                    sbufWriteU16(dst, vtxTableConfig()->frequency[band - 1][i]);
                }
            } else {
                return MSP_RESULT_ERROR;
            }
        }
        break;

    case MSP_VTXTABLE_POWERLEVEL:
        {
            const uint8_t powerLevel = sbufBytesRemaining(src) ? sbufReadU8(src) : 0;
            if (powerLevel > 0 && powerLevel <= VTX_TABLE_MAX_POWER_LEVELS) {
                sbufWriteU8(dst, powerLevel);  // powerLevel number (same as request)
                sbufWriteU16(dst, vtxTableConfig()->powerValues[powerLevel - 1]);
                sbufWriteU8(dst, VTX_TABLE_POWER_LABEL_LENGTH); // powerLevel label length
                for (int i = 0; i < VTX_TABLE_POWER_LABEL_LENGTH; i++) { // powerlevel label bytes
                    sbufWriteU8(dst, vtxTableConfig()->powerLabels[powerLevel - 1][i]);
                }
            } else {
                return MSP_RESULT_ERROR;
            }
        }
        break;
#endif // USE_VTX_TABLE

    case MSP_RESET_CONF:
        {
#if defined(USE_CUSTOM_DEFAULTS)
            defaultsType_e defaultsType = DEFAULTS_TYPE_CUSTOM;
#endif
            if (sbufBytesRemaining(src) >= 1) {
                // Added in MSP API 1.42
#if defined(USE_CUSTOM_DEFAULTS)
                defaultsType = sbufReadU8(src);
#else
                sbufReadU8(src);
#endif
            }

            bool success = false;
            if (!ARMING_FLAG(ARMED)) {
#if defined(USE_CUSTOM_DEFAULTS)
                success = resetEEPROM(defaultsType == DEFAULTS_TYPE_CUSTOM);
#else
                success = resetEEPROM(false);
#endif

                if (success && mspPostProcessFn) {
                    rebootMode = MSP_REBOOT_FIRMWARE;
                    *mspPostProcessFn = mspRebootFn;
                }
            }

            // Added in API version 1.42
            sbufWriteU8(dst, success);
        }

        break;
    default:
        return MSP_RESULT_CMD_UNKNOWN;
    }
    return MSP_RESULT_ACK;
}

#ifdef USE_FLASHFS
static void mspFcDataFlashReadCommand(sbuf_t *dst, sbuf_t *src)
{
    const unsigned int dataSize = sbufBytesRemaining(src);
    const uint32_t readAddress = sbufReadU32(src);
    uint16_t readLength;
    bool allowCompression = false;
    bool useLegacyFormat;
    if (dataSize >= sizeof(uint32_t) + sizeof(uint16_t)) {
        readLength = sbufReadU16(src);
        if (sbufBytesRemaining(src)) {
            allowCompression = sbufReadU8(src);
        }
        useLegacyFormat = false;
    } else {
        readLength = 128;
        useLegacyFormat = true;
    }

    serializeDataflashReadReply(dst, readAddress, readLength, useLegacyFormat, allowCompression);
}
#endif

static mspResult_e mspProcessInCommand(mspDescriptor_t srcDesc, int16_t cmdMSP, sbuf_t *src)
{
    uint32_t i;
    uint8_t value;
    const unsigned int dataSize = sbufBytesRemaining(src);
    switch (cmdMSP) {
    case MSP_SELECT_SETTING:
        value = sbufReadU8(src);
        if ((value & RATEPROFILE_MASK) == 0) {
            if (!ARMING_FLAG(ARMED)) {
                if (value >= PID_PROFILE_COUNT) {
                    value = 0;
                }
                changePidProfile(value);
            }
        } else {
            value = value & ~RATEPROFILE_MASK;

            if (value >= CONTROL_RATE_PROFILE_COUNT) {
                value = 0;
            }
            changeControlRateProfile(value);
        }
        break;

    case MSP_COPY_PROFILE:
        value = sbufReadU8(src);        // 0 = pid profile, 1 = control rate profile
        uint8_t dstProfileIndex = sbufReadU8(src);
        uint8_t srcProfileIndex = sbufReadU8(src);
        if (value == 0) {
            pidCopyProfile(dstProfileIndex, srcProfileIndex);
        }
        else if (value == 1) {
            copyControlRateProfile(dstProfileIndex, srcProfileIndex);
        }
        break;

#if defined(USE_GPS) || defined(USE_MAG)
    case MSP_SET_HEADING:
        magHold = sbufReadU16(src);
        break;
#endif

    case MSP_SET_RAW_RC:
#ifdef USE_RX_MSP
        {
            uint8_t channelCount = dataSize / sizeof(uint16_t);
            if (channelCount > MAX_SUPPORTED_RC_CHANNEL_COUNT) {
                return MSP_RESULT_ERROR;
            } else {
                uint16_t frame[MAX_SUPPORTED_RC_CHANNEL_COUNT];
                for (int i = 0; i < channelCount; i++) {
                    frame[i] = sbufReadU16(src);
                }
                rxMspFrameReceive(frame, channelCount);
            }
        }
#endif
        break;
#if defined(USE_ACC)
    case MSP_SET_ACC_TRIM:
        accelerometerConfigMutable()->accelerometerTrims.values.pitch = sbufReadU16(src);
        accelerometerConfigMutable()->accelerometerTrims.values.roll  = sbufReadU16(src);

        break;
#endif

    case MSP_SET_DEBUG_CONFIG:
        systemConfigMutable()->debug_mode = sbufReadU8(src);
        // RF TODO systemConfigMutable()->debug_axis = sbufReadU8(src);
        break;

    case MSP_SET_ARMING_CONFIG:
        armingConfigMutable()->auto_disarm_delay = sbufReadU8(src);
        break;

    case MSP_SET_PID:
        for (int i = 0; i < PID_AXIS_COUNT; i++) {
            currentPidProfile->pid[i].P = sbufReadU16(src);
            currentPidProfile->pid[i].I = sbufReadU16(src);
            currentPidProfile->pid[i].D = sbufReadU16(src);
            currentPidProfile->pid[i].F = sbufReadU16(src);
        }
        pidInitProfile(currentPidProfile);
        break;

    case MSP_SET_MODE_RANGE:
        i = sbufReadU8(src);
        if (i < MAX_MODE_ACTIVATION_CONDITION_COUNT) {
            modeActivationCondition_t *mac = modeActivationConditionsMutable(i);
            i = sbufReadU8(src);
            const box_t *box = findBoxByPermanentId(i);
            if (box) {
                mac->modeId = box->boxId;
                mac->auxChannelIndex = sbufReadU8(src);
                mac->range.startStep = sbufReadU8(src);
                mac->range.endStep = sbufReadU8(src);
                if (sbufBytesRemaining(src) != 0) {
                    mac->modeLogic = sbufReadU8(src);

                    i = sbufReadU8(src);
                    mac->linkedTo = findBoxByPermanentId(i)->boxId;
                }
                rcControlsInit();
            } else {
                return MSP_RESULT_ERROR;
            }
        } else {
            return MSP_RESULT_ERROR;
        }
        break;

    case MSP_SET_ADJUSTMENT_RANGE:
        break;

    case MSP_SET_RC_TUNING:
        currentControlRateProfile->rates_type = sbufReadU8(src);
        for (int i = 0; i < 3; i++) {
            currentControlRateProfile->rcRates[i] = sbufReadU8(src);
            currentControlRateProfile->rcExpo[i] = sbufReadU8(src);
            currentControlRateProfile->rates[i] = sbufReadU8(src);
            currentControlRateProfile->rate_limit[i] = sbufReadU16(src);
        }
        loadControlRateProfile();
        break;

    case MSP_SET_MOTOR_OVERRIDE:
#ifdef USE_MOTOR
        i = sbufReadU8(src);
        if (i >= MAX_SUPPORTED_MOTORS) {
            return MSP_RESULT_ERROR;
        }
        setMotorOverride(i, sbufReadU16(src));
#endif
        break;

    case MSP_SET_MOTOR_CONFIG:
        motorConfigMutable()->minthrottle = sbufReadU16(src);
        motorConfigMutable()->maxthrottle = sbufReadU16(src);
        motorConfigMutable()->mincommand = sbufReadU16(src);

        // sbufReadU8(src); MSP_MOTOR_CONFIG has motorCount here
        sbufReadU8(src); // compat: motorPoleCount

#if defined(USE_DSHOT_TELEMETRY)
        motorConfigMutable()->dev.useDshotTelemetry = sbufReadU8(src);
#else
        sbufReadU8(src);
#endif
        motorConfigMutable()->dev.motorPwmProtocol = sbufReadU8(src);
        motorConfigMutable()->dev.motorPwmRate = sbufReadU16(src);
        motorConfigMutable()->dev.motorPwmInversion = sbufReadU8(src);
        motorConfigMutable()->dev.useUnsyncedPwm = sbufReadU8(src);

        for (int i = 0; i < 4; i++)
            motorConfigMutable()->motorPoleCount[i] = sbufReadU8(src);
        for (int i = 0; i < 4; i++)
            motorConfigMutable()->motorRpmLpf[i] = sbufReadU8(src);

        motorConfigMutable()->mainRotorGearRatio[0] = sbufReadU16(src);
        motorConfigMutable()->mainRotorGearRatio[1] = sbufReadU16(src);
        motorConfigMutable()->tailRotorGearRatio[0] = sbufReadU16(src);
        motorConfigMutable()->tailRotorGearRatio[1] = sbufReadU16(src);
        break;

#ifdef USE_GPS
    case MSP_SET_GPS_CONFIG:
        gpsConfigMutable()->provider = sbufReadU8(src);
        gpsConfigMutable()->sbasMode = sbufReadU8(src);
        gpsConfigMutable()->autoConfig = sbufReadU8(src);
        gpsConfigMutable()->autoBaud = sbufReadU8(src);
        if (sbufBytesRemaining(src) >= 2) {
            // Added in API version 1.43
            gpsConfigMutable()->gps_set_home_point_once = sbufReadU8(src);
            gpsConfigMutable()->gps_ublox_use_galileo = sbufReadU8(src);
        }
        break;

#ifdef USE_GPS_RESCUE
        case MSP_SET_GPS_RESCUE:
        gpsRescueConfigMutable()->angle = sbufReadU16(src);
        gpsRescueConfigMutable()->initialAltitudeM = sbufReadU16(src);
        gpsRescueConfigMutable()->descentDistanceM = sbufReadU16(src);
        gpsRescueConfigMutable()->rescueGroundspeed = sbufReadU16(src);
        gpsRescueConfigMutable()->throttleMin = sbufReadU16(src);
        gpsRescueConfigMutable()->throttleMax = sbufReadU16(src);
        gpsRescueConfigMutable()->throttleHover = sbufReadU16(src);
        gpsRescueConfigMutable()->sanityChecks = sbufReadU8(src);
        gpsRescueConfigMutable()->minSats = sbufReadU8(src);
        if (sbufBytesRemaining(src) >= 6) {
            // Added in API version 1.43
            gpsRescueConfigMutable()->ascendRate = sbufReadU16(src);
            gpsRescueConfigMutable()->descendRate = sbufReadU16(src);
            gpsRescueConfigMutable()->allowArmingWithoutFix = sbufReadU8(src);
            gpsRescueConfigMutable()->altitudeMode = sbufReadU8(src);
        }
        if (sbufBytesRemaining(src) >= 2) {
            // Added in API version 1.44
            gpsRescueConfigMutable()->minRescueDth = sbufReadU16(src);
        }
        break;

    case MSP_SET_GPS_RESCUE_PIDS:
        gpsRescueConfigMutable()->throttleP = sbufReadU16(src);
        gpsRescueConfigMutable()->throttleI = sbufReadU16(src);
        gpsRescueConfigMutable()->throttleD = sbufReadU16(src);
        gpsRescueConfigMutable()->velP = sbufReadU16(src);
        gpsRescueConfigMutable()->velI = sbufReadU16(src);
        gpsRescueConfigMutable()->velD = sbufReadU16(src);
        gpsRescueConfigMutable()->yawP = sbufReadU16(src);
        break;
#endif
#endif

#ifdef USE_SERVOS
    case MSP_SET_SERVO_CONFIGURATION:
        if (dataSize != 1 + 12) {
            return MSP_RESULT_ERROR;
        }
        i = sbufReadU8(src);
        if (i >= MAX_SUPPORTED_SERVOS) {
            return MSP_RESULT_ERROR;
        }
        servoParamsMutable(i)->mid = sbufReadU16(src);
        servoParamsMutable(i)->min = sbufReadU16(src);
        servoParamsMutable(i)->max = sbufReadU16(src);
        servoParamsMutable(i)->rate = sbufReadU16(src);
        servoParamsMutable(i)->trim = sbufReadU16(src);
        servoParamsMutable(i)->speed = sbufReadU16(src);
        break;

    case MSP_SET_SERVO_OVERRIDE:
        i = sbufReadU8(src);
        if (i >= MAX_SUPPORTED_SERVOS) {
            return MSP_RESULT_ERROR;
        }
        setServoOverride(i, sbufReadU16(src));
        break;
#endif

    case MSP_SET_RC_DEADBAND:
        rcControlsConfigMutable()->deadband = sbufReadU8(src);
        rcControlsConfigMutable()->yaw_deadband = sbufReadU8(src);
        rcControlsConfigMutable()->alt_hold_deadband = sbufReadU8(src);
        sbufReadU16(src); // was flight3DConfigMutable()->deadband3d_throttle
        break;

    case MSP_SET_RESET_CURR_PID:
        resetPidProfile(currentPidProfile);
        break;

    case MSP_SET_SENSOR_ALIGNMENT: {
        // maintain backwards compatibility for API < 1.41
        const uint8_t gyroAlignment = sbufReadU8(src);
        sbufReadU8(src);  // discard deprecated acc_align
#if defined(USE_MAG)
        compassConfigMutable()->mag_alignment = sbufReadU8(src);
#else
        sbufReadU8(src);
#endif

        if (sbufBytesRemaining(src) >= 3) {
            // API >= 1.41 - support the gyro_to_use and alignment for gyros 1 & 2
#ifdef USE_MULTI_GYRO
            gyroConfigMutable()->gyro_to_use = sbufReadU8(src);
            gyroDeviceConfigMutable(0)->alignment = sbufReadU8(src);
            gyroDeviceConfigMutable(1)->alignment = sbufReadU8(src);
#else
            sbufReadU8(src);  // unused gyro_to_use
            gyroDeviceConfigMutable(0)->alignment = sbufReadU8(src);
            sbufReadU8(src);  // unused gyro_2_sensor_align
#endif
        } else {
            // maintain backwards compatibility for API < 1.41
#ifdef USE_MULTI_GYRO
            switch (gyroConfig()->gyro_to_use) {
            case GYRO_CONFIG_USE_GYRO_2:
                gyroDeviceConfigMutable(1)->alignment = gyroAlignment;
                break;
            case GYRO_CONFIG_USE_GYRO_BOTH:
                // For dual-gyro in "BOTH" mode we'll only update gyro 0
            default:
                gyroDeviceConfigMutable(0)->alignment = gyroAlignment;
                break;
            }
#else
            gyroDeviceConfigMutable(0)->alignment = gyroAlignment;
#endif

        }
        break;
    }

    case MSP_SET_ADVANCED_CONFIG:
        sbufReadU8(src);  // compat: gyro denom
        pidConfigMutable()->pid_process_denom = sbufReadU8(src);
        sbufReadU32(src); // compat: deprecated
        servoConfigMutable()->dev.servoPwmRate = sbufReadU16(src);
        break;

    case MSP_SET_FILTER_CONFIG:
        gyroConfigMutable()->gyro_lpf1_static_hz = sbufReadU8(src);
        gyroConfigMutable()->dterm_lpf1_static_hz = sbufReadU16(src);
        sbufReadU16(src); // was currentPidProfile->yaw_lowpass_hz
        if (sbufBytesRemaining(src) >= 8) {
            gyroConfigMutable()->gyro_soft_notch_hz_1 = sbufReadU16(src);
            gyroConfigMutable()->gyro_soft_notch_cutoff_1 = sbufReadU16(src);
            gyroConfigMutable()->dterm_notch_hz = sbufReadU16(src);
            gyroConfigMutable()->dterm_notch_cutoff = sbufReadU16(src);
        }
        if (sbufBytesRemaining(src) >= 4) {
            gyroConfigMutable()->gyro_soft_notch_hz_2 = sbufReadU16(src);
            gyroConfigMutable()->gyro_soft_notch_cutoff_2 = sbufReadU16(src);
        }
        if (sbufBytesRemaining(src) >= 1) {
            gyroConfigMutable()->dterm_lpf1_type = sbufReadU8(src);
        }
        if (sbufBytesRemaining(src) >= 10) {
            gyroConfigMutable()->gyro_hardware_lpf = sbufReadU8(src);
            sbufReadU8(src); // DEPRECATED: gyro_32khz_hardware_lpf
            gyroConfigMutable()->gyro_lpf1_static_hz = sbufReadU16(src);
            gyroConfigMutable()->gyro_lpf2_static_hz = sbufReadU16(src);
            gyroConfigMutable()->gyro_lpf1_type = sbufReadU8(src);
            gyroConfigMutable()->gyro_lpf2_type = sbufReadU8(src);
            gyroConfigMutable()->dterm_lpf2_static_hz = sbufReadU16(src);
        }
        if (sbufBytesRemaining(src) >= 9) {
            // Added in MSP API 1.41
            gyroConfigMutable()->dterm_lpf2_type = sbufReadU8(src);
#if defined(USE_DYN_LPF)
            gyroConfigMutable()->gyro_lpf1_dyn_min_hz = sbufReadU16(src);
            gyroConfigMutable()->gyro_lpf1_dyn_max_hz = sbufReadU16(src);
            gyroConfigMutable()->dterm_lpf1_dyn_min_hz = sbufReadU16(src);
            gyroConfigMutable()->dterm_lpf1_dyn_max_hz = sbufReadU16(src);
#else
            sbufReadU16(src);
            sbufReadU16(src);
            sbufReadU16(src);
            sbufReadU16(src);
#endif
        }
        if (sbufBytesRemaining(src) >= 8) {
            // Added in MSP API 1.42
#if defined(USE_DYN_NOTCH_FILTER)
            sbufReadU8(src); // DEPRECATED 1.43: dyn_notch_range
            sbufReadU8(src); // DEPRECATED 1.44: dyn_notch_width_percent
            dynNotchConfigMutable()->dyn_notch_q = sbufReadU16(src);
            dynNotchConfigMutable()->dyn_notch_min_hz = sbufReadU16(src);
#else
            sbufReadU8(src);
            sbufReadU8(src);
            sbufReadU16(src);
            sbufReadU16(src);
#endif
            sbufReadU8(src); // was rpmFilterConfigMutable()->rpm_filter_harmonics
            sbufReadU8(src); // was rpmFilterConfigMutable()->rpm_filter_min_hz
        }
        if (sbufBytesRemaining(src) >= 2) {
#if defined(USE_DYN_NOTCH_FILTER)
            // Added in MSP API 1.43
            dynNotchConfigMutable()->dyn_notch_max_hz = sbufReadU16(src);
#else
            sbufReadU16(src);
#endif
        }
        if (sbufBytesRemaining(src) >= 2) {
            // Added in MSP API 1.44
            sbufReadU8(src); // was currentPidProfile->dterm_lpf1_dyn_expo
#if defined(USE_DYN_NOTCH_FILTER)
            dynNotchConfigMutable()->dyn_notch_count = sbufReadU8(src);
#else
            sbufReadU8(src);
#endif
        }

        // reinitialize the gyro filters with the new values
        validateAndFixGyroConfig();
        gyroInitFilters();

        break;
    case MSP_SET_PID_ADVANCED:
        sbufReadU16(src);
        sbufReadU16(src);
        sbufReadU16(src); // was pidProfile.yaw_p_limit
        sbufReadU8(src); // reserved
        sbufReadU8(src); // was vbatPidCompensation
        sbufReadU8(src); // was currentPidProfile->feedforward_transition
        sbufReadU8(src); // was low byte of currentPidProfile->dtermSetpointWeight
        sbufReadU8(src); // reserved
        sbufReadU8(src); // reserved
        sbufReadU8(src); // reserved
        sbufReadU16(src); // was currentPidProfile->rate_accel_limit
        sbufReadU16(src); // was currentPidProfile->yaw_rate_accel_limit
        if (sbufBytesRemaining(src) >= 2) {
            sbufReadU8(src); // was currentPidProfile->levelAngleLimit
            sbufReadU8(src); // was pidProfile.levelSensitivity
        }
        if (sbufBytesRemaining(src) >= 4) {
            sbufReadU16(src); // was currentPidProfile->itermThrottleThreshold
            sbufReadU16(src); // was currentPidProfile->itermAcceleratorGain
        }
        if (sbufBytesRemaining(src) >= 2) {
            sbufReadU16(src); // was currentPidProfile->dtermSetpointWeight
        }
        if (sbufBytesRemaining(src) >= 14) {
            // Added in MSP API 1.40
            sbufReadU8(src); // was currentPidProfile->iterm_rotation
            sbufReadU8(src); // was currentPidProfile->smart_feedforward
            sbufReadU8(src); // was currentPidProfile->iterm_relax
            sbufReadU8(src); // was currentPidProfile->iterm_relax_type
            sbufReadU8(src); // was currentPidProfile->abs_control_gain
            sbufReadU8(src);
            sbufReadU8(src); // currentPidProfile->was acro_trainer_angle_limit
            // PID controller feedforward terms
            currentPidProfile->pid[PID_ROLL].F = sbufReadU16(src);
            currentPidProfile->pid[PID_PITCH].F = sbufReadU16(src);
            currentPidProfile->pid[PID_YAW].F = sbufReadU16(src);

            sbufReadU8(src); // was currentPidProfile->antiGravityMode
        }
        if (sbufBytesRemaining(src) >= 7) {
            // Added in MSP API 1.41
            sbufReadU8(src);
            sbufReadU8(src);
            sbufReadU8(src);
            sbufReadU8(src);
            sbufReadU8(src);
            sbufReadU8(src);
            sbufReadU8(src);
        }
        if(sbufBytesRemaining(src) >= 1) {
            // Added in MSP API 1.42
            sbufReadU8(src); // was currentPidProfile->iterm_relax_cutoff
        }
        if (sbufBytesRemaining(src) >= 3) {
            // Added in MSP API 1.43
            sbufReadU8(src); // was currentPidProfile->motor_output_limit
            sbufReadU8(src); // was currentPidProfile->auto_profile_cell_count
            sbufReadU8(src); // was currentPidProfile->dyn_idle_min_rpm
        }
        if (sbufBytesRemaining(src) >= 7) {
            // Added in MSP API 1.44
            sbufReadU8(src); // was currentPidProfile->feedforward_averaging
            sbufReadU8(src); // was currentPidProfile->feedforward_smooth_factor
            sbufReadU8(src); // was currentPidProfile->feedforward_boost
            sbufReadU8(src); // was currentPidProfile->feedforward_max_rate_limit
            sbufReadU8(src); // was currentPidProfile->feedforward_jitter_factor
            sbufReadU8(src);
            sbufReadU8(src);
        }
        pidInitProfile(currentPidProfile);

        break;
    case MSP_SET_SENSOR_CONFIG:
#if defined(USE_ACC)
        accelerometerConfigMutable()->acc_hardware = sbufReadU8(src);
#else
        sbufReadU8(src);
#endif
#if defined(USE_BARO)
        barometerConfigMutable()->baro_hardware = sbufReadU8(src);
#else
        sbufReadU8(src);
#endif
#if defined(USE_MAG)
        compassConfigMutable()->mag_hardware = sbufReadU8(src);
#else
        sbufReadU8(src);
#endif
        break;

#ifdef USE_ACC
    case MSP_ACC_CALIBRATION:
        if (!ARMING_FLAG(ARMED))
            accStartCalibration();
        break;
#endif

#if defined(USE_MAG)
    case MSP_MAG_CALIBRATION:
        if (!ARMING_FLAG(ARMED)) {
            compassStartCalibration();
        }
#endif

        break;
    case MSP_EEPROM_WRITE:
        if (ARMING_FLAG(ARMED)) {
            return MSP_RESULT_ERROR;
        }

        // This is going to take some time and won't be done where real-time performance is needed so
        // ignore how long it takes to avoid confusing the scheduler
        schedulerIgnoreTaskStateTime();

#if defined(USE_MSP_OVER_TELEMETRY)
        if (featureIsEnabled(FEATURE_RX_SPI) && srcDesc == getMspTelemetryDescriptor()) {
            dispatchAdd(&writeReadEepromEntry, MSP_DISPATCH_DELAY_US);
        } else
#endif
        {
            writeReadEeprom(NULL);
        }

        break;

#ifdef USE_BLACKBOX
    case MSP_SET_BLACKBOX_CONFIG:
        // Don't allow config to be updated while Blackbox is logging
        if (blackboxMayEditConfig()) {
            blackboxConfigMutable()->device = sbufReadU8(src);
            const int rateNum = sbufReadU8(src); // was rate_num
            const int rateDenom = sbufReadU8(src); // was rate_denom
            uint16_t pRatio = 0;
            if (sbufBytesRemaining(src) >= 2) {
                // p_ratio specified, so use it directly
                pRatio = sbufReadU16(src);
            } else {
                // p_ratio not specified in MSP, so calculate it from old rateNum and rateDenom
                pRatio = blackboxCalculatePDenom(rateNum, rateDenom);
            }

            if (sbufBytesRemaining(src) >= 1) {
                // sample_rate specified, so use it directly
                blackboxConfigMutable()->sample_rate = sbufReadU8(src);
            } else {
                // sample_rate not specified in MSP, so calculate it from old p_ratio
                blackboxConfigMutable()->sample_rate = blackboxCalculateSampleRate(pRatio);
            }
        }
        break;
#endif

#ifdef USE_VTX_COMMON
    case MSP_SET_VTX_CONFIG:
        {
            vtxDevice_t *vtxDevice = vtxCommonDevice();
            vtxDevType_e vtxType = VTXDEV_UNKNOWN;
            if (vtxDevice) {
                vtxType = vtxCommonGetDeviceType(vtxDevice);
            }
            uint16_t newFrequency = sbufReadU16(src);
            if (newFrequency <= VTXCOMMON_MSP_BANDCHAN_CHKVAL) {  // Value is band and channel
                const uint8_t newBand = (newFrequency / 8) + 1;
                const uint8_t newChannel = (newFrequency % 8) + 1;
                vtxSettingsConfigMutable()->band = newBand;
                vtxSettingsConfigMutable()->channel = newChannel;
                vtxSettingsConfigMutable()->freq = vtxCommonLookupFrequency(vtxDevice, newBand, newChannel);
            } else if (newFrequency <= VTX_SETTINGS_MAX_FREQUENCY_MHZ) { // Value is frequency in MHz
                vtxSettingsConfigMutable()->band = 0;
                vtxSettingsConfigMutable()->freq = newFrequency;
            }

            if (sbufBytesRemaining(src) >= 2) {
                vtxSettingsConfigMutable()->power = sbufReadU8(src);
                const uint8_t newPitmode = sbufReadU8(src);
                if (vtxType != VTXDEV_UNKNOWN) {
                    // Delegate pitmode to vtx directly
                    unsigned vtxCurrentStatus;
                    vtxCommonGetStatus(vtxDevice, &vtxCurrentStatus);
                    if ((bool)(vtxCurrentStatus & VTX_STATUS_PIT_MODE) != (bool)newPitmode) {
                        vtxCommonSetPitMode(vtxDevice, newPitmode);
                    }
                }
            }

            if (sbufBytesRemaining(src)) {
                    vtxSettingsConfigMutable()->lowPowerDisarm = sbufReadU8(src);
            }

            // API version 1.42 - this parameter kept separate since clients may already be supplying
            if (sbufBytesRemaining(src) >= 2) {
                vtxSettingsConfigMutable()->pitModeFreq = sbufReadU16(src);
            }

            // API version 1.42 - extensions for non-encoded versions of the band, channel or frequency
            if (sbufBytesRemaining(src) >= 4) {
                // Added standalone values for band, channel and frequency to move
                // away from the flawed encoded combined method originally implemented.
                uint8_t newBand = sbufReadU8(src);
                const uint8_t newChannel = sbufReadU8(src);
                uint16_t newFreq = sbufReadU16(src);
                if (newBand) {
                    newFreq = vtxCommonLookupFrequency(vtxDevice, newBand, newChannel);
                }
                vtxSettingsConfigMutable()->band = newBand;
                vtxSettingsConfigMutable()->channel = newChannel;
                vtxSettingsConfigMutable()->freq = newFreq;
            }

            // API version 1.42 - extensions for vtxtable support
            if (sbufBytesRemaining(src) >= 4) {
#ifdef USE_VTX_TABLE
                const uint8_t newBandCount = sbufReadU8(src);
                const uint8_t newChannelCount = sbufReadU8(src);
                const uint8_t newPowerCount = sbufReadU8(src);

                if ((newBandCount > VTX_TABLE_MAX_BANDS) ||
                    (newChannelCount > VTX_TABLE_MAX_CHANNELS) ||
                    (newPowerCount > VTX_TABLE_MAX_POWER_LEVELS)) {
                    return MSP_RESULT_ERROR;
                }
                vtxTableConfigMutable()->bands = newBandCount;
                vtxTableConfigMutable()->channels = newChannelCount;
                vtxTableConfigMutable()->powerLevels = newPowerCount;

                // boolean to determine whether the vtxtable should be cleared in
                // expectation that the detailed band/channel and power level messages
                // will follow to repopulate the tables
                if (sbufReadU8(src)) {
                    for (int i = 0; i < VTX_TABLE_MAX_BANDS; i++) {
                        vtxTableConfigClearBand(vtxTableConfigMutable(), i);
                        vtxTableConfigClearChannels(vtxTableConfigMutable(), i, 0);
                    }
                    vtxTableConfigClearPowerLabels(vtxTableConfigMutable(), 0);
                    vtxTableConfigClearPowerValues(vtxTableConfigMutable(), 0);
                }
#else
                sbufReadU8(src);
                sbufReadU8(src);
                sbufReadU8(src);
                sbufReadU8(src);
#endif
            }
        }
        break;
#endif

#ifdef USE_VTX_TABLE
    case MSP_SET_VTXTABLE_BAND:
        {
            char bandName[VTX_TABLE_BAND_NAME_LENGTH + 1];
            memset(bandName, 0, VTX_TABLE_BAND_NAME_LENGTH + 1);
            uint16_t frequencies[VTX_TABLE_MAX_CHANNELS];
            const uint8_t band = sbufReadU8(src);
            const uint8_t bandNameLength = sbufReadU8(src);
            for (int i = 0; i < bandNameLength; i++) {
                const char nameChar = sbufReadU8(src);
                if (i < VTX_TABLE_BAND_NAME_LENGTH) {
                    bandName[i] = toupper(nameChar);
                }
            }
            const char bandLetter = toupper(sbufReadU8(src));
            const bool isFactoryBand = (bool)sbufReadU8(src);
            const uint8_t channelCount = sbufReadU8(src);
            for (int i = 0; i < channelCount; i++) {
                const uint16_t frequency = sbufReadU16(src);
                if (i < vtxTableConfig()->channels) {
                    frequencies[i] = frequency;
                }
            }

            if (band > 0 && band <= vtxTableConfig()->bands) {
                vtxTableStrncpyWithPad(vtxTableConfigMutable()->bandNames[band - 1], bandName, VTX_TABLE_BAND_NAME_LENGTH);
                vtxTableConfigMutable()->bandLetters[band - 1] = bandLetter;
                vtxTableConfigMutable()->isFactoryBand[band - 1] = isFactoryBand;
                for (int i = 0; i < vtxTableConfig()->channels; i++) {
                    vtxTableConfigMutable()->frequency[band - 1][i] = frequencies[i];
                }
                // If this is the currently selected band then reset the frequency
                if (band == vtxSettingsConfig()->band) {
                    uint16_t newFreq = 0;
                    if (vtxSettingsConfig()->channel > 0 && vtxSettingsConfig()->channel <= vtxTableConfig()->channels) {
                        newFreq = frequencies[vtxSettingsConfig()->channel - 1];
                    }
                    vtxSettingsConfigMutable()->freq = newFreq;
                }
                vtxTableNeedsInit = true;  // reinintialize vtxtable after eeprom write
            } else {
                return MSP_RESULT_ERROR;
            }
        }
        break;

    case MSP_SET_VTXTABLE_POWERLEVEL:
        {
            char powerLevelLabel[VTX_TABLE_POWER_LABEL_LENGTH + 1];
            memset(powerLevelLabel, 0, VTX_TABLE_POWER_LABEL_LENGTH + 1);
            const uint8_t powerLevel = sbufReadU8(src);
            const uint16_t powerValue = sbufReadU16(src);
            const uint8_t powerLevelLabelLength = sbufReadU8(src);
            for (int i = 0; i < powerLevelLabelLength; i++) {
                const char labelChar = sbufReadU8(src);
                if (i < VTX_TABLE_POWER_LABEL_LENGTH) {
                    powerLevelLabel[i] = toupper(labelChar);
                }
            }

            if (powerLevel > 0 && powerLevel <= vtxTableConfig()->powerLevels) {
                vtxTableConfigMutable()->powerValues[powerLevel - 1] = powerValue;
                vtxTableStrncpyWithPad(vtxTableConfigMutable()->powerLabels[powerLevel - 1], powerLevelLabel, VTX_TABLE_POWER_LABEL_LENGTH);
                vtxTableNeedsInit = true;  // reinintialize vtxtable after eeprom write
            } else {
                return MSP_RESULT_ERROR;
            }
        }
        break;
#endif

#ifdef USE_DSHOT
    case MSP2_SEND_DSHOT_COMMAND:
        {
            const bool armed = ARMING_FLAG(ARMED);

            if (!armed) {
                const uint8_t commandType = sbufReadU8(src);
                const uint8_t motorIndex = sbufReadU8(src);
                const uint8_t commandCount = sbufReadU8(src);

                if (DSHOT_CMD_TYPE_BLOCKING == commandType) {
                    motorDisable();
                }

                for (uint8_t i = 0; i < commandCount; i++) {
                    const uint8_t commandIndex = sbufReadU8(src);
                    dshotCommandWrite(motorIndex, getMotorCount(), commandIndex, commandType);
                }

                if (DSHOT_CMD_TYPE_BLOCKING == commandType) {
                    motorEnable();
                }
            }
        }
        break;
#endif

#ifdef USE_CAMERA_CONTROL
    case MSP_CAMERA_CONTROL:
        {
            if (ARMING_FLAG(ARMED)) {
                return MSP_RESULT_ERROR;
            }

            const uint8_t key = sbufReadU8(src);
            cameraControlKeyPress(key, 0);
        }
        break;
#endif

    case MSP_SET_ARMING_DISABLED:
        {
            const uint8_t command = sbufReadU8(src);
            if (command) {
                mspArmingDisableByDescriptor(srcDesc);
                setArmingDisabled(ARMING_DISABLED_MSP);
                if (ARMING_FLAG(ARMED)) {
                    disarm(DISARM_REASON_ARMING_DISABLED);
                }
            } else {
                mspArmingEnableByDescriptor(srcDesc);
                if (mspIsMspArmingEnabled()) {
                    unsetArmingDisabled(ARMING_DISABLED_MSP);
                }
            }
        }
        break;

#ifdef USE_FLASHFS
    case MSP_DATAFLASH_ERASE:
        blackboxEraseAll();

        break;
#endif

#ifdef USE_GPS
    case MSP_SET_RAW_GPS:
        gpsSetFixState(sbufReadU8(src));
        gpsSol.numSat = sbufReadU8(src);
        gpsSol.llh.lat = sbufReadU32(src);
        gpsSol.llh.lon = sbufReadU32(src);
        gpsSol.llh.altCm = sbufReadU16(src) * 100; // alt changed from 1m to 0.01m per lsb since MSP API 1.39 by RTH. Received MSP altitudes in 1m per lsb have to upscaled.
        gpsSol.groundSpeed = sbufReadU16(src);
        GPS_update |= GPS_MSP_UPDATE;        // MSP data signalisation to GPS functions
        break;
#endif // USE_GPS
    case MSP_SET_FEATURE_CONFIG:
        featureConfigReplace(sbufReadU32(src));
        break;

#ifdef USE_BEEPER
    case MSP_SET_BEEPER_CONFIG:
        beeperConfigMutable()->beeper_off_flags = sbufReadU32(src);
        if (sbufBytesRemaining(src) >= 1) {
            beeperConfigMutable()->dshotBeaconTone = sbufReadU8(src);
        }
        if (sbufBytesRemaining(src) >= 4) {
            beeperConfigMutable()->dshotBeaconOffFlags = sbufReadU32(src);
        }
        break;
#endif

    case MSP_SET_BOARD_ALIGNMENT_CONFIG:
        boardAlignmentMutable()->rollDegrees = sbufReadU16(src);
        boardAlignmentMutable()->pitchDegrees = sbufReadU16(src);
        boardAlignmentMutable()->yawDegrees = sbufReadU16(src);
        break;

    case MSP_SET_MIXER_CONFIG:
        mixerConfigMutable()->main_rotor_dir = sbufReadU8(src);
        mixerConfigMutable()->tail_rotor_mode = sbufReadU8(src);
        mixerConfigMutable()->tail_motor_idle = sbufReadU8(src);
        mixerConfigMutable()->swash_ring = sbufReadU8(src);
        // RF TODO mixerConfigMutable()->swash_phase = sbufReadU8(src);
        break;

    case MSP_SET_MIXER_INPUT:
        i = sbufReadU8(src);
        if (i >= MIXER_INPUT_COUNT) {
            return MSP_RESULT_ERROR;
        }
        mixerInputsMutable(i)->rate = sbufReadU16(src);
        mixerInputsMutable(i)->min = sbufReadU16(src);
        mixerInputsMutable(i)->max = sbufReadU16(src);
        break;

    case MSP_SET_MIXER_RULE:
        i = sbufReadU8(src);
        if (i >= MIXER_RULE_COUNT) {
            return MSP_RESULT_ERROR;
        }
        sbufReadU32(src); // RF TODO remove me
        mixerRulesMutable(i)->oper = sbufReadU8(src);
        mixerRulesMutable(i)->input = sbufReadU8(src);
        mixerRulesMutable(i)->output = sbufReadU8(src);
        mixerRulesMutable(i)->offset = sbufReadU16(src);
        mixerRulesMutable(i)->weight = sbufReadU16(src);
        break;

    case MSP_SET_MIXER_OVERRIDE:
        i = sbufReadU8(src);
        if (i >= MIXER_INPUT_COUNT) {
            return MSP_RESULT_ERROR;
        }
        mixerSetOverride(i, sbufReadU16(src));
        break;

    case MSP_SET_RX_CONFIG:
        rxConfigMutable()->serialrx_provider = sbufReadU8(src);
        rxConfigMutable()->serialrx_inverted = sbufReadU8(src);
        rxConfigMutable()->halfDuplex = sbufReadU8(src);
        rxConfigMutable()->maxcheck = sbufReadU16(src);
        rxConfigMutable()->midrc = sbufReadU16(src);
        rxConfigMutable()->mincheck = sbufReadU16(src);
        rxConfigMutable()->rx_min_usec = sbufReadU16(src);
        rxConfigMutable()->rx_max_usec = sbufReadU16(src);
        sbufReadU8(src); // RF TODO remove
        sbufReadU8(src); // RF TODO remove
        sbufReadU8(src); // RF TODO remove
        sbufReadU8(src); // RF TODO remove
        sbufReadU8(src); // RF TODO remove
        sbufReadU8(src); // RF TODO remove
        sbufReadU8(src); // RF TODO remove
        sbufReadU8(src); // RF TODO remove
        sbufReadU8(src); // RF TODO remove
#ifdef USE_RX_SPI
        rxSpiConfigMutable()->rx_spi_protocol = sbufReadU8(src);
        rxSpiConfigMutable()->rx_spi_id = sbufReadU32(src);
        rxSpiConfigMutable()->rx_spi_rf_channel_count = sbufReadU8(src);
#else
        sbufReadU8(src);
        sbufReadU32(src);
        sbufReadU8(src);
#endif
        break;

    case MSP_SET_FAILSAFE_CONFIG:
        failsafeConfigMutable()->failsafe_delay = sbufReadU8(src);
        failsafeConfigMutable()->failsafe_off_delay = sbufReadU8(src);
        failsafeConfigMutable()->failsafe_throttle = sbufReadU16(src);
        failsafeConfigMutable()->failsafe_switch_mode = sbufReadU8(src);
        failsafeConfigMutable()->failsafe_throttle_low_delay = sbufReadU16(src);
        failsafeConfigMutable()->failsafe_procedure = sbufReadU8(src);
        break;

    case MSP_SET_RXFAIL_CONFIG:
        i = sbufReadU8(src);
        if (i < MAX_SUPPORTED_RC_CHANNEL_COUNT) {
            rxFailsafeChannelConfigsMutable(i)->mode = sbufReadU8(src);
            rxFailsafeChannelConfigsMutable(i)->step = CHANNEL_VALUE_TO_RXFAIL_STEP(sbufReadU16(src));
        } else {
            return MSP_RESULT_ERROR;
        }
        break;

    case MSP_SET_RSSI_CONFIG:
        rxConfigMutable()->rssi_channel = sbufReadU8(src);
        rxConfigMutable()->rssi_scale = sbufReadU8(src);
        rxConfigMutable()->rssi_invert = sbufReadU8(src);
        rxConfigMutable()->rssi_offset = sbufReadU8(src);
        break;

    case MSP_SET_RX_MAP:
        for (int i = 0; i < RX_MAPPABLE_CHANNEL_COUNT; i++) {
            rxConfigMutable()->rcmap[i] = sbufReadU8(src);
        }
        break;

    case MSP_SET_CF_SERIAL_CONFIG:
        {
            uint8_t portConfigSize = sizeof(uint8_t) + sizeof(uint16_t) + (sizeof(uint8_t) * 4);

            if (dataSize % portConfigSize != 0) {
                return MSP_RESULT_ERROR;
            }

            uint8_t remainingPortsInPacket = dataSize / portConfigSize;

            while (remainingPortsInPacket--) {
                uint8_t identifier = sbufReadU8(src);

                serialPortConfig_t *portConfig = serialFindPortConfigurationMutable(identifier);

                if (!portConfig) {
                    return MSP_RESULT_ERROR;
                }

                portConfig->identifier = identifier;
                portConfig->functionMask = sbufReadU16(src);
                portConfig->msp_baudrateIndex = sbufReadU8(src);
                portConfig->gps_baudrateIndex = sbufReadU8(src);
                portConfig->telemetry_baudrateIndex = sbufReadU8(src);
                portConfig->blackbox_baudrateIndex = sbufReadU8(src);
            }
        }
        break;
    case MSP2_COMMON_SET_SERIAL_CONFIG: {
        if (dataSize < 1) {
            return MSP_RESULT_ERROR;
        }
        unsigned count = sbufReadU8(src);
        unsigned portConfigSize = (dataSize - 1) / count;
        unsigned expectedPortSize = sizeof(uint8_t) + sizeof(uint32_t) + (sizeof(uint8_t) * 4);
        if (portConfigSize < expectedPortSize) {
            return MSP_RESULT_ERROR;
        }
        for (unsigned ii = 0; ii < count; ii++) {
            unsigned start = sbufBytesRemaining(src);
            uint8_t identifier = sbufReadU8(src);
            serialPortConfig_t *portConfig = serialFindPortConfigurationMutable(identifier);

            if (!portConfig) {
                return MSP_RESULT_ERROR;
            }

            portConfig->identifier = identifier;
            portConfig->functionMask = sbufReadU32(src);
            portConfig->msp_baudrateIndex = sbufReadU8(src);
            portConfig->gps_baudrateIndex = sbufReadU8(src);
            portConfig->telemetry_baudrateIndex = sbufReadU8(src);
            portConfig->blackbox_baudrateIndex = sbufReadU8(src);
            // Skip unknown bytes
            while (start - sbufBytesRemaining(src) < portConfigSize && sbufBytesRemaining(src)) {
                sbufReadU8(src);
            }
        }
        break;
    }

#ifdef USE_LED_STRIP_STATUS_MODE
    case MSP_SET_LED_COLORS:
        for (int i = 0; i < LED_CONFIGURABLE_COLOR_COUNT; i++) {
            hsvColor_t *color = &ledStripStatusModeConfigMutable()->colors[i];
            color->h = sbufReadU16(src);
            color->s = sbufReadU8(src);
            color->v = sbufReadU8(src);
        }
        break;
#endif

#ifdef USE_LED_STRIP
    case MSP_SET_LED_STRIP_CONFIG:
        {
            i = sbufReadU8(src);
            if (i >= LED_MAX_STRIP_LENGTH || dataSize != (1 + 4)) {
                return MSP_RESULT_ERROR;
            }
#ifdef USE_LED_STRIP_STATUS_MODE
            ledConfig_t *ledConfig = &ledStripStatusModeConfigMutable()->ledConfigs[i];
            *ledConfig = sbufReadU32(src);
            reevaluateLedConfig();
#else
            sbufReadU32(src);
#endif
            // API 1.41 - selected ledstrip_profile
            if (sbufBytesRemaining(src) >= 1) {
                ledStripConfigMutable()->ledstrip_profile = sbufReadU8(src);
            }
        }
        break;
#endif

#ifdef USE_LED_STRIP_STATUS_MODE
    case MSP_SET_LED_STRIP_MODECOLOR:
        {
            ledModeIndex_e modeIdx = sbufReadU8(src);
            int funIdx = sbufReadU8(src);
            int color = sbufReadU8(src);

            if (!setModeColor(modeIdx, funIdx, color)) {
                return MSP_RESULT_ERROR;
            }
        }
        break;
#endif

    case MSP_SET_NAME:
        memset(pilotConfigMutable()->name, 0, ARRAYLEN(pilotConfig()->name));
        for (unsigned int i = 0; i < MIN(MAX_NAME_LENGTH, dataSize); i++) {
            pilotConfigMutable()->name[i] = sbufReadU8(src);
        }
#ifdef USE_OSD
        osdAnalyzeActiveElements();
#endif
        break;

#ifdef USE_RTC_TIME
    case MSP_SET_RTC:
        {
            // Use seconds and milliseconds to make senders
            // easier to implement. Generating a 64 bit value
            // might not be trivial in some platforms.
            int32_t secs = (int32_t)sbufReadU32(src);
            uint16_t millis = sbufReadU16(src);
            rtcTime_t t = rtcTimeMake(secs, millis);
            rtcSet(&t);
        }

        break;
#endif

    case MSP_SET_TX_INFO:
        setRssiMsp(sbufReadU8(src));

        break;

#if defined(USE_BOARD_INFO)
    case MSP_SET_BOARD_INFO:
        if (!boardInformationIsSet()) {
            uint8_t length = sbufReadU8(src);
            char boardName[MAX_BOARD_NAME_LENGTH + 1];
            sbufReadData(src, boardName, MIN(length, MAX_BOARD_NAME_LENGTH));
            if (length > MAX_BOARD_NAME_LENGTH) {
                sbufAdvance(src, length - MAX_BOARD_NAME_LENGTH);
                length = MAX_BOARD_NAME_LENGTH;
            }
            boardName[length] = '\0';
            length = sbufReadU8(src);
            char manufacturerId[MAX_MANUFACTURER_ID_LENGTH + 1];
            sbufReadData(src, manufacturerId, MIN(length, MAX_MANUFACTURER_ID_LENGTH));
            if (length > MAX_MANUFACTURER_ID_LENGTH) {
                sbufAdvance(src, length - MAX_MANUFACTURER_ID_LENGTH);
                length = MAX_MANUFACTURER_ID_LENGTH;
            }
            manufacturerId[length] = '\0';

            setBoardName(boardName);
            setManufacturerId(manufacturerId);
            persistBoardInformation();
        } else {
            return MSP_RESULT_ERROR;
        }

        break;
#if defined(USE_SIGNATURE)
    case MSP_SET_SIGNATURE:
        if (!signatureIsSet()) {
            uint8_t signature[SIGNATURE_LENGTH];
            sbufReadData(src, signature, SIGNATURE_LENGTH);
            setSignature(signature);
            persistSignature();
        } else {
            return MSP_RESULT_ERROR;
        }

        break;
#endif
#endif // USE_BOARD_INFO
#if defined(USE_RX_BIND)
    case MSP2_BETAFLIGHT_BIND:
        if (!startRxBind()) {
            return MSP_RESULT_ERROR;
        }

        break;
#endif
    default:
        // we do not know how to handle the (valid) message, indicate error MSP $M!
        return MSP_RESULT_ERROR;
    }
    return MSP_RESULT_ACK;
}

static mspResult_e mspCommonProcessInCommand(mspDescriptor_t srcDesc, int16_t cmdMSP, sbuf_t *src, mspPostProcessFnPtr *mspPostProcessFn)
{
    UNUSED(mspPostProcessFn);
    const unsigned int dataSize = sbufBytesRemaining(src);
    UNUSED(dataSize); // maybe unused due to compiler options

    switch (cmdMSP) {
    case MSP_SET_VOLTAGE_METER_CONFIG: {
        int8_t id = sbufReadU8(src);

        //
        // find and configure an ADC voltage sensor
        //
        int8_t voltageSensorADCIndex;
        for (voltageSensorADCIndex = 0; voltageSensorADCIndex < MAX_VOLTAGE_SENSOR_ADC; voltageSensorADCIndex++) {
            if (id == voltageMeterADCtoIDMap[voltageSensorADCIndex]) {
                break;
            }
        }

        if (voltageSensorADCIndex < MAX_VOLTAGE_SENSOR_ADC) {
            voltageSensorADCConfigMutable(voltageSensorADCIndex)->vbatscale = sbufReadU8(src);
            voltageSensorADCConfigMutable(voltageSensorADCIndex)->vbatresdivval = sbufReadU8(src);
            voltageSensorADCConfigMutable(voltageSensorADCIndex)->vbatresdivmultiplier = sbufReadU8(src);
        } else {
            // if we had any other types of voltage sensor to configure, this is where we'd do it.
            sbufReadU8(src);
            sbufReadU8(src);
            sbufReadU8(src);
        }
        break;
    }

    case MSP_SET_CURRENT_METER_CONFIG: {
        int id = sbufReadU8(src);

        switch (id) {
            case CURRENT_METER_ID_BATTERY_1:
                currentSensorADCConfigMutable()->scale = sbufReadU16(src);
                currentSensorADCConfigMutable()->offset = sbufReadU16(src);
                break;
            default:
                sbufReadU16(src);
                sbufReadU16(src);
                break;
        }
        break;
    }

    case MSP_SET_BATTERY_CONFIG:
        batteryConfigMutable()->vbatmincellvoltage = sbufReadU8(src) * 10;      // vbatlevel_warn1 in MWC2.3 GUI
        batteryConfigMutable()->vbatmaxcellvoltage = sbufReadU8(src) * 10;      // vbatlevel_warn2 in MWC2.3 GUI
        batteryConfigMutable()->vbatwarningcellvoltage = sbufReadU8(src) * 10;  // vbatlevel when buzzer starts to alert
        batteryConfigMutable()->batteryCapacity = sbufReadU16(src);
        batteryConfigMutable()->voltageMeterSource = sbufReadU8(src);
        batteryConfigMutable()->currentMeterSource = sbufReadU8(src);
        if (sbufBytesRemaining(src) >= 6) {
            batteryConfigMutable()->vbatmincellvoltage = sbufReadU16(src);
            batteryConfigMutable()->vbatmaxcellvoltage = sbufReadU16(src);
            batteryConfigMutable()->vbatwarningcellvoltage = sbufReadU16(src);
        }
        break;

#if defined(USE_OSD)
    case MSP_SET_OSD_CONFIG:
        {
            const uint8_t addr = sbufReadU8(src);

            if ((int8_t)addr == -1) {
                /* Set general OSD settings */
#ifdef USE_MAX7456
                vcdProfileMutable()->video_system = sbufReadU8(src);
#else
                sbufReadU8(src); // Skip video system
#endif
#if defined(USE_OSD)
                osdConfigMutable()->units = sbufReadU8(src);

                // Alarms
                osdConfigMutable()->rssi_alarm = sbufReadU8(src);
                osdConfigMutable()->cap_alarm = sbufReadU16(src);
                sbufReadU16(src); // Skip unused (previously fly timer)
                osdConfigMutable()->alt_alarm = sbufReadU16(src);

                if (sbufBytesRemaining(src) >= 2) {
                    /* Enabled warnings */
                    // API < 1.41 supports only the low 16 bits
                    osdConfigMutable()->enabledWarnings = sbufReadU16(src);
                }

                if (sbufBytesRemaining(src) >= 4) {
                    // 32bit version of enabled warnings (API >= 1.41)
                    osdConfigMutable()->enabledWarnings = sbufReadU32(src);
                }

                if (sbufBytesRemaining(src) >= 1) {
                    // API >= 1.41
                    // selected OSD profile
#ifdef USE_OSD_PROFILES
                    changeOsdProfileIndex(sbufReadU8(src));
#else
                    sbufReadU8(src);
#endif // USE_OSD_PROFILES
                }

                if (sbufBytesRemaining(src) >= 1) {
                    // API >= 1.41
                    // OSD stick overlay mode

#ifdef USE_OSD_STICK_OVERLAY
                    osdConfigMutable()->overlay_radio_mode = sbufReadU8(src);
#else
                    sbufReadU8(src);
#endif // USE_OSD_STICK_OVERLAY

                }

                if (sbufBytesRemaining(src) >= 2) {
                    // API >= 1.43
                    // OSD camera frame element width/height
                    osdConfigMutable()->camera_frame_width = sbufReadU8(src);
                    osdConfigMutable()->camera_frame_height = sbufReadU8(src);
                }
#endif
            } else if ((int8_t)addr == -2) {
#if defined(USE_OSD)
                // Timers
                uint8_t index = sbufReadU8(src);
                if (index > OSD_TIMER_COUNT) {
                  return MSP_RESULT_ERROR;
                }
                osdConfigMutable()->timers[index] = sbufReadU16(src);
#endif
                return MSP_RESULT_ERROR;
            } else {
#if defined(USE_OSD)
                const uint16_t value = sbufReadU16(src);

                /* Get screen index, 0 is post flight statistics, 1 and above are in flight OSD screens */
                const uint8_t screen = (sbufBytesRemaining(src) >= 1) ? sbufReadU8(src) : 1;

                if (screen == 0 && addr < OSD_STAT_COUNT) {
                    /* Set statistic item enable */
                    osdStatSetState(addr, (value != 0));
                } else if (addr < OSD_ITEM_COUNT) {
                    /* Set element positions */
                    osdElementConfigMutable()->item_pos[addr] = value;
                    osdAnalyzeActiveElements();
                } else {
                  return MSP_RESULT_ERROR;
                }
#else
                return MSP_RESULT_ERROR;
#endif
            }
        }
        break;

    case MSP_OSD_CHAR_WRITE:
        {
            osdCharacter_t chr;
            size_t osdCharacterBytes;
            uint16_t addr;
            if (dataSize >= OSD_CHAR_VISIBLE_BYTES + 2) {
                if (dataSize >= OSD_CHAR_BYTES + 2) {
                    // 16 bit address, full char with metadata
                    addr = sbufReadU16(src);
                    osdCharacterBytes = OSD_CHAR_BYTES;
                } else if (dataSize >= OSD_CHAR_BYTES + 1) {
                    // 8 bit address, full char with metadata
                    addr = sbufReadU8(src);
                    osdCharacterBytes = OSD_CHAR_BYTES;
                } else {
                    // 16 bit character address, only visible char bytes
                    addr = sbufReadU16(src);
                    osdCharacterBytes = OSD_CHAR_VISIBLE_BYTES;
                }
            } else {
                // 8 bit character address, only visible char bytes
                addr = sbufReadU8(src);
                osdCharacterBytes = OSD_CHAR_VISIBLE_BYTES;
            }
            for (unsigned ii = 0; ii < MIN(osdCharacterBytes, sizeof(chr.data)); ii++) {
                chr.data[ii] = sbufReadU8(src);
            }
            displayPort_t *osdDisplayPort = osdGetDisplayPort(NULL);
            if (!osdDisplayPort) {
                return MSP_RESULT_ERROR;
            }

            if (!displayWriteFontCharacter(osdDisplayPort, addr, &chr)) {
                return MSP_RESULT_ERROR;
            }
        }
        break;
#endif // OSD

    default:
        return mspProcessInCommand(srcDesc, cmdMSP, src);
    }
    return MSP_RESULT_ACK;
}

/*
 * Returns MSP_RESULT_ACK, MSP_RESULT_ERROR or MSP_RESULT_NO_REPLY
 */
mspResult_e mspFcProcessCommand(mspDescriptor_t srcDesc, mspPacket_t *cmd, mspPacket_t *reply, mspPostProcessFnPtr *mspPostProcessFn)
{
    int ret = MSP_RESULT_ACK;
    sbuf_t *dst = &reply->buf;
    sbuf_t *src = &cmd->buf;
    const int16_t cmdMSP = cmd->cmd;
    // initialize reply by default
    reply->cmd = cmd->cmd;

    if (mspCommonProcessOutCommand(cmdMSP, dst, mspPostProcessFn)) {
        ret = MSP_RESULT_ACK;
    } else if (mspProcessOutCommand(cmdMSP, dst)) {
        ret = MSP_RESULT_ACK;
    } else if ((ret = mspFcProcessOutCommandWithArg(srcDesc, cmdMSP, src, dst, mspPostProcessFn)) != MSP_RESULT_CMD_UNKNOWN) {
        /* ret */;
    } else if (cmdMSP == MSP_SET_PASSTHROUGH) {
        mspFcSetPassthroughCommand(dst, src, mspPostProcessFn);
        ret = MSP_RESULT_ACK;
#ifdef USE_FLASHFS
    } else if (cmdMSP == MSP_DATAFLASH_READ) {
        mspFcDataFlashReadCommand(dst, src);
        ret = MSP_RESULT_ACK;
#endif
    } else {
        ret = mspCommonProcessInCommand(srcDesc, cmdMSP, src, mspPostProcessFn);
    }
    reply->result = ret;
    return ret;
}

void mspFcProcessReply(mspPacket_t *reply)
{
    sbuf_t *src = &reply->buf;
    UNUSED(src); // potentially unused depending on compile options.

    switch (reply->cmd) {
    case MSP_ANALOG:
        {
            uint8_t batteryVoltage = sbufReadU8(src);
            uint16_t mAhDrawn = sbufReadU16(src);
            uint16_t rssi = sbufReadU16(src);
            uint16_t amperage = sbufReadU16(src);

            UNUSED(rssi);
            UNUSED(batteryVoltage);
            UNUSED(amperage);
            UNUSED(mAhDrawn);

#ifdef USE_MSP_CURRENT_METER
            currentMeterMSPSet(amperage, mAhDrawn);
#endif
        }
        break;
    }
}

void mspInit(void)
{
    initActiveBoxIds();
}
