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

#include "common/utils.h"
#include "common/unit.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/telemetry.h"
#include "pg/rx.h"

#include "drivers/timer.h"
#include "drivers/serial.h"
#include "drivers/serial_softserial.h"

#include "io/serial.h"

#include "config/config.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "msp/msp_serial.h"

#include "rx/rx.h"

#include "telemetry/telemetry.h"
#include "telemetry/frsky_hub.h"
#include "telemetry/hott.h"
#include "telemetry/smartport.h"
#include "telemetry/ltm.h"
#include "telemetry/jetiexbus.h"
#include "telemetry/mavlink.h"
#include "telemetry/crsf.h"
#include "telemetry/ghst.h"
#include "telemetry/srxl.h"
#include "telemetry/ibus.h"
#include "telemetry/msp_shared.h"


serialPort_t *telemetrySharedPort = NULL;


bool telemetryIsSensorIdEnabled(sensor_id_e sensor_id)
{
    for (int i = 0; i < TELEM_SENSOR_SLOT_COUNT; i++) {
        if (telemetryConfig()->telemetry_sensors[i] == sensor_id)
            return true;
    }
    return false;
}

bool telemetryDetermineEnabledState(portSharing_e portSharing)
{
    bool enabled = (portSharing == PORTSHARING_NOT_SHARED);

    if (portSharing == PORTSHARING_SHARED) {
        if (isModeActivationConditionPresent(BOXTELEMETRY))
            enabled = IS_RC_MODE_ACTIVE(BOXTELEMETRY);
        else
            enabled = ARMING_FLAG(ARMED);
    }

    return enabled;
}

bool telemetryCheckRxPortShared(const serialPortConfig_t *portConfig, const SerialRXType serialrxProvider)
{
    if ((portConfig->functionMask & FUNCTION_RX_SERIAL) &&
        (portConfig->functionMask & TELEMETRY_SHAREABLE_PORT_FUNCTIONS_MASK) &&
        (serialrxProvider == SERIALRX_SPEKTRUM1024 ||
         serialrxProvider == SERIALRX_SPEKTRUM2048 ||
         serialrxProvider == SERIALRX_SBUS ||
         serialrxProvider == SERIALRX_SUMD ||
         serialrxProvider == SERIALRX_SUMH ||
         serialrxProvider == SERIALRX_XBUS_MODE_B ||
         serialrxProvider == SERIALRX_XBUS_MODE_B_RJ01 ||
         serialrxProvider == SERIALRX_IBUS)) {

        return true;
    }
#ifdef USE_TELEMETRY_IBUS
    if (portConfig->functionMask & FUNCTION_TELEMETRY_IBUS &&
        portConfig->functionMask & FUNCTION_RX_SERIAL &&
        serialrxProvider == SERIALRX_IBUS) {
        // IBUS serial RX & telemetry
        return true;
    }
#endif
    return false;
}


void telemetryProcess(timeUs_t currentTime)
{
    telemetryScheduleUpdate(currentTime);

#ifdef USE_TELEMETRY_FRSKY_HUB
    handleFrSkyHubTelemetry(currentTime);
#endif
#ifdef USE_TELEMETRY_HOTT
    handleHoTTTelemetry(currentTime);
#endif
#ifdef USE_TELEMETRY_SMARTPORT
    handleSmartPortTelemetry();
#endif
#ifdef USE_TELEMETRY_LTM
    handleLtmTelemetry();
#endif
#ifdef USE_TELEMETRY_JETIEXBUS
    handleJetiExBusTelemetry();
#endif
#ifdef USE_TELEMETRY_MAVLINK
    handleMAVLinkTelemetry();
#endif
#ifdef USE_TELEMETRY_CRSF
    handleCrsfTelemetry(currentTime);
#endif
#ifdef USE_TELEMETRY_GHST
    handleGhstTelemetry(currentTime);
#endif
#ifdef USE_TELEMETRY_SRXL
    handleSrxlTelemetry(currentTime);
#endif
#ifdef USE_TELEMETRY_IBUS
    handleIbusTelemetry();
#endif
}

void telemetryCheckState(void)
{
#ifdef USE_TELEMETRY_FRSKY_HUB
    checkFrSkyHubTelemetryState();
#endif
#ifdef USE_TELEMETRY_HOTT
    checkHoTTTelemetryState();
#endif
#ifdef USE_TELEMETRY_SMARTPORT
    checkSmartPortTelemetryState();
#endif
#ifdef USE_TELEMETRY_LTM
    checkLtmTelemetryState();
#endif
#ifdef USE_TELEMETRY_JETIEXBUS
    checkJetiExBusTelemetryState();
#endif
#ifdef USE_TELEMETRY_MAVLINK
    checkMAVLinkTelemetryState();
#endif
#ifdef USE_TELEMETRY_CRSF
    checkCrsfTelemetryState();
#endif
#ifdef USE_TELEMETRY_GHST
    checkGhstTelemetryState();
#endif
#ifdef USE_TELEMETRY_SRXL
    checkSrxlTelemetryState();
#endif
#ifdef USE_TELEMETRY_IBUS
    checkIbusTelemetryState();
#endif
}

void INIT_CODE telemetryInit(void)
{
    telemetryScheduleInit();

#ifdef USE_TELEMETRY_FRSKY_HUB
    initFrSkyHubTelemetry();
#endif
#ifdef USE_TELEMETRY_HOTT
    initHoTTTelemetry();
#endif
#ifdef USE_TELEMETRY_SMARTPORT
    initSmartPortTelemetry();
#endif
#ifdef USE_TELEMETRY_LTM
    initLtmTelemetry();
#endif
#ifdef USE_TELEMETRY_JETIEXBUS
    initJetiExBusTelemetry();
#endif
#ifdef USE_TELEMETRY_MAVLINK
    initMAVLinkTelemetry();
#endif
#ifdef USE_TELEMETRY_GHST
    initGhstTelemetry();
#endif
#ifdef USE_TELEMETRY_CRSF
    initCrsfTelemetry();
#if defined(USE_MSP_OVER_TELEMETRY)
    initCrsfMspBuffer();
#endif
#endif
#ifdef USE_TELEMETRY_SRXL
    initSrxlTelemetry();
#endif
#ifdef USE_TELEMETRY_IBUS
    initIbusTelemetry();
#endif
#if defined(USE_MSP_OVER_TELEMETRY)
    initSharedMsp();
#endif

    telemetryCheckState();
}


/** Telemetry scheduling framework **/

static telemetryScheduler_t sch;


bool INIT_CODE telemetryScheduleAdd(const telemetrySensor_t * sensor)
{
    if (sensor) {
        for (int i = 0; i < TELEM_SENSOR_SLOT_COUNT; i++) {
            telemetrySlot_t * slot = &sch.slots[i];
            if (!slot->sensor) {
                slot->sensor = sensor;
                slot->min_period = sensor->min_period;
                slot->max_period = sensor->max_period;
                slot->bucket = 0;
                slot->changed = true;
                return true;
            }
        }
    }

    return false;
}

void telemetryScheduleUpdate(timeUs_t currentTime)
{
    int delta = cmpTimeUs(currentTime, sch.update_time);

    for (int i = 0; i < TELEM_SENSOR_SLOT_COUNT; i++) {
        telemetrySlot_t * slot = &sch.slots[i];
        if (slot->sensor) {
            const telemetryValue_t value = slot->sensor->value();
            slot->changed |= (value != slot->value);
            slot->value = value;

            const int delay = slot->changed ? slot->min_period: slot->max_period;
            slot->bucket += delta * 1000 / delay;
            slot->bucket = constrain(slot->bucket, -2000000, 1000000);
        }
    }

    sch.update_time = currentTime;
}

telemetrySlot_t * telemetryScheduleNext(void)
{
    int index = sch.current_slot;

    for (int i = 0; i < TELEM_SENSOR_SLOT_COUNT; i++) {
        index = (index + 1) % TELEM_SENSOR_SLOT_COUNT;
        telemetrySlot_t * slot = &sch.slots[index];
        if (slot->sensor && slot->bucket > 0) {
            sch.current_slot = index;
            return slot;
        }
    }

    return NULL;
}

void telemetryScheduleCommit(telemetrySlot_t * slot)
{
    slot->bucket -= 1000000;
    slot->changed = false;
}


static uint32_t telemetry_legacy_sensors = 0;

bool telemetryIsSensorEnabled(uint32_t sensor_bits)
{
    return (telemetry_legacy_sensors & sensor_bits);
}


void INIT_CODE telemetryScheduleInit(void)
{
    memset(&sch, 0, sizeof(sch));

    telemetry_legacy_sensors = 0;

    for (int i = 0; i < TELEM_SENSOR_SLOT_COUNT; i++) {
        sensor_id_e id = telemetryConfig()->telemetry_sensors[i];
        if (id) {
            telemetry_legacy_sensors |= telemetrySensorGetLegacy(id);
        }
    }
}


#endif /* USE_TELEMETRY */
