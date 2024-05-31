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

#pragma once

#include "common/unit.h"

#include "pg/pg.h"
#include "pg/telemetry.h"

#include "io/serial.h"

#include "rx/rx.h"

#include "telemetry/sensors.h"


typedef struct {
    const telemetrySensor_t *   sensor;
    int                         min_period;
    int                         max_period;
    int                         bucket;
    bool                        changed;
    telemetryValue_t            value;
} telemetrySlot_t;

typedef struct {
    timeUs_t                    update_time;
    unsigned                    current_slot;
    telemetrySlot_t             slots[TELEM_SENSOR_SLOT_COUNT];
} telemetryScheduler_t;


extern serialPort_t *telemetrySharedPort;

bool telemetryDetermineEnabledState(portSharing_e portSharing);
bool telemetryCheckRxPortShared(const serialPortConfig_t *portConfig, const SerialRXType serialrxProvider);

void telemetryProcess(timeUs_t currentTime);
void telemetryCheckState(void);
void telemetryInit(void);

bool telemetryScheduleAdd(const telemetrySensor_t * sensor);

void telemetryScheduleUpdate(timeUs_t currentTime);
void telemetryScheduleCommit(telemetrySlot_t * slot);

telemetrySlot_t * telemetryScheduleNext(void);

void telemetryScheduleInit(void);

bool telemetryIsSensorEnabled(uint32_t sensor_bits);

