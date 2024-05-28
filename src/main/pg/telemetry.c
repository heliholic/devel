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

#include "common/utils.h"
#include "common/unit.h"

#include "config/config.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/telemetry.h"
#include "pg/rx.h"

#include "drivers/timer.h"
#include "drivers/serial.h"
#include "drivers/serial_softserial.h"

#include "io/serial.h"

#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "rx/rx.h"

#if 1
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
#endif

PG_REGISTER_WITH_RESET_TEMPLATE(telemetryConfig_t, telemetryConfig, PG_TELEMETRY_CONFIG, 0);

PG_RESET_TEMPLATE(telemetryConfig_t, telemetryConfig,
    .telemetry_inverted = false,
    .halfDuplex = 1,
    .gpsNoFixLatitude = 0,
    .gpsNoFixLongitude = 0,
    .frsky_coordinate_format = FRSKY_FORMAT_DMS,
    .frsky_unit = UNIT_METRIC,
    .frsky_vfas_precision = 0,
    .hottAlarmSoundInterval = 5,
    .pidValuesAsTelemetry = 0,
    .report_cell_voltage = false,
    .mavlink_mah_as_heading_divisor = 0,
    .flysky_sensors = { 0, },
    .telemetry_sensors = { 0, },
);

#endif
