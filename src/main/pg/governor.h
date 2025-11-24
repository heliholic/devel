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

#include "types.h"
#include "platform.h"

#include "pg/pg.h"

typedef enum {
    GOV_MODE_NONE = 0,
    GOV_MODE_EXTERNAL,
    GOV_MODE_ELECTRIC,
    GOV_MODE_NITRO,
} govMode_e;

typedef enum {
    GOV_THROTTLE_NORMAL = 0,
    GOV_THROTTLE_RANGE,
    GOV_THROTTLE_SWITCH,
} govThrottle_e;

typedef enum {
    GOV_FLAG_FALLBACK_PRECOMP       = 2,
    GOV_FLAG_VOLTAGE_COMP           = 3,
    GOV_FLAG_PID_SPOOLUP            = 4,
    GOV_FLAG_DYN_MIN_THROTTLE       = 6,
} govFlags_e;


#define GOV_THROTTLE_CURVE_POINTS    9

typedef struct governorConfig_s {
    uint8_t  gov_mode;
    uint8_t  gov_throttle_type;
    uint16_t gov_startup_time;
    uint16_t gov_spoolup_time;
    uint16_t gov_tracking_time;
    uint16_t gov_recovery_time;
    uint16_t gov_spooldown_time;
    uint8_t  gov_throttle_hold_timeout;
    uint8_t  gov_autorotation_timeout;
    uint8_t  gov_idle_throttle;
    uint8_t  gov_auto_throttle;
    uint8_t  gov_handover_throttle;
    uint8_t  gov_throttle_curve[GOV_THROTTLE_CURVE_POINTS];
    uint8_t  gov_pwr_filter;
    uint8_t  gov_rpm_filter;
    uint8_t  gov_tta_filter;
    uint8_t  gov_ff_filter;
    uint8_t  gov_d_filter;
} governorConfig_t;

PG_DECLARE(governorConfig_t, governorConfig);


