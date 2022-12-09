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

#pragma once

#include "common/time.h"

typedef struct positionConfig_s {
    uint8_t alt_source;
    uint8_t baro_alt_lpf;
    uint8_t baro_offset_lpf;
    uint8_t baro_drift_lpf;
    uint8_t gps_alt_lpf;
    uint8_t gps_offset_lpf;
    uint8_t gps_min_sats;
    uint8_t vario_lpf;
} positionConfig_t;

PG_DECLARE(positionConfig_t, positionConfig);

bool hasAltitudeOffset(void);

void positionInit(void);
void positionUpdate(void);

float getAltitude(void);
float getVario(void);

int32_t getEstimatedAltitudeCm(void);
int16_t getEstimatedVario(void);
