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

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <limits.h>
#include <math.h>

#include "platform.h"

#include "build/debug.h"

#include "common/maths.h"
#include "common/filter.h"

#include "fc/runtime_config.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "io/gps.h"

#include "sensors/sensors.h"
#include "sensors/barometer.h"

#include "flight/imu.h"
#include "flight/pid.h"
#include "flight/position.h"

typedef enum {
    DEFAULT = 0,
    BARO_ONLY,
    GPS_ONLY
} altSource_e;


PG_REGISTER_WITH_RESET_TEMPLATE(positionConfig_t, positionConfig, PG_POSITION, 0);

PG_RESET_TEMPLATE(positionConfig_t, positionConfig,
    .alt_source = DEFAULT,
    .baro_alt_lpf = 25,
    .baro_offset_lpf = 10,
    .baro_drift_lpf = 5,
    .gps_alt_lpf = 25,
    .gps_offset_lpf = 10,
    .gps_min_sats = 12,
    .vario_lpf = 25,
);

static bool wasARMED = false;

static bool haveBaroAlt = false;
static bool haveGpsAlt = false;

static bool haveBaroOffset = false;
static bool haveGpsOffset = false;

static float estimatedAltitude = 0;
static float estimatedVario = 0;

static float baroAlt = 0;
static float gpsAlt = 0;

static float baroAltOffset = 0;
static float gpsAltOffset = 0;

static float varioAltPrev = 0;

static pt2Filter_t     gpsFilter;
static pt2Filter_t     baroFilter;
static pt2Filter_t     varioFilter;

static pt3Filter_t     gpsOffsetFilter;
static pt3Filter_t     baroOffsetFilter;

static pt2Filter_t     driftFilter;


bool hasAltitudeOffset(void)
{
    return haveBaroOffset || haveGpsOffset;
}

float getAltitude(void)
{
    return estimatedAltitude;
}

float getVario(void)
{
    return estimatedVario;
}

int32_t getEstimatedAltitudeCm(void)
{
    return lrintf(estimatedAltitude);
}

int16_t getEstimatedVario(void)
{
    return lrintf(estimatedVario);
}


static float calculateVario(float altitude)
{
    float vario = (altitude - varioAltPrev) * pidGetPidFrequency();
    vario = pt2FilterApply(&varioFilter, vario);

    varioAltPrev = altitude;

    return vario;
}

static void calculateAltitude(void)
{
    float gpsGround = 0;
    float baroGround = 0;
    float baroDrift = 0;

#ifdef USE_BARO
    if (sensors(SENSOR_BARO)) {
        if (baroIsReady()) {
            baroAlt = pt2FilterApply(&baroFilter, baro.baroAltitude);
            baroGround = pt3FilterApply(&baroOffsetFilter, baroAlt);
            haveBaroAlt = true;
        }
        else {
            haveBaroAlt = false;
        }
    }
#endif

#ifdef USE_GPS
    if (sensors(SENSOR_GPS)) {
        if (STATE(GPS_FIX) && gpsSol.numSat >= positionConfig()->gps_min_sats) {
            gpsAlt = pt2FilterApply(&gpsFilter, gpsSol.llh.altCm);
            gpsGround = pt3FilterApply(&gpsOffsetFilter, gpsAlt);
            haveGpsAlt = true;
        }
        else {
            haveGpsAlt = false;
        }
    }
#endif

    if (ARMING_FLAG(ARMED)) {
        if (!wasARMED) {
            if (haveBaroAlt) {
                haveBaroOffset = true;
                baroAltOffset = baroGround;
            }
            if (haveGpsAlt) {
                haveGpsOffset = true;
                gpsAltOffset = gpsGround;
            }
            wasARMED = true;
        }
    }
    else {
        if (wasARMED) {
            haveBaroOffset = false;
            haveGpsOffset = false;
            wasARMED = false;
        }
    }

    if (haveBaroAlt && haveBaroOffset)
        baroAlt -= baroAltOffset;

    if (haveGpsAlt && haveGpsOffset)
        gpsAlt -= gpsAltOffset;

    if (haveBaroAlt) {
        estimatedAltitude = baroAlt;
        estimatedVario = calculateVario(baroAlt);
        if (haveBaroOffset && haveGpsAlt && haveGpsOffset) {
            baroDrift = pt2FilterApply(&driftFilter, baroAlt - gpsAlt);
            estimatedAltitude -= baroDrift;
        }
    }
    else if (haveGpsAlt) {
        estimatedAltitude = gpsAlt;
        estimatedVario = calculateVario(gpsAlt);
    }
    else {
        estimatedAltitude = 0;
        estimatedVario = 0;
    }

#if 1
    DEBUG(ALTITUDE, 0, estimatedAltitude);
    DEBUG(ALTITUDE, 1, estimatedVario);
    DEBUG(ALTITUDE, 2, baroAlt);
    DEBUG(ALTITUDE, 3, baroGround);
    DEBUG(ALTITUDE, 4, baroDrift);
    DEBUG(ALTITUDE, 5, gpsAlt);
    DEBUG(ALTITUDE, 6, gpsGround);
#else
    DEBUG(ALTITUDE, 0, baroGround);
    DEBUG(ALTITUDE, 1, baroDrift);
    DEBUG(ALTITUDE, 2, gpsAlt);
    DEBUG(ALTITUDE, 3, gpsGround);
#endif
}

void positionUpdate(void)
{
    calculateAltitude();
}

void positionInit(void)
{
    pt2FilterInit(&gpsFilter, pt2FilterGain(positionConfig()->gps_alt_lpf / 100.0f, pidGetDT()));
    pt2FilterInit(&baroFilter, pt2FilterGain(positionConfig()->baro_alt_lpf / 100.0f, pidGetDT()));
    pt2FilterInit(&varioFilter, pt2FilterGain(positionConfig()->vario_lpf / 100.0f, pidGetDT()));

    pt3FilterInit(&gpsOffsetFilter, pt3FilterGain(positionConfig()->gps_offset_lpf / 1000.0f, pidGetDT()));
    pt3FilterInit(&baroOffsetFilter, pt3FilterGain(positionConfig()->baro_offset_lpf / 1000.0f, pidGetDT()));

    pt2FilterInit(&driftFilter, pt2FilterGain(positionConfig()->baro_drift_lpf / 1000.0f, pidGetDT()));
}
