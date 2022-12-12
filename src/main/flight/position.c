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
//#include <unistd.h>
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
    .altSource = DEFAULT,
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

static pt3Filter_t     gpsFilter;
static pt3Filter_t     baroFilter;
static pt3Filter_t     varioFilter;

static pt3Filter_t     gpsOffsetFilter;
static pt3Filter_t     baroOffsetFilter;

static pt3Filter_t     driftFilter;


static float calculateVario(float altitude)
{
    float vario = (altitude - varioAltPrev) * pidGetPidFrequency();
    vario = pt3FilterApply(&varioFilter, vario);

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
            baroAlt = pt3FilterApply(&baroFilter, baro.baroAltitude);
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
        if (STATE(GPS_FIX) && gpsSol.numSat > 12) {
            gpsAlt = pt3FilterApply(&gpsFilter, gpsSol.llh.altCm);
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

    if (haveBaroOffset)
        baroAlt -= baroAltOffset;

    if (haveGpsOffset)
        gpsAlt -= gpsAltOffset;

    if (haveBaroAlt && haveBaroOffset) {
        estimatedAltitude = baroAlt;
        estimatedVario = calculateVario(baroAlt);
        if (haveGpsAlt && haveGpsOffset) {
            baroDrift = pt3FilterApply(&driftFilter, baroAlt - gpsAlt);
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

bool hasAltitudeOffset(void)
{
    return haveBaroOffset || haveGpsOffset;
}

float getAltitude(void)
{
    return estimatedAltitude;
}

int32_t getEstimatedAltitudeCm(void)
{
    return estimatedAltitude;
}

int16_t getEstimatedVarioCms(void)
{
    return estimatedVario;
}


void positionUpdate(void)
{
    calculateAltitude();
}

void positionInit(void)
{
    pt3FilterInit(&gpsFilter, pt3FilterGain(0.5f, pidGetDT()));
    pt3FilterInit(&baroFilter, pt3FilterGain(0.5f, pidGetDT()));
    pt3FilterInit(&varioFilter, pt3FilterGain(0.5f, pidGetDT()));

    pt3FilterInit(&gpsOffsetFilter, pt3FilterGain(0.05f, pidGetDT()));
    pt3FilterInit(&baroOffsetFilter, pt3FilterGain(0.05f, pidGetDT()));

    pt3FilterInit(&driftFilter, pt3FilterGain(0.01f, pidGetDT()));
}
