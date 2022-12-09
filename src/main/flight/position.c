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


static bool haveBaroAlt = false;
static bool haveGpsAlt = false;

static bool baroOffsetSet = false;
static bool gpsOffsetSet = false;

static float estimatedAltitudeCm = 0;
static float estimatedVarioCms = 0;

static float baroAlt = 0;
static float gpsAlt = 0;

static float varioAltPrev = 0;

static float baroAltOffset = 0;
static float gpsAltOffset = 0;

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

    if (!ARMING_FLAG(ARMED)) {
        baroOffsetSet = false;
        gpsOffsetSet = false;
    }

#ifdef USE_BARO
    if (sensors(SENSOR_BARO)) {
        if (baroIsReady()) {
            baroAlt = pt3FilterApply(&baroFilter, baro.baroAltitude);
            baroGround = pt3FilterApply(&baroOffsetFilter, baroAlt);
            haveBaroAlt = true;

            if (ARMING_FLAG(ARMED) && !baroOffsetSet) {
                baroAltOffset = baroGround;
                baroOffsetSet = true;
            }

            baroAlt -= baroAltOffset;
        }
    }
#endif

#ifdef USE_GPS
    if (sensors(SENSOR_GPS)) {
        if (STATE(GPS_FIX)) {
            gpsAlt = pt3FilterApply(&gpsFilter, gpsSol.llh.altCm);
            gpsGround = pt3FilterApply(&gpsOffsetFilter, gpsAlt);
            haveGpsAlt = true;

            if (ARMING_FLAG(ARMED) && !gpsOffsetSet) {
                gpsAltOffset = gpsGround;
                gpsOffsetSet = true;
            }

            gpsAlt -= gpsAltOffset;
        }

        if (!gpsOffsetSet) {
            haveGpsAlt = false;
        }
    }

#endif

    if (haveGpsAlt && haveBaroAlt) {
        baroDrift = pt3FilterApply(&driftFilter, baroAlt - gpsAlt);
        estimatedAltitudeCm = baroAlt - baroDrift;
        estimatedVarioCms = calculateVario(baroAlt);
    }
    else if (haveGpsAlt) {
        estimatedAltitudeCm = gpsAlt;
        estimatedVarioCms = calculateVario(gpsAlt);
    }
    else if (haveBaroAlt) {
        estimatedAltitudeCm = baroAlt;
        estimatedVarioCms = calculateVario(baroAlt);
    }

    DEBUG(ALTITUDE, 0, estimatedAltitudeCm);
    DEBUG(ALTITUDE, 1, estimatedVarioCms);
    DEBUG(ALTITUDE, 2, baroAlt);
    DEBUG(ALTITUDE, 3, baroGround);
    DEBUG(ALTITUDE, 4, baroDrift);
    DEBUG(ALTITUDE, 5, gpsAlt);
    DEBUG(ALTITUDE, 6, gpsGround);
}

bool isAltitudeOffset(void)
{
    return baroOffsetSet || gpsOffsetSet;
}

int32_t getEstimatedAltitudeCm(void)
{
    return estimatedAltitudeCm;
}

int16_t getEstimatedVarioCms(void)
{
    return estimatedVarioCms;
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
