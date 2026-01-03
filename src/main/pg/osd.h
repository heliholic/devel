/*
 * This file is part of Rotorflight.
 *
 * Rotorflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Rotorflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

 #pragma once

#include <stdint.h>
#include <stdbool.h>

#include "drivers/display.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#ifdef USE_OSD_PROFILES
#define OSD_PROFILE_COUNT 3
#else
#define OSD_PROFILE_COUNT 1
#endif

#define OSD_RCCHANNELS_COUNT 4

#define OSD_PROFILE_NAME_LENGTH 16


// NB: to ensure backwards compatibility, new enum values must be appended at the end but before the OSD_XXXX_COUNT entry.

// *** IMPORTANT ***
// See the information at the top of osd/osd_elements.c for instructions
// on how to add OSD elements.
typedef enum {
    OSD_RSSI_VALUE,
    OSD_MAIN_BATT_VOLTAGE,
    OSD_CROSSHAIRS,
    OSD_ARTIFICIAL_HORIZON,
    OSD_HORIZON_SIDEBARS,
    OSD_ITEM_TIMER_1,
    OSD_ITEM_TIMER_2,
    OSD_FLYMODE,
    OSD_CRAFT_NAME,
    OSD_THROTTLE_POS,
    OSD_VTX_CHANNEL,
    OSD_CURRENT_DRAW,
    OSD_MAH_DRAWN,
    OSD_GPS_SPEED,
    OSD_GPS_SATS,
    OSD_ALTITUDE,
    OSD_ROLL_PIDS,
    OSD_PITCH_PIDS,
    OSD_YAW_PIDS,
    OSD_POWER,
    OSD_PIDRATE_PROFILE,
    OSD_WARNINGS,
    OSD_AVG_CELL_VOLTAGE,
    OSD_GPS_LON,
    OSD_GPS_LAT,
    OSD_DEBUG,
    OSD_PITCH_ANGLE,
    OSD_ROLL_ANGLE,
    OSD_MAIN_BATT_USAGE,
    OSD_DISARMED,
    OSD_HOME_DIR,
    OSD_HOME_DIST,
    OSD_NUMERICAL_HEADING,
    OSD_NUMERICAL_VARIO,
    OSD_COMPASS_BAR,
    OSD_ESC_TMP,
    OSD_ESC_RPM,
    OSD_REMAINING_TIME_ESTIMATE,
    OSD_RTC_DATETIME,
    OSD_ADJUSTMENT_RANGE,
    OSD_CORE_TEMPERATURE,
    OSD_G_FORCE,
    OSD_MOTOR_DIAG,
    OSD_LOG_STATUS,
    OSD_FLIP_ARROW,
    OSD_LINK_QUALITY,
    OSD_FLIGHT_DIST,
    OSD_STICK_OVERLAY_LEFT,
    OSD_STICK_OVERLAY_RIGHT,
    OSD_PILOT_NAME,
    OSD_ESC_RPM_FREQ,
    OSD_RATE_PROFILE_NAME,
    OSD_PID_PROFILE_NAME,
    OSD_PROFILE_NAME,
    OSD_RSSI_DBM_VALUE,
    OSD_RC_CHANNELS,
    OSD_CAMERA_FRAME,
    OSD_EFFICIENCY,
    OSD_TOTAL_FLIGHTS,
    OSD_UP_DOWN_REFERENCE,
    OSD_TX_UPLINK_POWER,
    OSD_WATT_HOURS_DRAWN,
    OSD_AUX_VALUE,
    OSD_READY_MODE,
    OSD_RSNR_VALUE,
    OSD_SYS_GOGGLE_VOLTAGE,
    OSD_SYS_VTX_VOLTAGE,
    OSD_SYS_BITRATE,
    OSD_SYS_DELAY,
    OSD_SYS_DISTANCE,
    OSD_SYS_LQ,
    OSD_SYS_GOGGLE_DVR,
    OSD_SYS_VTX_DVR,
    OSD_SYS_WARNINGS,
    OSD_SYS_VTX_TEMP,
    OSD_SYS_FAN_SPEED,
    OSD_DEBUG2,
    OSD_CUSTOM_MSG0,
    OSD_CUSTOM_MSG1,
    OSD_CUSTOM_MSG2,
    OSD_CUSTOM_MSG3,
    OSD_LIDAR_DIST,
    OSD_ITEM_COUNT // MUST BE LAST
} osd_items_e;

// *** IMPORTANT ***
// Whenever new elements are added to 'osd_items_e', make sure to increment
// the parameter group version for 'osdConfig' in 'osd.c'

// *** IMPORTANT ***
// DO NOT REORDER THE STATS ENUMERATION. The order here cooresponds to the enabled flag bit position
// storage and changing the order will corrupt user settings. Any new stats MUST be added to the end
// just before the OSD_STAT_COUNT entry. YOU MUST ALSO add the new stat to the
// osdStatsDisplayOrder array in osd.c.
//
// IF YOU WANT TO REORDER THE STATS DISPLAY, then adjust the ordering of the osdStatsDisplayOrder array
typedef enum {
    OSD_STAT_RTC_DATE_TIME,
    OSD_STAT_TIMER_1,
    OSD_STAT_TIMER_2,
    OSD_STAT_MAX_SPEED,
    OSD_STAT_MAX_DISTANCE,
    OSD_STAT_MIN_BATTERY,
    OSD_STAT_END_BATTERY,
    OSD_STAT_BATTERY,
    OSD_STAT_MIN_RSSI,
    OSD_STAT_MAX_CURRENT,
    OSD_STAT_USED_MAH,
    OSD_STAT_MAX_ALTITUDE,
    OSD_STAT_BLACKBOX,
    OSD_STAT_BLACKBOX_NUMBER,
    OSD_STAT_MAX_G_FORCE,
    OSD_STAT_MAX_ESC_TEMP,
    OSD_STAT_MAX_ESC_RPM,
    OSD_STAT_MIN_LINK_QUALITY,
    OSD_STAT_FLIGHT_DISTANCE,
    OSD_STAT_MAX_FFT,
    OSD_STAT_TOTAL_FLIGHTS,
    OSD_STAT_TOTAL_TIME,
    OSD_STAT_TOTAL_DIST,
    OSD_STAT_MIN_RSSI_DBM,
    OSD_STAT_WATT_HOURS_DRAWN,
    OSD_STAT_MIN_RSNR,
    OSD_STAT_BEST_3_CONSEC_LAPS,
    OSD_STAT_BEST_LAP,
    OSD_STAT_FULL_THROTTLE_TIME,
    OSD_STAT_FULL_THROTTLE_COUNTER,
    OSD_STAT_AVG_THROTTLE,
    OSD_STAT_COUNT // MUST BE LAST
} osd_stats_e;

// Make sure the number of stats do not exceed the available 32bit storage
STATIC_ASSERT(OSD_STAT_COUNT <= 32, osdstats_overflow);

typedef enum {
    OSD_TIMER_1,
    OSD_TIMER_2,
    OSD_TIMER_COUNT
} osd_timer_e;


typedef struct osdConfig_s {
    // Alarms
    uint16_t cap_alarm;
    uint16_t alt_alarm;
    uint8_t rssi_alarm;

    uint8_t units;

    uint16_t timers[OSD_TIMER_COUNT];
    uint32_t enabledWarnings;

    uint8_t ahMaxPitch;
    uint8_t ahMaxRoll;
    uint32_t enabled_stats;
    uint8_t esc_temp_alarm;
    int16_t esc_rpm_alarm;
    int16_t esc_current_alarm;
    uint8_t core_temp_alarm;
    uint8_t ahInvert;                         // invert the artificial horizon
    uint8_t osdProfileIndex;
    uint8_t overlay_radio_mode;
    char profile[OSD_PROFILE_COUNT][OSD_PROFILE_NAME_LENGTH + 1];
    uint16_t link_quality_alarm;
    int16_t rssi_dbm_alarm;
    int16_t rsnr_alarm;
    uint8_t gps_sats_show_pdop;
    int8_t rcChannels[OSD_RCCHANNELS_COUNT];  // RC channel values to display, -1 if none
    uint8_t displayPortDevice;                // osdDisplayPortDevice_e
    uint16_t distance_alarm;
    uint8_t logo_on_arming;                   // show the logo on arming
    uint8_t logo_on_arming_duration;          // display duration in 0.1s units
    uint8_t camera_frame_width;               // The width of the box for the camera frame element
    uint8_t camera_frame_height;              // The height of the box for the camera frame element
    uint16_t framerate_hz;
    uint8_t cms_background_type;              // For supporting devices, determines whether the CMS background is transparent or opaque
    uint8_t stat_show_cell_value;
#ifdef USE_CRAFTNAME_MSGS
    uint8_t osd_craftname_msgs;               // Insert LQ/RSSI-dBm and warnings into CraftName
#endif //USE_CRAFTNAME_MSGS
    uint8_t aux_channel;
    uint16_t aux_scale;
    uint8_t aux_symbol;
    uint8_t canvas_cols;                      // Canvas dimensions for HD display
    uint8_t canvas_rows;
#ifdef USE_OSD_QUICK_MENU
    uint8_t osd_use_quick_menu;               // use QUICK menu YES/NO
#endif // USE_OSD_QUICK_MENU
#ifdef USE_SPEC_PREARM_SCREEN
    uint8_t osd_show_spec_prearm;
#endif // USE_SPEC_PREARM_SCREEN
    displayPortSeverity_e arming_logo;        // font from which to display logo on arming
} osdConfig_t;

PG_DECLARE(osdConfig_t, osdConfig);


typedef struct osdElementConfig_s {
    uint16_t item_pos[OSD_ITEM_COUNT];
} osdElementConfig_t;

PG_DECLARE(osdElementConfig_t, osdElementConfig);
