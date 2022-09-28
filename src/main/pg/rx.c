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

#include "platform.h"

#if defined(USE_PWM) || defined(USE_PPM) || defined(USE_SERIAL_RX) || defined(USE_RX_MSP) || defined(USE_RX_SPI)

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "rx.h"

#include "config/config_reset.h"

#include "drivers/io.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "rx/rx.h"
#include "rx/rx_spi.h"

PG_REGISTER_WITH_RESET_FN(rxConfig_t, rxConfig, PG_RX_CONFIG, 3);
void pgResetFn_rxConfig(rxConfig_t *rxConfig)
{
    RESET_CONFIG_2(rxConfig_t, rxConfig,
        .halfDuplex = 0,
        .serialrx_provider = SERIALRX_PROVIDER,
        .serialrx_inverted = 0,
        .spektrum_bind_pin_override_ioTag = IO_TAG(SPEKTRUM_BIND_PIN),
        .spektrum_bind_plug_ioTag = IO_TAG(BINDPLUG_PIN),
        .spektrum_sat_bind = 0,
        .spektrum_sat_bind_autoreset = 1,
        .midrc = RX_MID_USEC,
        .mincheck = 1050,
        .maxcheck = 1950,
        .rx_min_usec = RX_MIN_USEC,          // any of first 4 channels below this value will trigger rx loss detection
        .rx_max_usec = RX_MAX_USEC,         // any of first 4 channels above this value will trigger rx loss detection
        .rssi_src_frame_errors = false,
        .rssi_channel = 0,
        .rssi_scale = RSSI_SCALE_DEFAULT,
        .rssi_offset = 0,
        .rssi_invert = 0,
        .rssi_src_frame_lpf_period = 30,
        .max_aux_channel = DEFAULT_AUX_CHANNEL_COUNT,
        .srxl2_unit_id = 1,
        .srxl2_baud_fast = true,
        .sbus_baud_fast = false,
        .crsf_use_rx_snr = false,
        .msp_override_channels_mask = 0,
        .crsf_use_negotiated_baud = false,
    );

#ifdef RX_CHANNELS_TAER
    parseRcChannels("TAER1C23", rxConfig);
#else
    parseRcChannels("AETRC123", rxConfig);
#endif
}

#endif
