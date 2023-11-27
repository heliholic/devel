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

//
// fixed ids, voltage can be measured at many different places, these identifiers are the ones we support or would consider supporting.
//

typedef enum {
    VOLTAGE_METER_ID_NONE = 0,

    VOLTAGE_METER_ID_BATTERY = 10,          // 10-19 for battery meters
    VOLTAGE_METER_ID_BEC = 20,              // 20-29 for BEC / servo meters
    VOLTAGE_METER_ID_BUS = 30,              // 30-39 for BUS / 5V meters
    VOLTAGE_METER_ID_EXT = 40,              // 40-49 for EXT meters
    VOLTAGE_METER_ID_MCU = 50,              // 50-59 for MCU / 3.3V meters
    VOLTAGE_METER_ID_ESC_COMBINED = 60,     // 60 for ESC combined
    VOLTAGE_METER_ID_ESC_1 = 61,            // 61-79 for ESC voltages
    VOLTAGE_METER_ID_ESC_2,
    VOLTAGE_METER_ID_ESC_3,
    VOLTAGE_METER_ID_ESC_4,
    VOLTAGE_METER_ID_CELLS = 80,            // 80-119 for cell meters (40 cells)

} voltageMeterId_e;
