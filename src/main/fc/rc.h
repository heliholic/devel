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

#include "drivers/time.h"

#include "fc/rc_controls.h"
#include "fc/rc_rates.h"
#include "fc/rc_modes.h"
#include "fc/rc_smoothing.h"


extern float rcCommand[5];

void initRcProcessing(void);
void processRcCommand(void);
void updateRcCommands(void);

float getRcDeflection(int axis);
float getRawSetpoint(int axis);
float getRcCommandDelta(int axis);

void resetYawAxis(void);

void updateRcRefreshRate(timeUs_t currentTimeUs);
uint16_t getCurrentRxRefreshRate(void);

bool getRxRateValid(void);
