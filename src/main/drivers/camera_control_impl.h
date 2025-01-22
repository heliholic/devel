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

#include "pg/camera_control.h"

void cameraControlInit(void);

void cameraControlProcess(uint32_t currentTimeUs);
void cameraControlKeyPress(cameraControlKey_e key, uint32_t holdDurationMs);

void cameraControlHi(void);
void cameraControlLo(void);

void cameraControlSoftwarePwmInit(void);
void cameraControlSoftwarePwmEnable(uint32_t hiTime, uint32_t period);
void cameraControlSoftwarePwmDisable(void);
void cameraControlHardwarePwmInit(timerChannel_t *channel, const timerHardware_t *timerHardware, uint8_t inverted);
