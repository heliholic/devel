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

#include "platform.h"

enum {
    WIGGLE_OFF = 0,
    WIGGLE_ARMED,
    WIGGLE_ERROR,
    WIGGLE_BUZZER,
};


bool wiggleActive(void);
float wiggleGetAxis(int axis);
void wiggleTrigger(int action, int parmam);

void wiggleUpdate(timeUs_t wiggleUpdate);
void wiggleInit(void);

