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

#include "io/serial.h"
#include "common/printf.h"

// Either define it here, or on the command line
//#define USE_SERIAL_DPRINTF

#ifdef USE_SERIAL_DPRINTF

void initDebugSerial(serialPortIdentifier_e port, uint32_t baudRate);

int dprintf(const char *fmt, ...);

#else

static inline void initDebugSerial(__unused serialPortIdentifier_e port, __unused uint32_t baudRate) {}

static inline int dprintf(__unused const char *fmt, ...) { return 0; }

#endif /* USE_SERIAL_DPRINTF */
