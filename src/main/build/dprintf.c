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

#include "types.h"

#include "dprintf.h"

#ifdef USE_SERIAL_DPRINTF

#include "drivers/serial.h"

static serialPort_t * debugSerialPort = NULL;

void initDebugSerial(serialPortIdentifier_e port)
{
    debugSerialPort = openSerialPort(port, FUNCTION_NONE, NULL, NULL, 921600, MODE_TX, 0);
    dprintf("\r\nDebug port ready\r\n");
}

static void __dputc(void *p, char c)
{
    UNUSED(p);
    serialWrite(debugSerialPort, c);
}

int dprintf(const char *fmt, ...)
{
    if (debugSerialPort) {
        va_list va;
        va_start(va, fmt);
        int written = tfp_format(NULL, __dputc, fmt, va);
        va_end(va);
        return written;
    }

    return 0;
}

#endif
