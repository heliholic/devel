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

#include "pg/pg.h"

#define DEFAULT_SERVO_UPDATE    333

#define SERVO_OVERRIDE_MIN     -2000
#define SERVO_OVERRIDE_MAX      2000
#define SERVO_OVERRIDE_OFF      (SERVO_OVERRIDE_MAX + 1)

#define SERVO_TYPE_NORMAL       0
#define SERVO_TYPE_NARROW       1

#define SERVO_FLAGS_TYPE        (BIT(0) | BIT(1) | BIT(2))
#define SERVO_FLAGS_LINEAR      BIT(6)
#define SERVO_FLAGS_REVERSED    BIT(7)

typedef struct servoParam_s {
    uint8_t     servo_flags;   // Servo type, reverse, linear
    int8_t      servo_trim;    // Servo trim (x0.1%)
    int8_t      high_trim;     // High trim (x0.1%)
    int8_t      low_trim;      // Low trim (x0.1%)
    int8_t      high_limit;    // High travel limit (x0.2%)
    int8_t      low_limit;     // Low travel limit (x0.2%)
} servoParam_t;

PG_DECLARE_ARRAY(servoParam_t, MAX_SUPPORTED_SERVOS, servoParams);

typedef struct servoConfig_s {
    servoDevConfig_t dev;
} servoConfig_t;

PG_DECLARE(servoConfig_t, servoConfig);

void servoInit(void);
void servoInitConfig(void);
void servoUpdate(void);

uint8_t getServoCount(void);
int16_t getServoOutput(uint8_t servo);

bool    hasServoOverride(uint8_t servo);
int16_t getServoOverride(uint8_t servo);
int16_t setServoOverride(uint8_t servo, int16_t val);

