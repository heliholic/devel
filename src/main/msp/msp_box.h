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

#include "fc/rc_modes.h"

typedef struct box_s {
    uint8_t boxId;            // See boxId_e
    uint8_t permanentId;      // Permanent ID for the function, DO NOT REUSE IT
    char *boxName;            // GUI-readable box name
} box_t;

#define PERMANENT_ID_NONE 255

const box_t *findBoxByBoxId(boxId_e boxId);
const box_t *findBoxByPermanentId(uint8_t permanentId);

struct boxBitmask_s;
int packFlightModeFlags(struct boxBitmask_s *mspFlightModeFlags);
struct sbuf_s;
int serializeBoxNameFn(struct sbuf_s *dst, const box_t *box);
int serializeBoxPermanentIdFn(struct sbuf_s *dst, const box_t *box);
typedef int serializeBoxFn(struct sbuf_s *dst, const box_t *box);
void serializeBoxReply(struct sbuf_s *dst, int page, serializeBoxFn *serializeBox);
void initActiveBoxIds(void);
bool getBoxIdState(boxId_e boxid);
