/*
 * ES1090.h
 * Includes GNS5892 interface
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef PROTOCOL_ES1090_H
#define PROTOCOL_ES1090_H

#include <protocol.h>

#define ES1090_PREAMBLE_TYPE   RF_PREAMBLE_TYPE_AA
#define ES1090_PREAMBLE_SIZE   0 /* N/A */

#define ES1090_SYNCWORD        { 0x00 } /* 0x0285 , LSB first */
#define ES1090_SYNCWORD_SIZE   0
#define ES1090_CRC_TYPE        RF_CHECKSUM_TYPE_CRC_MODES
#define ES1090_CRC_SIZE        3
#define MODE_S_LONG_MSG_BYTES  (112/8)
#define ES1090_PAYLOAD_SIZE    (MODE_S_LONG_MSG_BYTES - ES1090_CRC_SIZE)

#define ES1090_AIR_TIME        1 /* 0.5 ms */

#define ES1090_TX_INTERVAL_MIN 900 /* in ms */ /* TBD */
#define ES1090_TX_INTERVAL_MAX 1000            /* TBD */

extern const rf_proto_desc_t es1090_proto_desc;

bool   es1090_decode(void *, container_t *, ufo_t *);
size_t es1090_encode(void *, container_t *);

// GNS5892 interface

void play5892(void);
void gns5892_setup(void);
void gns5892_loop(void);
void save_zone_stats(void);
void show_zone_stats(void);

void gns5892_test_mode(void);

extern bool gns5892_found;

#endif /* PROTOCOL_ES1090_H */
