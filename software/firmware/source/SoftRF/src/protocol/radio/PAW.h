/*
 * Pilot Aware P3I.h - using the ADS-L LDR O-Band protocol
 *
 * Encoder and decoder for ADS-L SRD-860 protocol embedded within P3I packet
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

#ifndef PROTOCOL_PAW_H
#define PROTOCOL_PAW_H

#define P3I_PREAMBLE_TYPE   RF_PREAMBLE_TYPE_AA
#define P3I_PREAMBLE_SIZE   10
#define P3I_SYNCWORD        {0xb4, 0x2b}
#define P3I_SYNCWORD_SIZE   2
#define P3I_NET_ID          0x00000000
#define P3I_PAYLOAD_SIZE    24
#define P3I_PAYLOAD_OFFSET  6
#define P3I_CRC_TYPE        RF_CHECKSUM_TYPE_CRC8_107
#define P3I_CRC_SIZE        1

//#define P3I_FDEV            RF_FREQUENCY_DEVIATION_19_2KHZ
#define P3I_FDEV            RF_FREQUENCY_DEVIATION_12_5KHZ
#define P3I_BANDWIDTH       RF_RX_BANDWIDTH_SS_50KHZ

#define P3I_AIR_TIME        10 /* in ms */

#define P3I_TX_INTERVAL_MIN 1600 /* in ms */
#define P3I_TX_INTERVAL_MAX 1800

/*
#define ADSL_PAYLOAD_SIZE    21
#define ADSL_CRC_TYPE        RF_CHECKSUM_TYPE_CRC_MODES
#define ADSL_CRC_SIZE        3
*/

/*  RF frame:
 *  ---------
 * +--------------+-------------------+------------------+
 * | Size (bits)  |   Description     |     Value        |
 * +--------------+-------------------+------------------+
 * |      2       |      Warmup       |                  |
 * +--------------+-------------------+------------------+
 * |      80      |    Preamble       |  0xAA,...,0xAA   |
 * +--------------+-------------------+------------------+
 * |      16      |    Syncword       |    0xb4, 0x2b    |
 * +--------------+-------------------+------------------+
 * |      32      |     Net ID        |  0x00,...,0x00   |
 * +--------------+-------------------+------------------+
 * |      8       |  Payload length   |     0x18 (24)    |
 * +--------------+-------------------+------------------+
 * |      8       |  CRC seed value   |        0x71      |
 * +--------------+-------------------+------------------+
 * |      192     | 21-byte ADS-L packet + 3-byte CRC    |
 * +--------------+-------------------+------------------+
 * |      8       |  CRC-8, POLY_107  |                  |
 * +--------------+-------------------+------------------+
 * |      4       |      Cooldown     |                  |
 * +--------------+-------------------+------------------+
 */

#include <ads-l.h>

typedef struct {
    uint32_t p3i_words[6];
} paw_packet_t;

extern const rf_proto_desc_t paw_proto_desc;

bool paw_decode(void *, container_t *, ufo_t *);
size_t paw_encode(void *, container_t *);

#endif /* PROTOCOL_PAW_H */
