/*
 * Protocol_P3I.cpp
 * Encoder and decoder for PilotAware using ADS-L LDR O-Band
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

#include <stdint.h>

#include <protocol.h>

#include "../../../SoftRF.h"
#include "../../driver/RF.h"
#include "../../driver/Settings.h"

// this is the ADS-L O-Band LDR protocol:

#include "ADSL.h"
#include "PAW.h"

const rf_proto_desc_t paw_proto_desc = {
  .name            = {'P','A','W', 0},
  .type             = RF_PROTOCOL_PAW,
  .modulation_type  = RF_MODULATION_TYPE_2FSK,
  .preamble_type    = P3I_PREAMBLE_TYPE,
  .preamble_size    = P3I_PREAMBLE_SIZE,
  .syncword         = P3I_SYNCWORD,
  .syncword_size    = P3I_SYNCWORD_SIZE,
  .syncword_skip    = 0,
  .net_id           = P3I_NET_ID,
  .payload_type     = RF_PAYLOAD_DIRECT,
  .payload_size     = P3I_PAYLOAD_SIZE,
  .payload_offset   = P3I_PAYLOAD_OFFSET,
  .crc_type         = P3I_CRC_TYPE,
  .crc_size         = P3I_CRC_SIZE,

  .bitrate          = RF_BITRATE_38400,
  .deviation        = P3I_FDEV,
  .whitening        = RF_WHITENING_NONE,  // RF_WHITENING_NICERF,
  .bandwidth        = P3I_BANDWIDTH,

  .air_time         = P3I_AIR_TIME,

  .tm_type          = RF_TIMING_INTERVAL,
  .tx_interval_min  = P3I_TX_INTERVAL_MIN,
  .tx_interval_max  = P3I_TX_INTERVAL_MAX,
  .slot0            = {0, 0},
  .slot1            = {0, 0}
};

bool paw_decode(void *pkt, container_t *this_aircraft, ufo_t *fop)
{
    // need to check the ADS-L CRC embedded in the PAW packet
    // - RF.cpp only checked the CRC8 external to the PAW payload
    //if (ADSL_Packet::checkPI((uint8_t  *) pkt, (uint8_t) P3I_PAYLOAD_SIZE))
    // use table-driven version instead:
    if (check_adsl_crc((const uint8_t *) pkt, (uint8_t) P3I_PAYLOAD_SIZE)) {
        Serial.println("PAW internal CRC24 wrong");
        return false;
    }

    ++rx_packets_counter;

    if (adsl_decode(pkt, this_aircraft, fop) == false)
        return false;

    fop->protocol = RF_PROTOCOL_PAW;
    return true;
}

size_t paw_encode(void *pkt, container_t *aircraft) {

  // if not airborne, transmit only once in 8 seconds
  if (ThisAircraft.airborne == 0 && ThisAircraft.timestamp < ThisAircraft.positiontime + 8 && (! test_mode))
      return 0;   // otherwise adsl_encode() will return 0

  size_t size = adsl_encode(pkt, aircraft);
  if (size != ADSL_PAYLOAD_SIZE + ADSL_CRC_SIZE  // 24
   || size != P3I_PAYLOAD_SIZE) {                // 24
Serial.print("paw_encode() error: adsl_encode() returned ");
Serial.println(size);
      return 0;
  }

/*
  // no need to compute the CRC since adsl_encode() already called setCRC()
  uint32_t crc = ADSL_Packet::calcPI((const uint8_t *) ptk, (uint8_t) ADSL_PAYLOAD_SIZE);
  uint8_t *CRC = ((uint8_t *) pkt) + ADSL_PAYLOAD_SIZE;
  CRC[0]=crc>>16; CRC[1]=crc>>8; CRC[2]=crc;
*/
  return P3I_PAYLOAD_SIZE;   // 24
}

