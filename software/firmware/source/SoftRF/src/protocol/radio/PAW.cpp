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


#if 1
// for now also decode (but not encode) old-protocol packets:

// why this is 56 bytes when only 24 are used?
/*
const uint8_t whitening_pattern[] PROGMEM = { 0x05, 0xb4, 0x05, 0xae, 0x14, 0xda,
  0xbf, 0x83, 0xc4, 0x04, 0xb2, 0x04, 0xd6, 0x4d, 0x87, 0xe2, 0x01, 0xa3, 0x26,
  0xac, 0xbb, 0x63, 0xf1, 0x01, 0xca, 0x07, 0xbd, 0xaf, 0x60, 0xc8, 0x12, 0xed,
  0x04, 0xbc, 0xf6, 0x12, 0x2c, 0x01, 0xd9, 0x04, 0xb1, 0xd5, 0x03, 0xab, 0x06,
  0xcf, 0x08, 0xe6, 0xf2, 0x07, 0xd0, 0x12, 0xc2, 0x09, 0x34, 0x20 };
*/
const uint8_t whitening_pattern[sizeof(p3i_packet_t)] PROGMEM = {
  0x05, 0xb4, 0x05, 0xae, 0x14, 0xda, 0xbf, 0x83,
  0xc4, 0x04, 0xb2, 0x04, 0xd6, 0x4d, 0x87, 0xe2,
  0x01, 0xa3, 0x26, 0xac, 0xbb, 0x63, 0xf1, 0x01
};

bool p3i_decode(void *p3i_pkt, container_t *this_aircraft, ufo_t *fop)
{
  p3i_packet_t *pkt = (p3i_packet_t *) p3i_pkt;

  uint8_t cs = 0;
  uint8_t *p = (uint8_t *)pkt;
  for (int i=0; i<sizeof(p3i_packet_t); i++) {
    *p ^= pgm_read_byte(&whitening_pattern[i]);
    cs ^= *p++;
  }
  if (cs) {
    Serial.println("P3I internal CS8 wrong");
    return(false);
  }

  if (pkt->sync != '$') {
    Serial.println("P3I sync byte not $");
    return(false);         // reject the occasional other type of packet
  }

  ++rx_packets_counter;

  fop->protocol = RF_PROTOCOL_P3I;

  fop->addr = pkt->icao;

  if (fop->addr == settings->ignore_id)
         return false;                 /* ID told in settings to ignore */
  if (fop->addr == ThisAircraft.addr)
         return false;                 /* same ID as this aircraft - ignore */

  fop->addr_type = (fop->addr >= 0xFF0000? ADDR_TYPE_FLARM : ADDR_TYPE_ICAO);

  fop->timestamp = (uint32_t) this_aircraft->timestamp;
  fop->gnsstime_ms = millis();

  fop->latitude = pkt->latitude;
  fop->longitude = pkt->longitude;
  fop->altitude = (float) pkt->altitude;
  fop->aircraft_type = (pkt->aircraft & 0x0F);   // higher bits signal packet is relayed
  fop->course = (float) pkt->track;
  fop->speed = (float) pkt->knots;

  fop->vs = 0;
  fop->stealth = 0;
  fop->no_track = 0;

  return true;
}
#endif

bool paw_decode(void *pkt, container_t *this_aircraft, ufo_t *fop)
{
    // need to check the ADS-L CRC embedded in the PAW packet
    // - RF.cpp only checked the CRC8 external to the PAW payload
    //if (ADSL_Packet::checkPI((uint8_t  *) pkt, (uint8_t) P3I_PAYLOAD_SIZE))
    // use table-driven version instead:
    if (check_adsl_crc((const uint8_t *) pkt, (uint8_t) P3I_PAYLOAD_SIZE)) {
        Serial.println("PAW internal CRC24 wrong");
#if 1
        Serial.println("Trying old P3I protocol...");
        return p3i_decode(pkt, this_aircraft, fop);
#else
        return false;
#endif
    }

    ++rx_packets_counter;

    if (adsl_decode(pkt, this_aircraft, fop) == false)
        return false;

    if (fop->addr_type != ADDR_TYPE_FLARM && fop->addr_type != ADDR_TYPE_ICAO) {
        // commercial PAW devices send addr_type 4, translated by ads-l.h to 0
        fop->addr_type = (fop->addr >= 0xFF0000? ADDR_TYPE_FLARM : ADDR_TYPE_ICAO);
    }

    fop->protocol = RF_PROTOCOL_PAW;
    return true;
}

size_t paw_encode(void *pkt, container_t *aircraft) {

  // if not airborne, transmit only once in 8 seconds
  // - but ignore that if dual PAW+FANET mode, since FANET will grab the slot
  static uint32_t paw_positiontime = 0;
  if (ThisAircraft.airborne == 0 && ThisAircraft.timestamp < paw_positiontime + 8
           && dual_protocol != RF_FANET_PAW && dual_protocol != RF_PAW_FANET && (! test_mode)) {
      RF_Transmit_Postpone(true);
      return 0;
  }
  paw_positiontime = ThisAircraft.timestamp;
  uint32_t adsl_positiontime = ThisAircraft.positiontime;
  ThisAircraft.positiontime = 0;              // otherwise adsl_encode() will return 0
  size_t size = adsl_encode(pkt, aircraft);
  ThisAircraft.positiontime = adsl_positiontime;
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

