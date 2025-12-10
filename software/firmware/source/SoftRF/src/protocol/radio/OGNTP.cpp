/*
 *
 * Protocol_OGNTP.cpp
 * Encoder and decoder for Open Glider Network tracker radio protocol
 * Copyright (C) 2017-2021 Linar Yusupov
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
#include <TimeLib.h>

#include "../../../SoftRF.h"
#include "../../driver/RF.h"
#include "../../driver/GNSS.h"
#include "../../driver/Settings.h"
#include "Legacy.h"

const rf_proto_desc_t ogntp_proto_desc = {
  "OGNTP",
  .type            = RF_PROTOCOL_OGNTP,
  .modulation_type = RF_MODULATION_TYPE_2FSK,
  .preamble_type   = OGNTP_PREAMBLE_TYPE,
  .preamble_size   = OGNTP_PREAMBLE_SIZE,
  .syncword        = OGNTP_SYNCWORD,
  .syncword_size   = OGNTP_SYNCWORD_SIZE,
  .syncword_skip   = OGNTP_SYNCWORD_SKIP,
  .net_id          = 0x0000, /* not in use */
  .payload_type    = RF_PAYLOAD_INVERTED,
  .payload_size    = OGNTP_PAYLOAD_SIZE,
  .payload_offset  = 0,
  .crc_type        = OGNTP_CRC_TYPE,
  .crc_size        = OGNTP_CRC_SIZE,

  .bitrate         = RF_BITRATE_100KBPS,
  .deviation       = RF_FREQUENCY_DEVIATION_50KHZ,
  .whitening       = RF_WHITENING_MANCHESTER,
  .bandwidth       = RF_RX_BANDWIDTH_SS_125KHZ,

  .air_time        = OGNTP_AIR_TIME,

#if defined(USE_TIME_SLOTS)
  .tm_type         = RF_TIMING_2SLOTS_PPS_SYNC,
#else
  .tm_type         = RF_TIMING_INTERVAL,
#endif
  .tx_interval_min = OGNTP_TX_INTERVAL_MIN,
  .tx_interval_max = OGNTP_TX_INTERVAL_MAX,
  .slot0           = {400,  800},
  .slot1           = {800, 1200}
};

static GPS_Position pos;
static OGN_TxPacket ogn_tx_pkt;
static OGN_RxPacket ogn_rx_pkt;

void ogntp_init()
{
  pos.Clear();
  ogn_rx_pkt.Clear();
}

void ogntp_fini()
{

}

bool ogntp_decode(void *pkt, container_t *this_aircraft, ufo_t *fop) {

  uint32_t *key = settings->igc_key;

  ogn_rx_pkt.recvBytes((uint8_t *) pkt);

/* that has been alreay done by RFHelper */
//  if (ogn_rx_pkt.checkFEC()) {
//    return false;
//  }

  if ( ogn_rx_pkt.Packet.Header.Other ||
#if defined(USE_OGN_ENCRYPTION)
      (ogn_rx_pkt.Packet.Header.Encrypted &&
       !(key[0] || key[1] || key[2] || key[3]))
#else
       ogn_rx_pkt.Packet.Header.Encrypted
#endif
     ) {
    return false;
  }

#if !defined(SOFTRF_ADDRESS)
  uint8_t addr_type = ADDR_TYPE_OGN;
#else
  uint8_t addr_type = (this_aircraft->addr == SOFTRF_ADDRESS ?
                        ADDR_TYPE_ICAO : ADDR_TYPE_OGN);
#endif

  /* ignore this device own (relayed) packets */
  if ((ogn_rx_pkt.Packet.Header.Address    == this_aircraft->addr) &&
      (ogn_rx_pkt.Packet.Header.AddrType   == addr_type          ) &&
      (ogn_rx_pkt.Packet.Header.RelayCount > 0 )) {
    return false;
  }

#if defined(USE_OGN_ENCRYPTION)
  if (ogn_rx_pkt.Packet.Header.Encrypted)
    ogn_rx_pkt.Packet.Decrypt(key);
  else
#endif
    ogn_rx_pkt.Packet.Dewhiten();

  fop->protocol  = RF_PROTOCOL_OGNTP;

  fop->addr      = ogn_rx_pkt.Packet.Header.Address;

  if (fop->addr == settings->ignore_id)
         return true;                  /* ID told in settings to ignore */
  if (fop->addr == ThisAircraft.addr)
         return true;                  /* same ID as this aircraft - ignore */

  fop->addr_type =
      (ogn_rx_pkt.Packet.Header.AddrType == ADDR_TYPE_ICAO ? ADDR_TYPE_ICAO : ADDR_TYPE_FLARM);
  fop->timestamp = this_aircraft->timestamp;
  fop->gnsstime_ms = millis();

  fop->stealth   = ogn_rx_pkt.Packet.Position.Stealth;
  fop->no_track  = 0;

  fop->latitude  = ogn_rx_pkt.Packet.DecodeLatitude() * 0.0001/60;
  fop->longitude = ogn_rx_pkt.Packet.DecodeLongitude() * 0.0001/60;
  fop->altitude  = (float) ogn_rx_pkt.Packet.DecodeAltitude() + this_aircraft->geoid_separation;  // MSL -> ellipsoid
  fop->pressure_altitude = (float) ogn_rx_pkt.Packet.DecodeStdAltitude();
  fop->aircraft_type = ogn_rx_pkt.Packet.Position.AcftType;
  fop->course    = ogn_rx_pkt.Packet.DecodeHeading() * 0.1;
  fop->speed     = (ogn_rx_pkt.Packet.DecodeSpeed() * 0.1) / _GPS_MPS_PER_KNOT;
  fop->vs        = (ogn_rx_pkt.Packet.DecodeClimbRate() * 0.1) * (_GPS_FEET_PER_METER * 60.0);
  fop->hdop      = (ogn_rx_pkt.Packet.DecodeDOP() + 10) * 10;

  return true;
}

size_t ogntp_encode(void *pkt, container_t *this_aircraft) {

  uint32_t *key = settings->igc_key;

  pos.Latitude  = (int32_t) (this_aircraft->latitude * 600000);
  pos.Longitude = (int32_t) (this_aircraft->longitude * 600000);
  pos.Altitude  = (int32_t) ((this_aircraft->altitude - this_aircraft->geoid_separation) * 10);  // ellipsoid -> MSL

  uint8_t aircraft_type = this_aircraft->aircraft_type;
  if (this_aircraft != &ThisAircraft
    && (this_aircraft->protocol == RF_PROTOCOL_LATEST || this_aircraft->protocol == RF_PROTOCOL_LEGACY)
    && this_aircraft->airborne==0
    && aircraft_type == AIRCRAFT_TYPE_UNKNOWN) {    // relaying landed-out traffic
        // && aircraft->landed_out)
      //aircraft_type = AIRCRAFT_TYPE_GLIDER;       // leave as "unknown" to signal landed-out
  }
  if (aircraft_type == AIRCRAFT_TYPE_WINCH) {
      aircraft_type = AIRCRAFT_TYPE_STATIC;
      pos.ClimbRate = 0;
      pos.StdAltitude = pos.Altitude;
  } else {
      pos.ClimbRate = this_aircraft->stealth ?
           0 : (int32_t) (this_aircraft->vs * (1.0 / (_GPS_FEET_PER_METER * 6.0)));
      if (this_aircraft->pressure_altitude != 0.0)
          pos.StdAltitude = (int32_t) (this_aircraft->pressure_altitude * 10);
      else
          pos.StdAltitude = pos.Altitude;
  }

  pos.Heading = (int16_t) (this_aircraft->course * 10);
  pos.Speed   = (int16_t) (this_aircraft->speed * 10 * _GPS_MPS_PER_KNOT);
  pos.HDOP    = (uint8_t) (ThisAircraft.hdop / 10);   // not this_aircraft-> since no hdop for Legacy

  pos.Encode(ogn_tx_pkt.Packet);

  ogn_tx_pkt.Packet.HeaderWord      = 0;

  ogn_tx_pkt.Packet.Header.Address  = this_aircraft->addr;

  if (this_aircraft != &ThisAircraft) {   // relaying another aircraft
      ogn_tx_pkt.Packet.Header.AddrType = this_aircraft->addr_type;
  } else {
      uint8_t addr_type = settings->id_method;
      if (addr_type == ADDR_TYPE_OVERRIDE)
          addr_type = ADDR_TYPE_FLARM;
      if (addr_type == ADDR_TYPE_FLARM && settings->rf_protocol == RF_PROTOCOL_OGNTP)
          ogn_tx_pkt.Packet.Header.AddrType = ADDR_TYPE_OGN;  
      else   // OGNTP is altprotocol, or ICAO (or random) ID
          ogn_tx_pkt.Packet.Header.AddrType = addr_type;  
  }

#if defined(USE_OGN_ENCRYPTION)
  if (key[0] || key[1] || key[2] || key[3])
    ogn_tx_pkt.Packet.Header.Encrypted = 1;
  else
#endif
    ogn_tx_pkt.Packet.Header.Encrypted = 0;

  ogn_tx_pkt.Packet.calcAddrParity();

  ogn_tx_pkt.Packet.Position.AcftType = (int16_t) aircraft_type;
  ogn_tx_pkt.Packet.Position.Stealth  = (int16_t) this_aircraft->stealth;
  int second = gnss.time.second();
  if (leap_seconds_correction != 0) {
      second -= (int) leap_seconds_correction;
      if (second < 0)   second += 60;
      if (second > 59)  second -= 60;
  }
  ogn_tx_pkt.Packet.Position.Time     = (unsigned) second;   // second();

#if defined(USE_OGN_ENCRYPTION)
  if (ogn_tx_pkt.Packet.Header.Encrypted)
    ogn_tx_pkt.Packet.Encrypt(key);
  else
#endif
    ogn_tx_pkt.Packet.Whiten();

  ogn_tx_pkt.calcFEC();

  memcpy((void *) pkt,  ogn_tx_pkt.Byte(), ogn_tx_pkt.Bytes);
  return (ogn_tx_pkt.Bytes);
}
