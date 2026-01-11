/*
 * Protocol_ADSL.cpp
 *
 * Encoder and decoder for ADS-L SRD-860 radio protocol
 * Copyright (C) 2024-2025 Linar Yusupov
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

/*
 * Issue 1:
 * https://www.easa.europa.eu/sites/default/files/dfu/ads-l_4_srd860_issue_1.pdf
 */

#include <stdint.h>

#include <protocol.h>
#include <TimeLib.h>

#include "../../../SoftRF.h"
#include "../../TrafficHelper.h"
#include "../../driver/RF.h"
#include "../../driver/GNSS.h"
#include "../../driver/Settings.h"
#include "../../system/Time.h"
#include "Legacy.h"

const rf_proto_desc_t adsl_proto_desc = {
  .name            = {'A','D','S','-','L', 0},
  .type            = RF_PROTOCOL_ADSL,
  .modulation_type = RF_MODULATION_TYPE_2FSK,
  .preamble_type   = ADSL_PREAMBLE_TYPE,
  .preamble_size   = ADSL_PREAMBLE_SIZE,
  .syncword        = ADSL_SYNCWORD,
  .syncword_size   = ADSL_SYNCWORD_SIZE,
  .syncword_skip   = ADSL_SYNCWORD_SKIP,
  .net_id          = 0x0000, /* not in use */
  .payload_type    = RF_PAYLOAD_INVERTED,
  .payload_size    = ADSL_PAYLOAD_SIZE,
  .payload_offset  = 0,
  .crc_type        = ADSL_CRC_TYPE,
  .crc_size        = ADSL_CRC_SIZE,

  .bitrate         = RF_BITRATE_100KBPS,
  .deviation       = RF_FREQUENCY_DEVIATION_50KHZ,
  .whitening       = RF_WHITENING_MANCHESTER,
  .bandwidth       = RF_RX_BANDWIDTH_SS_125KHZ,

  .air_time        = ADSL_AIR_TIME,

#if defined(USE_TIME_SLOTS)
  .tm_type         = RF_TIMING_2SLOTS_PPS_SYNC,
#else
  .tm_type         = RF_TIMING_INTERVAL,
#endif
  .tx_interval_min = ADSL_TX_INTERVAL_MIN,
  .tx_interval_max = ADSL_TX_INTERVAL_MAX,
  .slot0           = {400,  800},
  .slot1           = {800, 1200}
};

static GPS_Position pos;
static ADSL_Packet  r __attribute__((aligned(sizeof(uint32_t)))); /* Rx */
static ADSL_Packet  t __attribute__((aligned(sizeof(uint32_t)))); /* Tx */

void adsl_init()
{
  pos.Clear();
}

void adsl_fini()
{

}

bool adsl_decode(void *pkt, container_t *this_aircraft, ufo_t *fop) {

  uint8_t *ptr = (uint8_t *) pkt;

  r.Init();
  r.Version = *ptr;
  ptr += sizeof(ADSL_Packet::Version);

  for (int Idx=0; Idx<5; Idx++) {
    r.Word[Idx] = r.get4bytes(ptr + Idx * sizeof(r.Word[0]));
  }

  r.Descramble();

  fop->protocol  = RF_PROTOCOL_ADSL;

  fop->addr      = r.getAddress();

  if (fop->addr == settings->ignore_id)
         return false;                  /* ID told in settings to ignore */

  if (fop->addr == ThisAircraft.addr) {
      if (landed_out_mode) {
          // if "seeing itself" is via requested relay, show it
          // Serial.println("Received own ID - relayed as landed out");
      } else {
          Serial.println("warning: received same ID as this aircraft");
          return false;
      }
  }

  for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {
    container_t *cip = &Container[i];
    if (cip->addr == fop->addr) {
      if (cip->protocol == RF_PROTOCOL_LATEST
          && OurTime <= cip->timestamp + EXPORT_EXPIRATION_TIME) {
                    // 5s - not ENTRY_EXPIRATION_TIME (30s)
                    // since that takes too long after FLARM reception drops out
          // already tracked via other means
          //Serial.println("ADSL traffic also seen via Latest, ignore");
          return false;
      }
      if (RF_last_crc != 0 && RF_last_crc == cip->last_crc) {
          //Serial.println("duplicate packet");
          return false;
      }
      break;
    }
  }
  fop->last_crc = RF_last_crc;

  fop->latitude  = r.FNTtoFloat(r.getLat());
  fop->longitude = r.FNTtoFloat(r.getLon());
  fop->altitude  = (float) r.getAlt();
  fop->pressure_altitude = 0; /* TBD */
  fop->aircraft_type = r.getAcftTypeOGN();
  fop->course    = (45.0/0x40) * r.getTrack();
  fop->speed     = (0.25/ _GPS_MPS_PER_KNOT) * r.getSpeed();
  fop->vs        = (0.125 * _GPS_FEET_PER_METER * 60.0) * r.getClimb();
  fop->hdop      = r.getHorAccur();

  fop->addr_type = r.getAddrTypeOGN();
  fop->timestamp = (uint32_t) RF_time;      // this_aircraft->timestamp;
  fop->gnsstime_ms = millis();

  fop->airborne = (r.FlightState != 1);    // >>> ads-l.h lacks a method to read the "flight status" field?

  fop->stealth   = 0;
  fop->no_track  = 0;
  fop->relayed   = r.getRelay();

  return true;
}

size_t adsl_encode(void *pkt, container_t *aircraft) {

  // if not airborne, transmit only once in 8 seconds
  if (aircraft->airborne == 0 && (RF_time & 0x07 != 0x07))
      return 0;

  pos.Latitude  = (int32_t) (aircraft->latitude * 600000);
  pos.Longitude = (int32_t) (aircraft->longitude * 600000);
  pos.Altitude  = (int32_t) (aircraft->altitude * 10);        // height above ellipsoid
  if (aircraft->pressure_altitude != 0.0)
    pos.StdAltitude = (int32_t) (aircraft->pressure_altitude * 10);
  pos.ClimbRate = aircraft->stealth ?
                    0 : (int32_t) (aircraft->vs * (1.0 / (_GPS_FEET_PER_METER * 6.0)));
  pos.Heading = (int16_t) (aircraft->course * 10);
  pos.Speed   = (int16_t) (aircraft->speed * (10.0 * _GPS_MPS_PER_KNOT));
  pos.HDOP    = (uint8_t) (aircraft->hdop / 10);

  int second = gnss.time.second();
  if (leap_seconds_correction != 0) {
      second -= (int) leap_seconds_correction;
      if (second < 0)   second += 60;
      if (second > 59)  second -= 60;
  }
  pos.Sec     = second;
  pos.FracSec = 0;    // gnss.time.centisecond() is empty

  t.Init();

  t.setAddress(aircraft->addr);

  uint8_t aircraft_type = aircraft->aircraft_type;

  if (aircraft != &ThisAircraft) {   // relaying another aircraft
      t.setAddrTypeOGN(aircraft->addr_type);
      t.setRelay(1);
  } else {
      uint8_t addr_type = settings->id_method;
      if (addr_type == ADDR_TYPE_OVERRIDE)
          addr_type = ADDR_TYPE_FLARM;
      t.setAddrTypeOGN(addr_type);
      t.setRelay(0);
      if (landed_out_mode)
          aircraft_type = AIRCRAFT_TYPE_UNKNOWN;        // mark this aircraft as landed-out
  }

  if (aircraft_type == AIRCRAFT_TYPE_WINCH) {
      aircraft_type = AIRCRAFT_TYPE_STATIC;
      t.FlightState = 2;   // pretend to be airborne (with variable altitude)
  } else {
      t.FlightState = (aircraft->airborne? 2 : 1);   // >>> t.Meta.FlightState ?
  }

  t.setAcftTypeOGN((int16_t) aircraft_type);

  pos.Encode(t);
  t.Scramble();
  t.setCRC();

  memcpy((void *) pkt, &t.Version, t.Length);

  return (t.Length);
}
