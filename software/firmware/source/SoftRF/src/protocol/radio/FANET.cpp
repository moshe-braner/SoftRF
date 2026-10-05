/*
 * Protocol_FANET.cpp
 *
 * Encoder and decoder for open FANET radio protocol
 *
 * URL:
 *    Development -  https://github.com/3s1d/fanet-stm32
 *    Deprecated  -  https://github.com/3s1d/fanet
 *
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
//#include <string.h>

#include <protocol.h>
#include <TimeLib.h>

#include "../../../SoftRF.h"
#include "../../TrafficHelper.h"
#include "../../Wind.h"
#include "../../driver/RF.h"
#include "../../driver/Settings.h"
//#include "../../protocol/data/NMEA.h"
#include "../../protocol/data/FNF.h"
//#include "../../driver/Bluetooth.h"

const rf_proto_desc_t fanet_proto_desc = {
  "FANET",
  .type             = RF_PROTOCOL_FANET,
  .modulation_type  = RF_MODULATION_TYPE_LORA,
  .preamble_type    = 0 /* INVALID FOR LORA */,
  .preamble_size    = 0 /* INVALID FOR LORA */,
#if defined(FANET_DEPRECATED)
  .syncword         = { 0x12 },  // sx127x default value, valid for FANET
#error using FANET_DEPRECATED sync word
#else
  .syncword         = { 0xF1 },  // FANET+
#endif
  .syncword_size    = 1,
  .syncword_skip    = 0,
  .net_id           = 0x0000, /* not in use */
  .payload_type     = RF_PAYLOAD_DIRECT,
  .payload_size     = FANET_PAYLOAD_SIZE,
  .payload_offset   = 0,
  .crc_type         = RF_CHECKSUM_TYPE_NONE, /* LoRa packet has built-in CRC */
  .crc_size         = 0 /* INVALID FOR LORA */,
  .bitrate          = 0,  // radio.cpp will set, was DR_SF7B /* CR_5 BW_250 SF_7 */

  .deviation        = 0 /* INVALID FOR LORA */,
  .whitening        = RF_WHITENING_NONE,
  .bandwidth        = 0, /* INVALID FOR LORA */
     // later upstream code says: RF_RX_BANDWIDTH_SS_125KHZ, /* RF_RX_BANDWIDTH_SS_250KHZ */
     // although the .bandwidth field is not used in FANET

  .air_time         = FANET_AIR_TIME,

  .tm_type          = RF_TIMING_INTERVAL,
  .tx_interval_min  = FANET_TX_INTERVAL_MIN,
  .tx_interval_max  = FANET_TX_INTERVAL_MAX,
  .slot0            = {0, 0},
  .slot1            = {0, 0}
};

const uint8_t aircraft_type_to_fanet[] PROGMEM = {
	FANET_AIRCRAFT_TYPE_OTHER,
	FANET_AIRCRAFT_TYPE_GLIDER,
	FANET_AIRCRAFT_TYPE_POWERED,
	FANET_AIRCRAFT_TYPE_HELICOPTER,
	FANET_AIRCRAFT_TYPE_PARAGLIDER,   // changed from OTHER, following later upstream code
	FANET_AIRCRAFT_TYPE_POWERED,
	FANET_AIRCRAFT_TYPE_HANGGLIDER,
	FANET_AIRCRAFT_TYPE_PARAGLIDER,
	FANET_AIRCRAFT_TYPE_POWERED,
	FANET_AIRCRAFT_TYPE_POWERED,
	FANET_AIRCRAFT_TYPE_OTHER,
	FANET_AIRCRAFT_TYPE_BALLOON,
	FANET_AIRCRAFT_TYPE_BALLOON,
	FANET_AIRCRAFT_TYPE_UAV,
	FANET_AIRCRAFT_TYPE_OTHER,
	FANET_AIRCRAFT_TYPE_OTHER,
	FANET_AIRCRAFT_TYPE_OTHER    // added for "winch"
};

const uint8_t aircraft_type_from_fanet[] PROGMEM = {
	AIRCRAFT_TYPE_UNKNOWN,
	AIRCRAFT_TYPE_PARAGLIDER,
	AIRCRAFT_TYPE_HANGGLIDER,
	AIRCRAFT_TYPE_BALLOON,
	AIRCRAFT_TYPE_GLIDER,
	AIRCRAFT_TYPE_POWERED,
	AIRCRAFT_TYPE_HELICOPTER,
	AIRCRAFT_TYPE_UAV
};


/* ------------------------------------------------------------------------- */
/*
 *
 *  Created on: 30 Sep 2016
 *      Author: sid
 */
static void coord2payload_absolut(float lat, float lon, uint8_t *buf)
{
	if (buf == NULL)
		return;

	int32_t lat_i = roundf(lat * 93206.0f);
	int32_t lon_i = roundf(lon * 46603.0f);

	buf[0] = ((uint8_t*)&lat_i)[0];
	buf[1] = ((uint8_t*)&lat_i)[1];
	buf[2] = ((uint8_t*)&lat_i)[2];

	buf[3] = ((uint8_t*)&lon_i)[0];
	buf[4] = ((uint8_t*)&lon_i)[1];
	buf[5] = ((uint8_t*)&lon_i)[2];
}
/* ------------------------------------------------------------------------- */
/*
 *  Created on: 06 Dec 2017
 *      Author: Linar Yusupov
 */
static void payload_absolut2coord(float *lat, float *lon, uint8_t *buf)
{
  int32_t lat_i = 0;
  int32_t lon_i = 0;

  if (buf == NULL || lat == NULL || lon == NULL)
    return;

  ((uint8_t*)&lat_i)[0] = buf[0];
  ((uint8_t*)&lat_i)[1] = buf[1];
  ((uint8_t*)&lat_i)[2] = buf[2];
  ((uint8_t*)&lat_i)[3] = buf[2] & 0x80 ? 0xFF : 0x00;

  ((uint8_t*)&lon_i)[0] = buf[3];
  ((uint8_t*)&lon_i)[1] = buf[4];
  ((uint8_t*)&lon_i)[2] = buf[5];
  ((uint8_t*)&lon_i)[3] = buf[5] & 0x80 ? 0xFF : 0x00;

  *lat = (float) lat_i / 93206.0f;
  *lon = (float) lon_i / 46603.0f;
}

/* ------------------------------------------------------------------------- */

// The current scheme only queues one ack and one normal message.
// It is possible for a new message to overwrite the one already queued.
// But given that FANET transmissions are only once in 5 seconds or more,
// a situation where many messages queue up is not viable anyway.

/* Pending FANET ACK TX queue - transmitted in next available TX slot */
static uint8_t  fnf_queued_ack_vendor = 0;
static uint16_t fnf_queued_ack_id     = 0;

// Pending FANET TX queue
// pre-built frame queued here and transmitted in next available TX slot
uint8_t  fn_tx_pending_buf[MAX_PKT_SIZE];
uint8_t  fn_tx_pending_len = 0;

static uint32_t fanet_name_last_ms = 0;
uint32_t fanet_sos_last_ms  = 0;
uint8_t  fanet_sos_count    = 0;

#define FANET_SOS_INTERVAL_MS  30000  /* broadcast SOS message every 30 seconds */
#define FANET_SOS_MAX_COUNT    3      /* send SOS message this many times, then stop */

bool fanet_decode(void *fanet_pkt, container_t *this_aircraft, ufo_t *fop) {

  fanet_packet_t *pkt = (fanet_packet_t *) fanet_pkt;
  uint8_t *raw = (uint8_t *) fanet_pkt;
  unsigned int altitude;
  uint8_t speed_byte, climb_byte;
  int speed_int, climb_int;
#if defined(FANET_NEXT)
  uint8_t offset_byte;
  int offset_int;
#endif
  size_t payload_offset = FANET_HEADER_SIZE;   /* 4 bytes basic header */
  size_t payload_len = 0;
  bool unicast = false;
  bool ack_requested = false;

  uint32_t sender_addr = (pkt->vendor << 16) | pkt->address;

  /* ignore this device own (relayed) packets */
  //if (pkt->vendor  == SOFRF_FANET_VENDOR_ID &&
  //    pkt->address == (this_aircraft->addr & 0xFFFF) /* && */
  //    /* pkt->forward == 1 */) {
  //  return rval;
  //}
  if (sender_addr == this_aircraft->addr)
      return false;

// since FANET transmits once in 5s, this is not useful:
//  for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {
//      if (excess[i].addr == sender_addr)                 // known as far and not tracked
//          return false;
//  }

  bool has_ext = (pkt->ext_header != 0);

  // shared data fields of both air and ground tracking
  if (pkt->type == 1 || pkt->type == 7) {
    if (has_ext)
        return false;
    fop->addr = sender_addr;
    fop->protocol = RF_PROTOCOL_FANET;
    fop->addr_type = ADDR_TYPE_FLARM;    // i.e., device - was ADDR_TYPE_FANET
    fop->timestamp = this_aircraft->timestamp;
    fop->gnsstime_ms = millis();
    payload_absolut2coord(&(fop->latitude), &(fop->longitude),
      ((uint8_t *) pkt) + FANET_HEADER_SIZE);
  }

  bool rval = false;

  if (pkt->type == 1) {  /* Tracking  */

    altitude = ((pkt->altitude_msb << 8) | pkt->altitude_lsb);
    if (pkt->altitude_scale) {
      altitude = altitude * 4 /* -2 */;
    }
    fop->altitude = (float) altitude;

    fop->aircraft_type = AT_FROM_FANET(pkt->aircraft_type);

    fop->airborne = 1;     // implied by packet type 1

    fop->course = (float) pkt->heading * 360.0 / 256.0;

    speed_byte = pkt->speed;
    speed_int = (int) (speed_byte | (speed_byte & (1<<6) ? 0xFFFFFF80U : 0));

    if (pkt->speed_scale) {
      speed_int *= 5 /* -2 */;
    }
    fop->speed = ((float) speed_int) / (2 * _GPS_KMPH_PER_KNOT);

    climb_byte = pkt->climb;
    climb_int = (int) (climb_byte | (climb_byte & (1<<6) ? 0xFFFFFF80U : 0));

    if (pkt->climb_scale) {
      climb_int *= 5 /* +-2 */;
    }
    fop->vs = ((float)climb_int) * (_GPS_FEET_PER_METER * 6.0);

#if defined(FANET_NEXT)
    offset_byte = pkt->qne_offset;
    offset_int = (int) (offset_byte | (offset_byte & (1<<6) ? 0xFFFFFF80U : 0));

    if (pkt->qne_scale) {
      offset_int *= 4;
    }

    fop->pressure_altitude = fop->altitude + (float) offset_int;
#endif

    fop->stealth = 0;
    fop->no_track = !(pkt->track_online);
    if (settings->debug_flags & DEBUG_RELAY)  fop->no_track = 0;   // for testing

#if 0
    Serial.print(fop->addr, HEX);
    Serial.print(',');
    Serial.print(fop->aircraft_type, HEX);
    Serial.print(',');
    Serial.print(fop->latitude, 6);
    Serial.print(',');
    Serial.print(fop->longitude, 6);
    Serial.print(',');
    Serial.print(fop->altitude);
    Serial.print(',');
    Serial.print(fop->speed);
    Serial.print(',');
    Serial.print(fop->course);
    Serial.println();
    Serial.flush();
#endif
    if (settings->debug_flags & DEBUG_DEEPER) {
        snprintf_P(NMEABuffer, sizeof(NMEABuffer),
          PSTR("$PSRFF,%06X,%.6f,%.6f,%.1f,%.1f,%.1f,%.1f,%d\r\n"),
          fop->addr, fop->latitude, fop->longitude, fop->altitude,
          fop->course, fop->speed, fop->vs, fop->no_track);
      NMEAOutD();
    }

    rval = true;

  } else if (pkt->type == 7) {  /* Ground Tracking */

    uint8_t status = ((uint8_t *) fanet_pkt)[FANET_HEADER_SIZE + 6];

    fop->airborne = 0;                                  // implied by packet type 7
    fop->altitude = this_aircraft->altitude;            // use own alt as estimate
    fop->aircraft_type = ((status >> 4) & 0x0F) + 32;   // no overlap with aircraft type codes
    fop->course = 0;
    fop->speed = 0;
    fop->vs = 0;
    fop->stealth = 0;
    fop->no_track = !(status & 0x01);
    if (settings->debug_flags & DEBUG_RELAY)  fop->no_track = 0;

    rval = true;

  } else if (pkt->type == 0 || pkt->type == 2 || pkt->type == 3 || pkt->type == 4) {
    /* ACK, pilot name, text message or weather */

    //uint8_t vendor = raw[1];
    //uint16_t address = raw[2] | ((uint16_t)raw[3] << 8);

    /* Calculate actual payload offset - skip extended header if present */
    //bool has_ext = raw[0] & 0x80;
    if (RF_last_rx_len > payload_offset)
        payload_len = RF_last_rx_len - payload_offset;
    if (has_ext && payload_len > 0) {
        ack_requested = raw[4] & (1 << 6);
        unicast =  (RF_last_rx_len >= 8) && (raw[4] & (1 << 5));
        payload_offset += 1;                      /* +1 ext header byte */
        payload_len -= 1;
        if (unicast && payload_len >= 3) {
            payload_offset += 3;                  /* +3 dest addr bytes */
            payload_len -= 3;
        }
    }

    /* Drop unicast messages not addressed to us - only broadcast (dst 000000)
     * or unicast-to-our-address frames should be forwarded to the app. */
    uint32_t dest_addr;
    if (unicast) {
        uint32_t dest_mfr = raw[5];
        uint32_t dest_id = raw[6] | ((uint16_t)raw[7] << 8);
        dest_addr = (dest_mfr << 16) | dest_id;
        if (dest_addr != ThisAircraft.addr) {
            Serial.print("FANET unicast message Type ");
            Serial.print(pkt->type);
            Serial.print(" to ");
            Serial.print(dest_mfr, HEX);
            Serial.print(",");
            Serial.print(dest_id, HEX);
            Serial.println(" (not us, ignored)");
            return false;
        }
    }

    if (pkt->type == 0) {  /* ACK */
#if defined(INCLUDE_FNF)
        if (FNF_dest != DEST_NONE) {
            /* ACK must be unicast to us */
            if (unicast) {
                // type 0 is handled by FN_check_ack(), do not forward to app
                // this will generate a #FNR ACK to the app if matched
                FN_check_ack(pkt->vendor, pkt->address, 0);
            } else {
                Serial.println("Type 0 but not unicast - ignoring");
            }
        }
#else
        Serial.print("FANET Type 0 ACK from ");
        Serial.println(sender_addr, HEX);
#endif
        return false;    // do not forward Type 0 ACK to app as #FNF
    }

    Serial.print("FANET message RX Type ");
    Serial.print(pkt->type);
    Serial.print(" from ");
    Serial.print(sender_addr, HEX);
    Serial.print(" len=");
    Serial.print(payload_len);
    if (pkt->type == 3) {
        Serial.print(unicast ? " unicast" : " broadcast");
        //if (unicast && RF_last_rx_len >= 8) {
        //    Serial.print(" to ");
        //    Serial.print(dest_addr, HEX);
        //}
        Serial.print("\r\nFANET message payload: \"");
        Serial.print(filter_printable(&raw[payload_offset], payload_len, (char *)NULL));
        Serial.print("\"");
    }
    Serial.println();

    /* rval = false; */
    /* packet is not a traffic position report */

  } else {

    return false;  // pkt->type not supported

  }

#if defined(INCLUDE_FNF)
  /* Send raw FANET frame as #FNF to XCGuide (if connected via BLE with high MTU).
   * Use RF_last_rx_len (actual LoRa received length) not rx_size (fixed struct size),
   * since FANET types 2/3 are variable-length. */
  // this is now done here, after unicast filtering, rather than in ParseData()
  if (FNF_dest != DEST_NONE) {
      // Some apps send a type 3 message instead of (or as well as) the Type 0 ACK once
      // the message is displayed/read - so call FN_check_ack() on all unicast messages:
      // this will NOT generate a #FNR ACK to the app, but will clear the waiting for an ack
      if (unicast)
          FN_check_ack(pkt->vendor, pkt->address, pkt->type);
      NMEA_FNF_Out((const uint8_t *)fanet_pkt, RF_last_rx_len);
  }
#endif

  //if (pkt->type == 2 || pkt->type == 3 || pkt->type == 4)
  if (rval == false) {   // not a position message
    // for callsign and for PFLAM limit text length
    if (payload_len > CALLSIGN_LEN-1)
        payload_len = CALLSIGN_LEN-1;
    raw[payload_offset+payload_len] = '\0';
      // - OK since  MAX_PKT_SIZE > CALLSIGN_LEN, and NMEA_FNF_Out() already called
    container_t *cip = NULL;
    // find this aircraft in the tracking table
    for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {
        if (Container[i].addr == sender_addr) {
            cip = &Container[i];
            break;
        }
    }
    if (pkt->type == 2 && (! has_ext)) {
      if (cip != NULL && (cip->callsign[0]=='\0' || cip->callsign[CALLSIGN_LEN-1]=='?')) {
        /* copy FANET name (plus nullchar) into callsign */
        //strncpy((char *)cip->callsign, (char *)&raw[payload_offset], payload_len+1);
        //cip->callsign[payload_len] = '\0';
        strcpy((char *)cip->callsign, (char *)&raw[payload_offset]);
        cip->callsign[CALLSIGN_LEN-1] = '\0';
      }
    }
    uint8_t pflam_type = PFLAM_BCST;
    if (pkt->type == 2)  pflam_type = PFLAM_PNAME;
    else if (unicast)    pflam_type = PFLAM_UCST;
    if (cip != NULL)
        NMEA_PFLAM(pflam_type, cip, &raw[payload_offset]);  // limited to length 17 or 13
  }

  /* For unicast message with ack requested, send Type 0 ACK back to the sender
   * so they know the message was delivered to this device.
   * When the pilot presses OK in XCGuide, the app sends a separate
   * "read ACK" (#FNT) which we transmit later as a second acknowledgement. */
  if (unicast && ack_requested) {
      Serial.print("FANET: staging delivery ACK to ");
      Serial.print(pkt->vendor, HEX);
      Serial.print(",");
      Serial.println(pkt->address, HEX);
      // Queue for transmission - only need to store the destination,
      // will build the packet later before transmission.
      fnf_queued_ack_vendor = pkt->vendor;
      fnf_queued_ack_id     = pkt->address;
  }

  return rval;
}

/* ------------------------------------------------------------------------- */

// Additional FANET transmission packet types
// Code courtesy of Vlad Belayev

/*
 * Encode a FANET Type 0 (ACK) packet.
 * ACK is unicast to the specified address, no payload.
 * Building this packet here, just before actual transmission,
 *    avoids overwriting fn_tx_pending_buf[]
 */
static size_t fanet_type0_encode(void *fanet_pkt)
{
    uint8_t *frame = (uint8_t *) fanet_pkt;

    uint32_t id = ThisAircraft.addr;

    /* Byte 0: type=0, forward=0, ext_header=1 */
    frame[0] = 0x80;
    /* Source address */
    frame[1] = (id >> 16) & 0xFF;
    frame[2] = id & 0xFF;
    frame[3] = (id >> 8) & 0xFF;
    /* Extended header: no ACK, unicast */
    frame[4] = (1 << 5);  /* unicast bit */
    /* Destination address */
    frame[5] = fnf_queued_ack_vendor;
    frame[6] = fnf_queued_ack_id & 0xFF;
    frame[7] = (fnf_queued_ack_id >> 8) & 0xFF;

    fnf_queued_ack_vendor = 0;
    fnf_queued_ack_id     = 0;

    return 8;
}

/*
 * Encode a FANET Type 2 (NAME) packet.
 * Broadcast, variable length.
 */
static size_t fanet_type2_encode(void *fanet_pkt, container_t *this_aircraft) {

  const char *name = settings->callsign;
  if (name[0] == '\0' || name[0] == ' ')
      return 0;

  uint8_t *buf = (uint8_t *) fanet_pkt;
  uint32_t id = this_aircraft->addr;

  /* header (4 bytes) */
  buf[0] = (2 & 0x3F) | (1 << 6) | (0 << 7);  /* type=2, forward=1, ext_header=0 */
  buf[1] = (id >> 16) & 0xFF;                    /* vendor */
  buf[2] = id & 0xFF;                            /* address low */
  buf[3] = (id >> 8) & 0xFF;                     /* address high */

  /* name string */
  size_t len = strlen(name);
  if (len > MAX_PKT_SIZE - FANET_HEADER_SIZE)
    len = MAX_PKT_SIZE - FANET_HEADER_SIZE;
  memcpy(&buf[FANET_HEADER_SIZE], name, len);

  return FANET_HEADER_SIZE + len;    // variable, depends on strlen(settings->callsign)
}

/*
 * Encode a FANET Type 3 (MSG) packet.
 * Broadcast, variable length.
 */
static size_t fanet_type3_encode(void *fanet_pkt, container_t *this_aircraft,
                                  const char *msg) {

  uint8_t *buf = (uint8_t *) fanet_pkt;
  uint32_t id = this_aircraft->addr;

  /* header (4 bytes) - broadcast, no extended header */
  buf[0] = (3 & 0x3F) | (1 << 6) | (0 << 7);  /* type=3, forward=1, ext_header=0 */
  buf[1] = (id >> 16) & 0xFF;
  buf[2] = id & 0xFF;
  buf[3] = (id >> 8) & 0xFF;

  /* body: subtype(1) + message string */
  buf[FANET_HEADER_SIZE] = 0x00;  /* subtype 0 = normal message */

  size_t len = strlen(msg);
  size_t max_msg = MAX_PKT_SIZE - FANET_HEADER_SIZE - 1;  /* -1 for subtype byte */
  if (len > max_msg) len = max_msg;
  memcpy(&buf[FANET_HEADER_SIZE + 1], msg, len);

  return FANET_HEADER_SIZE + 1 + len;
}

/*
 * Encode a FANET Type 7 (ground tracking) packet.
 */
static size_t fanet_type7_encode(void *fanet_pkt, container_t *this_aircraft) {

  if (ground_status <= GROUND_STATUS_COUNTDOWN)
      return 0;

  uint8_t *buf = (uint8_t *) fanet_pkt;
  uint32_t id = this_aircraft->addr;

  /* header (4 bytes) - same layout as Type 1 */
  buf[0] = (7 & 0x3F) | (1 << 6) | (0 << 7);  /* type=7, forward=1, ext_header=0 */
  buf[1] = (id >> 16) & 0xFF;                    /* vendor */
  buf[2] = id & 0xFF;                            /* address low */
  buf[3] = (id >> 8) & 0xFF;                     /* address high */

  /* body: lat(3) + lon(3) + status(1) */
  coord2payload_absolut(this_aircraft->latitude, this_aircraft->longitude,
    &buf[FANET_HEADER_SIZE]);

  uint8_t online = (this_aircraft->no_track ? 0 : 1);

  buf[FANET_HEADER_SIZE + 6] = (ground_status << 4) | online;

  return FANET_HEADER_SIZE + FANET_GROUND_BODY_SIZE;
}

/*
 * Encode a FANET Type 1 (air tracking) packet.
 */
static size_t fanet_type1_encode(void *fanet_pkt, container_t *this_aircraft) {

  uint32_t id = this_aircraft->addr;
  float lat = this_aircraft->latitude;
  float lon = this_aircraft->longitude;
  int16_t alt = (int16_t) this_aircraft->altitude;
  unsigned int aircraft_type =  this_aircraft->aircraft_type;
  float speed = this_aircraft->speed * _GPS_KMPH_PER_KNOT;
  float climb = this_aircraft->vs / (_GPS_FEET_PER_METER * 60.0);
  float heading = this_aircraft->course;
  float turnrate = 0;
  int16_t alt_diff = this_aircraft->pressure_altitude == 0 ? 0 :
          (int16_t) (this_aircraft->pressure_altitude - this_aircraft->altitude);

  fanet_packet_t *pkt = (fanet_packet_t *) fanet_pkt;

  pkt->ext_header     = 0;
  pkt->forward        = 1;
  pkt->type           = 1;  /* Tracking  */

  //pkt->vendor         = SOFRF_FANET_VENDOR_ID;
  pkt->vendor         = (id >> 16) & 0x00FF;   // will not be a constant
  pkt->address        = id & 0x00FFFF;

  coord2payload_absolut(lat, lon, ((uint8_t *) pkt) + FANET_HEADER_SIZE);

  pkt->track_online   = (this_aircraft->no_track ? 0 : 1);
  pkt->aircraft_type  = AT_TO_FANET(aircraft_type);

  int altitude = constrain(alt, 0, 8190);
  pkt->altitude_scale = altitude > 2047 ? (altitude = (altitude + 2) / 4, 1) : 0;
  pkt->altitude_msb   = (altitude & 0x700) >> 8;
  pkt->altitude_lsb   = (altitude & 0x0FF);

  int speed2          = constrain((int)roundf(speed * 2.0f), 0, 635);
  if(speed2 > 127) {
    pkt->speed_scale  = 1;
    pkt->speed        = ((speed2 + 2) / 5);
  } else {
    pkt->speed_scale  = 0;
    pkt->speed        = speed2 & 0x7F;
  }

  int climb10         = this_aircraft->stealth ?
                        0 : constrain((int)roundf(climb * 10.0f), -315, 315);
  if(climb10 > 63) {
    pkt->climb_scale  = 1;
    pkt->climb        = ((climb10 + (climb10 >= 0 ? 2 : -2)) / 5);
  } else {
    pkt->climb_scale  = 0;
    pkt->climb        = climb10 & 0x7F;
  }

  pkt->heading        = constrain((int)roundf(heading * 256.0f)/360.0f, 0, 255);

  int turnr4          = constrain((int)roundf(turnrate * 4.0f), -255, 255);
  if(abs(turnr4) > 63) {
    pkt->turn_scale   = 1;
    pkt->turn_rate    = ((turnr4 + (turnr4 >= 0 ? 2 : -2)) / 4);
  } else {
    pkt->turn_scale   = 0;
    pkt->turn_rate    = turnr4 & 0x7F;
  }

#if defined(FANET_NEXT)
  int16_t offset      = constrain(alt_diff, -254, 254);
  if(abs(offset) > 63) {
    pkt->qne_scale    = 1;
    pkt->qne_offset   = ((offset + (offset >= 0 ? 2 : -2)) / 4);
  } else {
    pkt->qne_scale    = 0;
    pkt->qne_offset   = offset & 0x7F;
  }
#endif

  return sizeof(fanet_packet_t);
}

/*
 * Encode a FANET packet: air or ground tracking, name, or SOS as appropriate
 */
size_t fanet_encode(void *fanet_pkt, container_t *this_aircraft) {

  if (fnf_queued_ack_id != 0) {
      // Encode and transmit a FANET ACK frame if needed
      // - uses and clears fnf_queued_ack_vendor, fnf_queued_ack_id
      Serial.println("FN_TX: transmitting queued ACK to ");
      Serial.print(fnf_queued_ack_vendor, HEX);
      Serial.print(",");
      Serial.println(fnf_queued_ack_id, HEX);
      return (fanet_type0_encode(fanet_pkt));
  } else if (fn_tx_pending_len > 0) {
      // Transmit pending FANET frame if any, instead of encoding a new one.
      size_t len = fn_tx_pending_len;
      memcpy(fanet_pkt, fn_tx_pending_buf, len);
      fn_tx_pending_len = 0;   /* clear before transmit so re-entry is safe */
      Serial.print("FN_TX: transmitting queued FANET frame, len=");
      Serial.println(len);
      return len;
  }

  uint32_t now = millis();

  /* Distress mode: alternate SOS message and DISTRESS tracking */
  // this can be triggered by double-click or automatically 3 min after landing
  if (ground_status >= GROUND_STATUS_NEED_MED) {
    if (fanet_sos_count < FANET_SOS_MAX_COUNT &&
        (fanet_sos_last_ms == 0 ||
         (now - fanet_sos_last_ms) >= FANET_SOS_INTERVAL_MS)) {
      fanet_sos_last_ms = now;
      fanet_sos_count++;
      char sos_msg[60];
      snprintf(sos_msg, sizeof(sos_msg), "SOS %s %.4f,%.4f,%0f",
          (ground_status == GROUND_STATUS_NEED_MED? "NEED MED" : "DISTRESS"),
          this_aircraft->latitude, this_aircraft->longitude, this_aircraft->altitude);
      return fanet_type3_encode(fanet_pkt, this_aircraft, sos_msg);
    }
    // SOS message every 30 sec, ground tracking every few seconds:
    // (even after FANET_SOS_MAX_COUNT, keep sending type7 with distress status)
    return fanet_type7_encode(fanet_pkt, this_aircraft);
  }

  /* Every 2 minutes, send a Name packet (Type 2) instead of tracking */
  if ((now - fanet_name_last_ms) >= FANET_NAME_INTERVAL_MS) {
    fanet_name_last_ms = now;
    size_t s = fanet_type2_encode(fanet_pkt, this_aircraft);
    if (s > 0) return s;
  }

  // If distress mode then it is caught above.
  // If AUTO_DISTRESS then wait until landing situation is clear before sending ground status
  if (ground_status > GROUND_STATUS_COUNTDOWN /* || ground_status == GROUND_STATUS_INITIAL */ )
      return fanet_type7_encode(fanet_pkt, this_aircraft);

  /* Otherwise send normal Air Tracking (Type 1) */
  // >>> what if not airborne, still in ground_status=initial, should send type-7? Or send nothing?
  return fanet_type1_encode(fanet_pkt, this_aircraft);
}

