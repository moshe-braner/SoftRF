/*
 * GDL90Helper.cpp
 * Copyright (C) 2016-2021 Linar Yusupov
 *
 * Inspired by Eric's Dey Python GDL-90 encoder:
 * https://github.com/etdey/gdl90
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

#include <TimeLib.h>
#include <lib_crc.h>
#include <protocol.h>

#include "../../system/SoC.h"
#include "../../system/Time.h"
#include "GDL90.h"
#include "GNS5892.h"
#include "../../driver/RF.h"
#include "../../driver/Baro.h"
#include "../../driver/GNSS.h"
#include "../../driver/Settings.h"
#include "../../driver/WiFi.h"
#include "../../driver/Bluetooth.h"
#include "../../TrafficHelper.h"
#include "../radio/Legacy.h"
#include "NMEA.h"

#if defined(ENABLE_AHRS)
#include "../../AHRS.h"
#endif /* ENABLE_AHRS */

#define ADDR_TO_HEX_STR(s, c) (s += ((c) < 0x10 ? "0" : "") + String((c), HEX))

static GDL90_Msg_HeartBeat_t HeartBeat;
static GDL90_Msg_Traffic_t Traffic;
static GDL90_Msg_OwnershipGeometricAltitude_t GeometricAltitude;

const char *GDL90_CallSign_Prefix[] = {
  [RF_PROTOCOL_LEGACY]    = "FL",
  [RF_PROTOCOL_OGNTP]     = "OG",
  [RF_PROTOCOL_P3I]       = "PA",
  [RF_PROTOCOL_ADSB_1090] = "AD",
  [RF_PROTOCOL_ADSB_UAT]  = "UA",
  [RF_PROTOCOL_FANET]     = "FA",
  [RF_PROTOCOL_GDL90]     = "GD",    // data from external device
  [RF_PROTOCOL_LATEST]    = "FL"
};

const uint8_t aircraft_type_to_gdl90[] PROGMEM = {
	GDL90_EMITTER_CATEGORY_NONE,
	GDL90_EMITTER_CATEGORY_GLIDER,
	GDL90_EMITTER_CATEGORY_LIGHT,
	GDL90_EMITTER_CATEGORY_ROTORCRAFT,
	GDL90_EMITTER_CATEGORY_SKYDIVER,
	GDL90_EMITTER_CATEGORY_LIGHT,
	GDL90_EMITTER_CATEGORY_ULTRALIGHT,
	GDL90_EMITTER_CATEGORY_ULTRALIGHT,
	GDL90_EMITTER_CATEGORY_LIGHT,
	GDL90_EMITTER_CATEGORY_SMALL,
	GDL90_EMITTER_CATEGORY_MANEUVERABLE,
	GDL90_EMITTER_CATEGORY_BALLOON,
	GDL90_EMITTER_CATEGORY_BALLOON,
	GDL90_EMITTER_CATEGORY_UAV,
	GDL90_EMITTER_CATEGORY_UNASSIGNED1,
	GDL90_EMITTER_CATEGORY_NONE
};

const uint8_t gdl90_to_aircraft_type[] PROGMEM = {
	AIRCRAFT_TYPE_UNKNOWN,
	AIRCRAFT_TYPE_POWERED,
	AIRCRAFT_TYPE_POWERED,
	AIRCRAFT_TYPE_JET,
	AIRCRAFT_TYPE_JET,
	AIRCRAFT_TYPE_JET,
	AIRCRAFT_TYPE_POWERED,
	AIRCRAFT_TYPE_HELICOPTER,
	AIRCRAFT_TYPE_RESERVED,
	AIRCRAFT_TYPE_GLIDER,
	AIRCRAFT_TYPE_BALLOON,
	AIRCRAFT_TYPE_PARACHUTE,
	AIRCRAFT_TYPE_HANGGLIDER,
	AIRCRAFT_TYPE_RESERVED,
	AIRCRAFT_TYPE_UAV,
	AIRCRAFT_TYPE_RESERVED
};

#if defined(DO_GDL90_FF_EXT)
/*
 * See https://www.foreflight.com/connect/spec/ for details
 */
const GDL90_Msg_FF_ID_t msgFFid = {
  .Sub_Id       = 0, /* 0 for ID, 1 for AHRS */
  .Version      = 1, /* Must be 1 */
  /* Device serial number is 0xFFFFFFFFFFFFFFFF for invalid */
  .SerialNum    = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
  .ShortName    = {'S', 'o', 'f', 't', 'R', 'F', ' ', ' ' },
  .LongName     = {'S', 'o', 'f', 't', 'R', 'F',
                    ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ' },
#if !defined(USE_GDL90_MSL)
  .Capabilities = {0x00, 0x00, 0x00, 0x00}, /* WGS-84 ellipsoid altitude for Ownship Geometric report */
#else
  .Capabilities = {0x00, 0x00, 0x00, 0x01}, /* MSL altitude for Ownship Geometric report */
#endif /* USE_GDL90_MSL */
};
#endif /* DO_GDL90_FF_EXT */

/* convert a signed latitude to 2s complement ready for 24-bit packing */
static uint32_t makeLatitude(float latitude)
{
    int32_t int_lat;

    if (latitude > 90.0) {
      latitude = 90.0;
    }

    if (latitude < -90.0) {
      latitude = -90.0;
    }

    int_lat = (int) (latitude * (0x800000 / 180.0));

    if (int_lat < 0) {
      int_lat = (0x1000000 + int_lat) & 0xffffff;  /* 2s complement */  
    }

    return(int_lat);    
}

/* convert a signed longitude to 2s complement ready for 24-bit packing */
static uint32_t makeLongitude(float longitude)
{
    int32_t int_lon;

    if (longitude > 180.0) {
      longitude = 180.0;
    }

    if (longitude < -180.0) {
      longitude = -180.0;
    }

    int_lon = (int) (longitude * (0x800000 / 180.0));

    if (int_lon < 0) {
      int_lon = (0x1000000 + int_lon) & 0xffffff;  /* 2s complement */  
    }

    return(int_lon);    
}


static uint32_t pack24bit(uint32_t num)
{
  return( ((num & 0xff0000) >> 16) | (num & 0x00ff00) | ((num & 0xff) << 16) );
}

uint16_t GDL90_calcFCS(uint8_t msg_id, uint8_t *msg, int size)
{
  uint16_t crc16 = 0x0000;  /* seed value */

  crc16 = update_crc_gdl90(crc16, msg_id);   // in firmware\source\libraries\CRC\

  for (int i=0; i < size; i++)
  {
    crc16 = update_crc_gdl90(crc16, msg[i]);
  }    

  return(crc16);
}

uint8_t *GDL90_EscapeFilter(uint8_t *buf, uint8_t *p, int size)
{
  while (size--) {
    if (*p != 0x7D && *p != 0x7E) {
      *buf++ = *p++;
    } else {
      *buf++ = 0x7D;
      *buf++ = *p++ ^ 0x20;
    }   
  }

  return (buf);
}

static void *msgHeartbeat()
{
  time_t ts = elapsedSecsToday(OurTime);

  /* Status Byte 1 */
  HeartBeat.gnss_pos_valid  = isValidFix() ;
  HeartBeat.maint_reqd      = 0;
  HeartBeat.ident           = 0;
  HeartBeat.addr_type       = 0;
  HeartBeat.gnss_bat_low    = 0;
  HeartBeat.ratcs           = 0;
//HeartBeat.reserved1       = 0;
  HeartBeat.uat_init        = 1;

  /* Status Byte 2 */
  HeartBeat.time_stamp_ms   = (ts >> 16) & 1;
  HeartBeat.csa_req         = 0;
  HeartBeat.csa_not_avail   = 0;
//HeartBeat.reserved2       = 0;
//HeartBeat.reserved3       = 0;
//HeartBeat.reserved4       = 0;
//HeartBeat.reserved5       = 0;
  HeartBeat.utc_ok          = 0;

  HeartBeat.time_stamp      = (ts & 0xFFFF);   // LSB first
  HeartBeat.message_counts  = 0;

  return (&HeartBeat);
}

static void *msgType10and20(container_t *aircraft)
{
  int altitude;

  /*
   * The Altitude field "ddd" contains the pressure altitude
   * (referenced to 29.92 inches Hg), encoded using 25-foot resolution,
   * offset by 1,000 feet.
   * The 0xFFF value represents that the pressure altitude is invalid.
   * The minimum altitude that can be represented is -1,000 feet.
   * The maximum valid altitude is +101,350 feet.
   */

  /* If the aircraft's data has standard pressure altitude - make use it */
  if (aircraft->pressure_altitude != 0.0) {
    altitude = (int)(aircraft->pressure_altitude * _GPS_FEET_PER_METER);
  } else if (ThisAircraft.pressure_altitude != 0.0) {
    /* If this SoftRF unit is equiped with baro sensor - try to make an adjustment */
    float altDiff = ThisAircraft.pressure_altitude - ThisAircraft.altitude;
    altitude = (int)((aircraft->altitude + altDiff) * _GPS_FEET_PER_METER);
  } else {
    /* If there are no any choice - report GNSS AMSL altitude as pressure altitude */
    altitude = (int)((aircraft->altitude - ThisAircraft.geoid_separation) * _GPS_FEET_PER_METER);
  }
  altitude = (altitude + 1000) / 25; /* Resolution = 25 feet */

  int trackHeading = (int)(aircraft->course / (360.0 / 256)); /* convert to 1.4 deg single byte */

  if (altitude < 0) {
    altitude = 0;  
  }
  if (altitude > 0xffe) {
    altitude = 0xffe;  
  }
 
  uint8_t misc = 9;
  //altitude = 0x678;
  
  uint16_t horiz_vel = (uint16_t) aircraft->speed /* 0x123 */ ; /*  in knots */
  uint16_t vert_vel = (uint16_t) ((int16_t) (aircraft->vs / 64.0)) /* 0x456 */; /* in units of 64 fpm */

  Traffic.alert_status  = 0 /* 0x1 */;
  Traffic.addr_type     = 0 /* 0x2 */;
  Traffic.addr          = pack24bit(aircraft->addr) /* pack24bit(0x345678) */;
  Traffic.latitude      = pack24bit(makeLatitude(aircraft->latitude)) /* pack24bit(0x9abcde) */;
  Traffic.longitude     = pack24bit(makeLongitude(aircraft->longitude)) /* pack24bit(0xf12345) */;

  /*
   * workaround against "implementation dependant"
   * XTENSA's GCC bitmap layout in structures
   */
  Traffic.altitude      = ((altitude >> 4) & 0xFF) | (misc << 8);
  Traffic.misc          = (altitude & 0x00F);

  Traffic.nic           = 8 /* 0xa */;
  Traffic.nacp          = 8 /* 0xb */;

  /*
   * workaround against "implementation dependant"
   * XTENSA's GCC bitmap layout in structures
   */

  Traffic.horiz_vel = (((vert_vel >> 8) & 0xF) << 8)| (((horiz_vel >> 8) & 0xF) << 4) | ((horiz_vel >> 4) & 0xF) ;
  Traffic.vert_vel =  (((vert_vel >> 4) & 0xF) << 8) | ((vert_vel & 0xF) << 4) | (horiz_vel & 0xF) ;

  Traffic.track         = (trackHeading & 0xFF) /* 0x03 */;
  Traffic.emit_cat      = AT_TO_GDL90(aircraft->aircraft_type) /* 0x4 */;

  /*
   * When callsign is available - send it to a GDL90 client.
   * If it is not - generate a callsign substitute,
   * based upon a protocol ID and the ICAO address
   */
  if (aircraft->callsign[0] == '\0') {
    memcpy(aircraft->callsign, GDL90_CallSign_Prefix[aircraft->protocol],
             strlen(GDL90_CallSign_Prefix[aircraft->protocol]));
    String str = "";
    ADDR_TO_HEX_STR(str, (aircraft->addr >> 16) & 0xFF);
    ADDR_TO_HEX_STR(str, (aircraft->addr >>  8) & 0xFF);
    ADDR_TO_HEX_STR(str, (aircraft->addr      ) & 0xFF);
    str.toUpperCase();
    memcpy(aircraft->callsign + strlen(GDL90_CallSign_Prefix[aircraft->protocol]),
            str.c_str(), str.length());
    /* this callsign stays with aircraft until it expires */
  }

  memcpy(Traffic.callsign, aircraft->callsign, sizeof(Traffic.callsign));

  Traffic.emerg_code    = 0 /* 0x5 */;
//Traffic.reserved      = 0;

  return (&Traffic);
}

static void *msgOwnershipGeometricAltitude(container_t *aircraft)
{
  uint16_t vfom = 0x000A;

#if !defined(USE_GDL90_MSL)
  /*
   * The Geo Altitude field is a 16-bit signed integer that represents
   * the geometric altitude (height above WGS-84 ellipsoid),
   * encoded using 5-foot resolution
   */
  uint16_t altitude = (int16_t)(aircraft->altitude *    /*  was  + aircraft->geoid_separation */
                        (_GPS_FEET_PER_METER / 5));
#else
  /*
   * Vast majority of EFBs deviates from Rev A of GDL90 ICD (2007) specs
   * and uses MSL altitude here.
   * SkyDemon is the only known exception which uses WGS-84 altitude still.
   */
  uint16_t altitude = (int16_t)((aircraft->altitude - ThisAircraft.geoid_separation) * _GPS_FEET_PER_METER / 5);
#endif /* USE_GDL90_MSL */

  GeometricAltitude.geo_altitude  = ((altitude & 0x00FF) << 8) | ((altitude & 0xFF00) >> 8) ;
  GeometricAltitude.VFOM          = ((vfom & 0x00FF) << 8) | ((vfom & 0xFF00) >> 8);
  GeometricAltitude.vert_warning  = 0;

  return (&GeometricAltitude);
}

static size_t makeHeartbeat(uint8_t *buf)
{
  uint8_t *ptr = buf;
  uint8_t *msg = (uint8_t *) msgHeartbeat();
  uint16_t fcs = GDL90_calcFCS(GDL90_HEARTBEAT_MSG_ID, msg,
                               sizeof(GDL90_Msg_HeartBeat_t));
  uint8_t fcs_lsb, fcs_msb;
  
  fcs_lsb = fcs        & 0xFF;
  fcs_msb = (fcs >> 8) & 0xFF;

  *ptr++ = 0x7E; /* Start flag */
  *ptr++ = GDL90_HEARTBEAT_MSG_ID;
  ptr = GDL90_EscapeFilter(ptr, msg, sizeof(GDL90_Msg_HeartBeat_t));
  ptr = GDL90_EscapeFilter(ptr, &fcs_lsb, 1);
  ptr = GDL90_EscapeFilter(ptr, &fcs_msb, 1);
  *ptr++ = 0x7E; /* Stop flag */

  return(ptr-buf);
}

static size_t makeType10and20(uint8_t *buf, uint8_t id, container_t *aircraft)
{
// >>>  generate output for testing - report ownship as traffic
  if (settings->debug_flags & DEBUG_SIMULATE)
      id = GDL90_TRAFFIC_MSG_ID;

  uint8_t *ptr = buf;
  uint8_t *msg = (uint8_t *) msgType10and20(aircraft);
  uint16_t fcs = GDL90_calcFCS(id, msg, sizeof(GDL90_Msg_Traffic_t));
  uint8_t fcs_lsb, fcs_msb;
  
  fcs_lsb = fcs        & 0xFF;
  fcs_msb = (fcs >> 8) & 0xFF;

  *ptr++ = 0x7E; /* Start flag */
  *ptr++ = id;
  ptr = GDL90_EscapeFilter(ptr, msg, sizeof(GDL90_Msg_Traffic_t));
  ptr = GDL90_EscapeFilter(ptr, &fcs_lsb, 1);
  ptr = GDL90_EscapeFilter(ptr, &fcs_msb, 1);
  *ptr++ = 0x7E; /* Stop flag */

  return(ptr-buf);
}

static size_t makeGeometricAltitude(uint8_t *buf, container_t *aircraft)
{
  uint8_t *ptr = buf;
  uint8_t *msg = (uint8_t *) msgOwnershipGeometricAltitude(aircraft);
  uint16_t fcs = GDL90_calcFCS(GDL90_OWNGEOMALT_MSG_ID, msg,
                               sizeof(GDL90_Msg_OwnershipGeometricAltitude_t));
  uint8_t fcs_lsb, fcs_msb;

  fcs_lsb = fcs        & 0xFF;
  fcs_msb = (fcs >> 8) & 0xFF;

  *ptr++ = 0x7E; /* Start flag */
  *ptr++ = GDL90_OWNGEOMALT_MSG_ID;
  ptr = GDL90_EscapeFilter(ptr, msg, sizeof(GDL90_Msg_OwnershipGeometricAltitude_t));
  ptr = GDL90_EscapeFilter(ptr, &fcs_lsb, 1);
  ptr = GDL90_EscapeFilter(ptr, &fcs_msb, 1);
  *ptr++ = 0x7E; /* Stop flag */

  return(ptr-buf);
}

#if defined(DO_GDL90_FF_EXT)

static size_t makeFFid(uint8_t *buf)
{
  uint8_t *ptr = buf;
  uint8_t *msg = (uint8_t *) &msgFFid;
  uint16_t fcs = GDL90_calcFCS(GDL90_FFEXT_MSG_ID, msg, sizeof(GDL90_Msg_FF_ID_t));
  uint8_t fcs_lsb, fcs_msb;

  fcs_lsb = fcs        & 0xFF;
  fcs_msb = (fcs >> 8) & 0xFF;

  *ptr++ = 0x7E; /* Start flag */
  *ptr++ = GDL90_FFEXT_MSG_ID;
  ptr = GDL90_EscapeFilter(ptr, msg, sizeof(GDL90_Msg_FF_ID_t));
  ptr = GDL90_EscapeFilter(ptr, &fcs_lsb, 1);
  ptr = GDL90_EscapeFilter(ptr, &fcs_msb, 1);
  *ptr++ = 0x7E; /* Stop flag */

  return(ptr-buf);
}
#endif

#define makeOwnershipReport(b,a)  makeType10and20(b, GDL90_OWNSHIP_MSG_ID, a)
#define makeTrafficReport(b,a)    makeType10and20(b, GDL90_TRAFFIC_MSG_ID, a)

static void GDL90_Out(byte *buf, size_t size)
{
  if (size > 0) {
    switch(settings->gdl90)
    {
    case DEST_UART:
      if (SoC->UART_ops) {
        SoC->UART_ops->write(buf, size);
      } else {
        Serial.write(buf, size);
      }
      break;
    case DEST_UART2:
#if defined(ESP32)
      if (has_serial2)
        Serial2.write(buf, size);
#endif
      break;
#if !defined(EXCLUDE_WIFI)
    case DEST_UDP:
      {
        SoC->WiFi_transmit_UDP(GDL90_DST_PORT, buf, size);
      }
      break;
    case DEST_TCP:
#if defined(NMEA_TCP_SERVICE)
      WiFi_transmit_TCP((char*)buf, size);
#endif
      break;
#endif
#if defined(ARDUINO_ARCH_NRF52)
    case DEST_USB:
      {
        if (SoC->USB_ops) {
          SoC->USB_ops->write(buf, size);
        }
      }
      break;
#endif
    case DEST_BLUETOOTH:
      {
        if (BTactive && SoC->Bluetooth_ops) {
          SoC->Bluetooth_ops->write(buf, size);
        }
      }
      break;
    case DEST_NONE:
    default:
      break;
    }
  }
}

void GDL90_Export()
{
  size_t size;
  float distance;
  time_t this_moment = OurTime;
//  uint8_t *buf = (uint8_t *) (sizeof(UDPpacketBuffer) < UDP_PACKET_BUFSIZE ?
//                              NMEABuffer : UDPpacketBuffer);
  uint8_t *buf = (uint8_t *) NMEABuffer;

  if (settings->gdl90 != DEST_NONE) {
    size = makeHeartbeat(buf);
    GDL90_Out(buf, size);

#if defined(DO_GDL90_FF_EXT)
    size = makeFFid(buf);
    GDL90_Out(buf, size);
#endif /* DO_GDL90_FF_EXT */

#if defined(ENABLE_AHRS)
    size = AHRS_GDL90(buf);
    GDL90_Out(buf, size);
#endif /* ENABLE_AHRS */

    if (isValidFix()) {
      size = makeOwnershipReport(buf, &ThisAircraft);
      GDL90_Out(buf, size);

      size = makeGeometricAltitude(buf, &ThisAircraft);
      GDL90_Out(buf, size);

      for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {

        // do not echo GDL90 traffic data back to its source
        if (Container[i].addr
         && (Container[i].protocol != RF_PROTOCOL_GDL90 || settings->gdl90 != settings->gdl90_in)
         && (this_moment - Container[i].timestamp) <= EXPORT_EXPIRATION_TIME) {

          distance = Container[i].distance;

          if (distance < ALARM_ZONE_NONE) {
            size = makeTrafficReport(buf, &Container[i]);
            GDL90_Out(buf, size);
          }
        }
      }
    }
  }
}

// ********************************************************

// Code for processing GDL90 input - by Moshe Braner, Feb 2024

// decode the GDL90 traffic message and pass it to TrafficHelper.cpp
//   - but first discard traffic that is too far or too high
void process_traffic_message(char* buf)
{
  static ufo_t fo;
  GDL90_Msg_Traffic_t *tp = (GDL90_Msg_Traffic_t *) buf;

  // this is our private "fo" for GDL90 input only, same fields are overwritten each time
  fo.addr_type = ((tp->addr_type==0 || tp->addr_type==2)? ADDR_TYPE_ICAO : ADDR_TYPE_FLARM);
  uint32_t addr = pack24bit(tp->addr);
  if (addr == ThisAircraft.addr)        // somehow echoed back
      return;
  if (addr == settings->ignore_id)      // ID told in settings to ignore
      return;
  fo.addr = addr;
  uint32_t ulatlon = pack24bit(tp->latitude);
  if (ulatlon & 0x800000)  ulatlon |= 0xFF000000;
  int32_t ilatlon = (int32_t) ulatlon;
  fo.latitude = ((float) ilatlon) * (180.0 / 0x800000);
  if (fabs(fo.latitude - ThisAircraft.latitude) > 0.25)  // 15 nm
      return;
  ulatlon = pack24bit(tp->longitude);
  if (ulatlon & 0x800000)  ulatlon |= 0xFF000000;
  ilatlon = (int32_t) ulatlon;
  fo.longitude = ((float) ilatlon) * (180.0 / 0x800000);
  if (fabs(fo.longitude - ThisAircraft.longitude) > 0.25 * InvCosLat())  // 15 nm
      return;
  // tp->misc is really the LSNibble of alt
  // the real misc is in bits 8-11 of tp->altitude
  //uint32_t ialt = (((tp->altitude & 0xFF) << 4) | tp->misc);
  //uint8_t misc = ((tp->altitude & 0xF00) >> 8);
  // another way to handle this mess is to reference byte positions in the buf
  uint32_t ialt = (((uint32_t)((uint8_t)buf[10])) << 4) | ((((uint8_t)buf[11]) & 0xF0) >> 4);
  uint8_t misc = (((uint8_t)buf[11]) & 0x0F);
  fo.altitude = ((float) (25*ialt - 1000)) * (1.0 / _GPS_FEET_PER_METER);
  // this is pressure altitude, try and correct
  if (baro_chip != NULL)
      fo.altitude += ThisAircraft.baro_alt_diff;
  else
      fo.altitude += average_baro_alt_diff;
  if (fabs(fo.altitude - ThisAircraft.altitude) > 2000)  // meters
      return;
  fo.airborne = ((misc & 0x08) != 0);
  // similar mess:
  uint16_t horiz_vel = (((uint32_t)((uint8_t)buf[13])) << 4) | ((((uint8_t)buf[14]) & 0xF0) >> 4);
  fo.speed = (float) horiz_vel;       // knots
  //uint16_t vert_vel  = ((((uint8_t)buf[14]) & 0x0F) << 8) | ((uint8_t)buf[15]);
  if ((((uint8_t)buf[14]) & 0x0F) != 0) {
      fo.vs = 0;  // not available
  } else {
      fo.vs = (float) (((uint32_t)((uint8_t)buf[15])) << 6);    // was in units of 64 fpm
  }
  //fo.course = ((float)tp->track) * (360.0 / 256.0);
  if (misc & 0x03)
      fo.course = (float) ((((uint32_t)tp->track) * 360) >> 8);
  else
      fo.course = 0;  // not available
  fo.aircraft_type = GDL90_TO_AT(tp->emit_cat);

#if 0
Serial.printf("GDL90>%x %s, %f, %f, %.0f\r\n",
  fo.addr, fo.callsign, fo.latitude,  fo.longitude,  fo.altitude );
  /* , fo.speed, fo.vs, fo.course */
#endif

  fo.tx_type = TX_TYPE_ADSB;        // may not be correct, but have to assume
  fo.protocol = RF_PROTOCOL_GDL90;  // not an RF protocol, but that is the data source
  fo.timestamp = ThisAircraft.timestamp;
  fo.gnsstime_ms = millis();
  fo.addr_type = ADDR_TYPE_ICAO;
  fo.airborne = 1;
  fo.circling = 0;
#if defined(ESP32)
  ++adsb_packets_counter;    // in GNS5892
#endif
  RF_last_rssi = 0;
  AddTraffic(&fo, (char *) tp->callsign);
}

// Accummulate bytes in traffic data message - ignore all others
#define WAIT_FOR_FLAG 128
#define GOT_FLAG      129
void GDL90_bridge_buf(char c, char* buf, int& n)
{
    if (n == WAIT_FOR_FLAG) {
        if (c == 0x7E)           // wait for a start flag
            n = GOT_FLAG;
    } else if (n == GOT_FLAG) {
        if (c == GDL90_TRAFFIC_MSG_ID)
            n = 0;               // ready to receive message bytes
        else if (c != 0x7E)
            n = WAIT_FOR_FLAG;   // ignore non-traffic messages
        // else two flags in a row, leave state = GOT_FLAG
    } else if (n > 31) {
        if (n > (30+32)) {
            n = WAIT_FOR_FLAG;   // guard against buffer overrun
        } else {
            c ^= 0x20;           // finish escape sequence
            n -= 32;
            buf[n++] = c;
        }
    } else if (c == 0x7D) {
        n += 32;                 // start escape sequence
    } else if (c == 0x7E) {      // Start or stop flag
        if (n == 29) {           // length of a valid traffic message + checksum
            uint16_t fcs = GDL90_calcFCS(GDL90_TRAFFIC_MSG_ID, (uint8_t*)buf, 27);
            uint8_t fcs_lsb = fcs        & 0xFF;
            uint8_t fcs_msb = (fcs >> 8) & 0xFF;
            if (buf[28]==fcs_msb && buf[27]==fcs_lsb) {  // valid checksum
                process_traffic_message(buf);
            } else {
                Serial.println(F("GDL90 msg rcvd has invalid checksum"));
            }
            NMEA_bridge_sent = true;   // not really sent, but substantial processing
        } else {
            Serial.println(F("GDL90 msg rcvd has wrong length"));
        }
        n = WAIT_FOR_FLAG;
    } else {
       buf[n++] = c;
    }
}
