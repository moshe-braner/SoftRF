/*
 * TrafficHelper.cpp
 * Copyright (C) 2018-2021 Linar Yusupov
 *
 * Alarm_Latest() code by Moshe Braner 2022-2024
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

#include <math.h>

#include "../SoftRF.h"
#include "system/SoC.h"
#include "system/Time.h"
#include "TrafficHelper.h"
#include "driver/Settings.h"
#include "driver/RF.h"
#include "driver/GNSS.h"
#include "driver/Buzzer.h"
#include "driver/Strobe.h"
#include "driver/Filesys.h"
#include "ui/Web.h"
#include "protocol/radio/Legacy.h"
#include "protocol/data/NMEA.h"
#include "protocol/data/IGC.h"
#include "Wind.h"

#if !defined(EXCLUDE_VOICE)
#if defined(ESP32)
#include "driver/Voice.h"
#endif
#endif

#if defined(FILESYS)
//#include "SPIFFS.h"
File AlarmLog;
bool AlarmLogOpen = false;
#endif

void startlogs()
{
#if defined(FILESYS)
    // restart alarm log on first takeoff after boot
    if (settings->logalarms && AlarmLogOpen==false && FS_is_mounted) {
        const char *filename = "/alarmlog.txt";
        bool append = false;
#if defined(ESP32)
        if (FILESYS.exists(filename)) {
            if (FILESYS_free_kb() > 15)
                append = true;
            else
                FILESYS.remove(filename);
        }
        AlarmLog = FILESYS.open(filename, (append? FILE_APPEND : FILE_WRITE));
#else
        // the nRF52 FatFS does not have totalBytes() and usedBytes() nor FILE_APPEND
        if (FILESYS.exists(filename)) {
            if (FILESYS_free_kb() > 50)
                append = true;
            else
                FILESYS.remove(filename);
        }
        AlarmLog = FILESYS.open(filename, (append? (O_WRITE | O_APPEND) : (O_WRITE | O_CREAT)));
#endif
        if (AlarmLog) {
            AlarmLogOpen = true;
            if (append == false) {
              const char *p = "date,time,lat,lon,level,count,ID,relbrg,hdist,vdist\r\n";
              AlarmLog.write((const uint8_t *)p, strlen(p));
            }
        } else {
            Serial.println(F("Failed to open alarmlog.txt"));
        }
    }
#endif   // FILESYS
#if defined(USE_SD_CARD)
    // also start this flight's SDlog with a banner:
    int year   = gnss.date.year();
    if( year > 99)  year = year - 2000;
    int month  = gnss.date.month();
    int day    = gnss.date.day();
    int hour   = gnss.time.hour();
    int minute = gnss.time.minute();
    snprintf(NMEABuffer, sizeof(NMEABuffer),
        "%02d/%02d/%02d %02d:%02d takeoff\r\n", year, month, day, hour, minute);
    Serial.println(NMEABuffer);
    SD_log(NMEABuffer);
#endif
    // and, if flight-logging, start now:
    if (settings->logflight == FLIGHT_LOG_AIRBORNE
     || settings->logflight == FLIGHT_LOG_TRAFFIC) {
        openFlightLog();
    }
}

// close the alarm log and flight log after landing
void stoplogs()
{
#if defined(FILESYS)
    AlarmLog.close();
    AlarmLogOpen = false;
#endif
//#if defined(USE_SD_CARD)
    if (settings->logflight != FLIGHT_LOG_ALWAYS)
          closeFlightLog();
//#endif
}

unsigned long UpdateTrafficTimeMarker = 0;

container_t Container[MAX_TRACKING_OBJECTS]; // EmptyContainer;   // more fields
ufo_t fo; // EmptyFO;                                             // fewer fields

void EmptyContainer(container_t *p) { memset(p, 0, sizeof(CONTAINER)); }
void EmptyFO(ufo_t *p) { memset(p, 0, sizeof(UFO)); }

char fo_callsign[10];
uint8_t fo_raw[34];
traffic_by_dist_t traffic_by_dist[MAX_TRACKING_OBJECTS];
int max_alarm_level = ALARM_LEVEL_NONE;
int8_t maxrssi;
uint8_t adsb_acfts;
bool alarm_ahead = false;                    /* global, used for visual displays */
bool relay_waiting = false;
bool alt_relay_waiting = false;

float average_baro_alt_diff = 0;

static int8_t (*Alarm_Level)(container_t *, container_t *);

static uint32_t Alarm_timer = 0;

// Compute registration-number from ICAO ID - USA and Canada only

static void icao_canadian(container_t *fop)
{
    uint32_t icao = fop->addr;
    char *buf = (char *) fop->callsign;
    buf[0] = 'C';
    buf[1] = '-';
    icao -= 0xC00001;
    uint32_t dig, rem;
    dig = icao/(26*26*26);
    rem = icao - dig*(26*26*26);
    if (dig==2) dig=3;   // H is skipped, use I
    buf[2] = dig+'F';
    dig = rem/(26*26);
    rem = rem - dig*(26*26);
    buf[3] = dig+'A';
    dig = rem/26;
    rem = rem - dig*26;
    buf[4] = dig+'A';
    buf[5] = rem+'A';
    buf[6] = '?';
    buf[7] = '\0';
    buf[9] = '?';
}

// For USA based on:   https://github.com/guillaumemichel/icao-nnumber_converter

static const char nnumberchars[]  = "ABCDEFGHJKLMNPQRSTUVWXYZ0123456789";
#define LETTERSET_SIZE 25     // 24 letters (alphabet without I and O) + 1
#define suffix_size    601
#define bucket4_size   35
#define bucket3_size   951
#define bucket2_size   10111
#define bucket1_size   101711

static void get_suffix(uint32_t offset, char *buf)
{
    if (offset != 0) {
        uint32_t i0 = (offset-1)/LETTERSET_SIZE;
        *buf++ = nnumberchars[i0];
        uint32_t rem = (offset-1) - i0*LETTERSET_SIZE;
        if (rem != 0)
            *buf++ = nnumberchars[rem-1];
    }
    *buf++ = '?';
    *buf = '\0';
    return;
}

void icao_to_n(container_t *fop)
{
    if (fop->addr_type != ADDR_TYPE_ICAO)
        return;
    if (settings->band != RF_BAND_US)   // this is only for USA & Canada aircraft
        return;
    char *buf = (char *) fop->callsign;
    if (buf[0] != '\0' && buf[0] != ' ')        // already have a callsign
        return;
    uint32_t icao = fop->addr;
    if (icao > 0xC00000 && icao < 0xC0CDF9) {   // a valid Canadian ICAO ID
        icao_canadian(fop);
        return;
    }
    if (icao < 0xA00001 || icao > 0xADF7C7)     // not a valid US ICAO ID
        return;
    buf[9] = '?';    // past the trailing null char, marks as computed, not received
    icao -= 0xA00001;
    buf[0] = 'N';
    //buf[1] = '\0';
    char *output = &buf[1];
    uint32_t dig, rem;
    dig = icao/bucket1_size;        // digit 1 minus 1
    rem = icao - dig*bucket1_size;
    *output++ = dig+'1';
    if (rem < suffix_size)
        get_suffix(rem, output);
    else {
    rem -= suffix_size;             // shift for digit 2
    dig = rem/bucket2_size;
    rem = rem - dig*bucket2_size;
    *output++ = dig+'0';
    if (rem < suffix_size)
        get_suffix(rem, output);
    else {
    rem -= suffix_size;             // shift for digit 3
    dig = rem/bucket3_size;
    rem = rem - dig*bucket3_size;
    *output++ = dig+'0';
    if (rem < suffix_size)
        get_suffix(rem, output);
    else {
    rem -= suffix_size;             // shift for digit 4
    dig = rem/bucket4_size;
    rem = rem - dig*bucket4_size;
    *output++ = dig+'0';
    if (rem)
        *output++ = nnumberchars[rem-1];   // find last character
    *output++ = '?';       // this becomes part of the "callsign" string
    *output = '\0';
    }}}
}

/*
 * No any alarms issued by the firmware.
 * Rely upon high-level flight management software.
 */
static int8_t Alarm_None(container_t *this_aircraft, container_t *fop)
{
  return ALARM_LEVEL_NONE;
}

/*
 * Adjust relative altitude for relative vertical speed.
 */
float Adj_alt_diff(container_t *this_aircraft, container_t *fop)
{
  float alt_diff = fop->alt_diff;           /* positive means fop is higher than this_aircraft */
  float vsr = fop->vs - this_aircraft->vs;  /* positive means fop is rising relative to this_aircraft */
  if (vsr >  2000)  vsr =  2000;            /* ignore implausible data (units are fpm) */
  if (vsr < -2000)  vsr = -2000;
  float alt_change = vsr * 0.05;  /* expected change in 10 seconds, converted to meters */

  /* only adjust towards higher alarm level: */
  if (alt_diff > 0 && alt_change < 0) {
    alt_diff += alt_change;   /* makes alt_diff smaller */
    if (alt_diff < 0)  return 0;  /* minimum abs_alt_diff */
  } else if (alt_diff < 0 && alt_change > 0) {
    alt_diff += alt_change;   /* makes alt_diff less negative */
    if (alt_diff > 0)  return 0;  /* minimum abs_alt_diff */
  }

  /* GPS altitude is fuzzy so ignore the first VERTICAL_SLACK (30m) difference */
  if (alt_diff > VERTICAL_SLACK)
    return (alt_diff - VERTICAL_SLACK);
  if (alt_diff < -VERTICAL_SLACK)
    return (alt_diff + VERTICAL_SLACK);
  return 0;
}

/*
 * Simple, distance based alarm level assignment.
 */
static int8_t Alarm_Distance(container_t *this_aircraft, container_t *fop)
{
  int8_t rval = ALARM_LEVEL_NONE;

  float distance = fop->distance;
  if (distance > ALARM_ZONE_CLOSE
      || fabs(fop->adj_alt_diff) > VERTICAL_SEPARATION) {
    return ALARM_LEVEL_NONE;
  }

  float adj_distance;
  if (fop->adj_distance > distance)
         adj_distance = fop->adj_distance;
  else
         adj_distance = distance;

  if (adj_distance < ALARM_ZONE_EXTREME && fop->alert_level > ALARM_LEVEL_NONE)
    --fop->alert_level;     /* may sound new alarm for same URGENT level */
  if (adj_distance < ALARM_ZONE_URGENT) {
    rval = ALARM_LEVEL_URGENT;
  } else if (adj_distance < ALARM_ZONE_IMPORTANT) {
    rval = ALARM_LEVEL_IMPORTANT;
  } else if (adj_distance < ALARM_ZONE_LOW) {
    rval = ALARM_LEVEL_LOW;
  } else if (adj_distance < ALARM_ZONE_CLOSE) {
    rval = ALARM_LEVEL_CLOSE;
  }

  return rval;
}

/*
 * EXPERIMENTAL
 *
 * Linear, CoG and GS based collision prediction.
 */
static int8_t Alarm_Vector(container_t *this_aircraft, container_t *fop)
{
  if (fop->tx_type <= TX_TYPE_S)
    return Alarm_Distance(this_aircraft, fop);    // non-directional target

  if (fop->speed == 0)
    return Alarm_Distance(this_aircraft, fop);    // ADS-B target with no velocity message yet

  int8_t rval = ALARM_LEVEL_NONE;

  if (fop->gnsstime_ms - fop->prevtime_ms > 3000)   /* also catches prevtime_ms == 0 */
    return Alarm_Distance(this_aircraft, fop);

  float distance = fop->distance;
  if (distance > 2*ALARM_ZONE_CLOSE) {    // 3km
    return ALARM_LEVEL_NONE;
    /* save CPU cycles */
  }

  float abs_alt_diff = fabs(fop->adj_alt_diff);
  if (abs_alt_diff > VERTICAL_SEPARATION) {
    return ALARM_LEVEL_NONE;
    /* save CPU cycles */
  }

  if (distance > (fop->speed + this_aircraft->speed) * (ALARM_TIME_LOW * _GPS_MPS_PER_KNOT)) {
    return ALARM_LEVEL_NONE;
    /* save CPU cycles */
  }

  /* if either aircraft is turning, vector method is not usable */
  if (fabs(this_aircraft->turnrate) > 3.0 || fabs(fop->turnrate) > 3.0)
        return Alarm_Distance(this_aircraft, fop);

  float V_rel_magnitude, V_rel_direction, t;

  if (abs_alt_diff < VERTICAL_SEPARATION) {  /* no alarms if too high or too low */

    float adj_distance = fop->adj_distance;
    if (adj_distance < distance)
        adj_distance = distance;

    /* Subtract 2D velocity vector of traffic from 2D velocity vector of this aircraft */
    float this_course = D2R * this_aircraft->course;
    float that_course = D2R * fop->course;
    float V_rel_y = this_aircraft->speed * cos(this_course) -
                    fop->speed * cos(that_course);                      /* N-S */
    float V_rel_x = this_aircraft->speed * sin(this_course) -
                    fop->speed * sin(that_course);                      /* E-W */

    V_rel_magnitude = hypot(V_rel_x, V_rel_y) * _GPS_MPS_PER_KNOT;
    V_rel_direction = R2D * atan2(V_rel_x, V_rel_y);     /* direction fop is coming from */
    if (V_rel_direction < 0.0)
        V_rel_direction += 360.0;

    /* +- some degrees tolerance for collision course */
    /* also check the relative speed, ALARM_VECTOR_SPEED = 2 m/s */
    /* also adj_distance takes altitude difference into account */

    if (V_rel_magnitude > ALARM_VECTOR_SPEED) {

      /* time in seconds prior to impact */
      t = adj_distance / V_rel_magnitude;

      float rel_angle = fabs(V_rel_direction - fop->bearing);
      if (rel_angle > 180.0)  rel_angle = 360.0 - rel_angle;    // handle wraparound at 360

      if (rel_angle < ALARM_VECTOR_ANGLE && V_rel_magnitude > (3 * ALARM_VECTOR_SPEED)) {

        /* time limit values are compliant with FLARM data port specs */
        if (t < ALARM_TIME_CLOSE) {
          rval = ALARM_LEVEL_CLOSE;
          if (t < ALARM_TIME_LOW) {
            rval = ALARM_LEVEL_LOW;
            if (t < ALARM_TIME_IMPORTANT) {
              rval = ALARM_LEVEL_IMPORTANT;
              if (t < ALARM_TIME_URGENT)
                rval = ALARM_LEVEL_URGENT;
            }
          }
        }

      } else if (rel_angle < 2 * ALARM_VECTOR_ANGLE) {

        /* reduce alarm level since direction is less direct and/or relative speed is low */
        if (t < ALARM_TIME_LOW) {
          rval = ALARM_LEVEL_CLOSE;
          if (t < ALARM_TIME_IMPORTANT) {
            rval = ALARM_LEVEL_LOW;
            if (t < ALARM_TIME_URGENT) {
              rval = ALARM_LEVEL_IMPORTANT;
              if (t < ALARM_TIME_EXTREME)
                rval = ALARM_LEVEL_URGENT;
            }
          }
        }

      } else if (rel_angle < 3 * ALARM_VECTOR_ANGLE) {

        /* further reduce alarm level for larger angles */
        if (t < ALARM_TIME_IMPORTANT) {
          rval = ALARM_LEVEL_CLOSE;
          if (t < ALARM_TIME_URGENT) {
            rval = ALARM_LEVEL_LOW;
            if (t < ALARM_TIME_EXTREME)
              rval = ALARM_LEVEL_IMPORTANT;
          }
        }
      }

    }
  }

  if (rval >= ALARM_LEVEL_LOW && t < ALARM_TIME_EXTREME && fop->alert_level > ALARM_LEVEL_NONE)
      --fop->alert_level;     /* may sound new alarm for same URGENT level */

  /* send data out via NMEA for debugging */
  if ((settings->nmea_d || settings->nmea2_d) && (settings->debug_flags & DEBUG_ALARM)) {
    snprintf_P(NMEABuffer, sizeof(NMEABuffer),
      PSTR("$PSALV,%06X,%ld,%d,%.1f,%.1f,%.1f,%.1f,%.5f,%.5f,%.1f,%.1f,%.1f,%.5f,%.5f,%.1f,%.1f,%.1f\r\n"),
      fop->addr, fop->gnsstime_ms, rval, V_rel_magnitude, V_rel_direction, fop->bearing, t,
      this_aircraft->latitude, this_aircraft->longitude, this_aircraft->altitude,
         this_aircraft->speed, this_aircraft->course,
      fop->latitude, fop->longitude, fop->altitude, fop->speed, fop->course);
    NMEAOutD();
  }

  return rval;
}


/*
 * VERY EXPERIMENTAL
 *
 * "Latest" method is based on short history of (future) 2D velocity vectors (NS/EW).
 * The approach here tried to use the velocity components from the time points
 * that FLARM sends out, in the way it probably intended by sending the data
 * out in that format, and given the weak computing power of early hardware
 * - but turns out that's not what FLARM does...  See Wind.cpp project_that().
 * The new 2024 protocol sends speed, direction, and turn rate explicitly instead.
 * Either way, this algorithm assumes that circling aircraft will keep circling
 * for the relevant time period (the next 19 seconds).
 */
static int8_t Alarm_Latest(container_t *this_aircraft, container_t *fop)
{
  if (fop->distance > 2*ALARM_ZONE_CLOSE) {    // 3km
    return ALARM_LEVEL_NONE;
    /* save CPU cycles */
  }

  if (fop->tx_type <= TX_TYPE_S)
    return Alarm_Distance(this_aircraft, fop);    // non-directional target

  if (fop->speed == 0)
    return Alarm_Distance(this_aircraft, fop);    // ADS-B target with no velocity message yet

  if (fop->tx_type == TX_TYPE_TISB || fop->relayed)
    return (Alarm_Vector(this_aircraft, fop));    // data not timely enough for this algo

  float v2 = fop->speed + this_aircraft->speed;
  if (fop->distance > v2 * (ALARM_TIME_LOW * _GPS_MPS_PER_KNOT)) {
    return ALARM_LEVEL_NONE;
    /* save CPU cycles */
  }

  int vv = (int)v2;
  int dz = ((vv * vv) >> 8);
  dz = (int)fabs(fop->adj_alt_diff) - dz;   // rough accounting for potential zoom-up
  if (dz > VERTICAL_SEPARATION) {
    return ALARM_LEVEL_NONE;
    /* save CPU cycles */
  }

  // if fop->protocol==RF_PROTOCOL_LATEST turnrate is already known from the received packet
  if (fop->protocol == RF_PROTOCOL_LATEST &&
         fabs(this_aircraft->turnrate) < 2.0 && fabs(fop->turnrate) < 2.0) {
    /* neither aircraft is turning */
    return (Alarm_Vector(this_aircraft, fop));
    // hopefully this takes care of aerotows?
    // >>> or try and use this algorithm anyway?
  }

  /* here start the expensive calculations */

  /* calculate other aircraft's turn rate and project course into future time points */
  project_that(fop);

  /* Also compute such estimates for ThisAircraft  - both relative to air not ground */
  /* project_this(this_aircraft); */
  /* - was already called from Estimate_Wind() or Legacy_Encode() */

  if (fop->protocol != RF_PROTOCOL_LATEST &&
         fabs(this_aircraft->turnrate) < 2.0 && fabs(fop->turnrate) < 2.0) {
    /* neither aircraft is turning */
    return (Alarm_Vector(this_aircraft, fop));
    // hopefully this takes care of aerotows?
    // >>> or try and use this algorithm anyway?
  }

  // flag if both aircraft are circling in the same direction
  bool gaggling = (abs(this_aircraft->circling + fop->circling) == 2);

  // flag if possibly a tow operation
  bool towing = (this_aircraft->aircraft_type==AIRCRAFT_TYPE_TOWPLANE && fop->aircraft_type==AIRCRAFT_TYPE_GLIDER)
             || (this_aircraft->aircraft_type==AIRCRAFT_TYPE_GLIDER && fop->aircraft_type==AIRCRAFT_TYPE_TOWPLANE);
  if (towing) {
    float course_diff = fabs(this_aircraft->course - fop->course);
    if (course_diff > 20.0 && course_diff < 340.0)            towing = false;
    if (fabs(this_aircraft->turnrate - fop->turnrate) > 6.0)  towing = false;   // deg/sec
    if (fabs(this_aircraft->speed - fop->speed) > 15.0)       towing = false;   // knots
  }
  // actually diverted typical towing (both non-turning) to vector method above

  /* Use integer math for computational speed */

  /* also take altitude difference and zoom-up into account */
  dz = (int) fop->alt_diff;          // meters   - not adj_alt_diff since we re-compute zoom-up here
  float vsr = fop->vs - this_aircraft->vs;  // fpm, >0 if fop is rising relative to this_aircraft
  int absdz = abs(dz);
  int adjdz = absdz;
  // assume lower aircraft may be zooming up
  // potential zoom altitude is about V^2/20 (m, m/s)
  int vx, vy, vv20;
  if (dz < 0 && !fop->circling && vsr > 400) {
    // other aircraft is lower and not circling and relatively rising by > 2 m/s
    vx = fop->air_ew[0];
    vy = fop->air_ns[0];              // airspeed in quarter-meters per second
    vv = (vx*vx + vy*vy);
  } else if (dz > 0 && !this_aircraft->circling && vsr < -400) {
    // this aircraft is lower and not circling and relatively rising by > 2 m/s
    vx = this_aircraft->air_ew[0];
    vy = this_aircraft->air_ns[0];
    vv = (vx*vx + vy*vy);
  } else {
    vv = 0;
  }
  bool zoom = false;
  int32_t factor;
  if (vv > (20*20*4*4)) {
    vv20 = vv - (20*20*4*4);     // can zoom until airspeed decreases to 20 m/s
    adjdz -= vv20 >> 9;          // about 2/3 of possible zoom
    if (vv20 > 8000) {
      zoom = true;
      /* if zooming to level of other aircraft, speed decreases           */
      /* rough approximation: multiply speed by (1 - 5*dz/vv)             */
      /*    5 = 20, halved for average over time, halved again for sqrt() */
      factor = (5*16*64) * (int32_t) absdz;
      factor = 64 - factor/vv;         // if zoom, vv cannot be zero
      if (factor < 48)  factor = 48;   // <<< maybe should lower this limit, to 32?
    }
  }
  adjdz -= VERTICAL_SLACK;       // for possible GPS altitude discrepancy, 30m, reduced from 70
  if (adjdz < 0)
    adjdz = 0;
  if (adjdz > 60)                // meters - cannot reach the 120m 3D distance threshold below
    return (ALARM_LEVEL_NONE);

  /* may want to adjust dz over time? */
  /* - but is relative vs representative of future? */
  /* - in same thermal average relative vs = 0 */

  /* Project relative position second by second into the future */
  /* Time points in our ns/ew array of airspeeds are at +3,6,9,12,15,18 sec */

  /* prepare second-by-second velocity vectors */
  static int thisvx[20];
  static int thisvy[20];
  static int thatvx[20];
  static int thatvy[20];
  //int vx, vy;
  int *px = thisvx;
  int *py = thisvy;
  /* considered interpolating between the 4 points, but does not seem useful */
  int i, j;
  if (zoom && dz > 15) {
      for (i=0; i<6; i++) {
         int32_t v;
         v = this_aircraft->air_ew[i];
         v *= factor;
         vx = v >> 6;                   // for the 64x scaling of factor
         v = this_aircraft->air_ns[i];
         v *= factor;
         vy = v >> 6;
         for (j=0; j<3; j++) {
           *px++ = vx;
           *py++ = vy;    
         }
      }
  } else {
    for (i=0; i<6; i++) {
      vx = this_aircraft->air_ew[i];  /* quarter-meters per second */
      vy = this_aircraft->air_ns[i];
      for (j=0; j<3; j++) {
        *px++ = vx;
        *py++ = vy;    
      }
    }
  }
  *px = vx;  /* extrapolate one more second */
  *py = vy;

  /* same for the other aircraft */
  px = thatvx;
  py = thatvy;
  if (zoom && dz < -15) {
    for (i=0; i<6; i++) {
       int32_t v;
       v = fop->air_ew[i];
       v *= factor;
       vx = v >> 6;
       v = fop->air_ns[i];
       v *= factor;
       vy = v >> 6;
       for (j=0; j<3; j++) {
         *px++ = vx;
         *py++ = vy;    
       }
    }
  } else {
    for (i=0; i<6; i++) {
      vx = fop->air_ew[i];
      vy = fop->air_ns[i];
      for (j=0; j<3; j++) {
        *px++ = vx;
        *py++ = vy;    
      }
    }
  }
  *px = vx;
  *py = vy;    

  /* 2D position of fop relative to this aircraft */
  /* - computed in Traffic_Update() */
  /* convert from meters to quarter-meters */
  int dx = fop->dx << 2;
  int dy = fop->dy << 2;

  /* project paths over time and find minimum 3D distance */
  int minsqdist = 200*200*4*4;
  int mintime = ALARM_TIME_CLOSE;
  int vxmin = 0;
  int vymin = 0;
  /* if projections are from different times, offset the arrays */
  if (fop->projtime_ms > this_aircraft->projtime_ms + 500) {
    /* this_aircraft projection is older, shift by 1 second */
    i = 0;
    j = 1;
    dx -= thisvx[0];  /* this aircraft movement during the first second */
    dy -= thisvy[0];
  } else if (this_aircraft->projtime_ms > fop->projtime_ms + 500) {
    /* other aircraft projection is older */
    i = 1;
    j = 0;
    dx += thatvx[0];  /* other aircraft movement during the first second */
    dy += thatvy[0];
  } else {
    i = 0;
    j = 0;
  }
  int t;
  adjdz <<= 3;
  // <<2 for units: convert to quarter-meters, and
  // another <<1 to consider vertical separation 2x better than horizontal distance
  int sqdz = adjdz * adjdz;
  //int cursqdist = dx*dx + dy*dy;        // previous version
  int cursqdist = dx*dx + dy*dy + sqdz;   // causes more alarms

  for (t=0; t<18; t++) {  /* loop over the 1-second time points prepared */
    vx = thatvx[i] - thisvx[j];   /* relative velocity */
    vy = thatvy[i] - thisvy[j];
    dx += vx;   /* change in relative position over this second */
    dy += vy;
    /* dz += vz; */
    int sqdist = dx*dx + dy*dy + sqdz;
    if (sqdist < minsqdist) {
      minsqdist = sqdist;
      vxmin = vx;
      vymin = vy;
      mintime = t;
    }
    ++i;
    ++j;
  }

  if (cursqdist <= minsqdist || mintime == 0) {
      // if not getting any closer than current situation
      //     then don't sound an alarm
      return ALARM_LEVEL_NONE;
  }

  int8_t rval = ALARM_LEVEL_NONE;

  /* try and set thresholds for alarms with gaggles - and tows - in mind */
  /* squeezed between size of thermal, length of tow rope, and accuracy of prediction */
  if (minsqdist < 40*40*4*4) {                /* 40 meters 3D separation */
      if (mintime < ALARM_TIME_URGENT) {
        rval = ALARM_LEVEL_URGENT;
      } else if (mintime < ALARM_TIME_IMPORTANT) {
        rval = ALARM_LEVEL_IMPORTANT;
      } else {  /* min-dist time is at most 18 seconds */
        rval = ALARM_LEVEL_LOW;
      }
  } else if (minsqdist < 70*70*4*4 && !gaggling && !towing ) {
      if (mintime < ALARM_TIME_EXTREME) {
        rval = ALARM_LEVEL_URGENT;
      } else if (mintime < ALARM_TIME_URGENT) {
        rval = ALARM_LEVEL_IMPORTANT;
      } else if (mintime < ALARM_TIME_IMPORTANT) {
        rval = ALARM_LEVEL_LOW;
      } else {
        rval = ALARM_LEVEL_CLOSE;
      }
  } else if (minsqdist < 120*120*4*4 && !gaggling && !towing ) {
      if (mintime < ALARM_TIME_EXTREME) {
        rval = ALARM_LEVEL_IMPORTANT;
      } else if (mintime < ALARM_TIME_URGENT) {
        rval = ALARM_LEVEL_LOW;
      } else if (mintime < ALARM_TIME_IMPORTANT) {
        rval = ALARM_LEVEL_CLOSE;
      }
  }
  // reduce alarm level if collision speed is low
  int sqspeed = 0;
  if (rval > ALARM_LEVEL_NONE) {
    sqspeed = vxmin*vxmin + vymin*vymin;    /* relative speed at closest point, squared */
    if (sqspeed < 6*6*4*4) {     /* relative speed < 6 mps */
      --rval;       // <= IMPORTANT
      if (sqspeed < 4*4*4*4) {   /* relative speed < 4 mps */
        --rval;     // <= LOW
        if (sqspeed < 2*2*4*4)   /* relative speed < 2 mps */
          --rval;   // < LOW
      }
    }
  }
  if (rval < ALARM_LEVEL_NONE)
      rval = ALARM_LEVEL_NONE;

  if (rval >= ALARM_LEVEL_LOW && mintime < ALARM_TIME_EXTREME && fop->alert_level > ALARM_LEVEL_NONE)
      --fop->alert_level;     /* may sound new alarm even for same URGENT level */

  /* send data out via NMEA for debugging */
  if (rval > ALARM_LEVEL_CLOSE || fop->distance < 300 || minsqdist < 120*120*4*4) {
    if ((settings->nmea_d || settings->nmea2_d) && (settings->debug_flags & DEBUG_ALARM)) {
      snprintf_P(NMEABuffer, sizeof(NMEABuffer),
        PSTR("$PSALL,%06X,%ld,%ld,%d,%d,%d,%d,%.1f,%.1f,%.1f,%ld,%ld,%.1f,%.1f,%.1f,%.1f\r\n"),
          fop->addr, fop->projtime_ms, this_aircraft->projtime_ms, rval, mintime, minsqdist, sqspeed,
          this_aircraft->speed, this_aircraft->heading, this_aircraft->turnrate,
          fop->dy, fop->dx, fop->alt_diff, fop->speed, fop->heading, fop->turnrate);
      NMEAOutD();
    }
  }

  return rval;
}

void logOneTraffic(container_t *fop, const char *label)
{
//#if defined(USE_SD_CARD)
    uint32_t addr = ((fop->no_track && fop->tx_type==TX_TYPE_FLARM)? 0xAAAAAA : fop->addr);
    int alarm_level = fop->alarm_level - 1;
    if (alarm_level < ALARM_LEVEL_NONE)
        alarm_level = ALARM_LEVEL_NONE;
    snprintf_P(NMEABuffer, sizeof(NMEABuffer),
      PSTR("%s,%d,%d,%d,%d,%06x,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n"),
      label, alarm_level, fop->tx_type, fop->protocol, fop->aircraft_type, addr,
      (int)fop->distance, (int)fop->bearing,
      (int)fop->speed, (int)fop->course, (int)fop->turnrate,
      (int)fop->RelativeHeading, (int)fop->alt_diff, (int)(fop->vs - ThisAircraft.vs),
      (int)ThisAircraft.speed, (int)ThisAircraft.course, (int)ThisAircraft.turnrate,
      (int)(wind_speed * (1.0 / _GPS_MPS_PER_KNOT)), (int)wind_direction);
    //Serial.print(NMEABuffer);
    NMEAOutD();
    FlightLogComment(NMEABuffer+4);   // it will prepend the LPLT
//#endif
}

// insert data about all "close" traffic into the flight log
// - called after the logFlightPosition() call
// - do this even on FATFS, despite space on T-Echo being small
// - can avoid by not setting logflight=traffic
// - alarms will still log if settings->logalarms
void logCloseTraffic()
{
//#if defined(USE_SD_CARD)
    container_t *fop;
    for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {
        fop = &Container[i];
        if (fop->addr == 0)
            continue;
        if (fop->airborne == 0)
            continue;
        if (fop->adj_distance > 1000  // meters, adjusted for altitude difference
                && fop->alarm_level == ALARM_LEVEL_NONE)
            continue;
        if (OurTime > fop->timestamp + 3 /*seconds*/ )
            continue;
        logOneTraffic(fop, "LPLTT");
    }
//#endif
}

/* cos(latitude) is used to convert longitude difference into linear distance. */
/* Once computed, accurate enough through a significant range of latitude. */

static float cos_lat = 0.7071;
//static float cos_lat_m = 111300.0 * 0.7071;
static float inv_cos_lat = 1.4142;

float CosLat()
{
  static float oldlat = 45.0;
  float latitude = ThisAircraft.latitude;
  if (fabs(latitude-oldlat) > 0.3) {
    cos_lat = cos(D2R*latitude);
    if (cos_lat > 0.01)
        inv_cos_lat = 1.0 / cos_lat;
    oldlat = latitude;
  }
  return cos_lat;
}

float InvCosLat() { return inv_cos_lat; }

struct {
    float distance;
    float bearing;
    float alt_diff;
    int32_t dx;
    int32_t dy;
} stash;

// compute within the container_t struct
void Calc_Traffic_Distances(container_t *cip)
{
    cip->alt_diff = cip->altitude - ThisAircraft.altitude;
    /* use an approximation for distance & bearing between 2 points */
    float y = 111300.0 * (cip->latitude - ThisAircraft.latitude);     // meters
    float x = 111300.0 * (cip->longitude - ThisAircraft.longitude) * CosLat(/*ThisAircraft.latitude*/);
    cip->dx = (int32_t) x;
    cip->dy = (int32_t) y;
    cip->distance = hypot(x, y);   /* meters  */
    cip->bearing = R2D * atan2(x, y);         /* degrees from ThisAircraft to fop */
    if (cip->bearing < 0)
        cip->bearing += 360;
}

// compute and stash in the stash struct
void Stash_Traffic_Distances(ufo_t *fop)
{
    stash.alt_diff = fop->altitude - ThisAircraft.altitude;
    /* use an approximation for distance & bearing between 2 points */
    float x, y;
    y = 111300.0 * (fop->latitude  - ThisAircraft.latitude);         /* meters */
    x = 111300.0 * (fop->longitude - ThisAircraft.longitude) * CosLat(/*ThisAircraft.latitude*/);
    stash.dx = (int32_t) x;
    stash.dy = (int32_t) y;
    stash.distance = hypot(x, y);                 /* meters  */
    stash.bearing = R2D * atan2(x, y);            /* degrees from ThisAircraft to fop */
    if (stash.bearing < 0)
        stash.bearing += 360;
}

// copy from the stash struct to a container_t struct
void Copy_Traffic_Distances(container_t *cip)
{
    cip->distance = stash.distance;
    cip->bearing  = stash.bearing;
    cip->alt_diff = stash.alt_diff;
    cip->dx       = stash.dx;
    cip->dy       = stash.dy;
}

// assume dx, dy, distance, bearing, alt_diff have already been computed
void Traffic_Update(container_t *fop)
{
  if (fop->tx_type <= TX_TYPE_S) {       // non-directional target

    fop->adj_alt_diff = fop->alt_diff;
    fop->adj_distance = fop->distance + VERTICAL_SLOPE * fabs(fop->alt_diff);
    fop->RelativeHeading = 0;
    if (fop->protocol == RF_PROTOCOL_ADSB_1090) {
        if (fop->maxrssi == 0 || fop->rssi > fop->maxrssi) {
            fop->maxrssi = fop->rssi;
            fop->maxrssirelalt = fop->alt_diff;
        }
    }
    if (ThisAircraft.airborne == 0) {
        fop->alarm_level = ALARM_LEVEL_NONE;
        return;
    }
    // else fall through to alarm level computation below

  } else {

    //int rel_bearing = (int) (fop->bearing - ThisAircraft.course);
    int rel_heading = (int) (fop->bearing - ThisAircraft.heading);
    rel_heading += (rel_heading < -180 ? 360 : (rel_heading > 180 ? -360 : 0));
    fop->RelativeHeading = rel_heading;

    if (fop->protocol == RF_PROTOCOL_ADSB_1090) {
        if (fop->mindist == 0 || fop->distance < fop->mindist) {
            fop->mindist = fop->distance;
            fop->mindistrssi = fop->rssi;
        }
        if (fop->maxrssi == 0 || fop->rssi > fop->maxrssi) {
            fop->maxrssi = fop->rssi;
            fop->maxrssirelalt = fop->alt_diff;
        }
    }

    /* take altitude (and vert speed) differences into account as adjusted distance */
    float adj_alt_diff = Adj_alt_diff(&ThisAircraft, fop);
    fop->adj_alt_diff = adj_alt_diff;
    fop->adj_distance = fop->distance + VERTICAL_SLOPE * fabs(adj_alt_diff);

    /* follow FLARM docs: do not issue alarms about non-airborne traffic */
    /* (first minute exception removed) (demo-mode exception added) */
    if ((fop->airborne == 0 || ThisAircraft.airborne == 0) && !do_alarm_demo
              /* && (millis() - SetupTimeMarker > 60000) */ ) {
      fop->alarm_level = ALARM_LEVEL_NONE;
      return;
    }

    // do not compute alarms unless data is current
    if (OurTime > ThisAircraft.timestamp + 2)
        return;
    if (OurTime > fop->timestamp + 2)
        return;
  }

  if (Alarm_Level) {  // if a collision prediction algorithm selected

      uint8_t old_alarm_level = fop->alarm_level;
      fop->alarm_level = (*Alarm_Level)(&ThisAircraft, fop);

      /* Sound an alarm if new alert, or got closer than previous alert,     */
      /* or (hysteresis) got two levels farther, and then closer.            */
      /* E.g., if alarm was for LOW, alert_level was set to LOW.             */
      /* A new alarm alert will sound if close enough to now be IMPORTANT.   */
      /* If gone to CLOSE, then back to LOW, still no new alarm.             */
      /* If now gone to NONE (farther than CLOSE), set alert_level to CLOSE, */
      /* then next time returns to alarm_level LOW will give an alert.       */

      if (fop->alarm_level < fop->alert_level)       /* if just less by 1...   */
          fop->alert_level = fop->alarm_level + 1;   /* ...then no change here */

      if (Alarm_timer != 0 && millis() > Alarm_timer) {
          if (fop->alert_level > ALARM_LEVEL_NONE)
              --fop->alert_level;
          Alarm_timer = 0;
      }

//#if defined(USE_SD_CARD)
// - allow logalarms even on FATFS
      if (fop->alarm_level > old_alarm_level && FlightLogOpen) {
          if (settings->logalarms || settings->logflight == FLIGHT_LOG_TRAFFIC)
            logOneTraffic(fop, "LPLTA");  // do not wait until logFlightPosition()
      }
//#endif
  }
}

static float oldrange[12];
static float newrange[12];
static uint32_t oldrange_n[12];
static uint16_t newrange_n[12];
static float oldrssi_mean;
static float newrssi_sum;
static float newrssi_dev;
static uint32_t oldrssi_n;
static uint32_t newrssi_n;
static float oldrssi_ssd;
static float newrssi_ssd;

static void zero_range_stats()
{
    memset(oldrange, 0, sizeof(oldrange));
    memset(newrange, 0, sizeof(newrange));
    memset(oldrange_n, 0, sizeof(oldrange_n));
    memset(newrange_n, 0, sizeof(newrange_n));
    oldrssi_mean = 0.0;
    newrssi_sum = 0.0;
    newrssi_dev = 0.0;
    oldrssi_n = 0;
    newrssi_n = 0;
    oldrssi_ssd = 0.0;
    newrssi_ssd = 0.0;
}

#define RANGESTATSVERSION 1
/*
    range.txt file structure:
        version_num = 1
        linrange,logrange,n  - for oclock=0: lin=km, log=log2(km)
        ...

        linrange,logrange,n  - for oclock=11
        rssi_mean,rssi_mean_square_deviation   - n=sum of range n's
*/
// try and load range stats from file
static bool load_range_stats()
{
    zero_range_stats();
    if (! FILESYS.exists("/range.txt")) {
        Serial.println("range.txt does not exist");
        return false;
    }
    File statsfile = FILESYS.open("/range.txt", FILE_READ);
    if (! statsfile)
        return false;
    char buf[64];
    if (! getline(statsfile, buf, 64)) {
        statsfile.close();
        Serial.println("empty range.txt");
        return false;
    }
    int file_version;
    sscanf(buf, "%d", &file_version);
    if (file_version != RANGESTATSVERSION) {
        Serial.println("wrong version of range.txt");
        statsfile.close();
        return false;
    }
    // read rest of file into range_stats[]
    Serial.println("reading range.txt...");
    for (int oclock=0; oclock<12; oclock++) {
        if (! getline(statsfile, buf, 64)) {
            Serial.println("range.txt ended early");
            statsfile.close();
            zero_range_stats();
            return false;
        }
        //Serial.println(buf);
        float file_linrange;    // read but ignored
        sscanf(buf, "%f,%f,%d",
            &file_linrange,
            &oldrange[oclock],
            &oldrange_n[oclock]);
        snprintf(buf, 64, "%2d: %.1f,%f,%d",
            oclock,
            file_linrange,
            oldrange[oclock],
            oldrange_n[oclock]);
        Serial.println(buf);
        oldrssi_n += oldrange_n[oclock];
    }
    if (! getline(statsfile, buf, 64)) {
        Serial.println("range.txt ended early");
        statsfile.close();
        zero_range_stats();
        return false;
    }
    Serial.print("rssi: ");
    Serial.println(buf);
    sscanf(buf, "%f,%f", &oldrssi_mean, &oldrssi_ssd);
    statsfile.close();
    return true;
}

void sample_range(container_t *fop)
{
    if (! ThisAircraft.airborne)       return;
    if (! fop->airborne)               return;
    if (fop->tx_type < TX_TYPE_FLARM)  return;
    if (fop->distance < 1000.0)        return;
    if (4.0 * fabs(fop->alt_diff) > fop->distance)    return;
    int oclock = fop->RelativeHeading + 15;
    if (oclock < 0)     oclock += 360;
    if (oclock >= 360)  oclock -= 360;
    oclock /= 30;
    newrange[oclock] += log2(0.001f * fop->distance);
    ++newrange_n[oclock];
    float rssi = (float) fop->rssi;
    newrssi_sum += rssi;
    float rssi_dev = rssi - oldrssi_mean;
    newrssi_dev += rssi_dev;
    newrssi_ssd += rssi_dev * rssi_dev;
    ++newrssi_n;
}

// this is called after landing
void save_range_stats()
{
    if (newrssi_n == 0)  // no new data
        return;
    FILESYS.remove("/oldrange.txt");
    FILESYS.rename("/range.txt", "/oldrange.txt");
    File statsfile = FILESYS.open("/range.txt", FILE_WRITE);
    Serial.print(newrssi_n);
    Serial.println("new samples, writing range.txt...");
    statsfile.println((int)RANGESTATSVERSION);
    char buf[64];
    for (int oclock=0; oclock<12; oclock++) {
        if (newrange_n[oclock]) {
            newrange[oclock] += oldrange[oclock] * oldrange_n[oclock];
            newrange_n[oclock] += oldrange_n[oclock];
            newrange[oclock] /= (float) newrange_n[oclock];     // new mean
        } else {   // no new samples in this oclock
            newrange[oclock] = oldrange[oclock];
            newrange_n[oclock] = oldrange_n[oclock];
        }
        snprintf(buf, 64, "AN,%.1f,%f,%d",
            (newrange_n[oclock]? exp2(newrange[oclock]) : 0.0),
            newrange[oclock],
            newrange_n[oclock]);
        Serial.println(buf+3);
        statsfile.println(buf+3);   // skip the "AN,"
        FlightLogComment(buf);      // - it will prepend LPLT, resulting in, e.g., LPLTAN,...
    }
    newrssi_sum += oldrssi_mean * (float) oldrssi_n;   // total new sum
    newrssi_n   += oldrssi_n;                // total count
    //if (newrssi_n)
        newrssi_sum /= (float) newrssi_n;    // new mean
    float devsquared = (newrssi_dev * newrssi_dev);
    // - newrssi_sum is the *total* (new & old data) deviations from the old mean
    oldrssi_ssd *= (float) oldrssi_n;    // old summed square deviations from mean
    newrssi_ssd += oldrssi_ssd;          // total squared deviations from old mean
    //if (newrssi_n) {
        newrssi_ssd -= devsquared / (float) newrssi_n;   // new variance
        newrssi_ssd /= (float) newrssi_n;                // new mean_square_deviation
    //}
    snprintf(buf, 64, "AN,%f,%f", newrssi_sum, newrssi_ssd);
    Serial.println(buf+3);
    statsfile.println(buf+3);   // skip the "AN,"
    FlightLogComment(buf);      // - it will prepend LPLT, resulting in, e.g., LPLTAN,...
    statsfile.close();
    load_range_stats();         // in case of another flight
}

/* relay landed-out or ADS-B traffic if we are airborne */
void air_relay(container_t *cip)
{
    static uint32_t lastrelay = 0;

    bool tryrelay = true;
    bool relayed = false;
    bool often = false;
    bool landed_out = ((cip->protocol == RF_PROTOCOL_LATEST || cip->protocol == RF_PROTOCOL_LEGACY)
                        && cip->aircraft_type == AIRCRAFT_TYPE_UNKNOWN);
    bool normal_protocol = (current_RF_protocol == settings->rf_protocol);

    if (landed_out) {
        often = true;
    } else {
        // must be ADS-B (since no relay if *our* protocol is not Latest or Legacy)
        // - unless RELAY_ONLY, then it may be FLARM traffic
        if (settings->relay < RELAY_ALL)     // RELAY_LANDED
            return;
        if (! normal_protocol)  // do not altprotocol relay non-landed-out
            return;
        if (settings->relay == RELAY_ONLY && cip->tx_type < TX_TYPE_FLARM)
            return;
        if (cip->aircraft_type != AIRCRAFT_TYPE_JET && cip->aircraft_type != AIRCRAFT_TYPE_HELICOPTER) {
            if (cip->distance > 10000)  // only relay gliders and light planes if close
                return;
            // - The idea is that if the aircraft is also sending FLARM signals, then if close
            //     those signals will be received, and other protocols will be ignored.
            //   Thus if close and another protocol, then no FLARM, and safe to relay,
            //     meaning it won't make FLARM "see itself" and go crazy.
        }
        often = true;
    }

    // if alternated to another protocol (presumably OGNTP) for this time (always slot 1),
    // relay in alt protocol, but don't update timerelayed so will be relayed normally too

    if (normal_protocol) {

        // only relay once in a while:
        //   5 seconds for any, 15 for same aircraft (7 for ADS-B or landed out)
        if (millis() < lastrelay + 1000*ANY_RELAY_TIME)
            return;
        if (cip->timerelayed + (often? ANY_RELAY_TIME+2 : ENTRY_RELAY_TIME) > cip->timestamp)
            return;

        // only try and relay during first time slot, to maximize chance
        // that OGN ground stations (in North America) will receive it
        // (they may also receive a relay via OGNTP in slot 1)
        tryrelay = (RF_current_slot == 0);
    }
    // else alt protocol, only happens every 16 seconds, in slot 1

    if (tryrelay && ! RF_Transmit_Happened()      // no transmission yet in this time slot 
            && (millis()+15 < TxEndMarker)) {     // enough time left in current time slot
        delay(10);  // give receivers in other aircraft time to process the original packet
        // re-encode packets for relaying (might be in LEGACY, LATEST or OGNTP protocol)
        size_t s = RF_Encode(cip, false);     // no wait
        if (s != 0)
            relayed = RF_Transmit(s, false);   // no wait - use incoming packet's random timing
    }

    if (cip->timerelayed == 0) {           // first relay (since new or expired)
        cip->timerelayed = 1;              // may be overwritten below with real timestamp
        if (relayed && ! landed_out && settings->logflight == FLIGHT_LOG_TRAFFIC) {
            snprintf_P(NMEABuffer, sizeof(NMEABuffer),
              PSTR("$PSRLY,%02d:%02d,%06x,%s\r\n"),
              gnss.time.hour(), gnss.time.minute(), cip->addr, cip->callsign);
            NMEAOutC(NMEA_T);
            FlightLogComment(NMEABuffer+3);    // will appear as LPLTRLY
        }
    }

    if (relayed) {
        if (normal_protocol) {
            cip->timerelayed = ThisAircraft.timestamp;
            lastrelay = millis();
        }
        if (landed_out) {
            Serial.print("Relayed packet from landed-out aircraft ");
            Serial.print(cip->addr, HEX);
            if (normal_protocol)
                Serial.println("");
            else
                Serial.println(" in alt protocol ");
        } else {
            if (cip->tx_type < TX_TYPE_FLARM) {
                Serial.print("Relayed ADS-B packet from ");
                Serial.println((char *) cip->callsign);
            } else {
                Serial.print("Relayed packet from ");
                Serial.println(cip->addr, HEX);
            }
        }
        if ((settings->nmea_d || settings->nmea2_d) && (settings->debug_flags)) {
            snprintf_P(NMEABuffer, sizeof(NMEABuffer),
              PSTR("$PSARL,1,%06X,%ld\r\n"),
              cip->addr, cip->timerelayed);
            NMEAOutD();
        }
    } else if (/* not relayed && */ landed_out) {
        // reserve a future time slot for relay message
        relay_waiting = true;
        // - prevent transmission of this aircraft's position during next Slot 0
        // - the hope is that another packet will then arrive from the landed_out aircraft
        Serial.println("try relay landed-out aircraft next slot 0");
        if (settings->altprotocol != RF_PROTOCOL_NONE) {
            alt_relay_waiting = true;
            // - prevent transmission of this aircraft's position during next altprotocol slot
            Serial.println("try relay landed-out aircraft next altprotocol slot");
        }
    }
    // ADS-B packets arrive at random times, don't try and reserve a time slot
}

// update fields from a received packet into Container[]
void CopyTraffic(container_t *cip, ufo_t *fop, const char *callsign)
{
    cip->addr = fop->addr;
    cip->latitude = fop->latitude;
    cip->longitude = fop->longitude;
    cip->altitude = fop->altitude;
    cip->pressure_altitude = fop->pressure_altitude;
    cip->timestamp = fop->timestamp;
    cip->gnsstime_ms = fop->gnsstime_ms;
    cip->speed = fop->speed;
    cip->course = fop->course;
    cip->turnrate = fop->turnrate;
    cip->vs = fop->vs;
    cip->hdop = fop->hdop;
    cip->last_crc = fop->last_crc;
    cip->protocol = fop->protocol;
    cip->tx_type = fop->tx_type;
    cip->addr_type = fop->addr_type;
    cip->aircraft_type = fop->aircraft_type;
    cip->airborne = fop->airborne;
    cip->circling = fop->circling;
    cip->stealth = fop->stealth;
    cip->no_track = fop->no_track;
    cip->relayed = fop->relayed;

    cip->rssi = RF_last_rssi;

    // if callsign was passed, copy it into Container[]
    if (callsign) {
        if ((callsign[0]      != '\0' && callsign[0]      != ' ')
        &&  (cip->callsign[0] == '\0' || cip->callsign[0] == ' ')) {
            strncpy((char *) cip->callsign, callsign, 8);
            cip->callsign[8] = '\0';
            cip->callsign[9] = '\0';
        }
    }
    // if callsign was not received, compute USA N-number from ICAO ID (if in range)
    icao_to_n(cip);
}

void report_landed_out(ufo_t *fop)
{
    snprintf_P(NMEABuffer, sizeof(NMEABuffer),
       PSTR("$PSRLO,%02d:%02d,%06x,%.5f,%.5f\r\n"),
       gnss.time.hour(), gnss.time.minute(), fop->addr, fop->latitude, fop->longitude);
    NMEAOutC(NMEA_T);
    FlightLogComment(NMEABuffer+4);    // will appear as LPLTLO
    // also output to alarmlog
    if (AlarmLogOpen)
        AlarmLog.print((const char *) NMEABuffer);
}

void AddTraffic(ufo_t *fop, const char *callsign)
{
    container_t *cip;

    bool do_relay = false;

    if (settings->rf_protocol == RF_PROTOCOL_LATEST || settings->rf_protocol == RF_PROTOCOL_LEGACY) {
      // relay some traffic - only if we are airborne (or in "relay only" mode)
      if (settings->relay != RELAY_OFF
          && (ThisAircraft.airborne || settings->relay == RELAY_ONLY)
          && fop->relayed == false         // not a packet already relayed one hop
          && fop->tx_type > TX_TYPE_S      // not a non-directional target
          && (settings->debug_flags & DEBUG_SIMULATE) == 0)
      {
            do_relay = true;
      }
    }

    /* first check whether we are already tracking this object */
    int i;
    for (i=0; i < MAX_TRACKING_OBJECTS; i++) {

      cip = &Container[i];

      if (cip->addr == fop->addr) {

        bool fop_adsb = fop->protocol == RF_PROTOCOL_GDL90 || fop->protocol == RF_PROTOCOL_ADSB_1090;
        bool cip_adsb = cip->protocol == RF_PROTOCOL_GDL90 || cip->protocol == RF_PROTOCOL_ADSB_1090;

        if (fop_adsb && (! cip_adsb)) {
            // ignore external (ADS-B) data about aircraft we also receive from directly
            // - unless we heard from only via relay, accept direct data instead
            if (cip->relayed == 0 &&
                OurTime <= cip->timestamp + ENTRY_EXPIRATION_TIME)
                return;
            // take over this slot (fall through)
        }

        // overwrite external (ADS-B) data about aircraft that also has FLARM
        // - unless the "FLARM" is relayed, which may have originated as ADS-B
        else if (cip_adsb && (! fop_adsb)) {
            if (fop->relayed &&
                OurTime <= cip->timestamp + ENTRY_EXPIRATION_TIME)
                return;
            // else fall through
        }

        // if both are from ADS-B, prefer direct over TIS-B (relayed ADS-B treated as TIS-B)
        else if (cip_adsb && fop_adsb) {
            if (fop->tx_type == TX_TYPE_TISB && cip->tx_type > TX_TYPE_TISB
                && OurTime <= cip->timestamp + ENTRY_EXPIRATION_TIME)
                return;
            // else fall through
        }

        /* ignore "new" positions that are exactly the same as before */
        if (fop->altitude == cip->altitude &&
            fop->latitude == cip->latitude &&
            fop->longitude == cip->longitude) {
                cip->last_crc  = fop->last_crc;      // so 2nd time slot packet will be ignored
                cip->timestamp = fop->timestamp;     // so it won't expire
                if (do_relay)  air_relay(cip);
                return;
        }

        /* overwrite old entry, but preserve fields that store history */

        if ((fop->gnsstime_ms - cip->gnsstime_ms > 1200)
          /* packets spaced far enough apart, store new history */
        || (fop->gnsstime_ms - cip->prevtime_ms > 2600)) {
          /* previous history getting too old, drop it */
          /* this means using the past data from < 1200 ms ago */
          /* to avoid that would need to store data from yet another time point */
          cip->prevtime_ms  = cip->gnsstime_ms;
          cip->prevcourse   = cip->course;
          cip->prevheading  = cip->heading;
          /* cip->prevspeed = cip->speed; */
          cip->prevaltitude = cip->altitude;
        }
        // else retain the older history for now

        if (cip->aircraft_type != AIRCRAFT_TYPE_UNKNOWN
         && fop->aircraft_type == AIRCRAFT_TYPE_UNKNOWN
         && fop->airborne == 0) {
            // switched from normal to landed-out
            report_landed_out(fop);
        }

        CopyTraffic(cip, fop, callsign);
        Calc_Traffic_Distances(cip);
        // Now can update alarm_level
        Traffic_Update(cip);
        if (do_relay)  air_relay(cip);
        return;
      }
    }

    /* new object, try and find a slot for it */

    if (fop->aircraft_type == AIRCRAFT_TYPE_UNKNOWN && fop->airborne == 0) {
        // landed-out
        report_landed_out(fop);
    }

    /* replace an empty object if found */
    for (i=0; i < MAX_TRACKING_OBJECTS; i++) {
      if (Container[i].addr == 0) {
        cip = &Container[i];
        //*cip = EmptyContainer;
        EmptyContainer(cip);
        CopyTraffic(cip, fop, callsign);
        Calc_Traffic_Distances(cip);
        Traffic_Update(cip);
        sample_range(cip);
        if (do_relay)  air_relay(cip);
        return;
      }
    }
    /* replace an expired object if found */
    for (i=0; i < MAX_TRACKING_OBJECTS; i++) {
      if (OurTime > Container[i].timestamp + ENTRY_EXPIRATION_TIME) {
        cip = &Container[i];
        //*cip = EmptyContainer;
        EmptyContainer(cip);
        CopyTraffic(cip, fop, callsign);
        Calc_Traffic_Distances(cip);
        Traffic_Update(cip);
        //sample_range(cip);
        if (do_relay)  air_relay(cip);
        return;
      }
    }

    /* may need to replace a non-expired object:   */
    /* identify the least important current object */

    // we can't compute the alarm level of the new object yet
    // so just assume that if it deserves an alarm then it is
    // likely closer than some other object in the (full) table

    /* identify the farthest-away non-"followed" object */
    /*    (distance adjusted for altitude difference)   */
    uint32_t follow_id = settings->follow_id;
    int max_dist_ndx = MAX_TRACKING_OBJECTS;
    float max_dist = 0;
    float adj_distance;
    for (i=0; i < MAX_TRACKING_OBJECTS; i++) {
      if (Container[i].alarm_level == ALARM_LEVEL_NONE
          && Container[i].addr != follow_id && Container[i].relayed == false) {
        adj_distance = Container[i].adj_distance;
        if (adj_distance < Container[i].distance)
            adj_distance = Container[i].distance;
        if (adj_distance > max_dist)  {
          max_dist_ndx = i;
          max_dist = adj_distance;
        }
      }
    }

    /* replace the farthest currently-tracked object, */
    /* but only if the new object is closer (or "followed", or relayed) */;
    Stash_Traffic_Distances(fop);
    adj_distance = stash.distance + VERTICAL_SLOPE * fabs(stash.alt_diff);
    if (max_dist_ndx < MAX_TRACKING_OBJECTS
        && (adj_distance < max_dist || fop->addr == follow_id || fop->relayed)) {
      cip = &Container[max_dist_ndx];
      //*cip = EmptyContainer;
      EmptyContainer(cip);
      CopyTraffic(cip, fop, callsign);
      Copy_Traffic_Distances(cip);     // computed above by Stash_Traffic_Distances(fop)
      Traffic_Update(cip);
      //sample_range(cip);   - do not sample, aircraft may be closer than max range
      if (do_relay)  air_relay(cip);
      return;
    }

    /* otherwise ignore the new object */
}

void ParseData(void)
{
    uint8_t rf_protocol = settings->rf_protocol;
    size_t rx_size = RF_Payload_Size(rf_protocol);
    rx_size = rx_size > sizeof(fo_raw) ? sizeof(fo_raw) : rx_size;

    if (memcmp(RxBuffer, TxBuffer, rx_size) == 0) {
Serial.print("RF loopback is detected, rx_try=");     // seen on the sx1262?
Serial.println(which_rx_try);
      if (settings->nmea_p) {
        StdOut.println(F("$PSRFE,RF loopback is detected"));
      }
      //rx_packets_counter--;
      return;
    }

    memcpy(fo_raw, RxBuffer, rx_size);
    if (settings->nmea_p) {
      StdOut.print(F("$PSRFI,"));
      StdOut.print((unsigned long) now()); StdOut.print(F(","));
      StdOut.print(Bin2Hex(fo_raw, rx_size)); StdOut.print(F(","));
      StdOut.println(RF_last_rssi);
    }

    //fo = EmptyFO;  /* to ensure no data from past packets remains in any field */
    EmptyFO(&fo);    /* to ensure no data from past packets remains in any field */

    if (protocol_decode == NULL)
        return;

    if (((*protocol_decode)((void *) fo_raw, &ThisAircraft, &fo)) == false)
        return;

    if (fo.tx_type == TX_TYPE_NONE)   // not ADS-B or other external sources
        fo.tx_type = TX_TYPE_FLARM;   // may actually be OGNTP or P3I or FANET...

    if (settings->rf_protocol == RF_PROTOCOL_ADSB_UAT)
//  ||  settings->rf_protocol == RF_PROTOCOL_ADSB_1090)
        AddTraffic(&fo, fo_callsign);
    else
        AddTraffic(&fo, (char *) NULL);
}

void Traffic_setup()
{
  switch (settings->alarm)
  {
  case TRAFFIC_ALARM_NONE:
    Alarm_Level = &Alarm_None;
    break;
  case TRAFFIC_ALARM_VECTOR:
    Alarm_Level = &Alarm_Vector;
    break;
  case TRAFFIC_ALARM_LATEST:
    Alarm_Level = &Alarm_Latest;
    break;
  case TRAFFIC_ALARM_DISTANCE:
  default:
    Alarm_Level = &Alarm_Distance;
    break;
  }

  load_range_stats();

#if defined(USE_SD_CARD)
    if (settings->rx1090
    && (settings->debug_flags & DEBUG_DEEPER)
    && (settings->nmea_d || settings->nmea2_d))
        SD_log("$PSADX,addr,tx_type,maxrssirelalt,mindist,mindistrssi,maxrssi\r\n");
#endif
}

void Traffic_loop()
{
    if (! isTimeToUpdateTraffic())
        return;

    container_t *mfop = NULL;
    max_alarm_level = ALARM_LEVEL_NONE;          /* global, used for visual displays */
    alarm_ahead = false;                         /* global, used for strobe pattern */
    int sound_alarm_level = ALARM_LEVEL_NONE;    /* local, used for sound alerts */
    int alarmcount = 0;

    for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {

      container_t *fop = &Container[i];

      if (fop->addr) {  /* non-empty Container slot */

        // expire non-directional targets early
        uint32_t expiration_time = (fop->tx_type <= TX_TYPE_S)? NONDIR_EXPIRATION : ENTRY_EXPIRATION_TIME;

        if (OurTime <= fop->timestamp + expiration_time) {

          if ((RF_time - fop->timestamp) >= TRAFFIC_VECTOR_UPDATE_INTERVAL)
              continue;

          /* determine the highest alarm level seen at the moment */
          if (fop->alarm_level > max_alarm_level)
              max_alarm_level = fop->alarm_level;

          /* determine if any traffic with alarm level low+ is "ahead" */
          /* - this is for the strobe, increase flashing if "ahead" */
          if (fop->alarm_level >= ALARM_LEVEL_LOW) {
              if (abs(fop->RelativeHeading) < 45)
                  alarm_ahead = true;
          }

          /* figure out what is the highest alarm level needing a sound alert */
          if (fop->alarm_level > fop->alert_level
                   && fop->alarm_level > ALARM_LEVEL_CLOSE) {
              ++alarmcount;
              if (fop->alarm_level > sound_alarm_level) {
                  sound_alarm_level = fop->alarm_level;
                  mfop = fop;
              }
          }

        } else {   /* expired ufo */

// send out summary data about the aircraft
if (fop->protocol == RF_PROTOCOL_ADSB_1090 && (settings->debug_flags & DEBUG_DEEPER)) {
  if (settings->nmea_d || settings->nmea2_d) {
    snprintf_P(NMEABuffer, sizeof(NMEABuffer),
      PSTR("$PSADX,%06X,%d,%d,%d,%d,%d\r\n"),
      fop->addr, fop->tx_type, (int)fop->maxrssirelalt, (int)fop->mindist, fop->mindistrssi, fop->maxrssi);
    NMEAOutD();
#if defined(USE_SD_CARD)
    SD_log(NMEABuffer);
#endif
  } else {
    Serial.print(fop->addr, HEX);
    Serial.print(" expiring, tx_type ");
    Serial.print(fop->tx_type);
    Serial.print(" max-RSSI rel alt (ft): ");
    Serial.print((int)(3.2808*fop->maxrssirelalt));
    Serial.print(" min distance (m): ");
    Serial.print((int)fop->mindist);
    Serial.print(" min-dist RSSI: ");
    Serial.print(fop->mindistrssi);
    Serial.print(" max RSSI: ");
    Serial.println(fop->maxrssi);
  }
}
          sample_range(fop);

          // EmptyContainer(fop);
          fop->addr = 0;

          /* implied by empty:
          fop->addr = 0;
          fop->alert = 0;
          fop->alarm_level = 0;
          fop->alert_level = 0;
          fop->prevtime_ms = 0;
          etc... */
        }
      }
    }

    if (sound_alarm_level > ALARM_LEVEL_CLOSE) {   // implies mfop != NULL
      // use alarmcount to modify the sounds
      bool notified = Buzzer_Notify(sound_alarm_level, (alarmcount > 1));
#if !defined(EXCLUDE_VOICE)
#if defined(ESP32)
      //if (mfop != NULL)
        notified |= Voice_Notify(mfop, (alarmcount > 1));
#endif
#endif

      /* Doing this here means that alarms via $PSRAA follow the same
       * hysteresis algorithm as the sound alarms.
       * External devices (XCsoar) sounding alarms based on $PFLAU have
       * their own algorithms, are not affected and may be continuous */
      snprintf_P(NMEABuffer, sizeof(NMEABuffer), PSTR("$PSRAA,%d,%d,%06X*"),
                     sound_alarm_level-1, (alarmcount > 1), mfop->addr);
      NMEAOutC(NMEA_E_OUTPUT);

      //if (mfop != NULL) {
        // if (notified)
        {
          mfop->alert_level = mfop->alarm_level;   // was + 1;
          // warn again if alarm level gets higher than current one
          // also warn again for same level after 9 seconds
          if (Alarm_timer == 0)
              Alarm_timer = millis() + 9000;    // when may warn again about this aircraft
          mfop->alert |= TRAFFIC_ALERT_SOUND;   // not actually used for anything
        }
#if defined(FILESYS)
        if (alarmcount>0 && settings->logalarms && AlarmLogOpen) {
          int year  = gnss.date.year();
          if( year > 99)  year = year - 2000;
          int month = gnss.date.month();
          int day   = gnss.date.day();
          // $GPGGA,235317.00,4003.90395,N,10512.57934,W,...
          char *cp = &GPGGA_Copy[7];   // after the "$GPGGA,", start of timestamp
          char *ep = &GPGGA_Copy[35];
          while (*ep != 'E' && *ep != 'W') {
              if (ep > &GPGGA_Copy[48])  break;
              if (*ep == '\0')  break;
              ++ep;
          }
          ++ep;
          *ep = '\0';       // overwrite the comma after the "E" or "W"
//Serial.print("GGA time & position: ");
//Serial.println(cp);
          //int rel_bearing = (int) (mfop->bearing - ThisAircraft.course);
          //rel_bearing += (rel_bearing < -180 ? 360 : (rel_bearing > 180 ? -360 : 0));
          snprintf_P(NMEABuffer, sizeof(NMEABuffer),
              PSTR("%02d%02d%02d,%s,%d,%d,%06x,%d,%d,%d\r\n"),
              year, month, day, cp, mfop->alarm_level-1, alarmcount,
              mfop->addr, (int)mfop->RelativeHeading, (int)mfop->distance, (int)mfop->alt_diff);
//Serial.println(NMEABuffer);
          int len = strlen(NMEABuffer);
          if (AlarmLog.write((const uint8_t *)NMEABuffer, len) == len) {
              AlarmLog.flush();
          } else {
              // perhaps out of space in FILESYS
              AlarmLog.close();
              AlarmLogOpen = false;
          }
        }
#endif
      //}
    }

    UpdateTrafficTimeMarker = millis();
}

// currently this function is not called from anywhere (in "normal" mode)
//   - instead expired entries are purged in Traffic_loop()
void ClearExpired()
{
  for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {
    if (Container[i].addr &&
         (OurTime > Container[i].timestamp + ENTRY_EXPIRATION_TIME)) {
      //EmptyContainer(&Container[i]);
      Container[i].addr = 0;
    }
  }
}

int Traffic_Count()
{
  int count = 0;
  adsb_acfts = 0;
  int rssimax = -126;
  for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {
    if (Container[i].addr) {
      count++;
      int rssi = Container[i].rssi;
      if (rssi < 0) {               // not an ADS-B RSSI
          if (rssi > rssimax)
              rssimax = rssi;
      } else {
          ++adsb_acfts;
      }
    }
  }
  if (rssimax > -126)
      maxrssi = rssimax;
  else
      maxrssi = 0;
  return count;
}

/* this is used in Text_EPD.cpp for 'radar' display, */
/* so don't adjust for altitude difference.          */

int traffic_cmp_by_distance(const void *a, const void *b)
{
  traffic_by_dist_t *ta = (traffic_by_dist_t *)a;
  traffic_by_dist_t *tb = (traffic_by_dist_t *)b;

  if (ta->distance >  tb->distance) return  1;
/*  if (ta->distance == tb->distance) return  0; */
  if (ta->distance <  tb->distance) return -1;
  return  0;
}

/* called (as needed) from softRF.ino normal(), or from ParseData() above, */
/*   or every few minutes from Estimate_Wind() if ADDR_TYPE_RANDOM         */
void generate_random_id()
{
    uint32_t id = millis();
    id = (id ^ (id<<5) ^ (id>>5)) & 0x000FFFFF;
    if (settings->id_method == ADDR_TYPE_RANDOM)
      id |= 0x000E0000;
    else
      id |= 0x000F0000;
    id |= 0x00400000;
    ThisAircraft.addr = id;
}
