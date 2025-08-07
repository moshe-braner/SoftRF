/*
 * Wind.cpp
 * Copyright (C) 2022 Moshe Braner
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

#include "system/SoC.h"
#include "TrafficHelper.h"
#include "Wind.h"
#include "driver/Settings.h"
#include "driver/GNSS.h"
#include "driver/RF.h"
#include "driver/Filesys.h"
#include "protocol/radio/Legacy.h"
#include "protocol/data/NMEA.h"
#include "protocol/data/IGC.h"
#include "protocol/data/GNS5892.h"

float wind_best_ns = 0.0;  /* mps */
float wind_best_ew = 0.0;
float wind_speed = 0.0;
float wind_direction = 0.0;
// time_t AirborneTime = 0;

static float avg_abs_turnrate = 0.0;  /* absolute - average when circling */
static float avg_speed = 0.0;     /* average around the circle */
static float avg_climbrate = 0.0; /* fpm, based on GNSS data */

void Estimate_Wind()
{
  static uint32_t old_gnsstime = 0;
  static float old_lat = 0.0;     /* where a circle started, on its Northern edge */
  static float old_lon = 0.0;     /* where a circle started, on its Eastern edge */
  static uint32_t start_time = 0;
  static uint32_t turning_time = 0;
  static uint32_t old_lat_time = 0;    /* when those circles started */
  static uint32_t old_lon_time = 0;
  static float old_turnrate = 0.0;
  static float old_course = 0.0;
  static float cumul_turn = 0.0;  /* cumulative change of direction around circle */
  static int oldquadrant = 0;
  static float min_gs = 999.0;    /* min and max ground speed around circle */
  static float max_gs = 0.0;
  static float min_gs_course = 0.0;   /* directions of those min and max ground speeds */
  static float max_gs_course = 0.0;
  static float prev_gs_ns, prev_gs_ew, prev_cd_ns, prev_cd_ew;
  static float weight_gs, weight_ns, weight_ew;
  static float weight_00 = 1.0;
  static uint32_t decaytime = 0;
  static uint8_t first_circles;
  static bool this_circle;

  bool ok = false;
  bool ns = false;
  bool ew = false;

//  if (! ThisAircraft.airborne) {
//      weight_00 = 1.0;
//      return;
//  }

  uint32_t gnsstime_ms = ThisAircraft.gnsstime_ms;

  /* ignore repeats of same GPS reading */
  if (gnsstime_ms < old_gnsstime + 600)
    return;

  // this function is called every 666 ms
  // but a new GNSS reading is only once per second

  /* recover from GPS outage */
  if (gnsstime_ms - old_gnsstime > 5500) {
     old_gnsstime = gnsstime_ms;
     ThisAircraft.circling = 0;
     old_turnrate = 0.0;
     avg_abs_turnrate = 0.0;
     //avg_speed = 0.0;
     return;
  }

  old_gnsstime = gnsstime_ms;

  project_this(&ThisAircraft);              // which also calls this_airborne()
  float turnrate = ThisAircraft.turnrate;   // was computed in project_this()

  float course_change, interval, abs_turnrate, wind_ns, wind_ew;

  bool turning = (fabs(turnrate) > 2.0 && fabs(turnrate) < 50.0);
  if ((ThisAircraft.circling > 0 && turnrate < 0)
   || (ThisAircraft.circling < 0 && turnrate > 0))
        turning = false;

  if (turning) {
      /* watch for changes in ground speed */
      turning_time = gnsstime_ms;
      if (ThisAircraft.speed > max_gs) {
        max_gs = ThisAircraft.speed;               // knots
        max_gs_course = ThisAircraft.course;
      }
      if (ThisAircraft.speed < min_gs) {
        min_gs = ThisAircraft.speed;
        min_gs_course = ThisAircraft.course;
      }
  }

  if (ThisAircraft.circling == 0) { /* not considered in a stable circle */

    /* slowly decay wind towards zero over time */
    /* with a half-life of about 20 minutes     */
    /* - but new circling will refresh it       */
    if (gnsstime_ms > decaytime) {
        decaytime = gnsstime_ms + 100000;
        wind_best_ns *= 0.95;
        wind_best_ew *= 0.95;
        wind_speed  *= 0.95;

        /* piggy-back random time for new random ID */
        if (settings->id_method == ADDR_TYPE_RANDOM)   // get a new ID repeatedly
             generate_random_id();
    }

    if (old_turnrate == 0.0) {   // was not turning
      if (turning) {             // now turning
        // start watching whether stable
        // also start observing total turn and ground speed variation
        old_turnrate = turnrate;
        old_course = ThisAircraft.course;
        first_circles = 2;
        cumul_turn = 0.0;
        start_time = gnsstime_ms;
        min_gs = 999.0;
        max_gs = 0.0;
#if defined(USE_SD_CARD)
//if (settings->debug_flags & DEBUG_WIND) {
//snprintf_P(NMEABuffer, sizeof(NMEABuffer)," now turning crs=%.1f tr=%.1f\r\n", old_course, turnrate);
//FlightLogComment(NMEABuffer);
//}
#endif
      }
      return;
    }
  }
    
  // got here if (old_turnrate != 0.0) or circling

  if (ThisAircraft.circling && !turning) {  // meaning fabs(turnrate) < 2.0,
                                            // or turning the opposite way of the circling
      // turning_time is last time was still turning
      if (gnsstime_ms - turning_time > 1500) {    // more than momentarily
          old_lat_time = 0;        // do not measure drift in this circle
          old_lon_time = 0;
          if ((gnsstime_ms - turning_time > 5500)             // for a longer while
           || (ThisAircraft.circling > 0 && turnrate < -6.0)  // or turning sharper the wrong way
           || (ThisAircraft.circling < 0 && turnrate >  6.0)) {
if (settings->debug_flags & DEBUG_WIND) {
snprintf_P(NMEABuffer, sizeof(NMEABuffer)," stopped circling tr=%.1f\r\n", turnrate);
NMEAOutD();
#if defined(ESP32)
#if defined(USE_SD_CARD)
if (SD_is_mounted)
FlightLogComment(NMEABuffer);
#endif
#endif
}
              ThisAircraft.circling = 0;
              avg_abs_turnrate = 0.0;
              //avg_speed = 0.0;
              old_turnrate = 0.0;
              decaytime = gnsstime_ms + 100000;
          }        // else old_turnrate remains as-is
      }
      return;
  }

#if 0
  if ((ThisAircraft.circling > 0 && turnrate < -6.0)
   || (ThisAircraft.circling < 0 && turnrate >  6.0)) {
      /* turning the other way - not just a correction */ 
if (settings->debug_flags & DEBUG_WIND) {
snprintf_P(NMEABuffer, sizeof(NMEABuffer)," turning the other way tr=%.1f\r\n", turnrate);
#if defined(ESP32)
#if defined(USE_SD_CARD)
if (SD_is_mounted)
FlightLogComment(NMEABuffer);
}
#endif
#endif
      ThisAircraft.circling = 0;
      old_turnrate = 0.0;
      avg_abs_turnrate = 0.0;
      //avg_speed = 0.0;
      decaytime = gnsstime_ms + 100000;
      return;
  }
#endif

  // got here if was turning and still turning

  course_change = ThisAircraft.course - old_course;
  if (course_change >  180.0)  course_change -= 360.0;   // wraparound
  if (course_change < -180.0)  course_change += 360.0;
  cumul_turn += course_change;
  old_course = ThisAircraft.course;
  old_turnrate = turnrate;

  if (ThisAircraft.circling == 0) {

      if (fabs(cumul_turn) > 210.0) {     // completed somewhat more than a half-turn
if (settings->debug_flags & DEBUG_WIND) {
if (ThisAircraft.circling == 0) {
snprintf_P(NMEABuffer, sizeof(NMEABuffer)," circling %s tr=%.1f\r\n",
(cumul_turn>0.0? "right" : "left"), turnrate);
NMEAOutD();
#if defined(ESP32)
#if defined(USE_SD_CARD)
if (SD_is_mounted)
FlightLogComment(NMEABuffer);
#endif
#endif
}
}
          // declare state to be "circling", and set up to measure wind
          ThisAircraft.circling = (cumul_turn > 0.0? 1 : -1);
          this_circle = true;
          oldquadrant = 0;
          old_lat_time = 0;
          old_lon_time = 0;
          prev_gs_ns = prev_cd_ns = wind_best_ns;
          prev_gs_ew = prev_cd_ew = wind_best_ew;
          weight_gs = 0.04;
          weight_ns = 0.03;
          weight_ew = 0.03;   /* weights will increase or decrease later */
      }

      return;
  }

  /* processing when "circling": */

  /* note when a whole circle is done */

  if (fabs(cumul_turn) > 360.0) {  /* completed a circle */

      float direction, windspeed, airspeed;

     /* use ground speed observations to estimate wind */

      min_gs_course += 180.0;  /* point downwind */
      if (min_gs_course > 360.0)  min_gs_course -= 360.0;
      if (fabs(min_gs_course-max_gs_course) < 180.0) {       /* they do not straddle North */
        if (fabs(min_gs_course - max_gs_course) < 30.0) {    /* they are roughly the same */
          direction = 0.5 * (min_gs_course + max_gs_course);
          ok = true;
        }
      } else {                                               /* they do straddle North */
        if (min_gs_course > 270.0) {                         /* min_gs_course is W of N */
          if (fabs(360.0 - min_gs_course + max_gs_course) < 30.0)
            ok = true;
        } else if (max_gs_course > 270.0) {                  /* max_gs_course is W of N */
          if (fabs(360.0 - max_gs_course + min_gs_course) < 30.0)
            ok = true;
        }
        direction = 0.5 * (min_gs_course + max_gs_course - 360.0);
        if (direction < 0.0)  direction += 360.0;
      }
      if (first_circles)
          ok = false;      // ignore data from the first 2 circles of each thermal
      if (ok) {
        windspeed = (0.5f * _GPS_MPS_PER_KNOT) * (max_gs - min_gs);    // m/s
        airspeed  = (0.5f * _GPS_MPS_PER_KNOT) * (max_gs + min_gs);
      }
      if (ok && windspeed < 30.0 /* && windspeed > 1.0 */ ) {    /* ignore implausible values */
        wind_ns = windspeed * cos(D2R * direction);
        wind_ew = windspeed * sin(D2R * direction);      
        /* use large weights initially to dilute effect of initial estimate */
        //if (wind_best_ns == 0.0 || wind_best_ew == 0.0 || avg_speed == 0.0) {
        if (avg_speed == 0.0) {
            /* not initialized yet */
            weight_gs = weight_00;
        }
        /* only gradually change "best" estimate */
        wind_best_ns = (1.0 - weight_gs) * wind_best_ns + weight_gs * wind_ns;
        wind_best_ew = (1.0 - weight_gs) * wind_best_ew + weight_gs * wind_ew;
        avg_speed = (1.0 - weight_gs) * avg_speed + weight_gs * airspeed;
        /* give more weight to subsequent circles in same thermal */
        /* conversely if estimates are noisy reduce the weight    */
        if ((fabs(wind_best_ns) > 2.5 && fabs(wind_ns-prev_gs_ns) > 0.5*fabs(wind_best_ns))
         || (fabs(wind_best_ew) > 2.5 && fabs(wind_ew-prev_gs_ew) > 0.5*fabs(wind_best_ew))) {
           if (weight_gs > 0.03)  weight_gs -= 0.02;
        } else {
           if (weight_gs < 0.09)  weight_gs += 0.02;
        }
        prev_gs_ns = wind_ns;
        prev_gs_ew = wind_ew;
      }

      /* set up to observe the next circle */
      cumul_turn = 0.0;
      this_circle = true;
      if (first_circles)
          --first_circles;
      start_time = gnsstime_ms;
      min_gs = 999.0;
      max_gs = 0.0;

  }   /* done with GS around circle */

  /* also use drift while circling (steadily) to estimate wind */

  /* classify direction into 4 quadrants */

  int icourse = (int) (ThisAircraft.course + 0.5);
  if (icourse < 0)  icourse += 360;
  int quadrant = 1;
  if (icourse > 90) {
    ++quadrant;
    if (icourse > 180) {
      ++quadrant;
      if (icourse > 270)
        ++quadrant;
    }
  }

  /* detect completion of NS/EW-oriented circles */

  if (! first_circles) {
    if (ThisAircraft.circling > 0) {          /* clockwise */
      if (quadrant == 2 && oldquadrant == 1)  /* traveling East, on the North side of the circle */
        ns = true;
      if (quadrant == 3 && oldquadrant == 2)  /* traveling South, on the East side of the circle */
        ew = true;
    } else if (ThisAircraft.circling < 0) {   /* counter-clockwise */
      if (quadrant == 3 && oldquadrant == 4)  /* traveling West, on the North side of the circle */
        ns = true;
      if (quadrant == 4 && oldquadrant == 1)  /* traveling North, on the East side of the circle */
        ew = true;
    }
  }

  oldquadrant = quadrant;  /* set up to detect the next quadrant crossing */

  /* measure drift in completed circles */

  float drift_ns;
  if (ns) {  /* started and perhaps completed a circle */
    float new_lat = ThisAircraft.latitude;
    if (old_lat_time != 0) {  /* there is history to use */
      drift_ns = 111300.0 * (new_lat - old_lat);   /* how far further North, in meters */
      if (abs(drift_ns) > 300)  drift_ns = 0;      /* ignore implausible values */
      interval = 0.001 * (gnsstime_ms - old_lat_time);
      wind_ns = drift_ns / interval;               /* m/s */
      if (fabs(wind_ns-wind_best_ns) < 20.0 /* && fabs(wind_ns-wind_best_ns) > 1.0 */ ) {
        if (wind_best_ns == 0.0) {
          weight_ns = weight_00;
          wind_best_ns = wind_ns;
        } else {
          wind_best_ns = (1.0 - weight_ns) * wind_best_ns + weight_ns * wind_ns;
        }
        if (fabs(wind_best_ns) > 2.5 && fabs(wind_ns-prev_cd_ns) > 0.5*fabs(wind_best_ns))
          if (weight_ns > 0.02)  weight_ns -= 0.015;
        else
          if (weight_ns < 0.08)  weight_ns += 0.015;
        prev_cd_ns = wind_ns;
      }
    } else {
      ns = false;    // only use results when completing, not starting, a circle
    }
    old_lat = new_lat;  /* start observing a new circle */
    old_lat_time = gnsstime_ms;
  }

  float drift_ew;
  if (ew) {
    float new_lon = ThisAircraft.longitude;
    if (old_lon_time != 0) {
      drift_ew = 111300.0 * (new_lon - old_lon) * CosLat(/*ThisAircraft.latitude*/); /* how far further East */
      if (abs(drift_ew) > 300)  drift_ew = 0;
      interval = 0.001 * (gnsstime_ms - old_lon_time);
      wind_ew = drift_ew / interval;
      if (fabs(wind_ew-wind_best_ew) < 20.0 /* && fabs(wind_ew-wind_best_ew) > 1.0 */ ) {
        if (wind_best_ew == 0.0) {
          weight_ew = weight_00;
          wind_best_ew = wind_ew;
        } else {
          wind_best_ew = (1.0 - weight_ew) * wind_best_ew + weight_ew * wind_ew;
        }
        if (fabs(wind_best_ew) > 2.5 && fabs(wind_ew-prev_cd_ew) > 0.5*fabs(wind_best_ew))
          if (weight_ew > 0.02)  weight_ew -= 0.015;
        else
          if (weight_ew < 0.08)  weight_ew += 0.015;
        prev_cd_ew = wind_ew;
      }
    } else {
      ew = false;
    }
    old_lon = new_lon;  /* start observing a new circle */
    old_lon_time = gnsstime_ms;
  }

  if (ns || ew || ok) {
    abs_turnrate = 360000.0 / (float) (gnsstime_ms - start_time);  // absolute turnrate
    if (abs_turnrate > 50.0)  abs_turnrate = avg_abs_turnrate;   // ignore implausible data
    if (abs_turnrate <  2.0)  abs_turnrate = 0.0;                // ignore inaccurate data
    if (avg_abs_turnrate == 0.0)
        avg_abs_turnrate = abs_turnrate;
    else
        avg_abs_turnrate = 0.8 * avg_abs_turnrate + 0.2 * abs_turnrate;
    wind_speed = hypot(wind_best_ns, wind_best_ew);
    wind_direction = R2D * atan2(-wind_best_ew, -wind_best_ns);    // direction coming FROM
    if (wind_direction < 0.0)
        wind_direction += 360.0;
  }

  /* send data out via NMEA for debugging */
  if ((settings->nmea_d || settings->nmea2_d) && (settings->debug_flags & DEBUG_WIND)) {
    if (ok) {
      snprintf_P(NMEABuffer, sizeof(NMEABuffer),
        PSTR("$PSWGS,%ld,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.2f,%.1f,%.1f,%.1f,%.1f\r\n"),
        gnsstime_ms, ThisAircraft.speed, avg_speed, ThisAircraft.course,
        ThisAircraft.turnrate, ThisAircraft.circling*avg_abs_turnrate, min_gs_course, max_gs_course,
        weight_gs, wind_ns, wind_ew, wind_best_ns, wind_best_ew);
      NMEAOutD();
      snprintf_P(NMEABuffer, sizeof(NMEABuffer),
        PSTR("WGS,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.2f,%.1f,%.1f,%.1f,%.1f\r\n"),
        ThisAircraft.speed, avg_speed, ThisAircraft.course,
        ThisAircraft.turnrate, ThisAircraft.circling*avg_abs_turnrate, min_gs_course, max_gs_course,
        weight_gs, wind_ns, wind_ew, wind_best_ns, wind_best_ew);
      NMEAOutD();
#if defined(ESP32)
#if defined(USE_SD_CARD)
      if (SD_is_mounted)
        FlightLogComment(NMEABuffer);
#endif
#endif
    }
    if (ns) {
      snprintf_P(NMEABuffer, sizeof(NMEABuffer),
        PSTR("$PSWNS,%ld,%.5f,%.5f,%.1f,%.1f,%.1f,%.1f,%.1f,%.3f,%.1f,%.1f,%.1f\r\n"),
        gnsstime_ms, ThisAircraft.latitude, ThisAircraft.longitude,
        ThisAircraft.speed, ThisAircraft.course, ThisAircraft.turnrate,
        interval, drift_ns, weight_ns, wind_ns, wind_best_ns, wind_best_ew);
      NMEAOutD();
#if defined(ESP32)
#if defined(USE_SD_CARD)
      if (SD_is_mounted) {
        snprintf_P(NMEABuffer, sizeof(NMEABuffer),
          PSTR("WNS,%.0f,%.1f,%.3f,%.1f,%.1f,%.1f\r\n"),
          ThisAircraft.course, drift_ns, weight_ns, wind_ns, wind_best_ns, wind_best_ew);
        FlightLogComment(NMEABuffer);
      }
#endif
#endif
    }
    if (ew) {
      snprintf_P(NMEABuffer, sizeof(NMEABuffer),
        PSTR("$PSWEW,%ld,%.5f,%.5f,%.1f,%.1f,%.1f,%.1f,%.1f,%.3f,%.1f,%.1f,%.1f\r\n"),
        gnsstime_ms, ThisAircraft.latitude, ThisAircraft.longitude,
        ThisAircraft.speed, ThisAircraft.course, ThisAircraft.turnrate,
        interval, drift_ew, weight_ew, wind_ew, wind_best_ns, wind_best_ew);
      NMEAOutD();
#if defined(ESP32)
#if defined(USE_SD_CARD)
      if (SD_is_mounted) {
        snprintf_P(NMEABuffer, sizeof(NMEABuffer),
          PSTR("WEW,%.0f,%.1f,%.3f,%.1f,%.1f,%.1f\r\n"),
          ThisAircraft.course, drift_ew, weight_ew, wind_ew, wind_best_ns, wind_best_ew);
        FlightLogComment(NMEABuffer);
      }
#endif
#endif
      snprintf_P(NMEABuffer, sizeof(NMEABuffer),
        PSTR("$PSWSD,%.1f,%.0f\r\n"),
        wind_speed * (1.0 / _GPS_MPS_PER_KNOT), wind_direction);
      NMEAOutD();
#if defined(ESP32)
#if defined(USE_SD_CARD)
      if (SD_is_mounted) {
        snprintf_P(NMEABuffer, sizeof(NMEABuffer),
          PSTR("WSD,%.1f,%.0f\r\n"),
          wind_speed * (1.0 / _GPS_MPS_PER_KNOT), wind_direction);
        FlightLogComment(NMEABuffer);
      }
#endif
#endif
    }
  }

  if (ns || ew || ok) {
    if (this_circle) {     // reduce large weights no more than once per circle
        this_circle = false;
        if (weight_00 > 0.12)
            weight_00 *= 0.625;
        if (weight_gs > 0.12)
            weight_gs *= 0.625;
        if (weight_ns > 0.12)
            weight_ns *= 0.625;
        if (weight_ew > 0.12)
            weight_ew *= 0.625;
    }
  }

}


/* keep track of whether this aircraft is airborne */
void this_airborne(bool validfix)
{
    /* static vars to keep track of 'airborne' status: */
    static int airborne = -4;
    static float prevspeed = 0;
    static float initial_latitude = 0;
    static float initial_longitude = 0;
    static float initial_altitude = 0;

    int was_airborne = airborne;
    float speed = ThisAircraft.speed;

    if (initial_latitude == 0) {
      /* wait for stable fix */
      if (! GNSSTimeMarker)
        return;
      /* set initial location */
      initial_latitude  = ThisAircraft.latitude;
      initial_longitude = ThisAircraft.longitude;
      initial_altitude  = ThisAircraft.altitude;
    }

    if (! validfix) {

        if (airborne < 2)
            airborne = -4;
        else
            --airborne;        // after 60 calls (30 min) will be forced to "land"

    } else if (speed < 1.0) {

      if (airborne > 0) {
        airborne -= 2;
        /* after 30 calls (~30 sec if consecutive)
           with speed < 1 knot consider it a landing */
        // if alternating between >1kt and <1kt the -=2 will win over the ++
      } else /* not airborne */ {
        /* if had some speed and then stopped - reset to -4 again */
        airborne = -4;
        initial_latitude = 0;
      }

    } else if (airborne <= 0) {    /* not airborne but moving with speed > 1 knot */

      if (GNSSTimeMarker > 0 && ThisAircraft.prevtime_ms > 0) {  /* had fix for a while */

        if ( speed > 20.0                                               /* 20 knots  */
          || fabs(ThisAircraft.latitude - initial_latitude) > 0.0018f   /* about 200 meters */
          || fabs(ThisAircraft.longitude - initial_longitude) > 0.0027f
          || fabs(ThisAircraft.altitude - initial_altitude) > 120.0f) {
            /* movement larger than typical GNSS noise */
            uint32_t interval = ThisAircraft.gnsstime_ms - ThisAircraft.prevtime_ms;
            if (fabs(ThisAircraft.altitude - ThisAircraft.prevaltitude) > 0.020 * (float)interval
             || fabs(ThisAircraft.course - ThisAircraft.prevcourse) > 0.050 * (float)interval
             || speed > 4.0 * prevspeed || prevspeed > 4.0 * speed) {
               /* supposed initial movement is too jerky - wait for smoother changes */
            } else {
                ++airborne;
                /* require additional movements to call it airborne */
                initial_latitude  = ThisAircraft.latitude;
                initial_longitude = ThisAircraft.longitude;
                initial_altitude  = ThisAircraft.altitude;
            }
            prevspeed = speed;
            if (airborne > 0)    /* consistently good indications */
                 airborne = 60;  /* now really airborne */
        }
      }

    } else if (airborne < 60) {    /* airborne and moving with speed > 1 knot */

        ++airborne;  // so multiple momentary hovers will not accumulate to a "landing"

    }

    bool airborne_changed = false;
    if (ThisAircraft.airborne==0 && airborne>0) {
      airborne_changed = true;
      // AirborneTime = RF_time;
//#if defined(ESP32)
      startlogs();      // restart alarm log (and flight log) on first takeoff after boot
//#endif
    } else if (ThisAircraft.airborne==1 && airborne<=0) {
      airborne_changed = true;
      // AirborneTime = 0;
    }

    ThisAircraft.airborne = (airborne > 0)? 1 : 0;

    if (airborne_changed) {
      if (settings->nmea_t || settings->nmea2_t)
        sendPFLAJ();
    }

    if (airborne != was_airborne) {
      if ((settings->nmea_d || settings->nmea2_d) && (settings->debug_flags & DEBUG_PROJECTION)) {
        snprintf_P(NMEABuffer, sizeof(NMEABuffer),
          PSTR("$PSTAA,this_airborne: %d, %.1f, %.5f, %.5f, %.0f\r\n"),
            airborne, speed, ThisAircraft.latitude, ThisAircraft.longitude, ThisAircraft.altitude);
        NMEAOutD();
#if defined(ESP32)
#if defined(USE_SD_CARD)
        //int nmealen = strlen(NMEABuffer) - 2;   // overwrite existing \r\n
        //snprintf_P(NMEABuffer+nmealen, sizeof(NMEABuffer)-nmealen, " at %02d:%02d:%02d\r\n",
        //     gnss.time.hour(), gnss.time.minute(), gnss.time.second());
        //Serial.print(NMEABuffer);
        //SD_log(NMEABuffer);
        //snprintf_P(NMEABuffer, sizeof(NMEABuffer),
        //  PSTR("AA,%02d%02d%02d,%d,%.1f,%.5f,%.5f,%.0f\r\n"),
        //    gnss.time.hour(), gnss.time.minute(), gnss.time.second(),
        //    airborne, speed, ThisAircraft.latitude, ThisAircraft.longitude, ThisAircraft.altitude);
        if (SD_is_mounted && (settings->debug_flags & DEBUG_DEEPER)) {
          snprintf_P(NMEABuffer, sizeof(NMEABuffer), PSTR("AA,%d,%.1f\r\n"), airborne, speed);
          FlightLogComment(NMEABuffer);
        }
#endif
#endif
      }
    }

    if (airborne_changed) {
      snprintf_P(NMEABuffer, sizeof(NMEABuffer),
          PSTR("%s: %d/%02d/%02d %02d:%02d %.5f,%.5f\r\n"),
          (airborne? "takeoff" : "landing"),
          gnss.date.year(), gnss.date.month(), gnss.date.day(),
          gnss.time.hour(), gnss.time.minute(),
          ThisAircraft.latitude, ThisAircraft.longitude);
      Serial.print((const char *) NMEABuffer);
      // also output to alarmlog
      if (AlarmLogOpen)
          AlarmLog.print((const char *) NMEABuffer);
      if (airborne <= 0) {
          save_range_stats();
#if defined(ESP32)
          if (settings->rx1090)
              save_zone_stats();  // do this first as it will append comments to IGC file
#endif
          stoplogs();             // close the alarm log and flight log after landing
      }
    }
}


void report_this_projection(container_t *this_aircraft, int proj_type)
{
#if 0
    if (proj_type==1)
      Serial.printf("this_proj: no history  %.1f %.1f %d %d %d %d\r\n",
        this_aircraft->course, this_aircraft->heading,
        this_aircraft->air_ns[0], this_aircraft->air_ew[0],
        this_aircraft->fla_ns[0], this_aircraft->fla_ew[0]);
    else if (proj_type==2)
      Serial.printf("this_proj: not turning %.1f %.1f %d %d %d %d\r\n",
        this_aircraft->course, this_aircraft->heading,
        this_aircraft->air_ns[0], this_aircraft->air_ew[0],
        this_aircraft->fla_ns[0], this_aircraft->fla_ew[0]);
    else if (proj_type==3)
      Serial.printf("this_proj: circling    %.1f %.1f %d %d %d %d %d %d %d %d\r\n",
        this_aircraft->course, this_aircraft->heading,
        this_aircraft->air_ns[0], this_aircraft->air_ew[0],
        this_aircraft->air_ns[1], this_aircraft->air_ew[1],
        this_aircraft->fla_ns[0], this_aircraft->fla_ew[0],
        this_aircraft->fla_ns[1], this_aircraft->fla_ew[1]);
    else if (proj_type==4)
      Serial.printf("this_proj: history     %.1f %.1f %d %d %d %d %d %d %d %d\r\n",
        this_aircraft->course, this_aircraft->heading,
        this_aircraft->air_ns[0], this_aircraft->air_ew[0],
        this_aircraft->air_ns[1], this_aircraft->air_ew[1],
        this_aircraft->fla_ns[0], this_aircraft->fla_ew[0],
        this_aircraft->fla_ns[1], this_aircraft->fla_ew[1]);
    else
      Serial.printf("this_proj: eh?\r\n");
#endif
    if ((settings->nmea_d || settings->nmea2_d) && (settings->debug_flags & DEBUG_PROJECTION)) {
      snprintf_P(NMEABuffer, sizeof(NMEABuffer),
        PSTR("$PSPTA,%d,%d,%.1f,%.1f,%.1f,%.1f,%d,%d,%d,%d,%d\r\n"),
        this_aircraft->airborne,
        proj_type, this_aircraft->course, this_aircraft->heading,
        this_aircraft->prevheading, this_aircraft->turnrate, this_aircraft->circling,
        this_aircraft->air_ns[0], this_aircraft->air_ew[0],
        this_aircraft->air_ns[1], this_aircraft->air_ew[1]);
      NMEAOutD();
#if defined(ESP32)
#if defined(USE_SD_CARD)
      static uint8_t counter;
      ++counter;
      //if ((this_aircraft->circling && (counter & 0x01) == 0) || (counter & 0x07) == 0) {
      if (SD_is_mounted && (counter & 0x01) == 0) {
          // about every 5 (or 20) seconds
          FlightLogComment(NMEABuffer+3);  // LPLTPTA,...
      }
#endif
#endif
    }
}

void report_that_projection(container_t *fop, int proj_type)
{
#if 0
    Serial.printf("that_proj: %d %.0f %.1f %.0f %.1f %d %d %d %d %d %d\r\n",
          proj_type, fop->course, fop->speed, fop->heading, fop->turnrate,
          fop->air_ns[0], fop->air_ew[0],
          fop->air_ns[1], fop->air_ew[1],
          fop->air_ns[2], fop->air_ew[2]);
#endif
    if ((settings->nmea_d || settings->nmea2_d) && (settings->debug_flags & DEBUG_PROJECTION)) {
        snprintf_P(NMEABuffer, sizeof(NMEABuffer),
          PSTR("$PSPOA,%d,%.0f,%.1f,%.0f,%.1f,%d,%d,%d,%d\r\n"),
          proj_type, fop->course, fop->speed, fop->heading, fop->turnrate,
          fop->air_ns[0], fop->air_ew[0],
          fop->air_ns[1], fop->air_ew[1]);
        NMEAOutD();
#if defined(ESP32)
#if defined(USE_SD_CARD)
      static uint8_t counter;
      ++counter;
      if (SD_is_mounted && (counter & 0x07) == (fop->addr & 0x07)) {
          // about every 20 seconds for each other aircraft
          FlightLogComment(NMEABuffer+3);  // LPLTPOA,...
      }
#endif
#endif
    }
}


/*
 * Project the future path of this_aircraft into some future time points.
 */
void project_this(container_t *this_aircraft)
{
    int i, endturn;
    int16_t ns, ew;
    float gspeed, aspeed;
    float gs_ns, gs_ew, as_ns, as_ew;
    float course, heading, gturnrate, aturnrate, finterval;
    uint32_t interval;
    float c, s;
    bool report = false;
    int proj_type = 0;

    /* don't project this aircraft more often than every 600 ms */
    /* (actually the GNSS data is only updated once per second) */
    //static uint32_t time_to_project_course = 0;
    //static uint32_t old_gnsstime = 0;
    uint32_t old_gnsstime = this_aircraft->prevtime_ms;
    uint32_t gnsstime_ms = this_aircraft->gnsstime_ms;
    //if (gnsstime_ms < time_to_project_course)
    //    return;
    /* ignore repeats of same GPS reading */
    if (gnsstime_ms < old_gnsstime + 600)
        return;
    interval = gnsstime_ms - old_gnsstime;
    //old_gnsstime = gnsstime_ms;
    //time_to_project_course = gnsstime_ms + 600;

    static uint32_t time_to_report = 0;
    if (gnsstime_ms > time_to_report) {
        time_to_report = gnsstime_ms + 2300;
        report = true;
    }

    this_airborne(true);      /* determine the "airborne" flag */

    /* first get heading and turn rate */

    /* ground reference speed and course from last GNSS fix */
    gspeed = this_aircraft->speed * _GPS_MPS_PER_KNOT;
    course = this_aircraft->course;
    gs_ns = gspeed * cos(D2R * course);
    gs_ew = gspeed * sin(D2R * course);

    /* compute airspeed and heading from course and speed and wind */
    as_ns = gs_ns - wind_best_ns;
    as_ew = gs_ew - wind_best_ew;
    heading = R2D * atan2(as_ew, as_ns);
    if (heading < 0.0)
        heading += 360.0;
    this_aircraft->heading = heading;

    /* also compute ground-reference turn rate */
    finterval = (0.001f * (float) interval);
    if (interval <= 5500) {
      this_aircraft->projtime_ms = gnsstime_ms - (interval >> 1);
            /* midway between the 2 time points */
      float course_change = (course - this_aircraft->prevcourse);
      /* roll-over through 360 */
      if (course_change > 180.0)
          course_change -= 360.0;
      else if (course_change < -180.0)
          course_change += 360.0;
      gturnrate = course_change / finterval;
      if (fabs(gturnrate) <  2.0)  gturnrate = 0.0;
      if (fabs(gturnrate) > 50.0)  gturnrate = 0.0;
      if (interval < 1400 && this_aircraft->turnrate != 0.0 /* && gturnrate != 0.0 */ ) {
         /* short interval between packets, average with previously known turn rate */
         this_aircraft->turnrate = 0.5 * (gturnrate + this_aircraft->turnrate);
      } else {
         this_aircraft->turnrate = gturnrate;
      }
    } else {   // interval too long
      this_aircraft->projtime_ms = gnsstime_ms;
      //>>> if lost GNSS for a short time while circling, assume still circling?
      //if (interval > 11000)
          this_aircraft->turnrate = 0.0;
    }

    /* If this aircraft is circling then use the average turn rate and speed */
    /* around the circle as measured within the wind estimation function     */
    /* - this differs from the momentary turn rate if there is wind.         */
    /* If haven't yet completed enough circling to get turn rate and speed,  */
    /* then skip this "circling" projection and fall down to "turning".      */
    /* If stopped turning, it takes several seconds for "circling" to cancel */
    /* so check the actual turn rate and if inconsistent then drop through   */

    if (this_aircraft->circling != 0 && avg_abs_turnrate != 0.0 && avg_speed != 0.0
     && ((this_aircraft->circling < 0)? this_aircraft->turnrate < -2.0 : this_aircraft->turnrate > 2.0)) {

      proj_type = 3;
      this_aircraft->projtime_ms = gnsstime_ms;
      aturnrate = ((this_aircraft->circling < 0)? -avg_abs_turnrate : avg_abs_turnrate);
      aspeed = avg_speed;   /* average airspeed measured while circling */

    } else if (interval > 5500) {  // >>> was 3000

      /* if no usable history, assume a straight path */

      this_aircraft->projtime_ms = gnsstime_ms;
      this_aircraft->prevcourse  = this_aircraft->course;
      this_aircraft->prevheading = this_aircraft->heading;
      //>>> if lost GNSS for a short time, assume still turning at same rate?
      //if (interval > 11000)
      //  this_aircraft->turnrate = 0.0;   // already done above

      //aspeed = hypot(as_ns, as_ew);
      //this_aircraft->airspeed = aspeed;
      ns = (int16_t) roundf(4.0 * as_ns);
      ew = (int16_t) roundf(4.0 * as_ew);
      for (i=0; i<6; i++) {
        this_aircraft->air_ns[i] = ns;
        this_aircraft->air_ew[i] = ew;
      }

      if (settings->rf_protocol == RF_PROTOCOL_LEGACY) {    // but not LATEST
        /* also need to compute fla_ns[] & fla_ew[] for transmissions */
        ns = (int16_t) roundf(4.0 * gs_ns);
        ew = (int16_t) roundf(4.0 * gs_ew);
        for (i=0; i<4; i++) {
          this_aircraft->fla_ns[i] = ns;
          this_aircraft->fla_ew[i] = ew;
        }
      }

      if (report) report_this_projection(this_aircraft, 1);

      return;

    } else {  /* this aircraft not (truly) "circling" but has history */

      proj_type = 4;     // will be 2 if not turning

      aspeed = hypot(as_ns, as_ew);

      /* compute turn rate - degrees per second */
      /* uses current and previous course and time_ms */

      /* previous heading from past course and speed and wind */
      float prevheading = this_aircraft->prevheading;

      /* turn rate in the air reference frame (drifting with the wind) */

      float heading_change = heading - prevheading;
      /* roll-over through 360 */
      if (heading_change > 180.0)
          heading_change -= 360.0;
      else if (heading_change < -180.0)
          heading_change += 360.0;
      //interval = gnsstime_ms - this_aircraft->prevtime_ms;
      aturnrate = heading_change / finterval;
      if (fabs(aturnrate) <  2.0)  aturnrate = 0.0;        /* ignore inaccurate data */
      if (fabs(aturnrate) > 50.0)  aturnrate = 0.0;        /* ignore implausible data */
    }

    /* compute NS & EW speed components for future time points */

    if (fabs(aturnrate) < 2.0) {   // turnrate from avg_abs_turnrate or from heading change

      /* treat it as not turning at all */
      ns = (int16_t) roundf(4.0 * as_ns);
      ew = (int16_t) roundf(4.0 * as_ew);
      for (i=0; i<6; i++) {
        this_aircraft->air_ns[i] = ns;
        this_aircraft->air_ew[i] = ew;
      }

      //this_aircraft->turnrate = 0.0;     // over-riding turnrate from other sources

      if (settings->rf_protocol == RF_PROTOCOL_LEGACY) {    // but not LATEST
        /* also need to compute fla_ns[] & fla_ew[] for transmissions */
        ns = (int16_t) roundf(4.0 * gs_ns);
        ew = (int16_t) roundf(4.0 * gs_ew);
        for (i=0; i<4; i++) {
          this_aircraft->fla_ns[i] = ns;
          this_aircraft->fla_ew[i] = ew;
        }
      }

      if (report) report_this_projection(this_aircraft, 2);

      return;

    }
    
    /* turning */

    if (this_aircraft->projtime_ms < gnsstime_ms)
        heading -= aturnrate * 0.001 * (float)(gnsstime_ms - this_aircraft->projtime_ms);

    /* our internal intervals are 3 sec, even though transmissions may use 2 or 4 */

    if (fabs(aturnrate) > 9.0) {
      /* since the projection is in straight segments rather than a circle, */
      /* correct the speed for the polygon shortcut relative to the circumference */
      /* so that the projected trajectory will reach the points at the right time */
      /* factor = 360/PI/turnrate/interval * sin_approx(turnrate*interval/2) */
      /* - 1/2 slice angle in degrees over interval=3seconds is turnrate * 1.5 */
      //float factor = (360.0f/3.0f/3.1416f)/aturnrate * sin((D2R*1.5f) * aturnrate);
      //aspeed *= factor;
      //approximation:
      aspeed *= 1.0f - (D2R*D2R*0.375f) * aturnrate*aturnrate;
    }
    //this_aircraft->airspeed = aspeed;

    float dir_chg = 1.5 * aturnrate;  // first point will be 3 seconds into future
    heading += dir_chg;               // average heading between now and then
    dir_chg *= 2.0;                   // 3-second intervals after that
    if (this_aircraft->circling) {
        // even if proj type = 4
        endturn = 6;
    } else if (fabs(dir_chg) > 15.0) {
        // limit to a 90-degree turn
        //endturn = (int) (90.0f / fabs(dir_chg));
        endturn = (90*256) / (int)(256.0f*fabs(dir_chg));
        if (endturn == 0)  endturn = 1;
    } else {
        endturn = 6;
    }
    for (i=0; i<6; i++) {
       if (i == 0 || i < endturn) {
          if (heading >  360.0)  heading -= 360.0;
          if (heading < -360.0)  heading += 360.0;
          ns = (int16_t) roundf(4.0f * aspeed * cos(D2R * heading));
          ew = (int16_t) roundf(4.0f * aspeed * sin(D2R * heading));
//if (settings->debug_flags & DEBUG_PROJECTION && i==0) {
//snprintf_P(NMEABuffer, sizeof(NMEABuffer)," aspd=%.1f (%d,%d) fctr=%.2f gtr=%.1f atr=%.1f wnd(%.0f,%.0f)\r\n",
//aspeed, ns, ew, factor, gturnrate, aturnrate, wind_best_ns, wind_best_ew);
//FlightLogComment(NMEABuffer);
//}
          heading += dir_chg;
       }  // else stop turning, keep same velocity vector
       this_aircraft->air_ns[i] = ns;
       this_aircraft->air_ew[i] = ew;
    }

    if (settings->rf_protocol != RF_PROTOCOL_LEGACY) {
        if (report) report_this_projection(this_aircraft, proj_type);
        return;
    }

    // if (settings->rf_protocol == RF_PROTOCOL_LEGACY) {   // not needed for RF_PROTOCOL_LATEST

    /* also need to compute fla_ns[] & fla_ew[] for transmissions */

    /* first need to compute initial ground-reference turn rate */
    // compute this (approximately) for now+2s instead of now:
    //float windangle = course + 2.0*turnrate - wind_direction;
    //if (windangle > 0)  windangle -= 180.0;
    //else                windangle += 180.0;     /* angle from DOWNwind */
    //turnrate = turnrate / (1.0 + cos_approx(windangle) * wind_speed / aspeed);
    /* this increases the turnrate when heading upwind, and vice versa */

    // alternative: use turnrate computed above from course and prevcourse

    /* now imitate FLARM and keep speed and turn rate constant while circling */
    /* - not exact imitation, since FLARM sends the momentary ground-reference turn rate */
    float delta_t;
    if (this_aircraft->aircraft_type == AIRCRAFT_TYPE_TOWPLANE)         // known 4-second intervals
        delta_t = 4.0;
    else if (this_aircraft->aircraft_type == AIRCRAFT_TYPE_DROPPLANE)   // unverified assumptions
        delta_t = 4.0;
    else if (this_aircraft->aircraft_type == AIRCRAFT_TYPE_POWERED)
        delta_t = 4.0;
    else if (this_aircraft->circling)                  // typically gliders
        delta_t = 3.0;
    else
        delta_t = 2.0;
    dir_chg = delta_t * this_aircraft->turnrate;    // this is the ground-reference turn rate from course
    if (this_aircraft->circling) {
        endturn = 4;
    } else if (fabs(dir_chg) > 22.5) {
        // >>> limit to a 90-degree turn
        //     - FLARM may or may not want this in the projection?
        //endturn = (int) (90.0f / fabs(dir_chg));
        endturn = (90*256) / (int)(256.0f*fabs(dir_chg));
        if (endturn == 0)  endturn = 1;
    } else {
        endturn = 4;
    }
    for (i=0; i<4; i++) {
      if (i == 0 || i < endturn) {
        // first velocity direction will be "delta_t" seconds into future
        //   - because that is what FLARM seems to send
        course += dir_chg;
        if (course >  360.0)  course -= 360.0;
        if (course < -360.0)  course += 360.0;
        ns = (int16_t) roundf(4.0f * gspeed * cos(D2R * course));
        ew = (int16_t) roundf(4.0f * gspeed * sin(D2R * course));
      } // else stop turning, keep same velocity vector
      this_aircraft->fla_ns[i] = ns;
      this_aircraft->fla_ew[i] = ew;
    }

    //}

    if (report) report_this_projection(this_aircraft, proj_type);
}


/*
 * Project the future path of other aircraft into some future time points.
 */
void project_that(container_t *fop)
{
    int i, endturn;
    float aspeed, gspeed, course, heading, aturnrate;
    float gs_ns, gs_ew, windangle;  // as_ns, as_ew
    float as_ns, as_ew;
    int16_t ns, ew;
    bool report = false;

    static uint32_t time_to_report = 0;
    if (millis() > time_to_report) {
        time_to_report = millis() + 2300;
        report = true;
    }

    fop->projtime_ms = fop->gnsstime_ms;

    if (fop->protocol == RF_PROTOCOL_LEGACY || fop->protocol == RF_PROTOCOL_LATEST) {

      /* Turn rate was obtained in Legacy_decode() */
      /* For vector alarm method don't need more than that */
      //if (settings->alarm != TRAFFIC_ALARM_LATEST)
      //      return;

      /* Incoming Legacy packets provide us with the velocity data.       */
      /* But it is not in a good format, neither air nor ground reference */
      /* The data in the packets ignores the wind!                        */
      /* It is Ground Speed at some point in time (presumably now)        */
      /* and does not change later in the projection (points [1]-[3]).    */
      /* The turn rate is given as constant but likely also ground-ref.   */
      /*            https://pastebin.com/raw/nmi71bTG                     */
      /*            https://pastebin.com/raw/Pi014La8                     */

/*
To compute the correct air-reference circling path:
* Get speed vector from the first ns/ew pair.
* Compute ground-ref course C from atan2() of the first ns/ew pair.
* Compute ground-ref course from atan2() of the second ns/ew pair.
* Compute the (ground reference) turn rate R from the difference.
* Subtract the wind vector from the first pair to get the airspeed vector.
* Rewind it to time "now", or about 1 or 1.5 seconds before the first projection point.
   - that gives the heading.
   - also compute the course "now", by adding the wind back in.
* Compute scalar magnitude of airspeed A and of wind W.
* Compute the angle D of the ground-ref course C relative to the wind vector
   - 0 degrees means directly downwind.
* Compute a correction E to the turn rate:  E = cos(D)*R*W/A
* Add E to the turn rate R to get the air-reference turn rate.
   - Downwind turn rates get bigger (in air reference), upwind smaller.
   - or do it as R *= (1+cos(D)*W/A)
* Use the initial heading and the corrected turn rate to project the 4 future points.
* this is a total of 13 trig calls!
* might as well continue and compute 2 more points? (can use the no-trig method)
*
      float direction0 = atan2_approx((float) fop->fla_ns[0], (float) fop->fla_ew[0]);
      float direction1 = atan2_approx((float) fop->fla_ns[1], (float) fop->fla_ew[1]);
      float dir_chg = direction1 - direction0;
      if (dir_chg >  270.0) dir_chg -= 360.0;
      if (dir_chg < -270.0) dir_chg += 360.0;
      // FLARMs send lat/lon of position 2, 3, or 4 seconds into future
      // compute present ground-reference direction
      float dir_now;
      if (fop->interval == 4)   // AIRCRAFT_TYPE_TOWPLANE, 4-second intervals
          dir_now = direction0 - 0.5 * dir_chg;
      else if (fop->interval == 3)
          dir_now = direction0 - 0.667 * dir_chg;  // 3-second intervals
      else
          dir_now = direction0 - dir_chg;          // 2-second intervals
      gspeed = approxHypotenuse((float) fop->fla_ns[0], (float) fop->fla_ew[0]);
*/
      float dir_now = fop->course;            // already computed in legacy_decode()
      float dir_chg = 3.0 * fop->turnrate;    // internally we use 3-second intervals
      gspeed = fop->speed * (4.0 * _GPS_MPS_PER_KNOT);   // quarter-meters per sec
      gs_ns = gspeed * cos(D2R * dir_now);
      gs_ew = gspeed * sin(D2R * dir_now);     /* present ground-speed vector */
      as_ns = gs_ns - (4.0 * wind_best_ns);
      as_ew = gs_ew - (4.0 * wind_best_ew);    /* present air-speed vector */
      heading = R2D * atan2(as_ew, as_ns);
      if (heading < 0.0)
          heading += 360.0;
      fop->heading = heading;                  // used in $PSALL reporting
      aspeed = hypot(as_ns, as_ew);            // quarter-meters per sec
      windangle = dir_now - wind_direction;
      if (windangle > 0)  windangle -= 180.0;         /* angle from DOWNwind */
      else                windangle += 180.0;
      if (wind_speed > 1.0 && aspeed > 4.0)
          dir_chg *= (1.0 + cos(D2R * windangle) * 4.0 * wind_speed / aspeed);
      /* this makes air-ref turnrate smaller than the ground-ref turnrate
         when heading upwind, and vice versa */

      if (fabs(dir_chg) > 27.0) {
        /* since the projection is in straight segments rather than a circle, */
        /* correct the speed for the polygon shortcut relative to the circumference */
        /* so that the projected trajectory will reach the points at the right time */
        /* factor = 360/PI/turnrate/interval * sin_approx(turnrate*interval/2) */
        /* - 1/2 slice angle in degrees over interval=3seconds is turnrate * 1.5 */
        //float factor = (360.0f/3.1416f)/dir_chg * sin((0.5f*D2R)*dir_chg);
        //aspeed *= factor;
        //approximation:
        aspeed *= 1.0f - (D2R*D2R*0.04167f) * dir_chg*dir_chg;
      }

      /* whew! now can compute air-ref direction for any future time, simple trig: */
      heading += 0.5 * dir_chg;    // 1.5 sec into future
      if (fop->circling) {
          endturn = 6;
      } else if (fabs(dir_chg) > 15.0) {
          // limit to a 90-degree turn
          //endturn = (int) (90.0f / fabs(dir_chg));    // limit to a 90-degree turn
          endturn = (90*256) / (int)(256.0f*fabs(dir_chg));
          if (endturn == 0)  endturn = 1;
      } else {
          endturn = 6;
      }
      for (int i=0; i<6; i++) {
         if (i < endturn) {
             if (heading >  360.0) heading -= 360.0;
             if (heading < -360.0) heading += 360.0;
             ns = (int16_t) roundf(aspeed * cos(D2R * heading));
             ew = (int16_t) roundf(aspeed * sin(D2R * heading));
             heading += dir_chg;
         }  // else stop turning, keep same velocity vector
         fop->air_ns[i] = ns;
         fop->air_ew[i] = ew;
         // also fill in fla[] for air_relay - but simplify: ignore wind & exact timing
         if (i < 4) {
             fop->fla_ns[i] = ns;
             fop->fla_ew[i] = ew;
         }
      }

      if (report) report_that_projection(fop, 1);

      return;
    }

    /* for other protocols (including ADS-B), turn rate was NOT obtained in decode() */
    /* rely on history to compute turn rate */

    /* if no usable history, assume a straight path */

    if (fop->gnsstime_ms - fop->prevtime_ms > 3000) {

      fop->turnrate = 0.0;
      fop->heading = fop->course;   // not exact, but only used in $PSALL reporting

      //if (settings->alarm != TRAFFIC_ALARM_LATEST)  // don't need more than the turn rate
      //    return;

      gspeed = fop->speed * (4.0 * _GPS_MPS_PER_KNOT);   // quarter-meters per sec

      /* compute heading from course and speed and last wind estimate */
      float nsf = gspeed * cos(D2R * fop->course);
      float ewf = gspeed * sin(D2R * fop->course);
      int16_t ns, ew;
      ns = (int16_t) roundf(nsf - (4.0 * wind_best_ns));
      ew = (int16_t) roundf(ewf - (4.0 * wind_best_ew));

      /* project a straight line */
      for (i=0; i<6; i++) {
        fop->air_ns[i] = ns;
        fop->air_ew[i] = ew;
      }

      if (report) report_that_projection(fop, 2);

      // also fill in fla[] for air_relay
      ns = (int16_t) roundf(nsf);
      ew = (int16_t) roundf(ewf);
      for (i=0; i<4; i++) {
        fop->fla_ns[i] = ns;
        fop->fla_ew[i] = ew;
      }

      return;

    }

    /* have history - compute turn rate - degrees per second */
    /* uses current and previous course and time_ms */

    gspeed = fop->speed * (4.0 * _GPS_MPS_PER_KNOT);   // ground speed - quarter-meters per sec
    course = fop->course;

    /* previous heading from past course and speed and wind */
    float prevheading = fop->prevheading;

    /* same for current time point */
    float nsf = gspeed * cos(D2R * fop->course);
    float ewf = gspeed * sin(D2R * fop->course);
    as_ns = nsf - (4.0 * wind_best_ns);
    as_ew = ewf - (4.0 * wind_best_ew);
    heading = R2D * atan2(as_ew, as_ns);
    if (heading < 0.0)
        heading += 360.0;
    //if (heading >  360.0) heading -= 360.0;
    //if (heading < -360.0) heading += 360.0;
    fop->heading = heading;            /* will be carried over into prevheading */
    aspeed = hypot(as_ns, as_ew);      // air speed - quarter-mps

    /* turn rate in the air reference frame (drifting with the wind) */
    float heading_change = heading - prevheading;

    if (fabs(heading_change) > 270.0) {
      /* roll-over through 360 */
      if (heading > 270.0)  heading_change -= 360.0;
      else heading_change += 360.0;
    }
    uint32_t interval = 0;
    if (fop->gnsstime_ms > fop->prevtime_ms) {
        interval = fop->gnsstime_ms - fop->prevtime_ms;
        fop->projtime_ms = fop->gnsstime_ms - (interval >> 1);  /* midway between the 2 time points */
    }
    aturnrate = heading_change / (0.001f * (float) interval);
    if (fabs(aturnrate) > 50.0)  aturnrate = 0.0;        /* ignore implausible data */
    if (fabs(aturnrate) <  2.0)  aturnrate = 0.0;        /* ignore inaccurate data */
    if (interval < 1400 && fop->turnrate != 0) {
       /* short interval between packets, average with previously known turn rate */
       fop->turnrate = 0.5f * (aturnrate + fop->turnrate);
    } else {
       fop->turnrate = aturnrate;   // (stores air ref turnrate in the gnd ref field)
    }

    //if (settings->alarm != TRAFFIC_ALARM_LATEST)  // don't need more than the turn rate
    //      return;

    /*  compute air-reference NS & EW speed components for future time points */

    if (fabs(aturnrate) < 2.0) {   /* hardly turning - treat it as not turning at all */

      ns = (int16_t) as_ns;
      ew = (int16_t) as_ew;
      for (i=0; i<6; i++) {
        fop->air_ns[i] = ns;
        fop->air_ew[i] = ew;
      }

      // also fill in fla[] for air_relay
      ns = (int16_t) roundf(nsf);
      ew = (int16_t) roundf(ewf);        // this is already quarter-meters per sec
      for (i=0; i<4; i++) {
        fop->fla_ns[i] = ns;
        fop->fla_ew[i] = ew;
      }

      if (report) report_that_projection(fop, 3);

      return;
    }

    /* else, if turning */

    //heading -= aturnrate * (0.0005f * (float) interval);
    heading -= 0.5 * heading_change;

    if (fabs(aturnrate) > 6.0) {
      /* since the projection is in straight segments rather than a circle, */
      /* correct the speed for the polygon shortcut relative to the circumference */
      /* so that the projected trajectory will reach the points at the right time */
      /* factor = 360/PI/turnrate/interval * sin_approx(turnrate*interval/2) */
     /* - 1/2 slice angle in degrees over interval=3seconds is turnrate * 1.5 */
      //float factor = (360.0f/3.0f/3.1416f)/aturnrate * sin((D2R*1.5f) * aturnrate);
      //approximation:
      float factor = 1.0f - (D2R*D2R*0.375f) * aturnrate*aturnrate;
    }

    heading += 1.5 * aturnrate;         // first point will be 1.5 seconds into future
    float dir_chg = 3.0 * aturnrate;   // 3-second intervals after that
    endturn = 6;
    if (fabs(dir_chg) > 15.0) {
        // limit to a 90-degree turn
        endturn = (90*256) / (int)(256.0f*fabs(dir_chg));
        if (endturn == 0)  endturn = 1;
    }
    for (i=0; i<6; i++) {
        if (i < endturn) {
            if (heading >  360.0) heading -= 360.0;
            if (heading < -360.0) heading += 360.0;
            ns = (int16_t) roundf(aspeed * cos(D2R * heading));
            ew = (int16_t) roundf(aspeed * sin(D2R * heading));
            heading += dir_chg;
        }
        fop->air_ns[i] = ns;
        fop->air_ew[i] = ew;
        // also fill in fla[] for air_relay - but simplify: ignore wind & exact timing
        if (i < 4) {
            fop->fla_ns[i] = ns;
            fop->fla_ew[i] = ew;
        }
    }

    if (report) report_that_projection(fop, 4);
}


/* use GPS data to estimate this aircraft's climb rate, used if no baro sensor */
float Estimate_Climbrate(void)
{
    float alt_change = ThisAircraft.altitude - ThisAircraft.prevaltitude;
    float interval = 0.001f * (ThisAircraft.gnsstime_ms - ThisAircraft.prevtime_ms);
    float climbrate = alt_change / interval;
    climbrate *= (_GPS_FEET_PER_METER * 60.0f);                /* feet per minute */
    if (fabs(climbrate) > 4000.0)  climbrate = avg_climbrate;  /* ignore implausible data */
    if (fabs(climbrate) <  100.0)  climbrate = 0.0;            /* ignore inaccurate data */
    //avg_climbrate = 0.7f * avg_climbrate + 0.3f * climbrate;
    avg_climbrate = 0.5f * (avg_climbrate + climbrate);    // this is only called once in 4 seconds

    //static uint32_t time_to_report = 0;
    //if (millis() > time_to_report) {
    //    time_to_report = millis() + 6300;
#if 0
        Serial.printf("climbrate fpm: %.0f  %.0f,%.0f,%d,%d\r\n", avg_climbrate,
                 ThisAircraft.altitude, ThisAircraft.prevaltitude, ThisAircraft.gnsstime_ms, ThisAircraft.prevtime_ms);
#endif
        if ((settings->nmea_d || settings->nmea2_d) && (settings->debug_flags & DEBUG_PROJECTION)) {
            snprintf_P(NMEABuffer, sizeof(NMEABuffer),
               PSTR("$PSWCR,%.0f,%.2f,%.1f,%.1f,%d,%d\r\n"), avg_climbrate, climbrate*(1.0f/_GPS_FEET_PER_METER/60.0f),
                 ThisAircraft.altitude, ThisAircraft.prevaltitude, ThisAircraft.gnsstime_ms, ThisAircraft.prevtime_ms);
            NMEAOutD();
        }
    //}

    return avg_climbrate;
}
