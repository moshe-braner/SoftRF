/*
 * TrafficHelper.cpp
 * Copyright (C) 2019-2022 Linar Yusupov
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
#include <math.h>
#define D2R (3.141593f/180.0f)
#define R2D (180.0f/3.141593f)

#include "SoCHelper.h"
#include "TrafficHelper.h"
#include "NMEAHelper.h"
#include "EEPROMHelper.h"
#include "EPDHelper.h"

#include "SkyView.h"

traffic_t ThisAircraft, Container[MAX_TRACKING_OBJECTS], fo, EmptyFO;
traffic_by_dist_t traffic[MAX_TRACKING_OBJECTS];

static unsigned long UpdateTrafficTimeMarker = 0;
static unsigned long Traffic_Voice_TimeMarker = 0;

int max_alarm_level = ALARM_LEVEL_NONE;

void Traffic_Add()
{
    // Traffic_Update(&fo);    // already done in NMEAHelper.cpp

    float fo_distance = fo.distance;

    if (fo_distance > ALARM_ZONE_NONE) {
        return;
    }

    if ( settings->filter == TRAFFIC_FILTER_OFF  ||
        (fo.RelativeVertical > -500 && fo.RelativeVertical <  500) ) {

      int i;

      for (i=0; i < MAX_TRACKING_OBJECTS; i++) {
        traffic_t *cip = &Container[i];
        if (cip->ID == fo.ID) {
          fo.alert = cip->alert;
          fo.alert_level = cip->alert_level;
          if (fo.packet_type == 2) {  // PFLAU
            time_t interval = fo.timestamp - cip->timestamp;
            if (interval <= 3) {
              if (cip->packet_type == 1) {  // previous data was from PFLAA
                if (interval > 0) {  // new data
#if 0
                  // PFLAU following PFLAA, use the track, groundspeed, etc from the previous packet
                  fo.IDType = cip->IDType;
                  fo.Track = cip->Track;
                  fo.GroundSpeed = cip->GroundSpeed;
                  //fo.ClimbRate = cip->ClimbRate;   not available, empty in the PFLAA
                  fo.AcftType = cip->AcftType;
                  *cip = fo;
#else
                  // or, keep the previous data and only change the fields known from the PFLAU
                  // directly from the PFLAU:
                  cip->alarm_level = fo.alarm_level;
                  cip->distance = fo.distance;
                  cip->RelativeVertical = fo.RelativeVertical;
                  cip->RelativeBearing = fo.RelativeBearing;
                  // computed in Traffic_Update():
                  cip->adj_dist = fo.adj_dist;
                  cip->RelativeNorth = fo.RelativeNorth;
                  cip->RelativeEast = fo.RelativeEast;
                  // fields kept: IDType, Track, GroundSpeed, AcftType
#endif
                }  // else (PFLAU & PFLAA from same time) ignore the PFLAU
              } else {
                // compute track from the two distance/bearing points
                // also taking into account that this aircraft has moved too
                //  (very approximate since the time interval units are coarse)
                float our_move = ThisAircraft.GroundSpeed * 0.5 /*MPS_PER_KNOT*/ * interval;
                float x = fo.distance * sin(D2R * (float)fo.RelativeBearing)
                             - cip->distance * sin(D2R * (float)cip->RelativeBearing);
                float y = our_move + fo.distance * cos(D2R * (float)fo.RelativeBearing) 
                             - cip->distance * cos(D2R * (float)cip->RelativeBearing);
                fo.Track = R2D * atan2(x,y) + ThisAircraft.Track;
                *cip = fo;
              }
            } else {
              // if PFLAU with no recent history, track remains unknown
              fo.Track = 0;
              *cip = fo;
            }
          } else {
            // PFLAA - copy all the data from the new packet
            *cip = fo;
          }
          return;
        }
      }

      int max_dist_ndx = 0;
      int min_level_ndx = 0;
      float max_distance = 0;

      for (i=0; i < MAX_TRACKING_OBJECTS; i++) {

        if (! Container[i].ID) {
            Container[i] = fo;            // use an empty slot
            return;
        }

        if (ThisAircraft.timestamp > Container[i].timestamp + ENTRY_EXPIRATION_TIME) {
            Container[i] = fo;            // overwrite expired
            return;
        }

        float distance = Container[i].distance;
        if  (distance > max_distance) {
          max_dist_ndx = i;
          max_distance = distance;
        }
        if (Container[i].alarm_level < Container[min_level_ndx].alarm_level)
            min_level_ndx = i;
      }

      if (fo.alarm_level > Container[min_level_ndx].alarm_level) {
        Container[min_level_ndx] = fo;     // alarming traffic overrides
        return;
      }

      if (fo_distance <  max_distance &&
          fo.alarm_level >= Container[max_dist_ndx].alarm_level) {
        Container[max_dist_ndx] = fo;      // overwrite farthest traffic
        return;
      }
    }
}

/* cos(latitude) is used to convert longitude difference into linear distance. */
/* Once computed, accurate enough through a significant range of latitude. */
float CosLat()
{
  static float cos_lat = 0.7071;
  static float oldlat = 45.0;
  float latitude = ThisAircraft.latitude;
  if (fabs(latitude-oldlat) > 0.3) {
    cos_lat = cos(D2R*latitude);
    oldlat = latitude;
  }
  return cos_lat;
}

void Traffic_Update(traffic_t *fop)
{
  float distance, bearing;

  if (settings->protocol == PROTOCOL_GDL90) {

    /* use an approximation for distance & bearing between 2 points */
    float x, y;
    y = 111300.0 * (fop->latitude - ThisAircraft.latitude);         /* meters */
    x = 111300.0 * (fop->longitude - ThisAircraft.longitude) * CosLat();
    distance = hypot(x, y);      /* meters  */
    bearing = R2D * atan2(x, y);

    fop->RelativeNorth    = y;
    fop->RelativeEast     = x;
    fop->RelativeBearing  = bearing - ThisAircraft.Track;
    fop->RelativeVertical = fop->altitude - ThisAircraft.altitude;
    fop->distance = distance;
    fop->adj_dist = fabs(distance) + VERTICAL_SLOPE * fabs(fop->RelativeVertical);

  } else if (fop->packet_type == 2) {    /* a PFLAU sentence: distance & relative bearing known */

    fop->adj_dist = fabs(fop->distance) + VERTICAL_SLOPE * fabs(fop->RelativeVertical);

    bearing = fop->RelativeBearing + ThisAircraft.Track;
    fop->RelativeNorth = fop->distance * cos(D2R * bearing);
    fop->RelativeEast  = fop->distance * sin(D2R * bearing);

    //fop->Track = -360;   // unknown track, but may be filled in later

  } else {           /* a PFLAA sentence */

    distance = hypot((float) fop->RelativeNorth, (float) fop->RelativeEast);

    fop->distance = distance;
    fop->adj_dist = fabs(distance) + VERTICAL_SLOPE * fabs(fop->RelativeVertical);

    bearing = R2D * atan2((float) fop->RelativeEast, (float) fop->RelativeNorth);
    fop->RelativeBearing = bearing - ThisAircraft.Track;
  }

  if (fop->alarm_level < fop->alert_level)     /* if gone farther then...   */
      fop->alert_level = fop->alarm_level;     /* ...alert if comes nearer again */
}

static void Traffic_Voice_One(traffic_t *fop)
{
    int bearing;
    char message[80];

    if (settings->voice == VOICE_OFF)
        return;

    const char *u_dist, *u_alt;
    float voc_dist;
    int   voc_alt;
    const char *where;
    char how_far[32];
    char elev[32];

    bearing = fop->RelativeBearing;

    if (bearing < 0)    bearing += 360;
    if (bearing > 360)  bearing -= 360;

    int oclock = ((bearing + 15) % 360) / 30;

    switch (oclock)
    {
    case 0:
      where = "ahead";
      break;
    case 1:
      where = "1oclock";
      break;
    case 2:
      where = "2oclock";
      break;
    case 3:
      where = "3oclock";
      break;
    case 4:
      where = "4oclock";
      break;
    case 5:
      where = "5oclock";
      break;
    case 6:
      where = "6oclock";
      break;
    case 7:
      where = "7oclock";
      break;
    case 8:
      where = "8oclock";
      break;
    case 9:
      where = "9oclock";
      break;
    case 10:
      where = "10oclock";
      break;
    case 11:
      where = "11oclock";
      break;
    }

    /* for alarm messages use very short wording */

    if (max_alarm_level > ALARM_LEVEL_NONE) {
        voc_alt = (int) fop->RelativeVertical;
        snprintf(message, sizeof(message), "%s %s %s",
             (fop->alarm_level < ALARM_LEVEL_URGENT ? WARNING_WORD1 : WARNING_WORD3),
             where, (voc_alt > 70 ? "high" : voc_alt < -70 ? "low" : "level"));
        SoC->TTS(message, VOICE_3);  // faster female voice
        return;
    }

    if (settings->voice == VOICE_WARN)  // only voice collision alarms, not advisories
        return;

    /* for traffic advisory messages use longer wording */

    switch (settings->units)
    {
    case UNITS_IMPERIAL:
      u_dist = "miles";   // "nautical miles";
      u_alt  = "feet";
      voc_dist = (fop->distance * _GPS_MILES_PER_METER) /
                  _GPS_MPH_PER_KNOT;
      voc_alt  = abs((int) (fop->RelativeVertical *
                  _GPS_FEET_PER_METER));
      break;
    case UNITS_MIXED:
      u_dist = "kms";
      u_alt  = "feet";
      voc_dist = fop->distance / 1000.0;
      voc_alt  = abs((int) (fop->RelativeVertical *
                  _GPS_FEET_PER_METER));
      break;
    case UNITS_METRIC:
    default:
      u_dist = "kms";
      u_alt  = "metres";
      voc_dist = fop->distance / 1000.0;
      voc_alt  = abs((int) fop->RelativeVertical);
      break;
    }

    if (voc_dist < 1.0) {
      strcpy(how_far, "near");
    } else {
      if (voc_dist > 9.0) {
        voc_dist = 9.0;
      }
      snprintf(how_far, sizeof(how_far), "%u %s", (int) voc_dist, u_dist);
    }

    if (voc_alt < 100) {
      strcpy(elev, "level");
    } else {
      if (voc_alt > 500) {
        voc_alt = 500;
      }

      snprintf(elev, sizeof(elev), "%u hundred %s %s",
        (voc_alt / 100), u_alt,
        fop->RelativeVertical > 0 ? "above" : "below");
    }

    snprintf(message, sizeof(message),
                "%s %s %s %s",           // was "traffic %s distance %s altitude %s"
                ADVISORY_WORD, where, how_far, elev);

    SoC->TTS(message, VOICE_1);  // slower male voice
}


static void Traffic_Voice()
{
  int i=0;
  int ntraffic=0;
  int bearing;
  char message[80];
  int sound_alarm_ndx = -1;
  int sound_advisory_ndx = -1;
  max_alarm_level = ALARM_LEVEL_NONE;
  int sound_alarm_level = ALARM_LEVEL_NONE;

  for (i=0; i < MAX_TRACKING_OBJECTS; i++) {

    if (Container[i].ID && (ThisAircraft.timestamp <= Container[i].timestamp + VOICE_EXPIRATION_TIME)) {

         /* find the maximum alarm level, whether to be alerted or not */
         if (Container[i].alarm_level > max_alarm_level) {
             max_alarm_level = Container[i].alarm_level;
         }

         /* figure out what is the highest alarm level needing a sound alert */
         if (Container[i].alarm_level > sound_alarm_level
                  && Container[i].alarm_level > Container[i].alert_level) {
             sound_alarm_level = Container[i].alarm_level;
             sound_alarm_ndx = i;
         } else if (Container[i].alarm_level == sound_alarm_level
                  && Container[i].alarm_level > Container[i].alert_level) {  // can't be NONE
             if (sound_alarm_ndx < 0)
                 sound_alarm_ndx = i;
             else if (Container[i].adj_dist < Container[sound_alarm_ndx].adj_dist)
                 sound_alarm_ndx = i;
         } else if (Container[i].distance < ALARM_ZONE_CLOSE
                  && (Container[i].alert & TRAFFIC_ALERT_VOICE) == 0) {
             if (sound_advisory_ndx < 0)
                 sound_advisory_ndx = i;
             else if (Container[i].adj_dist < Container[sound_advisory_ndx].adj_dist)
                 sound_advisory_ndx = i;
         }

         // traffic[ntraffic].fop = &Container[i];
         // traffic[ntraffic].distance = Container[i].distance;
         
         ntraffic++;
       }
  }

  if (ntraffic == 0)
      return;

  if (sound_alarm_level > ALARM_LEVEL_NONE) {
      traffic_t *fop = &Container[sound_alarm_ndx];
      Traffic_Voice_One(fop);
      fop->alert_level = sound_alarm_level;
         /* no more alerts for this aircraft at this alarm level */
      fop->timestamp = now();
  }

  if (max_alarm_level > ALARM_LEVEL_NONE)    // do not create distractions
      return;

  /* do issue voice advisories for non-alarm traffic, if no alarms */
  if (sound_advisory_ndx >= 0 && settings->filter < TRAFFIC_FILTER_ALARM) {
      traffic_t *fop = &Container[sound_advisory_ndx];
      Traffic_Voice_One(fop);
      fop->alert |= TRAFFIC_ALERT_VOICE;
          /* no more advisories for this aircraft until it expires or alarms */
      fop->timestamp = now();
  }
}

void Traffic_setup()
{
  UpdateTrafficTimeMarker = millis();
  Traffic_Voice_TimeMarker = millis();
}

void Traffic_loop()
{
  time_t timenow = now();
  if (timenow > ThisAircraft.timestamp + ENTRY_EXPIRATION_TIME) {
      return;    /* data stream broken */
  }

  if (isTimeToUpdateTraffic()) {
    for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {
      traffic_t *fop = &Container[i];
      if (fop->ID) {
        if (timenow > fop->timestamp + ENTRY_EXPIRATION_TIME) {    // 5s
            *fop = EmptyFO;
        } else if (ThisAircraft.timestamp >= fop->timestamp + TRAFFIC_VECTOR_UPDATE_INTERVAL) {  // 2s
            Traffic_Update(fop);
        }
      }
    }
    UpdateTrafficTimeMarker = millis();
  }

  if (isTimeToVoice()) {
    if (settings->voice != VOICE_OFF) {
      Traffic_Voice();
      Traffic_Voice_TimeMarker = millis();
    }
  }
}

void Traffic_ClearExpired()
{
  time_t timenow = now();
  for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {
    if (Container[i].ID &&
        (timenow > Container[i].timestamp + ENTRY_EXPIRATION_TIME)) {
      Container[i] = EmptyFO;
    }
  }
}

int Traffic_Count()
{
  int count = 0;

  for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {
    if (Container[i].ID) {
      count++;
    }
  }

  return count;
}

/* still used by EPD_Draw_Text() */
int traffic_cmp_by_distance(const void *a, const void *b)
{
  traffic_by_dist_t *ta = (traffic_by_dist_t *)a;
  traffic_by_dist_t *tb = (traffic_by_dist_t *)b;

  if (ta->distance >  tb->distance) return  1;
  if (ta->distance <  tb->distance) return -1;
  return  0;
}

/* could have been used by Traffic_Voice() */
#if 0
int traffic_cmp_by_alarm(const void *a, const void *b)
{
  traffic_by_dist_t *ta = (traffic_by_dist_t *)a;
  traffic_by_dist_t *tb = (traffic_by_dist_t *)b;

  if (ta->fop->alarm_level >  tb->fop->alarm_level) return -1;   // sort descending
  if (ta->fop->alarm_level <  tb->fop->alarm_level) return  1;
  /* if same alarm level, then decide by distance (adjusted for altitude difference) */
  if (ta->fop->adj_dist >  tb->fop->adj_dist) return  1;
  if (ta->fop->adj_dist <  tb->fop->adj_dist) return -1;
  return  0;
}
#endif
