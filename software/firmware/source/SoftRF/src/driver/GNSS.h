/*
 * GNSSHelper.h
 * Copyright (C) 2016-2022 Linar Yusupov
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

// this file based on v1.2.

#ifndef GNSSHELPER_H
#define GNSSHELPER_H

#include <TinyGPS++.h>

typedef enum
{
  GNSS_MODULE_NONE,
  GNSS_MODULE_NMEA, /* generic NMEA */
  GNSS_MODULE_U6,   /* Ublox 6 */
  GNSS_MODULE_U7,   /* Ublox 7 */
  GNSS_MODULE_U8,   /* Ublox 8 */
  GNSS_MODULE_U9,   /* Ublox 9 */
  GNSS_MODULE_U10,  /* Ublox 10 */
  GNSS_MODULE_U11,  /* RESERVED */
  GNSS_MODULE_MAV,  /* MAVLink */
  GNSS_MODULE_SONY, /* S7XG */
  GNSS_MODULE_AT65, /* AT6558 */
  GNSS_MODULE_MT33, /* L80 */
  GNSS_MODULE_GOKE  /* Air530 */
} gnss_id_t;

typedef struct gnss_chip_ops_struct {
  gnss_id_t (*probe)();
  bool      (*setup)();
  void      (*loop)();
  void      (*fini)();
  uint16_t  gga_ms;
  uint16_t  rmc_ms;
} gnss_chip_ops_t;

//#define ENABLE_GNSS_STATS
#if defined(ENABLE_GNSS_STATS)
typedef struct gnss_stat_struct {
  unsigned long gga_count;
  unsigned long rmc_count;
  unsigned long gga_time_ms;
  unsigned long rmc_time_ms;
} gnss_stat_t;
#endif /* ENABLE_GNSS_STATS */

#define NMEA_EXP_TIME  3500 /* 3.5 seconds */

bool isValidGNSSFix  (void);
byte GNSS_setup      (void);
void GNSS_loop       (void);
void GNSS_fini       (void);
void GNSSTimeSync    (void);
void PickGNSSFix     (void);
#if !defined(EXCLUDE_EGM96)
void LookupSeparation (float, float);
float EGM96GeoidSeparation();
#endif
bool leap_seconds_valid(void);

extern const gnss_chip_ops_t *gnss_chip;  // added
extern TinyGPSPlus gnss;
extern gnss_id_t gnss_id;
extern volatile unsigned long PPS_TimeMarker;
extern const char *GNSS_name[];
extern int8_t leap_seconds_correction;
extern bool gnss_needs_reset;
extern bool gnss_new_fix;
extern bool gnss_new_time;
extern bool gnss_time_from_rmc;
extern uint32_t latest_Commit_Time;
extern uint32_t GNSSTimeSyncMarker;

#endif /* GNSSHELPER_H */
