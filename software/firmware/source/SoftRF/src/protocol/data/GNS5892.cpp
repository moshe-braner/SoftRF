/*
 * gns5892.cpp
 *
 * Copyright (C) 2024 Moshe Braner
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version - see <http://www.gnu.org/licenses/>.
 */

#if defined(ESP32)

#include <math.h>
#include <protocol.h>
#include "../../../SoftRF.h"
#include "../../system/SoC.h"
#include "../../system/Time.h"
#include "../../driver/Settings.h"
#include "../../driver/Baro.h"
#include "../../driver/GNSS.h"
#include "../../driver/Filesys.h"
#include "../../TrafficHelper.h"
#include "../radio/Legacy.h"
#include "GNS5892.h"
#include "NMEA.h"
#include "IGC.h"

static adsfo_t fo1090;  // EmptyFO1090;
static char buf1090[256];
static unsigned char msg[14];

static void EmptyFO1090(adsfo_t *p) { memset(p, 0, sizeof(ADSBFO)); }

typedef struct mmstruct {
    // variables filled in by message parsing:
    uint32_t cprlat;    // 17-bit relative representation
    uint32_t cprlon;
    int16_t  ewv;
    int16_t  nsv;
    int16_t  airspeed;
    uint8_t  alt_type;
    uint8_t  rssi;
    uint8_t  frame;     // DF
    uint8_t  type;      // TC
    uint8_t  sub;       // subtype
    uint8_t  fflag;     // odd/even
} mm_t;
static mm_t mm;  // EmptyMsg;

static void EmptyMsg(mm_t *p) { memset(p, 0, sizeof(mmstruct)); }

static uint8_t ac_type_table[16] =
{
    AIRCRAFT_TYPE_UFO,        // 0x0 unknown   // AIRCRAFT_TYPE_UKNOWN is handled as "landed out"
    AIRCRAFT_TYPE_GLIDER,     // 0x1 glider
    AIRCRAFT_TYPE_BALLOON,    // 0x2 LTA 
    AIRCRAFT_TYPE_PARACHUTE,  // 0x3 parachute 
    AIRCRAFT_TYPE_HANGGLIDER, // 0x4 hang glider
    AIRCRAFT_TYPE_UFO,        // 0x5 reserved
    AIRCRAFT_TYPE_UAV,        // 0x6 UAV
    AIRCRAFT_TYPE_UFO,        // 0x7 spacecraft
    AIRCRAFT_TYPE_UFO,        // 0x8 not used
    AIRCRAFT_TYPE_POWERED,    // 0x9 light 
    AIRCRAFT_TYPE_JET,        // 0xA med1
    AIRCRAFT_TYPE_JET,        // 0xB med2
    AIRCRAFT_TYPE_JET,        // 0xC high vortex
    AIRCRAFT_TYPE_JET,        // 0xD heavy
    AIRCRAFT_TYPE_JET,        // 0xE high perf
    AIRCRAFT_TYPE_HELICOPTER  // 0xF rotorcraft
};

uint32_t adsb_packets_counter = 0;

// data structures for collecting statistics on RSSI vs. distance

#define ZONESTATSVERSION 1
#define MINRSSI 24
#define MAXRSSI 43
// GNS5892 may report as high as 45, here we fold 44-45 into 43
#define MINSAMPLE 100
#define MAXSAMPLE 10000

typedef struct zonestruct {
    uint32_t ignore;
    uint32_t report;
    uint32_t alarm1;
    uint32_t alarm2;
} zonestats_t;
static zonestats_t zone_stats[1+MAXRSSI-MINRSSI];
static int32_t stats_count = 0;

// RSSI threshold defaults:  (adjusted down by value of settings->mode_s)
static uint8_t close_rssi  = 33;    // at 33-35 still do not report
static uint8_t report_rssi = 36;    // at 36-37 report, but no alarm
static uint8_t alarm1_rssi = 38;    // at 38-40 give alarm level "low"
static uint8_t alarm2_rssi = 41;    // at 41+ give alarm level "important"

static void zero_stats()
{
    SPIFFS.remove("/rssidist.txt");
    for (int rssi=MINRSSI; rssi <= MAXRSSI; rssi++)
       zone_stats[rssi-MINRSSI] = {0};
}

// try and load RSSI stats from file
static bool load_zone_stats()
{
    if (! SPIFFS.exists("/rssidist.txt")) {
        Serial.println("rssidist.txt does not exist in SPIFFS");
        return false;
    }
    File statsfile = SPIFFS.open("/rssidist.txt", FILE_READ);
    if (! statsfile)
        return false;
    char buf[64];
    if (! getline(statsfile, buf, 64)) {
        statsfile.close();
        return false;
    }
    int file_version;
    sscanf(buf, "%d", &file_version);
    if (file_version != ZONESTATSVERSION) {
        Serial.println("wrong version of rssidist.txt");
        statsfile.close();
        return false;
    }
    // read rest of file into zone_stats[]
    Serial.println("reading rssidist.txt...");
    for (int rssi=MINRSSI; rssi <= MAXRSSI; rssi++) {
        if (! getline(statsfile, buf, 64)) {
            Serial.println("rssidist.txt ended early");
            statsfile.close();
            zero_stats();
            return false;
        }
        //Serial.println(buf);
        int index = rssi - MINRSSI;
        int file_rssi;
        sscanf(buf, "%d,%d,%d,%d,%d",
            &file_rssi,
            &zone_stats[index].ignore,
            &zone_stats[index].report,
            &zone_stats[index].alarm1,
            &zone_stats[index].alarm2);
        snprintf(buf, 64, "%d,%d,%d,%d,%d",
            rssi,
            zone_stats[index].ignore,
            zone_stats[index].report,
            zone_stats[index].alarm1,
            zone_stats[index].alarm2);
        Serial.println(buf);
        if (file_rssi != rssi) {
            Serial.println("rssidist.txt rssi values wrong");
            statsfile.close();
            zero_stats();
            return false;
        }
    }
    statsfile.close();
    return true;
}

static void set_zone_thresholds(bool force)
{
    uint32_t closer  = 0;
    uint32_t farther = 0;
    for (int rssi=MINRSSI; rssi <= MAXRSSI; rssi++) {
        int index = rssi - MINRSSI;
        farther += zone_stats[index].ignore + zone_stats[index].report;
        closer  += zone_stats[index].alarm1 + zone_stats[index].alarm2;
    }
    if (closer < MINSAMPLE && (! force)) {
        // use the defaults (but continue to collect data)
        // adjust the defaults based on value of settings->mode_s:
        uint8_t adj = (uint8_t) settings->mode_s;
        //if (adj > 9)  adj = 9;
        close_rssi  -= adj;
        report_rssi -= adj;
        alarm1_rssi -= adj;
        alarm2_rssi -= adj;
        Serial.print("Using default Mode-S RSSI thresholds with 'gain'=");
        Serial.println(adj);
        return;
    }
    // - continue if accumulated at least 100 samples in under-1000m range
    if (closer > MAXSAMPLE || farther > 0x08000000)
        stats_count = -1;    // do not collect any more data
    for (int rssi=MINRSSI; rssi <= MAXRSSI; rssi++) {
        int index = rssi - MINRSSI;
#if 0
        uint32_t middle = zone_stats[index].ignore
                        + zone_stats[index].report
                        + zone_stats[index].alarm1
                        + zone_stats[index].alarm2;
        middle >>= 1;   // half the samples
        // Note: the sample is grossly biased since the larger distances cover
        // much more area, so may want to modify the definition of "median" below.
        // The threshold of 5000m in sample_rssi_zone() helps.
        uint32_t above = zone_stats[index].ignore;
        if (above > middle)
            // median distance is > 2*ALARM_ZONE_CLOSE, do not report
            report_rssi = rssi+1;
        above += zone_stats[index].report;
        if (above > middle)
            // median distance is > ALARM_ZONE_LOW, no alarm
            alarm1_rssi = rssi+1;
        above += zone_stats[index].alarm1;
        if (above > middle)
            // median distance is > ALARM_ZONE_IMPORTANT, alarm level not IMPORTANT
            alarm2_rssi = rssi+1;
#else
        // equivalent, perhaps more readable:
        uint32_t above  = zone_stats[index].ignore;
        uint32_t report = zone_stats[index].report;
        uint32_t alarm1 = zone_stats[index].alarm1;
        uint32_t below  = report + alarm1 + zone_stats[index].alarm2;
        if (above > below)
            // median distance is > 2*ALARM_ZONE_CLOSE, do not report
            report_rssi = rssi+1;
        above += report;
        below -= report;
        if (above > below)
            // median distance is > ALARM_ZONE_LOW, no alarm
            alarm1_rssi = rssi+1;
        above += alarm1;
        below -= alarm1;
        if (above > below)
            // median distance is > ALARM_ZONE_IMPORTANT, alarm level not IMPORTANT
            alarm2_rssi = rssi+1;
#endif
    }
    close_rssi = report_rssi - 3;   // mark as 3000m distance, but not reported in PFLAA

    Serial.print("Using auto-calibrated Mode-S RSSI thresholds: ");
    Serial.print(close_rssi);
    Serial.print(", ");
    Serial.print(report_rssi);
    Serial.print(", ");
    Serial.print(alarm1_rssi);
    Serial.print(", ");
    Serial.println(alarm2_rssi);
}

static void sample_rssi_zone(uint8_t rssi, float distance, float alt_diff)
{
    if (distance > 6000)  return;         // only sample smaller distances
    if (rssi < MINRSSI)   return;
    if (stats_count < 0)  return;         // do not collect any more data
    //uint32_t distance_3d = iapproxHypotenuse0((int32_t) distance, (int32_t) alt_diff);
    uint32_t distance_3d = (uint32_t) hypot(distance, alt_diff);
    //if (distance_3d > 6000)  return;     // only sample smaller distances
    if (rssi > MAXRSSI)  rssi = MAXRSSI;   // fold higher RSSIs into this top value
    int index = rssi - MINRSSI;
    if (distance_3d > 2*ALARM_ZONE_CLOSE)        zone_stats[index].ignore++;     // 3000m
    else if (distance_3d > ALARM_ZONE_LOW)       zone_stats[index].report++;     // 1000
    else if (distance_3d > ALARM_ZONE_IMPORTANT) zone_stats[index].alarm1++;     //  700
    else                                         zone_stats[index].alarm2++;
    ++stats_count;
}

// this is called after landing
void save_zone_stats()
{
    if (stats_count <= 0) {
        Serial.println("no update to rssidist.txt");
        return;
    }
    SPIFFS.remove("/oldrssi.txt");
    SPIFFS.rename("/rssidist.txt", "/oldrssi.txt");
    File statsfile = SPIFFS.open("/rssidist.txt", FILE_WRITE);
    Serial.print(stats_count);
    Serial.println("new samples, writing rssidist.txt...");
    statsfile.println((int)ZONESTATSVERSION);
    char buf[64];
    for (int rssi=MINRSSI; rssi <= MAXRSSI; rssi++) {
        int index = rssi - MINRSSI;
        snprintf(buf, 64, "RD,%d,%d,%d,%d,%d",
            rssi,
            zone_stats[index].ignore,
            zone_stats[index].report,
            zone_stats[index].alarm1,
            zone_stats[index].alarm2);
        //Serial.print("...");
        Serial.println(buf+3);     // skip the "RD,"
        statsfile.println(buf+3);
        FlightLogComment(buf);
        // - it will prepend LPLT, resulting in, e.g., LPLTRD,31,1234,321,45,7
    }
    statsfile.close();
}

// print the stats at any time
void show_zone_stats()
{
    Serial.println("RSSI zone stats:");
    char buf[64];
    for (int rssi=MINRSSI; rssi <= MAXRSSI; rssi++) {
        int index = rssi - MINRSSI;
        snprintf(buf, 64, "%d,%d,%d,%d,%d",
            rssi,
            zone_stats[index].ignore,
            zone_stats[index].report,
            zone_stats[index].alarm1,
            zone_stats[index].alarm2);
        //Serial.print("...");
        Serial.println(buf);
    }
    // for testing, save to SPIFFS file,
    // and compute thresholds too (even if sample is small)
    save_zone_stats();
    set_zone_thresholds(true);
}

// truncated CRC table, for 56 bit messages only, and skipped the CRC bits, leaving 32
static uint32_t mode_s_checksum_table[] = {
  0x018567, 0xff38b7, 0x80665f, 0xbfc92b, 0xa01e91, 0xaff54c, 0x57faa6, 0x2bfd53,
  0xea04ad, 0x8af852, 0x457c29, 0xdd4410, 0x6ea208, 0x375104, 0x1ba882, 0x0dd441,
  0xf91024, 0x7c8812, 0x3e4409, 0xe0d800, 0x706c00, 0x383600, 0x1c1b00, 0x0e0d80,
  0x0706c0, 0x038360, 0x01c1b0, 0x00e0d8, 0x00706c, 0x003836, 0x001c1b, 0xfff409
};

static uint32_t mode_s_checksum( int n ) {
  int bits = n * 8;
  if (bits != 56) {
//Serial.println("no CRC - bits!=56");
      return 0;
  }
  uint32_t crc = 0;
  int j;
  bits -= 24;   // skip the CRC bits
  int byte = 0;
  int bitmask = (1 << 7);
  for(j = 0; j < bits; /*j++*/) {
    if (msg[byte] & bitmask)
      crc ^= mode_s_checksum_table[j];
    j++;
    if ((j & 0x7) == 0) {
      byte++;
      bitmask = (1 << 7);
    } else {
      bitmask >>= 1;
    }
  }
  return crc;
}

uint32_t addr_from_crc( int n )
{
  static uint32_t last_crc = 0;
  static uint32_t last_val = 0;
  // CRC is always the last three bytes.
  uint32_t crc = (((uint32_t)msg[n-3]) << 16) |
                  (((uint32_t)msg[n-2]) << 8) |
                  (uint32_t)msg[n-1];
  // some aircraft send bursts of identical (?) messages
  if (crc == last_crc)          // very likely same aircraft at same altitude
      return last_val;          // save CPU cycles, return the same ICAO ID
  last_crc = crc;
  uint32_t crc2 = mode_s_checksum(n);
  if (crc2 != 0) {
      last_val = (crc ^ crc2);   // should result overlaid ICAO ID
      return last_val;
  }
  return 0;
}

//
// Compact Position Reporting decoding
//
// Adapted from dump1090, a Mode S message decoder for RTLSDR devices.
// Original code copyright (C) 2012 by Salvatore Sanfilippo <antirez@gmail.com>
// Heavily modified for efficiency - Copyright (C) 2024 by Moshe Braner <moshe.braner@gmail.com>

// The NL function uses the precomputed table from 1090-WP-9-14
static float NLtable[61] = {
 90.0,        // 0
 90.0,        // 1
 87.00000000, // 2
 86.53536998, // 3
 85.75541621, // 4
 84.89166191, // 5
 83.99173563, // 6
 83.07199445, // 7
 82.13956981, // 8
 81.19801349, // 9
 80.24923213, // 10
 79.29428225, // 11
 78.33374083, // 12
 77.36789461, // 13
 76.39684391, // 14
 75.42056257, // 15
 74.43893416, // 16
 73.45177442, // 17
 72.45884545, // 18
 71.45986473, // 19
 70.45451075, // 20
 69.44242631, // 21
 68.42322022, // 22
 67.39646774, // 23
 66.36171008, // 24
 65.31845310, // 25
 64.26616523, // 26
 63.20427479, // 27
 62.13216659, // 28
 61.04917774, // 29
 59.95459277, // 30
 58.84763776, // 31
 57.72747354, // 32
 56.59318756, // 33
 55.44378444, // 34
 54.27817472, // 35
 53.09516153, // 36
 51.89342469, // 37
 50.67150166, // 38
 49.42776439, // 39
 48.16039128, // 40
 46.86733252, // 41
 45.54626723, // 42
 44.19454951, // 43
 42.80914012, // 44
 41.38651832, // 45
 39.92256684, // 46
 38.41241892, // 47
 36.85025108, // 48
 35.22899598, // 49
 33.53993436, // 50
 31.77209708, // 51
 29.91135686, // 52
 27.93898710, // 53
 25.82924707, // 54
 23.54504487, // 55
 21.02939493, // 56
 18.18626357, // 57
 14.82817437, // 58
 10.47047130, // 59
           0  // 60
};

// this function is recursive - bi-section search
static int cprNLFunction_(float lat, int start, int end) {
    // we already know that NL >= start and NL <= end
    if (end <= start+1) {
        if (lat < NLtable[end])
            return end;
        return start;
    }
    int mid = ((start+end) >> 1);
    if (lat < NLtable[mid])
        return cprNLFunction_(lat, mid, end);
    return cprNLFunction_(lat, start, mid-1);
}

static int cprNLFunction(float lat) {
    if (lat < 0) lat = -lat; // Table is symmetric about the equator
    return cprNLFunction_(lat, 1, 59);
}

// store these numbers to avoid having to repeatdely do the divisions 360/NL & NL/360
static float dLonTable[60];
static float dLonInvTable[60];
// - if we are *really* short on memory space, can wait for GNSS fix first,
//   and then only set the tables up for latitudes near us

// this version uses a pre-computed NL
static float cprDlonFunction(int fflag, int NL) {
    if (NL > 1 && fflag)  NL--;
    return dLonTable[NL];
}
static float cprDlonInvFunction(int fflag, int NL) {
    if (NL > 1 && fflag)  NL--;
    return dLonInvTable[NL];
}

static float reflat=0;
static float reflon=0;

// variables precomputed for decoding of CPR lat/lon, based on our own location
static int NL[2];
static float dLat[2], flrlat[2], modlat[2], dLon[2], flrlon[2], modlon[2];
static uint32_t ourcprlat[2], ourcprlon[2];
static float dLatHalf, dLonHalf;
static int32_t maxcprdiff, maxcprdiff_sq;
static float maxdistance1090, maxaltdiff1090;

// similar values precomputed for adjacent NL zones
static int32_t cprMinuslat[2], cprNL0lat[2], cprNL1lat[2], cprPluslat[2];
static float dLonPlus[2], flrlonPlus[2], modlonPlus[2];
static float dLonMinus[2], flrlonMinus[2], modlonMinus[2];
static uint32_t ourcprlonPlus[2], ourcprlonMinus[2];

// This algorithm comes from:
// 1090-WP29-07-Draft_CPR101 (which also defines decodeCPR() )
//
// Despite what the earlier comment here said, we should *not* be using trunc().
// See Figure 5-5 / 5-6 and note that floor is applied to (0.5 + fRP - fEP), not
// directly to (fRP - fEP). Eq 38 is correct.
//
static bool decodeCPRrelative()
{
    if (reflat == 0)
        return false;                     // pre-comp has not happened

    // convert incoming cprlxx values to the "fractions" (how far into current zone)
    float fractional_lat = mm.cprlat * 7.629394531e-6;  // = 1/131072 = 2^-17
    float fractional_lon = mm.cprlon * 7.629394531e-6;

    float j, m, rlat, rlon;   // will receive decoded position of target

    // Compute the Latitude Index "j", using the odd/even fflag of the incoming message
    j = flrlat[mm.fflag] + floor(0.5 + modlat[mm.fflag] - fractional_lat);
    // latitude is zone border + the fractional part
    rlat = dLat[mm.fflag] * (j + fractional_lat);
    if (rlat >= 270) rlat -= 360;

    // Check to see that answer is reasonable - i.e. no more than 1/2 zone away
    float degsdiff = fabs(rlat - reflat);
    if (degsdiff > dLatHalf) {
      if (degsdiff > 0.5 * dLat[mm.fflag])   // more precise test
        return false;                       // Time to give up - Latitude error
    }

    // 'NL' is the number of logitude zones for the target's latitude.
    // NL[] was pre-computed based on reflat (our location), not rlat (target's).
    // Check whether the pre-computed NL is correct - likely, if target is close.
    //   - correct if: (fabs(rlat) < NLtable[NL] && fabs(rlat) >= NLtable[NL+1])
    float dLon2, scaled, flrlon2, modlon2;
    float absrlat = fabs(rlat);         // the coding of rlon depends on rlat!
    bool gt0 = (absrlat >= NLtable[NL[mm.fflag]]);
    bool lt1 = (absrlat < NLtable[NL[mm.fflag]+1]);
    if (gt0 || lt1) {
        // NL is incorrect for the target, need to recompute dLon etc.
        // First check whether the correct NL is the current one +-1,
        // which is likely if the target is close to the ref location.
        // For these adjacent NLs we've also pre-computed things.
        if (lt1 && absrlat >= NLtable[NL[mm.fflag]+2]) {
            //NL2 = NL[mm.fflag] + 1;
            dLon2 = dLonPlus[mm.fflag];
            flrlon2 = flrlonPlus[mm.fflag];
            modlon2 = modlonPlus[mm.fflag];
        } else if (gt0 && absrlat < NLtable[NL[mm.fflag]-1]) {
            //NL2 = NL[mm.fflag] - 1;
            dLon2 = dLonMinus[mm.fflag];
            flrlon2 = flrlonMinus[mm.fflag];
            modlon2 = modlonMinus[mm.fflag];
        } else {
            // Shift into non-adjacent zone.  This can be < 100 miles away at lat>45.
            // No choice but to do the full NL search and recompute.
            int NL2 = cprNLFunction_(absrlat, 1, 59);     // = cprNLFunction(rlat)
            dLon2 = cprDlonFunction(mm.fflag, NL2);
            scaled = reflon * cprDlonInvFunction(mm.fflag, NL2);
            flrlon2 = floor(scaled);
            modlon2 = scaled - flrlon2;
        }
    } else {     // pre-computed NL is OK
            dLon2 = dLon[mm.fflag];
            flrlon2 = flrlon[mm.fflag];
            modlon2 = modlon[mm.fflag];
    }

    // Compute the Longitude Index "m"
    m = flrlon2 + floor(0.5 + modlon2 - fractional_lon);
    // longitude is zone border + the fractional part (whew!)
    rlon = dLon2 * (m + fractional_lon);
    if (rlon > 180) rlon -= 360;

    // Check to see that answer is reasonable - i.e. no more than 1/2 zone away
    degsdiff = fabs(rlon - reflon);
    if (degsdiff > dLonHalf) {
      if (degsdiff > 0.5 * dLon2)    // more precise test
        return false;               // Time to give up - Longitude error
    }

    fo1090.latitude  = rlat;
    fo1090.longitude = rlon;
    return true;
}


static void CPRRelative_precomp()
{
    // do this every minute or several, using own-ship GNSS position for reflat/reflon

    // pre-compute all that is possible just based on reference lat/lon:
    // (two each: odd and even versions)

    // float dLat[2];     // degrees interval one zone covers - constant
    // float flrlat[2];   // floor(reflat / dLat) - i.e., zone index (integer value)
    // float modlat[2];   // mod(reflat,dLat)/dLat = reflat/dLat - flrlat[] - i.e., fraction
                          // - because mod(x,y) is defined as: x - y * floor(x/y)
    // uint32_t cprlat    // the "fraction" times 2^17 - this is what is transmitted
    // int NL[2];         // number of longitude zones: fewer at higher latitudes
    // float dLon[2];     // size of degrees interval one zone covers
    // float flrlon[2];   // floor(reflon / dLon) - i.e., zone index
    // float modlon[2];   // mod(reflon,dLon)/dLon = reflon/dLon - flrlon[] - i.e., fraction
    // uint32_t cprlon    // the "fraction" times 2^17

    if (! GNSSTimeMarker)        // wait until 30 sec after GNSS fix
        return;

    // The exact reference location is not important, just not too far away
    // - except that early filtering of "too far" targets is based on it, not on our actual location
    if (fabs(ThisAircraft.latitude  - reflat) < 0.15
    &&  fabs(ThisAircraft.longitude - reflon) < 0.21)    // about 9 nm
        return;

    reflat = ThisAircraft.latitude;
    reflon = ThisAircraft.longitude;

    for (int k=0; k<2; k++) {  // odd/even
        float invdLat = (k ? 59.0/360.0 : 60.0/360.0);
        // float scaled = reflat / dLat[k];
        float scaled = reflat * invdLat;
        flrlat[k] = floor(scaled);
        modlat[k] = scaled - flrlat[k];
        float lat0 = dLat[k] * flrlat[k];
        ourcprlat[k] = (uint32_t) ((reflat - lat0)/dLat[k] * (float)(1<<17) + 0.5);
        NL[k] = cprNLFunction(reflat);

        // need to compute NL based on target lat which is not known yet -
        // but when target is close NL is the same, so precompute on speculation
        // - see above in decodeCPRrelative() how this is used
        dLon[k] = cprDlonFunction(k, NL[k]);
        scaled = reflon * cprDlonInvFunction(k, NL[k]);
        flrlon[k] = floor(scaled);
        modlon[k] = scaled - flrlon[k];
        ourcprlon[k] = (uint32_t) ((reflon - dLon[k]*flrlon[k])/dLon[k] * (float)(1<<17) + 0.5);

        // pre-compute cpr values for latitudes at both edges of adjacent NL zones
        // - to allow parse() to detect the zone and compute the distance early
        //     - latitude is lower for higher NL
        //     - our lat is < NLtable[NL[k]], and >= NLtable[NL[k]+1]
        // note these out-of-bounds cpr values are signed!
        float edgelat = NLtable[NL[k]-1];
        if (reflat < 0)  edgelat = -edgelat;
        cprMinuslat[k] = (int32_t)((edgelat-lat0) / dLat[k] * (float)(1<<17) + 0.5);
        edgelat = NLtable[NL[k]];
        if (reflat < 0)  edgelat = -edgelat;
        cprNL0lat[k] = (int32_t)((edgelat-lat0) / dLat[k] * (float)(1<<17) + 0.5);
        edgelat = NLtable[NL[k]+1];
        if (reflat < 0)  edgelat = -edgelat;
        cprNL1lat[k] = (int32_t)((edgelat-lat0) / dLat[k] * (float)(1<<17) + 0.5);
        edgelat = NLtable[NL[k]+2];
        if (reflat < 0)  edgelat = -edgelat;
        cprPluslat[k] = (int32_t)((edgelat-lat0) / dLat[k] * (float)(1<<17) + 0.5);

        // pre-compute some other values for adjacent NL zones

        int NL2 = NL[k] + 1;
        dLonPlus[k] = cprDlonFunction(k, NL2);
        dLonHalf = 0.5 * dLonPlus[0];      // for both odd and even, this value is conservative
        scaled = reflon * cprDlonInvFunction(k, NL2);
        flrlonPlus[k] = floor(scaled);
        modlonPlus[k] = scaled - flrlonPlus[k];
        ourcprlonPlus[k] = (uint32_t) ((reflon - dLonPlus[k]*flrlonPlus[k])/dLonPlus[k] * (float)(1<<17) + 0.5);

        NL2 = NL[k] - 1;
        dLonMinus[k] = cprDlonFunction(k, NL2);
        scaled = reflon * cprDlonInvFunction(k, NL2);
        flrlonMinus[k] = floor(scaled);
        modlonMinus[k] = scaled - flrlonMinus[k];
        ourcprlonMinus[k] = (uint32_t) ((reflon - dLonMinus[k]*flrlonMinus[k])/dLonMinus[k] * (float)(1<<17) + 0.5);

        yield();
    }
}

static void CPRRelative_setup()
{
    // prepare lookup tables
    for (int i=1; i<60; i++) {    // yes we skip [0] which is not used
        dLonTable[i] = 360.0/i;
        dLonInvTable[i] = i/360.0;
    }

    // these do not change
    dLat[0] = 360.0/60.0;
    dLat[1] = 360.0/59.0;
    // for both odd and even, this value is conservative:
    dLatHalf = 0.5 * dLat[0];
    // first-cut range limit (along each axis):
    if (settings->hrange1090 > 0 && settings->hrange1090 < 84)
      maxcprdiff = 200*(16+settings->hrange1090);   // 9nm pre-computation threshold + range
    else
      maxcprdiff = 20000;  // 100km
    // a squared scaled version for 2D distance
    maxcprdiff_sq = (maxcprdiff >> 4) * (maxcprdiff >> 4);
    maxdistance1090 = 1000 * settings->hrange1090;
    maxaltdiff1090  =  100 * settings->vrange1090;

    // compute what does depend on reflat, reflon
    // - put this off until there is a GNSS fix
    //CPRRelative_precomp();
}


// the code here repeats some things that are done in Traffic.cpp Addtraffic(),
// would be better not to repeat, but here disjoint groups of fields are updated
// via 3 different types of messages, complicating things.

static int find_traffic_by_addr(uint32_t addr)
{
    for (int i=0; i<MAX_TRACKING_OBJECTS; i++) {
        if (Container[i].addr == addr) {
            if (Container[i].protocol == RF_PROTOCOL_ADSB_1090)
                return i;      // found
            if (Container[i].relayed == 0
                && OurTime <= Container[i].timestamp + ENTRY_EXPIRATION_TIME)
                return -1;     // already tracked via other means
            // was tracked via other means, but expired - clear this slot
            Container[i].addr = 0;
            return MAX_TRACKING_OBJECTS;
        }
    }
    return MAX_TRACKING_OBJECTS;    // not found
}

// make room for a new entry
static int add_traffic_by_dist(float alt_diff)
{
    int i;
    // replace an empty object if found
    for (i=0; i < MAX_TRACKING_OBJECTS; i++) {
      if (Container[i].addr == 0) {
        return i;
      }
    }
    // replace an expired object if found
    for (i=0; i < MAX_TRACKING_OBJECTS; i++) {
      if (OurTime > Container[i].timestamp + ENTRY_EXPIRATION_TIME) {
        return i;
      }
    }
    /* identify the farthest-away non-"followed" object */
    /*    (distance adjusted for altitude difference)   */
    uint32_t follow_id = settings->follow_id;
    int max_dist_ndx = MAX_TRACKING_OBJECTS;
    float max_dist = 0;
    float adj_distance;
    for (i=0; i < MAX_TRACKING_OBJECTS; i++) {
      container_t *cip = &Container[i];
      if (cip->alarm_level == ALARM_LEVEL_NONE
          && cip->addr != follow_id && cip->relayed == false) {
        adj_distance = cip->adj_distance;
        if (adj_distance < cip->distance)
            adj_distance = cip->distance;
        if (adj_distance > max_dist)  {
          max_dist_ndx = i;
          max_dist = adj_distance;
        }
      }
    }
    if (max_dist_ndx < MAX_TRACKING_OBJECTS) {
        // may replace the farthest object
        adj_distance = fo1090.distance + VERTICAL_SLOPE * fabs(alt_diff);
        if (adj_distance < max_dist || fo1090.addr == follow_id)
            return max_dist_ndx;
    }
    return MAX_TRACKING_OBJECTS;
}

// fill in certain fields from each message type
// anything not filled-in stays as all zeros

static void update_traffic_position(int index)
{
    // Find in table, or try and create a new entry
    // Only position messages create a traffic table entry
    //   - and only if not too far or high

    //already done:
    //int index = find_traffic_by_addr(fo1090.addr);
    //if (index < 0)        // already tracked via other means
    //    return;

    float alt_diff;
    container_t *cip;
    if (index == MAX_TRACKING_OBJECTS) {      // not found
        // convert pressure altitude to GNSS ellipsoid altitude
        if (mm.alt_type == 0) {               // not GPS altitude
          if (baro_chip != NULL)
            fo1090.altitude += ThisAircraft.baro_alt_diff;
          else
            fo1090.altitude += average_baro_alt_diff;
        }
        alt_diff = fo1090.altitude - ThisAircraft.altitude;
        index = add_traffic_by_dist(alt_diff);
        if (index == MAX_TRACKING_OBJECTS)    // no room
            return;
        cip = &Container[index];
        //*cip = EmptyContainer;
        EmptyContainer(cip);
        cip->addr      = fo1090.addr;
        cip->addr_type = ADDR_TYPE_ICAO;
        icao_to_n(cip);                           // compute USA N-number from ICAO ID
        cip->protocol  = RF_PROTOCOL_ADSB_1090;
        cip->aircraft_type = AIRCRAFT_TYPE_UFO;   // will be updated by identity message
        cip->tx_type   = fo1090.tx_type;
        cip->airborne  = 1;          // we only process airborne traffic (mm.type 9-22)
        cip->stealth   = false;     // affects PFLAA reporting
        cip->no_track  = false;     // affects PFLAA reporting
        //cip->circling  = 0;
        //cip->timerelayed = 0;
    } else {
        // this ID already tracked, just update some fields
        cip = &Container[index];
        if (fo1090.tx_type > cip->tx_type)
            cip->tx_type = fo1090.tx_type;
        if (mm.alt_type == 0) {               // not GPS altitude
          if (cip->baro_alt_diff != 0)        // from a velocity message
            fo1090.altitude += cip->baro_alt_diff;
          else if (baro_chip != NULL)
            fo1090.altitude += ThisAircraft.baro_alt_diff;
          else
            fo1090.altitude += average_baro_alt_diff;
        }
        alt_diff = fo1090.altitude - ThisAircraft.altitude;
        cip->prevaltitude = cip->altitude;
        cip->prevtime_ms = cip->gnsstime_ms;
    }
    if (settings->mode_s)
        sample_rssi_zone(mm.rssi, fo1090.distance, alt_diff);
    cip->altitude  = fo1090.altitude;
    cip->latitude  = fo1090.latitude;
    cip->longitude = fo1090.longitude;
    cip->distance  = fo1090.distance;
    cip->dx        = fo1090.dx;
    cip->dy        = fo1090.dy;
    cip->bearing   = fo1090.bearing;
    cip->alt_diff  = alt_diff;
    cip->rssi      = mm.rssi;
    cip->timestamp    = OurTime;
    cip->positiontime = OurTime;
    cip->gnsstime_ms  = millis();
    Traffic_Update(cip);

/* also send data out via NMEA */
if (settings->debug_flags & DEBUG_DEEPER) {
  if (settings->nmea_d || settings->nmea2_d) {
    snprintf_P(NMEABuffer, sizeof(NMEABuffer),
      PSTR("$PSADS,%06X,%d,%d,%d,%d\r\n"),
      cip->addr, cip->tx_type, (int)cip->alt_diff, (int)cip->distance, mm.rssi);
    NMEAOutD();
  }
}

    // relay some traffic - only if we are airborne (or in "relay only" mode)
    if (settings->rf_protocol == RF_PROTOCOL_LEGACY || settings->rf_protocol == RF_PROTOCOL_LATEST) {
        if (settings->relay > RELAY_LANDED)
               // && (ThisAircraft.airborne || settings->relay == RELAY_ONLY))
            air_relay(cip);
    }
}


// based in part on https://github.com/watson/libmodes/

// we have the 112-bit (14 byte) message as hex digits, 2 per byte
// (28 chars) preceded with '*' and finished by ';' + <CR><LF>
// for example: *8D4B1621994420C18804887668F9;
// (the shorter mode-S messages have fewer (14) chars,
// for example: *02E198BFAF8676;)
// Or, in mode 2+ with an RSSI prefix:
// starts with '+', 2 chars RSSI, rest as above:
// for example:  +1A8DC03ABC9901939CA00706079C17;

// The transponder ES messages (DF17) are structured as:
// DF: 5 bits
// CA: 3 bits
// ICAO ID: 24 bits
// ME (message body): 56 bits
// PI (CRC etc): 24 bits

// assume the data is always valid and uppercase
//int hex2bin(char c) {
//    if (c >= 'A')  return (0xA + (c - 'A'));
//    return (c - '0');
//}
// make this inline:
#define hex2bin(c) (((c)>='A')? (0xA+((c)-'A')) : ((c)-'0'))


// decode Gillham ("Gray") coded altitude
// - based on http://www.ccsinfo1090.com/forum/viewtopic.php?p=140960
// - requires input bits prepared as:
//   dab  D2 D4 A1 A2 A4 B1 B2 B4 (MSB to LSB, D1 assumed 0)
//   c    C1 C2 C4 (MSB to LSB)
static uint32_t GillhamDecode(uint32_t dab, uint32_t c)
{
    c ^= (c>>2);
    c ^= (c>>1);
    if (c == 0 || c == 5 || c == 6) {
        return 0;
    }
    if (c == 7)
        c = 5;
    dab ^= (dab>>4);
    dab ^= (dab>>2);
    dab ^= (dab>>1);
    if (dab & 0x01)
        c = 6 - c;
    return ((5*dab + c - 13) * 100);
}

// Decode the 12 bit AC altitude field (in DF 17 and others, but not Mode S).
// Returns the altitude, or 0 if it can't be decoded.
static float decode_ac12_field() {
    if (msg[5] == 0 && (msg[6]&0xF0) == 0)
        return 0;             // all bits 0 = altitude unknown
    uint32_t alt;
    if ((msg[5] & 0x01) == 0) {    // q_bit not set, altitude is Gillham-coded
/*
        byte 0-based:  5         6
        bit, 0-based:  40-47     48-55
        bit, 1-based:  41        49
                       BBBB BBBB BBBB BBBB
        altitude bits: ^^^^ ^^^Q ^^^^            mapping assumed, not in the book
                   A    1 2  4
                   B          1  2 4
                   C   1 2  4        
                   D              2 4
*/
        //int D = ((msg[6] & 0x40)>>5) | ((msg[6] & 0x10)>>4);
        //int A = ((msg[5] & 0x40)>>4) | ((msg[5] & 0x10)>>3) | ((msg[5] & 0x04)>>2);
        //int B = ((msg[5] & 0x02)<<1) | (((msg[6])>>6) & 0x02) | ((msg[6] & 0x20)>>5);
        //int DAB = (D<<6) | (A<<3) | B;
        uint32_t dab = ((msg[6] & 0x40) << 1) | ((msg[6] & 0x10) << 2) |
                       ((msg[5] & 0x40)>>1) | ((msg[5] & 0x10)) | ((msg[5] & 0x04)<<1) |
                       ((msg[5] & 0x02)<<1) | (((msg[6])>>6) & 0x02) | ((msg[6] & 0x20)>>5);
        uint32_t c = (((msg[5])>>5) & 0x04) | ((msg[5] & 0x20)>>4) | ((msg[5] & 0x08)>>3);
        alt = GillhamDecode(dab, c);
        if (alt == 0)
            return 0;
        //if (mm.msgtype == 'P')
        //    mm.msgtype = 'H';

    } else {
        // N is the 11 bit integer resulting from the removal of bit Q
        alt = ((msg[5]>>1)<<4) | ((msg[6]&0xF0) >> 4);
        alt = alt*25-1000;
    }

    return 0.3048 * (float) alt;   // meters
}

// ideally this should be calibrated for the specific aircraft
//   using actually received ADS-B data over time
float estimate_distance_from_rssi(uint8_t rssi)
{
// based on limited data from ground-based SoftRF reception of ADS-B DF17
// will be revised - and possibly should be calibrated for the individual installation
    if (rssi < close_rssi)
        return ALARM_ZONE_NONE;           // 15 km, will not be reported at all
    if (rssi < report_rssi)               // for RSSI 30-32 (with mode_s=3)
        return 2*ALARM_ZONE_CLOSE;        // estimate as 3000m, not reported in PFLAA
    if (rssi < alarm1_rssi)               // for RSSI 33-34
        return 2*ALARM_ZONE_LOW;          // report as 2000m, report, but still no alarm
    if (rssi < alarm2_rssi)               // for RSSI 35-37
        return ALARM_ZONE_IMPORTANT;      // report as 700m, generate alarm level LOW
    return ALARM_ZONE_URGENT;             // 400m, alarm level IMPORTANT for RSSI 38+
    // note: alarm will only be generated if this reported distance plus 5*alt_diff < zone,
    // i.e., with up to 60m alt_diff (possibly 120m alt_diff for alarm level 1 if level 2 RSSI)
}

// Mode S altitude replies - only altitude & ICAO ID
void update_mode_s_traffic(int i)
{
    // Find in table, or try and create a new entry
    //already done by parse_mode_s_altitude():
    //int i = find_traffic_by_addr(fo1090.addr);
    //if (i < 0)        // already tracked via other means
    //    return;

    float alt_diff;
    container_t *cip;
    if (i == MAX_TRACKING_OBJECTS) {  // not found
        // convert pressure altitude to GNSS ellipsoid altitude
        if (baro_chip != NULL)
          fo1090.altitude += ThisAircraft.baro_alt_diff;
        else
          fo1090.altitude += average_baro_alt_diff;
        alt_diff = fo1090.altitude - ThisAircraft.altitude;
        i = add_traffic_by_dist(alt_diff);
        if (i == MAX_TRACKING_OBJECTS)    // no room
            return;
        cip = &Container[i];
        //*cip = EmptyContainer;
        EmptyContainer(cip);
        cip->addr = fo1090.addr;
        cip->addr_type = ADDR_TYPE_ICAO;
        icao_to_n(cip);                   // compute USA N-number from ICAO ID
        cip->protocol  = RF_PROTOCOL_ADSB_1090;
        cip->aircraft_type = AIRCRAFT_TYPE_UFO;
        cip->tx_type   = fo1090.tx_type;
        cip->airborne  = 1;
    } else {
        // this ID already tracked, just update some fields
        cip = &Container[i];
        if (fo1090.tx_type > cip->tx_type)
            cip->tx_type = fo1090.tx_type;
        // >>> perhaps do this: (would only miss altitude updates via Mode S messages)
        //if (fo1090.tx_type < cip->tx_type)
        //    return;
        if (cip->baro_alt_diff != 0)        // from a velocity message
          fo1090.altitude += cip->baro_alt_diff;
        else if (baro_chip != NULL)
          fo1090.altitude += ThisAircraft.baro_alt_diff;
        else
          fo1090.altitude += average_baro_alt_diff;
        alt_diff = fo1090.altitude - ThisAircraft.altitude;
        cip->prevaltitude = cip->altitude;
        cip->prevtime_ms = cip->gnsstime_ms;
    }
    cip->altitude = fo1090.altitude;
    cip->alt_diff = alt_diff;
    if (cip->tx_type <= TX_TYPE_S) {       // no ADS-B position data
        cip->distance = fo1090.distance;   // estimated from RSSI
        cip->bearing  = 0;                 // non-directional
        cip->dy = fo1090.distance;         // may not be necessary?
        cip->dx = 0;
    }
    cip->rssi = mm.rssi;
    cip->timestamp   = OurTime;
    cip->mode_s_time = OurTime;
    cip->gnsstime_ms = millis();
    Traffic_Update(cip);

if (settings->debug_flags & DEBUG_DEEPER) {
if (settings->nmea_d || settings->nmea2_d) {
/* send data out as NMEA */
  if (settings->nmea_d || settings->nmea2_d) {
    snprintf_P(NMEABuffer, sizeof(NMEABuffer),
      PSTR("$PSMDS,%06X,%d,%d,%d,%d\r\n"),
      cip->addr, cip->tx_type, (int)cip->alt_diff, (int)cip->distance, mm.rssi);
    NMEAOutD();
  }
} else {
  Serial.print(cip->addr, HEX);
  Serial.print(" Mode S altitude (ft): ");
  Serial.print((int)(3.2808*cip->altitude));
  if (cip->tx_type <= TX_TYPE_S)
    Serial.print(" rough distance: ");
  else
    Serial.print(" ADS-B distance: ");
  Serial.print((int)cip->distance);  // may be filled in by a too-far ADS-B message
  Serial.print(" RSSI: ");
  Serial.print(mm.rssi);
  Serial.print(" tx_type: ");
  Serial.println(cip->tx_type);
}
}
}

// decode altitude from surveillance responses (DF 0 and 4)
static bool parse_mode_s_altitude(int index)
{
/*
byte 0-based:  0   1   2   3
bit, 0-based:  0   8   16  24-31
bit, 1-based:  1   9   17  25-32
               HH  HH  HH  HH----|
                       v         |
                       17   21   v
                       BBBB BBBB BBBB BBBB
altitude bits:            ^ ^^^^ ^^^^ ^^^^
                                  M Q
                      A     1 2  4       
                      B            1  2 4
                      C   1  2 4        
                      D                2 4
*/
    uint32_t alt;
    if (msg[3]==0 && (msg[2] & 1) == 0) {  // altitude not available
        alt = 0;
        return false;
    } else if ((msg[3] & 0x40) != 0) {     // M bit set - metric altitude
        // N is the 12 bit integer resulting from the removal of the M bit
        alt = ((msg[2]&0x1F)<<7) | ((msg[3]&0x80) >> 1) | (msg[3] & 0x3F);
if (settings->debug_flags & DEBUG_DEEPER) {
Serial.print("metric altitude: ");
Serial.println(alt);
}
        // convert altitude from meters into feet            
        //alt *= 3360;
        //alt >>= 10;
        // >>> this can't be right, since with 12 bits it is limited to 4095 meters
        alt = 0;
        return false;
    } else if ((msg[3] & 0x10) == 0) {     // q_bit not set - high altitude - Gray code
        // http://www.ccsinfo.com/forum/viewtopic.php?p=140960
        // C1 C2 C4 (MSB to LSB):
        int c = ((msg[2] & 0x10)>>2) | ((msg[2] & 0x04)>>1) | (msg[2] & 0x01);
        int dab = ((msg[3] & 0x04)<<5) | ((msg[3] & 0x01)<<6) |
                  ((msg[2] & 0x08)<<2) | ((msg[2] & 0x02)<<3) | ((msg[3] & 0x80)>>4) |
                  ((msg[3] & 0x20)>>3) | ((msg[3] & 0x08)>>2) | ((msg[3] & 0x02)>>1);
        alt = GillhamDecode(dab, c);
        if (alt == 0)               // not available
            return false;
    } else {
        // N is the 11 bit integer resulting from the removal of M & Q bits
        alt = ((msg[2]&0x1F)<<6) | ((msg[3]&0x80) >> 2) | ((msg[3]&0x20) >> 1) | (msg[3] & 0x0F);
        // altitude in feet
        alt = alt*25-1000;
    }
    fo1090.altitude = 0.3048 * (float) alt;    // convert altitude from feet into meters
    if (settings->vrange1090 /* 0 means no limit */
           && fabs(fo1090.altitude - ThisAircraft.altitude) > maxaltdiff1090) {
        if (fo1090.addr != settings->follow_id)
            return false;
    }
    update_mode_s_traffic(index);
    ++adsb_packets_counter;
    return true;
}

static bool parse_identity(int i)
{
    // do not create a new entry for an identity message,
    //   wait until a position message arrives
    //int i = find_traffic_by_addr(fo1090.addr);
    //if (i == MAX_TRACKING_OBJECTS)  // not found
    //    return false;
    //if (i < 0)                      // already tracked via other means
    //    return false;

    container_t *cip = &Container[i];

    // may be adding aircraft type (& callsign) to what is still Mode S traffic
    // - but do not update tx_type until valid position message arrives

    uint8_t ac_type = 0;
    if (mm.sub != 0 && mm.type > 2)
        ac_type = msg[4] - 0x18;       // 0x18 = 00011000 TC=3, CA=0
        // 0x0 unknown
        // 0x1 glider
        // 0x2 LTA 
        // 0x3 parachute 
        // 0x4 hang glider
        // 0x5 reserved
        // 0x6 UAV
        // 0x7 spacecraft
        // 0x8 not used
        // 0x9 light 
        // 0xA med1
        // 0xB med2
        // 0xC high vortex
        // 0xD heavy
        // 0xE high perf
        // 0xF rotorcraft
    cip->aircraft_type = ac_type_table[ac_type];

    //if (settings->debug_flags & DEBUG_RESVD1) {
    //    if (cip->aircraft_type == AIRCRAFT_TYPE_UFO)
    //        Serial.printf("mm.type=%d  mm.sub=%d  msg[4]=%d  ac_type=%d\r\n",
    //            mm.type, mm.sub, msg[4], ac_type);
    //}

    if (cip->callsign[0]!='\0' && cip->callsign[9]!='?')   // received callsign already recorded
        return true;

    //raw_callsign = last 6 bytes
    // decode callsign
    static const char *ais_charset = "@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_ !\"#$%&'()*+,-./0123456789:;<=>?";
    // Note that mapping 6-bit binary values into this table always results in a printable character.
    char c = ais_charset[msg[5]>>2];
    if (c == ' ')                       // received callsign is blank
        return true;
    cip->callsign[0] = c;
    cip->callsign[1] = ais_charset[((msg[5]&3)<<4)|(msg[6]>>4)];
    cip->callsign[2] = ais_charset[((msg[6]&15)<<2)|(msg[7]>>6)];
    cip->callsign[3] = ais_charset[msg[7]&63];
    cip->callsign[4] = ais_charset[msg[8]>>2];
    cip->callsign[5] = ais_charset[((msg[8]&3)<<4)|(msg[9]>>4)];
    cip->callsign[6] = ais_charset[((msg[9]&15)<<2)|(msg[10]>>6)];
    cip->callsign[7] = ais_charset[msg[10]&63];
    if (cip->callsign[7]==' ') { cip->callsign[7]='\0';
    if (cip->callsign[6]==' ') { cip->callsign[6]='\0';
    if (cip->callsign[5]==' ') { cip->callsign[5]='\0';
    if (cip->callsign[4]==' ') { cip->callsign[4]='\0';
    }}}} else cip->callsign[8] = '\0';
    cip->callsign[9] = '\0';                 // marks as a received callsign, not computed

    return true;
}

static bool parse_position(int index)
{
    // Most receiveable signals are from farther away than we may be interested in.
    // An efficient way to filter them out at this early stage will save a lot of CPU cycles.

    // filter by altitude first, weeds out the jets at 30,000 feet

    // altitude is in msg[5] & MSnibble of msg[6]
    if (mm.type <= 18) {     // baro alt
        //mm.alt_type = 0;
        fo1090.altitude = decode_ac12_field();   // meters
    } else {      // GNSS alt, rare
        mm.alt_type = 1;
        //mm.msgtype = 'G';
        fo1090.altitude = (float)((msg[5] << 4) | ((msg[6] >> 4) & 0x0F));
           // supposedly meters, but can't be right, since 12 bits is not enough
           fo1090.altitude = 0;
    }

    // filter by altitude, but always include "followed" aircraft
    if (settings->vrange1090 /* 0 means no limit */
           && fabs(fo1090.altitude - ThisAircraft.altitude) > maxaltdiff1090) {
        if (fo1090.addr != settings->follow_id)
            return false;
    }

    // prepare to decode lat/lon

    mm.fflag = ((msg[6] & 0x4) >> 2);
    //tflag = msg[6] & 0x8;
    mm.cprlat = ((msg[6]&3) << 15) | (msg[7] << 7) | (msg[8] >> 1);
    mm.cprlon = ((msg[8]&1) << 16) | (msg[9] << 8) | msg[10];

    int32_t m = (int32_t) mm.cprlat;
    int32_t r = (int32_t) ourcprlat[mm.fflag];   // convert from unsigned to signed...
    if (m-r > (1<<16)) {
        // maybe it's just wraparound on the edge of the zone
        // - choose the closer interpretation
        m -= (1<<17);
    } else if (r-m > (1<<16)) {
        m += (1<<17);
    }
    int32_t cprlatdiff = m - r;
    int32_t abslatdiff = abs(cprlatdiff);
    if (abslatdiff > maxcprdiff) {        // since even just lat diff is too far
        if (fo1090.addr != settings->follow_id) {
            if (index < MAX_TRACKING_OBJECTS)    // found
                Container[index].distance = 99999;
            return false;                // no need to compute slant distance
        }
    }

    // identify the NL zone, ours, an adjacent one, or beyond
    bool adjacent = true;
    if (reflat < 7.5 && reflat > 7.5) {              // one big NL zone around the equator
        r = (int32_t) ourcprlon[mm.fflag];
    } else if (reflat > 0) {
      if (m < cprNL1lat[mm.fflag]) {                 // target lat in higher-NL zone
        if (m < cprPluslat[mm.fflag]) {              // beyond the adjacent zone
            adjacent = false;
        }
        r = (int32_t) ourcprlonPlus[mm.fflag];
      } else if (m > cprNL0lat[mm.fflag]) {          // target lat in lower-NL zone
        if (m > cprMinuslat[mm.fflag]) {             // beyond the adjacent zone
            adjacent = false;
        }
        r = (int32_t) ourcprlonMinus[mm.fflag];
      } else {
        r = (int32_t) ourcprlon[mm.fflag];
      }
    } else {                                         // reflat < 0
      if (m > cprNL1lat[mm.fflag]) {                 // in higher-NL zone (towards equator)
        if (m > cprPluslat[mm.fflag]) {              // beyond the adjacent zone
            adjacent = false;
        }
        r = (int32_t) ourcprlonPlus[mm.fflag];
      } else if (m < cprNL0lat[mm.fflag]) {          // in lower-NL zone (towards south pole)
        if (m < cprMinuslat[mm.fflag]) {             // beyond the adjacent zone
            adjacent = false;
        }
        r = (int32_t) ourcprlonMinus[mm.fflag];
      } else {
        r = (int32_t) ourcprlon[mm.fflag];
      }
    }
    m = (int32_t) mm.cprlon;
    if (m-r > (1<<16)) {
        m -= (1<<17);
    } else if (r-m > (1<<16)) {
        m += (1<<17);
    }
    int32_t cprlondiff = m - r;
    int32_t abslondiff = abs(cprlondiff);
    if (adjacent) {
      if (fo1090.addr != settings->follow_id) {
        // reject some too-far traffic based on lon-diff alone
        if (abslondiff > maxcprdiff && fo1090.addr != settings->follow_id) {
            if (index < MAX_TRACKING_OBJECTS)     // found
                Container[index].distance = 99999;
            return false;
        }
        // weed out remaining too-far using pre-computed squared-hypotenuse
        // - no need to compute the un-squared distance at this point
        // - will compute more exact distance below using hypot()
        //   & relative to this target's actual location rather than reference
        abslatdiff >>= 4;
        abslondiff >>= 4;
        if (abslatdiff*abslatdiff + abslondiff*abslondiff > maxcprdiff_sq
                                       && fo1090.addr != settings->follow_id) {
            if (index < MAX_TRACKING_OBJECTS)    // found
                Container[index].distance = 99999;
            return false;
        }
      }
    }

    yield();

    if (decodeCPRrelative() == false)        // error decoding lat/lon
        return false;

    // compute more exact distance, & relative E & N, from this aircraft's actual location
    int32_t y = (int32_t)(111300.0 * (fo1090.latitude - ThisAircraft.latitude));     // meters
    int32_t x = (int32_t)(111300.0 * (fo1090.longitude - ThisAircraft.longitude) * CosLat(/*ThisAircraft.latitude*/));
    fo1090.dx = x;
    fo1090.dy = y;
    fo1090.distance = hypot(x, y);
    if (settings->hrange1090 && fo1090.distance > maxdistance1090) {
        if (fo1090.addr != settings->follow_id) {
            if (index < MAX_TRACKING_OBJECTS) {   // found, so used to be closer
                Container[index].latitude = fo1090.latitude;
                Container[index].longitude = fo1090.longitude;
                Container[index].distance = fo1090.distance;
                Container[index].dx = x;
                Container[index].dy = y;
                // but leave its timestamp alone, let it expire
            }
            return false;
        }
    }
    if (fo1090.distance == 0) {
        fo1090.bearing = 0;
    } else {
        fo1090.bearing = R2D * atan2((float)x,(float)y);
        if (fo1090.bearing < 0)
            fo1090.bearing += 360;
    }

if (settings->debug_flags & DEBUG_DEEPER) {
if (! (settings->nmea_d || settings->nmea2_d)) {
Serial.print(fo1090.addr, HEX);
Serial.print(" ADS-B altitude (ft): ");
Serial.print((int)(3.2808*fo1090.altitude));
Serial.print(" distance (m): ");
int d = (int)fo1090.distance;
if (d < 10000)  Serial.print("0");
if (d < 1000)   Serial.print("0");
Serial.print(d);
Serial.print(" bearing: ");
Serial.print((int)fo1090.bearing);
Serial.print(" RSSI: ");
Serial.println(mm.rssi);
//Serial.print(" Latitude: ");
//Serial.println(fo1090.latitude);
//Serial.print(" Longitude: ");
//Serial.println(fo1090.longitude);
//Serial.print(" dx: ");
//Serial.println(fo1090.dx);
//Serial.print(" dy: ");
//Serial.println(fo1090.dy);
}
}
    update_traffic_position(index);
    ++adsb_packets_counter;
    return true;
}

static bool parse_velocity(int i)
{    
    // do not create a new entry for a velocity message,
    //   wait until position message arrives
    //int i = find_traffic_by_addr(fo1090.addr);
    //if (i == MAX_TRACKING_OBJECTS)  // not found
    //    return false;
    //if (i < 0)                      // already tracked via other means
    //    return false;

    container_t *cip = &Container[i];

    // if the aircraft sends velocity too often, don't process more than once per second
    if (cip->velocitytime == OurTime)
        return false;
    cip->velocitytime = OurTime;

    int ew_dir;
    int ew_velocity;
    int ns_dir;
    int ns_velocity;

    if (mm.sub == 1 || mm.sub == 2) {   // ground speed

      ew_dir = (msg[5]&4) >> 2;
      ew_velocity = ((msg[5]&3) << 8) | msg[6];     // knots
      ns_dir = (msg[7]&0x80) >> 7;
      ns_velocity = ((msg[7]&0x7f) << 3) | ((msg[8]&0xe0) >> 5);
      if (ew_velocity > 0)        // zero means not available
          ew_velocity -= 1;
      if (ns_velocity > 0)
          ns_velocity -= 1;
      if (mm.sub == 2) {       // supersonic
          ew_velocity <<= 2;  // 4x
          ns_velocity <<= 2;  // 4x
      }

      if (ew_dir)
          mm.ewv = -ew_velocity;
      else
          mm.ewv = ew_velocity;
      if (ns_dir)
          mm.nsv = -ns_velocity;
      else
          mm.nsv = ns_velocity;

      // Compute velocity and angle from the two speed components
      cip->speed = (float) hypot((float) mm.nsv, (float) mm.ewv);   // knots
      if (cip->speed > 0) {
          cip->prevcourse = cip->course;
          cip->course = R2D * atan2((float) mm.ewv, (float) mm.nsv);
          // We don't want negative values but a 0-360 scale.
          if (cip->course < 0)
              cip->course += 360;
          //mm.track_is_valid = 1;
      } else {
          cip->course = 0;
          //mm.track_is_valid=0;
      }

      // the following fields are absent from a groundspeed message type
      //heading_is_valid=0; heading=0; airspeed_type=0; airspeed=0;

#if 0
    } else if (mm.sub == 3 || mm.sub == 4) {   // air speed (rare) (not processed here)

      //mm.heading_is_valid = ((msg[5] & 4) >> 2);
      cip->prevheading = cip->heading;
      int16_t iheading = (((msg[5] & 3) << 5) | ((msg[6] >> 3) & 0x1F));
      //cip->heading = (360.0/128) * iheading;
      cip->heading = ((iheading * 360 + 180) >> 7);
      //mm.airspeed_type = (((msg[7]) >> 7) & 1);
      mm.airspeed = ((msg[7]&0x7F) << 3) | (((msg[8]) >> 5) & 0x07);  // if 0, no info
      if (mm.airspeed > 0)        // zero means not available
          mm.airspeed -= 1;
      if (mm.sub == 4)
         mm.airspeed <<= 2; // 4x

      // the following fields are absent from an airspeed message type
      //ewv=0; nsv=0; groundspeed=0; track=0; track_is_valid=0;
#endif
    }

    //mm.vert_rate_source = (msg[8]&0x10) >> 4;   // 0=GNSS, 1=baro
    int vert_rate_sign = (msg[8]&0x8) >> 3;
    int raw_vert_rate = ((msg[8]&7) << 6) | ((msg[9]&0xfc) >> 2);
    cip->vs = (float)((raw_vert_rate - 1) << 6);    // fpm
    if (vert_rate_sign)  cip->vs = -cip->vs;
    int raw_alt_diff = msg[10];  // MSB is sign
    int alt_diff_sign = ((raw_alt_diff & 0x80) >> 7);
    // raw_alt_diff = ((raw_alt_diff & 0x7F) - 1) * 25;   // feet
    raw_alt_diff = ((((raw_alt_diff & 0x7F) - 1) * 7803) >> 10);   // meters
    if (alt_diff_sign)  raw_alt_diff = -raw_alt_diff;  // GNSS altitude is below baro altitude
    cip->baro_alt_diff = (float)raw_alt_diff;

    // keep an average estimate of baro alt diff as reported from nearby aircraft
    static int8_t prev_count = 63;
    // if altitude is very different from ours then ignore baro_alt_diff
    if (fabs(cip->altitude - ThisAircraft.altitude) < 2000) {
        if (average_baro_alt_diff == 0) {
            average_baro_alt_diff = cip->baro_alt_diff;
        } else {
            prev_count -= adsb_acfts;
            if (--prev_count <= 0) {   // decrement by 2 if one aircraft, 3 if two, ...
                prev_count = 63;
                average_baro_alt_diff = 0.8 * average_baro_alt_diff + 0.2 * cip->baro_alt_diff;
            }
        }
    }

    return true;
}

// assume the n chars in buf[] include the starting '*' but not the ending ';'

static bool parse(char *buf, int n)
{
    //mm = EmptyMsg;      // start with a clean slate of all zeros
    EmptyMsg(&mm);        // start with a clean slate of all zeros
    //int k=0;
    int i;
    if (buf[0] == '*') {
        if (n != 29)
            return false;     // not a 112-bit ES
        //mm.rssi = 0;
        i = 1;    // point to DF
    } else
    if (buf[0] == '+') {
        if (settings->mode_s) {
            if (n != 31 && n != 17)
                return false;     // not a 112-bit ES nor 56-bit Mode-S
        } else {
            if (n != 31)
                return false;
        }
        mm.rssi = (hex2bin(buf[1]) << 4) | hex2bin(buf[2]);
        i = 3;    // point to DF
    }
    else {
        return false;     // not a valid GNS5892 sentence
    }

    // parse just the first byte for now (DF, CF)
    msg[0] = (hex2bin(buf[i]) << 4) | hex2bin(buf[i+1]);
    i += 2;

    mm.frame = msg[0]>>3;    // Downlink Format
//Serial.print("DF ");
//Serial.print(mm.frame);
//Serial.print(" with RSSI ");
//Serial.println(mm.rssi);

/*
To determine whether you receive an ADS-B message or a TIS-B message you should start
looking at the Downlink Format (DF, first 5 bits of the message) if the DF = 17, then
it is an ADS-B message. If the DF = 18 then you look at the control field (CF, bits 6-8).
CF = 0 and CF = 1 are ADS-B messages. CF = 2,3 & 5 are TIS-B messages. CF = 6 are ADS-R
messages. CF = 4 are TIS-B / ADS-R system status messages.
https://aviation.stackexchange.com/questions/17610/what-icao-codes-are-reserved-for-tis-b-use
*/

    bool adsr = false;
    bool tisb = false;
    if (mm.frame == 18) {
        switch (msg[0] & 7) {      // CF
        case 2:
        case 3:
        case 5:
            tisb = true;           // mark as TIS-B
            break;
        case 6:
            adsr = true;           // mark as ADS-R
            break;
        case 4:
            // status messages, discard
            return false;
        default:
if (settings->debug_flags & DEBUG_DEEPER) {
Serial.print("DF18 with unusual CF=");
Serial.println((int)(msg[0] & 7));
}
            // treat the same as DF17
            break;
        }
    }

    int j=1;
    uint32_t addr;       // ICAO address
    container_t *cip;

    if (mm.frame == 0 || mm.frame == 4) {    // Mode S altitude replies

        if (! settings->mode_s)
            return false;

        float distance = estimate_distance_from_rssi(mm.rssi);
        if ((settings->debug_flags & DEBUG_DEEPER) == 0) {
            if (distance > 2*ALARM_ZONE_LOW)       // ignore weak signals
                return false;
        }
        static uint32_t lasttime = 0;
        uint32_t thistime = millis();
        if (thistime < lasttime + 300) {       // some aircraft send bursts of messages
//if ((settings->debug_flags & DEBUG_DEEPER) && ! (settings->debug_flags & DEBUG_ALARM))
if ((settings->debug_flags & (DEBUG_DEEPER|DEBUG_ALARM)) == DEBUG_DEEPER)
Serial.println("throttling Mode S message burst");
            return false;
        }
        lasttime = thistime;

        // convert the rest of the message from hex to binary
        while (i < n) {
            msg[j++] = (hex2bin(buf[i]) << 4) | hex2bin(buf[i+1]);
            i += 2;
        }
        addr = addr_from_crc( j );           // assume checksum OK, extract overlayed ICAO ID
        if (addr == 0)
            return false;
        if (addr == ThisAircraft.addr)       // somehow seeing ourselves
            return false;
        if (addr == settings->ignore_id)     // ID told in settings to ignore
            return false;

        // save CPU cycles: don't process if data will not be used
        int index = find_traffic_by_addr(addr);
        if (index < 0)                       // already tracked via other means
            return false;
        if (index < MAX_TRACKING_OBJECTS) {   // found
            cip = &Container[index];
            // if the aircraft has recently sent ADS-B position, don't process Mode S
            if (cip->positiontime == OurTime) {
//if ((settings->debug_flags & DEBUG_DEEPER) && !(settings->debug_flags & DEBUG_ALARM))
if ((settings->debug_flags & (DEBUG_DEEPER|DEBUG_ALARM)) == DEBUG_DEEPER)
Serial.println("ignoring S - have recent P");
                return false;
            }
            // if the aircraft sends Mode S altitudes too often, don't process more than once per second
            if (cip->mode_s_time == OurTime)
                return false;
        }

        //fo1090 = EmptyFO1090;
        EmptyFO1090(&fo1090);
        fo1090.addr = addr;
        fo1090.tx_type = TX_TYPE_S;
        fo1090.distance = distance;

        return parse_mode_s_altitude(index);
    }

    if (mm.frame != 17 && mm.frame != 18) {
        // all other DFs
        return false;
    }

    // parsing of the 56-bit ME - just DF 17-18:

    // parse the next 3 bytes (ICAO ID)
    while (j < 4) {
        msg[j++] = (hex2bin(buf[i]) << 4) | hex2bin(buf[i+1]);
        i += 2;
    }
    addr = (msg[1] << 16) | (msg[2] << 8) | msg[3];
    if (addr == ThisAircraft.addr)        // somehow seeing ourselves
        return false;
    if (addr == settings->ignore_id)      // ID told in settings to ignore
        return false;

    // save CPU cycles: don't process if data will not be used
    int index = find_traffic_by_addr(addr);
    if (index < 0)                        // already tracked via other means
        return false;
    if (index < MAX_TRACKING_OBJECTS)     // found
        cip = &Container[index];

    // convert the rest of the message from hex to binary
    n -= 6;                               // skip the CRC
    while (i < n) {
        msg[j++] = (hex2bin(buf[i]) << 4) | hex2bin(buf[i+1]);
        i += 2;
    }

    if (j != 11)    // 112 bits minus the CRC
        return false;

    // start with a clean slate
    //fo1090 = EmptyFO1090;
    EmptyFO1090(&fo1090);
    fo1090.addr = addr;

    if (tisb)
        fo1090.tx_type = TX_TYPE_TISB;
    else if (adsr)
        fo1090.tx_type = TX_TYPE_ADSR;
    //else if (mm.frame < 17)
    //    fo1090.tx_type = TX_TYPE_S;
    else
        fo1090.tx_type = TX_TYPE_ADSB;

    yield();

    mm.type = msg[4] >> 3;   // Extended squitter message type.
    mm.sub = msg[4] & 7;     // Extended squitter message subtype.

    if (mm.type >= 1 && mm.type <= 4) {

        if (index == MAX_TRACKING_OBJECTS)   // not found
            return false;

        return parse_identity(index);     // may add aircraft type to Mode S

    } else if (mm.type >= 9 && mm.type <= 22 && mm.type != 19) {

        if (index < MAX_TRACKING_OBJECTS) {   // found

            // if the aircraft sends positions too often, don't process more than once per second
            if (cip->positiontime == OurTime)
                return false;

            // mark as transmitting ADS-B, even if rejected below (too far, etc)
            //if (fo1090.tx_type > cip->tx_type)
            //    cip->tx_type = fo1090.tx_type;
            // - don't, since then Mode-S data may be treated as "directional"
        }

        return parse_position(index);

    } else if (mm.type == 19 && (mm.sub == 1 || mm.sub == 2)) {

        if (index == MAX_TRACKING_OBJECTS)            // not found
            return false;
        if (cip->tx_type < TX_TYPE_TISB)  // no position - ignore velocity
            return false;

        return parse_velocity(index);

    }

//Serial.printf("Not processed: DF=%d type=%d sub=%d\r\n", mm.frame, mm.type, mm.sub);

    return false;
}


static void send5892(const char *cmd)
{
    Serial2.print(cmd);
    Serial2.print("\r\n");
}

void play5892()
{
    if (settings->mode_s)
        send5892("#49-82");                     // all DFs, with RSSI
    else
        send5892("#49-03");                     // only DF 17 and 18, no RSSI
    //paused = false;
    //Serial.println("(GNS5892 un-paused)");
    rx1090found = false;    // will be reset to true when a response arrives
}

static void pause5892()
{
    send5892("#49-00");
    //paused = true;
    Serial.println("(GNS5892 paused)");
    while (Serial2.available()) {
        Serial2.read();
        yield();
    }
}

static void reset5892()
{
    Serial.println("(resetting GNS5892...)");
    send5892("#FF");
    delay(300);
}

void gns5892_setup()
{
  if (has_serial2 == false)
    return;

  if (load_zone_stats())
      set_zone_thresholds(false);

  CPRRelative_setup();

  pause5892();
  delay(200);
  reset5892();
  delay(2000);
  pause5892();
  delay(200);
  int limit = 2000;
  while (Serial2.available() && --limit>0)  Serial2.read();
  send5892("#00");    // query module firmware version
  delay(400);
  limit = 2000;
  while (Serial2.available() && --limit>0) {
      if (Serial2.read()=='#')
          rx1090found = true;
  }
  if (rx1090found)
      Serial.println(F(">>> GNS5892 module responded"));
  else
      Serial.println(F(">>> GNS5892 module did not respond yet"));
}


void gns5892_test_mode()
{
    settings->mode_s = test_mode;
    pause5892();
    delay(200);
    play5892();
}

// called from NMEA.cpp NMEA_loop() when appropriate
void gns5892_loop()
{
  //if (has_serial2 == false)
  //    return;

  CPRRelative_precomp();   // only precomputes upon first fix or big change in position

  static uint32_t playtime = 0;
  // do not unpause GNS5892 until CPRRelative_precomp() has set reflat
  if ((millis() > playtime + (rx1090found? 57700 : 1200)) && (reflat != 0 || !rx1090found)) {
      play5892();            // start GNS5892, and try and capture a #49 response
      if ((settings->debug_flags & DEBUG_DEEPER) && (reflat != 0 && rx1090found)) {
          Serial.printf("ThisAircraft.baro_alt_diff = %.0f\r\n", ThisAircraft.baro_alt_diff);
          Serial.printf("OthAcfts Avg baro_alt_diff = %.0f\r\n", average_baro_alt_diff);
      }
      playtime = millis();
  }

  int avail = Serial2.available();
  if (avail <= 0)
      return;
  if (avail > (GNS5892_INPUT_BUF_SIZE - 256))    // input buffer is getting full
      Serial2.readBytes(buf1090, 256);           // discard some input data

  static bool input_complete = false;
  static int n = 0;  // inputchars
  if (input_complete)
      n = 0;          // start a new input sentence
  input_complete = false;
  while (Serial2.available()) {  // loop until a full sentence or no more data
      char c = Serial2.read();
      if (c=='*' || c=='+' || c=='#') {
          buf1090[0] = c;            // start new sentence, drop any preceding data
          n = 1;
      } else if (n == 0) {      // wait for a valid starting char
          continue;
      } else if (c==';' || c=='\r' || c=='\n') {   // completed sentence
          if (n > 14 /* && n <= 32 */ ) {
              input_complete = true;
              break;
          }
          n = 0;                 // invalid, start over
      } else {
          buf1090[n++] = c;
      }
  }
  yield();
  if (! input_complete || n == 0)
      return;

  if (buf1090[0] == '*' || buf1090[0] == '+') {    // ADS-B data received
      (void) parse(buf1090, n);
      yield();
  } else if (buf1090[0] == '#') {                 // response to commands
      Serial.write(buf1090, n);                   // copy to console
      Serial.println("");
      if (buf1090[1]=='4' && buf1090[2]=='9') {   // response to "play"
          rx1090found = true;
          Serial.println(F(">>> GNS5892 module responded:"));
          char buf[16];
          snprintf(buf, 16, "#39-00-00-%02X", settings->rx1090x);  // set comparator offset
          send5892(buf);
      }
  }

  NMEA_bridge_sent = true;   // not really sent, but substantial processing
}

#endif  // ESP32
