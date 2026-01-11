/*
 * Protocol_Legacy, decoder for legacy radio protocol
 * Copyright (C) 2014-2015 Stanislaw Pusep
 *
 * Protocol_Legacy, encoder for legacy radio protocol
 * Copyright (C) 2016-2021 Linar Yusupov
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
#include <stdint.h>

#include <protocol.h>

#include "../../../SoftRF.h"
#include "../../TrafficHelper.h"
#include "../../Wind.h"
#include "../../system/Time.h"
#include "../../driver/RF.h"
#include "../../driver/Settings.h"
#include "../../driver/Filesys.h"
#include "../data/IGC.h"
#include "../data/NMEA.h"

const rf_proto_desc_t legacy_proto_desc = {
  "Legacy",
  .type            = RF_PROTOCOL_LEGACY,
  .modulation_type = RF_MODULATION_TYPE_2FSK,
  .preamble_type   = LEGACY_PREAMBLE_TYPE,
  .preamble_size   = LEGACY_PREAMBLE_SIZE,
  .syncword        = LEGACY_SYNCWORD,
  .syncword_size   = LEGACY_SYNCWORD_SIZE,
  .syncword_skip   = LEGACY_SYNCWORD_SKIP,     // ignore this many bytes when receiving
  .net_id          = 0x0000, /* not in use */
  .payload_type    = RF_PAYLOAD_INVERTED,
  .payload_size    = LEGACY_PAYLOAD_SIZE,
  .payload_offset  = 0,
  .crc_type        = LEGACY_CRC_TYPE,
  .crc_size        = LEGACY_CRC_SIZE,

  .bitrate         = RF_BITRATE_100KBPS,
  .deviation       = RF_FREQUENCY_DEVIATION_50KHZ,
  .whitening       = RF_WHITENING_MANCHESTER,
  .bandwidth       = RF_RX_BANDWIDTH_SS_125KHZ,

  .air_time        = LEGACY_AIR_TIME,

#if defined(USE_TIME_SLOTS)
  .tm_type         = RF_TIMING_2SLOTS_PPS_SYNC,
#else
  .tm_type         = RF_TIMING_INTERVAL,
#endif
  .tx_interval_min = LEGACY_TX_INTERVAL_MIN,
  .tx_interval_max = LEGACY_TX_INTERVAL_MAX,
  .slot0           = {400,  800},
  .slot1           = {800, 1200}
};

const rf_proto_desc_t flr_adsl_proto_desc = {
  "FLR_ADSL",
  .type            = RF_PROTOCOL_LEGACY,
  .modulation_type = RF_MODULATION_TYPE_2FSK,
  .preamble_type   = LEGACY_PREAMBLE_TYPE,
  .preamble_size   = LEGACY_PREAMBLE_SIZE,
  .syncword        = FLR_ADSL_SYNCWORD,
  .syncword_size   = FLR_ADSL_SYNCWORD_SIZE,
  .syncword_skip   = FLR_ADSL_SYNCWORD_SKIP,
  .net_id          = 0x0000, /* not in use */
  .payload_type    = RF_PAYLOAD_INVERTED,
  .payload_size    = FLR_ADSL_PAYLOAD_SIZE,
  .payload_offset  = 0,
  .crc_type        = LEGACY_CRC_TYPE,
  .crc_size        = LEGACY_CRC_SIZE,

  .bitrate         = RF_BITRATE_100KBPS,
  .deviation       = RF_FREQUENCY_DEVIATION_50KHZ,
  .whitening       = RF_WHITENING_MANCHESTER,
  .bandwidth       = RF_RX_BANDWIDTH_SS_125KHZ,

  .air_time        = LEGACY_AIR_TIME,

#if defined(USE_TIME_SLOTS)
  .tm_type         = RF_TIMING_2SLOTS_PPS_SYNC,
#else
  .tm_type         = RF_TIMING_INTERVAL,
#endif
  .tx_interval_min = LEGACY_TX_INTERVAL_MIN,
  .tx_interval_max = LEGACY_TX_INTERVAL_MAX,
  .slot0           = {400,  800},
  .slot1           = {800, 1200}
};

/* http://en.wikipedia.org/wiki/XXTEA */
void btea(uint32_t *v, int8_t n, const uint32_t key[4]) {
    uint32_t y, z, sum;
    uint32_t p, rounds, e;

    #define DELTA 0x9e3779b9
    // #define ROUNDS (6 + 52 / n)
    #define ROUNDS 6
    #define MX (((z >> 5 ^ y << 2) + (y >> 3 ^ z << 4)) ^ ((sum ^ y) + (key[(p & 3) ^ e] ^ z)))

    if (n > 1) {
        /* Coding Part */
        rounds = ROUNDS;
        sum = 0;
        z = v[n - 1];
        do {
            sum += DELTA;
            e = (sum >> 2) & 3;
            for (p = 0; p < n - 1; p++) {
                y = v[p + 1];
                z = v[p] += MX;
            }
            y = v[0];
            z = v[n - 1] += MX;
        } while (--rounds);
    } else if (n < -1) {
        /* Decoding Part */
        n = -n;
        rounds = ROUNDS;
        sum = rounds * DELTA;
        y = v[0];
        do {
            e = (sum >> 2) & 3;
            for (p = n - 1; p > 0; p--) {
                z = v[p - 1];
                y = v[p] -= MX;
            }
            z = v[n - 1];
            y = v[0] -= MX;
            sum -= DELTA;
        } while (--rounds);
    }
}


// first stage of decrypting
void btea2(uint32_t *data, bool encode)
{
    // for new protocol, this btea() stage uses fixed keys
    static const uint32_t keys[4] = { 0xa5f9b21c, 0xab3f9d12, 0xc6f34e34, 0xd72fa378 };
    btea(data+2, (encode? 4 : -4), keys);
}

// second stage of decrypting
void scramble(uint32_t *data, uint32_t timestamp)
{

    uint32_t wkeys[4];
    wkeys[0] = data[0];
    wkeys[1] = data[1];
    wkeys[2] = (timestamp >> 4);
    wkeys[3] = 0x956f6c77;         // the scramble KEY

    int z, y, x, sum, p, q;
    //int n = 16;                        // do by bytes instead of longwords
    byte *bkeys = (byte *) wkeys;

    z = bkeys[15];
    sum = 0;
    q = 2;                         // only 2 iterations
    do {
      sum += DELTA;
      y = bkeys[0];
      for (p=0; p<15; p++) {
        x = y;
        y = bkeys[p+1];
        x += ((((z >> 5) ^ (y << 2)) + ((y >> 3) ^ (z << 4))) ^ (sum ^ y));
        bkeys[p] = (byte)x;
        z = x & 0xff;
      }
      x = y;
      y = bkeys[0];
      x += ((((z >> 5) ^ (y << 2)) + ((y >> 3) ^ (z << 4))) ^ (sum ^ y));
      bkeys[15] = (byte)x;
      z = x & 0xff;
    } while (--q > 0);

    // now XOR results with last 4 words of the packet
    data[2] ^= wkeys[0];
    data[3] ^= wkeys[1];
    data[4] ^= wkeys[2];
    data[5] ^= wkeys[3];
}

/* http://pastebin.com/YK2f8bfm */
long obscure(uint32_t key, uint32_t seed) {
    uint32_t m1 = seed * (key ^ (key >> 16));
    uint32_t m2 = (seed * (m1 ^ (m1 >> 16)));
    return m2 ^ (m2 >> 16);
}

static const uint32_t table[8] = LEGACY_KEY1;

void make_key(uint32_t key[4], uint32_t timestamp, uint32_t address) {
    int8_t i, ndx;
    for (i = 0; i < 4; i++) {
        ndx = ((timestamp >> 23) & 1) ? i+4 : i ;
        key[i] = obscure(table[ndx] ^ ((timestamp >> 6) ^ address), LEGACY_KEY2) ^ LEGACY_KEY3;
    }
}

// lookup the divisor for latitude for new protocol
int londiv(int ilat)
{
    static uint8_t table1[] = {
        53,53,54,54,55,55,
        56,56,57,57,58,58,59,59,60,60,
        61,61,62,62,63,63,64,64,65,65,
        67,68,70,71,73,74,76,77,79,80,
        82,83,85,86,88,89,91,94,98,101,
        105,108,112,115,119,122,126,129,137,144,
        152,159,167,174,190,205,221,236,252
    };
    static uint16_t table2[] = {
        267,299,330,362,425,489,552,616,679,743,806
    };
    if (ilat < 14)
        return 52;
    if (ilat < 79)
        return table1[ilat-14];
    if (ilat > 89)
        return 806;
    return table2[ilat-79];
}

// pack integer value into a pseudo-floating format
unsigned int enscale( int value, unsigned int mbits, unsigned int ebits, unsigned int sbits)
{
    unsigned int offset = (1 << mbits);
    unsigned int signbit = (offset << ebits);
    unsigned int negative = 0;
    if (value < 0) {
        if (sbits == 0)      // underflow
            return 0;        // clamp to minimum
        value = -value;
        negative = signbit;
    }
    if (value < offset)
        return (negative | (unsigned int)value);  // just for efficiency
    unsigned int exp = 0;
    unsigned int mantissa = offset + (unsigned int)value;
    unsigned int mlimit = offset + offset - 1;
    unsigned int elimit = signbit - 1;
    while (mantissa > mlimit) {
        mantissa >>= 1;
        exp += offset;
        if (exp > elimit)                 // overflow
            return (negative | elimit);   // clamp to maximum
    }
    mantissa -= offset;
    return (negative | exp | mantissa);    
}

// unpack integer value from a pseudo-floating format
int descale( unsigned int value, unsigned int mbits, unsigned int ebits, unsigned int sbits)
{
    unsigned int offset = (1 << mbits);
    unsigned int signbit = (offset << ebits);
    unsigned int negative = 0;
    if (sbits != 0)
        negative = (value & signbit);
    value &= (signbit - 1);           // ignores signbit and higher
    if (value >= offset) {
        unsigned int exp = value >> mbits;
        value &= (offset - 1);
        value += offset;
        value <<= exp;
        value -= offset;
    }
    return (negative? -(int)value : value);
}

// interpret the data fields in the 2024 protocol packet
//     https://pastebin.com/YB1ppAbt
bool latest_decode(void* buffer, container_t* this_aircraft, ufo_t* fop)
{
    //uint32_t timestamp = (uint32_t) this_aircraft->timestamp;
    //uint32_t timestamp = (uint32_t) OurTime;
    uint32_t timestamp = (uint32_t) RF_time;   // incremented in RF.cpp 300 ms after PPS

#if 0
    if (settings->nmea_d || settings->nmea2_d) {
      if (settings->debug_flags & DEBUG_LEGACY) {
        /* output the raw (encrypted) packet as a whole, in hex */
        snprintf_P(NMEABuffer, sizeof(NMEABuffer),
          PSTR("$PSRFR,%ld,%06X,%s\r\n"),
          timestamp, fop->addr,
          bytes2Hex((byte *)buffer, sizeof (latest_packet_t)));
        //Serial.print(NMEABuffer);
        NMEAOutD();
      }
    }
#endif

    /* decrypt packet */
    uint32_t *wp = (uint32_t *) buffer;
    btea2(wp, false);
    scramble(wp, timestamp);

    latest_packet_t* pkt = (latest_packet_t *) buffer;

#if 0
    if (settings->nmea_d || settings->nmea2_d) {
      if (settings->debug_flags & DEBUG_LEGACY) {
        if (settings->debug_flags & DEBUG_DEEPER) {
          /* output the raw (but decrypted) packet as a whole, in hex */
          snprintf_P(NMEABuffer, sizeof(NMEABuffer),
            PSTR("$PSRFB,%ld,%06X,%s\r\n"),
            timestamp, fop->addr,
            bytes2Hex((byte *)pkt, sizeof (latest_packet_t)));
          //Serial.print(NMEABuffer);
          NMEAOutD();
          if ((timestamp & 0x0F) != pkt->timebits)
              Serial.println("timestamp != timebits");   // local & remote time out of sync
        }
      }
    }
#endif

    // discard packets that look like a decryption error
    // so far it seems that the last byte of the packet is always 0 if decrypted correctly
    // do not insist on exact timebits if relayed in original encryption
    unsigned int timebits = pkt->timebits;
    unsigned int localbits = (timestamp & 0x0F);
    bool time_error = (localbits != timebits);
    // allow being off-by-one (but not at the rollover when one is zero)
    if (localbits + 1 == timebits)
        time_error = false;
    else if (localbits == timebits + 1)
        time_error = false;
    if (time_error) {
        if (localbits == 0) {
            Serial.printf("decrypt time err - local rolled over early %d %d\r\n", localbits, timebits);
        // } else if (timebits == 0) {   // meaningless, timebits was not decrypted correctly
        } else {
            Serial.printf("decrypt time error > 1  - not at roll over %d %d\r\n", localbits, timebits);
        }
        Serial.print("- at time: ");  Serial.println((uint32_t) OurTime);
        return false;
    }
    if (pkt->lastbyte != 0 || pkt->needs3 != 3) {
        Serial.print("rejecting bad decrypt with OK timebits at time ");
        Serial.println((uint32_t) OurTime);
        return false;
    }

    //fop->timestamp = timestamp;
    //fop->gnsstime_ms = millis();

    fop->airborne = (pkt->airborne > 1);
    if (fop->relayed && fop->airborne)
        fop->tx_type  = TX_TYPE_ADSB;    // assumption
    fop->protocol = RF_PROTOCOL_LATEST;

    fop->stealth   = pkt->stealth;
    fop->no_track  = pkt->no_track;
    fop->aircraft_type = pkt->aircraft_type;
//Serial.printf("rcvd AC type: %d\n", fop->aircraft_type);

    int alt = descale(pkt->alt,12,1,0) - 1000;
//Serial.printf("altitude: %d\n", alt);
    fop->altitude = (float) alt;        // was  - this_aircraft->geoid_separation;

    float ref_lat = this_aircraft->latitude;
    float ref_lon = this_aircraft->longitude;

    int32_t round_lat;
    if (ref_lat < 0.0)
        round_lat = -(((int32_t) (-ref_lat * 1e7) + 26) / 52);
    else
        round_lat = ((int32_t) (ref_lat * 1e7) + 26) / 52;
    int32_t ilat = pkt->lat;
    ilat = (ilat - round_lat) & 0x0FFFFF;
    if (ilat >= 0x080000) ilat -= 0x100000;
    float lat = (float)((ilat + round_lat) * 52) * 1e-7;
//Serial.printf("latitude: %f\n", lat);
    fop->latitude = lat;

    int d = londiv((int)fabs(lat));

    int32_t round_lon;
    if (ref_lon < 0.0)
        round_lon = -(((int32_t) (-ref_lon * 1e7) + (d>>1)) / d);
    else
        round_lon = ((int32_t) (ref_lon * 1e7) + (d>>1)) / d;
    int32_t ilon = pkt->lon;
    ilon = (ilon - round_lon) & 0x0FFFFF;
    if (ilon >= 0x080000) ilon -= 0x0100000;
    float lon = (float)((ilon + round_lon) * d) * 1e-7;
//Serial.printf("longitude: %f\n", lon);
    fop->longitude = lon;

    int32_t turnrate = descale(pkt->turnrate,6,2,1);
    fop->turnrate = 0.05 * (float)turnrate;   // hopefully deg/sec
//Serial.printf("turnrate: %f deg/sec\n", fop->turnrate);
    fop->circling = (pkt->airborne == 3) ? (turnrate < 0 ? -1 : 1) : 0;

    int16_t speed10 = descale(pkt->speed,8,2,0);
//Serial.printf("speed10: %d\n", speed10);           // speed, in units of tenths of meters per second
    fop->speed = (1.0 / (10.0 * _GPS_MPS_PER_KNOT)) * (float)speed10;
//Serial.printf("speed: %.1f knots\n", fop->speed);

    int vs10 = descale(pkt->vs,6,2,1);
    fop->vs = ((float) vs10) * (_GPS_FEET_PER_METER * 6.0);
//Serial.printf("VS10? %d\n", vs10);

    int course = ((((uint32_t)pkt->course) & 0x03FF) >> 1);    // course as degrees 0-360
    fop->course = (float) course;
//Serial.printf("course: %d\n", course);

    //int gpsA = descale(pkt->gpsA,3,3,0) / 10;
//Serial.printf("gpsA: 0x%X -> %d\n", pkt->gpsA, gpsA);
    //int gpsB = descale(pkt->gpsB,2,3,0) >> 2;
//Serial.printf("gpsB: 0x%X -> %d\n", pkt->gpsB, gpsB);

    //int32_t unk8 = pkt->unk8;
//Serial.printf("unk8: %d  %x\n", unk8, unk8);

#if 0
    /* send received radio packet data out via NMEA for debugging */
    if (settings->nmea_d || settings->nmea2_d) {
      if (settings->debug_flags & DEBUG_LEGACY) {
        /* fields interpreted in packet */
        snprintf_P(NMEABuffer, sizeof(NMEABuffer),
          PSTR("$PSRFL,%ld,%06X,%.6f,%.6f,%.0f\r\n"),
          timestamp, fop->addr, fop->latitude, fop->longitude, fop->altitude);
        NMEAOutD();
        /* mystery fields in packet */
        snprintf_P(NMEABuffer, sizeof(NMEABuffer),
          PSTR("$PSRFX,%ld,%06X,%d,%d,%d,%d,%d,%d\r\n"),
          timestamp, fop->addr, turnrate, speed10, vs10, course, pkt->airborne, pkt->unk8);
        NMEAOutD();
        if (settings->debug_flags & DEBUG_DEEPER) {
          /* also output ownship data */
          float speedf = this_aircraft->speed * _GPS_MPS_PER_KNOT; /* m/s */
          float fvs10 = this_aircraft->vs * (10.0/ (_GPS_FEET_PER_METER * 60.0)); /* vs10 */
          snprintf_P(NMEABuffer, sizeof(NMEABuffer),
            PSTR("$PSRFA,%ld,%06X,%.6f,%.6f,%.0f,%.0f,%.0f,%.1f,%.1f\r\n"),
            timestamp, fop->addr,
            this_aircraft->latitude, this_aircraft->longitude, this_aircraft->altitude,
            speedf, this_aircraft->course, this_aircraft->turnrate, fvs10);
          NMEAOutD();
        }
      }
    }
#endif

    // do some sanity checks on the data
    if (fabs(fop->latitude - this_aircraft->latitude) > 1.0
     || fabs(fop->longitude - this_aircraft->longitude) > InvCosLat()
     || course > 360 )
    {
        Serial.println("implausible data - rejecting packet");
        return false;
    }

    return true;
}

bool legacy_decode(void *buffer, container_t *this_aircraft, ufo_t *fop) {

    legacy_packet_t *pkt = (legacy_packet_t *) buffer;

    fop->addr = pkt->addr;

    if (fop->addr == settings->ignore_id)
        return false;                 /* ID told in settings to ignore */

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
      if (Container[i].addr == fop->addr) {
        if (RF_last_crc != 0 && RF_last_crc == Container[i].last_crc) {
          //Serial.println("duplicate packet");  // usually duplicated in 2nd time slot
          bool exempt = (Container[i].aircraft_type == AIRCRAFT_TYPE_UNKNOWN
                          && settings->altprotocol != RF_PROTOCOL_NONE
                          && (RF_time & 0x0F) == 0x0F);
             // exempt last slot in 16 sec cycle for landed-out relay in alt-protocol
          if (! exempt)
              return false;
        }
        break;
      }
    }
    fop->last_crc = RF_last_crc;

    fop->relayed = false;
    if (pkt->addr_type > 3) {
        // probably air-relayed packet
        //   but do some sanity checks below
        fop->relayed = true;
    }
    fop->addr_type = pkt->addr_type & 3;
    // but leave pkt->addr_type as is, or decryption will fail

    //uint32_t timestamp = (uint32_t) this_aircraft->timestamp;
    //uint32_t timestamp = (uint32_t) OurTime;
    uint32_t timestamp = (uint32_t) RF_time;   // incremented in RF.cpp 300 ms after PPS
    fop->timestamp = timestamp;
    fop->gnsstime_ms = millis();

    if (pkt->msg_type == 2)
        return latest_decode(buffer, this_aircraft, fop);

    if (pkt->msg_type != 0) {
        Serial.println("skipping packet msg_type neither 2 nor 0");
        return false;
    }

#if 1
    if (! fop->relayed) {
        //Serial.print("received V6 packet from ID ");
        //Serial.println(fop->addr,HEX);
        snprintf_P(NMEABuffer, sizeof(NMEABuffer), PSTR("$PSRF6,%06X\r\n"), fop->addr);
        Serial.print(NMEABuffer);
        NMEAOutD();
#if defined(USE_SD_CARD)
        if (SD_is_mounted)
            FlightLogComment(NMEABuffer+2);   // it will prepend LPLT
#endif
    }
#endif

    // decrypt and decode old legacy protocol:

    uint32_t key[4];
    int ndx;
    uint8_t pkt_parity=0;

    make_key(key, timestamp, (pkt->addr << 8) & 0xffffff);
    btea((uint32_t *) pkt + 1, -5, key);

    for (ndx = 0; ndx < sizeof (legacy_packet_t); ndx++) {
      pkt_parity += parity(*(((unsigned char *) pkt) + ndx));
    }
    if (pkt_parity & 0x01) {
        //if (settings->nmea_p)
          Serial.println(F("$PSRFE,bad parity of decrypted packet"));
        return false;
    }

    uint8_t unk2 = pkt->_unk2;

    float ref_lat = this_aircraft->latitude;
    float ref_lon = this_aircraft->longitude;

    // this section revised by MB on 220526
    int32_t round_lat, round_lon;
    if (ref_lat < 0.0)
        round_lat = -(((int32_t) (-ref_lat * 1e7) + 0x40) >> 7);
    else
        round_lat = ((int32_t) (ref_lat * 1e7) + 0x40) >> 7;
    int32_t ilat = ((int32_t)pkt->lat - round_lat) & 0x07FFFF;
    if (ilat >= 0x040000) ilat -= 0x080000;
    float lat = (float)((ilat + round_lat) << 7) * 1e-7;
    if (ref_lon < 0.0)
        round_lon = -(((int32_t) (-ref_lon * 1e7) + 0x40) >> 7);
    else
        round_lon = ((int32_t) (ref_lon * 1e7) + 0x40) >> 7;
    int32_t ilon = ((int32_t)pkt->lon - round_lon) & 0x0FFFFF;
    if (ilon >= 0x080000) ilon -= 0x0100000;
    float lon = (float)((ilon + round_lon) << 7) * 1e-7;

    // do some sanity checks on the data
    if (fabs(lat - this_aircraft->latitude) > 1.0
     || fabs(lon - this_aircraft->longitude) > InvCosLat())
    {
        Serial.println("too far away - rejecting packet");
        return false;
    }

    uint8_t smult = pkt->smult;
    float nsf = (float) (((int) pkt->ns[0]) << smult);      /* quarter-meters per sec */
    float ewf = (float) (((int) pkt->ew[0]) << smult);
    float course = R2D * atan2(ewf, nsf);
    if (course < 0.0)
        course += 360.0;
    float speed4 = hypot(nsf, ewf);
    float interval, factor;
    if (pkt->aircraft_type == AIRCRAFT_TYPE_TOWPLANE) {      // known 4-second intervals
        //interval = 4.0;
        factor = (1.0/4.0);
    } else if (pkt->aircraft_type == AIRCRAFT_TYPE_DROPPLANE) {   // unverified assumptions
        //interval = 4.0;
        factor = (1.0/4.0);
    } else if (pkt->aircraft_type == AIRCRAFT_TYPE_POWERED) {
        //interval = 4.0;
        factor = (1.0/4.0);
    } else if (unk2 == 1) {
        //interval = 2.0;
        factor = (1.0/2.0);
    } else {                                          // circling gliders
        //interval = 3.0;
        factor = (1.0/3.0);
    }
    float turnrate = 0;
    if (speed4 > 0) {
      float nextcourse = R2D * atan2((float)pkt->ew[1], (float)pkt->ns[1]);
      if (nextcourse < 0.0)
          nextcourse += 360.0;
      float turnangle = (nextcourse - course);
      if (turnangle >  270.0) turnangle -= 360.0;
      if (turnangle < -270.0) turnangle += 360.0;
      turnrate = turnangle * factor;
      /* adjust course direction for turning during time between "now" and [0] */
      // it appears that the time of [0] after "now" is same as the interval between [0] & [1]
      // course -= interval * turnrate;
      course -= turnangle;
      if (course >  360.0) course -= 360.0;
      if (course < -360.0) course += 360.0;
    }

    uint16_t vs_u16 = pkt->vs;
    int16_t vs_i16 = (int16_t) (vs_u16 | (vs_u16 & (1<<9) ? 0xFC00U : 0));
    int16_t vs10 = vs_i16 << smult;

    int16_t alt = pkt->alt ; /* relative to WGS84 ellipsoid */

    fop->airborne = pkt->airborne;
    if (fop->relayed && fop->airborne) {
        //fop->protocol = RF_PROTOCOL_ADSB_1090;   // assumption
        fop->tx_type = TX_TYPE_ADSB;   // assumption
    }
    fop->protocol = RF_PROTOCOL_LEGACY;

    /* FLARM sometimes sends packets with implausible data */
    bool implausible = false;
    if (fop->airborne == 0 && (vs10 > 150 || vs10 < -150))
        implausible = true;
    if (unk2 == 2)   // appears with implausible data in speed fields
        implausible = true;
    /* if (fop->relayed) */ {   // additional sanity checks
        if (speed4 > 600.0)
            implausible = true;
        if (fabs(turnrate) > 100.0)
            implausible = true;
    }
    if (implausible) {
        Serial.println("implausible data - rejecting packet");
        return false;
    }

    if (unk2 == 0)
        fop->circling = 1;   // to the right
    else if (unk2 == 3)
        fop->circling = -1;
    else
        fop->circling = 0;

    /* adjust position to "now" - it sent a position 2 sec into future */
    float course2 = course - turnrate;     // average course over the previous 2 seconds
    float offset = speed4 * (2.0 / 4.0 / 111300.0);   // degslat/sec * 2 sec = degs moved
    fop->latitude  = lat - (offset * cos(D2R * course2));
    fop->longitude = lon - (offset * sin(D2R * course2) * InvCosLat());

    fop->altitude = (float) alt;   // was  - this_aircraft->geoid_separation;
    fop->speed = (1.0 / (4.0 * _GPS_MPS_PER_KNOT)) * speed4;
    fop->aircraft_type = pkt->aircraft_type;
    fop->course = course;
//    fop->heading = heading;
    fop->turnrate = turnrate;
         /* this is as reported by FLARM, which is ground-reference - at time [0]? */
    fop->vs = ((float) vs10) * (_GPS_FEET_PER_METER * 6.0);
    fop->stealth = pkt->stealth;
    fop->no_track = pkt->no_track;
    /* There is no need to keep the ns[] & ew[] data  */
    /* We already computed course and speed from them */
    //fop->fla_ns[0] = ((int16_t) pkt->ns[0]) << smult;
    //fop->fla_ns[1] = ((int16_t) pkt->ns[1]) << smult;
    //fop->fla_ew[0] = ((int16_t) pkt->ew[0]) << smult;
    //fop->fla_ew[1] = ((int16_t) pkt->ew[1]) << smult;

#if 0
    /* send received radio packet data out via NMEA for debugging */
    if (settings->nmea_d || settings->nmea2_d) {
      if (settings->debug_flags & DEBUG_LEGACY) {
        int16_t ns[4], ew[4];
        int i;
        for (i=0; i<4; i++) {
           ns[i] = ((int16_t) pkt->ns[i]) << smult;
           ew[i] = ((int16_t) pkt->ew[i]) << smult;
        }
        snprintf_P(NMEABuffer, sizeof(NMEABuffer),
          PSTR("$PSRFL,%06X,%ld,%.6f,%.6f,%.1f,%.1f,%.1f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n"),
          fop->addr, (int32_t)fop->gnsstime_ms /* - (int32_t)ThisAircraft.gnsstime_ms */,
          fop->latitude, fop->longitude, fop->altitude,
          course, turnrate, vs10, smult, fop->airborne, unk2,
          ns[0], ns[1], ns[2], ns[3], ew[0], ew[1], ew[2], ew[3]);
        NMEAOutD();
        if (settings->debug_flags & DEBUG_DEEPER) {
          /* also output the raw (but decrypted) packet as a whole, in hex */
          snprintf_P(NMEABuffer, sizeof(NMEABuffer), PSTR("$PSRFB,%06X,%ld,%s\r\n"),
            fop->addr, timestamp,
            bytes2Hex((byte *)pkt, sizeof (legacy_packet_t)));
          NMEAOutD();
        }
      }
    }
#endif

    return true;
}

bool flr_adsl_decode(void *buffer, container_t *this_aircraft, ufo_t *fop) {

    if (dual_protocol==RF_FLR_ADSL) {
        if (RF_last_protocol==RF_PROTOCOL_LEGACY)
            return legacy_decode(buffer, this_aircraft, fop);
        else
            return adsl_decode(buffer, this_aircraft, fop);
    }
    return false;
}

// fill in the data fields in the new 2024 protocol packet
size_t latest_encode(void *pkt_buffer, container_t *aircraft)
{
    latest_packet_t *pkt = (latest_packet_t *) pkt_buffer;
    // addr & addr_type already pre-filled from legacy_encode()

    uint32_t timestamp = (uint32_t) RF_time;   // incremented in RF.cpp 300 ms after PPS

#if 0
    uint32_t now_ms = millis();
    if (ref_time_ms > 0 && now_ms >= ref_time_ms + 1000) {
      OurTime += 1;
      ref_time_ms += 1000;
    }
    uint32_t timestamp = (uint32_t) OurTime;
    if (now_ms < ref_time_ms + 300)
        --timestamp;
    if (timestamp != (uint32_t) RF_time) {
Serial.printf("RF_time=%d but should be %d\r\n", (uint32_t) RF_time, timestamp);
        return 0;
    }
#endif

    pkt->timebits = (timestamp & 0x0F);

    pkt->msg_type = 2;

    pkt->stealth = aircraft->stealth;
    //if (aircraft->tx_type < TX_TYPE_FLARM)     // if relaying ADS-B
    //    pkt->no_track = 1;                     // then hide from OGN - alas then also rarely reported by FLARM
    //else
        pkt->no_track = aircraft->no_track;

    uint8_t aircraft_type = aircraft->aircraft_type;
    if (aircraft == &ThisAircraft) {                    // not relaying some other aircraft
      if (landed_out_mode) {
          aircraft_type = AIRCRAFT_TYPE_UNKNOWN;        // mark this aircraft as landed-out
          if (test_mode)
              pkt->addr_type |= 4;   // >>> as if relayed - for testing how FLARM reacts
      }
    } else {
      if (aircraft_type == AIRCRAFT_TYPE_UNKNOWN && aircraft->airborne == 0) {
           // relaying a landed-out SoftRF device
           if (settings->relay > RELAY_ONLY + 2)          // testing with relay,6
               aircraft_type = AIRCRAFT_TYPE_GLIDER;      // maybe more FLARM-compatible
      }
    }
    if (aircraft_type == AIRCRAFT_TYPE_WINCH) {
        aircraft_type = AIRCRAFT_TYPE_STATIC;
        pkt->airborne = 2;
    } else if (aircraft->airborne == 0) {
        pkt->airborne = 1;
    } else if (aircraft->circling != 0 && fabs(aircraft->turnrate) > 6.0) {
        pkt->airborne = 3;
    } else {
        pkt->airborne = 2;
    }
    if (do_alarm_demo)
        pkt->airborne = 2;
    pkt->aircraft_type = aircraft_type;

    float lat = aircraft->latitude;
    float lon = aircraft->longitude;

    if (lat < 0.0)
        pkt->lat = (uint32_t) (-(((int32_t) (-lat * 1e7) + 26) / 52)) & 0x0FFFFF;
    else
        pkt->lat = (((uint32_t) (lat * 1e7) + 26) / 52) & 0x0FFFFF;
    int d = londiv((int)fabs(lat));
    if (lon < 0.0)
        pkt->lon = (uint32_t) (-(((int32_t) (-lon * 1e7) + (d>>1)) / d)) & 0x0FFFFF;
    else
        pkt->lon = (((uint32_t) (lon * 1e7) + (d>>1)) / d) & 0x0FFFFF;

    int32_t alt = (int32_t) aircraft->altitude;    // was   + ThisAircraft.geoid_separation
    pkt->alt = enscale(alt+1000,12,1,0);  // 13 bits total, unsigned (with offset)

    int32_t turnrate = (int32_t) (20.0 * aircraft->turnrate);
    pkt->turnrate = enscale(turnrate,6,2,1);  // 9 bits total

    int32_t speed10 = (int32_t) (aircraft->speed * (10.0 * _GPS_MPS_PER_KNOT));
    pkt->speed = enscale(speed10,8,2,0);  // 10 bits total (unsigned)

    int32_t icourse = (int32_t) aircraft->course;
    if (icourse <   0)  icourse += 360;
    if (icourse > 360)  icourse -= 360;
    pkt->course = (icourse << 1) & 0x03FF;  // 10 bits total (unsigned, not scaled)

    int32_t vs10 = (int32_t) (aircraft->vs * (10.0 / (_GPS_FEET_PER_METER * 60.0)));
    pkt->vs = enscale(vs10,6,2,1);  // 9 bits total

    // need to fake something in the gps fields
    //pkt->gpsA = enscale(10*aircraft->gpsA,3,3,0);
    //pkt->gpsB = enscale(((aircraft->gpsB) << 2),2,3,0);
    pkt->gpsA = 0b010010;   // 3
    pkt->gpsB = 0b01010;   // 5
    pkt->unk8 = 11;  // fake

    pkt->needs3 = 3;
    pkt->has3 = 3;
    pkt->b1 = 0;
    pkt->b2 = 0;
    pkt->b3 = 0;
    pkt->c1 = 0;
    pkt->c2 = 0;
    pkt->lastbyte = 0;
    pkt->_unk1 = 0;

#if 0
    if (settings->nmea_d || settings->nmea2_d) {
      if (settings->debug_flags & DEBUG_LEGACY) {
        if (settings->debug_flags & DEBUG_DEEPER) {
          /* output the un-ecrypted packet as a whole, in hex */
          snprintf_P(NMEABuffer, sizeof(NMEABuffer),
            PSTR("$PSRRB,%ld,%06X,%s\r\n"),
            timestamp, aircraft->addr,
            bytes2Hex((byte *)pkt_buffer, sizeof (latest_packet_t)));
          NMEAOutD();
        }
      }
    }
#endif

    /* encrypt packet */
    uint32_t *wp = (uint32_t *) pkt_buffer;
    scramble(wp, timestamp);
    btea2(wp, true);

#if 0
    if (settings->nmea_d || settings->nmea2_d) {
      if (settings->debug_flags & DEBUG_LEGACY) {
        /* output the encrypted packet as a whole, in hex */
        snprintf_P(NMEABuffer, sizeof(NMEABuffer),
          PSTR("$PSRRR,%ld,%06X,%s\r\n"),
          timestamp, aircraft->addr,
          bytes2Hex((byte *)pkt_buffer, sizeof (latest_packet_t)));
        NMEAOutD();
      }
    }
#endif

    return (sizeof(latest_packet_t));
}

size_t legacy_encode(void *pkt_buffer, container_t *aircraft)
{
    legacy_packet_t *pkt = (legacy_packet_t *) pkt_buffer;

    uint32_t id = aircraft->addr;
    pkt->addr = id & 0x00FFFFFF;

    bool relay = (aircraft != &ThisAircraft);  // aircraft is some other aircraft
    bool relay_landed_out = (relay && aircraft->aircraft_type == AIRCRAFT_TYPE_UNKNOWN);

    uint8_t addr_type;
    if (relay) {
        addr_type = aircraft->addr_type;
        // relayed ADS-B will not be relayed again even without the mark
        // but marking it for SoftRF reception will show callsign as ADS_xxxxxx
        if (relay_landed_out) {
            if (settings->relay < RELAY_ONLY + 2)
                addr_type |= 4;                     // this marks it as a relayed packet
        } else if (settings->relay < RELAY_ONLY) {
             addr_type |= 4;
        }
        // for RELAY_ONLY don't mark, intended for reception by Classic FLARMs
    } else {
        addr_type = settings->id_method;
        if (addr_type == ADDR_TYPE_OVERRIDE)
            addr_type = ADDR_TYPE_FLARM;
        if (addr_type == ADDR_TYPE_FLARM && settings->rf_protocol == RF_PROTOCOL_OGNTP)
            addr_type = ADDR_TYPE_OGN;  // since main protocol is OGNTP & using device ID
    }
    pkt->addr_type = addr_type;      // ICAO or FLARM (or random)

    // relay in old protocol unless relaying landed-out traffic or in relay-only mode
    // - IOW, relay ADS-B traffic in old protocol for relay-all (visible to SoftRF),
    //   but in new protocol for relay-only (visible to Classic FLARM)
    if (current_RF_protocol == RF_PROTOCOL_LATEST &&
            (!relay || relay_landed_out || (settings->relay >= RELAY_ONLY)))
        return latest_encode(pkt_buffer, aircraft);

    int ndx;
    uint8_t pkt_parity;
    uint32_t key[4];

    float lat = aircraft->latitude;
    float lon = aircraft->longitude;
    int16_t alt = (int16_t) aircraft->altitude;    // was   + ThisAircraft.geoid_separation

    float course = aircraft->course;
    float speedf = aircraft->speed * _GPS_MPS_PER_KNOT; /* m/s */
    float vsf = aircraft->vs * (1.0 / (_GPS_FEET_PER_METER * 60.0)); /* m/s */

    uint16_t speed4 = (uint16_t) roundf(speedf * 4.0f);
    uint8_t smult;
    if        (speed4 & 0x200) {
      smult = 3;
    } else if (speed4 & 0x100) {
      smult = 2;
    } else if (speed4 & 0x080) {
      smult = 1;
    } else {
      smult = 0;
    }
    pkt->smult = smult;

      /* Compute NS & EW speed components for future time points. */
      if (relay)
          project_that(aircraft);
      else
          project_this(aircraft);       /* which also calls airborne() */
      if (do_alarm_demo)
          pkt->airborne = 1;
      else
          pkt->airborne = aircraft->airborne;
      int16_t vs10;
      if (aircraft->airborne) {
         for (int i=0; i<4; i++) {
             pkt->ns[i] = (int8_t) (aircraft->fla_ns[i] >> smult);
             pkt->ew[i] = (int8_t) (aircraft->fla_ew[i] >> smult);
             // quarter-meters per sec if smult==0
         }
         vs10 = (int16_t) roundf(vsf * 10.0f);
      } else {
         // >>> trying to avoid FLARM warnings about parked SoftRF devices
         for (int i=0; i<4; i++) {
             pkt->ns[i] = 0;
             pkt->ew[i] = 0;
         }
         vs10 = 0;
      }

/*  pkt->vs = aircraft->stealth ? 0 : vs10 >> pkt->smult; */
/*  - that degrades collision avoidance - should only mask vs in NMEA */
    pkt->vs = vs10 >> smult;

    pkt->stealth = aircraft->stealth;
    //if (aircraft->tx_type < TX_TYPE_FLARM)     // if relaying ADS-B
    //    pkt->no_track = 1;                     // then hide from OGN - alas then also rarely reported by FLARM
    //else
        pkt->no_track = aircraft->no_track;

    uint8_t aircraft_type = aircraft->aircraft_type;
    if (landed_out_mode)
        aircraft_type = AIRCRAFT_TYPE_UNKNOWN;    // marking ourself as landed-out
        // pkt->_unk1 = 1;
    //else if (relay_landed_out)
        //aircraft_type = AIRCRAFT_TYPE_GLIDER;     // assumption
    else if (aircraft_type == AIRCRAFT_TYPE_WINCH) {
        aircraft_type = AIRCRAFT_TYPE_STATIC;
        pkt->airborne = 1;
    }
    pkt->aircraft_type = aircraft_type;

    pkt->gps = 323;

    /* project position 2 seconds into future, as it seems that FLARM does that */
    if (aircraft_type != AIRCRAFT_TYPE_STATIC) {
        course += aircraft->turnrate;     // average course over the next 2 seconds
        float offset = speedf * (2.0 / 111300.0);   // degslat/sec * 2 sec = degs moved
        lat += (offset * cos(D2R * course));
        lon += (offset * sin(D2R * course) * InvCosLat());
    }

    // this section revised by MB on 220526
    if (lat < 0.0)
        pkt->lat = (uint32_t) (-(((int32_t) (-lat * 1e7) + 0x40) >> 7)) & 0x07FFFF;
    else
        pkt->lat = (((uint32_t) (lat * 1e7) + 0x40) >> 7) & 0x07FFFF;
    if (lon < 0.0)
        pkt->lon = (uint32_t) (-(((int32_t) (-lon * 1e7) + 0x40) >> 7)) & 0x0FFFFF;
    else
        pkt->lon = (((uint32_t) (lon * 1e7) + 0x40) >> 7) & 0x0FFFFF;

    if (alt < 0) {
        alt = 0;    // cannot be negative
    } else {
      if (alt >= (1<<13))
          alt = (1<<13)-1; // clamp to Max
    }
    pkt->alt = alt;

    // assume unk2 signals established circling
    if (aircraft->circling > 0)
        pkt->_unk2 = 0;
    else if (aircraft->circling < 0)
        pkt->_unk2 = 3;
    else
        pkt->_unk2 = 1;

    pkt->msg_type = 0;
    pkt->_unk1 = 0;
    pkt->_unk3 = 0;
//    pkt->_unk4 = 0;

//    if (relay
//    &&  (aircraft->protocol==RF_PROTOCOL_ADSB_1090 || aircraft->protocol==RF_PROTOCOL_GDL90)) {
//        pkt->no_track = 1;   // so that OGN ground stations will not report it
//    }

    pkt->parity = 0;
    for (ndx = 0; ndx < sizeof (legacy_packet_t); ndx++) {
      pkt_parity += parity(*(((unsigned char *) pkt) + ndx));
    }

    //pkt->parity = (pkt_parity % 2);
    pkt->parity = (pkt_parity & 0x01);

    /* encrypt packet */

    //uint32_t timestamp = (uint32_t) aircraft->timestamp;
    uint32_t timestamp = (uint32_t) RF_time;   // incremented in RF.cpp 300 ms after PPS
    make_key(key, timestamp , (pkt->addr << 8) & 0xffffff);
    uint32_t *wp = (uint32_t *) pkt_buffer;
    btea(wp+1, 5, key);

    return (sizeof(legacy_packet_t));
}
