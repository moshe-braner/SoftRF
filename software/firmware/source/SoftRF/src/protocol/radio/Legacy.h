/*
 * Protocol_Legacy.h
 * Copyright (C) 2014-2015 Stanislaw Pusep
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

#ifndef PROTOCOL_LEGACY_H
#define PROTOCOL_LEGACY_H

// 24 bit SYNC word: 31FAB6
//  add F5 prefix extension to match last 8 bits of the legacy nRF905 device preamble
/*  to support devices that include this extension in their SYNC definition, thus:
/*  IEEE Manchester(F531FAB6) = 55 99 A5 A9 55 66 65 96 */
#define LEGACY_PREAMBLE_TYPE   RF_PREAMBLE_TYPE_55
#define LEGACY_PREAMBLE_SIZE   1
#define LEGACY_SYNCWORD        {0x55, 0x99, 0xA5, 0xA9, 0x55, 0x66, 0x65, 0x96}
#define LEGACY_SYNCWORD_SIZE   8
#define LEGACY_SYNCWORD_SKIP   2     // ignore this many leading SYNC bytes when receiving
#define LEGACY_PAYLOAD_SIZE    24
#define LEGACY_CRC_TYPE        RF_CHECKSUM_TYPE_CCITT_FFFF
#define LEGACY_CRC_SIZE        2

#define LEGACY_AIR_TIME        5 /* in ms */

#define LEGACY_TX_INTERVAL_MIN 600 /* in ms */
#define LEGACY_TX_INTERVAL_MAX 1400

#define LEGACY_KEY1 { 0xe43276df, 0xdca83759, 0x9802b8ac, 0x4675a56b, \
                      0xfc78ea65, 0x804b90ea, 0xb76542cd, 0x329dfa32 }
#define LEGACY_KEY2 0x045d9f3b
#define LEGACY_KEY3 0x87b562f4

/* FTD-12 Version: 7.00 */
enum
{
	ADDR_TYPE_RANDOM,    /* FLARM stealth now */
	ADDR_TYPE_ICAO,
	ADDR_TYPE_FLARM,
	ADDR_TYPE_ANONYMOUS, /* was FLARM stealth, OGN */
	ADDR_TYPE_P3I,       /* not available in id_method settings field */
	ADDR_TYPE_FANET,     /* not available in id_method settings field */
	ADDR_TYPE_6,
	ADDR_TYPE_OVERRIDE   // added into id_method settings field dropdown to override chip ID
};

#define ADDR_TYPE_OGN 3
   // for OGNTP TX packet only - device ID is "ADDR_TYPE_FLARM" (2) elsewhere

enum
{
	AIRCRAFT_TYPE_UNKNOWN,     // marks landed-out
	AIRCRAFT_TYPE_GLIDER,
	AIRCRAFT_TYPE_TOWPLANE,
	AIRCRAFT_TYPE_HELICOPTER,
	AIRCRAFT_TYPE_PARACHUTE,
	AIRCRAFT_TYPE_DROPPLANE,
	AIRCRAFT_TYPE_HANGGLIDER,
	AIRCRAFT_TYPE_PARAGLIDER,
	AIRCRAFT_TYPE_POWERED,
	AIRCRAFT_TYPE_JET,
	AIRCRAFT_TYPE_UFO,         // actually used when unknown
	AIRCRAFT_TYPE_BALLOON,
	AIRCRAFT_TYPE_ZEPPELIN,
	AIRCRAFT_TYPE_UAV,
	AIRCRAFT_TYPE_RESERVED,
	AIRCRAFT_TYPE_STATIC,
	AIRCRAFT_TYPE_WINCH        // added, not in FLARM protocol
};

enum alarm_levels
{
	ALARM_LEVEL_NONE,
	ALARM_LEVEL_CLOSE,     /* no alarm yet, but getting close */
	ALARM_LEVEL_LOW,       /* 13-18 seconds to impact */
	ALARM_LEVEL_IMPORTANT, /*  9-12 seconds to impact */
	ALARM_LEVEL_URGENT     /*   0-8 seconds to impact */
};

enum
{
	ALARM_TYPE_TRAFFIC,
	ALARM_TYPE_SILENT,
	ALARM_TYPE_AIRCRAFT,
	ALARM_TYPE_OBSTACLE
};

enum
{
	GNSS_STATUS_NONE,
	GNSS_STATUS_3D_GROUND,
	GNSS_STATUS_3D_MOVING
};

enum
{
	POWER_STATUS_BAD,
	POWER_STATUS_GOOD
};

enum
{
	TX_STATUS_OFF,
	TX_STATUS_ON
};

typedef struct legacy_packet
{
    /********************/
    unsigned int addr:24;
    unsigned int msg_type:4;
    unsigned int addr_type:3;
    unsigned int _unk1:1;
    // unsigned int magic:8;
    /********************/
    int vs:10;
    unsigned int _unk2:2;
    unsigned int airborne:1;
    unsigned int stealth:1;
    unsigned int no_track:1;
    unsigned int parity:1;
    unsigned int gps:12;
    unsigned int aircraft_type:4;
    /********************/
    unsigned int lat:19;
    unsigned int alt:13;
    /********************/
    unsigned int lon:20;
    unsigned int _unk3:10;
    unsigned int smult:2;
    /********************/
    int8_t ns[4];
    int8_t ew[4];
    /********************/
} __attribute__((packed)) legacy_packet_t;

typedef struct latest_packet
{
    unsigned int addr:24;
    unsigned int msg_type:4;
    unsigned int addr_type:3;
    unsigned int _unk1:1;
    byte b1;
    byte b2;
    byte b3:6;
    uint8_t stealth:1;
    uint8_t no_track:1;
    byte needs3:4;
    byte has3:4;
    byte c1:2;
    byte timebits:4;           // increments each second
    uint16_t aircraft_type:4;  // wider data type to avoid GCC messing the bit offset
    byte c2:1;
    uint16_t alt:13;           // alt+1000, enscaled(12,1,0)
    uint32_t lat:20;
    uint32_t lon:20;
    uint16_t turnrate:9;       // deg/sec times 20, enscaled(6,2,1)
    uint16_t speed:10;         // m/s times 10, enscaled(8,2,0)
    uint16_t vs:9;             // vertical speed, m/s times 10, enscaled(6,2,1)
    uint16_t course:10;        // degrees (0-360) times 2
    byte airborne:2;
    uint16_t gpsA:6;           // GNSS horizontal accuracy, meters times 10, enscaled(3,3)
    uint16_t gpsB:5;           // GNSS vertical accuracy, meters times 4, enscaled(2,3)
    byte unk8:5;
    byte lastbyte;
} __attribute__((packed)) latest_packet_t;

bool legacy_decode(void *, container_t *, ufo_t *);
bool latest_decode(void *, container_t *, ufo_t *);
size_t legacy_encode(void *, container_t *);
size_t latest_encode(void *, container_t *);

unsigned int enscale( int value, unsigned int mbits, unsigned int ebits, unsigned int sbits);
int descale( unsigned int value, unsigned int mbits, unsigned int ebits, unsigned int sbits);

#endif /* PROTOCOL_LEGACY_H */
