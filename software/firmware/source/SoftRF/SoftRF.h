/*
 * SoftRF.h
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

// this version incorporates some things from main branch v1.1.

#ifndef SOFTRF_H
#define SOFTRF_H

#define SOFTRF

#if defined(ARDUINO) || defined(HACKRF_ONE)
#include <Arduino.h>
#endif /* ARDUINO */

#if defined(ENERGIA_ARCH_CC13XX) || defined(ENERGIA_ARCH_CC13X2) || defined(HACKRF_ONE) || defined(ARDUINO_ARCH_AVR)
#include <TimeLib.h>
#endif /* ENERGIA_ARCH_CC13XX || ENERGIA_ARCH_CC13X2 || defined(HACKRF_ONE) */

#if defined(RASPBERRY_PI)
#include <raspi/raspi.h>
#endif /* RASPBERRY_PI */

#define SOFTRF_FIRMWARE_VERSION "MB171"
#define SOFTRF_IDENT            "SoftRF"
#define SOFTRF_USB_FW_VERSION   0x0101

#define ENTRY_EXPIRATION_TIME  30 /* seconds - was 17 - NMEA export limited further by 'expire' setting */
#define NONDIR_EXPIRATION       5 /* seconds */
#define EXPORT_EXPIRATION_TIME  5 /* seconds - PFLAA export cutoff - default value for 'expire' setting */
#define ENTRY_RELAY_TIME       15 /* seconds - in normal mode, how often to relay same aircraft */
#define RELAY_ONLY_TIME         2 /* seconds - in relay-only mode, how often to relay same aircraft */
#define ANY_RELAY_TIME          5 /* seconds - in normal mode, how often to relay any aircraft */
#define LED_EXPIRATION_TIME     5 /* seconds */

/*
 * If you need for SoftRF to operate in wireless
 * client mode - specify your local AP's SSID/PSK:
 *
 * #define MY_ACCESSPOINT_SSID "My_AP_SSID"
 * #define MY_ACCESSPOINT_PSK  "My_AP_PSK"
 *
 * If SoftRF's built-in AP is not stable enough for you, consider
 * to use "reverse" operation when your smartphone is acting
 * as an AP for the SoftRF unit as a client:
 *
 * #define MY_ACCESSPOINT_SSID "AndroidAP"
 * #define MY_ACCESSPOINT_PSK  "12345678"
 */

// Default mode is AP with
// SSID: SoftRF-XXXXXX
// KEY:  12345678
// IP: 192.168.1.1
// NETMASK: 255.255.255.0

// defaults for station (client) mode
//   - oriented towards use with XCVario:
#define MY_ACCESSPOINT_SSID "XCVario-1234"
#define MY_ACCESSPOINT_PSK  "xcvario-21"

#define RELAY_DST_PORT  12390
#define RELAY_SRC_PORT  (RELAY_DST_PORT - 1)

#define GDL90_DST_PORT    4000
#define NMEA_TCP_PORT     2000
#define NMEA_UDP_PORT     10110  // default local port to listen for UDP packets
#define NMEA_UDP_PORT2    10111  // alternative NMEA output port
#define ALT_UDP_PORT      4352   // input port if NMEA UDP output is using NMEA_UDP_PORT
#define ALT_TCP_PORT      8880             // XCvario
#define NMEA_TCP_IP       "192.168.4.1"    // XCVario

/*
 * Serial I/O default values.
 * Can be overridden by platfrom-specific code.
 */
#if !defined(SERIAL_IN_BR)
/*
 * 9600 is default value of NMEA baud rate
 * for most of GNSS modules
 * being used in SoftRF project
 */
#define SERIAL_IN_BR      9600
#endif
#if !defined(SERIAL_IN_BITS)
#define SERIAL_IN_BITS    SERIAL_8N1
#endif

/*
 * 38400 is known as maximum baud rate
 * that HC-05 Bluetooth module
 * can handle without symbols loss.
 *
 * Applicable for Standalone Edition. Inherited by most of other SoftRF platforms.
 */
#define STD_OUT_BR        38400
#define STD_OUT_BITS      SERIAL_8N1

#if !defined(SERIAL_OUT_BR)
#define SERIAL_OUT_BR     STD_OUT_BR
#endif
#if !defined(SERIAL_OUT_BITS)
#define SERIAL_OUT_BITS   STD_OUT_BITS
#endif

#define UAT_RECEIVER_BR   2000000

#if defined(PREMIUM_PACKAGE) && !defined(RASPBERRY_PI)
#define ENABLE_AHRS
#endif /* PREMIUM_PACKAGE */

typedef struct CONTAINER {

    uint8_t   protocol;
    uint8_t   tx_type;
    uint8_t   addr_type;
    int8_t    alarm_level;
    int8_t    alert_level;

    time_t    timestamp;      // seconds (unix epoch)
    time_t    timerelayed;
    uint32_t  addr;
    float     latitude;      // signed decimal-degrees
    float     longitude;
    float     altitude;      // meters, changed to store altitude above ellipsoid
    float     geoid_separation; // meters = Height of the Geoid (mean sea level) above the Ellipsoid
    float     pressure_altitude;
    float     baro_alt_diff;    // only from ADS-B <<< check units & sign
    float     course;     /* CoG */   // degrees
    float     heading;    /* where the nose points = course - wind drift */
    float     speed;      /* ground speed in knots */
    float     vs;         /* feet per minute vertical speed */

    /* to be able to compute turn & climb rates */
    uint32_t  gnsstime_ms;    /* hopefully a more precise timestamp */
    uint32_t  prevtime_ms;    /* preceding timestamp */
    uint32_t  projtime_ms;    /* timestamp of last course projection */
    float     prevcourse;     /* previous course */
    float     prevheading;    /* previous heading */
/*  float     prevspeed;  */  /* previous speed */
    float     prevaltitude;   /* previous altitude */
    float     distance;       // meters
    float     mindist;
    float     bearing;
    float     turnrate;       // ground reference
    float     alt_diff;
    float     maxrssirelalt;
    float     adj_alt_diff;
    float     adj_distance;

    // projections in air reference frame for "Legacy" collision prediction
    int16_t   air_ns[6];
    int16_t   air_ew[6];

    /* 'legacy' specific data */
    int16_t   fla_ns[4];     // quarter-meters per second
    int16_t   fla_ew[4];
    int32_t   dx;        // EW distance to this other aircraft, in meters
    int32_t   dy;        // NS distance
//  uint8_t   msg_type;  // 2 = new 2024 protocol
    bool      stealth;
    bool      no_track;
    bool      relayed;    // has already been relayed one hop
    uint8_t   aircraft_type;
    uint8_t   airborne;
    int8_t    circling;   // 1=right, -1=left

    uint8_t   next;       // for linking into a list
    uint8_t   alert;      /* bitmap of issued voice/tone/ble/... alerts */

    int16_t   RelativeHeading;    // for voice and strobe

    uint16_t  hdop; /* cm */
    uint16_t  last_crc;
    int8_t    rssi;
    int8_t    mindistrssi;
    int8_t    maxrssi;

    /* ADS-B (ES, UAT, GDL90) specific data */
    uint8_t   callsign[10];    /* size of mdb.callsign + 1 */
    uint32_t  positiontime;
    uint32_t  velocitytime;
    uint32_t  mode_s_time;

} container_t;

// only the fields needed for processing incoming radio messages
// - made this smaller than 'container' since it is copied over and over
typedef struct UFO {
//#if defined(RASPBERRY_PI) || defined(ARDUINO_ARCH_NRF52)
#if defined(RASPBERRY_PI)
    uint8_t   raw[34];
#endif
    uint32_t  addr;
    float     latitude;
    float     longitude;
    float     altitude;
    float     pressure_altitude;
    time_t    timestamp;
    uint32_t  gnsstime_ms;
    float     speed;
    float     course;
    float     turnrate;
    float     vs;
    uint16_t  hdop;
    uint16_t  last_crc;
    // those below can be bit-packed into a smaller space:
    uint8_t   protocol;         // needs 4 bits
    uint8_t   tx_type;          // needs 3+ bits
    uint8_t   addr_type;        // needs 3 bits
    uint8_t   aircraft_type;    // needs 4 bits
    uint8_t   airborne;         // needs 1 bit
    int8_t    circling;         // needs 2 bits (signed)
    bool      stealth;          // needs 1 bit
    bool      no_track;         // needs 1 bit
    bool      relayed;          // needs 1 bit
} ufo_t;

// only the fields needed for processing ADS-B messages:
typedef struct ADSBFO {
    uint32_t  addr;
    float     latitude;
    float     longitude;
    float     altitude;    // meters
    float     distance;    // meters
    float     bearing;
    int32_t   dx;          // EW distance to this other aircraft, in meters
    int32_t   dy;          // NS distance
    uint8_t   tx_type;
    // if the following are added can remove all reference to "mm" at the "fo1090" level:
    //int8_t    rssi;
    //int8_t    alt_type;
} adsfo_t;

typedef struct hardware_info {
    byte  model;
    byte  revision;
    byte  soc;
    byte  rf;
    byte  gnss;
    byte  baro;
    byte  display;
    byte  storage;
    byte  rtc;
    byte  imu;
    byte  mag;
    byte  pmu;
} hardware_info_t;

typedef struct IODev_ops_struct {
  const char name[16];
  void (*setup)();
  void (*loop)();
  void (*fini)();
  int (*available)(void);
  int (*read)(void);
  size_t (*write)(const uint8_t *buffer, size_t size);
} IODev_ops_t;

typedef struct DB_ops_struct {
  bool (*setup)();
  bool (*fini)();
  bool (*query)(uint8_t, uint32_t, char *, size_t);
} DB_ops_t;

enum
{
	SOFTRF_MODE_NORMAL,
	SOFTRF_MODE_WATCHOUT,
	SOFTRF_MODE_BRIDGE,
	SOFTRF_MODE_RELAY,
	SOFTRF_MODE_TXRX_TEST,
	SOFTRF_MODE_LOOPBACK,
	SOFTRF_MODE_UAV,
	SOFTRF_MODE_RECEIVER,
	SOFTRF_MODE_CASUAL,
	SOFTRF_MODE_GPSBRIDGE,
	SOFTRF_MODE_MORENMEA     // obsolete
};

enum
{
	SOFTRF_MODEL_UNKNOWN,
	SOFTRF_MODEL_STANDALONE,
	SOFTRF_MODEL_PRIME,
	SOFTRF_MODEL_UAV,
	SOFTRF_MODEL_PRIME_MK2,
	SOFTRF_MODEL_RASPBERRY,
	SOFTRF_MODEL_UAT,
	SOFTRF_MODEL_SKYVIEW,
	SOFTRF_MODEL_RETRO,
	SOFTRF_MODEL_SKYWATCH,
	SOFTRF_MODEL_DONGLE,
	SOFTRF_MODEL_OCTAVE,
	SOFTRF_MODEL_UNI,
	SOFTRF_MODEL_WEBTOP_SERIAL,
	SOFTRF_MODEL_MINI,
	SOFTRF_MODEL_BADGE,
	SOFTRF_MODEL_ES,
	SOFTRF_MODEL_BRACELET,
	SOFTRF_MODEL_ACADEMY,
	SOFTRF_MODEL_LEGO,
	SOFTRF_MODEL_WEBTOP_USB,
	SOFTRF_MODEL_PRIME_MK3,
	SOFTRF_MODEL_BALKAN,
};

enum
{
	SOFTRF_SHUTDOWN_NONE,
	SOFTRF_SHUTDOWN_DEFAULT,
	SOFTRF_SHUTDOWN_DEBUG,
	SOFTRF_SHUTDOWN_ABORT,
	SOFTRF_SHUTDOWN_WATCHDOG,
	SOFTRF_SHUTDOWN_NMEA,
	SOFTRF_SHUTDOWN_BUTTON,
	SOFTRF_SHUTDOWN_LOWBAT,
	SOFTRF_SHUTDOWN_EXTPWR,
	SOFTRF_SHUTDOWN_SENSOR,
	SOFTRF_SHUTDOWN_CHARGE
};

enum
{
	STORAGE_NONE,
	STORAGE_FLASH,
	STORAGE_CARD,
	STORAGE_FLASH_AND_CARD,
};

#define STORAGE_SD STORAGE_CARD

enum
{
	IMU_NONE,
	ACC_BMA423,
	ACC_ADXL362,
	IMU_MPU6886,
	IMU_MPU9250,
	IMU_BNO080,
	IMU_ICM20948,
	IMU_QMI8658,
};

enum
{
	MAG_NONE,
	MAG_AK8963,
	MAG_AK09916,
	MAG_IIS2MDC,
	MAG_QMC6310,
};

static inline uint32_t DevID_Mapper(uint32_t id)
{
  // switched to restricting device ID to a 20-bit range
  // to avoid overlapping with any of FLARM ranges
  return (0x800000 | (id & 0x000FFFFF));
}

extern container_t ThisAircraft;
extern hardware_info_t hw_info;
extern const float txrx_test_positions[90][2] PROGMEM;
extern uint32_t SetupTimeMarker; 
extern uint32_t GNSSTimeMarker;

void reboot(void);
void shutdown(int);

#define TXRX_TEST_NUM_POSITIONS (sizeof(txrx_test_positions) / sizeof(float) / 2)
#define TXRX_TEST_ALTITUDE    438.0
#define TXRX_TEST_COURSE      280.0
#define TXRX_TEST_SPEED       50.0
#define TXRX_TEST_VS          -300.0

//#define ENABLE_TTN
//#define ENABLE_BT_VOICE
//#define TEST_PAW_ON_NICERF_SV610_FW466
#define  DO_GDL90_FF_EXT

#define LOGGER_IS_ENABLED 0

#if LOGGER_IS_ENABLED
#define StdOut  LogFile
#else
#define StdOut  Serial
#endif /* LOGGER_IS_ENABLED */

#endif /* SOFTRF_H */
