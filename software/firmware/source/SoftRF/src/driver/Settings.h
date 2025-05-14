/*
 * Settings.h - formerly EEPROMHelper.h
 * Copyright (C) 2016-2021 Linar Yusupov
 * Changed by Moshe Braner 2024 to storing settings in a text file
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

#ifndef SETTINGS_H
#define SETTINGS_H

#include "../../SoftRF.h"

//one of the following needs to be defined in SoftRF.h:  (not used any more)
//#define DEFAULT_REGION_EU
//#define DEFAULT_REGION_US

//#if !defined(DEFAULT_REGION_EU) && !defined(DEFAULT_REGION_US)
//#error No default region defined
//#endif

//#if defined(DEFAULT_REGION_EU) && defined(DEFAULT_REGION_US)
//#error Multiple default regions defined
//#endif

#include "../system/SoC.h"

#if !defined(EXCLUDE_EEPROM)
#if defined(ENERGIA_ARCH_CC13XX) || defined(ENERGIA_ARCH_CC13X2)
#include <EEPROM_CC13XX.h>
#else
#include <EEPROM.h>
#endif /* CC13XX or CC13X2 */
#endif /* EXCLUDE_EEPROM */

#define SOFTRF_EEPROM_MAGIC   0xBABADEDA
#define SOFTRF_EEPROM_VERSION 0xACAC0B0D
#define SOFTRF_SETTINGS_VERSION 1

// the "ui" settings have been appended into "settings"
#define ui settings

enum
{
	EEPROM_EXT_LOAD,
	EEPROM_EXT_DEFAULTS,
	EEPROM_EXT_STORE
};

enum
{
	STROBE_OFF = 0,
	STROBE_ALARM,
	STROBE_AIRBORNE,
	STROBE_ALWAYS
};

//#if !defined(EXCLUDE_VOICE)
//#if defined(ESP32)
enum
{
	VOICE_OFF = 0,
	VOICE_INT,
	VOICE_EXT
};
//#endif
//#endif

enum
{
	DEST_NONE,
	DEST_UART,
	DEST_UDP,
	DEST_TCP,
	DEST_USB,
	DEST_BLUETOOTH,
	DEST_UART2
};

enum
{
    ADSB_RX_NONE = 0,
    ADSB_RX_GNS5892 = 1,
    ADSB_RX_2,
    ADSB_RX_3
};

enum
{
	BAUD_DEFAULT = 0,
	BAUD_4800 = 1,
	BAUD_9600 = 2,
	BAUD_19200 = 3,
	BAUD_38400 = 4,
	BAUD_57600 = 5,
	BAUD_115200 = 6,
	BAUD_2000000 = 7
};

enum
{
	TCP_MODE_SERVER=0,
	TCP_MODE_CLIENT
};

enum
{
	RELAY_OFF=0,
	RELAY_LANDED,
	RELAY_ALL,
	RELAY_ONLY
};

enum
{
	EXT_GNSS_NONE=0,
	EXT_GNSS_39_4,
	EXT_GNSS_13_2,
	EXT_GNSS_15_14
};

enum
{
	SD_CARD_NONE=0,   // SCK, MISO, MOSI, SS
	SD_CARD_13_25,    // SD card on 13,25,2,0
	SD_CARD_13_VP,    // SD card on 13,VP,2,0
	SD_CARD_LORA      // SD card on 5,19,27,0
};

enum
{
	FLIGHT_LOG_NONE=0,
	FLIGHT_LOG_ALWAYS,
	FLIGHT_LOG_AIRBORNE,
	FLIGHT_LOG_TRAFFIC
};

enum stgidx {
    STG_NONE,
    STG_VERSION,
    STG_MODE,
    STG_PROTOCOL,
    STG_ALTPROTOCOL,
    STG_BAND,
    STG_ACFT_TYPE,
    STG_ID_METHOD,
    STG_AIRCRAFT_ID,
    STG_IGNORE_ID,
    STG_FOLLOW_ID,
    STG_ALARM,
    STG_HRANGE,
    STG_VRANGE,
    STG_OLD_TXPWR,    // obsolete
    STG_TXPOWER,      // new coding
    STG_VOLUME,
    STG_POINTER,
//#if defined(ESP32)
    STG_STROBE,
    STG_VOICE,
    STG_OWNSSID,
    STG_EXTSSID,
    STG_PSK,
    STG_HOST_IP,
    STG_TCPMODE,
    STG_TCPPORT,
//#endif
    STG_BLUETOOTH,
    STG_BAUD_RATE,
    STG_NMEA_OUT,
    STG_NMEA_G,
    STG_NMEA_S,
    STG_NMEA_T,
    STG_NMEA_E,
    STG_NMEA_D,
    STG_NMEA_P,
//#if defined(ESP32)
    STG_NMEA_OUT2,
    STG_NMEA2_G,
    STG_NMEA2_S,
    STG_NMEA2_T,
    STG_NMEA2_E,
    STG_NMEA2_D,
    STG_NMEA2_P,
    STG_ALTPIN0,
    STG_BAUDRATE2,
    STG_INVERT2,
    STG_ALT_UDP,
    STG_RX1090,
    STG_RX1090X,
    STG_MODE_S,
    STG_HRANGE1090,
    STG_VRANGE1090,
    STG_GDL90_IN,
//#endif
    STG_GDL90,
    STG_D1090,
    STG_RELAY,
    STG_EXPIRE,
    STG_PFLAA_CS,
    STG_STEALTH,
    STG_NO_TRACK,
    STG_POWER_SAVE,
    STG_POWER_EXT,
    STG_RFC,
    STG_ALARMLOG,
//#if defined(ESP32)
    STG_LOG_NMEA,
    STG_GNSS_PINS,
    STG_PPSWIRE,
    STG_SD_CARD,
//#endif
    STG_LOGFLIGHT,
    STG_LOGINTERVAL,
    STG_COMPFLASH,
    STG_IGC_PILOT,
    STG_IGC_TYPE,
    STG_IGC_REG,
    STG_IGC_CS,
    STG_GEOID,
    STG_LEAPSECS,
  //STG_JSON,
//#if defined(USE_EPAPER)
    STG_EPD_UNITS,
    STG_EPD_ZOOM,
    STG_EPD_ROTATE,
    STG_EPD_ORIENT,
    STG_EPD_ADB,
    STG_EPD_IDPREF,
    STG_EPD_VMODE,
    STG_EPD_AGHOST,
    STG_EPD_TEAM,
//#endif
    STG_DEBUG_FLAGS,
    STG_END
};

enum stgtyp {
    STG_OBSOLETE = -6,   // input-only (to be converted) (data type STG_INT1)
    STG_HIDDEN = -5,     // not visible in web page (data type STG_INT1)
    STG_HEX6  = -4,      // 6 hex digits
    STG_HEX2  = -3,      // 00..FF
    STG_UINT1 = -2,      // 0..255
    STG_INT1  = -1,      // -128..+127
    STG_VOID  = 0,
    STG_STR   = 1        // strings' "type" value equals their length
};

struct setting_struct {
    const char *label;
    char *value;
    int8_t type;
};

typedef struct __attribute__((packed)) PackedSettings {

    uint8_t  mode:4;            // do not move
    uint8_t  rf_protocol:4;     // do not move
    uint8_t  band:4;            // do not move
    uint8_t  txpower:2;         // do not move
    uint8_t  volume:2;
    uint8_t  acft_type;         // can be reduced to 5 bits (not 4!)

    uint8_t  geoid:7;
    bool     resvd1:1;

    bool     nmea_g:1;       // do not move
    bool     nmea_p:1;
    bool     nmea_t:1;
    bool     nmea_s:1;
    bool     nmea_d:1;
    uint8_t  nmea_out:3;     // do not move

    uint8_t  bluetooth:3; // ESP32 built-in Bluetooth  // do not move
    uint8_t  alarm:3;        // do not move
    bool     stealth:1;      // do not move
    bool     no_track:1;     // do not move

    uint8_t  gdl90:3;      // output destination
    uint8_t  d1090:3;
    uint8_t  json:1;       // was 2 bits
    bool     resvd2:1;

    uint8_t  pointer:2;
    uint8_t  power_save:2;
    uint8_t  power_ext:1;    /* if nonzero, shuts down if battery is not full */
    uint8_t  rx1090:2;       // attached ADS-B receiver module    // do not move
    uint8_t  mode_s:1;

    uint8_t  gnss_pins:2;    // external GNSS added to T-Beam     // do not move
    uint8_t  sd_card:2;      // gpio pins for SD card adapter
    uint8_t  logflight:2;
    uint8_t  loginterval:2;

    int8_t   freq_corr; /* +/-, kHz */   // <<< limited to +-30, so can liberate two bits
    uint8_t  relay:2;
    uint8_t  gdl90_in:3;    // data from this port will be interpreted as GDL90
    uint8_t  alt_udp:1;     // if 1 then use 10111 instead of 10110
    bool     nmea_e:1;
    bool     nmea2_e:1;     // whether to send bridged data
    uint8_t  baud_rate:3;   // for serial UART0    // do not move
    uint8_t  baudrate2:3;   // for aux UART2       // do not move
    bool     invert2:1;     // whether to invert the logic levels on UART2
    bool     altpin0:1;     // whether to use a different pin for UART0 RX

    /* encryption key provided by gliding contest organizer */
    uint32_t igc_key[4];

    /* added to allow setting aircraft ID and also an ID to ignore */
    uint32_t aircraft_id:24;  // do not move
    uint8_t  id_method:2;     // device ID, ICAO ID, or random    // do not move
    uint8_t  debug_flags:6;   /* each bit activates output of some debug info */
    uint32_t ignore_id:24;    // do not move
    uint8_t  strobe:2;
    bool    logalarms:1;
    uint8_t  voice:2;
    uint8_t  tcpport:1;       // 0=2000, 1=8880   // do not move
    uint8_t  tcpmode:1;       // do not move
    bool     ppswire:1;       // whether PPS wire added  // do not move
    uint32_t follow_id:24;    // do not move

    bool     nmea2_g:1;       // do not move
    bool     nmea2_p:1;
    bool     nmea2_t:1;
    bool     nmea2_s:1;
    bool     nmea2_d:1;
    uint8_t  nmea_out2:3;     // second NMEA output route    // do not move

    char    ssid[19];         // do not move
    char    psk[17];
    char    host_ip[16];

} settingb_t;

typedef struct Settings {

    int8_t   version;
    uint8_t  mode;
    uint8_t  rf_protocol;
    uint8_t  altprotocol;
    uint8_t  band;
    uint8_t  acft_type;
    uint8_t  alarm;
    uint8_t  hrange;
    uint8_t  vrange;
    int8_t   old_txpwr;  // int not uint to allow making it "obsolete".
    int8_t   txpower;    // int not uint to allow making it "hidden".
    uint8_t  id_method;
    uint32_t aircraft_id;
    uint32_t ignore_id;
    uint32_t follow_id;
    uint8_t  volume;
    uint8_t  pointer;
    uint8_t  bluetooth;     // no effect on T-Echo?  (always BLE, always active?)
    uint8_t  baud_rate;
    uint8_t  nmea_out;
    uint8_t  nmea_g;        // now these are bitfields
    uint8_t  nmea_p;
    uint8_t  nmea_t;
    uint8_t  nmea_s;
    uint8_t  nmea_d;
    uint8_t  nmea_e;
    uint8_t  baudrate2;
    uint8_t  nmea_out2;
    uint8_t  nmea2_g;       // now these are bitfields
    uint8_t  nmea2_p;
    uint8_t  nmea2_t;
    uint8_t  nmea2_s;
    uint8_t  nmea2_d;
    uint8_t  nmea2_e;       // whether to send bridged data
    bool     stealth;
    bool     no_track;
    uint8_t  gdl90;         // output destination
    uint8_t  d1090;
    //uint8_t  json;
    int8_t   geoid;
    int8_t   leapsecs;
    int8_t   freq_corr; /* +/-, kHz */   // <<< limited to +-30
    uint8_t  relay;
    int8_t   expire;
    bool     pflaa_cs;
    bool     logalarms;
    uint32_t debug_flags;   /* each bit activates output of some debug info */

//#if defined(ESP32)
    uint8_t  strobe;
    uint8_t  voice;
    char    myssid[20];    // if using AP mode
    char    ssid[20];      // if connecting to external network
    char    psk[20];
    char    host_ip[16];
    uint8_t  alt_udp;     // if 1 then use 10111 instead of 10110
    uint8_t  tcpmode;
    uint8_t  tcpport;
    uint8_t  power_save;
    uint8_t  power_ext;   /* if nonzero, shuts down if battery is not full */
    uint8_t  rx1090;
    uint8_t  rx1090x;     // settings for the ADS-B receiver module
    int8_t   mode_s;      // 0=off, 1-9="gain"
    uint8_t  hrange1090;  // km
    uint8_t  vrange1090;  // hundreds of meters
    uint8_t  gdl90_in;    // data from this port will be interpreted as GDL90
    uint8_t  gnss_pins;
    bool     ppswire;
    uint8_t  sd_card;      // gpio pins for SD card adapter
    bool     invert2;     // whether to invert the logic levels on UART2
    bool     altpin0;     // whether to use a different pin for UART0 RX
    bool     log_nmea;
//#endif
    uint8_t  logflight;
    uint8_t  loginterval;
    uint8_t  compflash;
    char     igc_pilot[32];
    char     igc_type[20];
    char     igc_reg[12];
    char     igc_cs[8];

    /* encryption key provided by contest organizers */
    uint32_t igc_key[4];

//#if defined(USE_EPAPER)
    // EPD UI settings
    uint8_t  units;
    uint8_t  zoom;
    uint8_t  rotate;
    uint8_t  orientation;
    uint8_t  adb;
    uint8_t  epdidpref;
    uint8_t  viewmode;
    uint8_t  antighost;
    uint32_t team;
//#endif

} settings_t;

typedef struct EEPROM_S {
    uint32_t  magic;
    uint32_t  version;
    settingb_t settings;
    uint32_t  version2;    // guard from both ends
} eeprom_struct_t;

typedef union EEPROM_U {
   eeprom_struct_t field;
   uint8_t raw[sizeof(eeprom_struct_t)];
} eeprom_t;

// bitfields

#define NMEA_BASIC 1

#define NMEA_G 0x000
#define NMEA_G_BASIC (NMEA_G + 1)
#define NMEA_G_GSA   (NMEA_G + 2)
#define NMEA_G_GST   (NMEA_G + 4)
#define NMEA_G_GSV   (NMEA_G + 8)
//#define NMEA_G_10  (NMEA_G + 0x10)
//#define NMEA_G_20  (NMEA_G + 0x20)
//#define NMEA_G_40  (NMEA_G + 0x40)
#define NMEA_G_OTHER (NMEA_G + 0x80)
#define NMEA_G_NONBASIC (NMEA_G + 0xFE)
#define NMEA_G_ALL   (NMEA_G + 0xFF)

#define NMEA_T 0x100
#define NMEA_T_BASIC (NMEA_T + 1)
#define NMEA_T_PROJ  (NMEA_T + 8)

#define NMEA_S 0x200
#define NMEA_S_BASIC (NMEA_S + 1)
#define NMEA_S_LK8   (NMEA_S + 2)
#define NMEA_S_AHRS  (NMEA_S + 4)
#define NMEA_S_ALL   (NMEA_S + 0xFF)

#define NMEA_E 0x300
#define NMEA_E_TUNNEL (NMEA_E + 1)
#define NMEA_E_OUTPUT (NMEA_E + 2)
#define NMEA_E_ALL    (NMEA_E + 0xFF)

#define NMEA_D 0x400
#define NMEA_D_BASIC   (NMEA_D + 1)
#define NMEA_D_ALL     (NMEA_D + 0xFF)
// maybe:
//#define NMEA_D_RADIO   (NMEA_D + 2)
//#define NMEA_D_TRAFFIC (NMEA_D + 4)
//#define NMEA_D_PROJ    (NMEA_D + 8)
//#define NMEA_D_UI      (NMEA_D + 0x10)
//#define NMEA_D_ADSB    (NMEA_D + 0x20)
//#define NMEA_D_40      (NMEA_D + 0x40)
//#define NMEA_D_OTHER   (NMEA_D + 0x80)

#define NMEA_P 0x500
#define NMEA_P_BASIC (NMEA_P + 1)

// also available:
//#define NMEA_6 0x600
//#define NMEA_7 0x700

#define DEBUG_WIND 0x01
#define DEBUG_PROJECTION 0x02
#define DEBUG_ALARM 0x04
#define DEBUG_LEGACY 0x08
#define DEBUG_DEEPER 0x10
// now debug_flags is 24 bits so can have many other specific values
#define DEBUG_SIMULATE 0x800000

void Adjust_Settings(void);
void Settings_setup(void);
void Settings_defaults(bool keepsome);
void EEPROM_store(void);
int  find_setting(const char *label);
bool load_setting(const int index, const char *value);
bool format_setting(const int index, const bool comment, char *buf=NULL, size_t size=0);
void show_settings_serial(void);
void save_settings_to_file(void);
bool load_settings_from_file(void);
const char *settings_message(const char *msg=NULL, const char *submsg=NULL, const int val=0);
void do_test_mode(void);

enum stg_default {
    STG_DEFAULT = 0,
    STG_EEPROM  = 1,
    STG_FILE    = 2
};
extern uint8_t settings_used;

extern settings_t *settings;
extern setting_struct stgdesc[STG_END];
extern const char * stgcomment[STG_END];
extern uint32_t baudrates[];
extern bool do_alarm_demo;
extern bool test_mode;
extern bool landed_out_mode;
extern int8_t geoid_from_setting;

#endif /* SETTINGS_H */
