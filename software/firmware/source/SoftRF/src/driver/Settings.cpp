/*
 * Settings.cpp - formerly EEPROMHelper.cpp
 * Copyright (C) 2016-2021 Linar Yusupov
 * Settings scheme redesigned by Moshe Braner 2024
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

#include "../system/SoC.h"
#include "Filesys.h"
#include "Settings.h"
#include "RF.h"
#include "LED.h"
#include "EPD.h"
#include "Buzzer.h"
#include "Bluetooth.h"
#include "../TrafficHelper.h"
#include "../protocol/radio/Legacy.h"
#include "../protocol/data/NMEA.h"
#include "../protocol/data/GDL90.h"
#include "../protocol/data/D1090.h"
//#include "../protocol/data/JSON.h"
#include "Battery.h"

settings_t settings_stored;
settings_t *settings;

uint8_t settings_used;
//int settings_file_version = 0;

bool use_eeprom = false;           // set to true if mode & SOFTRF_MODE_EEPROM

bool do_alarm_demo = false;        // activated by middle button on T-Beam
bool landed_out_mode = false;      // activated by button in status web page

bool test_mode = false;            // activated by double-clicking middle button on T-Beam
                                    // - or via web interface, or via $PSRFT
// Upon receiving a $PSRFT NMEA command,
// first the variable test_mode is toggled, then
// this is called, whether test_mode is on or off
// put custom code here for debugging, for example:
//#include "../protocol/radio/ES1090.h"
void do_test_mode()
{
#if defined(ESP32)
//    if (settings->rx1090)
//        gns5892_test_mode();
#endif
}

uint32_t baudrates[8] = 
{
    0,
    4800,
    9600,
    19200,
    38400,
    57600,
    115200,
    0
};

setting_struct stgdesc[STG_END];
const char * stgcomment[STG_END] = {NULL};

struct setting_minmax {
    uint8_t index;
    int8_t min;
    int8_t max;
};
#define NUM_MINMAX 8    // may need to manually enlarge this
setting_minmax stgminmax[NUM_MINMAX];

inline int8_t wifi_only(int8_t stg_type)
{
#if defined(EXCLUDE_WIFI)
    return STG_VOID;
#else
    return stg_type;
#endif
}

inline int8_t epd_only(int8_t stg_type)
{
#if defined(USE_EPAPER)
    return stg_type;
#else
    return STG_VOID;
#endif
}

inline int8_t esp_only(int8_t stg_type)
{
#if defined(ESP32)
    return stg_type;
#else
    return STG_VOID;
#endif
}

#define HIDE_T (1 << SOFTRF_MODEL_PRIME_MK2) // Lilygo T-Beam - has WiFi
#define HIDE_B (1 << SOFTRF_MODEL_BADGE)     // Lilygo T-Echo
#define HIDE_C (1 << SOFTRF_MODEL_CARD)      // Seeed Studios T1000-E
#define HIDE_H (1 << SOFTRF_MODEL_HANDHELD)  // Elecrow Thinknode M1
#define HIDE_P (1 << SOFTRF_MODEL_POCKET)    // Elecrow Thinknode M3 - no file system
#define HIDE_CP  (HIDE_C | HIDE_P)           // no display
#define HIDE_NRF (HIDE_B | HIDE_C | HIDE_H | HIDE_P)

// the first two chars of the label are the shorthand label, rest is the long label:

static void init_stgdesc()
{
  stgdesc[STG_VERSION]    = { "srSoftRF",     (char*)&settings->version,    STG_UINT1, STG_HIDDEN };
  stgdesc[STG_MODE]       = { "mdmode",       (char*)&settings->mode,       STG_UINT1, 0 /* HIDE_NRF */ };
  stgdesc[STG_PROTOCOL]   = { "pcprotocol",   (char*)&settings->rf_protocol,STG_UINT1, 0 };
  stgdesc[STG_ALTPROTOCOL]= { "apaltprotocol",(char*)&settings->altprotocol,STG_UINT1, 0 };
  stgdesc[STG_FLR_ADSL]   = { "faflr_adsl",   (char*)&settings->flr_adsl,   STG_UINT1, 0 };
  stgdesc[STG_BAND]       = { "bdband",       (char*)&settings->band,       STG_UINT1, 0 };
  stgdesc[STG_ACFT_TYPE]  = { "atacft_type",  (char*)&settings->acft_type,  STG_UINT1, 0 };
  stgdesc[STG_ID_METHOD]  = { "imid_method",  (char*)&settings->id_method,  STG_UINT1, 0 };
  stgdesc[STG_AIRCRAFT_ID]= { "aiaircraft_id",(char*)&settings->aircraft_id,STG_HEX6, 0 };
  stgdesc[STG_IGNORE_ID]  = { "iiignore_id",  (char*)&settings->ignore_id,  STG_HEX6, 0 };
  stgdesc[STG_FOLLOW_ID]  = { "fifollow_id",  (char*)&settings->follow_id,  STG_HEX6, 0 };
  stgdesc[STG_ALARM]      = { "alalarm",      (char*)&settings->alarm,      STG_UINT1, 0 };
  stgdesc[STG_HRANGE]     = { "hrhrange",     (char*)&settings->hrange,     STG_UINT1, 0 };
  stgdesc[STG_VRANGE]     = { "vrvrange",     (char*)&settings->vrange,     STG_UINT1, 0 };
  stgdesc[STG_OLD_TXPWR]  = { "tztxpower",    (char*)&settings->old_txpwr,  STG_OBSOLETE, 0 }; // old label for old coding
  stgdesc[STG_TXPOWER]    = { "txtx_power",   (char*)&settings->txpower,    STG_INT1, 0 };   // new label for new coding
  stgdesc[STG_VOLUME]     = { "bzvolume",     (char*)&settings->volume,     STG_UINT1, 0 };
  stgdesc[STG_POINTER]    = { "popointer",    (char*)&settings->pointer,    STG_UINT1, 0 };
  stgdesc[STG_STROBE]     = { "sbstrobe",     (char*)&settings->strobe,     esp_only(STG_UINT1), 0 };
  stgdesc[STG_VOICE]      = { "vcvoice",      (char*)&settings->voice,      esp_only(STG_UINT1), 0 };
  stgdesc[STG_OWNSSID]    = { "mymyssid",     settings->myssid,      wifi_only(sizeof(settings->myssid)), 0 };
  stgdesc[STG_EXTSSID]    = { "ssssid",       settings->ssid,        wifi_only(sizeof(settings->ssid)), 0 };
  stgdesc[STG_PSK]        = { "pwpsk",        settings->psk,         wifi_only(sizeof(settings->psk)), 0 };
  stgdesc[STG_HOST_IP]    = { "iphost_ip",    settings->host_ip,     wifi_only(sizeof(settings->host_ip)), 0 };
  stgdesc[STG_TCPMODE]    = { "mttcpmode",    (char*)&settings->tcpmode,    wifi_only(STG_UINT1), 0 };
  stgdesc[STG_TCPPORT]    = { "tptcpport",    (char*)&settings->tcpport,    wifi_only(STG_UINT1), 0 };
#if defined(ARDUINO_ARCH_NRF52)
  stgdesc[STG_BLUETOOTH]  = { "btbluetooth",  (char*)&settings->bluetooth,  STG_UINT1, STG_SAVEHIDE };
#else
  stgdesc[STG_BLUETOOTH]  = { "btbluetooth",  (char*)&settings->bluetooth,  STG_UINT1, 0 };
#endif
  stgdesc[STG_BAUD_RATE]  = { "brbaud_rate",  (char*)&settings->baud_rate,  STG_UINT1, 0 };
  stgdesc[STG_NMEA_OUT]   = { "n1nmea_out",   (char*)&settings->nmea_out,   STG_UINT1, 0 };
  stgdesc[STG_NMEA_G]     = { "g1nmea_g",     (char*)&settings->nmea_g,     STG_HEX2, 0 };
  stgdesc[STG_NMEA_P]     = { "p1nmea_p",     (char*)&settings->nmea_p,     STG_HEX2, STG_HIDDEN };
  stgdesc[STG_NMEA_T]     = { "t1nmea_t",     (char*)&settings->nmea_t,     STG_HEX2, 0 };
  stgdesc[STG_NMEA_S]     = { "s1nmea_s",     (char*)&settings->nmea_s,     STG_HEX2, 0 };
  stgdesc[STG_NMEA_D]     = { "d1nmea_d",     (char*)&settings->nmea_d,     STG_HEX2, 0 };
  stgdesc[STG_NMEA_E]     = { "e1nmea_e",     (char*)&settings->nmea_e,     STG_HEX2, 0 };
  stgdesc[STG_NMEA_OUT2]  = { "n2nmea_out2",  (char*)&settings->nmea_out2,  STG_UINT1, 0 };
  stgdesc[STG_NMEA2_G]    = { "g2nmea2_g",    (char*)&settings->nmea2_g,    STG_HEX2, 0 };
  stgdesc[STG_NMEA2_P]    = { "p2nmea2_p",    (char*)&settings->nmea2_p,    STG_HEX2, STG_HIDDEN };
  stgdesc[STG_NMEA2_T]    = { "t2nmea2_t",    (char*)&settings->nmea2_t,    STG_HEX2, 0 };
  stgdesc[STG_NMEA2_S]    = { "s2nmea2_s",    (char*)&settings->nmea2_s,    STG_HEX2, 0 };
  stgdesc[STG_NMEA2_D]    = { "d2nmea2_d",    (char*)&settings->nmea2_d,    STG_HEX2, 0 };
  stgdesc[STG_NMEA2_E]    = { "e2nmea2_e",    (char*)&settings->nmea2_e,    STG_HEX2, 0 };
  stgdesc[STG_ALTPIN0]    = { "a0altpin0",    (char*)&settings->altpin0,    esp_only(STG_UINT1), 0 };
  stgdesc[STG_BAUDRATE2]  = { "b2baudrate2",  (char*)&settings->baudrate2,  esp_only(STG_UINT1), 0 };
  stgdesc[STG_INVERT2]    = { "i2invert2",    (char*)&settings->invert2,    esp_only(STG_UINT1), 0 };
  stgdesc[STG_ALT_UDP]    = { "udalt_udp",    (char*)&settings->alt_udp,    wifi_only(STG_UINT1), 0 };
  stgdesc[STG_RX1090]     = { "r9rx1090",     (char*)&settings->rx1090,     esp_only(STG_UINT1), 0 };
  stgdesc[STG_RX1090X]    = { "x9rx1090x",    (char*)&settings->rx1090x,    esp_only(STG_UINT1), 0 };
  stgdesc[STG_MODE_S]     = { "msmode_s",     (char*)&settings->mode_s,     esp_only(STG_INT1), 0 };
  stgdesc[STG_HRANGE1090] = { "h9hrange1090", (char*)&settings->hrange1090, esp_only(STG_UINT1), 0 };
  stgdesc[STG_VRANGE1090] = { "v9vrange1090", (char*)&settings->vrange1090, esp_only(STG_UINT1), 0 };
  stgdesc[STG_GDL90_IN]   = { "9igdl90_in",   (char*)&settings->gdl90_in,   esp_only(STG_UINT1), 0 };
  stgdesc[STG_GDL90]      = { "90gdl90",      (char*)&settings->gdl90,      STG_UINT1, 0 };
  stgdesc[STG_D1090]      = { "d9d1090",      (char*)&settings->d1090,      STG_UINT1, 0 };
  stgdesc[STG_RELAY]      = { "ryrelay",      (char*)&settings->relay,      STG_UINT1, 0 };
  stgdesc[STG_EXPIRE]     = { "exexpire",     (char*)&settings->expire,     STG_INT1, 0 };
  stgdesc[STG_PFLAA_CS]   = { "acpflaa_cs",   (char*)&settings->pflaa_cs,   STG_UINT1, 0 };
  stgdesc[STG_STEALTH]    = { "ststealth",    (char*)&settings->stealth,    STG_UINT1, 0 };
  stgdesc[STG_NO_TRACK]   = { "ntno_track",   (char*)&settings->no_track,   STG_UINT1, 0 };
  stgdesc[STG_POWER_SAVE] = { "pspower_save", (char*)&settings->power_save, STG_UINT1, 0 };
  stgdesc[STG_POWER_EXT]  = { "pxpower_ext",  (char*)&settings->power_ext,  STG_INT1, 0 };
  stgdesc[STG_RFC]        = { "fcrfc",        (char*)&settings->freq_corr,  STG_INT1, 0 };
  stgdesc[STG_ALARMLOG]   = { "agalarmlog",   (char*)&settings->logalarms,  STG_UINT1, HIDE_P };  // no file system
  stgdesc[STG_LOG_NMEA]   = { "nglog_nmea",   (char*)&settings->log_nmea,   esp_only(STG_UINT1), 0 };
  stgdesc[STG_GNSS_PINS]  = { "gpgnss_pins",  (char*)&settings->gnss_pins,  esp_only(STG_UINT1), 0 };
  stgdesc[STG_PPSWIRE]    = { "ppppswire",    (char*)&settings->ppswire,    esp_only(STG_UINT1), 0 };
  stgdesc[STG_SD_CARD]    = { "sdsd_card",    (char*)&settings->sd_card,    esp_only(STG_UINT1), 0 };
  stgdesc[STG_LOGFLIGHT]  = { "lglogflight",  (char*)&settings->logflight,  STG_UINT1, HIDE_P };  // no file system
  stgdesc[STG_LOGINTERVAL]= { "liloginterval",(char*)&settings->loginterval,STG_UINT1, HIDE_P };
  stgdesc[STG_COMPFLASH]  = { "cfcompflash",  (char*)&settings->compflash,  STG_UINT1, HIDE_P };
  stgdesc[STG_IGC_PILOT]  = { "pligc_pilot",   settings->igc_pilot,         sizeof(settings->igc_pilot), HIDE_P };
  stgdesc[STG_IGC_TYPE]   = { "mmigc_type",    settings->igc_type,          sizeof(settings->igc_type), HIDE_P };
  stgdesc[STG_IGC_REG]    = { "rgigc_reg",     settings->igc_reg,           sizeof(settings->igc_reg), HIDE_P };
  stgdesc[STG_IGC_CS]     = { "ciigc_cs",      settings->igc_cs,            sizeof(settings->igc_cs), HIDE_P };
  stgdesc[STG_GN_TO_GP]   = { "gngn_to_gp",   (char*)&settings->gn_to_gp,   STG_UINT1, STG_SAVEHIDE };
  stgdesc[STG_GEOID]      = { "gegeoid",      (char*)&settings->geoid,      STG_INT1, STG_SAVEHIDE };
  stgdesc[STG_LEAPSECS]   = { "lsleapsecs",   (char*)&settings->leapsecs,   STG_INT1, STG_SAVEHIDE };
  stgdesc[STG_EPD_UNITS]  = { "ununits",      (char*)&settings->units,      epd_only(STG_UINT1), HIDE_CP };
  stgdesc[STG_EPD_ZOOM]   = { "zmzoom",       (char*)&settings->zoom,       epd_only(STG_UINT1), HIDE_CP };
  stgdesc[STG_EPD_ROTATE] = { "rtrotate",     (char*)&settings->rotate,     epd_only(STG_UINT1), HIDE_CP };
  stgdesc[STG_EPD_ORIENT] = { "ororientation",(char*)&settings->orientation,epd_only(STG_UINT1), HIDE_CP };
  stgdesc[STG_EPD_ADB]    = { "dbadb",        (char*)&settings->adb,        epd_only(STG_UINT1), HIDE_CP };
  stgdesc[STG_EPD_IDPREF] = { "ifepdidpref",  (char*)&settings->epdidpref,  epd_only(STG_UINT1), HIDE_CP };
  stgdesc[STG_EPD_VMODE]  = { "vmviewmode",   (char*)&settings->viewmode,   epd_only(STG_UINT1), HIDE_CP };
  stgdesc[STG_EPD_AGHOST] = { "ghantighost",  (char*)&settings->antighost,  epd_only(STG_UINT1), HIDE_CP };
  stgdesc[STG_EPD_TEAM]   = { "tmteam",       (char*)&settings->team,       epd_only(STG_HEX6), HIDE_CP };
  stgdesc[STG_CALLSIGN]   = { "cscallsign",   (char*)&settings->callsign,   sizeof(settings->callsign), 0 };
  stgdesc[STG_FANET_SOS]  = { "fsfanet_sos",  (char*)&settings->fanet_sos,  STG_UINT1, 0 };
  stgdesc[STG_DEBUG_FLAGS]= { "dgdebug_flags",(char*)&settings->debug_flags,STG_HEX8, 0 };

  // ensure no null labels in the array
  for (int i=0; i<STG_END; i++) {
     if (!stgdesc[i].label) {
         Serial.print("stg[");
         Serial.print(i);
         Serial.println("] - empty label");
         stgdesc[i].label = "nnnone";
         stgdesc[i].type  = STG_VOID;
     }
  }

  const char *yesno = "1=yes 0=no";
  const char *destinations = "0=off 1=serial 2=UDP 3=TCP 4=USB 5=BT ...";
  //const char *bauds = "0=default(38) 2=9600 3=19200 4=38400 ...";

  stgcomment[STG_MODE]       = "0=Normal ...";
  stgcomment[STG_PROTOCOL]   = "7=Latest 1=OGNTP 2=PAW 5=FANET";
  stgcomment[STG_ALTPROTOCOL]= "0=none 1=OGNTP 6=Legacy 8=ADSL";
  stgcomment[STG_FLR_ADSL]   = "1=FLR+ADSL rx (& some tx)";
  stgcomment[STG_BAND]       = "1=EU 2=US ...";
  stgcomment[STG_ACFT_TYPE]  = "1=GL 2=TOWPL 6=HG 7=PG 0=landed out";
  stgcomment[STG_ID_METHOD]  = "1=ICAO 2=device 5=FANET";
  stgcomment[STG_ALARM]      = "3=Latest 2=Vector 1=Dist";
  stgcomment[STG_HRANGE]     = "km";
  stgcomment[STG_VRANGE]     = "x100m";
  stgcomment[STG_TXPOWER]    = "0=off 1=low 2=full";
  stgcomment[STG_VOLUME]     = "0=off 1=low 2=full 3=ext";
  stgcomment[STG_TCPMODE]    = "0=server 1=client";
  stgcomment[STG_TCPPORT]    = "if client, 0=2000 1=8880";
  stgcomment[STG_BLUETOOTH]  = "0=off 1=classic 2=BLE";
  stgcomment[STG_BAUD_RATE]  = "0=default(38) 2=9600 3=19200 4=38400 ...";
  stgcomment[STG_NMEA_OUT]   = destinations;
  stgcomment[STG_NMEA_G]     = "0=off 1=basic 3=GSA ...";
  stgcomment[STG_NMEA_T]     = "0=off 1=basic";
  stgcomment[STG_NMEA_S]     = "0=off 1=basic 3=LK8EX1";
  stgcomment[STG_NMEA_D]     = yesno;
  stgcomment[STG_NMEA_E]     = "0=off 1=tunnel 2=output 3=both";
  stgcomment[STG_NMEA_OUT2]  = destinations;
  stgcomment[STG_NMEA2_G]    = "0=off 1=basic 3=GSA ...";
  stgcomment[STG_NMEA2_T]    = "0=off 1=basic";
  stgcomment[STG_NMEA2_S]    = "0=off 1=basic 3=LK8EX1";
  stgcomment[STG_NMEA2_D]    = yesno;
  stgcomment[STG_NMEA2_E]    = "0=off 1=tunnel 2=output 3=both";
  stgcomment[STG_BAUDRATE2]  = "0=off 2=9600 3=19200 4=38400 ...";
  stgcomment[STG_ALT_UDP]    = "0=10110 1=10111";
  stgcomment[STG_RX1090]     = "0=none 1=GNS5892";
  stgcomment[STG_RX1090X]    = "comparator offset";
  stgcomment[STG_MODE_S]     = "0=off, 1-9=default 'gain'";
  stgcomment[STG_HRANGE1090] = "km";
  stgcomment[STG_VRANGE1090] = "x100m";
//stgcomment[STG_GDL90_IN]   = destinations;
//stgcomment[STG_GDL90]      = destinations;
//stgcomment[STG_D1090]      = destinations;
  stgcomment[STG_RELAY]      = "0=off 1=landed 2=all 3=only";
  stgcomment[STG_EXPIRE]     = "secs no-rx report 1-30";
  stgcomment[STG_PFLAA_CS]   = yesno;
  stgcomment[STG_STEALTH]    = yesno;
  stgcomment[STG_NO_TRACK]   = yesno;
#if defined(ARDUINO_ARCH_NRF52)
  stgcomment[STG_POWER_SAVE] = "1=turn off BT after 10min";
#else
  stgcomment[STG_POWER_SAVE] = "1=turn off wifi after 10min";
#endif
  stgcomment[STG_POWER_EXT]  = "1=allow dual-power boot, shutdown long after USB off";
  stgcomment[STG_RFC]        = "freq correction +-30";
  stgcomment[STG_LEAPSECS]   = "leap seconds - automatic";
  stgcomment[STG_ALARMLOG]   = yesno;
  stgcomment[STG_LOG_NMEA]   = "1 = log all NMEA output to SD card";
  stgcomment[STG_LOGFLIGHT]  = "0=off 1=always 2=airborne 3=traffic";
  stgcomment[STG_LOGINTERVAL]= "seconds, 1-255";
#if defined(ESP32)
  stgcomment[STG_COMPFLASH]  = "0=log to RAM only, 1=also compress to flash";
#else
  stgcomment[STG_COMPFLASH]  = "0=log uncompressed, 1=compress";
#endif
#if defined(USE_EPAPER)
  stgcomment[STG_EPD_UNITS]  = "0=metric 1=imperial 2=mixed";
  stgcomment[STG_EPD_VMODE]  = "0=status 1=radar 2=text ...";
  stgcomment[STG_EPD_IDPREF] = "0=reg 1=tail 2=model 3=type, 4=hex";
  stgcomment[STG_EPD_AGHOST] = "0=off 1=auto 2=2min 3=5min";
#endif
  //stgcomment[STG_IGC_PILOT] = "also sent as FANET name";
  stgcomment[STG_FANET_SOS] = "0=off 1=manual 2=auto";

  stgminmax[0] = { STG_RFC,       -30, 30 };
  stgminmax[1] = { STG_GEOID,    -104, 84 };
  stgminmax[2] = { STG_LEAPSECS,   17, 19 };  // 18 is correct for 2025
  stgminmax[3] = { STG_TXPOWER,     0,  2 };
  stgminmax[4] = { STG_EXPIRE,      1, ENTRY_EXPIRATION_TIME };
  stgminmax[5] = { STG_MODE_S,      0,  9 };
  stgminmax[6] = { STG_FANET_SOS,   0,  2 };
  stgminmax[7] = { STG_END,         0,  0 };  // marks the end
}

bool hidden_setting(uint8_t index)
{
    if (hw_info.model >= SOFTRF_MODEL_UNKNOWN)
        return false;
    bool rval = (stgdesc[index].hidden & (1 << hw_info.model));
//if (rval)
//Serial.printf("setting %s is hidden\r\n", stgdesc[index].label);
    return rval;
}

// Adjust some settings after loading them
// - code moved from "EEPROM_extension".
void Adjust_Settings()
{
    if (settings->mode == SOFTRF_MODE_MORENMEA)   // obsolete
      settings->mode = SOFTRF_MODE_NORMAL;

//    if (settings->mode & SOFTRF_MODE_EEPROM) {
//      use_eeprom = true;                      // avoid flash file system
//      settings->mode &= 0x0F;
//    }
//        - done in load_setting()

#if defined(ARDUINO_ARCH_NRF52)
    if (settings->mode != SOFTRF_MODE_GPSBRIDGE
#if !defined(EXCLUDE_TEST_MODE)
        &&
        settings->mode != SOFTRF_MODE_TXRX_TEST
#endif /* EXCLUDE_TEST_MODE */
        ) {
      settings->mode = SOFTRF_MODE_NORMAL;
    }

#if defined(ARDUINO_ARCH_NRF52)
    // try not to lose connection to the device due to bad settings
    if (settings->nmea_out2 != DEST_BLUETOOTH && settings->nmea_out2 != DEST_USB) {
        if (settings->nmea_out == DEST_BLUETOOTH) {
            settings->nmea_out2 = DEST_USB;
        } else if (settings->nmea_out == DEST_USB) {
            settings->nmea_out2 = DEST_NONE;
        } else {
            settings->nmea_out  = DEST_BLUETOOTH;
            settings->nmea_out2 = DEST_USB;
        }
    }
    if (settings->nmea_out == DEST_BLUETOOTH || settings->nmea_out2 == DEST_BLUETOOTH)
        settings->bluetooth = BLUETOOTH_LE_HM10_SERIAL;
#endif

    if (settings->rf_protocol > RF_PROTOCOL_ADSL)
        settings->rf_protocol = RF_PROTOCOL_LATEST;
    if (settings->altprotocol > RF_PROTOCOL_ADSL)
        settings->altprotocol = RF_PROTOCOL_NONE;
    if (settings->rf_protocol == RF_PROTOCOL_NONE)  // old settings files before coding change
        settings->rf_protocol = RF_PROTOCOL_LATEST;
    if (settings->rf_protocol == RF_PROTOCOL_LEGACY && settings->altprotocol != RF_PROTOCOL_LATEST)
        settings->altprotocol = RF_PROTOCOL_NONE; 
    if (settings->altprotocol == RF_PROTOCOL_LEGACY && settings->rf_protocol != RF_PROTOCOL_LATEST)
        settings->altprotocol = RF_PROTOCOL_NONE;
    /*
     * Enforce legacy protocol setting for SX1276
     * if other value (UAT) left in EEPROM from other (UATM) radio
     */
    //if (settings->rf_protocol==RF_PROTOCOL_ADSB_1090 || settings->rf_protocol==RF_PROTOCOL_ADSB_UAT)
    //    settings->rf_protocol = RF_PROTOCOL_LEGACY;

    if (settings->bluetooth == BLUETOOTH_SPP)
        settings->bluetooth = BLUETOOTH_LE_HM10_SERIAL;

    if (settings->nmea_out == DEST_UDP  ||
        settings->nmea_out == DEST_TCP ) {
      settings->nmea_out = DEST_BLUETOOTH;
    }
    if (settings->gdl90 == DEST_UDP) {
      settings->gdl90 = DEST_BLUETOOTH;
    }
    if (settings->d1090 == DEST_UDP) {
      settings->d1090 = DEST_BLUETOOTH;
    }

    if (settings->nmea_out2 == settings->nmea_out)
        settings->nmea_out2 = DEST_NONE;
    if (settings->nmea_out2 == DEST_USB && settings->nmea_out  == DEST_UART)
        settings->nmea_out2 = DEST_NONE;
    if (settings->nmea_out  == DEST_USB && settings->nmea_out2 == DEST_UART)
        settings->nmea_out2 = DEST_NONE;

    rst_info *resetInfo = (rst_info *) SoC->getResetInfoPtr();
    if (resetInfo && resetInfo->reason == REASON_SOFT_RESTART)
        ui->viewmode = VIEW_MODE_CONF;     // after software restart show the settings
#endif /* ARDUINO_ARCH_NRF52 */

#if defined(ESP32)
#if defined(CONFIG_IDF_TARGET_ESP32) || defined(USE_USB_HOST)
    if (settings->nmea_out == DEST_USB) {
      settings->nmea_out = DEST_UART;
    }
    if (settings->gdl90 == DEST_USB) {
      settings->gdl90 = DEST_UART;
    }
    if (settings->gdl90_in == DEST_USB) {
      settings->gdl90_in = DEST_UART;
    }
#if !defined(EXCLUDE_D1090)
    if (settings->d1090 == DEST_USB) {
      settings->d1090 = DEST_UART;
    }
#endif
#endif /* CONFIG_IDF_TARGET_ESP32 */
#if defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3)
    if (settings->bluetooth != BLUETOOTH_OFF) {
#if defined(CONFIG_IDF_TARGET_ESP32S3)
      settings->bluetooth = BLUETOOTH_LE_HM10_SERIAL;
#else
      settings->bluetooth = BLUETOOTH_OFF;
#endif
    }
#endif /* CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3 */

  if (settings->debug_flags & DEBUG_SIMULATE)
      settings->rx1090 = ADSB_RX_NONE;

  /* enforce some restrictions on input and output routes */
  int nmea1 = settings->nmea_out;
  int nmea2 = settings->nmea_out2;
  Serial.print(F("NMEA_Output1 (given) = ")); Serial.println(nmea1);
  Serial.print(F("NMEA_Output2 (given) = ")); Serial.println(nmea2);
  if (hw_info.model == SOFTRF_MODEL_PRIME_MK2) {
    if (nmea1==DEST_USB)    nmea1==DEST_UART;
    if (nmea2==DEST_USB)    nmea2==DEST_UART;   // same thing
  }
  if (nmea2 == nmea1)    nmea2 = DEST_NONE;
  if (settings->gdl90_in == DEST_UDP) {
      if (settings->gdl90 == DEST_UDP) {
          settings->gdl90 = DEST_NONE;
          Serial.println(F("GDL input from UDP, GDL output turned OFF"));
      }
  }
  if (settings->gdl90 == nmea1)
      settings->nmea_out = DEST_NONE;   // can't do both at the same time
  if (settings->gdl90 == nmea2)
      settings->nmea_out2 = DEST_NONE;
  if (settings->d1090 == nmea1)
      settings->nmea_out = DEST_NONE;   // can't do both at the same time
  if (settings->d1090 == nmea2)
      settings->nmea_out2 = DEST_NONE;
//  bool wireless1 = (nmea1==DEST_UDP || nmea1==DEST_TCP || nmea1==DEST_BLUETOOTH);
//  bool wireless2 = (nmea2==DEST_UDP || nmea2==DEST_TCP || nmea2==DEST_BLUETOOTH);
  bool wifi1 = (nmea1==DEST_UDP || nmea1==DEST_TCP);
  bool wifi2 = (nmea2==DEST_UDP || nmea2==DEST_TCP);
// >>> try and allow Bluetooth along with WiFi:
//  if (wifi1 && nmea2==DEST_BLUETOOTH)
//        nmea2 = DEST_NONE;      // only one wireless output type possible
//  if (wifi2 && nmea1==DEST_BLUETOOTH)
//        nmea2 = DEST_NONE;
  Serial.print(F("NMEA_Output1 (adjusted) = ")); Serial.println(nmea1);
  settings->nmea_out  = nmea1;
  Serial.print(F("NMEA_Output2 (adjusted) = ")); Serial.println(nmea2);
  settings->nmea_out2 = nmea2;
  //if (nmea1==DEST_BLUETOOTH || nmea2==DEST_BLUETOOTH
  //      || settings->d1090 == DEST_BLUETOOTH || settings->gdl90 == DEST_BLUETOOTH) {
  //    if (settings->bluetooth == BLUETOOTH_OFF)
  //        settings->bluetooth = BLUETOOTH_SPP;
  //}
#if !defined(EXCLUDE_D1090)
  if (nmea1 != DEST_BLUETOOTH && nmea2 != DEST_BLUETOOTH
          && settings->d1090 != DEST_BLUETOOTH && settings->gdl90 != DEST_BLUETOOTH) {
      settings->bluetooth = BLUETOOTH_OFF;
  }
#else
  if (nmea1 != DEST_BLUETOOTH && nmea2 != DEST_BLUETOOTH && settings->gdl90 != DEST_BLUETOOTH) {
      settings->bluetooth = BLUETOOTH_OFF;
  }
#endif
  Serial.print(F("Bluetooth (adjusted) = ")); Serial.println(settings->bluetooth);

  /* enforce some hardware limitations (not enough GPIO pins) */
  if (hw_info.model == SOFTRF_MODEL_PRIME_MK2) {

      if (settings->altpin0 == 1)
          settings->altpin0 = Serial0AltRxPin;   // VP, 36
      if (settings->ppswire == 1)
          settings->ppswire = Serial0AltRxPin;   // VP, 36

      if (hw_info.revision < 8) {
          if (settings->voice == VOICE_EXT) {
              // pin 14 not available, cannot do external I2S
              settings->voice = VOICE_OFF;
          }
          if (settings->baudrate2 != BAUD_DEFAULT) {
              // aux serial on v0.7 uses VP, cannot use it for main serial RX nor for PPS
              if (settings->altpin0 == Serial0AltRxPin)
                  settings->altpin0 = 0;
              if (settings->ppswire == Serial0AltRxPin) {
                  if (settings->gnss_pins == EXT_GNSS_13_2
                  && (settings->voice == VOICE_OFF && settings->strobe == STROBE_OFF))
                      settings->ppswire = SOC_GPIO_PIN_VOICE;   // pin 25 for PPS
                  else
                      settings->ppswire = 0;
              }
          }
          if (settings->gnss_pins == EXT_GNSS_39_4) {
              // GNSS uses VP, cannot use it for main serial rx
              if (settings->altpin0 == Serial0AltRxPin)
                  settings->altpin0 = SOC_GPIO_PIN_VOICE;   // pin 25 for PPS
              if (settings->ppswire == Serial0AltRxPin) {
                  if (settings->sd_card == SD_CARD_13_25)
                      settings->ppswire = 0;
              }
          }
          if (settings->gnss_pins == EXT_GNSS_15_14) {
              // pin 25 is used for GNSS on v0.7 (rather than 15)
              settings->voice = VOICE_OFF;
              settings->strobe = STROBE_OFF;
              //if (settings->sd_card == SD_CARD_13_VP) {    // now uses 4 instead of VP
              //    settings->ppswire = false;
              //}
              if (settings->sd_card == SD_CARD_13_25) {
                  settings->sd_card = SD_CARD_NONE;
                  settings->gnss_pins = EXT_GNSS_NONE;  // don't know what's wired
                  settings->ppswire = 0;
              }
          }
          if (settings->gnss_pins == EXT_GNSS_13_2) {
              if (settings->sd_card == SD_CARD_13_25 || settings->sd_card == SD_CARD_13_VP) {
                  settings->sd_card = SD_CARD_NONE;
                  settings->gnss_pins = EXT_GNSS_NONE;  // don't know what's wired
              }
              // settings->ppswire = false;   // why?
          }
          if (settings->sd_card == SD_CARD_13_VP) {    // now uses 4 instead of VP
              if (settings->gnss_pins == EXT_GNSS_39_4 || settings->gnss_pins == EXT_GNSS_13_2) {
                  settings->sd_card = SD_CARD_NONE;
                  settings->gnss_pins = EXT_GNSS_NONE;
              }
              //if (settings->gnss_pins != EXT_GNSS_NONE)
              //    settings->ppswire = false;
          }
          //if (settings->ppswire && settings->gnss_pins == EXT_GNSS_15_14)
          //    settings->altpin0 = false;
          //if (settings->gnss_pins == EXT_GNSS_39_4)   // already done above
          //    settings->altpin0 = false;
          if (settings->altpin0 == Serial0AltRxPin && settings->rx1090 != ADSB_RX_NONE)
              settings->altpin0 = 0;
      } else {    // T-Beam v1.x
          //if (settings->gnss_pins == EXT_GNSS_NONE || settings->sd_card == SD_CARD_13_VP)
          if (settings->gnss_pins == EXT_GNSS_NONE)
              settings->ppswire = 0;     // PPS is hardwired on the board
          //if (settings->ppswire)
          //    settings->altpin0 = false;
          if (settings->gnss_pins == EXT_GNSS_13_2) {
              if (settings->sd_card == SD_CARD_13_25 || settings->sd_card == SD_CARD_13_VP) {
                  settings->sd_card = SD_CARD_NONE;
                  settings->gnss_pins = EXT_GNSS_NONE;  // don't know what's wired
              }
          }
          if (settings->ppswire == Serial0AltRxPin) {
              if (settings->sd_card == SD_CARD_13_VP) {
                  if (settings->voice == VOICE_OFF && settings->strobe == STROBE_OFF)
                      settings->ppswire = SOC_GPIO_PIN_VOICE;   // pin 25 for PPS
                  else
                      settings->ppswire = 0;
              }
          }
      }
      if (settings->altpin0 == settings->ppswire)
          settings->altpin0 == 0;
      if (settings->gnss_pins == EXT_GNSS_39_4) {
          settings->baudrate2 = BAUD_DEFAULT;       // meaning disabled
          settings->rx1090    = ADSB_RX_NONE;
      }
      if (settings->gnss_pins == EXT_GNSS_15_14) {
          settings->volume = BUZZER_OFF;
          if (settings->voice == VOICE_EXT)
              settings->voice = VOICE_OFF;
      }
      if (settings->sd_card == SD_CARD_13_25) {
          settings->voice = VOICE_OFF;
          settings->strobe = STROBE_OFF;
      }
      if (settings->rx1090 != ADSB_RX_NONE) {
          // dedicate Serial2 to the ADS-B receiver module
          settings->baudrate2 = BAUD_DEFAULT;       // will actually use 921600
          settings->invert2 = false;
          if (settings->nmea_out  == DEST_UART2)
              settings->nmea_out   = DEST_NONE;
          if (settings->nmea_out2 == DEST_UART2)
              settings->nmea_out2  = DEST_NONE;
          if (settings->gdl90     == DEST_UART2)
              settings->gdl90      = DEST_NONE;
          if (settings->gdl90_in  == DEST_UART2)
              settings->gdl90_in   = DEST_NONE;
          if (settings->d1090     == DEST_UART2)
              settings->d1090      = DEST_NONE;
      }
      if (settings->voice == VOICE_EXT) {
          settings->volume = BUZZER_OFF;  // free up pins 14 & 15 for I2S use
      }
  } else {
      settings->voice = VOICE_OFF;
  }

  // if SSID but no password, copy ssid to myssid, it is for AP mode
  if (settings->ssid[0] != '\0' && settings->psk[0] == '\0')
      strcpy(settings->myssid, settings->ssid);

#endif /* ESP32 */

  if (settings->rf_protocol == RF_PROTOCOL_ADSB_1090) {
      if (settings->altprotocol == RF_PROTOCOL_ADSB_1090
      ||  settings->altprotocol == RF_PROTOCOL_P3I
      ||  settings->altprotocol == RF_PROTOCOL_FANET) {
          settings->altprotocol = RF_PROTOCOL_NONE;
      } else {
          settings->rf_protocol = settings->altprotocol;
          settings->altprotocol = RF_PROTOCOL_ADSB_1090;
      }
  }
  if (settings->altprotocol == RF_PROTOCOL_ADSB_1090) {
      if (settings->rf_protocol == RF_PROTOCOL_P3I
      ||  settings->rf_protocol == RF_PROTOCOL_FANET) {
          settings->altprotocol = RF_PROTOCOL_NONE;
      }
  }

  if (settings->loginterval == 0)
      settings->loginterval = 1;

  /* if winch mode, use full transmission power */
  if (settings->acft_type == AIRCRAFT_TYPE_WINCH && settings->txpower == RF_TX_POWER_LOW)
      settings->txpower == RF_TX_POWER_FULL;

  // min and max values for some settings (type INT1 only)
  for (int i=0; i<NUM_MINMAX; i++) {
     int idx = stgminmax[i].index;
     if (idx == STG_END)
         break;
     if (stgdesc[idx].type == STG_INT1) {
       int8_t *stg = (int8_t *)stgdesc[idx].value;
       if (*stg < stgminmax[i].min)  *stg = stgminmax[i].min;
       if (*stg > stgminmax[i].max)  *stg = stgminmax[i].max;
     }
  }

  if (strcmp(settings->igc_pilot,"Chuck Yeager")==0) {
      // old settings may have had that as a default
      *settings->igc_pilot = '\0';
      *settings->igc_type  = '\0';
      *settings->igc_reg   = '\0';
      *settings->igc_cs    = '\0';
  }
  if (settings->igc_pilot[0]=='\0' && settings->callsign[0]!='\0')
    strncpy(settings->igc_pilot, settings->callsign, sizeof(settings->igc_pilot));
  if (settings->callsign[0]=='\0' && settings->igc_cs[0]!='\0')
    strncpy(settings->callsign, settings->igc_cs, sizeof(settings->callsign));
  if (settings->callsign[0]=='\0' && settings->igc_pilot[0]!='\0')
    strncpy(settings->callsign, settings->igc_pilot, sizeof(settings->callsign));
  if (settings->callsign[0]=='\0' && settings->igc_reg[0]!='\0')
    strncpy(settings->callsign, settings->igc_reg, sizeof(settings->callsign));
  if (settings->callsign[0]=='\0' && settings->igc_type[0]!='\0')
    strncpy(settings->callsign, settings->igc_type, sizeof(settings->callsign));
}

const char *settings_message(const char *newmsg, const char *submsg, const int val)
{
    static char msg[80] = {0};
    if (newmsg != NULL) {
        if (submsg != NULL)
            snprintf(msg, sizeof(msg), newmsg, submsg);
        else if (val >= 0)
            snprintf(msg, sizeof(msg), newmsg, val);
        else
            strncpy(msg, newmsg, sizeof(msg));
    }
    return msg;
}

void show_settings_short();  // forward declaration

#if defined(INCLUDE_EEPROM)
// start reading from the first byte (address 0) of the EEPROM
eeprom_t eeprom_block;
#endif

void Settings_setup()
{
  settings = &settings_stored;

  init_stgdesc();

  // start with defaults, then overwrite from file or EEPROM
  Settings_defaults();

  bool success = load_settings();

#if 0
Serial.println("settings shorthand:");
  show_settings_short();  // test shorthand formatting
#endif

  Adjust_Settings();

//  if (! success)               // load failed
//     save_settings_to_file();  // save to a file (or EEPROM)
// - no point saving, since it's the defaults
}

void Settings_defaults()
{
    settings_used = STG_DEFAULT;
    settings_message("Warning: reverted to default settings");

    settings->mode        = SOFTRF_MODE_NORMAL;

    if (hw_info.model == SOFTRF_MODEL_BRACELET)  // >>> T1000E, M3, maybe T-Echo?
    {
        settings->acft_type   = AIRCRAFT_TYPE_PARAGLIDER;
        settings->rf_protocol = RF_PROTOCOL_FANET;
        //settings->id_method   = ADDR_TYPE_FANET;
        settings->id_method   = ADDR_TYPE_FLARM;
    } else {
        settings->acft_type   = AIRCRAFT_TYPE_GLIDER;
        settings->rf_protocol = RF_PROTOCOL_LATEST;
        settings->id_method   = ADDR_TYPE_FLARM;
    }

#if defined(DEFAULT_REGION_US)
    settings->band        = RF_BAND_US;
#elif defined(DEFAULT_REGION_EU)
    settings->band        = RF_BAND_EU;
#else
    //if (hw_info.model == SOFTRF_MODEL_PRIME_MK2)
    //    settings->band    = RF_BAND_EU;
    //else
        settings->band    = RF_BAND_AUTO;
#endif

    settings->aircraft_id = 0;
    settings->txpower     = RF_TX_POWER_FULL;

#if defined(ESP32)   // T-Beam
    settings->nmea_out  = DEST_UART;
    settings->nmea_out2 = DEST_UDP;
#else   // nRF52
    settings->nmea_out  = DEST_USB;
#if defined(USBD_USE_CDC) && !defined(DISABLE_GENERIC_SERIALUSB)
    settings->nmea_out2 = DEST_NONE;
#else
    settings->nmea_out2 = DEST_BLUETOOTH;
#endif
#endif

    settings->nmea_g  = NMEA_BASIC;
    settings->nmea_p  = 0;
    settings->nmea_t  = NMEA_BASIC;
    settings->nmea_s  = NMEA_BASIC;
    settings->nmea_d  = 0;
    settings->nmea_e  = 0;

    settings->nmea2_g = NMEA_BASIC;
    settings->nmea2_p = 0;
    settings->nmea2_t = NMEA_BASIC;
    settings->nmea2_s = NMEA_BASIC;
    settings->nmea2_d = 0;
    settings->nmea2_e = 0;

#if defined(ARDUINO_ARCH_NRF52)
    settings->bluetooth  = BLUETOOTH_LE_HM10_SERIAL;
#else
    settings->bluetooth  = BLUETOOTH_OFF;
#endif
    settings->alarm      = TRAFFIC_ALARM_LATEST;
    settings->stealth    = false;
    settings->no_track   = false;

    settings->baud_rate  = BAUD_DEFAULT;      // Serial  - meaning 38400
    settings->baudrate2  = BAUD_DEFAULT;      // Serial2 - meaning disabled
    settings->invert2    = false;
    settings->freq_corr  = 0;

    if (hw_info.model == SOFTRF_MODEL_STANDALONE
     || hw_info.model == SOFTRF_MODEL_PRIME) {
      settings->pointer = DIRECTION_NORTH_UP;
    } else {
      settings->pointer = LED_OFF;
    }

    settings->ignore_id = 0;
    settings->follow_id = 0;

    settings->tcpmode = TCP_MODE_SERVER;
    strncpy(settings->host_ip, NMEA_TCP_IP, sizeof(settings->host_ip)-1);
    settings->host_ip[sizeof(settings->host_ip)-1] = '\0';
    settings->tcpport = 0;   // 2000
    settings->alt_udp    = false;

    settings->gdl90_in   = DEST_NONE;
    settings->gdl90      = DEST_NONE;
#if !defined(EXCLUDE_D1090)
    settings->d1090      = DEST_NONE;
#endif

    settings->logalarms  = false;
    settings->log_nmea   = false;

    if (hw_info.model == SOFTRF_MODEL_STANDALONE
     || hw_info.model == SOFTRF_MODEL_PRIME) {
      settings->volume  = BUZZER_OFF;
    } else {
      settings->volume  = BUZZER_VOLUME_FULL;
    }
    settings->voice     = VOICE_OFF;
    settings->strobe    = STROBE_OFF;

    settings->gnss_pins = EXT_GNSS_NONE;   // whether an external GNSS module was added to a T-Beam
    settings->ppswire   = 0;               // whether T-Beam v0.7 or external GNSS has PPS wire connected
    settings->sd_card   = SD_CARD_NONE;
    settings->logflight = FLIGHT_LOG_NONE;
    settings->loginterval = 4;
    settings->rx1090    = ADSB_RX_NONE;
    settings->mode_s    = 0;
    settings->gn_to_gp  = 0;
    settings->geoid     = 0;

    settings->myssid[0] = '\0';
    settings->myssid[sizeof(settings->ssid)-1] = '\0';
    settings->ssid[0] = '\0';   // default is empty string - speeds up booting
    settings->ssid[sizeof(settings->ssid)-1] = '\0';
    settings->psk[0] = '\0';
    settings->psk[sizeof(settings->psk)-1] = '\0';

    settings->relay = RELAY_LANDED;

  //settings->json        = JSON_OFF;
    settings->power_save  = (hw_info.model == SOFTRF_MODEL_BRACELET ? POWER_SAVE_NORECEIVE : POWER_SAVE_NONE);
    settings->power_ext   = 0;
    settings->altpin0     = 0;
    settings->debug_flags = 0;      // if and when debug output will be turned on - 0x3F for all

    settings->igc_key[0] = 0;
    settings->igc_key[1] = 0;
    settings->igc_key[2] = 0;
    settings->igc_key[3] = 0;

//#if defined(USE_EPAPER)
#if defined(DEFAULT_REGION_US)
    settings->units       = UNITS_IMPERIAL;
#else
    settings->units       = UNITS_METRIC;
#endif
    settings->zoom        = ZOOM_MEDIUM;
    settings->rotate      = ROTATE_0;
    settings->orientation = DIRECTION_TRACK_UP;
    settings->adb         = DB_NONE;
    settings->epdidpref   = ID_HEX;
    settings->viewmode    = VIEW_MODE_STATUS;
    settings->antighost   = ANTI_GHOSTING_AUTO;
    settings->team        = 0;
//#endif
    settings->fanet_sos   = 0;

    settings->version = 0;        // SOFTRF_SETTINGS_VERSION will come from file
    settings->altprotocol = RF_PROTOCOL_NONE;
    settings->flr_adsl    = 0;
    settings->rx1090x     = 100;
    settings->hrange      = 27;   // km
    settings->vrange      = 20;   // 2000m
    settings->hrange1090  = 27;   // km
    settings->vrange1090  = 20;   // 2000m
    settings->compflash   = false;
    settings->expire      = EXPORT_EXPIRATION_TIME;   // 5 secs
    settings->pflaa_cs    = true;
    settings->leapsecs    = 18;   // <<< hardcoded!
        // - Correct for 2025, and will automatically adjust after valid fix if necessary
    //strcpy(settings->igc_pilot, "Chuck Yeager");
    //strcpy(settings->igc_type,  "ASW20");
    //strcpy(settings->igc_reg,   "N1234");
    //strcpy(settings->igc_cs,    "XXX");
}

#if defined(INCLUDE_EEPROM)
void EEPROM_store()
{
  Serial.println("Writing EEPROM...");
  //if (SoC->Bluetooth_ops) { SoC->Bluetooth_ops->fini(); }
  //uint16_t size = sizeof(eeprom_struct_t);
  //for (int i=0; i<size; i++) {
  //  EEPROM.write(i, eeprom_block.raw[i]);
  //}
  //EEPROM.write_block(eeprom_block.raw, 0, size);
  EEPROM.put(0, eeprom_block.raw);
  EEPROM_commit();
}
#endif

bool format_setting(const int i, const bool comment, bool shorthand, char *buf, size_t size)
{
    int t = stgdesc[i].type;
    if (t == STG_VOID || t == STG_OBSOLETE)   // skipped in all output (web, file, serial)
        return false;
    char *v = stgdesc[i].value;
    if (shorthand) {
        const char c1 = stgdesc[i].label[0];
        const char c2 = stgdesc[i].label[1];
        switch (t) {
        case STG_INT1:
           snprintf(buf, size,"%c%c%d\n", c1, c2, (int)(*(int8_t*)v));
           break;
        case STG_UINT1:
           snprintf(buf, size,"%c%c%d\n", c1, c2, (int)(*(uint8_t*)v));
           break;
        case STG_HEX2:
           snprintf(buf, size,"%c%c%X\n", c1, c2, (int)(*(uint8_t*)v));
           break;
        case STG_HEX6:
        case STG_HEX8:
           snprintf(buf, size,"%c%c%X\n", c1, c2, (int)(*(uint32_t*)v));
           break;
        default:
           snprintf(buf, size,"%c%c%s\n", c1, c2, (t >= STG_STR)? v : "?");
           break;
        }
    } else {   // not shorthand
        const char *w = &stgdesc[i].label[2];
        switch (t) {
        case STG_INT1:
           snprintf(buf, size,"%s,%d\r\n", w, (int)(*(int8_t*)v));
           break;
        case STG_UINT1:
           snprintf(buf, size,"%s,%d\r\n", w, (int)(*(uint8_t*)v));
           break;
        case STG_HEX2:
           snprintf(buf, size,"%s,%02X\r\n", w, (int)(*(uint8_t*)v));
           break;
        case STG_HEX6:
           snprintf(buf, size,"%s,%06X\r\n", w, (int)(*(uint32_t*)v));
           break;
        case STG_HEX8:
           snprintf(buf, size,"%s,%X\r\n", w, (int)(*(uint32_t*)v));
           break;
        default:
           snprintf(buf, size,"%s,%s\r\n", w, (t >= STG_STR)? v : "?");
           break;
        }
        if (comment && stgcomment[i] != NULL) {
            int len = strlen(buf) - 2;   // clobber the \r\n
            if (size > 18) {
                while (len < 18)
                    buf[len++] = ' ';
            }
            snprintf(buf+len, size-len, "%s%s\r\n", " # ", stgcomment[i]);
        }
    }
    return true;
}

void show_settings_serial()
{
  for (int i=STG_MODE; i<STG_END; i++) {
     if (hidden_setting(i))
         continue;
     if (format_setting(i) == false)
         continue;
     Serial.print(NMEABuffer);
     delay(10);
  }

#if defined(USE_OGN_ENCRYPTION)
    if (settings->rf_protocol == RF_PROTOCOL_OGNTP) {
        Serial.print("IGC key");
        Serial.printf(" %08X", (settings->igc_key[0]? 0x88888888 : 0));
        Serial.printf(" %08X", (settings->igc_key[1]? 0x88888888 : 0));
        Serial.printf(" %08X", (settings->igc_key[2]? 0x88888888 : 0));
        Serial.printf(" %08X\r\n", (settings->igc_key[3]? 0x88888888 : 0));
    }
#endif
}

#if 0
void show_settings_short()
{
  for (int i=STG_MODE; i<STG_END; i++) {
     if (format_setting(i, false, true) == false)
         continue;
     Serial.print(NMEABuffer);
     Serial.print("\r");         // outputs xxxx\n\r
     delay(10);
  }
}
#endif

#if defined(INCLUDE_EEPROM)
void save_settings_to_EEPROM()
{
  Serial.println(F("Saving settings to EEPROM..."));
  eeprom_block.field.magic = SOFTRF_EEPROM_MAGIC;
  char *p = eeprom_block.field.text;
  if (use_eeprom)
      settings->mode |= SOFTRF_MODE_EEPROM;
  int size = 0;
  for (int i=STG_MODE; i<STG_END; i++) {             // skip version
     if (hidden_setting(i) && stgdesc[i].hidden != STG_SAVEHIDE)
         continue;
     if (use_eeprom && stgdesc[i].hidden == HIDE_P)  // avoiding the file system
         continue;
     if (format_setting(i, false, true, p, EEPROM_SIZE - size) == false)
         continue;
     int len = strlen(p);    // includes the '\n'
     p += len;
     size += len;
  }
  p[size] = '\0';
  settings->mode &= 0x0F;
//  while (size < EEPROM_SIZE)
//         p[size++] = '\0';
//Serial.println(p);
  EEPROM_store();
  delay(200);
}
#endif

#if defined(FILESYS)

void save_settings_to_file(bool reboot)
{
  SoC->WDT_fini();
  if (SoC->Bluetooth_ops) { SoC->Bluetooth_ops->fini(); }
#if defined(INCLUDE_EEPROM)
  if (! FS_is_mounted || use_eeprom) {
      if (use_eeprom)
          Serial.println(F("File system assumed broken"));
      else
          Serial.println(F("File system is not mounted"));
      save_settings_to_EEPROM();
      if (! reboot && SoC->Bluetooth_ops) { SoC->Bluetooth_ops->setup(); }
      SoC->WDT_setup();
      return;
  }
#endif
  Serial.println(F("Saving settings to settings.txt ..."));
  if (FILESYS.exists("/settings.txt"))
      FILESYS.remove("/settings.txt");
  File SettingsFile = FILESYS.open("/settings.txt", FILE_WRITE);
  if (SettingsFile) {
      snprintf(NMEABuffer,sizeof(NMEABuffer),"# originator: model %d fw %s ID %06X\r\n",
                   hw_info.model, SOFTRF_FIRMWARE_VERSION, SoC->getChipId());
      Serial.print(NMEABuffer);
      SettingsFile.write((const uint8_t *)NMEABuffer, strlen(NMEABuffer));
      settings->version = SOFTRF_SETTINGS_VERSION;
      bool write_error = false;
      for (int i=STG_VERSION; i<STG_END; i++) {
           if (format_setting(i) == false)
               continue;
           int len = strlen(NMEABuffer);
           if (SettingsFile.write((const uint8_t *)NMEABuffer, len) == len) {
               Serial.print(NMEABuffer);
           } else {
               Serial.println(F("Error writing to settings.txt"));
               write_error = true;
               break;
           }
           delay(10);
      }
      SettingsFile.close();
      if (write_error)
          FILESYS.remove("/settings.txt");
      else
          Serial.println(F("... OK"));
  } else {
      Serial.println(F("Failed to open settings.txt"));
  }
  if (! reboot && SoC->Bluetooth_ops) { SoC->Bluetooth_ops->setup(); }
  SoC->WDT_setup();
}

#endif /* FILESYS */

int find_setting(const char *p, bool sh)
{
    if (! sh) {
        if (strcmp(p,"nmea_l")==0)   p = "nmea_t";   // to accept old settings files
        if (strcmp(p,"nmea2_l")==0)  p = "nmea2_t";
    }
    for (int i=STG_VERSION; i<STG_END; i++) {
        if (sh) {
            if (p[0]==stgdesc[i].label[0] && p[1]==stgdesc[i].label[1])
                return i;
        } else {
            if (strcmp(p,&stgdesc[i].label[2])==0)
                return i;
        }
    }
    return STG_END;
}

// read in obsolete settings but convert to their replacements
void convert_obsolete(const int idx)
{
    if (idx == STG_OLD_TXPWR)
        settings->txpower = 2 - settings->old_txpwr;
}

bool load_setting(const int idx, const char *q)
{
    int t = stgdesc[idx].type;
    if (t == STG_VOID) {
        Serial.print(F(" - ignored on this platform"));
        return true;
    }
    if (t < STG_VOID && *q == '\0') {
        Serial.print(F(" - empty field, a number expected"));
        return false; 
    }
    char *v = stgdesc[idx].value;
    switch(t) {
    case STG_INT1:
    case STG_OBSOLETE:
        *(int8_t *)v = (int8_t) atoi(q);
        break;
    case STG_UINT1:
        *(uint8_t *)v = (uint8_t) atoi(q);
        break;
    case STG_HEX2:
        *(uint8_t *)v = (uint8_t) strtol(q,NULL,16);
        break;
    case STG_HEX6:
    case STG_HEX8:
        *(uint32_t *)v = (uint32_t) strtol(q,NULL,16);
        break;
    default:
        strncpy(v, q, t);
        v[t-1] = '\0';
        break;
    }
    if (t == STG_OBSOLETE)
        convert_obsolete(idx);
    if (idx == STG_MODE) {
        use_eeprom = (settings->mode | SOFTRF_MODE_EEPROM);
        settings->mode &= 0x0F;
    }
    return true;
}

static bool interpretShorthand(char * buf=NMEABuffer)
{
    int i = find_setting(buf, true);
    if (i == STG_END)   // not found
        return false;
    return load_setting(i,buf+2);  // what follows the short label, no comma
}

static bool interpretSetting(char * buf=NMEABuffer)
{
    char *p = buf;
    while (*p != ',') {
        p++;
        if (*p == '\0')
            return false;
    }
    *p = '\0';  // overwrites the comma
    ++p;        // points to what followed the comma

    int i = find_setting(buf, false);
    if (i == STG_END)   // not found
        return false;

    bool is_numerical = (stgdesc[i].type < STG_VOID);

    char *r = p;
    char *s = NULL;
    while (1) {
        if (*r == '\0') {     // end of the line
            if (s)
                *s = '\0';    // drop trailing spaces
            break;
        } else if (*r == ' ') {
            if (is_numerical) {    // for settings that are numbers
                *r = '\0';         // treat anything after first space as a comment
                break;
            }
            if (s == NULL)
                s = r;  // first space
        } else if (*r=='#' || *r=='*' || *r==';' || *r=='/') {
            if (s)
                *s = '\0';    // end the string at the first space
            else
                *r = '\0';    // comment started without a space
            break;
        } else {  // not a space and not one of the chars signaling a comment
            s = NULL;
            // consider the previous space and this non-space part of the string value
        }
        r++;
    }

    return load_setting(i,p);
}

// known obsolete settings labels, ignore them, don't complain
bool known_obsolete_label()
{
    if (strcmp(NMEABuffer, "json")==0)
        return true;
    return false;
}

#if defined(FILESYS)

bool load_settings_from_file()
{
    if (! FS_is_mounted) {
        Serial.println(F("File system is not mounted"));
        return false;
    }
    if (! FILESYS.exists("/settings.txt")) {
        Serial.println(F("File settings.txt does not exist"));
        return false;
    }
    File SettingsFile = FILESYS.open("/settings.txt", FILE_READ);
    if (!SettingsFile) {
        Serial.println(F("Failed to open settings.txt"));
        return false;
    }
    //Settings_defaults();    // already done in settings_setup()
    //settings->version = 0;
    int limit = 200;
    Serial.println(F("Loading settings from file..."));
    int nsettings = -1;
    bool all_settings_valid = true;
    while (getline(SettingsFile, NMEABuffer, sizeof(NMEABuffer)) && --limit>0) {
        Serial.println(NMEABuffer);
        // allow blank or comment lines
        if (NMEABuffer[0] == '#')  continue;
        if (NMEABuffer[0] == '*')  continue;
        if (NMEABuffer[0] == ';')  continue;
        if (NMEABuffer[0] == '/')  continue;
        if (NMEABuffer[0] == ' ')  continue;
        if (NMEABuffer[0] == '\0')  continue;
        if (interpretSetting() == false) {
          if (! known_obsolete_label()) {
            // load what is valid and ignore the invalid
            all_settings_valid = false;
            settings_message("Invalid setting label '%s' in file", NMEABuffer);
            Serial.println(settings_message());
          }
        } else {
            ++nsettings;
        }
        delay(10);
    }
    SettingsFile.close();
    if (settings->version != SOFTRF_SETTINGS_VERSION) {
        // version number was wrong or version line missing
        Serial.println(F("bad settings.txt version, erased file"));
        FILESYS.remove("/settings.txt");
        Settings_defaults();
        return false;
    }
    if (nsettings > 0) {                        // not counting the "version"
        //Serial.print("... Loaded ");
        //Serial.print(nsettings);
        //Serial.println(F(" settings from file"));
        settings_used = STG_FILE;    // may be a mix of defaults and values from file
        if (all_settings_valid) {
            settings_message("Loaded %d user settings from file on boot", (char *)NULL, nsettings);
            Serial.println(settings_message());
        }
    }
    return true;
}

#endif /* FILESYS */

bool load_settings()
{
#if defined(FILESYS)
    if (use_eeprom) {
        Serial.println(F("File system assumed broken"));
    } else if (load_settings_from_file() == true) {
        return true;
    }
#endif

#if defined(INCLUDE_EEPROM)

    // settings file not found, try and read settings from EEPROM block

    if (SoC->EEPROM_begin(sizeof(eeprom_t)) == false) {
        Serial.println(F("WARNING! cannot access EEPROM. Using defaults..."));
        return false;
    }

    //for (int i=0; i<sizeof(eeprom_t); i++) {
    //  eeprom_block.raw[i] = EEPROM.read(i);
    //}
    //delay(10);
    EEPROM.get(0, eeprom_block.raw);

    if (eeprom_block.field.magic != SOFTRF_EEPROM_MAGIC) {
        Serial.println(F("WARNING! EEPROM magic wrong. Using defaults..."));
        return false;
    }

    Serial.println(F("Loading settings from EEPROM..."));
    settings->version = SOFTRF_SETTINGS_VERSION;
    char *settings_text = eeprom_block.field.text;
    int nsettings = 0;
    bool all_settings_valid = true;
    char *p = settings_text;
    for (int i=0; i<EEPROM_SIZE; i++) {
        if (settings_text[i] == '\0')
            break;
        if (settings_text[i] == '\n') {
            settings_text[i] = '\0';
//Serial.println(p);
            if (interpretShorthand(p) == false) {
                    all_settings_valid = false;
                    settings_message("Invalid shorthand setting '%s'", p);
                    Serial.println(settings_message());
            } else {
                ++nsettings;
            }
            p = &settings_text[i+1];
        }
    }

    if (nsettings > 0) {                        // not counting the "version"
        settings_used = STG_EEPROM;
        if (all_settings_valid) {
            settings_message("Loaded %d settings from EEPROM on boot", (char *)NULL, nsettings);
            Serial.println(settings_message());
        }
    }
    
#endif
    return true;
}

