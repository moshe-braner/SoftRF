/*
 * OLEDHelper.cpp
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

#include "../system/SoC.h"

#if defined(USE_OLED)

#include <Wire.h>

#include "OLED.h"

#include "Settings.h"
#include "RF.h"
#include "../protocol/data/GNS5892.h"
#include "../protocol/data/NMEA.h"
#include "LED.h"
#include "GNSS.h"
#include "Baro.h"
#include "Battery.h"
#include "../TrafficHelper.h"
#include "../system/Time.h"
#include "WiFi.h"

enum
{
  OLED_PAGE_SETTINGS,
  OLED_PAGE_RADIO,
#if !defined(EXCLUDE_OLED_BARO_PAGE)
  OLED_PAGE_BARO,
#endif /* EXCLUDE_OLED_BARO_PAGE */
#if !defined(EXCLUDE_OLED_ACFT_PAGE)
  OLED_PAGE_ACFT,
#endif /* EXCLUDE_OLED_ACFT_PAGE */
#if !defined(EXCLUDE_OLED_WIFI_PAGE)
  OLED_PAGE_WIFI,
#endif /* EXCLUDE_OLED_WIFI_PAGE */
  OLED_PAGE_COUNT
};

#if !defined(EXCLUDE_OLED_049)
enum
{
  OLED_049_PAGE_ID,
  OLED_049_PAGE_PROTOCOL,
  OLED_049_PAGE_RX,
  OLED_049_PAGE_SATS_TX,
  OLED_049_PAGE_ACFTS,
  OLED_049_PAGE_UPTIME,
  OLED_049_PAGE_VOLTAGE,
  OLED_049_PAGE_COUNT
};
#endif /* EXCLUDE_OLED_049 */

#if defined(CONFIG_IDF_TARGET_ESP32S3)
U8X8_SSD1306_128X64_NONAME_HW_I2C     u8x8_i2c(U8X8_PIN_NONE);
#else
U8X8_SSD1306_128X64_NONAME_HW_I2C     u8x8_i2c(U8X8_PIN_NONE, SOC_GPIO_PIN_TBEAM_SCL, SOC_GPIO_PIN_TBEAM_SDA);
U8X8_SSD1306_128X64_NONAME_2ND_HW_I2C u8x8_i2c2(U8X8_PIN_NONE, TTGO_V2_OLED_PIN_SCL, TTGO_V2_OLED_PIN_SDA);
#endif

U8X8 *u8x8 = NULL;

static bool OLED_display_titles = false;
static uint32_t prev_tx_packets_counter = (uint32_t) -1;
static uint32_t prev_rx_packets_counter = (uint32_t) -1;
static uint32_t prev_adsb_packets_counter = (uint32_t) -1;
static bool prev_rx1090found = false;
// extern uint32_t tx_packets_counter, rx_packets_counter, adsb_packets_counter;

static uint32_t prev_acrfts_counter = (uint32_t) -1;
static uint32_t prev_sats_counter   = (uint32_t) -1;
static uint32_t prev_uptime_minutes = (uint32_t) -1;
static int32_t  prev_voltage        = (uint32_t) -1;
static char     prev_fix            = ' ';
static int8_t   prev_band           = -1;
static int8_t   prev_type           = -1;
static int8_t   prev_min            = -1;
static int8_t   prev_maxrssi        = 0;

#if !defined(EXCLUDE_OLED_BARO_PAGE)
static int32_t  prev_altitude       = (int32_t)   -10000;
static int32_t  prev_temperature    = (int32_t)   -100;
static uint32_t prev_pressure       = (uint32_t)  -1;
static int32_t  prev_cdr            = (int32_t)   -10000; /* climb/descent rate */
#endif /* EXCLUDE_OLED_BARO_PAGE */

unsigned long OLEDTimeMarker = 0;

const char *ISO3166_CC[] = {
  [RF_BAND_AUTO] = "--",
  [RF_BAND_EU]   = "EU",
  [RF_BAND_US]   = "US",
  [RF_BAND_AU]   = "AU",
  [RF_BAND_NZ]   = "NZ",
  [RF_BAND_RU]   = "RU",
  [RF_BAND_CN]   = "CN",
  [RF_BAND_UK]   = "UK",
  [RF_BAND_IN]   = "IN",
  [RF_BAND_IL]   = "IL",
  [RF_BAND_KR]   = "KR"
};

const char *aircraft_type_lbl[] = {
  [AIRCRAFT_TYPE_UNKNOWN]    = "--",
  [AIRCRAFT_TYPE_GLIDER]     = "GL",
  [AIRCRAFT_TYPE_TOWPLANE]   = "TP",
  [AIRCRAFT_TYPE_HELICOPTER] = "HC",
  [AIRCRAFT_TYPE_PARACHUTE]  = "PC",
  [AIRCRAFT_TYPE_DROPPLANE]  = "DP",
  [AIRCRAFT_TYPE_HANGGLIDER] = "HG",
  [AIRCRAFT_TYPE_PARAGLIDER] = "PG",
  [AIRCRAFT_TYPE_POWERED]    = "PP",
  [AIRCRAFT_TYPE_JET]        = "JT",
  [AIRCRAFT_TYPE_UFO]        = "UF",
  [AIRCRAFT_TYPE_BALLOON]    = "BL",
  [AIRCRAFT_TYPE_ZEPPELIN]   = "ZP",
  [AIRCRAFT_TYPE_UAV]        = "DR",
  [AIRCRAFT_TYPE_RESERVED]   = "--",
  [AIRCRAFT_TYPE_STATIC]     = "ST",
  [AIRCRAFT_TYPE_WINCH]      = "WI"
};

const char ID_text[]       = "ID";
const char PROTOCOL_text[] = "PROTOCOL";
const char TX_text[]       = "TX";
const char RX_text[]       = "RX";
const char ACFTS_text[]    = "ACFTS";
const char SATS_text[]     = "SATS";
const char FIX_text[]      = "FIX";
const char TYP_text[]      = "TYP";
const char BND_text[]      = "BND";
const char BAT_text[]      = "BAT";

#if !defined(EXCLUDE_OLED_BARO_PAGE)
const char ALT_text[]      = "ALT M";
const char TEMP_text[]     = "TEMP C";
const char PRES_text[]     = "PRES MB";
const char CDR_text[]      = "CDR FPM";
#endif /* EXCLUDE_OLED_BARO_PAGE */

static const uint8_t Dot_Tile[] = { 0x00, 0x00, 0x00, 0x18, 0x18, 0x00, 0x00, 0x00 };

static uint8_t OLED_current_page = OLED_PAGE_SETTINGS;
static uint8_t page_count        = OLED_PAGE_COUNT;
static bool showing_message = false;

//byte OLED_setup()
// done in ESP32.cpp ESP32_Display_setup() instead

static void OLED_settings()
{
  char buf[16];
  uint32_t disp_value;

  if (!OLED_display_titles) {

    u8x8->clear();

    u8x8->drawString(1, 1, ID_text);

    snprintf (buf, sizeof(buf), "%06X", ThisAircraft.addr);
    u8x8->draw2x2String(0, 2, buf);

    if (settings->debug_flags & DEBUG_SIMULATE)
        u8x8->drawString(8, 1, "SIM");
    else
        u8x8->drawString(8, 1, PROTOCOL_text);

    char c = Protocol_ID[ThisAircraft.protocol][0];
    if (ThisAircraft.protocol == RF_PROTOCOL_LATEST)
        c = 'T';
    u8x8->draw2x2Glyph(14, 2, c);

    u8x8->drawString( 0, 5, BND_text);
    u8x8->drawString( 5, 5, TYP_text);
    u8x8->drawString(12, 5, BAT_text);

    //u8x8->drawTile  (4, 6, 1, (uint8_t *) Dot_Tile);
    //u8x8->drawTile  (4, 7, 1, (uint8_t *) Dot_Tile);

    u8x8->drawGlyph (13, 7, '.');

    prev_uptime_minutes = (uint32_t) -1;
    prev_voltage        = (uint32_t) -1;
    prev_band           = -1;
    prev_type           = -1;

    OLED_display_titles = true;
  }

#if 0
  uint32_t uptime_minutes = millis() / 60000;
  if (prev_uptime_minutes != uptime_minutes) {
    uint32_t uptime_hours = uptime_minutes / 60;
    disp_value = uptime_hours % 100;
    if (disp_value < 10) {
      buf[0] = '0';
      itoa(disp_value, buf+1, 10);
    } else {
      itoa(disp_value, buf, 10);
    }


    u8x8->draw2x2String(0, 6, buf);

    disp_value = uptime_minutes % 60;
    if (disp_value < 10) {
      buf[0] = '0';

      itoa(disp_value, buf+1, 10);
    } else {
      itoa(disp_value, buf, 10);
    }

    u8x8->draw2x2String(5, 6, buf);

    prev_uptime_minutes = uptime_minutes;
  }
#else
  // uptime is not important, show band and aircraft type instead:
  if (prev_band != settings->band) {
    prev_band = settings->band;
    if (prev_band >= 0 && prev_band < 11)
      u8x8->draw2x2String(0, 6, ISO3166_CC[prev_band]);
  }
  if (prev_type != settings->acft_type) {
    prev_type = settings->acft_type;
    if (prev_type >= 0 && prev_type < 17)
      u8x8->draw2x2String(5, 6, aircraft_type_lbl[prev_type]);
  }
#endif

  int32_t  voltage = Battery_voltage() > BATTERY_THRESHOLD_INVALID ?
                              (int) (Battery_voltage() * 10.0 + 0.5) : 0;

  if (prev_voltage != voltage) {
    if (voltage) {
      disp_value = voltage / 10;
      disp_value = disp_value > 9 ? 9 : disp_value;
      u8x8->draw2x2Glyph(11, 6, '0' + disp_value);

      disp_value = voltage % 10;

      u8x8->draw2x2Glyph(14, 6, '0' + disp_value);
    } else {
      u8x8->draw2x2Glyph(11, 6, 'N');
      u8x8->draw2x2Glyph(14, 6, 'A');
    }
    prev_voltage = voltage;
  }
}

static void OLED_radio()
{
  char buf[16];
  uint32_t disp_value;

  if (!OLED_display_titles) {

    u8x8->clear();

    u8x8->drawString( 1, 0, ACFTS_text);
    u8x8->drawString( 7, 0, SATS_text);
    if (settings->debug_flags & DEBUG_SIMULATE)
        u8x8->drawString(12, 0, "SIM");
    else
        u8x8->drawString(12, 0, FIX_text);
    u8x8->drawString(0, 4, TX_text);
    if (settings->rx1090) {
        u8x8->drawString(8, 4, RX_text);
        u8x8->drawString(7, 6, "ADS");
    } else if (settings->gdl90_in != DEST_NONE) {
        u8x8->drawString(8, 4, RX_text);
        u8x8->drawString(7, 6, "GDL");
    } else {
        //u8x8->drawString(10, 4, RX_text);
        u8x8->drawString(8, 4, RX_text);
        u8x8->drawString(7, 6, "RSS");
    }

    if (settings->power_save & POWER_SAVE_NORECEIVE &&
        (hw_info.rf == RF_IC_SX1276 || hw_info.rf == RF_IC_SX1262)) {
      u8x8->draw2x2String(10, 4, "OFF");
      prev_rx_packets_counter = rx_packets_counter;
    } else {
      prev_rx_packets_counter = (uint32_t) -1;
    }

    prev_adsb_packets_counter = -1;

    if (settings->mode        == SOFTRF_MODE_RECEIVER ||
        settings->rf_protocol == RF_PROTOCOL_ADSB_UAT ||
        settings->txpower     == RF_TX_POWER_OFF) {
      u8x8->draw2x2String(0, 5, "OFF");
      prev_tx_packets_counter = tx_packets_counter;
    } else {
      prev_tx_packets_counter = (uint32_t) -1;
    }

    prev_acrfts_counter = (uint32_t) -1;
    prev_sats_counter   = (uint32_t) -1;
    prev_fix            = ' ';

    OLED_display_titles = true;
  }

  uint32_t acrfts_counter = Traffic_Count();
  uint32_t sats_counter   = gnss.satellites.value();
  char fix = (isValidGNSSFix()? (leap_seconds_valid()? '+' : '!') : '-');

  if (prev_acrfts_counter != acrfts_counter) {
    disp_value = acrfts_counter > 99 ? 99 : acrfts_counter;
    itoa(disp_value, buf, 10);
    if (disp_value < 10) {
      strcat_P(buf,PSTR(" "));
    }
    u8x8->draw2x2String(1, 1, buf);
    prev_acrfts_counter = acrfts_counter;
  }

  if (prev_sats_counter != sats_counter) {
    disp_value = sats_counter > 99 ? 99 : sats_counter;
    itoa(disp_value, buf, 10);
    if (disp_value < 10) {
      strcat_P(buf,PSTR(" "));
    }
    u8x8->draw2x2String(7, 1, buf);
    prev_sats_counter = sats_counter;
  }

  if (prev_fix != fix) {
    u8x8->draw2x2Glyph(12, 1, fix);
    prev_fix = fix;
  }

  bool show_adsb = false;
  if (settings->rx1090 != ADSB_RX_NONE || settings->gdl90_in != DEST_NONE)
      show_adsb = true;

  if (rx_packets_counter != prev_rx_packets_counter) {
    disp_value = rx_packets_counter % 1000;
    itoa(disp_value, buf, 10);
    if (disp_value < 10) {
      strcat_P(buf,PSTR("  "));
    } else {
      if (disp_value < 100) {
        strcat_P(buf,PSTR(" "));
      };
    }
    //u8x8->draw2x2String(10, ((settings->rx1090 || (settings->gdl90_in != DEST_NONE))? 4 : 5), buf);
    u8x8->draw2x2String(10, 4, buf);
    if (! show_adsb) {                     // then show max RSSI instead
      if (maxrssi != prev_maxrssi) {
        if (maxrssi < 0) {
          disp_value = maxrssi;
          if (disp_value < -99)
              disp_value = -99;
          itoa(disp_value, buf, 10);
          if (disp_value > -10)
              strcat_P(buf,PSTR(" "));
          u8x8->draw2x2String(10, 6, buf);
        } else {
          u8x8->draw2x2String(10, 6, "---");
        }
        prev_maxrssi = maxrssi;
      }
    }
    prev_rx_packets_counter = rx_packets_counter;
  }

  if (show_adsb) {
    if (adsb_packets_counter != prev_adsb_packets_counter || prev_rx1090found != rx1090found) {
      if (rx1090found) {
          disp_value = adsb_packets_counter % 1000;
          itoa(disp_value, buf, 10);
          if (disp_value < 10) {
              strcat_P(buf,PSTR("  "));
          } else if (disp_value < 100) {
              strcat_P(buf,PSTR(" "));
          }
          u8x8->draw2x2String(10, 6, buf);
      } else {
          u8x8->draw2x2String(10, 6, "---");
      }
      prev_adsb_packets_counter = adsb_packets_counter;
      prev_rx1090found = rx1090found;
    }
  }

  if (tx_packets_counter > 0 && settings->txpower == RF_TX_POWER_OFF) {   // for winch mode
    u8x8->draw2x2String(0, 5, "OFF");
    prev_tx_packets_counter = tx_packets_counter = 0;
  } else if (tx_packets_counter != prev_tx_packets_counter) {
    disp_value = tx_packets_counter % 1000;
    itoa(disp_value, buf, 10);
    if (disp_value < 10) {
        strcat_P(buf,PSTR("  "));
    } else if (disp_value < 100) {
        strcat_P(buf,PSTR(" "));
    }
    u8x8->draw2x2String(0, 5, buf);
    prev_tx_packets_counter = tx_packets_counter;
  }
}

#if !defined(EXCLUDE_OLED_BARO_PAGE)
static void OLED_baro()
{
  char buf[16];

  if (!OLED_display_titles) {

    u8x8->clear();

    u8x8->drawString( 2, 1, ALT_text);

    u8x8->drawString( 10, 1, TEMP_text);

    u8x8->drawString( 1, 5, PRES_text);

    u8x8->drawString( 9, 5, CDR_text);

    prev_altitude     = (int32_t)   -10000;
    prev_temperature  = prev_altitude;
    prev_pressure     = (uint32_t)  -1;
    prev_cdr          = prev_altitude;

    OLED_display_titles = true;
  }

  int32_t altitude    = Baro_altitude();        /* metres */
  int32_t temperature = Baro_temperature();     /* Celcius */
  uint32_t pressure   = Baro_pressure() / 100;  /* mbar */
  int32_t cdr         = ThisAircraft.vs;        /* feet per minute */

  if (prev_altitude != altitude) {
    snprintf(buf, sizeof(buf), "%4d", altitude);
    u8x8->draw2x2String(0, 2, buf);
    prev_altitude = altitude;
  }

  if (prev_temperature != temperature) {
    snprintf(buf, sizeof(buf), "%3d", temperature);
    u8x8->draw2x2String(10, 2, buf);
    prev_temperature = temperature;
  }

  if (prev_pressure != pressure) {
    snprintf(buf, sizeof(buf), "%4d", pressure);
    u8x8->draw2x2String(0, 6, buf);
    prev_pressure = pressure;
  }

  if (prev_cdr != cdr) {
    int disp_value = constrain(cdr, -999, 999);
    snprintf(buf, sizeof(buf), "%3d", abs(disp_value));
    u8x8->drawGlyph    ( 9, 6, disp_value < 0 ? '_' : ' ');
    u8x8->draw2x2String(10, 6, buf);
    prev_cdr = cdr;
  }
}
#endif /* EXCLUDE_OLED_BARO_PAGE */

#if !defined(EXCLUDE_OLED_WIFI_PAGE)
static void OLED_wifi()
{
  char buf[16];

  if (!OLED_display_titles) {

    u8x8->clear();

    u8x8->drawString( 0, 2, "WiFi SSID:");
    if (WiFi.getMode() == WIFI_STA)
        u8x8->drawString( 0, 3, WiFi.SSID().c_str());
    else if (WiFi.getMode() == WIFI_AP)
        u8x8->drawString( 0, 3, host_name.c_str());
    else
        u8x8->drawString( 2, 3, "--------");

    u8x8->drawString( 0, 5, "IP address:");
    if (WiFi.getMode() == WIFI_STA) {
        if(WiFi.status() == WL_CONNECTED)
            u8x8->drawString( 0, 6, WiFi.localIP().toString().c_str());
        else
            u8x8->drawString( 0, 6, "-not connected-");
    } else if (WiFi.getMode() == WIFI_AP) {
        u8x8->drawString( 0, 6, WiFi.softAPIP().toString().c_str());
    } else {
        u8x8->drawString( 2, 6, "--------");
    }

    OLED_display_titles = true;
  }
}
#endif /* EXCLUDE_OLED_WIFI_PAGE */

#if !defined(EXCLUDE_OLED_ACFT_PAGE)
static void OLED_acft()
{
  static int prev_dist = -1;
  static int prev_alt = 9999;

  static uint32_t next_ms = 0;
  if ((OLED_display_titles == true || prev_min >= 0) && millis() < next_ms)
    return;
  next_ms = millis() + 3000;

  static int prev_i = -1;
  if (OLED_display_titles == false)
      prev_i = -1;                   // inspect Container[0] first
  if (prev_i < 0)
      OLED_display_titles = false;  // for transition from no-traffic to traffic

  char buf[16];
  int age;
  int i = prev_i + 1;
  if (i >= MAX_TRACKING_OBJECTS)
      i = 0;      // wrap around to the beginning of Container[]
  int j = i;     // remember where we started
  bool found = false;
  do {
    age = OurTime - Container[i].timestamp;
    if (Container[i].addr != 0 && age < ENTRY_EXPIRATION_TIME) {
        // an(other) aircraft to show
        found = true;
        break;
    }
    ++i;
    if (i >= MAX_TRACKING_OBJECTS)
        i = 0;   // wrap around to the beginning of Container[]
  } while (i != j);

  int minute = gnss.time.minute();
  if (! found) {     // no aircraft to show
      prev_i = -1;
      if (prev_min != minute) {
          u8x8->clear();
          u8x8->drawString(5, 1, "UTC");
          snprintf (buf, sizeof(buf), "%02d:%02d", gnss.time.hour(), minute);
          u8x8->draw2x2String(2, 2, buf);
          prev_min = minute;
          u8x8->drawString(2, 6, "NO TRAFFIC");
      }
      prev_dist = -1;
      prev_alt = 9999;
      //OLED_display_titles = true;   // wait until next_ms
      return;
  }
  prev_min = -1;

  int dist = (int) (Container[i].distance * 0.001);    // kilometers
  int rel_alt = (int) (Container[i].alt_diff * 0.01);  // hundreds of meters

  if (OLED_display_titles) {
      if (i == prev_i) {
          if (dist != prev_dist) {
              snprintf (buf, sizeof(buf), "%dkm", dist);
              u8x8->drawString(1, 7, "     ");
              u8x8->drawString(1, 7, buf);
              prev_dist = dist;
          }
          if (rel_alt != prev_alt) {
              snprintf (buf, sizeof(buf), "%s%dm", (rel_alt < 0 ? "-" : "+"), 100*abs(rel_alt));
              u8x8->drawString(6, 7, "      ");
              u8x8->drawString(6, 7, buf);
              prev_alt = rel_alt;
          }
          snprintf (buf, sizeof(buf), "%ds", age);
          u8x8->drawString( 13, 4, "   ");
          u8x8->drawString( 13, 4, buf);
          return;         // nothing else needs changing in the display
      }
  }

  prev_i = i;

  if (!OLED_display_titles) {
      u8x8->clear();
      u8x8->drawString(1, 1, "ID:");
      u8x8->drawString(1, 3, "TYPE:");
      u8x8->drawString(1, 5, "PROT:");
      u8x8->drawString(14, 6, "#");
      //u8x8->drawString(1, 7, "KM:");
      prev_dist = -1;
      prev_alt = 9999;
      OLED_display_titles = true;
  }

  snprintf (buf, sizeof(buf), "%d", i);
  u8x8->drawString( 15, 6, buf);

  snprintf (buf, sizeof(buf), "%ds", age);
  u8x8->drawString( 13, 4, "   ");
  u8x8->drawString( 13, 4, buf);

  if (dist != prev_dist) {
      snprintf (buf, sizeof(buf), "%dkm", dist);
      u8x8->drawString(1, 7, "     ");
      u8x8->drawString(1, 7, buf);
      prev_dist = dist;
  }
  if (rel_alt != prev_alt) {
      snprintf (buf, sizeof(buf), "%s%dm", (rel_alt < 0 ? "-" : "+"), 100*abs(rel_alt));
      u8x8->drawString(6, 7, "      ");
      u8x8->drawString(6, 7, buf);
      prev_alt = rel_alt;
  }

  snprintf (buf, sizeof(buf), "%06X", Container[i].addr);
  u8x8->drawString(7, 1, buf);
  u8x8->drawString(7, 3, aircraft_type_lbl[Container[i].aircraft_type]);
  u8x8->drawString(7, 5, Protocol_ID[Container[i].protocol]);
}
#endif /* EXCLUDE_OLED_ACFT_PAGE */


#if !defined(EXCLUDE_OLED_049)

void OLED_049_func()
{
  char buf[16];
  uint32_t disp_value;
  uint32_t acrfts_counter;
  uint32_t sats_counter;
  uint32_t uptime_minutes;
  int32_t  voltage;

  switch (OLED_current_page)
  {
  case OLED_049_PAGE_ID:
    if (!OLED_display_titles) {
      u8x8->clear();
      u8x8->drawString(5, 4, ID_text);
      snprintf (buf, sizeof(buf), "%06X", ThisAircraft.addr);
      u8x8->draw2x2Glyph ( 8, 4, buf[0]);
      u8x8->draw2x2Glyph (10, 4, buf[1]);
      u8x8->draw2x2String( 4, 6, buf+2);

      OLED_display_titles = true;
    }

    break;

  case OLED_049_PAGE_PROTOCOL:
    if (!OLED_display_titles) {
      u8x8->clear();
      u8x8->drawString(4, 4, PROTOCOL_text);
      u8x8->draw2x2String(5, 6, Protocol_ID[ThisAircraft.protocol]);

      OLED_display_titles = true;
    }

    break;

  case OLED_049_PAGE_RX:
    if (!OLED_display_titles) {
      u8x8->clear();

      u8x8->drawString(5, 4, RX_text);

      if (settings->power_save & POWER_SAVE_NORECEIVE &&
          (hw_info.rf == RF_IC_SX1276 || hw_info.rf == RF_IC_SX1262)) {
        u8x8->draw2x2String(5, 6, "OFF");
        prev_rx_packets_counter = rx_packets_counter;
      } else {
        prev_rx_packets_counter = (uint32_t) -1;
      }

      OLED_display_titles = true;
    }

    if (rx_packets_counter != prev_rx_packets_counter) {
      disp_value = rx_packets_counter % 1000;
      itoa(disp_value, buf, 10);

      if (disp_value < 10) {
        strcat_P(buf,PSTR("  "));
      } else {
        if (disp_value < 100) {
          strcat_P(buf,PSTR(" "));
        };
      }

      u8x8->draw2x2String(5, 6, buf);
      prev_rx_packets_counter = rx_packets_counter;
    }

    break;

  case OLED_049_PAGE_SATS_TX:
    if (!OLED_display_titles) {
      u8x8->clear();

      u8x8->drawString( 4, 4, SATS_text);
      prev_sats_counter   = (uint32_t) -1;

      u8x8->drawString(10, 4, TX_text);

      if (settings->mode        == SOFTRF_MODE_RECEIVER ||
          settings->rf_protocol == RF_PROTOCOL_ADSB_UAT ||
          settings->txpower     == RF_TX_POWER_OFF) {
        u8x8->draw2x2String(8, 6, "NA");
        prev_tx_packets_counter = tx_packets_counter;
      } else {
        prev_tx_packets_counter = (uint32_t) -1;
      }

      OLED_display_titles = true;
    }

    sats_counter   = gnss.satellites.value();

    if (prev_sats_counter != sats_counter) {
      disp_value = sats_counter > 9 ? 9 : sats_counter;

      u8x8->draw2x2Glyph(4, 6, '0' + disp_value);
      prev_sats_counter = sats_counter;
    }

    if (tx_packets_counter != prev_tx_packets_counter) {
      disp_value = tx_packets_counter % 100;
      itoa(disp_value, buf, 10);

      if (disp_value < 10) {
        strcat_P(buf,PSTR(" "));
      } else {
      }

      u8x8->draw2x2String(8, 6, buf);
      prev_tx_packets_counter = tx_packets_counter;
    }

    break;

  case OLED_049_PAGE_ACFTS:
    if (!OLED_display_titles) {
      u8x8->clear();
      u8x8->drawString( 5, 4, ACFTS_text);
      prev_acrfts_counter = (uint32_t) -1;

      OLED_display_titles = true;
    }

    acrfts_counter = Traffic_Count();

    if (prev_acrfts_counter != acrfts_counter) {
      disp_value = acrfts_counter > 99 ? 99 : acrfts_counter;
      itoa(disp_value, buf, 10);

      if (disp_value < 10) {
        strcat_P(buf,PSTR(" "));
      }

      u8x8->draw2x2String(5, 6, buf);
      prev_acrfts_counter = acrfts_counter;
    }

    break;

  case OLED_049_PAGE_UPTIME:
    if (!OLED_display_titles) {
      u8x8->clear();
      u8x8->drawString( 5, 4, UPTIME_text);
      u8x8->drawTile  (7, 6, 1, (uint8_t *) Dot_Tile);
      u8x8->drawTile  (7, 7, 1, (uint8_t *) Dot_Tile);
      prev_uptime_minutes = (uint32_t) -1;

      OLED_display_titles = true;
    }

    uptime_minutes = millis() / 60000;

    if (prev_uptime_minutes != uptime_minutes) {
      uint32_t uptime_hours = uptime_minutes / 60;
      disp_value = uptime_hours % 100;
      itoa(disp_value, buf, 10);

      u8x8->draw2x2String(5, 6, buf);

      disp_value = uptime_minutes % 60;
      if (disp_value < 10) {
        buf[0] = '0';
        itoa(disp_value, buf+1, 10);
      } else {
        itoa(disp_value, buf, 10);
      }

      u8x8->draw2x2String(8, 6, buf);

      prev_uptime_minutes = uptime_minutes;
    }

    break;

  case OLED_049_PAGE_VOLTAGE:
    if (!OLED_display_titles) {
      u8x8->clear();
      u8x8->drawString(5, 4, BAT_text);
      u8x8->drawGlyph (7, 7, '.');
      prev_voltage        = (uint32_t) -1;

      OLED_display_titles = true;
    }

    voltage = Battery_voltage() > BATTERY_THRESHOLD_INVALID ?
              (int) (Battery_voltage() * 10.0) : 0;

    if (prev_voltage != voltage) {
      if (voltage) {
        disp_value = voltage / 10;
        disp_value = disp_value > 9 ? 9 : disp_value;
        u8x8->draw2x2Glyph(5, 6, '0' + disp_value);

        disp_value = voltage % 10;

        u8x8->draw2x2Glyph(8, 6, '0' + disp_value);
      } else {
        u8x8->draw2x2Glyph(5, 6, 'N');
        u8x8->draw2x2Glyph(8, 6, 'A');
      }
      prev_voltage = voltage;
    }

    break;

  default:
    break;
  }
}

#endif /* EXCLUDE_OLED_049 */

void OLED_loop()
{
  if (showing_message)
      return;

  if (u8x8) {
    if (isTimeToOLED()) {              // every 500 ms
#if !defined(EXCLUDE_OLED_049)
      if (hw_info.display == DISPLAY_OLED_0_49) {
        OLED_049_func();
      } else
#endif /* EXCLUDE_OLED_049 */
        switch (OLED_current_page)
        {
        case OLED_PAGE_RADIO:
          OLED_radio();
          break;
#if !defined(EXCLUDE_OLED_BARO_PAGE)
        case OLED_PAGE_BARO:
          OLED_baro();
          break;
#endif /* EXCLUDE_OLED_BARO_PAGE */
#if !defined(EXCLUDE_OLED_WIFI_PAGE)
        case OLED_PAGE_WIFI:
          OLED_wifi();
          break;
#endif /* EXCLUDE_OLED_WIFI_PAGE */
#if !defined(EXCLUDE_OLED_ACFT_PAGE)
        case OLED_PAGE_ACFT:
          OLED_acft();
          break;
#endif /* EXCLUDE_OLED_ACFT_PAGE */
        case OLED_PAGE_SETTINGS:
        default:
          OLED_settings();
          break;
        }

      OLEDTimeMarker = millis();
    }
  }
}

void OLED_fini(int reason)
{
  if (u8x8) {
    u8x8->clear();
    switch (hw_info.display)
    {
#if !defined(EXCLUDE_OLED_049)
    case DISPLAY_OLED_0_49:
      if (reason != SOFTRF_SHUTDOWN_CHARGE) {
        u8x8->draw2x2String(5, 5, reason == SOFTRF_SHUTDOWN_LOWBAT ? "BAT"
                                : reason == SOFTRF_SHUTDOWN_EXTPWR ? "EXT" : "OFF");
        delay(2000);
      }
      u8x8->noDisplay();
      break;
#endif /* EXCLUDE_OLED_049 */
    case DISPLAY_OLED_TTGO:
    case DISPLAY_OLED_HELTEC:
    case DISPLAY_OLED_1_3:
    default:
      if (reason != SOFTRF_SHUTDOWN_CHARGE) {
        u8x8->draw2x2String(1, 3, reason == SOFTRF_SHUTDOWN_LOWBAT ? "LOW BAT"
                                : reason == SOFTRF_SHUTDOWN_EXTPWR ? "EXT PWR" : "  OFF  ");
      }
      break;
    }
  }
}

void OLED_msg(const char *msg1, const char *msg2)
{
  if (u8x8) {
    u8x8->clear();
    switch (hw_info.display)
    {
#if !defined(EXCLUDE_OLED_049)
    case DISPLAY_OLED_0_49:
      if (msg1)
        u8x8->draw2x2String(5, 3, msg1);
      if (msg2)
      u8x8->draw2x2String(5, 6, msg2);
      break;
#endif /* EXCLUDE_OLED_049 */
    case DISPLAY_OLED_TTGO:
    case DISPLAY_OLED_HELTEC:
    case DISPLAY_OLED_1_3:
    default:
      if (msg1)
        u8x8->draw2x2String(1, 2, msg1);
      if (msg2)
        u8x8->draw2x2String(1, 4, msg2);
      break;
    }
    showing_message = true;
  }
}

void OLED_no_msg()
{
    showing_message = false;
    OLED_display_titles = false;   // redraw same page from before message
}

void OLED_info1()
{
  if (u8x8) {

    u8x8->clear();

    switch (hw_info.display)
    {
#if !defined(EXCLUDE_OLED_049)
    case DISPLAY_OLED_0_49:
      {
        u8x8->draw2x2Glyph(  4, 4, 'R');
        u8x8->draw2x2Glyph(  6, 4, hw_info.rf      != RF_IC_NONE       ? '+' : '-');
        u8x8->draw2x2Glyph(  8, 4, 'G');
        u8x8->draw2x2Glyph( 10, 4, hw_info.gnss    != GNSS_MODULE_NONE ? '+' : '-');
        u8x8->draw2x2Glyph(  4, 6, 'O');
        u8x8->draw2x2Glyph(  6, 6, hw_info.display != DISPLAY_NONE     ? '+' : '-');
        u8x8->draw2x2Glyph(  8, 6, 'I');
        u8x8->draw2x2Glyph( 10, 6, hw_info.imu     != IMU_NONE         ? '+' : '-');

        delay(3000);

        const char buf[] = SOFTRF_FIRMWARE_VERSION;
        int ndx = strlen(buf) - 3;
        ndx = ndx < 0 ? 0 : ndx;
        u8x8->clear();
        u8x8->drawString  (4, 4, "VERSION");
        u8x8->draw2x2Glyph(5, 6, toupper(buf[ndx++]));
        u8x8->draw2x2Glyph(7, 6, toupper(buf[ndx++]));
        u8x8->draw2x2Glyph(9, 6, toupper(buf[ndx]));

        delay(2000);

        u8x8->clear();
        u8x8->drawString   (4, 4, "REGION");
        u8x8->draw2x2String(6, 6, ISO3166_CC[settings->band]);
      }
      break;
#endif /* EXCLUDE_OLED_049 */
    case DISPLAY_OLED_TTGO:
    case DISPLAY_OLED_HELTEC:
    case DISPLAY_OLED_1_3:
    default:

      u8x8->draw2x2String( 0, 0, "RADIO");
      u8x8->draw2x2String(14, 0, hw_info.rf   != RF_IC_NONE       ? "+" : "-");
      u8x8->draw2x2String( 0, 2, "GNSS");
      u8x8->draw2x2String(14, 2, hw_info.gnss != GNSS_MODULE_NONE ? "+" : "-");
      u8x8->draw2x2String( 0, 4, "OLED");
      u8x8->draw2x2String(14, 4, hw_info.display != DISPLAY_NONE  ? "+" : "-");
      u8x8->draw2x2String( 0, 6, "BARO");
      u8x8->draw2x2String(14, 6, hw_info.baro != BARO_MODULE_NONE ? "+" : "-");

      break;
    }

    delay(3000);
  }
}

void OLED_info2()
{
  if (u8x8) {

    u8x8->clear();

    switch (hw_info.display)
    {
    case DISPLAY_OLED_TTGO:
    case DISPLAY_OLED_HELTEC:
    case DISPLAY_OLED_1_3:
    default:

      u8x8->draw2x2String( 0, 0, "RTC");
      u8x8->draw2x2String(14, 0, hw_info.rtc != RTC_NONE ? "+" : "-");
      u8x8->draw2x2String( 0, 2, "IMU");
      u8x8->draw2x2String(14, 2, hw_info.imu != IMU_NONE ? "+" : "-");
      u8x8->draw2x2String( 0, 4, "MAG");
      u8x8->draw2x2String(14, 4, hw_info.mag != MAG_NONE ? "+" : "-");
      u8x8->draw2x2String( 0, 6, "CARD");
      u8x8->draw2x2String(14, 6, hw_info.storage == STORAGE_CARD ||
                                 hw_info.storage == STORAGE_FLASH_AND_CARD ?
                                                           "+" : "-");
      break;
    }

    delay(3000);
  }
}

void OLED_info3(int acfts, char *reg, char *mam, char *cn)
{
  if (u8x8) {

    u8x8->clear();

    switch (hw_info.display)
    {
    case DISPLAY_OLED_TTGO:
    case DISPLAY_OLED_HELTEC:
    case DISPLAY_OLED_1_3:
    default:

      if (acfts == -1) {
        u8x8->draw2x2String( 6, 1, "NO");
        u8x8->draw2x2String( 0, 3, "AIRCRAFT");
        u8x8->draw2x2String( 4, 5, "DATA");
      } else {
        char str1[9], str2[9], str3[9], str4[9];

        memset(str1, 0, sizeof(str1));
        memset(str2, 0, sizeof(str2));
        memset(str3, 0, sizeof(str3));
        memset(str4, 0, sizeof(str4));

        snprintf(str1, 6, "%d", acfts);
        strncpy (str2, reg, 8);
        strncpy (str3, mam, 8);
        strncpy (str4,  cn, 8);

        u8x8->draw2x2String( 4, 0, str1);
        u8x8->draw2x2String( 0, 2, str2);
        u8x8->draw2x2String( 0, 4, str3);
        u8x8->draw2x2String( 0, 6, str4);
      }

      break;
    }

    delay(3000);
  }
}

void OLED_Next_Page()
{
  if (u8x8) {

    if (showing_message) {
        //Serial.println("leaving OLED message mode");
        showing_message = false;       // escape message mode
        OLED_display_titles = false;   // redraw same page from before message
        return;
    }

    OLED_current_page = (OLED_current_page + 1) % page_count;

#if !defined(EXCLUDE_OLED_BARO_PAGE)
    if (hw_info.display   != DISPLAY_OLED_0_49 &&
        OLED_current_page == OLED_PAGE_BARO    &&
        hw_info.baro      == BARO_MODULE_NONE) {
      OLED_current_page = (OLED_current_page + 1) % page_count;
    }
#endif /* EXCLUDE_OLED_BARO_PAGE */

#if !defined(EXCLUDE_OLED_049)
    if (hw_info.display   == DISPLAY_OLED_0_49      &&
        OLED_current_page == OLED_049_PAGE_ACFTS    &&
        settings->power_save & POWER_SAVE_NORECEIVE &&
        (hw_info.rf == RF_IC_SX1276 || hw_info.rf == RF_IC_SX1262)) {
      OLED_current_page = (OLED_current_page + 1) % page_count;
    }
#endif /* EXCLUDE_OLED_049 */

    OLED_display_titles = false;
    prev_min = -1;
  }
}

#endif /* USE_OLED */
