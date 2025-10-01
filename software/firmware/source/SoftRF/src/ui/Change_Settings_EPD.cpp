/*
 * Change_Settings.cpp
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

#include "../system/SoC.h"

#if defined(USE_EPAPER)

#include "../driver/EPD.h"

#include <TimeLib.h>

#include "../TrafficHelper.h"
#include "../driver/Settings.h"
#include "../protocol/data/NMEA.h"
#include "../protocol/data/GDL90.h"
#include "../driver/GNSS.h"
#include "../driver/LED.h"
#include "../driver/RF.h"

#include <protocol.h>
#include "../protocol/radio/Legacy.h"

#include <gfxfont.h>
#include <FreeMonoBold12pt7b.h>


struct set_entry
{
    const int code;
    const char *label;
};

set_entry actypes[] = {
  {AIRCRAFT_TYPE_GLIDER,     "Glider"},
  {AIRCRAFT_TYPE_TOWPLANE,   "Towplane"},
  {AIRCRAFT_TYPE_HELICOPTER, "Helicopter"},
  {AIRCRAFT_TYPE_POWERED,    "Powered"},
  {AIRCRAFT_TYPE_HANGGLIDER, "Hangglider"},
  {AIRCRAFT_TYPE_PARAGLIDER, "Paraglider"},
  {AIRCRAFT_TYPE_DROPPLANE,  "Dropplane"},
  {AIRCRAFT_TYPE_PARACHUTE,  "Parachute"},
  {AIRCRAFT_TYPE_BALLOON,    "Balloon"},
  {AIRCRAFT_TYPE_UAV,        "UAV"},
  {AIRCRAFT_TYPE_STATIC,     "Static"},
  {AIRCRAFT_TYPE_WINCH,      "Winch"},
  {-1, NULL}
};

set_entry protocols[] = {
  {RF_PROTOCOL_LEGACY,    "LEGACY"},
  {RF_PROTOCOL_LATEST,    "LATEST"},
  {RF_PROTOCOL_OGNTP,     "OGNTP"},
  {RF_PROTOCOL_P3I,       "P3I"},
  {RF_PROTOCOL_FANET,     "FANET"},
  {RF_PROTOCOL_ADSL,      "ADS-L"},
//  {RF_PROTOCOL_ADSB_UAT,  "ADSB-UAT"},
//  {RF_PROTOCOL_ADSB_1090, "ADSB-1090"},
  {-1, NULL}
};

set_entry regions[] = {
  {RF_BAND_EU, "EU"},
  {RF_BAND_US, "US"},
  {RF_BAND_UK, "UK"},
  {RF_BAND_AU, "AU"},
  {RF_BAND_NZ, "NZ"},
  {RF_BAND_RU, "RU"},
  {RF_BAND_CN, "CN"},
  {RF_BAND_IN, "IN"},
  {RF_BAND_IL, "IL"},
  {RF_BAND_KR, "KR"},
  {-1, NULL}
};

set_entry alarms[] = {
  {TRAFFIC_ALARM_LATEST,   "Latest"},
  {TRAFFIC_ALARM_VECTOR,   "Vector"},
  {TRAFFIC_ALARM_DISTANCE, "Distance"},
  {TRAFFIC_ALARM_NONE,     "None"},
  {-1, NULL}
};

set_entry units[] = {
  {UNITS_METRIC,   "Metric"},
  {UNITS_IMPERIAL, "Imperial"},
  {UNITS_MIXED,    "Mixed"},
  {-1, NULL}
};

set_entry directions[] = {
  {DIRECTION_TRACK_UP, "Track Up"},
  {DIRECTION_NORTH_UP, "North Up"},
  {-1, NULL}
};

set_entry relays[] = {
  {RELAY_OFF, "None"},
  {RELAY_LANDED, "Landed"},
//{RELAY_ALL, "ADS-B"},
//{RELAY_ONLY, "Only"},
  {-1, NULL}
};

set_entry idtypes[] = {
  {ADDR_TYPE_FLARM,     "Device"},
  {ADDR_TYPE_ICAO,      "ICAO"},
  {ADDR_TYPE_ANONYMOUS, "Anonymous"},
  {ADDR_TYPE_RANDOM,    "Random"},
  {-1, NULL}
};

set_entry hexdigits[] = {
  {0, "0"},
  {1, "1"},
  {2, "2"},
  {3, "3"},
  {4, "4"},
  {5, "5"},
  {6, "6"},
  {7, "7"},
  {8, "8"},
  {9, "9"},
  {0xA, "A"},
  {0xB, "B"},
  {0xC, "C"},
  {0xD, "D"},
  {0xE, "E"},
  {0xF, "F"},
  {-1, NULL}
};

enum
{
	DECISION_CANCEL = 0,
	DECISION_REVIEW,
	DECISION_SAVE
};

set_entry decisions[] = {
  {DECISION_CANCEL, "cancel"},
  {DECISION_REVIEW, "review"},
  {DECISION_SAVE,  "! SAVE !"},
  {-1, NULL}
};

static int decision = 0;
static int actype = 0;
static int protocol = 0;
static int region = 0;
static int alarm = 0;
static int unit = 0;
static int direction = 0;
static int relay = 0;
static int idtype = 0;
static int id1 = 0;
static int id2 = 0;
static int id3 = 0;
static int id4 = 0;
static int id5 = 0;
static int id6 = 0;

/* search for a given code and return the index */
int get_one_setting(int setting, set_entry *list)
{
    int i = 0;
    while (list[i].code != setting) {
        ++i;
        if (list[i].code < 0)
            return 0;
    }
    return i;
}

struct page {
    int *indexvar;
    set_entry *options;
    const char *line1;
    const char *line2;
    const char *line3;    
};

page pages[] = {
  {&decision, decisions, NULL, "what to", "do next:"},
  {&actype, actypes, NULL, "Aircraft", "Type:"},
  {&protocol, protocols, NULL, "RF", "Protocol:"},
  {&region, regions, NULL, "Frequency", "Band:"},
  {&alarm, alarms, "Collision", "Prediction", "Algorithm:"},
  {&relay, relays, NULL, "Air", "Relay:"},
  {&unit, units, NULL, "Display", "Units:"},
  {&direction, directions, NULL, "Display", "Orientation:"},
  {&idtype, idtypes, NULL, "Acft ID", "Type:"},
  {&id1, hexdigits, "Acft ID", "hex digit", "#1 (X-----):"},
  {&id2, hexdigits, "Acft ID", "hex digit", "#2 (-X----):"},
  {&id3, hexdigits, "Acft ID", "hex digit", "#3 (--X---):"},
  {&id4, hexdigits, "Acft ID", "hex digit", "#4 (---X--):"},
  {&id5, hexdigits, "Acft ID", "hex digit", "#5 (----X-):"},
  {&id6, hexdigits, "Acft ID", "hex digit", "#6 (-----X):"},
  {NULL, NULL, NULL, NULL, NULL}
};

int curpage = 0;

static bool chgconf_initialized = false;
static bool chgconf_changed = false;

void get_settings()
{
    if (chgconf_initialized)
        return;
    actype    = get_one_setting((int) settings->acft_type, actypes);
    protocol  = get_one_setting((int) settings->rf_protocol, protocols);
    region    = get_one_setting((int) settings->band, regions);
    alarm     = get_one_setting((int) settings->alarm, alarms);
    relay     = get_one_setting((int) settings->relay, relays);
    idtype    = get_one_setting((int) settings->id_method, idtypes);
    uint32_t id = settings->aircraft_id;
    id1 = ((id & 0x00F00000) >> 20);
    id2 = ((id & 0x000F0000) >> 16);
    id3 = ((id & 0x0000F000) >> 12);
    id4 = ((id & 0x00000F00) >> 8);
    id5 = ((id & 0x000000F0) >> 4);
    id6 =  (id & 0x0000000F);
    unit      = get_one_setting((int) ui->units, units);
    direction = get_one_setting((int) ui->orientation, directions);
    decision = 0;  // cancel
    curpage = 1;   // actypes
    chgconf_initialized = true;
    chgconf_changed = true;
}

void EPD_chgconf_save()
{
    settings->acft_type   = actypes[actype].code;
    settings->rf_protocol = protocols[protocol].code;
    settings->band        = regions[region].code;
    settings->alarm       = alarms[alarm].code;
    settings->relay       = relays[relay].code;
    settings->id_method   = idtypes[idtype].code;
    uint32_t id = id6;
    id |= (id5 << 4);
    id |= (id4 << 8);
    id |= (id3 << 12);
    id |= (id2 << 16);
    id |= (id1 << 20);
    settings->aircraft_id = id;
    ui->units       = units[unit].code;
    ui->orientation = directions[direction].code;
    SoC->WDT_fini();
    if (SoC->Bluetooth_ops) { SoC->Bluetooth_ops->fini(); }
    //EEPROM_store();
    save_settings_to_file();
}

/*
 * Scroll to the next page, i.e., next item to be adjusted.
 * Tied to the Mode button.
 */
void EPD_chgconf_page()
{
    if (EPD_view_mode != VIEW_CHANGE_SETTINGS)
        return;
    if (curpage == 0) {  // pages[curpage].indexvar == &decision
        if (decision == DECISION_CANCEL) {
            chgconf_initialized = false;
            EPD_view_mode = VIEW_MODE_CONF;
            conf_initialized = false;
            return;
        }
        if (decision == DECISION_SAVE) {
            EPD_view_mode = VIEW_SAVE_SETTINGS;
            return;
        }
    }
    ++curpage;
    if (pages[curpage].indexvar == NULL)
        curpage = 0;
    chgconf_changed = true;
//Serial.print("curpage: ");
//Serial.println(curpage);
}

/*
 * Scroll to the next value available for this item.
 * Tied to the Touch button.
 */
void EPD_chgconf_next()
{
    if (EPD_view_mode != VIEW_CHANGE_SETTINGS)
        return;
    int i = *(pages[curpage].indexvar) + 1;
    if (pages[curpage].options[i].code < 0)
        i = 0;
    *(pages[curpage].indexvar) = i;
    chgconf_changed = true;
//Serial.print("index: ");
//Serial.println(i);
}

void EPD_chgconf_prev()
{
    if (EPD_view_mode != VIEW_CHANGE_SETTINGS)
        return;
    int i = *(pages[curpage].indexvar);
    if (i == 0) {
        while (pages[curpage].options[i].code >= 0)
            ++i;
    }
    *(pages[curpage].indexvar) = i-1;
    chgconf_changed = true;
}

static void EPD_Draw_chgconf()
{

#if defined(USE_EPD_TASK)
    if (EPD_update_in_progress != EPD_UPDATE_NONE)
        return;
#endif

    display->setFont(&FreeMonoBold12pt7b);

      uint16_t x = 4;
      uint16_t y = 20;

      int16_t  tbx, tby;
      uint16_t tbw, tbh;

      display->fillScreen(GxEPD_WHITE);

      Serial.println();

      const char *line;

      line = pages[curpage].line1;
      if (line == NULL) {
          display->getTextBounds("dummy", 0, 0, &tbx, &tby, &tbw, &tbh);
          y += tbh;
      } else {
          display->getTextBounds(line, 0, 0, &tbx, &tby, &tbw, &tbh);
          y += tbh;
          display->setCursor(x, y);
          display->print(line);
          Serial.println(line);
      }
      y += TEXT_VIEW_LINE_SPACING;

      line = pages[curpage].line2;
      display->getTextBounds(line, 0, 0, &tbx, &tby, &tbw, &tbh);
      y += tbh;
      display->setCursor(x, y);
      display->print(line);
      Serial.println(line);

      y += TEXT_VIEW_LINE_SPACING;

      line = pages[curpage].line3;
      display->getTextBounds(line, 0, 0, &tbx, &tby, &tbw, &tbh);
      y += tbh;
      display->setCursor(x, y);
      display->print(line);
      Serial.println(line);

      y += TEXT_VIEW_LINE_SPACING + 20;
      x += 20;

      line = pages[curpage].options[*(pages[curpage].indexvar)].label;
      display->getTextBounds(line, 0, 0, &tbx, &tby, &tbw, &tbh);
      y += tbh;
      display->setCursor(x, y);
      display->print(line);
      Serial.println(line);

      Serial.println();

#if defined(USE_EPD_TASK)
    /* a signal to background EPD update task */
    EPD_update_in_progress = EPD_UPDATE_FAST;
#else
    display->display(true);
#endif
}

void EPD_chgconf_loop()
{
  //if (isTimeToEPD()) {
  //    EPDTimeMarker = millis();


      if (EPD_view_mode == VIEW_CHANGE_SETTINGS) {
          get_settings();
          if (chgconf_changed) {
              EPD_Draw_chgconf();
              chgconf_changed = false;
          }

      } else if (EPD_view_mode == VIEW_SAVE_SETTINGS) {
          Serial.println("SAVING SETTINGS...");
          EPD_chgconf_save();
          EPD_Message("SETTINGS", "SAVED");
          Serial.println("...SETTINGS SAVED");
          delay(500);
          EPD_view_mode = VIEW_REBOOT;

      } else if (EPD_view_mode == VIEW_REBOOT) {
          Serial.println("NOW REBOOTING");
          reboot();
          Serial.println(F("This will never be printed."));
      }
  //}
}

#endif /* USE_EPAPER */
