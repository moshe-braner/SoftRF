/*
 * View_Radar_EPD.cpp
 * Copyright (C) 2019-2021 Linar Yusupov
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

// this version follows
//   https://github.com/nbonniere/SoftRF/tree/master/software/firmware/source/SkyView

#include "EPDHelper.h"

#include <Fonts/Picopixel.h>
#include <Fonts/FreeMono9pt7b.h>
#include <Fonts/FreeMonoBold9pt7b.h>
#include <Fonts/FreeSerifBold12pt7b.h>
#include <Fonts/FreeMonoBold18pt7b.h>

#include <TimeLib.h>

#include "TrafficHelper.h"
#include "BatteryHelper.h"
#include "EEPROMHelper.h"
#include "NMEAHelper.h"
#include "GDL90Helper.h"
#include "ApproxMath.h"

#include "SkyView.h"

static navbox_t navbox1;
static navbox_t navbox2;
static navbox_t navbox3;
static navbox_t navbox4;

static int EPD_zoom = ZOOM_MEDIUM;

#define ICON_AIRPLANE

#if defined(ICON_AIRPLANE)
//#define ICON_AIRPLANE_POINTS 6
//const int16_t epd_Airplane[ICON_AIRPLANE_POINTS][2] = {{0,-4},{0,10},{-8,0},{8,0},{-3,8},{3,8}};
#define ICON_AIRPLANE_POINTS 12
const float epd_Airplane[ICON_AIRPLANE_POINTS][2] = {{0,-4},{0,10},{-8,0},{9,0},{-3,8},{4,8},{1,-4},{1,10},{-10,1},{11,1},{-2,9},{3,9}};
#else  //ICON_AIRPLANE
#define ICON_ARROW_POINTS 4
const float epd_Arrow[ICON_ARROW_POINTS][2] = {{-6,5},{0,-6},{6,5},{0,2}};
#endif //ICON_AIRPLANE
#define ICON_TARGETS_POINTS 5
const float epd_Target[ICON_TARGETS_POINTS][2] = {{4,4},{0,-6},{-4,4},{-5,-3},{0,2}};

#define MAX_DRAW_POINTS 12

// 2D rotation
void EPD_2D_Rotate(float &tX, float &tY, float tCos, float tSin)
{
    float tTemp;
    tTemp = tX * tCos - tY * tSin;
    tY = tX * tSin + tY *  tCos;
    tX = tTemp;
}

static void EPD_Draw_NavBoxes()
{
  int16_t  tbx, tby;
  uint16_t tbw, tbh;

  uint16_t top_navboxes_x = navbox1.x;
  uint16_t top_navboxes_y = navbox1.y;
  uint16_t top_navboxes_w = navbox1.width + navbox2.width;
  uint16_t top_navboxes_h = maxof2(navbox1.height, navbox2.height);

  {
    display->fillRect(top_navboxes_x, top_navboxes_y,
                      top_navboxes_w, top_navboxes_h,
                      GxEPD_WHITE);

    display->drawRoundRect( navbox1.x + 1, navbox1.y + 1,
                            navbox1.width - 2, navbox1.height - 2,
                            4, GxEPD_BLACK);

    display->drawRoundRect( navbox2.x + 1, navbox2.y + 1,
                            navbox2.width - 2, navbox2.height - 2,
                            4, GxEPD_BLACK);

    display->setFont(&Picopixel);

    display->getTextBounds(navbox1.title, 0, 0, &tbx, &tby, &tbw, &tbh);
    display->setCursor(navbox1.x + 5, navbox1.y + 5 + tbh);
    display->print(navbox1.title);

    display->getTextBounds(navbox2.title, 0, 0, &tbx, &tby, &tbw, &tbh);
    display->setCursor(navbox2.x + 5, navbox2.y + 5 + tbh);
    display->print(navbox2.title);

    display->setFont(&FreeMonoBold18pt7b);

    display->setCursor(navbox1.x + 40, navbox1.y + 32);
    display->print(navbox1.value);

    display->setFont(&FreeSerifBold12pt7b);

    display->setCursor(navbox2.x + 8, navbox2.y + 30);
    display->print(navbox2.value == PROTOCOL_NMEA  ? "NMEA" :
                   navbox2.value == PROTOCOL_GDL90 ? " GDL" : " UNK" );
  }

  uint16_t bottom_navboxes_x = navbox3.x;
  uint16_t bottom_navboxes_y = navbox3.y;
  uint16_t bottom_navboxes_w = navbox3.width + navbox4.width;
  uint16_t bottom_navboxes_h = maxof2(navbox3.height, navbox4.height);

  {
    display->fillRect(bottom_navboxes_x, bottom_navboxes_y,
                      bottom_navboxes_w, bottom_navboxes_h,
                      GxEPD_WHITE);

    display->drawRoundRect( navbox3.x + 1, navbox3.y + 1,
                            navbox3.width - 2, navbox3.height - 2,
                            4, GxEPD_BLACK);
    display->drawRoundRect( navbox4.x + 1, navbox4.y + 1,
                            navbox4.width - 2, navbox4.height - 2,
                            4, GxEPD_BLACK);

    display->setFont(&Picopixel);

    display->getTextBounds(navbox3.title, 0, 0, &tbx, &tby, &tbw, &tbh);
    display->setCursor(navbox3.x + 5, navbox3.y + 5 + tbh);
    display->print(navbox3.title);

    display->getTextBounds(navbox4.title, 0, 0, &tbx, &tby, &tbw, &tbh);
    display->setCursor(navbox4.x + 5, navbox4.y + 5 + tbh);
    display->print(navbox4.title);

    display->setFont(&FreeSerifBold12pt7b);

    display->setCursor(navbox3.x + 10, navbox3.y + 30);

    if (settings->units == UNITS_METRIC || settings->units == UNITS_MIXED) {
      display->print(navbox3.value == ZOOM_LOWEST ? "10 KM" :
                     navbox3.value == ZOOM_LOW    ? " 5 KM" :
                     navbox3.value == ZOOM_MEDIUM ? " 2 KM" :
                     navbox3.value == ZOOM_HIGH   ? " 1 KM" : "");
    } else {
      display->print(navbox3.value == ZOOM_LOWEST ? " 5 NM" :
                     navbox3.value == ZOOM_LOW    ? "2.5NM" :
                     navbox3.value == ZOOM_MEDIUM ? " 1 NM" :
                     navbox3.value == ZOOM_HIGH   ? "0.5NM" : "");
    }

    display->setFont(&FreeMonoBold18pt7b);

    display->setCursor(navbox4.x + 15, navbox4.y + 32);
    display->print((float) navbox4.value / 10);
  }
}

void EPD_radar_Draw_Message(const char *msg1, const char *msg2)
{
  int16_t  tbx, tby;
  uint16_t tbw, tbh;
  uint16_t x, y;

  if (msg1 != NULL && strlen(msg1) != 0) {
    uint16_t radar_x = 0;
    uint16_t radar_y = (display->height() - display->width()) / 2;
    uint16_t radar_w = display->width();

    display->setFont(&FreeMonoBold18pt7b);

    {
      display->fillRect(radar_x, radar_y, radar_w, radar_w, GxEPD_WHITE);

      if (msg2 == NULL) {
        display->getTextBounds(msg1, 0, 0, &tbx, &tby, &tbw, &tbh);
        x = (radar_w - tbw) / 2;
        y = radar_y + (radar_w + tbh) / 2;
        display->setCursor(x, y);
        display->print(msg1);
      } else {
        display->getTextBounds(msg1, 0, 0, &tbx, &tby, &tbw, &tbh);
        x = (radar_w - tbw) / 2;
        y = radar_y + radar_w / 2 - tbh;
        display->setCursor(x, y);
        display->print(msg1);

        display->getTextBounds(msg2, 0, 0, &tbx, &tby, &tbw, &tbh);
        x = (radar_w - tbw) / 2;
        y = radar_y + radar_w / 2 + tbh;
        display->setCursor(x, y);
        display->print(msg2);
      }
    }
  }
}

static void EPD_Draw_Radar()
{
  int16_t  tbx, tby;
  uint16_t tbw, tbh;
  int16_t x;
  int16_t y;
  char cog_text[6];
  int16_t scale;
  
  /* divider is distance range */
  //int32_t divider = 2000; // default 2000m 
  float divider = (1.0 / 2000.0); // default 2000m 

  display->setFont(&FreeMono9pt7b);
  display->getTextBounds("N", 0, 0, &tbx, &tby, &tbw, &tbh);

  uint16_t radar_x = 0;
  uint16_t radar_y = (display->height() - display->width()) / 2;
  uint16_t radar_w = display->width();

  display->fillRect(radar_x, radar_y, radar_w, radar_w, GxEPD_WHITE);

  int16_t radar_center_x = radar_w / 2;
  int16_t radar_center_y = radar_y + radar_w / 2;
  int16_t radius = radar_w / 2 - 2;

  float epd_Points[MAX_DRAW_POINTS][2];

  if (settings->units == UNITS_METRIC || settings->units == UNITS_MIXED) {
    switch(EPD_zoom)
    {
    case ZOOM_LOWEST:
      divider = (1.0 / 10000.0); /* 20 KM */
      break;
    case ZOOM_LOW:
      divider = (1.0 /  5000.0); /* 10 KM */
      break;
    case ZOOM_HIGH:
      divider = (1.0 /  1000);   /*  2 KM */
      break;
    case ZOOM_MEDIUM:
    default:
      divider = (1.0 /  2000.0); /*  4 KM */
      break;
    }
  } else {
    switch(EPD_zoom)
    {
    case ZOOM_LOWEST:
      divider = (1.0 / 9260.0);  /* 10 NM */
      break;
    case ZOOM_LOW:
      divider = (1.0 / 4630.0);  /*  5 NM */
      break;
    case ZOOM_HIGH:
      divider = (1.0 /  926.0);  /*  1 NM */
      break;
    case ZOOM_MEDIUM:
    default:
      divider = (1.0 / 1852.0);  /*  2 NM */
      break;
    }
  }

  {
//  float trSin = sin_approx(-ThisAircraft.Track);   // an error from 2023
//  float trCos = cos_approx(-ThisAircraft.Track);
    float trSin = sin_approx(ThisAircraft.Track);
    float trCos = cos_approx(ThisAircraft.Track);

    for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {
      if (Container[i].ID && (now() - Container[i].timestamp) <= EPD_EXPIRATION_TIME) {

        float rel_x;
        float rel_y;
        float tgtSin;
        float tgtCos;

        bool isTeam = (Container[i].ID == settings->team) ;

        rel_x = Container[i].RelativeEast;
        rel_y = Container[i].RelativeNorth;
        tgtSin = sin_approx(Container[i].Track);
        tgtCos = cos_approx(Container[i].Track);

        for (int i=0; i < ICON_TARGETS_POINTS; i++) {
          epd_Points[i][0] = epd_Target[i][0];
          epd_Points[i][1] = epd_Target[i][1];
          EPD_2D_Rotate(epd_Points[i][0], epd_Points[i][1], tgtCos, tgtSin);
        }
#if 0
        Serial.print(F(" ID="));
        Serial.print((Container[i].ID >> 16) & 0xFF, HEX);
        Serial.print((Container[i].ID >>  8) & 0xFF, HEX);
        Serial.print((Container[i].ID      ) & 0xFF, HEX);
        Serial.println();

        Serial.print(F(" RelativeNorth=")); Serial.println(Container[i].RelativeNorth);
        Serial.print(F(" RelativeEast="));  Serial.println(Container[i].RelativeEast);
#endif
        switch (settings->orientation) {
          case DIRECTION_NORTH_UP:
            break;
          case DIRECTION_TRACK_UP:
            // rotate relative to ThisAircraft.Track
            EPD_2D_Rotate(rel_x, rel_y, trCos, trSin);
            for (int i=0; i < ICON_TARGETS_POINTS; i++)
               EPD_2D_Rotate(epd_Points[i][0], epd_Points[i][1], trCos, -trSin);
            break;
          default:
            /* TBD */
            break;
        }

#if 0
      Serial.print(F("Debug "));
      Serial.print(trSin);
      Serial.print(F(", "));
      Serial.print(trCos);
      Serial.print(F(", "));
      Serial.print(rel_x);
      Serial.print(F(", "));
      Serial.print(rel_y);
      Serial.println();
      Serial.flush();
#endif
#if 0
      Serial.print(F("Debug "));
      Serial.print(tgtSin);
      Serial.print(F(", "));
      Serial.print(tgtCos);
      Serial.print(F(", "));
      Serial.print(epd_Points[1][0]);
      Serial.print(F(", "));
      Serial.print(epd_Points[1][1]);
      Serial.println();
      Serial.flush();
#endif

        float radius_divider = divider * (float)radius;
        x = constrain((int)(rel_x * radius_divider), -32768, 32767);
        y = constrain((int)(rel_y * radius_divider), -32768, 32767);

        scale = Container[i].alarm_level + 1;

        switch(scale)
        {
        case 1:
          // alarm level 0 - draw target as a simple triangle
          display->fillTriangle(radar_center_x + x + (int16_t) epd_Points[0][0],
                                radar_center_y - y + (int16_t) epd_Points[0][1],
                                radar_center_x + x + (int16_t) epd_Points[1][0],
                                radar_center_y - y + (int16_t) epd_Points[1][1],
                                radar_center_x + x + (int16_t) epd_Points[2][0],
                                radar_center_y - y + (int16_t) epd_Points[2][1],
                                GxEPD_BLACK);
          break;
        case 2:
        case 3:
        case 4:
          // draw an arrowhead (triangle with a rear notch)
          display->fillTriangle(radar_center_x + x + scale * (int16_t) epd_Points[0][0],
                                radar_center_y - y + scale * (int16_t) epd_Points[0][1],
                                radar_center_x + x + scale * (int16_t) epd_Points[1][0],
                                radar_center_y - y + scale * (int16_t) epd_Points[1][1],
                                radar_center_x + x + scale * (int16_t) epd_Points[4][0],
                                radar_center_y - y + scale * (int16_t) epd_Points[4][1],
                                GxEPD_BLACK);
          display->fillTriangle(radar_center_x + x + scale * (int16_t) epd_Points[2][0],
                                radar_center_y - y + scale * (int16_t) epd_Points[2][1],
                                radar_center_x + x + scale * (int16_t) epd_Points[1][0],
                                radar_center_y - y + scale * (int16_t) epd_Points[1][1],
                                radar_center_x + x + scale * (int16_t) epd_Points[4][0],
                                radar_center_y - y + scale * (int16_t) epd_Points[4][1],
                                GxEPD_BLACK);
          break;
        default:   // should never happen
          break;
        }

        if (Container[i].RelativeVertical >   EPD_RADAR_V_THRESHOLD) {
          // draw a '+' next to target triangle
          display->drawLine(radar_center_x + x - 2 + (int16_t) epd_Points[3][0],
                            radar_center_y - y     + (int16_t) epd_Points[3][1],
                            radar_center_x + x + 2 + (int16_t) epd_Points[3][0],
                            radar_center_y - y     + (int16_t) epd_Points[3][1],
                                GxEPD_BLACK);
          display->drawLine(radar_center_x + x     + (int16_t) epd_Points[3][0],
                            radar_center_y - y + 2 + (int16_t) epd_Points[3][1],
                            radar_center_x + x     + (int16_t) epd_Points[3][0],
                            radar_center_y - y - 2 + (int16_t) epd_Points[3][1],
                                GxEPD_BLACK);
        } else if (Container[i].RelativeVertical < - EPD_RADAR_V_THRESHOLD) {
          // draw a '-' next to target triangle
          display->drawLine(radar_center_x + x - 2 + (int16_t) epd_Points[3][0],
                            radar_center_y - y     + (int16_t) epd_Points[3][1],
                            radar_center_x + x + 2 + (int16_t) epd_Points[3][0],
                            radar_center_y - y     + (int16_t) epd_Points[3][1],
                                GxEPD_BLACK);
        } else {
            // TBD
        }
        // if Team match, draw a circle around target
        if (isTeam) {
          display->drawCircle(radar_center_x + x,
                              radar_center_y - y,
                              7, GxEPD_BLACK);
        }
      }
    }

    // draw range circles
    display->drawCircle(radar_center_x, radar_center_y, radius,   GxEPD_BLACK);
    display->drawCircle(radar_center_x, radar_center_y, radius/2, GxEPD_BLACK);

#if defined(ICON_AIRPLANE)
    /* draw little airplane */
    for (int i=0; i < ICON_AIRPLANE_POINTS; i++) {
    epd_Points[i][0] = epd_Airplane[i][0];
    epd_Points[i][1] = epd_Airplane[i][1];
    }
    switch (settings->orientation)
    {
    case DIRECTION_NORTH_UP:
      // rotate relative to ThisAircraft.Track
      for (int i=0; i < ICON_AIRPLANE_POINTS; i++) {
        EPD_2D_Rotate(epd_Points[i][0], epd_Points[i][1], trCos, trSin);
      }
      break;
    case DIRECTION_TRACK_UP:
      break;
    default:
      /* TBD */
      break;
    }
    display->drawLine(radar_center_x + (int16_t) epd_Points[0][0],
                      radar_center_y + (int16_t) epd_Points[0][1],
                      radar_center_x + (int16_t) epd_Points[1][0],
                      radar_center_y + (int16_t) epd_Points[1][1],
                      GxEPD_BLACK);
    display->drawLine(radar_center_x + (int16_t) epd_Points[2][0],
                      radar_center_y + (int16_t) epd_Points[2][1],
                      radar_center_x + (int16_t) epd_Points[3][0],
                      radar_center_y + (int16_t) epd_Points[3][1],
                      GxEPD_BLACK);
    display->drawLine(radar_center_x + (int16_t) epd_Points[4][0],
                      radar_center_y + (int16_t) epd_Points[4][1],
                      radar_center_x + (int16_t) epd_Points[5][0],
                      radar_center_y + (int16_t) epd_Points[5][1],
                      GxEPD_BLACK);
    display->drawLine(radar_center_x + (int16_t) epd_Points[6][0],
                      radar_center_y + (int16_t) epd_Points[6][1],
                      radar_center_x + (int16_t) epd_Points[7][0],
                      radar_center_y + (int16_t) epd_Points[7][1],
                      GxEPD_BLACK);
    display->drawLine(radar_center_x + (int16_t) epd_Points[8][0],
                      radar_center_y + (int16_t) epd_Points[8][1],
                      radar_center_x + (int16_t) epd_Points[9][0],
                      radar_center_y + (int16_t) epd_Points[9][1],
                      GxEPD_BLACK);
    display->drawLine(radar_center_x + (int16_t) epd_Points[10][0],
                      radar_center_y + (int16_t) epd_Points[10][1],
                      radar_center_x + (int16_t) epd_Points[11][0],
                      radar_center_y + (int16_t) epd_Points[11][1],
                      GxEPD_BLACK);
#else  //ICON_AIRPLANE
    /* draw arrow tip */
    for (int i=0; i < ICON_ARROW_POINTS; i++) {
      epd_Points[i][0] = epd_Arrow[i][0];
      epd_Points[i][1] = epd_Arrow[i][1];
    }
    switch (settings->orientation) 
    {
    case DIRECTION_NORTH_UP:
    // rotate relative to ThisAircraft.Track
    for (int i=0; i < ICON_ARROW_POINTS; i++) {
      EPD_2D_Rotate(epd_Points[i][0], epd_Points[i][1], trCos, trSin);
    }
      break;
    case DIRECTION_TRACK_UP:
      break;
    default:
      /* TBD */
      break;
    }
    display->drawLine(radar_center_x + (int16_t) epd_Points[0][0],
                      radar_center_y + (int16_t) epd_Points[0][1],
                      radar_center_x + (int16_t) epd_Points[1][0],
                      radar_center_y + (int16_t) epd_Points[1][1],
                      GxEPD_BLACK);
    display->drawLine(radar_center_x + (int16_t) epd_Points[1][0],
                      radar_center_y + (int16_t) epd_Points[1][1],
                      radar_center_x + (int16_t) epd_Points[2][0],
                      radar_center_y + (int16_t) epd_Points[2][1],
                      GxEPD_BLACK);
    display->drawLine(radar_center_x + (int16_t) epd_Points[2][0],
                      radar_center_y + (int16_t) epd_Points[2][1],
                      radar_center_x + (int16_t) epd_Points[3][0],
                      radar_center_y + (int16_t) epd_Points[3][1],
                      GxEPD_BLACK);
    display->drawLine(radar_center_x + (int16_t) epd_Points[3][0],
                      radar_center_y + (int16_t) epd_Points[3][1],
                      radar_center_x + (int16_t) epd_Points[0][0],
                      radar_center_y + (int16_t) epd_Points[0][1],
                      GxEPD_BLACK);
#endif //ICON_AIRPLANE

    switch (settings->orientation)
    {
    case DIRECTION_NORTH_UP:
      // draw W, E, N, S
      x = radar_x + radar_w / 2 - radius + tbw/2;
      y = radar_y + (radar_w + tbh) / 2;
      display->setCursor(x , y);
      display->print("W");
      x = radar_x + radar_w / 2 + radius - (3 * tbw)/2;
      y = radar_y + (radar_w + tbh) / 2;
      display->setCursor(x , y);
      display->print("E");
      x = radar_x + (radar_w - tbw) / 2;
      y = radar_y + radar_w/2 - radius + (3 * tbh)/2;
      display->setCursor(x , y);
      display->print("N");
      x = radar_x + (radar_w - tbw) / 2;
      y = radar_y + radar_w/2 + radius - tbh/2;
      display->setCursor(x , y);
      display->print("S");
      break;
    case DIRECTION_TRACK_UP:
      // draw L, R, B
      x = radar_x + radar_w / 2 - radius + tbw/2;
      y = radar_y + (radar_w + tbh) / 2;
      display->setCursor(x , y);
      display->print("L");
      x = radar_x + radar_w / 2 + radius - (3 * tbw)/2;
      y = radar_y + (radar_w + tbh) / 2;
      display->setCursor(x , y);
      display->print("R");
      x = radar_x + (radar_w - tbw) / 2;
      y = radar_y + radar_w/2 + radius - tbh/2;
      display->setCursor(x , y);
      display->print("B");

      // draw aircraft heading
      display->setFont(&FreeMonoBold9pt7b);
      snprintf(cog_text, sizeof(cog_text), "%03d", ThisAircraft.Track);
      display->getTextBounds(cog_text, 0, 0, &tbx, &tby, &tbw, &tbh);
      x = radar_x + (radar_w - tbw) / 2;
      y = radar_y + radar_w/2 - radius + (3 * tbh)/2;
      display->setCursor(x , y);
      display->print(cog_text);
      display->drawRoundRect(x-2, y-tbh-2, tbw+8, tbh+6, 4, GxEPD_BLACK);
      break;
    default:
      /* TBD */
      break;
    }
  }
}

void EPD_radar_setup()
{
  EPD_zoom = settings->zoom;

  uint16_t radar_x = 0;
  uint16_t radar_y = 0;
  uint16_t radar_w = 0;

  if (display) {
    radar_y = (display->height() - display->width()) / 2;
    radar_w = display->width();
  }

  memcpy(navbox1.title, NAVBOX1_TITLE, strlen(NAVBOX1_TITLE));
  navbox1.x = 0;
  navbox1.y = 0;

  if (display) {
    navbox1.width  = display->width() / 2;
    navbox1.height = (display->height() - display->width()) / 2;
  }

  navbox1.value      = 0;
  navbox1.timestamp  = millis();

  memcpy(navbox2.title, NAVBOX2_TITLE, strlen(NAVBOX2_TITLE));
  navbox2.x = navbox1.width;
  navbox2.y = navbox1.y;
  navbox2.width  = navbox1.width;
  navbox2.height = navbox1.height;
  navbox2.value      = PROTOCOL_NONE;
  navbox2.timestamp  = millis();

  memcpy(navbox3.title, NAVBOX3_TITLE, strlen(NAVBOX3_TITLE));
  navbox3.x = 0;
  navbox3.y = radar_y + radar_w;
  navbox3.width  = navbox1.width;
  navbox3.height = navbox1.height;
  navbox3.value      = EPD_zoom;
  navbox3.timestamp  = millis();

  memcpy(navbox4.title, NAVBOX4_TITLE, strlen(NAVBOX4_TITLE));
  navbox4.x = navbox3.width;
  navbox4.y = navbox3.y;
  navbox4.width  = navbox3.width;
  navbox4.height = navbox3.height;
  navbox4.value      = (int) (Battery_voltage() * 10.0);
  navbox4.timestamp  = millis();
}

void EPD_radar_loop()
{
  if (isTimeToDisplay() && SoC->EPD_is_ready()) {

    bool hasData = settings->protocol == PROTOCOL_NMEA  ? NMEA_isConnected()  :
                   settings->protocol == PROTOCOL_GDL90 ? GDL90_isConnected() :
                   false;

    if (hasData) {

      bool hasFix = settings->protocol == PROTOCOL_NMEA  ? isValidGNSSFix()   :
                    settings->protocol == PROTOCOL_GDL90 ? GDL90_hasOwnShip() :
                    false;

      if (hasFix) {
        EPD_Draw_Radar();
      } else {
        EPD_radar_Draw_Message(NO_FIX_TEXT, NULL);
      }
    } else {
      EPD_radar_Draw_Message(NO_DATA_TEXT, NULL);
    }

    navbox1.value = Traffic_Count();

    switch (settings->protocol)
    {
    case PROTOCOL_GDL90:
      navbox2.value = GDL90_hasHeartBeat() ?
                      PROTOCOL_GDL90 : PROTOCOL_NONE;
      break;
    case PROTOCOL_NMEA:
    default:
      navbox2.value = (NMEA_hasFLARM() || NMEA_hasGNSS()) ?
                      PROTOCOL_NMEA  : PROTOCOL_NONE;
      break;
    }

    navbox3.value = EPD_zoom;
    navbox4.value = (int) (Battery_voltage() * 10.0);

    EPD_Draw_NavBoxes();

    SoC->EPD_update(EPD_UPDATE_FAST);

    EPDTimeMarker = millis();
  }
}

void EPD_radar_zoom()
{
  if (EPD_zoom < ZOOM_HIGH) EPD_zoom++;
}

void EPD_radar_unzoom()
{
  if (EPD_zoom > ZOOM_LOWEST) EPD_zoom--;
}
