/*
 * NMEAHelper.cpp
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

#include <TinyGPS++.h>
#include <TimeLib.h>

#include "SoCHelper.h"
#include "NMEAHelper.h"
#include "TrafficHelper.h"
#include "EEPROMHelper.h"
#include "WiFiHelper.h"

#include "SkyView.h"

TinyGPSPlus nmea;

TinyGPSCustom T_AlarmLevel      (nmea, "PFLAA", 1);
TinyGPSCustom T_RelativeNorth   (nmea, "PFLAA", 2);
TinyGPSCustom T_RelativeEast    (nmea, "PFLAA", 3);
TinyGPSCustom T_RelativeVertical(nmea, "PFLAA", 4);
TinyGPSCustom T_IDType          (nmea, "PFLAA", 5);
TinyGPSCustom T_ID              (nmea, "PFLAA", 6);
TinyGPSCustom T_Track           (nmea, "PFLAA", 7);
TinyGPSCustom T_TurnRate        (nmea, "PFLAA", 8);
TinyGPSCustom T_GroundSpeed     (nmea, "PFLAA", 9);
TinyGPSCustom T_ClimbRate       (nmea, "PFLAA", 10);
TinyGPSCustom T_AcftType        (nmea, "PFLAA", 11);

TinyGPSCustom S_RX              (nmea, "PFLAU", 1);
TinyGPSCustom S_TX              (nmea, "PFLAU", 2);
TinyGPSCustom S_GPS             (nmea, "PFLAU", 3);
TinyGPSCustom S_Power           (nmea, "PFLAU", 4);
TinyGPSCustom S_AlarmLevel      (nmea, "PFLAU", 5);
TinyGPSCustom S_RelativeBearing (nmea, "PFLAU", 6);
TinyGPSCustom S_AlarmType       (nmea, "PFLAU", 7);
TinyGPSCustom S_RelativeVertical(nmea, "PFLAU", 8);
TinyGPSCustom S_RelativeDistance(nmea, "PFLAU", 9);
TinyGPSCustom S_ID              (nmea, "PFLAU", 10);

status_t NMEA_Status;

static unsigned long NMEA_TimeMarker = 0;

static void NMEA_Parse_Character(char c)
{
    static uint32_t old_id;
    static int old_level;

    bool isValidSentence = nmea.encode(c);
    if (! isValidSentence)
        return;

      if (nmea.location.isUpdated()) {
        ThisAircraft.latitude  = nmea.location.lat();
        ThisAircraft.longitude = nmea.location.lng();
      }
      if (nmea.altitude.isUpdated()) {
        ThisAircraft.altitude = nmea.altitude.meters();
      }
      if (nmea.course.isUpdated()) {
        int raw = nmea.course.deg();
        // make sure it will fit in the uint16 Track variable:
        if (raw < 0)          raw += 360;
        else if (raw >= 360)  raw -= 360;
        if (raw < 0 || raw >= 360)  raw = 0;
        ThisAircraft.Track = raw;
      }
      if (nmea.speed.isUpdated()) {
        ThisAircraft.GroundSpeed = nmea.speed.knots();
      }

      if (T_ID.isUpdated()) {
        fo = EmptyFO;

//        Serial.print(F(" ID=")); Serial.print(ID.value());

        fo.ID = strtol(T_ID.value(), NULL, 16);

#if 0
        Serial.print(F(" ID="));
        Serial.print((fo.ID >> 16) & 0xFF, HEX);
        Serial.print((fo.ID >>  8) & 0xFF, HEX);
        Serial.print((fo.ID      ) & 0xFF, HEX);
        Serial.println();
#endif

        if (T_AlarmLevel.isUpdated())
        {
//          Serial.print(F(" AlarmLevel=")); Serial.print(T_AlarmLevel.value());
          fo.alarm_level = atoi(T_AlarmLevel.value());
//          Serial.print(F(" AlarmLevel=")); Serial.println(fo.alarm_level);
        }
        if (T_RelativeNorth.isUpdated())
        {
//          Serial.print(F(" RelativeNorth=")); Serial.print(T_RelativeNorth.value());
          fo.RelativeNorth = atoi(T_RelativeNorth.value());
//          Serial.print(F(" RelativeNorth=")); Serial.println(fo.RelativeNorth);
        }
        if (T_RelativeEast.isUpdated())
        {
//          Serial.print(F(" RelativeEast=")); Serial.print(T_RelativeEast.value());
          fo.RelativeEast = atoi(T_RelativeEast.value());
//          Serial.print(F(" RelativeEast=")); Serial.println(fo.RelativeEast);
        }
        if (T_RelativeVertical.isUpdated())
        {
//          Serial.print(F(" RelativeVertical=")); Serial.print(T_RelativeVertical.value());
          fo.RelativeVertical = atoi(T_RelativeVertical.value());
//          Serial.print(F(" RelativeVertical=")); Serial.println(fo.RelativeVertical);
        }
        if (T_IDType.isUpdated())
        {
//          Serial.print(F(" IDType=")); Serial.print(T_IDType.value());
          fo.IDType = atoi(T_IDType.value());
//          Serial.print(F(" IDType=")); Serial.println(fo.IDType);
        }
        if (T_Track.isUpdated())
        {
//          Serial.print(F(" Track=")); Serial.print(T_Track.value());
          int raw = atoi(T_Track.value());
          // make sure it will fit in the uint16 Track variable:
          if (raw < 0)          raw += 360;
          else if (raw >= 360)  raw -= 360;
          if (raw < 0 || raw >= 360)  raw = 0;
          fo.Track = raw;
//          Serial.print(F(" Track=")); Serial.println(fo.Track);
        }
        if (T_TurnRate.isUpdated())
        {
//          Serial.print(F(" TurnRate=")); Serial.print(T_TurnRate.value());
          fo.TurnRate = atoi(T_TurnRate.value());
//          Serial.print(F(" TurnRate=")); Serial.println(fo.TurnRate);
        }
        if (T_GroundSpeed.isUpdated())
        {
//          Serial.print(F(" GroundSpeed=")); Serial.print(T_GroundSpeed.value());
          fo.GroundSpeed = atoi(T_GroundSpeed.value());
//          Serial.print(F(" GroundSpeed=")); Serial.println(fo.GroundSpeed);
        }
        if (T_ClimbRate.isUpdated())
        {
//          Serial.print(F(" ClimbRate=")); Serial.println(T_ClimbRate.value());
          /* TBD */
        }
        if (T_AcftType.isUpdated())
        {
//          Serial.print(F(" AcftType=")); Serial.print(T_AcftType.value());
          fo.AcftType = atoi(T_AcftType.value());
//          Serial.print(F(" AcftType=")); Serial.println(fo.AcftType);
        }

        old_level = ALARM_LEVEL_NONE;
        if (fo.alarm_level > ALARM_LEVEL_NONE) {
            old_id = fo.ID;    // save for processing of PFLAU
            old_level = fo.alarm_level;
        }

        fo.timestamp = ThisAircraft.timestamp = now();
        fo.packet_type = 1;
        Traffic_Update(&fo);
        Traffic_Add();

      } else if (S_RX.isUpdated()) {

        fo = EmptyFO;    // treat PFLAU too as traffic

        NMEA_Status.RX = atoi(S_RX.value());

        if (S_TX.isUpdated())
        {
//          Serial.print(F(" TX=")); Serial.print(S_TX.value());
          NMEA_Status.TX = atoi(S_TX.value());
//          Serial.print(F(" TX=")); Serial.println(NMEA_Status.TX);
        }
        if (S_GPS.isUpdated())
        {
//          Serial.print(F(" GPS=")); Serial.print(S_GPS.value());
          NMEA_Status.GPS = atoi(S_GPS.value());
//          Serial.print(F(" GPS=")); Serial.println(NMEA_Status.GPS);
        }
        if (S_Power.isUpdated())
        {
//          Serial.print(F(" Power=")); Serial.print(S_Power.value());
          NMEA_Status.Power = atoi(S_Power.value());
//          Serial.print(F(" Power=")); Serial.println(NMEA_Status.Power);
        }
        if (S_AlarmLevel.isUpdated())
        {
//          Serial.print(F(" AlarmLevel=")); Serial.print(S_AlarmLevel.value());
          NMEA_Status.alarm_level = atoi(S_AlarmLevel.value());
          fo.alarm_level = NMEA_Status.alarm_level;
//          Serial.print(F(" AlarmLevel=")); Serial.println(NMEA_Status.alarm_level);
        }
        if (S_RelativeBearing.isUpdated())
        {
//          Serial.print(F(" RelativeBearing=")); Serial.print(S_RelativeBearing.value());
          NMEA_Status.RelativeBearing = atoi(S_RelativeBearing.value());
          fo.RelativeBearing = NMEA_Status.RelativeBearing;   // -180..180
//          Serial.print(F(" RelativeBearing=")); Serial.println(NMEA_Status.RelativeBearing);
        }
        if (S_AlarmType.isUpdated())
        {
//          Serial.print(F(" AlarmType=")); Serial.print(S_AlarmType.value());
          NMEA_Status.AlarmType = atoi(S_AlarmType.value());
          if (NMEA_Status.AlarmType == 4)          // traffic "advisory"
              fo.alarm_level = ALARM_LEVEL_NONE;   // does not ignore obstacle warnings
//          Serial.print(F(" AlarmType=")); Serial.println(NMEA_Status.AlarmType);
        }
        if (S_RelativeVertical.isUpdated())
        {
//          Serial.print(F(" RelativeVertical=")); Serial.print(S_RelativeVertical.value());
          NMEA_Status.RelativeVertical = atoi(S_RelativeVertical.value());
          fo.RelativeVertical = NMEA_Status.RelativeVertical;     // meters
//          Serial.print(F(" RelativeVertical=")); Serial.println(NMEA_Status.RelativeVertical);
        }
        if (S_RelativeDistance.isUpdated())
        {
//          Serial.print(F(" RelativeDistance=")); Serial.print(S_RelativeDistance.value());
          NMEA_Status.RelativeDistance = strtol(S_RelativeDistance.value(), NULL, 10);
          fo.distance = NMEA_Status.RelativeDistance;     // meters
//          Serial.print(F(" RelativeDistance=")); Serial.println(NMEA_Status.RelativeDistance);
        }
        if (S_ID.isUpdated())
        {
//          Serial.print(F(" ID=")); Serial.print(S_ID.value());
          NMEA_Status.ID = strtol(S_ID.value(), NULL, 16);
          fo.ID = NMEA_Status.ID;
#if 0
          Serial.print(F(" ID="));
          Serial.print((NMEA_Status.ID >> 16) & 0xFF, HEX);
          Serial.print((NMEA_Status.ID >>  8) & 0xFF, HEX);
          Serial.print((NMEA_Status.ID      ) & 0xFF, HEX);
          Serial.println();
#endif
        } else {    // very old FLARMs don't send the ID
          if (old_level > ALARM_LEVEL_NONE && fo.alarm_level == old_level)
              fo.ID = old_id;   // inherited from the preceding FPLAA
          else if (fo.alarm_level > ALARM_LEVEL_NONE)
              fo.ID = 0x123456;
        }

        /* PFLAU sentence may not include any traffic, update timestamp anyway */
        fo.timestamp = ThisAircraft.timestamp = NMEA_Status.timestamp = now();
        fo.packet_type = 2;

        if (fo.ID) {
            Traffic_Update(&fo);
            Traffic_Add();
        }
      }
}

void NMEA_setup()
{
  if (settings->protocol == PROTOCOL_NMEA) {
    switch (settings->connection)
    {
    case CON_SERIAL:
      uint32_t SerialBaud;

      switch (settings->baudrate)
      {
      case B4800:
        SerialBaud = 4800;
        break;
      case B9600:
        SerialBaud = 9600;
        break;
      case B19200:
        SerialBaud = 19200;
        break;
      case B57600:
        SerialBaud = 57600;
        break;
      case B115200:
        SerialBaud = 115200;
        break;
      case B2000000:
        SerialBaud = 2000000;
        break;
      case B38400:
      default:
        SerialBaud = 38400;
        break;
      }

      SoC->swSer_begin(SerialBaud);
      break;
    case CON_BLUETOOTH_SPP:
    case CON_BLUETOOTH_LE:
      if (SoC->Bluetooth) {
        SoC->Bluetooth->setup();
      }
      break;
    case CON_NONE:
    case CON_WIFI_UDP:
    default:
      break;
    }

    switch (settings->bridge)
    {
    case BRIDGE_BT_SPP:
    case BRIDGE_BT_LE:
      if (SoC->Bluetooth) {
        SoC->Bluetooth->setup();
      }
      break;
    default:
      break;
    }

    NMEA_TimeMarker = millis();
  }
}

int NMEA_add_checksum(char *buf)
{
  size_t sentence_size = strlen(buf);

  //calculate the checksum
  unsigned char cs = 0;
  for (unsigned int n = 1; n < sentence_size - 1; n++) {
    cs ^= buf[n];
  }

  char *csum_ptr = buf + sentence_size;
  snprintf_P(csum_ptr, 5, PSTR("%02X\r\n"), cs);  // assumes there is room!

  return (sentence_size + 4);
}

void NMEA_bridge_send(char *buf, int len)
{
    if (settings->bridge == BRIDGE_UDP) {
        WiFi_Transmit_UDP(buf, (size_t) len);
    } else if (settings->bridge == BRIDGE_TCP) {
        // tbd
    } else if (settings->bridge == BRIDGE_BT_SPP
            || settings->bridge == BRIDGE_BT_LE) {
        if (SoC->Bluetooth) {
//Serial.print("calling BT_write(): ");
//Serial.print(buf);
            SoC->Bluetooth->write((uint8_t *) buf, (size_t) len);
        }
    } else if (settings->bridge == BRIDGE_SERIAL) {
        // buf[len] = '\0';
        Serial.print(buf);
    }
}

void NMEA_Out(char *buf)
{
    int len = NMEA_add_checksum(buf);
    if (settings->bridge != BRIDGE_SERIAL)
        Serial.print(buf);         // otherwise sent via bridge_send()
    NMEA_bridge_send(buf, len);    // even if no input received
}

void NMEA_bridge_buffer(char c)
{
    static char buf[128+3];
    static int n = 0;

    if (settings->bridge == BRIDGE_NONE)
        return;

    if (! NMEA_isConnected())  // still true before processing this newly input char
        n = 0;

    if (c=='\r' || c=='\n')
        return;

    if (n == 0 && c != '$')
        return;

    buf[n++] = c;

    if (n < 5)
        return;

    if (buf[n-3] == '*') {
        buf[n++] = '\r';
        buf[n++] = '\n';
        buf[n]   = '\0';
        NMEA_bridge_send(buf, n);
        n = 0;
    }

    if (n >= 128)
        n = 0;
}

void NMEA_loop()
{
  size_t size;

#if !defined(EXCLUDE_HEARTBEAT)
  char buf[40];
  static uint32_t heartbeat = 0;
  if (heartbeat == 0)
      heartbeat = millis();
  if (millis() > heartbeat + 4900) {
      snprintf_P(buf, sizeof(buf)-4,     // leave room for checksum
              PSTR("$PSKVH,%06X,v%s,%d,%d,%d*"),
              SoC->getChipId() & 0xFFFFFF, SKYVIEW_FIRMWARE_VERSION,
              settings->connection, settings->bridge, settings->filter);
      NMEA_Out(buf);
      heartbeat = millis();
  }
#endif

  switch (settings->connection)
  {
  case CON_SERIAL:
    while (SerialInput.available() > 0) {
      char c = SerialInput.read();
      Serial.print(c);
      NMEA_bridge_buffer(c);
      NMEA_Parse_Character(c);
      NMEA_TimeMarker = millis();
    }
    /* read data from microUSB port */
#if !defined(RASPBERRY_PI)
    if ((void *) &Serial != (void *) &SerialInput)
#endif
    {
      while (Serial.available() > 0) {
        char c = Serial.read();
        NMEA_bridge_buffer(c);
        NMEA_Parse_Character(c);
        NMEA_TimeMarker = millis();
      }
    }
    break;
  case CON_WIFI_UDP:
    size = SoC->WiFi_Receive_UDP((uint8_t *) UDPpacketBuffer, sizeof(UDPpacketBuffer));
    if (size > 0) {
      for (size_t i=0; i < size; i++) {
        char c = UDPpacketBuffer[i];
        if (settings->bridge == BRIDGE_SERIAL)
            NMEA_bridge_buffer(c);   // only output complete sentences
        else
            Serial.print(c);         // as received, unfiltered
        NMEA_Parse_Character(c);
      }
      NMEA_TimeMarker = millis();
    }
    break;
  case CON_BLUETOOTH_SPP:
  case CON_BLUETOOTH_LE:
    if (SoC->Bluetooth) {
      while (SoC->Bluetooth->available() > 0) {
        char c = SoC->Bluetooth->read();
        if (settings->bridge == BRIDGE_SERIAL)
            NMEA_bridge_buffer(c);
        else
            Serial.print(c);
        NMEA_Parse_Character(c);
        NMEA_TimeMarker = millis();
      }
    }
    break;
  case CON_NONE:
  default:
    break;
  }
}

bool NMEA_isConnected()
{
  return (NMEA_TimeMarker > DATA_TIMEOUT &&
         (millis() - NMEA_TimeMarker) < DATA_TIMEOUT);
}

bool NMEA_hasGNSS()
{
  return (nmea.time.isValid() && nmea.time.age() < NMEA_EXP_TIME);
}

bool NMEA_hasFLARM()
{
  return (S_RX.isValid() && S_RX.age() < NMEA_EXP_TIME);
}
