/*
 * NMEAHelper.cpp
 * Copyright (C) 2017-2022 Linar Yusupov
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

// This file originally based on v1.2.
// Major revision in Jan. 2024: buffer each input separately, for bridging.

#include <TimeLib.h>

#include "NMEA.h"
#include "GDL90.h"
#include "GNS5892.h"
#include "../../driver/GNSS.h"
#include "../../driver/RF.h"
#include "../../system/SoC.h"
// which does #include "../../SoftRF.h"
#include "../../system/Time.h"
#include "../../driver/WiFi.h"
#include "../../driver/Settings.h"
#include "../../driver/RF.h"
#include "../../driver/Battery.h"
#include "../../driver/Baro.h"
#if defined(USE_SD_CARD)
#include <SD.h>
#include <FS.h>
#endif
#if defined(ESP32)
#include "../../driver/OLED.h"
#include "../../driver/Strobe.h"
#endif
#include "../../driver/Bluetooth.h"
#include "../../driver/Filesys.h"
#include "IGC.h"
#include "../../TrafficHelper.h"

#if defined(USE_SD_CARD)
#include <SD.h>
#include <FS.h>
#endif

#define ADDR_TO_HEX_STR(s, c) (s += ((c) < 0x10 ? "0" : "") + String((c), HEX))

#if defined(ESP32)   // only on SD card
File NMEALog;
bool NMEALogOpen = false;
uint32_t next_SD_sync = 0;
#endif

uint8_t NMEA_Source = DEST_NONE;   // identifies which port a sentence came from

char NMEABuffer[NMEA_BUFFER_SIZE]; //buffer for NMEA data
char GPGGA_Copy[NMEA_BUFFER_SIZE];   //store last $GGA sentence

//static char NMEA_Callsign[NMEA_CALLSIGN_SIZE];

#if defined(NMEA_TCP_SERVICE)

WiFiServer NmeaTCPServer(NMEA_TCP_PORT);
NmeaTCP_t NmeaTCP[MAX_NMEATCP_CLIENTS];
bool TCP_active = false;

// TCP-client code copied from OGNbase
// see also https://github.com/espressif/arduino-esp32/tree/master/libraries/WiFi/examples

// #include <ESP32Ping.h>
#include <WiFiClient.h>

WiFiClient   client;

static int WiFi_connect_TCP()
{
    // if (Ping.ping(host, 2))
    {
        if (!client.connect(settings->host_ip,
                            settings->tcpport? ALT_TCP_PORT : NMEA_TCP_PORT,
                            5000)) {
            Serial.println(F("Failed to connect as TCP client"));
            return 0;
        }
//Serial.println("Connected as TCP client");
        return 1;
    }
//Serial.println(F("TCP host not responding to ping"));
    return 0;
}

static int WiFi_disconnect_TCP()
{
    client.stop();
    return 0;
}

int WiFi_transmit_TCP(const char *buf, size_t size)
{
  if (TCP_active) {
    if (settings->tcpmode == TCP_MODE_SERVER) {
      for (uint8_t acc_ndx = 0; acc_ndx < MAX_NMEATCP_CLIENTS; acc_ndx++) {

        if (NmeaTCP[acc_ndx].client && NmeaTCP[acc_ndx].client.connected()){
          if (NmeaTCP[acc_ndx].ack) {
            NmeaTCP[acc_ndx].client.write(buf, size);
          }
        }
      }
    }
    else if (settings->tcpmode == TCP_MODE_CLIENT) {
      if (client.connected())
      {
        client.write((byte *) buf, size);
      }
    }
  }
  return 0;
}

static int WiFi_receive_TCP(char* RXbuffer, int RXbuffer_size)
{
    int i = 0;

    if (client.connected())
    {
        while (client.available() && i < RXbuffer_size - 1) {
            RXbuffer[i] = client.read();
            i++;
        }
        RXbuffer[i] = '\0';
        return i;
    }
    client.stop();
    return -1;
}

static void WiFi_flush_TCP()
{
//static bool db;
//db = ((settings->nmea_d || settings->nmea2_d) && (settings->debug_flags & DEBUG_DEEPER));
    if (client.connected())
    {
//if (db && client.available())
//Serial.println(F("TCP_input_flushed"));
        while (client.available()) {
            char c = client.read();
            yield();
        }
        return;
    }
    client.stop();
}

static int WiFi_isconnected_TCP()
{
    return client.connected();
}

#endif // defined(NMEA_TCP_SERVICE)

#if defined(USE_NMEALIB)
#include <nmealib.h>

NmeaMallocedBuffer nmealib_buf;
#endif /* USE_NMEALIB */

const char *NMEA_CallSign_Prefix[] = {
  [RF_PROTOCOL_LEGACY]    = "FLO",
  [RF_PROTOCOL_OGNTP]     = "OGN",
  [RF_PROTOCOL_P3I]       = "PAW",
  [RF_PROTOCOL_ADSB_1090] = "ADS",
  [RF_PROTOCOL_ADSB_UAT]  = "UAT",
  [RF_PROTOCOL_FANET]     = "FAN",
  [RF_PROTOCOL_GDL90]     = "GDL",   // data from external device
  [RF_PROTOCOL_LATEST]    = "FLR",
  [RF_PROTOCOL_ADSL]      = "ADL"
};

const uint8_t data_source_code[] = {
    [TX_TYPE_NONE]  = 0,
    [TX_TYPE_A]     = 6,
    [TX_TYPE_C]     = 6,
    [TX_TYPE_S]     = 6,
    [TX_TYPE_TISB]  = 4,
    [TX_TYPE_ADSR]  = 3,
    [TX_TYPE_ADSB]  = 1,
    [TX_TYPE_FLARM] = 0
};

#define isTimeToPGRMZ() (millis() - PGRMZ_TimeMarker > 1000)
unsigned long PGRMZ_TimeMarker = 0;

extern uint32_t tx_packets_counter, rx_packets_counter;

#if defined(ENABLE_AHRS)
#include "../../driver/AHRSHelper.h"

#define isTimeToRPYL()  (millis() - RPYL_TimeMarker > AHRS_INTERVAL)
unsigned long RPYL_TimeMarker = 0;
#endif /* ENABLE_AHRS */

#if defined(USE_NMEA_CFG)

TinyGPSCustom C_Version;   /* 1 */
TinyGPSCustom C_Mode;
TinyGPSCustom C_Protocol;
TinyGPSCustom C_Band;
TinyGPSCustom C_AcftType;
TinyGPSCustom C_Alarm;
TinyGPSCustom C_TxPower;
TinyGPSCustom C_Volume;
TinyGPSCustom C_Pointer;
TinyGPSCustom C_NMEA_gnss; /* 10 */
TinyGPSCustom C_NMEA_private;
TinyGPSCustom C_NMEA_traffic;
TinyGPSCustom C_NMEA_sensors;
TinyGPSCustom C_NMEA_Output;
TinyGPSCustom C_GDL90_Output;
TinyGPSCustom C_D1090_Output;
TinyGPSCustom C_Stealth;
TinyGPSCustom C_noTrack;
TinyGPSCustom C_PowerSave; /* 19 */

// additional settings added by MB
TinyGPSCustom D_Version;   /* 1 */
TinyGPSCustom D_id_method;
TinyGPSCustom D_aircraft_id;
TinyGPSCustom D_ignore_id;
TinyGPSCustom D_follow_id;
TinyGPSCustom D_baud_rate;
TinyGPSCustom D_power_ext;
TinyGPSCustom D_NMEA_debug;
TinyGPSCustom D_debug_flags;
TinyGPSCustom D_NMEA2;    /* 10 */
TinyGPSCustom D_NMEA2_gnss;
TinyGPSCustom D_NMEA2_private;
TinyGPSCustom D_NMEA2_legacy;
TinyGPSCustom D_NMEA2_sensors;
TinyGPSCustom D_NMEA2_debug;
TinyGPSCustom D_relay;
TinyGPSCustom D_bluetooth;  /* 17 */
TinyGPSCustom D_baudrate2;
TinyGPSCustom D_invert2;
TinyGPSCustom D_extern1;
TinyGPSCustom D_extern2;  /* 21 */
TinyGPSCustom D_altpin0;
TinyGPSCustom D_voice;
TinyGPSCustom D_strobe;

TinyGPSCustom F_Version;   /* 1 */
TinyGPSCustom F_rx1090;
TinyGPSCustom F_rx1090x;
TinyGPSCustom F_mode_s;
TinyGPSCustom F_gdl90_in;
TinyGPSCustom F_gnss_pins;
TinyGPSCustom F_ppswire;
TinyGPSCustom F_logalarms;
TinyGPSCustom F_sd_card;
TinyGPSCustom F_logflight;
TinyGPSCustom F_loginterval;
TinyGPSCustom F_alt_udp;
TinyGPSCustom F_tcpmode;
TinyGPSCustom F_tcpport;
TinyGPSCustom F_geoid;
TinyGPSCustom F_freq_corr;  /* 15 */

// one setting at a time:
TinyGPSCustom S_Version;
TinyGPSCustom S_label;
TinyGPSCustom S_value;

TinyGPSCustom T_testmode;

#if defined(USE_OGN_ENCRYPTION)
/* Security and privacy */
TinyGPSCustom K_Version;
TinyGPSCustom K_IGC_Key;
#endif /* USE_OGN_ENCRYPTION */

#if defined(USE_SKYVIEW_CFG)
#include "../../driver/EPD.h"

TinyGPSCustom V_Version; /* 1 */
TinyGPSCustom V_Adapter;
TinyGPSCustom V_Connection;
TinyGPSCustom V_Units;
TinyGPSCustom V_Zoom;
TinyGPSCustom V_Protocol;
TinyGPSCustom V_Baudrate;
TinyGPSCustom V_Server;
TinyGPSCustom V_Key;
TinyGPSCustom V_Rotate;  /* 10 */
TinyGPSCustom V_Orientation;
TinyGPSCustom V_AvDB;
TinyGPSCustom V_ID_Pref;
TinyGPSCustom V_VMode;
TinyGPSCustom V_Voice;
TinyGPSCustom V_AntiGhost;
TinyGPSCustom V_Filter;
TinyGPSCustom V_PowerSave;
TinyGPSCustom V_Team;    /* 19 */
#endif /* USE_SKYVIEW_CFG */
#endif /* USE_NMEA_CFG */

static char *ltrim(char *s)
{
  if(s) {
    while(*s && isspace(*s))
      ++s;
  }

  return s;
}

// if '*' found, add checksum (and \r\n) after it.
// otherwise do not change the string but return its length.
unsigned int NMEA_add_checksum(char *buf)
{
  //calculate the checksum - get sentence length as a side effect
  unsigned int n;
  char c;
  unsigned char cs = 0;
  for (n = 1; n < NMEA_BUFFER_SIZE - 5; n++) {
    c = buf[n];
    if (c == '*')
        break;
    if (c == '\0')
        break;
    cs ^= c;
  }
//Serial.print("' - n:");
//Serial.println(n);
  if (c == '*') {
      char *csum_ptr = &buf[n+1];
      snprintf_P(csum_ptr, 5, PSTR("%02X\r\n"), cs);
//Serial.print("checksum added: ");
//Serial.print(buf);
      return (n + 5);
  }
  if (c == '\0')
      return n;
  return 0;
}

// send self-test and version sentences out, imitating a FLARM
void sendPFLAV(bool nowait)
{
  static uint32_t whensent = 0;
  if (!nowait) { 
      if (OurTime == whensent)
          return;                 // only continue once per second
      whensent = OurTime;
      if (OurTime < 1000000)
          return;
      if ((((uint32_t) OurTime) & 0x1F) != 7)   // send once in 32 seconds
          return;
  }
  uint32_t timebits = (((uint32_t) OurTime) & 0x60);
  if (nowait || timebits == 0x20) {
    uint32_t pps = SoC->get_PPS_TimeMarker();
    snprintf_P(NMEABuffer, sizeof(NMEABuffer), PSTR("$PFLAE,A,0,0%s*"),
        (pps > 0 && millis()-pps < 2000)? ",PPS received" : "");
    NMEAOutC(NMEA_T);
  }
  if (nowait || timebits == 0x60) {
    snprintf_P(NMEABuffer, sizeof(NMEABuffer), PSTR("$PFLAV,A,2.4,7.24,%s-%s*"),
                 SOFTRF_IDENT, SOFTRF_FIRMWARE_VERSION);  // our version in obstacle db text field
    NMEAOutC(NMEA_T);
  }
}

// copy into plain static variables for efficiency in NMEA_Loop():
static bool is_a_prime_mk2 = false;
static unsigned int UDP_NMEA_Output_Port = NMEA_UDP_PORT;
bool has_serial2 = false;
bool rx1090found = false;

void sendPFLAJ()
{
    snprintf_P(NMEABuffer, sizeof(NMEABuffer), PSTR("$PFLAJ,A,%d,%d,%d*"),
                 ThisAircraft.airborne, FlightLogOpen, rx1090found);
    NMEAOutC(NMEA_T_PROJ);
}

void NMEA_setup()
{
  if (settings->alt_udp)
    UDP_NMEA_Output_Port = NMEA_UDP_PORT2;  // or use ALT_UDP_PORT (4352)?

#if defined(ESP32)
  if (hw_info.model == SOFTRF_MODEL_PRIME_MK2 /* && hw_info.revision >= 8 */) {

    is_a_prime_mk2 = true;

    // use Serial2 as an auxillary port for data bridging (or ADS-B data input)
    uint32_t Serial2Baud = baudrates[settings->baudrate2];
    size_t Serial2BufSize = SerialBufSize;
    if (settings->rx1090 == ADSB_RX_GNS5892) {
        Serial2Baud = GNS5892_BAUDRATE;
        Serial2BufSize = GNS5892_INPUT_BUF_SIZE;
        Serial.println(F("Using Serial2 for ADS-B module"));
    }

    if (Serial2Baud == 0 || settings->gnss_pins == EXT_GNSS_39_4) {
        // note BAUD_DEFAULT here means Serial2 disabled, not 38400
        Serial.println(F("Serial2 NOT started"));
    } else {
        Serial2.setRxBufferSize(Serial2BufSize);
        uint8_t rx_pin = Serial2RxPin;   // VN
        if (hw_info.revision < 8)
            rx_pin = Serial0AltRxPin;    // VP
        const char *label = "Aux Serial";
        if (settings->rx1090)
            label = "ADS-B Module";
        if (ESP32_pin_reserved(rx_pin, false, label) == false
         && ESP32_pin_reserved(Serial2TxPin, false, label) == false) {
            Serial2.begin(Serial2Baud, SERIAL_8N1, rx_pin, Serial2TxPin, settings->invert2);
            has_serial2 = true;
            Serial.printf("Serial2 started on pins %d,%d at baud rate %d, logic:%d\r\n",
               rx_pin, Serial2TxPin, Serial2Baud, settings->invert2);
            delay(500);
            int limit = 199;
            if (Serial2.available() > 0) {
                Serial.println("Input data available on Serial2:");
                while (Serial2.available() > 0 && --limit > 0)
                    Serial.write(Serial2.read());
                Serial.println("...");
            }
        }
    }
  }
#endif

#if defined(USE_NMEA_CFG)
  const char *psrf_c = "PSRFC";
  int term_num = 1;

  C_Version.begin      (gnss, psrf_c, term_num++); /* 1 */
  C_Mode.begin         (gnss, psrf_c, term_num++);
  C_Protocol.begin     (gnss, psrf_c, term_num++);
  C_Band.begin         (gnss, psrf_c, term_num++);
  C_AcftType.begin     (gnss, psrf_c, term_num++);
  C_Alarm.begin        (gnss, psrf_c, term_num++);
  C_TxPower.begin      (gnss, psrf_c, term_num++);
  C_Volume.begin       (gnss, psrf_c, term_num++);
  C_Pointer.begin      (gnss, psrf_c, term_num++);
  C_NMEA_gnss.begin    (gnss, psrf_c, term_num++); /* 10 */
  C_NMEA_private.begin (gnss, psrf_c, term_num++);
  C_NMEA_traffic.begin (gnss, psrf_c, term_num++);
  C_NMEA_sensors.begin (gnss, psrf_c, term_num++);
  C_NMEA_Output.begin  (gnss, psrf_c, term_num++);
  C_GDL90_Output.begin (gnss, psrf_c, term_num++);
  C_D1090_Output.begin (gnss, psrf_c, term_num++);
  C_Stealth.begin      (gnss, psrf_c, term_num++);
  C_noTrack.begin      (gnss, psrf_c, term_num++);
  C_PowerSave.begin    (gnss, psrf_c, term_num  ); /* 19 */

  const char *psrf_d = "PSRFD";
  term_num = 1;

  D_Version.begin       (gnss, psrf_d, term_num++); /* 1 */
  D_id_method.begin     (gnss, psrf_d, term_num++);
  D_aircraft_id.begin   (gnss, psrf_d, term_num++);
  D_ignore_id.begin     (gnss, psrf_d, term_num++);
  D_follow_id.begin     (gnss, psrf_d, term_num++);
  D_baud_rate.begin     (gnss, psrf_d, term_num++);
  D_power_ext.begin     (gnss, psrf_d, term_num++);
  D_NMEA_debug.begin    (gnss, psrf_d, term_num++);
  D_debug_flags.begin   (gnss, psrf_d, term_num++);
  D_NMEA2.begin         (gnss, psrf_d, term_num++); /* 10 */
  D_NMEA2_gnss.begin    (gnss, psrf_d, term_num++);
  D_NMEA2_private.begin (gnss, psrf_d, term_num++);
  D_NMEA2_legacy.begin  (gnss, psrf_d, term_num++);
  D_NMEA2_sensors.begin (gnss, psrf_d, term_num++);
  D_NMEA2_debug.begin   (gnss, psrf_d, term_num++);
  D_relay.begin         (gnss, psrf_d, term_num++);
  D_bluetooth.begin     (gnss, psrf_d, term_num++);
  D_baudrate2.begin     (gnss, psrf_d, term_num++);
  D_invert2.begin       (gnss, psrf_d, term_num++);
  D_extern1.begin       (gnss, psrf_d, term_num++);
  D_extern2.begin       (gnss, psrf_d, term_num++); /* 21 */
  D_altpin0.begin       (gnss, psrf_d, term_num++);
  D_voice.begin         (gnss, psrf_d, term_num++);
  D_strobe.begin        (gnss, psrf_d, term_num++);

  const char *psrf_f = "PSRFF";
  term_num = 1;
  F_Version.begin       (gnss, psrf_f, term_num++); /* 1 */
  F_rx1090.begin        (gnss, psrf_f, term_num++);
  F_rx1090x.begin       (gnss, psrf_f, term_num++);
  F_mode_s.begin        (gnss, psrf_f, term_num++);
  F_gdl90_in.begin      (gnss, psrf_f, term_num++);
  F_gnss_pins.begin     (gnss, psrf_f, term_num++);
  F_ppswire.begin       (gnss, psrf_f, term_num++);
  F_logalarms.begin     (gnss, psrf_f, term_num++);
  F_sd_card.begin       (gnss, psrf_f, term_num++);
  F_logflight.begin     (gnss, psrf_f, term_num++);
  F_loginterval.begin   (gnss, psrf_f, term_num++);
  F_alt_udp.begin       (gnss, psrf_f, term_num++);
  F_tcpmode.begin       (gnss, psrf_f, term_num++);
  F_tcpport.begin       (gnss, psrf_f, term_num++);
  F_geoid.begin         (gnss, psrf_f, term_num++);
  F_freq_corr.begin     (gnss, psrf_f, term_num++); /* 15 */


  const char *psrf_s = "PSRFS";
  term_num = 1;
  S_Version.begin       (gnss, psrf_s, term_num++);
  S_label.begin         (gnss, psrf_s, term_num++);
  S_value.begin         (gnss, psrf_s, term_num++);

  const char *psrf_t = "PSRFT";
  term_num = 1;
  T_testmode.begin      (gnss, psrf_t, term_num++);

#if defined(USE_OGN_ENCRYPTION)
/* Security and privacy */
  const char *psrf_k = "PSRFK";
  term_num = 1;
  K_Version.begin      (gnss, psrf_k, term_num++);
  K_IGC_Key.begin      (gnss, psrf_k, term_num  );
#endif /* USE_OGN_ENCRYPTION */

#if defined(USE_SKYVIEW_CFG)
  const char *pskv_c = "PSKVC";
  term_num = 1;

  V_Version.begin      (gnss, pskv_c, term_num++); /* 1 */
  V_Adapter.begin      (gnss, pskv_c, term_num++);
  V_Connection.begin   (gnss, pskv_c, term_num++);
  V_Units.begin        (gnss, pskv_c, term_num++);
  V_Zoom.begin         (gnss, pskv_c, term_num++);
  V_Protocol.begin     (gnss, pskv_c, term_num++);
  V_Baudrate.begin     (gnss, pskv_c, term_num++);
  V_Server.begin       (gnss, pskv_c, term_num++);
  V_Key.begin          (gnss, pskv_c, term_num++);
  V_Rotate.begin       (gnss, pskv_c, term_num++); /* 10 */
  V_Orientation.begin  (gnss, pskv_c, term_num++);
  V_AvDB.begin         (gnss, pskv_c, term_num++);
  V_ID_Pref.begin      (gnss, pskv_c, term_num++);
  V_VMode.begin        (gnss, pskv_c, term_num++);
  V_Voice.begin        (gnss, pskv_c, term_num++);
  V_AntiGhost.begin    (gnss, pskv_c, term_num++);
  V_Filter.begin       (gnss, pskv_c, term_num++);
  V_PowerSave.begin    (gnss, pskv_c, term_num++);
  V_Team.begin         (gnss, pskv_c, term_num  ); /* 19 */
#endif /* USE_SKYVIEW_CFG */
#endif /* USE_NMEA_CFG */

#if defined(NMEA_TCP_SERVICE)
  if (settings->nmea_out  == DEST_TCP
   || settings->nmea_out2 == DEST_TCP
   || settings->gdl90_in  == DEST_TCP
   || settings->gdl90     == DEST_TCP) {
    // note: no TCP input possible unless TCP output destination is active
    if (settings->tcpmode == TCP_MODE_SERVER) {
        NmeaTCPServer.begin();
        Serial.print(F("NMEA TCP server has started at port: "));
        Serial.println(NMEA_TCP_PORT);
        NmeaTCPServer.setNoDelay(true);
        TCP_active = true;
    } else if (settings->tcpmode == TCP_MODE_CLIENT) {
        if (WiFi_connect_TCP()) {
            Serial.print(F("Connected as TCP client to host: "));
            Serial.println(settings->host_ip);
            TCP_active = true;
        } else {
            Serial.print(F("Failed to connect to host: "));
            Serial.println(settings->host_ip);
        }
    }
  }
#endif /* NMEA_TCP_SERVICE */

#if defined(USE_NMEALIB)
  memset(&nmealib_buf, 0, sizeof(nmealib_buf));
#endif /* USE_NMEALIB */

  memset(GPGGA_Copy, 0, sizeof(GPGGA_Copy));

  PGRMZ_TimeMarker = millis();

#if defined(ENABLE_AHRS)
  RPYL_TimeMarker = millis();
#endif /* ENABLE_AHRS */

  sendPFLAV(true);
}

void NMEA_Out(uint8_t dest, const char *buf, size_t size, bool nl)
{
#if 0
if (NMEA_Source != DEST_NONE) {     // only external sources
  Serial.print("NMEA_Out(");
  Serial.print(NMEA_Source);
  Serial.print(",");
  Serial.print(dest);
  Serial.print("): ");
  //Serial.write(buf, size);
  Serial.print(buf);
  if (nl) Serial.print("\r\n");
}
#endif

  if (dest == NMEA_Source)          // do not echo NMEA back to its source
    return;                         // NMEA_Source = DEST_NONE for internal NMEA

  if (dest == settings->gdl90_in)   // do not send NMEA to GDL90 source
    return;

  switch (dest)
  {
  case DEST_UART:
    {
      if (NMEA_Source == DEST_USB)   // do not echo USB to UART
        return; 
      if (SoC->UART_ops) {
        SoC->UART_ops->write((const byte*) buf, size);
        if (nl)
          SoC->UART_ops->write((const byte *) "\r\n", 2);
      } else {
        Serial.write(buf, size);
        if (nl)
          Serial.write((const byte *) "\r\n",2);
      }
    }
    break;
  case DEST_UART2:
    {
      if (has_serial2) {
        Serial2.write(buf, size);
        if (nl)
          Serial2.write((const byte *) "\r\n",2);
      }
    }
    break;
#if !defined(EXCLUDE_WIFI)
  case DEST_UDP:
    {
      size_t udp_size = size;

      if (size > sizeof(UDPpacketBuffer) - 2)
        udp_size = sizeof(UDPpacketBuffer) - 2;
      memcpy(UDPpacketBuffer, buf, udp_size);

      if (nl) {
        UDPpacketBuffer[udp_size++] = '\r';
        UDPpacketBuffer[udp_size++] = '\n';
      }

      SoC->WiFi_transmit_UDP(UDP_NMEA_Output_Port, (byte *) UDPpacketBuffer, udp_size);
    }
    break;
  case DEST_TCP:
    {
#if defined(NMEA_TCP_SERVICE)
      if (TCP_active) {
        WiFi_transmit_TCP(buf, size);
        if (nl)
          WiFi_transmit_TCP("\r\n", 2);
      }
#endif
    }
    break;
#endif
#if defined(ARDUINO_ARCH_NRF52)
  case DEST_USB:
    {
      if (NMEA_Source == DEST_UART)   // do not echo UART to USB
        return; 
      if (SoC->USB_ops) {
        SoC->USB_ops->write((const byte *) buf, size);
        if (nl)
          SoC->USB_ops->write((const byte *) "\r\n", 2);
      }
    }
    break;
#endif
  case DEST_BLUETOOTH:
    {
      if (BTactive && SoC->Bluetooth_ops) {
        SoC->Bluetooth_ops->write((const byte *) buf, size);
        if (nl)
          SoC->Bluetooth_ops->write((const byte *) "\r\n", 2);
      }
    }
    break;
  case DEST_NONE:
  default:
    break;
  }
  yield();
}

void NMEA_Outs(uint16_t nmeatype, const char *buf, unsigned int size, bool nl)
{
    uint16_t genus  = (nmeatype & 0xFF00);
    uint8_t species = (uint8_t)(nmeatype & 0x00FF);
    if (species == 0)  species = NMEA_BASIC;   // = 1
    bool out1 = false;
    bool out2 = false;
    switch (genus) {
      case NMEA_G:
        if ((settings->nmea_g)  & species)  out1 = true;
        if ((settings->nmea2_g) & species)  out2 = true;
        break;
      case NMEA_T:
        if ((settings->nmea_t)  & species)  out1 = true;
        if ((settings->nmea2_t) & species)  out2 = true;
        break;
      case NMEA_S:
        if ((settings->nmea_s)  & species)  out1 = true;
        if ((settings->nmea2_s) & species)  out2 = true;
        break;
      case NMEA_E:
        if ((settings->nmea_e)  & species)  out1 = true;
        if ((settings->nmea2_e) & species)  out2 = true;
        break;
      case NMEA_D:
        if ((settings->nmea_d)  & species)  out1 = true;
        if ((settings->nmea2_d) & species)  out2 = true;
        break;
      case NMEA_P:
        if ((settings->nmea_p)  & species)  out1 = true;
        if ((settings->nmea2_p) & species)  out2 = true;
        break;
      default:
        return;  // no output
    }
    if (out1)
        NMEA_Out(settings->nmea_out,  buf, size, nl);
    if (out2)
        NMEA_Out(settings->nmea_out2, buf, size, nl);

#if defined(ESP32)   // only on SD card
    if ((out1 || out2) && NMEALogOpen) {
        if (NMEALog.write((const uint8_t *) buf, size) < size) {
            NMEALog.close();
            const char *p = "Write to NMEAlog failed\r\n";
            Serial.print(p);
            SD_log(p);
            FlightLogComment(p);
        }
        if (nl)
            NMEALog.write((const uint8_t *) "\r\n", 2);
    }
#endif
}

void NMEAOutC(int nmeatype)
{
      unsigned int nmealen = NMEA_add_checksum();
      NMEA_Outs(nmeatype, NMEABuffer, nmealen, false);
}

void NMEAOutD()
{
    NMEA_Outs(NMEA_D_ALL, NMEABuffer, strlen(NMEABuffer), false);
}

bool NMEA_encode(const char *buf, const int len)
{
    for (int i=0; i<len; i++) {
        if (gnss.encode(buf[i]))     // valid sentence
            return true;
    }
    return false;
}

// Send buffered sentences to bridged outputs
bool NMEA_bridge_sent = false;
void NMEA_bridge_send(char *buf, int len)
{
#if 0
    Serial.print("bridge_send(source=");
    Serial.print(NMEA_Source);
    Serial.print("): ");
    //Serial.write(buf, len);
    Serial.print(buf);
#endif
    // First check whether it is GNSS or FLARM sentences, skip them (assume echo)
    if (buf[1]=='G' && buf[2]=='P')
        return;
    if (buf[1]=='P' && buf[2]=='F' && buf[3]=='L' && buf[4]=='A')
        return;

#if defined(USE_NMEA_CFG)
    // Also trap PSRF/PSKV config sentences, process internally instead
    if (buf[1]=='P' && buf[2]=='S') {
      if ((buf[3]=='R' && buf[4]=='F') || (buf[3]=='K' && buf[4]=='V')) {
        NMEA_bridge_sent = true;   // not really sent, but substantial processing
        // if checksum is wrong, send it back to the source:
        if (buf[len-5] == '*') {
            char c1 = buf[len-4];
            char c2 = buf[len-3];
            buf[len-4] = '\0';   // include the '*'
            len = NMEA_add_checksum(buf);
            if (buf[len-4] != c1 || buf[len-3] != c2) {
                uint8_t dest = NMEA_Source;
                NMEA_Source = DEST_NONE;
                NMEA_Out(dest, "\r\n-- correct checksum is: ", 26, false);
                NMEA_Out(dest, buf, len, true);   // add blank lines before and after
                NMEA_Source = dest;
                return;
            }
        }
        if (NMEA_encode(buf, len))             // valid sentence
            NMEA_Process_SRF_SKV_Sentences();
        return;                                // even if not valid sentence
      }
    }
#endif

    // remaining are sentences that are NOT GNSS, FLARM or PSRF/PSKV
    // forward them to configured outputs, but do not echo to source

#if 1
    Serial.print("bridge_send(source=");
    Serial.print(NMEA_Source);
    Serial.print("): ");
    //Serial.write(buf, len);
    Serial.print(buf);
#endif

    if (settings->nmea_e && settings->nmea_out != NMEA_Source) {
        NMEA_Out(settings->nmea_out, buf, len, false);
        NMEA_bridge_sent = true;
    }
    if (settings->nmea2_e && settings->nmea_out2 != NMEA_Source) {
        NMEA_Out(settings->nmea_out2, buf, len, false);
        NMEA_bridge_sent = true;
    }
}

// set up separate buffers for potentially-bridged output from each input port,
// this is the common code for all these buffers:
void NMEA_bridge_buf(char c, char* buf, int& n)
{
    if (c == '$') {
        n = 0;
        // start new sentence, drop any preceding data
        // fall through to buf[n++] = c;
    } else if (n == 0) {      // wait for a '$' (or '!')
        if (c != '!')
            return;
        // if '!', start new sentence of some related protocols
        // fall through to buf[n++] = c;
    } else if (c=='\r' || c=='\n') {
        if (n > 5 && n <= 128) {
            // sentences missing "*xx" ending are ignored unless started with '!'
            // >>> or could forward all sentences even without checksum?
            if (buf[0] == '!' || buf[n-3] == '*') {
                buf[n++] = '\r';
                buf[n++] = '\n';      // add a proper line-ending
                buf[n]   = '\0';
                NMEA_bridge_send(buf, n);
            }
        }
        n = 0;
        return;
    } else if (n >= 128) {
        n = 0;
        return;
    }
    buf[n++] = c;
}

void NMEA_loop()
{
#if defined(ESP32)
  // open NMEA log file only after there is a GNSS fix
  // - to be able to embed the date into the file name
  if (next_SD_sync==0 && isValidGNSSFix()) {
    next_SD_sync = millis() + (10*60*1000);
    if (settings->log_nmea && SD_is_mounted) {
      char filename[20];
      strcpy(filename,"/logs/ymd1NMEA.txt");
      makeLogNameDate(filename+6);   // fills in ymd
      while (SD.exists(filename)) {
          if (filename[6+3] == '9')
              filename[6+3] = 'A';
          else
              ++filename[6+3];
          if (filename[6+3] == 'Z')
              break;
      }
      if (filename[6+3] != 'Z') {
          NMEALog = SD.open(filename, FILE_WRITE);
          if (NMEALog) {
              NMEALogOpen = true;
              Serial.print("New NMEA log file: ");
              Serial.println(filename);
          } else {
              Serial.println("Failed to open NMEA log file for writing");
          }
      } else {
          Serial.println("too many NMEA logs for today");
      }
    }
  }
  if (NMEALogOpen && millis() > next_SD_sync) {  // every 10 minutes
      NMEALog.flush();     // keep most data in case of power loss
      next_SD_sync = millis() + (10*60*1000);
  }
#endif

  NMEA_bridge_sent = false;

  NMEA_Source = DEST_NONE;

    /*
     * Check SW/HW UARTs, USB, TCP and BT for data
     */

    bool gdl90 = false;

    if ((settings->debug_flags & DEBUG_SIMULATE) == 0
#if defined(ESP32)
#if defined(USE_SD_CARD)
        || SIMfileOpen
#endif
        || (settings->gnss_pins != EXT_GNSS_NONE)
#endif
        ) {      // if not reading sim data from Serial, poll Serial here:

#if defined(ARDUINO_ARCH_NRF52)
      static char usb_buf[128+3];
      static int usb_n = 0;
      if (SoC->USB_ops) {
        gdl90 = (settings->gdl90_in == DEST_USB);
        while (SoC->USB_ops->available() > 0) {   // may mirror UART?
            int c = SoC->USB_ops->read();
            //if (settings->nmea_out == DEST_UART || settings->nmea_out2 == DEST_UART)
            //    continue;
            NMEA_Source = DEST_USB;
//#if defined(ESP32)
//            if (gdl90)
//                GDL90_bridge_buf(c, usb_buf, usb_n);
//            else
//#endif
                NMEA_bridge_buf(c, usb_buf, usb_n);
        }
      }
#endif

#if ! defined(ARDUINO_ARCH_NRF52)
      static char uart_buf[128+3];
      static int uart_n = 0;
      gdl90 = (settings->gdl90_in == DEST_UART);
      while (Serial.available() > 0) {
          NMEA_Source = DEST_UART;
          int c = Serial.read();
#if defined(ESP32)
          if (gdl90)
              GDL90_bridge_buf(c, uart_buf, uart_n);
          else
#endif
              NMEA_bridge_buf(c, uart_buf, uart_n);
      }
#endif

    }  // end of polling Serial

#if defined(ESP32)
  if (is_a_prime_mk2) {

    static char uart2_buf[128+3];
    static int uart2_n = 0;
    if (has_serial2) {
        if (settings->rx1090 == ADSB_RX_GNS5892) {
            // Serial2 is dedicated to the ADS-B receiver module
            gns5892_loop();
        } else {
            gdl90 = (settings->gdl90_in == DEST_UART2);
            while (Serial2.available() > 0) {
                NMEA_Source = DEST_UART2;
                int c = Serial2.read();
                if (gdl90)
                    GDL90_bridge_buf(c, uart2_buf, uart2_n);
                else
                    NMEA_bridge_buf(c, uart2_buf, uart2_n);
            }
        }
        if (NMEA_bridge_sent) {  // also set by GDL90_bridge_buf() or gns5892_loop()
            yield();
            return;           // check wireless inputs next time around
        }
    }

  }  // end if (is_a_prime_mk2)
#endif

    static char bt_buf[128+3];
    static int bt_n = 0;
    if (SoC->Bluetooth_ops) {
      gdl90 = (settings->gdl90_in == DEST_BLUETOOTH);
      while (BTactive && SoC->Bluetooth_ops->available() > 0) {
          NMEA_Source = DEST_BLUETOOTH;
          int c = SoC->Bluetooth_ops->read();
#if defined(ESP32)
          if (gdl90)
              GDL90_bridge_buf(c, bt_buf, bt_n);
          else
#endif
              NMEA_bridge_buf(c, bt_buf, bt_n);
      }
    }

    if (NMEA_bridge_sent) {
        yield();
        return;     // probably not using other wireless, but in any case postpone
    }

#if defined(ESP32)

    static char udp_buf[128+3];
    static int udp_n = 0;
    while (udp_is_ready) {
      gdl90 = (settings->gdl90_in == DEST_UDP);
      size_t size = WiFi_Receive_UDP((uint8_t *) UDPpacketBuffer, sizeof(UDPpacketBuffer));
      if (size <= 0)
          break;
      NMEA_Source = DEST_UDP;
      for (size_t i=0; i < size; i++) {
          char c = UDPpacketBuffer[i];
          if (gdl90)
              GDL90_bridge_buf(c, udp_buf, udp_n);
          else
              NMEA_bridge_buf(c, udp_buf, udp_n);
      }
    }

#if defined(NMEA_TCP_SERVICE)
    static char tcpinbuf[128];
    static char tcpoutbuf[MAX_NMEATCP_CLIENTS][128+3];
    static int tcp_n[MAX_NMEATCP_CLIENTS] = {0};
//  static bool tcp_n_init = false;
    if (TCP_active) {
      gdl90 = (settings->gdl90_in == DEST_TCP);
      if (settings->tcpmode == TCP_MODE_CLIENT) {
        while (1) {
          int n = WiFi_receive_TCP(tcpinbuf, 128);
          if (n <= 0)
              break;
          NMEA_Source = DEST_TCP;
          for (int i=0; i<n; i++) {
               if (gdl90)
                   GDL90_bridge_buf(tcpinbuf[i], tcpoutbuf[0], tcp_n[0]);
               else
                   NMEA_bridge_buf(tcpinbuf[i], tcpoutbuf[0], tcp_n[0]);
          }
        }
      }
      else if (settings->tcpmode == TCP_MODE_SERVER) {
        // Note: this may collect input from more than one client.  To keep their sentences
        // from getting mixed need MAX_NMEATCP_CLIENTS (which is just 2) separate buffers.
        // Note: same NMEA_Source for all clients, won't forward from one to another
        for (int i = 0; i < MAX_NMEATCP_CLIENTS; i++) {
          if (NmeaTCP[i].client && NmeaTCP[i].client.connected()) {
//            if (! tcp_n_init)
//              tcp_n[i] = 0;
            while (NmeaTCP[i].client.available()) {
                NMEA_Source = DEST_TCP;
                int c = NmeaTCP[i].client.read();
                if (gdl90)
                    GDL90_bridge_buf(c, tcpoutbuf[i], tcp_n[i]);
                else
                    NMEA_bridge_buf(c, tcpoutbuf[i], tcp_n[i]);
            }
          }
        }
//        tcp_n_init = true; 
      }
    }
#endif

#endif // ESP32

    if (NMEA_bridge_sent) {
        yield();
        return;           // process sensors next time around
    }

  NMEA_Source = DEST_NONE;  // for all internal messages sent below

  sendPFLAV(false);

  if (baro_chip != NULL && ThisAircraft.pressure_altitude != 0.0 && isTimeToPGRMZ()) {

    int altitude = constrain(
            (int) (ThisAircraft.pressure_altitude * _GPS_FEET_PER_METER),
            -1000, 60000);

    /* https://developer.garmin.com/downloads/legacy/uploads/2015/08/190-00684-00.pdf */
    snprintf_P(NMEABuffer, sizeof(NMEABuffer), PSTR("$PGRMZ,%d,f,%c*"),
               altitude, isValidGNSSFix() ? '3' : '1'); /* feet , 3D fix */
    NMEAOutC(NMEA_S);

#if 0
// moved to baro_loop()
#if !defined(EXCLUDE_LK8EX1)
    char str_Vcc[6];
    dtostrf(Battery_voltage(), 3, 1, str_Vcc);

    snprintf_P(NMEABuffer, sizeof(NMEABuffer), PSTR("$LK8EX1,999999,%d,%d,%d,%s*"),
            constrain((int) ThisAircraft.pressure_altitude, -1000, 99998), /* meters */
            (int) ((ThisAircraft.vs * 100) / (_GPS_FEET_PER_METER * 60)),  /* cm/s   */
            constrain((int) Baro_temperature(), -99, 98),                  /* deg. C */
            str_Vcc);
    NMEAOutC(NMEA_S_LK8);

#endif /* EXCLUDE_LK8EX1 */
#endif

    PGRMZ_TimeMarker = millis();
  }

#if defined(ENABLE_AHRS)
  if (((settings->nmea_s  | settings->nmea2_s) & NMEA_S_AHRS) && isTimeToRPYL()) {

    AHRS_NMEA();

    RPYL_TimeMarker = millis();
  }
#endif /* ENABLE_AHRS */

#if defined(NMEA_TCP_SERVICE)

    if (TCP_active && settings->tcpmode == TCP_MODE_SERVER) {

    // TCP clients housekeeping:
  
    int i;

    if (NmeaTCPServer.hasClient()) {
      for(i = 0; i < MAX_NMEATCP_CLIENTS; i++) {
        // find free/disconnected spot
        if (!NmeaTCP[i].client || !NmeaTCP[i].client.connected()) {
          if(NmeaTCP[i].client) {
            NmeaTCP[i].client.stop();
            NmeaTCP[i].connect_ts = 0;
          }
          NmeaTCP[i].client = NmeaTCPServer.available();
          NmeaTCP[i].connect_ts = OurTime;
          NmeaTCP[i].ack = false;
          NmeaTCP[i].client.print(F("PASS?"));
          break;
        }
      }
      if (i >= MAX_NMEATCP_CLIENTS) {
        // no free/disconnected spot so reject
        NmeaTCPServer.available().stop();
      }
    }

    for (i = 0; i < MAX_NMEATCP_CLIENTS; i++) {
      if (NmeaTCP[i].client && NmeaTCP[i].client.connected() &&
         !NmeaTCP[i].ack && NmeaTCP[i].connect_ts > 0 &&
         (OurTime - NmeaTCP[i].connect_ts) >= NMEATCP_ACK_TIMEOUT) {

          if (! is_a_prime_mk2) {
              /* Clean TCP input buffer from any pass codes sent by client */
              while (NmeaTCP[i].client.available()) {
                char c = NmeaTCP[i].client.read();
                yield();
              }
          }

          /* send acknowledge */
          NmeaTCP[i].client.print(F("AOK"));
          NmeaTCP[i].ack = true;
      }
    }
  }
#endif

}

#if defined(ESP32)
void flushNMEAlog()
{
  if (NMEALogOpen)
      NMEALog.flush();
}
void closeNMEAlog()
{
  if (NMEALogOpen) {
      NMEALog.close();
      NMEALogOpen = false;
  }
}
#endif

void NMEA_fini()
{
#if defined(NMEA_TCP_SERVICE)
  if (TCP_active) {
    if (settings->tcpmode == TCP_MODE_SERVER)
        NmeaTCPServer.stop();
    else if (settings->tcpmode == TCP_MODE_CLIENT)
        WiFi_disconnect_TCP();
  }
#endif /* NMEA_TCP_SERVICE */
#if defined(ESP32)
  closeNMEAlog();
#endif
}

void NMEA_Export()
{
    NMEA_Source = DEST_NONE;

    float voltage = Battery_voltage();
    if (voltage < BATTERY_THRESHOLD_INVALID)
        voltage = 0;

#if !defined(EXCLUDE_SOFTRF_HEARTBEAT)
    static int beatcount = 0;
    if (++beatcount >= 10) {
      beatcount = 0;
      unsigned int nmealen;
      int nacft = Traffic_Count();   // maxrssi and adsb_acfts are byproducts
      snprintf_P(NMEABuffer, sizeof(NMEABuffer),
              PSTR("$PSRFH,%06X,%d,%d,%d,%d,%d,%d,%d,%d*"),
              ThisAircraft.addr, settings->rf_protocol,
              millis(), (int)(voltage*100), SoC->getFreeHeap(),
              rx_packets_counter, tx_packets_counter, nacft, maxrssi);
      NMEAOutC(NMEA_T);
      // also output an LK8EX1 sentence here if not sent from baro_loop()
      // - just to report the battery charge percentage
      // - LK8000 specs say to send percent instead of volts send as an integer, percent+1000
      if (baro_chip == NULL) {
          snprintf_P(NMEABuffer, sizeof(NMEABuffer), PSTR("$LK8EX1,999999,99999,9999,99,%d*"),
              1000+(int)Battery_charge());
          NMEAOutC(NMEA_S_LK8);
      }
    }
#endif /* EXCLUDE_SOFTRF_HEARTBEAT */

    if (! (settings->nmea_t || settings->nmea2_t))
         return;

    container_t *cip, *fop;
    int alt_diff;
    int abs_alt_diff;
    float distance;
    float adj_dist;
    float bearing;

    int alarm_level = ALARM_LEVEL_NONE;
    uint32_t follow_id = settings->follow_id;

    /* High priority object (most relevant target) */
    int HP_index = MAX_TRACKING_OBJECTS;
    int HP_alt_diff = 0;
    int HP_alarm_level = ALARM_LEVEL_NONE;
    float HP_adj_dist  = 999999999;
    float HP_distance  = 999999999;
    float HP_bearing = 0;
    uint32_t HP_addr = 0;
    bool HP_nondir = false;
    bool HP_stealth = false;
    int total_objects = 0;
    int head = 0;

    bool has_Fix = (isValidFix() || (settings->mode == SOFTRF_MODE_TXRX_TEST));
    bool deeper = (settings->debug_flags & DEBUG_DEEPER);

    if (has_Fix) {

      float maxdistance = (settings->hrange? 1000 * settings->hrange : 100000);
      float maxaltdiff  = (settings->vrange?  100 * settings->vrange :  20000);

      for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {

        cip = &Container[i];

        if (cip->addr && ((OurTime - cip->timestamp) <= settings->expire)) {
#if 0
          Serial.println(i);
          Serial.printf("%06X\r\n", cip->addr);
          Serial.println(cip->latitude, 4);
          Serial.println(cip->longitude, 4);
          Serial.println(cip->altitude);
          Serial.println(cip->addr_type);
          Serial.println(cip->vs);
          Serial.println(cip->aircraft_type);
          Serial.println(cip->stealth);
          Serial.println(cip->no_track);
#endif
          bool stealth = (cip->stealth || ThisAircraft.stealth);  /* reciprocal */

          alarm_level = cip->alarm_level;

          distance = cip->distance;
          bearing = cip->bearing;

          alt_diff = (int) cip->alt_diff;                 /* sent to NMEA */
          abs_alt_diff = (int) fabs(cip->adj_alt_diff);   /* used to pick high-priority traffic */
          adj_dist = cip->adj_distance;

          bool show = true;

          /* mask some data following FLARM protocol: */
          if (stealth && alarm_level <= ALARM_LEVEL_CLOSE) {
            if (distance > STEALTH_DISTANCE || abs(alt_diff) > STEALTH_VERTICAL) {
                show = false;
            }
          }

       /* if (cip->protocol == RF_PROTOCOL_LEGACY && cip->airborne == 0)
                   show = false; */

          if ((alarm_level > ALARM_LEVEL_NONE
               //|| (distance < ALARM_ZONE_NONE && abs_alt_diff < VERTICAL_VISIBILITY_RANGE)
               || (cip->tx_type <= TX_TYPE_ADSB)   // filtering done in ADS-B processing
               || (distance < maxdistance && abs_alt_diff < maxaltdiff)
               || cip->addr == follow_id)
              && show) {

             /* put candidate traffic to report into a sorted list */

             int next, previous;
             if (total_objects == 0) {  /* first inserted into the list */
               head = i;
               cip->next = MAX_TRACKING_OBJECTS;
               total_objects = 1;
             } else {
               next = head;
               while (next < MAX_TRACKING_OBJECTS) {
                 fop = &Container[next];
                 if (fop->alarm_level <= alarm_level
                  && fop->addr != follow_id
                  && fop->adj_distance >= adj_dist)
                        break;   /* insert before this one */
                 /* else */
                   previous = next;  /* the preceding list entry */
                   next = fop->next;
               }
               cip->next = next;
               if (head == next)
                 head = i;
               else
                 Container[previous].next = i;
               total_objects++;
             }

             /* Alarm or close traffic is treated as highest priority */
             if (alarm_level > HP_alarm_level ||
                    (alarm_level == HP_alarm_level && adj_dist <= HP_adj_dist)) {  // was <
                HP_bearing = bearing;
                HP_alt_diff = alt_diff;
                HP_alarm_level = alarm_level;
                HP_distance = distance;
                HP_adj_dist = adj_dist;
                HP_addr = (stealth? 0xFFFFF0 + i : cip->addr);
                HP_stealth = stealth;
                HP_nondir = (cip->tx_type <= TX_TYPE_S);
             }
          }
        }
      }

      fop = &Container[head];

      for (int i=0; i < total_objects && i < MAX_NMEA_OBJECTS; i++) {

         // note that MAX_NMEA_OBJECTS (6) < MAX_TRACKING_OBJECTS (8)

         // may want to skip the HP object if there are many to report
         // since it will be in the PFLAU sentence - but XCsoar etc
         // seem to ignore the PFLAU, so report the HP object both ways
         //if (total_objects < MAX_NMEA_OBJECTS || fop->addr != HP_addr) {

         uint8_t addr_type = fop->addr_type;
         if (addr_type > ADDR_TYPE_FLARM)
             addr_type = ADDR_TYPE_RANDOM;
         uint32_t id = fop->addr;
         alarm_level = fop->alarm_level;
         alt_diff = (int) (fop->alt_diff);  /* sent to NMEA */
         int dx = fop->dx;
         int dy = fop->dy;

         bool stealth = (fop->stealth || ThisAircraft.stealth);  /* reciprocal */
         if (stealth) {
            if (abs(alt_diff) > 300)
                continue;               // do not report this aircraft
           if (dx*dx + dy*dy > (2000*2000))
                continue;               // do not report this aircraft
           id = 0xFFFFF0 + i;            // show as anonymous
           addr_type = ADDR_TYPE_RANDOM;    // (0) - not ADDR_TYPE_ANONYMOUS
           if (alarm_level <= ALARM_LEVEL_CLOSE)
               alt_diff = (alt_diff & 0xFFFFFF00) + 128;   /* fuzzify */
         }

         // if this aircraft is not airborne, no-track targets should not be reported,
         //  unless the target is closer than 200m horizontally and 100m vertically.

         if (fop->no_track && ! ThisAircraft.airborne) {
            if (abs(alt_diff) > 100)
                continue;               // do not report this aircraft
           if (dx*dx + dy*dy > (200*200))
                continue;               // do not report this aircraft
         }

         if (fop->distance > 99000)     // too-far ADS-B aircraft
             continue;

         // If either target or this aircraft is "stealth" then:
         //   course should be empty
         //   speed should be empty
         //   climbrate should be empty
         //   rel_alt should be fuzzyfied if closer than 2k/300m but no alarm

         // For non-directional targets:
         //   <RelativeEast>  (dx) needs to be empty (not zero)
         //   <RelativeNorth> (dy) needs to be the estimated distance
         //   course should be empty
         //   speed should be empty
         //   climbrate should be empty

         char str_dx[8];                      // need room for a minus sign
         snprintf(str_dx, 8, "%d", dx);
         char str_climb_rate[8];
         char str_course[4];
         char str_speed[4];
         str_climb_rate[0] = '\0';
         str_course[0] = '\0';
         str_speed[0] = '\0';
         if (fop->tx_type <= TX_TYPE_S) {     // a non-directional target
             dy = (int) fop->distance;
             if (dy > 2*ALARM_ZONE_LOW && !deeper)
                 continue;                   // only report if close (2000m)
             str_dx[0] = '\0';
         } else if (! stealth) {
             dtostrf(
               constrain(fop->vs / (_GPS_FEET_PER_METER * 60.0), -32.7, 32.7),
               5, 1, str_climb_rate);
             dtostrf(
               constrain(fop->course, 0.0, 360.0),
               3, 0, str_course);
             float speed  = fop->speed * _GPS_MPS_PER_KNOT;
             if (fop->airborne && speed == 0)
                 speed = 0.1;
             if (! fop->airborne && speed != 0)
                 speed = 0.0;
             dtostrf(
               constrain(speed, 0.0, 999.0),
               3, 0, str_speed);
         }

         if (alarm_level > ALARM_LEVEL_NONE)  --alarm_level;
           /* for NMEA export bypass CLOSE added between NONE and LOW */

         // data_source should be 1, 3, 4, 6 for ADS-B, ADS-R, TIS-B, Mode-S
         //   (GDL90 should probably be treated as ADS-B)
         // convert tx_type to the code used by FLARM:
         int data_source = data_source_code[fop->tx_type];

         /*
          * When callsign is available - send it to a NMEA client.
          * If it is not - generate a callsign substitute,
          * based upon a protocol ID and the ICAO address
          */

         if (settings->pflaa_cs) {

           const char *prefix;
           if (fop->tx_type > TX_TYPE_ADSB) {
             bool landed_out = (fop->aircraft_type == AIRCRAFT_TYPE_UNKNOWN
                                && fop->airborne == 0 && fop->protocol == RF_PROTOCOL_LATEST);
             if (landed_out)
               prefix = "LND";
             else
               prefix = NMEA_CallSign_Prefix[fop->protocol];   // FLR, FLO, OGN, ADL
           } else if (fop->tx_type == TX_TYPE_ADSB) {
               prefix = "ADS";
           } else if (fop->tx_type == TX_TYPE_ADSR) {
               prefix = "ADR";
           } else if (fop->tx_type == TX_TYPE_TISB) {
               prefix = "ADT";
           } else {                         // Mode A,C,S
               prefix = "MDS";
           }

           if (fop->callsign[0] == '\0') {     // no callsign, substitute ID

             snprintf_P(NMEABuffer, sizeof(NMEABuffer),
//            PSTR("$PFLAA,%d,%d,%s,%d,%d,%06X!%s_%06X,%d,,%d,%s,%d,%d,%d,%d" PFLAA_EXT1_FMT "*"),
// aircraft type is supposed to be hex:
              PSTR("$PFLAA,%d,%d,%s,%d,%d,%06X!%s_%06X,%s,,%s,%s,%X,%d,%d,%d" PFLAA_EXT1_FMT "*"),
              alarm_level, dy, str_dx,
              alt_diff, addr_type, id, prefix, id,
              ltrim(str_course), ltrim(str_speed), ltrim(str_climb_rate), fop->aircraft_type,
              (fop->no_track? 1 : 0), data_source, fop->rssi  PFLAA_EXT1_ARGS );

           } else {   /* there is a callsign from incoming data */

             if (fop->callsign[sizeof(fop->callsign)-1] != '?')   // not set up by icao_to_n()
                 fop->callsign[sizeof(fop->callsign)-1] = '\0';   // ensure termination

             snprintf_P(NMEABuffer, sizeof(NMEABuffer),
              PSTR("$PFLAA,%d,%d,%s,%d,%d,%06X!%s_%s,%s,,%s,%s,%X,%d,%d,%d" PFLAA_EXT1_FMT "*"),
              alarm_level, dy, str_dx,
              alt_diff, addr_type, id, prefix, fop->callsign,
              ltrim(str_course), ltrim(str_speed), ltrim(str_climb_rate), fop->aircraft_type,
              (fop->no_track? 1 : 0), data_source, fop->rssi  PFLAA_EXT1_ARGS );
           }

         } else {    // skip the callsign

           snprintf_P(NMEABuffer, sizeof(NMEABuffer),
              PSTR("$PFLAA,%d,%d,%s,%d,%d,%06X,%s,,%s,%s,%X,%d,%d,%d" PFLAA_EXT1_FMT "*"),
              alarm_level, dy, str_dx,
              alt_diff, addr_type, id,
              ltrim(str_course), ltrim(str_speed), ltrim(str_climb_rate), fop->aircraft_type,
              (fop->no_track? 1 : 0), data_source, fop->rssi  PFLAA_EXT1_ARGS );
         }

         NMEAOutC(NMEA_T);

        //}  /* done skipping the HP object */

        if (fop->next >= MAX_TRACKING_OBJECTS)  break;    /* belt and suspenders */

        fop = &Container[fop->next];
      }
    }

    /* One PFLAU NMEA sentence is mandatory regardless of traffic reception status */
    int power_status = (voltage > 0 && voltage < Battery_threshold()) ?
                         POWER_STATUS_BAD : POWER_STATUS_GOOD;

    if (total_objects > 0) {
        if (HP_addr == 0) {
           /* no aircraft has been identified as high priority, use */
           /*  the aircraft from the top of the sorted list, if any */
           fop = &Container[head];
           if (fop->addr) {
               HP_bearing = fop->bearing;
               HP_alt_diff = fop->alt_diff;
               HP_alarm_level = fop->alarm_level;
               HP_distance = fop->distance;
               HP_nondir = false;
               if (fop->stealth || ThisAircraft.stealth) {
                   HP_addr = 0xFFFFF0;
                   HP_stealth = true;
               } else {
                   HP_addr = fop->addr;
                   HP_stealth = false;
               }
           }
        }
    }

    int gps_status = (ThisAircraft.airborne ? GNSS_STATUS_3D_MOVING : GNSS_STATUS_3D_GROUND);
    int tx_status = (settings->txpower == RF_TX_POWER_OFF ? TX_STATUS_OFF : TX_STATUS_ON);
#if defined(ESP32)
    if (do_alarm_demo) {
        if ((millis() - SetupTimeMarker) < (1000*STROBE_INITIAL_RUN)) {
            gps_status = GNSS_STATUS_3D_MOVING;
            HP_alarm_level = ALARM_LEVEL_IMPORTANT;
        } else {
            OLED_no_msg();
            do_alarm_demo = false;  // turn demo off here, in case buzzer is set to OFF
        }
    } else
#endif
    {
        if (! has_Fix) {
            gps_status = GNSS_STATUS_NONE;
            tx_status = TX_STATUS_OFF;
        }
    }
    if (HP_addr) {
        if (HP_stealth && HP_alarm_level <= ALARM_LEVEL_CLOSE) {
            HP_alt_diff = (HP_alt_diff & 0xFFFFFF00) + 128;   /* fuzzify */
        }
        // rel_bearing needs to be empty (not zero) for non-directional targets
        char str_relbrg[6] = "";
        if (! HP_nondir) {             // not a non-directional target
            int rel_bearing = (int) (HP_bearing - ThisAircraft.course);
            rel_bearing += (rel_bearing < -180 ? 360 : (rel_bearing > 180 ? -360 : 0));
            snprintf(str_relbrg, 6, "%d", rel_bearing);
        }
        if (HP_alarm_level > ALARM_LEVEL_NONE)  --HP_alarm_level;
        int alarm_type = (HP_alarm_level > 0)? ALARM_TYPE_AIRCRAFT : ALARM_TYPE_TRAFFIC;
        snprintf_P(NMEABuffer, sizeof(NMEABuffer),
                PSTR("$PFLAU,%d,%d,%d,%d,%d,%s,%d,%d,%u,%06X" PFLAU_EXT1_FMT "*"),
                total_objects, tx_status, gps_status,
                power_status, HP_alarm_level, str_relbrg,
                alarm_type, HP_alt_diff, (int) HP_distance, HP_addr
                PFLAU_EXT1_ARGS );
    } else if (do_alarm_demo) {
        // simulate an aircraft
        snprintf_P(NMEABuffer, sizeof(NMEABuffer),
                PSTR("$PFLAU,1,1,%d,1,%d,0,2,0,500,AAAAAA" PFLAU_EXT1_FMT "*"),
                GNSS_STATUS_3D_MOVING, (ALARM_LEVEL_IMPORTANT - 1)
                PFLAU_EXT1_ARGS );
    } else {
        snprintf_P(NMEABuffer, sizeof(NMEABuffer),
                PSTR("$PFLAU,0,%d,%d,%d,%d,,0,,," PFLAU_EXT1_FMT "*"),  // had extra comma
                tx_status, gps_status,
                power_status, ALARM_LEVEL_NONE
                PFLAU_EXT1_ARGS );
    }

    NMEAOutC(NMEA_T);

//#if defined(USE_SD_CARD)
#if 0
if (settings->debug_flags) {
    nmealen -= 2;   // overwrite existing \r\n
    snprintf_P(NMEABuffer+nmealen, sizeof(NMEABuffer)-nmealen, " at %02d:%02d:%02d\r\n",
         gnss.time.hour(), gnss.time.minute(), gnss.time.second());
    Serial.print(NMEABuffer);
    SD_log(NMEABuffer);
}
#endif
}

#if defined(USE_NMEALIB)

void NMEA_Position()    // only called in txrx_test() mode (and maybe from RPi)
{
  NmeaInfo info;
  size_t i;
  struct timeval tv;

  if (settings->nmea_g || settings->nmea2_g) {

    nmeaInfoClear(&info);

    info.sig = NMEALIB_SIG_SENSITIVE;
    info.fix = NMEALIB_FIX_3D;

    tv.tv_sec  = ThisAircraft.timestamp;
    tv.tv_usec = 0;

    nmeaTimeSet(&info.utc, &info.present, &tv);

    info.latitude = ((int) ThisAircraft.latitude) * 100.0;
    info.latitude += (ThisAircraft.latitude - (int) ThisAircraft.latitude) * 60.0;
    info.longitude = ((int) ThisAircraft.longitude) * 100.0;
    info.longitude += (ThisAircraft.longitude - (int) ThisAircraft.longitude) * 60.0;
    info.speed = ThisAircraft.speed * _GPS_KMPH_PER_KNOT;
    info.height = ThisAircraft.geoid_separation;
    info.elevation = ThisAircraft.altitude - info.height; /* above MSL */
    info.track = ThisAircraft.course;

#if 0
    info.mtrack = 55;
    info.magvar = 55;
#endif
    info.hdop = 2.3;
    info.vdop = 1.2;
    info.pdop = 2.594224354;

    nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_SIG);
    nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_FIX);
    nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_LAT);
    nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_LON);
    nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_SPEED);
    nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_ELV);
    nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_HEIGHT);

    nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_TRACK);
#if 0
    nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_MTRACK);
    nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_MAGVAR);
#endif
    nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_HDOP);
    nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_VDOP);
    nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_PDOP);

#if 0
    info.satellites.inUseCount = NMEALIB_MAX_SATELLITES;
    nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_SATINUSECOUNT);
    for (i = 0; i < NMEALIB_MAX_SATELLITES; i++) {
      info.satellites.inUse[i] = (unsigned int) (i + 1);
    }
    nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_SATINUSE);

    info.satellites.inViewCount = NMEALIB_MAX_SATELLITES;
    for (i = 0; i < NMEALIB_MAX_SATELLITES; i++) {
      info.satellites.inView[i].prn = (unsigned int) i + 1;
      info.satellites.inView[i].elevation = (int) ((i * 10) % 90);
      info.satellites.inView[i].azimuth = (unsigned int) (i + 1);
      info.satellites.inView[i].snr = 99 - (unsigned int) i;
    }
    nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_SATINVIEWCOUNT);
    nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_SATINVIEW);
#endif

    size_t gen_sz = nmeaSentenceFromInfo(&nmealib_buf, &info, (NmeaSentence)
      (NMEALIB_SENTENCE_GPGGA | NMEALIB_SENTENCE_GPRMC));
   // (NMEALIB_SENTENCE_GPGGA | NMEALIB_SENTENCE_GPGSA | NMEALIB_SENTENCE_GPRMC));

    if (gen_sz)
      NMEA_Outs(NMEA_G, nmealib_buf.buffer, gen_sz, false);
  }
}

void NMEA_GGA()
{
  //if (! (settings->nmea_g || settings->nmea2_g))
  //  return;

  NmeaInfo info;

  float latitude = gnss.location.lat();
  float longitude = gnss.location.lng();

  nmeaInfoClear(&info);

  info.utc.hour = gnss.time.hour();
  info.utc.min = gnss.time.minute();
  info.utc.sec = gnss.time.second();
  info.utc.hsec = gnss.time.centisecond();

  info.latitude = ((int) latitude) * 100.0;
  info.latitude += (latitude - (int) latitude) * 60.0;
  info.longitude = ((int) longitude) * 100.0;
  info.longitude += (longitude - (int) longitude) * 60.0;

  info.sig = (NmeaSignal) gnss.location.Quality();
  info.satellites.inViewCount = gnss.satellites.value();

  info.hdop = gnss.hdop.hdop();

  info.height = ThisAircraft.geoid_separation;   // may be from setting or EGM
  info.elevation = ThisAircraft.altitude;        // above ellipsoid
  info.elevation -= info.height;                 // above MSL

  //if (info.height == 0.0 && info.sig != (NmeaSignal) Invalid) {
  //  info.height = EGM96GeoidSeparation();
  //  info.elevation -= info.height;
  //}

  nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_UTCTIME);
  nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_LAT);
  nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_LON);
  nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_SIG);
  /* Should be SATINUSECOUNT, but it seems to be a bug in NMEALib */
  nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_SATINVIEWCOUNT);
  nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_HDOP);
  nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_ELV);
  nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_HEIGHT);

  size_t gen_sz = nmeaSentenceFromInfo(&nmealib_buf, &info, (NmeaSentence)
                                        NMEALIB_SENTENCE_GPGGA );

  if (gen_sz) {
    //strncpy(GPGGA_Copy, nmealib_buf.buffer, gen_sz);  // for traffic alarm logging
    //GPGGA_Copy[gen_sz] = '\0';
    NMEA_Outs(NMEA_G, nmealib_buf.buffer, gen_sz, false);
  }
}
#endif /* USE_NMEALIB */

#if defined(USE_NMEA_CFG)

#include "../../driver/Buzzer.h"
#include "../../driver/LED.h"
#include "GDL90.h"
#include "D1090.h"

//#if !defined(SERIAL_FLUSH)
//#define SERIAL_FLUSH()       Serial.flush()
//#endif

// this is used specifically to respond to the source
// while NMEA_Out() specifically avoids that,
// so mask the source
void nmea_cfg_reply(bool blanklines=true)
{
    unsigned int nmealen = NMEA_add_checksum();
    uint8_t dest = NMEA_Source;
    NMEA_Source = DEST_NONE;
    if (blanklines)
        NMEA_Out(dest, "\r\n", 2, false);
    NMEA_Out(dest, NMEABuffer, nmealen, blanklines);
    NMEA_Source = dest;
}

static void nmea_cfg_restart(bool save_settings)
{
  if (save_settings) {
      //EEPROM_store();
      save_settings_to_file();   // this also shows the new settings
  }
  Serial.println();
  Serial.println(F("Restart is in progress. Please, wait..."));
  Serial.println();
  SoC->WDT_fini();
  if (SoC->Bluetooth_ops) { SoC->Bluetooth_ops->fini(); }
  delay(2000);
  reboot();
}

bool isdecdigit(const char *p)
{
    if (*p >= '0' && *p <= '9')  return true;
    return false;
}

bool ishexdigit(const char *p)
{
    if (*p >= '0' && *p <= '9')  return true;
    if (*p >= 'A' && *p <= 'F')  return true;
    if (*p >= 'a' && *p <= 'f')  return true;
    return false;
}

bool cfg_is_updated;

void tryupdate(TinyGPSCustom &field, int idx)
{
    const char *p = field.value();
    if (p[0] == '\0')              // empty field
        return;
    int8_t t = stgdesc[idx].type;
    if (t == STG_VOID)
        return;
    const char *label = stgdesc[idx].label;
    const char *msg = "not changed, still";
    char *v  = stgdesc[idx].value;
    int32_t cur_val, cfg_val;
    switch (t) {
    case STG_HIDDEN:
    case STG_INT1:
    case STG_UINT1:
       cur_val = (t==STG_UINT1? (*(uint8_t*)v) : (*(int8_t*)v));
       cfg_val = cur_val;
       if (isdecdigit(p))
           cfg_val = atoi(p);
       else
           msg = "given value invalid, left as";
       if (cfg_val != cur_val) {
           msg = "changed to";
           cfg_is_updated = true;
           if (t==STG_UINT1)
               *(uint8_t *)v = (uint8_t) cfg_val;
           else
               *(int8_t *)v = (int8_t) cfg_val;
       }
       break;
    case STG_HEX2:
    case STG_HEX6:
       cur_val = (t==STG_HEX2? (*(uint8_t*)v) : (*(uint32_t*)v));
       cfg_val = cur_val;
       if (ishexdigit(p))
           cfg_val = strtol(p,NULL,16);
       else
           msg = "given value invalid, left as";
       if (cfg_val != cur_val) {
           msg = "changed to";
           cfg_is_updated = true;
           if (t==STG_HEX2)
               *(uint8_t *)v = (uint8_t) cfg_val;
           else
               *(uint32_t *)v = (uint32_t) cfg_val;
       }
       break;
    default:
       if (t > STG_VOID) {
         if (strcmp(p, (char*)v)) {
           msg = "changed to";
           cfg_is_updated = true;
           strncpy(v, p, t);
           v[t-1] = '\0';
         }
       }
       break;
    }
    if (t==STG_INT1 || t==STG_UINT1 || t==STG_HIDDEN) {
      snprintf_P(NMEABuffer, sizeof(NMEABuffer),
        PSTR("%s %s %d\r\n"), label, msg, (int) cfg_val);
    } else if (t==STG_HEX2 || t==STG_HEX6) {
      snprintf_P(NMEABuffer, sizeof(NMEABuffer),
        PSTR("%s %s %X\r\n"), label, msg, cfg_val);
    } else if (t > STG_VOID) {
      snprintf_P(NMEABuffer, sizeof(NMEABuffer),
        PSTR("%s %s %s\r\n"), label, msg, (char*)v);
    }
    nmea_cfg_reply();   // no '*' and thus checksum will not be added
}

void NMEA_Process_SRF_SKV_Sentences()
{
  if (C_Version.isUpdated()) {

      if (strncmp(C_Version.value(), "RST", 3) == 0) {             // $PSRFC,RST*2D
          Serial.println(F("PSRFC Reboot..."));
          nmea_cfg_restart(false);
          // just reboot, settings not saved

      } else if (strncmp(C_Version.value(), "SAV", 3) == 0) {      // $PSRFC,SAV*3C
          Serial.println(F("PSRFC Save & Reboot..."));
          nmea_cfg_restart(true);
          // save settings to EEPROM and reboot

      } else if (strncmp(C_Version.value(), "OFF", 3) == 0) {      // $PSRFC,OFF*37
          Serial.println(F("PSRFC Shutdown..."));
          shutdown(SOFTRF_SHUTDOWN_NMEA);

#if defined(USE_OLED)
      } else if (strncmp(C_Version.value(), "PAG", 3) == 0) {      // $PSRFC,PAG*2E
          Serial.println(F("PSRFC Page Switch"));
          OLED_Next_Page();
#elif defined(USE_EPAPER)
      } else if (strncmp(C_Version.value(), "PAG", 3) == 0) {      // $PSRFC,PAG*2E
          Serial.println(F("PSRFC Page Switch"));
          EPD_Mode(false);
#endif
      } else if (strncmp(C_Version.value(), "DEM", 3) == 0) {      // $PSRFC,DEM*34
          Serial.println(F("PSRFC Alarm Demo"));
          SetupTimeMarker = millis();
          do_alarm_demo = true;
#if defined(USE_OLED)
          OLED_msg("ALARM", " DEMO");
#endif

      } else if (strncmp(C_Version.value(), "TX0", 3) == 0) {      // $PSRFC,TX0*44
          Serial.println(F("PSRFC TX Off"));
          settings->txpower = RF_TX_POWER_OFF;

      } else if (strncmp(C_Version.value(), "TX1", 3) == 0) {      // $PSRFC,TX1*45
          Serial.println(F("PSRFC TX On"));
          settings->txpower = RF_TX_POWER_FULL;

      } else if (strncmp(C_Version.value(), "?", 1) == 0) {        // $PSRFC,?*47

          snprintf_P(NMEABuffer, sizeof(NMEABuffer),
            PSTR("$PSRFC,%d,%d,%d,%d,%d,%d,%d,%d,%d,%X,%X,%X,%X,%d,%d,%d,%d,%d,%d*"),
            PSRFX_VERSION,      settings->mode,      settings->rf_protocol,
            settings->band,     settings->acft_type, settings->alarm,
            settings->txpower,  settings->volume,    settings->pointer,
            settings->nmea_g,   settings->nmea_p,    settings->nmea_t,
            settings->nmea_s,   settings->nmea_out,  settings->gdl90,
            settings->d1090,    settings->stealth,   settings->no_track,
            settings->power_save );

          nmea_cfg_reply();

      } else if (isdecdigit(C_Version.value())) {

          cfg_is_updated = false;

          tryupdate(C_Mode, STG_MODE);
          tryupdate(C_Protocol, STG_PROTOCOL);
          tryupdate(C_Band, STG_BAND);
          tryupdate(C_AcftType, STG_ACFT_TYPE);
          tryupdate(C_Alarm, STG_ALARM);
          tryupdate(C_TxPower, STG_TXPOWER);
          tryupdate(C_Volume, STG_VOLUME);
          tryupdate(C_Pointer, STG_POINTER);
          tryupdate(C_NMEA_gnss, STG_NMEA_G);
          tryupdate(C_NMEA_private, STG_NMEA_P);
          tryupdate(C_NMEA_traffic, STG_NMEA_T);
          tryupdate(C_NMEA_sensors, STG_NMEA_S);
          tryupdate(C_NMEA_Output, STG_NMEA_OUT);
          tryupdate(C_GDL90_Output, STG_GDL90);
          tryupdate(C_D1090_Output, STG_D1090);
          tryupdate(C_Stealth, STG_STEALTH);
          tryupdate(C_noTrack, STG_NO_TRACK);
          tryupdate(C_PowerSave, STG_POWER_SAVE);

          if (cfg_is_updated && atoi(C_Version.value()))
              nmea_cfg_restart(true);
          else
              NMEA_encode("$PSRFC,0,,,,,,,,,,,,,,,,,,*48\r\n", 31);
              // fill in the structure inside TinyGPS with empty strings for all fields
      }
  }

  if (D_Version.isUpdated()) {

      if (strncmp(D_Version.value(), "?", 1) == 0) {

          //char psrfd_buf[MAX_PSRFD_LEN];
          snprintf_P(NMEABuffer, sizeof(NMEABuffer),
            PSTR("$PSRFD,%d,%d,%06X,%06X,%06X,%d,%d,%X,%02X,%d,%X,%X,%X,%X,%X,%d,%d,%d,%d,%X,%X,%d,%d,%d*"),
            PSRFX_VERSION,         settings->id_method,  settings->aircraft_id,
            settings->ignore_id,   settings->follow_id,  settings->baud_rate,
            settings->power_ext,   settings->nmea_d,     settings->debug_flags,
            settings->nmea_out2,   settings->nmea2_g,    settings->nmea2_p,
            settings->nmea2_t,     settings->nmea2_s,    settings->nmea2_d,
            settings->relay,       settings->bluetooth,  settings->baudrate2,
            settings->invert2,     settings->nmea_e,     settings->nmea2_e,
            settings->altpin0,     settings->voice,      settings->strobe);

          nmea_cfg_reply();

      } else if (isdecdigit(D_Version.value())) {

          cfg_is_updated = false;

          tryupdate(D_baud_rate, STG_BAUD_RATE);
          tryupdate(D_power_ext, STG_POWER_EXT);
          tryupdate(D_NMEA_debug, STG_NMEA_D);
          tryupdate(D_debug_flags, STG_DEBUG_FLAGS);
          tryupdate(D_NMEA2, STG_NMEA_OUT2);
          if (settings->nmea_out2 == settings->nmea_out)
              settings->nmea_out2 = DEST_NONE;
          tryupdate(D_NMEA2_gnss, STG_NMEA_G);
          tryupdate(D_NMEA2_private, STG_NMEA_P);
          tryupdate(D_NMEA2_legacy, STG_NMEA_T);
          tryupdate(D_NMEA2_sensors, STG_NMEA_S);
          tryupdate(D_NMEA2_debug, STG_NMEA_D);
          tryupdate(D_relay, STG_RELAY);
          tryupdate(D_bluetooth, STG_BLUETOOTH);
#if defined(ESP32)
          tryupdate(D_baudrate2, STG_BAUDRATE2);
          tryupdate(D_invert2, STG_INVERT2);
#endif
          tryupdate(D_extern1, STG_NMEA_E);
          tryupdate(D_extern2, STG_NMEA2_E);
#if defined(ESP32)
          tryupdate(D_altpin0, STG_ALTPIN0);
#endif
          tryupdate(D_voice, STG_VOICE);
          tryupdate(D_strobe, STG_STROBE);
          tryupdate(D_id_method, STG_ID_METHOD);
          tryupdate(D_aircraft_id, STG_AIRCRAFT_ID);
          tryupdate(D_ignore_id, STG_IGNORE_ID);
          tryupdate(D_follow_id, STG_FOLLOW_ID);

          if (cfg_is_updated && atoi(D_Version.value()))
              nmea_cfg_restart(true);
          else
              NMEA_encode("$PSRFD,0,,,,,,,,,,,,,,,,,,,,,,,*47\r\n", 36);
      }
  }

  if (F_Version.isUpdated()) {

      if (strncmp(F_Version.value(), "?", 1) == 0) {           // $PSRFF,?*42
        //char psrff_buf[MAX_PSRFF_LEN];
        snprintf_P(NMEABuffer, sizeof(NMEABuffer),
          PSTR("$PSRFF,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d*"),
          PSRFX_VERSION,         settings->rx1090,      settings->rx1090x,
          settings->mode_s,      settings->gdl90_in,    settings->gnss_pins,
          settings->ppswire,     settings->logalarms,   settings->sd_card,
          settings->logflight,   settings->loginterval, settings->alt_udp,
          settings->tcpmode,     settings->tcpport,     settings->geoid,
          settings->freq_corr);
        nmea_cfg_reply();

      } else if (isdecdigit(F_Version.value())) {

          cfg_is_updated = false;

          tryupdate(F_gdl90_in, STG_GDL90_IN);
          tryupdate(F_alt_udp, STG_ALT_UDP);
          tryupdate(F_tcpmode, STG_TCPMODE);
          tryupdate(F_tcpport, STG_TCPPORT);
          tryupdate(F_geoid, STG_GEOID);
          tryupdate(F_freq_corr, STG_RFC);
          tryupdate(F_logflight, STG_LOGFLIGHT);
          tryupdate(F_loginterval, STG_LOGINTERVAL);
          tryupdate(F_logalarms, STG_ALARMLOG);
#if defined(ESP32)
          tryupdate(F_rx1090, STG_RX1090);
          tryupdate(F_rx1090x, STG_RX1090X);
          tryupdate(F_mode_s, STG_MODE_S);
          tryupdate(F_gnss_pins, STG_GNSS_PINS);
          tryupdate(F_ppswire, STG_PPSWIRE);
          tryupdate(F_sd_card, STG_SD_CARD);
#endif
          if (cfg_is_updated && atoi(F_Version.value()))
              nmea_cfg_restart(true);
          else
              NMEA_encode("$PSRFF,0,,,,,,,,,,,,,,,*61\r\n", 28);
      }
  }

  // one setting at a time
  if (S_Version.isUpdated()) {

    char version0 = *S_Version.value();
    char label0 = 0xFF;
    if (S_label.isUpdated())
        label0 = *S_label.value();

    if (version0 == '?' || label0 == '?') {   // treat $PSRFS,0,?*xx same as $PSRFS,?*xx
      // reply in the same format as the settings file, including comments
      for (int i=STG_MODE; i<STG_END; i++) {
         if (format_setting(i, true) == false)
               continue;
         nmea_cfg_reply(false);  // do not add blank lines between the settings
         yield();
      }

    } else if (isdecdigit(&version0)) {

      int i = STG_NONE;
      if (label0 != 0xFF && label0 != '\0')
          i = find_setting(S_label.value());
      if (i == STG_NONE) {
        snprintf_P(NMEABuffer, sizeof(NMEABuffer),
            PSTR("%s - no matching label\r\n"), S_label.value());
        nmea_cfg_reply();

      } else {    // label matches a setting

        bool query = true;      // treat $PSRFS,0,label*xx same as $PSRFS,0,label,?*xx
        cfg_is_updated = true;  // for non-UINT1 settings assume it's a change
        bool loaded = false;
        if (S_value.isUpdated())
            query = (*S_value.value() == '?');
        if (! query) {
            if (stgdesc[i].type == STG_UINT1)
                cfg_is_updated = (*((uint8_t *)(stgdesc[i].value)) != atoi(S_value.value()));
            else if (stgdesc[i].type == STG_INT1 || stgdesc[i].type == STG_HIDDEN)
                cfg_is_updated = (*((int8_t *)(stgdesc[i].value)) != atoi(S_value.value()));
            if (cfg_is_updated)
                loaded = load_setting(i, S_value.value());
        }

        if (!query && cfg_is_updated && !loaded) {
          snprintf_P(NMEABuffer, sizeof(NMEABuffer),
                  PSTR("%s,%s - error\r\n"), S_label.value(), S_value.value());
          nmea_cfg_reply();

        } else {   // if (query || loaded)

          // reply with a complete $PSRFS sentence (and no setting comment)
          if (format_setting(i, false) == true) {
              char buf[sizeof(NMEABuffer)+9];
              strcpy(buf, "$PSRFS,");
              if (loaded)
                  buf[7] = version0;      // digit given after $PSRFS,
              else
                  buf[7] = '0';           // signals no change
              buf[8] = ',';
              strcpy(buf+9, NMEABuffer);  // format_setting() put its output in NMEABuffer
              int len = strlen(buf);
              buf[len-2] = '*';       // overwrite the \r\n from format_setting()
              buf[len-1] = '\0';
              strcpy(NMEABuffer, buf);
              nmea_cfg_reply();           // will output what's now in NMEABuffer
          }

          // only restart if Version field is nonzero
          if ( /* loaded && */ version0 != '0')
              nmea_cfg_restart(true);
        }
      }
    }
  }

  // $PSRFT,0*5F  or  $PSRFT,1*5E  to set test_mode to 0 or 1, or $PSRFT,?*50 to query
  // Also reports the chip ID, which is not settable, transmitted if ID type is "device"
  if (T_testmode.isUpdated()) {

      char tval = T_testmode.value()[0];
      if (tval == '?') {
          snprintf_P(NMEABuffer, sizeof(NMEABuffer), PSTR("$PSRFT,%d,%06X*"),
                test_mode, (SoC->getChipId() & 0x00FFFFFF));
          nmea_cfg_reply();
          // put custom code here for debugging, for example:
#if defined(ESP32)
          if (settings->rx1090)
              show_zone_stats();
#endif
      } else {
          if (tval == '0')
              test_mode = false;
          else if (tval == '1')
              test_mode = true;
          else
              test_mode = !test_mode;
          if (test_mode) {
#if defined(ESP32)
              OLED_msg("TEST",   " MODE");
#endif
              Serial.println("Test Mode on");
          } else {
#if defined(ESP32)
              OLED_msg("NOTEST", " MODE");
#endif
              Serial.println("Test Mode off");
          }
          do_test_mode();
      }
  }

#if defined(USE_OGN_ENCRYPTION)
  if (K_Version.isUpdated()) {

      if (strncmp(K_Version.value(), "?", 1) == 0) {

        snprintf_P(NMEABuffer, sizeof(NMEABuffer),
            PSTR("$PSRFK,%d,%08X%08X%08X%08X*"),
            PSRFX_VERSION,
            settings->igc_key[0]? 0x88888888 : 0,
            settings->igc_key[1]? 0x88888888 : 0,
            settings->igc_key[2]? 0x88888888 : 0,
            settings->igc_key[3]? 0x88888888 : 0);
            /* mask the key from prying eyes */

        nmea_cfg_reply();

      } else if (isdecdigit(K_Version.value())) {

          char buf[32 + 1];

          strncpy(buf, K_IGC_Key.value(), sizeof(buf));

          settings->igc_key[3] = strtoul(buf + 24, NULL, 16);
          buf[24] = 0;
          settings->igc_key[2] = strtoul(buf + 16, NULL, 16);
          buf[16] = 0;
          settings->igc_key[1] = strtoul(buf +  8, NULL, 16);
          buf[ 8] = 0;
          settings->igc_key[0] = strtoul(buf +  0, NULL, 16);

          snprintf_P(buf, sizeof(buf),
            PSTR("%08X%08X%08X%08X"),
            settings->igc_key[0], settings->igc_key[1],
            settings->igc_key[2], settings->igc_key[3]);

          Serial.print(F("IGC Key = ")); Serial.println(buf);

          if (atoi(K_Version.value()))
              nmea_cfg_restart(true);
          else
              NMEA_encode("$PSRFK,0,*74\r\n", 14);
      }
  }
#endif /* USE_OGN_ENCRYPTION */

#if defined(USE_SKYVIEW_CFG)
  if (V_Version.isUpdated()) {

      if (strncmp(V_Version.value(), "?", 1) == 0) {

#if 1 // defined(ABANDON_EEPROM)
          snprintf_P(NMEABuffer, sizeof(NMEABuffer),
            PSTR("$PSKVC,%d,%d,%d,%d,%d,%d,%d,%d,%d,%X*"),
            PSKVC_VERSION,       settings->units,       settings->zoom,
            settings->rotate,    settings->orientation, settings->adb,
            settings->epdidpref, settings->viewmode,    settings->antighost,
            settings->team);
#else
          snprintf_P(NMEABuffer, sizeof(NMEABuffer),
            PSTR("$PSKVC,%d,%d,%d,%d,%d,%d,%d,%s,%s,%d,%d,%d,%d,%d,%d,%d,%d,%d,%X*"),
            PSKVC_VERSION,  ui->adapter,      ui->connection,
            ui->units,      ui->zoom,         ui->protocol,
            ui->baudrate,   ui->server,       ui->key,
            ui->rotate,     ui->orientation,  ui->adb,
            ui->epdidpref,  ui->viewmode,     ui->voice,
            ui->antighost,  ui->filter,       ui->power_save,
            ui->team);
#endif
          nmea_cfg_reply();

#if 1  // defined(ABANDON_EEPROM)

      } else if (isdecdigit(V_Version.value())) {

          cfg_is_updated = false;

          tryupdate(V_Units, STG_EPD_UNITS);
          tryupdate(V_Zoom, STG_EPD_ZOOM);
          tryupdate(V_Rotate, STG_EPD_ROTATE);
          tryupdate(V_Orientation, STG_EPD_ORIENT);
          tryupdate(V_AvDB, STG_EPD_ADB);
          tryupdate(V_ID_Pref, STG_EPD_IDPREF);
          tryupdate(V_VMode, STG_EPD_VMODE);
          tryupdate(V_AntiGhost, STG_EPD_AGHOST);
          tryupdate(V_Team, STG_EPD_TEAM);
          if (cfg_is_updated && atoi(V_Version.value()))
              nmea_cfg_restart(true);
          else
              NMEA_encode("$PSKVC,0,,,,,,,,,*41\r\n", 22);
#endif
      }
  }
#endif /* USE_SKYVIEW_CFG */
}
#endif /* USE_NMEA_CFG */

static char hexbuf[NMEA_BUFFER_SIZE];
char *bytes2Hex(byte *buffer, size_t size)
{
  char *p = hexbuf;
  for (int i=0; i < size && i < NMEA_BUFFER_SIZE/2-1; i++) {
    byte c = buffer[i];
    byte cl = c & 0x0F;
    byte cu = (c >> 4) & 0x0F;
    *p++ = (cu < 0x0A)? '0'+cu : 'A'+cu-0x0A;
    *p++ = (cl < 0x0A)? '0'+cl : 'A'+cl-0x0A;
  }
  *p = '\0';
  return hexbuf;
}
