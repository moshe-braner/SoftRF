/*
 * GNSSHelper.cpp
 * Copyright (C) 2016-2022 Linar Yusupov
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

// this file based on v1.2.

#if defined(ARDUINO)
#include <Arduino.h>
#endif
#include <TimeLib.h>

#include "../system/SoC.h"
#include "../TrafficHelper.h"
#include "GNSS.h"
#include "Settings.h"
#include "Filesys.h"
#include "../protocol/data/NMEA.h"
#include "../system/Time.h"
#include "WiFi.h"
#include "RF.h"
#include "Battery.h"
#include "../protocol/data/D1090.h"

#if defined(USE_EGM96)
//#include <egm96s.h>
#if defined(FILESYS)
//#include "SPIFFS.h"
#endif
static float geo_sep_from_file = 0.0;
#endif

//#define DO_GNSS_DEBUG

#if !defined(DO_GNSS_DEBUG)
#define GNSS_DEBUG_PRINT
#define GNSS_DEBUG_PRINTLN
#else
#define GNSS_DEBUG_PRINT    Serial.print
#define GNSS_DEBUG_PRINTLN  Serial.println
#endif

#if !defined(GNSS_FLUSH)
#define GNSS_FLUSH()        Serial_GNSS_Out.flush()
#endif

// since last PPS:
bool gnss_new_fix  = false;
bool gnss_new_time = false;
bool gnss_time_from_rmc = false;
uint32_t latest_Commit_Time = 0;   // millis() at first Time-commit after last PPS

bool gnss_needs_reset = false;

static bool is_prime_mk2 = false;
gnss_id_t gnss_id = GNSS_MODULE_NONE;

uint32_t GNSSTimeSyncMarker = 0;
volatile unsigned long PPS_TimeMarker = 0;

#if defined(ESP32)
static uint8_t get_pps_pin()
{
    if (hw_info.model == SOFTRF_MODEL_PRIME_MK3)
        return SOC_GPIO_PIN_S3_GNSS_PPS;
    if (hw_info.model == SOFTRF_MODEL_PRIME_MK2) {
        if (settings->gnss_pins == EXT_GNSS_NONE) {
            if (hw_info.revision >= 8 || settings->ppswire)
                return SOC_GPIO_PIN_TBEAM_V08_PPS;
        } else if (settings->ppswire) {
            if (hw_info.revision < 8) {
                if (settings->gnss_pins == EXT_GNSS_39_4)
                    return SOC_GPIO_PIN_VOICE;   // pin 25 for PPS - VP is used for rx
                if (settings->baudrate2 != BAUD_DEFAULT) {
                    // if VP is used for aux serial, use pin 25 for PPS if possible
                    if (settings->gnss_pins == EXT_GNSS_13_2)
                        return SOC_GPIO_PIN_VOICE;
                    return SOC_UNUSED_PIN;
                }
            } else {   // hw_info.revision >= 8
                if (settings->sd_card == SD_CARD_13_VP) {
                    if (settings->voice == VOICE_OFF && settings->strobe == STROBE_OFF)
                        return SOC_GPIO_PIN_VOICE;   // pin 25 for PPS
                    return SOC_UNUSED_PIN;
                }
            }
            return Serial0AltRxPin;     // "VP" pin used for external GNSS PPS
        }
    }
    return SOC_UNUSED_PIN;
}
#else
static uint8_t get_pps_pin() { return SOC_GPIO_PIN_GNSS_PPS; }
#endif

const gnss_chip_ops_t *gnss_chip = NULL;
extern const gnss_chip_ops_t goke_ops; /* forward declaration */

boolean gnss_set_sucess = false ;
TinyGPSPlus gnss;  // Create an Instance of the TinyGPS++ object called gnss

uint8_t GNSSbuf[250]; // at least 3 lines of 80 characters each
                      // and 40+30*N bytes for "UBX-MON-VER" payload

int GNSS_cnt           = 0;
uint16_t FW_Build_Year = 2000 + ((__DATE__[ 9]) - '0') * 10 + ((__DATE__[10]) - '0');

const char *GNSS_name[] = {
  [GNSS_MODULE_NONE]    = "NONE",
  [GNSS_MODULE_NMEA]    = "NMEA",
  [GNSS_MODULE_U6]      = "U6",
  [GNSS_MODULE_U7]      = "U7",
  [GNSS_MODULE_U8]      = "U8",
  [GNSS_MODULE_U9]      = "U9",
  [GNSS_MODULE_U10]     = "U10",
  [GNSS_MODULE_U11]     = "U11",
  [GNSS_MODULE_MAV]     = "MAV",
  [GNSS_MODULE_SONY]    = "SONY",
  [GNSS_MODULE_AT65]    = "AT65",
  [GNSS_MODULE_MT33]    = "MT33",
  [GNSS_MODULE_GOKE]    = "GOKE"
};

#if defined(ENABLE_GNSS_STATS)
/*
 * Stats collected by Linar:
 * Sony: GGA -  24 , RMC -  38
 * L76K: GGA -  70+, RMC - 135+    // == AT65
 * Goke: GGA - 185+, RMC - 265+
 * Neo6: GGA - 138 , RMC -  67
 * MT33: GGA -  48 , RMC - 175
 */

gnss_stat_t gnss_stats;
#endif /* ENABLE_GNSS_STATS */

bool nmea_handshake(const char *req, const char *resp, bool skipline)
{
  bool rval = false;

  if (resp == NULL || strlen(resp) == 0) {
    return rval;
  }

  // clean any leftovers
  Serial_GNSS_In.flush();

  while (Serial_GNSS_In.available() > 0) { Serial_GNSS_In.read(); }

  unsigned long start_time = millis();
  unsigned long timeout_ms = (req == NULL ? 3000 : 2000) ;

  while ((millis() - start_time) < timeout_ms) {

    while (Serial_GNSS_In.read() != '\n' && (millis() - start_time) < timeout_ms) { yield(); }

    delay(5);

    /* wait for pause after NMEA burst */
    if (req && Serial_GNSS_In.available() > 0) {
      continue;
    } else {
      /* send request */
      if (req) {
        Serial_GNSS_Out.write((uint8_t *) req, strlen(req));
        GNSS_FLUSH();
      }

      /* skip first line when expected response contains 2 of them */
      if (skipline) {
        start_time = millis();
        while (Serial_GNSS_In.read() != '\n' && (millis() - start_time) < timeout_ms) { yield(); }
      }

      int i=0;
      char c;

      /* take response into buffer */
      while ((millis() - start_time) < timeout_ms) {

        c = Serial_GNSS_In.read();

        if (isPrintable(c) || c == '\r' || c == '\n') {
          if (i >= sizeof(GNSSbuf)) break;
          GNSSbuf[i++] = c;
        } else {
          /* ignore */
          continue;
        }

        if (c == '\n') break;
      }

      if (!strncmp((char *) &GNSSbuf[0], resp, strlen(resp))) {
        rval = true;
        break;
      }
    }
  }

  return rval;
}

static gnss_id_t generic_nmea_probe()
{
  return nmea_handshake(NULL, "$G", false) ? GNSS_MODULE_NMEA : GNSS_MODULE_NONE;
}

static bool generic_nmea_setup()
{
  return true;
}

static void generic_nmea_loop()
{

}

static void generic_nmea_fini()
{

}

const gnss_chip_ops_t generic_nmea_ops = {
  generic_nmea_probe,
  generic_nmea_setup,
  generic_nmea_loop,
  generic_nmea_fini,
  /* use Ublox timing values for 'generic NMEA' module */
  138 /* GGA */, 67 /* RMC */
};

#if !defined(EXCLUDE_GNSS_UBLOX)

 /* CFG-MSG */
// by default enable: RMC & GGA only
// for Stratux and SkyDemon enable: RMC, GGA, GSA, GST, GSV; disable: GLL, VTG
const uint8_t enaRMC[] PROGMEM = {0xF0, 0x04, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01}; // enable RMC
const uint8_t enaGGA[] PROGMEM = {0xF0, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01}; // enable GGA
const uint8_t enaGSA[] PROGMEM = {0xF0, 0x02, 0x00, 0x05, 0x00, 0x00, 0x00, 0x01}; // enable GSA
const uint8_t disGSA[] PROGMEM = {0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01}; // disable GSA
const uint8_t enaGST[] PROGMEM = {0xF0, 0x07, 0x00, 0x05, 0x00, 0x00, 0x00, 0x01}; // enable GST
const uint8_t disGST[] PROGMEM = {0xF0, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01}; // disable GST
const uint8_t enaGSV[] PROGMEM = {0xF0, 0x03, 0x00, 0x05, 0x00, 0x00, 0x00, 0x01}; // enable GSV
const uint8_t disGSV[] PROGMEM = {0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01}; // disable GSV
const uint8_t disVTG[] PROGMEM = {0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01}; // disable VTG
const uint8_t disGLL[] PROGMEM = {0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01}; // disable GLL

/* Stratux Setup: enable GPS & Galileo & Glonass for u-blox 8 */
/* error on u-blox 6 can be ignored */
const uint8_t setGNSS_U8[] PROGMEM = {0x00, 0x00, 0xFF, 0x07,
                                      0x00, 0x08, 0x10, 0x00, 0x01, 0x00, 0x01, 0x01,  /* GPS */
                                      0x01, 0x01, 0x03, 0x00, 0x01, 0x00, 0x01, 0x01,  /* SBAS */
                                      0x02, 0x08, 0x08, 0x00, 0x01, 0x00, 0x01, 0x01,  /* Galileo */
                                      0x04, 0x00, 0x08, 0x00, 0x00, 0x00, 0x01, 0x01,  /* IMES */
                                      0x03, 0x08, 0x10, 0x00, 0x00, 0x00, 0x01, 0x01,  /* Beidou */
                                      0x05, 0x01, 0x03, 0x00, 0x01, 0x00, 0x01, 0x01,  /* QZSS */
                                      0x06, 0x08, 0x10, 0x00, 0x01, 0x00, 0x01, 0x01}; /* Glonass */

/* Stratux Setup: set NMEA protocol version and numbering for u-blox 8: NMEA protocol v4.0 extended */
/* error on u-blox 6 can be ignored */
const uint8_t setNMEA_U8[] PROGMEM = {0x00, 0x40, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00,
                                      0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00,
                                      0x00, 0x00, 0x00, 0x00};

 /* Stratux Setup: configure SBAS  for u-blox 6 & 8 - disable integrity, enable auto-scan */
const uint8_t setSBAS[] PROGMEM = {0x01, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00};

 /* CFG-PRT */
uint8_t setBR[] = {0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0x96,
                   0x00, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00};

//const setBR9600[] = 
//B5 62 06 00 14 00 01 00 00 00 10 00 00 00 80 25 00 00 03 00 02 00 00 00 00 00 D5 17
//B5 62 06 09 0D 00 58 2D 77 05 03 00 00 00 68 2D 77 05 07 38 E9

const uint8_t setNav5[] PROGMEM = {0xFF, 0xFF, 0x07, 0x03, 0x00, 0x00, 0x00, 0x00,
                                   0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00,
                                   0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00,
                                   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                   0x00, 0x00, 0x00, 0x00};

 /* CFG-RST */
const uint8_t CFG_RST[12] PROGMEM = { 0xb5, 0x62, 0x06, 0x04,
                                      0x04, 0x00,     // payload is 4 bytes
                                      0x00, 0x00,     // navBbrMask (hot start)
                                      0x01,           // resetMode   
                                      0x00, 0x0F, 0x66};

 /* CFG-RST */
//const uint8_t CFG_RST_COLD[12] PROGMEM = { 0xB5, 0x62, 0x06, 0x04,
//                                           0x04, 0x00,     // payload is 4 bytes
//                                           0xFF, 0xB9,     // navBbrMask
//                                           0x00,           // resetMode 0x00 = Hardware reset
//                                           0x00, 0xC6, 0x8B };

const uint8_t CFG_RST_COLD[] PROGMEM = { 0xFF, 0xB9, 0x00, 0x00 };
const uint8_t CFG_RST_WARM[] PROGMEM = { 0x01, 0x00, 0x00, 0x00 };
const uint8_t CFG_RST_HOT[]  PROGMEM = { 0x00, 0x00, 0x00, 0x00 };

const uint8_t RXM_PMREQ_OFF[16] PROGMEM = {0xb5, 0x62, 0x02, 0x41, 0x08, 0x00,
                                           0x00, 0x00, 0x00, 0x00, 0x02, 0x00,
                                           0x00, 0x00, 0x4d, 0x3b};
 /* CFG-CFG */
//const uint8_t factoryUBX[] PROGMEM = { 0xB5, 0x62, 0x06, 0x09,
//                                       0x0D, 0x00,              // payload is 13 bytes
//                                       0xFF, 0xFB, 0x00, 0x00,  // clearMask
//                                       0x00, 0x00, 0x00, 0x00,  // saveMask
//                                       0xFF, 0xFF, 0x00, 0x00,  // loadMask
//                                       0x17,                    // deviceMask
//                                       0x2B, 0x7E } ;
// 
// adapted code from github.com/Xinyuan-LilyGO/LilyGo-LoRa-Series/tree/master/examples/GPS/UBlox_Recovery
//
//const uint8_t cfg_clear2[] PROGMEM = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0xFF, 0xFF, 0x00, 0x00,
//                                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x1C, 0xA2};
const uint8_t cfg_clear2[] PROGMEM =
  { 0xFF, 0xFB, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02 };
//{ 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02 };
//const uint8_t cfg_clear4[] PROGMEM = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0xFF, 0xFF, 0x00, 0x00,
//                                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x1B, 0xA1};
const uint8_t cfg_clear4[] PROGMEM =
  { 0xFF, 0xFB, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04 };
//{ 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04 };
//const uint8_t cfg_clear1[] PROGMEM = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0xFF, 0xFF, 0x00, 0x00,
//                                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x1B, 0xA1};
const uint8_t cfg_clear1[] PROGMEM =
  { 0xFF, 0xFB, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01 };
//{ 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01 };
//const uint8_t cfg_load3[]  PROGMEM = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00,
//                                  0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x07, 0x21, 0xB7};
const uint8_t cfg_load3[]  PROGMEM =
 { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x07 };

#if defined(USE_GNSS_PSM)
static bool gnss_psm_active = false;

/* Max Performance Mode (default) */
const uint8_t RXM_MAXP[] PROGMEM = {0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x00, 0x21, 0x91};

/* Power Save Mode */
const uint8_t RXM_PSM[] PROGMEM  = {0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x01, 0x22, 0x92};
#endif /* USE_GNSS_PSM */

uint8_t makeUBXCFG(uint8_t cl, uint8_t id, uint8_t msglen, const uint8_t *msg)
{
  if (msglen > (sizeof(GNSSbuf) - 8) ) {
    msglen = sizeof(GNSSbuf) - 8;
  }

  // Construct the UBX packet
  GNSSbuf[0] = 0xB5;   // header
  GNSSbuf[1] = 0x62;   // header
  GNSSbuf[2] = cl;  // class
  GNSSbuf[3] = id;     // id
  GNSSbuf[4] = msglen; // length
  GNSSbuf[5] = 0x00;

  GNSSbuf[6+msglen] = 0x00; // CK_A
  GNSSbuf[7+msglen] = 0x00; // CK_B

  for (int i = 2; i < 6; i++) {
    GNSSbuf[6+msglen] += GNSSbuf[i];
    GNSSbuf[7+msglen] += GNSSbuf[6+msglen];
  }

  for (int i = 0; i < msglen; i++) {
    GNSSbuf[6+i] = pgm_read_byte(&msg[i]);
    GNSSbuf[6+msglen] += GNSSbuf[6+i];
    GNSSbuf[7+msglen] += GNSSbuf[6+msglen];
  }
  GNSS_DEBUG_PRINT("prepared:");
  for (int i = 0; i < msglen+8; i++) {
    GNSS_DEBUG_PRINT(GNSSbuf[i], HEX);
    GNSS_DEBUG_PRINT(",");
  }
  GNSS_DEBUG_PRINTLN("");
  return (msglen + 8);
}

// Send a byte array of UBX protocol to the GPS
static void sendUBX(const uint8_t *MSG, uint8_t len) {
  for (int i = 0; i < len; i++) {
    Serial_GNSS_Out.write( MSG[i]);
    GNSS_DEBUG_PRINT(MSG[i], HEX);
    GNSS_DEBUG_PRINT(",");
  }
  GNSS_DEBUG_PRINTLN("");
//  Serial_GNSS_Out.println();
}

// Calculate expected UBX ACK packet and parse UBX response from GPS
static boolean getUBX_ACK(uint8_t cl, uint8_t id) {
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[2] = {cl, id};
  unsigned long startTime = millis();
  GNSS_DEBUG_PRINT(F(" * Reading ACK response: "));

  // Construct the expected ACK packet
  makeUBXCFG(0x05, 0x01, 2, ackPacket);

  while (1) {

    // Test for success
    if (ackByteID > 9) {
      // All packets in order!
      GNSS_DEBUG_PRINTLN(F(" (SUCCESS!)"));
if (settings->debug_flags) {
Serial.print(F("getUBX_ACK() got response after ms: "));
Serial.println(millis() - startTime);
}
      return true;
    }

    // Timeout if no valid response in 2 seconds
    if (millis() - startTime > 4000) {       // 2000 is too short!
      GNSS_DEBUG_PRINTLN(F(" (FAILED!)"));
      return false;
    }

    // Make sure data is available to read
    if (Serial_GNSS_In.available()) {
      b = Serial_GNSS_In.read();
      GNSS_DEBUG_PRINT(b, HEX);
      if (b == 0xA)
          GNSS_DEBUG_PRINTLN("");
      else
          GNSS_DEBUG_PRINT(",");

      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == GNSSbuf[ackByteID]) {
        ackByteID++;
        //GNSS_DEBUG_PRINT(b, HEX);
      }
      else {
        ackByteID = 0;  // Reset and look again, invalid order
      }
    }
    yield();
  }
}

static void setup_UBX()
{
  uint8_t msglen;

#if 0
  unsigned int baudrate = 38400;

  setBR[ 8] = (baudrate      ) & 0xFF;
  setBR[ 9] = (baudrate >>  8) & 0xFF;
  setBR[10] = (baudrate >> 16) & 0xFF;

  SoC->swSer_begin(9600);

  Serial.print(F("Switching baud rate onto "));
  Serial.println(baudrate);

  msglen = makeUBXCFG(0x06, 0x00, sizeof(setBR), setBR);
  sendUBX(GNSSbuf, msglen);
  gnss_set_sucess = getUBX_ACK(0x06, 0x00);

  if (!gnss_set_sucess) {
    Serial.print(F("WARNING: Unable to set baud rate onto "));
    Serial.println(baudrate); 
  }
  Serial_GNSS_In.flush();
  SoC->swSer_begin(baudrate);
#endif

  if (gnss_id == GNSS_MODULE_U8) {

    GNSS_DEBUG_PRINTLN(F("Setting GNSS configuration"));

    msglen = makeUBXCFG(0x06, 0x3E, sizeof(setGNSS_U8), setGNSS_U8);
    sendUBX(GNSSbuf, msglen);
    gnss_set_sucess = getUBX_ACK(0x06, 0x3E);
    if (!gnss_set_sucess) {
      //GNSS_DEBUG_PRINTLN(F("WARNING: Unable to set GNSS configuration"));
      Serial.println(F("WARNING: Unable to set GNSS configuration"));
    }

    GNSS_DEBUG_PRINTLN(F("Setting NMEA version 4.0 extended"));

    msglen = makeUBXCFG(0x06, 0x17, sizeof(setNMEA_U8), setNMEA_U8);
    sendUBX(GNSSbuf, msglen);
    gnss_set_sucess = getUBX_ACK(0x06, 0x17);
    if (!gnss_set_sucess) {
      //GNSS_DEBUG_PRINTLN(F("WARNING: Unable to set NMEA version 4.0 extended"));
      Serial.println(F("WARNING: Unable to set NMEA version 4.0 extended"));
    }

  }

  GNSS_DEBUG_PRINTLN(F("Setting SBAS"));

  msglen = makeUBXCFG(0x06, 0x16, sizeof(setSBAS), setSBAS);
  sendUBX(GNSSbuf, msglen);
  gnss_set_sucess = getUBX_ACK(0x06, 0x16);
  if (!gnss_set_sucess) {
    //GNSS_DEBUG_PRINTLN(F("WARNING: Unable to set SBAS"));
    Serial.println(F("WARNING: Unable to set SBAS"));
  }

  GNSS_DEBUG_PRINTLN(F("Airborne <2g navigation mode: "));

  // Set the navigation mode (Airborne, < 2g)
  msglen = makeUBXCFG(0x06, 0x24, sizeof(setNav5), setNav5);
  sendUBX(GNSSbuf, msglen);
  gnss_set_sucess = getUBX_ACK(0x06, 0x24);
  if (!gnss_set_sucess) {
    //GNSS_DEBUG_PRINTLN(F("WARNING: Unable to set airborne <2g navigation mode."));
    Serial.println(F("WARNING: Unable to set airborne <2g navigation mode."));
  }

  GNSS_DEBUG_PRINTLN(F("Switching on NMEA GGA: "));

  msglen = makeUBXCFG(0x06, 0x01, sizeof(enaGGA), enaGGA);
  sendUBX(GNSSbuf, msglen);
  gnss_set_sucess = getUBX_ACK(0x06, 0x01);
  if (!gnss_set_sucess) {
    //GNSS_DEBUG_PRINTLN(F("WARNING: Unable to enable NMEA GGA."));
    Serial.println(F("WARNING: Unable to enable NMEA GGA."));
  }

  GNSS_DEBUG_PRINTLN(F("Switching on NMEA RMC: "));

  msglen = makeUBXCFG(0x06, 0x01, sizeof(enaRMC), enaRMC);
  sendUBX(GNSSbuf, msglen);
  gnss_set_sucess = getUBX_ACK(0x06, 0x01);
  if (!gnss_set_sucess) {
    //GNSS_DEBUG_PRINTLN(F("WARNING: Unable to enable NMEA RMC."));
    Serial.println(F("WARNING: Unable to enable NMEA RMC."));
  }

  GNSS_DEBUG_PRINTLN(F("Switching off NMEA GLL: "));

  msglen = makeUBXCFG(0x06, 0x01, sizeof(disGLL), disGLL);
  sendUBX(GNSSbuf, msglen);
  gnss_set_sucess = getUBX_ACK(0x06, 0x01);
  if (!gnss_set_sucess) {
    //GNSS_DEBUG_PRINTLN(F("WARNING: Unable to disable NMEA GLL."));
    Serial.println(F("WARNING: Unable to disable NMEA GLL."));
  }

  bool enable = ((settings->nmea_g | settings->nmea2_g) & NMEA_G_GSA);
  if (enable) {
    GNSS_DEBUG_PRINTLN(F("Setting NMEA GSA: "));
    msglen = makeUBXCFG(0x06, 0x01, sizeof(enaGSA), (enable ? enaGSA : disGSA));
    sendUBX(GNSSbuf, msglen);
    gnss_set_sucess = getUBX_ACK(0x06, 0x01);
    if (!gnss_set_sucess) {
      //GNSS_DEBUG_PRINTLN(F("WARNING: Unable to set NMEA GSA."));
      Serial.println(F("WARNING: Unable to set NMEA GSA."));
    } else {
      Serial.print(F("Set GNSS NMEA GSA to "));
      Serial.println(enable);
    }
  }

  enable = ((settings->nmea_g | settings->nmea2_g) & NMEA_G_GST);
  if (enable) {
    GNSS_DEBUG_PRINTLN(F("Setting NMEA GST: "));
    msglen = makeUBXCFG(0x06, 0x01, sizeof(enaGST), (enable ? enaGST : disGST));
    sendUBX(GNSSbuf, msglen);
    gnss_set_sucess = getUBX_ACK(0x06, 0x01);
    if (!gnss_set_sucess) {
      //GNSS_DEBUG_PRINTLN(F("WARNING: Unable to set NMEA GST."));
      Serial.println(F("WARNING: Unable to set NMEA GST."));
    } else {
      Serial.print(F("Set GNSS NMEA GST to "));
      Serial.println(enable);
    }
  }

  enable = ((settings->nmea_g | settings->nmea2_g) & NMEA_G_GSV);
  if (enable) {
    GNSS_DEBUG_PRINTLN(F("Setting NMEA GSV: "));
    msglen = makeUBXCFG(0x06, 0x01, sizeof(enaGSV), (enable ? enaGSV : disGSV));
    sendUBX(GNSSbuf, msglen);
    gnss_set_sucess = getUBX_ACK(0x06, 0x01);
    if (!gnss_set_sucess) {
      //GNSS_DEBUG_PRINTLN(F("WARNING: Unable to set NMEA GSV."));
      Serial.println(F("WARNING: Unable to set NMEA GSV."));
    } else {
      Serial.print(F("Set GNSS NMEA GSV to "));
      Serial.println(enable);
    }
  }

  GNSS_DEBUG_PRINTLN(F("Switching off NMEA VTG: "));

  msglen = makeUBXCFG(0x06, 0x01, sizeof(disVTG), disVTG);
  sendUBX(GNSSbuf, msglen);
  gnss_set_sucess = getUBX_ACK(0x06, 0x01);

  if (!gnss_set_sucess) {
    //GNSS_DEBUG_PRINTLN(F("WARNING: Unable to disable NMEA VTG."));
    Serial.println(F("WARNING: Unable to disable NMEA VTG."));
  }
}

/* ------ BEGIN -----------  https://github.com/Black-Thunder/FPV-Tracker */

enum ubloxState{ WAIT_SYNC1, WAIT_SYNC2, GET_CLASS, GET_ID, GET_LL, GET_LH, GET_DATA, GET_CKA, GET_CKB };

ubloxState ubloxProcessDataState = WAIT_SYNC1;

unsigned short ubloxExpectedDataLength;
unsigned short ubloxClass, ubloxId;
unsigned char  ubloxCKA, ubloxCKB;

// process serial data
// data is stored inside #GNSSbuf, data size inside #GNSS_cnt
// warning : if #GNSSbuf is too short, data is truncated.
static int ubloxProcessData(unsigned char data) {
	int parsed = 0;

	switch (ubloxProcessDataState) {
	case WAIT_SYNC1:
		if (data == 0xb5) {
			ubloxProcessDataState = WAIT_SYNC2;
		}
		break;

	case WAIT_SYNC2:
		if (data == 0x62) {
			ubloxProcessDataState = GET_CLASS;
		}
		else if (data == 0xb5) {
			// ubloxProcessDataState = GET_SYNC2;
		}
		else {
			ubloxProcessDataState = WAIT_SYNC1;
		}
		break;
	case GET_CLASS:
		ubloxClass = data;
		ubloxCKA = data;
		ubloxCKB = data;
		ubloxProcessDataState = GET_ID;
		break;

	case GET_ID:
		ubloxId = data;
		ubloxCKA += data;
		ubloxCKB += ubloxCKA;
		ubloxProcessDataState = GET_LL;
		break;

	case GET_LL:
		ubloxExpectedDataLength = data;
		ubloxCKA += data;
		ubloxCKB += ubloxCKA;
		ubloxProcessDataState = GET_LH;
		break;

	case GET_LH:
		ubloxExpectedDataLength += data << 8;
		GNSS_cnt = 0;
		ubloxCKA += data;
		ubloxCKB += ubloxCKA;
		ubloxProcessDataState = GET_DATA;
		break;

	case GET_DATA:
		ubloxCKA += data;
		ubloxCKB += ubloxCKA;
		if (GNSS_cnt < sizeof(GNSSbuf)) {
			GNSSbuf[GNSS_cnt++] = data;
		}
		if ((--ubloxExpectedDataLength) == 0) {
			ubloxProcessDataState = GET_CKA;
		}
		break;

	case GET_CKA:
		if (ubloxCKA != data) {
			ubloxProcessDataState = WAIT_SYNC1;
		}
		else {
			ubloxProcessDataState = GET_CKB;
		}
		break;

	case GET_CKB:
		if (ubloxCKB == data) {
			parsed = 1;
		}
		ubloxProcessDataState = WAIT_SYNC1;
		break;

	}

	return parsed;
}

/* ------ END -----------  https://github.com/Black-Thunder/FPV-Tracker */

static bool ublox_query(uint8_t requestedClass, uint8_t requestedID, uint16_t timelimit)
{
  unsigned long startTime = millis();

Serial.println("ublox_query()...");

  uint8_t msglen = makeUBXCFG(requestedClass, requestedID, 0, NULL);
  while (Serial_GNSS_In.available() > 0) { Serial_GNSS_In.read(); }
  sendUBX(GNSSbuf, msglen);
  yield();

  int count = 1;

  // Get the message back from the GPS
  GNSS_DEBUG_PRINT(F(" * Reading response: "));

  uint32_t waittime = (millis() - startTime);

  while (waittime < (uint32_t) timelimit ) {

    if ((waittime > 1000 && count==1) || (waittime > 2000 && count==2)) {
Serial.println("... re-sending UBX command");
      uint8_t msglen = makeUBXCFG(requestedClass, requestedID, 0, NULL); // MON-VER
      while (Serial_GNSS_In.available() > 0) { Serial_GNSS_In.read(); }
      sendUBX(GNSSbuf, msglen);
      ++count;
      yield();
    }

    if (waittime > timelimit-100) {
      Serial.println("... time out");
      break;
    }

    if (Serial_GNSS_In.available()) {

      unsigned char c = Serial_GNSS_In.read();
      GNSS_DEBUG_PRINT(c, HEX);

      if (ubloxProcessData(c)) {
        if (ubloxClass == requestedClass && ubloxId == requestedID) {
            GNSS_cnt = 0;
            return true;
        }
      }
    }

    waittime = (millis() - startTime);
    yield();
  }

  GNSS_cnt = 0;
  return false;
}

static byte ublox_version()
{
    byte rval = GNSS_MODULE_NMEA;
    if (ublox_query(0x0A, 0x04, 3000) == false) // MON-VER
        return rval;

            // UBX-MON-VER data description
            // uBlox 6  - page 166 : https://www.u-blox.com/sites/default/files/products/documents/u-blox6_ReceiverDescrProtSpec_%28GPS.G6-SW-10018%29_Public.pdf
            // uBlox 7  - page 153 : https://www.u-blox.com/sites/default/files/products/documents/u-blox7-V14_ReceiverDescriptionProtocolSpec_%28GPS.G7-SW-12001%29_Public.pdf
            // uBlox M8 - page 300 : https://www.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_%28UBX-13003221%29_Public.pdf

    Serial.print(F("INFO: UBLOX GNSS module HW version: "));
    Serial.println((char *) &GNSSbuf[30]);

    Serial.print(F("INFO: UBLOX GNSS module FW version: "));
    Serial.println((char *) &GNSSbuf[0]);

#ifdef DO_GNSS_DEBUG
    for(unsigned i = 30 + 10; i < GNSS_cnt; i+=30) {
      Serial.print(F("INFO: GNSS module extension: "));
      Serial.println((char *) &GNSSbuf[i]);
    }
#endif

    if (GNSSbuf[33] == '4')
      rval = GNSS_MODULE_U6;
    else if (GNSSbuf[33] == '7')
      rval = GNSS_MODULE_U7;
    else if (GNSSbuf[33] == '8')
      rval = GNSS_MODULE_U8;
    else if (GNSSbuf[32] == '1' && GNSSbuf[33] == '9')
      rval = GNSS_MODULE_U9;
    else if (GNSSbuf[33] == 'A')
      rval = GNSS_MODULE_U10;

    return rval;
}

static gnss_id_t ublox_probe()
{
  /*
   * ESP8266 NodeMCU and ESP32 DevKit (with NodeMCU adapter)
   * have no any spare GPIO pin to provide GNSS Tx feedback
   */
  return(hw_info.model == SOFTRF_MODEL_STANDALONE && hw_info.revision == 0 ?
         GNSS_MODULE_NMEA : (gnss_id_t) ublox_version());
}

static bool ublox_setup()
{
#if 1
  // Set the navigation mode (Airborne, 1G)
  // Turning off some GPS NMEA sentences on the uBlox modules
  setup_UBX();
#else
  //Serial_GNSS_Out.write("$PUBX,41,1,0007,0003,9600,0*10\r\n");
  Serial_GNSS_Out.write("$PUBX,41,1,0007,0003,38400,0*20\r\n");

  GNSS_FLUSH();
  SoC->swSer_begin(38400);

  // Turning off some GPS NMEA strings on the uBlox modules
  Serial_GNSS_Out.write("$PUBX,40,GLL,0,0,0,0*5C\r\n"); delay(250);
  Serial_GNSS_Out.write("$PUBX,40,GSV,0,0,0,0*59\r\n"); delay(250);
  Serial_GNSS_Out.write("$PUBX,40,VTG,0,0,0,0*5E\r\n"); delay(250);
  if (! ((settings->nmea_g | settings->nmea2_g) & NMEA_G_GSA))
    Serial_GNSS_Out.write("$PUBX,40,GSA,0,0,0,0*4E\r\n"); delay(250);
#endif

  return true;
}

static void ublox_loop()
{
#if defined(USE_GNSS_PSM)
  if (settings->power_save & POWER_SAVE_GNSS) {
    if (hw_info.model == SOFTRF_MODEL_UNI) {

      if (!gnss_psm_active && isValidGNSSFix() && gnss.satellites.value() > 5) {
        // Setup for Power Save Mode (Default Cyclic 1s)
        for (int i = 0; i < sizeof(RXM_PSM); i++) {
          Serial_GNSS_Out.write(pgm_read_byte(&RXM_PSM[i]));
        }

        GNSS_DEBUG_PRINTLN(F("INFO: GNSS Power Save Mode"));
        gnss_psm_active = true;
      } else if (  gnss_psm_active &&
                 ((gnss.satellites.isValid() && gnss.satellites.value() <= 5) ||
                   gnss.satellites.age() > NMEA_EXP_TIME)) {
        // Setup for Continuous Mode
        for (int i = 0; i < sizeof(RXM_MAXP); i++) {
          Serial_GNSS_Out.write(pgm_read_byte(&RXM_MAXP[i]));
        }

        GNSS_DEBUG_PRINTLN(F("INFO: GNSS Continuous Mode"));
        gnss_psm_active = false;
      }
    }
  }
#endif /* USE_GNSS_PSM */
}

static void ublox_fini()
{
  // Controlled Software reset
  for (int i = 0; i < sizeof(CFG_RST); i++) {
    Serial_GNSS_Out.write(pgm_read_byte(&CFG_RST[i]));
  }

  delay(hw_info.gnss == GNSS_MODULE_U8 ? 1000 : 600);

  // power off until wakeup call
  for (int i = 0; i < sizeof(RXM_PMREQ_OFF); i++) {
    Serial_GNSS_Out.write(pgm_read_byte(&RXM_PMREQ_OFF[i]));
  }
}

const gnss_chip_ops_t ublox_ops = {
  ublox_probe,
  ublox_setup,
  ublox_loop,
  ublox_fini,
  138 /* GGA */, 67 /* RMC */
};


#if 0
// adapted code from github.com/Xinyuan-LilyGO/LilyGo-LoRa-Series/tree/master/examples/GPS/UBlox_Recovery
int getAck(uint8_t requestedClass, uint8_t requestedID)
{
    uint16_t    ubxFrameCounter = 0;
    bool        ubxFrame = 0;
    uint32_t    startTime = millis();
    uint16_t    needRead;

    while (millis() - startTime < 800) {
        while (Serial_GNSS_In.available()) {
            int c = Serial_GNSS_In.read();
            switch (ubxFrameCounter) {
            case 0:
                if (c == 0xB5) {
                    ubxFrameCounter++;
                }
                break;
            case 1:
                if (c == 0x62) {
                    ubxFrameCounter++;
                } else {
                    ubxFrameCounter = 0;
                }
                break;
            case 2:
                if (c == requestedClass) {
                    ubxFrameCounter++;
                } else {
                    ubxFrameCounter = 0;
                }
                break;
            case 3:
                if (c == requestedID) {
                    ubxFrameCounter++;
                } else {
                    ubxFrameCounter = 0;
                }
                break;
            case 4:
                needRead = c;
                ubxFrameCounter++;
                break;
            case 5:
                needRead |=  (c << 8);
                ubxFrameCounter++;
                break;
            case 6:
                if (needRead >= sizeof(GNSSbuf)) {
                    ubxFrameCounter = 0;
                    break;
                }
                if (Serial_GNSS_In.readBytes(GNSSbuf, needRead) != needRead) {
                    ubxFrameCounter = 0;
                } else {
                    return needRead;
                }
                break;

            default:
                break;
            }
        }
        yield();
    }
    return 0;
}
#endif

static bool ublox_factory_reset(bool coldstart)
{
  // reset GPS to factory settings
  //for (int i = 0; i < sizeof(factoryUBX); i++) {
  //  Serial_GNSS_Out.write(pgm_read_byte(&factoryUBX[i]));
  //}

  // adapted code from:
  //   github.com/Xinyuan-LilyGO/LilyGo-LoRa-Series/tree/master/examples/GPS/UBlox_Recovery
  //Serial_GNSS_In.write(cfg_clear2, sizeof(cfg_clear2));   // clear flash config
  int msglen = makeUBXCFG(0x06, 0x09, sizeof(cfg_clear2), cfg_clear2);
  sendUBX(GNSSbuf, msglen);
  //if (getAck(0x05, 0x01)) {
  if (getUBX_ACK(0x06, 0x09)) {
      Serial.println("cleared flash config");
  } else return false;
//  Serial_GNSS_In.write(cfg_clear4, sizeof(cfg_clear4));   // clear EEPROM config
  msglen = makeUBXCFG(0x06, 0x09, sizeof(cfg_clear4), cfg_clear4);
  sendUBX(GNSSbuf, msglen);
  if (getUBX_ACK(0x06, 0x09)) {
      Serial.println("cleared EEPROM config");
  } else return false;
//  Serial_GNSS_In.write(cfg_clear1, sizeof(cfg_clear1));   // clear BBR config
  msglen = makeUBXCFG(0x06, 0x09, sizeof(cfg_clear1), cfg_clear1);
  sendUBX(GNSSbuf, msglen);
  if (getUBX_ACK(0x06, 0x09)) {
      Serial.println("cleared BBR config");
  } else return false;

/*
 * Do NOT "load" the new (factory default) settings since the baud rate will change and thus there
 * would be no ACK received.  Instead, just do the restart, that will load the new settings.
 *
//  Serial_GNSS_In.write(cfg_load3, sizeof(cfg_load3));    // load config from all 3 devices (but not SPI)
  msglen = makeUBXCFG(0x06, 0x09, sizeof(cfg_load3), cfg_load3);
  sendUBX(GNSSbuf, msglen);
  if (getUBX_ACK(0x06, 0x09)) {
      Serial.println("UBX config reloaded");
  } else return false;
  delay(600);
*/

  delay(100);

  if (coldstart) {
    // Cold Start (Forced Watchdog)
    Serial.println("Cold-starting GNSS module...");
  //for (int i = 0; i < sizeof(CFG_RST_COLD); i++) {
  //  Serial_GNSS_Out.write(pgm_read_byte(&CFG_RST_COLD[i]));
  //}
//  Serial_GNSS_In.write(CFG_RST_COLD, sizeof(CFG_RST_COLD));
    msglen = makeUBXCFG(0x06, 0x04, sizeof(CFG_RST_COLD), CFG_RST_COLD);
    sendUBX(GNSSbuf, msglen);
  } else {  // warm start
    msglen = makeUBXCFG(0x06, 0x04, sizeof(CFG_RST_WARM), CFG_RST_WARM);
    sendUBX(GNSSbuf, msglen);
  }
  delay(2000);

  return true;
}
#endif /* EXCLUDE_GNSS_UBLOX */

static void reset_gnss()
{
    if (hw_info.model == SOFTRF_MODEL_PRIME_MK2 ||
        hw_info.model == SOFTRF_MODEL_PRIME_MK3) {
#if !defined(EXCLUDE_GNSS_UBLOX)
      if (hw_info.gnss == GNSS_MODULE_U6 ||
          hw_info.gnss == GNSS_MODULE_U7 ||
          hw_info.gnss == GNSS_MODULE_U8) {

            Serial.println("Ublox factory reset...");
            if (ublox_factory_reset(true))
                Serial.println("... done");
            else
                Serial.println("... failed");

      }
#endif
#if !defined(EXCLUDE_GNSS_AT65)
      if (hw_info.gnss == GNSS_MODULE_AT65) {
          if (Serial_GNSS_Out.availableForWrite() < 16)
              delay(600);
          if (Serial_GNSS_Out.availableForWrite() < 16) {
              Serial.println("Serial_GNSS_Out.write() blocked?");
              return;
          }
          Serial.println("AT65 factory defaults...");
          Serial_GNSS_Out.write("$PCAS10,3*1F\r\n");   // factory defaults
          delay(600);
          if (Serial_GNSS_Out.availableForWrite() < 16)
              delay(600);
          if (Serial_GNSS_Out.availableForWrite() < 16) {
              Serial.println("Serial_GNSS_Out.write() blocked?");
              return;
          }
          Serial_GNSS_Out.write("\0");
          delay(300);
          Serial.println("AT65 cold start...");
          Serial_GNSS_Out.write("$PCAS10,2*1E\r\n");   // cold start
          delay(600);
          Serial.println("... done");
      }
#endif
    }
}

#if !defined(EXCLUDE_GNSS_SONY)
static gnss_id_t sony_probe()
{
  /* Wake-up */
  Serial_GNSS_Out.write("@WUP\r\n");       delay(500);

  /* Firmware version request */
  return nmea_handshake("@VER\r\n", "[VER] Done", true) ?
                        GNSS_MODULE_SONY : GNSS_MODULE_NONE;
}

static bool sony_setup()
{
  /* Idle */
  Serial_GNSS_Out.write("@GSTP\r\n");
  GNSS_FLUSH();
  delay(2000);

#if !defined(EXCLUDE_LOG_GNSS_VERSION)
  while (Serial_GNSS_In.available() > 0) { Serial_GNSS_In.read(); }

  Serial_GNSS_Out.write("@VER\r\n");

  int i=0;
  char c;
  unsigned long start_time = millis();

  /* take response into buffer */
  while ((millis() - start_time) < 2000) {

    c = Serial_GNSS_In.read();

    if (isPrintable(c) || c == '\r' || c == '\n') {
      if (i >= sizeof(GNSSbuf) - 1) break;
      GNSSbuf[i++] = c;
    } else {
      /* ignore */
      continue;
    }

    if (c == '\n') break;
  }

  GNSSbuf[i] = 0;

  if (strlen((char *) &GNSSbuf[0])) {
    Serial.print(F("INFO: Sony GNSS module FW version: "));
    Serial.println((char *) &GNSSbuf[0]);
  }

  delay(250);
#endif

  /* GGA + GSA + RMC */
  Serial_GNSS_Out.write("@BSSL 0x25\r\n"); delay(250);
  // >>> should find out how to enable/disable GSA
  /* GPS + GLONASS. This command must be issued at Idle state */
  Serial_GNSS_Out.write("@GNS 3\r\n");     delay(250);
  /*  Positioning algorithm. This command must be issued at Idle state */
  Serial_GNSS_Out.write("@GUSE 0\r\n");    delay(250);

 if (get_pps_pin() != SOC_UNUSED_PIN) {
   /* Enable 1PPS output */
   Serial_GNSS_Out.write("@GPPS 1\r\n");    delay(250);
 }

#if defined(USE_GNSS_PSM)
  if (settings->power_save & POWER_SAVE_GNSS) {
    /*
     * Low power, 1 second interval, no sleep
     *
     * WARNING: use of this mode may cause issues
     */
    Serial_GNSS_Out.write("@GSOP 2 1000 0\r\n"); delay(250);
  }
#endif /* USE_GNSS_PSM */

  /*
   * Hot start for TTFF
   * When the conditions for the hot start have not been met,
   * positioning is started automatically using a warm start or cold start.
   */
  Serial_GNSS_Out.write("@GSR\r\n");

  GNSS_FLUSH(); delay(100);

  return true;
}

static void sony_loop()
{

}

static void sony_fini()
{
   /* Idle */
  Serial_GNSS_Out.write("@GSTP\r\n");
  GNSS_FLUSH(); delay(1500);

  /* Sony GNSS sleep level (0-2)
   * This command must be issued at Idle state.
   * When this command is issued at Exec state, error is returned.
   */

//  Serial_GNSS_Out.write("@SLP 0\r\n");
  Serial_GNSS_Out.write("@SLP 1\r\n");
//  Serial_GNSS_Out.write("@SLP 2\r\n");

  GNSS_FLUSH(); delay(100);
}

const gnss_chip_ops_t sony_ops = {
  sony_probe,
  sony_setup,
  sony_loop,
  sony_fini,
  24 /* GGA */, 38 /* RMC */
};
#endif /* EXCLUDE_GNSS_SONY */

#if !defined(EXCLUDE_GNSS_MTK)
static gnss_id_t mtk_probe()
{
  /* Firmware version request */
  return nmea_handshake("$PMTK605*31\r\n", "$PMTK705", false) ?
                        GNSS_MODULE_MT33 : GNSS_MODULE_NMEA;
}

static bool mtk_setup()
{
#if !defined(EXCLUDE_LOG_GNSS_VERSION)
  Serial_GNSS_Out.write("$PMTK605*31\r\n");

  int i=0;
  char c;
  unsigned long start_time = millis();

  /* take response into buffer */
  while ((millis() - start_time) < 2000) {

    c = Serial_GNSS_In.read();

    if (isPrintable(c) || c == '\r' || c == '\n') {
      if (i >= sizeof(GNSSbuf) - 1) break;
      GNSSbuf[i++] = c;
    } else {
      /* ignore */
      continue;
    }

    if (c == '\n') break;
  }

  GNSSbuf[i] = 0;

  size_t len = strlen((char *) &GNSSbuf[0]);

  if (len > 19) {
    for (int i=9; i < len; i++) {
      if (GNSSbuf[i] == ',') {
        GNSSbuf[i] = 0;
      }
    }
    Serial.print(F("INFO: MTK GNSS module FW version: "));
    Serial.println((char *) &GNSSbuf[9]);
  }

  delay(250);
#endif

  /* RMC + GGA + GSA */
  Serial_GNSS_Out.write("$PMTK314,0,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n");
  // >>> should find out how to enable/disable GSA
  GNSS_FLUSH(); delay(250);

  /* Aviation mode */
  Serial_GNSS_Out.write("$PMTK886,2*2A\r\n");
  GNSS_FLUSH(); delay(250);

  return true;
}

static void mtk_loop()
{

}

static void mtk_fini()
{
  /* Stop mode */
  Serial_GNSS_Out.write("$PMTK161,0*28\r\n");
  GNSS_FLUSH(); delay(250);
}

const gnss_chip_ops_t mtk_ops = {
  mtk_probe,
  mtk_setup,
  mtk_loop,
  mtk_fini,
  48 /* GGA */, 175 /* RMC */
};
#endif /* EXCLUDE_GNSS_MTK */

#if !defined(EXCLUDE_GNSS_GOKE)
static gnss_id_t goke_probe()
{
  /* Firmware version request */
  return nmea_handshake("$PGKC462*2F\r\n", "$PGKC463", false) ?
                        GNSS_MODULE_GOKE : GNSS_MODULE_NMEA;
}

static void goke_sendcmd(const char *cmd)
{
  while (Serial_GNSS_In.available() > 0) { while (Serial_GNSS_In.read() != '\n') {yield();} }
  Serial_GNSS_In.write(cmd);
  GNSS_FLUSH();
  delay(250);
}

static bool goke_setup()
{
  /* There are reports that Air530 does not actually work with GALILEO yet */
  if (settings->band == RF_BAND_CN) {
    /* GPS + BEIDOU */
    goke_sendcmd("$PGKC115,1,0,1,0*2A\r\n");
  } else {
    /* GPS + GLONASS */
    goke_sendcmd("$PGKC115,1,1,0,0*2A\r\n");
  }

#if 0
  /* SBAS */
  goke_sendcmd("$PGKC239,1*3A\r\n");
#endif

  if ((settings->nmea_g | settings->nmea2_g) & NMEA_G_GSA)  /* RMC + GGA + GSA */
    goke_sendcmd("$PGKC242,0,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0*36\r\n");
  else   /* RMC + GGA */
    goke_sendcmd("$PGKC242,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*37\r\n");

  if (get_pps_pin() != SOC_UNUSED_PIN) {
    /* Enable 3D fix 1PPS output */
    goke_sendcmd("$PGKC161,2,200,1000*04\r\n");
  }

  return true;
}

static void goke_loop()
{

}

static void goke_fini()
{
  goke_sendcmd("$PGKC051,0*37\r\n");
  // goke_sendcmd("$PGKC051,1*36\r\n");
  // goke_sendcmd("$PGKC105,4*33\r\n");
}

const gnss_chip_ops_t goke_ops = {
  goke_probe,
  goke_setup,
  goke_loop,
  goke_fini,
  185 /* GGA */, 265 /* RMC */
};
#endif /* EXCLUDE_GNSS_GOKE */

#if !defined(EXCLUDE_GNSS_AT65)
static gnss_id_t at65_probe()
{
  /* Firmware version request */
  return nmea_handshake("$PCAS06,0*1B\r\n", "$GPTXT,01,01,02", false) ?
                        GNSS_MODULE_AT65 : GNSS_MODULE_NMEA;
}

static bool at65_setup()
{
#if !defined(EXCLUDE_LOG_GNSS_VERSION)
  Serial_GNSS_Out.write("$PCAS06,0*1B\r\n");

  int i=0;
  char c;
  unsigned long start_time = millis();

  /* take response into buffer */
  while ((millis() - start_time) < 2000) {

    c = Serial_GNSS_In.read();

    if (isPrintable(c) || c == '\r' || c == '\n') {
      if (i >= sizeof(GNSSbuf) - 1) break;
      GNSSbuf[i++] = c;
    } else {
      /* ignore */
      continue;
    }

    if (c == '\n') break;
  }

  GNSSbuf[i] = 0;

  size_t len = strlen((char *) &GNSSbuf[0]);

  if (len > 19) {
    for (int i=19; i < len; i++) {
      if (GNSSbuf[i] == '*') {
        GNSSbuf[i] = 0;
      }
    }
    Serial.print(F("INFO: AT65 GNSS module FW version: "));
    Serial.println((char *) &GNSSbuf[19]);
  }

  delay(250);
#endif

  /* Assume that we deal with fake NEO module (AT6558 based) */
  if (hw_info.model == SOFTRF_MODEL_BADGE)
      Serial_GNSS_Out.write("$PCAS04,7*1E\r\n"); /* GPS + GLONASS + BEIDOU */
  else
      Serial_GNSS_Out.write("$PCAS04,5*1C\r\n"); /* GPS + GLONASS */     
    //Serial_GNSS_Out.write("$PCAS04,7*1E\r\n"); /* GPS + GLONASS + BEIDOU */
  delay(250);
  Serial_GNSS_Out.write("$PCAS02,1000*2E\r\n");  /* set positioning frequency to 1Hz */
  if ((settings->nmea_g | settings->nmea2_g) & NMEA_G_GSA) {
    /* GGA,RMC and GSA */
    Serial_GNSS_Out.write("$PCAS03,1,0,1,0,1,0,0,0,0,0,,,0,0*03\r\n");
    delay(250);
  } else {
    /* GGA and RMC */
    Serial_GNSS_Out.write("$PCAS03,1,0,0,0,1,0,0,0,0,0,,,0,0*02\r\n");
    delay(250);
  }
  Serial_GNSS_Out.write("$PCAS11,6*1B\r\n"); /* Aviation < 2g */
  delay(250);

  return true;
}

static void at65_loop()
{

}

static void at65_fini()
{

}

const gnss_chip_ops_t at65_ops = {
  at65_probe,
  at65_setup,
  at65_loop,
  at65_fini,
  70 /* GGA */, 135 /* RMC */
};
#endif /* EXCLUDE_GNSS_AT65 */

static bool GNSS_fix_cache = false;
static bool badGGA = true;

int8_t leap_seconds_correction = 0;

static uint8_t leap_valid = 2;  // means not known whether valid

bool leap_seconds_valid()
{
    // could start = 0 but risk 13 minutes of no-transmit if no response
    // - usually gets set to 0 in first check below (if no leap seconds)
    //    - before even the first fix, so no bad transmissions happen

    if (leap_valid == 1)
        return true;

    // this is only called if GNSS_fix_cache == true
    switch (hw_info.gnss) {
#if !defined(EXCLUDE_GNSS_UBLOX)
        case GNSS_MODULE_U6:
        case GNSS_MODULE_U7:
        case GNSS_MODULE_U8:
        case GNSS_MODULE_U9:
        case GNSS_MODULE_U10:
            static uint32_t next_check = 0;
            static uint8_t checks_count = 0;
            static uint8_t max_checks = 18;   // 13 minutes
            if (checks_count<max_checks && (checks_count==0 || millis()>next_check)) {
                if (ublox_query(0x01, 0x20, 2000) == true) {    // NAV-TIMEGPS
                    int8_t leap_seconds_from_gnss = GNSSbuf[10];
                    Serial.print("UBX leap seconds = ");
                    Serial.println(leap_seconds_from_gnss);
                    if ((GNSSbuf[11] & 0x04) == 0) {
                        leap_valid = 0;    // known invalid
                        Serial.println(F("UBX says leap seconds unknown, is using default"));
                        // U6 defaults to 15, 2025 correct value is 18:
                        leap_seconds_correction = settings->leapsecs - leap_seconds_from_gnss;
                        Serial.print(F("SoftRF using leap_seconds_correction = "));
                        Serial.println(leap_seconds_correction);
                    } else {
                        leap_valid = 1;    // known valid, no need to ask again
                        Serial.println("UBX says leap seconds known valid");
                        leap_seconds_correction = 0;    // no correction needed
                        if (settings->leapsecs != leap_seconds_from_gnss) {
                            settings->leapsecs = leap_seconds_from_gnss;
                            save_settings_to_file();
                            // this will only happen once in a few years!
                        }
                    }
                } else {
                    // (query failed) - no change in leap_valid
                    Serial.println("No response to UBX leap seconds query");
                }
                next_check = millis() + 43000;
                ++checks_count;
                if (checks_count >= 18) {
                    leap_valid = 2;     // in case it never works
                    max_checks = 36;    // keep trying for up to 13 more minutes
                }
            }
            break;
#endif
        case GNSS_MODULE_AT65:
        default:
            return true;    // on other models assume valid
            break;
    }
    return (leap_valid != 0);   // if not known invalid, assume valid
}

bool isValidGNSSFix()
{
  return GNSS_fix_cache;
}

// variables for $PFSIM
TinyGPSCustom P_timestamp;
TinyGPSCustom P_addr;
TinyGPSCustom P_addrtype;
TinyGPSCustom P_actype;
TinyGPSCustom P_lat;
TinyGPSCustom P_lon;
TinyGPSCustom P_alt;
TinyGPSCustom P_speed;
TinyGPSCustom P_course;
TinyGPSCustom P_vs;
TinyGPSCustom P_turnrate;

// call this to try other baud rates if the default 9600 failed to receive NMEA
static gnss_id_t probe_baud_rates()
{
    if (settings->mode == SOFTRF_MODE_GPSBRIDGE)  // do not change baud rate
        return generic_nmea_probe();              // just try one more time at same rate
    gnss_id_t rval = GNSS_MODULE_NONE;
    for (int i=BAUD_115200; i>=BAUD_9600; i--) {
        unsigned long baudrate = baudrates[i];
#if defined(ARDUINO_ARCH_NRF52)
        Serial_GNSS_In.flush();
        Serial_GNSS_In.end();
        SoC->swSer_begin(baudrate);   // because updateBaudRate() not supported
#else
        Serial_GNSS_In.updateBaudRate(baudrate);
#endif
        Serial.print(F("Trying baud rate... "));
        Serial.println(baudrate);
        delay(500);
        rval = generic_nmea_probe();
        if (rval == GNSS_MODULE_NMEA) {
            Serial.println(F("... got NMEA!"));
            break;
        }
#if !defined(EXCLUDE_GNSS_UBLOX) && defined(ENABLE_UBLOX_RFS)
        if (hw_info.model == SOFTRF_MODEL_PRIME_MK2 ||
            hw_info.model == SOFTRF_MODEL_PRIME_MK3) {
            // no NMEA, try UBX
            byte version = ublox_version();
            if (version == GNSS_MODULE_U6 ||
                version == GNSS_MODULE_U7 ||
                version == GNSS_MODULE_U8) {
                    Serial.println(F("... got UBX!"));
                    // rval still == GNSS_MODULE_NONE
                    // but baud rate is now correct
                    // and later code will find the UBX
                    break;
            }
        }
#endif
    }
    return rval;
}

byte GNSS_setup() {

  if (hw_info.model == SOFTRF_MODEL_PRIME_MK2 /* && hw_info.revision >= 8 */)
      is_prime_mk2 = true;

  //gnss_id_t gnss_id = GNSS_MODULE_NONE;

#if defined(USE_SD_CARD)
  if ((! is_prime_mk2) || (settings->debug_flags & DEBUG_SIMULATE) == 0
      || ((! SIMfileOpen) && (settings->gnss_pins != EXT_GNSS_NONE))) {
         // this assumes SD_setup() is called before GNSS_setup()
#else
  if ((! is_prime_mk2) || (settings->debug_flags & DEBUG_SIMULATE) == 0
      || (settings->gnss_pins != EXT_GNSS_NONE)) {
#endif
      unsigned long baudrate = SERIAL_IN_BR;
      if (settings->mode == SOFTRF_MODE_GPSBRIDGE)
          baudrate = baudrates[settings->baud_rate];  // a way to reach a misconfigured GNSS module
      SoC->swSer_begin(baudrate);
      delay(500);  // added to make sure swSer is ready
  }

  if (is_prime_mk2 && settings->debug_flags & DEBUG_SIMULATE) {
      const char *psrf_p = "PFSIM";
      int term_num = 1;
      P_timestamp.begin (gnss, psrf_p, term_num++);   // for "target" data sentences
      P_addr.begin      (gnss, psrf_p, term_num++);
      P_addrtype.begin  (gnss, psrf_p, term_num++);
      P_actype.begin    (gnss, psrf_p, term_num++);
      P_lat.begin       (gnss, psrf_p, term_num++);
      P_lon.begin       (gnss, psrf_p, term_num++);
      P_alt.begin       (gnss, psrf_p, term_num++);
      P_speed.begin     (gnss, psrf_p, term_num++);
      P_course.begin    (gnss, psrf_p, term_num++);
      P_vs.begin        (gnss, psrf_p, term_num++);
      P_turnrate.begin  (gnss, psrf_p, term_num++);
      GNSS_cnt = 0;
      gnss_chip = &generic_nmea_ops;
      return GNSS_MODULE_NMEA;
  }

  if (hw_info.model == SOFTRF_MODEL_PRIME_MK2 ||
      hw_info.model == SOFTRF_MODEL_PRIME_MK3 ||
      hw_info.model == SOFTRF_MODEL_UNI       ||
      hw_info.model == SOFTRF_MODEL_BADGE     ||
      hw_info.model == SOFTRF_MODEL_LEGO)
  {
    // power on by wakeup call
    Serial_GNSS_Out.write((uint8_t) 0); GNSS_FLUSH(); delay(1000);
  }

#if !defined(EXCLUDE_GNSS_SONY)
  gnss_id = (gnss_id == GNSS_MODULE_NONE ?
              (gnss_chip = &sony_ops,         gnss_chip->probe()) : gnss_id);
#endif /* EXCLUDE_GNSS_SONY */

  if (gnss_id == GNSS_MODULE_NONE) {               // not Sony, listen for $G sentences
      gnss_id = generic_nmea_ops.probe();
      if (gnss_id == GNSS_MODULE_NONE) {           // no NMEA sentences seen
          Serial.println(F("No GNSS NMEA at 9600 baud"));
          delay(500);
          gnss_id = probe_baud_rates();       // try other baud rates, ending with 9600 
      }
  }

  if (gnss_id == GNSS_MODULE_NONE) {               // no NMEA sentences seen at any baud rate

#if !defined(EXCLUDE_GNSS_UBLOX) && defined(ENABLE_UBLOX_RFS)
    if (hw_info.model == SOFTRF_MODEL_PRIME_MK2 ||
        hw_info.model == SOFTRF_MODEL_PRIME_MK3) {

      byte version = ublox_version();

      if (version == GNSS_MODULE_U6 ||
          version == GNSS_MODULE_U7 ||
          version == GNSS_MODULE_U8) {   // UBX connection at 9600 baud did work

        gnss_chip = &ublox_ops;

        Serial.println(F("WARNING: UBLOX GNSS detected but no NMEA!"));
        gnss_id = generic_nmea_ops.probe();    // try one more time, following UBX response
        if (gnss_id == GNSS_MODULE_NMEA) {

            gnss_id = (gnss_id_t) version;     // got NMEA on the third time
            Serial.println(F("... OK after ublox_version()"));
            // and fall through to probe() call below

        } else {

          Serial.print(F("Reset UBLOX GNSS to factory default config: "));

          //(void) ublox_factory_reset(true);
          (void) ublox_factory_reset(false);    // reset config, but do a warmstart

          gnss_id = generic_nmea_ops.probe();    // try again listening for NMEA
          if (gnss_id == GNSS_MODULE_NONE)
              gnss_id = probe_baud_rates();

          if (gnss_id == GNSS_MODULE_NONE) {
            Serial.println(F("FAILURE"));
            return (byte) gnss_id;
          }

          gnss_id = (gnss_id_t) version;     // we know it is a UBLOX
          Serial.println(F("SUCCESS"));      // and fall through to probe() call below
        }

      } else {
          Serial.println(F("WARNING: no NMEA and Ublox GNSS not detected (at any baud rate)"));
          return (byte) gnss_id;    // GNSS_MODULE_NONE
      }

    } else   // not SOFTRF_MODEL_PRIME_MK*
#endif /* EXCLUDE_GNSS_UBLOX && ENABLE_UBLOX_RFS */

        return (byte) gnss_id;    // GNSS_MODULE_NONE
  }

#if !defined(EXCLUDE_GNSS_UBLOX)
  gnss_id = (gnss_id == GNSS_MODULE_NMEA ?
              (gnss_chip = &ublox_ops,  gnss_chip->probe()) : gnss_id);
#endif /* EXCLUDE_GNSS_UBLOX */
#if !defined(EXCLUDE_GNSS_MTK)
  gnss_id = (gnss_id == GNSS_MODULE_NMEA ?
              (gnss_chip = &mtk_ops,    gnss_chip->probe()) : gnss_id);
#endif /* EXCLUDE_GNSS_MTK */
#if !defined(EXCLUDE_GNSS_GOKE)
  gnss_id = (gnss_id == GNSS_MODULE_NMEA ?
              (gnss_chip = &goke_ops,   gnss_chip->probe()) : gnss_id);
#endif /* EXCLUDE_GNSS_GOKE */
#if !defined(EXCLUDE_GNSS_AT65)
  gnss_id = (gnss_id == GNSS_MODULE_NMEA ?
              (gnss_chip = &at65_ops,   gnss_chip->probe()) : gnss_id);
#endif /* EXCLUDE_GNSS_AT65 */

  Serial.print("GNSS type found: ");
  Serial.println(GNSS_name[gnss_id]);

  gnss_chip = (gnss_id == GNSS_MODULE_NMEA ? &generic_nmea_ops : gnss_chip);

  if (gnss_chip) gnss_chip->setup();

  uint8_t pps_pin = get_pps_pin();
  if (pps_pin != SOC_UNUSED_PIN
#if defined(ESP32)
   && ESP32_pin_reserved(pps_pin, false, "GNSS PPS") == false
#endif
   ) {
      //bool rising = true;
      //if (gnss_id == GNSS_MODULE_U7) {
      //    if (ublox_query(0x06, 0x31, 2000) == true) {       // CFG-TP5
      //        Serial.printf("U7 time pulse flags: %02X\r\n", GNSSbuf[28]);
      //        rising = ((GNSSbuf[28] & 0x40) != 0);
      //    }
      //}
      pinMode(pps_pin, INPUT_PULLDOWN);
#if !defined(NOT_AN_INTERRUPT)
      attachInterrupt(digitalPinToInterrupt(pps_pin), SoC->GNSS_PPS_handler, RISING);
//    attachInterrupt(digitalPinToInterrupt(pps_pin), SoC->GNSS_PPS_handler, (rising ? RISING : FALLING));
#else
      int interrupt_num = digitalPinToInterrupt(pps_pin);
      if (interrupt_num != NOT_AN_INTERRUPT)
        attachInterrupt(interrupt_num, SoC->GNSS_PPS_handler, RISING);
//      attachInterrupt(interrupt_num, SoC->GNSS_PPS_handler, (rising ? RISING : FALLING));
#endif
  } else {
      settings->ppswire = false;
  }

#if defined(USE_NMEALIB)
//Serial.println("GNSS_setup: using NMEALIB");
#else
//Serial.println("GNSS_setup: not using NMEALIB");
#endif

//#if defined(USE_NMEA_CFG)
  NMEA_Source = DEST_NONE;
//#endif /* USE_NMEA_CFG */

  return (byte) gnss_id;
}

void GNSS_loop()
{

#if defined(ESP32)
  if (gnss_needs_reset) {
      gnss_needs_reset = false;
      reset_gnss();
      Serial.println("Rebooting...");
      delay(1000);
      reboot();
  }
#endif

  PickGNSSFix();

  /*
   * Both GGA and RMC NMEA sentences are required.
   * No fix when any of them is missing or lost.
   * Valid date is critical for legacy protocol (only).
   */
  GNSS_fix_cache = gnss.location.isValid() && !badGGA    &&
                   gnss.altitude.isValid()               &&
                   gnss.date.isValid()                   &&
                  (gnss.location.age() <= NMEA_EXP_TIME) &&
                  (gnss.altitude.age() <= NMEA_EXP_TIME) &&
                  (gnss.date.age()     <= NMEA_EXP_TIME);

  if (gnss_chip) gnss_chip->loop();

  if (! GNSS_fix_cache)
      return;
  
  if (leap_valid != 1)
      (void) leap_seconds_valid();  // periodically check again

  GNSSTimeSync();
}

void GNSS_fini()
{
  uint8_t pps_pin = get_pps_pin();
  if (pps_pin != SOC_UNUSED_PIN) {
#if !defined(NOT_AN_INTERRUPT)
    detachInterrupt(digitalPinToInterrupt(pps_pin));
#else
    int interrupt_num = digitalPinToInterrupt(pps_pin);
    if (interrupt_num != NOT_AN_INTERRUPT) {
      detachInterrupt(interrupt_num);
    }
#endif

  }

  if (gnss_chip) gnss_chip->fini();
}

/*
 * Sync with GNSS time every 60 seconds
 */
void GNSSTimeSync()
{
  if ((GNSSTimeSyncMarker == 0 || (millis() - GNSSTimeSyncMarker > 60000)) &&
       gnss.time.isValid()                                                 &&
       gnss.time.isUpdated()                                               &&
       gnss.date.year() >= FW_Build_Year                                   &&
      (gnss.time.age() <= 1000) /* 1s */ ) {
#if 0
    Serial.print("Valid: ");
    Serial.println(gnss.time.isValid());
    Serial.print("isUpdated: ");
    Serial.println(gnss.time.isUpdated());
    Serial.print("age: ");
    Serial.println(gnss.time.age());
#endif
    setTime(gnss.time.hour(),
            gnss.time.minute(),
            gnss.time.second(),
            gnss.date.day(),
            gnss.date.month(),
            gnss.date.year());
    GNSSTimeSyncMarker = millis();

// piggy-back on this once-a-minute timer to show GNSS timing stats if collected
#if defined(ENABLE_GNSS_STATS)
    if (gnss_stats.gga_count > 100) {
        Serial.print("Average GGA ms after PPS: ");
        Serial.println(gnss_stats.gga_time_ms / gnss_stats.gga_count);
        gnss_stats.gga_time_ms = 0;
        gnss_stats.gga_count = 0;
    }
    if (gnss_stats.rmc_count > 100) {
        Serial.print("Average RMC ms after PPS: ");
        Serial.println(gnss_stats.rmc_time_ms / gnss_stats.rmc_count);
        gnss_stats.rmc_time_ms = 0;
        gnss_stats.rmc_count = 0;
    }
#endif
  }
}

int int2digits(const char *p)
{
    return (10*(p[0]-'0') + (p[1]-'0'));
}

static struct {
    uint32_t hms;
    uint32_t addr;
    float lat;
    float lon;
    float alt;
    float speed;
    float course;
    float vs;
    float turnrate;
    uint8_t addrtype;
    uint8_t actype;
    //uint8_t hour;
    //uint8_t minute;
    //uint8_t second;
    bool waiting;
} pfsim;

void add_pfsim_traffic()
{
    if (! pfsim.waiting)
        return;
    uint32_t hour   = (uint32_t) gnss.time.hour();
    uint32_t minute = (uint32_t) gnss.time.minute();
    uint32_t second = (uint32_t) gnss.time.second();
    uint32_t gnss_hms = (second + 60 * (minute + 60 * hour));
    if (pfsim.hms < gnss_hms) {
        pfsim.waiting = false;   // too late
        return;
    }
    if (pfsim.hms > gnss_hms)    // too early
        return;
    //fo = EmptyFO;
    EmptyFO(&fo);
    fo.protocol = RF_PROTOCOL_LATEST;
    fo.addr_type = pfsim.addrtype;
    fo.timestamp = OurTime;
    fo.addr = pfsim.addr;
    fo.latitude = pfsim.lat;
    fo.longitude = pfsim.lon;
    fo.altitude = pfsim.alt;   // was  - ThisAircraft.geoid_separation;
    fo.course = pfsim.course;
    fo.speed = (1.0 / _GPS_MPS_PER_KNOT) * pfsim.speed;
    fo.vs = pfsim.vs * (_GPS_FEET_PER_METER * 60.0);
    fo.gnsstime_ms = millis();
    fo.turnrate = pfsim.turnrate;
    fo.aircraft_type = pfsim.actype;
    fo.airborne = 1;
    fo.circling = (fo.turnrate < 10.0) ? -1 : ((fo.turnrate > 10.0) ? 1 : 0);
    fo.tx_type = TX_TYPE_FLARM;
    AddTraffic(&fo, "SIM");
    pfsim.waiting = false;
}

void process_pfsim_sentence()
{
    if (! (settings->debug_flags & DEBUG_SIMULATE))
        return;
    if (! P_timestamp.isUpdated())
        return;
    const char *p = P_timestamp.value();
    uint32_t hour   = int2digits(&p[0]);
    uint32_t minute = int2digits(&p[2]);
    uint32_t second = int2digits(&p[4]);
    pfsim.hms = (second + 60 * (minute + 60 * hour));
    pfsim.addr = strtol(P_addr.value(),NULL,16);
    pfsim.lat = atof(P_lat.value());
    pfsim.lon = atof(P_lon.value());
    pfsim.alt = atof(P_alt.value());
    pfsim.speed = atof(P_speed.value());
    pfsim.course = atof(P_course.value());
    pfsim.vs = atof(P_vs.value());
    pfsim.turnrate = atof(P_turnrate.value());
    pfsim.addrtype = atoi(P_addrtype.value());
    pfsim.actype = atoi(P_actype.value());
    pfsim.waiting = true;
    add_pfsim_traffic();   // will turn pfsim.waiting off unless too early
}

// determine in one place (here) when a "new fix" is obtained
// - no longer need to handle this in SoftRF.ino and in Time.cpp
uint8_t Try_GNSS_sentence() {

    static uint32_t prev_fix_ms = 0;
    static uint32_t new_gga_ms  = 0;
    static uint32_t new_rmc_ms  = 0;
    static int ndx = sizeof(GNSSbuf)-2;
    static char old_sec = '\0';

    char c = GNSSbuf[GNSS_cnt];
    if (c == '$')
        ndx = GNSS_cnt;
    if (gnss.encode(c) == false)
        return 0;
    if (GNSS_cnt < ndx+6)
        return 0;

    // if got here, gnss.encode said it is a valid sentence
    size_t write_size = GNSS_cnt;
    if (c=='\r' || c=='\n') {
        --write_size;
        //c = GNSSbuf[write_size];
        //if (c=='\r' || c=='\n')
        //    --write_size;
    }
    write_size = write_size - ndx + 1;    // \r\n not included
    char *gb = (char *) &GNSSbuf[ndx];
    ndx = sizeof(GNSSbuf)-2;             // anticipating next sentence

    bool is_g = (gb[1]=='G');
    bool is_p = (gb[1]=='P');
    if (!is_g && !is_p)
        return 0;        // neither GNSS sentence nor SoftRF sentence
    if (is_p && gb[2]=='G')
        return 0;        // ignore $PGRMZ
    if (gb[6] != ',')
        return 0;        // not $GPGGA, $GPRMC, etc nor $PFSIM, $PSRF* or $PSKVC

    if (is_p) {
        if (gb[2]=='S' && ((gb[3]=='R' && gb[4]=='F') || (gb[3]=='K' && gb[4]=='V'))) {
            NMEA_Process_SRF_SKV_Sentences();
            GNSSbuf[GNSS_cnt+1] = '\0';
            Serial.println(gb);
            return 2;
        }
        if (gb[2]=='F' && gb[3]=='S' && gb[4]=='I' && gb[5]=='M') {
            process_pfsim_sentence();
            GNSSbuf[GNSS_cnt+1] = '\0';
            Serial.println(gb);
            return 2;
        }
    }

    bool is_gga = (is_g && gb[3]=='G' && gb[4]=='G' && gb[5]=='A');
    bool is_rmc = false;
    if (! is_gga)
      is_rmc = (is_g && gb[3]=='R' && gb[4]=='M' && gb[5]=='C');
    uint32_t now_ms = millis();
    if (now_ms > prev_fix_ms + 600) {           // expect one fix per second
      gnss_new_fix = gnss_new_time = false;     // withdraw what was not consumed
      if ((is_gga && new_gga_ms != 0) || (is_rmc && new_rmc_ms != 0)
                 || (latest_Commit_Time != 0 && now_ms > latest_Commit_Time + 600)) {
          // other sentence failed to arrive within same second - start over
          latest_Commit_Time = 0;
          new_gga_ms  = 0;
          new_rmc_ms  = 0;
      }
      if (is_gga && gb[12] != old_sec) {   // extra check that it's a new second
          new_gga_ms = now_ms;
          old_sec = gb[12];
          if (! latest_Commit_Time) {
              latest_Commit_Time = now_ms - gnss.time.age();  // for use by Time_loop()
              // age() should be small since we just now did gnss.encode().
              gnss_time_from_rmc = false;    // GGA arrived before RMC
          }
          if (write_size > 40) {
              badGGA = false;
              strncpy(GPGGA_Copy, gb, write_size);  // for traffic alarm logging
              GPGGA_Copy[write_size] = '\0';
          } else {
              badGGA = true;
              GNSS_fix_cache = false;
              GPGGA_Copy[7] = '\0';
          }
      }
      if (is_rmc) {
          new_rmc_ms = now_ms;
          if (! latest_Commit_Time) {
              latest_Commit_Time = now_ms - gnss.time.age();
              gnss_time_from_rmc = true;     // RMC arrived before GGA
          }
      }
      if (new_gga_ms && new_rmc_ms) {   // received both GGA & RMC sentences
#if defined(ENABLE_GNSS_STATS)
          uint32_t pps = ref_time_ms;   // maintained in Time.cpp
          uint32_t when;
          if (new_gga_ms > pps) {
              when = new_gga_ms - pps;
              if (when > 1000)   when -= 1000;
          } else {
              when = pps - new_gga_ms;
              if (when > 1000)   when -= 1000;
              when = 1000 - when;
          }
          if (when < 1000) {
              gnss_stats.gga_time_ms += when;
              gnss_stats.gga_count++;
          }
          if (new_rmc_ms > pps) {
              when = new_rmc_ms - pps;
              if (when > 1000)   when -= 1000;
          } else {
              when = pps - new_rmc_ms;
              if (when > 1000)   when -= 1000;
              when = 1000 - when;
          }
          if (when < 1000) {
              gnss_stats.rmc_time_ms += when;
              gnss_stats.rmc_count++;
          }
#endif /* ENABLE_GNSS_STATS */
          new_gga_ms  = 0;              // reset markers
          new_rmc_ms  = 0;
          prev_fix_ms = now_ms;         // for timing the wait for next fix
          gnss_new_fix  = true;         // flags for external consumption
          gnss_new_time = true;         // will be reset to false when consumed
      }
    }
    // get here even if within same second, to output GSV to NMEA destinations

    if (settings->nmea_g == 0 && settings->nmea2_g == 0)
        return 1;

    uint16_t nmeatype = NMEA_G_OTHER;
    if (is_gga || is_rmc) {
        nmeatype = NMEA_G;
    } else if (is_g) {
        if (((settings->nmea_g | settings->nmea2_g) & NMEA_G_NONBASIC) == 0)
            return 1;
        if (is_g && gb[3]=='G' && gb[4]=='S') {
            if (gb[5]=='A')  nmeatype = NMEA_G_GSA;
            else
            if (gb[5]=='T')  nmeatype = NMEA_G_GST;
            else
            if (gb[5]=='V')  nmeatype = NMEA_G_GSV;
        }
    }

    /*
     * Work around issue with "always 0.0,M" GGA geoid separation value
     * given by some Chinese GNSS chipsets
     */
#if defined(USE_NMEALIB)
    if (is_gga && gnss.separation.meters() == 0.0) {
        NMEA_GGA();
            // GGA is output either indirectly (via NMEA_GGA()) or directly (below).
            // Observed on a T-Beam: NMEA_GGA() while no fix, then direct.
    }
    else
#endif
    {
        if (is_g && settings->gn_to_gp)   // convert $GN, $GA, $GL to $GP if needed
            gb[2] = 'P';
        NMEA_Outs(nmeatype, gb, write_size, true);
    }
    return 1;
}

void PickGNSSFix()
{
  uint8_t c = 0;

  if (is_prime_mk2) {

    if (settings->debug_flags & DEBUG_SIMULATE) {
      static uint8_t SentenceType = 0;
#if defined(USE_SD_CARD)
      if (TARGETfileOpen && GNSS_cnt == 0) {
        add_pfsim_traffic();   // if any waiting and its time has arrived
        while (! pfsim.waiting) {
          if (!TARGETfile.available()) {
            GNSS_cnt = 0;
            break;
          }
          c = TARGETfile.read();
          GNSSbuf[GNSS_cnt] = c;
          SentenceType = Try_GNSS_sentence();
          if (c=='\n' || GNSS_cnt == sizeof(GNSSbuf)-1) {
            GNSS_cnt = 0;
            if (SentenceType)   // got valid sentence
                return;         // allow main loop to report, will get back here later
          } else {
            GNSS_cnt++;
            yield();
          }
        }
      }
#endif
      static uint8_t d = 0;
      static bool pfsim_sentence = false;
      // read simulated GNSS or PFSIM sentences from either a file or the serial port
      static uint32_t burst_start = 0;
      static uint32_t next_burst = 40200;   // start sim running 40s after boot
//Serial.printf("PickGNSSFix(): millis %d next_burst %d\r\n", millis(), next_burst);
      bool ext_gnss = (settings->gnss_pins != EXT_GNSS_NONE);
      while (true) {
#if defined(USE_SD_CARD)
        if (SIMfileOpen) {
          if (millis() < next_burst)        // ignore input until next simulated second
              return;
          if (!SIMfile.available())
              return;
          c = SIMfile.read();
        } else
#endif
        {
          // let external source decide timing
          if (ext_gnss) {                             // read from external GNSS port
              if (Serial_GNSS_In.available() <= 0)
                  return;
              c = Serial_GNSS_In.read();
          } else {                                    // read from USB port
              if (SerialOutput.available() <= 0)
                  return;
              c = SerialOutput.read();
          }
        }
        if (GNSS_cnt == 0) {
          SentenceType = 0;
          if (c == '.' || c == '\r' || c == '\n') {
            // a blank line or starting with '.' means: wait for next second
            if (burst_start != 0) {
                next_burst = ref_time_ms + 1100;  // 100 ms after next simulated PPS
                burst_start = 0;
            } else if (c == '.') {
                next_burst += 1000;               // ".." for a 2-sec delay, etc
            } else if (c == '\n' && d == '\n') {
                    next_burst += 1000;           // 2 blank lines for a 2-sec delay, etc
            }
            // '\r' ignored unless burst_start != 0
            if (c != '\r')
                d = c;
            return;
          }
          // else if (c == 'B') process IGC B-record as sim input ?
        }
        d = 0;
        if (isPrintable(c) || c == '\r' || c == '\n') {
          if (burst_start == 0)
              burst_start = millis();
          GNSSbuf[GNSS_cnt] = c;
          uint8_t rval = Try_GNSS_sentence();
          if (rval != 0)
              SentenceType = rval;
          //if (SentenceType) {
          //    GNSSbuf[GNSS_cnt+1] = '\n';
          //    GNSSbuf[GNSS_cnt+2] = '\0';
          //    Serial.print("sim> ");
          //    Serial.println((const char *) GNSSbuf);
          //}
          if (c=='\n' || GNSS_cnt == sizeof(GNSSbuf)-1) {
            GNSS_cnt = 0;
            if (SentenceType)   // got valid sentence
                return;              // break to allow main loop to run
          } else {
            GNSS_cnt++;
            yield();
          }
        }
      }        // infinite loop unless !available() above, or pausing, or SentenceType == 2
      return;
    }              // end of if (settings->debug_flags & DEBUG_SIMULATE)

    // only use the internal (or add-on serial) GNSS, leave other ports alone for data bridging
    while (true) {
      if (Serial_GNSS_In.available() > 0) {
       c = Serial_GNSS_In.read();
      } else {
        /* return back if no input data */
        return;
      }
      if (c == -1) {
        /* retry */
        continue;
      }
      if (isPrintable(c) || c == '\r' || c == '\n') {
        GNSSbuf[GNSS_cnt] = c;
      } else {
        /* ignore */
        continue;
      }
      NMEA_Source = DEST_NONE;
      (void) Try_GNSS_sentence();
      //if (GNSSbuf[GNSS_cnt] == '\n' || GNSS_cnt == sizeof(GNSSbuf)-1) {
      if (c == '\r' || c == '\n' || GNSS_cnt == sizeof(GNSSbuf)-1) {
        GNSS_cnt = 0;
      } else {
        GNSS_cnt++;
        yield();
      }
    }        // infinite loop unless !Serial_GNSS_In.available() above

    return;
  }

  // other models:

  /*
   * Check SW/HW UARTs, USB and BT for data
   * WARNING! Make use only one input source at a time.
   */
   // - note there is nothing here to stop interleaving sentences from different sources!
  while (true) {
#if !defined(USE_NMEA_CFG)
    if (Serial_GNSS_In.available() > 0) {
      c = Serial_GNSS_In.read();
    } else if (Serial.available() > 0) {
      c = Serial.read();
    } else if (SoC->Bluetooth_ops && SoC->Bluetooth_ops->available() > 0) {
      c = SoC->Bluetooth_ops->read();
    } else {
      /* return back if no input data */
      break;
    }
      /*
       * Don't forget to disable echo:
       *
       * stty raw -echo -F /dev/rfcomm0
       *
       * GNSS input becomes garbled otherwise
       */

      // Serial.write((char) c);
      /* Ignore Bluetooth input for a while */
      // break;
#else
    /*
     * Give priority to control channels over default GNSS input source on
     * 'Dongle', 'Retro', 'Uni', 'Mini', 'Badge', 'Academy' and 'Lego' Editions
     */

    /* Bluetooth input is first */
    if (SoC->Bluetooth_ops && SoC->Bluetooth_ops->available() > 0) {
      c = SoC->Bluetooth_ops->read();

      NMEA_Source = DEST_BLUETOOTH;

    /* USB input is second */
    } else if (SoC->USB_ops && SoC->USB_ops->available() > 0) {
      c = SoC->USB_ops->read();

      NMEA_Source = DEST_USB;

#if defined(ARDUINO_NUCLEO_L073RZ)
      /* This makes possible to configure S76x's built-in SONY GNSS from aside */
      if (hw_info.model == SOFTRF_MODEL_DONGLE) {
        Serial_GNSS_Out.write(c);
      }
#endif

    /* Serial input is third */
    } else if (SerialOutput.available() > 0) {
      c = SerialOutput.read();

      NMEA_Source = DEST_UART;

#if 0
      /* This makes possible to configure HTCC-AB02S built-in GOKE GNSS from aside */
      if (hw_info.model == SOFTRF_MODEL_MINI) {
        Serial_GNSS_Out.write(c);
      }
#endif

    /* Built-in GNSS input */
    } else if (Serial_GNSS_In.available() > 0) {
      c = Serial_GNSS_In.read();
      NMEA_Source = DEST_NONE;
    } else {
      /* return back if no input data */
      break;
    }
#endif /* USE_NMEA_CFG */

    if (c == -1) {
      /* retry */
      continue;
    }

    if (isPrintable(c) || c == '\r' || c == '\n') {
      GNSSbuf[GNSS_cnt] = c;
    } else {
      /* ignore */
      continue;
    }

#if defined(ENABLE_GNSS_STATS2)
    if ( (GNSS_cnt >= 5) &&
         (GNSSbuf[GNSS_cnt-5] == '$') &&
         (GNSSbuf[GNSS_cnt-4] == 'G') &&
        ((GNSSbuf[GNSS_cnt-3] == 'P') || (GNSSbuf[GNSS_cnt-3] == 'N'))) {
      if ( (GNSSbuf[GNSS_cnt-2] == 'G') &&
           (GNSSbuf[GNSS_cnt-1] == 'G') &&
           (GNSSbuf[GNSS_cnt-0] == 'A')) {
        gnss_stats.gga_time_ms = millis();
        gnss_stats.gga_count++;
      } else if ( (GNSSbuf[GNSS_cnt-2] == 'R') &&
                  (GNSSbuf[GNSS_cnt-1] == 'M') &&
                  (GNSSbuf[GNSS_cnt-0] == 'C')) {
        gnss_stats.rmc_time_ms = millis();
        gnss_stats.rmc_count++;
      }
    }
#endif /* ENABLE_GNSS_STATS2 */

    if (Try_GNSS_sentence() == 1) {
#if defined(USE_NMEA_CFG)
      //if (GNSSbuf[1]!='G')
      if (GNSSbuf[1]=='P')
          NMEA_Process_SRF_SKV_Sentences();
          // if it was a valid sentence but not a GNSS sentence
#endif /* USE_NMEA_CFG */
    }

#if defined(ENABLE_D1090_INPUT)
    if (GNSSbuf[GNSS_cnt]   == '\n' &&
        GNSS_cnt             >  1   &&
        GNSSbuf[GNSS_cnt-1] == '\r' &&
        GNSSbuf[GNSS_cnt-2] == ';')
    {
      int i=0;

      if (GNSS_cnt > 16 && GNSSbuf[GNSS_cnt-17] == '*') {
        for (i=0; i<14; i++) {
          if (!isxdigit(GNSSbuf[GNSS_cnt-16+i])) break;
        }
        if (i>=14) {
          D1090_Import(&GNSSbuf[GNSS_cnt-17]);
          GNSS_cnt -= 18;
        }
      } else if (GNSS_cnt > 30 && GNSSbuf[GNSS_cnt-31] == '*') {
        for (i=0; i<28; i++) {
          if (!isxdigit(GNSSbuf[GNSS_cnt-30+i])) break;
        }
        if (i>=28) {
          D1090_Import(&GNSSbuf[GNSS_cnt-31]);
          GNSS_cnt -= 32;
        }
      }
    }
#endif /* ENABLE_D1090_INPUT */

    //if (GNSSbuf[GNSS_cnt] == '\n' || GNSS_cnt == sizeof(GNSSbuf)-1) {
    if (c == '\r' || c == '\n' || GNSS_cnt == sizeof(GNSSbuf)-1) {
      GNSS_cnt = 0;
    } else {
      GNSS_cnt++;
      yield();
    }
  }    // end of while()

}

#if defined(USE_EGM96)
/*
 *  Algorithm of EGM96 geoid offset approximation was taken from XCSoar
 */
static float AsBearing(float angle)
{
  float retval = angle;

  while (retval < 0)
    retval += 360.0;

  while (retval >= 360.0)
    retval -= 360.0;

  return retval;
}

void LookupSeparation(float lat, float lon)
{
#if defined(FILESYS)
  if (! FS_is_mounted) {
      Serial.println(F("File system is not mounted"));
      return;
  }

  int ilat, ilon;

  ilat = round((90.0 - lat) / 2.0);
  ilon = round(AsBearing(lon) / 2.0);

  int offset = ilat * 180 + ilon;

  if (offset >= 90*180)
    return;

  if (offset < 0)
    return;

  int retval = -1;
  File egm96file = FILESYS.open("/egm96s.dem", "r");
  if (!egm96file) {
      Serial.println(F("File egm96s.dem not found"));
      return;
  } else {
      if (! egm96file.seek(offset, SeekSet))
          Serial.println(F("Failed to seek to offset in egm96s.dem"));
      else
          retval = (int) egm96file.read();
      egm96file.close();
  }
  if (retval < 0)   // not read from file
      return;
  retval -= 127;
  if (retval < -120 || retval > 120) {
      Serial.print(F("LookupSeparation(): invalid value"));
      return;
  }
  geo_sep_from_file = (retval? (float) retval : 0.1);    // exactly zero means n.a.
  Serial.print(F("LookupSeparation() retrieved: "));
  Serial.print(retval);
  Serial.print(F(" from offset: "));
  Serial.println(offset);
#endif
}

float EGM96GeoidSeparation()
{
    if (settings->geoid != 0)                  // zero means n.a.
        return (float) settings->geoid;
    static uint32_t when_loaded = 0;
    if ((when_loaded == 0 || millis() > when_loaded + 1200000)    // every 20 minutes
             && ThisAircraft.latitude != 0.0 && isValidGNSSFix()) {
        LookupSeparation (ThisAircraft.latitude, ThisAircraft.longitude);
        when_loaded = millis();
    }
    return geo_sep_from_file;
}

#else
float EGM96GeoidSeparation()
{
    float rval = (float) settings->geoid;
    if (settings->geoid == 0)
        rval += 0.1;                   // zero means n.a.
    return rval;
}
#endif /* USE_EGM96 */
