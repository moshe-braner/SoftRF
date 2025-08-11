/*
 * SoftRF(.ino) firmware
 * Copyright (C) 2016-2021 Linar Yusupov
 *
 * Author: Linar Yusupov, linar.r.yusupov@gmail.com
 *
 * Web: http://github.com/lyusupov/SoftRF
 *
 * Credits:
 *   Arduino core for ESP8266 is developed/supported by ESP8266 Community (support-esp8266@esp8266.com)
 *   AVR/Arduino nRF905 Librarrfy/Driver is developed by Zak Kemble, contact@zakkemble.co.uk
 *   flarm_decode is developed by Stanislaw Pusep, http://github.com/creaktive
 *   Arduino Time Library is developed by Paul Stoffregen, http://github.com/PaulStoffregen
 *   "Aircraft" and MAVLink Libraries are developed by Andy Little
 *   TinyGPS++ and PString Libraries are developed by Mikal Hart
 *   Adafruit NeoPixel Library is developed by Phil Burgess, Michael Miller and others
 *   TrueRandom Library is developed by Peter Knight
 *   IBM LMIC and Semtech Basic MAC frameworks for Arduino are maintained by Matthijs Kooijman
 *   ESP8266FtpServer is developed by David Paiva
 *   Lib_crc is developed by Lammert Bies
 *   OGN library is developed by Pawel Jalocha
 *   NMEA library is developed by Timur Sinitsyn, Tobias Simon, Ferry Huberts
 *   ADS-B encoder C++ library is developed by yangbinbin (yangbinbin_ytu@163.com)
 *   Arduino Core for ESP32 is developed by Hristo Gochkov
 *   ESP32 BT SPP library is developed by Evandro Copercini
 *   Adafruit BMP085 library is developed by Limor Fried and Ladyada
 *   Adafruit BMP280 library is developed by Kevin Townsend
 *   Adafruit MPL3115A2 library is developed by Limor Fried and Kevin Townsend
 *   U8g2 monochrome LCD, OLED and eInk library is developed by Oliver Kraus
 *   NeoPixelBus library is developed by Michael Miller
 *   jQuery library is developed by JS Foundation
 *   EGM96 data is developed by XCSoar team
 *   BCM2835 C library is developed by Mike McCauley
 *   SimpleNetwork library is developed by Dario Longobardi
 *   ArduinoJson library is developed by Benoit Blanchon
 *   Flashrom library is part of the flashrom.org project
 *   Arduino Core for TI CC13X0 and CC13X2 is developed by Robert Wessels
 *   EasyLink library is developed by Robert Wessels and Tony Cave
 *   Dump978 library is developed by Oliver Jowett
 *   FEC library is developed by Phil Karn
 *   AXP202X library is developed by Lewis He
 *   Arduino Core for STM32 is developed by Frederic Pillon
 *   TFT library is developed by Bodmer
 *   STM32duino Low Power and RTC libraries are developed by Wi6Labs
 *   Basic MAC library is developed by Michael Kuyper
 *   LowPowerLab SPIFlash library is maintained by Felix Rusu
 *   Arduino core for ASR650x is developed by Aaron Lee (HelTec Automation)
 *   ADXL362 library is developed by Anne Mahaffey
 *   Arduino Core for nRF52 and TinyUSB library are developed by Ha Thach
 *   Arduino-NVM library is developed by Frank Holtz
 *   AceButton library is developed by Brian Park
 *   GxEPD2 library is developed by Jean-Marc Zingg
 *   Adafruit GFX library is developed by Adafruit Industries
 *   U8g2 fonts for Adafruit GFX are developed by Oliver Kraus
 *   Adafruit SPIFlash and SleepyDog libraries are developed by Adafruit Industries
 *   SdFat library is developed by Bill Greiman
 *   Arduino MIDI library is developed by Francois Best (Forty Seven Effects)
 *   Arduino uCDB library is developed by Ioulianos Kakoulidis
 *   Wind estimation and collision algorithm for circling aircraft by Moshe Braner
 *   Optimized ADS-B decoding by Moshe Braner
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

#include <math.h>

#include "src/system/SoC.h"
#include "src/system/OTA.h"
#include "src/system/Time.h"
#include "src/driver/LED.h"
#include "src/driver/GNSS.h"
#include "src/driver/RF.h"
#include "src/driver/Buzzer.h"
#include "src/driver/Strobe.h"
#include "src/driver/Settings.h"
#include "src/driver/Battery.h"
#include "src/driver/Filesys.h"
#include "src/protocol/data/MAVLink.h"
#include "src/protocol/data/GDL90.h"
#include "src/protocol/data/GNS5892.h"
#include "src/protocol/data/NMEA.h"
#include "src/protocol/data/IGC.h"
#include "src/protocol/data/D1090.h"
#include "src/driver/WiFi.h"
#include "src/ui/Web.h"
#include "src/driver/Baro.h"
#include "src/TTNHelper.h"
#include "src/TrafficHelper.h"
#include "src/Wind.h"

#if !defined(EXCLUDE_VOICE)
#if defined(ESP32)
#include "src/driver/Voice.h"
#endif
#endif

#if defined(ENABLE_AHRS)
#include "src/driver/AHRS.h"
#endif /* ENABLE_AHRS */

#if LOGGER_IS_ENABLED
#include "src/system/Log.h"
#endif /* LOGGER_IS_ENABLED */

#define DEBUG 0
#define DEBUG_TIMING 0

#define isTimeToDisplay() (millis() > LEDTimeMarker    + 1000)
#define isTimeToExport()  (millis() > ExportTimeMarker + 1000)

container_t ThisAircraft;

hardware_info_t hw_info = {
  .model    = DEFAULT_SOFTRF_MODEL,
  .revision = 0,
  .soc      = SOC_NONE,
  .rf       = RF_IC_NONE,
  .gnss     = GNSS_MODULE_NONE,
  .baro     = BARO_MODULE_NONE,
  .display  = DISPLAY_NONE,
  .storage  = STORAGE_NONE,
  .rtc      = RTC_NONE,
  .imu      = IMU_NONE
};

uint32_t LEDTimeMarker = 0;
uint32_t ExportTimeMarker = 0;
uint32_t GNSSTimeMarker = 0;
uint32_t SetupTimeMarker = 0;
uint32_t IGCTimeMarker = 0;

void setup()
{
  rst_info *resetInfo;

  hw_info.soc = SoC_setup(); // Has to be very first procedure in the execution order

  // ESP32_setup() now initializes the buzzer and strobe pins to avoid early output

  resetInfo = (rst_info *) SoC->getResetInfoPtr();

  // Serial.begin(SERIAL_OUT_BR, SERIAL_OUT_BITS);   // already done in SOC setup

#if defined(USBD_USE_CDC) && !defined(DISABLE_GENERIC_SERIALUSB)
  /* Let host's USB and console drivers to warm-up */
  delay(2000);
#elif defined(USE_TINYUSB) && defined(USBCON)
  for (int i=0; i < 20; i++) {if (Serial) break; else delay(100);}
#endif

#if LOGGER_IS_ENABLED
  Logger_setup();
#endif /* LOGGER_IS_ENABLED */

  Serial.println();
  Serial.print(F(SOFTRF_IDENT));
  Serial.print(" ");
  Serial.print(SoC->name);
  Serial.print(F(" FW.REV: " SOFTRF_FIRMWARE_VERSION " DEV.ID: "));
  //Serial.println(String(SoC->getChipId(), HEX));
  Serial.println(SoC->getChipId(), HEX);
  Serial.println(F("Copyright (C) 2015-2021 Linar Yusupov. All rights reserved."));
  Serial.println(F("This fork by Moshe Braner, 2024"));
  Serial.flush();

  if (resetInfo) {
    Serial.println(""); Serial.print(F("Reset reason: ")); Serial.println(resetInfo->reason);
  }
  Serial.println(SoC->getResetReason());
  Serial.print(F("Free heap size: ")); Serial.println(SoC->getFreeHeap());
  Serial.println(SoC->getResetInfo()); Serial.println("");
#if defined(ESP32)
  Serial.print(F("Memory available in PSRAM before setup: "));
  Serial.println(ESP.getFreePsram());
#endif

  // Settings_setup() needs to be done after Serial is working
  Settings_setup();

  // can only do the setup()s below after Settings_setup(), to know the settings,

  uint32_t SerialBaud = baudrates[settings->baud_rate];
  if (SerialBaud == 0)    // BAUD_DEFAULT
    SerialBaud = SERIAL_OUT_BR;
  //if (settings->mode == SOFTRF_MODE_GPSBRIDGE)
  //  SerialBaud = 9600;
#if defined(ESP32)
  if (SerialBaud != SERIAL_OUT_BR || settings->altpin0) {
    if (settings->altpin0) {
      if (ESP32_pin_reserved(Serial0AltRxPin, false, "Alt RX")) {
          settings->altpin0 = false;
      } else {
          Serial.print("Switching RX pin to ");
          Serial.println(Serial0AltRxPin);
      }
    }
    if (SerialBaud != SERIAL_OUT_BR) {
      Serial.print("Switching baud rate to ");
      Serial.println(SerialBaud);
    }
    delay(500);
    Serial.end();
    delay(1500);
    if (settings->altpin0)
      Serial.begin(SerialBaud, SERIAL_OUT_BITS, Serial0AltRxPin);
    else
      Serial.begin(SerialBaud, SERIAL_OUT_BITS);
  }
  Serial.setRxBufferSize(SerialBufSize);
#else
  if (SerialBaud != SERIAL_OUT_BR) {
    if (SerialBaud != SERIAL_OUT_BR) {
      Serial.print("Switching baud rate to ");
      Serial.println(SerialBaud);
    }
    delay(500);
    Serial.end();
    delay(1500);
    Serial.begin(SerialBaud, SERIAL_OUT_BITS);
  }
#endif

  if (settings->stealth
        || settings->id_method == ADDR_TYPE_RANDOM
        || settings->id_method == ADDR_TYPE_ANONYMOUS) {
    ThisAircraft.addr = 0;  /* will be filled in later */
  } else if (settings->id_method == ADDR_TYPE_ICAO && settings->aircraft_id != 0) {
    ThisAircraft.addr = settings->aircraft_id;
  } else if (settings->id_method == ADDR_TYPE_OVERRIDE && settings->aircraft_id != 0) {
    ThisAircraft.addr = settings->aircraft_id;
  } else {
    uint32_t id = SoC->getChipId() & 0x00FFFFFF;
    ThisAircraft.addr = id;
  }
Serial.printf("\r\nID_method: %d, settings_ID: %06X, used_ID: %06X\r\n\r\n",
settings->id_method, settings->aircraft_id, ThisAircraft.addr);

  SoC->Button_setup();

  // do this before Baro_setup - Wire.begin() happens there
  hw_info.display = SoC->Display_setup();

#if defined(ESP32)
  ESP32_charge_mode();  // if both battery & USB power, shut down and just charge
#endif

Serial.println(F("calling Baro_setup()..."));
  // do this before Filesys_setup since this tickles pins 13,2
  if (! (settings->debug_flags & DEBUG_SIMULATE))
      hw_info.baro = Baro_setup();
Serial.println(F("... Baro_setup() returned"));

//#if defined(USE_SD_CARD)
  //if (hw_info.model == SOFTRF_MODEL_PRIME_MK2) {
    // do this before RF_setup() to make sure that:
    // - SD card get started without interference
    // - radio chip sets SPI the way it wants it
    Filesys_setup();
    delay(200);
#if defined(ESP32)
Serial.print("Memory available in PSRAM before FlightLog_setup(): ");
Serial.println(ESP.getFreePsram());
#endif
    if (settings->logflight != FLIGHT_LOG_NONE)
        FlightLog_setup();
  //}
#if defined(ESP32)
Serial.print("Memory available in PSRAM after FlightLog_setup(): ");
Serial.println(ESP.getFreePsram());
#endif
//#endif

  hw_info.rf = RF_setup();
  delay(100);

  Buzzer_setup();
  Strobe_setup();

#if defined(ENABLE_AHRS)
  hw_info.imu = AHRS_setup();
#endif /* ENABLE_AHRS */

#if !defined(EXCLUDE_MAVLINK)
  if (settings->mode == SOFTRF_MODE_UAV) {
    Serial.begin(57600);
    MAVLink_setup();
    ThisAircraft.aircraft_type = AIRCRAFT_TYPE_UAV;  
  }  else
#endif /* EXCLUDE_MAVLINK */
  {
    hw_info.gnss = GNSS_setup();
    ThisAircraft.aircraft_type = settings->acft_type;
  }
  ThisAircraft.protocol = settings->rf_protocol;
  ThisAircraft.stealth  = settings->stealth;
  ThisAircraft.no_track = settings->no_track;

  Battery_setup();
  Traffic_setup();

  SoC->swSer_enableRx(false);

  WiFi_setup();

  if (SoC->USB_ops) {
     SoC->USB_ops->setup();
  }

  if (SoC->Bluetooth_ops) {
     SoC->Bluetooth_ops->setup();
  }

  OTA_setup();
  Web_setup();
  NMEA_setup();

#if defined(ESP32)
  if (settings->rx1090 == ADSB_RX_GNS5892)
      gns5892_setup();
#endif

  LED_setup();   // moved here to allow Serial2 to grab pin 4

#if defined(ENABLE_TTN)
  TTN_setup();
#endif

  delay(1000);

  /* expedite restart on WDT reset */
  if (resetInfo->reason != REASON_WDT_RST) {
    LED_test();
  }

#if !defined(EXCLUDE_VOICE)
#if defined(ESP32)
  Voice_setup();
  Voice_test(resetInfo->reason);
#endif
#endif

  switch (settings->mode)
  {
  case SOFTRF_MODE_TXRX_TEST:
  case SOFTRF_MODE_WATCHOUT:
    Time_setup();
    break;
  case SOFTRF_MODE_BRIDGE:
    break;
  case SOFTRF_MODE_NORMAL:
  case SOFTRF_MODE_UAV:
  default:
    SoC->swSer_enableRx(true);
    break;
  }

//Serial.println("calling Buzzer_test()");
  SoC->Buzzer_test(resetInfo->reason);

#if defined(USE_SD_CARD)
  MD5_test();
#endif

  SoC->post_init();

  SoC->WDT_setup();

//Serial.print("hw_info.model=");
//Serial.print(hw_info.model);
//Serial.print(", revision=");
//Serial.println(hw_info.revision);
Serial.print("reset.reason=");
Serial.println(resetInfo->reason);
Serial.print("reset_reason=");
Serial.println(SoC->getResetReason());
Serial.print("reset_info=");
Serial.println(SoC->getResetInfo());
#if defined(ESP32)
Serial.print("Memory available in PSRAM after setup: ");
Serial.println(ESP.getFreePsram());
#endif
Serial.print("settings_used=");
Serial.println(settings_used);

#if defined(ARDUINO_ARCH_NRF52)
    // check if any .IGX files need decompression
    //Serial.println("FlightLog_decomp()...");
    FlightLog_decomp();
#endif

  SetupTimeMarker = millis();

Serial.println("\r\n... setup() done\r\n");
Serial.println("Current settings:");
show_settings_serial();
Serial.println("");
}

void shutparts()
{
  SoC->WDT_fini();
  if (SoC->Bluetooth_ops)
     SoC->Bluetooth_ops->fini();
  closeFlightLog();
  if (AlarmLogOpen)
    AlarmLog.close();
#if defined(USE_SD_CARD)
  closeSDlog();
#endif
#if !defined(SERIAL_FLUSH)
#define SERIAL_FLUSH()       Serial.flush()
#endif
  SERIAL_FLUSH();
  SoC->swSer_enableRx(false);
#if defined(ESP32)
  Voice_fini();
  Buzzer_fini();
  Strobe_fini();
  Web_fini();
  NMEA_fini();
  WiFi_fini();
#endif
  RF_Shutdown();
  if (SoC->USB_ops)
     SoC->USB_ops->fini();
  if (settings->mode != SOFTRF_MODE_UAV)
    GNSS_fini();
  delay(1000);
}

void shutdown(int reason)
{
//Serial.println("shutdown()...");
  shutparts();
  SoC->Display_fini(reason);
  SoC->Button_fini();
  SoC_fini(reason);
}

void reboot()
{
  shutparts();
  SoC->reset();
}


#ifdef TIMETEST
static uint32_t lastsuccess = 0;
#endif

void normal()
{
  static bool firstfix = true;

  bool rx_tried   = false;
  bool rx_success = false;
  bool tx_success = false;

  Baro_loop();

#if defined(ENABLE_AHRS)
  AHRS_loop();
#endif /* ENABLE_AHRS */

  GNSS_loop();

  Time_loop();   /* this is where GNSS time data is processed for Legacy protocol */

  static uint32_t initial_time = 0;

  bool validfix = isValidFix();

  if (validfix) {

    static float prev_lat = 0;
    static float prev_lon = 0;
    if (firstfix) {
        if (gnss.separation.meters()==0) {
            Serial.println(F("Warning: GNSS gives ellipsoid altitude and no geoid & MSL"));
        } else {
            Serial.print(F("GNSS reported geoid separation: "));
            Serial.println(gnss.separation.meters());
        }
        (void) leap_seconds_valid();    // computes leap_seconds_correction
        SetupTimeMarker = millis();
        prev_lat = gnss.location.lat();
        prev_lon = gnss.location.lng();
        firstfix = false;
        validfix = false;            // wait for next fix
    } else if (gnss_new_fix) {
        if (fabs(gnss.location.lat()-prev_lat) > 0.15)   // looks like bad data
            validfix = false;
        if (fabs(gnss.location.lng()-prev_lon) > 0.25)
            validfix = false;
#if 0
#if defined(ESP32)
#if defined(USE_SD_CARD)
        // compute course and compare with what TinyGPS says
        if (validfix && ThisAircraft.airborne) {
          float dy = (gnss.location.lat() - prev_lat);
          float dx = (gnss.location.lng() - prev_lon) * CosLat(ThisAircraft.latitude);
          float course2 = atan2(dx, dy);
          float coursediff = fabs(course2-gnss.course.deg());
          if (coursediff > 180)  coursediff = 360 - coursediff;
          if (coursediff > 30) {
              FlightLogComment("CRS gnss.course differs from lat/lon change\r\n");
          }
        }
#endif
#endif
#endif
        prev_lat = gnss.location.lat();
        prev_lon = gnss.location.lng();
    }
  }

  static uint32_t badgps=0;

  /* allow knowing when there was a good fix for 30 sec */
  if (validfix) {   // still, after the adjustments above
    if (initial_time == 0) {
      initial_time = millis();
Serial.printf("First fix:\r\n\
    lat/lon: %.5f %.5f\r\n\
    date: %d %d %d\r\n\
    time: %d %d %d\r\n",
gnss.location.lat(), gnss.location.lng(),
gnss.date.year(), gnss.date.month(), gnss.date.day(),
gnss.time.hour(), gnss.time.minute(), gnss.time.second());
      validfix = false;            // wait for next fix
    } else if (GNSSTimeMarker == 0) {
      if (millis() > initial_time + 30000 || (settings->debug_flags & DEBUG_SIMULATE)) {
        /* 30 sec after first fix */
        GNSSTimeMarker = millis();
      } else {
        validfix = false;          // do not transmit yet
      }
    }
  }

  if (validfix) {   // still, after the additional adjustments above

    badgps = 0;

    if (gnss_new_fix) {           // set in GNSS.cpp

      gnss_new_fix = false;       // reset until new data arrives

#if 0
      if (settings->rf_protocol != RF_PROTOCOL_LEGACY
       && settings->rf_protocol != RF_PROTOCOL_LATEST
       && settings->rf_protocol != RF_PROTOCOL_OGNTP)
        ThisAircraft.timestamp = now();     // updated by GNSSTimeSync()
      else
#endif
        ThisAircraft.timestamp = OurTime;      // updated in either Time.cpp or RF.cpp

      /* store previous course & altitude and timestamp
          so as to allow computation of turn and climb rates */
      ThisAircraft.prevtime_ms = ThisAircraft.gnsstime_ms;
      ThisAircraft.prevcourse = ThisAircraft.course;
      ThisAircraft.prevheading = ThisAircraft.heading;
      ThisAircraft.prevaltitude = ThisAircraft.altitude;

      // the new fix data
      ThisAircraft.gnsstime_ms = ref_time_ms;          /* last PPS, real or assumed */
      ThisAircraft.latitude = gnss.location.lat();
      ThisAircraft.longitude = gnss.location.lng();
      ThisAircraft.geoid_separation = gnss.separation.meters();
      ThisAircraft.altitude = gnss.altitude.meters() + ThisAircraft.geoid_separation;
             // - converts MSL altitude in GGA to altitude above ellipsoid
      if (ThisAircraft.aircraft_type == AIRCRAFT_TYPE_WINCH) {
        /* for "winch" aircraft type, elevate above ground */
        ThisAircraft.altitude += (float) (((ThisAircraft.timestamp & 0x03) * 100) + 100);
      }
      ThisAircraft.course = gnss.course.deg();
      if (ThisAircraft.course < 0.0)   ThisAircraft.course += 360.0;
      ThisAircraft.speed = gnss.speed.knots();
      ThisAircraft.hdop = (uint16_t) gnss.hdop.value();

      /* if no baro sensor, fill in ThisAircraft.vs based on GPS data */
      if (baro_chip == NULL) {
        /* only do this once every 4 seconds */
        static uint32_t time_to_estimate_climb = 0;
        if (ThisAircraft.gnsstime_ms > time_to_estimate_climb) {
          time_to_estimate_climb = ThisAircraft.gnsstime_ms + 3900;
          ThisAircraft.vs = Estimate_Climbrate();
        }
      } /* else it was filled above in Baro_loop() */

#if 0
/* check timing */
#include "WiFi.h"
      static uint32_t msthen = 0;
      if ((settings->debug_flags & DEBUG_DEEPER) && udp_is_ready) {
        uint32_t msnow = millis();
        if (msnow > msthen+10000) {
          msthen = msnow;
          snprintf_P(UDPpacketBuffer, UDP_PACKET_BUFSIZE,
            PSTR("$PSTCT,%ld,%ld,%ld,%ld\r\n"),
               msnow, ThisAircraft.gnsstime_ms, ThisAircraft.prevtime_ms, ThisAircraft.timestamp);
          SoC->WiFi_transmit_UDP(NMEA_UDP_PORT, (byte *) UDPpacketBuffer,
                         strlen(UDPpacketBuffer));
        }
      }
#endif

// excluding the EGM96 lookup means the altitude reported to external devices in GGA sentences
// will be ellipsoid rather than MSL - but only in the rare cases of "fake" GPS units
// and this has no effect on the internal SoftRF calculations nor what it transmits.

      /*
       * When geoidal separation is zero or not available - use approx. EGM96 value
       */
      if (ThisAircraft.geoid_separation == 0.0) {
        ThisAircraft.geoid_separation = EGM96GeoidSeparation();
        /* we can assume the GPS unit is giving ellipsoid height */
        /* we now store ellipsoid altitude - so leave it alone */
        //ThisAircraft.altitude -= ThisAircraft.geoid_separation;
      }

      Estimate_Wind();      // estimate wind from present and past GNSS data

      /* generate a random aircraft ID if necessary */
      /* doing it here (after some delay) allows use of millis() as seed for random ID */
      if (ThisAircraft.addr == 0)   // ADDR_TYPE_ANONYMOUS - do this once
        generate_random_id();

    }  /* end of if(new_fix) */

    if ((settings->debug_flags & DEBUG_SIMULATE) == 0) {

      // check for newly received data, usually returns false
      // >>> do this here (too?) to ensure no incoming packets are missed
      rx_tried = true;
      rx_success = RF_Receive();
if (rx_success) which_rx_try = 1;
      // if received a packet, postpone transmission until next time around the loop().

      if (!rx_success && RF_Transmit_Ready() && settings->relay != RELAY_ONLY) {
          // Don't bother with the encode() if can't transmit right now
          size_t s = RF_Encode(&ThisAircraft);  // returns 0 if implausible data
          if (s != 0) {
              RF_Transmit(s);
              // if actually transmitted, time-slot is then locked out
              if (RF_Transmit_Happened())
                tx_success = true;
          }
      }
      /* - this only actually transmits when some preset random time is reached */

    }

  } else {      /* not validfix */

    if (badgps==0) badgps = millis();
    if (millis() > badgps + 30000) {
      /* 30 seconds without GPS fix - wipe history */
      badgps = 0;
      initial_time = 0;
      GNSSTimeMarker = 0;
      ThisAircraft.prevtime_ms = 0;
      this_airborne(false);     // after 60 calls (30 min) forced to "land"
    }
  }

  if ((settings->debug_flags & DEBUG_SIMULATE) == 0) {

#if 0
// >>> this may be causing "RF loopback" on sx1262?
// - nope still get some loopbacks in try1
// - but with this code included it only happens in try2?
// - note that the "try1" will be reached again in about 1-2 ms around the loop
    // ensure receiver is re-activated
    if (!rx_tried || tx_success)
      rx_success = RF_Receive();
if (rx_success) which_rx_try = 2;
#endif

//if (rx_success)
//Serial.println("received packet...");

#if DEBUG
    rx_success = true;
#endif

    /* process received data - only if we know where we are */
    if (rx_success && validfix)  ParseData();

  }

#if defined(ENABLE_TTN)
  TTN_loop();
#endif

  if (validfix) {
    /* handle the known traffic - only if we know where we are */
    Traffic_loop();
  }

  if (validfix && settings->logflight != FLIGHT_LOG_NONE) {
    // Try and write to the flight log on the SD card between RF time slots (to avoid
    // contention with RF chip for the SPI bus, if SD adapter is on same bus on a T-Beam),
    // but after the GNSS fix for the current second.
    // This is not an issue in simulation mode since the radio is not being used.
    // Too tight a limit on the reporting time window results in some missing reports.
    uint32_t msnow = millis();
#if defined(ESP32)
    uint32_t ms_since_pps = (msnow - ref_time_ms);
    if (msnow > IGCTimeMarker && ms_since_pps > 270
            && ms_since_pps < ((settings->debug_flags & DEBUG_SIMULATE)? 800 : 370)) {
      logFlightPosition();
      if (settings->logflight == FLIGHT_LOG_TRAFFIC)
          logCloseTraffic();
      IGCTimeMarker = ref_time_ms + (1000 * (uint32_t) settings->loginterval) + 320;
    }
#else
    // on T-Echo don't worry about SPI bus
    if (msnow > IGCTimeMarker) {
      logFlightPosition();
      if (settings->logflight == FLIGHT_LOG_TRAFFIC)
          logCloseTraffic();
      IGCTimeMarker = ref_time_ms + (1000 * (uint32_t) settings->loginterval) + 320;
    }
#endif
  }

  if (isTimeToDisplay()) {
    if (validfix) {
      LED_DisplayTraffic();
    } else {
      LED_Clear();
    }
    LEDTimeMarker = millis();
  }

  Buzzer_loop();   /* may sound collision alarms */

  Strobe_loop();

#if !defined(EXCLUDE_VOICE)
#if defined(ESP32)
  Voice_loop();   /* may sound collision alarms */
#endif
#endif

  if (isTimeToExport()) {
    NMEA_Export();
    GDL90_Export();

    if (validfix) {
      D1090_Export();
    }
    ExportTimeMarker = millis();
  }

  // Handle Air Connect
  NMEA_loop();

  //ClearExpired();    // now done in Traffic_loop() instead
}

#if !defined(EXCLUDE_MAVLINK)
void uav()
{
  bool success = false;

  PickMAVLinkFix();

  MAVLinkTimeSync();
  MAVLinkSetWiFiPower();

  ThisAircraft.timestamp = OurTime;  // now();

  if (isValidMAVFix()) {
    ThisAircraft.latitude = the_aircraft.location.gps_lat / 1e7;
    ThisAircraft.longitude = the_aircraft.location.gps_lon / 1e7;
    ThisAircraft.altitude = the_aircraft.location.gps_alt / 1000.0;
    ThisAircraft.course = the_aircraft.location.gps_cog;
    ThisAircraft.speed = (the_aircraft.location.gps_vog / 100.0) / _GPS_MPS_PER_KNOT;
    ThisAircraft.pressure_altitude = the_aircraft.location.baro_alt;
    ThisAircraft.hdop = the_aircraft.location.gps_hdop;

    RF_Transmit(RF_Encode(&ThisAircraft));
  }

  success = RF_Receive();

  if (success && isValidMAVFix()) ParseData();

  if (isTimeToExport() && isValidMAVFix()) {
    MAVLinkShareTraffic();
    ExportTimeMarker = millis();
  }

  ClearExpired();
}
#endif /* EXCLUDE_MAVLINK */

#if !defined(EXCLUDE_WIFI)
void bridge()
{
  bool success;

  size_t tx_size = WiFi_Receive_UDP(&TxBuffer[0], MAX_PKT_SIZE);

  if (tx_size > 0) {
    RF_Transmit(tx_size);
  }

  success = RF_Receive();

  if(success)
  {
    size_t rx_size = RF_Payload_Size(settings->rf_protocol);
    rx_size = rx_size > sizeof(fo_raw) ? sizeof(fo_raw) : rx_size;

    memset(fo_raw, 0, sizeof(fo_raw));
    memcpy(fo_raw, RxBuffer, rx_size);

    if (settings->nmea_p) {
      StdOut.print(F("$PSRFI,"));
      StdOut.print((unsigned long) now());    StdOut.print(F(","));
      StdOut.print(Bin2Hex(fo_raw, rx_size)); StdOut.print(F(","));
      StdOut.println(RF_last_rssi);
    }

    Raw_Transmit_UDP();
  }

  if (isTimeToDisplay()) {
    LED_Clear();
    LEDTimeMarker = millis();
  }
}
#endif /* EXCLUDE_WIFI */

#if !defined(EXCLUDE_WATCHOUT_MODE)
void watchout()
{
  bool success;

  success = RF_Receive();

  if (success) {
    size_t rx_size = RF_Payload_Size(settings->rf_protocol);
    rx_size = rx_size > sizeof(fo_raw) ? sizeof(fo_raw) : rx_size;

    memset(fo_raw, 0, sizeof(fo_raw));
    memcpy(fo_raw, RxBuffer, rx_size);

    if (settings->nmea_p) {
      StdOut.print(F("$PSRFI,"));
      StdOut.print((unsigned long) now());    StdOut.print(F(","));
      StdOut.print(Bin2Hex(fo_raw, rx_size)); StdOut.print(F(","));
      StdOut.println(RF_last_rssi);
    }
  }

  if (isTimeToDisplay()) {
    LED_Clear();
    LEDTimeMarker = millis();
  }
}
#endif /* EXCLUDE_WATCHOUT_MODE */

#if !defined(EXCLUDE_TEST_MODE)

unsigned int pos_ndx = 0;
unsigned long TxPosUpdMarker = 0;

void txrx_test()
{
  bool success = false;
#if DEBUG_TIMING
  unsigned long baro_start_ms, baro_end_ms;
  unsigned long tx_start_ms, tx_end_ms, rx_start_ms, rx_end_ms;
  unsigned long parse_start_ms, parse_end_ms, led_start_ms, led_end_ms;
  unsigned long export_start_ms, export_end_ms;
  unsigned long oled_start_ms, oled_end_ms;
#endif
  ThisAircraft.timestamp = OurTime;    // now();

  if (TxPosUpdMarker == 0 || (millis() - TxPosUpdMarker) > 4000 ) {
    ThisAircraft.latitude =  pgm_read_float( &txrx_test_positions[pos_ndx][0]);
    ThisAircraft.longitude =  pgm_read_float( &txrx_test_positions[pos_ndx][1]);
    pos_ndx = (pos_ndx + 1) % TXRX_TEST_NUM_POSITIONS;
    TxPosUpdMarker = millis();
  }
  ThisAircraft.altitude = TXRX_TEST_ALTITUDE;
  ThisAircraft.course = TXRX_TEST_COURSE;
  ThisAircraft.speed = TXRX_TEST_SPEED;
  ThisAircraft.vs = TXRX_TEST_VS;

#if DEBUG_TIMING
  baro_start_ms = millis();
#endif
  Baro_loop();
#if DEBUG_TIMING
  baro_end_ms = millis();
#endif

#if defined(ENABLE_AHRS)
  AHRS_loop();
#endif /* ENABLE_AHRS */

#if DEBUG_TIMING
  tx_start_ms = millis();
#endif
  RF_Transmit(RF_Encode(&ThisAircraft));
#if DEBUG_TIMING
  tx_end_ms = millis();
  rx_start_ms = millis();
#endif
  success = RF_Receive();
#if DEBUG_TIMING
  rx_end_ms = millis();
#endif

#if DEBUG_TIMING
  parse_start_ms = millis();
#endif
  if (success) ParseData();
#if DEBUG_TIMING
  parse_end_ms = millis();
#endif

#if defined(ENABLE_TTN)
  TTN_loop();
#endif

  Traffic_loop();

#if DEBUG_TIMING
  led_start_ms = millis();
#endif
  if (isTimeToDisplay()) {
    LED_DisplayTraffic();
    LEDTimeMarker = millis();
  }
#if DEBUG_TIMING
  led_end_ms = millis();
#endif

  Buzzer_loop();

  Strobe_loop();

#if !defined(EXCLUDE_VOICE)
#if defined(ESP32)
  Voice_loop();
#endif
#endif

#if DEBUG_TIMING
  export_start_ms = millis();
#endif
  if (isTimeToExport()) {
#if defined(USE_NMEALIB)
    NMEA_Position();
#endif
    NMEA_Export();
    GDL90_Export();
    D1090_Export();
    ExportTimeMarker = millis();
  }
#if DEBUG_TIMING
  export_end_ms = millis();
#endif

#if DEBUG_TIMING
  oled_start_ms = millis();
#endif
//  SoC->Display_loop();
#if DEBUG_TIMING
  oled_end_ms = millis();
#endif

#if DEBUG_TIMING
  if (baro_start_ms - baro_end_ms) {
    Serial.print(F("Baro start: "));
    Serial.print(baro_start_ms);
    Serial.print(F(" Baro stop: "));
    Serial.println(baro_end_ms);
  }
  if (tx_end_ms - tx_start_ms) {
    Serial.print(F("TX start: "));
    Serial.print(tx_start_ms);
    Serial.print(F(" TX stop: "));
    Serial.println(tx_end_ms);
  }
  if (rx_end_ms - rx_start_ms) {
    Serial.print(F("RX start: "));
    Serial.print(rx_start_ms);
    Serial.print(F(" RX stop: "));
    Serial.println(rx_end_ms);
  }
  if (parse_end_ms - parse_start_ms) {
    Serial.print(F("Parse start: "));
    Serial.print(parse_start_ms);
    Serial.print(F(" Parse stop: "));
    Serial.println(parse_end_ms);
  }
  if (led_end_ms - led_start_ms) {
    Serial.print(F("LED start: "));
    Serial.print(led_start_ms);
    Serial.print(F(" LED stop: "));
    Serial.println(led_end_ms);
  }
  if (export_end_ms - export_start_ms) {
    Serial.print(F("Export start: "));
    Serial.print(export_start_ms);
    Serial.print(F(" Export stop: "));
    Serial.println(export_end_ms);
  }
  if (oled_end_ms - oled_start_ms) {
    Serial.print(F("OLED start: "));
    Serial.print(oled_start_ms);
    Serial.print(F(" OLED stop: "));
    Serial.println(oled_end_ms);
  }
#endif

  // Handle Air Connect
  NMEA_loop();

  ClearExpired();
}

#endif /* EXCLUDE_TEST_MODE */

// use this mode to connect to UBlox U-Center utility software to
// diagnose the GNSS module - should also set baud rate to 9600
void gpsbridge()
{
  if (Serial.available()) {      // If anything comes in from USB,
    Serial_GNSS_In.write(Serial.read());   // read it and send it out to the GPS.
  }

  if (Serial_GNSS_In.available()) {     // If anything comes in from the GPS,
    Serial.write(Serial_GNSS_In.read());   // read it and send it out to USB.
  }
}

void loop()
{
  // Do common RF stuff first
  if (settings->mode != SOFTRF_MODE_GPSBRIDGE)
    RF_loop();

  switch (settings->mode)
  {
  case SOFTRF_MODE_NORMAL:
    normal();
    break;
#if !defined(EXCLUDE_TEST_MODE)
  case SOFTRF_MODE_TXRX_TEST:
    txrx_test();
    break;
#endif /* EXCLUDE_TEST_MODE */
#if !defined(EXCLUDE_MAVLINK)
  case SOFTRF_MODE_UAV:
    uav();
    break;
#endif /* EXCLUDE_MAVLINK */
#if !defined(EXCLUDE_WIFI)
  case SOFTRF_MODE_BRIDGE:
    bridge();
    break;
#endif /* EXCLUDE_WIFI */
#if !defined(EXCLUDE_WATCHOUT_MODE)
  case SOFTRF_MODE_WATCHOUT:
    watchout();
    break;
#endif /* EXCLUDE_WATCHOUT_MODE */
  case SOFTRF_MODE_GPSBRIDGE:
    gpsbridge();
    break;
  default:
    normal();
    break;
  }

  // Show status info on tiny OLED display
  SoC->Display_loop();

  // battery status LED
  LED_loop();

  // Handle DNS
  WiFi_loop();

  // Handle Web
  Web_loop();

  // Handle OTA update.
  OTA_loop();

#if LOGGER_IS_ENABLED
  Logger_loop();
#endif /* LOGGER_IS_ENABLED */

  SoC->loop();

  if (SoC->Bluetooth_ops) {
    SoC->Bluetooth_ops->loop();
  }

  if (SoC->USB_ops) {
    SoC->USB_ops->loop();
  }

  if (SoC->UART_ops) {
     SoC->UART_ops->loop();
  }

  Battery_loop();

  SoC->Button_loop();

#if defined(TAKE_CARE_OF_MILLIS_ROLLOVER)
  /* restart the device when uptime is more than 47 days */
  if (millis() > (47 * 24 * 3600 * 1000UL)) {
    SoC->reset();
  }
#endif /* TAKE_CARE_OF_MILLIS_ROLLOVER */

  yield();
}
