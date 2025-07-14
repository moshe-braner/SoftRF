/*
 * WebHelper.cpp
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

//#include <string.h>

#include "../system/SoC.h"

#if defined(EXCLUDE_WIFI)
void Web_setup()    {}
void Web_loop()     {}
void Web_fini()     {}
#else

#include <Arduino.h>

#if !defined(ESP32)
void Web_setup()    {}
void Web_loop()     {}
void Web_fini()     {}
#else

#include "SPIFFS.h"
#include "SD.h"

#include "../system/SoC.h"
#include "../driver/Battery.h"
#include "../driver/RF.h"
#include "Web.h"
#include "../driver/Settings.h"
#include "../driver/Filesys.h"
#include "../driver/OLED.h"
#include "../driver/GNSS.h"
#include "../driver/Baro.h"
#include "../driver/LED.h"
#include "../driver/Buzzer.h"
#include "../driver/Voice.h"
#include "../driver/Bluetooth.h"
#include "../TrafficHelper.h"
#include "../protocol/radio/Legacy.h"
#include "../protocol/data/NMEA.h"
#include "../protocol/data/IGC.h"
#include "../protocol/data/GDL90.h"
#include "../protocol/data/D1090.h"
#include "../protocol/data/GNS5892.h"

#if defined(ENABLE_AHRS)
#include "../driver/AHRS.h"
#endif /* ENABLE_AHRS */

static uint32_t prev_rx_pkt_cnt = 0;

//static const char Logo[] PROGMEM = {
//#include "../Logo.h"
//    } ;

// #include "jquery_min_js.h"     - skipped to save space

static bool BTpaused = false;
static bool reboot_pending = false;

const char *page_message()
{
    if (settings->debug_flags & DEBUG_SIMULATE)
        return "Warning: simulation mode (debug_flag 800000)";
    if (BTpaused)
        return "Bluetooth paused, reboot to resume";
    return settings_message();
}

static const char home[] PROGMEM =
    "<br><br><input type=button onClick=\"location.href='/'\" value='Back to Home'><br><br>";

byte getVal(char c)
{
   if (c >= '0' && c <= '9')
     return (byte)(c - '0');
   else
     return (byte)(toupper(c)-'A'+10);
}

//#if DEBUG
#if 0
void Hex2Bin(String str, byte *buffer)
{
  char hexdata[2 * PKT_SIZE + 1];
  
  str.toCharArray(hexdata, sizeof(hexdata));
  for(int j = 0; j < PKT_SIZE * 2 ; j+=2)
  {
    buffer[j>>1] = getVal(hexdata[j+1]) + (getVal(hexdata[j]) << 4);
  }
}
#endif

// Compiler said:
//  Global variables use 63296 bytes of dynamic memory, maximum is 327680 bytes.
// Saw this in serial output:
//  handleRoot()...
//  Free memory: 19904
//  Stopping Bluetooth for web access
//  Free memory: 68892
// But in v112 it is over 150000 - why?

void stop_bluetooth(size_t needed_mem)
{
  size_t freemem = ESP.getFreeHeap();
  Serial.print(F("Free memory: "));  Serial.println(freemem);
  if (!BTactive || BTpaused || freemem > needed_mem)
      return;
  if (settings->bluetooth != BLUETOOTH_OFF) {
      if (SoC->Bluetooth_ops) {
          Serial.println(F("Stopping Bluetooth for web access"));
          SoC->Bluetooth_ops->fini();
          BTpaused = true;
          OLED_msg("BT", "OFF");
      }
  }
  yield();
  Serial.print(F("Free memory (BT off): "));
  Serial.println(ESP.getFreeHeap());
}

void close_logs()
{
#if defined(USE_SD_CARD)
    closeFlightLog();
    closeSDlog();
#endif
    if (AlarmLogOpen)
        AlarmLog.close();
}

static const char about_html[] PROGMEM = "<html>\
  <head>\
    <meta name='viewport' content='width=device-width, initial-scale=1'>\
    <title>About</title>\
  </head>\
<body>\
<h1>About</h1>\
<h4>&nbsp;&nbsp;&nbsp;&nbsp;This version of SoftRF by Moshe Braner</h4>\
<h4>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;http://github.com/moshe-braner/SoftRF</h4>\
<h4>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;mo\
she.bra\
ner@\
gm\
ail.\
com</h4>\
<h4>&nbsp;&nbsp;&nbsp;&nbsp;Based on the SoftRF project by Linar Yusupov</h4>\
<h4>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;http://github.com/lyusupov/SoftRF</h4>\
<br>\
<h2>Credits</h2>\
<p>(in historical order)</p>\
<table width=100%%>\
<tr><th align=left>Ivan Grokhotkov</th><td>Arduino core for ESP8266</td></tr>\
<tr><th align=left>Zak Kemble</th><td>nRF905 library</td></tr>\
<tr><th align=left>Stanislaw Pusep</th><td>flarm_decode</td></tr>\
<tr><th align=left>Paul Stoffregen</th><td>Arduino Time Library</td></tr>\
<tr><th align=left>Mikal Hart</th><td>TinyGPS++ and PString Libraries</td></tr>\
<tr><th align=left>Phil Burgess</th><td>Adafruit NeoPixel Library</td></tr>\
<tr><th align=left>Andy Little</th><td>Aircraft and MAVLink Libraries</td></tr>\
<tr><th align=left>Peter Knight</th><td>TrueRandom Library</td></tr>\
<tr><th align=left>Matthijs Kooijman</th><td>IBM LMIC and Semtech Basic MAC frameworks for Arduino</td></tr>\
<tr><th align=left>David Paiva</th><td>ESP8266FtpServer</td></tr>\
<tr><th align=left>Lammert Bies</th><td>Lib_crc</td></tr>\
<tr><th align=left>Pawel Jalocha</th><td>OGN library</td></tr>\
<tr><th align=left>Timur Sinitsyn, Tobias Simon, Ferry Huberts</th><td>NMEA library</td></tr>\
<tr><th align=left>yangbinbin (yangbinbin_ytu@163.com)</th><td>ADS-B encoder C++ library</td></tr>\
<tr><th align=left>Hristo Gochkov</th><td>Arduino core for ESP32</td></tr>\
<tr><th align=left>Evandro Copercini</th><td>ESP32 BT SPP library</td></tr>\
<tr><th align=left>Limor Fried and Ladyada</th><td>Adafruit BMP085 library</td></tr>\
<tr><th align=left>Kevin Townsend</th><td>Adafruit BMP280 library</td></tr>\
<tr><th align=left>Limor Fried and Kevin Townsend</th><td>Adafruit MPL3115A2 library</td></tr>\
<tr><th align=left>Oliver Kraus</th><td>U8g2 LCD, OLED and eInk library</td></tr>\
<tr><th align=left>Michael Miller</th><td>NeoPixelBus library</td></tr>\
<tr><th align=left>Shenzhen Xin Yuan (LilyGO) ET company</th><td>TTGO T-Beam and T-Watch</td></tr>\
<tr><th align=left>JS Foundation</th><td>jQuery library</td></tr>\
<tr><th align=left>XCSoar team</th><td>EGM96 data</td></tr>\
<tr><th align=left>Mike McCauley</th><td>BCM2835 C library</td></tr>\
<tr><th align=left>Dario Longobardi</th><td>SimpleNetwork library</td></tr>\
<tr><th align=left>Benoit Blanchon</th><td>ArduinoJson library</td></tr>\
<tr><th align=left>flashrom.org project</th><td>Flashrom library</td></tr>\
<tr><th align=left>Robert Wessels and Tony Cave</th><td>EasyLink library</td></tr>\
<tr><th align=left>Oliver Jowett</th><td>Dump978 library</td></tr>\
<tr><th align=left>Phil Karn</th><td>FEC library</td></tr>\
<tr><th align=left>Lewis He</th><td>AXP20X library</td></tr>\
<tr><th align=left>Bodmer</th><td>TFT library</td></tr>\
<tr><th align=left>Michael Kuyper</th><td>Basic MAC library</td></tr>\
<tr><th align=left>Tim Eckel and Horst Reiterer</th><td>ToneAC library</td></tr>\
<tr><th align=left>Moshe Braner</th><td>Collision algorithm for circling aircraft</td></tr>\
</table>\
<hr>\
Copyright (C) 2015-2021 &nbsp;&nbsp;&nbsp; Linar Yusupov\
</body>\
</html>";

void set_upload(char *buf, const char *filename, const char *pageurl)
{
  snprintf_P ( buf, 320, PSTR(
 "<html>\
 <p>Select and upload %s</p>\
 <form method='POST' action='%s' enctype='multipart/form-data'>\
 <input type='file' name='name'><input type='submit' value='Upload' title='Upload'>\
 </form>\
 </html>"), filename, pageurl);
}

static File UploadFile;

//static const char *texttext  = "text/text";    // not used
//static const char *textplain = "text/plain";   // displays in too-small a font?
static const char *textplain = "text/html";
static const char *texthtml  = "text/html";
static const char *octet = "application/octet-stream";

void serve_html(const char *html)
{
    SoC->swSer_enableRx(false);
    server.sendHeader(String(F("Cache-Control")), String(F("no-cache, no-store, must-revalidate")));
    server.sendHeader(String(F("Pragma")), String(F("no-cache")));
    server.sendHeader(String(F("Expires")), String(F("-1")));
    server.send(200, texthtml, html);
    SoC->swSer_enableRx(true);
}

void serve_file(File file, const char *filename, const char *RAMbuf=NULL)
{
    char buf[64];
    snprintf(buf, 64, "attachment; filename=%s", filename);
    SoC->swSer_enableRx(false);
    yield();
    SoC->WDT_fini();    // otherwise serving large files result in WDT crash
    server.sendHeader("Content-Type", octet);
    server.sendHeader("Content-Disposition", buf);
    server.sendHeader("Connection", "close");
    if (RAMbuf)
        server.send(200, octet, RAMbuf);  // must be null-terminated
    else
        server.streamFile(file, octet);
    yield();
    SoC->WDT_setup();
    SoC->swSer_enableRx(true);
}

void anyUpload(bool toSD)
{
  HTTPUpload& uploading = server.upload();

  if (uploading.status == UPLOAD_FILE_START)
  {
    String filename = uploading.filename;
    if (filename.c_str()[0] == '\0') {
        server.send(500, textplain, "no file specified");
        return;
    }
#if defined(USE_SD_CARD)
    if (toSD) {
        if (filename.startsWith("/"))
            filename = "/logs" + filename;
        else
            filename = "/logs/" + filename;
        Serial.print(F("uploading into SD: "));
        Serial.println(filename);
        UploadFile = SD.open(filename.c_str(), FILE_WRITE);
        if (! UploadFile)
            Serial.println(F("Failed to open file for writing on SD"));
    } else
#endif
#if defined(ESP32)
    {
        if (! filename.startsWith("/"))
            filename = "/" + filename;
        Serial.print(F("uploading into SPIFFS: "));
        Serial.println(filename);
        UploadFile = SPIFFS.open(filename.c_str(), FILE_WRITE);
        if (! UploadFile)
            Serial.println(F("Failed to open file for writing in SPIFFS"));
    }
#endif
  }
  else if (uploading.status == UPLOAD_FILE_WRITE)
  {
    if (UploadFile) {
      SoC->swSer_enableRx(false);
      yield();
      SoC->WDT_fini();    // in case the file.write() will take a while
      size_t loaded = UploadFile.write(uploading.buf, uploading.currentSize);
      delay(30);
      SoC->WDT_setup();
      SoC->swSer_enableRx(true);
      // Serial.print(F("... bytes: ")); Serial.println(loaded);
    }
  } 
  else if (uploading.status == UPLOAD_FILE_END)
  {
    if (UploadFile) {
      UploadFile.close();
      Serial.print(F("Uploaded Size: ")); Serial.println(uploading.totalSize);
      if (uploading.totalSize > 0) {
        server.send(200, texthtml,
          "<html><p align=center><h3 align=center>uploaded file</h3></p></html>");
      } else {
        server.send(500, textplain, "uploaded zero bytes");
      }
    } else {
      server.send(500, textplain, "couldn't create file");
    }
  }
  yield();
}

#if defined(USE_EGM96)
void egmUpload()   // into SPIFFS
{
    //Serial.println(F("Loading egm96s.dem into SPIFFS..."));
    anyUpload(false);
}
#else
void egmUpload()
{
    Serial.println(F("egm96s.dem ignored..."));
}
#endif

void wavUpload()   // into SPIFFS
{
    //Serial.println(F("Replacing waves.tar in SPIFFS..."));
    clear_waves();
    anyUpload(false);
}

void sdlogUpload()   // into SD card /logs/
{
    anyUpload(true);
}

void alarmlogfile()
{
#if defined(USE_SD_CARD)
    //closeSDlog();
#endif
    //closeFlightLog();
    if (AlarmLogOpen) {
      AlarmLog.close();
      AlarmLogOpen = false;
    }
    if (! SPIFFS.exists("/alarmlog.txt")) {
        server.send(404, textplain, "Alarm log file does not exist");
        return;
    }
    AlarmLog = SPIFFS.open("/alarmlog.txt", FILE_READ);
    if (AlarmLog) {
        serve_file(AlarmLog, "alarmlog.txt");
        AlarmLog.close();
    }
}

void settingsdownload()
{
    if (! SPIFFS.exists("/settings.txt")) {
        server.send(404, textplain, "settings file does not exist");
        return;
    }
    File file = SPIFFS.open("/settings.txt", FILE_READ);
    if (file) {
        serve_file(file, "settings.txt");
        file.close();
    }
}

void settingsupload()
{
    anyUpload(false);   // into SPIFFS
}

void settingsbackup()
{
    if (! SPIFFS.exists("/settings.txt"))
        save_settings_to_file();
    if (! SPIFFS.exists("/settings.txt")) {
        server.send(500, textplain, "failed to write settings.txt");
        return;
    }
    if (SPIFFS.exists("/settingb.txt"))
        SPIFFS.remove("/settingb.txt");
    SPIFFS.rename("/settings.txt","/settingb.txt");
    save_settings_to_file();
    if (! SPIFFS.exists("/settings.txt")) {     // save_settings_to_file() failed
        SPIFFS.rename("/settingb.txt","/settings.txt");
        server.send(500, textplain, "failed to make a copy of settings.txt");
        return;
    }
    server.send_P(200, texthtml,
      PSTR("<html><p align=center><h3 align=center>Copied settings.txt to settingb.txt</h3></p></html>"));
}

void settingsreboot(int status, const char *msg)
{
  close_logs();
  char buf[440];
  snprintf_P ( buf, 440,
      PSTR("<html>\
<head>\
<meta http-equiv='refresh' content='30; url=/'>\
<meta name='viewport' content='width=device-width, initial-scale=1'>\
<title>Restarting...</title>\
</head>\
<body>\
<p align=center><h3 align=center>%s</h3></p>\
<p align=center><h3 align=center>Restart is in progress, please wait...</h3></p>\
</body>\
</html>"), msg);
  server.send(status, texthtml, buf);
  SoC->WDT_fini();
  if (SoC->Bluetooth_ops) { SoC->Bluetooth_ops->fini(); }
  //EEPROM_store();
  delay(2000);
  reboot();
}

void settingsswap()
{
    if (! SPIFFS.exists("/settingb.txt")) {
        server.send(500, textplain, "settingb.txt backup file does not exist");
        return;
    }
    if (! SPIFFS.exists("/settings.txt"))
        save_settings_to_file();
    if (! SPIFFS.exists("/settings.txt")) {
        SPIFFS.rename("/settingb.txt","/settings.txt");
        settingsreboot(500, "no settings.txt and cannot create one, restored settingb.txt");
        return;
    }
    SPIFFS.rename("/settingb.txt","/settingt.txt");
    SPIFFS.rename("/settings.txt","/settingb.txt");
    SPIFFS.rename("/settingt.txt","/settings.txt");
    load_settings_from_file();             // test the new settings file
    if (SPIFFS.exists("/settings.txt")) {  // load_settings_from_file() did not fail
        settingsreboot(200, "swapped settings.txt & settingb.txt");
    } else {
        SPIFFS.remove("/settings.txt");   // former settingb.txt
        SPIFFS.rename("/settingb.txt","/settings.txt");
        settingsreboot(500, "invalid settingb.txt, restored settings.txt");
    }
}

void delPSRAMlog()
{
  char buf[180];
  snprintf_P ( buf, 180, PSTR(
 "<html>\
 <h3>Delete the flight data in RAM?</h3>\
 <input type=button onClick=\"location.href='/dodelramlog'\" value='Delete'>\
 </html>"));
  server.send(200, texthtml, buf);
}
void dodelPSRAMlog()
{
  clearPSRAMlog();

  char buf[180];
  snprintf_P ( buf, 180,
    PSTR("<html><h3>Flight log RAM cleared</h3>%s</html>"), home);
  server.send(200, texthtml, buf);
}

// download current RAM flight log
void RAMlogfile()
{
    if (! PSRAMbuf || ! PSRAMbufUsed || ! FlightLogPath[0]) {
          Serial.println(F("no flight log in PSRAM"));
          server.send ( 404, textplain, "no flight log in RAM");
          return;
    }

    closeFlightLog();

    // the only method that worked was to append a null char
    //    and handle the PSRAMbuf log as a C-string:
    // server.send(200, octet, PSRAMbuf);
    // server.sendContent() did not work
    //    it puts the content in the web page despite "Disposition"
    PSRAMbuf[PSRAMbufUsed] = '\0';
    File file;   // dummy - RAMbuf will be used instead
    serve_file(file, FlightLogPath+1, PSRAMbuf);
    delay(2000);
    return;
}

#if defined(USE_SD_CARD)
// download current RAM flight log
void lastSDlog()
{
    if (! SD_is_mounted)
        return;

    closeFlightLog();
    //closeSDlog();
    String lastlog = " ";
    bool found = false;
    if (FlightLogPath[0]) {            // the last file written since boot
        lastlog = FlightLogPath+6;     // bare name, skip the "/logs/"
        found = true;
    } else {
        // find latest file by alphabetical order of file name
        File root = SD.open("/logs");
        if (! root) {
            Serial.println(F("Cannot open SD/logs"));
            server.send ( 500, textplain, "(cannot open SD/logs)");
            return;
        }
        File file = root.openNextFile();
        String file_name;
        while(file) {
            file_name = file.name();
            if (file_name.endsWith(".IGC") || file_name.endsWith(".igc")) {
                if (strcmp(lastlog.c_str(), file.name()) < 0) {
                    lastlog = file.name();
                    //Serial.print(F("Candidate latest log: "));
                    //Serial.println(lastlog);
                    found = true;
                }
            }
            file = root.openNextFile();
        }
        file.close();
        root.close();
    }
    if (found) {
        Serial.print(F("Sending latest log: ")); Serial.println(lastlog);
        lastlog = "/logs/" + lastlog;
        File file = SD.open(lastlog.c_str(), FILE_READ);
        if (file) {
            serve_file(file, lastlog.c_str()+6);  // skip the "/logs/"
            delay(2000);
            file.close();
        } else {
            Serial.print(F("Could not open latest log: ")); Serial.println(lastlog);
            server.send ( 404, textplain, "Could not open flight log file");
        }
    } else {
        server.send ( 404, textplain, "No flight log found");
    }
}
#endif

void handleSettings() {

  Serial.println(F("handleSettings()..."));

  stop_bluetooth((size_t)70000);

  bool is_prime_mk2 = false;
  if (hw_info.model == SOFTRF_MODEL_PRIME_MK2 /* && hw_info.revision >= 5 */)
    is_prime_mk2 = true;

  size_t size = 10000;
  size_t totalsize = size;
  char *offset;
  size_t len = 0;
  char *Settings_temp = (char *) malloc(size);

  if (Settings_temp == NULL) {
    Serial.println(F(">>> not enough RAM"));
    return;
  }

  Serial.println(F("Constructing basic settings page..."));

  offset = Settings_temp;

  /* Common part 1 */
  snprintf_P ( offset, size,
    PSTR("<html>\
<head>\
<meta name='viewport' content='width=device-width, initial-scale=1'>\
<title>Basic Settings</title>\
</head>\
<body>\
<table width=100%%>\
<tr>\
<td align=center><input type=button onClick=\"location.href='/'\" value='Home'></td>\
<td align=center><h2>Basic Settings</h2></td>\
<td align=center><input type=button onClick=\"location.href='/advstgs'\" value='Advanced Settings'></td>\
</tr>\
<tr><td>&nbsp;</td><th>(%s)</th><td>&nbsp;</td></tr>\
<tr><td>&nbsp;</td><th>If changed, click 'Save and restart' at the bottom</th><td>&nbsp;</td></tr>\
</table>\
<form action='/input' method='GET'>\
<table border=1 frame=hsides rules=rows width=100%%>\
<tr>\
<th align=left>Mode</th>\
<td align=right>\
<select name='mode'>\
<option %s value='%d'>Normal</option>\
<option %s value='%d'>GNSS Bridge</option>\
<option %s value='%d'>UAV</option>\
<option %s value='%d'>Bridge</option>\
</select>\
</td>\
</tr>"),
  page_message(),
  (settings->mode == SOFTRF_MODE_NORMAL ? "selected" : "") , SOFTRF_MODE_NORMAL,
  (settings->mode == SOFTRF_MODE_GPSBRIDGE ? "selected" : ""), SOFTRF_MODE_GPSBRIDGE,
  (settings->mode == SOFTRF_MODE_UAV ? "selected" : ""), SOFTRF_MODE_UAV,
  (settings->mode == SOFTRF_MODE_BRIDGE ? "selected" : ""), SOFTRF_MODE_BRIDGE
  );

  len = strlen(offset);
  offset += len;
  size -= len;

    snprintf_P ( offset, size,
      PSTR("\
<tr>\
<th align=left>Device ID</th>\
<td align=right>%06x\
</td>\
</tr>"),(SoC->getChipId() & 0x00FFFFFF));
    
  len = strlen(offset);
  offset += len;
  size -= len;    

    snprintf_P ( offset, size,
      PSTR("\
<tr>\
<th align=left>Aircraft (ICAO) ID (6 HEX digits)</th>\
<td align=right>\
<INPUT type='text' name='aircraft_id' maxlength='6' size='6' value='%06X'>\
</td>\
</tr>"),
  settings->aircraft_id);

  len = strlen(offset);
  offset += len;
  size -= len;

    snprintf_P ( offset, size,
      PSTR("\
<tr>\
<th align=left>ID type to use:</th>\
<td align=right>\
<select name='id_method'>\
<option %s value='%d'>ICAO</option>\
<option %s value='%d'>Device</option>\
<option %s value='%d'>Anonymous</option>\
</select>\
</td>\
</tr>"),
    (settings->id_method == ADDR_TYPE_ICAO ? "selected" : ""),      ADDR_TYPE_ICAO,
    ((settings->id_method!=ADDR_TYPE_ICAO && settings->id_method!=ADDR_TYPE_ANONYMOUS) ? "selected" : ""),
     (settings->id_method==ADDR_TYPE_OVERRIDE? ADDR_TYPE_OVERRIDE : ADDR_TYPE_FLARM),
    (settings->id_method == ADDR_TYPE_ANONYMOUS ? "selected" : ""), ADDR_TYPE_ANONYMOUS
    );
  
  len = strlen(offset);
  offset += len;
  size -= len;

  /* Radio specific part 1 */
  if (hw_info.rf == RF_IC_SX1276 || hw_info.rf == RF_IC_SX1262) {
    snprintf_P ( offset, size,
      PSTR("\
<tr>\
<th align=left>Protocol</th>\
<td align=right>\
<select name='protocol'>\
<option %s value='%d'>%s</option>\
<option %s value='%d'>%s</option>\
<option %s value='%d'>%s</option>\
<option %s value='%d'>%s</option>\
<option %s value='%d'>%s</option>\
</select>\
</td>\
</tr>"),
    (settings->rf_protocol == RF_PROTOCOL_LEGACY ? "selected" : ""),
     RF_PROTOCOL_LEGACY, legacy_proto_desc.name,
    (settings->rf_protocol == RF_PROTOCOL_LATEST ? "selected" : ""),
     RF_PROTOCOL_LATEST, "Latest",
    (settings->rf_protocol == RF_PROTOCOL_OGNTP ? "selected" : ""),
     RF_PROTOCOL_OGNTP, ogntp_proto_desc.name,
    (settings->rf_protocol == RF_PROTOCOL_P3I ? "selected" : ""),
     RF_PROTOCOL_P3I, p3i_proto_desc.name,
    (settings->rf_protocol == RF_PROTOCOL_FANET ? "selected" : ""),
     RF_PROTOCOL_FANET, fanet_proto_desc.name
    );
  } else {
    snprintf_P ( offset, size,
      PSTR("\
<tr>\
<th align=left>Protocol</th>\
<td align=right>%s\
</td>\
</tr>"),
    (settings->rf_protocol == RF_PROTOCOL_LEGACY   ? legacy_proto_desc.name :
    (settings->rf_protocol == RF_PROTOCOL_LATEST   ? "Latest" :
    (
#if !defined(EXCLUDE_UAT978)
     settings->rf_protocol == RF_PROTOCOL_ADSB_UAT ? uat978_proto_desc.name :
#endif
    (settings->rf_protocol == RF_PROTOCOL_FANET    ? fanet_proto_desc.name  :
     "UNK"))))
    );
  }
  len = strlen(offset);
  offset += len;
  size -= len;

  /* Common part 2 */
  snprintf_P ( offset, size,
    PSTR("\
<tr>\
<th align=left>Region</th>\
<td align=right>\
<select name='band'>\
<!--<option %s value='%d'>AUTO</option>-->\
<option %s value='%d'>EU (868.2 MHz)</option>\
<option %s value='%d'>RU (868.8 MHz)</option>\
<option %s value='%d'>CN (470 MHz)</option>\
<option %s value='%d'>US/CA (915 MHz)</option>\
<option %s value='%d'>NZ (869.25 MHz)</option>\
<option %s value='%d'>AU (921 MHz)</option>\
<option %s value='%d'>IN (866 MHz)</option>\
<option %s value='%d'>KR (920.9 MHz)</option>\
<option %s value='%d'>IL (916.2 MHz)</option>\
<option %s value='%d'>UK P3I (869.52)</option>\
</select>\
</td>\
</tr>\
<tr>\
<th align=left>Aircraft type</th>\
<td align=right>\
<select name='acft_type'>\
<option %s value='%d'>Glider</option>\
<option %s value='%d'>Towplane</option>\
<option %s value='%d'>Powered</option>\
<option %s value='%d'>Helicopter</option>\
<option %s value='%d'>UAV</option>\
<option %s value='%d'>Hangglider</option>\
<option %s value='%d'>Paraglider</option>\
<option %s value='%d'>Balloon</option>\
<option %s value='%d'>Static</option>\
<option %s value='%d'>Winch</option>\
</select>\
</td>\
</tr>\
<tr>\
<th align=left>Alarm trigger</th>\
<td align=right>\
<select name='alarm'>\
<option %s value='%d'>None</option>\
<option %s value='%d'>Distance</option>\
<option %s value='%d'>Vector</option>\
<option %s value='%d'>Latest</option>\
</select>\
</td>\
</tr>\
<tr>\
<th align=left>Buzzer</th>\
<td align=right>\
<select name='volume'>\
<option %s value='%d'>Off</option>\
<option %s value='%d'>Soft</option>\
<option %s value='%d'>Loud</option>\
<option %s value='%d'>External</option>\
</select>\
</td>\
</tr>"),
  (settings->band == RF_BAND_AUTO ? "selected" : ""), RF_BAND_AUTO,
  (settings->band == RF_BAND_EU ? "selected" : ""), RF_BAND_EU,
  (settings->band == RF_BAND_RU ? "selected" : ""), RF_BAND_RU,
  (settings->band == RF_BAND_CN ? "selected" : ""), RF_BAND_CN,
  (settings->band == RF_BAND_US ? "selected" : ""), RF_BAND_US,
  (settings->band == RF_BAND_NZ ? "selected" : ""), RF_BAND_NZ,
  (settings->band == RF_BAND_AU ? "selected" : ""), RF_BAND_AU,
  (settings->band == RF_BAND_IN ? "selected" : ""), RF_BAND_IN,
  (settings->band == RF_BAND_KR ? "selected" : ""), RF_BAND_KR,
  (settings->band == RF_BAND_IL ? "selected" : ""), RF_BAND_IL,
  (settings->band == RF_BAND_UK ? "selected" : ""), RF_BAND_UK,
  (settings->acft_type == AIRCRAFT_TYPE_GLIDER ? "selected" : ""),  AIRCRAFT_TYPE_GLIDER,
  (settings->acft_type == AIRCRAFT_TYPE_TOWPLANE ? "selected" : ""),  AIRCRAFT_TYPE_TOWPLANE,
  (settings->acft_type == AIRCRAFT_TYPE_POWERED ? "selected" : ""),  AIRCRAFT_TYPE_POWERED,
  (settings->acft_type == AIRCRAFT_TYPE_HELICOPTER ? "selected" : ""),  AIRCRAFT_TYPE_HELICOPTER,
  (settings->acft_type == AIRCRAFT_TYPE_UAV ? "selected" : ""),  AIRCRAFT_TYPE_UAV,
  (settings->acft_type == AIRCRAFT_TYPE_HANGGLIDER ? "selected" : ""),  AIRCRAFT_TYPE_HANGGLIDER,
  (settings->acft_type == AIRCRAFT_TYPE_PARAGLIDER ? "selected" : ""),  AIRCRAFT_TYPE_PARAGLIDER,
  (settings->acft_type == AIRCRAFT_TYPE_BALLOON ? "selected" : ""),  AIRCRAFT_TYPE_BALLOON,
  (settings->acft_type == AIRCRAFT_TYPE_STATIC ? "selected" : ""),  AIRCRAFT_TYPE_STATIC,
  (settings->acft_type == AIRCRAFT_TYPE_WINCH ? "selected" : ""),  AIRCRAFT_TYPE_WINCH,
  (settings->alarm == TRAFFIC_ALARM_NONE ? "selected" : ""),  TRAFFIC_ALARM_NONE,
  (settings->alarm == TRAFFIC_ALARM_DISTANCE ? "selected" : ""),  TRAFFIC_ALARM_DISTANCE,
  (settings->alarm == TRAFFIC_ALARM_VECTOR ? "selected" : ""),  TRAFFIC_ALARM_VECTOR,
  (settings->alarm == TRAFFIC_ALARM_LATEST ? "selected" : ""),  TRAFFIC_ALARM_LATEST,
  (settings->volume == BUZZER_OFF ? "selected" : ""), BUZZER_OFF,
  (settings->volume == BUZZER_VOLUME_LOW ? "selected" : ""), BUZZER_VOLUME_LOW,
  (settings->volume == BUZZER_VOLUME_FULL ? "selected" : ""), BUZZER_VOLUME_FULL,
  (settings->volume == BUZZER_EXT ? "selected" : ""), BUZZER_EXT
  );

  len = strlen(offset);
  offset += len;
  size -= len;

  yield();

  /* SoC specific part 1 */
  if (SoC->id == SOC_ESP32) {
    snprintf_P ( offset, size,
      PSTR("\
<tr>\
<th align=left>Built-in Bluetooth</th>\
<td align=right>\
<select name='bluetooth'>\
<option %s value='%d'>Off</option>\
<option %s value='%d'>SPP</option>\
<option %s value='%d'>LE</option>\
</select>\
</td>\
</tr>"),
    (settings->bluetooth == BLUETOOTH_OFF ? "selected" : ""), BLUETOOTH_OFF,
    (settings->bluetooth == BLUETOOTH_SPP ? "selected" : ""), BLUETOOTH_SPP,
    (settings->bluetooth == BLUETOOTH_LE_HM10_SERIAL ? "selected" : ""), BLUETOOTH_LE_HM10_SERIAL
    );

    len = strlen(offset);
    offset += len;
    size -= len;
  }

  yield();

  /* Common part 3 */
  snprintf_P ( offset, size,
    PSTR("\
<tr>\
<th align=left>NMEA primary output</th>\
<td align=right>\
<select name='nmea_out'>\
<option %s value='%d'>Off</option>\
<option %s value='%d'>UDP</option>\
<option %s value='%d'>Serial</option>"),
  (settings->nmea_out == DEST_NONE  ? "selected" : ""), DEST_NONE,
  (settings->nmea_out == DEST_UDP   ? "selected" : ""), DEST_UDP,
  (settings->nmea_out == DEST_UART  ? "selected" : ""), DEST_UART);

  len = strlen(offset);
  offset += len;
  size -= len;

  /* SoC specific part 2 */
  if (SoC->id == SOC_ESP32) {
    if (is_prime_mk2) {
     snprintf_P ( offset, size,
       PSTR("\
<option %s value='%d'>Serial 2</option>\
<option %s value='%d'>Bluetooth</option>\
<option %s value='%d'>TCP</option>"),
     (settings->nmea_out == DEST_UART2 ? "selected" : ""), DEST_UART2,
     (settings->nmea_out == DEST_BLUETOOTH ? "selected" : ""), DEST_BLUETOOTH,
     (settings->nmea_out == DEST_TCP       ? "selected" : ""), DEST_TCP);
    } else {
     snprintf_P ( offset, size,
       PSTR("\
<option %s value='%d'>Bluetooth</option>\
<option %s value='%d'>TCP</option>"),
     (settings->nmea_out == DEST_BLUETOOTH ? "selected" : ""), DEST_BLUETOOTH,
     (settings->nmea_out == DEST_TCP       ? "selected" : ""), DEST_TCP);
    }
    len = strlen(offset);
    offset += len;
    size -= len;
  }

    snprintf_P ( offset, size,
      PSTR("\
<tr>\
<th align=left>&nbsp;&nbsp;&nbsp;&nbsp;Sentences:</th>\
</tr>\
<tr>\
<th align=left>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;GNSS</th>\
<td align=right>\
<input type='radio' name='nmea_g' value='0' %s>Off\
<input type='radio' name='nmea_g' value='%d' %s>On\
</td>\
</tr>\
<tr>\
<th align=left>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Sensors</th>\
<td align=right>\
<input type='radio' name='nmea_s' value='0' %s>Off\
<input type='radio' name='nmea_s' value='%d' %s>On\
</td>\
</tr>\
<tr>\
<th align=left>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Traffic</th>\
<td align=right>\
<input type='radio' name='nmea_t' value='0' %s>Off\
<input type='radio' name='nmea_t' value='%d' %s>On\
</td>\
</tr>"),
  (!settings->nmea_g ? "checked" : ""),
   (settings->nmea_g ? settings->nmea_g : 1), (settings->nmea_g ? "checked" : ""),
  (!settings->nmea_s ? "checked" : ""),
   (settings->nmea_s ? settings->nmea_s : 1), (settings->nmea_s ? "checked" : ""),
  (!settings->nmea_t ? "checked" : ""),
   (settings->nmea_t ? settings->nmea_t : 1), (settings->nmea_t ? "checked" : ""));
       // - if "on" is chosen keep the bitfield value if not zero, else set to 1 (basic sentences)

    len = strlen(offset);
    offset += len;
    size -= len;

  /* second NMEA output route */
  snprintf_P ( offset, size,
    PSTR("\
<tr>\
<th align=left>NMEA second output</th>\
<td align=right>\
<select name='nmea_out2'>\
<option %s value='%d'>Off</option>\
<option %s value='%d'>UDP</option>\
<option %s value='%d'>Serial</option>"),
  (settings->nmea_out2 == DEST_NONE  ? "selected" : ""), DEST_NONE,
  (settings->nmea_out2 == DEST_UDP   ? "selected" : ""), DEST_UDP,
  (settings->nmea_out2 == DEST_UART  ? "selected" : ""), DEST_UART);

  len = strlen(offset);
  offset += len;
  size -= len;


  if (SoC->id == SOC_ESP32) {
    if (is_prime_mk2) {
     snprintf_P ( offset, size,
       PSTR("\
<option %s value='%d'>Serial 2</option>\
<option %s value='%d'>Bluetooth</option>\
<option %s value='%d'>TCP</option>"),
     (settings->nmea_out2 == DEST_UART2 ? "selected" : ""), DEST_UART2,
     (settings->nmea_out2 == DEST_BLUETOOTH ? "selected" : ""), DEST_BLUETOOTH,
     (settings->nmea_out2 == DEST_TCP       ? "selected" : ""), DEST_TCP);
    } else {
     snprintf_P ( offset, size,
       PSTR("\
<option %s value='%d'>Bluetooth</option>\
<option %s value='%d'>TCP</option>"),
     (settings->nmea_out2 == DEST_BLUETOOTH ? "selected" : ""), DEST_BLUETOOTH,
     (settings->nmea_out2 == DEST_TCP       ? "selected" : ""), DEST_TCP);
    }
    len = strlen(offset);
    offset += len;
    size -= len;
  }

// Serial.println(F("Settings page part 4 done"));

    snprintf_P ( offset, size,
      PSTR("\
<tr>\
<th align=left>&nbsp;&nbsp;&nbsp;&nbsp;Sentences:</th>\
</tr>\
<tr>\
<th align=left>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;GNSS</th>\
<td align=right>\
<input type='radio' name='nmea2_g' value='0' %s>Off\
<input type='radio' name='nmea2_g' value='%d' %s>On\
</td>\
</tr>\
<tr>\
<th align=left>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Sensors</th>\
<td align=right>\
<input type='radio' name='nmea2_s' value='0' %s>Off\
<input type='radio' name='nmea2_s' value='%d' %s>On\
</td>\
</tr>\
<tr>\
<th align=left>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Traffic</th>\
<td align=right>\
<input type='radio' name='nmea2_t' value='0' %s>Off\
<input type='radio' name='nmea2_t' value='%d' %s>On\
</td>\
</tr>\
<tr>\
<th align=left>Serial Output Baud Rate:</th>\
<td align=right>\
<select name='baud_rate'>\
<option %s value='%d'>Default</option>\
<option %s value='%d'>4800</option>\
<option %s value='%d'>9600</option>\
<option %s value='%d'>19200</option>\
<option %s value='%d'>38400</option>\
<option %s value='%d'>57600</option>\
<option %s value='%d'>115200</option>\
</select>\
</td>\
</tr>"),
  (!settings->nmea2_g ? "checked" : ""),
   (settings->nmea2_g ? settings->nmea2_g : 1), (settings->nmea2_g ? "checked" : ""),
  (!settings->nmea2_s ? "checked" : ""),
   (settings->nmea2_s ? settings->nmea2_s : 1), (settings->nmea2_s ? "checked" : ""),
  (!settings->nmea2_t ? "checked" : ""),
   (settings->nmea2_t ? settings->nmea2_t : 1), (settings->nmea2_t ? "checked" : ""),
  (settings->baud_rate == BAUD_DEFAULT ? "selected" : ""), BAUD_DEFAULT,
  (settings->baud_rate == BAUD_4800    ? "selected" : ""), BAUD_4800,
  (settings->baud_rate == BAUD_9600    ? "selected" : ""), BAUD_9600,
  (settings->baud_rate == BAUD_19200   ? "selected" : ""), BAUD_19200,
  (settings->baud_rate == BAUD_38400   ? "selected" : ""), BAUD_38400,
  (settings->baud_rate == BAUD_57600   ? "selected" : ""), BAUD_57600,
  (settings->baud_rate == BAUD_115200  ? "selected" : ""), BAUD_115200);

    len = strlen(offset);
    offset += len;
    size -= len;

  yield();

  /* Common part 6 */
  snprintf_P ( offset, size,
    PSTR("\
<tr>\
<th align=left>Stealth</th>\
<td align=right>\
<input type='radio' name='stealth' value='0' %s>Off\
<input type='radio' name='stealth' value='1' %s>On\
</td>\
</tr>\
<tr>\
<th align=left>No track</th>\
<td align=right>\
<input type='radio' name='no_track' value='0' %s>Off\
<input type='radio' name='no_track' value='1' %s>On\
</td>\
</tr>"),
  (!settings->stealth  ? "checked" : "") , (settings->stealth  ? "checked" : ""),
  (!settings->no_track ? "checked" : "") , (settings->no_track ? "checked" : "")
  );

  len = strlen(offset);
  offset += len;
  size -= len;

    snprintf_P ( offset, size,
      PSTR("\
<tr>\
<th align=left>Flight logging:</th>\
<td align=right>\
<select name='logflight'>\
<option %s value='%d'>Off</option>\
<option %s value='%d'>Always</option>\
<option %s value='%d'>Airborne</option>\
<option %s value='%d'>Traffic</option>\
</td>\
</tr>\
<tr>\
<th align=left>Flight log interval (sec):</th>\
<td align=right>\
<INPUT type='number' name='loginterval' min='1' max='255' size='4' value='%d'>\
</td>\
</tr>"),
  (settings->logflight==FLIGHT_LOG_NONE     ? "selected" : ""), FLIGHT_LOG_NONE,
  (settings->logflight==FLIGHT_LOG_ALWAYS   ? "selected" : ""), FLIGHT_LOG_ALWAYS,
  (settings->logflight==FLIGHT_LOG_AIRBORNE ? "selected" : ""), FLIGHT_LOG_AIRBORNE,
  (settings->logflight==FLIGHT_LOG_TRAFFIC  ? "selected" : ""), FLIGHT_LOG_TRAFFIC,
   settings->loginterval);

    len = strlen(offset);
    offset += len;
    size -= len;

  yield();

  /* Common part 7 */
  snprintf_P ( offset, size,
    PSTR("\
</table>\
<p align=center><INPUT type='submit' value='Save and restart'></p>\
</form>\
</body>\
</html>")
  );

  len = strlen(offset);
  offset += len;
  Serial.print(F("Settings page size: ")); Serial.print(offset-Settings_temp);
  Serial.print(F(" out of allocated: ")); Serial.println(totalsize);
  // currently about 3700
  serve_html(Settings_temp);
  Serial.print(F("Free memory (settings page staging): "));
  Serial.println(ESP.getFreeHeap());
  delay(1000);
  free(Settings_temp);
}

void handleAdvStgs() {

  Serial.println(F("handleAdvStgs()..."));

  stop_bluetooth((size_t)60000);

//  bool is_prime_mk2 = false;
//  if (hw_info.model == SOFTRF_MODEL_PRIME_MK2 /* && hw_info.revision >= 5 */)
//    is_prime_mk2 = true;

  size_t size = 15000;
  size_t totalsize = size;
  char *offset;
  size_t len = 0;
  char *Settings_temp = (char *) malloc(size);

  if (Settings_temp == NULL) {
    Serial.println(F(">>> not enough RAM"));
    return;
  }

  Serial.println(F("Constructing advanced settings page..."));

  offset = Settings_temp;

  snprintf_P ( offset, size,
    PSTR("<html>\
<head>\
<meta name='viewport' content='width=device-width, initial-scale=1'>\
<title>Advanced Settings</title>\
</head>\
<body>\
<table width=100%%>\
<tr>\
<td align=center><input type=button onClick=\"location.href='/'\" value='Home'></td>\
<td align=center><h2>Advanced Settings</h2></td>\
<td align=center><input type=button onClick=\"location.href='/settings'\" value='Basic Settings'></td>\
</tr>\
<tr><td>&nbsp;</td><th align=center>(%s)</th><td>&nbsp;</td></tr>\
<tr><td>&nbsp;</td><th align=center>If changed, click 'Save and restart' at the bottom</th><td>&nbsp;</td></tr>\
</table>\
<form action='/input' method='GET'>\
<table border=1 frame=hsides rules=rows width=100%%>"), page_message());
  len = strlen(offset);
  offset += len;
  size -= len;

  for (int i=STG_MODE; i<STG_END; i++) {
      char buf[64];
      if (stgdesc[i].type == STG_VOID)
          continue;
      if (stgdesc[i].type == STG_HIDDEN)   // only accessible via editing the file
          continue;
      if (format_setting(i, false, buf, 64) == false)
          continue;
      const char *w = stgdesc[i].label;
      int comma = strlen(w);
      const char *v = &buf[comma+1];
      if (buf[comma] != ',')  // should not happen
          v = "";
      if (i == STG_PSK && settings->psk[0] != '\0')
          v = "********";
      const char *z = stgcomment[i];
      if (! z)
          z = "";

      snprintf_P ( offset, size, PSTR("\
<tr>\
<th align=left>&nbsp;&nbsp;%s</th>\
<td align=center><INPUT type='text' name='%s' maxlength='32' value='%s' size='16'></td>\
<td align=left>%s</td>\
</tr>"),
           w, w, v, z);
           // the setting's label used both as text and the name of the INPUT
      len = strlen(offset);
      offset += len;
      size -= len;
      yield();
  }

#if defined(USE_OGN_ENCRYPTION)
  if (settings->rf_protocol == RF_PROTOCOL_OGNTP) {
    snprintf_P ( offset, size,
      PSTR("\
<tr>\
<th align=left>IGC key (HEX)</th>\
<td align=right>\
<INPUT type='text' name='igc_key' maxlength='32' size='32' value='%08X%08X%08X%08X'>\
</td>\
</tr>"),
    settings->igc_key[0]? 0x88888888 : 0,
    settings->igc_key[1]? 0x88888888 : 0,
    settings->igc_key[2]? 0x88888888 : 0,
    settings->igc_key[3]? 0x88888888 : 0);
       /* mask the key from prying eyes */

    len = strlen(offset);
    offset += len;
    size -= len;
  }
#endif

  snprintf_P ( offset, size,
    PSTR("\
</table>\
<p align=center><INPUT type='submit' value='Save and restart'></p>\
</form>\
</body>\
</html>")
  );
  len = strlen(offset);
  offset += len;

  Serial.print(F("AdvStgs page size: ")); Serial.print(offset-Settings_temp);
  Serial.print(F(" out of allocated: ")); Serial.println(totalsize);
  // currently about ------
  serve_html(Settings_temp);
  Serial.print(F("Free memory (AdvStgs page staging): "));
  Serial.println(ESP.getFreeHeap());
  delay(1000);
  free(Settings_temp);
}

void handleRoot() {

  Serial.println(F("handleRoot()..."));

  stop_bluetooth((size_t)50000);

  int sec = millis() / 1000;
  int min = sec / 60;
  int hr = min / 60;

  float vbat = Battery_voltage();
  uint8_t low_voltage = (vbat <= Battery_threshold()? 1 : 0);
  if (vbat < 2.0) {
      vbat = 0.0;
      low_voltage = 2;
  }
  bool usbpwr = ESP32_onExternalPower();
  float vusb = (usbpwr? 0.001*(float)ESP32_VbusVoltage() : 0.0);

  //time_t timestamp = ThisAircraft.timestamp;
  int hour   = gnss.time.hour();
  int minute = gnss.time.minute();
  unsigned int sats = gnss.satellites.value(); // Number of satellites in use (u32)
  char str_lat[16];
  char str_lon[16];
  char str_alt[16];
  char str_vbat[8];
  char str_vusb[8];

  size_t size = 4900;
  char *Root_temp = (char *) malloc(size);
  if (Root_temp == NULL) {
      Serial.println(F(">>> not enough RAM"));
      return;
  }

  dtostrf(ThisAircraft.latitude,  8, 4, str_lat);
  dtostrf(ThisAircraft.longitude, 8, 4, str_lon);
  dtostrf(ThisAircraft.altitude-ThisAircraft.geoid_separation, 7, 1, str_alt);   // MSL
  dtostrf(vbat, 4, 2, str_vbat);
  if (usbpwr)
      dtostrf(vusb, 4, 2, str_vusb);
  else if (ESP32_onBatteryPower())
      strcpy(str_vusb,"none");
  else
      strcpy(str_vusb,"????");

  char adsb_s[88];
  if (settings->rx1090 == ADSB_RX_NONE && settings->gdl90_in == DEST_NONE) {
      adsb_s[0] = '\0';
  } else if (settings->rx1090 != ADSB_RX_NONE && ! rx1090found) {
      strcpy(adsb_s, "<tr><th align=left>ADS-B receiver not present</th></tr>");
  } else {
      snprintf(adsb_s, 88,
         "<tr><th align=left>ADS-B Packets</th><td>&nbsp;</td><td align=right>%d</td></tr>",
         adsb_packets_counter);
  }

  char tx_s[8];
  if (settings->txpower == RF_TX_POWER_OFF) {
      strcpy(tx_s, "OFF");
  } else {
      snprintf(tx_s, 8, "%u", tx_packets_counter);
  }

  char traffics[24];
  int acrfts_counter = Traffic_Count();   // maxrssi and adsb_acfts are byproducts
  if (acrfts_counter == 0) {
      traffics[0] = '\0';
  } else if (adsb_acfts > 0) {
      snprintf(traffics, 24, "(%d ADS-B)", adsb_acfts);
  } else {
      snprintf(traffics, 24, "(max RSSI %d)", maxrssi);
  }

  size_t spiffs_used = SPIFFS.usedBytes();
  size_t spiffs_available = SPIFFS.totalBytes() - spiffs_used;

  yield();

  snprintf_P ( Root_temp, size,
    PSTR("<html>\
 <head>\
  <meta name='viewport' content='width=device-width, initial-scale=1'>\
  <title>SoftRF status</title>\
 </head>\
<body>\
 <table width=100%%>\
  <tr>\
   <td align=center><input type=button onClick=\"location.href='/reboot'\" value='Reboot'></td>\
   <td align=center><h2>SoftRF status</h2></td>\
   <td align=center><input type=button onClick=\"location.href='/about'\" value='About'></td>\
  </tr>\
  <tr><td>&nbsp;</td><th align=center>(%s)</th><td>&nbsp;</td></tr>\
 </table>\
 <table width=100%%>\
  <tr><th align=left>Device ID</th><td align=right>%06X</td></tr>\
  <tr><th align=left>Transmitted ID</th><td align=right>%06X</td></tr>\
  <tr><th align=left>Software Version</th><td align=right>%s&nbsp;&nbsp;%s</td></tr>"
#if !defined(ENABLE_AHRS)
 "</table><table width=100%%>\
  <tr><td align=left><table><tr><th align=left>GNSS&nbsp;&nbsp;</th><td align=right>%s</td></tr></table></td>\
  <td align=center><table><tr><th align=left>Radio&nbsp;&nbsp;</th><td align=right>%s</td></tr></table></td>\
  <td align=right><table><tr><th align=left>Baro&nbsp;&nbsp;</th><td align=right>%s</td></tr></table></td></tr>\
  </table><table width=100%%>"
#else
 "<tr><td align=left><table><tr><th align=left>GNSS&nbsp;&nbsp;</th><td align=right>%s</td></tr></table></td>\
  <td align=right><table><tr><th align=left>Radio&nbsp;&nbsp;</th><td align=right>%s</td></tr></table></td></tr>\
  <tr><td align=left><table><tr><th align=left>Baro&nbsp;&nbsp;</th><td align=right>%s</td></tr></table></td>\
  <td align=right><table><tr><th align=left>AHRS&nbsp;&nbsp;</th><td align=right>%s</td></tr></table></td></tr>"
#endif /* ENABLE_AHRS */
 "<tr><th align=left>Uptime</th><td align=right>%02d:%02d:%02d</td></tr>\
  <tr><th align=left>Free memory</th><td align=right>%u</td></tr>\
  <tr><th align=left>Battery voltage</th><td align=right><font color=%s>%s</font></td></tr>\
  <tr><th align=left>USB voltage</th><td align=right>%s</td></tr>\
 </table>\
 <table width=100%%>\
  <tr><th align=left>Packets</th>\
   <td align=middle>Tx %s</td>\
   <td align=right>Rx %u</td>\
  </tr>\
  %s\
  <tr>\
   <th align=left>Current traffic</th>\
   <td align=left>%d</td>\
   <td align=right>%s</td>\
  </tr>\
  <tr>\
   <th align=right>Landed-Out Mode %s&nbsp;</th>\
   <td align=left>\
    <input type=button onClick=\"location.href='/landed_out'\" value='%s'>\
   </td>\
   <td>&nbsp;</td>\
  </tr>\
 </table>\
 <hr>\
 <table width=100%%>\
  <tr><td><h3 align=left>Most recent GNSS fix</h3></td><td>&nbsp;</td><td align=right>%s</td>\
  <tr><th align=left>UTC Time</th><td><font color=%s>(%s)</font></td><td align=right>%02d:%02d</td></tr>\
  <tr><th align=left>Satellites</th><td>&nbsp;</td><td align=right>%d</td></tr>\
  <tr><th align=left>Latitude</th><td>&nbsp;</td><td align=right>%s</td></tr>\
  <tr><th align=left>Longitude</th><td>&nbsp;</td><td align=right>%s</td></tr>\
  <tr><td align=left><b>Altitude</b>&nbsp;%s</td><td align=left>%s</td><td align=right>%s</td></tr>\
 </table>\
 <hr>\
 <table width=100%%>\
  <tr>\
   <td align=left><input type=button onClick=\"location.href='/settings'\" value='Basic Settings'></td>\
   <td align=center><input type=button onClick=\"location.href='/advstgs'\" value='Advanced Settings'></td>\
   <td align=right><input type=button onClick=\"location.href='/firmware'\" value='Firmware update'></td>\
  </tr>\
 </table>\
 <hr>\
 <b>Settings file:</b>\
 <table width=100%%>\
  <tr>\
   <td><input type=button onClick=\"location.href='/settingsdownload'\" value='Download'></td>\
   <td><input type=button onClick=\"location.href='/settingsupload'\" value='Upload'></td>\
   <td><input type=button onClick=\"location.href='/settingsbackup'\" value='Backup'></td>\
   <td><input type=button onClick=\"location.href='/settingsswap'\" value='Restore/Swap'></td>\
  </tr>\
 </table>\
 <hr>\
 <b>Flight Log:</b>&nbsp;&nbsp;%s\
 <table width=100%%>\
  <tr>%s%s</tr>\
 </table>\
 <hr>\
 <b>Flash memory:</b>\
 <table width=100%%>\
  <tr>\
   <td align=center>%d bytes used, %d available</td>\
   <td align=left><input type=button onClick=\"location.href='/listall'\" value='Manage Files'></td>\
  </tr>\
 </table>\
</body>\
</html>"),
    page_message(),
    (SoC->getChipId() & 0x00FFFFFF), ThisAircraft.addr, SOFTRF_FIRMWARE_VERSION,
    (SoC == NULL ? "NONE" : SoC->name),
    GNSS_name[hw_info.gnss],
    (rf_chip   == NULL ? "NONE" : rf_chip->name),
    (baro_chip == NULL ? "NONE" : baro_chip->name),
#if defined(ENABLE_AHRS)
    (ahrs_chip == NULL ? "NONE" : ahrs_chip->name),
#endif /* ENABLE_AHRS */
    hr, min % 60, sec % 60, ESP.getFreeHeap(),
    (low_voltage==1? "red" : (low_voltage==0? "green": "black")), str_vbat, str_vusb,
    tx_s, rx_packets_counter, adsb_s, acrfts_counter, traffics,
    (landed_out_mode? "Active" : "Off"),
    (landed_out_mode? "Stop" : "Activate"),
    ((hw_info.model == SOFTRF_MODEL_PRIME_MK2) ?
 "<input type=button onClick=\"location.href='/gps_reset'\" value='Reset GNSS'>" : ""),
    (isValidGNSSFix() ? (leap_seconds_valid()? "green" : "black") : "red"),
    (isValidGNSSFix() ? (leap_seconds_valid()? "valid fix" : "leap seconds assumed") : "no valid fix"),
    hour, minute, sats, str_lat, str_lon,
    (isValidGNSSFix() ?
          (ThisAircraft.geoid_separation==0 ? "(above ellipsoid - MSL n.a.)"
           : "(above MSL)") : ""),
    (isValidGNSSFix() ? (ThisAircraft.airborne? "(Airborne)" : "(Not Airborne)") : ""), str_alt,
    FlightLogStatus(),
#if defined(USE_SD_CARD)
(PSRAMbufUsed?
 "<td align=center><input type=button onClick=\"location.href='/ramfltlog'\" value='Download Current'></td>\
  <td align=center><input type=button onClick=\"location.href='/delramlog'\" value='Clear Current'></td>" : ""),
(SD_is_mounted?
 "<td align=center><input type=button onClick=\"location.href='/lastsdlog'\" value='Download Latest'></td>\
  <td align=center><input type=button onClick=\"location.href='/listlogs'\" value='List Files'></td>\
  <td align=center><input type=button onClick=\"location.href='/trashlogs'\" value='Clear Files'></td>"  : 
 "<td align=center><input type=button onClick=\"location.href='/listlogs'\" value='List Archived'></td>\
  <td align=center><input type=button onClick=\"location.href='/clearlogs'\" value='Clear Archive'></td>"),
#else
(PSRAMbufUsed?
 "<td align=center><input type=button onClick=\"location.href='/ramfltlog'\" value='Download Current'></td>\
  <td align=center><input type=button onClick=\"location.href='/delramlog'\" value='Clear Current'></td>" : ""),
 "<td align=center><input type=button onClick=\"location.href='/listlogs'\" value='List Archived'></td>\
  <td align=center><input type=button onClick=\"location.href='/clearlogs'\" value='Clear Archive'></td>",
#endif
    spiffs_used, spiffs_available
  );
  Serial.print(F("Status page size: ")); Serial.print(strlen(Root_temp));
  Serial.print(F(" out of allocated: ")); Serial.println(size);
  // currently about 3950
  serve_html(Root_temp);
  free(Root_temp);
  yield();

#if 0
  Serial.println(F("Files in SPIFFS:"));
//  if (!SPIFFS.begin(true)) {
//      Serial.println(F("Error mounting SPIFFS"));
//      return;
//  }
  File root = SPIFFS.open("/");
  if (! root) {
      Serial.println(F("Cannot open SPIFFS root"));
      return;
  }
  File file = root.openNextFile();
  while(file) {
      Serial.print("... ");
      Serial.print(file.name());
      Serial.print("  [");
      Serial.print(file.size());
      Serial.println(" bytes]");
      file = root.openNextFile();
  }
  file.close();
  root.close();
  Serial.println(F("... end of files in SPIFFS"));
#endif
}

void handleInput() {
  Serial.println(F("Settings from web page:"));
  for ( uint8_t i = 0; i < server.args(); i++ ) {
    if (server.argName(i).equals(stgdesc[STG_PSK].label)) {
        if (! server.arg(i).equals("********")) {
            strncpy(settings->psk, server.arg(i).c_str(), sizeof(settings->psk)-1);
            settings->psk[sizeof(settings->psk)-1] = '\0';
        }
#if defined(USE_OGN_ENCRYPTION)
    } else if (server.argName(i).equals("igc_key")) {
        uint32_t key;
        char buf[32 + 1];
        server.arg(i).toCharArray(buf, sizeof(buf));
        key = strtoul(buf + 24, NULL, 16);
        if (key != 0x88888888)  settings->igc_key[3] = key;
        buf[24] = 0;
        key = strtoul(buf + 16, NULL, 16);
        if (key != 0x88888888)  settings->igc_key[2] = key;
        buf[16] = 0;
        key = strtoul(buf + 8, NULL, 16);
        if (key != 0x88888888)  settings->igc_key[1] = key;
        buf[8] = 0;
        key = strtoul(buf + 0, NULL, 16);
        if (key != 0x88888888)  settings->igc_key[0] = key;
#endif
    } else {
        char p[32];
        char q[32];
        server.argName(i).toCharArray(p, sizeof(p));
        server.arg(i).toCharArray(q, sizeof(q));
        Serial.print(p);
        Serial.print(",");
        Serial.print(q);
        int i = find_setting(p);
        if (i == STG_NONE)
            Serial.print("  - no matching label");
        if (load_setting(i,q) == false)
            Serial.print("  - error");
        Serial.println("");
    }
    yield();
  }                // end for (server.args())

  // make some adjustments to settings
  Adjust_Settings();

  if (SPIFFS.exists("/settingb.txt"))
      SPIFFS.remove("/settingb.txt");
  SPIFFS.rename("/settings.txt","/settingb.txt");
  save_settings_to_file();   // this also shows the new settings
  if (! SPIFFS.exists("/settings.txt")) {   // saving the file failed
      SPIFFS.rename("/settingb.txt","/settings.txt");
      server.send(500, textplain, "cannot save the new settings file");
      return;
  }
  settingsreboot(200, "New settings saved.");
}

void handleNotFound() {

  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += ( server.method() == HTTP_GET ) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";

  for ( uint8_t i = 0; i < server.args(); i++ ) {
    message += " " + server.argName ( i ) + ": " + server.arg ( i ) + "\n";
  }

  server.send ( 404, textplain, message );
}

void confirmFormat()
{
  char buf[240];
  snprintf_P ( buf, 240, PSTR(
 "<html>%s\
 <h3>Format SPIFFS?  All files will be lost!</h3><br><br>\
 <input type=button onClick=\"location.href='/doformat'\" value='Confirm'>\
 </html>"), home);
  server.send(200, texthtml, buf);
}

enum {
    FILE_OP_DELETE,
    FILE_OP_TRASH,
    FILE_OP_DELTRASH,
    FILE_OP_DELETE_LOGS,
    FILE_OP_TRASH_LOGS,
    FILE_OP_TRASH_ALL,
    FILE_OP_EMPTY_TRASH,
    FILE_OP_DELALRMLOG
};

void confirmDelete(int file_op, const char *filename=NULL)
{
  char f[32];
  f[0] = 's';
  f[1] = '\0';
  const char *g = "all";
  char h[32];
  h[0] = '\0';     // default, will be overwritten below
  if (filename) {
      f[0] = ' ';
      strcpy(f+1,filename);
      g = "the";
      if (file_op == FILE_OP_DELETE)
          strcpy(h,"delete~");
      else if (file_op == FILE_OP_TRASH)
          strcpy(h,"trash~");
      else
          strcpy(h,"deltrash~");
      strcpy(h+strlen(h),filename);  // with the extension
  } else {
      if (file_op == FILE_OP_DELETE_LOGS)
          strcpy(h,"doclearlogs");
      else if (file_op == FILE_OP_TRASH_LOGS)
          strcpy(h,"dotrashlogs");
      else if (file_op == FILE_OP_TRASH_ALL)
          strcpy(h,"dotrashall");
      else if (file_op == FILE_OP_DELALRMLOG) {
          g = "the alarm log";
          f[0] = '\0';
          strcpy(h,"delalrmlog");
      }
      else
          strcpy(h,"doemptytrash");
  }
  bool trash = (file_op == FILE_OP_TRASH || file_op == FILE_OP_TRASH_LOGS || file_op == FILE_OP_TRASH_ALL);
  char buf[240];
  snprintf_P ( buf, 240, PSTR(
 "<html>%s\
 <h3>%s %s file%s%s?</h3>\
 <input type=button onClick=\"location.href='/%s'\" value='Confirm'>\
 </html>"), home, (trash? "Move" : "Delete"), g, f, (trash? " to trash" : ""), h);
  server.send(200, texthtml, buf);
}

// handle special URL for file deletion confirmation
bool confirm_del_file(const char *filename)
{
  if (strncmp(filename,"confdel~",8)!=0)
      return false;
  confirmDelete(FILE_OP_DELETE, &filename[8]);
  return true;
}

// handle special URL for confirmation of move file to trash
bool confirm_trash_file(const char *filename)
{
  if (strncmp(filename,"conftrash~",10)!=0)
      return false;
  confirmDelete(FILE_OP_TRASH, &filename[10]);
  return true;
}

// handle special URL for confirmation of deleting file in trash
bool confirm_deltrash_file(const char *filename)
{
  if (strncmp(filename,"confdeltrash~",13)!=0)
      return false;
  confirmDelete(FILE_OP_DELTRASH, &filename[13]);
  return true;
}

// handle special URL for file deletion in SPIFFS
bool delete_file(const char *filename)
{
  if (strncmp(filename,"delete~",7)!=0)
      return false;
  String filepath = "/";
  filepath += &filename[7];
  if (SPIFFS.exists(filepath)) {
      SPIFFS.remove(filepath);
      char buf[148];
      snprintf_P ( buf, 148, PSTR("<h3>Deleted file %s</h3>%s"), &filename[7], home);
      server.send(200, texthtml, buf);
      return true;
  }
  char buf[148];
  snprintf_P ( buf, 148, PSTR("<h3>Cannot delete file SPIFFS%s - not found</h3>%s"),
                                     filepath.c_str(), home);
  server.send(404, texthtml, buf);
  return true;
}

// handle special URL for moving file from SD/logs to SD/logs/old
bool trash_file(const char *filename)
{
  if (strncmp(filename,"trash~",6)!=0)
      return false;
  String filepath = "/logs/";
  filepath += &filename[6];
  if (SD.exists(filepath)) {
      if (filepath.endsWith("sdlog.txt"))
          closeSDlog();
      else if (filepath.endsWith("NMEA.txt"))
          closeNMEAlog();
      String oldpath = "/logs/old/";
      oldpath += &filename[6];
      if (SD.exists(oldpath))
          SD.remove(oldpath);
      SD.rename(filepath,oldpath);
      char buf[148];
      snprintf_P ( buf, 148, PSTR("<h3>SD%s moved to /old/</h3>%s"), filepath.c_str(), home);
      server.send(200, texthtml, buf);
      return true;
  }
  char buf[148];
  snprintf_P ( buf, 148, PSTR("<h3>Cannot move file SD%s - not found</h3>%s"),
                                     filepath.c_str(), home);
  server.send(404, texthtml, buf);
  return true;
}

// handle special URL for deletion of file within SD/logs/old
bool del_trash(const char *filename)
{
  if (strncmp(filename,"deltrash~",9)!=0)
      return false;
  String filepath = "/logs/old/";
  filepath += &filename[9];
  if (SD.exists(filepath)) {
      SD.remove(filepath);
      char buf[148];
      snprintf_P ( buf, 148, PSTR("<h3>Deleted file SD%s</h3>%s"),
                                     filepath.c_str(), home);
      server.send(200, texthtml, buf);
      return true;
  }
  char buf[148];
  snprintf_P ( buf, 148, PSTR("<h3>Cannot delete file SD%s - not found</h3>%s"),
                                     filepath.c_str(), home);
  server.send(404, texthtml, buf);
  return true;
}

// This function is called from onNotFound() and attempts to
// send the requested file to the client (if it exists)
// It also handles the processing of requests about specific files
// such as downloading or deleting a flight log

bool handleFileRead(String path) {
  Serial.println("handleFileRead: " + path);
  if (path.endsWith("/"))
    return false;
  if (! path.startsWith("/"))
    path = "/" + path;
  //size_t slash = path.find_last_of("/");
  //slash = ((slash == std::string::npos)? 0 : slash+1);
  // - can't seem to make that compile, so roll our own:
  const char *filename = path.c_str();
  int slash = strlen(filename);
  while (slash > 0) {
      if (filename[slash-1] == '/')
          break;
      --slash;
  }
  filename += slash;    // point to after the last slash
  if (confirm_del_file(filename) == true)       // if special URL for file deletion confirmation
      return true;
      // avoid the handleNotFound() message
  if (confirm_trash_file(filename) == true)     // if URL for confirm SD file move to trash
      return true;
  if (confirm_deltrash_file(filename) == true)  // if URL for confirm deletion of SD trash file
      return true;
  if (delete_file(filename) == true)            // if special URL for file deletion
      return true;
  if (trash_file(filename) == true)             // if special URL for SD file move to trash
      return true;
  if (del_trash(filename) == true)              // if special URL for deletion of SD trash file
      return true;
  // otherwise... // look in SPIFFS first
  char zpath[40];
  strncpy(zpath,filename-1,40);  // includes the "/"
  bool is_igc = (strlen(filename)==12 && strncmp(filename+8,".IGC",4)==0);
  if (is_igc) {
    char lastchar = zpath[12];
    zpath[12]='Z';
    if (SPIFFS.exists(zpath) && lastchar == 'C') {
      // IGC requested but IGZ exists - first decompress then download
      size_t size = 0;
      File file = SPIFFS.open(zpath);       
      if (file)  size = file.size();
      file.close();
      if (size == 0)  return false;
      if (6*size > PSRAMavailable()) {
          server.send(500, textplain,
              "Not enough RAM space, save & clear current flight log first");
          return true;  // avoid the handleNotFound() message
      }
      suspendFlightLog();           // close SPIFFS file, pause PSRAM logging
      if (decompressfile(zpath)) {
          PSRAMbuf[PSRAMbufUsed] = '\0';
          serve_file(file, filename, PSRAMbuf);    // 'file' is a dummy argument
      } else {
          Serial.println("decompression failed");
          server.send(500, textplain, "decompression failed");
      }
      delay(2000);
      resumeFlightLog();
      return true;
    }
  }
  // else fall through - includes ".IGZ", resulting in download as-is
  if (SPIFFS.exists(zpath)) {
      if (strcmp(zpath,"/alarmlog.txt")==0 && AlarmLogOpen) {
          AlarmLog.close();
          AlarmLogOpen = false;
      }
      File file = SPIFFS.open(zpath, FILE_READ);
      if (file) {
          serve_file(file, filename);
          delay(500);
          file.close();
      }
      Serial.println(String(F("\tSent file: SPIFFS")) + path);
      return true;
  }
#if defined(USE_SD_CARD)
  // if not found in SPIFFS, look on SD card
  if (strncmp(path.c_str(),"/logs/",6)==0) {
      strncpy(zpath,path.c_str(),40);
  } else {
      strcpy(zpath,"/logs/");
      strncpy(zpath+6,filename,34);
  }
  if (SD.exists(zpath)) {
      if (strcmp(zpath,"/logs/sdlog.txt")==0)
          closeSDlog();
      else if (path.endsWith("NMEA.txt"))
          closeNMEAlog();
      File file = SD.open(zpath, FILE_READ);
      if (file) {
          serve_file(file, filename);
          delay(500);
          file.close();
      }
      Serial.println(String(F("\tSent file: SD")) + zpath);
      return true;
  }
#endif
  Serial.println(String(F("\tFile Not Found: ")) + zpath);
  return false;
}

// list flight log (or all) files, in SD/logs/ or in SPIFFS/

int igc2num(char c)
{
    if (c >= '0' && c <= '9')
        return (c - '0');
    if (c >= 'A' && c <= 'Z')
        return (c - 'A' + 10);
    return 0;
}

#define FILELSTSIZ 8000

// the modes list_files() can work in:
enum {
    LIST_SPIFFS_LOGS,
    LIST_SPIFFS_ALL,
    LIST_SD_LOGS,
    LIST_SD_ALL,
    LIST_SD_OLD
};

void list_files(int mode)
{
  char *filelist = (char *) malloc(FILELSTSIZ);
  if (! filelist) {
      Serial.println(F("cannot allocate memory for file list"));
      server.send ( 500, textplain, "Cannot allocate memory for file list");
      return;
  }
#if defined(USE_SD_CARD)
  if (mode==LIST_SD_LOGS || mode==LIST_SD_ALL || mode==LIST_SD_OLD) {
      // update directory data for open files
      // (mainly current NMEA log - flight logs are closed between block writes)
      //SD.sync();
      // not supported, need to flush() the individual files instead
      flushNMEAlog();
  }
#endif
  int len = 0;
  char *cp = filelist;
  int more;
  if (mode == LIST_SPIFFS_ALL || mode == LIST_SPIFFS_LOGS) {
      size_t spiffs_used = SPIFFS.usedBytes();
      size_t spiffs_available = SPIFFS.totalBytes() - spiffs_used;
  }
  File root;
  const char *folder = "/";
  const char *del_op = "confdel~";   // delete after confirmation
  const char *igc_suffix = ".IGC";
  if (mode == LIST_SPIFFS_LOGS || mode == LIST_SPIFFS_ALL) {
      root = SPIFFS.open("/");
      igc_suffix = ".IGZ";
      size_t spiffs_used = SPIFFS.usedBytes();
      size_t spiffs_available = SPIFFS.totalBytes() - spiffs_used;
      Serial.println(F("Files in SPIFFS/:"));
      snprintf(filelist, FILELSTSIZ, "%s%sFiles in flash: (%d bytes available)<br><br>",
           home, (mode==LIST_SPIFFS_LOGS? "Flight Log " : ""), spiffs_available);
#if defined(USE_SD_CARD)
  } else if (mode == LIST_SD_OLD) {
      folder = "/logs/old/";
      del_op = "confdeltrash~";     // delete after confirmation
      root = SD.open("/logs/old");
      Serial.println(F("Files in SD/logs/old:"));
      snprintf(filelist, FILELSTSIZ, "%sFiles in SD/logs/old:<br><br>", home);
#if 0
  } else if (mode == LIST_SD_LOGS) {
      folder = "/logs/";
      del_op = "conftrash~";        // move to /old
      root = SD.open("/logs");
      Serial.println(F("Flight log files in SD/logs:"));
      snprintf(filelist, FILELSTSIZ, "%sFlight log files in SD/logs:<br><br>", home);
#endif
  } else {    // LIST_SD_ALL or LIST_SD_LOGS
      folder = "/logs/";
      del_op = "conftrash~";        // move to /old
      root = SD.open("/logs");
      Serial.println(F("Files in SD/logs:"));
      snprintf(filelist, FILELSTSIZ, "%sFiles in SD/logs:<br><br>", home);
#endif
  }
  if (! root) {
      Serial.println(F("Cannot open files folder"));
      server.send ( 500, textplain, "Cannot open files folder");
      free(filelist);
      return;
  }
  more = strlen(cp);
  len += more;
  cp  += more;
  int nfiles = 0;
  int nlogs = 0;
  File file = root.openNextFile();
  for (; file; file=root.openNextFile()) {
      if (file.isDirectory())
          continue;
      Serial.print("  ");
      char fn[32];
      strncpy(fn,file.name(),32);
      Serial.print(fn);
      Serial.print("  [");
      Serial.print(file.size());
      Serial.println(" bytes]");
      String file_name = fn;
      bool is_igc = file_name.endsWith(igc_suffix);
      if (is_igc)
          ++nlogs;
      // show NMEA & SD logs too (actually all .txt files)
      bool is_txt = file_name.endsWith(".txt");
      if ((mode == LIST_SPIFFS_LOGS || mode == LIST_SD_LOGS) && (! is_igc) && (! is_txt))
          continue;
      if (len < FILELSTSIZ-160) {
        strcpy(cp, "&nbsp;<a href=\"");
        more = strlen(cp);
        len += more;
        cp  += more;
        int file_size = file.size();
        char buf[16];
        if (file_size)
          snprintf(buf, 16, "%d bytes", file_size);
        else
          strcpy(buf,"size unknown");
        if (is_igc) {
          if (mode == LIST_SPIFFS_LOGS || mode == LIST_SPIFFS_ALL)
              fn[11] = 'C';   // replace "IGZ" with "IGC" - will decompress before download
          int year = igc2num(fn[0]);
          snprintf(cp, FILELSTSIZ-len,
            "%s%s\">%s</a>&nbsp;[20%d%d-%02d-%02d]&nbsp;[%s]",
            folder, fn, file.name(), (year<4? 3 : 2), year, igc2num(fn[1]), igc2num(fn[2]), buf);
        } else if (file_name.endsWith("NMEA.txt")) {
            int year = igc2num(fn[0]);
            snprintf(cp, FILELSTSIZ-len,
              "%s%s\">%s</a>&nbsp;[20%d%d-%02d-%02d]&nbsp;[%s]",
              folder, fn, fn, (year<4? 3 : 2), year, igc2num(fn[1]), igc2num(fn[2]), buf);
        } else {
          snprintf(cp, FILELSTSIZ-len,
            "%s%s\">%s</a>&nbsp;&nbsp;&nbsp;&nbsp;[%s]", folder, fn, fn, buf);
        }
        more = strlen(cp);
        len += more;
        cp  += more;
        // use special URLs for file deletion ops
        file_name = del_op;
        file_name += file.name();
        snprintf(cp, FILELSTSIZ-len, "&nbsp;&nbsp;<a href=\"/%s\">Delete</a><br>", file_name.c_str());            
        more = strlen(cp);
        len += more;
        cp  += more;
      } else {
        Serial.println("... and more ...");
        snprintf(cp, FILELSTSIZ-len, "... and more ...<br>");
        more = strlen(cp);
        len += more;
        cp  += more;
        break;
      }
      ++nfiles;
      yield();
      //file = root.openNextFile();
  }
  if (nfiles == 0) {
      Serial.println("  (none)");
      snprintf(cp, FILELSTSIZ-len, "&nbsp;&nbsp;(none)<br>");
      more = strlen(cp);
      len += more;
      cp  += more;
  }
  file.close();
  root.close();
  const char *label = "Back to Home";
  const char *url = "/";
  if (len < FILELSTSIZ-120  && nfiles > 0) {
      if (mode == LIST_SPIFFS_LOGS || mode == LIST_SPIFFS_ALL) {
          label = "Delete All Flight Logs";
          url = "/clearlogs";
      }
      if (mode == LIST_SD_LOGS && nlogs > 0) {
          label = "Move All Flight Logs to Trash";
          url = "/trashlogs";
      }
      if (mode == LIST_SD_ALL) {
          label = "Move All Files to Trash";
          url = "/trashall";
      }
      if (mode == LIST_SD_OLD) {
          label = "Empty Trash";
          url = "/emptytrash";
      }
      snprintf(cp, FILELSTSIZ-len, "<br><br><input type=button onClick=\"location.href='%s'\" value='%s'>",
                     url, label);
      more = strlen(cp);
      len += more;
      cp  += more;
  }
  if (len < FILELSTSIZ-120) {
    if (mode == LIST_SD_LOGS || mode == LIST_SD_ALL) {
      label = "Open Trash";
      url = "/listsdold";
      snprintf(cp, FILELSTSIZ-len, "<br><br><input type=button onClick=\"location.href='%s'\" value='%s'>",
                     url, label);
      more = strlen(cp);
      len += more;
      cp  += more;
    }
  }

  if (mode == LIST_SPIFFS_ALL) {

    if (len < FILELSTSIZ-400) {

      snprintf_P(cp, FILELSTSIZ-len, PSTR(
 "<br>&nbsp;<hr>&nbsp;<br>\
 <table width=100%%>\
  <tr>\
   <td><input type=button onClick=\"location.href='/clralrmlog'\" value='Clear Alarm Log'></td>\
   <td><input type=button onClick=\"location.href='/wavupload'\" value='Upload waves.tar'></td>"
#if defined(USE_EGM96)
  "<td><input type=button onClick=\"location.href='/egmupload'\" value='Upload egm96s.dem'></td>"
#endif
  "<td><input type=button onClick=\"location.href='/format'\" value='FORMAT flash filesystem'></td>\
  </tr>\
 </table>"));

    } else if (len < FILELSTSIZ-120) {
      label = "Upload any file";
      url = "/anyupload";
      snprintf(cp, FILELSTSIZ-len, "<br><br><input type=button onClick=\"location.href='%s'\" value='%s'>",
                     url, label);
    }
  }

  serve_html(filelist);
  delay(500);
  free(filelist);
}

//void list_spiffs_logs() { list_files(LIST_SPIFFS_LOGS); }
void list_spiffs_all()  { list_files(LIST_SPIFFS_ALL);  }
//void list_sd_logs()     { list_files(LIST_SD_LOGS);     }
void list_sd_old()      { list_files(LIST_SD_OLD);      }
void list_sd_all()      { list_files(LIST_SD_ALL);      }

// automatically choose to list SPIFFS or SD
void list_logs()
{
#if defined(USE_SD_CARD)
    if (SD_is_mounted) {
        //closeSDlog();
        //closeNMEAlog();
        // - only close logs before download or deletion or shutdown
        list_files(LIST_SD_LOGS);
    } else {
        list_files(LIST_SPIFFS_LOGS);
    }
#else
    list_files(LIST_SPIFFS_LOGS);
#endif
};

void doClearLogs()
{
    closeFlightLog();
    int nfiles = 0;
    File root = SPIFFS.open("/");
    if (! root) {
        Serial.println(F("Cannot open SPIFFS/"));
        server.send ( 500, textplain, "(cannot open SPIFFS/)");
        return;
    }
    Serial.println(F("clearing flight logs..."));
    File file = root.openNextFile();
    String file_name;
    while(file) {
        file_name = "/";
        file_name += file.name();
        file = root.openNextFile();
        if (file_name.endsWith(".IGZ")) {
            SPIFFS.remove(file_name);
            Serial.print("...");
            Serial.println(file_name);
            ++nfiles;
        }
        yield();
    }
    file.close();
    root.close();
    Serial.print("Deleted ");
    Serial.print(nfiles);
    Serial.println(" flight log files");
    char buf[148];
    snprintf_P ( buf, 148, PSTR("<h3>Deleted %d flight log files</h3>%s"), nfiles, home);
    server.send(200, texthtml, buf);
}

void doClearSD(bool allfiles)
{
#if defined(USE_SD_CARD)
    closeFlightLog();
    if (allfiles) {
        closeSDlog();
        closeNMEAlog();
    }
    int nmoved = 0;
    int ndeleted = 0;
    File root = SD.open("/logs");
    if (! root) {
        Serial.println(F("Cannot open SD/logs"));
        server.send ( 500, textplain, "(cannot open SD/logs)");
        return;
    }
    if (allfiles)
        Serial.println(F("clearing all files..."));
    else
        Serial.println(F("clearing flight logs..."));
    File file = root.openNextFile();
    String file_name;
    while(file) {
        file_name = file.name();
        String logpath;
        logpath = "/logs/";
        logpath += file_name;
        bool move = (file.size() > 4000);
        file = root.openNextFile();
        if (allfiles || file_name.endsWith(".IGC") || file_name.endsWith(".igc")) {
            if (move) {            // move large files
                String oldpath;
                oldpath = "/logs/old/";
                oldpath += file_name;
                if (SD.exists(oldpath)) {
                    if (SD.remove(oldpath))
                        Serial.print("removed: ");    // old file with same name
                    else
                        Serial.print("failed to remove: ");
                    Serial.println(oldpath);
                }
                if (SD.rename(logpath,oldpath)) {
                    Serial.print("moved: ");
                    ++nmoved;
                } else {
                    Serial.print("failed to move: ");
                }
            } else {       // delete small files
                if (SD.remove(logpath)) {
                    Serial.print("removed: ");
                    ++ndeleted;
                } else {
                    Serial.print("failed to remove: ");
                }
            }
            Serial.println(logpath);
        }
        yield();
    }
    file.close();
    root.close();
    Serial.print("Moved ");
    Serial.print(nmoved);
    Serial.print(" files to /old and deleted ");
    Serial.print(ndeleted);
    Serial.println(" small files");
    char buf[148];
    snprintf_P ( buf, 148, PSTR("<h3>Moved %d files & deleted %d</h3>%s"),
                                     nmoved, ndeleted, home);
    server.send(200, texthtml, buf);
#endif
}

void doClearSDLogs() { doClearSD(false); }
void doClearSDAll()  { doClearSD(true); }

void doEmptyTrash()
{
#if defined(USE_SD_CARD)
    int nfiles = 0;
    File root = SD.open("/logs/old");
    if (! root) {
        Serial.println(F("Cannot open SD/logs/old"));
        server.send ( 500, textplain, "(cannot open SD/logs/old)");
    }
    Serial.println(F("deleting files in SD/logs/old..."));
    File file = root.openNextFile();
    String file_name;
    while(file) {
        file_name = "/logs/old/";
        file_name += file.name();
        file = root.openNextFile();
        //if (file_name.endsWith(".IGC") || file_name.endsWith(".igc")) {
        //rather, delete all files in /logs/old/
            Serial.print("...");
            Serial.println(file_name.c_str());
            SD.remove(file_name.c_str());
            ++nfiles;
        //}
        yield();
    }
    file.close();
    root.close();
    Serial.print("Deleted ");
    Serial.print(nfiles);
    Serial.println(" files from /old");
    char buf[148];
    snprintf_P ( buf, 148, PSTR("<h3>Deleted %d files in /old/</h3>%s"), nfiles, home);
    server.send(200, texthtml, buf);
#endif
}

void handleClearLogs()
{
    confirmDelete(FILE_OP_DELETE_LOGS);
}

void handleClearSDLogs()
{
    confirmDelete(FILE_OP_TRASH_LOGS);
}

void handleClearSDAll()
{
    confirmDelete(FILE_OP_TRASH_ALL);
}

void handleEmptyTrash()
{
    confirmDelete(FILE_OP_EMPTY_TRASH);
}

void confdelalarmlog()
{
    confirmDelete(FILE_OP_DELALRMLOG);
}

void Web_setup()
{
  server.on ( "/", handleRoot );

  server.on ( "/settings", handleSettings );
  server.on ( "/advstgs",  handleAdvStgs );

  server.on ( "/reboot", []() {
    Serial.println(F("Rebooting from web page..."));
    settingsreboot(200, "");
  } );

  server.on ( "/about", []() {
    serve_html(about_html);
  } );

  server.on ( "/show", []() {
    // put custom code here for debugging, for example:
    if (settings->rx1090)
        show_zone_stats();
  } );

  server.on ( "/landed_out", []() {
    if (landed_out_mode) {
        landed_out_mode = false;
        OLED_msg("NORMAL", "MODE");
        Serial.println("landed_out_mode off");
        server.send(200, texthtml,
          "<html><p align=center><h3 align=center>LANDED-OUT MODE STOPPED</h3></p></html>");
    } else {
        landed_out_mode = true;
        OLED_msg("LANDED", "OUT");
        Serial.println("landed_out_mode on");
        server.send(200, texthtml,
          "<html><p align=center><h3 align=center>LANDED-OUT MODE ACTIVATED</h3></p></html>");
    }
  } );

  server.on ( "/testmode", []() {
      test_mode = !test_mode;
      if (test_mode) {
          OLED_msg("TEST",   " MODE");
          Serial.println("Test Mode on");
          server.send(200, textplain, "TEST MODE ON");
      } else {
          OLED_msg("NOTEST", " MODE");
          Serial.println("Test Mode off");
          server.send(200, textplain, "TEST MODE OFF");
      }
      do_test_mode();
  } );

  server.on ( "/gps_reset", []() {
    Serial.println(F("Factory Reset GNSS..."));
    gnss_needs_reset = true;
    server.send(200, texthtml,
      "<html><p align=center><h3 align=center>Factory reset & cold-start GNSS...</h3></p></html>");
  } );

  server.on ( "/format", confirmFormat );
  server.on ( "/doformat", []() {
    closeFlightLog();
    clear_waves();
    Serial.println(F("Formatting spiffs..."));
    SPIFFS.format();
    server.send(200, textplain, "SPIFFS cleared, suggest saving settings now!");
  } );

  server.on ( "/wavupload", []() {
    char buf[320];
    set_upload(buf, "waves.tar (read instructions first)", "/dowavupld");
    serve_html(buf);
  } );
  server.on ("/dowavupld", HTTP_POST,  // if the client posts to the upload page
    [](){ server.send(200); },         // Send 200 to tell the client we are ready to receive
    wavUpload                          // Receive and save the file
  );

  server.on ( "/egmupload", []() {
    char buf[320];
    set_upload(buf, "egm96s.dem", "/doegmupld");
    serve_html(buf);
  } );
  server.on ("/doegmupld", HTTP_POST,  // if the client posts to the upload page
    [](){ server.send(200); },         // Send 200 to tell the client we are ready to receive
    egmUpload                          // Receive and save the file
  );

#if defined(USE_SD_CARD)
  server.on ( "/logupload", []() {
    char buf[320];
    set_upload(buf, "a file into SD/logs", "/dologupld");
    serve_html(buf);
  } );
  server.on ("/dologupld", HTTP_POST,  // if the client posts to the upload page
    [](){ server.send(200); },         // Send 200 to tell the client we are ready to receive
    sdlogUpload                        // Receive and save the file
  );
#endif

  server.on ( "/settingsupload", []() {
    char buf[320];
    set_upload(buf, "settings.txt", "/dostgupld");
    serve_html(buf);
  } );
  server.on ("/dostgupld", HTTP_POST,  // if the client posts to the upload page
    [](){ server.send(200); },        // Send 200 to tell the client we are ready to receive
    settingsupload                    // Receive and save the file
  );

  server.on ( "/settingsdownload", settingsdownload );
  server.on ( "/settingsbackup",   settingsbackup );
  server.on ( "/settingsswap",     settingsswap );

  server.on ( "/alarmlog", alarmlogfile );
  server.on ( "/clralrmlog", confdelalarmlog );
  server.on ( "/delalrmlog", []() {
    if (SPIFFS.exists("/alarmlog.txt")) {
        if (AlarmLogOpen) {
          AlarmLog.close();
          AlarmLogOpen = false;
        }
        SPIFFS.remove("/alarmlog.txt");
    }
    server.send(200, textplain, "Alarm Log cleared");
  } );

  // PSRAM
  server.on ( "/ramfltlog", RAMlogfile );
  server.on ( "/delramlog", delPSRAMlog );
  server.on ( "/dodelramlog", dodelPSRAMlog );

  // SPIFFS
  server.on ( "/listall", list_spiffs_all );
  server.on ( "/clearlogs", handleClearLogs );
  server.on ( "/doclearlogs", doClearLogs );

  // either SPIFFS or SD
  server.on ( "/listlogs", list_logs );

#if defined(USE_SD_CARD)
  // SD card
  server.on ( "/lastsdlog", lastSDlog );
  server.on ( "/listsdold", list_sd_old );
  server.on ( "/listsdall", list_sd_all );
  server.on ( "/trashlogs", handleClearSDLogs );
  server.on ( "/trashall", handleClearSDAll );
  server.on ( "/dotrashlogs", doClearSDLogs );
  server.on ( "/dotrashall", doClearSDAll );
  server.on ( "/emptytrash", handleEmptyTrash );
  server.on ( "/doemptytrash", doEmptyTrash );
#endif

  server.on ( "/input", handleInput );

  server.onNotFound([]() {                          // If the client requests any URI
    if (!handleFileRead(server.uri()))              // send it if it exists
        handleNotFound();
  });

  server.on ("/firmware", HTTP_GET, [](){
    SoC->swSer_enableRx(false);
    stop_bluetooth((size_t)999999);
    server.sendHeader(String(F("Connection")), String(F("close")));
    server.sendHeader(String(F("Access-Control-Allow-Origin")), String(F("*")));
    server.send_P(200,
      PSTR("text/html"),
      PSTR("\
<html>\
  <head>\
    <meta name='viewport' content='width=device-width, initial-scale=1'>\
    <title>Firmware update</title>\
  </head>\
<body>\
<body>\
 <h1 align=center>Firmware update</h1>\
 <hr>\
 <table width=100%%>\
  <tr>\
    <td align=center>\
<form method='POST' action='/update' enctype='multipart/form-data' id='upload_form'>\
    <input type='file' name='update'>\
    <input type='submit' value='Update'>\
</form>\
    </td>\
  </tr>\
 </table>\
 <h4 align=center>Tap [Browse], select file, and then</h4>\
 <h4 align=center>tap [Update] and wait patiently...</h4>\
</body>\
</html>")
    );
  SoC->swSer_enableRx(true);
  });

  server.on ("/update", HTTP_POST, [](){
    SoC->swSer_enableRx(false);
#if defined(USE_SD_CARD)
    closeFlightLog();     // OTA while PSRAM logging will lose the log
#endif
    server.sendHeader(String(F("Connection")), String(F("close")));
    server.sendHeader(String(F("Access-Control-Allow-Origin")), "*");
    //server.send(200, textplain, (Update.hasError())?"UPDATE FAILED":"UPDATE DONE, REBOOTING");
    if (Update.hasError())
      server.send(200, texthtml,
        "<html><p align=center><h3 align=center>UPDATE FAILED</h3></p></html>");
    else
      server.send(200, texthtml,
        "<html><p align=center><h3 align=center>UPDATE DONE, REBOOTING...</h3></p></html>");
//    SoC->swSer_enableRx(true);
    Buzzer_fini();
    Voice_fini();
    RF_Shutdown();
    delay(1000);
    reboot_pending = true;   // will reboot 2 seconds later in Web_loop()
    //SoC->reset();
  },[](){
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      Serial.setDebugOutput(true);
      SoC->WiFiUDP_stopAll();
      SoC->WDT_fini();
      Serial.printf("Update: %s\r\n", upload.filename.c_str());
      uint32_t maxSketchSpace = SoC->maxSketchSpace();
      if (Update.begin(maxSketchSpace)) {   //start with max available size
        blue_LED_1hz();
        OLED_msg("UPDATE", "...");
      } else {
        Update.printError(Serial);
        blue_LED_4hz();
        OLED_msg("UPDATE", "FAILED");
        Serial.println("update.begin failed");
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(Serial);
        blue_LED_4hz();
        OLED_msg("UPDATE", "FAILED");
        Serial.println("\r\nupdate.write failed");
      } else {
        static uint16_t i = 0;
        static uint16_t j = 0;
        ++i;
        if ((i >> 4) != j) { 
          j = (i >> 4);
          Serial.print(".");
          if ((j & 31) == 0)
              Serial.print("\r\n");
          char buf[8];
          snprintf(buf,7,"..%d..",j);
          OLED_msg("UPDATE", buf);
        }
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) { //true to set the size to the current progress
        Serial.printf("\r\nUpdate Success: %u\r\nRebooting...\r\n", upload.totalSize);
        blue_LED_on();
        OLED_msg("UPDATE", "SUCCESS");
      } else {
        Update.printError(Serial);
        blue_LED_4hz();
        OLED_msg("UPDATE", "FAILED");
        Serial.println("\r\nupdate.end failed");
      }
      Serial.setDebugOutput(false);
    }
    yield();
  });

//  server.on ( "/logo.png", []() {
//    server.send_P ( 200, "image/png", Logo, sizeof(Logo) );   - skipped to save space
//  } );

//  server.on ( "/jquery.min.js", []() { ... } );   - skipped to save space

  server.begin();
  Serial.println (F("HTTP server has started at port: 80"));

  delay(1000);
}

void Web_loop()
{
  server.handleClient();
  if (reboot_pending) {
    close_logs();
    delay(2000);
    SoC->reset();
  }
}

void Web_fini()
{
  server.stop();
}

#endif /* ESP32 */

#endif /* EXCLUDE_WIFI */
