/*
 * Buzzer.cpp
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

#include "../system/SoC.h"

#if defined(EXCLUDE_BUZZER)
void  Buzzer_setup()       {}
bool  Buzzer_Notify(int8_t level, bool multi_alarm) {return false;}
void  Buzzer_loop()        {}
void  Buzzer_fini()        {}
#else

#include "Buzzer.h"
#include "Settings.h"
#include "Strobe.h"

#if defined(ESP32)
#include <toneAC.h>
#include "OLED.h"
static uint8_t buzzer1pin = SOC_UNUSED_PIN;
static uint8_t buzzer2pin = SOC_UNUSED_PIN;
#endif

/* need this for the alarm levels enum: */
#include "../protocol/radio/Legacy.h"
/* - perhaps should move those to another location? */

#include "../protocol/data/NMEA.h"

static uint32_t BuzzerTimeMarker = 0;
static uint8_t BuzzerBeeps = 0;     /* how many beeps to go */
static uint8_t BuzzerBeep  = 0;     /* how many beeps done */
static uint8_t BuzzerState = 0;     /* 1 = buzzing */
static bool double_beep = false;
static uint16_t BuzzerToneHz = 0;    /* variable tone */
static uint16_t BuzzerBeepMS = 0;    /* how long each beep */

void ext_buzzer(bool state)
{
#if defined(ESP32)
  if (buzzer1pin != SOC_UNUSED_PIN)
      digitalWrite(buzzer1pin, state? HIGH : LOW);
#endif
}

static int volume = 10;

void Buzzer_setup(void)
{
  if (settings->volume == BUZZER_OFF)
      return;

#if defined(ESP32)
  buzzer1pin = SOC_GPIO_PIN_BUZZER;
  if (buzzer1pin == SOC_UNUSED_PIN) {
      settings->volume = BUZZER_OFF;
      return;
  }
  if (ESP32_pin_reserved(buzzer1pin, false, "Buzzer")) {
      settings->volume = BUZZER_OFF;
      return;
  }

  buzzer2pin = SOC_GPIO_PIN_BUZZER2;
  if (hw_info.model == SOFTRF_MODEL_PRIME_MK2 && hw_info.revision < 8) {
      buzzer2pin = SOC_GPIO_PIN_VOICE;
      // (gpio15 not available on T-Beam v0.7, use 25 instead)
      if (settings->voice != VOICE_OFF || settings->strobe != STROBE_OFF) {
          // voice or strobe overrides passive buzzer on v0.7
          if (settings->volume != BUZZER_EXT) {
              settings->volume = BUZZER_OFF;
              return;
          }
      }
  }

  if (settings->volume == BUZZER_EXT) {
      pinMode(buzzer1pin, OUTPUT);
      ext_buzzer(true);   // sound buzzer briefly for self-test
      delay(80);
      ext_buzzer(false);
      delay(40);
      ext_buzzer(true);
      delay(80);
      ext_buzzer(false);
//      delay(100);
//      ext_buzzer(true);
//      delay(200);
//      ext_buzzer(false);
  } else {
      if (ESP32_pin_reserved(buzzer2pin, false, "Buzzer")) {
          settings->volume = BUZZER_OFF;
          return;
      }
      toneAC_setup(buzzer2pin, buzzer1pin);
  }
#endif
  // for nRF52 the buzzer pin is chosen in nRF52.cpp nrf52_setup()

  volume = (settings->volume == BUZZER_VOLUME_LOW ? 8 : 10);
  BuzzerToneHz = 0;
  BuzzerBeepMS = 0;
  BuzzerBeeps = 0;
  BuzzerState = 0;
  BuzzerTimeMarker = 0;
  double_beep = false;
}

bool Buzzer_Notify(int8_t alarm_level, bool multi_alarm)
{
  if (settings->volume == BUZZER_OFF)
      return false;

  if (BuzzerTimeMarker != 0)   // beeping in progress
      return false;

  /* if more than one alarm aircraft, emit double-beeps */

  double_beep = false;
  if (alarm_level == ALARM_LEVEL_LOW) {
    BuzzerToneHz = ALARM_TONE_HZ_LOW;
    if (multi_alarm) {
        BuzzerBeepMS = ALARM_TONE_MS_LOW / 2;
        BuzzerBeeps  = ALARM_BEEPS_LOW * 2;
        double_beep = true;
    } else {
        BuzzerBeepMS = ALARM_TONE_MS_LOW;
        BuzzerBeeps  = ALARM_BEEPS_LOW;
    }
  } else if (alarm_level == ALARM_LEVEL_IMPORTANT) {
    BuzzerToneHz = ALARM_TONE_HZ_IMPORTANT;
    if (multi_alarm) {
        BuzzerBeepMS = ALARM_TONE_MS_IMPORTANT / 2;
        BuzzerBeeps  = ALARM_BEEPS_IMPORTANT * 2;
        double_beep = true;
    } else {
        BuzzerBeepMS = ALARM_TONE_MS_IMPORTANT;
        BuzzerBeeps  = ALARM_BEEPS_IMPORTANT;
    }
  } else if (alarm_level == ALARM_LEVEL_URGENT) {
    BuzzerToneHz = ALARM_TONE_HZ_URGENT;
    BuzzerBeepMS = ALARM_TONE_MS_URGENT;
    if (multi_alarm)
        BuzzerBeeps  = ALARM_BEEPS_URGENT + 2;
    else
        BuzzerBeeps  = ALARM_BEEPS_URGENT;
  } else {    /* whether NONE or CLOSE */
    return false;
  }

  BuzzerBeep = 1;  // starting the first beep

  if (settings->volume == BUZZER_EXT) {
    ext_buzzer(true);
  } else {
    unsigned long duration = 0;  // forever, until turned off
    uint8_t background = 1;       // return to main thread while sounding tone
    toneAC(BuzzerToneHz, volume, duration, background);
  }
  BuzzerState = 1;
  BuzzerTimeMarker = millis() + BuzzerBeepMS;
//if (settings->debug_flags & 0x80) {
//snprintf_P(NMEABuffer, sizeof(NMEABuffer),"alarm level %d buzzer turned on at %d ms\r\n", alarm_level, millis());
//NMEAOutD();
//}
  return true;
}

void Buzzer_loop(void)
{
  if (settings->volume == BUZZER_OFF)
      return;

  if (BuzzerTimeMarker != 0 && millis() > BuzzerTimeMarker) {

    if (BuzzerBeeps > 1) {

      if (BuzzerState == 1) {   /* a beep is ending */
        if (settings->volume == BUZZER_EXT)
          ext_buzzer(false);
        else
          noToneAC();
        BuzzerState = 0;
        uint32_t gap;
        if (double_beep && (BuzzerBeep & 1))
            gap = ALARM_MULTI_GAP_MS;   /* short break making it a double-beep */
        else
            gap = BuzzerBeepMS;         /* gap is same length as the beep */
        BuzzerTimeMarker = millis() + gap;           /* time of next beep */
//if (settings->debug_flags & 0x80) {
//snprintf_P(NMEABuffer, sizeof(NMEABuffer),"... buzzer turned off at %d ms\r\n", millis());
//NMEAOutD();
//}
      } else {  /* sound is off, start another beep */
        if (settings->volume == BUZZER_EXT)
          ext_buzzer(true);
        else
          toneAC(BuzzerToneHz, volume, 0, 1);
        BuzzerState = 1;
        --BuzzerBeeps;
        ++BuzzerBeep;
        BuzzerTimeMarker = millis() + BuzzerBeepMS;  /* timer of next gap */
//if (settings->debug_flags & 0x80) {
//snprintf_P(NMEABuffer, sizeof(NMEABuffer),"... buzzer turned on  at %d ms\r\n", millis());
//NMEAOutD();
//}
      }

    } else {   /* done beeping, turn it all off */

      if (settings->volume == BUZZER_EXT)
        ext_buzzer(false);
      else
        noToneAC();
      BuzzerTimeMarker = 0;
//if (settings->debug_flags & 0x80) {
//snprintf_P(NMEABuffer, sizeof(NMEABuffer),"... buzzer final off at  %d ms\r\n", millis());
//NMEAOutD();
//}
    }

    return;
  }

  /* strobe does a self test, do something similar with buzzer */
  if (do_alarm_demo && BuzzerTimeMarker == 0) {
      delay(1000);
      uint32_t t = millis() - SetupTimeMarker;
      if (t < (1000*(STROBE_INITIAL_RUN+3))) {
          // STROBE_INITIAL_RUN = 9
          // 3s LOW, 3s IMPORTANT, 3s IMPORTANT-multi, 3s URGENT
          int8_t level = (t > (1000*(STROBE_INITIAL_RUN-0)) ? ALARM_LEVEL_URGENT :
                          t > (1000*(STROBE_INITIAL_RUN-6)) ? ALARM_LEVEL_IMPORTANT :
                          ALARM_LEVEL_LOW);
          bool multalarm = false;
          if (level==ALARM_LEVEL_IMPORTANT && t > (1000*(STROBE_INITIAL_RUN-3)))
               multalarm = true;
          Buzzer_Notify(level, multalarm);
      } else {
#if defined(USE_OLED)
          OLED_no_msg();
#endif
          do_alarm_demo = false;
      }
  }
}

void Buzzer_fini(void)
{
  BuzzerTimeMarker = 0;

  /* if (settings->volume == BUZZER_OFF)
      return;  */

  if (settings->volume == BUZZER_EXT) {
      ext_buzzer(false);
      return;
  }

  // play descending tones to signal shutdown
  toneAC(640, volume, 250, 0);
  toneAC(440, volume, 250, 0);
}

#endif  /* EXCLUDE_BUZZER */
