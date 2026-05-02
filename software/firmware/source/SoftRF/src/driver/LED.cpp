/*
 * LEDHelper.cpp
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
#include "../system/Time.h"

#include <TimeLib.h>

#include "LED.h"
#include "Battery.h"
#include "Settings.h"
#include "Buzzer.h"
#include "../TrafficHelper.h"

static uint32_t prev_tx_packets_counter = 0;
static uint32_t prev_rx_packets_counter = 0;

static int status_LED = SOC_UNUSED_PIN;

// IMPORTANT: To reduce NeoPixel burnout risk, add 1000 uF capacitor across
// pixel power leads, add 300 - 500 Ohm resistor on first pixel's data input
// and minimize distance between Arduino and first pixel.  Avoid connecting
// on a live circuit...if you must, connect GND first.

void LED_setup() {

#if defined(ESP32)
  if (hw_info.model == SOFTRF_MODEL_PRIME_MK2)
      if (ESP32_pin_reserved(SOC_GPIO_PIN_STATUS, false, "Status LED")) return;
#endif

#if !defined(EXCLUDE_LED_RING)
  if (SOC_GPIO_PIN_LED != SOC_UNUSED_PIN && settings->pointer != LED_OFF) {
    uni_begin();
    uni_show(); // Initialize all pixels to 'off'
  }
#endif /* EXCLUDE_LED_RING */

  status_LED = SOC_GPIO_PIN_STATUS;

  if (status_LED != SOC_UNUSED_PIN) {
    pinMode(status_LED, OUTPUT);
    /* Indicate positive power supply */
#if defined(ESP32)
    digitalWrite(status_LED, ((hw_info.revision < 8)? HIGH : LOW));
#else
    digitalWrite(status_LED, LED_STATE_ON);
#endif
  }
}

#if !defined(EXCLUDE_LED_RING)
// Fill the dots one after the other with a color
static void colorWipe(color_t c, uint8_t wait) {
  for (uint16_t i = 0; i < uni_numPixels(); i++) {
    uni_setPixelColor(i, c);
    uni_show();
    delay(wait);
  }
}

//Theatre-style crawling lights.
static void theaterChase(color_t c, uint8_t wait) {
  for (int j = 0; j < 10; j++) { //do 10 cycles of chasing
    for (int q = 0; q < 3; q++) {
      for (int i = 0; i < uni_numPixels(); i = i + 3) {
        uni_setPixelColor(i + q, c);  //turn every third pixel on
      }
      uni_show();

      delay(wait);

      for (int i = 0; i < uni_numPixels(); i = i + 3) {
        uni_setPixelColor(i + q, LED_COLOR_BLACK);      //turn every third pixel off
      }
    }
  }
}
#endif /* EXCLUDE_LED_RING */

void LED_test() {
#if !defined(EXCLUDE_LED_RING)
  if (SOC_GPIO_PIN_LED != SOC_UNUSED_PIN && settings->pointer != LED_OFF) {
    // Some example procedures showing how to display to the pixels:
    colorWipe(uni_Color(255, 0, 0), 50); // Red
    colorWipe(uni_Color(0, 255, 0), 50); // Green
    colorWipe(uni_Color(0, 0, 255), 50); // Blue
    // Send a theater pixel chase in...
    theaterChase(uni_Color(127, 127, 127), 50); // White
    theaterChase(uni_Color(127, 0, 0), 50); // Red
    theaterChase(uni_Color(0, 0, 127), 50); // Blue

    //  rainbow(20);
    //  rainbowCycle(20);
    //  theaterChaseRainbow(50);
    colorWipe(uni_Color(0, 0, 0), 50); // clear
  }
#endif /* EXCLUDE_LED_RING */
}

#if !defined(EXCLUDE_LED_RING)
static void LED_Clear_noflush() {
    for (uint16_t i = 0; i < RING_LED_NUM; i++) {
      uni_setPixelColor(i, LED_COLOR_BACKLIT);
    }

    if (rx_packets_counter > prev_rx_packets_counter) {
      uni_setPixelColor(LED_STATUS_RX, LED_COLOR_MI_GREEN);
      prev_rx_packets_counter = rx_packets_counter;

      if (settings->mode == SOFTRF_MODE_WATCHOUT) {
        for (uint16_t i = 0; i < RING_LED_NUM; i++) {
          uni_setPixelColor(i, LED_COLOR_RED);
        }
      } else if (settings->mode == SOFTRF_MODE_BRIDGE) {
        for (uint16_t i = 0; i < RING_LED_NUM; i++) {
          uni_setPixelColor(i, LED_COLOR_MI_RED);
        }
      }

    }  else {
      uni_setPixelColor(LED_STATUS_RX, LED_COLOR_BLACK);
    }

    if (tx_packets_counter > prev_tx_packets_counter) {
      uni_setPixelColor(LED_STATUS_TX, LED_COLOR_MI_GREEN);
      prev_tx_packets_counter = tx_packets_counter;
    } else {
      uni_setPixelColor(LED_STATUS_TX, LED_COLOR_BLACK);
    }

    uni_setPixelColor(LED_STATUS_POWER,
      Battery_voltage() > Battery_threshold() ? LED_COLOR_MI_GREEN : LED_COLOR_MI_RED);
    uni_setPixelColor(LED_STATUS_SAT,
      isValidFix() ? LED_COLOR_MI_GREEN : LED_COLOR_MI_RED);
}
#endif /* EXCLUDE_LED_RING */

void LED_Clear() {
#if !defined(EXCLUDE_LED_RING)
  if (SOC_GPIO_PIN_LED != SOC_UNUSED_PIN && settings->pointer != LED_OFF) {
    LED_Clear_noflush();

    SoC->swSer_enableRx(false);
    uni_show();
    SoC->swSer_enableRx(true);
  }
#endif /* EXCLUDE_LED_RING */
}

void LED_DisplayTraffic() {
#if !defined(EXCLUDE_LED_RING)
  int bearing, distance;
  int led_num;
  color_t color;

  if (SOC_GPIO_PIN_LED != SOC_UNUSED_PIN && settings->pointer != LED_OFF) {
    LED_Clear_noflush();

    for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {

      if (Container[i].addr && (OurTime - Container[i].timestamp) <= LED_EXPIRATION_TIME) {

        bearing  = (int) Container[i].bearing;
        distance = (int) Container[i].distance;

        if (settings->pointer == DIRECTION_TRACK_UP) {
          bearing = (360 + bearing - (int)ThisAircraft.course) % 360;
        }

        led_num = ((bearing + LED_ROTATE_ANGLE + SECTOR_PER_LED/2) % 360) / SECTOR_PER_LED;
//      Serial.print(bearing);
//      Serial.print(" , ");
//      Serial.println(led_num);
//      Serial.println(distance);
        if (distance < LED_DISTANCE_FAR) {
          if (distance >= 0 && distance <= LED_DISTANCE_CLOSE) {
            color =  LED_COLOR_RED;
          } else if (distance > LED_DISTANCE_CLOSE && distance <= LED_DISTANCE_NEAR) {
            color =  LED_COLOR_YELLOW;
          } else if (distance > LED_DISTANCE_NEAR && distance <= LED_DISTANCE_FAR) {
            color =  LED_COLOR_BLUE;
          }
          uni_setPixelColor(led_num, color);
        }
      }
    }

    SoC->swSer_enableRx(false);
    uni_show();
    SoC->swSer_enableRx(true);

  }
#endif /* EXCLUDE_LED_RING */
}

void LED_loop() {

#if defined(ESP32)
  if (hw_info.model == SOFTRF_MODEL_PRIME_MK2) {
    if (hw_info.revision >= 8)
      return;
    if (settings->gnss_pins == EXT_GNSS_15_14)
      return;    // pin 14 is connected to the LED on the v0.7
    if (settings->volume != BUZZER_OFF)
      return;
    if (Battery_voltage() > Battery_threshold() ) {
        /* Indicate positive power supply */
        if (led_status != LED_STATE_ON)
            digitalWrite(status_LED, LED_STATE_ON);
    } else {
        digitalWrite(status_LED, (millis() & 0x0200)? HIGH : LOW);
    }
  }

#else  // nRF52
  if (status_LED == SOC_UNUSED_PIN)
      return;

  if (USBMSC_flag) {
      //uint8_t USBMSC_LED = SOC_GPIO_LED_USBMSC;   // red LED in all models
      //if (USBMSC_LED != SOC_UNUSED_PIN)
      //    digitalWrite(USBMSC_LED, LED_STATE_ON);
      // - was turned on in nRF52_msc_write_cb()
      // - let it stay on until USBMSC is done
      //if (hw_info.model == SOFTRF_MODEL_CARD)            // T1000E
      //    digitalWrite(SOC_GPIO_LED_T1000_GREEN, LOW);   // off
      // - let green & red LEDs both operate, even though co-located on T1000E & M3 & T-Echo
      return;
  }

  if (hw_info.model == SOFTRF_MODEL_BADGE) {   // T-Echo
#if 0
      // only use status_LED = green
      if (! isValidFix()) {
          // make the LED blink fast if no GPS fix
          digitalWrite(status_LED, (millis() & 0x080)? HIGH : LOW);
      } else if (Battery_voltage() > Battery_threshold() ) {
          digitalWrite(status_LED, LOW);   // LED_STATE_ON = LOW
      } else {
          // make the LED blink slowly if battery is low
          digitalWrite(status_LED, (millis() & 0x0200)? HIGH : LOW);
      }
#endif
#if 0
      // red LED shows power status, blue or green LED shows GPS fix status
      // - but it turns out the red is co-located with the blue & green
      // - the other red is controlled by the USB power connection?
      uint8_t power_LED = SOC_GPIO_LED_TECHO_LED_RED;
      if (Battery_voltage() > Battery_threshold() ) {
          //digitalWrite(power_LED, LOW);   // LED_STATE_ON = LOW
          // make the LED light up in short flashes to show power is OK
          digitalWrite(power_LED, ((millis() & 0x0700) == 0x0700)? LOW : HIGH);
          // (red will stay on while connected as mass storage device)
      } else {
          // make the LED blink slowly if battery is low
          digitalWrite(power_LED, (millis() & 0x0200)? HIGH : LOW);
      }
      uint8_t green_LED = SOC_GPIO_LED_TECHO_LED_GREEN;  // pins depend on hw_info.revision
      uint8_t blue_LED  = SOC_GPIO_LED_TECHO_LED_BLUE;
      if (isValidFix()) {
          digitalWrite(blue_LED, LOW);   // LED_STATE_ON = LOW
          digitalWrite(green_LED, HIGH);
      } else {
          // make the LED blink fast if no GPS fix
          digitalWrite(green_LED, (millis() & 0x080)? HIGH : LOW);
          digitalWrite(blue_LED, HIGH);
      }
#endif
#if 1
      // color scheme for RGB LED, same as M3:
      uint8_t red_LED   = SOC_GPIO_LED_TECHO_LED_RED;  // pins depend on hw_info.revision
      uint8_t green_LED = SOC_GPIO_LED_TECHO_LED_GREEN;
      uint8_t blue_LED  = SOC_GPIO_LED_TECHO_LED_BLUE;
      if (status_LED == red_LED)        // from booting
          digitalWrite(red_LED, HIGH);  // turn it off
      // blue if GNSS fix, else green - turn off the other LED
      if (isValidFix()) {
          status_LED = blue_LED;
          digitalWrite(green_LED, HIGH);
      } else {
          status_LED = green_LED;
          digitalWrite(blue_LED, HIGH);
      }
      if (Battery_voltage() > Battery_threshold() ) {
          digitalWrite(status_LED, LOW);
      } else {
          // make the LED blink slowly if battery is low
          digitalWrite(status_LED, (millis() & 0x0200)? HIGH : LOW);
      }
#endif
  }

  if (hw_info.model == SOFTRF_MODEL_HANDHELD) {   // M1
#if 0
      status_LED = SOC_GPIO_LED_M1_BLUE;
      // red during boot, blue later - turn off the red LED
      digitalWrite(SOC_GPIO_LED_M1_RED, HIGH);   // LED_STATE_ON = LOW
      if (! isValidFix()) {
          // make the LED blink fast if no GPS fix
          digitalWrite(status_LED, (millis() & 0x080)? HIGH : LOW);
      } else if (Battery_voltage() > Battery_threshold() ) {
          digitalWrite(status_LED, LOW);   // LED_STATE_ON = LOW
      } else {
          // make the LED blink slowly if battery is low
          digitalWrite(status_LED, (millis() & 0x0200)? HIGH : LOW);
      }
#endif
#if 0
      status_LED = (isValidFix()? SOC_GPIO_LED_M1_BLUE : SOC_GPIO_LED_M1_RED);
      // blue if GNSS fix, else red - turn off the other LED
      if (status_LED == SOC_GPIO_LED_M1_BLUE)
          digitalWrite(SOC_GPIO_LED_M1_RED, HIGH);   // LED_STATE_ON = LOW
      else
          digitalWrite(SOC_GPIO_LED_M1_BLUE, HIGH);
      if (Battery_voltage() > Battery_threshold() ) {
          digitalWrite(status_LED, LOW);   // LED_STATE_ON = LOW
      } else {
          // make the LED blink slowly if battery is low
          digitalWrite(status_LED, (millis() & 0x0200)? HIGH : LOW);
      }
#endif
#if 1
      // separate red LED shows power status
      // blue LED within red/blue combo LED shows GPS fix status
      // (red will light during booting and while mass storage device is accessed)
      if (Battery_voltage() > Battery_threshold() ) {
          //digitalWrite(SOC_GPIO_LED_M1_RED_PWR, HIGH);
          // make the LED light up in short flashes to show power is OK
          //digitalWrite(SOC_GPIO_LED_M1_RED_PWR, ((millis() & 0x0700) == 0x0700)? HIGH : LOW);
          // or just leave it off, as the other LED shows power is on,
          // so use the separate LED to show that external power is connected:
          digitalWrite(SOC_GPIO_LED_M1_RED_PWR, digitalRead(SOC_GPIO_PIN_M1_VUSB_SEN)? HIGH : LOW);
      } else {
          // make the LED blink slowly if battery is low
          digitalWrite(SOC_GPIO_LED_M1_RED_PWR, (millis() & 0x0200)? HIGH : LOW);
      }
      if (isValidFix()) {
          digitalWrite(SOC_GPIO_LED_M1_BLUE, LOW);   // LED_STATE_ON = LOW
      } else {
          // make the LED blink fast if no GPS fix
          digitalWrite(SOC_GPIO_LED_M1_BLUE, (millis() & 0x080)? HIGH : LOW);
      }
#endif
  }

  if (hw_info.model == SOFTRF_MODEL_POCKET) {   // M3
      if (status_LED == SOC_GPIO_LED_M3_RED)        // from booting
          digitalWrite(SOC_GPIO_LED_M3_RED, HIGH);  // turn it off
      // blue if GNSS fix, else green - turn off the other LED
      uint8_t green_LED = SOC_GPIO_LED_M3_GREEN;
      uint8_t blue_LED  = SOC_GPIO_LED_M3_BLUE;
      if (isValidFix()) {
          status_LED = blue_LED;
          digitalWrite(green_LED, HIGH);   // LED_STATE_ON = LOW
      } else {
          status_LED = green_LED;
          digitalWrite(blue_LED, HIGH);
      }
      if (Battery_voltage() > Battery_threshold() ) {
          digitalWrite(status_LED, LOW);
      } else {
          // make the LED blink slowly if battery is low
          digitalWrite(status_LED, (millis() & 0x0200)? HIGH : LOW);
      }
  }

  if (hw_info.model == SOFTRF_MODEL_CARD) {       // T1000E
      status_LED = SOC_GPIO_LED_T1000_GREEN;      // was red during boot
      digitalWrite(SOC_GPIO_LED_T1000_RED, LOW);  // switch to green, turn red off
      if (! isValidFix()) {
          // make the LED blink fast if no GPS fix
          digitalWrite(SOC_GPIO_LED_T1000_GREEN, (millis() & 0x080)? HIGH : LOW);
      } else if (Battery_voltage() > Battery_threshold() ) {
          digitalWrite(SOC_GPIO_LED_T1000_GREEN, HIGH);   // LED_STATE_ON = HIGH
      } else {
          // make the LED blink slowly if battery is low
          digitalWrite(SOC_GPIO_LED_T1000_GREEN, (millis() & 0x0200)? HIGH : LOW);
      }
  }
#endif
}
