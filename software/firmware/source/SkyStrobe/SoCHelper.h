/*
 * SoCHelper.h
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

#ifndef SOCHELPER_H
#define SOCHELPER_H

#define SOC_UNUSED_PIN 255

#include "Platform_ESP8266.h"
#include "Platform_ESP32.h"
#include "BluetoothHelper.h"

/* assume no useful buttons on the board used for SkyStrobe */
#define EXCLUDE_BUTTONS

/* These are defined in Platform_ESP32.h:
#define SOC_GPIO_PIN_BUZZER    14
#define SOC_GPIO_PIN_BUZZER2   15
#define SOC_GPIO_PIN_LED_RED   16
#define SOC_GPIO_PIN_LED_GREEN 17
#define SOC_GPIO_PIN_LED_BLUE  18
#define SOC_GPIO_PIN_I2S_DATA  25
#define SOC_GPIO_PIN_I2S_BCK   26
#define SOC_GPIO_PIN_I2S_WS    27
#define SOC_GPIO_PIN_DCBUZZ    32
#define SOC_GPIO_PIN_STROBE    33
#define SOC_GPIO_PIN_SWITCH1   34   // need external pull-up resistor
#define SOC_GPIO_PIN_SWITCH2   35   // need external pull-up resistor
*/

#if defined(SOC_GPIO_PIN_STROBE)
#define STROBEPIN   SOC_GPIO_PIN_STROBE
#else
#define STROBEPIN   SOC_UNUSED_PIN
#endif

#if defined(SOC_GPIO_PIN_LED_RED)
#define REDLEDPIN   SOC_GPIO_PIN_LED_RED
#else
#define REDLEDPIN   SOC_UNUSED_PIN
#endif

#if defined(SOC_GPIO_PIN_LED_GREEN)
#define GREENLEDPIN SOC_GPIO_PIN_LED_GREEN
#else
#define GREENLEDPIN SOC_UNUSED_PIN
#endif

#if defined(SOC_GPIO_PIN_LED_BLUE)
#define BLUELEDPIN  SOC_GPIO_PIN_LED_BLUE
#else
#define BLUELEDPIN  SOC_UNUSED_PIN
#endif

#if !defined(SOC_GPIO_PIN_I2S_BCK)
#define SOC_GPIO_PIN_I2S_BCK   SOC_UNUSED_PIN
#endif
#if !defined(SOC_GPIO_PIN_I2S_WS)
#define SOC_GPIO_PIN_I2S_WS    SOC_UNUSED_PIN
#endif
#if !defined(SOC_GPIO_PIN_I2S_DATA)
#define SOC_GPIO_PIN_I2S_DATA  SOC_UNUSED_PIN
#endif

#if !defined(SOC_GPIO_PIN_BUZZER)
#define SOC_GPIO_PIN_BUZZER   SOC_UNUSED_PIN
#endif
#if !defined(SOC_GPIO_PIN_BUZZER2)
#define SOC_GPIO_PIN_BUZZER2  SOC_UNUSED_PIN
#endif
#if !defined(SOC_GPIO_PIN_DCBUZZ)
#define SOC_GPIO_PIN_DCBUZZ   SOC_UNUSED_PIN
#endif

#if !defined(SOC_GPIO_PIN_SWITCH1)
#define SOC_GPIO_PIN_SWITCH1   SOC_UNUSED_PIN
#endif
#if !defined(SOC_GPIO_PIN_SWITCH2)
#define SOC_GPIO_PIN_SWITCH2   SOC_UNUSED_PIN
#endif

#if defined(SOC_GPIO_PIN_SWITCH1)
#define SWITCH1_PIN SOC_GPIO_PIN_SWITCH1
#else
#define SWITCH1_PIN SOC_UNUSED_PIN
#endif

#if defined(SOC_GPIO_PIN_SWITCH2)
#define SWITCH2_PIN SOC_GPIO_PIN_SWITCH2
#else
#define SWITCH2_PIN SOC_UNUSED_PIN
#endif

typedef struct SoC_ops_struct {
  uint8_t id;
  const char name[16];
  void (*setup)();
  void (*fini)();
  uint32_t (*getChipId)();
  bool (*EEPROM_begin)(size_t);
  void (*WiFi_setOutputPower)(int);
  bool (*WiFi_hostname)(String);
  void (*swSer_begin)(unsigned long);
  void (*swSer_enableRx)(boolean);
  uint32_t (*maxSketchSpace)();
  void (*WiFiUDP_stopAll)();
  void (*Battery_setup)();
  float (*Battery_voltage)();
  void (*WiFi_set_param)(int, int);
  size_t (*WiFi_Receive_UDP)(uint8_t *, size_t);
  void (*WiFi_Transmit_UDP)(int, byte *, size_t);
  int  (*WiFi_clients_count)();
  void (*Button_setup)();
  void (*Button_loop)();
  void (*Button_fini)();
  void (*WDT_setup)();
  void (*WDT_fini)();
  Bluetooth_ops_t *Bluetooth;
} SoC_ops_t;

enum
{
	SOC_NONE,
	SOC_ESP8266,
	SOC_ESP32,
};

extern const SoC_ops_t *SoC;
#if defined(ESP8266)
extern const SoC_ops_t ESP8266_ops;
#endif
#if defined(ESP32)
extern const SoC_ops_t ESP32_ops;
#endif

byte SoC_setup(void);
void SoC_fini(void);

#endif /* SOCHELPER_H */
