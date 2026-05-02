/*
 * Platform_nRF52.h
 * Copyright (C) 2020-2026 Linar Yusupov
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

// This version edited in 2026 to support T1000E and M3

#if defined(ARDUINO_ARCH_NRF52)  || defined(ARDUINO_ARCH_NRF52840)

#ifndef PLATFORM_NRF52_H
#define PLATFORM_NRF52_H

#include <avr/dtostrf.h>
#include <pcf8563.h>
#include <Adafruit_SPIFlash.h>

/* Maximum of tracked flying objects is now SoC-specific constant */
#define MAX_TRACKING_OBJECTS    8
#define MAX_NMEA_OBJECTS        6

#define DEFAULT_SOFTRF_MODEL    SOFTRF_MODEL_BADGE

#define isValidFix()            isValidGNSSFix()

#define uni_begin()             strip.begin()
#define uni_show()              strip.show()
#define uni_setPixelColor(i, c) strip.setPixelColor(i, c)
#define uni_numPixels()         strip.numPixels()
#define uni_Color(r,g,b)        strip.Color(r,g,b)
#define color_t                 uint32_t

#define snprintf_P              snprintf
#define EEPROM_commit()         {}

// State when LED is litted
#if defined(LED_STATE_ON)
#undef  LED_STATE_ON
#endif /* LED_STATE_ON */
#define LED_STATE_ON            (hw_info.model == SOFTRF_MODEL_CARD ? HIGH : LOW)

#define USE_TINYUSB
#define USBCON

#define SerialOutput            Serial1
#define USBSerial               Serial
#define swSer                   Serial2   /* for compatibility with v1.0 GNSS.cpp */
#define Serial_GNSS_In          Serial2
#define Serial_GNSS_Out         Serial_GNSS_In
#define UATSerial               Serial1

enum rst_reason {
  REASON_DEFAULT_RST      = 0,  /* normal startup by power on */
  REASON_WDT_RST          = 1,  /* hardware watch dog reset */
  REASON_EXCEPTION_RST    = 2,  /* exception reset, GPIO status won't change */
  REASON_SOFT_WDT_RST     = 3,  /* software watch dog reset, GPIO status won't change */
  REASON_SOFT_RESTART     = 4,  /* software restart ,system_restart , GPIO status won't change */
  REASON_DEEP_SLEEP_AWAKE = 5,  /* wake up from deep-sleep */
  REASON_EXT_SYS_RST      = 6   /* external system reset */
};

enum nRF52_board_id {
  NRF52_NORDIC_PCA10059,        /* reference low power board */
  NRF52_LILYGO_TECHO_REV_0,     /* 20-8-6 */
  NRF52_LILYGO_TECHO_REV_1,     /* 2020-12-12 */
  NRF52_LILYGO_TECHO_REV_2,     /* 2021-3-26 */
  NRF52_LILYGO_TECHO_PLUS,      /* 2025 */
  NRF52_SEEED_T1000E,
  NRF52_ELECROW_TN_M1,
  NRF52_ELECROW_TN_M3,
  NRF52_ELECROW_OTHER,          // older M1 or M3 with "ELECROW"
  NRF52_UNSUPPORTED
};

// #define TECHO_DISPLAY_MODEL   GxEPD2_154_D67

enum nRF52_display_id {
  EP_UNKNOWN,
  EP_GDEH0154D67,
  EP_GDEP015OC1,
  EP_DEPG0150BN,
  EP_GDEY037T03,
  TFT_LH114TIF03,
};

typedef struct {
  uint64_t         id;
  nRF52_board_id   rev;
  nRF52_display_id panel;
  uint8_t          tag;
} __attribute__((packed)) prototype_entry_t;

struct rst_info {
  uint32_t reason;
  uint32_t exccause;
  uint32_t epc1;
  uint32_t epc2;
  uint32_t epc3;
  uint32_t excvaddr;
  uint32_t depc;
};

//extern struct rst_info reset_info;   // use SoC->getResetInfoPtr() instead

#define VBAT_MV_PER_LSB       (0.73242188F)   // 3.0V ADC range and 12-bit ADC resolution = 3000mV/4096
#define SOC_ADC_VOLTAGE_DIV   (2.0F)          // 100K + 100K voltage divider on VBAT
#define REAL_VBAT_MV_PER_LSB  (SOC_ADC_VOLTAGE_DIV * VBAT_MV_PER_LSB)

#if !defined(_PINNUM)
#define _PINNUM(port, pin)    ((port)*32 + (pin))
#endif

#define DFU_MAGIC_SKIP        (0x6d)
#define SHUTDOWN_MAGIC        (0x5A)
#define SLEEP_MAGIC           (0x5B)
#define CHARGE_MAGIC          (0x5C)
#define AWAKE_MAGIC           (0x5D)
#define RESET_MAGIC           (0x5E)
#define USB_MAGIC             (0x5F)

#define BMM150_ADDRESS        (0x10)
#define QMA6100P_ADDRESS      (0x12)
#define XL9555_ADDRESS        (0x20) /* A0 = A1 = A2 = LOW */
#define FT6X36_ADDRESS        (0x38)
#define M10Q_ADDRESS          (0x42)
#define BQ27220_ADDRESS       (0x55)
#define DRV2605_ADDRESS       (0x5A)
#define MPU9250_ADDRESS       (0x68)
#define ICM20948_ADDRESS      (0x68)
#define BME280_ADDRESS        (0x77)
#define BHI260AP_ADDRESS_L    (0x28)
#define BHI260AP_ADDRESS_H    (0x29)
#define AHT20_ADDRESS         (0x38)
#define SC7A20H_ADDRESS_L     (0x18)
#define SC7A20H_ADDRESS_H     (0x19)

#define MIDI_CHANNEL_TRAFFIC  1
#define MIDI_CHANNEL_VARIO    2

/************** T-Echo section *************/

/* Peripherals */
#define SOC_GPIO_PIN_CONS_RX            _PINNUM(0, 8) // P0.08
#define SOC_GPIO_PIN_CONS_TX            _PINNUM(0, 6) // P0.06

#define SOC_GPIO_PIN_GNSS_RX            _PINNUM(1, 9) // P1.09
#define SOC_GPIO_PIN_GNSS_TX            _PINNUM(1, 8) // P1.08

#define SOC_GPIO_PIN_GNSS_WKE           _PINNUM(1, 2) // P1.02
#define SOC_GPIO_PIN_GNSS_RST           _PINNUM(1, 5) // P1.05 (REV_2 only)
#define SOC_GPIO_PIN_GNSS_TECHO_PPS     _PINNUM(1, 4) // P1.04

#define SOC_GPIO_PIN_LED                SOC_UNUSED_PIN

#define SOC_GPIO_LED_TECHO_REV_0_GREEN  _PINNUM(0, 13) // P0.13 (Green)
#define SOC_GPIO_LED_TECHO_REV_0_RED    _PINNUM(0, 14) // P0.14 (Red)
#define SOC_GPIO_LED_TECHO_REV_0_BLUE   _PINNUM(0, 15) // P0.15 (Blue)
#define SOC_GPIO_LED_TECHO_REV_1_GREEN  _PINNUM(0, 15)
#define SOC_GPIO_LED_TECHO_REV_1_RED    _PINNUM(0, 13)
#define SOC_GPIO_LED_TECHO_REV_1_BLUE   _PINNUM(0, 14)
#define SOC_GPIO_LED_TECHO_REV_2_GREEN  _PINNUM(1,  1) // P1.01 (Green)
#define SOC_GPIO_LED_TECHO_REV_2_RED    _PINNUM(1,  3) // P1.03 (Red)
#define SOC_GPIO_LED_TECHO_REV_2_BLUE   _PINNUM(0, 14) // P0.14 (Blue)

#define SOC_GPIO_LED_TECHO_LED_GREEN  (hw_info.revision == 0 ? SOC_GPIO_LED_TECHO_REV_0_GREEN : \
                                       hw_info.revision == 1 ? SOC_GPIO_LED_TECHO_REV_1_GREEN : \
                                       hw_info.revision == 2 ? SOC_GPIO_LED_TECHO_REV_2_GREEN : \
                                       SOC_UNUSED_PIN)
#define SOC_GPIO_LED_TECHO_LED_RED    (hw_info.revision == 0 ? SOC_GPIO_LED_TECHO_REV_0_RED : \
                                       hw_info.revision == 1 ? SOC_GPIO_LED_TECHO_REV_1_RED : \
                                       hw_info.revision == 2 ? SOC_GPIO_LED_TECHO_REV_2_RED : \
                                       SOC_UNUSED_PIN)
#define SOC_GPIO_LED_TECHO_LED_BLUE   (hw_info.revision == 0 ? SOC_GPIO_LED_TECHO_REV_0_BLUE : \
                                       hw_info.revision == 1 ? SOC_GPIO_LED_TECHO_REV_1_BLUE : \
                                       hw_info.revision == 2 ? SOC_GPIO_LED_TECHO_REV_2_BLUE : \
                                       SOC_UNUSED_PIN)

#define SOC_GPIO_PIN_BATTERY            _PINNUM(0, 4) // P0.04 (AIN2)

#define SOC_GPIO_PIN_RX3                SOC_UNUSED_PIN
#define SOC_GPIO_PIN_TX3                SOC_UNUSED_PIN

/* SPI */
#define SOC_GPIO_PIN_TECHO_REV_0_MOSI   _PINNUM(0, 22) // P0.22
#define SOC_GPIO_PIN_TECHO_REV_1_MOSI   SOC_GPIO_PIN_TECHO_REV_0_MOSI
#define SOC_GPIO_PIN_TECHO_REV_2_MOSI   SOC_GPIO_PIN_TECHO_REV_0_MOSI
#define SOC_GPIO_PIN_TECHO_REV_0_MISO   _PINNUM(0, 23) // P0.23
#define SOC_GPIO_PIN_TECHO_REV_1_MISO   SOC_GPIO_PIN_TECHO_REV_0_MISO
#define SOC_GPIO_PIN_TECHO_REV_2_MISO   SOC_GPIO_PIN_TECHO_REV_0_MISO
#define SOC_GPIO_PIN_TECHO_REV_0_SCK    _PINNUM(0, 19) // P0.19
#define SOC_GPIO_PIN_TECHO_REV_1_SCK    SOC_GPIO_PIN_TECHO_REV_0_SCK
#define SOC_GPIO_PIN_TECHO_REV_2_SCK    SOC_GPIO_PIN_TECHO_REV_0_SCK
#define SOC_GPIO_PIN_SS                 _PINNUM(0, 24) // P0.24

/* NRF905 */
#define SOC_GPIO_PIN_TXE                SOC_UNUSED_PIN
#define SOC_GPIO_PIN_CE                 SOC_UNUSED_PIN
#define SOC_GPIO_PIN_PWR                SOC_UNUSED_PIN

/* SX1262 or SX1276 */
#define SOC_GPIO_PIN_TECHO_REV_0_RST    _PINNUM(0, 25) // P0.25
#define SOC_GPIO_PIN_TECHO_REV_1_RST    SOC_GPIO_PIN_TECHO_REV_0_RST
#define SOC_GPIO_PIN_TECHO_REV_2_RST    SOC_GPIO_PIN_TECHO_REV_0_RST
#define SOC_GPIO_PIN_TECHO_REV_0_DIO0   SOC_UNUSED_PIN
#define SOC_GPIO_PIN_TECHO_REV_1_DIO0   _PINNUM(1,  1) // P1.01
#define SOC_GPIO_PIN_TECHO_REV_2_DIO0   _PINNUM(0, 15) // P0.15
#define SOC_GPIO_PIN_DIO1               _PINNUM(0, 20) // P0.20
#define SOC_GPIO_PIN_BUSY               _PINNUM(0, 17) // P0.17

/* RF antenna switch */
#define SOC_GPIO_PIN_ANT_RXTX           SOC_UNUSED_PIN

/* I2C */
#define SOC_GPIO_PIN_SDA                _PINNUM(0, 26) // P0.26
#define SOC_GPIO_PIN_SCL                _PINNUM(0, 27) // P0.27

/* buttons */
#define SOC_GPIO_PIN_TECHO_REV_0_BUTTON _PINNUM(1, 10) // P1.10
#define SOC_GPIO_PIN_TECHO_REV_1_BUTTON SOC_GPIO_PIN_TECHO_REV_0_BUTTON
#define SOC_GPIO_PIN_TECHO_REV_2_BUTTON SOC_GPIO_PIN_TECHO_REV_0_BUTTON
#define SOC_GPIO_PIN_PAD                _PINNUM(0, 11) // P0.11

/* E-paper */
#define SOC_GPIO_PIN_EPD_MISO           _PINNUM(1,  7) // P1.07
#define SOC_GPIO_PIN_EPD_MOSI           _PINNUM(0, 29) // P0.29
#define SOC_GPIO_PIN_EPD_SCK            _PINNUM(0, 31) // P0.31
#define SOC_GPIO_PIN_EPD_SS             _PINNUM(0, 30) // P0.30
#define SOC_GPIO_PIN_EPD_DC             _PINNUM(0, 28) // P0.28
#define SOC_GPIO_PIN_EPD_RST            _PINNUM(0,  2) // P0.02
#define SOC_GPIO_PIN_EPD_BUSY           _PINNUM(0,  3) // P0.03
#define SOC_GPIO_PIN_EPD_BLGT           _PINNUM(1, 11) // P1.11

/* Power: EINK, RGB, CN1 (, RF) REV_2: FLASH, GNSS, SENSOR */
#define SOC_GPIO_PIN_IO_PWR             _PINNUM(0, 12) // P0.12
/* REV_2 power: RF */
#define SOC_GPIO_PIN_3V3_PWR            _PINNUM(0, 13) // P0.13
/* Modded REV_1 3V3 power */
#define SOC_GPIO_PIN_TECHO_REV_1_3V3_PWR  SOC_GPIO_PIN_TECHO_REV_1_DIO0

/* MX25R1635F SPI flash */
#define SOC_GPIO_PIN_SFL_MOSI           _PINNUM(1, 12) // P1.12
#define SOC_GPIO_PIN_SFL_MISO           _PINNUM(1, 13) // P1.13
#define SOC_GPIO_PIN_SFL_SCK            _PINNUM(1, 14) // P1.14
#define SOC_GPIO_PIN_SFL_SS             _PINNUM(1, 15) // P1.15
#define SOC_GPIO_PIN_SFL_HOLD           _PINNUM(0,  5) // P0.05 (REV_1 and REV_2)
#define SOC_GPIO_PIN_SFL_WP             _PINNUM(0,  7) // P0.07 (REV_1 and REV_2)

/* RTC */
#define SOC_GPIO_PIN_R_INT              _PINNUM(0, 16) // P0.16

/* NFC */
#define SOC_GPIO_PIN_NFC_ANT1           _PINNUM(0,  9) // P0.09
#define SOC_GPIO_PIN_NFC_ANT2           _PINNUM(0, 10) // P0.10

/* T-Echo Plus */
#define SOC_GPIO_PIN_MOTOR_EN           _PINNUM(0,  8) // P0.08
#define SOC_GPIO_PIN_TECHO_BUZZER       _PINNUM(0,  6) // P0.06


/************** Sensecap T1000E section *************/

/* Peripherals */
#define SOC_GPIO_PIN_CONS_T1000_RX    _PINNUM(0, 17) // P0.17
#define SOC_GPIO_PIN_CONS_T1000_TX    _PINNUM(0, 16) // P0.16

/* AG3335MN, L1 band only */
#define SOC_GPIO_PIN_GNSS_T1000_RX    _PINNUM(0, 14) // P0.14
#define SOC_GPIO_PIN_GNSS_T1000_TX    _PINNUM(0, 13) // P0.13

#define SOC_GPIO_PIN_GNSS_T1000_PPS   SOC_UNUSED_PIN
#define SOC_GPIO_PIN_GNSS_T1000_EN    _PINNUM(1, 11) // P1.11 active HIGH
#define SOC_GPIO_PIN_GNSS_T1000_OUT   _PINNUM(1, 14) // P1.14 RESETB OUT
#define SOC_GPIO_PIN_GNSS_T1000_RST   _PINNUM(1, 15) // P1.15 active HIGH
#define SOC_GPIO_PIN_GNSS_T1000_VRTC  _PINNUM(0,  8) // P0.08 VRTC EN
#define SOC_GPIO_PIN_GNSS_T1000_SINT  _PINNUM(1, 12) // P1.12 SLEEP Interrupt
#define SOC_GPIO_PIN_GNSS_T1000_RINT  _PINNUM(0, 15) // P0.15 RTC Interrupt

/* SPI */
#define SOC_GPIO_PIN_T1000_MOSI       _PINNUM(1,  9) // P1.09
#define SOC_GPIO_PIN_T1000_MISO       _PINNUM(1,  8) // P1.08
#define SOC_GPIO_PIN_T1000_SCK        _PINNUM(0, 11) // P0.11
#define SOC_GPIO_PIN_T1000_SS         _PINNUM(0, 12) // P0.12

/* LR1110 */
#define SOC_GPIO_PIN_T1000_RST        _PINNUM(1, 10) // P1.10
#define SOC_GPIO_PIN_T1000_DIO9       _PINNUM(1,  1) // P1.01
#define SOC_GPIO_PIN_T1000_BUSY       _PINNUM(0,  7) // P0.07

/* I2C */
#define SOC_GPIO_PIN_T1000_SDA        _PINNUM(0, 26) // P0.26
#define SOC_GPIO_PIN_T1000_SCL        _PINNUM(0, 27) // P0.27

/* button */
#define SOC_GPIO_PIN_T1000_BUTTON     _PINNUM(0,  6) // P0.06 Key IO, must be configured as input_pulldown

/* LED */
#define SOC_GPIO_LED_T1000_GREEN      _PINNUM(0, 24) // P0.24 active HIGH
#define SOC_GPIO_LED_T1000_RED        _PINNUM(0,  3) // P0.03 NC?

/* ADC */
#define SOC_GPIO_PIN_T1000_BATTERY    _PINNUM(0,  2) // P0.02 Baterry level dectect
#define SOC_GPIO_PIN_T1000_VCC        _PINNUM(0,  4) // P0.04 VCC voltage dectect
#define SOC_GPIO_PIN_T1000_TEMP       _PINNUM(0, 31) // P0.31 Temperature Sensor ADC input
#define SOC_GPIO_PIN_T1000_LUX        _PINNUM(0, 29) // P0.29 Light Sensor ADC input

#define SOC_ADC_T1000_VOLTAGE_DIV     (2.0F) // 100K + 100K voltage divider on VBAT

/* battery charger */
#define SOC_GPIO_PIN_T1000_CHG_PWR    _PINNUM(0,  5) // P0.05 Charger insert detect, must be configured as no pullup or pulldown
#define SOC_GPIO_PIN_T1000_CHG_STATUS _PINNUM(1,  3) // P1.03 active LOW
#define SOC_GPIO_PIN_T1000_CHG_DONE   _PINNUM(1,  4) // P1.04 active LOW

/* buzzer */
#define SOC_GPIO_PIN_T1000_BUZZER     _PINNUM(0, 25) // P0.25
#define SOC_GPIO_PIN_T1000_BUZZER_EN  _PINNUM(1,  5) // P1.05

/* QMA6100P */
#define SOC_GPIO_PIN_T1000_ACC_EN     _PINNUM(1,  7) // P1.07 active HIGH
#define SOC_GPIO_PIN_T1000_ACC_INT    _PINNUM(1,  2) // P1.02

/* Sensors */
#define SOC_GPIO_PIN_T1000_3V3_EN     _PINNUM(1,  6) // P1.06

/* GD25Q64EN (or P25Q16H ?) QSPI flash */
#define SOC_GPIO_PIN_SFL_T1000_MOSI   _PINNUM(0, 21) // P0.21
#define SOC_GPIO_PIN_SFL_T1000_MISO   _PINNUM(0, 22) // P0.22
#define SOC_GPIO_PIN_SFL_T1000_SCK    _PINNUM(0, 19) // P0.19
#define SOC_GPIO_PIN_SFL_T1000_SS     _PINNUM(0, 20) // P0.20
#define SOC_GPIO_PIN_SFL_T1000_HOLD   _PINNUM(1,  0) // P1.00
#define SOC_GPIO_PIN_SFL_T1000_WP     _PINNUM(0, 23) // P0.23

#define SOC_GPIO_PIN_SFL_T1000_EN     _PINNUM(1, 13) // P1.13 active HIGH

/* misc. */
#define SOC_GPIO_PIN_T1000_MCU_RESET  _PINNUM(0, 18) // P0.18

/************** Thinknode M1 section *************/

/* https://www.elecrow.com/download/product/CIL12901M/ThinkNode%20M1_LoRa_Meshtastic_Transceiver_DataSheet.pdf */

/* Peripherals */
#define SOC_GPIO_PIN_CONS_M1_RX   _PINNUM(0,  9) // P0.09 , No NFC
#define SOC_GPIO_PIN_CONS_M1_TX   _PINNUM(0, 10) // P0.10 , No NFC

/* L76K */
#define SOC_GPIO_PIN_GNSS_M1_RX   _PINNUM(1,  9) // P1.09
#define SOC_GPIO_PIN_GNSS_M1_TX   _PINNUM(1,  8) // P1.08

#define SOC_GPIO_PIN_GNSS_M1_PPS  SOC_UNUSED_PIN // TBD
#define SOC_GPIO_PIN_GNSS_M1_WKE  _PINNUM(1,  2) // P1.02
#define SOC_GPIO_PIN_GNSS_M1_RST  _PINNUM(1,  5) // P1.05
#define SOC_GPIO_PIN_GNSS_M1_SW   _PINNUM(1,  1) // P1.01

/* SPI */
#define SOC_GPIO_PIN_M1_MOSI      _PINNUM(0, 22) // P0.22
#define SOC_GPIO_PIN_M1_MISO      _PINNUM(0, 23) // P0.23
#define SOC_GPIO_PIN_M1_SCK       _PINNUM(0, 19) // P0.19
#define SOC_GPIO_PIN_M1_SS        _PINNUM(0, 24) // P0.24

/* SX1262 */
#define SOC_GPIO_PIN_M1_RST       _PINNUM(0, 25) // P0.25
#define SOC_GPIO_PIN_M1_DIO1      _PINNUM(0, 20) // P0.20
#define SOC_GPIO_PIN_M1_DIO3      _PINNUM(0, 21) // P0.21
#define SOC_GPIO_PIN_M1_BUSY      _PINNUM(0, 17) // P0.17

/* E-paper */
#define SOC_GPIO_PIN_EPD_M1_MISO  _PINNUM(0, 11) // P0.11 NC ?
#define SOC_GPIO_PIN_EPD_M1_MOSI  _PINNUM(0, 29) // P0.29
#define SOC_GPIO_PIN_EPD_M1_SCK   _PINNUM(0, 31) // P0.31
#define SOC_GPIO_PIN_EPD_M1_SS    _PINNUM(0, 30) // P0.30
#define SOC_GPIO_PIN_EPD_M1_DC    _PINNUM(0, 28) // P0.28
#define SOC_GPIO_PIN_EPD_M1_RST   _PINNUM(0,  2) // P0.02
#define SOC_GPIO_PIN_EPD_M1_BUSY  _PINNUM(0,  3) // P0.03
#define SOC_GPIO_PIN_EPD_M1_BLGT  _PINNUM(1, 11) // P1.11

/* Power: EINK, RGB, RF, FLASH, GNSS, SENSOR */
#define SOC_GPIO_PIN_IO_M1_PWR    _PINNUM(0, 12) // P0.12

/* I2C int. (same that T-Echo has) */
#define SOC_GPIO_PIN_M1_SDA_INT   _PINNUM(0, 26) // P0.26
#define SOC_GPIO_PIN_M1_SCL_INT   _PINNUM(0, 27) // P0.27

/* buttons */
#define SOC_GPIO_PIN_M1_BUTTON1   _PINNUM(1,  7) // P1.07
#define SOC_GPIO_PIN_M1_BUTTON2   _PINNUM(1, 10) // P1.10

/* LED */
#define SOC_GPIO_LED_M1_RED       _PINNUM(1,  6) // P1.06, active HIGH
#define SOC_GPIO_LED_M1_RED_PWR   _PINNUM(1,  4) // P1.04
#define SOC_GPIO_LED_M1_BLUE      _PINNUM(0, 13) // P0.13, active HIGH
/* NC ? */
#define SOC_GPIO_LED_M1_1         _PINNUM(0, 14) // P0.14
#define SOC_GPIO_LED_M1_2         _PINNUM(0, 15) // P0.15

/* buzzer */
#define SOC_GPIO_PIN_M1_BUZZER    _PINNUM(0,  6) // P0.06

/* MX25R1635F (?) or WP25R1635F (?) SPI flash */
#define SOC_GPIO_PIN_SFL_M1_MOSI  _PINNUM(1, 12) // P1.12
#define SOC_GPIO_PIN_SFL_M1_MISO  _PINNUM(1, 13) // P1.13
#define SOC_GPIO_PIN_SFL_M1_SCK   _PINNUM(1, 14) // P1.14
#define SOC_GPIO_PIN_SFL_M1_SS    _PINNUM(1, 15) // P1.15
#define SOC_GPIO_PIN_SFL_M1_HOLD  _PINNUM(0,  5) // P0.05
#define SOC_GPIO_PIN_SFL_M1_WP    _PINNUM(0,  7) // P0.07

/* RTC */
#define SOC_GPIO_PIN_RTC_M1_INT   _PINNUM(0, 16) // P0.16

/* ADC */
#define SOC_GPIO_PIN_M1_BATTERY   _PINNUM(0,  4) // P0.04 (AIN2)

/* digital input */
#define SOC_GPIO_PIN_M1_VBAT_SEN  _PINNUM(0,  8) // P0.08
#define SOC_GPIO_PIN_M1_VUSB_SEN  _PINNUM(1,  3) // P1.03
#define SOC_GPIO_PIN_M1_VGPS_SEN  _PINNUM(1,  1) // P1.01

/************** Thinknode M3 section *************/

/* Peripherals */
#define SOC_GPIO_PIN_CONS_M3_RX   _PINNUM(0,  9) // P0.09 , No NFC
#define SOC_GPIO_PIN_CONS_M3_TX   _PINNUM(0, 10) // P0.10 , No NFC

/* L76K */
#define SOC_GPIO_PIN_GNSS_M3_RX   _PINNUM(0, 20) // P0.20
#define SOC_GPIO_PIN_GNSS_M3_TX   _PINNUM(0, 22) // P0.22

#define SOC_GPIO_PIN_GNSS_M3_PPS  SOC_UNUSED_PIN // TBD
#define SOC_GPIO_PIN_GNSS_M3_WKE  _PINNUM(0, 21) // P0.21
#define SOC_GPIO_PIN_GNSS_M3_RST  _PINNUM(0, 25) // P0.25 active HIGH
#define SOC_GPIO_PIN_GNSS_M3_EN   _PINNUM(0, 14) // P0.14 active HIGH

/* SPI */
#define SOC_GPIO_PIN_M3_MOSI      _PINNUM(1, 14) // P1.14
#define SOC_GPIO_PIN_M3_MISO      _PINNUM(1, 15) // P1.15
#define SOC_GPIO_PIN_M3_SCK       _PINNUM(1, 13) // P1.13
#define SOC_GPIO_PIN_M3_SS        _PINNUM(1, 12) // P1.12

/* LR1110 */
#define SOC_GPIO_PIN_M3_RST       _PINNUM(1, 10) // P1.10
#define SOC_GPIO_PIN_M3_DIO9      _PINNUM(1,  8) // P1.08
#define SOC_GPIO_PIN_M3_BUSY      _PINNUM(1, 11) // P1.11

/* I2C */
#define SOC_GPIO_PIN_M3_SDA       _PINNUM(0, 26) // P0.26
#define SOC_GPIO_PIN_M3_SCL       _PINNUM(0, 27) // P0.27

#define SOC_GPIO_PIN_M3_EEPROM_EN _PINNUM(0,  7) // P0.07 active HIGH

/* button */
#define SOC_GPIO_PIN_M3_BUTTON    _PINNUM(0, 12) // P0.12
#define SOC_GPIO_PIN_M3_BUT_EN    _PINNUM(0, 16) // P0.16 active HIGH

/* LED */
#define SOC_GPIO_LED_M3_RED       _PINNUM(1,  1) // P1.01
#define SOC_GPIO_LED_M3_GREEN     _PINNUM(1,  3) // P1.03
#define SOC_GPIO_LED_M3_BLUE      _PINNUM(1,  5) // P1.05
#define SOC_GPIO_LED_M3_RGB_PWR   _PINNUM(0, 29) // P0.29 active HIGH

/* buzzer */
#define SOC_GPIO_PIN_M3_BUZZER    _PINNUM(0, 23) // P0.23
#define SOC_GPIO_PIN_M3_EN1       _PINNUM(1,  4) // P1.04 active HIGH
#define SOC_GPIO_PIN_M3_EN2       _PINNUM(1,  2) // P1.02 active HIGH

/* RTC */
#define SOC_GPIO_PIN_RTC_M3_INT   SOC_UNUSED_PIN // TBD

/* ADC */
#define SOC_GPIO_PIN_M3_BATTERY   _PINNUM(0,  5) // P0.05
#define SOC_GPIO_PIN_M3_ADC_EN    _PINNUM(0, 17) // P0.17 active HIGH

#define SOC_ADC_M3_VOLTAGE_DIV    (1.75F)

/* digital input */
#define SOC_GPIO_PIN_M3_BAT_CHRG  _PINNUM(1,  0) // P1.00
#define SOC_GPIO_PIN_M3_BAT_FULL  _PINNUM(0, 24) // P0.24
#define SOC_GPIO_PIN_M3_VUSB_SEN  _PINNUM(0, 31) // P0.31
#define SOC_GPIO_PIN_M3_VGPS_SEN  SOC_UNUSED_PIN // TBD

/* Sensors (SC7A20H + AHT20) */
#define SOC_GPIO_PIN_M3_ACC_EN    _PINNUM(0,  2) // P0.02 active HIGH ?
#define SOC_GPIO_PIN_M3_TEMP_EN   _PINNUM(0,  3) // P0.03 active HIGH

/************** general section *************/

//#define SOC_GPIO_PIN_GNSS_PPS _PINNUM(1, 4) // P1.04
//#define SOC_GPIO_PIN_GNSS_WKE _PINNUM(1, 2) // P1.02
//#define SOC_GPIO_PIN_GNSS_RST _PINNUM(1, 5) // P1.05 (REV_2 only)

#define SOC_GPIO_LED_PCA10059_STATUS    _PINNUM(0,  6) // P0.06
#define SOC_GPIO_LED_PCA10059_GREEN     _PINNUM(1,  9) // P1.09 (Green)
#define SOC_GPIO_LED_PCA10059_RED       _PINNUM(0,  8) // P0.08 (Red)
#define SOC_GPIO_LED_PCA10059_BLUE      _PINNUM(0, 12) // P0.12 (Blue)

// start out with red LED while booting, may change colors later
#define SOC_GPIO_PIN_STATUS   (hw_info.model == SOFTRF_MODEL_CARD ? SOC_GPIO_LED_T1000_RED   : \
                               hw_info.model == SOFTRF_MODEL_POCKET  ? SOC_GPIO_LED_M3_RED   : \
                               hw_info.model == SOFTRF_MODEL_HANDHELD ? SOC_GPIO_LED_M1_RED  : \
                               hw_info.revision == 0 ? SOC_GPIO_LED_TECHO_REV_0_RED : \
                               hw_info.revision == 1 ? SOC_GPIO_LED_TECHO_REV_1_RED : \
                               hw_info.revision == 2 ? SOC_GPIO_LED_TECHO_REV_2_RED : \
                               SOC_UNUSED_PIN /* SOC_GPIO_LED_PCA10059_STATUS */ )

#define SOC_GPIO_LED_USBMSC   (hw_info.model == SOFTRF_MODEL_CARD  ? SOC_GPIO_LED_T1000_RED : \
                               hw_info.model == SOFTRF_MODEL_POCKET   ? SOC_GPIO_LED_M3_RED : \
                               hw_info.model == SOFTRF_MODEL_HANDHELD ? SOC_GPIO_LED_M1_RED : \
                               hw_info.revision == 0 ? SOC_GPIO_LED_TECHO_REV_0_RED : \
                               hw_info.revision == 1 ? SOC_GPIO_LED_TECHO_REV_1_RED : \
                               hw_info.revision == 2 ? SOC_GPIO_LED_TECHO_REV_2_RED : \
                               SOC_UNUSED_PIN /* SOC_GPIO_LED_PCA10059_RED */ )

#define SOC_GPIO_LED_BLE      (hw_info.model == SOFTRF_MODEL_CARD  ? SOC_GPIO_LED_T1000_GREEN : \
                               hw_info.model == SOFTRF_MODEL_POCKET ? SOC_GPIO_LED_M3_BLUE    : \
                               hw_info.model == SOFTRF_MODEL_HANDHELD ? SOC_GPIO_LED_M1_BLUE  : \
                               hw_info.revision == 0 ? SOC_GPIO_LED_TECHO_REV_0_BLUE : \
                               hw_info.revision == 1 ? SOC_GPIO_LED_TECHO_REV_1_BLUE : \
                               hw_info.revision == 2 ? SOC_GPIO_LED_TECHO_REV_2_BLUE : \
                               SOC_UNUSED_PIN /* SOC_GPIO_LED_PCA10059_BLUE */ )

#define SOC_GPIO_PIN_GNSS_PPS (hw_info.model == SOFTRF_MODEL_BADGE    ? \
                               SOC_GPIO_PIN_GNSS_TECHO_PPS :            \
                               hw_info.model == SOFTRF_MODEL_CARD     ? \
                               SOC_GPIO_PIN_GNSS_T1000_PPS :            \
                               hw_info.model == SOFTRF_MODEL_HANDHELD ? \
                               SOC_GPIO_PIN_GNSS_M1_PPS :               \
                               hw_info.model == SOFTRF_MODEL_POCKET   ? \
                               SOC_GPIO_PIN_GNSS_M3_PPS : SOC_UNUSED_PIN)

#define SOC_GPIO_PIN_PCA10059_MOSI      _PINNUM(0, 22) // P0.22
#define SOC_GPIO_PIN_PCA10059_MISO      _PINNUM(0, 13) // P0.13
#define SOC_GPIO_PIN_PCA10059_SCK       _PINNUM(0, 14) // P0.14

#define SOC_GPIO_PIN_WB_MOSI            _PINNUM(1, 12) // P1.12
#define SOC_GPIO_PIN_WB_MISO            _PINNUM(1, 13) // P0.13
#define SOC_GPIO_PIN_WB_SCK             _PINNUM(1, 11) // P1.11
#define SOC_GPIO_PIN_WB_SS              _PINNUM(1, 10) // P1.10

#define SOC_GPIO_PIN_PCA10059_RST       _PINNUM(0, 15) // P0.15

#define SOC_GPIO_PIN_WB_RST   _PINNUM(1,  6) // P1.06
#define SOC_GPIO_PIN_WB_DIO1  _PINNUM(1, 15) // P1.15
#define SOC_GPIO_PIN_WB_BUSY  _PINNUM(1, 14) // P1.14

/* RF antenna switch */
#define SOC_GPIO_PIN_WB_TXEN  _PINNUM(1,  7) // P1.07
#define SOC_GPIO_PIN_WB_RXEN  _PINNUM(1,  5) // P1.05

/* buttons */
#define SOC_GPIO_PIN_PCA10059_BUTTON    _PINNUM(1,  6) // P1.06
#define SOC_GPIO_PIN_BUTTON   (nRF52_board == NRF52_NORDIC_PCA10059 ? \
                               SOC_GPIO_PIN_PCA10059_BUTTON :         \
                               SOC_GPIO_PIN_TECHO_REV_0_BUTTON)

#define EXCLUDE_WIFI
#define EXCLUDE_ETHERNET
#define EXCLUDE_OTA
//#define USE_ARDUINO_WIFI
//#define USE_WIFI_NINA         false
//#define USE_WIFI_CUSTOM       true

#define EXCLUDE_CC13XX
//#define EXCLUDE_TEST_MODE
//#define EXCLUDE_SOFTRF_HEARTBEAT
//#define EXCLUDE_LK8EX1

#define EXCLUDE_GNSS_UBLOX
#define EXCLUDE_GNSS_SONY
#define EXCLUDE_GNSS_MTK
//#define EXCLUDE_GNSS_GOKE     /* 'Air530' GK9501 GPS/GLO/BDS (GAL inop.) */
//#define EXCLUDE_GNSS_AT65     /* Quectel L76K */
#define EXCLUDE_GNSS_UC65
//#define EXCLUDE_GNSS_AG33

/* Component                         Cost */
/* -------------------------------------- */
#define USE_NMEALIB
#define USE_NMEA_CFG               //  +    kb
#define USE_SKYVIEW_CFG            //  +    kb
//#define USE_EGM96
//#define EXCLUDE_BMP180           //  -    kb
//#define EXCLUDE_BMP280           //  -    kb
#define EXCLUDE_BME680             //  -    kb
#define EXCLUDE_BME280AUX          //  -    kb
//#define EXCLUDE_MPL3115A2        //  -    kb
#define EXCLUDE_NRF905           //  -    kb
#define EXCLUDE_MAVLINK          //  -    kb
#define EXCLUDE_UATM             //  -    kb
#define EXCLUDE_UAT978           //  -    kb
#define EXCLUDE_D1090            //  -    kb
#define EXCLUDE_LED_RING         //  -    kb

//#define USE_BASICMAC
#define EXCLUDE_SX1276           //  -  3 kb

//#define USE_OLED                 //  +    kb
//#define EXCLUDE_OLED_BARO_PAGE
//#define EXCLUDE_OLED_049
#define USE_EPAPER                 //  +    kb
#define USE_EPD_TASK
#define USE_TIME_SLOTS
//#define USE_JSON_SETTINGS

/* Experimental */
//#define USE_WEBUSB_SERIAL
//#define USE_WEBUSB_SETTINGS
//#define USE_USB_MIDI
//#define USE_BLE_MIDI
#define USE_PWM_SOUND
//#define USE_GDL90_MSL
//#define USE_IBEACON
//#define EXCLUDE_NUS
//#define EXCLUDE_IMU
#define USE_OGN_ENCRYPTION

#define ENABLE_ADSL
#define ENABLE_PROL
#if !defined(ARDUINO_ARCH_MBED) && !defined(ARDUINO_ARCH_ZEPHYR)
//#define ENABLE_REMOTE_ID
//#define USE_EXT_I2S_DAC
//#define USE_TFT
#define USE_RADIOLIB
//#define EXCLUDE_LR11XX
#define EXCLUDE_LR20XX
#define EXCLUDE_CC1101
#define EXCLUDE_SI443X
#define EXCLUDE_SI446X
#define EXCLUDE_SX1231
#define EXCLUDE_SX1280
//#define ENABLE_RECORDER
//#define ENABLE_NFC
//#define EXCLUDE_BLUETOOTH
#else
#undef USE_EPAPER
//#define EXCLUDE_BLUETOOTH
#define USE_ARDUINOBLE
#define EXCLUDE_IMU
#define EXCLUDE_BME280AUX
#if defined(ARDUINO_ARCH_ZEPHYR)
#define EXCLUDE_BLUETOOTH
#define EXCLUDE_EEPROM
#undef USE_NMEALIB
#define USE_RADIOLIB
#define EXCLUDE_LR20XX
#endif /* ARDUINO_ARCH_ZEPHYR */
#endif /* ARDUINO_ARCH_MBED */

// skip huge firmware blob at least for now
#define EXCLUDE_BHI260
//#define USE_BHI260_RAM_FW

#define EXCLUDE_PMU

#define EXCLUDE_WIP

/* FTD-012 data port protocol version 8 and 9 */
//#define PFLAA_EXT1_FMT  ",%d,%d,%d"
//#define PFLAA_EXT1_ARGS ,Container[i].no_track,data_source,Container[i].rssi

// these are defined in nRF52.cpp and called from Buzzer.cpp:
void noToneAC();
void toneAC(unsigned long frequency, uint8_t volume = 10, unsigned long length = 0, uint8_t background = 0);

// called from SoftRF.ino:
void nRF52_charge_mode();

// called from Battery.cpp:
bool nRF52_onExternalPower();

#if defined(USE_PWM_SOUND)
#define SOC_GPIO_PIN_BUZZER   (nRF52_board == NRF52_SEEED_T1000E  ? SOC_GPIO_PIN_T1000_BUZZER : \
                               nRF52_board == NRF52_ELECROW_TN_M1 ? SOC_GPIO_PIN_M1_BUZZER    : \
                               nRF52_board == NRF52_ELECROW_TN_M3 ? SOC_GPIO_PIN_M3_BUZZER    : \
                               nRF52_board == NRF52_LILYGO_TECHO_PLUS ? SOC_GPIO_PIN_TECHO_BUZZER : \
                               hw_info.rf != RF_IC_SX1262 ? SOC_UNUSED_PIN           : \
                               hw_info.revision == 1 ? SOC_GPIO_PIN_TECHO_REV_1_DIO0 : \
                               hw_info.revision == 2 ? SOC_GPIO_PIN_TECHO_REV_2_DIO0 : \
                               SOC_UNUSED_PIN)

#define ALARM_TONE_HZ         2480 // seems to be the best value for 27 mm piezo buzzer
#else
#define SOC_GPIO_PIN_BUZZER   SOC_UNUSED_PIN
#endif /* USE_PWM_SOUND */

#define SOC_GPIO_PIN_EXT_PWR  (nRF52_board == NRF52_SEEED_T1000E  ? SOC_GPIO_PIN_T1000_CHG_PWR : \
                               nRF52_board == NRF52_ELECROW_TN_M1 ? SOC_GPIO_PIN_M1_VUSB_SEN   : \
                               nRF52_board == NRF52_ELECROW_TN_M3 ? SOC_GPIO_PIN_M3_VUSB_SEN   : \
                               SOC_UNUSED_PIN)
                               // T-Echo has no ext_power_pin

#if !defined(EXCLUDE_LED_RING)
#include <Adafruit_NeoPixel.h>

extern Adafruit_NeoPixel strip;
#endif /* EXCLUDE_LED_RING */

#if !defined(PIN_SERIAL2_RX) && !defined(PIN_SERIAL2_TX)
extern Uart Serial2;
#endif

extern PCF8563_Class *rtc;
extern const char *nRF52_Device_Manufacturer, *nRF52_Device_Model, *Hardware_Rev[];

#if defined(USE_EPAPER)
typedef void EPD_Task_t;
#endif /* USE_EPAPER */

#if defined(USE_OLED)
#define U8X8_OLED_I2C_BUS_TYPE          U8X8_SH1106_128X64_NONAME_HW_I2C
extern bool nRF52_OLED_probe_func();
#define plat_oled_probe_func            nRF52_OLED_probe_func
#endif /* USE_OLED */

extern FatFileSystem fatfs;
extern bool FATFS_is_mounted;
extern bool USBMSC_flag;

#endif /* PLATFORM_NRF52_H */

#endif /* ARDUINO_ARCH_NRF52 */
