/*
 * Platform_nRF52.cpp
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

// This version adapted to support the SenseCap T1000E and the Thinknode M3
// in addition to the T-Echo.

#if defined(ARDUINO_ARCH_NRF52)  || defined(ARDUINO_ARCH_NRF52840)

#include <SPI.h>
#include <Wire.h>
#include <pcf8563.h>
#include <Adafruit_SPIFlash.h>
#include "Adafruit_TinyUSB.h"
#include <Adafruit_SleepyDog.h>
#include "nrf_wdt.h"
#include "nrf_nvic.h"

#include "../system/SoC.h"
#include "../driver/RF.h"
#include "../driver/Settings.h"
#include "../driver/Filesys.h"
#include "../driver/GNSS.h"
#include "../driver/Baro.h"
#include "../driver/LED.h"
#include "../driver/Bluetooth.h"
#include "../driver/EPD.h"
#include "../driver/Battery.h"
#include "../driver/Buzzer.h"
#include "../protocol/data/NMEA.h"
#include "../protocol/data/GDL90.h"
#include "../protocol/data/D1090.h"
#include "../protocol/data/IGC.h"
#include "../system/Time.h"

#include "uCDB.hpp"

#if defined(USE_BLE_MIDI)
#include <bluefruit.h>
#endif /* USE_BLE_MIDI */

#if defined(USE_BLE_MIDI) || defined(USE_USB_MIDI)
#include <MIDI.h>
#endif /* USE_BLE_MIDI || USE_USB_MIDI */

typedef volatile uint32_t REG32;
#define pREG32 (REG32 *)

#define DEVICE_ID_HIGH    (*(pREG32 (0x10000060)))
#define DEVICE_ID_LOW     (*(pREG32 (0x10000064)))

#define EPD_STACK_SZ      (256*3)

// RFM95W pin mapping
lmic_pinmap lmic_pins = {
    .nss = SOC_GPIO_PIN_SS,
    .txe = LMIC_UNUSED_PIN,
    .rxe = LMIC_UNUSED_PIN,
    .rst = SOC_GPIO_PIN_PCA10059_RST,
    .dio = {LMIC_UNUSED_PIN, LMIC_UNUSED_PIN, LMIC_UNUSED_PIN},
    .busy = SOC_GPIO_PIN_BUSY,
    .tcxo = LMIC_UNUSED_PIN,
};

#if !defined(EXCLUDE_LED_RING)
// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIX_NUM, SOC_GPIO_PIN_LED,
                              NEO_GRB + NEO_KHZ800);
#endif /* EXCLUDE_LED_RING */

//char UDPpacketBuffer[4]; // Dummy definition to satisfy build sequence

//static uint32_t prev_tx_packets_counter = 0;
//static uint32_t prev_rx_packets_counter = 0;
//extern uint32_t tx_packets_counter, rx_packets_counter;

struct rst_info reset_info = {
  .reason = REASON_DEFAULT_RST,
};

static uint32_t bootCount __attribute__ ((section (".noinit")));

static uint8_t gpregret = 0;
static uint8_t gpregret2 = 0;

static nRF52_board_id nRF52_board = NRF52_UNSUPPORTED;    /* default */
static nRF52_display_id nRF52_display = EP_UNKNOWN;

const char *nRF52_Device_Manufacturer = SOFTRF_IDENT;
const char *nRF52_Device_Model = "Unsupported";
const uint16_t nRF52_Device_Version = SOFTRF_USB_FW_VERSION;
static uint16_t nRF52_USB_VID = 0x239A; /* Adafruit Industries */
static uint16_t nRF52_USB_PID = 0x8029; /* Feather nRF52840 Express */

const char *Hardware_Rev[] = {
  [0] = "2020-8-6",
  [1] = "2020-12-12",
  [2] = "2021-3-26",
  [3] = "Unknown"
};

const prototype_entry_t techo_prototype_boards[] = {
  { 0x684f99bd2d5c7fae, NRF52_LILYGO_TECHO_REV_0, EP_GDEP015OC1,  0 }, /* orange */
  { 0xf353e11726ea8220, NRF52_LILYGO_TECHO_REV_0, EP_GDEH0154D67, 0 }, /* blue   */
  { 0xf4e0f04ded1892da, NRF52_LILYGO_TECHO_REV_1, EP_GDEH0154D67, 0 }, /* green  */
  { 0x65ab5994ea2c9094, NRF52_LILYGO_TECHO_REV_1, EP_GDEH0154D67, 0 }, /* blue   */
  { 0x6460429ea6fb7e39, NRF52_NORDIC_PCA10059,    EP_UNKNOWN,     0 },
/*{ 0x0b0e14e96a3beb79, NRF52_ELECROW_TN_M1,      EP_GDEH0154D67, 0 },*/
};

PCF8563_Class *rtc = nullptr;
I2CBus        *i2c = nullptr;

static bool nRF52_has_rtc      = false;
static bool nRF52_has_spiflash = false;
static bool RTC_sync           = false;
static bool ADB_is_open        = false;
// static bool nRF52_has_vff      = false; /* very first fix */

static uint8_t mode_button_pin = SOC_UNUSED_PIN;
static uint8_t up_button_pin   = SOC_UNUSED_PIN;

#if defined(USE_PWM_SOUND)
static uint8_t buzzerpin = SOC_UNUSED_PIN;
#endif

bool FATFS_is_mounted = false;

RTC_Date fw_build_date_time = RTC_Date(__DATE__, __TIME__);

static TaskHandle_t EPD_Task_Handle = NULL;

#if !defined(ARDUINO_NRF52840_PCA10056)
#error "This nRF52 build variant is not supported!"
#endif

#if SPI_32MHZ_INTERFACE == 0
#define _SPI_DEV    NRF_SPIM3 // 32 Mhz
#define _SPI1_DEV   NRF_SPIM2
#elif SPI_32MHZ_INTERFACE == 1
#define _SPI_DEV    NRF_SPIM2
#define _SPI1_DEV   NRF_SPIM3 // 32 Mhz
#else
  #error "not supported yet"
#endif

#if defined(USE_EPAPER)

#if SPI_INTERFACES_COUNT == 1
SPIClass SPI1(_SPI1_DEV,
              SOC_GPIO_PIN_EPD_MISO,
              SOC_GPIO_PIN_EPD_SCK,
              SOC_GPIO_PIN_EPD_MOSI);
#endif

#if 0   // will be done later once board is known
GxEPD2_BW<GxEPD2_154_D67, GxEPD2_154_D67::HEIGHT> epd_d67(GxEPD2_154_D67(
                                                          SOC_GPIO_PIN_EPD_SS,
                                                          SOC_GPIO_PIN_EPD_DC,
                                                          SOC_GPIO_PIN_EPD_RST,
                                                          SOC_GPIO_PIN_EPD_BUSY));
GxEPD2_BW<GxEPD2_154, GxEPD2_154::HEIGHT>         epd_c1 (GxEPD2_154(
                                                          SOC_GPIO_PIN_EPD_SS,
                                                          SOC_GPIO_PIN_EPD_DC,
                                                          SOC_GPIO_PIN_EPD_RST,
                                                          SOC_GPIO_PIN_EPD_BUSY));
GxEPD2_BW<GxEPD2_150_BN, GxEPD2_150_BN::HEIGHT>   epd_bn (GxEPD2_150_BN(
                                                          SOC_GPIO_PIN_EPD_SS,
                                                          SOC_GPIO_PIN_EPD_DC,
                                                          SOC_GPIO_PIN_EPD_RST,
                                                          SOC_GPIO_PIN_EPD_BUSY));
#endif

GxEPD2_GFX *display;
#endif /* USE_EPAPER */

static Adafruit_FlashTransport_QSPI *FlashTrans = NULL;
static Adafruit_SPIFlash            *SPIFlash   = NULL;

#if 0   // will be done later once board is known
Adafruit_FlashTransport_QSPI HWFlashTransport(SOC_GPIO_PIN_SFL_SCK,
                                              SOC_GPIO_PIN_SFL_SS,
                                              SOC_GPIO_PIN_SFL_MOSI,
                                              SOC_GPIO_PIN_SFL_MISO,
                                              SOC_GPIO_PIN_SFL_WP,
                                              SOC_GPIO_PIN_SFL_HOLD);
Adafruit_SPIFlash QSPIFlash (&HWFlashTransport);
#endif

#define SFLASH_CMD_READ_CONFIG  0x15

static uint32_t spiflash_id = 0;
static uint8_t mx25_status_config[3] = {0x00, 0x00, 0x00};

/// Flash device list count
enum {
  MX25R1635F_INDEX,
  ZD25WQ16B_INDEX,
  GD25Q64C_INDEX,
  P25Q16H_INDEX,
  W25Q128JV_INDEX,
  EXTERNAL_FLASH_DEVICE_COUNT
};

/// List of all possible flash devices used by nRF52840 boards
static SPIFlash_Device_t possible_devices[] = {
  // LilyGO T-Echo, Elecrow ThinkNode M1
  [MX25R1635F_INDEX] = MX25R1635F,
  // LilyGO T-Echo
  [ZD25WQ16B_INDEX]  = ZD25WQ16B,
  // Seeed T1000-E
  [GD25Q64C_INDEX]   = GD25Q64C,
  // Seeed Wio L1
  [P25Q16H_INDEX]    = P25Q16H,
  // LilyGO T-Ultima
  [W25Q128JV_INDEX]  = W25Q128JV_PM,
};

// USB Mass Storage object
Adafruit_USBD_MSC usb_msc;

// file system object from SdFat
FatFileSystem fatfs;

#if defined(USE_WEBUSB_SERIAL) || defined(USE_WEBUSB_SETTINGS)
// USB WebUSB object
Adafruit_USBD_WebUSB usb_web;
#endif

#if defined(USE_WEBUSB_SERIAL)
// Landing Page: scheme (0: http, 1: https), url
WEBUSB_URL_DEF(landingPage, 1 /*https*/, "adafruit.github.io/Adafruit_TinyUSB_Arduino/examples/webusb-serial/index.html");
#endif /* USE_WEBUSB_SERIAL */

#if defined(USE_WEBUSB_SETTINGS)
// Landing Page: scheme (0: http, 1: https), url
WEBUSB_URL_DEF(landingPage, 1 /*https*/, "lyusupov.github.io/SoftRF/settings.html");
#endif /* USE_WEBUSB_SETTINGS */

#if defined(USE_USB_MIDI)
// USB MIDI object
Adafruit_USBD_MIDI usb_midi;

// Create a new instance of the Arduino MIDI Library,
// and attach usb_midi as the transport.
MIDI_CREATE_INSTANCE(Adafruit_USBD_MIDI, usb_midi, MIDI_USB);
#endif /* USE_USB_MIDI */

//ui_settings_t ui_settings;
/* = {
#if defined(DEFAULT_REGION_US)
    .units        = UNITS_IMPERIAL,
#else
    .units        = UNITS_METRIC,
#endif
    .zoom         = ZOOM_MEDIUM,
    .protocol     = PROTOCOL_NMEA,
    .rotate       = ROTATE_0,
    .orientation  = DIRECTION_TRACK_UP,
    .adb          = DB_NONE,
    .idpref       = ID_TYPE,
    .vmode        = VIEW_MODE_STATUS,
    .voice        = VOICE_OFF,
    .aghost       = ANTI_GHOSTING_OFF,
    .filter       = TRAFFIC_FILTER_OFF,
    .team         = 0
}; */
//ui_settings_t *ui;

#if !defined(EXCLUDE_IMU)
#define IMU_UPDATE_INTERVAL 500 /* ms */

#include <MPU9250.h>
#include <ICM_20948.h>
#include <QMA6100P.h>
#if !defined(EXCLUDE_BHI260)
#include <SensorBHI260AP.hpp>
#include <bosch/BoschSensorDataHelper.hpp>

#if defined(USE_BHI260_RAM_FW)
#include "bosch/firmware/bosch_app30_shuttle_bhi260.h"
#endif /* USE_BHI260_RAM_FW */
#endif /* EXCLUDE_BHI260 */

MPU9250         imu_1;
ICM_20948_I2C   imu_2;
QMA6100P        imu_3;
#if !defined(EXCLUDE_BHI260)
SensorBHI260AP  imu_4;

SensorXYZ bhi_accel(SensorBHI260AP::ACCEL_PASSTHROUGH, imu_4);
// SensorXYZ bhi_gyro(SensorBHI260AP::GYRO_PASSTHROUGH, imu_4);
#endif /* EXCLUDE_BHI260 */

static bool nRF52_has_imu = false;
static unsigned long IMU_Time_Marker = 0;

extern float IMU_g;
#endif /* EXCLUDE_IMU */

#include <SensorDRV2605.hpp>
SensorDRV2605 vibra;
static bool nRF52_has_vibra = false;

#include <AHT20.h>
AHT20 aht20;

//uCDB<FatFileSystem, File> ucdb(fatfs);
uCDB<FatVolume, File32> ucdb(fatfs);

// Callback invoked when received READ10 command.
// Copy disk's data to buffer (up to bufsize) and
// return number of copied bytes (must be multiple of block size)
static int32_t nRF52_msc_read_cb (uint32_t lba, void* buffer, uint32_t bufsize)
{
  // Note: SPIFLash Bock API: readBlocks/writeBlocks/syncBlocks
  // already include 4K sector caching internally. We don't need to cache it, yahhhh!!
  return SPIFlash->readBlocks(lba, (uint8_t*) buffer, bufsize/512) ? bufsize : -1;
}

// Callback invoked when received WRITE10 command.
// Process data in buffer to disk's storage and
// return number of written bytes (must be multiple of block size)
static int32_t nRF52_msc_write_cb (uint32_t lba, uint8_t* buffer, uint32_t bufsize)
{
  uint8_t USBMSC_LED = SOC_GPIO_LED_USBMSC;
  if (USBMSC_LED != SOC_UNUSED_PIN) {
      digitalWrite(USBMSC_LED, LED_STATE_ON);
  }
  USBMSC_flag = true;

  // Note: SPIFLash Bock API: readBlocks/writeBlocks/syncBlocks
  // already include 4K sector caching internally. We don't need to cache it, yahhhh!!
  return SPIFlash->writeBlocks(lba, buffer, bufsize/512) ? bufsize : -1;
}

bool USBMSC_flag = false;

// Callback invoked when WRITE10 command is completed (status received and accepted by host).
// used to flush any pending cache.
static void nRF52_msc_flush_cb (void)
{
  // sync with flash
  SPIFlash->syncBlocks();

  // clear file system's cache to force refresh
  fatfs.cacheClear();

  uint8_t USBMSC_LED = SOC_GPIO_LED_USBMSC;
  if (USBMSC_LED != SOC_UNUSED_PIN) {
      digitalWrite(USBMSC_LED, 1-LED_STATE_ON);
  }
  USBMSC_flag = false;
}

#if !defined(EXCLUDE_PMU)
#define  POWERS_CHIP_SY6970
#define  SDA    SOC_GPIO_PIN_SDA
#define  SCL    SOC_GPIO_PIN_SCL
#include <XPowersLib.h>

PowersSY6970 sy6970;

static bool nRF52_has_pmu = false;
#endif /* EXCLUDE_PMU */

#include <ExtensionIOXL9555.hpp>
ExtensionIOXL9555 *xl9555 = nullptr;
bool nRF52_has_extension  = false;

#include <SenseCAP.h>

static bool nRF52_bl_check(const char* signature)
{
  int  i, j;
  bool match    = false;
  uint8_t* data = (uint8_t*) 0x000F4000UL;
  int length    = 0xFE000 - 0xF4000 - 2048 ; /* 38 KB */
  int str_len   = strlen(signature);

  for (i = 0; i < length - str_len; i++) {
    if (data[i] == signature[0]) {
      match = true;
      for (j = 1; j < str_len; j++) {
        if (data[i + j] != signature[j]) {
            match = false;
            break;
        }
      }
      if (match) {
        return true;
      }
    }
  }
  return false;
}

static uint8_t ext_power_pin = SOC_UNUSED_PIN;

static void nRF52_system_off(int reason)
{
#if !defined(ARDUINO_ARCH_MBED) && !defined(ARDUINO_ARCH_ZEPHYR)
  uint8_t sd_en;
  (void) sd_softdevice_is_enabled(&sd_en);

    // Clear LATCH before sleep to prevent stale DETECT assertion:
    //NRF_GPIO->LATCH = NRF_GPIO->LATCH;
    NRF_GPIO->LATCH = 0xFFFFFFFF;
    // clear stale reset reasons:
    NRF_POWER->RESETREAS = 0xFFFFFFFF;

  // Enter System OFF state
  if ( sd_en ) {

    sd_power_system_off();

  } else {

    NRF_POWER->SYSTEMOFF = 1;

  }
#else
  NRF_POWER->SYSTEMOFF = 1;
#endif /* ARDUINO_ARCH_MBED */
}

static uint32_t reset_reason;
static uint32_t latch;

static void nRF52_WDT_fini();   // forward declaration, resets WDT timer


static void nRF52_setup()
{
#if !defined(ARDUINO_ARCH_MBED) && !defined(ARDUINO_ARCH_ZEPHYR)

  latch = NRF_GPIO->LATCH;

  /* uint32_t */ reset_reason = readResetReason();

  // Clear LATCH to prevent stale DETECT assertion:
  //NRF_GPIO->LATCH = NRF_GPIO->LATCH;
  NRF_GPIO->LATCH = 0xFFFFFFFF;
  // clear stale reset reasons:
  NRF_POWER->RESETREAS = 0xFFFFFFFF;

#else
  /* uint32_t */ reset_reason = 0; /* TBD */
#endif /* ARDUINO_ARCH_MBED */

  if      (reset_reason & POWER_RESETREAS_RESETPIN_Msk)
  {
      reset_info.reason = REASON_EXT_SYS_RST;
  }
  else if (reset_reason & POWER_RESETREAS_DOG_Msk)
  {
      reset_info.reason = REASON_WDT_RST;
  }
  else if (reset_reason & POWER_RESETREAS_SREQ_Msk)
  {
      reset_info.reason = REASON_SOFT_RESTART;
  }
  else if (reset_reason & POWER_RESETREAS_LOCKUP_Msk)
  {
      reset_info.reason = REASON_SOFT_WDT_RST;
  }
  else if (reset_reason & POWER_RESETREAS_OFF_Msk)
  {
      reset_info.reason = REASON_DEEP_SLEEP_AWAKE;
  }
  else if (reset_reason & POWER_RESETREAS_LPCOMP_Msk)
  {
      reset_info.reason = REASON_DEEP_SLEEP_AWAKE;
  }
  else if (reset_reason & POWER_RESETREAS_DIF_Msk)
  {
      reset_info.reason = REASON_DEEP_SLEEP_AWAKE;
  }
  else if (reset_reason & POWER_RESETREAS_NFC_Msk)
  {
      reset_info.reason = REASON_DEEP_SLEEP_AWAKE;
  }
  else if (reset_reason & POWER_RESETREAS_VBUS_Msk)
  {
      reset_info.reason = REASON_DEEP_SLEEP_AWAKE;
  }

  gpregret  = NRF_POWER->GPREGRET;
  gpregret2 = NRF_POWER->GPREGRET2;

  /* inactivate initVariant() of PCA10056 */
  pinMode(PIN_LED1, INPUT);
  pinMode(PIN_LED2, INPUT);
  pinMode(PIN_LED3, INPUT);
  pinMode(PIN_LED4, INPUT);

  nRF52_board = nRF52_bl_check("TECHOBOOT")   ? NRF52_LILYGO_TECHO_REV_2 :
                nRF52_bl_check("T1000-E")     ? NRF52_SEEED_T1000E       :
                nRF52_bl_check("ThinkNodeM1") ? NRF52_ELECROW_TN_M1      :
                nRF52_bl_check("ThinkNodeM3") ? NRF52_ELECROW_TN_M3      :
                nRF52_bl_check("ELECROWBOOT") ? NRF52_ELECROW_OTHER      :
                nRF52_bl_check("NRF52BOOT")   ? NRF52_LILYGO_TECHO_REV_2 :
                NRF52_UNSUPPORTED;

  if (nRF52_board == NRF52_LILYGO_TECHO_REV_2) {
      pinMode(SOC_GPIO_PIN_3V3_PWR, INPUT);
      pinMode(SOC_GPIO_PIN_IO_PWR,  INPUT_PULLUP);
  }

  if (nRF52_board == NRF52_LILYGO_TECHO_REV_2 || nRF52_board == NRF52_UNSUPPORTED) {
    for (int i=0; i < sizeof(techo_prototype_boards) / sizeof(prototype_entry_t); i++) {
      if (techo_prototype_boards[i].id == ((uint64_t) DEVICE_ID_HIGH << 32 | (uint64_t) DEVICE_ID_LOW)) {
        nRF52_board   = techo_prototype_boards[i].rev;
        nRF52_display = techo_prototype_boards[i].panel;
        break;
      }
    }
  }

  if (nRF52_board == NRF52_LILYGO_TECHO_REV_1) {
    pinMode(SOC_GPIO_PIN_TECHO_REV_1_3V3_PWR,  INPUT_PULLUP);
  }

  if (nRF52_board != NRF52_ELECROW_TN_M1 &&
      nRF52_board != NRF52_ELECROW_TN_M3 &&
      nRF52_board != NRF52_LILYGO_TECHO_REV_0 &&
      nRF52_board != NRF52_LILYGO_TECHO_REV_1 &&
      nRF52_board != NRF52_LILYGO_TECHO_REV_2
      // && nRF52_board != NRF52_UNSUPPORTED
      ) {
#if !defined(EXCLUDE_IMU)
    pinMode(SOC_GPIO_PIN_T1000_ACC_EN, INPUT_PULLUP);
    delay(5);

#if !defined(ARDUINO_ARCH_MBED) && !defined(ARDUINO_ARCH_ZEPHYR)
    Wire.setPins(SOC_GPIO_PIN_T1000_SDA, SOC_GPIO_PIN_T1000_SCL);
#endif /* ARDUINO_ARCH_MBED */
    Wire.begin();
    Wire.beginTransmission(QMA6100P_ADDRESS);
    nRF52_has_imu = (Wire.endTransmission() == 0);
    Wire.end();
    pinMode(SOC_GPIO_PIN_T1000_ACC_EN, INPUT);

nRF52_WDT_fini();

    if (nRF52_has_imu || nRF52_board == NRF52_SEEED_T1000E) {
      nRF52_board        = NRF52_SEEED_T1000E;
      hw_info.model      = SOFTRF_MODEL_CARD;
      nRF52_Device_Model = "Card Edition";
      nRF52_USB_VID      = 0x2886; /* Seeed Technology */
      nRF52_USB_PID      = 0x0057; /* SenseCAP T1000-E */

#if 1
#if !defined(ARDUINO_ARCH_MBED) && !defined(ARDUINO_ARCH_ZEPHYR)

      //if (reset_reason & POWER_RESETREAS_VBUS_Msk)
      // - on T1000E, CHG_PWR wakeup is like any GPIO pin wakeup:
      if ((reset_reason & POWER_RESETREAS_VBUS_Msk)
      || (latch & (1 << SOC_GPIO_PIN_T1000_CHG_PWR)) != 0) {

        // wakeup was due to USB power, finish booting, may go into "charge mode" later
        NRF_POWER->GPREGRET2 = CHARGE_MAGIC;

      } else {      // if wake-up was NOT due to connection of USB power

        // detect the SHUTDOWN_MAGIC from the initial shutdown (software reset)
        // and go to sleep - this time in a lower power state.
        // But allow USB power to cause a full boot!
        //  Skipped the next two lines:
        //if (reset_reason & POWER_RESETREAS_VBUS_Msk ||
        //    reset_reason & POWER_RESETREAS_RESETPIN_Msk ||

        if ((reset_reason & POWER_RESETREAS_SREQ_Msk) && NRF_POWER->GPREGRET2 == SHUTDOWN_MAGIC) {
          // this startup is following the reset() call in nRF52_fini()

          // Only if no Vbus at the moment:
          // Arrange for connection of USB power to cause a wake-up
          //   - apparently disconnecting from USB power also wakes up!
          // Only do this if currently no USB power, otherwise will wake up on disconnect.
          // Thus, if turned off while USB power is on, USB power will not wake it up,
          //   but next turn-off while USB power is off will "arm" this wake-up method
          // Alas it seems that the T1000E always wakes up when USB power is disconnected?
          pinMode(SOC_GPIO_PIN_T1000_CHG_PWR, INPUT);
          delay(100);
          bool chg_pwr = digitalRead(SOC_GPIO_PIN_T1000_CHG_PWR);
          if (! chg_pwr)   // if USB power is off
              pinMode(SOC_GPIO_PIN_T1000_CHG_PWR, INPUT_SENSE_HIGH);

          // Always configure PIN_CNF[n].INPUT before PIN_CNF[n].SENSE:
          pinMode(SOC_GPIO_PIN_T1000_BUTTON, INPUT_PULLDOWN);
          // Make sure that a pin is in a level that cannot trigger the sense mechanism before enabling it.
#if 0
          delay(100);
          while (digitalRead(SOC_GPIO_PIN_T1000_BUTTON) == HIGH) {
digitalWrite(SOC_GPIO_LED_T1000_GREEN, HIGH);
pinMode(SOC_GPIO_LED_T1000_GREEN, OUTPUT);
              delay(100);
digitalWrite(SOC_GPIO_LED_T1000_GREEN, LOW);
delay(200);
              nRF52_WDT_fini();
          }
pinMode(SOC_GPIO_LED_T1000_GREEN, INPUT);
#else
// make sure at least one green blink is shown if reached this point
          do {
digitalWrite(SOC_GPIO_LED_T1000_GREEN, HIGH);
pinMode(SOC_GPIO_LED_T1000_GREEN, OUTPUT);
              delay(150);
digitalWrite(SOC_GPIO_LED_T1000_GREEN, LOW);
pinMode(SOC_GPIO_LED_T1000_GREEN, INPUT);
delay(150);
              nRF52_WDT_fini();
          } while (digitalRead(SOC_GPIO_PIN_T1000_BUTTON) == HIGH);
#endif
          // configure button for wakeup
          pinMode(SOC_GPIO_PIN_T1000_BUTTON, INPUT_PULLDOWN_SENSE);

          // configure some additional pins for minimal power
          //pinMode(SOC_GPIO_PIN_T1000_3V3_EN, INPUT_PULLDOWN);
          //pinMode(SOC_GPIO_PIN_SFL_T1000_EN, INPUT);            // or INPUT_PULLDOWN ?
          //pinMode(SOC_GPIO_PIN_T1000_SS, INPUT);  // >>> "SOC_GPIO_PIN_IO_PWR" in Vlad's code

          // >>> LR1110 NSS must stay high to prevent spurious wakeup ?
          //pinMode(SOC_GPIO_PIN_T1000_SS, INPUT_PULLUP);

          // Clear LATCH to prevent stale DETECT assertion:
          //NRF_GPIO->LATCH = NRF_GPIO->LATCH;
          NRF_GPIO->LATCH = 0xFFFFFFFF;

          // go into deep sleep
          NRF_POWER->GPREGRET = DFU_MAGIC_SKIP;
          if (chg_pwr)
              NRF_POWER->GPREGRET2 = USB_MAGIC;
          else
              NRF_POWER->GPREGRET2 = SLEEP_MAGIC;
          nRF52_system_off(REASON_DEFAULT_RST);

        }   // end of SHUTDOWN_MAGIC

        // if no SHUTDOWN_MAGIC then stay awake and finish booting
        NRF_POWER->GPREGRET2 = AWAKE_MAGIC;

      }
#endif /* ARDUINO_ARCH_MBED */

      // turn off "sense":
      pinMode(SOC_GPIO_PIN_T1000_CHG_PWR, INPUT);
      pinMode(SOC_GPIO_PIN_T1000_BUTTON, INPUT_PULLDOWN);

      // enable peripherals
      digitalWrite(SOC_GPIO_PIN_SFL_T1000_EN, HIGH);
      pinMode(SOC_GPIO_PIN_SFL_T1000_EN, OUTPUT);

      // digitalWrite(SOC_GPIO_LED_T1000_RED, HIGH);
      // pinMode(SOC_GPIO_LED_T1000_RED, OUTPUT);
#endif
    }        // end of if(T1000E)

#endif /* EXCLUDE_IMU */

  }  // end of if (nRF52_board != several models)

  if (nRF52_board == NRF52_ELECROW_OTHER) {     // "ELECROWBOOT"
    pinMode(SOC_GPIO_PIN_M3_EEPROM_EN,  INPUT_PULLUP);
    pinMode(SOC_GPIO_PIN_M3_TEMP_EN,    INPUT_PULLUP);
    delay(5);
#if !defined(ARDUINO_ARCH_MBED) && !defined(ARDUINO_ARCH_ZEPHYR)
    Wire.setPins(SOC_GPIO_PIN_M3_SDA, SOC_GPIO_PIN_M3_SCL);
#endif /* ARDUINO_ARCH_MBED */
    Wire.begin();
    Wire.beginTransmission(AHT20_ADDRESS);
    if (Wire.endTransmission() == 0)
        nRF52_board = NRF52_ELECROW_TN_M3;    // known to have AHT20
    else
        nRF52_board = NRF52_ELECROW_TN_M1;    // rash assumption
    Wire.end();
    pinMode(SOC_GPIO_PIN_M3_TEMP_EN,    INPUT);
    pinMode(SOC_GPIO_PIN_M3_EEPROM_EN,  INPUT);
  }

  if (nRF52_board == NRF52_ELECROW_TN_M1) {  // "ThinkNodeM1" or ELECROWBOOT without AHT20
#if 1
#if !defined(ARDUINO_ARCH_MBED) && !defined(ARDUINO_ARCH_ZEPHYR)
    if (reset_reason & POWER_RESETREAS_VBUS_Msk) {
      NRF_POWER->GPREGRET2 = CHARGE_MAGIC;
    } else {
      if ((reset_reason & POWER_RESETREAS_SREQ_Msk) && NRF_POWER->GPREGRET2 == SHUTDOWN_MAGIC) {
          pinMode(SOC_GPIO_PIN_M1_VUSB_SEN, INPUT);
          delay(100);
          bool chg_pwr = digitalRead(SOC_GPIO_PIN_M1_VUSB_SEN);
          if (! chg_pwr)   // if USB power is off
              pinMode(SOC_GPIO_PIN_M1_VUSB_SEN, INPUT_SENSE_HIGH);
          pinMode(SOC_GPIO_PIN_M1_BUTTON1, INPUT_PULLUP);
          delay(100);
          while (digitalRead(SOC_GPIO_PIN_M1_BUTTON1) == LOW) {
              delay(100);
              nRF52_WDT_fini();
          }
          pinMode(SOC_GPIO_PIN_M1_BUTTON1, INPUT_PULLUP_SENSE);
          pinMode(SOC_GPIO_PIN_IO_M1_PWR, INPUT);
          NRF_GPIO->LATCH = 0xFFFFFFFF;
          NRF_POWER->GPREGRET = DFU_MAGIC_SKIP;
          if (chg_pwr)
              NRF_POWER->GPREGRET2 = USB_MAGIC;
          else
              NRF_POWER->GPREGRET2 = SLEEP_MAGIC;
          nRF52_system_off(REASON_DEFAULT_RST);
      }
      NRF_POWER->GPREGRET2 = AWAKE_MAGIC;
    }
#endif /* ARDUINO_ARCH_MBED */
#endif
    pinMode(SOC_GPIO_PIN_M1_VUSB_SEN, INPUT);
    pinMode(SOC_GPIO_PIN_M1_BUTTON1, INPUT_PULLUP);
    digitalWrite(SOC_GPIO_PIN_IO_M1_PWR, HIGH);
    pinMode(SOC_GPIO_PIN_IO_M1_PWR, OUTPUT);
  }

  if (nRF52_board == NRF52_ELECROW_TN_M3) {  /* "ThinkNodeM3" or AHT20 */
      hw_info.model      = SOFTRF_MODEL_POCKET;
      nRF52_Device_Model = "Pocket Edition";
#if 1
#if !defined(ARDUINO_ARCH_MBED) && !defined(ARDUINO_ARCH_ZEPHYR)
      if (reset_reason & POWER_RESETREAS_VBUS_Msk) {

        // wakeup was due to USB power, finish booting, may go into "charge mode" later
        NRF_POWER->GPREGRET2 = CHARGE_MAGIC;

      } else {      // if wake-up was NOT due to connection of USB power

        // detect the SHUTDOWN_MAGIC from the initial shutdown (software reset)
        // and go to sleep - this time in a lower power state.
        // But allow USB power to cause a full boot!
        //  Skipped the next two lines:
        //if (reset_reason & POWER_RESETREAS_VBUS_Msk ||
        //    reset_reason & POWER_RESETREAS_RESETPIN_Msk ||

        if ((reset_reason & POWER_RESETREAS_SREQ_Msk) && NRF_POWER->GPREGRET2 == SHUTDOWN_MAGIC) {
          // this startup is following the reset() call in nRF52_fini()

          // Only if no Vbus at the moment:
          // Arrange for connection of USB power to cause a wake-up
          //   - apparently disconnecting from USB power also wakes up!
          // Does not work on T-Echo since it has no ext_power_pin
          // Only do this if currently no USB power, otherwise will wake up on disconnect.
          // Thus, if turned off while USB power is on, USB power will not wake it up,
          //   but next turn-off while USB power is off will "arm" this wake-up method
          pinMode(SOC_GPIO_PIN_M3_VUSB_SEN, INPUT);
          delay(100);
          bool chg_pwr = digitalRead(SOC_GPIO_PIN_M3_VUSB_SEN);
          if (! chg_pwr)   // if USB power is off
              pinMode(SOC_GPIO_PIN_M3_VUSB_SEN, INPUT_PULLDOWN_SENSE);
          pinMode(SOC_GPIO_PIN_M3_BUT_EN, INPUT_PULLUP);
          // Always configure PIN_CNF[n].INPUT before PIN_CNF[n].SENSE:
          pinMode(SOC_GPIO_PIN_M3_BUTTON, INPUT_PULLUP);
          // Make sure that a pin is in a level that cannot trigger the sense mechanism before enabling it.
          delay(100);
          while (digitalRead(SOC_GPIO_PIN_M3_BUTTON) == LOW) {
              delay(100);
              nRF52_WDT_fini();
          }
          // configure button for wakeup
          pinMode(SOC_GPIO_PIN_M3_BUTTON, INPUT_PULLUP_SENSE);
          // Clear LATCH to prevent stale DETECT assertion:
          //NRF_GPIO->LATCH = NRF_GPIO->LATCH;
          NRF_GPIO->LATCH = 0xFFFFFFFF;
          // go into deep sleep
          NRF_POWER->GPREGRET = DFU_MAGIC_SKIP;
          if (chg_pwr)
              NRF_POWER->GPREGRET2 = USB_MAGIC;
          else
              NRF_POWER->GPREGRET2 = SLEEP_MAGIC;
          nRF52_system_off(REASON_DEFAULT_RST);

        }   // end of SHUTDOWN_MAGIC

        // if no SHUTDOWN_MAGIC then stay awake and finish booting
        NRF_POWER->GPREGRET2 = AWAKE_MAGIC;
      }
#endif /* ARDUINO_ARCH_MBED */
#endif
      // turn off "sense":
      pinMode(SOC_GPIO_PIN_M3_VUSB_SEN, INPUT);
      pinMode(SOC_GPIO_PIN_M3_BUTTON, INPUT_PULLUP);
      pinMode(SOC_GPIO_PIN_M3_BUT_EN, INPUT_PULLUP);
  }

#if !defined(ARDUINO_ARCH_MBED) && !defined(ARDUINO_ARCH_ZEPHYR)
  switch (nRF52_board)
  {
    case NRF52_SEEED_T1000E:
      Wire.setPins(SOC_GPIO_PIN_T1000_SDA, SOC_GPIO_PIN_T1000_SCL);
#if !defined(EXCLUDE_IMU)
      pinMode(SOC_GPIO_PIN_T1000_ACC_EN, INPUT_PULLUP);
      delay(100);
#endif /* EXCLUDE_IMU */
      break;
    case NRF52_ELECROW_TN_M3:
      Wire.setPins(SOC_GPIO_PIN_M3_SDA, SOC_GPIO_PIN_M3_SCL);

#if !defined(EXCLUDE_IMU)
      pinMode(SOC_GPIO_PIN_M3_ACC_EN,       INPUT_PULLUP);
#endif /* EXCLUDE_IMU */

      pinMode(SOC_GPIO_PIN_M3_TEMP_EN,      INPUT_PULLUP);
      pinMode(SOC_GPIO_PIN_M3_EEPROM_EN,    INPUT_PULLUP);
      break;
    case NRF52_LILYGO_TECHO_REV_0:
    case NRF52_LILYGO_TECHO_REV_1:
    case NRF52_LILYGO_TECHO_REV_2:
    case NRF52_LILYGO_TECHO_PLUS:  // but not detected yet!
    case NRF52_ELECROW_TN_M1:
    case NRF52_NORDIC_PCA10059:
      digitalWrite(SOC_GPIO_PIN_IO_PWR, HIGH);
      pinMode(SOC_GPIO_PIN_IO_PWR, OUTPUT); /* VDD_POWR is ON */
    default:
      Wire.setPins(SOC_GPIO_PIN_SDA, SOC_GPIO_PIN_SCL);
      break;
  }
#endif /* ARDUINO_ARCH_MBED */
  Wire.begin();

  Wire.beginTransmission(PCF8563_SLAVE_ADDRESS);
  nRF52_has_rtc = (Wire.endTransmission() == 0);
  if (!nRF52_has_rtc) {
    delay(5);
    Wire.beginTransmission(PCF8563_SLAVE_ADDRESS);
    nRF52_has_rtc = (Wire.endTransmission() == 0);
    if (!nRF52_has_rtc) {
      delay(5);
      Wire.beginTransmission(PCF8563_SLAVE_ADDRESS);
      nRF52_has_rtc = (Wire.endTransmission() == 0);
    }
  }

nRF52_WDT_fini();

#if !defined(EXCLUDE_IMU)
  if (nRF52_board == NRF52_LILYGO_TECHO_REV_2) { /* T-Echo or T114 */
    /* MPU9250 or ICM20948 start-up time for register R/W is 11-100 ms */
    delay(90);

    Wire.beginTransmission(MPU9250_ADDRESS);
    nRF52_has_imu = (Wire.endTransmission() == 0);
    if (nRF52_has_imu == false) {
      Wire.beginTransmission(ICM20948_ADDRESS);
      nRF52_has_imu = (Wire.endTransmission() == 0);
      if (nRF52_has_imu == false) {
        Wire.beginTransmission(BHI260AP_ADDRESS_L);
        nRF52_has_imu = (Wire.endTransmission() == 0);

        if (nRF52_has_imu) {
          Wire.beginTransmission(DRV2605_ADDRESS);
          if (Wire.endTransmission() == 0) {
            nRF52_board = NRF52_LILYGO_TECHO_PLUS;
          }
        }
      }
    }

  } else if (nRF52_board == NRF52_ELECROW_TN_M3) {
    Wire.beginTransmission(SC7A20H_ADDRESS_H);
    nRF52_has_imu = (Wire.endTransmission() == 0);
  }
#endif /* EXCLUDE_IMU */

  Wire.end();

nRF52_WDT_fini();

#if !defined(ARDUINO_ARCH_MBED) && !defined(ARDUINO_ARCH_ZEPHYR)
  /* (Q)SPI flash init */

  switch (nRF52_board)
  {
    case NRF52_LILYGO_TECHO_REV_0:
      possible_devices[MX25R1635F_INDEX].max_clock_speed_mhz  = 33;
      possible_devices[MX25R1635F_INDEX].supports_qspi        = false;
      possible_devices[MX25R1635F_INDEX].supports_qspi_writes = false;
    case NRF52_LILYGO_TECHO_REV_1:
    case NRF52_LILYGO_TECHO_REV_2:
    case NRF52_LILYGO_TECHO_PLUS:
    case NRF52_ELECROW_TN_M1:
      FlashTrans = new Adafruit_FlashTransport_QSPI(SOC_GPIO_PIN_SFL_SCK,
                                                    SOC_GPIO_PIN_SFL_SS,
                                                    SOC_GPIO_PIN_SFL_MOSI,
                                                    SOC_GPIO_PIN_SFL_MISO,
                                                    SOC_GPIO_PIN_SFL_WP,
                                                    SOC_GPIO_PIN_SFL_HOLD);
      break;
    case NRF52_SEEED_T1000E:
      FlashTrans = new Adafruit_FlashTransport_QSPI(SOC_GPIO_PIN_SFL_T1000_SCK,
                                                    SOC_GPIO_PIN_SFL_T1000_SS,
                                                    SOC_GPIO_PIN_SFL_T1000_MOSI,
                                                    SOC_GPIO_PIN_SFL_T1000_MISO,
                                                    SOC_GPIO_PIN_SFL_T1000_WP,
                                                    SOC_GPIO_PIN_SFL_T1000_HOLD);
      break;
    case NRF52_NORDIC_PCA10059:
    case NRF52_ELECROW_TN_M3:
      // no SPI flash
    default:
      break;
  }

  if (FlashTrans != NULL) {
    FlashTrans->begin();
    FlashTrans->runCommand(0xAB); /* RDP/RES */
    FlashTrans->end();

    SPIFlash = new Adafruit_SPIFlash(FlashTrans);
    nRF52_has_spiflash = SPIFlash->begin(possible_devices,
                                         EXTERNAL_FLASH_DEVICE_COUNT);
  }
#endif /* ARDUINO_ARCH_MBED */

  hw_info.storage = nRF52_has_spiflash ? STORAGE_FLASH : STORAGE_NONE;

  if (nRF52_board == NRF52_ELECROW_TN_M1) {
#if 0
    pinMode(SOC_GPIO_PIN_M1_BUZZER, INPUT);
    int buzzer_high_impedance = digitalRead(SOC_GPIO_PIN_M1_BUZZER);
    pinMode(SOC_GPIO_PIN_M1_BUZZER, INPUT_PULLUP);
    delay(1);
    int buzzer_pullup = digitalRead(SOC_GPIO_PIN_M1_BUZZER);
    pinMode(SOC_GPIO_PIN_M1_BUZZER, INPUT);
    if (buzzer_high_impedance == LOW && buzzer_pullup == LOW)
#else
    // allow M1 to work even if buzzer got disconnected
#endif
    {
      //nRF52_board        = NRF52_ELECROW_TN_M1;
        hw_info.model      = SOFTRF_MODEL_HANDHELD;
        nRF52_Device_Model = "Handheld Edition";

      // SHUTDOWN_MAGIC already done above
      // best to do as early as possible in the boot sequence
    }
  }

  if (nRF52_board == NRF52_LILYGO_TECHO_REV_0
  ||  nRF52_board == NRF52_LILYGO_TECHO_REV_1
  ||  nRF52_board == NRF52_LILYGO_TECHO_REV_2) {
#if 1
#if !defined(ARDUINO_ARCH_MBED) && !defined(ARDUINO_ARCH_ZEPHYR)
      // detect the SHUTDOWN_MAGIC from the first shutdown (software reset)
      // and go back to sleep - this time in a lower power state.
      // Alas the T-Echo has no ext_power_pin, can only look for POWER_RESETREAS_VBUS_Msk
    if (reset_reason & POWER_RESETREAS_VBUS_Msk) {
      NRF_POWER->GPREGRET2 = CHARGE_MAGIC;
    } else {
      if ((reset_reason & POWER_RESETREAS_SREQ_Msk) && NRF_POWER->GPREGRET2 == SHUTDOWN_MAGIC) {
        if (nRF52_board == NRF52_LILYGO_TECHO_REV_1)
           pinMode(SOC_GPIO_PIN_BUTTON, INPUT_PULLUP);
        else
            pinMode(SOC_GPIO_PIN_BUTTON, INPUT);
        delay(100);
        while (digitalRead(SOC_GPIO_PIN_BUTTON) == LOW) {
            delay(100);
            nRF52_WDT_fini();
        }
        if (nRF52_board == NRF52_LILYGO_TECHO_REV_1)
           pinMode(SOC_GPIO_PIN_BUTTON, INPUT_PULLUP_SENSE);
        else
            pinMode(SOC_GPIO_PIN_BUTTON, INPUT_SENSE_LOW);
        pinMode(SOC_GPIO_PIN_IO_PWR, INPUT);
        NRF_GPIO->LATCH = 0xFFFFFFFF;
        NRF_POWER->GPREGRET = DFU_MAGIC_SKIP;
        NRF_POWER->GPREGRET2 = SLEEP_MAGIC;
        nRF52_system_off(REASON_DEFAULT_RST);
      }
      NRF_POWER->GPREGRET2 = AWAKE_MAGIC;
    }
#endif /* ARDUINO_ARCH_MBED */
#endif
  }

  if (nRF52_board == NRF52_UNSUPPORTED) {
      pinMode(SOC_GPIO_PIN_IO_PWR, INPUT);
      //while(1)
      //    delay(1000);
      nRF52_system_off(REASON_DEFAULT_RST);
  }

  ext_power_pin = SOC_GPIO_PIN_EXT_PWR;    // T-Echo has no ext_power_pin

#if !defined(ARDUINO_ARCH_MBED) && !defined(ARDUINO_ARCH_ZEPHYR)
  USBDevice.setID(nRF52_USB_VID, nRF52_USB_PID);
  USBDevice.setManufacturerDescriptor(nRF52_Device_Manufacturer);
  USBDevice.setProductDescriptor(nRF52_Device_Model);
  USBDevice.setDeviceVersion(nRF52_Device_Version);
#endif /* ARDUINO_ARCH_MBED */

nRF52_WDT_fini();

  /* GPIO pins init */
  switch (nRF52_board)
  {
    case NRF52_LILYGO_TECHO_REV_0:
      /* Wake up Air530 GNSS */
      digitalWrite(SOC_GPIO_PIN_GNSS_WKE, HIGH);
      pinMode(SOC_GPIO_PIN_GNSS_WKE, OUTPUT);

      pinMode(SOC_GPIO_LED_TECHO_REV_0_RED,   OUTPUT);   // red on while booting
      pinMode(SOC_GPIO_LED_TECHO_REV_0_GREEN, OUTPUT);
      pinMode(SOC_GPIO_LED_TECHO_REV_0_BLUE,  OUTPUT);

      ledOn (SOC_GPIO_LED_TECHO_REV_0_RED);
      ledOff(SOC_GPIO_LED_TECHO_REV_0_GREEN);
      ledOff(SOC_GPIO_LED_TECHO_REV_0_BLUE);

      lmic_pins.rst = SOC_GPIO_PIN_TECHO_REV_0_RST;
      lmic_pins.dio[0] = SOC_GPIO_PIN_TECHO_REV_0_DIO0;
      lmic_pins.dio[1] = SOC_GPIO_PIN_DIO1; /* for sx1262 */

      hw_info.revision = 0;
      hw_info.touch    = TOUCH_TTP223;
      break;

    case NRF52_LILYGO_TECHO_REV_1:
      /* Wake up Air530 GNSS */
      digitalWrite(SOC_GPIO_PIN_GNSS_WKE, HIGH);
      pinMode(SOC_GPIO_PIN_GNSS_WKE, OUTPUT);

      pinMode(SOC_GPIO_LED_TECHO_REV_1_RED,   OUTPUT);
      pinMode(SOC_GPIO_LED_TECHO_REV_1_GREEN, OUTPUT);
      pinMode(SOC_GPIO_LED_TECHO_REV_1_BLUE,  OUTPUT);

      ledOn (SOC_GPIO_LED_TECHO_REV_1_RED);
      ledOff(SOC_GPIO_LED_TECHO_REV_1_GREEN);
      ledOff(SOC_GPIO_LED_TECHO_REV_1_BLUE);

      lmic_pins.rst = SOC_GPIO_PIN_TECHO_REV_1_RST;
      lmic_pins.dio[0] = SOC_GPIO_PIN_TECHO_REV_1_DIO0;
      lmic_pins.dio[1] = SOC_GPIO_PIN_DIO1; /* for sx1262 */

      hw_info.revision = 1;
      hw_info.touch    = TOUCH_TTP223;
      break;

    case NRF52_LILYGO_TECHO_REV_2:
    case NRF52_LILYGO_TECHO_PLUS:
      /* Wake up Quectel L76K GNSS */
      digitalWrite(SOC_GPIO_PIN_GNSS_RST, HIGH);
      pinMode(SOC_GPIO_PIN_GNSS_RST, OUTPUT);
      digitalWrite(SOC_GPIO_PIN_GNSS_WKE, HIGH);
      pinMode(SOC_GPIO_PIN_GNSS_WKE, OUTPUT);

      pinMode(SOC_GPIO_LED_TECHO_REV_2_RED,   OUTPUT);
      pinMode(SOC_GPIO_LED_TECHO_REV_2_GREEN, OUTPUT);
      pinMode(SOC_GPIO_LED_TECHO_REV_2_BLUE,  OUTPUT);

      ledOn (SOC_GPIO_LED_TECHO_REV_2_RED);
      ledOff(SOC_GPIO_LED_TECHO_REV_2_GREEN);
      ledOff(SOC_GPIO_LED_TECHO_REV_2_BLUE);

      lmic_pins.rst = SOC_GPIO_PIN_TECHO_REV_2_RST;
      lmic_pins.dio[0] = SOC_GPIO_PIN_TECHO_REV_2_DIO0;
      lmic_pins.dio[1] = SOC_GPIO_PIN_DIO1; /* for sx1262 */

      hw_info.revision = 2;
      hw_info.touch    = TOUCH_TTP223;
      break;

    case NRF52_SEEED_T1000E:

      digitalWrite(SOC_GPIO_PIN_T1000_3V3_EN, HIGH);
      pinMode(SOC_GPIO_PIN_T1000_3V3_EN, OUTPUT);

      digitalWrite(SOC_GPIO_PIN_T1000_BUZZER_EN, HIGH);
      pinMode(SOC_GPIO_PIN_T1000_BUZZER_EN, OUTPUT);

      digitalWrite(SOC_GPIO_PIN_GNSS_T1000_EN, HIGH);
      pinMode(SOC_GPIO_PIN_GNSS_T1000_EN, OUTPUT);

      digitalWrite(SOC_GPIO_PIN_GNSS_T1000_VRTC, HIGH);
      pinMode(SOC_GPIO_PIN_GNSS_T1000_VRTC, OUTPUT);

      digitalWrite(SOC_GPIO_PIN_GNSS_T1000_RST, LOW);
      pinMode(SOC_GPIO_PIN_GNSS_T1000_RST, OUTPUT);

      digitalWrite(SOC_GPIO_PIN_GNSS_T1000_SINT, HIGH);
      pinMode(SOC_GPIO_PIN_GNSS_T1000_SINT, OUTPUT);

      digitalWrite(SOC_GPIO_PIN_GNSS_T1000_RINT, LOW);
      pinMode(SOC_GPIO_PIN_GNSS_T1000_RINT, OUTPUT);

      digitalWrite(SOC_GPIO_LED_T1000_GREEN, LOW);
      pinMode(SOC_GPIO_LED_T1000_GREEN, OUTPUT);
      digitalWrite(SOC_GPIO_LED_T1000_RED, HIGH);    // on while booting
      pinMode(SOC_GPIO_LED_T1000_RED, OUTPUT);

      lmic_pins.nss  = SOC_GPIO_PIN_T1000_SS;
      lmic_pins.rst  = SOC_GPIO_PIN_T1000_RST;
      lmic_pins.busy = SOC_GPIO_PIN_T1000_BUSY;
      lmic_pins.dio[0] = SOC_GPIO_PIN_T1000_DIO9; /* LR1110 */

      hw_info.revision = 3; /* Unknown */
      hw_info.audio    = AUDIO_PWM;
      break;

    case NRF52_ELECROW_TN_M1:

      /* TBD */
      // digitalWrite(SOC_GPIO_PIN_M1_DIO3, HIGH);
      // pinMode(SOC_GPIO_PIN_M1_DIO3, OUTPUT);

      /* Wake up Quectel L76K GNSS */
      digitalWrite(SOC_GPIO_PIN_GNSS_M1_RST, HIGH);
      pinMode(SOC_GPIO_PIN_GNSS_M1_RST, OUTPUT);
      digitalWrite(SOC_GPIO_PIN_GNSS_M1_WKE, HIGH);
      pinMode(SOC_GPIO_PIN_GNSS_M1_WKE, OUTPUT);

      digitalWrite(SOC_GPIO_LED_M1_RED_PWR, HIGH);
      pinMode(SOC_GPIO_LED_M1_RED_PWR,      OUTPUT);

      digitalWrite(SOC_GPIO_LED_M1_RED,  LOW);    // on while booting
      pinMode(SOC_GPIO_LED_M1_RED,  OUTPUT);
      digitalWrite(SOC_GPIO_LED_M1_BLUE, HIGH);
      pinMode(SOC_GPIO_LED_M1_BLUE, OUTPUT);

      lmic_pins.dio[1] = SOC_GPIO_PIN_M1_DIO1;    /* for sx1262 */
      lmic_pins.nss  = SOC_GPIO_PIN_M1_SS;
      lmic_pins.rst  = SOC_GPIO_PIN_M1_RST;
      lmic_pins.busy = SOC_GPIO_PIN_M1_BUSY;

      hw_info.revision = 3; /* Unknown */
      hw_info.audio    = AUDIO_PWM;
      break;

    case NRF52_ELECROW_TN_M3:

      digitalWrite(SOC_GPIO_PIN_M3_ADC_EN, HIGH);
      pinMode(SOC_GPIO_PIN_M3_ADC_EN, OUTPUT);

      digitalWrite(SOC_GPIO_PIN_M3_EN1, HIGH);
      pinMode(SOC_GPIO_PIN_M3_EN1, OUTPUT);
      digitalWrite(SOC_GPIO_PIN_M3_EN2, HIGH);
      pinMode(SOC_GPIO_PIN_M3_EN2, OUTPUT);

      pinMode(SOC_GPIO_PIN_M3_BUT_EN, INPUT_PULLUP);
      digitalWrite(SOC_GPIO_PIN_GNSS_M3_EN, HIGH);
      pinMode(SOC_GPIO_PIN_GNSS_M3_EN, OUTPUT);
      digitalWrite(SOC_GPIO_PIN_GNSS_M3_RST, LOW);
      pinMode(SOC_GPIO_PIN_GNSS_M3_RST, OUTPUT);
      digitalWrite(SOC_GPIO_PIN_GNSS_M3_WKE, HIGH);
      pinMode(SOC_GPIO_PIN_GNSS_M3_WKE, OUTPUT);

      digitalWrite(SOC_GPIO_LED_M3_RGB_PWR, HIGH);
      pinMode(SOC_GPIO_LED_M3_RGB_PWR,      OUTPUT);

      pinMode(SOC_GPIO_LED_M3_RED,   OUTPUT);
      pinMode(SOC_GPIO_LED_M3_GREEN, OUTPUT);
      pinMode(SOC_GPIO_LED_M3_BLUE,  OUTPUT);

      digitalWrite(SOC_GPIO_LED_M3_RED,   LOW);    // on while booting
      digitalWrite(SOC_GPIO_LED_M3_GREEN, HIGH);
      digitalWrite(SOC_GPIO_LED_M3_BLUE,  HIGH);

      lmic_pins.nss  = SOC_GPIO_PIN_M3_SS;
      lmic_pins.rst  = SOC_GPIO_PIN_M3_RST;
      lmic_pins.busy = SOC_GPIO_PIN_M3_BUSY;
      lmic_pins.dio[0] = SOC_GPIO_PIN_M3_DIO9; /* LR1110 */

      hw_info.revision = 3; /* Unknown */
      hw_info.audio    = AUDIO_PWM;
      break;

    case NRF52_NORDIC_PCA10059:
    default:
      pinMode(SOC_GPIO_LED_PCA10059_STATUS, OUTPUT);
      pinMode(SOC_GPIO_LED_PCA10059_GREEN,  OUTPUT);
      pinMode(SOC_GPIO_LED_PCA10059_RED,    OUTPUT);
      pinMode(SOC_GPIO_LED_PCA10059_BLUE,   OUTPUT);

      ledOn (SOC_GPIO_LED_PCA10059_RED);
      ledOff(SOC_GPIO_LED_PCA10059_GREEN);
      ledOff(SOC_GPIO_LED_PCA10059_BLUE);
      ledOff(SOC_GPIO_LED_PCA10059_STATUS);

      hw_info.revision = 3; /* Unknown */
      break;
  }

#if !defined(ARDUINO_ARCH_MBED) && !defined(ARDUINO_ARCH_ZEPHYR)
  i2c = new I2CBus(Wire);

  if (nRF52_has_rtc && (i2c != nullptr)) {
    rtc = new PCF8563_Class(*i2c);

    switch (nRF52_board)
    {
      case NRF52_ELECROW_TN_M3:
        // pinMode(SOC_GPIO_PIN_RTC_M3_INT, INPUT);   // unused?
        break;
      case NRF52_ELECROW_TN_M1:
        pinMode(SOC_GPIO_PIN_RTC_M1_INT, INPUT);  // actually same as SOC_GPIO_PIN_R_INT
        break;
      //case NRF52_LILYGO_TECHO_REV_0:
      //case NRF52_LILYGO_TECHO_REV_1:
      //case NRF52_LILYGO_TECHO_REV_2:
      //case NRF52_LILYGO_TECHO_PLUS:
      //case NRF52_NORDIC_PCA10059:
      default:
        pinMode(SOC_GPIO_PIN_R_INT, INPUT);
        break;
    }

    hw_info.rtc = RTC_PCF8563;
  }
#endif /* ARDUINO_ARCH_MBED */

nRF52_WDT_fini();

#if defined(USE_TINYUSB)
  switch (nRF52_board)
  {
    case NRF52_SEEED_T1000E:
      Serial1.setPins(SOC_GPIO_PIN_CONS_T1000_RX, SOC_GPIO_PIN_CONS_T1000_TX);
#if defined(EXCLUDE_WIFI)
      Serial1.begin(SERIAL_OUT_BR, SERIAL_OUT_BITS);
#endif /* EXCLUDE_WIFI */
      break;
    case NRF52_ELECROW_TN_M3:
      Serial1.setPins(SOC_GPIO_PIN_CONS_M3_RX, SOC_GPIO_PIN_CONS_M3_TX);
#if defined(EXCLUDE_WIFI)
      Serial1.begin(SERIAL_OUT_BR, SERIAL_OUT_BITS);
#endif /* EXCLUDE_WIFI */
      break;
    case NRF52_ELECROW_TN_M1:
    case NRF52_LILYGO_TECHO_PLUS:
      Serial1.setPins(SOC_GPIO_PIN_CONS_M1_RX, SOC_GPIO_PIN_CONS_M1_TX);
#if defined(EXCLUDE_WIFI)
      Serial1.begin(SERIAL_OUT_BR, SERIAL_OUT_BITS);
#endif /* EXCLUDE_WIFI */
      break;
    case NRF52_LILYGO_TECHO_REV_0:
    case NRF52_LILYGO_TECHO_REV_1:
    case NRF52_LILYGO_TECHO_REV_2:
    case NRF52_NORDIC_PCA10059:
    default:
      Serial1.setPins(SOC_GPIO_PIN_CONS_RX, SOC_GPIO_PIN_CONS_TX);
#if defined(EXCLUDE_WIFI)
      Serial1.begin(SERIAL_OUT_BR, SERIAL_OUT_BITS);
#endif /* EXCLUDE_WIFI */
      break;
  }
#endif /* USE_TINYUSB */

nRF52_WDT_fini();

#if !defined(EXCLUDE_IMU)
  if (nRF52_has_imu) {
    switch (nRF52_board)
    {
      case NRF52_LILYGO_TECHO_REV_2:
      case NRF52_LILYGO_TECHO_PLUS:
        Wire.begin();

        if (imu_1.setup(MPU9250_ADDRESS)) {
          imu_1.verbose(false);
          if (imu_1.isSleeping()) {
            imu_1.sleep(false);
          }
          hw_info.imu = IMU_MPU9250;
          hw_info.mag = MAG_AK8963;
          IMU_Time_Marker = millis();
#if !defined(EXCLUDE_BHI260)
        } else if (
#if defined(USE_BHI260_RAM_FW)
                   imu_4.setFirmware(bosch_firmware_image,
                                     bosch_firmware_size,
                                     bosch_firmware_type),
#endif /* USE_BHI260_RAM_FW */
                   imu_4.begin(Wire, BHI260AP_ADDRESS_L,
                                     SOC_GPIO_PIN_SDA, SOC_GPIO_PIN_SCL)) {
          float sample_rate = 12.5;
          uint32_t report_latency_ms = 0; /* Report immediately */

          // Enable acceleration
          bhi_accel.enable(sample_rate, report_latency_ms);
          // Enable gyroscope
          // bhi_gyro.enable(sample_rate, report_latency_ms);

          hw_info.imu = IMU_BHI260AP;
          IMU_Time_Marker = millis();
#endif /* EXCLUDE_BHI260 */
        } else {
          bool ad0 = (ICM20948_ADDRESS == 0x69) ? true : false;

          for (int t=0; t<3; t++) {
            if (imu_2.begin(Wire, ad0) == ICM_20948_Stat_Ok) {
              hw_info.imu = IMU_ICM20948;
              hw_info.mag = MAG_AK09916;
              IMU_Time_Marker = millis();

              break;
            }
            delay(IMU_UPDATE_INTERVAL);
          }
        }
        break;

      case NRF52_SEEED_T1000E:
        Wire.begin();

        if (imu_3.begin()) {
          imu_3.softwareReset();
          delay(5);
          imu_3.setRange(SFE_QMA6100P_RANGE4G);
          imu_3.enableAccel(true);
          imu_3.calibrateOffsets();
          // imu_3.setOffset();

          hw_info.imu     = ACC_QMA6100P;
          IMU_Time_Marker = millis();
        }
        break;

      case NRF52_ELECROW_TN_M3:
        /* TBD */
        hw_info.imu     = ACC_SC7A20H;
        IMU_Time_Marker = millis();
        break;

      default:
        break;
    }
  }
#endif /* EXCLUDE_IMU */

nRF52_WDT_fini();

#if !defined(ARDUINO_ARCH_MBED) && !defined(ARDUINO_ARCH_ZEPHYR)
  if (nRF52_has_spiflash) {
    spiflash_id = SPIFlash->getJEDECID();

    //mx25_status_config[0] = SPIFlash->readStatus();
    //HWFlashTransport.readCommand(SFLASH_CMD_READ_CONFIG,   mx25_status_config + 1, 2);
    //mx25_status_config[2] |= 0x2;       /* High performance mode */
    //SPIFlash->writeEnable();
    //HWFlashTransport.writeCommand(SFLASH_CMD_WRITE_STATUS, mx25_status_config,     3);
    //SPIFlash->writeDisable();
    //SPIFlash->waitUntilReady();

    //uint32_t const wr_speed = min(80 * 1000000U, (uint32_t)F_CPU);
    //uint32_t rd_speed = wr_speed;
    //HWFlashTransport.setClockSpeed(wr_speed, rd_speed);

    // Set disk vendor id, product id and revision with string up to 8, 16, 4 characters respectively
    usb_msc.setID(nRF52_Device_Manufacturer, "External Flash", "1.0");

    // Set callback
    usb_msc.setReadWriteCallback(nRF52_msc_read_cb,
                                 nRF52_msc_write_cb,
                                 nRF52_msc_flush_cb);

    // Set disk size, block size should be 512 regardless of spi flash page size
    usb_msc.setCapacity(SPIFlash->size()/512, 512);

    // MSC is ready for read/write
    usb_msc.setUnitReady(true);

    usb_msc.begin();

    FATFS_is_mounted = fatfs.begin(SPIFlash);
  }
#endif /* ARDUINO_ARCH_MBED */

nRF52_WDT_fini();

#if defined(USE_USB_MIDI)
  // Initialize MIDI with no any input channels
  // This will also call usb_midi's begin()
  MIDI_USB.begin(MIDI_CHANNEL_OFF);
#endif /* USE_USB_MIDI */

  Serial.begin(SERIAL_OUT_BR, SERIAL_OUT_BITS);

#if defined(USE_TINYUSB) && defined(USBCON)
  for (int i=0; i < 20; i++) {if (Serial) break; else delay(100);}
#endif

  if (nRF52_board == NRF52_LILYGO_TECHO_PLUS) {
    nRF52_has_vibra = vibra.begin(Wire, DRV2605_ADDRESS);

    if (nRF52_has_vibra) {
      vibra.selectLibrary(1);
#if SENSORLIB_VERSION == SENSORLIB_VERSION_VAL(0, 3, 1)
      vibra.setMode(SensorDRV2605::MODE_INTTRIG);
#endif /* (0, 3, 1) */
#if SENSORLIB_VERSION >= SENSORLIB_VERSION_VAL(0, 4, 1)
      vibra.setMode(HapticMode::INTERNAL_TRIGGER);
#endif /* (0, 4, 1) */

      digitalWrite(SOC_GPIO_PIN_MOTOR_EN, HIGH);
      pinMode(SOC_GPIO_PIN_MOTOR_EN, OUTPUT);
      hw_info.haptic = HAPTIC_DRV2605;
    }
    hw_info.audio = AUDIO_PWM;
  }

#if defined(USE_PWM_SOUND)
  buzzerpin = SOC_GPIO_PIN_BUZZER;
#endif

#if 0
  // turn on LED briefly to show an early sign of life
  // - instead, we turned on an LED as soon as the board was identified
  // - and leave it on until it changes color once booting is complete
  int status_LED = SOC_GPIO_PIN_STATUS;
  if (status_LED != SOC_UNUSED_PIN) {
      pinMode(status_LED, OUTPUT);
      digitalWrite(status_LED, LED_STATE_ON);
      delay(500);
      digitalWrite(status_LED, 1-LED_STATE_ON);
      pinMode(status_LED, INPUT);
  }
#endif

nRF52_WDT_fini();

}     // end of setup()

static void print_dest(int dest)
{
  switch (dest)
  {
    case DEST_UART       :  Serial.println(F("UART"));          break;
    case DEST_USB        :  Serial.println(F("USB CDC"));       break;
    case DEST_BLUETOOTH  :  Serial.println(F("Bluetooth LE"));  break;
    case DEST_NONE       :
    default              :  Serial.println(F("NULL"));          break;
  }
}

static void nRF52_post_init()
{
#if 0
  uint32_t variant = NRF_FICR->INFO.VARIANT;
  char bc_high     = (variant >> 8) & 0xFF;
  char bc_low      = (variant     ) & 0xFF;
  const char *rev  = "UNKNOWN";

  switch (bc_high)
  {
    case 'A': rev = "Engineering A"; break;
    case 'B': rev = "Engineering B"; break;
    case 'C':
      if (bc_low == 'A')
              rev = "Engineering C";
      else
              rev = "1";
      break;
    case 'D':
      if (bc_low == 'A')
              rev = "Engineering D";
      else
              rev = "2";
      break;
    case 'F': rev = "3";             break;
    default:                         break;
  }

  Serial.println();
  Serial.print  (F("MCU Revision: "));
  Serial.println(rev);
#endif

  if (nRF52_board == NRF52_LILYGO_TECHO_REV_0 ||
      nRF52_board == NRF52_LILYGO_TECHO_REV_1 ||
      nRF52_board == NRF52_LILYGO_TECHO_REV_2 ||
      nRF52_board == NRF52_LILYGO_TECHO_PLUS  ||
      nRF52_board == NRF52_ELECROW_TN_M1) {

      if (nRF52_board == NRF52_ELECROW_TN_M1)
          digitalWrite(SOC_GPIO_LED_M1_RED, HIGH);    // LED_STATE_ON = LOW

#if 0
    char strbuf[32];
    Serial.println();
    Serial.print  (F("64-bit Device ID: "));
    snprintf(strbuf, sizeof(strbuf),"0x%08x%08x", DEVICE_ID_HIGH, DEVICE_ID_LOW);
    Serial.println(strbuf);
#endif

#if 0
    Serial.println();
    Serial.print  (F("SPI FLASH JEDEC ID: "));
    Serial.print  (spiflash_id, HEX);           Serial.print(" ");
    Serial.print  (F("STATUS/CONFIG: "));
    Serial.print  (mx25_status_config[0], HEX); Serial.print(" ");
    Serial.print  (mx25_status_config[1], HEX); Serial.print(" ");
    Serial.print  (mx25_status_config[2], HEX); Serial.println();
#endif

    Serial.println();
    Serial.print  (nRF52_board == NRF52_ELECROW_TN_M1  ? F("Elecrow ") : F("LilyGO T-"));
    Serial.print  (nRF52_board == NRF52_ELECROW_TN_M1  ? F("TN-M1")  :   F("Echo"));
    Serial.print  (F(" ("));
    Serial.print  (hw_info.revision > 2 ?
                   Hardware_Rev[3] : Hardware_Rev[hw_info.revision]);
    Serial.println(F(") Power-on Self Test"));
    Serial.println();
    Serial.flush();

    Serial.println(F("Built-in components:"));

    Serial.print(F("RADIO   : "));
    Serial.println(hw_info.rf      == RF_IC_SX1262 ||
                   hw_info.rf      == RF_IC_SX1276 ||
                   hw_info.rf      == RF_IC_LR1110     ? F("PASS") : F("FAIL"));
    Serial.flush();
    Serial.print(F("GNSS    : "));
    if (nRF52_board == NRF52_LILYGO_TECHO_REV_0 ||
        nRF52_board == NRF52_LILYGO_TECHO_REV_1) {
      Serial.println(hw_info.gnss  == GNSS_MODULE_GOKE ? F("PASS") : F("FAIL"));
    } else {
      Serial.println(hw_info.gnss  == GNSS_MODULE_AT65 ? F("PASS") : F("FAIL"));
    }

    Serial.flush();
    Serial.print(F("DISPLAY : "));
    Serial.println(hw_info.display == DISPLAY_EPD_1_54 ? F("PASS") : F("FAIL"));
    Serial.flush();
    Serial.print(F("RTC     : "));
    Serial.println(hw_info.rtc     == RTC_PCF8563      ? F("PASS") : F("FAIL"));
    Serial.flush();
    Serial.print(F("FLASH   : "));
    Serial.println(hw_info.storage == STORAGE_FLASH_AND_CARD ||
                   hw_info.storage == STORAGE_FLASH    ? F("PASS") : F("FAIL"));
    Serial.flush();

    if (nRF52_board == NRF52_LILYGO_TECHO_REV_1 ||
        nRF52_board == NRF52_LILYGO_TECHO_REV_2 ||
        nRF52_board == NRF52_LILYGO_TECHO_PLUS) {
      Serial.print(F("BMx280  : "));
      Serial.println(hw_info.baro == BARO_MODULE_BMP280 ? F("PASS") : F("N/A"));
      Serial.flush();
    }

#if !defined(EXCLUDE_IMU)
    {
      Serial.println();
      Serial.println(F("External components:"));
      Serial.print(F("IMU     : "));
      Serial.println(hw_info.imu   == IMU_MPU9250  ||
                     hw_info.imu   == IMU_ICM20948 ||
                     hw_info.imu   == IMU_BHI260AP     ? F("PASS") : F("N/A"));
    }
    Serial.flush();
#endif /* EXCLUDE_IMU */

#if 0
    if (nRF52_board == NRF52_LILYGO_TECHO_PLUS) {
      BoschSensorInfo info = imu_4.getSensorInfo();
      info.printInfo(Serial);
      Serial.flush();
    }
#endif

  } else if (nRF52_board == NRF52_SEEED_T1000E) {

    digitalWrite(SOC_GPIO_LED_T1000_RED, LOW);   // LED_STATE_ON = HIGH

#if 0
    Serial.println();
    Serial.print  (F("SPI FLASH JEDEC ID: "));
    Serial.print  (spiflash_id, HEX);           Serial.print(" ");
#endif

    Serial.println();
    Serial.println(F("Seeed T1000-E Power-on Self Test"));
    Serial.println();
    Serial.flush();

    Serial.println(F("Built-in components:"));

    Serial.print(F("RADIO   : "));
    Serial.println(hw_info.rf    == RF_IC_LR1110     ? F("PASS") : F("FAIL"));
    Serial.flush();
    Serial.print(F("GNSS    : "));
    Serial.println(hw_info.gnss  == GNSS_MODULE_AG33 ? F("PASS") : F("FAIL"));
    Serial.flush();
    Serial.print(F("FLASH   : "));
    Serial.println(hw_info.storage == STORAGE_FLASH  ? F("PASS") : F("FAIL"));
    Serial.flush();

#if !defined(EXCLUDE_IMU)
    Serial.print(F("IMU     : "));
    Serial.println(hw_info.imu   == ACC_QMA6100P     ? F("PASS") : F("FAIL"));
    Serial.flush();
#endif /* EXCLUDE_IMU */

#if !defined(ARDUINO_ARCH_MBED) && !defined(ARDUINO_ARCH_ZEPHYR)
    analogReference(AR_INTERNAL_3_0);
    analogReadResolution(10);
    delay(1);

    Serial.print(F("TEMP    : "));
    Serial.print((float) t1000e_ntc_sample() / 10.0f);
    Serial.println(F(" Celsius"));
    Serial.print(F("LIGHT   : "));
    Serial.print(t1000e_lux_sample());
    Serial.println(F(" %"));
    Serial.print(F("BATTERY : "));
    Serial.print(t1000e_bat_sample());
    Serial.println(F(" %"));
    Serial.flush();
#endif /* !MBED && !ZEPHYR */

  } else if (nRF52_board == NRF52_ELECROW_TN_M3) {

    digitalWrite(SOC_GPIO_LED_M3_RED, HIGH);     // LED_STATE_ON = LOW

    Serial.println();
    Serial.println(F("Elecrow ThinkNode-M3 Power-on Self Test"));
    Serial.println();
    Serial.flush();

    Serial.println(F("Built-in components:"));

    Serial.print(F("RADIO   : "));
    Serial.println(hw_info.rf    == RF_IC_LR1110     ? F("PASS") : F("FAIL"));
    Serial.flush();
    Serial.print(F("GNSS    : "));
    Serial.println(hw_info.gnss  == GNSS_MODULE_AT65 ? F("PASS") : F("FAIL"));
    Serial.flush();
    Serial.print(F("RTC     : "));
    Serial.println(hw_info.rtc   == RTC_PCF8563      ? F("PASS") : F("FAIL"));
    Serial.flush();

#if !defined(EXCLUDE_IMU)
    Serial.print(F("IMU     : "));
    Serial.println(hw_info.imu   == ACC_SC7A20H      ? F("PASS") : F("FAIL"));
    Serial.flush();
#endif /* EXCLUDE_IMU */

    aht20.begin();
    float humidity, temperature;

    if (aht20.getSensor(&humidity, &temperature)) {
      Serial.print(F("TEMP    : "));
      Serial.print(temperature);
      Serial.println(F(" degrees Celsius"));
      Serial.print(F("HUMID   : "));
      Serial.print(humidity*100);
      Serial.println(F(" % rH"));
      Serial.flush();
    }

  } else if (nRF52_board == NRF52_NORDIC_PCA10059) {
    Serial.println();
    Serial.println(F("Board: Nordic PCA10059 USB Dongle"));
    Serial.println();
    Serial.flush();
  }

  Serial.println();
  Serial.println(F("Power-on Self Test is complete."));
  Serial.println();
  Serial.flush();

  if (nRF52_board == NRF52_ELECROW_TN_M3) {
    Serial.println(F("No file system on M3\r\n"));
  } else {
    if (FATFS_is_mounted)
        Serial.printf("FATFS mounted, free space: %d kB\r\n\r\n", FILESYS_free_kb());
    else
        Serial.println(F("Failed to mount FATFS\r\n"));
  }

  Serial.println(F("Data output device(s):"));

  Serial.print(F("NMEA   - "));
  print_dest(settings->nmea_out);

  Serial.print(F("NMEA2  - "));
  print_dest(settings->nmea_out2);

  Serial.print(F("GDL90  - "));
  print_dest(settings->gdl90);

  Serial.print(F("D1090  - "));
  print_dest(settings->d1090);

  Serial.println();
  Serial.flush();

#if 1
  // >>> see what was passed from the previous shutdown
  Serial.print("reset_reason: ");
  Serial.println(reset_reason, HEX);
  Serial.print("reset_info.reason: ");
  Serial.println(SoC->getResetReason());
  Serial.print("latch: ");
  Serial.println(latch, HEX);
  Serial.print("gpregret:  ");
  Serial.println(gpregret, HEX);
  Serial.print("gpregret2 at boot: ");
  Serial.println(gpregret2, HEX);
  Serial.print("GPREGRET2 now: ");
  Serial.println(NRF_POWER->GPREGRET2, HEX);
#endif

nRF52_WDT_fini();

#if defined(USE_EPAPER)
  if (nRF52_board == NRF52_LILYGO_TECHO_REV_0 ||
      nRF52_board == NRF52_LILYGO_TECHO_REV_1 ||
      nRF52_board == NRF52_LILYGO_TECHO_REV_2 ||
      nRF52_board == NRF52_LILYGO_TECHO_PLUS  ||
      nRF52_board == NRF52_ELECROW_TN_M1) {
    /* EPD back light on */
    // as it happens, SOC_GPIO_PIN_EPD_M1_BLGT == SOC_GPIO_PIN_EPD_BLGT
    uint8_t bl_state = digitalRead(SOC_GPIO_PIN_EPD_BLGT);
    //if (nRF52_board == NRF52_ELECROW_TN_M1)
    //    digitalWrite(SOC_GPIO_PIN_EPD_BLGT, LOW);
    //else
    //    digitalWrite(SOC_GPIO_PIN_EPD_BLGT, HIGH);
    digitalWrite(SOC_GPIO_PIN_EPD_BLGT, 1-bl_state);

    EPD_info1();

    /* EPD back light off */
    digitalWrite(SOC_GPIO_PIN_EPD_BLGT, bl_state);

    char key[8];
    char out[64];
    uint8_t tokens[3] = { 0 };
    cdbResult rt;
    int c, i = 0, token_cnt = 0;

    int acfts;
    char *reg, *mam, *cn;
    reg = mam = cn = NULL;

    if (ADB_is_open) {
      acfts = ucdb.recordsNumber();

      snprintf(key, sizeof(key),"%06X", ThisAircraft.addr);

      rt = ucdb.findKey(key, strlen(key));

      switch (rt) {
        case KEY_FOUND:
          while ((c = ucdb.readValue()) != -1 && i < (sizeof(out) - 1)) {
            if (c == '|') {
              if (token_cnt < (sizeof(tokens) - 1)) {
                token_cnt++;
                tokens[token_cnt] = i+1;
              }
              c = 0;
            }
            out[i++] = (char) c;
          }
          out[i] = 0;

          reg = out + tokens[1];
          mam = out + tokens[0];
          cn  = out + tokens[2];

          break;

        case KEY_NOT_FOUND:
        default:
          break;
      }

      reg = (reg != NULL) && strlen(reg) ? reg : (char *) "REG: N/A";
      mam = (mam != NULL) && strlen(mam) ? mam : (char *) "M&M: N/A";
      cn  = (cn  != NULL) && strlen(cn)  ? cn  : (char *) "N/A";

      EPD_info2(acfts, reg, mam, cn);

    } else {
      acfts = -1;
    }
  }
#endif /* USE_EPAPER */

}

static void nRF52_loop()
{
  // Reload the watchdog
  if (nrf_wdt_started(NRF_WDT)) {
    Watchdog.reset();
  }

  if (!RTC_sync) {
    if (rtc &&
        gnss.date.isValid()                         &&
        gnss.time.isValid()                         &&
        gnss.date.year() >= fw_build_date_time.year &&
        gnss.date.year() <  fw_build_date_time.year + 15 ) {
      rtc->setDateTime(gnss.date.year(),   gnss.date.month(),
                       gnss.date.day(),    gnss.time.hour(),
                       gnss.time.minute(), gnss.time.second());
      RTC_sync = true;
    }
  }

#if !defined(EXCLUDE_IMU)
  if (hw_info.imu == IMU_MPU9250 &&
      (millis() - IMU_Time_Marker) > IMU_UPDATE_INTERVAL) {
    if (imu_1.update()) {
      float a_x = imu_1.getAccX();
      float a_y = imu_1.getAccY();
      float a_z = imu_1.getAccZ();

      IMU_g = sqrtf(a_x*a_x + a_y*a_y + a_z*a_z);
    }
    IMU_Time_Marker = millis();
  }

  if (hw_info.imu == IMU_ICM20948 &&
      (millis() - IMU_Time_Marker) > IMU_UPDATE_INTERVAL) {
    if (imu_2.dataReady()) {
      imu_2.getAGMT();

      // milli g's
      float a_x = imu_2.accX();
      float a_y = imu_2.accY();
      float a_z = imu_2.accZ();
#if 0
      Serial.print("{ACCEL: ");
      Serial.print(a_x);
      Serial.print(",");
      Serial.print(a_y);
      Serial.print(",");
      Serial.print(a_z);
      Serial.println("}");
#endif
      IMU_g = sqrtf(a_x*a_x + a_y*a_y + a_z*a_z) / 1000;
    }
    IMU_Time_Marker = millis();
  }

  if (hw_info.imu == ACC_QMA6100P &&
      (millis() - IMU_Time_Marker) > IMU_UPDATE_INTERVAL) {
    outputData data;

    imu_3.getAccelData(&data);
    imu_3.offsetValues(data.xData, data.yData, data.zData);

    float a_x = data.xData;
    float a_y = data.yData;
    float a_z = data.zData;
#if 0
    Serial.print("{ACCEL: ");
    Serial.print(a_x);
    Serial.print(",");
    Serial.print(a_y);
    Serial.print(",");
    Serial.print(a_z);
    Serial.println("}");
#endif
    IMU_g = sqrtf(a_x*a_x + a_y*a_y + a_z*a_z);
    IMU_Time_Marker = millis();
  }

#if 0 /* TODO */ // !defined(EXCLUDE_BHI260)
  if (hw_info.imu == IMU_BHI260AP &&
      (millis() - IMU_Time_Marker) > (IMU_UPDATE_INTERVAL / 10)) {
    // Update sensor fifo
    imu_4.update();

    if (bhi_accel.hasUpdated()) {
      float a_x = bhi_accel.getX();
      float a_y = bhi_accel.getY();
      float a_z = bhi_accel.getZ();
#if 0
      Serial.print("{ACCEL: ");
      Serial.print(a_x);
      Serial.print(",");
      Serial.print(a_y);
      Serial.print(",");
      Serial.print(a_z);
      Serial.println("}");
#endif
      IMU_g = sqrtf(a_x*a_x + a_y*a_y + a_z*a_z); /* TBD */
    }

    IMU_Time_Marker = millis();
  }
#endif /* EXCLUDE_BHI260 */
#endif /* EXCLUDE_IMU */

  // the beeps on first fix done via SoftRF.ino instead of here

}  // end of nRF52_loop()


static void nRF52_fini(int reason)
{
  nRF52_WDT_fini();

//if (reason != SOFTRF_SHUTDOWN_CHARGE) {

  if (nRF52_has_spiflash) {
    usb_msc.setUnitReady(false);
//  usb_msc.end(); /* N/A */
  }

  if (SPIFlash != NULL) {
    FlashTrans->runCommand(0xB9); /* DP */
    SPIFlash->end();
  }

#if !defined(EXCLUDE_IMU)
  if (hw_info.imu == IMU_MPU9250) {
    imu_1.sleep(true);
  }

  if (hw_info.imu == IMU_ICM20948) {
    imu_2.sleep(true);
    // imu_2.lowPower(true);
  }

  if (hw_info.imu == ACC_QMA6100P) {
    imu_3.enableAccel(false);
  }

#if !defined(EXCLUDE_BHI260)
  if (hw_info.imu == IMU_BHI260AP) {
    /* TBD */
    // imu_4.deinit();
  }
#endif /* EXCLUDE_BHI260 */
#endif /* EXCLUDE_IMU */

//}  // end of if (reason != SOFTRF_SHUTDOWN_CHARGE)

  switch (nRF52_board)
  {
    case NRF52_LILYGO_TECHO_REV_0:
#if 0
      /* Air530 GNSS ultra-low power tracking mode */
      digitalWrite(SOC_GPIO_PIN_GNSS_WKE, LOW);
      pinMode(SOC_GPIO_PIN_GNSS_WKE, OUTPUT);

    // Serial_GNSS_Out.write("$PGKC105,4*33\r\n");
#else
      pinMode(SOC_GPIO_PIN_GNSS_WKE, INPUT);

      // Serial_GNSS_Out.write("$PGKC051,0*37\r\n");
      // Serial_GNSS_Out.write("$PGKC051,1*36\r\n");
#endif
      // Serial_GNSS_Out.flush(); delay(250);

      ledOff(SOC_GPIO_LED_TECHO_REV_0_GREEN);
      ledOff(SOC_GPIO_LED_TECHO_REV_0_RED);
      ledOff(SOC_GPIO_LED_TECHO_REV_0_BLUE);

      pinMode(SOC_GPIO_LED_TECHO_REV_0_GREEN, INPUT);
      pinMode(SOC_GPIO_LED_TECHO_REV_0_RED,   INPUT);
      pinMode(SOC_GPIO_LED_TECHO_REV_0_BLUE,  INPUT);

      pinMode(SOC_GPIO_PIN_IO_PWR, INPUT);
      pinMode(SOC_GPIO_PIN_SFL_SS, INPUT);
      break;

    case NRF52_LILYGO_TECHO_REV_1:
#if 0
      /* Air530 GNSS ultra-low power tracking mode */
      digitalWrite(SOC_GPIO_PIN_GNSS_WKE, LOW);
      pinMode(SOC_GPIO_PIN_GNSS_WKE, OUTPUT);

    // Serial_GNSS_Out.write("$PGKC105,4*33\r\n");
#else
      pinMode(SOC_GPIO_PIN_GNSS_WKE, INPUT);

      // Serial_GNSS_Out.write("$PGKC051,0*37\r\n");
      // Serial_GNSS_Out.write("$PGKC051,1*36\r\n");
#endif
      // Serial_GNSS_Out.flush(); delay(250);

      ledOff(SOC_GPIO_LED_TECHO_REV_1_GREEN);
      ledOff(SOC_GPIO_LED_TECHO_REV_1_RED);
      ledOff(SOC_GPIO_LED_TECHO_REV_1_BLUE);

      pinMode(SOC_GPIO_LED_TECHO_REV_1_GREEN, INPUT);
      pinMode(SOC_GPIO_LED_TECHO_REV_1_RED,   INPUT);
      pinMode(SOC_GPIO_LED_TECHO_REV_1_BLUE,  INPUT);

      pinMode(SOC_GPIO_PIN_SFL_WP,    INPUT);
      pinMode(SOC_GPIO_PIN_SFL_HOLD,  INPUT);
      pinMode(SOC_GPIO_PIN_SFL_SS,    INPUT);
      pinMode(SOC_GPIO_PIN_IO_PWR,    INPUT);
      /* Cut 3.3V power off on modded REV_1 board */
      pinMode(SOC_GPIO_PIN_TECHO_REV_1_3V3_PWR, INPUT_PULLDOWN);
      break;

    case NRF52_LILYGO_TECHO_REV_2:
    case NRF52_LILYGO_TECHO_PLUS:
      digitalWrite(SOC_GPIO_PIN_GNSS_WKE, LOW);

      ledOff(SOC_GPIO_LED_TECHO_REV_2_GREEN);
      ledOff(SOC_GPIO_LED_TECHO_REV_2_RED);
      ledOff(SOC_GPIO_LED_TECHO_REV_2_BLUE);

      pinMode(SOC_GPIO_LED_TECHO_REV_2_GREEN, INPUT_PULLUP);
      pinMode(SOC_GPIO_LED_TECHO_REV_2_RED,   INPUT_PULLUP);
      pinMode(SOC_GPIO_LED_TECHO_REV_2_BLUE,  INPUT_PULLUP);

      if (nRF52_board == NRF52_LILYGO_TECHO_PLUS && nRF52_has_vibra == true) {
        vibra.stop();
#if SENSORLIB_VERSION == SENSORLIB_VERSION_VAL(0, 3, 1)
      vibra.setMode(1<<6); /* Standby */
#endif /* (0, 3, 1) */
#if SENSORLIB_VERSION >= SENSORLIB_VERSION_VAL(0, 4, 1)
      vibra.setMode(HapticMode::STANDBY);
#endif /* (0, 4, 1) */

        pinMode(SOC_GPIO_PIN_MOTOR_EN, INPUT);
      }

      pinMode(SOC_GPIO_PIN_SFL_HOLD,  INPUT);
      pinMode(SOC_GPIO_PIN_SFL_WP,    INPUT);
      pinMode(SOC_GPIO_PIN_SFL_SS,    INPUT);
      pinMode(SOC_GPIO_PIN_GNSS_RST,  INPUT);
      pinMode(SOC_GPIO_PIN_IO_PWR,    INPUT);
      /* Cut 3.3V power off on REV_2 board */
      // pinMode(SOC_GPIO_PIN_3V3_PWR,   INPUT_PULLDOWN);
      break;

    case NRF52_SEEED_T1000E:

// >>> high-pitch tone to signal we got this far
//tone(buzzerpin, 3640, 250); delay(450);

#if 1  // Vlad's code:
      /* Drive all power-enable pins LOW before switching to INPUT.
         OUTPUT LOW clears the output latch AND actively shuts off
         the external regulator before we release the pin. */
      digitalWrite(SOC_GPIO_PIN_GNSS_T1000_EN,   LOW);
      pinMode(SOC_GPIO_PIN_GNSS_T1000_EN,   OUTPUT);
      digitalWrite(SOC_GPIO_PIN_GNSS_T1000_VRTC, LOW);
      pinMode(SOC_GPIO_PIN_GNSS_T1000_VRTC, OUTPUT);
      digitalWrite(SOC_GPIO_PIN_T1000_3V3_EN,    LOW);
      pinMode(SOC_GPIO_PIN_T1000_3V3_EN,    OUTPUT);
      digitalWrite(SOC_GPIO_PIN_T1000_BUZZER_EN, LOW);
      pinMode(SOC_GPIO_PIN_T1000_BUZZER_EN, OUTPUT);
      digitalWrite(SOC_GPIO_PIN_SFL_T1000_EN,    LOW);
      pinMode(SOC_GPIO_PIN_SFL_T1000_EN,    OUTPUT);
#if !defined(EXCLUDE_IMU)
      digitalWrite(SOC_GPIO_PIN_T1000_ACC_EN,    LOW);
      pinMode(SOC_GPIO_PIN_T1000_ACC_EN,    OUTPUT);
#endif /* EXCLUDE_IMU */
      delay(20);

      /* Now release to INPUT_PULLDOWN for minimum SYSTEMOFF current */
      pinMode(SOC_GPIO_PIN_GNSS_T1000_EN,   INPUT_PULLDOWN);
      pinMode(SOC_GPIO_PIN_GNSS_T1000_VRTC, INPUT_PULLDOWN);
      pinMode(SOC_GPIO_PIN_T1000_3V3_EN,    INPUT_PULLDOWN);
      pinMode(SOC_GPIO_PIN_T1000_BUZZER_EN, INPUT_PULLDOWN);
      pinMode(SOC_GPIO_PIN_SFL_T1000_EN,    INPUT_PULLDOWN);
#if !defined(EXCLUDE_IMU)
      pinMode(SOC_GPIO_PIN_T1000_ACC_EN,    INPUT_PULLDOWN);
#endif /* EXCLUDE_IMU */

      /* GNSS control pins */
      pinMode(SOC_GPIO_PIN_GNSS_T1000_RINT, INPUT_PULLDOWN);
      pinMode(SOC_GPIO_PIN_GNSS_T1000_SINT, INPUT_PULLDOWN);
      pinMode(SOC_GPIO_PIN_GNSS_T1000_RST,  INPUT_PULLDOWN);

      /* LR1110 NSS must stay high to prevent spurious wakeup */
      pinMode(SOC_GPIO_PIN_T1000_SS,        INPUT_PULLUP);

      /* LR1110 SPI pins - set to INPUT to prevent current leakage */
      pinMode(SOC_GPIO_PIN_T1000_MOSI,    INPUT);
      pinMode(SOC_GPIO_PIN_T1000_MISO,    INPUT);
      pinMode(SOC_GPIO_PIN_T1000_SCK,     INPUT);

      /* LR1110 control pins */
      pinMode(SOC_GPIO_PIN_T1000_DIO9,    INPUT);
      pinMode(SOC_GPIO_PIN_T1000_BUSY,    INPUT);

      /* LEDs off */
      digitalWrite(SOC_GPIO_LED_T1000_GREEN, LOW);
      digitalWrite(SOC_GPIO_LED_T1000_RED,   LOW);
      pinMode(SOC_GPIO_LED_T1000_GREEN,    INPUT);
      pinMode(SOC_GPIO_LED_T1000_RED,      INPUT);

#else   // old code
      pinMode(SOC_GPIO_PIN_GNSS_T1000_RINT, INPUT_PULLDOWN);
      pinMode(SOC_GPIO_PIN_GNSS_T1000_SINT, INPUT_PULLDOWN);
      pinMode(SOC_GPIO_PIN_GNSS_T1000_RST,  INPUT_PULLDOWN);
      pinMode(SOC_GPIO_PIN_GNSS_T1000_VRTC, INPUT_PULLUP);
      pinMode(SOC_GPIO_PIN_GNSS_T1000_EN,   INPUT_PULLDOWN);

#if !defined(EXCLUDE_IMU)
      pinMode(SOC_GPIO_PIN_T1000_ACC_EN,    INPUT_PULLDOWN);
#endif /* EXCLUDE_IMU */
      pinMode(SOC_GPIO_PIN_T1000_BUZZER_EN, INPUT_PULLDOWN);
      pinMode(SOC_GPIO_PIN_T1000_3V3_EN,    INPUT_PULLDOWN);

      pinMode(SOC_GPIO_PIN_T1000_SS,        INPUT_PULLUP);

      digitalWrite(SOC_GPIO_LED_T1000_GREEN, LOW);
      digitalWrite(SOC_GPIO_LED_T1000_RED,   LOW);
      pinMode(SOC_GPIO_LED_T1000_GREEN,     INPUT);
      pinMode(SOC_GPIO_LED_T1000_RED,       INPUT);
      pinMode(SOC_GPIO_PIN_SFL_T1000_EN,    INPUT);
#endif
      break;

    case NRF52_ELECROW_TN_M1:

      digitalWrite(SOC_GPIO_PIN_GNSS_M1_WKE, LOW);

      digitalWrite(SOC_GPIO_LED_M1_RED,  HIGH);
      digitalWrite(SOC_GPIO_LED_M1_BLUE, HIGH);

      pinMode(SOC_GPIO_LED_M1_RED,       INPUT);
      pinMode(SOC_GPIO_LED_M1_BLUE,      INPUT);
      pinMode(SOC_GPIO_LED_M1_RED_PWR,   INPUT);

      pinMode(SOC_GPIO_PIN_IO_M1_PWR,    INPUT);
      pinMode(SOC_GPIO_PIN_SFL_M1_HOLD,  INPUT);
      pinMode(SOC_GPIO_PIN_SFL_M1_WP,    INPUT);
      pinMode(SOC_GPIO_PIN_SFL_M1_SS,    INPUT);
      pinMode(SOC_GPIO_PIN_GNSS_M1_RST,  INPUT);
      break;

    case NRF52_ELECROW_TN_M3:

      digitalWrite(SOC_GPIO_PIN_M3_ADC_EN,   LOW);
      //pinMode(SOC_GPIO_PIN_M3_ADC_EN,   OUTPUT);
      digitalWrite(SOC_GPIO_PIN_M3_EN1,   LOW);
      //pinMode(SOC_GPIO_PIN_M3_EN1,   OUTPUT);
      digitalWrite(SOC_GPIO_PIN_M3_EN2,   LOW);
      //pinMode(SOC_GPIO_PIN_M3_EN2,   OUTPUT);
      digitalWrite(SOC_GPIO_PIN_GNSS_M3_EN,   LOW);
      //pinMode(SOC_GPIO_PIN_GNSS_M3_EN,   OUTPUT);

      /* LR1110 NSS must stay high to prevent spurious wakeup? */
      //pinMode(SOC_GPIO_PIN_M3_SS, INPUT_PULLUP);

      digitalWrite(SOC_GPIO_PIN_GNSS_M3_WKE, LOW);
      digitalWrite(SOC_GPIO_LED_M3_RGB_PWR,  LOW);

      digitalWrite(SOC_GPIO_LED_M3_RED,  HIGH);
      digitalWrite(SOC_GPIO_LED_M3_GREEN,HIGH);
      digitalWrite(SOC_GPIO_LED_M3_BLUE, HIGH);

      pinMode(SOC_GPIO_LED_M3_RED,       INPUT);
      pinMode(SOC_GPIO_LED_M3_GREEN,     INPUT);
      pinMode(SOC_GPIO_LED_M3_BLUE,      INPUT);
      pinMode(SOC_GPIO_LED_M3_RGB_PWR,   INPUT);

//      pinMode(SOC_GPIO_PIN_GNSS_M3_WKE,  INPUT);
      pinMode(SOC_GPIO_PIN_GNSS_M3_RST,  INPUT);
      pinMode(SOC_GPIO_PIN_GNSS_M3_EN,   INPUT);
      pinMode(SOC_GPIO_PIN_M3_EEPROM_EN, INPUT_PULLDOWN);
      pinMode(SOC_GPIO_PIN_M3_EN1,       INPUT);
      pinMode(SOC_GPIO_PIN_M3_EN2,       INPUT);
      pinMode(SOC_GPIO_PIN_M3_ADC_EN,    INPUT);
      pinMode(SOC_GPIO_PIN_M3_ACC_EN,    INPUT_PULLDOWN);
      pinMode(SOC_GPIO_PIN_M3_TEMP_EN,   INPUT_PULLDOWN);

      // no equivalent to SOC_GPIO_PIN_IO_M1_PWR ?

      break;

    case NRF52_NORDIC_PCA10059:
    default:
//      ledOff(SOC_GPIO_LED_PCA10059_GREEN);
      ledOff(SOC_GPIO_LED_PCA10059_RED);
      ledOff(SOC_GPIO_LED_PCA10059_BLUE);
      ledOff(SOC_GPIO_LED_PCA10059_STATUS);

//      pinMode(SOC_GPIO_LED_PCA10059_GREEN,  INPUT);
      pinMode(SOC_GPIO_LED_PCA10059_RED,    INPUT);
      pinMode(SOC_GPIO_LED_PCA10059_BLUE,   INPUT);
      pinMode(SOC_GPIO_LED_PCA10059_STATUS, INPUT);
      break;
  }

  nRF52_WDT_fini();

if (reason != SOFTRF_SHUTDOWN_CHARGE) {

  Serial_GNSS_In.end();

  if (i2c != nullptr) Wire.end();
}

  if (nRF52_board != NRF52_SEEED_T1000E &&
      nRF52_board != NRF52_ELECROW_TN_M3) {
    pinMode(SOC_GPIO_PIN_SS, INPUT_PULLUP);
  }

  pinMode(lmic_pins.rst,  INPUT);

  // mode_button_pin was also set up in Button_setup()

  switch (nRF52_board)
  {
    case NRF52_SEEED_T1000E:
      mode_button_pin = SOC_GPIO_PIN_T1000_BUTTON;
      pinMode(mode_button_pin, INPUT_PULLDOWN);
      break;

    case NRF52_ELECROW_TN_M1:
      mode_button_pin = SOC_GPIO_PIN_M1_BUTTON1;
      pinMode(mode_button_pin, INPUT_PULLUP);
      break;

    case NRF52_ELECROW_TN_M3:
      mode_button_pin = SOC_GPIO_PIN_M3_BUTTON;
      pinMode(mode_button_pin, INPUT_PULLUP);
      break;

    case NRF52_LILYGO_TECHO_REV_1:
      mode_button_pin = SOC_GPIO_PIN_BUTTON;
      pinMode(mode_button_pin, INPUT_PULLUP);
      break;

    //case NRF52_LILYGO_TECHO_REV_0:
    //case NRF52_LILYGO_TECHO_REV_2:
    //case NRF52_LILYGO_TECHO_PLUS:
    //case NRF52_NORDIC_PCA10059:
    default:
      mode_button_pin = SOC_GPIO_PIN_BUTTON;
      pinMode(mode_button_pin, INPUT);
      break;
  }

  // pinMode(SOC_GPIO_PIN_PAD,    INPUT);

  // wait for button-press to end
  delay(100);
  while (digitalRead(mode_button_pin) == (nRF52_board == NRF52_SEEED_T1000E ? HIGH : LOW)) {
      int status_LED = SOC_GPIO_PIN_STATUS;      // red LED on T1000E & M3
      pinMode(status_LED, OUTPUT);
      digitalWrite(status_LED, LED_STATE_ON);
      delay(100);
      digitalWrite(status_LED, 1-LED_STATE_ON);  // blink LED while waiting
      pinMode(status_LED, INPUT);
      delay(200);
      nRF52_WDT_fini();       // resets the WDT timer
  }

#if defined(USE_TINYUSB)
  Serial1.end();
#endif

Serial.println("about to setup wake-up pins");

  switch (reason)
  {
#if 0
#if defined(USE_SERIAL_DEEP_SLEEP)
  case SOFTRF_SHUTDOWN_NMEA:
#if !defined(ARDUINO_ARCH_MBED) && !defined(ARDUINO_ARCH_ZEPHYR)
    pinMode(SOC_GPIO_PIN_CONS_RX, INPUT_PULLUP_SENSE /* INPUT_SENSE_LOW */);
#endif /* ARDUINO_ARCH_MBED */
    break;
#endif
#endif
  //case SOFTRF_SHUTDOWN_BUTTON:
  //case SOFTRF_SHUTDOWN_LOWBAT:
  //case SOFTRF_SHUTDOWN_CHARGE:
  //case SOFTRF_SHUTDOWN_EXTPWR:
  //case SOFTRF_SHUTDOWN_NMEA:
  //case SOFTRF_SHUTDOWN_RESET:
  default:

Serial.println("about to end Serial");
Serial.flush();
delay(100);

    Serial.end();

    nRF52_WDT_fini();

 /* Reset-based shutdown: reboot into a clean-slate state, then
    nRF52_setup() detects SHUTDOWN_MAGIC and enters SYSTEMOFF with
    all peripherals at power-on defaults. This replicates the low-drain
    path observed (on T1000E) when USB is unplugged post-shutdown. */
    NRF_POWER->GPREGRET = DFU_MAGIC_SKIP;
    if (reason == SOFTRF_SHUTDOWN_RESET)
        NRF_POWER->GPREGRET2 = RESET_MAGIC;    // just reset, do not sleep
    else
        NRF_POWER->GPREGRET2 = SHUTDOWN_MAGIC;
    // setting up the wake-up pins here is therefore meaningless
    // but do it anyway in case we experiment with nRF52_system_off() here

#if 0
    // maybe need this before the software reset too?
    //    - already did it at boot, at beginning of setup()
    // Clear LATCH before sleep to prevent stale DETECT assertion:
    //NRF_GPIO->LATCH = NRF_GPIO->LATCH;
    NRF_GPIO->LATCH = 0xFFFFFFFF;
    // clear stale reset reasons:
    NRF_POWER->RESETREAS = 0xFFFFFFFF;
#endif
#if 1
    //NVIC_SystemReset();    // software reset
#if !defined(ARDUINO_ARCH_MBED) && !defined(ARDUINO_ARCH_ZEPHYR)
    uint8_t sd_en;
    (void) sd_softdevice_is_enabled(&sd_en);
    if (sd_en)    // Run bootloader with SoftDevice enabled    
        sd_nvic_SystemReset();
    else         // Run bootloader with SoftDevice disabled
#endif
        NVIC_SystemReset();
#endif

    // cannot get here if we did a SystemReset()
#if 1
  // setup wake-up pins
#if !defined(ARDUINO_ARCH_MBED) && !defined(ARDUINO_ARCH_ZEPHYR)
    if (nRF52_board == NRF52_ELECROW_TN_M3)
        pinMode(SOC_GPIO_PIN_M3_BUT_EN, INPUT_PULLUP);
    pinMode(mode_button_pin, nRF52_board == NRF52_SEEED_T1000E ?
                             INPUT_PULLDOWN_SENSE /* INPUT_SENSE_HIGH */ :
                             INPUT_PULLUP_SENSE   /* INPUT_SENSE_LOW  */);
#endif
#endif
    nRF52_system_off(reason);

    break;
  }

Serial.println("should never get here");

#if 0
  //Serial.end();
  nRF52_system_off(reason);
#endif

}   // end of nRF52_fini()

bool nRF52_onExternalPower()
{
    bool ext_power = false;
    if (ext_power_pin != SOC_UNUSED_PIN) {
        pinMode(ext_power_pin, INPUT);
        delay(100);
        ext_power = digitalRead(ext_power_pin);
//Serial.print("ext_power = ");
//Serial.println(ext_power);
    } else {
        ext_power = (SoC->Battery_param(BATTERY_PARAM_VOLTAGE) > 4.1f);
        // not great since after full charge it might be >4.1 without ext power
        // - but if settings->power_ext=false then no problem
    }
    return ext_power;
}

static void nRF52_reset()
{
  if (nrf_wdt_started(NRF_WDT)) {
    // When WDT is active - CRV, RREN and CONFIG are blocked
    // There is no way to stop/disable watchdog using source code
    // It can only be reset by WDT timeout, Pin reset, Power reset
#if defined(USE_EPAPER)
    if (hw_info.display == DISPLAY_EPD_1_54) {

#if defined(USE_EPD_TASK)
      while (EPD_update_in_progress != EPD_UPDATE_NONE) {
          nRF52_WDT_fini();
          delay(100);
      }
//    while (!SoC->Display_lock()) { delay(10); }
#endif

      EPD_Message("PLEASE,", "WAIT..");
#if defined(USE_EPD_TASK)
      while (EPD_update_in_progress != EPD_UPDATE_NONE) {
          nRF52_WDT_fini();
          delay(100);
      }
#endif

    }
#endif /* USE_EPAPER */

#if 0
    while (true) { delay(100); }
// >>> this intentionally provokes the dog to bite?
// >>> call sd_nvic_SystemReset() instead?
//     - but rumor has it that even a reset() doesn't stop the watchdog!
  } else {
    NVIC_SystemReset();  // >>> if SD active should use sd_nvic_SystemReset()
#endif
  }

  nRF52_fini(SOFTRF_SHUTDOWN_RESET);  // will turn off peripherals and then reset

#if 0  // older code:
#if !defined(ARDUINO_ARCH_MBED) && !defined(ARDUINO_ARCH_ZEPHYR)
    uint8_t sd_en;
    (void) sd_softdevice_is_enabled(&sd_en);
    if (sd_en)    // Run bootloader with SoftDevice enabled    
        sd_nvic_SystemReset();
    else         // Run bootloader with SoftDevice disabled
#endif
        NVIC_SystemReset();
#endif
}

static uint32_t nRF52_getChipId()
{
#if !defined(SOFTRF_ADDRESS)
  uint32_t id = DEVICE_ID_LOW;

  return DevID_Mapper(id);
#else
  return (SOFTRF_ADDRESS & 0xFFFFFFFFU );
#endif
}

static void* nRF52_getResetInfoPtr()
{
  return (void *) &reset_info;
}

static String nRF52_getResetInfo()
{
  switch (reset_info.reason)
  {
    default                     : return F("No reset information available");
  }
}

static String nRF52_getResetReason()
{
  switch (reset_info.reason)
  {
    case REASON_DEFAULT_RST       : return F("DEFAULT");
    case REASON_WDT_RST           : return F("WDT");
    case REASON_EXCEPTION_RST     : return F("EXCEPTION");
    case REASON_SOFT_WDT_RST      : return F("SOFT_WDT");
    case REASON_SOFT_RESTART      : return F("SOFT_RESTART");
    case REASON_DEEP_SLEEP_AWAKE  : return F("DEEP_SLEEP_AWAKE");
    case REASON_EXT_SYS_RST       : return F("EXT_SYS");
    default                       : return F("NO_MEAN");
  }
}

static uint32_t nRF52_getFreeHeap()
{
  return dbgHeapTotal() - dbgHeapUsed();
}

static long nRF52_random(long howsmall, long howBig)
{
  return random(howsmall, howBig);
}

#if defined(USE_USB_MIDI)
byte note_sequence[] = {62,65,69,65,67,67,65,64,69,69,67,67,62,62};
#endif /* USE_USB_MIDI */

static void nRF52_Batt_beeps()
{
#if defined(USE_PWM_SOUND)
  if (buzzerpin == SOC_UNUSED_PIN)  return;
  /* if (settings->volume != BUZZER_OFF)  return; */

nRF52_WDT_fini();

  float charge = SoC->Battery_param(BATTERY_PARAM_CHARGE);
  Serial.print("Battery charge % ");
  Serial.println(charge);

  if (charge > 35.0f)
    { tone(buzzerpin, 1300, 100); delay(300); }
  else if (charge > 15.0f)
    { tone(buzzerpin, 1040, 100); delay(300); }
  else
    { tone(buzzerpin,  640, 400); delay(800); }
  if (charge > 35.0f)
    { tone(buzzerpin, 1300, 100); delay(300); }
  if (charge > 55.0f)
    { tone(buzzerpin, 1300, 100); delay(300); }
  if (charge > 75.0f)
    { tone(buzzerpin, 1300, 100); delay(300); }
  if (charge > 95.0f)
    { tone(buzzerpin, 1300, 100); delay(300); }

nRF52_WDT_fini();
#endif /* USE_PWM_SOUND */
}

// This is called from SoftRF.ino after SoC->setup()
// If both battery & USB power, shut down and just charge
// - setting power_ext defeats this
// - do this here so can be called after settings are known
// Doing this here has the advantage of beeping to verify charge mode.
// To fully boot if settings->power_ext=false, only plug in after boot
void nRF52_charge_mode()
{
    bool ext_power = nRF52_onExternalPower();
    bool ext_pwr_wake = false;
    if (nRF52_board == NRF52_SEEED_T1000E)
        ext_pwr_wake = ((reset_reason & POWER_RESETREAS_VBUS_Msk) != 0
                       || (latch & (1 << SOC_GPIO_PIN_T1000_CHG_PWR)) != 0);
    if (nRF52_board == NRF52_ELECROW_TN_M1
    ||  nRF52_board == NRF52_ELECROW_TN_M3)
        ext_pwr_wake = ((reset_reason & POWER_RESETREAS_VBUS_Msk) != 0);

    Serial.print("\r\nsettings->power_ext = ");
    Serial.println(settings->power_ext);
    Serial.print("onExternalPower() = ");
    Serial.println(ext_power);
    Serial.print("wakeup was due to external power = ");
    Serial.println(ext_pwr_wake);

    if (ext_pwr_wake == false
        // if wake-up was NOT due to connection of USB power,
        // - i.e., if turned on by button while charging
//  || ext_power == false
        // or if there is no USB power connected
        // (even if removal of USB power caused a Vbus wake-up),
    || settings->power_ext) {
        // or with the setting to avoid charge mode,
        // then do not enter charge mode, finish booting:
        tone(buzzerpin, 1400, 250);  // delay(300);
nRF52_WDT_fini();
        return;
    }

    // beep to signal entering charge mode and report battery level
    nRF52_Batt_beeps();

    Serial.println("\r\nCHARGE MODE\r\n");

    // play descending tones to signal shutdown
    delay(1000);
    tone(buzzerpin, 640, 250); delay(300);
    tone(buzzerpin, 440, 250); delay(900);

    //SoC->Display_fini(SOFTRF_SHUTDOWN_CHARGE);
    //SoC->Button_fini();
    //  - moved the call to nRF52_charge_mode() to before setup of display & buttons

    nRF52_fini(SOFTRF_SHUTDOWN_CHARGE);
}

static void nRF52_Buzzer_test(int var)
{
#if defined(USE_USB_MIDI)
  if (USBDevice.mounted() /* && settings->volume != BUZZER_OFF */ ) {
    unsigned int position = 0;
    unsigned int current  = 0;

    for (; position <= sizeof(note_sequence); position++) {
      // Setup variables for the current and previous
      // positions in the note sequence.
      current = position;
      // If we currently are at position 0, set the
      // previous position to the last note in the sequence.
      unsigned int previous = (current == 0) ? (sizeof(note_sequence)-1) : current - 1;

      // Send Note On for current position at full velocity (127) on channel 1.
      MIDI_USB.sendNoteOn(note_sequence[current], 127, MIDI_CHANNEL_TRAFFIC);

      // Send Note Off for previous note.
      MIDI_USB.sendNoteOff(note_sequence[previous], 0, MIDI_CHANNEL_TRAFFIC);

      delay(286);
    }

    MIDI_USB.sendNoteOff(note_sequence[current], 0, MIDI_CHANNEL_TRAFFIC);
  }
#endif /* USE_USB_MIDI */

  if (nRF52_board == NRF52_LILYGO_TECHO_PLUS && nRF52_has_vibra == true) {
    vibra.setWaveform(0, 75); /* Transition Ramp Down Short Smooth 2 - 100 to 0% */
    vibra.setWaveform(1, 0);
    vibra.run();
  }

#if defined(USE_PWM_SOUND)
  if (buzzerpin != SOC_UNUSED_PIN /* && settings->volume != BUZZER_OFF */ ) {
/*
    tone(buzzerpin, 440,  500); delay(700);
    tone(buzzerpin, 640,  500); delay(700);
    tone(buzzerpin, 840,  500); delay(700);
*/
    tone(buzzerpin, 1040, 500); delay(1000);
    nRF52_Batt_beeps();
    noTone(buzzerpin);
    pinMode(buzzerpin, INPUT);
  }
#endif /* USE_PWM_SOUND */
}

#if defined(USE_BLE_MIDI)
extern BLEMidi blemidi;
extern midi::MidiInterface<BLEMidi> MIDI_BLE;
#endif /* USE_BLE_MIDI */

#if 0
static void nRF52_Sound_tone(int hz, uint8_t volume)
{
#if defined(USE_PWM_SOUND)
  if (SOC_GPIO_PIN_BUZZER != SOC_UNUSED_PIN /* && volume != BUZZER_OFF */ ) {
    if (hz > 0) {
      tone(SOC_GPIO_PIN_BUZZER, hz, ALARM_TONE_MS);
    } else {
      noTone(SOC_GPIO_PIN_BUZZER);
      pinMode(SOC_GPIO_PIN_BUZZER, INPUT);
    }
  }
#endif /* USE_PWM_SOUND */

#if defined(USE_USB_MIDI)
  if (USBDevice.mounted() /* && settings->volume != BUZZER_OFF */ ) {
    if (hz > 0) {
      MIDI_USB.sendNoteOn (60, 127, MIDI_CHANNEL_TRAFFIC); // 60 == middle C
    } else {
      MIDI_USB.sendNoteOff(60,   0, MIDI_CHANNEL_TRAFFIC);
    }
  }
#endif /* USE_USB_MIDI */

#if defined(USE_BLE_MIDI)
  if (/* settings->volume != BUZZER_OFF  && */
      Bluefruit.connected() && blemidi.notifyEnabled()) {
    if (hz > 0) {
      MIDI_BLE.sendNoteOn (60, 127, MIDI_CHANNEL_TRAFFIC); // 60 == middle C
    } else {
      MIDI_BLE.sendNoteOff(60,   0, MIDI_CHANNEL_TRAFFIC);
    }
  }
#endif /* USE_BLE_MIDI */
}
#endif

// mimic the API of ESP32 ToneAC, as used by buzzer.cpp
// if duration = 0 then forever, until turned off
// background = true to return to main thread while sounding tone
void noToneAC()
{
#if defined(USE_PWM_SOUND)
  if (buzzerpin != SOC_UNUSED_PIN)
      noTone(buzzerpin);
#endif
}
void toneAC(unsigned long hz, uint8_t volume, unsigned long duration, uint8_t background)
{
#if defined(USE_PWM_SOUND)
  if (buzzerpin == SOC_UNUSED_PIN)  return;
  /* if (settings->volume == BUZZER_OFF)  return; */
  if (hz == 0)
      noToneAC();
  if (background && duration == 0)
      duration = 0xFFFF;                    // not quite forever
  tone(buzzerpin, hz, duration);
  if (background)
      return;
  delay(duration);
  noToneAC();
#endif
}

/* wrapper function to fit the SoC_ops structure */
static void nRF52_Buzzer_tone(int hz, int duration)
{
  toneAC(hz, (uint8_t)10, duration, 0);
}

// toggle buzzer on/off via button
static void nRF52_BuzzerVolumeChange()
{
#if defined(USE_PWM_SOUND)
  if (settings->volume == BUZZER_OFF) {
      settings->volume = BUZZER_VOLUME_FULL;
      nRF52_Buzzer_tone(640,  500); delay(500);
      nRF52_Buzzer_tone(840,  500);
/*
  } else if (settings->volume == BUZZER_VOLUME_FULL) {
      settings->volume = BUZZER_OFF;
      nRF52_Buzzer_tone(740,  500); delay(700);
      nRF52_Buzzer_tone(740,  600);
*/
  } else {
      settings->volume = BUZZER_OFF;
      nRF52_Buzzer_tone(840,  500); delay(500);
      nRF52_Buzzer_tone(640,  500);
  }
#endif /* USE_PWM_SOUND */
}

static void nRF52_WiFi_set_param(int ndx, int value)
{
  /* NONE */
}

static void nRF52_WiFi_transmit_UDP(int port, byte *buf, size_t size)
{
  /* NONE */
}

static bool nRF52_EEPROM_begin(size_t size)
{
#if !defined(EXCLUDE_EEPROM)
  if (size > EEPROM.length()) {
    return false;
  }

  EEPROM.begin();
#endif

  return true;
}

#if 1
static void nRF52_EEPROM_extension(int cmd)
{
    // nothing to do
}
#else
static void nRF52_EEPROM_extension(int cmd)
{
  uint8_t *raw = (uint8_t *) &ui_settings;

  switch (cmd)
  {
    case EEPROM_EXT_STORE:
      for (int i=0; i<sizeof(ui_settings_t); i++) {
        EEPROM.write(sizeof(eeprom_t) + i, raw[i]);
      }
      return;
    case EEPROM_EXT_DEFAULTS:
      //ui_settings.adapter      = 0;
      //ui_settings.connection   = 0;
#if defined(DEFAULT_REGION_US)
      ui_settings.units        = UNITS_IMPERIAL;
#else
      ui_settings.units        = UNITS_METRIC;
#endif
      ui_settings.zoom         = ZOOM_MEDIUM;
      //ui_settings.protocol     = PROTOCOL_NMEA;
      //ui_settings.baudrate     = 0;
      //strcpy(ui->server, "");
      //strcpy(ui->key,    "");
      ui_settings.rotate       = ROTATE_0;
      ui_settings.orientation  = DIRECTION_TRACK_UP;
      ui_settings.adb          = DB_NONE;
      ui_settings.epdidpref    = ID_TYPE;
      ui_settings.viewmode     = VIEW_MODE_STATUS;
      ui_settings.voice        = VOICE_OFF;
      ui_settings.antighost    = ANTI_GHOSTING_OFF;
      //ui_settings.filter       = TRAFFIC_FILTER_OFF;
      ui_settings.power_save   = 0;
      ui_settings.team         = 0;
      break;
    case EEPROM_EXT_LOAD:
    default:
      for (int i=0; i<sizeof(ui_settings_t); i++) {
        raw[i] = EEPROM.read(sizeof(eeprom_t) + i);
      }

#if defined(USE_WEBUSB_SETTINGS) && !defined(USE_WEBUSB_SERIAL)

      usb_web.setLandingPage(&landingPage);
      usb_web.begin();

#endif /* USE_WEBUSB_SETTINGS */

      break;
  }
}
#endif

static void nRF52_SPI_begin()
{
#if !defined(ARDUINO_ARCH_MBED) && !defined(ARDUINO_ARCH_ZEPHYR)
  switch (nRF52_board)
  {
    case NRF52_SEEED_T1000E:
      SPI.setPins(SOC_GPIO_PIN_T1000_MISO,
                  SOC_GPIO_PIN_T1000_SCK,
                  SOC_GPIO_PIN_T1000_MOSI);
      break;
    case NRF52_ELECROW_TN_M3:
      SPI.setPins(SOC_GPIO_PIN_M3_MISO,
                  SOC_GPIO_PIN_M3_SCK,
                  SOC_GPIO_PIN_M3_MOSI);
      break;
    case NRF52_NORDIC_PCA10059:
      SPI.setPins(SOC_GPIO_PIN_PCA10059_MISO,
                  SOC_GPIO_PIN_PCA10059_SCK,
                  SOC_GPIO_PIN_PCA10059_MOSI);
      break;
    //case NRF52_LILYGO_TECHO_REV_0:
    //case NRF52_LILYGO_TECHO_REV_1:
    //case NRF52_LILYGO_TECHO_REV_2:
    //case NRF52_LILYGO_TECHO_PLUS:
    //case NRF52_ELECROW_TN_M1:
    default:
      SPI.setPins(SOC_GPIO_PIN_TECHO_REV_0_MISO,
                  SOC_GPIO_PIN_TECHO_REV_0_SCK,
                  SOC_GPIO_PIN_TECHO_REV_0_MOSI);
      break;
  }
#endif /* ARDUINO_ARCH_MBED */

  SPI.begin();
}

static void nRF52_swSer_begin(unsigned long baud)
{
#if !defined(ARDUINO_ARCH_MBED) && !defined(ARDUINO_ARCH_ZEPHYR)
  switch (nRF52_board)
  {
    case NRF52_SEEED_T1000E:
      Serial_GNSS_In.setPins(SOC_GPIO_PIN_GNSS_T1000_RX,
                             SOC_GPIO_PIN_GNSS_T1000_TX);
      baud = 115200; /* Airoha AG3335 default value */
      break;
    case NRF52_ELECROW_TN_M3:
      Serial_GNSS_In.setPins(SOC_GPIO_PIN_GNSS_M3_RX,
                             SOC_GPIO_PIN_GNSS_M3_TX);
      break;
    //case NRF52_LILYGO_TECHO_REV_0:
    //case NRF52_LILYGO_TECHO_REV_1:
    //case NRF52_LILYGO_TECHO_REV_2:
    //case NRF52_LILYGO_TECHO_PLUS:
    //case NRF52_NORDIC_PCA10059:
    //case NRF52_ELECROW_TN_M1:
    default:
      Serial_GNSS_In.setPins(SOC_GPIO_PIN_GNSS_RX, SOC_GPIO_PIN_GNSS_TX);
      break;
  }
#endif /* ARDUINO_ARCH_MBED */

  Serial_GNSS_In.begin(baud);

  if (nRF52_board == NRF52_SEEED_T1000E)
  {
    for (int i=0; i<25; i++) {
      /* Enable Sleep mode locking */
      Serial_GNSS_Out.write("$PAIR382,1*2E\r\n"); delay(40);
    }
  }
}

static void nRF52_swSer_enableRx(boolean arg)
{
  /* NONE */
}

SemaphoreHandle_t Display_Semaphore;
unsigned long TaskInfoTime;

#if defined(USE_EPAPER)

#include <SoftSPI.h>
SoftSPI swSPI(SOC_GPIO_PIN_EPD_MOSI,
              SOC_GPIO_PIN_EPD_MOSI, /* half duplex */
              SOC_GPIO_PIN_EPD_SCK);

// As it happens, the EPD MOSI, SS, DC, RST and BUSY pins
// are the same on the M1 as the T-Echo
// - even as the EPD MISO pin is different

static nRF52_display_id nRF52_EPD_ident()
{
  nRF52_display_id rval = EP_GDEH0154D67; /* default */

  digitalWrite(SOC_GPIO_PIN_EPD_SS, HIGH);
  pinMode(SOC_GPIO_PIN_EPD_SS, OUTPUT);
  digitalWrite(SOC_GPIO_PIN_EPD_DC, HIGH);
  pinMode(SOC_GPIO_PIN_EPD_DC, OUTPUT);

  digitalWrite(SOC_GPIO_PIN_EPD_RST, LOW);
  pinMode(SOC_GPIO_PIN_EPD_RST, OUTPUT);
  delay(20);
  pinMode(SOC_GPIO_PIN_EPD_RST, INPUT_PULLUP);
  delay(200);
  pinMode(SOC_GPIO_PIN_EPD_BUSY, INPUT);

  swSPI.begin();

  uint8_t buf_2D[11];
  uint8_t buf_2E[10];

  taskENTER_CRITICAL();

  digitalWrite(SOC_GPIO_PIN_EPD_DC, LOW);
  digitalWrite(SOC_GPIO_PIN_EPD_SS, LOW);

  swSPI.transfer_out(0x2D);

  pinMode(SOC_GPIO_PIN_EPD_MOSI, INPUT);
  digitalWrite(SOC_GPIO_PIN_EPD_DC, HIGH);

  for (int i=0; i<sizeof(buf_2D); i++) {
    buf_2D[i] = swSPI.transfer_in();
  }

  digitalWrite(SOC_GPIO_PIN_EPD_SCK, LOW);
  digitalWrite(SOC_GPIO_PIN_EPD_DC,  LOW);
  digitalWrite(SOC_GPIO_PIN_EPD_SS,  HIGH);

  taskEXIT_CRITICAL();

  delay(1);

  taskENTER_CRITICAL();

  digitalWrite(SOC_GPIO_PIN_EPD_DC, LOW);
  digitalWrite(SOC_GPIO_PIN_EPD_SS, LOW);

  swSPI.transfer_out(0x2E);

  pinMode(SOC_GPIO_PIN_EPD_MOSI, INPUT);
  digitalWrite(SOC_GPIO_PIN_EPD_DC, HIGH);

  for (int i=0; i<sizeof(buf_2E); i++) {
    buf_2E[i] = swSPI.transfer_in();
  }

  digitalWrite(SOC_GPIO_PIN_EPD_SCK, LOW);
  digitalWrite(SOC_GPIO_PIN_EPD_DC,  LOW);
  digitalWrite(SOC_GPIO_PIN_EPD_SS,  HIGH);

  taskEXIT_CRITICAL();

  swSPI.end();

#if 0
  Serial.print("2D: ");
  for (int i=0; i<sizeof(buf_2D); i++) {
    Serial.print(buf_2D[i], HEX);
    Serial.print(' ');
  }
  Serial.println();

  Serial.print("2E: ");
  for (int i=0; i<sizeof(buf_2E); i++) {
    Serial.print(buf_2E[i], HEX);
    Serial.print(' ');
  }
  Serial.println();

/*
 *  0x2D:
 *  FF FF FF FF FF FF FF FF FF FF FF - C1
 *  00 00 00 00 00 FF 40 00 00 00 01 - D67 SYX 1942
 *  00 00 00 FF 00 00 40 01 00 00 00 - D67 SYX 2118
 *  00 00 00 FF 00 00 40 01 00 00 00 - D67 SYX 2129
 *  00 00 00 00 00 00 00 00 00 00 00 - DEPG0150BN
 *  00 00 40 20 10 00 40 03 00 00 00 - TBD (M1)
 *                                   - "20.05.21" (Plus)
 *
 *  0x2E:
 *  00 00 00 00 00 00 00 00 00 00    - C1
 *  00 00 00 00 00 00 00 00 00 00    - D67 SYX 1942
 *  00 05 00 9A 00 55 35 37 14 0C    - D67 SYX 2118
 *  00 00 00 00 00 00 00 00 00 00    - D67 SYX 2129
 *  00 00 00 00 00 00 00 00 00 00    - DEPG0150BN
 *  00 00 00 00 00 00 00 00 00 00    - TBD (M1)
 *                                   - "20.05.21" (Plus)
 */
#endif

  bool is_ff = true;
  for (int i=0; i<sizeof(buf_2D); i++) {
    if (buf_2D[i] != 0xFF) {is_ff = false; break;}
  }

  bool is_00 = true;
  for (int i=0; i<sizeof(buf_2D); i++) {
    if (buf_2D[i] != 0x00) {is_00 = false; break;}
  }

  if (is_00) {
    rval = EP_DEPG0150BN;
  }

  return rval;
}

#endif /* USE_EPAPER */

static byte nRF52_Display_setup()
{
  byte rval = DISPLAY_NONE;

  if (nRF52_board == NRF52_NORDIC_PCA10059 ||
      nRF52_board == NRF52_SEEED_T1000E    ||
      nRF52_board == NRF52_ELECROW_TN_M3) {
      /* Nothing to do */

  } else {

#if defined(USE_EPAPER)

#if SPI_INTERFACES_COUNT >= 2
    switch (nRF52_board)
    {
      case NRF52_ELECROW_TN_M1:
        SPI1.setPins(SOC_GPIO_PIN_EPD_M1_MISO,
                     SOC_GPIO_PIN_EPD_M1_SCK,
                     SOC_GPIO_PIN_EPD_M1_MOSI);
        break;
      //case NRF52_LILYGO_TECHO_REV_0:
      //case NRF52_LILYGO_TECHO_REV_1:
      //case NRF52_LILYGO_TECHO_REV_2:
      //case NRF52_LILYGO_TECHO_PLUS:
      default:
        SPI1.setPins(SOC_GPIO_PIN_EPD_MISO,
                     SOC_GPIO_PIN_EPD_SCK,
                     SOC_GPIO_PIN_EPD_MOSI);
        break;
    }
#endif

    // As it happens, the EPD MOSI, SS, DC, RST and BUSY pins
    // are the same on the M1 as the T-Echo
    // - even as the EPD MISO pin is different

    if (nRF52_display == EP_UNKNOWN) {
      nRF52_display = nRF52_EPD_ident();
    }

    switch (nRF52_display)
    {
    case EP_GDEP015OC1:
      display = new GxEPD2_BW<GxEPD2_154, GxEPD2_154::HEIGHT> (
                      GxEPD2_154(
                        (nRF52_board==NRF52_ELECROW_TN_M1? SOC_GPIO_PIN_EPD_M1_SS : SOC_GPIO_PIN_EPD_SS),
                        SOC_GPIO_PIN_EPD_DC,
                        SOC_GPIO_PIN_EPD_RST,
                        SOC_GPIO_PIN_EPD_BUSY));
      break;
    case EP_DEPG0150BN:
      display = new GxEPD2_BW<GxEPD2_150_BN, GxEPD2_150_BN::HEIGHT> (
                      GxEPD2_150_BN(
                        (nRF52_board==NRF52_ELECROW_TN_M1? SOC_GPIO_PIN_EPD_M1_SS : SOC_GPIO_PIN_EPD_SS),
                        SOC_GPIO_PIN_EPD_DC,
                        SOC_GPIO_PIN_EPD_RST,
                        SOC_GPIO_PIN_EPD_BUSY));
      break;
    case EP_GDEH0154D67:
    default:
      display = new GxEPD2_BW<GxEPD2_154_D67, GxEPD2_154_D67::HEIGHT> (
                      GxEPD2_154_D67(
                        (nRF52_board==NRF52_ELECROW_TN_M1? SOC_GPIO_PIN_EPD_M1_SS : SOC_GPIO_PIN_EPD_SS),
                        SOC_GPIO_PIN_EPD_DC,
                        SOC_GPIO_PIN_EPD_RST,
                        SOC_GPIO_PIN_EPD_BUSY));
      break;
    }

    display->epd2.selectSPI(SPI1, SPISettings(4000000, MSBFIRST, SPI_MODE0));

    if (EPD_setup(true)) {

#if defined(USE_EPD_TASK)
      Display_Semaphore = xSemaphoreCreateBinary();

      if( Display_Semaphore != NULL ) {
        xSemaphoreGive( Display_Semaphore );
      }

      xTaskCreate(EPD_Task, "EPD", EPD_STACK_SZ, NULL,
                  /* TASK_PRIO_HIGH */ TASK_PRIO_LOW , &EPD_Task_Handle);

      TaskInfoTime = millis();
#endif
      rval = DISPLAY_EPD_1_54;
    }

    /* EPD back light off */
    pinMode(SOC_GPIO_PIN_EPD_BLGT, OUTPUT);
    digitalWrite(SOC_GPIO_PIN_EPD_BLGT, nRF52_board == NRF52_ELECROW_TN_M1 ?
                                        HIGH : LOW);
#endif /* USE_EPAPER */
  }

  return rval;
}

static void nRF52_Display_loop()
{
  switch (hw_info.display)
  {
#if defined(USE_EPAPER)
  case DISPLAY_EPD_1_54:
    EPD_loop();
    break;
#endif /* USE_EPAPER */

  case DISPLAY_NONE:
  default:
    break;
  }
}

void nRF52_Display_blank()
{
     EPD_Message("SCREEN", "SAVER");
     delay (1500);
     screen_saver = true;             // ignore touch until mode button pressed
     EPD_Message(NULL, NULL);         // blank the screen
}

static void nRF52_Display_fini(int reason)
{
  if (nRF52_board == NRF52_NORDIC_PCA10059 ||
      nRF52_board == NRF52_SEEED_T1000E    ||
      nRF52_board == NRF52_ELECROW_TN_M3)
          return;      /* Nothing to do */

  if (hw_info.display != DISPLAY_EPD_1_54)
          return;      /* Nothing to do */

#if defined(USE_EPAPER)

  EPD_fini(reason, screen_saver);

#if defined(USE_EPD_TASK)
  if( EPD_Task_Handle != NULL )
  {
    vTaskDelete( EPD_Task_Handle );
  }

  if( Display_Semaphore != NULL )
  {
    vSemaphoreDelete( Display_Semaphore );
  }
#endif

  SPI1.end();

  if (nRF52_board == NRF52_ELECROW_TN_M1) {
      pinMode(SOC_GPIO_PIN_EPD_M1_SS,  INPUT);
      pinMode(SOC_GPIO_PIN_EPD_M1_DC,  INPUT);
      pinMode(SOC_GPIO_PIN_EPD_M1_RST, INPUT);
  } else {
      // pinMode(SOC_GPIO_PIN_EPD_MISO, INPUT);
      // pinMode(SOC_GPIO_PIN_EPD_MOSI, INPUT);
      // pinMode(SOC_GPIO_PIN_EPD_SCK,  INPUT);
      pinMode(SOC_GPIO_PIN_EPD_SS,  INPUT);
      pinMode(SOC_GPIO_PIN_EPD_DC,  INPUT);
      pinMode(SOC_GPIO_PIN_EPD_RST, INPUT);
  }
  // pinMode(SOC_GPIO_PIN_EPD_BUSY, INPUT);
  pinMode(SOC_GPIO_PIN_EPD_BLGT, INPUT);

#endif /* USE_EPAPER */
}

static bool nRF52_Display_lock()
{
  bool rval = false;

  if ( Display_Semaphore != NULL ) {
    rval = (xSemaphoreTake( Display_Semaphore, ( TickType_t ) 0 ) == pdTRUE);
  }
//Serial.print("Display_lock: "); Serial.println(rval); Serial.flush();
  return rval;
}

static bool nRF52_Display_unlock()
{
  bool rval = false;

  if ( Display_Semaphore != NULL ) {
    rval = (xSemaphoreGive( Display_Semaphore ) == pdTRUE);
  }
//Serial.print("Display_unlock: "); Serial.println(rval); Serial.flush();
  return rval;
}

static void nRF52_Battery_setup()
{

}

static float nRF52_Battery_param(uint8_t param)
{
  uint32_t bat_adc_pin;
  float rval, voltage, mult;

  switch (param)
  {
  case BATTERY_PARAM_THRESHOLD:
    rval = BATTERY_THRESHOLD_LIPO;
/*
    rval = hw_info.model == SOFTRF_MODEL_BADGE    ? BATTERY_THRESHOLD_LIPO   :
           hw_info.model == SOFTRF_MODEL_CARD     ? BATTERY_THRESHOLD_LIPO   :
           hw_info.model == SOFTRF_MODEL_HANDHELD ? BATTERY_THRESHOLD_LIPO   :
           hw_info.model == SOFTRF_MODEL_POCKET   ? BATTERY_THRESHOLD_LIPO   :
                                                    BATTERY_THRESHOLD_NIMHX2;
*/
    break;

  case BATTERY_PARAM_CUTOFF:
    rval = BATTERY_CUTOFF_LIPO;
/*
    rval = hw_info.model == SOFTRF_MODEL_BADGE    ? BATTERY_CUTOFF_LIPO   :
           hw_info.model == SOFTRF_MODEL_CARD     ? BATTERY_CUTOFF_LIPO   :
           hw_info.model == SOFTRF_MODEL_HANDHELD ? BATTERY_CUTOFF_LIPO   :
           hw_info.model == SOFTRF_MODEL_POCKET   ? BATTERY_CUTOFF_LIPO   :
                                                    BATTERY_CUTOFF_NIMHX2;
*/
    break;

  case BATTERY_PARAM_CHARGE:
    voltage = Battery_voltage();
    if (voltage < Battery_cutoff())
      return 0;

    if (voltage > 4.2f)
      return 100.0f;

    if (voltage < 3.6f) {
      voltage -= 3.3f;
      return (voltage * (100.0f / 3.0f));
    }

    voltage -= 3.6f;
    rval = 10.0f + (voltage * 150.0f);
    break;

  case BATTERY_PARAM_VOLTAGE:
  default:
    voltage = 0.0f;

    switch (hw_info.pmu)
    {
#if !defined(EXCLUDE_PMU)
    case PMU_SY6970:
      if (sy6970.isBatteryConnect() /* TODO */) {
        voltage = sy6970.getBattVoltage();
      }
      break;
#endif /* EXCLUDE_PMU */
    case PMU_NONE:
    default:
      // Set the analog reference to 3.0V (default = 3.6V)
#if !defined(ARDUINO_ARCH_MBED) && !defined(ARDUINO_ARCH_ZEPHYR)
      analogReference(AR_INTERNAL_3_0);
#endif /* ARDUINO_ARCH_MBED */

      // Set the resolution to 12-bit (0..4095)
#if !defined(ARDUINO_ARCH_ZEPHYR)
      analogReadResolution(12); // Can be 8, 10, 12 or 14
#endif /* AARDUINO_ARCH_ZEPHYR */

      // Let the ADC settle
      delay(1);

      switch (nRF52_board)
      {
        case NRF52_SEEED_T1000E:
          bat_adc_pin = SOC_GPIO_PIN_T1000_BATTERY;
          mult        = SOC_ADC_T1000_VOLTAGE_DIV;
          break;
        case NRF52_ELECROW_TN_M1:
          bat_adc_pin = SOC_GPIO_PIN_M1_BATTERY;
          mult        = SOC_ADC_VOLTAGE_DIV;
          break;
        case NRF52_ELECROW_TN_M3:
          bat_adc_pin = SOC_GPIO_PIN_M3_BATTERY;
          mult        = SOC_ADC_M3_VOLTAGE_DIV;
          break;
        //case NRF52_LILYGO_TECHO_REV_0:
        //case NRF52_LILYGO_TECHO_REV_1:
        //case NRF52_LILYGO_TECHO_REV_2:
        //case NRF52_LILYGO_TECHO_PLUS:
        //case NRF52_NORDIC_PCA10059:
        default:
          bat_adc_pin = SOC_GPIO_PIN_BATTERY;
          mult        = SOC_ADC_VOLTAGE_DIV;
          break;
      }

      // Get the raw 12-bit, 0..3000mV ADC value
      voltage = analogRead(bat_adc_pin);

      // Set the ADC back to the default settings
#if !defined(ARDUINO_ARCH_MBED) && !defined(ARDUINO_ARCH_ZEPHYR)
      analogReference(AR_DEFAULT);
#endif /* ARDUINO_ARCH_MBED */
#if !defined(ARDUINO_ARCH_ZEPHYR)
      analogReadResolution(10);
#endif /* AARDUINO_ARCH_ZEPHYR */

      // Convert the raw value to compensated mv, taking the resistor-
      // divider into account (providing the actual LIPO voltage)
      // ADC range is 0..3000mV and resolution is 12-bit (0..4095)
      voltage *= (mult * VBAT_MV_PER_LSB);

      break;
    }

    rval = voltage * 0.001;
    break;
  }

  return rval;
}

void nRF52_GNSS_PPS_Interrupt_handler() {
  PPS_TimeMarker = millis();
}

static unsigned long nRF52_get_PPS_TimeMarker() {
  return PPS_TimeMarker;
}

static bool nRF52_Baro_setup() {
  if (nRF52_board == NRF52_SEEED_T1000E ||
      nRF52_board == NRF52_ELECROW_TN_M3)
        return false;
  // Baro_probe() no longer called from Baro_setup() so need to call it here:
  if (Baro_probe()) {                  // found baro sensor on Wire
      Serial.println(F("BMP found"));
      return true;
  }
  return false;
}

static void nRF52_UATSerial_begin(unsigned long baud)
{

}

static void nRF52_UATModule_restart()
{

}

static void nRF52_WDT_setup()
{
  Watchdog.enable(60000);   // <<< was 12000
}

static void nRF52_WDT_fini()
{
  // cannot disable nRF's WDT
  if (nrf_wdt_started(NRF_WDT)) {
    Watchdog.reset();
  }
}

#include <AceButton.h>
using namespace ace_button;

AceButton button_1(SOC_GPIO_PIN_BUTTON);
AceButton button_2(SOC_GPIO_PIN_PAD);

// The event handler for the button.
void handleEvent(AceButton* button, uint8_t eventType,
    uint8_t buttonState) {

  screen_saver_timer = millis();  // restart timer on any button activity

  switch (eventType) {

    case AceButton::kEventClicked:
    // case AceButton::kEventReleased:

      if (nRF52_board == NRF52_ELECROW_TN_M3 || nRF52_board == NRF52_SEEED_T1000E) {

          nRF52_BuzzerVolumeChange();   // toggle buzzer on/off

      } else {   // T-Echo and TN-M1

#if defined(USE_EPAPER)
        if (button == &button_1) {
#if 0
          if (eventType == AceButton::kEventClicked) {
            Serial.println(F("kEventClicked."));
          } else if (eventType == AceButton::kEventReleased) {
            Serial.println(F("kEventReleased."));
          }
#endif
          EPD_Mode(screen_saver);    // if screen_saver then redraws current page/mode
          screen_saver = false;

        } else if (button == &button_2) {    // touch
          if (! screen_saver)
              EPD_Up();
        }
#endif
      }

      if (FlightLogOpen)
          completeFlightLog();   // manually ensure complete flight log

      break;

    case AceButton::kEventDoubleClicked:
      if (button == &button_1) {

      if (nRF52_board == NRF52_SEEED_T1000E) {

             // TBD  <<< Vlad does stuff here

      } else if (nRF52_board == NRF52_ELECROW_TN_M3) {

             // TBD

      } else {   // T-Echo and TN-M1

#if defined(USE_EPAPER)
        if (settings->mode == SOFTRF_MODE_GPSBRIDGE) {
            settings->mode = SOFTRF_MODE_NORMAL;
            Serial.println(F("Switching to normal mode..."));
            EPD_Message("NORMAL", "MODE");
            delay(600);
        } else if (EPD_view_mode == VIEW_MODE_CONF) {
            EPD_view_mode = VIEW_CHANGE_SETTINGS;
            //Serial.println(F("Switching to change-settings screen..."));

        } else if (EPD_view_mode == VIEW_CHANGE_SETTINGS) {
            EPD_view_mode = VIEW_SAVE_SETTINGS;
            //Serial.println(F("Switching to save-settings screen..."));

        } else {   // view_modes other than CONF: toggle backlight
            //Serial.println(F("kEventDoubleClicked."));
            // as it happens, SOC_GPIO_PIN_EPD_M1_BLGT == SOC_GPIO_PIN_EPD_BLGT
            digitalWrite(SOC_GPIO_PIN_EPD_BLGT, 1-digitalRead(SOC_GPIO_PIN_EPD_BLGT));
        }
      }
#endif

      } else if (button == &button_2) {    // double-touch
#if defined(USE_EPAPER)
        if (! screen_saver)
            EPD_Down();
#endif
                 // more TBD <<< Vlad does stuff here
      }
      break;

    case AceButton::kEventLongPressed:

      if (button == &button_1) {

#if defined(USE_EPAPER)
        if (up_button_pin != SOC_UNUSED_PIN && digitalRead(up_button_pin) == LOW)
            screen_saver = false;
        else
            screen_saver = true;
#endif

        // switch to red LED to indicate shutdown

        if (hw_info.model == SOFTRF_MODEL_BADGE) {
            digitalWrite(SOC_GPIO_LED_TECHO_LED_RED, LOW);
            // ignore the other colors here
        }

        if (nRF52_board == NRF52_ELECROW_TN_M1) {
            digitalWrite(SOC_GPIO_LED_M1_RED,  LOW);
            digitalWrite(SOC_GPIO_LED_M1_BLUE, HIGH);
        }

        if (nRF52_board == NRF52_ELECROW_TN_M3) {
            digitalWrite(SOC_GPIO_LED_M3_RED,   LOW);
            digitalWrite(SOC_GPIO_LED_M3_GREEN, HIGH);
            digitalWrite(SOC_GPIO_LED_M3_BLUE,  HIGH);
        }

        if (nRF52_board == NRF52_SEEED_T1000E) {
            digitalWrite(SOC_GPIO_LED_T1000_RED,   HIGH);
            digitalWrite(SOC_GPIO_LED_T1000_GREEN, LOW);
        }

        shutdown(SOFTRF_SHUTDOWN_BUTTON);
        //Serial.println(F("This will never be printed."));
      
      } else if (button == &button_2) {    // long touch
#if defined(USE_EPAPER)
        if (digitalRead(mode_button_pin) != LOW)     // long-touch without push
            nRF52_Display_blank();
#endif
      }

      break;
  }
}

/* Callbacks for push button interrupt */
void onModeButtonEvent() {
  button_1.check();
}

/* Callbacks for touch button interrupt */
void onUpButtonEvent() {
  button_2.check();
}

static void nRF52_Button_setup()
{
  switch (nRF52_board)
  {
    case NRF52_SEEED_T1000E:
      mode_button_pin = SOC_GPIO_PIN_T1000_BUTTON;
      break;

    case NRF52_ELECROW_TN_M1:
      mode_button_pin = SOC_GPIO_PIN_M1_BUTTON1;
      up_button_pin   = SOC_GPIO_PIN_M1_BUTTON2;
      break;

    case NRF52_ELECROW_TN_M3:
      mode_button_pin = SOC_GPIO_PIN_M3_BUTTON;
      break;

    case NRF52_LILYGO_TECHO_REV_0:
    case NRF52_LILYGO_TECHO_REV_1:
    case NRF52_LILYGO_TECHO_REV_2:
    case NRF52_LILYGO_TECHO_PLUS:
    case NRF52_NORDIC_PCA10059:
      up_button_pin   = SOC_GPIO_PIN_PAD;
    default:
      mode_button_pin = SOC_GPIO_PIN_BUTTON;
      break;
  }

  // Button(s) uses external pull up resistor.
  pinMode(mode_button_pin, nRF52_board == NRF52_LILYGO_TECHO_REV_1 ? INPUT_PULLUP   :
                           nRF52_board == NRF52_ELECROW_TN_M1      ? INPUT_PULLUP   :
                           nRF52_board == NRF52_SEEED_T1000E       ? INPUT_PULLDOWN :
                           INPUT);
  if (up_button_pin != SOC_UNUSED_PIN) {
      if (nRF52_board == NRF52_ELECROW_TN_M1)
          pinMode(up_button_pin, INPUT);  // <<< was INPUT_PULLUP, maybe s.b. just INPUT ?
      else
          pinMode(up_button_pin, INPUT);
  }

  button_1.init(mode_button_pin, nRF52_board == NRF52_SEEED_T1000E ? LOW : HIGH);
  if (up_button_pin != SOC_UNUSED_PIN) { button_2.init(up_button_pin); }

  // Configure the ButtonConfig with the event handler, and enable all higher
  // level events.
  ButtonConfig* ModeButtonConfig = button_1.getButtonConfig();
  ModeButtonConfig->setEventHandler(handleEvent);
  //ModeButtonConfig->setFeature(ButtonConfig::kFeatureClick);      // <<< why is this line skipped?
  ModeButtonConfig->setFeature(ButtonConfig::kFeatureDoubleClick);
  ModeButtonConfig->setFeature(ButtonConfig::kFeatureLongPress);
  ModeButtonConfig->setFeature(ButtonConfig::kFeatureSuppressAfterClick);
  ModeButtonConfig->setFeature(ButtonConfig::kFeatureSuppressAfterDoubleClick);
  ModeButtonConfig->setFeature(
                    ButtonConfig::kFeatureSuppressClickBeforeDoubleClick);
//  ModeButtonConfig->setDebounceDelay(15);
  //ModeButtonConfig->setClickDelay(600);
  //ModeButtonConfig->setDoubleClickDelay(1500);
  ModeButtonConfig->setClickDelay(300);
  ModeButtonConfig->setDoubleClickDelay(600);
  ModeButtonConfig->setLongPressDelay(2000);

  ButtonConfig* UpButtonConfig = button_2.getButtonConfig();
  UpButtonConfig->setEventHandler(handleEvent);
  UpButtonConfig->setFeature(ButtonConfig::kFeatureClick);
  UpButtonConfig->setFeature(ButtonConfig::kFeatureDoubleClick);
  UpButtonConfig->setFeature(ButtonConfig::kFeatureLongPress);
  UpButtonConfig->setFeature(ButtonConfig::kFeatureSuppressAfterClick);
  UpButtonConfig->setFeature(ButtonConfig::kFeatureSuppressAfterDoubleClick);
//  UpButtonConfig->setDebounceDelay(15);
  //UpButtonConfig->setClickDelay(600);
  //UpButtonConfig->setDoubleClickDelay(1500);
  UpButtonConfig->setClickDelay(300);
  UpButtonConfig->setDoubleClickDelay(600);
  UpButtonConfig->setLongPressDelay(2000);

#if 0
//  attachInterrupt(digitalPinToInterrupt(mode_button_pin), onModeButtonEvent, CHANGE );  <<< why is this commented out?

  if (up_button_pin != SOC_UNUSED_PIN) {
      attachInterrupt(digitalPinToInterrupt(up_button_pin),   onUpButtonEvent,   CHANGE );   // <<< why needed?
  }
#endif
}

static void nRF52_Button_loop()
{
  button_1.check();           // <<< this is NOT interrupt-driven

  switch (nRF52_board)
  {
    case NRF52_LILYGO_TECHO_REV_0:
    case NRF52_LILYGO_TECHO_REV_1:
    case NRF52_LILYGO_TECHO_REV_2:
    case NRF52_LILYGO_TECHO_PLUS:
    case NRF52_NORDIC_PCA10059:
    case NRF52_ELECROW_TN_M1:
      button_2.check();
      break;
    default:
      break;
  }
}

static void nRF52_Button_fini()
{
#if 0
//  detachInterrupt(digitalPinToInterrupt(SOC_GPIO_PIN_BUTTON));

// only detach the second button, if any:

  switch (nRF52_board)
  {
    case NRF52_LILYGO_TECHO_REV_0:
    case NRF52_LILYGO_TECHO_REV_1:
    case NRF52_LILYGO_TECHO_REV_2:
    case NRF52_LILYGO_TECHO_PLUS:
    case NRF52_NORDIC_PCA10059:
      detachInterrupt(digitalPinToInterrupt(SOC_GPIO_PIN_PAD));
      break;
    case NRF52_ELECROW_TN_M1:
      detachInterrupt(digitalPinToInterrupt(SOC_GPIO_PIN_M1_BUTTON2));
      break;
    default:
      break;
  }
#endif
}

#if defined(USE_WEBUSB_SERIAL) && !defined(USE_WEBUSB_SETTINGS)
void line_state_callback(bool connected)
{
  if ( connected ) usb_web.println("WebUSB Serial example");
}
#endif /* USE_WEBUSB_SERIAL */

static void nRF52_USB_setup()
{
  if (USBSerial && USBSerial != Serial) {
    USBSerial.begin(SERIAL_OUT_BR, SERIAL_OUT_BITS);
  }

#if defined(USE_WEBUSB_SERIAL) && !defined(USE_WEBUSB_SETTINGS)

  usb_web.setLandingPage(&landingPage);
  usb_web.setLineStateCallback(line_state_callback);
  //usb_web.setStringDescriptor("TinyUSB WebUSB");
  usb_web.begin();

#endif /* USE_WEBUSB_SERIAL */
}

static void nRF52_USB_loop()
{

}

static void nRF52_USB_fini()
{
  if (USBSerial && USBSerial != Serial) {
    USBSerial.end();
  }

#if defined(USE_WEBUSB_SERIAL) && !defined(USE_WEBUSB_SETTINGS)

  /* TBD */

#endif /* USE_WEBUSB_SERIAL */
}

static int nRF52_USB_available()
{
  int rval = 0;

  if (USBSerial) {
    rval = USBSerial.available();
  }

#if defined(USE_WEBUSB_SERIAL) && !defined(USE_WEBUSB_SETTINGS)

  if ((rval == 0) && USBDevice.mounted() && usb_web.connected()) {
    rval = usb_web.available();
  }

#endif /* USE_WEBUSB_SERIAL */

  return rval;
}

static int nRF52_USB_read()
{
  int rval = -1;

  if (USBSerial) {
    rval = USBSerial.read();
  }

#if defined(USE_WEBUSB_SERIAL) && !defined(USE_WEBUSB_SETTINGS)

  if ((rval == -1) && USBDevice.mounted() && usb_web.connected()) {
    rval = usb_web.read();
  }

#endif /* USE_WEBUSB_SERIAL */

  return rval;
}

static size_t nRF52_USB_write(const uint8_t *buffer, size_t size)
{
  size_t rval = size;

  if (USBSerial && (size < USBSerial.availableForWrite())) {
    rval = USBSerial.write(buffer, size);
  }

#if defined(USE_WEBUSB_SERIAL) && !defined(USE_WEBUSB_SETTINGS)

  size_t rval_webusb = size;

  if (USBDevice.mounted() && usb_web.connected()) {
    rval_webusb = usb_web.write(buffer, size);
  }

//  rval = min(rval, rval_webusb);

#endif /* USE_WEBUSB_SERIAL */

  return rval;
}

IODev_ops_t nRF52_USBSerial_ops = {
  "nRF52 USBSerial",
  nRF52_USB_setup,
  nRF52_USB_loop,
  nRF52_USB_fini,
  nRF52_USB_available,
  nRF52_USB_read,
  nRF52_USB_write
};

static bool nRF52_ADB_setup()
{
  if (FATFS_is_mounted) {
    const char fileName[] = "/Aircrafts/ogn.cdb";

    if (ucdb.open(fileName) != CDB_OK) {
      Serial.print("Invalid CDB: ");
      Serial.println(fileName);
    } else {
      ADB_is_open = true;
    }
  }

  return ADB_is_open;
}

static bool nRF52_ADB_fini()
{
  if (ADB_is_open) {
    ucdb.close();
    ADB_is_open = false;
  }

  return !ADB_is_open;
}

/*
 * One aircraft CDB (20000+ records) query takes:
 * 1)     FOUND : 5-7 milliseconds
 * 2) NOT FOUND :   3 milliseconds
 */
static bool nRF52_ADB_query(uint8_t type, uint32_t id, char *buf, size_t size)
{
  char key[8];
  char out[64];
  uint8_t tokens[3] = { 0 };
  cdbResult rt;
  int c, i = 0, token_cnt = 0;
  bool rval = false;

  if (!ADB_is_open) {
    return rval;
  }

  snprintf(key, sizeof(key),"%06X", id);

  rt = ucdb.findKey(key, strlen(key));

  switch (rt) {
    case KEY_FOUND:
      while ((c = ucdb.readValue()) != -1 && i < (sizeof(out) - 1)) {
        if (c == '|') {
          if (token_cnt < (sizeof(tokens) - 1)) {
            token_cnt++;
            tokens[token_cnt] = i+1;
          }
          c = 0;
        }
        out[i++] = (char) c;
      }
      out[i] = 0;

      switch (ui->epdidpref)
      {
      case ID_TAIL:
        snprintf(buf, size, "CN: %s",
          strlen(out + tokens[2]) ? out + tokens[2] : "N/A");
        break;
      case ID_MAM:
        snprintf(buf, size, "%s",
          strlen(out + tokens[0]) ? out + tokens[0] : "Unknown");
        break;
      case ID_REG:
      default:
        snprintf(buf, size, "%s",
          strlen(out + tokens[1]) ? out + tokens[1] : "REG: N/A");
        break;
      }

      rval = true;
      break;

    case KEY_NOT_FOUND:
    default:
      break;
  }

  return rval;
}

DB_ops_t nRF52_ADB_ops = {
  nRF52_ADB_setup,
  nRF52_ADB_fini,
  nRF52_ADB_query
};

const SoC_ops_t nRF52_ops = {
  SOC_NRF52,
  "nRF52",
  nRF52_setup,
  nRF52_post_init,
  nRF52_loop,
  nRF52_fini,
  nRF52_reset,
  nRF52_getChipId,
  nRF52_getResetInfoPtr,
  nRF52_getResetInfo,
  nRF52_getResetReason,
  nRF52_getFreeHeap,
  nRF52_random,
  nRF52_Buzzer_test,
  nRF52_Buzzer_tone,
  NULL,
  nRF52_WiFi_set_param,
  nRF52_WiFi_transmit_UDP,
  NULL,
  NULL,
  NULL,
  nRF52_EEPROM_begin,
  //nRF52_EEPROM_extension,
  nRF52_SPI_begin,
  nRF52_swSer_begin,
  nRF52_swSer_enableRx,
  &nRF52_Bluetooth_ops,
  &nRF52_USBSerial_ops,
  NULL,
  nRF52_Display_setup,
  nRF52_Display_loop,
  nRF52_Display_fini,
#if 0
  nRF52_Display_lock,
  nRF52_Display_unlock,
#endif
  nRF52_Battery_setup,
  nRF52_Battery_param,
  nRF52_GNSS_PPS_Interrupt_handler,
  nRF52_get_PPS_TimeMarker,
  nRF52_Baro_setup,
  nRF52_UATSerial_begin,
  nRF52_UATModule_restart,
  nRF52_WDT_setup,
  nRF52_WDT_fini,
  nRF52_Button_setup,
  nRF52_Button_loop,
  nRF52_Button_fini,
  &nRF52_ADB_ops
};

#endif /* ARDUINO_ARCH_NRF52 */
