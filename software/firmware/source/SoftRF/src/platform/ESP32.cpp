/*
 * Platform_ESP32.cpp
 * Copyright (C) 2018-2022 Linar Yusupov
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
#if defined(ESP32)

#include "sdkconfig.h"

#include <SPI.h>
#include <esp_err.h>
#include <esp_wifi.h>

#if defined(CONFIG_IDF_TARGET_ESP32)
#include "esp_heap_caps.h"
#endif

#include "SPIFFS.h"

bool SPIFFS_is_mounted = false;

#if !defined(CONFIG_IDF_TARGET_ESP32S2)
#include <esp_bt.h>
#include <BLEDevice.h>
#endif /* CONFIG_IDF_TARGET_ESP32S2 */

#include <soc/rtc_cntl_reg.h>
#include <soc/efuse_reg.h>
#include <Wire.h>
#include <rom/rtc.h>
#include <rom/spi_flash.h>
#include <soc/adc_channel.h>
#include <flashchips.h>

//#include <axp20x.h>
//#define  XPOWERS_CHIP_AXP2102
//#include <XPowersLib.h>
// consolidated to a single AXP library, chip choice at runtime,
//    based on XPowersLibInterface_Example
#include "XPowersLibInterface.hpp"
#include "XPowersAXP2101.tpp"
#include "XPowersAXP192.tpp"

#include <pcf8563.h>

#include "../system/SoC.h"
#include "../system/Time.h"
#include "../driver/Buzzer.h"
#include "../driver/Strobe.h"
#include "../driver/Settings.h"
#include "../driver/RF.h"
#include "../driver/WiFi.h"
#include "../driver/Bluetooth.h"
#include "../driver/LED.h"
#include "../driver/Baro.h"
#include "../driver/Battery.h"
#include "../driver/OLED.h"
#include "../driver/GNSS.h"
#include "../driver/Filesys.h"
#include "../protocol/data/NMEA.h"
#include "../protocol/data/IGC.h"
#include "../protocol/data/GDL90.h"
#include "../protocol/data/D1090.h"

#if defined(USE_TFT)
#include <TFT_eSPI.h>
#endif /* USE_TFT */

#include <battery.h>

// SX12xx pin mapping
lmic_pinmap lmic_pins = {
    .nss  = SOC_GPIO_PIN_SS,
    .txe  = LMIC_UNUSED_PIN,
    .rxe  = LMIC_UNUSED_PIN,
    .rst  = SOC_GPIO_PIN_RST,
    .dio  = {LMIC_UNUSED_PIN, LMIC_UNUSED_PIN, LMIC_UNUSED_PIN},
    .busy = SOC_GPIO_PIN_TXE,
    .tcxo = LMIC_UNUSED_PIN,
};

WebServer server ( 80 );

#if defined(USE_NEOPIXELBUS_LIBRARY)
NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> strip(PIX_NUM, SOC_GPIO_PIN_LED);
#else /* USE_ADAFRUIT_NEO_LIBRARY */
// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIX_NUM, SOC_GPIO_PIN_LED,
                              NEO_GRB + NEO_KHZ800);
#endif /* USE_NEOPIXELBUS_LIBRARY */

#if defined(USE_OLED)
//U8X8_OLED_I2C_BUS_TYPE u8x8_ttgo  (TTGO_V2_OLED_PIN_RST);
#if defined(CONFIG_IDF_TARGET_ESP32S3)
U8X8_SSD1306_128X64_NONAME_HW_I2C     u8x8_ttgo (TTGO_V2_OLED_PIN_RST);
#else
// for T-Beam create two u8x8_ttgo objects, for Wire & Wire1
//    - default is Wire1 (GPIO 21,22)
U8X8_SSD1306_128X64_NONAME_2ND_HW_I2C
   u8x8_ttgo (TTGO_V2_OLED_PIN_RST, TTGO_V2_OLED_PIN_SCL, TTGO_V2_OLED_PIN_SDA);      // GPIO 22,21
U8X8_SSD1306_128X64_NONAME_HW_I2C
   u8x8_ttgo2(TTGO_V2_OLED_PIN_RST, SOC_GPIO_PIN_TBEAM_SCL, SOC_GPIO_PIN_TBEAM_SDA);  // GPIO 2,13
#endif /* CONFIG_IDF_TARGET_ESP32S3 */
U8X8_OLED_I2C_BUS_TYPE u8x8_heltec(HELTEC_OLED_PIN_RST);
U8X8_SH1106_128X64_NONAME_HW_I2C u8x8_1_3(U8X8_PIN_NONE);
#endif /* USE_OLED */

#if defined(USE_TFT)
static TFT_eSPI *tft = NULL;

void TFT_off()
{
    tft->writecommand(TFT_DISPOFF);
    tft->writecommand(TFT_SLPIN);
}

void TFT_backlight_adjust(uint8_t level)
{
    ledcWrite(BACKLIGHT_CHANNEL, level);
}

bool TFT_isBacklightOn()
{
    return (bool)ledcRead(BACKLIGHT_CHANNEL);
}

void TFT_backlight_off()
{
    ledcWrite(BACKLIGHT_CHANNEL, 0);
}

void TFT_backlight_on()
{
    ledcWrite(BACKLIGHT_CHANNEL, 250);
}
#endif /* USE_TFT */

//AXP20X_Class axp_xxx;
//XPowersPMU   axp_2xxx;
XPowersLibInterface *PMU = NULL;

uint32_t BlueLEDTimeMarker = 5000;

// static bool bt_turned_off = false;

static int esp32_board = ESP32_DEVKIT; /* default */
static size_t ESP32_Min_AppPart_Size = 0;

static portMUX_TYPE GNSS_PPS_mutex = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE PMU_mutex      = portMUX_INITIALIZER_UNLOCKED;
volatile bool PMU_Irq = false;

static union {
  uint8_t efuse_mac[6];
  uint64_t chipmacid;
};

static bool TFT_display_frontpage = false;
static uint32_t prev_tx_packets_counter = 0;
static uint32_t prev_rx_packets_counter = 0;
extern uint32_t tx_packets_counter, rx_packets_counter;
extern bool loopTaskWDTEnabled;

const char *ESP32S2_Device_Manufacturer = SOFTRF_IDENT;
const char *ESP32S2_Device_Model = "Standalone Edition"; /* 303a:8132 */
const char *ESP32S3_Device_Model = "Prime Edition Mk.3"; /* 303a:8133 */
const uint16_t ESP32S2_Device_Version = SOFTRF_USB_FW_VERSION;

#if defined(EXCLUDE_WIFI)
// Dummy definition to satisfy build sequence
char UDPpacketBuffer[UDP_PACKET_BUFSIZE];
#endif /* EXCLUDE_WIFI */

#if defined(CONFIG_IDF_TARGET_ESP32S3)
#include <Adafruit_SPIFlash.h>
#include "../driver/EPD.h"
#include "uCDB.hpp"

SPIClass uSD_SPI(HSPI);
SdFat    uSD(&uSD_SPI);

static bool uSD_is_mounted = false;
bool button_pressed = false;

Adafruit_FlashTransport_ESP32 HWFlashTransport;
Adafruit_SPIFlash QSPIFlash(&HWFlashTransport);

static Adafruit_SPIFlash *SPIFlash = &QSPIFlash;

/// Flash device list count
enum {
  EXTERNAL_FLASH_DEVICE_COUNT
};

/// List of all possible flash devices used by ESP32 boards
static SPIFlash_Device_t possible_devices[] = { };

static bool ESP32_has_spiflash  = false;
static uint32_t spiflash_id     = 0;
static bool FATFS_is_mounted    = false;
static bool ADB_is_open         = false;

#if CONFIG_TINYUSB_MSC_ENABLED
  #if defined(USE_ADAFRUIT_MSC)
    #include "Adafruit_TinyUSB.h"

    // USB Mass Storage object
    Adafruit_USBD_MSC usb_msc;
  #else
    #include "USBMSC.h"

    // USB Mass Storage object
    USBMSC usb_msc;
  #endif /* USE_ADAFRUIT_MSC */
#endif /* CONFIG_TINYUSB_MSC_ENABLED */

// file system object from SdFat
FatFileSystem fatfs;

ui_settings_t ui_settings;
/* = {
    .units        = UNITS_METRIC,
    .zoom         = ZOOM_MEDIUM,
    .protocol     = PROTOCOL_NMEA,
    .rotate       = ROTATE_0,
    .orientation  = DIRECTION_TRACK_UP,
    .adb          = DB_NONE,
    .epdidpref    = ID_TYPE,
    .vmode        = VIEW_MODE_STATUS,
    .voice        = VOICE_OFF,
    .aghost       = ANTI_GHOSTING_OFF,
    .filter       = TRAFFIC_FILTER_OFF,
    .team         = 0
}; */

ui_settings_t *ui;
uCDB<FatFileSystem, File> ucdb(fatfs);

#if CONFIG_TINYUSB_MSC_ENABLED
#if defined(USE_ADAFRUIT_MSC)
// Callback invoked when received READ10 command.
// Copy disk's data to buffer (up to bufsize) and
// return number of copied bytes (must be multiple of block size)
static int32_t ESP32_msc_read_cb (uint32_t lba, void* buffer, uint32_t bufsize)
{
  // Note: SPIFLash Bock API: readBlocks/writeBlocks/syncBlocks
  // already include 4K sector caching internally. We don't need to cache it, yahhhh!!
  return SPIFlash->readBlocks(lba, (uint8_t*) buffer, bufsize/512) ? bufsize : -1;
}

// Callback invoked when received WRITE10 command.
// Process data in buffer to disk's storage and
// return number of written bytes (must be multiple of block size)
static int32_t ESP32_msc_write_cb (uint32_t lba, uint8_t* buffer, uint32_t bufsize)
{
  // Note: SPIFLash Bock API: readBlocks/writeBlocks/syncBlocks
  // already include 4K sector caching internally. We don't need to cache it, yahhhh!!
  return SPIFlash->writeBlocks(lba, buffer, bufsize/512) ? bufsize : -1;
}

// Callback invoked when WRITE10 command is completed (status received and accepted by host).
// used to flush any pending cache.
static void ESP32_msc_flush_cb (void)
{
  // sync with flash
  SPIFlash->syncBlocks();

  // clear file system's cache to force refresh
  fatfs.cacheClear();
}

#else

// Callback invoked when received READ10 command.
// Copy disk's data to buffer (up to bufsize) and
// return number of copied bytes (must be multiple of block size)
static int32_t ESP32_msc_read_cb (uint32_t lba, uint32_t offset, void* buffer, uint32_t bufsize)
{
  // Note: SPIFLash Bock API: readBlocks/writeBlocks/syncBlocks
  // already include 4K sector caching internally. We don't need to cache it, yahhhh!!
  return SPIFlash->readBlocks(lba, offset, (uint8_t*) buffer, bufsize/512) ?
         bufsize : -1;
}

// Callback invoked when received WRITE10 command.
// Process data in buffer to disk's storage and
// return number of written bytes (must be multiple of block size)
static int32_t ESP32_msc_write_cb (uint32_t lba, uint32_t offset, uint8_t* buffer, uint32_t bufsize)
{
  // Note: SPIFLash Bock API: readBlocks/writeBlocks/syncBlocks
  // already include 4K sector caching internally. We don't need to cache it, yahhhh!!
  int32_t rval = SPIFlash->writeBlocks(lba, offset, buffer, bufsize/512) ?
                 bufsize : -1;

#if 1
  // sync with flash
  SPIFlash->syncBlocks();

  // clear file system's cache to force refresh
  fatfs.cacheClear();
#endif

  return rval;
}
#endif /* USE_ADAFRUIT_MSC */
#endif /* CONFIG_TINYUSB_MSC_ENABLED */

#include "SensorQMC6310.hpp"
SensorQMC6310 mag;

#if !defined(EXCLUDE_IMU)
#include "SensorQMI8658.hpp"
SensorQMI8658 imu;
#endif /* EXCLUDE_IMU */
#endif /* CONFIG_IDF_TARGET_ESP32S3 */

#if defined(ENABLE_D1090_INPUT)
#include <mode-s.h>

mode_s_t state;
#endif /* ENABLE_D1090_INPUT */

static void IRAM_ATTR ESP32_PMU_Interrupt_handler() {
  portENTER_CRITICAL_ISR(&PMU_mutex);
  PMU_Irq = true;
  portEXIT_CRITICAL_ISR(&PMU_mutex);
}

static uint32_t ESP32_getFlashId()
{
  return g_rom_flashchip.device_id;
}

#if defined(CORE_DEBUG_LEVEL) && CORE_DEBUG_LEVEL>0 && !defined(TAG)
#define TAG "MAC"
#endif


// GPIO Pin Reservation System
// Rreturns false (and reserves the pin) if OK, returns true if pin not available
// Called with null label before Serial is started
static uint64_t pinmap = 0;
bool ESP32_pin_reserved(uint8_t pin, bool shared, const char *label)
{
    if (pin == SOC_UNUSED_PIN)
        return false;
    if (shared) {
        if (label)  Serial.printf("%s sharing pin %d\r\n", label, pin);   // just for info
        return false;
    }
    uint64_t pinmask = ((uint64_t)1 << pin);
    if (pinmap & pinmask) {
        if (label)  Serial.printf("Pin %d already used, not available for %s\r\n", pin, label);
        return true;
    }
    pinmap |= pinmask;
    if (label)  Serial.printf("Pin %d now reserved for %s\r\n", pin, label);
    return false;
}
#if 0
void ESP32_pin_unreserve(uint8_t pin)
{
    uint64_t pinmask = ((uint64_t)1 << pin);
    pinmap &= (~pinmask);
    Serial.printf("Pin %d now un-reserved\r\n", pin);
}
#endif
void list_reserved_pins()
{
    Serial.print("Pins reserved so far:");
    for (int pin=0; pin<64; pin++) {
        uint64_t pinmask = ((uint64_t)1 << pin);
        if (pinmap & pinmask) {
            Serial.print(" ");
            Serial.print(pin);
        }
    }
    Serial.println("");
}

//int Wire_Trans_rval;  // not used, remove

gpio_num_t middle_button_pin = (gpio_num_t) SOC_UNUSED_PIN;


static void print_dest(int dest)
{
  switch (dest)
  {
    case DEST_UART       :  Serial.println(F("UART"));      break;
    case DEST_UART2      :  Serial.println(F("UART2"));     break;
    case DEST_USB        :  Serial.println(F("USB CDC"));   break;
    case DEST_UDP        :  Serial.println(F("UDP"));       break;
    case DEST_TCP        :  Serial.println(F("TCP"));       break;
    case DEST_BLUETOOTH  :  Serial.println(F("Bluetooth")); break;
    case DEST_NONE       :
    default              :  Serial.println(F("NULL"));      break;
  }
}

bool has_axp = false;
bool has_axp202 = false;
bool has_axp192 = false;
bool has_axp2101 = false;

static bool PMU_probe() {

  if (!PMU) {
      PMU = new XPowersAXP2101(Wire1);
      if (PMU->init()) {
          Serial.println("AXP2101 PMU init succeeded");
          has_axp2101 = true;
          return true;
      }
      Serial.println("Failed to find AXP2101 PMU");
      delete PMU;
      PMU = NULL;
  }

  if (hw_info.model == SOFTRF_MODEL_PRIME_MK2) {
    if (!PMU) {
        PMU = new XPowersAXP192(Wire1);
        if (PMU->init()) {
            Serial.println("AXP192 PMU init succeeded");
            has_axp192 = true;
            return true;
        }
        Serial.println("Failed to find AXP192 PMU");
        delete PMU;
        PMU = NULL;
    }
  }

  return false;
}

static void ESP32_setup()
{
#if !defined(SOFTRF_ADDRESS)

  esp_err_t ret = ESP_OK;
  uint8_t null_mac[6] = {0};

  ret = esp_efuse_mac_get_custom(efuse_mac);
  if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Get base MAC address from BLK3 of EFUSE error (%s)", esp_err_to_name(ret));
    /* If get custom base MAC address error, the application developer can decide what to do:
     * abort or use the default base MAC address which is stored in BLK0 of EFUSE by doing
     * nothing.
     */

    ESP_LOGI(TAG, "Use base MAC address which is stored in BLK0 of EFUSE");
    chipmacid = ESP.getEfuseMac();
  } else {
    if (memcmp(efuse_mac, null_mac, 6) == 0) {
      ESP_LOGI(TAG, "Use base MAC address which is stored in BLK0 of EFUSE");
      chipmacid = ESP.getEfuseMac();
    }
  }
#endif /* SOFTRF_ADDRESS */

#if ESP32_DISABLE_BROWNOUT_DETECTOR
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
#endif

  size_t flash_size = spi_flash_get_chip_size();
  size_t min_app_size = flash_size;

  esp_partition_iterator_t it;
  const esp_partition_t *part;

//ESP_LOGI(TAG, "checking partitions...");

  it = esp_partition_find(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_ANY, NULL);
  if (it) {
    do {
      part = esp_partition_get(it);
      if (part->subtype == ESP_PARTITION_SUBTYPE_APP_FACTORY) {
        continue;
      }
      if (part->size < min_app_size) {
        min_app_size = part->size;
      }
    } while (it = esp_partition_next(it));

    if (it) esp_partition_iterator_release(it);
  }

  if (min_app_size && (min_app_size != flash_size)) {
    ESP32_Min_AppPart_Size = min_app_size;
  }

//ESP_LOGI(TAG, "checking for PSRAM...");

  if (psramFound()) {

    uint32_t flash_id = ESP32_getFlashId();

    /*
     *    Board         |   Module   |  Flash memory IC
     *  ----------------+------------+--------------------
     *  DoIt ESP32      | WROOM      | GIGADEVICE_GD25Q32
     *  TTGO T3  V2.0   | PICO-D4 IC | GIGADEVICE_GD25Q32
     *  TTGO T3  V2.1.6 | PICO-D4 IC | GIGADEVICE_GD25Q32
     *  TTGO T22 V06    |            | WINBOND_NEX_W25Q32_V
     *  TTGO T22 V08    |            | WINBOND_NEX_W25Q32_V
     *  TTGO T22 V11    |            | BOYA_BY25Q32AL
     *  TTGO T8  V1.8   | WROVER     | GIGADEVICE_GD25LQ32
     *  TTGO T8 S2 V1.1 |            | WINBOND_NEX_W25Q32_V
     *  TTGO T5S V1.9   |            | WINBOND_NEX_W25Q32_V
     *  TTGO T5S V2.8   |            | BOYA_BY25Q32AL
     *  TTGO T5  4.7    | WROVER-E   | XMC_XM25QH128C
     *  TTGO T-Watch    |            | WINBOND_NEX_W25Q128_V
     *  Ai-T NodeMCU-S3 | ESP-S3-12K | GIGADEVICE_GD25Q64C
     *  TTGO T-Dongle   |            | BOYA_BY25Q32AL
     *  TTGO S3 Core    |            | GIGADEVICE_GD25Q64C
     */

    switch(flash_id)
    {
    case MakeFlashId(GIGADEVICE_ID, GIGADEVICE_GD25LQ32):
      /* ESP32-WROVER module with ESP32-NODEMCU-ADAPTER */
      hw_info.model = SOFTRF_MODEL_STANDALONE;
      break;
    case MakeFlashId(WINBOND_NEX_ID, WINBOND_NEX_W25Q128_V):
      hw_info.model = SOFTRF_MODEL_SKYWATCH;
      esp32_board = ESP32_TTGO_T_WATCH;
      //Serial.println("Warning: T-watch AXP202 not supported");
      break;
#if defined(CONFIG_IDF_TARGET_ESP32)
    case MakeFlashId(WINBOND_NEX_ID, WINBOND_NEX_W25Q32_V):
    case MakeFlashId(BOYA_ID, BOYA_BY25Q32AL):
    default:
      hw_info.model = SOFTRF_MODEL_PRIME_MK2;
      heap_caps_malloc_extmem_enable(1024);    // <<< try and make libraries use less RAM
#elif defined(CONFIG_IDF_TARGET_ESP32S2)
    default:
      esp32_board   = ESP32_S2_T8_V1_1;
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
    case MakeFlashId(GIGADEVICE_ID, GIGADEVICE_GD25Q64):
    default:
      hw_info.model = SOFTRF_MODEL_PRIME_MK3;
#else
#error "This ESP32 family build variant is not supported!"
#endif
      break;
    }
  } else {
#if defined(CONFIG_IDF_TARGET_ESP32)
    uint32_t chip_ver = REG_GET_FIELD(EFUSE_BLK0_RDATA3_REG, EFUSE_RD_CHIP_VER_PKG);
    uint32_t pkg_ver  = chip_ver & 0x7;
    if (pkg_ver == EFUSE_RD_CHIP_VER_PKG_ESP32PICOD4) {
      esp32_board    = ESP32_TTGO_V2_OLED;
      lmic_pins.rst  = SOC_GPIO_PIN_TBEAM_RF_RST_V05;
      lmic_pins.busy = SOC_GPIO_PIN_TBEAM_RF_BUSY_V08;
    }
#elif defined(CONFIG_IDF_TARGET_ESP32S2)
    esp32_board      = ESP32_S2_T8_V1_1;
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
    esp32_board      = ESP32_S3_DEVKIT;
#endif /* CONFIG_IDF_TARGET_ESP32 */
  }

#if 0   /* we use ToneAC (or external active buzzer), not LEDC */
  if (SOC_GPIO_PIN_BUZZER != SOC_UNUSED_PIN) {
    ledcAttachPin(SOC_GPIO_PIN_BUZZER, LEDC_CHANNEL_BUZZER);
    ledcSetup(LEDC_CHANNEL_BUZZER, 0, LEDC_RESOLUTION_BUZZER);
  }
#endif

//ESP_LOGI(TAG, "buzzer & strobe pins low...");

  if (hw_info.model == SOFTRF_MODEL_PRIME_MK2) {
    esp32_board = ESP32_TTGO_T_BEAM;

    // prevent external buzzer being "on" for a while at boot
    // - do this as early as possible - even though board rev is not known yet
    if (SOC_GPIO_PIN_BUZZER != SOC_UNUSED_PIN)
        pinMode(SOC_GPIO_PIN_BUZZER, INPUT_PULLDOWN);    // for T-Beam v1.x
    if (SOC_GPIO_PIN_VOICE != SOC_UNUSED_PIN)
        pinMode(SOC_GPIO_PIN_VOICE, INPUT_PULLDOWN);     // for T-Beam v0.7
    // similarly for strobe
    if (SOC_GPIO_PIN_STROBE != SOC_UNUSED_PIN) {
        //pinMode(STROBEPIN, OUTPUT);
        //digitalWrite(STROBEPIN, LOW);
        pinMode(SOC_GPIO_PIN_STROBE, INPUT_PULLDOWN);
    }

//ESP_LOGI(TAG, "starting wires...");

    // Start up both Wires on fixed GPIO pins
    // Do not change these later (on PRIME_MK2)
    // Baro probe and OLED probe try both wires
    // PMU probe needs Wire1
    Wire1.begin(TTGO_V2_OLED_PIN_SDA  , TTGO_V2_OLED_PIN_SCL);      // GPIO 21,22
    ESP32_pin_reserved(TTGO_V2_OLED_PIN_SDA, false, (const char *) NULL);
    ESP32_pin_reserved(TTGO_V2_OLED_PIN_SCL, false, (const char *) NULL);
    //if (settings->gnss_pins != EXT_GNSS_13_2)     // <<< cannot access settings yet
    //    Wire.begin(SOC_GPIO_PIN_TBEAM_SDA, SOC_GPIO_PIN_TBEAM_SCL);    // GPIO 13,2

//ESP_LOGI(TAG, "PMU probe...");

    has_axp = PMU_probe();

    if (has_axp192) {

        hw_info.revision = 8;
        hw_info.pmu = PMU_AXP192;

        PMU->setChargingLedMode(XPOWERS_CHG_LED_ON);

        PMU->setPowerChannelVoltage(XPOWERS_LDO2, 3300);  // LoRa
        PMU->enablePowerOutput(XPOWERS_LDO2);

        PMU->setPowerChannelVoltage(XPOWERS_DCDC1, 3300); // OLED
        PMU->enablePowerOutput(XPOWERS_DCDC1);

        PMU->setPowerChannelVoltage(XPOWERS_LDO3, 3300);  // GPS
        PMU->enablePowerOutput(XPOWERS_LDO3);

        //protected oled power source
        PMU->setProtectedChannel(XPOWERS_DCDC1);
        //protected esp32 power source
        PMU->setProtectedChannel(XPOWERS_DCDC3);

        //disable not use channel
        PMU->disablePowerOutput(XPOWERS_DCDC2);

        PMU->setChargerConstantCurr(XPOWERS_AXP192_CHG_CUR_550MA);
        PMU->setChargeTargetVoltage(XPOWERS_AXP192_CHG_VOL_4V2);

        PMU->disableTSPinMeasure();
        PMU->enableBattDetection();
        PMU->enableVbusVoltageMeasure();
        PMU->enableBattVoltageMeasure();
        PMU->enableSystemVoltageMeasure();

        PMU->setSysPowerDownVoltage(2800);

        //PMU->enablePowerOutput(XPOWERS_EXTEN);

        pinMode(SOC_GPIO_PIN_TBEAM_V08_PMU_IRQ, INPUT /* INPUT_PULLUP */);     // GPIO pin 35
        attachInterrupt(digitalPinToInterrupt(SOC_GPIO_PIN_TBEAM_V08_PMU_IRQ),
                        ESP32_PMU_Interrupt_handler, FALLING);
        PMU->disableIRQ(XPOWERS_AXP192_ALL_IRQ);
        PMU->enableIRQ(XPOWERS_AXP192_PKEY_LONG_IRQ |
                       XPOWERS_AXP192_PKEY_SHORT_IRQ
                      );
        PMU->clearIrqStatus();

    } else if (has_axp2101) {

        // Set the minimum common working voltage of the PMU VBUS input,
        // below this value will turn off the PMU
        //PMU->setVbusVoltageLimit(XPOWERS_AXP2101_VBUS_VOL_LIM_4V36);

        // Set the maximum current of the PMU VBUS input,
        // higher than this value will turn off the PMU
        PMU->setVbusCurrentLimit(XPOWERS_AXP2101_VBUS_CUR_LIM_1500MA);

        // DCDC1 1500~3400mV, IMAX=2A
        PMU->setProtectedChannel(XPOWERS_DCDC1);
        PMU->setPowerChannelVoltage(XPOWERS_DCDC1, 3300);
        PMU->enablePowerOutput(XPOWERS_DCDC1);

        // not clear whether esp32 power source on T-Beam v1.2 is DCDC1 or DCDC3?
        PMU->setProtectedChannel(XPOWERS_DCDC3);
        PMU->setPowerChannelVoltage(XPOWERS_DCDC3, 3300);
        PMU->enablePowerOutput(XPOWERS_DCDC3);

        PMU->disablePowerOutput(XPOWERS_DCDC2);
        PMU->disablePowerOutput(XPOWERS_DCDC4);
        PMU->disablePowerOutput(XPOWERS_DCDC5);
        PMU->disablePowerOutput(XPOWERS_ALDO4);
        PMU->disablePowerOutput(XPOWERS_BLDO1);
        PMU->disablePowerOutput(XPOWERS_BLDO2);
        PMU->disablePowerOutput(XPOWERS_DLDO1);
        PMU->disablePowerOutput(XPOWERS_DLDO2);

        // ALDO 500~3500mV, 100mV/step, IMAX=300mA
        //PMU->setButtonBatteryChargeVoltage(3100); // GNSS battery - old syntax

        // PMU->enableDC1();
        //PMU->enableButtonBatteryCharge();    // <<< this old syntax not available via this API

        // new syntax: GNSS RTC PowerVDD
        PMU->setPowerChannelVoltage(XPOWERS_VBACKUP, 3300);
        PMU->enablePowerOutput(XPOWERS_VBACKUP);

        PMU->enablePowerOutput(XPOWERS_ALDO1);

        PMU->setPowerChannelVoltage(XPOWERS_ALDO2, 3300); // LoRa, AXP2101 power-on value: 2800
        PMU->enablePowerOutput(XPOWERS_ALDO2);

        PMU->setPowerChannelVoltage(XPOWERS_ALDO3, 3300); // GPS,  AXP2101 power-on value: 3300
        PMU->enablePowerOutput(XPOWERS_ALDO3);

        //PMU->enableALDO2();    // <<< this syntax not available via this API
        //PMU->enableALDO3();

        PMU->setChargeTargetVoltage(XPOWERS_AXP2101_CHG_VOL_4V2);

        PMU->setChargingLedMode(XPOWERS_CHG_LED_ON);

        pinMode(SOC_GPIO_PIN_TBEAM_V08_PMU_IRQ, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(SOC_GPIO_PIN_TBEAM_V08_PMU_IRQ),
                        ESP32_PMU_Interrupt_handler, FALLING);
        PMU->disableIRQ(XPOWERS_AXP2101_ALL_IRQ);

        PMU->setChargerConstantCurr(XPOWERS_AXP2101_CHG_CUR_500MA);
        PMU->disableTSPinMeasure();
        PMU->enableBattDetection();
        PMU->enableVbusVoltageMeasure();
        PMU->enableBattVoltageMeasure();
        PMU->enableSystemVoltageMeasure();

        PMU->setSysPowerDownVoltage(2800);

        PMU->enableIRQ(XPOWERS_AXP2101_PKEY_LONG_IRQ | XPOWERS_AXP2101_PKEY_SHORT_IRQ);
        PMU->clearIrqStatus();

        hw_info.revision = 12;
        hw_info.pmu = PMU_AXP2101;

    } else {

        // no PMU
        hw_info.revision = 2;
        if (RF_SX12XX_RST_is_connected)
          hw_info.revision = 5;

        // see if restarting Wire1 will help OLED probe on no-PMU board:
        WIRE_FINI(Wire1);
        Wire1.begin(TTGO_V2_OLED_PIN_SDA, TTGO_V2_OLED_PIN_SCL);
        // >>> exercise the I2C bus
        Wire1.beginTransmission(SSD1306_OLED_I2C_ADDR);
        Wire1.endTransmission();
    }

    lmic_pins.rst  = SOC_GPIO_PIN_TBEAM_RF_RST_V05;
    lmic_pins.busy = SOC_GPIO_PIN_TBEAM_RF_BUSY_V08;

    /* use middle button on T-Beam v1.x to turn off transmissions in winch mode */
    if (hw_info.model == SOFTRF_MODEL_PRIME_MK2) {
        if (hw_info.revision == 8 || hw_info.revision == 12) {
            middle_button_pin = (gpio_num_t) SOC_GPIO_PIN_TBEAM_V08_BUTTON;
            pinMode(middle_button_pin, INPUT);
            //gpio_pullup_en(middle_button_pin);
            // reservation done in button_setup():
            //ESP32_pin_reserved(SOC_GPIO_PIN_TBEAM_V08_BUTTON, false, (const char *) NULL);
        //} else {   // v0.7
            //ESP32_pin_reserved(SOC_GPIO_PIN_TBEAM_V05_BUTTON, false, (const char *) NULL);
        }
    }

#if defined(CONFIG_IDF_TARGET_ESP32S2)
  } else if (esp32_board == ESP32_S2_T8_V1_1) {
    lmic_pins.nss  = SOC_GPIO_PIN_T8_S2_LORA_SS;
    lmic_pins.rst  = SOC_GPIO_PIN_T8_S2_LORA_RST;
    lmic_pins.busy = LMIC_UNUSED_PIN;

    pinMode(SOC_GPIO_PIN_T8_S2_PWR_EN, INPUT_PULLUP);

#if defined(USE_USB_HOST)
    Serial.end();
    Serial.begin(SERIAL_OUT_BR, SERIAL_IN_BITS,
                 SOC_GPIO_PIN_T8_S2_CONS_RX, SOC_GPIO_PIN_T8_S2_CONS_TX);
#endif /* USE_USB_HOST */

#endif /* CONFIG_IDF_TARGET_ESP32S2 */

#if defined(CONFIG_IDF_TARGET_ESP32S3)
  } else if (hw_info.model == SOFTRF_MODEL_PRIME_MK3 ||
             esp32_board   == ESP32_S3_DEVKIT) {
    Wire1.begin(SOC_GPIO_PIN_S3_PMU_SDA , SOC_GPIO_PIN_S3_PMU_SCL);
    Wire1.beginTransmission(AXP2101_SLAVE_ADDRESS);

    has_axp = PMU_probe();

    if (has_axp2101) {
      esp32_board   = ESP32_TTGO_T_BEAM_SUPREME;
      hw_info.model = SOFTRF_MODEL_PRIME_MK3; /* allow psramFound() to fail */
      hw_info.pmu   = PMU_AXP2101;

      /* inactivate tinyUF2 LED output setting */
      pinMode(SOC_GPIO_PIN_S3_GNSS_PPS, INPUT);

      // Set the minimum common working voltage of the PMU VBUS input,
      // below this value will turn off the PMU
      PMU->setVbusVoltageLimit(XPOWERS_AXP2101_VBUS_VOL_LIM_4V36);

      // Set the maximum current of the PMU VBUS input,
      // higher than this value will turn off the PMU
      PMU->setVbusCurrentLimit(XPOWERS_AXP2101_VBUS_CUR_LIM_1500MA);

      // DCDC1 1500~3400mV, IMAX=2A
      PMU->setDC1Voltage(3300);

      // DCDC5 1400~3700mV, 100mV/step, 24 steps, IMAX=1A
      PMU->setDC5Voltage(3700);

      // ALDO 500~3500V, 100mV/step, IMAX=300mA
      PMU->setALDO3Voltage(3300); // LoRa, AXP2101 power-on value: 3300
      PMU->setALDO4Voltage(3300); // GNSS, AXP2101 power-on value: 2900

      PMU->setALDO2Voltage(3300); // RTC
      PMU->setALDO1Voltage(3300); // sensors, OLED
      PMU->setBLDO1Voltage(3300); // uSD

      // PMU->enableDC1();
      PMU->enableDC5();

      PMU->enableALDO3();
      PMU->enableALDO4();

      PMU->enableALDO2();
      PMU->enableALDO1();
      PMU->enableBLDO1();

      PMU->setChargingLedMode(XPOWERS_CHG_LED_ON);

      pinMode(SOC_GPIO_PIN_S3_PMU_IRQ, INPUT /* INPUT_PULLUP */);

      attachInterrupt(digitalPinToInterrupt(SOC_GPIO_PIN_S3_PMU_IRQ),
                      ESP32_PMU_Interrupt_handler, FALLING);

      PMU->disableIRQ(XPOWERS_AXP2101_ALL_IRQ);
      PMU->clearIrqStatus();

      PMU->setChargerConstantCurr(XPOWERS_AXP2101_CHG_CUR_500MA);
      PMU->disableTSPinMeasure();

      PMU->enableBattDetection();
      PMU->enableVbusVoltageMeasure();
      PMU->enableBattVoltageMeasure();
      PMU->enableSystemVoltageMeasure();

      PMU->enableIRQ(XPOWERS_AXP2101_PKEY_LONG_IRQ |
                         XPOWERS_AXP2101_PKEY_SHORT_IRQ);

      /* Wake up Quectel L76K GNSS */
      digitalWrite(SOC_GPIO_PIN_S3_GNSS_WAKE, HIGH);
      pinMode(SOC_GPIO_PIN_S3_GNSS_WAKE, OUTPUT);

      Wire1.beginTransmission(PCF8563_SLAVE_ADDRESS);
      if (Wire1.endTransmission() == 0) {
        hw_info.rtc = RTC_PCF8563;
      }

      /* wait until every LDO voltage will settle down */
      delay(200);

#if 0
      Wire.begin(SOC_GPIO_PIN_S3_SDA, SOC_GPIO_PIN_S3_SCL);
      Wire.beginTransmission(QMC6310_SLAVE_ADDRESS);
      if (Wire.endTransmission() == 0) {
        hw_info.mag = MAG_QMC6310;
      }
      WIRE_FINI(Wire);
#else
      bool has_qmc = mag.begin(Wire, QMC6310_SLAVE_ADDRESS,
                               SOC_GPIO_PIN_S3_SDA, SOC_GPIO_PIN_S3_SCL);
      mag.deinit(); WIRE_FINI(Wire);
      hw_info.mag  = has_qmc ? MAG_QMC6310 : hw_info.mag;
#endif

#if !defined(EXCLUDE_IMU)
#if 0
      uSD_SPI.begin(SOC_GPIO_PIN_S3_IMU_SCK,
                    SOC_GPIO_PIN_S3_IMU_MISO,
                    SOC_GPIO_PIN_S3_IMU_MOSI,
                    SOC_GPIO_PIN_S3_IMU_SS);
      uSD_SPI.setHwCs(false);

      pinMode(SOC_GPIO_PIN_S3_IMU_SS, OUTPUT);
      digitalWrite(SOC_GPIO_PIN_S3_IMU_SS, HIGH);

      delay(50);

      uSD_SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
      digitalWrite(SOC_GPIO_PIN_S3_IMU_SS, LOW);

      // reset device
      uSD_SPI.transfer(QMI8658_REG_RESET);
      uSD_SPI.transfer(0xB0);

      digitalWrite(SOC_GPIO_PIN_S3_IMU_SS, HIGH);
      uSD_SPI.endTransaction();

      delay(50);

      uSD_SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
      digitalWrite(SOC_GPIO_PIN_S3_IMU_SS, LOW);

      uSD_SPI.transfer(QMI8658_REG_WHOAMI | 0x80 /* read */);
      hw_info.imu = (uSD_SPI.transfer(0x00) == 0x5) ? IMU_QMI8658 : IMU_NONE;

      digitalWrite(SOC_GPIO_PIN_S3_IMU_SS, HIGH);
      uSD_SPI.endTransaction();
#else
      bool has_qmi = imu.begin(SOC_GPIO_PIN_S3_IMU_SS,
                               SOC_GPIO_PIN_S3_IMU_MOSI,
                               SOC_GPIO_PIN_S3_IMU_MISO,
                               SOC_GPIO_PIN_S3_IMU_SCK,
                               uSD_SPI);
      imu.deinit();
      hw_info.imu  = has_qmi ? IMU_QMI8658 : hw_info.imu;
#endif

      if (hw_info.imu == IMU_NONE) {
        uSD_SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
        digitalWrite(SOC_GPIO_PIN_S3_IMU_SS, LOW);

        // reset device
        uSD_SPI.transfer(MPU6886_REG_PWR_MGMT_1);
        uSD_SPI.transfer(0x80);

        digitalWrite(SOC_GPIO_PIN_S3_IMU_SS, HIGH);
        uSD_SPI.endTransaction();

        delay(100);

        uSD_SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
        digitalWrite(SOC_GPIO_PIN_S3_IMU_SS, LOW);

        uSD_SPI.transfer(MPU6886_REG_WHOAMI | 0x80 /* read */);
        hw_info.imu = (uSD_SPI.transfer(0x00) ==  0x19) ? IMU_MPU6886 : IMU_NONE;

        digitalWrite(SOC_GPIO_PIN_S3_IMU_SS, HIGH);
        uSD_SPI.endTransaction();
      }

      uSD_SPI.end();
#endif /* EXCLUDE_IMU */
    } else {
      WIRE_FINI(Wire1);
      esp32_board      = ESP32_S3_DEVKIT;
      hw_info.model    = SOFTRF_MODEL_STANDALONE;
      hw_info.revision = 203;

#if !defined(EXCLUDE_IMU)
      uSD_SPI.begin(SOC_GPIO_PIN_S3_IMU_SCK,
                    SOC_GPIO_PIN_S3_IMU_MISO,
                    SOC_GPIO_PIN_S3_IMU_MOSI,
                    SOC_GPIO_PIN_S3_IMU_SS);
      uSD_SPI.setHwCs(false);

      pinMode(SOC_GPIO_PIN_S3_IMU_SS, OUTPUT);
      digitalWrite(SOC_GPIO_PIN_S3_IMU_SS, HIGH);

      delay(50);

      uSD_SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
      digitalWrite(SOC_GPIO_PIN_S3_IMU_SS, LOW);

      // reset device
      uSD_SPI.transfer(MPU9250_REG_PWR_MGMT_1);
      uSD_SPI.transfer(0x80);

      digitalWrite(SOC_GPIO_PIN_S3_IMU_SS, HIGH);
      uSD_SPI.endTransaction();

      delay(100);

      uSD_SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
      digitalWrite(SOC_GPIO_PIN_S3_IMU_SS, LOW);

      uSD_SPI.transfer(MPU9250_REG_WHOAMI | 0x80 /* read */);
      uint8_t whoami = uSD_SPI.transfer(0x00);
      hw_info.imu = (whoami == 0x71 || whoami == 0x73) ? IMU_MPU9250 : IMU_NONE;

      digitalWrite(SOC_GPIO_PIN_S3_IMU_SS, HIGH);
      uSD_SPI.endTransaction();

      uSD_SPI.end();

      hw_info.mag = (hw_info.imu == IMU_MPU9250) ? MAG_AK8963 : hw_info.mag;
#endif /* EXCLUDE_IMU */
    }

#if ARDUINO_USB_CDC_ON_BOOT
    SerialOutput.begin(SERIAL_OUT_BR, SERIAL_OUT_BITS,
                       SOC_GPIO_PIN_S3_CONS_RX,
                       SOC_GPIO_PIN_S3_CONS_TX);
#endif /* ARDUINO_USB_CDC_ON_BOOT */

    lmic_pins.nss  = SOC_GPIO_PIN_S3_SS;
    lmic_pins.rst  = SOC_GPIO_PIN_S3_RST;
    lmic_pins.busy = SOC_GPIO_PIN_S3_BUSY;

    ESP32_has_spiflash = SPIFlash->begin(possible_devices,
                                         EXTERNAL_FLASH_DEVICE_COUNT);
    if (ESP32_has_spiflash) {
      spiflash_id = SPIFlash->getJEDECID();

      uint32_t capacity = spiflash_id & 0xFF;
      if (capacity >= 0x17) { /* equal or greater than 1UL << 23 (8 MiB) */
        hw_info.storage = STORAGE_FLASH;

#if CONFIG_TINYUSB_MSC_ENABLED
  #if defined(USE_ADAFRUIT_MSC)
        // Set disk vendor id, product id and revision
        // with string up to 8, 16, 4 characters respectively
        usb_msc.setID(ESP32S2_Device_Manufacturer, "Internal Flash", "1.0");

        // Set callback
        usb_msc.setReadWriteCallback(ESP32_msc_read_cb,
                                     ESP32_msc_write_cb,
                                     ESP32_msc_flush_cb);

        // Set disk size, block size should be 512 regardless of spi flash page size
        usb_msc.setCapacity(SPIFlash->size()/512, 512);

        // MSC is ready for read/write
        usb_msc.setUnitReady(true);

        usb_msc.begin();

  #else

        // Set disk vendor id, product id and revision
        // with string up to 8, 16, 4 characters respectively
        usb_msc.vendorID(ESP32S2_Device_Manufacturer);
        usb_msc.productID("Internal Flash");
        usb_msc.productRevision("1.0");

        // Set callback
        usb_msc.onRead(ESP32_msc_read_cb);
        usb_msc.onWrite(ESP32_msc_write_cb);

        // MSC is ready for read/write
        usb_msc.mediaPresent(true);

        // Set disk size, block size should be 512 regardless of spi flash page size
        usb_msc.begin(SPIFlash->size()/512, 512);
  #endif /* USE_ADAFRUIT_MSC */
#endif /* CONFIG_TINYUSB_MSC_ENABLED */

        FATFS_is_mounted = fatfs.begin(SPIFlash);
      }
    }

    int uSD_SS_pin = (esp32_board == ESP32_S3_DEVKIT) ?
                     SOC_GPIO_PIN_S3_SD_SS_DK : SOC_GPIO_PIN_S3_SD_SS_TBEAM;

    /* uSD-SPI init */
    uSD_SPI.begin(SOC_GPIO_PIN_S3_SD_SCK,
                  SOC_GPIO_PIN_S3_SD_MISO,
                  SOC_GPIO_PIN_S3_SD_MOSI,
                  uSD_SS_pin);

    pinMode(uSD_SS_pin, OUTPUT);
    digitalWrite(uSD_SS_pin, HIGH);

    uSD_is_mounted = uSD.cardBegin(uSD_SS_pin);

    if (uSD_is_mounted && uSD.card()->cardSize() > 0) {
      hw_info.storage = (hw_info.storage == STORAGE_FLASH) ?
                        STORAGE_FLASH_AND_CARD : STORAGE_CARD;
    }

    ui = &ui_settings;
#endif /* CONFIG_IDF_TARGET_ESP32S3 */
  }

#if ARDUINO_USB_CDC_ON_BOOT && \
    (defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3))
  if (USB.manufacturerName(ESP32S2_Device_Manufacturer)) {
    char usb_serial_number[16];
    uint16_t pid;

    pid = (esp32_board == ESP32_TTGO_T_BEAM_SUPREME) ? SOFTRF_USB_PID_PRIME_MK3  :
          (esp32_board == ESP32_S2_T8_V1_1         ) ? SOFTRF_USB_PID_WEBTOP     :
          (esp32_board == ESP32_S3_DEVKIT          ) ? SOFTRF_USB_PID_STANDALONE :
          USB_PID /* 0x1001 */ ;

    snprintf(usb_serial_number, sizeof(usb_serial_number),
             "%02X%02X%02X%02X%02X%02X",
             efuse_mac[0], efuse_mac[1], efuse_mac[2],
             efuse_mac[3], efuse_mac[4], efuse_mac[5]);

    USB.VID(USB_VID); // USB_ESPRESSIF_VID = 0x303A
    USB.PID(pid);
    USB.productName(esp32_board == ESP32_TTGO_T_BEAM_SUPREME ?
                    ESP32S3_Device_Model : ESP32S2_Device_Model);
    USB.firmwareVersion(ESP32S2_Device_Version);
    USB.serialNumber(usb_serial_number);
    USB.begin();
  }

  Serial.begin(SERIAL_OUT_BR);

  for (int i=0; i < 20; i++) {if (Serial) break; else delay(100);}
#else
  Serial.begin(SERIAL_OUT_BR, SERIAL_OUT_BITS);
#endif /* ARDUINO_USB_CDC_ON_BOOT && (CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3) */

  delay(300);
  if (SPIFFS.begin(true)) {
      Serial.print(F("\r\n\r\nSPIFFS mounted, total space: "));
      Serial.print(SPIFFS.totalBytes());
      Serial.print(F(", used space: "));
      Serial.println(SPIFFS.usedBytes());
      SPIFFS_is_mounted = true;
  } else {
      Serial.println(F("An Error has occurred while mounting SPIFFS"));
  }
//ESP_LOGI(TAG, "ESP32_setup() done");
}

static void ESP32_post_init()
{
#if defined(CONFIG_IDF_TARGET_ESP32S3)
  if (hw_info.model == SOFTRF_MODEL_PRIME_MK3)
  {
    Serial.println();
    Serial.println(F("Power-on Self Test"));
    Serial.println();
    Serial.flush();

    Serial.println(F("Built-in components:"));

    Serial.print(F("RADIO   : "));
    Serial.println(hw_info.rf      == RF_IC_SX1262 ||
                   hw_info.rf      == RF_IC_SX1276     ? F("PASS") : F("FAIL"));
    Serial.flush();
    Serial.print(F("GNSS    : "));
    Serial.println(hw_info.gnss    != GNSS_MODULE_NONE ? F("PASS") : F("FAIL"));
    Serial.flush();
    Serial.print(F("DISPLAY : "));
    Serial.println(hw_info.display == DISPLAY_OLED_1_3 ? F("PASS") : F("FAIL"));
    Serial.flush();
    Serial.print(F("RTC     : "));
    Serial.println(hw_info.rtc     == RTC_PCF8563      ? F("PASS") : F("FAIL"));
    Serial.flush();
    Serial.print(F("BARO    : "));
    Serial.println(hw_info.baro  == BARO_MODULE_BMP280 ? F("PASS") : F("N/A"));
    Serial.flush();
#if !defined(EXCLUDE_IMU)
    Serial.print(F("IMU     : "));
    Serial.println(hw_info.imu     != IMU_NONE         ? F("PASS") : F("FAIL"));
    Serial.flush();
#endif /* EXCLUDE_IMU */
    Serial.print(F("MAG     : "));
    Serial.println(hw_info.mag     != MAG_NONE         ? F("PASS") : F("FAIL"));
    Serial.flush();

    Serial.println();
    Serial.println(F("External components:"));
    Serial.print(F("CARD    : "));
    Serial.println(hw_info.storage == STORAGE_CARD ||
                   hw_info.storage == STORAGE_FLASH_AND_CARD
                                                       ? F("PASS") : F("N/A"));
    Serial.flush();

    Serial.println();
    Serial.println(F("Power-on Self Test is complete."));
    Serial.flush();
  }

  Serial.println();

  if (!uSD_is_mounted) {
    Serial.println(F("WARNING: unable to mount micro-SD card."));
  } else {
    // The number of 512 byte sectors in the card
    // or zero if an error occurs.
    size_t cardSize = uSD.card()->cardSize();

    if (cardSize == 0) {
      Serial.println(F("WARNING: invalid micro-SD card size."));
    } else {
      uint8_t cardType = uSD.card()->type();

      Serial.print(F("SD Card Type: "));
      if(cardType == SD_CARD_TYPE_SD1){
          Serial.println(F("V1"));
      } else if(cardType == SD_CARD_TYPE_SD2){
          Serial.println(F("V2"));
      } else if(cardType == SD_CARD_TYPE_SDHC){
          Serial.println(F("SDHC"));
      } else {
          Serial.println(F("UNKNOWN"));
      }

      Serial.print("SD Card Size: ");
      Serial.print(cardSize / (2 * 1024));
      Serial.println(" MB");

      uSD.fsBegin();
    }
  }

#else /* not CONFIG_IDF_TARGET_ESP32S3 */

  Serial.println();

  if (hw_info.model == SOFTRF_MODEL_PRIME_MK2)
    Serial.println("Detected MODEL_PRIME_MK2");
  else
    { Serial.print("Detected model "); Serial.println(hw_info.model); }
  Serial.print("hw_info.revision: "); Serial.println(hw_info.revision);

  Serial.println();
  Serial.print(F("CPU Freq = "));
  Serial.print(getCpuFrequencyMhz());
  Serial.println(F(" MHz"));

#endif /* CONFIG_IDF_TARGET_ESP32S3 */

  Serial.println();
  Serial.println(F("Data output device(s):"));

  Serial.print(F("NMEA   - "));
  print_dest(settings->nmea_out);
  Serial.print(F("NMEA2  - "));
  print_dest(settings->nmea_out2);

  Serial.print(F("GDL90 in -  "));
  print_dest(settings->gdl90_in);
  Serial.print(F("GDL90 out - "));
  print_dest(settings->gdl90);

#if !defined(EXCLUDE_D1090)
  Serial.print(F("D1090 out - "));
  print_dest(settings->d1090);
#endif

  Serial.println();
  switch (settings->bluetooth)
  {
    case BLUETOOTH_OFF   :  Serial.println(F("BT: OFF"));    break;
    case BLUETOOTH_SPP   :  Serial.println(F("BT: SPP"));    break;
    case BLUETOOTH_LE_HM10_SERIAL :  Serial.println(F("BT: LE"));    break;
    default              :  Serial.println(F("BT?"));       break;
  }
  Serial.println();

  if (hw_info.model == SOFTRF_MODEL_PRIME_MK2) {
    if (settings->gnss_pins != EXT_GNSS_NONE) {
        Serial.println(F("Using external GNSS"));
        if (has_axp192 || has_axp2101)
            Serial.println(F("Turning off internal GNSS"));
        if (has_axp192)
            PMU->disablePowerOutput(XPOWERS_LDO3);
        else if (has_axp2101)
            PMU->disablePowerOutput(XPOWERS_ALDO3);
    }
    if (settings->mode == SOFTRF_MODE_GPSBRIDGE) {
        Serial.println(F("GPS bridge mode"));
        if (has_axp192 || has_axp2101)
            Serial.println(F("Turning off radio"));
        if (has_axp192)
            PMU->disablePowerOutput(XPOWERS_LDO2);
        else if (has_axp2101)
            PMU->disablePowerOutput(XPOWERS_ALDO2);
    }
  }

  Serial.flush();

  switch (hw_info.display)
  {
#if defined(USE_OLED)
  case DISPLAY_OLED_TTGO:
  case DISPLAY_OLED_HELTEC:
  case DISPLAY_OLED_1_3:
    if (settings->mode == SOFTRF_MODE_GPSBRIDGE) {
        OLED_msg("GNSS", "BRIDGE");
        Serial.println(">>> GNSS Bridge Mode...");
    } else {
        OLED_info1();
    }

#if defined(CONFIG_IDF_TARGET_ESP32S3)
    if (hw_info.model == SOFTRF_MODEL_PRIME_MK3)
    {
      char key[8];
      char out[64];
      uint8_t tokens[3] = { 0 };
      cdbResult rt;
      int c, i = 0, token_cnt = 0;

      int acfts;
      char *reg, *mam, *cn;
      reg = mam = cn = NULL;

      OLED_info2();

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
        cn  = (cn  != NULL) && strlen(cn)  ? cn  : (char *) " CN: N/A";

      } else {
        acfts = -1;
      }

      OLED_info3(acfts, reg, mam, cn);
    }
#endif /* CONFIG_IDF_TARGET_ESP32S3 */

    break;
#endif /* USE_OLED */
  case DISPLAY_NONE:
  default:
    break;
  }
}

void blue_LED_on()
{
  if (PMU)
    PMU->setChargingLedMode(XPOWERS_CHG_LED_ON);
}

void blue_LED_off()
{
  if (PMU)
    PMU->setChargingLedMode(XPOWERS_CHG_LED_OFF);
}

void blue_LED_1hz()
{
  if (PMU)
    PMU->setChargingLedMode(XPOWERS_CHG_LED_BLINK_1HZ);
}

void blue_LED_4hz()
{
  if (PMU)
    PMU->setChargingLedMode(XPOWERS_CHG_LED_BLINK_4HZ);
}

static void ESP32_loop()
{
  bool is_irq = false;
  bool down = false;

  static int blue_LED_old_state = -1;
  int blue_LED_new_state = blue_LED_old_state;

  switch (hw_info.pmu)
  {
  case PMU_AXP192:
  case PMU_AXP2101:
    portENTER_CRITICAL_ISR(&PMU_mutex);
    is_irq = PMU_Irq;
    portEXIT_CRITICAL_ISR(&PMU_mutex);

    if (is_irq) {

      PMU->getIrqStatus();

      if (PMU->isPekeyLongPressIrq()) {
        down = true;
      }
      if (PMU->isPekeyShortPressIrq()) {
#if defined(USE_OLED)
        OLED_Next_Page();
#endif
      }

      PMU->clearIrqStatus();

      portENTER_CRITICAL_ISR(&PMU_mutex);
      PMU_Irq = false;
      portEXIT_CRITICAL_ISR(&PMU_mutex);

      if (down) {
        Serial.println("shutdown()...");
        Serial.flush();
        shutdown(SOFTRF_SHUTDOWN_BUTTON);
      }
    }

#if 0
    if (isTimeToBattery()) {
      if (Battery_voltage() > Battery_threshold()) {
        PMU->setChargingLedMode(XPOWERS_CHG_LED_ON);
      } else {
        PMU->setChargingLedMode(XPOWERS_CHG_LED_BLINK_1HZ);
      }
    }
#else
    // new functions for blue LED:
    //   fast blink if waiting for GNSS
    //   and turn off if not transmitting
    if (millis() > BlueLEDTimeMarker) {
      float volts = Battery_voltage();
      if (! isValidFix()) {
        blue_LED_new_state = XPOWERS_CHG_LED_BLINK_4HZ;
      } else if (settings->txpower == RF_TX_POWER_OFF) {
        blue_LED_new_state = XPOWERS_CHG_LED_OFF;
      } else if (volts > BATTERY_THRESHOLD_INVALID && volts < Battery_threshold()) {
        blue_LED_new_state = XPOWERS_CHG_LED_BLINK_1HZ;
      } else {
        blue_LED_new_state = XPOWERS_CHG_LED_ON;
      }
      if (blue_LED_new_state != blue_LED_old_state) {
        PMU->setChargingLedMode((xpowers_chg_led_mode_t) blue_LED_new_state);
        blue_LED_old_state = blue_LED_new_state;
      }
      BlueLEDTimeMarker = millis() + 1000;
    }
#endif
    break;

  case PMU_NONE:
  default:
    break;
  }

#if 0
  // show a message if long-press middle button turned off Bluetooth
  if (bt_turned_off) {
    OLED_msg("BT", "OFF");
    delay(1000);
    bt_turned_off = false;  // message done, but BT is still off
  }
#endif
}

static void ESP32_fini(int reason)
{
#if defined(CONFIG_IDF_TARGET_ESP32S3)
  if (ESP32_has_spiflash) {
#if CONFIG_TINYUSB_MSC_ENABLED
  #if defined(USE_ADAFRUIT_MSC)
    usb_msc.setUnitReady(false);
//  usb_msc.end(); /* N/A */
  #else
    usb_msc.mediaPresent(false);
    usb_msc.end();
  #endif /* USE_ADAFRUIT_MSC */
#endif /* CONFIG_TINYUSB_MSC_ENABLED */
  }

  if (SPIFlash != NULL) SPIFlash->end();
#endif /* CONFIG_IDF_TARGET_ESP32S3 */

  SPI.end();

  esp_wifi_stop();

#if !defined(CONFIG_IDF_TARGET_ESP32S2) && !defined(CONFIG_IDF_TARGET_ESP32S3)
  esp_bt_controller_disable();
#endif /* CONFIG_IDF_TARGET_ESP32S2 */

  if (hw_info.model == SOFTRF_MODEL_PRIME_MK2 ||
             hw_info.model == SOFTRF_MODEL_PRIME_MK3) {

    switch (hw_info.pmu)
    {
    case PMU_AXP192:
      //PMU->setChargingLedMode(XPOWERS_CHG_LED_OFF);
      PMU->setChargingLedMode(XPOWERS_CHG_LED_CTRL_CHG);
             // The charging indicator is controlled by the charger

      //PMU->disableButtonBatteryCharge();  - old syntax
      //PMU->disablePowerOutput(XPOWERS_VBACKUP);  - new syntax
      //   - but prefer to keep the GNSS data alive so don't do that

#if PMK2_SLEEP_MODE == 2
      { int ret;
      // PEK or GPIO edge wake-up function enable setting in Sleep mode
      do {
          // In order to ensure that it is set correctly,
          // the loop waits for it to return the correct return value
          ret = axp_xxx.setSleep();
          delay(500);
      } while (ret != AXP_PASS) ; }

      // Turn off all power channels, only use PEK or AXP GPIO to wake up

      // After setting AXP202/AXP192 to sleep,
      // it will start to record the status of the power channel that was turned off after setting,
      // it will restore the previously set state after PEK button or GPIO wake up

#endif /* PMK2_SLEEP_MODE */

      PMU->disablePowerOutput(XPOWERS_LDO2);
      PMU->disablePowerOutput(XPOWERS_LDO3);
      PMU->disablePowerOutput(XPOWERS_DCDC2);

      /* workaround against AXP I2C access blocking by 'noname' OLED */
#if defined(USE_OLED)
      if (u8x8 == NULL)
#endif /* USE_OLED */
      {
          PMU->disablePowerOutput(XPOWERS_DCDC1);
      }
      //PMU->disablePowerOutput(XPOWERS_EXTEN);

      delay(20);

      /*
       * When driven by SoftRF the V08+ T-Beam takes:
       * in 'full power' - 160 - 180 mA
       * in 'stand by'   - 600 - 900 uA
       * in 'power off'  -  50 -  90 uA
       * of current from 3.7V battery
       */
#if   PMK2_SLEEP_MODE == 1
      /* Deep sleep with wakeup by power button click */
      esp_sleep_enable_ext1_wakeup(1ULL << SOC_GPIO_PIN_TBEAM_V08_PMU_IRQ,
                                 ESP_EXT1_WAKEUP_ALL_LOW);
#elif PMK2_SLEEP_MODE == 2
      // Cut MCU power off, PMU remains in sleep until wakeup by PEK button press
      axp_xxx.setPowerOutPut(AXP192_DCDC3, AXP202_OFF);
#else
      /*
       * Complete power off
       *
       * to power back on either:
       * - press and hold PWR button for 1-2 seconds then release, or
       * - cycle micro-USB power
       */
      PMU->shutdown();
#endif /* PMK2_SLEEP_MODE */
      break;

    case PMU_AXP2101:
      //PMU->setChargingLedMode(XPOWERS_CHG_LED_OFF);
      PMU->setChargingLedMode(XPOWERS_CHG_LED_CTRL_CHG);
             // The charging indicator is controlled by the charger

      //PMU->disableButtonBatteryCharge();  - old syntax
      //PMU->disablePowerOutput(XPOWERS_VBACKUP);  - new syntax
      //   - but prefer to keep the GNSS data alive so don't do that

      PMU->disablePowerOutput(XPOWERS_ALDO2);
      PMU->disablePowerOutput(XPOWERS_ALDO3);

      delay(20);

      /*
       * Complete power off
       *
       * to power back on either:
       * - press and hold PWR button for 1-2 seconds then release, or
       * - cycle micro-USB power
       */
      PMU->shutdown();
      break;

    case PMU_NONE:
    default:
      break;
    }
  } else if (esp32_board == ESP32_S2_T8_V1_1) {
    pinMode(SOC_GPIO_PIN_T8_S2_PWR_EN, INPUT);

    esp_sleep_enable_ext1_wakeup(1ULL << SOC_GPIO_PIN_T8_S2_BUTTON,
                                 ESP_EXT1_WAKEUP_ALL_LOW);
  }

  esp_deep_sleep_start();
}

static void ESP32_reset()
{
  ESP.restart();
}

static uint32_t ESP32_getChipId()
{
#if !defined(SOFTRF_ADDRESS)
  static uint32_t id = 0;
  if (id == 0) {
      id = (uint32_t) efuse_mac[5]        | ((uint32_t) efuse_mac[4] << 8) | \
          ((uint32_t) efuse_mac[3] << 16) | ((uint32_t) efuse_mac[2] << 24);
      id = DevID_Mapper(id);
  }
  return id;
#else
  return (SOFTRF_ADDRESS & 0x00FFFFFFU );
#endif /* SOFTRF_ADDRESS */
}

static struct rst_info reset_info = {
  .reason = REASON_DEFAULT_RST,
};

static void* ESP32_getResetInfoPtr()
{
  switch (rtc_get_reset_reason(0))
  {
    case POWERON_RESET          : reset_info.reason = REASON_DEFAULT_RST; break;
    case DEEPSLEEP_RESET        : reset_info.reason = REASON_DEEP_SLEEP_AWAKE; break;
    case TG0WDT_SYS_RESET       : reset_info.reason = REASON_WDT_RST; break;
    case TG1WDT_SYS_RESET       : reset_info.reason = REASON_WDT_RST; break;
    case RTCWDT_SYS_RESET       : reset_info.reason = REASON_WDT_RST; break;
    case INTRUSION_RESET        : reset_info.reason = REASON_EXCEPTION_RST; break;
    case RTCWDT_CPU_RESET       : reset_info.reason = REASON_WDT_RST; break;
    case RTCWDT_BROWN_OUT_RESET : reset_info.reason = REASON_EXT_SYS_RST; break;
    case RTCWDT_RTC_RESET       :
      /* Slow start of GD25LQ32 causes one read fault at boot time with current ESP-IDF */
      if (ESP32_getFlashId() == MakeFlashId(GIGADEVICE_ID, GIGADEVICE_GD25LQ32))
                                  reset_info.reason = REASON_DEFAULT_RST;
      else
                                  reset_info.reason = REASON_WDT_RST;
                                  break;
#if !defined(CONFIG_IDF_TARGET_ESP32S2) && !defined(CONFIG_IDF_TARGET_ESP32S3)
    case SW_RESET               : reset_info.reason = REASON_SOFT_RESTART; break;
    case OWDT_RESET             : reset_info.reason = REASON_WDT_RST; break;
    case SDIO_RESET             : reset_info.reason = REASON_EXCEPTION_RST; break;
    case TGWDT_CPU_RESET        : reset_info.reason = REASON_WDT_RST; break;
    case SW_CPU_RESET           : reset_info.reason = REASON_SOFT_RESTART; break;
    case EXT_CPU_RESET          : reset_info.reason = REASON_EXT_SYS_RST; break;
#endif /* CONFIG_IDF_TARGET_ESP32S2 */
    default                     : reset_info.reason = REASON_DEFAULT_RST;
  }

  return (void *) &reset_info;
}

static String ESP32_getResetInfo()
{
  switch (rtc_get_reset_reason(0))
  {
    case POWERON_RESET          : return F("Vbat power on reset");
    case DEEPSLEEP_RESET        : return F("Deep Sleep reset digital core");
    case TG0WDT_SYS_RESET       : return F("Timer Group0 Watch dog reset digital core");
    case TG1WDT_SYS_RESET       : return F("Timer Group1 Watch dog reset digital core");
    case RTCWDT_SYS_RESET       : return F("RTC Watch dog Reset digital core");
    case INTRUSION_RESET        : return F("Instrusion tested to reset CPU");
    case RTCWDT_CPU_RESET       : return F("RTC Watch dog Reset CPU");
    case RTCWDT_BROWN_OUT_RESET : return F("Reset when the vdd voltage is not stable");
    case RTCWDT_RTC_RESET       : return F("RTC Watch dog reset digital core and rtc module");
#if !defined(CONFIG_IDF_TARGET_ESP32S2) && !defined(CONFIG_IDF_TARGET_ESP32S3)
    case SW_RESET               : return F("Software reset digital core");
    case OWDT_RESET             : return F("Legacy watch dog reset digital core");
    case SDIO_RESET             : return F("Reset by SLC module, reset digital core");
    case TGWDT_CPU_RESET        : return F("Time Group reset CPU");
    case SW_CPU_RESET           : return F("Software reset CPU");
    case EXT_CPU_RESET          : return F("for APP CPU, reseted by PRO CPU");
#endif /* CONFIG_IDF_TARGET_ESP32S2 */
    default                     : return F("No reset information available");
  }
}

static String ESP32_getResetReason()
{

  switch (rtc_get_reset_reason(0))
  {
    case POWERON_RESET          : return F("POWERON_RESET");
    case DEEPSLEEP_RESET        : return F("DEEPSLEEP_RESET");
    case TG0WDT_SYS_RESET       : return F("TG0WDT_SYS_RESET");
    case TG1WDT_SYS_RESET       : return F("TG1WDT_SYS_RESET");
    case RTCWDT_SYS_RESET       : return F("RTCWDT_SYS_RESET");
    case INTRUSION_RESET        : return F("INTRUSION_RESET");
    case RTCWDT_CPU_RESET       : return F("RTCWDT_CPU_RESET");
    case RTCWDT_BROWN_OUT_RESET : return F("RTCWDT_BROWN_OUT_RESET");
    case RTCWDT_RTC_RESET       : return F("RTCWDT_RTC_RESET");
#if !defined(CONFIG_IDF_TARGET_ESP32S2) && !defined(CONFIG_IDF_TARGET_ESP32S3)
    case SW_RESET               : return F("SW_RESET");
    case OWDT_RESET             : return F("OWDT_RESET");
    case SDIO_RESET             : return F("SDIO_RESET");
    case TGWDT_CPU_RESET        : return F("TGWDT_CPU_RESET");
    case SW_CPU_RESET           : return F("SW_CPU_RESET");
    case EXT_CPU_RESET          : return F("EXT_CPU_RESET");
#endif /* CONFIG_IDF_TARGET_ESP32S2 */
    default                     : return F("NO_MEAN");
  }
}

static uint32_t ESP32_getFreeHeap()
{
  return ESP.getFreeHeap();
}

static long ESP32_random(long howsmall, long howBig)
{
  return random(howsmall, howBig);
}

#if 0

// old sound code, not used, switched buzzer to ToneAC

#if !defined(CONFIG_IDF_TARGET_ESP32S2) && defined(USE_BLE_MIDI)
extern bool deviceConnected;
extern BLECharacteristic* pMIDICharacteristic;

uint8_t midiPacket[] = {
   0x80,  // header
   0x80,  // timestamp, not implemented
   0x00,  // status
   0x3c,  // 0x3c == 60 == middle c
   0x00   // velocity
};

byte note_sequence[] = {62,65,69,65,67,67,65,64,69,69,67,67,62,62};
#endif /* USE_BLE_MIDI */

static void ESP32_Sound_test(int var)
{
  if (SOC_GPIO_PIN_BUZZER != SOC_UNUSED_PIN && settings->volume != BUZZER_OFF) {

    ledcAttachPin(SOC_GPIO_PIN_BUZZER, LEDC_CHANNEL_BUZZER);
    ledcSetup(LEDC_CHANNEL_BUZZER, 0, LEDC_RESOLUTION_BUZZER);

    if (var == REASON_DEFAULT_RST ||
        var == REASON_EXT_SYS_RST ||
        var == REASON_SOFT_RESTART) {
      ledcWriteTone(LEDC_CHANNEL_BUZZER, 440);delay(500);
      ledcWriteTone(LEDC_CHANNEL_BUZZER, 640);delay(500);
      ledcWriteTone(LEDC_CHANNEL_BUZZER, 840);delay(500);
      ledcWriteTone(LEDC_CHANNEL_BUZZER, 1040);
    } else if (var == REASON_WDT_RST) {
      ledcWriteTone(LEDC_CHANNEL_BUZZER, 440);delay(500);
      ledcWriteTone(LEDC_CHANNEL_BUZZER, 1040);delay(500);
      ledcWriteTone(LEDC_CHANNEL_BUZZER, 440);delay(500);
      ledcWriteTone(LEDC_CHANNEL_BUZZER, 1040);
    } else {
      ledcWriteTone(LEDC_CHANNEL_BUZZER, 1040);delay(500);
      ledcWriteTone(LEDC_CHANNEL_BUZZER, 840);delay(500);
      ledcWriteTone(LEDC_CHANNEL_BUZZER, 640);delay(500);
      ledcWriteTone(LEDC_CHANNEL_BUZZER, 440);
    }
    delay(600);

    ledcWriteTone(LEDC_CHANNEL_BUZZER, 0); // off

    ledcDetachPin(SOC_GPIO_PIN_BUZZER);
    pinMode(SOC_GPIO_PIN_BUZZER, INPUT_PULLDOWN);
  }

#if !defined(CONFIG_IDF_TARGET_ESP32S2) && defined(USE_BLE_MIDI)
  if (settings->volume != BUZZER_OFF                  &&
      settings->bluetooth == BLUETOOTH_LE_HM10_SERIAL &&
      pMIDICharacteristic != NULL                     &&
      deviceConnected) {

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
      // note down
      midiPacket[2] = 0x90; // note down, channel 0
      midiPacket[3] = note_sequence[current];
      midiPacket[4] = 127;  // velocity
      pMIDICharacteristic->setValue(midiPacket, 5); // packet, length in bytes
      pMIDICharacteristic->notify();

      // Send Note Off for previous note.
      // note up
      midiPacket[2] = 0x80; // note up, channel 0
      midiPacket[3] = note_sequence[previous];
      midiPacket[4] = 0;    // velocity
      pMIDICharacteristic->setValue(midiPacket, 5); // packet, length in bytes)
      pMIDICharacteristic->notify();

      // play note for 286ms
      delay(286);
    }

    // note up
    midiPacket[2] = 0x80; // note up, channel 0
    midiPacket[3] = note_sequence[current];
    midiPacket[4] = 0;    // velocity
    pMIDICharacteristic->setValue(midiPacket, 5); // packet, length in bytes)
    pMIDICharacteristic->notify();
  }
#endif /* USE_BLE_MIDI */
}

static void ESP32_Sound_tone(int hz, uint8_t volume)
{
  if (SOC_GPIO_PIN_BUZZER != SOC_UNUSED_PIN && volume != BUZZER_OFF) {
    if (hz > 0) {
      ledcAttachPin(SOC_GPIO_PIN_BUZZER, LEDC_CHANNEL_BUZZER);
      ledcSetup(LEDC_CHANNEL_BUZZER, 0, LEDC_RESOLUTION_BUZZER);

      ledcWriteTone(LEDC_CHANNEL_BUZZER, hz);
      ledcWrite(LEDC_CHANNEL_BUZZER, volume == BUZZER_VOLUME_FULL ? 0xFF : 0x07);
    } else {
      ledcWriteTone(LEDC_CHANNEL_BUZZER, 0); // off

      ledcDetachPin(SOC_GPIO_PIN_BUZZER);
      pinMode(SOC_GPIO_PIN_BUZZER, INPUT_PULLDOWN);
    }
  }

#if !defined(CONFIG_IDF_TARGET_ESP32S2) && defined(USE_BLE_MIDI)
  if (volume != BUZZER_OFF                            &&
      settings->bluetooth == BLUETOOTH_LE_HM10_SERIAL &&
      pMIDICharacteristic != NULL                     &&
      deviceConnected) {
    midiPacket[3] = 60; // 60 == middle C
    if (hz > 0) {
      // Send Note On for current position at full velocity (127) on channel 1.
      // note down
      midiPacket[2] = 0x90; // note down, channel 0
      midiPacket[4] = 127;  // velocity
    } else {
      // Send Note Off for previous note.
      // note up
      midiPacket[2] = 0x80; // note up, channel 0
      midiPacket[4] = 0;    // velocity
    }
    pMIDICharacteristic->setValue(midiPacket, 5); // packet, length in bytes)
    pMIDICharacteristic->notify();
  }
#endif /* USE_BLE_MIDI */
}

#endif /* old sound code */

#include <toneAC.h>

static void Buzzer_tone(int hz, int duration)
{
    int volume = (settings->volume == BUZZER_VOLUME_LOW ? 2 : 10);
    toneAC(hz, volume, duration, false);
}

/* dummy function to fit the SoC_ops structure */
static void ESP32_Buzzer_tone(int hz, uint8_t volume)
{
    Buzzer_tone(hz, 500);
}

void ESP32_Buzzer_test(int reason)
{
    if (settings->volume == BUZZER_OFF)
        return;
    if (settings->volume == BUZZER_EXT)
        return;

Serial.println("Buzzer_test() using ToneAC...");

    if (reason == REASON_DEFAULT_RST ||
        reason == REASON_EXT_SYS_RST ||
        reason == REASON_SOFT_RESTART) {
Serial.println("... tone 1:");
      Buzzer_tone(440, 500);
      delay(100);
Serial.println("... tone 2:");
      Buzzer_tone(640, 500);
      delay(100);
Serial.println("... tone 3:");
      Buzzer_tone(840, 500);
      delay(100);
Serial.println("... tone 4:");
      Buzzer_tone(1040, 600);
    } else if (reason == REASON_WDT_RST) {
      Buzzer_tone(440,  500);
      Buzzer_tone(1040, 500);
      Buzzer_tone(440,  500);
      Buzzer_tone(1040, 600);
    } else {
      Buzzer_tone(1040, 500);
      Buzzer_tone(840,  500);
      Buzzer_tone(640,  500);
      Buzzer_tone(440,  600);
    }

    noToneAC();
}

static uint32_t ESP32_maxSketchSpace()
{
  return ESP32_Min_AppPart_Size ? ESP32_Min_AppPart_Size :
           SoC->id == SOC_ESP32S3 ?
             0x200000  /* 8MB-tinyuf2.csv */ :
             0x1E0000; /* min_spiffs.csv */
}

static const int8_t ESP32_dBm_to_power_level[21] = {
  8,  /* 2    dBm, #0 */
  8,  /* 2    dBm, #1 */
  8,  /* 2    dBm, #2 */
  8,  /* 2    dBm, #3 */
  8,  /* 2    dBm, #4 */
  20, /* 5    dBm, #5 */
  20, /* 5    dBm, #6 */
  28, /* 7    dBm, #7 */
  28, /* 7    dBm, #8 */
  34, /* 8.5  dBm, #9 */
  34, /* 8.5  dBm, #10 */
  44, /* 11   dBm, #11 */
  44, /* 11   dBm, #12 */
  52, /* 13   dBm, #13 */
  52, /* 13   dBm, #14 */
  60, /* 15   dBm, #15 */
  60, /* 15   dBm, #16 */
  68, /* 17   dBm, #17 */
  74, /* 18.5 dBm, #18 */
  76, /* 19   dBm, #19 */
  78  /* 19.5 dBm, #20 */
};

static void ESP32_WiFi_set_param(int ndx, int value)
{
#if !defined(EXCLUDE_WIFI)
  uint32_t lt = value * 60; /* in minutes */

  switch (ndx)
  {
  case WIFI_PARAM_TX_POWER:
    if (value > 20) {
      value = 20; /* dBm */
    }

    if (value < 0) {
      value = 0; /* dBm */
    }

    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(ESP32_dBm_to_power_level[value]));
    break;
  case WIFI_PARAM_DHCP_LEASE_TIME:
    tcpip_adapter_dhcps_option(
      (tcpip_adapter_dhcp_option_mode_t) TCPIP_ADAPTER_OP_SET,
      (tcpip_adapter_dhcp_option_id_t)   TCPIP_ADAPTER_IP_ADDRESS_LEASE_TIME,
      (void*) &lt, sizeof(lt));
    break;
  default:
    break;
  }
#endif /* EXCLUDE_WIFI */
}

static IPAddress ESP32_WiFi_get_broadcast()
{
  tcpip_adapter_ip_info_t info;
  IPAddress broadcastIp;

  if (WiFi.getMode() == WIFI_STA) {
    tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &info);
  } else {
    tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_AP, &info);
  }
  broadcastIp = ~info.netmask.addr | info.ip.addr;

  return broadcastIp;
}

static void ESP32_WiFi_transmit_UDP(int port, byte *buf, size_t size)
{
#if !defined(EXCLUDE_WIFI)
  IPAddress ClientIP;
  WiFiMode_t mode = WiFi.getMode();
  int i = 0;

  switch (mode)
  {
  case WIFI_STA:
    ClientIP = ESP32_WiFi_get_broadcast();

    Uni_Udp.beginPacket(ClientIP, port);
    Uni_Udp.write(buf, size);
    Uni_Udp.endPacket();

    break;
  case WIFI_AP:
    wifi_sta_list_t stations;
    ESP_ERROR_CHECK(esp_wifi_ap_get_sta_list(&stations));

    tcpip_adapter_sta_list_t infoList;
    ESP_ERROR_CHECK(tcpip_adapter_get_sta_list(&stations, &infoList));

    while(i < infoList.num) {
      ClientIP = infoList.sta[i++].ip.addr;

      Uni_Udp.beginPacket(ClientIP, port);
      Uni_Udp.write(buf, size);
      Uni_Udp.endPacket();
    }
    break;
  case WIFI_OFF:
  default:
    break;
  }
#endif /* EXCLUDE_WIFI */
}

static void ESP32_WiFiUDP_stopAll()
{
/* not implemented yet */
}

static bool ESP32_WiFi_hostname(String aHostname)
{
#if defined(EXCLUDE_WIFI)
  return false;
#else
  return WiFi.setHostname(aHostname.c_str());
#endif /* EXCLUDE_WIFI */
}

static int ESP32_WiFi_clients_count()
{
#if defined(EXCLUDE_WIFI)
  return 0;
#else
  WiFiMode_t mode = WiFi.getMode();

  switch (mode)
  {
  case WIFI_AP:
    wifi_sta_list_t stations;
    ESP_ERROR_CHECK(esp_wifi_ap_get_sta_list(&stations));

    tcpip_adapter_sta_list_t infoList;
    ESP_ERROR_CHECK(tcpip_adapter_get_sta_list(&stations, &infoList));

    return infoList.num;
  case WIFI_STA:
  default:
    return -1; /* error */
  }
#endif /* EXCLUDE_WIFI */
}

static bool ESP32_EEPROM_begin(size_t size)
{
  bool rval = true;

#if !defined(EXCLUDE_EEPROM)
  rval = EEPROM.begin(size);
#endif

  return rval;
}

static void ESP32_EEPROM_extension(int cmd)
{
    // nothing to do
}

static void ESP32_SPI_begin()
{
  switch (esp32_board)
  {
    case ESP32_S2_T8_V1_1:
      SPI.begin(SOC_GPIO_PIN_T8_S2_SCK,  SOC_GPIO_PIN_T8_S2_MISO,
                SOC_GPIO_PIN_T8_S2_MOSI, SOC_GPIO_PIN_T8_S2_SS);
      break;
    case ESP32_S3_DEVKIT:
    case ESP32_TTGO_T_BEAM_SUPREME:
      SPI.begin(SOC_GPIO_PIN_S3_SCK,  SOC_GPIO_PIN_S3_MISO,
                SOC_GPIO_PIN_S3_MOSI, SOC_GPIO_PIN_S3_SS);
      break;
    default:
      SPI.begin(SOC_GPIO_PIN_SCK,  SOC_GPIO_PIN_MISO,
                SOC_GPIO_PIN_MOSI, SOC_GPIO_PIN_SS);
      ESP32_pin_reserved(SOC_GPIO_PIN_SCK,  false, (const char *) NULL);
      ESP32_pin_reserved(SOC_GPIO_PIN_MISO, false, (const char *) NULL);
      ESP32_pin_reserved(SOC_GPIO_PIN_MOSI, false, (const char *) NULL);
      ESP32_pin_reserved(SOC_GPIO_PIN_SS,   false, (const char *) NULL);
      break;
  }
}

static void ESP32_swSer_begin(unsigned long baud)
{
  if (hw_info.model == SOFTRF_MODEL_PRIME_MK2) {

    Serial.print(F("INFO: TTGO T-Beam rev. "));
    Serial.print(hw_info.revision);
    Serial.println(F(" is detected."));

    if (settings->gnss_pins == EXT_GNSS_39_4) {
      if (hw_info.revision < 8) {
          if (ESP32_pin_reserved(Serial0AltRxPin, false, "EXT GNSS RX")) return;
          if (ESP32_pin_reserved(Serial2TxPin,    false, "EXT GNSS TX")) return;
          Serial.println(F("Connecting to external GNSS on pins VP,4"));
          Serial_GNSS_In.begin(baud, SERIAL_IN_BITS,
                               Serial0AltRxPin,
                               Serial2TxPin);
      } else {
          if (ESP32_pin_reserved(Serial2RxPin, false, "EXT GNSS RX")) return;
          if (ESP32_pin_reserved(Serial2TxPin, false, "EXT GNSS TX")) return;
          Serial.println(F("Connecting to external GNSS on pins VN,4"));
          Serial_GNSS_In.begin(baud, SERIAL_IN_BITS,
                               Serial2RxPin,
                               Serial2TxPin);  // Serial1, but the pins Serial2 could have used
      }
    } else if (settings->gnss_pins == EXT_GNSS_13_2) {
      if (ESP32_pin_reserved(SOC_GPIO_PIN_TBEAM_SDA, false, "EXT GNSS RX")) return;
      if (ESP32_pin_reserved(SOC_GPIO_PIN_TBEAM_SCL, false, "EXT GNSS TX")) return;
      Serial.println(F("Connecting to external GNSS on pins 13,2"));
      Serial_GNSS_In.begin(baud, SERIAL_IN_BITS,
                           SOC_GPIO_PIN_TBEAM_SDA,
                           SOC_GPIO_PIN_TBEAM_SCL);   // the pins BMP (& OLED) could have used
    } else if (settings->gnss_pins == EXT_GNSS_15_14) {
      if (hw_info.revision < 8) {
          if (ESP32_pin_reserved(SOC_GPIO_PIN_VOICE,  false, "EXT GNSS RX")) return;
          if (ESP32_pin_reserved(SOC_GPIO_PIN_BUZZER, false, "EXT GNSS TX")) return;
          Serial.println(F("Connecting to external GNSS on pins 25,14"));
          Serial_GNSS_In.begin(baud, SERIAL_IN_BITS,
                           SOC_GPIO_PIN_VOICE,
                           SOC_GPIO_PIN_BUZZER);
      } else {
          if (ESP32_pin_reserved(SOC_GPIO_PIN_BUZZER2, false, "EXT GNSS RX")) return;
          if (ESP32_pin_reserved(SOC_GPIO_PIN_BUZZER,  false, "EXT GNSS TX")) return;
          Serial.println(F("Connecting to external GNSS on pins 15,14"));
          Serial_GNSS_In.begin(baud, SERIAL_IN_BITS,
                           SOC_GPIO_PIN_BUZZER2,
                           SOC_GPIO_PIN_BUZZER);
      }
    } else if (hw_info.revision >= 8) {
      Serial.println(F("Connecting to internal GNSS"));
      Serial_GNSS_In.begin(baud, SERIAL_IN_BITS,
                           SOC_GPIO_PIN_TBEAM_V08_RX,
                           SOC_GPIO_PIN_TBEAM_V08_TX);
    } else {
      Serial.println(F("Connecting to internal GNSS"));
      Serial_GNSS_In.begin(baud, SERIAL_IN_BITS,
                           SOC_GPIO_PIN_TBEAM_V05_RX,
                           SOC_GPIO_PIN_TBEAM_V05_TX);
    }
  } else if (hw_info.model == SOFTRF_MODEL_PRIME_MK3) {

    Serial.println(F("INFO: TTGO T-Beam Supreme is detected."));

    Serial_GNSS_In.begin(baud, SERIAL_IN_BITS,
                         SOC_GPIO_PIN_S3_GNSS_RX,
                         SOC_GPIO_PIN_S3_GNSS_TX);

  } else {
    if (esp32_board == ESP32_TTGO_T_WATCH) {
      Serial.println(F("Warning: TTGO T-Watch detected - not supported."));
    } else if (esp32_board == ESP32_TTGO_V2_OLED) {
      /* 'Mini' (TTGO T3 + GNSS) */
      Serial.print(F("INFO: TTGO T3 rev. "));
      Serial.print(hw_info.revision);
      Serial.println(F(" is detected."));
      Serial_GNSS_In.begin(baud, SERIAL_IN_BITS,
                           TTGO_V2_PIN_GNSS_RX, TTGO_V2_PIN_GNSS_TX);
    } else if (esp32_board == ESP32_S2_T8_V1_1) {
      Serial.println(F("INFO: TTGO T8_S2 rev. 1.1 is detected."));
      Serial_GNSS_In.begin(baud, SERIAL_IN_BITS,
                           SOC_GPIO_PIN_T8_S2_GNSS_RX,
                           SOC_GPIO_PIN_T8_S2_GNSS_TX);
    } else if (esp32_board == ESP32_S3_DEVKIT) {
      Serial.println(F("INFO: ESP32-S3 DevKit is detected."));
      Serial_GNSS_In.begin(baud, SERIAL_IN_BITS,
                           SOC_GPIO_PIN_S3_GNSS_RX, SOC_GPIO_PIN_S3_GNSS_TX);
    } else {
      /* open Standalone's GNSS port */
      Serial_GNSS_In.begin(baud, SERIAL_IN_BITS,
                           SOC_GPIO_PIN_GNSS_RX, SOC_GPIO_PIN_GNSS_TX);
    }
  }

  /* Default Rx buffer size (256 bytes) is sometimes not big enough */
  // Serial_GNSS_In.setRxBufferSize(512);

  /* Need to gather some statistics on variety of flash IC usage */  // <<< why here?
  Serial.print(F("Flash memory ID: "));
  Serial.println(ESP32_getFlashId(), HEX);
}

static void ESP32_swSer_enableRx(boolean arg)
{

}

static byte ESP32_Display_setup()
{
  byte rval = DISPLAY_NONE;

  if (esp32_board != ESP32_TTGO_T_WATCH &&
      esp32_board != ESP32_S2_T8_V1_1) {

#if defined(USE_OLED)
    bool has_oled = false;

    /* SSD1306 I2C OLED probing */

    if (esp32_board == ESP32_S3_DEVKIT) {
      Wire.begin(SOC_GPIO_PIN_S3_SDA, SOC_GPIO_PIN_S3_SCL);
      Wire.beginTransmission(SSD1306_OLED_I2C_ADDR);
      has_oled = (Wire.endTransmission() == 0);
      WIRE_FINI(Wire);
      if (has_oled) {
        u8x8 = &u8x8_ttgo;
        rval = DISPLAY_OLED_TTGO;
      }
    } else if (esp32_board == ESP32_TTGO_T_BEAM_SUPREME) {
      Wire.begin(SOC_GPIO_PIN_S3_SDA, SOC_GPIO_PIN_S3_SCL);
      Wire.beginTransmission(SH1106_OLED_I2C_ADDR);
      has_oled = (Wire.endTransmission() == 0);
      WIRE_FINI(Wire);
      if (has_oled) {
        u8x8 = &u8x8_1_3;
        rval = DISPLAY_OLED_1_3;
      }

    } else if (hw_info.model == SOFTRF_MODEL_PRIME_MK2) {

        // Start up Wire here rather than in ESP32_setup since we need to look at settings
        // OLED probe (here) and Baro probe try both wires
        ESP32_pin_reserved(SOC_GPIO_PIN_TBEAM_SDA, true, "Wire");  // allow re-use of these pins later
        ESP32_pin_reserved(SOC_GPIO_PIN_TBEAM_SCL, true, "Wire");
        Wire.begin(SOC_GPIO_PIN_TBEAM_SDA, SOC_GPIO_PIN_TBEAM_SCL);    // GPIO 13,2

        Wire1.beginTransmission(SSD1306_OLED_I2C_ADDR);      // GPIO 21,22
        has_oled = (Wire1.endTransmission() == 0);
        if (has_oled) {
          u8x8 = &u8x8_ttgo;
          rval = DISPLAY_OLED_TTGO;
          Serial.println(F("u8x8_ttgo OLED found at pins 21,22"));
        } else {
          if (settings->gnss_pins != EXT_GNSS_13_2) {
            Wire.beginTransmission(SSD1306_OLED_I2C_ADDR);      // GPIO 13,2
            has_oled = (Wire.endTransmission() == 0);
          }
          if (has_oled) {
            u8x8 = &u8x8_ttgo2;
            rval = DISPLAY_OLED_TTGO;
            Serial.println(F("u8x8_ttgo OLED found at pins 2,13"));
            ESP32_pin_reserved(SOC_GPIO_PIN_TBEAM_SDA, false, "OLED");   // but may be shared with BMP
            ESP32_pin_reserved(SOC_GPIO_PIN_TBEAM_SCL, false, "OLED");
          } else {
            Serial.println(F("OLED not found at pins 21,22 nor 2,13"));
          }
        }

    } else {
        Wire1.begin(HELTEC_OLED_PIN_SDA, HELTEC_OLED_PIN_SCL);
        Wire1.beginTransmission(SSD1306_OLED_I2C_ADDR);
        has_oled = (Wire1.endTransmission() == 0);
        if (has_oled) {
          u8x8 = &u8x8_heltec;
          esp32_board = ESP32_HELTEC_OLED;
          rval = DISPLAY_OLED_HELTEC;
        }

    }  // end if (PRIME_MK2)

    if (u8x8) {
      u8x8->begin();
      u8x8->setFont(u8x8_font_chroma48medium8_r);
      u8x8->clear();

      uint8_t shift_y = (hw_info.model == SOFTRF_MODEL_PRIME_MK3 ? 1 : 0);

      const char SoftRF_text1[]  = "SoftRF";
      const char SoftRF_text2[]  = "and";
      const char SoftRF_text3[]  = "LilyGO";
      const char DFLT_text[]     = "DFLT";
      const char USER_text[]     = "USER";
      const char PROM_text[]     = "PROM";

      if (shift_y) {
        u8x8->draw2x2String( 2, 2 - shift_y, SoftRF_text1);
        u8x8->drawString   ( 6, 3, SoftRF_text2);
        u8x8->draw2x2String( 2, 4, SoftRF_text3);
        u8x8->drawString   ( 3, 6 + shift_y, SOFTRF_FIRMWARE_VERSION);
        u8x8->drawString   (11, 6 + shift_y, ISO3166_CC[settings->band]);

      } else {
        if (settings->debug_flags & DEBUG_SIMULATE)
            u8x8->draw2x2String( 2, 1, "SIMUL");
        else
            u8x8->draw2x2String( 2, 1, SoftRF_text1);
        u8x8->drawString( 1, 4, "ver");
        u8x8->draw2x2String( 5, 4, SOFTRF_FIRMWARE_VERSION);
        u8x8->draw2x2String( 1, 6,
            settings_used==STG_DEFAULT? DFLT_text :
            settings_used==STG_EEPROM?  PROM_text :
            USER_text);
        u8x8->drawString( 10, 7, "stgs");

      }

    }

    SoC->ADB_ops && SoC->ADB_ops->setup();
#endif /* USE_OLED */

  }

  return rval;
}

static void ESP32_Display_loop()
{
  char buf[16];
  uint32_t disp_value;

  uint16_t tbw;
  uint16_t tbh;

  switch (hw_info.display)
  {

#if defined(USE_TFT)
#if LV_HOR_RES == 240
  case DISPLAY_TFT_TTGO_240:
    if (tft) {
      if (!TFT_display_frontpage) {
        tft->fillScreen(TFT_NAVY);

        tft->setTextFont(2);
        tft->setTextSize(2);
        tft->setTextColor(TFT_WHITE, TFT_NAVY);

        tbw = tft->textWidth(ID_text);
        tbh = tft->fontHeight();

        tft->setCursor(tft->textWidth(" "), tft->height()/6 - tbh);
        tft->print(ID_text);

        tbw = tft->textWidth(PROTOCOL_text);

        tft->setCursor(tft->width() - tbw - tft->textWidth(" "),
                       tft->height()/6 - tbh);
        tft->print(PROTOCOL_text);

        tbw = tft->textWidth(RX_text);
        tbh = tft->fontHeight();

        tft->setCursor(tft->textWidth("   "), tft->height()/2 - tbh);
        tft->print(RX_text);

        tbw = tft->textWidth(TX_text);

        tft->setCursor(tft->width()/2 + tft->textWidth("   "),
                       tft->height()/2 - tbh);
        tft->print(TX_text);

        tft->setTextFont(4);
        tft->setTextSize(2);

        snprintf (buf, sizeof(buf), "%06X", ThisAircraft.addr);

        tbw = tft->textWidth(buf);
        tbh = tft->fontHeight();

        tft->setCursor(tft->textWidth(" "), tft->height()/6);
        tft->print(buf);

        tbw = tft->textWidth("O");

        tft->setCursor(tft->width() - tbw - tft->textWidth(" "),
                       tft->height()/6);
        tft->print(Protocol_ID[ThisAircraft.protocol][0]);

        itoa(rx_packets_counter % 1000, buf, 10);
        tft->setCursor(tft->textWidth(" "), tft->height()/2);
        tft->print(buf);

        itoa(tx_packets_counter % 1000, buf, 10);
        tft->setCursor(tft->width()/2 + tft->textWidth(" "), tft->height()/2);
        tft->print(buf);

        TFT_display_frontpage = true;

      } else { /* TFT_display_frontpage */

        if (rx_packets_counter > prev_rx_packets_counter) {
          disp_value = rx_packets_counter % 1000;
          itoa(disp_value, buf, 10);

          if (disp_value < 10) {
            strcat_P(buf,PSTR("  "));
          } else {
            if (disp_value < 100) {
              strcat_P(buf,PSTR(" "));
            };
          }

          tft->setTextFont(4);
          tft->setTextSize(2);

          tft->setCursor(tft->textWidth(" "), tft->height()/2);
          tft->print(buf);

          prev_rx_packets_counter = rx_packets_counter;
        }
        if (tx_packets_counter > prev_tx_packets_counter) {
          disp_value = tx_packets_counter % 1000;
          itoa(disp_value, buf, 10);

          if (disp_value < 10) {
            strcat_P(buf,PSTR("  "));
          } else {
            if (disp_value < 100) {
              strcat_P(buf,PSTR(" "));
            };
          }

          tft->setTextFont(4);
          tft->setTextSize(2);

          tft->setCursor(tft->width()/2 + tft->textWidth(" "), tft->height()/2);
          tft->print(buf);

          prev_tx_packets_counter = tx_packets_counter;
        }
      }
    }

    break;
#endif /* LV_HOR_RES == 240 */

#if LV_HOR_RES == 135
  case DISPLAY_TFT_TTGO_135:
    if (tft) {
      if (!TFT_display_frontpage) {
        tft->fillScreen(TFT_NAVY);

        tft->setTextFont(2);
        tft->setTextSize(2);
        tft->setTextColor(TFT_WHITE, TFT_NAVY);

        tbw = tft->textWidth(ID_text);
        tbh = tft->fontHeight();

        tft->setCursor(tft->textWidth(" "), tft->height()/4 - tbh - 1);
        tft->print(ID_text);

        tbw = tft->textWidth(PROTOCOL_text);

        tft->setCursor(tft->width() - tbw - tft->textWidth(" "),
                       tft->height()/4 - tbh - 1);
        tft->print(PROTOCOL_text);

        tbw = tft->textWidth(RX_text);
        tbh = tft->fontHeight();

        tft->setCursor(tft->textWidth("   "), 3*tft->height()/4 - tbh - 1);
        tft->print(RX_text);

        tbw = tft->textWidth(TX_text);

        tft->setCursor(tft->width()/2 + tft->textWidth("   "),
                       3*tft->height()/4 - tbh - 1);
        tft->print(TX_text);

        tft->setTextFont(2);
        tft->setTextSize(3);

        snprintf (buf, sizeof(buf), "%06X", ThisAircraft.addr);

        tbw = tft->textWidth(buf);
        tbh = tft->fontHeight();

        tft->setCursor(tft->textWidth(" "), tft->height()/4 - 7);
        tft->print(buf);

        tbw = tft->textWidth("O");

        tft->setCursor(tft->width() - tbw - tft->textWidth(" "),
                       tft->height()/4 - 7);
        tft->print(Protocol_ID[ThisAircraft.protocol][0]);

        itoa(rx_packets_counter % 1000, buf, 10);
        tft->setCursor(tft->textWidth(" "), 3*tft->height()/4 - 7);
        tft->print(buf);

        itoa(tx_packets_counter % 1000, buf, 10);
        tft->setCursor(tft->width()/2 + tft->textWidth(" "), 3*tft->height()/4 - 7);
        tft->print(buf);

        TFT_display_frontpage = true;

      } else { /* TFT_display_frontpage */

        if (rx_packets_counter > prev_rx_packets_counter) {
          disp_value = rx_packets_counter % 1000;
          itoa(disp_value, buf, 10);

          if (disp_value < 10) {
            strcat_P(buf,PSTR("  "));
          } else {
            if (disp_value < 100) {
              strcat_P(buf,PSTR(" "));
            };
          }

          tft->setTextFont(2);
          tft->setTextSize(3);

          tft->setCursor(tft->textWidth(" "), 3*tft->height()/4 - 7);
          tft->print(buf);

          prev_rx_packets_counter = rx_packets_counter;
        }
        if (tx_packets_counter > prev_tx_packets_counter) {
          disp_value = tx_packets_counter % 1000;
          itoa(disp_value, buf, 10);

          if (disp_value < 10) {
            strcat_P(buf,PSTR("  "));
          } else {
            if (disp_value < 100) {
              strcat_P(buf,PSTR(" "));
            };
          }

          tft->setTextFont(2);
          tft->setTextSize(3);

          tft->setCursor(tft->width()/2 + tft->textWidth(" "), 3*tft->height()/4 - 7);
          tft->print(buf);

          prev_tx_packets_counter = tx_packets_counter;
        }
      }
    }

    break;

#endif /* LV_HOR_RES == 135 */
#endif /* USE_TFT */

#if defined(USE_OLED)
  case DISPLAY_OLED_TTGO:
  case DISPLAY_OLED_HELTEC:
  case DISPLAY_OLED_1_3:
    OLED_loop();
    break;
#endif /* USE_OLED */

  case DISPLAY_NONE:
  default:
    break;
  }
}

static void ESP32_Display_fini(int reason)
{
  if (hw_info.model == SOFTRF_MODEL_PRIME_MK2) {
    if (settings->volume != BUZZER_OFF && settings->volume != BUZZER_EXT) {
      Buzzer_tone(640, 250);
      Buzzer_tone(440, 250);
    }
    if (hw_info.revision >= 8) {
      // if Serial2 or external GNSS used this pin,
      // turn red LED back on to show shutdown in progress
#if (Serial2TxPin == SOC_GPIO_PIN_TBEAM_LED_V11)
      if (has_serial2 || settings->rx1090 != ADSB_RX_NONE) {
          has_serial2 = false;    // in NMEA.cpp
          delay(300);
          Serial2.end();
      } else
#endif
      if (settings->gnss_pins == EXT_GNSS_39_4) {
          Serial_GNSS_In.end();
      }
      delay(300);
      pinMode(SOC_GPIO_PIN_TBEAM_LED_V11, OUTPUT);
      digitalWrite(SOC_GPIO_PIN_TBEAM_LED_V11, LOW);
#if 0
    // skip, since we removed the shutdown function from the pushbutton in v0.7
    // and replaced it with long-press for the alarm demo (short is for pages)
    } else if (hw_info.revision == 5) {
      // if external GNSS used this pin, turn LED back on to show shutdown in progress
      if (settings->gnss_pins == EXT_GNSS_15_14) {
          Serial_GNSS_In.end();
          delay(300);
          pinMode(SOC_GPIO_PIN_TBEAM_LED_V05, OUTPUT);
          digitalWrite(SOC_GPIO_PIN_TBEAM_LED_V05, HIGH);
      }
#endif
    }
  }

  switch (hw_info.display)
  {
#if defined(USE_OLED)
  case DISPLAY_OLED_TTGO:
  case DISPLAY_OLED_HELTEC:
  case DISPLAY_OLED_1_3:

    SoC->ADB_ops && SoC->ADB_ops->fini();

    OLED_fini(reason);

    if (u8x8) {

      delay(3000); /* Keep shutdown message on OLED for 3 seconds */

      u8x8->noDisplay();
    }
    break;
#endif /* USE_OLED */

#if defined(USE_TFT)
  case DISPLAY_TFT_TTGO_240:
  case DISPLAY_TFT_TTGO_135:
    if (tft) {
        int level;
        const char *msg = (reason == SOFTRF_SHUTDOWN_LOWBAT) ?
                   "LOW BAT" : "  OFF  ";

        for (level = 250; level >= 0; level -= 25) {
          TFT_backlight_adjust(level);
          delay(100);
        }

        tft->fillScreen(TFT_NAVY);

        tft->setTextFont(4);
        tft->setTextSize(2);
        tft->setTextColor(TFT_WHITE, TFT_NAVY);

        uint16_t tbw = tft->textWidth(msg);
        uint16_t tbh = tft->fontHeight();

        tft->setCursor((tft->width() - tbw)/2, (tft->height() - tbh)/2);
        tft->print(msg);

        for (level = 0; level <= 250; level += 25) {
          TFT_backlight_adjust(level);
          delay(100);
        }

        delay(2000);

        for (level = 250; level >= 0; level -= 25) {
          TFT_backlight_adjust(level);
          delay(100);
        }

        TFT_backlight_off();
        int bl_pin = (esp32_board == ESP32_S2_T8_V1_1) ?
                     SOC_GPIO_PIN_T8_S2_TFT_BL : SOC_GPIO_PIN_TWATCH_TFT_BL;

        ledcDetachPin(bl_pin);
        pinMode(bl_pin, INPUT_PULLDOWN);

        tft->fillScreen(TFT_NAVY);
        TFT_off();
    }
    break;
#endif /* USE_TFT */

  case DISPLAY_NONE:
  default:
    delay(2000);   // chance to observe the red LED
    break;
  }
}

int ESP32_VbusVoltage()
{
    if (PMU)
        return PMU->getVbusVoltage();
    return 0;
}

/* external Power is available from USB port (VBUS0) */
bool ESP32_onExternalPower() {
    switch (hw_info.pmu)
    {
    case PMU_AXP192:
        //return (PMU->getVbusVoltage() > 4000)
        return PMU->isVbusIn();
        break;
    case PMU_AXP2101:
        return PMU->isVbusIn();
        break;
    case PMU_NONE:
    default:
        return (Battery_voltage() >= 4.1);
        break;
    }
}

/* battery Power is available */
bool ESP32_onBatteryPower() {
    switch (hw_info.pmu)
    {
    case PMU_AXP192:
    case PMU_AXP2101:
        return PMU->isBatteryConnect();
        break;
    case PMU_NONE:
    default:
        return (Battery_voltage() >= 3.0);
        break;
    }
}

static void ESP32_Battery_setup()
{
  if ((hw_info.model    == SOFTRF_MODEL_PRIME_MK2 &&
       hw_info.revision >= 8)                     ||
       hw_info.model    == SOFTRF_MODEL_PRIME_MK3 ||
       hw_info.model    == SOFTRF_MODEL_SKYWATCH) {

    /* T-Beam v08+, T-Beam Supreme and T-Watch have PMU */

    /* TBD */
  } else {
#if defined(CONFIG_IDF_TARGET_ESP32)
    if (hw_info.model == SOFTRF_MODEL_PRIME_MK2 ||
            (esp32_board == ESP32_TTGO_V2_OLED && hw_info.revision == 16)) {
        if (ESP32_pin_reserved(35, false, "Battery ADC")) return;
        calibrate_voltage(ADC1_GPIO35_CHANNEL);
    } else {
        calibrate_voltage(ADC1_GPIO36_CHANNEL);
    }
#elif defined(CONFIG_IDF_TARGET_ESP32S2)
    calibrate_voltage(ADC1_GPIO9_CHANNEL);
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
    calibrate_voltage(ADC1_GPIO2_CHANNEL);
#else
#error "This ESP32 family build variant is not supported!"
#endif /* CONFIG_IDF_TARGET_ESP32 */
  }
}

static float ESP32_Battery_param(uint8_t param)
{
  float rval, voltage;

  switch (param)
  {
  case BATTERY_PARAM_THRESHOLD:
    rval = (hw_info.model == SOFTRF_MODEL_PRIME_MK2  && hw_info.revision ==  8) ?
            BATTERY_THRESHOLD_LIPO /* + 0.1 */ :
            hw_info.model == SOFTRF_MODEL_PRIME_MK2  ||
            hw_info.model == SOFTRF_MODEL_PRIME_MK3  || /* TBD */
            /* TTGO T3 V2.1.6 */
           (hw_info.model == SOFTRF_MODEL_STANDALONE && hw_info.revision == 16) ?
            BATTERY_THRESHOLD_LIPO : BATTERY_THRESHOLD_NIMHX2;
    break;

  case BATTERY_PARAM_CUTOFF:
    rval = (hw_info.model == SOFTRF_MODEL_PRIME_MK2  && hw_info.revision ==  8) ?
            BATTERY_CUTOFF_LIPO /* + 0.2 */ :
            hw_info.model == SOFTRF_MODEL_PRIME_MK2  ||
            hw_info.model == SOFTRF_MODEL_PRIME_MK3  || /* TBD */
            /* TTGO T3 V2.1.6 */
           (hw_info.model == SOFTRF_MODEL_STANDALONE && hw_info.revision == 16) ?
            BATTERY_CUTOFF_LIPO : BATTERY_CUTOFF_NIMHX2;
    break;

  case BATTERY_PARAM_CHARGE:
    // assume a LiPo battery, for which full=4.2, threshold=3.5 and cutoff=3.2
    voltage = Battery_voltage();
    if (voltage < BATTERY_CUTOFF_LIPO)
      return 0;
    if (voltage > BATTERY_FULL_LIPO)
      return 100.0;
    if (voltage < BATTERY_THRESHOLD_LIPO) {
      return ((voltage - BATTERY_CUTOFF_LIPO)
          * (10.0 / (BATTERY_THRESHOLD_LIPO - BATTERY_CUTOFF_LIPO)));   // 0 to 10% over 0.3V
    }
    return (10.0 + (voltage - BATTERY_THRESHOLD_LIPO)
                 * (90.0 / (BATTERY_FULL_LIPO - BATTERY_THRESHOLD_LIPO)));   // 10 to 100% over 0.7V
    break;

  case BATTERY_PARAM_VOLTAGE:
  default:
    voltage = 0.0;

    switch (hw_info.pmu)
    {
    case PMU_AXP192:
    case PMU_AXP2101:
      if (PMU->isBatteryConnect()) {
        voltage = PMU->getBattVoltage();
      }
      break;

    case PMU_NONE:
    default:
      voltage = (float) read_voltage();

      /* T-Beam v02-v07 and T3 V2.1.6 have voltage divider 100k/100k on board */
      if (hw_info.model == SOFTRF_MODEL_PRIME_MK2   ||
         (esp32_board   == ESP32_TTGO_V2_OLED && hw_info.revision == 16) ||
         (esp32_board   == ESP32_S2_T8_V1_1)) {
        voltage += voltage;
      }
      break;
    }

    rval = voltage * 0.001;
    break;
  }

  return rval;
}

static void IRAM_ATTR ESP32_GNSS_PPS_Interrupt_handler()
{
  portENTER_CRITICAL_ISR(&GNSS_PPS_mutex);
  PPS_TimeMarker = millis();    /* millis() has IRAM_ATTR */
  portEXIT_CRITICAL_ISR(&GNSS_PPS_mutex);
}

static unsigned long ESP32_get_PPS_TimeMarker()
{
  unsigned long rval;
  portENTER_CRITICAL_ISR(&GNSS_PPS_mutex);
  rval = PPS_TimeMarker;
  portEXIT_CRITICAL_ISR(&GNSS_PPS_mutex);
  return rval;
}

static bool ESP32_Baro_setup()
{
  if (hw_info.model == SOFTRF_MODEL_SKYWATCH) {

    return false;

  } else if (esp32_board == ESP32_S2_T8_V1_1) {

    Wire.setPins(SOC_GPIO_PIN_T8_S2_SDA, SOC_GPIO_PIN_T8_S2_SCL);

  } else if (esp32_board == ESP32_S3_DEVKIT ||
             esp32_board == ESP32_TTGO_T_BEAM_SUPREME) {

    Wire.setPins(SOC_GPIO_PIN_S3_SDA, SOC_GPIO_PIN_S3_SCL);

  } else if (hw_info.model != SOFTRF_MODEL_PRIME_MK2) {

    if ((hw_info.rf != RF_IC_SX1276 && hw_info.rf != RF_IC_SX1262) ||
        RF_SX12XX_RST_is_connected) {
      return false;
    }

#if DEBUG
    Serial.println(F("INFO: RESET pin of SX12xx radio is not connected to MCU."));
#endif

    Wire.setPins(SOC_GPIO_PIN_SDA, SOC_GPIO_PIN_SCL);

  } else {   // is SOFTRF_MODEL_PRIME_MK2

//    if (hw_info.revision == 2 && RF_SX12XX_RST_is_connected) {
//      hw_info.revision = 5;
//    }

    if (Baro_probe()) {                          // found baro sensor (it tries both Wires)
//#if (Serial2TxPin == SOC_GPIO_PIN_TBEAM_SCL)
//        settings->baudrate2 == BAUD_DEFAULT;   // disable Serial2 since uses pins 2,13
//#endif
        Serial.println(F("BMP found"));
        return true;
    }

    return false;
  }

  return true;
}

static void ESP32_UATSerial_begin(unsigned long baud)
{
  /* open Standalone's I2C/UATSerial port */
  UATSerial.begin(baud, SERIAL_IN_BITS, SOC_GPIO_PIN_CE, SOC_GPIO_PIN_PWR);
}

static void ESP32_UATSerial_updateBaudRate(unsigned long baud)
{
  UATSerial.updateBaudRate(baud);
}

static void ESP32_UATModule_restart()
{
  digitalWrite(SOC_GPIO_PIN_TXE, LOW);
  pinMode(SOC_GPIO_PIN_TXE, OUTPUT);

  delay(100);

  digitalWrite(SOC_GPIO_PIN_TXE, HIGH);

  delay(100);

  pinMode(SOC_GPIO_PIN_TXE, INPUT);
}

static void ESP32_WDT_setup()
{
  enableLoopWDT();
}

static void ESP32_WDT_fini()
{
  disableLoopWDT();
}

#include <AceButton.h>
using namespace ace_button;

AceButton button_1(SOC_GPIO_PIN_TBEAM_V05_BUTTON);
AceButton button_2(SOC_GPIO_PIN_TBEAM_V08_BUTTON);

// The event handlers for the buttons

// this is for the first pushbutton on a T-Beam v0.x
void handleEvent(AceButton* button, uint8_t eventType,
    uint8_t buttonState) {

  if (button != &button_1)
      return;

  switch (eventType) {
    case AceButton::kEventReleased:
      break;
    case AceButton::kEventClicked:
#if defined(USE_OLED)
      OLED_Next_Page();
#endif
      break;
    case AceButton::kEventDoubleClicked:
      test_mode = !test_mode;
      if (test_mode) {
          OLED_msg("TEST",   " MODE");
          Serial.println("Test Mode on");
      } else {
          OLED_msg("NOTEST", " MODE");
          Serial.println("Test Mode off");
      }
      do_test_mode();
      break;
    case AceButton::kEventLongPressed:
      // v1.x shutdown is via PMU interrupt
      // skip v0.7 shutdown, should use slide switch instead
      if (hw_info.model == SOFTRF_MODEL_PRIME_MK2  && hw_info.revision <  8) {
          if (settings->mode != SOFTRF_MODE_NORMAL) {
              settings->mode = SOFTRF_MODE_NORMAL;
              OLED_msg("NORMAL", " MODE");
          } else if (ThisAircraft.aircraft_type == AIRCRAFT_TYPE_WINCH) {
              if (settings->txpower == RF_TX_POWER_OFF) {
                  settings->txpower = RF_TX_POWER_FULL;
              } else {
                  settings->txpower = RF_TX_POWER_OFF;
              }
          } else if (millis() > 10000) {
              SetupTimeMarker = millis();
              do_alarm_demo = true;
              OLED_msg("ALARM", " DEMO");
#if defined(USE_SD_CARD)
              closeSDlog();
              closeNMEAlog();
#endif
              //if (settings->logflight == FLIGHT_LOG_ALWAYS)
              completeFlightLog();
          }
      }
      break;
  }
}

// this is for the middle button on a T-Beam v1.x
void handleEvent2(AceButton* button, uint8_t eventType,
    uint8_t buttonState) {

  if (button != &button_2)
      return;

  // Click middle button on T-Beam to trigger alarm demo.
  // (Long-press the first button on the T-Beam v0.7)
  // But in winch mode the button turns transmissions on/off instead.
  // Can also double-click middle button to toggle test_mode
  // Can also long-press middle button to turn on "landed_out_mode"

  switch (eventType) {
    case AceButton::kEventReleased:
      break;
    case AceButton::kEventClicked:
      if (millis() < 10000 || settings->mode != SOFTRF_MODE_NORMAL) {
          settings->mode = SOFTRF_MODE_NORMAL;
          OLED_msg("NORMAL", " MODE");
      } else if (ThisAircraft.aircraft_type == AIRCRAFT_TYPE_WINCH) {
          if (settings->txpower == RF_TX_POWER_OFF)
              settings->txpower = RF_TX_POWER_FULL;
          else
              settings->txpower = RF_TX_POWER_OFF;
      } else /* if (!bt_turned_off) */ {
          SetupTimeMarker = millis();
          do_alarm_demo = true;
          OLED_msg("ALARM", " DEMO");
      }
      break;
    case AceButton::kEventDoubleClicked:
      test_mode = !test_mode;
      if (test_mode) {
          OLED_msg("TEST",   " MODE");
          Serial.println("Test Mode on");
      } else {
          OLED_msg("NOTEST", " MODE");
          Serial.println("Test Mode off");
      }
      do_test_mode();
      break;
    case AceButton::kEventLongPressed:
#if 1
      if (landed_out_mode) {
          landed_out_mode = false;
          OLED_msg("NORMAL", "MODE");
          Serial.println("landed_out_mode off");
      } else {
          landed_out_mode = true;
          OLED_msg("LANDED", "OUT");
          Serial.println("landed_out_mode on");
      }
#else
      if (settings->bluetooth != BLUETOOTH_OFF && millis() > 15000) {
          if (SoC->Bluetooth_ops) {
              Serial.println(F("Turning off Bluetooth due to button press"));
              SoC->Bluetooth_ops->fini();
              bt_turned_off = true;
          }
      }
#endif
      break;
  }
}

/* Callbacks for push button interrupt */
void onPageButtonEvent() {
  button_1.check();
}

static void ESP32_Button_setup()
{
  if (hw_info.model == SOFTRF_MODEL_PRIME_MK2)
        list_reserved_pins();

  if (( hw_info.model == SOFTRF_MODEL_PRIME_MK2 &&
       (hw_info.revision == 2 || hw_info.revision == 5)) ||
       esp32_board == ESP32_S2_T8_V1_1 ||
       esp32_board == ESP32_S3_DEVKIT) {

    int button_pin = (esp32_board == ESP32_S2_T8_V1_1) ?
                     SOC_GPIO_PIN_T8_S2_BUTTON :
                     (esp32_board == ESP32_S3_DEVKIT) ?
                     SOC_GPIO_PIN_S3_BUTTON :
                     SOC_GPIO_PIN_TBEAM_V05_BUTTON;

    if (hw_info.model == SOFTRF_MODEL_PRIME_MK2)
        if (ESP32_pin_reserved(button_pin, false, "Button1")) return;

    // Button(s) uses external pull up resistor.
    pinMode(button_pin, button_pin == 0 ? INPUT_PULLUP : INPUT);

    button_1.init(button_pin);

    // Configure the ButtonConfig with the event handler, and enable all higher
    // level events.
    ButtonConfig* PageButtonConfig = button_1.getButtonConfig();
    PageButtonConfig->setEventHandler(handleEvent);
    //PageButtonConfig->setFeature(ButtonConfig::kFeatureClick);
    PageButtonConfig->setFeature(ButtonConfig::kFeatureDoubleClick);
    PageButtonConfig->setFeature(ButtonConfig::kFeatureLongPress);
    PageButtonConfig->setFeature(ButtonConfig::kFeatureSuppressAfterClick);
    PageButtonConfig->setFeature(ButtonConfig::kFeatureSuppressAfterDoubleClick);
    PageButtonConfig->setFeature(ButtonConfig::kFeatureSuppressClickBeforeDoubleClick);
//  PageButtonConfig->setDebounceDelay(15);
    PageButtonConfig->setClickDelay(300);
    PageButtonConfig->setDoubleClickDelay(600);
    PageButtonConfig->setLongPressDelay(2000);
  }

  if (middle_button_pin != SOC_UNUSED_PIN) {
    if (hw_info.model == SOFTRF_MODEL_PRIME_MK2) {
        if (ESP32_pin_reserved(middle_button_pin, false, "Button2")) return;
    }
    button_2.init(middle_button_pin);
    ButtonConfig* MidButtonConfig = button_2.getButtonConfig();
    MidButtonConfig->setEventHandler(handleEvent2);
    //MidButtonConfig->setFeature(ButtonConfig::kFeatureClick);
    MidButtonConfig->setFeature(ButtonConfig::kFeatureDoubleClick);
    MidButtonConfig->setFeature(ButtonConfig::kFeatureLongPress);
    MidButtonConfig->setFeature(ButtonConfig::kFeatureSuppressAfterClick);
    MidButtonConfig->setFeature(ButtonConfig::kFeatureSuppressAfterDoubleClick);
    MidButtonConfig->setFeature(ButtonConfig::kFeatureSuppressClickBeforeDoubleClick);
//  MidButtonConfig->setDebounceDelay(15);
    MidButtonConfig->setClickDelay(300);
    MidButtonConfig->setDoubleClickDelay(600);
    MidButtonConfig->setLongPressDelay(2000);
  }
}

static void ESP32_Button_loop()
{
  if (( hw_info.model == SOFTRF_MODEL_PRIME_MK2 &&
       (hw_info.revision == 2 || hw_info.revision == 5)) ||
       esp32_board == ESP32_S2_T8_V1_1 ||
       esp32_board == ESP32_S3_DEVKIT) {
    button_1.check();
  }

  if (middle_button_pin != SOC_UNUSED_PIN) {
    button_2.check();
  }
}

static void ESP32_Button_fini()
{
  if (esp32_board == ESP32_S2_T8_V1_1 ||
      esp32_board == ESP32_S3_DEVKIT) {
    int button_pin = esp32_board == ESP32_S2_T8_V1_1 ?
                     SOC_GPIO_PIN_T8_S2_BUTTON :
                     SOC_GPIO_PIN_S3_BUTTON;
    while (digitalRead(button_pin) == LOW);
  }
}

void ESP32_charge_mode()
{
    if ((has_axp == false)
    || (settings->power_ext)
    || (ESP32_onExternalPower() == false || ESP32_onBatteryPower() == false))
        return;

    bool normal_boot = false;
    if (middle_button_pin != SOC_UNUSED_PIN) {
        if (digitalRead(middle_button_pin) == LOW)
            normal_boot = true;
    }
    if (! normal_boot) {
        OLED_msg("CHARGE", "MODE");
        delay(2000);   // during which middle button may be pressed
    }
    if (middle_button_pin != SOC_UNUSED_PIN) {
        if (digitalRead(middle_button_pin) == LOW)
            normal_boot = true;
    }
    if (normal_boot) {
        OLED_msg("NORMAL", "BOOT...");
        delay(1000);
        OLED_no_msg();
        return;
    }

    Serial.println("\r\nCHARGE MODE");
    //delay(500);
    ESP32_Display_fini(SOFTRF_SHUTDOWN_CHARGE);
    ESP32_Button_fini();
    ESP32_fini(SOFTRF_SHUTDOWN_CHARGE);  // set blue LED to charge mode & shut down
}

#if defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3)

#define USB_TX_FIFO_SIZE (MAX_TRACKING_OBJECTS * 65 + 75 + 75 + 42 + 20)
#define USB_RX_FIFO_SIZE (256)

#if defined(USE_USB_HOST)

#include "usb/usb_host.h"

#include "usb_host.hpp"
#include "usb_acm.hpp"

#define USB_MAX_WRITE_CHUNK_SIZE 256

cbuf *USB_RX_FIFO, *USB_TX_FIFO;
USBhost host;           // host is required to detect any device, before USB class is initialized
USBacmDevice *device;   // when USB class is detected from descriptor

void acm_events(int event, void *data, size_t len)
{
    switch (event)
    {
    case CDC_CTRL_SET_CONTROL_LINE_STATE:
        log_i("CDC_CTRL_SET_CONTROL_LINE_STATE");
        device->setLineCoding(SERIAL_OUT_BR, 0, 0, 8);
        break;

    case CDC_DATA_IN:
    {
        device->INDATA();
        if (len > 0) {
          USB_RX_FIFO->write((char *) data,
                       USB_RX_FIFO->room() > len ?
                       len : USB_RX_FIFO->room());
        }
        break;
    }
    case CDC_DATA_OUT:

        break;

    case CDC_CTRL_SET_LINE_CODING:
        log_i("CDC_CTRL_SET_LINE_CODING");
        break;
    }
}

void client_event_callback(const usb_host_client_event_msg_t *event_msg, void *arg)
{
    if (event_msg->event == USB_HOST_CLIENT_EVENT_NEW_DEV)
    {
        host.open(event_msg);
        usb_device_info_t info = host.getDeviceInfo();
        log_i("device speed: %s, device address: %d, max ep_ctrl size: %d, config: %d", info.speed ? "USB_SPEED_FULL" : "USB_SPEED_LOW", info.dev_addr, info.bMaxPacketSize0, info.bConfigurationValue);
        const usb_device_desc_t *dev_desc = host.getDeviceDescriptor();
        int offset = 0;
        for (size_t i = 0; i < dev_desc->bNumConfigurations; i++)
        {
            const usb_config_desc_t *config_desc = host.getConfigurationDescriptor();
            for (size_t n = 0; n < config_desc->bNumInterfaces; n++)
            {
                const usb_intf_desc_t *intf = usb_parse_interface_descriptor(config_desc, n, 0, &offset);
                if (intf->bInterfaceClass == 0x0a) // CDC ACM
                {
                    device = new USBacmDevice(config_desc, &host);
                    n = config_desc->bNumInterfaces;
                    if (device)
                    {
                        device->init();
                        device->onEvent(acm_events);
                        device->setControlLine(1, 1);
                        device->INDATA();
                    }
                }

//                printf("config: %d[%d], interface: %d[%d], intf class: %d\n", i, dev_desc->bNumConfigurations, n, config_desc->bNumInterfaces, intf->bInterfaceClass);
            }
        }
    }
    else
    {
        log_w("DEVICE gone event");
        if (device)
        {
            device->deinit();
            delete(device);
        }
        device = NULL;
    }
}

static void ESP32S2_USB_setup()
{
  USB_RX_FIFO = new cbuf(USB_RX_FIFO_SIZE);
  USB_TX_FIFO = new cbuf(USB_TX_FIFO_SIZE);

  host.registerClientCb(client_event_callback);
  host.init();
}

static void ESP32S2_USB_loop()
{
    if (device && device->isConnected())
    {
      uint8_t chunk[USB_MAX_WRITE_CHUNK_SIZE];
      size_t size = (USB_TX_FIFO->available() < USB_MAX_WRITE_CHUNK_SIZE ?
                     USB_TX_FIFO->available() : USB_MAX_WRITE_CHUNK_SIZE);

      USB_TX_FIFO->read((char *) chunk, size);
      device->OUTDATA(chunk, size);
    }
}

static void ESP32S2_USB_fini()
{
    delete(USB_RX_FIFO);
    delete(USB_TX_FIFO);
}

static int ESP32S2_USB_available()
{
  int rval = 0;

  rval = USB_RX_FIFO->available();

  return rval;
}

static int ESP32S2_USB_read()
{
  int rval = -1;

  rval = USB_RX_FIFO->read();

  return rval;
}

static size_t ESP32S2_USB_write(const uint8_t *buffer, size_t size)
{
  size_t rval = size;

  rval = USB_TX_FIFO->write((char *) buffer,
                      (USB_TX_FIFO->room() > size ? size : USB_TX_FIFO->room()));

  return rval;
}

#elif ARDUINO_USB_CDC_ON_BOOT

#define USE_ASYNC_USB_OUTPUT
#define USBSerial                Serial

#if !ARDUINO_USB_MODE && defined(USE_ASYNC_USB_OUTPUT)
#define USB_MAX_WRITE_CHUNK_SIZE CONFIG_TINYUSB_CDC_TX_BUFSIZE

cbuf *USB_TX_FIFO;
#endif /* USE_ASYNC_USB_OUTPUT */

static void ESP32S2_USB_setup()
{
  USBSerial.setRxBufferSize(USB_RX_FIFO_SIZE);
#if ARDUINO_USB_MODE
  /* native CDC (HWCDC) */
  USBSerial.setTxBufferSize(USB_TX_FIFO_SIZE);
#elif defined(USE_ASYNC_USB_OUTPUT)
  USB_TX_FIFO = new cbuf(USB_TX_FIFO_SIZE);
#endif /* ARDUINO_USB_MODE */
}

static void ESP32S2_USB_loop()
{
#if !ARDUINO_USB_MODE && defined(USE_ASYNC_USB_OUTPUT)
  if (USBSerial)
  {
    uint8_t chunk[USB_MAX_WRITE_CHUNK_SIZE];

    size_t size = USBSerial.availableForWrite();
    size = (size > USB_MAX_WRITE_CHUNK_SIZE ? USB_MAX_WRITE_CHUNK_SIZE : size);
    size = (USB_TX_FIFO->available() < size ? USB_TX_FIFO->available() : size);

    USB_TX_FIFO->read((char *) chunk, size);
    USBSerial.write(chunk, size);
  }
#endif /* USE_ASYNC_USB_OUTPUT */
}

static void ESP32S2_USB_fini()
{
#if !ARDUINO_USB_MODE && defined(USE_ASYNC_USB_OUTPUT)
  delete(USB_TX_FIFO);
#endif /* USE_ASYNC_USB_OUTPUT */
}

static int ESP32S2_USB_available()
{
  int rval = 0;

  if (USBSerial) {
    rval = USBSerial.available();
  }

  return rval;
}

static int ESP32S2_USB_read()
{
  int rval = -1;

  if (USBSerial) {
    rval = USBSerial.read();
  }

  return rval;
}

static size_t ESP32S2_USB_write(const uint8_t *buffer, size_t size)
{
  size_t rval = size;

#if ARDUINO_USB_MODE
  /* Espressif native CDC (HWCDC) */
  if (USBSerial && (size < USBSerial.availableForWrite())) {
    rval = USBSerial.write(buffer, size);
  }
#else
  /* TinyUSB CDC (USBCDC) */
#if defined(USE_ASYNC_USB_OUTPUT)
  rval = USB_TX_FIFO->write((char *) buffer,
                      (USB_TX_FIFO->room() > size ? size : USB_TX_FIFO->room()));
#else
  if (USBSerial) {
    rval = USBSerial.write(buffer, size);
  }
#endif /* USE_ASYNC_USB_OUTPUT */
#endif /* ARDUINO_USB_MODE */

  return rval;
}
#endif /* USE_USB_HOST || ARDUINO_USB_CDC_ON_BOOT */

#if ARDUINO_USB_CDC_ON_BOOT || defined(USE_USB_HOST)
IODev_ops_t ESP32S2_USBSerial_ops = {
  "ESP32S2 USB",
  ESP32S2_USB_setup,
  ESP32S2_USB_loop,
  ESP32S2_USB_fini,
  ESP32S2_USB_available,
  ESP32S2_USB_read,
  ESP32S2_USB_write
};
#endif /* USE_USB_HOST || ARDUINO_USB_CDC_ON_BOOT */
#endif /* CONFIG_IDF_TARGET_ESP32S2 */

#if defined(CONFIG_IDF_TARGET_ESP32S3)
static bool ESP32_ADB_setup()
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

static bool ESP32_ADB_fini()
{
  if (ADB_is_open) {
    ucdb.close();
    ADB_is_open = false;
  }

  return !ADB_is_open;
}

/*
 * One aircraft CDB (20000+ records) query takes:
 * 1)     FOUND : xxx milliseconds
 * 2) NOT FOUND : xxx milliseconds
 */
static bool ESP32_ADB_query(uint8_t type, uint32_t id, char *buf, size_t size)
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

DB_ops_t ESP32_ADB_ops = {
  ESP32_ADB_setup,
  ESP32_ADB_fini,
  ESP32_ADB_query
};
#endif /* CONFIG_IDF_TARGET_ESP32S3 */

const SoC_ops_t ESP32_ops = {
#if defined(CONFIG_IDF_TARGET_ESP32)
  SOC_ESP32,
  "ESP32",
#elif defined(CONFIG_IDF_TARGET_ESP32S2)
  SOC_ESP32S2,
  "ESP32-S2",
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
  SOC_ESP32S3,
  "ESP32-S3",
#endif /* CONFIG_IDF_TARGET_ESP32-S2-S3 */
  ESP32_setup,
  ESP32_post_init,
  ESP32_loop,
  ESP32_fini,
  ESP32_reset,
  ESP32_getChipId,
  ESP32_getResetInfoPtr,
  ESP32_getResetInfo,
  ESP32_getResetReason,
  ESP32_getFreeHeap,
  ESP32_random,
  ESP32_Buzzer_test,
  ESP32_Buzzer_tone,
  ESP32_maxSketchSpace,
  ESP32_WiFi_set_param,
  ESP32_WiFi_transmit_UDP,
  ESP32_WiFiUDP_stopAll,
  ESP32_WiFi_hostname,
  ESP32_WiFi_clients_count,
  ESP32_EEPROM_begin,
  ESP32_EEPROM_extension,
  ESP32_SPI_begin,
  ESP32_swSer_begin,
  ESP32_swSer_enableRx,
#if !defined(CONFIG_IDF_TARGET_ESP32S2)
  &ESP32_Bluetooth_ops,
#else
  NULL,
#endif /* CONFIG_IDF_TARGET_ESP32S2 */
#if (defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3)) && \
   (ARDUINO_USB_CDC_ON_BOOT || defined(USE_USB_HOST))
  &ESP32S2_USBSerial_ops,
#else
  NULL,
#endif /* USE_USB_HOST */
  NULL,
  ESP32_Display_setup,
  ESP32_Display_loop,
  ESP32_Display_fini,
  ESP32_Battery_setup,
  ESP32_Battery_param,
  ESP32_GNSS_PPS_Interrupt_handler,
  ESP32_get_PPS_TimeMarker,
  ESP32_Baro_setup,
  ESP32_UATSerial_begin,
  ESP32_UATModule_restart,
  ESP32_WDT_setup,
  ESP32_WDT_fini,
  ESP32_Button_setup,
  ESP32_Button_loop,
  ESP32_Button_fini,
#if defined(CONFIG_IDF_TARGET_ESP32S3)
  &ESP32_ADB_ops
#else
  NULL
#endif /* CONFIG_IDF_TARGET_ESP32S3 */
};

#endif /* ESP32 */
