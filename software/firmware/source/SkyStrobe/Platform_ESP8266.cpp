/*
 * Platform_ESP8266.cpp
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
#if defined(ESP8266)

#include "SoCHelper.h"
#include "EEPROMHelper.h"
#include "WiFiHelper.h"

#include "SkyStrobe.h"

Exp_SoftwareSerial SerialInput(SOC_GPIO_PIN_SWSER_RX, SOC_GPIO_PIN_SWSER_TX , false, 256);

ESP8266WebServer server ( 80 );

static void ESP8266_setup()
{

}

static void ESP8266_fini()
{

}

static uint32_t ESP8266_getChipId()
{
  return ESP.getChipId();
}

static bool ESP8266_EEPROM_begin(size_t size)
{
  EEPROM.begin(size);
  return true;
}

static void ESP8266_WiFi_setOutputPower(int dB)
{
  WiFi.setOutputPower(dB);
}

static bool ESP8266_WiFi_hostname(String aHostname)
{
  return WiFi.hostname(aHostname);
}

static void ESP8266_swSer_begin(unsigned long baud)
{
  SerialInput.begin(baud);
}

static void ESP8266_swSer_enableRx(boolean arg)
{
  SerialInput.enableRx(arg);
}

static uint32_t ESP8266_maxSketchSpace()
{
  return (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
}

static void ESP8266_WiFiUDP_stopAll()
{
  WiFiUDP::stopAll();
}

static void ESP8266_Battery_setup()
{

}

static float ESP8266_Battery_voltage()
{
  return analogRead (SOC_GPIO_PIN_BATTERY) / SOC_A0_VOLTAGE_DIVIDER ;
}

static void ESP8266_WiFi_set_param(int ndx, int value)
{
  switch (ndx)
  {
  case WIFI_PARAM_TX_POWER:
    WiFi.setOutputPower(value);
    break;
  case WIFI_PARAM_DHCP_LEASE_TIME:
    if (WiFi.getMode() == WIFI_AP) {
      wifi_softap_set_dhcps_lease_time((uint32) (value * 60)); /* in minutes */
    }
    break;
  default:
    break;
  }
}

static size_t ESP8266_WiFi_Receive_UDP(uint8_t *buf, size_t max_size)
{
  return WiFi_Receive_UDP(buf, max_size);
}

static void ESP8266_WiFi_Transmit_UDP(int port, byte *buf, size_t size)
{
  IPAddress ClientIP;
  struct station_info *stat_info;
  WiFiMode_t mode = WiFi.getMode();

  switch (mode)
  {
  case WIFI_STA:
    ClientIP = ESP8266_WiFi_get_broadcast();

    Serial_GNSS_In.enableRx(false);

    Uni_Udp.beginPacket(ClientIP, port);
    Uni_Udp.write(buf, size);
    Uni_Udp.endPacket();

    Serial_GNSS_In.enableRx(true);

    break;
  case WIFI_AP:
    stat_info = wifi_softap_get_station_info();

    while (stat_info != NULL) {
      ClientIP = stat_info->ip.addr;

      Serial_GNSS_In.enableRx(false);

      Uni_Udp.beginPacket(ClientIP, port);
      Uni_Udp.write(buf, size);
      Uni_Udp.endPacket();

      Serial_GNSS_In.enableRx(true);

      stat_info = STAILQ_NEXT(stat_info, next);
    }
    wifi_softap_free_station_info();
    break;
  case WIFI_OFF:
  default:
    break;
  }
}

static int ESP8266_WiFi_clients_count()
{
  struct station_info *stat_info;
  int clients = 0;
  WiFiMode_t mode = WiFi.getMode();

  switch (mode)
  {
  case WIFI_AP:
    stat_info = wifi_softap_get_station_info();

    while (stat_info != NULL) {
      clients++;

      stat_info = STAILQ_NEXT(stat_info, next);
    }
    wifi_softap_free_station_info();

    return clients;
  case WIFI_STA:
  default:
    return -1; /* error */
  }
}

static void ESP8266_Button_setup()
{

}

static void ESP8266_Button_loop()
{

}

static void ESP8266_Button_fini()
{

}

static void ESP8266_WDT_setup()
{
  /* TBD */
}

static void ESP8266_WDT_fini()
{
  /* TBD */
}

const SoC_ops_t ESP8266_ops = {
  SOC_ESP8266,
  "ESP8266",
  ESP8266_setup,
  ESP8266_fini,
  ESP8266_getChipId,
  ESP8266_EEPROM_begin,
  ESP8266_WiFi_setOutputPower,
  ESP8266_WiFi_hostname,
  ESP8266_swSer_begin,
  ESP8266_swSer_enableRx,
  ESP8266_maxSketchSpace,
  ESP8266_WiFiUDP_stopAll,
  ESP8266_Battery_setup,
  ESP8266_Battery_voltage,
  ESP8266_WiFi_set_param,
  ESP8266_WiFi_Receive_UDP,
  ESP8266_WiFi_Transmit_UDP,
  ESP8266_WiFi_clients_count,
#if defined(EXCLUDE_BUTTONS)
  NULL, NULL, NULL,
#else  ESP8266_Button_setup,
  ESP8266_Button_loop,
  ESP8266_Button_fini,
#endif
  ESP8266_WDT_setup,
  ESP8266_WDT_fini,
  NULL
};

#endif /* ESP8266 */
