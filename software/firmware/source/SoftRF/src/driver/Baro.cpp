/*
 * BaroHelper.cpp
 * Copyright (C) 2018-2021 Linar Yusupov
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

#include "Baro.h"
#include "Settings.h"
#include "Battery.h"
#include "../protocol/data/NMEA.h"

// including BMP180 & MPL3115A2 still hangs at probe() even with ESP32 Core 2.0.3
// #define EXCLUDE_BMP280
#define EXCLUDE_BMP180
#define EXCLUDE_MPL3115A2

barochip_ops_t *baro_chip = NULL;

#if defined(EXCLUDE_BMP180) && defined(EXCLUDE_BMP280) && defined(EXCLUDE_MPL3115A2)
byte  Baro_setup()        {return BARO_MODULE_NONE;}
bool  Baro_probe()        {return false;}
void  Baro_loop()         {}
float Baro_altitude()     {return 0;}
float Baro_pressure()     {return 0;}
float Baro_temperature()  {return 0;}

#else

#if !defined(EXCLUDE_BMP180)
#include <Adafruit_BMP085.h>
#endif /* EXCLUDE_BMP180 */
#if !defined(EXCLUDE_BMP280)
#include <Adafruit_BMP280.h>
#endif /* EXCLUDE_BMP280 */
#if !defined(EXCLUDE_MPL3115A2)
#include <Adafruit_MPL3115A2.h>
#endif /* EXCLUDE_MPL3115A2 */

#include <TinyGPS++.h>

#if !defined(EXCLUDE_BMP180)
Adafruit_BMP085 bmp180;
#endif /* EXCLUDE_BMP180 */
#if !defined(EXCLUDE_BMP280)
Adafruit_BMP280 bmp280;
#endif /* EXCLUDE_BMP280 */
#if !defined(EXCLUDE_MPL3115A2)
Adafruit_MPL3115A2 mpl3115a2 = Adafruit_MPL3115A2();
#endif /* EXCLUDE_MPL3115A2 */

static float Baro_altitude_cache            = 0;
static float Baro_pressure_cache            = 101325;
static float Baro_temperature_cache         = 0;

static unsigned long BaroAltitudeTimeMarker = 0;
static unsigned long BaroPresTempTimeMarker = 0;
static float prev_pressure_altitude         = 0;

static float Baro_VS[VS_AVERAGING_FACTOR];
static int avg_ndx = 0;

static float altitude_from_pressure()
{
    // ensure the float (not double) version of pow() is called
    return  44330.0f * (1.0f - pow(Baro_pressure_cache * (1.0f / 101325.0f), 0.1903f));
#if 0
// polynomial approximation of altitude as a function of pressure ratio
// - courtesy of Rick Sheppe
// - may or may not be faster to compute than the formula using pow()
    float ratio = Baro_pressure_cache * (1.0 / 101325.0);
    float sum = ratio * -1.752317e+04;
    sum = ratio * (sum + 6.801427e+04);
    sum = ratio * (sum - 1.087470e+05);
    sum = ratio * (sum + 9.498147e+04);
    sum = ratio * (sum - 5.669573e+04);
    return (sum + 1.997137e+04);
#endif
}

#if !defined(EXCLUDE_BMP180)

static bool bmp180_probe()
{
#if defined(ESP32)
  if (hw_info.model == SOFTRF_MODEL_PRIME_MK2) {
      if (settings->gnss_pins != EXT_GNSS_13_2) {
          bmp180.setWire(&Wire);
          if (bmp180.begin())
              return true;
      }
      bmp180.setWire(&Wire1);
      if (bmp180.begin())
          return true;
  } else
#endif
  {
      bmp180.setWire(&Wire);
      if (bmp180.begin())
          return true;
  }
  return false;
}

static void bmp180_setup()
{
  Serial.print(F("Temperature = "));
  Serial.print(bmp180.readTemperature());
  Serial.println(F(" *C"));

  Baro_pressure_cache = (float) bmp180.readPressure();
  Serial.print(F("Pressure = "));
  Serial.print(Baro_pressure_cache);
  Serial.println(F(" Pa"));
  
  // Calculate altitude assuming 'standard' barometric
  // pressure of 1013.25 millibar = 101325 Pascal
  Serial.print(F("Altitude using library = "));
  Serial.print(bmp180.readAltitude());
  Serial.println(F(" meters"));
  Serial.print(F("Altitude using our code = "));
  Serial.print(altitude_from_pressure());
  
  Serial.println();
  delay(500);
}

static float bmp180_altitude()
{
  // return bmp180.readAltitude(sealevelPressure);
  Baro_pressure_cache = (float) bmp180.readPressure();
  return altitude_from_pressure();
}

static float bmp180_pressure()
{
  //return (float) bmp180.readPressure();
  return Baro_pressure_cache;   // assume altitude has been read recently
}

static float bmp180_temperature()
{
  return bmp180.readTemperature();
}

barochip_ops_t bmp180_ops = {
  BARO_MODULE_BMP180,
  "BMP180",
  bmp180_probe,
  bmp180_setup,
  bmp180_altitude,
  bmp180_pressure,
  bmp180_temperature
};
#endif /* EXCLUDE_BMP180 */

#if !defined(EXCLUDE_BMP280)
static bool bmp280_probe()
{
// is there a problem with probing both Wires on oldest T-Beams?
//    if (hw_info.revision == 2)
//      return false;
#if defined(ESP32)
  if (hw_info.model == SOFTRF_MODEL_PRIME_MK2) {
    bmp280.setWire(&Wire);
    bool found = false;
    if (bmp280.begin(BMP280_ADDRESS,     BMP280_CHIPID))  found = true;
    else if (bmp280.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID))  found = true;
    else if (bmp280.begin(BMP280_ADDRESS,     BME280_CHIPID))  found = true;
    else if (bmp280.begin(BMP280_ADDRESS_ALT, BME280_CHIPID))  found = true;
    if (found) {
        //ESP32_pin_unreserve(SOC_GPIO_PIN_TBEAM_SDA);
        //ESP32_pin_unreserve(SOC_GPIO_PIN_TBEAM_SCL);
        if (ESP32_pin_reserved(SOC_GPIO_PIN_TBEAM_SDA, false, "BMP"))  // also used by OLED
            ESP32_pin_reserved(SOC_GPIO_PIN_TBEAM_SDA, true, "BMP");
        if (ESP32_pin_reserved(SOC_GPIO_PIN_TBEAM_SCL, false, "BMP"))
            ESP32_pin_reserved(SOC_GPIO_PIN_TBEAM_SCL, true, "BMP");
        return true;
    }
    Serial.println(F("BMP280 not found on pins 2,13..."));
    Serial.println(F("probing BMP280 on pins 21,22..."));
    bmp280.setWire(&Wire1);
    if (bmp280.begin(BMP280_ADDRESS,     BMP280_CHIPID))  return true;
    if (bmp280.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID))  return true;
    if (bmp280.begin(BMP280_ADDRESS,     BME280_CHIPID))  return true;
    if (bmp280.begin(BMP280_ADDRESS_ALT, BME280_CHIPID))  return true;
  } else
#endif
  {
    bmp280.setWire(&Wire);
    if (bmp280.begin(BMP280_ADDRESS,     BMP280_CHIPID))  return true;
    if (bmp280.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID))  return true;
    if (bmp280.begin(BMP280_ADDRESS,     BME280_CHIPID))  return true;
    if (bmp280.begin(BMP280_ADDRESS_ALT, BME280_CHIPID))  return true;
  }
  return false;
}

static void bmp280_setup()
{
    Serial.print(F("Temperature = "));
    Serial.print(bmp280.readTemperature());
    Serial.println(F(" *C"));

    Baro_pressure_cache = (float) bmp280.readPressure();
    // first reading often way off, wait and do another one:
    delay(1000);
    Baro_pressure_cache = (float) bmp280.readPressure();
    Serial.print(F("Pressure = "));
    Serial.print(Baro_pressure_cache);
    Serial.println(F(" Pa"));

    Serial.print(F("Altitude using library = "));
    Serial.print(bmp280.readAltitude(1013.25)); // this should be adjusted to your local pressure
    Serial.println(F(" m"));
    Serial.print(F("Altitude using our code = "));
    Serial.print(altitude_from_pressure());

    Serial.println();
    delay(500);
}

static float bmp280_altitude()
{
    // return bmp280.readAltitude(sealevelPressure);
    Baro_pressure_cache = (float) bmp280.readPressure();
    return altitude_from_pressure();
}

static float bmp280_pressure()
{
    //return bmp280.readPressure();
    return Baro_pressure_cache;   // assume altitude has been read recently
}

static float bmp280_temperature()
{
    return bmp280.readTemperature();
}

barochip_ops_t bmp280_ops = {
  BARO_MODULE_BMP280,
  "BMP280",
  bmp280_probe,
  bmp280_setup,
  bmp280_altitude,
  bmp280_pressure,
  bmp280_temperature
};
#endif /* EXCLUDE_BMP280 */

#if !defined(EXCLUDE_MPL3115A2)
static bool mpl3115a2_probe()
{
//  return mpl3115a2.begin();
#if defined(ESP32)
  if (settings->gnss_pins != EXT_GNSS_13_2) {
    if (mpl3115a2.begin(&Wire))
      return true;
  }
  if (mpl3115a2.begin(&Wire1))
    return true;
#else
  if (mpl3115a2.begin(&Wire))
    return true;
#endif
  return false;
}

static void mpl3115a2_setup()
{
  float pascals = mpl3115a2.getPressure();
  // Our weather page presents pressure in Inches (Hg)
  // Use http://www.onlineconversion.com/pressure.htm for other units
  Serial.print(pascals/3377); Serial.println(F(" Inches (Hg)"));

  float altm = mpl3115a2.getAltitude();
  Serial.print(altm); Serial.println(F(" meters"));

  float tempC = mpl3115a2.getTemperature();
  Serial.print(tempC); Serial.println(F("*C"));

  delay(250);
}

static float mpl3115a2_pressure()
{
  //return mpl3115a2.getPressure();
  return Baro_pressure_cache;   // assume altitude has been read recently
}

static float mpl3115a2_altitude()
{
  Baro_pressure_cache = mpl3115a2_pressure();
  //mpl3115a2.setSeaPressure((float)101325.0);
  //return mpl3115a2.getAltitude();
  return altitude_from_pressure();
}

static float mpl3115a2_temperature()
{
  return mpl3115a2.getTemperature();
}

barochip_ops_t mpl3115a2_ops = {
  BARO_MODULE_MPL3115A2,
  "MPL3115A2",
  mpl3115a2_probe,
  mpl3115a2_setup,
  mpl3115a2_altitude,
  mpl3115a2_pressure,
  mpl3115a2_temperature
};
#endif /* EXCLUDE_MPL3115A2 */

bool Baro_probe()
{

#if !defined(EXCLUDE_BMP180)
  baro_chip = &bmp180_ops;
  if (baro_chip->probe())  return true;
#endif /* EXCLUDE_BMP180 */

#if !defined(EXCLUDE_BMP280)
  baro_chip = &bmp280_ops;
  if (baro_chip->probe())  return true;
#endif /* EXCLUDE_BMP280 */

#if !defined(EXCLUDE_MPL3115A2)
  baro_chip = &mpl3115a2_ops;
  if (baro_chip->probe())  return true;
#endif /* EXCLUDE_MPL3115A2 */

  baro_chip = NULL;
  return false;
}

byte Baro_setup()
{
Serial.println("baro setting up");

  if ( SoC->Baro_setup() /* && Baro_probe() */ ) {    // Baro_probe() also called from inside ESP32_Baro_setup()

    if (baro_chip == NULL) {
        Serial.println(F("BUG WARNING! Baro_probe() was not called"));
        return BARO_MODULE_NONE;
    }

    Serial.print(baro_chip->name);
    Serial.println(F(" barometric pressure sensor is detected."));

    baro_chip->setup();

    Baro_altitude_cache    = baro_chip->altitude();    // also fills in Baro_pressure_cache
    Baro_pressure_cache    = baro_chip->pressure();
    Baro_temperature_cache = baro_chip->temperature();
    BaroPresTempTimeMarker = millis();

    ThisAircraft.pressure_altitude = prev_pressure_altitude = Baro_altitude_cache;
    BaroAltitudeTimeMarker = millis();

    for (int i=0; i<VS_AVERAGING_FACTOR; i++) {
      Baro_VS[i] = 0;
    }

    return baro_chip->type;

  } else {
    baro_chip = NULL;
    Serial.println(F("Barometric pressure sensor was NOT detected."));
    return BARO_MODULE_NONE;
  }
}

void Baro_loop()
{
  if (baro_chip == NULL) return;

  if (isTimeToBaroAltitude()) {

    /* Draft of pressure altitude and vertical speed calculation */
    Baro_altitude_cache = baro_chip->altitude();    // also fills in Baro_pressure_cache

    ThisAircraft.pressure_altitude = Baro_altitude_cache;
    ThisAircraft.baro_alt_diff = ThisAircraft.altitude - ThisAircraft.pressure_altitude;

#if !defined(EXCLUDE_LK8EX1)
    if ((settings->nmea_s | settings->nmea2_s) & NMEA_S_LK8) {
      snprintf_P(NMEABuffer, sizeof(NMEABuffer), PSTR("$LK8EX1,%d,%.2f,%d,%d,%d*"),
            (int)Baro_pressure_cache,                                      /* Pascals */
            Baro_altitude_cache,                                           /* meters */
            (int) (ThisAircraft.vs * (100 / (_GPS_FEET_PER_METER * 60))),  /* cm/s   */
            constrain((int) Baro_temperature(), -99, 98),                  /* deg. C */
            1000+(int)Battery_charge());
            // - LK8000 specs say send percent instead of volts as an integer, percent+1000
      int nmealen = NMEA_add_checksum();
      NMEA_Source = DEST_NONE;
      NMEA_Outs(NMEA_S_LK8, NMEABuffer, nmealen, false);
    }
#endif /* EXCLUDE_LK8EX1 */

    Baro_VS[avg_ndx] = (Baro_altitude_cache - prev_pressure_altitude) /
                       (millis() - BaroAltitudeTimeMarker) * 1000;  /* in m/s */

    ThisAircraft.vs = 0;
    for (int i=0; i<VS_AVERAGING_FACTOR; i++) {
      ThisAircraft.vs += Baro_VS[i];
    }
    ThisAircraft.vs /= VS_AVERAGING_FACTOR;

    if (ThisAircraft.vs > -0.1 && ThisAircraft.vs < 0.1) {
      ThisAircraft.vs = 0;
    }

    ThisAircraft.vs *= (_GPS_FEET_PER_METER * 60.0) ; /* feet per minute */

    prev_pressure_altitude = Baro_altitude_cache;
    BaroAltitudeTimeMarker = millis();
    avg_ndx = (avg_ndx + 1) % VS_AVERAGING_FACTOR;

#if 0
    Serial.print(F("P.Alt. = ")); Serial.print(ThisAircraft.pressure_altitude);
    Serial.print(F(" , VS avg. = ")); Serial.println(ThisAircraft.vs);
#endif
  }

  if (isTimeToBaroPresTemp()) {
    // Baro_pressure_cache was filled in by baro_chip->altitude() above
    Baro_temperature_cache = baro_chip->temperature();
    BaroPresTempTimeMarker = millis();
  }
}

float Baro_altitude()
{
  return Baro_altitude_cache;
}

float Baro_pressure()
{
  return Baro_pressure_cache;
}

float Baro_temperature()
{
  return Baro_temperature_cache;
}

#endif /* EXCLUDE_BMP180 && EXCLUDE_BMP280 && EXCLUDE_MPL3115A2 */
