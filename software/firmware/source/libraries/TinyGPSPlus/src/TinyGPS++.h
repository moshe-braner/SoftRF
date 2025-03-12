/*
TinyGPS++ - a small GPS library for Arduino providing universal NMEA parsing
Based on work by and "distanceBetween" and "courseTo" courtesy of Maarten Lamers.
Suggestion to add satellites, courseTo(), and cardinal() by Matt Monson.
Location precision improvements suggested by Wayne Holder.
Copyright (C) 2008-2013 Mikal Hart
All rights reserved.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

// modified by Moshe Braner, 2025, for efficiency as used by SoftRF

#ifndef __TinyGPSPlus_h
#define __TinyGPSPlus_h

#if (defined(ARDUINO) && ARDUINO >= 100 && !defined(RASPBERRY_PI)) || \
    defined(HACKRF_ONE)
#include "Arduino.h"
#else
#if defined(RASPBERRY_PI)
#include <raspi/raspi.h>
#else
#include "WProgram.h"
#endif /* RASPBERRY_PI */
#endif
#include <limits.h>

#define _GPS_VERSION "1.0.2" // software version of this library
#define _GPS_MPH_PER_KNOT 1.15077945f
#define _GPS_MPS_PER_KNOT 0.51444444f
#define _GPS_KMPH_PER_KNOT 1.852f
#define _GPS_MILES_PER_METER 0.00062137112f
#define _GPS_KM_PER_METER 0.001f
#define _GPS_FEET_PER_METER 3.2808399f
#if !defined(ARDUINO_ARCH_AVR) && !defined(ENERGIA_ARCH_CC13XX)
#define _GPS_MAX_FIELD_SIZE 33
#else
#define _GPS_MAX_FIELD_SIZE 19
#endif

/*
struct RawDegrees
{
   uint16_t deg;
   uint32_t billionths;
   bool negative;
public:
   RawDegrees() : deg(0), billionths(0), negative(false)
   {}
};
*/

enum FixQuality { Invalid = 0, GPS = 1, DGPS = 2, PPS = 3, RTK = 4, FloatRTK = 5, Estimated = 6, Manual = 7, Simulated = 8 };
enum FixMode { N = 'N', A = 'A', D = 'D', E = 'E'};

struct TinyGPSLocation
{
   friend class TinyGPSPlus;
public:
   bool isValid() const    { return valid; }
   bool isUpdated() const  { return updated; }
   uint32_t age() const    { return valid ? millis() - lastCommitTime : (uint32_t)ULONG_MAX; }
   //const RawDegrees &rawLat()     { updated = false; return rawLatData; }
   //const float &rawLat()     { updated = false; return Latitude; }
   //const RawDegrees &rawLng()     { updated = false; return rawLngData; }
   //const float &rawLng()     { updated = false; return Longitude; }
   float lat();
   float lng();
   FixQuality Quality() { updated = false; return fixQuality; }
   FixMode Mode() { updated = false; return fixMode; }

   TinyGPSLocation() : valid(false), updated(false), fixQuality(Invalid), newFixQuality(Invalid), fixMode(N), newFixMode(N)
   {}

private:
   bool valid, updated;
   //float rawLatData, rawLngData, rawNewLatData, rawNewLngData;
   float Latitude, Longitude, NewLatitude, NewLongitude;
   uint32_t lastCommitTime;
   void commit();
   void setLatitude(const char *term);
   void setLongitude(const char *term);
   FixQuality fixQuality, newFixQuality;
   FixMode fixMode, newFixMode;
};

typedef struct ymd_s {
   uint8_t Year;
   uint8_t Month;
   uint8_t Day;
} ymd_t;

struct TinyGPSDate
{
   friend class TinyGPSPlus;
public:
   bool isValid() const       { return valid; }
   bool isUpdated() const     { return updated; }
   uint32_t age() const       { return valid ? millis() - lastCommitTime : (uint32_t)ULONG_MAX; }

   //uint32_t value()           { updated = false; return date; }
   uint16_t year();
   uint8_t month();
   uint8_t day();

   TinyGPSDate() : valid(false), updated(false)
   { Date.Year=0; Date.Month=0; Date.Day=0; }

private:
   bool valid, updated;
   //uint32_t date, newDate;
   ymd_t Date, newDate;
   uint32_t lastCommitTime;
   void commit();
   void setDate(const char *term);
};

typedef struct hmsc_s {
   uint8_t Hour;
   uint8_t Minute;
   uint8_t Second;
   uint8_t CentiSec;   
} hmsc_t;

struct TinyGPSTime
{
   friend class TinyGPSPlus;
public:
   bool isValid() const       { return valid; }
   bool isUpdated() const     { return updated; }
   uint32_t age() const       { return valid ? millis() - lastCommitTime : (uint32_t)ULONG_MAX; }

   //uint32_t value()           { updated = false; return time; }
   uint8_t hour();
   uint8_t minute();
   uint8_t second();
   uint8_t centisecond();

   TinyGPSTime() : valid(false), updated(false)
   { Time.Hour=0; Time.Minute=0; Time.Second=0; Time.CentiSec=0; }

private:
   bool valid, updated;
   //uint32_t time, newTime;
   hmsc_t Time, newTime;
   uint32_t lastCommitTime;
   void commit();
   void setTime(const char *term);
};

struct TinyGPSDecimal
{
   friend class TinyGPSPlus;
public:
   bool isValid() const    { return valid; }
   bool isUpdated() const  { return updated; }
   uint32_t age() const    { return valid ? millis() - lastCommitTime : (uint32_t)ULONG_MAX; }
   float value()           { updated = false; return val; }

   TinyGPSDecimal() : valid(false), updated(false), val(0)
   {}

private:
   bool valid, updated;
   uint32_t lastCommitTime;
   float val, newval;
   void commit();
   void set(const char *term);
};

struct TinyGPSInteger
{
   friend class TinyGPSPlus;
public:
   bool isValid() const    { return valid; }
   bool isUpdated() const  { return updated; }
   uint32_t age() const    { return valid ? millis() - lastCommitTime : (uint32_t)ULONG_MAX; }
   uint32_t value()        { updated = false; return val; }

   TinyGPSInteger() : valid(false), updated(false), val(0)
   {}

private:
   bool valid, updated;
   uint32_t lastCommitTime;
   uint32_t val, newval;
   void commit();
   void set(const char *term);
};

struct TinyGPSSpeed : TinyGPSDecimal
{
   float knots()    { return value(); }
   float mph()      { return _GPS_MPH_PER_KNOT * value(); }
   float mps()      { return _GPS_MPS_PER_KNOT * value(); }
   float kmph()     { return _GPS_KMPH_PER_KNOT * value(); }
};

struct TinyGPSCourse : public TinyGPSDecimal
{
   float deg()      { return value(); }
};

struct TinyGPSAltitude : TinyGPSDecimal
{
   float meters()       { return value(); }
   float miles()        { return _GPS_MILES_PER_METER * value(); }
   float kilometers()   { return _GPS_KM_PER_METER * value(); }
   float feet()         { return _GPS_FEET_PER_METER * value(); }
};

struct TinyGPSGeoidSeparation : TinyGPSDecimal
{
   float meters()       { return value(); }
   float miles()        { return _GPS_MILES_PER_METER * value(); }
   float kilometers()   { return _GPS_KM_PER_METER * value(); }
   float feet()         { return _GPS_FEET_PER_METER * value(); }
};

struct TinyGPSHDOP : TinyGPSDecimal
{
   float hdop() { return value(); }
};

class TinyGPSPlus;
class TinyGPSCustom
{
public:
   TinyGPSCustom() {};
   TinyGPSCustom(TinyGPSPlus &gps, const char *sentenceName, int termNumber);
   void begin(TinyGPSPlus &gps, const char *_sentenceName, int _termNumber);

   bool isUpdated() const  { return updated; }
   bool isValid() const    { return valid; }
   uint32_t age() const    { return valid ? millis() - lastCommitTime : (uint32_t)ULONG_MAX; }
   const char *value()     { updated = false; return buffer; }

private:
   void commit();
   void set(const char *term);

   char stagingBuffer[_GPS_MAX_FIELD_SIZE + 1];
   char buffer[_GPS_MAX_FIELD_SIZE + 1];
   unsigned long lastCommitTime;
   bool valid, updated;
   const char *sentenceName;
   int termNumber;
   friend class TinyGPSPlus;
   TinyGPSCustom *next;
};

class TinyGPSPlus
{
public:
  TinyGPSPlus();
  bool encode(char c); // process one character received from GPS
  TinyGPSPlus &operator << (char c) {encode(c); return *this;}

  TinyGPSLocation location;
  TinyGPSDate date;
  TinyGPSTime time;
  TinyGPSSpeed speed;
  TinyGPSCourse course;
  TinyGPSAltitude altitude;
  TinyGPSInteger satellites;
  TinyGPSHDOP hdop;
  TinyGPSGeoidSeparation separation;

  static const char *libraryVersion() { return _GPS_VERSION; }
#if 0
  static float distanceBetween(float lat1, float long1, float lat2, float long2);
  static float courseTo(float lat1, float long1, float lat2, float long2);
  static const char *cardinal(float course);
#endif

  static float parseDecimal(const char *term);
  static int parse2digits(const char *p);
  static void parseDate(const char *term, ymd_t *date);
  static void parseTime(const char *term, hmsc_t *time);
  static float parseDegrees(const char *term);

  uint32_t charsProcessed()   const { return encodedCharCount; }
  uint32_t sentencesWithFix() const { return sentencesWithFixCount; }
  uint32_t failedChecksum()   const { return failedChecksumCount; }
  uint32_t passedChecksum()   const { return passedChecksumCount; }

private:
  enum {GPS_SENTENCE_GPGGA, GPS_SENTENCE_GPRMC, GPS_SENTENCE_OTHER};

  // parsing state variables
  uint8_t parity;
  bool isChecksumTerm;
  char term[_GPS_MAX_FIELD_SIZE];
  uint8_t curSentenceType;
  uint8_t curTermNumber;
  uint8_t curTermOffset;
  bool sentenceHasFix;

  // custom element support
  friend class TinyGPSCustom;
  TinyGPSCustom *customElts;
  TinyGPSCustom *customCandidates;
  void insertCustom(TinyGPSCustom *pElt, const char *sentenceName, int index);

  // statistics
  uint32_t encodedCharCount;
  uint32_t sentencesWithFixCount;
  uint32_t failedChecksumCount;
  uint32_t passedChecksumCount;

  // internal utilities
  int fromHex(char a);
  bool endOfTermHandler();
};

#endif // def(__TinyGPSPlus_h)
