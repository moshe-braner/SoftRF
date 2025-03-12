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

#include "TinyGPS++.h"

#include <string.h>
#include <ctype.h>
#include <stdlib.h>

#define _GPRMCterm   "GPRMC"
#define _GPGGAterm   "GPGGA"
#define _GNRMCterm   "GNRMC"
#define _GNGGAterm   "GNGGA"

TinyGPSPlus::TinyGPSPlus()
  :  parity(0)
  ,  isChecksumTerm(false)
  ,  curSentenceType(GPS_SENTENCE_OTHER)
  ,  curTermNumber(0)
  ,  curTermOffset(0)
  ,  sentenceHasFix(false)
  ,  customElts(0)
  ,  customCandidates(0)
  ,  encodedCharCount(0)
  ,  sentencesWithFixCount(0)
  ,  failedChecksumCount(0)
  ,  passedChecksumCount(0)
{
  term[0] = '\0';
}

//
// public methods
//

bool TinyGPSPlus::encode(char c)
{
  ++encodedCharCount;

  switch(c)
  {
  case ',': // term terminators
    parity ^= (uint8_t)c;
  case '\r':
  case '\n':
  case '*':
    {
      bool isValidSentence = false;
      if (curTermOffset < sizeof(term))
      {
        term[curTermOffset] = 0;
        isValidSentence = endOfTermHandler();
      }
      ++curTermNumber;
      curTermOffset = 0;
      isChecksumTerm = c == '*';
      return isValidSentence;
    }
    break;

  case '$': // sentence begin
    curTermNumber = curTermOffset = 0;
    parity = 0;
    curSentenceType = GPS_SENTENCE_OTHER;
    isChecksumTerm = false;
    sentenceHasFix = false;
    return false;

  default: // ordinary characters
    if (curTermOffset < sizeof(term) - 1)
      term[curTermOffset++] = c;
    if (!isChecksumTerm)
      parity ^= c;
    return false;
  }

  return false;
}

//
// internal utilities
//
int TinyGPSPlus::fromHex(char a)
{
  if (a >= 'A' && a <= 'F')
    return a - 'A' + 10;
  else if (a >= 'a' && a <= 'f')
    return a - 'a' + 10;
  else
    return a - '0';
}

static const float powers_of_10[5] = {
    0.1,
    0.01,
    0.001,
    0.0001,
    0.00001  
};

// static
// Parse a (potentially negative) number with up to 5 decimal digits -xxxx.yyyyy
float TinyGPSPlus::parseDecimal(const char *term)
{
  bool negative = *term == '-';
  if (negative) ++term;
  float ret = (float) atol(term);
  while (*term && *term != '.')  ++term;
  if (*term == '.') {
    ++term;
    if (isdigit(*term)) {
      int moredigits = 0;
      uint32_t fraction = (*term - '0');
      while (moredigits < 4 && isdigit(*(++term))) {
        fraction = 10*fraction + (*term - '0');
        ++moredigits;
      }
      ret += powers_of_10[moredigits] * (float) fraction;
    }
  }
  return negative ? -ret : ret;
}

//static
//Parse 2 digits in place within a longer string
int TinyGPSPlus::parse2digits(const char *p)
{
  if (!isdigit(p[0]) || !isdigit(p[1]))
      return -1;
  uint8_t i;
  i = 10 * (*p - '0');
  ++p;
  i += (*p - '0');
  return i;
}

// static
// Parse a date in the format YYMMDD
void TinyGPSPlus::parseDate(const char *term, ymd_t *date)
{
  int i;
  i = parse2digits(term);
  if (i < 0) return;
  date->Day  = i;
  i = parse2digits(term+2);
  if (i < 0) return;
  date->Month = i;
  i = parse2digits(term+4);
  if (i < 0) return;
  date->Year   = i;
}

// static
// Parse a time in the format HHMMSS.CC
void TinyGPSPlus::parseTime(const char *term, hmsc_t *time)
{
  int i;
  i = parse2digits(term);
  if (i < 0) return;
  time->Hour   = i;
  term += 2;
  i = parse2digits(term);
  if (i < 0) return;
  time->Minute = i;
  term += 2;
  i = parse2digits(term);
  if (i < 0) return;
  time->Second = i;
  term += 2;
  time->CentiSec = 0;
  if (*term == '.') {
    ++term;
    if (isdigit(*term)) {
      time->CentiSec = 10 * (*term - '0');
      ++term;
      if (isdigit(*term))
      time->CentiSec += (*term - '0');
    }
  }
}

// static
// Parse degrees in that funny NMEA format DDMM.MMMM - always positive
float TinyGPSPlus::parseDegrees(const char *term)
{
  //while (!isdigit(*term))
  //  ++term;
  int c0 = term[0];
  if (!isdigit(c0))  return 0.0;
  int c1 = term[1];
  if (!isdigit(c1))  return 0.0;
  int c2 = term[2];
  if (!isdigit(c2))  return 0.0;
  if (!isdigit(term[3]))  return 0.0;
  int deg;
  if (term[4] == '.') {                             // latitude DDMM.MMMM
    deg = 10*(c0-'0') + (c1-'0');
    term += 2;
  } else if (isdigit(term[4]) && term[5] == '.') {  // longitude DDDMM.MMMM
    if (c0 == '0') {
      deg = 10*(c1-'0') + (c2-'0');
      term += 3;
    } else if (c0 == '1') {
      deg = 100 + 10*(c1-'0') + (c2-'0');
      term += 3;
    } else {
      return 0.0;
    }
  } else {
    return 0.0;
  }
  return (float) deg + (1.0f / 60.0f) * parseDecimal(term);
}

#define COMBINE(sentence_type, term_number) (((unsigned)(sentence_type) << 5) | term_number)

// Processes a just-completed term
// Returns true if new sentence has just passed checksum test and is validated
bool TinyGPSPlus::endOfTermHandler()
{
  // If it's the checksum term, and the checksum checks out, commit
  if (isChecksumTerm)
  {
    byte checksum = 16 * fromHex(term[0]) + fromHex(term[1]);
    if (checksum == parity)
    {
      passedChecksumCount++;
      if (sentenceHasFix)
        ++sentencesWithFixCount;

      switch(curSentenceType)
      {
      case GPS_SENTENCE_GPRMC:
        date.commit();
        time.commit();
        if (sentenceHasFix)
        {
           location.commit();
           speed.commit();
           course.commit();
        }
        break;
      case GPS_SENTENCE_GPGGA:
        time.commit();
        if (sentenceHasFix)
        {
          location.commit();
          altitude.commit();
          separation.commit();
        }
        satellites.commit();
        hdop.commit();
        break;
      }

      // Commit all custom listeners of this sentence type
      for (TinyGPSCustom *p = customCandidates; p != NULL && strcmp(p->sentenceName, customCandidates->sentenceName) == 0; p = p->next)
         p->commit();
      return true;
    }

    else
    {
      ++failedChecksumCount;
    }

    return false;
  }

  // the first term determines the sentence type
  if (curTermNumber == 0)
  {
    if (!strcmp(term, _GPRMCterm) || !strcmp(term, _GNRMCterm))
      curSentenceType = GPS_SENTENCE_GPRMC;
    else if (!strcmp(term, _GPGGAterm) || !strcmp(term, _GNGGAterm))
      curSentenceType = GPS_SENTENCE_GPGGA;
    else
      curSentenceType = GPS_SENTENCE_OTHER;

    // Any custom candidates of this sentence type?
    for (customCandidates = customElts; customCandidates != NULL && strcmp(customCandidates->sentenceName, term) < 0; customCandidates = customCandidates->next);
    if (customCandidates != NULL && strcmp(customCandidates->sentenceName, term) > 0)
       customCandidates = NULL;

    return false;
  }

  if (curSentenceType != GPS_SENTENCE_OTHER && term[0])
    switch(COMBINE(curSentenceType, curTermNumber))
  {
    case COMBINE(GPS_SENTENCE_GPRMC, 1): // Time in both sentences
    case COMBINE(GPS_SENTENCE_GPGGA, 1):
      time.setTime(term);
      break;
    case COMBINE(GPS_SENTENCE_GPRMC, 2): // GPRMC validity
      sentenceHasFix = term[0] == 'A';
      break;
    case COMBINE(GPS_SENTENCE_GPRMC, 3): // Latitude
    case COMBINE(GPS_SENTENCE_GPGGA, 2):
      location.setLatitude(term);
      break;
    case COMBINE(GPS_SENTENCE_GPRMC, 4): // N/S
    case COMBINE(GPS_SENTENCE_GPGGA, 3):
      if (term[0] == 'S')
        location.NewLatitude = -location.NewLatitude;
      break;
    case COMBINE(GPS_SENTENCE_GPRMC, 5): // Longitude
    case COMBINE(GPS_SENTENCE_GPGGA, 4):
      location.setLongitude(term);
      break;
    case COMBINE(GPS_SENTENCE_GPRMC, 6): // E/W
    case COMBINE(GPS_SENTENCE_GPGGA, 5):
      if (term[0] == 'W')
        location.NewLongitude = -location.NewLongitude;
      break;
    case COMBINE(GPS_SENTENCE_GPRMC, 7): // Speed (GPRMC)
      speed.set(term);
      break;
    case COMBINE(GPS_SENTENCE_GPRMC, 8): // Course (GPRMC)
      course.set(term);
      break;
    case COMBINE(GPS_SENTENCE_GPRMC, 9): // Date (GPRMC)
      date.setDate(term);
      break;
    case COMBINE(GPS_SENTENCE_GPGGA, 6): // Fix data (GPGGA)
      sentenceHasFix = term[0] > '0';
      location.newFixQuality = sentenceHasFix ? (FixQuality)(term[0] - '0') : Invalid;
      break;
    case COMBINE(GPS_SENTENCE_GPGGA, 7): // Satellites used (GPGGA)
      satellites.set(term);
      break;
    case COMBINE(GPS_SENTENCE_GPGGA, 8): // HDOP
      hdop.set(term);
      break;
    case COMBINE(GPS_SENTENCE_GPGGA, 9): // Altitude (GPGGA)
      altitude.set(term);
      break;
    case COMBINE(GPS_SENTENCE_GPGGA, 11): // Geoid Separation (GPGGA)
      separation.set(term);
      break;
    case COMBINE(GPS_SENTENCE_GPRMC, 12):
      location.newFixMode = (FixMode)term[0];
      break;
  }

  // Set custom values as needed
  for (TinyGPSCustom *p = customCandidates; p != NULL && strcmp(p->sentenceName, customCandidates->sentenceName) == 0 && p->termNumber <= curTermNumber; p = p->next)
    if (p->termNumber == curTermNumber)
         p->set(term);

  return false;
}

#if 0   // SoftRF does not use these

/* static */
float TinyGPSPlus::distanceBetween(float lat1, float long1, float lat2, float long2)
{
  // returns distance in meters between two positions, both specified
  // as signed decimal-degrees latitude and longitude. Uses great-circle
  // distance computation for hypothetical sphere of radius 6372795 meters.
  // Because Earth is no exact sphere, rounding errors may be up to 0.5%.
  // Courtesy of Maarten Lamers
  float delta = radians(long1-long2);
  float sdlong = sin(delta);
  float cdlong = cos(delta);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  float slat1 = sin(lat1);
  float clat1 = cos(lat1);
  float slat2 = sin(lat2);
  float clat2 = cos(lat2);
  delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
  delta = sq(delta);
  delta += sq(clat2 * sdlong);
  delta = sqrt(delta);
  float denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
  delta = atan2(delta, denom);
  return delta * 6372795.0f;
}

float TinyGPSPlus::courseTo(float lat1, float long1, float lat2, float long2)
{
  // returns course in degrees (North=0, West=270) from position 1 to position 2,
  // both specified as signed decimal-degrees latitude and longitude.
  // Because Earth is no exact sphere, calculated course may be off by a tiny fraction.
  // Courtesy of Maarten Lamers
  float dlon = radians(long2-long1);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  float a1 = sin(dlon) * cos(lat2);
  float a2 = sin(lat1) * cos(lat2) * cos(dlon);
  a2 = cos(lat1) * sin(lat2) - a2;
  a2 = atan2(a1, a2);
  if (a2 < 0.0)
  {
    a2 += TWO_PI;
  }
  return degrees(a2);
}

const char *TinyGPSPlus::cardinal(float course)
{
  static const char* directions[] = {"N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"};
  int direction = (int)((course + 11.25f) / 22.5f);
  return directions[direction % 16];
}

#endif

void TinyGPSLocation::commit()
{
   Latitude = NewLatitude;
   Longitude = NewLongitude;
   fixQuality = newFixQuality;
   fixMode = newFixMode;
   lastCommitTime = millis();
   valid = updated = true;
}

void TinyGPSLocation::setLatitude(const char *term)
{
   NewLatitude = TinyGPSPlus::parseDegrees(term);
}

void TinyGPSLocation::setLongitude(const char *term)
{
   NewLongitude = TinyGPSPlus::parseDegrees(term);
}

float TinyGPSLocation::lat()
{
   updated = false;
   return Latitude;
}

float TinyGPSLocation::lng()
{
   updated = false;
   return Longitude;
}

void TinyGPSDate::commit()
{
   //Date.Year  = newDate.Year;
   //Date.Month = newDate.Month;
   //Date.Day   = newDate.Day;
   Date = newDate;
   lastCommitTime = millis();
   valid = updated = true;
}

void TinyGPSTime::commit()
{
   //Time.Hour     = newTime.Hour;
   //Time.Minute   = newTime.Minute;
   //Time.Second   = newTime.Second;
   //Time.CentiSec = newTime.CentiSec;
   Time = newTime;
   lastCommitTime = millis();
   valid = updated = true;
}

void TinyGPSTime::setTime(const char *term)
{
   //newTime = (uint32_t)TinyGPSPlus::parseDecimal(term);
   TinyGPSPlus::parseTime(term, &newTime);
}

void TinyGPSDate::setDate(const char *term)
{
   //newDate = atol(term);
   TinyGPSPlus::parseDate(term, &newDate);
}

uint16_t TinyGPSDate::year()
{
   updated = false;
   return Date.Year + 2000;
}

uint8_t TinyGPSDate::month()
{
   updated = false;
   return Date.Month;
}

uint8_t TinyGPSDate::day()
{
   updated = false;
   return Date.Day;
}

uint8_t TinyGPSTime::hour()
{
   updated = false;
   return Time.Hour;
}

uint8_t TinyGPSTime::minute()
{
   updated = false;
   return Time.Minute;
}

uint8_t TinyGPSTime::second()
{
   updated = false;
   return Time.Second;
}

uint8_t TinyGPSTime::centisecond()
{
   updated = false;
   return Time.CentiSec;
}

void TinyGPSDecimal::commit()
{
   val = newval;
   lastCommitTime = millis();
   valid = updated = true;
}

void TinyGPSDecimal::set(const char *term)
{
   newval = TinyGPSPlus::parseDecimal(term);
}

void TinyGPSInteger::commit()
{
   val = newval;
   lastCommitTime = millis();
   valid = updated = true;
}

void TinyGPSInteger::set(const char *term)
{
   newval = atol(term);
}

TinyGPSCustom::TinyGPSCustom(TinyGPSPlus &gps, const char *_sentenceName, int _termNumber)
{
   begin(gps, _sentenceName, _termNumber);
}

void TinyGPSCustom::begin(TinyGPSPlus &gps, const char *_sentenceName, int _termNumber)
{
   lastCommitTime = 0;
   updated = valid = false;
   sentenceName = _sentenceName;
   termNumber = _termNumber;
   memset(stagingBuffer, '\0', sizeof(stagingBuffer));
   memset(buffer, '\0', sizeof(buffer));

   // Insert this item into the GPS tree
   gps.insertCustom(this, _sentenceName, _termNumber);
}

void TinyGPSCustom::commit()
{
   strcpy(this->buffer, this->stagingBuffer);
   lastCommitTime = millis();
   valid = updated = true;
}

void TinyGPSCustom::set(const char *term)
{
   strncpy(this->stagingBuffer, term, sizeof(this->stagingBuffer));
}

void TinyGPSPlus::insertCustom(TinyGPSCustom *pElt, const char *sentenceName, int termNumber)
{
   TinyGPSCustom **ppelt;

   for (ppelt = &this->customElts; *ppelt != NULL; ppelt = &(*ppelt)->next)
   {
      int cmp = strcmp(sentenceName, (*ppelt)->sentenceName);
      if (cmp < 0 || (cmp == 0 && termNumber < (*ppelt)->termNumber))
         break;
   }

   pElt->next = *ppelt;
   *ppelt = pElt;
}
