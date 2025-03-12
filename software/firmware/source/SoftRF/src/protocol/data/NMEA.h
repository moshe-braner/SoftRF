/*
 * NMEAHelper.h
 * Copyright (C) 2017-2022 Linar Yusupov
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

// this file based on v1.2.

#ifndef NMEAHELPER_H
#define NMEAHELPER_H

#ifdef __cplusplus
#include "../../system/SoC.h"
#endif /* __cplusplus */

#define GNS5892_BAUDRATE        921600
#define GNS5892_INPUT_BUF_SIZE    1000

#define NMEA_BUFFER_SIZE    128
#define NMEA_CALLSIGN_SIZE  (3 /* prefix */ + 1 /* _ */ + 6 /* ICAO */ + 1 /* EOL */)

#define PSRFX_VERSION       1

//#define PSRFC_VERSION       1
//#define MAX_PSRFC_LEN       96

//#define PSRFD_VERSION       1
//#define MAX_PSRFD_LEN       96

//#define PSRFF_VERSION       1
//#define MAX_PSRFF_LEN       96

//#define PSRFS_VERSION       1
//#define MAX_PSRFS_LEN       96

#define PSKVC_VERSION       2
//#define MAX_PSKVC_LEN       96

void NMEA_setup(void);
void NMEA_loop(void);
void flushNMEAlog();
void closeNMEAlog();
void NMEA_fini();
void NMEA_Export(void);
void NMEA_Position(void);
void NMEA_Out(uint8_t, const char *, size_t, bool);
void NMEA_Outs(uint16_t, const char *, unsigned int, bool);
void NMEAOutC(int nmeatype);   // with checksum
void NMEAOutD(void);           // without checksum
void NMEA_GGA(void);

extern char NMEABuffer[NMEA_BUFFER_SIZE];
unsigned int NMEA_add_checksum(char *buf=NMEABuffer);

void sendPFLAJ();

int WiFi_transmit_TCP(const char *buf, size_t size);

char *bytes2Hex(byte *, size_t);

extern uint8_t NMEA_Source;
extern char GPGGA_Copy[NMEA_BUFFER_SIZE];
extern bool has_serial2;
extern bool rx1090found;
extern bool NMEA_bridge_sent;

#if defined(USE_NMEA_CFG)
void NMEA_Process_SRF_SKV_Sentences(void);
#endif /* USE_NMEA_CFG */

#if defined(NMEA_TCP_SERVICE)

typedef struct NmeaTCP_struct {
  WiFiClient client;
  time_t connect_ts;  /* connect time stamp */
  bool ack;           /* acknowledge */
} NmeaTCP_t;

#define MAX_NMEATCP_CLIENTS    2
#define NMEATCP_ACK_TIMEOUT    2 /* seconds */

#endif

#if !defined(PFLAA_EXT1_FMT)
#define PFLAA_EXT1_FMT  ""
#endif /* PFLAA_EXT1_FMT */

#if !defined(PFLAA_EXT1_ARGS)
#define PFLAA_EXT1_ARGS
#endif /* PFLAA_EXT1_ARGS */

#if !defined(PFLAU_EXT1_FMT)
#define PFLAU_EXT1_FMT  ""
#endif /* PFLAU_EXT1_FMT */

#if !defined(PFLAU_EXT1_ARGS)
#define PFLAU_EXT1_ARGS
#endif /* PFLAU_EXT1_ARGS */

#endif /* NMEAHELPER_H */

