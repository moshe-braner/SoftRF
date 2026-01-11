/*
 * RFHelper.h
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

#ifndef RFHELPER_H
#define RFHELPER_H

#include <nRF905.h>
#include <TimeLib.h>

#include "../system/SoC.h"

#if defined(USE_BASICMAC)
#include <basicmac.h>
#else
#include <lmic.h>
#endif
#include <hal/hal.h>
#include <lib_crc.h>
#include <protocol.h>
#include <freqplan.h>

#include "GNSS.h"
#include "../protocol/radio/Legacy.h"
#include "../protocol/radio/OGNTP.h"
#include "../protocol/radio/P3I.h"
#include "../protocol/radio/FANET.h"
#include "../protocol/radio/UAT978.h"
#include "../protocol/radio/ADSL.h"

#define maxof2(a,b)       (a > b ? a : b)
#define maxof3(a,b,c)     maxof2(maxof2(a,b),c)
#define maxof4(a,b,c,d)   maxof2(maxof2(a,b),maxof2(c,d))
#define maxof5(a,b,c,d,e) maxof2(maxof2(a,b),maxof3(c,d,e))

/* Max. paket's payload size for all supported RF protocols */
//#define MAX_PKT_SIZE  32 /* 48 = UAT LONG_FRAME_DATA_BYTES */

#if !defined(EXCLUDE_UAT978)
#define MAX_PKT_SIZE  maxof5(LEGACY_PAYLOAD_SIZE+LEGACY_CRC_SIZE+3, OGNTP_PAYLOAD_SIZE, \
                             P3I_PAYLOAD_SIZE, FANET_PAYLOAD_SIZE, \
                             UAT978_PAYLOAD_SIZE)
#else
#define MAX_PKT_SIZE  maxof4(LEGACY_PAYLOAD_SIZE+LEGACY_CRC_SIZE+3, OGNTP_PAYLOAD_SIZE, \
                             P3I_PAYLOAD_SIZE, FANET_PAYLOAD_SIZE)
#endif

#define RXADDR {0x31, 0xfa , 0xb6} // Address of this device (4 bytes)
#define TXADDR {0x31, 0xfa , 0xb6} // Address of device to send to (4 bytes)

enum
{
  RF_IC_NONE,
  RF_IC_NRF905,
  RF_IC_SX1276,
  RF_IC_UATM,
  RF_IC_CC13XX,
  RF_DRV_OGN,
  RF_IC_SX1262
};

enum
{
  RF_TX_POWER_OFF,
  RF_TX_POWER_LOW,
  RF_TX_POWER_FULL
};

enum
{
  RF_SINGLE_PROTOCOL,
  RF_FLR_ADSL
};

typedef struct rfchip_ops_struct {
  byte type;
  const char name[8];
  bool (*probe)();
  void (*setup)();
  void (*channel)(uint8_t);
  bool (*receive)();
  void (*transmit)();
  void (*shutdown)();
} rfchip_ops_t;

typedef struct Slot_descr_struct {
  uint16_t begin;
  uint16_t duration;
  unsigned long tmarker;
} Slot_descr_t;

typedef struct Slots_descr_struct {
  uint16_t      interval_min;
  uint16_t      interval_max;
  uint16_t      interval_mid;
  uint16_t      adj;
  uint16_t      air_time;
  Slot_descr_t  s0;
  Slot_descr_t  s1;
  uint8_t       current;
} Slots_descr_t;

String Bin2Hex(byte *, size_t);
uint8_t parity(uint32_t);

byte    RF_setup(void);
void    RF_SetChannel(void);
void    RF_loop(void);
bool    RF_Transmit_Happened();
bool    RF_Transmit_Ready(bool wait);
size_t  RF_Encode(container_t *cip, bool wait);
bool    RF_Transmit(size_t size, bool wait);
bool    RF_Receive(void);
void    RF_Shutdown(void);
uint8_t RF_Payload_Size(uint8_t);
bool    in_family(uint8_t protocol);

extern byte TxBuffer[MAX_PKT_SIZE], RxBuffer[MAX_PKT_SIZE];
extern uint32_t TxTimeMarker;
extern uint32_t TxEndMarker;
extern time_t RF_time;
extern uint8_t RF_current_slot;
extern uint8_t current_RF_protocol;
extern uint8_t dual_protocol;

extern const rfchip_ops_t *rf_chip;
extern bool RF_SX12XX_RST_is_connected;
extern size_t (*protocol_encode)(void *, container_t *);
extern bool (*protocol_decode)(void *, container_t *, ufo_t *);

extern const char *Protocol_ID[];
extern uint32_t RF_last_crc;
extern int8_t RF_last_rssi;
extern int8_t which_rx_try;
extern uint8_t RF_last_protocol;

extern uint32_t rx_packets_counter, tx_packets_counter;

/* #define TIMETEST */
#ifdef TIMETEST
void increment_fake_time(void);
void reset_fake_time(void);
#endif

#endif /* RFHELPER_H */
