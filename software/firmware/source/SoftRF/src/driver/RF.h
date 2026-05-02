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

#include <TimeLib.h>

#include "../system/SoC.h"

#include <lib_crc.h>
#include <protocol.h>
#include <freqplan.h>

#include "GNSS.h"
#include "../protocol/radio/Legacy.h"
#include "../protocol/radio/OGNTP.h"
#include "../protocol/radio/ADSL.h"
#include "../protocol/radio/PAW.h"
#include "../protocol/radio/FANET.h"
#include "../protocol/radio/ES1090.h"
#include "../protocol/radio/UAT978.h"

#define maxof2(a,b)       (a > b ? a : b)
#define maxof3(a,b,c)     maxof2(maxof2(a,b),c)
#define maxof4(a,b,c,d)   maxof2(maxof2(a,b),maxof2(c,d))
#define maxof5(a,b,c,d,e) maxof2(maxof2(a,b),maxof3(c,d,e))

/* Max. paket's payload size for all supported RF protocols */
//#define MAX_PKT_SIZE  32 /* 48 = UAT LONG_FRAME_DATA_BYTES */

#if 0
#if !defined(EXCLUDE_UAT978)
#define MAX_PKT_SIZE  maxof5(LEGACY_PAYLOAD_SIZE+LEGACY_CRC_SIZE+3, OGNTP_PAYLOAD_SIZE+OGNTP_CRC_SIZE, \
                             P3I_PAYLOAD_SIZE+P3I_PAYLOAD_OFFSET+P3I_CRC_SIZE, \
                             FANET_PAYLOAD_SIZE, UAT978_PAYLOAD_SIZE)
#else
#define MAX_PKT_SIZE  maxof4(LEGACY_PAYLOAD_SIZE+LEGACY_CRC_SIZE+3, OGNTP_PAYLOAD_SIZE+OGNTP_CRC_SIZE, \
                             P3I_PAYLOAD_SIZE+P3I_PAYLOAD_OFFSET+P3I_CRC_SIZE, FANET_PAYLOAD_SIZE)
#endif
#else
#if !defined(EXCLUDE_UAT978)
#define MAX_PKT_SIZE 48
#else
#define MAX_PKT_SIZE 32
#endif
#endif

//#define RADIOLIB_MAX_DATA_LENGTH    128
#define RADIOLIB_MAX_DATA_LENGTH    (2 * MAX_PKT_SIZE)
extern uint8_t RL_txPacket[RADIOLIB_MAX_DATA_LENGTH];
extern uint8_t RL_rxPacket[RADIOLIB_MAX_DATA_LENGTH];

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
  RF_IC_SX1262,
  RF_IC_LR1110,
  RF_IC_LR2021
};

enum
{
  RF_TX_POWER_OFF,
  RF_TX_POWER_LOW,
  RF_TX_POWER_FULL
};

#define RF_CHANNEL_NONE 0xFF

static const int NUM_DIO = 3;

// this struct is filled in in SoC setup

#define u1_t uint8_t
#define u2_t uint16_t

struct lmic_pinmap {
    u1_t nss;

    // Written HIGH in TX mode, LOW otherwise.
    // Typically used with a single RXTX switch pin.
    u1_t txe;
    // Written HIGH in RX mode, LOW otherwise.
    // Typicaly used with separate RX/TX pins, to allow switching off
    // the antenna switch completely.
    u1_t rxe;

    u1_t rst;
    u1_t dio[NUM_DIO];
    u1_t busy;
    u1_t tcxo;
};

// Use this for any unused pins.
const u1_t LMIC_UNUSED_PIN = 0xff;

extern lmic_pinmap lmic_pins;

typedef struct rfchip_ops_struct {
  byte type;
  const char name[8];
  //bool (*probe)();         // called directly instead, during RF_setup
  //void (*setup)(const rf_proto_desc_t*, bool);
    // - can keep out of here if only called from receive() and transmit()
  void (*setfreq)(uint32_t);
  uint8_t (*receive)(uint8_t *packet);
  int16_t (*transmit)(uint8_t *packet, uint8_t length);
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

// called directly during RF_setup
//bool lr1276_probe();
//bool lr1262_probe();
//bool lr1110_probe();
//bool lr1121_probe();
bool rf_chips_probe();

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
const char *protocol_lbl(uint8_t main, uint8_t alt);

extern FreqPlan RF_FreqPlan;
extern byte TxBuffer[MAX_PKT_SIZE], RxBuffer[MAX_PKT_SIZE];
extern uint32_t TxTimeMarker;
extern uint32_t TxEndMarker;
//extern time_t RF_time;
extern uint32_t RF_time;
extern uint8_t RF_current_slot;
extern uint8_t current_TX_protocol;
extern uint8_t dual_protocol;
extern const rf_proto_desc_t  *curr_rx_protocol_ptr;
extern const rf_proto_desc_t  *curr_tx_protocol_ptr;

//extern float Vtcxo;

extern const rfchip_ops_t *rf_chip;
extern bool use_hardware_manchester;
extern uint8_t tx_power;
extern bool receive_active;
extern volatile bool receive_complete;
extern bool RF_SX12XX_RST_is_connected;
extern size_t (*protocol_encode)(void *, container_t *);
extern bool (*protocol_decode)(void *, container_t *, ufo_t *);

extern const char *Protocol_ID[];
extern const char *dual_protocol_lbl[];
extern uint32_t RF_last_crc;
extern int8_t RF_last_rssi;
extern int8_t which_rx_try;
extern uint8_t RF_last_protocol;

extern uint32_t rx_packets_counter;
extern uint32_t tx_packets_counter;
extern uint32_t adsb_packets_counter;
extern uint32_t receive_cb_count;

/* #define TIMETEST */
#ifdef TIMETEST
void increment_fake_time(void);
void reset_fake_time(void);
#endif

#endif /* RFHELPER_H */
