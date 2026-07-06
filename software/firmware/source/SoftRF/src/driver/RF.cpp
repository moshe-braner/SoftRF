/*
 * RF.cpp
 * by Moshe Braner
 *
 * Code to handle RF in a radio-module-agnostic manner.
 * Module-specific code is in radio.cpp.
 *
 * Based in part on RF.cpp by Linar Yusupov
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

#if defined(ARDUINO)
#include <SPI.h>
#endif /* ARDUINO */

#include <RadioLib.h>

#include "RF.h"
#include "../system/Time.h"
#include "../system/SoC.h"
#include "../TrafficHelper.h"
#include "Settings.h"
#include "Battery.h"
#include "../ui/Web.h"
#include "../protocol/data/NMEA.h"
#if !defined(EXCLUDE_MAVLINK)
#include "../protocol/data/MAVLink.h"
#endif /* EXCLUDE_MAVLINK */
#include <fec.h>

#if LOGGER_IS_ENABLED
#include "../system/Log.h"
#endif /* LOGGER_IS_ENABLED */

#include <manchester.h>

byte RxBuffer[MAX_PKT_SIZE] __attribute__((aligned(sizeof(uint32_t))));

//time_t RF_time;
uint32_t RF_time;
uint8_t RF_current_slot = 0;
uint8_t RF_current_chan = 0;
uint32_t RF_OK_from   = 0;
uint32_t RF_OK_until  = 0;
uint32_t TxTimeMarker = 0;
uint32_t TxTimeMarker2 = 0;
uint32_t TxEndMarker  = 0;
byte TxBuffer[MAX_PKT_SIZE] __attribute__((aligned(sizeof(uint32_t))));

uint8_t RL_txPacket[RADIOLIB_MAX_DATA_LENGTH];
uint8_t RL_rxPacket[RADIOLIB_MAX_DATA_LENGTH];

uint32_t tx_packets_counter = 0;
uint32_t rx_packets_counter = 0;
uint32_t adsb_packets_counter = 0;

static uint32_t invalid_manchester_packets = 0;
static uint8_t  invalid_manchester_counter = 0;

//int8_t which_rx_try = 0;
int8_t RF_last_rssi = 0;
uint32_t RF_last_crc = 0;            // for detection of duplicate packets
uint8_t RF_last_protocol = 0;        // for flr_adsl simultaneous reception
uint8_t RF_last_rx_len = 0;          // for variable-length FANET packets
uint8_t current_RX_protocol;
uint8_t current_TX_protocol;
uint8_t dual_protocol = RF_SINGLE_PROTOCOL;
bool rx_flr_adsl = false;
//uint8_t next_tx_protocol = 0;

FreqPlan RF_FreqPlan;
static bool RF_ready = false;

const rfchip_ops_t *rf_chip = NULL;
bool RF_SX12XX_RST_is_connected = true;

const char *Protocol_ID[] = {
  [RF_PROTOCOL_NONE]      = "---",  // 0
  [RF_PROTOCOL_OGNTP]     = "OGN",
  [RF_PROTOCOL_PAW]       = "PAW",
  [RF_PROTOCOL_ADSB_1090] = "ADS",
  [RF_PROTOCOL_ADSB_UAT]  = "UAT",
  [RF_PROTOCOL_FANET]     = "FAN",
  [RF_PROTOCOL_LEGACY]    = "LEG",
  [RF_PROTOCOL_LATEST]    = "LAT",
  [RF_PROTOCOL_ADSL]      = "ADL",  // 8
  [RF_PROTOCOL_GDL90]     = "GDL"   // 9 - not an RF protocol
};

struct protocol_duo
{
    const uint8_t main;
    const uint8_t alt;
    const char *label;
};

protocol_duo protocol_duos[] = {
  {RF_PROTOCOL_LEGACY, RF_PROTOCOL_LATEST, "L+T"},
  {RF_PROTOCOL_LEGACY, RF_PROTOCOL_OGNTP,  "L+O"},
  {RF_PROTOCOL_LEGACY, RF_PROTOCOL_ADSL,   "L+A"},
  {RF_PROTOCOL_LATEST, RF_PROTOCOL_LEGACY, "T+L"},
  {RF_PROTOCOL_LATEST, RF_PROTOCOL_OGNTP,  "T+O"},
  {RF_PROTOCOL_LATEST, RF_PROTOCOL_ADSL,   "T+A"},
  {RF_PROTOCOL_LATEST, RF_PROTOCOL_FANET,  "T+F"},
  {RF_PROTOCOL_LATEST, RF_PROTOCOL_PAW,    "T+P"},
  {RF_PROTOCOL_OGNTP,  RF_PROTOCOL_LEGACY, "O+L"},
  {RF_PROTOCOL_OGNTP,  RF_PROTOCOL_LATEST, "O+T"},
  {RF_PROTOCOL_OGNTP,  RF_PROTOCOL_ADSL,   "O+A"},
  {RF_PROTOCOL_ADSL,   RF_PROTOCOL_OGNTP,  "A+O"},
  {RF_PROTOCOL_ADSL,   RF_PROTOCOL_LEGACY, "A+L"},
  {RF_PROTOCOL_ADSL,   RF_PROTOCOL_LATEST, "A+T"},
  {RF_PROTOCOL_FANET,  RF_PROTOCOL_LATEST, "F+T"},
  {RF_PROTOCOL_FANET,  RF_PROTOCOL_OGNTP,  "F+O"},
  {RF_PROTOCOL_FANET,  RF_PROTOCOL_PAW,    "F+P"},
  {RF_PROTOCOL_FANET,  RF_PROTOCOL_ADSL,   "F+A"},
  {RF_PROTOCOL_FANET,RF_PROTOCOL_ADSB_1090,"F+B"},
  {RF_PROTOCOL_PAW,    RF_PROTOCOL_LATEST, "P+T"},
  {RF_PROTOCOL_PAW,    RF_PROTOCOL_OGNTP,  "P+O"},
  {RF_PROTOCOL_PAW,    RF_PROTOCOL_FANET,  "P+F"},
  {RF_PROTOCOL_PAW,    RF_PROTOCOL_ADSL,   "P+A"},
  {RF_PROTOCOL_PAW, RF_PROTOCOL_ADSB_1090, "P+B"},
  {RF_PROTOCOL_NONE, RF_PROTOCOL_NONE, NULL}
};

const char *protocol_lbl(uint8_t main, uint8_t alt)
{
    if (alt == RF_PROTOCOL_NONE)
        return Protocol_ID[main];
    for (int i=0; ; i++) {
        if (protocol_duos[i].label == NULL)
            return "???";
        if (protocol_duos[i].main == main && protocol_duos[i].alt == alt)
            return protocol_duos[i].label;
    }
}

const char *dual_protocol_lbl[] = {
  [RF_SINGLE_PROTOCOL] = "PROTOCOL",
  [RF_FLR_ADSL]        = "FLR_ADSL",
  [RF_FLR_FANET]       = "FLR_FANT",
  [RF_FLR_PAW]         = "FLR_PAW",
  [RF_FANET_PAW]       = "FANT_PAW",
  [RF_PAW_FANET]       = "PAW_FANT",
  [RF_FANET_ADSB]      = "FANT_ADS",
  [RF_PAW_ADSB]        = "PAW_ADSB"
};

size_t (*protocol_encode)(void *, container_t *);
size_t (*mainprotocol_encode)(void *, container_t *);
size_t (*altprotocol_encode)(void *, container_t *);
bool   (*protocol_decode)(void *, container_t *, ufo_t *);
bool   (*mainprotocol_decode)(void *, container_t *, ufo_t *);
bool   (*altprotocol_decode)(void *, container_t *, ufo_t *);
const rf_proto_desc_t  *curr_rx_protocol_ptr;
const rf_proto_desc_t  *curr_tx_protocol_ptr;
const rf_proto_desc_t  *mainprotocol_ptr;
const rf_proto_desc_t  *altprotocol_ptr;

static Slots_descr_t Time_Slots, *ts;
static uint8_t       RF_timing = RF_TIMING_INTERVAL;

extern const gnss_chip_ops_t *gnss_chip;

#define RF_CHANNEL_NONE 0xFF

/* was only used for old P3I "NiceRF":
const uint8_t whitening_pattern[] PROGMEM = { 0x05, 0xb4, 0x05, 0xae, 0x14, 0xda,
  0xbf, 0x83, 0xc4, 0x04, 0xb2, 0x04, 0xd6, 0x4d, 0x87, 0xe2, 0x01, 0xa3, 0x26,
  0xac, 0xbb, 0x63, 0xf1, 0x01, 0xca, 0x07, 0xbd, 0xaf, 0x60, 0xc8, 0x12, 0xed,
  0x04, 0xbc, 0xf6, 0x12, 0x2c, 0x01, 0xd9, 0x04, 0xb1, 0xd5, 0x03, 0xab, 0x06,
  0xcf, 0x08, 0xe6, 0xf2, 0x07, 0xd0, 0x12, 0xc2, 0x09, 0x34, 0x20 };
*/

#if 0
String Bin2Hex(byte *buffer, size_t size)
{
  String str = "";
  for (int i=0; i < size; i++) {
    byte c = buffer[i];
    str += (c < 0x10 ? "0" : "") + String(c, HEX);
  }
  return str;
}
#endif

uint8_t parity(uint32_t x) {
    uint8_t parity=0;
    while (x > 0) {
      if (x & 0x1) {
          parity++;
      }
      x >>= 1;
    }
    // return (parity % 2);
    return (parity & 0x01);
}

int8_t tx_power = 0;

static void calc_txpower()
{
  tx_power = 2;   /* 2 dBm is minimum for RFM95W on PA_BOOST pin */

  if (RF_FreqPlan.Plan == RF_BAND_AUTO)
      return;

  if (RF_FreqPlan.Protocol == RF_PROTOCOL_PAW
    && RF_FreqPlan.Plan != RF_BAND_EU
    && RF_FreqPlan.Plan != RF_BAND_UK
    && RF_FreqPlan.Plan != RF_BAND_RU) {
      return;    // 869.525 MHz not legal in the region
  }

  if (settings->txpower == RF_TX_POWER_FULL) {

    /* Load regional max. EIRP at first */
    tx_power = RF_FreqPlan.MaxTxPower;

    if (settings->relay >= RELAY_ONLY) {
        if (tx_power > 8)  tx_power = 8;
    }

    if (rf_chip->type == RF_IC_LR1110) {
#if 1
      /*
       * Enforce Tx power limit until confirmation
       * that LR11xx is doing well
       * when antenna is not connected
       */
      if (tx_power > 17)  tx_power = 17;
#else
      if (tx_power > 22)  tx_power = 22;
#endif

      //uint32_t frequency = RF_FreqPlan.getChanFrequency(0);
      //bool high = (frequency > 1000000000) ; /* above 1GHz */
      bool high = (RF_FreqPlan.BaseFreq > 1000000000);  /* above 1GHz */
      if (high && tx_power > 13) tx_power = 13;

    } else if (rf_chip->type == RF_IC_SX1262) {

      /* SX1262 is unable to give more than 22 dBm */
      //if (LMIC.txpow > 22)
      //  LMIC.txpow = 22;
      // The sx1262 has internal protection against antenna mismatch.
      // And yet the T-Echo instructions warn against using without an antenna.
      // But keep is a bit lower ayway
      if (tx_power > 19)  tx_power = 19;

    } else {

      /* SX1276 is unable to give more than 20 dBm */
      //if (LMIC.txpow > 20)
      //  LMIC.txpow = 20;
    // Most T-Beams have an sx1276, it can give 20 dBm but only safe with a good antenna.
    // Note that the regional legal limit RF_FreqPlan.MaxTxPower also applies,
    //   it is only 14 dBm in EU, but 30 in Americas, India & Australia.
    //if (hw_info.model != SOFTRF_MODEL_PRIME_MK2) {
        /*
         * Enforce Tx power limit until confirmation
         * that RFM95W is doing well
         * when antenna is not connected
         */
        if (tx_power > 17)  tx_power = 17;
    //}
    }
  }
}

static void set_initial_protocol(uint8_t protocol)   // only used during RF_setup()
{
  RF_FreqPlan.setPlan(settings->band, protocol);
  switch (protocol)
  {
  case RF_PROTOCOL_ADSL:
    mainprotocol_ptr = &adsl_proto_desc;
    protocol_encode = &adsl_encode;
    protocol_decode = &adsl_decode;
    break;
  case RF_PROTOCOL_OGNTP:
    mainprotocol_ptr = &ogntp_proto_desc;
    protocol_encode = &ogntp_encode;
    protocol_decode = &ogntp_decode;
    break;
  case RF_PROTOCOL_PAW:
    mainprotocol_ptr = &paw_proto_desc;
    protocol_encode = &paw_encode;
    protocol_decode = &paw_decode;
    break;
  case RF_PROTOCOL_FANET:
    mainprotocol_ptr = &fanet_proto_desc;
    protocol_encode = &fanet_encode;
    protocol_decode = &fanet_decode;
    break;
  case RF_PROTOCOL_ADSB_1090:             // only usable as alt-protocol
    mainprotocol_ptr = &es1090_proto_desc;
    protocol_encode = &es1090_encode;     // which does nothing
    protocol_decode = &es1090_decode;
    break;
  case RF_PROTOCOL_LEGACY:
    mainprotocol_ptr = &legacy_proto_desc;
    protocol_encode = &legacy_encode;     // encodes both LEGACY and LATEST
    protocol_decode = &legacy_decode;     // decodes both LEGACY and LATEST
    break;
  case RF_PROTOCOL_LATEST:
  default:
    mainprotocol_ptr = &latest_proto_desc;
    protocol_encode = &legacy_encode;     // encodes both LEGACY and LATEST
    protocol_decode = &legacy_decode;     // decodes both LEGACY and LATEST
    break;
  }
}

/*
// ADS-L HDR protocol for the 200-450 time slot:
static void set_protocol_for_uplink()
{
  curr_rx_protocol_ptr = &uplink_proto_desc;
  curr_tx_protocol_ptr = &uplink_proto_desc;
  protocol_decode = &adsl_decode;
  protocol_encode = &adsl_encode;
  current_RX_protocol = RF_PROTOCOL_ADSL;
  current_TX_protocol = RF_PROTOCOL_ADSL;
  rx_flr_adsl = false;
  RF_FreqPlan.setPlan((uint8_t) settings->band, (uint8_t) RF_PROTOCOL_PAW);
  calc_txpower();
  set_channel(0);
}
*/

uint16_t manchester_decoded(uint8_t *buf)
{
    uint8_t val1 = pgm_read_byte(&ManchesterDecode[*buf]);
    if (val1 & 0xF0)  // invalid value, rx error
        ++invalid_manchester_counter;
    buf++;
    uint8_t val2 = pgm_read_byte(&ManchesterDecode[*buf]);
    if (val2 & 0xF0)
        ++invalid_manchester_counter;
    return ((val1 & 0x0F) << 4) | (val2 & 0x0F);
}

#define REJECT_INVALID_MANCHESTER 1

static bool receive()
{
  bool sw_manchester = (curr_rx_protocol_ptr->whitening == RF_WHITENING_MANCHESTER
                         && use_hardware_manchester == false);
  uint8_t crc8, pkt_crc8;
  uint16_t crc16, pkt_crc16;
  uint8_t i;
  bool success = false;

  // rf_chip->receive() does:
  //   if not already listening:
  //     set up the radio chip
  //     start listening
  //   if nothing received return 0
  //   if packet arrived return its size
  uint8_t size = rf_chip->receive(RL_rxPacket);     // includes the CRC bytes

  //receive_active = false;

#if 0
if (settings->debug_flags & DEBUG_DEEPER2) {
if (! receive_active)   // polling said a packet was received
{
Serial.print("rf_chip->receive() returned ");
Serial.print(size);
Serial.println(" bytes:");
Serial.println(bytes2Hex((byte *)RL_rxPacket, size));
}
}
#endif

  if (size == 0)
      return false;

#if 0
  // for reasons unknown, RadioLib returned RSSIs much lower than BASICMAC
  // may want to fudge it:
  if (rf_chip->type == RF_IC_SX1262) {
      if (RF_last_rssi < -30)
          RF_last_rssi += 30;
  }
#endif

  unsigned crc_type = curr_rx_protocol_ptr->crc_type;

  if (rx_flr_adsl) {
    if (sw_manchester) {
      // examine 2 later bytes in the sync word to identify the protocol
      // - that is 4 bytes before Manchester decoding
      uint8_t byte1 = manchester_decoded(&RL_rxPacket[0]);
      uint8_t byte2 = manchester_decoded(&RL_rxPacket[2]);
      if (byte1==FLR_ID_BYTE_1 && byte2==FLR_ID_BYTE_2) {
          RF_last_protocol = RF_PROTOCOL_LATEST;
          crc_type = RF_CHECKSUM_TYPE_CCITT_FFFF;
      } else if (byte1==ADSL_ID_BYTE_1 && byte2==ADSL_ID_BYTE_2) {
          RF_last_protocol = RF_PROTOCOL_ADSL;
          crc_type = RF_CHECKSUM_TYPE_CRC_MODES;
          size -= 4;        // packet 3 bytes shorter but CRC one byte longer than Legacy
      } else {
          RF_last_protocol = RF_PROTOCOL_NONE;
          //success = false;
//Serial.printf("sw Unidentified packet protocol 0x%02x 0x%02x\r\n", byte1, byte2);
          return 0;
      }
    } else {  // Manchester hardware or no Manchester - assume inverted payload
      // examine 2 later bytes in the sync word to identify the protocol
      // - that was 4 bytes before Manchester decoding
      uint8_t byte1 = ~RL_rxPacket[0];
      uint8_t byte2 = ~RL_rxPacket[1];
      // a direct if(~RL_rxPacket[0]==FLR_ID_BYTE_1) failed!
      // - perhaps ~RL_rxPacket[0] is "upgraded" to an int with more bits?
      if (byte1==FLR_ID_BYTE_1 && byte2==FLR_ID_BYTE_2) {
          RF_last_protocol = RF_PROTOCOL_LATEST;
          crc_type = RF_CHECKSUM_TYPE_CCITT_FFFF;
      } else if (byte1==ADSL_ID_BYTE_1 && byte2==ADSL_ID_BYTE_2) {
          RF_last_protocol = RF_PROTOCOL_ADSL;
          crc_type = RF_CHECKSUM_TYPE_CRC_MODES;
          size -= 2;        // packet 3 bytes shorter but CRC one byte longer than Legacy
      } else {
          RF_last_protocol = RF_PROTOCOL_NONE;
          //success = false;
Serial.printf("hw Unidentified packet protocol 0x%02x 0x%02x s.b. 0x%02x 0x%02x\r\n",
byte1, byte2, FLR_ID_BYTE_1, FLR_ID_BYTE_2);
          return 0;
      }
    }
  }

  if (sw_manchester)
    size >>= 1;

  if (size > sizeof(RxBuffer))
      size = sizeof(RxBuffer);

//Serial.print("size=");
//Serial.println(size);
//Serial.println(bytes2Hex((byte *) RL_rxPacket, size));

  u1_t offset = curr_rx_protocol_ptr->payload_offset;    // only nonzero for PAW

  if (sw_manchester) {

    invalid_manchester_counter = 0;

    if (rx_flr_adsl) {
      // skip the unused sync bytes
      size -= 3;
      // shift the payload bits as needed
      uint16_t val = manchester_decoded(&RL_rxPacket[4]);
      byte prevbyte = (val & 0xFF);
      byte *p = &RL_rxPacket[6];
      byte *r = &RxBuffer[0];
      for (u1_t i=0; i < size; i++) {
          val = manchester_decoded(p);
#if REJECT_INVALID_MANCHESTER
          if (invalid_manchester_counter > 1) {
              // chances of passing CRC are slim, abort this packet
              ++invalid_manchester_packets;
Serial.println("invalid Manchester");
              return 0;
          }
#endif
          byte newbyte = (val & 0xFF);
          *r++ = (prevbyte << 7) | (newbyte >> 1);
          prevbyte = newbyte;
          p += 2;
      }
    } else {
      // single protocol, no bit-shifting needed, just copy by bytes
      // note: payload inversion is built into manchester_decoded()
      size -= offset;
      byte *p = &RL_rxPacket[offset+offset];
      for (u1_t i=0; i < size; i++) {
          uint16_t val = manchester_decoded(p);
#if REJECT_INVALID_MANCHESTER
          if (invalid_manchester_counter > 1) {
              // chances of passing CRC are slim, abort this packet
              ++invalid_manchester_packets;
              return 0;
          }
#endif
          RxBuffer[i] = (val & 0xFF);
          p += 2;
      }
      RF_last_protocol = current_RX_protocol;
    }

#if !REJECT_INVALID_MANCHESTER
    if (invalid_manchester_counter != 0)
        ++invalid_manchester_packets;
#endif

  } else {  // Manchester hardware or no Manchester
    if (rx_flr_adsl) {
      // skip the unused sync bytes
      size -= 3;
      // shift the payload bits as needed
      // assumes inverted payload
      byte *p = &RL_rxPacket[2];
      byte *q = &RL_rxPacket[3];
      byte *r = &RxBuffer[0];
      for (u1_t i=0; i < size; i++) {
          *r++ = ~((*p << 7) | (*q >> 1));
          ++p;
          ++q;
      }
    } else {
      // single protocol, no bit-shifting needed, just copy by bytes
      size -= offset;
      if (curr_rx_protocol_ptr->payload_type == RF_PAYLOAD_INVERTED) {
          for (u1_t i=0; i < size; i++) {
             RxBuffer[i] = ~RL_rxPacket[offset+i];
          }
      } else {
          for (u1_t i=0; i < size; i++) {
             RxBuffer[i] = RL_rxPacket[offset+i];
          }
      }
      RF_last_protocol = current_RX_protocol;
    }
  }

#if 0
Serial.print("After bit-shifting and Manchester decoding, size: ");
Serial.println(size);
Serial.println(bytes2Hex((byte *)RxBuffer, size));
#endif

//Serial.println(bytes2Hex((byte *) RxBuffer, size));

  // now can compute and check the CRC

  uint8_t size2 = size - curr_rx_protocol_ptr->crc_size;

  switch (crc_type)
  {
  case RF_CHECKSUM_TYPE_CCITT_FFFF:    // includes FLR packet in FLR_ADSL dual mode
    crc16 = 0xffff;  /* seed value */
    break;
  case RF_CHECKSUM_TYPE_CCITT_0000:
    crc16 = 0x0000;  /* seed value */
    break;
  case RF_CHECKSUM_TYPE_CRC8_107:
    crc8 = 0x71;     /* seed value */
    break;
  //case RF_CHECKSUM_TYPE_CRC_MODES:
  //case RF_CHECKSUM_TYPE_GALLAGER:
  //case RF_CHECKSUM_TYPE_NONE:
  default:
     /* crc left not initialized */
    break;
  }

  if (crc_type != RF_CHECKSUM_TYPE_CRC_MODES) {  // not ADS-L

    switch (curr_rx_protocol_ptr->type)
    {
    case RF_PROTOCOL_LATEST:    // includes FLR_ADSL dual mode
    case RF_PROTOCOL_LEGACY:
      /* take in account NRF905/FLARM "address" bytes */
      crc16 = update_crc_ccitt(crc16, 0x31);
      crc16 = update_crc_ccitt(crc16, 0xFA);
      crc16 = update_crc_ccitt(crc16, 0xB6);
      break;
    //case RF_PROTOCOL_OGNTP:
    //case RF_PROTOCOL_FANET:
    default:
      break;
    }

    for (i = 0; i < size2; i++)
    {

      switch (crc_type)
      {
      case RF_CHECKSUM_TYPE_CCITT_FFFF:    // includes FLR packet in FLR_ADSL dual mode
      case RF_CHECKSUM_TYPE_CCITT_0000:
        crc16 = update_crc_ccitt(crc16, (u1_t)(RxBuffer[i]));
        break;
      case RF_CHECKSUM_TYPE_CRC8_107:
        update_crc8(&crc8, (u1_t)(RxBuffer[i]));
        break;
      //case RF_CHECKSUM_TYPE_GALLAGER:
      //case RF_CHECKSUM_TYPE_NONE:
      default:
        break;
      }

#if 0
      switch (curr_rx_protocol_ptr->whitening)
      {
      case RF_WHITENING_NICERF:
        RxBuffer[i] ^= pgm_read_byte(&whitening_pattern[i]);
        break;
      //case RF_WHITENING_MANCHESTER:
      //case RF_WHITENING_NONE:
      default:
        break;
      }
#endif
    }

  }

  switch (crc_type)
  {
  case RF_CHECKSUM_TYPE_CCITT_FFFF:    // includes FLR packet in FLR_ADSL dual mode
  case RF_CHECKSUM_TYPE_CCITT_0000:
    pkt_crc16 = (RxBuffer[size-2] << 8 | RxBuffer[size-1]);
    if (crc16 == pkt_crc16) {
      RF_last_crc = crc16;
      success = true;
    } else {
      //success = false;
Serial.println("FLR CRC wrong");
    }
    break;
  case RF_CHECKSUM_TYPE_CRC_MODES:    // includes ADSL packet in FLR_ADSL dual mode
    //if (ADSL_Packet::checkPI((uint8_t  *) RxBuffer, size))
    // use table-driven version instead:
    if (check_adsl_crc((const uint8_t *)RxBuffer, (uint8_t) size)) {
      //success = false;
Serial.println("ADS-L CRC wrong");
    } else {
      RF_last_crc = (RxBuffer[size-3] << 16 | RxBuffer[size-2] << 8 | RxBuffer[size-1]);
      success = true;
    }
    break;
  case RF_CHECKSUM_TYPE_GALLAGER:
    if (LDPC_Check((uint8_t  *) RxBuffer)) {
      success = false;
Serial.println("OGNTP CRC wrong");
    } else {
      success = true;
    }
    break;
  case RF_CHECKSUM_TYPE_CRC8_107:
    pkt_crc8 = RxBuffer[i];
    if (crc8 == pkt_crc8) {
      RF_last_crc = crc8;
      success = true;
    } else {
      success = false;
Serial.println("PAW external CRC8 wrong");
    }
    break;
  //case RF_CHECKSUM_TYPE_NONE:
  default:
    RF_last_rx_len = size2;    // for variable-length FANET packets
    success = true;
    break;
  }

#if 1
if (success && (settings->debug_flags  & DEBUG_DEEPER)) {
uint8_t protocol = curr_rx_protocol_ptr->type;
if (rx_flr_adsl)  protocol = RF_last_protocol;
Serial.printf("RX in prot %d, time slot %d, sec %d(%d) + %d ms, RSSI %d\r\n",
    protocol, RF_current_slot, RF_time, (RF_time & 0x0F), millis()-ref_time_ms, RF_last_rssi);
}
#endif

  if (success) {
      if (curr_rx_protocol_ptr->type == RF_PROTOCOL_ADSB_1090)
          ++adsb_packets_counter;
      else if (curr_rx_protocol_ptr->type != RF_PROTOCOL_PAW)
          ++rx_packets_counter;
      // else wait to see if ADSL decoding of the PAW payload will succeed
      return true;
  }

  return false;
}

static uint8_t transmit(uint8_t passed_size) {

  bool sw_manchester = (curr_tx_protocol_ptr->whitening == RF_WHITENING_MANCHESTER
                         && ! use_hardware_manchester);
  u1_t crc8;
  u2_t crc16;

  switch (curr_tx_protocol_ptr->crc_type)
  {
  case RF_CHECKSUM_TYPE_CCITT_0000:
    crc16 = 0x0000;  /* seed value */
    break;
  case RF_CHECKSUM_TYPE_CCITT_FFFF:
    crc16 = 0xffff;  /* seed value */
    break;
  case RF_CHECKSUM_TYPE_CRC8_107:
    crc8 = 0x71;     /* seed value */
    break;
  //case RF_CHECKSUM_TYPE_GALLAGER:
  //case RF_CHECKSUM_TYPE_NONE:
  default:
     /* crc16 left not initialized */
    break;
  }

  size_t length = 0;

  switch (curr_tx_protocol_ptr->type)
  {
  case RF_PROTOCOL_LEGACY:
  case RF_PROTOCOL_LATEST:
    /* take in account NRF905/FLARM "address" bytes */
    crc16 = update_crc_ccitt(crc16, 0x31);
    crc16 = update_crc_ccitt(crc16, 0xFA);
    crc16 = update_crc_ccitt(crc16, 0xB6);
    if (crc16 == 0)  // failed to allocate lookup table
        return 0;
    break;
  case RF_PROTOCOL_PAW:
    /* insert Net ID */
    RL_txPacket[length++] = (u1_t) ((curr_tx_protocol_ptr->net_id >> 24) & 0x000000FF);
    RL_txPacket[length++] = (u1_t) ((curr_tx_protocol_ptr->net_id >> 16) & 0x000000FF);
    RL_txPacket[length++] = (u1_t) ((curr_tx_protocol_ptr->net_id >>  8) & 0x000000FF);
    RL_txPacket[length++] = (u1_t) ((curr_tx_protocol_ptr->net_id >>  0) & 0x000000FF);
    /* insert byte with payload size */
    RL_txPacket[length++] = curr_tx_protocol_ptr->payload_size;

    /* insert byte with CRC-8 seed value when necessary */
    if (curr_tx_protocol_ptr->crc_type == RF_CHECKSUM_TYPE_CRC8_107) {
      RL_txPacket[length++] = crc8;
    }
    break;
  //case RF_PROTOCOL_OGNTP:
  //case RF_PROTOCOL_ADSL:
  //case RF_PROTOCOL_FANET:
  default:
    break;
  }

  uint8_t size = curr_tx_protocol_ptr->payload_size;
  if (curr_tx_protocol_ptr->type == RF_PROTOCOL_FANET) {
      // FANET uses variable size packets
      size = passed_size;
  } else if (size != passed_size) {
      Serial.printf("size %d != passed_size %d\r\n", size, passed_size);
  }
  //uint8_t offset = curr_tx_protocol_ptr->payload_offset;    // only nonzero for PAW

//  if (curr_tx_protocol_ptr->crc_type == RF_CHECKSUM_TYPE_CRC_MODES) {
//      // CRC was computed by adsl_encode(), but not included within payload size
//      size += curr_tx_protocol_ptr->crc_size;   // 21 + 3 = 24
//  }
//  // similarly for OGNTP, CRC embedded in the payload
//  // else CRC is computed and added to the packet below

  bool inv = (curr_tx_protocol_ptr->payload_type == RF_PAYLOAD_INVERTED);

  for (uint8_t i=0; i < size; i++) {

    uint8_t c = TxBuffer[i];

    //switch (curr_tx_protocol_ptr->whitening)
    //{
    //case RF_WHITENING_NICERF:
    //  RL_txPacket[length] = c ^ pgm_read_byte(&whitening_pattern[i]);
                // PAW with embedded ADS-L no longer does whitening
    //  break;
    //case RF_WHITENING_MANCHESTER:
    //case RF_WHITENING_NONE:
    //default:
      if (sw_manchester) {
          RL_txPacket[length] = pgm_read_byte(&ManchesterEncode[(c >> 4) & 0x0F]);
          length++;
          RL_txPacket[length] = pgm_read_byte(&ManchesterEncode[(c     ) & 0x0F]);
      } else if (inv) {
          RL_txPacket[length] = ~c;
      } else {
          RL_txPacket[length] = c;
      }
    //  break;
    //}

    switch (curr_tx_protocol_ptr->crc_type)
    {
    case RF_CHECKSUM_TYPE_CCITT_FFFF:
    case RF_CHECKSUM_TYPE_CCITT_0000:
      crc16 = update_crc_ccitt(crc16, (u1_t)c);
      break;
    case RF_CHECKSUM_TYPE_CRC8_107:
      update_crc8(&crc8, (u1_t)c);
      break;
    //case RF_CHECKSUM_TYPE_CRC_MODES:
    //case RF_CHECKSUM_TYPE_GALLAGER:
    //case RF_CHECKSUM_TYPE_NONE:
    default:
      break;
    }

    length++;
  }

/*
if (settings->debug_flags & DEBUG_DEEPER2) {
Serial.println("Copied into RL_txPacket:");
Serial.println(bytes2Hex((byte *) RL_txPacket, length));
}
*/
  switch (curr_tx_protocol_ptr->crc_type)
  {
  case RF_CHECKSUM_TYPE_CCITT_FFFF:
  case RF_CHECKSUM_TYPE_CCITT_0000:
    if (sw_manchester) {
      RL_txPacket[length++] = pgm_read_byte(&ManchesterEncode[(((crc16 >>  8) & 0xFF) >> 4) & 0x0F]);
      RL_txPacket[length++] = pgm_read_byte(&ManchesterEncode[(((crc16 >>  8) & 0xFF)     ) & 0x0F]);
      RL_txPacket[length++] = pgm_read_byte(&ManchesterEncode[(((crc16      ) & 0xFF) >> 4) & 0x0F]);
      RL_txPacket[length++] = pgm_read_byte(&ManchesterEncode[(((crc16      ) & 0xFF)     ) & 0x0F]);
    } else if (inv) {
        RL_txPacket[length++] = ~((crc16 >> 8) & 0xFF);
        RL_txPacket[length++] = ~((crc16     ) & 0xFF);
    } else {
        RL_txPacket[length++] = (crc16 >> 8) & 0xFF;
        RL_txPacket[length++] = (crc16     ) & 0xFF;
    }
    break;
  case RF_CHECKSUM_TYPE_CRC8_107:
/*
    if (sw_manchester) {
      // never happens: CRC8_107 only used for PAW, which does not Manchester encode
      RL_txPacket[length++] = pgm_read_byte(&ManchesterEncode[(crc8 >> 4) & 0x0F]);
      RL_txPacket[length++] = pgm_read_byte(&ManchesterEncode[(crc8     ) & 0x0F]);
    } else {
      // inv never happens: CRC8_107 only used for PAW, which does not invert
      RL_txPacket[length++] = (inv ? ~crc8 : crc8);
    }
*/
    RL_txPacket[length++] = crc8;
    break;
  //case RF_CHECKSUM_TYPE_CRC_MODES:  // CRC put within packet by adsl_encode()
  //case RF_CHECKSUM_TYPE_GALLAGER:
  //case RF_CHECKSUM_TYPE_NONE:
  default:
    break;
  }

//Serial.println("calling rf_chip->transmit()...");
  int rl_state = rf_chip->transmit(RL_txPacket, length);

  if (rl_state == RADIOLIB_ERR_NONE) {
    //memset(RL_txPacket.payload, 0, sizeof(RL_txPacket.payload));   why ???
    return length;

  } else if (rl_state == RADIOLIB_ERR_TX_TIMEOUT) {
    // timeout occured while transmitting packet
    Serial.println(F("tx timeout!"));
    // perhaps tx happened anyway, but interrupt was not working
    // - return value as if tx happened:
    return length;

  } else if (rl_state == RADIOLIB_ERR_PACKET_TOO_LONG) {
    // the supplied packet was longer than 256 bytes
    Serial.println(F("tx too long!"));

  } else {
    // some other error occurred
    Serial.print(F("tx failed, code "));
    Serial.println((int16_t) rl_state);
  }

  return 0;
}


// The wrapper code common to all the RF chips:

#if 0
// these protocols share the frequency plan and time slots
bool in_family(uint8_t protocol)
{
    if (protocol == RF_PROTOCOL_LATEST)
        return true;
    if (protocol == RF_PROTOCOL_ADSL)
        return true;
    if (protocol == RF_PROTOCOL_OGNTP)
        return true;
    if (protocol == RF_PROTOCOL_LEGACY)
        return true;
    return false;
}
#endif

/*
uint8_t useOGNfreq(uint8_t protocol)
{
    if (protocol == RF_PROTOCOL_OGNTP)
        return 1;
    //if (protocol == RF_PROTOCOL_ADSL)
    //    return 1;
    // - switched to using FLARM frequency for ADS-L
    return 0;
}
*/

static void set_channel(uint8_t channel)
{
  static uint8_t prev_channel  = RF_CHANNEL_NONE;
  static uint8_t prev_protocol = RF_PROTOCOL_NONE;

  if (! RF_ready)
      return;          // leave prev == NONE

  uint8_t protocol = RF_FreqPlan.Protocol;

  // if continuous reception on one channel, make sure the radio is refreshed
  bool refresh = true;
  if (RF_FreqPlan.Channels > 1)   // frequency hopping
      refresh = false;
  else if (dual_protocol == RF_FLR_PAW || dual_protocol == RF_FLR_FANET)  // protocol hopping
      refresh = false;
  else if (settings->txpower != RF_TX_POWER_OFF
           && protocol != RF_PROTOCOL_FANET && protocol != RF_PROTOCOL_PAW)  // frequent tx
      refresh = false;

  if (refresh || channel != prev_channel || protocol != prev_protocol) {

    uint32_t frequency = RF_FreqPlan.getChanFrequency(channel);

    /* correction of not more than 30 kHz is allowed */
    int8_t fc = settings->freq_corr;
    //if (rf_chip->type == RF_IC_SX1276) {
      /* Most of SX1262 designs use TCXO */
      // but allow frequency correction on it anyway
    if (fc != 0) {
      if (fc > 30) {
        fc = 30;
      } else if (fc < -30) {
        fc = -30;
      };
      frequency = frequency + (fc * 1000);
    }

    //if (receive_active) {     // done within rf_chip->setfreq()
      //finishReceive();
      //receive_active = false;
    //}

    if (rf_chip)
      rf_chip->setfreq(frequency);

if (settings->debug_flags & DEBUG_DEEPER2) {
Serial.printf("Set freq for band %d, prot %d (prev %d), channel %d Hz %d\r\n",
RF_FreqPlan.Plan, RF_FreqPlan.Protocol, prev_protocol, channel, frequency);
}

    prev_channel  = channel;
    prev_protocol = protocol;
  }
}

byte RF_setup(void)
{
  // resurrected the option of auto-band:
  //if (settings->band == RF_BAND_AUTO)
  //    settings->band == RF_BAND_EU;

  // "UK" freqs now mapped to EU:
  if (settings->band == RF_BAND_UK)
      settings->band == RF_BAND_EU;

  if (settings->altprotocol == settings->rf_protocol
        //|| ! in_family(settings->rf_protocol)
        //|| ! in_family(settings->altprotocol)
        //|| (rf_chip != &sx1276_ops && rf_chip != &sx1262_ops)
        ) {
      settings->altprotocol = RF_PROTOCOL_NONE;
  }

  const char *p = protocol_lbl(settings->rf_protocol, settings->altprotocol);
  if (*p == '?')   // not a listed combination
      settings->altprotocol = RF_PROTOCOL_NONE;

  set_initial_protocol(settings->altprotocol==RF_PROTOCOL_NONE? settings->rf_protocol : settings->altprotocol);
  //RF_FreqPlan.setPlan(settings->band, settings->altprotocol);
  altprotocol_ptr = mainprotocol_ptr;
  altprotocol_encode = protocol_encode;
  altprotocol_decode = protocol_decode;

  set_initial_protocol(settings->rf_protocol);   // sets mainprotocol_ptr
  //RF_FreqPlan.setPlan(settings->band, settings->rf_protocol);
  curr_rx_protocol_ptr = mainprotocol_ptr;
  curr_tx_protocol_ptr = mainprotocol_ptr;
  mainprotocol_encode = protocol_encode;
  mainprotocol_decode = protocol_decode;
  current_RX_protocol = settings->rf_protocol;
  current_TX_protocol = settings->rf_protocol;

  Serial.printf("Main RF protocol: %d\r\n", mainprotocol_ptr->type);
  Serial.printf(" Alt RF protocol: %d\r\n", (settings->altprotocol==RF_PROTOCOL_NONE? 0 : altprotocol_ptr->type));

  if (settings->rf_protocol==RF_PROTOCOL_LATEST && settings->altprotocol==RF_PROTOCOL_ADSL) {
      if (settings->flr_adsl) {     // use dual-protocol reception trick
          dual_protocol = RF_FLR_ADSL;
          Serial.println("set up FLR_ADSL rx, FLR tx + some ADSL tx");
      } else {
          Serial.println("set up FLR rx & tx + some ADSL tx");
      }
  }
  if (settings->rf_protocol==RF_PROTOCOL_ADSL && settings->altprotocol==RF_PROTOCOL_LATEST) {
      if (settings->flr_adsl) {     // use dual-protocol reception trick
          dual_protocol = RF_FLR_ADSL;
          Serial.println("set up FLR_ADSL rx, ADSL tx + some FLR tx");
      } else {
          Serial.println("set up ADSL rx & tx + some FLR tx");
      }
  }
  if (settings->rf_protocol==RF_PROTOCOL_LATEST && settings->altprotocol==RF_PROTOCOL_OGNTP) {
      if (settings->flr_adsl) {
          dual_protocol = RF_FLR_ADSL;
          Serial.println("set up FLR_ADSL rx + some OGNTP tx");
      } else {
          Serial.println("set up FLR rx & tx + some OGNTP tx");
      }
  }
  if ((settings->rf_protocol==RF_PROTOCOL_LATEST && settings->altprotocol==RF_PROTOCOL_FANET)
  ||  (settings->rf_protocol==RF_PROTOCOL_FANET  && settings->altprotocol==RF_PROTOCOL_LATEST)) {
      dual_protocol = RF_FLR_FANET;
      if (settings->flr_adsl)
          Serial.println("set up FLR_FANET time slicing, FLR_ADSL rx + some ADSL tx");
      else
          Serial.println("set up FLR_FANET time slicing");
  }
  if (settings->rf_protocol==RF_PROTOCOL_FANET && settings->altprotocol==RF_PROTOCOL_ADSL) {
      dual_protocol = RF_FLR_FANET;
      if (settings->flr_adsl)
          Serial.println("set up FANET+ADSL time slicing, FLR_ADSL rx + ADSL tx");
      else
          Serial.println("set up FANET+ADSL time slicing");
  }
  if (settings->rf_protocol==RF_PROTOCOL_FANET && settings->altprotocol==RF_PROTOCOL_OGNTP) {
      dual_protocol = RF_FLR_FANET;
      Serial.println("set up FANET_OGNTP time slicing");
      if (settings->flr_adsl)
          Serial.println("set up FANET+OGNTP time slicing, FLR_ADSL rx, some ADSL tx");
      else
          Serial.println("set up FANET+OGNTP time slicing");
  }
  if ((settings->rf_protocol==RF_PROTOCOL_LATEST && settings->altprotocol==RF_PROTOCOL_PAW)
  ||  (settings->rf_protocol==RF_PROTOCOL_PAW    && settings->altprotocol==RF_PROTOCOL_LATEST)) {
      dual_protocol = RF_FLR_PAW;
      if (settings->flr_adsl)
          Serial.println("set up FLR_PAW time slicing, FLR_ADSL rx + some ADSL tx");
      else
          Serial.println("set up FLR_PAW time slicing");
  }
  if (settings->rf_protocol==RF_PROTOCOL_PAW && settings->altprotocol==RF_PROTOCOL_ADSL) {
      dual_protocol = RF_FLR_PAW;
      if (settings->flr_adsl)
          Serial.println("set up PAW+ADSL time slicing, FLR_ADSL rx + ADSL tx");
      else
          Serial.println("set up PAW+ADSL time slicing");
  }
  if (settings->rf_protocol==RF_PROTOCOL_PAW && settings->altprotocol==RF_PROTOCOL_OGNTP) {
      dual_protocol = RF_FLR_PAW;
      Serial.println("set up PAW_OGNTP time slicing");
      if (settings->flr_adsl)
          Serial.println("set up PAW+OGNTP time slicing, FLR_ADSL rx, some ADSL tx");
      else
          Serial.println("set up PAW+OGNTP time slicing");
  }
  if (settings->rf_protocol==RF_PROTOCOL_FANET && settings->altprotocol==RF_PROTOCOL_NONE) {
      if (settings->flr_adsl)
          Serial.println("set up FANET protocol, plus some FLR & ADSL tx");
  }
  if (settings->rf_protocol==RF_PROTOCOL_PAW && settings->altprotocol==RF_PROTOCOL_NONE) {
      if (settings->flr_adsl)
          Serial.println("set up PAW protocol, plus some FLR & ADSL tx");
  }
  if (settings->rf_protocol==RF_PROTOCOL_FANET && settings->altprotocol==RF_PROTOCOL_PAW) {
      dual_protocol = RF_FANET_PAW;
      if (settings->flr_adsl)
          Serial.println("set up FANET+PAW time slicing, some FLR & ADSL rx & tx");
      else
          Serial.println("set up FANET+PAW time slicing");
  }
  if (settings->rf_protocol==RF_PROTOCOL_PAW && settings->altprotocol==RF_PROTOCOL_FANET) {
      dual_protocol = RF_PAW_FANET;
      if (settings->flr_adsl)
          Serial.println("set up PAW+FANET time slicing, some FLR & ADSL rx & tx");
      else
          Serial.println("set up PAW+FANET time slicing");
  }

  SoC->SPI_begin();

  if (rf_chips_probe() == false)
      return RF_IC_NONE;

  set_channel(0);   // tries to initialize cur_freq - not if BAND_AUTO

  calc_txpower();   // does nothing if BAND_AUTO

  return rf_chip->type;
}

void RF_chip_channel()
{
    //uint8_t OGN = useOGNfreq(protocol);
    uint8_t OGN = (RF_FreqPlan.Protocol == RF_PROTOCOL_OGNTP ? 1 : 0);
    RF_current_chan = RF_FreqPlan.getChannel((time_t)RF_time, RF_current_slot, OGN);
    set_channel(RF_current_chan);
}

void set_protocol_for_slot()
{
  const rf_proto_desc_t  *prev_rx_protocol_ptr = curr_rx_protocol_ptr;

  // Transmit one packet in alt protocol once every 4 seconds:
  // In time Slot 0 for ADS-L & FLR, and in Slot 1 for OGNTP.
  // If alt protocol is OGNTP transmit in third protocol in seconds 3,11
  // This arrangement is not used for time-slicing with FANET or PAW

  bool sec_3_7_11_15 = ((RF_time & 0x03) == 0x03);
  bool sec_3_11      = ((RF_time & 0x07) == 0x03);

  if (RF_current_slot == 0) {

    if (dual_protocol == RF_FANET_PAW || dual_protocol == RF_PAW_FANET) {
        // new for MB204: allow combined FANET & PAW, with optional FLR_ADSL too
        if (settings->flr_adsl) {
            curr_rx_protocol_ptr = &flr_adsl_proto_desc;
            protocol_decode = &flr_adsl_decode;
            if (sec_3_7_11_15) {
                curr_tx_protocol_ptr = &adsl_proto_desc;
                protocol_encode = &adsl_encode;
            } else {
                curr_tx_protocol_ptr = &latest_proto_desc;
                protocol_encode = &legacy_encode;
            }
        } else {
/*
            if (sec_3_7_11_15) {
                curr_rx_protocol_ptr = altprotocol_ptr;
                protocol_decode = altprotocol_decode;
            } else {
                curr_rx_protocol_ptr = mainprotocol_ptr;
                protocol_decode = mainprotocol_decode;
            }
*/
            // use same rx protocol as in preceding Slot 1
            if (current_RX_protocol == RF_PROTOCOL_PAW) {
                curr_rx_protocol_ptr = &paw_proto_desc;
                protocol_decode = paw_decode;
            } else {
                curr_rx_protocol_ptr = &fanet_proto_desc;
                protocol_decode = fanet_decode;
            }
        }
    } else if (dual_protocol == RF_FLR_FANET || dual_protocol == RF_FLR_PAW) {
        // RF_FLR_FANET may also mean FANET+ADSL or FANET+OGNTP
        if (sec_3_7_11_15 && settings->flr_adsl) {
            curr_rx_protocol_ptr = &flr_adsl_proto_desc;
            protocol_decode = &flr_adsl_decode;
            if (settings->altprotocol == RF_PROTOCOL_ADSL) {
                curr_tx_protocol_ptr = &latest_proto_desc;
                protocol_encode = &legacy_encode;
            } else {     // altprotocol is Latest or OGNTP
                curr_tx_protocol_ptr = &adsl_proto_desc;
                protocol_encode = &adsl_encode;
            }
        } else if (settings->rf_protocol != RF_PROTOCOL_FANET && settings->rf_protocol != RF_PROTOCOL_PAW) {
            // mainprotocol is not FANET (or PAW), it may be LATEST or ADSL or OGNTP
            curr_tx_protocol_ptr = mainprotocol_ptr;
            protocol_encode = mainprotocol_encode;
            if (settings->flr_adsl) {
                curr_rx_protocol_ptr = &flr_adsl_proto_desc;
                protocol_decode = &flr_adsl_decode;
            } else {
                curr_rx_protocol_ptr = mainprotocol_ptr;
                protocol_decode = mainprotocol_decode;
            }
        } else if (settings->altprotocol != RF_PROTOCOL_FANET && settings->altprotocol != RF_PROTOCOL_PAW) {
            // altprotocol is not FANET (or PAW), it may be LATEST or ADSL or OGNTP
            curr_tx_protocol_ptr = altprotocol_ptr;
            protocol_encode = altprotocol_encode;
            if (settings->flr_adsl && settings->altprotocol != RF_PROTOCOL_OGNTP) {
                curr_rx_protocol_ptr = &flr_adsl_proto_desc;
                protocol_decode = &flr_adsl_decode;
            } else {
                curr_rx_protocol_ptr = altprotocol_ptr;
                protocol_decode = altprotocol_decode;
            }
        }
    } else if (sec_3_7_11_15 && settings->altprotocol != RF_PROTOCOL_NONE) {
        if (settings->altprotocol == RF_PROTOCOL_OGNTP) {
            if (sec_3_11 && settings->flr_adsl && settings->rf_protocol != RF_PROTOCOL_ADSL) {
                // transmit ADS-L every 8 sec
                curr_rx_protocol_ptr = &flr_adsl_proto_desc;
                curr_tx_protocol_ptr = &adsl_proto_desc;
                protocol_decode = &flr_adsl_decode;
                protocol_encode = &adsl_encode;
            } else {
                // stay in main protocol
                // - will transmit in OGNTP in Slot 1
                curr_rx_protocol_ptr = mainprotocol_ptr;
                curr_tx_protocol_ptr = mainprotocol_ptr;
                protocol_decode = mainprotocol_decode;
                protocol_encode = mainprotocol_encode;
            }
        } else if (settings->altprotocol == RF_PROTOCOL_ADSB_1090) {
            // will listen to ADS-B in Slot 1
            if (sec_3_11 && settings->flr_adsl) {
                // transmit ADS-L every 8 sec
                curr_rx_protocol_ptr = &flr_adsl_proto_desc;
                curr_tx_protocol_ptr = &adsl_proto_desc;
                protocol_decode = &flr_adsl_decode;
                protocol_encode = &adsl_encode;
            } else {
                // stay in main protocol
                curr_rx_protocol_ptr = mainprotocol_ptr;
                curr_tx_protocol_ptr = mainprotocol_ptr;
                protocol_decode = mainprotocol_decode;
                protocol_encode = mainprotocol_encode;
            }
        } else {    // Latest+ADSL, or ADSL+Latest
            if (settings->flr_adsl
             && (settings->altprotocol == RF_PROTOCOL_LATEST
              || settings->altprotocol == RF_PROTOCOL_ADSL)) {
                curr_rx_protocol_ptr = &flr_adsl_proto_desc;
                protocol_decode = &flr_adsl_decode;
            } else {
                curr_rx_protocol_ptr = mainprotocol_ptr;
                protocol_decode = mainprotocol_decode;
            }
            curr_tx_protocol_ptr = altprotocol_ptr;
            protocol_encode = altprotocol_encode;
        }
    } else {    // single protocol, or not sec_3_7_11_15
        if (settings->flr_adsl
         && (settings->rf_protocol == RF_PROTOCOL_LATEST || settings->rf_protocol == RF_PROTOCOL_ADSL)) {
            curr_rx_protocol_ptr = &flr_adsl_proto_desc;
            protocol_decode = &flr_adsl_decode;
        } else if (sec_3_7_11_15 && settings->flr_adsl && settings->rf_protocol == RF_PROTOCOL_OGNTP) {
            curr_rx_protocol_ptr = &flr_adsl_proto_desc;
            protocol_decode = &flr_adsl_decode;
        } else {
            curr_rx_protocol_ptr = mainprotocol_ptr;
            protocol_decode = mainprotocol_decode;
        }
        // new for MB204: option for some transmit & receive of FLR & ADSL
        //   even while in single-protocol FANET or PAW mode:
        if (settings->rf_protocol == RF_PROTOCOL_FANET || settings->rf_protocol == RF_PROTOCOL_PAW) {
            if (settings->flr_adsl) {
                // listen to flr_adsl instead of FANET/PAW one Slot 0
                curr_rx_protocol_ptr = &flr_adsl_proto_desc;
                protocol_decode = &flr_adsl_decode;
                if (sec_3_7_11_15) {
                    // transmit in ADS-L once every 4 seconds
                    curr_tx_protocol_ptr = &adsl_proto_desc;
                    protocol_encode = &adsl_encode;
                } else {
                    // transmit in Latest protocol in 3 of every 4 seconds
                    curr_tx_protocol_ptr = &latest_proto_desc;
                    protocol_encode = &legacy_encode;
                }
            } else {
                // skip rx & tx in flr_adsl in Slot 0
                // (but no actual FANET or PAW tx happens in Slot 0)
                curr_tx_protocol_ptr = mainprotocol_ptr;
                protocol_encode = mainprotocol_encode;
            }
        } else {    // single-protocol LATEST, ADSL or OGNTP
            curr_tx_protocol_ptr = mainprotocol_ptr;
            protocol_encode = mainprotocol_encode;
        }
    }

  } else {  // slot 1

    if (dual_protocol == RF_FANET_PAW || dual_protocol == RF_PAW_FANET) {
        // new for MB204: allow combined FANET & PAW
/*
        if (sec_3_7_11_15) {
            curr_rx_protocol_ptr = altprotocol_ptr;
            protocol_decode = altprotocol_decode;
        } else {
            curr_rx_protocol_ptr = mainprotocol_ptr;
            protocol_decode = mainprotocol_decode;
        }
*/
        // PAW transmits 3x more often than FANET.
        // Listen to FANET 3/4 of the seconds, else PAW:
#if 0
        //if (millis() & 0x03)
        if (SoC->random(0,400) < 300) {
            curr_rx_protocol_ptr = &fanet_proto_desc;
            protocol_decode = &fanet_decode;
        } else {
            curr_rx_protocol_ptr = &paw_proto_desc;
            protocol_decode = &paw_decode;
        }
        // TX protocol depends on a 3- or 4-step cycle:
        if (next_tx_protocol == 0) {
            curr_tx_protocol_ptr = &fanet_proto_desc;
            protocol_encode = &fanet_encode;
        } else {    // 1..3
            curr_tx_protocol_ptr = &paw_proto_desc;
            protocol_encode = &paw_encode;
        }
#else
        // synchronize rx & tx to maximize communications between SoftRF devices,
        // but still listen to FANET more often,and transmit PAW more often:
        // sec  RX     TX
        //   0  PAW    PAW
        //   1  FANET  PAW
        //   2  FANET  PAW   - or PAW rx if dual_protocol == RF_PAW_FANET
        //   3  FANET  FANET
        //   4  FANET  PAW
        //   5  PAW    PAW
        //   6  FANET  FANET
        //   7  FANET  PAW
        // but transmissions actually happen on average every 2 sec, with random variation
        uint32_t RF_time_7 = (RF_time & 0x07);
        if (RF_time_7 == 0 || RF_time_7 == 5 || (RF_time_7 == 2 && dual_protocol == RF_PAW_FANET)) {
            curr_rx_protocol_ptr = &paw_proto_desc;
            protocol_decode = &paw_decode;
        } else {
            curr_rx_protocol_ptr = &fanet_proto_desc;
            protocol_decode = &fanet_decode;
        }
        if (RF_time_7 == 3 || RF_time_7 == 6) {
            curr_tx_protocol_ptr = &fanet_proto_desc;
            protocol_encode = &fanet_encode;
        } else {
            curr_tx_protocol_ptr = &paw_proto_desc;
            protocol_encode = &paw_encode;
        }
#endif
    } else if (dual_protocol == RF_FANET_ADSB || dual_protocol == RF_PAW_ADSB) {
        // for future use:
        if (sec_3_7_11_15) {
            curr_rx_protocol_ptr = altprotocol_ptr;     // ADSB
            protocol_decode = altprotocol_decode;
        } else {
            curr_rx_protocol_ptr = mainprotocol_ptr;    // FANET or PAW
            protocol_decode = mainprotocol_decode;
        }
        curr_tx_protocol_ptr = mainprotocol_ptr;        // FANET or PAW
        protocol_encode = mainprotocol_encode;
    } else if (dual_protocol == RF_FLR_FANET) {
        // FANET+ (at least XCtracer) only transmits FLARM in Slot 1 of odd seconds (?)
        // So listen for FLARM in Slot 1 every 4 seconds in odd seconds
        // This reduces the reception of FANET by 25%
        if (sec_3_7_11_15) {
            if (settings->flr_adsl) {
                curr_rx_protocol_ptr = &flr_adsl_proto_desc;
                protocol_decode = &flr_adsl_decode;
            } else {
                curr_rx_protocol_ptr = &latest_proto_desc;
                protocol_decode = &legacy_decode;
            }
        } else {
            curr_rx_protocol_ptr = &fanet_proto_desc;
            protocol_decode = &fanet_decode;
        }
        curr_tx_protocol_ptr = &fanet_proto_desc;
        protocol_encode = &fanet_encode;
    } else if (dual_protocol == RF_FLR_PAW) {
        curr_rx_protocol_ptr = &paw_proto_desc;
        curr_tx_protocol_ptr = &paw_proto_desc;
        protocol_decode = &paw_decode;
        protocol_encode = &paw_encode;
    } else if (sec_3_7_11_15 && settings->altprotocol == RF_PROTOCOL_OGNTP) {
        if (sec_3_11 && settings->flr_adsl && settings->rf_protocol != RF_PROTOCOL_ADSL) {
            // stay in main protocol - transmitted ADSL in slot 0
            curr_rx_protocol_ptr = mainprotocol_ptr;
            curr_tx_protocol_ptr = mainprotocol_ptr;
            protocol_decode = mainprotocol_decode;
            protocol_encode = mainprotocol_encode;
        } else {
            curr_rx_protocol_ptr = mainprotocol_ptr;
            curr_tx_protocol_ptr = &ogntp_proto_desc;
            protocol_decode = mainprotocol_decode;
            protocol_encode = &ogntp_encode;
        }
    } else {
        curr_tx_protocol_ptr = mainprotocol_ptr;
        protocol_encode = mainprotocol_encode;
        if (settings->altprotocol == RF_PROTOCOL_ADSB_1090) {
            curr_rx_protocol_ptr = altprotocol_ptr;
            protocol_decode = altprotocol_decode;
        } else {
            curr_rx_protocol_ptr = mainprotocol_ptr;
            protocol_decode = mainprotocol_decode;
        }
    }
    // note: no flr_adsl rx in slot 1 even if settings->flr_adsl
    //   (other than sec_3_7_11_15 in dual_protocol RF_FLR_FANET)
  }

  current_RX_protocol = curr_rx_protocol_ptr->type;
  current_TX_protocol = curr_tx_protocol_ptr->type;
  rx_flr_adsl = (curr_rx_protocol_ptr == &flr_adsl_proto_desc);

  if (prev_rx_protocol_ptr != curr_rx_protocol_ptr) {
      RF_FreqPlan.setPlan(settings->band, current_RX_protocol);
      calc_txpower();
      // tx will switch to curr_tx_protocol_ptr
  }
  RF_chip_channel();

/*
  if (dual_protocol == RF_SINGLE_PROTOCOL && current_TX_protocol != settings->rf_protocol) {
      Serial.print("set up to tx one time slot in protocol ");
      Serial.print(current_TX_protocol);
      Serial.print(" rather than ");
      Serial.println(settings->rf_protocol);
  }
*/
}

void RF_loop()
{
  if (!RF_ready) {
    if (RF_FreqPlan.Plan == RF_BAND_AUTO) {
        if (settings->band == RF_BAND_AUTO)
            return;   // band is not known yet
        // if settings->band was AUTO, changed in SoftRF.ino after GNSS fix
        RF_FreqPlan.setPlan(settings->band, current_RX_protocol);
        calc_txpower();
    }
    RF_ready = true;
  }

//  if (dual_protocol == RF_SINGLE_PROTOCOL
//   && current_RX_protocol == settings->rf_protocol
//   && in_family(settings->rf_protocol) == false) {

#if 0
  if (settings->altprotocol == RF_PROTOCOL_NONE
   && in_family(settings->rf_protocol) == false) {
      RF_SetChannel();    /* use original code */
      return;
  }
#endif

  /* Experimental code by Moshe Braner */
  /* More correct on frequency hopping & time slots, and uses less CPU time */
  /* - requires OurTime to be set to UTC time in seconds - done in Time_loop() */
  /* - also needs time since PPS, it is stored in ref_time_ms */

  if (ref_time_ms == 0)    /* no GNSS time yet */
    return;

  uint32_t now_ms = millis();

  // no need to adjust RF_time until setting up a new Time Slot
  if (now_ms < RF_OK_until)      /* channel already computed */
      return;

  RF_time = OurTime;      // OurTime may have been updated in Time_loop since last RF_loop

  if (now_ms < ref_time_ms) {   /* should not happen */
    --OurTime;
    ref_time_ms -= 1000;
    return;
  }

  uint32_t ms_since_pps = now_ms - ref_time_ms;
  if (ms_since_pps >= 1300) {   // should not happen since Time_loop takes care of this
    ++OurTime;
    ++RF_time;
    ref_time_ms += 1000;
    ms_since_pps -= 1000;
  }

  uint32_t slot_base_ms = ref_time_ms;
  if (ms_since_pps < 300) {  /* does not happen often? */
    /* channel does _NOT_ change at PPS rollover in middle of slot 1 */
    /* - therefore change the reference second to the previous one: */
    --RF_time;
    slot_base_ms -= 1000;
    ms_since_pps += 1000;
  }

  bool sec_15 = ((RF_time & 0x0F) == 0x0F);

  if (ms_since_pps >= 380 && ms_since_pps < 800) {

    RF_current_slot = 0;
    set_protocol_for_slot();
    RF_OK_from  = slot_base_ms + 405;
    RF_OK_until = slot_base_ms + 800;
    TxEndMarker = slot_base_ms + 795;
    if (current_TX_protocol == RF_PROTOCOL_ADSB_1090) {
        TxTimeMarker = TxEndMarker;     // prevent transmission
    } else if (current_TX_protocol == RF_PROTOCOL_ADSL) {  // ADS-L slot starts at 450
        TxTimeMarker = slot_base_ms + 455 + SoC->random(0, 335);
    } else if (current_TX_protocol == RF_PROTOCOL_FANET) {
        TxTimeMarker = RF_OK_until;
    } else if (current_TX_protocol == RF_PROTOCOL_PAW) {
        TxTimeMarker = RF_OK_until;
    } else {
        TxTimeMarker = slot_base_ms + 405 + SoC->random(0, 385);
    }

  } else if (ms_since_pps >= 800 && ms_since_pps < 1300) {

    RF_current_slot = 1;
    set_protocol_for_slot();
    //int neighbors;
    //if (settings->rf_protocol == RF_PROTOCOL_FANET || settings->altprotocol == RF_PROTOCOL_FANET) {
        //(void) Traffic_Count();   // called often enough by NMEA_Export()
        //neighbors = fanet_acfts;
    //}
    /* channel does _NOT_ change at PPS rollover in middle of slot 1 */
    RF_OK_from  = slot_base_ms + 805;
    RF_OK_until = slot_base_ms + 1380;
    if (current_TX_protocol == RF_PROTOCOL_FANET
     || current_TX_protocol == RF_PROTOCOL_PAW) {
        TxTimeMarker = RF_OK_until;                      // transmit only in FANET/PAW
        TxEndMarker = slot_base_ms + 1370;
        if (TxTimeMarker2 < RF_OK_from) {                // tx happened, or was missed
            uint32_t interval;
            if (dual_protocol == RF_FANET_PAW || dual_protocol == RF_PAW_FANET) {
                interval =  805;
#if 0
                if (fanet_acfts < 4 && next_tx_protocol > 2) {
                    next_tx_protocol = 0;      // skip one PAW tx and go on to FANET tx
//Serial.print("next_tx_protocol = ");
//Serial.println(next_tx_protocol);
                }
#endif
            } else if (current_TX_protocol == RF_PROTOCOL_PAW) {
                interval = 1805;
            } else {              // FANET without PAW (may be with Latest or ADS-L)
                interval = 3805 + 1000 * (fanet_acfts >> 1);
            }
            // Slot 1 for this purpose is 805..1370, length 565 ms
            uint32_t when = SoC->random(0, 942);
            if (when < 565)       // 60% chance
                when += 1000;     // tx any time in slot 1, some second in the future
            else if (when > 754)  // 20% chance
                when -= 377;      // tx in last 33% of slot 1 but in preceding second
            else                        // remaining 20% chance
                when += (2000 - 565);   // in first 33% of slot 1 in following second
            TxTimeMarker2 = slot_base_ms + interval + when;
            // average interval 3 sec for PAW, 5+ sec for FANET,
            // 2 sec for alternating PAW(3x)+FANET, i.e. 2.7s for PAW and 8 for FANET
        }
    } else if (current_TX_protocol == RF_PROTOCOL_ADSB_1090) {
        TxTimeMarker = TxEndMarker;     // prevent transmission
    } else if (current_TX_protocol == RF_PROTOCOL_ADSL) {
        // For ADS-L the official time slot ends at 1000,
        // and also not supposed to transmit fix more than 500 ms old:
        // Only transmit ADS-L in Slot 0, except for relaying:
        TxEndMarker = slot_base_ms + 995;
        TxTimeMarker = TxEndMarker;         // prevent transmission (relay bypasses this)
    } else if (sec_15 && current_TX_protocol == RF_PROTOCOL_LATEST) {
        // Some other receivers may mis-decrypt packets sent after the next PPS
        // so limit the transmissions to the pre-PPS half of the slot.
        TxEndMarker = slot_base_ms + 995;
        //TxTimeMarker = slot_base_ms + 805 + SoC->random(0, 185);
        TxTimeMarker = slot_base_ms + 805 + SoC->random(0, 385);
        if (TxTimeMarker > TxEndMarker)     // in 50% of the cases no tx in slot 1 in sec 15:
            TxTimeMarker = TxEndMarker;     // prevent transmission (relay bypasses this)
    } else {   // Legacy, OGNTP
        TxEndMarker = slot_base_ms + 1195;
        TxTimeMarker = slot_base_ms + 805 + SoC->random(0, 385);
    }

  } else { /* now 300-380 ms and somehow have not set up Slot 1 */

//Serial.println("<380 ms and somehow...");
    RF_current_slot = 1;
    set_protocol_for_slot();
    RF_OK_until = slot_base_ms + 380;
    RF_OK_from   = RF_OK_until;
    TxTimeMarker = RF_OK_until;          /* do not transmit for now */
    TxEndMarker  = RF_OK_until;
    return;
  }

/*
  if (settings->debug_flags & DEBUG_DEEPER) {
      Serial.print("New freq at: ");
      Serial.print(now_ms - slot_base_ms);                   
      Serial.print(" ms after PPS, OK until ");
      Serial.println(RF_OK_until - slot_base_ms);
  }
*/
if (settings->debug_flags & DEBUG_DEEPER) {
int rxprot = (rx_flr_adsl? 0xD : current_RX_protocol);
uint32_t txtime = ((current_TX_protocol==RF_PROTOCOL_FANET || current_TX_protocol==RF_PROTOCOL_PAW)?
                        TxTimeMarker2 : TxTimeMarker);
Serial.printf("Prot %X/%d, Slot %d set for sec %d at PPS+%d ms, PPS %d, tx ok %d - %d, gd to %d\r\n",
rxprot, current_TX_protocol, RF_current_slot, (RF_time & 0x0F),
ms_since_pps, slot_base_ms, txtime, TxEndMarker, RF_OK_until);
}
}

bool RF_Transmit_Happened()
{
    if (/* dual_protocol == RF_FLR_FANET && */ current_TX_protocol == RF_PROTOCOL_FANET)
        return (TxTimeMarker2 == 0);
    if (/* dual_protocol == RF_FLR_PAW && */ current_TX_protocol == RF_PROTOCOL_PAW)
        return (TxTimeMarker2 == 0);
    //if (! TxEndMarker)
    //    return (TxTimeMarker > millis());   // for protocols handled by the original code
    return (TxTimeMarker >= RF_OK_until);
}

bool RF_Transmit_Ready(bool wait)
{
    //if (RF_Transmit_Happened())
    //    return false;
    uint32_t now_ms = millis();
    if ((/* dual_protocol == RF_FLR_FANET && */ current_TX_protocol == RF_PROTOCOL_FANET)
    ||  (/* dual_protocol == RF_FLR_PAW   && */ current_TX_protocol == RF_PROTOCOL_PAW))
        return (TxTimeMarker2 != 0 && now_ms > TxTimeMarker2 && now_ms < TxEndMarker);
    //if (! TxEndMarker)                     // for protocols handled by the original code
    //    return (now_ms > TxTimeMarker);
    return (now_ms >= (wait? TxTimeMarker : RF_OK_from) && now_ms < TxEndMarker);
}

// postpone transmission by one time slot
void RF_Transmit_Postpone(bool paw)
{
    if (paw) {
        if (TxTimeMarker2 != 0 && TxTimeMarker2 < TxEndMarker)
            TxTimeMarker2 += 1000;
    } else {
        if (TxTimeMarker != 0 && TxTimeMarker < TxEndMarker)
            TxTimeMarker = TxEndMarker;
    }
}

size_t RF_Encode(container_t *fop, bool wait)
{
  if (settings->txpower == RF_TX_POWER_OFF)
      return 0;

  if (! RF_ready)
      return 0;

  size_t size = 0;

  if (protocol_encode) {

    /* sanity checks: don't send bad data */
    const char *p = NULL;
    static uint32_t last_bad = 0;
    if (fop->latitude == 0.0)
        p = "position";
    else if (fop->altitude > 30000.0)   // meters
        p = "altitude";
    else if (fop->speed > (300.0 / _GPS_MPS_PER_KNOT))   // 300 m/s or about 600 knots
        p = "speed";
    else if (fabs(fop->vs) > (20.0 * (_GPS_FEET_PER_METER * 60.0))
             && fop->aircraft_type != AIRCRAFT_TYPE_JET && fop->aircraft_type != AIRCRAFT_TYPE_UFO)
        p = "vs";
    else if (fabs(fop->turnrate) > 100.0)
        p = "turnrate";
    if (p) {
        if (millis() > last_bad + 1000) {
            Serial.print("skipping sending bad ");
            Serial.println(p);
            last_bad = millis();
        }
        return 0;
    }

    // encode in the current tx protocol
    if (RF_Transmit_Ready(wait)) 
        size = (*protocol_encode)((void *) &TxBuffer[0], fop);

  }

  return size;
}

bool RF_Transmit(size_t size, bool wait)   // called with no-wait only for RELAY_ONLY
{
//Serial.println("RF_transmit()...");

  if (current_TX_protocol == RF_PROTOCOL_ADSB_1090 ||
      current_TX_protocol == RF_PROTOCOL_ADSB_UAT) {
    return false;   /* no transmit on 1090 or 978 MHz */
  }


  if (!RF_ready || rf_chip==NULL || size == 0)
      return false;

  size_t RF_tx_size;

      /* Experimental code by Moshe Braner */

      if (RF_Transmit_Ready(wait)) {

        if (current_TX_protocol == RF_PROTOCOL_ADSL) {
            // for relaying in ADS-L instead of normal protocol
            curr_tx_protocol_ptr = &adsl_proto_desc;
        }

        if (current_TX_protocol != current_RX_protocol) {
            RF_FreqPlan.setPlan(settings->band, current_TX_protocol);
            calc_txpower();
            RF_chip_channel();
        }

        if (settings->txpower != RF_TX_POWER_OFF) {
            RF_tx_size = transmit((uint8_t)size);
            if (RF_tx_size)
                tx_packets_counter++;
//else
//Serial.println("... RF_tx_size=0");

if (settings->debug_flags & DEBUG_DEEPER) {
Serial.printf("TX in protocol %d at %d ms, size=%d, RF_tx_size=%d\r\n",
    current_TX_protocol, millis()-ref_time_ms, size, RF_tx_size);
if (settings->debug_flags & DEBUG_DEEPER2)
Serial.println(bytes2Hex((byte *) TxBuffer, size));
}
        }

        if (current_TX_protocol == RF_PROTOCOL_FANET
         || current_TX_protocol == RF_PROTOCOL_PAW) {
            TxTimeMarker2 = 0;
#if 0
            if (dual_protocol == RF_FANET_PAW || dual_protocol == RF_PAW_FANET) {
                ++next_tx_protocol;
                if (next_tx_protocol > 3)
                    next_tx_protocol = 0;
//Serial.print("next_tx_protocol = ");
//Serial.println(next_tx_protocol);
            }
#endif
        } else {
            TxTimeMarker = RF_OK_until;
               // do not transmit again (even relay) until next slot
        }
        /* do not set next transmit time here - it is done in RF_loop() */
        //if (curr_tx_protocol_ptr != curr_rx_protocol_ptr)
        if (current_TX_protocol != current_RX_protocol) {
            // go back to rx mode ASAP
            //delay(1);
            RF_FreqPlan.setPlan(settings->band, current_RX_protocol);
            RF_chip_channel();
            //Serial.println("returned to normal protocol...");
        }

        return (RF_tx_size != 0);
      }

Serial.println("... not RF_ready");

  return false;
}

bool RF_Receive(void)
{
  if (settings->power_save & POWER_SAVE_NORECEIVE)
      return false;
  if (!RF_ready)
      return false;
  if (!rf_chip)
      return false;

  if (receive() == false)
      return false;

//Serial.printf("rx at %d s + %d ms\r\n", OurTime, millis()-ref_time_ms);

#if REJECT_INVALID_MANCHESTER
if (receive_cb_count & 0x01FF == 0)
Serial.printf("%d packets bad CRC + %d bad manchester out of %d rx (%d good ADS-B)\r\n",
receive_cb_count - invalid_manchester_packets - rx_packets_counter - adsb_packets_counter,
invalid_manchester_packets, receive_cb_count, adsb_packets_counter);
#else
if (receive_cb_count & 0x01FF == 0)
Serial.printf("%d packets bad CRC out of (%d rx, %d bad manchester kept) (%d good ADS-B)\r\n",
receive_cb_count - rx_packets_counter, receive_cb_count,
invalid_manchester_packets, adsb_packets_counter);
#endif

  return true;
}

void RF_Shutdown(void)
{
  if (rf_chip) {
    rf_chip->shutdown();
  }
}

#if 0
uint8_t RF_Payload_Size(uint8_t protocol)
{
  switch (protocol)
  {
    case RF_PROTOCOL_LATEST:    return latest_proto_desc.payload_size;
    case RF_PROTOCOL_LEGACY:    return legacy_proto_desc.payload_size;
    case RF_PROTOCOL_OGNTP:     return ogntp_proto_desc.payload_size;
    case RF_PROTOCOL_ADSL:      return adsl_proto_desc.payload_size;
    case RF_PROTOCOL_PAW:       return paw_proto_desc.payload_size;
    case RF_PROTOCOL_FANET:     return fanet_proto_desc.payload_size;
#if !defined(EXCLUDE_UAT978)
    case RF_PROTOCOL_ADSB_UAT:  return uat978_proto_desc.payload_size;
#endif
    default:                    return 0;
  }
}
#endif

