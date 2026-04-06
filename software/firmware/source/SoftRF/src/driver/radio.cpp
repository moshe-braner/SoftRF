/*
 * radio.cpp
 * by Moshe Braner
 *
 * Code to handle the various types of radio modules
 * All modules accessed via the RadioLib library
 *
 * Based in part on radiolib.cpp by Linar Yusupov
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

#include "RF.h"
#include "../system/SoC.h"
#include "../driver/Battery.h"
#include "Settings.h"

#include <RadioLib.h>

#ifndef RadioSPI
#define RadioSPI        SPI
#endif

static Module *mod;

float Vtcxo = 1.6;   // safe default?

bool use_hardware_manchester = false;
bool receive_active = false;
volatile bool receive_complete = false;
uint32_t receive_cb_count = 0;
static bool transmit_complete = false;
static float cur_freq;

void receive_handler(void) {
  receive_complete = true;
}

uint8_t tx_power = 2;

static uint8_t calc_txpower()
{
  uint8_t power = 2;   /* 2 dBm is minimum for RFM95W on PA_BOOST pin */

  if (settings->txpower == RF_TX_POWER_FULL) {

    /* Load regional max. EIRP at first */
    power = RF_FreqPlan.MaxTxPower;

    if (settings->relay >= RELAY_ONLY) {
        if (power > 8)  power = 8;
    }

    if (rf_chip->type == RF_IC_LR11XX) {
#if 1
      /*
       * Enforce Tx power limit until confirmation
       * that LR11xx is doing well
       * when antenna is not connected
       */
      if (power > 17)  power = 17;
#else
      if (power > 22)  power = 22;
#endif

      uint32_t frequency = RF_FreqPlan.getChanFrequency(0);
      bool high = (frequency > 1000000000) ; /* above 1GHz */
      if (high && power > 13) power = 13;

    } else if (rf_chip->type == RF_IC_SX1262) {

      /* SX1262 is unable to give more than 22 dBm */
      //if (LMIC.txpow > 22)
      //  LMIC.txpow = 22;
      // The sx1262 has internal protection against antenna mismatch.
      // And yet the T-Echo instructions warn against using without an antenna.
      // But keep is a bit lower ayway
      if (power > 19)  power = 19;

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
        if (power > 17)  power = 17;
    //}
    }
  }

  return power;
}

#if defined(INCLUDE_SX127X) || defined(INCLUDE_SX126X)

#if defined(INCLUDE_SX127X)
SX1276  *radio_sx1276;
static bool sx1276_probe(void);
static void sx1276_setup(const rf_proto_desc_t*, bool);
static void sx1276_setfreq(uint32_t);
static uint8_t sx1276_receive(uint8_t *packet);
static int16_t sx1276_transmit(uint8_t *packet, uint8_t length);
static void sx1276_shutdown(void);
const rfchip_ops_t sx1276_ops = {
  RF_IC_SX1276,
  "SX127x",
  //sx1276_probe,
  //sx1276_setup,
  sx1276_setfreq,
  sx1276_receive,
  sx1276_transmit,
  sx1276_shutdown
};
#endif

#if defined(INCLUDE_SX126X)
SX1262  *radio_sx1262;
static bool sx1262_probe(void);
static void sx1262_setup(const rf_proto_desc_t*, bool);
static void sx1262_setfreq(uint32_t);
static uint8_t sx1262_receive(uint8_t *packet);
static int16_t sx1262_transmit(uint8_t *packet, uint8_t length);
static void sx1262_shutdown(void);
const rfchip_ops_t sx1262_ops = {
  RF_IC_SX1262,
  "SX126x",
  //sx1262_probe,
  //sx1262_setup,
  sx1262_setfreq,
  sx1262_receive,
  sx1262_transmit,
  sx1262_shutdown
};
#endif

bool sx1276_probe()
{
  uint32_t irq  = (lmic_pins.dio[0] == LMIC_UNUSED_PIN) ? RADIOLIB_NC : lmic_pins.dio[0];
  uint32_t busy = (lmic_pins.busy   == LMIC_UNUSED_PIN) ? RADIOLIB_NC : lmic_pins.busy;
  mod = new Module(lmic_pins.nss, irq, lmic_pins.rst, busy, RadioSPI);
  radio_sx1276 = new SX1276(mod);
  int state = radio_sx1276->beginFSK();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("Found sx1276"));
  } else {
    Serial.print(F("did not find sx1276"));
    delete radio_sx1276;
    delete mod;
    return false;
  }
if (settings->debug_flags != 0)
  use_hardware_manchester = true;   // only on sx1276 - does not work yet
  return true;
}

static void sx1276_setfreq(uint32_t ifreq)
{
    if (receive_active) {
      (void) radio_sx1276->finishReceive();
      (void) radio_sx1276->standby();
      receive_active = false;
    }

    cur_freq = 0.000001f * (float) ifreq;
    int rl_state = radio_sx1276->setFrequency(cur_freq);
#if RADIOLIB_DEBUG_BASIC
    if (rl_state == RADIOLIB_ERR_INVALID_FREQUENCY) {
      Serial.println(F("[sx1276] Selected frequency is invalid for this module!"));
      return;
    }
#endif
    // will also re-set the frequency after begin() in sx1276_setup()
}

static void sx1276_setup(const rf_proto_desc_t *rf_protocol, bool tx)
{
  static const rf_proto_desc_t *prev_protocol = NULL;
  static bool prev_tx = false;

  int rl_state;

  if (rf_protocol == prev_protocol) {
    // if only frequency changed, that was done by set_protocol_for_slot()
    //   (indirectly) calling rf_chip->setfreq()
    if (tx == prev_tx) {
        // nothing changed
        return;
    }
    // same protocol but tx changed, only need to change sync word if FSK
    if (rf_protocol->modulation_type == RF_MODULATION_TYPE_2FSK) {
      uint8_t *syncword = (uint8_t *) rf_protocol->syncword;
      uint8_t syncword_size = rf_protocol->syncword_size;
      if (tx == false) {   // for rx skip some leading sync bytes
          syncword += rf_protocol->syncword_skip;
          syncword_size -= rf_protocol->syncword_skip;
      }
      rl_state = radio_sx1276->setSyncWord(syncword, (size_t) syncword_size);
    }
    prev_tx = tx;
    return;
  }
  
  prev_protocol = rf_protocol;
  prev_tx = tx;

  float br, bw, fdev;

  switch (rf_protocol->modulation_type)
  {
  case RF_MODULATION_TYPE_LORA:

    switch (RF_FreqPlan.Bandwidth)
    {
    case RF_RX_BANDWIDTH_SS_125KHZ:
      bw = 250.0f; /* BW_250 */
      break;
    case RF_RX_BANDWIDTH_SS_250KHZ:
      bw = 500.0f; /* BW_500 */
      break;
    case RF_RX_BANDWIDTH_SS_62KHZ:
      bw = 125.0f; /* BW_125 */
      break;
    //case RF_RX_BANDWIDTH_SS_125KHZ:
    default:
      bw = 250.0f; /* BW_250 */
      break;
    }

    //rl_state = radio_sx1276->setBandwidth(bw);

#if 0
    //switch (rf_protocol->type)
    {
    //case RF_PROTOCOL_FANET:
    //default:
      rl_state = radio_sx1276->setSpreadingFactor(7); /* SF_7 */
      rl_state = radio_sx1276->setCodingRate(5);      /* CR_5 */
      //break;
    }
#endif

    rl_state = radio_sx1276->begin(cur_freq, bw, 7, 5, rf_protocol->syncword[0], tx_power, 8, 0);
    delay(1);
#if RADIOLIB_DEBUG_BASIC
    if (rl_state == RADIOLIB_ERR_INVALID_BANDWIDTH) {
      Serial.println(F("[sx1276] Selected bandwidth is invalid for this module!"));
      return;
    } else if (rl_state != RADIOLIB_ERR_NONE) {
      Serial.println(F("[sx1276] LORA begin() error!"));
      return;
    }
#endif

    //rl_state = radio_sx1276->setSyncWord((uint8_t) rf_protocol->syncword[0]);

    //rl_state = radio_sx1276->setPreambleLength(8);

    rl_state = radio_sx1276->setCrcFiltering(false);

    break;

  case RF_MODULATION_TYPE_2FSK:
  case RF_MODULATION_TYPE_PPM: /* TBD */
  default:

    //rl_state = radio_sx1276->setModem(RADIOLIB_MODEM_FSK);
    // - setModem() for the sx1276 does an implicit begin()!
    // - so first compute needed parameters, then call begin():

    switch (rf_protocol->bitrate)
    {
    case RF_BITRATE_38400:
      br = 38.4f;
      break;
    //case RF_BITRATE_100KBPS:
    default:
      br = 100.0f;
      break;
    }
    //rl_state = radio_sx1276->setBitRate(br);

    switch (rf_protocol->deviation)
    {
    case RF_FREQUENCY_DEVIATION_12_5KHZ:
      fdev = 12.5f;
      break;
    case RF_FREQUENCY_DEVIATION_25KHZ:
      fdev = 25.0f;
      break;
    //case RF_FREQUENCY_DEVIATION_50KHZ:
    default:
      fdev = 50.0f;
      break;
    }
    //rl_state = radio_sx1276->setFrequencyDeviation(fdev);

    switch (rf_protocol->bandwidth)
    {
    case RF_RX_BANDWIDTH_SS_125KHZ:
      bw = 234.3f;
      break;
    case RF_RX_BANDWIDTH_SS_250KHZ:
    case RF_RX_BANDWIDTH_SS_200KHZ:
      bw = 467.0f;
      break;
    case RF_RX_BANDWIDTH_SS_50KHZ:
      bw = 117.3f;
      break;
    case RF_RX_BANDWIDTH_SS_62KHZ:
      bw = 156.2f;
      break;
    case RF_RX_BANDWIDTH_SS_166KHZ:
      bw = 312.0f;
      break;
    //case RF_RX_BANDWIDTH_SS_100KHZ:
    default:
      bw = 234.3f;
      break;
    }
    //rl_state = radio_sx1276->setRxBandwidth(bw);

    uint16_t preamble = ((uint16_t)(rf_protocol->preamble_size) << 3);

    //rl_state = radio_sx1276->setPreambleLength(rf_protocol->preamble_size * 8);
    ////rl_state = radio_sx1276->setDataShaping(RADIOLIB_SHAPING_0_5);

    rl_state = radio_sx1276->beginFSK(cur_freq, br, fdev, bw, tx_power, preamble, false);
    delay(1);
#if RADIOLIB_DEBUG_BASIC
  if (rl_state == RADIOLIB_ERR_INVALID_BIT_RATE) {
    Serial.println(F("[sx1276] Selected bit rate is invalid for this module!"));
    return;
  } else if (rl_state != RADIOLIB_ERR_NONE) {
    Serial.println(F("[sx1276] error selecting bit rate"));
    return;
  } else if (rl_state == RADIOLIB_ERR_INVALID_FREQUENCY_DEVIATION) {
    Serial.println(F("[sx1276] Selected frequency deviation is invalid for this module!"));
    return;
  }
#endif

    size_t pkt_size = rf_protocol->payload_offset + rf_protocol->payload_size +
                      rf_protocol->crc_size;

    switch (rf_protocol->whitening)
    {
    case RF_WHITENING_MANCHESTER:
      if (use_hardware_manchester) {
          //rl_state = radio_sx1276->setEncoding(RADIOLIB_ENCODING_MANCHESTER);
          // maybe it needs to be RADIOLIB_ENCODING_MANCHESTER_INV
          //if (rf_protocol->payload_type == RF_PAYLOAD_INVERTED)
          //    rl_state = radio_sx1276->setEncoding(RADIOLIB_ENCODING_MANCHESTER_INV);
          //else
              rl_state = radio_sx1276->setEncoding(RADIOLIB_ENCODING_MANCHESTER);
          // or maybe it is the PAYLOAD that needs to be inverted?
      } else {
          rl_state = radio_sx1276->setEncoding(RADIOLIB_ENCODING_NRZ);
          pkt_size += pkt_size;
      }
      break;
    //case RF_WHITENING_PN9:
    //case RF_WHITENING_NONE:
    //case RF_WHITENING_NICERF:
    default:
      rl_state = radio_sx1276->setEncoding(RADIOLIB_ENCODING_NRZ);
      break;
    }

    rl_state = radio_sx1276->fixedPacketLengthMode(pkt_size);

    uint8_t *syncword = (uint8_t *) rf_protocol->syncword;
    uint8_t syncword_size = rf_protocol->syncword_size;
    if (tx == false) {   // for rx skip some leading sync bytes
        syncword += rf_protocol->syncword_skip;
        syncword_size -= rf_protocol->syncword_skip;
    }

    /* >>> Work around premature P3I syncword detection */
    if (rf_protocol->type == RF_PROTOCOL_P3I && rf_protocol->syncword_size == 2) {
      uint8_t preamble = rf_protocol->preamble_type == RF_PREAMBLE_TYPE_AA ?
                         0xAA : 0x55;
      uint8_t sword[4] = { preamble,
                           preamble,
                           syncword[0],
                           syncword[1]
                         };
      rl_state = radio_sx1276->setSyncWord(sword, 4);
    } else {
      rl_state = radio_sx1276->setSyncWord((uint8_t *) syncword, (size_t) syncword_size);
      // - but is setSyncWord() available for sx1276?
    }
    break;
  }             // end switch (rf_protocol->modulation_type) 

  /* setRfSwitchTable(); */

  radio_sx1276->setCrcFiltering(false);    // CRC done in software

  radio_sx1276->setOutputPower(tx_power);

  radio_sx1276->setPacketReceivedAction(receive_handler);
}

static uint8_t sx1276_receive(uint8_t *packet)
{
  if (settings->power_save & POWER_SAVE_NORECEIVE) {
    return 0;
  }

  int rl_state;

  if (!receive_active) {
    sx1276_setup(curr_rx_protocol_ptr, false);
    receive_complete = false;
    rl_state = radio_sx1276->startReceive();
    if (rl_state == RADIOLIB_ERR_NONE) {
        receive_active = true;
//Serial.println("rx started");
    }
  }

  if (receive_complete == false)
      return 0;

  receive_complete = false;
  receive_active = false;
  receive_cb_count++;

  uint8_t length = radio_sx1276->getPacketLength();
  if (length == 0) {
      (void) radio_sx1276->finishReceive();
      return 0;
  }

  if (length > RADIOLIB_MAX_DATA_LENGTH)
      length = RADIOLIB_MAX_DATA_LENGTH;
  rl_state = radio_sx1276->readData(packet, length);
  (void) radio_sx1276->finishReceive();
  if (rl_state != RADIOLIB_ERR_NONE)
      return 0;
  RF_last_rssi = radio_sx1276->getRSSI(true);
  return length;
}

static int16_t sx1276_transmit(uint8_t *packet, uint8_t length)
{
  if (receive_active) {
      (void) radio_sx1276->finishReceive();
      (void) radio_sx1276->standby();
      receive_active = false;
  }

  sx1276_setup(curr_tx_protocol_ptr, true);

  int rl_state = radio_sx1276->transmit(packet, (size_t) length);

  return (rl_state);
}


static void sx1276_shutdown()
{
  if (radio_sx1276)
    (void) radio_sx1276->sleep();
  RadioSPI.end();
}

/******************************************************/

bool sx1262_probe()
{
#if 0
  switch (hw_info.model)
  {
  case SOFTRF_MODEL_BADGE:
  case SOFTRF_MODEL_PRIME_MK2:
  default:
    Vtcxo = 1.6;
    break;
  }
#endif
  uint32_t irq  = (lmic_pins.dio[0] == LMIC_UNUSED_PIN) ? RADIOLIB_NC : lmic_pins.dio[0];
  uint32_t busy = (lmic_pins.busy   == LMIC_UNUSED_PIN) ? RADIOLIB_NC : lmic_pins.busy;
  mod = new Module(lmic_pins.nss, irq, lmic_pins.rst, busy, RadioSPI);
  radio_sx1262 = new SX1262(mod);
  int state = radio_sx1262->beginFSK();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("Found sx1262"));
  } else {
    Serial.print(F("did not find sx1262"));
    delete radio_sx1262;
    delete mod;
    return false;
  }
  return true;
}

static void sx1262_setfreq(uint32_t ifreq)
{
    if (receive_active) {
      (void) radio_sx1262->finishReceive();
      receive_active = false;
    }

    int rl_state = radio_sx1262->setFrequency(0.000001f * (float) ifreq);
#if RADIOLIB_DEBUG_BASIC
    if (rl_state == RADIOLIB_ERR_INVALID_FREQUENCY) {
      Serial.println(F("[sx1262] Selected frequency is invalid for this module!"));
      return;
    }
#endif
}

static void sx1262_setup(const rf_proto_desc_t *rf_protocol, bool tx)
{
  static const rf_proto_desc_t *prev_protocol = NULL;
  static bool prev_tx = false;

  int rl_state;

  if (rf_protocol == prev_protocol) {    // no need to re-setup everything
    if (tx == prev_tx)
        return;
    // only tx changed, only need to change sync word if FSK
    if (rf_protocol->modulation_type == RF_MODULATION_TYPE_2FSK) {
      uint8_t *syncword = (uint8_t *) rf_protocol->syncword;
      uint8_t syncword_size = rf_protocol->syncword_size;
      if (tx == false) {   // for rx skip some leading sync bytes
          syncword += rf_protocol->syncword_skip;
          syncword_size -= rf_protocol->syncword_skip;
      }
      rl_state = radio_sx1262->setSyncWord(syncword, (size_t) syncword_size);
    }
    prev_tx = tx;
    return;
  }
  
  prev_protocol = rf_protocol;
  prev_tx = tx;

  switch (rf_protocol->modulation_type)
  {
  case RF_MODULATION_TYPE_LORA:
#if 0
    rl_state = radio_sx1262->begin(434.0f, 125.0f, 9, 7, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 10, 8, Vtcxo, false);
#else
    rl_state = radio_sx1262->setModem(RADIOLIB_MODEM_LORA);
    delay(1);
    rl_state = radio_sx1262->setTCXO(Vtcxo);
#endif

    float br, fdev, bw;

    switch (RF_FreqPlan.Bandwidth)
    {
    case RF_RX_BANDWIDTH_SS_125KHZ:
      bw = 250.0f; /* BW_250 */
      break;
    case RF_RX_BANDWIDTH_SS_62KHZ:
      bw = 125.0f; /* BW_125 */
      break;
    case RF_RX_BANDWIDTH_SS_250KHZ:
      bw = 500.0f; /* BW_500 */
      break;
    //case RF_RX_BANDWIDTH_SS_125KHZ:
    default:
      bw = 250.0f; /* BW_250 */
      break;
    }

    rl_state = radio_sx1262->setBandwidth(bw);

#if RADIOLIB_DEBUG_BASIC
    if (rl_state == RADIOLIB_ERR_INVALID_BANDWIDTH) {
      Serial.println(F("[sx1262] Selected bandwidth is invalid for this module!"));
      return;
    }
#endif

    //switch (rf_protocol->type)
    {
    //case RF_PROTOCOL_FANET:
    //default:
      rl_state = radio_sx1262->setSpreadingFactor(7); /* SF_7 */
      rl_state = radio_sx1262->setCodingRate(5);      /* CR_5 */
      //break;
    }

    rl_state = radio_sx1262->setSyncWord((uint8_t) rf_protocol->syncword[0]);
    rl_state = radio_sx1262->setPreambleLength(8);
    rl_state = radio_sx1262->explicitHeader();
    rl_state = radio_sx1262->setCRC(0);

    break;

  case RF_MODULATION_TYPE_2FSK:
  case RF_MODULATION_TYPE_PPM: /* TBD */
  default:

#if 0
    rl_state = radio_sx1262->beginFSK(4.8f, 5.0f, 156.2f, 16, Vtcxo);
#else
    rl_state = radio_sx1262->setModem(RADIOLIB_MODEM_FSK);
    delay(1);
    rl_state = radio_sx1262->setTCXO(Vtcxo);
#endif

    switch (rf_protocol->bitrate)
    {
    case RF_BITRATE_38400:
      br = 38.4f;
      break;
    //case RF_BITRATE_100KBPS:
    default:
      br = 100.0f;
      break;
    }
    rl_state = radio_sx1262->setBitRate(br);

#if RADIOLIB_DEBUG_BASIC
  if (rl_state == RADIOLIB_ERR_INVALID_BIT_RATE) {
    Serial.println(F("[sx1262] Selected bit rate is invalid for this module!"));
    return;
  } else if (rl_state != RADIOLIB_ERR_NONE) {
    Serial.println(F("[sx1262] error selecting bit rate"));
    return;
  }
#endif

    switch (rf_protocol->deviation)
    {
    case RF_FREQUENCY_DEVIATION_12_5KHZ:
      fdev = 12.5f;
      break;
    case RF_FREQUENCY_DEVIATION_25KHZ:
      fdev = 25.0f;
      break;
    //case RF_FREQUENCY_DEVIATION_50KHZ:
    default:
      fdev = 50.0f;
      break;
    }
    rl_state = radio_sx1262->setFrequencyDeviation(fdev);

#if RADIOLIB_DEBUG_BASIC
  if (rl_state == RADIOLIB_ERR_INVALID_FREQUENCY_DEVIATION) {
    Serial.println(F("[sx1262] Selected frequency deviation is invalid for this module!"));
    return;
  }
#endif

    switch (rf_protocol->bandwidth)
    {
    case RF_RX_BANDWIDTH_SS_125KHZ:
      bw = 234.3f;
      break;
    case RF_RX_BANDWIDTH_SS_250KHZ:
    case RF_RX_BANDWIDTH_SS_200KHZ:
      bw = 467.0f;
      break;
    case RF_RX_BANDWIDTH_SS_50KHZ:
      bw = 117.3f;
      break;
    case RF_RX_BANDWIDTH_SS_62KHZ:
      bw = 156.2f;
      break;
    case RF_RX_BANDWIDTH_SS_166KHZ:
      bw = 312.0f;
      break;
    //case RF_RX_BANDWIDTH_SS_100KHZ:
    default:
      bw = 234.3f;
      break;
    }
    rl_state = radio_sx1262->setRxBandwidth(bw);

    rl_state = radio_sx1262->setPreambleLength(rf_protocol->preamble_size * 8);
    rl_state = radio_sx1262->setDataShaping(RADIOLIB_SHAPING_0_5);

#if 0
    switch (rf_protocol->crc_type)
    {
    case RF_CHECKSUM_TYPE_CCITT_FFFF:
    case RF_CHECKSUM_TYPE_CCITT_0000:
    case RF_CHECKSUM_TYPE_CCITT_1D02:
    case RF_CHECKSUM_TYPE_CRC8_107:
    case RF_CHECKSUM_TYPE_RS:
      /* CRC is driven by software */
      rl_state = radio_sx1262->setCRC(0);
      break;
    case RF_CHECKSUM_TYPE_GALLAGER:
    case RF_CHECKSUM_TYPE_CRC_MODES:
    case RF_CHECKSUM_TYPE_NONE:
    default:
      rl_state = radio_sx1262->setCRC(0);
      break;
    }
#else
    rl_state = radio_sx1262->setCRC(0);
#endif

    size_t pkt_size = rf_protocol->payload_offset + rf_protocol->payload_size +
                      rf_protocol->crc_size;

    switch (rf_protocol->whitening)
    {
    case RF_WHITENING_MANCHESTER:
      pkt_size += pkt_size;
      break;
    //case RF_WHITENING_PN9:
    //case RF_WHITENING_NONE:
    //case RF_WHITENING_NICERF:
    default:
      break;
    }

    rl_state = radio_sx1262->setWhitening(false);

    rl_state = radio_sx1262->fixedPacketLengthMode(pkt_size);

    uint8_t *syncword = (uint8_t *) rf_protocol->syncword;
    uint8_t syncword_size = rf_protocol->syncword_size;
    if (tx == false) {   // for rx skip some leading sync bytes
        syncword += rf_protocol->syncword_skip;
        syncword_size -= rf_protocol->syncword_skip;
    }

    /* >>> Work around premature P3I syncword detection */
    if (rf_protocol->type == RF_PROTOCOL_P3I && rf_protocol->syncword_size == 2) {
      uint8_t preamble = rf_protocol->preamble_type == RF_PREAMBLE_TYPE_AA ?
                         0xAA : 0x55;
      uint8_t sword[4] = { preamble,
                           preamble,
                           syncword[0],
                           syncword[1]
                         };
      rl_state = radio_sx1262->setSyncWord(sword, 4);
    } else {
      rl_state = radio_sx1262->setSyncWord((uint8_t *) syncword, (size_t) syncword_size);
    }
    break;
  }             // end switch (rf_protocol->modulation_type) 

  /* setRfSwitchTable(); */

  radio_sx1262->setOutputPower(tx_power);

  rl_state = radio_sx1262->setRxBoostedGainMode(true);

  radio_sx1262->setPacketReceivedAction(receive_handler);
}

static uint8_t sx1262_receive(uint8_t *packet)
{
  if (settings->power_save & POWER_SAVE_NORECEIVE) {
    return 0;
  }

  int rl_state;

  if (!receive_active) {
    sx1262_setup(curr_rx_protocol_ptr, false);
    receive_complete = false;
    rl_state = radio_sx1262->startReceive();
    if (rl_state == RADIOLIB_ERR_NONE)
        receive_active = true;
  }

  if (receive_complete == false)
      return 0;

  receive_complete = false;
  receive_active = false;
  receive_cb_count++;

  uint8_t length = radio_sx1262->getPacketLength();
  if (length == 0) {
      (void) radio_sx1262->finishReceive();
      return 0;
  }

  if (length > RADIOLIB_MAX_DATA_LENGTH)
      length = RADIOLIB_MAX_DATA_LENGTH;
  rl_state = radio_sx1262->readData(packet, length);
  (void) radio_sx1262->finishReceive();
  if (rl_state != RADIOLIB_ERR_NONE)
      return 0;
  RF_last_rssi = radio_sx1262->getRSSI(true);
  return length;
}

static int16_t sx1262_transmit(uint8_t *packet, uint8_t length)
{
  sx1262_setup(curr_tx_protocol_ptr, true);

  int rl_state = radio_sx1262->transmit(packet, (size_t) length);

  return (rl_state);
}

static void sx1262_shutdown()
{
  if (radio_sx1262)
    (void) radio_sx1262->sleep(false);
  RadioSPI.end();
}

#endif /* EXCLUDE_SX12XX */

/******************************************************/

#if defined(INCLUDE_LR11X0)

LR11x0  *radio_lr11xx;
//LR1121  *radio_lr1121;

static bool lr1110_probe(void);
static bool lr1121_probe(void);
static void lr11xx_setup(const rf_proto_desc_t*, bool);
static void lr11xx_setfreq(uint32_t);
//static void lr1121_setfreq(uint32_t);
static uint8_t lr11xx_receive(uint8_t *packet)
static int16_t lr11xx_transmit(uint8_t *packet, uint8_t length)
static void lr11xx_shutdown(void);
//static void lr1121_shutdown(void);

const rfchip_ops_t lr1110_ops = {
  RF_IC_LR11XX,
  "LR11XX",
  //lr11xx_probe,
  //lr11xx_setup,
  lr11xx_setfreq,
  lr11xx_receive,
  lr11xx_transmit,
  lr11xx_shutdown
};

/*
const rfchip_ops_t lr1121_ops = {
  RF_IC_LR1121,
  "LR1121",
  //lr1121_probe,
  //lr1121_setup,
  lr1121_setfreq,
  lr11xx_receive,
  lr11xx_transmit,
  lr1121_shutdown
};
*/

#if 0
#if RADIOLIB_VERSION_MAJOR >= 7 && RADIOLIB_VERSION_MINOR > 1
static uint8_t lr11xx_roundRampTime(uint32_t rampTimeUs) {

  //static uint32_t prev_rampTime = 0xFFFFFFFF;
  //static uint8_t prev_rounded = RADIOLIB_LRXXXX_PA_RAMP_2U;
  //if (rampTimeUs == prev_rampTime)
  //    return prev_rounded;

  uint8_t regVal;

  // Round up the ramp time to nearest discrete register value
  if(rampTimeUs == 48) {          // the only value actually used
    regVal = RADIOLIB_LRXXXX_PA_RAMP_48U;
  } else if(rampTimeUs <= 2) {
    regVal = RADIOLIB_LRXXXX_PA_RAMP_2U;
  } else if(rampTimeUs <= 4) {
    regVal = RADIOLIB_LRXXXX_PA_RAMP_4U;
  } else if(rampTimeUs <= 8) {
    regVal = RADIOLIB_LRXXXX_PA_RAMP_8U;
  } else if(rampTimeUs <= 16) {
    regVal = RADIOLIB_LRXXXX_PA_RAMP_16U;
  } else if(rampTimeUs <= 32) {
    regVal = RADIOLIB_LRXXXX_PA_RAMP_32U;
  } else if(rampTimeUs <= 48) {
    regVal = RADIOLIB_LRXXXX_PA_RAMP_48U;
  } else if(rampTimeUs <= 64) {
    regVal = RADIOLIB_LRXXXX_PA_RAMP_64U;
  } else if(rampTimeUs <= 80) {
    regVal = RADIOLIB_LRXXXX_PA_RAMP_80U;
  } else if(rampTimeUs <= 96) {
    regVal = RADIOLIB_LRXXXX_PA_RAMP_96U;
  } else if(rampTimeUs <= 112) {
    regVal = RADIOLIB_LRXXXX_PA_RAMP_112U;
  } else if(rampTimeUs <= 128) {
    regVal = RADIOLIB_LRXXXX_PA_RAMP_128U;
  } else if(rampTimeUs <= 144) {
    regVal = RADIOLIB_LRXXXX_PA_RAMP_144U;
  } else if(rampTimeUs <= 160) {
    regVal = RADIOLIB_LRXXXX_PA_RAMP_160U;
  } else if(rampTimeUs <= 176) {
    regVal = RADIOLIB_LRXXXX_PA_RAMP_176U;
  } else if(rampTimeUs <= 192) {
    regVal = RADIOLIB_LRXXXX_PA_RAMP_192U;
  } else if(rampTimeUs <= 208) {
    regVal = RADIOLIB_LRXXXX_PA_RAMP_208U;
  } else if(rampTimeUs <= 240) {
    regVal = RADIOLIB_LRXXXX_PA_RAMP_240U;
  } else if(rampTimeUs <= 272) {
    regVal = RADIOLIB_LRXXXX_PA_RAMP_272U;
  } else {  // 304
    regVal = RADIOLIB_LRXXXX_PA_RAMP_304U;
  }

  //prev_rampTime = rampTimeUs;
  //prev_rounded = regVal;

  return regVal;
}
#endif /* RADIOLIB_VERSION */
#endif

static uint64_t lr11xx_eui_be = 0xdeadbeefdeadbeef;

static const uint32_t rfswitch_dio_pins_hpdtek[] = {
    RADIOLIB_LR11X0_DIO5, RADIOLIB_LR11X0_DIO6,
    RADIOLIB_NC, RADIOLIB_NC, RADIOLIB_NC
};

static const Module::RfSwitchMode_t rfswitch_table_hpdtek[] = {
    // mode                  DIO5  DIO6
    { LR11x0::MODE_STBY,   { LOW,  LOW  } },
    { LR11x0::MODE_RX,     { HIGH, LOW  } },
    { LR11x0::MODE_TX,     { LOW,  HIGH } },
    { LR11x0::MODE_TX_HP,  { LOW,  HIGH } },
    { LR11x0::MODE_TX_HF,  { LOW,  LOW  } },
    { LR11x0::MODE_GNSS,   { LOW,  LOW  } },
    { LR11x0::MODE_WIFI,   { LOW,  LOW  } },
    END_OF_MODE_TABLE,
};

/* setDioAsRfSwitch(0x0f, 0x0, 0x09, 0x0B, 0x0A, 0x0, 0x4, 0x0) */
static const uint32_t rfswitch_dio_pins_seeed[] = {
    RADIOLIB_LR11X0_DIO5, RADIOLIB_LR11X0_DIO6,
    RADIOLIB_LR11X0_DIO7, RADIOLIB_LR11X0_DIO8,
    RADIOLIB_NC
};

static const Module::RfSwitchMode_t rfswitch_table_seeed[] = {
    // mode                  DIO5  DIO6  DIO7  DIO8
    { LR11x0::MODE_STBY,   { LOW,  LOW,  LOW,  LOW  } },
    // SKY13373
    { LR11x0::MODE_RX,     { HIGH, LOW,  LOW,  HIGH } },
    { LR11x0::MODE_TX,     { HIGH, HIGH, LOW,  HIGH } },
    { LR11x0::MODE_TX_HP,  { LOW,  HIGH, LOW,  HIGH } },
    { LR11x0::MODE_TX_HF,  { LOW,  LOW,  LOW,  LOW  } },
    // BGA524N6
    { LR11x0::MODE_GNSS,   { LOW,  LOW,  HIGH, LOW  } },
    // LC
    { LR11x0::MODE_WIFI,   { LOW,  LOW,  LOW,  LOW  } },
    END_OF_MODE_TABLE,
};

/* setDioAsRfSwitch(0x07, 0x0, 0x02, 0x03, 0x01, 0x0, 0x4, 0x0) */
static const uint32_t rfswitch_dio_pins_ebyte[] = {
    RADIOLIB_LR11X0_DIO5, RADIOLIB_LR11X0_DIO6, RADIOLIB_LR11X0_DIO7,
    RADIOLIB_NC, RADIOLIB_NC
};

static const Module::RfSwitchMode_t rfswitch_table_ebyte[] = {
    // mode                  DIO5  DIO6  DIO7
    { LR11x0::MODE_STBY,   { LOW,  LOW,  LOW  } },
    { LR11x0::MODE_RX,     { LOW,  HIGH, LOW  } },
    { LR11x0::MODE_TX,     { HIGH, HIGH, LOW  } },
    { LR11x0::MODE_TX_HP,  { HIGH, LOW,  LOW  } },
    { LR11x0::MODE_TX_HF,  { LOW,  LOW,  LOW  } },
    { LR11x0::MODE_GNSS,   { LOW,  LOW,  HIGH } },
    { LR11x0::MODE_WIFI,   { LOW,  LOW,  LOW  } },
    END_OF_MODE_TABLE,
};

/* setDioAsRfSwitch(0x0F, 0x0, 0x0C, 0x08, 0x08, 0x6, 0x0, 0x5) */
static const uint32_t rfswitch_dio_pins_radiomaster[] = {
    RADIOLIB_LR11X0_DIO5, RADIOLIB_LR11X0_DIO6,
    RADIOLIB_LR11X0_DIO7, RADIOLIB_LR11X0_DIO8,
    RADIOLIB_NC
};

static const Module::RfSwitchMode_t rfswitch_table_radiomaster[] = {
    // mode                  DIO5  DIO6  DIO7  DIO8
    { LR11x0::MODE_STBY,   { LOW,  LOW,  LOW,  LOW  } },
    // SKY13373 ( V1-DIO7 V2-DIO8 )
    { LR11x0::MODE_RX,     { LOW,  LOW,  HIGH, HIGH } },
    { LR11x0::MODE_TX,     { LOW,  LOW,  LOW,  HIGH } },
    { LR11x0::MODE_TX_HP,  { LOW,  LOW,  LOW,  HIGH } },
    // AT2401C ( RXEN-DIO5 TXEN-DIO6 )
    { LR11x0::MODE_TX_HF,  { LOW,  HIGH, HIGH, LOW  } },
    { LR11x0::MODE_GNSS,   { LOW,  LOW,  LOW,  LOW  } },
    { LR11x0::MODE_WIFI,   { HIGH, LOW,  HIGH, LOW  } },
    END_OF_MODE_TABLE,
};

static const uint32_t rfswitch_dio_pins_elecrow[] = {
    RADIOLIB_LR11X0_DIO5, RADIOLIB_LR11X0_DIO6,
    RADIOLIB_NC, RADIOLIB_NC, RADIOLIB_NC
};

static const Module::RfSwitchMode_t rfswitch_table_elecrow[] = {
    // mode                  DIO5  DIO6
    { LR11x0::MODE_STBY,   { LOW,  LOW  } },
    { LR11x0::MODE_RX,     { HIGH, LOW  } },
    { LR11x0::MODE_TX,     { HIGH, HIGH } },
    { LR11x0::MODE_TX_HP,  { LOW,  HIGH } },
    { LR11x0::MODE_TX_HF,  { LOW,  LOW  } },
    { LR11x0::MODE_GNSS,   { LOW,  LOW  } },
    { LR11x0::MODE_WIFI,   { LOW,  LOW  } },
    END_OF_MODE_TABLE,
};

static const uint32_t rfswitch_dio_pins_lilygo[] = {
    RADIOLIB_LR11X0_DIO5, RADIOLIB_LR11X0_DIO6,
    10 /* SOC_GPIO_PIN_ELRS_HF_RX */, 14 /* SOC_GPIO_PIN_ELRS_HF_TX */,
    RADIOLIB_NC
};

static const Module::RfSwitchMode_t rfswitch_table_lilygo[] = {
    // mode                  DIO5  DIO6  HF_RX HF_TX
    { LR11x0::MODE_STBY,   { LOW,  LOW,  LOW,  LOW  } },
    { LR11x0::MODE_RX,     { LOW,  HIGH, HIGH, LOW  } },
    { LR11x0::MODE_TX,     { HIGH, LOW,  LOW,  LOW  } },
    { LR11x0::MODE_TX_HP,  { HIGH, LOW,  LOW,  LOW  } },
    { LR11x0::MODE_TX_HF,  { LOW,  LOW,  LOW,  HIGH } },
    { LR11x0::MODE_GNSS,   { LOW,  LOW,  LOW,  LOW  } },
    { LR11x0::MODE_WIFI,   { LOW,  LOW,  LOW,  LOW  } },
    END_OF_MODE_TABLE,
};

bool lr1110_probe()
{
  switch (hw_info.model)
  {
  case SOFTRF_MODEL_BADGE:
  case SOFTRF_MODEL_PRIME_MK3:
    // HPDTeK HPD-16E
    // LR1121 TCXO Voltage 2.85~3.15V
    Vtcxo = 3.0;
    break;

  case SOFTRF_MODEL_POCKET:
    Vtcxo = 3.3;
    break;

  case SOFTRF_MODEL_CARD:
    // Seeed T1000E
  default:
    Vtcxo = 1.6;
    break;
  }
  uint32_t irq  = (lmic_pins.dio[0] == LMIC_UNUSED_PIN) ? RADIOLIB_NC : lmic_pins.dio[0];
  uint32_t busy = (lmic_pins.busy   == LMIC_UNUSED_PIN) ? RADIOLIB_NC : lmic_pins.busy;
  mod = new Module(lmic_pins.nss, irq, lmic_pins.rst, busy, RadioSPI);
  radio_lr11xx = new LR1110(mod);
  int state = radio_lr11xx->beginGFSK(4.8, 5.0, 156.2, 16, Vtcxo);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("Found LR1110"));
  } else {
    Serial.print(F("did not find LR1110"));
    delete radio_lr11xx;
    delete mod;
    return false;
  }
  return true;
}

bool lr1121_probe()
{
  uint32_t irq  = (lmic_pins.dio[0] == LMIC_UNUSED_PIN) ? RADIOLIB_NC : lmic_pins.dio[0];
  uint32_t busy = (lmic_pins.busy   == LMIC_UNUSED_PIN) ? RADIOLIB_NC : lmic_pins.busy;
  mod = new Module(lmic_pins.nss, irq, lmic_pins.rst, busy, RadioSPI);
  radio_lr11xx = new LR1121(mod);
  switch (hw_info.model)
  {
  case SOFTRF_MODEL_ACADEMY:
    radio_lr11xx->irqDioNum = 8; /* DIO8 as IRQ on WIO-2021 */
    Vtcxo = 0.0; /* XTAL */
    break;
  default:
    radio_lr11xx->irqDioNum = 11; /* LR2021 DIO11 as IRQ */
    Vtcxo = 1.6;
    break;
  }
  int state = radio_lr11xx->beginGFSK(4.8, 5.0, 156.2, 16, Vtcxo);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("Found LR1121"));
  } else {
    Serial.print(F("did not find LR1121"));
    delete radio_lr11xx;
    delete mod;
    return false;
  }
  return true;
}

static void lr11xx_setfreq(uint32_t ifreq)
{
    if (receive_active) {
      (void) radio_lr11xx->finishReceive();
      receive_active = false;
    }

    int rl_state = radio_lr11xx->setFrequency(0.000001f * (float) ifreq);
#if RADIOLIB_DEBUG_BASIC
    if (rl_state == RADIOLIB_ERR_INVALID_FREQUENCY) {
      Serial.println(F("[LR11XX] Selected frequency is invalid for this module!"));
      return;
    }
#endif
}

/*
static void lr1121_setfreq(uint32_t ifreq)
{
    radio_lr1121->setFrequency(0.000001f * (float) ifreq);
}
*/

static void lr11xx_setup(const rf_proto_desc_t *rf_protocol, bool tx)
{
  static const rf_proto_desc_t *prev_protocol = NULL;
  static bool prev_tx = false;

  int rl_state;

  if (rf_protocol == prev_protocol) {    // no need to re-setup everything
    if (tx == prev_tx)
        return;
    // only tx changed, only need to change sync word if FSK
    if (rf_protocol->modulation_type == RF_MODULATION_TYPE_2FSK) {
      uint8_t *syncword = (uint8_t *) rf_protocol->syncword;
      uint8_t syncword_size = rf_protocol->syncword_size;
      if (tx == false) {   // for rx skip some leading sync bytes
          syncword += rf_protocol->syncword_skip;
          syncword_size -= rf_protocol->syncword_skip;
      }
      rl_state = radio_lr11xx->setSyncWord(syncword, (size_t) syncword_size);
    }
    prev_tx = tx;
    return;
  }
  
  prev_protocol = rf_protocol;
  prev_tx = tx;

  //uint64_t eui_le = __builtin_bswap64(lr11xx_eui_be);

  /*
   *  Product/Module  |   IC   |       EUI          | Use case
   *  ----------------+--------+--------------------+-----------
   *  HPDTeK HPD-16E  | LR1121 | 0x0016c001f0182465 | Standalone
   *  HPDTeK HPD-16E  | LR1121 | 0x0016c001f01824af | Badge
   *  HPDTeK HPD-16E  | LR1121 | 0x0016c001f018276a | Prime Mk3
   *  Seeed T1000-E   | LR1110 | 0x0016c001f03a86ab | Card
   *  Ebyte E80       | LR1121 | 0x0016c001f047ac30 | Academy
   *  RadioMaster XR1 | LR1121 | 0x0016c001f09aa1b7 | Nano
   *  Elecrow TN-M3   | LR1110 | 0x0016c001f00f0b12 | Pocket
   */

  uint32_t frequency = RF_FreqPlan.getChanFrequency(0);
  bool high = (frequency > 1000000000) ; /* above 1GHz */

  switch (rf_protocol->modulation_type)
  {
  case RF_MODULATION_TYPE_LORA:
#if 0
    rl_state = radio_lr11xx->begin(125.0f, 9, 7,
                                    RADIOLIB_LR11X0_LORA_SYNC_WORD_PRIVATE,
                                    8, Vtcxo);
#else
    rl_state = radio_lr11xx->setModem(RADIOLIB_MODEM_LORA);
    delay(1);
    rl_state = radio_lr11xx->setTCXO(Vtcxo);
#endif

    float br, fdev, bw;

    switch (RF_FreqPlan.Bandwidth)
    {
    case RF_RX_BANDWIDTH_SS_125KHZ:
      bw = high ? 406.25f  : 250.0f; /* BW_250 */
      break;
    case RF_RX_BANDWIDTH_SS_62KHZ:
      bw = high ? 203.125f : 125.0f; /* BW_125 */
      break;
    case RF_RX_BANDWIDTH_SS_250KHZ:
      bw = high ? 812.5f   : 500.0f; /* BW_500 */
      break;
    //case RF_RX_BANDWIDTH_SS_125KHZ:
    default:
      bw = high ? 406.25f  : 250.0f; /* BW_250 */
      break;
    }

    rl_state = radio_lr11xx->setBandwidth(bw, high);

#if RADIOLIB_DEBUG_BASIC
    if (rl_state == RADIOLIB_ERR_INVALID_BANDWIDTH) {
      Serial.println(F("[LR11XX] Selected bandwidth is invalid for this module!"));
      return;
    }
#endif

    //switch (rf_protocol->type)
    {
    //case RF_PROTOCOL_FANET:
    //default:
      rl_state = radio_lr11xx->setSpreadingFactor(7); /* SF_7 */
      rl_state = radio_lr11xx->setCodingRate(5);      /* CR_5 */
      //break;
    }

    rl_state = radio_lr11xx->setSyncWord((uint8_t) rf_protocol->syncword[0]);
    rl_state = radio_lr11xx->setPreambleLength(8);
    rl_state = radio_lr11xx->explicitHeader();
    rl_state = radio_lr11xx->setCRC(0);

    break;

  case RF_MODULATION_TYPE_2FSK:
  case RF_MODULATION_TYPE_PPM: /* TBD */
  default:

#if 0
    rl_state = radio_lr11xx->beginGFSK(4.8f, 5.0f, 156.2f, 16, Vtcxo);
#else
    rl_state = radio_lr11xx->setModem(RADIOLIB_MODEM_FSK);
    delay(1);
    rl_state = radio_lr11xx->setTCXO(Vtcxo);
#endif

    switch (rf_protocol->bitrate)
    {
    case RF_BITRATE_38400:
      br = high ? 125.0f :  38.4f;
      break;
    //case RF_BITRATE_100KBPS:
    default:
      br = high ? 125.0f : 100.0f;
      break;
    }
    rl_state = radio_lr11xx->setBitRate(br);

#if RADIOLIB_DEBUG_BASIC
  if (rl_state == RADIOLIB_ERR_INVALID_BIT_RATE) {
    Serial.println(F("[LR11XX] Selected bit rate is invalid for this module!"));
    return;
  } else if (rl_state == RADIOLIB_ERR_INVALID_BIT_RATE_BW_RATIO) {
    Serial.println(F("[LR11XX] Selected bit rate to bandwidth ratio is invalid!"));
    Serial.println(F("[LR11XX] Increase receiver bandwidth to set this bit rate."));
    return;
  }
#endif

    switch (rf_protocol->deviation)
    {
    //case RF_FREQUENCY_DEVIATION_10KHZ:
    //  fdev = high ? 62.5f : 10.0f;
    //  break;
    case RF_FREQUENCY_DEVIATION_12_5KHZ:
      fdev = 12.5f;
      break;
    case RF_FREQUENCY_DEVIATION_25KHZ:
      fdev = 25.0f;
      break;
    //case RF_FREQUENCY_DEVIATION_50KHZ:
    default:
      fdev = high ? 62.5f : 50.0f;
      break;
    }
    rl_state = radio_lr11xx->setFrequencyDeviation(fdev);

#if RADIOLIB_DEBUG_BASIC
  if (rl_state == RADIOLIB_ERR_INVALID_FREQUENCY_DEVIATION) {
    Serial.println(F("[LR11XX] Selected frequency deviation is invalid for this module!"));
    return;
  }
#endif

    switch (rf_protocol->bandwidth)
    {
    case RF_RX_BANDWIDTH_SS_125KHZ:
      bw = 234.3f;
      break;
    case RF_RX_BANDWIDTH_SS_50KHZ:
      bw = 117.3f;
      break;
    case RF_RX_BANDWIDTH_SS_62KHZ:
      bw = 156.2f;
      break;
    case RF_RX_BANDWIDTH_SS_100KHZ:
      bw = 234.3f;
      break;
    case RF_RX_BANDWIDTH_SS_166KHZ:
      bw = 312.0f;
      break;
    case RF_RX_BANDWIDTH_SS_200KHZ:
    case RF_RX_BANDWIDTH_SS_250KHZ:  /* TBD */
    //case RF_RX_BANDWIDTH_SS_1567KHZ: /* TBD */
      bw = 467.0f;
      break;
    //case RF_RX_BANDWIDTH_SS_125KHZ:
    default:
      bw = 234.3f;
      break;
    }
    rl_state = radio_lr11xx->setRxBandwidth(bw);

    rl_state = radio_lr11xx->setPreambleLength(rf_protocol->preamble_size * 8);
    rl_state = radio_lr11xx->setDataShaping(RADIOLIB_SHAPING_0_5);

#if 0
    switch (rf_protocol->crc_type)
    {
    case RF_CHECKSUM_TYPE_CCITT_FFFF:
    case RF_CHECKSUM_TYPE_CCITT_0000:
    case RF_CHECKSUM_TYPE_CCITT_1D02:
    case RF_CHECKSUM_TYPE_CRC8_107:
    case RF_CHECKSUM_TYPE_RS:
      /* CRC is driven by software */
      rl_state = radio_lr11xx->setCRC(0);
      break;
    case RF_CHECKSUM_TYPE_GALLAGER:
    case RF_CHECKSUM_TYPE_CRC_MODES:
    case RF_CHECKSUM_TYPE_NONE:
    default:
      rl_state = radio_lr11xx->setCRC(0);
      break;
    }
#else
    rl_state = radio_lr11xx->setCRC(0);
#endif

    size_t pkt_size = rf_protocol->payload_offset + rf_protocol->payload_size +
                      rf_protocol->crc_size;

    switch (rf_protocol->whitening)
    {
    case RF_WHITENING_MANCHESTER:
      pkt_size += pkt_size;
      break;
    //case RF_WHITENING_PN9:
    //case RF_WHITENING_NONE:
    //case RF_WHITENING_NICERF:
    default:
      break;
    }

    rl_state = radio_lr11xx->setWhitening(false, 0x0001 /* default SX128x value */);

    rl_state = radio_lr11xx->fixedPacketLengthMode(pkt_size);

    rl_state = radio_lr11xx->disableAddressFiltering();

    uint8_t *syncword = (uint8_t *) rf_protocol->syncword;
    uint8_t syncword_size = rf_protocol->syncword_size;
    if (tx == false) {   // for rx skip some leading sync bytes
        syncword += rf_protocol->syncword_skip;
        syncword_size -= rf_protocol->syncword_skip;
    }

    /* Work around premature P3I syncword detection */
    if (rf_protocol->type == RF_PROTOCOL_P3I && rf_protocol->syncword_size == 2) {
      uint8_t preamble = rf_protocol->preamble_type == RF_PREAMBLE_TYPE_AA ?
                         0xAA : 0x55;
      uint8_t sword[4] = { preamble,
                           preamble,
                           syncword[0],
                           syncword[1]
                         };
      rl_state = radio_lr11xx->setSyncWord(sword, 4);
    } else {
      rl_state = radio_lr11xx->setSyncWord((uint8_t *) syncword, (size_t) syncword_size);
    }
    break;
  }

  switch (hw_info.model)
  {
  case SOFTRF_MODEL_CARD:
    radio_lr11xx->setRfSwitchTable(rfswitch_dio_pins_seeed, rfswitch_table_seeed);
    {
      bool useHp = (tx_power > 14);
      //rl_state = radio_lr11xx->setOutputPower(tx_power, useHp, useHp, 0x04, 0x07, lr11xx_roundRampTime(48) - 0x03);
      rl_state = radio_lr11xx->setOutputPower(tx_power, useHp, useHp, 0x04, 0x07, RADIOLIB_LRXXXX_PA_RAMP_48U - 0x03);
    }
    break;

  case SOFTRF_MODEL_POCKET:
    radio_lr11xx->setRfSwitchTable(rfswitch_dio_pins_elecrow, rfswitch_table_elecrow);
    {
      bool useHp = (tx_power > 14);
      //rl_state = radio_lr11xx->setOutputPower(tx_power, useHp, useHp, 0x04, 0x07, lr11xx_roundRampTime(48) - 0x03);
      rl_state = radio_lr11xx->setOutputPower(tx_power, useHp, useHp, 0x04, 0x07, RADIOLIB_LRXXXX_PA_RAMP_48U - 0x03);
    }
    break;

  //case SOFTRF_MODEL_BADGE:
  //case SOFTRF_MODEL_PRIME_MK3:
  default:
    radio_lr11xx->setRfSwitchTable(rfswitch_dio_pins_hpdtek, rfswitch_table_hpdtek);
    {
      uint8_t paSel = 0;
      uint8_t paSupply = 0;
      if (high) {
        paSel = 2;
      } else if (true || (tx_power > 14)) {
        paSel = 1;
        paSupply = 1;
      }
      //rl_state = radio_lr11xx->setOutputPower(tx_power, paSel, paSupply, 0x04, 0x07, lr11xx_roundRampTime(48) - 0x03);
      rl_state = radio_lr11xx->setOutputPower(tx_power, paSel, paSupply, 0x04, 0x07, RADIOLIB_LRXXXX_PA_RAMP_48U - 0x03);
    }
    break;
  }
//#endif

  rl_state = radio_lr11xx->setRxBoostedGainMode(true);

  radio_lr11xx->setPacketReceivedAction(receive_handler);
}

static uint8_t lr11xx_receive(uint8_t *packet)
{
  if (settings->power_save & POWER_SAVE_NORECEIVE) {
    return 0;
  }

  int rl_state;

  if (!receive_active) {
    lr11xx_setup(curr_rx_protocol_ptr, false);
    receive_complete = false;
    rl_state = radio_lr11xx->startReceive();
    if (rl_state == RADIOLIB_ERR_NONE)
        receive_active = true;
  }

  if (receive_complete == false)
      return 0;

  receive_complete = false;
  receive_active = false;
  receive_cb_count++;

  uint8_t length = radio_lr11xx->getPacketLength();
  if (length == 0) {
      (void) radio_lr11xx->finishReceive();
      return 0;
  }

  if (length > RADIOLIB_MAX_DATA_LENGTH)
      length = RADIOLIB_MAX_DATA_LENGTH;
  rl_state = radio_lr11xx->readData(packet, length);
  (void) radio_lr11xx->finishReceive();
  if (rl_state != RADIOLIB_ERR_NONE)
      return 0;
  RF_last_rssi = radio_lr11xx->getRSSI(true));
  return length;
}

static int16_t lr11xx_transmit(uint8_t *packet, uint8_t length)
{
  lr11xx_setup(curr_tx_protocol_ptr, true);

  int rl_state = radio_lr11xx->transmit(packet, (size_t) length);

  return (rl_state);
}

static void lr11xx_shutdown()
{
  int rl_state = radio_lr11xx->standby(RADIOLIB_LR11X0_STANDBY_RC);
  rl_state = radio_lr11xx->setTCXO(0);
  rl_state = radio_lr11xx->sleep(false, 0);
  RadioSPI.end();
}

/*
static void lr1121_shutdown()
{
  int rl_state = radio_lr1121->standby(RADIOLIB_LR11X0_STANDBY_RC);
  rl_state = radio_lr1121->setTCXO(0);
  rl_state = radio_lr1121->sleep(false, 0);
  RadioSPI.end();
}
*/

#endif /* LR11XX */

/******************************************************/

#if defined(INCLUDE_LR2021)

#include <fec.h>

/*
 * DF17 Rx optimization method:
 *
 * 0 - without optimization
 * 1 - DF17 sync word
 * 2 - DF17 OOK detection word
 * 3 - corrected version
 */
#define OPT_DF17 3

static bool lr2021_probe(void);
static void lr20xx_setup(const rf_proto_desc_t *, bool);
static void lr20xx_setfreq(uint32_t);
static uint8_t lr20xx_receive(uint8_t *packet)
static int16_t lr20xx_transmit(uint8_t *packet, uint8_t length)
static void lr20xx_shutdown(void);
const rfchip_ops_t lr2021_ops = {
  RF_IC_LR2021,
  "LR2021",
  //lr2021_probe,
  //lr20xx_setfreq,
  lr20xx_receive,
  lr20xx_transmit,
  lr20xx_shutdown
};

LR2021  *radio_lr20xx;

//static uint64_t lr20xx_eui_be = 0xdeadbeefdeadbeef;

mode_s_t rl_mode_s_state;

static const uint32_t rfswitch_dio_pins_MXD8721[] = {
    RADIOLIB_LR2021_DIO5, RADIOLIB_LR2021_DIO6,
    RADIOLIB_NC, RADIOLIB_NC, RADIOLIB_NC
};

static const Module::RfSwitchMode_t rfswitch_table_MXD8721[] = {
    // mode                  DIO5  DIO6
    { LR2021::MODE_STBY,   { LOW,  LOW  } },
    { LR2021::MODE_RX,     { LOW,  HIGH } },
    { LR2021::MODE_TX,     { LOW,  HIGH } },
    { LR2021::MODE_RX_HF,  { HIGH, LOW  } },
    { LR2021::MODE_TX_HF,  { HIGH, LOW  } },
    LR2021::MODE_END_OF_TABLE,
};

// this function is called when a complete packet
// is received by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif

static bool lr2021_probe()
{
  uint32_t irq  = (lmic_pins.dio[0] == LMIC_UNUSED_PIN) ? RADIOLIB_NC : lmic_pins.dio[0];
  uint32_t busy = (lmic_pins.busy   == LMIC_UNUSED_PIN) ? RADIOLIB_NC : lmic_pins.busy;
  mod = new Module(lmic_pins.nss, irq, lmic_pins.rst, busy, RadioSPI);
  radio_lr20xx = new LR2021(mod);
  int state = radio_lr20xx->beginGFSK(434.0f, 4.8f, 5.0f, 153.8f, 10, 16, Vtcxo);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("Found LR2021"));
  } else {
    Serial.print(F("did not find LR2021"));
    delete radio_lr20xx;
    delete mod;
    return false;
  }
  return true;
}

static void lr20xx_setfreq(uint32_t ifreq)
{
    if (receive_active) {
      (void) radio_lr20xx->finishReceive();
      receive_active = false;
    }

    int rl_state = radio_lr20xx->setFrequency(0.000001f * (float) ifreq);
#if RADIOLIB_DEBUG_BASIC
    if (rl_state == RADIOLIB_ERR_INVALID_FREQUENCY) {
      Serial.println(F("[LR20XX] Selected frequency is invalid for this module!"));
      return;
    }
#endif
}

static void lr20xx_setup(const rf_proto_desc_t *rf_protocol, bool tx)
{
  static const rf_proto_desc_t *prev_protocol = NULL;
  static bool prev_tx = false;

  int rl_state;

  if (rf_protocol == prev_protocol) {    // no need to re-setup everything
    if (tx == prev_tx)
        return;
    // only tx changed, only need to change sync word if FSK
    if (rf_protocol->modulation_type == RF_MODULATION_TYPE_2FSK) {
      uint8_t *syncword = (uint8_t *) rf_protocol->syncword;
      uint8_t syncword_size = rf_protocol->syncword_size;
      if (tx == false) {   // for rx skip some leading sync bytes
          syncword += rf_protocol->syncword_skip;
          syncword_size -= rf_protocol->syncword_skip;
      }
      rl_state = radio_lr20xx->setSyncWord(syncword, (size_t) syncword_size);
    }
    prev_tx = tx;
    return;
  }
  
  prev_protocol = rf_protocol;
  prev_tx = tx;

  uint32_t frequency = RF_FreqPlan.getChanFrequency(0);
  bool high = (frequency >= 1500000000) ; /* above 1.5 GHz */

  float br, fdev, bw;
  switch (rf_protocol->modulation_type)
  {
  case RF_MODULATION_TYPE_LORA:

    rl_state = radio_lr20xx->setModem(RADIOLIB_MODEM_LORA);
    delay(1);
    rl_state = radio_lr20xx->setTCXO(Vtcxo);

    switch (RF_FreqPlan.Bandwidth)
    {
    case RF_RX_BANDWIDTH_SS_125KHZ:
      bw = high ? 406.0f : 250.0f; /* BW_250 */
      break;
    case RF_RX_BANDWIDTH_SS_62KHZ:
      bw = high ? 203.0f : 125.0f; /* BW_125 */
      break;
    case RF_RX_BANDWIDTH_SS_250KHZ:
      bw = high ? 812.0f : 500.0f; /* BW_500 */
      break;
    //case RF_RX_BANDWIDTH_SS_125KHZ:
    default:
      bw = high ? 406.0f : 250.0f; /* BW_250 */
      break;
    }

    rl_state = radio_lr20xx->setBandwidth(bw);

#if RADIOLIB_DEBUG_BASIC
    if (rl_state == RADIOLIB_ERR_INVALID_BANDWIDTH) {
      Serial.println(F("[LR20XX] Selected bandwidth is invalid for this module!"));
      return;
    }
#endif

    switch (rf_protocol->type)
    {
    case RF_PROTOCOL_FANET:
    default:
      rl_state = radio_lr20xx->setSpreadingFactor(7); /* SF_7 */
      rl_state = radio_lr20xx->setCodingRate(5);      /* CR_5 */
      break;
    }

    rl_state = radio_lr20xx->setSyncWord((uint8_t) rf_protocol->syncword[0]);

#if RADIOLIB_DEBUG_BASIC
    if (rl_state == RADIOLIB_ERR_INVALID_SYNC_WORD) {
      Serial.println(F("[LR20XX] Selected sync word is invalid for this module!"));
      return;
    }
#endif

    rl_state = radio_lr20xx->setPreambleLength(8);
    rl_state = radio_lr20xx->explicitHeader();
    rl_state = radio_lr20xx->setCRC(0);
    break;

  case RF_MODULATION_TYPE_PPM:
#if RADIOLIB_DEBUG_BASIC
    Serial.print(F("[LR20XX] Initializing OOK ... "));
#endif

    rl_state = radio_lr20xx->beginOOK(434.0f, 4.8f, 153.8f, 10, 16, Vtcxo);
    //rl_state = radio_lr20xx->setModem(RADIOLIB_MODEM_???);
    //delay(1);
    //rl_state = radio_lr20xx->setTCXO(Vtcxo);

#if RADIOLIB_DEBUG_BASIC
    if (rl_state != RADIOLIB_ERR_NONE) {
      Serial.print(F("OOK setup failed, code "));
      Serial.println((int16_t) rl_state);
      return;
    }
#endif

    //switch (rf_protocol->bitrate)
    //{
    //case RF_BITRATE_2000KBPS:
    //default:
      br = 2000.0f;
    //  break;
    //}
    rl_state = radio_lr20xx->setBitRate(br);

#if RADIOLIB_DEBUG_BASIC
  if (rl_state == RADIOLIB_ERR_INVALID_BIT_RATE) {
    Serial.println(F("[LR20XX] Selected bit rate is invalid for this module!"));
    return;
  } else if (rl_state == RADIOLIB_ERR_INVALID_BIT_RATE_BW_RATIO) {
    Serial.println(F("[LR20XX] Selected bit rate to bandwidth ratio is invalid!"));
    Serial.println(F("[LR20XX] Increase receiver bandwidth to set this bit rate."));
    return;
  }
#endif

    //switch (rf_protocol->bandwidth)
    //{
    //case RF_RX_BANDWIDTH_SS_1567KHZ:
    //default:
      bw = 3076.0f;
    //  break;
    //}

    rl_state = radio_lr20xx->setRxBandwidth(bw);

    if (rf_protocol->preamble_size > 0) {
      rl_state = radio_lr20xx->setPreambleLength(rf_protocol->preamble_size * 8);
    }
    rl_state = radio_lr20xx->setDataShaping(RADIOLIB_SHAPING_NONE);

    //switch (rf_protocol->crc_type)
    //{
    //case RF_CHECKSUM_TYPE_CRC_MODES:
    //default:
      /* CRC is driven by software */
      rl_state = radio_lr20xx->setCRC(0);
//      rl_state = radio_lr20xx->setCRC(3, 0, 0x1FFF409UL, false);
    //  break;
    //}

    {
      size_t pkt_size = rf_protocol->payload_offset +
                        rf_protocol->payload_size   +
                        rf_protocol->crc_size;

#if OPT_DF17 == 0
      switch (rf_protocol->whitening)
      {
      case RF_WHITENING_MANCHESTER:
        {
          uint8_t enc = rf_protocol->payload_type == RF_PAYLOAD_INVERTED ?
                        RADIOLIB_ENCODING_MANCHESTER_INV :
                        RADIOLIB_ENCODING_MANCHESTER;
          rl_state = radio_lr20xx->setEncoding(enc);
        }
        break;
      default:
        rl_state = radio_lr20xx->setEncoding(RADIOLIB_ENCODING_NRZ);
        break;
      }
#endif /* OPT_DF17 == 0 */

#if OPT_DF17 != 0
      // use software Manchester decoding
      // 112 bits is 14 bytes, 28 bytes Manchester encoded
      pkt_size += pkt_size;
#if OPT_DF17 == 3
      pkt_size -= 2;       // 26 bytes, since the leading 95A6 are implicit   
#else
      pkt_size -= 1;
#endif
      rl_state = radio_lr20xx->setEncoding(RADIOLIB_ENCODING_NRZ);
#endif /* OPT_DF17 != 0 */

//    rl_state = radio_lr20xx->setWhitening(false);
      rl_state = radio_lr20xx->fixedPacketLengthMode(pkt_size);
    }

#if OPT_DF17 == 0
    rl_state = radio_lr20xx->setSyncWord((uint8_t *) rf_protocol->syncword,
                                     (size_t)    rf_protocol->syncword_size);
    rl_state = radio_lr20xx->ookDetector(0x0285, 16, 0, false, false, 0);
    rl_state = radio_lr20xx->setGain(13);
#endif /* OPT_DF17 == 0 */

#if OPT_DF17 == 1
    {
      uint8_t df17_sync_word[1] = { 0x95 };
      rl_state = radio_lr20xx->setSyncWord(df17_sync_word, sizeof(df17_sync_word));
      rl_state = radio_lr20xx->ookDetector(0x0285, 16, 0, false, false, 0);
      rl_state = radio_lr20xx->setGain(0);
    }
#endif /* OPT_DF17 == 1 */

#if OPT_DF17 == 2
    rl_state = radio_lr20xx->setSyncWord((uint8_t *) rf_protocol->syncword,
                                     (size_t)    rf_protocol->syncword_size);
    rl_state = radio_lr20xx->ookDetector(0xa902, 16, 0, false, false, 0);
    rl_state = radio_lr20xx->setGain(0);
#endif /* OPT_DF17 == 2 */

#if OPT_DF17 == 3
    // https://blog.z2labs.io/article/plane-spotting-with-semtechs-lr2021-
    uint8_t df17_sync_word[1] = { 0xA6 };
    // - encoded second half (0xD) of the first byte of the true payload
    rl_state = radio_lr20xx->setSyncWord(df17_sync_word, 1);
    rl_state = radio_lr20xx->ookDetector(0xA902, 16, 0, false, false, 0);
    // - the A902 encodes (in reverse bit order) the second half of the ADS-B preamble,
    //   plus the first half (0x8, encoded 0x95) of the first byte of the true payload
    // - the pattern length may need to be specified as 1 less than the full length of 16?
    rl_state = radio_lr20xx->setGain(0);
#endif /* OPT_DF17 == 3 */

    rl_state = radio_lr20xx->setOokDetectionThreshold(-80); /* TODO */
    break;

  case RF_MODULATION_TYPE_2FSK:
  default:

#if 0
    //rl_state = radio_lr20xx->beginGFSK(434.0f, 4.8f, 5.0f, 153.8f, 10, 16, Vtcxo);
#else
    rl_state = radio_lr20xx->setModem(RADIOLIB_MODEM_FSK);
    delay(1);
    rl_state = radio_lr20xx->setTCXO(Vtcxo);
#endif

    switch (rf_protocol->bitrate)
    {
    case RF_BITRATE_100KBPS:
      br = high ? 125.0f : 100.0f; /* SX128x minimum is 125 kbps */
      break;
    case RF_BITRATE_38400:
      br = high ? 125.0f :  38.4f; /* SX128x minimum is 125 kbps */
      break;
    case RF_BITRATE_1042KBPS:
      br = 1041.667f;
      break;
    //case RF_BITRATE_100KBPS:
    default:
      br = high ? 125.0f : 100.0f; /* SX128x minimum is 125 kbps */
      break;
    }
    rl_state = radio_lr20xx->setBitRate(br);

#if RADIOLIB_DEBUG_BASIC
  if (rl_state == RADIOLIB_ERR_INVALID_BIT_RATE) {
    Serial.println(F("[LR20XX] Selected bit rate is invalid for this module!"));
    return;
  } else if (rl_state == RADIOLIB_ERR_INVALID_BIT_RATE_BW_RATIO) {
    Serial.println(F("[LR20XX] Selected bit rate to bandwidth ratio is invalid!"));
    Serial.println(F("[LR20XX] Increase receiver bandwidth to set this bit rate."));
    return;
  }
#endif

    switch (rf_protocol->deviation)
    {
    case RF_FREQUENCY_DEVIATION_50KHZ:
      fdev = high ? 62.5f : 50.0f; /* SX128x minimum is 62.5 kHz */
      break;
    case RF_FREQUENCY_DEVIATION_10KHZ:
      fdev = high ? 62.5f : 10.0f; /* SX128x minimum is 62.5 kHz */
      break;
    case RF_FREQUENCY_DEVIATION_12_5KHZ:
      fdev = 12.5f;
      break;
    case RF_FREQUENCY_DEVIATION_25KHZ:
      fdev = 25.0f;
      break;
    case RF_FREQUENCY_DEVIATION_625KHZ:
#if RADIOLIB_CHECK_PARAMS == 1
      fdev = 500.0f; /* FDEV = 500.0 kHz is a specified maximum for LR2021 */
#else
      fdev = 625.0f;
#endif
      break;
    //case RF_FREQUENCY_DEVIATION_50KHZ:
    //case RF_FREQUENCY_DEVIATION_NONE:
    default:
      fdev = high ? 62.5f : 50.0f; /* SX128x minimum is 62.5 kHz */
      break;
    }
    rl_state = radio_lr20xx->setFrequencyDeviation(fdev);

#if RADIOLIB_DEBUG_BASIC
  if (rl_state == RADIOLIB_ERR_INVALID_FREQUENCY_DEVIATION) {
    Serial.println(F("[LR20XX] Selected frequency deviation is invalid for this module!"));
    return;
  }
#endif

    switch (rf_protocol->bandwidth)
    {
    case RF_RX_BANDWIDTH_SS_125KHZ:
      bw = 307.7f;
      break;
    case RF_RX_BANDWIDTH_SS_50KHZ:
      bw = 119.0f;
      break;
    case RF_RX_BANDWIDTH_SS_62KHZ:
      bw = 153.8f;
      break;
    case RF_RX_BANDWIDTH_SS_100KHZ:
      bw = 238.1f;
      break;
    case RF_RX_BANDWIDTH_SS_166KHZ:
      bw = 370.4f;
      break;
    case RF_RX_BANDWIDTH_SS_200KHZ:
      bw = 476.2f;
      break;
    case RF_RX_BANDWIDTH_SS_250KHZ:
      bw = 555.6f;
      break;
    case RF_RX_BANDWIDTH_SS_1567KHZ:
      bw = 3076.0f;
      break;
    //case RF_RX_BANDWIDTH_SS_125KHZ:
    default:
      bw = 307.7f;
      break;
    }

    rl_state = radio_lr20xx->setRxBandwidth(bw);
    rl_state = radio_lr20xx->setPreambleLength(rf_protocol->preamble_size * 8);
    rl_state = radio_lr20xx->setDataShaping(RADIOLIB_SHAPING_0_5);

/*
    switch (rf_protocol->crc_type)
    {
    case RF_CHECKSUM_TYPE_CCITT_FFFF:
    case RF_CHECKSUM_TYPE_CCITT_0000:
    case RF_CHECKSUM_TYPE_CCITT_1D02:
    case RF_CHECKSUM_TYPE_CRC8_107:
    case RF_CHECKSUM_TYPE_RS:
      rl_state = radio_lr20xx->setCRC(0);   // CRC is done in software
      break;
    case RF_CHECKSUM_TYPE_GALLAGER:
    case RF_CHECKSUM_TYPE_CRC_MODES:
    case RF_CHECKSUM_TYPE_NONE:
    default:
      rl_state = radio_lr20xx->setCRC(0);
      break;
    }
*/
    rl_state = radio_lr20xx->setCRC(0);

    size_t pkt_size = rf_protocol->payload_offset + rf_protocol->payload_size +
                      rf_protocol->crc_size;

    switch (rf_protocol->whitening)
    {
    case RF_WHITENING_MANCHESTER:
      //pkt_size += pkt_size;   - only if using hardware Manchester?
      break;
    //case RF_WHITENING_PN9:
    //case RF_WHITENING_NONE:
    //case RF_WHITENING_NICERF:
    default:
      break;
    }

    rl_state = radio_lr20xx->setWhitening(false, 0x0001 /* default SX128x value */);
    rl_state = radio_lr20xx->disableAddressFiltering();

    /* Work around premature P3I syncword detection */
    if (rf_protocol->type == RF_PROTOCOL_P3I && rf_protocol->syncword_size == 2) {
      uint8_t preamble = rf_protocol->preamble_type == RF_PREAMBLE_TYPE_AA ?
                         0xAA : 0x55;
      uint8_t sword[4] = { preamble,
                           preamble,
                           rf_protocol->syncword[0],
                           rf_protocol->syncword[1]
                         };
      rl_state = radio_lr20xx->setSyncWord(sword, 4);
    } else {
      rl_state = radio_lr20xx->setSyncWord((uint8_t *) rf_protocol->syncword,
                                       (size_t)    rf_protocol->syncword_size);
    }

    rl_state = radio_lr20xx->fixedPacketLengthMode(pkt_size);
    break;
  }

  switch (hw_info.model)
  {
  //case SOFTRF_MODEL_BADGE:
  //case SOFTRF_MODEL_PRIME_MK3:
  default:
    radio_lr20xx->setRfSwitchTable(rfswitch_dio_pins_MXD8721, rfswitch_table_MXD8721);
    rl_state = radio_lr20xx->setOutputPower(tx_power);
    break;
  }

  rl_state = radio_lr20xx->setRxBoostedGainMode(high ? RADIOLIB_LR2021_RX_BOOST_HF :
                                                       RADIOLIB_LR2021_RX_BOOST_LF);
  radio_lr20xx->setPacketReceivedAction(receive_handler);
}

static uint8_t lr20xx_receive(uint8_t *packet)
{
  if (settings->power_save & POWER_SAVE_NORECEIVE) {
    return 0;
  }

  int rl_state;

  if (!receive_active) {
    lr20xx_setup(curr_rx_protocol_ptr, false);
    receive_complete = false;
    rl_state = radio_lr20xx->startReceive();
    if (rl_state == RADIOLIB_ERR_NONE)
        receive_active = true;
  }

  if (receive_complete == false)
      return 0;

  receive_complete = false;
  receive_active = false;
  receive_cb_count++;

  uint8_t length = radio_lr20xx->getPacketLength();
  if (length == 0) {
      (void) radio_lr20xx->finishReceive();
      return 0;
  }

#if OPT_DF17 == 3
  if (length > RADIOLIB_MAX_DATA_LENGTH-2)
      length = RADIOLIB_MAX_DATA_LENGTH-2;
  if (curr_rx_protocol_ptr->type == RF_PROTOCOL_ADSB_1090) {
      // the detected-preamble + syncword covered up the first byte of the ADS-B packet
      // thus need to insert it manually into the packet buffer:
      packet[0] = 0x95;
      packet[1] = 0xA6;
  }
  rl_state = radio_lr20xx->readData(&packet[2], length);
#else
  if (length > RADIOLIB_MAX_DATA_LENGTH)
      length = RADIOLIB_MAX_DATA_LENGTH;
  rl_state = radio_lr20xx->readData(packet, length);
#endif
  (void) radio_lr20xx->finishReceive();
  if (rl_state != RADIOLIB_ERR_NONE)
      return 0;
  RF_last_rssi = radio_lr20xx->getRSSI(true));
  return length;
}

static int16_t lr20xx_transmit(uint8_t *packet, uint8_t length)
{
  if (curr_tx_protocol_ptr->type == RF_PROTOCOL_ADSB_1090 ||
      curr_tx_protocol_ptr->type == RF_PROTOCOL_ADSB_UAT) {
    return RADIOLIB_ERR_NONE; /* no transmit on 1090 or 978 MHz */
  }

  lr20xx_setup(curr_tx_protocol_ptr, true);

  int rl_state = radio_lr20xx->transmit(packet, (size_t) length);

  return (rl_state);
}

static void lr20xx_shutdown()
{
  int rl_state = radio_lr20xx->standby(RADIOLIB_LR2021_STANDBY_RC);
  rl_state = radio_lr20xx->setTCXO(0);
  rl_state = radio_lr20xx->sleep(false, 0);
  RadioSPI.end();
}
#endif /* INCLUDE_LR2021 */

/******************************************************/


bool rf_chips_probe()
{
#if defined(INCLUDE_SX127X)
    if (sx1276_probe()) {
      rf_chip = &sx1276_ops;
#else
    if (false) {
#endif
#if defined(INCLUDE_SX126X)
    } else if (sx1262_probe()) {
      rf_chip = &sx1262_ops;
#endif
#if defined(INCLUDE_LR11X0)
    } else if (lr1110_probe()) {
      rf_chip = &lr1110_ops;
#endif
#if defined(INCLUDE_LR2021)
    } else if (lr1121_probe()) {
      rf_chip = &slr1121_ops;
#endif
    }

    if (rf_chip && rf_chip->name) {
      Serial.print(rf_chip->name);
      Serial.println(F(" RFIC is detected."));
      tx_power = calc_txpower();
      //rf_chip->setup();
      return true;
    }
    
    Serial.println(F("WARNING! None of supported RFICs is detected!"));    
    return false;
}
