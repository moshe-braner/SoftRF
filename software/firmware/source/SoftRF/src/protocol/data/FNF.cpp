/*
 * FNF.cpp
 * 2026 Moshe Braner - based on code by Vlad Belayev
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

#include "../../system/SoC.h"
// which does #include "../../SoftRF.h"
#include "../../TrafficHelper.h"
#include "../../driver/Settings.h"
#include "NMEA.h"
#include "FNF.h"

#if defined(INCLUDE_FNF)

uint8_t FNF_dest = DEST_NONE;

uint8_t fnf_airmode = 0;   // 0=auto, 1=forced airborne
uint8_t fnf_rfmode = 15;   // bitmask: bit0 FANET_RX, bit1 FANET_TX, bit2 FLARM_RX, bit3 FLARM_TX

// mask NMEA source, otherwise NMEA_Out() refuses to output to source
void NMEA_Out_to_FNF_dest(const char *buf, int len)
{
    uint8_t saved_source = NMEA_Source;
    NMEA_Source = DEST_NONE;
    NMEA_Out(FNF_dest, buf, len, true);
    NMEA_Source = saved_source;
}

/*
 * #FNF - FANET frame output for XCGuide (GXAirCom protocol)
 * Format: #FNF src_manufacturer,src_id,broadcast,signature,type,length,payload\n
 * All header fields hex without leading zeros, payload bytes hex with leading zeros.
 */

// this is no longer called from TrafficHelper ParseData() as soon as a packet has arrived
// instead it is called from fanet_decode()

void NMEA_FNF_Out(const uint8_t *raw, size_t raw_len)
{
    if ( /* FNF_dest == DEST_NONE || */ raw_len < FANET_HEADER_SIZE)
        return;

    /* Parse FANET header (4 bytes) */
    uint8_t type = raw[0] & 0x3F;

    uint8_t vendor = raw[1];
    uint16_t address = raw[2] | ((uint16_t)raw[3] << 8);

    /* Calculate actual payload offset - skip extended header if present */
    bool has_ext = raw[0] & 0x80;
    bool unicast = false;
    bool ack_requested = false;
    size_t payload_offset = FANET_HEADER_SIZE;   /* 4 bytes basic header */
    if (has_ext && raw_len > payload_offset) {
        ack_requested = raw[payload_offset] & (1 << 6);
        unicast = raw[payload_offset] & (1 << 5);
        payload_offset += 1;                      /* +1 ext header byte */
        if (unicast)
            payload_offset += 3;                  /* +3 dest addr bytes */
    }

    // unicast messages not addressed to us were dropped earlier, in fanet_decode()

    size_t payload_len = (raw_len > payload_offset) ? raw_len - payload_offset : 0;

    /* Build #FNF sentence into NMEABuffer */
    int len = snprintf(NMEABuffer, sizeof(NMEABuffer)-1, "#FNF %X,%X,%X,%X,%X,%X,%s",
        (unsigned)vendor,       /* src_manufacturer */
        (unsigned)address,      /* src_id */
        (unsigned)(unicast ? 0 : 1),  /* 1=broadcast, 0=unicast */
        0,                      /* signature (not used in SoftRF) */
        (unsigned)type,         /* FANET frame type */
        (unsigned)payload_len,  /* payload length */
        bytes2Hex(&raw[payload_offset], payload_len));

    //NMEABuffer[len++] = '\n';
    NMEABuffer[len] = '\0';

    NMEA_Out_to_FNF_dest(NMEABuffer, len);

    if (settings->debug_flags)
        Serial.println(NMEABuffer);
}

/*
 * Parse #FNG command: set ground tracking type
 * Format: #FNG groundType\r\n
 * groundType: single hex digit 0-F
 */
void FN_process_FNG(const char *args)
{
    unsigned int gtype = 0;
    Serial.print("FNG: raw args='");
    Serial.print(args);
    Serial.println("'");
    if (sscanf(args, "%1X", &gtype) == 1  && (gtype==GROUND_STATUS_INITIAL ||
         (gtype>=GROUND_STATUS_NEED_RIDE && gtype<=GROUND_STATUS_DISTRESS && (gtype!=10 && gtype!=11)))) {
        ground_status = (uint8_t)gtype;
        if (gtype != GROUND_STATUS_INITIAL)
            ThisAircraft.airborne = 0;  /* explicit ground command - force ground mode */
        /* Distress ground types trigger SOS message transmission */
        NMEA_Out_to_FNF_dest("#FNR OK", 8);
        Serial.print("FNG: ground type=0x");
        Serial.print(gtype, HEX);
        Serial.print((gtype >= GROUND_STATUS_NEED_MED) ? " (distress)" : "");
        Serial.println();

    } else {
        NMEA_Out_to_FNF_dest("#FNR ERR,22,Bad ground type", 28);
        Serial.print("FNG: bad ground type=");
        Serial.println(gtype, HEX);
    }
}

/* Pending ACK tracking for unicast messages sent via #FNT */
static uint8_t  fnf_ack_pending_mfr = 0;
static uint16_t fnf_ack_pending_id  = 0;
static uint32_t fnf_ack_pending_ms  = 0;
#define FNF_ACK_TIMEOUT_MS  15000

/* Retransmission of unicast ACK-required frames when no ACK arrives in time.
 * Original frame is kept so it can be re-queued unchanged (same payload,
 * so the recipient's dedup logic still recognizes it as the same message). */
#define FNF_ACK_MAX_RESENDS   2   /* additional attempts beyond the original TX */
static uint8_t  fnf_ack_pending_frame[MAX_PKT_SIZE];
static size_t   fnf_ack_pending_frame_len = 0;
static uint8_t  fnf_ack_resends_left = 0;

/*
 * Parse #FNT command: transmit a FANET frame
 * Format: #FNT type,dest_manufacturer,dest_id,forward,ack_required,length,payload[,signature]\r\n
 * All values in hex.
 *
 * FANET header layout:
 *   Byte 0: [7] ext_header, [6] forward, [5-0] type
 *   Bytes 1-3: src manufacturer(1) + src address(2, LE)
 *   Byte 4 (if ext_header): [7-6] ACK, [5] unicast, [4] signature, [3-0] reserved
 *   Bytes 5-7 (if unicast): dest manufacturer(1) + dest address(2, LE)
 */
void FN_process_FNT(const char *args)
{
    unsigned int type, dest_mfr, dest_id, fwd, ack, plen;
    int consumed = 0;

    if (sscanf(args, "%X,%X,%X,%X,%X,%X,%n",
               &type, &dest_mfr, &dest_id, &fwd, &ack, &plen, &consumed) < 6) {
        NMEA_Out_to_FNF_dest("#FNR ERR,22,Parse error", 24);
        Serial.println("#FNT Parse error");
        return;
    }

    bool unicast = (dest_mfr != 0 || dest_id != 0);
    bool need_ext = (unicast || ack);

    /* Calculate header size */
    size_t hdr_size = FANET_HEADER_SIZE;         /* 4 bytes basic */
    if (need_ext) hdr_size += 1;                 /* +1 extended header byte */
    if (unicast)  hdr_size += 3;                 /* +3 dest address bytes */

    if (hdr_size + plen >= MAX_PKT_SIZE) {
        NMEA_Out_to_FNF_dest("#FNR ERR,21,Frame too long", 27);
        Serial.println("#FNT Frame too long");
        return;
    }

    /* Parse hex payload */
    const char *hex = args + consumed;
    size_t hex_len = strlen(hex);

    /* The declared length must match the actual hex payload present -
     * otherwise a truncated line (e.g. app/BLE bug) would silently
     * transmit a short or garbage frame instead of being rejected. */
    if (hex_len != (size_t)plen * 2) {
        char buf[64];
        int len = snprintf(buf, sizeof(buf),
                            "#FNR ERR,23,Length mismatch %u!=%u",
                            (unsigned)hex_len, (unsigned)(plen * 2));
        NMEA_Out_to_FNF_dest(buf, len);
        Serial.print("FNT: length mismatch, declared plen=");
        Serial.print(plen);
        Serial.print(" (");
        Serial.print(plen * 2);
        Serial.print(" hex chars) but got ");
        Serial.print(hex_len);
        Serial.println(" hex chars - rejecting");
        return;
    }

    //uint8_t frame[MAX_PKT_SIZE];
    uint8_t *frame = fn_tx_pending_buf;
    fn_tx_pending_len = 0;

    /* Build FANET header */
    frame[0] = (type & 0x3F) | ((fwd ? 1 : 0) << 6) | (need_ext ? 0x80 : 0);
    uint32_t id = ThisAircraft.addr;
    frame[1] = (id >> 16) & 0xFF;    /* src vendor */
    frame[2] = id & 0xFF;            /* src address low */
    frame[3] = (id >> 8) & 0xFF;     /* src address high */

    size_t pos = FANET_HEADER_SIZE;

    if (need_ext) {
        uint8_t ext = 0;
        if (ack)     ext |= (1 << 6);    /* ACK requested (value=1) */
        if (unicast) ext |= (1 << 5);    /* unicast */
        frame[pos++] = ext;
    }

    if (unicast) {
        frame[pos++] = (uint8_t)(dest_mfr & 0xFF);
        frame[pos++] = (uint8_t)(dest_id & 0xFF);
        frame[pos++] = (uint8_t)((dest_id >> 8) & 0xFF);
    }

    if (hex2bytes(hex, &frame[pos], plen) == false) {
            NMEA_Out_to_FNF_dest("#FNR ERR,22,Bad payload hex", 28);
            Serial.println("#FNT Bad payload hex");
            return;
    }

    size_t frame_len = pos + plen;

    /* Queue for transmission via RF */
    Serial.print("FNT: TX Type ");
    Serial.print(type);
    Serial.print(" len=");
    Serial.print(frame_len);
    if (unicast) {
        Serial.print(" to ");
        Serial.print(dest_mfr, HEX);
        Serial.print(",");
        Serial.print(dest_id, HEX);
    } else {
        Serial.print(" broadcast");
    }
    Serial.print(" ack=");
    Serial.println(ack);

    /* Log the payload as received from the app (hex + printable ASCII),
     * as handed to us, before any radio TX - so a report of a truncated
     * or garbled message can be checked against what the app actually
     * sent, without needing to enable DEBUG_BLE_TX and decode by hand. */
    Serial.print("FNT: payload[");
    Serial.print(plen);
    Serial.print("]=");
    Serial.write((const uint8_t *)hex, hex_len);
    Serial.print(" \"");
    Serial.print(filter_printable((const unsigned char *) &frame[pos], plen));
    Serial.println("\"");

    /* Queue for transmission in next available FANET TX slot */
    //memcpy(fn_tx_pending_buf, frame, frame_len);
    fn_tx_pending_len = frame_len;

    /* If ACK requested for unicast, track it so we can match incoming ACK,
     * and keep a copy of the frame so we can resend it on timeout. */
    if (ack && unicast) {
        fnf_ack_pending_mfr = (uint8_t)dest_mfr;
        fnf_ack_pending_id  = (uint16_t)dest_id;
        fnf_ack_pending_ms  = millis();
        memcpy(fnf_ack_pending_frame, frame, frame_len);
        fnf_ack_pending_frame_len = frame_len;
        fnf_ack_resends_left = FNF_ACK_MAX_RESENDS;
    }

    NMEA_Out_to_FNF_dest("#FNR OK", 8);
}

/*
 * Check incoming FANET ACK packets (Type 0 or 3) addressed to us.
 * If it matches an ACK we're waiting for, clear the pending status.
 * Called from fanet_decode().
 */
void FN_check_ack(uint8_t sender_mfr, uint16_t sender_id, uint8_t type)
{
    /* Only forward ACK to app if we're actually waiting for one (from #FNT with ack) */
    if (fnf_ack_pending_ms != 0 &&
        sender_mfr == fnf_ack_pending_mfr && sender_id == fnf_ack_pending_id) {
        Serial.print("FN_check_ack: type ");
        Serial.print(type);
        Serial.print(" - matches pending ACK for ");
        Serial.print(sender_mfr, HEX);
        Serial.print(",");
        Serial.println(sender_id, HEX);
        if (type == 0) {              // send #FNR ACK only if not also sending #FNF
            char buf[32];
            int len = snprintf(buf, sizeof(buf), "#FNR ACK,%X,%X",
                         (unsigned)sender_mfr, (unsigned)sender_id);
            NMEA_Out_to_FNF_dest(buf, len);
        }
        // clear the waiting for an ack, even on type 3
        fnf_ack_pending_ms = 0;
        fnf_ack_pending_frame_len = 0;
    } else if (type == 0) {
        Serial.print("FN_check_ack: ignoring unsolicited type 0 ACK from ");
        Serial.print(sender_mfr, HEX);
        Serial.print(",");
        Serial.println(sender_id, HEX);
    }
}

/*
 * Check for ACK timeout - called periodically from NMEA_Export().
 * If we've been waiting for an ACK longer than FNF_ACK_TIMEOUT_MS:
 *   - if resend attempts remain, re-queue the original frame for TX and
 *     restart the timeout (up to FNF_ACK_MAX_RESENDS extra attempts, every
 *     FNF_ACK_TIMEOUT_MS);
 *   - otherwise give up and send #FNR NACK to XCGuide.
 */
void FN_check_ack_timeout()
{
    if (FNF_dest == DEST_NONE || fnf_ack_pending_ms == 0)
        return;

    if ((millis() - fnf_ack_pending_ms) >= FNF_ACK_TIMEOUT_MS) {
        if (fnf_ack_resends_left > 0 && fnf_ack_pending_frame_len > 0) {
            fnf_ack_resends_left--;
            Serial.print("FN_check_ack_timeout: no ACK, resending (");
            Serial.print(fnf_ack_resends_left);
            Serial.println(" attempt(s) left)");
            memcpy(fn_tx_pending_buf, fnf_ack_pending_frame, fnf_ack_pending_frame_len);
            fn_tx_pending_len = fnf_ack_pending_frame_len;
            fnf_ack_pending_ms = millis();   /* restart timeout window */
        } else {
            char buf[32];
            int len = snprintf(buf, sizeof(buf), "#FNR NACK,%X,%X",
                               (unsigned)fnf_ack_pending_mfr, (unsigned)fnf_ack_pending_id);
            NMEA_Out_to_FNF_dest(buf, len);
            fnf_ack_pending_ms = 0;
            fnf_ack_pending_frame_len = 0;
        }
    }
}

/*
 * Process incoming #SYC commands from XCGuide (handshake/config protocol).
 * Any #SYC command triggers FNF mode (XCGuide may send FNTPWR? before VER?).
 * Queries are replied to, settings are acknowledged.
 */
bool SYC_process_command(char *buf, int len)
{
    if (len < 8 /* || buf[0] != '#' || buf[1] != 'S' */ || buf[2] != 'Y' || buf[3] != 'C')
        return false;

    /* Strip trailing \r\n */
    while (len > 0 && (buf[len-1] == '\r' || buf[len-1] == '\n'))
        len--;
    buf[len] = '\0';

    char *arg = buf + 5;  /* skip "#SYC " */

    /* Any #SYC command from XCGuide enables FNF mode - it may send
     * FNTPWR? or other queries before VER?, so don't wait for VER?. */
    if (FNF_dest == DEST_NONE) {
        FNF_dest = NMEA_Source;
        Serial.println("FNF enabled by #SYC handshake");
    }

    if (strcmp(arg, "VER?") == 0) {
        NMEA_Out_to_FNF_dest("#SYC VER=v008.1", 16);
        Serial.println("#SYC VER");
        return true;
    }
    if (strcmp(arg, "FNTPWR?") == 0) {
        NMEA_Out_to_FNF_dest("#SYC FNTPWR=14", 15);
        Serial.println("#SYC FNTPWR");
        return true;
    }
    if (strcmp(arg, "RFMODE?") == 0) {
        char reply[32];
        int rlen = snprintf(reply, sizeof(reply), "#SYC RFMODE=%u", fnf_rfmode);
        NMEA_Out_to_FNF_dest(reply, rlen);
        Serial.println("#SYC RFMODE");
        return true;
    }
    if (strcmp(arg, "NAME?") == 0) {
        char reply[64];
        //const char *name = fnf_session_name[0] ? fnf_session_name : settings->callsign;
        const char *name = settings->callsign;
        int rlen = snprintf(reply, sizeof(reply), "#SYC NAME=%s", name);
        NMEA_Out_to_FNF_dest(reply, rlen);
        Serial.println("#SYC NAME");
        return true;
    }
    if (strcmp(arg, "AIRMODE?") == 0) {
        char reply[32];
        int rlen = snprintf(reply, sizeof(reply), "#SYC AIRMODE=%u", fnf_airmode);
        NMEA_Out_to_FNF_dest(reply, rlen);
        Serial.println("#SYC AIRMODE");
        return true;
    }
    if (strcmp(arg, "TYPE?") == 0) {
        char reply[32];
        int rlen = snprintf(reply, sizeof(reply), "#SYC TYPE=%u",
                            AT_TO_FANET(ThisAircraft.aircraft_type));
        NMEA_Out_to_FNF_dest(reply, rlen);
        Serial.println("#SYC TYPE");
        return true;
    }

    /* Statements with '=' - parse known settings, acknowledge all */
    if (strchr(arg, '=') != NULL) {
        char *eq = strchr(arg, '=');
        *eq = '\0';
        const char *val = eq + 1;

        if (strcmp(arg, "NAME") == 0) {
            //strncpy(fnf_session_name, val, sizeof(fnf_session_name) - 1);
            strncpy(settings->callsign, val, sizeof(settings->callsign) - 1);
            //fnf_session_name[sizeof(fnf_session_name) - 1] = '\0';
            settings->callsign[sizeof(settings->callsign) - 1] = '\0';
            //Serial.printf("FNF session name set: %s\n", fnf_session_name);
            Serial.printf("FNF session name set: %s\n", settings->callsign);
        }
        else if (strcmp(arg, "AIRMODE") == 0) {
            uint8_t mode = (uint8_t)atoi(val);
            fnf_airmode = (mode == 1) ? 1 : 0;
            if (fnf_airmode) {
                ThisAircraft.airborne = 1;
                // - this may not stick, Wind.cpp this_airborne() will change it
                ground_status = GROUND_STATUS_AIRBORNE;
            }
            Serial.printf("FNF airmode set: %u\n", fnf_airmode);
        }
        else if (strcmp(arg, "TYPE") == 0) {
            uint8_t ftype = (uint8_t)atoi(val);
            if (ftype <= 7) {
                ThisAircraft.aircraft_type = AT_FROM_FANET(ftype);
                Serial.printf("FNF aircraft type set: FANET %u -> SoftRF %u\n",
                              ftype, ThisAircraft.aircraft_type);
            }
        }
        else if (strcmp(arg, "RFMODE") == 0) {
            uint8_t mode = (uint8_t)atoi(val);
            fnf_rfmode = mode & 0x0F;
            // - this is not actually used, no effect on SoftRF behavior
            Serial.printf("FNF rfmode set: %u\n", fnf_rfmode);
        }

        *eq = '=';  /* restore buffer */
        NMEA_Out_to_FNF_dest("#SYC OK", 8);
        return true;
    }

    return false;
}

/*
 * Process incoming #FN commands received from BLE
 * Returns true if the line was handled as an #FN command.
 */
bool FN_process_command(char *buf, int len)
{
    if (len < 5 /* || buf[0] != '#' || buf[1] != 'F' */ || buf[2] != 'N')
        return false;

    /* First #FN command from BLE enables FNF mode (app handshake) */
    if (FNF_dest == DEST_NONE) {
        FNF_dest = NMEA_Source;
        Serial.println("FNF enabled by #FN command");
    }

    /* Strip trailing \r\n */
    while (len > 0 && (buf[len-1] == '\r' || buf[len-1] == '\n'))
        len--;
    buf[len] = '\0';

    if (buf[3] == 'G' && len >= 5) {
        FN_process_FNG(buf + 4 + (buf[4] == ' ' ? 1 : 0));
        return true;
    }
    if (buf[3] == 'T' && buf[4] == ' ' && len > 5) {
        Serial.print("FN_cmd: received #FNT len=");
        Serial.println(len);
        FN_process_FNT(buf + 5);
        return true;
    }

    return false;
}

#endif  // INCLUDE_FNF

