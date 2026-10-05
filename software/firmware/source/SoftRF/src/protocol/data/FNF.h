/*
 * FNF.h
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

#ifndef FNF_H
#define FNF_H

//#if defined(ARDUINO_ARCH_NRF52) || defined(ARDUINO_ARCH_NRF52840)
#define INCLUDE_FNF
//#endif

#if defined(INCLUDE_FNF)
void NMEA_FNF_Out(const uint8_t *raw, size_t raw_len);
void FN_process_FNG(const char *args);
void FN_process_FNT(const char *args);
void FN_check_ack(uint8_t sender_mfr, uint16_t sender_id, uint8_t type);
void FN_check_ack_timeout();
bool SYC_process_command(char *buf, int len);
bool FN_process_command(char *buf, int len);
extern uint8_t FNF_dest;
#endif

#endif /* FNF_H */

