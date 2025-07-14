/*
 * IGC.cpp
 * Copyright (C) 2024 Moshe Braner
 *
 * Security-record code here borrowed from LK8000.
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

// include this first, to get USE_SD_CARD
#include "../../system/SoC.h"

#if defined(ESP32)
#if defined(USE_SD_CARD)
#include <SD.h>
#endif
#endif

#include "MD5.h"

#include "../../driver/Settings.h"
#include "../../driver/GNSS.h"
#include "../../driver/Baro.h"
#include "../../driver/Filesys.h"
#include "NMEA.h"
#include "IGC.h"

#if defined(ARDUINO_ARCH_NRF52)
#include "../../driver/EPD.h"
#endif

#if defined(ESP32)
char *PSRAMbuf = NULL;
char *PSRAMbuf2 = NULL;
size_t PSRAMbufSize = 0;
size_t PSRAMbufSize2 = 0;
size_t PSRAMbufUsed = 0;
size_t PSRAMbufUsed2 = 0;
bool suspended = false;
#endif

char FlightLogPath[20] = {'\0'};
char compfilename[16];
File FlightLog, compfile;
bool FlightLogOpen = false;    // means log is in process, but the *file* may be closed
bool compfileOpen = false;   // means log is in process, but the *file* may be closed
bool FlightLogFail = false;
uint32_t FlightLogClosed = 0;  // when a file was finalized, don't open a new one for a minute
size_t FlightLogPosition = 0;
size_t compfilePosition = 0;

//#define EXAMPLE_B_RECORD           "B1751494352910N07215306WA0012300123\r\n"
//static const char emptybrecord[] = "B~~~~^^~~~~^^^~~~~~~^^^~~~~~^^~~~^^\r\n";
static char brecord_format[] = "B1751494352910N07215306WA%05d%05d\r\n";
#define B_RECORD_SIZE 38
   // sizeof(emptybrecord) - the null char at end is included in sizeof()
static char compbrecord[B_RECORD_SIZE];     // for compression
#define PRE_POS_NUM   8    // number of pre-stored B-records
#define DATA_BLOCK_SIZE 3000
#define G_RECORD_SIZE 38   // 2 * (G+16+\r\n) - or, if compressed, 2 * (0x0A+G+16+\n), same size
#if defined(ESP32)
static char data_block_buf[DATA_BLOCK_SIZE + B_RECORD_SIZE*PRE_POS_NUM];
       // static, since malloc()ed would be in PSRAM, slowing MD5 updates
#else
static char *data_block_buf = NULL;   // will malloc() it later
#endif
static int data_block_used = 0;
static char *pre_positions_buf = NULL;
static int num_pre_positions = 0;
static int pre_positions_head = 0;
static int pre_positions_next = 0;

#if defined(ESP32)
static MD5_CTX md5_buf[8];   // do not malloc() since PSRAM access is slow for this
#else
static MD5_CTX *md5_buf = NULL;   // will malloc() it in setup()
#endif
static MD5_CTX *md5_a, *md5_b, *md5_c, *md5_d;
static MD5_CTX *md5_a_copy, *md5_b_copy, *md5_c_copy, *md5_d_copy;

/*
 * Compress flight logs
 *
            0         1         2         3
Position:    123456789 123456789 123456789 123456
B-record:  "B1751494352910N07215306WA0012300123\r\n"
Template:  "-xxxx--xxxx---xxxxxx---xxxxx--xxx--\r\n"   (22 template positions)
Compress:  "-----xx----xxx------xxx-----xx---xx\r\n"   (12 digits in 6 bytes, no header byte)

Decompression algorithm:  Read first byte of each line to classify it:
* If 0x0A, read the rest of the line verbatim (for header lines and G-records)
      - end of variable-length lines is signaled by \n, translate to \r\n
* If 0x0C, translate to "LPLT" and read the rest of the line verbatim
      - end of variable-length lines is signaled by \n, translate to \r\n
* Else bitwise-and the byte with 0xE0, and:
* If 0xA0, increment B-record template byte at index=(byte & 0x1F)
* If 0xE0, decrement B-record template byte at index=(byte & 0x1F)
* If 0xC0, read next byte into B-record template at index=(byte & 0x1F)
* Else, read compressed B-record (fixed-length, 6 bytes = 12 digits) (combine with template)
This relies on the digits being 0-9, thus a value > 9 in either nibble is not compressed digits.
*/

static uint8_t tpos[22] = {1,2,3,4,7,8,9,10,14,15,16,17,18,19,23,24,25,26,27,30,31,32};
static uint8_t bpos[12] = { 5, 6, 11, 12, 13, 20, 21, 22, 28, 29, 33, 34 };

// decompress a flash file (into PSRAMbuf on T-Beam)
bool decompressfile(char *filename)
{
    compfile = FILESYS.open(filename, FILE_READ);
    if (! compfile) {
        Serial.println("Failed to open compressed file for decompression");
        return false;
    }
#if defined(ESP32)
    if (! PSRAMbuf)
        return false;
    char *p = PSRAMbuf;
    char *t = PSRAMbuf + (PSRAMbufSize - 40);
#elif defined(ARDUINO_ARCH_NRF52)
    uint32_t free_kb = (IGCFS_is_mounted? IGCFS_free_kb() : 0);
    if (free_kb < 50+((6*compfile.size())>>10)) {
        compfile.close();
        Serial.println("Not enough file space for decompression");
        return false;
    }
    char sfn[8];
    sfn[0]=filename[1];
    sfn[1]=filename[2];
    sfn[2]=filename[3];
    sfn[3]='_';
    sfn[4]=filename[8];
    sfn[5]='\0';
    EPD_Message("DECOMP", sfn);
    delay(500);
    char outfilename[24];
    strcpy(outfilename,filename);
    outfilename[strlen(outfilename)-1] = 'C';   // overwriting 'X'
    File outfile = IGCFILESYS.open(outfilename, FILE_WRITE);
    if (! outfile) {
        compfile.close();
        Serial.println("Failed to open IGC file for decompression");
        return false;
    }
    char *p = data_block_buf;
    char *t = p + DATA_BLOCK_SIZE;  // there is also the prepositioning buf space above that
#endif
    char brecord[B_RECORD_SIZE];
    //strcpy(brecord, emptybrecord);
    memset(brecord,0,B_RECORD_SIZE);
    brecord[0] = 'B';
    brecord[35] = '\r';
    brecord[36] = '\n';
    //brecord[37] = '\0';
    int state = 0;
    int i = 0;
    while (compfile.available()) {
#if defined(ESP32)
        if (p >= t)
            break;
#elif defined(ARDUINO_ARCH_NRF52)
        if (p >= t) {
            size_t outsize = p - data_block_buf;
            if (outfile.write((uint8_t *)data_block_buf, outsize) < outsize) {
                compfile.close();
                outfile.close();
                IGCFILESYS.remove(outfilename);
                Serial.println("Failed to write to IGC file in decompression");
                return false;
            }
            p = data_block_buf;
        }
#endif
        uint8_t c = compfile.read();
        if (state == 0) {    // beginning of a line
            if (c == 0x0A) {
                state = 0xAA;
            } else if (c == 0x0C) {
                strcpy(p,"LPLT");
                p += 4;
                state = 0xAA;
            } else {
                int opr = (c & 0xE0);
                int idx = (c & 0x1F);
                if (idx > 21)  idx = 21;   // should not happen
                if (opr == 0xA0) {
                    ++brecord[tpos[idx]];
                } else if (opr == 0xE0) {
                    --brecord[tpos[idx]];
                } else if (opr == 0xC0) {
                    state = tpos[idx];
                    // will read next byte into template at that position
                } else {
                    // read a compressed B-record, starting with current byte
                    i = 0;
                    char c1 = '0' + (c & 0x0F);
                    char c2 = '0' + ((c & 0xF0) >> 4);
                    brecord[bpos[i++]] = c1;
                    brecord[bpos[i++]] = c2;
                    state = 0xBB;   // will read 5 more bytes
                }
            }
        } else if (state == 0xAA) {  // verbatim
            if (c == '\n') {      // end of line
                *p++ = '\r';
                *p++ = '\n';
                state = 0;
            } else {
                *p++ = c;
            }
        } else if (state == 0xBB) {   // decompress B-record
            char c1 = '0' + (c & 0x0F);
            char c2 = '0' + ((c & 0xF0) >> 4);
            if (i < 11) {
                brecord[bpos[i++]] = c1;
                brecord[bpos[i++]] = c2;
            }
            if (i >= 12) {
                memcpy(p, brecord, 37);
                p += 37;
                state = 0;
            }
        } else if (state <= 32) {   // template byte (following opr==0xC0)
            brecord[state] = c;
            state = 0;
        }
    }
    compfile.close();
#if defined(ESP32)
    PSRAMbufUsed = (p - PSRAMbuf);
#elif defined(ARDUINO_ARCH_NRF52)
    if (p > data_block_buf) {
        size_t outsize = p - data_block_buf;
        if (outfile.write((uint8_t *)data_block_buf, outsize) < outsize) {
            outfile.close();
            IGCFILESYS.remove(outfilename);
            Serial.println("Failed last write to IGC file in decompression");
            return false;
        }
    }
    outfile.close();
    Serial.println("... OK, deleting .IGX file");
    IGCFILESYS.remove(filename);
    delay(500);
#endif
    return true;
}

void FlightLog_setup()
{
#if defined(ESP32)
    if (hw_info.model == SOFTRF_MODEL_PRIME_MK2) {
#if defined(USE_SD_CARD)
        if (SD_is_mounted == false)
#endif
        {
            // log to PSRAM instead
            // get a big 3.5MB buffer - or at least 1MB
            // - transparent PSRAM access is limited to 4MB total
            size_t size = 3500000;
            while (size >= 1000000) {
                PSRAMbuf = (char *) malloc(size);
                if (PSRAMbuf)
                    break;
                size -= 500000;
            }
            if (! PSRAMbuf) {
                FlightLogFail = true;
                Serial.println("Failed to allocate PSRAM for IGC");
                return;
            } else {
                PSRAMbufSize = size;
                Serial.print("IGC PSRAMbufSize=");
                Serial.println(PSRAMbufSize);
            }
        }
    }
    // << what if ESP32 but not PRIME_MK2?
#elif defined(ARDUINO_ARCH_NRF52)
    data_block_buf = (char *) malloc(DATA_BLOCK_SIZE + B_RECORD_SIZE*PRE_POS_NUM);
    if (! data_block_buf) {
        FlightLogFail = true;
        Serial.println("Failed to allocate data_block_buf");
        return;
    }
    md5_buf = (MD5_CTX *) malloc(8*sizeof(MD5_CTX));   // about 1500 bytes total
    if (! md5_buf) {
        FlightLogFail = true;
        Serial.println("Failed to allocate MD5_buf");
        return;
    }
#endif

    //Serial.printf("flightlog data_block size: %d\r\n", DATA_BLOCK_SIZE + B_RECORD_SIZE*PRE_POS_NUM);
    pre_positions_buf = &data_block_buf[DATA_BLOCK_SIZE];

    MD5_CTX *p = md5_buf;
    md5_a = p++;
    md5_b = p++;
    md5_c = p++;
    md5_d = p++;
    md5_a_copy = p++;
    md5_b_copy = p++;
    md5_c_copy = p++;
    md5_d_copy = p;
}

#if defined(ARDUINO_ARCH_NRF52)
void FlightLog_decomp()
{
    // if there are xxxxxxxx.IGX files, decompress them (if there is enough space)
    Serial.println(F("Looking for .IGX files..."));
    File root = IGCFILESYS.open("/");
    if (! root) {
        Serial.println(F("Cannot open IGCFILESYS root"));
        return;
    }
    char fn[20];
    fn[0] = '/';
    bool any_decomp = false;
    File file = root.openNextFile();
    for (; file; file=root.openNextFile()) {
        if (file.isDirectory())
            continue;
        //strncpy(&fn[1], file.name(), 19);
        file.getName(fn+1,19);
        Serial.println(fn);
        if (strlen(fn) != 13)   // including the '/'
            continue;
        if (strncmp(fn+9,".IGX",4) == 0) {
            Serial.println(F("- try decompress this one..."));
            file.close();
            if (! data_block_buf)
                FlightLog_setup();
            if (decompressfile(fn));
                any_decomp = true;
            //else
            //    EPD_Message("DECOMP", "FAILED");
            delay(500);    
        }
    }
    file.close();
    root.close();
    if (any_decomp)
        EPD_Message("DECOMP", "DONE");
    delay(500);
}
#endif

char clean_igc_char(char c)
{
    if (c >= 0x2D && c <= 0x5B)
      return c;        // for efficiency
    if (c >= 0x20 && c <= 0x7E && c != 0x24 &&
        c != 0x2A && c != 0x2C && c != 0x21 &&
        c != 0x5C && c != 0x5E && c != 0x7E) {
      return c;
    }
    return ' ';
}

void failFlightLog()
{
#if defined(ESP32)
    if (! PSRAMbuf)
        FlightLog.close();
    PSRAMbufUsed = 0;
#elif defined(ARDUINO_ARCH_NRF52)
    if (! settings->compflash)
        FlightLog.close();
#endif
    FlightLogOpen = false;
    FlightLogFail = true;
    FlightLogPosition = 0;
    compfilePosition = 0;
}

void reopenFlightLog()
{
    if (FlightLogFail)
        return;
    if (! FlightLogOpen)    // no log file in process
        return;
#if defined(ESP32)
    int limit = 10;
    if (PSRAMbuf)
#elif defined(ARDUINO_ARCH_NRF52)
    int limit = 50;
    if (settings->compflash)
#endif
    {
        if (compfileOpen) {
            if (FILESYS_free_kb() < limit) {   // flash almost full, stop logging into it
                compfileOpen = false;
                char buf[16];
                strcpy(buf,compfilename);
                buf[8] = '_';    // rename /xxxxxxx1.IGZ /xxxxxxx_.IGZ
                FILESYS.rename(compfilename,buf);
                Serial.print("flash almost full, renamed ");
                Serial.print(compfilename);
                Serial.print(" ");
                Serial.println(buf);
            } else {
#if defined(ESP32)
                compfile = FILESYS.open(compfilename, "r+");
                   // - FILE_WRITE doesn't work for this on SPIFFS
                if (!compfile || !compfile.seek(compfilePosition,SeekSet))
                    compfileOpen = false;
#elif defined(ARDUINO_ARCH_NRF52)
                compfile = FILESYS.open(compfilename, FILE_WRITE);
                if (!compfile || !compfile.seek(compfilePosition))
                    compfileOpen = false;
#endif
                Serial.print("Reopened flash flight log at position ");
                Serial.println(compfilePosition);
            }
        }
        return;
    }
    if (IGCFS_free_kb() < 50) {   // space almost full (on SD card this is a dummy function)
        failFlightLog();
        return;
    }
    if (!FlightLogPath[0])  // file name has not been generated
        return;
    FlightLog = IGCFILESYS.open(FlightLogPath, FILE_WRITE);
    if (!FlightLog) {
        Serial.println("Failed to re-open flight log for writing");
        FlightLogFail = true;
        return;
    }
    if (!FlightLog.seek(FlightLogPosition)) {
        Serial.println("Flight log seek() failed");
        FlightLog.close();
        FlightLogFail = true;
        return;
    }
}

size_t write_g_record(MD5_CTX *md5, size_t data_position)
{
    MD5::MD5Final(md5);
    char *digest = MD5::make_digest(md5);
    if (strlen(digest) == 32) {
        char *buf = &data_block_buf[data_position];
        buf[0] = 'G';
        strncpy(&buf[1], digest, 16);
        strcpy(&buf[17], "\r\nG");
        strncpy(&buf[20], digest+16, 16);
        strcpy(&buf[36], "\r\n");
    }
    return (data_position + G_RECORD_SIZE);
}

void MD5_update(const char *data, size_t size)
{
    MD5::MD5Update(md5_a, data, size);
    MD5::MD5Update(md5_b, data, size);
    MD5::MD5Update(md5_c, data, size);
    MD5::MD5Update(md5_d, data, size);
}

void compressblock(size_t insize)
{
#if defined(ESP32)
    // output to unused space in PSRAMbuf, then write to flash all at once
    // - since PSRAMbuf is much larger than SPIFFS there will always be space, but check:
    if (PSRAMbufUsed + insize + 1024 > PSRAMbufSize)
        return;
    char *outbuf = PSRAMbuf + PSRAMbufUsed;
#elif defined(ARDUINO_ARCH_NRF52)
    char *outbuf = (char *) malloc(insize+400);
    if (! outbuf) {
        failFlightLog();
        Serial.println("Cannot allocate compression buffer");
        return;
    }
#endif

    // process the 'insize' bytes at data_block_buf
    char *p = data_block_buf;
    char *t = data_block_buf + insize;
    size_t outsize = 0;
    bool compress = true;
    int i;
    uint8_t b[4];
    while (p < t) {
#if defined(ARDUINO_ARCH_NRF52)
        if (outsize > insize+350) {  // should not happen
            free(outbuf);
            failFlightLog();
            Serial.println("Compression buffer overflow");
            return;
        }
#endif
        char *q = p;       // look ahead into record
        char c = *p++;
        if (compress) {    // beginning of a record
            if (c == 'B') {
                // compare with previous B-record
                // output any needed template changes
                for (i=0; i<22; i++) {
                    int pos = tpos[i];
                    if (q[pos] != compbrecord[pos]) {
                        if (q[pos] == compbrecord[pos] + 1) {
                            outbuf[outsize++] = (0xA0 | i);
                        } else if (q[pos] == compbrecord[pos] - 1) {
                            outbuf[outsize++] = (0xE0 | i);
                        } else {
                            outbuf[outsize++] = (0xC0 | i);
                            outbuf[outsize++] = q[pos];
                        }
                        compbrecord[pos] = q[pos];
                    }
                }
                // format and output the compressed B-record
                i = 0;
                while (i < 12) {
                    uint8_t c1 = (uint8_t) q[bpos[i++]];
                    uint8_t c2 = (uint8_t) q[bpos[i++]];
                    outbuf[outsize++] = ((c1-'0') | ((c2-'0')<<4));
                }
                p = q + 37;    // past the \r\n
            } else if (c == 'L') {    // LPLT comment lines
                outbuf[outsize++] = 0x0C;
                p += 3;    // past the "LPLT"
                compress = false;
            } else {   // neither B nor L - it is A, H, I, G, etc
                outbuf[outsize++] = 0x0A;
                outbuf[outsize++] = c;
                compress = false;
            }
        } else {  // if not compress, then pass verbatim
            if (c != '\r') {             // drop the \r
                outbuf[outsize++] = c;
                if (c == '\n')
                    compress = true;     // end of the line
            }
        }
    }
    yield();

    // write data block to file all at once
    if (compfile.write((uint8_t *)outbuf, outsize) < outsize) {  // failed
        compfileOpen = false;
#if defined(ARDUINO_ARCH_NRF52)
        failFlightLog();
#endif
        Serial.println("write to flash flight log failed");
    } else {
        compfilePosition += (outsize - 4*G_RECORD_SIZE);  // overwrite this G record next time
        Serial.print("Wrote ");
        Serial.print(outsize);
        Serial.print(" bytes to compressed flight log in flash, total ");
        Serial.println(compfilePosition + 4*G_RECORD_SIZE);
    }
#if defined(ARDUINO_ARCH_NRF52)
    free(outbuf);
#endif
    yield();
}

void completeFlightLog()
{
    if (data_block_used == 0)
        return;
    if (! FlightLogFail)
        reopenFlightLog();
    if (FlightLogFail)
        return;
#if defined(ESP32)
    if (PSRAMbuf) {
        if (FlightLogPosition + data_block_used + 4*G_RECORD_SIZE + 1 > PSRAMbufSize) {
            // if not enough PSRAM buffer space to write the cache, then stop here
            // - the file is complete from the previous flush
            failFlightLog();
            return;
        }
    }
#endif

    // finalize a *copy* of the MD5 context - keep the main context un-finalized
    *md5_a_copy = *md5_a;
    *md5_b_copy = *md5_b;
    *md5_c_copy = *md5_c;
    *md5_d_copy = *md5_d;
    size_t size = data_block_used;
    // append G-records within the data_block_buf
    // - this may spill into the pre-positioning area
    size = write_g_record(md5_a_copy, size);
    size = write_g_record(md5_b_copy, size);
    size = write_g_record(md5_c_copy, size);
    size = write_g_record(md5_d_copy, size);

    // send the new data (including the new G-record) to file and/or PSRAM
    // then (temporarily) close the file
#if defined(ESP32)
    if (PSRAMbuf)
#elif defined(ARDUINO_ARCH_NRF52)
    if (settings->compflash)
#endif
    {
        if (compfileOpen) {
            compressblock(size);   // compress data and write to compfile
            compfile.close();
        }
#if defined(ESP32)
        memcpy(PSRAMbuf+FlightLogPosition, data_block_buf, size);  // overwrite old G-record
        PSRAMbufUsed = FlightLogPosition + size;  // includes the new G-record
#endif
    } else {
        if (FlightLog.write((const uint8_t*) data_block_buf, size) < size) {
            failFlightLog();
            return;
        }
        FlightLog.close();
    }
    FlightLogPosition += data_block_used;  // not including the G-record
    data_block_used = 0;
    Serial.print("Flight log updated, size now: ");
    Serial.println(FlightLogPosition);
    yield();
}

// common code to the functions below
void igc_file_append(const char *data, size_t size)
{
    if (data_block_used + size > DATA_BLOCK_SIZE) {
        // flush the block before writing the new data
        completeFlightLog();
        if (FlightLogFail)
            return;
    }
    memcpy(&data_block_buf[data_block_used], data, size);
    data_block_used += size;
    //Serial.print("data_block_used = ");
    //Serial.println(data_block_used);
}

// assume chars are IGC valid, input must be const
void igc_file_append_const(const char *data)
{
    if (! FlightLogOpen)
        return;
    size_t size = strlen(data);
    if (size < 3)
        return;   // should not happen
    igc_file_append(data, size);         // include \r\n
    MD5_update(data, size-2);            // exclude \r\n
    yield();
}

// this checks and corrects the chars, they cannot be const
// by default commas are replaced with spaces in both the IGC file and MD5
void igc_file_append_nonconst(char *data, bool keepcommas=false, bool iscomment=false)
{
    if (! FlightLogOpen)
        return;
    char *p = data;
    size_t size = 0;
    for (; *p; ++p) {
        if (*p == '\r')
            break;
        if (!keepcommas || *p != 0x2C)     // if (keepcommas) leave commas in file
            *p = clean_igc_char(*p);
        ++size;
    }
    if (size == 0)
        return;   // should not happen
    igc_file_append(data, size+2);     // include \r\n
    if (iscomment)
        return;                       // skip MD5 calc
    if (keepcommas) {
        char *q = data;
        for (p=data; *p; ++p) {
            if (*p == '\r')
                break;
            if (*p != 0x2C) {
                *q++ = *p;             // *skip* the commas in MD5 calculation
            } else {
                --size;
                if (size == 0)
                    return;
            }
        }
    }
    MD5_update(data, size);            // exclude \r\n
    yield();
}

// this checks and corrects the chars, they cannot be const
// commas are kept in the file but skipped in the MD5 calc
void igc_file_append_commas(char *data)
{
    igc_file_append_nonconst(data, true, false);
}

// this checks and corrects the chars, they cannot be const
// this excludes the (LPLT) comment from the MD5 calculation
void igc_file_append_comment(char *data)
{
    igc_file_append_nonconst(data, true, true);
}

void closeFlightLog()
{
    if (! FlightLogOpen)         // no log file in process
        return;
    if (FlightLogFail)
        return;
    completeFlightLog();         // can still be extended
    FlightLogOpen = false;
    FlightLogClosed = millis();
    //FlightLogPath[0] = '\0';   // leave intact for Web.cpp/flightlogfile()
    if (FlightLogFail)
        Serial.println("Error closing flight log");
    else
        Serial.println("Flight log closed OK");
}

/*
Short file name style: YMDCXXXF.IGC
Y = Year; value 0 to 9, cycling every 10 years
M = Month; value 1 to 9 then A for 10, B=11, C=12.
D = Day; value 1 to 9 then A=10, B=11, C=12, ...
C = manufacturer's IGC code letter
XXX = unique FR Serial Number; 3 alphanumeric characters
F = Flight number of the day; 1 to 9 then A=10 through Z=35
*/

void makeLogNameDate(char *p)
{
    uint16_t year = gnss.date.year() - 2000;
    char c = ((uint8_t)'0' + (uint8_t)(year % 10));
    *p++ = c;
    uint8_t month = gnss.date.month();
    if (month < 10)
        c = ((uint8_t)'0' + month);
    else
        c = ((uint8_t)'A' + (month-10));
    *p++ = c;
    uint8_t day = gnss.date.day();
    if (day < 10)
        c = ((uint8_t)'0' + day);
    else
        c = ((uint8_t)'A' + (day-10));
    *p++ = c;
}

void makeFlightLogName()
{
    char buf[8];
    makeLogNameDate(buf);
    buf[3] = 'X';
    String chip = String((SoC->getChipId() & 0x0FFF), HEX);
    strncpy(buf+4,chip.c_str(),4);
    strupr(buf+4);
#if defined(ESP32)
    String basename = "/logs/";   // SD_BASEPATH;
    if (PSRAMbuf)
        basename = "/";
#else
    String basename = "/";  // BASEPATH;
#endif
    basename += buf;
    int flightnum = 1;
    char flightn = '1';
    String filename = basename;
    filename += flightn;
    filename += ".IGC";
    strcpy(compfilename, filename.c_str());
    compfilename[12] = 'Z';   // .IGZ means compressed
#if defined(ESP32)
    const char *fn = (PSRAMbuf? compfilename : filename.c_str());
    while (PSRAMbuf? SPIFFS.exists(fn) : SD.exists(fn))
#else
    const char *fn = (settings->compflash? compfilename : filename.c_str());
    while (IGCFILESYS.exists(filename.c_str()) || IGCFILESYS.exists(compfilename))
#endif
    {
        Serial.print(fn);
        Serial.println(" already exists");     // compressed and/or not
        ++flightnum;
        if (flightnum < 10) {
            ++flightn;
        } else if (flightnum < 36) {
            flightn = ('A' + (flightnum-10));
        } else {
            // overwrite this one
#if defined(ESP32)
            if (PSRAMbuf)
                SPIFFS.remove(fn);
            else
                SD.remove(fn);
#else
            IGCFILESYS.remove(filename.c_str());  // delete compressed
            IGCFILESYS.remove(compfilename);      // delete uncompressed
#endif
            break;
        }
        filename = basename;
        filename += flightn;
        filename += ".IGC";
        compfilename[8] = flightn;
    }
    strcpy(FlightLogPath, filename.c_str());
    Serial.print("New flight log: ");
    Serial.println(FlightLogPath);
}

bool writeIGCHeader()
{
    if (! FlightLogOpen)
        return false;

    char buf[80];
    Serial.println(F("Writing IGC Header..."));
    snprintf(buf, 80, "AXLK%06X\r\n", (SoC->getChipId() & 0x00FFFFFF));
    strupr(buf);      // redundant since "%06X" above?
    igc_file_append_nonconst(buf);    // 6 hex digits (only 3 in file name)
    int y = gnss.date.year() - 2000;
    snprintf(buf, 80, "HFDTE%02d%02d%02d\r\n", gnss.date.day(), gnss.date.month(), y);
    igc_file_append_nonconst(buf);
    igc_file_append_const("HFFXA035\r\n");
    snprintf(buf, 80, "HFPLTPILOTINCHARGE: %s\r\n", settings->igc_pilot);
    Serial.print(buf);
    igc_file_append_commas(buf);
    snprintf(buf, 80, "HFGTYGLIDERTYPE: %s\r\n", settings->igc_type);
    Serial.print(buf);
    igc_file_append_commas(buf);
    snprintf(buf, 80, "HFGIDGLIDERID: %s\r\n", settings->igc_reg);
    Serial.print(buf);
    igc_file_append_commas(buf);
    snprintf(buf, 80, "HFCIDCOMPETITIONID: %s\r\n", settings->igc_cs);
    Serial.print(buf);
    igc_file_append_commas(buf);
    //printf("HFCCLCOMPETITIONCLASS: %s\r\n",____);
    igc_file_append_const("HFFTYFRTYPE: SoftRF\r\n");
    snprintf(buf, 80, "HFRFWFIRMWAREVERSION: %s\r\n", SOFTRF_FIRMWARE_VERSION);
    //Serial.print(buf);
    igc_file_append_nonconst(buf);
#if defined(ESP32)
    igc_file_append_const("HFRHWHARDWAREVERSION: T-Beam\r\n");
#elif defined(ARDUINO_ARCH_NRF52)
    igc_file_append_const("HFRHWHARDWAREVERSION: T-Echo\r\n");
#endif
    //igc_file_append_const("HFGPSRECEIVER: Generic\r\n");
    if (baro_chip != NULL) {
        strcpy(buf, "HFPRSPRESSALTSENSOR: Bosch,BME280,9163m\r\n");
        igc_file_append_commas(buf);
        igc_file_append_const("HFALPALTPRESSURE:ISA\r\n");
//    } else {
//        igc_file_append_const("HFALPALTPRESSURE:ISA\r\n");
          // we output a copy of the GNSS ellipsoid altitude, no neither NIL nor ISA?
    }
    //igc_file_append_const("HFALGALTGPS:GEO\r\n");   // for non IGC loggers that output geoid alt
    igc_file_append_const("HFALGALTGPS:ELL\r\n");     // but we output the ellipsoid altitude
    igc_file_append_const("HFDTM100GPSDATUM: WGS-1984\r\n");
    igc_file_append_const("I00\r\n");   // >>> may add FXA etc later
    return true;  
}

void init_md5()
{
#define ul (unsigned long)
    // LK8000 keys:
    MD5::MD5Initialize(md5_a, ul 0x63e54c01, ul 0x25adab89, ul 0x44baecfe, ul 0x60f25476);
    MD5::MD5Initialize(md5_b, ul 0x41e24d03, ul 0x23b8ebea, ul 0x4a4bfc9e, ul 0x640ed89a);
    MD5::MD5Initialize(md5_c, ul 0x61e54e01, ul 0x22cdab89, ul 0x48b20cfe, ul 0x62125476);
    MD5::MD5Initialize(md5_d, ul 0xc1e84fe8, ul 0x21d1c28a, ul 0x438e1a12, ul 0x6c250aee);
#undef ul
}

// XCsoar keys:
//  { 0x1C80A301,0x9EB30b89,0x39CB2Afe,0x0D0FEA76 },
//  { 0x48327203,0x3948ebea,0x9a9b9c9e,0xb3bed89a },
//  { 0x67452301,0xefcdab89,0x98badcfe,0x10325476 },
//  { 0xc8e899e8,0x9321c28a,0x438eba12,0x8cbe0aee },

// if need space, erase oldest flight log
bool eraseOldestFlightLog()
{
    String oldestlog = "~";
    int found = 0;
    // find oldest file by alphabetical order of file name
    File root = FILESYS.open("/");
    if (! root)
        return false;
    File file = root.openNextFile();
    String file_name;
    while(file) {
        file_name = file.name();
        Serial.println(file_name);
        if (file_name.endsWith(".IGC") || file_name.endsWith(".igc")
        ||  file_name.endsWith(".IGZ") || file_name.endsWith(".igz")) {
            ++found;
            if (strcmp(oldestlog.c_str(), file.name()) > 0) {
                oldestlog = file.name();
                Serial.print(F("Candidate oldest log: "));
                Serial.println(oldestlog);
            }
        }
        file = root.openNextFile();
    }
    file.close();
    root.close();
    if (found < 2) {     // do not erase the most recent one
        Serial.println("Found <= 1 file");
        return false;
    }
    Serial.print("Removing ");
    Serial.println(oldestlog);
    oldestlog = "/" + oldestlog;
    FILESYS.remove(oldestlog.c_str());
    return true;
}

bool makeFlightLogSpace()
{
#if defined(ESP32)
    if (PSRAMbuf && settings->compflash) {
        uint32_t needed_kb = 100 / settings->loginterval;
        while (FILESYS_free_kb() < needed_kb) {
            if (eraseOldestFlightLog() == false) {
                Serial.print("cannot free ");
                Serial.print(needed_kb);
                Serial.println(" kb for flight log");
                return false;
            }
        }
    }
#elif defined(ARDUINO_ARCH_NRF52)
    // do not start new log if not enough space for 3 hours of recording
    uint32_t free_kb = (IGCFS_is_mounted? IGCFS_free_kb() : 0);
    uint32_t needed_kb = ((settings->compflash)? 100 : 400);
    needed_kb = 50 + (needed_kb / settings->loginterval);
    while (free_kb < needed_kb) {
        if (eraseOldestFlightLog() == false) {
            Serial.print("cannot free ");
            Serial.print(needed_kb);
            Serial.println(" kb for flight log");
            return false;
        }
    }
#endif
    return true;   // if using SD card, assumes there is room
}

void openFlightLog()
{
    if (FlightLogFail)
        return;
    if (GNSSTimeMarker == 0)
        return;
#if defined(ESP32)
    if (settings->logflight == FLIGHT_LOG_ALWAYS) {
        // pause before starting another log, to allow shutdown instead
        if ((! PSRAMbuf) && FlightLogClosed != 0 && millis() < FlightLogClosed + 120000)
            return;
    }
#elif defined(ARDUINO_ARCH_NRF52)
    if (settings->logflight == FLIGHT_LOG_ALWAYS) {
        // pause before starting another log, to allow shutdown instead
        if (FlightLogClosed != 0 && millis() < FlightLogClosed + 120000)
            return;
    }
#endif

#if defined(ESP32)
    if (PSRAMbufUsed) {       // already have data in PSRAMbuf
        // keep on adding to same "file" - FlightLogPosition kept as it was
        data_block_used = 0;  // was also done when completing the previous "file"
        FlightLogOpen = true;
        return;
    }
#endif

    // get here if no PSRAMbuf or first use of PSRAMbuf

#if defined(ESP32)
    bool have_space = true;
#endif
    if (makeFlightLogSpace() == false) {
#if defined(ESP32)
        have_space = false;    // but log to PSRAM
#else
        failFlightLog();
        return;
#endif
    }

    makeFlightLogName();

#if defined(ESP32)
    if (have_space) {
      if (PSRAMbuf && settings->compflash)
#else
      if (settings->compflash)
#endif
      {
        compfile = FILESYS.open(compfilename, FILE_WRITE);
        if (compfile) {
            compfile.close();      // will be reopen()ed later
            compfileOpen = true;
            compfilePosition = 0;
        } else {
#if defined(ESP32)
            Serial.println("File open failed, logging to PSRAM only");
#else
            Serial.println("Failed to open flight log for writing");
            failFlightLog();
#endif
            return;
        }
        // the B-record needs to be initialized once for the file,
        // afterwards it will be preserved between compressblock() calls
        //strcpy(compbrecord, emptybrecord);
        memset(compbrecord,0,B_RECORD_SIZE);
        compbrecord[0] = 'B';
        compbrecord[35] = '\r';
        compbrecord[36] = '\n';
        //compbrecord[37] = '\0';

      } else {  // ESP32 and writing to SD, or NRF52 and not compressing

        FlightLog = IGCFILESYS.open(FlightLogPath, FILE_WRITE);
        if (! FlightLog) {
            Serial.println("Failed to open flight log for writing");
            failFlightLog();
            return;
        }
        FlightLog.close();      // will be reopen()ed later
      }
#if defined(ESP32)
    } else {
        Serial.println("Logging to PSRAM only");
    }
#endif

    FlightLogPosition = 0;
    data_block_used = 0;
    FlightLogOpen = true;
    init_md5();
    writeIGCHeader();         // to cache
}

void logFlightPosition()
{
#if defined(ESP32)
    if (suspended)
        return;
#endif
    if (! pre_positions_buf)  // FlightLog_setup() has not happened
        FlightLog_setup();
    if (FlightLogFail)
        return;
    if (! FlightLogOpen && settings->logflight == FLIGHT_LOG_ALWAYS)
        openFlightLog();
    // if FLIGHT_LOG_AIRBORNE then Wind.cpp/this_airborne() will open the file on takeoff 

    // >>> testing: simulate takeoff 90 seconds after GPS fix
    //if (! FlightLogOpen && settings->logflight == FLIGHT_LOG_AIRBORNE
    //               && millis() > GNSSTimeMarker + 60000) {
    //    openFlightLog();
    //    Serial.println("90s - open log");
    //}

    static bool prepositioning = false;
    if (! FlightLogOpen) {
        // waiting for takeoff
        // populate a ring buffer of positions
        // to be used to start the flight log once it is opened
        if (FlightLogFail)
            return;
        if (settings->logflight != FLIGHT_LOG_AIRBORNE
         && settings->logflight != FLIGHT_LOG_TRAFFIC)
            return;
        prepositioning = true;
    } else if (prepositioning) {   // but now FlightLogOpen
        // write the stored positions to the newly opened file
        for (int i=0; i<num_pre_positions; i++) {
            igc_file_append_nonconst(pre_positions_buf + (B_RECORD_SIZE * pre_positions_head));
            pre_positions_head = (pre_positions_head + 1) % PRE_POS_NUM;
        }
        prepositioning = false;
        num_pre_positions = 0;     // in case a new flight log is started later
        pre_positions_head = 0;
        pre_positions_next = 0;
    }

    // B1751494352910N07215306WA0021000210\r\n
    // Number of time/lat/lon digits in GGA varies between GNSS module types:
    // $GPGGA,192054.00,4521.2812,N,07547.2398,W,...
    // $GPGGA,235317.00,4003.90395,N,10512.57934,W,...
    // $GNGGA,144935.000,4521.30100,N,07547.22870,W,...
    char *gp = &GPGGA_Copy[7];   // after the "$GPGGA,", start of timestamp
    //if (strlen(gp) < 35) {
    if (*gp == '\0') {           // set in GNSS.cpp
        //Serial.print("Bad GGA skipped");
        //Serial.println(GPGGA_Copy);
        return;
    }
    char *bp = &brecord_format[1];
    *bp++ = *gp++;
    *bp++ = *gp++;
    *bp++ = *gp++;
    *bp++ = *gp++;
    *bp++ = *gp++;
    *bp++ = *gp++;
    while (*gp++ != ',') { if (gp > &GPGGA_Copy[20]) break; }   // seek to start of lat
    *bp++ = *gp++;   // degrees
    *bp++ = *gp++;
    *bp++ = *gp++;   // minutes
    *bp++ = *gp++;
    if (*gp != '.') {
        Serial.print("Bad GGA: ");
        Serial.println(GPGGA_Copy);
        return;
    }
    ++gp;            // skip the period
    *bp++ = *gp++;
    *bp++ = *gp++;
    *bp++ = *gp++;
    while (*gp++ != ',') { if (gp > &GPGGA_Copy[32]) break; }   // seek to N or S
    if (*gp != 'N' && *gp != 'S') {
        Serial.print("Bad GGA: ");
        Serial.println(GPGGA_Copy);
        return;
    }
    *bp++ = *gp++;
    ++gp;            // skip the comma, to start of lon
    *bp++ = *gp++;   // degrees
    *bp++ = *gp++;
    *bp++ = *gp++;
    *bp++ = *gp++;   // minutes
    *bp++ = *gp++;
    if (*gp != '.') {
        Serial.print("Bad GGA: ");
        Serial.println(GPGGA_Copy);
        return;
    }
    ++gp;            // skip the period
    *bp++ = *gp++;
    *bp++ = *gp++;
    *bp++ = *gp++;
    while (*gp++ != ',') { if (gp > &GPGGA_Copy[NMEA_BUFFER_SIZE-2]) break; }   // seek to E or W
    if (*gp != 'E' && *gp != 'W') {
        Serial.print("Bad GGA: ");
        Serial.println(GPGGA_Copy);
        return;
    }
    *bp++ = *gp;

    int galt = (int) ThisAircraft.altitude;   // assumes this is height above ellipsoid
    int palt;
    if (baro_chip != NULL /* && ! (settings->debug_flags & DEBUG_SIMULATE) */ )
        palt = (int) ThisAircraft.pressure_altitude;
    else
        palt = galt;

    char buf[B_RECORD_SIZE];
    char *pp = buf;
    if (prepositioning)  pp = pre_positions_buf + (B_RECORD_SIZE * pre_positions_next);
    snprintf(pp, B_RECORD_SIZE, brecord_format, palt, galt);

    if (prepositioning) {
        pre_positions_next = (pre_positions_next + 1) % PRE_POS_NUM;
        if (num_pre_positions < PRE_POS_NUM)
            ++num_pre_positions;
        else  // table was already full, overwrote the old head
            pre_positions_head = (pre_positions_head + 1) % PRE_POS_NUM;
    } else {
        igc_file_append_nonconst(buf);
        //igc_file_append_const(pp);  if can skip the checking of chars
    }
}

// must be given a null-terminated string
void FlightLogComment(const char *data)
{
    if (FlightLogFail)
        return;
    if (! FlightLogOpen)
        return;
    char buf[80];
    strcpy(buf, "LPLT");
    size_t len = strlen(data);
    if (len > 72)  len = 72;   // IGC format limits lines to 76 chars
    strncpy(buf+4, data, len);
    len += 4;
    if (buf[len-2] != '\r') {
        buf[len++] = '\r';
        buf[len++] = '\n';
    }
    buf[len] = '\0';
    //if (secure)
    //    igc_file_append_commas(buf)
    //else
    igc_file_append_comment(buf);
}

#if defined(ESP32)
// generate a message for display (in the web interface)
const char *FlightLogStatus()
{
    bool logging = (settings->logflight != FLIGHT_LOG_NONE);
    static char buf[20];
    snprintf(buf, 20, "Logged %d bytes", FlightLogPosition);
    if (PSRAMbuf) {
        if (FlightLogPosition)
            return buf;
        else if (FlightLogOpen)
            return "Logging to RAM";
        else if (FlightLogFail)
                return "RAM is full";
        else if (logging)
            return "No log yet";
    } else {
#if defined(USE_SD_CARD)
        if (SD_is_mounted) {
            if (FlightLogPosition)
                return buf;
            else if (FlightLogOpen)
                return "Logging to SD card";
            else if (FlightLogFail)
                return "SD card error";
            else if (logging)
                return "SD card found";
        } else {                          // should not happen
            if (FlightLogFail)
                return "RAM not available";
            else if (logging)
                return "SD card not found";
        }
#else
        if (FlightLogPosition)
            return buf;
        else if (FlightLogOpen)
            return "Logging...";
        else if (FlightLogFail)
                return "File space full";
#endif
    }
    return "Not logging";
}
#endif

#if defined(ESP32)

size_t PSRAMavailable()
{
    return (PSRAMbufSize - PSRAMbufUsed);
}

// pause logging, and point to unused portion of PSRAMbuf
void suspendFlightLog()
{
    completeFlightLog();    // if logging in progress - now data_block_buf is available
    compfileOpen = false;   // can continue PSRAM logging, but stop writing to SPIFFS
    PSRAMbuf2 = PSRAMbuf;
    PSRAMbufSize2 = PSRAMbufSize;
    PSRAMbufUsed2 = PSRAMbufUsed;
    if (! PSRAMbuf) {
        // there is an SD card, but trying to decompress a leftover log in SPIFFS
        size_t size = 1000000;
        PSRAMbuf = (char *) malloc(size);
        if (! PSRAMbuf) {
            Serial.println("Failed to allocate PSRAM for decomp");
            return;
        }
        PSRAMbufSize = size;
        PSRAMbufUsed = 0;
    }
    PSRAMbuf += PSRAMbufUsed;
    PSRAMbufSize -= PSRAMbufUsed;
    PSRAMbufUsed = 0;
    suspended = true;
}

// pause logging, and point to unused portion of PSRAMbuf
void resumeFlightLog()
{
    if (! PSRAMbuf2)  // was only malloc()ed for the decompression
        free(PSRAMbuf);
    PSRAMbuf = PSRAMbuf2;
    PSRAMbufSize = PSRAMbufSize2;
    PSRAMbufUsed = PSRAMbufUsed2;
    suspended = false;
}

void clearPSRAMlog()
{
    FlightLogPosition = 0;
    PSRAMbufUsed = 0;
    FlightLogOpen = false;
    compfileOpen = false;
    compfilePosition = 0;
}

#endif  // ESP32

// run a test of MD5 machinery
// this processes the text as-is, including commas
void test_final(MD5_CTX *md5)
{
    MD5::MD5Final(md5);
    char *digest = MD5::make_digest(md5);
    char buf[40];
    buf[0] = 'G';
    strncpy(&buf[1], digest, 16);
    strcpy(&buf[17], "\r\nG");
    strncpy(&buf[20], digest+16, 16);
    strcpy(&buf[36], "\r\n");
    Serial.print(buf);
}
void MD5_test()
{
    if (! (settings->debug_flags & DEBUG_SIMULATE))  return;
    if (FlightLogFail)  return;
#if defined(ESP32)
    if (SD_is_mounted == false)  return;
#endif
    //MD5::make_hash(md5_a, "abcdefghijklmnopqrstuvwxyz");
    //Serial.println("MD5 should be:   c3fcd3d76192e4007dfb496cca67e13b");
    //Serial.print("MD5 computed as: ");
    //Serial.println(MD5::make_digest(md5_a));
    init_md5();
#if defined(ESP32)
    String filename = "/logs/";   // SD_BASEPATH;
#else
    String filename = "/";  // BASEPATH;
#endif
    filename += "TEST.TXT";
    File file = IGCFILESYS.open(filename.c_str(), FILE_READ);
    if (! file)  return;
    Serial.println("MD5 test:");
    const char *cp;
    char buf[128];
    while (getline(file, buf, sizeof(buf))) {
        MD5_update(buf, strlen(cp));
        yield();
    }
    file.close();
    test_final(md5_a);
    test_final(md5_b);
    test_final(md5_c);
    test_final(md5_d);
}

