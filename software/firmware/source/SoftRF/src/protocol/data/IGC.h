#ifndef IGC_H
#define IGC_H

#if defined(IGCFILESYS)

void makeLogNameDate(char *buf);  // needs buf[4]
void FlightLog_setup();
void openFlightLog();
size_t PSRAMavailable();
void clearPSRAMlog();
void suspendFlightLog();
bool decompressfile(char *filename);
void resumeFlightLog();
void logFlightPosition();
void completeFlightLog();
void closeFlightLog();
const char *FlightLogStatus();
void FlightLogComment(const char *data);
void MD5_test();

#if defined(ARDUINO_ARCH_NRF52)
void FlightLog_decomp();
#endif

extern bool FlightLogOpen;
extern char FlightLogPath[];

extern char *PSRAMbuf;
extern size_t PSRAMbufUsed;

#endif

#endif
