#if !defined(_RADIOLIB_USER_BUILD_OPTIONS_H)
#define _RADIOLIB_USER_BUILD_OPTIONS_H

// this file can be used to define any user build options
// most commonly, RADIOLIB_EXCLUDE_* macros
// or enabling debug output

//#define RADIOLIB_DEBUG_BASIC        (1)   // basic debugging (e.g. reporting GPIO timeouts or module not being found)
//#define RADIOLIB_DEBUG_PROTOCOL     (1)   // protocol information (e.g. LoRaWAN internal information)
//#define RADIOLIB_DEBUG_SPI          (1)   // verbose transcription of all SPI communication - produces large debug logs!
//#define RADIOLIB_VERBOSE_ASSERT     (1)   // verbose assertions - will print out file and line number on failure


// MB: exclude all modules not explicitly included.  For this version of SoftRF:
// - compiling for ESP32 boards will include sx1276 & sx1262 (T-Beam)
// - compiling for nRF52 boards will include sx1262 (T-Echo) and LR1110 (T1000E, Thinknode M3)
// - some future boards will hopefully include the LR20xx

#if defined(ESP32)
#define INCLUDE_SX127X                    (1)
#define INCLUDE_SX126X                    (1)
#define RADIOLIB_EXCLUDE_STM32WLX         (1)
#endif

#if defined(ARDUINO_ARCH_NRF52)
#define INCLUDE_SX126X                    (1)
#define INCLUDE_LR11X0                    (1)
#endif


#if !defined(INCLUDE_CC1101)
#define RADIOLIB_EXCLUDE_CC1101           (1)
#endif

#if !defined(INCLUDE_NRF24)
#define RADIOLIB_EXCLUDE_NRF24            (1)
#endif

#if !defined(INCLUDE_RF69)
#define RADIOLIB_EXCLUDE_RF69             (1)
#define RADIOLIB_EXCLUDE_SX1231           (1) // dependent on RADIOLIB_EXCLUDE_RF69
#endif

#if !defined(INCLUDE_SI443X)
#define RADIOLIB_EXCLUDE_SI443X           (1)
#define RADIOLIB_EXCLUDE_RFM2X            (1) // dependent on RADIOLIB_EXCLUDE_SI443X
#endif

#if !defined(INCLUDE_SX127X)
#define RADIOLIB_EXCLUDE_SX127X           (1)
#endif

#if !defined(INCLUDE_SX126X)
#define RADIOLIB_EXCLUDE_SX126X           (1)
#define RADIOLIB_EXCLUDE_STM32WLX         (1) // dependent on RADIOLIB_EXCLUDE_SX126X
#endif

#if !defined(INCLUDE_SX128X)
#define RADIOLIB_EXCLUDE_SX128X           (1)
#endif

#if !defined(INCLUDE_LR11X0)
#define RADIOLIB_EXCLUDE_LR11X0           (1)
#endif

#if !defined(INCLUDE_LR2021)
#define RADIOLIB_EXCLUDE_LR2021           (1)
#endif

#endif
