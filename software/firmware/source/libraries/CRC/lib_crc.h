    /*******************************************************************\
    *                                                                   *
    *   Library         : lib_crc                                       *
    *   File            : lib_crc.h                                     *
    *   Author          : Lammert Bies  1999-2008                       *
    *   E-mail          : info@lammertbies.nl                           *
    *   Language        : ANSI C                                        *
    *                                                                   *
    *                                                                   *
    *   Description                                                     *
    *   ===========                                                     *
    *                                                                   *
    *   The file lib_crc.h contains public definitions  and  proto-     *
    *   types for the CRC functions present in lib_crc.c.               *
    *                                                                   *
    *                                                                   *
    *   Dependencies                                                    *
    *   ============                                                    *
    *                                                                   *
    *   none                                                            *
    *                                                                   *
    *                                                                   *
    *   Modification history                                            *
    *   ====================                                            *
    *                                                                   *
    *   Date        Version Comment                                     *
    *                                                                   *
    *   2026-05-04 by Moshe Braner: added table-driven ADS-L CRC        *
    *                                                                   *
    *   2008-04-20  1.16    Added CRC-CCITT routine for Kermit          *
    *                                                                   *
    *   2007-04-01  1.15    Added CRC16 calculation for Modbus          *
    *                                                                   *
    *   2007-03-28  1.14    Added CRC16 routine for Sick devices        *
    *                                                                   *
    *   2005-12-17  1.13    Added CRC-CCITT with initial 0x1D0F         *
    *                                                                   *
    *   2005-02-14  1.12    Added CRC-CCITT with initial 0x0000         *
    *                                                                   *
    *   2005-02-05  1.11    Fixed bug in CRC-DNP routine                *
    *                                                                   *
    *   2005-02-04  1.10    Added CRC-DNP routines                      *
    *                                                                   *
    *   2005-01-07  1.02    Changes in tst_crc.c                        *
    *                                                                   *
    *   1999-02-21  1.01    Added FALSE and TRUE mnemonics              *
    *                                                                   *
    *   1999-01-22  1.00    Initial source                              *
    *                                                                   *
    \*******************************************************************/



#define CRC_VERSION     "1.16+"

#define FALSE           0
#define TRUE            1

uint16_t  update_crc_16(     uint16_t crc, char c                 );
uint32_t  update_crc_32(     uint32_t crc, char c                 );
uint16_t  update_crc_ccitt(  uint16_t crc, char c                 );
uint16_t  update_crc_dnp(    uint16_t crc, char c                 );
uint16_t  update_crc_kermit( uint16_t crc, char c                 );
uint16_t  update_crc_sick(   uint16_t crc, char c, char prev_byte );
uint16_t  update_crc_gdl90(  uint16_t crc, char c                 );

void      update_crc8( uint8_t *crc, uint8_t m );

uint32_t  check_adsl_crc(const uint8_t *data, uint8_t size);
uint32_t  calc_adsl_crc(const uint8_t *data, uint8_t size);
