/****************************************************************************
*  Copyright (C) SIEMENS CORP., 2013 All rights reserved.
*****************************************************************************
* FILE NAME    : ldah_flash.h
* ---------------------------------------------------------------------------
* DESCRIPTION  : flash management functions
*****************************************************************************
* Diese Software ist Freeware. Sie wird Ihnen unentgeltlich zur Verfuegung  *
* gestellt. Sie darf frei kopiert, modifiziert und benutzt sowie an Dritte  *
* weitergegeben werden. Die Software darf nur unter Beibehaltung aller      *
* Schutzrechtsvermerke sowie nur vollstaedig und unveraendert weitergegeben *
* werden. Die kommerzielle Weitergabe an Dritte (z.B. im Rahmen von         *
* Share-/Freeware-Distributionen) ist nur mit vorheriger schriftlicher      *
* Genehmigung der Siemens Aktiengesellschaft erlaubt.                       *
* DA DIE SOFTWARE IHNEN UNENTGELTLICH UEBERLASSEN WIRD, KOENNEN DIE AUTOREN *
* UND RECHTSINHABER DAFUER KEINE HAFTUNG UEBERNEHMEN. IHRE BENUTZUNG ERFOLGT*
* AUF EIGENE GEFAHR UND VERANTWORTUNG. DIE AUTOREN UND RECHTSINHABER HAFTEN *
* NUR FUER VORSATZ UND GROBE FAHRLAESSIGKEIT. WEITERGEHENDE ANSPRUECHE SIND *
* AUSGESCHLOSSEN. INSBESONDERE HAFTEN DIE AUTOREN UND RECHTSINHABER NICHT   *
* FUER ETWAIGE MAENGEL ODER FOLGESCHAEDEN.                                  *
* Falls Sie Fehler in der Software bemerken, teilen Sie es uns bitte mit.   *
*                                                                           *
* This Software is Freeware. It is distributed for free. You may copy,      *
* modify and use it for free as well as distribute it to others. You may    *
* only distribute it as a whole, unmodified and with all trademarks and     *
* copyright notices. You may not distribute it commercially (e.g. as a      *
* Share-/Freeware-Distributor) without the express written permission of    *
* Siemens Aktiengesellschaft. SINCE THIS SOFTWARE IS DISTRIBUTED FOR FREE,  *
* IT IS PROVIDED "AS IS" WITHOUT ANY REPRESENTATION OR WARRANTY OF ANY KIND *
* EITHER EXPRESSED OR IMPLIED INCLUDING BUT NOT LIMITED TO IMPLIED          *
* WARRANTIES FOR MERCHANTIBILITY OR FITNESS FOR USE. ANY USE OF THE         *
* APPLICATION EXAMPLE IS ON YOUR OWN RISK AND RESPONSIBILITY.               *
* If you happen to find any faults in it please tell us.                    *
*****************************************************************************/

#ifndef _FLASH_H
#define _FLASH_H

/* Size of the flash and it's sector */
#define F_SIZE  0x800000
#define S_SIZE  0x10000


#define FLASH_DEF	         DPR_UINT16
#define FLASH_WIDTH                 2


/* flash device codes */
#define FLASH_29LV6410H                 0xD7            /* 29LV6410H */
#define FLASH_29GL064N90TFI06           0x7E            /* 29GL064N90TFI06 */


/* FLASH29 command definitions */
#define FLASH_CMD_FIRST               (FLASH_DEF)0xaaaaaaaa
#define FLASH_CMD_SECOND              (FLASH_DEF)0x55555555
#define FLASH_CMD_FOURTH              (FLASH_DEF)0xaaaaaaaa
#define FLASH_CMD_FIFTH               (FLASH_DEF)0x55555555
#define FLASH_CMD_SIXTH               (FLASH_DEF)0x10101010
#define FLASH_CMD_SECTOR              (FLASH_DEF)0x30303030

#define FLASH_CMD_PROGRAM             (FLASH_DEF)0xa0a0a0a0
#define FLASH_CMD_CHIP_ERASE          (FLASH_DEF)0x80808080
#define FLASH_CMD_READ_RESET          (FLASH_DEF)0xf0f0f0f0
#define FLASH_CMD_AUTOSELECT          (FLASH_DEF)0x90909090


/*  FLASH29 command register addresses */
#define FLASH_REG_FIRST_CYCLE         0x5555
#define FLASH_REG_SECOND_CYCLE        0x2aaa


#define Q7(ix)          ((ix & 0x80) >> 7)      /* Check DQ7 bit */
#define Q5(ix)          ((ix & 0x20) >> 5)      /* Check DQ5 bit */


#define STATUS          DPR_UINT32
#define STATUS_OK       0x00
#define STATUS_ERROR    0x01
#define STATUS_ERROR_TIMOUT    0x02

/* Start adress of different parts in flash*/
#define BL_START_OFFSET		    0x00000   /* Bootloader */
#define FW_START_OFFSET		    0x80000   /* Firmware */
#define FLASH_FS_OFFSET       0x5b0000  /* flash fs area */
#define SNMP_START_OFFSET	    0x5d0000  /* SNMP Data */
#define OVS1_START_OFFSET		  0x5e0000  /* OVS1 Data */
#define OVS4_START_OFFSET     0x7d0000  /* OVS4 Data */
#define OVS2_START_OFFSET     0x7e0000  /* OVS2 Data */
#define SERIAL_START_OFFSET   0x7f0000  /* Serial Data */

#define LAD_FILE_OFFSET			12
#ifdef __cplusplus
extern "C" {
#endif

/* Function declarations */
STATUS flashPoll(volatile FLASH_DEF *, FLASH_DEF );
STATUS flashSectorErase(DPR_CHAR *mapStart, int);
STATUS flashErase(DPR_CHAR *mapStart);
STATUS flashEraseSectors(DPR_CHAR *, int, int, const char *);
STATUS flashWrite(DPR_CHAR *mapStart, FLASH_DEF *, int, int);
DPR_CHAR flashTypeGet(DPR_CHAR *mapStart);
DPR_VOID flashReset(DPR_CHAR *mapStart);

#ifdef __cplusplus
}
#endif

#endif /* _FLASH_H */
