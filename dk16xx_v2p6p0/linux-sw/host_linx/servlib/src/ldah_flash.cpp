/****************************************************************************
*  Copyright (C) SIEMENS CORP., 2013 All rights reserved.
*****************************************************************************
* FILE NAME    : ldah_flash.cpp
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

#include "os.h"
#include "ldah_flash.h"
#include "servusrx.h"
#include "tracemcrm.h"
extern CTrmWrapper ti;


/**
 * flashPoll - wait for a flash device operation to complete
 *
 * This routine polls a specified address on a flash device
 * until the device operation at that location completes or times out.
 *
 * While a flash operation is in progress, a read on the device
 * returns on Q7 (data bit 7) the complement of the previous value of Q7.  Once
 * the flash operation has completed, the Q7 bit returns the true data
 * of the last write. Subsequent reads return the correct data in Q0-7.
 *
 * The Q5 bit implements a timeout functionality.  When a currently
 * executing flash operation exceeds specified limits, the Q5 bit is set (to 1).
 *
 * RETURNS: STATUS_OK, or STATUS_ERROR if the timeout (!Q5) occurs before the device operation
 * completes.
 **/
STATUS flashPoll(volatile FLASH_DEF *pFA, FLASH_DEF value)
{
  STATUS retVal = STATUS_OK;
  volatile FLASH_DEF * pTest = (FLASH_DEF *) pFA;
  volatile FLASH_DEF * pVal  = (FLASH_DEF *) &value;
  int ix;         /* byte counter */
  int vBit;       /* programmed value of DQ7 */
  time_t ltime;		/* timeout */
  time_t xtime;		/* timeout */
  time_t ltime_start; /* timeout */
  TRM_OUT02(&ti, GR_FL, LV_FCTINT, "> flashPoll pFA=%#x value=%#x",pFA,value);


  for(ix = 0/*(FLASH_WIDTH/FLASH_CHIP_WIDTH - 1)*/; (ix >= 0 ) && (retVal == STATUS_OK);
     ix--, pTest++, pVal++) 
  {
	//TRM_OUT01(&ti, GR_INIT, LV_INFO, "flashPoll ix = %d",ix);
	int l=0;
    vBit = Q7(*pVal);
	time( &ltime_start ); /* remember start time before loop */
	
    while(Q7(*pTest) != vBit) 
	{
		time( &ltime ); /* actual time */
		xtime = ltime - ltime_start;
		if (( l == 0 ) && ( xtime > 1000 )) {
			TRM_OUT01(&ti, GR_FL, LV_FCTINT, "looping for 1 sec , while bit 7 doesn't change  Q7 = %x",Q7(*pTest));
			l++;
		}
		if (( l == 1 ) && ( xtime > 100000 )) {
			TRM_OUT01(&ti, GR_FL, LV_FCTINT, "looping for 10 sec , while bit 7 doesn't change  Q7 = %x",Q7(*pTest));
			l++;
		}

		if ( ( ltime - ltime_start ) > 60 ) /* break loop after 1 minutes */
		{
			TRM_OUT00(&ti, GR_FL, LV_FCTINT, "timeout 60 sec bit 7 doesn't change ");
			retVal = STATUS_ERROR_TIMOUT;
			break;
		}

		if(Q5(*pTest) == 1) {    /* timeout ? */ 
			break;
      }
    }

    if(Q7(*pTest) != vBit) 
	{         /* check Q7 & Q5 race */
      retVal = STATUS_ERROR;
      TRM_OUT00(&ti, GR_FL, LV_FCTINT, "Q7 & Q5 race ");
    }
  }
  TRM_OUT01(&ti, GR_FL, LV_FCTINT, "< flashPoll retval=%#x",retVal);
  return(retVal);
}

static void flashControll(DPR_CHAR *mapStart, unsigned long offset, unsigned long value)
{
	TRM_OUT03(&ti, GR_FL, LV_FCTINT, "> flashControll mapStart=%#x offset=%#x value=%#x",mapStart,offset,value);

  *(volatile FLASH_DEF *)(offset * FLASH_WIDTH + mapStart)  = (FLASH_DEF)value;
  /* 29LV1024 (at least) requires 20ms delay */
  /* DPR_TASK_DELAY(20);*/
  /*DPR_TASK_DELAY(20);*/
  TRM_OUT00(&ti, GR_FL, LV_FCTINT, "< flashControll ");

}

/**
 * flashReset - reset the flash to read mode
 *
 * This routine resets the flash to read mode, it guarantees that further opeations are
 * possible in case the flash is not correctly erased or programmed.
 *
 * RETURNS: Nothing
 **/
DPR_VOID flashReset(DPR_CHAR *mapStart)
{
  TRM_OUT00(&ti, GR_FL, LV_FCTINT, "> flashReset ");

  flashControll(mapStart, FLASH_REG_FIRST_CYCLE, FLASH_CMD_FIRST);
  flashControll(mapStart, FLASH_REG_SECOND_CYCLE, FLASH_CMD_SECOND);
  flashControll(mapStart, FLASH_REG_FIRST_CYCLE, FLASH_CMD_READ_RESET);
  TRM_OUT00(&ti, GR_FL, LV_FCTINT, "< flashReset ");

  return;
}

/**
 * flashSectorErase - erase the contents of a sector
 *
 * This routine clears the contents of one sector in the flash memory.
 *
 * Flash devices are erased by writing the six-byte erase code
 * into specific address locations, which sets all byte locations to a high
 * value (0xFF).
 *
 * RETURNS: STATUS_OK, or STATUS_ERROR if the contents of sector cannot be erased.
 **/
STATUS flashSectorErase(DPR_CHAR *mapStart, int offsetFA)
{
  STATUS retVal = STATUS_OK;
  volatile FLASH_DEF * pTEMP = (FLASH_DEF *)(mapStart + offsetFA);
  time_t ltime;
  time_t ltime_start;
  TRM_OUT01(&ti, GR_FL, LV_FCTINT, "> flashSectorErase offsetFA=%#x",offsetFA);

  flashControll(mapStart, FLASH_REG_FIRST_CYCLE, FLASH_CMD_FIRST);
  flashControll(mapStart, FLASH_REG_SECOND_CYCLE, FLASH_CMD_SECOND);
  flashControll(mapStart, FLASH_REG_FIRST_CYCLE, FLASH_CMD_CHIP_ERASE);
  flashControll(mapStart, FLASH_REG_FIRST_CYCLE, FLASH_CMD_FOURTH);
  flashControll(mapStart, FLASH_REG_SECOND_CYCLE, FLASH_CMD_FIFTH);
  *pTEMP = FLASH_CMD_SECTOR;
  time( &ltime_start );
  
  TRM_OUT00(&ti, GR_FL, LV_FCTINT, "> flashPoll start ");

  do {
    retVal = flashPoll (pTEMP, (FLASH_DEF)0xffffffff);

	time( &ltime );
    // printf( "Time in seconds since UTC 1/1/70:\t%ld\n", ltime );
	if ( ( ltime - ltime_start ) > 60 )
	{
		TRM_OUT00(&ti, GR_FL, LV_FCTINT, "timeout 60 sec ");
		break;
	}

    /*printf("flashSectorErase *pTEMP=x%x, retVal=x%x\n", *pTEMP, retVal);*/
  } while((*pTEMP != (FLASH_DEF)0xffffffff) && (retVal == STATUS_OK));

  TRM_OUT01(&ti, GR_FL, LV_FCTINT, "< flashSectorErase retVal=%#x",retVal);
  return(retVal);
}


/**
 * flashErase - erase the contents of flash memory
 *
 * This routine clears the contents of flash memory.
 *
 * Flash devices are erased by writing a flash erase command to
 * the device and verifying that each flash location is set to a high value
 * (0xFF).
 *
 * RETURNS: STATUS_OK, or STATUS_ERROR if the contents of flash memory cannot be erased.
 **/
STATUS flashErase(DPR_CHAR *mapStart)
{
  volatile FLASH_DEF *pFA = (FLASH_DEF *)mapStart;
  STATUS retVal = STATUS_OK;
  /*int ix;*/
  TRM_OUT00(&ti, GR_FL, LV_FCTINT, "> flashErase ");

  flashControll(mapStart, FLASH_REG_FIRST_CYCLE, FLASH_CMD_FIRST);
  flashControll(mapStart, FLASH_REG_SECOND_CYCLE, FLASH_CMD_SECOND);
  flashControll(mapStart, FLASH_REG_FIRST_CYCLE, FLASH_CMD_CHIP_ERASE);
  flashControll(mapStart, FLASH_REG_FIRST_CYCLE, FLASH_CMD_FOURTH);
  flashControll(mapStart, FLASH_REG_SECOND_CYCLE, FLASH_CMD_FIFTH);
  flashControll(mapStart, FLASH_REG_FIRST_CYCLE, FLASH_CMD_SIXTH);

  do {
    retVal = flashPoll (pFA, (FLASH_DEF) 0xffffffff);
  } while((*pFA != (FLASH_DEF) 0xffffffff) && (retVal == STATUS_OK));

  flashReset(mapStart);
  TRM_OUT01(&ti, GR_FL, LV_FCTINT, "< flashErase retVal=%#x",retVal);

  return(retVal);
}


/**
 *
 * flashWrite - write data to flash memory
 *
 * This routine copies specified data of a specified length, <size>, into a
 * specified offset, <offset>, in the flash memory.
 * The parameter <offset> must be appropriately aligned for the width of
 * the Flash devices in use.
 *
 * RETURNS: STATUS_OK, or STATUS_ERROR if the write operation fails.
 **/
STATUS flashWrite(DPR_CHAR *mapStart, FLASH_DEF *buffer, int size, int offset)
{
  volatile FLASH_DEF *pFA;
  FLASH_DEF value;
  STATUS retVal = STATUS_OK;
  TRM_OUT01(&ti, GR_FL, LV_FCTINT, "> flashWrite size=%#x",size);

  if(buffer == NULL) {
    return STATUS_ERROR;
  }

  for(pFA = (FLASH_DEF *)(mapStart + offset);
     pFA < (FLASH_DEF *)(mapStart + size + offset) &&
     (retVal == STATUS_OK);
     pFA++) {
    flashControll(mapStart, FLASH_REG_FIRST_CYCLE, FLASH_CMD_FIRST);
    flashControll(mapStart, FLASH_REG_SECOND_CYCLE, FLASH_CMD_SECOND);
    flashControll(mapStart, FLASH_REG_FIRST_CYCLE, FLASH_CMD_PROGRAM);


    value = *buffer++;

    /* write value to flash */
    *pFA = value;

    do {
      retVal = flashPoll(pFA, (FLASH_DEF)value);
      /*DPRINT("retVal is 0x%x", retVal);*/
      /*DPRINT("value is 0x%xx", value);*/
      /*DPRINT("*pFA is 0x%x\n", *pFA);*/
    } while((*pFA != value) && (retVal == STATUS_OK));
  }

  flashReset(mapStart);
  TRM_OUT01(&ti, GR_FL, LV_FCTINT, "< flashWrite retVal=%#x",retVal);

  return retVal;
}

/**
 * flashTypeGet - determine the device type of on-board flash memory
 *
 * This routine uses the `autoselect' command to determine the device type of
 * on-board flash memory for flash devices.
 *
 * RETURNS: DPR_CHAR: An "unsigned char" indicating the device type of the
 *          on-board flash memory.
 *
 * IMPORTANT NOTE: DPR_CHAR must be defined as "unsigned char". See os_linux.h !!!
 *
 **/

DPR_CHAR flashTypeGet(DPR_CHAR *mapStart)
{

  volatile FLASH_DEF *pFA = (FLASH_DEF *)mapStart;        /* flash address */
  DPR_CHAR retVal;
  TRM_OUT00(&ti, GR_FL, LV_FCTINT, "> flashTypeGet ");

  /* suppose the wait for completion task will be done
  by the PCI card, make tests to prove that */

  /* struct timespec req;

  req.tv_sec = 0;
  req.tv_nsec = 20000000; */

  flashControll(mapStart, FLASH_REG_FIRST_CYCLE, FLASH_CMD_FIRST);
  flashControll(mapStart, FLASH_REG_SECOND_CYCLE, FLASH_CMD_SECOND);
  flashControll(mapStart, FLASH_REG_FIRST_CYCLE, FLASH_CMD_AUTOSELECT);

  /* 29LV1024 (at least) requires 20ms delay */

  /* It seems we cannot always safely use taskDelay() */

  /* nanosleep(&req, NULL); */

  /*printf("Manufacturer ID ist %x\n", *pFA);*/
  /*printf("Device ID ist %x\n", *(pFA + 1));*/

  /* only for test
  for(i = 0; i<16; i++){
    printf("%02x ", *((unsigned char*)pFA + i) );
  }
  printf("\n");*/

  retVal = (DPR_CHAR) *++pFA;

  flashReset(mapStart);
  TRM_OUT01(&ti, GR_FL, LV_FCTINT, "< flashTypeGet retVal=%#x",retVal);

  return(retVal);
}

/**
 * flashEraseSectors - delete some sectors in the flash
 *
 * This routine deletes the content between start and end address.
 *
 * RETURNS: STATUS_OK, or STATUS_ERROR if anything went wrong.
 **/
STATUS flashEraseSectors(DPR_CHAR *mapFlash, int start_offset, int end_offset, const char *name)
{
  int i, nosector = (end_offset - start_offset) / S_SIZE;
  TRM_OUT00(&ti, GR_FL, LV_FCTINT, "> flashEraseSectors ");

  for(i = 0; i < nosector; i++) {
    TRM_OUT01(&ti, GR_FL, LV_FCTINT, "flashing sector %d", i);
    if(flashSectorErase(mapFlash, (start_offset + i * S_SIZE)) != STATUS_OK) {
      return STATUS_ERROR;
    }
  }
  TRM_OUT00(&ti, GR_FL, LV_FCTINT, "< flashEraseSectors ");

  return STATUS_OK;
}
