/****************************************************************************
*  Copyright (C) SIEMENS CORP., 2013 All rights reserved.
*****************************************************************************
* FILE NAME    : iodu_com.h
* ---------------------------------------------------------------------------
* DESCRIPTION  : common iodata update
*****************************************************************************/
/*                                                                           */
/* Diese Software ist Freeware. Sie wird Ihnen unentgeltlich zur Verfuegung  */
/* gestellt. Sie darf frei kopiert, modifiziert und benutzt sowie an Dritte  */
/* weitergegeben werden. Die Software darf nur unter Beibehaltung aller      */
/* Schutzrechtsvermerke sowie nur vollstaedig und unveraedert weitergegeben  */
/* werden. Die kommerzielle Weitergabe an Dritte (z.B. im Rahmen von         */
/* Share-/Freeware-Distributionen) ist nur mit vorheriger schriftlicher      */
/* Genehmigung der Siemens Aktiengesellschaft erlaubt.                       */
/* DA DIE SOFTWARE IHNEN UNENTGELTLICH UBERLASSEN WIRD, KOENNEN DIE AUTOREN  */
/* UND RECHTSINHABER DAFUER KEINE HAFTUNG UEBERNEHMEN. IHRE BENUTZUNG ERFOLGT*/
/* AUF EIGENE GEFAHR UND VERANTWORTUNG. DIE AUTOREN UND RECHTSINHABER HAFTEN */
/* NUR FUER VORSATZ UND GROBE FAHRLAESSIGKEIT. WEITERGEHENDE ANSPRUECHE SIND */
/* AUSGESCHLOSSEN. INSBESONDERE HAFTEN DIE AUTOREN UND RECHTSINHABER NICHT   */
/* FUER ETWAIGE MAENGEL ODER FOLGESCHAEDEN.                                  */
/* Falls Sie Fehler in der Software bemerken, teilen Sie es uns bitte mit.   */
/*                                                                           */
/* This Software is Freeware. It is distributed for free. You may copy,      */
/* modify and use it for free as well as distribute it to others. You may    */
/* only distribute it as a whole, unmodified and with all trademarks and     */
/* copyright notices. You may not distribute it commercially (e.g. as a      */
/* Share-/Freeware-Distributor) without the express written permission of    */
/* Siemens Aktiengesellschaft. SINCE THIS SOFTWARE IS DISTRIBUTED FOR FREE,  */
/* IT IS PROVIDED "AS IS" WITHOUT ANY REPRESENTATION OR WARRANTY OF ANY KIND */
/* EITHER EXPRESSED OR IMPLIED INCLUDING BUT NOT LIMITED TO IMPLIED          */
/* WARRANTIES FOR MERCHANTIBILITY OR FITNESS FOR USE. ANY USE OF THE         */
/* APPLICATION EXAMPLE IS ON YOUR OWN RISK AND RESPONSIBILITY.               */
/* If you happen to find any faults in it please tell us.                    */
/*                                                                           */
/*****************************************************************************/

#include "pnio_inc.h"

/* ATTENTION !!!!!!
   PNIO_SET/GET_IOXS have in PNIOBASE.H and in PNIO_INC.H
   different representation of GOOD/BAD Status

               PNIO_SET/GET_IOXS      IO-Base Library
   GOOD        >0                     PNIO_S_GOOD = 0
   BAD         0                      PNIO_S_BAD  = 1

   BECAUSE use only IODU_SET/GET_IOXS to be comform with IO-Base Library  */

#define IODU_SET_IOXS(pIoxs, IoxsLen, val, type) \
    PNIO_SET_IOXS(pIoxs, IoxsLen, ((val)==PNIO_S_GOOD) ? 1 : 0, type)
#define IODU_GET_IOXS(pIoxs, IoxsLen) \
    (PNIO_GET_IOXS_DATA_STATE(pIoxs, IoxsLen) ? PNIO_S_GOOD : PNIO_S_BAD)

typedef struct _IODU_CONSUMER_APDU_STATUS
{
  PNIO_UINT16 CycleCount;
  PNIO_UINT8  DataStatus;
  PNIO_UINT8  TransferStatus;
} IODU_CONSUMER_APDU_STATUS;

#define IODU_APDU_DATA_VALID_MASK 0x04
#define IODU_APDU_TRANSFER_MASK 0x0F

#define IODU_GET_CONSUMER_QUALITY(pApduStatus) \
  ( !(((IODU_CONSUMER_APDU_STATUS *)(pApduStatus))->DataStatus & IODU_APDU_DATA_VALID_MASK) ? PNIO_S_BAD : \
      !(((IODU_CONSUMER_APDU_STATUS *)(pApduStatus))->TransferStatus & IODU_APDU_TRANSFER_MASK) ? PNIO_S_BAD : PNIO_S_GOOD)

/* trace macros only for test */
/* #define IODU_TRC_ON */
#ifdef  IODU_TRC_ON
 #include "stdio.h" /* wegen IODU_TRC_X */
 #define IODU_TRC(_p_)    printf _p_ ;
 /* printf("KRAMIOTLB: "s"\n") */
#else
 #define IODU_TRC(_p_)
#endif

