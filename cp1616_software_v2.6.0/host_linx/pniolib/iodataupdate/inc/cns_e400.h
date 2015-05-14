/*****************************************************************************/
/*  Copyright (C) SIEMENS CORP., 2013 All rights reserved.*/
/*****************************************************************************/
/*                                                                           */
/* Diese Software ist Freeware. Sie wird Ihnen unentgeltlich zur Verfuegung  */
/* gestellt. Sie darf frei kopiert, modifiziert und benutzt sowie an Dritte  */
/* weitergegeben werden. Die Software darf nur unter Beibehaltung aller      */
/* Schutzrechtsvermerke sowie nur vollstaedig und unveraedert weitergegeben  */
/* werden. Die kommerzielle Weitergabe an Dritte (z.B. im Rahmen von         */
/* Share-/Freeware-Distributionen) ist nur mit vorheriger schriftlicher      */
/* Genehmigung der Siemens Aktiengesellschaft erlaubt.                       */
/* DA DIE SOFTWARE IHNEN UNENTGELTLICH UEBERLASSEN WIRD, KOENNEN DIE AUTOREN */
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

#ifndef _CNS_E400_H_
#define _CNS_E400_H_

#ifndef WIN32
	#include <inttypes.h>
#endif
typedef uint32_t CNS_UINT32;
typedef uint8_t CNS_UINT8;

#define CNS_FCT_ATTR
#define CNS_MAX_POLL_TIMEOUT 0x00000400UL

void       CNS_FCT_ATTR CnsInOutDone(
    const CNS_UINT8 *IRTE_SWI_BaseAdr);

CNS_UINT32 CNS_FCT_ATTR CnsBlockRead(
    const CNS_UINT8 *IRTE_SWI_BaseAdr,
    const CNS_UINT32 iBlockStart,
    const CNS_UINT32 iBlockLen,
    const CNS_UINT32 PollTimeout);

CNS_UINT32 CNS_FCT_ATTR CnsBlockFreeRead(
    const CNS_UINT8 *IRTE_SWI_BaseAdr,
    const CNS_UINT32 PollTimeout);

CNS_UINT32 CNS_FCT_ATTR CnsBlockWrite(
    const CNS_UINT8 *IRTE_SWI_BaseAdr,
    const CNS_UINT32 iBlockStart,
    const CNS_UINT32 iBlockLen,
    const CNS_UINT32 PollTimeout);

CNS_UINT32 CNS_FCT_ATTR CnsBlockFreeWrite(
    const CNS_UINT8 *IRTE_SWI_BaseAdr,
    const CNS_UINT32 PollTimeout);

typedef void (CNS_FCT_ATTR *CNS_EXCP_FCT) (
    const CNS_UINT32 Line,
    CNS_UINT8 *const sFile,
    const CNS_UINT32 ModuleID,
    CNS_UINT8 *const sErr,
    const CNS_UINT32 Error,
    const CNS_UINT32 DW_0,
    const CNS_UINT32 DW_1);

void       CNS_FCT_ATTR CnsSetExcp(
    CNS_EXCP_FCT const excp_fct);

#endif /* _CNS_E400_H_ */
