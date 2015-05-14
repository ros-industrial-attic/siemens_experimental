/*****************************************************************************/
/* DESCRIPTION: configuration file for KRAMIOTLB interface                   */
/* DATE:        29.11.2004                                                   */
/*                                                                           */
/*****************************************************************************/
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
#ifndef KRAMIOTLB_CFG_H
#define KRAMIOTLB_CFG_H

#include "os.h"
#include "pniobase.h"

#define SWAP_W(var) CPU_TO_LE16(var)
#define SWAP_D(var) CPU_TO_LE(var)
#define CNS_SWAP_32(var) CPU_TO_LE(var)

/* trace macros only for test */
/* #define KTLB_TRC_ON */
#ifdef  KTLB_TRC_ON
#include "stdio.h"              /* wegen KTLB_TRC_X */
#define KTLB_TRC_0(s)        printf("KRAMIOTLB(%s): "s"\n", __FUNCTION__)
#define KTLB_TRC_1(s,p1)     printf("KRAMIOTLB(%s): "s"\n", __FUNCTION__, p1)
#define KTLB_TRC_2(s,p1,p2)  printf("KRAMIOTLB(%s): "s"\n", __FUNCTION__, p1, p2)
#else
#define KTLB_TRC_0(s)
#define KTLB_TRC_1(s,p1)
#define KTLB_TRC_2(s,p1,p2)
#endif

/* #define IODU_ENABLE_MULTIUSER_LOCKING */
#ifndef __sparc__ /* FIXME */
#define IODU_ENABLE_INTERPROCESS_LOCKING
#endif

/* define, to use KRAMIOTLB_AddItem, KRAMIOTLB_FreeItems */
/* #define KRAMIOTLB_SUPPORT_TARGET */

/* define, to use KRAMIOTLB_GetContrItemByLogAddr */
#define KRAMIOTLB_SUPPORT_HOST_CONTROLLER

#define KRAMIOTLB_SUPPORT_HOST_CONTROLLER_HASH

/* define, to use KRAMIOTLB_CreateContrHash, KRAMIOTLB_FreeContrHash */
#define KRAMIOTLB_SUPPORT_HOST_CONTROLLER_HASH_SORT

/* define, to use  IODU_ctrl_data_read_ex, IODU_ctrl_data_write_ex */
#define IODU_SUPPORT_IO_CACHE_CONTROLLER

/* define, to use KRAMIOTLB_GetDeviceItems */
#define KRAMIOTLB_SUPPORT_HOST_DEVICE

/* build for host software */
#define KRAMIOTLB_HOST

#define IODU_ERTEC_TYPE IODU_ERTEC400

#endif /* KRAMIOTLB_CFG_H */
