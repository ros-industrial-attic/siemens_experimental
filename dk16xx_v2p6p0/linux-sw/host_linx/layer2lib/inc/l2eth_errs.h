/****************************************************************************/
/*                                                                          */
/*  Copyright (C) SIEMENS CORP., 2013 All rights reserved.*/
/*                                                                          */
/*    file: l2eth_errs.h                                                    */
/*                                                                          */
/*    Description:                                                          */
/*    error codes for IO-Base user interface                                */
/*                                                                          */
/****************************************************************************/
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
#ifndef _L2ETH_ERRS_H
#define _L2ETH_ERRS_H

/****************************************************************************/
/* IMPORTANT                                                                */
/* All the errors which are specific to firmware should have values between */
/* 0-255 because, the error field is only 1 byte for the communication betn */
/* L2 agent and L2 host.                                                    */
/****************************************************************************/

#define L2ETH_OK                            0x00  /* success */

/****************************************************************************/
/* F I R M W A R E   S P E C I F I C   E R R O R S                          */
/****************************************************************************/
#define L2ETH_ERR_PRM_HND                   0x01  /* parameter Handle is illegal */
#define L2ETH_ERR_PRM_BUF                   0x02  /* The buffer pointer or buffer contents are invalid */
#define L2ETH_ERR_PRM_LEN                   0x03  /* parameter length is wrong */
#define L2ETH_ERR_PRM_QUERY                 0x04  /* parameter query is invalid */
#define L2ETH_ERR_PRM_OID                   0x05  /* parameter oid is invalid */
#define L2ETH_ERR_OID_READONLY              0x06  /* the oid cannot be written */
#define L2ETH_ERR_CANCEL_RQB                0x07  /* requested RQB to FW is cancelled, returning unused RQB to Host */
#define L2ETH_ERR_DELIVERY                  0x08  /* requested message could not be delivered to FW. Host should reattempt sending the same thr DPR */
#define L2ETH_ERR_SEQUENCE                  0x09  /* wrong calling sequence */
#define L2ETH_ERR_MAX_REACHED               0x0A  /* maximal number of opens reached; close unused applications */
#define L2ETH_ERR_NO_RESOURCE               0x0B  /* no resouce too many requests been processed */

#define L2ETH_ERR_INTERNAL                  0xFF  /* fatal error, contact SIEMENS hotline */

/****************************************************************************/
/* H O S T   S P E C I F I C   E R R O R S                                  */
/****************************************************************************/

#ifndef _L2AG_FIRMWARE
/* THE FW SHOULD USE ONLY FW SPECIFIC ERRORS,
   THE HOST CAN USE BOTH HOST AND FW SPECIFIC ERRORS */

#define L2ETH_ERR_PRM_PCBF                  0x101  /* parameter cbf is illegal */
#define L2ETH_ERR_PRM_TYPE                  0x102  /* parameter type has no valid value */
#define L2ETH_ERR_PRM_CP_ID                 0x103  /* parameter CpIndex is wrong - out of allowed range */
#define L2ETH_ERR_PRM_TIMEOUT               0x104  /* parameter timeout has no valid value */
#define L2ETH_ERR_PRM_MODE                  0x105  /* parameter mode is invalid */
#define L2ETH_ERR_PRM_PKT                   0x106  /* parameter packet pointer is invalid */
#define L2ETH_ERR_LLDP_FRAME                0x107  /* trying to send lldp frame */

/* other errors */
#define L2ETH_ERR_NO_CONNECTION             0x301  /* L2Interface data not available, because L2Interface is not                                                                                                connected to controller */
#define L2ETH_ERR_NO_LIC_SERVER             0x302  /* licence server not running, check your installation  */
#define L2ETH_ERR_NO_FW_COMMUNICATION       0x303  /* no communication with firmware, propably driver is not
                                                      loaded, card is not found with cpid provided
                                                      or make sure firmware runs, if not reset cp */

#endif /* _L2AG_FIRMWARE */

#endif  /* _L2ETH_ERRS_H */

