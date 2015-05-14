/*****************************************************************************/
/*                                                                           */
/*  Copyright (C) SIEMENS CORP., 2013 All rights reserved.*/
/*                                                                           */
/*    file: l2eth_config.h													                         */
/*                                                                           */
/*    Description:                                                           */
/*    l2 configuration declarations											                     */
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

#ifndef L2ETH_CONFIG_H
#define L2ETH_CONFIG_H

/* only hw constants */

/* maximum recieve buffer frame count */
#define MAX_ETHERNET_RECEIVE_FRAME_COUNT 40     //60


/* maximum send buffer pool frame count */
#define MAX_ETHERNET_SEND_FRAME_COUNT 40       //60

/* max of l2 packet frame size is */
#define MAX_ETHERNET_FRAME_SIZE 1520
#define MIN_ETHERNET_FRAME_SIZE 64	/* Data(46) + Ethernet Head(14) + Vlan tag(4) */

/* limits of l2 packet size */
#define MAX_L2_FRAME_SIZE 1518
#define MIN_L2_FRAME_SIZE 60

/* max CPs Supported */
#define MAX_CP_SUPPORTED (8)


#endif /* L2ETH_CONFIG_H */
