/*****************************************************************************/
/* Copyright (C) SIEMENS CORP., 2013 All rights reserved.*/
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

#ifndef TRACEGLOBAL_H
#define TRACEGLOBAL_H

#define TRACEMAXNAMELENGTH    255

#define TRACEDEPTH_NOTHING    0x00000000
#define TRACEDEPTH_ERRORS     0x00000001
#define TRACEDEPTH_WARNINGS   0x00000002
#define TRACEDEPTH_INFO       0x00000003
#define TRACEDEPTH_LEVEL1     0x00000004
#define TRACEDEPTH_LEVEL2     0x00000005
#define TRACEDEPTH_LEVEL3     0x00000006
#define TRACEDEPTH_LEVEL4     0x00000007
#define TRACEDEPTH_LEVEL5     0x00000008
#define TRACEDEPTH_LEVEL6     0x00000009

#define TRACEDEPTH_ALL        0x0FFFFFFF
#define TRACEDEPTH_DEFAULT    TRACEDEPTH_ERRORS

#define TRACEGROUP_ALL        0xFFFFFFFF

#define TRACEDEST_NOTRACE     0x00000000
#define TRACEDEST_NEWFILE     0x00000001
#define TRACEDEST_SAMEFILE    0x00000002
#define TRACEDEST_DEBUGOUT    0x00000004

#define TRACEFILEFAST_OFF     0x00000000
#define TRACEFILEFAST_ON      0x00000001

#define TRACETIME_OFF         0x00000000
#define TRACETIME_ON          0x00000001

#endif /* TRACEGLOBAL_H */
