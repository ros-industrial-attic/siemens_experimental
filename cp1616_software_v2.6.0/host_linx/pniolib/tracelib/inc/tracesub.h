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

#ifndef _TRACE_SUB_H_
#define _TRACE_SUB_H_

#define GET_GR_SNAME(subsys) subsys##_NAME

/* trace groups */
#define GR_INIT                 0x00000001      /* interface initialisation functions */
#define GR_INIT_NAME            "[INIT]"

#define GR_STATE                0x00000002      /* change state functions */
#define GR_STATE_NAME           "[STATE]"

#define GR_IO                   0x00000004      /* cyclic io-data function */
#define GR_IO_NAME              "[IO]"

#define GR_DS                   0x00000008      /* acyclic data function */
#define GR_DS_NAME              "[DS]"

#define GR_ALARM                0x00000010      /* alarm function */
#define GR_ALARM_NAME           "[ALARM]"

#define GR_MGT                  0x00000020      /* managment functions */
#define GR_MGT_NAME             "[MGT]"

#define GR_CHNL                 0x00000040      /* transport channel functions */
#define GR_CHNL_NAME            "[CHNL]"

#define GR_RT                   0x00000080      /* rt channel funktions */
#define GR_RT_NAME              "[RT]"

#define GR_IOR                  0x00000100      /* io router, concentrator, wdg funktions */
#define GR_IOR_NAME             "[IOR]"

#define GR_PE                   0x00000200      /* PE: PROFIenergy    */
#define GR_PE_NAME              "[PE]"

#define GR_FL                   0x00000400      /* flash operations   */
#define GR_FL_NAME              "[FL]"

#define GR_DR                   0x00000800      /* communication with driver */
#define GR_DR_NAME              "[DR]"

#endif /* _TRACE_SUB_H_ */

