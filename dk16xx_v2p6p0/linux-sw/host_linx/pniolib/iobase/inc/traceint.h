
/****************************************************************************
*  Copyright (C) SIEMENS CORP., 2013 All rights reserved.
*****************************************************************************
* FILE NAME    : traceint.h
* ---------------------------------------------------------------------------
* DESCRIPTION  : pnio specific trace implementation
*****************************************************************************/
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

#ifndef _PNIO_TRACEINT_H
#define _PNIO_TRACEINT_H

#include "tracesub.h"
#include "tracemcr.h"

#if defined(PC_TRACE_ON) || defined(PRN_TRACE_ON) || defined(WDB_TRACE_ON)

using namespace std;


 inline void trcint_ShowPtr(OSTREAM &os, const void *pVal, const char *Name)
 {
    os<<", "<<Name<<"=";
    if(pVal)
        os<<pVal;
    else
        os<<"NULL";
 }

 inline void trcint_ShowPtrValue(OSTREAM &os, const void *pVal, const char *Name)
 {
    os<<", "<<Name<<"=";
    if(pVal)
        os<<*(unsigned long*)pVal;
    else
        os<<"NULL";
 }

 inline void trcint_ShowStatus(OSTREAM &os, const PNIO_IOXS *pStat, const char *Name)
 {
    os<<", "<<Name<<"=";
    if(pStat)
    {
      switch(*pStat)
      {
      case PNIO_S_GOOD: os<<"GOOD"; break;
      case PNIO_S_BAD:  os<<"BAD"; break;
      default: os<<"?? ("<< (unsigned long)*pStat << ")";
      }
    }
    else
        os<<"NULL";
 }

 inline const char * PnioModeToStr(const PNIO_MODE_TYPE Mode)
 {
      switch(Mode)
      {
      case PNIO_MODE_OFFLINE: return "OFFLINE";
      case PNIO_MODE_CLEAR:   return "CLEAR";
      case PNIO_MODE_OPERATE: return "OPERATE";
      default: return "??";
      }
 }

 inline void trcint_ShowMode(OSTREAM &os, const PNIO_MODE_TYPE *pMode, const char *Name)
 {
    os<<", "<<Name<<"=";
    if(pMode)
    {
      switch(*pMode)
      {
      case PNIO_MODE_OFFLINE: os<<"OFFLINE"; break;
      case PNIO_MODE_CLEAR:   os<<"CLEAR"; break;
      case PNIO_MODE_OPERATE: os<<"OPERATE"; break;
      default: os<<"?? ("<< (unsigned long)*pMode << ")";
      }
    }
    else
        os<<"NULL";
 }

 inline void trcint_ShowActType(OSTREAM &os, const PNIO_DEV_ACT_TYPE *pActType, const char *Name)
 {
    os<<", "<<Name<<"=";
    if(pActType)
    {
      switch(*pActType)
      {
      case PNIO_DA_FALSE:   os<<"DA_FALSE"; break;
      case PNIO_DA_TRUE:   os<<"DA_TRUE"; break;
      default: os<<"?? ("<< (unsigned long)*pActType << ")";
      }
    }
    else
        os<<"NULL";
 }

 inline void trcint_ShowAlarmType(OSTREAM &os, const PNIO_ALARM_TYPE *pAlType, const char *Name)
 {
    os<<", "<<Name<<"=";
    if(pAlType)
    {
      switch(*pAlType)
      {
      case PNIO_ALARM_DIAGNOSTIC:   os<<"DIAGNOSTIC"; break;
      case PNIO_ALARM_PROCESS:      os<<"PROCESS"; break;
      case PNIO_ALARM_PULL:         os<<"PULL"; break;
      case PNIO_ALARM_PLUG:         os<<"PLUG"; break;
      case PNIO_ALARM_STATUS:       os<<"STATUS"; break;
      case PNIO_ALARM_UPDATE:       os<<"UPDATE"; break;
      case PNIO_ALARM_REDUNDANCY:   os<<"REDUNDANCY"; break;
      case PNIO_ALARM_CONTROLLED_BY_SUPERVISOR:   os<<"CONTROLLED_BY_SUPERVISOR"; break;
      case PNIO_ALARM_RELEASED_BY_SUPERVISOR:     os<<"RELEASED_BY_SUPERVISOR"; break;
      case PNIO_ALARM_PLUG_WRONG:             os<<"PLUG_WRONG"; break;
      case PNIO_ALARM_RETURN_OF_SUBMODULE:    os<<"RETURN_OF_SUBMODULE"; break;
      case PNIO_ALARM_DIAGNOSTIC_DISAPPEARS:   os<<"DIAGNOSTIC_DISAPPEARS"; break;
      case PNIO_ALARM_DEV_FAILURE:   os<<"DEV_FAILURE"; break;
      case PNIO_ALARM_DEV_RETURN:    os<<"DEV_RETURN"; break;
      default: os<<"?? ("<< (unsigned long)*pAlType << ")";
      }
    }
    else
        os<<"NULL";
 }

 inline void trcint_ShowPullPlugAction(OSTREAM &os, const PNIO_DEV_ACTION Action, const char *Name)
 {
    os<<", "<<Name<<"=";
    switch(Action)
    {
        case PNIO_MOD_PULL: os << " MOD_PULL"; break;
        case PNIO_MOD_PLUG: os << " MOD_PLUG"; break;
        case PNIO_SUB_PULL: os << " SUB_PULL"; break;
        case PNIO_SUB_PLUG: os << " SUB_PLUG"; break;

        default: os<<"?? ("<< (unsigned long)Action << ")";
    }
 }

 void trcint_ShowAddr(OSTREAM &os, const PNIO_ADDR *add);
 void trcint_ShowDAddr(OSTREAM &os, const PNIO_DEV_ADDR *add);
 void trcint_ShowData(OSTREAM &os, PNIO_UINT32 Len, const PNIO_UINT8 *Buf);
 void trcint_ShowDerror(OSTREAM &os, const PNIO_ERR_STAT *err);
 void trcint_ShowDiagReq(OSTREAM &os, const PNIO_CTRL_DIAG * diag);

#endif /* PC_TRACE_ON */

#endif // _PNIO_TRACEINT_H
