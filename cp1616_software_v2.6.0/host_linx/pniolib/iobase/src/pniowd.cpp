/****************************************************************************
*  Copyright (C) SIEMENS CORP., 2013 All rights reserved.
*****************************************************************************
* FILE NAME    : pniowd.cpp
* ---------------------------------------------------------------------------
* DESCRIPTION  : user interface functions
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

#include "pniointr.h"

/*
 * global critical section to serialise the dll function calls
 */
DPR_MUTEX PnioMutexWd;
DPR_MUTEX PnioMutexDownload;

/*************************************************************************************
*
* Exported functions
*
**************************************************************************************/

/*===========================================================================
* FUNCTION : PNIO_CP_set_appl_watchdog
*----------------------------------------------------------------------------
* PURPOSE  : With this function an application program registers
*            itself with the Formware Watchdog
*----------------------------------------------------------------------------
* RETURNS  : PNIO_OK on success
*----------------------------------------------------------------------------
* INPUTS   : CpIndex - Unique identification for the communication module
*                      (module index in the component configuration)
*            wdTimeOutInMs - timeout value in milliseconds
*            pnio_appl_wd_cbf - Callback function for signaling the timeout
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
PNIO_UINT32 PNIO_CODE_ATTR
PNIO_CP_set_appl_watchdog(PNIO_UINT32 CpIndex,
    PNIO_UINT32 wdTimeOutInMs,
    PNIO_CBF_APPL_WATCHDOG pnio_appl_wd_cbf)
{
    PNIO_UINT32 Ret = PNIO_OK;

    DPR_MUTEX_LOCK(PnioMutexWd);

    TRC_OUT(GR_INIT, LV_FCTPUB1, "->PNIO_CP_set_appl_watchdog");

    TRC_IF_ON_EXPR(GR_INIT, LV_FCTPUB2,
        OSTREAM trcos;
        trcos << showbase << hex;
        trcos << "  CpIndex=" << CpIndex;
        trcos << ", wdTimeOutInMs=" << wdTimeOutInMs;
        trcos << ", pnio_appl_wd_cbf=" << (void *)pnio_appl_wd_cbf;
        trcos << ends;
        TRC_OUT_OBJECT(GR_INIT, LV_FCTPUB2, trcos);
        );

    Ret = ICommon::set_appl_watchdog(CpIndex, wdTimeOutInMs, pnio_appl_wd_cbf);

    TRC_IF_ON_EXPR(GR_INIT, LV_FCTPUB1,
        OSTREAM trcos1;
        trcos1 << showbase << hex;
        trcos1 << "<-PNIO_CP_set_appl_watchdog, ret=" << Ret;
        trcos1 << ends;
        TRC_OUT_OBJECT(GR_INIT, LV_FCTPUB1, trcos1);
        );

    DPR_MUTEX_UNLOCK(PnioMutexWd);

    return Ret;
}

/*===========================================================================
* FUNCTION : PNIO_CP_trigger_watchdog
*----------------------------------------------------------------------------
* PURPOSE  : With this function an application program resets timeout trigger
*----------------------------------------------------------------------------
* RETURNS  : PNIO_OK on success
*----------------------------------------------------------------------------
* INPUTS   : CpIndex - Unique identification for the communication module
*                      (module index in the component configuration)
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
PNIO_UINT32 PNIO_CODE_ATTR
PNIO_CP_trigger_watchdog(PNIO_UINT32 CpIndex)
{
    PNIO_UINT32 Ret = PNIO_OK;

    DPR_MUTEX_LOCK(PnioMutexWd);

    TRC_OUT(GR_STATE, LV_FCTPUB1, "->PNIO_CP_trigger_watchdog");

    TRC_IF_ON_EXPR(GR_STATE, LV_FCTPUB2,
        OSTREAM trcos;
        trcos << showbase << hex;
        trcos << "  CpIndex=" << CpIndex;
        trcos << ends;
        TRC_OUT_OBJECT(GR_INIT, LV_FCTPUB2, trcos);
        );

    Ret = ICommon::trigger_watchdog(CpIndex);

    TRC_IF_ON_EXPR(GR_STATE, LV_FCTPUB1,
        OSTREAM trcos1;
        trcos1 << showbase << hex;
        trcos1 << "<-PNIO_CP_trigger_watchdog, ret=" << Ret;
        trcos1 << ends;
        TRC_OUT_OBJECT(GR_INIT, LV_FCTPUB1, trcos1);
        );

    DPR_MUTEX_UNLOCK(PnioMutexWd);

    return Ret;
}

/*===========================================================================
* FUNCTION : PNIO_CP_register_cbf
*----------------------------------------------------------------------------
* PURPOSE  : With this function an application program registers global events
*----------------------------------------------------------------------------
* RETURNS  : PNIO_OK on success
*----------------------------------------------------------------------------
* INPUTS   :
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
PNIO_UINT32 PNIO_CODE_ATTR
PNIO_CP_register_cbf(PNIO_UINT32 AppHandle,
        PNIO_CP_CBE_TYPE CbeType,
        PNIO_CP_CBF Cbf)
{
    PNIO_UINT32 Ret = PNIO_OK;

    DPR_MUTEX_LOCK(PnioMutexWd);

    TRC_OUT(GR_STATE, LV_FCTPUB1, "->PNIO_CP_register_cbf");

    TRC_IF_ON_EXPR(GR_STATE, LV_FCTPUB2,
        OSTREAM trcos;
        trcos << showbase << hex;
        trcos << "  AppHandle=" << AppHandle;
        trcos << "  CbeType=" << CbeType;
        trcint_ShowPtr(trcos, (void *)Cbf, "Cbf");
        trcos << ends;
        TRC_OUT_OBJECT(GR_INIT, LV_FCTPUB2, trcos);
        );

        ICommon *pICommonInst = ICommon::get_instance(AppHandle);

    if(pICommonInst) {
        TRC_OUT(GR_INIT, LV_FCTPUB1, " found device instance ");

        Ret = pICommonInst->CP_register_cbf(CbeType, Cbf);
        if(Ret != PNIO_OK) {
            TRC_OUT(GR_INIT, LV_ERR, "  * CP_register_cbf() failed ");
        }
    } else {
        TRC_OUT(GR_INIT, LV_ERR, "  * ERROR getting device instance ");
        Ret = PNIO_ERR_WRONG_HND;
    }

    TRC_IF_ON_EXPR(GR_STATE, LV_FCTPUB1,
        OSTREAM trcos1;
        trcos1 << showbase << hex;
        trcos1 << "<-PNIO_CP_register_cbf, ret=" << Ret;
        trcos1 << ends;
        TRC_OUT_OBJECT(GR_INIT, LV_FCTPUB1, trcos1);
        );

    DPR_MUTEX_UNLOCK(PnioMutexWd);

    return Ret;
}

/*===========================================================================
* FUNCTION : PNIO_CP_set_opdone
*----------------------------------------------------------------------------
* PURPOSE  : With this function an application program
*----------------------------------------------------------------------------
* RETURNS  : PNIO_OK on success
*----------------------------------------------------------------------------
* INPUTS   :
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
PNIO_UINT32 PNIO_CODE_ATTR
PNIO_CP_set_opdone(PNIO_UINT32 AppHandle, PNIO_CYCLE_INFO * pCycleInfo)
{
    PNIO_UINT32 Ret = PNIO_OK;

    //DPR_MUTEX_LOCK(PnioMutexWd);

    TRC_OUT(GR_RT, LV_FCTPUB1, "->PNIO_CP_set_opdone");

    TRC_IF_ON_EXPR(GR_RT, LV_FCTPUB2,
        OSTREAM trcos;
        trcos << showbase << hex;
        trcos << "  AppHandle=" << AppHandle;
        trcos << ends;
        TRC_OUT_OBJECT(GR_RT, LV_FCTPUB2, trcos);
        );

        ICommon *pICommonInst = ICommon::get_instance(AppHandle);

    if(pICommonInst) {
        Ret = pICommonInst->CP_set_opdone(pCycleInfo);
    } else {
        Ret = PNIO_ERR_WRONG_HND;
    }

    TRC_IF_ON_EXPR(GR_RT, LV_FCTPUB1,
        OSTREAM trcos1;
        trcos1 << showbase << hex;
        trcos1 << "<-PNIO_CP_set_opdone, ret=" << Ret;
        trcos1 << ends;
        TRC_OUT_OBJECT(GR_RT, LV_FCTPUB1, trcos1);
        );

    //DPR_MUTEX_UNLOCK(PnioMutexWd);

    return Ret;
}

PNIO_UINT32 PNIO_CODE_ATTR
PNIO_CP_cycle_stat(PNIO_UINT32 AppHandle, int MeasureNr, PNIO_CYCLE_STAT * pCycleStat)
{
    PNIO_UINT32 Ret = PNIO_OK;

    //DPR_MUTEX_LOCK(PnioMutexWd);

    ICommon *pICommonInst = ICommon::get_instance(AppHandle);

    if(pICommonInst) {
        Ret = pICommonInst->CP_cycle_stat(pCycleStat,MeasureNr);
    } else {
        Ret = PNIO_ERR_WRONG_HND;
    }

    //DPR_MUTEX_UNLOCK(PnioMutexWd);

    return Ret;


    if (pCycleStat == NULL)     {

    }
}

PNIO_UINT32 PNIO_CODE_ATTR
PNIO_CP_cycle_info(PNIO_UINT32 AppHandle, PNIO_CI_ENTRY * pCycleInfoEntry, int MeasureNr, PNIO_UINT32 Offset)
{
    PNIO_UINT32 Ret = PNIO_ERR_INTERNAL;

    //DPR_MUTEX_LOCK(PnioMutexWd);

    ICommon *pICommonInst = ICommon::get_instance(AppHandle);

    if(pICommonInst) {
        Ret = pICommonInst->CP_cycle_info(pCycleInfoEntry, MeasureNr, Offset);
    } else {
        Ret = PNIO_ERR_WRONG_HND;
    }

    //DPR_MUTEX_UNLOCK(PnioMutexWd);

    return Ret;
}

