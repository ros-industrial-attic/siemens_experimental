/****************************************************************************
*  Copyright (C) SIEMENS CORP., 2013 All rights reserved.
*****************************************************************************
* FILE NAME    : pniousrd.cpp
* ---------------------------------------------------------------------------
* DESCRIPTION  : device specific user interface functions
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

/* global critical section to serialise the dll function calls */
DPR_MUTEX PnioMutexDevice;

/* exported functions */
#ifdef MANAGED_CP_EXCHANGE
extern "C" PNIO_UINT32 PNIO_CODE_ATTR ldah_check(PNIO_UINT32 CpIndex);
#endif

/*===========================================================================
* FUNCTION : PNIO_device_open
*----------------------------------------------------------------------------
* PURPOSE  : With this function an application program registers
*            itself with the PROFInet IO as IO device
*----------------------------------------------------------------------------
* RETURNS  : PNIO_OK on success; else *
*----------------------------------------------------------------------------
* INPUTS   : CpIndex - Unique identification for the communication module
*                      (module index in the component configuration)
*            ExtPar - Extended parameter (see earlier definition),
*                     among others the following parameters can be
*                     applied via the OR logical operator
*            NumOfSlots - Maximal Count of available slots
*            VendorId - Unique ID
*            DeviceId - Unique ID
*            InstanceId - Unique ID
*            pDevAnnotation - pointer to struct, which describes device
*            NumOfCbf - Length of array pointed with pCbf
*            pCbf - Array of callback functions
* OUTPUS   : pApplHandle - pointer to Handle, which is assigned to the device
*----------------------------------------------------------------------------
* COMMENTS : With this function an application program registers itself
*            with the PROFInet IO as IO device. The actual configuration
*            is specified. The Target/Actual comparison of
*            the configuration is conducted internally.
*==========================================================================*/
PNIO_UINT32 PNIO_CODE_ATTR
PNIO_device_open(PNIO_UINT32 CpIndex,
    PNIO_UINT32 ExtPar,
    PNIO_UINT16 VendorId,
    PNIO_UINT16 DeviceId,
    PNIO_UINT16 InstanceId,
    PNIO_UINT32 MaxAR,
    PNIO_ANNOTATION * pDevAnnotation,
    PNIO_CFB_FUNCTIONS *pCbf,
    PNIO_UINT32 * pDevHndl)
{
    PNIO_UINT32 Ret = PNIO_OK;
    DPR_MUTEX_LOCK(PnioMutexDevice);

    TRC_OUT01(GR_INIT, LV_FCTPUB1, "->PNIO_device_open CpIndex %d", CpIndex);

#ifdef MANAGED_CP_EXCHANGE
    (void)ldah_check(CpIndex);
#endif

    TRC_IF_ON_EXPR(GR_INIT, LV_FCTPUB2,
        OSTREAM trcos;
        trcos << showbase << hex;
        trcos << "  CpIndex=" << CpIndex;
        trcos << ", ExtPar=" << ExtPar;
        trcos << ", VendorId=" << VendorId;
        trcos << ", DeviceId=" << DeviceId;
        trcos << ", InstanceId=" << InstanceId;
        trcos << ", MaxAR=" << MaxAR;

        if(pDevAnnotation) {
            char tmp1[MAX_DEVICE_TYPE_LENGTH+2];
            char tmp2[MAX_ORDER_ID_LENGTH+2];

            memcpy(tmp1, pDevAnnotation->deviceType, sizeof(pDevAnnotation->deviceType));
            tmp1[MAX_DEVICE_TYPE_LENGTH+1] = '\0';
            trcos << ", DeviceType='" << tmp1 << "'";

            memcpy(tmp2, pDevAnnotation->orderId, sizeof(pDevAnnotation->orderId));
            tmp2[MAX_ORDER_ID_LENGTH+1] = '\0';
            trcos << ", OrderId='" << tmp2 << "'";

            trcos << ", HwRevision=" << pDevAnnotation->hwRevision;
            trcos << ", SwRevisionPrefix='" << pDevAnnotation->swRevisionPrefix << "'";
            trcos << ", SwRevision1=" << pDevAnnotation->swRevision1;
            trcos << ", SwRevision2=" << pDevAnnotation->swRevision2;
            trcos << ", SwRevision3=" << pDevAnnotation->swRevision3;
        } else {
            trcos << ", pDevAnnotation=NULL";
        }
        trcint_ShowPtr(trcos, pDevHndl, "pDevHndl");
        trcint_ShowPtr(trcos, pCbf, "pCbf");
        trcos << ends;
        TRC_OUT_OBJECT(GR_INIT, LV_FCTPUB2, trcos);
        );

        Ret = IDevice::device_open(CpIndex, ExtPar, VendorId,
            DeviceId, InstanceId, MaxAR, pDevAnnotation, pCbf, pDevHndl);

    TRC_IF_ON_EXPR(GR_INIT, LV_FCTPUB1,
        OSTREAM trcos1;
        trcos1 << showbase << hex;
        trcos1 << "<-PNIO_device_open, ret=" << Ret;
        trcint_ShowPtrValue(trcos1, pDevHndl, "pDevHndl");
        trcos1 << '\0';
        TRC_OUT_OBJECT(GR_INIT, LV_FCTPUB1, trcos1);
        );

    DPR_MUTEX_UNLOCK(PnioMutexDevice);

    return Ret;
}

/*===========================================================================
* FUNCTION : PNIO_device_close
*----------------------------------------------------------------------------
* PURPOSE  : close Device from handle
*----------------------------------------------------------------------------
* RETURNS  : PNIO_OK on success; else PNIO_ERR_WRONG_HND, PNIO_ERR_INTERNAL
*----------------------------------------------------------------------------
* INPUTS   : - DevHndl -  Handle Handle from PNIO_device_open
* OUTPUS   : -
*----------------------------------------------------------------------------
* COMMENTS : With this function the application program deregisters
* an IO-Device with PROFInet, which had been previously registered with
* PNIO_device_open
*==========================================================================*/
PNIO_UINT32 PNIO_CODE_ATTR PNIO_device_close(PNIO_UINT32 DevHndl)
{
    PNIO_UINT32 Ret;

    DPR_MUTEX_LOCK(PnioMutexDevice);

    TRC_OUT(GR_INIT, LV_FCTPUB1, "->PNIO_device_close");

    TRC_IF_ON_EXPR(GR_INIT, LV_FCTPUB2,
        OSTREAM trcos;
        trcos << showbase << hex;
        trcos << "  DevHndl=" << DevHndl;
        trcos << ends;
        TRC_OUT_OBJECT(GR_INIT, LV_FCTPUB2, trcos);
        );

    ICommon *pICommonInst = ICommon::get_instance(DevHndl);

    if(pICommonInst) {

        Ret = ((IDevice *) pICommonInst)->device_close();
        if(Ret != PNIO_OK) {
            TRC_OUT(GR_INIT, LV_ERR, "  * device_close() failed ");
        }
    } else {
        TRC_OUT(GR_INIT, LV_ERR, "  * ERROR getting device instance ");
        Ret = PNIO_ERR_WRONG_HND;
    }

    TRC_IF_ON_EXPR(GR_INIT, LV_FCTPUB1,
        OSTREAM trcos1;
        trcos1 << showbase << hex;
        trcos1 << "<-PNIO_device_close, ret=" << Ret;
        trcos1 << '\0';
        TRC_OUT_OBJECT(GR_INIT, LV_FCTPUB1, trcos1);
        );

    DPR_MUTEX_UNLOCK(PnioMutexDevice);

    return Ret;
}

PNIO_UINT32 PNIO_CODE_ATTR PNIO_set_appl_state_ready(PNIO_UINT32 DevHndl,
    PNIO_UINT16 ArNumber,
    PNIO_UINT16 SessionKey,
    PNIO_APPL_READY_LIST_TYPE * pList)
{
    PNIO_UINT32 Ret;

    DPR_MUTEX_LOCK(PnioMutexDevice);

    TRC_OUT(GR_INIT, LV_FCTPUB1, "->PNIO_set_appl_state_ready");

    TRC_IF_ON_EXPR(GR_INIT, LV_FCTPUB2,
        OSTREAM trcos;
        trcos << showbase << hex;
        trcos << "  DevHndl=" << DevHndl;
        trcos << "  ArNumber=" << ArNumber;
        trcos << "  SessionKey=" << SessionKey;
        trcint_ShowPtr(trcos, pList, "pList");
        trcos << ends;
        TRC_OUT_OBJECT(GR_INIT, LV_FCTPUB2, trcos);
        );

    ICommon *pICommonInst = ICommon::get_instance(DevHndl);

    if(pICommonInst) {
        Ret = ((IDevice *) pICommonInst)->set_appl_state_ready(ArNumber,
            SessionKey, pList);
    } else {
        TRC_OUT(GR_INIT, LV_ERR, "  * ERROR getting device instance ");
        Ret = PNIO_ERR_WRONG_HND;
    }

    TRC_IF_ON_EXPR(GR_INIT, LV_FCTPUB1,
        OSTREAM trcos1;
        trcos1 << showbase << hex;
        trcos1 << "<-PNIO_set_appl_state_ready, ret=" << Ret;
        trcos1 << '\0';
        TRC_OUT_OBJECT(GR_INIT, LV_FCTPUB1, trcos1);
        );

    DPR_MUTEX_UNLOCK(PnioMutexDevice);

    return Ret;
}

PNIO_UINT32 PNIO_CODE_ATTR PNIO_device_ar_abort(PNIO_UINT32 DevHndl,
    PNIO_UINT16 ArNumber,
    PNIO_UINT16 SessionKey)
{
    PNIO_UINT32 Ret;

    DPR_MUTEX_LOCK(PnioMutexDevice);

    TRC_OUT(GR_INIT, LV_FCTPUB1, "->PNIO_device_ar_abort");

    TRC_IF_ON_EXPR(GR_INIT, LV_FCTPUB2,
        OSTREAM trcos;
        trcos << showbase << hex;
        trcos << "  DevHndl=" << DevHndl;
        trcos << "  ArNumber=" << ArNumber;
        trcos << "  SessionKey=" << SessionKey;
        trcos << ends;
        TRC_OUT_OBJECT(GR_INIT, LV_FCTPUB2, trcos);
        );

    ICommon *pICommonInst = ICommon::get_instance(DevHndl);

    if(pICommonInst) {
        Ret = ((IDevice *) pICommonInst)->device_ar_abort(ArNumber,
            SessionKey);
    } else {
        TRC_OUT(GR_INIT, LV_ERR, "  * ERROR getting device instance ");
        Ret = PNIO_ERR_WRONG_HND;
    }

    TRC_IF_ON_EXPR(GR_INIT, LV_FCTPUB1,
        OSTREAM trcos1;
        trcos1 << showbase << hex;
        trcos1 << "<-PNIO_device_ar_abort, ret=" << Ret;
        trcos1 << '\0';
        TRC_OUT_OBJECT(GR_INIT, LV_FCTPUB1, trcos1);
        );

    DPR_MUTEX_UNLOCK(PnioMutexDevice);

    return Ret;
}

PNIO_UINT32 PNIO_CODE_ATTR PNIO_device_start(PNIO_UINT32 DevHndl)
{
    PNIO_UINT32 Ret;

    DPR_MUTEX_LOCK(PnioMutexDevice);

    TRC_OUT(GR_INIT, LV_FCTPUB1, "->PNIO_device_start");

    TRC_IF_ON_EXPR(GR_INIT, LV_FCTPUB2,
        OSTREAM trcos;
        trcos << showbase << hex;
        trcos << "  DevHndl=" << DevHndl;
        trcos << ends;
        TRC_OUT_OBJECT(GR_INIT, LV_FCTPUB2, trcos);
        );

    ICommon *pICommonInst = ICommon::get_instance(DevHndl);

    if(pICommonInst) {
        Ret = ((IDevice *) pICommonInst)->device_start();
     } else {
        TRC_OUT(GR_INIT, LV_ERR, "  * ERROR getting device instance ");
        Ret = PNIO_ERR_WRONG_HND;
    }

    TRC_IF_ON_EXPR(GR_INIT, LV_FCTPUB1,
        OSTREAM trcos1;
        trcos1 << showbase << hex;
        trcos1 << "<-PNIO_device_start, ret=" << Ret;
        trcos1 << '\0';
        TRC_OUT_OBJECT(GR_INIT, LV_FCTPUB1, trcos1);
        );

    DPR_MUTEX_UNLOCK(PnioMutexDevice);

    return Ret;
}

PNIO_UINT32 PNIO_CODE_ATTR PNIO_device_stop(PNIO_UINT32 DevHndl)
{
    PNIO_UINT32 Ret;

    DPR_MUTEX_LOCK(PnioMutexDevice);

    TRC_OUT(GR_INIT, LV_FCTPUB1, "->PNIO_device_stop");

    TRC_IF_ON_EXPR(GR_INIT, LV_FCTPUB2,
        OSTREAM trcos;
        trcos << showbase << hex;
        trcos << "  DevHndl=" << DevHndl;
        trcos << ends;
        TRC_OUT_OBJECT(GR_INIT, LV_FCTPUB2, trcos);
        );

    ICommon *pICommonInst = ICommon::get_instance(DevHndl);

    if(pICommonInst) {
        Ret = ((IDevice *) pICommonInst)->device_stop();
     } else {
        TRC_OUT(GR_INIT, LV_ERR, "  * ERROR getting device instance ");
        Ret = PNIO_ERR_WRONG_HND;
    }

    TRC_IF_ON_EXPR(GR_INIT, LV_FCTPUB1,
        OSTREAM trcos1;
        trcos1 << showbase << hex;
        trcos1 << "<-PNIO_device_stop, ret=" << Ret;
        trcos1 << '\0';
        TRC_OUT_OBJECT(GR_INIT, LV_FCTPUB1, trcos1);
        );

    DPR_MUTEX_UNLOCK(PnioMutexDevice);

    return Ret;
}

/*===========================================================================
* FUNCTION : PNIO_set_dev_state
*----------------------------------------------------------------------------
* PURPOSE  : set device state to selected value
*----------------------------------------------------------------------------
* RETURNS  : PNIO_OK on success; else *
*----------------------------------------------------------------------------
* INPUTS   : - DevHndl -  Handle Handle from PNIO_device_open
*            - DevState - PNIO_DEVSTATE_OPERATE_OK
*                         PNIO_DEVSTATE_CLEAR_OK
*                         PNIO_DEVSTATE_CLEAR_STATION_PROBLEM
*                         PNIO_DEVSTATE_OPERATE_STATION_PROBLEM
* OUTPUS   : -
*----------------------------------------------------------------------------
* COMMENTS : With this function the application program changes
* a state of IO Device in PROFINET IO system
*==========================================================================*/
PNIO_UINT32 PNIO_CODE_ATTR
PNIO_set_dev_state(PNIO_UINT32 DevHndl,
    PNIO_UINT32 DevState)
{
    PNIO_UINT32 Ret;

    DPR_MUTEX_LOCK(PnioMutexDevice);

    TRC_OUT(GR_INIT, LV_FCTPUB1, "->PNIO_set_dev_state");

    TRC_IF_ON_EXPR(GR_INIT, LV_FCTPUB2,
        OSTREAM trcos;
        trcos << showbase << hex;
        trcos << "  DevHndl=" << DevHndl;
        trcos << "  DevState=" << DevState;
        trcos << ends;
        TRC_OUT_OBJECT(GR_INIT, LV_FCTPUB2, trcos);
        );

    ICommon *pICommonInst = ICommon::get_instance(DevHndl);

    if(pICommonInst) {
        Ret = ((IDevice *) pICommonInst)->set_dev_state(DevState);
    } else {
        TRC_OUT(GR_INIT, LV_ERR, "  * ERROR getting device instance ");
        Ret = PNIO_ERR_WRONG_HND;
    }

    TRC_IF_ON_EXPR(GR_INIT, LV_FCTPUB1,
        OSTREAM trcos1;
        trcos1 << showbase << hex;
        trcos1 << "<-PNIO_set_dev_state, ret=" << Ret;
        trcos1 << '\0';
        TRC_OUT_OBJECT(GR_INIT, LV_FCTPUB1, trcos1);
        );

    DPR_MUTEX_UNLOCK(PnioMutexDevice);

    return Ret;
}

PNIO_UINT32 PNIO_CODE_ATTR PNIO_api_add(PNIO_UINT32 DevHndl,
    PNIO_UINT32 Api,
    PNIO_UINT16 MaxnumSlots,
    PNIO_UINT16 MaxnumSubslots)
{
    PNIO_UINT32 Ret;

    DPR_MUTEX_LOCK(PnioMutexDevice);

    TRC_OUT(GR_INIT, LV_FCTPUB1, "->PNIO_api_add");

    TRC_IF_ON_EXPR(GR_INIT, LV_FCTPUB2,
        OSTREAM trcos;
        trcos << showbase << hex;
        trcos << "  DevHndl=" << DevHndl;
        trcos << "  Api=" << Api;
        trcos << "  MaxnumSlots=" << MaxnumSlots;
        trcos << "  MaxnumSubslots=" << MaxnumSubslots;
        trcos << ends;
        TRC_OUT_OBJECT(GR_INIT, LV_FCTPUB2, trcos);
        );

    ICommon *pICommonInst = ICommon::get_instance(DevHndl);

    if(pICommonInst) {
        Ret = ((IDevice *) pICommonInst)->api_add(Api,
            MaxnumSlots, MaxnumSubslots);
    } else {
        TRC_OUT(GR_INIT, LV_ERR, "  * ERROR getting device instance ");
        Ret = PNIO_ERR_WRONG_HND;
    }

    TRC_IF_ON_EXPR(GR_INIT, LV_FCTPUB1,
        OSTREAM trcos1;
        trcos1 << showbase << hex;
        trcos1 << "<-PNIO_api_add, ret=" << Ret;
        trcos1 << '\0';
        TRC_OUT_OBJECT(GR_INIT, LV_FCTPUB1, trcos1);
        );

    DPR_MUTEX_UNLOCK(PnioMutexDevice);

    return Ret;
}

PNIO_UINT32 PNIO_CODE_ATTR PNIO_api_remove(PNIO_UINT32 DevHndl,
    PNIO_UINT32 Api)
{
    PNIO_UINT32 Ret;

    DPR_MUTEX_LOCK(PnioMutexDevice);

    TRC_OUT(GR_INIT, LV_FCTPUB1, "->PNIO_api_remove");

    TRC_IF_ON_EXPR(GR_INIT, LV_FCTPUB2,
        OSTREAM trcos;
        trcos << showbase << hex;
        trcos << "  DevHndl=" << DevHndl;
        trcos << "  Api=" << Api;
        trcos << ends;
        TRC_OUT_OBJECT(GR_INIT, LV_FCTPUB2, trcos);
        );

    ICommon *pICommonInst = ICommon::get_instance(DevHndl);

    if(pICommonInst) {
        Ret = ((IDevice *) pICommonInst)->api_remove(Api);
    } else {
        TRC_OUT(GR_INIT, LV_ERR, "  * ERROR getting device instance ");
        Ret = PNIO_ERR_WRONG_HND;
    }

    TRC_IF_ON_EXPR(GR_INIT, LV_FCTPUB1,
        OSTREAM trcos1;
        trcos1 << showbase << hex;
        trcos1 << "<-PNIO_api_remove, ret=" << Ret;
        trcos1 << '\0';
        TRC_OUT_OBJECT(GR_INIT, LV_FCTPUB1, trcos1);
        );

    DPR_MUTEX_UNLOCK(PnioMutexDevice);

    return Ret;
}

PNIO_UINT32 PNIO_CODE_ATTR PNIO_mod_pull(PNIO_UINT32 DevHndl,
    PNIO_UINT32 Api,
    PNIO_DEV_ADDR * pAddr)
{
    PNIO_UINT32 Ret;

    DPR_MUTEX_LOCK(PnioMutexDevice);

    TRC_OUT(GR_INIT, LV_FCTPUB1, "->PNIO_mod_pull");

    TRC_IF_ON_EXPR(GR_INIT, LV_FCTPUB2,
        OSTREAM trcos;
        trcos << showbase << hex;
        trcos << "  DevHndl=" << DevHndl;
        trcos << "  Api=" << Api;
        trcos << ", pAddr=";
        trcint_ShowDAddr(trcos, pAddr);
        trcos << ends;
        TRC_OUT_OBJECT(GR_INIT, LV_FCTPUB2, trcos);
        );

    ICommon *pICommonInst = ICommon::get_instance(DevHndl);

    if(pICommonInst) {
        Ret = ((IDevice *) pICommonInst)->mod_pull(Api, pAddr);
    } else {
        TRC_OUT(GR_INIT, LV_ERR, "  * ERROR getting device instance ");
        Ret = PNIO_ERR_WRONG_HND;
    }

    TRC_IF_ON_EXPR(GR_INIT, LV_FCTPUB1,
        OSTREAM trcos1;
        trcos1 << showbase << hex;
        trcos1 << "<-PNIO_mod_pull, ret=" << Ret;
        trcos1 << '\0';
        TRC_OUT_OBJECT(GR_INIT, LV_FCTPUB1, trcos1);
        );

    DPR_MUTEX_UNLOCK(PnioMutexDevice);

    return Ret;
}

PNIO_UINT32 PNIO_CODE_ATTR PNIO_sub_pull(PNIO_UINT32 DevHndl,
    PNIO_UINT32 Api,
    PNIO_DEV_ADDR * pAddr)
{
    PNIO_UINT32 Ret;

    DPR_MUTEX_LOCK(PnioMutexDevice);

    TRC_OUT(GR_INIT, LV_FCTPUB1, "->PNIO_sub_pull");

    TRC_IF_ON_EXPR(GR_INIT, LV_FCTPUB2,
        OSTREAM trcos;
        trcos << showbase << hex;
        trcos << "  DevHndl=" << DevHndl;
        trcos << "  Api=" << Api;
        trcos << ", pAddr=";
        trcint_ShowDAddr(trcos, pAddr);
        trcos << ends;
        TRC_OUT_OBJECT(GR_INIT, LV_FCTPUB2, trcos);
        );

    ICommon *pICommonInst = ICommon::get_instance(DevHndl);

    if(pICommonInst) {
        Ret = ((IDevice *) pICommonInst)->sub_pull(Api, pAddr);
    } else {
        TRC_OUT(GR_INIT, LV_ERR, "  * ERROR getting device instance ");
        Ret = PNIO_ERR_WRONG_HND;
    }

    TRC_IF_ON_EXPR(GR_INIT, LV_FCTPUB1,
        OSTREAM trcos1;
        trcos1 << showbase << hex;
        trcos1 << "<-PNIO_sub_pull, ret=" << Ret;
        trcos1 << '\0';
        TRC_OUT_OBJECT(GR_INIT, LV_FCTPUB1, trcos1);
        );

    DPR_MUTEX_UNLOCK(PnioMutexDevice);

    return Ret;
}

PNIO_UINT32 PNIO_CODE_ATTR PNIO_sub_plug(PNIO_UINT32 DevHndl,
    PNIO_UINT32 Api,
    PNIO_DEV_ADDR * pAddr,
    PNIO_UINT32 SubIdent)
{
    PNIO_UINT32 Ret;

    DPR_MUTEX_LOCK(PnioMutexDevice);

    TRC_OUT(GR_INIT, LV_FCTPUB1, "->PNIO_sub_plug");

    TRC_IF_ON_EXPR(GR_INIT, LV_FCTPUB2,
        OSTREAM trcos;
        trcos << showbase << hex;
        trcos << "  DevHndl=" << DevHndl;
        trcos << "  Api=" << Api;
        trcos << ", pAddr=";
        trcint_ShowDAddr(trcos, pAddr);
        trcos << ", SubIdent=" << SubIdent;
        trcos << ends;
        TRC_OUT_OBJECT(GR_INIT, LV_FCTPUB2, trcos);
        );

    ICommon *pICommonInst = ICommon::get_instance(DevHndl);

    if(pICommonInst) {
        Ret = ((IDevice *) pICommonInst)->sub_plug(Api, pAddr, SubIdent);
    } else {
        TRC_OUT(GR_INIT, LV_ERR, "  * ERROR getting device instance ");
        Ret = PNIO_ERR_WRONG_HND;
    }

    TRC_IF_ON_EXPR(GR_INIT, LV_FCTPUB1,
        OSTREAM trcos1;
        trcos1 << showbase << hex;
        trcos1 << "<-PNIO_sub_plug, ret=" << Ret;
        trcos1 << '\0';
        TRC_OUT_OBJECT(GR_INIT, LV_FCTPUB1, trcos1);
        );

    DPR_MUTEX_UNLOCK(PnioMutexDevice);

    return Ret;
}

PNIO_UINT32 PNIO_CODE_ATTR PNIO_sub_plug_ext(
        PNIO_UINT32          DevHndl,
        PNIO_UINT32          Api,
        PNIO_DEV_ADDR      * pAddr,
        PNIO_UINT32          SubIdent,
        PNIO_UINT32          AlarmType)
{
    PNIO_UINT32 Ret;

    DPR_MUTEX_LOCK(PnioMutexDevice);

    TRC_OUT(GR_INIT, LV_FCTPUB1, "->PNIO_sub_plug_ext");

    TRC_IF_ON_EXPR(GR_INIT, LV_FCTPUB2,
        OSTREAM trcos;
        trcos << showbase << hex;
        trcos << "  DevHndl=" << DevHndl;
        trcos << "  Api=" << Api;
        trcos << ", pAddr=";
        trcint_ShowDAddr(trcos, pAddr);
        trcos << ", SubIdent=" << SubIdent;
        trcos << ", AlarmType=" << AlarmType;
        trcos << ends;
        TRC_OUT_OBJECT(GR_INIT, LV_FCTPUB2, trcos);
        );

    ICommon *pICommonInst = ICommon::get_instance(DevHndl);

    if(pICommonInst) {
        Ret = ((IDevice *) pICommonInst)->sub_plug_ext(Api, pAddr, SubIdent, AlarmType);
    } else {
        TRC_OUT(GR_INIT, LV_ERR, "  * ERROR getting device instance ");
        Ret = PNIO_ERR_WRONG_HND;
    }

    TRC_IF_ON_EXPR(GR_INIT, LV_FCTPUB1,
        OSTREAM trcos1;
        trcos1 << showbase << hex;
        trcos1 << "<-PNIO_sub_plug_ext, ret=" << Ret;
        trcos1 << '\0';
        TRC_OUT_OBJECT(GR_INIT, LV_FCTPUB1, trcos1);
        );

    DPR_MUTEX_UNLOCK(PnioMutexDevice);

    return Ret;
}

PNIO_UINT32 PNIO_CODE_ATTR PNIO_sub_plug_ext_IM(
        PNIO_UINT32          DevHndl,
        PNIO_UINT32          Api,
        PNIO_DEV_ADDR      * pAddr,
        PNIO_UINT32          SubIdent,
        PNIO_UINT32          AlarmType,
        PNIO_PLUG_IM0_BITS   IM0_bits)
{
    PNIO_UINT32 Ret;

    DPR_MUTEX_LOCK(PnioMutexDevice);

    TRC_OUT(GR_INIT, LV_FCTPUB1, "->PNIO_sub_plug_ext_IM");

    TRC_IF_ON_EXPR(GR_INIT, LV_FCTPUB2,
        OSTREAM trcos;
        trcos << showbase << hex;
        trcos << "  DevHndl=" << DevHndl;
        trcos << "  Api=" << Api;
        trcos << ", pAddr=";
        trcint_ShowDAddr(trcos, pAddr);
        trcos << ", SubIdent=" << SubIdent;
        trcos << ", AlarmType=" << AlarmType;
        trcos << ", IM0_bits=" << IM0_bits;
        trcos << ends;
        TRC_OUT_OBJECT(GR_INIT, LV_FCTPUB2, trcos);
        );

    ICommon *pICommonInst = ICommon::get_instance(DevHndl);

    if(pICommonInst) {
        Ret = ((IDevice *) pICommonInst)->sub_plug_ext_im(Api, pAddr, SubIdent, AlarmType, IM0_bits);
    } else {
        TRC_OUT(GR_INIT, LV_ERR, "  * ERROR getting device instance ");
        Ret = PNIO_ERR_WRONG_HND;
    }

    TRC_IF_ON_EXPR(GR_INIT, LV_FCTPUB1,
        OSTREAM trcos1;
        trcos1 << showbase << hex;
        trcos1 << "<-PNIO_sub_plug_ext_IM, ret=" << Ret;
        trcos1 << '\0';
        TRC_OUT_OBJECT(GR_INIT, LV_FCTPUB1, trcos1);
        );

    DPR_MUTEX_UNLOCK(PnioMutexDevice);

    return Ret;
}

PNIO_UINT32 PNIO_CODE_ATTR PNIO_mod_plug(PNIO_UINT32 DevHndl,
    PNIO_UINT32 Api,
    PNIO_DEV_ADDR * pAddr,
    PNIO_UINT32 ModIdent)
{
    PNIO_UINT32 Ret;

    DPR_MUTEX_LOCK(PnioMutexDevice);

    TRC_OUT(GR_INIT, LV_FCTPUB1, "->PNIO_mod_plug");

    TRC_IF_ON_EXPR(GR_INIT, LV_FCTPUB2,
        OSTREAM trcos;
        trcos << showbase << hex;
        trcos << "  DevHndl=" << DevHndl;
        trcos << "  Api=" << Api;
        trcos << ", pAddr=";
        trcint_ShowDAddr(trcos, pAddr);
        trcos << ", ModIdent=" << ModIdent;
        trcos << ends;
        TRC_OUT_OBJECT(GR_INIT, LV_FCTPUB2, trcos);
        );

    ICommon *pICommonInst = ICommon::get_instance(DevHndl);

    if(pICommonInst) {
        Ret = ((IDevice *) pICommonInst)->mod_plug(Api,
            pAddr, ModIdent);
    } else {
        TRC_OUT(GR_INIT, LV_ERR, "  * ERROR getting device instance ");
        Ret = PNIO_ERR_WRONG_HND;
    }

    TRC_IF_ON_EXPR(GR_INIT, LV_FCTPUB1,
        OSTREAM trcos1;
        trcos1 << showbase << hex;
        trcos1 << "<-PNIO_mod_plug, ret=" << Ret;
        trcos1 << '\0';
        TRC_OUT_OBJECT(GR_INIT, LV_FCTPUB1, trcos1);
        );

    DPR_MUTEX_UNLOCK(PnioMutexDevice);

    return Ret;
}

// *-----------------------------------------------
// * set diagnostic data in pnio stack
// *-----------------------------------------------
PNIO_UINT16 PNIO_CODE_ATTR PNIO_build_channel_properties(PNIO_UINT32 DevHndl,
    PNIO_UINT16 Type,
    PNIO_UINT16 Spec,
    PNIO_UINT16 Dir)
{
    PNIO_UINT16 RetProperty;

    DPR_MUTEX_LOCK(PnioMutexDevice);

    TRC_OUT(GR_INIT, LV_FCTPUB1, "->PNIO_build_channel_properties");

    TRC_IF_ON_EXPR(GR_INIT, LV_FCTPUB2,
        OSTREAM trcos;
        trcos << showbase << hex;
        trcos << "  DevHndl=" << DevHndl;
        trcos << "  Type=" << Type;
        trcos << ", Spec=" << Spec;
        trcos << ", Dir=" << Dir;
        trcos << ends;
        TRC_OUT_OBJECT(GR_INIT, LV_FCTPUB2, trcos);
        );

    ICommon *pICommonInst = ICommon::get_instance(DevHndl);

    if(pICommonInst) {
        RetProperty = ((IDevice *) pICommonInst)->build_channel_properties(Type,
            Spec, Dir);
    } else {
        TRC_OUT(GR_INIT, LV_ERR, "  * ERROR getting device instance ");
        RetProperty = 0;
    }

    TRC_IF_ON_EXPR(GR_INIT, LV_FCTPUB1,
        OSTREAM trcos1;
        trcos1 << showbase << hex;
        trcos1 << "<-PNIO_build_channel_properties, ret=" << RetProperty;
        trcos1 << '\0';
        TRC_OUT_OBJECT(GR_INIT, LV_FCTPUB1, trcos1);
        );

    DPR_MUTEX_UNLOCK(PnioMutexDevice);

    return RetProperty;
}

PNIO_UINT32 PNIO_CODE_ATTR PNIO_diag_channel_add(PNIO_UINT32 DevHndl,
    PNIO_UINT32 Api,
    PNIO_DEV_ADDR * pAddr,
    PNIO_UINT16 ChannelNum,
    PNIO_UINT16 ChannelProp,
    PNIO_UINT32 ChannelErrType,
    PNIO_UINT16 DiagTag)
{
    PNIO_UINT32 Ret;

    DPR_MUTEX_LOCK(PnioMutexDevice);

    TRC_OUT(GR_INIT, LV_FCTPUB1, "->PNIO_diag_channel_add");

    TRC_IF_ON_EXPR(GR_INIT, LV_FCTPUB2,
        OSTREAM trcos;
        trcos << showbase << hex;
        trcos << "  DevHndl=" << DevHndl;
        trcos << "  Api=" << Api;
        trcos << ", pAddr=";
        trcint_ShowDAddr(trcos, pAddr);
        trcos << ", ChannelNum=" << ChannelNum;
        trcos << ", ChannelProp=" << ChannelProp;
        trcos << ", ChannelErrType=" << ChannelErrType;
        trcos << ", DiagTag=" << DiagTag;
        trcos << ends;
        TRC_OUT_OBJECT(GR_INIT, LV_FCTPUB2, trcos);
        );

    ICommon *pICommonInst = ICommon::get_instance(DevHndl);

    if(pICommonInst) {
        Ret = ((IDevice *) pICommonInst)->diag_channel_add(Api,
            pAddr, ChannelNum, ChannelProp, ChannelErrType, DiagTag);
    } else {
        TRC_OUT(GR_INIT, LV_ERR, "  * ERROR getting device instance ");
        Ret = PNIO_ERR_WRONG_HND;
    }

    TRC_IF_ON_EXPR(GR_INIT, LV_FCTPUB1,
        OSTREAM trcos1;
        trcos1 << showbase << hex;
        trcos1 << "<-PNIO_diag_channel_add, ret=" << Ret;
        trcos1 << '\0';
        TRC_OUT_OBJECT(GR_INIT, LV_FCTPUB1, trcos1);
        );

    DPR_MUTEX_UNLOCK(PnioMutexDevice);

    return Ret;
}

PNIO_UINT32 PNIO_CODE_ATTR PNIO_diag_channel_remove(PNIO_UINT32 DevHndl,
    PNIO_UINT32 Api,
    PNIO_DEV_ADDR * pAddr,
    PNIO_UINT16 DiagTag)
{
    PNIO_UINT32 Ret;

    DPR_MUTEX_LOCK(PnioMutexDevice);

    TRC_OUT(GR_INIT, LV_FCTPUB1, "->PNIO_diag_channel_remove");

    TRC_IF_ON_EXPR(GR_INIT, LV_FCTPUB2,
        OSTREAM trcos;
        trcos << showbase << hex;
        trcos << "  DevHndl=" << DevHndl;
        trcos << "  Api=" << Api;
        trcos << ", pAddr=";
        trcint_ShowDAddr(trcos, pAddr);
        trcos << ", DiagTag=" << DiagTag;
        trcos << ends;
        TRC_OUT_OBJECT(GR_INIT, LV_FCTPUB2, trcos);
        );

    ICommon *pICommonInst = ICommon::get_instance(DevHndl);

    if(pICommonInst) {
        Ret = ((IDevice *) pICommonInst)->diag_remove(Api,
            pAddr, DiagTag, 0 /* channel */);
    } else {
        TRC_OUT(GR_INIT, LV_ERR, "  * ERROR getting device instance ");
        Ret = PNIO_ERR_WRONG_HND;
    }

    TRC_IF_ON_EXPR(GR_INIT, LV_FCTPUB1,
        OSTREAM trcos1;
        trcos1 << showbase << hex;
        trcos1 << "<-PNIO_diag_channel_remove, ret=" << Ret;
        trcos1 << '\0';
        TRC_OUT_OBJECT(GR_INIT, LV_FCTPUB1, trcos1);
        );

    DPR_MUTEX_UNLOCK(PnioMutexDevice);

    return Ret;
}

PNIO_UINT32 PNIO_CODE_ATTR PNIO_diag_generic_add(PNIO_UINT32 DevHndl,
    PNIO_UINT32 Api,
    PNIO_DEV_ADDR * pAddr,
    PNIO_UINT16 ChannelNum,
    PNIO_UINT16 ChannelProp,
    PNIO_UINT16 DiagTag,
    PNIO_UINT16 UserStructIdent,
    PNIO_UINT8 * pInfoData,
    PNIO_UINT32 InfoDataLen)
{
    PNIO_UINT32 Ret;

    DPR_MUTEX_LOCK(PnioMutexDevice);

    TRC_OUT(GR_INIT, LV_FCTPUB1, "->PNIO_diag_generic_add");

    TRC_IF_ON_EXPR(GR_INIT, LV_FCTPUB2,
        OSTREAM trcos;
        trcos << showbase << hex;
        trcos << "  DevHndl=" << DevHndl;
        trcos << "  Api=" << Api;
        trcos << ", pAddr=";
        trcint_ShowDAddr(trcos, pAddr);
        trcos << ", ChannelNum=" << ChannelNum;
        trcos << ", ChannelProp=" << ChannelProp;
        trcos << ", DiagTag=" << DiagTag;
        trcos << ", UserStructIdent=" << UserStructIdent;
        trcos << ", pInfoData=";
        trcint_ShowData(trcos, InfoDataLen, pInfoData);
        trcos << ", InfoDataLen=" << InfoDataLen;
        trcos << ends;
        TRC_OUT_OBJECT(GR_INIT, LV_FCTPUB2, trcos);
        );

    ICommon *pICommonInst = ICommon::get_instance(DevHndl);

    if(pICommonInst) {
        Ret = ((IDevice *) pICommonInst)->diag_generic_add(Api,
            pAddr, ChannelNum, ChannelProp, DiagTag,
            UserStructIdent, pInfoData, InfoDataLen);
    } else {
        TRC_OUT(GR_INIT, LV_ERR, "  * ERROR getting device instance ");
        Ret = PNIO_ERR_WRONG_HND;
    }

    TRC_IF_ON_EXPR(GR_INIT, LV_FCTPUB1,
        OSTREAM trcos1;
        trcos1 << showbase << hex;
        trcos1 << "<-PNIO_diag_generic_add, ret=" << Ret;
        trcos1 << '\0';
        TRC_OUT_OBJECT(GR_INIT, LV_FCTPUB1, trcos1);
        );

    DPR_MUTEX_UNLOCK(PnioMutexDevice);

    return Ret;
}

PNIO_UINT32 PNIO_CODE_ATTR PNIO_diag_ext_channel_add(PNIO_UINT32 DevHndl,
    PNIO_UINT32 Api,
    PNIO_DEV_ADDR * pAddr,
    PNIO_UINT16 ChannelNum,
    PNIO_UINT16 ChannelProp,
    PNIO_UINT16 ChannelErrType,
    PNIO_UINT16 ExtChannelErrType,
        PNIO_UINT32 ExtChannelAddValue,
    PNIO_UINT16 DiagTag)
{
    PNIO_UINT32 Ret;

    DPR_MUTEX_LOCK(PnioMutexDevice);

    TRC_OUT(GR_INIT, LV_FCTPUB1, "->PNIO_diag_ext_channel_add");

    TRC_IF_ON_EXPR(GR_INIT, LV_FCTPUB2,
        OSTREAM trcos;
        trcos << showbase << hex;
        trcos << "  DevHndl=" << DevHndl;
        trcos << "  Api=" << Api;
        trcos << ", pAddr=";
        trcint_ShowDAddr(trcos, pAddr);
        trcos << ", ChannelNum=" << ChannelNum;
        trcos << ", ChannelProp=" << ChannelProp;
        trcos << ", ChannelErrType=" << ChannelErrType;
        trcos << ", ExtChannelErrType=" << ExtChannelErrType;
        trcos << ", ExtChannelAddValue=" << ExtChannelAddValue;
        trcos << ", DiagTag=" << DiagTag;
        trcos << ends;
        TRC_OUT_OBJECT(GR_INIT, LV_FCTPUB2, trcos);
        );

    ICommon *pICommonInst = ICommon::get_instance(DevHndl);

    if(pICommonInst) {
        Ret = ((IDevice *) pICommonInst)->diag_ext_channel_add(Api,
            pAddr, ChannelNum, ChannelProp, ChannelErrType,
                     ExtChannelErrType, ExtChannelAddValue, DiagTag);
    } else {
        TRC_OUT(GR_INIT, LV_ERR, "  * ERROR getting device instance ");
        Ret = PNIO_ERR_WRONG_HND;
    }

    TRC_IF_ON_EXPR(GR_INIT, LV_FCTPUB1,
        OSTREAM trcos1;
        trcos1 << showbase << hex;
        trcos1 << "<-PNIO_diag_ext_channel_add, ret=" << Ret;
        trcos1 << '\0';
        TRC_OUT_OBJECT(GR_INIT, LV_FCTPUB1, trcos1);
        );

    DPR_MUTEX_UNLOCK(PnioMutexDevice);

    return Ret;
}

PNIO_UINT32 PNIO_CODE_ATTR PNIO_diag_ext_channel_remove(PNIO_UINT32 DevHndl,
    PNIO_UINT32 Api,
    PNIO_DEV_ADDR * pAddr,
    PNIO_UINT16 DiagTag)
{
    PNIO_UINT32 Ret;

    DPR_MUTEX_LOCK(PnioMutexDevice);

    TRC_OUT(GR_INIT, LV_FCTPUB1, "->PNIO_diag_ext_channel_remove");

    TRC_IF_ON_EXPR(GR_INIT, LV_FCTPUB2,
        OSTREAM trcos;
        trcos << showbase << hex;
        trcos << "  DevHndl=" << DevHndl;
        trcos << "  Api=" << Api;
        trcos << ", pAddr=";
        trcint_ShowDAddr(trcos, pAddr);
        trcos << ", DiagTag=" << DiagTag;
        trcos << ends;
        TRC_OUT_OBJECT(GR_INIT, LV_FCTPUB2, trcos);
        );

    ICommon *pICommonInst = ICommon::get_instance(DevHndl);

    if(pICommonInst) {
        Ret = ((IDevice *) pICommonInst)->diag_remove(Api,
            pAddr, DiagTag, 3 /* ext_channel */);
    } else {
        TRC_OUT(GR_INIT, LV_ERR, "  * ERROR getting device instance ");
        Ret = PNIO_ERR_WRONG_HND;
    }

    TRC_IF_ON_EXPR(GR_INIT, LV_FCTPUB1,
        OSTREAM trcos1;
        trcos1 << showbase << hex;
        trcos1 << "<-PNIO_diag_ext_channel_remove, ret=" << Ret;
        trcos1 << '\0';
        TRC_OUT_OBJECT(GR_INIT, LV_FCTPUB1, trcos1);
        );

    DPR_MUTEX_UNLOCK(PnioMutexDevice);

    return Ret;
}

PNIO_UINT32 PNIO_CODE_ATTR PNIO_diag_generic_remove(PNIO_UINT32 DevHndl,
    PNIO_UINT32 Api,
    PNIO_DEV_ADDR * pAddr,
    PNIO_UINT16 DiagTag)
{
    PNIO_UINT32 Ret;

    DPR_MUTEX_LOCK(PnioMutexDevice);

    TRC_OUT(GR_INIT, LV_FCTPUB1, "->PNIO_diag_generic_remove");

    TRC_IF_ON_EXPR(GR_INIT, LV_FCTPUB2,
        OSTREAM trcos;
        trcos << showbase << hex;
        trcos << "  DevHndl=" << DevHndl;
        trcos << "  Api=" << Api;
        trcos << ", pAddr=";
        trcint_ShowDAddr(trcos, pAddr);
        trcos << ", DiagTag=" << DiagTag;
        trcos << ends;
        TRC_OUT_OBJECT(GR_INIT, LV_FCTPUB2, trcos);
        );

    ICommon *pICommonInst = ICommon::get_instance(DevHndl);

    if(pICommonInst) {
        Ret = ((IDevice *) pICommonInst)->diag_remove(Api,
            pAddr, DiagTag, 1 /* generic */);
    } else {
        TRC_OUT(GR_INIT, LV_ERR, "  * ERROR getting device instance ");
        Ret = PNIO_ERR_WRONG_HND;
    }

    TRC_IF_ON_EXPR(GR_INIT, LV_FCTPUB1,
        OSTREAM trcos1;
        trcos1 << showbase << hex;
        trcos1 << "<-PNIO_diag_generic_remove, ret=" << Ret;
        trcos1 << '\0';
        TRC_OUT_OBJECT(GR_INIT, LV_FCTPUB1, trcos1);
        );

    DPR_MUTEX_UNLOCK(PnioMutexDevice);

    return Ret;
}

PNIO_UINT32 PNIO_CODE_ATTR PNIO_process_alarm_send(PNIO_UINT32 DevHndl,
    PNIO_UINT32 Api,
    PNIO_UINT16 ArNumber,
    PNIO_UINT16 SessionKey,
    PNIO_DEV_ADDR * pAddr,
    PNIO_UINT8 * pData,
    PNIO_UINT32 DataLen,
    PNIO_UINT16 UserStructIdent,
    PNIO_UINT32 UserHndl)
{
    PNIO_UINT32 Ret;

    DPR_MUTEX_LOCK(PnioMutexDevice);

    TRC_OUT(GR_INIT, LV_FCTPUB1, "->PNIO_process_alarm_send");

    TRC_IF_ON_EXPR(GR_INIT, LV_FCTPUB2,
        OSTREAM trcos;
        trcos << showbase << hex;
        trcos << "  DevHndl=" << DevHndl;
        trcos << "  Api=" << Api;
        trcos << "  ArNumber=" << ArNumber;
        trcos << "  SessionKey=" << SessionKey;
        trcos << ", pAddr=";
        trcint_ShowDAddr(trcos, pAddr);
        trcos << ", pData=";
        trcint_ShowData(trcos, DataLen, pData);
        trcos << ", DataLen=" << DataLen;
        trcos << ", UserStructIdent=" << UserStructIdent;
        trcos << ", UserHndl=" << UserHndl;
        trcos << ends;
        TRC_OUT_OBJECT(GR_INIT, LV_FCTPUB2, trcos);
        );

    ICommon *pICommonInst = ICommon::get_instance(DevHndl);

    if(pICommonInst) {
        Ret = ((IDevice *) pICommonInst)->process_alarm_send(Api,
            ArNumber, SessionKey, pAddr, pData, DataLen,
            UserStructIdent, UserHndl);
    } else {
        TRC_OUT(GR_INIT, LV_ERR, "  * ERROR getting device instance ");
        Ret = PNIO_ERR_WRONG_HND;
    }

    TRC_IF_ON_EXPR(GR_INIT, LV_FCTPUB1,
        OSTREAM trcos1;
        trcos1 << showbase << hex;
        trcos1 << "<-PNIO_process_alarm_send, ret=" << Ret;
        trcos1 << '\0';
        TRC_OUT_OBJECT(GR_INIT, LV_FCTPUB1, trcos1);
        );

    DPR_MUTEX_UNLOCK(PnioMutexDevice);

    return Ret;
}

PNIO_UINT32 PNIO_CODE_ATTR PNIO_diag_alarm_send(PNIO_UINT32 DevHndl,
    PNIO_UINT32 Api,
    PNIO_UINT16 ArNumber,
    PNIO_UINT16 SessionKey,
    PNIO_UINT32 AlarmState,     /* alarm appears/disappears */
    PNIO_DEV_ADDR * pAddr,
    PNIO_UINT8 * pData,
    PNIO_UINT32 DataLen,
    PNIO_UINT16 UserStructIdent,
    PNIO_UINT32 UserHndl)
{
    PNIO_UINT32 Ret;

    DPR_MUTEX_LOCK(PnioMutexDevice);

    TRC_OUT(GR_INIT, LV_FCTPUB1, "->PNIO_diag_alarm_send");

    TRC_IF_ON_EXPR(GR_INIT, LV_FCTPUB2,
        OSTREAM trcos;
        trcos << showbase << hex;
        trcos << "  DevHndl=" << DevHndl;
        trcos << "  Api=" << Api;
        trcos << "  ArNumber=" << ArNumber;
        trcos << "  SessionKey=" << SessionKey;
        trcos << "  AlarmState=" << AlarmState;
        trcos << ", pAddr=";
        trcint_ShowDAddr(trcos, pAddr);
        trcos << ", pData=";
        trcint_ShowData(trcos, DataLen, pData);
        trcos << ", DataLen=" << DataLen;
        trcos << ", UserStructIdent=" << UserStructIdent;
        trcos << ", UserHndl=" << UserHndl;
        trcos << ends;
        TRC_OUT_OBJECT(GR_INIT, LV_FCTPUB2, trcos);
        );

    ICommon *pICommonInst = ICommon::get_instance(DevHndl);

    if(pICommonInst) {
        Ret = ((IDevice *) pICommonInst)->diag_alarm_send(Api,
            ArNumber, SessionKey, AlarmState, pAddr, pData,
            DataLen, UserStructIdent, UserHndl);
    } else {
        TRC_OUT(GR_INIT, LV_ERR, "  * ERROR getting device instance ");
        Ret = PNIO_ERR_WRONG_HND;
    }

    TRC_IF_ON_EXPR(GR_INIT, LV_FCTPUB1,
        OSTREAM trcos1;
        trcos1 << showbase << hex;
        trcos1 << "<-PNIO_diag_alarm_send, ret=" << Ret;
        trcos1 << '\0';
        TRC_OUT_OBJECT(GR_INIT, LV_FCTPUB1, trcos1);
        );

    DPR_MUTEX_UNLOCK(PnioMutexDevice);

    return Ret;
}

PNIO_UINT32 PNIO_CODE_ATTR PNIO_ret_of_sub_alarm_send(
    PNIO_UINT32 DevHndl,
    PNIO_UINT32 Api,
    PNIO_UINT16 ArNumber,
    PNIO_UINT16 SessionKey,
    PNIO_DEV_ADDR *pAddr,
    PNIO_UINT32 UserHndl)
{
    PNIO_UINT32 Ret;

    DPR_MUTEX_LOCK(PnioMutexDevice);

    TRC_OUT(GR_INIT, LV_FCTPUB1, "->PNIO_ret_of_sub_alarm_send");

    TRC_IF_ON_EXPR(GR_INIT, LV_FCTPUB2,
        OSTREAM trcos;
        trcos << showbase << hex;
        trcos << "  DevHndl=" << DevHndl;
        trcos << "  Api=" << Api;
        trcos << "  ArNumber=" << ArNumber;
        trcos << "  SessionKey=" << SessionKey;
        trcos << ", pAddr=";
        trcint_ShowDAddr(trcos, pAddr);
        trcos << ", UserHndl=" << UserHndl;
        trcos << ends;
        TRC_OUT_OBJECT(GR_INIT, LV_FCTPUB2, trcos);
        );

    ICommon *pICommonInst = ICommon::get_instance(DevHndl);

    if(pICommonInst) {
        Ret = ((IDevice *) pICommonInst)->ret_of_sub_alarm_send(Api,
            ArNumber, SessionKey, pAddr, UserHndl);
    } else {
        TRC_OUT(GR_INIT, LV_ERR, "  * ERROR getting device instance ");
        Ret = PNIO_ERR_WRONG_HND;
    }

    TRC_IF_ON_EXPR(GR_INIT, LV_FCTPUB1,
        OSTREAM trcos1;
        trcos1 << showbase << hex;
        trcos1 << "<-PNIO_ret_of_sub_alarm_send, ret=" << Ret;
        trcos1 << '\0';
        TRC_OUT_OBJECT(GR_INIT, LV_FCTPUB1, trcos1);
        );

    DPR_MUTEX_UNLOCK(PnioMutexDevice);

    return Ret;
}

// *-------------------------------------------------
// * data exchange functions, called by the user
// *-------------------------------------------------
PNIO_UINT32 PNIO_CODE_ATTR PNIO_initiate_data_read(PNIO_UINT32 DevHndl)
{
    PNIO_UINT32 Ret;

    /*
    this function is reentrant
    DPR_MUTEX_LOCK(PnioMutexDevice);
    */

    TRC_OUT(GR_IO, LV_FCTPUB1, "->PNIO_initiate_data_read");

    TRC_IF_ON_EXPR(GR_IO, LV_FCTPUB2,
        OSTREAM trcos;
        trcos << showbase << hex;
        trcos << "  DevHndl=" << DevHndl;
        trcos << ends;
        TRC_OUT_OBJECT(GR_IO, LV_FCTPUB2, trcos);
        );

    ICommon *pICommonInst = ICommon::get_instance(DevHndl);

    if(pICommonInst) {
        Ret = ((IDevice *) pICommonInst)->initiate_data_read(DevHndl);
        if(Ret != PNIO_OK) {
            TRC_OUT(GR_IO, LV_ERR, "  * initiate_data_read failed ");
        }
    } else {
        TRC_OUT(GR_IO, LV_ERR, "  * ERROR getting device instance ");
        Ret = PNIO_ERR_WRONG_HND;
    }

    TRC_IF_ON_EXPR(GR_IO, LV_FCTPUB1,
        OSTREAM trcos1;
        trcos1 << showbase << hex;
        trcos1 << "<-PNIO_initiate_data_read, ret=" << Ret;
        trcos1 << '\0';
        TRC_OUT_OBJECT(GR_IO, LV_FCTPUB1, trcos1);
        );

    /*
    DPR_MUTEX_UNLOCK(PnioMutexDevice);
    */

    return Ret;
}

PNIO_UINT32 PNIO_CODE_ATTR PNIO_initiate_data_write(PNIO_UINT32 DevHndl)
{
    PNIO_UINT32 Ret;

    /*
    this function is reentrant
    DPR_MUTEX_LOCK(PnioMutexDevice);
    */

    TRC_OUT(GR_IO, LV_FCTPUB1, "->PNIO_initiate_data_write");

    TRC_IF_ON_EXPR(GR_IO, LV_FCTPUB2,
        OSTREAM trcos;
        trcos << showbase << hex;
        trcos << "  DevHndl=" << DevHndl;
        trcos << ends;
        TRC_OUT_OBJECT(GR_IO, LV_FCTPUB2, trcos);
        );

    ICommon *pICommonInst = ICommon::get_instance(DevHndl);

    if(pICommonInst) {
        Ret = ((IDevice *) pICommonInst)->initiate_data_write(DevHndl);
    } else {
        TRC_OUT(GR_IO, LV_ERR, "  * ERROR getting device instance ");
        Ret = PNIO_ERR_WRONG_HND;
    }

    TRC_IF_ON_EXPR(GR_IO, LV_FCTPUB1,
        OSTREAM trcos1;
        trcos1 << showbase << hex;
        trcos1 << "<-PNIO_initiate_data_write, ret=" << Ret;
        trcos1 << '\0';
        TRC_OUT_OBJECT(GR_IO, LV_FCTPUB1, trcos1);
        );

    /*
    DPR_MUTEX_UNLOCK(PnioMutexDevice);
    */

    return Ret;
}

PNIO_UINT32 PNIO_CODE_ATTR PNIO_initiate_data_read_ext(PNIO_UINT32 DevHndl,
                                            PNIO_DEV_ADDR*     pAddr,
                                            PNIO_ACCESS_ENUM   AccessType)
{
    PNIO_UINT32 Ret;

    TRC_OUT(GR_IO, LV_FCTPUB1, "->PNIO_initiate_data_read_ext");

    TRC_IF_ON_EXPR(GR_IO, LV_FCTPUB2,
        OSTREAM trcos;
        trcos << showbase << hex;
        trcos << "  DevHndl=" << DevHndl;
        trcos << ", pAddr=";
        trcint_ShowDAddr(trcos, pAddr);
        trcos << ", AccessType=" << AccessType;
        trcos << ends;
        TRC_OUT_OBJECT(GR_IO, LV_FCTPUB2, trcos);
        );

    ICommon *pICommonInst = ICommon::get_instance(DevHndl);

    if(pICommonInst) {
        Ret = ((IDevice *) pICommonInst)->initiate_data_read_ext(DevHndl,
                                                   pAddr, AccessType);
    } else {
        TRC_OUT(GR_IO, LV_ERR, "  * ERROR getting device instance ");
        Ret = PNIO_ERR_WRONG_HND;
    }

    TRC_IF_ON_EXPR(GR_IO, LV_FCTPUB1,
        OSTREAM trcos1;
        trcos1 << showbase << hex;
        trcos1 << "<-PNIO_initiate_data_read_ext, ret=" << Ret;
        trcos1 << '\0';
        TRC_OUT_OBJECT(GR_IO, LV_FCTPUB1, trcos1);
        );

    return Ret;
}

PNIO_UINT32 PNIO_CODE_ATTR PNIO_initiate_data_write_ext(PNIO_UINT32 DevHndl,
                                            PNIO_DEV_ADDR*     pAddr,
                                            PNIO_ACCESS_ENUM   AccessType)
{
    PNIO_UINT32 Ret;

    TRC_OUT(GR_IO, LV_FCTPUB1, "->PNIO_initiate_data_write_ext");

    TRC_IF_ON_EXPR(GR_IO, LV_FCTPUB2,
        OSTREAM trcos;
        trcos << showbase << hex;
        trcos << "  DevHndl=" << DevHndl;
        trcos << ", pAddr=";
        trcint_ShowDAddr(trcos, pAddr);
        trcos << ", AccessType=" << AccessType;
        trcos << ends;
        TRC_OUT_OBJECT(GR_IO, LV_FCTPUB2, trcos);
        );

    ICommon *pICommonInst = ICommon::get_instance(DevHndl);

    if(pICommonInst) {
        Ret = ((IDevice *) pICommonInst)->initiate_data_write_ext(DevHndl,
                                                   pAddr, AccessType);
    } else {
        TRC_OUT(GR_IO, LV_ERR, "  * ERROR getting device instance ");
        Ret = PNIO_ERR_WRONG_HND;
    }

    TRC_IF_ON_EXPR(GR_IO, LV_FCTPUB1,
        OSTREAM trcos1;
        trcos1 << showbase << hex;
        trcos1 << "<-PNIO_initiate_data_write_ext, ret=" << Ret;
        trcos1 << '\0';
        TRC_OUT_OBJECT(GR_IO, LV_FCTPUB1, trcos1);
        );

    return Ret;
}

/*===========================================================================
* FUNCTION : PNIO_device_data_test
*----------------------------------------------------------------------------
  PURPOSE  : This test function sends data to firmware, and receves the same
             data from firmware again (firmware sends the data back),
             useful for performance measurement
*----------------------------------------------------------------------------
* RETURNS  : PNIO_OK on success; else PNIO_ERR_WRONG_HND, PNIO_ERR_INTERNAL
*----------------------------------------------------------------------------
* INPUTS   : ApplHandle - Handle of controler
* OUTPUS   : -
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
PNIO_UINT32 PNIO_CODE_ATTR PNIO_device_data_test(
    PNIO_UINT32 ApplHandle,
    PNIO_UINT8 * pBuffer,
    PNIO_UINT32 BufLen)
{
    DPR_MUTEX_LOCK(PnioMutexDevice);

    TRC_OUT(GR_STATE, LV_FCTPUB1, "->PNIO_device_data_test");

    TRC_IF_ON_EXPR(GR_STATE, LV_FCTPUB2,
        OSTREAM trcos;
        trcos << showbase << hex;
        trcos << "  ApplHandle=" << ApplHandle;
        trcos << ", BufLen=" << BufLen;
        trcos << ", pBuffer:";
        trcos << ends;
        TRC_OUT_OBJECT(GR_IO, LV_FCTPUB1, trcos);
        if(pBuffer) {
            TRC_OUTD(GR_IO, LV_FCTPUB1, pBuffer, BufLen);
        } else {
            TRC_OUT(GR_IO, LV_FCTPUB1, "NULL");
        }
    );

    PNIO_UINT32 Ret;

    ICommon *pInst = ICommon::get_instance(ApplHandle);

    if(pInst) {
        Ret = ((IDevice *) pInst)->TestBlockSendReceive(pBuffer, BufLen);
    } else {
        Ret = PNIO_ERR_WRONG_HND;
    }

    TRC_IF_ON_EXPR(GR_STATE, LV_FCTPUB1,
        OSTREAM trcos1;
        trcos1 << showbase << hex;
        trcos1 << "<-PNIO_device_data_test, ret=" << Ret;
        trcos1 << '\0';
        TRC_OUT_OBJECT(GR_STATE, LV_FCTPUB1, trcos1);
        );

    DPR_MUTEX_UNLOCK(PnioMutexDevice);

    return Ret;
}
