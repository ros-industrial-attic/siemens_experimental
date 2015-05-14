/****************************************************************************
*  Copyright (C) SIEMENS CORP., 2013 All rights reserved.
*****************************************************************************
* FILE NAME    : pniousrx.cpp
* ---------------------------------------------------------------------------
* DESCRIPTION  : controller specific user interface functions
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
DPR_MUTEX PnioMutexController;


/* exported functions */
#ifdef MANAGED_CP_EXCHANGE
extern "C" PNIO_UINT32 PNIO_CODE_ATTR ldah_check(PNIO_UINT32 CpIndex);
#endif

/*===========================================================================
* FUNCTION : PNIO_controller_open
*----------------------------------------------------------------------------
* PURPOSE  : With this function an application program registers
*            itself with the PROFInet IO as IO controller
*----------------------------------------------------------------------------
* RETURNS  : PNIO_OK on success; else *
*----------------------------------------------------------------------------
* INPUTS   : CpIndex - Unique identification for the communication module
*                      (module index in the component configuration)
*            ExtPar - Extended parameter (see earlier definition),
*                     among others the following parameters can be
*                     applied via the OR logical operator:PNIO_MODE_CTRL
*            cbf_RecReadConf - Callback function for signaling the results
                               of data set read tasks
*            cbf_RecWriteConf - Callback function for signaling the results
*                               of data set write tasks
*            cbf_AlarmInd - Callback function for signaling alarms
* OUTPUS   : pApplHandle - Handle, which is assigned to the controller.
*----------------------------------------------------------------------------
* COMMENTS : With this function an application program registers itself
*            with the PROFInet IO as IO Controller. Additionally the
*            task-specific callback functions relevant to the IO controller
*            are also registered. It is possible, to set the same callback
*            function for all call back events. The function pointers must
*            not be NULL, except for Alarm (cbf_AlarmInd)
*==========================================================================*/
PNIO_UINT32 PNIO_CODE_ATTR
PNIO_controller_open(PNIO_UINT32 CpIndex,
    PNIO_UINT32 ExtPar,
    PNIO_CBF cbf_RecReadConf,
    PNIO_CBF cbf_RecWriteConf,
    PNIO_CBF cbf_AlarmInd,
    PNIO_UINT32 * pApplHandle)
{
    PNIO_UINT32 Ret = PNIO_OK;

    DPR_MUTEX_LOCK(PnioMutexController);

    TRC_OUT01(GR_INIT, LV_FCTPUB1, "->PNIO_controller_open CpIndex %d", CpIndex);

    TRC_IF_ON_EXPR(GR_INIT, LV_FCTPUB2,
        OSTREAM trcos;
        trcos << showbase << hex;
        trcos << "  CpIndex=" << CpIndex;
        trcos << ", ExtPar=" << ExtPar;
        trcos << ", cbf_RecReadConf=" << (void *)cbf_RecReadConf;
        trcos << ", cbf_RecWriteConf=" << (void *)cbf_RecWriteConf;
        trcos << ", cbf_AlarmInd=" << (void *) cbf_AlarmInd;
        trcint_ShowPtr(trcos, pApplHandle, "ApplHandle");
        trcos << ends;
        TRC_OUT_OBJECT(GR_INIT, LV_FCTPUB2, trcos);
        );

#ifdef MANAGED_CP_EXCHANGE
    // PL: 11.03.2011: iorouter (device+controller) => ignore return value
    (void)ldah_check(CpIndex);
#endif

    Ret = IController::controller_open(CpIndex, ExtPar,
        cbf_RecReadConf, cbf_RecWriteConf, cbf_AlarmInd, pApplHandle);

    TRC_IF_ON_EXPR(GR_INIT, LV_FCTPUB1,
        OSTREAM trcos1;
        trcos1 << showbase << hex;
        trcos1 << "<-PNIO_controller_open, ret=" << Ret;
        trcint_ShowPtrValue(trcos1, pApplHandle, "ApplHandle");
        trcos1 << ends;
        TRC_OUT_OBJECT(GR_INIT, LV_FCTPUB1, trcos1);
        );

    DPR_MUTEX_UNLOCK(PnioMutexController);

    return Ret;
}

/*===========================================================================
* FUNCTION : PNIO_controller_close
*----------------------------------------------------------------------------
* PURPOSE  : close Controller from handle
*----------------------------------------------------------------------------
* RETURNS  : PNIO_OK on success; else PNIO_ERR_WRONG_HND, PNIO_ERR_INTERNAL
*----------------------------------------------------------------------------
* INPUTS   : - ApplHandle -  Handle Handle from PNIO_controller_open
* OUTPUS   : -
*----------------------------------------------------------------------------
* COMMENTS : With this function the application program deregisters
* an IO-Controller with PROFInet, which had been previously registered with
* PNIO_controller_open
*==========================================================================*/
PNIO_UINT32 PNIO_CODE_ATTR PNIO_controller_close(PNIO_UINT32 ApplHandle)
{
    PNIO_UINT32 Ret;

    DPR_MUTEX_LOCK(PnioMutexController);

    TRC_OUT(GR_INIT, LV_FCTPUB1, "->PNIO_controller_close");

    TRC_IF_ON_EXPR(GR_INIT, LV_FCTPUB2,
        OSTREAM trcos;
        trcos << showbase << hex;
        trcos << "  ApplHandle=" << ApplHandle;
        trcos << ends;
        TRC_OUT_OBJECT(GR_INIT, LV_FCTPUB2, trcos);
        );

    ICommon *pICommonInst = ICommon::get_instance(ApplHandle);

    if(pICommonInst) {
        Ret = ((IController *) pICommonInst)->controller_close();
        if(Ret != PNIO_OK) {
            TRC_OUT(GR_INIT, LV_ERR, "  * Controller Close failed ");
        }
    } else {
        TRC_OUT(GR_INIT, LV_ERR, "  * ERROR getting Controller instance ");
        Ret = PNIO_ERR_WRONG_HND;
    }

    TRC_IF_ON_EXPR(GR_INIT, LV_FCTPUB1,
        OSTREAM trcos1;
        trcos1 << showbase << hex;
        trcos1 << "<-PNIO_controller_close, ret=" << Ret;
        trcos1 << ends;
        TRC_OUT_OBJECT(GR_INIT, LV_FCTPUB1, trcos1);
        );

    DPR_MUTEX_UNLOCK(PnioMutexController);

    return Ret;
}

/* internal function                     */
/* will be used for internal diagnostics */

PNIO_UINT32 PNIO_CODE_ATTR pnio_get_kramhdr(PNIO_UINT32 ApplHandle,
                                             KRAMIOTLB_Header *HostIOTlbHdr,
                                             PNIO_UINT32 *InstanceHndl)
{
    PNIO_UINT32 Ret;

    DPR_MUTEX_LOCK(PnioMutexController);

    TRC_OUT(GR_INIT, LV_FCTPUB1, "->pnio_get_kramhdr");

    TRC_IF_ON_EXPR(GR_INIT, LV_FCTPUB2,
        OSTREAM trcos;
        trcos << showbase << hex;
        trcos << "  ApplHandle=" << ApplHandle;
        trcos << ends;
        TRC_OUT_OBJECT(GR_INIT, LV_FCTPUB2, trcos);
        );

    ICommon *pICommonInst = ICommon::get_instance(ApplHandle);

    if(pICommonInst) {
        *HostIOTlbHdr = *((IController*)pICommonInst)->get_iotlbhdr();
        *InstanceHndl = pICommonInst->get_iodataupdate_handle();
        Ret = PNIO_OK;
    } else {
        TRC_OUT(GR_INIT, LV_ERR, "  * ERROR getting Controller instance ");
        Ret = PNIO_ERR_WRONG_HND;
    }

    TRC_IF_ON_EXPR(GR_INIT, LV_FCTPUB1,
        OSTREAM trcos1;
        trcos1 << showbase << hex;
        trcos1 << "<-pnio_get_kramhdr, ret=" << Ret;
        trcos1 << ends;
        TRC_OUT_OBJECT(GR_INIT, LV_FCTPUB1, trcos1);
        );

    DPR_MUTEX_UNLOCK(PnioMutexController);

    return Ret;
}

/*===========================================================================
* FUNCTION : PNIO_set_mode
*----------------------------------------------------------------------------
* PURPOSE  : set new mode for Controller from handle
*----------------------------------------------------------------------------
* RETURNS  : PNIO_OK on success; else PNIO_ERR_WRONG_HND, PNIO_ERR_INTERNAL
*----------------------------------------------------------------------------
* INPUTS   : - ApplHandle -  Handle of controler
*            - Mode - new mode
* OUTPUS   :
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
PNIO_UINT32 PNIO_CODE_ATTR PNIO_set_mode(PNIO_UINT32 ApplHandle,
    PNIO_MODE_TYPE Mode)
{
    PNIO_UINT32 Ret;

    DPR_MUTEX_LOCK(PnioMutexController);

    TRC_OUT(GR_STATE, LV_FCTPUB1, "->PNIO_set_mode");

    TRC_IF_ON_EXPR(GR_STATE, LV_FCTPUB2,
        OSTREAM trcos;
        trcos << showbase << hex;
        trcos << "  ApplHandle=" << ApplHandle;
        trcint_ShowMode(trcos, &Mode, "Mode");
        trcos << ends;
        TRC_OUT_OBJECT(GR_STATE, LV_FCTPUB2, trcos);
        );
    ICommon *pInst = ICommon::get_instance(ApplHandle);

    if(pInst) {
        Ret = ((IController *) pInst)->set_mode(Mode);
    } else {
        Ret = PNIO_ERR_WRONG_HND;
    }

    TRC_IF_ON_EXPR(GR_INIT, LV_FCTPUB1,
        OSTREAM trcos1;
        trcos1 << showbase << hex;
        trcos1 << "<-PNIO_set_mode, ret=" << Ret;
        trcos1 << ends;
        TRC_OUT_OBJECT(GR_STATE, LV_FCTPUB1, trcos1);
        );

    DPR_MUTEX_UNLOCK(PnioMutexController);

    return Ret;
}

/*===========================================================================
* FUNCTION : PNIO_data_read
*----------------------------------------------------------------------------
* PURPOSE  : With this functions the IO data is read.
*            The caller of the function is to make available
*            the data buffer pBuffer. After return of the function,
*            the IO data is now available  in this data buffer.
*----------------------------------------------------------------------------
* RETURNS  : PNIO_OK on success; else PNIO_ERR_WRONG_HND, PNIO_ERR_INTERNAL
*----------------------------------------------------------------------------
* INPUTS   : ApplHandle - Handle of controler
*            pAddr - Address of the sub module
*            BufLen - Length of the data buffer (in byte) made available
*            IOlocState - IO local status (consumer status IOCS)
* OUTPUS   : pDataLen - read length of the data buffer (in byte)
*            pBuffer - Data buffer
*            pIOremState - IO remote status (provider status IOPS)
*----------------------------------------------------------------------------
* COMMENTS : pAddr is the address of the remote sub-module.
*
* N.B.  - THIS FUNCTION TO BE MODIFIED FOR IMPLEMENTING DIRECT ACCESS TO KRAM
*
*==========================================================================*/
PNIO_UINT32 PNIO_CODE_ATTR
PNIO_data_read(PNIO_UINT32 ApplHandle,
    PNIO_ADDR * pAddr,
    PNIO_UINT32 BufLen,
    PNIO_UINT32 * pDataLen,
    PNIO_UINT8 * pBuffer,
    PNIO_IOXS IOlocState,
    PNIO_IOXS * pIOremState)
{
    /*
    this function is reentrant
    DPR_MUTEX_LOCK(PnioMutexController);
    */

    TRC_OUT(GR_IO, LV_FCTPUB1, "->PNIO_data_read");

    TRC_IF_ON_EXPR(GR_IO, LV_FCTPUB2,
        OSTREAM trcos;
        trcos << showbase << hex;
        trcos << "  ApplHandle=" << ApplHandle;
        trcos << ", pAddr=";
        trcint_ShowAddr(trcos, pAddr);
        trcos << "  BufLen=" << BufLen;
        trcint_ShowPtrValue(trcos, pDataLen, "pDataLen");
        trcint_ShowPtr(trcos, pBuffer, "pBuffer");
        trcint_ShowStatus(trcos, &IOlocState, "IOlocState");
        trcint_ShowPtr(trcos, pIOremState, "pIOremState");
        trcos << ends;
        TRC_OUT_OBJECT(GR_IO, LV_FCTPUB2, trcos);
        );
    PNIO_UINT32 Ret;

    if(!pDataLen) {
        Ret = PNIO_ERR_PRM_LEN;
    } else {
        ICommon *pInst = ICommon::get_instance(ApplHandle);

        if(pInst) {
            Ret = ((IController *) pInst)->perf_io(pAddr, ACC_T_READ,
                &IOlocState, pIOremState, &BufLen, pBuffer);
            *pDataLen = BufLen;
        } else {
            Ret = PNIO_ERR_WRONG_HND;
        }
    }

    TRC_IF_ON_EXPR(GR_IO, LV_FCTPUB1,
        OSTREAM trcos1;
        trcos1 << showbase << hex;
        trcos1 << "<-PNIO_data_read, ret=" << Ret;
        trcint_ShowStatus(trcos1, pIOremState, "pIOremState");
        trcint_ShowPtrValue(trcos1, pDataLen, "pDataLen");
        trcos1 << ", pBuffer:";
        trcos1 << ends;
        TRC_OUT_OBJECT(GR_IO, LV_FCTPUB1, trcos1);
        if(Ret == PNIO_OK && pBuffer) {
            TRC_OUTD(GR_IO, LV_FCTPUB1, pBuffer, *pDataLen);
        } else {
            TRC_OUT(GR_IO, LV_FCTPUB1, "NULL");
        }
        );

    /*
    this function is reentrant
    DPR_MUTEX_UNLOCK(PnioMutexController);
    */

    return Ret;
}

/*===========================================================================
* FUNCTION : PNIO_output_data_read
*----------------------------------------------------------------------------
* PURPOSE  : With this functions the IO output data is read.
*            The caller of the function is to make available
*            the data buffer pBuffer. After return of the function,
*            the IO data is now available  in this data buffer.
*----------------------------------------------------------------------------
* RETURNS  : PNIO_OK on success; else PNIO_ERR_WRONG_HND, PNIO_ERR_INTERNAL
*----------------------------------------------------------------------------
* INPUTS   : ApplHandle - Handle of controler
*            pAddr - Address of the sub module
*            BufLen - Length of the data buffer (in byte) made available
* OUTPUS   : pDataLen - read length of the data buffer (in byte)
*            pBuffer - Data buffer
*            pIOlocState - IO local status (consumer status IOCS)
*            pIOremState - IO remote status (provider status IOPS)
*==========================================================================*/
PNIO_UINT32 PNIO_CODE_ATTR PNIO_output_data_read(PNIO_UINT32 ApplHandle,
    PNIO_ADDR * pAddr,
    PNIO_UINT32 BufLen,
    PNIO_UINT32 * pDataLen,
    PNIO_UINT8 * pBuffer,
    PNIO_IOXS * pIOlocState,
    PNIO_IOXS * pIOremState)
{
    /*
    this function is reentrant
    DPR_MUTEX_LOCK(PnioMutexController);
    */

    TRC_OUT(GR_IO, LV_FCTPUB1, "->PNIO_output_data_read");

    TRC_IF_ON_EXPR(GR_IO, LV_FCTPUB2,
        OSTREAM trcos;
        trcos << showbase << hex;
        trcos << "  ApplHandle=" << ApplHandle;
        trcos << ", pAddr=";
        trcint_ShowAddr(trcos, pAddr);
        trcos << "  BufLen=" << BufLen;
        trcint_ShowPtrValue(trcos, pDataLen, "pDataLen");
        trcint_ShowPtr(trcos, pBuffer, "pBuffer");
        trcint_ShowPtr(trcos, pIOlocState, "pIOlocState");
        trcint_ShowPtr(trcos, pIOremState, "pIOremState");
        trcos << ends;
        TRC_OUT_OBJECT(GR_IO, LV_FCTPUB2, trcos);
        );
    PNIO_UINT32 Ret;

    if(!pDataLen) {
        Ret = PNIO_ERR_PRM_LEN;
    } else {
        ICommon *pInst = ICommon::get_instance(ApplHandle);

        if(pInst) {
            Ret = ((IController *) pInst)->perf_io(pAddr, ACC_T_READ_OUTPUT,
                pIOlocState, pIOremState, &BufLen, pBuffer);
            *pDataLen = BufLen;
        } else {
            Ret = PNIO_ERR_WRONG_HND;
        }
    }

    TRC_IF_ON_EXPR(GR_IO, LV_FCTPUB1,
        OSTREAM trcos1;
        trcos1 << showbase << hex;
        trcos1 << "<-PNIO_output_data_read, ret=" << Ret;
        trcint_ShowStatus(trcos1, pIOlocState, "pIOlocState");
        trcint_ShowStatus(trcos1, pIOremState, "pIOremState");
        trcint_ShowPtrValue(trcos1, pDataLen, "pDataLen");
        trcos1 << ", pBuffer:";
        trcos1 << ends;
        TRC_OUT_OBJECT(GR_IO, LV_FCTPUB1, trcos1);
        if(Ret == PNIO_OK && pBuffer) {
            TRC_OUTD(GR_IO, LV_FCTPUB1, pBuffer, *pDataLen);
        } else {
            TRC_OUT(GR_IO, LV_FCTPUB1, "NULL");
        }
        );
    /*
    this function is reentrant
    DPR_MUTEX_UNLOCK(PnioMutexController);
    */

    return Ret;
}

PNIO_UINT32 PNIO_CODE_ATTR
PNIO_data_read_cache_refresh(PNIO_UINT32 ApplHandle)
{
    TRC_OUT(GR_IO, LV_FCTPUB1, "->PNIO_data_read_cache_refresh");
    ICommon *pInst = ICommon::get_instance(ApplHandle);

    PNIO_UINT32 Ret = PNIO_OK;

    if(pInst) {
        ((IController *) pInst)->read_cache_refresh();
    } else {
        Ret = PNIO_ERR_WRONG_HND;
    }

    TRC_OUT01(GR_IO, LV_FCTPUB1, "<-PNIO_data_read_cache_refresh, ret=0x%x",Ret);

    return Ret;
}

PNIO_UINT32 PNIO_CODE_ATTR
PNIO_data_read_cache(PNIO_UINT32 ApplHandle,
    PNIO_ADDR * pAddr,
    PNIO_UINT32 BufLen,
    PNIO_UINT32 * pDataLen,
    PNIO_UINT8 * pBuffer,
    PNIO_IOXS   IOlocState,
    PNIO_IOXS * pIOremState)
{
    /*
    this function is reentrant
    DPR_MUTEX_LOCK(PnioMutexController);
    */

    TRC_OUT(GR_IO, LV_FCTPUB1, "->PNIO_data_read_cache");

    TRC_IF_ON_EXPR(GR_IO, LV_FCTPUB2,
        OSTREAM trcos;
        trcos << showbase << hex;
        trcos << "  ApplHandle=" << ApplHandle;
        trcos << ", pAddr=";
        trcint_ShowAddr(trcos, pAddr);
        trcos << "  BufLen=" << BufLen;
        trcint_ShowPtrValue(trcos, pDataLen, "pDataLen");
        trcint_ShowPtr(trcos, pBuffer, "pBuffer");
        //trcint_ShowStatus(trcos, &IOlocState, "IOlocState");
        trcint_ShowPtr(trcos, pIOremState, "pIOremState");
        trcos << ends;
        TRC_OUT_OBJECT(GR_IO, LV_FCTPUB2, trcos);
        );
    PNIO_UINT32 Ret;

    if(!pDataLen) {
        Ret = PNIO_ERR_PRM_LEN;
    } else {
        ICommon *pInst = ICommon::get_instance(ApplHandle);

        if(pInst) {
            Ret = ((IController *) pInst)->perf_io(pAddr, ACC_T_READ,
                &IOlocState, pIOremState, &BufLen, pBuffer, IController::ICONT_MT_CACHE);
            *pDataLen = BufLen;
        } else {
            Ret = PNIO_ERR_WRONG_HND;
        }
    }

    TRC_IF_ON_EXPR(GR_IO, LV_FCTPUB1,
        OSTREAM trcos1;
        trcos1 << showbase << hex;
        trcos1 << "<-PNIO_data_read_cache, ret=" << Ret;
        trcint_ShowStatus(trcos1, pIOremState, "pIOremState");
        trcint_ShowPtrValue(trcos1, pDataLen, "pDataLen");
        trcos1 << ", pBuffer:";
        trcos1 << ends;
        TRC_OUT_OBJECT(GR_IO, LV_FCTPUB1, trcos1);
        if(Ret == PNIO_OK && pBuffer) {
            TRC_OUTD(GR_IO, LV_FCTPUB1, pBuffer, *pDataLen);
        } else {
            TRC_OUT(GR_IO, LV_FCTPUB1, "NULL");
        }
        );

    /*
    this function is reentrant
    DPR_MUTEX_UNLOCK(PnioMutexController);
    */

    return Ret;
}

/*===========================================================================
* FUNCTION : PNIO_data_write
*----------------------------------------------------------------------------
* PURPOSE  : With this function the IO data is written and
*            the corresponding IOlocState is set.
*            The data buffer pBuffer contains the IO data to be written.
*----------------------------------------------------------------------------
* RETURNS  : PNIO_OK on success; else PNIO_ERR_WRONG_HND, PNIO_ERR_INTERNAL
*----------------------------------------------------------------------------
* INPUTS   : ApplHandle - Handle of controler
*            pAddr - Address of the sub module
*            BufLen - Length of the data buffer (in byte) made available
*            pBuffer - Data buffer
*            IOlocState - IO local status (consumer status IOCS)
* OUTPUS   :
*            pIOremState - IO remote status (provider status IOPS)
*----------------------------------------------------------------------------
* COMMENTS : pAddr is the address of the remote sub-module.
*            This function is used for always writing output data.
*
* N.B.  - THIS FUNCTION TO BE MODIFIED FOR IMPLEMENTING DIRECT ACCESS TO KRAM
*
*==========================================================================*/
PNIO_UINT32 PNIO_CODE_ATTR
PNIO_data_write(PNIO_UINT32 ApplHandle,
    PNIO_ADDR * pAddr,
    PNIO_UINT32 BufLen,
    PNIO_UINT8 * pBuffer,
    PNIO_IOXS IOlocState,
    PNIO_IOXS * pIOremState)
{
    /*
    this function is reentrant
    DPR_MUTEX_LOCK(PnioMutexController);
    */

    TRC_OUT(GR_IO, LV_FCTPUB1, "->PNIO_data_write");

    TRC_IF_ON_EXPR(GR_IO, LV_FCTPUB2,
        OSTREAM trcos;
        trcos << showbase << hex;
        trcos << "  ApplHandle=" << ApplHandle;
        trcos << ", pAddr=";
        trcint_ShowAddr(trcos, pAddr);
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

    TRC_IF_ON_EXPR(GR_IO, LV_FCTPUB1,
        OSTREAM trcos1;
        trcint_ShowStatus(trcos1, &IOlocState, "IOlocState");
        trcint_ShowPtr(trcos1, pIOremState, "pIOremState");
        trcos1 << ends;
        TRC_OUT_OBJECT(GR_IO, LV_FCTPUB2, trcos1);
        );
    PNIO_UINT32 Ret;

    ICommon *pInst = ICommon::get_instance(ApplHandle);

    if(pInst) {
        Ret = ((IController *) pInst)->perf_io(pAddr, ACC_T_WRITE,
            &IOlocState, pIOremState, &BufLen, pBuffer);
    } else {
        Ret = PNIO_ERR_WRONG_HND;
    }

    TRC_IF_ON_EXPR(GR_IO, LV_FCTPUB1,
        OSTREAM trcos2;
        trcos2 << showbase << hex;
        trcos2 << "<-PNIO_data_write, ret=" << Ret;
        trcint_ShowStatus(trcos2, pIOremState, "pIOremState");
        trcos2 << ends;
        TRC_OUT_OBJECT(GR_IO, LV_FCTPUB1, trcos2);
        );

    /*
    DPR_MUTEX_UNLOCK(PnioMutexController);
    */

    return Ret;
}


PNIO_UINT32 PNIO_CODE_ATTR PNIO_data_write_cache_flush(
        PNIO_UINT32 ApplHandle)
{
    TRC_OUT(GR_IO, LV_FCTPUB1, "->PNIO_data_write_cache_flush");
    ICommon *pInst = ICommon::get_instance(ApplHandle);

    PNIO_UINT32 Ret = PNIO_OK;

    if(pInst) {
        ((IController *) pInst)->write_cache_flush();
    } else {
        Ret = PNIO_ERR_WRONG_HND;
    }

    TRC_OUT01(GR_IO, LV_FCTPUB1, "<-PNIO_data_write_cache_flush, ret=0x%x",Ret);

    return Ret;
}

PNIO_UINT32 PNIO_CODE_ATTR PNIO_data_write_cache (
        PNIO_UINT32   ApplHandle,        /* in */
        PNIO_ADDR *   pAddr,             /* in */
        PNIO_UINT32   BufLen,            /* in */
        PNIO_UINT8 *  pBuffer,           /* in */
        PNIO_IOXS     IOlocState,        /* in */
        PNIO_IOXS *   pIOremState)
{
    /*
    this function is reentrant
    DPR_MUTEX_LOCK(PnioMutexController);
    */

    TRC_OUT(GR_IO, LV_FCTPUB1, "->PNIO_data_write_cache");

    TRC_IF_ON_EXPR(GR_IO, LV_FCTPUB2,
        OSTREAM trcos;
        trcos << showbase << hex;
        trcos << "  ApplHandle=" << ApplHandle;
        trcos << ", pAddr=";
        trcint_ShowAddr(trcos, pAddr);
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

    TRC_IF_ON_EXPR(GR_IO, LV_FCTPUB1,
        OSTREAM trcos1;
        trcint_ShowStatus(trcos1, &IOlocState, "IOlocState");
        trcos1 << ends;
        TRC_OUT_OBJECT(GR_IO, LV_FCTPUB2, trcos1);
        );
    PNIO_UINT32 Ret;

    ICommon *pInst = ICommon::get_instance(ApplHandle);

    if(pInst) {
        Ret = ((IController *) pInst)->perf_io(pAddr, ACC_T_WRITE,
            &IOlocState, pIOremState, &BufLen, pBuffer, IController::ICONT_MT_CACHE);
    } else {
        Ret = PNIO_ERR_WRONG_HND;
    }

    TRC_IF_ON_EXPR(GR_IO, LV_FCTPUB1,
        OSTREAM trcos2;
        trcos2 << showbase << hex;
        trcos2 << "<-PNIO_data_write_cache, ret=" << Ret;
        trcos2 << ends;
        TRC_OUT_OBJECT(GR_IO, LV_FCTPUB1, trcos2);
        );

    /*
    DPR_MUTEX_UNLOCK(PnioMutexController);
    */

    return Ret;
}


/*===========================================================================
* FUNCTION : PNIO_register_cbf
*----------------------------------------------------------------------------
* PURPOSE  : With this function a callback function for the event
*            type CbeType is registered. The function pointer must not be NULL
*----------------------------------------------------------------------------
* RETURNS  : PNIO_OK on success; else PNIO_ERR_WRONG_HND, PNIO_ERR_INTERNAL
*----------------------------------------------------------------------------
* INPUTS   : ApplHandle - Handle of controler
*            CbeType - Callback event type
*            Cbf - Callback function
* OUTPUS   : -
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
PNIO_UINT32 PNIO_CODE_ATTR
PNIO_register_cbf(PNIO_UINT32 ApplHandle,
    PNIO_CBE_TYPE CbeType,
    PNIO_CBF Cbf)
{
    DPR_MUTEX_LOCK(PnioMutexController);

    TRC_OUT(GR_MGT, LV_FCTPUB1, "->PNIO_register_cbf");

    TRC_IF_ON_EXPR(GR_MGT, LV_FCTPUB2,
        OSTREAM trcos;
        trcos << showbase << hex;
        trcos << "  ApplHandle=" << ApplHandle;
        trcos << "  CbeType=" << CbeType;
        trcos << ", Cbf=" << Cbf;
        trcos << ends;
        TRC_OUT_OBJECT(GR_MGT, LV_FCTPUB2, trcos);
        );
    PNIO_UINT32 Ret;

    ICommon *pInst = ICommon::get_instance(ApplHandle);

    if(pInst) {
        Ret = ((IController *) pInst)->register_cbf(CbeType, Cbf);
    } else {
        Ret = PNIO_ERR_WRONG_HND;
    }

    TRC_IF_ON_EXPR(GR_MGT, LV_FCTPUB1,
        OSTREAM trcos1;
        trcos1 << showbase << hex;
        trcos1 << "<-PNIO_register_cbf, ret=" << Ret;
        trcos1 << ends;
        TRC_OUT_OBJECT(GR_MGT, LV_FCTPUB1, trcos1);
        );

    DPR_MUTEX_UNLOCK(PnioMutexController);

    return Ret;
}
#if 0
/*===========================================================================
* FUNCTION : PNIO_alarm_resp
*----------------------------------------------------------------------------
* PURPOSE  : With this function an alarm is acknowledged,
*            which was previously reported with the
*            callback event PNIO_CBE_ALARM_IND.
*----------------------------------------------------------------------------
* RETURNS  : PNIO_OK on success; else PNIO_ERR_WRONG_HND, PNIO_ERR_INTERNAL
*----------------------------------------------------------------------------
* INPUTS   : ApplHandle - Handle of controler
*            IndRef - Internally assigned reference
* OUTPUS   : -
*----------------------------------------------------------------------------
* COMMENTS : The reference IndRef from PNIO_CBE_ALARM_IND is to be passed,
*            which creates the reference between PNIO_CBE_ALARM_IND
*            and the corresponding PNIO_ds_write_resp.
*==========================================================================*/
PNIO_UINT32 PNIO_CODE_ATTR PNIO_alarm_resp(PNIO_UINT32 ApplHandle,
    PNIO_REF IndRef)
{
    DPR_MUTEX_LOCK(PnioMutexController);

    TRC_OUT(GR_ALARM, LV_FCTPUB1, "->PNIO_alarm_resp");

    TRC_IF_ON_EXPR(GR_ALARM, LV_FCTPUB2,
        OSTREAM trcos;
        trcos << showbase << hex;
        trcos << "  ApplHandle=" << ApplHandle;
        trcos << ", IndRef=" << IndRef;
        trcos << ends;
        TRC_OUT_OBJECT(GR_ALARM, LV_FCTPUB2, trcos);
        );
    PNIO_UINT32 Ret;

    ICommon *pInst = ICommon::get_instance(ApplHandle);

    if(pInst) {
        Ret = ((IController *) pInst)->alarm_resp(IndRef);
    } else {
        Ret = PNIO_ERR_WRONG_HND;
    }

    TRC_IF_ON_EXPR(GR_ALARM, LV_FCTPUB1,
        OSTREAM trcos1;
        trcos1 << showbase << hex;
        trcos1 << "<-PNIO_alarm_resp, ret=" << Ret;
        trcos1 << ends;
        TRC_OUT_OBJECT(GR_ALARM, LV_FCTPUB1, trcos1);
        );

    DPR_MUTEX_UNLOCK(PnioMutexController);

    return Ret;
}
#endif
/*===========================================================================
* FUNCTION : PNIO_rec_write_req
*----------------------------------------------------------------------------
* PURPOSE  : With this function the controller sends a write data record
*            the result of this job is signaled by the callback event
*            PNIO_CBE_REC_WRITE_CONF
*----------------------------------------------------------------------------
* RETURNS  : PNIO_OK on success; else PNIO_ERR_WRONG_HND, PNIO_ERR_INTERNAL
*----------------------------------------------------------------------------
* INPUTS   : ApplHandle - Handle of controler
*            pAddr - Address of the sub module
*            RecordIndex- data record number
*            ReqRef - Reference assigned by the IO base user program
*            Length - Length of the data buffer (in byte) made available
*            pBuffer - IO local status (consumer status IOCS)
* OUTPUS   : -
*----------------------------------------------------------------------------
* COMMENTS : -
*==========================================================================*/
PNIO_UINT32 PNIO_CODE_ATTR
PNIO_rec_write_req(PNIO_UINT32 ApplHandle,
    PNIO_ADDR * pAddr,
    PNIO_UINT32 RecordIndex,
    PNIO_REF ReqRef,
    PNIO_UINT32 Length,
    PNIO_UINT8 * pBuffer)
{
    DPR_MUTEX_LOCK(PnioMutexController);

    TRC_OUT(GR_DS, LV_FCTPUB1, "->PNIO_rec_write_req");

    TRC_IF_ON_EXPR(GR_DS, LV_FCTPUB2,
        OSTREAM trcos;
        OSTREAM trcos1;
        trcos << showbase << hex;
        trcos << "  ApplHandle=" << ApplHandle;
        trcos << ", pAddr=";
        trcint_ShowAddr(trcos, pAddr);
        trcos << ", RecordIndex=" << RecordIndex;
        trcos << ", ReqRef=" << ReqRef;
        trcos << ends;
        TRC_OUT_OBJECT(GR_DS, LV_FCTPUB2, trcos);
        trcos1 << showbase << hex;
        trcos1 << "  Length=" << Length;
        trcos1 << ", pBuffer=";
        trcint_ShowData(trcos1, Length, pBuffer);
        trcos1 << ends;
        TRC_OUT_OBJECT(GR_DS, LV_FCTPUB2, trcos1);
        );
    PNIO_UINT32 Ret;

    ICommon *pInst = ICommon::get_instance(ApplHandle);

    if(pInst) {
        Ret = ((IController *) pInst)->perf_ds(pAddr, ACC_T_WRITE, ReqRef,
            RecordIndex, Length, pBuffer);
    } else {
        Ret = PNIO_ERR_WRONG_HND;
    }

    TRC_IF_ON_EXPR(GR_DS, LV_FCTPUB1,
        OSTREAM trcos1;
        trcos1 << showbase << hex;
        trcos1 << "<-PNIO_rec_write_req, ret=" << Ret;
        trcos1 << ends;
        TRC_OUT_OBJECT(GR_DS, LV_FCTPUB1, trcos1);
        );

    DPR_MUTEX_UNLOCK(PnioMutexController);

    return Ret;
}

/*===========================================================================
* FUNCTION : PNIO_rec_read_req
*----------------------------------------------------------------------------
* PURPOSE  : With this function a controller sends a read data record job.
*            the result of the job is signaled by callback event
*            PNIO_CBE_REC_READ_CONF
*----------------------------------------------------------------------------
* RETURNS  : PNIO_OK on success; else PNIO_ERR_WRONG_HND, PNIO_ERR_INTERNAL
*----------------------------------------------------------------------------
* INPUTS   : ApplHandle - Handle of controler
*            pAddr - Address of the sub module
*            RecordIndex - Data record number
*            ReqRef - reference assigned by IO base user program
*            Length - Length of data buffer
* OUTPUS   :
*
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
PNIO_UINT32 PNIO_CODE_ATTR
PNIO_rec_read_req(PNIO_UINT32 ApplHandle,
    PNIO_ADDR * pAddr,
    PNIO_UINT32 RecordIndex,
    PNIO_REF ReqRef,
    PNIO_UINT32 Length)
{
    DPR_MUTEX_LOCK(PnioMutexController);

    TRC_OUT(GR_DS, LV_FCTPUB1, "->PNIO_rec_read_req");

    TRC_IF_ON_EXPR(GR_DS, LV_FCTPUB2,
        OSTREAM trcos;
        trcos << showbase << hex;
        trcos << "  ApplHandle=" << ApplHandle;
        trcos << ", pAddr=";
        trcint_ShowAddr(trcos, pAddr);
        trcos << ", RecordIndex=" << RecordIndex;
        trcos << ", ReqRef=" << ReqRef;
        trcos << ", Length=" << Length;
        trcos << ends;
        TRC_OUT_OBJECT(GR_DS, LV_FCTPUB2, trcos);
        );
    PNIO_UINT32 Ret;

    ICommon *pInst = ICommon::get_instance(ApplHandle);

    if(pInst) {
        Ret = ((IController *) pInst)->perf_ds(pAddr, ACC_T_READ, ReqRef,
            RecordIndex, Length, NULL);
    } else {
        Ret = PNIO_ERR_WRONG_HND;
    }

    TRC_IF_ON_EXPR(GR_DS, LV_FCTPUB1,
        OSTREAM trcos1;
        trcos1 << showbase << hex;
        trcos1 << "<-PNIO_rec_read_req, ret=" << Ret;
        trcos1 << ends;
        TRC_OUT_OBJECT(GR_DS, LV_FCTPUB1, trcos1);
        );

    DPR_MUTEX_UNLOCK(PnioMutexController);

    return Ret;
}

/*===========================================================================
* FUNCTION : PNIO_device_activate
*----------------------------------------------------------------------------
* PURPOSE  : With this function an IO device is activated or deactivated
*            by the IO controller
*----------------------------------------------------------------------------
* RETURNS  : PNIO_OK on success; else PNIO_ERR_WRONG_HND, PNIO_ERR_INTERNAL
*----------------------------------------------------------------------------
* INPUTS   : ApplHandle - Handle of controler
*            pAddr - logical address of the device to which the task is sent
*            DeviceMode - PNIO_DA_FALSE deactivates the device;
*                         PNIO_DA_TRUE activates the device
* OUTPUS   : -
*----------------------------------------------------------------------------
* COMMENTS : Any logical address of the IO device, for which the activation
*            is intended, is passed to pAddr. In the controller mode OFFLINE
*            neither activation nor deactivation of devices is possible
*            After the initial start-up all devices are activated and
*            are in the mode OPERATE. The application status of the controller
*            is CLEAR and the user now is to deactivate the required devices.
*            A subsequent mode change of the controller from CLEAR to OPERATE
*            sets the IOxS of the data from/to the activated device to GOOD.
*==========================================================================*/
PNIO_UINT32 PNIO_CODE_ATTR
PNIO_device_activate(PNIO_UINT32 ApplHandle,
    PNIO_ADDR * pAddr,
    PNIO_DEV_ACT_TYPE DeviceMode)
{
    DPR_MUTEX_LOCK(PnioMutexController);

    TRC_OUT(GR_STATE, LV_FCTPUB1, "->PNIO_device_activate");

    TRC_IF_ON_EXPR(GR_STATE, LV_FCTPUB2,
        OSTREAM trcos;
        trcos << showbase << hex;
        trcos << "  ApplHandle=" << ApplHandle;
        trcos << ", pAddr=";
        trcint_ShowAddr(trcos, pAddr);
        trcos << ", DeviceMode=" << DeviceMode;
        trcos << ends;
        TRC_OUT_OBJECT(GR_STATE, LV_FCTPUB2, trcos);
        );
    PNIO_UINT32 Ret;

    ICommon *pInst = ICommon::get_instance(ApplHandle);

    if(pInst) {
        Ret = ((IController *) pInst)->device_activate(pAddr, DeviceMode);
    } else {
        Ret = PNIO_ERR_WRONG_HND;
    }

    TRC_IF_ON_EXPR(GR_STATE, LV_FCTPUB1,
        OSTREAM trcos1;
        trcos1 << showbase << hex;
        trcos1 << "<-PNIO_device_activate, ret=" << Ret;
        trcos1 << ends;
        TRC_OUT_OBJECT(GR_STATE, LV_FCTPUB1, trcos1);
        );

    DPR_MUTEX_UNLOCK(PnioMutexController);

    return Ret;
}

/*===========================================================================
* FUNCTION : PNIO_ctrl_diag_req
*----------------------------------------------------------------------------
* PURPOSE  : send dignostic request to IO controller
*----------------------------------------------------------------------------
* RETURNS  : PNIO_OK on success; else PNIO_ERR_WRONG_HND, PNIO_ERR_INTERNAL
*----------------------------------------------------------------------------
* INPUTS   : ApplHandle - Handle of controler
*==========================================================================*/

PNIO_UINT32 PNIO_CODE_ATTR PNIO_ctrl_diag_req(
  PNIO_UINT32        Handle,
  PNIO_CTRL_DIAG*    pDiagReq)
{
    DPR_MUTEX_LOCK(PnioMutexController);

    TRC_OUT(GR_STATE, LV_FCTPUB1, "->PNIO_ctrl_diag_req");

    TRC_IF_ON_EXPR(GR_STATE, LV_FCTPUB2,
        OSTREAM trcos;
        trcos << showbase << hex;
        trcos << "  Handle=" << Handle;
        trcos << ", pDiagReq=";
        trcint_ShowDiagReq(trcos, pDiagReq);
        trcos << ends;
        TRC_OUT_OBJECT(GR_STATE, LV_FCTPUB2, trcos);
        );
    PNIO_UINT32 Ret;

    ICommon *pInst =  ICommon::get_instance(Handle);

    if(pInst) {
        Ret = ((IController *) pInst)->ctrl_diag_req(pDiagReq);
    } else {
        Ret = PNIO_ERR_WRONG_HND;
    }

    TRC_IF_ON_EXPR(GR_STATE, LV_FCTPUB1,
        OSTREAM trcos1;
        trcos1 << showbase << hex;
        trcos1 << "<-PNIO_ctrl_diag_req, ret=" << Ret;
        trcos1 << ends;
        TRC_OUT_OBJECT(GR_STATE, LV_FCTPUB1, trcos1);
        );

    DPR_MUTEX_UNLOCK(PnioMutexController);

    return Ret;
}

/*===========================================================================
* FUNCTION : PNIO_data_test
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
PNIO_UINT32 PNIO_CODE_ATTR PNIO_data_test(
    PNIO_UINT32 ApplHandle,
    PNIO_UINT8 * pBuffer,
    PNIO_UINT32 BufLen)
{
    DPR_MUTEX_LOCK(PnioMutexController);

    TRC_OUT(GR_STATE, LV_FCTPUB1, "->PNIO_data_test");

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
        Ret = ((IController *) pInst)->TestBlockSendReceive(pBuffer, BufLen);
    } else {
        Ret = PNIO_ERR_WRONG_HND;
    }

    TRC_IF_ON_EXPR(GR_STATE, LV_FCTPUB1,
        OSTREAM trcos1;
        trcos1 << showbase << hex;
        trcos1 << "<-PNIO_data_test, ret=" << Ret;
        trcos1 << ends;
        TRC_OUT_OBJECT(GR_STATE, LV_FCTPUB1, trcos1);
        );

    DPR_MUTEX_UNLOCK(PnioMutexController);

    return Ret;
}

/*===========================================================================
* FUNCTION : PNIO_data_test_io
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
PNIO_UINT32 PNIO_CODE_ATTR PNIO_data_test_io(
    PNIO_UINT32 ApplHandle,
    PNIO_UINT32 area,
    PNIO_UINT32 operation,
    PNIO_UINT32 BufLen,
    PNIO_UINT8 * pBuffer
    )
{
    DPR_MUTEX_LOCK(PnioMutexController);

    TRC_OUT(GR_STATE, LV_FCTPUB1, "->PNIO_data_test_io");

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
        Ret = ((IController *) pInst)->TestIO(area, operation, BufLen, pBuffer);
    } else {
        Ret = PNIO_ERR_WRONG_HND;
    }

    TRC_IF_ON_EXPR(GR_STATE, LV_FCTPUB1,
        OSTREAM trcos1;
        trcos1 << showbase << hex;
        trcos1 << "<-PNIO_data_test_io, ret=" << Ret;
        trcos1 << ends;
        TRC_OUT_OBJECT(GR_STATE, LV_FCTPUB1, trcos1);
        );

    DPR_MUTEX_UNLOCK(PnioMutexController);

    return Ret;
} // PNIO_data_test_io


#ifdef PROFI_ENERGY
/*===========================================================================
* FUNCTION : PNIO_pe_cmd_req
*----------------------------------------------------------------------------
  PURPOSE  : PROFIenergy (PE) general service request function.
*----------------------------------------------------------------------------
* RETURNS  : PNIO_OK on success; else PNIO_ERR_WRONG_HND, PNIO_ERR_INTERNAL
*----------------------------------------------------------------------------
* INPUTS   : Handle - Application handle returned by ctrl open
*
*
* OUTPUS   : -
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
PNIO_UINT32 PNIO_CODE_ATTR PNIO_pe_cmd_req(
        PNIO_UINT32      Handle,            /* in */
        PNIO_ADDR       *pAddr,             /* in */
        PNIO_REF         ReqRef,            /* in */
        PNIO_PE_REQ_PRM *pPeReqPrm          /* in */
        )
{

    if (pPeReqPrm == NULL) {
        return PNIO_ERR_PRM_INVALIDARG;
    }

    DPR_MUTEX_LOCK(PnioMutexController);

    TRC_OUT03(GR_PE, LV_INFO, "->PNIO_pe_cmd_req: cmd=%d adr=%u ref=%#x", pPeReqPrm->CmdId, pAddr->u.Addr, ReqRef);

    PNIO_UINT32 Ret;

    ICommon *pInst = ICommon::get_instance(Handle);

    if(pInst) {
        Ret = ((IController *) pInst)->perf_pe(Handle, pAddr, ReqRef, pPeReqPrm);
    } else {
        Ret = PNIO_ERR_WRONG_HND;
    }


    TRC_OUT(GR_PE, LV_INFO, "<-PNIO_pe_cmd_req");
    DPR_MUTEX_UNLOCK(PnioMutexController);

    return Ret;
} // PNIO_pe_cmd_req


/*===========================================================================
* FUNCTION : PNIO_register_cbf
*----------------------------------------------------------------------------
* PURPOSE  : With this function a callback function for the event
*            type CbeType is registered. The function pointer must not be NULL
*----------------------------------------------------------------------------
* RETURNS  : PNIO_OK on success; else PNIO_ERR_WRONG_HND, PNIO_ERR_INTERNAL
*----------------------------------------------------------------------------
* INPUTS   : ApplHandle - Handle of controler
*            CbeType - Callback event type
*            Cbf - Callback function
* OUTPUS   : -
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
PNIO_UINT32 PNIO_CODE_ATTR
PNIO_register_pe_cbf(PNIO_UINT32 ApplHandle, PNIO_PE_CBF Cbf)
{
    PNIO_UINT32 Ret = PNIO_OK;

    DPR_MUTEX_LOCK(PnioMutexController);
    TRC_OUT01(GR_MGT, LV_FCTPUB1, "->PNIO_register_cbf: cbf=%#x", Cbf);

    ICommon *pInst = ICommon::get_instance(ApplHandle);
    if(pInst) {
        Ret = ((IController *) pInst)->register_pe_cbf(Cbf);
    }
    else {
        Ret = PNIO_ERR_WRONG_HND;
    }

    TRC_OUT01(GR_MGT, LV_FCTPUB1, "<-PNIO_register_cbf: ret=%#x", Ret);
    DPR_MUTEX_UNLOCK(PnioMutexController);

    return Ret;
}
#endif /* PROFI_ENERGY */
