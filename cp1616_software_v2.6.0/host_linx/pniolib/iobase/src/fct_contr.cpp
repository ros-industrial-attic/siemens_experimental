/****************************************************************************
*  Copyright (C) SIEMENS CORP., 2013 All rights reserved.
*****************************************************************************
* FILE NAME    : fct_contr.cpp
* ---------------------------------------------------------------------------
* DESCRIPTION  : Controller class implementations
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

#include "cp16xx.h"
#include "pniointr.h"
#include "pnioag_rqb.h"
#include "iodataupdate.h"
#include "fct_contr.h"

#define PNIO_CEP_NO_COMFORT_ALARM_HDL   0x00000001

#define BASIC_LENGTH(x)    (sizeof((x).blk_len) +\
                            sizeof((x).opcode) +\
                            sizeof((x).handle) +\
                            sizeof((x).agent_ret) +\
                            sizeof((x).resp_ret))

#define NEW_ORDERID() (++m_uOrderId)

/*===========================================================================
* FUNCTION : IController::IController
*----------------------------------------------------------------------------
* PURPOSE  : Constructor
*----------------------------------------------------------------------------
* RETURNS  :
*----------------------------------------------------------------------------
* INPUTS   : - m_pCbf_AlarmInd - Callback function for Alarm
*            - m_pCbf_DsReadConf - Callback function for Datarec Read
*            - m_pCbf_DsWriteConf - Callback function for Datarec write
*            - m_OpenExtPar - Extended parameter
* OUTPUS   : -
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
IController::IController(void):
    ICommon(),
    m_pUserCbf_Mode(NULL),
    m_pUserCbf_DevAct(NULL),
    m_pUserCbf_Alarm(NULL),
    m_pUserCbf_DR_Read(NULL),
    m_pUserCbf_DR_Write(NULL),
    m_pCbf_DsReadConf(NULL),
    m_pCbf_DsWriteConf(NULL),
    m_pCbf_AlarmInd(NULL),
    m_pUserCbf_CpStopReq(NULL),
    m_pUserCbf_StartLedFlash(NULL),
    m_pUserCbf_StopLedFlash(NULL),
    m_pUserCbf_CtrlGetDiag(NULL),
    m_Mode(PNIO_MODE_OFFLINE),
    m_OpenExtPar(0),
    m_pInCache(0),
    m_CnsBndInArr(0),
    m_CnsBndInArrLen(0),
    m_pOutCache(0),
    m_CnsBndOutArr(0),
    m_CnsBndOutArrLen(0)
{
    m_bController = true;
    m_pHostKramIOTlbExt = NULL;  /* byte slice access: IO table extension, default: not present */
#ifdef PROFI_ENERGY
    m_pPeMgt = new cPeMgt(this);
#endif /* PROFI_ENERGY */
}

/*===========================================================================
* FUNCTION : IController::controller_open
*----------------------------------------------------------------------------
* PURPOSE  : Controller open request handler
*----------------------------------------------------------------------------
* RETURNS  : PNIO_OK on success; else PNIO_ERR_PRM_EXT_PAR or PNIO_ERR_INTERNAL
*----------------------------------------------------------------------------
* INPUTS   : CpIndex - Unique identification for the communication module
*                      (module index in the component configuration)
*            ExtPar - Extended parameter (see earlier definition),
*                     among others the following parameters can be
*                     applied via the OR logical operator:PNIO_MODE_CTRL
*            cbf_RecReadConf - Callback function for signaling the results
*                              of data set read tasks
*            cbf_RecWriteConf - Callback function for signaling the results
*                               of data set write tasks
*            cbf_AlarmInd - Callback function for signaling alarms
* OUTPUS   : pApplHandle - Handle, which is assigned to the controller.
*----------------------------------------------------------------------------
* COMMENTS : This controller handle must be used by user for any other
* function calls
*==========================================================================*/
PNIO_UINT32 IController::controller_open(PNIO_UINT32 CpIndex,
    PNIO_UINT32 ExtPar, PNIO_CBF cbf_RecReadConf,
    PNIO_CBF cbf_RecWriteConf, PNIO_CBF cbf_AlarmInd,
    PNIO_UINT32 * pApplHandle)
{
    IController *pThis = NULL;
    light_T_SYNCH_COMMAND Rq;
    PNIO_UINT32 Ret, sendLen, expRLen;

    if(!pApplHandle)
        return PNIO_ERR_PRM_HND;

    if(!cbf_RecReadConf || !cbf_RecWriteConf)
        return PNIO_ERR_PRM_CALLBACK;

    if(!(ExtPar & PNIO_CEP_MODE_CTRL))
        return PNIO_ERR_PRM_EXT_PAR;

    if(ExtPar & PNIO_CEP_MODE_CTRL) {
        // to do :
        // only one instance mit PNIO_CEP_MODE_CTRL on CP can exist
    }

    // in first Stage ComfortAlarmHandler is "must"
    ExtPar &= ~PNIO_CEP_NO_COMFORT_ALARM_HDL;

    pThis = new IController;

    if(NULL == pThis)
        return PNIO_ERR_NO_RESOURCE;

    pThis->m_OpenExtPar = ExtPar;
    pThis->m_pUserCbf_Alarm = cbf_AlarmInd;
    pThis->m_pUserCbf_DR_Read = cbf_RecReadConf;
    pThis->m_pUserCbf_DR_Write = cbf_RecWriteConf;

    TRC_OUT(GR_INIT, LV_FCTPUB1, "IController::controller_open : call InitCp");
    Ret = pThis->InitCp(CpIndex);

    if(Ret != PNIO_OK)
        goto controller_open_fail_init;

    sendLen = BASIC_LENGTH(Rq) + sizeof(Rq.u.open_ctrl_ext);
    expRLen = BASIC_LENGTH(Rq) + sizeof(Rq.u.resp_open_ctrl);

    memset(&Rq, 0, sizeof (Rq));
    Rq.blk_len = CPU_TO_LE(sendLen);
    Rq.opcode = (SYNCH_CHNL_OP)CPU_TO_LE(SCO_OPEN_CONTROLLER_EXT);
    Rq.u.open_ctrl.id = CPU_TO_LE(1/* instance in FW is always unic! CpIndex */);
    Rq.u.open_ctrl_ext.host_version = (PNIO_UINT32)PNIO_API_VERSION;

    if(pThis->m_OpenExtPar & PNIO_CEP_NO_COMFORT_ALARM_HDL)
        Rq.u.open_ctrl.ext_prm = CPU_TO_LE(CTRL_OPEN_EPAR_DIRECT);
    else
        Rq.u.open_ctrl.ext_prm = CPU_TO_LE(CTRL_OPEN_EPAR_ALARMHND);

#ifdef IRT_NODMA

    Rq.u.open_ctrl.dma_mem_addr = 0;
    Rq.u.open_ctrl.dma_mem_len = 0;
#else
  #ifdef IRT_DMA_VIA_HOST

    Rq.u.open_ctrl.dma_mem_addr = 0;
    Rq.u.open_ctrl.dma_mem_len  = 0;
#else
    Rq.u.open_ctrl.dma_mem_addr = CPU_TO_LE(DPR_PTR_TO_ULONG(pThis->m_pCpAdapter->pIRTDMAPhysAddr));
    Rq.u.open_ctrl.dma_mem_len = CPU_TO_LE(pThis->m_pCpAdapter->IRTDMALen);
#endif /* IRT_DMA_VIA_HOST */
#endif /* IRT_NODMA */

    Ret = pThis->SendReceiveSynch((char*)&Rq, sendLen, &expRLen);

    TRC_OUT01(GR_INIT, LV_FCTINT,
        "IController::controller_open received len = %d", expRLen);
    if(Ret != PNIO_OK) {
        goto controller_open_fail_send;
    }

    if(LE_TO_CPU(Rq.opcode) != SCO_OPEN_CONTROLLER_EXT) {
        TRC_OUT01(GR_INIT, LV_ERR,
            "IController::controller_open received unknown opcode = 0x%x",
            LE_TO_CPU(Rq.opcode));
        Ret = PNIO_ERR_INTERNAL;
        goto controller_open_fail_send;
    }

    if((Ret = ConvertAgentErrorCodeToApiErrorCode(LE_TO_CPU(Rq.agent_ret))) == PNIO_OK)
        Ret = LE_TO_CPU(Rq.resp_ret);

    if(Ret == PNIO_OK) {
        IODATA_adr_info adr_info;

        // save user callback in instance structure
        pThis->m_pCbf_DsReadConf = cbf_RecReadConf;
        pThis->m_pCbf_DsWriteConf = cbf_RecWriteConf;
        pThis->m_pCbf_AlarmInd = cbf_AlarmInd;

        TRC_OUT(GR_INIT, LV_FCTINT, "IController::controller_open SendReceiveSynch OK");

        *pApplHandle = ICommon::get_handle(pThis);
        pThis->m_hIodataUpdate = LE_TO_CPU(Rq.u.resp_open_ctrl.iodataupdate_hnd);

        KRAMIOTLB_Header iotlb_hdr;
        KRAMIOTLB_Item **iotlb_arr;
        PNIO_UINT32 iotlb_items_count = 0;

        KRAMIOTLB_init(pThis->m_pCpAdapter->pKramTlb, SIZE_KRAMTLB, &iotlb_hdr);

        /* get count of KRAMIOTLB_Item(s) in KRAM */
        KRAMIOTLB_GetHandleItems(pThis->m_hIodataUpdate, 0,
            &iotlb_items_count, NULL /* don't fill the array */, &iotlb_hdr);

        /* create temporary array of (KRAMIOTLB_Item *) to read from KRAM */
        iotlb_arr = (KRAMIOTLB_Item **)malloc(sizeof(KRAMIOTLB_Item *) * iotlb_items_count);
        DPR_ASSERT(iotlb_arr);

        /* fill temporary (KRAMIOTLB_Item *) array */
        KRAMIOTLB_GetHandleItems(pThis->m_hIodataUpdate, 0,
            &iotlb_items_count, iotlb_arr, &iotlb_hdr);

        /* create copy of configuration (list of KRAMIOTLB_Item) in user space */
        pThis->m_pHostIOTlb = (KRAMIOTLB_Item *)malloc(sizeof(*pThis->m_HostIOTlbHdr.pMaxItemPos) +
            sizeof(KRAMIOTLB_Item) * iotlb_items_count);

        /* initialize copy of configuration */
        KRAMIOTLB_init(pThis->m_pHostIOTlb, iotlb_items_count * sizeof(KRAMIOTLB_Item),
            &pThis->m_HostIOTlbHdr);

        *pThis->m_HostIOTlbHdr.pMaxItemPos = CPU_TO_LE(iotlb_items_count - 1);

        KRAMIOTLB_Item *pLocalKramItemsBeg = (KRAMIOTLB_Item *)((char *)pThis->m_pHostIOTlb +
            sizeof(*pThis->m_HostIOTlbHdr.pMaxItemPos));

        /* copy KRAMIOTLB_Item from KRAM to local application memory */
        for(PNIO_UINT32 i=0; i<iotlb_items_count; i++)
            pLocalKramItemsBeg[i] = *(iotlb_arr[i]);

        if(iotlb_arr)
            free(iotlb_arr);

        adr_info.IOTLB_base = (PNIO_UINT8 *)pThis->m_pHostIOTlb;
        adr_info.iotlb_len = iotlb_items_count * sizeof(KRAMIOTLB_Item);

        adr_info.EREG_base = (PNIO_UINT8 *)pThis->m_pCpAdapter->pErtecSwiBase;
        adr_info.KRAM_base = (PNIO_UINT8 *)pThis->m_pCpAdapter->pErtecIOTotal;
#ifdef IRT_NODMA
        adr_info.DMA_base = NULL;
#else
        adr_info.DMA_base = (PNIO_UINT8 *)pThis->m_pCpAdapter->pIRTDMAImage;
#endif

        adr_info.IrtAccessStatus = (PNIO_UINT8 *)&pThis->m_pCpAdapter->IrtAccessStatus;

        TRC_OUT02(GR_INIT, LV_FCTINT, "IODU_ctrl_open(0x%x, 0x%x)...",
            pThis->m_hIodataUpdate, *pApplHandle);

        Ret = IODU_ctrl_open(CpIndex, pThis->m_hIodataUpdate, pThis->m_OpenExtPar,
            &adr_info, &pThis->m_pCpAdapter->pIODUItem);

        if(Ret != PNIO_OK) {
            TRC_OUT01(GR_INIT, LV_ERR, "Error IODU_ctrl_open returned 0x%x", Ret);
            pThis->controller_close();
            return Ret;
        }

        /* CBSA: Controller Byte Slice Access extension */
        if ( pThis->m_OpenExtPar & PNIO_CEP_SLICE_ACCESS ) {
            pThis->m_pHostKramIOTlbExt = new HostKramIOTableExt(&pThis->m_HostIOTlbHdr);
            pThis->m_pHostKramIOTlbExt->updateAddressHash();
        }

        PNIO_UINT32      ItemsToExcludeNum = 0;
        KRAMIOTLB_Item * pItemsToExclude = NULL;

#ifdef IO_ROUTER

        IoConcentrator::PURE_MODULE_INFO * pPureTransferItems = NULL;

        pThis->m_Ioc.ioConcentratorInit(pThis, &(pThis->m_TransferWD));
        if (pThis->m_TransferWD.Init(&(pThis->m_Ioc), pThis)) {
            ((ICommon *)pThis)->CP_register_cbf_transferwd(&(pThis->transfer_wd_cbf_wrapper));
        }

        // get list of modules that will be used only from IORouter
        ItemsToExcludeNum = pThis->m_Ioc.getNumOfPureTransferModules();

        if (ItemsToExcludeNum != 0) {
            pPureTransferItems = new IoConcentrator::PURE_MODULE_INFO[ItemsToExcludeNum];
            if (!pPureTransferItems) {
                Ret = PNIO_ERR_OS_RES;
                goto controller_router_init_fail;
            }
            pThis->m_Ioc.getPureTransferModuleList(pPureTransferItems, ItemsToExcludeNum);

            pItemsToExclude = new KRAMIOTLB_Item[ItemsToExcludeNum];
            if (!pItemsToExclude) {
                Ret = PNIO_ERR_OS_RES;
                if(pPureTransferItems)
                    delete [] pPureTransferItems;
                goto controller_router_init_fail;
            }

            memset(pItemsToExclude, 0, sizeof(KRAMIOTLB_Item) * ItemsToExcludeNum);

            for(PNIO_UINT32 ti = 0; ti < ItemsToExcludeNum; ++ti) {
                pItemsToExclude[ti].log_addr = pPureTransferItems[ti].log_addr;

                if(pPureTransferItems[ti].in_out_type & IoConcentrator::IO_IN)
                    pItemsToExclude[ti].io_out_type =
                        (KRAMIOTLB_IN_OUT_TYPE)(pItemsToExclude[ti].io_out_type | KRAMIOTLB_IO_IN);

                if(pPureTransferItems[ti].in_out_type & IoConcentrator::IO_OUT) {
                    pItemsToExclude[ti].log_addr |= 0x8000;
                    pItemsToExclude[ti].io_out_type =
                        (KRAMIOTLB_IN_OUT_TYPE)(pItemsToExclude[ti].io_out_type | KRAMIOTLB_IO_OUT);
                }

                if(pPureTransferItems[ti].in_out_type & IoConcentrator::IO_PDEV)
                    pItemsToExclude[ti].io_out_type =
                        (KRAMIOTLB_IN_OUT_TYPE)(pItemsToExclude[ti].io_out_type | KRAMIOTLB_IO_PDEV);

                if(pPureTransferItems[ti].in_out_type & IoConcentrator::IO_DDEX)
                    pItemsToExclude[ti].io_out_type =
                        (KRAMIOTLB_IN_OUT_TYPE)(pItemsToExclude[ti].io_out_type | KRAMIOTLB_IO_DDEX);
            }

            if(pPureTransferItems)
                delete [] pPureTransferItems;
        }

#endif /* IO_ROUTER */

        Ret = IODU_ctrl_cache_init(pThis->m_pCpAdapter->pIODUItem,
            &pThis->m_pOutCache,
            &pThis->m_CnsBndOutArr, &pThis->m_CnsBndOutArrLen,
            &pThis->m_pInCache,
            &pThis->m_CnsBndInArr, &pThis->m_CnsBndInArrLen,
            pItemsToExclude, ItemsToExcludeNum);

        if(pItemsToExclude)
            delete [] pItemsToExclude;

        if(Ret != PNIO_OK) {
            TRC_OUT01(GR_INIT, LV_ERR,
                "Error IODU_ctrl_cache_init returned 0x%x", Ret);
            pThis->controller_close();
            return Ret;
        }

#ifndef IRT_NODMA
  #ifdef IRT_DMA_VIA_HOST
        Ret = pThis->init_irt_dma_ranges();
        TRC_OUT01(GR_INIT, LV_FCTINT, "init_irt_dma_ranges() returned 0x%x", Ret);
  #endif /* IRT_DMA_VIA_HOST */
#endif /* IRT_NODMA */

        /* copy output APDU status bytes from DPRAM memory to DMA memory, to prevent garbage */
        if(adr_info.DMA_base) {
            PNIO_UINT32 offset_min, offset_max, kret;

            TRC_OUT(GR_STATE, LV_FCTPUB1, "save APDU states");

            offset_min = offset_max = 0;
            kret = KRAMIOTLB_GetDataOffset(pThis->get_iodataupdate_handle(),
                KRAMIOTLB_IO_OUT, KRAMIOTLB_IO_SYNC,
                &offset_min, &offset_max, pThis->get_iotlbhdr());
            if(kret == KRAMIOTLB_OK) {
                memcpy(adr_info.DMA_base + offset_min,
                adr_info.KRAM_base + offset_min,
                (offset_max - offset_min));
            }
        }
    } else {
        TRC_OUT01(GR_INIT, LV_ERR, "Error SendReceiveSynch returned 0x%x", Ret);
        goto controller_open_fail_send;
    }

#ifdef PROFI_ENERGY
    /* PE (PROFIenergy) handler init */
    TRC_OUT(GR_INIT, LV_INFO, "IController::controller_open: ->PE init()");
    pThis->m_pPeMgt->init();
#endif /* PROFI_ENERGY */
    return PNIO_OK;

#ifdef IO_ROUTER
controller_router_init_fail:
    pThis->m_TransferWD.Uninit();
    pThis->m_Ioc.ioConcentratorUnInit();
#endif /* IO_ROUTER */

controller_open_fail_send:
    pThis->UninitCp();

controller_open_fail_init:
    delete(pThis);

    return Ret;
}

/*===========================================================================
* FUNCTION : IController::controller_close
*----------------------------------------------------------------------------
* PURPOSE  : Controller close
*----------------------------------------------------------------------------
* RETURNS  : PNIO_OK on success; else PNIO_ERR_SEQUENCE
*----------------------------------------------------------------------------
* INPUTS   : -
* OUTPUS   : -
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
PNIO_UINT32 IController::controller_close(void)
{
    light_T_SYNCH_COMMAND Rq;
    PNIO_UINT32 Ret, sendLen, expRLen;

    TRC_OUT(GR_INIT, LV_FCTINT, "IODU_ctrl_close...");

#ifdef PROFI_ENERGY
    /* PE (PROFIenergy) handler un-init */
    TRC_OUT(GR_INIT, LV_INFO, "IController::controller_close: ->PE uninit()");
    m_pPeMgt->uninit();
#endif /* PROFI_ENERGY */

#ifdef IO_ROUTER
    m_TransferWD.Uninit();
    m_Ioc.ioConcentratorUnInit();
#endif /* IO_ROUTER */

    deinit_and_unregister();
    m_bClosePending = true;

    Ret = IODU_ctrl_close(m_pCpAdapter->pIODUItem);
    if(Ret != PNIO_OK)
        TRC_OUT02(GR_INIT, LV_ERR, "Error IODU_ctrl_close(0x%x) returned 0x%x, ignore",
            m_hIodataUpdate, Ret);

    sendLen = BASIC_LENGTH(Rq) + sizeof(Rq.u.close_ctrl);
    expRLen = BASIC_LENGTH(Rq);

    memset(&Rq, 0, sizeof (Rq));
    Rq.blk_len = CPU_TO_LE(sendLen);
    Rq.opcode = (SYNCH_CHNL_OP)CPU_TO_LE(SCO_CLOSE);
    Rq.handle = CPU_TO_LE(NEW_ORDERID());
    Rq.u.close_ctrl.EmergencyClose = m_bEmergencyClose ? 0x01 : 0x00;

    Ret = SendReceiveSynch((char *)&Rq, sendLen, &expRLen, 90000);

    if(Ret != PNIO_OK) {
        TRC_OUT01(GR_INIT, LV_ERR,
            "IController::controller_close SendReceiveSynch ret = 0x%x", Ret);
        goto controller_close_fail;
    }

    if(LE_TO_CPU(Rq.opcode) != SCO_CLOSE) {
        Ret = PNIO_ERR_INTERNAL;
        TRC_OUT01(GR_INIT, LV_ERR,
            "IController::controller_close received unknown opcode = 0x%x",
            LE_TO_CPU(Rq.opcode));
        goto controller_close_fail;
    }

    if((Ret = ConvertAgentErrorCodeToApiErrorCode(LE_TO_CPU(Rq.agent_ret))) == PNIO_OK)
        Ret = LE_TO_CPU(Rq.resp_ret);

    if(Ret == PNIO_ERR_WRONG_HND) {
        Ret = PNIO_OK;
        TRC_OUT(GR_INIT, LV_ERR,
            "IController::controller_close received PNIO_ERR_WRONG_HND, ignore");
    }

    if(Ret != PNIO_OK) {
        TRC_OUT01(GR_INIT, LV_ERR,
            "IController::controller_close Rq.resp_ret = 0x%x", Ret);
    } else {
        if (m_pHostKramIOTlbExt) {
            delete m_pHostKramIOTlbExt;
        }
        if(m_pHostIOTlb)
            free(m_pHostIOTlb);
        if(m_pInCache)
            delete [] m_pInCache;
        if(m_CnsBndInArr)
            delete [] m_CnsBndInArr;
        if(m_pOutCache)
            delete [] m_pOutCache;
        if(m_CnsBndOutArr)
            delete [] m_CnsBndOutArr;
        UninitCp();

        delete(this);
    }

controller_close_fail:
    return Ret;
}

/*===========================================================================
* FUNCTION : IController::register_cbf
*----------------------------------------------------------------------------
* PURPOSE  : function to do register_cbf calls
*----------------------------------------------------------------------------
* RETURNS  : PNIO_ERR_ALLREADY_DONE if already set.
*----------------------------------------------------------------------------
* INPUTS   : CbeType  - Calback type
*            cbf  - Callback function
* OUTPUS   :
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
PNIO_UINT32 IController::register_cbf(PNIO_CBE_TYPE CbeType, PNIO_CBF cbf)
{
    PNIO_UINT32 Ret;

    if(!cbf)
        return PNIO_ERR_PRM_CALLBACK;

    if(m_Mode != PNIO_MODE_OFFLINE)
        return PNIO_ERR_MODE_VALUE;

    switch (CbeType) {
    case PNIO_CBE_MODE_IND:
        if(m_pUserCbf_Mode) {
            Ret = PNIO_ERR_ALREADY_DONE;
        } else {
            m_pUserCbf_Mode = cbf;
            Ret = PNIO_OK;
        }
        break;
    case PNIO_CBE_DEV_ACT_CONF:
        if(m_pUserCbf_DevAct) {
            Ret = PNIO_ERR_ALREADY_DONE;
        } else {
            m_pUserCbf_DevAct = cbf;
            Ret = PNIO_OK;
        }
        break;
    case PNIO_CBE_CP_STOP_REQ:
        if(m_pUserCbf_CpStopReq) {
            Ret = PNIO_ERR_ALREADY_DONE;
        } else {
            light_T_SYNCH_COMMAND Rq;
            PNIO_UINT32 sendLen, expRLen;

            m_pUserCbf_CpStopReq = cbf;

            sendLen = BASIC_LENGTH(Rq);
            expRLen = BASIC_LENGTH(Rq);

            memset(&Rq, 0, sizeof (Rq));
            Rq.blk_len = CPU_TO_LE(sendLen);
            Rq.opcode = (SYNCH_CHNL_OP)CPU_TO_LE(SCO_REGISTER_STOP_CBF);
            Rq.handle = CPU_TO_LE(NEW_ORDERID());

            Ret = SendReceiveSynch((char *)&Rq, sendLen, &expRLen);

            if(Ret != PNIO_OK)
                return Ret;

            if(LE_TO_CPU(Rq.opcode) != SCO_REGISTER_STOP_CBF) {
                TRC_OUT01(GR_STATE, LV_ERR,
                    "IController::register_cbf(SCO_REGISTER_STOP_CBF) received unknown opcode = 0x%x",
                    LE_TO_CPU(Rq.opcode));
                return PNIO_ERR_INTERNAL;
            }

            if((Ret = ConvertAgentErrorCodeToApiErrorCode(LE_TO_CPU(Rq.agent_ret))) == PNIO_OK)
                Ret = LE_TO_CPU(Rq.resp_ret);

            return Ret;
        }
        break;

    case PNIO_CBE_START_LED_FLASH:
        if(m_pUserCbf_StartLedFlash) {
            Ret = PNIO_ERR_ALREADY_DONE;
        } else {
            m_pUserCbf_StartLedFlash = cbf;
            Ret = PNIO_OK;
        }
        break;

    case PNIO_CBE_STOP_LED_FLASH:
        if(m_pUserCbf_StopLedFlash) {
            Ret = PNIO_ERR_ALREADY_DONE;
        } else {
            m_pUserCbf_StopLedFlash = cbf;
            Ret = PNIO_OK;
        }
        break;

    case PNIO_CBE_CTRL_DIAG_CONF:
        if(m_pUserCbf_CtrlGetDiag) {
            Ret = PNIO_ERR_ALLREADY_DONE;
        } else {
            m_pUserCbf_CtrlGetDiag = cbf;
            Ret = PNIO_OK;
        }
        break;

    default:
        Ret = PNIO_ERR_PRM_TYPE;
        break;
    }

    return Ret;
}


#ifdef PROFI_ENERGY
/*===========================================================================
* FUNCTION : IController::register_pe_cbf
*----------------------------------------------------------------------------
*==========================================================================*/

PNIO_UINT32 IController::register_pe_cbf(PNIO_PE_CBF cbf)
{
    /* cbf pointer == NULL means de-register, is allowed
    if ( !cbf ) {
        return PNIO_ERR_PRM_CALLBACK;
    }
    */
    /*  always allowed
    if ( m_Mode != PNIO_MODE_OFFLINE ) {
        return PNIO_ERR_MODE_VALUE;
    }
    */
    /* re-register is possible
    if ( m_pUserCbf_PE ) {
        return = PNIO_ERR_ALREADY_DONE;
    }
    */
    m_pPeMgt->m_pUserCbf_PE = cbf;
    return PNIO_OK;
}
#endif /* PROFI_ENERGY */

/*===========================================================================
* FUNCTION : IController::set_mode
*----------------------------------------------------------------------------
* PURPOSE  : Changing Controller modes
*----------------------------------------------------------------------------
* RETURNS  : PNIO_OK on success;
*            on error PNIO_ERR_MODE_VALUE, PNIO_ERR_SET_MODE_NOT_ALLOWED
*----------------------------------------------------------------------------
* INPUTS   : - Mode - requested mode.
* OUTPUS   : -
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
PNIO_UINT32 IController::set_mode(PNIO_MODE_TYPE Mode)
{
    light_T_SYNCH_COMMAND Rq;
    PNIO_UINT32 Ret, sendLen, expRLen;

    if(!(m_OpenExtPar & PNIO_CEP_MODE_CTRL))
        return PNIO_ERR_SET_MODE_NOT_ALLOWED;

    if(Mode != PNIO_MODE_OFFLINE &&
        Mode != PNIO_MODE_CLEAR &&
        Mode != PNIO_MODE_OPERATE)
        return PNIO_ERR_MODE_VALUE;

    if(m_Mode == Mode)
        return PNIO_OK;

    sendLen = BASIC_LENGTH(Rq) + sizeof (Rq.u.set_mode);
    expRLen = BASIC_LENGTH(Rq);

    memset(&Rq, 0, sizeof (Rq));
    Rq.blk_len = CPU_TO_LE(sendLen);
    Rq.opcode = (SYNCH_CHNL_OP)CPU_TO_LE(SCO_SET_MODE);
    Rq.handle = CPU_TO_LE(NEW_ORDERID());
    Rq.u.set_mode = (PNIO_MODE_TYPE)CPU_TO_LE(Mode);

    Ret = SendReceiveSynch((char *)&Rq, sendLen, &expRLen);

    if(Ret != PNIO_OK)
        return Ret;

    if(LE_TO_CPU(Rq.opcode) != SCO_SET_MODE) {
        TRC_OUT01(GR_STATE, LV_ERR,
            "IController::set_mode received unknown opcode = 0x%x",
            LE_TO_CPU(Rq.opcode));
        return PNIO_ERR_INTERNAL;
    }

    if((Ret = ConvertAgentErrorCodeToApiErrorCode(LE_TO_CPU(Rq.agent_ret))) == PNIO_OK)
        Ret = LE_TO_CPU(Rq.resp_ret);

    return Ret;
}

/*===========================================================================
* FUNCTION : IController::device_activate
*----------------------------------------------------------------------------
* PURPOSE  : Device activate function
*----------------------------------------------------------------------------
* RETURNS  : PNIO_OK on success;
*            on error PNIO_ERR_PRM_ADD, PNIO_ERR_MODE_VALUE
*----------------------------------------------------------------------------
* INPUTS   : - pAddr - Logical address of device
*            - mode - requested device activation mode- PNIO_DA_TRUE/PNIO_DA_FALSE
* OUTPUS   : -
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
PNIO_UINT32 IController::device_activate(const PNIO_ADDR * pAddr,
    PNIO_DEV_ACT_TYPE mode)
{
    if(!pAddr ||
        pAddr->AddrType != PNIO_ADDR_LOG ||
        (pAddr->IODataType != PNIO_IO_IN && pAddr->IODataType != PNIO_IO_OUT))
        return PNIO_ERR_PRM_ADD;

    // sanity addres bound
    if(pAddr->u.Addr > 0x7FFF)
        return PNIO_ERR_PRM_ADD;

    if(!(m_OpenExtPar & PNIO_CEP_MODE_CTRL))
        return PNIO_ERR_DEV_ACT_NOT_ALLOWED;

    if(mode != PNIO_DA_FALSE && mode != PNIO_DA_TRUE)
        return PNIO_ERR_MODE_VALUE;

    // if(!m_pUserCbf_DevAct) return PNIO_ERR_UNREG_DEV_ACT_CALLBACK;

    light_T_SYNCH_COMMAND Rq;
    PNIO_UINT32 Ret, sendLen, expRLen;

    sendLen = BASIC_LENGTH(Rq) + sizeof (Rq.u.dev_state);
    expRLen = BASIC_LENGTH(Rq);

    memset(&Rq, 0, sizeof (Rq));
    Rq.blk_len = CPU_TO_LE(sendLen);
    Rq.opcode = (SYNCH_CHNL_OP)CPU_TO_LE(SCO_SET_DEV_STATE);
    Rq.handle = CPU_TO_LE(NEW_ORDERID());
    Rq.u.dev_state.Addr.AddrType = (PNIO_ADDR_TYPE)CPU_TO_LE(pAddr->AddrType);
    Rq.u.dev_state.Addr.IODataType = (PNIO_IO_TYPE)CPU_TO_LE(pAddr->IODataType);
    Rq.u.dev_state.Addr.u.Addr = CPU_TO_LE(pAddr->u.Addr);

    Rq.u.dev_state.DevActMode = (PNIO_DEV_ACT_TYPE)CPU_TO_LE(mode);

    Ret = SendReceiveSynch((char *)&Rq, sendLen, &expRLen);

    if(Ret != PNIO_OK)
        return Ret;

    if(LE_TO_CPU(Rq.opcode) != SCO_SET_DEV_STATE) {
        TRC_OUT01(GR_STATE, LV_ERR,
            "IController::device_activate received unknown opcode = 0x%x",
            LE_TO_CPU(Rq.opcode));

        return PNIO_ERR_INTERNAL;
    }

    if((Ret = ConvertAgentErrorCodeToApiErrorCode(LE_TO_CPU(Rq.agent_ret))) == PNIO_OK)
        Ret = LE_TO_CPU(Rq.resp_ret);

    return Ret;
}

/*===========================================================================
* FUNCTION : IController::perf_ds
*----------------------------------------------------------------------------
* PURPOSE  : dataset read write
*----------------------------------------------------------------------------
* RETURNS  : PNIO_OK on success;
*            on error PNIO_ERR_PRM_ADD, PNIO_ERR_PRM_BUF
*----------------------------------------------------------------------------
* INPUTS   : pAddr - Logical address of device
*            accesst - access type
*            ReqRef  - user req ref
*            RecordIndex  - Dataset RecordIndex
*            Length  - length
*            pBuffer  - data for write
* OUTPUS   : - pBuffer  - data for read
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
PNIO_UINT32 IController::perf_ds(PNIO_ADDR * pAddr,
    ACC_T accesst, PNIO_REF ReqRef, PNIO_UINT32 RecordIndex,
    PNIO_UINT32 Length, PNIO_UINT8 * pBuffer)
{
    if(!pAddr ||
        pAddr->AddrType != PNIO_ADDR_LOG ||
        (pAddr->IODataType != PNIO_IO_IN && pAddr->IODataType != PNIO_IO_OUT))
        return PNIO_ERR_PRM_ADD;

    // sanity addres bound
    if(pAddr->u.Addr > 0x7FFF)
        return PNIO_ERR_PRM_ADD;

    if(Length == 0 || Length > PNIO_MAX_REC_LEN)
        return PNIO_ERR_VALUE_LEN;

    if(Length > PNIOI_DREC_MAX_SIZE) {
        TRC_OUT01(GR_DS, LV_WARN,
            "IController::perf_ds Length=%u -> PNIOI_DREC_MAX_SIZE(%d)", Length);
        Length = PNIOI_DREC_MAX_SIZE;
    }

    if(accesst == ACC_T_WRITE) {
        if(!pBuffer && Length)
            return PNIO_ERR_PRM_BUF;
    } else if(accesst == ACC_T_READ) {
    }

    if(RecordIndex > 0xFFFF)
        return PNIO_ERR_PRM_REC_INDEX;

    T_SYNCH_COMMAND Rq;
    PNIO_UINT32 Ret, sendLen, expRLen;

    sendLen = (BASIC_LENGTH(Rq) + sizeof (Rq.u.rw_dr) - PNIOI_DREC_MAX_SIZE);
    if(accesst == ACC_T_WRITE)
        sendLen += Length;

    expRLen = BASIC_LENGTH(Rq);

    memset(&Rq, 0, sizeof (Rq));
    Rq.blk_len = CPU_TO_LE(sendLen);
    Rq.opcode = (SYNCH_CHNL_OP)CPU_TO_LE(SCO_RW_DR);
    Rq.handle = CPU_TO_LE(NEW_ORDERID());
    Rq.u.rw_dr.UserRef = CPU_TO_LE(ReqRef);
    Rq.u.rw_dr.ActionType = (PNAGC_DR_RQ_TYPE)((accesst == ACC_T_READ) ?
        CPU_TO_LE(PNAGC_READ_DR) : CPU_TO_LE(PNAGC_WRITE_DR));
    Rq.u.rw_dr.Addr.AddrType = (PNIO_ADDR_TYPE)CPU_TO_LE(pAddr->AddrType);
    Rq.u.rw_dr.Addr.IODataType = (PNIO_IO_TYPE)CPU_TO_LE(pAddr->IODataType);
    Rq.u.rw_dr.Addr.u.Addr = CPU_TO_LE(pAddr->u.Addr);

    Rq.u.rw_dr.RecordIndex = CPU_TO_LE(RecordIndex);
    Rq.u.rw_dr.Length = CPU_TO_LE(Length);

    if(accesst == ACC_T_WRITE)
        memcpy(Rq.u.rw_dr.Buffer, pBuffer, Length);

    Ret = SendReceiveSynch((char *)&Rq, sendLen, &expRLen);

    if(Ret != PNIO_OK)
        return Ret;

    if(LE_TO_CPU(Rq.opcode) != SCO_RW_DR) {
        TRC_OUT01(GR_DS, LV_ERR,
            "IController::perf_ds received unknown opcode = 0x%x",
            LE_TO_CPU(Rq.opcode));

        return PNIO_ERR_INTERNAL;
    }

    if((Ret = ConvertAgentErrorCodeToApiErrorCode(LE_TO_CPU(Rq.agent_ret))) == PNIO_OK)
        Ret = LE_TO_CPU(Rq.resp_ret);

    return Ret;
}


#ifdef PROFI_ENERGY
/*==========================================================================*/

PNIO_UINT32 IController::perf_pe(PNIO_UINT32 Handle, PNIO_ADDR * pAddr, PNIO_REF ReqRef, PNIO_PE_REQ_PRM *pPeReqPrm)
{
    PNIO_UINT32 pnioRet;

    if(!pAddr ||  pAddr->AddrType != PNIO_ADDR_LOG ||
        (pAddr->IODataType != PNIO_IO_IN && pAddr->IODataType != PNIO_IO_OUT)) {
        return PNIO_ERR_PRM_ADD;
    }

    // sanity addres bound
    if(pAddr->u.Addr > 0x7FFF) {
        return PNIO_ERR_PRM_ADD;
    }

    // check if PE called already registered - it has to
    if (!m_pPeMgt->m_pUserCbf_PE) {
        return PNIO_ERR_SEQUENCE;
    }

    pnioRet = m_pPeMgt->handle_pe_cmd_request(Handle, pAddr, ReqRef, pPeReqPrm);

    return pnioRet;
} //
#endif /* PROFI_ENERGY */

/*===========================================================================
* FUNCTION : IController::perf_io
*----------------------------------------------------------------------------
* PURPOSE  : io read write
*----------------------------------------------------------------------------
* RETURNS  : PNIO_OK on success;
*            on error PNIO_ERR_PRM_RSTATE, PNIO_ERR_PRM_BUF
*----------------------------------------------------------------------------
* INPUTS   : - pAddr - Logical address of device
*              accesst  - access type
*              pLocStat  - Local IO remote/local status
*              pRemStat  - Rem IO remote/local status
*              pDataLen  - data length
*              pData  - data for write
* OUTPUS   : - pData  - data for read
*----------------------------------------------------------------------------
* COMMENTS : angelehnt an PNIO_IO_read_write_request()
*
* N.B.  - THIS FUNCTION TO BE MODIFIED FOR IMPLEMENTING DIRECT ACCESS TO KRAM
*
*==========================================================================*/
PNIO_UINT32 IController::perf_io(PNIO_ADDR * pAddr,
    ACC_T accesst, PNIO_IOXS * pLocStat, PNIO_IOXS * pRemStat,
    PNIO_UINT32 * pDataLen, PNIO_UINT8 * pData, ICONT_MEDIUM_TYPE Med)
{
    PNIO_UINT32 Ret;

    if(!pData && (*pDataLen))
        return PNIO_ERR_PRM_BUF;
    if(!pRemStat)
        return PNIO_ERR_PRM_RSTATE;

    if(!pAddr ||
        pAddr->AddrType != PNIO_ADDR_LOG ||
        (pAddr->IODataType != PNIO_IO_IN && pAddr->IODataType != PNIO_IO_OUT))
        return PNIO_ERR_PRM_ADD;

    // sanity addres bound
    if(pAddr->u.Addr > 0x7FFF)
        return PNIO_ERR_PRM_ADD;

    if(accesst == ACC_T_WRITE) {
        if(pAddr->IODataType != PNIO_IO_OUT) {
            // user can write io-data only from type PNIO_IO_OUT
            return PNIO_ERR_PRM_IO_TYPE;
        }
    } else if(accesst == ACC_T_READ) {
        if(pAddr->IODataType != PNIO_IO_IN) {
            // user can read io-data only from type PNIO_IO_IN
            return PNIO_ERR_PRM_IO_TYPE;
        }
    } else if(accesst == ACC_T_READ_OUTPUT) {
        // user can read(ACC_T_READ_OUTPUT) io-data only from type PNIO_IO_OUT
        if(pAddr->IODataType != PNIO_IO_OUT) {
            return PNIO_ERR_PRM_IO_TYPE;
        }
    } else {
        return PNIO_ERR_PRM_IO_TYPE;
    }

    if(*pDataLen > PNIO_MAX_IO_LEN)
        return PNIO_ERR_VALUE_LEN;
    if(*pLocStat != PNIO_S_GOOD && *pLocStat != PNIO_S_BAD)
        return PNIO_ERR_PRM_LOC_STATE;


    // iodataupdate help us todo this io job
    if(accesst == ACC_T_READ) {
        if(Med == ICONT_MT_CACHE) {
            Ret = IODU_ctrl_data_read_ex(m_pCpAdapter->pIODUItem,
                pAddr->u.Addr, *pDataLen, pDataLen, pData, *pLocStat, pRemStat,
                m_CnsBndInArr ? m_CnsBndInArr[0].l_offset : 0,
                m_CnsBndOutArr ? m_CnsBndOutArr[0].l_offset : 0,
                m_pInCache, m_pOutCache);
        } else {
            Ret = IODU_ctrl_data_read(m_pCpAdapter->pIODUItem, pAddr->u.Addr,
                *pDataLen, pDataLen, pData, *pLocStat, pRemStat,
                m_pOutCache, m_CnsBndOutArr ? m_CnsBndOutArr[0].l_offset : 0);
        }
    } else if (accesst == ACC_T_WRITE) {
        if(Med == ICONT_MT_CACHE) {
#ifdef IO_ROUTER
            IOR_CONCENTRATOR_ENTRY *pSubMod;
            if( (pSubMod = m_Ioc.getIocModule(pAddr->u.Addr)) != NULL ) {
                // is a shared module -> io concentrator write_ex required
                Ret = m_Ioc.IOC_data_write_cache(pAddr->u.Addr, pSubMod, *pDataLen,
                    pData, *pLocStat, pRemStat);
            } else {
                Ret = IODU_ctrl_data_write_ex(m_pCpAdapter->pIODUItem,
                    0x8000 | pAddr->u.Addr, *pDataLen, pData, *pLocStat, pRemStat,
                    m_CnsBndInArr ? m_CnsBndInArr[0].l_offset : 0,
                    m_CnsBndOutArr ? m_CnsBndOutArr[0].l_offset : 0,
                    m_pInCache, m_pOutCache);
            }

#else
            Ret = IODU_ctrl_data_write_ex(m_pCpAdapter->pIODUItem,
                0x8000 | pAddr->u.Addr, *pDataLen, pData, *pLocStat, pRemStat,
                m_CnsBndInArr ? m_CnsBndInArr[0].l_offset : 0,
                m_CnsBndOutArr ? m_CnsBndOutArr[0].l_offset : 0,
                m_pInCache, m_pOutCache);
#endif /* IO_ROUTER */

        } else {
#ifdef IO_ROUTER
            IOR_CONCENTRATOR_ENTRY *pSubMod;
            if( (pSubMod = m_Ioc.getIocModule(pAddr->u.Addr)) != NULL ) {
                // is a shared module -> io concentrator write required
                Ret = m_Ioc.IOC_data_write(pAddr->u.Addr, pSubMod, *pDataLen, pData, *pLocStat, pRemStat);
            } else {
                Ret = IODU_ctrl_data_write(m_pCpAdapter->pIODUItem, 0x8000 | pAddr->u.Addr,
                    *pDataLen, pData, *pLocStat, pRemStat,
                    m_pOutCache, m_CnsBndOutArr ? m_CnsBndOutArr[0].l_offset : 0);
            }
#else
            Ret = IODU_ctrl_data_write(m_pCpAdapter->pIODUItem, 0x8000 | pAddr->u.Addr,
                *pDataLen, pData, *pLocStat, pRemStat,
                m_pOutCache, m_CnsBndOutArr ? m_CnsBndOutArr[0].l_offset : 0);
#endif /* IO_ROUTER */

        }
    } else if( accesst == ACC_T_READ_OUTPUT ) {
        Ret = IODU_ctrl_kram_read(m_pCpAdapter->pIODUItem, 0x8000 | pAddr->u.Addr,
            pAddr->IODataType, *pDataLen, pDataLen, pData, pLocStat, pRemStat);
    } else {
        Ret = PNIO_ERR_INTERNAL;
    }

    return Ret;
}

void IController::read_cache_refresh()
{
    IODU_ctrl_data_read_cache_refresh(m_pCpAdapter->pIODUItem,
        m_pInCache, m_CnsBndInArr, m_CnsBndInArrLen);
}

void IController::write_cache_flush()
{
#ifdef IO_ROUTER

    if(m_Ioc.getNumOfIocModules()) {
        m_Ioc.IOC_update_cache();
    }

    IODU_ctrl_data_write_cache_flush(m_pCpAdapter->pIODUItem,
        m_pOutCache, m_CnsBndOutArr, m_CnsBndOutArrLen);

    m_Ioc.updateUserStatusFromCache(); // update status store after flush

    m_TransferWD.CacheSet();  // set uptodate flags
    m_TransferWD.KramSet();
#else

    IODU_ctrl_data_write_cache_flush(m_pCpAdapter->pIODUItem,
       m_pOutCache, m_CnsBndOutArr, m_CnsBndOutArrLen);
#endif /* IO_ROUTER */
}

/*===========================================================================
* FUNCTION : IController::alarm_resp
*----------------------------------------------------------------------------
* PURPOSE  : Alarm response
*----------------------------------------------------------------------------
* RETURNS  :
*----------------------------------------------------------------------------
* INPUTS   : - IndRef - the application assigned reference

* OUTPUS   : -
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
PNIO_UINT32 IController::alarm_resp(PNIO_REF IndRef)
{
    light_T_SYNCH_COMMAND Rq;
    PNIO_UINT32 Ret, sendLen, expRLen;

    sendLen = BASIC_LENGTH(Rq) + sizeof (Rq.u.ctrl_al_confirm);
    expRLen = BASIC_LENGTH(Rq);

    memset(&Rq, 0, sizeof (Rq));
    Rq.blk_len = CPU_TO_LE(sendLen);
    Rq.opcode = (SYNCH_CHNL_OP)CPU_TO_LE(SCO_CTRL_ALARM_CONFIRM);
    Rq.handle = CPU_TO_LE(NEW_ORDERID());
    Rq.u.ctrl_al_confirm.ind_ref = CPU_TO_LE((PNIO_UINT32) IndRef);

    Ret = SendReceiveSynch((char *)&Rq, sendLen, &expRLen, 30000);

    if(Ret != PNIO_OK)
        return Ret;

    if(LE_TO_CPU(Rq.opcode) != SCO_CTRL_ALARM_CONFIRM) {
        TRC_OUT01(GR_STATE, LV_ERR,
            "IController::alarm_resp received unknown opcode = 0x%x",
            LE_TO_CPU(Rq.opcode));

        return PNIO_ERR_INTERNAL;
    }

    if((Ret = ConvertAgentErrorCodeToApiErrorCode(LE_TO_CPU(Rq.agent_ret))) == PNIO_OK)
        Ret = LE_TO_CPU(Rq.resp_ret);

    return Ret;
}

/*===========================================================================
* FUNCTION : IController::ctrl_diag_req
*----------------------------------------------------------------------------
* PURPOSE  : Diagnosis and Info helper function
*----------------------------------------------------------------------------
* RETURNS  :
*----------------------------------------------------------------------------
* INPUTS   : - Diag
*==========================================================================*/
PNIO_UINT32 IController::ctrl_diag_req(PNIO_CTRL_DIAG * pDiagReq)
{
    light_T_SYNCH_COMMAND Rq;
    PNIO_UINT32 Ret, sendLen, expRLen, i;

    if(!pDiagReq) {
        return PNIO_ERR_PRM_POINTER;
    }

    if( pDiagReq->DiagService != PNIO_CTRL_DIAG_RESERVED &&
        pDiagReq->DiagService != PNIO_CTRL_DIAG_CONFIG_SUBMODULE_LIST &&
        pDiagReq->DiagService != PNIO_CTRL_DIAG_DEVICE_STATE &&
        pDiagReq->DiagService != PNIO_CTRL_DIAG_CONFIG_IOROUTER_PRESENT  &&
        pDiagReq->DiagService != PNIO_CTRL_DIAG_CONFIG_OUTPUT_SLICE_LIST &&
        pDiagReq->DiagService != PNIO_CTRL_DIAG_CONFIG_NAME_ADDR_INFO        ) {
        return PNIO_ERR_PRM_INVALIDARG;
    }

    sendLen = BASIC_LENGTH(Rq) + sizeof (Rq.u.ctrl_diag);
    expRLen = BASIC_LENGTH(Rq);

    memset(&Rq, 0, sizeof (Rq));
    Rq.blk_len = CPU_TO_LE(sendLen);
    Rq.opcode = (SYNCH_CHNL_OP)CPU_TO_LE(SCO_CTRL_DIAG);
    Rq.handle = CPU_TO_LE(NEW_ORDERID());
    Rq.u.ctrl_diag.DiagService = (PNIO_CTRL_DIAG_ENUM)CPU_TO_LE(pDiagReq->DiagService);
    Rq.u.ctrl_diag.ReqRef = CPU_TO_LE(pDiagReq->ReqRef);
    Rq.u.ctrl_diag.Reserved2 = CPU_TO_LE(pDiagReq->Reserved2);

    for(i = 0; i < (sizeof(pDiagReq->u.Reserved1)/sizeof(pDiagReq->u.Reserved1[0])) ; ++i)
        Rq.u.ctrl_diag.u.Reserved1[i] = CPU_TO_LE(pDiagReq->u.Reserved1[i]);

    Ret = SendReceiveSynch((char *)&Rq, sendLen, &expRLen);

    if(Ret != PNIO_OK)
        return Ret;

    if(LE_TO_CPU(Rq.opcode) != SCO_CTRL_DIAG) {
        TRC_OUT01(GR_STATE, LV_ERR,
            "IController::ctrl_diag_req received unknown opcode = 0x%x",
            LE_TO_CPU(Rq.opcode));

        return PNIO_ERR_INTERNAL;
    }

    if((Ret = ConvertAgentErrorCodeToApiErrorCode(LE_TO_CPU(Rq.agent_ret))) == PNIO_OK)
        Ret = LE_TO_CPU(Rq.resp_ret);

    return Ret;
}

/*===========================================================================
* FUNCTION : IController::ProcModeInd
*----------------------------------------------------------------------------
* PURPOSE  : callback function for mode channel
*----------------------------------------------------------------------------
* RETURNS  :
*----------------------------------------------------------------------------
* INPUTS   :
* OUTPUS   :
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
PNIO_UINT32 IController::ProcModeInd(CHANNEL *channel)
{
    PNIO_UINT32 Handle, Length;
    T_MODE_CHNL_DATA *pRq;

    TRC_OUT(GR_INIT, LV_FCTPUB1, "-> callback IController::ProcModeInd");

    Length = channel->user_pool_length_used;
    pRq = (T_MODE_CHNL_DATA *)channel->user_pool_ptr;
    if(Length > sizeof(T_MODE_CHNL_DATA)) {
        TRC_OUT02(GR_STATE, LV_ERR,
            "IController::ProcModeInd expected max %d, got unexpected length %d",
            sizeof(T_MODE_CHNL_DATA), Length);
        return PNIO_ERR_NO_RESOURCE;
    }

    Handle = ICommon::get_handle(this);

    switch (LE_TO_CPU(pRq->opcode)) {
    case MCO_CTRL_MODE_CHANGED:
        {
            m_Mode = (PNIO_MODE_TYPE)LE_TO_CPU(pRq->u.set_mode);

            if(m_pUserCbf_Mode) {
                TRC_OUT02(GR_STATE, LV_FCTCLBF,
                    "-> CBF_MODE_IND call user callback() ApplHandle=0x%x Mode=%s",
                    Handle, PnioModeToStr(m_Mode));
                PNIO_CBE_PRM cbf_prm;
                cbf_prm.Handle = Handle;
                cbf_prm.CbeType = PNIO_CBE_MODE_IND;
                cbf_prm.ModeInd.Mode = m_Mode;
                m_pUserCbf_Mode(&cbf_prm);
                TRC_OUT01(GR_STATE, LV_FCTCLBF,
                    "<- CBF_MODE_IND call user callback() ends ApplHandle=0x%x", Handle);
            } else {
                TRC_OUT02(GR_STATE, LV_FCTCLBF,
                    "CBF_MODE_IND decayed ApplHandle=0x%x Mode=%s",
                    Handle, PnioModeToStr(m_Mode));
            }
        }
        break;

    case MCO_CTRL_DEV_ACT:
        {
            if(m_pUserCbf_DevAct) {
                PNIO_ADDR Addr;
                TRC_OUT01(GR_STATE, LV_FCTCLBF,
                    "-> CBF_DEV_ACT_IND call user callback() ApplHandle=0x%x",
                    Handle);
                PNIO_CBE_PRM cbf_prm;
                cbf_prm.Handle = Handle;
                cbf_prm.CbeType = PNIO_CBE_DEV_ACT_CONF;

                Addr.AddrType = (PNIO_ADDR_TYPE)LE_TO_CPU(pRq->u.dev_state.Addr.AddrType);
                Addr.IODataType = (PNIO_IO_TYPE)LE_TO_CPU(pRq->u.dev_state.Addr.IODataType);
                Addr.u.Addr = LE_TO_CPU(pRq->u.dev_state.Addr.u.Addr);
                cbf_prm.DevActConf.pAddr = &Addr;

                cbf_prm.DevActConf.Mode = (PNIO_DEV_ACT_TYPE)LE_TO_CPU(pRq->u.dev_state.DevActMode);
                cbf_prm.DevActConf.Result = LE_TO_CPU(pRq->resp_ret);

                TRC_IF_ON_EXPR(GR_STATE, LV_FCTPUB2,
                    OSTREAM trcos;
                    trcos << showbase << hex;
                    trcos << "  pAddr=";
                    trcint_ShowAddr(trcos, cbf_prm.DevActConf.pAddr);
                    trcint_ShowActType(trcos, &cbf_prm.DevActConf.Mode, "Mode");
                    trcos << ", Result=" << cbf_prm.DevActConf.Result;
                    trcos << ends;
                    TRC_OUT_OBJECT(GR_STATE, LV_FCTPUB2, trcos);
                    );

                m_pUserCbf_DevAct(&cbf_prm);
                TRC_OUT01(GR_STATE, LV_FCTCLBF,
                    "<- CBF_DEV_ACT_IND call user callback() ends ApplHandle=0x%x", Handle);
            } else {
                TRC_OUT01(GR_STATE, LV_FCTCLBF,
                    "CBF_DEV_ACT_IND decayed ApplHandle=0x%x", Handle);
            }
        }
        break;

    case MCO_STOP_REQUEST:
        {
            if(m_pUserCbf_CpStopReq) {
                TRC_OUT01(GR_STATE, LV_FCTCLBF,
                    "-> CBE_CP_STOP_REQ call user callback() ApplHandle=0x%x", Handle);
                PNIO_CBE_PRM cbf_prm;
                cbf_prm.Handle = Handle;
                cbf_prm.CbeType = PNIO_CBE_CP_STOP_REQ;
                m_pUserCbf_CpStopReq(&cbf_prm);
                TRC_OUT01(GR_STATE, LV_FCTCLBF,
                    "<- CBE_CP_STOP_REQ call user callback() ends ApplHandle=0x%x", Handle);
            } else {
                TRC_OUT01(GR_STATE, LV_FCTCLBF,
                    "CBE_CP_STOP_REQ decayed ApplHandle=0x%x", Handle);
            }
        }
        break;

    case MCO_LED:
        {
            if(pRq->u.led.Start) {
                if(m_pUserCbf_StartLedFlash) {
                    TRC_OUT01(GR_STATE, LV_FCTCLBF,
                        "-> PNIO_CBF_START_LED_FLASH call user callback() ApplHandle=0x%x",
                        Handle);
                    PNIO_CBE_PRM cbf_prm;
                    cbf_prm.Handle = Handle;
                    cbf_prm.CbeType = PNIO_CBE_START_LED_FLASH;
                    cbf_prm.LedFlash.Frequency = LE_TO_CPU(pRq->u.led.Frequency);

                    TRC_IF_ON_EXPR(GR_STATE, LV_FCTPUB2,
                        OSTREAM trcos;
                        trcos << showbase << hex;
                        trcos << ", Frequency=" << cbf_prm.LedFlash.Frequency;
                        trcos << ends;
                        TRC_OUT_OBJECT(GR_STATE, LV_FCTPUB2, trcos);
                        );

                    m_pUserCbf_StartLedFlash(&cbf_prm);
                    TRC_OUT01(GR_STATE, LV_FCTCLBF,
                        "<- PNIO_CBF_START_LED_FLASH call user callback() ends ApplHandle=0x%x",
                        Handle);
                } else {
                    TRC_OUT01(GR_STATE, LV_FCTCLBF,
                        "PNIO_CBF_START_LED_FLASH decayed ApplHandle=0x%x", Handle);
                }
            } else {
                if(m_pUserCbf_StopLedFlash) {

                    TRC_OUT01(GR_STATE, LV_FCTCLBF,
                        "-> PNIO_CBF_STOP_LED_FLASH call user callback() ApplHandle=0x%x",
                        Handle);
                    PNIO_CBE_PRM cbf_prm;
                    cbf_prm.Handle = Handle;
                    cbf_prm.CbeType = PNIO_CBE_STOP_LED_FLASH;

                    TRC_IF_ON_EXPR(GR_STATE, LV_FCTPUB2,
                        OSTREAM trcos;
                        trcos << showbase << hex;
                        trcos << ends;
                        TRC_OUT_OBJECT(GR_STATE, LV_FCTPUB2, trcos);
                        );

                    m_pUserCbf_StopLedFlash(&cbf_prm);
                    TRC_OUT01(GR_STATE, LV_FCTCLBF,
                        "<- PNIO_CBF_STOP_LED_FLASH call user callback() ends ApplHandle=0x%x",
                        Handle);
                } else {
                    TRC_OUT01(GR_STATE, LV_FCTCLBF,
                        "PNIO_CBF_STOP_LED_FLASH decayed ApplHandle=0x%x", Handle);
                }
            }
        }
        break;

    default:
        TRC_OUT01(GR_STATE, LV_ERR,
            "IController::ProcModeInd received unknown opcode = 0x%x", LE_TO_CPU(pRq->opcode));
        DPR_ASSERT(0);
        break;
    }

    TRC_OUT(GR_MGT, LV_FCTPUB1, "<- IController::ProcModeInd done");

    return PNIO_OK;
}

/*===========================================================================
* FUNCTION : ProcDataRec
*----------------------------------------------------------------------------
* PURPOSE  : callback function for Datarec channel
*----------------------------------------------------------------------------
* RETURNS  :
*----------------------------------------------------------------------------
* INPUTS   :
* OUTPUS   :
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
PNIO_UINT32 IController::ProcDataRec(CHANNEL *channel)
{
    PNIO_UINT32 Length;
    T_DREC_CHNL_DATA *pRq;

    TRC_OUT(GR_CHNL, LV_FCTPUB1, "-> callback IController::ProcDataRec");

    Length = channel->user_pool_length_used;
    pRq = (T_DREC_CHNL_DATA *)channel->user_pool_ptr;

    switch(LE_TO_CPU(pRq->opcode)) {
    case DREC_REC:
        ProcDataRecordResponse(pRq);
        break;
    case DREC_CTRL_DIAG:
        ProcDiagResponse(pRq);
        break;
    default:
        TRC_OUT01(GR_STATE, LV_ERR,
            "IController::ProcDataRec received unknown opcode = 0x%x",
            LE_TO_CPU(pRq->opcode));
        DPR_ASSERT(0);
        break;
    }

    TRC_OUT(GR_CHNL, LV_FCTPUB1, "-> callback IController::ProcDataRec");

    return PNIO_OK;
}

void IController::ProcDataRecordResponse(const void *pRequest)
{
    const T_DREC_CHNL_DATA *pRq = (T_DREC_CHNL_DATA *)pRequest;
    PNIO_UINT32 Handle, FixPart;
    PNIO_ADDR Addr;

    TRC_OUT(GR_CHNL, LV_FCTPUB1, "-> IController::ProcDataRecordResponse");

    if(LE_TO_CPU(pRq->blk_len) > sizeof(T_DREC_CHNL_DATA)) {
        TRC_OUT02(GR_STATE, LV_ERR,
            "IController::ProcDataRecordResponse expected max %d, got unexpected length %d",
            sizeof(T_DREC_CHNL_DATA), LE_TO_CPU(pRq->blk_len));
        DPR_ASSERT(0);
    }

    Handle = ICommon::get_handle(this);

    const t_read_write_dr_resp *pDrRq = &(pRq->u.rw_dr_resp);

    FixPart = sizeof(T_DREC_CHNL_DATA) - PNIOI_DREC_MAX_SIZE;
    if((LE_TO_CPU(pDrRq->rqb.ActionType) == PNAGC_READ_DR) &&
            LE_TO_CPU(pDrRq->rqb.Length) != (LE_TO_CPU(pRq->blk_len) - FixPart)) {
        TRC_OUT02(GR_DS, LV_ERR,
            "IController::ProcDataRecordResponse: unexpected datarecord length = %ld (must be %ld)",
            LE_TO_CPU(pDrRq->rqb.Length), (LE_TO_CPU(pRq->blk_len) - FixPart));
        DPR_ASSERT(0);
    }

    // PE switch: Check for PE DataRecord and handle it separately
    //
    PNIO_UINT32  RecordIndex = LE_TO_CPU(pDrRq->rqb.RecordIndex);
#ifdef PROFI_ENERGY
    if ( (RecordIndex & PE_DR_INDEX_MASK) == PE_SAP_DR_INDEX) {
        // PROFIenergy (PE) Response Handling
        PNIO_REF     currReqRef  = LE_TO_CPU(pDrRq->rqb.UserRef);
        TRC_OUT01(GR_PE, LV_INFO, "ProcDataRecordResponse:-> handle_pe_dr_resp: ReqRef=%#x", currReqRef);
        m_pPeMgt->handle_pe_dr_resp(pDrRq);
    }
    else {
#endif /* PROFI_ENERGY */

    if(((LE_TO_CPU(pDrRq->rqb.ActionType) == PNAGC_READ_DR) && m_pUserCbf_DR_Read) ||
        ((LE_TO_CPU(pDrRq->rqb.ActionType) == PNAGC_WRITE_DR) && m_pUserCbf_DR_Write)) {
        PNIO_CBE_PRM cbf_prm;
        cbf_prm.Handle = Handle;

        switch (LE_TO_CPU(pDrRq->rqb.ActionType)) {
        case PNAGC_READ_DR:

            TRC_OUT01(GR_DS, LV_FCTCLBF,
                "-> IController::ProcDataRecordResponse call user m_pUserCbf_DR_Read() ApplHandle=0x%x",
                Handle);

            cbf_prm.CbeType = PNIO_CBE_REC_READ_CONF;

            Addr.AddrType = (PNIO_ADDR_TYPE)LE_TO_CPU(pDrRq->rqb.Addr.AddrType);
            Addr.IODataType = (PNIO_IO_TYPE)LE_TO_CPU(pDrRq->rqb.Addr.IODataType);
            Addr.u.Addr = LE_TO_CPU(pDrRq->rqb.Addr.u.Addr);
            cbf_prm.RecReadConf.pAddr = &Addr;

            cbf_prm.RecReadConf.RecordIndex = LE_TO_CPU(pDrRq->rqb.RecordIndex);
            cbf_prm.RecReadConf.ReqRef = LE_TO_CPU(pDrRq->rqb.UserRef);
            cbf_prm.RecReadConf.pBuffer = pDrRq->rqb.Buffer;
            cbf_prm.RecReadConf.Length = pDrRq->rqb.Length;

            cbf_prm.RecReadConf.Err.ErrCode = pDrRq->err.ErrCode;
            cbf_prm.RecReadConf.Err.ErrDecode = pDrRq->err.ErrDecode;
            cbf_prm.RecReadConf.Err.ErrCode1 = pDrRq->err.ErrCode1;
            cbf_prm.RecReadConf.Err.ErrCode2 = pDrRq->err.ErrCode2;
            cbf_prm.RecReadConf.Err.AddValue1 = LE_TO_CPU16(pDrRq->err.AddValue1);
            cbf_prm.RecReadConf.Err.AddValue2 = LE_TO_CPU16(pDrRq->err.AddValue2);

            TRC_IF_ON_EXPR(GR_DS, LV_FCTPUB2,
                OSTREAM trcos;
                OSTREAM trcos1;
                OSTREAM trcos2;
                trcos << showbase << hex;
                trcos << "    ApplHandle=" << cbf_prm.Handle;
                trcos << ", pAddr=";
                trcint_ShowAddr(trcos, cbf_prm.RecReadConf.pAddr);
                trcos << ", RecordIndex=" << cbf_prm.RecReadConf.RecordIndex;
                trcos << ", ReqRef=" << cbf_prm.RecReadConf.ReqRef;
                trcos << ends;
                TRC_OUT_OBJECT(GR_DS, LV_FCTPUB2, trcos);
                trcos1 << showbase << hex;
                trcos1 << "   ";
                trcint_ShowDerror(trcos1, &cbf_prm.RecReadConf.Err);
                trcos1 << ends;
                TRC_OUT_OBJECT(GR_DS, LV_FCTPUB2, trcos1);
                trcos2 << showbase << hex;
                trcos2 << "    Length=" << cbf_prm.RecReadConf.Length;
                trcos2 << ", pBuffer=";
                trcint_ShowData(trcos2, cbf_prm.RecReadConf.Length, cbf_prm.RecReadConf.pBuffer);
                trcos2 << ends;
                TRC_OUT_OBJECT(GR_DS, LV_FCTPUB2, trcos2);
                );

            m_pUserCbf_DR_Read(&cbf_prm);
            TRC_OUT01(GR_DS, LV_FCTCLBF,
                "<- IController::ProcDataRecordResponse call user m_pUserCbf_DR_Read() ends ApplHandle=0x%x",
                Handle);

            break;
        case PNAGC_WRITE_DR:

            TRC_OUT01(GR_DS, LV_FCTCLBF,
                "-> IController::ProcDataRecordResponse call user m_pUserCbf_DR_Write() ApplHandle=0x%x",
                Handle);

            cbf_prm.CbeType = PNIO_CBE_REC_WRITE_CONF;

            Addr.AddrType = (PNIO_ADDR_TYPE)LE_TO_CPU(pDrRq->rqb.Addr.AddrType);
            Addr.IODataType = (PNIO_IO_TYPE)LE_TO_CPU(pDrRq->rqb.Addr.IODataType);
            Addr.u.Addr = LE_TO_CPU(pDrRq->rqb.Addr.u.Addr);
            cbf_prm.RecWriteConf.pAddr = &Addr;

            cbf_prm.RecWriteConf.RecordIndex = LE_TO_CPU(pDrRq->rqb.RecordIndex);
            cbf_prm.RecWriteConf.ReqRef = LE_TO_CPU(pDrRq->rqb.UserRef);

            cbf_prm.RecWriteConf.Err.ErrCode = pDrRq->err.ErrCode;
            cbf_prm.RecWriteConf.Err.ErrDecode = pDrRq->err.ErrDecode;
            cbf_prm.RecWriteConf.Err.ErrCode1 = pDrRq->err.ErrCode1;
            cbf_prm.RecWriteConf.Err.ErrCode2 = pDrRq->err.ErrCode2;
            cbf_prm.RecWriteConf.Err.AddValue1 = LE_TO_CPU16(pDrRq->err.AddValue1);
            cbf_prm.RecWriteConf.Err.AddValue2 = LE_TO_CPU16(pDrRq->err.AddValue2);

            TRC_IF_ON_EXPR(GR_DS, LV_FCTPUB2,
                OSTREAM trcos;
                OSTREAM trcos1;
                trcos << showbase << hex;
                trcos << "    ApplHandle=" << cbf_prm.Handle;
                trcos << ", pAddr=";
                trcint_ShowAddr(trcos, cbf_prm.RecWriteConf.pAddr);
                trcos << ", RecordIndex=" << cbf_prm.RecWriteConf.RecordIndex;
                trcos << ", ReqRef=" << cbf_prm.RecWriteConf.ReqRef;
                trcos << ends;
                TRC_OUT_OBJECT(GR_DS, LV_FCTPUB2, trcos);
                trcos1 << "   ";
                trcint_ShowDerror(trcos1, &cbf_prm.RecWriteConf.Err);
                trcos1 << ends;
                TRC_OUT_OBJECT(GR_DS, LV_FCTPUB2, trcos1);
                );

            m_pUserCbf_DR_Write(&cbf_prm);
            TRC_OUT01(GR_DS, LV_FCTCLBF,
                "<- IController::ProcDataRecordResponse call user m_pUserCbf_DR_Write() ends ApplHandle=0x%x",
                Handle);

            break;
        default:
            TRC_OUT01(GR_DS, LV_ERR,
                "IController::ProcDataRecordResponse unknown dr_rw_type = 0x%x",
                LE_TO_CPU(pDrRq->rqb.ActionType));
            DPR_ASSERT(0);
            break;
        }
    } else {
        TRC_OUT01(GR_DS, LV_FCTCLBF,
            "IController::ProcDataRecordResponse event decayed ApplHandle=0x%x", Handle);
    }
#ifdef PROFI_ENERGY
    } // end else PE_DR_INDEX
#endif /* PROFI_ENERGY */

    TRC_OUT(GR_MGT, LV_FCTPUB1, "<- IController::ProcDataRecordResponse done");
}

/*-----------------------------------------------------------------------------
 * Name  : IController:: ProcDiagResponse
 * Descr : Callback handling function. Calls user callback if registered.
 *         Calling sequenz: procCannelRead -> ProcDataRec() -> ProcDiagResponse ->
 *                          user-callback
 */
void IController::ProcDiagResponse(const void *pRequest)
{
    const T_DREC_CHNL_DATA *pRq = (T_DREC_CHNL_DATA *)pRequest;
    PNIO_UINT32 Handle = ICommon::get_handle(this);

    TRC_OUT(GR_MGT, LV_FCTPUB1, "-> IController::ProcDiagResponse ");

    if(m_pUserCbf_CtrlGetDiag) {
        PNIO_CBE_PRM cbf_prm;
        cbf_prm.Handle = Handle;
        cbf_prm.CbeType = PNIO_CBE_CTRL_DIAG_CONF;

        PNIO_CTRL_DIAG locCtrlDiagRqb;
        PNIO_CTRL_DIAG_DEVICE locDeviceState;

        memset(&locCtrlDiagRqb, 0, sizeof(locCtrlDiagRqb));
        memset(&locDeviceState, 0, sizeof(locDeviceState));

        locCtrlDiagRqb.DiagService = (PNIO_CTRL_DIAG_ENUM)LE_TO_CPU(pRq->u.diag_resp.rqb.DiagService);
        locCtrlDiagRqb.ReqRef      = LE_TO_CPU(pRq->u.diag_resp.rqb.ReqRef);
        locCtrlDiagRqb.Reserved2   = LE_TO_CPU(pRq->u.diag_resp.rqb.Reserved2);

        cbf_prm.CtrlDiagConf.pDiagData = &locCtrlDiagRqb;
        cbf_prm.CtrlDiagConf.ErrorCode = LE_TO_CPU(pRq->u.diag_resp.ErrorCode);

        switch(locCtrlDiagRqb.DiagService) {
        case PNIO_CTRL_DIAG_CONFIG_SUBMODULE_LIST:
            for(int i=0; i<8; i++)
                locCtrlDiagRqb.u.Reserved1[i] = LE_TO_CPU(pRq->u.diag_resp.rqb.u.Reserved1[i]);

            create_config_submodules_list(
                LE_TO_CPU(pRq->u.diag_resp.u.CycleTime),
                &cbf_prm.CtrlDiagConf.DiagDataBufferLen,
                &cbf_prm.CtrlDiagConf.pDiagDataBuffer);
            break;
        case PNIO_CTRL_DIAG_DEVICE_STATE:
            locCtrlDiagRqb.u.Addr.AddrType = (PNIO_ADDR_TYPE)LE_TO_CPU(pRq->u.diag_resp.rqb.u.Addr.AddrType);
            locCtrlDiagRqb.u.Addr.IODataType = (PNIO_IO_TYPE)LE_TO_CPU(pRq->u.diag_resp.rqb.u.Addr.IODataType);
            locCtrlDiagRqb.u.Addr.u.Addr = LE_TO_CPU(pRq->u.diag_resp.rqb.u.Addr.u.Addr);

            locDeviceState.Mode = (PNIO_DEV_ACT_TYPE)LE_TO_CPU(pRq->u.diag_resp.u.DeviceState.Mode);
            locDeviceState.DiagState = LE_TO_CPU(pRq->u.diag_resp.u.DeviceState.DiagState);
            locDeviceState.Reason = LE_TO_CPU(pRq->u.diag_resp.u.DeviceState.Reason);

            cbf_prm.CtrlDiagConf.pDiagDataBuffer = (PNIO_UINT8*)&locDeviceState;
            cbf_prm.CtrlDiagConf.DiagDataBufferLen = sizeof(locDeviceState);
            break;
#ifdef IO_ROUTER

        case PNIO_CTRL_DIAG_CONFIG_IOROUTER_PRESENT:
                diag_config_iorouter_present(
                    &cbf_prm.CtrlDiagConf.DiagDataBufferLen,
                    &cbf_prm.CtrlDiagConf.pDiagDataBuffer);
            break;
        case PNIO_CTRL_DIAG_CONFIG_OUTPUT_SLICE_LIST:
                for(int i = 0; i < 8; i++) {
                    locCtrlDiagRqb.u.Reserved1[i] = LE_TO_CPU(pRq->u.diag_resp.rqb.u.Reserved1[i]);
                }
            create_config_output_slice_list(
                &cbf_prm.CtrlDiagConf.DiagDataBufferLen,
                &cbf_prm.CtrlDiagConf.pDiagDataBuffer);
            break;
#endif /* IO_ROUTER */

        /* PL: RQ: AP00856344: Extension of PNIO_ctrl_diag_req()  FW V2.5.2.0 and greater required
        */
        case PNIO_CTRL_DIAG_CONFIG_NAME_ADDR_INFO:

            PNIO_CTRL_DIAG_CONFIG_NAME_ADDR_INFO_DATA addrInfoData;

            memset(&addrInfoData, 0, sizeof(addrInfoData));

            strcpy((char*)&addrInfoData.name[0], (const char*)&pRq->u.diag_resp.u.CtrlNameAddrInfo.name[0]);  /* Gerätename */
            strcpy((char*)&addrInfoData.TypeOfStation[0], (const char*)&pRq->u.diag_resp.u.CtrlNameAddrInfo.TypeOfStation[0]); /* Gerätetyp */
            addrInfoData.ip_addr = LE_TO_CPU(pRq->u.diag_resp.u.CtrlNameAddrInfo.ip_addr);
            addrInfoData.ip_mask = LE_TO_CPU(pRq->u.diag_resp.u.CtrlNameAddrInfo.ip_mask);
            addrInfoData.default_router = LE_TO_CPU(pRq->u.diag_resp.u.CtrlNameAddrInfo.default_router);

            cbf_prm.CtrlDiagConf.pDiagDataBuffer = (PNIO_UINT8*)&addrInfoData;
            cbf_prm.CtrlDiagConf.DiagDataBufferLen = sizeof(addrInfoData);

            break;

        default:
            break;
        } /* end switch DiagService */

        TRC_OUT(GR_MGT, LV_FCTPUB1, " IController::ProcDiagResponse: ->user callback calling ... ");

        m_pUserCbf_CtrlGetDiag(&cbf_prm);

        TRC_OUT(GR_MGT, LV_FCTPUB1, " IController::ProcDiagResponse: <-user callback DONE");

        /* free allocated resources (only for some services required)
        */
        switch(locCtrlDiagRqb.DiagService) {
        case PNIO_CTRL_DIAG_CONFIG_SUBMODULE_LIST:
        case PNIO_CTRL_DIAG_CONFIG_IOROUTER_PRESENT:
        case PNIO_CTRL_DIAG_CONFIG_OUTPUT_SLICE_LIST:
            if(cbf_prm.CtrlDiagConf.pDiagDataBuffer) {
                delete [] cbf_prm.CtrlDiagConf.pDiagDataBuffer;
            }
            break;
        default:
            break;
        }

        TRC_OUT01(GR_DS, LV_FCTCLBF,
            "<- IController::ProcDataRecordResponse call user m_pUserCbf_CtrlGetDiag() ends ApplHandle=0x%x",
            Handle);
    } else {
        TRC_OUT01(GR_DS, LV_FCTCLBF,
            "IController::ProcDiagResponse event decayed ApplHandle=0x%x", Handle);
    }

    TRC_OUT(GR_MGT, LV_FCTPUB1, "<- IController::ProcDiagResponse done");
    return;
} /* end of ProcDiagResponse */

/*-----------------------------------------------------------------------------
 * Name  : create_config_submodules_list
 * Descr : helper function
 */
void IController::create_config_submodules_list(PNIO_UINT32 SendClock,
    PNIO_UINT32 *pBufferLen, PNIO_UINT8 **ppBuffer)
{
    KRAMIOTLB_Item **iotlb_arr = 0;
    PNIO_UINT32 iotlb_items_count = 0, i, count, newCount, iter;
    PNIO_CTRL_DIAG_CONFIG_SUBMODULE merk;

    PNIO_CTRL_DIAG_CONFIG_SUBMODULE *pConfigSubs;
    PNIO_UINT32 CycleTimeWithoutRedRat = SendClock * 125 / 4;

    *pBufferLen = 0;
    *ppBuffer = 0;

    /* get count of configuration items */
    KRAMIOTLB_GetHandleItems(m_hIodataUpdate, 0,
        &iotlb_items_count, NULL /* don't fill the array */, &m_HostIOTlbHdr);

    iotlb_arr = (KRAMIOTLB_Item **)malloc(sizeof(KRAMIOTLB_Item *) * iotlb_items_count);
    if(!iotlb_arr){
      TRC_OUT(GR_DS, LV_ERR,
          "IController::create_config_submodules_list can't allocate memory for iotlb_arr");
      return;
    }

    /* fill temporary array of pointer to configuration items */
    KRAMIOTLB_GetHandleItems(m_hIodataUpdate, 0,
        &iotlb_items_count, iotlb_arr, &m_HostIOTlbHdr);

    pConfigSubs = new PNIO_CTRL_DIAG_CONFIG_SUBMODULE[iotlb_items_count];

    if(!pConfigSubs) {
        if(iotlb_arr)
            free(iotlb_arr);
        TRC_OUT(GR_DS, LV_ERR,
            "IController::create_config_submodules_list can't allocate memory for pConfigSubs");
        return;
    }

    *ppBuffer = (PNIO_UINT8*)pConfigSubs;
    *pBufferLen = sizeof(PNIO_CTRL_DIAG_CONFIG_SUBMODULE) * iotlb_items_count;

    memset(*ppBuffer, 0, *pBufferLen);

    for(i = 0; i < iotlb_items_count; i++) {
        pConfigSubs[i].Address.AddrType = PNIO_ADDR_LOG;

        if(LE_TO_CPU(iotlb_arr[i]->io_out_type) & KRAMIOTLB_IO_OUT) {
            pConfigSubs[i].Address.IODataType = PNIO_IO_OUT;
            pConfigSubs[i].Reserved1[1] = PNIO_IO_OUT;
            pConfigSubs[i].Address.u.Addr = 0x7FFF & LE_TO_CPU(iotlb_arr[i]->log_addr);
        } else {
            pConfigSubs[i].Address.IODataType = PNIO_IO_IN;
            pConfigSubs[i].Reserved1[1] = PNIO_IO_IN;
            pConfigSubs[i].Address.u.Addr = LE_TO_CPU(iotlb_arr[i]->log_addr);
        }

        pConfigSubs[i].DataLength = LE_TO_CPU(iotlb_arr[i]->data_length);
        pConfigSubs[i].DataType = LE_TO_CPU(iotlb_arr[i]->io_sync_type) == KRAMIOTLB_IO_ASYNC ?
            PNIO_DATA_RT : PNIO_DATA_IRT;

        if(LE_TO_CPU(iotlb_arr[i]->io_out_type) & KRAMIOTLB_IO_DDEX)
            pConfigSubs[i].ComType = PNIO_COM_DIRECT_DATA_EX;
        else
            pConfigSubs[i].ComType = PNIO_COM_UNICAST;

        pConfigSubs[i].Api = LE_TO_CPU16(iotlb_arr[i]->api);

        pConfigSubs[i].ReductionFactor = LE_TO_CPU(iotlb_arr[i]->reduct_ratio);
        pConfigSubs[i].Phase = LE_TO_CPU(iotlb_arr[i]->phase);
        pConfigSubs[i].CycleTime = CycleTimeWithoutRedRat * LE_TO_CPU(iotlb_arr[i]->reduct_ratio);

        pConfigSubs[i].Reserved1[0] = PNIO_ADDR_GEO;
        pConfigSubs[i].StatNo  = LE_TO_CPU(iotlb_arr[i]->device_ar);
        pConfigSubs[i].Slot    = LE_TO_CPU(iotlb_arr[i]->slot_nr);
        pConfigSubs[i].Subslot = LE_TO_CPU(iotlb_arr[i]->subslot_nr);
    }

    // Do a Bubble Sort on the listed Items in the Array. Order by Device-Nr, Slotnr, Subslotnr.
	count = iotlb_items_count;
	do {
		newCount = 1;
		for (iter = 0; iter < count - 1; ++iter) {
			if (pConfigSubs[iter].StatNo > pConfigSubs[iter + 1].StatNo) {
				merk = pConfigSubs[iter];
				pConfigSubs[iter] = pConfigSubs[iter + 1];
				pConfigSubs[iter + 1] = merk;
				newCount = iter + 1;
			} else if (pConfigSubs[iter].StatNo	== pConfigSubs[iter + 1].StatNo) {
				if (pConfigSubs[iter].Slot > pConfigSubs[iter + 1].Slot) {
					merk = pConfigSubs[iter];
					pConfigSubs[iter] = pConfigSubs[iter + 1];
					pConfigSubs[iter + 1] = merk;
					newCount = iter + 1;
				} else if (pConfigSubs[iter].Slot == pConfigSubs[iter + 1].Slot) {
					if (pConfigSubs[iter].Subslot > pConfigSubs[iter + 1].Subslot) {
						merk = pConfigSubs[iter];
						pConfigSubs[iter] = pConfigSubs[iter + 1];
						pConfigSubs[iter + 1] = merk;
						newCount = iter + 1;
					}
				}
			}
		}
		count = newCount;
	} while (count > 1);

    if(iotlb_arr) {
        free(iotlb_arr);
    }
    return;
} /* end of create_config_submodules_list */

/*===========================================================================
* FUNCTION : ProcAlarm
*----------------------------------------------------------------------------
* PURPOSE  : callback function for Alarm channel, to process alarm msgs.
*----------------------------------------------------------------------------
* RETURNS  :
*----------------------------------------------------------------------------
* INPUTS   :
* OUTPUS   :
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
PNIO_UINT32 IController::ProcAlarm(CHANNEL *channel)
{
    PNIO_UINT32 Handle, Length, Ret;
    T_CTRL_RECEIVE_ALARM *pRq;

    TRC_OUT(GR_ALARM, LV_FCTPUB1, "-> callback IController::ProcAlarm");

    if(m_bClosePending){
      TRC_OUT(GR_ALARM, LV_FCTPUB1, "            IController::ProcAlarm processing broken, because Close-Sequence running");
      return PNIO_OK;
    }

    Length = channel->user_pool_length_used;
    pRq = (T_CTRL_RECEIVE_ALARM *)channel->user_pool_ptr;

    if(Length > sizeof(T_CTRL_RECEIVE_ALARM)) {
        TRC_OUT02(GR_ALARM, LV_ERR,
            "IController::ProcAlarm expected max %d, got unexpected length %d",
            sizeof(T_CTRL_RECEIVE_ALARM), Length);
        return PNIO_ERR_NO_RESOURCE;
    }

    switch((PNIO_ALARM_TYPE)LE_TO_CPU(pRq->rqb.AlarmType)) {
    case PNIO_ALARM_DEV_FAILURE:
        KRAMIOTLB_SetDeviceItemsAvailable(m_hIodataUpdate, LE_TO_CPU16(pRq->rqb.DeviceNum),
            KRAMIOTLB_SUBM_NOT_AVAILABLE, &m_HostIOTlbHdr);

        /* CBSA: Controller Byte Slice Access extension */
        if ( m_OpenExtPar & PNIO_CEP_SLICE_ACCESS ) {
            KRAMIOTLB_SetDeviceItemsAvailable(m_hIodataUpdate, LE_TO_CPU16(pRq->rqb.DeviceNum),
                KRAMIOTLB_SUBM_NOT_AVAILABLE, m_pHostKramIOTlbExt->getIOTlbHeaderPointer());
        }

        /* set pnio status for all input and output slots to BAD */
        IODU_ctrl_set_device_ioxs(m_pCpAdapter->pIODUItem, LE_TO_CPU16(pRq->rqb.DeviceNum),
            (KRAMIOTLB_IN_OUT_TYPE)(KRAMIOTLB_IO_IN | KRAMIOTLB_IO_OUT), PNIO_S_BAD, 0xFFFFFFFF);

        break;
    case PNIO_ALARM_DEV_RETURN:
        KRAMIOTLB_SetDeviceItemsAvailable(m_hIodataUpdate, LE_TO_CPU16(pRq->rqb.DeviceNum),
            KRAMIOTLB_SUBM_AVAILABLE, &m_HostIOTlbHdr);

        /* CBSA: Controller Byte Slice Access extension */
        if ( m_OpenExtPar & PNIO_CEP_SLICE_ACCESS ) {
            KRAMIOTLB_SetDeviceItemsAvailable(m_hIodataUpdate, LE_TO_CPU16(pRq->rqb.DeviceNum),
                KRAMIOTLB_SUBM_AVAILABLE, m_pHostKramIOTlbExt->getIOTlbHeaderPointer());
        }

        /* set pnio status for device pdevs and for input slots with zero data length to GOOD */
        IODU_ctrl_set_device_ioxs(m_pCpAdapter->pIODUItem, LE_TO_CPU16(pRq->rqb.DeviceNum),
            (KRAMIOTLB_IN_OUT_TYPE)(KRAMIOTLB_IO_IN | KRAMIOTLB_IO_PDEV), PNIO_S_GOOD, 1);

        break;
    case PNIO_ALARM_PULL:
    case PNIO_ALARM_PLUG:
    default:
        break;
    }

    Handle = ICommon::get_handle(this);

    if(m_pUserCbf_Alarm) {
        PNIO_ADDR Addr;
        PNIO_CTRL_ALARM_DATA AlarmData;

        TRC_OUT02(GR_ALARM, LV_FCTCLBF,
            "-> call user Alarm callback(0x%x) ApplHandle=0x%x", m_pUserCbf_Alarm, Handle);
        PNIO_CBE_PRM cbf_prm;
        cbf_prm.Handle = Handle;
        cbf_prm.CbeType = PNIO_CBE_ALARM_IND;

        cbf_prm.AlarmInd.IndRef = LE_TO_CPU(pRq->ind_ref);

        AlarmData.AlarmType = (PNIO_ALARM_TYPE)LE_TO_CPU(pRq->rqb.AlarmType);
        AlarmData.AlarmPriority = (PNIO_APRIO_TYPE)LE_TO_CPU(pRq->rqb.AlarmPriority);
        AlarmData.DeviceNum = LE_TO_CPU16(pRq->rqb.DeviceNum);
        AlarmData.SlotNum = LE_TO_CPU16(pRq->rqb.SlotNum);
        AlarmData.SubslotNum = LE_TO_CPU16(pRq->rqb.SubslotNum);
        AlarmData.CpuAlarmPriority = LE_TO_CPU16(pRq->rqb.CpuAlarmPriority);
        AlarmData.PnioCompatModtype = LE_TO_CPU(pRq->rqb.PnioCompatModtype);
        AlarmData.AlarmTinfo.CompatDevGeoaddr = LE_TO_CPU16(pRq->rqb.AlarmTinfo.CompatDevGeoaddr);
        AlarmData.AlarmTinfo.ProfileType = pRq->rqb.AlarmTinfo.ProfileType;
        AlarmData.AlarmTinfo.AinfoType = pRq->rqb.AlarmTinfo.AinfoType;
        AlarmData.AlarmTinfo.ControllerFlags = pRq->rqb.AlarmTinfo.ControllerFlags;
        AlarmData.AlarmTinfo.DeviceFlag = pRq->rqb.AlarmTinfo.DeviceFlag;
        AlarmData.AlarmTinfo.PnioVendorIdent = LE_TO_CPU16(pRq->rqb.AlarmTinfo.PnioVendorIdent);
        AlarmData.AlarmTinfo.PnioDevIdent = LE_TO_CPU16(pRq->rqb.AlarmTinfo.PnioDevIdent);
        AlarmData.AlarmTinfo.PnioDevInstance = LE_TO_CPU16(pRq->rqb.AlarmTinfo.PnioDevInstance);
        memcpy(AlarmData.DiagDs, pRq->rqb.DiagDs, sizeof(AlarmData.DiagDs));
        memcpy(AlarmData.PrAlarmInfo, pRq->rqb.PrAlarmInfo, sizeof(AlarmData.PrAlarmInfo));
        AlarmData.AlarmAinfo.BlockType = LE_TO_CPU16(pRq->rqb.AlarmAinfo.BlockType);
        AlarmData.AlarmAinfo.BlockVersion = LE_TO_CPU16(pRq->rqb.AlarmAinfo.BlockVersion);
        AlarmData.AlarmAinfo.Api = LE_TO_CPU(pRq->rqb.AlarmAinfo.Api);
        AlarmData.AlarmAinfo.AlarmSpecifier = LE_TO_CPU16(pRq->rqb.AlarmAinfo.AlarmSpecifier);
        AlarmData.AlarmAinfo.ModIdent = LE_TO_CPU(pRq->rqb.AlarmAinfo.ModIdent);
        AlarmData.AlarmAinfo.SubIdent = LE_TO_CPU(pRq->rqb.AlarmAinfo.SubIdent);
        AlarmData.AlarmAinfo.UserStrucIdent = LE_TO_CPU16(pRq->rqb.AlarmAinfo.UserStrucIdent);
        AlarmData.AlarmAinfo.UserAlarmDataLen = LE_TO_CPU16(pRq->rqb.AlarmAinfo.UserAlarmDataLen);


#ifdef PNIO_ALARM_OLD_STRUC
		memcpy(AlarmData.AlarmAinfo.UserAlarmData, pRq->rqb.AlarmAinfo.UserAlarmData,
            AlarmData.AlarmAinfo.UserAlarmDataLen);
#else
        memcpy(AlarmData.AlarmAinfo.UAData.UserAlarmData, pRq->rqb.AlarmAinfo.UAData.UserAlarmData,
            AlarmData.AlarmAinfo.UserAlarmDataLen);
#endif

		cbf_prm.AlarmInd.pAlarmData = &AlarmData;

        Addr.AddrType = (PNIO_ADDR_TYPE)LE_TO_CPU(pRq->addr.AddrType);
        Addr.IODataType = (PNIO_IO_TYPE)LE_TO_CPU(pRq->addr.IODataType);
        Addr.u.Addr = LE_TO_CPU(pRq->addr.u.Addr);
        cbf_prm.AlarmInd.pAddr = &Addr;

        TRC_IF_ON_EXPR(GR_ALARM, LV_FCTPUB2,
            OSTREAM trcos;
            trcos << showbase << hex;
            trcos << "    DeviceNum=" << cbf_prm.AlarmInd.pAlarmData->DeviceNum;
            trcint_ShowAlarmType(trcos, &cbf_prm.AlarmInd.pAlarmData->AlarmType, "AlarmType");
            trcos << ", pAddr=";
            trcint_ShowAddr(trcos, cbf_prm.DevActConf.pAddr);
            trcos << ", IndRef=" << cbf_prm.AlarmInd.IndRef;
            trcos << ends;
            TRC_OUT_OBJECT(GR_ALARM, LV_FCTPUB2, trcos);
            );

        m_pUserCbf_Alarm(&cbf_prm);
        TRC_OUT02(GR_ALARM, LV_FCTCLBF,
            "<- back from user Alarm callback(0x%x) ApplHandle=0x%x", m_pUserCbf_Alarm, Handle);
    } else {
        TRC_OUT01(GR_ALARM, LV_FCTCLBF, "Alarm event decayed ApplHandle=0x%x", Handle);
    }

    /* send alarm acknowledge */
    Ret = alarm_resp(LE_TO_CPU(pRq->ind_ref));
    TRC_OUT02(GR_ALARM, LV_FCTCLBF, "alarm_resp(0x%x): ret 0x%x", LE_TO_CPU(pRq->ind_ref), Ret);

    TRC_OUT(GR_ALARM, LV_FCTPUB1, "<- IContgroller::ProcAlarm done");

    return PNIO_OK;
}

/*===========================================================================
* FUNCTION : TestBlockSendReceive
*----------------------------------------------------------------------------
* PURPOSE  :
*----------------------------------------------------------------------------
* RETURNS  :
*----------------------------------------------------------------------------
* INPUTS   :
* OUTPUS   :
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
PNIO_UINT32 IController::TestBlockSendReceive(PNIO_UINT8 *Buffer, PNIO_UINT32 Length)
{
    PNIO_UINT32 Ret = 0;
    light_T_SYNCH_COMMAND Rq;

    if((NULL == Buffer) && Length)
        return PNIO_ERR_PRM_BUF;

    if(Length > sizeof(Rq.u.test_ping.Data))
        return PNIO_ERR_PRM_LEN;

    PNIO_UINT32 sendLen = BASIC_LENGTH(Rq) + sizeof (Rq.u.test_ping)
                          - sizeof(Rq.u.test_ping.Data) + Length;
    PNIO_UINT32 expRLen = BASIC_LENGTH(Rq) + sizeof (Rq.u.test_ping);

    memset(&Rq, 0, sizeof(Rq));
    Rq.blk_len = CPU_TO_LE(sendLen);
    Rq.opcode = (SYNCH_CHNL_OP)CPU_TO_LE(SCO_TEST_PING);
    Rq.handle = CPU_TO_LE(NEW_ORDERID());
    memcpy(Rq.u.test_ping.Data, Buffer, Length);
    Rq.u.test_ping.DataLen = CPU_TO_LE(Length);

    Ret = SendReceiveSynch((char *)&Rq, sendLen, &expRLen);

    if(Ret != PNIO_OK)
        return Ret;

    if(LE_TO_CPU(Rq.opcode) != SCO_TEST_PING) {
        TRC_OUT01(GR_STATE, LV_ERR,
            "IController::TestBlockSendReceive received unknown opcode = 0x%x",
            LE_TO_CPU(Rq.opcode));
        return PNIO_ERR_INTERNAL;
    }

    if((Ret = ConvertAgentErrorCodeToApiErrorCode(LE_TO_CPU(Rq.agent_ret))) == PNIO_OK)
        Ret = LE_TO_CPU(Rq.resp_ret);

    if(PNIO_OK == Ret)
        memcpy(Buffer, Rq.u.test_ping.Data, MIN(Length, LE_TO_CPU(Rq.u.test_ping.DataLen)));

    return Ret;
}

/*===========================================================================
* FUNCTION : TestIO
*----------------------------------------------------------------------------
* PURPOSE  :
*----------------------------------------------------------------------------
* RETURNS  :
*----------------------------------------------------------------------------
* INPUTS   :
* OUTPUS   :
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
PNIO_UINT32 IController::TestIO(PNIO_UINT32 area_idx, PNIO_UINT32 operation, PNIO_UINT32 Length, PNIO_UINT8 *Buffer)
{
  PNIO_UINT8 *pArea=0;

  switch(area_idx)
  {
    case 0: /* KRAM over PCI */ pArea = (PNIO_UINT8 *)m_pCpAdapter->pErtecIOTotal; break;
    case 1: /* DMA Pool on HOST */ pArea = (PNIO_UINT8 *)m_pCpAdapter->pIRTDMAImage; break;
    default: /* local memory on HOST */ pArea = (PNIO_UINT8 *)m_pOutCache; break;
  }

  if(operation & 1) // copy output data from user buffer to memory
    memcpy(pArea, Buffer, Length);
  if(operation & 2) // copy input data from memory to user buffer
    memcpy(Buffer + Length, pArea + Length, Length);

  return PNIO_OK;
}

/*===========================================================================
* FUNCTION : RegisterStartOp
*----------------------------------------------------------------------------
* PURPOSE  :
*----------------------------------------------------------------------------
* RETURNS  :
*----------------------------------------------------------------------------
* INPUTS   :
* OUTPUS   :
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
PNIO_UINT32 IController::RegisterStartOp(void)
{
    PNIO_UINT32 Ret = 0;
    light_T_SYNCH_COMMAND Rq;

    PNIO_UINT32 sendLen = BASIC_LENGTH(Rq);
    PNIO_UINT32 expRLen = BASIC_LENGTH(Rq);

    memset(&Rq, 0, sizeof(Rq));
    Rq.blk_len = CPU_TO_LE(sendLen);
    Rq.opcode = (SYNCH_CHNL_OP)CPU_TO_LE(SCO_REGISTER_START_OP_CBF);
    Rq.handle = CPU_TO_LE(NEW_ORDERID());

    Ret = SendReceiveSynch((char *)&Rq, sendLen, &expRLen);

    if(Ret != PNIO_OK)
        return Ret;

    if(LE_TO_CPU(Rq.opcode) != SCO_REGISTER_START_OP_CBF) {
        TRC_OUT01(GR_STATE, LV_ERR,
            "IController::RegisterStartOp received unknown opcode = 0x%x",
            LE_TO_CPU(Rq.opcode));
        return PNIO_ERR_INTERNAL;
    }

    if((Ret = ConvertAgentErrorCodeToApiErrorCode(LE_TO_CPU(Rq.agent_ret))) == PNIO_OK)
        Ret = LE_TO_CPU(Rq.resp_ret);

    return Ret;
}

#ifdef IO_ROUTER

/*-----------------------------------------------------------------------------
 * Name  : create_config_output_slice_list
 * Descr :
 * Param :
 *  [ IN]:
 *  [OUT]:
 * Return: -
 */
void IController::create_config_output_slice_list(PNIO_UINT32 *pBufferLen,
    PNIO_UINT8 **ppBuffer)
{
    KRAMIOTLB_Item **iotlb_arr;
    PNIO_UINT32 iotlb_items_count = 0, i, la;
    PNIO_CTRL_DIAG_CONFIG_OUTPUT_SLICE *pDiagOutSlice = NULL;
    int numCfgSlices = 0;
    int numApplOnlyOutputModules = 0;
    int numDiagSlices = 0;

    /* get count of configuration items */
    KRAMIOTLB_GetHandleItems(m_hIodataUpdate, 1,
        &iotlb_items_count, NULL /* don't fill the array */, &m_HostIOTlbHdr);
    iotlb_arr = (KRAMIOTLB_Item **)malloc(sizeof(KRAMIOTLB_Item *) * iotlb_items_count);
    DPR_ASSERT(iotlb_arr);

    /* fill temporary array of pointers to configuration items */
    KRAMIOTLB_GetHandleItems(m_hIodataUpdate, 1,
        &iotlb_items_count, iotlb_arr, &m_HostIOTlbHdr);

    *pBufferLen = 0;     // preset output values
    *ppBuffer   = NULL;

    // calculate total num of slices from transfer sdb
    PNIO_UINT32 maxAddr = m_Ioc.getLogAddrMax();
    for(la = 0; la <= maxAddr; ++la) {                  // for all log_addr
        IOR_CONCENTRATOR_ENTRY *pSubMod = m_Ioc.getIocHostModule(la);
        if(pSubMod) {
            numCfgSlices += m_Ioc.getNumOfSlices(pSubMod);
        }
    }
    // calculate number of writeable modules without transfer bits
    for(i = 0; i < iotlb_items_count; ++i) {
        if(LE_TO_CPU(iotlb_arr[i]->io_out_type) & KRAMIOTLB_IO_OUT) {
            PNIO_UINT32 logAddr = 0x7fff & LE_TO_CPU(iotlb_arr[i]->log_addr);
            IOR_CONCENTRATOR_ENTRY *pSubMod = m_Ioc.getIocModule(logAddr);
            if(!pSubMod) {
                numApplOnlyOutputModules += 1;
            }
        }
    }

    // calculate total number of slices that are reported to application
    numDiagSlices = numCfgSlices + numApplOnlyOutputModules;

    if(numDiagSlices == 0) {
        if(iotlb_arr)
            free(iotlb_arr);
        TRC_OUT(GR_IOR, LV_INFO, "IOC: diag slices == ZERO");
        return;
    }
    pDiagOutSlice = new PNIO_CTRL_DIAG_CONFIG_OUTPUT_SLICE[numDiagSlices];

    if(!pDiagOutSlice) {
        if(iotlb_arr)
            free(iotlb_arr);
        TRC_OUT(GR_IOR, LV_ERR, "IOC: diag: can't allocate memory");
        return;
    }

    *ppBuffer = (PNIO_UINT8*)pDiagOutSlice;
    *pBufferLen = sizeof(PNIO_CTRL_DIAG_CONFIG_OUTPUT_SLICE) * numDiagSlices;

    memset(*ppBuffer, 0, *pBufferLen);

    PNIO_CTRL_DIAG_CONFIG_OUTPUT_SLICE *pCurrOutSlice = pDiagOutSlice;  // set to begin

    // add slices found in transfer sdb
    for(la = 0; la <= maxAddr; ++la) {                      // check all log_addr
        IOR_CONCENTRATOR_ENTRY *pSubMod = m_Ioc.getIocHostModule(la);
        if(pSubMod) {
            int numSlcs = 0, j;
            if((numSlcs = m_Ioc.getNumOfSlices(pSubMod)) > 0) {
                for(j = 0; j < numSlcs; ++j) {
                    IOR_OUTPUT_SLICE *pSlc = m_Ioc.getSliceAtIdx(pSubMod, j);  // io concentrator table slice

                    pCurrOutSlice->Address.AddrType   = PNIO_ADDR_LOG; // always logical addr
                    pCurrOutSlice->Address.IODataType = PNIO_IO_OUT;   // always output
                    pCurrOutSlice->Address.u.Addr     = la;            // current log_addr
                    pCurrOutSlice->BitOffset = m_Ioc.getSliceBitOffset(pSlc);
                    pCurrOutSlice->BitLength = m_Ioc.getSliceBitLength(pSlc);
                    pCurrOutSlice++;
                }
            }
        }
    }

    // add writeable modules without transfer bits: one slice, slice length is module length
    for(i = 0; i < iotlb_items_count; ++i) {
        if(LE_TO_CPU(iotlb_arr[i]->io_out_type) & KRAMIOTLB_IO_OUT) {
            PNIO_UINT32 logAddr = 0x7fff & LE_TO_CPU(iotlb_arr[i]->log_addr);
            IOR_CONCENTRATOR_ENTRY *pSubMod = m_Ioc.getIocModule(logAddr);
            if(!pSubMod) {
                // Output Module not shared and not pure -> slice spans entire module
                pCurrOutSlice->Address.AddrType   = PNIO_ADDR_LOG; // always logical addr
                pCurrOutSlice->Address.IODataType = PNIO_IO_OUT;   // always output
                pCurrOutSlice->Address.u.Addr     = logAddr;       // current log_addr
                pCurrOutSlice->BitOffset = 0;
                pCurrOutSlice->BitLength = 8*LE_TO_CPU(iotlb_arr[i]->data_length);
                pCurrOutSlice++;
            }
        }
    }

    if(iotlb_arr)
        free(iotlb_arr);
} // end of create_config_output_slice_list

/*-----------------------------------------------------------------------------
 * Name  : diag_config_iorouter_present    (pre callback handling)
 * Descr : Called by IController::ProcDiagResponse before the registered
 *         user callback function is called: m_pUserCbf_CtrlGetDiag
 * Param :
 *  [OUT]: pBufferLen: This is the response paramenter, it has boolean meaning.
 *            == 0   : NO iorouter present (configured)
 *            != 0   : (true) iorouter is present (configured)
 *  [OUT]: pBuffer: This param has no meaning, we allocate it only to avoid
 *                  zero pointer access by the user (by handling the callback)
 * Return: -
 */
void IController::diag_config_iorouter_present(PNIO_UINT32 *pBufferLen,
    PNIO_UINT8 **ppBuffer)
{
    PNIO_UINT8  *pDummyBuffer;
    const int dummyBuffLen = 4;
    *pBufferLen = 0;  // preset to false == not present
    *ppBuffer = 0;

    pDummyBuffer = new PNIO_UINT8[dummyBuffLen];    // small dummy buffer
    if(!pDummyBuffer) {
        TRC_OUT(GR_IOR, LV_ERR,
            "IController::diag_config_iorouter_present can't allocate memory");
        return;
    }

    *ppBuffer = pDummyBuffer;
    memset(pDummyBuffer, 0, dummyBuffLen);

    if(m_Ioc.getNumOfIocModules()) {
        // there are some submoduls configured for ioconcentrator
        *pBufferLen = 1;       // set to true == present
        memset(pDummyBuffer, 0xff, dummyBuffLen);
    }
    return;
} // end of diag_config_iorouter_present

/*===========================================================================
* FUNCTION : transfer_wd_cbf_wrapper
*----------------------------------------------------------------------------
* PURPOSE  : Static wrapper function for transfer watchdog callback
*----------------------------------------------------------------------------
* RETURNS  :PNIO_UINT32
*----------------------------------------------------------------------------
* INPUTS   :
* OUTPUS   :
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
void IController::transfer_wd_cbf_wrapper(PNIO_CP_CBE_PRM * prm, IController * pThis)
{
    pThis->transfer_wd_cbf(prm);
}

/*===========================================================================
* FUNCTION : transfer_wd_cbf
*----------------------------------------------------------------------------
* PURPOSE  : Member wrapper function for transfer watchdog callback
*----------------------------------------------------------------------------
* RETURNS  :PNIO_UINT32
*----------------------------------------------------------------------------
* INPUTS   :
* OUTPUS   :
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
void IController::transfer_wd_cbf(PNIO_CP_CBE_PRM * prm)
{
    m_TransferWD.Clock(prm);
}

#endif /* IO_ROUTER */

/* CBSA: Controller Byte Slice Access extensions. PL: 11.01.2010
*/

/*-----------------------------------------------------------------------------
 * Name  : HostKramIOTableExt:   Ctor
 *
 * Descr : HostKramIOTableExt is a wrapper class of the "extended KRAM IO table".
 *         (array of additional KRAMIOTLB_Items).
 *         but it is not necessary because for the access to the table only methods
 *         of this class are used.
 * Param : [ IN]: pHostIOTlbHeader: Pointer to the header of the original HostIOTlb.
 *
 */
HostKramIOTableExt::HostKramIOTableExt(const KRAMIOTLB_Header *pHostIOTlbHeader)
{

    PNIO_UINT32 numItemsRequired = 0;    // required num slots in the array

    numItemsRequired = calculateNumItems(pHostIOTlbHeader);


    // create a table of the required size
    kramItem = new KRAMIOTLB_Item [numItemsRequired];
    numItems = numItemsRequired;
    lastValidIndex = INVALID_INDEX; // table is empty

    // set kramItem array content.
    initIotlbExtContent(pHostIOTlbHeader);

    // init 'KRAMIOTLB_Header' of IOTableExt. i.e. kramItem array header.
    // Pointer to KRAMIOTLB_Header is required by the KRAMIOTLB_xxx functions.
    initIotlbExtHdr();

}  /* end of Ctor */

/*-----------------------------------------------------------------------------
 * Name  : ~HostKramIOTableExt:   Dtor
 */
HostKramIOTableExt::~HostKramIOTableExt()
{
    delete [] kramItem;
}

/*-----------------------------------------------------------------------------
 * Name  : HostKramIOTableExt::operator []
 * Descr : gets array item ref.
 */
KRAMIOTLB_Item& HostKramIOTableExt::operator [] (PNIO_UINT32 index)
{
    if ( index > (numItems - 1) ) {
        // error
        // TBD! error handling / trace
        return kramItem[numItems - 1];
    }
    return kramItem[index];

} /* end of operator[] */

/*-----------------------------------------------------------------------------
 * Name  : HostKramIOTableExt::updateAddressHash()
 * Descr : Updates the 'PNIO logical address list' named contrHashTlb array.
 *         contrHashTlb array is a global array of pointers to KRAMIOTLB_Items.
 *         Originally created contrHashTlb array contains only entries for modul
 *         starting addresses. E.g. for a 4 byte long modul at logical addr. 20
 *         only the array field 20 is set. contrHashTlb[20] = *KRAMIOTLB_Items
 *         contrHashTlb[21-23] are zero == NULL.
 *         This function writes pointers into the contrHashTlb array for all log.
 *         addresses. == Pointers to KRAMIOTLB_Items saved in this class.
 *         See also KRAMIOTLB_CreateContrHash()
 */
void HostKramIOTableExt::updateAddressHash (void)
{

    KRAMIOTLB_Item **hashTlb;
    PNIO_UINT32      hashTlbSize = 0xffff;  // max. supported adresses

    // contrHashTlb glob var. in ..\pniolib\kramiotlb\src\krtlb_host.c
    // must be already set (table created) before calling this function.
    if (!contrHashTlb) {
        TRC_OUT(GR_INIT, LV_ERR, "HostKramIOTableExt::updateAddressHash: no hash. FATAL ERROR => no action");
        return;
    }

    hashTlb = contrHashTlb;

    if ( lastValidIndex == INVALID_INDEX ) {
        // no items - no action
        TRC_OUT(GR_INIT, LV_ERR, "HostKramIOTableExt::updateAddressHash: zero table length WARNING");
        return;
    }
    for (PNIO_UINT32 i=0; i <= lastValidIndex; i++ ) {
        hashTlb[SWAP_D(kramItem[i].log_addr)] = &kramItem[i];
    }

    return;
} /* end of updateAddressHash */


/*-----------------------------------------------------------------------------
 * Name  : HostKramIOTableExt::getIOTlbHeaderPointer
 * Descr :
 * Return: HeaderPointer
 */
 KRAMIOTLB_Header* HostKramIOTableExt::getIOTlbHeaderPointer (void)
{
    return &iotlbExtHdr;

} /* end of getIOTlbHeaderPointer */


/*-----------------------------------------------------------------------------
 * Name  : HostKramIOTableExt::initIotlbExtHdr
 * Descr : Sets kramItem array header values. see KRAMIOTLB_init()
 */
void HostKramIOTableExt::initIotlbExtHdr (void)
{

    KRAMIOTLB_Header *pHdr = &iotlbExtHdr;
    pHdr->pMaxItemPos = &lastValidIndex;       /* pMaxItemPos points to a uint which contains
                                                  the max. currently filled element index    */
    pHdr->pFirstItem  = &kramItem[0];          /* position of first item */
    pHdr->pLastItem   = &kramItem[numItems];   /* last possible possition */

    return;
} /* end of initIotlbExtHdr */


/*-----------------------------------------------------------------------------
 * Name  : HostKramIOTableExt::initIotlbExtContent
 * Descr :
 * Param : [ IN]: const KRAMIOTLB_Header *pHostIOTlbHeader: Pointer to the IOTlbHeader.
 *         This is the Header of the original HostIOTlb (for entire modul access).
 *
 */
void HostKramIOTableExt::initIotlbExtContent (const KRAMIOTLB_Header *pHostIOTlbHeader)
{
    KRAMIOTLB_Item *pItem, *pCurMaxItem;
    PNIO_UINT32    extItemsCount = 0;

    if ( numItems == 0 ) {
            // no items - table zero slots, zero length - no action
            return;
    }

    pCurMaxItem = &pHostIOTlbHeader->pFirstItem[SWAP_D(*pHostIOTlbHeader->pMaxItemPos)];
    pItem = pHostIOTlbHeader->pFirstItem;

    int modlen = 0;
    for (pItem; pItem <= pCurMaxItem; pItem++) {
        PNIO_UINT32 modlen = SWAP_D(pItem->data_length);  // length in bytes
        if ( modlen > 1 ) {
            // handle this modul item for all possible log addresses
            PNIO_UINT32 begin_la = SWAP_D(pItem->log_addr);
            PNIO_UINT32 i, la;
            for ( i=1, la = begin_la + 1; la < begin_la + modlen; la++, i++ ) {
                extItemsCount++;
                KRAMIOTLB_Item  newItem  = *pItem;    // copy struct content: newItem <-- original-Base-Item

                PNIO_UINT32 h_data_offset = SWAP_D(pItem->data_offset);
                PNIO_UINT32 h_data_length = SWAP_D(pItem->data_length);
                h_data_offset += i;
                h_data_length -= i;
                // modify item values
                newItem.log_addr    = SWAP_D(la);
                newItem.data_offset = SWAP_D(h_data_offset);
                newItem.data_length = SWAP_D(h_data_length);

                // write new item into the iotlbext
                addItem(newItem);
            }
        }
    }

    return;
} /* end of initIotlbExtContent */


/*-----------------------------------------------------------------------------
 * Name  : HostKramIOTableExt::calculateNumItems
 * Descr : Calculates a number of items required for slice access. I.e. length
 *         of the HostKramIOTableExt kramItem array. Originaly (without slice access)
 *         one module requires one KRAMIOTLB_Item which describes the module.
 *         In particular the moduls logical start address and the length (num bytes).
 *         For modul slice access we need one KRAMIOTLB_Item for each logical
 *         address which is valid for this module. I.e.: 4 byte long modul has
 *         4 valid logical addresses e.g.20, 21, 22, 23 => we need 3 additional
 *         items. Generally speaking: Required additional items = moduleLen - 1
 *
 *
 * Param : [ IN]: const KRAMIOTLB_Header *pHostIOTlbHeader: Pointer to the IOTlbHeader.
 *         This is the Header of the original HostIOTlb (for entire modul access).
 *
 * Return: PNIO_UINT32  Number of items or zero if no additional items required.
 */
PNIO_UINT32 HostKramIOTableExt::calculateNumItems (const KRAMIOTLB_Header *pHostIOTlbHeader)
{
    PNIO_UINT32 numExtItems = 0;
    KRAMIOTLB_Item *pItem, *pCurMaxItem;

    pCurMaxItem = &pHostIOTlbHeader->pFirstItem[SWAP_D(*pHostIOTlbHeader->pMaxItemPos)];
    pItem = pHostIOTlbHeader->pFirstItem;

    for (pItem; pItem <= pCurMaxItem; pItem++) {
        PNIO_UINT32 dtlen = SWAP_D(pItem->data_length);
        if ( dtlen == 0 ) {
            continue;  // module without io data (e.h. head module) ignore it
        }
        if (dtlen > 0x8000 ) {
            TRC_OUT02(GR_INIT, LV_ERR, "HostKramIOTableExt::calculateNumItems: invalid len=%u addr=%u ",dtlen, SWAP_D(pItem->log_addr) );
            continue;
        }
        numExtItems = numExtItems + (dtlen - 1);
    }

    TRC_OUT01(GR_INIT, LV_FCTPUB1, "HostKramIOTableExt::calculateNumItems: numExtItems=%u ",numExtItems);
    return numExtItems;

} /* end of calculateNumItems */

/*-----------------------------------------------------------------------------
 * Name  : addItem
 * Descr : Writes given item into the array at the next free index (position).
 * Param : [ IN]: Item to be added to the table at the next free index
 */
void HostKramIOTableExt::addItem ( KRAMIOTLB_Item &newItem )
{

    if ( numItems == 0 ) {
            // no items - table zero slots, zero length - no space - no action
            TRC_OUT(GR_INIT, LV_ERR, "HostKramIOTableExt::addItem: zero table length WARNING");
            return;
    }

    if ( (lastValidIndex != INVALID_INDEX) && (lastValidIndex >= (numItems -1)) ) {
        // no more space, programm error, no action
        TRC_OUT(GR_INIT, LV_ERR, "HostKramIOTableExt::addItem: no space FATAL ERROR");
        return;
    }

    if (lastValidIndex == INVALID_INDEX) {
        lastValidIndex = 0;              // first element
    }
    else {
        lastValidIndex++;
    }

    kramItem[lastValidIndex] = newItem;  // struct copy

    TRC_OUT02(GR_INIT, LV_FCTPUB1, "HostKramIOTableExt::addItem: NumItems max=%u current=%u ", numItems, lastValidIndex +1);
    return;
} /* end of addItem */



/*===========================================================================
* FUNCTION : ConvertAgentErrorCodeToApiErrorCode
*----------------------------------------------------------------------------
* PURPOSE  : Converting error codes
*----------------------------------------------------------------------------
* RETURNS  :PNIO_UINT32
*----------------------------------------------------------------------------
* INPUTS   :
* OUTPUS   :
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
PNIO_UINT32 ConvertAgentErrorCodeToApiErrorCode(PNIO_UINT32 InErr)
{
    PNIO_UINT32 RteErr;

    switch(InErr) {
    case PNIO_AGETPC_RET_OK:
        RteErr = PNIO_OK;
        break;
    case PNIO_AGETPC_RET_NO_PNIO_CONFIG:
        RteErr = PNIO_ERR_NO_CONFIG;
        break;
    case PNIO_AGETPC_RET_ALREADY_OPENED:
        RteErr = PNIO_ERR_MAX_REACHED;
        break;
    case PNIO_AGETPC_RET_OPEN_NOT_ALLOWED:
        RteErr = PNIO_ERR_CONFIG_IN_UPDATE;
        break;
    case PNIO_AGETPC_RET_CONFIG_INCONS:
        RteErr = PNIO_ERR_INVALID_CONFIG;
        break;
    case PNIO_AGETPC_RET_NOT_OPENED:
        RteErr = PNIO_ERR_WRONG_HND;
        break;
    case PNIO_AGETPC_RET_REBOOT_AFTER_EXCEPTION:
        RteErr = PNIO_ERR_AFTER_EXCEPTION;
        break;
    case PNIO_AGETPC_RET_ERROR_PARAM:
        RteErr = PNIO_ERR_WRONG_RQB_LEN;
        break;
    default:
        RteErr = PNIO_ERR_INTERNAL;
        break;
    }

    if(InErr != PNIO_AGETPC_RET_OK)
        TRC_OUT02(GR_MGT, LV_ERR,
            "ConvertAgentErrorCodeToApiErrorCode(0x%x) ret=0x%x", InErr, RteErr);
    return RteErr;
}
