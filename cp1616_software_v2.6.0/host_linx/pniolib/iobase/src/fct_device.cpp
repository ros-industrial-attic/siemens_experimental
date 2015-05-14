/****************************************************************************
*  Copyright (C) SIEMENS CORP., 2013 All rights reserved.
*****************************************************************************
* FILE NAME    : fct_device.cpp
* ---------------------------------------------------------------------------
* DESCRIPTION  : Device class implementations
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
#include "pniodag_rqb.h"
#include "iodataupdate.h"
#include "fct_device.h"

#define BASIC_LENGTH(x)    (sizeof((x).blk_len) +\
                            sizeof((x).opcode) +\
                            sizeof((x).handle) +\
                            sizeof((x).agent_ret) +\
                            sizeof((x).resp_ret))

#define CBF_BASIC_LENGTH(x) (sizeof((x).blk_len) +\
                             sizeof((x).opcode) +\
                             sizeof((x).handle))

/*===========================================================================
* FUNCTION : IDevice::IDevice
*----------------------------------------------------------------------------
* PURPOSE  : Constructor
*----------------------------------------------------------------------------
* RETURNS  :
*----------------------------------------------------------------------------
* INPUTS   :
* OUTPUS   :
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
IDevice::IDevice(void):
    ICommon(),
    m_OpenExtPar(0)
{
    m_bController = false;
    memset(&m_Cbfs, 0, sizeof(m_Cbfs));
}


bool IDevice::ar_has_slot_0_pluged = false;
bool IDevice::ctrl_wants_slot_0 = false;
bool IDevice::do_pdev_ioxs_check = false;
PNIO_UINT32 IDevice::slot0_modid_pluged = 0;
PNIO_UINT32 IDevice::slot0_modid_ctrl = 0;
PNIO_UINT32 IDevice::slot1_modid_pluged = 0;
PNIO_UINT32 IDevice::slot1_modid_ctrl = 0;

/*===========================================================================
* FUNCTION : CBF_GET_INITIAL_IOXS
*----------------------------------------------------------------------------
* PURPOSE  : Callback for checking initial IOXS Setup for DAP
*----------------------------------------------------------------------------
* RETURNS  : GOOD/BAD 
*----------------------------------------------------------------------------
* INPUTS   :
* OUTPUS   :
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
PNIO_IOXS    IDevice::CBF_GET_INITIAL_IOXS  /* get initial IOXS status */
       (PNIO_UINT32          DevHndl,       /* Handle for Multidevice */
        PNIO_DEV_ADDR      * pAddr)         /* geographical address */
{

  if (!do_pdev_ioxs_check) return PNIO_S_GOOD;
  //if (ctrl_wants_slot_0 == ar_has_slot_0_pluged) return PNIO_S_GOOD;
  if((slot0_modid_pluged != 0 && (slot0_modid_pluged == slot0_modid_ctrl)) ||
       (slot1_modid_pluged != 0 && (slot1_modid_pluged == slot1_modid_ctrl)))
  {
    return PNIO_S_GOOD;
  }
  return PNIO_S_BAD;
}
		
		
		
/*===========================================================================
* FUNCTION : device_open
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
PNIO_UINT32 IDevice::device_open(PNIO_UINT32 CpIndex,
    PNIO_UINT32 ExtPar, PNIO_UINT16 VendorId, PNIO_UINT16 DeviceId,
    PNIO_UINT16 InstanceId, PNIO_UINT32 MaxAR, PNIO_ANNOTATION * pDevAnnotation,
    PNIO_CFB_FUNCTIONS *pCbf, PNIO_UINT32 * pDevHndl)
{
    IDevice *pThis = NULL;
    light_T_SYNCHD_CHNL Rq;
    PNIO_UINT32 Ret, sendLen, expRLen;
    PNIO_UINT32 ext_prm;

    if(!pDevHndl)
        return PNIO_ERR_PRM_HND;

    if(!pDevAnnotation)
        return PNIO_ERR_PRM_DEV_ANNOTATION;

    if(!pCbf)
        return PNIO_ERR_PRM_PCBF;

    if(pCbf->size > sizeof(PNIO_CFB_FUNCTIONS))
        return PNIO_ERR_PRM_PCBF;

	/* Reset PDEV Check Flags */	
    ar_has_slot_0_pluged = false;
	ctrl_wants_slot_0 = false;
	do_pdev_ioxs_check = true;		
		
    /* check is callback array size less than sum
       of all mandatory callbacks */
    if(pCbf->size <
        (sizeof(PNIO_CFB_FUNCTIONS) -
         sizeof(pCbf->cbf_start_led_flash) -
         sizeof(pCbf->cbf_stop_led_flash))) {
        TRC_OUT(GR_INIT, LV_FCTPUB1, "IDevice::device_open : callback "
            "array size less than sum of all mandatory callbacks");
        return PNIO_ERR_PRM_PCBF;
    }

    /* check for callback, only flash is optional */
    if(!pCbf->cbf_data_write ||
       !pCbf->cbf_data_read ||
       !pCbf->cbf_rec_read ||
       !pCbf->cbf_rec_write ||
       !pCbf->cbf_alarm_done ||
       !pCbf->cbf_check_ind ||
       !pCbf->cbf_ar_check_ind ||
       !pCbf->cbf_ar_info_ind ||
       !pCbf->cbf_ar_indata_ind ||
       !pCbf->cbf_ar_abort_ind ||
       !pCbf->cbf_ar_offline_ind ||
       !pCbf->cbf_apdu_status_ind ||
       !pCbf->cbf_prm_end_ind ||
       !pCbf->cbf_cp_stop_req ||
       !pCbf->cbf_device_stopped)
        return PNIO_ERR_PRM_PCBF;

    pThis = new IDevice;

    if(NULL == pThis)
        return PNIO_ERR_NO_RESOURCE;

    pThis->m_OpenExtPar = ExtPar;
    memcpy(&pThis->m_Cbfs, pCbf, pCbf->size);

    TRC_OUT(GR_INIT, LV_FCTPUB1, "IDevice::device_open : call InitCp");
    Ret = pThis->InitCp(CpIndex);

    if(Ret != PNIO_OK)
        goto device_open_fail_init;

    sendLen = BASIC_LENGTH(Rq) + sizeof(Rq.u.open_device_ext);
    expRLen = BASIC_LENGTH(Rq) + sizeof(Rq.u.resp_open_device);

    memset(&Rq, 0, sizeof (Rq));
    Rq.blk_len = CPU_TO_LE(sendLen);
    Rq.opcode = (SYNCHD_CHNL_OP)CPU_TO_LE(SCOD_OPEN_DEVICE_EXT);
    Rq.u.open_device.cp_index = CPU_TO_LE(1/* instance in FW is always unic! CpIndex */);
    Rq.u.open_device.ext_prm = CPU_TO_LE(ExtPar);
    Rq.u.open_device.vendor_id = CPU_TO_LE(VendorId);
    Rq.u.open_device.device_id = CPU_TO_LE(DeviceId);
    Rq.u.open_device.instance_id = CPU_TO_LE(InstanceId);
    Rq.u.open_device.max_ar = CPU_TO_LE(MaxAR);
    memcpy(Rq.u.open_device.dev_annot.deviceType, pDevAnnotation->deviceType,
        sizeof(Rq.u.open_device.dev_annot.deviceType));
    memcpy(Rq.u.open_device.dev_annot.orderId, pDevAnnotation->orderId,
        sizeof(pDevAnnotation->orderId));
    Rq.u.open_device.dev_annot.hwRevision = CPU_TO_LE16(pDevAnnotation->hwRevision);
    Rq.u.open_device.dev_annot.swRevisionPrefix = pDevAnnotation->swRevisionPrefix;
    Rq.u.open_device.dev_annot.swRevision1 = CPU_TO_LE16(pDevAnnotation->swRevision1);
    Rq.u.open_device.dev_annot.swRevision2 = CPU_TO_LE16(pDevAnnotation->swRevision2);
    Rq.u.open_device.dev_annot.swRevision3 = CPU_TO_LE16(pDevAnnotation->swRevision3);
    ext_prm = 0;
    if(pCbf->cbf_cp_stop_req)
        ext_prm |= EPRM_CP_STOP_REGISTERED;
    if(pCbf->cbf_pull_plug_conf)
        ext_prm |= EPRM_PULL_PLUG_REGISTERED;

    Rq.u.open_device.ext_agent_info = CPU_TO_LE(ext_prm);

#ifdef IRT_NODMA
    Rq.u.open_device.dma_mem_addr = 0;
    Rq.u.open_device.dma_mem_len = 0;
#else
    Rq.u.open_device.dma_mem_addr = CPU_TO_LE(DPR_PTR_TO_ULONG(pThis->m_pCpAdapter->pIRTDMAPhysAddr));
    Rq.u.open_device.dma_mem_len = CPU_TO_LE(pThis->m_pCpAdapter->IRTDMALen);
#endif

    Rq.u.open_device_ext.host_version = (PNIO_UINT32)PNIO_API_VERSION;

    Ret = pThis->SendReceiveSynch((char *)&Rq, sendLen, &expRLen);

    TRC_OUT01(GR_INIT, LV_FCTINT, "IDevice::device_open received len = 0x%x",
        expRLen);
    if(Ret != PNIO_OK)
        goto device_open_fail_send;

    if(LE_TO_CPU(Rq.opcode) != SCOD_OPEN_DEVICE_EXT) {
        TRC_OUT01(GR_INIT, LV_ERR,
            "IDevice::device_open received unknown opcode = 0x%x",
            LE_TO_CPU(Rq.opcode));
        Ret = PNIO_ERR_INTERNAL;
        goto device_open_fail_send;
    }

    if((Ret = ConvertDAgentErrorCodeToApiErrorCode(LE_TO_CPU(Rq.agent_ret))) == PNIO_OK)
        Ret = LE_TO_CPU(Rq.resp_ret);

    if(Ret == PNIO_OK) {
        IODATA_adr_info adr_info;

        memset(&adr_info, 0, sizeof(adr_info));

        TRC_OUT(GR_INIT, LV_FCTINT, "IDevice::device_open SendReceiveSynch OK");

        *pDevHndl = ICommon::get_handle(pThis);
        pThis->m_hIodataUpdate = LE_TO_CPU(Rq.u.resp_open_device.device_hnd);

        adr_info.IOTLB_base = (PNIO_UINT8 *)pThis->m_pCpAdapter->pKramTlb;
        adr_info.EREG_base = (PNIO_UINT8 *)pThis->m_pCpAdapter->pErtecSwiBase;
        adr_info.KRAM_base = (PNIO_UINT8 *)pThis->m_pCpAdapter->pErtecIOTotal;
#ifdef IRT_NODMA
        adr_info.DMA_base = NULL;
#else
        adr_info.DMA_base = (PNIO_UINT8 *)pThis->m_pCpAdapter->pIRTDMAImage;
#endif
        adr_info.iotlb_len = SIZE_KRAMTLB;
        adr_info.IrtAccessStatus = (PNIO_UINT8 *)&pThis->m_pCpAdapter->IrtAccessStatus;

        /* create memory to copy KRAMIOTLB in user space */
        adr_info.host_mem_KRAMTLB_max_cnt = (64 * 2 + 5)/* 64 subslots in & 64 out & 5 pdevs */;
        pThis->m_pHostIOTlb = (KRAMIOTLB_Item *)malloc(adr_info.host_mem_KRAMTLB_max_cnt *
            sizeof(KRAMIOTLB_Item));
        DPR_ASSERT(pThis->m_pHostIOTlb);
        adr_info.host_mem_KRAMTLB = pThis->m_pHostIOTlb;

        TRC_OUT02(GR_INIT, LV_FCTINT, "IODU_dev_open(0x%x, 0x%x)...",
            pThis->m_hIodataUpdate, *pDevHndl);
        Ret = IODU_dev_open(CpIndex,
            pThis->m_hIodataUpdate,
            *pDevHndl,
            pCbf->cbf_data_read, pCbf->cbf_data_write, &(IDevice::CBF_GET_INITIAL_IOXS), 0,
            ExtPar,
            &adr_info,
            &pThis->m_pCpAdapter->pIODUItem);

        /* link table header to dpram. Note: at controller this points to */
        /* a copy of the table.                                           */
        /* for device, we cannot copy the table at this moment            */
        pThis->m_HostIOTlbHdr = pThis->m_pCpAdapter->pIODUItem->krtlbHeader;

        if(Ret != PNIO_OK) {
            TRC_OUT01(GR_INIT, LV_ERR, "Error IODU_dev_open returned 0x%x", Ret);
            pThis->device_close();
            return Ret;
        }

    } else {
        TRC_OUT01(GR_INIT, LV_ERR, "Error SendReceiveSynch returned 0x%x", Ret);
        goto device_open_fail_send;
    }

    return PNIO_OK;

  device_open_fail_send:
    pThis->UninitCp();

  device_open_fail_init:
    delete(pThis);

    return Ret;
}

/*===========================================================================
* FUNCTION : IDevice::device_close
*----------------------------------------------------------------------------
* PURPOSE  : Device close
*----------------------------------------------------------------------------
* RETURNS  :
*----------------------------------------------------------------------------
* INPUTS   : -
* OUTPUS   : -
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
PNIO_UINT32 IDevice::device_close(void)
{
    light_T_SYNCHD_CHNL Rq;
    PNIO_UINT32 Ret, sendLen, expRLen;

    TRC_OUT(GR_INIT, LV_FCTINT, "IODU_dev_close...");

    deinit_and_unregister();
    m_bClosePending = true;

    Ret = IODU_dev_close(m_pCpAdapter->pIODUItem);
    if(Ret != PNIO_OK)
        TRC_OUT02(GR_INIT, LV_ERR, "Error IODU_dev_close(0x%x) returned 0x%x, ignore",
            m_hIodataUpdate, Ret);

    free(m_pHostIOTlb);

    sendLen = BASIC_LENGTH(Rq);
    expRLen = BASIC_LENGTH(Rq);

    memset(&Rq, 0, sizeof (Rq));
    Rq.blk_len = CPU_TO_LE(sendLen);
    Rq.opcode = (SYNCHD_CHNL_OP)CPU_TO_LE(SCOD_CLOSE);
    Rq.handle = CPU_TO_LE(m_hIodataUpdate);

    Ret = SendReceiveSynch((char *)&Rq, sendLen, &expRLen, 90000);

	ar_has_slot_0_pluged = false;
	ctrl_wants_slot_0 = false;
	do_pdev_ioxs_check = false;		
	
    if(Ret != PNIO_OK) {
        TRC_OUT01(GR_INIT, LV_ERR,
            "IDevice::device_close SendReceiveSynch ret = 0x%x, ignore", Ret);
        Ret = PNIO_OK;
        goto device_close_fail;
    }

    if(LE_TO_CPU(Rq.opcode) != SCOD_CLOSE) {
        TRC_OUT01(GR_INIT, LV_ERR,
            "IDevice::device_close received unknown opcode = 0x%x",
            LE_TO_CPU(Rq.opcode));
        Ret = PNIO_ERR_INTERNAL;
        goto device_close_fail;
    }

    if((Ret = ConvertDAgentErrorCodeToApiErrorCode(LE_TO_CPU(Rq.agent_ret))) == PNIO_OK)
        Ret = LE_TO_CPU(Rq.resp_ret);

    if(Ret == PNIO_ERR_WRONG_HND) {
        Ret = PNIO_OK;
        TRC_OUT(GR_INIT, LV_ERR,
            "IDevice::device_close received PNIO_ERR_WRONG_HND, ignore");
    }

    if(Ret != PNIO_OK) {
        TRC_OUT01(GR_INIT, LV_ERR,
            "IDevice::device_close ret = 0x%x", Ret);
    } else {
        UninitCp();

        delete(this);
    }

device_close_fail:
    return Ret;
}

/*===========================================================================
* FUNCTION : IDevice::set_appl_state_ready
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
PNIO_UINT32 IDevice::set_appl_state_ready(PNIO_UINT16 ArNumber,
    PNIO_UINT16 SessionKey, PNIO_APPL_READY_LIST_TYPE * pList)
{
    T_SYNCHD_CHNL *pRq = NULL;
    PNIO_UINT32 Ret, sendLen, expRLen;

    /* recode the double queue list into pseudo array */
    unsigned long ArCnt = 0, ModulesCnt = 0, SubmodulesCnt = 0;
    fw_PNIO_APPL_READY_AP_TYPE  *currentAr;
    fw_PNIO_APPL_READY_MODULE_TYPE *currentModule;
    fw_PNIO_APPL_READY_SUBMODULE_TYPE *currentSubmodule;

    PNIO_APPL_READY_AP_TYPE  *nextAr;
    PNIO_APPL_READY_MODULE_TYPE *nextModule;
    PNIO_APPL_READY_SUBMODULE_TYPE *nextSubmodule;

    if(pList && pList->ap_list.Flink) {
        for(nextAr = (PNIO_APPL_READY_AP_TYPE *)pList->ap_list.Flink;
            nextAr;
            nextAr = (PNIO_APPL_READY_AP_TYPE *)nextAr->link.Flink) {
            ArCnt++;
            for(nextModule = (PNIO_APPL_READY_MODULE_TYPE *)nextAr->module_list.Flink;
                nextModule;
                nextModule = (PNIO_APPL_READY_MODULE_TYPE *)nextModule->link.Flink) {
                ModulesCnt++;
                for(nextSubmodule = (PNIO_APPL_READY_SUBMODULE_TYPE *)nextModule->submodule_list.Flink;
                    nextSubmodule;
                    nextSubmodule = (PNIO_APPL_READY_SUBMODULE_TYPE *)nextSubmodule->link.Flink) {
                    SubmodulesCnt++;
                }
            }
        }
    }

    sendLen = BASIC_LENGTH(*pRq) + sizeof(pRq->u.set_appl_state_ready) +
        ArCnt * sizeof(fw_PNIO_APPL_READY_AP_TYPE) +
        ModulesCnt * sizeof(fw_PNIO_APPL_READY_MODULE_TYPE) +
        SubmodulesCnt * sizeof(fw_PNIO_APPL_READY_SUBMODULE_TYPE);
    expRLen = BASIC_LENGTH(*pRq);

    if(sendLen > m_pCpAdapter->chnls[SYNCH].send_pool_length /*DPR_CHANNEL_SIZE*/)
        return PNIO_ERR_NO_RESOURCE;

    pRq = (T_SYNCHD_CHNL *)m_pCpAdapter->chnls[SYNCH].send_pool_ptr;
    if(!pRq)
        return PNIO_ERR_NO_RESOURCE;
    memset(pRq, 0, sizeof (*pRq));

    pRq->blk_len = CPU_TO_LE(sendLen);
    pRq->opcode = (SYNCHD_CHNL_OP)CPU_TO_LE(SCOD_SET_APPL_STATE_READY);
    pRq->handle = CPU_TO_LE(m_hIodataUpdate);
    pRq->u.set_appl_state_ready.ArNumber = CPU_TO_LE16(ArNumber);
    pRq->u.set_appl_state_ready.SessionKey = CPU_TO_LE16(SessionKey);
    pRq->u.set_appl_state_ready.ArCnt = CPU_TO_LE(ArCnt);
    pRq->u.set_appl_state_ready.ModulesCnt = CPU_TO_LE(ModulesCnt);
    pRq->u.set_appl_state_ready.SubmodulesCnt = CPU_TO_LE(SubmodulesCnt);

    if(pList && pList->ap_list.Flink) {
        currentAr = (fw_PNIO_APPL_READY_AP_TYPE *)(&pRq[1]);
        currentModule = (fw_PNIO_APPL_READY_MODULE_TYPE *)
            ((char *)currentAr + ArCnt * sizeof(fw_PNIO_APPL_READY_AP_TYPE));
        currentSubmodule = (fw_PNIO_APPL_READY_SUBMODULE_TYPE *)
            ((char *)currentModule + ModulesCnt * sizeof(fw_PNIO_APPL_READY_MODULE_TYPE));

        for(nextAr = (PNIO_APPL_READY_AP_TYPE *)pList->ap_list.Flink;
            nextAr;
            currentAr++,
            nextAr = (PNIO_APPL_READY_AP_TYPE *)nextAr->link.Flink) {

            currentAr->api = CPU_TO_LE(nextAr->api);

            for(nextModule = (PNIO_APPL_READY_MODULE_TYPE *)nextAr->module_list.Flink;
                nextModule;
                currentModule++,
                nextModule = (PNIO_APPL_READY_MODULE_TYPE *)nextModule->link.Flink) {

                currentModule->slot_nr = CPU_TO_LE16(nextModule->slot_nr);

                for(nextSubmodule = (PNIO_APPL_READY_SUBMODULE_TYPE *)nextModule->submodule_list.Flink;
                    nextSubmodule;
                    currentSubmodule++,
                    nextSubmodule = (PNIO_APPL_READY_SUBMODULE_TYPE *)nextSubmodule->link.Flink) {

                    currentSubmodule->subslot_nr = CPU_TO_LE16(nextSubmodule->subslot_nr);

                }
            }
        }
    }

    Ret = SendReceiveSynch((char *)pRq, sendLen, &expRLen);

    if(Ret != PNIO_OK)
        goto set_appl_state_ready_quit;

    if(LE_TO_CPU(pRq->opcode) != SCOD_SET_APPL_STATE_READY) {
        TRC_OUT01(GR_STATE, LV_ERR,
            "IDevice::set_appl_state_ready received unknown opcode = 0x%x",
            LE_TO_CPU(pRq->opcode));
        Ret = PNIO_ERR_INTERNAL;
        goto set_appl_state_ready_quit;
    }

    if((Ret = ConvertDAgentErrorCodeToApiErrorCode(LE_TO_CPU(pRq->agent_ret))) == PNIO_OK)
        Ret = LE_TO_CPU(pRq->resp_ret);

set_appl_state_ready_quit:

    return Ret;
}

/*===========================================================================
* FUNCTION : IDevice::set_ar_abort
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
PNIO_UINT32 IDevice::device_ar_abort(PNIO_UINT16 ArNumber,
    PNIO_UINT16 SessionKey)
{
    light_T_SYNCHD_CHNL Rq;
    PNIO_UINT32 Ret, sendLen, expRLen;

    sendLen = BASIC_LENGTH(Rq) + sizeof(Rq.u.ar_abort);
    expRLen = BASIC_LENGTH(Rq);

    memset(&Rq, 0, sizeof (Rq));
    Rq.blk_len = CPU_TO_LE(sendLen);
    Rq.opcode = (SYNCHD_CHNL_OP)CPU_TO_LE(SCOD_DEVICE_AR_ABORT);
    Rq.handle = CPU_TO_LE(m_hIodataUpdate);
    Rq.u.ar_abort.ArNumber = CPU_TO_LE16(ArNumber);
    Rq.u.ar_abort.SessionKey = CPU_TO_LE16(SessionKey);

    Ret = SendReceiveSynch((char *)&Rq, sendLen, &expRLen);

    if(Ret != PNIO_OK)
        return Ret;

    if(LE_TO_CPU(Rq.opcode) != SCOD_DEVICE_AR_ABORT) {
        TRC_OUT01(GR_STATE, LV_ERR,
            "IDevice::set_ar_abort received unknown opcode = 0x%x",
            LE_TO_CPU(Rq.opcode));
        return PNIO_ERR_INTERNAL;
    }

    if((Ret = ConvertDAgentErrorCodeToApiErrorCode(LE_TO_CPU(Rq.agent_ret))) == PNIO_OK)
        Ret = LE_TO_CPU(Rq.resp_ret);

    return Ret;
}

/*===========================================================================
* FUNCTION : IDevice::device_start
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
PNIO_UINT32 IDevice::device_start(void)
{
    light_T_SYNCHD_CHNL Rq;
    PNIO_UINT32 Ret, sendLen, expRLen;

    sendLen = BASIC_LENGTH(Rq);
    expRLen = BASIC_LENGTH(Rq);

    memset(&Rq, 0, sizeof (Rq));
    Rq.blk_len = CPU_TO_LE(sendLen);
    Rq.opcode = (SYNCHD_CHNL_OP)CPU_TO_LE(SCOD_START);
    Rq.handle = CPU_TO_LE(m_hIodataUpdate);

    Ret = SendReceiveSynch((char *)&Rq, sendLen, &expRLen);

    if(Ret != PNIO_OK)
        return Ret;

    if(LE_TO_CPU(Rq.opcode) != SCOD_START) {
        TRC_OUT01(GR_STATE, LV_ERR,
            "IDevice::device_start received unknown opcode = 0x%x",
            LE_TO_CPU(Rq.opcode));
        return PNIO_ERR_INTERNAL;
    }

    if((Ret = ConvertDAgentErrorCodeToApiErrorCode(LE_TO_CPU(Rq.agent_ret))) == PNIO_OK)
        Ret = LE_TO_CPU(Rq.resp_ret);

    return Ret;
}

/*===========================================================================
* FUNCTION : IDevice::device_stop
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
PNIO_UINT32 IDevice::device_stop(void)
{
    light_T_SYNCHD_CHNL Rq;
    PNIO_UINT32 Ret, sendLen, expRLen;

    sendLen = BASIC_LENGTH(Rq);
    expRLen = BASIC_LENGTH(Rq);

    memset(&Rq, 0, sizeof (Rq));
    Rq.blk_len = CPU_TO_LE(sendLen);
    Rq.opcode = (SYNCHD_CHNL_OP)CPU_TO_LE(SCOD_STOP);
    Rq.handle = CPU_TO_LE(m_hIodataUpdate);

    Ret = SendReceiveSynch((char *)&Rq, sendLen, &expRLen);

    if(Ret != PNIO_OK)
        return Ret;

    if(LE_TO_CPU(Rq.opcode) != SCOD_STOP) {
        TRC_OUT01(GR_STATE, LV_ERR,
            "IDevice::device_stop received unknown opcode = 0x%x",
            LE_TO_CPU(Rq.opcode));
        return PNIO_ERR_INTERNAL;
    }

    if((Ret = ConvertDAgentErrorCodeToApiErrorCode(LE_TO_CPU(Rq.agent_ret))) == PNIO_OK)
        Ret = LE_TO_CPU(Rq.resp_ret);

    return Ret;
}

/*===========================================================================
* FUNCTION : IDevice::dev_set_state
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
PNIO_UINT32 IDevice::set_dev_state(PNIO_UINT32 DevState)
{
    light_T_SYNCHD_CHNL Rq;
    PNIO_UINT32 Ret, sendLen, expRLen;

    if(DevState != PNIO_DEVSTAT_OK &&
        DevState != PNIO_DEVSTAT_STATION_PROBLEM)
        return PNIO_ERR_PRM_DEV_STATE;

    sendLen = BASIC_LENGTH(Rq) + sizeof(Rq.u.set_dev_state);
    expRLen = BASIC_LENGTH(Rq);

    memset(&Rq, 0, sizeof (Rq));
    Rq.blk_len = CPU_TO_LE(sendLen);
    Rq.opcode = (SYNCHD_CHNL_OP)CPU_TO_LE(SCOD_SET_DEV_STATE);
    Rq.handle = CPU_TO_LE(m_hIodataUpdate);
    Rq.u.set_dev_state.dev_state = CPU_TO_LE(DevState);

    Ret = SendReceiveSynch((char *)&Rq, sendLen, &expRLen);

    if(Ret != PNIO_OK)
        return Ret;

    if(LE_TO_CPU(Rq.opcode) != SCOD_SET_DEV_STATE) {
        TRC_OUT01(GR_STATE, LV_ERR,
            "IDevice::set_dev_state received unknown opcode = 0x%x",
            LE_TO_CPU(Rq.opcode));
        return PNIO_ERR_INTERNAL;
    }

    if((Ret = ConvertDAgentErrorCodeToApiErrorCode(LE_TO_CPU(Rq.agent_ret))) == PNIO_OK)
        Ret = LE_TO_CPU(Rq.resp_ret);

    return Ret;
}

/*===========================================================================
* FUNCTION : IDevice::api_add
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
PNIO_UINT32 IDevice::api_add(PNIO_UINT32 Api, PNIO_UINT16 MaxnumSlots,
    PNIO_UINT16 MaxnumSubslots)
{
    light_T_SYNCHD_CHNL Rq;
    PNIO_UINT32 Ret, sendLen, expRLen;

    sendLen = BASIC_LENGTH(Rq) + sizeof(Rq.u.api_add);
    expRLen = BASIC_LENGTH(Rq);

    memset(&Rq, 0, sizeof (Rq));
    Rq.blk_len = CPU_TO_LE(sendLen);
    Rq.opcode = (SYNCHD_CHNL_OP)CPU_TO_LE(SCOD_API_ADD);
    Rq.handle = CPU_TO_LE(m_hIodataUpdate);
    Rq.u.api_add.api = CPU_TO_LE(Api);
    Rq.u.api_add.max_slots = CPU_TO_LE16(MaxnumSlots);
    Rq.u.api_add.max_subslots = CPU_TO_LE16(MaxnumSubslots);

    if(MaxnumSlots > 0x7FFF || MaxnumSubslots > 0x7FFF){
      TRC_OUT(GR_STATE, LV_ERR,
            "IDevice::api_add wrong MaxnumSlots or MaxnumSubslots, supported MaxnumSlots=0x7FFF, MaxnumSubslots=0x7FFF");
      return PNIO_ERR_INVALID_CONFIG;
    }

    Ret = SendReceiveSynch((char *)&Rq, sendLen, &expRLen);

    if(Ret != PNIO_OK)
        return Ret;

    if(LE_TO_CPU(Rq.opcode) != SCOD_API_ADD) {
        TRC_OUT01(GR_STATE, LV_ERR,
            "IDevice::api_add received unknown opcode = 0x%x",
            LE_TO_CPU(Rq.opcode));
        return PNIO_ERR_INTERNAL;
    }

    if((Ret = ConvertDAgentErrorCodeToApiErrorCode(LE_TO_CPU(Rq.agent_ret))) == PNIO_OK)
        Ret = LE_TO_CPU(Rq.resp_ret);

    return Ret;
}

/*===========================================================================
* FUNCTION : IDevice::api_remove
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
PNIO_UINT32 IDevice::api_remove(PNIO_UINT32 Api)
{
    light_T_SYNCHD_CHNL Rq;
    PNIO_UINT32 Ret, sendLen, expRLen;

    sendLen = BASIC_LENGTH(Rq) + sizeof(Rq.u.api_remove);
    expRLen = BASIC_LENGTH(Rq);

    memset(&Rq, 0, sizeof (Rq));
    Rq.blk_len = CPU_TO_LE(sendLen);
    Rq.opcode = (SYNCHD_CHNL_OP)CPU_TO_LE(SCOD_API_REMOVE);
    Rq.handle = CPU_TO_LE(m_hIodataUpdate);
    Rq.u.api_remove.api = CPU_TO_LE(Api);

    Ret = SendReceiveSynch((char *)&Rq, sendLen, &expRLen);

    if(Ret != PNIO_OK)
        return Ret;

    if(LE_TO_CPU(Rq.opcode) != SCOD_API_REMOVE) {
        TRC_OUT01(GR_STATE, LV_ERR,
            "IDevice::api_remove received unknown opcode = 0x%x",
            LE_TO_CPU(Rq.opcode));
        return PNIO_ERR_INTERNAL;
    }

    if((Ret = ConvertDAgentErrorCodeToApiErrorCode(LE_TO_CPU(Rq.agent_ret))) == PNIO_OK)
        Ret = LE_TO_CPU(Rq.resp_ret);

    return Ret;
}

/*===========================================================================
* FUNCTION : IDevice::mod_pull
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
PNIO_UINT32 IDevice::mod_pull(PNIO_UINT32 Api, PNIO_DEV_ADDR * pAddr)
{
    light_T_SYNCHD_CHNL Rq;
    PNIO_UINT32 Ret, sendLen, expRLen;

    if(!pAddr)
        return PNIO_ERR_PRM_ADD;

    sendLen = BASIC_LENGTH(Rq) + sizeof(Rq.u.pull);
    expRLen = BASIC_LENGTH(Rq);

    memset(&Rq, 0, sizeof (Rq));
    Rq.blk_len = CPU_TO_LE(sendLen);
    Rq.opcode = (SYNCHD_CHNL_OP)CPU_TO_LE(SCOD_PULL);
    Rq.handle = CPU_TO_LE(m_hIodataUpdate);
    Rq.u.pull.obj_type = 0;
    Rq.u.pull.Api = CPU_TO_LE(Api);
    Rq.u.pull.Addr.AddrType = (PNIO_ADDR_TYPE)CPU_TO_LE(pAddr->AddrType);
    Rq.u.pull.Addr.IODataType = (PNIO_IO_TYPE)CPU_TO_LE(pAddr->IODataType);
    Rq.u.pull.Addr.u.Geo.Slot = CPU_TO_LE(pAddr->u.Geo.Slot);
    Rq.u.pull.Addr.u.Geo.Subslot = CPU_TO_LE(pAddr->u.Geo.Subslot);

    Ret = SendReceiveSynch((char *)&Rq, sendLen, &expRLen);

    if(Ret != PNIO_OK)
        return Ret;

    if(LE_TO_CPU(Rq.opcode) != SCOD_PULL) {
        TRC_OUT01(GR_STATE, LV_ERR,
            "IDevice::mod_pull received unknown opcode = 0x%x",
            LE_TO_CPU(Rq.opcode));
        return PNIO_ERR_INTERNAL;
    }

    if((Ret = ConvertDAgentErrorCodeToApiErrorCode(LE_TO_CPU(Rq.agent_ret))) == PNIO_OK)
        Ret = LE_TO_CPU(Rq.resp_ret);

    return Ret;
}

/*===========================================================================
* FUNCTION : IDevice::sub_pull
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
PNIO_UINT32 IDevice::sub_pull(PNIO_UINT32 Api, PNIO_DEV_ADDR * pAddr)
{
    light_T_SYNCHD_CHNL Rq;
    PNIO_UINT32 Ret, sendLen, expRLen;

    if(!pAddr)
        return PNIO_ERR_PRM_ADD;

    sendLen = BASIC_LENGTH(Rq) + sizeof(Rq.u.pull);
    expRLen = BASIC_LENGTH(Rq);

    memset(&Rq, 0, sizeof (Rq));
    Rq.blk_len = CPU_TO_LE(sendLen);
    Rq.opcode = (SYNCHD_CHNL_OP)CPU_TO_LE(SCOD_PULL);
    Rq.handle = CPU_TO_LE(m_hIodataUpdate);
    Rq.u.pull.obj_type = 1;
    Rq.u.pull.Api = CPU_TO_LE(Api);
    Rq.u.pull.Addr.AddrType = (PNIO_ADDR_TYPE)CPU_TO_LE(pAddr->AddrType);
    Rq.u.pull.Addr.IODataType = (PNIO_IO_TYPE)CPU_TO_LE(pAddr->IODataType);
    Rq.u.pull.Addr.u.Geo.Slot = CPU_TO_LE(pAddr->u.Geo.Slot);
    Rq.u.pull.Addr.u.Geo.Subslot = CPU_TO_LE(pAddr->u.Geo.Subslot);

    if (pAddr->u.Geo.Slot == 0)
    {
      slot0_modid_pluged = 0;
    }
    else if(pAddr->u.Geo.Slot == 1)
    {
      slot1_modid_pluged = 0;
    }

    Ret = SendReceiveSynch((char *)&Rq, sendLen, &expRLen);

    if(Ret != PNIO_OK)
        return Ret;

    if(LE_TO_CPU(Rq.opcode) != SCOD_PULL) {
        TRC_OUT01(GR_STATE, LV_ERR,
            "IDevice::sub_pull received unknown opcode = 0x%x",
            LE_TO_CPU(Rq.opcode));
        return PNIO_ERR_INTERNAL;
    }

    if((Ret = ConvertDAgentErrorCodeToApiErrorCode(LE_TO_CPU(Rq.agent_ret))) == PNIO_OK)
        Ret = LE_TO_CPU(Rq.resp_ret);

    return Ret;
}

/*===========================================================================
* FUNCTION : IDevice::sub_plug
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
PNIO_UINT32 IDevice::sub_plug(PNIO_UINT32 Api, PNIO_DEV_ADDR * pAddr,
    PNIO_UINT32 SubIdent)
{
    light_T_SYNCHD_CHNL Rq;
    PNIO_UINT32 Ret, sendLen, expRLen;

    if(!pAddr)
        return PNIO_ERR_PRM_ADD;

    sendLen = BASIC_LENGTH(Rq) + sizeof(Rq.u.plug);
    expRLen = BASIC_LENGTH(Rq);

    memset(&Rq, 0, sizeof (Rq));
    Rq.blk_len = CPU_TO_LE(sendLen);
    Rq.opcode = (SYNCHD_CHNL_OP)CPU_TO_LE(SCOD_PLUG);
    Rq.handle = CPU_TO_LE(m_hIodataUpdate);
    Rq.u.plug.obj_type = 1;
    Rq.u.plug.Api = CPU_TO_LE(Api);
    Rq.u.plug.Addr.AddrType = (PNIO_ADDR_TYPE)CPU_TO_LE(pAddr->AddrType);
    Rq.u.plug.Addr.IODataType = (PNIO_IO_TYPE)CPU_TO_LE(pAddr->IODataType);
    Rq.u.plug.Addr.u.Geo.Slot = CPU_TO_LE(pAddr->u.Geo.Slot);
    Rq.u.plug.Addr.u.Geo.Subslot = CPU_TO_LE(pAddr->u.Geo.Subslot);
    Rq.u.plug.Ident = CPU_TO_LE(SubIdent);

    Ret = SendReceiveSynch((char *)&Rq, sendLen, &expRLen);

    if(Ret != PNIO_OK)
        return Ret;

    if(LE_TO_CPU(Rq.opcode) != SCOD_PLUG) {
        TRC_OUT01(GR_STATE, LV_ERR,
            "IDevice::sub_plug received unknown opcode = 0x%x",
            LE_TO_CPU(Rq.opcode));
        return PNIO_ERR_INTERNAL;
    }

    if((Ret = ConvertDAgentErrorCodeToApiErrorCode(LE_TO_CPU(Rq.agent_ret))) == PNIO_OK)
        Ret = LE_TO_CPU(Rq.resp_ret);

    return Ret;
}

/*===========================================================================
* FUNCTION : IDevice::sub_plug
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
PNIO_UINT32 IDevice::sub_plug_ext(PNIO_UINT32 Api, PNIO_DEV_ADDR * pAddr,
    PNIO_UINT32 SubIdent, PNIO_UINT32 AlarmType)
{
    light_T_SYNCHD_CHNL Rq;
    PNIO_UINT32 Ret, sendLen, expRLen;

    if(!pAddr)
        return PNIO_ERR_PRM_ADD;

    sendLen = BASIC_LENGTH(Rq) + sizeof(Rq.u.plug);
    expRLen = BASIC_LENGTH(Rq);

    memset(&Rq, 0, sizeof (Rq));
    Rq.blk_len = CPU_TO_LE(sendLen);
    Rq.opcode = (SYNCHD_CHNL_OP)CPU_TO_LE(SCOD_PLUG_SUB_EXT);
    Rq.handle = CPU_TO_LE(m_hIodataUpdate);
    Rq.u.plug.obj_type = CPU_TO_LE(1);
    Rq.u.plug.Api = CPU_TO_LE(Api);
    Rq.u.plug.Addr.AddrType = (PNIO_ADDR_TYPE)CPU_TO_LE(pAddr->AddrType);
    Rq.u.plug.Addr.IODataType = (PNIO_IO_TYPE)CPU_TO_LE(pAddr->IODataType);
    Rq.u.plug.Addr.u.Geo.Slot = CPU_TO_LE(pAddr->u.Geo.Slot);
    Rq.u.plug.Addr.u.Geo.Subslot = CPU_TO_LE(pAddr->u.Geo.Subslot);
    Rq.u.plug.Ident = CPU_TO_LE(SubIdent);
    Rq.u.plug.AlarmType = CPU_TO_LE(AlarmType);

    Ret = SendReceiveSynch((char *)&Rq, sendLen, &expRLen);

    if(Ret != PNIO_OK)
        return Ret;

    if(LE_TO_CPU(Rq.opcode) != SCOD_PLUG_SUB_EXT) {
        TRC_OUT01(GR_STATE, LV_ERR,
            "IDevice::sub_plug_ext received unknown opcode = 0x%x",
            LE_TO_CPU(Rq.opcode));
        return PNIO_ERR_INTERNAL;
    }

    if((Ret = ConvertDAgentErrorCodeToApiErrorCode(LE_TO_CPU(Rq.agent_ret))) == PNIO_OK)
        Ret = LE_TO_CPU(Rq.resp_ret);

    return Ret;
}

/*===========================================================================
* FUNCTION : IDevice::sub_plug_ext_im
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
PNIO_UINT32 IDevice::sub_plug_ext_im(PNIO_UINT32 Api, PNIO_DEV_ADDR * pAddr,
    PNIO_UINT32 SubIdent, PNIO_UINT32 AlarmType, PNIO_PLUG_IM0_BITS  IM0_bits)
{
    light_T_SYNCHD_CHNL Rq;
    PNIO_UINT32 Ret, sendLen, expRLen;

    if(!pAddr)
        return PNIO_ERR_PRM_ADD;

    sendLen = BASIC_LENGTH(Rq) + sizeof(Rq.u.plug);
    expRLen = BASIC_LENGTH(Rq);

    memset(&Rq, 0, sizeof (Rq));
    Rq.blk_len = CPU_TO_LE(sendLen);
    Rq.opcode = (SYNCHD_CHNL_OP)CPU_TO_LE(SCOD_PLUG_SUB_EXT_IM);
    Rq.handle = CPU_TO_LE(m_hIodataUpdate);
    Rq.u.plug.obj_type = CPU_TO_LE(1);
    Rq.u.plug.Api = CPU_TO_LE(Api);
    Rq.u.plug.Addr.AddrType = (PNIO_ADDR_TYPE)CPU_TO_LE(pAddr->AddrType);
    Rq.u.plug.Addr.IODataType = (PNIO_IO_TYPE)CPU_TO_LE(pAddr->IODataType);
    Rq.u.plug.Addr.u.Geo.Slot = CPU_TO_LE(pAddr->u.Geo.Slot);
    Rq.u.plug.Addr.u.Geo.Subslot = CPU_TO_LE(pAddr->u.Geo.Subslot);
    Rq.u.plug.Ident = CPU_TO_LE(SubIdent);
    Rq.u.plug.AlarmType = CPU_TO_LE(AlarmType);
    Rq.u.plug.IM0_bits  = (PNIO_PLUG_IM0_BITS)CPU_TO_LE(IM0_bits);

    Ret = SendReceiveSynch((char *)&Rq, sendLen, &expRLen);

    if(Ret != PNIO_OK)
        return Ret;

    if(LE_TO_CPU(Rq.opcode) != SCOD_PLUG_SUB_EXT_IM) {
        TRC_OUT01(GR_STATE, LV_ERR,
            "IDevice::sub_plug_ext_im received unknown opcode = 0x%x",
            LE_TO_CPU(Rq.opcode));
        return PNIO_ERR_INTERNAL;
    }

    if((Ret = ConvertDAgentErrorCodeToApiErrorCode(LE_TO_CPU(Rq.agent_ret))) == PNIO_OK)
        Ret = LE_TO_CPU(Rq.resp_ret);

    return Ret;
}

/*===========================================================================
* FUNCTION : IDevice::mod_plug
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
PNIO_UINT32 IDevice::mod_plug(PNIO_UINT32 Api, PNIO_DEV_ADDR * pAddr,
    PNIO_UINT32 ModIdent)
{
    light_T_SYNCHD_CHNL Rq;
    PNIO_UINT32 Ret, sendLen, expRLen;

    if(!pAddr)
        return PNIO_ERR_PRM_ADD;

    sendLen = BASIC_LENGTH(Rq) + sizeof(Rq.u.plug);
    expRLen = BASIC_LENGTH(Rq);

    memset(&Rq, 0, sizeof (Rq));
    Rq.blk_len = CPU_TO_LE(sendLen);
    Rq.opcode = (SYNCHD_CHNL_OP)CPU_TO_LE(SCOD_PLUG);
    Rq.handle = CPU_TO_LE(m_hIodataUpdate);
    Rq.u.plug.obj_type = 0;
    Rq.u.plug.Api = CPU_TO_LE(Api);
    Rq.u.plug.Addr.AddrType = (PNIO_ADDR_TYPE)CPU_TO_LE(pAddr->AddrType);
    Rq.u.plug.Addr.IODataType = (PNIO_IO_TYPE)CPU_TO_LE(pAddr->IODataType);
    Rq.u.plug.Addr.u.Geo.Slot = CPU_TO_LE(pAddr->u.Geo.Slot);
    Rq.u.plug.Addr.u.Geo.Subslot = CPU_TO_LE(pAddr->u.Geo.Subslot);
    Rq.u.plug.Ident = CPU_TO_LE(ModIdent);

    if (pAddr->u.Geo.Slot == 0)
    {
      ar_has_slot_0_pluged = true;
      slot0_modid_pluged = CPU_TO_LE(ModIdent);
    }
    else if(pAddr->u.Geo.Slot == 1)
    {
      slot1_modid_pluged = CPU_TO_LE(ModIdent);
    }

    Ret = SendReceiveSynch((char *)&Rq, sendLen, &expRLen);

    if(Ret != PNIO_OK)
        return Ret;

    if(LE_TO_CPU(Rq.opcode) != SCOD_PLUG) {
        TRC_OUT01(GR_STATE, LV_ERR,
            "IDevice::mod_plug received unknown opcode = 0x%x",
            LE_TO_CPU(Rq.opcode));
        return PNIO_ERR_INTERNAL;
    }

    if((Ret = ConvertDAgentErrorCodeToApiErrorCode(LE_TO_CPU(Rq.agent_ret))) == PNIO_OK)
        Ret = LE_TO_CPU(Rq.resp_ret);

    return Ret;
}

/*===========================================================================
* FUNCTION : IDevice::build_channel_properties
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
PNIO_UINT16 IDevice::build_channel_properties(PNIO_UINT16 Type,
    PNIO_UINT16 Spec, PNIO_UINT16 Dir)
{
    light_T_SYNCHD_CHNL Rq;
    PNIO_UINT32 Ret, sendLen, expRLen;

    sendLen = BASIC_LENGTH(Rq) + sizeof(Rq.u.build_ch_prop);
    expRLen = BASIC_LENGTH(Rq);

    memset(&Rq, 0, sizeof (Rq));
    Rq.blk_len = CPU_TO_LE(sendLen);
    Rq.opcode = (SYNCHD_CHNL_OP)CPU_TO_LE(SCOD_BUILD_CH_PROP);
    Rq.handle = CPU_TO_LE(m_hIodataUpdate);
    Rq.u.build_ch_prop.Type = CPU_TO_LE16(Type);
    Rq.u.build_ch_prop.Spec = CPU_TO_LE16(Spec);
    Rq.u.build_ch_prop.Dir = CPU_TO_LE16(Dir);

    Ret = SendReceiveSynch((char *)&Rq, sendLen, &expRLen);

    if(Ret != PNIO_OK)
        return Ret;

    if(LE_TO_CPU(Rq.opcode) != SCOD_BUILD_CH_PROP) {
        TRC_OUT01(GR_STATE, LV_ERR,
            "IDevice::build_channel_properties received unknown opcode = 0x%x",
            LE_TO_CPU(Rq.opcode));
        return PNIO_ERR_INTERNAL;
    }

    if((Ret = ConvertDAgentErrorCodeToApiErrorCode(LE_TO_CPU(Rq.agent_ret))) == PNIO_OK)
        Ret = LE_TO_CPU(Rq.resp_ret);

    return Ret;
}

/*===========================================================================
* FUNCTION : IDevice::diag_remove
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
PNIO_UINT32 IDevice::diag_remove(PNIO_UINT32 Api,
    PNIO_DEV_ADDR *pAddr, PNIO_UINT16 DiagTag, PNIO_UINT8 DiagType)
{
    light_T_SYNCHD_CHNL Rq;
    PNIO_UINT32 Ret, sendLen, expRLen;

    if(!pAddr)
        return PNIO_ERR_PRM_ADD;

    sendLen = BASIC_LENGTH(Rq) + sizeof(Rq.u.diag_remove);
    expRLen = BASIC_LENGTH(Rq);

    memset(&Rq, 0, sizeof (Rq));
    Rq.blk_len = CPU_TO_LE(sendLen);
    Rq.opcode = (SYNCHD_CHNL_OP)CPU_TO_LE(SCOD_DIAG_REMOVE);
    Rq.handle = CPU_TO_LE(m_hIodataUpdate);
    Rq.u.diag_remove.diag_type = DiagType;
    Rq.u.diag_remove.Api = CPU_TO_LE(Api);
    Rq.u.diag_remove.Addr.AddrType = (PNIO_ADDR_TYPE)CPU_TO_LE(pAddr->AddrType);
    Rq.u.diag_remove.Addr.IODataType = (PNIO_IO_TYPE)CPU_TO_LE(pAddr->IODataType);
    Rq.u.diag_remove.Addr.u.Geo.Slot = CPU_TO_LE(pAddr->u.Geo.Slot);
    Rq.u.diag_remove.Addr.u.Geo.Subslot = CPU_TO_LE(pAddr->u.Geo.Subslot);
    Rq.u.diag_remove.DiagTag = CPU_TO_LE16(DiagTag);

    Ret = SendReceiveSynch((char *)&Rq, sendLen, &expRLen);

    if(Ret != PNIO_OK)
        return Ret;

    if(LE_TO_CPU(Rq.opcode) != SCOD_DIAG_REMOVE) {
        TRC_OUT01(GR_STATE, LV_ERR,
            "IDevice::diag_remove received unknown opcode = 0x%x",
            LE_TO_CPU(Rq.opcode));
        return PNIO_ERR_INTERNAL;
    }

    if((Ret = ConvertDAgentErrorCodeToApiErrorCode(LE_TO_CPU(Rq.agent_ret))) == PNIO_OK)
        Ret = LE_TO_CPU(Rq.resp_ret);

    return Ret;
}

/*===========================================================================
* FUNCTION : IDevice::diag_channel_add
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
PNIO_UINT32 IDevice::diag_channel_add(PNIO_UINT32 Api,
    PNIO_DEV_ADDR *pAddr, PNIO_UINT16 ChannelNum, PNIO_UINT16 ChannelProp,
    PNIO_UINT32 ChannelErrType, PNIO_UINT16 DiagTag)
{
    light_T_SYNCHD_CHNL Rq;
    PNIO_UINT32 Ret, sendLen, expRLen;

    if(!pAddr)
        return PNIO_ERR_PRM_ADD;

    sendLen = BASIC_LENGTH(Rq) + sizeof(Rq.u.diag_ch_add);
    expRLen = BASIC_LENGTH(Rq);

    memset(&Rq, 0, sizeof (Rq));
    Rq.blk_len = CPU_TO_LE(sendLen);
    Rq.opcode = (SYNCHD_CHNL_OP)CPU_TO_LE(SCOD_DIAG_CH_ADD);
    Rq.handle = CPU_TO_LE(m_hIodataUpdate);
    Rq.u.diag_ch_add.Api = CPU_TO_LE(Api);
    Rq.u.diag_ch_add.Addr.AddrType = (PNIO_ADDR_TYPE)CPU_TO_LE(pAddr->AddrType);
    Rq.u.diag_ch_add.Addr.IODataType = (PNIO_IO_TYPE)CPU_TO_LE(pAddr->IODataType);
    Rq.u.diag_ch_add.Addr.u.Geo.Slot = CPU_TO_LE(pAddr->u.Geo.Slot);
    Rq.u.diag_ch_add.Addr.u.Geo.Subslot = CPU_TO_LE(pAddr->u.Geo.Subslot);
    Rq.u.diag_ch_add.ChannelNum = CPU_TO_LE16(ChannelNum);
    Rq.u.diag_ch_add.ChannelProp = CPU_TO_LE16(ChannelProp);
    Rq.u.diag_ch_add.ChannelErrType = CPU_TO_LE(ChannelErrType);
    Rq.u.diag_ch_add.DiagTag = CPU_TO_LE16(DiagTag);

    Ret = SendReceiveSynch((char *)&Rq, sendLen, &expRLen);

    if(Ret != PNIO_OK)
        return Ret;

    if(LE_TO_CPU(Rq.opcode) != SCOD_DIAG_CH_ADD) {
        TRC_OUT01(GR_STATE, LV_ERR,
            "IDevice::diag_channel_add received unknown opcode = 0x%x",
            LE_TO_CPU(Rq.opcode));
        return PNIO_ERR_INTERNAL;
    }

    if((Ret = ConvertDAgentErrorCodeToApiErrorCode(LE_TO_CPU(Rq.agent_ret))) == PNIO_OK)
        Ret = LE_TO_CPU(Rq.resp_ret);

    return Ret;
}

/*===========================================================================
* FUNCTION : IDevice::diag_channel_add
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
PNIO_UINT32 IDevice::diag_ext_channel_add(PNIO_UINT32 Api,
    PNIO_DEV_ADDR *pAddr, PNIO_UINT16 ChannelNum, PNIO_UINT16 ChannelProp,
    PNIO_UINT32 ChannelErrType, PNIO_UINT16 ExtChannelErrType,
    PNIO_UINT32 ExtChannelAddValue, PNIO_UINT16 DiagTag)
{
    light_T_SYNCHD_CHNL Rq;
    PNIO_UINT32 Ret, sendLen, expRLen;

    if(!pAddr)
        return PNIO_ERR_PRM_ADD;

    sendLen = BASIC_LENGTH(Rq) + sizeof(Rq.u.diag_ext_ch_add);
    expRLen = BASIC_LENGTH(Rq);

    memset(&Rq, 0, sizeof (Rq));
    Rq.blk_len = CPU_TO_LE(sendLen);
    Rq.opcode = (SYNCHD_CHNL_OP)CPU_TO_LE(SCOD_DIAG_EXT_CH_ADD);
    Rq.handle = CPU_TO_LE(m_hIodataUpdate);
    Rq.u.diag_ext_ch_add.Api = CPU_TO_LE(Api);
    Rq.u.diag_ext_ch_add.Addr.AddrType = (PNIO_ADDR_TYPE)CPU_TO_LE(pAddr->AddrType);
    Rq.u.diag_ext_ch_add.Addr.IODataType = (PNIO_IO_TYPE)CPU_TO_LE(pAddr->IODataType);
    Rq.u.diag_ext_ch_add.Addr.u.Geo.Slot = CPU_TO_LE(pAddr->u.Geo.Slot);
    Rq.u.diag_ext_ch_add.Addr.u.Geo.Subslot = CPU_TO_LE(pAddr->u.Geo.Subslot);
    Rq.u.diag_ext_ch_add.ChannelNum = CPU_TO_LE16(ChannelNum);
    Rq.u.diag_ext_ch_add.ChannelProp = CPU_TO_LE16(ChannelProp);
    Rq.u.diag_ext_ch_add.ChannelErrType = CPU_TO_LE(ChannelErrType);
        Rq.u.diag_ext_ch_add.ExtChannelErrType = CPU_TO_LE(ExtChannelErrType);
        Rq.u.diag_ext_ch_add.ExtChannelAddValue = CPU_TO_LE(ExtChannelAddValue);
    Rq.u.diag_ext_ch_add.DiagTag = CPU_TO_LE16(DiagTag);

    Ret = SendReceiveSynch((char *)&Rq, sendLen, &expRLen);

    if(Ret != PNIO_OK)
        return Ret;

    if(LE_TO_CPU(Rq.opcode) != SCOD_DIAG_EXT_CH_ADD) {
        TRC_OUT01(GR_STATE, LV_ERR,
            "IDevice::diag_ext_channel_add received unknown opcode = 0x%x",
            LE_TO_CPU(Rq.opcode));
        return PNIO_ERR_INTERNAL;
    }

    if((Ret = ConvertDAgentErrorCodeToApiErrorCode(LE_TO_CPU(Rq.agent_ret))) == PNIO_OK)
        Ret = LE_TO_CPU(Rq.resp_ret);

    return Ret;
}

/*===========================================================================
* FUNCTION : IDevice::diag_generic_add
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
PNIO_UINT32 IDevice::diag_generic_add(PNIO_UINT32 Api,
    PNIO_DEV_ADDR *pAddr, PNIO_UINT16 ChannelNum,
    PNIO_UINT16 ChannelProp, PNIO_UINT16 DiagTag,
    PNIO_UINT16 UserStructIdent,
    PNIO_UINT8 *pInfoData, PNIO_UINT32 InfoDataLen)
{
    T_SYNCHD_CHNL Rq;
    PNIO_UINT32 Ret, sendLen, expRLen;

    if(!pAddr)
        return PNIO_ERR_PRM_ADD;

    if(InfoDataLen > sizeof(Rq.u.diag_gen_add.InfoData))
        return PNIO_ERR_PRM_LEN;

    if(InfoDataLen && !pInfoData)
        return PNIO_ERR_PRM_BUF;

    sendLen = BASIC_LENGTH(Rq) + sizeof(Rq.u.diag_gen_add)
        - sizeof(Rq.u.diag_gen_add.InfoData) + InfoDataLen;
    expRLen = BASIC_LENGTH(Rq);

    memset(&Rq, 0, sizeof (Rq));
    Rq.blk_len = CPU_TO_LE(sendLen);
    Rq.opcode = (SYNCHD_CHNL_OP)CPU_TO_LE(SCOD_DIAG_GENERIC_ADD);
    Rq.handle = CPU_TO_LE(m_hIodataUpdate);
    Rq.u.diag_gen_add.Api = CPU_TO_LE(Api);
    Rq.u.diag_gen_add.Addr.AddrType = (PNIO_ADDR_TYPE)CPU_TO_LE(pAddr->AddrType);
    Rq.u.diag_gen_add.Addr.IODataType = (PNIO_IO_TYPE)CPU_TO_LE(pAddr->IODataType);
    Rq.u.diag_gen_add.Addr.u.Geo.Slot = CPU_TO_LE(pAddr->u.Geo.Slot);
    Rq.u.diag_gen_add.Addr.u.Geo.Subslot = CPU_TO_LE(pAddr->u.Geo.Subslot);
    Rq.u.diag_gen_add.ChannelNum = CPU_TO_LE16(ChannelNum);
    Rq.u.diag_gen_add.ChannelProp = CPU_TO_LE16(ChannelProp);
    Rq.u.diag_gen_add.DiagTag = CPU_TO_LE16(DiagTag);
    Rq.u.diag_gen_add.UserStructIdent = CPU_TO_LE16(UserStructIdent);
    memcpy(Rq.u.diag_gen_add.InfoData, pInfoData, InfoDataLen);
    Rq.u.diag_gen_add.InfoDataLen = CPU_TO_LE(InfoDataLen);

    Ret = SendReceiveSynch((char *)&Rq, sendLen, &expRLen);

    if(Ret != PNIO_OK)
        return Ret;

    if(LE_TO_CPU(Rq.opcode) != SCOD_DIAG_GENERIC_ADD) {
        TRC_OUT01(GR_STATE, LV_ERR,
            "IDevice::diag_generic_add received unknown opcode = 0x%x",
            LE_TO_CPU(Rq.opcode));
        return PNIO_ERR_INTERNAL;
    }

    if((Ret = ConvertDAgentErrorCodeToApiErrorCode(LE_TO_CPU(Rq.agent_ret))) == PNIO_OK)
        Ret = LE_TO_CPU(Rq.resp_ret);

    return Ret;
}

/*===========================================================================
* FUNCTION : IDevice::process_alarm_send
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
PNIO_UINT32 IDevice::process_alarm_send(PNIO_UINT32 Api,
    PNIO_UINT16 ArNumber, PNIO_UINT16 SessionKey, PNIO_DEV_ADDR *pAddr,
    PNIO_UINT8 *pData, PNIO_UINT32 DataLen, PNIO_UINT16 UserStructIdent,
    PNIO_UINT32 UserHndl)
{
    T_SYNCHD_CHNL Rq;
    PNIO_UINT32 Ret, sendLen, expRLen;

    if(!pAddr)
        return PNIO_ERR_PRM_ADD;

    if(DataLen > PNIOD_ALRM_USER_DATA_MAX_LEN /*sizeof(Rq.u.alarm_send.Data)*/)
        return PNIO_ERR_PRM_LEN;

    if(DataLen && !pData)
        return PNIO_ERR_PRM_BUF;

    sendLen = BASIC_LENGTH(Rq) + sizeof(Rq.u.alarm_send)
        - sizeof(Rq.u.alarm_send.Data) + DataLen;
    expRLen = BASIC_LENGTH(Rq);

    memset(&Rq, 0, sizeof (Rq));
    Rq.blk_len = CPU_TO_LE(sendLen);
    Rq.opcode = (SYNCHD_CHNL_OP)CPU_TO_LE(SCOD_ALARM_SEND);
    Rq.handle = CPU_TO_LE(m_hIodataUpdate);
    Rq.u.alarm_send.alarm_type = 0;
    Rq.u.alarm_send.Api = CPU_TO_LE(Api);
    Rq.u.alarm_send.ArNumber = CPU_TO_LE16(ArNumber);
    Rq.u.alarm_send.SessionKey = CPU_TO_LE16(SessionKey);
    Rq.u.alarm_send.Addr.AddrType = (PNIO_ADDR_TYPE)CPU_TO_LE(pAddr->AddrType);
    Rq.u.alarm_send.Addr.IODataType = (PNIO_IO_TYPE)CPU_TO_LE(pAddr->IODataType);
    Rq.u.alarm_send.Addr.u.Geo.Slot = CPU_TO_LE(pAddr->u.Geo.Slot);
    Rq.u.alarm_send.Addr.u.Geo.Subslot = CPU_TO_LE(pAddr->u.Geo.Subslot);
    memcpy(Rq.u.alarm_send.Data, pData, DataLen);
    Rq.u.alarm_send.DataLen = CPU_TO_LE(DataLen);
    Rq.u.alarm_send.UserStructIdent = CPU_TO_LE16(UserStructIdent);
    Rq.u.alarm_send.UserHndl = CPU_TO_LE(UserHndl);

    Ret = SendReceiveSynch((char *)&Rq, sendLen, &expRLen);

    if(Ret != PNIO_OK)
        return Ret;

    if(LE_TO_CPU(Rq.opcode) != SCOD_ALARM_SEND) {
        TRC_OUT01(GR_STATE, LV_ERR,
            "IDevice::process_alarm_send received unknown opcode = 0x%x",
            LE_TO_CPU(Rq.opcode));
        return PNIO_ERR_INTERNAL;
    }

    if((Ret = ConvertDAgentErrorCodeToApiErrorCode(LE_TO_CPU(Rq.agent_ret))) == PNIO_OK)
        Ret = LE_TO_CPU(Rq.resp_ret);

    return Ret;
}

/*===========================================================================
* FUNCTION : IDevice::diag_alarm_send
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
PNIO_UINT32 IDevice::diag_alarm_send(PNIO_UINT32 Api, PNIO_UINT16 ArNumber,
    PNIO_UINT16 SessionKey, PNIO_UINT32 AlarmState,
    PNIO_DEV_ADDR *pAddr, PNIO_UINT8 *pData,
    PNIO_UINT32 DataLen, PNIO_UINT16 UserStructIdent,
    PNIO_UINT32 UserHndl)
{
    T_SYNCHD_CHNL Rq;
    PNIO_UINT32 Ret, sendLen, expRLen;

    if(!pAddr)
        return PNIO_ERR_PRM_ADD;

    if(DataLen > PNIOD_ALRM_USER_DATA_MAX_LEN /*sizeof(Rq.u.alarm_send.Data)*/)
        return PNIO_ERR_PRM_LEN;

    if(DataLen && !pData)
        return PNIO_ERR_PRM_BUF;

    sendLen = BASIC_LENGTH(Rq) + sizeof(Rq.u.alarm_send)
        - sizeof(Rq.u.alarm_send.Data) + DataLen;
    expRLen = BASIC_LENGTH(Rq);

    memset(&Rq, 0, sizeof (Rq));
    Rq.blk_len = CPU_TO_LE(sendLen);
    Rq.opcode = (SYNCHD_CHNL_OP)CPU_TO_LE(SCOD_ALARM_SEND);
    Rq.handle = CPU_TO_LE(m_hIodataUpdate);
    Rq.u.alarm_send.alarm_type = 1;
    Rq.u.alarm_send.Api = CPU_TO_LE(Api);
    Rq.u.alarm_send.ArNumber = CPU_TO_LE16(ArNumber);
    Rq.u.alarm_send.SessionKey = CPU_TO_LE16(SessionKey);
    Rq.u.alarm_send.AlarmState = CPU_TO_LE(AlarmState);
    Rq.u.alarm_send.Addr.AddrType = (PNIO_ADDR_TYPE)CPU_TO_LE(pAddr->AddrType);
    Rq.u.alarm_send.Addr.IODataType = (PNIO_IO_TYPE)CPU_TO_LE(pAddr->IODataType);
    Rq.u.alarm_send.Addr.u.Geo.Slot = CPU_TO_LE(pAddr->u.Geo.Slot);
    Rq.u.alarm_send.Addr.u.Geo.Subslot = CPU_TO_LE(pAddr->u.Geo.Subslot);
    Rq.u.alarm_send.UserStructIdent = CPU_TO_LE16(UserStructIdent);
    Rq.u.alarm_send.UserHndl = CPU_TO_LE(UserHndl);
    memcpy(Rq.u.alarm_send.Data, pData, DataLen);
    Rq.u.alarm_send.DataLen = CPU_TO_LE(DataLen);

    Ret = SendReceiveSynch((char *)&Rq, sendLen, &expRLen);

    if(Ret != PNIO_OK)
        return Ret;

    if(LE_TO_CPU(Rq.opcode) != SCOD_ALARM_SEND) {
        TRC_OUT01(GR_STATE, LV_ERR,
            "IDevice::diag_alarm_send received unknown opcode = 0x%x",
            LE_TO_CPU(Rq.opcode));
        return PNIO_ERR_INTERNAL;
    }

    if((Ret = ConvertDAgentErrorCodeToApiErrorCode(LE_TO_CPU(Rq.agent_ret))) == PNIO_OK)
        Ret = LE_TO_CPU(Rq.resp_ret);

    return Ret;
}

/*===========================================================================
* FUNCTION : IDevice::ret_of_sub_alarm_send
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
PNIO_UINT32 IDevice::ret_of_sub_alarm_send(PNIO_UINT32 Api,
    PNIO_UINT16 ArNumber, PNIO_UINT16 SessionKey, PNIO_DEV_ADDR *pAddr,
    PNIO_UINT32 UserHndl)
{
    light_T_SYNCHD_CHNL Rq;
    PNIO_UINT32 Ret, sendLen, expRLen;

    if(!pAddr)
        return PNIO_ERR_PRM_ADD;

    sendLen = BASIC_LENGTH(Rq) + sizeof(Rq.u.ret_of_sub_alarm_send);
    expRLen = BASIC_LENGTH(Rq);

    memset(&Rq, 0, sizeof (Rq));
    Rq.blk_len = CPU_TO_LE(sendLen);
    Rq.opcode = (SYNCHD_CHNL_OP)CPU_TO_LE(SCOD_RET_SUB_ALARM_SEND);
    Rq.handle = CPU_TO_LE(m_hIodataUpdate);
    Rq.u.ret_of_sub_alarm_send.Api = CPU_TO_LE(Api);
    Rq.u.ret_of_sub_alarm_send.ArNumber = CPU_TO_LE16(ArNumber);
    Rq.u.ret_of_sub_alarm_send.SessionKey = CPU_TO_LE16(SessionKey);
    Rq.u.ret_of_sub_alarm_send.Addr.AddrType = (PNIO_ADDR_TYPE)CPU_TO_LE(pAddr->AddrType);
    Rq.u.ret_of_sub_alarm_send.Addr.IODataType = (PNIO_IO_TYPE)CPU_TO_LE(pAddr->IODataType);
    Rq.u.ret_of_sub_alarm_send.Addr.u.Geo.Slot = CPU_TO_LE(pAddr->u.Geo.Slot);
    Rq.u.ret_of_sub_alarm_send.Addr.u.Geo.Subslot = CPU_TO_LE(pAddr->u.Geo.Subslot);
    Rq.u.ret_of_sub_alarm_send.UserHndl = UserHndl;

    Ret = SendReceiveSynch((char *)&Rq, sendLen, &expRLen);

    if(Ret != PNIO_OK)
        return Ret;

    if(LE_TO_CPU(Rq.opcode) != SCOD_RET_SUB_ALARM_SEND) {
        TRC_OUT01(GR_STATE, LV_ERR,
            "IDevice::ret_of_sub_alarm_send received unknown opcode = 0x%x",
            LE_TO_CPU(Rq.opcode));
        return PNIO_ERR_INTERNAL;
    }

    if((Ret = ConvertDAgentErrorCodeToApiErrorCode(LE_TO_CPU(Rq.agent_ret))) == PNIO_OK)
        Ret = LE_TO_CPU(Rq.resp_ret);

    return Ret;
}

/*===========================================================================
* FUNCTION : IDevice::initiate_data_read
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
PNIO_UINT32 IDevice::initiate_data_read(PNIO_UINT32 ApplHandle)
{
    return IODU_intiate_data_read(m_pCpAdapter->pIODUItem);
}

/*===========================================================================
* FUNCTION : IDevice::initiate_data_write
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
PNIO_UINT32 IDevice::initiate_data_write(PNIO_UINT32 ApplHandle)
{
    return IODU_intiate_data_write(m_pCpAdapter->pIODUItem);
}

/*===========================================================================
* FUNCTION : IDevice::initiate_data_read
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
PNIO_UINT32 IDevice::initiate_data_read_ext(PNIO_UINT32 ApplHandle,
    PNIO_DEV_ADDR *pAddr, PNIO_ACCESS_ENUM AccessType)
{
    return IODU_initiate_data_read_ext(m_pCpAdapter->pIODUItem,
        pAddr, AccessType);
}

/*===========================================================================
* FUNCTION : IDevice::initiate_data_write
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
PNIO_UINT32 IDevice::initiate_data_write_ext(PNIO_UINT32 ApplHandle,
    PNIO_DEV_ADDR *pAddr, PNIO_ACCESS_ENUM AccessType)
{
    return IODU_initiate_data_write_ext(m_pCpAdapter->pIODUItem,
        pAddr, AccessType);
}

/*===========================================================================
* FUNCTION : IDevice::ProcModeInd
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
PNIO_UINT32 IDevice::ProcModeInd(CHANNEL *channel)
{
    PNIO_UINT32 Handle, Length, Ret;
    T_MODED_CHNL *pRq;

    TRC_OUT(GR_INIT, LV_FCTPUB1, "-> callback IDevice::ProcModeInd");

    Length = channel->user_pool_length_used;
    pRq = (T_MODED_CHNL *)channel->user_pool_ptr;
    if(Length > sizeof(*pRq)) {
        TRC_OUT02(GR_STATE, LV_ERR,
            "IDevice::ProcModeInd expected max %d, got unexpected length %d",
            sizeof(*pRq), Length);
        return PNIO_ERR_NO_RESOURCE;
    }

    Handle = ICommon::get_handle(this);

    switch (LE_TO_CPU(pRq->opcode)) {
    case MCOD_CBF_LED:
        if(Length != CBF_BASIC_LENGTH(*pRq) + sizeof(pRq->u.led)) {
            TRC_OUT02(GR_STATE, LV_ERR,
                "IDevice::ProcModeInd(MCOD_CBF_LED) expected %d unexpected length %d",
                CBF_BASIC_LENGTH(*pRq) + sizeof(pRq->u.led), Length);
            return PNIO_OK;
        }
        if(pRq->u.led.Start) {
            if(m_Cbfs.cbf_start_led_flash) {
                TRC_OUT01(GR_STATE, LV_FCTCLBF,
                    "-> PNIO_CBF_START_LED_FLASH call user callback() ApplHandle=0x%x", Handle);
                m_Cbfs.cbf_start_led_flash(Handle, LE_TO_CPU(pRq->u.led.Frequency));
                TRC_OUT01(GR_STATE, LV_FCTCLBF,
                    "<- PNIO_CBF_START_LED_FLASH call user callback() ends ApplHandle=0x%x", Handle);
            } else {
                TRC_OUT01(GR_STATE, LV_FCTCLBF, "PNIO_CBF_START_LED_FLASH decayed ApplHandle=0x%x",
                    Handle);
            }
        } else {
            if(m_Cbfs.cbf_stop_led_flash) {
                TRC_OUT01(GR_STATE, LV_FCTCLBF,
                    "-> PNIO_CBF_STOP_LED_FLASH call user callback() ApplHandle=0x%x", Handle);
                m_Cbfs.cbf_stop_led_flash(Handle);
                TRC_OUT01(GR_STATE, LV_FCTCLBF,
                    "<- PNIO_CBF_STOP_LED_FLASH call user callback() ends ApplHandle=0x%x", Handle);
            } else {
                TRC_OUT01(GR_STATE, LV_FCTCLBF, "PNIO_CBF_STOP_LED_FLASH decayed ApplHandle=0x%x",
                    Handle);
            }
        }
        break;

    case MCOD_CBF_INFO:
        if(Length != CBF_BASIC_LENGTH(*pRq) + sizeof(pRq->u.ar_ind)) {
            TRC_OUT02(GR_STATE, LV_ERR,
                "IDevice::ProcModeInd(MCOD_CBF_INFO) expected %d unexpected length %d",
                CBF_BASIC_LENGTH(*pRq) + sizeof(pRq->u.ar_ind), Length);
            return PNIO_OK;
        }
        switch(pRq->u.ar_ind.ArIndType) {
        case AR_INDATA:
            if(m_Cbfs.cbf_ar_indata_ind) {
                TRC_OUT03(GR_STATE, LV_FCTCLBF,
                    "-> PNIO_CBF_AR_INDATA_IND call user callback() ApplHandle=0x%x ArNumber=%d SessionKey=%d",
                    Handle, LE_TO_CPU16(pRq->u.ar_ind.ArNumber), LE_TO_CPU16(pRq->u.ar_ind.SessionKey));
                m_Cbfs.cbf_ar_indata_ind(Handle,
                    LE_TO_CPU16(pRq->u.ar_ind.ArNumber),
                    LE_TO_CPU16(pRq->u.ar_ind.SessionKey));
                TRC_OUT01(GR_STATE, LV_FCTCLBF,
                    "<- PNIO_CBF_AR_INDATA_IND call user callback() ends ApplHandle=0x%x", Handle);
            } else {
                TRC_OUT01(GR_STATE, LV_FCTCLBF, "PNIO_CBF_AR_INDATA_IND decayed ApplHandle=0x%x",
                    Handle);
            }
            break;

        case AR_ABORT:

			ar_has_slot_0_pluged = false;
			ctrl_wants_slot_0 = false;
                        slot0_modid_ctrl = 0;
                        slot0_modid_ctrl = 0;
            if(m_Cbfs.cbf_ar_abort_ind) {
                TRC_OUT03(GR_STATE, LV_FCTCLBF,
                    "-> PNIO_CBF_AR_ABORT_IND call user callback() ApplHandle=0x%x ArNumber=%d SessionKey=%d",
                    Handle, LE_TO_CPU16(pRq->u.ar_ind.ArNumber), LE_TO_CPU16(pRq->u.ar_ind.SessionKey));
                m_Cbfs.cbf_ar_abort_ind(Handle,
                    LE_TO_CPU16(pRq->u.ar_ind.ArNumber),
                    LE_TO_CPU16(pRq->u.ar_ind.SessionKey),
                    (PNIO_AR_REASON)LE_TO_CPU(pRq->u.ar_ind.ReasonCode));
                TRC_OUT01(GR_STATE, LV_FCTCLBF,
                    "<- PNIO_CBF_AR_ABORT_IND call user callback() ends ApplHandle=0x%x", Handle);
            } else {
                TRC_OUT01(GR_STATE, LV_FCTCLBF, "PNIO_CBF_AR_ABORT_IND decayed ApplHandle=0x%x",
                    Handle);
            }
            break;

        case AR_OFFLINE:
			ar_has_slot_0_pluged = false;
			ctrl_wants_slot_0 = false;
                        slot0_modid_ctrl = 0;
                        slot0_modid_ctrl = 0;
            if(m_Cbfs.cbf_ar_offline_ind) {
                TRC_OUT03(GR_STATE, LV_FCTCLBF,
                    "-> PNIO_CBF_AR_OFFLINE_IND call user callback() ApplHandle=0x%x ArNumber=%d SessionKey=%d",
                    Handle, LE_TO_CPU16(pRq->u.ar_ind.ArNumber), LE_TO_CPU16(pRq->u.ar_ind.SessionKey));
                m_Cbfs.cbf_ar_offline_ind(Handle,
                    LE_TO_CPU16(pRq->u.ar_ind.ArNumber),
                    LE_TO_CPU16(pRq->u.ar_ind.SessionKey),
                    (PNIO_AR_REASON)LE_TO_CPU(pRq->u.ar_ind.ReasonCode));
                TRC_OUT01(GR_STATE, LV_FCTCLBF,
                    "<- PNIO_CBF_AR_OFFLINE_IND call user callback() ends ApplHandle=0x%x", Handle);
            } else {
                TRC_OUT01(GR_STATE, LV_FCTCLBF, "PNIO_CBF_AR_OFFLINE_IND decayed ApplHandle=0x%x",
                    Handle);
            }
            break;

        default:
            TRC_OUT01(GR_STATE, LV_ERR, "MCOD_CBF_INFO unknown indication type %d",
                pRq->u.ar_ind.ArIndType);

        };
        break;

    case MCOD_CBF_APDU_STATUS_IND:
        if(Length != CBF_BASIC_LENGTH(*pRq) + sizeof(pRq->u.apdu_status_ind)) {
            TRC_OUT02(GR_STATE, LV_ERR,
                "IDevice::ProcModeInd(MCOD_CBF_APDU_STATUS_IND) expected %d unexpected length %d",
                CBF_BASIC_LENGTH(*pRq) + sizeof(pRq->u.apdu_status_ind), Length);
            return PNIO_OK;
        }
        if(m_Cbfs.cbf_apdu_status_ind) {
            TRC_OUT03(GR_STATE, LV_FCTCLBF,
                "-> PNIO_CBF_APDU_STATUS_IND call user callback() ApplHandle=0x%x ArNumber=%d SessionKey=%d",
                Handle, LE_TO_CPU16(pRq->u.apdu_status_ind.ArNumber),
                LE_TO_CPU16(pRq->u.apdu_status_ind.SessionKey));
            TRC_OUT01(GR_STATE, LV_FCTCLBF, "                            ApduStatus=%d",
                (PNIO_APDU_STATUS_IND)LE_TO_CPU(pRq->u.apdu_status_ind.ApduStatus));
            m_Cbfs.cbf_apdu_status_ind(Handle,
                LE_TO_CPU16(pRq->u.apdu_status_ind.ArNumber),
                LE_TO_CPU16(pRq->u.apdu_status_ind.SessionKey),
                (PNIO_APDU_STATUS_IND)LE_TO_CPU(pRq->u.apdu_status_ind.ApduStatus));
            TRC_OUT01(GR_STATE, LV_FCTCLBF,
                "<- PNIO_CBF_APDU_STATUS_IND call user callback() ends ApplHandle=0x%x", Handle);
        } else {
            TRC_OUT01(GR_STATE, LV_FCTCLBF, "PNIO_CBF_APDU_STATUS_IND decayed ApplHandle=0x%x",
                Handle);
        }
        break;

    case MCOD_CBF_STOP_REQUEST:
        if(Length != CBF_BASIC_LENGTH(*pRq)) {
            TRC_OUT02(GR_STATE, LV_ERR,
                "IDevice::ProcModeInd(MCOD_CBF_STOP_REQUEST) expected %d unexpected length %d",
                CBF_BASIC_LENGTH(*pRq), Length);
            return PNIO_OK;
        }

        if(m_Cbfs.cbf_cp_stop_req) {
            TRC_OUT01(GR_STATE, LV_FCTCLBF,
                "-> PNIO_CBF_CP_STOP_REQ call user callback() ApplHandle=0x%x", Handle);
            m_Cbfs.cbf_cp_stop_req(Handle);
            TRC_OUT01(GR_STATE, LV_FCTCLBF,
                "<- PNIO_CBF_CP_STOP_REQ call user callback() ends ApplHandle=0x%x", Handle);
        } else {
            TRC_OUT01(GR_STATE, LV_FCTCLBF, "PNIO_CBF_CP_STOP_REQ decayed ApplHandle=0x%x",
                Handle);
        }
        break;

    case MCOD_CBF_PRM_END_IND:
        if(Length != CBF_BASIC_LENGTH(*pRq) + sizeof(pRq->u.prm_end_ind)) {
            TRC_OUT02(GR_STATE, LV_ERR,
                "IDevice::ProcModeInd(MCOD_CBF_PRM_END_IND) expected %d unexpected length %d",
                CBF_BASIC_LENGTH(*pRq) + sizeof(pRq->u.prm_end_ind), Length);
            return PNIO_ERR_VALUE_LEN;
        }
        if(m_Cbfs.cbf_prm_end_ind) {
            TRC_OUT03(GR_STATE, LV_FCTCLBF,
                "-> PNIO_CBF_PRM_END_IND call user callback() ApplHandle=0x%x ArNumber=%d SessionKey=%d",
                Handle, LE_TO_CPU16(pRq->u.prm_end_ind.ArNumber),
                LE_TO_CPU16(pRq->u.prm_end_ind.SessionKey));
            TRC_OUT03(GR_STATE, LV_FCTCLBF, "                        Api=%d, SlotNr=%d, SubslotNr=%d",
                LE_TO_CPU(pRq->u.prm_end_ind.Api), LE_TO_CPU16(pRq->u.prm_end_ind.SlotNr),
                LE_TO_CPU16(pRq->u.prm_end_ind.SubslotNr));

            m_Cbfs.cbf_prm_end_ind(Handle,
                LE_TO_CPU16(pRq->u.prm_end_ind.ArNumber),
                LE_TO_CPU16(pRq->u.prm_end_ind.SessionKey),
                LE_TO_CPU(pRq->u.prm_end_ind.Api),
                LE_TO_CPU16(pRq->u.prm_end_ind.SlotNr),
                LE_TO_CPU16(pRq->u.prm_end_ind.SubslotNr));

            TRC_OUT01(GR_STATE, LV_FCTCLBF,
                "<- PNIO_CBF_PRM_END_IND call user callback() ends ApplHandle=0x%x", Handle);
        } else {
            TRC_OUT01(GR_STATE, LV_FCTCLBF, "PNIO_CBF_PRM_END_IND decayed ApplHandle=0x%x",
                Handle);
        }
        break;

    case MCOD_CBF_DEVICE_STOPPED:
        if(Length != CBF_BASIC_LENGTH(*pRq) + sizeof(pRq->u.dev_stopped)) {
            TRC_OUT02(GR_STATE, LV_ERR,
                "IDevice::ProcModeInd(MCOD_CBF_DEVICE_STOPPED) expected %d unexpected length %d",
                CBF_BASIC_LENGTH(*pRq) + sizeof(pRq->u.dev_stopped), Length);
            return PNIO_OK;
        }

        if(m_Cbfs.cbf_device_stopped) {
            TRC_OUT01(GR_STATE, LV_FCTCLBF,
                "-> PNIO_CBF_DEVICE_STOPPED call user callback() ApplHandle=0x%x", Handle);
            m_Cbfs.cbf_device_stopped(Handle,
                LE_TO_CPU(pRq->u.dev_stopped.Reserved));
            TRC_OUT01(GR_STATE, LV_FCTCLBF,
                "<- PNIO_CBF_DEVICE_STOPPED call user callback() ends ApplHandle=0x%x", Handle);
        } else {
            TRC_OUT01(GR_STATE, LV_FCTCLBF, "PNIO_CBF_DEVICE_STOPPED decayed ApplHandle=0x%x",
                Handle);
        }
        break;

    case MCOD_CBF_PULL_PLUG_CNF:
        if(Length != CBF_BASIC_LENGTH(*pRq) + sizeof(pRq->u.pull_plug_cnf)) {
            TRC_OUT02(GR_STATE, LV_ERR,
                "IDevice::ProcModeInd(MCOD_CBF_PULL_PLUG_CNF) expected %d unexpected length %d",
                CBF_BASIC_LENGTH(*pRq) + sizeof(pRq->u.pull_plug_cnf), Length);
            return PNIO_OK;
        }

        if(m_Cbfs.cbf_pull_plug_conf) {
            PNIO_DEV_ADDR Addr;
            Addr.AddrType = (PNIO_ADDR_TYPE)LE_TO_CPU(pRq->u.pull_plug_cnf.Addr.AddrType);
            Addr.IODataType = (PNIO_IO_TYPE)LE_TO_CPU(pRq->u.pull_plug_cnf.Addr.IODataType);
            Addr.u.Geo.Slot = LE_TO_CPU(pRq->u.pull_plug_cnf.Addr.u.Geo.Slot);
            Addr.u.Geo.Subslot = LE_TO_CPU(pRq->u.pull_plug_cnf.Addr.u.Geo.Subslot);

            TRC_IF_ON_EXPR(GR_STATE, LV_FCTCLBF,
                OSTREAM trcos1;
                trcos1 << showbase << hex;
                trcos1 << "-> PNIO_CBF_PULL_PLUG_CNF call user callback() ApplHandle=" << Handle;
                trcint_ShowPullPlugAction(trcos1, (PNIO_DEV_ACTION)LE_TO_CPU(pRq->u.pull_plug_cnf.Action), "Action");
                trcos1 << " Slot=" << Addr.u.Geo.Slot;
                trcos1 << " Subslot=" << Addr.u.Geo.Subslot;
                trcos1 << " ErrorCode=" << LE_TO_CPU(pRq->u.pull_plug_cnf.ErrorCode);
                trcos1 << ends;
                TRC_OUT_OBJECT(GR_STATE, LV_FCTCLBF, trcos1);
                );

            m_Cbfs.cbf_pull_plug_conf(Handle,
                LE_TO_CPU(pRq->u.pull_plug_cnf.Api),
                (PNIO_DEV_ACTION)LE_TO_CPU(pRq->u.pull_plug_cnf.Action),
                &Addr,
                LE_TO_CPU(pRq->u.pull_plug_cnf.ErrorCode));
            TRC_OUT01(GR_STATE, LV_FCTCLBF,
                "<- PNIO_CBF_PULL_PLUG_CNF call user callback() ends ApplHandle=0x%x", Handle);
        } else {
            TRC_OUT01(GR_STATE, LV_FCTCLBF, "PNIO_CBF_PULL_PLUG_CNF decayed ApplHandle=0x%x",
                Handle);
        }
        break;
    case MCOD_SET_IO_VALIDITY:
      if(Length != CBF_BASIC_LENGTH(*pRq) + sizeof(pRq->u.set_io_validity)) {
          TRC_OUT02(GR_STATE, LV_ERR,
              "IDevice::ProcModeInd(MCOD_SET_IO_VALIDITY) expected %d unexpected length %d",
              CBF_BASIC_LENGTH(*pRq) + sizeof(pRq->u.set_io_validity), Length);
          return PNIO_OK;
      } else {
        PNIO_DEV_ADDR addr;
        addr.AddrType = PNIO_ADDR_GEO;
        addr.IODataType = (PNIO_IO_TYPE)LE_TO_CPU(pRq->u.set_io_validity.Addr.IODataType);
        addr.u.Geo.Slot = LE_TO_CPU(pRq->u.set_io_validity.Addr.u.Geo.Slot);
        addr.u.Geo.Subslot = LE_TO_CPU(pRq->u.set_io_validity.Addr.u.Geo.Subslot);

        IODU_VALIDITY iodu_valid = LE_TO_CPU(pRq->u.set_io_validity.Validity) == 1 ? ITEM_VALID : ITEM_INVALID;
        TRC_OUT04(GR_STATE, LV_INFO,
            "IDevice::ProcModeInd SET_IO_VALIDITY s=%i ss=%i io=%i(0-IN,1-OUT) valid=%i(0-INVAL,1-VAL)",
                   addr.u.Geo.Slot, addr.u.Geo.Subslot, addr.IODataType, iodu_valid);

        /* the VALIDITY flag in KRAMIOTLB will be set to new value in firmware */

        /* build IODU SubmoduleList again, to consider new possible KRAMIOTLB items */
        /* and to update existing item, because items will be copied to host lockal memory */
        Ret = IODU_update_subslot_tbl_adv(m_pCpAdapter->pIODUItem,0,0,KRAMIOTLB_IsItemValidWithoutIORouter);
        if(Ret != PNIO_OK){
          TRC_OUT01(GR_STATE, LV_ERR,
              "IDevice::ProcModeInd(MCOD_SET_IO_VALIDITY) IODU_update_subslot_tbl_adv ret 0x%x", Ret);
        }

        /* only if item got ITEM_INVALID his IOXS is to be set to BAD */
        if(iodu_valid == ITEM_INVALID){
          Ret = IODU_set_io_validity_data(m_pCpAdapter->pIODUItem, &addr, iodu_valid);
          if(Ret != PNIO_OK){
            TRC_OUT01(GR_STATE, LV_ERR,
                "IDevice::ProcModeInd(MCOD_SET_IO_VALIDITY) IODU_set_io_validity_data ret 0x%x", Ret);
          }
        }
      }

      break;

    default:
        TRC_OUT01(GR_STATE, LV_ERR,
            "IDevice::ProcModeInd received unknown opcode = 0x%x", LE_TO_CPU(pRq->opcode));
        DPR_ASSERT(0);
        break;
    }

    TRC_OUT(GR_MGT, LV_FCTPUB1, "<- IDevice::ProcModeInd done");

    return PNIO_OK;
}

/*===========================================================================
* FUNCTION : IDevice::ProcDataRec
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
PNIO_UINT32 IDevice::ProcDataRec(CHANNEL *channel)
{
    PNIO_UINT32 Ret, Handle, Length;
    T_DATAREC_CHNL *pRq;
    PNIO_DEV_ADDR Addr;
    PNIO_ERR_STAT Err;
    T_DATAREC_CHNL *pRespRq;
    PNIO_UINT32 sendLen, bufLen;

    memset(&Err, 0, sizeof(Err));
    TRC_OUT(GR_INIT, LV_FCTPUB1, "-> callback IDevice::ProcDataRec");

    Length = channel->user_pool_length_used;
    pRq = (T_DATAREC_CHNL *)channel->user_pool_ptr;
    pRespRq = (T_DATAREC_CHNL *)channel->send_pool_ptr;
    if(Length > sizeof(*pRq)) {
        TRC_OUT02(GR_STATE, LV_ERR,
            "IDevice::ProcDataRec expected max %d, got unexpected length %d",
            sizeof(*pRq), Length);
        return PNIO_ERR_NO_RESOURCE;
    }

    Handle = ICommon::get_handle(this);
    Ret = PNIO_OK;

    switch (LE_TO_CPU(pRq->opcode)) {
    case DCOD_CBF_REC_READ:
        if(Length != CBF_BASIC_LENGTH(*pRq) + sizeof(pRq->u.rec_read)) {
            TRC_OUT02(GR_STATE, LV_ERR,
                "IDevice::ProcDataRec expected %d unexpected length %d",
                CBF_BASIC_LENGTH(*pRq) + sizeof(pRq->u.rec_read), Length);
            return PNIO_ERR_VALUE_LEN;
        }

        if(m_Cbfs.cbf_rec_read) {
            bufLen = MIN(LE_TO_CPU(pRq->u.rec_read.BufLen), PNIOD_DREC_MAX_SIZE);
            Addr.AddrType = (PNIO_ADDR_TYPE)LE_TO_CPU(pRq->u.rec_read.Addr.AddrType);
            Addr.IODataType = (PNIO_IO_TYPE)LE_TO_CPU(pRq->u.rec_read.Addr.IODataType);
            Addr.u.Geo.Slot = LE_TO_CPU(pRq->u.rec_read.Addr.u.Geo.Slot);
            Addr.u.Geo.Subslot = LE_TO_CPU(pRq->u.rec_read.Addr.u.Geo.Subslot);

            TRC_OUT01(GR_STATE, LV_FCTCLBF,
                "-> PNIO_CBF_REC_READ call user callback() ApplHandle=0x%x", Handle);
            m_Cbfs.cbf_rec_read(Handle,
                LE_TO_CPU(pRq->u.rec_read.Api),
                LE_TO_CPU16(pRq->u.rec_read.ArNumber),
                LE_TO_CPU16(pRq->u.rec_read.SessionKey),
                LE_TO_CPU(pRq->u.rec_read.SequenceNum),
                &Addr,
                LE_TO_CPU(pRq->u.rec_read.RecordIndex),
                &bufLen,
                pRespRq->u.rec_read_resp.Buffer,
                &Err);

            TRC_OUT01(GR_STATE, LV_FCTCLBF,
                "<- PNIO_CBF_REC_READ call user callback() ends ApplHandle=0x%x", Handle);
        } else {
            TRC_OUT01(GR_STATE, LV_FCTCLBF, "PNIO_CBF_REC_READ decayed ApplHandle=0x%x",
                Handle);
            bufLen = 0;
            Err.ErrCode = 0x81;
            Err.ErrDecode = 0x80;
            Err.ErrCode1 = 9;
            Err.ErrCode2 = 0;
        }

        sendLen = CBF_BASIC_LENGTH(*pRespRq) +
            sizeof(pRespRq->u.rec_read_resp.SequenceNum) +
            sizeof(pRespRq->u.rec_read_resp.PnioState) +
            sizeof(pRespRq->u.rec_read_resp.BufLen) +
            bufLen;

        pRespRq->blk_len = CPU_TO_LE(sendLen);
        pRespRq->opcode = (DATARECD_CHNL_OP)CPU_TO_LE(DCOD_CBF_REC_READ_RESPONSE);
        pRespRq->handle = CPU_TO_LE(m_hIodataUpdate);
        pRespRq->u.rec_read_resp.SequenceNum = pRq->u.rec_read.SequenceNum;
        pRespRq->u.rec_read_resp.PnioState.ErrCode = Err.ErrCode;
        pRespRq->u.rec_read_resp.PnioState.ErrDecode = Err.ErrDecode;
        pRespRq->u.rec_read_resp.PnioState.ErrCode1 = Err.ErrCode1;
        pRespRq->u.rec_read_resp.PnioState.ErrCode2 = Err.ErrCode2;
        pRespRq->u.rec_read_resp.PnioState.AddValue1 = CPU_TO_LE16(Err.AddValue1);
        pRespRq->u.rec_read_resp.PnioState.AddValue2 = CPU_TO_LE16(Err.AddValue2);
        pRespRq->u.rec_read_resp.BufLen = CPU_TO_LE(bufLen);

        Ret = Send(DATAREC, (char *)pRespRq, sendLen);
        if(PNIO_OK != Ret) {
            TRC_OUT01(GR_CHNL, LV_ERR,
                "IDevice::ProcDataRec(DCOD_CBF_REC_READ_RESPONSE) fail to send response %d bytes",
                sendLen);
        }
        break;

    case DCOD_CBF_REC_WRITE:
        if(Length > CBF_BASIC_LENGTH(*pRq) + sizeof(pRq->u.rec_write)) {
            TRC_OUT02(GR_STATE, LV_ERR,
                "IDevice::ProcDataRec expected %d unexpected length %d",
                CBF_BASIC_LENGTH(*pRq) + sizeof(pRq->u.rec_write), Length);
            return PNIO_ERR_VALUE_LEN;
        }

        if(m_Cbfs.cbf_rec_write) {
            bufLen = MIN(LE_TO_CPU(pRq->u.rec_write.BufLen), PNIOD_DREC_MAX_SIZE);
            Addr.AddrType = (PNIO_ADDR_TYPE)LE_TO_CPU(pRq->u.rec_write.Addr.AddrType);
            Addr.IODataType = (PNIO_IO_TYPE)LE_TO_CPU(pRq->u.rec_write.Addr.IODataType);
            Addr.u.Geo.Slot = LE_TO_CPU(pRq->u.rec_write.Addr.u.Geo.Slot);
            Addr.u.Geo.Subslot = LE_TO_CPU(pRq->u.rec_write.Addr.u.Geo.Subslot);

            TRC_OUT01(GR_STATE, LV_FCTCLBF,
                "-> PNIO_CBF_REC_WRITE call user callback() ApplHandle=0x%x", Handle);
            m_Cbfs.cbf_rec_write(Handle,
                LE_TO_CPU(pRq->u.rec_write.Api),
                LE_TO_CPU16(pRq->u.rec_write.ArNumber),
                LE_TO_CPU16(pRq->u.rec_write.SessionKey),
                LE_TO_CPU(pRq->u.rec_write.SequenceNum),
                &Addr,
                LE_TO_CPU(pRq->u.rec_write.RecordIndex),
                &bufLen,
                pRq->u.rec_write.Buffer,
                &Err);

            TRC_OUT01(GR_STATE, LV_FCTCLBF,
                "<- PNIO_CBF_REC_WRITE call user callback() ends ApplHandle=0x%x", Handle);
        } else {
            TRC_OUT01(GR_STATE, LV_FCTCLBF, "PNIO_CBF_REC_WRITE decayed ApplHandle=0x%x",
                Handle);
            bufLen = 0;
            Err.ErrCode = 0x81;
            Err.ErrDecode = 0x80;
            Err.ErrCode1 = 9;
            Err.ErrCode2 = 0;
        }

        sendLen = CBF_BASIC_LENGTH(*pRespRq) + sizeof(pRespRq->u.rec_write_resp);

        pRespRq->blk_len = CPU_TO_LE(sendLen);
        pRespRq->opcode = (DATARECD_CHNL_OP)CPU_TO_LE(DCOD_CBF_REC_WRITE_RESPONSE);
        pRespRq->handle = CPU_TO_LE(m_hIodataUpdate);
        pRespRq->u.rec_write_resp.SequenceNum = pRq->u.rec_write.SequenceNum;
        pRespRq->u.rec_write_resp.PnioState.ErrCode = Err.ErrCode;
        pRespRq->u.rec_write_resp.PnioState.ErrDecode = Err.ErrDecode;
        pRespRq->u.rec_write_resp.PnioState.ErrCode1 = Err.ErrCode1;
        pRespRq->u.rec_write_resp.PnioState.ErrCode2 = Err.ErrCode2;
        pRespRq->u.rec_write_resp.PnioState.AddValue1 = CPU_TO_LE16(Err.AddValue1);
        pRespRq->u.rec_write_resp.PnioState.AddValue2 = CPU_TO_LE16(Err.AddValue2);
        pRespRq->u.rec_write_resp.BufLen = CPU_TO_LE(bufLen);

        Ret = Send(DATAREC, (char *)pRespRq, sendLen);
        if(PNIO_OK != Ret) {
            TRC_OUT01(GR_CHNL, LV_ERR,
                "IDevice::ProcDataRec(DCOD_CBF_REC_WRITE_RESPONSE) fail to send response %d bytes",
                sendLen);
        }
        break;

    case DCOD_CBF_ALARM_ACK:
        if(Length != CBF_BASIC_LENGTH(*pRq) + sizeof(pRq->u.req_done)) {
            TRC_OUT02(GR_STATE, LV_ERR,
                "IDevice::ProcDataRec(DCOD_CBF_ALARM_ACK) expected %d unexpected length %d",
                CBF_BASIC_LENGTH(*pRq) + sizeof(pRq->u.req_done), Length);
            return PNIO_OK;
        }
        if(m_Cbfs.cbf_alarm_done) {
            TRC_OUT01(GR_STATE, LV_FCTCLBF,
                "-> PNIO_CBF_REQ_DONE call user callback() ApplHandle=0x%x", Handle);
            m_Cbfs.cbf_alarm_done(Handle,
                LE_TO_CPU(pRq->u.req_done.UserHndl),
                LE_TO_CPU(pRq->u.req_done.Status), &Err);
            TRC_OUT01(GR_STATE, LV_FCTCLBF,
                "<- PNIO_CBF_REQ_DONE call user callback() ends ApplHandle=0x%x", Handle);
        } else {
            TRC_OUT01(GR_STATE, LV_FCTCLBF, "PNIO_CBF_REQ_DONE decayed ApplHandle=0x%x", Handle);
        }

        sendLen = CBF_BASIC_LENGTH(*pRespRq) + sizeof(pRespRq->u.req_done_resp);

        pRespRq->blk_len = CPU_TO_LE(sendLen);
        pRespRq->opcode = (DATARECD_CHNL_OP)CPU_TO_LE(DCOD_CBF_ALARM_ACK_RESPONSE);
        pRespRq->handle = CPU_TO_LE(m_hIodataUpdate);
        pRespRq->u.req_done_resp.UserHndl = pRq->u.req_done.UserHndl;
        pRespRq->u.req_done_resp.Status = pRq->u.req_done.Status;
        pRespRq->u.req_done_resp.PnioState.ErrCode = Err.ErrCode;
        pRespRq->u.req_done_resp.PnioState.ErrDecode = Err.ErrDecode;
        pRespRq->u.req_done_resp.PnioState.ErrCode1 = Err.ErrCode1;
        pRespRq->u.req_done_resp.PnioState.ErrCode2 = Err.ErrCode2;
        pRespRq->u.req_done_resp.PnioState.AddValue1 = CPU_TO_LE16(Err.AddValue1);
        pRespRq->u.req_done_resp.PnioState.AddValue2 = CPU_TO_LE16(Err.AddValue2);

        Ret = Send(DATAREC, (char *)pRespRq, sendLen);
        if(PNIO_OK != Ret) {
            TRC_OUT01(GR_CHNL, LV_ERR,
                "IDevice::ProcDataRec(DCOD_CBF_ALARM_ACK_RESPONSE) fail to send response %d bytes",
                sendLen);
        }
        break;

    case DCOD_CBF_CHECK_IND:
    {
        PNIO_UINT32 ModIdent, SubIdent;
        PNIO_UINT16 ModState, SubState;

        if(Length != CBF_BASIC_LENGTH(*pRq) + sizeof(pRq->u.check_ind)) {
            TRC_OUT02(GR_STATE, LV_ERR,
                "IDevice::ProcDataRec(DCOD_CBF_CHECK_IND) expected %d unexpected length %d",
                CBF_BASIC_LENGTH(*pRq) + sizeof(pRq->u.check_ind), Length);
            return PNIO_OK;
        }
        if(m_Cbfs.cbf_check_ind) {

            Addr.AddrType = (PNIO_ADDR_TYPE)LE_TO_CPU(pRq->u.check_ind.Addr.AddrType);
            Addr.IODataType = (PNIO_IO_TYPE)LE_TO_CPU(pRq->u.check_ind.Addr.IODataType);
            Addr.u.Geo.Slot = LE_TO_CPU(pRq->u.check_ind.Addr.u.Geo.Slot);
            Addr.u.Geo.Subslot = LE_TO_CPU(pRq->u.check_ind.Addr.u.Geo.Subslot);
            ModIdent = LE_TO_CPU(pRq->u.check_ind.ModIdent);
            ModState = LE_TO_CPU16(pRq->u.check_ind.ModState);
            SubIdent = LE_TO_CPU(pRq->u.check_ind.SubIdent);
            SubState = LE_TO_CPU16(pRq->u.check_ind.SubState);

            TRC_OUT01(GR_STATE, LV_FCTCLBF,
                "-> PNIO_CBF_CHECK_IND call user callback() ApplHandle=0x%x", Handle);
            m_Cbfs.cbf_check_ind(Handle,
                LE_TO_CPU(pRq->u.check_ind.Api),
                LE_TO_CPU16(pRq->u.check_ind.ArNumber),
                LE_TO_CPU16(pRq->u.check_ind.SessionKey),
                &Addr, &ModIdent, &ModState, &SubIdent, &SubState);
            TRC_OUT01(GR_STATE, LV_FCTCLBF,
                "<- PNIO_CBF_CHECK_IND call user callback() ends ApplHandle=0x%x",
                Handle);
            TRC_OUT04(GR_STATE, LV_FCTCLBF,
                 "    ModIdent=0x%x ModState=0x%x SubIdent=0x%x SubState=0x%x",
                 ModIdent, ModState, SubIdent, SubState);

        } else {
            TRC_OUT01(GR_STATE, LV_FCTCLBF, "PNIO_CBF_CHECK_IND decayed ApplHandle=0x%x",
                Handle);
        }

        sendLen = CBF_BASIC_LENGTH(*pRespRq) + sizeof(pRespRq->u.check_ind_resp);

        pRespRq->blk_len = CPU_TO_LE(sendLen);
        pRespRq->opcode = (DATARECD_CHNL_OP)CPU_TO_LE(DCOD_CBF_CHECK_IND_RESPONSE);
        pRespRq->handle = CPU_TO_LE(m_hIodataUpdate);
        pRespRq->u.check_ind_resp.Api = pRq->u.check_ind.Api;
        pRespRq->u.check_ind_resp.ArNumber = pRq->u.check_ind.ArNumber;
        pRespRq->u.check_ind_resp.SessionKey = pRq->u.check_ind.SessionKey;
        pRespRq->u.check_ind_resp.ModIdent = CPU_TO_LE(ModIdent);
        pRespRq->u.check_ind_resp.ModState = CPU_TO_LE16(ModState);
        pRespRq->u.check_ind_resp.SubIdent = CPU_TO_LE(SubIdent);
        pRespRq->u.check_ind_resp.SubState = CPU_TO_LE16(SubState);

        Ret = Send(DATAREC, (char *)pRespRq, sendLen);
        if(PNIO_OK != Ret) {
            TRC_OUT01(GR_CHNL, LV_ERR,
                "IDevice::ProcDataRec(DCOD_CBF_ALARM_ACK_RESPONSE) fail to send response %d bytes",
                sendLen);
        }
        break;

    }
    default:
        TRC_OUT01(GR_STATE, LV_ERR,
            "IDevice::ProcDataRec received unknown opcode = 0x%x", LE_TO_CPU(pRq->opcode));
        DPR_ASSERT(0);
        break;
    }

    TRC_OUT(GR_MGT, LV_FCTPUB1, "<- IDevice::ProcDataRec done");

    return Ret;
}

/*
    deserialize stream received from FW and reconstruct a PNIO_AR_TYPE struct, with respect to
    pointer size on given platform
*/
int RecodeStructAr(fw_PNIO_AR_TYPE *srcArType, PNIO_AR_TYPE *trgArType, PNIO_UINT32 ArDataLen,
    KRAMIOTLB_Item *pAvailSubslots, PNIO_UINT32 *pAvailSubslotsNum)
{

    fw_PNIO_IOCR_TYPE    *pcIocrType;
    fw_PNIO_MODULE_TYPE  *pcModuleType;
    fw_PNIO_SUBMOD_TYPE  *pcSubmodType;

    PNIO_IOCR_TYPE       *pIocrType;
    PNIO_MODULE_TYPE     *pModuleType;
    PNIO_SUBMOD_TYPE     *pSubmodType;
    PNIO_UINT32           numSubmodType, curAvailSubslots = 0;

    PNIO_UINT32           i, j;

    trgArType->pNextAr = NULL;
    trgArType->ArNumber = LE_TO_CPU16(srcArType->ArNumber);
    trgArType->SessionKey = LE_TO_CPU16(srcArType->SessionKey);
    trgArType->NumOfIocr = LE_TO_CPU(srcArType->NumOfIocr);
    trgArType->pIoCrList = NULL;
    trgArType->NumOfMod = LE_TO_CPU(srcArType->NumOfMod);
    trgArType->pModList = NULL;

    TRC_IF_ON_EXPR(GR_STATE, LV_TIMECRIT,
        OSTREAM trcos;
        trcos << showbase << hex;
        trcos << "  pNextAr(ignored)=" << srcArType->pNextAr;
        trcos << "  ArNumber=" << trgArType->ArNumber;
        trcos << ", SessionKey=" << trgArType->SessionKey;
        trcos << ", NumOfIocr=" << trgArType->NumOfIocr;
        trcos << ", NumOfMod=" << trgArType->NumOfMod;
        trcos << ends;
        TRC_OUT_OBJECT(GR_INIT, LV_TIMECRIT, trcos);
    );

    pcIocrType = (fw_PNIO_IOCR_TYPE *)((char*)srcArType + sizeof(*srcArType));
    pIocrType = (PNIO_IOCR_TYPE *)((char*)trgArType + sizeof(*trgArType));
    trgArType->pIoCrList = pIocrType;

    for(i = 0; i < trgArType->NumOfIocr; i++) {
        pIocrType[i].CycleTime = LE_TO_CPU(pcIocrType[i].CycleTime);
        pIocrType[i].Direction = (PNIO_IOCR_TYPE_ENUM)LE_TO_CPU(pcIocrType[i].Direction);
        pIocrType[i].IOCRProperties = (PNIO_IOCR_PROP_ENUM)LE_TO_CPU(pcIocrType[i].IOCRProperties);
        pIocrType[i].SendClock = LE_TO_CPU(pcIocrType[i].SendClock);
        pIocrType[i].ReductioFactor = LE_TO_CPU(pcIocrType[i].ReductioFactor);
        pIocrType[i].Phase = LE_TO_CPU(pcIocrType[i].Phase);
        pIocrType[i].NumOfIoCs = LE_TO_CPU(pcIocrType[i].NumOfIoCs);
        /* FIXME correct this number, now this information is missing */
        pIocrType[i].NumOfIoCs = 0;
        pIocrType[i].pIoCsList = NULL;

        memcpy(pIocrType[i].reserved, pcIocrType[i].reserved, sizeof(pIocrType[i].reserved));

        TRC_IF_ON_EXPR(GR_STATE, LV_TIMECRIT,
            OSTREAM trcos;
            trcos << "  i=" << i;
            trcos << showbase << hex;
            trcos << ", offset to pIocrType[i]=" << ((char *)&pcIocrType[i] - (char *)srcArType);
            trcos << ", CycleTime=" << pIocrType[i].CycleTime;
            trcos << ", Direction=" << pIocrType[i].Direction;
            trcos << ", IOCRProperties=" << pIocrType[i].IOCRProperties;
            trcos << ", pIocrType[i].SendClock=" << pIocrType[i].SendClock;
            trcos << ", pIocrType[i].ReductioFactor=" << pIocrType[i].ReductioFactor;
            trcos << ", pIocrType[i].Phase=" << pIocrType[i].Phase;
            trcos << ", pIocrType[i].NumOfIoCs=" << pIocrType[i].NumOfIoCs
                << " (" << LE_TO_CPU(pcIocrType[i].NumOfIoCs) <<")";
/*          trcos << ", offset to pIocrType[i].pIoCsList=" <<
                ((char *)&pcIocrType[i].pIoCsList - (char *)srcArType);*/

            trcos << ends;
            TRC_OUT_OBJECT(GR_INIT, LV_TIMECRIT, trcos);
        );
    }

    /* find out and correct first MODULE_TYPE pointer */
    pcModuleType = (fw_PNIO_MODULE_TYPE *)(&pcIocrType[trgArType->NumOfIocr]);
    pModuleType = (PNIO_MODULE_TYPE *)(&pIocrType[trgArType->NumOfIocr]);
    trgArType->pModList = pModuleType;
    for(numSubmodType = 0, i = 0; i < trgArType->NumOfMod; i++) {
        pModuleType[i].Api = LE_TO_CPU(pcModuleType[i].Api);
        pModuleType[i].SlotNum = LE_TO_CPU(pcModuleType[i].SlotNum);
        pModuleType[i].NumOfSubmod = LE_TO_CPU(pcModuleType[i].NumOfSubmod);
        pModuleType[i].ModProperties = LE_TO_CPU(pcModuleType[i].ModProperties);
        pModuleType[i].ModIdent = LE_TO_CPU(pcModuleType[i].ModIdent);
        pModuleType[i].pSubList = NULL;

        if (pModuleType[i].SlotNum == 0)
        {
           IDevice::ctrl_wants_slot_0 = true;
           IDevice::slot0_modid_ctrl = LE_TO_CPU(pcModuleType[i].ModIdent);
        }
        else if(pModuleType[i].SlotNum == 1)
        {
           IDevice::slot1_modid_ctrl = LE_TO_CPU(pcModuleType[i].ModIdent);
        }


        TRC_OUT03(GR_STATE, LV_TIMECRIT,
            "Module(%d): Api %d, SlotNum %d",
            i,
            trgArType->pModList[i].Api,
            trgArType->pModList[i].SlotNum);
        TRC_OUT04(GR_STATE, LV_TIMECRIT,
            "Module(%d): NumOfSubmod %d, ModProperties %d, ModIdent 0x%x",
            i,
            trgArType->pModList[i].NumOfSubmod,
            trgArType->pModList[i].ModProperties,
            trgArType->pModList[i].ModIdent);

        numSubmodType += pModuleType[i].NumOfSubmod;
    }

    /* find out first SUBMOD_TYPE pointer */
    pcSubmodType = (fw_PNIO_SUBMOD_TYPE *)(&pcModuleType[trgArType->NumOfMod]);
    pSubmodType = (PNIO_SUBMOD_TYPE *)(&pModuleType[trgArType->NumOfMod]);

    for(i = 0; i < numSubmodType; i++) {
        pSubmodType[i].SlotNum = LE_TO_CPU(pcSubmodType[i].SlotNum);
        pSubmodType[i].SubSlotNum = LE_TO_CPU(pcSubmodType[i].SubSlotNum);
        pSubmodType[i].SubMProperties = LE_TO_CPU(pcSubmodType[i].SubMProperties);
        pSubmodType[i].SubMIdent = LE_TO_CPU(pcSubmodType[i].SubMIdent);
        pSubmodType[i].InDatLen = LE_TO_CPU(pcSubmodType[i].InDatLen);
        pSubmodType[i].InIopsLen = LE_TO_CPU(pcSubmodType[i].InIopsLen);
        pSubmodType[i].InIocsLen = LE_TO_CPU(pcSubmodType[i].InIocsLen);
        pSubmodType[i].OutDatLen = LE_TO_CPU(pcSubmodType[i].OutDatLen);
        pSubmodType[i].OutIopsLen = LE_TO_CPU(pcSubmodType[i].OutIopsLen);
        pSubmodType[i].OutIocsLen = LE_TO_CPU(pcSubmodType[i].OutIocsLen);
        memcpy(pSubmodType[i].reserved, pcSubmodType[i].reserved, sizeof(pSubmodType[i].reserved));

        TRC_OUT05(GR_STATE, LV_TIMECRIT,
            "Submodule(%d): SlotNum %d, SubSlotNum %d, SubMProperties %d, SubMIdent 0x%x",
            i,
            pSubmodType[i].SlotNum,
            pSubmodType[i].SubSlotNum,
            pSubmodType[i].SubMProperties,
            pSubmodType[i].SubMIdent);
        TRC_OUT04(GR_STATE, LV_TIMECRIT,
            "Submodule(%d): InDatLen %d, InIopsLen %d, InIocsLen %d",
            i,
            pSubmodType[i].InDatLen,
            pSubmodType[i].InIopsLen,
            pSubmodType[i].InIocsLen);
        TRC_OUT04(GR_STATE, LV_TIMECRIT,
            "Submodule(%d): OutDatLen %d, OutIopsLen %d, OutIocsLen %d",
            i,
            pSubmodType[i].OutDatLen,
            pSubmodType[i].OutIopsLen,
            pSubmodType[i].OutIocsLen);

        if(pAvailSubslotsNum) {
            if(pSubmodType[i].InIopsLen) {
                if(pAvailSubslots && curAvailSubslots < *pAvailSubslotsNum) {
                    pAvailSubslots[curAvailSubslots].slot_nr = pSubmodType[i].SlotNum;
                    pAvailSubslots[curAvailSubslots].subslot_nr = pSubmodType[i].SubSlotNum;
                    pAvailSubslots[curAvailSubslots].io_out_type = KRAMIOTLB_IO_OUT;
                }
                curAvailSubslots++;
            }
            if(pSubmodType[i].OutIopsLen) {
                if(pAvailSubslots && curAvailSubslots < *pAvailSubslotsNum) {
                    pAvailSubslots[curAvailSubslots].slot_nr = pSubmodType[i].SlotNum;
                    pAvailSubslots[curAvailSubslots].subslot_nr = pSubmodType[i].SubSlotNum;
                    pAvailSubslots[curAvailSubslots].io_out_type = KRAMIOTLB_IO_IN;
                }
                curAvailSubslots++;
            }
        }
    }

    if(pAvailSubslotsNum)
        *pAvailSubslotsNum = curAvailSubslots;

    /* correct SUBMOD_TYPE and RECORD_TYPE pointers */
    for(i = j = 0; i < trgArType->NumOfMod; i++) {
        pModuleType[i].pSubList = &pSubmodType[j];
        j += pModuleType[i].NumOfSubmod;
    }

    return 0;
}

/*===========================================================================
* FUNCTION : IDevice::ProcAlarm
*----------------------------------------------------------------------------
* PURPOSE  : callback function for Alarm channel
*----------------------------------------------------------------------------
* RETURNS  :
*----------------------------------------------------------------------------
* INPUTS   :
* OUTPUS   :
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
PNIO_UINT32 IDevice::ProcAlarm(CHANNEL *channel)
{
    PNIO_UINT32 Handle, Length, hostLength, Ret;
    T_ALARM_CHNL *pRq;

    TRC_OUT(GR_ALARM, LV_FCTPUB1, "-> callback IDevice::ProcAlarm");

    Length = channel->user_pool_length_used;
    pRq = (T_ALARM_CHNL *)channel->user_pool_ptr;

    Handle = ICommon::get_handle(this);

    switch(LE_TO_CPU(pRq->opcode)) {
#if 0
    case ACOD_CBF_DEV_ALARM_IND:
        if(Length > CBF_BASIC_LENGTH(*pRq) + sizeof(pRq->u.alarm_ind)) {
          TRC_OUT02(GR_ALARM, LV_ERR,
              "IDevice::ProcAlarm expected %d unexpected length %d",
              CBF_BASIC_LENGTH(*pRq) + sizeof(pRq->u.alarm_ind), Length);
          return PNIO_ERR_VALUE_LEN;
        }
        if(m_Cbfs.cbf_alarm_ind) {
            TRC_OUT01(GR_ALARM, LV_FCTCLBF,
                "-> PNIO_CBF_DEV_ALARM_IND call user callback() ApplHandle=0x%x", Handle);
            m_Cbfs.cbf_alarm_ind(Handle, &pRq->u.alarm_ind.al_blk);
            TRC_OUT01(GR_ALARM, LV_FCTCLBF,
                "<- PNIO_CBF_DEV_ALARM_IND call user callback() ends ApplHandle=0x%x", Handle);
        } else {
            TRC_OUT01(GR_STATE, LV_FCTCLBF, "PNIO_CBF_DEV_ALARM_IND decayed ApplHandle=0x%x",
                Handle);
        }
        break;
#endif
    case ACOD_CBF_AR_CHECK_IND:
        if(Length < CBF_BASIC_LENGTH(*pRq) + sizeof(pRq->u.ar_check_ind)) {
          TRC_OUT02(GR_ALARM, LV_ERR,
              "IDevice::ProcAlarm expected %d unexpected length %d",
              CBF_BASIC_LENGTH(*pRq) + sizeof(pRq->u.ar_check_ind), Length);
          return PNIO_ERR_VALUE_LEN;
        }
        if(m_Cbfs.cbf_ar_check_ind) {
            /* see RecodeStructAr */
            t_cbfd_ar_check_ind *ar_check_ind = (t_cbfd_ar_check_ind *)((char *)pRq +
                CBF_BASIC_LENGTH(*pRq));
            fw_PNIO_AR_TYPE *src = (fw_PNIO_AR_TYPE *)(((char *)&ar_check_ind->ArDataLen) +
                sizeof(ar_check_ind->ArDataLen));

            hostLength = Length;
            if(sizeof(void *) > sizeof(PNIO_UINT32)) {
                hostLength += (sizeof(PNIO_AR_TYPE) - sizeof(fw_PNIO_AR_TYPE));
                hostLength += ((sizeof(PNIO_MODULE_TYPE) - sizeof(fw_PNIO_MODULE_TYPE))
                    * LE_TO_CPU(src->NumOfMod));
                hostLength += ((sizeof(PNIO_IOCR_TYPE) - sizeof(fw_PNIO_IOCR_TYPE))
                    * LE_TO_CPU(src->NumOfIocr));

                TRC_OUT02(GR_STATE, LV_FCTPUB2, "Length %u, korrigiert %u", Length, hostLength);
            }
            char *tmp = new char[hostLength];

            if(tmp) {
                ar_check_ind->HostIp = LE_TO_CPU(ar_check_ind->HostIp);
                ar_check_ind->ArType = LE_TO_CPU16(ar_check_ind->ArType);
                ar_check_ind->ArUUID.TimeLow = LE_TO_CPU(ar_check_ind->ArUUID.TimeLow);
                ar_check_ind->ArUUID.TimeMid = LE_TO_CPU16(ar_check_ind->ArUUID.TimeMid);
                ar_check_ind->ArUUID.TimeHiAndVersion = LE_TO_CPU16(ar_check_ind->ArUUID.TimeHiAndVersion);
                ar_check_ind->ArProperties = LE_TO_CPU(ar_check_ind->ArProperties);
                ar_check_ind->CmiObjUUID.TimeLow = LE_TO_CPU(ar_check_ind->CmiObjUUID.TimeLow);
                ar_check_ind->CmiObjUUID.TimeMid = LE_TO_CPU16(ar_check_ind->CmiObjUUID.TimeMid);
                ar_check_ind->CmiObjUUID.TimeHiAndVersion = LE_TO_CPU16(ar_check_ind->CmiObjUUID.TimeHiAndVersion);
                ar_check_ind->CmiStationNameLenght = LE_TO_CPU16(ar_check_ind->CmiStationNameLenght);
                ar_check_ind->ArDataLen = LE_TO_CPU(ar_check_ind->ArDataLen);

                TRC_IF_ON_EXPR(GR_ALARM, LV_FCTPUB2,
                    OSTREAM trcos;
                    trcos << showbase << hex;
                    trcos << "  HostIp=" << ar_check_ind->HostIp;
                    trcos << ", ArType=" << ar_check_ind->ArType;
                    trcos << ", ArUUID.TimeLow=" << ar_check_ind->ArUUID.TimeLow;
                    trcos << ", ArUUID.TimeMid=" << ar_check_ind->ArUUID.TimeMid;
                    trcos << ", ArUUID.TimeHiAndVersion=" << ar_check_ind->ArUUID.TimeHiAndVersion;
                    trcos << ", ArUUID.ClockSeqHiAndReserved=" << ar_check_ind->ArUUID.ClockSeqHiAndReserved;
                    trcos << ", ArUUID.ClockSeqLow=" << ar_check_ind->ArUUID.ClockSeqLow;
                    trcos << ", ArUUID.Node=";
                    trcint_ShowData(trcos, sizeof(ar_check_ind->ArUUID.Node),
                        ar_check_ind->ArUUID.Node);
                    trcos << ", ArProperties=" << ar_check_ind->ArProperties;
                    trcos << ends;
                    TRC_OUT_OBJECT(GR_ALARM, LV_FCTPUB2, trcos);
                    );

                TRC_IF_ON_EXPR(GR_ALARM, LV_FCTPUB2,
                    OSTREAM trcos1;
                    trcos1 << showbase << hex;
                    trcos1 << "  CmiObjUUID.TimeLow=" << ar_check_ind->CmiObjUUID.TimeLow;
                    trcos1 << ", CmiObjUUID.TimeMid=" << ar_check_ind->CmiObjUUID.TimeMid;
                    trcos1 << ", CmiObjUUID.TimeHiAndVersion=" << ar_check_ind->CmiObjUUID.TimeHiAndVersion;
                    trcos1 << ", CmiObjUUID.ClockSeqHiAndReserved=" << ar_check_ind->CmiObjUUID.ClockSeqHiAndReserved;
                    trcos1 << ", CmiObjUUID.ClockSeqLow=" << ar_check_ind->CmiObjUUID.ClockSeqLow;
                    trcos1 << ", CmiObjUUID.Node=";
                    trcint_ShowData(trcos1, sizeof(ar_check_ind->CmiObjUUID.Node),
                        ar_check_ind->CmiObjUUID.Node);

                    trcos1 << ", CmiStationNameLenght=" << ar_check_ind->CmiStationNameLenght;
                    trcos1 << ", ArDataLen=" << ar_check_ind->ArDataLen;
                    trcos1 << ends;
                    TRC_OUT_OBJECT(GR_ALARM, LV_FCTPUB2, trcos1);
                    );

                RecodeStructAr(src, (PNIO_AR_TYPE *)tmp, ar_check_ind->ArDataLen, NULL, NULL);

                TRC_OUT01(GR_ALARM, LV_FCTCLBF,
                    "-> PNIO_CBF_AR_CHECK_IND call user callback() ApplHandle=0x%x", Handle);
                m_Cbfs.cbf_ar_check_ind(Handle,
                    ar_check_ind->HostIp,
                    ar_check_ind->ArType,
                    ar_check_ind->ArUUID,
                    ar_check_ind->ArProperties,
                    ar_check_ind->CmiObjUUID,
                    ar_check_ind->CmiStationNameLenght,
                    ar_check_ind->CmiStationName,
                    (PNIO_AR_TYPE*)tmp);
                TRC_OUT01(GR_ALARM, LV_FCTCLBF,
                    "<- PNIO_CBF_AR_CHECK_IND call user callback() ends ApplHandle=0x%x", Handle);
                delete [] tmp;
            } else {
                TRC_OUT01(GR_ALARM, LV_ERR,
                    "-> PNIO_CBF_AR_CHECK_IND ENOMEM ApplHandle=0x%x", Handle);
            }
        } else {
            TRC_OUT01(GR_ALARM, LV_FCTCLBF, "PNIO_CBF_AR_CHECK_IND decayed ApplHandle=0x%x",
                Handle);
        }
        break;

    case ACOD_CBF_AR_INFO_IND:
        if(Length < CBF_BASIC_LENGTH(*pRq) + sizeof(pRq->u.ar_info_ind)) {
          TRC_OUT02(GR_ALARM, LV_ERR,
              "IDevice::ProcAlarm expected %d unexpected length %d",
              CBF_BASIC_LENGTH(*pRq) + sizeof(pRq->u.ar_info_ind), Length);
          return PNIO_ERR_VALUE_LEN;
        }
        if(m_Cbfs.cbf_ar_info_ind) {
            /* see RecodeStructAr */
            t_cbfd_ar_info_ind *ar_info_ind = (t_cbfd_ar_info_ind *)((char *)pRq +
                CBF_BASIC_LENGTH(*pRq));
            fw_PNIO_AR_TYPE *src = (fw_PNIO_AR_TYPE *)(((char *)&ar_info_ind->ArDataLen) +
                sizeof(ar_info_ind->ArDataLen));

            hostLength = Length;
            if(sizeof(void *) > sizeof(PNIO_UINT32)) {
                hostLength += (sizeof(PNIO_AR_TYPE) - sizeof(fw_PNIO_AR_TYPE));
                hostLength += ((sizeof(PNIO_MODULE_TYPE) - sizeof(fw_PNIO_MODULE_TYPE))
                    * LE_TO_CPU(src->NumOfMod));
                hostLength += ((sizeof(PNIO_IOCR_TYPE) - sizeof(fw_PNIO_IOCR_TYPE))
                    * LE_TO_CPU(src->NumOfIocr));

                TRC_OUT02(GR_STATE, LV_FCTPUB2, "Length %u, korrigiert %u", Length, hostLength);
            }
            char *tmp = (char *)malloc( sizeof(char) * hostLength );

            if(tmp)  {
                ar_info_ind->BlkLen = LE_TO_CPU(ar_info_ind->BlkLen);
                ar_info_ind->ArNumber = LE_TO_CPU16(ar_info_ind->ArNumber);
                ar_info_ind->SessionKey = LE_TO_CPU16(ar_info_ind->SessionKey);
                ar_info_ind->ArDataLen = LE_TO_CPU(ar_info_ind->ArDataLen);

                TRC_IF_ON_EXPR(GR_ALARM, LV_FCTPUB2,
                    OSTREAM trcos;
                    trcos << showbase << hex;
                    trcos << " BlkLen=" << ar_info_ind->BlkLen;
                    trcos << " ArDataLen=" << ar_info_ind->ArDataLen;
                    trcos << ends;
                    TRC_OUT_OBJECT(GR_ALARM, LV_FCTPUB2, trcos);
                    );

                PNIO_UINT32   AvailableSubslotsNum = 256;
                KRAMIOTLB_Item *pAvailableSubslots = (KRAMIOTLB_Item *)malloc( sizeof(KRAMIOTLB_Item) *AvailableSubslotsNum);

                RecodeStructAr(src, (PNIO_AR_TYPE *)tmp, ar_info_ind->ArDataLen,
                    pAvailableSubslots, &AvailableSubslotsNum);

                TRC_OUT01(GR_ALARM, LV_INFO,
                    "RecodeStructAr() AvailableSubslotsNum = %d", AvailableSubslotsNum);
                /*for(PNIO_UINT32 i=0; i<AvailableSubslotsNum; i++)
                {
                  TRC_OUT02(GR_ALARM, LV_INFO,
                      "RecodeStructAr() Available(belong to user application or splitted but not PURE) Slot %d Subslot %d",
                         pAvailableSubslots[i].slot_nr, pAvailableSubslots[i].subslot_nr);
                }*/

                DPR_ASSERT(AvailableSubslotsNum < 256); /* increase AvailableSubslotsNum */

                /* first trigger table update in iodata */
                Ret = IODU_update_subslot_tbl_adv(m_pCpAdapter->pIODUItem,
                    pAvailableSubslots, AvailableSubslotsNum,
                    KRAMIOTLB_IsItemValidDeviceHost);
                if(PNIO_OK != Ret) {
                    TRC_OUT02(GR_ALARM, LV_ERR,
                        "IODU_update_subslot_tbl(0x%x) returns 0x%x", m_hIodataUpdate, Ret);
                } else {
                    TRC_OUT02(GR_ALARM, LV_FCTCLBF,
                        "IODU_update_subslot_tbl(0x%x) returns 0x%x", m_hIodataUpdate, Ret);
                }

                if(pAvailableSubslots)
                    free(pAvailableSubslots);

                /* copy output APDU status bytes from DPRAM memory to DMA memory, to prevent garbage */
                ICommon *pThis = this;
                IODATA_adr_info &adr_info = m_pCpAdapter->pIODUItem->adr_info;
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

                TRC_OUT03(GR_ALARM, LV_FCTCLBF,
                    "-> PNIO_CBF_AR_INFO_IND(ArNumber %d, SessionKey %d) call user callback() ApplHandle=0x%x", 
                      Handle, ar_info_ind->ArNumber, ar_info_ind->SessionKey);
                m_Cbfs.cbf_ar_info_ind(Handle,
                    ar_info_ind->ArNumber,
                    ar_info_ind->SessionKey,
                    (PNIO_AR_TYPE*)tmp);
                TRC_OUT01(GR_ALARM, LV_FCTCLBF,
                    "<- PNIO_CBF_AR_INFO_IND call user callback() ends ApplHandle=0x%x", Handle);
                free(tmp);
            } else {
                TRC_OUT02(GR_ALARM, LV_ERR,
                    "-> PNIO_CBF_AR_INFO_IND ApplHandle=0x%x can't allocate %d Bytes", Handle, sizeof(char) * hostLength);
            }
        } else {
            TRC_OUT01(GR_ALARM, LV_FCTCLBF, "PNIO_CBF_AR_INFO_IND decayed ApplHandle=0x%x", Handle);
        }
        break;

    default:
        TRC_OUT01(GR_ALARM, LV_ERR,
            "IDevice::ProcAlarm received unknown opcode = 0x%x", LE_TO_CPU(pRq->opcode));
        DPR_ASSERT(0);
        break;
    }

    TRC_OUT(GR_ALARM, LV_FCTPUB1, "<- IDevice::ProcAlarm done");

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
PNIO_UINT32 IDevice::TestBlockSendReceive(PNIO_UINT8 *Buffer, PNIO_UINT32 Length)
{
    light_T_SYNCHD_CHNL Rq;
    PNIO_UINT32 Ret, sendLen, expRLen;

    if(NULL == Buffer)
        return PNIO_ERR_PRM_BUF;

    if(Length > sizeof(Rq.u.test_ping.Data))
        return PNIO_ERR_PRM_LEN;

    sendLen = BASIC_LENGTH(Rq) + sizeof(Rq.u.test_ping)
        - sizeof(Rq.u.test_ping.Data) + Length;
    expRLen = BASIC_LENGTH(Rq);

    memset(&Rq, 0, sizeof (Rq));
    Rq.blk_len = CPU_TO_LE(sendLen);
    Rq.opcode = (SYNCHD_CHNL_OP)CPU_TO_LE(SCOD_TEST_PING);
    Rq.handle = CPU_TO_LE(m_hIodataUpdate);
    memcpy(Rq.u.test_ping.Data, Buffer, Length);
    Rq.u.test_ping.DataLen = CPU_TO_LE(Length);

    Ret = SendReceiveSynch((char *)&Rq, sendLen, &expRLen);

    if(Ret != PNIO_OK)
        return Ret;

    if(LE_TO_CPU(Rq.opcode) != SCOD_TEST_PING) {
        TRC_OUT01(GR_STATE, LV_ERR,
            "IDevice::TestBlockSendReceive received unknown opcode = 0x%x",
            LE_TO_CPU(Rq.opcode));
        return PNIO_ERR_INTERNAL;
    }

    if((Ret = ConvertDAgentErrorCodeToApiErrorCode(LE_TO_CPU(Rq.agent_ret))) == PNIO_OK)
        Ret = LE_TO_CPU(Rq.resp_ret);

    if(PNIO_OK == Ret)
        memcpy(Buffer, Rq.u.test_ping.Data, MIN(Length, LE_TO_CPU(Rq.u.test_ping.DataLen)));

    return Ret;
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
PNIO_UINT32 IDevice::RegisterStartOp(void)
{
    PNIO_UINT32 Ret = 0;
    light_T_SYNCHD_CHNL Rq;

    PNIO_UINT32 sendLen = BASIC_LENGTH(Rq);
    PNIO_UINT32 expRLen = BASIC_LENGTH(Rq);

    memset(&Rq, 0, sizeof(Rq));
    Rq.blk_len = CPU_TO_LE(sendLen);
    Rq.opcode = (SYNCHD_CHNL_OP)CPU_TO_LE(SCOD_REGISTER_START_OP_CBF);
    Rq.handle = CPU_TO_LE(m_hIodataUpdate);

    Ret = SendReceiveSynch((char *)&Rq, sendLen, &expRLen);

    if(Ret != PNIO_OK)
        return Ret;

    if(LE_TO_CPU(Rq.opcode) != SCOD_REGISTER_START_OP_CBF) {
        TRC_OUT01(GR_STATE, LV_ERR,
            "IDevice::RegisterStartOp received unknown opcode = 0x%x",
            LE_TO_CPU(Rq.opcode));
        return PNIO_ERR_INTERNAL;
    }

    if((Ret = ConvertDAgentErrorCodeToApiErrorCode(LE_TO_CPU(Rq.agent_ret))) == PNIO_OK)
        Ret = LE_TO_CPU(Rq.resp_ret);

    return Ret;
}

/*===========================================================================
* FUNCTION : ConvertDAgentErrorCodeToApiErrorCode
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
PNIO_UINT32 ConvertDAgentErrorCodeToApiErrorCode(PNIO_UINT32 InErr)
{
    PNIO_UINT32 RteErr;

    switch(InErr) {
    case  PNIO_DAGET_RET_OK:
        RteErr = PNIO_OK;
        break;
    case PNIO_DAGENT_RET_OPEN_NOT_ALLOWED:
        RteErr = PNIO_ERR_CONFIG_IN_UPDATE;
        break;
    case PNIO_DAGENT_RET_REBOOT_AFTER_EXCEPTION:
        RteErr = PNIO_ERR_AFTER_EXCEPTION;
        break;
    case PNIO_DAGENT_RET_ERROR_PARAM:
        RteErr = PNIO_ERR_WRONG_RQB_LEN;
        break;
    default:
        RteErr = PNIO_ERR_INTERNAL;
        break;
    }

    if(InErr != PNIO_DAGET_RET_OK)
        TRC_OUT02(GR_MGT, LV_ERR,
            "ConvertDAgentErrorCodeToApiErrorCode(0x%x) ret=0x%x", InErr, RteErr);
    return RteErr;
}
