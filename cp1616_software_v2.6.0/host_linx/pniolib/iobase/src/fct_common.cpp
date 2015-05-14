/****************************************************************************
*  Copyright (C) SIEMENS CORP., 2013 All rights reserved.
*****************************************************************************
* FILE NAME    : fct_common.cpp
* ---------------------------------------------------------------------------
* DESCRIPTION  : base class for PNIO controller and PNIO device.
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
#include "wd_dpr_msg.h"

#define MAX_APPS 10
static ICommon* icp[MAX_APPS];

#if defined(PRN_TRACE_ON) || defined(WDB_TRACE_ON)
unsigned long g_pnio_trc_dest = 0;   /* trace output destination. 0: no output, 1: stdout printf */
unsigned long g_pnio_trc_group= 0;
unsigned long g_pnio_trc_depth= 0;
#endif /* _TRACE_ON */


static class StaticTraceInit {
public:
    StaticTraceInit(void) {
        TRC_INIT_FROM_FILE("pniotrace.conf");

        IODU_Init();
        InitCriticalSections();

        memset(icp, 0, sizeof(ICommon*) * MAX_APPS);
    };

    ~StaticTraceInit(void) {
        DestroyCriticalSections();
        TRC_DEINIT();
    };
} __static__init;

typedef union {
  DPR_MSG_HDR rqb;
  WD_TIMEOUT_DPRMGTCHNL_RB wd_rqb;
} ATTR_PACKED PNIO_DPR_MGT_CHNL_RQB;


static void SetCycleInfo(DPR_ADAPTER *pCpAdp, PNIO_CYCLE_INFO *pInfo)
{
    pInfo->ClockCount = LE_TO_CPU(*(PNIO_UINT32 *)(pCpAdp->pErtecSwiBase+HP_CLOCK_CNT_VAL));
    pInfo->CountSinceCycleStart = LE_TO_CPU(*(PNIO_UINT32 *)(pCpAdp->pErtecSwiBase+HP_CYCLE_TIME_VAL));
    pInfo->CycleCount = LE_TO_CPU(*(PNIO_UINT16 *)(pCpAdp->pErtecSwiBase+HP_CYCLE_CNT_VAL));
}

#define INDEX2HANDLE(x) (x + 1)
#define HANDLE2INDEX(x) (x - 1)
#define INVALIDHANDLE   (0)
#define LASTVALIDHANDLE (MAX_APPS)
PNIO_UINT32 add_instance(ICommon* ptr) {
    PNIO_UINT32 i;
    for(i = 0; i < MAX_APPS; ++i) {
        if(!icp[i]) {
            icp[i] = ptr;
            return INDEX2HANDLE(i);
        }
    }
    return INVALIDHANDLE;
}

void remove_instance(PNIO_UINT32 handle) {
    if(INVALIDHANDLE == handle ||
        handle > LASTVALIDHANDLE ||
         !icp[HANDLE2INDEX(handle)]) {

       TRC_OUT01(GR_INIT, LV_ERR, "remove_instance:: invalid handle %d", handle);
       DPR_ASSERT(0);
    }
    else {
      icp[HANDLE2INDEX(handle)] = NULL;
    }
}

ICommon *find_instance(PNIO_UINT32 handle) {
    if(INVALIDHANDLE == handle || handle > LASTVALIDHANDLE)
        return NULL;

    return icp[HANDLE2INDEX(handle)];
}

#define MAX_CP_WATCHDOGS 10

static struct CP_WATCHDOG {
    PNIO_CBF_APPL_WATCHDOG cbfApplWatchdog;
    unsigned long                  timeout;
    unsigned long               watchdogId;
    unsigned long                  user_id;
    DPR_DRV_HANDLE                    file;
    DPR_THREAD_HANDLE            th_reader;
    int                        stop_thread;
    PNIO_UINT32                    CpIndex;
    char                    *user_pool_ptr;
    unsigned long         user_pool_length;
} CpWds[MAX_CP_WATCHDOGS];

ICommon::ICommon(void):
    m_pCbf_StartOp_Ind(NULL),
    m_pCbf_OpFault_Ind(NULL),
    m_pCbf_NewCycle_Ind(NULL),
#ifdef IO_ROUTER
    m_pCbf_TransferWD(NULL),
#endif /* IO_ROUTER */
    m_semSendReceiveSynchWaiterAvailable(false),
    m_pCpAdapter(NULL),
    m_bClosePending(false),
    m_bEmergencyClose(false),
    m_hIodataUpdate(0),
    m_hInstanceHandle(0),
    m_uOrderId(0),
    m_pHostIOTlb(0),
    m_cycleCountEntity(0)
{
    m_hInstanceHandle = add_instance(this);
    CP_cycle_stat(NULL,-1); /* reset first measure */
}

ICommon::~ICommon(void)
{
    remove_instance(m_hInstanceHandle);
}

PNIO_UINT32 ICommon::get_handle(ICommon *ptr)
{
    if(ptr)
      return ptr->m_hInstanceHandle;
    else{
      DPR_ASSERT(ptr);
      return 0xFFFFFFFF;
    }
}

ICommon *ICommon::get_instance(PNIO_UINT32 ApplHandle)
{
    return find_instance(ApplHandle);
}

/*===========================================================================
* FUNCTION : ICommon::InitCp
*----------------------------------------------------------------------------
* PURPOSE  : this function Initalizes PNIOLib <-> DPRLib
*----------------------------------------------------------------------------
* RETURNS  : PNIO_OK on success; else PNIO_ERR_INTERNAL
*----------------------------------------------------------------------------
* INPUTS   : -
* OUTPUS   : -
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
PNIO_UINT32 ICommon::InitCp(PNIO_UINT32 cpIndex)
{
    PNIO_UINT32 Ret;
    char tmp[36];
    struct t_register_app tmp_app;
    struct t_dma_address tmp_dma_addr;
    TRC_OUT01(GR_INIT, LV_FCTINT, "-> InitCp %d", cpIndex);

    if(cpIndex < 1) {
        TRC_OUT(GR_INIT, LV_ERR, "first cp index is 1");
        return PNIO_ERR_PRM_CP_ID;
    }

    if(DPR_SEM_CREATE(m_semSendReceiveSynch)) {
        TRC_OUT(GR_INIT, LV_ERR,
            "DPR_SEM_CREATE(semSendReceiveSynch) failed");
        return PNIO_ERR_INTERNAL;
    }

    m_pCpAdapter = new DPR_ADAPTER;
    memset(m_pCpAdapter, 0, sizeof(DPR_ADAPTER));
    m_pCpAdapter->icommon_ptr = this;
    m_pCpAdapter->CpIndex = cpIndex;
    DPR_MUTEX_CREATE_UNLOCKED(m_pCpAdapter->synch_channel_mutex);

    snprintf(tmp, sizeof(tmp)-1, DPR_CONTROL_INTERFACE, DRIVER_IDX(cpIndex));
    m_pCpAdapter->fd_control = DPR_DRV_OPEN(tmp);

    if(!m_pCpAdapter->fd_control) {
        TRC_OUT01(GR_INIT, LV_ERR, "ERROR openning %s", tmp);
        Ret = PNIO_ERR_PRM_CP_ID;
        goto InitCp_create_adapter_fail;
    }

    memset(&tmp_app, 0, sizeof(tmp_app));
    if(!(DPR_DRV_IOCTL(m_pCpAdapter->fd_control, CP16XX_IOC_OAPP,
        &tmp_app, sizeof(tmp_app), sizeof(tmp_app)))) {
        m_pCpAdapter->user_id = tmp_app.user_id;
        TRC_OUT01(GR_INIT, LV_INFO, "register application success, user_id 0x%x",
            m_pCpAdapter->user_id);
    } else {
        TRC_OUT01(GR_INIT, LV_ERR, "ioctl register application failed, error '%s'",
            DPR_STRERROR());
        if(EMFILE == errno)
            Ret = PNIO_ERR_MAX_REACHED;
        else
            Ret = PNIO_ERR_CREATE_INSTANCE;
        goto InitCp_create_adapter_ioctl_fail;
    }

    m_pCpAdapter->pKramTlb = (char *)DPR_DRV_MMAP(m_pCpAdapter->fd_control,
        MMAP_OFFSET_DPRAM + OFFSET_KRAMTLB, SIZE_KRAMTLB, m_pCpAdapter->user_id);
    if(!m_pCpAdapter->pKramTlb) {
        TRC_OUT01(GR_INIT, LV_ERR, "mmap pKramTlb failed, error '%s'", DPR_STRERROR());
        Ret = PNIO_ERR_NO_RESOURCE;
        goto InitCp_create_adapter_mmap_fail;
    }
#ifdef IO_ROUTER
    m_pCpAdapter->pShMemIocTable = (char *)DPR_DRV_MMAP(m_pCpAdapter->fd_control,
        MMAP_OFFSET_DPRAM + OFFSET_IOCTABLE, SIZE_IOCTABLE, m_pCpAdapter->user_id);
    if(!m_pCpAdapter->pShMemIocTable) {
        TRC_OUT01(GR_INIT, LV_ERR, "mmap pShMemIocTab failed, error '%s'", DPR_STRERROR());
        Ret = PNIO_ERR_NO_RESOURCE;
        goto InitCp_create_adapter_mmap_fail;
    }
#endif /* IO_ROUTER */

    m_pCpAdapter->pErtecSwiBase = (char *)DPR_DRV_MMAP(m_pCpAdapter->fd_control,
        MMAP_OFFSET_IRTE + OFFSET_ERTEC_BASE, SIZE_ERTEC_BASE, m_pCpAdapter->user_id);
    if(!m_pCpAdapter->pErtecSwiBase) {
        TRC_OUT01(GR_INIT, LV_ERR, "mmap pErtecSwiBase failed, error '%s'", DPR_STRERROR());
        Ret = PNIO_ERR_NO_RESOURCE;
        goto InitCp_create_adapter_mmap_fail;
    }

    m_pCpAdapter->pErtecIOTotal = (char *)DPR_DRV_MMAP(m_pCpAdapter->fd_control,
        MMAP_OFFSET_IRTE + OFFSET_IO_TOTAL, SIZE_IO_TOTAL, m_pCpAdapter->user_id);
    if(!m_pCpAdapter->pErtecIOTotal) {
        TRC_OUT01(GR_INIT, LV_ERR, "mmap pErtecIOTotal failed, error '%s'", DPR_STRERROR());
        Ret = PNIO_ERR_NO_RESOURCE;
        goto InitCp_create_adapter_mmap_fail;
    }

    m_pCpAdapter->pIRTDMAImage = (char *)DPR_DRV_MMAP(m_pCpAdapter->fd_control,
        MMAP_OFFSET_DMA + OFFSET_DMA_IMAGE, SIZE_DMA_IMAGE, m_pCpAdapter->user_id);

    if(!m_pCpAdapter->pIRTDMAImage) {
        TRC_OUT01(GR_INIT, LV_ERR, "mmap pIRTDMAImage failed, error '%s'", DPR_STRERROR());
        Ret = PNIO_ERR_NO_RESOURCE;
        goto InitCp_create_adapter_mmap_fail;
    }

    memset(&tmp_dma_addr, 0, sizeof(tmp_dma_addr));
    if(!(DPR_DRV_IOCTL(m_pCpAdapter->fd_control, CP16XX_IOC_GET_PNIO_DMA_RANGE,
        &tmp_dma_addr, sizeof(tmp_dma_addr), sizeof(tmp_dma_addr)))) {
        m_pCpAdapter->pIRTDMAPhysAddr = tmp_dma_addr.dma_address;
        m_pCpAdapter->IRTDMALen       = tmp_dma_addr.dma_size;
        TRC_OUT02(GR_INIT, LV_INFO, "get pnio dma range success, addr 0x%x, len 0x%x",
            m_pCpAdapter->pIRTDMAPhysAddr, m_pCpAdapter->IRTDMALen);
    } else {
        TRC_OUT01(GR_INIT, LV_ERR, "ioctl get pnio dma range failed, error '%s'",
            DPR_STRERROR());
        Ret = PNIO_ERR_CREATE_INSTANCE;
        goto InitCp_create_adapter_mmap_fail;
    }

    snprintf(tmp, sizeof(tmp)-1, DPR_SYNC_INTERFACE, DRIVER_IDX(cpIndex));
    if(PNIO_OK != OpenDprChannel(tmp, m_pCpAdapter, SYNCH)) {
        TRC_OUT01(GR_INIT, LV_ERR, "ERROR creating %s", tmp);
        Ret = PNIO_ERR_NO_RESOURCE;
        goto InitCp_create_adapter_fail_sync;
    }

    snprintf(tmp, sizeof(tmp)-1, DPR_ALARM_INTERFACE, DRIVER_IDX(cpIndex));
    if(PNIO_OK != OpenDprChannel(tmp, m_pCpAdapter, ALARM)) {
        TRC_OUT01(GR_INIT, LV_ERR, "ERROR creating %s", tmp);
        Ret = PNIO_ERR_NO_RESOURCE;
        goto InitCp_create_adapter_fail_alarm;
    }

    snprintf(tmp, sizeof(tmp)-1, DPR_MODEIND_INTERFACE, DRIVER_IDX(cpIndex));
    if(PNIO_OK != OpenDprChannel(tmp, m_pCpAdapter, MODIND)) {
        TRC_OUT01(GR_INIT, LV_ERR, "ERROR creating %s", tmp);
        Ret = PNIO_ERR_NO_RESOURCE;
        goto InitCp_create_adapter_fail_modind;
    }

    snprintf(tmp, sizeof(tmp)-1, DPR_RECORDS_INTERFACE, DRIVER_IDX(cpIndex));
    if(PNIO_OK != OpenDprChannel(tmp, m_pCpAdapter, DATAREC)) {
        TRC_OUT01(GR_INIT, LV_ERR, "ERROR creating %s", tmp);
        Ret = PNIO_ERR_NO_RESOURCE;
        goto InitCp_create_adapter_fail_datarec;
    }

    TRC_OUT(GR_INIT, LV_FCTINT, "<- end InitCp");
    return PNIO_OK;

InitCp_create_adapter_fail_datarec:
    CloseDprChannel(&m_pCpAdapter->chnls[MODIND]);

InitCp_create_adapter_fail_modind:
    CloseDprChannel(&m_pCpAdapter->chnls[ALARM]);

InitCp_create_adapter_fail_alarm:
     CloseDprChannel(&m_pCpAdapter->chnls[SYNCH]);

InitCp_create_adapter_fail_sync:

InitCp_create_adapter_mmap_fail:
    if(m_pCpAdapter->pKramTlb)
        DPR_DRV_MUNMAP(m_pCpAdapter->fd_control,
            m_pCpAdapter->pKramTlb, SIZE_KRAMTLB, m_pCpAdapter->user_id);
#ifdef IO_ROUTER
    if(m_pCpAdapter->pShMemIocTable)
        DPR_DRV_MUNMAP(m_pCpAdapter->fd_control,
            m_pCpAdapter->pShMemIocTable, SIZE_IOCTABLE, m_pCpAdapter->user_id);
#endif /* IO_ROUTER */
    if(m_pCpAdapter->pErtecSwiBase)
        DPR_DRV_MUNMAP(m_pCpAdapter->fd_control,
            m_pCpAdapter->pErtecSwiBase, SIZE_ERTEC_BASE, m_pCpAdapter->user_id);
    if(m_pCpAdapter->pErtecIOTotal)
        DPR_DRV_MUNMAP(m_pCpAdapter->fd_control,
            m_pCpAdapter->pErtecIOTotal, SIZE_IO_TOTAL, m_pCpAdapter->user_id);
    if(m_pCpAdapter->pIRTDMAImage)
        DPR_DRV_MUNMAP(m_pCpAdapter->fd_control,
            m_pCpAdapter->pIRTDMAImage, SIZE_DMA_IMAGE, m_pCpAdapter->user_id);

    DPR_DRV_IOCTL(m_pCpAdapter->fd_control, CP16XX_IOC_CAPP, &tmp_app, sizeof(tmp_app), 0);

InitCp_create_adapter_ioctl_fail:
    DPR_DRV_CLOSE(m_pCpAdapter->fd_control);

InitCp_create_adapter_fail:
    DPR_MUTEX_DESTROY(m_pCpAdapter->synch_channel_mutex);
    delete m_pCpAdapter;
    m_pCpAdapter = NULL;

    return Ret;
}

/*===========================================================================
* FUNCTION : UninitCp
*----------------------------------------------------------------------------
* PURPOSE  : this function Uninitalizes PNIOLib
*----------------------------------------------------------------------------
* RETURNS  : PNIO_OK on success; else PNIO_ERR_INTERNAL
*----------------------------------------------------------------------------
* INPUTS   : -
* OUTPUS   : -
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
PNIO_UINT32 ICommon::UninitCp(void)
{
    struct t_register_app tmp_app;
    int ret;

    TRC_OUT(GR_INIT, LV_FCTINT, "-> UninitCp");

#ifndef IRT_NODMA
  #ifdef IRT_DMA_VIA_HOST
    uninit_irt_dma_ranges();
    TRC_OUT(GR_INIT, LV_FCTINT, "  uninit_irt_dma_ranges()");
  #endif /* IRT_DMA_VIA_HOST */
#endif /* IRT_NODMA */

    /* warn SyncChannel */
    m_bEmergencyClose = true;
    DPR_SEM_POST(m_semSendReceiveSynch);

    /* pause for a second to let SyncChannel to complete */
    DPR_TASK_DELAY(1000);

    /* close channels */
    CloseDprChannel(&m_pCpAdapter->chnls[DATAREC]);
    CloseDprChannel(&m_pCpAdapter->chnls[MODIND]);
    CloseDprChannel(&m_pCpAdapter->chnls[ALARM]);
    CloseDprChannel(&m_pCpAdapter->chnls[SYNCH]);

    DPR_DRV_MUNMAP(m_pCpAdapter->fd_control,
        m_pCpAdapter->pKramTlb, SIZE_KRAMTLB, m_pCpAdapter->user_id);
    DPR_DRV_MUNMAP(m_pCpAdapter->fd_control,
        m_pCpAdapter->pShMemIocTable, SIZE_IOCTABLE, m_pCpAdapter->user_id);
    DPR_DRV_MUNMAP(m_pCpAdapter->fd_control,
        m_pCpAdapter->pErtecSwiBase, SIZE_ERTEC_BASE, m_pCpAdapter->user_id);
    DPR_DRV_MUNMAP(m_pCpAdapter->fd_control,
        m_pCpAdapter->pErtecIOTotal, SIZE_IO_TOTAL, m_pCpAdapter->user_id);
    DPR_DRV_MUNMAP(m_pCpAdapter->fd_control,
        m_pCpAdapter->pIRTDMAImage, SIZE_DMA_IMAGE, m_pCpAdapter->user_id);

    TRC_OUT01(GR_INIT, LV_FCTINT, "ioctl unregister application, user_id 0x%x",
        m_pCpAdapter->user_id);

    tmp_app.user_id = m_pCpAdapter->user_id;
    if((ret = DPR_DRV_IOCTL(m_pCpAdapter->fd_control, CP16XX_IOC_CAPP,
        &tmp_app, sizeof(tmp_app), 0)) < 0) {
         TRC_OUT01(GR_INIT, LV_ERR, "ioctl unregister application failed, error '%s'",
            DPR_STRERROR());
    }

    DPR_DRV_CLOSE(m_pCpAdapter->fd_control);

    DPR_SEM_DESTROY(m_semSendReceiveSynch);
    DPR_MUTEX_DESTROY(m_pCpAdapter->synch_channel_mutex);
    delete(m_pCpAdapter);
    m_pCpAdapter = NULL;

    TRC_OUT(GR_INIT, LV_FCTINT, "<- end UninitCp");

    return PNIO_OK;
}

/*===========================================================================
* FUNCTION : Send
*----------------------------------------------------------------------------
* PURPOSE  : to send messages in Chnl
*----------------------------------------------------------------------------
* RETURNS  : PNIO_ERR_INTERNAL on error ; else  PNIO_OK
*----------------------------------------------------------------------------
* INPUTS   : pRq - send and receive Request
*            sendLen - send length
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
PNIO_UINT32 ICommon::Send(CHANNELS Chnl, char *pRq, PNIO_UINT32 sendLen)
{
    size_t ret;

    TRC_OUT01(GR_STATE, LV_FCTCLBF, "ICommon::Send sendLen %d", sendLen);

    TRC_OUT02(GR_CHNL, LV_DATA, "Send sends %d bytes data in channel %d:",
        sendLen, Chnl);
    TRC_OUTD(GR_CHNL, LV_DATA, (unsigned char *)pRq, sendLen);

    ret = DPR_DRV_WRITE(m_pCpAdapter->chnls[Chnl].file, pRq, sendLen);

    if(ret != sendLen) {
        TRC_OUT03(GR_CHNL, LV_ERR,
            "Send failed, written %Zu bytes from %d, error '%s'",
            ret, sendLen, DPR_STRERROR());
        return PNIO_ERR_NO_FW_COMMUNICATION;
    }

    return PNIO_OK;
}

/*===========================================================================
* FUNCTION : SendReceiveSynch
*----------------------------------------------------------------------------
* PURPOSE  : to send Sychronous messages in Synchchannel
*----------------------------------------------------------------------------
* RETURNS  : PNIO_ERR_INTERNAL on error ; else  PNIO_OK
*----------------------------------------------------------------------------
* INPUTS   : pRq - send and receive Request
*            sendLen - send length
*            receiveLen - expected Receive length
* OUTPUS   : pRq - send and receive Request
*            receiveLen - expected Receive length
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
PNIO_UINT32 ICommon::SendReceiveSynch(char *pRq,
    PNIO_UINT32 sendLen, PNIO_UINT32 *receiveLen, PNIO_UINT32 timeout_ms)
{
    size_t ret;
    unsigned long expectedReceiveLen = *receiveLen;
    int sem_ret;
    char *tmp;
    PNIO_UINT32 tmpLength, Ret = PNIO_OK;
/*
    PNIO_UINT32 tmpOpcode, tmpHandle;
*/
    DPR_MUTEX_LOCK(m_pCpAdapter->synch_channel_mutex);

    TRC_OUT02(GR_STATE, LV_FCTCLBF, "ICommon::SendReceiveSynch sendLen %d, receiveLen %d",
        sendLen, *receiveLen);

    TRC_OUT02(GR_CHNL, LV_DATA, "SendReceiveSynch sends %d bytes data in channel %d:",
        sendLen, SYNCH);
    TRC_OUTD(GR_CHNL, LV_DATA, (unsigned char *)pRq, sendLen);
/*
    tmpOpcode = ((PNIO_UINT32 *)pRq)[1];
    tmpHandle = ((PNIO_UINT32 *)pRq)[2];
*/
    m_semSendReceiveSynchWaiterAvailable = true;

    ret = DPR_DRV_WRITE(m_pCpAdapter->chnls[SYNCH].file, pRq, sendLen);

    if(ret != sendLen) {
        TRC_OUT03(GR_CHNL, LV_ERR,
            "SynchChannelWrite failed, written %Zu bytes from %d, error '%s'",
            ret, sendLen, DPR_STRERROR());
        Ret = PNIO_ERR_NO_FW_COMMUNICATION;
        goto SendReceiveSynchExit;
    }

    if(m_bEmergencyClose) {
        TRC_OUT(GR_STATE, LV_FCTCLBF, "EmergencyClose  requested");
        Ret = PNIO_ERR_INTERNAL;
        goto SendReceiveSynchExit;
    }

    TRC_OUT01(GR_STATE, LV_FCTCLBF, "DPR_SEM_WAIT_TIME(%d ms) semSendReceiveSynch - for Receive msg", timeout_ms);

    sem_ret = DPR_SEM_WAIT_TIME(m_semSendReceiveSynch, timeout_ms);

    TRC_OUT(GR_STATE, LV_FCTCLBF, "DPR_SEM_WAIT_TIME waiting an synch channel ended");

    if(sem_ret == DPR_SEM_RET_TIMEOUT) {
        TRC_OUT01(GR_STATE, LV_ERR, "DPR_SEM_WAIT_TIME timeout after %d by communication through Synch channel, suppose firmware fatal error", timeout_ms);
        Ret = PNIO_ERR_NO_FW_COMMUNICATION;
        goto SendReceiveSynchExit;
    }

    tmpLength = *receiveLen;

    if(tmpLength >= m_pCpAdapter->chnls[SYNCH].user_pool_length_used) {
        memcpy(pRq, m_pCpAdapter->chnls[SYNCH].user_pool_ptr,
            m_pCpAdapter->chnls[SYNCH].user_pool_length_used);
        tmp = pRq;
    } else {
        tmp = NULL;
    }
    tmpLength = m_pCpAdapter->chnls[SYNCH].user_pool_length_used;

    if(!tmp) {
        TRC_OUT(GR_CHNL, LV_ERR, "ReadSyncMessage failed");
        Ret = PNIO_ERR_INTERNAL;
        goto SendReceiveSynchExit;
    }

#if 0
    if(tmpOpcode != ((PNIO_UINT32 *)pRq)[1]) {
        printf("DPR_ASSERT, opcode is not in sync, opcode 0x%08x, Rq.opcode 0x%08x\n",
            tmpOpcode, ((PNIO_UINT32 *)pRq)[1]);
        printf("dump raw:\n");
        for(PNIO_UINT32 i = 0; i < tmpLength; ++i) {
            if(!(i % 16))
                printf("\n\t");

            printf(" 0x%02x", pRq[i]);
        }
        printf("\n");

        fflush(stdout);
        DPR_ASSERT(0);
    }

    if(LE_TO_CPU(tmpOpcode) > 1 &&
        LE_TO_CPU(tmpOpcode) < 100 &&
        tmpHandle != ((PNIO_UINT32 *)pRq)[2]) {
        printf("DPR_ASSERT  opcode 0x%08x, handle 0x%08x, Rq.handle 0x%08x\n",
            tmpOpcode, tmpHandle, ((PNIO_UINT32 *)pRq)[2]);
        fflush(stdout);
        DPR_ASSERT(0);
    }
#endif
    /* verify the msg length */
    if((expectedReceiveLen != 0) && expectedReceiveLen != tmpLength) {
        TRC_OUT02(GR_CHNL, LV_ERR, "unexpected length %d, expected length %d",
            tmpLength, expectedReceiveLen);
        Ret = PNIO_ERR_INTERNAL;
        goto SendReceiveSynchExit;
    }

    *receiveLen = tmpLength;

SendReceiveSynchExit:
    m_semSendReceiveSynchWaiterAvailable = false;
    DPR_MUTEX_UNLOCK(m_pCpAdapter->synch_channel_mutex);

    return Ret;
}

/*===========================================================================
* FUNCTION : procChannelRead
*----------------------------------------------------------------------------
* PURPOSE  : callback processing threads for PNIO communication
*----------------------------------------------------------------------------
* RETURNS  : -
*----------------------------------------------------------------------------
* INPUTS   :
* OUTPUS   :
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
static DPR_THREAD_RETURN_TYPE DPR_THREAD_DECL procChannelRead(void *arg)
{
    CHANNEL *my_channel = (CHANNEL *)arg;
    DPR_ADAPTER *adapter = (DPR_ADAPTER *)my_channel->parent;
    ICommon *pThis = adapter->icommon_ptr;
    int ret;

    TRC_OUT01(GR_CHNL, LV_FCTCLBF, "-> procChannelRead channel_nr=%d",
        my_channel->channel_number);
    /* loop till shutdown req */
    while(!my_channel->stop_thread) {
        TRC_OUT02(GR_CHNL, LV_FCTCLBF,
            "procChannelRead channel_nr=%d: wait for new msg, length %d",
            my_channel->channel_number,
            my_channel->user_pool_length);

        ret = DPR_DRV_READ(my_channel->file, my_channel->user_pool_ptr,
            my_channel->user_pool_length);

        if(ret < 0) {
            TRC_OUT04(GR_CHNL, LV_FCTCLBF,
                "procChannelRead channel_nr=%d: read error ret %d errno %d(%s)",
                my_channel->channel_number, ret, errno, DPR_STRERROR());
            continue;
        } else if(!ret && DPR_DRV_ERROR(my_channel->file)) {
            TRC_OUT01(GR_CHNL, LV_FCTCLBF,
                "procChannelRead channel_nr=%d: read error, ret 0, ferror()",
                my_channel->channel_number);
            continue;
        } else if((unsigned long)ret > my_channel->user_pool_length) {
            TRC_OUT02(GR_CHNL, LV_ERR,
                "procChannelRead channel_nr=%d: buffer is too small, need %d bytes",
                my_channel->channel_number, ret);
            continue;
        }

        my_channel->user_pool_length_used = ret;

        TRC_OUT02(GR_CHNL, LV_FCTCLBF, "procChannelRead channel_nr=%d: receives %d bytes data",
            my_channel->channel_number, my_channel->user_pool_length_used);
        TRC_OUTD(GR_CHNL, LV_FCTCLBF,
            (unsigned char *)my_channel->user_pool_ptr, my_channel->user_pool_length_used);

        switch(my_channel->channel_number) {
        case SYNCH:
            if(pThis->m_semSendReceiveSynchWaiterAvailable){
              DPR_SEM_POST(pThis->m_semSendReceiveSynch);
            }
            else {
              TRC_OUT01(GR_CHNL, LV_WARN,
                "procChannelRead %d: nobody waits for data, junk received data", my_channel->channel_number);
            }
            break;
        case ALARM:
            pThis->ProcAlarm(my_channel);
            break;
        case MODIND:
            pThis->ProcModeInd(my_channel);
            break;
        case DATAREC:
            pThis->ProcDataRec(my_channel);
            break;
        default:
            TRC_OUT01(GR_CHNL, LV_ERR,
                "procChannelRead channel_nr=%d: undefined channel", my_channel->channel_number);
        }
    }

    TRC_OUT01(GR_CHNL, LV_FCTCLBF, "<- procChannelRead channel_nr=%d",
        my_channel->channel_number);

    DPR_THREAD_END();
}

/*===========================================================================
* FUNCTION : ICommon::OpenDprChannel
*----------------------------------------------------------------------------
* PURPOSE  : this function wrapper for openning single channel,
*            start reader thread
*----------------------------------------------------------------------------
* RETURNS  : PNIO_OK on success
*----------------------------------------------------------------------------
* INPUTS   :
* OUTPUS   :
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
PNIO_UINT32 ICommon::OpenDprChannel(const char *device, DPR_ADAPTER *adapter, CHANNELS chnl) {

    DPR_DRV_HANDLE file;
    int ret;
    struct t_read_pool usr_pool;
    CHANNEL *channel = &adapter->chnls[chnl];
    char *tmpptr = NULL;

    TRC_OUT01(GR_INIT, LV_FCTPUB1, "-> OpenDprChannel %s", device);

    if(NULL != channel->file)
        return PNIO_ERR_ALREADY_DONE;

    file = DPR_DRV_OPEN(device);
    if(!file) {
        TRC_OUT01(GR_INIT, LV_ERR, "ERROR openning %s", device);
        return PNIO_ERR_INTERNAL;
    }

    usr_pool.user_id = adapter->user_id;
    if((ret = DPR_DRV_IOCTL(file, CP16XX_IOC_BIND,
        &usr_pool, sizeof(usr_pool), sizeof(usr_pool))) < 0) {
        TRC_OUT02(GR_INIT, LV_ERR, "ioctl bind '%s' failed, error '%s'",
            device, DPR_STRERROR());
        free(tmpptr);
        DPR_DRV_CLOSE(file);
        return PNIO_ERR_DRIVER_IOCTL_FAILED;
    }

    TRC_OUT03(GR_INIT, LV_FCTPUB1, "          Channel %s pool read len=%lu pool write len=%lu",
               device, usr_pool.read_length, usr_pool.write_length);

    if(usr_pool.read_length) {
        channel->user_pool_ptr = (char *)malloc(usr_pool.read_length);
        if(!channel->user_pool_ptr) {
            TRC_OUT02(GR_INIT, LV_ERR, "alloc memory receive pool for '%s' len %d failed", device, usr_pool.read_length);
            DPR_DRV_CLOSE(file);
            return PNIO_ERR_NO_RESOURCE;
        }
    }

    if(usr_pool.write_length) {
        channel->send_pool_ptr = (char *)malloc(usr_pool.write_length);
        if(!channel->send_pool_ptr) {
            TRC_OUT02(GR_INIT, LV_ERR, "alloc memory temp send pool for '%s' len %d failed", device, usr_pool.write_length);
            DPR_DRV_CLOSE(file);
            return PNIO_ERR_NO_RESOURCE;
        }
    }

    channel->channel_number = chnl;
    channel->file = file;
    channel->user_pool_length = usr_pool.read_length;
    channel->send_pool_length = usr_pool.write_length;
    channel->parent = adapter;
    channel->stop_thread = 0;

    /* thread for processing the callback messages */
    if(!DPR_THREAD_CREATE(&channel->th_reader, "", procChannelRead, channel)) {
        TRC_OUT01(GR_MGT, LV_ERR, "Create thread for %s failed", device);
        goto OpenDprChannelfailchannel;
    }
    TRC_OUT02(GR_INIT, LV_FCTPUB1, "Create thread for %s ok, handle %x",
        device, channel->th_reader);

    TRC_OUT01(GR_INIT, LV_FCTPUB1, "<- OpenDprChannel %s", device);
    return PNIO_OK;

OpenDprChannelfailchannel:
    channel->stop_thread = 1;
    channel->parent = NULL;
    if(channel->user_pool_ptr)
        free(channel->user_pool_ptr);
    channel->user_pool_ptr = NULL;
    channel->user_pool_length = 0;
    DPR_DRV_CLOSE(channel->file);
    channel->file = NULL;

    return PNIO_ERR_START_THREAD_FAILED;
}

/*===========================================================================
* FUNCTION : ICommon::CloseDprChannel
*----------------------------------------------------------------------------
* PURPOSE  : this function wrapper for openning single channel,
*            start reader thread
*----------------------------------------------------------------------------
* RETURNS  : PNIO_OK on success; else PNIO_ERR_INTERNAL
*----------------------------------------------------------------------------
* INPUTS   :
* OUTPUS   :
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
PNIO_UINT32 ICommon::CloseDprChannel(CHANNEL *channel) {

    struct t_read_pool usr_pool;
    int ret;
    DPR_ADAPTER *adapter = (DPR_ADAPTER *)channel->parent;

    TRC_OUT(GR_INIT, LV_FCTPUB1, "-> CloseDprChannel");

    channel->stop_thread = 1;

    usr_pool.user_id = adapter->user_id;
    ret = DPR_DRV_IOCTL(channel->file, CP16XX_IOC_UNBIND,
        &usr_pool, sizeof(usr_pool), 0);

    if(channel->th_reader) {
        TRC_OUT01(GR_INIT, LV_FCTPUB1, "Stop thread, handle %x",
            channel->th_reader);

        DPR_THREAD_JOIN(channel->th_reader);
        channel->th_reader = 0;
    }

    channel->parent = NULL;
    if(channel->user_pool_ptr)
        free(channel->user_pool_ptr);
    channel->user_pool_ptr = NULL;
    channel->user_pool_length = 0;
    if(channel->send_pool_ptr)
        free(channel->send_pool_ptr);
    channel->send_pool_ptr = NULL;
    channel->send_pool_length = 0;
    DPR_DRV_CLOSE(channel->file);
    channel->file= NULL;

    TRC_OUT(GR_INIT, LV_FCTPUB1, "<- CloseDprChannel");

    return PNIO_OK;
}

/*===========================================================================
* FUNCTION : procMgtChannelRead
*----------------------------------------------------------------------------
* PURPOSE  : callback processing threads
*----------------------------------------------------------------------------
* RETURNS  : -
*----------------------------------------------------------------------------
* INPUTS   :
* OUTPUS   :
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
static DPR_THREAD_RETURN_TYPE DPR_THREAD_DECL procMgtChannelRead(void *arg)
{
    struct CP_WATCHDOG *my_channel = (struct CP_WATCHDOG *)arg;

    int ret;
    /*union {
        DPR_MSG_HDR *rqb;
        WD_TIMEOUT_DPRMGTCHNL_RB *wd_rqb;
    } u;*/
    PNIO_DPR_MGT_CHNL_RQB *u;

    TRC_OUT(GR_STATE, LV_FCTCLBF, "-> procMgtChannelRead");

    /* loop till shutdown req */
    while(!my_channel->stop_thread) {
        TRC_OUT(GR_CHNL, LV_FCTCLBF, "procMgtChannelRead: wait for new msg");

        ret = DPR_DRV_READ(my_channel->file, my_channel->user_pool_ptr,
            my_channel->user_pool_length);

        if(ret < 0) {
            TRC_OUT01(GR_CHNL, LV_FCTCLBF,
                "procMgtChannelRead: read error ret %d", ret);
            continue;
        } else if(!ret && DPR_DRV_ERROR(my_channel->file)) {
            TRC_OUT(GR_CHNL, LV_FCTCLBF,
                "procMgtChannelRead: read error, ret 0, ferror()");
            continue;
        } else if((unsigned long)ret > my_channel->user_pool_length) {
            TRC_OUT01(GR_CHNL, LV_ERR,
                "procMgtChannelRead: buffer is too small, need %d bytes", ret);
            continue;
        }

        TRC_OUT01(GR_CHNL, LV_FCTCLBF, "procMgtChannelRead: read %d bytes", ret);

        u = (PNIO_DPR_MGT_CHNL_RQB *)my_channel->user_pool_ptr;
        TRC_OUT01(GR_CHNL, LV_FCTCLBF, "procMgtChannelRead: hostref %lu", LE_TO_CPU(u->rqb.hostref));
        TRC_OUT01(GR_CHNL, LV_FCTCLBF, "procMgtChannelRead: subsystem 0x%02x", u->rqb.subsystem);
        TRC_OUT01(GR_CHNL, LV_FCTCLBF, "procMgtChannelRead: userid %u", LE_TO_CPU(u->rqb.userid));
        TRC_OUT01(GR_CHNL, LV_FCTCLBF, "procMgtChannelRead: response %u", LE_TO_CPU(u->rqb.response));
        TRC_OUT01(GR_CHNL, LV_FCTCLBF, "procMgtChannelRead: ulength %u", LE_TO_CPU(u->rqb.userdatalength));

        switch(u->rqb.subsystem) {
        case FW_SUBSYS_WATCHDOG:
            TRC_OUT01(GR_CHNL, LV_FCTCLBF, "procMgtChannelRead: wd_opcode %u", LE_TO_CPU16(u->wd_rqb.wd_opcode));
            switch(LE_TO_CPU16(u->wd_rqb.wd_opcode)) {
            case WD_OPC_TIMEOUT_IND:
                TRC_OUT02(GR_CHNL, LV_FCTCLBF, "procMgtChannelRead: wd_user_ref %u, my_channel->watchdogId %u",
                    LE_TO_CPU(u->wd_rqb.wd_user_ref), my_channel->watchdogId);
                if(LE_TO_CPU(u->wd_rqb.wd_user_ref) == my_channel->watchdogId) {
                    if(my_channel->cbfApplWatchdog) {
                        TRC_OUT02(GR_CHNL, LV_FCTCLBF, "-> call user Timeout callback(0x%x) CpIndex=0x%x",
                                                         my_channel->cbfApplWatchdog, my_channel->CpIndex);
                        my_channel->cbfApplWatchdog(my_channel->CpIndex);
                        TRC_OUT02(GR_ALARM, LV_FCTCLBF,
                            "<- back from user Timeout callback(0x%x) CpIndex=0x%x",
                            my_channel->cbfApplWatchdog, my_channel->CpIndex);
                    } else {
                        TRC_OUT01(GR_CHNL, LV_FCTCLBF,
                            "Timeout event decayed CpIndex=0x%x", my_channel->CpIndex);
                    }
                }
                break;

            default:
                TRC_OUT01(GR_CHNL, LV_ERR,
                    "unknown opcode 0x%04x", LE_TO_CPU16(u->wd_rqb.wd_opcode));
            }
            break;

        default:
            TRC_OUT01(GR_CHNL, LV_ERR,
                "unknown subsystem 0x%02x", u->rqb.subsystem);
        }
    }

    TRC_OUT(GR_CHNL, LV_FCTCLBF, "<- procMgtChannelRead");

    my_channel->th_reader = 0;

    DPR_THREAD_END();
}

/*===========================================================================
* FUNCTION : ICommon::set_appl_watchdog
*----------------------------------------------------------------------------
* PURPOSE  :
*----------------------------------------------------------------------------
* RETURNS  : PNIO_OK on success
*----------------------------------------------------------------------------
* INPUTS   :
* OUTPUS   :
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
PNIO_UINT32 ICommon::set_appl_watchdog(PNIO_UINT32 cpIndex,
    PNIO_UINT32 wdTimeOutInMs,
    PNIO_CBF_APPL_WATCHDOG pnio_appl_wd_cbf) {
    char device[36];
    int ret;
    struct CP_WATCHDOG *channel;
    struct t_register_appl_wd usr_wd;

    TRC_OUT(GR_INIT, LV_FCTPUB1, "-> set_appl_watchdog");

    if(cpIndex > MAX_CP_WATCHDOGS)
        return PNIO_ERR_MAX_REACHED;

    channel = &CpWds[DRIVER_IDX(cpIndex)];

    if(wdTimeOutInMs) {
        if(!pnio_appl_wd_cbf)
            return PNIO_ERR_PRM_CALLBACK;

        if(channel->cbfApplWatchdog)
            return PNIO_ERR_ALREADY_DONE;

        if(channel->file)
            return PNIO_ERR_INTERNAL;

        snprintf(device, sizeof(device)-1, DPR_MGT_INTERFACE, DRIVER_IDX(cpIndex));
        channel->file = DPR_DRV_OPEN(device);
        if(!channel->file) {
            TRC_OUT01(GR_INIT, LV_ERR, "openning '%s' failed", device);
            return PNIO_ERR_PRM_CP_ID;
        }

        usr_wd.timeout = wdTimeOutInMs;
        if((ret = DPR_DRV_IOCTL(channel->file, CP16XX_IOC_OWD,
            &usr_wd, sizeof(usr_wd), sizeof(usr_wd))) < 0) {
            TRC_OUT02(GR_INIT, LV_ERR, "ioctl register watchdogs for '%s' failed, error '%s'",
                device, DPR_STRERROR());
            DPR_DRV_CLOSE(channel->file);
            channel->file = NULL;
            if(EMFILE == errno)
                return PNIO_ERR_MAX_REACHED;
            else
                return PNIO_ERR_CREATE_INSTANCE;
        }

        TRC_OUT01(GR_INIT, LV_INFO, "CP16XX_IOC_OWD successfuly pool read len %lu", usr_wd.read_length);

        if(usr_wd.read_length < sizeof(PNIO_DPR_MGT_CHNL_RQB)){
          TRC_OUT02(GR_INIT, LV_ERR, "memory pool is too small %d (expected >= %d)",
                      usr_wd.read_length, sizeof(PNIO_DPR_MGT_CHNL_RQB));
          DPR_DRV_IOCTL(channel->file, CP16XX_IOC_CWD, &usr_wd, sizeof(usr_wd), 0);
          DPR_DRV_CLOSE(channel->file);
          return PNIO_ERR_NO_RESOURCE;
        }

        channel->user_pool_length = sizeof(PNIO_DPR_MGT_CHNL_RQB);
        channel->user_pool_ptr = (char *)malloc(channel->user_pool_length);
        if(!channel->user_pool_ptr) {
            TRC_OUT01(GR_INIT, LV_ERR, "alloc memory pool for '%s' failed", device);
            DPR_DRV_IOCTL(channel->file, CP16XX_IOC_CWD, &usr_wd, sizeof(usr_wd), 0);
            DPR_DRV_CLOSE(channel->file);
            return PNIO_ERR_NO_RESOURCE;
        }

        channel->cbfApplWatchdog = pnio_appl_wd_cbf;
        channel->watchdogId = usr_wd.wd_id;
        channel->user_id = usr_wd.user_id;
        channel->stop_thread = 0;
        channel->CpIndex = cpIndex;

        /* thread for processing the callback messages */
        if(!DPR_THREAD_CREATE(&channel->th_reader, "", procMgtChannelRead, channel)) {
            TRC_OUT01(GR_MGT, LV_ERR, "Create thread for '%s' failed", device);
            goto set_appl_watchdog_fail;
        }
        TRC_OUT02(GR_INIT, LV_FCTPUB1, "Create thread for '%s' ok, handle %x",
            device, channel->th_reader);
    } else {
        if(!channel->cbfApplWatchdog)
            return PNIO_ERR_ALREADY_DONE;

        if(!channel->file)
            return PNIO_ERR_INTERNAL;

        channel->stop_thread = 1;

        usr_wd.wd_id = channel->watchdogId;
        usr_wd.user_id = channel->user_id;
        ret = DPR_DRV_IOCTL(channel->file, CP16XX_IOC_CWD, &usr_wd, sizeof(usr_wd), 0);

        if(channel->th_reader) {
            TRC_OUT01(GR_INIT, LV_FCTPUB1, "Stop thread, handle %x", channel->th_reader);

            DPR_THREAD_JOIN(channel->th_reader);
            channel->th_reader = 0;
        }

        channel->cbfApplWatchdog = NULL;
        free(channel->user_pool_ptr);
        DPR_DRV_CLOSE(channel->file);
        channel->file = NULL;
    }

    TRC_OUT(GR_INIT, LV_FCTPUB1, "<- set_appl_watchdog");
    return PNIO_OK;

set_appl_watchdog_fail:
    channel->cbfApplWatchdog = NULL;
    DPR_DRV_CLOSE(channel->file);
    channel->file = NULL;

    return PNIO_ERR_CREATE_INSTANCE;
}

/*===========================================================================
* FUNCTION : ICommon::trigger_watchdog
*----------------------------------------------------------------------------
* PURPOSE  :
*----------------------------------------------------------------------------
* RETURNS  : PNIO_OK on success
*----------------------------------------------------------------------------
* INPUTS   :
* OUTPUS   :
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
PNIO_UINT32 ICommon::trigger_watchdog(PNIO_UINT32 cpIndex) {
    int ret;
    struct t_register_appl_wd usr_wd;
    struct CP_WATCHDOG *channel;

    TRC_OUT(GR_STATE, LV_FCTPUB1, "-> trigger_watchdog");

    if(cpIndex > MAX_CP_WATCHDOGS)
        return PNIO_ERR_PRM_CP_ID;

    channel = &CpWds[DRIVER_IDX(cpIndex)];

    if(!channel->cbfApplWatchdog)
        return PNIO_ERR_PRM_HND;

    if(!channel->file)
        return PNIO_ERR_PRM_HND;

    usr_wd.wd_id = channel->watchdogId;
    usr_wd.user_id = channel->user_id;

    if((ret = DPR_DRV_IOCTL(channel->file, CP16XX_IOC_TWD,
        &usr_wd, sizeof(usr_wd), 0)) < 0) {
        TRC_OUT01(GR_STATE, LV_ERR, "trigger watchdogs failed, error '%s'",
            DPR_STRERROR());

        return PNIO_ERR_DRIVER_IOCTL_FAILED;
    }

    TRC_OUT(GR_STATE, LV_FCTPUB1, "<- trigger_watchdog");

    return PNIO_OK;
}

/*===========================================================================
* FUNCTION : procCallbackIRT
*----------------------------------------------------------------------------
* PURPOSE  : callback processing threads for IRT
*----------------------------------------------------------------------------
* RETURNS  : -
*----------------------------------------------------------------------------
* INPUTS   :
* OUTPUS   :
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
#if defined(RTAI)

#include "rtai.h"
#include "rtai_sem.h"
#include "rtai_sched.h"
#include "rtai_registry.h"
#include "rtai_lxrt.h"

DPR_THREAD_RETURN_TYPE DPR_THREAD_DECL procCallbackIRT(void *arg)
{
    CHANNEL *my_channel = (CHANNEL *)arg;
    DPR_ADAPTER *adapter = (DPR_ADAPTER *)my_channel->parent;
    ICommon *pThis = adapter->icommon_ptr;
    PNIO_CP_CBE_PRM prm;
    struct sched_param mysched;
    SEM *sem;
    RT_TASK *rttask;
    const char *rttask_name, *rtsema_name;
    char tmp[200], *ptmp;
    struct t_register_app reg;

    ioctl(fileno(my_channel->file), CP16XX_IOC_IRTCPID, &reg);

    switch(my_channel->user_pool_length){
    case CP16XX_STARTOP:
      rtsema_name = RTAI_MAGIC_STARTOP;
      rttask_name = RTAI_MAGIC_STARTOP"t";
      break;
    case CP16XX_OPFAULT:
      rtsema_name = RTAI_MAGIC_OPFAULT;
      rttask_name = RTAI_MAGIC_OPFAULT"t";
      break;
    case CP16XX_NEWCYCLE:
      rtsema_name = RTAI_MAGIC_NEWCYCLE;
      rttask_name = RTAI_MAGIC_NEWCYCLE"t";
      break;
    default:
      goto procCallbackIRT_rt_task_init_fail;
      break;
    }

    TRC_OUT01(GR_CHNL, LV_FCTCLBF, "-> procCallbackIRT %d",
        my_channel->channel_number);

    mysched.sched_priority = 99;
    if(sched_setscheduler(0, SCHED_FIFO, &mysched) == -1) {
        TRC_OUT02(GR_CHNL, LV_ERR,
            "procCallbackIRT %d unable to force priority, error '%s',"
            " you need root capabilities to run RTAI stuff",
            my_channel->channel_number, DPR_STRERROR());
        goto procCallbackIRT_fifo_fail;
    }

    mlockall(MCL_CURRENT | MCL_FUTURE);

    ptmp = RTAI_UNIC_NAME(tmp, rttask_name, (int)reg.user_id);
    if(!(rttask = rt_task_init(nam2num(ptmp), 0, 0, 0))) {
        TRC_OUT03(GR_CHNL, LV_ERR, "procCallbackIRT %d unable to init RT '%s' task, error '%s'",
            my_channel->channel_number, ptmp, DPR_STRERROR());
        goto procCallbackIRT_rt_task_init_fail;
    }
    TRC_OUT02(GR_CHNL, LV_FCTCLBF, "procCallbackIRT %d init RT %s task",
        my_channel->channel_number, ptmp);

    ptmp = RTAI_UNIC_NAME(tmp, rtsema_name, (int)reg.user_id);
    if(!(sem = (SEM *)rt_get_adr(nam2num(ptmp)))) {
        TRC_OUT02(GR_CHNL, LV_ERR, "procCallbackIRT %d unable to locate RT semaphore, error '%s'",
            my_channel->channel_number, DPR_STRERROR());
        goto procCallbackIRT_rt_get_adr_fail;
    }
    TRC_OUT02(GR_CHNL, LV_FCTCLBF, "procCallbackIRT %d locate RT %s semaphore",
        my_channel->channel_number, ptmp);

    rt_make_hard_real_time();

    my_channel->ready = 1;
    while(!my_channel->stop_thread) {

        rt_sem_wait(sem);

        if(my_channel->stop_thread)
            break;

        switch(my_channel->user_pool_length) {
        case CP16XX_STARTOP:
            pThis->m_pCpAdapter->IrtAccessStatus = IRT_ACCESS_STATUS_INSIDE;
            if(pThis->m_pCbf_StartOp_Ind) {
                prm.CbeType = PNIO_CP_CBE_STARTOP_IND;
                prm.CpIndex = adapter->CpIndex;
                prm.u.StartOp.AppHandle = ICommon::get_handle(pThis);
                SetCycleInfo(pThis->m_pCpAdapter, &prm.u.StartOp.CycleInfo);
                pThis->setCycleStat(&prm.u.StartOp.CycleInfo, PNIO_CP_CI_STARTOP);
                pThis->m_pCbf_StartOp_Ind(&prm);
            } else {
                TRC_OUT02(GR_CHNL, LV_TIMECRIT,
                    "procCallbackIRT %d: decayed PNIO_CP_CBE_STARTOP_IND, Handle=0x%x",
                    my_channel->channel_number, ICommon::get_handle(pThis));
            }
            break;
        case CP16XX_OPFAULT:
            pThis->m_pCpAdapter->IrtAccessStatus = IRT_ACCESS_STATUS_OUTSIDE;
            if(pThis->m_pCbf_OpFault_Ind) {
                prm.CbeType = PNIO_CP_CBE_OPFAULT_IND;
                prm.CpIndex = adapter->CpIndex;
                prm.u.OpFault.AppHandle = ICommon::get_handle(pThis);
                SetCycleInfo(pThis->m_pCpAdapter, &prm.u.OpFault.CycleInfo);
                pThis->setCycleStat(&prm.u.StartOp.CycleInfo, PNIO_CP_CI_OPFAULT);
                pThis->m_pCbf_OpFault_Ind(&prm);
            } else {
                TRC_OUT02(GR_CHNL, LV_TIMECRIT,
                    "procCallbackIRT %d: decayed PNIO_CP_CBE_OPFAULT_IND, Handle=0x%x",
                    my_channel->channel_number, ICommon::get_handle(pThis));
            }
            break;
        case CP16XX_NEWCYCLE:
#ifndef IO_ROUTER
            if(pThis->m_pCbf_NewCycle_Ind) {
                prm.CbeType = PNIO_CP_CBE_NEWCYCLE_IND;
                prm.CpIndex = adapter->CpIndex;
                prm.u.NewCycle.AppHandle = ICommon::get_handle(pThis);
                SetCycleInfo(pThis->m_pCpAdapter, &prm.u.NewCycle.CycleInfo);
                pThis->setCycleStat(&prm.u.StartOp.CycleInfo, PNIO_CP_CI_NEWCYCLE);
                pThis->m_pCbf_NewCycle_Ind(&prm);
            } else {
                TRC_OUT02(GR_CHNL, LV_TIMECRIT,
                    "procCallbackIRT %d: decayed PNIO_CP_CBE_NEWCYCLE_IND, Handle=0x%x",
                    my_channel->channel_number, ICommon::get_handle(pThis));
            }
#else  /* IO_ROUTER */
            prm.CbeType = PNIO_CP_CBE_NEWCYCLE_IND;
            prm.CpIndex = adapter->CpIndex;
            prm.u.NewCycle.AppHandle = ICommon::get_handle(pThis);
            SetCycleInfo(pThis->m_pCpAdapter, &prm.u.NewCycle.CycleInfo);
            pThis->setCycleStat(&prm.u.StartOp.CycleInfo, PNIO_CP_CI_NEWCYCLE);
            if(pThis->m_pCbf_NewCycle_Ind) {
                    pThis->m_pCbf_NewCycle_Ind(&prm);
            }
                        if (pThis->m_pCbf_TransferWD) {
                        pThis->m_pCbf_TransferWD(&prm, static_cast<IController*>(pThis));
                        } else if(!pThis->m_pCbf_NewCycle_Ind) {
                TRC_OUT02(GR_CHNL, LV_TIMECRIT,
                    "procCallbackIRT %d: decayed PNIO_CP_CBE_NEWCYCLE_IND, Handle=0x%x",
                    my_channel->channel_number, ICommon::get_handle(pThis));
                        }
#endif  /* IO_ROUTER */
            break;
        default:
            TRC_OUT02(GR_CHNL, LV_TIMECRIT,
                "procCallbackIRT %d: unknown callback with %d",
                    my_channel->channel_number, my_channel->user_pool_length);
        }
    }
    rt_make_soft_real_time();

    TRC_OUT01(GR_CHNL, LV_FCTCLBF, "<- procCallbackIRT %d",
        my_channel->channel_number);

procCallbackIRT_rt_get_adr_fail:
    rt_task_delete(rttask);

procCallbackIRT_rt_task_init_fail:
    munlockall();

procCallbackIRT_fifo_fail:
    my_channel->exited = 1;

    DPR_THREAD_END();
}

#elif defined(XENOMAI)

#include "native/sem.h"
#include "native/task.h"

DPR_THREAD_RETURN_TYPE DPR_THREAD_DECL procCallbackIRT(void *arg)
{
    CHANNEL *my_channel = (CHANNEL *)arg;
    DPR_ADAPTER *adapter = (DPR_ADAPTER *)my_channel->parent;
    ICommon *pThis = adapter->icommon_ptr;
    PNIO_CP_CBE_PRM prm;
    struct sched_param mysched;
    RT_SEM rtsem;
    RT_TASK rttask;
    const char *rttask_name, *rtsema_name;
    char tmp[200], *ptmp;
    struct t_register_app reg;
    int err;

    ioctl(fileno(my_channel->file), CP16XX_IOC_IRTCPID, &reg);

    switch(my_channel->user_pool_length){
    case CP16XX_STARTOP:
      rtsema_name = XENO_MAGIC_STARTOP;
      rttask_name = XENO_MAGIC_STARTOP"t";
      break;
    case CP16XX_OPFAULT:
      rtsema_name = XENO_MAGIC_OPFAULT;
      rttask_name = XENO_MAGIC_OPFAULT"t";
      break;
    case CP16XX_NEWCYCLE:
      rtsema_name = XENO_MAGIC_NEWCYCLE;
      rttask_name = XENO_MAGIC_NEWCYCLE"t";
      break;
    default:
      goto procCallbackIRT_rt_task_init_fail;
      break;
    }

    TRC_OUT01(GR_CHNL, LV_FCTCLBF, "-> procCallbackIRT %d",
        my_channel->channel_number);

    mysched.sched_priority = 99;
    if(sched_setscheduler(0, SCHED_FIFO, &mysched) == -1) {
        TRC_OUT02(GR_CHNL, LV_ERR,
            "procCallbackIRT %d unable to force priority, error '%s',"
            " you need root capabilities to run RTAI stuff",
            my_channel->channel_number, DPR_STRERROR());
        goto procCallbackIRT_fifo_fail;
    }

    mlockall(MCL_CURRENT | MCL_FUTURE);

    ptmp = XENO_UNIC_NAME(tmp, rttask_name, (int)reg.user_id);
    if((err = rt_task_shadow(&rttask, ptmp, 99, 0))) {
        TRC_OUT03(GR_CHNL, LV_ERR, "procCallbackIRT %d unable to init RT '%s' task, error '%s'",
            my_channel->channel_number, ptmp, strerror(err));
        goto procCallbackIRT_rt_task_init_fail;
    }
    TRC_OUT02(GR_CHNL, LV_TIMECRIT, "procCallbackIRT %d init RT %s task",
        my_channel->channel_number, ptmp);

    ptmp = XENO_UNIC_NAME(tmp, rtsema_name, (int)reg.user_id);
    if((err = rt_sem_bind(&rtsem, ptmp, TM_NONBLOCK))) {
        TRC_OUT02(GR_CHNL, LV_ERR, "procCallbackIRT %d unable to locate RT semaphore, error '%s'",
            my_channel->channel_number, strerror(err));
        goto procCallbackIRT_rt_get_adr_fail;
    }
    TRC_OUT02(GR_CHNL, LV_TIMECRIT, "procCallbackIRT %d locate RT %s semaphore",
        my_channel->channel_number, ptmp);

    my_channel->ready = 1;
    while(!my_channel->stop_thread) {

        rt_sem_p(&rtsem, TM_INFINITE);

        if(my_channel->stop_thread)
            break;

        switch(my_channel->user_pool_length) {
        case CP16XX_STARTOP:
            pThis->m_pCpAdapter->IrtAccessStatus = IRT_ACCESS_STATUS_INSIDE;
            if(pThis->m_pCbf_StartOp_Ind) {
                prm.CbeType = PNIO_CP_CBE_STARTOP_IND;
                prm.CpIndex = adapter->CpIndex;
                prm.u.StartOp.AppHandle = ICommon::get_handle(pThis);
                SetCycleInfo(pThis->m_pCpAdapter, &prm.u.StartOp.CycleInfo);
                pThis->setCycleStat(&prm.u.StartOp.CycleInfo, PNIO_CP_CI_STARTOP);
                pThis->m_pCbf_StartOp_Ind(&prm);
            } else {
                TRC_OUT02(GR_CHNL, LV_TIMECRIT,
                    "procCallbackIRT %d: decayed PNIO_CP_CBE_STARTOP_IND, Handle=0x%x",
                    my_channel->channel_number, ICommon::get_handle(pThis));
            }
            break;
        case CP16XX_OPFAULT:
            pThis->m_pCpAdapter->IrtAccessStatus = IRT_ACCESS_STATUS_OUTSIDE;
            if(pThis->m_pCbf_OpFault_Ind) {
                prm.CbeType = PNIO_CP_CBE_OPFAULT_IND;
                prm.CpIndex = adapter->CpIndex;
                prm.u.OpFault.AppHandle = ICommon::get_handle(pThis);
                SetCycleInfo(pThis->m_pCpAdapter, &prm.u.OpFault.CycleInfo);
                pThis->setCycleStat(&prm.u.StartOp.CycleInfo, PNIO_CP_CI_OPFAULT);
                pThis->m_pCbf_OpFault_Ind(&prm);
            } else {
                TRC_OUT02(GR_CHNL, LV_TIMECRIT,
                    "procCallbackIRT %d: decayed PNIO_CP_CBE_OPFAULT_IND, Handle=0x%x",
                    my_channel->channel_number, ICommon::get_handle(pThis));
            }
            break;
        case CP16XX_NEWCYCLE:
#ifndef IO_ROUTER
            if(pThis->m_pCbf_NewCycle_Ind) {
                prm.CbeType = PNIO_CP_CBE_NEWCYCLE_IND;
                prm.CpIndex = adapter->CpIndex;
                prm.u.NewCycle.AppHandle = ICommon::get_handle(pThis);
                SetCycleInfo(pThis->m_pCpAdapter, &prm.u.NewCycle.CycleInfo);
                pThis->setCycleStat(&prm.u.StartOp.CycleInfo, PNIO_CP_CI_NEWCYCLE);
                pThis->m_pCbf_NewCycle_Ind(&prm);
            } else {
                TRC_OUT02(GR_CHNL, LV_TIMECRIT,
                    "procCallbackIRT %d: decayed PNIO_CP_CBE_NEWCYCLE_IND, Handle=0x%x",
                    my_channel->channel_number, ICommon::get_handle(pThis));
            }
#else  /* IO_ROUTER */
            prm.CbeType = PNIO_CP_CBE_NEWCYCLE_IND;
            prm.CpIndex = adapter->CpIndex;
            prm.u.NewCycle.AppHandle = ICommon::get_handle(pThis);
            SetCycleInfo(pThis->m_pCpAdapter, &prm.u.NewCycle.CycleInfo);
            pThis->setCycleStat(&prm.u.StartOp.CycleInfo, PNIO_CP_CI_NEWCYCLE);
            if(pThis->m_pCbf_NewCycle_Ind) {
                pThis->m_pCbf_NewCycle_Ind(&prm);
            }
            if (pThis->m_pCbf_TransferWD) {
                pThis->m_pCbf_TransferWD(&prm, static_cast<IController *>(pThis));
            } else if(!pThis->m_pCbf_NewCycle_Ind) {
                TRC_OUT02(GR_CHNL, LV_TIMECRIT,
                    "procCallbackIRT %d: decayed PNIO_CP_CBE_NEWCYCLE_IND, Handle=0x%x",
                    my_channel->channel_number, ICommon::get_handle(pThis));
            }
#endif  /* IO_ROUTER */
            break;
        default:
            TRC_OUT02(GR_CHNL, LV_TIMECRIT,
                "procCallbackIRT %d: unknown callback with %d",
                    my_channel->channel_number, my_channel->user_pool_length);
        }
    }

    TRC_OUT01(GR_CHNL, LV_TIMECRIT, "<- procCallbackIRT %d",
        my_channel->channel_number);

    rt_sem_unbind(&rtsem);

procCallbackIRT_rt_get_adr_fail:
    rt_task_delete(&rttask);

procCallbackIRT_rt_task_init_fail:
    munlockall();

procCallbackIRT_fifo_fail:
    my_channel->exited = 1;

    DPR_THREAD_END();
}

#else /* !RTAI && !XENOMAI */

DPR_THREAD_RETURN_TYPE DPR_THREAD_DECL procCallbackIRT(void *arg)
{
    CHANNEL *my_channel = (CHANNEL *)arg;
    DPR_ADAPTER *adapter = (DPR_ADAPTER *)my_channel->parent;
    ICommon *pThis = adapter->icommon_ptr;
    PNIO_CP_CBE_PRM prm;
    int ret;

    TRC_OUT01(GR_CHNL, LV_FCTCLBF, "-> procCallbackIRT %d",
        my_channel->channel_number);
#if 0
    {
        struct sched_param mysched;
        mysched.sched_priority = 99;
        if(sched_setscheduler(0, SCHED_FIFO, &mysched) == -1) {
            TRC_OUT02(GR_CHNL, LV_ERR, "procCallbackIRT %d unable to force priority, error '%s'",
                my_channel->channel_number, DPR_STRERROR());
        }
    }
#endif
    my_channel->ready = 1;
    while(!my_channel->stop_thread) {

        ret = DPR_DRV_READ(my_channel->file, my_channel->user_pool_ptr,
            my_channel->user_pool_length);

        if(my_channel->stop_thread)
            break;

        if(ret < 0) {
            TRC_OUT02(GR_CHNL, LV_TIMECRIT,
                "procCallbackIRT %d: read error ret %d",
                my_channel->channel_number, ret);
            continue;
        } else if(!ret && DPR_DRV_ERROR(my_channel->file)) {
            TRC_OUT01(GR_CHNL, LV_TIMECRIT,
                "procCallbackIRT %d: read error, ret 0, ferror()",
                my_channel->channel_number);
            continue;
        } else if((unsigned long)ret > my_channel->user_pool_length) {
            TRC_OUT02(GR_CHNL, LV_TIMECRIT,
                "procCallbackIRT %d: buffer is too small, need %d bytes",
                my_channel->channel_number, ret);
            continue;
        }

        switch(my_channel->user_pool_length) {
        case CP16XX_STARTOP:
            pThis->m_pCpAdapter->IrtAccessStatus = IRT_ACCESS_STATUS_INSIDE;
            if(pThis->m_pCbf_StartOp_Ind) {
                prm.CbeType = PNIO_CP_CBE_STARTOP_IND;
                prm.CpIndex = adapter->CpIndex;
                prm.u.StartOp.AppHandle = ICommon::get_handle(pThis);
                SetCycleInfo(pThis->m_pCpAdapter, &prm.u.StartOp.CycleInfo);
                pThis->setCycleStat(&prm.u.StartOp.CycleInfo, PNIO_CP_CI_STARTOP);
                pThis->m_pCbf_StartOp_Ind(&prm);
            } else {
                TRC_OUT02(GR_CHNL, LV_TIMECRIT,
                    "procCallbackIRT %d: decayed PNIO_CP_CBE_STARTOP_IND, Handle=0x%x",
                    my_channel->channel_number, ICommon::get_handle(pThis));
            }
            break;
        case CP16XX_OPFAULT:
            pThis->m_pCpAdapter->IrtAccessStatus = IRT_ACCESS_STATUS_OUTSIDE;
            if(pThis->m_pCbf_OpFault_Ind) {
                prm.CbeType = PNIO_CP_CBE_OPFAULT_IND;
                prm.CpIndex = adapter->CpIndex;
                prm.u.OpFault.AppHandle = ICommon::get_handle(pThis);
                SetCycleInfo(pThis->m_pCpAdapter, &prm.u.OpFault.CycleInfo);
                pThis->setCycleStat(&prm.u.StartOp.CycleInfo, PNIO_CP_CI_OPFAULT);
                pThis->m_pCbf_OpFault_Ind(&prm);
            } else {
                TRC_OUT02(GR_CHNL, LV_TIMECRIT,
                    "procCallbackIRT %d: decayed PNIO_CP_CBE_OPFAULT_IND, Handle=0x%x",
                    my_channel->channel_number, ICommon::get_handle(pThis));
            }
            break;
        case CP16XX_NEWCYCLE:
#ifndef IO_ROUTER
            if(pThis->m_pCbf_NewCycle_Ind) {
                prm.CbeType = PNIO_CP_CBE_NEWCYCLE_IND;
                prm.CpIndex = adapter->CpIndex;
                prm.u.NewCycle.AppHandle = ICommon::get_handle(pThis);
                SetCycleInfo(pThis->m_pCpAdapter, &prm.u.NewCycle.CycleInfo);
                pThis->setCycleStat(&prm.u.StartOp.CycleInfo, PNIO_CP_CI_NEWCYCLE);
                pThis->m_pCbf_NewCycle_Ind(&prm);
            } else {
                TRC_OUT02(GR_CHNL, LV_TIMECRIT,
                    "procCallbackIRT %d: decayed PNIO_CP_CBE_NEWCYCLE_IND, Handle=0x%x",
                    my_channel->channel_number, ICommon::get_handle(pThis));
            }
#else  /* IO_ROUTER */
            prm.CbeType = PNIO_CP_CBE_NEWCYCLE_IND;
            prm.CpIndex = adapter->CpIndex;
            prm.u.NewCycle.AppHandle = ICommon::get_handle(pThis);
            SetCycleInfo(pThis->m_pCpAdapter, &prm.u.NewCycle.CycleInfo);
            pThis->setCycleStat(&prm.u.StartOp.CycleInfo, PNIO_CP_CI_NEWCYCLE);
            if(pThis->m_pCbf_NewCycle_Ind) {
                    pThis->m_pCbf_NewCycle_Ind(&prm);
            }
            if(pThis->m_pCbf_TransferWD) {
                pThis->m_pCbf_TransferWD(&prm, static_cast<IController*>(pThis));
            } else if(!pThis->m_pCbf_NewCycle_Ind) {
                TRC_OUT02(GR_CHNL, LV_TIMECRIT,
                    "procCallbackIRT %d: decayed PNIO_CP_CBE_NEWCYCLE_IND, Handle=0x%x",
                    my_channel->channel_number, ICommon::get_handle(pThis));
            }
#endif  /* IO_ROUTER */
            break;
        default:
            TRC_OUT02(GR_CHNL, LV_TIMECRIT,
                "procCallbackIRT %d: unknown callback with %lu",
                    my_channel->channel_number, my_channel->user_pool_length);
        }
    }

    TRC_OUT01(GR_CHNL, LV_FCTCLBF, "<- procCallbackIRT %d",
        my_channel->channel_number);
    my_channel->exited = 1;

    DPR_THREAD_END();
}
#endif /* !RTAI */

/* we neew two pahses stop to avoid race conditions */
static PNIO_UINT32 start_irt_thread(const char *device,
    DPR_ADAPTER *adapter, CHANNELS chnl, int length) {
    DPR_DRV_HANDLE file = adapter->fd_control;
    CHANNEL *channel = &adapter->chnls[chnl];
    char *tmpptr = NULL;
    struct t_register_app reg;

    if(!file || !channel){
      DPR_ASSERT(0);
      return PNIO_ERR_INTERNAL;
    }

    TRC_OUT01(GR_INIT, LV_FCTPUB1, "-> start_irt_thread %s", device);

    reg.user_id = adapter->user_id;
    switch(chnl) {
    case IRT_STARTOP:
        reg.flags = REG_STARTOP;
        break;
    case IRT_OPFAULT:
        reg.flags = REG_OPFAULT;
        break;
    case IRT_NEWCYCLE:
        reg.flags = REG_NEWCYCLE;
        break;
    default:
        return PNIO_ERR_INTERNAL;
    }

    /* inform driver, that we do have callback */
    if(DPR_DRV_IOCTL(file, CP16XX_IOC_IRTCBF, &reg, sizeof(reg), 0)) {
        return PNIO_ERR_ALREADY_DONE;
    }

    /* prepare thread for a callback. */
    if(length) {
        tmpptr = (char *)malloc(length);
        if(!tmpptr) {
            TRC_OUT01(GR_INIT, LV_ERR, "ERROR alloc %s, memory pool", device);
            return PNIO_ERR_NO_RESOURCE;
        }
    }

    channel->channel_number = chnl;
    channel->file = file;
    channel->user_pool_length = length;
    channel->user_pool_ptr = tmpptr;
    channel->parent = adapter;
    channel->stop_thread = 0;
    channel->ready = 0;
    channel->exited = 0;

    /* thread for processing the callback messages */
    if(!DPR_THREAD_CREATE(&channel->th_reader, "", procCallbackIRT, channel)) {
        TRC_OUT01(GR_MGT, LV_ERR, "Create IRT thread for %s failed", device);
        goto register_cbf_ioctlfail;
    }

    while(!channel->ready && !channel->exited)
        DPR_TASK_DELAY(1);

    if(!channel->exited) {
        TRC_OUT02(GR_INIT, LV_FCTPUB1, "Create IRT thread for %s ok, handle %x",
            device, channel->th_reader);
    } else {
        TRC_OUT01(GR_INIT, LV_ERR, "Create IRT thread for %s failed, tread exits self", device);
        goto register_cbf_ioctlfail;
    }

    switch(chnl) {
    case IRT_STARTOP:
        reg.flags = RUN_STARTOP;
        break;
    case IRT_OPFAULT:
        reg.flags = RUN_OPFAULT;
        break;
    case IRT_NEWCYCLE:
        reg.flags = RUN_NEWCYCLE;
        break;
    default:
        return PNIO_ERR_INTERNAL;
    }
    /* now is thread read to service events */
    if(DPR_DRV_IOCTL(file, CP16XX_IOC_IRTCBF, &reg, sizeof(reg), 0)) {
        return PNIO_ERR_INTERNAL;
    }

    TRC_OUT01(GR_INIT, LV_FCTPUB1, "<- start_irt_thread %s", device);
    return PNIO_OK;

register_cbf_ioctlfail:
    channel->stop_thread = 1;
    channel->parent = NULL;
    if(channel->user_pool_ptr)
        free(channel->user_pool_ptr);
    channel->user_pool_ptr = NULL;
    channel->user_pool_length = 0;
    channel->file = NULL;

    return PNIO_ERR_START_RT_THREAD_FAILED;
}

/* we neew two pahses stop to avoid race conditions */
static PNIO_UINT32 stop_irt_thread(const char *device,
    DPR_ADAPTER *adapter, CHANNELS chnl, int length) {
    DPR_DRV_HANDLE file = adapter->fd_control;
    CHANNEL *channel = &adapter->chnls[chnl];
    int ret;
    struct t_register_app reg;

    DPR_ASSERT(file && channel);

    TRC_OUT01(GR_INIT, LV_FCTPUB1, "-> stop_irt_thread %s", device);

    reg.user_id = adapter->user_id;
    switch(chnl) {
    case IRT_STARTOP:
        reg.flags = DEL_STARTOP;
        break;
    case IRT_OPFAULT:
        reg.flags = DEL_OPFAULT;
        break;
    case IRT_NEWCYCLE:
        reg.flags = DEL_NEWCYCLE;
        break;
    default:
        return PNIO_ERR_INTERNAL;
    }

    /* inform driver, that we do not have callback anymore */
    if((ret = DPR_DRV_IOCTL(file, CP16XX_IOC_IRTCBF, &reg, sizeof(reg), 0))) {
        TRC_OUT02(GR_INIT, LV_ERR, "ioctl DEL_* %s failed, returns %i", device, ret);
    }

    channel->stop_thread = 1;
    /* propably all threads are gone, but someone is stucks in read without
       event. Kick it up. */

    reg.user_id = adapter->user_id;
    switch(chnl) {
    case IRT_STARTOP:
        reg.flags = KICKUP_STARTOP;
        break;
    case IRT_OPFAULT:
        reg.flags = KICKUP_OPFAULT;
        break;
    case IRT_NEWCYCLE:
        reg.flags = KICKUP_NEWCYCLE;
        break;
    default:
        return PNIO_ERR_INTERNAL;
    }
    if((ret = DPR_DRV_IOCTL(file, CP16XX_IOC_IRTCBF, &reg, sizeof(reg), 0))) {
        TRC_OUT02(GR_INIT, LV_ERR, "ioctl KICKUP_* %s failed, returns %i", device, ret);
    }

    /* now all threads must be gone. */

    if(channel->th_reader) {
        TRC_OUT01(GR_INIT, LV_FCTPUB1, "Stop thread, handle %x", channel->th_reader);

        DPR_THREAD_JOIN(channel->th_reader);
        channel->th_reader = 0;
    }

    channel->parent = NULL;
    if(channel->user_pool_ptr)
        free(channel->user_pool_ptr);
    channel->user_pool_ptr = NULL;
    channel->user_pool_length = 0;
    channel->file = NULL;

    TRC_OUT01(GR_INIT, LV_FCTPUB1, "<- stop_irt_thread %s", device);

    return PNIO_OK;
}

/*===========================================================================
* FUNCTION : ICommon::deinit_and_unregister
*----------------------------------------------------------------------------
* PURPOSE  :
*----------------------------------------------------------------------------
* RETURNS  : PNIO_OK on success
*----------------------------------------------------------------------------
* INPUTS   :
* OUTPUS   :
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
void ICommon::deinit_and_unregister(void) {
    if(m_pCbf_StartOp_Ind) {
        stop_irt_thread("PNIO_CP_CBE_STARTOP_IND", m_pCpAdapter,
            IRT_STARTOP, CP16XX_STARTOP);
        m_pCbf_StartOp_Ind = NULL;
    }
    if(m_pCbf_OpFault_Ind) {
        stop_irt_thread("PNIO_CP_CBE_OPFAULT_IND", m_pCpAdapter,
            IRT_OPFAULT, CP16XX_OPFAULT);
        m_pCbf_OpFault_Ind = NULL;
    }
#ifdef IO_ROUTER
    if(m_pCbf_NewCycle_Ind || m_pCbf_TransferWD) {
    stop_irt_thread("PNIO_CP_CBE_NEWCYCLE_IND", m_pCpAdapter,
        IRT_NEWCYCLE, CP16XX_NEWCYCLE);
    m_pCbf_NewCycle_Ind = NULL;
    m_pCbf_TransferWD = NULL;
#else /* IO_ROUTER */
    if(m_pCbf_NewCycle_Ind) {
    stop_irt_thread("PNIO_CP_CBE_NEWCYCLE_IND", m_pCpAdapter,
        IRT_NEWCYCLE, CP16XX_NEWCYCLE);
    m_pCbf_NewCycle_Ind = NULL;
#endif /* IO_ROUTER */
    }
}

/*===========================================================================
* FUNCTION : ICommon::CP_register_cbf
*----------------------------------------------------------------------------
* PURPOSE  :
*----------------------------------------------------------------------------
* RETURNS  : PNIO_OK on success
*----------------------------------------------------------------------------
* INPUTS   :
* OUTPUS   :
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
PNIO_UINT32 ICommon::CP_register_cbf(PNIO_CP_CBE_TYPE CbeType, PNIO_CP_CBF Cbf) {
    PNIO_UINT32 ret;
    PNIO_UINT8 irt_items = 0;

    TRC_OUT(GR_STATE, LV_FCTPUB1, "-> CP_register_cbf");

    // unregister is not possible
    if(!Cbf)
        return PNIO_ERR_PRM_CALLBACK;

    // Controller:only if IRT devices configured
    if(m_bController) {
        ret = KRAMIOTLB_IsIoSyncConfigured(m_hIodataUpdate,
            KRAMIOTLB_IO_SYNC, &irt_items, &m_HostIOTlbHdr);
        if(ret != PNIO_OK)
           return ret;
    }

    if(CbeType == PNIO_CP_CBE_NEWCYCLE_IND){
        if((irt_items > 0) && m_pCbf_StartOp_Ind){
            TRC_OUT(GR_INIT, LV_ERR, "register NEWCYCLE callback befor register STARTOP callback");
            return PNIO_ERR_NEWCYCLE_SEQUENCE_REG;
        }

        if(m_pCbf_NewCycle_Ind) {
            ret = PNIO_ERR_ALREADY_DONE;
        } else {
            m_pCbf_NewCycle_Ind = Cbf;
#ifdef IO_ROUTER
           /* newcycle callback need itr-thread-started, so start it here, if not already done in controller_open()
            * Note: irt-thread is started during controller open, but only if router is configured
            */
            if (!m_pCbf_TransferWD) {
                ret = start_irt_thread("PNIO_CP_CBE_NEWCYCLE_IND", m_pCpAdapter,
                    IRT_NEWCYCLE, CP16XX_NEWCYCLE);

                if(ret != PNIO_OK) {
                    stop_irt_thread("PNIO_CP_CBE_NEWCYCLE_IND", m_pCpAdapter,
                        IRT_NEWCYCLE, CP16XX_NEWCYCLE);
                    m_pCbf_NewCycle_Ind = NULL;
                    TRC_OUT(GR_INIT, LV_ERR,  "CP_register_cbf: NEWCYCLE ERROR ");
                }
                else {
                    TRC_OUT(GR_INIT, LV_INFO, "CP_register_cbf: NEWCYCLE  Cb + Th OK");  /* callback register + irt-thread start ok*/
                }
            }
            else {
                /* callback register only, irt-thrad was already started during controller_open() */
                TRC_OUT(GR_INIT, LV_INFO, "CP_register_cbf: NEWCYCLE  Cb OK");
                ret = PNIO_OK;
            }
#else /* IO_ROUTER */
            ret = start_irt_thread("PNIO_CP_CBE_NEWCYCLE_IND", m_pCpAdapter,
                IRT_NEWCYCLE, CP16XX_NEWCYCLE);

            if(ret != PNIO_OK) {
                stop_irt_thread("PNIO_CP_CBE_NEWCYCLE_IND", m_pCpAdapter,
                    IRT_NEWCYCLE, CP16XX_NEWCYCLE);
                m_pCbf_NewCycle_Ind = NULL;
            }
#endif /* IO_ROUTER */
        }
    } else {
        // Controller:only if IRT devices configured
        if(m_bController && irt_items == 0){
            TRC_OUT(GR_INIT, LV_ERR,
                "controller configuration has no IRT items, callback registration is impossible");
            return PNIO_ERR_INVALID_CONFIG;
        }

        /* sequence...
            ioctl(CP16XX_IOC_IRTCBF);
            OpFault;
            StartOp; */
        switch(CbeType) {
        case PNIO_CP_CBE_STARTOP_IND:
            if(m_pCbf_StartOp_Ind) {
                ret = PNIO_ERR_ALREADY_DONE;
            } else if(!m_pCbf_OpFault_Ind) {
                TRC_OUT(GR_INIT, LV_ERR, "register OPFAULT callback befor register STARTOP callback");
                return PNIO_ERR_OPFAULT_NOT_REG;
            } else {
                m_pCbf_StartOp_Ind = Cbf;

                ret = start_irt_thread("PNIO_CP_CBE_STARTOP_IND", m_pCpAdapter,
                    IRT_STARTOP, CP16XX_STARTOP);

                if(ret == PNIO_OK)
                    ret = RegisterStartOp();

                if(ret != PNIO_OK) {
                    stop_irt_thread("PNIO_CP_CBE_STARTOP_IND", m_pCpAdapter,
                        IRT_STARTOP, CP16XX_STARTOP);
                    m_pCbf_StartOp_Ind = NULL;
                }
            }
            break;
        case PNIO_CP_CBE_OPFAULT_IND:
            if(m_pCbf_OpFault_Ind) {
                ret = PNIO_ERR_ALREADY_DONE;
            } else {
                m_pCbf_OpFault_Ind = Cbf;

                ret = start_irt_thread("PNIO_CP_CBE_OPFAULT_IND", m_pCpAdapter,
                    IRT_OPFAULT, CP16XX_OPFAULT);

                if(ret != PNIO_OK) {
                    stop_irt_thread("PNIO_CP_CBE_OPFAULT_IND", m_pCpAdapter,
                        IRT_OPFAULT, CP16XX_OPFAULT);
                    m_pCbf_OpFault_Ind = NULL;
                }
            }
            break;
        default:
            ret = PNIO_ERR_PRM_TYPE;
        }
    }

    TRC_OUT(GR_STATE, LV_FCTPUB1, "<- CP_register_cbf");

    return ret;
}

/*===========================================================================
* FUNCTION : ICommon::CP_set_opdone
*----------------------------------------------------------------------------
* PURPOSE  :
*----------------------------------------------------------------------------
* RETURNS  : PNIO_OK on success
*----------------------------------------------------------------------------
* INPUTS   :
* OUTPUS   :
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
PNIO_UINT32 ICommon::CP_set_opdone(PNIO_CYCLE_INFO * pCycleInfo)
{
  if (NULL != pCycleInfo) {
    SetCycleInfo(m_pCpAdapter, pCycleInfo);
    setCycleStat(pCycleInfo, PNIO_CP_CI_OPDONE);
  }
#ifdef RTAI
    *(DPR_UINT32 *)(m_pCpAdapter->pErtecSwiBase + IRT_SYNCMODE) |= CPU_TO_LE(0x00000003);
#else
    size_t ret;
    static const char value[CP16XX_OPDONE] = {0, 0, 0};

    ret = DPR_DRV_WRITE(m_pCpAdapter->fd_control, &value, CP16XX_OPDONE);
    if(ret != CP16XX_OPDONE) {
        TRC_OUT03(GR_CHNL, LV_ERR,
            "CP_set_opdone failed, written %Zu bytes from %d, error '%s'",
            ret, CP16XX_OPDONE, DPR_STRERROR());
        return PNIO_ERR_NO_FW_COMMUNICATION;
    }
#endif
    m_pCpAdapter->IrtAccessStatus = IRT_ACCESS_STATUS_OUTSIDE;

    return PNIO_OK;
}

/*===========================================================================
* FUNCTION : ICommon::init_irt_dma_ranges
*----------------------------------------------------------------------------
* PURPOSE  :
*----------------------------------------------------------------------------
* RETURNS  : PNIO_OK on success
*----------------------------------------------------------------------------
* INPUTS   :
* OUTPUS   :
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
PNIO_UINT32 ICommon::init_irt_dma_ranges(void)
{
    PNIO_UINT8 irt_items = 0;
    KRAMIOTLB_Ret kret;
    struct t_dma_range dma_range;
    PNIO_UINT32 offset_min, offset_max, ret;

    TRC_OUT(GR_STATE, LV_FCTPUB1, "-> init_irt_dma_ranges");

    /* only if IRT devices configured */
    ret = KRAMIOTLB_IsIoSyncConfigured(m_hIodataUpdate,
        KRAMIOTLB_IO_SYNC, &irt_items, &m_HostIOTlbHdr);
    if(ret != PNIO_OK || !irt_items)
        return PNIO_ERR_INVALID_CONFIG;

    /* calculate DMA ranges */
    dma_range.user_id = m_pCpAdapter->user_id;

    offset_min = offset_max = 0;
    kret = KRAMIOTLB_GetDataOffset(m_hIodataUpdate,
        KRAMIOTLB_IO_IN, KRAMIOTLB_IO_SYNC,
        &offset_min, &offset_max, &m_HostIOTlbHdr);
    if(kret != KRAMIOTLB_OK)
        return PNIO_ERR_INVALID_CONFIG;

    dma_range.offset_in = offset_min;
    dma_range.length_in = offset_max - offset_min;

    offset_min = offset_max = 0;
    kret = KRAMIOTLB_GetDataOffset(m_hIodataUpdate,
        KRAMIOTLB_IO_OUT, KRAMIOTLB_IO_SYNC,
        &offset_min, &offset_max, &m_HostIOTlbHdr);
    if(kret != KRAMIOTLB_OK)
        return PNIO_ERR_INVALID_CONFIG;

    dma_range.offset_out = offset_min;
    dma_range.length_out = offset_max - offset_min;

    if(DPR_DRV_IOCTL(m_pCpAdapter->fd_control, CP16XX_IOC_SET_DMA_RANGE,
        &dma_range, sizeof(dma_range), 0))
        return PNIO_ERR_DRIVER_IOCTL_FAILED;

    TRC_OUT(GR_STATE, LV_FCTPUB1, "<- init_irt_dma_ranges");

    return PNIO_OK;
}

/*===========================================================================
* FUNCTION : ICommon::uninit_irt_dma_ranges
*----------------------------------------------------------------------------
* PURPOSE  :
*----------------------------------------------------------------------------
* RETURNS  : PNIO_OK on success
*----------------------------------------------------------------------------
* INPUTS   :
* OUTPUS   :
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
PNIO_UINT32 ICommon::uninit_irt_dma_ranges(void)
{
    struct t_dma_range dma_range;

    TRC_OUT(GR_STATE, LV_FCTPUB1, "-> uninit_irt_dma_ranges");

    dma_range.user_id = m_pCpAdapter->user_id;
    dma_range.offset_in = 0;
    dma_range.length_in = 0;
    dma_range.offset_out = 0;
    dma_range.length_out = 0;

    if(DPR_DRV_IOCTL(m_pCpAdapter->fd_control, CP16XX_IOC_SET_DMA_RANGE,
        &dma_range, sizeof(dma_range), 0))
        return PNIO_ERR_DRIVER_IOCTL_FAILED;

    TRC_OUT(GR_STATE, LV_FCTPUB1, "<- uninit_irt_dma_ranges");

    return PNIO_OK;
}

PNIO_UINT32 ICommon::CP_cycle_stat (PNIO_CYCLE_STAT * pCycleStat, int MeasureNr)
{
    if(pCycleStat == NULL) {    /* reset counters */
        m_cycleStat.FaultOccurred = PNIO_TRUE; /* stop interrupt updating */
        strcpy(m_cycleStat.FaultTime, "");
        m_cycleStat.StartOp.Valid = PNIO_FALSE;
        m_cycleStat.StartOp.Min = 0;
        m_cycleStat.StartOp.Max = 0;
        m_cycleStat.OpDone.Valid = PNIO_FALSE;
        m_cycleStat.OpDone.Min = 0;
        m_cycleStat.OpDone.Max = 0;
        m_cycleStat.ApplDelay.Valid = PNIO_FALSE;
        m_cycleStat.ApplDelay.Min = 0;
        m_cycleStat.ApplDelay.Max = 0;

        m_ciPoolIndex = 0;
        m_actCycleCounter = 0;
        m_prevStartOp = 0;

        for(int i = 0; i < CI_POOL_ENTRY_NR; i++) {
            m_ciPool[i].Valid = PNIO_FALSE;
            m_ciPool[i].Entry.Type = PNIO_CP_CI_UNKNOWN;
            m_ciPool[i].Entry.CycleInfo.CycleCount = 0;
            m_ciPool[i].Entry.CycleInfo.ClockCount = 0;
            m_ciPool[i].Entry.CycleInfo.CountSinceCycleStart = 0;
        }
        /* reset all blocks */
        if(MeasureNr == -1) {
          m_ciMeasureNr = 0;
        }
        m_cycleStat.FaultOccurred = PNIO_FALSE; /* start interrupt updating */
    } else if((MeasureNr < -1) || (MeasureNr >= CI_POOL_MEASURE_NR)) {
      return PNIO_ERR_PRM_MEASURE_NUMBER;
    } else if(MeasureNr == -1) {
      *pCycleStat = m_cycleStat;
    } else if(MeasureNr<m_ciMeasureNr) {
      *pCycleStat = m_ciPoolExt[MeasureNr].ciPoolStatExt;
    } else {
      return PNIO_ERR_NO_CYCLE_INFO_DATA; /* no data available */
    }

    return PNIO_OK;
}

PNIO_UINT32 ICommon::CP_cycle_info (PNIO_CI_ENTRY * pCycleInfoEntry, int MeasureNr, PNIO_UINT32 Offset)
{
    PNIO_UINT32 index;

    if(!pCycleInfoEntry)
        return PNIO_ERR_PRM_POINTER;

    if(Offset >= CI_POOL_ENTRY_NR)
        return PNIO_ERR_PRM_CYCLE_OFFSET;

    if((MeasureNr < -1) || (MeasureNr >= CI_POOL_MEASURE_NR))
        return PNIO_ERR_PRM_MEASURE_NUMBER;

    if(MeasureNr == -1) {
        if(m_ciPoolIndex < Offset) {
            index = CI_POOL_ENTRY_NR + m_ciPoolIndex - Offset;
        } else {
            index = m_ciPoolIndex - Offset;
        }

        if(m_ciPool[index].Valid == PNIO_FALSE) {
            return PNIO_ERR_NO_CYCLE_INFO_DATA;
        }
        *pCycleInfoEntry = m_ciPool[index].Entry;
    } else {
        if(MeasureNr < m_ciMeasureNr) {
            if(m_ciPoolExt[MeasureNr].ciValidCount < Offset)
                return PNIO_ERR_NO_CYCLE_INFO_DATA;
            else if(m_ciPoolExt[MeasureNr].ciPoolExt[Offset].Valid == PNIO_FALSE)
                return PNIO_ERR_NO_CYCLE_INFO_DATA;
            else
                *pCycleInfoEntry = m_ciPoolExt[MeasureNr].ciPoolExt[Offset].Entry;
        } else
            return PNIO_ERR_NO_CYCLE_INFO_DATA;
    }

    return PNIO_OK;
}

void ICommon::setCycleStat(PNIO_CYCLE_INFO * pCycleInfo, PNIO_CP_CI_TYPE Type)
{
    /* no full sequence checking here as not all interrupts have to
     * be registered by PNIO Lib user */
    PNIO_UINT32 prevCycleCount;
    PNIO_UINT32 nextCycleCount;
    PNIO_UINT32 prevIndex = m_ciPoolIndex;

    if(m_cycleStat.FaultOccurred) {
        return;
    }

    /* New pool entry */
    if(m_ciPoolIndex < (CI_POOL_ENTRY_NR-1) ) {
      ++m_ciPoolIndex;
    }
    else {
        m_ciPoolIndex = 0;
    }

    m_ciPool[m_ciPoolIndex].Valid = PNIO_TRUE;
    m_ciPool[m_ciPoolIndex].Entry.Type = Type;
    m_ciPool[m_ciPoolIndex].Entry.CycleInfo = *pCycleInfo;

    /* opfault? */
    if(Type == PNIO_CP_CI_OPFAULT) {
        m_cycleStat.FaultOccurred = PNIO_TRUE;
        TRC_GetFormattedLocalTime(m_cycleStat.FaultTime, PNIO_CI_FAULT_TIME_LEN);
    }
    /* jumping cycle counter ? */
    else if(m_ciPool[prevIndex].Valid) {
        prevCycleCount = m_ciPool[prevIndex].Entry.CycleInfo.CycleCount;
        nextCycleCount = (PNIO_UINT16) (prevCycleCount + m_cycleCountEntity);
        if((prevCycleCount != pCycleInfo->CycleCount) &&
            (nextCycleCount != pCycleInfo->CycleCount)) {
            PNIO_UINT32 m_cycleCountEntityNew;
            /* first check if cycle_count_Entity has changed (due to configuration changes) */
            m_cycleCountEntityNew = LE_TO_CPU(*(PNIO_UINT32 *)(m_pCpAdapter->pErtecSwiBase + 0x11038));
            /* if there is no change, an error occurred. When entity changed, ignore change*/
            if(m_cycleCountEntityNew == m_cycleCountEntity) {
                m_cycleStat.FaultOccurred = PNIO_TRUE;
                TRC_GetFormattedLocalTime(m_cycleStat.FaultTime, PNIO_CI_FAULT_TIME_LEN);
                TRC_OUT02(GR_RT, LV_ERR, "ERROR cycle counter jumped, prev: %x act: %x ",
                    prevCycleCount ,pCycleInfo->CycleCount);
            } else {
                /* change in entity, so save the value for the next cycle */
                m_cycleCountEntity = m_cycleCountEntityNew;
            }
        }
    }
    if(PNIO_FALSE == m_cycleStat.FaultOccurred) {
        /* Calculate min/max */
        switch (Type) {
        case PNIO_CP_CI_STARTOP:
            if(m_cycleStat.StartOp.Valid) {
                if(m_cycleStat.StartOp.Min > pCycleInfo->CountSinceCycleStart) {
                    m_cycleStat.StartOp.Min = pCycleInfo->CountSinceCycleStart;
                } else if(m_cycleStat.StartOp.Max < pCycleInfo->CountSinceCycleStart) {
                    m_cycleStat.StartOp.Max = pCycleInfo->CountSinceCycleStart;
                }
            } else {
                m_cycleStat.StartOp.Min = m_cycleStat.StartOp.Max = pCycleInfo->CountSinceCycleStart;
                m_cycleStat.StartOp.Valid = PNIO_TRUE;
            }
            m_prevStartOp = pCycleInfo->ClockCount;
            break;
        case PNIO_CP_CI_OPDONE:
            if(m_cycleStat.OpDone.Valid) {
                if(m_cycleStat.OpDone.Min > pCycleInfo->CountSinceCycleStart) {
                    m_cycleStat.OpDone.Min = pCycleInfo->CountSinceCycleStart;
                } else if(m_cycleStat.OpDone.Max < pCycleInfo->CountSinceCycleStart) {
                    m_cycleStat.OpDone.Max = pCycleInfo->CountSinceCycleStart;
                }
            } else {
                m_cycleStat.OpDone.Min = m_cycleStat.OpDone.Max = pCycleInfo->CountSinceCycleStart;
                m_cycleStat.OpDone.Valid = PNIO_TRUE;
            }
            /* calculation of ApplDelay: ApplDelay = ClockCount(OpDone) - ClockCount(StartOp) */
            if(m_cycleStat.StartOp.Valid) {
                PNIO_UINT32 appDelay = pCycleInfo->ClockCount - m_prevStartOp;

                if(m_cycleStat.ApplDelay.Valid) {
                    if(m_cycleStat.ApplDelay.Min > appDelay) {
                        m_cycleStat.ApplDelay.Min = appDelay;
                    } else if(m_cycleStat.ApplDelay.Max < appDelay) {
                        m_cycleStat.ApplDelay.Max = appDelay;
                    }
                } else {
                    m_cycleStat.ApplDelay.Min = m_cycleStat.ApplDelay.Max = appDelay;
                    m_cycleStat.ApplDelay.Valid = PNIO_TRUE;
                }
            }
            break;
        default:
            break;
        }
    }
    /* error occurred. Now the statistic must be saved */
    else {
        /* check buffer. If there is no place for data, ignore them */
        if(m_ciMeasureNr < CI_POOL_MEASURE_NR - 1) {
            /* copy all values */
            m_ciPoolExt[m_ciMeasureNr].ciValidCount=0;
            /* copy upt to CI_POOL_ENTRY_NR entries */
            for(PNIO_UINT32 i = 0; i < CI_POOL_ENTRY_NR; i++) {
                if(CP_cycle_info(&m_ciPoolExt[m_ciMeasureNr].ciPoolExt[i].Entry, -1, i) == PNIO_OK) {
                    m_ciPoolExt[m_ciMeasureNr].ciPoolExt[i].Valid = PNIO_TRUE;
                    m_ciPoolExt[m_ciMeasureNr].ciValidCount++;
                } else {
                    m_ciPoolExt[m_ciMeasureNr].ciPoolExt[i].Valid = PNIO_FALSE;
                }
            }
            /* copy actual statistics */
            m_ciPoolExt[m_ciMeasureNr].ciPoolStatExt = m_cycleStat;
            /* increment Measure counter */
            m_ciMeasureNr++;
        }
        /* restart measure for the next block */
        CP_cycle_stat(NULL, 0);
    }
}

#ifdef IO_ROUTER
/* Has to be called before NewCycle-User_Cbf is registered */
PNIO_UINT32 ICommon::CP_register_cbf_transferwd(void (*cbf) (PNIO_CP_CBE_PRM  *, IController *))
{
    PNIO_UINT32 ret;
    m_pCbf_TransferWD = cbf;

    TRC_OUT(GR_STATE, LV_FCTPUB1, "-> CP_register_cbf_transferwd");

    ret = start_irt_thread("PNIO_CP_CBE_NEWCYCLE_IND", m_pCpAdapter,
        IRT_NEWCYCLE, CP16XX_NEWCYCLE);
    if(ret != PNIO_OK) {
        stop_irt_thread("PNIO_CP_CBE_NEWCYCLE_IND", m_pCpAdapter,
        IRT_NEWCYCLE, CP16XX_NEWCYCLE);
        m_pCbf_TransferWD = NULL;
    }

    TRC_OUT(GR_STATE, LV_FCTPUB1, "<- CP_register_cbf_transferwd");

    return ret;
}
#endif /* IO_ROUTER */

extern DPR_MUTEX PnioMutexController;
extern DPR_MUTEX PnioMutexDevice;
extern DPR_MUTEX PnioMutexWd;
#ifdef IO_ROUTER
extern DPR_MUTEX PnioMutexTransferWd;
#endif
void InitCriticalSections(void)
{
    DPR_MUTEX_CREATE_UNLOCKED(PnioMutexController);
    DPR_MUTEX_CREATE_UNLOCKED(PnioMutexDevice);
    DPR_MUTEX_CREATE_UNLOCKED(PnioMutexWd);
#ifdef IO_ROUTER
    DPR_MUTEX_CREATE_UNLOCKED(PnioMutexTransferWd);
#endif
}

void DestroyCriticalSections(void)
{
    DPR_MUTEX_DESTROY(PnioMutexController);
    DPR_MUTEX_DESTROY(PnioMutexDevice);
    DPR_MUTEX_DESTROY(PnioMutexWd);
#ifdef IO_ROUTER
    DPR_MUTEX_DESTROY(PnioMutexTransferWd);
#endif
}
