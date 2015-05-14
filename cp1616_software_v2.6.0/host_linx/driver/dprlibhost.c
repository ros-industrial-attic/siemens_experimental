/*****************************************************************************/
/*  Copyright (C) SIEMENS CORP., 2013 All rights reserved.*/
/*****************************************************************************/
/*  F i l e                dprlibhost.c                                      */
/*****************************************************************************/
/*  D e s c r i p t i o n:  interface definition which are used only between */
/*                          the modules of the dprlib                        */
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

#include "os.h"             /* system dependencies */
#include "dprintern.h"      /* common layer for FW and driver */
#include "fw1616dk_vers.h"  /* fw version constants FW_VER_MAJOR, FW_VER_MINOR1... */

/*===========================================================================
* FUNCTION : dprlib_start_threads
*----------------------------------------------------------------------------
* PURPOSE  :
*----------------------------------------------------------------------------
* RETURNS  : VOID
*----------------------------------------------------------------------------
* INPUTS   : VIOD
* OUTPUTS  :
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
static int dprlib_start_threads(CpData *pCP)
{
    int ret;

    DPRLIBLOGMSG("->\n");

    /* check if threads have to be stopped */
    if(DPR_ATOMIC_READ(pCP->dprStopAllThreads)) {
        DPRLIBERRMSG("dprStopAllThreads=true => no start - exit\n");
        return DPR_ERROR;
    }

    /* start all waiting threads and semaphores */
    ret = dprlib_start_all_tasks_and_semaphores(pCP);

    DPR_WRITE_UINT32(DPR_INT_DPR_READY, &pCP->pDpramConfigArea->HPIIOData);
    DPR_WRITE_UINT32(DPR_INT_DPR_READY, &pCP->pDpramConfigArea->HPIMessage);
    DPR_WRITE_UINT32(DPR_INT_DPR_READY, &pCP->pDpramConfigArea->HPIMessageRC);

    DPRLIBLOGMSG("<- ret=%d (0=DPR_ERR, 1=DPR_OK) \n", ret);

    return ret;
}

/*===========================================================================
* FUNCTION : dprlib_trigger_intr
*----------------------------------------------------------------------------
* PURPOSE  : notify host - interrupt call
*----------------------------------------------------------------------------
* RETURNS  :
*----------------------------------------------------------------------------
* INPUTS   :
* OUTPUTS  :
*----------------------------------------------------------------------------
* COMMENTS : change values in the register to notify host.
*==========================================================================*/
DPR_VOID dprlib_trigger_intr(CpData *pCP)
{
    pCP->trigger_irq(pCP->parent);
    DPRLIBCHATMSG("IRQ Host->FW, C=0x%08x, M=0x%08x, MRC=0x%08x\n",
        DPR_READ_UINT32(&pCP->pDpramConfigArea->HPIConfig),
        DPR_READ_UINT32(&pCP->pDpramConfigArea->HPIMessage),
        DPR_READ_UINT32(&pCP->pDpramConfigArea->HPIMessageRC));
}

/*===========================================================================
* FUNCTION : dprlib_trigger_self_intr
*----------------------------------------------------------------------------
* PURPOSE  : notify itself - interrupt call
*----------------------------------------------------------------------------
* RETURNS  :
*----------------------------------------------------------------------------
* INPUTS   :
* OUTPUTS  :
*----------------------------------------------------------------------------
* COMMENTS : change values in the register to notify host.
*==========================================================================*/
DPR_VOID dprlib_trigger_self_intr(CpData *pCP)
{
    /* dprlib_trigger_self_intr:  this call is not needed on host */
    DPR_ASSERT(0);
}

/*===========================================================================
* FUNCTION : dprlib_init_channels
*----------------------------------------------------------------------------
* PURPOSE  : Initialize all Channels
*----------------------------------------------------------------------------
* RETURNS  :
*----------------------------------------------------------------------------
* INPUTS   :
* OUTPUTS  :
*----------------------------------------------------------------------------
* COMMENTS : if requested size not available then reset the size to 0.
*            host must check this
*==========================================================================*/
int dprlib_init_channels(CpData *pCP)
{
    int i, cfg_loop_cnt,cfg_loop_max;
    DPR_UINT32 tmpSize, startoffset, endoffset;
    RING  *pRings = pCP->pDpramConfigArea->Rings;
    DPR_CHANNEL_INFO *pChnlInfo = pCP->dprlib_channel_info;

    DPRLIBLOGMSG("->\n");

    dprlib_post_local_version(pCP);

    cfg_loop_cnt = 0;
    cfg_loop_max = 100;

    do {
        DPRLIBLOGMSG("CONF_CHNLs loop: cnt=%d max=%d \n",cfg_loop_cnt, cfg_loop_max);
        /* repeat configuration */
        if(DPR_ATOMIC_READ(pCP->dprStopAllThreads)) {
            DPRLIBERRMSG("dprStopAllThreads requested - exit\n");
            return DPR_ERROR;
        }

        /* first copy all requested sizes into the dpram */
        /* for(i = 0; i < DPR_CFG_MAX_CHANNEL; i++) {
            DPRLIBLOGMSG("Chnl(%d) TDRSize %u, RDRSize %u\n", i,
                pChnlInfo[i].TDRSize, pChnlInfo[i].RDRSize);
            // set requested sizes in DPR structures
            DPR_WRITE_UINT32(0, &pRings[DPRLIB_WRITE_RING(i)].pRingStart);
            DPR_WRITE_UINT32(pChnlInfo[i].TDRSize, &pRings[DPRLIB_WRITE_RING(i)].pRingEnd);
            DPR_WRITE_UINT32(0, &pRings[DPRLIB_READ_RING(i)].pRingStart);
            DPR_WRITE_UINT32(pChnlInfo[i].RDRSize, &pRings[DPRLIB_READ_RING(i)].pRingEnd);
        }*/

        /* HR 18.03.2008
        no request for channel size from host necessary;
        firmware set channel size, host must work with the channel size setted by firmare */

        /* trigger interrupt at firmware */
        DPRLIBLOGMSG("trigger DPR_INT_CONF_CHNL interrupt to FW\n");
        DPR_WRITE_UINT32(DPR_INT_CONF_CHNL, &pCP->pDpramConfigArea->HPIConfig);
        dprlib_trigger_intr(pCP);

        /* wait for response from firmware */
        DPRLIBLOGMSG("wait till config interrupt comes from FW\n");
    } while(DPR_SEM_WAIT_TIME(pCP->sem_config_conf, 1000) && cfg_loop_cnt < cfg_loop_max);

    if (cfg_loop_cnt >= cfg_loop_max) {
        DPRLIBERRMSG("No int from FW: max reached max=%d - exit\n",cfg_loop_max);
        return DPR_ERROR;
    }
    else {
        DPRLIBLOGMSG("OK int from FW arrived \n");
    }

    if(DPR_ATOMIC_READ(pCP->dprStopAllThreads)) {
        DPRLIBERRMSG("dprStopAllThreads - exit\n");
        return DPR_ERROR;
    }

    /* check results from firmware */
    for (i = 0; i< DPR_CFG_MAX_CHANNEL; i++) {
        /* check TDR size */
        startoffset = DPR_READ_UINT32(&(pRings[DPRLIB_WRITE_RING(i)].pRingStart));
        endoffset = DPR_READ_UINT32(&(pRings[DPRLIB_WRITE_RING(i)].pRingEnd));
        tmpSize = endoffset - startoffset;
        if(pChnlInfo[i].TDRSize > tmpSize) {
            DPRLIBERRMSG("Received invalid TDRSize from FW. Chnl(%d) requested %u, got %u\n",
                i, pChnlInfo[i].TDRSize, tmpSize);
            dprlib_uninit_dpram(pCP, DPR_ERR_CHNL_INIT_INVALID_SIZE);
            return DPR_ERROR;
        }

        DPRLIBLOGMSG("Chnl(%d) TDRSize requested %u, got %u, startoffset 0x%x, endoffset 0x%x\n",
            i, pChnlInfo[i].TDRSize, tmpSize, startoffset, endoffset);
        pChnlInfo[i].TDRSize = tmpSize;

        /* check RDR size */
        startoffset = DPR_READ_UINT32(&(pRings[DPRLIB_READ_RING(i)].pRingStart));
        endoffset = DPR_READ_UINT32(&(pRings[DPRLIB_READ_RING(i)].pRingEnd));
        tmpSize = endoffset - startoffset;
        if(pChnlInfo[i].RDRSize > tmpSize) {
            DPRLIBERRMSG("ivalid RDRSize from FW Chnl(%d) requested %u, got %u ->uninit_dpram... \n",
                i, pChnlInfo[i].RDRSize, tmpSize);
            dprlib_uninit_dpram(pCP, DPR_ERR_CHNL_INIT_INVALID_SIZE);
            DPRLIBERRMSG("ERROR exit\n");
            return DPR_ERROR;
        }

        DPRLIBLOGMSG("Chnl(%d) RDRSize requested %u, got %u, startoffset 0x%x, endoffset 0x%x\n",
            i, pChnlInfo[i].RDRSize, tmpSize, startoffset, endoffset);
        pChnlInfo[i].RDRSize = tmpSize;
    } /* end for */
    DPRLIBLOGMSG("OK: bDriverConnected TRUE \n");
    DPR_ATOMIC_SET(pCP->bDriverConnected, 1);
    DPRLIBLOGMSG("<- ret DPR_OK\n");
    return DPR_OK;
}

/*===========================================================================
* FUNCTION : DPRLIB_start
*----------------------------------------------------------------------------
* PURPOSE  :
*----------------------------------------------------------------------------
* RETURNS  :
*----------------------------------------------------------------------------
* INPUTS   :
* OUTPUTS  :
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
int DPRLIB_start(void *arg)
{
    CpData *pCP = (CpData *)arg;
    DPR_VUINT32 total;
    int ret;

    DPRLIBLOGMSG("->\n");

    /* setup configuration data */
    DPR_ATOMIC_SET(pCP->dprStopAllThreads, 0);
        DPR_ATOMIC_SET(pCP->bDriverConnected, 0);

    total = 20000; /* 18 kB */
    memset(pCP->dprlib_channel_info, 0, sizeof(pCP->dprlib_channel_info));

    /*
    !!! ring size is defined jet in firmware
    pCP->dprlib_channel_info[DPR_PNIO_CMD_CHN].TDRSize = DPR_CHANNEL_SIZE;
    pCP->dprlib_channel_info[DPR_PNIO_CMD_CHN].RDRSize = DPR_CHANNEL_SIZE;
    pCP->dprlib_channel_info[DPR_PNIO_ALARM_CHN].TDRSize = 0;
    pCP->dprlib_channel_info[DPR_PNIO_ALARM_CHN].RDRSize = DPR_CHANNEL_SIZE;
    pCP->dprlib_channel_info[DPR_PNIO_MOD_CHN].TDRSize = 0;
    pCP->dprlib_channel_info[DPR_PNIO_MOD_CHN].RDRSize = 2 * DPR_CHANNEL_SIZE;
    pCP->dprlib_channel_info[DPR_PNIO_DATA_REC_CHN].TDRSize = DPR_CHANNEL_SIZE;
    pCP->dprlib_channel_info[DPR_PNIO_DATA_REC_CHN].RDRSize = DPR_CHANNEL_SIZE;
    pCP->dprlib_channel_info[DPR_MGT_CHN].TDRSize = DPR_MGT_CHANNEL_SIZE;
    pCP->dprlib_channel_info[DPR_MGT_CHN].RDRSize = DPR_MGT_CHANNEL_SIZE;
    pCP->dprlib_channel_info[DPR_L2_SEND_CHN].TDRSize = L2ETH_SEND_CHANNEL_MAX_LENGTH;
    pCP->dprlib_channel_info[DPR_L2_SEND_CHN].RDRSize = L2ETH_SEND_CHANNEL_MAX_LENGTH;
    pCP->dprlib_channel_info[DPR_L2_RECV_CHN].TDRSize = L2ETH_RECV_CHANNEL_MAX_LENGTH;
    pCP->dprlib_channel_info[DPR_L2_RECV_CHN].RDRSize = L2ETH_RECV_CHANNEL_MAX_LENGTH;

    !!! ring size is defined jet in firmware */

    if(!(ret = dprlib_init_dpram(pCP, &total)))
        DPRLIBLOGMSG("total dpram size %u\n", total);

    DPRLIBLOGMSG("<-\n");

    return ret;
}

/*===========================================================================
* FUNCTION : dprlib_init_dpram
*----------------------------------------------------------------------------
* PURPOSE  :
*----------------------------------------------------------------------------
* RETURNS  :
*----------------------------------------------------------------------------
* INPUTS   :
* OUTPUTS  :
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
int dprlib_init_dpram(CpData *pCP, DPR_VUINT32 *Size)
{
    DPR_UINT32 reqSize = *Size;
    int ret;
    int retry, retry_max;

    DPRLIBLOGMSG("->\n");

    /* check if already initialized */
    if(DPR_READ_UINT32(&pCP->pDpramConfigArea->DpramMaxSize) > 0) {
        DPRLIBLOGMSG("pDpramConfigArea->DpramMaxSize bereits nicht NULL 0x%08x\n",
            DPR_READ_UINT32(&pCP->pDpramConfigArea->DpramMaxSize));
    }

    /* start host - init all threads and register callback functions */
    if(dprlib_start_threads(pCP) != DPR_OK) {
        DPRLIBERRMSG("ERROR in dprlib_start_threads\n");
        dprlib_uninit_dpram(pCP, DPR_ERR_DPRAM_CRITICAL_ERROR);
        return DPR_ERROR;
    }

    retry = 0;
    retry_max = 5;
    do {
        /* repeat DPRAM configuration */
        retry++;
        DPRLIBLOGMSG("DPRAM config loop: retry-cnt=%d max=%d \n", retry, retry_max);

        if(DPR_ATOMIC_READ(pCP->dprStopAllThreads)) {
            DPRLIBERRMSG("dprStopAllThreads true => no start - exit\n");
            return DPR_ERROR;
        }

        /* set requested size in Dual port RAM */
        DPR_WRITE_UINT32(reqSize, &pCP->pDpramConfigArea->DpramMaxSize);

        /* set interrupt info value and send interrupt to FW */
        DPR_WRITE_UINT32(DPR_INT_CONF_DPRAM, &pCP->pDpramConfigArea->HPIConfig);
        dprlib_trigger_intr(pCP);

        DPRLIBLOGMSG("wait till config interrupt comes from FW cnt=%d\n",retry);
    } while(DPR_SEM_WAIT_TIME(pCP->sem_config_conf, 1000) && retry < retry_max);

    if(retry >= retry_max) {
        DPRLIBINFMSG("No interrupt from FW, -> dprlib_uninit_dpram: retry-max=%d \n", retry_max);
        dprlib_uninit_dpram(pCP, DPR_ERR_HOST_SHUTDOWN_REQUEST);
        DPRLIBERRMSG("retry-max reached - exit\n");
        return DPR_ERROR;
    } else {
        DPRLIBLOGMSG("OK interrupt from FW arrived \n");
    }

    /* check if module has to be stopped */
    if(DPR_ATOMIC_READ(pCP->dprStopAllThreads)) {
        DPRLIBERRMSG("dprStopAllThreads requested - exit\n");
        return DPR_ERROR;
    }

    *Size = DPR_READ_UINT32(&pCP->pDpramConfigArea->DpramMaxSize);

    DPRLIBLOGMSG("DPRAM config OK: reqSize=%u DpramMaxSize=%u ->init-channels...\n",reqSize, *Size);
    ret = dprlib_init_channels(pCP);

    DPRLIBLOGMSG("<- ret=%d (1=DPR_OK) \n", ret);
    return ret;
}

/*===========================================================================
* FUNCTION : DPRLIB_stop
*----------------------------------------------------------------------------
* PURPOSE  : to shut down
*----------------------------------------------------------------------------
* RETURNS  :
*----------------------------------------------------------------------------
* INPUTS   :
* OUTPUTS  :
*----------------------------------------------------------------------------
* COMMENTS : this function is called to make a shutdown of the
*            DPR interface.
*==========================================================================*/
void DPRLIB_stop(void *arg, unsigned long errorcode)
{
    CpData *pCP = (CpData *)arg;

    DPRLIBLOGMSG("-> errorcode %lu\n", errorcode);
        DPR_ATOMIC_SET(pCP->bDriverConnected, 0);
    pCP->wakeup_daemon(pCP->parent, DPRLIB_STOP);
    DPRLIBLOGMSG("<-\n");
}

/*===========================================================================
* FUNCTION : dprlib_uninit_dpram
*----------------------------------------------------------------------------
* PURPOSE  :
*----------------------------------------------------------------------------
* RETURNS  :
*----------------------------------------------------------------------------
* INPUTS   :
* OUTPUTS  :
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
void dprlib_uninit_dpram(CpData *pCP, DPR_UINT32 errorcode)
{
    DPRLIBLOGMSG("->\n");

    if(DPR_ERR_HOST_SHUTDOWN_REQUEST != errorcode &&
        DPR_ERR_FW_SHUTDOWN_REQUEST != errorcode) {
        DPR_WRITE_UINT32(DPR_INT_DPR_ERR, &pCP->pDpramConfigArea->HPIMessage);
    }
    DPR_WRITE_UINT32(DPR_INT_DPR_SHUTDOWN, &pCP->pDpramConfigArea->HPIConfig);
    dprlib_trigger_intr(pCP);

    dprlib_stop_all_tasks_and_semaphores(pCP);

    DPRLIBLOGMSG("<-\n");
}

/*===========================================================================
* FUNCTION : DPRLIB_restart_req
*----------------------------------------------------------------------------
* PURPOSE  : request to restart FW
*----------------------------------------------------------------------------
* RETURNS  :
*----------------------------------------------------------------------------
* INPUTS   :
* OUTPUTS  :
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
int DPRLIB_restart_req(void *arg)
{
    CpData *pCP = (CpData *)arg;

    DPRLIBLOGMSG("->\n");
    pCP->wakeup_daemon(pCP->parent, DPRLIB_RESET_FW);
    DPRLIBLOGMSG("<-\n");

    return DPR_OK;
}
