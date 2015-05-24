/****************************************************************************
*  Copyright (C) SIEMENS CORP., 2013 All rights reserved.
*****************************************************************************
* FILE NAME    : ldah_flash.cpp
* ---------------------------------------------------------------------------
* DESCRIPTION  : configuration local download functions
*****************************************************************************
* Diese Software ist Freeware. Sie wird Ihnen unentgeltlich zur Verfuegung  *
* gestellt. Sie darf frei kopiert, modifiziert und benutzt sowie an Dritte  *
* weitergegeben werden. Die Software darf nur unter Beibehaltung aller      *
* Schutzrechtsvermerke sowie nur vollstaedig und unveraendert weitergegeben *
* werden. Die kommerzielle Weitergabe an Dritte (z.B. im Rahmen von         *
* Share-/Freeware-Distributionen) ist nur mit vorheriger schriftlicher      *
* Genehmigung der Siemens Aktiengesellschaft erlaubt.                       *
* DA DIE SOFTWARE IHNEN UNENTGELTLICH UEBERLASSEN WIRD, KOENNEN DIE AUTOREN *
* UND RECHTSINHABER DAFUER KEINE HAFTUNG UEBERNEHMEN. IHRE BENUTZUNG ERFOLGT*
* AUF EIGENE GEFAHR UND VERANTWORTUNG. DIE AUTOREN UND RECHTSINHABER HAFTEN *
* NUR FUER VORSATZ UND GROBE FAHRLAESSIGKEIT. WEITERGEHENDE ANSPRUECHE SIND *
* AUSGESCHLOSSEN. INSBESONDERE HAFTEN DIE AUTOREN UND RECHTSINHABER NICHT   *
* FUER ETWAIGE MAENGEL ODER FOLGESCHAEDEN.                                  *
* Falls Sie Fehler in der Software bemerken, teilen Sie es uns bitte mit.   *
*                                                                           *
* This Software is Freeware. It is distributed for free. You may copy,      *
* modify and use it for free as well as distribute it to others. You may    *
* only distribute it as a whole, unmodified and with all trademarks and     *
* copyright notices. You may not distribute it commercially (e.g. as a      *
* Share-/Freeware-Distributor) without the express written permission of    *
* Siemens Aktiengesellschaft. SINCE THIS SOFTWARE IS DISTRIBUTED FOR FREE,  *
* IT IS PROVIDED "AS IS" WITHOUT ANY REPRESENTATION OR WARRANTY OF ANY KIND *
* EITHER EXPRESSED OR IMPLIED INCLUDING BUT NOT LIMITED TO IMPLIED          *
* WARRANTIES FOR MERCHANTIBILITY OR FITNESS FOR USE. ANY USE OF THE         *
* APPLICATION EXAMPLE IS ON YOUR OWN RISK AND RESPONSIBILITY.               *
* If you happen to find any faults in it please tell us.                    *
*****************************************************************************/

#include "os.h"
#include "cp16xx.h"
#include "tracemcrm.h"
#include "pnioerrx.h"

#include "lda_spec.h"
#include "ldah_flash.h"

#include "dpr_msg.h"
#include "sysa_fwlda_ifhost.h"
#include "servusrx.h"

//#define DUMP_SDB_CONTENT
#undef DUMP_SDB_CONTENT

#if defined DUMP_SDB_CONTENT
  void sysa_dump_buf( void *inBuff, unsigned int length );
  extern "C" void __cdecl dbg_printf(char *szFormat, ... );
#endif // DUMP_SDB_CONTENT

/* external functions */
void ldah_close(DPR_DRV_HANDLE control_fd, struct t_register_app *app, void *mapStart);
DPR_UINT32 ldah_open(unsigned long cp_idx, struct t_register_app *app, DPR_CHAR **ppmapStart, DPR_DRV_HANDLE *con_fd);

/* external objects */
extern CTrmWrapper ti;

DPR_UINT32 ldah_init_ovssdb_list(LDA_OVSSDB_LIST_HEADER ** p_hdr)
{
  if(*p_hdr != 0) {
    TRM_OUT00(&ti, GR_MGT, LV_ERR, "*p_hdr must be equal 0");
    return PNIO_ERR_INTERNAL;
  }

  *p_hdr = (LDA_OVSSDB_LIST_HEADER *)DPR_VMALLOC(sizeof(LDA_OVSSDB_LIST_HEADER));
  if(*p_hdr == 0) {
    TRM_OUT00(&ti, GR_MGT, LV_ERR, "can't allocate p_hdr");
    return PNIO_ERR_OS_RES;
  }

  (*p_hdr)->Type = 0x00000A0A; /* as valid OVSSDB_LIST Type */
  (*p_hdr)->Version = 0; /* use 0 first */
  (*p_hdr)->HdrLen = sizeof(LDA_OVSSDB_LIST_HEADER);        /* length of LDA_OVSSDB_LIST_HEADER */

  (*p_hdr)->SDBListLen = sizeof(LDA_OVSSDB_LIST_HEADER);    /* length of whole data block = LDA_OVSSDB_LIST_HEADER + OVS_LIST_DIRECTORY + SDB-Datas */

  (*p_hdr)->SDBDirCnt = 0;      /* count of LDA_OVSSDB_LIST_DIR_ITEM items (Section 1) */
  (*p_hdr)->SDBDatasOffset = 0; /* offset of Section 2 */
  (*p_hdr)->SDBDatasLen = 0;    /* length of Section 2 */

  (*p_hdr)->SDBDir = 0; /* reserved, for internal use only */
  (*p_hdr)->SDBDatas  = 0;  /* reserved, for internal use only */

  return PNIO_OK;
}

DPR_UINT32 ldah_free_ovssdb_list(LDA_OVSSDB_LIST_HEADER ** p_hdr)
{
  if(!(*p_hdr)) {
    TRM_OUT00(&ti, GR_MGT, LV_WARN, "double free of p_hdr or corruption");
    return PNIO_OK;
  }

  if((*p_hdr)->SDBDatas) {
    DPR_VFREE((*p_hdr)->SDBDatas);
  }

  if((*p_hdr)->SDBDir) {
    DPR_VFREE((*p_hdr)->SDBDir);
  }

  DPR_VFREE(*p_hdr);

  *p_hdr = 0;

  return PNIO_OK;
}

DPR_UINT32 ldah_add_sdb_to_ovssdb_list(LDA_OVSSDB_LIST_HEADER *p_hdr, DPR_UINT32 sdb_ovsid, DPR_CHAR *sdb_data)
{
  DPR_UINT32 fct_ret = PNIO_OK;
  LDA_OVS_BST * ovssdb_hdr = (LDA_OVS_BST *)sdb_data;
  DPR_UINT32 ovssdb_len = lda_be32_2_nat(ovssdb_hdr->head.blk_len);
  DPR_VOID * p_tmp = 0;

  /* add OVS_SDB in OVSSDB_LIST */
  p_tmp = DPR_VREALLOC(p_hdr->SDBDir, (p_hdr->SDBDirCnt + 1) * sizeof(LDA_OVSSDB_LIST_DIR_ITEM));
  if(p_tmp) {

    p_hdr->SDBDir = (LDA_OVSSDB_LIST_DIR_ITEM*)p_tmp;

    p_hdr->SDBDir[p_hdr->SDBDirCnt].SDBNumber = (DPR_UINT16)lda_be16_2_nat(ovssdb_hdr->head.blk_nr);

    if(p_hdr->SDBDir[p_hdr->SDBDirCnt].SDBNumber >= 1000) {
      p_hdr->SDBDir[p_hdr->SDBDirCnt].SDBContent = (DPR_UINT16)lda_be16_2_nat(ovssdb_hdr->info.sdb_ident);
    } else {
      p_hdr->SDBDir[p_hdr->SDBDirCnt].SDBContent = 0xffff;
    }

    p_hdr->SDBDir[p_hdr->SDBDirCnt].SDBLen = ovssdb_len;
    p_hdr->SDBDir[p_hdr->SDBDirCnt].VmdID = sdb_ovsid;
    p_hdr->SDBDir[p_hdr->SDBDirCnt].OffsetOfSdbBody = p_hdr->SDBDatasLen;
    memcpy(p_hdr->SDBDir[p_hdr->SDBDirCnt].SdbTimeStamp, ovssdb_hdr->head.time_stamp1, LDA_OVS_TIMESTAMP_LEN);

    /*TRM_OUT02(&ti, GR_INIT, LV_INFO, "ldah_add_sdb_to_ovssdb_list:: timestamp=%08x%04x",
               *(DPR_UINT32*)&ovssdb_hdr->head.time_stamp1[0], *(DPR_UINT16*)&ovssdb_hdr->head.time_stamp1[4]);*/


    p_hdr->SDBDirCnt++;
  } else {
    TRM_OUT01(&ti, GR_MGT, LV_ERR, "can't allocate p_hdr->SDBDir with SDBDirCnt=%d",
              p_hdr->SDBDirCnt + 1);
    return PNIO_ERR_OS_RES;
  }

  p_tmp = DPR_VREALLOC(p_hdr->SDBDatas, p_hdr->SDBDatasLen + ovssdb_len);
  if(p_tmp) {

    p_hdr->SDBDatas = (DPR_CHAR*)p_tmp;

    memcpy(p_hdr->SDBDatas + p_hdr->SDBDatasLen, sdb_data, ovssdb_len);

    p_hdr->SDBDatasLen += ovssdb_len;
  } else {
    TRM_OUT01(&ti, GR_MGT, LV_ERR, "can't allocate p_hdr->SDBDatas with len=%d",
              p_hdr->SDBDatasLen + ovssdb_len);

    return PNIO_ERR_OS_RES;
  }

  return fct_ret;
}

extern "C"
int ldah_compare_ovssdb_lhdr_items(const void *i1, const void *i2)
{
  LDA_OVSSDB_LIST_DIR_ITEM *p1 = (LDA_OVSSDB_LIST_DIR_ITEM*)i1,
                                 *p2 = (LDA_OVSSDB_LIST_DIR_ITEM*)i2;
  return(p2->VmdID - p1->VmdID); /* from big to low */
}

DPR_UINT32 ldah_create_ovssdb_list(DPR_CHAR *p_data, DPR_UINT32 data_length,
                                   LDA_OVSSDB_LIST_HEADER ** p_ovssdb_list)
{
  DPR_UINT32 i, ret, cur_xdb_offset = 0 ;
  LDA_XDB_HEADER *pXdbHdr = (LDA_XDB_HEADER*)p_data;
  LDA_XDB_CON_HEADER *pConHdr = 0;
  LDA_XDB_CON_SDB_DIR_HEADER *pConSdbDirHdr = 0;
  LDA_XDB_CON_SDB_DIR_ITEM *pConSdbDirItem = 0;
  LDA_OVS_BST *pSdbHdr = 0;

  /*LDA_XDB_SDB_HEADER *pXdbsdbHdr;*/


  ret = ldah_init_ovssdb_list(p_ovssdb_list);
  if(ret != PNIO_OK) {
    return ret;
  }

  /*TRM_OUT01(&ti, GR_MGT, LV_INFO, "len=%d XDB:", data_length);*/
  /*TRM_OUTD(&ti, GR_MGT, LV_INFO, p_data, data_length);*/

  if(pXdbHdr->HdrType != LDA_XDB_HEADER_TYPE || pXdbHdr->HdrId != LDA_XDB_HEADER_ID){
      TRM_OUT02(&ti, GR_INIT, LV_ERR, "unknown HEADER_TYPE=x%02x or HEADER_ID=x%02x; corrupted xdb data or old xdb format",
                     pXdbHdr->HdrType, pXdbHdr->HdrId);
      ldah_free_ovssdb_list(p_ovssdb_list);
      return PNIO_ERR_CORRUPTED_DATA;
  }

  if(lda_le16_2_nat(pXdbHdr->HdrLen) > data_length){
      TRM_OUT02(&ti, GR_INIT, LV_ERR, "pXdbHdr->HdrLen(0x%x) > data_length(0x%x); corrupted xdb data",
                     lda_le16_2_nat(pXdbHdr->HdrLen), data_length);
      ldah_free_ovssdb_list(p_ovssdb_list);
      return PNIO_ERR_CORRUPTED_DATA;
  }

  pConHdr = (LDA_XDB_CON_HEADER *)(p_data + lda_le16_2_nat(pXdbHdr->HdrLen));
  cur_xdb_offset = lda_le16_2_nat(pXdbHdr->HdrLen);

  do {
    TRM_OUT04(&ti, GR_INIT, LV_INFO, "XDB-Container len=%x type=%x id=%x vmdid=0x%x",
              lda_le32_2_nat(pConHdr->ConLen), pConHdr->ConType, pConHdr->ConId, pConHdr->ConVmdid);

    if(lda_le32_2_nat(pConHdr->ConLen) < sizeof(LDA_XDB_CON_HEADER) ||
        lda_le32_2_nat(pConHdr->ConLen) > (data_length - cur_xdb_offset)) {
      TRM_OUT01(&ti, GR_INIT, LV_ERR, "empty or corrupted container data, container len = %x", lda_le32_2_nat(pConHdr->ConLen));
      ldah_free_ovssdb_list(p_ovssdb_list);
      return PNIO_ERR_CORRUPTED_DATA;
    }

    pConSdbDirHdr = (LDA_XDB_CON_SDB_DIR_HEADER *)((DPR_CHAR*)pConHdr + sizeof(LDA_XDB_CON_HEADER));
    cur_xdb_offset += sizeof(LDA_XDB_CON_HEADER);

    if((lda_le32_2_nat(pConHdr->ConLen) - sizeof(LDA_XDB_CON_HEADER)) < lda_le32_2_nat(pConSdbDirHdr->SdbDirLen)) {
      TRM_OUT01(&ti, GR_INIT, LV_ERR, "empty or corrupted container data, container len < SdbDirLen(0x%0x)",
                                       lda_le32_2_nat(pConSdbDirHdr->SdbDirLen));
      ldah_free_ovssdb_list(p_ovssdb_list);
      return PNIO_ERR_CORRUPTED_DATA;
    }

    TRM_OUT02(&ti, GR_INIT, LV_INFO, "XDB-Container directory has 0x%x sdbs dir_len=0x%x",
        lda_le32_2_nat(pConSdbDirHdr->SdbDirItemsNumber), lda_le32_2_nat(pConSdbDirHdr->SdbDirLen));

    pConSdbDirItem = (LDA_XDB_CON_SDB_DIR_ITEM *)((DPR_CHAR*)pConSdbDirHdr + sizeof(LDA_XDB_CON_SDB_DIR_HEADER));
    cur_xdb_offset += sizeof(LDA_XDB_CON_SDB_DIR_HEADER);

    for(i=0; i< lda_le32_2_nat(pConSdbDirHdr->SdbDirItemsNumber); i++){

      TRM_OUT04(&ti, GR_INIT, LV_INFO, "SDB directory item nr=x%x content=x%x offset=x%x len=x%x",
                  lda_le16_2_nat(pConSdbDirItem->SdbNumber), lda_le16_2_nat(pConSdbDirItem->SdbContent),
                    lda_le32_2_nat(pConSdbDirItem->SdbOffset), lda_le32_2_nat(pConSdbDirItem->SdbLen));

      pSdbHdr = (LDA_OVS_BST *)((DPR_CHAR*)pConHdr + lda_le32_2_nat(pConSdbDirItem->SdbOffset));
      TRM_OUTD(&ti, GR_INIT, LV_INFO, (DPR_CHAR*)pSdbHdr, 16);
      ldah_add_sdb_to_ovssdb_list(*p_ovssdb_list, pConHdr->ConVmdid, (DPR_CHAR*)pSdbHdr);
      pConSdbDirItem++;
    }

    pConHdr = (LDA_XDB_CON_HEADER*)((DPR_CHAR*)pConHdr + lda_le32_2_nat(pConHdr->ConLen));
  }while((DPR_CHAR*)pConHdr < (p_data + data_length));

  /* sort SDBs list(LDA_OVSSDB_LIST_HEADER::SDBDir) by LDA_OVSSDB_LIST_DIR_ITEM::VmdID */
  qsort((*p_ovssdb_list)->SDBDir, (*p_ovssdb_list)->SDBDirCnt, sizeof((*p_ovssdb_list)->SDBDir[0]), ldah_compare_ovssdb_lhdr_items);

  return PNIO_OK;
}

/*===========================================================================
* FUNCTION : procMgtChannelReadDownload
*----------------------------------------------------------------------------
* PURPOSE  : callback processing messages from the cp
*----------------------------------------------------------------------------
* RETURNS  : -
*----------------------------------------------------------------------------
* INPUTS   :
* OUTPUS   :
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
static DPR_THREAD_RETURN_TYPE DPR_THREAD_DECL procMgtChannelReadDownload(void *arg)
{
  CP_FWLDA *my_channel = (struct CP_FWLDA *)arg;

  int ret;

  TRM_OUT00(&ti, GR_STATE, LV_INFO, "-> procMgtChannelReadDownload rcv loop...");

  /* loop till shutdown req */
  while(!my_channel->stop_thread) {
    TRM_OUT01(&ti, GR_CHNL, LV_FCTCLBF, "procMgtChannelReadDownload: wait for new msg, max_rcv_len=%d",
              my_channel->user_pool_length);

    ret = DPR_DRV_READ(my_channel->mgt_chnl, my_channel->user_pool_ptr,
                       my_channel->user_pool_length);                  // == ReadDeviceDriver

    if(ret < 0) {
      TRM_OUT01(&ti, GR_CHNL, LV_ERR,
                "procMgtChannelReadDownload: read error ret %d", ret);
      continue;
    } else if(!ret && DPR_DRV_ERROR(my_channel->mgt_chnl)) {
      TRM_OUT00(&ti, GR_CHNL, LV_ERR,
              "procMgtChannelReadDownload: read error, ret 0, ferror()");
      continue;
    } else if((unsigned long)ret > my_channel->user_pool_length) {
      TRM_OUT01(&ti, GR_CHNL, LV_ERR,
                "procMgtChannelReadDownload: buffer is too small, need %d bytes", ret);
      continue;
    }

    TRM_OUT01(&ti, GR_CHNL, LV_INFO, "procMgtChannelReadDownload: OK! read %d bytes", ret);

    DPR_SEM_POST(my_channel->sem_cnf_received);
    // because of segmented send fw->host synchronisation with data consumer is necessary
    DPR_TASK_DELAY(30);

  } // end while (not stop)

  TRM_OUT00(&ti, GR_STATE, LV_INFO, "<- procMgtChannelReadDownload");

  DPR_THREAD_END();
}

/*-----------------------------------------------------------------------------
 * Name  : ldah_mgt_ch_read
 * Descr : reads data from device (driver)
 * Param :
 *  [ IN]: *channel           Ptr to channel struct
 *  [OUT]: *p_received_len    Num of bytes received
 * Return: ==0: PNIO_OK, != 0 PNIO_ERR_xxx
 */
DPR_UINT32 ldah_mgt_ch_read(CP_FWLDA *channel, unsigned long *p_received_len)
{
  CP_FWLDA *my_channel = channel;
  int ret;
  DPR_UINT32 fct_ret = PNIO_OK;

  TRM_OUT01(&ti, GR_STATE, LV_INFO, ">ldah_mgt_ch_read: rcv-pool-len=%lu",my_channel->user_pool_length);

  int retry_count = 0;
  do {
      ret = (int)DPR_DRV_READ(my_channel->mgt_chnl, my_channel->user_pool_ptr,
                          my_channel->user_pool_length);                  // == ReadDeviceDriver
      retry_count++;
  } while ( (ret == -EAGAIN)  && (retry_count <= 5) );

  TRM_OUT02(&ti, GR_STATE, LV_INFO, " ldah_mgt_ch_read: ret=%d retry_count=%d ", ret, retry_count);

    if(ret < 0) {
      TRM_OUT01(&ti, GR_CHNL, LV_ERR, "ldah_mgt_ch_read: read error ret %d", ret);
      fct_ret = PNIO_ERR_INTERNAL;
    } else if(!ret && DPR_DRV_ERROR(my_channel->mgt_chnl)) {
      TRM_OUT00(&ti, GR_CHNL, LV_ERR, "ldah_mgt_ch_read: read error, ret 0, ferror()");
      fct_ret = PNIO_OK; // PNIO_ERR_INTERNAL;  no data -> ignore error, trace only
    } else if((unsigned long)ret > my_channel->user_pool_length) {
      TRM_OUT01(&ti, GR_CHNL, LV_ERR, "ldah_mgt_ch_read: buffer is too small, need %d bytes", ret);
      fct_ret = PNIO_ERR_VALUE_LEN;
    } else if((unsigned long)ret > 0 && (unsigned long)ret <= my_channel->user_pool_length) {
        /* receive ok */
        TRM_OUT01(&ti, GR_CHNL, LV_INFO, "ldah_mgt_ch_readad: OK! read %d bytes", ret);
        *p_received_len = (unsigned long)ret;
        fct_ret = PNIO_OK;
    }

    TRM_OUT02(&ti, GR_STATE, LV_INFO, "<ldah_mgt_ch_read: received-bytes=%lu pnio-result=%#x", *p_received_len, fct_ret );
    return fct_ret;
} /* end of ldah_mgt_ch_read */


/*-----------------------------------------------------------------------------
 * Name  : ldah_sr_rqb_ext
 * Descr : Send Receive Requestblock. Extended version with segmentation.
 * Return: ==0: PNIO_OK, != 0 PNIO_ERR_INTERNAL, ...
 */
DPR_UINT32 ldah_sr_rqb_ext(CP_FWLDA *channel, DPR_UINT32 tm_ms, FWLDA_OPCODE ldaf_opcode,
                           DPR_UINT32 vmdid, DPR_UINT32 add_data_len, DPR_CHAR * add_data)
{
  size_t ret;
  DPR_UINT32 total_requested_len = sizeof(FWLDA_DPR_RQB) + add_data_len;
  DPR_UINT32 rqbLen;
  FWLDA_DPR_RQB *pRqb     = NULL;  /* RQB to be send */
  FWLDA_DPR_RQB *pRcvBuff = NULL;  /* ptr to receive buffer */
  FWLDA_OPCODE ldaf_opc;
  DPR_CHAR *p_dt;
  DPR_UINT32 dt_len;
  DPR_UINT32 dt_total_sent = 0;
  DPR_UINT32 send_buf_len;         /* max available send buffer len in the DPR channel */
  DPR_UINT32 dt_available_rqb_len; /* available num of data bytes in the request block */
  DPR_UINT32 result = PNIO_OK;
  int seg_num   = 0;
  int sem_ret;

  TRM_OUT02(&ti, GR_INIT, LV_INFO, ">ldah_sr_rqb_ext: total_requested_len=%lu send_pool_length=%lu ", total_requested_len, channel->send_pool_length);

  if ( add_data_len == 0 || add_data == NULL ) {
      TRM_OUT00(&ti, GR_INIT, LV_WARN, "<ldah_sr_rqb_ext: NO DATA!");
      return PNIO_OK;
  }

  if ( channel->send_pool_length <= 64  ) {
      send_buf_len = channel->send_pool_length;
  }
  else {
      send_buf_len = channel->send_pool_length - 64;
  }

  if(total_requested_len <= send_buf_len ) {
    /* normal single send */
    ldaf_opc = ldaf_opcode;   /* FWLDA_OP_DOWNLOAD_SDB_RQB  */
    p_dt   = add_data;
    dt_len = add_data_len;
  }
  else {
    /* segmented send */
    ldaf_opc = FWLDA_OP_DOWNLOAD_SDB_RQB_SEGM;
    p_dt   = add_data;
    dt_len = send_buf_len - sizeof(FWLDA_DPR_RQB);  /* netto data == sdb content */
  }

  rqbLen = sizeof(FWLDA_DPR_RQB) + dt_len;
  dt_available_rqb_len = rqbLen - sizeof(FWLDA_DPR_RQB);

  pRqb = (FWLDA_DPR_RQB*)DPR_VMALLOC(rqbLen);
  if(!pRqb) {
    TRM_OUT00(&ti, GR_INIT, LV_ERR, "<ldah_sr_rqb_ext: RQB alloc memory failed");
    return PNIO_ERR_NO_RESOURCE;
  }

  pRqb->dpr_hdr.hostref = CPU_TO_LE(channel->user_id);
  pRqb->dpr_hdr.subsystem = FWLDA_SUBSYS_INDEX;  /* Fix: AP01394131 */
  pRqb->dpr_hdr.userid = CPU_TO_LE(0xabcd);
  pRqb->dpr_hdr.userdatalength = CPU_TO_LE(rqbLen);

  pRqb->fwlda_rqb.opcode = (FWLDA_OPCODE)CPU_TO_LE(ldaf_opc);
  pRqb->fwlda_rqb.ovs_inst_id = CPU_TO_LE(vmdid);
  pRqb->fwlda_rqb.rqb_len = CPU_TO_LE(sizeof(pRqb->fwlda_rqb) + dt_len);

  if(dt_len) {
    memcpy((DPR_CHAR*)pRqb + sizeof(FWLDA_DPR_RQB), p_dt, dt_len);
  }

  do {
      seg_num += 1;

      TRM_OUT02(&ti, GR_CHNL, LV_INFO, "ldah_sr_rqb_ext: DRV send...  seg=%d len=%d ",seg_num, rqbLen);
      ret = DPR_DRV_WRITE(channel->mgt_chnl, pRqb, rqbLen);

      if(ret != rqbLen) {
        TRM_OUT03(&ti, GR_CHNL, LV_ERR, "ldah_sr_rqb_ext: DRV send failed, seg=%d written %u from %u",seg_num, ret, rqbLen);
        result = PNIO_ERR_NO_FW_COMMUNICATION;
      } else {
        TRM_OUT02(&ti, GR_CHNL, LV_INFO, "ldah_sr_rqb_ext: DRV SEND OK DONE, seg=%d   len=%d", seg_num, ret);
      }

      /* wait for confirmation from firmware */
      sem_ret = DPR_SEM_WAIT_TIME(channel->sem_cnf_received, tm_ms );

      if(sem_ret == DPR_SEM_RET_TIMEOUT){
        TRM_OUT02(&ti, GR_CHNL, LV_ERR, "ldah_sr_rqb_ext: seg=%d resp.timeout after %lu msec",seg_num, tm_ms);
        result = PNIO_ERR_NO_FW_COMMUNICATION;
        goto sr_end;
      }

      /* response from cp arived
       */
      TRM_OUT00(&ti, GR_CHNL, LV_INFO, "ldah_sr_rqb_ext: DRV send confirm arrived!");

      pRcvBuff = (FWLDA_DPR_RQB *)channel->user_pool_ptr;

      if( (pRcvBuff->dpr_hdr.subsystem != FWLDA_SUBSYS_INDEX) || (LE_TO_CPU(pRcvBuff->dpr_hdr.userid) != 0xabcd) ) {      /* Fix: AP01394131 */
        TRM_OUT01(&ti, GR_CHNL, LV_ERR, "ldah_sr_rqb_ext: seg=%d resp. invalid hdr",seg_num);
        result = PNIO_ERR_INTERNAL;
        goto sr_end;
      }
      /* check if the rqb is a right confirmation */
      if(LE_TO_CPU(pRcvBuff->fwlda_rqb.opcode) != (ldaf_opc + 1)) {
        TRM_OUT03(&ti, GR_CHNL, LV_ERR, "ldah_sr_rqb_ext: seg=%d resp.wrong opcode %d (expected %d)", seg_num, pRcvBuff->fwlda_rqb.opcode, ldaf_opc + 1);
        result = PNIO_ERR_WRONG_RQB_LEN; /* ??? */
        goto sr_end;
      }

      /* check fwlda command response */
      if(LE_TO_CPU(pRcvBuff->fwlda_rqb.response) != SUBSYS_FWLDA_OK) {
        TRM_OUT02(&ti, GR_CHNL, LV_ERR, "ldah_sr_rqb_ext: seg_num=%d resp. error result=%d", seg_num, pRcvBuff->fwlda_rqb.response);
        result = PNIO_ERR_INTERNAL;
        goto sr_end;
      }

      /* dt ok sent */
      dt_total_sent += dt_len;

      if ( add_data_len - dt_total_sent == 0 ) {
          TRM_OUT03(&ti, GR_CHNL, LV_INFO, "ldah_sr_rqb_ext: send completed! seg=%d snd-done=%d snd-rqd=%d", seg_num, dt_total_sent, add_data_len);
          result = PNIO_OK;
          break;  /* send ok done  - exit loop */
      }
      /* prepare next segment for send: fill send Rqb */
      p_dt = p_dt + dt_len;
      if ( add_data_len - dt_total_sent <= dt_available_rqb_len ) {
          /* last segment */
          ldaf_opc = FWLDA_OP_DOWNLOAD_SDB_RQB_SEGM_END;
          dt_len = add_data_len - dt_total_sent;
      }
      else {
          /* next segment */
          ldaf_opc = FWLDA_OP_DOWNLOAD_SDB_RQB_SEGM;
          dt_len = dt_available_rqb_len;
      }
      rqbLen = sizeof(FWLDA_DPR_RQB) + dt_len;

      pRqb->dpr_hdr.hostref = CPU_TO_LE(channel->user_id);
      pRqb->dpr_hdr.subsystem = FWLDA_SUBSYS_INDEX;           /* Fix: AP01394131 */
      pRqb->dpr_hdr.userid = CPU_TO_LE(0xabcd);
      pRqb->dpr_hdr.userdatalength = CPU_TO_LE(rqbLen);

      pRqb->fwlda_rqb.opcode = (FWLDA_OPCODE)CPU_TO_LE(ldaf_opc);
      pRqb->fwlda_rqb.ovs_inst_id = CPU_TO_LE(vmdid);
      pRqb->fwlda_rqb.rqb_len = CPU_TO_LE(sizeof(pRqb->fwlda_rqb) + dt_len);

      memcpy((DPR_CHAR*)pRqb + sizeof(FWLDA_DPR_RQB), p_dt, dt_len);

  } while ( 1 );

  sr_end:
  if ( pRqb ) {
      DPR_VFREE(pRqb);
      pRqb = 0;
  }
  TRM_OUT03(&ti, GR_CHNL, LV_INFO, "<ldah_sr_rqb_ext: DONE! num-segs=%d rqd-len=%d sent-len=%d", seg_num, add_data_len, dt_total_sent);
  return result;
} /* end of ldah_sr_rqb_ext */


/*-----------------------------------------------------------------------------
 * Name  : ldah_sr_rqb_ext_sync
 * Descr : Send Receive Requestblock. Extended version with segmentation + sync.
 *   Note: This function is not used yet. It is not necessary, "Synchronous
 *         channel" is required only by XL-SDB upload (segmented extra large SDB)
 *         not by sdb download (writing to the cp16xx). during sdb download
 *         each sdb sgment is confirmed by the firmware, so no synchronous
 *         receive of confirmations is required.
 *
 * Return: ==0: PNIO_OK, != 0 PNIO_ERR_INTERNAL, ...
 */
DPR_UINT32 ldah_sr_rqb_ext_sync(CP_FWLDA *channel, DPR_UINT32 tm_ms, FWLDA_OPCODE ldaf_opcode,
                           DPR_UINT32 vmdid, DPR_UINT32 add_data_len, DPR_CHAR * add_data)
{
  size_t ret;
  DPR_UINT32 total_requested_len = sizeof(FWLDA_DPR_RQB) + add_data_len;
  DPR_UINT32 rqbLen;
  FWLDA_DPR_RQB *pRqb     = NULL;  /* RQB to be send */
  FWLDA_DPR_RQB *pRcvBuff = NULL;  /* ptr to receive buffer */
  FWLDA_OPCODE ldaf_opc;
  DPR_CHAR *p_dt;
  DPR_UINT32 dt_len;
  DPR_UINT32 dt_total_sent = 0;
  DPR_UINT32 send_buf_len;         /* max available send buffer len in the DPR channel */
  DPR_UINT32 dt_available_rqb_len; /* available num of data bytes in the request block */
  DPR_UINT32 result = PNIO_OK;
  int seg_num   = 0;
  int sem_ret;

  TRM_OUT02(&ti, GR_INIT, LV_INFO, ">ldah_sr_rqb_ext_sync: total_requested_len=%lu send_pool_length=%lu ", total_requested_len, channel->send_pool_length);

  if ( add_data_len == 0 || add_data == NULL ) {
      TRM_OUT00(&ti, GR_INIT, LV_WARN, "<ldah_sr_rqb_ext_sync: NO DATA!");
      return PNIO_OK;
  }

  if ( channel->send_pool_length <= 64  ) {
      send_buf_len = channel->send_pool_length;
  }
  else {
      send_buf_len = channel->send_pool_length - 64;
  }

  if(total_requested_len <= send_buf_len ) {
    /* normal single send */
    ldaf_opc = ldaf_opcode;   /* FWLDA_OP_DOWNLOAD_SDB_RQB  */
    p_dt   = add_data;
    dt_len = add_data_len;
  }
  else {
    /* segmented send */
    ldaf_opc = FWLDA_OP_DOWNLOAD_SDB_RQB_SEGM;
    p_dt   = add_data;
    dt_len = send_buf_len - sizeof(FWLDA_DPR_RQB);  /* netto data == sdb content */
  }

  rqbLen = sizeof(FWLDA_DPR_RQB) + dt_len;
  dt_available_rqb_len = rqbLen - sizeof(FWLDA_DPR_RQB);

  pRqb = (FWLDA_DPR_RQB*)DPR_VMALLOC(rqbLen);
  if(!pRqb) {
    TRM_OUT00(&ti, GR_INIT, LV_ERR, "<ldah_sr_rqb_ext_sync: RQB alloc memory failed");
    return PNIO_ERR_NO_RESOURCE;
  }

  pRqb->dpr_hdr.hostref = CPU_TO_LE(channel->user_id);
  pRqb->dpr_hdr.subsystem = FWLDA_SUBSYS_INDEX;          /* Fix: AP01394131 */
  pRqb->dpr_hdr.userid = CPU_TO_LE(0xabcd);
  pRqb->dpr_hdr.userdatalength = CPU_TO_LE(rqbLen);

  pRqb->fwlda_rqb.opcode = (FWLDA_OPCODE)CPU_TO_LE(ldaf_opc);
  pRqb->fwlda_rqb.ovs_inst_id = CPU_TO_LE(vmdid);
  pRqb->fwlda_rqb.rqb_len = CPU_TO_LE(sizeof(pRqb->fwlda_rqb) + dt_len);

  if(dt_len) {
    memcpy((DPR_CHAR*)pRqb + sizeof(FWLDA_DPR_RQB), p_dt, dt_len);
  }

  do {
      seg_num += 1;

      TRM_OUT02(&ti, GR_CHNL, LV_INFO, "ldah_sr_rqb_ext_sync: DRV send...  seg=%d len=%d ",seg_num, rqbLen);
      ret = DPR_DRV_WRITE(channel->mgt_chnl, pRqb, rqbLen);

      if(ret != rqbLen) {
        TRM_OUT03(&ti, GR_CHNL, LV_ERR, "ldah_sr_rqb_ext_sync: DRV send failed, seg=%d written %u from %u",seg_num, ret, rqbLen);
        result = PNIO_ERR_NO_FW_COMMUNICATION;
      } else {
        TRM_OUT02(&ti, GR_CHNL, LV_INFO, "ldah_sr_rqb_ext_sync: DRV SEND OK DONE, seg=%d   len=%d", seg_num, ret);
      }

      /* receive confirmation from the firmware (from the driver) */
      TRM_OUT01(&ti, GR_CHNL, LV_INFO, "ldah_sr_rqb_ext_sync: RECV RESP. req-opcode=%d",ldaf_opcode);

      /* receive confirmation msg synchronously */
      unsigned long received_len = 0;
      result = ldah_mgt_ch_read(channel, &received_len);
      if ( result != PNIO_OK ) {
          TRM_OUT00(&ti, GR_CHNL, LV_ERR, "ldah_sr_rqb_ext_sync: ch_read ERROR");
          return result;
      }
      if ( received_len == 0 ) {
          TRM_OUT00(&ti, GR_CHNL, LV_ERR, "ldah_sr_rqb_ext_sync: ch_read ERR LEN=0");
          return PNIO_ERR_INTERNAL;
      }

      /* response from cp arived
       */
      TRM_OUT00(&ti, GR_CHNL, LV_INFO, "ldah_sr_rqb_ext_sync: DRV send confirm arrived!");

      pRcvBuff = (FWLDA_DPR_RQB *)channel->user_pool_ptr;

      if( (pRcvBuff->dpr_hdr.subsystem != FWLDA_SUBSYS_INDEX) || (LE_TO_CPU(pRcvBuff->dpr_hdr.userid) != 0xabcd) ) {      /* Fix: AP01394131 */
        TRM_OUT01(&ti, GR_CHNL, LV_ERR, "ldah_sr_rqb_ext_sync: seg=%d resp. invalid hdr",seg_num);
        result = PNIO_ERR_INTERNAL;
        goto sr_end;
      }
      /* check if the rqb is a right confirmation */
      if(LE_TO_CPU(pRcvBuff->fwlda_rqb.opcode) != (ldaf_opc + 1)) {
        TRM_OUT03(&ti, GR_CHNL, LV_ERR, "ldah_sr_rqb_ext_sync: seg=%d resp.wrong opcode %d (expected %d)", seg_num, pRcvBuff->fwlda_rqb.opcode, ldaf_opc + 1);
        result = PNIO_ERR_WRONG_RQB_LEN; /* ??? */
        goto sr_end;
      }

      /* check fwlda command response */
      if(LE_TO_CPU(pRcvBuff->fwlda_rqb.response) != SUBSYS_FWLDA_OK) {
        TRM_OUT02(&ti, GR_CHNL, LV_ERR, "ldah_sr_rqb_ext_sync: seg_num=%d resp. error result=%d", seg_num, pRcvBuff->fwlda_rqb.response);
        result = PNIO_ERR_INTERNAL;
        goto sr_end;
      }

      /* dt ok sent */
      dt_total_sent += dt_len;

      if ( add_data_len - dt_total_sent == 0 ) {
          TRM_OUT03(&ti, GR_CHNL, LV_INFO, "ldah_sr_rqb_ext_sync: send completed! seg=%d snd-done=%d snd-rqd=%d", seg_num, dt_total_sent, add_data_len);
          result = PNIO_OK;
          break;  /* send ok done  <<<--- exit loop */
      }
      /* prepare next segment for send: fill send Rqb */
      p_dt = p_dt + dt_len;
      if ( add_data_len - dt_total_sent <= dt_available_rqb_len ) {
          /* last segment */
          ldaf_opc = FWLDA_OP_DOWNLOAD_SDB_RQB_SEGM_END;
          dt_len = add_data_len - dt_total_sent;
      }
      else {
          /* next segment */
          ldaf_opc = FWLDA_OP_DOWNLOAD_SDB_RQB_SEGM;
          dt_len = dt_available_rqb_len;
      }
      rqbLen = sizeof(FWLDA_DPR_RQB) + dt_len;

      pRqb->dpr_hdr.hostref = CPU_TO_LE(channel->user_id);
      pRqb->dpr_hdr.subsystem = FWLDA_SUBSYS_INDEX;        /* Fix: AP01394131 */
      pRqb->dpr_hdr.userid = CPU_TO_LE(0xabcd);
      pRqb->dpr_hdr.userdatalength = CPU_TO_LE(rqbLen);

      pRqb->fwlda_rqb.opcode = (FWLDA_OPCODE)CPU_TO_LE(ldaf_opc);
      pRqb->fwlda_rqb.ovs_inst_id = CPU_TO_LE(vmdid);
      pRqb->fwlda_rqb.rqb_len = CPU_TO_LE(sizeof(pRqb->fwlda_rqb) + dt_len);

      memcpy((DPR_CHAR*)pRqb + sizeof(FWLDA_DPR_RQB), p_dt, dt_len);

  } while ( 1 );

  sr_end:
  if ( pRqb ) {
      DPR_VFREE(pRqb);
      pRqb = 0;
  }
  TRM_OUT03(&ti, GR_CHNL, LV_INFO, "<ldah_sr_rqb_ext_sync: DONE! num-segs=%d rqd-len=%d sent-len=%d", seg_num, add_data_len, dt_total_sent);
  return result;
} /* end of ldah_sr_rqb_ext_sync */


/*-----------------------------------------------------------------------------
 * Name  : ldah_rcv_xl_cnf
 * Descr : Local download agent host: Receive "XL-SDB" Confirmation
 *         I.e.: Receive one segmented XL-SDB
 * Param :
 *  [ IN]: channel, opcode
 *  [OUT]: pRcvdLen, ppRcvdData
 * Return: ==0: PNIO_OK, != 0 : PNIO_ERR_xxx
 */
DPR_UINT32 ldah_rcv_xl_cnf (CP_FWLDA *channel, FWLDA_OPCODE opcode, unsigned int* pRcvdLen, DPR_CHAR** ppRcvdData)
{
    DPR_UINT32 res = PNIO_OK;
    FWLDA_DPR_RQB *pRqb = NULL;
    unsigned int total_expected_cnf_data_length = 0;
    unsigned int total_received_cnf_data_length = 0;
    unsigned int current_rcvd_cnf_data_length   = 0;     // confirmation data length== SDB content
    DPR_CHAR *p_seg_dt  = NULL;                          // ptr to current received neto data == SDB ptr
    DPR_CHAR *p_rcvbuff = NULL;
    int num_segs = 1;
    int rcvd_seg_num = 0;
    int rcvd_seg_num_previous = 0;
    unsigned int fwlda_dpr_rb_length = 0;
    unsigned int seg_dt_len;
    unsigned int rqb_dt_len;
    DPR_UINT32 rcv_timeout_ms = 3000;

    TRM_OUT00(&ti, GR_CHNL, LV_INFO, "->ldah_rcv_xl_cnf: ");

    if (ppRcvdData == NULL) {
        TRM_OUT00(&ti, GR_CHNL, LV_ERR, "ldah_rcv_xl_cnf: ERR PAR4");
        *ppRcvdData = NULL;
        res= PNIO_ERR_INTERNAL;
        goto rcv_xl_cnf_err;
    }

    *pRcvdLen = 0;  // pre-set output param to zero for error exits

    // 1st segment is already received in the pool
    pRqb = (FWLDA_DPR_RQB *)channel->user_pool_ptr;

    // retrieve the received data length == (sizeof(FWLDA_DPR_RQB) + SDB-Segment len ) == pRqb->dpr_hdr.userdatalength);
    fwlda_dpr_rb_length   = LE_TO_CPU(pRqb->dpr_hdr.userdatalength);
    if ( fwlda_dpr_rb_length <= sizeof(FWLDA_DPR_RQB) ) {
        return PNIO_ERR_WRONG_RQB_LEN;
    }
    seg_dt_len = fwlda_dpr_rb_length - sizeof(FWLDA_DPR_RQB);       // current SDB len. whole-SDB or SDB-Segment len

    rqb_dt_len = LE_TO_CPU(pRqb->fwlda_rqb.rqb_len);                // == sizeof(FWLDA_RQB) + SDBLen;
    if ( rqb_dt_len <= sizeof(FWLDA_RQB) ) {
        TRM_OUT02(&ti, GR_CHNL, LV_ERR, "ldah_rcv_xl_cnf: ERR rqb=%u sizeof(FWLDA_RQB)=%u ",rqb_dt_len, sizeof(FWLDA_RQB) );
        res= PNIO_ERR_WRONG_RQB_LEN;
        goto rcv_xl_cnf_err;
    }
    rqb_dt_len = rqb_dt_len - sizeof(FWLDA_RQB);                                 // total SDB len (without any hdr)

    total_expected_cnf_data_length = rqb_dt_len + sizeof(FWLDA_DPR_RQB); // total confirmation data len == DPR-Hdr + FWLDA-Hdr + SDB len
    p_rcvbuff = (DPR_CHAR*)DPR_VMALLOC(total_expected_cnf_data_length);
    if (!p_rcvbuff) {
        TRM_OUT00(&ti, GR_CHNL, LV_ERR, "ldah_rcv_xl_cnf: ERR MEM");
        *ppRcvdData = NULL;
        res= PNIO_ERR_NO_RESOURCE;
        goto rcv_xl_cnf_err;
    }

	// segment number - sequence check
	rcvd_seg_num = pRqb->dpr_hdr.reserved1[2];
    rcvd_seg_num_previous = rcvd_seg_num;       // save current (1st) segement number
    TRM_OUT01(&ti, GR_CHNL, LV_INFO, "ldah_rcv_xl_cnf: rcvd_seg_num=%d ",rcvd_seg_num);

    if (total_expected_cnf_data_length < seg_dt_len) {
        // programm error - wrong rqb params from the firmware
        TRM_OUT02(&ti, GR_CHNL, LV_ERR, "ldah_rcv_xl_cnf: ERR te=%lu seg=%u ",total_expected_cnf_data_length, seg_dt_len);
        res= PNIO_ERR_INTERNAL;
        goto rcv_xl_cnf_err;
    }

    // copy data from the DPRAM channel to the receive buffer
    //
    p_seg_dt = ((DPR_CHAR*)pRqb);  // 1st segment includes  FWLDA_DPR_RQB headers
    current_rcvd_cnf_data_length = seg_dt_len + sizeof(FWLDA_DPR_RQB);
    memcpy(p_rcvbuff, p_seg_dt, current_rcvd_cnf_data_length);
    if ( total_expected_cnf_data_length == current_rcvd_cnf_data_length ) {
        // we have already received all data in 1st segment -> non segmented short sdb
        TRM_OUT01(&ti, GR_CHNL, LV_INFO, "<-ldah_rcv_xl_cnf: OK: 1Seg len=%lu ", current_rcvd_cnf_data_length);
        *ppRcvdData = p_rcvbuff;
        *pRcvdLen   = current_rcvd_cnf_data_length;
        return PNIO_OK;
    }

    // here segmented data receive handling
    rcv_timeout_ms = 6000;
#ifdef _DEBUG
	rcv_timeout_ms = 60000; /* set long timeout time if under dubugger */
#endif
    num_segs = 1;      // 1st segment already received and handled

    total_received_cnf_data_length = current_rcvd_cnf_data_length;

    do {
		TRM_OUT03(&ti, GR_CHNL, LV_INFO, "ldah_rcv_xl_cnf: WAIT FOR NEXT SEG: rcvd-segs=%d Bytes expected=%u rcvd=%u",
			      num_segs, total_expected_cnf_data_length, total_received_cnf_data_length );

        /* wait for data from firmware - via receiver thread */
        res = DPR_SEM_WAIT_TIME(channel->sem_cnf_received, rcv_timeout_ms);

        if(res == DPR_SEM_RET_TIMEOUT){
          TRM_OUT02(&ti, GR_CHNL, LV_ERR, "ldah_rcv_xl_cnf: ERR seg=%d Timeout=%lu [msec]",num_segs+1,  rcv_timeout_ms);
          res= PNIO_ERR_NO_FW_COMMUNICATION;
          goto rcv_xl_cnf_err;
        }

        // dt segment is already received in the DPRAM channel receive pool
        num_segs++;
        pRqb = (FWLDA_DPR_RQB *)channel->user_pool_ptr;

        // received data length
        fwlda_dpr_rb_length   = LE_TO_CPU(pRqb->dpr_hdr.userdatalength); // == (sizeof(FWLDA_DPR_RQB) + SDB-Segment len )
        if ( fwlda_dpr_rb_length <= sizeof(FWLDA_DPR_RQB) ) {
            TRM_OUT03(&ti, GR_CHNL, LV_ERR, "ldah_rcv_xl_cnf: ERR seg=%d dpr-rb-len=%lu hdr=%lu",num_segs, fwlda_dpr_rb_length, sizeof(FWLDA_DPR_RQB) );
            res = PNIO_ERR_WRONG_RQB_LEN;
            goto rcv_xl_cnf_err;
        }

        // for subsequent segments we remove the header
        seg_dt_len = fwlda_dpr_rb_length - sizeof(FWLDA_DPR_RQB);       // current SDB len. whole-SDB or SDB-Segment len

        if ( (total_received_cnf_data_length + seg_dt_len) > total_expected_cnf_data_length ) {
            // programm error - wrong rqb params from the firmware
            TRM_OUT03(&ti, GR_CHNL, LV_ERR, "ldah_rcv_xl_cnf: ERR seg=%d tr=%lu te=%lu",num_segs,
                      (total_received_cnf_data_length + seg_dt_len),  total_expected_cnf_data_length );
            res = PNIO_ERR_INTERNAL;
            goto rcv_xl_cnf_err;
        }

		// check segment number - return error if segment lost
		rcvd_seg_num = pRqb->dpr_hdr.reserved1[2];
        if ( (rcvd_seg_num_previous + 1) != rcvd_seg_num ) {
            // error, segment lost
            TRM_OUT03(&ti, GR_CHNL, LV_ERR, "ldah_rcv_xl_cnf: ERROR SEG-NUM rcvd_seg_num_previous=%d rcvd_seg_num=%d loop-seg=%d ",
                      rcvd_seg_num_previous, rcvd_seg_num, num_segs);
            res = PNIO_ERR_INTERNAL;
            goto rcv_xl_cnf_err;
        }
        else {
            // ok
            TRM_OUT04(&ti, GR_CHNL, LV_INFO, "ldah_rcv_xl_cnf: rcvd_seg_num=%d loop-seg=%d rcvd-curr=%d rcvd-total=%u",
                      rcvd_seg_num, num_segs, seg_dt_len, (total_received_cnf_data_length + seg_dt_len) );
        }
        rcvd_seg_num_previous = rcvd_seg_num;       // save current segement number

        // copy data from the DPRAM channel to the receive buffer
        //
        p_seg_dt = ((DPR_CHAR*)pRqb) + sizeof(FWLDA_DPR_RQB);
        current_rcvd_cnf_data_length = seg_dt_len;
        memcpy( ((DPR_CHAR*)p_rcvbuff) + total_received_cnf_data_length, p_seg_dt, current_rcvd_cnf_data_length);

        total_received_cnf_data_length += current_rcvd_cnf_data_length;

        if ( total_expected_cnf_data_length < total_received_cnf_data_length ) {
            // err to much data
            res = PNIO_ERR_INTERNAL;
            goto rcv_xl_cnf_err;
        }

        // segment number (counter) was already incremented above in the loop!!!  ( see above: num_segs++; )

    } while ( total_expected_cnf_data_length > total_received_cnf_data_length );

    // ok all data received (the whole SDB)
    *ppRcvdData = p_rcvbuff;
    *pRcvdLen   = total_received_cnf_data_length;
    TRM_OUT02(&ti, GR_CHNL, LV_INFO, "<-ldah_rcv_xl_cnf: OK: NumSeg=%d total-len=%lu ",num_segs,total_received_cnf_data_length);
    return PNIO_OK;

rcv_xl_cnf_err:
    if (p_rcvbuff) {
        DPR_VFREE(p_rcvbuff);
    }
    *ppRcvdData = NULL;
    *pRcvdLen   = 0;
    TRM_OUT02(&ti, GR_CHNL, LV_ERR, "<-ldah_rcv_xl_cnf: ERR: NumSeg=%d result=%#x ",num_segs, res);
    return res;

} /* end of ldah_rcv_xl_cnf */


/*-----------------------------------------------------------------------------
 * Name  : ldah_rcv_xl_cnf_sync
 * Descr : Local download agent host: Receive "XL-SDB" Confirmation synchronously
 *         I.e.: Receive one segmented XL-SDB
 * Param :
 *  [ IN]: channel, opcode
 *  [OUT]: pRcvdLen, ppRcvdData
 * Return: ==0: PNIO_OK, != 0 : PNIO_ERR_xxx
 */
DPR_UINT32 ldah_rcv_xl_cnf_sync (CP_FWLDA *channel, FWLDA_OPCODE opcode, unsigned int* pRcvdLen, DPR_CHAR** ppRcvdData)
{
    DPR_UINT32 res = PNIO_OK;
    FWLDA_DPR_RQB *pRqb = NULL;
    unsigned int total_expected_cnf_data_length = 0;
    unsigned int total_received_cnf_data_length = 0;
    unsigned int current_rcvd_cnf_data_length   = 0;     // confirmation data length== SDB content
    DPR_CHAR *p_seg_dt  = NULL;                          // ptr to current received neto data == SDB ptr
    DPR_CHAR *p_rcvbuff = NULL;
    int num_segs = 1;
    int rcvd_seg_num = 0;
    int rcvd_seg_num_previous = 0;
    unsigned int fwlda_dpr_rb_length = 0;
    unsigned int seg_dt_len;
    unsigned int rqb_dt_len;

    TRM_OUT00(&ti, GR_CHNL, LV_INFO, "->ldah_rcv_xl_cnf_sync: ");

    if (ppRcvdData == NULL) {
        TRM_OUT00(&ti, GR_CHNL, LV_ERR, "ldah_rcv_xl_cnf_sync: ERR PAR4");
        *ppRcvdData = NULL;
        res= PNIO_ERR_INTERNAL;
        goto rcv_xl_cnf_sync_err;
    }

    *pRcvdLen = 0;  // pre-set output param to zero for error exits

    // 1st segment is already received in the pool
    pRqb = (FWLDA_DPR_RQB *)channel->user_pool_ptr;

    // retrieve the received data length == (sizeof(FWLDA_DPR_RQB) + SDB-Segment len ) == pRqb->dpr_hdr.userdatalength);
    fwlda_dpr_rb_length   = LE_TO_CPU(pRqb->dpr_hdr.userdatalength);
    if ( fwlda_dpr_rb_length <= sizeof(FWLDA_DPR_RQB) ) {
        return PNIO_ERR_WRONG_RQB_LEN;
    }
    seg_dt_len = fwlda_dpr_rb_length - sizeof(FWLDA_DPR_RQB);       // current SDB len. whole-SDB or SDB-Segment len

    rqb_dt_len = LE_TO_CPU(pRqb->fwlda_rqb.rqb_len);                // == sizeof(FWLDA_RQB) + SDBLen;
    if ( rqb_dt_len <= sizeof(FWLDA_RQB) ) {
        TRM_OUT02(&ti, GR_CHNL, LV_ERR, "ldah_rcv_xl_cnf_sync: ERR rqb=%u sizeof(FWLDA_RQB)=%u ",rqb_dt_len, sizeof(FWLDA_RQB) );
        res= PNIO_ERR_WRONG_RQB_LEN;
        goto rcv_xl_cnf_sync_err;
    }
    rqb_dt_len = rqb_dt_len - sizeof(FWLDA_RQB);                                 // total SDB len (without any hdr)

    total_expected_cnf_data_length = rqb_dt_len + sizeof(FWLDA_DPR_RQB); // total confirmation data len == DPR-Hdr + FWLDA-Hdr + SDB len
    p_rcvbuff = (DPR_CHAR*)DPR_VMALLOC(total_expected_cnf_data_length);
    if (!p_rcvbuff) {
        TRM_OUT00(&ti, GR_CHNL, LV_ERR, "ldah_rcv_xl_cnf_sync: ERR MEM");
        *ppRcvdData = NULL;
        res= PNIO_ERR_NO_RESOURCE;
        goto rcv_xl_cnf_sync_err;
    }

	// segment number - sequence check
	rcvd_seg_num = pRqb->dpr_hdr.reserved1[2];
    rcvd_seg_num_previous = rcvd_seg_num;       // save current (1st) segement number
    TRM_OUT01(&ti, GR_CHNL, LV_INFO, "ldah_rcv_xl_cnf_sync: rcvd_seg_num=%d ",rcvd_seg_num);

    if (total_expected_cnf_data_length < seg_dt_len) {
        // programm error - wrong rqb params from the firmware
        TRM_OUT02(&ti, GR_CHNL, LV_ERR, "ldah_rcv_xl_cnf_sync: ERR te=%lu seg=%u ",total_expected_cnf_data_length, seg_dt_len);
        res= PNIO_ERR_INTERNAL;
        goto rcv_xl_cnf_sync_err;
    }

    // copy data from the DPRAM channel to the receive buffer
    //
    p_seg_dt = ((DPR_CHAR*)pRqb);  // 1st segment includes  FWLDA_DPR_RQB headers
    current_rcvd_cnf_data_length = seg_dt_len + sizeof(FWLDA_DPR_RQB);
    memcpy(p_rcvbuff, p_seg_dt, current_rcvd_cnf_data_length);
    if ( total_expected_cnf_data_length == current_rcvd_cnf_data_length ) {
        // we have already received all data in 1st segment -> non segmented short sdb
        TRM_OUT01(&ti, GR_CHNL, LV_INFO, "<-ldah_rcv_xl_cnf_sync: OK: 1Seg len=%lu ", current_rcvd_cnf_data_length);
        *ppRcvdData = p_rcvbuff;
        *pRcvdLen   = current_rcvd_cnf_data_length;
        return PNIO_OK;
    }

    // here segmented data receive handling
    num_segs = 1;      // 1st segment already received and handled

    total_received_cnf_data_length = current_rcvd_cnf_data_length;

    do {
		TRM_OUT03(&ti, GR_CHNL, LV_INFO, "ldah_rcv_xl_cnf_sync: WAIT FOR NEXT SEG: rcvd-segs=%d Bytes expected=%u rcvd=%u",
			      num_segs, total_expected_cnf_data_length, total_received_cnf_data_length );

        DPR_UINT32 result;
        unsigned long received_len = 0;
        result = ldah_mgt_ch_read(channel, &received_len);
        if ( result != PNIO_OK ) {
            TRM_OUT00(&ti, GR_CHNL, LV_ERR, "ch_read ERROR");
            res = result;
            goto rcv_xl_cnf_sync_err;
        }
        if ( received_len == 0 ) {
            TRM_OUT00(&ti, GR_CHNL, LV_ERR, "ch_read ERR LEN=0");
            res = PNIO_ERR_INTERNAL;  // ???? retry ???
            goto rcv_xl_cnf_sync_err;
        }

        // dt segment is already received in the DPRAM channel receive pool
        num_segs++;
        pRqb = (FWLDA_DPR_RQB *)channel->user_pool_ptr;

        // received data length
        fwlda_dpr_rb_length   = LE_TO_CPU(pRqb->dpr_hdr.userdatalength); // == (sizeof(FWLDA_DPR_RQB) + SDB-Segment len )
        if ( fwlda_dpr_rb_length <= sizeof(FWLDA_DPR_RQB) ) {
            TRM_OUT03(&ti, GR_CHNL, LV_ERR, "ldah_rcv_xl_cnf_sync: ERR seg=%d dpr-rb-len=%lu hdr=%lu",num_segs, fwlda_dpr_rb_length, sizeof(FWLDA_DPR_RQB) );
            res = PNIO_ERR_WRONG_RQB_LEN;
            goto rcv_xl_cnf_sync_err;
        }

        // for subsequent segments we remove the header
        seg_dt_len = fwlda_dpr_rb_length - sizeof(FWLDA_DPR_RQB);       // current SDB len. whole-SDB or SDB-Segment len

        if ( (total_received_cnf_data_length + seg_dt_len) > total_expected_cnf_data_length ) {
            // programm error - wrong rqb params from the firmware
            TRM_OUT03(&ti, GR_CHNL, LV_ERR, "ldah_rcv_xl_cnf_sync: ERR seg=%d tr=%lu te=%lu",num_segs,
                      (total_received_cnf_data_length + seg_dt_len),  total_expected_cnf_data_length );
            res = PNIO_ERR_INTERNAL;
            goto rcv_xl_cnf_sync_err;
        }

		// check segment number - return error if segment lost
		rcvd_seg_num = pRqb->dpr_hdr.reserved1[2];
        if ( (rcvd_seg_num_previous + 1) != rcvd_seg_num ) {
            // error, segment lost
            TRM_OUT03(&ti, GR_CHNL, LV_ERR, "ldah_rcv_xl_cnf_sync: ERROR SEG-NUM rcvd_seg_num_previous=%d rcvd_seg_num=%d loop-seg=%d ",
                      rcvd_seg_num_previous, rcvd_seg_num, num_segs);
            res = PNIO_ERR_INTERNAL;
            goto rcv_xl_cnf_sync_err;
        }
        else {
            // ok
            TRM_OUT04(&ti, GR_CHNL, LV_INFO, "ldah_rcv_xl_cnf_sync: rcvd_seg_num=%d loop-seg=%d rcvd-curr=%d rcvd-total=%u",
                      rcvd_seg_num, num_segs, seg_dt_len, (total_received_cnf_data_length + seg_dt_len) );
        }
        rcvd_seg_num_previous = rcvd_seg_num;       // save current segement number

        // copy data from the DPRAM channel to the receive buffer
        //
        p_seg_dt = ((DPR_CHAR*)pRqb) + sizeof(FWLDA_DPR_RQB);
        current_rcvd_cnf_data_length = seg_dt_len;
        memcpy( ((DPR_CHAR*)p_rcvbuff) + total_received_cnf_data_length, p_seg_dt, current_rcvd_cnf_data_length);

        total_received_cnf_data_length += current_rcvd_cnf_data_length;

        if ( total_expected_cnf_data_length < total_received_cnf_data_length ) {
            // err to much data
            res = PNIO_ERR_INTERNAL;
            goto rcv_xl_cnf_sync_err;
        }

        // segment number (counter) was already incremented above in the loop!!!  ( see above: num_segs++; )

    } while ( total_expected_cnf_data_length > total_received_cnf_data_length );

    // ok all data received (the whole SDB)
    *ppRcvdData = p_rcvbuff;
    *pRcvdLen   = total_received_cnf_data_length;
    TRM_OUT02(&ti, GR_CHNL, LV_INFO, "<-ldah_rcv_xl_cnf_sync: OK: NumSeg=%d total-len=%lu ",num_segs,total_received_cnf_data_length);
    return PNIO_OK;

rcv_xl_cnf_sync_err:
    if (p_rcvbuff) {
        DPR_VFREE(p_rcvbuff);
    }
    *ppRcvdData = NULL;
    *pRcvdLen   = 0;
    TRM_OUT02(&ti, GR_CHNL, LV_ERR, "<-ldah_rcv_xl_cnf_sync: ERR: NumSeg=%d result=%#x ",num_segs, res);
    return res;

} /* end of ldah_rcv_xl_cnf_sync */


/*-----------------------------------------------------------------------------
 * Name  : ldah_send_receive_rqb
 * Descr : Synchronously sends data and receives confirmation from the firmware.
 * Param :
 *  [ IN]:
 *  [OUT]: pprd: Pointer to a pointer to received data buffer
 *               This receive buffer is allocated here in case of XL SDB Upload.
 *               All data are copied into the buffer. Buffer contains header + data.
 *               E.g. SDB-Upload:  FWLDA_DPR_RQB + SDBData
 *
 * Return: ==0: ok, != 0 error
 */
DPR_UINT32 ldah_send_receive_rqb(CP_FWLDA *channel, DPR_UINT32 tm_ms, FWLDA_OPCODE ldaf_opcode,
                                 DPR_UINT32 vmdid, DPR_UINT32 add_data_len, DPR_CHAR* add_data,
                                 DPR_CHAR** pprd = NULL)
{
  size_t ret;
  DPR_UINT32 rqbLen = sizeof(FWLDA_DPR_RQB) + add_data_len;
  FWLDA_DPR_RQB *pRqb = 0;
  int sem_ret;

  TRM_OUT01(&ti, GR_CHNL, LV_INFO, "->ldah_send_receive_rqb: opcode=%d", ldaf_opcode);

  /* SEND REQ */

  /* check if x-large sdb download */
  if(rqbLen > channel->send_pool_length  &&  ldaf_opcode == FWLDA_OP_DOWNLOAD_SDB_RQB ) {
    /* rqb is too big - segmented send required */
    DPR_UINT32 result;

    TRM_OUT03(&ti, GR_CHNL, LV_WARN, "ldah_send_receive_rqb, data_len=%u, rqbLen(%lu) > channel->send_pool_length (%lu)",
                                      add_data_len, rqbLen, channel->send_pool_length);

    result= ldah_sr_rqb_ext(channel, tm_ms, ldaf_opcode, vmdid, add_data_len, add_data);   /* <<<==== SEGMENTED SEND <<<<<<<<<<<========= */

    TRM_OUT02(&ti, GR_CHNL, LV_WARN, "<-- ldah_send_receive_rqb, data_len=%u DONE! result=%#x ",add_data_len, result);
    return result;  // <<< EXIT <<<
  }

 /* handle other FWLDA_OP_xxx requests other than FWLDA_OP_DOWNLOAD_SDB_RQB
  * allocate and prepare fwlda request block
  */
  pRqb = (FWLDA_DPR_RQB*)DPR_VMALLOC(rqbLen);
  if(!pRqb) {
    TRM_OUT00(&ti, GR_INIT, LV_ERR, "ldah_send_receive_rqb, alloc memory for FWLDA_DPR_RQB failed");
    return PNIO_ERR_NO_RESOURCE;
  }

  /* check if SDB upload command */
  if ( ldaf_opcode == FWLDA_OP_GET_SDB_RQB ) {
      // set new opcode to signal that we can receive x-large sdbs
      // without this, the firmware can't decide if send of XL sdbs is ok
      ldaf_opcode = FWLDA_OP_GET_SDB_XL_RQB;
  }
  pRqb->dpr_hdr.hostref = CPU_TO_LE(channel->user_id);
  pRqb->dpr_hdr.subsystem = FWLDA_SUBSYS_INDEX;
  pRqb->dpr_hdr.userid = CPU_TO_LE(0xabcd);
  pRqb->dpr_hdr.userdatalength = CPU_TO_LE(rqbLen);

  pRqb->fwlda_rqb.opcode = (FWLDA_OPCODE)CPU_TO_LE(ldaf_opcode);
  pRqb->fwlda_rqb.ovs_inst_id = CPU_TO_LE(vmdid);
  pRqb->fwlda_rqb.rqb_len = CPU_TO_LE(sizeof(pRqb->fwlda_rqb) + add_data_len);

  if(add_data_len) {
    memcpy((DPR_CHAR*)pRqb + sizeof(FWLDA_DPR_RQB), add_data, add_data_len);
  }

  ret = DPR_DRV_WRITE(channel->mgt_chnl, pRqb, rqbLen);

  DPR_VFREE(pRqb);
  pRqb = 0;

  if(ret != rqbLen) {
    TRM_OUT03(&ti, GR_CHNL, LV_ERR,
              "Send failed, written %u bytes from %d, error '%s'",
              ret, rqbLen, DPR_STRERROR());
    return PNIO_ERR_NO_FW_COMMUNICATION;
  } else {
    TRM_OUT02(&ti, GR_CHNL, LV_INFO, "ldah_send_receive_rqb, sent rqb to fw opcode=%d, len=%d",
              ldaf_opcode, rqbLen);
  }

  /* RECEIVE CONFIRMATION */

  /* wait for confirmation from the firmware (from the driver) timeout: see caller ldah_s7prj_write_backup()  */
  TRM_OUT01(&ti, GR_CHNL, LV_INFO, "ldah_send_receive_rqb: RECV RESP. req-opcode=%d",ldaf_opcode);

  sem_ret = DPR_SEM_WAIT_TIME(channel->sem_cnf_received, tm_ms );

  if(sem_ret == DPR_SEM_RET_TIMEOUT){
    TRM_OUT01(&ti, GR_CHNL, LV_ERR, "ldah_send_receive_rqb, timeout after %lu msec", tm_ms);
    return PNIO_ERR_NO_FW_COMMUNICATION;
  }

  pRqb = (FWLDA_DPR_RQB *)channel->user_pool_ptr;

  if( (pRqb->dpr_hdr.subsystem != FWLDA_SUBSYS_INDEX) || (LE_TO_CPU(pRqb->dpr_hdr.userid) != 0xabcd) ) {
    TRM_OUT00(&ti, GR_CHNL, LV_ERR, "ldah_send_receive_rqb, wrong dpr channel identification");
    return PNIO_ERR_INTERNAL;
  }

  // Changes to receive x-large WEB SDBs segmented
  // check for new xl-sdb upload confirmation - receive all data sgments
  if(LE_TO_CPU(pRqb->fwlda_rqb.opcode) == FWLDA_OP_GET_SDB_XL_SEG_CNF ) {
      DPR_UINT32 result = PNIO_OK;
      unsigned int rdlen  = 0;
      TRM_OUT02(&ti, GR_CHNL, LV_INFO, "ldah_send_receive_rqb: XL-SDB-CNF= %d pool-len=%u", pRqb->fwlda_rqb.opcode, channel->user_pool_length);
      result = ldah_rcv_xl_cnf (channel, pRqb->fwlda_rqb.opcode, &rdlen, pprd);
      return result;
  }

  /* other confirmation */
  /* check is the rqb a confirmation */
  if(LE_TO_CPU(pRqb->fwlda_rqb.opcode) != (ldaf_opcode + 1)) {
    TRM_OUT02(&ti, GR_CHNL, LV_ERR, "ldah_send_receive_rqb, wrong opcode %d (expected %d)", pRqb->fwlda_rqb.opcode, ldaf_opcode + 1);
    return PNIO_ERR_WRONG_RQB_LEN;
  }

  if(LE_TO_CPU(pRqb->fwlda_rqb.response) != SUBSYS_FWLDA_OK) {
    TRM_OUT01(&ti, GR_CHNL, LV_ERR, "ldah_send_receive_rqb, confirmation error response %d", pRqb->fwlda_rqb.response);
    return PNIO_ERR_INTERNAL;
  }

  return PNIO_OK;
} /* end of ldah_send_receive_rqb */

/*-----------------------------------------------------------------------------
 * Name  : ldah_send_receive_rqb_sync
 * Descr : Synchronously receives confirmation from the firmware.
 * Param :
 *  [ IN]: channel:
 *         tm_ms: timeout not use
 *  [OUT]: pprd: Pointer to a pointer to received data buffer
 *               This receive buffer is allocated here in case of XL SDB Upload.
 *               All data are copied into the buffer. Buffer contains header + data.
 *               E.g. SDB-Upload:  FWLDA_DPR_RQB + SDBData
 *
 * Return: ==0: ok, != 0 error
 */
DPR_UINT32 ldah_send_receive_rqb_sync(CP_FWLDA *channel, DPR_UINT32 tm_ms, FWLDA_OPCODE ldaf_opcode,
                                 DPR_UINT32 vmdid, DPR_UINT32 add_data_len, DPR_CHAR* add_data,
                                 DPR_CHAR** pprd = NULL)
{
  size_t ret;
  DPR_UINT32 rqbLen = sizeof(FWLDA_DPR_RQB) + add_data_len;
  FWLDA_DPR_RQB *pRqb = 0;
  int sem_ret;
  DPR_UINT32 result;

  TRM_OUT01(&ti, GR_CHNL, LV_INFO, "->ldah_send_receive_rqb_sync: opcode=%d", ldaf_opcode);

  /* SEND REQ */
  /* check if x-large sdb download */
  if(rqbLen > channel->send_pool_length  &&  ldaf_opcode == FWLDA_OP_DOWNLOAD_SDB_RQB ) {
    /* rqb is too big - segmented send required */

    TRM_OUT03(&ti, GR_CHNL, LV_WARN, "ldah_send_receive_rqb_sync, data_len=%u, rqbLen(%lu) > channel->send_pool_length (%lu)",
                                      add_data_len, rqbLen, channel->send_pool_length);

    result= ldah_sr_rqb_ext_sync(channel, tm_ms, ldaf_opcode, vmdid, add_data_len, add_data);   /* SEGMENTED SEND SYNC */

    TRM_OUT02(&ti, GR_CHNL, LV_WARN, "<-- ldah_send_receive_rqb_sync, data_len=%u DONE! result=%#x ",add_data_len, result);
    return result;  // <<< EXIT <<<
  }

  pRqb = (FWLDA_DPR_RQB*)DPR_VMALLOC(rqbLen);
  if(!pRqb) {
    TRM_OUT00(&ti, GR_INIT, LV_ERR, "ldah_send_receive_rqb_sync, alloc memory for FWLDA_DPR_RQB failed");
    return PNIO_ERR_NO_RESOURCE;
  }

  /* check if SDB upload command */
  if ( ldaf_opcode == FWLDA_OP_GET_SDB_RQB ) {
      // set new opcode to signal that we can receive x-large sdbs
      // without this, the firmware can't decide if send of XL sdbs is ok
      ldaf_opcode = FWLDA_OP_GET_SDB_XL_RQB;
  }
  pRqb->dpr_hdr.hostref = CPU_TO_LE(channel->user_id);
  pRqb->dpr_hdr.subsystem = FWLDA_SUBSYS_INDEX;      /* Fix: AP01394131 */
  pRqb->dpr_hdr.userid = CPU_TO_LE(0xabcd);
  pRqb->dpr_hdr.userdatalength = CPU_TO_LE(rqbLen);

  pRqb->fwlda_rqb.opcode = (FWLDA_OPCODE)CPU_TO_LE(ldaf_opcode);
  pRqb->fwlda_rqb.ovs_inst_id = CPU_TO_LE(vmdid);
  pRqb->fwlda_rqb.rqb_len = CPU_TO_LE(sizeof(pRqb->fwlda_rqb) + add_data_len);

  if(add_data_len) {
    memcpy((DPR_CHAR*)pRqb + sizeof(FWLDA_DPR_RQB), add_data, add_data_len);
  }

  ret = DPR_DRV_WRITE(channel->mgt_chnl, pRqb, rqbLen);

  DPR_VFREE(pRqb);
  pRqb = 0;

  if(ret != rqbLen) {
    TRM_OUT03(&ti, GR_CHNL, LV_ERR,
              "Send failed, written %u bytes from %d, error '%s'",
              ret, rqbLen, DPR_STRERROR());
    return PNIO_ERR_NO_FW_COMMUNICATION;
  } else {
    TRM_OUT02(&ti, GR_CHNL, LV_INFO, "ldah_send_receive_rqb_sync, sent rqb to fw opcode=%d, len=%d",
              ldaf_opcode, rqbLen);
  }

  /* RECEIVE CONFIRMATION */

  /* receive confirmation from the firmware (from the driver) */
  TRM_OUT01(&ti, GR_CHNL, LV_INFO, "ldah_send_receive_rqb_sync: RECV RESP. req-opcode=%d",ldaf_opcode);

  /* receive confirmation msg synchronously */
  unsigned long received_len = 0;
  result = ldah_mgt_ch_read(channel, &received_len);
  if ( result != PNIO_OK ) {
      TRM_OUT00(&ti, GR_CHNL, LV_ERR, "ldah_send_receive_rqb_sync: ch_read ERROR");
      return result;
  }
  if ( received_len == 0 ) {
      TRM_OUT00(&ti, GR_CHNL, LV_ERR, "ch_read ERR LEN=0");
      return PNIO_ERR_INTERNAL;
  }

  pRqb = (FWLDA_DPR_RQB *)channel->user_pool_ptr;

  if( (pRqb->dpr_hdr.subsystem != FWLDA_SUBSYS_INDEX) || (LE_TO_CPU(pRqb->dpr_hdr.userid) != 0xabcd) ) {
    TRM_OUT00(&ti, GR_CHNL, LV_ERR, "ldah_send_receive_rqb_sync, wrong dpr channel identification");
    return PNIO_ERR_INTERNAL;
  }

  // Changes to receive x-large WEB SDBs segmented
  // check for new xl-sdb upload confirmation - receive all data sgments
  if(LE_TO_CPU(pRqb->fwlda_rqb.opcode) == FWLDA_OP_GET_SDB_XL_SEG_CNF ) {
      DPR_UINT32 result = PNIO_OK;
      unsigned int rdlen  = 0;
      TRM_OUT02(&ti, GR_CHNL, LV_INFO, "ldah_send_receive_rqb_sync: XL-SDB-CNF= %d pool-len=%u", pRqb->fwlda_rqb.opcode, channel->user_pool_length);
      result = ldah_rcv_xl_cnf_sync (channel, pRqb->fwlda_rqb.opcode, &rdlen, pprd);
      return result;
  }

  /* other confirmation */
  /* check is the rqb a confirmation */
  if(LE_TO_CPU(pRqb->fwlda_rqb.opcode) != (ldaf_opcode + 1)) {
    TRM_OUT02(&ti, GR_CHNL, LV_ERR, "ldah_send_receive_rqb_sync, wrong opcode %d (expected %d)", pRqb->fwlda_rqb.opcode, ldaf_opcode + 1);
    return PNIO_ERR_WRONG_RQB_LEN;
  }

  if(LE_TO_CPU(pRqb->fwlda_rqb.response) != SUBSYS_FWLDA_OK) {
    TRM_OUT01(&ti, GR_CHNL, LV_ERR, "ldah_send_receive_rqb_sync, confirmation error response %d", pRqb->fwlda_rqb.response);
    return PNIO_ERR_INTERNAL;
  }

  return PNIO_OK;
} /* end of ldah_send_receive_rqb_sync */


/*-----------------------------------------------------------------------------
 * Name  : ldah_set_timestamp_ovssdb_list
 * Descr :
 * Return: ==0: PNIO_OK, != 0 error
 */
DPR_UINT32 ldah_set_timestamp_ovssdb_list(LDA_OVSSDB_LIST_HEADER * p_ovssdb_list,
                                          DPR_CHAR timestamp[LDA_OVS_TIMESTAMP_LEN])
{
  DPR_UINT32 i;
  LDA_OVS_PNIO_CNSTY_PRM_BLK *pPnioCnstyPrmBlk = 0;

  LDA_OVS_CNSTY_SDB *pCnstyPrmBlk = 0;

  for(i=0; i<p_ovssdb_list->SDBDirCnt; i++) {

    /*printf("OVSSDB len=%-4d vmdid=%-3d number=%-4d content=0x%-4x offset=%d\n",
        p_ovssdb_list->SDBDir[i].SDBLen, p_ovssdb_list->SDBDir[i].VmdID,
        p_ovssdb_list->SDBDir[i].SDBNumber, p_ovssdb_list->SDBDir[i].SDBContent,
        p_ovssdb_list->SDBDir[i].OffsetOfSdbBody) ;*/

    LDA_OVS_BST *pSdbHdr = (LDA_OVS_BST *)(p_ovssdb_list->SDBDatas + p_ovssdb_list->SDBDir[i].OffsetOfSdbBody);

    /* set timestamp value */
    memcpy(pSdbHdr->head.time_stamp1, timestamp, LDA_OVS_TIMESTAMP_LEN);
    memcpy(pSdbHdr->head.time_stamp2, timestamp, LDA_OVS_TIMESTAMP_LEN);

    /* find PNIO consystency SDB and CP consystency SDB and set cnsty attribut */
    if( p_ovssdb_list->SDBDir[i].SDBNumber >= 1000 ) {
      if(p_ovssdb_list->SDBDir[i].SDBContent == LDA_OVS_SDB_ID_PNIO_CNSTY) {
        pPnioCnstyPrmBlk = (LDA_OVS_PNIO_CNSTY_PRM_BLK *)((DPR_CHAR*)pSdbHdr + sizeof(LDA_OVS_BST));
      }

      if(p_ovssdb_list->SDBDir[i].SDBContent == LDA_OVS_SDB_ID_CNSTY) {
        pCnstyPrmBlk = (LDA_OVS_CNSTY_SDB *)((DPR_CHAR*)pSdbHdr + sizeof(LDA_OVS_BST));
      }

      /* set cnsty attribut */
      switch(p_ovssdb_list->SDBDir[i].SDBContent) {
      case LDA_OVS_SDB_ID_CNSTY:
        pSdbHdr->info.cnsty_attr = LDA_OVS_CNSTY_ATTR_IS_CNSTY_SDB;
        break;
      case LDA_OVS_SDB_ID_PNIO_CNSTY:
        pSdbHdr->info.cnsty_attr = LDA_OVS_CNSTY_ATTR_IS_AND_INCLUD_IN_CNSTY_SDB;
        break;
    default:
        if(p_ovssdb_list->SDBDir[i].VmdID == LDA_XDB_IPC_SLOT_NUMBER){
          pSdbHdr->info.cnsty_attr = LDA_OVS_CNSTY_ATTR_NO_CNSTY;
        }
        else {
          pSdbHdr->info.cnsty_attr = LDA_OVS_CNSTY_ATTR_INCLUD_IN_CNSTY_SDB;
        }
        break;
      }
    }
  }

  if(pCnstyPrmBlk) {
  LDA_OVS_CNSTY_DATA *pCnstyDataItem = (LDA_OVS_CNSTY_DATA *)((DPR_CHAR*)pCnstyPrmBlk + sizeof(*pCnstyPrmBlk));
  for(i=0; (DPR_UINT16)i< pCnstyPrmBlk->SdbCnt; i++) {
    memcpy(pCnstyDataItem->TimeStamp, timestamp, LDA_OVS_TIMESTAMP_LEN);
    TRM_OUT02(&ti, GR_CHNL, LV_INFO, "ldah_set_timestamp_ovssdb_list, set timest=x..%x sdb_nr=%d in CP-cnsty sdb",
        *(DPR_UINT32*)timestamp, lda_be16_2_nat(pCnstyDataItem->BlockNumber));

    pCnstyDataItem++;
  }
  }
  else {
    TRM_OUT00(&ti, GR_CHNL, LV_WARN, "ldah_set_timestamp_ovssdb_list, can't find consistency SDB");
  }


  if(pPnioCnstyPrmBlk) {

    LDA_OVS_PNIO_CNSTY_DATA *pPnioCnstyDataItem = (LDA_OVS_PNIO_CNSTY_DATA *)((DPR_CHAR*)pPnioCnstyPrmBlk + sizeof(*pPnioCnstyPrmBlk));

    for(i=0; i< lda_be16_2_nat(pPnioCnstyPrmBlk->SDB_Cnt); i++) {
      memcpy(pPnioCnstyDataItem->TimeStamp, timestamp, LDA_OVS_TIMESTAMP_LEN);
      TRM_OUT02(&ti, GR_CHNL, LV_INFO, "ldah_set_timestamp_ovssdb_list, set timest=x..%x sdb_nr=%d in PNIO-cnsty sdb",
                *(DPR_UINT32*)timestamp, lda_be16_2_nat(pPnioCnstyDataItem->BlockNumber));

      pPnioCnstyDataItem++;
    }
  }

  return PNIO_OK;
}

DPR_UINT32 ovsid_from_vmdid(DPR_UINT32 vmdid)
{
  DPR_UINT32 ovsid;
  switch(vmdid)
  {
  case LDA_XDB_STM_SLOT_NUMBER:
    ovsid = FWLDA_OVS_STM_PROJECT;
    break;
  case LDA_XDB_IPC_SLOT_NUMBER:
    ovsid = FWLDA_OVS_CP_REMANENT;
    break;
  case LDA_XDB_CP_SLOT_NUMBER:
  default:
  ovsid = FWLDA_OVS_CP_PROJECT;
    //TRM_OUT01(&ti, GR_CHNL, LV_ERR, "unknown vmdid=%d\n", vmdid);
    break;
  }

  return ovsid;
}

DPR_UINT32 vmdid_from_ovsid(DPR_UINT32 ovsid)
{
  DPR_UINT32 vmdid = 0xffffffff;
  switch(ovsid)
  {
  case FWLDA_OVS_STM_PROJECT:
    vmdid = LDA_XDB_STM_SLOT_NUMBER;
    break;
  case FWLDA_OVS_CP_PROJECT:
    vmdid = LDA_XDB_CP_SLOT_NUMBER;
    break;
  case FWLDA_OVS_CP_REMANENT:
    vmdid = LDA_XDB_IPC_SLOT_NUMBER;
    break;
  default:
    TRM_OUT01(&ti, GR_CHNL, LV_ERR, "unknown ovsid=%d\n", ovsid);
    break;
  }

  return vmdid;
}


/*-----------------------------------------------------------------------------
 * Name  : ldah_deinit_mgt_communication
 * Descr : unbind, free channel's pool memory and close mgt channel.
 *         Stop/delete receiver worker thread, delete rcv coordination sema
 * Param :
 *  [ IN]: channel
 * Return: -
 */
void ldah_deinit_mgt_communication(CP_FWLDA *channel)
{
  DPR_UINT32 fct_ret = PNIO_OK;
  struct t_read_pool   usr_pool;

  TRM_OUT02(&ti, GR_INIT, LV_FCTPUB1, "ldah_deinit_mgt_communication cp_idx=%d user_id=%d", channel->cp_index, channel->user_id);

  channel->stop_thread = 1;

  usr_pool.user_id = channel->user_id;
  if(DPR_DRV_IOCTL(channel->mgt_chnl, CP16XX_IOC_UNBIND,
                   &usr_pool, sizeof(usr_pool), 0) < 0) {
    TRM_OUT01(&ti, GR_INIT, LV_ERR, "ioctl unbind failed, error '%s'", DPR_STRERROR());
    fct_ret = PNIO_ERR_DRIVER_IOCTL_FAILED;
  }

  TRM_OUT00(&ti, GR_INIT, LV_FCTPUB1, "ldah_deinit_mgt_communication CP16XX_IOC_UNBIND ok");

  if(channel->th_reader) {
    TRM_OUT01(&ti, GR_INIT, LV_FCTPUB1, "Stop thread, handle %x", channel->th_reader);

    DPR_THREAD_JOIN(channel->th_reader);
    channel->th_reader = 0;
  }

  if(channel->user_pool_ptr) DPR_VFREE(channel->user_pool_ptr);

  DPR_SEM_DESTROY(channel->sem_cnf_received);

  if(channel->mgt_chnl) {
    DPR_DRV_CLOSE(channel->mgt_chnl);
    channel->mgt_chnl = NULL;
  }

  TRM_OUT00(&ti, GR_INIT, LV_FCTPUB1, "ldah_deinit_mgt_communication DONE");
  return;
} /* end of ldah_deinit_mgt_communication */

/*-----------------------------------------------------------------------------
 * Name  : ldah_deinit_mgt_ch
 * Descr : unbind, free channel's pool memory and close mgt channel
 * Param :
 *  [ IN]: channel
 * Return: -
 */
void ldah_deinit_mgt_ch(CP_FWLDA *channel)
{
  DPR_UINT32 fct_ret = PNIO_OK;
  struct t_read_pool   usr_pool;

  TRM_OUT02(&ti, GR_INIT, LV_FCTPUB1, ">ldah_deinit_mgt_ch cp_idx=%d user_id=%d", channel->cp_index, channel->user_id);

  usr_pool.user_id = channel->user_id;
  if(DPR_DRV_IOCTL(channel->mgt_chnl, CP16XX_IOC_UNBIND,
                   &usr_pool, sizeof(usr_pool), 0) < 0) {
    TRM_OUT01(&ti, GR_INIT, LV_ERR, "ioctl unbind failed, error '%s'", DPR_STRERROR());
    fct_ret = PNIO_ERR_DRIVER_IOCTL_FAILED;
  }

  TRM_OUT00(&ti, GR_INIT, LV_FCTPUB1, "ldah_deinit_mgt_ch CP16XX_IOC_UNBIND ok");

  if(channel->user_pool_ptr) DPR_VFREE(channel->user_pool_ptr);

  if(channel->mgt_chnl) {
    DPR_DRV_CLOSE(channel->mgt_chnl);
    channel->mgt_chnl = NULL;
  }

  TRM_OUT00(&ti, GR_INIT, LV_FCTPUB1, " <ldah_deinit_mgt_ch done");
  return;
} /* end of ldah_deinit_mgt_ch */

/*-----------------------------------------------------------------------------
 * Name  : ldah_init_mgt_communication
 * Descr : Opens management channel for communication with the firmware, binds
 *         the channel.
 *         Creates receiver thread and semaphore
 *         Note: current application has to be registered before calling this fct.
 * Param :
 *  [ IN]: Channel:cp_index: board id
 *                 user_id : user id returned by the driver appl. registry
 *
 *  [OUT]: channel->mgt_chnl:         channel handle
 *         channel->send_pool_length: send pool length
 *         channel->user_pool_length: rceive pool length
 *         channel->user_pool_ptr   : allocated pool memory
 *
 * Return: ==0: ok, != 0 error: PNIO_ERR_xxx
 */
DPR_UINT32 ldah_init_mgt_communication(CP_FWLDA *channel)
{
  char tmp[36];
  struct t_read_pool  usr_pool;
  DPR_UINT32 fct_ret = PNIO_OK;

  TRM_OUT02(&ti, GR_INIT, LV_INFO, ">ldah_init_mgt_communication cp_idx=%d user_id=%d", channel->cp_index, channel->user_id);

  snprintf(tmp, sizeof(tmp)-1, DPR_MGT_INTERFACE, (unsigned int)(channel->cp_index - 1));
  channel->mgt_chnl = DPR_DRV_OPEN(tmp);
  if(!channel->mgt_chnl) {
    TRM_OUT01(&ti, GR_INIT, LV_ERR, "opening '%s' failed", tmp);
    fct_ret = PNIO_ERR_CREATE_INSTANCE;
    goto ldah_init_mgt_communication_end;
  }

  if(DPR_SEM_CREATE(channel->sem_cnf_received)) {
    TRM_OUT00(&ti, GR_INIT, LV_ERR,
            "DPR_SEM_CREATE(channel->sem_cnf_received) failed");
    fct_ret = PNIO_ERR_INTERNAL;
    goto ldah_init_mgt_communication_end;
  }

  usr_pool.user_id = channel->user_id;
  if(DPR_DRV_IOCTL(channel->mgt_chnl, CP16XX_IOC_BIND,
                   &usr_pool, sizeof(usr_pool), sizeof(usr_pool)) < 0) {
    TRM_OUT02(&ti, GR_INIT, LV_ERR, "ioctl bind '%s' failed, error '%s'",
              tmp, DPR_STRERROR());
    fct_ret = PNIO_ERR_DRIVER_IOCTL_FAILED;
    goto ldah_init_mgt_communication_end;
  }

  TRM_OUT02(&ti, GR_INIT, LV_FCTPUB1, "ioctl bind successfuly, pool read len=%lu pool write len=%lu",
                                     usr_pool.read_length, usr_pool.write_length);

  if(usr_pool.write_length == 0 ||
      usr_pool.read_length == 0){
    TRM_OUT00(&ti, GR_INIT, LV_WARN, "ioctl bind successfuly, but dpr-pools still not initialized");
    fct_ret = PNIO_ERR_NO_FW_COMMUNICATION;
    goto ldah_init_mgt_communication_end;
  }

  channel->send_pool_length = usr_pool.write_length;
  channel->user_pool_length = usr_pool.read_length;
  channel->user_pool_ptr = (DPR_CHAR*)DPR_VMALLOC(channel->user_pool_length);
  if(!channel->user_pool_ptr) {
    TRM_OUT01(&ti, GR_INIT, LV_ERR, "alloc memory pool for '%s' failed", tmp);
    fct_ret = PNIO_ERR_OS_RES;
    channel->user_pool_length = 0;
    goto ldah_init_mgt_communication_end;
  }

  channel->stop_thread = 0;
  /* thread for processing the callback messages */
  if(!DPR_THREAD_CREATE(&channel->th_reader, "", procMgtChannelReadDownload, channel)) {
    TRM_OUT01(&ti, GR_MGT, LV_ERR, "Create thread for read from '%s' failed", tmp);
    fct_ret = PNIO_ERR_CREATE_INSTANCE;
    goto ldah_init_mgt_communication_end;
  }

  TRM_OUT02(&ti, GR_INIT, LV_FCTPUB1, "Create thread for '%s' ok, handle %x",
            tmp, channel->th_reader);

  ldah_init_mgt_communication_end:

  if(fct_ret != PNIO_OK) {
    ldah_deinit_mgt_communication(channel);
  }

  return fct_ret;

} /* end of ldah_init_mgt_communication */

/*-----------------------------------------------------------------------------
 * Name  : ldah_init_mgt_ch
 * Descr : Init communication channel only, without receiver thread.
 *         Receive is called directly from "sender thread".
 *         Note: current application has to be registered before calling this fct.
 * Param :
 *  [ IN]: Channel:cp_index: board id
 *                 user_id : user id returned by the driver appl. registry
 *
 *  [OUT]: channel->mgt_chnl:         channel handle
 *         channel->send_pool_length: send pool length
 *         channel->user_pool_length: rceive pool length
 *         channel->user_pool_ptr   : allocated pool memory
 *
 * Return: ==0: ok, != 0 error: PNIO_ERR_xxx
 */
DPR_UINT32 ldah_init_mgt_ch ( CP_FWLDA *channel )
{
  char tmp[36];
  struct t_read_pool  usr_pool;
  DPR_UINT32 fct_ret = PNIO_OK;

  TRM_OUT02(&ti, GR_INIT, LV_INFO, "ldah_init_mgt_ch cp_idx=%d user_id=%d", channel->cp_index, channel->user_id);

  snprintf(tmp, sizeof(tmp)-1, DPR_MGT_INTERFACE, (unsigned int)(channel->cp_index - 1));
  channel->mgt_chnl = DPR_DRV_OPEN(tmp);
  if(!channel->mgt_chnl) {
    TRM_OUT01(&ti, GR_INIT, LV_ERR, "opening '%s' failed", tmp);
    fct_ret = PNIO_ERR_CREATE_INSTANCE;
    goto ldah_init_mgt_ch_end;
  }

  /* bind this user to the channel */
  usr_pool.user_id = channel->user_id;
  if(DPR_DRV_IOCTL(channel->mgt_chnl, CP16XX_IOC_BIND,
                   &usr_pool, sizeof(usr_pool), sizeof(usr_pool)) < 0) {
    TRM_OUT02(&ti, GR_INIT, LV_ERR, "ioctl bind '%s' failed, error '%s'",
              tmp, DPR_STRERROR());
    fct_ret = PNIO_ERR_DRIVER_IOCTL_FAILED;
    goto ldah_init_mgt_ch_end;
  }

  TRM_OUT02(&ti, GR_INIT, LV_INFO, "ioctl bind successfuly, pool read len=%lu pool write len=%lu",
                                     usr_pool.read_length, usr_pool.write_length);

  if(usr_pool.write_length == 0 ||
      usr_pool.read_length == 0){
    TRM_OUT00(&ti, GR_INIT, LV_WARN, "ioctl bind successfuly, but dpr-pools still not initialized");
    fct_ret = PNIO_ERR_NO_FW_COMMUNICATION;
    goto ldah_init_mgt_ch_end;
  }

  channel->send_pool_length = usr_pool.write_length;
  channel->user_pool_length = usr_pool.read_length;
  channel->user_pool_ptr = (DPR_CHAR*)DPR_VMALLOC(channel->user_pool_length);
  if(!channel->user_pool_ptr) {
    TRM_OUT01(&ti, GR_INIT, LV_ERR, "alloc memory pool for '%s' failed", tmp);
    fct_ret = PNIO_ERR_OS_RES;
    channel->user_pool_length = 0;
    goto ldah_init_mgt_ch_end;
  }

  ldah_init_mgt_ch_end:

  if(fct_ret != PNIO_OK) {
    ldah_deinit_mgt_ch(channel);
  }

  return fct_ret;
} /* end of ldah_init_mgt_ch */


DPR_UINT32 ldah_test_mgt_communication(DPR_UINT32 cp_idx, unsigned long user_id)
{
  CP_FWLDA channel;
  DPR_UINT32 fct_ret;
  DPR_CHAR mgt_communication_initialized = 0;

  memset(&channel, 0, sizeof(channel));

  channel.cp_index = cp_idx;
  channel.user_id = user_id;

  for(int i=0; i<30; i++) {

    if(mgt_communication_initialized == 0){
      fct_ret = ldah_init_mgt_communication(&channel);
      if(fct_ret != PNIO_OK) {
        TRM_OUT02(&ti, GR_INIT, LV_INFO, "ldah_init_mgt_communication(cp_idx=%u) failed, ret=0x%x, try again", cp_idx, fct_ret);
        DPR_TASK_DELAY(2000);
        continue;
      }
      else{
        mgt_communication_initialized = 1;
      }
    }

    fct_ret = ldah_send_receive_rqb(&channel, 2000, FWLDA_OP_GET_FW_INFO_RQB, 0, 0, 0);
    TRM_OUT01(&ti, GR_INIT, LV_FCTPUB1, "ldah_send_receive_rqb(GET_FW_INFO) ret 0x%x", fct_ret);

    if(fct_ret == PNIO_OK) {
      /* firmware does arnswer host request */
      TRM_OUT01(&ti, GR_INIT, LV_FCTPUB1, "ldah_send_receive_rqb(GET_FW_INFO) answered after %d sec.", i*2);
      break;
    }
    else if(fct_ret == PNIO_ERR_WRONG_RQB_LEN){
      /* firmware not supports this host request, possible old version */
      TRM_OUT00(&ti, GR_INIT, LV_FCTPUB1, "ldah_send_receive_rqb(GET_FW_INFO) not supported, possible old fw version");
      break;
    }
    else
    {
      DPR_TASK_DELAY(2000);
    }
  }

  if(fct_ret != PNIO_OK){
    TRM_OUT00(&ti, GR_INIT, LV_FCTPUB1, "firmware run up can not be verificated");
    fct_ret = PNIO_ERR_NO_RESET_VERIFICATION;
  }

  ldah_deinit_mgt_communication(&channel);

  return fct_ret;
}

DPR_CHAR ldah_is_IPC_sdb_available(LDA_OVSSDB_LIST_HEADER * p_ovssdb_list)
{
  for(DPR_UINT32 i = 0; i< p_ovssdb_list->SDBDirCnt; i++ ) {
    if(FWLDA_OVS_CP_REMANENT == ovsid_from_vmdid(p_ovssdb_list->SDBDir[i].VmdID)){
      TRM_OUT00(&ti, GR_INIT, LV_FCTPUB1, "ldah_is_IPC_sdb_available: IPC sdb available, perform IPC-Start/StopDownload");
      return 1;
    }
  }
  return 0;
}

/*-----------------------------------------------------------------------------
 * Name  : ldah_download_ovssdb_list
 * Descr : Writes SDBs to the cp16xx. Called by ldah_download_config_intern()
 * Return: ==0: == PNIO_OK, != 0 PNIO_ERR_xxx
 */
DPR_UINT32 ldah_download_ovssdb_list(DPR_UINT32 cp_idx, DPR_DRV_HANDLE con_fd, unsigned long user_id,
                                     LDA_OVSSDB_LIST_HEADER * p_ovssdb_list)
{
  DPR_UINT32 sdb_ovsid;

  CP_FWLDA channel;
  int time_out;

  DPR_UINT32 fct_ret = PNIO_OK;

  DPR_CHAR is_IPC_sdbs_available = ldah_is_IPC_sdb_available(p_ovssdb_list);

  FWLDA_START_SUBSYSTEM start_rqb_data;
  memset(&start_rqb_data, 0, sizeof(start_rqb_data));
  start_rqb_data.blk_len = sizeof(start_rqb_data);

  TRM_OUT00(&ti, GR_INIT, LV_FCTPUB1, "-> ldah_download_ovssdb_list");

  memset(&channel, 0, sizeof(channel));

  channel.fd_control = con_fd;
  channel.cp_index = cp_idx;
  channel.user_id = user_id;

  fct_ret = ldah_init_mgt_communication(&channel);
  if(fct_ret != 0) {
    TRM_OUT02(&ti, GR_INIT, LV_ERR, "ldah_init_mgt_communication(cp_idx=%u) failed, ret=0x%x", cp_idx, fct_ret);
    return fct_ret;
  }

  TRM_OUT01(&ti, GR_INIT, LV_FCTPUB1, "has to load %d SDBs", p_ovssdb_list->SDBDirCnt);

  int do_reset2factory = 1; /* default= do reset */
  if (do_reset2factory) {
	  time_out = 80000;
	  TRM_OUT01(&ti, GR_INIT, LV_FCTPUB1, "RESET to factory time_out=%d[sec]",(int)time_out/1000);
      fct_ret = ldah_send_receive_rqb(&channel, time_out, FWLDA_OP_RESET_TO_FACTORY_RQB, 0, 0, 0);
      if(fct_ret != PNIO_OK) {
        TRM_OUT01(&ti, GR_MGT, LV_ERR, "ldah_send_receive_rqb(RESET_TO_FACTORY) failed ret 0x%x", fct_ret);
        fct_ret = PNIO_ERR_INTERNAL;
        goto ldah_download_ovssdb_list_fail_do_deinit;
      }
  } /* end if do reset to factory */
  if(is_IPC_sdbs_available){
  TRM_OUT00(&ti, GR_INIT, LV_FCTPUB1, "START download sequence and delete all SDBs in CP RAM");
  fct_ret = ldah_send_receive_rqb(&channel, 2000, FWLDA_OP_DOWNLOAD_START_RQB, 0, 0, 0);
  if(fct_ret != PNIO_OK) {
    TRM_OUT01(&ti, GR_MGT, LV_ERR, "ldah_send_receive_rqb(START) failed ret 0x%x", fct_ret);
    fct_ret = PNIO_ERR_INTERNAL;
    goto ldah_download_ovssdb_list_fail_do_deinit;
  }
  }

/*#define DELETE_SDBS*/
#if DELETE_SDBS
  TRM_OUT00(&ti, GR_INIT, LV_FCTPUB1, "DELETE all SDBs in CP RAM");
  fct_ret = ldah_send_receive_rqb(&channel, 120000, FWLDA_OP_DELALL_SDBS_RQB, 0, 0, 0);
  if(fct_ret != PNIO_OK) {
    TRM_OUT01(&ti, GR_MGT, LV_ERR, "ldah_send_receive_rqb(DELALL_SDBS) failed ret 0x%x", fct_ret);
    fct_ret = PNIO_ERR_INTERNAL;
    goto ldah_download_ovssdb_list_fail_do_deinit;
  }
#else
  TRM_OUT00(&ti, GR_MGT, LV_WARN, " INFO: DELETE_ALL_SDBS OFF! (is OK)");
#endif

  /* start sdb verteiler */
  #if 0
  TRM_OUT00(&ti, GR_INIT, LV_FCTPUB1, "START subsystem: sdb distributor");
  start_rqb_data.subsytem_id = 1; /* start sdb-verteiler */
  fct_ret = ldah_send_receive_rqb(&channel, 10000, FWLDA_OP_START_MGT_RQB, FWLDA_OVS_CP_PROJECT,
                                    sizeof(start_rqb_data), (DPR_CHAR*)&start_rqb_data);
  if(fct_ret != PNIO_OK) {
    TRM_OUT01(&ti, GR_MGT, LV_ERR, "ldah_send_receive_rqb(START_SDB_DISTRIB) failed ret 0x%x", fct_ret);
    fct_ret = PNIO_ERR_INTERNAL;
    goto ldah_download_ovssdb_list_fail_do_deinit;
  }
  #endif

  for(DPR_UINT32 i = 0; i< p_ovssdb_list->SDBDirCnt; i++ ) {
    sdb_ovsid = ovsid_from_vmdid(p_ovssdb_list->SDBDir[i].VmdID);

    TRM_OUT03(&ti, GR_INIT, LV_FCTPUB1, "DOWNLOAD SDB: sdb_nr=%d sdb_len=%d ovsid=%d",
              p_ovssdb_list->SDBDir[i].SDBNumber, p_ovssdb_list->SDBDir[i].SDBLen, sdb_ovsid);
    /* TRM_OUTD(&ti, GR_INIT, LV_FCTPUB1, (DPR_CHAR*)(p_ovssdb_list->SDBDatas + p_ovssdb_list->SDBDir[i].OffsetOfSdbBody),
                             p_ovssdb_list->SDBDir[i].SDBLen);*/

#ifdef _DEBUG
	time_out = 360000; /* 6 min */
#else
	time_out = 120000;
#endif
    fct_ret = ldah_send_receive_rqb(&channel, time_out, FWLDA_OP_DOWNLOAD_SDB_RQB, sdb_ovsid,
                                           p_ovssdb_list->SDBDir[i].SDBLen,
                                           p_ovssdb_list->SDBDatas + p_ovssdb_list->SDBDir[i].OffsetOfSdbBody);
    if(fct_ret != PNIO_OK) {
      TRM_OUT01(&ti, GR_MGT, LV_ERR, "ldah_send_receive_rqb(DOWNLOAD_SDB) failed ret 0x%x", fct_ret);
      fct_ret = PNIO_ERR_INTERNAL;
      goto ldah_download_ovssdb_list_fail_do_deinit;
    }

  } /* end for all SDBs */

  if(is_IPC_sdbs_available){
  TRM_OUT00(&ti, GR_INIT, LV_FCTPUB1, "END download sequence");
#ifdef _DEBUG
	time_out = 360000; /* 6 min */
#else
	time_out = 60000;
#endif

  fct_ret = ldah_send_receive_rqb(&channel, time_out, FWLDA_OP_DOWNLOAD_END_RQB, 0, 0, 0);
  if(fct_ret != PNIO_OK) {
    TRM_OUT01(&ti, GR_MGT, LV_ERR, "ldah_send_receive_rqb(END) failed ret 0x%x", fct_ret);
    fct_ret = PNIO_ERR_INTERNAL;
    goto ldah_download_ovssdb_list_fail_do_deinit;
  }
  }

  /* start ip subsystems after download */
  TRM_OUT00(&ti, GR_INIT, LV_FCTPUB1, "START subsystem: ip subsystems");
#ifdef _DEBUG
	time_out = 480000; /* 8 min */
#else
	time_out = 120000;
#endif
  start_rqb_data.subsytem_id = 0; /* start ip system group */
  fct_ret = ldah_send_receive_rqb(&channel, time_out, FWLDA_OP_START_MGT_RQB, FWLDA_OVS_CP_PROJECT,
                                    sizeof(start_rqb_data), (DPR_CHAR*)&start_rqb_data);
  if(fct_ret != PNIO_OK) {
    TRM_OUT01(&ti, GR_MGT, LV_ERR, "ldah_send_receive_rqb(START_MGT,CP_PROJECT) failed ret 0x%x", fct_ret);
    fct_ret = PNIO_ERR_INTERNAL;
    goto ldah_download_ovssdb_list_fail_do_deinit;
  }

  TRM_OUT00(&ti, GR_INIT, LV_FCTPUB1, "COPY ram configuration to rom");
  /* save configuration for Stationsmanager in ROM */
#ifdef _DEBUG
	time_out = 480000; /* 8 min */
#else
	time_out = 120000;
#endif
  fct_ret = ldah_send_receive_rqb(&channel, time_out, FWLDA_OP_COPY_RAM_TO_ROM_RQB, FWLDA_OVS_STM_PROJECT, 0, 0);
  if(fct_ret != PNIO_OK) {
    TRM_OUT01(&ti, GR_MGT, LV_ERR, "ldah_send_receive_rqb(RAM_TO_ROM,STM_PROJECT) failed ret 0x%x", fct_ret);
    fct_ret = PNIO_ERR_INTERNAL;
    goto ldah_download_ovssdb_list_fail_do_deinit;
  }

  /* save configuration for CP in ROM */
#ifdef _DEBUG
	time_out = 600000; /* 10 min */
#else
	time_out = 360000;
#endif

  fct_ret = ldah_send_receive_rqb(&channel, time_out, FWLDA_OP_COPY_RAM_TO_ROM_RQB, FWLDA_OVS_CP_PROJECT, 0, 0);
  if(fct_ret != PNIO_OK) {
    TRM_OUT01(&ti, GR_MGT, LV_ERR, "ldah_send_receive_rqb(RAM_TO_ROM,CP_PROJECT) failed ret 0x%x", fct_ret);
    fct_ret = PNIO_ERR_INTERNAL;
    goto ldah_download_ovssdb_list_fail_do_deinit;
  }

  /* save configuration for IPC in ROM */

#ifdef _DEBUG
	time_out = 480000; /* 8 min */
#else
	time_out = 120000;
#endif
  fct_ret = ldah_send_receive_rqb(&channel, time_out, FWLDA_OP_COPY_RAM_TO_ROM_RQB, FWLDA_OVS_CP_REMANENT, 0, 0);
  if(fct_ret != PNIO_OK) {
    TRM_OUT01(&ti, GR_MGT, LV_ERR, "ldah_send_receive_rqb(RAM_TO_ROM,CP_REMANENT) failed ret 0x%x", fct_ret);
    fct_ret = PNIO_ERR_INTERNAL;
    goto ldah_download_ovssdb_list_fail_do_deinit;
  }

  ldah_download_ovssdb_list_fail_do_deinit:

  ldah_deinit_mgt_communication(&channel);

  TRM_OUT01(&ti, GR_INIT, LV_FCTPUB1, "<- ldah_download_ovssdb_list, ret = 0x%x", fct_ret);
  return fct_ret;
} /* end of ldah_download_ovssdb_list */

DPR_UINT32 ldah_reset_to_factory(DPR_UINT32 cp_idx, DPR_DRV_HANDLE con_fd, unsigned long user_id)
{
  CP_FWLDA channel;

  DPR_UINT32 fct_ret = PNIO_OK;

  TRM_OUT00(&ti, GR_INIT, LV_FCTPUB1, "-> ldah_reset_to_factory");

  memset(&channel, 0, sizeof(channel));

  channel.fd_control = con_fd;
  channel.cp_index = cp_idx;
  channel.user_id = user_id;

  fct_ret = ldah_init_mgt_communication(&channel);
  if(fct_ret != 0) {
    TRM_OUT02(&ti, GR_INIT, LV_ERR, "ldah_init_mgt_communication(cp_idx=%u) failed, ret=0x%x", cp_idx, fct_ret);
    return fct_ret;
  }

  fct_ret = ldah_send_receive_rqb(&channel, 40000, FWLDA_OP_RESET_TO_FACTORY_RQB, 0, 0, 0);
  if(fct_ret != PNIO_OK) {
    TRM_OUT01(&ti, GR_MGT, LV_ERR, "ldah_send_receive_rqb(RESET_TO_FACTORY) failed ret 0x%x", fct_ret);
    fct_ret = PNIO_ERR_INTERNAL;
  }

  ldah_deinit_mgt_communication(&channel);

  TRM_OUT01(&ti, GR_INIT, LV_FCTPUB1, "<- ldah_reset_to_factory, ret = 0x%x", fct_ret);
  return fct_ret;
}

DPR_UINT32 ldah_download_config_intern(DPR_UINT32 cp_idx, DPR_DRV_HANDLE con_fd, unsigned long user_id, DPR_CHAR *p_data, DPR_UINT32 data_length)
{
  DPR_UINT32 ret;
  LDA_OVSSDB_LIST_HEADER *p_ovssdb_list = 0;
  DPR_CHAR ovs_timestamp[LDA_OVS_TIMESTAMP_LEN];
  DPR_UINT32 utc_time;
  int do_download = 1;  /* debug aim only, default=1 */

  /* XDB -> XDB_SDB_List -> CPOVS_SDB_List -> set timestamps -> download in CP */
  ret = ldah_create_ovssdb_list(p_data, data_length, &p_ovssdb_list);
  if(ret != PNIO_OK) {
    TRM_OUT02(&ti, GR_MGT, LV_ERR, "ldah_create_ovssdb_list data_length=%d ret=0x%x",
              data_length, ret);
    goto ldah_download_config_free_list;
  }

  utc_time = CPU_TO_BE(DPR_DRV_UTCTIME());

  memset(ovs_timestamp, 0, sizeof(ovs_timestamp));
  /* copy utc_time in last bytes of ovs_timestamp */
  memcpy(ovs_timestamp + (sizeof(ovs_timestamp) - sizeof(utc_time)), &utc_time, sizeof(utc_time));

  TRM_OUT00(&ti, GR_MGT, LV_INFO, "ldah_set_timestamp_ovssdb_list deaktivated\n");

  /*
  ret = ldah_set_timestamp_ovssdb_list(p_ovssdb_list, ovs_timestamp);
  if(ret != PNIO_OK) {
    TRM_OUT02(&ti, GR_MGT, LV_ERR, "ldah_set_timestamp_ovssdb_list data_length=%d ret=0x%x",
              data_length, ret);
    goto ldah_download_config_free_list;
  }
  */

  /* only for test */
  for(DPR_UINT32 i=0; i<p_ovssdb_list->SDBDirCnt; i++) {
    DPR_UINT16 sdb_nr = p_ovssdb_list->SDBDir[i].SDBNumber;

    TRM_OUT05(&ti, GR_MGT, LV_FCTPUB2, "OVSSDB len=%-4d vmdid=%-3d number=%-4d content=0x%-4x offset=%d",
              p_ovssdb_list->SDBDir[i].SDBLen, p_ovssdb_list->SDBDir[i].VmdID,
              p_ovssdb_list->SDBDir[i].SDBNumber, p_ovssdb_list->SDBDir[i].SDBContent,
              p_ovssdb_list->SDBDir[i].OffsetOfSdbBody) ;
    /*TRM_OUTD(&ti, GR_MGT, LV_INFO, p_ovssdb_list->SDBDatas + p_ovssdb_list->SDBDir[i].OffsetOfSdbBody,
                               p_ovssdb_list->SDBDir[i].SDBLen);*/

    #if defined DUMP_SDB_CONTENT
        DPR_UINT16 debug_sdb_nr = 1045;
        if( sdb_nr == debug_sdb_nr ) {
            dbg_printf("\nSDB-DUMP: nr=%d len=%u \n",sdb_nr, p_ovssdb_list->SDBDir[i].SDBLen);
            sysa_dump_buf(p_ovssdb_list->SDBDatas + p_ovssdb_list->SDBDir[i].OffsetOfSdbBody, p_ovssdb_list->SDBDir[i].SDBLen);
            dbg_printf("SDB-DUMP: nr=%d len=%u END\n",sdb_nr, p_ovssdb_list->SDBDir[i].SDBLen);
        }
    #endif // DUMP_SDB_CONTENT
  }

  if (do_download) {
      ret = ldah_download_ovssdb_list(cp_idx, con_fd, user_id, p_ovssdb_list);
      if(ret != PNIO_OK) {
        TRM_OUT02(&ti, GR_MGT, LV_ERR, "ldah_download_ovssdb_list cp_idx=%d ret=0x%x",
                  cp_idx, ret);
      }
  }

  ldah_download_config_free_list:

  ldah_free_ovssdb_list(&p_ovssdb_list);

  return ret;
}

/*-----------------------------------------------------------------------------
 * Name  : ldah_download_config
 * Descr : writes S7 config data (XDB file) into the cp16xx
 * Param :
 *  [ IN]: cp_idx: Board index
 *         p_data: Ptr to da buffer (file content)
 *         data_length: num bytes to be written - downloaded
 *  [OUT]: -
 * Return: ==0: == PNIO_OK, != 0 PNIO_ERR_xxx
 */
DPR_UINT32 ldah_download_config(DPR_UINT32 cp_idx, DPR_CHAR *p_data, DPR_UINT32 data_length)
{
  DPR_UINT32 ret;

  struct t_register_app app;
  DPR_CHAR *mapStart = 0;    // pointer to mapped cp16xx flash memory - NOT USED here
  DPR_DRV_HANDLE con_fd = 0;

  ret = ldah_open(cp_idx, &app, &mapStart, &con_fd);
  if(ret != PNIO_OK) {
    TRM_OUT02(&ti, GR_MGT, LV_ERR, "ldah_download_config, ldah_open cp_idx=%d ret=0x%x",
              cp_idx, ret);
    goto ldah_download_config_end;
  }

  ret = ldah_download_config_intern(cp_idx, con_fd, app.user_id, p_data, data_length);
  if(ret != PNIO_OK) {
    TRM_OUT02(&ti, GR_MGT, LV_ERR, "ldah_download_config, ldah_download_config_intern cp_idx=%d ret=0x%x",
              cp_idx, ret);
  }

  ldah_close(con_fd, &app, mapStart);

  ldah_download_config_end:

  return ret;
} /* end of ldah_download_config */

DPR_UINT32 ldah_reset_firmware(DPR_UINT32 cp_idx, DPR_CHAR type)
{
  DPR_UINT32 ioctrl_ret, ret = PNIO_OK;

  char control_fd_path[64];
  DPR_DRV_HANDLE control_fd = 0;
  DPR_CHAR * mapStart = 0;
  struct t_register_app app;
  int loc_errno, ioct = (type == 0) ? CP16XX_IOCSHUTDOWN : CP16XX_IOCRESET;

#if defined(_DPR_WINDOWS)
  #define WAITTIME_AFTER_RESET_SEC  45 /* because of Ndis combined driver we have to wait longer */
#else
  #define WAITTIME_AFTER_RESET_SEC  25
#endif
  int waittime_after_reset = WAITTIME_AFTER_RESET_SEC; /* sec. */

  if(cp_idx < 1) {
    TRM_OUT01(&ti, GR_INIT, LV_ERR, "invalid cp-index=%d (1st index is 1)", cp_idx);
    return PNIO_ERR_PRM_CP_ID;
  }

  snprintf(control_fd_path, sizeof(control_fd_path)-1, DPR_CONTROL_INTERFACE, (unsigned int)(cp_idx - 1));

  if(!(control_fd = DPR_DRV_OPEN(control_fd_path))) {
    TRM_OUT01(&ti, GR_INIT, LV_ERR, "open <%s> failed", control_fd_path);
    return PNIO_ERR_PRM_CP_ID;
  }

  ioctrl_ret = DPR_DRV_IOCTL(control_fd, ioct, NULL, 0, 0);   /*  <---  RESET THE HW   */

  loc_errno = errno;

  DPR_DRV_CLOSE(control_fd);

  if(ioctrl_ret) {
    TRM_OUT01(&ti, GR_INIT, LV_ERR, "resetting the card failed, error '%s'",
                                             strerror(loc_errno));
    ret = PNIO_ERR_DRIVER_IOCTL_FAILED;
  } else {
    TRM_OUT00(&ti, GR_INIT, LV_INFO, "card resetting kicked off, check fw run up...");

    ret = ldah_open(cp_idx, &app, &mapStart, &control_fd);
    if(ret != PNIO_OK) {
      return ret;
    }

    TRM_OUT01(&ti, GR_INIT, LV_INFO, "firmware needs %d sec. at least to perform reset, wait...",
                 waittime_after_reset);
    DPR_TASK_DELAY(waittime_after_reset * 1000);
    /* connect firmware, read life time, check does firmware perform reset */
    ret = ldah_test_mgt_communication(cp_idx, app.user_id);

    ldah_close(control_fd, &app, (void*)mapStart);
  }

  return ret;
}

/*-----------------------------------------------------------------------------
 * Name  : ldah_cp_set_time
 * Descr :
 * Param :
 *  [ IN]:
 *  [OUT]:
 * Return: ==0: PNIO_OK, != 0 PNIO_ERR_xxx
 */
DPR_UINT32 ldah_cp_set_time ( DPR_UINT32 CpIndex, PNIO_CP_SET_TIME_TYPE *pCpTime )
{
    struct t_register_app app;
    DPR_DRV_HANDLE control_fd = 0;
    DPR_CHAR * mapStart = 0;
    CP_FWLDA channel;
    DPR_UINT32 ret = PNIO_OK;
    DPR_UINT32 time_out_ms;

    ret = ldah_open(CpIndex, &app, &mapStart, &control_fd);
    if(ret != PNIO_OK) {
      return ret;
    }
    TRM_OUT01(&ti, GR_INIT, LV_INFO, "ldah_cp_set_time: DRV opened. cp-idx=%d",CpIndex);

    //ret = ldah_test_mgt_communication(cp_idx, app.user_id);

    memset(&channel, 0, sizeof(channel));
    channel.cp_index = CpIndex;
    channel.user_id  = app.user_id;

    ret = ldah_init_mgt_communication(&channel);
    if(ret != PNIO_OK) {
      TRM_OUT02(&ti, GR_INIT, LV_ERR, "ldah_cp_set_time: ldah_init_mgt_communication(cp-idx=%u) failed, ret=%#x", CpIndex, ret);
      goto set_time_end;
    }

    time_out_ms = 3000;
    ret = ldah_send_receive_rqb(&channel, time_out_ms, FWLDA_OP_CP_SET_TIME_RQB, 0, sizeof(PNIO_CP_SET_TIME_TYPE), (DPR_CHAR*)pCpTime);
    if(ret != PNIO_OK) {
      TRM_OUT01(&ti, GR_MGT, LV_ERR, "ldah_cp_set_time: send-rqb failed ret %#x", ret);
      ret = PNIO_ERR_INTERNAL;
    }
    else {
      TRM_OUT01(&ti, GR_INIT, LV_INFO, "ldah_cp_set_time: send-rqb ret 0x%x", ret);
    }

    set_time_end:
    ldah_close(control_fd, &app, (void*)mapStart);

    return ret;
} /* end of ldah_cp_set_time */

/*-----------------------------------------------------------------------------
 * Name  : ldah_cp_set_type_of_station
 * Descr :
 * Param :
 *  [ IN]:
 *  [OUT]:
 * Return: ==0: PNIO_OK, != 0 PNIO_ERR_xxx
 */
DPR_UINT32 ldah_cp_set_type_of_station (PNIO_UINT32 CpIndex, char* TypeName)
{
    DPR_UINT32 ret = PNIO_OK;
    struct t_register_app app;
    DPR_DRV_HANDLE control_fd = 0;
    DPR_CHAR * mapStart = 0;
    CP_FWLDA channel;
    DPR_UINT32 time_out_ms;
    int type_name_len = strlen(TypeName);

    // catt of given type name if too long
    if(type_name_len > 255) {
        type_name_len = 255;
        *(TypeName + type_name_len) = '\0';
    }

    ret = ldah_open(CpIndex, &app, &mapStart, &control_fd);
    if(ret != PNIO_OK) {
      TRM_OUT02(&ti, GR_INIT, LV_ERR, "ldah_cp_set_type_of_station: ldah_open failed, ret=%#x", CpIndex, ret);
      return ret;
    }
    TRM_OUT01(&ti, GR_INIT, LV_INFO, "ldah_cp_set_type_of_station: DRV opened. cp-idx=%d",CpIndex);

    //ret = ldah_test_mgt_communication(cp_idx, app.user_id);

    memset(&channel, 0, sizeof(channel));
    channel.cp_index = CpIndex;
    channel.user_id  = app.user_id;

    ret = ldah_init_mgt_communication(&channel);
    if(ret != PNIO_OK) {
      TRM_OUT02(&ti, GR_INIT, LV_ERR, "ldah_cp_set_type_of_station: ldah_init_mgt_communication(cp-idx=%u) failed, ret=%#x", CpIndex, ret);
      goto set_tos_end;
    }

    time_out_ms = 3000;
    ret = ldah_send_receive_rqb(&channel, time_out_ms, FWLDA_OP_CP_SET_TYPE_OF_STATION_RQB, 0, type_name_len + 1, (DPR_CHAR*)TypeName);
    if(ret != PNIO_OK) {
      TRM_OUT01(&ti, GR_MGT, LV_ERR, "ldah_cp_set_type_of_station: send-recv-rqb failed ret %#x", ret);
      ret = PNIO_ERR_INTERNAL;
    }
    else {
      TRM_OUT00(&ti, GR_INIT, LV_INFO, "ldah_cp_set_type_of_station: send-recv-rqb ret OK");
    }

    set_tos_end:
    ldah_close(control_fd, &app, (void*)mapStart);

    return ret;
} /* end of ldah_cp_set_type_of_station */


#if defined DUMP_SDB_CONTENT
/************************************************************************
*******    TRACE  DUMP  HELPERS           *******************************
*************************************************************************/

void __cdecl dbg_printf(char *szFormat, ... )
{
    va_list pArg;
    char szMessage[1024];
    szMessage[0] = '\0';

    va_start(pArg, szFormat);
    int msgLen = _vsnprintf (szMessage, sizeof(szMessage) - 3, szFormat, pArg);
    va_end(pArg);

    char *p = szMessage;
    p += (msgLen < 0) ?  sizeof(szMessage) - 3 : msgLen;
    *p++ = '\r';
    *p++ = '\n';
    *p++ = '\0';

    OutputDebugString( szMessage );   /*  Windows ONLY !!! */
    return;
} // end of dbg_printf()


/************************************************************************
 * Name  : dump_lineOut
 * Descrp: prints one line out
 *         lineBuf: line buffer  : Zero terminated ASCII string with '\n'
 * output: -
 */
void dump_lineOut( char *lineBuf ) {

    dbg_printf("%s",lineBuf);  /* OutputDebugString() --> debuger or DbgView output  Windows ONLY !!! */

    // printf("%s\n",lineBuf);
    return;
}     /* end of dump_lineOut() */

/************************************************************************
 * Name  : dump_buildHdr
 * Descrp: Creates offset number as ascii string i.e. dump line prefix (header)
 * input : val: value
 *         buf: Line-Buffer for the dump line. Line header is placed at the begin.
 *         len: available length for the header (should be 10)
 * Note  : here   len = 10;   |00000000: .. .. .. ..
 *                            |<-------->|. .. .. ..
 * output: -
 */
void dump_buildHdr( unsigned int val, char *buf, int len ) {

    char hex_digits[16]={'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
    int bridx = len - 1;  // buffer right index  == 9,  8,7,6,...0
    int digit_hex;
    unsigned int tmp_val;

    //for ( i = 0; i < len; i++ )
    //    *(buf+i) = ' '; // clear available header space with blanks

    /* set separators of the header-line ( 00000000: hh hh ... ) i.e. ": "  at index 8,9 */
    *(buf + bridx) = ' ';
    bridx--;
    *(buf + bridx) = ':';
    bridx--;                   // index of the LSB hex digit

    /* write value as hex digit ascii => itoa */
    tmp_val = val;
    do {
        digit_hex = (tmp_val % 16);
        *(buf + bridx) = hex_digits[digit_hex];    // convert int 0x0 - 0xF  --> ascii char '0' - 'F'
        bridx--;
        tmp_val /= 16;
    } while ( tmp_val );

    for ( ; bridx > 0; --bridx ) {
        *(buf + bridx) = '0';      // fill padding zeros
    }

    return;
} /* end of dump_buildHdr() */

/************************************************************************
 * Name  : sysa_dump_buf
 * Descrp:
 *   This routine dumps a data field. The length of the dump can be given by
 *   argument.
 *   Format of one output line (output line index):
 *   0  - 48: Hex bytes (16 bytes, 3 ASCII-digits for a byte => 16*3=48 )
 *   49     : blank
 *   50 - 65: ASCII chars (16 bytes, 1 byte = 1 ASCII-char  => 16 ASCII digits)
 *   66     : \0
 *   67 - 79: not used
 * input : outBuff: output buffer
 *         inBuff : buffer to be dumped
 *         length : number of bytes to be dumped
 * output: -
 */
void sysa_dump_buf( void *inBuff, unsigned int length ) {

    char hex_digits[16] =
        {
            '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'
        };

    char output_line[82]; // 0-79 + \r\n\0
    char *line_address;
    int hix, ai, lineNum, byteOffset, outLineIdx, i;
    unsigned char byteVal;
    char *buffer = (char*)inBuff;
    int hdrLen = 10; /*|00000000: hh hh hh ... */

    /* preset output line, set \0 at the end */
    for ( i = 0; i < sizeof(output_line) - 1; output_line[i++] = ' ' )
        ;

    output_line[i] = '\0';
    line_address = buffer;
    lineNum = 0;
    byteOffset = 0;
    outLineIdx = 0;

    /* init hex digit index and ascii byte index */
    hix = 0 + hdrLen;
    ai = 50 + hdrLen;

    /* dump content of the input buffer */
    while (length--) {
        byteVal = *buffer++;

        /* hex part */
        output_line[hix++] = hex_digits[byteVal >> 4];  /* high nibbel */
        output_line[hix++] = hex_digits[byteVal & 0xF]; /* low nibbel  */
        hix++;                                          /* blank       */

        /* ascii part */
        output_line[ai++] = (byteVal >= ' ' && byteVal < 127) ? byteVal : '.';

        if (ai > (65 + hdrLen)) {
            /* one line content is complete, print it out:
             */
            dump_buildHdr(byteOffset, output_line, hdrLen);
            dump_lineOut(output_line);

            /* clear line buffer, set counter */
            for ( i = 0; i < sizeof(output_line) - 1; output_line[i++] = ' ' )
                ;

            output_line[i] = '\0';
            hix = 0 + hdrLen;
            ai = 50 + hdrLen;
            line_address = buffer; /* address is not used */
            lineNum++;
            byteOffset += 16;
        }
    } /* while */

    /* if there are not yet printed parts of a line, display them
     * I.e. print out last not complete line
     */
    if (hix > (0 + hdrLen)) {
        dump_buildHdr(byteOffset, output_line, hdrLen);
        dump_lineOut(output_line);
    }

} /* end of dump_buf() */

#endif  // DUMP_SDB_CONTENT
