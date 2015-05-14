/****************************************************************************
*  Copyright (C) SIEMENS CORP., 2013 All rights reserved.
*****************************************************************************
* FILE NAME    : ldah_flash.cpp
* ---------------------------------------------------------------------------
* DESCRIPTION  : managed cp replace functions
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
#include "ldah_flash.h"
#include "lda_spec.h"
#include "dpr_msg.h"
#include "sysa_fwlda_ifhost.h"
#include "pnioerrx.h"
#include "servusrx.h"
#include "tracemcrm.h"

#define UMASK      00777               /* use user's umask for creating files */
#define LDAH_BACKUP_FW_IMAGE_FILE    "/backup/16xx_cp%lu_fw_backup.fwl"
#define LDAH_BACKUP_S7PRJ_FILE       "/backup/16xx_cp%lu_s7prj_backup.xdb"
#define LDAH_BACKUP_S7PRJTMS_FILE    "/backup/16xx_cp%lu_s7prjtms_backup.txt"
#define LDAH_BACKUP_SER_NUMBER_FILE  "/backup/16xx_cp%lu_ser_nr_backup.txt"

#define FWLDA_RET_OK             0x00000000
#define FWLDA_RET_FILE_NOT_EXIST 0x00000001
#define FWLDA_RET_INTERNAL       0x00000002 /* see library trace for more information */

/* external objects */
extern DPR_MUTEX ServMutexDownload;
extern CTrmWrapper ti;

/* external functions */

void ldah_close(DPR_DRV_HANDLE control_fd, struct t_register_app *app, void *mapStart);
DPR_UINT32 ldah_open(unsigned long cp_idx, struct t_register_app *app, DPR_CHAR **ppmapStart, DPR_DRV_HANDLE *con_fd);


#if defined (LDAH_TEST)
    DPR_UINT32 get_sdb_test (DPR_UINT32 cp_idx, unsigned long user_id );
#endif

bool ldah_get_fw_image_version(char *p_fw_image, unsigned long image_length, LDA_FW_VERS *p_fw_version);

DPR_UINT32 ldah_reset_firmware(DPR_UINT32 cp_idx, DPR_CHAR type);

DPR_UINT32 ldah_init_mgt_communication(CP_FWLDA *channel);
void       ldah_deinit_mgt_communication(CP_FWLDA *channel);
DPR_UINT32 ldah_init_mgt_ch(CP_FWLDA *channel);               // channel with sync receive (without receiver thread)
void       ldah_deinit_mgt_ch(CP_FWLDA *channel);

DPR_UINT32 ldah_reset_to_factory(DPR_UINT32 cp_idx, DPR_DRV_HANDLE con_fd, unsigned long user_id);
DPR_UINT32 ldah_download_config_intern(DPR_UINT32 cp_idx, DPR_DRV_HANDLE con_fd, unsigned long user_id, DPR_CHAR *p_data, DPR_UINT32 data_length);
DPR_UINT32 ldah_send_receive_rqb(CP_FWLDA *channel, DPR_UINT32 tm_ms, FWLDA_OPCODE ldaf_opcode,
                                        DPR_UINT32 vmdid, DPR_UINT32 add_data_len, DPR_CHAR* add_data, DPR_CHAR** ppr=NULL);
DPR_UINT32 ldah_send_receive_rqb_sync(CP_FWLDA *channel, DPR_UINT32 tm_ms, FWLDA_OPCODE ldaf_opcode,
                                        DPR_UINT32 vmdid, DPR_UINT32 add_data_len, DPR_CHAR* add_data, DPR_CHAR** ppr=NULL);

DPR_UINT32 ldah_create_ovssdb_list(DPR_CHAR *p_data, DPR_UINT32 data_length,
                                   LDA_OVSSDB_LIST_HEADER ** p_ovssdb_list);
DPR_UINT32 ldah_add_sdb_to_ovssdb_list(LDA_OVSSDB_LIST_HEADER *p_hdr, DPR_UINT32 sdb_ovsid, DPR_CHAR *sdb_data);

DPR_UINT32 ldah_free_ovssdb_list(LDA_OVSSDB_LIST_HEADER ** p_hdr);
DPR_UINT32 ldah_init_ovssdb_list(LDA_OVSSDB_LIST_HEADER ** p_hdr);
DPR_UINT32 vmdid_from_ovsid(DPR_UINT32 ovsid);
DPR_UINT32 ovsid_from_vmdid(DPR_UINT32 vmdid);

static DPR_UINT32 ldah_serial_nr_read_backup(unsigned long cp_idx, char *backup_ser_nr, unsigned long ser_nr_len)
{
  char serial_nr_filename[512];
  int fw_fd, loc_errno;
  unsigned long ret;

  memset(backup_ser_nr, 0, ser_nr_len);

  strcpy(serial_nr_filename, DPR_DRV_CONFIG_PATH());
  snprintf(serial_nr_filename + strlen(serial_nr_filename),
           sizeof(serial_nr_filename) - strlen(serial_nr_filename),
           LDAH_BACKUP_SER_NUMBER_FILE, cp_idx);

  fw_fd = open(serial_nr_filename, O_RDONLY | O_BINARY, UMASK);
  if(fw_fd == -1) {
    TRM_OUT01(&ti, GR_INIT, LV_INFO, "Cannot open backup file <%s>", serial_nr_filename);
    return FWLDA_RET_FILE_NOT_EXIST;
  } else {

    ret = read(fw_fd, backup_ser_nr, ser_nr_len);

    loc_errno = errno;

    (void)close(fw_fd);

    if(ret != ser_nr_len) {
      TRM_OUT04(&ti, GR_INIT, LV_ERR, "wrong serial_nr length %d (expected %d), errno %i errno-txt %s",
                                                ret, ser_nr_len, loc_errno, strerror(loc_errno));
      return FWLDA_RET_INTERNAL;
    }
  }

  return FWLDA_RET_OK;
}

static
unsigned long ldah_serial_nr_write_backup(unsigned long cp_idx, char *backup_ser_nr, unsigned long ser_nr_length)
{
  char serial_nr_filename[512];
  int old_process_mode_mask, fw_fd, loc_errno;
  unsigned long ret;

  strcpy(serial_nr_filename, DPR_DRV_CONFIG_PATH());
  snprintf(serial_nr_filename + strlen(serial_nr_filename),
           sizeof(serial_nr_filename) - strlen(serial_nr_filename),
           LDAH_BACKUP_SER_NUMBER_FILE, cp_idx);

  /* sets the calling process's file mode creation mask (umask) to mask & 0777 */
  old_process_mode_mask = umask(0); /* umask() modified global-process umask */
                                    /* to create file which can be processed from each user */
  fw_fd = open(serial_nr_filename, O_RDWR | O_BINARY | O_CREAT | O_TRUNC, UMASK);

  /* restore process's file mode creation mask */
  (void)umask(old_process_mode_mask);

  if(fw_fd == -1) {
    TRM_OUT01(&ti, GR_INIT, LV_INFO, "Cannot open backup file <%s>", serial_nr_filename);
    return FWLDA_RET_FILE_NOT_EXIST;
  } else {

    ret = write(fw_fd, backup_ser_nr, ser_nr_length);

    loc_errno = errno;

    (void)close(fw_fd);

    if(ret != ser_nr_length) {
      TRM_OUT04(&ti, GR_INIT, LV_ERR, "wrong length written %d (expected %d), errno %i errno-txt %s",
                                          ret, ser_nr_length, loc_errno, strerror(loc_errno));
      return FWLDA_RET_INTERNAL;
    }
  }

  return FWLDA_RET_OK;
}

static
unsigned long ldah_fw_image_read_backup_version(unsigned long cp_idx, LDA_FW_VERS *fw_version_backup)
{
  char fw_image_filename[512];
  int fw_fd;
  struct stat fw_stat;

  strcpy(fw_image_filename, DPR_DRV_CONFIG_PATH());
  snprintf(fw_image_filename + strlen(fw_image_filename),
           sizeof(fw_image_filename) - strlen(fw_image_filename),
           LDAH_BACKUP_FW_IMAGE_FILE, cp_idx);

  fw_fd = open(fw_image_filename, O_RDONLY | O_BINARY, UMASK);
  if(fw_fd == -1) {
    TRM_OUT01(&ti, GR_INIT, LV_INFO, "Cannot open backup file <%s>", fw_image_filename);
    return FWLDA_RET_FILE_NOT_EXIST;
  } else {

    /* get file size */
    if(fstat(fw_fd, &fw_stat) == -1) {
      TRM_OUT01(&ti, GR_INIT, LV_ERR, "fstat failed, errno-txt %s", strerror(errno));
      (void)close(fw_fd);
      return FWLDA_RET_INTERNAL;
    }

    char *p_fw_image = (char*)DPR_VMALLOC(fw_stat.st_size);
    if(p_fw_image == 0) {
      TRM_OUT01(&ti, GR_INIT, LV_ERR, "can't allocate %d bytes for fw-image", fw_stat.st_size);
      (void)close(fw_fd);
      return FWLDA_RET_INTERNAL;
    }

    if(read(fw_fd, p_fw_image, fw_stat.st_size) <= 0 ) {
      TRM_OUT01(&ti, GR_INIT, LV_ERR, "read failed, errno-txt %s", strerror(errno));
      (void)close(fw_fd);
      return FWLDA_RET_INTERNAL;
    }

    ldah_get_fw_image_version(p_fw_image, fw_stat.st_size, fw_version_backup);

    DPR_VFREE(p_fw_image);
    (void)close(fw_fd);
  }

  return FWLDA_RET_OK;
}

/* save firmware image as backup */
static
unsigned long ldah_fw_image_write_backup(unsigned long cp_idx, DPR_DRV_HANDLE control_fd, unsigned long user_id,
                                         FWLDA_FW_VERS *fw_version_from_cp)
{
  unsigned long ret, len;
  int old_process_mode_mask, fw_fd, loc_errno;
  char fw_image_filename[512];

  strcpy(fw_image_filename, DPR_DRV_CONFIG_PATH());
  snprintf(fw_image_filename + strlen(fw_image_filename),
           sizeof(fw_image_filename) - strlen(fw_image_filename),
           LDAH_BACKUP_FW_IMAGE_FILE, cp_idx);

  /* sets the calling process's file mode creation mask (umask) to mask & 0777 */
  old_process_mode_mask = umask(0); /* umask() modified global-process umask */
                                    /* to create file which can be processed from each user */
  /* Dump Firmware */
  fw_fd = open(fw_image_filename, O_RDWR | O_BINARY | O_CREAT | O_TRUNC, UMASK);

  /* restore process's file mode creation mask */
  (void)umask(old_process_mode_mask);

  if(fw_fd == -1) {
    TRM_OUT01(&ti, GR_INIT, LV_ERR, "Cannot open backup file <%s>", fw_image_filename);
    return FWLDA_RET_FILE_NOT_EXIST;
  }

  /* create fwl header */
  char LadFileVersionStr[128];

  snprintf(LadFileVersionStr, sizeof(LadFileVersionStr)-1,
           " "LDA_LAD_FILE_VERSION_STR"%d.%d.%d.%d.%d", fw_version_from_cp->t1, fw_version_from_cp->t2,
                                        fw_version_from_cp->t3, fw_version_from_cp->t4, fw_version_from_cp->t5);
  LadFileVersionStr[sizeof(LadFileVersionStr)-1] = 0;
  DPR_UINT32 LadFileVersionStrLen = (DPR_UINT32)strlen(LadFileVersionStr);

  DPR_UINT32 FwlHeaderLen = sizeof(LDA_FWL_HEADER) + LadFileVersionStrLen;
  LDA_FWL_HEADER *pFwlHeader = (LDA_FWL_HEADER*)DPR_VMALLOC(FwlHeaderLen);
  if(!pFwlHeader) {
    TRM_OUT01(&ti, GR_INIT, LV_ERR, "ldah_fw_image_write_backup; can't allocate %d bytes", FwlHeaderLen);
    (void)close(fw_fd);
    return FWLDA_RET_INTERNAL;
  }

  pFwlHeader->LadFileOffset = lda_nat_2_le16(FwlHeaderLen);
  pFwlHeader->VersionOfFwlFormat = lda_nat_2_le16(5);
  pFwlHeader->FwlSignaturLen = LDA_FWL_SIGNATUR_LEN;

  memcpy(pFwlHeader->FwlSignatur, LDA_FWL_SIGNATUR, LDA_FWL_SIGNATUR_LEN);
  pFwlHeader->LadFileVersionLen = LadFileVersionStrLen;

  memcpy((DPR_CHAR*)pFwlHeader + sizeof(*pFwlHeader), LadFileVersionStr, LadFileVersionStrLen);

  ret = write(fw_fd, pFwlHeader, FwlHeaderLen);

  loc_errno = errno;

  DPR_VFREE(pFwlHeader);

  if(ret != FwlHeaderLen) {
    TRM_OUT03(&ti, GR_INIT, LV_ERR, "ldah_fw_image_write_backup; cannot write fwl header len=%ul ret=%ul, error '%s'",
                                      FwlHeaderLen, ret, strerror(loc_errno));
    (void)close(fw_fd);
    return FWLDA_RET_INTERNAL;
  }

  len = FLASH_FS_OFFSET - FW_START_OFFSET;

  /* Map the whole flash into user space */
  volatile char *pCpFlash = (volatile char *)DPR_DRV_MMAP(control_fd, MMAP_OFFSET_EMIF_CS0, F_SIZE, user_id);
  if(pCpFlash == 0) {
    TRM_OUT01(&ti, GR_INIT, LV_ERR, "ldah_fw_image_write_backup; mmap MMAP_OFFSET_EMIF_CS0(Flash area) failed, error '%s'", strerror(errno));
    (void)close(fw_fd);
    return FWLDA_RET_INTERNAL;
  }

  /* find netto/net fw image length */
  volatile DPR_UINT16 *pFlashWord = (volatile  DPR_UINT16 *)(pCpFlash + FW_START_OFFSET + len - sizeof(*pFlashWord));

  /* start from flash end, till first word with value != 0xffff */
  while(*pFlashWord == 0xFFFF && len > 0) {
    len -= sizeof(*pFlashWord);
    pFlashWord--;
  }

  if(len == 0) {
    TRM_OUT01(&ti, GR_INIT, LV_ERR, "ldah_fw_image_write_backup; can't dump the firmware image, fw image len = %ul", len);
    DPR_DRV_MUNMAP(control_fd, (void*)pCpFlash, F_SIZE, user_id);
    (void)close(fw_fd);
    return FWLDA_RET_INTERNAL;
  }

  /* only for test
  for(unsigned long i = 0; i<len; i++){
      printf("%02x ", *((unsigned char*)pCpFlash + FW_START_OFFSET + i) );
      if((i+1)%32 == 0) printf("\n");
    }
    printf("\n");*/

  /* slow but save
  pFlashWord = (volatile DPR_UINT16 *)(pCpFlash + FW_START_OFFSET);
  while(len > 0){
    ret = write(fw_fd, (void*)pFlashWord, sizeof(*pFlashWord));
    len -= sizeof(*pFlashWord);
    pFlashWord++;
  }*/

  /*don't use direct write(fw_fd, pCpFlash + FW_START_OFFSET, len);
    because OS try to optimize read of flash data and reads wrong */

  /* optimized */
  pFlashWord = (volatile DPR_UINT16 *)(pCpFlash + FW_START_OFFSET);
  DPR_UINT16 *pFwImg = (DPR_UINT16 *)DPR_VMALLOC(len);

  if(!pFwImg) {
    TRM_OUT01(&ti, GR_INIT, LV_ERR, "ldah_fw_image_write_backup; cannot allocate %ul bytes for pFwImg", len);
    DPR_DRV_MUNMAP(control_fd, (void*)pCpFlash, F_SIZE, user_id);
    (void)close(fw_fd);
    return FWLDA_RET_INTERNAL;
  } else {
    memcpy(pFwImg, (void*)pFlashWord, len);

    ret = write(fw_fd, pFwImg, len);
    if(pFwImg) DPR_VFREE(pFwImg);
  }

  DPR_DRV_MUNMAP(control_fd, (void*)pCpFlash, F_SIZE, user_id);
  (void)close(fw_fd);

  if(ret != len) {
    TRM_OUT02(&ti, GR_INIT, LV_ERR, "ldah_fw_image_write_backup; cannot write fw image len=%ul ret=%ul", len, ret);
    return FWLDA_RET_INTERNAL;
  }

  return FWLDA_RET_OK;
}

DPR_UINT32 ldah_image_restore(unsigned long cp_idx, DPR_CHAR *mapFlash,
                                  DPR_UINT32 start_offset, DPR_UINT32 end_offset, char *file_name);

extern "C"
int ldah_compare_fwlda_sdb_dat_items(const void *i1, const void *i2)
{
  FWLDA_SDB_DAT *p1 = (FWLDA_SDB_DAT*)i1,
                      *p2 = (FWLDA_SDB_DAT*)i2;

  if(p1->ovs_inst == p2->ovs_inst) {
    if(p1->nr == p2->nr) {
      return(p1->context - p2->context);
    } else
      return(p1->nr - p2->nr);
  } else
    return(p1->ovs_inst - p2->ovs_inst);

}

/* get list of sdbs from backup xdb file */
DPR_UINT32 ldah_s7prj_read_backup_info(DPR_UINT32 cp_idx, DPR_UINT32 *sdb_cnt, FWLDA_SDB_DAT *sdbs)
{
  DPR_UINT32 ret, fct_ret = FWLDA_RET_OK;
  char s7prj_backup_filename[512];
  int fw_fd = 0;
  struct stat fw_stat;
  LDA_OVSSDB_LIST_HEADER  *pOVSSDB_hdr = 0;
  DPR_CHAR *p_s7prj_backup = 0;

  strcpy(s7prj_backup_filename, DPR_DRV_CONFIG_PATH());
  snprintf(s7prj_backup_filename + strlen(s7prj_backup_filename),
           sizeof(s7prj_backup_filename) - strlen(s7prj_backup_filename),
           LDAH_BACKUP_S7PRJ_FILE, (unsigned long)cp_idx);

  fw_fd = open(s7prj_backup_filename, O_RDONLY | O_BINARY, UMASK);
  if(fw_fd == -1) {
    TRM_OUT01(&ti, GR_INIT, LV_INFO, "Cannot open backup file <%s>", s7prj_backup_filename);
    fct_ret = FWLDA_RET_FILE_NOT_EXIST;
    goto ldah_s7prj_read_backup_info_end;
  }

  /* get file size */
  if(fstat(fw_fd, &fw_stat) == -1) {
    TRM_OUT01(&ti, GR_INIT, LV_ERR, "fstat failed, errno-txt %s", strerror(errno));
    fct_ret = FWLDA_RET_INTERNAL;
    goto ldah_s7prj_read_backup_info_end;
  }

  p_s7prj_backup = (DPR_CHAR*)DPR_VMALLOC(fw_stat.st_size);
  if(p_s7prj_backup == 0) {
    TRM_OUT01(&ti, GR_INIT, LV_ERR, "can't allocate %d bytes for fw-image", fw_stat.st_size);
    fct_ret = FWLDA_RET_INTERNAL;
    goto ldah_s7prj_read_backup_info_end;
  }

  if(read(fw_fd, p_s7prj_backup, fw_stat.st_size) <= 0 ) {
    TRM_OUT01(&ti, GR_INIT, LV_ERR, "read failed, errno-txt %s", strerror(errno));
    fct_ret = FWLDA_RET_INTERNAL;
    DPR_VFREE(p_s7prj_backup);
    goto ldah_s7prj_read_backup_info_end;
  }

  ret = ldah_create_ovssdb_list(p_s7prj_backup, fw_stat.st_size, &pOVSSDB_hdr);

  if(p_s7prj_backup) DPR_VFREE(p_s7prj_backup);

  if(ret != PNIO_OK) {
    TRM_OUT01(&ti, GR_INIT, LV_ERR, "ldah_create_ovssdb_list failed, ret=0x%x", ret);
    fct_ret = FWLDA_RET_INTERNAL;
    goto ldah_s7prj_read_backup_info_end;
  }

  if(pOVSSDB_hdr->SDBDirCnt > *sdb_cnt) {
    TRM_OUT02(&ti, GR_INIT, LV_ERR, "pOVSSDB_hdr->SDBDirCnt(%d) > *sdb_cnt(%d)", pOVSSDB_hdr->SDBDirCnt, *sdb_cnt);
    fct_ret = FWLDA_RET_INTERNAL;
    goto ldah_s7prj_read_backup_info_end;
  }

  for(DPR_UINT32 i=0; i<pOVSSDB_hdr->SDBDirCnt; i++) {
    sdbs[i].sdb_length = pOVSSDB_hdr->SDBDir[i].SDBLen;
    sdbs[i].ovs_inst = ovsid_from_vmdid(pOVSSDB_hdr->SDBDir[i].VmdID);
    sdbs[i].context = pOVSSDB_hdr->SDBDir[i].SDBContent;
    sdbs[i].nr = pOVSSDB_hdr->SDBDir[i].SDBNumber;
    memcpy(sdbs[i].timestamp, pOVSSDB_hdr->SDBDir[i].SdbTimeStamp, LDA_OVS_TIMESTAMP_LEN);
  }

  *sdb_cnt = pOVSSDB_hdr->SDBDirCnt;

  /*(void)ldah_s7prj_read_sdb_timestamps(cp_idx, sdb_cnt, sdbs);*/

  ldah_s7prj_read_backup_info_end:

  if(pOVSSDB_hdr) (void)ldah_free_ovssdb_list(&pOVSSDB_hdr);

  if(fw_fd != -1)(void)close(fw_fd);

  if(fct_ret != FWLDA_RET_OK) {
    *sdb_cnt = 0;
  }

  return fct_ret;
}

/*-----------------------------------------------------------------------------
 * Name  : ldah_write_xdb_backup
 * Descr : Writes XDB file to the hadr disk. The XDB file is created from the
 *         given SDB list (see LDA_OVSSDB_LIST_HEADER *p_fw_info).
 *         Is called by ldah_s7prj_write_backup().
 * Param :
 *  [ IN]:    cp_idx   : board index (1st board == 1) Only one supported yet.
 *  [IN/OUT]: p_fw_info: Pointer to a list of SDBs which are saved to the hard disk.
 *                       Note: Type is LDA_OVSSDB_LIST_HEADER*
 * Return: ==0: PNIO_OK,  != 0 PNIO_ERR_xxx
 */
DPR_UINT32 ldah_write_xdb_backup(DPR_UINT32 cp_idx, LDA_OVSSDB_LIST_HEADER *p_fw_info)
{
  DPR_UINT32 xdb_len = 0, fct_ret = PNIO_OK;

  LDA_XDB_HEADER *p_xdb_hdr;
  LDA_XDB_CON_HEADER con_hdr;
  LDA_XDB_CON_SDB_DIR_HEADER con_sdb_dir_hdr;
  LDA_XDB_CON_SDB_DIR_ITEM con_sdb_dir_item;
  /*LDA_XDB_SDB_HEADER xdb_hdr;*/

  LDA_OVS_BST *p_ovs_sdb = 0;

  DPR_CHAR * p_xdb = 0;
  DPR_VOID * p_tmp = 0;

  memset(&con_sdb_dir_hdr, 0, sizeof(con_sdb_dir_hdr));
  memset(&con_sdb_dir_item, 0, sizeof(con_sdb_dir_item));

  /* create xdb header */
  p_xdb = (DPR_CHAR*)DPR_VMALLOC(sizeof(LDA_XDB_HEADER));
  if(!p_xdb) {
    TRM_OUT01(&ti, GR_MGT, LV_ERR, "can't allocate p_xdb with len=%d",
               sizeof(LDA_XDB_HEADER));
    return PNIO_ERR_OS_RES;
  }

  /* initialize xdb header */
  p_xdb_hdr = (LDA_XDB_HEADER *)p_xdb;
  memset(p_xdb_hdr, 0, sizeof(*p_xdb_hdr));

  p_xdb_hdr->HdrType = LDA_XDB_HEADER_TYPE;
  p_xdb_hdr->HdrId   = LDA_XDB_HEADER_ID;
  p_xdb_hdr->HdrLen = lda_nat_2_le16(sizeof(*p_xdb_hdr));
  p_xdb_hdr->HdrContainerNumber = lda_nat_2_le16(p_fw_info->SDBDirCnt);

  xdb_len = sizeof(*p_xdb_hdr);

  for(DPR_UINT32 i=0; i < p_fw_info->SDBDirCnt; i++) {

    p_ovs_sdb = (LDA_OVS_BST *)(p_fw_info->SDBDatas + p_fw_info->SDBDir[i].OffsetOfSdbBody);

    /* create container header */
    con_hdr.ConId = LDA_XDB_CONTAINER_ID;
    con_hdr.ConType = LDA_XDB_CONTAINER_TYPE;
    con_hdr.ConLen = lda_nat_2_le32(sizeof(LDA_XDB_CON_HEADER) +
                                    sizeof(LDA_XDB_CON_SDB_DIR_HEADER) +
                                    sizeof(LDA_XDB_CON_SDB_DIR_ITEM) + p_fw_info->SDBDir[i].SDBLen);
    con_hdr.ConVmdid = vmdid_from_ovsid(p_fw_info->SDBDir[i].VmdID);

    /* create container directory header */
    con_sdb_dir_hdr.SdbDirLen = lda_nat_2_le32(sizeof(LDA_XDB_CON_SDB_DIR_HEADER) + sizeof(LDA_XDB_CON_SDB_DIR_ITEM));
    con_sdb_dir_hdr.SdbDirVersion = lda_nat_2_le16(0);
    con_sdb_dir_hdr.SdbDirItemsNumber = lda_nat_2_le32(1);
    /* create container directory item */
    con_sdb_dir_item.SdbNumber = lda_nat_2_le16(p_fw_info->SDBDir[i].SDBNumber);
    con_sdb_dir_item.SdbContent = lda_nat_2_le16(p_fw_info->SDBDir[i].SDBContent);
    con_sdb_dir_item.SdbOffset = lda_nat_2_le32(sizeof(LDA_XDB_CON_HEADER) +
                                                 sizeof(LDA_XDB_CON_SDB_DIR_HEADER) +
                                                  sizeof(LDA_XDB_CON_SDB_DIR_ITEM));
    con_sdb_dir_item.SdbLen = lda_nat_2_le32(p_fw_info->SDBDir[i].SDBLen);


    p_tmp = DPR_VREALLOC(p_xdb, xdb_len + lda_le32_2_nat(con_hdr.ConLen));
    if(!p_tmp) {
      TRM_OUT01(&ti, GR_MGT, LV_ERR, "can't allocate p_xdb with len=%d",
                xdb_len + lda_le32_2_nat(con_hdr.ConLen));
      return PNIO_ERR_OS_RES;
    } else {

      p_xdb = (DPR_CHAR*)p_tmp;

      /* copy container header */
      memcpy(p_xdb + xdb_len, &con_hdr, sizeof(con_hdr));

      /* copy container directory header */
      memcpy(p_xdb + xdb_len + sizeof(con_hdr), &con_sdb_dir_hdr, sizeof(con_sdb_dir_hdr));

      /* copy container directory item */
      memcpy(p_xdb + xdb_len + sizeof(con_hdr) + sizeof(con_sdb_dir_hdr), &con_sdb_dir_item, sizeof(con_sdb_dir_item));

      /* copy sdb header and data */
      memcpy(p_xdb + xdb_len + sizeof(con_hdr) + sizeof(con_sdb_dir_hdr) + sizeof(con_sdb_dir_item),
              p_ovs_sdb, lda_be32_2_nat(p_ovs_sdb->head.blk_len));
      TRM_OUT03(&ti, GR_INIT, LV_INFO, "SDB-Header and Data len=%d timestamp=%08x%04x",
                 lda_be32_2_nat(p_ovs_sdb->head.blk_len), *(DPR_UINT32*)&p_ovs_sdb->head.time_stamp1[0], *(DPR_UINT16*)&p_ovs_sdb->head.time_stamp1[4]);
      TRM_OUTD(&ti, GR_INIT, LV_INFO, (unsigned char*)p_ovs_sdb, lda_be32_2_nat(p_ovs_sdb->head.blk_len));

      xdb_len += lda_le32_2_nat(con_hdr.ConLen);

      TRM_OUT04(&ti, GR_INIT, LV_INFO, "ldah_write_xdb_backup cont_len=%d sdb_len=%d sdb_nr=%d xdb_len=%d",
        lda_le32_2_nat(con_hdr.ConLen), lda_be32_2_nat(p_ovs_sdb->head.blk_len), lda_be16_2_nat(p_ovs_sdb->head.blk_nr), xdb_len);
    }
  }

  unsigned long ret;
  int old_process_mode_mask, fw_fd = -1;
  char xdb_filename[512];

  strcpy(xdb_filename, DPR_DRV_CONFIG_PATH());
  snprintf(xdb_filename + strlen(xdb_filename),
           sizeof(xdb_filename) - strlen(xdb_filename),
           LDAH_BACKUP_S7PRJ_FILE, (unsigned long)cp_idx);

  /* sets the calling process's file mode creation mask (umask) to mask & 0777 */
  old_process_mode_mask = umask(0); /* umask() modified global-process umask */
                                    /* to create file which can be processed from each user */
  fw_fd = open(xdb_filename, O_RDWR | O_BINARY | O_CREAT | O_TRUNC, UMASK);

  /* restore process's file mode creation mask */
  (void)umask(old_process_mode_mask);

  if(fw_fd == -1) {
    TRM_OUT01(&ti, GR_INIT, LV_ERR, "Cannot open backup file <%s>", xdb_filename);
    fct_ret = PNIO_ERR_INTERNAL;
    goto ldah_write_xdb_backup_end;
  }

  /* save xdb-file */
  ret = write(fw_fd, p_xdb, xdb_len);

  if(ret < xdb_len) {
    TRM_OUT02(&ti, GR_INIT, LV_ERR, "cannot write the xdb, ret = %d (expected %d)", ret, xdb_len);
    fct_ret = FWLDA_RET_INTERNAL;
  }

  ldah_write_xdb_backup_end:

  if(fw_fd != -1)(void)close(fw_fd);
  if(p_xdb) DPR_VFREE(p_xdb);

  return fct_ret;
} /* end of ldah_write_xdb_backup */


/*-----------------------------------------------------------------------------
 * Name  : ldah_s7prj_write_backup
 * Descr : Reads sdbs from the cp16xx (XL-SDB Upload) and creates backup xdb file.
 *         Fix: Reading segments from the firmware was not coordinated well.
 *         ldah_send_receive_rqb_sync() function is required for upload!!!
 *         And also the neu "synchronous channel" created by ldah_init_mgt_ch()
 *         is rquired for SDB upload.
 * Param :
 *  [ IN]: cp_idx: board index
 *         FWLDA_GET_FW_INFO_DAT *p_fw_info:
 * Return: ==0: PNIO_OK, != 0 PNIO_ERR_xxx
 */
DPR_UINT32 ldah_s7prj_write_backup(DPR_UINT32 cp_idx, unsigned long user_id, FWLDA_GET_FW_INFO_DAT *p_fw_info)
{
  DPR_UINT32 fct_ret, sdb_data_len;
  CP_FWLDA channel;
  FWLDA_SDB_DAT *p_fw_info_sdbs = (FWLDA_SDB_DAT*)((char*)p_fw_info + sizeof(*p_fw_info));

  FWLDA_DPR_RQB *pTmpRespRqb;
  FWLDA_SDB_DAT get_sdb_rqb;
  LDA_OVSSDB_LIST_HEADER *p_ovssdb_list = 0;
  FWLDA_OPCODE ldaf_opcode;  // new for support of large sdbs
  int large_sdb_supported=0;

  memset(&get_sdb_rqb, 0, sizeof(get_sdb_rqb));

  memset(&channel, 0, sizeof(channel));
  channel.cp_index = cp_idx;
  channel.user_id = user_id;

  if(p_fw_info->cp_state != FWLDA_CP_STATE_RUN){
    /* in this case cp configuration data is not consistent
       or remote data download is running,
       It makes little meaning, create new configuration backup */
    TRM_OUT01(&ti, GR_INIT, LV_INFO, "ldah_s7prj_write_backup: cp_state(%x) != FWLDA_CP_STATE_RUN ...", p_fw_info->cp_state);
    TRM_OUT00(&ti, GR_INIT, LV_INFO, "                         cp configuration data is not consistent or will be updated, can't create config backup");
    return PNIO_ERR_CONFIG_IN_UPDATE;
  }

  fct_ret = ldah_init_ovssdb_list(&p_ovssdb_list);
  if(fct_ret != PNIO_OK) {
    TRM_OUT01(&ti, GR_INIT, LV_ERR, "ldah_init_ovssdb_list() failed, ret=0x%x", fct_ret);
    return fct_ret;
  }

  /*  use new synchronous receive calls */
  //fct_ret = ldah_init_mgt_communication(&channel);
  fct_ret = ldah_init_mgt_ch(&channel);

  if(fct_ret != 0) {
    TRM_OUT02(&ti, GR_INIT, LV_ERR, "ldah_init_mgt_ch(cp_idx=%u) failed, ret=0x%x", cp_idx, fct_ret);
    (void)ldah_free_ovssdb_list(&p_ovssdb_list);
    return fct_ret;
  }

  // Changes to receive x-large WEB SDBs segmented
  DPR_CHAR* pRcvBuff = NULL;    // ptr to received data ==SDB, is set in the ldah_send_receive_rqb()

  for(DPR_UINT32 i = 0; i<p_fw_info->sdb_cnt; i++) {
    /* get SDBs from Firmware */

    get_sdb_rqb.context = p_fw_info_sdbs[i].context;
    get_sdb_rqb.nr = p_fw_info_sdbs[i].nr;
    get_sdb_rqb.ovs_inst = p_fw_info_sdbs[i].ovs_inst;

    TRM_OUT02(&ti, GR_MGT, LV_INFO, "get sdb from fw sdb_nr=%d sdb_context=0x%x", get_sdb_rqb.nr, get_sdb_rqb.context);

    // Changes to receive x-large WEB SDBs segmented
    // pointer to a ponter to received data. Buffer is allocated dynamicly in the ldah_send_receive_rqb()
    // but only if fct_ret == PNIO_OK. Buffer must be freed here
    //
	// check FW Version because FW < 2.5.1 does not support large sdbs for miniweb webserver
	if (  (p_fw_info->fw_version.t1  >= 2) && ( p_fw_info->fw_version.t2 >= 5) && (p_fw_info->fw_version.t2 > 0) ) // bigger than 2.5.0
		large_sdb_supported = 1;
	else
		large_sdb_supported = 0;
    TRM_OUT01(&ti, GR_INIT, LV_INFO, "large_sdb_supported = %d", large_sdb_supported);
	//  FW < 2.5.1 FWLDA_OP_GET_SDB_RQB  FW >= 2.5.1 FWLDA_OP_GET_SDB_XL_RQB
	if ( large_sdb_supported) {
		ldaf_opcode = FWLDA_OP_GET_SDB_XL_RQB;
	}
	else
	{
		ldaf_opcode = FWLDA_OP_GET_SDB_RQB ;
	}

    // bugfix: segmented sdb upload requires synchronous receive
    fct_ret = ldah_send_receive_rqb_sync(&channel,2000 , ldaf_opcode /* FWLDA_OP_GET_SDB_XL_RQB */ , p_fw_info_sdbs[i].ovs_inst,
                                           sizeof(get_sdb_rqb), (DPR_CHAR *)&get_sdb_rqb, &pRcvBuff);    if(fct_ret != PNIO_OK) {
      TRM_OUT01(&ti, GR_MGT, LV_ERR, "ldah_send_receive_rqb(FWLDA_OP_GET_SDB_RQB) failed ret 0x%x", fct_ret);
      fct_ret = PNIO_ERR_INTERNAL;
      goto ldah_s7prj_write_backup_end;
    }

    /* interpret datas from firmware */
    //pTmpRespRqb = (FWLDA_DPR_RQB*)channel.user_pool_ptr;    // Changes to receive x-large WEB SDBs segmented
    if (large_sdb_supported)
		pTmpRespRqb = (FWLDA_DPR_RQB*)pRcvBuff;
	else
		pTmpRespRqb = (FWLDA_DPR_RQB*)channel.user_pool_ptr;

    sdb_data_len = LE_TO_CPU(pTmpRespRqb->fwlda_rqb.rqb_len) - sizeof(pTmpRespRqb->fwlda_rqb);

    TRM_OUT03(&ti, GR_MGT, LV_INFO, "add sdb to ovssdb_list sdb_nr=%d sdb_context=0x%x sdb_len=%d",
              get_sdb_rqb.nr, get_sdb_rqb.context, sdb_data_len);

    fct_ret = ldah_add_sdb_to_ovssdb_list(p_ovssdb_list, get_sdb_rqb.ovs_inst, (DPR_CHAR*)pTmpRespRqb + sizeof(FWLDA_DPR_RQB));
    if(fct_ret != PNIO_OK) {
      TRM_OUT01(&ti, GR_MGT, LV_ERR, "ldah_add_sdb_to_ovssdb_list() failed ret 0x%x", fct_ret);
      fct_ret = PNIO_ERR_INTERNAL;
      goto ldah_s7prj_write_backup_end;
    }

    // xl-sdbs changes important !!!
    if (pRcvBuff) {
        DPR_VFREE(pRcvBuff);
    }

  } // end for (..sdb_cnt..)

  fct_ret = ldah_write_xdb_backup(cp_idx, p_ovssdb_list);

  if(fct_ret != PNIO_OK) {
    TRM_OUT01(&ti, GR_MGT, LV_ERR, "ldah_write_xdb_backup() failed ret 0x%x", fct_ret);
    fct_ret = PNIO_ERR_INTERNAL;
    goto ldah_s7prj_write_backup_end;
  }

  /* obsolet
  fct_ret = ldah_write_sdbs_timestamp_backup(cp_idx, p_fw_info);

  if(fct_ret != PNIO_OK) {
    TRM_OUT01(&ti, GR_MGT, LV_ERR, "ldah_write_xdb_backup() failed ret 0x%x", fct_ret);
    fct_ret = PNIO_ERR_INTERNAL;
    goto ldah_s7prj_write_backup_end;
  }*/

  ldah_s7prj_write_backup_end:

  (void)ldah_free_ovssdb_list(&p_ovssdb_list);

  ldah_deinit_mgt_ch(&channel);

  return fct_ret;
} /* end of ldah_s7prj_write_backup */

/* download backup xdb in cp */
DPR_UINT32 ldah_s7prj_restore(DPR_UINT32 cp_idx, DPR_DRV_HANDLE con_fd, unsigned long user_id)
{
  DPR_UINT32 ret, fct_ret = FWLDA_RET_OK;
  char s7prj_backup_filename[512];
  int fw_fd = 0;
  struct stat fw_stat;
  DPR_CHAR *p_s7prj_data = 0;

  strcpy(s7prj_backup_filename, DPR_DRV_CONFIG_PATH());
  snprintf(s7prj_backup_filename + strlen(s7prj_backup_filename),
           sizeof(s7prj_backup_filename) - strlen(s7prj_backup_filename),
           LDAH_BACKUP_S7PRJ_FILE, (unsigned long)cp_idx);

  fw_fd = open(s7prj_backup_filename, O_RDONLY | O_BINARY, UMASK);
  if(fw_fd == -1) {
    TRM_OUT01(&ti, GR_INIT, LV_INFO, "Cannot open backup file <%s>", s7prj_backup_filename);
    fct_ret = FWLDA_RET_FILE_NOT_EXIST;
    goto ldah_s7prj_restore_end;
  }

  /* get file size */
  if(fstat(fw_fd, &fw_stat) == -1) {
    TRM_OUT01(&ti, GR_INIT, LV_ERR, "fstat failed, errno-txt %s", strerror(errno));
    fct_ret = FWLDA_RET_INTERNAL;
    goto ldah_s7prj_restore_end;
  }

  p_s7prj_data = (DPR_CHAR*)DPR_VMALLOC(fw_stat.st_size);
  if(p_s7prj_data == 0) {
    TRM_OUT01(&ti, GR_INIT, LV_ERR, "can't allocate %d bytes for fw-image", fw_stat.st_size);
    fct_ret = FWLDA_RET_INTERNAL;
    goto ldah_s7prj_restore_end;
  }

  if(read(fw_fd, p_s7prj_data, fw_stat.st_size) <= 0 ) {
    TRM_OUT01(&ti, GR_INIT, LV_ERR, "read failed, errno-txt %s", strerror(errno));
    fct_ret = FWLDA_RET_INTERNAL;
    goto ldah_s7prj_restore_end;
  }

  /* download s7prj */
  ret = ldah_download_config_intern(cp_idx, con_fd, user_id, p_s7prj_data, fw_stat.st_size);

  if(ret != PNIO_OK) {
    TRM_OUT02(&ti, GR_MGT, LV_ERR, "ldah_s7prj_restore, ldah_download_config_intern cp_idx=%d ret=0x%x",
              cp_idx, ret);
    fct_ret = FWLDA_RET_INTERNAL;
    goto ldah_s7prj_restore_end;
  }

  ldah_s7prj_restore_end:

  if(p_s7prj_data) DPR_VFREE(p_s7prj_data);
  if(fw_fd != -1) (void)close(fw_fd);

  return fct_ret;
}

/* returned 0 if prj1 is equal to prj2  */
/*            otherwise prj1 is unequal to prj2 */
DPR_UINT32 ldah_s7prj_info_compare(DPR_UINT32 prj1_sdb_cnt, FWLDA_SDB_DAT * prj1_sdbs,
                                   DPR_UINT32 prj2_sdb_cnt, FWLDA_SDB_DAT * prj2_sdbs)
{
  FWLDA_SDB_DAT *p;

  if(prj1_sdb_cnt != prj2_sdb_cnt) {
    TRM_OUT02(&ti, GR_INIT, LV_INFO, "ldah_s7prj_info_compare; cnt1(%d) != cnt2(%d)", prj1_sdb_cnt, prj2_sdb_cnt);
    return 1;
  }

  /*p = prj1_sdbs;
  for(DPR_UINT32 j=0; j<prj1_sdb_cnt;j++){
    TRM_OUT04(&ti, GR_INIT, LV_ERR, "prj1.sdb_dat ovs_ist=%d nr=%d context=x%x len=%d", p[j].ovs_inst, p[j].nr, p[j].context, p[j].sdb_length);
  }*/

  qsort(prj1_sdbs, prj1_sdb_cnt, sizeof(FWLDA_SDB_DAT), ldah_compare_fwlda_sdb_dat_items);

  /*p = prj1_sdbs;
  for(DPR_UINT32 j=0; j<prj1_sdb_cnt;j++){
    TRM_OUT04(&ti, GR_INIT, LV_ERR, "_prj1.sdb_dat ovs_ist=%d nr=%d context=x%x len=%d", p[j].ovs_inst, p[j].nr, p[j].context, p[j].sdb_length);
  }*/


  qsort(prj2_sdbs, prj1_sdb_cnt, sizeof(FWLDA_SDB_DAT), ldah_compare_fwlda_sdb_dat_items);

  for(DPR_UINT32 i=0; i<prj1_sdb_cnt; i++) {
    int i_cmp_prop, i_cmp_timestamp;

    i_cmp_prop = ldah_compare_fwlda_sdb_dat_items(&prj1_sdbs[i], &prj2_sdbs[i]);

    if(i_cmp_prop == 0){
      i_cmp_timestamp = memcmp(prj1_sdbs[i].timestamp, prj2_sdbs[i].timestamp, sizeof(prj1_sdbs[0].timestamp));

      if(i_cmp_timestamp != 0){
        TRM_OUT05(&ti, GR_INIT, LV_INFO, "sdb_idx=%d timestamps (%04x%02x/%04x%02x) not equal", i,
                  *(DPR_UINT32*)prj1_sdbs[i].timestamp, *(DPR_UINT32*)&prj1_sdbs[i].timestamp[4],
                   *(DPR_UINT32*)prj2_sdbs[i].timestamp, *(DPR_UINT32*)&prj2_sdbs[i].timestamp[4]);
      }
    }

    if(i_cmp_prop != 0 || i_cmp_timestamp != 0) {

      p = prj1_sdbs;
      for(DPR_UINT32 j=0; j<prj1_sdb_cnt;j++) {
        TRM_OUT05(&ti, GR_INIT, LV_INFO, "prj1.sdb_dat ovs_ist=%d nr=%d context=x%x len=%d ts[4]=%08x",
          p[j].ovs_inst, p[j].nr, p[j].context, p[j].sdb_length, *(DPR_UINT32*)p[j].timestamp);
      }
      p = prj2_sdbs;
      for(DPR_UINT32 j=0; j<prj2_sdb_cnt;j++) {
        TRM_OUT05(&ti, GR_INIT, LV_INFO, "prj2.sdb_dat ovs_ist=%d nr=%d context=x%x len=%d ts[4]=%08x",
          p[j].ovs_inst, p[j].nr, p[j].context, p[j].sdb_length, *(DPR_UINT32*)p[j].timestamp);
      }
      return 1;
    }
  }

  return 0;
}

/*-----------------------------------------------------------------------------
 * Name  : ldah_get_fw_info
 * Descr : Controller function ONLY! Obsolete, use SERV_CP_get_fw_info() instead.
 * Return: ==0 ==PNIO_OK: ok, != 0 error
 */
DPR_UINT32 ldah_get_fw_info(DPR_UINT32 cp_idx, unsigned long user_id, FWLDA_GET_FW_INFO_DAT** p_fw_info)
{
  DPR_UINT32 fct_ret;
  CP_FWLDA channel;

  FWLDA_DPR_RQB *pTmpRespRqb;
  DPR_UINT32 fw_info_len;
  FWLDA_GET_FW_INFO_DAT *pTmpInfoDat;

  memset(&channel, 0, sizeof(channel));
  channel.cp_index = cp_idx;
  channel.user_id = user_id;

  fct_ret = ldah_init_mgt_communication(&channel);
  if(fct_ret != 0) {
    TRM_OUT02(&ti, GR_INIT, LV_ERR, "ldah_init_mgt_communication(cp_idx=%u) failed, ret=0x%x", cp_idx, fct_ret);
    return fct_ret;
  }

  /* second sequence, only for test
  ldah_deinit_mgt_communication(&channel);

  fct_ret = ldah_init_mgt_communication(&channel);
  if(fct_ret != 0){
    TRM_OUT01(&ti, GR_INIT, LV_ERR, "ldah_init_mgt_communication(cp_idx=%u) failed, ret=0x%x", fct_ret);
    return fct_ret;
  }*/

  /* save configuration for Stationsmanager in ROM */
  fct_ret = ldah_send_receive_rqb(&channel, 2000, FWLDA_OP_GET_FW_INFO_RQB, 0, 0, 0);
  if(fct_ret != PNIO_OK) {
    TRM_OUT01(&ti, GR_MGT, LV_ERR, "ldah_send_receive_rqb(FWLDA_OP_GET_FW_INFO_RQB) failed ret 0x%x", fct_ret);
    goto ldah_get_fw_info_end;
  }

  /* interpret datas from firmware */
  pTmpRespRqb = (FWLDA_DPR_RQB*)channel.user_pool_ptr;
  fw_info_len = LE_TO_CPU(pTmpRespRqb->fwlda_rqb.rqb_len) - sizeof(pTmpRespRqb->fwlda_rqb);

  TRM_OUT01(&ti, GR_MGT, LV_INFO, "ldah_get_fw_info, fw_info_len=%d", fw_info_len);

  /* consystency check vor FWLDA_GET_FW_INFO_DAT data */
  pTmpInfoDat = (FWLDA_GET_FW_INFO_DAT*)((DPR_CHAR*)pTmpRespRqb + sizeof(*pTmpRespRqb));
  if(pTmpInfoDat->blk_len != sizeof(*pTmpInfoDat) + pTmpInfoDat->sdb_cnt * sizeof(FWLDA_SDB_DAT)){
    TRM_OUT02(&ti, GR_MGT, LV_ERR, "ldah_get_fw_info:: not consistent FWLDA_GET_FW_INFO_DAT (must be %d, is %d)",
               pTmpInfoDat->blk_len, sizeof(*pTmpInfoDat) + pTmpInfoDat->sdb_cnt * sizeof(FWLDA_SDB_DAT));
    fct_ret = PNIO_ERR_INTERNAL;
    goto ldah_get_fw_info_end;
  }

  *p_fw_info = (FWLDA_GET_FW_INFO_DAT *)DPR_VMALLOC(fw_info_len);
  if(!p_fw_info) {
    TRM_OUT01(&ti, GR_MGT, LV_ERR, "ldah_get_fw_info: can't allocate %d bytes", fw_info_len);
    fct_ret = PNIO_ERR_OS_RES;
    goto ldah_get_fw_info_end;
  }

  memcpy(*p_fw_info, pTmpInfoDat, fw_info_len);

  ldah_get_fw_info_end:

  ldah_deinit_mgt_communication(&channel);

  return fct_ret;
} /* end of ldah_get_fw_info */

/*-----------------------------------------------------------------------------
 * Name  : ldah_check   (local download agent host)
 * Descr : Main function to handle bord replacement
 * Param : [ IN]: cp index
 * Return: ==0: == PNIO_OK ok, != 0 error
 */
extern "C"
DPR_UINT32 PNIO_CODE_ATTR ldah_check(DPR_UINT32 cp_idx)
{
  DPR_UINT32 ret;
  char backup_serial[FWLDA_SER_NUMBER_LEN];
  LDA_FW_VERS backup_fw_version;
  int large_sdb_supported=0;

  struct t_register_app app;
  DPR_DRV_HANDLE control_fd = 0;
  DPR_CHAR * mapStart = 0;
  FWLDA_SDB_DAT *pSdbInfo = 0;
  DPR_CHAR fw_info_from_cp_is_available = 0, fw_backup_is_available = 0, fw_was_seted_to_default = 0;

  FWLDA_GET_FW_INFO_DAT *p_fw_info = 0;
  DPR_UINT32 backup_s7prj_sdb_cnt = 256; /* maximum expected sdb count */
  FWLDA_SDB_DAT backup_s7prj_sdbs[256];

  memset(backup_serial, 0, sizeof(backup_serial));
  memset(&backup_fw_version, 0, sizeof(backup_fw_version));

  DPR_MUTEX_LOCK(ServMutexDownload);

  ret = ldah_open(cp_idx, &app, &mapStart, &control_fd);
  if(ret != 0) {
    TRM_OUT02(&ti, GR_INIT, LV_ERR, "ldah_open(cp_idx=%u) failed, ret=0x%x", cp_idx, ret);
    goto ldah_check_close;
  }

#if defined (LDAH_TEST)
  DPR_UINT32 tst = get_sdb_test ( cp_idx, app.user_id );
  ldah_close(control_fd, &app, mapStart);
  DPR_MUTEX_UNLOCK(ServMutexDownload);
  return tst;
#endif
  /* get hw ident, FW version and s7 configuration information from cp firmware */
  ret = ldah_get_fw_info(cp_idx, app.user_id, &p_fw_info);
  if(ret == PNIO_ERR_NO_FW_COMMUNICATION) {
    TRM_OUT00(&ti, GR_INIT, LV_ERR, "ldah_get_fw_info ret NO_FW_COMMUNICATION, can't work now any more");
    goto ldah_check_close;
  }

  if(ret != PNIO_OK){

    TRM_OUT00(&ti, GR_INIT, LV_ERR, "FW_INFO not available, possible old firmware version, stop check can't work now any more");
    TRM_OUT00(&ti, GR_INIT, LV_ERR, "Feature CP_EXCHANGE_WITHOUT_PG is as of now not available");
    /* if old firmware version < 2.2 loaded, fw can't answers GET_FW_INFO request */
    /* is a fix, if you comment next goto command, the loading of FW < 2.2 will be unpossible, because this check code
       will overwrite old firmware with firmware backup, if firmware backup available */
    goto ldah_check_close;

    /* TRM_OUT00(&ti, GR_INIT, LV_ERR, "FW_INFO not available, possible old firmware version, load backup in CP if available");*/
    /* if backup firmware availble, load fw backup in CP */
    /* create psedo FWLDA_GET_FW_INFO_DAT */
    p_fw_info = (FWLDA_GET_FW_INFO_DAT *)DPR_VMALLOC(sizeof(*p_fw_info));
    if(!p_fw_info){
      TRM_OUT01(&ti, GR_INIT, LV_ERR, "can't allocate psedo p_fw_info len %d", sizeof(*p_fw_info));
      goto ldah_check_close;
   }

   memset(p_fw_info, 0, sizeof(*p_fw_info));
   p_fw_info->blk_len = sizeof(*p_fw_info);
   memcpy(p_fw_info->cp_ser_nr, "NOT_AVAILABLE___", sizeof(p_fw_info->cp_ser_nr));
   memset(&p_fw_info->fw_version, 0xF, sizeof(p_fw_info->fw_version));
  }
  else {
    /* hw ident, FW version and s7 configuration information from cp firmware available, do comparation */

    char stmp[32];

    fw_info_from_cp_is_available = 1;

    TRM_OUT00(&ti, GR_INIT, LV_INFO, "FW_INFO from CP is available");
    TRM_OUT01(&ti, GR_INIT, LV_INFO, "FW_INFO.ext_par = <0x%02x>", p_fw_info->ext_par);
    TRM_OUT01(&ti, GR_INIT, LV_INFO, "FW_INFO.serial = <%16s>", p_fw_info->cp_ser_nr);
    TRM_OUT05(&ti, GR_INIT, LV_INFO, "FW_INFO.fw_version = %d.%d.%d.%d.%d", p_fw_info->fw_version.t1, p_fw_info->fw_version.t2,
               p_fw_info->fw_version.t3, p_fw_info->fw_version.t4, p_fw_info->fw_version.t5);
    TRM_OUT01(&ti, GR_INIT, LV_INFO, "FW_INFO.hw_version = %d", p_fw_info->hw_version);
    TRM_OUT01(&ti, GR_INIT, LV_INFO, "FW_INFO.cp_state = %d", p_fw_info->cp_state);
    // check FW version to decide wether small or big sdbs are supported
	if (  (p_fw_info->fw_version.t1  >= 2) && ( p_fw_info->fw_version.t2 >= 5) && (p_fw_info->fw_version.t2 > 0) ) // bigger than 2.5.0
		large_sdb_supported = 1;
	else
		large_sdb_supported = 0;
    TRM_OUT01(&ti, GR_INIT, LV_INFO, "large_sdb_supported = %d", large_sdb_supported);

    memset(stmp, 0, sizeof(stmp)); /* preparation for no EOS string p_fw_info->mlfb */
    memcpy(stmp, (char*)p_fw_info->mlfb, sizeof(p_fw_info->mlfb));

    TRM_OUT01(&ti, GR_INIT, LV_INFO, "FW_INFO.mlfb = %s", stmp);

    snprintf(stmp, sizeof(stmp)-1, "%02X-%02X-%02X-%02X-%02X-%02X", p_fw_info->mac[0], p_fw_info->mac[1], p_fw_info->mac[2],
                                                                    p_fw_info->mac[3], p_fw_info->mac[4], p_fw_info->mac[5]);
    TRM_OUT01(&ti, GR_INIT, LV_INFO, "FW_INFO.mac = %s", stmp);
    snprintf(stmp, sizeof(stmp)-1, "%d.%d.%d.%d",
                p_fw_info->pnio_ip[0], p_fw_info->pnio_ip[1], p_fw_info->pnio_ip[2], p_fw_info->pnio_ip[3]);
    TRM_OUT01(&ti, GR_INIT, LV_INFO, "FW_INFO.pnio_ip = %s", stmp);
    snprintf(stmp, sizeof(stmp)-1, "%d.%d.%d.%d",
                p_fw_info->pnio_net_mask[0], p_fw_info->pnio_net_mask[1], p_fw_info->pnio_net_mask[2], p_fw_info->pnio_net_mask[3]);
    TRM_OUT01(&ti, GR_INIT, LV_INFO, "FW_INFO.pnio_net_mask = %s", stmp);
    snprintf(stmp, sizeof(stmp)-1, "%d.%d.%d.%d",
                p_fw_info->pnio_gw_ip[0], p_fw_info->pnio_gw_ip[1], p_fw_info->pnio_gw_ip[2], p_fw_info->pnio_gw_ip[3]);
    TRM_OUT01(&ti, GR_INIT, LV_INFO, "FW_INFO.pnio_gw_ip = %s", stmp);

    pSdbInfo = (FWLDA_SDB_DAT*)((DPR_CHAR*)p_fw_info + sizeof(*p_fw_info));
    for(DPR_UINT32 i=0; i<p_fw_info->sdb_cnt; i++) {
      TRM_OUT04(&ti, GR_INIT, LV_INFO, "FW_INFO.sdb len=%d nr=%d context=0x%x ovs_inst=%d",
                pSdbInfo[i].sdb_length, pSdbInfo[i].nr, pSdbInfo[i].context, pSdbInfo[i].ovs_inst);
    }

    if(p_fw_info->ext_par & FWLDA_EXTPAR_NO_CP_EXCHANGE)
    {
      TRM_OUT00(&ti, GR_INIT, LV_INFO, "FW from CP don't permit CP_EXCHANGE_WITHOUT_PG feature (FWLDA_EXTPAR_NO_CP_EXCHANGE)");
      goto ldah_check_close;
    }
  }

  ret = ldah_serial_nr_read_backup(cp_idx, backup_serial, FWLDA_SER_NUMBER_LEN);
  if(ret != PNIO_OK && fw_info_from_cp_is_available ) {
    TRM_OUT00(&ti, GR_INIT, LV_INFO, "ldah_check no backup serial number available, create backup");

    ret = ldah_fw_image_write_backup(cp_idx, control_fd, app.user_id, &p_fw_info->fw_version);
    TRM_OUT01(&ti, GR_INIT, LV_INFO, "ldah_write_fw_image_backup ret 0x%x", ret);

    ret = ldah_s7prj_write_backup(cp_idx, app.user_id, p_fw_info);
    TRM_OUT01(&ti, GR_INIT, LV_INFO, "ldah_write_s7prj_backup ret 0x%x", ret);

    ret = ldah_serial_nr_write_backup(cp_idx, (char*)p_fw_info->cp_ser_nr, FWLDA_SER_NUMBER_LEN);
    TRM_OUT01(&ti, GR_INIT, LV_INFO, "ldah_write_serial_nr_backup ret 0x%x", ret);

    TRM_OUT00(&ti, GR_INIT, LV_INFO, "ldah_check cp backup created");
    goto ldah_check_close;
  }

  TRM_OUT01(&ti, GR_INIT, LV_INFO, "backup.serial = <%16s>", backup_serial);

  ret = ldah_fw_image_read_backup_version(cp_idx, &backup_fw_version);
  TRM_OUT01(&ti, GR_INIT, LV_INFO, "ldah_fw_image_read_backup_version ret 0x%x", ret);

  if(ret == FWLDA_RET_OK){
    fw_backup_is_available = 1;
    TRM_OUT05(&ti, GR_INIT, LV_INFO, "backup.fw_image_version = %d.%d.%d.%d.%d", backup_fw_version.t1,
               backup_fw_version.t2, backup_fw_version.t3, backup_fw_version.t4, backup_fw_version.t5);
  }
  else {
    TRM_OUT00(&ti, GR_INIT, LV_INFO, "no fw image backup available");
    TRM_OUT05(&ti, GR_INIT, LV_INFO, "backup.fw_image_version = %d.%d.%d.%d.%d", backup_fw_version.t1,
               backup_fw_version.t2, backup_fw_version.t3, backup_fw_version.t4, backup_fw_version.t5);
  }

  ret = ldah_s7prj_read_backup_info(cp_idx, &backup_s7prj_sdb_cnt, backup_s7prj_sdbs);
  TRM_OUT01(&ti, GR_INIT, LV_INFO, "ldah_s7prj_read_backup_info ret 0x%x", ret);

  /* compare backup_serial_nr and cp_serial_nr */
  if(memcmp(backup_serial, p_fw_info->cp_ser_nr, sizeof(backup_serial)) == 0) {

    TRM_OUT00(&ti, GR_INIT, LV_INFO, "SAME CP detected, compare fw version with version of fw backup and s7prj with s7prj backup");

    if(memcmp(&backup_fw_version, &p_fw_info->fw_version, sizeof(backup_fw_version)) != 0) {

      TRM_OUT00(&ti, GR_INIT, LV_INFO, "backup fw version is not equal to fw version from cp, backup fw image");

      ret = ldah_fw_image_write_backup(cp_idx, control_fd, app.user_id, &p_fw_info->fw_version);
      TRM_OUT01(&ti, GR_INIT, LV_INFO, "ldah_fw_image_write_backup ret 0x%x", ret);
    }
    else {
      TRM_OUT00(&ti, GR_INIT, LV_INFO, "backup and CP firmware version are identical");
    }

    if(ldah_s7prj_info_compare(backup_s7prj_sdb_cnt, backup_s7prj_sdbs, p_fw_info->sdb_cnt,
                               (FWLDA_SDB_DAT*)((DPR_CHAR*)p_fw_info + sizeof(*p_fw_info))) != 0) {

      TRM_OUT00(&ti, GR_INIT, LV_INFO, "backup s7prj is not equal to s7prj from cp, backup s7prj");

      ret = ldah_s7prj_write_backup(cp_idx, app.user_id, p_fw_info);
      TRM_OUT01(&ti, GR_INIT, LV_INFO, "ldah_s7prj_write_backup ret 0x%x", ret);
    }
    else {
        TRM_OUT00(&ti, GR_INIT, LV_INFO, "backup and CP s7prj are identical");
    }
  }
  else {
    TRM_OUT01(&ti, GR_INIT, LV_INFO, "backup.serial  =<%16s>", backup_serial);
    TRM_OUT01(&ti, GR_INIT, LV_INFO, "FW_INFO.serial =<%16s>", p_fw_info->cp_ser_nr);

    TRM_OUT00(&ti, GR_INIT, LV_INFO, "ldah_check backup/cp-serial nr not equal, CP-REPLACED-CASE detected");

    if(!fw_backup_is_available){
      TRM_OUT00(&ti, GR_INIT, LV_INFO, "no fw backup available");
    }

    /* if fw backup available and actual cp fw version is not equal to version of fw backup,
       than load fw backup into cp */
    if(fw_backup_is_available &&
        memcmp(&backup_fw_version, &p_fw_info->fw_version, sizeof(backup_fw_version)) != 0) {

      char fw_image_filename[512];

      fw_was_seted_to_default = 1;

      strcpy(fw_image_filename, DPR_DRV_CONFIG_PATH());
      snprintf(fw_image_filename + strlen(fw_image_filename),
           sizeof(fw_image_filename) - strlen(fw_image_filename),
           LDAH_BACKUP_FW_IMAGE_FILE, (unsigned long)cp_idx);

      ret = ldah_image_restore(cp_idx, mapStart, FW_START_OFFSET, FLASH_FS_OFFSET, fw_image_filename);
	  if ( ret != PNIO_OK ) {
        TRM_OUT01(&ti, GR_INIT, LV_ERR, "ldah_fw_image_restore failed, ret=0x%x", ret);
        goto ldah_check_close;
	  }
      TRM_OUT00(&ti, GR_INIT, LV_INFO, "ldah_fw_image_restore OK ");

      /* The backup firmware was loaded,
             "restart CP, wait till CP go on, after that check is backup s7 configuration muss be load in CP */

      /* unregister an application to the driver, to be able to make reset */

      /* perform complete close */
      ldah_close(control_fd, &app, (void*)mapStart);

      if(ret == PNIO_OK) {
        /* restart CP, wait till CP go on ... */
        ret = ldah_reset_firmware(cp_idx, 0);
        TRM_OUT01(&ti, GR_INIT, LV_ERR, "ldah_reset_firmware, ret=0x%x", ret);
      }

      ret = ldah_open(cp_idx, &app, &mapStart, &control_fd);
      if(ret != 0) {
        TRM_OUT01(&ti, GR_INIT, LV_ERR, "can't connect to cp after firmware download, ldah_open(cp_idx=%u) failed, ret=0x%x", ret);
        goto ldah_check_close;
      }
      else {
        TRM_OUT00(&ti, GR_INIT, LV_INFO, "successfuly connected to cp after firmware download");
      }
    }
    else {
        TRM_OUT00(&ti, GR_INIT, LV_INFO, "backup and CP firmware version are identical");
    }

    if(fw_was_seted_to_default ||
        ldah_s7prj_info_compare(backup_s7prj_sdb_cnt, backup_s7prj_sdbs,
                               p_fw_info->sdb_cnt, (FWLDA_SDB_DAT*)((DPR_CHAR*)p_fw_info + sizeof(*p_fw_info))) != 0) {
      TRM_OUT00(&ti, GR_INIT, LV_INFO, "restore s7prj from backup");
      ret = ldah_s7prj_restore(cp_idx, control_fd, app.user_id);
      TRM_OUT01(&ti, GR_INIT, LV_INFO, "ldah_s7prj_restore ret 0x%x", ret);
    }
    else {
      TRM_OUT00(&ti, GR_INIT, LV_INFO, "backup and CP s7prj are identical");
    }

    ret = ldah_serial_nr_write_backup(cp_idx, (char*)p_fw_info->cp_ser_nr, FWLDA_SER_NUMBER_LEN);
    TRM_OUT01(&ti, GR_INIT, LV_INFO, "ldah_serial_nr_write_backup ret 0x%x", ret);
  }


  ldah_check_close:

  if(p_fw_info) DPR_VFREE(p_fw_info);

  ldah_close(control_fd, &app, mapStart);

  DPR_MUTEX_UNLOCK(ServMutexDownload);

  return ret;
} /* end of ldah_check */


/*-----------------------------------------------------------------------------
 * Name  : ldah_serv_fw_info
 * Descr : Internal function to retireve FW name + address information.
 *         See user ServLib service: SERV_CP_get_fw_info()
 * Param :
 *  [ IN]: cp_idx: Index of the cp16xx board. <1, num-boards>,
 *                 ONLY ONE BOARD SUPPORTED => only 1 is a valid value
 *  [OUT]: SERV_CP_FW_INFO_TYPE. Buffer with fw infos (returned params)
 * Return: PNIO_OK, PNIO_ERR_xxx, e.g.: PNIO_ERR_MAX_REACHED, ...
 */
DPR_UINT32 ldah_serv_fw_info(DPR_UINT32 cp_idx, SERV_CP_FW_INFO_TYPE* p_info)
{
    DPR_UINT32 ret;
    struct t_register_app   app;
    DPR_DRV_HANDLE          control_fd = 0;       // driver handle, file descriptor
    char                    control_fd_path[64];
    FWLDA_DPR_RQB          *pTmpRespRqb;
    DPR_UINT32              fw_info_len = 0;
    DPR_UINT32              fw_info_blk_len = 0;
    FWLDA_CP_FW_INFO_TYPE  *pFwInfoDat;


    TRM_OUT01(&ti, GR_INIT, LV_INFO, ">ldah_serv_fw_info  cp-idx=%d", cp_idx);


    /* open application access point and register current application */
    snprintf(control_fd_path, sizeof(control_fd_path)-1, DPR_CONTROL_INTERFACE, (unsigned int)(cp_idx - 1));

    if(!(control_fd = DPR_DRV_OPEN(control_fd_path))) {
        TRM_OUT01(&ti, GR_INIT, LV_ERR, "<ldah_serv_fw_info: open <%s> failed", control_fd_path);
        return PNIO_ERR_PRM_CP_ID;
    }

    app.flags = OAPP_APP_NORMAL;  //  OAPP_APP_EXCLUSIVE;

    /*  register an application to the driver. */
    if(DPR_DRV_IOCTL(control_fd, CP16XX_IOC_OAPP, &app, sizeof(app), sizeof(app))) {
        TRM_OUT01(&ti, GR_INIT, LV_ERR, "<ldah_serv_fw_info: ioctl register app. failed! ERROR '%s'", strerror(errno));
        DPR_DRV_CLOSE(control_fd);
        control_fd = 0;
        if(EMFILE == errno)
            return PNIO_ERR_MAX_REACHED;
        else
            return PNIO_ERR_CREATE_INSTANCE;
    }

    /* application registered, open mgt communication channel */
    CP_FWLDA channel;
    memset(&channel, 0, sizeof(channel));
    channel.cp_index = cp_idx;
    channel.user_id  = app.user_id;

    ret = ldah_init_mgt_ch(&channel);
    if(ret != 0) {
        TRM_OUT01(&ti, GR_INIT, LV_ERR, "ldah_serv_fw_info: init_mgt_ch ERROR ret=0x%x", ret);
        goto ldah_serv_fwi_end;
    }

    /* send request to the firmware and receive response data */
    ret = ldah_send_receive_rqb_sync(&channel, 2000, FWLDA_OP_CP_GET_FW_INFO_RQB, 0, 0, 0);
    if(ret != PNIO_OK) {
        TRM_OUT01(&ti, GR_INIT, LV_ERR, "ldah_send_receive_rqb() failed ret 0x%x", ret);
        ldah_deinit_mgt_ch(&channel);
        goto ldah_serv_fwi_end;
    }

    /* response ok received, prepare otput struct for the user */
    pTmpRespRqb = (FWLDA_DPR_RQB*)channel.user_pool_ptr;
    fw_info_len = LE_TO_CPU(pTmpRespRqb->fwlda_rqb.rqb_len) - sizeof(pTmpRespRqb->fwlda_rqb);

    pFwInfoDat = (FWLDA_CP_FW_INFO_TYPE*)((DPR_CHAR*)pTmpRespRqb + sizeof(*pTmpRespRqb));

    TRM_OUT03(&ti, GR_INIT, LV_INFO, "ldah_serv_fw_info: fw_info_len expected=%d received=%d blk_len=%d",
              sizeof(FWLDA_CP_FW_INFO_TYPE), fw_info_len, pFwInfoDat->blk_len);

    /* consystency check for resp. data */
    fw_info_blk_len = pFwInfoDat->blk_len;

    /* response ok: fill out the output buffer */

    /* firmware version */
    p_info->fw_version.v1 = pFwInfoDat->fw_version.t1;
    p_info->fw_version.v2 = pFwInfoDat->fw_version.t2;
    p_info->fw_version.v3 = pFwInfoDat->fw_version.t3;
    p_info->fw_version.v4 = pFwInfoDat->fw_version.t4;
    p_info->fw_version.v5 = pFwInfoDat->fw_version.t5;

    /* cp serial number */
    memset(p_info->cp_ser_nr, 0, sizeof(p_info->cp_ser_nr));
    strcpy((char*)p_info->cp_ser_nr, (const char*)pFwInfoDat->cp_ser_nr);

    /* mlfb */
    memset(p_info->mlfb, 0, sizeof(p_info->mlfb));
    strcpy((char*)p_info->mlfb, (const char*)pFwInfoDat->mlfb);

    /* hw version */
    p_info->hw_version = pFwInfoDat->hw_version;

    /* mac address */
    memcpy(p_info->mac, pFwInfoDat->mac, sizeof(pFwInfoDat->mac));

    /* ip suite */
    p_info->ip_addr        = pFwInfoDat->ip_addr;
    p_info->ip_mask        = pFwInfoDat->ip_mask;
    p_info->default_router = pFwInfoDat->default_router;

    /* name of station */
    memset(p_info->name, 0, sizeof(p_info->name));
    strcpy((char*)p_info->name, (const char*)pFwInfoDat->station_name);

    /* type of station */
    memset(p_info->TypeOfStation, 0, sizeof(p_info->TypeOfStation));
    strcpy((char*)p_info->TypeOfStation, (const char*)pFwInfoDat->type_of_station);

ldah_serv_fwi_end:

    ldah_deinit_mgt_ch(&channel);

    /* close application drv */
    if(control_fd != 0){
        DPR_DRV_IOCTL(control_fd, CP16XX_IOC_CAPP, &app, sizeof(app), 0);
        DPR_DRV_CLOSE(control_fd);
    }

    TRM_OUT01(&ti, GR_INIT, LV_INFO, "<ldah_serv_fw_info: DONE: 0==OK: ret=%#x", ret);
    return ret;
} /* end of ldah_serv_fw_info */


//----------------------------------------------------------------------------
//---------------------------------------   Helper ---------------------------

#if defined (LDAH_TEST)
/*-----------------------------------------------------------------------------
 * Name  : get_sdb_test
 * Descr :
 * Return: ==0: ok, != 0 error
 */
DPR_UINT32 get_sdb_test (DPR_UINT32 cp_idx, unsigned long user_id )
{

  DPR_UINT32 fct_ret;
  CP_FWLDA channel;

  FWLDA_DPR_RQB *pTmpRespRqb;
  //DPR_UINT32 fw_info_len;
  //FWLDA_GET_FW_INFO_DAT *pTmpInfoDat;

  memset(&channel, 0, sizeof(channel));
  channel.cp_index = cp_idx;
  channel.user_id = user_id;

  fct_ret = ldah_init_mgt_communication(&channel);
  if(fct_ret != 0) {
    TRM_OUT02(&ti, GR_INIT, LV_ERR, "get_sdb_test: ldah_init_mgt_communication(cp_idx=%u) failed, ret=0x%x", cp_idx, fct_ret);
    return fct_ret;
  }


  DPR_CHAR* pRcvBuff = NULL;    // ptr to received data ==SDB, is set in the ldah_send_receive_rqb()

  /* get rqb  */

  TRM_OUT00(&ti, GR_MGT, LV_INFO, "get_sdb_test: FWLDA_OP_GET_SDB_XL_RQB ");

  // Changes to receive x-large WEB SDBs segmented
  // pointer to a ponter to received data. Buffer is allocated dynamicly in the ldah_send_receive_rqb()
  // but only if fct_ret == PNIO_OK. Buffer must be freed here
  //
  int tout = 60000;
  fct_ret = ldah_send_receive_rqb(&channel, tout, FWLDA_OP_GET_SDB_XL_RQB, 0, 0, 0, &pRcvBuff);
  if(fct_ret != PNIO_OK) {
    TRM_OUT01(&ti, GR_MGT, LV_ERR, "get_sdb_test: ldah_send_receive_rqb(FWLDA_OP_GET_SDB_RQB) failed ret 0x%x", fct_ret);
    fct_ret = PNIO_ERR_INTERNAL;
    goto get_sdb_test_end;
  }

  /* interpret datas from firmware */
  //pTmpRespRqb = (FWLDA_DPR_RQB*)channel.user_pool_ptr;    // Changes to receive x-large WEB SDBs segmented
  pTmpRespRqb = (FWLDA_DPR_RQB*)pRcvBuff;


  //.........

get_sdb_test_end:

  ldah_deinit_mgt_communication(&channel);

  return fct_ret;
} /* end of get_sdb_test */

#endif //(LDAH_TEST)
