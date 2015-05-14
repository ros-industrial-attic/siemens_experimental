/****************************************************************************
*  Copyright (C) SIEMENS CORP., 2013 All rights reserved.
*****************************************************************************
* FILE NAME    : ldah_fwdown.cpp
* ---------------------------------------------------------------------------
* DESCRIPTION  : firmware local download functions
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
#include "pnioerrx.h"
#include "tracemcrm.h"
#include "cp16xx.h"
#include "ldah_flash.h"
#include "lda_spec.h"

#define UMASK      00777               /* use user's umask for creating files */

/* external objects */
extern CTrmWrapper ti;

/* external functions */
extern DPR_UINT32 ldah_reset_firmware(DPR_UINT32 cp_idx, DPR_CHAR type);
#define PRINTOUT printf

void ldah_close(DPR_DRV_HANDLE control_fd, struct t_register_app *app, void *mapStart)
{
  if(control_fd == 0){
    TRM_OUT01(&ti, GR_INIT, LV_WARN, "ldah_close: control_fd=%d, leave function", control_fd);
    return;
  }

  if(mapStart != 0){
	DPR_DRV_MUNMAP(control_fd, mapStart, F_SIZE, app->user_id);
  }
  else{
    TRM_OUT01(&ti, GR_INIT, LV_WARN, "ldah_close: mapStart=%p not valid", mapStart);
  }

  DPR_DRV_IOCTL(control_fd, CP16XX_IOC_CAPP, app, sizeof(*app), 0);

  DPR_DRV_CLOSE(control_fd);
}

/**
 * init - initialization of the program
 *
 * This routine opens the control file, lock it which guarantees only
 * one instance of this program is running, register to the driver,
 * map the flash into user space and check the type of the flash.
 *
 * RETURNS: file descriptor of the control file, or -1 if anything went
 * wrong.
 **/
DPR_UINT32 ldah_open(unsigned long cp_idx, struct t_register_app *app, DPR_CHAR **ppmapStart, DPR_DRV_HANDLE *con_fd)
{
  DPR_CHAR f_type;
  char control_fd_path[64];

  if(con_fd == 0) return PNIO_ERR_INTERNAL;

  *con_fd = 0;

  if(cp_idx < 1) {
    TRM_OUT00(&ti, GR_INIT, LV_ERR, "first cp index is 1");
    return PNIO_ERR_PRM_CP_ID;
  }

  snprintf(control_fd_path, sizeof(control_fd_path)-1, DPR_CONTROL_INTERFACE, (unsigned int)(cp_idx - 1));

  if(!(*con_fd = DPR_DRV_OPEN(control_fd_path))) {
    TRM_OUT01(&ti, GR_INIT, LV_ERR, "open <%s> failed", control_fd_path);
    return PNIO_ERR_PRM_CP_ID;
  }

  app->flags = OAPP_APP_EXCLUSIVE;

  /*  Register an application to the driver. */
  if(DPR_DRV_IOCTL(*con_fd, CP16XX_IOC_OAPP, app, sizeof(*app), sizeof(*app))) {
    TRM_OUT01(&ti, GR_INIT, LV_ERR, "ioctl register application failed, error '%s'",
              strerror(errno));
    /*flock(control_fd, LOCK_UN);*/
    DPR_DRV_CLOSE(*con_fd);
	  *con_fd = 0;

    if(EMFILE == errno)
      return PNIO_ERR_MAX_REACHED;
    else
      return PNIO_ERR_CREATE_INSTANCE;
  }

  /*  Map the whole flash into user space. */
  *ppmapStart = (DPR_CHAR *)DPR_DRV_MMAP(*con_fd, MMAP_OFFSET_EMIF_CS0, F_SIZE, app->user_id);
  if(!(*ppmapStart)) {
    TRM_OUT01(&ti, GR_INIT, LV_ERR, "mmap MMAP_OFFSET_EMIF_CS0(Flash area) failed, error '%s'", strerror(errno));

    DPR_DRV_IOCTL(*con_fd, CP16XX_IOC_CAPP, app, sizeof(*app), 0);
    DPR_DRV_CLOSE(*con_fd);
	  *con_fd = 0;

    return PNIO_ERR_NO_RESOURCE;
  }

  /* only for test
  for(int i = 0; i<16; i++){
    printf("%02x ", *((unsigned char*)*ppmapStart + i) );
  }
  printf("\n");*/


  /* Check flash type, if not 29LV6410H then return error. */
  
#define NO_HOST_GET_TYPE // may disturb FW Flash Write .. so the following check is removed 
#ifdef HOST_GET_TYPE
  f_type = flashTypeGet(*ppmapStart); 
  if((f_type != FLASH_29LV6410H) &&
       (f_type != FLASH_29GL064N90TFI06) ) {
    TRM_OUT01(&ti, GR_INIT, LV_ERR, "Incompatible flash type %x", f_type);

    ldah_close(*con_fd, app, (void*)*ppmapStart);
	  *con_fd = 0;
    return PNIO_ERR_INTERNAL;
  }
#endif
  //*con_fd = control_fd;
  return PNIO_OK;
}

/**
 * setFlag - set the FW_OK flag
 *
 * This routine sets the FW_OK flag at the end fo the flash after
 * successfully loading the firmware.
 *
 * RETURNS: 0, or -1 if anything went wrong.
 **/
int setFlag(DPR_CHAR *mapStart)
{
  FLASH_DEF *backup;

  if(memcmp("!FW_OK!!", mapStart + F_SIZE -8, 8) == 0)
    return(0);

  if((backup = (FLASH_DEF *)DPR_VMALLOC(S_SIZE)) == NULL) {
    TRM_OUT00(&ti, GR_INIT, LV_ERR, "Can not allocate memory to backup the serial data");
    return(-1);
  }

  memcpy(backup, mapStart + F_SIZE - S_SIZE, S_SIZE);
  memcpy((char *)backup + S_SIZE - 8, "!FW_OK!!", 8);

  if(flashSectorErase(mapStart, F_SIZE - S_SIZE) != STATUS_OK) {
    TRM_OUT00(&ti, GR_INIT, LV_ERR, "Can not erase the last sector in the flash");
    DPR_VFREE(backup);
    return(-1);
  }

  if(flashWrite(mapStart, backup, S_SIZE, F_SIZE - S_SIZE) != STATUS_OK) {
    TRM_OUT00(&ti, GR_INIT, LV_ERR, "Can not set the FW_OK flag");
    DPR_VFREE(backup);
    return(-1);
  }

  DPR_VFREE(backup);
  return(0);
}

bool ldah_get_fw_image_version(char *p_fw_image, unsigned long image_length, LDA_FW_VERS *p_fw_version)
{
  const char *pProFwVersion = LDA_LAD_FILE_VERSION_STR;
  const unsigned long ProFwVersionLen = (unsigned long)strlen(pProFwVersion);
  unsigned long i;
  int ver_tags_cnt;
  unsigned int vtags[5];

  char *pVersionBeginn = 0;

  memset(p_fw_version, 0, sizeof(*p_fw_version));

  if(image_length <= ProFwVersionLen) {
    TRM_OUT02(&ti, GR_INIT, LV_ERR, "ldah_get_fw_image_version image_length = %d < ProFwVersionLen = %d", image_length, ProFwVersionLen);
    return false;
  }

  /* find FW-Version */
  for(i = 0; i < image_length - ProFwVersionLen; i++) {
    if(memcmp(p_fw_image + i, pProFwVersion, ProFwVersionLen) == 0) {

      pVersionBeginn = p_fw_image + i + ProFwVersionLen;

      ver_tags_cnt = sscanf(pVersionBeginn, "%u.%u.%u.%u.%u", &vtags[0], &vtags[1], &vtags[2], &vtags[3], &vtags[4]);
      if(ver_tags_cnt != 5) {
        TRM_OUT01(&ti, GR_INIT, LV_ERR, "ldah_get_fw_image_version ver_tags_cnt = %d (expected 5)", ver_tags_cnt);
        return false;
      } else {
        p_fw_version->t1 = vtags[0];
        p_fw_version->t2 = vtags[1];
        p_fw_version->t3 = vtags[2];
        p_fw_version->t4 = vtags[3];
        p_fw_version->t5 = vtags[4];
        return true;
      }
    }
  }

  TRM_OUT00(&ti, GR_INIT, LV_ERR, "can't find firmware version in the firmware image");
  return false;
}

DPR_UINT32 ldah_flash_data(DPR_CHAR *mapFlash, DPR_UINT32 start_offset, DPR_UINT32 end_offset,
                                 DPR_CHAR *p_data, DPR_UINT32 data_length)
{
  int words, lwords;
  DPR_UINT32 ret = PNIO_OK, fw_lad_offset, fw_lad_size,i;
  DPR_CHAR *p_tmp = 0;

  switch (start_offset) {
	case BL_START_OFFSET:
			if ((end_offset - start_offset) == F_SIZE)
				fw_lad_offset = 0;
			else
				fw_lad_offset = LAD_FILE_OFFSET;
		break;
	case FW_START_OFFSET:
  fw_lad_offset = 16 * 16 * (int)p_data[1] + (int)p_data[0];
		break;
	default:
		fw_lad_offset = 0;
		break;

  }



  if(fw_lad_offset > data_length) {
    TRM_OUT00(&ti, GR_INIT, LV_ERR, "lad_offset is bigger than firmware image length");
    return PNIO_ERR_CORRUPTED_DATA;
  }

  fw_lad_size = data_length - fw_lad_offset;
  if(fw_lad_size > (end_offset - start_offset)) {
    TRM_OUT00(&ti, GR_INIT, LV_ERR, "firmware image length to big");
    return PNIO_ERR_CORRUPTED_DATA;
  }

  /* if fw_lad_size is not FLASH_WIDTH granular, than alloc new data area */
  if((i = (fw_lad_size % FLASH_WIDTH)) != 0) {
    //fw_lad_size = data_length - fw_lad_offset + FLASH_WIDTH - i;
	fw_lad_size = data_length + FLASH_WIDTH - i;
    /* alloc temporary buffer for firmware image */
    p_tmp = (DPR_CHAR *)DPR_VMALLOC(fw_lad_size * sizeof(DPR_CHAR));
    if(p_tmp == 0)
      return PNIO_ERR_NO_RESOURCE;

    memcpy(p_tmp, p_data + fw_lad_offset, fw_lad_size);
    memset(p_tmp + fw_lad_size - FLASH_WIDTH + i, 0xff, FLASH_WIDTH - i);

    p_data = p_tmp; /* don't forget to free */
  }

    /* The flash must be programed wordwise */
  if ((end_offset - start_offset) == F_SIZE) {
	/* going to load the whole flash, ignore the last sector */
	words = (fw_lad_size - S_SIZE) / (FLASH_WIDTH * 99);
	lwords = (fw_lad_size - S_SIZE) % (FLASH_WIDTH * 99);
	end_offset = SERIAL_START_OFFSET;
  } else {
	  words = fw_lad_size / (FLASH_WIDTH * 99);
	  lwords = fw_lad_size % (FLASH_WIDTH * 99);
  }

  /* Erase the firmware first */
  TRM_OUT00(&ti, GR_INIT, LV_INFO, "erase flash, be patient, need some time ...");

  if(flashEraseSectors(mapFlash, start_offset, end_offset, "firmware") != 0) {

    TRM_OUT00(&ti, GR_INIT, LV_ERR, "flashEraseSectors() failed");
    ret = PNIO_ERR_FLASH_ACCESS;
    goto ldah_flash_fwl_image_failed;
  }

  TRM_OUT00(&ti, GR_INIT, LV_INFO, "flash new firmware...");

  for(i = 0; i < 99; i++) {

    if(flashWrite(mapFlash, (FLASH_DEF*)(p_data + fw_lad_offset + i * words * FLASH_WIDTH), words * FLASH_WIDTH,
                  start_offset + i * words * FLASH_WIDTH) != STATUS_OK) {
      TRM_OUT01(&ti, GR_INIT, LV_ERR, "Can not load firmware(words*FLASH_WIDTH), %d words written", i * words);

      ret = PNIO_ERR_FLASH_ACCESS;
      goto ldah_flash_fwl_image_failed;
    }

    if((i+1)%10 == 0) {
      TRM_OUT01(&ti, GR_FL, LV_INFO, "firmware loading in progress, performed %2d%%", i+1);
	  PRINTOUT("firmware loading in progress, performed %2d%%\n", i+1);
    }
  }

  if(flashWrite(mapFlash, (FLASH_DEF*)(p_data + fw_lad_offset + i * words * FLASH_WIDTH), lwords,
                start_offset + i * words * FLASH_WIDTH) != STATUS_OK) {
    TRM_OUT01(&ti, GR_FL, LV_ERR, "can not load firmware(lwords), %d words written", i * words);

    ret = PNIO_ERR_FLASH_ACCESS;
    goto ldah_flash_fwl_image_failed;
  }

  setFlag(mapFlash);

  ldah_flash_fwl_image_failed:

  if(p_tmp) DPR_VFREE(p_tmp);

  return ret;
}

DPR_UINT32 ldah_download_firmware(DPR_UINT32 cp_idx, DPR_CHAR *p_data, DPR_UINT32 data_length)
{
  DPR_UINT32 ret;
  LDA_FW_VERS fw_version;

  DPR_DRV_HANDLE control_fd = 0;
  DPR_CHAR * mapStart = 0;
  struct t_register_app app;

  if(!ldah_get_fw_image_version((char*)p_data, data_length, &fw_version)) {
    return PNIO_ERR_CORRUPTED_DATA;
  }

  /* to do, ask FW about FW-Version,
     if(FW_Version_remote == FW_Version_image)
     return PNIO_ERR_ALLREADY_DONE; */

  ret = ldah_open(cp_idx, &app, &mapStart, &control_fd);
  if(ret != PNIO_OK) {
    return ret;
  }

  /* flash firmware image */
  ret = ldah_flash_data((DPR_CHAR *)mapStart, FW_START_OFFSET, FLASH_FS_OFFSET, p_data, data_length);
  TRM_OUT01(&ti, GR_INIT, LV_INFO, "ldah_flash_data(firmware image) ret 0x%x", ret);

  /*  Unregister an application to the driver. */
  /*if(DPR_DRV_IOCTL(control_fd, CP16XX_IOC_CAPP, &app, sizeof(app), 0)) {
    TRM_OUT01(&ti, GR_INIT, LV_ERR, "ioctl unregister application failed, error '%s'",
              strerror(errno));
  }

  if(DPR_DRV_IOCTL(control_fd, CP16XX_IOCSHUTDOWN, NULL, 0, 0)) {
    TRM_OUT01(&ti, GR_INIT, LV_ERR, "resetting the card failed, error '%s'",
              strerror(errno));
  } else {
    TRM_OUT00(&ti, GR_INIT, LV_INFO, "card resetting performed");
  }*/

  ldah_close(control_fd, &app, (void*)mapStart);

  if(ret == PNIO_OK) {
    /* reset cp to run up new firmware */
    ret = ldah_reset_firmware(cp_idx, 0);
  }

  return ret;
}

/* read backup image and flash in cp */
DPR_UINT32 ldah_image_restore(unsigned long cp_idx, DPR_CHAR *mapFlash,
                                  DPR_UINT32 start_offset, DPR_UINT32 end_offset, char *file_name)
{
  int fw_fd;
  struct stat fw_stat;
  unsigned long ret;

  fw_fd = open(file_name, O_RDONLY | O_BINARY, UMASK);
  if(fw_fd == -1) {
    TRM_OUT01(&ti, GR_INIT, LV_INFO, "Cannot open backup file <%s>", file_name);
    return PNIO_ERR_CORRUPTED_DATA;
  } else {

    /* get file size */
    if(fstat(fw_fd, &fw_stat) == -1) {
      TRM_OUT01(&ti, GR_INIT, LV_ERR, "fstat failed, errno-txt %s", strerror(errno));
      (void)close(fw_fd);
      return PNIO_ERR_CORRUPTED_DATA;
    }

    DPR_CHAR *p_fw_image = (DPR_CHAR*)DPR_VMALLOC(fw_stat.st_size);
    if(p_fw_image == 0) {
      TRM_OUT01(&ti, GR_INIT, LV_ERR, "can't allocate %d bytes for fw-image", fw_stat.st_size);
      (void)close(fw_fd);
      return PNIO_ERR_INTERNAL;
    }

    if(read(fw_fd, p_fw_image, fw_stat.st_size) <= 0 ) {
      TRM_OUT01(&ti, GR_INIT, LV_ERR, "read failed, errno-txt %s", strerror(errno));
      DPR_VFREE(p_fw_image);
      (void)close(fw_fd);
      return PNIO_ERR_CORRUPTED_DATA;
    }

	    /* download fwl image */
    ret = ldah_flash_data(mapFlash, start_offset, end_offset, p_fw_image, fw_stat.st_size);

    DPR_VFREE(p_fw_image);
    (void)close(fw_fd);
	if ( ret != PNIO_OK ) {
	    TRM_OUT01(&ti, GR_INIT, LV_INFO, "ldah_flash_data failed ret=0x%x", ret);
		return PNIO_ERR_FLASH_ACCESS;
	}

  }

  return PNIO_OK;
}
