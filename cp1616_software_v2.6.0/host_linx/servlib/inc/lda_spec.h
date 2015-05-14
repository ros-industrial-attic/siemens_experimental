/****************************************************************************
*  Copyright (C) SIEMENS CORP., 2013 All rights reserved.
*****************************************************************************
* FILE NAME    : lda_spec.h
* ---------------------------------------------------------------------------
* DESCRIPTION  : data types and function declarations for local data agent
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

#ifndef LDA_SPEC_H
#define LDA_SPEC_H

#undef ATTR_PACKED
#if defined(_MSC_VER)
  #pragma pack( push, safe_old_packing, 1 )
  #define ATTR_PACKED
#elif defined(__GNUC__)
  #define ATTR_PACKED  __attribute__ ((packed))
#elif defined(BYTE_ATTR_PACKING)
  #include "pack.h"
  #define ATTR_PACKED PPC_BYTE_PACKED
#else
  #error please adapt pniobase.h header for your compiler
#endif

#undef PNIO_CODE_ATTR
#ifdef PASCAL
  #define PNIO_CODE_ATTR __stdcall
#else
  #define PNIO_CODE_ATTR
#endif

#define LDA_API_VERSION 1000

/* LE = LW.LB|LW.HB|HW.LB|HW.HB */
/* BE = HW.HB|HW.LB|LW.HB|LW.LB */

typedef struct {
  DPR_CHAR blo;
  DPR_CHAR bhi;
} ATTR_PACKED LDA_LE_UINT16;

typedef struct {
  LDA_LE_UINT16 wlo;
  LDA_LE_UINT16 whi;
} ATTR_PACKED LDA_LE_UINT32;

typedef struct {
  DPR_CHAR bhi;
  DPR_CHAR blo;
} ATTR_PACKED LDA_BE_UINT16;

typedef struct {
  LDA_BE_UINT16 whi;
  LDA_BE_UINT16 wlo;
} ATTR_PACKED LDA_BE_UINT32;

inline LDA_LE_UINT16 lda_nat_2_le16(unsigned long c){
  LDA_LE_UINT16 ret;
  ret.blo = (DPR_CHAR)(CPU_TO_LE16(c));
  ret.bhi = (DPR_CHAR)(CPU_TO_LE16(c) >> 8);
  return ret;
}

inline LDA_LE_UINT32 lda_nat_2_le32(unsigned long c){
  LDA_LE_UINT32 ret;
  ret.wlo = lda_nat_2_le16(c);
  ret.whi = lda_nat_2_le16(c >> 16);
  return ret;
}

inline unsigned long lda_le32_2_nat(LDA_LE_UINT32 c){
  return LE_TO_CPU(*(DPR_UINT32*)&c);
}

inline unsigned long lda_le16_2_nat(LDA_LE_UINT16 c){
  return LE_TO_CPU16(*(DPR_UINT16*)&c);
}

inline LDA_BE_UINT16 lda_nat_2_be16(unsigned long c){
  LDA_BE_UINT16 ret;
  ret.blo = (DPR_CHAR)(CPU_TO_BE16(c) >> 8 );
  ret.bhi = (DPR_CHAR)(CPU_TO_BE16(c));
  return ret;
}

inline LDA_BE_UINT32 lda_nat_2_be32(unsigned long c){
  LDA_BE_UINT32 ret;
  ret.wlo = lda_nat_2_be16(c);
  ret.whi = lda_nat_2_be16(c >> 16);
  return ret;
}

inline unsigned long lda_be32_2_nat(LDA_BE_UINT32 c){
  return BE_TO_CPU(*(unsigned long*)&c);
}

inline unsigned long lda_be16_2_nat(LDA_BE_UINT16 c){
  return BE_TO_CPU16(*(unsigned long*)&c);
}

inline LDA_LE_UINT16 lda_be16_2_le16(LDA_BE_UINT16 c)
{
  LDA_LE_UINT16 ret_le;
  ret_le.blo = c.blo;
  ret_le.bhi = c.bhi;
  return ret_le;
}

inline LDA_BE_UINT16 lda_le16_2_be16(LDA_LE_UINT16 c)
{
  LDA_BE_UINT16 ret_be;
  ret_be.blo = c.blo;
  ret_be.bhi = c.bhi;
  return ret_be;
}

#define LDA_XDB_HEADER_TYPE 0xAF
#define LDA_XDB_HEADER_ID   0x04
#define LDA_XDB_CONTAINER_TYPE 0xFF
#define LDA_XDB_CONTAINER_ID   0x01

typedef struct {
  DPR_CHAR        HdrType;   /* Header type 0xAF */
  DPR_CHAR        HdrId;     /* Header id 0x04 */
  LDA_LE_UINT16   HdrLen;    /* Header length */
  LDA_LE_UINT16   HdrOriginatorStirngLen;
  DPR_CHAR        HdrOriginatorStirng[64];
  LDA_LE_UINT16   HdrXdbVersion;
  LDA_LE_UINT16   HdrContainerNumber;
  DPR_CHAR        HdrReserved[4];
} ATTR_PACKED LDA_XDB_HEADER;

typedef struct {
  DPR_CHAR        ConType;   /* Container type */
  DPR_CHAR        ConId;     /* Container id */
  LDA_LE_UINT32   ConLen;    /* Containel length */
  DPR_CHAR        ConVmdid;  /* Container VmdId */
  DPR_CHAR        ConReserved; /* reserved */
} ATTR_PACKED LDA_XDB_CON_HEADER;

typedef struct {
  LDA_LE_UINT32   SdbDirLen;         /* Directory length */
  LDA_LE_UINT16   SdbDirVersion;     /* Directory version */
  LDA_LE_UINT32   SdbDirItemsNumber; /* Directory items number */
  DPR_CHAR        SdbDirReserved[4]; /* reserved */
} ATTR_PACKED LDA_XDB_CON_SDB_DIR_HEADER;

typedef struct {
  LDA_LE_UINT16   SdbNumber;      /* Sdb number */
  LDA_LE_UINT16   SdbContent;     /* Sdb content */
  LDA_LE_UINT32   SdbOffset;      /* Sdb offset */
  LDA_LE_UINT32   SdbLen;         /* Sdb length */
  DPR_CHAR        SdbReserved[4]; /* reserved */
} ATTR_PACKED LDA_XDB_CON_SDB_DIR_ITEM;

/* obsolet
typedef struct {
  LDA_LE_UINT16 BlockType;
  LDA_LE_UINT16 SDBHeaderLen;
  LDA_LE_UINT16 SDBLen;
  LDA_LE_UINT16 SDBNumber;
  LDA_LE_UINT16 DeviceType;
  LDA_BE_UINT16 SDBContent;
  DPR_CHAR  VmdID;
  DPR_CHAR  Reserved;
} ATTR_PACKED LDA_XDB_SDB_HEADER;*/

/* definition of backup format for configuration data */

#define LDA_XDB_STM_SLOT_NUMBER  125 /* virtual slot number from station manager in xdb format */
#define LDA_XDB_CP_SLOT_NUMBER     1 /* virtual slot number from cp in xdb format */
#define LDA_XDB_IPC_SLOT_NUMBER  201 /* virtual slot number from ip-config in xdb format */

#define LDA_OVS_BST_HEADER_ID    0x7070
#define LDA_S5_DYNAMIC_CREATED   0x08
#define LDA_OVS_TIMESTAMP_LEN    6

typedef struct {
  DPR_UINT16 SDBNumber;
  DPR_UINT16 SDBContent;
  DPR_UINT32 SDBLen;
  DPR_CHAR   SdbTimeStamp [LDA_OVS_TIMESTAMP_LEN];
  DPR_UINT16 VmdID;
  DPR_UINT32 OffsetOfSdbBody; /* from beginn of Section 2. SDB-Datas (LDA_OVSSDB_LIST_HEADER::OffsetOfSDBDatas) */
  DPR_CHAR   Reserved[4];
} ATTR_PACKED LDA_OVSSDB_LIST_DIR_ITEM;

typedef struct {
  DPR_UINT32 Type;    /* use 0x00000A0A as valid OVSSDB_LIST Type */
  DPR_UINT32 Version; /* use 0 first */
  DPR_UINT32 HdrLen;          /* length of LDA_OVSSDB_LIST_HEADER */
  DPR_UINT32 SDBListLen;      /* length of whole data block = LDA_OVSSDB_LIST_HEADER + OVS_LIST_DIRECTORY + SDB-Datas */

  DPR_UINT32 SDBDirCnt;      /* count of LDA_OVSSDB_LIST_DIR_ITEM items (Section 1) */
  DPR_UINT32 SDBDatasOffset; /* offset of Section 2 */
  DPR_UINT32 SDBDatasLen;    /* length of Section 2 */

  LDA_OVSSDB_LIST_DIR_ITEM * SDBDir;   /* reserved, for internal use only */
  DPR_CHAR                * SDBDatas;  /* reserved, for internal use only */

  /* Section 1. OVS_LIST_DIRECTORY is variable count of LDA_OVSSDB_LIST_DIR_ITEM items */
  /*    LDA_OVSSDB_LIST_DIR_ITEM[SdbCnt] */
  /* Section 2. SDB-Datas */
} ATTR_PACKED LDA_OVSSDB_LIST_HEADER;

/* definition of OVSSDB header */
typedef struct {
  LDA_BE_UINT16 head_id;           /* o:00 header id: 0x7070                   */
  DPR_CHAR      sys_vers_no;       /* o:02 Bit 7 = 0 : Baustein <= 64 k        */
  /* Bit 7 = 1 : Baustein >  64 k        */
  /*             => DPR_UINT16 len2 and DPR_UINT16 len3 */
  /*             become DPR_UINT32 len2and3 */
  DPR_CHAR  attr;                  /* o:03 */
  DPR_CHAR  tool;                  /* o:04 */
  DPR_CHAR  blk_typ;               /* o:05 typ of the block, i.e SDB, DB      */
  LDA_BE_UINT16 blk_nr;            /* o:06 Number of the block (KEY)          */
  LDA_BE_UINT32 blk_len;           /* o:08 len of the complete block          */
  LDA_BE_UINT32 password;          /* o:12 */
  DPR_CHAR  time_stamp1 [6];       /* o:16 time stamp 1: object ident         */
  DPR_CHAR  time_stamp2 [6];       /* o:22 time stamp 2: interface ident      */
  LDA_BE_UINT16 len2;              /* o:28 len of part 2                      */
  LDA_BE_UINT16 len3;              /* o:30 len of part 3                      */
  LDA_BE_UINT16 locdatalen;        /* o:32 */
  LDA_BE_UINT16 len1;              /* o:34 len of part1 (parameter blocks)    */

} ATTR_PACKED LDA_OVS_BST_HEADER; /* 36 byte long */

#define LDA_OVS_CNSTY_ATTR_NO_CNSTY                      0x00
#define LDA_OVS_CNSTY_ATTR_INCLUD_IN_CNSTY_SDB           0x01
#define LDA_OVS_CNSTY_ATTR_IS_CNSTY_SDB                  0x02
#define LDA_OVS_CNSTY_ATTR_IS_AND_INCLUD_IN_CNSTY_SDB    0x03

typedef struct {
  DPR_CHAR  len;
  DPR_CHAR  pbl_id;
  LDA_BE_UINT16 baugruppen_typ;
  LDA_BE_UINT16 sdb_ident;
  DPR_CHAR  load_attr;
  DPR_CHAR  cnsty_attr;        /* see SPH AS mechanisms vol 2 chp. 1.92 LDA_OVS_CNSTY_ATTR_... */
  DPR_CHAR  baugruppen_id;
  DPR_CHAR  cpu_interface;
  DPR_CHAR  rem_tsap [2];
  DPR_CHAR  next_staddr;
  DPR_CHAR  rem_staddr;
} ATTR_PACKED LDA_OVS_SDB_1000_PBL; /* 14 byte long */


typedef struct {
  LDA_OVS_BST_HEADER    head;   /* header of the block                  */
  LDA_OVS_SDB_1000_PBL info;    /* additional info for SDB > 1000        */
} ATTR_PACKED LDA_OVS_BST;

#define LDA_OVS_SDB_ID_PNIO_CNSTY               0x1200
#define LDA_PBL_ID_PNIO_CNSTY_DATA              0x01

typedef struct /* must be saved in BIGENDIAN */ {
  LDA_BE_UINT16         BlockNumber;
  LDA_BE_UINT16         SDBIdent;

  DPR_CHAR              TimeStamp [LDA_OVS_TIMESTAMP_LEN];

  LDA_BE_UINT16         Reserved_0;
}ATTR_PACKED LDA_OVS_PNIO_CNSTY_DATA;

typedef struct /* must be saved in BIGENDIAN */ {
  DPR_CHAR              PrmBlockID;
  DPR_CHAR              Reserved_0;
  LDA_BE_UINT16         Length;

  LDA_BE_UINT32         Reserved_1;

  LDA_BE_UINT16         SDB_Cnt;
  LDA_BE_UINT16         CE_Len;  /* Anzahl der OVS_PNIO_CNSTY_PC_DATA-struct */
}ATTR_PACKED LDA_OVS_PNIO_CNSTY_PRM_BLK;

#define LDA_OVS_SDB_ID_CNSTY              0x1000
#define LDA_PBL_ID_CNSTY_DATA             0x01

typedef struct {
  LDA_BE_UINT16         BlockNumber;
  LDA_BE_UINT16         SDBIdent;

  DPR_CHAR              TimeStamp [LDA_OVS_TIMESTAMP_LEN];
}ATTR_PACKED LDA_OVS_CNSTY_DATA;

typedef struct {
  DPR_CHAR              PrmBlockID;
  DPR_CHAR              vers;
  DPR_UINT16             len;     /* LITTLENDIAN */
  DPR_UINT16             SdbCnt;  /* LITTLENDIAN */
  /*LDA_OVS_CNSTY_DATA   Entry[260];*/
}ATTR_PACKED LDA_OVS_CNSTY_SDB;

typedef struct {
  DPR_UINT16  t1;
  DPR_UINT16  t2;
  DPR_UINT16  t3;
  DPR_UINT16  t4;
  DPR_UINT16  t5;
} ATTR_PACKED LDA_FW_VERS;

typedef struct CP_FWLDA {
  DPR_DRV_HANDLE              fd_control;
  DPR_DRV_HANDLE              mgt_chnl;
  DPR_THREAD_HANDLE              th_reader;
  DPR_UINT16                     stop_thread;
  DPR_UINT32                  cp_index;
  DPR_UINT32                  user_id;
  DPR_CHAR              * user_pool_ptr;
  DPR_UINT32              user_pool_length;
  DPR_UINT32              send_pool_length;
  DPR_SEMAPHORE         sem_cnf_received;
} CpFwlda;

#define LDA_FWL_SIGNATUR     "SIMATIC-NET FW-Loader 1616 Host Backup"
#define LDA_FWL_SIGNATUR_LEN 38
#define LDA_LAD_FILE_VERSION_STR  "CP 1616/CP 1604  Version:  V"

typedef struct {
  LDA_LE_UINT16 LadFileOffset;
  LDA_LE_UINT16 VersionOfFwlFormat;
  DPR_CHAR  FwlSignaturLen;
  DPR_CHAR  FwlSignatur[LDA_FWL_SIGNATUR_LEN];

  DPR_CHAR  LadFileVersionLen;
  /* DPR_CHAR LadFileVersion[LadFileVersionLen] */
  /* DPR_CHAR LadFileContent[...] */

} ATTR_PACKED LDA_FWL_HEADER;

/* Serial information related:   !!!Attention!!!
 * -----------------------------------------------
 * LDA_FLASH_HW_INFO: This structure is also used by the flashrescue_cmd.
 * It must be synchronized manually with the definition of the firmware struct FLASH_HW_INFO
 * defined in cp1616dk\firmware\fwloader\inc\bootdef.h
 *
 * PL: 11.02.2009 parameter added: PHY type
 */
#define MAX_MLFB_LEN          32
#define MAX_SER_LEN           16
#define MAX_BOOT_VERS_LEN     24
#define HW_INFO_OFFSET        0x103   /* Start offset with the last 64K */

typedef struct {
    DPR_CHAR macEthernet[6];
    DPR_CHAR MLFB[MAX_MLFB_LEN];
    DPR_CHAR serialNum[MAX_SER_LEN];
    DPR_CHAR bootVersion[MAX_BOOT_VERS_LEN];
    DPR_CHAR num_mac_adrs;
    DPR_CHAR all_mac_adrs_saved;
    DPR_CHAR macExt[5*6];
    DPR_CHAR hw_ver[2];
    DPR_CHAR phy_type;
	DPR_CHAR padding[69];	 /* space for future use: total len=6+32+16+24+1+1+30+2+1+69=182  */
} ATTR_PACKED LDA_FLASH_HW_INFO;

#ifdef __cplusplus
extern "C" {
#endif

/* functions */

#ifdef __cplusplus
}
#endif


#if defined(_MSC_VER)
  #pragma pack( pop, safe_old_packing )
#elif defined(BYTE_ATTR_PACKING)
  #include "unpack.h"
#endif

#endif /* LDA_SPEC_H */
