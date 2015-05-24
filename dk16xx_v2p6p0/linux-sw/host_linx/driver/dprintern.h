/*****************************************************************************/
/*  Copyright (C) SIEMENS CORP., 2013 All rights reserved.*/
/*****************************************************************************/
/*  F i l e                dprintern.h                                       */
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

#ifndef _DPRINTERN_H
#define _DPRINTERN_H

#include "dprlib.h" /* include DPRAM interface */

#if defined(_MSC_VER)
 #pragma pack( push, safe_old_packing, 4 )
 #define  ATTR_PACKED
#elif defined(__GNUC__)
 #define ATTR_PACKED  __attribute__ ((aligned (4)))
#elif defined(BYTE_ATTR_PACKING)
 #include "pack.h"
 #define ATTR_PACKED PPC_BYTE_PACKED
#else
 #error please adapt dprintern.h header for your compiler, need byte packing
#endif

/* define Interrupt types */

#define DPR_INT_DPR_READY       0xA5A07777 /* initialization value for interrupt reason SPI/HPI... */
#define DPR_INT_DPR_ERR         0xA5A08888 /* error value for interrupt reason SPI/HPI... */

/* configuration interrupt reason: (re)configuration of DPRAM(CHannels), CP-Restart: 
                                              will be setted from FW and HOST in   pCP->pDpramConfigArea->HPIConfig,
                                              will be readed from FW and HOST from pCP->pDpramConfigArea->SPIConfig */ 

#define DPR_INT_CP_START        0xA5A01111 /* setted from FW, if Firmware is started, 
                                              FW-DRP is ready to start connection from host driver */
#define DPR_INT_CONF_DPRAM      0xA5A070A0 /* hostdriver orders FW to initialize DPRAM(for example after reset), 
                                              FW does that and confirms with interrupt */
#define DPR_INT_CONF_CHNL       0xA5A060B0 /* hostdriver orders FW to initialize DPRAM-Channels, 
                                              FW does that and contirms with interrupt */
#define DPR_INT_RESTART_REQ     0xA5A050C0 /* FIRMWARE orders Host to restart FIRMARE(for example after download new fw-images),
                                             (Firmare can't restarts him self)*/
#define DPR_INT_DPR_SHUTDOWN    0xA5A0EFFF /* setted from hostdriver,
                                              hostdriver shutdown, hostdriver releases DPRAM interface */

/* dpram-message interrupt reason: one message in DPRAM-Channels is available, 
                                              will be setted from FW and HOST in   pCP->pDpramConfigArea->HPIMessage,
                                              will be readed from FW and HOST from pCP->pDpramConfigArea->SPIMessage */
#define DPR_INT_MESSAGE         0xA5A0A020 /* one message in DPRAM-Channels is available */

/* dpram-message interrupt reason: read comfirmation for one full-ring 
                                              will be setted from FW and HOST in   pCP->pDpramConfigArea->HPIMessageRC,
                                              will be readed from FW and HOST from pCP->pDpramConfigArea->SPIMessageRC */
#define DPR_INT_MESSAGERC       0xA5A0B050 /* one of the full-rings war readed, write can write again */


#define DPR_INT_IO_DATA         0xA5A0C070 /* reserved, not used */

/* define lock types */
#define DPR_LOCK_RING           0x52528000
#define DPR_UNLOCK_RING         0x52520000

#define DPR_OWNER_WRITE         0x52523333

#define DPR_RING_FULL           0x52526666
#define DPR_RING_NOTFULL        0x52526677

#define DPR_OWNER_READ          0x52525555

#define DPR_SAFTY_ZONE_DATA     0x94948585

#define DPR_MESSAGE_CHECKSUM    0xA5A04488

#define DPR_CONFIG_AREA_OFFSET  0x00100000  /* at 1 mb from begin of DPRAM */

typedef struct _DPR_MSG_HEAD {
    DPR_UINT32 MsgLength;     /* message Length */
    DPR_UINT32 FctHandle;     /* Function handle is filled by the PNIOLIB or
                                any other libraries in host and used by FW */
} ATTR_PACKED DPR_MSG_HEAD;

typedef volatile struct _RING {
    DPR_VUINT32 pRingStart;
    DPR_VUINT32 pRingEnd;
    DPR_VUINT32 pRead;
    DPR_VUINT32 pWrite;
    DPR_VUINT32 RingFull;
} ATTR_PACKED RING,*PRING;

#if defined(_DPR_FW)

#define DPRLIB_READ_RING(Channel) (Channel*2)
#define DPRLIB_WRITE_RING(Channel) (Channel*2+1)

typedef volatile struct _DPR_BASE {
    DPR_VUINT32 DummyMemory; /* memory which should not be written due to the 1K bug of the DPRAM */
    DPR_VUINT32 DpramMaxSize;
    DPR_VUINT32 UserSize;
    DPR_VUINT32 RemoteVersion;
    DPR_VUINT32 LocalVersion;
    DPR_VUINT32 HPIConfig; /* FW write here the confirmation to HOST configuration request */
    DPR_VUINT32 SPIConfig; /* FW reads here the config interrupt reason */
    DPR_VUINT32 HPIIOData; /* interrupt handling for IO-Data, not used, reserved */
    DPR_VUINT32 SPIIOData; /* interrupt handling for IO-Data, not used, reserved */
    DPR_VUINT32 HPIMessage;
    DPR_VUINT32 SPIMessage;
    DPR_VUINT32 HPIMessageRC;
    DPR_VUINT32 SPIMessageRC;
    RING  Rings[DPR_CFG_MAX_CHANNEL * 2];
} ATTR_PACKED DPR_BASE, *PDPR_BASE;

#else

#define DPRLIB_READ_RING(Channel) (Channel*2+1)
#define DPRLIB_WRITE_RING(Channel) (Channel*2)

typedef struct _DPR_BASE {
    DPR_VUINT32 DummyMemory; /* memory which should not be written due to the 1K bug of the DPRAM */
    DPR_VUINT32 DpramMaxSize;
    DPR_VUINT32 UserSize;
    DPR_VUINT32 LocalVersion;
    DPR_VUINT32 RemoteVersion;
    DPR_VUINT32 SPIConfig;  /* HOST reads here the confirmation from FW*/
    DPR_VUINT32 HPIConfig;  /* HOST writes here the congiguration requst to FW */
    DPR_VUINT32 SPIIOData;
    DPR_VUINT32 HPIIOData;
    DPR_VUINT32 SPIMessage;
    DPR_VUINT32 HPIMessage;
    DPR_VUINT32 SPIMessageRC;
    DPR_VUINT32 HPIMessageRC;
  RING  Rings[DPR_CFG_MAX_CHANNEL * 2];
} ATTR_PACKED DPR_BASE, *PDPR_BASE;

#endif

#if defined(_MSC_VER)
 #pragma pack( pop, safe_old_packing )
#elif defined(BYTE_ATTR_PACKING)
 #include "unpack.h"
#endif

/*****************************************************************************/
/* Computing macros which switch between physical Address and Offset which   */
/* will be stored in the ring structures.                                    */
/* the base for the offsets will be the pointer to the configuration area    */
/*****************************************************************************/
#define GETOFFSET(addr) ((DPR_UINT32)(DPR_PTR_TO_ULONG(addr) - DPR_PTR_TO_ULONG(pCP->pDpramConfigArea)))
#define GETADDR(offs) (((DPR_CHAR *)(pCP->pDpramConfigArea))+((offs)))

/*****************************************************************************/
/* if any thread waits for Read/Write/ReadComplete Locks, it needs           */
/* notification for each channel. In Addition the callback pointer for       */
/* notifying the application is stored here.                                 */
/*****************************************************************************/
/* keep the channels in priority order */

typedef struct _ChannelData {
    DPR_SEMAPHORE     bsemRC;         /* for ReadComplete notification */
    DPR_SEMAPHORE     bsemWriteLock;  /* for Write functions           */
    DPR_SEMAPHORE     bsemReadLock;   /* for Read functions           */
    DPR_READ_CBF      readcbf;        /* read callback function for a channel*/
} ChannelData;

#ifndef _DPR_FW
typedef struct _DPR_CHANNEL_INFO {
    DPR_UINT32 TDRSize;
    DPR_UINT32 RDRSize;
} DPR_CHANNEL_INFO;

enum work { DPRLIB_START, DPRLIB_STOP, DPRLIB_RESET_FW };

#endif

/*****************************************************************************/
/* structure containing the module data for one CP                           */
/*****************************************************************************/
typedef struct _CpData {
    PDPR_BASE pDpramConfigArea;            /* base pointer to the configuration block */
    ChannelData ChannelDatas[DPR_CFG_MAX_CHANNEL];     /* channel info for each channel */
#ifndef _DPR_FW
    DPR_CHANNEL_INFO dprlib_channel_info[DPR_CFG_MAX_CHANNEL];

    DPR_SEMAPHORE sem_config_conf; /* configuration from Firmware arrived */

    void (*trigger_irq)(void*);
    void (*wakeup_daemon)(void*, enum work);

    void  *parent;
#endif
    DPR_THREAD_HANDLE thndl_proc;
    DPR_SEMAPHORE     sem_proc;       /* semaphore and thread for controlling interrupt thread */

    DPR_ATOMIC bDriverConnected;
    DPR_ATOMIC dprStopAllThreads; /* safe removal of threads */
    DPR_ATOMIC semaphores_valid;
    unsigned long loop_counter;
} CpData;

#ifdef _DPR_FW
    extern CpData FWCpData;
#endif

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************/
/* Tooling functions which are implemented in the common part of the lib     */
/*****************************************************************************/
extern unsigned long dprlib_align(unsigned long Size);   /* util for alignment of pointers */
extern int dprlib_message_proc(void *, DPR_CHN_TYPE, DPR_CHN_TYPE, DPR_UINT32); /* procedure for messages */
extern void dprlib_messagerc_proc(void *);               /* procedure for Read Confirmation */
extern DPR_THREAD_RETURN_TYPE dprlib_proc(void *);           /* interrupt procedure */
extern void dprlib_config_proc(void *);                  /* configuration procedure */
extern void dprlib_int_callback(void *);                 /* interrupt callback function */
extern DPR_VOID dprlib_stop_all_tasks_and_semaphores(CpData *pCP); /* stops all waiting tasks  */
extern int dprlib_start_all_tasks_and_semaphores(CpData *pCP); /* start all waiting tasks */
extern DPR_UINT32 dprlib_get_remote_version(CpData *pCP);
extern void dprlib_post_local_version(CpData *pCP);

/*****************************************************************************/
/* Tooling functions which are implemented in the device dependend part of   */
/* the lib                                                                   */
/*****************************************************************************/
extern DPR_VOID dprlib_trigger_intr(CpData *pCP); /* triggers interrupt */
extern DPR_VOID dprlib_trigger_self_intr(CpData *pCP); /* triggers own interrupt */
extern int dprlib_init_dpram(CpData *pCP, DPR_VUINT32 *Size);
extern DPR_VOID dprlib_uninit_dpram(CpData *pCP, DPR_UINT32 errCode);
extern int dprlib_init_channels(CpData *pCP);

#ifdef __cplusplus
}
#endif

#endif /* _DPRINTERN_H */
