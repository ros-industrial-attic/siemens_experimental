/****************************************************************************
*  Copyright (C) SIEMENS CORP., 2013 All rights reserved.
*****************************************************************************
*         FILE : pniointr.h
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

#ifndef _PNIO_PNIOINTR_H
#define _PNIO_PNIOINTR_H

#include "os.h"
#include "pnioerrx.h"
#include "pniousrx.h"
#include "pniousrd.h"
#include "pniousrt.h"
#include "traceint.h"
#include "iodataupdate.h"
#ifdef IO_ROUTER
#include "ioconcentrator.h"
#include "ior_dpram.h"
#include "transferwd.h"
#endif /* IO_ROUTER */

#ifdef PROFI_ENERGY
#include "pnioag_rqb.h"
#include "pnio_pe_pdu.h"
#include "pnio_pe.h"
#endif /* PROFI_ENERGY */

#define CI_POOL_ENTRY_NR 20
#define CI_POOL_MEASURE_NR 50

#define MIN(x, y) (((x) < (y))?(x):(y))

/* #define IRT_NODMA */
#ifndef IRT_NODMA
/* #define IRT_DMA_VIA_HOST */
#endif /* IRT_NODMA */

class ICommon;

typedef enum tagChannels {
    SYNCH = 0,
    ALARM = 1,
    MODIND = 2,
    DATAREC = 3,
    MGT_WD = 4,
    IRT_STARTOP = 5,
    IRT_OPFAULT = 6,
    IRT_NEWCYCLE = 7,
    CHANNELS_TOTAL
} CHANNELS;

typedef struct tagChannel {
    int stop_thread;
    int ready;
    int exited;
    unsigned channel_number;
    DPR_DRV_HANDLE file;

    unsigned long user_pool_length_used; /* actualy readed from read-ring */
    unsigned long user_pool_length;      /* allocated length of read-ring buffer to read from fw */
    char *user_pool_ptr;                 /* read-ring buffer to read from fw */

    unsigned long send_pool_length;      /* maximal possible length to write(send) to fw */
    char *send_pool_ptr;                 /* temporary send buffer, if channel allow send to fw */

    DPR_THREAD_HANDLE th_reader;

    void *parent;
} CHANNEL;

typedef struct tagDprAdapter {
    PNIO_UINT32 CpIndex;
    ICommon *icommon_ptr;
    unsigned long user_id;
    DPR_DRV_HANDLE fd_control;
    CHANNEL chnls[CHANNELS_TOTAL];

    DPR_MUTEX synch_channel_mutex;
#ifdef IO_ROUTER
    char *pShMemIocTable;  /* io-router concentrator tabele */
#endif /* IO_ROUTER */
    char *pKramTlb;
    char *pErtecSwiBase;
    char *pErtecIOTotal; /* contents of IRTin, IRTout, SRTin, SRTout IO space */
    char *pIRTDMAImage;

    char IrtAccessStatus;

    IODU_Item *pIODUItem;

    void *pIRTDMAPhysAddr;
    unsigned long IRTDMALen;
} DPR_ADAPTER;

typedef struct {
        PNIO_BOOL       Valid;
        PNIO_CI_ENTRY   Entry;
} CI_POOL_ENTRY;

/*
 * Access types
 */
enum ACC_T {
    ACC_T_READ,
    ACC_T_WRITE,
    ACC_T_READ_OUTPUT
};

typedef struct {
  unsigned int     ciValidCount;
  CI_POOL_ENTRY    ciPoolExt[CI_POOL_ENTRY_NR];
  PNIO_CYCLE_STAT  ciPoolStatExt;
} CI_POOL_EXT_ENTRY;

/*
 * Base class for controller and Device
 */
class ICommon {
  public:
    ICommon(void);
    virtual ~ICommon(void);

    /* create(append to vector) or find DPR_ADAPTER
       and attach current instance to it */
    PNIO_UINT32 InitCp(PNIO_UINT32 CpIndex);

    /* dettach this instance from DPR_ADAPTER,
       if the last instance, destroy DPR_ADAPTER */
    PNIO_UINT32 UninitCp(void);

    /* something with fast close */
    void set_emergency_close(bool EmergencyClose) {
        m_bEmergencyClose = EmergencyClose;
    };

    virtual PNIO_UINT32 ProcAlarm(CHANNEL *channel) = 0;
    virtual PNIO_UINT32 ProcModeInd(CHANNEL *channel) = 0;
    virtual PNIO_UINT32 ProcDataRec(CHANNEL *channel) = 0;

    virtual PNIO_UINT32 TestBlockSendReceive(PNIO_UINT8 *buffer,
        PNIO_UINT32 length) {
        return PNIO_ERR_INTERNAL;
    };

    virtual PNIO_UINT32 RegisterStartOp(void) = 0;

    PNIO_UINT32 Send(CHANNELS Chnl, char *pRq, PNIO_UINT32 send_length);
    PNIO_UINT32 SendReceiveSynch(char *pRq, PNIO_UINT32 send_length,
        PNIO_UINT32 *receive_length, PNIO_UINT32 timeout_ms = 5000);
    PNIO_UINT32 OpenDprChannel(const char *device, DPR_ADAPTER *adapter, CHANNELS chnl);
    PNIO_UINT32 CloseDprChannel(CHANNEL *chn);

    static ICommon *get_instance(PNIO_UINT32);
    static PNIO_UINT32 get_handle(ICommon *);

    static PNIO_UINT32 set_appl_watchdog(PNIO_UINT32 CpIndex,
        PNIO_UINT32 wdTimeOutInMs, PNIO_CBF_APPL_WATCHDOG pnio_appl_wd_cbf);
    static PNIO_UINT32 trigger_watchdog(PNIO_UINT32 CpIndex);

    PNIO_CP_CBF m_pCbf_StartOp_Ind;
    PNIO_CP_CBF m_pCbf_OpFault_Ind;
    PNIO_CP_CBF m_pCbf_NewCycle_Ind;

    PNIO_UINT32 CP_register_cbf(PNIO_CP_CBE_TYPE CbeType, PNIO_CP_CBF Cbf);
#ifdef IO_ROUTER
    void (*m_pCbf_TransferWD) (PNIO_CP_CBE_PRM  *, IController *);
    PNIO_UINT32 CP_register_cbf_transferwd(void (*) (PNIO_CP_CBE_PRM  *, IController *));
#endif /* IO_ROUTER */
    PNIO_UINT32 CP_set_opdone(PNIO_CYCLE_INFO * pCycleInfo);

    PNIO_UINT32 get_iodataupdate_handle(void) { return m_hIodataUpdate; };
    const KRAMIOTLB_Header *get_iotlbhdr(void) { return &m_HostIOTlbHdr; };

    PNIO_UINT32 init_irt_dma_ranges(void);
    PNIO_UINT32 uninit_irt_dma_ranges(void);

    PNIO_UINT32 CP_cycle_stat(PNIO_CYCLE_STAT * pCycleStat, int MeasureNr);
    PNIO_UINT32 CP_cycle_info(PNIO_CI_ENTRY * pCycleInfoEntry, int MeasureNr, PNIO_UINT32 Offset);
    void        setCycleStat(PNIO_CYCLE_INFO * pCycleInfo, PNIO_CP_CI_TYPE Type);

    DPR_SEMAPHORE m_semSendReceiveSynch;
    bool          m_semSendReceiveSynchWaiterAvailable;

    /* backlink to DPR_ADAPTER */
    DPR_ADAPTER *m_pCpAdapter;
    bool        m_bClosePending;

  protected:

    bool        m_bEmergencyClose;
    bool        m_bController;
    PNIO_UINT32 m_hIodataUpdate;
    PNIO_UINT32 m_hInstanceHandle;
    PNIO_UINT32 m_uOrderId;

    KRAMIOTLB_Item *m_pHostIOTlb;
    KRAMIOTLB_Header m_HostIOTlbHdr;

    PNIO_UINT32      m_cycleCountEntity;
    PNIO_CYCLE_STAT  m_cycleStat;
    PNIO_UINT32      m_ciPoolIndex;
    PNIO_UINT32      m_actCycleCounter;
    PNIO_UINT32      m_prevStartOp;
    CI_POOL_ENTRY    m_ciPool[CI_POOL_ENTRY_NR];
    CI_POOL_EXT_ENTRY m_ciPoolExt[CI_POOL_MEASURE_NR];
    int              m_ciMeasureNr;

  protected:
    void deinit_and_unregister(void);
};


/*  CBSA: Controller Byte Slice Access extensions. PL: 11.01.2010
 */
extern "C" KRAMIOTLB_Item **contrHashTlb;  // def. in krtlb_host.c

/*  KRAM IO Table wrapper: (Host KRAM IO table extension)
 */
class HostKramIOTableExt {

public:
    HostKramIOTableExt(const KRAMIOTLB_Header *pHostIOTlbHeader);
    HostKramIOTableExt(PNIO_UINT32 numOfItems);
    ~HostKramIOTableExt();
    KRAMIOTLB_Item& operator [] (PNIO_UINT32 index);  // get array item
    void updateAddressHash(void);                     // update contrHashTlb
    void addItem (KRAMIOTLB_Item &newItem);           // insert next item
    KRAMIOTLB_Header* getIOTlbHeaderPointer();

private:
    PNIO_UINT32 numItems;         // total num slots in the array
    static const PNIO_UINT32 INVALID_INDEX = 0xFFFFFFFF;
    PNIO_UINT32 lastValidIndex;   // used for array init. (array is filled without gaps)
    KRAMIOTLB_Item* kramItem;     // array of KRAMIOTLB_Items (item storage)

    KRAMIOTLB_Header iotlbExtHdr; // kramItem array header. Required by the KRAMIOTLB_xxx functions

    PNIO_UINT32 calculateNumItems (const KRAMIOTLB_Header *pHostIOTlbHeader);
    void initIotlbExtContent (const KRAMIOTLB_Header *pHostIOTlbHeader);
    void initIotlbExtHdr();       // set current header values
};


/*
 * Class for controller Instances
 */
class IController:public ICommon {
  public:
    enum ICONT_MEDIUM_TYPE {
      ICONT_MT_KRAM,
      ICONT_MT_CACHE
    };

    static PNIO_UINT32 controller_open(PNIO_UINT32 CpIndex,
        PNIO_UINT32 ExtPar,
        PNIO_CBF cbf_RecReadConf,
        PNIO_CBF cbf_RecWriteConf,
        PNIO_CBF cbf_AlarmInd,
        PNIO_UINT32 * ApplHandle);

    PNIO_UINT32 controller_close(void);

    PNIO_UINT32 device_activate(const PNIO_ADDR * pAddr,
        PNIO_DEV_ACT_TYPE mode);

    PNIO_UINT32 perf_io(PNIO_ADDR * pAddr,
        ACC_T accesst,
        PNIO_IOXS * pLocStat,
        PNIO_IOXS * pRemStat,
        PNIO_UINT32 * pDataLen,
        PNIO_UINT8 * pData,
        ICONT_MEDIUM_TYPE Med = ICONT_MT_KRAM);

    void read_cache_refresh();
    void write_cache_flush();

    PNIO_UINT32 perf_ds(PNIO_ADDR * pAddr,
        ACC_T accesst,
        PNIO_REF ReqRef,
        PNIO_UINT32 RecordIndex,
        PNIO_UINT32 Length,
        PNIO_UINT8 * pBuffer);

#ifdef PROFI_ENERGY
    // pe: profi energy handling - see also PNIO_register_pe_cbf()
    PNIO_UINT32 register_pe_cbf(PNIO_PE_CBF cbf);

    PNIO_UINT32 perf_pe(
        PNIO_UINT32      Handle,
        PNIO_ADDR       *pAddr,
        PNIO_REF         ReqRef,
        PNIO_PE_REQ_PRM *pPeReqPrm);
    // pe: end
#endif /* PROFI_ENERGY */

    PNIO_UINT32 set_mode(PNIO_MODE_TYPE Mode);

    PNIO_UINT32 register_cbf(PNIO_CBE_TYPE CbeType,
        PNIO_CBF cbf);

    PNIO_UINT32 alarm_resp(PNIO_REF IndRef);

    PNIO_UINT32 ctrl_diag_req(PNIO_CTRL_DIAG* pDiagReq);

    void create_config_submodules_list(PNIO_UINT32 CycleTime,
        PNIO_UINT32 *pBufferLen, PNIO_UINT8 **ppBuffer);

#ifdef IO_ROUTER
    void create_config_output_slice_list(PNIO_UINT32 *pBufferLen, PNIO_UINT8 **ppBuffer);
    void diag_config_iorouter_present(PNIO_UINT32 *pBufferLen, PNIO_UINT8 **ppBuffer);
    static void transfer_wd_cbf_wrapper(PNIO_CP_CBE_PRM * prm, IController * pThis);
    void transfer_wd_cbf(PNIO_CP_CBE_PRM * prm);
#endif /* IO_ROUTER */

    void ProcDataRecordResponse(const void *pRequest);
    void ProcDiagResponse(const void *pRequest);

    virtual PNIO_UINT32 ProcAlarm(CHANNEL *channel);
    virtual PNIO_UINT32 ProcModeInd(CHANNEL *channel);
    virtual PNIO_UINT32 ProcDataRec(CHANNEL *channel);

    virtual PNIO_UINT32 TestBlockSendReceive(PNIO_UINT8 *buffer,
                                               PNIO_UINT32 length);
    virtual PNIO_UINT32 TestIO(PNIO_UINT32 area_idx, PNIO_UINT32 operation,
                                 PNIO_UINT32 Length, PNIO_UINT8 *Buffer);

    virtual PNIO_UINT32 RegisterStartOp(void);

#ifdef PROFI_ENERGY
    friend  cPeMgt::cPeMgt(IController*); // Global friend  ???
    cPeMgt  *m_pPeMgt;          // ptr to pe class
#endif /* PROFI_ENERGY */

private:
    IController(void);

    /* misc bits */
    PNIO_CBF m_pUserCbf_Mode;
    PNIO_CBF m_pUserCbf_DevAct;
    PNIO_CBF m_pUserCbf_Alarm;
    PNIO_CBF m_pUserCbf_DR_Read;
    PNIO_CBF m_pUserCbf_DR_Write;
    PNIO_CBF m_pCbf_DsReadConf;
    PNIO_CBF m_pCbf_DsWriteConf;
    PNIO_CBF m_pCbf_AlarmInd;
    PNIO_CBF m_pUserCbf_CpStopReq;
    PNIO_CBF m_pUserCbf_StartLedFlash;
    PNIO_CBF m_pUserCbf_StopLedFlash;
    PNIO_CBF m_pUserCbf_CtrlGetDiag;

    PNIO_MODE_TYPE m_Mode;      /* Actual mode */
    PNIO_UINT32  m_OpenExtPar;

    /* for read cache */
    PNIO_UINT8 *m_pInCache;
    IODU_CONS_BND *m_CnsBndInArr;
    PNIO_UINT32 m_CnsBndInArrLen;

    /* for write cache */
    PNIO_UINT8 *m_pOutCache;           /* array of PNIO_UINT8 */
    IODU_CONS_BND *m_CnsBndOutArr; /* array of IODU_CONS_W_BOUND */
    PNIO_UINT32 m_CnsBndOutArrLen;

    /* byte slice access extensions */
    HostKramIOTableExt  *m_pHostKramIOTlbExt;  // ptr to KramIOTlb wrapper
    //void create_HostKramIOTlbExt(void);

#ifdef IO_ROUTER
    //friends of class IoConcentrator;
    friend PNIO_UINT32 IoConcentrator::IOC_data_write(
        PNIO_UINT32 LogAddr, IOR_CONCENTRATOR_ENTRY* pSubMod,
        PNIO_UINT32 BufLen, PNIO_UINT8 * pBuffer,
        PNIO_IOXS IOlocState, PNIO_IOXS *pIOremState);
    friend PNIO_UINT32 IoConcentrator::IOC_data_write_cache (
        PNIO_UINT32 LogAddr, IOR_CONCENTRATOR_ENTRY* pSubMod,
        PNIO_UINT32 BufLen, PNIO_UINT8 * pBuffer,
        PNIO_IOXS IOlocState, PNIO_IOXS *pIOremState);
    friend PNIO_UINT32 IoConcentrator::IOC_update_cache ( void );
    friend int IoConcentrator::ioConcentratorInit(IController *pICtrl, CTransferWD *pTWDg);
    IoConcentrator      m_Ioc;             /* Io concentrator object */
    CTransferWD         m_TransferWD;      /* Transfer Watchdog Object */
#endif /* IO_ROUTER */

};

/*
 * Class for Device Instances
 */
class IDevice:public ICommon {
  public:

    static PNIO_UINT32 device_open(PNIO_UINT32 CpIndex,
        PNIO_UINT32 ExtPar,
        PNIO_UINT16 VendorId,
        PNIO_UINT16 DeviceId,
        PNIO_UINT16 InstanceId,
        PNIO_UINT32 MaxAR,
        PNIO_ANNOTATION * pDevAnnotation,
        PNIO_CFB_FUNCTIONS *pCbf,
        PNIO_UINT32 * pDevHndl);

    PNIO_UINT32 set_appl_state_ready(PNIO_UINT16 ArNumber,
        PNIO_UINT16 SessionKey, PNIO_APPL_READY_LIST_TYPE *pList);

    PNIO_UINT32 device_ar_abort(PNIO_UINT16 ArNumber,
        PNIO_UINT16 SessionKey);

    PNIO_UINT32 device_close(void);

    PNIO_UINT32 device_start(void);

    PNIO_UINT32 device_stop(void);

    PNIO_UINT32 set_dev_state(PNIO_UINT32 DevState);

    PNIO_UINT32 api_add(PNIO_UINT32 Api,
        PNIO_UINT16 MaxnumSlots, PNIO_UINT16 MaxnumSubslots);

    PNIO_UINT32 api_remove(PNIO_UINT32 Api);

    PNIO_UINT32 mod_pull(PNIO_UINT32 Api, PNIO_DEV_ADDR * pAddr);

    PNIO_UINT32 sub_pull(PNIO_UINT32 Api, PNIO_DEV_ADDR * pAddr);

    PNIO_UINT32 sub_plug(PNIO_UINT32 Api, PNIO_DEV_ADDR * pAddr,
        PNIO_UINT32 SubIdent);

    PNIO_UINT32 sub_plug_ext(PNIO_UINT32 Api, PNIO_DEV_ADDR * pAddr,
        PNIO_UINT32 SubIdent, PNIO_UINT32 AlarmType);

    PNIO_UINT32 sub_plug_ext_im(PNIO_UINT32 Api, PNIO_DEV_ADDR * pAddr,
        PNIO_UINT32 SubIdent, PNIO_UINT32 AlarmType, PNIO_PLUG_IM0_BITS  IM0_bits);

    PNIO_UINT32 mod_plug(PNIO_UINT32 Api, PNIO_DEV_ADDR * pAddr,
        PNIO_UINT32 ModIdent);

    PNIO_UINT16 build_channel_properties(PNIO_UINT16 Type,
        PNIO_UINT16 Spec, PNIO_UINT16 Dir);

    PNIO_UINT32 diag_remove(PNIO_UINT32 Api,
        PNIO_DEV_ADDR *pAddr, PNIO_UINT16 DiagTag, PNIO_UINT8 DiagType);

    PNIO_UINT32 diag_channel_add(PNIO_UINT32 Api,
        PNIO_DEV_ADDR *pAddr, PNIO_UINT16 ChannelNum,
        PNIO_UINT16 ChannelProp, PNIO_UINT32 ChannelErrType,
        PNIO_UINT16 DiagTag);

    PNIO_UINT32 diag_ext_channel_add(PNIO_UINT32 Api,
        PNIO_DEV_ADDR *pAddr, PNIO_UINT16 ChannelNum,
        PNIO_UINT16 ChannelProp, PNIO_UINT32 ChannelErrType,
        PNIO_UINT16 ExtChannelErrType, PNIO_UINT32 ExtChannelAddValue,
        PNIO_UINT16 DiagTag);

    PNIO_UINT32 diag_generic_add(PNIO_UINT32 Api,
        PNIO_DEV_ADDR *pAddr, PNIO_UINT16 ChannelNum,
        PNIO_UINT16 ChannelProp, PNIO_UINT16 DiagTag,
        PNIO_UINT16 UserStructIdent, PNIO_UINT8 *pInfoData,
        PNIO_UINT32 InfoDataLen);

    PNIO_UINT32 process_alarm_send(PNIO_UINT32 Api,
        PNIO_UINT16 ArNumber, PNIO_UINT16 SessionKey,
        PNIO_DEV_ADDR *pAddr, PNIO_UINT8 *pData,
        PNIO_UINT32 DataLen, PNIO_UINT16 UserStructIdent,
        PNIO_UINT32 UserHndl);

    PNIO_UINT32 diag_alarm_send(PNIO_UINT32 Api,
        PNIO_UINT16 ArNumber, PNIO_UINT16 SessionKey,
        PNIO_UINT32 AlarmState, PNIO_DEV_ADDR *pAddr, PNIO_UINT8 *pData,
        PNIO_UINT32 DataLen, PNIO_UINT16 UserStructIdent,
        PNIO_UINT32 UserHndl);

    PNIO_UINT32 ret_of_sub_alarm_send(PNIO_UINT32 Api,
        PNIO_UINT16 ArNumber, PNIO_UINT16 SessionKey,
        PNIO_DEV_ADDR *pAddr, PNIO_UINT32 UserHndl);

    PNIO_UINT32 initiate_data_read(PNIO_UINT32 ApplHandle);

    PNIO_UINT32 initiate_data_read_ext(PNIO_UINT32 ApplHandle,
        PNIO_DEV_ADDR *pAddr, PNIO_ACCESS_ENUM AccessType);

    PNIO_UINT32 initiate_data_write(PNIO_UINT32 ApplHandle);

    PNIO_UINT32 initiate_data_write_ext(PNIO_UINT32 ApplHandle,
        PNIO_DEV_ADDR *pAddr, PNIO_ACCESS_ENUM AccessType);

    virtual PNIO_UINT32 ProcAlarm(CHANNEL *channel);
    virtual PNIO_UINT32 ProcModeInd(CHANNEL *channel);
    virtual PNIO_UINT32 ProcDataRec(CHANNEL *channel);

    virtual PNIO_UINT32 TestBlockSendReceive(PNIO_UINT8 *buffer,
        PNIO_UINT32 length);

    virtual PNIO_UINT32 RegisterStartOp(void);

	static bool ar_has_slot_0_pluged;
	static bool ctrl_wants_slot_0;
	static bool do_pdev_ioxs_check;
        static PNIO_UINT32 slot0_modid_pluged;
        static PNIO_UINT32 slot0_modid_ctrl;
        static PNIO_UINT32 slot1_modid_pluged;
        static PNIO_UINT32 slot1_modid_ctrl;
	
	
  private:
    IDevice(void);

	static PNIO_IOXS    CBF_GET_INITIAL_IOXS(PNIO_UINT32          DevHndl, PNIO_DEV_ADDR      * pAddr); 
	
    /* misc bits */
    PNIO_UINT32  m_OpenExtPar;

    PNIO_CFB_FUNCTIONS m_Cbfs;
	
	
};

PNIO_UINT32 ConvertAgentErrorCodeToApiErrorCode(PNIO_UINT32 InErr);
PNIO_UINT32 ConvertDAgentErrorCodeToApiErrorCode(PNIO_UINT32 InErr);

/*
 * global methode to avoid Static mutexes for VxWorks
 */
void InitCriticalSections(void);
void DestroyCriticalSections(void);

#endif /* _PNIO_PNIOINTR_H */

