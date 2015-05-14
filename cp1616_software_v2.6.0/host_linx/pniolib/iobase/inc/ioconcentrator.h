/****************************************************************************
*  Copyright (C) SIEMENS CORP., 2013 All rights reserved.
*****************************************************************************
*         FILE : ioconcentrator.h
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

#ifndef _PNIO_IOCONCENTRATOR_H
#define _PNIO_IOCONCENTRATOR_H
#ifdef IO_ROUTER                   /* for the io-router only  */

#include "os.h"
#include "ior_dpram.h"


void print_buffer(const unsigned char *buf, unsigned int length);  // dbg aim

class IController;
class CTransferWD;

/* Io concentrator
*/
class IoConcentrator
{
public:
    IoConcentrator(void);
    ~IoConcentrator(void);

    typedef enum {
        IO_IN   = 1,
        IO_OUT  = 2,
        IO_PDEV = 4,
        IO_DDEX = 8
    } IN_OUT_TYPE;

    typedef struct
    {
        PNIO_UINT32 log_addr;
        IN_OUT_TYPE in_out_type;
    }
    PURE_MODULE_INFO;

    typedef struct
    {
        IOR_CONCENTRATOR_ENTRY  * pTransferData;
        PNIO_IOXS                 kramUserStatus;
        PNIO_IOXS                 cacheUserStatus;
    }
    IOC_MODULE_INFO;

    int  ioConcentratorInit(IController *pICtrl, CTransferWD *pTWDg);
    void ioConcentratorUnInit(void);
    PNIO_UINT32 IOC_data_write(
        PNIO_UINT32 LogAddr, IOR_CONCENTRATOR_ENTRY* pSubMod,
        PNIO_UINT32 BufLen, PNIO_UINT8 * pBuffer,
        PNIO_IOXS IOlocState, PNIO_IOXS *pIOremState
    );
    PNIO_UINT32 IOC_data_write_cache (
        PNIO_UINT32 LogAddr, IOR_CONCENTRATOR_ENTRY* pSubMod,
        PNIO_UINT32 BufLen, PNIO_UINT8 * pBuffer,
        PNIO_IOXS IOlocState, PNIO_IOXS *pIOremState
    );
    PNIO_UINT32 IOC_update_cache ( void );
    PNIO_UINT32 IOC_watchdog_data_write ( PNIO_UINT32 LogAddr );

    int ioDataMerge (
        PNIO_UINT32 LogAddr, IOR_CONCENTRATOR_ENTRY* pSubMod,
        PNIO_UINT8 *pSrcBuffer, PNIO_UINT32 SrcBuffLen,
        PNIO_UINT8 *pDstBuffer, PNIO_UINT32 *DstBuffLen
    );

    int                      getNumOfIocModules(void);
    PNIO_UINT32              getNumOfPureTransferModules();
    PNIO_UINT32              getNumOfIocHostModules();
    int                      getNumOfSlices(IOR_CONCENTRATOR_ENTRY* pSubMod);
    int                      getSliceBitOffset(IOR_OUTPUT_SLICE* pSlice);
    int                      getSliceBitLength(IOR_OUTPUT_SLICE* pSlice);
    IOR_OUTPUT_SLICE*        getSliceAtIdx (IOR_CONCENTRATOR_ENTRY* pSubMod, int idx);
    IOR_CONCENTRATOR_ENTRY*  getIocHostModule(PNIO_UINT32 LogAddr);
    IOR_CONCENTRATOR_ENTRY*  getIocModule(PNIO_UINT32 LogAddr);
    IOR_CONCENTRATOR_ENTRY*  getIocModuleAtIdx(int idx);
    PNIO_UINT32              getLogAddrMax(void)
    {
        return m_hashAddrMax;                  // ret lagest configured log_addr (used for iteration)
    }
    void                     getPureTransferModuleList(PURE_MODULE_INFO * pAddrList, PNIO_UINT32 numOfItems);
    PNIO_UINT32              getIocModAddrMax(bool FilterPureModules = false);

    PNIO_UINT32              getMinTwdTime(void);
    PNIO_UINT32              getTwdTime(PNIO_UINT32 idx);
    bool                     getTwdValidity(void);
    IOR_CONCENTRATOR_HEADER* getIocTabHdr(void)
    {
        return m_pIocTabHdr;                   // ptr to ioc table (user space copy)
    }

    IOR_CONCENTRATOR_HEADER* getShmTabHdr(void)
    {
        return m_pShmTabHdr;                   // ptr to the io concentrator table (shared mem,original)
    }

    void TraceConcentratorTable(IOR_CONCENTRATOR_HEADER * pTableHeader);

    void updateUserStatusFromCache(void);

private:
    CTransferWD             *m_pTWDg;          // link to the transfer watchdog
    IController             *m_pICtrl;         // backlink to the owner (IController)
    IOR_CONCENTRATOR_HEADER *m_pShmTabHdr;     // ptr to the shared memory table (copy of the DPR_ADAPTER member)
    IOR_CONCENTRATOR_HEADER *m_pIocTabHdr;     // ptr to the local copy of the ioc table (offset 0)
    int                      m_iocTabSize;
    IOC_MODULE_INFO        **m_pIocAddrHash;   // pointer to array of pointers,  log_addr == array index
    IOC_MODULE_INFO         *m_pIocIdxTable;   // array of module infos
    PNIO_UINT32              m_hashAddrMax;    // array limit, max index
    bool                     m_moduleSliceAcessIsOn; // CBSA: Controller Byte Slice Access extension is ON flag

    int  calculateIocTabSize(IOR_CONCENTRATOR_HEADER *pIocTabHdr);
    int  copyAndSwapConcentratorTable (IOR_CONCENTRATOR_HEADER* m_pIocTabHdr,
                                       IOR_CONCENTRATOR_HEADER* pDprHdr);
    int  initAddrHash(void);
    void unInitAddrHash(void);
    int  getShadowDataPtrAndLength (IOR_CONCENTRATOR_ENTRY* pSubMod,
                                    PNIO_UINT8 **dataPtr,
                                    PNIO_UINT32 *dataLen
                                   );
    PNIO_UINT8  getShadowIoxs (IOR_CONCENTRATOR_ENTRY* pSubMod);
    void setShadowHostStatus (PNIO_UINT32 LogAddr, PNIO_IOXS status);

    void updateUserStatus(PNIO_UINT32 LogAddr, PNIO_IOXS status);
    void getUserStatus(PNIO_UINT32 LogAddr, PNIO_IOXS * pStatus);
    void cacheStatusToStore(PNIO_UINT32 LogAddr, PNIO_IOXS status);
    void cacheStatusFromStore(PNIO_UINT32 LogAddr, PNIO_IOXS * pStatus);

}
; /* IoConcentrator */


#endif /* IO_ROUTER */
#endif /* _PNIO_IOCONCENTRATOR_H */

