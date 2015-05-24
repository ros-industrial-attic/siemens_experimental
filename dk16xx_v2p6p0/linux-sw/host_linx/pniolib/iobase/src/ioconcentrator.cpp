/****************************************************************************
*  Copyright (C) SIEMENS CORP., 2013 All rights reserved.
*****************************************************************************
* FILE NAME    : ioconcentrator.cpp
* ---------------------------------------------------------------------------
* DESCRIPTION  : IO-Route concentrator implementations
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
#ifdef IO_ROUTER        /* for io-router only */

#include "ioconcentrator.h"
#include "pniointr.h"


/*-----------------------------------------------------------------------------
 * ioconcentrator  ctor, dtor
 */
IoConcentrator::IoConcentrator(void)
{
    m_pTWDg        = NULL;
    m_pICtrl       = NULL;
    m_pShmTabHdr   = NULL;
    m_pIocTabHdr   = NULL;
    m_iocTabSize   = 0;
    m_pIocAddrHash = NULL;
    m_pIocIdxTable = NULL;
    m_hashAddrMax  = 0;
    return;
} /* end of ctor */

IoConcentrator::~IoConcentrator(void)
{
    ioConcentratorUnInit();
    return;
} /* end of ctor */

/*-----------------------------------------------------------------------------
 * Name  : IoConcentrator::ioConcentratorInit   /   UnInit()
 * Descr : Creates a user space copy of the shmem concentrator table =>
 *         Pointer to the shmem table must be already mapped to the user space.
 *         See creation of the DPR_ADAPTER in ICommon::InitCp
 * Param :
 *  [ IN]: pICtrl: owner class, required for access to the controller objets.
 *  [ IN]: pTWDg : Transfer watchDog object. (kram + cache uptodate flags)
 *  [OUT]: -
 * Return: ==0: ok, != 0 error
 */
int IoConcentrator::ioConcentratorInit(IController *pICtrl, CTransferWD *pTWDg)
{
    m_pICtrl = pICtrl;  // set backlink to the owner IController
    m_pTWDg  = pTWDg;   // link to transfer watchdog
                        // CBSA: Controller Byte Slice Access extension flag is saved in IoConcentrator obj.
    m_moduleSliceAcessIsOn = (m_pICtrl->m_OpenExtPar & PNIO_CEP_SLICE_ACCESS) ? true : false;

    // use shared memory tab initiated (delivered) by the firmware
    // save copy of the shared memory pointer for faster access
    m_pShmTabHdr = (IOR_CONCENTRATOR_HEADER *) m_pICtrl->m_pCpAdapter->pShMemIocTable;
    TRC_OUT01(GR_INIT, LV_FCTINT, "->IOC: ioConcentratorInit pSh=%#x", m_pShmTabHdr);
    DPR_ASSERT(m_pShmTabHdr);

    int requiredMemSize = calculateIocTabSize((IOR_CONCENTRATOR_HEADER *)m_pShmTabHdr);
    TRC_OUT02(GR_INIT, LV_FCTINT, "IOC: currTab: Ptr=%p Size=%d",m_pShmTabHdr,requiredMemSize);

    if ( requiredMemSize == 0 ) {
        // concentrator table not present, empty or invalid => no routing
        TRC_OUT(GR_INIT, LV_WARN, "IOC: ioConcentratorInit NO IOCTAB");
        return -1;
    }

    m_pIocTabHdr = (IOR_CONCENTRATOR_HEADER*)malloc(requiredMemSize);
    if (!m_pIocTabHdr) {
        // no mem
        TRC_OUT01(GR_INIT, LV_ERR, "IOC: Error ioConcentratorInit no mem. req=%d",
            requiredMemSize);
        return -1;
    }

    // copy shmem table to the local space for faster access
    // memcpy(m_pIocTabHdr, m_pShmTabHdr, requiredMemSize);
    copyAndSwapConcentratorTable(m_pIocTabHdr, m_pShmTabHdr);  // dst, src

    TraceConcentratorTable(m_pIocTabHdr); // trace output level is checked inside

    if ( initAddrHash() != 0) {
        free(m_pIocTabHdr);
        m_pIocTabHdr = NULL;
        return -1;
    }
    TRC_OUT03(GR_INIT, LV_FCTINT, "<-IOC: ioConcentratorInit OK pIocTab=%#x pHash=%#x maxAddr=%d",
        m_pIocTabHdr, m_pIocAddrHash, m_hashAddrMax);
    return 0; // ok
} /* end of IoConcentrator::ioConcentratorInit */

void IoConcentrator::ioConcentratorUnInit (void)
{
    if (m_pIocTabHdr) {
        free(m_pIocTabHdr);
        m_pIocTabHdr = NULL;
    }
    unInitAddrHash();
    return;
} /* end of IoConcentrator::ioConcentratorUnInit */

/*-----------------------------------------------------------------------------
 * Name  : calculateIocTabSize
 * Descr : Fixed mem area of MMAP_IOCTABLE_SIZE is reserved in the shared memory
 *         Current size of the table must be calculated. Shared memory table is
 *         used for analysis, user ram copy is not existent at this point! =>
 *         use SWAP_D macro to access table elements !!!
 * Param :
 *  [ IN]: pIocTabHdr: Pointer to the shared memory table (source tab)
 *  [OUT]:
 * Return: Size of the table in bytes, zero: empty or invalid table present.
 */
int IoConcentrator::calculateIocTabSize(IOR_CONCENTRATOR_HEADER *pIocTabHdr)
{
    volatile PNIO_UINT32 valid;
    int tabSize = 0;
    volatile IOR_CONCENTRATOR_HEADER *pH = pIocTabHdr;

    // make some consistency checks
    if (pH->Validity == 0) {
        TRC_OUT(GR_INIT, LV_WARN, "IOC: IocTab empty => NO ROUTING");
        return 0; // empty
    } else {
        // ok
        valid = pH->Validity;
    }
    if ( SWAP_D(pH->ConEntriesOff) != sizeof(IOR_CONCENTRATOR_HEADER)  ) {
        TRC_OUT(GR_INIT, LV_ERR, "IOC: ERROR ShMemTab INVALID");
        return 0; // invalid
    }
    if ( SWAP_D(pH->NumOfConEntries) == 0 ) {
        TRC_OUT(GR_INIT, LV_WARN, "IOC: No entries in IocTab => NO ROUTING");
        return 0; // no entries in table
    }
    tabSize = sizeof(IOR_CONCENTRATOR_HEADER) +
              (SWAP_D(pH->NumOfConEntries) * sizeof(IOR_CONCENTRATOR_ENTRY)) +
              (SWAP_D(pH->TotalNumOfSlices) * sizeof(IOR_OUTPUT_SLICE));

    if ( valid != pH->Validity ) {
        // should never occure, race conditions, table was changed by the firmware?
        TRC_OUT(GR_INIT, LV_ERR, "IOC: ERROR ShMemTab changed");
    }
    return tabSize;

} /* end of calculateIocTabSize */

/*-----------------------------------------------------------------------------
 * Name  : initAddrHash
 * Descr : Creates a index table for fast access to a submodule.
 *         LogAddr -> IOR_CONCENTRATOR_ENTRY*
 *         Calls getIocModAddrMax() saves the returned value in m_hashAddrMax
 *   Note: This function is called by ioConcentratorInit(). User space ioc table
 *         must be already created before calling this function!
 *   Note: PL: 11.02.2011:  CBSA= Controller Byte Slice Access extension ==
 *         Write access to one or more bytes inside the submodule are allowed.
 *         I.e. The LogicaAddress can be any address of the submodule. =>
 *         IocAddrHash size is increased by the length of the last submodule.
 *         (submodule with the largest logical address == idxMax).
 * Param : -
 * Return: ==0: ok, != 0 error
 */
int IoConcentrator::initAddrHash(void)
{
    PNIO_UINT32 idxMax = getIocModAddrMax();   // idxMax == greatest logical address

    TRC_OUT01(GR_INIT, LV_FCTINT, "->IOC: initAddrHash: current MaxLogAddr=%u ",idxMax);

    if ( m_moduleSliceAcessIsOn ) {
        // module slice acces is allowed == all module addresse are valid for write
        // we enlarge the max allowed logical address by the size of the last submod.
        // Example:1) we have only one subModule 1 byte long at addr. 0
        //            idxMax=0, subModSize=1 => no enlargement, idxMax stay the same
        //         2) only one subModule at addr 0 2 bytes long:
        //            idxMax=0, subModSize=2 => enlargement by 1, => idxMax=1 =>
        //            two valid addresses: 0, 1.
        //
        IOR_CONCENTRATOR_ENTRY *pSubMod = getIocModuleAtIdx(idxMax);
        if (!pSubMod) {
            TRC_OUT01(GR_INIT, LV_ERR, "IOC: ERROR initAddrHash PROG-ERROR idxMax=%d",idxMax);
            return -1;
        }
        int subModSize = (pSubMod->ShadowLength > (PNIO_UINT32)0) ? pSubMod->ShadowLength -1 : 0;
        idxMax += subModSize;
        TRC_OUT01(GR_INIT, LV_FCTINT, "IOC: initAddrHash: moduleSliceAcessIsOn New MaxLogAddr=%u ",idxMax);
    }
    // allocate address table
    unsigned int addrTableSize = ( (idxMax + 1) * sizeof(IOC_MODULE_INFO*) );    // size of a pointer list
    m_pIocAddrHash = (IOC_MODULE_INFO**)malloc(addrTableSize);
    if (!m_pIocAddrHash) {
        // no mem
        TRC_OUT01(GR_INIT, LV_ERR, "IOC: ERROR initAddrHash no mem. req=%d",addrTableSize);
        return -1;
    }
    memset(m_pIocAddrHash, 0, addrTableSize);

    // allocate index table
    unsigned int idxTableSize = sizeof(IOC_MODULE_INFO) * getNumOfIocModules();
    if (idxTableSize > 0) {
        m_pIocIdxTable = (IOC_MODULE_INFO*)malloc(idxTableSize);
        if (!m_pIocIdxTable) {
            // no mem
            TRC_OUT01(GR_INIT, LV_ERR, "IOC: ERROR initAddrHash, m_pIocIdxTable no mem. req=%d",idxTableSize);
            unInitAddrHash();
            return -1;
        }
        memset(m_pIocIdxTable, 0, idxTableSize);
    }

    // set member max index for later access check
    m_hashAddrMax = idxMax;

    // for all submodules: set submod ptr at array[LogAddr]
    IOR_CONCENTRATOR_ENTRY *pSubMod = getIocModuleAtIdx(0);
    int i = 0;
    int j = 0;
    while(pSubMod) {
        // determine adress of index table entry
        IOC_MODULE_INFO* pIdxTabEntry = &(m_pIocIdxTable[i]);
        // set module info struct
        pIdxTabEntry->pTransferData = pSubMod;
        pIdxTabEntry->kramUserStatus  = PNIO_S_BAD;
        pIdxTabEntry->cacheUserStatus  = PNIO_S_BAD;
        // link adress table entry to module info. i.e.: set addrHash entry to point
        // to the moduleInfo in the IocIndexTble.
        m_pIocAddrHash[pSubMod->LogAddr] = pIdxTabEntry;
        if ( m_moduleSliceAcessIsOn ) {
            // we check all successive subModul log.addresses end set the corresponding addrHash entries
            // Now we set all successive subModul log.addresses to the same IndexTabEntry. => we 'merge'
            //     all bytes, i.e. bytes without shared bits too. See IOC_data_write()
            // Next valid submod addr. ist pSubMod->LogAddr +1,+1... if subModSize is > 1.
            //
            // Note (possible Optimization): if the byte log. addr is one of the shared byte,
            //   i.e. io-data-write requieres merge, we set this addr entry in the AddrHash, otherwise
            //   we don't set the addr. => write access goes NOT thrue the concentrator. It is handled
            //   as normal PNIO_write acces. See perf_io() call of m_Ioc.getIocModule.
            //   But in this case we would NOT merge data status ??? Not a gut solution, but faster
            //   PL: 22.02.2011: this optimization is not used, we put all byte addresses of the subModule
            //   into the AddrHash table.
            //
            int          subModSize = pSubMod->ShadowLength;
            PNIO_UINT32  logAddr    = pSubMod->LogAddr;

            for (j=0; j < (subModSize - 1); j++) {
                m_pIocAddrHash[logAddr + j] = pIdxTabEntry;
            }
        }

        // next module  in concentrator table
        pSubMod = getIocModuleAtIdx(++i);
    } // end while(pSubMod)

    TRC_OUT02(GR_INIT, LV_FCTINT, "<-IOC: initAddrHash: TotalSubMods=%d MaxAddr=%u ",i,m_hashAddrMax);
    return 0;
} /* end of initAddrHash */

/*-----------------------------------------------------------------------------
 * Name  : unInitAddrHash
 * Descr :
 * Param :
 *  [ IN]:
 *  [OUT]:
 * Return: ==0: ok, != 0 error
 */
void IoConcentrator::unInitAddrHash(void)
{
    if (m_pIocAddrHash) {
        free(m_pIocAddrHash);
        m_pIocAddrHash = NULL;
    }
    if (m_pIocIdxTable) {
        free(m_pIocIdxTable);
        m_pIocIdxTable = NULL;
    }
    return;
} /* end of unInitAddrHash */

/*-----------------------------------------------------------------------------
 * Name  : IOC_data_write
 * Descr : User buffer slices are merged with the shadow buffer.
 *         User buffer contains the IO-Router Controller apllication 'OUTPUT DATA'.
 *         Shadow buffer contains all IO-Router Device data which are tobe routed.
 *         LocalIOState is merged with the "shadow IOState".
 *         Finaly merged data + merged IOState are written into the KRAM by the
 *         IODU_ctrl_data_write() call.
 *
 * IMPORTANT: PL: 11.02.2011: CBSA changes. Controller Byte Slice Access extensions.
 *         See commenst in function code below.
 * Param :
 *  [ IN]: LogAddr: 1) Logical addres of subModule begin. 1st addr of subModul or
 *                  2) If Controller Byte Slice Access extension (CBSA) is ON
 *                     it can be any logical addr. of the subModule.
 *  [ IN]: pSubMod: Pointer to the concetrator table. Note: IOR_CONCENTRATOR_ENTRY
 *                  contains always the beginning address of subModule.
 *  [ IN]: UsrBufLen : Length of the io-data to be written (number ob bytes).
 *  [ IN]: pUsrBuffer: Pointer to the io-data to be written.
 *  [ IN]: IOlocState : provider state
 *  [OUT]: pIOremState: consumer state
 *
 * Return: PNIO_OK ==0: ok, != PNIO_OK errors
 */
PNIO_UINT32 IoConcentrator::IOC_data_write(PNIO_UINT32 LogAddr, IOR_CONCENTRATOR_ENTRY* pSubMod,
        PNIO_UINT32 UsrBufLen, PNIO_UINT8 * pUsrBuffer,
        PNIO_IOXS IOlocState, PNIO_IOXS *pIOremState)
{
    PNIO_UINT8  dstBuff[PNIO_MAX_IO_LEN];
    PNIO_UINT32 dstBuffLen = sizeof(dstBuff);
    PNIO_UINT32 IoduRes;
    *pIOremState = PNIO_S_BAD;
    PNIO_UINT32 wrLen      = UsrBufLen;  // IO-Router Controller appl. write io-data length
    PNIO_UINT8 *pwrData    = pUsrBuffer; // ptr to the io-data to be written
    PNIO_UINT8  tmpBuff[PNIO_MAX_IO_LEN];
    PNIO_UINT32 tmpBuffLen = sizeof(tmpBuff);
    PNIO_UINT32 tmpReadLen = 0;               // current KRAM read len [out]
    KRAMIOTLB_Item  *pItem = NULL;
    IODU_Item       *pIODUItem = m_pICtrl->m_pCpAdapter->pIODUItem;
    PNIO_UINT32 LogAddr1st = pSubMod->LogAddr; // 1st logical address of the subModul (subModul begin)
                                               // required because of CBSA -> LogAddr is not begin addr.

    // 1st check num of slices for this submodule, if zero: user is not allowed to write
    // any data to this output address -> return error
    if ( getNumOfSlices(pSubMod) <= 0 ) {
        TRC_OUT01(GR_IOR, LV_ERR, "IOC: ERROR write: SLICEs=ZERO addr=%u",pSubMod->LogAddr);
        return PNIO_ERR_PRM_ROUTER_ADD;
    }

    // trace user data
    TRC_OUT04(GR_IOR, LV_TIMECRIT, "IocW:USER: Addr=%u Len=%d SubModAddr=%u IOPS=%d ",LogAddr, wrLen, pSubMod->LogAddr, IOlocState);

    // Store local status for later use by:
    // - transfer watchdog
    // - PNIO_data_write_cache_flush
    updateUserStatus(LogAddr1st, IOlocState);     // works
    cacheStatusToStore(LogAddr1st, IOlocState);   // works for CBSA too, ok

    // merge user write data with io-router device data
    ioDataMerge (LogAddr, pSubMod, pUsrBuffer, UsrBufLen, &dstBuff[0], &dstBuffLen);

    // merge local iox status:  PNIO_S_GOOD | PNIO_S_BAD
    PNIO_UINT8 shadowIoxs = getShadowIoxs(pSubMod);
    PNIO_IOXS IOmergedLocState = (PNIO_IOXS)((PNIO_UINT8)IOlocState | shadowIoxs);

    // trace merged local state  i.e. IOPS (provider state)
    TRC_OUT03(GR_IOR, LV_TIMECRIT, "IocW:MRGD: Addr=%u Len=%d IOPS=%d ",LogAddr, dstBuffLen, IOmergedLocState);

    IoduRes = IODU_ctrl_data_write(m_pICtrl->m_pCpAdapter->pIODUItem,
                                   0x8000 | LogAddr,                   /* any subModulAddr: data are written to */
                                   dstBuffLen,                         /* set by ioDataMerge() */
                                   &dstBuff[0],                        /* data to be written   */
                                   IOmergedLocState, pIOremState,      /* provider state, consumer state */
                                   m_pICtrl->m_pOutCache,
                                   m_pICtrl->m_CnsBndOutArr ? m_pICtrl->m_CnsBndOutArr[0].l_offset : 0);

    if ( IoduRes == PNIO_OK) {
        // set uptodate flags used by the watchdog
        m_pTWDg->KramSet(LogAddr1st);
        m_pTWDg->CacheSet(LogAddr1st);
    }

    if( IoduRes == PNIO_OK && IOlocState == PNIO_S_GOOD && IOmergedLocState == PNIO_S_BAD)
    {
      TRC_OUT01(GR_IOR, LV_WARN, "Not all slice have local state PNIO_S_GOOD => written IOPS= BAD LogAddr1=%u", LogAddr1st);
      return PNIO_WARN_LOCAL_STATE_BAD;
    }
    else
    {
      return IoduRes;
    }

} /* end of IOC_data_write */

/*-----------------------------------------------------------------------------
 * Name  : IOC_data_write_cache
 * Descr :
 * Param :
 *  [ IN]:
 *  [OUT]:
 * Return: PNIO_OK ==0: ok, != PNIO_OK errors
 */
PNIO_UINT32 IoConcentrator::IOC_data_write_cache (PNIO_UINT32 LogAddr, IOR_CONCENTRATOR_ENTRY* pSubMod,
        PNIO_UINT32 UsrBufLen, PNIO_UINT8 * pUsrBuffer,
        PNIO_IOXS IOlocState,  PNIO_IOXS * pIOremState)
{
    PNIO_UINT8  dstBuff[PNIO_MAX_IO_LEN];
    PNIO_UINT32 dstBuffLen = sizeof(dstBuff);
    PNIO_UINT32 IoduRes;
		*pIOremState = PNIO_S_BAD;

    // 1st check num of slices for this submodule, if zero: user is not allowed to write
    // any data to this output address -> return error
    if ( getNumOfSlices(pSubMod) <= 0 ) {
        TRC_OUT01(GR_IOR, LV_ERR, "IOC: ERROR write_cache: SLICEs=ZERO addr=%u",pSubMod->LogAddr);
        return PNIO_ERR_PRM_ROUTER_ADD;
    }

    // trace user data
    TRC_OUT04(GR_IOR, LV_TIMECRIT, "IoWC:USER: Addr=%u w1:%#x Len=%d IOS=%d ",LogAddr,pUsrBuffer,UsrBufLen,IOlocState);

    // store local status
    cacheStatusToStore(LogAddr, IOlocState);

    ioDataMerge (LogAddr, pSubMod, pUsrBuffer, UsrBufLen, &dstBuff[0], &dstBuffLen);

    // merge local iox status:  PNIO_S_GOOD | PNIO_S_BAD
    PNIO_UINT8 shadowIoxs = getShadowIoxs(pSubMod);
    PNIO_IOXS IOmergedLocState = (PNIO_IOXS)((PNIO_UINT8)IOlocState | shadowIoxs);

    // trace merged data
    TRC_OUT04(GR_IOR, LV_TIMECRIT, "IoWC:MRGD: Addr=%u w1:%#x Len=%d IOS=%d ",
        LogAddr, &dstBuff[0], dstBuffLen, IOmergedLocState);

    IoduRes = IODU_ctrl_data_write_ex(m_pICtrl->m_pCpAdapter->pIODUItem, 0x8000 | LogAddr,
                                      dstBuffLen, &dstBuff[0], IOmergedLocState, pIOremState,
                                      m_pICtrl->m_CnsBndInArr  ? m_pICtrl->m_CnsBndInArr[0].l_offset : 0,
                                      m_pICtrl->m_CnsBndOutArr ? m_pICtrl->m_CnsBndOutArr[0].l_offset : 0,
                                      m_pICtrl->m_pInCache, m_pICtrl->m_pOutCache);
    if( IoduRes == PNIO_OK) {
        // set uptodate flags used by the watchdog, cache flag only, not a kram flag
        m_pTWDg->CacheSet(LogAddr);
    }

    if( IoduRes == PNIO_OK && IOlocState == PNIO_S_GOOD && IOmergedLocState == PNIO_S_BAD)
    {
      TRC_OUT01(GR_IOR, LV_WARN, "not all components of splitted module have local state PNIO_S_GOOD, output data was written with local state PNIO_S_BAD LogAddr=%u", LogAddr);
      return PNIO_WARN_LOCAL_STATE_BAD;
    }
    else
    {
      return IoduRes;
    }
} /* end of IOC_data_write_cache */


/*-----------------------------------------------------------------------------
 * Name  : IOC_update_cache
 * Descr : Is called from pnio user PNIO_..cache_flush IController::write_cache_flush
 *         For all submodules check cach flag: cache uptodate?
 *         No: read cache content, merge, write into the cache,
 *             set flag cache uptodate
 *         write whole cach out -> call original IODU_ctrl_data_write_cache_flush
 * Return: PNIO_OK, PNIO_ERR_xxx
 */
PNIO_UINT32 IoConcentrator::IOC_update_cache(void)
{
    PNIO_UINT8  dstBuff[PNIO_MAX_IO_LEN];
    PNIO_UINT32 dstBuffLen = sizeof(dstBuff);
    PNIO_UINT32 IoduRes;
    PNIO_UINT8  usrBuff[PNIO_MAX_IO_LEN];     // kram read result
    PNIO_UINT32 usrBuffLen = sizeof(usrBuff); // required/possible len [in]
    PNIO_UINT32 usrReadLen = 0;               // current read len [out]
    PNIO_IOXS   IOlocState, IOremState;       // read IOX states from KRAM
    IODU_Item  *pIODUItem = m_pICtrl->m_pCpAdapter->pIODUItem;

    // for all submodules:
    PNIO_UINT32 lamax = getLogAddrMax();
    PNIO_UINT32 la;
    for ( la = 0; la <= lamax; la++ ) {
        IOR_CONCENTRATOR_ENTRY *pSubMod = getIocHostModule(la);
        if ( pSubMod ) {
            if ( !m_pTWDg->IsCacheSet(la) ) {
                // cache is not uptodate - update content

                IoduRes = IODU_ctrl_CACHE_read(pIODUItem, (0x8000 | la),
                                               usrBuffLen, &usrReadLen, &usrBuff[0],
                                               &IOlocState, &IOremState,
                                               m_pICtrl->m_CnsBndInArr  ? m_pICtrl->m_CnsBndInArr[0].l_offset : 0,
                                               m_pICtrl->m_CnsBndOutArr ? m_pICtrl->m_CnsBndOutArr[0].l_offset : 0,
                                               m_pICtrl->m_pInCache, m_pICtrl->m_pOutCache);
                if ( IoduRes != PNIO_OK ) {
                    TRC_OUT02(GR_IOR, LV_ERR, "IOC: IOC_write_cache_flush: CACHE_read ERR=%u Addr=%u",
                        IoduRes, la);
                    // ignore
                    continue;
                }

                // get local status from store (local status written in PNIO_data_write_cache
                // - NOT local status merged with transfer data)
                cacheStatusFromStore(la, &IOlocState);

                //dstBuffLen will be modified in ioDataMerge
                //and must be initialized every time we are going here
                dstBuffLen = sizeof(dstBuff);

                // merge read cache content with shadowram content == transferdata
                ioDataMerge (la, pSubMod, &usrBuff[0], usrReadLen, &dstBuff[0], &dstBuffLen);

                // merge local iox status. Cache + Shadow
                PNIO_UINT8 shadowIoxs = getShadowIoxs(pSubMod);
                PNIO_IOXS IOmergedLocState = (PNIO_IOXS)((PNIO_UINT8)IOlocState | shadowIoxs);

                IoduRes = IODU_ctrl_data_write_ex(pIODUItem, (0x8000 | la),
                                                  dstBuffLen, &dstBuff[0],
                                                  IOmergedLocState, &IOremState,
                                                  m_pICtrl->m_CnsBndInArr  ? m_pICtrl->m_CnsBndInArr[0].l_offset : 0,
                                                  m_pICtrl->m_CnsBndOutArr ? m_pICtrl->m_CnsBndOutArr[0].l_offset : 0,
                                                  m_pICtrl->m_pInCache, m_pICtrl->m_pOutCache);
                if ( IoduRes != PNIO_OK ) {
                    TRC_OUT02(GR_IOR, LV_ERR, "IOC: IOC_write_cache_flush: CACHE_write ERR=%u Addr=%u",
                        IoduRes, la);
                    // ignore
                    continue;
                }

                // set uptodate flag used by the watchdog
                if ( IoduRes == PNIO_OK) {
                    m_pTWDg->CacheSet(la);
                }

            } // end if not uptodate
        }
    } // end for all submodules

    return PNIO_OK;
} /* end of IOC_update_cache */

/*-----------------------------------------------------------------------------
 * Name  : IOC_watchdog_data_write
 * Descr :
 *   Note: We are using low level IODU_kram_read. (not IODU_ctrl_kram_read)
 * Param :
 *  [ IN]:
 *  [OUT]:
 * Return: PNIO_OK, PNIO_ERR_xxx
 */
PNIO_UINT32 IoConcentrator::IOC_watchdog_data_write(PNIO_UINT32 LogAddr)
{
    PNIO_UINT8  dstBuff[PNIO_MAX_IO_LEN];     // merged data
    PNIO_UINT32 dstBuffLen = sizeof(dstBuff);
    PNIO_UINT8  usrBuff[PNIO_MAX_IO_LEN];     // kram read result
    PNIO_UINT32 usrBuffLen = sizeof(usrBuff); // required/possible len [in]
    PNIO_UINT32 usrReadLen = 0;               // current read len [out]
    PNIO_IOXS   IOlocState, IOremState;       // read IOX states from KRAM
    PNIO_IOXS   IOmergedLocState;
    PNIO_UINT32 IoduRes;
    KRAMIOTLB_Item  *pItem = NULL;
    KRAMIOTLB_Ret    kr_ret;
    IODU_Item       *pIODUItem = m_pICtrl->m_pCpAdapter->pIODUItem;

    IOR_CONCENTRATOR_ENTRY* pSubMod = getIocHostModule(LogAddr);
    if ( pSubMod == NULL ) {
        TRC_OUT01(GR_IOR, LV_ERR, "IOC: IOC_watchdog_data_write: wrong IOC addr=%u",
            LogAddr);
        return PNIO_ERR_PRM_ADD;
    }

    // get the controller item from  logical address of the module
    kr_ret = KRAMIOTLB_FindContrItemByLogAddr(pIODUItem->Handle,
             (0x8000 | LogAddr),
             KRAMIOTLB_IO_IN,
             &pItem,
             &pIODUItem->krtlbHeader);

    if(kr_ret != KRAMIOTLB_OK || pItem == NULL) {
        TRC_OUT01(GR_IOR, LV_ERR, "IOC: IOC_watchdog_data_write: wrong KRAM addr=%u", LogAddr);
        return PNIO_ERR_UNKNOWN_ADDR;
    }

    // check connection to the device
    // PL. we ignore bad-connections (not available devices) and continue work
    if(pItem->available == KRAMIOTLB_SUBM_NOT_AVAILABLE) {
        TRC_OUT01(GR_IOR, LV_FCTINT, "IOC: IOC_watchdog_data_write: NO_CONNECTION addr=%u", LogAddr);
        //return PNIO_ERR_NO_CONNECTION;
    }

    // read current iodata from the KRAM - from device
    IoduRes = IODU_kram_read(pIODUItem, pItem, KRAMIOTLB_CONTR,
                             usrBuffLen, &usrReadLen, &usrBuff[0],
                             &IOlocState, 0, &IOremState);
    if ( IoduRes != PNIO_OK ) {
        TRC_OUT01(GR_IOR, LV_ERR, "IOC: IOC_watchdog_data_write: IODU_kram_read ERR=%u", IoduRes);
        return IoduRes;
    }

    // trace KRAM data(original)
    TRC_OUT04(GR_IOR, LV_TIMECRIT, "WdgW:KRAM: Addr=%u w1,2:%#x %#x IOS=%d ",
        LogAddr, &usrBuff[0], &usrBuff[4], IOlocState);

    // read local status prior to write merge
    getUserStatus(LogAddr, &IOlocState);

    // merge KRAM data with "transfer" data
    ioDataMerge (LogAddr, pSubMod, &usrBuff[0], usrBuffLen, &dstBuff[0], &dstBuffLen);

    // merge local iox status. KRAM + Shadow
    PNIO_UINT8 shadowIoxs = getShadowIoxs(pSubMod);
    IOmergedLocState = (PNIO_IOXS)((PNIO_UINT8)IOlocState | shadowIoxs);

    // trace merged data(result)
    TRC_OUT04(GR_IOR, LV_TIMECRIT, "WdgW:MRGD: Addr=%u w1,2:%#x %#x IOS=%d ",
        LogAddr, &dstBuff[0], &dstBuff[4], IOmergedLocState);

    // write merged data and ioxs into the KRAM -> into the peripheral device
    IoduRes = IODU_kram_write(pIODUItem, pItem, KRAMIOTLB_CONTR,
                              dstBuffLen, &dstBuff[0],
                              IOmergedLocState, &IOremState);
    if ( IoduRes != PNIO_OK ) {
        TRC_OUT01(GR_IOR, LV_ERR, "IOC: IOC_watchdog_data_write: IODU_kram_write ERR=%u", IoduRes);
        return IoduRes;
    }
    return PNIO_OK;
} /* end of IOC_watchdog_data_write */

/* old function should be removed */
#if 0
/*-----------------------------------------------------------------------------
 * Name  : sliceMerge
 * Descr :
 * Param :
 *  [ IN]: pdst, dstLen: Destionation buffer pointer and length (original/old content)
 *  [ IN]: psrc, srcLen: Source buffer pointer and length (new content)
 *  [ IN]: byteLen  : New content len. Number of bytes, including splitted bytes!
 *  [ IN]: byteBegin: Byte offset of the 1st new byte (same in src and dst buffer)
 *  [ IN]: bitBegin : Bit offset of the 1st new content bit. (LSB= offset 0, MSB=7)
 *  [ IN]: bitEnd   : Bit offset of the valid new content bit, values 0...7
 *  [OUT]: Merged bits in the destination buffer
 * Return: ==0: ok, != 0 error
 */
int sliceMerge ( PNIO_UINT8  *pdst, PNIO_UINT32 dstLen,
                 PNIO_UINT8  *psrc, PNIO_UINT32 srcLen,
                 PNIO_UINT32 byteLen,  PNIO_UINT32 byteBegin,
                 PNIO_UINT32 bitBegin, PNIO_UINT32 bitEnd)
{
    PNIO_UINT8 *pd = pdst + byteBegin;
    PNIO_UINT8 *ps = psrc + byteBegin;
    PNIO_UINT8  mask;

    if ( (byteBegin + byteLen) > dstLen || (byteBegin + byteLen) > srcLen ) {
        TRC_OUT03(GR_IOR, LV_ERR, "IOC: Error BuffLen slice=%u dst=%u src=%u",
            (byteBegin + byteLen), dstLen, srcLen);
        return -1;
    }
    if ( byteLen >= 2 ) {
        if ( byteLen > 2 ) {
            // handle middle bytes
            memcpy( pd+1, ps+1, byteLen-2 );
        }
        // handle edge (splitted) bytes
        // handle left side (begin, low addr.) byte
        if ( bitBegin == 0 ) {
            *pd = *ps;
        } else {
            // E.g. bitBegin=3 (first new bit offset=3)
            mask = (PNIO_UINT8)((PNIO_UINT16)0xFF00 >> (8-bitBegin)); // mask: 11111000
            // mask= 11111000 (LSB)     new bits=1, old bits=0
            //       -new-old
            *pd = *pd & ~mask;         // clear new bits in old buf. ~mask=00000111
            *pd = *pd | (*ps & mask);  // clear old bits of new content, set new bits in dest
        }
        // handle right side (end, high addr) byte
        pd += (byteLen-1);
        ps += (byteLen-1);
        if ( bitEnd == 7 ) {
            *pd = *ps;
        } else {
            // E.g. bitEnd=4 (last new bit offset=4 =5bits)
            mask = (PNIO_UINT8)((PNIO_UINT16)0xFF00 >> (8-(bitEnd+1))); // 11100000
            // mask= 11100000 (LSB)   old bits=1, new bits=0
            //       old-new-
            *pd = *pd & mask;          // clear new bits in old buf. mask=11100000
            *pd = *pd | (*ps & ~mask); // clear old bits of new content, set new bits in dest
        }
    } else if ( byteLen == 1 ) {
        // handle one byte merge
        // E.g. bitBegin=2, bitEnd=6: (MSB)onnnnnoo(LSB)
        mask = (PNIO_UINT8)((PNIO_UINT16)0xFF00 >> (8-bitBegin));   // mask: 11111100
        mask = mask & (PNIO_UINT8)~((PNIO_UINT16)0xFF00 >> (8-(bitEnd+1)));
        // mask: 10000000 -> (01111111 & 11111100) = 01111100
        *pd = *pd & ~mask;          // clear new bits in old buf. ~mask=10000011
        *pd = *pd | (*ps & mask);   // clear old bits of new content, set new bits in dest
    } else {
        // byteLen ==0  warning, no action
        TRC_OUT03(GR_IOR, LV_WARN, "IOC: sliceMerge: byteLen=ZERO %u %u %u",
            byteBegin, bitBegin, bitEnd);
    }
    return 0;
} // end of sliceMerge
#endif 0

/*-----------------------------------------------------------------------------
 * Name  : sliceMerge
 * Descr :
 * Param :
 *  [ IN]: pdst     : Destionation buffer pointer (original content == output buffer)
 *                    Points always to SubModul begin (offset 0).
 *  [ IN]: dstLen   : SubModul length in bytes.
 *  [ IN]: psrc     : Source buffer pointer. (new content, data to be written)
 *  [ IN]: srcLen   : Data length == num bytes to be written.
 *
 * --- following 4 input params describe write 'bit slice' == Range of bits of the write data
 *                    which have to be merged, written over the dest buffer data.
 *  [ IN]: byteLen  : Bit-Slice length. Number of bytes, including splitted bytes!
 *  [ IN]: byteBegin: Byte offset of the 1st bit slice byte (offset to subModul begin)
 *  [ IN]: bitBegin : Bit offset of the 1st bit slice bit. (LSB= offset 0, MSB=7)
 *  [ IN]: bitEnd   : Bit offset of the last valid bit slice bit, values 0...7
 *
 * --- following 2 input params are new default parameter added for CBSA (partial modul access).
 *
 *  [ IN]: srcAddrOffset: Offset of the 'psrc' (=write data) to the submodul begin.
 *  [ IN]: partial_access:=True if this merge is one of the CBSA. Controller Byte Slice Access.
 *
 *  [OUT]: Merged bits in the destination buffer 'pdst'
 *
 * Return: ==0: ok, != 0 error
 */
int sliceMerge   (PNIO_UINT8  *pdst, PNIO_UINT32 dstLen,
                  PNIO_UINT8  *psrc, PNIO_UINT32 srcLen,
                  PNIO_UINT32 byteLen,  PNIO_UINT32 byteBegin,
                  PNIO_UINT32 bitBegin, PNIO_UINT32 bitEnd,
                  PNIO_UINT32 srcAddrOffset=0,
                  bool        partial_access = false)
{
    PNIO_UINT8 *pd;   // destination
    PNIO_UINT8 *ps;   // source = user data to be written (slice only)
    PNIO_UINT8  mask;

    if ( (byteBegin + byteLen) > dstLen ) {
        TRC_OUT02(GR_IOR, LV_ERR, "IOC: MRG:Error BuffLen sliceTotal=%u dst=%u ",(byteBegin + byteLen), dstLen);
        return -1;
    }

    if (partial_access) {
        // this is a controller byte slice access (CBSA) = subModul partial access
        // 1st save bit-slice descriptor parameters for later calculations
        PNIO_UINT32 byteLenOrig  = byteLen;
        PNIO_UINT32 byteBeginOrig= byteBegin;
        PNIO_UINT32 bitBeginOrig = bitBegin;
        PNIO_UINT32 bitEndOrig   = bitEnd;

        // check bit-slice begin in relation to the write data begin
        if ( byteBeginOrig < srcAddrOffset ) {
            // this bit-slice begin is out of range of the write data block
            // check slice end. Slice end offset == offset of the last byte of the slice == (begin + len - 1)
            //
            if ( (byteBeginOrig + byteLenOrig -1) < srcAddrOffset ) {
                // bit slice ends before write block begins =>
                // this slice is out of write block => no merge necessary
                TRC_OUT04(GR_IOR, LV_WARN, "IOC: MRG:OutOffWr1 OK: sb=%u sl=%u wb=%u wl=%u",
                          byteBegin, byteLen, srcAddrOffset, srcLen);
                return 0;
            }
            else {
                // bit slice end is inside or behind the write block => merge required
                // part of this slice is in the range of the write data,
                // but slice begin is not inside of the wr data =>
                // we have to modify slice begin variables for later merge process
                byteBegin = srcAddrOffset;
                bitBegin  = 0;

                // we have to check if the bit-lice end, is behind the wr block end
                // note: last wr block offset == srcAddrOffset + srcLen -1
                if ( (byteBeginOrig + byteLenOrig -1) > (srcAddrOffset + srcLen -1) ) {
                    // bit slice end is behind the write block
                    // we have to modify slice end variables for later merge process
                    // we reduce the length of the bit slice
                    // current bit-slice len is diff of 'wr block end +1' - 'current slice begin'
                    byteLen = (srcAddrOffset + srcLen) - byteBegin;
                    bitEnd  = 7;
                }
            }
        }
        else if ( byteBeginOrig <= (srcAddrOffset + srcLen -1) ) {
            // bit slice begins inside of the write block
            // check slice end: we have to check if it's behind the wr block end
            if ( (byteBeginOrig + byteLenOrig -1) > (srcAddrOffset + srcLen -1) ) {
                // bit slice ends behind the write block
                // we have reduce the length of the bit slice
                byteLen = srcAddrOffset + srcLen - byteBegin;
                bitEnd  = 7;
            }
        }
        else {
            // bit slice begins behind the write block
            // byteBegin > (srcAddrOffset + srcLen -1) =>
            // this slice is out of write block, begins after it => no merge necessary
            TRC_OUT04(GR_IOR, LV_WARN, "IOC: MRG:OutOffWr2 OK: sb=%u sl=%u wb=%u wl=%u",
                      byteBegin, byteLen, srcAddrOffset, srcLen);
            return 0;
        }
    } // end if partial_access

    // do the bit slice merge
    //--------------------------
    // set dest. ptr (output buffer with dev data) and src. ptr (user write data)
    // Note: original setting for standard access (write at subModul begin)
    //     PNIO_UINT8 *pd = pdst + byteBegin;
    //     PNIO_UINT8 *ps = psrc + byteBegin;
    //
    // calculate slice begin offset in the src. buffer == merge begin
    // Note: byteBegin was corrected so it should be always >= srcAddrOffset
    //
    if (byteBegin < srcAddrOffset) {
        TRC_OUT02(GR_IOR, LV_ERR, "IOC: MRG:ERROR sliceBegin=%u srcAddrOff=%u ",byteBegin, srcAddrOffset);
        return -1;
    }

    PNIO_UINT32 byteBeginInSrcBuff = byteBegin - srcAddrOffset;   // get slice begin in the src buffer
    pd = pdst + byteBeginInSrcBuff;
    ps = psrc + byteBeginInSrcBuff;

    if ( byteLen >= 2 ) {
        if ( byteLen > 2 ) {
            // handle middle bytes
            memcpy( pd+1, ps+1, byteLen-2 );
        }
        // handle edge (splitted) bytes
        // handle left side (begin, low addr.) byte
        if ( bitBegin == 0 ) {
            *pd = *ps;
        } else {
            // E.g. bitBegin=3 (first new bit offset=3)
            mask = (PNIO_UINT8)((PNIO_UINT16)0xFF00 >> (8-bitBegin)); // mask: 11111000
            // mask= 11111000 (LSB)     new bits=1, old bits=0
            //       -new-old
            *pd = *pd & ~mask;         // clear new bits in old buf. ~mask=00000111
            *pd = *pd | (*ps & mask);  // clear old bits of new content, set new bits in dest
        }
        // handle right side (end, high addr) byte
        pd += (byteLen-1);
        ps += (byteLen-1);
        if ( bitEnd == 7 ) {
            *pd = *ps;
        } else {
            // E.g. bitEnd=4 (last new bit offset=4 =5bits)
            mask = (PNIO_UINT8)((PNIO_UINT16)0xFF00 >> (8-(bitEnd+1))); // 11100000
            // mask= 11100000 (LSB)   old bits=1, new bits=0
            //       old-new-
            *pd = *pd & mask;          // clear new bits in old buf. mask=11100000
            *pd = *pd | (*ps & ~mask); // clear old bits of new content, set new bits in dest
        }
    } else if ( byteLen == 1 ) {
        // handle one byte merge
        // E.g. bitBegin=2, bitEnd=6: (MSB)onnnnnoo(LSB)
        mask = (PNIO_UINT8)((PNIO_UINT16)0xFF00 >> (8-bitBegin));   // mask: 11111100
        mask = mask & (PNIO_UINT8)~((PNIO_UINT16)0xFF00 >> (8-(bitEnd+1)));
        // mask: 10000000 -> (01111111 & 11111100) = 01111100
        *pd = *pd & ~mask;          // clear new bits in old buf. ~mask=10000011
        *pd = *pd | (*ps & mask);   // clear old bits of new content, set new bits in dest
    } else {
        // byteLen ==0  warning, no action
        TRC_OUT03(GR_IOR, LV_WARN, "IOC: MRG:byteLen=ZERO %u %u %u",byteBegin, bitBegin, bitEnd);
    }
    return 0;
} // end of sliceMerge


/*-----------------------------------------------------------------------------
 * Name  : ioDataMerge
 * Descr : 'Device io-data' (== supervisor controller data) are merged with
 *         io-data written by the 'io-router controller' given by the caller
 *         in pSrcBuffer. 'IO-Router device' io-data are already written into
 *         the 'shadow buffer' (this is made by the router agent in the firmware.
 *         Shadow buffer content is 1st copied into the destination (output) buffer
 *         and afterwards the source buffer content is pasted over the dest buffer.
 *         I.e.: Slices of the "src buffer" are written over the dest buff content.
 * Param :
 *  [ IN]: LogAddr: Logical addres of subModule begin. 1st addr of subModul
 *         If partial_access: (=CBSA) any module addr is valid
 *  [ IN]: pSubMod: Pointer to the concetrator table entry. IOR_CONCENTRATOR_ENTRY
 *  [ IN]: pSrcBuffer: Pointer to the user io-data to be written.
 *  [ IN]: SrcBuffLen: Length of the io-data to be written (number ob bytes).
 *  [OUT]: pDstBuffer: Pointer to the output buffer, merged data are returned in.
 *                     DestBuffer is a temporary buffer of the max. size= PNIO_MAX_IO_LEN
 *                     given by the caller.
 *  [IN|OUT]: pDstBuffLen: IN: DestBuff max available length of the output buffer.
 *                             Must be == PNIO_MAX_IO_LEN  == 254
 *                        OUT: SubModule total length.
 *                             If partial_access (CBSA): write data len == SrcBuffLen
 * Return: int: 0 == ok
 */
int IoConcentrator::ioDataMerge (PNIO_UINT32 LogAddr, IOR_CONCENTRATOR_ENTRY* pSubMod,
                                 PNIO_UINT8 *pSrcBuffer, PNIO_UINT32 SrcBuffLen,
                                 PNIO_UINT8 *pDstBuffer, PNIO_UINT32 *pDstBuffLen)
{

    PNIO_UINT8* pShadowData = NULL;
    PNIO_UINT32 shadowLen   = 0;     // ==subModule length
    PNIO_UINT32 addr_offset = 0;     // for std access addr-offset == 0
    PNIO_UINT32 copy_len;
    bool partial_access = false;


    getShadowDataPtrAndLength(pSubMod, &pShadowData, &shadowLen);

    if (!pShadowData) {
        TRC_OUT01(GR_IOR, LV_ERR, "IOC: Error Addr=%u NO shadow", LogAddr);
        return -1;
    }

    copy_len = shadowLen;

    // trace shadow data + iox state
    TRC_OUT03(GR_IOR, LV_TIMECRIT, "IOCm SHDW: Addr=%u Len=%d IOS=%d ",LogAddr, shadowLen, getShadowIoxs(pSubMod));


    // if (m_moduleSliceAcessIsOn) {
    // Note: Check of 'ModuleSliceAcess Flag' is not necessary. If it's OFF, IocAddrHash contains only
    //       submodul begin addresses. See PNIO_controller_open() ExtPar: controller byte slice access (CBSA)

    // we check if this write request is a partial access. (Modul Teilzugriff).
    // If YES: we don't copy the whole submodule data from shadowBuff into the destBuff, but only
    // the corresponding bytes which are tobe written.
    // E.g.: 8 byte submodule at addr 0. We write 3 bytes at addr 1. Only 3 byte are copied.
    // If NOT: we handle this request as the standard one. (full subModul access).
    //
    // Note: The write access is a partial module byte slice access if either logical address is not
    //       modul begin address or the data length is shorter than the subModul total length.
    //       pSubMod->LogAddr is always the subModul begin address. 'LogAddr' is any subModule addr.
    //
    addr_offset = LogAddr - pSubMod->LogAddr;

    if ( addr_offset != 0 || SrcBuffLen < shadowLen ) {
        partial_access = true;
    }

    copy_len = SrcBuffLen;   // SrcBuffLen= requested write length (num bytes tobe written)

    if ( (addr_offset + copy_len) > shadowLen ) {
        // write data len too large
        TRC_OUT03(GR_IOR, LV_ERR, "IOC: ERROR AddrOffset=%u cpLen=%u GT shadowLen=%u",addr_offset, copy_len, shadowLen);
        // be defensive, reduce copy-len, continue
        copy_len = shadowLen - addr_offset;
    }

    // copy corresponding io-router device data to the dest. buffer. I.e. only the block which is tobe written
    //
    memcpy( pDstBuffer, pShadowData + addr_offset, copy_len );  // copy: destBuff <-- shadowBuff

    *pDstBuffLen = copy_len;                                    // set output: dest buffer len

    // merge i.e. copy 'user write bits' over the dest. buffer (shadow data) according to the
    // bit slice descriptors
    //
    int numSlices = (int)getNumOfSlices(pSubMod);
    for ( int i = 0; i < numSlices; i++ ) {
        IOR_OUTPUT_SLICE *pSlice = getSliceAtIdx(pSubMod,i);
        if ( sliceMerge(pDstBuffer,
                        shadowLen,                  /* max subModul len */
                        pSrcBuffer,
                        copy_len,                   /* write data len. == SrcBuffLen */
                        pSlice->ByteLength, pSlice->ByteOffset,
                        pSlice->FirstBitOffset, pSlice->LastBitOffset,
                        addr_offset,
                        partial_access ) != 0 ) {
            // error
            TRC_OUT01(GR_IOR, LV_ERR, "IOC: ioDataMerge: ERROR sliceMerge: idx=%d ",i);
            // be defensive ignore, try next one
        }
    } // end for all slices

    return 0; // ok
} /* end of ioDataMerge */

/*-----------------------------------------------------------------------------
 * Name  : getNumOfIocModules
 * Descr : Delivers the total number of SubModule entriest in the
 *         concentrator table, including submodules with zero slices i.e.
 *         pure transfer no mixed modules
 * Param :
 *  [ IN]:
 *  [OUT]:
 * Return: ==0: ok, != 0 error
 */
int IoConcentrator::getNumOfIocModules (void)
{
    if (!m_pIocTabHdr) {
        return 0;
    }
    return (m_pIocTabHdr->NumOfConEntries);

} /* end of getNumOfIocModules */

/*-----------------------------------------------------------------------------
 * Name  : getIocHostModule(PNIO_UINT32 LogAddr);
 * Descr : Delivers pointer to an "IOC-Submodule" with number slices > 0
 *         "IOC-SubModule" is one with valid concentrator slice(s)
 * Param :
 *  [ IN]: LogAddr
 *  [OUT]:
 * Return: != NULL Ptr to the ioconcentrator submodule with this logical address
 *         == NULL This log addr is not one of ioc module
 */
IOR_CONCENTRATOR_ENTRY* IoConcentrator::getIocHostModule(PNIO_UINT32 LogAddr)
{
    if (!m_pIocTabHdr) {
        return NULL;
    }
    if ( LogAddr <= m_hashAddrMax && m_pIocAddrHash[LogAddr]) {
        // range ok
        IOR_CONCENTRATOR_ENTRY *psm = m_pIocAddrHash[LogAddr]->pTransferData;
        if ( psm ) {
            if ( getNumOfSlices(psm) > 0 ) {
                return psm;  // is an io concentrator submodule
            } else {
                return NULL; // no concentrator slices => not ioc submodule
            }
        } else {
            return NULL; // not an ioc submodule
        }
    } else {
        return NULL; // out of range => not configured ioc submodule addr
    }
} /* end of getIocHostModule */

/*-----------------------------------------------------------------------------
 * Name  : getIocModule
 * Descr : Delivers pointer to the submodule entry as function of logaddr.
 *         Independet of the number of slices
 * Param :
 *  [ IN]: LogAddr: logical io address
 *  [OUT]:
 * Return: Ptr: != 0 ok,  == 0 No concentrator entry available.
 */
IOR_CONCENTRATOR_ENTRY*  IoConcentrator::getIocModule(PNIO_UINT32 LogAddr)
{
    if (!m_pIocTabHdr) {
        return NULL;  // no router / concentrator available
    }
    if ( LogAddr <= m_hashAddrMax && m_pIocAddrHash[LogAddr]) {
        return ( m_pIocAddrHash[LogAddr]->pTransferData );
    } else {
        return NULL;  // addr out of range
    }
} /* end of getIocModule */

/*-----------------------------------------------------------------------------
 * Name  : getIocModuleAtIdx
 * Descr : Delivers pointer to a submodule entry in the concentrator table.
 * Param :
 *  [ IN]: Index of the required entry
 *  [OUT]:
 * Return: ptr to IOR_CONCENTRATOR_ENTRY  ==ok, or  0 ==error
 */
IOR_CONCENTRATOR_ENTRY* IoConcentrator::getIocModuleAtIdx(int idx)
{
    IOR_CONCENTRATOR_ENTRY *pSubMod_Ret = NULL;
    if (!m_pIocTabHdr) {
        return NULL;
    }
    if ( (PNIO_UINT32)idx >= m_pIocTabHdr->NumOfConEntries ) {
        return NULL; // out of range
    }
    pSubMod_Ret = (IOR_CONCENTRATOR_ENTRY*)(((char*)m_pIocTabHdr) + m_pIocTabHdr->ConEntriesOff);  // 1st SubModule entry
    pSubMod_Ret = pSubMod_Ret + idx;
    return pSubMod_Ret;
} /* end of getIocModuleAtIdx */

/*-----------------------------------------------------------------------------
 * Name  : getNumOfSlices
 * Descr :
 * Param :
 *  [ IN]:
 *  [OUT]:
 * Return: >= 0: ok, < 0 error
 */
int IoConcentrator::getNumOfSlices (IOR_CONCENTRATOR_ENTRY* pSubMod)
{
    if (!pSubMod) {
        return 0;
    } else {
        return (int)(pSubMod->NumOfSlices);
    }
}

/*-----------------------------------------------------------------------------
 * Name  : getNumOfPureTransferModules
 * Descr :
 * Param :
 *  [ IN]:
 *  [OUT]:
 * Return:
 */
PNIO_UINT32 IoConcentrator::getNumOfPureTransferModules ()
{
    int numOfPureTransferModules = 0;
    int numOfSubModules = getNumOfIocModules();

    for (int ti=0; ti<numOfSubModules; ti++) {

        IOR_CONCENTRATOR_ENTRY* pSubmod = getIocModuleAtIdx(ti);
        if (getNumOfSlices(pSubmod) == 0) {
            // pure transfer module found
            ++numOfPureTransferModules;
        }
    }

    return (PNIO_UINT32) numOfPureTransferModules;
}

/*-----------------------------------------------------------------------------
 * Name  : getNumOfIocHostModules
 * Descr :
 * Param :
 *  [ IN]:
 *  [OUT]:
 * Return:
 */
PNIO_UINT32 IoConcentrator::getNumOfIocHostModules(void)
{
    int numOfIocHostModules = 0;
    int numOfIocModules = getNumOfIocModules();

    for (int ti=0; ti<numOfIocModules; ti++) {

        IOR_CONCENTRATOR_ENTRY* pSubmod = getIocModuleAtIdx(ti);
        if (getNumOfSlices(pSubmod)) {
            // pure transfer module found
            ++numOfIocHostModules;
        }
    }

    return (PNIO_UINT32) numOfIocHostModules;
}

/*-----------------------------------------------------------------------------
 * Name  : getMinTwdTime
 * Descr : Get minimal watchdog time of sliced module
 * Param :
 *  [ IN]:
 *  [OUT]:
 * Return: minimal watchdog time in multiples of 31.25us
 */
PNIO_UINT32 IoConcentrator::getMinTwdTime(void)
{
    PNIO_UINT32 minTwdTime = 0;
    PNIO_UINT32 numOfIocModules = getNumOfIocModules();

    for (PNIO_UINT32 i=0; i<numOfIocModules; i++) {
        PNIO_UINT32 twdTime = getTwdTime(i);
        if (twdTime) {
            // module has valid watchdog time, check if wd time is smaller than previous ones
            if (minTwdTime == 0 || twdTime < minTwdTime ) {
                minTwdTime = twdTime;
            }
        }
    }

    return minTwdTime;
}

/*-----------------------------------------------------------------------------
 * Name  : getTwdTime
 * Descr : Get watchdog time of sliced module
 * Param :
 *  [ IN]: PNIO_UINT32 idx: Index of ioc module, range: 0..getNumOfIocModules()
 *  [OUT]:
 * Return: 0 if pure transfer module, otherwise watchdog time in multiples of 31.25us
 */
PNIO_UINT32 IoConcentrator::getTwdTime(PNIO_UINT32 idx)
{
    PNIO_UINT32 twdTime = 0;

    if (getIocModuleAtIdx(idx)->NumOfSlices > 0) {
        // calculate pointer to first concentrator entry
        IOR_CONCENTRATOR_ENTRY  *pConcEntry  =
            (IOR_CONCENTRATOR_ENTRY * ) (m_pIocTabHdr->ConEntriesOff + (PNIO_UINT8*)m_pShmTabHdr);
        // proceed to concentrator entry at index idx
        twdTime = LE_TO_CPU((pConcEntry+idx)->WDTime);
    }

    return twdTime;
}

/*-----------------------------------------------------------------------------
 * Name  : getTwdValidity
 * Descr :
 * Param :
 *  [ IN]:
 *  [OUT]:
 * Return:
 */
bool IoConcentrator::getTwdValidity()
{
    return (LE_TO_CPU(m_pShmTabHdr->WDTrigger)) ? true:false;
}

/*-----------------------------------------------------------------------------
 * Name  : getPureTransferModuleList
 * Descr : copies information about all pure modules to previously allocated array.
 * Param :
 *  [ IN]: pAddrList: Pointer to array of module info structs
 *         numOfListItems: Number of array elements
 *  [OUT]:
 * Return:
 */
void IoConcentrator::getPureTransferModuleList(PURE_MODULE_INFO * pAddrList, PNIO_UINT32 numOfListItems)
{
    unsigned int pureIdx = 0;
    PNIO_UINT32 numOfPureTransferModules = (PNIO_UINT32) getNumOfPureTransferModules ();
    PNIO_UINT32 numOfSubModules = (PNIO_UINT32) getNumOfIocModules();

    DPR_ASSERT(numOfListItems >= numOfPureTransferModules);

    // Iterate over all submodule entries in concentrator table.
    for (int ti=0; ti< (int)numOfSubModules; ti++) {
        IOR_CONCENTRATOR_ENTRY* pSubmod = getIocModuleAtIdx(ti);
        if (getNumOfSlices(pSubmod) == 0) {
            // pure transfer module found
            DPR_ASSERT(pureIdx < numOfPureTransferModules);
            (pAddrList+pureIdx)->in_out_type = IO_OUT;
            (pAddrList+pureIdx)->log_addr    = pSubmod->LogAddr;
            pureIdx++;
        }
    }
}

/*-----------------------------------------------------------------------------
 * Name  : getSliceBitOffset    getSliceBitLength
 * Descr : BitOffset == total num bits from submodul begin.
 *         BitLength == total num bits of a slice
 * Param :
 *  [ IN]:
 *  [OUT]:
 * Return: ==0: ok, != 0 error
 */
int IoConcentrator::getSliceBitOffset(IOR_OUTPUT_SLICE* pSlice)
{
    return (int)(pSlice->ByteOffset * 8 + pSlice->FirstBitOffset);
}

int IoConcentrator::getSliceBitLength(IOR_OUTPUT_SLICE* pSlice)
{
    // BitLen = (ByteLen-2)*8  + (8-FirstBitOff) + (LastBitOff+1)
    return (pSlice->ByteLength*8 + pSlice->LastBitOffset - pSlice->FirstBitOffset - 7);
} /* end of getSliceBitLength */

/*-----------------------------------------------------------------------------
 * Name  : getShadowDataPtrAndLength
 * Descr : Returns pointer to the transfer data in the shared memory
 * Param :
 *  [ IN]: pSubMod : Pointer to the submodule entry an info is asked from.
 *  [OUT]: *dataPtr: Pointer to the io-data in the shadow memory
 *  [OUT]: dataLen : Length of the data
 * Return: == 0: ok, !=0 error
 */
int IoConcentrator::getShadowDataPtrAndLength (IOR_CONCENTRATOR_ENTRY* pSubMod,
        PNIO_UINT8 **dataPtr, PNIO_UINT32 *dataLen)
{
    if (!pSubMod) {
        *dataPtr = NULL;
        return -1;
    } else {
        // Shared memory pointer must be used to access transfer data
        *dataPtr = (PNIO_UINT8*)(((PNIO_UINT8*)m_pShmTabHdr) + pSubMod->ShadowDataOff);
        *dataLen = (PNIO_UINT32)(pSubMod->ShadowLength);
        return 0;
    }
}

/*-----------------------------------------------------------------------------
 * Name  : getShadowIoxsPtrAndLength
 * Descr : Delivers IOX status from shared memory
 * Param :
 *  [ IN]: pSubMod : Pointer to the submodule entry an info is asked from.
 * Return: IOXS
 */
PNIO_UINT8 IoConcentrator::getShadowIoxs (IOR_CONCENTRATOR_ENTRY* pSubMod)
{
    if (!pSubMod) {
        return PNIO_S_BAD;
    } else {
        return *((PNIO_UINT8*)(((PNIO_UINT8*)m_pShmTabHdr) + pSubMod->StatusDataOff));
        //int ioxsLen = (PNIO_UINT32)(pSubMod->StatusLength); // always 1
    }
}

/*-----------------------------------------------------------------------------
 * Name  : setShadowHostStatus
 * Descr : set host data status in shared memory
 * Param :
 *  [ IN]: LogAddr: Log. Address of module
 *      status: local status
 * Return: void
 */
void IoConcentrator::setShadowHostStatus (PNIO_UINT32 LogAddr, PNIO_IOXS status)
{
    PNIO_UINT32 HostStatusDataOff = m_pIocAddrHash[LogAddr]->pTransferData->HostStatusDataOff;
    PNIO_UINT8* pHostStatusData = (PNIO_UINT8*)(((PNIO_UINT8*)m_pShmTabHdr) + HostStatusDataOff);

    *pHostStatusData = status;
}


/*-----------------------------------------------------------------------------
 * Name  : getSliceAtIdx
 * Descr :
 * Param :
 *  [ IN]: pSubMod: Pointer to the submodule entry a slice is asked from.
 *  [ IN]: idx    : Index of the required slice
 *  [OUT]:
 * Return: != 0 ok, valid ptr. == 0: error or nomore slices (out of range)
 */
IOR_OUTPUT_SLICE* IoConcentrator::getSliceAtIdx(IOR_CONCENTRATOR_ENTRY* pSubMod, int idx)
{

    IOR_OUTPUT_SLICE *pSlice = NULL;

    if ( (PNIO_UINT32)idx >= (PNIO_UINT32)(pSubMod->NumOfSlices) ) {
        return NULL; // out of range
    }
    pSlice = (IOR_OUTPUT_SLICE*)(((char*)m_pIocTabHdr) + pSubMod->SlicesOff);  // 1st SubModule entry
    pSlice = pSlice + idx;
    return pSlice;
} /* end of getSliceAtIdx */

/*-----------------------------------------------------------------------------
 * Name  : getIocModAddrMax
 * Descr : Scans all configured submodules in the concentrator table and
 *         returns the largest log_addr found. MaxAddr is used as limit for the
 *         addrHashTable creation and later for access boundary check.
 *         Is called by the initAddrHash(). User space ioc table (m_pIocTabHdr)
 *         must be already created before calling this function!
 * Param : FilterPureModules - optional. If true, pure transfer modules
 *      (at least one slice in concentrator table) are ignored
 * Return: LogAddr: ( 0 - 0x7FFF )  Note: zero is a valid LogAddr
 */
PNIO_UINT32 IoConcentrator::getIocModAddrMax(bool FilterPureModules)
{
    PNIO_UINT32 lamax = 0;                       // log-addr max
    IOR_CONCENTRATOR_HEADER *pH = m_pIocTabHdr;  // ptr to user space ioc tab (already swapped)

    if ( !pH ) {
        TRC_OUT(GR_IOR, LV_ERR, "IOC: getIocModAddrMax ERROR NO TAB");
        return 0;
    }

    // Determine pointers to concentrator entries
    IOR_CONCENTRATOR_ENTRY *pSubMod = (IOR_CONCENTRATOR_ENTRY*)(pH->ConEntriesOff + (PNIO_UINT8*)pH);
    PNIO_UINT32 numEntries = pH->NumOfConEntries;

    // for all submodules: find largest log_addr configured
    for (PNIO_UINT32 i=0; i < numEntries; i++) {
        if (!FilterPureModules || pSubMod->NumOfSlices) {
            lamax = max(lamax, pSubMod->LogAddr);
        }
        pSubMod++;          // go to the next submodule
    } // end for all submodule entries
    TRC_OUT01(GR_IOR, LV_INFO, "IOC: <-getIocModAddrMax=%u", lamax);
    return lamax;

} /* end of getIocModAddrMax */


/*-----------------------------------------------------------------------------
 * Name  : updateUserStatus
 * Descr :
 * Param :
 *  [ IN]:
 *  [OUT]:
 * Return:
 */
void IoConcentrator::updateUserStatus(PNIO_UINT32 LogAddr, PNIO_IOXS status)
{
    m_pIocAddrHash[LogAddr]->kramUserStatus = status;
    setShadowHostStatus(LogAddr, status);
}

/*-----------------------------------------------------------------------------
 * Name  : getUserStatus
 * Descr :
 * Param :
 *  [ IN]:
 *  [OUT]:
 * Return:
 */
void IoConcentrator::getUserStatus(PNIO_UINT32 LogAddr, PNIO_IOXS * pStatus)
{
    *pStatus = m_pIocAddrHash[LogAddr]->kramUserStatus;
}

/*-----------------------------------------------------------------------------
 * Name  : cacheStatusToStore
 * Descr :
 * Param :
 *  [ IN]:
 *  [OUT]:
 * Return:
 */
void IoConcentrator::cacheStatusToStore(PNIO_UINT32 LogAddr, PNIO_IOXS status)
{
    m_pIocAddrHash[LogAddr]->cacheUserStatus = status;
}

/*-----------------------------------------------------------------------------
 * Name  : cacheStatusFromStore
 * Descr :
 * Param :
 *  [ IN]:
 *  [OUT]:
 * Return:
 */
void IoConcentrator::cacheStatusFromStore(PNIO_UINT32 LogAddr, PNIO_IOXS * pStatus)
{
    *pStatus = m_pIocAddrHash[LogAddr]->cacheUserStatus;
}

/*-----------------------------------------------------------------------------
 * Name  : updateUserStatusFromCache
 * Descr :
 * Param :
 *  [ IN]:
 *  [OUT]:
 * Return:
 */
void IoConcentrator::updateUserStatusFromCache(void)
{
    int numOfIocModules = getNumOfIocModules();

    // provider (local) status set by application/user:
    // copy status of all ioc modules from cache to kram store
    for (int i=0; i<numOfIocModules; i++) {
        PNIO_IOXS cacheUserStatus = m_pIocIdxTable[i].cacheUserStatus;
        m_pIocIdxTable[i].kramUserStatus = cacheUserStatus;
        setShadowHostStatus(m_pIocIdxTable[i].pTransferData->LogAddr, cacheUserStatus);
    }
}

/*-----------------------------------------------------------------------------
 * Name  : copyAndSwapConcentratorTable
 * Descr : LE_TO_CPU swaps Little Endian 32ints to the CPU endianes.
 *         SharedMemoty IOC Table is written by the fw and is Little Endian.
 *         User space table must be already allocated before calling this funct.
 *         Attribute:  m_pIocTabHdr ;  // dest. memory have to be already allocated
 * Param :
 *  [ IN]: pDprHdr: DualPort (SharedMem) IOC Table pointer
 *  [OUT]:
 * Return: -
 */
int IoConcentrator::copyAndSwapConcentratorTable(IOR_CONCENTRATOR_HEADER * pUsrTabHdr,
        IOR_CONCENTRATOR_HEADER * pDprHdr)
{

    IOR_CONCENTRATOR_HEADER *pHostHdr = pUsrTabHdr;  // ptr to user space ioc tab
    IOR_CONCENTRATOR_ENTRY  *pDprEntry;
    IOR_CONCENTRATOR_ENTRY  *pHostEntry;
    IOR_OUTPUT_SLICE    *pDprSlice;
    IOR_OUTPUT_SLICE     *pHostSlice;

    if ( !pDprHdr || !pHostHdr ) {
        TRC_OUT02(GR_IOR, LV_ERR, "IOC: ERROR PAR a1=%#x a2=%#x",pDprHdr,pHostHdr);
        return -1;
    }

    // copy concentrator header from dpr to host buffer and convert - if necessary - to little endian
    pHostHdr->Validity    = LE_TO_CPU(pDprHdr->Validity);
    pHostHdr->NumOfConEntries  = LE_TO_CPU(pDprHdr->NumOfConEntries);
    pHostHdr->TotalNumOfSlices = LE_TO_CPU(pDprHdr->TotalNumOfSlices);
    pHostHdr->NewCycleIrqTime  = LE_TO_CPU(pDprHdr->NewCycleIrqTime);
    pHostHdr->ConEntriesOff  = LE_TO_CPU(pDprHdr->ConEntriesOff);

    // Determine pointers to concentrator entries
    pDprEntry  = (IOR_CONCENTRATOR_ENTRY * ) (pHostHdr->ConEntriesOff + (PNIO_UINT8*)pDprHdr);
    pHostEntry = (IOR_CONCENTRATOR_ENTRY * ) (pHostHdr->ConEntriesOff + (PNIO_UINT8*)pHostHdr);

    for (PNIO_UINT32 i=0; i<pHostHdr->NumOfConEntries; i++) {

        // copy concentrator entry from dpr to host buffer and convert little endian -> cpu endianes
        pHostEntry->LogAddr   = LE_TO_CPU(pDprEntry->LogAddr);
        pHostEntry->ShadowDataOff = LE_TO_CPU(pDprEntry->ShadowDataOff);
        pHostEntry->ShadowLength = LE_TO_CPU(pDprEntry->ShadowLength);
        pHostEntry->StatusDataOff   = LE_TO_CPU(pDprEntry->StatusDataOff);
        pHostEntry->StatusLength    = LE_TO_CPU(pDprEntry->StatusLength);
        pHostEntry->HostStatusDataOff = LE_TO_CPU(pDprEntry->HostStatusDataOff);
        pHostEntry->HostStatusLength  = LE_TO_CPU(pDprEntry->HostStatusLength);
        pHostEntry->WDTime   = LE_TO_CPU(pDprEntry->WDTime);
        pHostEntry->NumOfSlices  = LE_TO_CPU(pDprEntry->NumOfSlices);
        pHostEntry->SlicesOff  = LE_TO_CPU(pDprEntry->SlicesOff);

        // Determine pointers to output slices
        pDprSlice  = (IOR_OUTPUT_SLICE * ) (pHostEntry->SlicesOff + (PNIO_UINT8*)pDprHdr);
        pHostSlice = (IOR_OUTPUT_SLICE * ) (pHostEntry->SlicesOff + (PNIO_UINT8*)pHostHdr);

        // copy controller output slice from dpr to host buffer and swap elements
        for (PNIO_UINT32 j=0; j < pHostEntry->NumOfSlices; j++) {
            pHostSlice->ByteLength   = LE_TO_CPU(pDprSlice->ByteLength);
            pHostSlice->ByteOffset  = LE_TO_CPU(pDprSlice->ByteOffset);
            pHostSlice->FirstBitOffset  = LE_TO_CPU(pDprSlice->FirstBitOffset);
            pHostSlice->LastBitOffset  = LE_TO_CPU(pDprSlice->LastBitOffset);
            // go to next slice
            pDprSlice++;
            pHostSlice++;
        }
        // go to next concentrator entry
        pDprEntry++;
        pHostEntry++;
    } // end for entries
    return 0;
} /* end of copyAndSwapConcentratorTable */


/*-----------------------------------------------------------------------------
 * Name  : TraceConcentratorTable    debug aim
 * Descr : Prints content of the Concentrator Table (already swapped).
 *         Group/depth has to be GR_IOR / LV_INFO
 * Param :
 *  [ IN]: pIocHdr: IOC Table pointer
 *  [OUT]:
 * Return: -
 */
void IoConcentrator::TraceConcentratorTable(IOR_CONCENTRATOR_HEADER * pTableHeader)
{
    TRC_OUT(GR_IOR, LV_FCTINT, "-> TraceConcentratorTable:");

    OSTREAM trcos1;
    IOR_CONCENTRATOR_ENTRY * pConcEntry;
    IOR_OUTPUT_SLICE * pSlice;
    trcos1 << "IOR_CONCENTRATOR_HEADER: ";
    pTableHeader->Validity? trcos1 <<"valid":trcos1<<"invalid";
    trcos1 << ", NumOfConEntries=" << pTableHeader->NumOfConEntries;
    trcos1 << ", TotalNumOfSlices=" << pTableHeader->TotalNumOfSlices;
    trcos1 << ", NewCycleIrqTime=" << pTableHeader->NewCycleIrqTime << "*32us";
    trcos1 << ends;
    TRC_OUT_OBJECT(GR_IOR, LV_INFO, trcos1);
    pConcEntry = (IOR_CONCENTRATOR_ENTRY * ) (pTableHeader->ConEntriesOff + (char*)pTableHeader);
    for ( PNIO_UINT32 i=0; i<pTableHeader->NumOfConEntries; i++ ) {
        OSTREAM trcos2;
        trcos2 << "IOR_CONCENTRATOR_ENTRY " << i+1 << ":";
        trcos2 << " LogAddr=" << pConcEntry->LogAddr;
        //   trcos2 << ", ShadowDataOff=" << pConcEntry->ShadowDataOff;
        trcos2 << ", ShadowLength="  << pConcEntry->ShadowLength;
        //   trcos2 << ", StatusDataOff=" << pConcEntry->StatusDataOff;
        trcos2 << ", StatusLength="  << pConcEntry->StatusLength;
        trcos2 << ", WDTime="        << pConcEntry->WDTime;
        trcos2 << ", NumOfSlices="   << pConcEntry->NumOfSlices;
        //   trcos2 << ", SlicesOff="     << pConcEntry->SlicesOff;
        trcos2 << ends;
        TRC_OUT_OBJECT(GR_IOR, LV_INFO, trcos2);

        pSlice =  (IOR_OUTPUT_SLICE *) (pConcEntry->SlicesOff + (char*)pTableHeader);
        for ( PNIO_UINT32 j=0; j<pConcEntry->NumOfSlices; j++ ) {
            OSTREAM trcos3;
            trcos3 << "IOR_OUTPUT_SLICE " << j+1 << ":";
            trcos3 << "ByteLength=" << pSlice->ByteLength;
            trcos3 << ", ByteOffset=" << pSlice->ByteOffset;
            trcos3 << ", FirstBitOffset=" << pSlice->FirstBitOffset;
            trcos3 << ", LastBitOffset=" << pSlice->LastBitOffset;
            trcos3 << ends;
            TRC_OUT_OBJECT(GR_IOR, LV_INFO, trcos3);
            pSlice++;   // Go to next slice
        }
        pConcEntry++;   // Go to next concentrator entry
    } // end for entries
    TRC_OUT(GR_IOR, LV_FCTINT, "<- TraceConcentratorTable");
    return;
} /* end of printIOConcentratorTable */

#endif /* IO_ROUTER */

