/****************************************************************************
*  Copyright (C) SIEMENS CORP., 2013 All rights reserved.
*****************************************************************************
* FILE NAME    : transferwd.cpp
* ---------------------------------------------------------------------------
* DESCRIPTION  : Transfer watchdog implementation
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

#include "pniointr.h"
#include "ior_dpram.h"

DPR_MUTEX PnioMutexTransferWd;

CTransferWD::CTransferWD():
    m_pIoc(NULL),
    m_pController(NULL),
    m_pTimeMon(NULL),
    m_pTimeMonIndex(NULL),
    m_NumOfIocHostModulePerCycle(0),
    m_actIocHostModuleIndex(0),
    m_NumOfIocHostModules(0),
    m_MaxIocHostModuleAddr(0),
    m_reduction(0),
    m_clockCount(0),
    m_wdTime(0),
    m_newCycleIrqTime(0),
    m_state(TWD_INIT)
{
}

CTransferWD::~CTransferWD()
{
    // deallocate buffers
    if (m_pTimeMon) {
        delete [] m_pTimeMon;
        m_pTimeMon = NULL;
    }
    if (m_pTimeMonIndex) {
        delete [] m_pTimeMonIndex;
        m_pTimeMonIndex = NULL;
    }
}


/*-----------------------------------------------------------------------------
 * Name  : Init
 * Descr : allocate buffers, some initialization
 * Param :
 *  [ IN]: pIoc - Pointer to corresponding io concentrator class
 *         pController - Pointer to corresponding controller class
 *  [OUT]:
 * Return:
 */
bool CTransferWD::Init(IoConcentrator * pIoc, IController * pController)
{
    IOR_CONCENTRATOR_HEADER * pTableHeader = NULL;
    IOR_CONCENTRATOR_ENTRY * pTableEntry = NULL;
    int NumOfModules;

    TRC_OUT(GR_IOR, LV_FCTINT, "-> CTransferWD::Init");

    DPR_ASSERT(pIoc);
    DPR_ASSERT(pController);

    m_NumOfIocHostModules = 0;
    m_MaxIocHostModuleAddr = 0;
    m_pIoc = pIoc;
    m_pController = pController;

    // Get pointer to concentrator table copy
    if(!m_pIoc){
        TRC_OUT(GR_IOR, LV_FCTINT, "<- CTransferWD::Init m_pIoc=0(no ioc table available)");
        return false;
    }

    pTableHeader = m_pIoc->getIocTabHdr();
    if (!pTableHeader) {
        TRC_OUT(GR_IOR, LV_FCTINT, "<- CTransferWD::Init (no ioc table available)");
        return false;
    }
    if (!pTableHeader->Validity) {
        TRC_OUT(GR_IOR, LV_FCTINT, "<- CTransferWD::Init (no valid ioc table available)");
        return false;
    }

    //
    NumOfModules            = m_pIoc->getNumOfIocModules();
    m_NumOfIocHostModules   = m_pIoc->getNumOfIocHostModules();
    m_MaxIocHostModuleAddr  = m_pIoc->getIocModAddrMax(true);

    // io router running but only pure transfer modules configured
    // -> nothing to do for transfer watchdog watchdog
    if (m_NumOfIocHostModules == 0) {
        TRC_OUT(GR_IOR, LV_FCTINT, "<- CTransferWD::Init (no sliced modules configured)");
        return false;
    }

    TRC_OUT02(GR_IOR, LV_ERR, "Transfer Watchdog found %d sliced modules with largest address %d",
        m_NumOfIocHostModules, m_MaxIocHostModuleAddr);

    // Deallocate previously allocated buffers
    if (m_pTimeMon) {
        delete [] m_pTimeMon;
        m_pTimeMon = NULL;
    }
    if (m_pTimeMonIndex) {
        delete [] m_pTimeMonIndex;
        m_pTimeMonIndex = NULL;
    }

    // Allocate timeout monitor array
    m_pTimeMon = new TIME_MON_ENTRY_TYPE[m_MaxIocHostModuleAddr+1];
    if (!m_pTimeMon) {
        TRC_OUT(GR_IOR, LV_ERR, "Timeout monitor array could not be allcated");
        return false;
    }

    // Allocate timeout monitor index array (index array is used for fast access to timeout monitor array)
    m_pTimeMonIndex = new TIME_MON_INDEX_ENTRY_TYPE[m_NumOfIocHostModules];
    if (!m_pTimeMonIndex) {
        TRC_OUT(GR_IOR, LV_ERR, "Timeout monitor index array could not be allcated");
        delete [] m_pTimeMon;
        m_pTimeMon = NULL;
        return false;
    }

    // Initialize timeout monitor index array
    for (int j=0, i=0; j<NumOfModules; j++)  // Iterate over concentrator table
    {
        pTableEntry = m_pIoc->getIocModuleAtIdx(j);
        if(pTableEntry){
          if (pTableEntry->NumOfSlices != 0)   // sliced submodule?
          {
              (m_pTimeMonIndex+i)->addr = pTableEntry->LogAddr;        // store logical address of sliced module
              (m_pTimeMonIndex+i)->pTimeMonEntry = m_pTimeMon+(pTableEntry->LogAddr); // store pointer to element in timeout monitor array
              i++;
          }
        }
        else{
          TRC_OUT01(GR_IOR, LV_ERR, "pTableEntry = 0, module index %d", j);
          DPR_ASSERT(0);
        }
    }

    // Trace sliced modules
    TRC_IF_ON_EXPR(GR_IOR, LV_INFO,
        if (m_NumOfIocHostModules) {
        OSTREAM trcos;
        trcos << "Transfer watchdog assigned to modules";
        for (PNIO_UINT32 i=0; i< m_NumOfIocHostModules; i++) {
                trcos << " " << (m_pTimeMonIndex+i)->addr;
            }
            trcos << ends;
            TRC_OUT_OBJECT(GR_IOR, LV_INFO, trcos);
        }
    );

    // internal state: waiting for arrival of watchdog time
    DPR_MUTEX_LOCK(PnioMutexTransferWd);
    m_state = TWD_WAIT_WD_TIME;
    DPR_MUTEX_UNLOCK(PnioMutexTransferWd);

    TRC_OUT(GR_IOR, LV_FCTINT, "<- CTransferWD::Init");

    return true;
}

/*-----------------------------------------------------------------------------
 * Name  : Uninit
 * Descr : deallocate buffers
 * Param :
 *  [ IN]:
 *  [OUT]:
 * Return:
 */
void  CTransferWD::Uninit()
{
    DPR_MUTEX_LOCK(PnioMutexTransferWd);

    TRC_OUT(GR_IOR, LV_FCTINT, "-> CTransferWD::Uninit");

    m_state = TWD_UNINIT;

    // deallocate buffers
    if (m_pTimeMon) {
        delete [] m_pTimeMon;
        m_pTimeMon = NULL;
    }
    if (m_pTimeMonIndex) {
        delete [] m_pTimeMonIndex;
        m_pTimeMonIndex = NULL;
    }

    TRC_OUT(GR_IOR, LV_FCTINT, "<- CTransferWD::Uninit");

    DPR_MUTEX_UNLOCK(PnioMutexTransferWd);
}

/*-----------------------------------------------------------------------------
 * Name  : Clock
 * Descr : Monitoring activity, function is triggered by new cycle interrupt
 * Param :
 *  [ IN]:
 *  [OUT]:
 * Return:
 */
void CTransferWD::Clock(PNIO_CP_CBE_PRM * prm)
{
    DPR_MUTEX_LOCK(PnioMutexTransferWd);

    switch (m_state) {
    case TWD_WAIT_WD_TIME:
        if (++m_clockCount > m_pollRate) {
            m_clockCount = 0;
            if (m_pIoc->getTwdValidity()) {
                InitWatchdogTimer();
                if (m_NumOfIocHostModules != 0) {
                    // Reset timeout monitor array and start watchdog
                    CacheResetNoLock();
                    KramResetNoLock();
                    m_clockCount = 0;
                    m_state = TWD_RUN;
                    TRC_OUT(GR_IOR, LV_FCTINT, "Transfer watchdog is monitoring transfer of io-data");
                } else {
                    TRC_OUT(GR_IOR, LV_WARN, "Transfer watchdog start requested but no sliced modules available");
                }
            }
        }
        break;

    case TWD_RUN:
        m_clockCount++;

        // Watchdog interval is subdivided to avoid cpu load peaks.
        // Checking all modules belonging to this interval fragment:
        for (PNIO_UINT32 i=0; i< m_NumOfIocHostModulePerCycle && m_actIocHostModuleIndex < m_NumOfIocHostModules; i++) {
            if (!((m_pTimeMonIndex+m_actIocHostModuleIndex)->pTimeMonEntry)->kramSet) {

                TRC_OUT02(GR_IOR, LV_TIMECRIT, "Transfer watchdog triggers log addr %d. ERTEC cycle count: %d",
                    (m_pTimeMonIndex+m_actIocHostModuleIndex)->addr, prm->u.NewCycle.CycleInfo.CycleCount);
                // io-data not up to date -> update module
                m_pIoc->IOC_watchdog_data_write((m_pTimeMonIndex+m_actIocHostModuleIndex)->addr);
            }
            // Reset flags
            ((m_pTimeMonIndex+m_actIocHostModuleIndex)->pTimeMonEntry)->kramSet = false;
            ((m_pTimeMonIndex+m_actIocHostModuleIndex)->pTimeMonEntry)->cacheSet = false;

            // go to next sliced submodule
            m_actIocHostModuleIndex++;
        }
        if (m_clockCount >= m_reduction) {
            // reset clock counter
            m_clockCount = 0;
            // all submodules should be updated by now -> reset submodule index
            m_actIocHostModuleIndex = 0;
        }
        break;

    default:
        break;
    }
    DPR_MUTEX_UNLOCK(PnioMutexTransferWd);
}


/*-----------------------------------------------------------------------------
 * Name  : CacheSet
 * Descr : inform watchdog of io data cache update - for instance in PNIO_data_write_cache(..)
 * Param :
 *  [ IN]: logical address of ioc host module whose cache was updated
 *  [OUT]:
 * Return:
 */
void CTransferWD::CacheSet(PNIO_UINT32 addr) {
    DPR_MUTEX_LOCK(PnioMutexTransferWd);
    TRC_OUT01(GR_IOR, LV_TIMECRIT, "Transfer watchdog was informed of cache set: log address %d", addr);
    if ((m_state == TWD_INIT)||(m_state == TWD_UNINIT) ) {
        DPR_MUTEX_UNLOCK(PnioMutexTransferWd);
        return;
    }
    DPR_ASSERT(addr <= m_MaxIocHostModuleAddr);
    (m_pTimeMon+addr)->cacheSet = true;
    DPR_MUTEX_UNLOCK(PnioMutexTransferWd);
}

/*-----------------------------------------------------------------------------
 * Name  : CacheSet
 * Descr : informs watchdog of io data cache update: all modules
 * Param :
 *  [ IN]:
 *  [OUT]:
 * Return:
 */
void CTransferWD::CacheSet()
{
    DPR_MUTEX_LOCK(PnioMutexTransferWd);
    TRC_OUT(GR_IOR, LV_TIMECRIT, "Transfer watchdog was informed of cache set: all sliced modules");
    if ((m_state == TWD_INIT)||(m_state == TWD_UNINIT) ) {
        DPR_MUTEX_UNLOCK(PnioMutexTransferWd);
        return;
    }
    for (PNIO_UINT32 i=0; i<m_NumOfIocHostModules; i++) {
        ((m_pTimeMonIndex+i)->pTimeMonEntry)->cacheSet = true; // access to timeout monitor array via index array
    }
    DPR_MUTEX_UNLOCK(PnioMutexTransferWd);
}

/*-----------------------------------------------------------------------------
 * Name  : CacheReset
 * Descr : reset cache monitor flag at the beginning of a new timeout interval
 * Param :
 *  [ IN]: logical address of ioc host module whose cache monitor flag is to be reset
 *  [OUT]:
 * Return:
 */
void CTransferWD::CacheReset(PNIO_UINT32 addr)
{
    DPR_MUTEX_LOCK(PnioMutexTransferWd);
    TRC_OUT01(GR_IOR, LV_TIMECRIT, "Transfer watchdog was informed of cache reset: log address %d", addr);
    if ((m_state == TWD_INIT)||(m_state == TWD_UNINIT) ) {
        DPR_MUTEX_UNLOCK(PnioMutexTransferWd);
        return;
    }
    DPR_ASSERT(addr <= m_MaxIocHostModuleAddr);
    (m_pTimeMon+addr)->cacheSet = false;
    DPR_MUTEX_UNLOCK(PnioMutexTransferWd);
}

/*-----------------------------------------------------------------------------
 * Name  : CacheReset
 * Descr : reset all cache monitor flags at the beginning of a new timeout interval
 * Param :
 *  [ IN]:
 *  [OUT]:
 * Return:
 */
void CTransferWD::CacheReset()
{
    DPR_MUTEX_LOCK(PnioMutexTransferWd);
    CacheResetNoLock();
    DPR_MUTEX_UNLOCK(PnioMutexTransferWd);
}

/*-----------------------------------------------------------------------------
 * Name  : KramSet
 * Descr : informs watchdog of io data update in kram, for instance in PNIO_data_write
 * Param :
 *  [ IN]: logical address of updated ioc host module
 *  [OUT]:
 * Return:
 */
void CTransferWD::KramSet(PNIO_UINT32 addr)
{
    DPR_MUTEX_LOCK(PnioMutexTransferWd);
    TRC_OUT01(GR_IOR, LV_TIMECRIT, "Transfer watchdog was informed of kram set: log address %d", addr);
    if ((m_state == TWD_INIT)||(m_state == TWD_UNINIT) ) {
        DPR_MUTEX_UNLOCK(PnioMutexTransferWd);
        return;
    }
    DPR_ASSERT(addr <= m_MaxIocHostModuleAddr);
    (m_pTimeMon+addr)->kramSet = true;
    DPR_MUTEX_UNLOCK(PnioMutexTransferWd);
}

/*-----------------------------------------------------------------------------
 * Name  : KramSet
 * Descr : informs watchdog of io data update in kram. All ioc host modules
 *         have been updated, for instance in PNIO_data_write_cache_flush
 * Param :
 *  [ IN]:
 *  [OUT]:
 * Return:
 */
void CTransferWD::KramSet()
{
    DPR_MUTEX_LOCK(PnioMutexTransferWd);
    TRC_OUT(GR_IOR, LV_TIMECRIT, "Transfer watchdog was informed of kram set: all sliced modules");
    if ((m_state == TWD_INIT)||(m_state == TWD_UNINIT) ) {
        DPR_MUTEX_UNLOCK(PnioMutexTransferWd);
        return;
    }
    for (PNIO_UINT32 i=0; i<m_NumOfIocHostModules; i++) {
        ((m_pTimeMonIndex+i)->pTimeMonEntry)->kramSet = false; // access to timeout monitor array  via index array
    }
    DPR_MUTEX_UNLOCK(PnioMutexTransferWd);
}

/*-----------------------------------------------------------------------------
 * Name  : KramReset
 * Descr : reset kram monitor flag at the beginning of a new timeout interval
 * Param :
 *  [ IN]: logical address of ioc host module whose kram monitor flag is to be reset
 *  [OUT]:
 * Return:
 */
void CTransferWD::KramReset(PNIO_UINT32 addr)
{
    DPR_MUTEX_LOCK(PnioMutexTransferWd);
    TRC_OUT01(GR_IOR, LV_TIMECRIT, "Transfer watchdog was informed of kram reset: log address %d", addr);
    if ((m_state == TWD_INIT)||(m_state == TWD_UNINIT) ) {
        DPR_MUTEX_UNLOCK(PnioMutexTransferWd);
        return;
    }
    DPR_ASSERT(addr <= m_MaxIocHostModuleAddr);
    (m_pTimeMon+addr)->cacheSet = false;
    DPR_MUTEX_UNLOCK(PnioMutexTransferWd);
}

/*-----------------------------------------------------------------------------
 * Name  : KramReset
 * Descr : reset all cache monitor flags at the beginning of a new timeout interval
 * Param :
 *  [ IN]:
 *  [OUT]:
 * Return:
 */
void CTransferWD::KramReset()
{
    DPR_MUTEX_LOCK(PnioMutexTransferWd);
    KramResetNoLock();
    DPR_MUTEX_UNLOCK(PnioMutexTransferWd);
}

/*-----------------------------------------------------------------------------
 * Name  : IsCacheSet
 * Descr : Check if io data cache has been updated within current watchdog interval
 * Param :
 *  [ IN]:
 *  [OUT]:
 * Return: Logical address of module to check
 */
bool CTransferWD::IsCacheSet(PNIO_UINT32 addr)
{
    DPR_MUTEX_LOCK(PnioMutexTransferWd);
    TRC_OUT01(GR_IOR, LV_TIMECRIT, "Transfer watchdog was queried about cache flag: log address %d", addr);
    if ((m_state == TWD_INIT)||(m_state == TWD_UNINIT) ) {
        DPR_MUTEX_UNLOCK(PnioMutexTransferWd);
        return false;
    }
    DPR_ASSERT(addr <= m_MaxIocHostModuleAddr);
    DPR_MUTEX_UNLOCK(PnioMutexTransferWd);
    return (m_pTimeMon+addr)->cacheSet;
}

/*-----------------------------------------------------------------------------
 * Name  : CacheResetNoLock
 * Descr : reset all cache monitor flags at the beginning of a new timeout interval
 * Param :
 *  [ IN]:
 *  [OUT]:
 * Return:
 */
void CTransferWD::CacheResetNoLock()
{
    TRC_OUT(GR_IOR, LV_TIMECRIT, "Transfer watchdog was informed of cache reset: all sliced modules");
    if ((m_state == TWD_INIT)||(m_state == TWD_UNINIT) ) {
        return;
    }
    for (PNIO_UINT32 i=0; i<m_NumOfIocHostModules; i++) {
        ((m_pTimeMonIndex+i)->pTimeMonEntry)->cacheSet = false; // access to timeout monitor array  via index array
    }
}

/*-----------------------------------------------------------------------------
 * Name  : KramResetNoLock
 * Descr : reset all cache monitor flags at the beginning of a new timeout interval
 * Param :
 *  [ IN]:
 *  [OUT]:
 * Return:
 */
void CTransferWD::KramResetNoLock()
{
    TRC_OUT(GR_IOR, LV_TIMECRIT, "Transfer watchdog was informed of kram reset: all sliced modules");
    if ((m_state == TWD_INIT)||(m_state == TWD_UNINIT) ) {
        return;
    }
    for (PNIO_UINT32 i=0; i<m_NumOfIocHostModules; i++) {
        ((m_pTimeMonIndex+i)->pTimeMonEntry)->kramSet = false; // access to timeout monitor array  via index array
    }
}

/*-----------------------------------------------------------------------------
 * Name  : InitWatchdogTimer
 * Descr : read watchdog time from ioc table, set corresponding watchdog
 *         parameters. Called after connect from upper controller.
 * Param :
 *  [ IN]:
 *  [OUT]:
 * Return:
 */
void CTransferWD::InitWatchdogTimer()
{
    // set reduction ration of new cycle interrupt
    m_newCycleIrqTime = m_pIoc->getIocTabHdr()->NewCycleIrqTime;

    m_wdTime = m_pIoc->getMinTwdTime();
    if (m_wdTime < m_newCycleIrqTime) {
        TRC_OUT02(GR_IOR, LV_ERR, "Transfer watchdog interval %u*31,25us too small. Defaulting to %u*31,25us",m_wdTime, m_newCycleIrqTime);
        m_wdTime = m_newCycleIrqTime;
    }
    m_reduction = m_wdTime / m_newCycleIrqTime;

    // calculate number of submodules that are updated within each new cycle interrupt
    m_NumOfIocHostModulePerCycle = m_NumOfIocHostModules / m_reduction;
    if (m_NumOfIocHostModules % m_reduction) {   // round up
        m_NumOfIocHostModulePerCycle++;
    }

    TRC_OUT02(GR_IOR, LV_INFO, "Watchdog monitor interval: %d*31,25us, reduction ratio: %d", m_wdTime, m_reduction);
}

#endif // IO_ROUTER
