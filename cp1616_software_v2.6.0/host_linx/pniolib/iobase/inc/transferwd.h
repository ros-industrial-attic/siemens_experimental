/****************************************************************************
*  Copyright (C) SIEMENS CORP., 2013 All rights reserved.
*****************************************************************************
* FILE NAME    : transferwd.h
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
#ifndef TRANSFERWD_H_
#define TRANSFERWD_H_

#include "pniointr.h"

class IController;

class CTransferWD
{
public:
    CTransferWD();
    virtual ~CTransferWD();

    bool Init(IoConcentrator * pIoc, IController * pController);
    void Uninit();
    void Clock(PNIO_CP_CBE_PRM * prm);

    // Interface to KRAM and Cache Flags
    void CacheSet(PNIO_UINT32 addr);        /* set flag "cache is up to date" of specified submodule */
    void CacheSet();                        /* set flag "cache is up to date" of all submodules */
    void CacheReset(PNIO_UINT32 addr);      /* reset flag "cache is up to date" of specified submodule */
    void CacheReset();                      /* reset flag "cache is up to date" of all submodules */
    void KramSet(PNIO_UINT32 addr);         /* set flag "KRAM is up to date" of specified submodule */
    void KramSet();                         /* set flag "KRAM is up to date" of all submodules */
    void KramReset(PNIO_UINT32 addr);       /* reset flag "KRAM is up to date" of specified submodule */
    void KramReset();                       /* reset flag "KRAM is up to date" flag of all submodules */
    bool IsCacheSet(PNIO_UINT32 addr);      /* return true if flag "cache is up to date" of specified submodule is set */

private:

    typedef struct
    {
        bool        cacheSet;
        bool        kramSet;
    }
    TIME_MON_ENTRY_TYPE;

    typedef struct
    {
        PNIO_UINT32            addr;            // logical address
        TIME_MON_ENTRY_TYPE *  pTimeMonEntry;  // pointer to element in timeout monitor array
    }
    TIME_MON_INDEX_ENTRY_TYPE;

    typedef enum {
        TWD_INIT,         // Initial state
        TWD_WAIT_WD_TIME, // Waiting for valid watchdog time in conc table
        TWD_READY,        // Ready for monitor start
        TWD_RUN,          // Monitoring modules...
        TWD_STOP,         // Monitoring stopped, buffers still allocated, ready for monitor restart
        TWD_UNINIT        // Uninitializing, deallocating buffers
    } TWD_STATE_TYPE;

    static const PNIO_UINT32 m_pollRate = 5;

    void                InitWatchdogTimer(void);
    void 				CacheResetNoLock();                
    void 				KramResetNoLock();                 
    IoConcentrator *    m_pIoc;                     // Pointer to io-concentrator instance
    IController    *    m_pController;              // Pointer to controller instance

    // Timeout monitor array - contains all sliced modules
    // Index is logical address
    TIME_MON_ENTRY_TYPE * m_pTimeMon;               
    // Index array - used for fast iteration over all sliced modules.
    // Elements contain pointers to the sliced submodules in timeout monitor array
    // Index of index array is NOT logical address of submodules
    TIME_MON_INDEX_ENTRY_TYPE * m_pTimeMonIndex;    

    PNIO_UINT32 m_NumOfIocHostModulePerCycle;
    PNIO_UINT32 m_actIocHostModuleIndex;

    PNIO_UINT32 m_NumOfIocHostModules;  // number of sliced modules
    PNIO_UINT32 m_MaxIocHostModuleAddr; // largest logical address of sliced modules

    PNIO_UINT32 m_reduction;            // reduction ratio of new cycle interrupt
    PNIO_UINT32 m_clockCount;           // reduction counter (counts 0,1,...,m_reduction,0,1,....)
    PNIO_UINT32 m_wdTime;               // watchdog interval
    PNIO_UINT32 m_newCycleIrqTime;      // new cycle interrupt interval

    TWD_STATE_TYPE  m_state;            // internal transfer watchdog state
};

#endif /*TRANSFERWD_H_*/
