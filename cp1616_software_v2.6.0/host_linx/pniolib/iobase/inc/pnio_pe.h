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

#ifndef _PNIO_PE_H
#define _PNIO_PE_H

#include "os.h"
#include "pnio_pe_util.h"

class ICommon;
class cPeTimer;
class cPeReq;

///////////////////////////////////////////////////////////////////////////////////////
///
///  PROFIenergy Command Manager
///

class cPeMgt {

    //friend class cPnioDrRqb;

    int           m_TickCount;
    //cPeRqQueue    m_PeRqQ(cPeMgt& r); // ref to this class
    cPeRqQueue    m_PeRqQ; // ref to this class

public:
    cPeMgt(IController* pCtrl);
    ~cPeMgt();

    void handleTimerCallback();
    void init();
    void uninit();
    //int  add_pe_request(cPeReq *rq);
    PNIO_UINT32 handle_pe_cmd_request(PNIO_UINT32 Handle, PNIO_ADDR *pAddr, PNIO_REF user_ref, PNIO_PE_REQ_PRM *pPeReqPrm);
    void handle_pe_dr_resp(const t_read_write_dr_resp *pDrRq);
    void handle_pe_dr_read_resp(const t_read_write_dr_resp *pDrRq);
    void handle_pos_read_resp(PNIO_ADDR *pnio_addr, PNIO_REF ReqRef,
                              PNIO_ERR_STAT &e, PNIO_UINT32 len, PNIO_UINT8* pbuff);
    void handle_neg_read_resp(PNIO_ADDR *pnio_addr, PNIO_REF ReqRef, PNIO_ERR_STAT &e,
                              PNIO_UINT32 len, PNIO_UINT8* pbuff);
    void handle_pe_dr_write_resp(const t_read_write_dr_resp *pDrRq);
    void handle_pos_write_resp(PNIO_ADDR *pnio_addr, PNIO_REF user_ref);
    void handle_neg_write_resp(PNIO_ADDR *pnio_addr, PNIO_REF user_rq_ref, PNIO_ERR_STAT &e);
    void increment_req_err_busy_count(PNIO_REF user_rq_ref, PNIO_UINT32 len, PNIO_UINT8* buff);
    void pass_resp_to_user(PNIO_ADDR *pnio_addr, PNIO_REF user_ref, PNAGC_DR_RQ_TYPE dr_rw_action_type,
                           PNIO_ERR_STAT &e, PNIO_UINT32 len, PNIO_UINT8* buff);


    IController  *m_pCtrl;              // back link to outer class
    cPeServiceRef m_PeServiceRef;       // initial size == max size
    cPeTimer     *m_pTimer;             // timer active object (thread)
    PNIO_PE_CBF   m_pUserCbf_PE;        // user PROFIenergy callback function pointer
};



///////////////////////////////////////////////////////////////////////////////////////
///
///
class cPeTimer {

public:
    enum {
        st_idle,
        st_running
    }  m_state;

    cPeMgt         *m_pPeMgt;    // link to parent
    uint16_t        m_TimerId;
    int             m_time_ms;

    static void rawCallback (uint16_t TimerId, cPeTimer* pPeT)
    {
        cPeTimer *p = pPeT;
        p->m_pPeMgt->handleTimerCallback ();
    }

    //friend static DPR_THREAD_RETURN_TYPE DPR_THREAD_DECL peTimerThEntry(void *arg);

public:

    cPeTimer( cPeMgt* p_parent, int t_ms, bool cyclic = true);
    ~cPeTimer();

    void start (void);
    void stop  (void);

    DPR_UINT16             m_stop_thread; /* flag to stop the thread */
    DPR_THREAD_HANDLE      m_th;          /* thread handle */

};  // cPeTimer




#endif /* _PNIO_PE_H  */

