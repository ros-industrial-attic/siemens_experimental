/*****************************************************************************
*  Copyright (C) SIEMENS CORP., 2013 All rights reserved.
******************************************************************************
*/
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
/*-----------------------------------------------------------------------------
 *  Project     : PROFINET IO
 *  Package     : CP1616 DK
 *  Component   : PROFIenergy ( PE )
 *-----------------------------------------------------------------------------
 *
 *  D e s c r i p t i o n:
 *
 *-----------------------------------------------------------------------------
 *
 *  H i s t o r y:
 *  12.10.2010 PL: first implementation
 *
 *----------------------------------------------------------------------------*/
#ifdef PROFI_ENERGY
#include "pniointr.h"

//#include "tracesub.h"
//#include "tracemcr.h"

#include "pnio_pe_util.h"

///////////////////////////////////////////////////////////////////////////////////////
/// cPeRqQueue
///
/// Note: Removing elemens from a MAP !!!
/// When removing elements, be careful not to saw off the branch on which you are sitting.
/// There is a big danger that will you remove an element to which your iterator is referring.
/// For example: remove all elements having a certain value:
///   for (pos = c.begin(); pos != c.end(); ) {
///       if (pos->second == value) {
///           c.erase(pos++);
///       }
///       else {
///           ++pos;
///       }
///   }
/// Note that pos++ increments pos so that it refers to the next element but yields a copy of its original value.
/// Thus, pos doesn't refer to the element that is removed when erase() is called.
///



cPeRqQueue::cPeRqQueue(cPeMgt& r) : m_cPeMgtBackref(r)
{
    TRC_OUT(GR_PE, LV_INFO, "->cPeRqQueue::Ctor m_cPeMgtBackref");
}

cPeRqQueue::~cPeRqQueue()
{
    RqListIterator rq_position;

    TRC_OUT(GR_PE, LV_INFO, "->cPeRqQueue::Dtor");
    if ( !m_Rqs.empty() ) {

        Lock pe_lock(m_PeReqMutex);  // <<<<<< lock

        cPeReq* p_rq = NULL;
        for (rq_position = m_Rqs.begin(); rq_position != m_Rqs.end(); ++rq_position) {
            TRC_OUT02(GR_PE, LV_INFO, "Q del element: %#x %#x ",rq_position->first, rq_position->second);
            p_rq = rq_position->second;
            delete p_rq;
            rq_position->second = NULL;
        }
    }
    TRC_OUT(GR_PE, LV_INFO, "<-cPeRqQueue::Dtor");
}


int cPeRqQueue::add_rq(cPeReq* p_rq)
{
    RqKeyType k = p_rq->get_key();
    TRC_OUT01(GR_PE, LV_INFO, "->Q add_rq Key=%#x",k);
    Lock pe_lock(m_PeReqMutex);
    m_Rqs.insert(std::make_pair(k, p_rq));
    return 0;
}

cPeReq* cPeRqQueue::find_rq(RqKeyType k)
{
    cPeReq* p_rq = NULL;
    RqListIterator rq_position = m_Rqs.end();

    TRC_OUT01(GR_PE, LV_INFO, "->Q find_rq Key=%#x",k);
    Lock pe_lock(m_PeReqMutex);

    rq_position = m_Rqs.find(k);
    if (rq_position != m_Rqs.end() ) {
        // found
        p_rq = rq_position->second;
        TRC_OUT01(GR_PE, LV_INFO, "<-Q FOUND rq Key=%#x",k);
    }
    else {
        p_rq = NULL;
        TRC_OUT01(GR_PE, LV_INFO, "<-Q NOT found rq Key=%#x",k);
    }
    return p_rq;
}

/*-----------------------------------------------------------------------------
 * Name  : cPeRqQueue::remove_rq
 * Descr :
 * Param :
 *  [ IN]:
 *  [OUT]:
 * Return: Rq pointer: == NULL: Not found, != NULL: OK, Ptr to found Rq obj.
 */
cPeReq* cPeRqQueue::remove_rq(RqKeyType ref)
{
    cPeReq* p_rq = NULL;
    RqListIterator rq_position = m_Rqs.end();

    TRC_OUT01(GR_PE, LV_INFO, "->Q remove_rq %#x",ref);
    Lock pe_lock(m_PeReqMutex);
    rq_position = m_Rqs.find(ref);
    if (rq_position != m_Rqs.end() ) {
        // found
        p_rq = rq_position->second;
        m_Rqs.erase(rq_position);
        TRC_OUT01(GR_PE, LV_INFO, "<-Q remove_rq: OK REMOVED Key=%#x",ref);
    }
    else {
        // not found
        TRC_OUT01(GR_PE, LV_ERR, "<-Q remove_rq: REF NOT FOUND Key=%#x",ref);
    }
    return p_rq;
} /* end of cPeRqQueue::remove_rq */


/*-----------------------------------------------------------------------------
 * Name  : timeout_chk
 * Descr :
 */
int cPeRqQueue::timeout_chk(void)
{
    int result = 0;
    int qsize = m_Rqs.size();
    if (qsize == 0) {
        TRC_OUT(GR_PE, LV_INFO, "<->Q timeout_chk empty (no action) OK");
        return result;
    }
    TRC_OUT01(GR_PE, LV_INFO, "->Q timeout_chk num-elements=%d",qsize);

    RqListIterator rq_position;
    cPeReq*        p_rq = NULL;
    RqKeyType      k;
    PNIO_ADDR      pnioAddr;
    PNIO_REF       currUserReqRef;
    PNIO_UINT8     currCmdId;
    PNIO_UINT8     currCmdModifier;
    //PNIO_UINT8     currState    = 0x02;
    //PNIO_UINT8     currStructId = 0xff;
    //PNIO_ERR_STAT  e = {0,0};
    PNIO_UINT8     myErrCode = 0xDE;     // ReadResp.

    // user callback param struct for negative confirmation
    PNIO_UINT8  buff[ sizeof(PE_CBE_HDR) + sizeof(PNIO_PE_PRM_GENERIC_CONF_NEG)];
    PNIO_PE_CBE_PRM *p_cb = (PNIO_PE_CBE_PRM*)&buff[0];
    memset(&buff[0], 0, sizeof(buff));

    Lock pe_lock(m_PeReqMutex);   //  LOCK the Queue for the whole loop <<<<<<<<<<<    LOCK    <<<<<<<<<

    // because of 'eraseing' elements at iterator position do NOT increment iterator in the for(...) statement!!!
    // =>RUNTIME ERROR: Iterator is invalid after eraseing element
    //
    for (rq_position = m_Rqs.begin(); rq_position != m_Rqs.end(); /* ++rq_position */ ) {
        k    = rq_position->first;
        p_rq = rq_position->second;
        if (p_rq->lifetime_sec > 0) {
            p_rq->lifetime_sec--;
        }
        else {
            TRC_OUT01(GR_PE, LV_ERR, " timeout_chk: ERROR lifetime= %d",p_rq->lifetime_sec);
            p_rq->lifetime_sec = 0;
        }

        PNIO_UINT32 logAddr = p_rq->log_addr;

        pnioAddr.AddrType   = PNIO_ADDR_LOG;
        pnioAddr.IODataType = PNIO_IO_IN;
        pnioAddr.u.Addr     = logAddr;


        if (p_rq->lifetime_sec == 0) {
            // time out: return negative response to user
            //-----------

            // !!! increment iterator befor delete the element at current position !!!
            // here: rq_position is incremented, but erease works with 'old' value !!!
            // Note: we can erase the elem ptr from the map now, we still have the ptr
            //       and will delete elem after we are done with it. See below
            m_Rqs.erase(rq_position++);


            currUserReqRef = p_rq->user_ref;   // originaly saved user-ref
            currCmdId      = p_rq->cmd_id;
            currCmdModifier= p_rq->cmd_modifier;

            PNIO_UINT8  serv_ref = p_rq->serv_ref;  // debug aim only

            TRC_OUT04(GR_PE, LV_INFO, "->Q timeout_chk: TO: UsrRef=%#x serv-ref=%#x CmdId=%d Modif=%d",
                      currUserReqRef, k, currCmdId, currCmdModifier);

            // free service-ref, return it back to the q
            m_cPeMgtBackref.m_PeServiceRef.put_ref(k);
            delete p_rq;
            p_rq = NULL;

            // fill neg. response parameter struct. =callback parm. buffer
            p_cb->pe_hdr.CmdId           = (PNIO_PE_CMD_ENUM)currCmdId;
            p_cb->pe_hdr.Addr.AddrType   = PNIO_ADDR_LOG;      // == 0
            p_cb->pe_hdr.Addr.IODataType = PNIO_IO_IN;         // == 0
            p_cb->pe_hdr.Addr.u.Addr     = logAddr;

            p_cb->pe_hdr.Ref             = currUserReqRef;
            p_cb->pe_hdr.Err.ErrCode     = myErrCode;          // dr read =0xDE;
            p_cb->pe_hdr.Err.ErrDecode   = 0x80;               // PNIORW
            p_cb->pe_hdr.Err.ErrCode1    = 0xA0;  // == appl. read err,  0xC3 == Resource unavailable

            p_cb->pe_hdr.state           = 0x02;  // resp available with error
            p_cb->pe_hdr.struct_id       = 0xFF;  // response struct_id

            //p_cb->pe_hdr.length          = 0;  // no more parameters
            p_cb->pe_hdr.length          = sizeof(PNIO_PE_PRM_GENERIC_CONF_NEG); // no conf pdu present

            // this params have no meaning - only to provide space
            p_cb->pe_prm.RespNegative.err_code = 0xFF; // err_code without meaning
            p_cb->pe_prm.RespNegative.reserved = 0xEE; // err_code without meaning


            // check if PE callback is already registered - call it
            if (m_cPeMgtBackref.m_pUserCbf_PE) {
                TRC_OUT01(GR_PE, LV_INFO, " cPeMgt: --> user PE Conf- TIMEOUT CmdId=%#x",p_cb->pe_hdr.CmdId  );
                m_cPeMgtBackref.m_pUserCbf_PE( p_cb );
                TRC_OUT01(GR_PE, LV_INFO, " cPeMgt: <-- user PE Conf- callback done! CmdId=%#x",p_cb->pe_hdr.CmdId );
            }
            // buff is 'auto' - no free is necessary

        }
        else {
            // retry: ask for service response again. Send a new 'DataRecord Read Request'
            //--------
            // But only if previous 'DataRecord Read Request' returned 'ok busy' retrigger req.
            // If we are already waiting for a response, do nothing, wait for timeout.

            // check the state of the request
            if ( p_rq->m_state == cPeReq::st_rd_busy ) {
                // previous read req. returned 'ok-busy' so we have to retry
                // create 'DataRecord Read Request PDU' to request for a 'pe response'
                int used_buff_len = 0;                        // max possible buffer len
                PNIO_UINT8  buff[sizeof(PE_PDU_DR_READ_RQ)];
                PE_PDU_DR_READ_RQ *pPdu = (PE_PDU_DR_READ_RQ*)&buff[0]; // pointer to pdu header
                memset(&buff[0], 0, sizeof(buff));                      // clear buffer content

                PNIO_UINT16   blk_type    = PE_DR_BLKTYPE_READ;    /* 0x0800 RecordDataWrite, 0x0801 RecordDataRead  */
                PNIO_UINT16   blk_length  = sizeof(PE_PDU_DR_READ_RQ) - (sizeof(pPdu->dr_hdr.block_type) +
                                                                         sizeof(pPdu->dr_hdr.block_length) );

                pPdu->dr_hdr.block_type   = CPU_TO_BE(blk_type);
                pPdu->dr_hdr.block_length = CPU_TO_BE(blk_length);
                pPdu->dr_hdr.version_high = PE_DR_BLKVERS_HIGT;
                pPdu->dr_hdr.version_low  = PE_DR_BLKVERS_LOW;

                pPdu->pe_hdr.service_id   = p_rq->cmd_id;
                pPdu->pe_hdr.rq_ref       = p_rq->serv_ref;
                pPdu->pe_hdr.modif_state  = p_rq->cmd_modifier;
                pPdu->pe_hdr.struct_id    = 0x00;

                used_buff_len = PNIOI_DREC_MAX_SIZE;    // DataRecord MAX possible length

                // use standard data record read/write function to send this service request
                // Note: service-ref is used as "userRef" in the standard DataRecord read request.
                result = m_cPeMgtBackref.m_pCtrl->perf_ds(&pnioAddr, ACC_T_READ,
                                                          p_rq->serv_ref, PE_SAP_DR_INDEX,/*0x80A0*/
                                                          used_buff_len, &buff[0]
                                                          );
                if (result != PNIO_OK) {
                    // error
                    TRC_OUT02(GR_PE, LV_ERR, " timeout_chk: ERROR perf_ds ret=%#x lifetime_sec=%d",result,p_rq->lifetime_sec);

                    // we did not reach timeout of this PeReq so we can ignore this error and
                    // wait for next retry. -> no action - let PeReq state equal cPeReq::st_rd_busy
                    // => we retry at next tick

                }
                else {
                    // ok, set rq state
                    p_rq->set_state(cPeReq::st_rd_pend);
                }
            } // end if m_state == st_rd_busy

            // set iterator to the next rq - must be done here, not possible in the for(...). See 'erase' above
            rq_position++;
            TRC_OUT01(GR_PE, LV_INFO, "->Q timeout_chk: lifetime_sec=%d", p_rq->lifetime_sec);
        }

    } // end for all PeReqs

    TRC_OUT(GR_PE, LV_INFO, "<-Q timeout_chk");
    return result;
} /* end of timeout_chk */



///////////////////////////////////////////////////////////////////////////////////////
/// cPeServiceRef
///
cPeServiceRef::cPeServiceRef(int init_size)
{
    TRC_OUT01(GR_PE, LV_INFO, "<->cPeServiceRef::Ctor siz=%d",init_size);
    for (int i=1; i <= init_size; i++) {
        m_Refs.push_back((PNIO_UINT8)i);
    }
}

cPeServiceRef::~cPeServiceRef()
{
    TRC_OUT01(GR_PE, LV_INFO, "<->cPeServiceRef::Dtor siz=%d",m_Refs.size() );
}

// get_ref:  0: not available, 1..254: valid ref
PNIO_UINT8 cPeServiceRef::get_ref()
{
    PNIO_UINT8 el = 0;   // preset to invalid element
    Lock ref_lock(m_PeRefMutex);
    if ( !m_Refs.empty() ) {
        el = m_Refs.front();
        m_Refs.pop_front();
    }
    TRC_OUT01(GR_PE, LV_INFO, "<->cPeServiceRef::get_ref: el=%d",el );
    return el;
}

void cPeServiceRef::put_ref(PNIO_UINT8 ref)
{
    TRC_OUT01(GR_PE, LV_INFO, "<->cPeServiceRef::put_ref: ref=%d",ref );
    Lock ref_lock(m_PeRefMutex);
    m_Refs.push_back(ref);
}

int  cPeServiceRef::size()
{
    return m_Refs.size();
}

#endif /* PROFI_ENERGY */

