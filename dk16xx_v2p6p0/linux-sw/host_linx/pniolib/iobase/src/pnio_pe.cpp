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
#ifdef PROFI_ENERGY

#include "pniointr.h"

//#include "pnio_pe.h"

#if !defined (_PE_ALIGN_TEST_DONE_)
//PE_STATIC_ASSERT(sizeof(ALIGN_TST) == 7); //lint !e92  Negative array dimension or bit field length (-1)
// if error occurs in the line above -> byte alignment is wrong. Byte alignment is required !!!
#define _PE_ALIGN_TEST_DONE_
#endif


///////////////////////////////////////////////////////////////////////////////////////
/// cPeMgt
///
cPeMgt::cPeMgt(IController* pCtrl) :
  m_TickCount(0),
  m_PeRqQ(*this),
  m_PeServiceRef(254)

{
    m_pCtrl       = pCtrl;
    m_pTimer      = NULL;
    m_pUserCbf_PE = NULL;
    TRC_OUT(GR_PE, LV_INFO, "->cPeMgt::Ctor ");

}

cPeMgt::~cPeMgt()
{
    TRC_OUT(GR_PE, LV_INFO, "->cPeMgt::Dtor ");
}

/*-----------------------------------------------------------------------------
 * Name  : cPeMgt::handleTimerCallback
 * Descr : Is called by PE TimerThread to check for timeouts and to poll for
 *         PE service responses.
 *   Note: 'DataRecord Read Response' dosn't recall the read request immediatelly.
 *         A new req. is called from here (from the timer thread).
 */
void cPeMgt::handleTimerCallback()
{
    m_TickCount++;
    TRC_OUT01(GR_PE, LV_INFO, "cPeMgt::handleTimerCallback: TICK=%04d",m_TickCount);
    m_PeRqQ.timeout_chk();
} /* end of cPeMgt::handleTimerCallback */


/*-----------------------------------------------------------------------------
 * Name  : cPeMgt::init
 * Descr :
 */
void cPeMgt::init()
{
    TRC_OUT(GR_PE, LV_INFO, "->cPeMgt::init ");
    m_pTimer = new cPeTimer(this, 1000);
} /* end of cPeMgt::init */


/*-----------------------------------------------------------------------------
 * Name  : cPeMgt::uninit
 * Descr :
 */
void cPeMgt::uninit()
{
    TRC_OUT(GR_PE, LV_INFO, "->cPeMgt::Un-init ");
    delete m_pTimer;  // TBD! don't call dtor -> call kill. BUT DON'T forget to close Thread handle !!!
} /* end of cPeMgt::uninit */


/*-----------------------------------------------------------------------------
 * Name  : cPeMgt::handle_pe_cmd_request
 * Descr : PROFIenergy Service Request. Called by the user appl. PNIO_pe_cmd_req()
 *         ->IController::perf_pe()
 * Param :
 *  [ IN]:
 *  [OUT]:
 * Return: ==0: ok, != 0 PNIO_ERR_xxx   See pnioerr.h
 *         PNIO_ERR_NO_RESOURCE
 *         PNIO_ERR_PRM_INVALIDARG
 */
PNIO_UINT32 cPeMgt::handle_pe_cmd_request(PNIO_UINT32 Handle, PNIO_ADDR *pAddr, PNIO_REF user_ref, PNIO_PE_REQ_PRM *pPeReqPrm)
{
    PNIO_UINT32 result = PNIO_OK;
    cPeReq *pPeRq = NULL;
    PNIO_REF newReqRef;

    TRC_OUT02(GR_PE, LV_INFO, "->cPeMgt::handle_pe_cmd_request adr=%u ref=%#x", pAddr->u.Addr, user_ref);

    PNIO_UINT8 serv_ref = m_PeServiceRef.get_ref();
    if (!serv_ref) {
        result = PNIO_ERR_NO_RESOURCE;
        TRC_OUT01(GR_PE, LV_INFO, "<-cPeMgt::handle_pe_cmd_request no ref ret=%d =ERR_NO_RESOURCE",result);
        return result;
    }

    // create data record write request buffer
    int used_buff_len = 0;                 // total used buffer len
    PNIO_UINT8  buff[PNIOI_DREC_MAX_SIZE]; // 4096
    PE_PDU_DR_HDR *pHdr = (PE_PDU_DR_HDR*)&buff[0]; // pointer to DR + PE pdu header
    memset(&buff[0], 0, sizeof(buff));     // clear buffer content

    used_buff_len = sizeof(PE_PDU_DR_HDR); // fix size: DR + PE Header = 10 byte

    // check if structs are byte packed and aligned
    DPR_ASSERT(sizeof(PE_PDU_DR_HDR) == 10);
    DPR_ASSERT(sizeof(PE_PDU_START_PAUSE_REQ) == 10 + 4);  // pause_time_ms: 4 byte

    // fill pe header fix parts
    pHdr->pe_hdr.rq_ref  = serv_ref;

    // fill service specific params
    switch ( pPeReqPrm->CmdId ) {
        case PNIO_PE_CMD_START_PAUSE:
            {
                TRC_OUT02(GR_PE, LV_INFO, " handle_pe_cmd_request: START PAUSE cmdId=%d modif=%d",
                          pPeReqPrm->CmdId, pPeReqPrm->CmdModifier);
                pHdr->pe_hdr.service_id = pPeReqPrm->CmdId;
                pHdr->pe_hdr.modif_state= pPeReqPrm->CmdModifier;
                pHdr->pe_hdr.struct_id  = PE_CMD_STRUCT_ID_START_PAUSE_RQ;

                PE_PDU_START_PAUSE_REQ *pPdu = (PE_PDU_START_PAUSE_REQ*)&buff[0];
                pPdu->pause_time_ms = CPU_TO_BE(pPeReqPrm->rq.StartPause.time_ms);
                used_buff_len += sizeof(pPdu->pause_time_ms);
                break;
            }
        case PNIO_PE_CMD_END_PAUSE:
            {
                TRC_OUT02(GR_PE, LV_INFO, " handle_pe_cmd_request: END PAUSE cmdId=%d modif=%d",
                          pPeReqPrm->CmdId, pPeReqPrm->CmdModifier);
                pHdr->pe_hdr.service_id = pPeReqPrm->CmdId;
                pHdr->pe_hdr.modif_state= pPeReqPrm->CmdModifier;
                pHdr->pe_hdr.struct_id  = PE_CMD_STRUCT_ID_NOT_AVAILABLE; /* no params */
                break;
            }
        case PNIO_PE_CMD_Q_MODES:
            {
                TRC_OUT02(GR_PE, LV_INFO, " handle_pe_cmd_request: QUERY MODES cmdId=%d modif=%d",
                          pPeReqPrm->CmdId, pPeReqPrm->CmdModifier);
                pHdr->pe_hdr.service_id = pPeReqPrm->CmdId;
                pHdr->pe_hdr.modif_state= pPeReqPrm->CmdModifier;
                if (pPeReqPrm->CmdModifier == PE_CMD_MODIF_Q_MODE_LIST_ALL) {
                    pHdr->pe_hdr.struct_id  = PE_CMD_STRUCT_ID_NOT_AVAILABLE; /* no params */
                }
                else if (pPeReqPrm->CmdModifier == PE_CMD_MODIF_Q_MODE_GET_MODE) {
                    pHdr->pe_hdr.struct_id  = PE_CMD_STRUCT_ID_Q_MODE_GET_MODE_RQ;
                    PE_PDU_Q_MODE_GET_MODE_REQ *pPdu = (PE_PDU_Q_MODE_GET_MODE_REQ*)&buff[0];
                    /* add parameters */
                    pPdu->pe_mode_id = pPeReqPrm->rq.ModeGet.peModeId;
                    used_buff_len += sizeof(pPdu->pe_mode_id);
                    pPdu->reserved   = 0;
                    used_buff_len += sizeof(pPdu->reserved);
                }
                else {
                    result = PNIO_ERR_PRM_INVALIDARG;
                    goto cmd_req_end;
                }
                break;
            }
        case PNIO_PE_CMD_PEM_STATUS:
            {
                TRC_OUT02(GR_PE, LV_INFO, " handle_pe_cmd_request: PEM STATUS cmdId=%d modif=%d",
                          pPeReqPrm->CmdId, pPeReqPrm->CmdModifier);
                pHdr->pe_hdr.service_id = pPeReqPrm->CmdId;
                pHdr->pe_hdr.modif_state= pPeReqPrm->CmdModifier;
                pHdr->pe_hdr.struct_id  = PE_CMD_STRUCT_ID_NOT_AVAILABLE; /* no params */
                break;
            }
        case PNIO_PE_CMD_PE_IDENTIFY:
            {
                TRC_OUT02(GR_PE, LV_INFO, " handle_pe_cmd_request: PE IDENTIFY cmdId=%d modif=%d",
                          pPeReqPrm->CmdId, pPeReqPrm->CmdModifier);
                pHdr->pe_hdr.service_id = pPeReqPrm->CmdId;
                pHdr->pe_hdr.modif_state= pPeReqPrm->CmdModifier;
                pHdr->pe_hdr.struct_id  = PE_CMD_STRUCT_ID_NOT_AVAILABLE; /* no params */
                break;
            }
        case PNIO_PE_CMD_Q_MSRMT:
            {
                TRC_OUT02(GR_PE, LV_INFO, " handle_pe_cmd_request: QUERY MEASUREMENT cmdId=%d modif=%d",
                          pPeReqPrm->CmdId, pPeReqPrm->CmdModifier);
                pHdr->pe_hdr.service_id = pPeReqPrm->CmdId;
                pHdr->pe_hdr.modif_state= pPeReqPrm->CmdModifier;
                if (pPeReqPrm->CmdModifier == PE_CMD_MODIF_Q_MSRMT_LIST_ALL) {
                    pHdr->pe_hdr.struct_id  = PE_CMD_STRUCT_ID_NOT_AVAILABLE; /* no params */
                }
                else if (pPeReqPrm->CmdModifier == PE_CMD_MODIF_Q_MSRMT_GET_VAL) {
                    pHdr->pe_hdr.struct_id  = PE_CMD_STRUCT_ID_Q_MSRMT_GET_VAL_RQ;
                    PE_PDU_Q_MSRMT_GET_VAL_REQ *pPdu = (PE_PDU_Q_MSRMT_GET_VAL_REQ*)&buff[0];
                    pPdu->prm.num_measurement_ids = pPeReqPrm->rq.MeasurementValue.numMeasurementIds;
                    for (int i=0; i < pPeReqPrm->rq.MeasurementValue.numMeasurementIds; i++) {
                        pPdu->prm.measurement_id[i] = pPeReqPrm->rq.MeasurementValue.Measurement_ID[i];
                        used_buff_len += sizeof(pPdu->prm.measurement_id);
                    }
                }
                else {
                    result = PNIO_ERR_PRM_INVALIDARG;
                    goto cmd_req_end;
                }
                break;
            }
        default:
            TRC_OUT02(GR_PE, LV_ERR, " handle_pe_cmd_request: ERR cmdId=%d modif=%d", pPeReqPrm->CmdId, pPeReqPrm->CmdModifier);
            result = PNIO_ERR_PRM_INVALIDARG;
            goto cmd_req_end;
    } /* end switch CmdId */

    // fill DataRecord Header
    // block_type: 0x800 : Reserved for profiles covering energy saving BlockType for request service
    // See FDIS 61158-6-10_v23_FINAL__TC2WG11_TO_PNO_CO.pdf   = PNIO Norm Page 459
    pHdr->dr_hdr.block_type   = CPU_TO_BE16(0x800);
    pHdr->dr_hdr.version_high = 0x01;
    pHdr->dr_hdr.version_low  = 0x00;

    if ( used_buff_len < (int)sizeof(PE_PDU_DR_HDR) ) {
        used_buff_len = sizeof(PE_PDU_DR_HDR);
    }
    pHdr->dr_hdr.block_length = CPU_TO_BE16(used_buff_len - (sizeof(pHdr->dr_hdr.block_type) + sizeof(pHdr->dr_hdr.block_length)));

    // Creatre a new service request obj, add it into the rq container.
    // Do it before perf_ds() i.e. 'DR Write Req' is called.
    // Service response is requested (asked) by the 'read data record request' it is generated
    // by the 'write data record' callback
    // Note: dr write rq and read rq results are handled asynchronously in the callback 'dr-callback'
    //       IController::ProcDataRecordResponse() -> cPeMgt::handle_pe_dr_resp()
    //
    pPeRq = new cPeReq(Handle, pAddr, user_ref, pPeReqPrm->CmdId, pPeReqPrm->CmdModifier, serv_ref);
    if (pPeRq) {
        pPeRq->set_state(cPeReq::st_wr_pend);
        m_PeRqQ.add_rq(pPeRq);
    }

    // use standard data record read/write function to send this service request
    // But we replace the 'User ReqRef' by 'pe service ref'. Service-ref is the key in the
    // command management
    //
    newReqRef = serv_ref;
    result = m_pCtrl->perf_ds(pAddr, ACC_T_WRITE, newReqRef, PE_SAP_DR_INDEX,/*0x80A0*/ used_buff_len, &buff[0]);
    if (result != PNIO_OK) {
        TRC_OUT01(GR_PE, LV_ERR, " handle_pe_cmd_request: perf_ds: PNIO-ERR=%#x", result);

        cPeReq *prq = m_PeRqQ.remove_rq(serv_ref);
        if (prq) {
            delete prq;
        }
        else {
            TRC_OUT01(GR_PE, LV_INFO, " handle_pe_cmd_request: remove_rq: RQ already removed rq-ref=%d",serv_ref);
        }
        goto cmd_req_end;
    }
    // write service request ok, wait for the service response (signaled as callback)

cmd_req_end:
    TRC_OUT01(GR_PE, LV_INFO, "<-cPeMgt::handle_pe_cmd_request ret=%d",result);
    return result;
} /* end of cPeMgt::handle_pe_cmd_request */

/*-----------------------------------------------------------------------------
 * Name  : cPeMgt::handle_pe_dr_resp
 * Descr : Handles PROFIenergy (PE) Data Record Response. (Read+Write Responses)
 *         Is called by ProcDataRecordResponse() function, it is a standard
 *         Data Record handling function. PROFIenergy Data Records
 *         have specific PROFIenergy Service Access Point 'Data Record Index'
 */
void cPeMgt::handle_pe_dr_resp(const t_read_write_dr_resp *pDrRq)
{
    PNIO_UINT32  RecordIndex;
    PNAGC_DR_RQ_TYPE  dr_rw_action_type;

    TRC_OUT(GR_PE, LV_INFO, "->cPeMgt::handle_pe_dr_resp");

    RecordIndex = LE_TO_CPU(pDrRq->rqb.RecordIndex);
    if ( RecordIndex != PE_SAP_DR_INDEX ) {
        TRC_OUT01(GR_PE, LV_ERR, "->cPeMgt::handle_pe_dr_resp invalid index", RecordIndex);
        return;
    }

    /* check if PE callback was registered */
    //DPR_ASSERT(m_pUserCbf_PE);

    /* here only PROFIenergy (PE) related data record */

    dr_rw_action_type = (PNAGC_DR_RQ_TYPE) LE_TO_CPU(pDrRq->rqb.ActionType);
    if (dr_rw_action_type == PNAGC_READ_DR) {
        handle_pe_dr_read_resp(pDrRq);
    }
    else if (dr_rw_action_type == PNAGC_WRITE_DR) {
        handle_pe_dr_write_resp(pDrRq);
    }
    else {
        /* prog error */
        TRC_OUT01(GR_PE, LV_ERR, "->cPeMgt::handle_pe_dr_resp invalid DR ActionType",dr_rw_action_type);
        DPR_ASSERT(0);
    }

    TRC_OUT(GR_PE, LV_INFO, "<-cPeMgt::handle_pe_dr_resp");
    return;
} /* end of cPeMgt::handle_pe_dr_resp */

/*-----------------------------------------------------------------------------
 * Name  : cPeMgt::handle_pe_dr_read_resp
 * Descr : check data record error code: If error: repeate 'dr read request'
 *         If ok: pass PE service confirmation to the user.
 */
void cPeMgt::handle_pe_dr_read_resp(const t_read_write_dr_resp *pDrRq)
{

    TRC_OUT(GR_PE, LV_INFO, "->cPeMgt::handle_pe_dr_read_resp");

    PNIO_ADDR pnio_addr;
    pnio_addr.AddrType = (PNIO_ADDR_TYPE)LE_TO_CPU(pDrRq->rqb.Addr.AddrType);
    pnio_addr.IODataType = (PNIO_IO_TYPE)LE_TO_CPU(pDrRq->rqb.Addr.IODataType);
    pnio_addr.u.Addr = LE_TO_CPU(pDrRq->rqb.Addr.u.Addr);

    PNIO_UINT32  len   = pDrRq->rqb.Length;
    PNIO_UINT8  *pBuff = const_cast<PNIO_UINT8*> (&pDrRq->rqb.Buffer[0]);

    PNIO_REF  ReqRef  = LE_TO_CPU(pDrRq->rqb.UserRef);

    PNIO_ERR_STAT  e;
    e.ErrCode   = pDrRq->err.ErrCode;
    e.ErrDecode = pDrRq->err.ErrDecode;
    e.ErrCode1  = pDrRq->err.ErrCode1;
    e.ErrCode2  = pDrRq->err.ErrCode2;
    e.AddValue1 = LE_TO_CPU16(pDrRq->err.AddValue1);
    e.AddValue2 = LE_TO_CPU16(pDrRq->err.AddValue2);
    TRC_OUT05(GR_PE, LV_WARN, " handle_pe_dr_read_resp: ERR %#x %#x %#x %#x %#x",e.ErrCode,e.ErrDecode,
              e.ErrCode1, e.ErrCode2, e.AddValue1);

    // check if DR Read resp ok
    if ( e.ErrCode == 0 ) {
        // ok,
        // pass positive response to user (i.e. prepare parametr and call user callback)
        handle_pos_read_resp(&pnio_addr, ReqRef, e, len, pBuff);
    }
    else if (e.ErrCode == 0xDE && e.ErrDecode == 0x80 && e.ErrCode1 == 0xC3) {
        // device is busy but ok -> it's not a real error
        // repeating the read request is made by the timer thread peTimerThEntry()
        // - nothing todo here increment 'bussy counter' only
        increment_req_err_busy_count(ReqRef, len, pBuff);
    }
    else {
        //  negative read response - real errors
        pass_resp_to_user(&pnio_addr, ReqRef, PNAGC_READ_DR, e, len, pBuff);
    }
    TRC_OUT(GR_PE, LV_INFO, "<-cPeMgt::handle_pe_dr_read_resp");
    return;
} /* end of cPeMgt::handle_pe_dr_read_resp */


/*-----------------------------------------------------------------------------
 * Name  : cPeMgt::handle_pe_dr_write_resp
 * Descr : This function is called  by IController::ProcDataRecordResponse
 *         DR Write Response Callback has no valid data. => rqb.Length and
 *         rqb.Buffer[] content are invalid.
 */
void cPeMgt::handle_pe_dr_write_resp(const t_read_write_dr_resp *pDrRq)
{
    PNIO_UINT32  len   = 0;
    PNIO_UINT8  *pBuff = NULL;
    PNIO_ADDR pnio_addr;
    pnio_addr.AddrType = (PNIO_ADDR_TYPE)LE_TO_CPU(pDrRq->rqb.Addr.AddrType);
    pnio_addr.IODataType = (PNIO_IO_TYPE)LE_TO_CPU(pDrRq->rqb.Addr.IODataType);
    pnio_addr.u.Addr = LE_TO_CPU(pDrRq->rqb.Addr.u.Addr);


    // UserRef in the DR response is 'service-ref' of PE Command request.
    // It was replaced in handle_pe_cmd_request(). Original UserRef was saved
    // in cPeReq object.
    // Note: 'service-ref' is a key in the map to find correspondig Req to a Resp.
    //
    PNIO_REF  user_rq_ref;
    user_rq_ref = LE_TO_CPU(pDrRq->rqb.UserRef);

    // check DataRecord write response result
    if (pDrRq->err.ErrCode != 0) {
        // error
        PNIO_ERR_STAT  e;
        e.ErrCode   = pDrRq->err.ErrCode;
        e.ErrDecode = pDrRq->err.ErrDecode;
        e.ErrCode1  = pDrRq->err.ErrCode1;
        e.ErrCode2  = pDrRq->err.ErrCode2;
        e.AddValue1 = LE_TO_CPU16(pDrRq->err.AddValue1);
        e.AddValue2 = LE_TO_CPU16(pDrRq->err.AddValue2);
        TRC_OUT05(GR_PE, LV_WARN, " handle_pe_dr_read_resp: ERR %#x %#x %#x %#x %#x",
                  e.ErrCode,e.ErrDecode, e.ErrCode1, e.ErrCode2, e.AddValue1);
        // in case of errors - len and buff are invalid
        len  = 0;
        pBuff = NULL;
        //pass_resp_to_user(&pnio_addr, user_rq_ref, PNAGC_WRITE_DR, e, len, pBuff);
        handle_neg_write_resp(&pnio_addr, user_rq_ref, e);
    }
    else {
        // dr write ok:
        handle_pos_write_resp(&pnio_addr, user_rq_ref);
    }

    return;
} /* end of cPeMgt::handle_pe_dr_write_resp */

/*-----------------------------------------------------------------------------
 * Name  : handle_pos_write_resp
 * Descr : This function is called  by IController::ProcDataRecordResponse
 * Param :
 *  [ IN]: PNIO_ADDR  *pAddr: Addr
 *         PNIO_REF    ReqRef: 'service-ref' from DR Write request
 *         PNIO_REF    user_ref: 'service-ref' from DR Write request,
 *  [OUT]:
 */
void cPeMgt::handle_pos_write_resp(PNIO_ADDR *pAddr, PNIO_REF ReqRef)
{
    PNIO_UINT32 result = PNIO_OK;
    cPeReq *pPeRq = NULL;         // ptr to the 'pe command request obj'


    TRC_OUT01(GR_PE, LV_INFO, "->handle_pos_write_resp: service-ref=%#x", ReqRef);

    PNIO_UINT8  serv_ref = (PNIO_UINT8)ReqRef;

    // lock the whole RqOueue TBD !!!!!!!!!!!!!!!!!!!!!!!!!! (could be deleted somewhere else)

    // find corresponding cPeReq
    // retrieve params to build DR read Requst
    // set PeReq state for farther PeReq management (e.g.timeout check)
    pPeRq = m_PeRqQ.find_rq((RqKeyType)serv_ref);
    if (pPeRq) {
        // found
        // create DataRecord to read pe service response (to request for response)
        int used_buff_len = 0;                        // total used buffer len
        PNIO_UINT8  buff[sizeof(PE_PDU_DR_READ_RQ)];
        PE_PDU_DR_READ_RQ *pPdu = (PE_PDU_DR_READ_RQ*)&buff[0]; // pointer to pdu header
        memset(&buff[0], 0, sizeof(buff));            // clear buffer content

        used_buff_len = PNIOI_DREC_MAX_SIZE; //4096;
        pPdu->dr_hdr.block_type   = PE_DR_BLKTYPE_READ;
        pPdu->dr_hdr.block_length = sizeof(PE_PDU_DR_READ_RQ) - sizeof(pPdu->dr_hdr.block_type)
                                    - sizeof(pPdu->dr_hdr.block_length);
        pPdu->dr_hdr.version_high = PE_DR_BLKVERS_HIGT;
        pPdu->dr_hdr.version_low  = PE_DR_BLKVERS_LOW;

        pPdu->pe_hdr.service_id   = pPeRq->cmd_id;
        pPdu->pe_hdr.rq_ref       = pPeRq->serv_ref;
        pPdu->pe_hdr.modif_state  = pPeRq->cmd_modifier;
        pPdu->pe_hdr.struct_id    = 0x00;

        // check refs
        if (serv_ref != pPeRq->serv_ref ) {
            TRC_OUT02(GR_PE, LV_ERR, " _pos_write_resp: ERROR REF: ReqRef=%#x PeRq.ref=%#x",serv_ref,pPeRq->serv_ref);
            // we use the saved one from PeRq
            serv_ref = pPeRq->serv_ref;
        }

        // use standard data record read function to send DR Read Request
        result = m_pCtrl->perf_ds(pAddr, ACC_T_READ, (PNIO_REF)serv_ref, PE_SAP_DR_INDEX,/*0x80A0*/ used_buff_len, &buff[0]);
        if (result == PNIO_OK) {
            // set PeRq state
            pPeRq->set_state(cPeReq::st_rd_pend);
        }
        else {
            // error
            TRC_OUT01(GR_PE, LV_ERR, " _pos_write_resp: ERROR perf_ds(READ) ret=%#x",result);
            // negative resp to user
            PNIO_ERR_STAT   e = {0,0};
            e.ErrCode     = 0xDF;       // DR write err
            e.ErrDecode   = 0x80;       // PNIORW
            e.ErrCode1    = 0xA0;       // == appl. read err
            handle_neg_write_resp(pAddr, (PNIO_REF)serv_ref, e);
        }
    }
    else {
        // PeRq not found
        TRC_OUT01(GR_PE, LV_ERR, " _pos_write_resp: ERROR RQ NOT FOUND  ref=%d", serv_ref);
        // negative resp to user
        PNIO_ERR_STAT   e = {0,0};
        e.ErrCode     = 0xDF;       // DR write err
        e.ErrDecode   = 0x80;       // PNIORW
        e.ErrCode1    = 0xA0;       // == appl. read err
        handle_neg_write_resp(pAddr, (PNIO_REF)serv_ref, e);
    }

    // un-lock the whole RqOueue TBD !!!!!!!!!!!!!!!!!!!!!!!!!!


    TRC_OUT01(GR_PE, LV_INFO, "<-handle_pos_write_resp: service-ref=%#x", serv_ref);
    return;
} /* end of handle_pos_write_resp */


/*-----------------------------------------------------------------------------
 * Name  : handle_neg_write_resp
 * Descr : This function is called  by IController::ProcDataRecordResponse ->
 *         handle_pe_dr_write_resp()
 * Param :
 *  [ IN]: PNIO_ADDR  *pAddr : Addr
 *         PNIO_REF    ReqRef: 'service-ref' from DR Write request
 *         PNIO_ERR_STAT &e  : Error
 *  [OUT]:
 */
void cPeMgt::handle_neg_write_resp(PNIO_ADDR *pnio_addr, PNIO_REF ReqRef, PNIO_ERR_STAT &e)
{
    cPeReq           *pPeRq  = NULL;
    PNIO_PE_CBE_PRM  *p_cb      = NULL;                  // user callback param pointer
    PNIO_REF         currUserReqRef = 0;
    PNIO_PE_CMD_ENUM currCmdId     = PNIO_PE_CMD_RESERVED; // 0;
    PNIO_UINT32      logAddr = pnio_addr->u.Addr;

    TRC_OUT03(GR_PE, LV_INFO, "->cPeMgt::handle_neg_write_resp: LogAddr=%u RqRef=%#x ErrCode=%#x",logAddr, ReqRef, e.ErrCode);


    PNIO_UINT8  serv_ref = (PNIO_UINT8)ReqRef;  // input param

    // find and remove cPeReq obj from the req queue
    //
    pPeRq = m_PeRqQ.remove_rq((RqKeyType)serv_ref);
    if (pPeRq) {
        // PeReq found
        int time_to_live = pPeRq->lifetime_sec;
        TRC_OUT01(GR_PE, LV_INFO, " _resp_to_user: time=%d",time_to_live);

        // retrieve important params

        currUserReqRef = pPeRq->user_ref;   // originaly saved user-ref
        currCmdId      = pPeRq->cmd_id;

        // we don't need PeReq params more
        // -> free service-ref, return it back to the q
        // -> delete this rq
        if (pPeRq) {
            m_PeServiceRef.put_ref(serv_ref);
            delete pPeRq;
            pPeRq = NULL;
        }

    }
    else {
        // PeReq NOT found - should not happen, we cannot retrieve original UserReqRef
        // which is saved in PeReq obj.
        // be defensive: - set to invalid val
        TRC_OUT01(GR_PE, LV_ERR, " _resp_to_user: ERROR: NO REQ FOUND serv-ref=%#x", serv_ref);
        currUserReqRef = (PNIO_REF)(-1);
        currCmdId      = PNIO_PE_CMD_RESERVED; // 0;
    }

    // build user callback parameter struct

    int required_buff_len = sizeof(PE_CBE_HDR) + sizeof(PNIO_PE_PRM_GENERIC_CONF_NEG);

    p_cb = (PNIO_PE_CBE_PRM*)malloc(required_buff_len);  // <--- dynamic MEM ALLOC !!!
    if (!p_cb) {
        TRC_OUT01(GR_PE, LV_ERR, " _resp_to_user: ERROR: NO MEM  rqd-size=%d", required_buff_len);
        return;
    }
    memset(p_cb, 0, required_buff_len);

    p_cb->pe_hdr.CmdId        = currCmdId; // (PNIO_PE_CMD_ENUM)p_pdu_hdr->pe_hdr.service_id; // PNIO_UINT8 enum
    p_cb->pe_hdr.Addr         = *pnio_addr;
    p_cb->pe_hdr.Ref          = currUserReqRef;
    p_cb->pe_hdr.Err          = e;
    p_cb->pe_hdr.state        = 0x02; // ready-with-error
    p_cb->pe_hdr.struct_id    = 0xFF; // response struct_id
    p_cb->pe_hdr.length       = 0;    // no conf pdu present
    // PNIO_PE_PRM_GENERIC_CONF_NEG content is zero

    // check if PE callback is already registered - call it
    if (m_pUserCbf_PE) {
        TRC_OUT04(GR_PE, LV_INFO, " cPeMgt: --> user PE NegConf callback: logAddr=%u UsrRef=%#x ErrCode=%#x CmdId=%#x",
            logAddr, currUserReqRef, e.ErrCode, currCmdId);

        m_pUserCbf_PE( (PNIO_PE_CBE_PRM*) p_cb);

        TRC_OUT02(GR_PE, LV_INFO, " cPeMgt: <-- user PE NegConf callback done! Ref=%#x CmdId=%#x",currUserReqRef, currCmdId);
    }

    // free allocated callback param struct.
    if (p_cb) {
        free(p_cb);
    }

    TRC_OUT01(GR_PE, LV_INFO,"<-cPeMgt::handle_neg_write_resp: RqRef=%#x", ReqRef);
    return;
} /* end of handle_neg_write_resp */


/*-----------------------------------------------------------------------------
 * Name  : increment_req_err_busy_count
 * Descr :
 * Param :
 *  [ IN]:
 *  [OUT]:
 */
void cPeMgt::increment_req_err_busy_count(PNIO_REF ReqRef, PNIO_UINT32 len, PNIO_UINT8* pbuff)
{
    cPeReq *pPeRq = NULL;

    TRC_OUT03(GR_PE, LV_INFO,"<->cPeMgt::increment_req_err_busy_count: RqRef=%#x len=%u pbuff=%#x",ReqRef,len,pbuff);

    RqKeyType  serv_ref = (RqKeyType)ReqRef;

    // lock the whole RqOueue TBD !!!!!!!!!!!!!!!!!!!!!!!!!!

    pPeRq = m_PeRqQ.find_rq((RqKeyType)serv_ref);
    if (pPeRq) {
        // found
        pPeRq->err_count++;
        pPeRq->set_state(cPeReq::st_rd_busy);
    }
    else {
        // PeRq not found
        TRC_OUT01(GR_PE, LV_ERR, " increment_req_err_count: ERROR RQ NOT FOUND  ref=%d", serv_ref);
        // ignore
    }

    // un-lock the whole RqOueue TBD !!!!!!!!!!!!!!!!!!!!!!!!!!

    return;
} /* end of increment_req_err_count */


/*-----------------------------------------------------------------------------
 * Name  : cPeMgt::handle_pos_read_resp
 *
 * Descr : Handles positive DataRecord Read Response.
 *         Creates callback parameter struct and calls the 'Callback funct.'
 *         This function is called either from 'DataRecord Response Handling'
 *         i.e. IController::ProcDataRecordResponse() -> handle_pe_dr_resp()
 * Param :
 *  [ IN]: PNIO_ADDR *pnio_addr: Ptr to logical input addr
 *         PNIO_REF ReqRef     : ReqRef from command response RequestBlock
 *         PNIO_ERR_STAT &e    : Error
 *         PNIO_UINT32 len     : Buffer length
 *         PNIO_UINT8* pbuff   : Received DataRequest Pdu
 *
 */
void cPeMgt::handle_pos_read_resp(PNIO_ADDR *pnio_addr, PNIO_REF ReqRef, PNIO_ERR_STAT &e,
                                  PNIO_UINT32 len, PNIO_UINT8* pbuff
                                  )
{
    cPeReq *pPeRq = NULL;
    PE_PDU_DR_HDR   *p_pdu_hdr = (PE_PDU_DR_HDR*)pbuff; // pointer to the 'pe pdu' header
    PE_PDU_CNF      *p_pdu_cnf = (PE_PDU_CNF*)pbuff;    //
    PNIO_PE_CBE_PRM *p_cb      = NULL;                  // user callback param pointer
    //PNIO_UINT8      myErrCode  = 0xDE;   // ReadResp - in this function generated code
    //bool            pe_pdu_present = false;
    PNIO_REF        currUserReqRef = 0;
    PNIO_PE_CMD_ENUM currCmdId     = PNIO_PE_CMD_RESERVED; // 0;
    PNIO_UINT32      logAddr       = pnio_addr->u.Addr;


    TRC_OUT04(GR_PE, LV_INFO, "->cPeMgt::handle_pos_read_resp: logAddr=%u RqRef=%#x len=%d pbuff=%#x",
              logAddr, ReqRef, len, pbuff);

    // check input params: if ErrCode == 0 pbuff must be present i.e. != NULL
    if ( len == 0 || pbuff == NULL ) {
        TRC_OUT01(GR_PE, LV_ERR, " _resp_to_user: ERROR: inp. NULL RqRef=%#x", ReqRef);
        return;
    }

    PNIO_UINT8  serv_ref = (PNIO_UINT8)ReqRef;  // input param

    // find and remove cPeReq obj from the req queue
    //
    pPeRq = m_PeRqQ.remove_rq((RqKeyType)serv_ref);
    if (pPeRq) {
        // PeReq found
        int time_to_live = pPeRq->lifetime_sec;
        TRC_OUT01(GR_PE, LV_INFO, " _resp_to_user: time=%d",time_to_live);

        // retrieve important params

        currUserReqRef = pPeRq->user_ref;
        currCmdId      = pPeRq->cmd_id;

        // dbg check
        PNIO_UINT8  PeRq_serv_ref = (PNIO_UINT8)pPeRq->serv_ref;  // input param
        if ( serv_ref != PeRq_serv_ref) {
            TRC_OUT02(GR_PE, LV_ERR, " _resp_to_user: ERROR: rb-ref=%#x PeRq_ref=%#x",serv_ref,PeRq_serv_ref);
        }

        // we don't need PeReq params more
        // -> free service-ref, return it back to the q
        // -> delete this rq
        if (pPeRq) {
            m_PeServiceRef.put_ref(serv_ref);
            delete pPeRq;
            pPeRq = NULL;
        }

    }
    else {
        // PeReq NOT found - should not happen, probably timed out
        // be defensive: - no action
        TRC_OUT01(GR_PE, LV_ERR, " _resp_to_user: ERROR: NO REQ FOUND serv-ref=%#x", serv_ref);
        return;
    }

    if ( len < sizeof(PE_PDU_DR_HDR) ) {
        // pe service resp header is present, so we can access params.
        TRC_OUT02(GR_PE, LV_ERR, " _resp_to_user: ERROR: NO PDU len=%d rqd-len=%d",len,sizeof(PE_PDU_DR_HDR));
        return;
    }

    // positive read resp: build positive user callback parameter struct, call user callback
    //--------------------

    PNIO_UINT16 pdu_blk_len  = p_pdu_hdr->dr_hdr.block_length;

    int blk_len  = (int) BE_TO_CPU16(pdu_blk_len);
    int param_len= blk_len - (sizeof(p_pdu_hdr->dr_hdr.version_high) +
                              sizeof(p_pdu_hdr->dr_hdr.version_low)  +
                              sizeof(p_pdu_hdr->pe_hdr));

    if (param_len < 0) {
        param_len = 0;
    }

    int required_buff_len = sizeof(PNIO_PE_CBE_PRM);     // ca. 1K mem

    p_cb = (PNIO_PE_CBE_PRM*)malloc(required_buff_len);  // <--- dynamic MEM ALLOC !!!
    if (!p_cb) {
        TRC_OUT01(GR_PE, LV_ERR, " _resp_to_user: ERROR: NO MEM  rqd-size=%d", required_buff_len);
        return;
    }
    memset(p_cb, 0, required_buff_len);

    // build user callback parameter struct
    p_cb->pe_hdr.CmdId        = currCmdId; // (PNIO_PE_CMD_ENUM)p_pdu_hdr->pe_hdr.service_id;
    p_cb->pe_hdr.Addr         = *pnio_addr;
    p_cb->pe_hdr.Ref          = currUserReqRef;
    p_cb->pe_hdr.Err          = e;
    p_cb->pe_hdr.state        = p_pdu_hdr->pe_hdr.modif_state;
    p_cb->pe_hdr.struct_id    = p_pdu_hdr->pe_hdr.struct_id;   /* service modifier */

    // fill callback parameters
    switch ( p_pdu_hdr->pe_hdr.service_id ) {
        case PNIO_PE_CMD_START_PAUSE:
            {
                PNIO_PE_PRM_START_PAUSE_CONF* p_dst = &p_cb->pe_prm.StartPause;
                PE_PDU_START_PAUSE_CONF_POS*      p_src = &p_pdu_cnf->pe_cnf.cdt.start_pause_conf;
                p_dst->pe_mode_id = p_src->pe_mode_id;
                break;
            }
        case PNIO_PE_CMD_END_PAUSE:
            {
                //PNIO_PE_PRM_END_PAUSE_CONF* p_dst = &p_cb->pe_prm.EndPause;
                p_cb->pe_prm.EndPause.time_ms = BE_TO_CPU(p_pdu_cnf->pe_cnf.cdt.end_pause_conf.time_to_operate);
                break;
            }
        case PNIO_PE_CMD_Q_MODES:
            if ( p_cb->pe_hdr.struct_id == PE_CMD_STRUCT_ID_Q_MODE_LIST_ALL_RS ) {
                // list all
                PNIO_PE_PRM_Q_MODE_LIST_ALL_CONF* p_dst = &p_cb->pe_prm.ModeList;
                PE_PDU_Q_MODE_LIST_ALL_CONF_POS*  p_src = &p_pdu_cnf->pe_cnf.cdt.q_mode_list_all_conf;
                p_dst->numModeIds = p_src->num_mode_ids;

                for (int i=0; i < p_src->num_mode_ids; i++) {
                    p_dst->peModeId[i] = p_src->pe_mode_id[i];
                }

            }
            else if ( p_cb->pe_hdr.struct_id == PE_CMD_STRUCT_ID_Q_MODE_GET_MODE_RS) {
                // get mode
                // TBD!!!
            }
            break;
        case PNIO_PE_CMD_PEM_STATUS:
            {
                PNIO_PE_PRM_PEM_STATUS_CONF* p_dst = &p_cb->pe_prm.PemStatus;   // callback response params
                PE_PDU_PEM_STATUS_CONF_POS*  p_src = &p_pdu_cnf->pe_cnf.cdt.pem_status_conf;  // received resp. pdu
                p_dst->PE_Mode_ID_Source       = p_src->pe_mode_id_source;
                p_dst->PE_Mode_ID_Destination  = p_src->pe_mode_id_destination;
                p_dst->Time_to_operate         = BE_TO_CPU(p_src->time_to_operate);
                p_dst->Remaining_time_to_destination     = BE_TO_CPU(p_src->remaining_time_to_destination);
                p_dst->Mode_Power_Consumption            = (float)BE_TO_CPU(p_src->mode_power_consumption);
                p_dst->Energy_Consumption_to_Destination = (float)BE_TO_CPU(p_src->energy_consumption_to_destination);
                p_dst->Energy_Consumption_to_operate     = (float)BE_TO_CPU(p_src->energy_consumption_to_operate);
                break;
            }
        case PNIO_PE_CMD_PE_IDENTIFY:
            {
                PNIO_PE_PRM_PE_IDENTIFY_CONF* p_dst = &p_cb->pe_prm.PeIdentify;
                PE_PDU_PE_IDENTIFY_CONF_POS*  p_src = &p_pdu_cnf->pe_cnf.cdt.pe_identify_conf;

                if ( p_cb->pe_hdr.struct_id == PE_CMD_STRUCT_ID_PE_IDENTIFY_RS ) {
                    p_dst->numServices = p_src->num_services;
                    memset(&p_dst->serviceId[0], 0, sizeof(p_dst->serviceId));
                    for (int i=0; i < p_src->num_services; i++) {
                        p_dst->serviceId[i] = p_src->service_id[i];
                    }
                }
                else {
                    // err
                    p_dst->numServices = 0;
                }
            }
            break;
        case PNIO_PE_CMD_Q_MSRMT:
            // TBD!!!
        default:
            TRC_OUT01(GR_PE, LV_ERR, " _resp_to_user: ERROR: UNKOWN CMD-ID=%#x",currCmdId);

    } /* end switch service_id */


    // check if PE callback is already registered - call it
    if (m_pUserCbf_PE) {
        TRC_OUT03(GR_PE, LV_INFO, " cPeMgt: --> user PE Conf+ Cb: logAddr=%u UsrRef=%#x CmdId=%#x",
                                  logAddr,currUserReqRef,currCmdId);

        m_pUserCbf_PE( (PNIO_PE_CBE_PRM*) p_cb);

        TRC_OUT02(GR_PE, LV_INFO, " cPeMgt: <-- user PE Conf+ Cb: logAddr=%u UsrRef=%#x ",logAddr,currUserReqRef);
    }

    // free allocated callback param struct.
    if (p_cb) {
        free(p_cb);
    }

    TRC_OUT02(GR_PE, LV_INFO, "<-cPeMgt::handle_pos_read_resp: logAddr=%u UsrRef=%#x ",logAddr,currUserReqRef);
    return;
} /* end of cPeMgt::handle_pos_read_resp */


/*-----------------------------------------------------------------------------
 * Name  : handle_neg_read_resp
 * Descr : This function is called  by IController::ProcDataRecordResponse ->
 *         handle_pe_dr_read_resp()
 * Param :
 *  [ IN]: PNIO_ADDR  *pAddr : Addr
 *         PNIO_REF    ReqRef: 'service-ref' from DR Write request
 *         PNIO_ERR_STAT &e  : Error
 *  [OUT]:
 */
void cPeMgt::handle_neg_read_resp(PNIO_ADDR *pnio_addr, PNIO_REF ReqRef, PNIO_ERR_STAT &e,
                                  PNIO_UINT32 len,
                                  PNIO_UINT8* pbuff
                                  )
{
    cPeReq           *pPeRq  = NULL;
    PNIO_PE_CBE_PRM  *p_cb   = NULL;                  // user callback param pointer
    PNIO_REF         currUserReqRef = 0;
    PNIO_PE_CMD_ENUM currCmdId      = PNIO_PE_CMD_RESERVED; // 0;
    PNIO_UINT32      logAddr = pnio_addr->u.Addr;

    TRC_OUT03(GR_PE, LV_INFO, "->cPeMgt::handle_neg_read_resp: LogAddr=%u RqRef=%#x ErrCode=%#x",logAddr, ReqRef, e.ErrCode);

    PNIO_UINT8  serv_ref = (PNIO_UINT8)ReqRef;  // input param

    // find and remove cPeReq obj from the req queue
    //
    pPeRq = m_PeRqQ.remove_rq((RqKeyType)serv_ref);
    if (pPeRq) {
        // PeReq found
        int time_to_live = pPeRq->lifetime_sec;
        TRC_OUT01(GR_PE, LV_INFO, " _resp_to_user: time=%d",time_to_live);

        // retrieve important params

        currUserReqRef = pPeRq->user_ref;   // originaly saved user-ref
        currCmdId      = pPeRq->cmd_id;

        // we don't need PeReq params more
        // -> free service-ref, return it back to the q
        // -> delete this rq
        if (pPeRq) {
            m_PeServiceRef.put_ref(serv_ref);
            delete pPeRq;
            pPeRq = NULL;
        }

    }
    else {
        // PeReq NOT found - should not happen, we cannot retrieve original UserReqRef
        // which is saved in PeReq obj.
        // be defensive: - no action
        TRC_OUT01(GR_PE, LV_ERR, " _resp_to_user: ERROR: NO REQ FOUND serv-ref=%#x", serv_ref);
        return;
    }

    // build user callback parameter struct
    // check if pdu is present
    bool pe_pdu_present = false;
    PE_PDU_CNF    *p_pdu_cnf = (PE_PDU_CNF*)pbuff;    //
    PE_PDU_DR_HDR *p_pdu_hdr = (PE_PDU_DR_HDR*)pbuff; // pointer to the 'pe pdu' header
    PNIO_UINT32    min_len = sizeof(DR_BLOCK_HDR) + sizeof(PE_PDU_HDR) + sizeof(PE_PDU_DT_CONF_NEG);

    PNIO_UINT8 currState    = 0x02;
    PNIO_UINT8 currStructId = 0xff;

    if ( pbuff && (len >= min_len) ) {
        pe_pdu_present = true;
        currCmdId = (PNIO_PE_CMD_ENUM)p_pdu_hdr->pe_hdr.service_id; // PNIO_UINT8 enum
        currState = p_pdu_hdr->pe_hdr.modif_state;
        currStructId = p_pdu_hdr->pe_hdr.struct_id;
    }
    else {
        pe_pdu_present = false;
        TRC_OUT03(GR_PE, LV_ERR, " _resp_to_user: ERROR: NO-PDU RqRef=%#x len= rqd-len=%d", ReqRef, len, min_len );
    }

    // calculate required response buffer size
    int required_buff_len = sizeof(PE_CBE_HDR) + sizeof(PNIO_PE_PRM_GENERIC_CONF_NEG);

    p_cb = (PNIO_PE_CBE_PRM*)malloc(required_buff_len);  // <--- dynamic MEM ALLOC !!!
    if (!p_cb) {
        TRC_OUT01(GR_PE, LV_ERR, " _resp_to_user: ERROR: NO MEM  rqd-size=%d", required_buff_len);
        return;
    }
    memset(p_cb, 0, required_buff_len);

    p_cb->pe_hdr.CmdId        = currCmdId; // (PNIO_PE_CMD_ENUM)p_pdu_hdr->pe_hdr.service_id; // PNIO_UINT8 enum
    p_cb->pe_hdr.Addr         = *pnio_addr;
    p_cb->pe_hdr.Ref          = currUserReqRef;
    p_cb->pe_hdr.Err          = e;
    p_cb->pe_hdr.state        = currState;
    p_cb->pe_hdr.struct_id    = currStructId;
    p_cb->pe_hdr.length       = sizeof(PNIO_PE_PRM_GENERIC_CONF_NEG); // no conf pdu present

    if ( pe_pdu_present ) {
        p_cb->pe_prm.RespNegative.err_code = p_pdu_cnf->pe_cnf.cdt.negative_conf.err_code;

    }
    else {
        p_cb->pe_prm.RespNegative.err_code = 0xff;
    }

    // check if PE callback is already registered - call it
    if (m_pUserCbf_PE) {
        TRC_OUT04(GR_PE, LV_INFO, " cPeMgt: --> user PE Read NegConf Cb: logAddr=%u UsrRef=%#x ErrCode=%#x CmdId=%#x",
            logAddr, currUserReqRef, e.ErrCode, currCmdId);

        m_pUserCbf_PE( (PNIO_PE_CBE_PRM*) p_cb);

        TRC_OUT02(GR_PE, LV_INFO, " cPeMgt: <-- user PE Read NegConf Cb done! Ref=%#x CmdId=%#x",currUserReqRef, currCmdId);
    }

    // free allocated callback param struct.
    if (p_cb) {
        free(p_cb);
    }

    TRC_OUT01(GR_PE, LV_INFO,"<-cPeMgt::handle_neg_read_resp: RqRef=%#x", ReqRef);
    return;
} /* end of handle_neg_read_resp */


/*-----------------------------------------------------------------------------
 * Name  : pass_resp_to_user
 * Descr : Handles negative and positive DataRecord Read/Write Response.
 *         Negative resp: e.ErrCode != 0x00: PNIO_PE_PRM_GENERIC_CONF_NEG
 *         Positive resp: Callback parameter struct depending on 'service-id'
 *         Creates callback parameter struct and calls the 'Callback funct.'
 *         This function is called either from 'DataRecord Response Handling' or
 *         from the 'Request Timeout Checker' (Timer thread).
 * Param : Parameters are either from 'DataRecord Read/Write Response' or are
 *         synthesized from cPeReq object from Command Queue Manager cPeMgt
 *  [ IN]: PNIO_ADDR *pnio_addr: Ptr to logical input addr
 *         PNIO_REF ReqRef     : ReqRef from command response RequestBlock
 *         PNAGC_DR_RQ_TYPE dr_rw_action_type: DR READ / WRITE
 *         PNIO_ERR_STAT &e    : Error
 *         PNIO_UINT32 len     : Buffer length
 *         PNIO_UINT8* pbuff   : Received DataRequest Pdu
 *
 */
void cPeMgt::pass_resp_to_user(PNIO_ADDR *pnio_addr,
                               PNIO_REF ReqRef,
                               PNAGC_DR_RQ_TYPE dr_rw_action_type,
                               PNIO_ERR_STAT &e,
                               PNIO_UINT32 len,
                               PNIO_UINT8* pbuff
                               )
{

    cPeReq *pPeRq = NULL;
    PE_PDU_DR_HDR   *p_pdu_hdr = (PE_PDU_DR_HDR*)pbuff; // pointer to the 'pe pdu' header
    PE_PDU_CNF      *p_pdu_cnf = (PE_PDU_CNF*)pbuff;    //                                                     //
    PNIO_PE_CBE_PRM *p_cb      = NULL;                  // user callback param pointer
    PNIO_UINT8      myErrCode;                          // in this function generated code
    bool            pe_pdu_present = false;
    PNIO_REF        currUserReqRef = 0;
    PNIO_PE_CMD_ENUM currCmdId     = PNIO_PE_CMD_RESERVED; // 0;
    PNIO_UINT32      currHandle    = (PNIO_UINT32) -1;

    TRC_OUT03(GR_PE, LV_INFO, "cPeMgt::pass_resp_to_user: WR=%d UserRef=%#x ErrCode=%#x",
              dr_rw_action_type, ReqRef, e.ErrCode);

    if (dr_rw_action_type == (PNAGC_DR_RQ_TYPE)ACC_T_READ) {
        myErrCode    = 0xDE;  // ReadResp
    }
    else {
        myErrCode    = 0xDF;  // WriteResp
    }

    // check input params: if ErrCode == 0 pbuff must be present i.e. != NULL
    if ( e.ErrCode == 0 && pbuff == NULL ) {
        TRC_OUT01(GR_PE, LV_ERR, " _resp_to_user: ERROR: pbuff NULL UserRef=%#x", ReqRef);
        len = 0;  // force error handling below
    }

    // check pe len for save access to the pdu header
    // Note: len == 0 occurres in case of "DataRecord Read/Write" negative responses.
    // I.e. raw DR respnse - no PE response.

    PNIO_UINT32  pdu_serv_ref  = (PNIO_UINT32)0xffffffff;
    PNIO_UINT32  rqb_serv_ref  = (PNIO_UINT32)ReqRef;

    if ( len >= sizeof(PE_PDU_DR_HDR) ) {
        // pe service resp header is present, so we can retrieve service-ref from the resp. pdu
        // find and remove cPeReq obj from the req queue
        pe_pdu_present = true;
        pdu_serv_ref = (RqKeyType)p_pdu_hdr->pe_hdr.rq_ref;
    }

    // debug aim only: check if the DR-Ifc returned 'rqb-service-ref' == 'pe pdu service-ref'
    if ( pdu_serv_ref != rqb_serv_ref ) {
        TRC_OUT02(GR_PE, LV_ERR, " _resp_to_user: ERROR REF:pdu=%#x NOT rqb=%#x",pdu_serv_ref, rqb_serv_ref);
        // no action, we use the rqb ref
    }

    PNIO_UINT8  serv_ref = (PNIO_UINT8)rqb_serv_ref;  // input param

    // find and remove cPeReq obj from the req queue
    //
    pPeRq = m_PeRqQ.remove_rq((RqKeyType)serv_ref);
    if (pPeRq) {
        // PeReq found
        int time_to_live = pPeRq->lifetime_sec;
        TRC_OUT01(GR_PE, LV_INFO, " _resp_to_user: time=%d",time_to_live);

        // retrieve important params

        currUserReqRef = pPeRq->user_ref;
        currCmdId      = pPeRq->cmd_id;
        currHandle     = pPeRq->user_handle;

        // we don't need PeReq params more
        // -> free service-ref, return it back to the q
        // -> delete this rq
        if (pPeRq) {
            m_PeServiceRef.put_ref(serv_ref);
            delete pPeRq;
            pPeRq = NULL;
        }

    }
    else {
        // PeReq NOT found - should not happen, we cannot retrieve original UserReqRef
        // which is saved in PeReq obj.
        // be defensive: - no action
        TRC_OUT01(GR_PE, LV_ERR, " _resp_to_user: ERROR: NO REQ FOUND serv-ref=%#x", serv_ref);
        return;
    }

    // IMPORTANT: pPeRq is no more valid here

    if (e.ErrCode != 0x00) {
        // negative response
        //-------------------
        // build user callback parameter struct

        int required_buff_len = sizeof(PE_CBE_HDR) + sizeof(PNIO_PE_PRM_GENERIC_CONF_NEG);

        p_cb = (PNIO_PE_CBE_PRM*)malloc(required_buff_len);  // <--- dynamic MEM ALLOC !!!
        if (!p_cb) {
            TRC_OUT01(GR_PE, LV_ERR, " _resp_to_user: ERROR: NO MEM  rqd-size=%d", required_buff_len);
            return;
        }
        memset(p_cb, 0, required_buff_len);

        p_cb->pe_hdr.CmdId        = currCmdId;  // (PNIO_PE_CMD_ENUM)p_pdu_hdr->pe_hdr.service_id; // PNIO_UINT8 enum
        p_cb->pe_hdr.Handle       = currHandle; // return saved user hndl from PNIO_ctrl_open()
        p_cb->pe_hdr.Addr         = *pnio_addr;
        p_cb->pe_hdr.Ref          = currUserReqRef;
        p_cb->pe_hdr.Err          = e;
        if (pe_pdu_present) {
            p_cb->pe_hdr.state    = p_pdu_hdr->pe_hdr.modif_state;
        }
        else {
            p_cb->pe_hdr.state    = 0x02; // ready-with-error
        }

        p_cb->pe_hdr.struct_id    = 0xFF; /* response struct_id; */

        if ( len >= (sizeof(PE_PDU_DR_HDR) + sizeof(PE_PDU_DT_CONF_NEG)) ) {
            p_cb->pe_hdr.length       = sizeof(PNIO_PE_PRM_GENERIC_CONF_NEG);
            // neg. response pe param (pe error code) present, copy it
            PE_PDU_DR_NEG_CNF *p_pdu = (PE_PDU_DR_NEG_CNF*)pbuff;
            p_cb->pe_prm.RespNegative.err_code  = p_pdu->pe_neg_cnf.err_code;
        }
        else {
            p_cb->pe_hdr.length       = 0;
        }
    }
    else {
        // positive read resp. -  build positive user callback parameter struct - call callback
        //--------------------

        int blk_len  = 0; //p_pdu_hdr->dr_hdr.block_length;
        int param_len= 0; //blk_len - sizeof(p_pdu_hdr->dr_hdr.version_high) - sizeof(p_pdu_hdr->dr_hdr.version_low)
                          //     - sizeof(p_pdu_hdr->pe_hdr);

        if (pe_pdu_present) {
            blk_len  = p_pdu_hdr->dr_hdr.block_length;
            param_len= blk_len - sizeof(p_pdu_hdr->dr_hdr.version_high) - sizeof(p_pdu_hdr->dr_hdr.version_low)
                        - sizeof(p_pdu_hdr->pe_hdr);
        }

        if (param_len < 0) {
            param_len = 0;
        }

        int required_buff_len = sizeof(p_cb->pe_hdr) + param_len;

        p_cb = (PNIO_PE_CBE_PRM*)malloc(required_buff_len);  // <--- dynamic MEM ALLOC !!!
        if (!p_cb) {
            TRC_OUT01(GR_PE, LV_ERR, " _resp_to_user: ERROR: NO MEM  rqd-size=%d", required_buff_len);
            return;
        }
        memset(p_cb, 0, required_buff_len);

        // build user callback parameter struct
        p_cb->pe_hdr.CmdId        = currCmdId; // (PNIO_PE_CMD_ENUM)p_pdu_hdr->pe_hdr.service_id;
        p_cb->pe_hdr.Handle       = currHandle; // return saved user hndl from PNIO_ctrl_open()
        p_cb->pe_hdr.Addr         = *pnio_addr;
        p_cb->pe_hdr.Ref          = currUserReqRef;
        p_cb->pe_hdr.Err          = e;
        p_cb->pe_hdr.state        = p_pdu_hdr->pe_hdr.modif_state;
        p_cb->pe_hdr.struct_id    = p_pdu_hdr->pe_hdr.struct_id;   /* service modifier */

        // fill callback parameters
        switch ( p_pdu_hdr->pe_hdr.service_id ) {
            case PNIO_PE_CMD_START_PAUSE:
                {
                    PNIO_PE_PRM_START_PAUSE_CONF* p_dst = &p_cb->pe_prm.StartPause;
                    PE_PDU_START_PAUSE_CONF_POS*      p_src = &p_pdu_cnf->pe_cnf.cdt.start_pause_conf;
                    p_dst->pe_mode_id = p_src->pe_mode_id;
                    break;
                }
            case PNIO_PE_CMD_END_PAUSE:
                {
                    //PNIO_PE_PRM_END_PAUSE_CONF* p_dst = &p_cb->pe_prm.EndPause;
                    p_cb->pe_prm.EndPause.time_ms = p_pdu_cnf->pe_cnf.cdt.end_pause_conf.time_to_operate;
                    break;
                }
            case PNIO_PE_CMD_Q_MODES:
                if ( p_cb->pe_hdr.struct_id == PE_CMD_STRUCT_ID_Q_MODE_LIST_ALL_RS ) {
                    // list all
                    PNIO_PE_PRM_Q_MODE_LIST_ALL_CONF* p_dst = &p_cb->pe_prm.ModeList;
                    PE_PDU_Q_MODE_LIST_ALL_CONF_POS*  p_src = &p_pdu_cnf->pe_cnf.cdt.q_mode_list_all_conf;
                    p_dst->numModeIds = p_src->num_mode_ids;

                    for (int i=0; i < p_src->num_mode_ids; i++) {
                        p_dst->peModeId[i] = p_src->pe_mode_id[i];
                    }

                }
                else if ( p_cb->pe_hdr.struct_id == PE_CMD_STRUCT_ID_Q_MODE_GET_MODE_RS) {
                    // get mode
                    // TBD!!!
                }
                break;
            case PNIO_PE_CMD_PEM_STATUS:
                {
                    PNIO_PE_PRM_PEM_STATUS_CONF* p_dst = &p_cb->pe_prm.PemStatus;
                    PE_PDU_PEM_STATUS_CONF_POS*  p_src = &p_pdu_cnf->pe_cnf.cdt.pem_status_conf;
                    p_dst->PE_Mode_ID_Source       = p_src->pe_mode_id_source;
                    p_dst->PE_Mode_ID_Destination  = p_src->pe_mode_id_destination;
                    p_dst->Time_to_operate         = p_src->time_to_operate;
                    p_dst->Remaining_time_to_destination     = p_src->remaining_time_to_destination;
                    p_dst->Mode_Power_Consumption            = (float)p_src->mode_power_consumption;
                    p_dst->Energy_Consumption_to_Destination = (float)p_src->energy_consumption_to_destination;
                    p_dst->Energy_Consumption_to_operate     = (float)p_src->energy_consumption_to_operate;
                    break;
                }
            case PNIO_PE_CMD_PE_IDENTIFY:
                // TBD!!!
            case PNIO_PE_CMD_Q_MSRMT:
                // TBD!!!
            default:
                TRC_OUT01(GR_PE, LV_ERR, " _resp_to_user: ERROR: UNKOWN CMD-ID=%#x",currCmdId);

        } /* end switch service_id */

    } // end positive resp.

    // check if PE callback is already registered - call it
    if (m_pUserCbf_PE) {
        TRC_OUT02(GR_PE, LV_INFO, " cPeMgt: --> user PE Conf callback ErrCode=%#x CmdId=%#x",e.ErrCode, currCmdId);
        m_pUserCbf_PE( (PNIO_PE_CBE_PRM*) p_cb);
        TRC_OUT01(GR_PE, LV_INFO, " cPeMgt: <-- user PE Conf callback done! CmdId=%#x", currCmdId);
    }

    // free allocated callback param struct.
    if (p_cb) {
        free(p_cb);
    }

    TRC_OUT02(GR_PE, LV_INFO, "cPeMgt::pass_resp_to_user: WR=%d UserRef=%#x",dr_rw_action_type, ReqRef);
    return;
} /* end of pass_resp_to_user */




///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
/// cPeTimer
///

static DPR_THREAD_RETURN_TYPE DPR_THREAD_DECL peTimerThEntry(void *arg);

cPeTimer::cPeTimer( cPeMgt* p_parent, int time_ms, bool cyclic) :
  m_state(st_idle),
  m_pPeMgt (p_parent),
  m_TimerId(0xffff),
  m_time_ms(time_ms),
  m_stop_thread(0),
  m_th(0)
{
    TRC_OUT(GR_PE, LV_INFO, "->cPeTimer::Ctor");

    /* create thread for calling tick callback */
    if( ! DPR_THREAD_CREATE(&m_th, "", peTimerThEntry, this) ) {
        TRC_OUT(GR_PE, LV_ERR, "PE: create th ERROR ");
    }


}

cPeTimer::~cPeTimer()
{
    TRC_OUT(GR_PE, LV_INFO, "->cPeTimer::Dtor");
    // kill the thread
    TRC_OUT(GR_PE, LV_INFO, "  cPeTimer: Th->stopped: waiting... ");
    m_stop_thread = 1;
    DPR_THREAD_JOIN(m_th);
    m_th = 0;
    TRC_OUT(GR_PE, LV_INFO, "<-cPeTimer::Dtor");

}

void cPeTimer::start (void)
{
    TRC_OUT(GR_PE, LV_INFO, " cPeTimer start");
    m_state = st_running;
}

void cPeTimer::stop  (void)
{
    TRC_OUT(GR_PE, LV_INFO, " cPeTimer stop");
    m_state = st_idle;
}



//----------------------------------------------------------------------------
// Timer thread - calls timer callback function
//
static DPR_THREAD_RETURN_TYPE DPR_THREAD_DECL peTimerThEntry(void *arg)
{
  cPeTimer *pPeTimer = (cPeTimer*)arg;
  int loop_cnt = 0;

  TRC_OUT(GR_PE, LV_INFO, "->peTimerThEntry endless loop ...");

  /* loop till shutdown req */
  while(!pPeTimer->m_stop_thread) {
      TRC_OUT01(GR_PE, LV_INFO, " peTimerThEntry TICK: %04d", loop_cnt);

      if ( pPeTimer->m_state == cPeTimer::st_running ) {
          pPeTimer->rawCallback(1, pPeTimer);
      }
      DPR_TASK_DELAY(pPeTimer->m_time_ms);

      loop_cnt++;
  } // end while

  TRC_OUT(GR_PE, LV_INFO, "<-peTimerThEntry STOPPED exiting ...");

  DPR_THREAD_END();
}

#endif /* PROFI_ENERGY */

