/*****************************************************************************/
/*  Copyright (C) SIEMENS CORP., 2013 All rights reserved.*/
/*****************************************************************************/
/*  F i l e                l2eth_base.cpp                                    */
/*****************************************************************************/
/*  This module contains the common dual port RAM functions. There are       */
/*  no os dependent functions.                                               */
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

#include "os.h"
#include "cp16xx.h"

#include "l2eth_config.h"
#include "l2eth_errs.h"
#include "l2eth_defs.h"
#include "l2eth_user.h"
#include "l2eth_base.h"

#include "l2eth_rqb.h"
#include "l2eth_buf_mgmt.h"

#ifdef __KERNEL__
#include <linux/sched.h>
#include <linux/time.h>
#else
#include "tracemcr.h"
#endif

/*------------------------------------------------------------
 * defines
------------------------------------------------------------*/
/* layer2 interface needs to filter LLDP frames */
#define L2ETH_LLDP_CHECK

/* maximum multicast size */
#define L2ETH_MAX_MULTICAST_LIST_MAC_ADDRESS 19

/* maximum ports supported by the card */
#define L2ETH_MAX_PORTS 4


#ifdef __KERNEL__
static DECLARE_COMPLETION(on_exit);
#endif

/* error handling blocks - avoid goto  */
#define BEGIN_BLOCK do
#define LEAVE_BLOCK break
#define END_BLOCK  while (0)

L2ETH_UINT32 getMinBufferLength(L2ETH_OID tag)
{
    switch(tag) {
    case L2ETH_OID_READWRITE:
        return 0;
    case L2ETH_OID_MULTICAST_LIST:
        return sizeof (L2ETH_MAC_ADDR);
    case L2ETH_OID_MAXIMUM_FRAME_SIZE:
        return sizeof (L2ETH_UINT32);
    case L2ETH_OID_READONLY:
        return 0;
    case L2ETH_OID_PERMANENT_ADDRESS:
        return sizeof (L2ETH_MAC_ADDR);
    case L2ETH_OID_MAXIMUM_LIST_SIZE:
        return sizeof (L2ETH_UINT32);
    case L2ETH_OID_MEDIA_CONNECT_STATUS:
        return sizeof (L2ETH_PORT_STATUS) * L2ETH_MAX_PORTS;
    default:
        return 0;
    }
}

L2ETH_UINT32 getMaxBufferLength(L2ETH_OID tag)
{
    switch(tag) {
    case L2ETH_OID_READWRITE:
        return 0;
    case L2ETH_OID_MULTICAST_LIST:
        return sizeof (L2ETH_MAC_ADDR) * L2ETH_MAX_MULTICAST_LIST_MAC_ADDRESS;
    case L2ETH_OID_MAXIMUM_FRAME_SIZE:
        return sizeof (L2ETH_UINT32);
    case L2ETH_OID_READONLY:
        return 0;
    case L2ETH_OID_PERMANENT_ADDRESS:
        return sizeof (L2ETH_MAC_ADDR);
    case L2ETH_OID_MAXIMUM_LIST_SIZE:
        return sizeof (L2ETH_UINT32);
    case L2ETH_OID_MEDIA_CONNECT_STATUS:
        return sizeof (L2ETH_PORT_STATUS) * L2ETH_MAX_PORTS;
    default:
        return 0;
    }
}

/*------------------------------------------------------------
// forward declarations
------------------------------------------------------------*/
DPR_THREAD_RETURN_TYPE DPR_THREAD_DECL procChannelRead(L2ETH_VOID * arg);
inline L2ETH_UINT32 add_handle(l2eth_base * pInst, L2ETH_UINT32 pos);
inline L2ETH_UINT32 remove_handle(L2ETH_UINT32 pos);
inline L2ETH_UINT32 l2_get_handle(l2eth_base * icp);

static L2ETH_UINT32 context_handle = 0;
L2ETH_UINT32 get_new_context_handle(L2ETH_MODE Mode)
{
    if (Mode == L2ETH_ONLINE)
        return ++context_handle;

    return context_handle;
}

/*------------------------------------------------------------
// methods and structure to keep list of application instance
------------------------------------------------------------*/
/*#define MAX_CP_SUPPORTED 8*/

/*
   g_L2handleArray keeps l2eth_base * for each cp, based on cpid
   ie l2eth_base * for cpIndex 1 is kept at g_L2handleArray[0],
   ie l2eth_base * for cpIndex 2 is kept at g_L2handleArray[1]  ....
*/

l2eth_base * g_L2handleArray[MAX_CP_SUPPORTED] = { NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL };

/* add a element at pos */
inline L2ETH_UINT32 add_handle(l2eth_base * pInst, L2ETH_UINT32 pos)
{
    L2ETH_UINT32 Ret = L2ETH_OK;
    if(!g_L2handleArray[pos]) {
        /* empty slot add value */
        g_L2handleArray[pos] = pInst;
    } else {
        Ret = L2ETH_ERR_PRM_HND;
    }
    return Ret;
}

/* remove a element at pos */
inline L2ETH_UINT32 remove_handle(L2ETH_UINT32 pos)
{
    L2ETH_UINT32 Ret = L2ETH_OK;
    if(!g_L2handleArray[pos]) {
        /* error */
        Ret = L2ETH_ERR_PRM_HND;
    } else {
        //delete is done by close method
        g_L2handleArray[pos] = NULL;
    }
    return Ret;
}

/* l2_get_handle */
inline L2ETH_UINT32 l2_get_handle(l2eth_base * icp)
{
    return (icp ?
        ((icp->m_pCpAdapter->cp_index - 1) + APP_HANDLE_OFFSET_BASE) : L2ETH_ERR_PRM_HND);
}

/* l2_get_instance */
l2eth_base *l2_get_instance(L2ETH_UINT32 handle)
{
    L2ETH_INT32 pos;
    /* AppHandle is sum of APP_HANDLE_OFFSET_BASE and cpid-1 */
    pos = handle - APP_HANDLE_OFFSET_BASE;
    if((pos >= 0) && (pos < MAX_CP_SUPPORTED)) {
        /* valid range of cpids
           check if added to array */
        if(g_L2handleArray[pos]) {
            /* passed found instance value return instance */
            return g_L2handleArray[pos];
        }
    }
    /* error */
    return NULL;
}


//------------------------------------------------------------



//------------------------------------------------------------
// Constructor
//------------------------------------------------------------
L2ETH_VOID l2_base_cons(l2eth_base * cp)
{
    cp->m_bEmergencyClose = L2ETH_FALSE;
    cp->m_dma_receive_frame_count = 0;
    cp->m_dma_send_frame_count = 0;

    cp->m_l2_mode = L2ETH_OFFLINE;
    cp->m_bmodeChangeRequested = L2ETH_FALSE;
    cp->m_maxOidFrameSize = MAX_ETHERNET_FRAME_SIZE;
    cp->m_pCpAdapter = NULL;

    cp->m_send_cbf = NULL;
    cp->m_recv_cbf = NULL;
    cp->m_status_cbf = NULL;
    cp->m_mode_cbf = NULL;

    cp->m_hSendBuff = NULL;
    cp->m_hRecvBuff = NULL;

    TRC_INIT_FROM_FILE("l2eth_trace.conf");
    TRC_OUT(GR_INIT, LV_FCTPUB1, "HOST L2ETH build version 01.04.00.00");
}

//------------------------------------------------------------
// Destructor
//------------------------------------------------------------
L2ETH_VOID l2_base_des(l2eth_base * cp)
{
    cp->m_pCpAdapter = NULL;

    cp->m_send_cbf = NULL;
    cp->m_recv_cbf = NULL;
    cp->m_status_cbf = NULL;
    cp->m_mode_cbf = NULL;

    cp->m_hSendBuff = NULL;
    cp->m_hRecvBuff = NULL;
    TRC_OUT(GR_INIT, LV_FCTPUB1, "l2eth_base_des called ");

    TRC_DEINIT();
}


//------------------------------------------------------------
// interface support functions
//------------------------------------------------------------


//------------------------------------------------------------
//  open
//------------------------------------------------------------
L2ETH_UINT32 l2_open(L2ETH_UINT32 cpIndex, /* in */
    L2ETH_CBF_RECEIVE_IND receiveCbf,   /* in */
    L2ETH_CBF_SEND_COMPL sendCompleteCbf,       /* in */
    L2ETH_CBF_STATUS_IND indicationCbf, /* in */
    L2ETH_CBF_MODE_COMPL modeCompCbf,   /* in */
    L2ETH_UINT32 * applHandle /* out: */ )
{
    l2eth_base *pThis = NULL;
    L2ETH_UINT16 sendLen;
    L2ETH_UINT32 Ret = L2ETH_OK, expRLen;

    L2ETH_MSG_COMMAND Rq;

    /* check already opened or not
       handle = APP_HANDLE_OFFSET_BASE + cpIndex-1 */
    pThis = l2_get_instance(APP_HANDLE_OFFSET_BASE + cpIndex - 1);

    if(pThis) {
        TRC_OUT01(GR_INIT, LV_ERR,
            " already opened once. return handle %u!",
            (cpIndex - 1) + APP_HANDLE_OFFSET_BASE);
        TRC_OUT01(GR_INIT, LV_FCTPUB1, "<- l2_open cp_index %d", cpIndex);
        return L2ETH_ERR_MAX_REACHED;
    }

    /* create an instance  */
    pThis = (l2eth_base *) DPR_ZALLOC(sizeof (l2eth_base));
    if(!pThis) {
        TRC_OUT(GR_INIT, LV_ERR, " failed to create instance - l2eth_base!");
        return L2ETH_ERR_NO_RESOURCE;
    }

    /* initialize structure */
    l2_base_cons(pThis);


    TRC_OUT(GR_INIT, LV_FCTPUB1, "->   l2_open ");

    Ret = l2_init_cp(pThis, cpIndex);
    if(L2ETH_OK != Ret) {
        DPR_FREE(pThis);
        return Ret;
    }
    TRC_OUT(GR_INIT, LV_FCTPUB1, "  l2_init_cp ok ");

    /* send message to FW */
    BEGIN_BLOCK {

        sendLen = sizeof (L2ETH_MSG_COMMAND);
        expRLen = sizeof (L2ETH_MSG_HEADER);

        memset(&Rq, 0, sizeof (Rq));

        Rq.l2_header.blk_len = CPU_TO_LE16(sendLen);
        Rq.l2_header.opcode = SCO_L2ETH_OPEN;

        /* set the physical address */
        Rq.u.open_l2.dma_phys_adds =
            CPU_TO_LE(pThis->m_pCpAdapter->m_dmaPhysicalPool);
        /* set the dma_virt_adds as physical address - EDD specific for future */
        Rq.u.open_l2.dma_virt_adds = Rq.u.open_l2.dma_phys_adds;
        /* set preload buffer */
        Rq.u.open_l2.preload_buf_cnt = CPU_TO_LE(PRE_LOAD_BUF_SIZE);
        /* preload address is at the begining of dma pool */
        Rq.u.open_l2.preload_buf = Rq.u.open_l2.dma_phys_adds;

        /* send open request - synch call */
        Ret = l2_send_receive_synch(pThis, (L2ETH_UINT8 *) & Rq, sendLen, &expRLen);

        if(Ret != L2ETH_OK) {
            LEAVE_BLOCK;
        }
        if(Rq.l2_header.opcode != SCO_L2ETH_OPEN) {
            TRC_OUT01(GR_INIT, LV_ERR,
                "   l2_open received unknown opcode = 0x%x", Rq.l2_header.opcode);
            Ret = L2ETH_ERR_INTERNAL;
            LEAVE_BLOCK;
        }

        Ret = Rq.l2_header.resp_ret;
        if(Ret == L2ETH_OK) {
            /* save user callback in instance structure */
            pThis->m_send_cbf = sendCompleteCbf;
            pThis->m_recv_cbf = receiveCbf;
            pThis->m_status_cbf = indicationCbf;
            pThis->m_mode_cbf = modeCompCbf;

            /* TRC_OUT(GR_INIT, LV_FCTPUB1, "   open send_receive_synch OK"); */

            /* save App handle */
            add_handle(pThis, cpIndex - 1);

            /* return App handle to user  */
            /* user handle as APP_HANDLE_OFFSET_BASE + cpIndex - 1 */
            *applHandle = APP_HANDLE_OFFSET_BASE + cpIndex - 1;

        } else {
            TRC_OUT01(GR_INIT, LV_ERR, "  Error send_receive_synch returned 0x%x", Ret);
            LEAVE_BLOCK;
        }
    }
    END_BLOCK;

    if(L2ETH_OK != Ret) {
        l2_uninit_cp(pThis);

        /* delete the instance */
        DPR_FREE(pThis);
    }

    return Ret;
}

//------------------------------------------------------------
//  l2_close
//------------------------------------------------------------
L2ETH_UINT32 l2_close(l2eth_base * cp)
{

    L2ETH_UINT16 sendLen;
    L2ETH_UINT32 Ret = L2ETH_OK, expRLen;

    L2ETH_MSG_COMMAND Rq;

    TRC_OUT(GR_INIT, LV_FCTPUB1, "->  l2_close...");

    /* check mode - mode must be offline to close */
    if((L2ETH_OFFLINE != cp->m_l2_mode) || (L2ETH_TRUE == cp->m_bmodeChangeRequested)) {
        /* requested achange of mode must wait for mode change callback */
        Ret = L2ETH_ERR_SEQUENCE;
        TRC_OUT01(GR_INIT, LV_ERR, " l2_close offline mode is not set, Ret = 0x%x", Ret);
        return Ret;

    }

    sendLen = sizeof (L2ETH_MSG_COMMAND);
    expRLen = sizeof (L2ETH_MSG_HEADER);

    memset(&Rq, 0, sizeof (Rq));
    Rq.l2_header.blk_len = CPU_TO_LE16(sendLen);
    Rq.l2_header.opcode = SCO_L2ETH_CLOSE;
    Rq.u.close_l2.EmergencyClose =
        (L2ETH_UINT8) CPU_TO_LE(cp->m_bEmergencyClose ? 0x01 : 0x00);

    Ret = l2_send_receive_synch(cp, (L2ETH_UINT8 *) & Rq, sendLen, &expRLen);

    if(Ret != L2ETH_OK) {
        TRC_OUT01(GR_INIT, LV_ERR, " l2_close send_receive_synch Ret = 0x%x", Ret);
        return Ret;
    }

    if(Rq.l2_header.opcode != SCO_L2ETH_CLOSE) {
        Ret = L2ETH_ERR_INTERNAL;
        TRC_OUT01(GR_INIT, LV_ERR,
            " l2_close received unknown opcode = 0x%x", Rq.l2_header.opcode);
        return Ret;
    }

    Ret = Rq.l2_header.resp_ret;
    if(Ret != L2ETH_OK) {
        TRC_OUT01(GR_INIT, LV_ERR, " l2_close Rq.l2_header.resp_ret = 0x%x", Ret);
    } else {

        /* remove handle */
        remove_handle(cp->m_pCpAdapter->cp_index - 1);

        l2_uninit_cp(cp);

        DPR_FREE(cp);
    }
    TRC_OUT(GR_INIT, LV_FCTPUB1, "<-  l2_close...");

    return Ret;
}

//------------------------------------------------------------
//  l2_get_allocate_packet  - // get free buffer to send packet
//------------------------------------------------------------
L2ETH_UINT32 l2_get_allocate_packet(l2eth_base * cp,
    L2ETH_PACKET ** pPkt) /* out */
{
    L2ETH_UINT32 Ret = L2ETH_OK;

    /* check for NULL */
    if(!cp->m_hSendBuff) {
        TRC_OUT(GR_INIT, LV_ERR, "ERROR m_hSendBuff memory pool not set");
        return L2ETH_ERR_NO_RESOURCE;
    }

    /* get packet from mubbffer manager */
    if(L2ETH_BUF_SUCCESS != l2_buf_get_next(cp->m_hSendBuff, pPkt)) {
        TRC_OUT(GR_INIT, LV_ERR, "ERROR l2_buf_get_next ");
        Ret = L2ETH_ERR_NO_RESOURCE;
    }

    return Ret;
}

//------------------------------------------------------------
//  l2_validate_packet_buffer
//------------------------------------------------------------
L2ETH_UINT32 l2_validate_packet_buffer(l2eth_base * cp,
    L2ETH_PACKET_BUFFER_TYPE BuffType,
    L2ETH_PACKET * packet)
{
    L2ETH_UINT32 Ret = L2ETH_OK;

    if(L2ETH_DMA_SEND_BUFFER == BuffType) {
        /* check the pBuffer is inside the dma area for Send buffers */
        if((packet->pBuffer <
                (cp->m_pCpAdapter->dma_base_ptr +
                    MAX_ETHERNET_FRAME_SIZE * cp->m_dma_receive_frame_count))
            || (packet->pBuffer >
                (cp->m_pCpAdapter->dma_base_ptr +
                    (MAX_ETHERNET_FRAME_SIZE * cp->m_dma_receive_frame_count) +
                    (MAX_ETHERNET_FRAME_SIZE * cp->m_dma_send_frame_count)))) {
            Ret = L2ETH_ERR_PRM_BUF;
        }
    } else if(L2ETH_DMA_RECEIVE_BUFFER == BuffType) {
        /* check the pBuffer is inside the dma area for receive buffers */
        if((packet->pBuffer < cp->m_pCpAdapter->dma_base_ptr)
            || (packet->pBuffer >
                (cp->m_pCpAdapter->dma_base_ptr +
                    (MAX_ETHERNET_FRAME_SIZE * cp->m_dma_receive_frame_count)))) {
            Ret = L2ETH_ERR_PRM_BUF;
        }
    }
    return Ret;
}


#ifdef L2ETH_LLDP_CHECK

#define L2ETH_LLDP_ETHERTYPE     0x88CC /* LLDP  - LE */
#define L2ETH_VLAN_TAG           0x8100 /* LLDP  - LE */

//------------------------------------------------------------
//  isLLDP_frame
//------------------------------------------------------------
L2ETH_BOOL isLLDP_frame(L2ETH_UINT8 * pFrame, L2ETH_UINT32 FrameLen)
{
    L2ETH_UINT16 TypeLen;

    if (FrameLen < 14)
        return L2ETH_FALSE; /* too short */

    /* -----------------------------------------------------------------------*/
    /* get bytes 12..15 from Frame                                            */
    /* Note that this is in inet-format (big endian)                          */
    /* -----------------------------------------------------------------------*/

    /* skip Dest. und Src. MAC Addr, total 12 bytes */
    TypeLen = ((L2ETH_UINT16 *)pFrame)[6];

    /* -----------------------------------------------------------------------*/
    /* check for VLAN-Tag. if so we must read the real Type/len 4 Bytes ahead */
    /* -----------------------------------------------------------------------*/
    if (TypeLen == CPU_TO_BE16(L2ETH_VLAN_TAG)) {
        if (FrameLen < 18) {
            return L2ETH_FALSE; /* too short */
        }

        TypeLen = ((L2ETH_UINT16 *)pFrame)[8];
    }

    /* -----------------------------------------------------------------------*/
    /* Now check for the LLDP Frametype                                       */
    /* -----------------------------------------------------------------------*/
    if (TypeLen == CPU_TO_BE16(L2ETH_LLDP_ETHERTYPE)) {
        return L2ETH_TRUE;
    }

    return L2ETH_FALSE;
}
#endif

//------------------------------------------------------------
//  l2_send_packet  - send packets to l2 interface
//------------------------------------------------------------
L2ETH_UINT32 l2_send_packet(l2eth_base * cp, L2ETH_PACKET * packet)
{
    L2ETH_UINT32 Ret, offset;
    L2ETH_MSG_COMMAND SendPacketRq;
    L2ETH_UINT16 sendLen;

    /* check mode - mode must be online to send packet */
    if(L2ETH_ONLINE != cp->m_l2_mode || L2ETH_TRUE == cp->m_bmodeChangeRequested) {
        /* requested achange of mode must wait for mode change callback  */
        Ret = L2ETH_ERR_SEQUENCE;
        TRC_OUT01(GR_INIT, LV_ERR,
            " l2_send_packet online mode is not set, Ret = 0x%x", Ret);
        return Ret;
    }

    /* user is trying to send a very big frame */
    if(MAX_L2_FRAME_SIZE < packet->DataLength || MIN_L2_FRAME_SIZE > packet->DataLength) {
        Ret = L2ETH_ERR_PRM_LEN;
        TRC_OUT01(GR_CHNL, LV_ERR, " user is trying to send a frame of invalid  size %u",
            packet->DataLength);
        return Ret;
    }

    /* check the packet is a valid 1 or not */
    Ret = l2_validate_packet_buffer(cp, L2ETH_DMA_SEND_BUFFER, packet);
    if(L2ETH_OK != Ret) {
        TRC_OUT01(GR_CHNL, LV_ERR, " send buffer failed validate_packet_buffer .. %p",
            packet->pBuffer);
        return Ret;
    }

#ifdef L2ETH_LLDP_CHECK
    /* reject LLDP frames */
    if(L2ETH_TRUE == isLLDP_frame((L2ETH_UINT8 *) packet->pBuffer, packet->DataLength)) {
        TRC_OUT01(GR_CHNL, LV_ERR, " trying to send LLDP frame %p", packet->pBuffer);
        return L2ETH_ERR_LLDP_FRAME;
    }
#endif

    sendLen = sizeof (L2ETH_MSG_HEADER) + sizeof(rq_packet_l2);
    memset(&SendPacketRq, 0, sizeof(SendPacketRq));
    SendPacketRq.l2_header.blk_len = CPU_TO_LE16(sendLen);
    SendPacketRq.l2_header.opcode = SCO_L2ETH_SEND;
    SendPacketRq.l2_header.handle = CPU_TO_LE(cp->m_context_handle);

    offset = (L2ETH_UINT32)(packet->pBuffer - cp->m_pCpAdapter->dma_base_ptr);

    SendPacketRq.u.send_pkts_l2.BufferOffset =
        CPU_TO_LE(cp->m_pCpAdapter->m_dmaPhysicalPool + offset);
    SendPacketRq.u.send_pkts_l2.DataLength = CPU_TO_LE(packet->DataLength);
    SendPacketRq.u.send_pkts_l2.Context = CPU_TO_LE(packet->Context);

    /* send the message to FW through dpr channel */
    Ret = l2_channel_write(cp, L2ETH_SEND, (L2ETH_UINT8 *) & SendPacketRq,
        sizeof (L2ETH_MSG_COMMAND));
    if(L2ETH_OK != Ret) {
        TRC_OUT01(GR_CHNL, LV_ERR, " fail to send %d bytes", sendLen);
    }

    return Ret;
}


//------------------------------------------------------------
//  l2_free_packet  - returns allocated packet back to l2 interface
//                      return the packet to the send buffer handler
//------------------------------------------------------------
L2ETH_UINT32 l2_free_packet(l2eth_base * cp, L2ETH_PACKET * pPkt)
{
    L2ETH_UINT32 Ret = L2ETH_OK;

    //TRC_OUT01(GR_INIT, LV_FCTPUB1, "-> l2_free_packet address 0x%x", pPkt->pBuffer);

    /* validate the buffer pointers */
    Ret = l2_validate_packet_buffer(cp, L2ETH_DMA_SEND_BUFFER, pPkt);
    if(L2ETH_OK != Ret) {
        TRC_OUT01(GR_CHNL, LV_ERR, " send buffer is not from dma pool.. %p",
            pPkt->pBuffer);
        return Ret;
    }

    if(!cp->m_hSendBuff) {
        TRC_OUT(GR_INIT, LV_ERR, "ERROR m_hSendBuff memory pool not set");
        return L2ETH_ERR_INTERNAL;
    }
    //call buffer management - free buffer m_hSendBuff
    if(L2ETH_BUF_SUCCESS != l2_buf_free(cp->m_hSendBuff, pPkt)) {
        TRC_OUT(GR_INIT, LV_ERR, "ERROR free packet m_hSendBuff ");
        return L2ETH_ERR_INTERNAL;
    }
    //TRC_OUT(GR_MGT, LV_FCTPUB1, "<- l2_free_packet done");

    return Ret;
}


//------------------------------------------------------------
//  l2_return_packet  - returns read packets back to l2 interface
//------------------------------------------------------------
L2ETH_UINT32 l2_return_packet(l2eth_base * cp, L2ETH_PACKET * pPkt, L2ETH_PACKET_OWNER owner)
{
    //return the packet to the buffer handler
    L2ETH_UINT32 Ret = L2ETH_OK;

    TRC_OUT01(GR_INIT, LV_FCTPUB1, "-> l2_return_packet address %p", pPkt->pBuffer);

    Ret = l2_validate_packet_buffer(cp, L2ETH_DMA_RECEIVE_BUFFER, pPkt);
    if(L2ETH_OK != Ret) {
        TRC_OUT01(GR_CHNL, LV_ERR, " receive buffer is not from dma pool.. %p",
            pPkt->pBuffer);
        return Ret;
    }

    /* try to relase the ownership  */
    if(l2_release_rbuff_ownership(cp, (L2ETH_UINT8 *) pPkt->pBuffer, owner)) {
        /* user does not have ownership for this buffer / invalid buffer pointer */
        TRC_OUT01(GR_INIT, LV_ERR, "l2_return_packet: L2ETH_ERR_SEQUENCE 0x%x ",
            L2ETH_ERR_SEQUENCE);
        return L2ETH_ERR_SEQUENCE;
    }

    /* check mode - if offline return packet to buffer mgmt */
    if((L2ETH_OFFLINE == cp->m_l2_mode) || (L2ETH_PACKET_OWNER_FW == owner)) {
        /* free the packet back to the buffer mgmt */
        if(!cp->m_hRecvBuff) {
            return L2ETH_ERR_INTERNAL;
        }
        //TRC_OUT01(GR_INIT, LV_FCTPUB1, "-> call l2_buf_free address 0x%x",pPkt->pBuffer);

        //call buffer management - free buffer m_hRecvBuff
        if(L2ETH_BUF_SUCCESS != l2_buf_free(cp->m_hRecvBuff, pPkt)) {
            TRC_OUT(GR_INIT, LV_ERR, "ERROR free buffer m_hRecvBuff ");
            return L2ETH_ERR_INTERNAL;
        }
    } else {
        // send the packet again to the L2 Agent in FW
        Ret = l2_send_receive_packet(cp, pPkt);
    }

    TRC_OUT(GR_MGT, LV_FCTPUB1, "<- l2_return_packet done");

    return Ret;
}

//------------------------------------------------------------
//   set_mode
//------------------------------------------------------------
L2ETH_UINT32 l2_set_mode(l2eth_base * cp, L2ETH_MODE Mode)
{
    L2ETH_UINT32 Ret = L2ETH_OK;
    L2ETH_UINT16 sendLen;
    L2ETH_MSG_COMMAND Rq;

    TRC_OUT(GR_INIT, LV_FCTPUB1, "->  l2_set_mode");

    /* mode change is requested ... user must wait for mode change call back */
    cp->m_bmodeChangeRequested = L2ETH_TRUE;

    /* set command values */
    sendLen = sizeof (L2ETH_MSG_COMMAND);

    memset(&Rq, 0, sizeof (Rq));
    Rq.l2_header.blk_len = CPU_TO_LE16(sendLen);
    Rq.l2_header.opcode = SCO_L2ETH_SET_MODE;
    Rq.l2_header.handle = CPU_TO_LE(get_new_context_handle(Mode));
    Rq.u.mode_l2.Mode = (L2ETH_MODE)CPU_TO_LE(Mode);

    /* send message to fw */
    Ret = l2_channel_write(cp, L2ETH_SEND, (L2ETH_UINT8 *) & Rq, sendLen);

    if(Ret != L2ETH_OK) {
        TRC_OUT01(GR_INIT, LV_ERR, " l2_set_mode channel_write Ret = 0x%x", Ret);
    }

    TRC_OUT(GR_MGT, LV_FCTPUB1, "<-  l2_set_mode ");

    return Ret;
}

//------------------------------------------------------------
//  l2_getset_information
//------------------------------------------------------------
L2ETH_UINT32 l2_getset_information(l2eth_base * cp, GET_SET_TYPE getsettype,
    L2ETH_QUERY * pQuery)
{
    L2ETH_UINT32 Ret = L2ETH_OK;
    L2ETH_UINT32 sendLen, expRLen, offset;
    L2ETH_MSG_COMMAND Rq;
    L2ETH_VOID *dmaQryBuff;
//    TRC_OUT(GR_INIT, LV_FCTPUB1, "->  l2_getset_information");

    /* set default error values */
    pQuery->BytesNeeded = getMinBufferLength(pQuery->Oid);
    pQuery->BytesTransferred = 0;


    /* check if oids are the delimiters or out of range used */
    if((L2ETH_OID_READWRITE == (L2ETH_UINT32) pQuery->Oid) ||
        (L2ETH_OID_READONLY == (L2ETH_UINT32) pQuery->Oid) || (L2ETH_OID_MAX <= (L2ETH_UINT32) pQuery->Oid)) {
        /* return error */
        Ret = L2ETH_ERR_PRM_OID;
        TRC_OUT01(GR_INIT, LV_ERR,
            "set_information trying to set delimiter OID = 0x%x", Ret);
        return Ret;
    }

    /* set information */
    memset(&Rq, 0, sizeof(Rq));
    if(SET_INFO == getsettype) {
        /* check if user wants to set any read only oids */
        if(L2ETH_OID_READONLY < (L2ETH_UINT32) pQuery->Oid) {
            /* return error */
            Ret = L2ETH_ERR_PRM_OID;    // L2ETH_ERR_OID_READONLY- requested in doc review
            TRC_OUT01(GR_INIT, LV_ERR,
                "set_information trying to set read only OIDs = 0x%x", Ret);
            return Ret;
        }

        /* if user sends bigger size than allowed for set_info return error */
        if(getMaxBufferLength(pQuery->Oid) < pQuery->BufferLength) {
            TRC_OUT01(GR_INIT, LV_INFO,
                "BufferLength is more than max=%d ", getMaxBufferLength(pQuery->Oid));

            //pQuery->BufferLength = getMaxBufferLength(pQuery->Oid);
            pQuery->BytesNeeded = getMaxBufferLength(pQuery->Oid);
            return L2ETH_ERR_PRM_LEN;
        }

        /* SCO_L2ETH_SET_INFO as opcode for command */
        Rq.l2_header.opcode = SCO_L2ETH_SET_INFO;

    } else {                    /* get information */
        /* opcode for command as SCO_L2ETH_GET_INFO */
        Rq.l2_header.opcode = SCO_L2ETH_GET_INFO;
    }


    /* checking is needed only for minimum size */
    if(getMinBufferLength(pQuery->Oid) > pQuery->BufferLength) {
        TRC_OUT01(GR_INIT, LV_ERR,
            "BufferLength is lower than min=%d ", getMinBufferLength(pQuery->Oid));

        //pQuery->BytesNeeded = l2eth_oid_lengths[pQuery->Oid].minBuf_Length; //already initialized

        return L2ETH_ERR_PRM_LEN;
    }


    /* check iod is OID_GEN_MAXIMUM_FRAME_SIZE
       OID_GEN_MAXIMUM_FRAME_SIZE is handled by the host interface
       not passed to FW */
    if(L2ETH_OID_MAXIMUM_FRAME_SIZE == (L2ETH_UINT32) pQuery->Oid) {
        TRC_OUT(GR_MGT, LV_FCTPUB1, "L2ETH_OID_MAXIMUM_FRAME_SIZE info");

        /* set information */
        if(SET_INFO == getsettype) {
            if(MAX_ETHERNET_FRAME_SIZE < *(L2ETH_UINT32 *) pQuery->pBuffer) {
                /* requested size more than maximum */
                //pQuery->BytesTransferred = 0;  //initialised to 0
                Ret = L2ETH_ERR_PRM_BUF;
                TRC_OUT02(GR_INIT, LV_ERR,
                    "L2ETH_OID_MAXIMUM_FRAME_SIZE req frame size 0x%x > MAX_ETHERNET_FRAME_SIZE = 0x%x",
                    *(L2ETH_UINT32 *) pQuery->pBuffer, MAX_ETHERNET_FRAME_SIZE);
                return Ret;

            }
            /* set maximum oid frame size */
            pQuery->BytesNeeded = 0;
            pQuery->BytesTransferred = getMaxBufferLength(pQuery->Oid);
            memcpy(&cp->m_maxOidFrameSize, pQuery->pBuffer, pQuery->BytesTransferred);
            TRC_OUT01(GR_INIT, LV_FCTPUB1, "set L2ETH_OID_MAXIMUM_FRAME_SIZE done 0x%x",
                cp->m_maxOidFrameSize);
            return Ret;
        } else {
            /* get maximum oid frame size */
            pQuery->BytesNeeded = 0;
            pQuery->BytesTransferred = getMaxBufferLength(pQuery->Oid);
            memcpy(pQuery->pBuffer, &cp->m_maxOidFrameSize, pQuery->BytesTransferred);
            TRC_OUT01(GR_INIT, LV_FCTPUB1, "get L2ETH_OID_MAXIMUM_FRAME_SIZE done 0x%x",
                cp->m_maxOidFrameSize);
            return Ret;
        }

    }

    /* setting L2ETH_OID_MULTICAST_LIST needs BufferLength divisible by 6 */
    if((L2ETH_OID_MULTICAST_LIST == (L2ETH_UINT32) pQuery->Oid) && (SET_INFO == getsettype) &&
        (pQuery->BufferLength % sizeof (L2ETH_MAC_ADDR) != 0)) {
        TRC_OUT(GR_INIT, LV_ERR,
            "set_information L2ETH_OID_MULTICAST_LIST length must be divicible by 6");
        return L2ETH_ERR_PRM_LEN;
    }


    /* send the request to Firmware */
    sendLen = sizeof (L2ETH_MSG_COMMAND);
    expRLen = sizeof (L2ETH_MSG_COMMAND);

    Rq.l2_header.blk_len = (L2ETH_UINT16) CPU_TO_LE(sendLen);


    /* copy query pBuffer to dma area and modify the address */
    dmaQryBuff = (L2ETH_VOID *) l2_get_dma_queryInfo_buffer(cp);

    if(!dmaQryBuff) {
        Ret = L2ETH_ERR_NO_RESOURCE;
        TRC_OUT01(GR_INIT, LV_ERR,
            " l2_getset_information failed getting dma buffer = 0x%x",
            Rq.l2_header.opcode);
        return Ret;
    }
    TRC_OUT02(GR_INIT, LV_FCTPUB1, "dmaQryBuff %p, size %d", dmaQryBuff,
        MAX_DMA_QUERY_BUFFER_SIZE);

    /* copy  pQuery->pBuffer to dmaQueryInfo */
    memcpy(dmaQryBuff, pQuery->pBuffer, pQuery->BufferLength);

    /* get dma relative offset */
    offset = (L2ETH_UINT32)((L2ETH_UINT8 *)dmaQryBuff - cp->m_pCpAdapter->dma_base_ptr);
    TRC_OUT01(GR_INIT, LV_FCTPUB1, "pQuery->pBuffer offset 0x%x", offset);


    Rq.u.query_l2.Oid = (L2ETH_UINT32) CPU_TO_LE(pQuery->Oid);
    Rq.u.query_l2.BufferOffset =
        (L2ETH_UINT32) CPU_TO_LE(cp->m_pCpAdapter->m_dmaPhysicalPool + offset);
    Rq.u.query_l2.BufferLength = CPU_TO_LE(pQuery->BufferLength);
    Rq.u.query_l2.BytesTransferred = CPU_TO_LE(pQuery->BytesTransferred);
    Rq.u.query_l2.BytesNeeded = CPU_TO_LE(pQuery->BytesNeeded);


    TRC_OUT(GR_MGT, LV_FCTPUB1, "send msg to FW - getset_information");

    Ret = l2_send_receive_synch(cp, (L2ETH_UINT8 *) & Rq, sendLen, &expRLen);      /* synchronous message */
    if(Ret != L2ETH_OK) {
        TRC_OUT01(GR_INIT, LV_ERR,
            " l2_getset_information send_receive_synch Ret = 0x%x", Ret);
        return Ret;
    }


    /* check set/ get information call */
    if(SET_INFO == getsettype) {        /* set information call */
        if(Rq.l2_header.opcode != SCO_L2ETH_SET_INFO_ACK) {
            Ret = L2ETH_ERR_INTERNAL;
            TRC_OUT01(GR_INIT, LV_ERR,
                " l2_getset_information received unknown opcode = 0x%x", Rq.l2_header.opcode);
            return Ret;
        }
    } else {                    /* get information call */
        if(Rq.l2_header.opcode != SCO_L2ETH_GET_INFO_ACK) {
            Ret = L2ETH_ERR_INTERNAL;
            TRC_OUT01(GR_INIT, LV_ERR,
                " l2_getset_information received unknown opcode = 0x%x", Rq.l2_header.opcode);
            return Ret;
        }
    }

    Ret = Rq.l2_header.resp_ret;

    /* copy query data back */
    pQuery->Oid = (L2ETH_OID)LE_TO_CPU(Rq.u.query_l2.Oid);
    pQuery->BufferLength = LE_TO_CPU(Rq.u.query_l2.BufferLength);
    pQuery->BytesTransferred = LE_TO_CPU(Rq.u.query_l2.BytesTransferred);
    pQuery->BytesNeeded = LE_TO_CPU(Rq.u.query_l2.BytesNeeded);

    /* get dma related offset */
    offset =
        (L2ETH_UINT32) LE_TO_CPU(Rq.u.query_l2.BufferOffset) -
        cp->m_pCpAdapter->m_dmaPhysicalPool;
    TRC_OUT02(GR_CHNL, LV_FCTPUB1, "query buff offset 0x%x, buff size %d", offset,
        pQuery->BufferLength);

    /* copy dmaQueryInfo to pQuery->pBuffer */
    memcpy(pQuery->pBuffer,
        (L2ETH_UINT8 *) (offset + cp->m_pCpAdapter->dma_base_ptr), pQuery->BufferLength);

    return Ret;
}


//------------------------------------------------------------
// public functions
//------------------------------------------------------------


//------------------------------------------------------------
// Initialize the CP
//------------------------------------------------------------
L2ETH_UINT32 l2_init_cp(l2eth_base * cp, L2ETH_UINT32 cpIndex)
{
    L2ETH_UINT32 Ret = L2ETH_OK;
    int k_errno;
    char tmp[36];
    struct t_register_app tmp_app;

    TRC_OUT01(GR_INIT, LV_FCTPUB1, "-> l2_init_cp %d", cpIndex);

    //create semaphore for synchronous operation in send channel
    if(L2ETH_SEM_CREATE(cp->m_semSendReceiveSynch)) {
        TRC_OUT(GR_INIT, LV_ERR, "DPR_SEM_CREATE(semSendReceiveSynch) failed");
        return L2ETH_ERR_INTERNAL;
    }
    //create adapter instance
    cp->m_pCpAdapter = (L2ETH_DPR_ADAPTER *) L2ETH_ZALLOC(sizeof (L2ETH_DPR_ADAPTER));
    if(!cp->m_pCpAdapter) {
        TRC_OUT(GR_INIT, LV_ERR, "create adapter failed");

        /* destroy semaphores */
        L2ETH_SEM_DESTROY(cp->m_semSendReceiveSynch);

        return L2ETH_ERR_NO_RESOURCE;
    }

    cp->m_pCpAdapter->l2eth_base_ptr = cp;
    cp->m_pCpAdapter->cp_index = cpIndex;

    L2ETH_MUTEX_CREATE_UNLOCKED(cp->m_pCpAdapter->synch_channel_mutex);

    BEGIN_BLOCK {
        snprintf(tmp, sizeof(tmp)-1, DPR_CONTROL_INTERFACE, DRIVER_IDX(cpIndex));

        //open control device for cp
        cp->m_pCpAdapter->fd_control = L2ETH_DRV_OPEN((const char *)tmp);
        if(!cp->m_pCpAdapter->fd_control) {
            TRC_OUT01(GR_INIT, LV_ERR, "openning %s failed", tmp);
            /* could not open the device file
               FW communication failed - may be driver is not be loaded. */
            Ret = L2ETH_ERR_NO_FW_COMMUNICATION;
            LEAVE_BLOCK;
        }

        tmp_app.flags = OAPP_APP_L2; /* only OAPP_APP_L2 application can bind L2_SEND/RECV_CHNL */

        //register application with driver
        if(!(k_errno = L2ETH_DRV_IOCTL(cp->m_pCpAdapter->fd_control,
            DPR_IOC_OAPP, &tmp_app, sizeof(tmp_app), sizeof(tmp_app)))) {
            cp->m_pCpAdapter->user_id = tmp_app.user_id;
            TRC_OUT01(GR_INIT, LV_INFO, "register application success, user_id 0x%x",
                cp->m_pCpAdapter->user_id);
        } else {
            TRC_OUT02(GR_INIT, LV_ERR, "ioctl register application for %s failed, error '%s'",
                tmp, L2ETH_STRERROR());
            if(-EMFILE == k_errno)
                Ret = L2ETH_ERR_MAX_REACHED;
            else
                Ret = L2ETH_ERR_NO_RESOURCE;
            LEAVE_BLOCK;
        }

        /* map initialize dma area */
        Ret = l2_init_dma_pool(cp, cp->m_pCpAdapter->fd_control, cpIndex);
        if(L2ETH_OK != Ret) {
            LEAVE_BLOCK;
        }

        // create channel device for send
        snprintf(tmp, sizeof(tmp)-1, DPR_L2_SEND_INTERFACE, DRIVER_IDX(cpIndex));

        Ret = l2_open_dpr_channel(tmp, cp->m_pCpAdapter, L2ETH_SEND,
                                   MAX_ETHERNET_SEND_FRAME_COUNT * L2ETH_SIZEOF_MSG_COMMAND);
        if(L2ETH_OK != Ret) {
            TRC_OUT01(GR_INIT, LV_ERR, "creating %s failed", tmp);
            LEAVE_BLOCK;
        }
        //create channel device for receive
        snprintf(tmp, sizeof(tmp)-1, DPR_L2_RECV_INTERFACE, DRIVER_IDX(cpIndex));

        Ret = l2_open_dpr_channel(tmp, cp->m_pCpAdapter, L2ETH_RECEIVE,
                                   MAX_ETHERNET_RECEIVE_FRAME_COUNT * L2ETH_SIZEOF_MSG_COMMAND);
        if(L2ETH_OK != Ret) {
            TRC_OUT01(GR_INIT, LV_ERR, "creating channel for %s failed", tmp);
            l2_close_dpr_channel(&(cp->m_pCpAdapter->chnls[L2ETH_SEND]));
            LEAVE_BLOCK;
        }
        TRC_OUT(GR_INIT, LV_INFO, "open_dpr_channel ok ");

    }
    END_BLOCK;
    if(L2ETH_OK != Ret) {
        TRC_OUT(GR_INIT, LV_FCTPUB1, "l2_init_cp error ");

        // close device
        if(cp->m_pCpAdapter->fd_control) {

            if(cp->m_pCpAdapter->user_id) {     /* DPR_IOC_OAPP performed. clean now */
                /* unregister app */
                if(L2ETH_DRV_IOCTL(cp->m_pCpAdapter->fd_control, DPR_IOC_CAPP,
                    &tmp_app, sizeof(tmp_app), 0)) {
                    TRC_OUT01(GR_INIT, LV_ERR, "unregister application failed, error '%s'",
                        L2ETH_STRERROR());
                }
            }

            // check if memory is mapped
            if(cp->m_pCpAdapter->dma_base_ptr) {
                //unmap memory
                if(0 != L2ETH_DRV_MUNMAP(cp->m_pCpAdapter->fd_control, cp->m_pCpAdapter->dma_base_ptr,
                    cp->m_pCpAdapter->m_dma_size, cp->m_pCpAdapter->user_id)) {
                    TRC_OUT(GR_INIT, LV_FCTPUB1, "l2_init_cp:  L2ETH_DRV_MUNMAP error");
                }
            }

            L2ETH_DRV_CLOSE(cp->m_pCpAdapter->fd_control);
        }

        /* destroy mutex */
        L2ETH_MUTEX_DESTROY(cp->m_pCpAdapter->synch_channel_mutex);

        /* destroy semaphores */
        L2ETH_SEM_DESTROY(cp->m_semSendReceiveSynch);

        //delete adapter instance
        L2ETH_FREE(cp->m_pCpAdapter);
        cp->m_pCpAdapter = NULL;

        return Ret;
    }

    TRC_OUT(GR_INIT, LV_FCTPUB1, "<- end l2_init_cp");
    return Ret;
}

//------------------------------------------------------------
//  l2_uninit_cp - clean-up operations
//------------------------------------------------------------
L2ETH_UINT32 l2_uninit_cp(l2eth_base * cp)
{
    struct t_register_app tmp_app;
    L2ETH_UINT32 Ret = L2ETH_OK;

    TRC_OUT(GR_INIT, LV_FCTPUB1, "-> l2_uninit_cp");

    /* system is stopping */
    cp->m_bEmergencyClose = L2ETH_TRUE;

    /* send a fake signal to stop */
    L2ETH_SEM_POST(cp->m_semSendReceiveSynch);

    /* close channels */
    l2_close_dpr_channel(&cp->m_pCpAdapter->chnls[L2ETH_SEND]);
    l2_close_dpr_channel(&cp->m_pCpAdapter->chnls[L2ETH_RECEIVE]);


    /* TRC_OUT01(GR_INIT, LV_FCTINT, "ioctl unregister application, user_id 0x%x",
       m_pCpAdapter->user_id); */
    memset(&tmp_app, 0, sizeof(tmp_app));
    tmp_app.user_id = cp->m_pCpAdapter->user_id;

    /* unregister application */
    if(L2ETH_DRV_IOCTL(cp->m_pCpAdapter->fd_control, DPR_IOC_CAPP,
        &tmp_app, sizeof(tmp_app), 0)) {
        TRC_OUT01(GR_INIT, LV_ERR, "ioctl unregister application failed, error '%s'",
            L2ETH_STRERROR());
    }


    TRC_OUT02(GR_INIT, LV_INFO, " try munmap dma_base_ptr=%p, size=%x",
        cp->m_pCpAdapter->dma_base_ptr, cp->m_pCpAdapter->m_dma_size);
    /* DPR_MUNMAP memory */
    if(0 != L2ETH_DRV_MUNMAP(cp->m_pCpAdapter->fd_control, cp->m_pCpAdapter->dma_base_ptr,
        cp->m_pCpAdapter->m_dma_size, cp->m_pCpAdapter->user_id)) {
        TRC_OUT(GR_INIT, LV_FCTPUB1, "l2_uninit_cp:  L2ETH_DRV_MUNMAP error");
    }


    if(cp->m_hRecvBuff) {
        if(L2ETH_BUF_SUCCESS != l2_buf_remove(&cp->m_hRecvBuff)) {
            TRC_OUT(GR_INIT, LV_ERR, "ERROR free buffer m_hRecvBuff ");
            Ret = L2ETH_ERR_INTERNAL;
        }
    }

    if(cp->m_hSendBuff) {
        if(L2ETH_BUF_SUCCESS != l2_buf_remove(&cp->m_hSendBuff)) {
            TRC_OUT(GR_INIT, LV_ERR, "ERROR free buffer m_hSendBuff ");
            Ret = L2ETH_ERR_INTERNAL;
        }
    }


    //close device
    L2ETH_DRV_CLOSE(cp->m_pCpAdapter->fd_control);

    //destroy semaphores
    L2ETH_SEM_DESTROY(cp->m_semSendReceiveSynch);
    L2ETH_MUTEX_DESTROY(cp->m_pCpAdapter->synch_channel_mutex);

    //delete adpter instance
    L2ETH_FREE(cp->m_pCpAdapter);
    cp->m_pCpAdapter = NULL;

    TRC_OUT(GR_INIT, LV_FCTPUB1, "<- end l2_uninit_cp");

    return Ret;
}

//------------------------------------------------------------
//  l2_send_receive_packet -
//         send 'packet to l2 agent thru receive ack channel
//------------------------------------------------------------
L2ETH_UINT32 l2_send_receive_packet(l2eth_base * cp, L2ETH_PACKET * pPacket)
{
    L2ETH_UINT32 offset, Ret = L2ETH_OK;
    L2ETH_MSG_COMMAND RecvPacketRq;
    L2ETH_UINT16 sendLen;

    sendLen = sizeof (L2ETH_MSG_HEADER) + sizeof (rq_packet_l2);

    if(!pPacket) {
        TRC_OUT(GR_INIT, LV_ERR, "ERROR Packet not allocated");
        return L2ETH_ERR_PRM_PKT;
    }

    /* set FW as receive buffer owner */
    l2_addref_rbuff_ownership(cp, (L2ETH_UINT8 *) pPacket->pBuffer, L2ETH_PACKET_OWNER_FW);

    offset = (L2ETH_UINT32)(pPacket->pBuffer - cp->m_pCpAdapter->dma_base_ptr);

    /*TRC_OUT02(GR_INIT, LV_ERR, "send_receive_packet packet offset 0x%x,  dma_base_ptr 0x%x  ", offset,
       (L2ETH_INT32)(cp->m_pCpAdapter->dma_base_ptr)); */

    RecvPacketRq.l2_header.blk_len = CPU_TO_LE16(sendLen);
    RecvPacketRq.l2_header.resp_ret = L2ETH_OK;
    RecvPacketRq.l2_header.opcode = SCO_L2ETH_RECEIVE_ACK;
    RecvPacketRq.l2_header.handle = CPU_TO_LE(cp->m_context_handle);

    // initialize receive packet
    RecvPacketRq.u.receive_packet.DataLength = CPU_TO_LE(MAX_ETHERNET_FRAME_SIZE);      //CPU_TO_LE(pPacket->DataLength);
    RecvPacketRq.u.receive_packet.BufferOffset =
        (L2ETH_UINT32) CPU_TO_LE(cp->m_pCpAdapter->m_dmaPhysicalPool + offset);
    RecvPacketRq.u.receive_packet.Context = CPU_TO_LE(pPacket->Context);        //CPU_TO_LE(0);

    Ret = l2_channel_write(cp, L2ETH_RECEIVE, (L2ETH_UINT8 *) & RecvPacketRq, sendLen);
    if(L2ETH_OK != Ret) {
        TRC_OUT01(GR_CHNL, LV_ERR, "fail to send %d bytes", sendLen);
    }

    return Ret;
}


//------------------------------------------------------------
//  l2_send_all_receive_packets -
//                      send 'N' packets to l2 agent thru receive ack channel
//------------------------------------------------------------
L2ETH_UINT32 l2_send_all_receive_packets(l2eth_base * cp, L2ETH_UINT32 reqCount)
{
    L2ETH_UINT32 i, Ret = L2ETH_OK;
    L2ETH_PACKET *pPkt;

    TRC_OUT(GR_INIT, LV_FCTPUB1, "-> l2_send_all_receive_packets");

    if(!cp->m_hRecvBuff) {
        TRC_OUT(GR_INIT, LV_ERR, "ERROR m_hRecvBuff memory pool not set");
        return L2ETH_ERR_INTERNAL;
    }

    /* try to get free buffers
       requesing assuming that user does not hold any packets */
    for(i = 0; i < reqCount; i++) {

        if(L2ETH_BUF_SUCCESS != l2_buf_get_next(cp->m_hRecvBuff, &pPkt)) {
            /* user is holding some receive packets...
               that packets will be send to FW when user calls return packet */
            TRC_OUT(GR_INIT, LV_INFO, "user is holding few packets");
            break;
        }

        Ret = l2_send_receive_packet(cp, pPkt);

        if(L2ETH_OK != Ret) {   /* error in sending packet to FW. break now */
            break;              //for loop
        }
    }

    TRC_OUT(GR_MGT, LV_FCTPUB1, "<- l2_send_all_receive_packets done");

    return Ret;
}

//------------------------------------------------------------
//  l2_set_emergency_close -
//                      set set_emergency_close flag
//------------------------------------------------------------
L2ETH_VOID l2_set_emergency_close(l2eth_base * cp, L2ETH_BOOL EmergencyClose)
{
    cp->m_bEmergencyClose = EmergencyClose;
}

//------------------------------------------------------------
//  l2_proc_send_cnf - send confirmation callback
//------------------------------------------------------------
L2ETH_UINT32 l2_proc_send_cnf(l2eth_base * cp)
{
    L2ETH_UINT32 Handle, sendLength, Length, offset;
    L2ETH_MSG_COMMAND Rq;
    L2ETH_PACKET *pPkt = NULL;

    TRC_OUT(GR_CHNL, LV_FCTPUB1, "-> callback  l2_proc_send_cnf");
//    Length = sizeof (L2ETH_MSG_COMMAND);
    Length = sizeof (L2ETH_MSG_HEADER) + sizeof (rq_packet_l2);

    sendLength = Length;
    l2_get_message(cp, (L2ETH_UINT8 *) & Rq, L2ETH_SEND, &Length);
    //if(Length > sizeof(Rq)) {
    if(Length > sendLength) {
        TRC_OUT03(GR_STATE, LV_ERR,
            " l2_proc_send_cnf max %Zu, expected %d, got unexpected length %d",
            sizeof(Rq), sendLength, Length);

        L2ETH_DBG_ASSERT(Length < sizeof (L2ETH_MSG_COMMAND));
    }

    Handle = l2_get_handle(cp);

    offset =
        (L2ETH_UINT32) LE_TO_CPU(Rq.u.send_acks_l2.BufferOffset) -
        cp->m_pCpAdapter->m_dmaPhysicalPool;
    TRC_OUT02(GR_CHNL, LV_FCTPUB1, "packet offset 0x%x, pkr size %d", offset,
        LE_TO_CPU(Rq.u.send_acks_l2.DataLength));

    l2_buf_get_packet(cp->m_hSendBuff,
        (L2ETH_UINT8 *) (offset + cp->m_pCpAdapter->dma_base_ptr), &pPkt);

    pPkt->DataLength = LE_TO_CPU((Rq.u.send_acks_l2).DataLength);
    pPkt->Context = LE_TO_CPU((Rq.u.send_acks_l2).Context);
    pPkt->pBuffer = (L2ETH_UINT8 *) (offset + cp->m_pCpAdapter->dma_base_ptr);

    TRC_OUT03(GR_CHNL, LV_FCTPUB1,
        "call user Cbf_Send with ApplHandle=0x%x, pPkt=%p, pBuff=%p", Handle, pPkt,
        pPkt->pBuffer);

    /* if mode is online call callback function */
    if(L2ETH_ONLINE == cp->m_l2_mode) {
        /* call user callback function */
        cp->m_send_cbf(Handle, pPkt, Rq.l2_header.resp_ret);
    }

    TRC_OUT(GR_MGT, LV_FCTPUB1, "<-  l2_proc_send_cnf done");

    return L2ETH_OK;
}

//------------------------------------------------------------
//  l2_proc_receive_packet - receive indication handler
//------------------------------------------------------------
L2ETH_UINT32 l2_proc_receive_packet(l2eth_base * cp)
{
    L2ETH_UINT32 Ret, Handle, Length, offset, CbfRet;
    L2ETH_MSG_COMMAND Rq;
    L2ETH_PACKET *pPkt = NULL;

    TRC_OUT(GR_CHNL, LV_FCTPUB1, "-> callback  l2_proc_receive_packet");

    Length = sizeof (L2ETH_MSG_HEADER) + sizeof (rq_packet_l2);

    l2_get_message(cp, (L2ETH_UINT8 *) & Rq, L2ETH_RECEIVE, &Length);
    if(Length < sizeof (L2ETH_MSG_COMMAND)) {
        TRC_OUT01(GR_STATE, LV_ERR,
            " l2_proc_receive_packet  got unexpected length %d", Length);
    }

    Handle = l2_get_handle(cp);

    offset = (L2ETH_UINT32)LE_TO_CPU(Rq.u.receive_packet.BufferOffset) -
        cp->m_pCpAdapter->m_dmaPhysicalPool;

    TRC_OUT01(GR_CHNL, LV_FCTPUB1, "packet offset 0x%x", offset);

    l2_buf_get_packet(cp->m_hRecvBuff,
        (L2ETH_UINT8 *) (offset + cp->m_pCpAdapter->dma_base_ptr), &pPkt);

    /* change address to virtual address */
    pPkt->DataLength = LE_TO_CPU((Rq.u.receive_packet).DataLength);
    pPkt->Context = LE_TO_CPU((Rq.u.receive_packet).Context);
    pPkt->pBuffer = (L2ETH_UINT8 *) (offset + cp->m_pCpAdapter->dma_base_ptr);

    Ret = l2_validate_packet_buffer(cp, L2ETH_DMA_RECEIVE_BUFFER, pPkt);
    if(L2ETH_OK != Ret) {
        TRC_OUT01(GR_CHNL, LV_ERR,
            " receive buffer offset from FW is invalid.. Packet is lost %p",
            pPkt->pBuffer);
        return Ret;
    }

    if((L2ETH_ERR_CANCEL_RQB == Rq.l2_header.resp_ret) || (L2ETH_OFFLINE == cp->m_l2_mode)) {
        /* current mode is offline, user callback is not required.
           give the buffer back to the buffer pool as free buffer. */
        l2_return_packet(cp, pPkt, L2ETH_PACKET_OWNER_FW);
    } else {
        /* set HOST user as receive buffer owner */
        l2_addref_rbuff_ownership(cp, (L2ETH_UINT8 *) pPkt->pBuffer, L2ETH_PACKET_OWNER_USER);

#ifdef L2ETH_LLDP_CHECK
        /* reject LLDP frames */
        if(L2ETH_TRUE == isLLDP_frame((L2ETH_UINT8 *) pPkt->pBuffer, pPkt->DataLength)) {
            TRC_OUT01(GR_CHNL, LV_ERR, " reject LLDP frame %p", pPkt->pBuffer);
            l2_return_packet(cp, pPkt, L2ETH_PACKET_OWNER_USER);
            return L2ETH_ERR_LLDP_FRAME;
        }
#endif
        TRC_OUT03(GR_CHNL, LV_FCTPUB1,
            "call user ReceiveCbf with Handle=0x%x, pPkt=%p, pBuff=%p", Handle, pPkt,
            pPkt->pBuffer);

        /* call user callback function
           if user returns 0 that means the packet is no more needed.
           And user not going to call return_packet */
        CbfRet = cp->m_recv_cbf(Handle, pPkt);

        if(!CbfRet)
            l2_return_packet(cp, pPkt, L2ETH_PACKET_OWNER_USER);
    }

    TRC_OUT(GR_MGT, LV_FCTPUB1, "<-  l2_proc_receive_packet done");
    return L2ETH_OK;
}


//------------------------------------------------------------
//  l2_proc_mode_cnf - mode change callback handler
//------------------------------------------------------------
L2ETH_UINT32 l2_proc_mode_cnf(l2eth_base * cp)
{
    L2ETH_UINT32 sendLength, Length, Handle;
    L2ETH_MSG_COMMAND Rq;
    L2ETH_UINT32 Ret = L2ETH_OK;

    TRC_OUT(GR_CHNL, LV_FCTPUB1, "-> callback  l2_proc_mode_cnf");

    Length = sizeof (L2ETH_MSG_COMMAND);
    Handle = l2_get_handle(cp);

    BEGIN_BLOCK {
        sendLength = Length;
        l2_get_message(cp, (L2ETH_UINT8 *) & Rq, L2ETH_RECEIVE, &Length);
        if(Length > sendLength) {
            TRC_OUT02(GR_STATE, LV_ERR,
                " l2_proc_mode_cnf expected max %d, got unexpected length %d",
                sizeof (Rq), Length);
            L2ETH_DBG_ASSERT(Length < sizeof (L2ETH_MSG_COMMAND));
        }

        /* check return error code  */
        if(L2ETH_OK != Rq.l2_header.resp_ret) {
            TRC_OUT01(GR_STATE, LV_ERR,
                " l2_proc_mode_cnf failed - firmware error %d", Length);
            Ret = Rq.l2_header.resp_ret;
            LEAVE_BLOCK;
        }

        /* check if mode is same */
        if(cp->m_l2_mode == (L2ETH_MODE)LE_TO_CPU(Rq.u.mode_l2.Mode)) {
            /* same mode */
            TRC_OUT(GR_INIT, LV_WARN, " l2_set_mode requested mode is same as current");
            LEAVE_BLOCK;
        } else { /* mode change success do ... */
            /* mode changing from offline to online */
            if(L2ETH_ONLINE == (L2ETH_MODE)LE_TO_CPU(Rq.u.mode_l2.Mode)) {
                /* mode changed to online */
                cp->m_l2_mode = (L2ETH_MODE)LE_TO_CPU(Rq.u.mode_l2.Mode);
                cp->m_context_handle = LE_TO_CPU(Rq.l2_header.handle);

                TRC_OUT01(GR_MGT, LV_FCTPUB1, "get_user_rbuff_ownership_count %d",
                    l2_get_user_rbuff_ownership_count(cp));

                TRC_OUT01(GR_MGT, LV_FCTPUB1,
                    " mode online - call send_all_receive_packets %d times !",
                    MAX_ETHERNET_RECEIVE_FRAME_COUNT);

                /* send receive request to FW - request n number of receive request to agent */
                Ret = l2_send_all_receive_packets(cp, MAX_ETHERNET_RECEIVE_FRAME_COUNT);

                if(L2ETH_OK != Ret) {
                    //Ret = L2ETH_ERR_NO_FW_COMMUNICATION;
                    TRC_OUT(GR_INIT, LV_ERR,
                        " l2_set_mode failed to send receive request to agent ");
                    LEAVE_BLOCK;
                }

            } else {
                /* mode changing from online to offline */
                cp->m_l2_mode = (L2ETH_MODE)LE_TO_CPU(Rq.u.mode_l2.Mode);
                TRC_OUT(GR_MGT, LV_FCTPUB1, "mode changed to offline");
            }
        }
    }
    END_BLOCK;
    TRC_OUT03(GR_CHNL, LV_FCTPUB1,
        "call user ModeCbf with Handle=0x%x, mode=0x%x, Ret=0x%x", Handle, cp->m_l2_mode,
        Ret);

    /* mode change request finished - reset flag */
    cp->m_bmodeChangeRequested = L2ETH_FALSE;

    /* call user callback function */
    cp->m_mode_cbf(Handle, cp->m_l2_mode, Ret);

    TRC_OUT(GR_MGT, LV_FCTPUB1, "<-  l2_proc_mode_cnf done");

    return Ret;
}

//------------------------------------------------------------
//  l2_proc_status_ind - status change indication handler
//------------------------------------------------------------
L2ETH_UINT32 l2_proc_status_ind(l2eth_base * cp)
{
    L2ETH_UINT32 Handle, sendLength, Length;
    L2ETH_MSG_COMMAND Rq;
    L2ETH_UINT32 Ret = L2ETH_OK;

    TRC_OUT(GR_CHNL, LV_FCTPUB1, "-> callback  l2_proc_status_ind");

    Length = sizeof (L2ETH_MSG_COMMAND);

    //get msg from channel
    sendLength = Length;
    Handle = l2_get_handle(cp);

    l2_get_message(cp, (L2ETH_UINT8 *) & Rq, L2ETH_RECEIVE, &Length);
    if(Length > sendLength) {
        TRC_OUT02(GR_STATE, LV_ERR,
            " l2_proc_status_ind expected max %d, got unexpected length %d",
            sizeof (Rq), Length);
        //return L2ETH_ERR_NO_RESOURCE;
        L2ETH_DBG_ASSERT(Length < sizeof (L2ETH_MSG_COMMAND));
    }

    TRC_OUT02(GR_CHNL, LV_FCTPUB1,
        "call user StatusCbf with Handle=0x%x, oid=0x%x", Handle,
        (L2ETH_OID) LE_TO_CPU(Rq.u.status_l2.Oid));

    /*if mode is online call callback function */
    if(L2ETH_ONLINE == cp->m_l2_mode) {
        /* call user callback function */
        cp->m_status_cbf(Handle, (L2ETH_OID) LE_TO_CPU(Rq.u.status_l2.Oid));
    }

    TRC_OUT(GR_MGT, LV_FCTPUB1, "<-  l2_proc_status_ind done");

    return Ret;
}

//------------------------------------------------------------
//  l2_channel_write - send packet/message via DPR channel
//------------------------------------------------------------
L2ETH_UINT32 l2_channel_write(l2eth_base * cp, L2ETH_CHANNELS Chnl, L2ETH_UINT8 * pRq,
    L2ETH_UINT32 sendLen)
{
    size_t wcount;
    TRC_OUT02(GR_STATE, LV_FCTCLBF, "l2_channel_write %d Send sendLen %d", Chnl, sendLen);

    /*TRC_OUT02(GR_CHNL, LV_DATA, "Send sends %d bytes data in channel %d:", sendLen, Chnl);
       TRC_OUTD(GR_CHNL, LV_DATA, (L2ETH_UINT8 *)pRq, sendLen); */

    wcount = L2ETH_DRV_WRITE(cp->m_pCpAdapter->chnls[Chnl].file, pRq, sendLen );

    if(wcount != sendLen) {
        TRC_OUT03(GR_CHNL, LV_ERR,
            "writing to device file failed, written %Zu bytes from %d, error '%s'",
            wcount, sendLen, L2ETH_STRERROR());
        /*
           buffer count and size is always less than dpr channel size.
           if write failed means there is some problem in FW that
           it is unable to read the messages in the channel
         */
        return L2ETH_ERR_NO_FW_COMMUNICATION;
    }

    return L2ETH_OK;
}

//------------------------------------------------------------
//  l2_send_receive_synch - send packet to DPR channel and wait for ack
//------------------------------------------------------------
L2ETH_UINT32 l2_send_receive_synch(l2eth_base * cp, L2ETH_UINT8 * pRq, L2ETH_UINT32 sendLen,
    L2ETH_UINT32 * receiveLen)
{
    size_t wcount;
    L2ETH_UINT32 expectedReceiveLen = *receiveLen;
    L2ETH_UINT8 *tmp;
    L2ETH_UINT32 tmpLength, Ret = L2ETH_OK;

    // get mutex
    L2ETH_MUTEX_LOCK(cp->m_pCpAdapter->synch_channel_mutex);

    /*TRC_OUT02(GR_STATE, LV_FCTCLBF,
       " send_receive_synch sendLen %d, receiveLen %d", sendLen, *receiveLen);
       TRC_OUTD(GR_CHNL, LV_DATA, (L2ETH_UINT8 *)pRq, sendLen); */


    //write info to the channel
    wcount = L2ETH_DRV_WRITE(cp->m_pCpAdapter->chnls[L2ETH_SEND].file, pRq, sendLen);

    if(wcount != sendLen) {
        TRC_OUT03(GR_CHNL, LV_ERR,
            "SynchChannelWrite failed, written %Zu bytes from %d, error '%s'",
            wcount, sendLen, L2ETH_STRERROR());
        Ret = L2ETH_ERR_NO_FW_COMMUNICATION;
        L2ETH_MUTEX_UNLOCK(cp->m_pCpAdapter->synch_channel_mutex);
        return Ret;
    }

    TRC_OUT(GR_STATE, LV_FCTCLBF, " start L2ETH_SEM_WAIT semSendReceiveSynch");

    /* wait for Receive msg */
#ifdef __KERNEL__
    if(!cp->m_bEmergencyClose)
        L2ETH_SEM_WAIT(cp->m_semSendReceiveSynch);
#else
    while((!cp->m_bEmergencyClose) && (L2ETH_SEM_WAIT(cp->m_semSendReceiveSynch)));       /* keep semicolon separately for more clarity */
#endif

    TRC_OUT(GR_STATE, LV_FCTCLBF, "end L2ETH_SEM_WAIT semSendReceiveSynch");


    if(cp->m_bEmergencyClose) {
        TRC_OUT(GR_STATE, LV_FCTCLBF, "EmergencyClose  requested");
        Ret = L2ETH_ERR_INTERNAL;
        L2ETH_MUTEX_UNLOCK(cp->m_pCpAdapter->synch_channel_mutex);
        return Ret;
    }

    tmpLength = *receiveLen;
    tmp = l2_get_message(cp, pRq, L2ETH_SEND, &tmpLength);
    if(!tmp) {
        TRC_OUT(GR_CHNL, LV_ERR, "l2_get_message failed");
        Ret = L2ETH_ERR_INTERNAL;
        L2ETH_MUTEX_UNLOCK(cp->m_pCpAdapter->synch_channel_mutex);
        return Ret;
    }

    /* verify the msg length */
    if((expectedReceiveLen != 0) && expectedReceiveLen != tmpLength) {
        TRC_OUT02(GR_CHNL, LV_ERR,
            "unexpected length %d, expected length %d", tmpLength, expectedReceiveLen);
        Ret = L2ETH_ERR_INTERNAL;
        L2ETH_MUTEX_UNLOCK(cp->m_pCpAdapter->synch_channel_mutex);
        return Ret;
    }

    *receiveLen = tmpLength;

    L2ETH_MUTEX_UNLOCK(cp->m_pCpAdapter->synch_channel_mutex);

    return Ret;
}

//------------------------------------------------------------
//  l2_get_message - read dpr message
//------------------------------------------------------------
L2ETH_UINT8 *l2_get_message(l2eth_base * cp, L2ETH_UINT8 * buf, L2ETH_CHANNELS Chnl,
    L2ETH_UINT32 * msgLen)
{
    L2ETH_CHANNEL_DATA *my_channel;

    my_channel = &(cp->m_pCpAdapter->chnls[Chnl]);
#if 0
    TRC_OUT02(GR_CHNL, LV_DATA, "l2_get_message receives %d bytes data in channel %d:",
       my_channel->user_pool_length_used, Chnl);
    TRC_OUTD(GR_CHNL, LV_DATA,
       (L2ETH_UINT8 *)my_channel->user_pool_ptr, my_channel->user_pool_length_used);
#endif
    if(*msgLen >= my_channel->user_pool_length_used) {
        memcpy(buf, my_channel->user_pool_ptr, my_channel->user_pool_length_used);
    }
    *msgLen = my_channel->user_pool_length_used;
    return buf;
}

//------------------------------------------------------------
//  l2_get_dma_queryInfo_buffer - status callback handler
//------------------------------------------------------------
L2ETH_UINT32 *l2_get_dma_queryInfo_buffer(l2eth_base * cp)
{
    /* get dma memmory for query info set/get */
    if(cp->m_pCpAdapter->dma_base_ptr) {
        /* dma memory pool is divided to 3 parts
           |  receive pool    |   send pool   | query pool  |
           there is only 1 query dma pool for set_info and get_info
           this pool is synchronized by by the user interface implementation */
        return (L2ETH_UINT32 *) ((L2ETH_UINT8 *) cp->m_pCpAdapter->dma_base_ptr +
            (cp->m_dma_receive_frame_count * MAX_ETHERNET_FRAME_SIZE) +
            (cp->m_dma_send_frame_count * MAX_ETHERNET_FRAME_SIZE));
    }

    return NULL;

}

//  l2_open_dpr_channel - open dpr channel
//------------------------------------------------------------
L2ETH_UINT32 l2_open_dpr_channel(const char * pFileName, L2ETH_DPR_ADAPTER * adapter, 
                                  L2ETH_CHANNELS chnl, L2ETH_UINT32 min_read_length)
{

    L2ETH_DRV_HANDLE file = NULL;
    struct t_read_pool usr_pool;
    L2ETH_CHANNEL_DATA *channel = &adapter->chnls[chnl];
    int k_errno;


    TRC_OUT01(GR_INIT, LV_FCTPUB1, "-> l2_open_dpr_channel %s", pFileName);

    if(NULL != channel->file)
        return L2ETH_ERR_NO_RESOURCE;

    file = L2ETH_DRV_OPEN((const char *)pFileName);
    if(!file) {
        TRC_OUT01(GR_INIT, LV_ERR, "openning %s failed", pFileName);
        return L2ETH_ERR_NO_RESOURCE;
    }

    usr_pool.user_id = adapter->user_id;
    if((k_errno = L2ETH_DRV_IOCTL(file, DPR_IOC_BIND, &usr_pool, sizeof(usr_pool), sizeof(usr_pool))) < 0) {
        TRC_OUT02(GR_INIT, LV_ERR, "register memory pool for %s failed, error '%s'",
            pFileName, L2ETH_STRERROR());

        L2ETH_DRV_CLOSE(file);
        if(EMFILE == k_errno)
            return L2ETH_ERR_MAX_REACHED;
        else
            return L2ETH_ERR_NO_RESOURCE;
    }

    channel->user_pool_length = usr_pool.read_length;
    if(channel->user_pool_length < min_read_length) {
      TRC_OUT02(GR_INIT, LV_ERR, "driver provided memory pool length(%d) is smaller than min. necessary length(%d)", channel->user_pool_length, min_read_length);

      /* please adapt l2eth_config.h or dprlib.h for send channel size to match MAX_ETHERNET_SEND_FRAME_COUNT */

      L2ETH_DRV_IOCTL(file, DPR_IOC_UNBIND, &usr_pool, sizeof(usr_pool), sizeof(usr_pool));
      L2ETH_DRV_CLOSE(file);
      return L2ETH_ERR_NO_RESOURCE;
    }

    channel->user_pool_ptr = (L2ETH_UINT8 *) L2ETH_VMALLOC(channel->user_pool_length);
    if(!channel->user_pool_ptr) {
        TRC_OUT02(GR_INIT, LV_ERR, "alloc memory pool for %s len %d failed", pFileName, channel->user_pool_length);
        L2ETH_DRV_IOCTL(file, DPR_IOC_UNBIND, &usr_pool, sizeof(usr_pool), sizeof(usr_pool));
        L2ETH_DRV_CLOSE(file);
        return L2ETH_ERR_NO_RESOURCE;
    }

    channel->channel_number = (L2ETH_UINT32) chnl;
    channel->file = file;
    channel->parent = (L2ETH_UINT8 *) adapter;
    channel->stop_thread = 0;

    /* thread for processing the callback messages */
    if(!L2ETH_THREAD_CREATE(&channel->th_reader, "", procChannelRead, channel)) {
        TRC_OUT01(GR_MGT, LV_ERR, "Create thread for %s failed", pFileName);

        L2ETH_DRV_IOCTL(channel->file, DPR_IOC_UNBIND, &usr_pool, sizeof(usr_pool), 0);

        channel->stop_thread = 1;
        channel->parent = NULL;
        if(channel->user_pool_ptr)
            L2ETH_FREE(channel->user_pool_ptr);
        channel->user_pool_ptr = NULL;
        channel->user_pool_length = 0;
        L2ETH_DRV_CLOSE(channel->file);
        channel->file = NULL;

        return L2ETH_ERR_NO_RESOURCE;
    }
    /*TRC_OUT02(GR_INIT, LV_FCTPUB1, "Create thread for %s ok, handle %x",
       pFileName, channel->th_reader); */

    TRC_OUT01(GR_INIT, LV_FCTPUB1, "<- l2_open_dpr_channel %s", pFileName);
    return L2ETH_OK;

}


//------------------------------------------------------------
//  l2_close_dpr_channel - close dpr channel
//------------------------------------------------------------
L2ETH_UINT32 l2_close_dpr_channel(L2ETH_CHANNEL_DATA * channel)
{
    struct t_read_pool usr_pool;
    L2ETH_DPR_ADAPTER *adapter = (L2ETH_DPR_ADAPTER *) channel->parent;

    TRC_OUT(GR_INIT, LV_FCTPUB1, "-> l2_close_dpr_channel");


    usr_pool.user_id = adapter->user_id;
    L2ETH_DRV_IOCTL(channel->file, DPR_IOC_UNBIND, &usr_pool, sizeof(usr_pool), 0);

    /*  call UserThreadJoin as soon as channel->stop_thread is set */
    channel->stop_thread = 1;

    if(channel->th_reader) {
#ifdef __KERNEL__
        wait_for_completion(&on_exit);
#else
        //TRC_OUT01(GR_INIT, LV_FCTPUB1, "Stop thread, handle %x", channel->th_reader);
        L2ETH_THREAD_JOIN(channel->th_reader);
#endif
        channel->th_reader = 0;
    }

    channel->parent = NULL;
    if(channel->user_pool_ptr)
        L2ETH_VFREE(channel->user_pool_ptr);
    channel->user_pool_ptr = NULL;
    channel->user_pool_length = 0;
    L2ETH_DRV_CLOSE(channel->file);
    channel->file = NULL;

    TRC_OUT(GR_INIT, LV_FCTPUB1, "<- l2_close_dpr_channel");

    return L2ETH_OK;
}

/*===========================================================================
* FUNCTION : procChannelRead - callback processing threads
*==========================================================================*/
L2ETH_THREAD_RETURN_TYPE DPR_THREAD_DECL procChannelRead(L2ETH_VOID * arg)
{
    L2ETH_CHANNEL_DATA *my_channel = (L2ETH_CHANNEL_DATA *) arg;
    L2ETH_DPR_ADAPTER *adapter = (L2ETH_DPR_ADAPTER *) my_channel->parent;
    l2eth_base *pThis = adapter->l2eth_base_ptr;
    L2ETH_INT32 rCount;

    L2ETH_MSG_HEADER *pHeader;


    TRC_OUT01(GR_CHNL, LV_FCTCLBF, "-> procChannelRead %d", my_channel->channel_number);
    /* loop till shutdown req */
    while(!my_channel->stop_thread) {
        rCount = L2ETH_DRV_READ(my_channel->file, my_channel->user_pool_ptr,
            my_channel->user_pool_length);

        if(rCount < 0) {
            TRC_OUT02(GR_CHNL, LV_FCTCLBF,
                "procChannelRead %d: read error rCount %d", my_channel->channel_number,
                rCount);
            continue;
        } else if(!rCount && L2ETH_DRV_ERROR(my_channel->file)) {
            TRC_OUT01(GR_CHNL, LV_FCTCLBF,
                "procChannelRead %d: read error, rCount 0, ferror()",
                my_channel->channel_number);
            continue;
        } else if((L2ETH_UINT32) rCount > my_channel->user_pool_length) {
            TRC_OUT02(GR_CHNL, LV_ERR,
                "procChannelRead %d: buffer is too small, need %d bytes",
                my_channel->channel_number, rCount);
            continue;
        }

        my_channel->user_pool_length_used = rCount;

        switch (my_channel->channel_number) {
        case L2ETH_SEND:
            pHeader = (L2ETH_MSG_HEADER *) my_channel->user_pool_ptr;
            switch (pHeader->opcode) {
            case SCO_L2ETH_OPEN:
            case SCO_L2ETH_CLOSE:
            case SCO_L2ETH_SET_INFO_ACK:
            case SCO_L2ETH_GET_INFO_ACK:
                //to start with send channel used for open /close
                L2ETH_SEM_POST(pThis->m_semSendReceiveSynch);
                break;

            case SCO_L2ETH_SEND_ACK:
                l2_proc_send_cnf(pThis);
                break;

            default:
                TRC_OUT01(GR_CHNL, LV_ERR,
                    " message with unknown opcode in send channel %d:",
                    pHeader->opcode);
                break;
            }
            break;

        case L2ETH_RECEIVE:
            pHeader = (L2ETH_MSG_HEADER *) my_channel->user_pool_ptr;
            switch (pHeader->opcode) {
            case SCO_L2ETH_RECEIVE:
                l2_proc_receive_packet(pThis);
                break;

            case SCO_L2ETH_SET_MODE_ACK:
                l2_proc_mode_cnf(pThis);
                break;

            case SCO_L2ETH_STATUS_ACK:
                l2_proc_status_ind(pThis);
                break;
            default:
                TRC_OUT01(GR_CHNL, LV_ERR,
                    " message with unknown opcode in receive channel %d:",
                    pHeader->opcode);
                break;
            }
            break;

        default:
            TRC_OUT01(GR_CHNL, LV_ERR,
                "procChannelRead %d: undefined channel", my_channel->channel_number);
        }
    }

    TRC_OUT01(GR_CHNL, LV_FCTCLBF, "<- procChannelRead %d", my_channel->channel_number);

    my_channel->th_reader = 0;

#ifdef __KERNEL__
    complete_and_exit(&on_exit, 0);
#endif

    L2ETH_THREAD_END();
}

L2ETH_UINT32 l2_get_card_timer(l2eth_base * cp)
{
    struct t_read_timer tmp_timer;
    int k_errno;

    TRC_OUT(GR_INIT, LV_INFO, " try DPR_IOCTL DPR_IOC_GET_CP_TIMER ");
    //  get the dma physical address
    if(!(k_errno = L2ETH_DRV_IOCTL(cp->m_pCpAdapter->fd_control, DPR_IOC_GET_CP_TIMER,
        &tmp_timer, 0, sizeof(tmp_timer)))) {
        TRC_OUT01(GR_INIT, LV_INFO, "get card success, Timer 0x%lx", tmp_timer.cp_timer);
    } else {
        TRC_OUT01(GR_INIT, LV_ERR, "ERROR %s ioctl get  card Timer", L2ETH_STRERROR());
        return 0;
    }
    TRC_OUT(GR_INIT, LV_INFO, " end USER_IOCTL DPR_IOC_GET_CP_TIMER ");
    return tmp_timer.cp_timer;
}

L2ETH_UINT32 l2_init_dma_pool(l2eth_base * cp, L2ETH_DRV_HANDLE file, L2ETH_UINT32 cpIndex)
{
    L2ETH_UINT32 Ret = L2ETH_OK;
    L2ETH_UINT32 hostDMAoffset;

    struct t_dma_address tmp_dma;
    L2ETH_BUFFER_DISCRIPTOR buf_desc;

    L2ETH_INT32 tempSize, balPackets, buffCount;
    int k_errno;

    tempSize = MAX_DMA_SIZE % DPR_GET_PAGE_SIZE;

    BEGIN_BLOCK {
        //  get the dma physical address and size
        memset(&tmp_dma, 0, sizeof(tmp_dma));
        if(!(k_errno = L2ETH_DRV_IOCTL(file, DPR_IOC_GET_L2_DMA_PHYS_ADDR,
            &tmp_dma, sizeof(tmp_dma), sizeof(tmp_dma)))) {
            cp->m_pCpAdapter->m_dmaPhysicalPool = DPR_PTR_TO_ULONG(tmp_dma.dma_address);
            cp->m_pCpAdapter->m_dma_size = tmp_dma.dma_size;
            TRC_OUT02(GR_INIT, LV_INFO, "get dma physical adds 0x%x, size 0x%x",
                cp->m_pCpAdapter->m_dmaPhysicalPool, cp->m_pCpAdapter->m_dma_size);
        } else {
            DPR_DRV_CLOSE(file);
            if(-EMFILE == k_errno)
                Ret = L2ETH_ERR_MAX_REACHED;
            else
                Ret = L2ETH_ERR_NO_RESOURCE;
            LEAVE_BLOCK;
        }

#ifdef __KERNEL__
        hostDMAoffset = DPR_PTR_TO_ULONG(tmp_dma.dma_address);
#else
        hostDMAoffset = MMAP_OFFSET_L2_DMA;
#endif
        TRC_OUT02(GR_INIT, LV_INFO, "hostDMAoffset 0x%x  size:0x%x ", hostDMAoffset,
            cp->m_pCpAdapter->m_dma_size);

        /* mmap - map dma memory */
        cp->m_pCpAdapter->dma_base_ptr =
            (L2ETH_UINT8 *) L2ETH_DRV_MMAP(file, hostDMAoffset, cp->m_pCpAdapter->m_dma_size, cp->m_pCpAdapter->user_id);

        if(!cp->m_pCpAdapter->dma_base_ptr) {
            TRC_OUT01(GR_INIT, LV_ERR, "ERROR mmap dma_base_ptr - %s", L2ETH_STRERROR());
            return L2ETH_ERR_NO_RESOURCE;

        } else {
            TRC_OUT02(GR_INIT, LV_INFO, " m_pCpAdapter->dma_base_ptr =%p ,size =%x",
                cp->m_pCpAdapter->dma_base_ptr, cp->m_pCpAdapter->m_dma_size);
        }


        /* check if we have enough dma memory for - send, receive and query buffer */

        /* get the balance packes aailable for send and receive without PRE_LOAD_BUF_SIZE */
        balPackets =
            ((L2ETH_UINT32) cp->m_pCpAdapter->m_dma_size / MAX_ETHERNET_FRAME_SIZE) -
            PRE_LOAD_BUF_SIZE;

        /* calculate balace buffer */
        tempSize =
            cp->m_pCpAdapter->m_dma_size - ((balPackets * MAX_ETHERNET_FRAME_SIZE) +
            (PRE_LOAD_BUF_SIZE * MAX_ETHERNET_FRAME_SIZE));
        //TRC_OUT03(GR_INIT, LV_INFO,
        //              "buffCount %d, tempSize %d, dma MAXval %d",buffCount , tempSize, MAX_DMA_QUERY_BUFFER_SIZE);

        /* check enough memory for query pool  */
        if(MAX_DMA_QUERY_BUFFER_SIZE > tempSize) {
            /* not enough memory reduce total count */
            balPackets--;

            tempSize =
                cp->m_pCpAdapter->m_dma_size - ((balPackets * MAX_ETHERNET_FRAME_SIZE) +
                (PRE_LOAD_BUF_SIZE * MAX_ETHERNET_FRAME_SIZE));
            //TRC_OUT03(GR_INIT, LV_INFO,
            //      "buffCount %d, tempSize %d, dma MAXval %d",buffCount , tempSize, MAX_DMA_QUERY_BUFFER_SIZE);
        }

        /*get possible buffer count for receive */
        buffCount = balPackets / 2;


        /* check enough size for send and receive buffers */
        if(1 > buffCount) {
            Ret = L2ETH_ERR_NO_RESOURCE;
            TRC_OUT(GR_INIT, LV_ERR, " not enough dma memory ");
            LEAVE_BLOCK;
        }
        //set m_hRecvBuff as dma base address
        buf_desc.base_address = (L2ETH_UINT8 *) cp->m_pCpAdapter->dma_base_ptr;
        buf_desc.buf_size = MAX_ETHERNET_FRAME_SIZE;

        cp->m_dma_receive_frame_count =
            (MAX_ETHERNET_RECEIVE_FRAME_COUNT_TOTAL >
            (PRE_LOAD_BUF_SIZE + buffCount)) ? (PRE_LOAD_BUF_SIZE +
            buffCount) : MAX_ETHERNET_RECEIVE_FRAME_COUNT_TOTAL;

        buf_desc.buf_cnt = cp->m_dma_receive_frame_count;

        buf_desc.def_fill = 0;  // fill with 0

        TRC_OUT02(GR_INIT, LV_INFO,
            "call l2_buf_init - m_hRecvBuff at base %p, count %d",
            buf_desc.base_address, buf_desc.buf_cnt);

        /* initialize buffer manager for receive buffer */
        if(L2ETH_BUF_SUCCESS != l2_buf_init(buf_desc, PRE_LOAD_BUF_SIZE,
                &(cp->m_hRecvBuff))) {
            TRC_OUT01(GR_INIT, LV_ERR, "ERROR allocating buffer mgmgt frames %d",
                buf_desc.buf_cnt);
            Ret = L2ETH_ERR_NO_RESOURCE;
            LEAVE_BLOCK;
        }

        /* set receive buffer ownerships  */
        l2_init_rbuff_ownership(cp);

        /* set m_hSendBuff pool at the end of receive pool area
           ie. dma base address + MAX_ETHERNET_FRAME_SIZE * MAX_ETHERNET_RECEIVE_FRAME_COUNT) */
        /*
           buf_desc.base_address = (L2ETH_UINT8 *) ((L2ETH_UINT8 *) cp->m_pCpAdapter->dma_base_ptr +
           MAX_DMA_RECEIVE_POOL_SIZE);
         */
        /* get send buffer start address */
        buf_desc.base_address = (L2ETH_UINT8 *) cp->m_pCpAdapter->dma_base_ptr +
            MAX_ETHERNET_FRAME_SIZE * cp->m_dma_receive_frame_count;

        /* if enough dma memory is available use config.h defined send frame count */
        cp->m_dma_send_frame_count =
            (MAX_ETHERNET_SEND_FRAME_COUNT >
            ((balPackets + PRE_LOAD_BUF_SIZE) -
                cp->m_dma_receive_frame_count)) ? ((balPackets + PRE_LOAD_BUF_SIZE) -
            cp->m_dma_receive_frame_count) : MAX_ETHERNET_SEND_FRAME_COUNT;

        buf_desc.buf_cnt = cp->m_dma_send_frame_count;

        TRC_OUT02(GR_INIT, LV_INFO,
            "call l2_buf_init - m_hSendBuff at base %p, count %d",
            buf_desc.base_address, buf_desc.buf_cnt);

        //initialize buffer manager for send buffer
        if(L2ETH_BUF_SUCCESS != l2_buf_init(buf_desc, 0, &cp->m_hSendBuff)) {
            TRC_OUT01(GR_INIT, LV_ERR, "ERROR allocating buffer mgmgt frames %d",
                buf_desc.buf_cnt);
            Ret = L2ETH_ERR_NO_RESOURCE;
            LEAVE_BLOCK;
        }

    }
    END_BLOCK;
    if(L2ETH_OK != Ret) {
        //un-map dma memory
        if(cp->m_pCpAdapter->dma_base_ptr) {

            TRC_OUT02(GR_INIT, LV_INFO, " try munmap dma_base_ptr =%p, size =%x",
                cp->m_pCpAdapter->dma_base_ptr, cp->m_pCpAdapter->m_dma_size);
            //unmap map
            if(0 != L2ETH_DRV_MUNMAP(cp->m_pCpAdapter->fd_control, cp->m_pCpAdapter->dma_base_ptr,
                    cp->m_pCpAdapter->m_dma_size, cp->m_pCpAdapter->user_id)) {
                TRC_OUT(GR_INIT, LV_FCTPUB1, "uninit_cp:  DPR_MUNMAP error");
            }

        }
    }

    return Ret;
}

/* mark preload buffers */
L2ETH_UINT32 l2_init_rbuff_ownership(l2eth_base * cp)
{
    L2ETH_INT16 pos, count = 0;

    /* mark preload buffers are with FW ( value = -1) */
    for(pos = 0; pos < PRE_LOAD_BUF_SIZE; pos++) {
        cp->m_user_receive_buffer_ref_count[pos] = -1;
        count++;
    }

    /* mark remaining buffers are with L2ETH_LIB ( value = 0) */
    for(pos = PRE_LOAD_BUF_SIZE; pos < MAX_ETHERNET_RECEIVE_FRAME_COUNT_TOTAL; pos++) {
        cp->m_user_receive_buffer_ref_count[pos] = 0;
        count++;
    }

    /* set start address  */
    //user_receive_buffer_base = (L2ETH_UINT32)pktBuff; //start address

    return count;
}

/* change ownership of buffer */
L2ETH_INT32 l2_addref_rbuff_ownership(l2eth_base * cp,
    L2ETH_UINT8 * pktBuff,      /* buffer pointer */
    L2ETH_PACKET_OWNER owner)   /* buffer owner */
{
    L2ETH_INT16 pos;


    pos = (L2ETH_INT16) ((pktBuff - cp->m_pCpAdapter->dma_base_ptr)
        / MAX_ETHERNET_FRAME_SIZE);

    /* check pos is less than array max limit */
    if((pos < 0) || (pos > MAX_ETHERNET_RECEIVE_FRAME_COUNT_TOTAL)) {
        TRC_OUT01(GR_INIT, LV_ERR, "ERROR buffer pointer %d", L2ETH_ERR_PRM_BUF);
        return -1;
    }

    if(L2ETH_PACKET_OWNER_FW == owner) {
        cp->m_user_receive_buffer_ref_count[pos] = -1;
    } else if(L2ETH_PACKET_OWNER_USER == owner) {
        cp->m_user_receive_buffer_ref_count[pos] = 1;
    } else {                    /* L2ETH_PACKET_OWNER_L2ETH_LIB */
        cp->m_user_receive_buffer_ref_count[pos] = 0;
    }

    TRC_OUT02(GR_INIT, LV_FCTPUB1, "a buffer %p , ref_count %d",
        pktBuff, cp->m_user_receive_buffer_ref_count[pos]);

    return 0;
}

/* release ownership of buffer */
L2ETH_INT32 l2_release_rbuff_ownership(l2eth_base * cp,
    L2ETH_UINT8 * pktBuff,     /* packet pointer */
    L2ETH_PACKET_OWNER owner)  /* packet owner */
{
    L2ETH_INT16 pos, ref_cnt;

    pos = (L2ETH_INT16) ((pktBuff - cp->m_pCpAdapter->dma_base_ptr)
                / MAX_ETHERNET_FRAME_SIZE);

    /* check pos is less than array max limit */
    if((0 > pos) || (MAX_ETHERNET_RECEIVE_FRAME_COUNT_TOTAL < pos)) {
        TRC_OUT01(GR_INIT, LV_ERR, "ERROR buffer pointer %d", L2ETH_ERR_PRM_BUF);
        return -1;
    }

    if(L2ETH_PACKET_OWNER_USER == owner) {
        ref_cnt = cp->m_user_receive_buffer_ref_count[pos];
        TRC_OUT02(GR_INIT, LV_FCTPUB1,
            "l2_release_rbuff_ownership buffer %p, org ref_count %d", pktBuff, ref_cnt);
        /* check the buffer ownership is with user now */
        if(1 > ref_cnt) {
            TRC_OUT02(GR_INIT, LV_ERR,
                "User do not have ownership for buffer %p, ref_count %d", pktBuff, ref_cnt);
            return -1;
        }
    }

    /* change ownership to L2ETH_LIB */
    cp->m_user_receive_buffer_ref_count[pos] = ref_cnt = 0;

    TRC_OUT02(GR_INIT, LV_FCTPUB1, "l2_release_rbuff_ownership buffer %p, ref_count %d",
        pktBuff, ref_cnt);

    return 0;
}

/* get count of user owned buffers */
L2ETH_UINT32 l2_get_user_rbuff_ownership_count(l2eth_base * cp)
{
    L2ETH_INT16 pos, count = 0;

    for(pos = 0; pos < MAX_ETHERNET_RECEIVE_FRAME_COUNT_TOTAL; pos++) {
        if(cp->m_user_receive_buffer_ref_count[pos] == 1) {
            /* user is owner */
            count++;
        }
    }
    return count;
}

/* mark all buffers are with L2ETH_LIB ( value = 0) */
L2ETH_VOID l2_clear_rbuff_ownership(l2eth_base * cp)
{
    L2ETH_UINT32 pos;
    /* mark remaining buffers  are free ( value = 0) */
    for(pos = 0; pos < MAX_ETHERNET_RECEIVE_FRAME_COUNT_TOTAL; pos++) {
        cp->m_user_receive_buffer_ref_count[pos] = 0;
    }
}
