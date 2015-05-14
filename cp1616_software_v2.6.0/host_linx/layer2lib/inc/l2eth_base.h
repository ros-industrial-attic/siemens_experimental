/****************************************************************************/
/*                                                                          */
/*  Copyright (C) SIEMENS CORP., 2013 All rights reserved.*/
/*                                                                          */
/*    file: l2eth_base.h                                                    */
/*                                                                          */
/*    Description:                                                          */
/*    interface function declarations for layer 2 interface .               */
/*                                                                          */
/****************************************************************************/
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
#ifndef L2ETH_BASE_H
#define L2ETH_BASE_H

#ifdef _MSC_VER
#if _MSC_VER > 1000
#pragma once
#endif
#endif

#include "os.h"

#include "l2eth_defs.h"
#include "l2eth_lib_cfg.h"
#include "l2eth_rqb.h"
#include "l2eth_user.h"
#include "l2eth_buf_mgmt.h"

/* ioctl */
#define DPR_IOC_OAPP                   CP16XX_IOC_OAPP
#define DPR_IOC_CAPP                   CP16XX_IOC_CAPP
#define DPR_IOC_BIND                   CP16XX_IOC_BIND
#define DPR_IOC_UNBIND                 CP16XX_IOC_UNBIND
#define DPR_IOC_GET_L2_DMA_PHYS_ADDR   CP16XX_IOC_GET_L2_DMA_PHYS_ADDR
#define DPR_IOC_GET_CP_TIMER           CP16XX_IOC_GET_CP_TIMER

/*  enums to check DMA buffer type  */
typedef enum TAG_L2ETH_PACKET_BUFFER_TYPE {
    L2ETH_DMA_SEND_BUFFER,
    L2ETH_DMA_RECEIVE_BUFFER,
} L2ETH_PACKET_BUFFER_TYPE;


/* l2 channel defines */
typedef enum L2ETH_CHANNELS_TYPE {
    L2ETH_SEND = 0,
    L2ETH_RECEIVE,
} L2ETH_CHANNELS;


/* channel data */
typedef struct tagChannelData {
    L2ETH_UINT32 stop_thread;
    L2ETH_UINT32 channel_number;
    L2ETH_DRV_HANDLE file;

    L2ETH_UINT32 user_pool_length_used;
    L2ETH_UINT32 user_pool_length;
    L2ETH_UINT8 *user_pool_ptr;

    L2ETH_THREAD_HANDLE th_reader;
    L2ETH_UINT8 *parent;
} L2ETH_CHANNEL_DATA;


/* dpr channel adapter */
typedef struct tagL2ethDprAdapter {
    L2ETH_UINT32 cp_index;
    struct l2eth_base_struct *l2eth_base_ptr;
    L2ETH_UINT32 user_id;

    L2ETH_DRV_HANDLE fd_control;
    L2ETH_CHANNEL_DATA chnls[2];
    L2ETH_MUTEX synch_channel_mutex;

        /* virtual dma memory */
    L2ETH_UINT8 *dma_base_ptr;

        /* store dma Physical memory and size */
    L2ETH_UINT32 m_dmaPhysicalPool;
    L2ETH_UINT32 m_dma_size;
} L2ETH_DPR_ADAPTER;


/* struct l2eth_base_struct */
typedef struct  l2eth_base_struct {

    /* semaphore to keep synchronous operation */
    L2ETH_SEMAPHORE m_semSendReceiveSynch;

    /* emergency close operation */
    L2ETH_BOOL m_bEmergencyClose;

    /* keep max possible send and receive frame count */
    L2ETH_UINT32 m_dma_receive_frame_count;
    L2ETH_UINT32 m_dma_send_frame_count;

    /* to keep mode info */
    L2ETH_MODE m_l2_mode;
    L2ETH_BOOL m_bmodeChangeRequested;

    /* to keep max IOD Frame size */
    L2ETH_UINT32 m_maxOidFrameSize;

    /* store adapter information  */
    L2ETH_DPR_ADAPTER *m_pCpAdapter;

    /* store callback function pointer for send */
    L2ETH_CBF_SEND_COMPL m_send_cbf;

    /* store callback function pointer for receive */
    L2ETH_CBF_RECEIVE_IND m_recv_cbf;

    /* store callback function pointer for status */
    L2ETH_CBF_STATUS_IND m_status_cbf;

    /* store callback function pointer for mode change */
    L2ETH_CBF_MODE_COMPL m_mode_cbf;

    /* buffer management handle for send */
    L2ETH_BUFFER_HANDLE m_hSendBuff;

    /* buffer management handle for receive */
    L2ETH_BUFFER_HANDLE m_hRecvBuff;

    /* to keep receive buffer ownership info */
    L2ETH_INT16 m_user_receive_buffer_ref_count[MAX_ETHERNET_RECEIVE_FRAME_COUNT_TOTAL];

    /* to mark async transfer between different sessions.
       new session begins with MODE_ONLINE comfirmation and ends with
       another MODE_OPNLINE comfirmation */
    L2ETH_UINT32 m_context_handle;
} l2eth_base;


/* function declarations */

L2ETH_VOID l2_base_cons(l2eth_base *cp);

L2ETH_VOID l2_base_des(l2eth_base *cp);

L2ETH_UINT32 l2_open(L2ETH_UINT32 cpIndex,      /* in */
    L2ETH_CBF_RECEIVE_IND receiveCbf,       /* in */
    L2ETH_CBF_SEND_COMPL sendCompleteCbf,   /* in */
    L2ETH_CBF_STATUS_IND indicationCbf,     /* in */
    L2ETH_CBF_MODE_COMPL modeCompCbf,    /* in */
    L2ETH_UINT32 * pApplHandle);          /* out */

L2ETH_UINT32 l2_close(l2eth_base *cp);

L2ETH_UINT32 l2_get_allocate_packet(l2eth_base *cp,
    L2ETH_PACKET ** ppPkt);    /* out */

L2ETH_UINT32 l2_send_packet(l2eth_base *cp,
    L2ETH_PACKET * pPkt);      /* in */

L2ETH_UINT32 l2_free_packet(l2eth_base *cp,
    L2ETH_PACKET * pPkt);      /* in */

L2ETH_UINT32 l2_return_packet(l2eth_base *cp,
    L2ETH_PACKET * pPkt,
    L2ETH_PACKET_OWNER owner); /* in */

L2ETH_UINT32 l2_set_mode(l2eth_base *cp,
    L2ETH_MODE mode);          /* in */

L2ETH_UINT32 l2_getset_information(l2eth_base *cp,
    GET_SET_TYPE getsettype,   /* in */
    L2ETH_QUERY * pQuery);     /* in, out */

L2ETH_UINT32 l2_uninit_cp(l2eth_base *cp);

L2ETH_UINT32 l2_init_cp(l2eth_base *cp,
    L2ETH_UINT32 cp_index);     /* in */

l2eth_base *l2_get_instance(L2ETH_UINT32 handle); /* in */

L2ETH_UINT32 l2_get_handle(l2eth_base *pBase); /* in */

L2ETH_UINT32 l2_send_receive_synch(l2eth_base *cp,
    L2ETH_UINT8 * pRq,              /* in, out */
    L2ETH_UINT32 send_length,       /* in */
    L2ETH_UINT32 * receive_length); /* in,out */

L2ETH_UINT32 l2_proc_receive_packet(l2eth_base *cp);

L2ETH_UINT32 l2_proc_send_cnf(l2eth_base *cp);

L2ETH_UINT32 l2_proc_mode_cnf(l2eth_base *cp);

L2ETH_UINT32 l2_proc_status_ind(l2eth_base *cp);

L2ETH_VOID l2_set_emergency_close(l2eth_base *cp,
    L2ETH_BOOL emergencyClose);  /* in */

L2ETH_UINT32 l2_close_dpr_channel(L2ETH_CHANNEL_DATA * pChnl); /* in */

L2ETH_UINT32 l2_open_dpr_channel(const char * pFileName, /* in */
    L2ETH_DPR_ADAPTER * pAdapter,   /* in */
    L2ETH_CHANNELS chnl,            /* in */
    L2ETH_UINT32 min_read_len);     /* in */

/* initalize dma area  */
L2ETH_UINT32 l2_init_dma_pool(l2eth_base *cp,
    L2ETH_DRV_HANDLE file,  /* in */
    L2ETH_UINT32 cpIndex);  /* in */

/* sending receive buffers to FW  */
L2ETH_UINT32 l2_send_receive_packet(l2eth_base *cp,
    L2ETH_PACKET * pPacket);   /* in */

L2ETH_UINT32 l2_send_all_receive_packets(l2eth_base *cp,
    L2ETH_UINT32 reqCount);    /* in */

/* write message to a channel  */
L2ETH_UINT32 l2_channel_write(l2eth_base *cp,
    L2ETH_CHANNELS Chnl,       /* in */
    L2ETH_UINT8 * pRq,         /* in */
    L2ETH_UINT32 send_length); /* in */

/* read message from a channel  */
L2ETH_UINT8 *l2_get_message(l2eth_base *cp,
    L2ETH_UINT8 * buf,      /* in, out */
    L2ETH_CHANNELS Chnl,    /* in */
    L2ETH_UINT32 * msgLen); /* in,out */

/* validate the send and receive buffer retured  */
L2ETH_UINT32 l2_validate_packet_buffer(l2eth_base *cp,
    L2ETH_PACKET_BUFFER_TYPE BuffType,      /* in */
    L2ETH_PACKET * packet); /* in */

/* get dma buffer for set /get query info  */
L2ETH_UINT32 *l2_get_dma_queryInfo_buffer(l2eth_base *cp);

/* mark preload buffers */
L2ETH_UINT32 l2_init_rbuff_ownership(l2eth_base *cp);

/* change ownership of buffer,
    returns -1 for error, others are ok */
L2ETH_INT32 l2_addref_rbuff_ownership(l2eth_base *cp,
    L2ETH_UINT8 * pktBuff,    /* buffer pointer */
    L2ETH_PACKET_OWNER owner); /* buffer owner */

/* release -  release ownership of buffer
    returns -1 for error, others are ok */
L2ETH_INT32 l2_release_rbuff_ownership(l2eth_base *cp,
    L2ETH_UINT8 * pktBuff,    /* packet pointer */
    L2ETH_PACKET_OWNER owner);      /* packet owner */

/* get count of user owned buffers */
L2ETH_UINT32 l2_get_user_rbuff_ownership_count(l2eth_base *cp);

/* mark all buffers */
L2ETH_VOID l2_clear_rbuff_ownership(l2eth_base *cp);

/* get cp1616 card timer) */
L2ETH_UINT32 l2_get_card_timer(l2eth_base *cp);

#endif /* L2ETH_BASE_H */
