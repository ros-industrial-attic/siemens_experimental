/****************************************************************************/
/*                                                                          */
/*  Copyright (C) SIEMENS CORP., 2013 All rights reserved.*/
/*                                                                          */
/*    file: l2eth_rqb.h                                                     */
/*                                                                          */
/*    Description:                                                          */
/*    l2 request blocks                                                     */
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

#ifndef L2ETH_RQB_H
#define L2ETH_RQB_H

#include "dprlib.h"
#include "l2eth_defs.h"
#include "l2eth_config.h"

#undef  ATTR_PACKED
#if defined(_MSC_VER)
#pragma pack( push, safe_old_packing, 4 )
#define  ATTR_PACKED
#elif defined(__GNUC__)
#define ATTR_PACKED  __attribute__ ((aligned(4)))
#elif defined(BYTE_ATTR_PACKING)
#include "pack.h"
#define ATTR_PACKED PPC_BYTE_PACKED
#else
#error please adapt l2_rqb.h header for your compiler
#endif


/* preload buffer needs to be passed to EDD */
#define PRE_LOAD_BUF_SIZE         30 /* will be compared with SYS_ADAPT_RX_RQBS_CNT in firmware */

/* max dma buffer for query data */
#define MAX_DMA_QUERY_BUFFER_SIZE 114  // max L2ETH_OID_MULTICAST_LIST, 19*6 =114

/* handle lookup offset */
#define APP_HANDLE_OFFSET_BASE 0x21A0B0

/* total recieve buffer count */
#define MAX_ETHERNET_RECEIVE_FRAME_COUNT_TOTAL (PRE_LOAD_BUF_SIZE + MAX_ETHERNET_RECEIVE_FRAME_COUNT)

/* max size for send packet pool */
#define MAX_DMA_SEND_POOL_SIZE (MAX_ETHERNET_SEND_FRAME_COUNT*MAX_ETHERNET_FRAME_SIZE)

/* max size for receive packet pool */
#define MAX_DMA_RECEIVE_POOL_SIZE (MAX_ETHERNET_RECEIVE_FRAME_COUNT_TOTAL*MAX_ETHERNET_FRAME_SIZE)

/* total dma pool size
   dma is divided in to 3 regions for a CP
                     |  Receive buffer pool     | Send  buffer Pool      | query buffer              |*/
#define MAX_DMA_SIZE (MAX_DMA_RECEIVE_POOL_SIZE + MAX_DMA_SEND_POOL_SIZE + MAX_DMA_QUERY_BUFFER_SIZE )


typedef enum SEND_CHNL_OP_TAG {
    SCO_L2ETH_UNKNOWN,
    SCO_L2ETH_OPEN,
    SCO_L2ETH_CLOSE,
    SCO_L2ETH_SET_MODE,
    SCO_L2ETH_SET_MODE_ACK,
    SCO_L2ETH_SET_INFO,
    SCO_L2ETH_SET_INFO_ACK,
    SCO_L2ETH_GET_INFO,
    SCO_L2ETH_GET_INFO_ACK,
    SCO_L2ETH_SEND,
    SCO_L2ETH_SEND_ACK,
    SCO_L2ETH_RECEIVE,
    SCO_L2ETH_RECEIVE_ACK,
    SCO_L2ETH_STATUS_ACK,
} SEND_CHNL_OP;

typedef struct t_rq_open_l2 {
    L2ETH_UINT32 dma_phys_adds;  /* dma physical address  */
    L2ETH_UINT32 dma_virt_adds;  /* dma virtual address - reserved for future */
    L2ETH_UINT32 preload_buf_cnt;/* preload buffer count */
    L2ETH_UINT32 preload_buf;    /* preload buffer offset */
} ATTR_PACKED rq_open_l2;

typedef struct t_rq_close_l2 {
    L2ETH_UINT8 EmergencyClose; /* if 1 perform close without send confirm. */
} ATTR_PACKED rq_close_l2;

typedef struct t_rq_mode_l2 {
    L2ETH_MODE Mode;           /* Mode value */
} ATTR_PACKED rq_mode_l2;

typedef struct t_rq_query_l2 {
    L2ETH_UINT32 Oid;
    L2ETH_UINT32 BufferOffset;
    L2ETH_UINT32 BufferLength;
    L2ETH_UINT32 BytesTransferred;
    L2ETH_UINT32 BytesNeeded;
} ATTR_PACKED rq_query_l2;

typedef struct t_rq_status_l2 {
    L2ETH_UINT32 Oid;
} ATTR_PACKED rq_status_l2;

typedef struct t_rq_packet_l2 {
    L2ETH_UINT32 DataLength;
    L2ETH_UINT32 BufferOffset;
    L2ETH_UINT32 Context;
} ATTR_PACKED rq_packet_l2;

typedef struct T_L2ETH_MSG_HEADER {
    L2ETH_UINT8 opcode;
    L2ETH_UINT8 resp_ret;       /* only for response error code */
    L2ETH_UINT16 blk_len;
    L2ETH_UINT32 handle;        /* context */
} ATTR_PACKED L2ETH_MSG_HEADER;

typedef struct T_L2ETH_MSG_COMMAND {
    L2ETH_MSG_HEADER l2_header;
    union {
        rq_open_l2 open_l2;
        rq_close_l2 close_l2;
        rq_mode_l2 mode_l2;
        rq_query_l2 query_l2;
        rq_status_l2 status_l2;
        rq_packet_l2 send_pkts_l2;
        rq_packet_l2 send_acks_l2;
        rq_packet_l2 receive_packet;
        rq_packet_l2 receive_acks_l2;
    } u;
} ATTR_PACKED L2ETH_MSG_COMMAND;

#define L2ETH_SIZEOF_MSG_COMMAND 28

typedef enum L2ETH_PACKET_OWNER_TAG {
    L2ETH_PACKET_OWNER_FW,
    L2ETH_PACKET_OWNER_USER,
    L2ETH_PACKET_OWNER_L2ETH_LIB
} L2ETH_PACKET_OWNER;

#if defined(_MSC_VER)
#pragma pack( pop, safe_old_packing )
#elif defined(BYTE_ATTR_PACKING)
#include "unpack.h"
#endif

#undef ATTR_PACKED

#endif /* L2ETH_RQB_H */
