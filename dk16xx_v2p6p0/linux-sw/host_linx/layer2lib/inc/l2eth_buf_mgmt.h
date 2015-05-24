/****************************************************************************/
/*                                                                          */
/*  Copyright (C) SIEMENS CORP., 2013 All rights reserved.*/
/*                                                                          */
/*    file: l2eth_buf_mgmt.h                                                */
/*                                                                          */
/*    Description:                                                          */
/*    interface function declarations for buffer /packet management         */
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

#ifndef         _L2ETH_BUF_MGMT_H
#define         _L2ETH_BUF_MGMT_H

#ifdef __cplusplus
#ifndef EXTERN
#define EXTERN extern "C"
#endif
#else
#define EXTERN
#endif /* __cplusplus */

/* Function error codes */
typedef enum {
    L2ETH_BUF_SUCCESS,
    L2ETH_BUF_NO_RESOURCES,
    L2ETH_BUF_INVALID_PARAM
} L2ETH_BUF_ERROR;

typedef struct {
    L2ETH_UINT8 *base_address;
    L2ETH_UINT32 buf_size;
    L2ETH_UINT32 buf_cnt;
    L2ETH_UINT8 def_fill;
} L2ETH_BUFFER_DISCRIPTOR;

/*-----------------------------------------------------------------------------------*/
/*                                                                                   */
/*  |<---------------------------L2ETH_BUFFER_CONFIG-------------------------->|     */
/*  |                                                                                */
/*  |<-----BUF DISC----->|<-----AVAIL FLAG ARRAY----->|<-----PACKET BLOCK----->|     */
/*                                                                                   */
/*-----------------------------------------------------------------------------------*/
typedef struct {
    L2ETH_BUFFER_DISCRIPTOR bd;
    L2ETH_UINT32 cur_pos;       /* this variable may be removed. */
    L2ETH_UINT32 avail_flag_array_size; /* no of elements of the avail flag array */
    L2ETH_PACKET *pPacketBlock; /* pointer to the packet block */
    L2ETH_UINT32 avail_flag[1];  /* bit-wise - 0=free, 1=in use */
} L2ETH_BUFFER_CONFIG;

typedef L2ETH_BUFFER_CONFIG *L2ETH_BUFFER_HANDLE;

/****INTERFACES TO BE EXPOSED****/
/*
Function to initialize the buffers.
The caller should allocate the memory and pass it to the buffer,
this function will only divide that allocated memory into split buffers
*/
EXTERN L2ETH_BUF_ERROR l2_buf_init(L2ETH_BUFFER_DISCRIPTOR buf_disc,    /* [in] */
    L2ETH_UINT32 prealloc_cnt,  /* [in] */
    L2ETH_BUFFER_CONFIG ** hBuf  /* [out] */
    );


/*
Function to remove the buffer management, associated with the buffers.
It's the callers responsibility free the memory
*/
EXTERN L2ETH_BUF_ERROR l2_buf_remove(L2ETH_BUFFER_HANDLE *hBuf /* [in] */ );

/*
Function to get the next free buffer
*/
EXTERN L2ETH_BUF_ERROR l2_buf_get_next(L2ETH_BUFFER_HANDLE hBuf,        /* [in] */
    L2ETH_PACKET ** pNext       /* [out] */
    );
/*
Function to free buffer
*/
EXTERN L2ETH_BUF_ERROR l2_buf_free(L2ETH_BUFFER_HANDLE hBuf,    /* [in] */
    L2ETH_PACKET * pNext        /* [in] */
    );

/*
Function to retrieve associated param of a given buffer
*/
L2ETH_BUF_ERROR l2_buf_get_packet(L2ETH_BUFFER_HANDLE hBuf,     /* [in] */
    L2ETH_UINT8 * pBuf,         /* [in] */
    L2ETH_PACKET ** ppPacket    /* [out] */
    );

/*
Function returns the number of free buffers in the buffer-ring
*/
EXTERN L2ETH_UINT32 l2_buf_get_free_cnt(L2ETH_BUFFER_HANDLE hBuf, /* [in] */
    L2ETH_UINT32 bits_cnt  /* [in] */
    );

/*
Function returns the number of total buffers in the buffer-ring
*/
EXTERN L2ETH_UINT32 l2_buf_get_total_cnt(L2ETH_BUFFER_HANDLE hBuf /* [in] */ );


#endif /* _L2ETH_BUF_MGMT_H */

