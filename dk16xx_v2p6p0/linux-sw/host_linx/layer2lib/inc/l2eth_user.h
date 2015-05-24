/****************************************************************************/
/*                                                                          */
/*  Copyright (C) SIEMENS CORP., 2013 All rights reserved.*/
/*                                                                          */
/*    file: l2eth_user.h                                                    */
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

#ifndef L2ETH_USER_H
#define L2ETH_USER_H

#undef ATTR_PACKED
#if defined(_MSC_VER)
#pragma pack( push, safe_old_packing, 4 )
#define ATTR_PACKED
#elif defined(__GNUC__)
#define ATTR_PACKED  __attribute__ ((aligned (4)))
#elif defined(BYTE_ATTR_PACKING)
#include "pack.h"
#define ATTR_PACKED PPC_BYTE_PACKED
#else
#error please adapt l2user.h header for your compiler
#endif

#undef L2ETH_CODE_ATTR
#ifdef PASCAL
#define L2ETH_CODE_ATTR __stdcall
#else
#define L2ETH_CODE_ATTR
#endif

/* include basic structures and other typedefs */
#include "l2eth_defs.h"

/*===========================================================================
* FUNCTION : L2ETH_CBF_STATUS_IND
*----------------------------------------------------------------------------
* PURPOSE  : user callback function for status change
*----------------------------------------------------------------------------
* RETURNS  : nil
*----------------------------------------------------------------------------
* INPUTS   : -
*            Handle - Handle for communication channel
*            Oid    - Status that has changed
* OUTPUS   : -
*----------------------------------------------------------------------------
* COMMENTS : This user callback is called by the interface to intimate
* the changed status. If the user is interested in a detailed change,
* user can call l2eth_get_information.
*==========================================================================*/
typedef void (*L2ETH_CBF_STATUS_IND) (L2ETH_UINT32 Handle,      /* in */
    L2ETH_OID Oid);             /* in */

/*===========================================================================
* FUNCTION : L2ETH_CBF_RECEIVE_IND
*----------------------------------------------------------------------------
* PURPOSE  : user callback function for receive
*----------------------------------------------------------------------------
* RETURNS  : Reserved
*----------------------------------------------------------------------------
* INPUTS   : -
*            Handle - Handle for communication channel
*            pPacket - Pointer to the receive packet
* OUTPUS   : -
*----------------------------------------------------------------------------
* COMMENTS : This user callback is called by the interface to pass
* a receive buffer to the user application. The receive buffer has to be
* returned to the interface us-ing l2eth_return_paket().
*==========================================================================*/
typedef int (*L2ETH_CBF_RECEIVE_IND) (L2ETH_UINT32 Handle,      /* in */
    L2ETH_PACKET * pPacket);   /* in */

/*===========================================================================
* FUNCTION : L2ETH_CBF_SEND_COMPL
*----------------------------------------------------------------------------
* PURPOSE  : user callback function for send
*----------------------------------------------------------------------------
* RETURNS  : nil
*----------------------------------------------------------------------------
* INPUTS   : -
*            Handle - Handle for communication channel
*            pPacket - Pointer to the receive packet
*            Error - Error code of the send procedure
* OUTPUS   : -
*----------------------------------------------------------------------------
* COMMENTS : This user callback is called by the interface to return
* the send buffer and send status to the user application.
*==========================================================================*/
typedef void (*L2ETH_CBF_SEND_COMPL) (L2ETH_UINT32 Handle,      /* in */
    L2ETH_PACKET * pPacket,    /* in */
    L2ETH_UINT32 Error);       /* in */

/*===========================================================================
* FUNCTION : L2ETH_CBF_MODE_COMPL
*----------------------------------------------------------------------------
* PURPOSE  : This user callback is called by the interface to intimate the changed mode.
                         This function is called by the L2 host interface, as a result of user calling
                         l2eth_set_mode.
*----------------------------------------------------------------------------
* RETURNS  : nil
*----------------------------------------------------------------------------
* INPUTS   : -
*            Handle - Handle for communication channel
*            Mode - The new mode, after changing
*            Error - Error code of the send procedure
* OUTPUS   : -
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
typedef void (*L2ETH_CBF_MODE_COMPL) (L2ETH_UINT32 Handle,      /* in */
    L2ETH_MODE Mode,        /* in */
    L2ETH_UINT32 Error);    /* in */

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================
* FUNCTION : l2eth_open
*----------------------------------------------------------------------------
* PURPOSE  : open l2 interface
*----------------------------------------------------------------------------
* RETURNS  : L2ETH_OK on success; else Error code for execution of function
*----------------------------------------------------------------------------
* INPUTS   : -
*            CpIndex - card number
*            CbfReceiveInd - Callback function that is called by the in-terface
*                            to return a receive buffer      to the  application.
*            CbfSendCompl - Callback function that is called by the in-terface
*                           to report a change in status.
*            CbfStatusInd - Callback function that is called by the in-terface
*                           to return the send buffer to the applica-tion.
*            CbfModeCompl - Callback function, to report a change in mode.
*
* OUTPUS   : -
*            pHandle - Handle for communication channel
*----------------------------------------------------------------------------
* COMMENTS : This function registers the user application and
* its callback functions with the interface. The returned handle is required
* for all other calls to identify the application.
*==========================================================================*/
L2ETH_UINT32 l2eth_open(L2ETH_UINT32 CpIndex, /* in */
    L2ETH_CBF_RECEIVE_IND CbfReceiveInd,    /* in */
    L2ETH_CBF_SEND_COMPL CbfSendCompl,      /* in */
    L2ETH_CBF_STATUS_IND CbfStatusInd,      /* in */
    L2ETH_CBF_MODE_COMPL CbfModeCompl,      /* in */
    L2ETH_UINT32 * pHandle);  /* out */

/*===========================================================================
* FUNCTION : l2eth_close
*----------------------------------------------------------------------------
* PURPOSE  : close l2 interface
*----------------------------------------------------------------------------
* RETURNS  : L2ETH_OK on success; else Error code for execution of function
*----------------------------------------------------------------------------
* INPUTS   : -
*            Handle - Handle for communication channel
* OUTPUS   : -
*----------------------------------------------------------------------------
* COMMENTS : This function de-registers the user application from
* the interface. The passed handle becomes invalid after this function call.
*==========================================================================*/
L2ETH_UINT32 l2eth_close(L2ETH_UINT32 Handle);        /* in */

/*===========================================================================
* FUNCTION : l2eth_allocate_packet
*----------------------------------------------------------------------------
* PURPOSE  : get free packet for send purpose
*----------------------------------------------------------------------------
* RETURNS  : L2ETH_OK on success; else Error code for execution of function
*----------------------------------------------------------------------------
* INPUTS   : -
*            Handle - Handle for communication channel
* OUTPUS   : -
*            ppPacket - address of the free packet
*----------------------------------------------------------------------------
* COMMENTS : This function provides the user the next available buffer.
* If there is no memory available an error is returned.
* User must call this function before making the send request.
*==========================================================================*/
L2ETH_UINT32 l2eth_allocate_packet(L2ETH_UINT32 Handle,     /* in */
    L2ETH_PACKET ** ppPacket);      /* out */

/*===========================================================================
* FUNCTION : l2eth_free_packet
*----------------------------------------------------------------------------
* PURPOSE  : free the packet allocated by "l2eth_allocate_packet"
*----------------------------------------------------------------------------
* RETURNS  : L2ETH_OK on success; else Error code for execution of function
*----------------------------------------------------------------------------
* INPUTS   : -
*            Handle - Handle for communication channel
* OUTPUS   : -
*            pPacket - address of the previously allocated packet packet
*----------------------------------------------------------------------------
* COMMENTS : This function provides frees the allocated packet.
* User must call this function when he no more requires the packets' for send
* purpose
*==========================================================================*/
L2ETH_UINT32 l2eth_free_packet(L2ETH_UINT32 Handle, /* in */
    L2ETH_PACKET * pPacket);        /* in */

/*===========================================================================
* FUNCTION : l2eth_send
*----------------------------------------------------------------------------
* PURPOSE  : send l2 packet
*----------------------------------------------------------------------------
* RETURNS  : L2ETH_OK on success; else Error code for execution of function
*----------------------------------------------------------------------------
* INPUTS   : -
*            handle - Handle for communication channel
*            packet - address of the packet
* OUTPUS   : -
*----------------------------------------------------------------------------
* COMMENTS : This function passes the send data to the L2 Agent
* through DPRAM. The interface possesses the send buffer until    -
* the callback "sendCompleteCbf" is called by the interface.
*==========================================================================*/
L2ETH_UINT32 l2eth_send(L2ETH_UINT32 Handle,        /* in */
    L2ETH_PACKET * pPacket);        /* in */

/*===========================================================================
* FUNCTION : l2eth_return_packet
*----------------------------------------------------------------------------
* PURPOSE  : return packet received in receive call back function
*----------------------------------------------------------------------------
* RETURNS  : L2ETH_OK on success; else Error code for execution of function
*----------------------------------------------------------------------------
* INPUTS   : -
*            Handle - Handle for communication channel
*            pPacket - address of the packet
* OUTPUS   : -
*----------------------------------------------------------------------------
* COMMENTS : This function returns the receive buffer of the interface.
* It removes the message from the receive-queue.
*==========================================================================*/
L2ETH_UINT32 l2eth_return_packet(L2ETH_UINT32 Handle,       /* in */
    L2ETH_PACKET * pPacket);        /* in */

/*===========================================================================
* FUNCTION : l2eth_set_mode
*----------------------------------------------------------------------------
* PURPOSE  : set l2 mode
*----------------------------------------------------------------------------
* RETURNS  : L2ETH_OK on success; else Error code for execution of function
*----------------------------------------------------------------------------
* INPUTS   : -
*            handle - Handle for communication channel
*            mode - Mode of the communication channel
* OUTPUS   : -
*----------------------------------------------------------------------------
* COMMENTS : This function activates/enables (mode=ONLINE ),
* the receive and send mode or deactivates/disables
* the same (mode = OFFLINE )
*==========================================================================*/
L2ETH_UINT32 l2eth_set_mode(L2ETH_UINT32 Handle,    /* in */
    L2ETH_MODE Mode);       /* in */

/*===========================================================================
* FUNCTION : l2eth_set_information
*----------------------------------------------------------------------------
* PURPOSE  : set l2 information
*----------------------------------------------------------------------------
* RETURNS  : L2ETH_OK on success; else Error code for execution of function
*----------------------------------------------------------------------------
* INPUTS   : -
*            Handle - Handle for communication channel
*            pQuery - pointer to the query structure
* OUTPUS   : -
*----------------------------------------------------------------------------
* COMMENTS : This function is used to set different flags
* like multicast address.
*==========================================================================*/
L2ETH_UINT32 l2eth_set_information(L2ETH_UINT32 Handle,     /* in */
    L2ETH_QUERY * pQuery);  /* in */

/*===========================================================================
* FUNCTION : l2eth_get_information
*----------------------------------------------------------------------------
* PURPOSE  : get l2 information
*----------------------------------------------------------------------------
* RETURNS  : L2ETH_OK on success; else Error code for execution of function
*----------------------------------------------------------------------------
* INPUTS   : -
*            Handle - Handle for communication channel
* OUTPUS   : -
*            pQuery - pointer to the query structure
*----------------------------------------------------------------------------
* COMMENTS : This function is used to get different flags
* like multicast address.
*==========================================================================*/
L2ETH_UINT32 l2eth_get_information(L2ETH_UINT32 Handle,     /* in  */
    L2ETH_QUERY * pQuery);  /* out */

#ifdef __cplusplus
}
#endif

#if defined(_MSC_VER)
#pragma pack( pop, safe_old_packing )
#elif defined(BYTE_ATTR_PACKING)
#include "unpack.h"
#endif

#endif /* L2ETH_USER_H */
