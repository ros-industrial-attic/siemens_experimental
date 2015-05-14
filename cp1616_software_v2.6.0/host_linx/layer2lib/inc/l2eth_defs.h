/*****************************************************************************/
/*                                                                           */
/*  Copyright (C) SIEMENS CORP., 2013 All rights reserved.*/
/*                                                                           */
/*    file: l2eth_defs.h                                                     */
/*                                                                           */
/*    Description:                                                           */
/*    data types declarations for layer 2 interface                          */
/*                                                                           */
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
#ifndef L2ETH_DEFS_H
#define L2ETH_DEFS_H

#undef  ATTR_PACKED
#if defined(_MSC_VER)
#pragma pack( push, safe_old_packing, 1 )
#define  ATTR_PACKED
#elif defined(__GNUC__)
#define ATTR_PACKED  __attribute__ ((aligned(1)))
#elif defined(BYTE_ATTR_PACKING)
#include "pack.h"
#define ATTR_PACKED PPC_BYTE_PACKED
#else
#error please adapt l2ag_rqb.h header for your compiler
#endif

/* base data type specification
   elementary data types are defined in os_linux, os_xxx files */

#include "os.h"

typedef DPR_CHAR   L2ETH_UINT8;
typedef DPR_UINT16 L2ETH_UINT16;
typedef DPR_UINT32 L2ETH_UINT32;

typedef DPR_VOID   L2ETH_VOID;

typedef DPR_INT16 L2ETH_INT16;
typedef DPR_INT32 L2ETH_INT32;

typedef enum L2ETH_BOOL_TYPE {
    L2ETH_FALSE = 0,
    L2ETH_TRUE
} L2ETH_BOOL;

/* L2 modes */
typedef enum L2ETH_MODE_TYPE {
    L2ETH_OFFLINE,
    L2ETH_ONLINE
} L2ETH_MODE;

/* structure to define the minimum and maximum size (BufferLength)
   allowed for each query OID */

typedef struct L2ETH_OID_LENGTH_TAG {
    L2ETH_UINT32 minBuf_Length;
    L2ETH_UINT32 maxBuf_Length;
} L2ETH_OID_LENGTH_INFO;

/* L2 objects IDs for get and set data */
typedef enum L2ETH_OID_TYPE {
    /*The OIDs that can be get and set */
    L2ETH_OID_READWRITE = 0x00000000,  //delimiter
    L2ETH_OID_MULTICAST_LIST,
    L2ETH_OID_MAXIMUM_FRAME_SIZE,

    /*The OIDs which can be only get */
    L2ETH_OID_READONLY,         //delimiter
    L2ETH_OID_PERMANENT_ADDRESS,
    L2ETH_OID_MAXIMUM_LIST_SIZE,
    L2ETH_OID_MEDIA_CONNECT_STATUS,

    /*The OIDs maximum */
    L2ETH_OID_MAX  //delimiter
} L2ETH_OID;

/* enum to for get_information, set_information types */
typedef enum GET_SET_TYPE_TAG {
    SET_INFO,
    GET_INFO
} GET_SET_TYPE;


/* L2 structure for get and set data */
typedef struct L2ETH_QUERY_TYPE {
    L2ETH_OID Oid;
    L2ETH_UINT8 * pBuffer;
    L2ETH_UINT32 BufferLength;
    L2ETH_UINT32 BytesTransferred;
    L2ETH_UINT32 BytesNeeded;
} L2ETH_QUERY;

/* L2 structure for send and receive data */
typedef struct L2ETH_PACKET_TYPE {
    L2ETH_UINT32 DataLength;
    L2ETH_UINT8 *pBuffer;
    L2ETH_UINT32 Context;
} L2ETH_PACKET;

/* L2 structure for MAC address*/
typedef L2ETH_UINT8 L2ETH_MAC_ADDR[6];

/* maximum ports supported by the card */
#define L2ETH_MAX_PORTS 4

/* port status defines. */
#define L2ETH_LINK_UNKNOWN        0       /* unknown Link/Bitrate/Duplex */

/* Link:   Status of link */
#define L2ETH_LINK_UNKNOWN        0       /* unknown */
#define L2ETH_LINK_UP             1       /* Link Up     */
#define L2ETH_LINK_DOWN           2       /* Link Down   */
#define L2ETH_LINK_UP_CLOSED      3       /* port closed */
#define L2ETH_LINK_UP_DISABLED    4       /* post disabled */


/* Bitrate:   Speed if link is up */
#define L2ETH_LINK_SPEED_10       1       /* 10  MBit/s  */
#define L2ETH_LINK_SPEED_100      2       /* 100 MBit/s  */

/* Duplex:   Duplex Mode  if link is up */
#define L2ETH_LINK_MODE_HALF      1       /* Half Duplex */
#define L2ETH_LINK_MODE_FULL      2       /* Full Duplex */

/* L2 structure for Link status information */
typedef struct{
    L2ETH_UINT32    Link;
    L2ETH_UINT32    Bitrate;
    L2ETH_UINT32    Duplex;
}L2ETH_PORT_STATUS;

#if defined(_MSC_VER)
#pragma pack( pop, safe_old_packing )
#elif defined(BYTE_ATTR_PACKING)
#include "unpack.h"
#endif

#undef ATTR_PACKED

#endif /* L2ETH_DEFS_H */

