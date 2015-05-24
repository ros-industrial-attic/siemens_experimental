/*---------------------------------------------------------------------------*/
/* Copyright (C) SIEMENS CORP., 2013 All rights reserved.*/
/*---------------------------------------------------------------------------*/
/*                                                                           */
/*   Project           : CP16xx                                              */
/*   Filename          : l2eth_ping.h                                        */
/*                                                                           */
/*---------------------------------------------------------------------------*/
/*   Description:  Layer2 sample program                                     */
/*                                                                           */
/*---------------------------------------------------------------------------*/
/* Attention : Callbacks are running concurrent in other threads so all      */
/*             printf statements should be synchronized. But this sample     */
/*             application doesn't synchronize for simplicity  !             */
/*---------------------------------------------------------------------------*/
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

#ifndef __L2ETH_PING_HEAD_H__
#define __L2ETH_PING_HEAD_H__


/****************************************************************************
* User Defined Structures
*
*****************************************************************************/
/* Begin - Declaring a structure for Ip Header */

struct ipheader{
    unsigned char verlen;     /* Version and Internet Header Length. The Version field indicates the format of the internet header. */
    unsigned char tos;        /* The Type of Service provides an indication of the abstract parameters of the quality of service desired.. */
    unsigned short totlen;    /* Total Length, includes datagram and header length. */
    unsigned short id;        /* An identifying value assigned by the sender to aid in assembling the fragments of a datagram. */
    unsigned short frag;      /* Flags and Fragmentation Offset. */
    unsigned char ttl;        /* Time to live, the number of hops to live. */
    unsigned char prot;       /* This field indicates the next level protocol used in the data portion of the internet datagram. */
    unsigned short chksum;    /* A checksum on the header only. */
    unsigned long sourceip;   /* The source address. */
    unsigned long destip;     /* The destination address. */
};

/* End - Declaring a structure for Ip Header */
/*****************************************************************************/


/****************************************************************************
* User Defines
*
*************************************************************************************************************/
# define T_ERROR -1                                             /* Error value */
# define T_SUCCESS 0                                            /* Success value */
# define SOURCE_IP "192.168.1.39"                               /* Source IP address should be changed here. */
# define SOURCE_MAC "08-00-06-93-da-0e"                         /* Source MAC address should be changed here. */
/*# define PRINTF*/                                             /* If defined, then will print all messages. */
# define LENGTH_OF_PACKET 1514                                  /* Maximum length of the packet is declared here. */
/*************************************************************************************************************/


/****************************************************************************
* User Global variables
*
*************************************************************************************************************/
L2ETH_UINT32 CP_INDEX = 1;                                      /* Holds the CP_INDEX to be used. */
static L2ETH_UINT32 dwHandle = 0;                               /* 0 - not opend; other than 0, opened. */
static int mode_flag = L2ETH_OFFLINE;                           /* 0 - mode is online; 1 - mode is offline. */
static int identity = 0;                                        /* Used for filling Identity field in IP header. */
static int ntransmitted = 0;                                    /* Used for filling transmitted field in ICMP header.*/
static long global_long = 12345;                                /* To fill the context field. */
L2ETH_UINT32 status;                                            /* To store the return value of each function. */
int flag;                                                       /* To flag an event. */
/************************************************************************************************************/



/*********************************************************************************************
* Begin - User Defined Functions
*********************************************************************************************/
void user_close ( int sig );

int Fill_Ping_Data ( L2ETH_PACKET *, unsigned char *, unsigned char *, unsigned int );
int get_MAC ( unsigned char *, unsigned char *, int );
void user_callback_l2eth_send_complete ( L2ETH_UINT32 , L2ETH_UINT32 , L2ETH_PACKET * );
void user_callback_l2eth_mode_complete_cbf ( L2ETH_UINT32 , L2ETH_MODE , L2ETH_UINT32 );
void user_callback_l2eth_status_indication ( L2ETH_UINT32       , L2ETH_OID     );
int user_callback_l2eth_receive_indication ( L2ETH_UINT32 ,L2ETH_PACKET * );
unsigned short checksum ( unsigned short *, unsigned int );
int l2eth_ping_reply ( L2ETH_UINT32 , L2ETH_PACKET * );
/*********************************************************************************************/

#endif /*__L2ETH_PING_HEAD_H__*/
