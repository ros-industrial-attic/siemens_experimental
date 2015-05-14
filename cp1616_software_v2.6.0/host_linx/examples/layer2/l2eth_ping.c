/*---------------------------------------------------------------------------*/
/* Copyright (C) SIEMENS CORP., 2013 All rights reserved.*/
/*---------------------------------------------------------------------------*/
/*                                                                           */
/*   Project           : CP16xx                                              */
/*   Filename          : l2eth_ping.c                                        */
/*                                                                           */
/*                                                                           */
/*---------------------------------------------------------------------------*/
/*   Description:  This sample program shows the usages of                   */
/*                 Layer2 interfaces.                                        */
/*                                                                           */
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


/**********************************************************************************************
  Notes: !!! IMPORTANT !!! !!! IMPORTANT !!! !!! IMPORTANT !!! !!! IMPORTANT !!!

    Before you can use this program to receive reply packets, you should modify
    addresses in the header file l2eth_ping_header.h:
       SOURCE_MAC: Mac addr. of the cp1616 (see below)
       SOURCE_IP : == source_pseudo_IP_address (see below)

    The program opens the channel, set the mode to online and send the user-specified
    number of packets, and specified length of each packet to the specified destination.
    After sending the packets, the program waits for the user input to close the channel
    and exit. While waiting for the user input it will accept any ECHO_REQUEST packets from
    any other systems also.
    If the user enters wrong parameter syntax, the program displays the correct one.

    User can at any point of time exit the program by pressing "Ctrl+C."

    If the user wants to send ECHO_REQUEST packets and to receive REPLY packets from the
    remote machine, he needs to add the static entry to the ARP list in that machine.

    $:\>arp -s source_pseudo_IP_address cp1616_board_MAC_address

    The "source_pseudo_IP_address" is the "source ip address" specified in this program.
    See 'SOURCE_IP' constant in the l2eth_ping_header.h This address has no meaning for the
    source (client) machine. This address must not be assigned to any node on the network.
    This address act as a makeshift for the echo reply. Since we are not using IP stack by this
    test program (we are sending raw frames over layer two interface) no ARP can and should
    respond. This source_pseudo_IP_address must be added to the ARP table of the remote machine.
    I.e. ICMP_ECHO server. The MAC address of this ARP entry (cp1616_board_MAC_address) is a
    "layer2 mac address" of the cp1616 board in the client (requestor) machine.
    You can find this mac address printed on the board or in the WEB interface of the board or
    you can use the mac address displayed in STEP7 but reduced by one.

************************************************************************************************/


/********************************************/
/* Begin - System Defined headers */
# include <stdio.h>
# include <unistd.h>
# include <stdlib.h>
# include <sys/time.h>

# include <netinet/in.h>
# include <arpa/inet.h>
# include <string.h>

# include <time.h>
# include <malloc.h>
# include <sys/socket.h>

# include <errno.h>

# include <sys/param.h>
# include <sys/types.h>
# include <sys/cdefs.h>
# include <sys/socket.h>
# include <sys/file.h>

# include <netinet/in_systm.h>
# include <netinet/in.h>
# include <netinet/ip.h>
# include <netinet/ip_icmp.h>
# include <netdb.h>
# include <signal.h>
/* End - System Defined headers */


/* Begin - User Defined headers */
# include "l2eth_defs.h"
# include "l2eth_errs.h"
# include "l2eth_user.h"
# include "l2eth_ping_header.h"
/* End - User Defined headers */
/********************************************/



/*====================================================================================
* FUNCTION      : int l2eth_ping_reply ()
* DESCRIPTION   : This function handles the ICMP packet type.
* RETURNS       : SUCCESS
*====================================================================================*/
int l2eth_ping_reply(L2ETH_UINT32 dwHandle /* Input */ ,
                     L2ETH_PACKET * pPkt /* Input/Output */ )
{

    struct icmp *recv_icp;      /* Points to the ICMP Header of receive packet. */
    struct icmp *Send_icp;      /* Points to the ICMP Header of send packet. */
    struct ipheader *Recv_ip_hdr;       /* Points to the IP Header of receive packet. */
    struct ipheader *Send_ip_hdr;       /* Points to the IP Header of send packet. */
    unsigned long send_time;    /* To store the "send time" value in receive packet. */
    unsigned char recv_ip[6];   /* To store the IP address of the sender. */
    unsigned char recv_mac[7];  /* To store the MAC address of the sender. */
    L2ETH_PACKET *Send_pPkt;    /* Pointer to L2ETH packet to send. */
    struct timeval tm;


    recv_icp = (struct icmp *)&(pPkt->pBuffer[34]);

    if(recv_icp->icmp_type == ICMP_ECHO) {      /* Check if the packet type is ICMP_ECHO */
#ifdef PRINTF
        printf("\nICMP_ECHO packet received.");
#endif

        /*      The received packet buffer cannot be re-send.  SO, start allocating a new packet to send. */
        status = l2eth_allocate_packet(dwHandle, &Send_pPkt);
        if(L2ETH_OK != status) {
            printf("\nError in l2eth_allocate_packet and the error code is:%u", status);
        } else {
#ifdef PRINTF
            printf("\nSuccessfully allocated packet.");
#endif
        }


        /* Copying the entire packet contents to the newly allocated packet. */
        memcpy(&(Send_pPkt->pBuffer[0]), &(pPkt->pBuffer[0]), ((pPkt->DataLength) + 14));       /* 14 is added due to a bug in current version. */
        Send_pPkt->DataLength = (pPkt->DataLength) + 14;        /* Setting the Packet length. */


        /* Creating the Ethernet Header. */
        memcpy(&(Send_pPkt->pBuffer[0]), &(pPkt->pBuffer[6]), 6);       /* Copying the Destination MAC address. */
        memcpy(&(Send_pPkt->pBuffer[6]), &(pPkt->pBuffer[0]), 6);       /* Copying the Source MAC address. */


        /* Creating the IP header. */
        Recv_ip_hdr = (struct ipheader *)&pPkt->pBuffer[14];    /* 0-13: Ethernet header, 14 - Start of IP header */
        Send_ip_hdr = (struct ipheader *)&Send_pPkt->pBuffer[14];       /* 0-13: Ethernet header, 14 - Start of IP header */
        memcpy(&Send_ip_hdr->sourceip, &Recv_ip_hdr->destip, 4);        /* Copying the Source IP address */
        memcpy(&Send_ip_hdr->destip, &Recv_ip_hdr->sourceip, 4);        /* Copying the Destination IP address */
        Send_ip_hdr->chksum = 0;        /* Checksum field is made zero before calcutating checksum value. */
        Send_ip_hdr->chksum = checksum((unsigned short *)Send_ip_hdr, 20);      /* Calculates checksum and copies the value */


        /* Creating the ICMP header. */
        Send_icp = (struct icmp *)&(Send_pPkt->pBuffer[34]);    /* 0-13: Ethernet Header, 14-33: Ip Header, 34- ICMP Header Starts */
        recv_icp = (struct icmp *)&(pPkt->pBuffer[34]); /* 0-13: Ethernet Header, 14-33: Ip Header, 34- ICMP Header Starts */

        Send_icp->icmp_type = ICMP_ECHOREPLY;   /* Setting the packet type as ICMP_ECHOREPLY */
        Send_icp->icmp_cksum = 0x0000;  /* Checksum field is made zero before calcutating checksum value. */
        Send_icp->icmp_cksum = checksum((unsigned short *)Send_icp, ((pPkt->DataLength) - 20)); /* Calculates checksum and copies the value */


        /* Sending the ICMP_ECHOREPLY type packet. */
        status = l2eth_send(dwHandle, Send_pPkt);

        if ( L2ETH_OK != status ) {
            printf("\nError in putting ICMP_ECHO_REPLY packet header in DPR Lib and the return code is %d",
                   status);
        } else {
#ifdef PRINTF
            printf("\nSuccessfully put ICMP_ECHO_REPLY packet header in DPR Lib and the return code is %d",
                   status);
#endif
        }
    }

    else if(recv_icp->icmp_type == ICMP_ECHOREPLY) {    /* Check if the packet type is ICMP_ECHOREPLY */

#ifdef PRINTF
        printf("\nICMP_ECHO_REPLY packet received.");
#endif
        /* Not verifying the checksum of the received packet. */
        gettimeofday(&tm, NULL);                        /* Get the time of the day. */
        memcpy(&send_time, &(pPkt->pBuffer[46]), 4);    /* 46-The place where the time is copied while sending. */
        memcpy(recv_ip, &(pPkt->pBuffer[26]), 4);       /* Copying the destination IP address */
        memcpy(recv_mac, &(pPkt->pBuffer[6]), 6);

        printf("\nReply from IP:%d.%d.%d.%d and MAC:%02x.%02x.%02x.%02x.%02x.%02x: Bytes=%u Time:%ld micro-sec",
            recv_ip[0], recv_ip[1], recv_ip[2], recv_ip[3], recv_mac[0], recv_mac[1],
            recv_mac[2], recv_mac[3], recv_mac[4], recv_mac[5], ((pPkt->DataLength) - 28),
            (tm.tv_usec - send_time));
        flag = 1;
    }

    else {
#ifdef PRINTF
        printf("\nICMP type packet received but not ICMP_ECHO or ICMP_ECHOREPLY");
#endif
    }

    return T_SUCCESS;
}
/*========================================================================================*/



/*====================================================================================
* FUNCTION      : int Fill_Ping_Data ()
* DESCRIPTION   : This function will form a PING packet
* RETURNS       : SUCCESS/ERROR
*====================================================================================*/
int Fill_Ping_Data(L2ETH_PACKET * pPacket, unsigned char *dest_mac,
    unsigned char *dest_ip, unsigned int len)
{

    struct ipheader *ip;        /* Pointer to IP header. */
    struct icmp *recv_icp;      /* Pointer to ICMP header. */
    struct timeval *time;       /* Pointer to store the time value. */
    unsigned int i;
    unsigned char ch = 'a';


    memset(pPacket->pBuffer, 0, len + 42);      /* 14 - Ethernet Header, 20 - IP Header; 8 - ICMP hdr, Total 42 extra */
    pPacket->Context = global_long++;   /* Setting the packet context. */

    memcpy(pPacket->pBuffer, dest_mac, 6);      /* Copying the destination mac address into (0-5) bytes */

    /* Copying the source mac address into (6-11) bytes */
    status =
        get_MAC((unsigned char *)&pPacket->pBuffer[6], (unsigned char *)SOURCE_MAC, 18);
    if(status != T_SUCCESS) {
        printf("\nProblem in SOURCE_MAC");
        return T_ERROR;
    }

    pPacket->pBuffer[12] = 0x08;        /* 12-13, 0x0800 for IP datagram. */
    pPacket->pBuffer[13] = 0x00;


    ip = (struct ipheader *)&pPacket->pBuffer[14];      /* 14- Start of IP header. */

    ip->verlen = 0x45;          /* '4' is version type and '5' is len of header in words */
    ip->tos = 0x00;             /* Normal */
    ip->totlen = (((len + 28) & 0xff00) >> 8) | (((len + 28) & 0x00ff) << 8);   /*20 for IP header + 8 for ICMP header */
    ip->id = identity++;        /* Identification */
    ip->frag = 0x0000;          /* No fragmentation required */
    ip->ttl = 32;               /* Time to live is 32 hops */
    ip->prot = 0x01;            /* 1 for ICMP Protocol */
    ip->chksum = 0;             /* Checksum field is assigned to zero before calcuating the checksum. */
    ip->sourceip = inet_addr(SOURCE_IP);
    ip->destip = inet_addr((const char *)dest_ip);
    ip->chksum = checksum((unsigned short *)ip, 20);    /* Calculates checksum and assigns; 20-Length of IP header */

    recv_icp = (struct icmp *)&pPacket->pBuffer[34];    /* 34-Start of ICMP header. */

    recv_icp->icmp_type = ICMP_ECHO;    /* Setting packet type as ICMP_ECHO */
    recv_icp->icmp_code = 0;    /* Corresponding code to ICMP_ECHO is zero */
    recv_icp->icmp_cksum = 0;   /* Initially the checksum field is assigned zero to calculate
                                   the checksum */
    recv_icp->icmp_seq = ntransmitted++;        /* Assigining some value. */
    recv_icp->icmp_id = 0x9999; /* Setting the ICMP ID to 0x9999. */


    time = (struct timeval *)&pPacket->pBuffer[42];     /* Copying time value at 42 */
    gettimeofday(time, NULL);   /* Gets the current time of day. */

    for(i = 8; i < len; i++) {
        pPacket->pBuffer[42 + i] = ch++;        /* Filling the remaining data field with some value */
    }

    recv_icp->icmp_cksum = checksum((unsigned short *)recv_icp, len + 8);       /* Calculating the ICMP checksum; */

    pPacket->DataLength = len + 42;     /* Setting the Packet Length */

    return T_SUCCESS;
}
/*========================================================================================*/



/*===============================================================================================
* FUNCTION      : int get_MAC ()
* DESCRIPTION   : This function converts the MAC_ADDRESS in string format to required 6 byte format.
* RETURNS       : SUCCESS/ERROR
*===============================================================================================*/
int get_MAC(unsigned char *mac_s, unsigned char *mac_c, int length)
{

    int i, j = 0, curr;         /* To hold temporary values. */
    unsigned char mac[20];


    for(i = 0; i < length; i++) {       /* Remove '-' or '.' in MAC address. */

        if(mac_c[i] >= '0' && mac_c[i] <= '9')
            mac[j++] = (mac_c[i] - 48);

        else if(mac_c[i] >= 'A' && mac_c[i] <= 'F')
            mac[j++] = (mac_c[i] - 55);

        else if(mac_c[i] >= 'a' && mac_c[i] <= 'f')
            mac[j++] = (mac_c[i] - 87);

        else
            continue;

    }

    if(j != 12) {
        /* After deleting either '.' or '-', the MAC address should be 12 bytes. */
        printf("\nUser entered invalid MAC address.");
        return T_ERROR;
    } else {
        mac[j] = '\0';          /* Terminating the string and the string is now 12 bytes. */
    }

    i = 0;
    curr = 0;
    while(curr < j) {           /* Convert the 12byte string to 6 byte. */

        mac_s[i] = mac[curr++]; /* Copying byte by byte */
        mac_s[i] = mac_s[i] << 4;       /* Shifting the byte by 4 bits to upper nibble. */
        mac_s[i] = mac_s[i] | (mac[curr++] & 0x0000ffff);       /* Copying 4 bits of next byte to lower nibble. */
        i++;
    }

    return T_SUCCESS;
}
/*========================================================================================*/



/*========================================================================================
* FUNCTION      : unsigned short checksum ( unsigned short *, unsigned int )
* DESCRIPTION   : This function returns the checksum calculated for the data it received
* RETUNRS       : checksum value.
*========================================================================================*/
unsigned short checksum(unsigned short *ip1, unsigned int len)
{

    long sum = 0;

    while(len > 1) {
        sum = sum + *ip1++;
        if(sum & 0x80000000)
            sum = (sum & 0x80000000) + (sum >> 16);
        len -= 2;
    }

    while(sum >> 16)
        sum = (sum & 0xffff) + (sum >> 16);
    return ~sum;

}
/*========================================================================================*/



/*==============================================================================================================
* FUNCTION      : void user_callback_l2eth_send_complete ( L2ETH_UINT32 , L2ETH_PACKET *, L2ETH_UINT32 )
* DESCRIPTION   : This user callback is called by the interface to return the send buffer and send status to
*                 the user application.
* RETURNS       : NA
*==============================================================================================================*/
void user_callback_l2eth_send_complete(L2ETH_UINT32 dwHandle, L2ETH_PACKET * pPkt,
    L2ETH_UINT32 error)
{

    /* Free the received packet. */
    status = l2eth_free_packet(dwHandle, (L2ETH_PACKET *) pPkt);
    if(L2ETH_OK != status) {
        printf("\nError in freeing the packet.");
    }

}
/*==============================================================================================================*/



/*====================================================================================
* FUNCTION      : void user_close ( int sig )
* DESCRIPTION   : This function user_close the channel before exiting the program.
* RETURNS       : NA
*====================================================================================*/
void user_close(int sig)
{

    if(dwHandle != 0) {         /* Check if channel is opened or closed. 0 - closed. */

        if(mode_flag != L2ETH_OFFLINE) {        /* Check if the mode is online or offline. */
            status = l2eth_set_mode(dwHandle, L2ETH_OFFLINE);
            if(status != L2ETH_OK) {
                printf("\nUnable to set the mode to online and the return code is %x",
                    status);
                exit(1);
            }
        }

        /* Before calling "l2eth_close" we need to wait for the callback  "user_callback_l2eth_mode_complete_cbf"
           But in this sample,for simplicity this is not synchronized, so "sleep" for 2 sec. is used. */

        sleep(2);

        /* Now the mode is offline and calling the close function. */

        status = l2eth_close(dwHandle);
        if(status != L2ETH_OK) {
            printf("\nFailed to close the channel and the return code is %x", status);
            exit(1);
        } else {
            dwHandle = 0;       /* Reset the open channel flag. */
#ifdef PRINTF
            printf("\nSuccessully closed the channel and the return code is %x", status);
#endif
        }
    }

    exit(0);                    /* Exit the program. */
}
/*=============================================================================================*/



/*===========================================================================================================
* FUNCTION      : void user_callback_l2eth_mode_complete_cbf ( L2ETH_UINT32 , L2ETH_MODE , L2ETH_UINT32 )
* DESCRIPTION   : This user callback is called by the L2 host interface to intimate the changed mode.
* RETURNS       : NA
*===========================================================================================================*/
void user_callback_l2eth_mode_complete_cbf(L2ETH_UINT32 handle, L2ETH_MODE mode,
    L2ETH_UINT32 error)
{


    if(mode == L2ETH_ONLINE) {
#ifdef PRINTF
        printf("\n>>L2 Test:user_callback_l2_ethernet_mode_complete_cbf and the mode is online");
#endif
        mode_flag = L2ETH_ONLINE;
    } else if(mode == L2ETH_OFFLINE) {
#ifdef PRINTF
        printf("\n>>L2 Test:user_callback_l2_ethernet_mode_complete_cbf and the mode is offline");
#endif
        mode_flag = L2ETH_OFFLINE;
    } else {
#ifdef PRINTF
        printf("\n>>L2 Test:user_callback_l2_ethernet_mode_complete_cbf and the mode is Invalid");
#endif
    }

}
/*=============================================================================================*/



/*==============================================================================================
* FUNCTION      : void user_callback_l2eth_status_indication( L2ETH_UINT32      handle, L2ETH_OID )
* DESCRIPTION   : This user callback is called by the interface to intimate the changed status.
* RETURNS       : NA
*=============================================================================================*/
void user_callback_l2eth_status_indication(L2ETH_UINT32 handle, /* in */
    L2ETH_OID oid               /* in */
    )
{

    if(oid == L2ETH_OID_MEDIA_CONNECT_STATUS) {
#ifdef PRINTF
        printf
            ("\n>>L2 Test:user_callback_l2_ethernet_status_indication and the media connect status has got changed.");
#endif
    } else {
#ifdef PRINTF
        printf("\n>>L2 Test:user_callback_l2_ethernet_status_indication and invalid oid");
#endif
    }

}
/*=============================================================================================*/



/*=============================================================================================
* FUNCTION      : int user_callback_l2eth_receive_indication ( L2ETH_UINT32 , L2ETH_PACKET * )
*
* DESCRIPTION   :
*
* This user callback is called by the interface to pass a receive buffer to the
* user application.  This buffer has to be returned to the interface
* using l2_ethernet_return_packet().
*
* NOTE:-If user do not want to consume a packet, he can return "0" from this function. In this case
* he do not have to call "l2eth_return_packet", the library underneath will take care.
* In case user wants to use the packet, he should call "l2eth_return_packet" after he has used it
* but should return "1" from this function.
*=============================================================================================*/
int user_callback_l2eth_receive_indication(L2ETH_UINT32 dwHandle, L2ETH_PACKET * pPkt)
{


    unsigned char source_mac[6];
    int length;

#ifdef PRINTF
    printf("\nThe received packet length is %d", pPkt->DataLength);
#endif

    length = sizeof (SOURCE_MAC);
    status = get_MAC(source_mac, (unsigned char *)SOURCE_MAC, length);  /* Converts the MAC address in string format to required 6 byte format */
    if(status != L2ETH_OK) {
        printf("\nError returned by get_MAC.");
    } else {

        status = memcmp(&(pPkt->pBuffer[0]), source_mac, 6);    /* Check if the packet is send to this machine or a broadcast one. */

        if(status == 0) {       /* 0 - The packet is send to this machine only and not a broadcast. */

            if(pPkt->pBuffer[23] == 0x01) {     /* Check the received packet is ICMP type or not. */
                l2eth_ping_reply(dwHandle, pPkt);       /* Call the function to handle ICMP type packet sent to this machine. */
            }
        }
    }

    status = l2eth_return_packet(dwHandle, pPkt);       /* Call the function to return the packet */
    if(status != L2ETH_OK) {
        printf("\nError in return_packet and the return code is %x", status);
    } else {
#ifdef PRINTF
        printf("\nSuccess and the return code is %x", status);
#endif
    }

    /* We have used the packet, and we have called "l2eth_return_packet" so return "1" */
    return 1;

}
/*=============================================================================================*/



/*===========================================================================
* FUNCTION      : int main()
* DESCRIPTION   : This is the main routine of the program.
* RETURNS       : SUCCESS/FAIL
*==========================================================================*/
int main(int argc, char *argv[])
{                               /* argc - Number of arguments passed to main function including the program name. */

    L2ETH_PACKET *pPkt;         /*      Pointer to L2 packet. */
    unsigned char *dest_MAC = NULL;     /*      To store destination MAC address. */
    unsigned char *dest_IP = NULL;      /*      To store destination IP address. */
    char *temp, *stopstring;    /*      Used to hold temporary character array addresses. */
    unsigned char MAC_Addr[6];  /*      To store 6 byte destination MAC address. */
    int count = 4;              /*      To store the default number of ping packets to send. */
    int length = 500;           /*      To store the default length of each packet. */
    int slength;                /*      To store the user entered length of each packet. */
    int ip_mac = 0;             /*      Flag to see whether the user entered IP and MAC or not. */
    int i;



    signal(SIGINT, user_close); /*      Catches the CTRL+C signal and calls user_close function to close the channel and exit the program. */

    if(argc < 5 || argc > 9) {  /*      Program needs atleast IP and MAC addresses. So, the number of arguments should be atleast 5. */
        printf("\nSyntax: -i xxx.xxx.xxx.xxx -m xx.xx.xx.xx.xx.xx -n x -l x\n");
    } else {

        i = 1;
        while(i < argc) {       /*      Read the user inputs. */

            if(*argv[i] == '-') {

                switch (*(++(argv[i]))) {       /* If the argument starts with '-', get the immediated character/choice. */

                case 'i':
                    dest_IP = (unsigned char *)argv[++i];       /* Gets the next argument. */
                    ip_mac++;   /*      To check whether the user entered both IP and MAC later. */
                    break;

                case 'm':
                    dest_MAC = (unsigned char *)argv[++i];      /* Gets the next argument. */
                    ip_mac++;   /*      To check whether the user entered both IP and MAC later. */
                    break;

                case 'n':
                    temp = argv[++i];   /* Gets the next argument. */
                    count = (int)strtod(temp, &stopstring);     /* Converts the string format value to decimal equivalent until it come across an alphabet/NULL. */
                    break;

                case 'l':
                    temp = argv[++i];   /* Gets the next argument. */
                    length = (int)strtod(temp, &stopstring);    /* Converts the string format value to decimal equivalent until it come across an alphabet/NULL. */
                    break;
                }

                i++;
            } else {            /* If your enters invalid syntax, print the syntax. */

                printf("\n%d ", i);
                printf("\nSyntax: -i xxx.xxx.xxx.xxx -m xx.xx.xx.xx.xx.xx -n x -l x\n");
                return T_SUCCESS;
            }

        }

        if(ip_mac != 2) {       /*      Check if user entered both IP and MAC address.  If entered, the value will be 2. */
            printf
                ("\nTo complete the operation, user must enter IP and MAC address. See the syntax for reference.");
            printf("\nSyntax: -i xxx.xxx.xxx.xxx -m xx.xx.xx.xx.xx.xx -n x -l x\n");
            user_close(0);      /* Closes the channel and exit the program. */
        }

        /* Verify the packet length is in acceptable range or not. */
        if(length > LENGTH_OF_PACKET) { /*      Check if Packet length is greater than maximum allowed. */
            printf("\nThe packet length is greater than allowed size.");
            return T_SUCCESS;
        } else if(length < 8) { /*      8 bytes are required to included the current time of day. */
            printf("\nThe packet length is less than required.");
            return T_SUCCESS;
        }



        /* Open the channel to send the packets. */
        status = l2eth_open( /*in */ CP_INDEX,
                        /*in */ user_callback_l2eth_receive_indication, /* receive callback  */
                        /*in */ user_callback_l2eth_send_complete,      /* send callback     */
                        /*in */ user_callback_l2eth_status_indication,  /* status callback   */
                        /*in */ user_callback_l2eth_mode_complete_cbf,  /* Mode call back    */
                        /*out */ &dwHandle                                                              /* handle            */
            );

        if(status != L2ETH_OK) {
            /* If any error occurs while opening the channel, print the error message and error code. */
            printf("\nFailed to open the channel and the return code is %x, handle %u",
                   status, dwHandle);
            user_close(0);      /* Closes the channel and exit the program. */
        } else {
#ifdef PRINTF
            printf("\nSuccessfully opened channel and the handle is %u", dwHandle);
#endif
        }


        status = l2eth_set_mode(dwHandle, L2ETH_ONLINE);        /* Set the mode to online to send packets. */
        if(status != L2ETH_OK) {
            printf("\nUnable to set the mode to online and the return code is %x",
                status);
            user_close(0);      /* Exit the program. */
        }

        slength = strlen((const char *)dest_MAC);       /* Gets the length of user entered MAC address. */

        status = get_MAC(MAC_Addr, dest_MAC, slength);  /* Converts the MAC address to required 6 byte format */
        if(status != T_SUCCESS) {
            printf("\nget_MAC returned error.");
            user_close(0);      /* Closes the channel and exit the program. */
        }

        printf("\nPinging host with IP %s and MAC %s with %u bytes of data.\n", dest_IP,
            dest_MAC, length);

        sleep(1);

        for(i = 0; i < count; i++) {

            flag = 0;           /* Reset the flag on each iteration. */

            /* Start allocating the packet. */
            status = l2eth_allocate_packet(dwHandle, &pPkt);    /* Allocate a packet to send. */
            if(L2ETH_OK != status) {
                printf("\nError in l2eth_allocate_packet and the error code is:%x",
                    status);
                return T_SUCCESS;
            }


            status = Fill_Ping_Data(pPkt, MAC_Addr, dest_IP, length);   /* Turns the packet into ping packet type. */
            if(status != T_SUCCESS) {
                printf("\nError returned by Fill_Ping_Data");
                user_close(0);  /* Exit the program. */
            }


            status = l2eth_send(dwHandle, pPkt);        /* Sends a packet. */
            if(L2ETH_OK != status) {
                printf
                    ("\nError in putting the packet header in DPR Lib and the return code is %x",
                    status);
                user_close(0);  /* Exit the program. */

            } else {
#ifdef PRINTF
                printf("\nSuccessfully put the packet header in DPR Lib.");
#endif
            }

            sleep(2);

            if(flag == 0) {     /*      In 2 seconds, if ping reply packet comes then the flag will be set to 1. If not, the packet is not received. */
                printf("\nRequest timed out.");
            }
        } /* end for loop */
    }

    printf("\nNow the mode is online and accepts ICMP_ECHO request.");

    printf("\nPress anykey and enter to close the channel and exit:");
    scanf("%d", &i);

    user_close(0);              /* Closes the channel and exit the program. */

    return T_SUCCESS;
}
/*===========================================================================*/
/* End - Main Program */
/*===========================================================================*/
