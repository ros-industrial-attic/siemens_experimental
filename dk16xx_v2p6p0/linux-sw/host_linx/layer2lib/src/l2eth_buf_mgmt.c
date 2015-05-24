/*****************************************************************************/
/*  Copyright (C) SIEMENS CORP., 2013 All rights reserved.*/
/*****************************************************************************/
/*  F i l e                l2eth_buf_mgmt.c                                  */
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
#include "l2eth_defs.h"
#include "l2eth_buf_mgmt.h"

/************************************************************************************/
/*                                                                                  */
/************************************************************************************/
L2ETH_BUF_ERROR l2_buf_init(L2ETH_BUFFER_DISCRIPTOR buf_disc,   /* [in] */
                            L2ETH_UINT32 prealloc_cnt,          /* [in] */
                            L2ETH_BUFFER_CONFIG ** hBuf)        /* [out] */
{
    L2ETH_BUF_ERROR errorCode = L2ETH_BUF_SUCCESS;
    L2ETH_BUFFER_CONFIG *bufConfig;

    L2ETH_UINT32 cfgSize;
    L2ETH_UINT32 cfgSizeExt;

    L2ETH_UINT8 extraBitsInEnd;
    L2ETH_UINT8 avail_flag_array_size;

    L2ETH_UINT32 i;             /* counter */
    L2ETH_UINT32 quotient, remainder;

    /* Check the validity of buffer discriptor */
    if(buf_disc.base_address == 0) return L2ETH_BUF_INVALID_PARAM;

    /*
       It is caller's responsibility to make sure that he has allocated sufficient memory
       to accomodate all the buffers.
     */

    /* Create a buffer configuration structure */

    avail_flag_array_size = (L2ETH_UINT8) (buf_disc.buf_cnt / 32); /* number of DWORD elements required in an array */
    cfgSizeExt = avail_flag_array_size * 4; /* extra space required for the avail_flag_array */

    extraBitsInEnd = (L2ETH_UINT8) (buf_disc.buf_cnt % 32);

    if(0 != extraBitsInEnd) {
        avail_flag_array_size++;

        cfgSizeExt += 4; /* extra DWORD for accommodating the remaining bits' (remainder of /32) */
    }

    cfgSize = sizeof (L2ETH_BUFFER_CONFIG) /* Size of basic buffer config stucture */
        +(cfgSizeExt - 4 ) /* Additional size taken by availability flags - 4 bytes are allocated inside the structure */
        + (sizeof (L2ETH_PACKET) * buf_disc.buf_cnt);   /* Additional size taken by packets      */

    bufConfig = (L2ETH_BUFFER_CONFIG *) DPR_ZALLOC(cfgSize * sizeof (L2ETH_UINT8)); /* size-1, becouse 1 location for flags is already
                                                                                   allocated within config structure */
    if(bufConfig == 0) return L2ETH_BUF_INVALID_PARAM;

    /* Fill the configuration structure */
    memcpy(&bufConfig->bd, &buf_disc, sizeof (L2ETH_BUFFER_DISCRIPTOR));

    /* set the flag array size */
    bufConfig->avail_flag_array_size = avail_flag_array_size;

    /* Point the pParamBlock to its address in memory. */
    bufConfig->pPacketBlock =
        (L2ETH_PACKET *) ((L2ETH_UINT8 *) bufConfig +
        (sizeof (L2ETH_BUFFER_CONFIG) + (cfgSizeExt - 4)));

    /*
       The unused bits in the last element of avail_flag_array MUST be set to 'Full' so that they are not used.
       Check if there are any extra bits
     */
    if(0 != extraBitsInEnd) {
        L2ETH_UINT32 bufFullMask = 0xFFFFFFFF << extraBitsInEnd;        /* 1 1 1 1 1 <--... 0 0 0 */

        bufConfig->avail_flag[bufConfig->avail_flag_array_size - 1] = bufFullMask;      /* last element of the array */
    }

    /*
       There is a fixed relationship between single buffer and packet. One buffer is permenantly linked
       with one packet. The packet contains pointer to the buffer. Create the packet-buffer relationship by
       assigning the buffers into packets.
     */
    for(i = 0; i < bufConfig->bd.buf_cnt; i++) {
        bufConfig->pPacketBlock[i].pBuffer =
            (L2ETH_UINT8 *) (bufConfig->bd.base_address + (bufConfig->bd.buf_size * i));
    }

    /*
       Pre-allocation of buffers : At the beginning, a number of buffers can be marked as 'used' straightaway.
       These buffers are allocated contigeously at the beginning.
     */

    /* check for the param */
    if(prealloc_cnt > bufConfig->bd.buf_cnt) {
        errorCode = L2ETH_BUF_INVALID_PARAM;
        DPR_FREE(bufConfig);
        return errorCode;
    }

    quotient = prealloc_cnt / 32;
    remainder = prealloc_cnt % 32;
    for(i = 0; i < quotient; i++) {
        bufConfig->avail_flag[i] = 0xFFFFFFFF;
    }

    for(i = 0; i < remainder; i++) {
        bufConfig->avail_flag[quotient] |= (0x00000001 << i);
    }

    /* Fill the buffers with default characters */
    memset(bufConfig->bd.base_address, bufConfig->bd.def_fill,
        (bufConfig->bd.buf_cnt * bufConfig->bd.buf_size));

    /* return the pointer as handle */
    *hBuf = bufConfig;

    return errorCode;
}

/************************************************************************************/
/*                                                                                  */
/************************************************************************************/
L2ETH_BUF_ERROR l2_buf_remove(L2ETH_BUFFER_HANDLE *hBuf) /* [in] */
{
    /* Check the validity of buffer discriptor */
    if(hBuf == 0)
        return L2ETH_BUF_INVALID_PARAM;

    DPR_FREE(*hBuf);

    return L2ETH_BUF_SUCCESS;
}

/************************************************************************************/
/*                                                                                  */
/************************************************************************************/
L2ETH_BUF_ERROR l2_buf_get_next(L2ETH_BUFFER_HANDLE hBuf,       /* [in] */
                                L2ETH_PACKET ** pNext)       /* [out] */
{
    L2ETH_BUF_ERROR errorCode = L2ETH_BUF_SUCCESS;

    /* hBuf locking is done by the interface implementation file l2eth_user.c */
    L2ETH_BUFFER_CONFIG *bufConfig = (L2ETH_BUFFER_CONFIG *)hBuf;
    L2ETH_UINT32 avail_flag_array_index;
    L2ETH_UINT32 avail_flag32;

    /*
       Go through the availability flag array, to check for free buffers
       Advantage of using combination of bit-wise operators with array of DWORD is that,
       32 buffers can be checked for availability in single operation
     */

    L2ETH_UINT32 free_buf_bit_no = 0;

    for(avail_flag_array_index = 0;
        avail_flag_array_index < bufConfig->avail_flag_array_size;
        avail_flag_array_index++) {
        avail_flag32 = bufConfig->avail_flag[avail_flag_array_index];

        if((avail_flag32 & 0xFFFFFFFF) == 0xFFFFFFFF) { /* all buffers are filled */
            /* go to next set of 32 buffers */
            continue;
        }

        /*
           binary search for free buffer
         */

        /* Word comparison */
        {
            L2ETH_UINT32 avail_flag16 = (L2ETH_UINT16) (avail_flag32 & 0x0000FFFF);        /* lower word */

            if(avail_flag16 == 0x0000FFFF) {    /* lower word is full */
                avail_flag16 = (L2ETH_UINT16) ((avail_flag32 & 0xFFFF0000) >> 16);        /* go for higher word */
                free_buf_bit_no += 16;
            }

            /* Byte comparison */
            {
                L2ETH_UINT8 avail_flag8 = (L2ETH_UINT8) (avail_flag16 & 0x00FF);  /* lower byte */

                if(avail_flag8 == 0x00FF) {     /* lower byte is full */
                    avail_flag8 = (L2ETH_UINT8) ((avail_flag16 & 0xFF00) >> 8);  /* go for higher byte */
                    free_buf_bit_no += 8;
                }

                /* Nibble comparison */
                {
                    L2ETH_UINT8 avail_flag4 = avail_flag8 & 0x0F;        /* lower nibble */

                    if(avail_flag4 == 0x0F) {   /* lower nibble is full */
                        avail_flag4 = (avail_flag8 & 0xF0) >> 4;        /* go for higher nibble */
                        free_buf_bit_no += 4;
                    }

                    /* two bit comparison */
                    {
                        L2ETH_UINT8 avail_flag2 = avail_flag4 & 0x03;    /* lower two bits */

                        if(avail_flag2 == 0x03) {       /* lower two bits are full */
                            avail_flag2 = (avail_flag4 & 0x0C) >> 2;    /* go for higher two bits */
                            free_buf_bit_no += 2;
                        }

                        /* one bit comparison */
                        {
                            L2ETH_UINT8 avail_flag1 = avail_flag2 & 0x01;        /* lower bit */

                            if(avail_flag1 == 0x01) {   /* lower bit is full */
                                avail_flag1 = (avail_flag2 & 0x02) >> 1;        /* go for higher bit */
                                free_buf_bit_no += 1;
                            }

                        }       /*1 */
                    }           /*2 */
                }               /*4 */
            }                   /*8 */
        }                       /*16 */

        /* mark the newly found free buffer as 'full' */
        bufConfig->avail_flag[avail_flag_array_index] |= 0x01 << free_buf_bit_no;

        break;                  /* exit the loop as free buffer is already found out */
    }

    if(avail_flag_array_index == bufConfig->avail_flag_array_size) {    /* all buffers are full */
        *pNext = 0x00000000;

        errorCode = L2ETH_BUF_NO_RESOURCES;
    } else {

        /* calculate next free buffers address based on the free_buf_bit_no */

        L2ETH_UINT32 next_buf_index = avail_flag_array_index * 32 + free_buf_bit_no;

        *pNext = bufConfig->pPacketBlock + next_buf_index;

        errorCode = L2ETH_BUF_SUCCESS;
    }

    return errorCode;
}

/************************************************************************************/
/*                                                                                  */
/************************************************************************************/
L2ETH_BUF_ERROR l2_buf_free(L2ETH_BUFFER_HANDLE hBuf,   /* [in] */
                            L2ETH_PACKET * pNext)        /* [in] */
{
    L2ETH_BUF_ERROR errorCode = L2ETH_BUF_SUCCESS;

    /* hBuf locking is done by the interface implementation file l2eth_user.c */
    L2ETH_BUFFER_CONFIG *bufConfig = (L2ETH_BUFFER_CONFIG *) hBuf;

    L2ETH_UINT32 next_buf_index = (L2ETH_UINT32)(pNext - bufConfig->pPacketBlock);

    L2ETH_UINT32 avail_flag_array_index = next_buf_index / 32;

    L2ETH_UINT32 free_buf_bit_no = next_buf_index % 32;

    /* mark the buffer as 'free' */
    bufConfig->avail_flag[avail_flag_array_index] &= ~(0x01 << free_buf_bit_no);

    return errorCode;
}

/************************************************************************************/
/*                                                                                  */
/************************************************************************************/
L2ETH_UINT32 l2_buf_get_total_cnt(L2ETH_BUFFER_HANDLE hBuf) /* [in] */
{
    L2ETH_BUFFER_CONFIG *bufConfig = (L2ETH_BUFFER_CONFIG *) hBuf;

    return bufConfig->bd.buf_cnt;
}

/************************************************************************************/
/*                                                                                  */
/************************************************************************************/
L2ETH_UINT32 l2_buf_get_free_cnt(L2ETH_BUFFER_HANDLE hBuf, /* [in] */
                                 L2ETH_UINT32 bits_cnt)  /* [in] */
{
    L2ETH_BUFFER_CONFIG *bufConfig = (L2ETH_BUFFER_CONFIG *) hBuf;

    L2ETH_UINT32 freeCnt = 0, i, j, k;

    for(i = 0; i < (bufConfig->avail_flag_array_size - 1); ++i) {
        if(!bufConfig->avail_flag[i]) {
            freeCnt += 32;
        } else {
            for(j = 0; j < 32; ++j) {
                if(!(bufConfig->avail_flag[i] & (0x1 << j)))
                    ++freeCnt;
            }
        }
    }

    k = bits_cnt % 32;
    for(j = 0; j < k; ++j) {
        if(!(bufConfig->avail_flag[i] & (0x1 << j)))
            ++freeCnt;
    }

    return freeCnt;
}

/************************************************************************************/
/*                                                                                  */
/************************************************************************************/
L2ETH_BUF_ERROR l2_buf_get_packet(L2ETH_BUFFER_HANDLE hBuf,    /* [in] */
                                  L2ETH_UINT8 * pBuf,          /* [in] */
                                  L2ETH_PACKET ** ppPacket)    /* [out] */
{

    L2ETH_BUF_ERROR errorCode = L2ETH_BUF_SUCCESS;
    L2ETH_BUFFER_CONFIG *bufConfig = (L2ETH_BUFFER_CONFIG *) hBuf;

    L2ETH_UINT32 buf_index;


    /* Gives the index of the requested buffer */
    buf_index = ((L2ETH_UINT32) (pBuf - bufConfig->bd.base_address)) / (bufConfig->bd.buf_size);

    *ppPacket = bufConfig->pPacketBlock + buf_index;

    return errorCode;
}

