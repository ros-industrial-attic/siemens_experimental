/*****************************************************************************/
/*  Copyright (C) SIEMENS CORP., 2013 All rights reserved.*/
/*****************************************************************************/
/*  F i l e                dprintern.c                                       */
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

#include "os.h"             /* system dependencies */
#include "dprintern.h"      /* common layer for FW and driver */

#ifdef _DPR_FW
        extern void SYS_ISR_IrqEnable(int t);
        extern void SYS_ISR_IrqDisable(int t);

        #if !defined(USE_OS_POSIX)
                #include "fw1616dk_vers.h"  /* fw version constants FW_VER_MAJOR, FW_VER_MINOR1... */
        #else
                EXTERN void mgt_read_fw_version_parts(int* major, int* minor1, int* minor2, int* buildno);
        #endif /* USE_OS_POSIX */

        #include "sysa_telnettrc.h"

#else
        #include "fw1616dk_vers.h"  /* fw version constants FW_VER_MAJOR, FW_VER_MINOR1... */
        #include "driver_vers.h"    /* driver version constants */
#endif



/*===========================================================================
* FUNCTION : dprlib_align
*----------------------------------------------------------------------------
* PURPOSE  : for alignning size to 4 bytes
*----------------------------------------------------------------------------
* RETURNS  : aligned size
*----------------------------------------------------------------------------
* INPUTS   : Size
* OUTPUTS  :
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
unsigned long dprlib_align(unsigned long Size)
{
    while( Size&0x3 )
        Size++;
    return Size;
}

/*===========================================================================
* FUNCTION : dprlib_raw_write
*----------------------------------------------------------------------------
* PURPOSE  : writes a message to a ring. the function can handle the overrun
*            of the rings
*----------------------------------------------------------------------------
* RETURNS  : the next free position to write
*----------------------------------------------------------------------------
* INPUTS   : iWritePos - Position to write
*            iReadPos - Readpointer, must be checked
*            iRingStart - Start of the ring
*            iRingEnd - End of the ring
*            data - pointer to the data to be written
*            dataSize - size of the data to be written
* OUTPUTS  :
*----------------------------------------------------------------------------
* COMMENTS : there is no check if the message fits between read- and writeptr
*            the calling function must do these checks
*==========================================================================*/
static DPR_CHAR *dprlib_raw_write(DPR_CHAR *iWritePos, DPR_CHAR *iReadPos,
    DPR_CHAR *iRingStart, DPR_CHAR *iRingEnd, DPR_CHAR *data, DPR_UINT32 dataSize)
{
    /* there are two cases in writing: first case is that the Read_pointer is behind the */
    /* write pointer. In this case we have to check if the message will be split around  */
    /* ring*/
    if(iReadPos <= iWritePos) {
        /* message fits complete in the remaining place */
        if(iWritePos+dataSize <= iRingEnd) {
            DPR_MEMCPY_TO_PCI((DPR_CHAR *)iWritePos, data, dataSize);
            iWritePos += dataSize;
            if (iWritePos == iRingEnd) {
                iWritePos = iRingStart;
            }
        } else {
            /* message must be split: one part to the end of the ring, other part at the start of the ring */
            DPR_MEMCPY_TO_PCI((DPR_CHAR *)iWritePos, data, (int)(iRingEnd - iWritePos));
            DPR_MEMCPY_TO_PCI((DPR_CHAR *)iRingStart, &data[(int)(iRingEnd - iWritePos)],
                dataSize - (int)(iRingEnd - iWritePos));
            iWritePos = iRingStart + (dataSize - (int)(iRingEnd - iWritePos));
        }
    } else {
        /* second case: Readpointer is behind writepointer,
           so the message must fit between them */
        DPR_MEMCPY_TO_PCI((DPR_CHAR *)iWritePos, data, dataSize);
        iWritePos += dataSize;
    }
    return iWritePos;
}

/*===========================================================================
* FUNCTION : dprlib_raw_read
*----------------------------------------------------------------------------
* PURPOSE  : reads a message from the ring
*----------------------------------------------------------------------------
* RETURNS  : pointer to the next message only if the message is 4 byte aligned !!!
*            If NOT points to the 1st fill byte of 1-3 alignment bytes !!!
*----------------------------------------------------------------------------
* INPUTS   : iWritePos - writepointer
*            iReadPos - Readpointer
*            iRingStart/End - Start/End of the ring
*            data - pointer to the receive buffer
*            dataSize - size of the data to be read
* OUTPUTS  :
*----------------------------------------------------------------------------
* COMMENTS : there is no memory allocation, must be done by the calling function.
*            All pointers are real pointers => no problem with 64 bit
*==========================================================================*/
static DPR_CHAR *dprlib_raw_read(DPR_CHAR *iWritePos, DPR_CHAR *iReadPos,
    DPR_CHAR *iRingStart, DPR_CHAR *iRingEnd, DPR_CHAR *data, DPR_UINT32 dataSize)
{
    /* there are two cases in writing: first case is that the Read_pointer
       is behind the write pointer. In this case we can copy the message */
    if(iReadPos < iWritePos) {
        DPR_ASSERT(iReadPos + dataSize <= iWritePos);
        if(data) {
            DPR_MEMCPY_FROM_PCI(data, (DPR_CHAR *)iReadPos, dataSize);
        }
        iReadPos += dataSize;
    } else {
        /* read pointer is in front of write pointer, so check for ring overrun */
        DPR_ASSERT(iReadPos != iWritePos);
        if(iReadPos+dataSize < iRingEnd) {
            /* complete message can be copied with one call */
            if(data)
                DPR_MEMCPY_FROM_PCI(data, (DPR_CHAR *)iReadPos, dataSize);
            iReadPos += dataSize;
        } else {
            /* ring overrun. Message must be split in two parts */
            long dif = (long)(iRingEnd - iReadPos);
            long rest = dataSize - dif;
            if(data) {
                DPR_MEMCPY_FROM_PCI(data, (DPR_CHAR *)iReadPos, dif);
                DPR_MEMCPY_FROM_PCI(&data[dif], (DPR_CHAR *)iRingStart, rest);
            }
            iReadPos = iRingStart + rest;
        }
    }
    return iReadPos;
}

/*===========================================================================
* FUNCTION : dprlib_write_message_to_ring
*----------------------------------------------------------------------------
* PURPOSE  : 1.Write message to a ring
*            2.Get the ring info from configuration structure
*            3.Comapre readPtr, writePtr, endPtr
*            4.Write msg to memory location
*            5.Modify WritePtr, end ptr
*----------------------------------------------------------------------------
* RETURNS  : int
*----------------------------------------------------------------------------
* INPUTS   : RingIndex, message, message length
* OUTPUTS  :
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
int dprlib_write_message_to_ring(CpData * pCP, DPR_UINT32 RingIndex,
    DPR_UINT32 handle, void * msg, unsigned long msgSize)
{
    DPR_UINT32 ChIndex;
    int WriteFinished = 0;
    int CanWriteNow;
    DPR_CHAR *ringStart;
    DPR_CHAR *ringEnd;
    DPR_CHAR *readPos;
    DPR_CHAR *writePos;
    RING *pRing;
    DPR_UINT32 writeSize;
    DPR_MSG_HEAD msgHead;
    int ringFullCount  = 0;  // current polling counter


#ifdef _DPR_FW
    TelnetTrcDprWriteMsgToRing(RingIndex/2, handle, msg, msgSize);
#endif

    pRing = &(pCP->pDpramConfigArea->Rings[RingIndex]);

    /* get the channel index of ring to get writelock */
    ChIndex = RingIndex/2;

    msgHead.MsgLength = msgSize + sizeof(DPR_MSG_HEAD);
    msgHead.FctHandle = handle;

    /* compute the length wich will be written to the ring buffer. In some cases, the comparison */
    /* between size and writepos can be wrong when the size would be compared unaligned */
    writeSize = dprlib_align(msgHead.MsgLength);

   /* continue trying till WriteLock  is available
    * PL:??? comment seems to be wrong. We are here looping until there is enough space
    * in the ring to write the given message.
    */
    while(!WriteFinished) {
        if(DPR_ATOMIC_READ(pCP->dprStopAllThreads)) {
            DPRLIBLOGMSG("dprStopAllThreads - exit\n");
            return DPR_ERROR;
        }

        /* check if ring initialized or not */
        if(0 == DPR_READ_UINT32(&pRing->pRingStart)) {
            DPRLIBERRMSG("Ring %d not initialized, pRingStart is NULL\n", RingIndex);
            return DPR_ERROR;
        }

        /* wait only 1st time - not if we are looping for ring-free
         */
        if ( ringFullCount == 0 ) {
            DPR_SEM_WAIT(pCP->ChannelDatas[ChIndex].bsemWriteLock);
        }

        /* make copy of original pointers */
        ringStart = (DPR_CHAR *) GETADDR(DPR_READ_UINT32(&pRing->pRingStart));
        ringEnd   = (DPR_CHAR *) GETADDR(DPR_READ_UINT32(&pRing->pRingEnd));
        readPos   = (DPR_CHAR *) GETADDR(DPR_READ_UINT32(&pRing->pRead));
        writePos  = (DPR_CHAR *) GETADDR(DPR_READ_UINT32(&pRing->pWrite));

        /* check if msg is bigger than ring */
        if( writeSize >= (DPR_UINT32)(ringEnd - ringStart)) {
            DPRLIBERRMSG("Ring %d not initialized, message is bigger than ring\n", RingIndex);
            DPR_SEM_POST(pCP->ChannelDatas[ChIndex].bsemWriteLock);
            return DPR_ERROR;
        }

        /* check if there is place enough to write the message. if not, set CanWriteNow to false */
        if ((writePos < readPos) && (writeSize >= (DPR_UINT32)(readPos-writePos))) {
            CanWriteNow = 0;
        } else if ((writePos >= readPos) &&
            ((DPR_UINT32)(ringEnd - writePos + readPos - ringStart) <= writeSize)) {
            CanWriteNow = 0;
        } else {
            CanWriteNow = 1;
        }

       /* ring is full, we can't write now, we try it again in the loop
        * Note: PL: because of problems to get a "ring free" interrupt  to wake up this thread,
        * we are polling for a free space in the ring ("non busy wait" - sleep)
        *
        */
        if(!CanWriteNow) {
            ringFullCount++;

            // signal to reader that the ring is full, set flag in the control structure
            DPR_WRITE_UINT32(DPR_RING_FULL, &pRing->RingFull);

            if ( ringFullCount < 50 ) {
                /* wait a short time 1st */
                DPR_TASK_DELAY(20); /* millisecs */
                /* release write lock for other writers */
                //DPR_SEM_POST(pCP->ChannelDatas[ChIndex].bsemWriteLock);
                continue;
            }
            else if ( (ringFullCount >= 50) && (ringFullCount < 150) ) {
                /* to be sure that the reader got a "message interrupt" - generate one again */
                if ( ringFullCount == 50 ) {
                    DPR_WRITE_UINT32(DPR_INT_MESSAGE, &pCP->pDpramConfigArea->HPIMessage);
                    dprlib_trigger_intr(pCP);
                }

                /* host must be very busy - wait longer */
                DPR_TASK_DELAY(100); /* millisecs */
                continue;
            }
            else {
               /* timeout ERROR: reader is not reading, we assume that reader hangs,
                *  and no information can be transferd over that ring -> exit with error
                */
                DPRLIBERRMSG("W-RING FULL ERROR t=%d [millisec] ===> ERROR EXIT \n", (20*50 + 100*100) );
                return DPR_ERROR;
            }


    #if 0  /* this part was replaced by code above: wating on bsemRC rplaced by polling and sleep */
            {
                int g_dprlibfw_full_to = 2000;  /* [sec] timeout = max wait time til ring free */
                /* use wait DPR_SEM_WAIT_TIME  */
                /* to be sure that reader got a "message interrupt" - generate one more */
                /* notify reader with an interrupt to read the messages */
                DPR_WRITE_UINT32(DPR_INT_MESSAGE, &pCP->pDpramConfigArea->HPIMessage);
                dprlib_trigger_intr(pCP);

                /* release write lock for other writers */
                DPR_SEM_POST(pCP->ChannelDatas[ChIndex].bsemWriteLock);

                /* waiting constant time, that reader read data and confirmed that with MessageRC interrupt,
                 * so ring is not full again. PL: increased 1000 -> 20000, because of RQ:AP01415165
                 * IMPORTANT: default watchdog time out in the host driver is 1000ms, if host is to slow
                 * the timeout should be increased too. See driver/cp16xx_base.c:  int watchdog_cycle = 1000;
                 */
                DPRLIBINFMSG("W-RING FULL waiting max %d [sec] \n",g_dprlibfw_full_to);

                if(DPR_SEM_WAIT_TIME(pCP->ChannelDatas[ChIndex].bsemRC, (g_dprlibfw_full_to * 1000) )) {

                    /* if reader not read data from ring, we assume that reader hangs,
                       and no information can be transferd over that ring */
                    DPRLIBERRMSG("W-RING FULL TIMEOUT= %d [sec] ===> ERROR EXIT \n",g_dprlibfw_full_to);
                    return DPR_ERROR;
                }
            }
    #endif // 0

            // check if shut down is in progress
            if(DPR_ATOMIC_READ(pCP->dprStopAllThreads)) {
                DPRLIBERRMSG("dprStopAllThreads\n");
                return DPR_ERROR;
            }
        } else { /* otherwise write the message */
            ringFullCount = 0;
            DPR_WRITE_UINT32(DPR_RING_NOTFULL, &pRing->RingFull);

            /* first write header - length= sizeof(head) =8bytes */
            msgHead.MsgLength = CPU_TO_LE(msgHead.MsgLength);
            msgHead.FctHandle = CPU_TO_LE(msgHead.FctHandle);

            writePos = dprlib_raw_write(writePos, readPos, ringStart, ringEnd,
                (DPR_CHAR *)&msgHead, sizeof(DPR_MSG_HEAD));

            /* then write user message - length is the original user data length. NOT aligned */
            writePos = dprlib_raw_write(writePos, readPos, ringStart, ringEnd,
                msg, msgSize);

           /* writepos must be aligned to 4 bytes. After alignment, check for ring overrun.
            * PL: Fix error under 64bit Windows in the next line:
            * At input to the align function, writePos is a real pointer,
            * after conversion to ULONG writePos 'upper part' is zero => comparison fails !!!
            * Line was chaneged - previous version=
            * writePos = DPR_ULONG_TO_PTR(dprlib_align(DPR_PTR_TO_ULONG(writePos)));
            * Line fixed:
            * writePos = (DPR_CHAR *) GETADDR(dprlib_align(GETOFFSET(writePos)));
            */
            writePos = (DPR_CHAR *) GETADDR(dprlib_align(GETOFFSET(writePos)));
            if(writePos == ringEnd) {
                writePos = ringStart;
            }

           /* update the ring control structure in the shared memory
            */
            DPR_WRITE_UINT32(GETOFFSET(writePos), &pRing->pWrite);

            /* 1 msg write finished - Trigger interrupt*/
            DPR_WRITE_UINT32(DPR_INT_MESSAGE, &pCP->pDpramConfigArea->HPIMessage);
            dprlib_trigger_intr(pCP);

            /* release write lock for other threads */
            DPR_SEM_POST(pCP->ChannelDatas[ChIndex].bsemWriteLock);

            /* message could be written */
            WriteFinished = 1;
        }
    } /* end while !WriteFinished */
    return DPR_OK;
} /* end of dprlib_write_message_to_ring */

/*===========================================================================
* FUNCTION : DPRLIB_channel_write_message
*----------------------------------------------------------------------------
* PURPOSE  : do any modification needed in the header.
*            and redirect channel Index to ring index
*----------------------------------------------------------------------------
* RETURNS  : int
*----------------------------------------------------------------------------
* INPUTS   : Channel
* OUTPUTS  :
*----------------------------------------------------------------------------
* COMMENTS : no special processing is done now.
*==========================================================================*/
int DPRLIB_channel_write_message(void * pCP, DPR_CHN_TYPE Chnl,
    DPR_UINT32 handle, void * msg, unsigned long msgSize)
{
    if(NULL == pCP) {
#ifdef _DPR_FW
        pCP = &FWCpData;
#else
        return DPR_ERROR;
#endif
    }

    if(DPR_ATOMIC_READ(((CpData *)pCP)->semaphores_valid))
        return dprlib_write_message_to_ring((CpData *)pCP, DPRLIB_WRITE_RING(Chnl),
            handle, msg, msgSize);
    else
        return DPR_ERROR;
}

/*===========================================================================
* FUNCTION : DPRLIB_channel_register_cbf
*----------------------------------------------------------------------------
* INPUTS   : Chnl - Channel Index
*            msg  - message
*            msgLen - Lenth of message
* OUTPUTS  :
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
int DPRLIB_channel_register_cbf(
    void *pCP,
    DPR_CHN_TYPE Chnl,
    DPR_READ_CBF cbf)
{
    int ret;

    if(NULL == pCP) {
#ifdef _DPR_FW
        pCP = &FWCpData;
#else
        return DPR_ERROR;
#endif
    }

    if(cbf) {
        ((CpData *)pCP)->ChannelDatas[Chnl].readcbf = cbf;
        ret = DPR_OK;
    } else {
        DPRLIBERRMSG("channel %d, cbf is NULL %p\n", Chnl, cbf);
        ret = DPR_ERROR;
    }

    return ret;
}

/*===========================================================================
* FUNCTION : dprlib_read_message_from_ring
*----------------------------------------------------------------------------
* PURPOSE  : read mesage from ring
*----------------------------------------------------------------------------
* RETURNS  : DPR_OK: ==1  or DPR_ERROR ==0
*----------------------------------------------------------------------------
* INPUTS   : pCP
*            Read ring index
*              In the FW:  #define DPRLIB_READ_RING(Channel) (Channel*2)
*              In the HOST:#define DPRLIB_READ_RING(Channel) (Channel*2+1) !!!
*            msg: Pointer to the receive buffer, was allocated in cp16xx_base_read_cbf()
*            msgLen: User-data length, was retrieved from receive ring in
*                    dprlib_message_proc()
* OUTPUTS  :
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
int dprlib_read_message_from_ring(CpData *pCP, DPR_UINT32 RingIndex,
    DPR_VOID *msg, DPR_UINT32 msgLen)
{
    DPR_UINT32 ChIndex;
    DPR_CHAR *ringStart;
    DPR_CHAR *ringEnd;
    DPR_CHAR *readPos;
    DPR_CHAR *writePos;
    RING *pRing;

    pRing = &(pCP->pDpramConfigArea->Rings[RingIndex]);

    /* get the channel index of ring to get readlock ==bsemReadLock */
    ChIndex = RingIndex/2;

    /* check if ring initialized or not */
    if(0 == DPR_READ_UINT32(&pRing->pRingStart)) {
        DPRLIBERRMSG("Ring %d not initialized, pRingStart is NULL\n", RingIndex);
        return DPR_ERROR;
    }

    DPR_SEM_WAIT(pCP->ChannelDatas[ChIndex].bsemReadLock);

    /* make copy of original pointers */
    ringStart = (DPR_CHAR *) GETADDR(DPR_READ_UINT32(&pRing->pRingStart));
    ringEnd   = (DPR_CHAR *) GETADDR(DPR_READ_UINT32(&pRing->pRingEnd));
    readPos   = (DPR_CHAR *) GETADDR(DPR_READ_UINT32(&pRing->pRead));
    writePos  = (DPR_CHAR *) GETADDR(DPR_READ_UINT32(&pRing->pWrite));

    if(readPos != writePos) {
       /* there are data present in the ring, receive them.
        * After dprlib_raw_read() copmleted, readPos points to the
        * the 1st fill byte of 1-3 alignment bytes !!! or to the next
        * DPR message header if length was 4 byte aligned.
        * I.e.: readPost is NOT aligned  !!!
        */
        readPos = dprlib_raw_read(writePos, readPos,
                                  ringStart, ringEnd, msg, msgLen);
    } else {
       /* ring is empty or full  PL:??? ring full not clear???
        */
        DPR_SEM_POST(pCP->ChannelDatas[ChIndex].bsemReadLock);
        return DPR_ERROR;
    }

   /* readPos must be aligned to 4 bytes. After alignment, check for ring overrun
    * PL: Fix error under 64bit Windows: at input readPos is a real pointer,
    * after alignmen+conversion readPos 'upper part' is zero => comparison fails !!!
    * readPos = DPR_ULONG_TO_PTR(dprlib_align(DPR_PTR_TO_ULONG(readPos)));
    * Line fixed:
    */
    readPos = (DPR_CHAR *) GETADDR(dprlib_align(GETOFFSET(readPos)));
    if(readPos >= ringEnd)  {
        readPos = ringStart;
    }
    DPR_WRITE_UINT32(GETOFFSET(readPos), &pRing->pRead);

    if(DPR_RING_FULL == DPR_READ_UINT32(&pRing->RingFull)) {
        /* notify read complete */
        DPR_WRITE_UINT32(DPR_INT_MESSAGERC, &pCP->pDpramConfigArea->HPIMessageRC);
        dprlib_trigger_intr(pCP);
    }

    DPR_SEM_POST(pCP->ChannelDatas[ChIndex].bsemReadLock);

#ifdef _DPR_FW
    TelnetTrcDprReadMsgFromRing(ChIndex, msg, msgLen);
#endif

    return DPR_OK;
}

/*===========================================================================
* FUNCTION : DPRLIB_channel_read_message
*----------------------------------------------------------------------------
* PURPOSE  : read mesage from ring
*----------------------------------------------------------------------------
* RETURNS  :
*----------------------------------------------------------------------------
* INPUTS   :
* OUTPUTS  :
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
int DPRLIB_channel_read_message(void * pCP, DPR_CHN_TYPE Chnl,
    void * msg, unsigned long msgLen)
{
    int ret;

    if(NULL == pCP) {
#ifdef _DPR_FW
        pCP = &FWCpData;
#else
        return DPR_ERROR;
#endif
    }

    ret = dprlib_read_message_from_ring((CpData *)pCP,
                                         DPRLIB_READ_RING(Chnl), msg, msgLen);
    return ret;
}

/*===========================================================================
* FUNCTION : dprlib_message_proc
*----------------------------------------------------------------------------
* PURPOSE  : Received SP interrupt from host for MESSAGE
*    Host inserted a new message to one of the TDR ring
*    identify the ring and read all messages
*
* IN-HOST-Context: Is called from dprlib_proc() to receive messages from
*    the firmware. max_packets_at_once is allways =0  but
*    max_packets_at_once is allways ==0  means receive all messages at once !!!
*
*----------------------------------------------------------------------------
* RETURNS  : 0 no more packets, 1 are more packets to proceed
*----------------------------------------------------------------------------
* INPUTS   : max_packets_at_once: ==0  means receive all messages at once !!!
* OUTPUTS  :
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
int dprlib_message_proc(void *arg, DPR_CHN_TYPE first_ch_index,
    DPR_CHN_TYPE last_ch_index, DPR_UINT32 max_packets_at_once)
{
    DPR_UINT32 RingIndex;
    DPR_CHN_TYPE ChIndex;
    DPR_MSG_HEAD head = {0,0};
    CpData *pCP = (CpData*)arg;
    DPR_UINT32 user_id;
    unsigned long msg_length;
    unsigned long msg_with_head_length;
    RING *pRing;
    unsigned long more_messages, total_messages_found = 0;

    /*
    1. Check each TDR ring(for host index 0,2,4....) for any new message
    2. check read ptr and write ptr
    */

    DPRLIBCHATMSG("start: max_packets_at_once=%u \n", max_packets_at_once);

    /* keep checking till.. there is no msgs pending in any channel */
    do {
        more_messages = 0;

        for(ChIndex = first_ch_index; ChIndex <= last_ch_index; ++ChIndex)
        {
            RingIndex = DPRLIB_READ_RING(ChIndex);
            pRing = &(pCP->pDpramConfigArea->Rings[RingIndex]);

           /* PL:??? check if is initialized, i.e. pRingStart 'offset' is eq zero
            *    we are reading content of pRingStart
            */
            if(0 == DPR_READ_UINT32(&pRing->pRingStart))
            {
              /* ring length = 0, nothing to do*/
              continue;
            }

           /* unsigned int conparison: if pRead == pWrite: there is no receive message
            *  present in the ring or the ring is full, see else if below
            */
            if(DPR_READ_UINT32(&pRing->pRead) != DPR_READ_UINT32(&pRing->pWrite))
            {
               /* read msgs... & process it
                * 1st: read message header to get the 'user-message' length. Note: sizeof(head) == 8 bytes
                *      head.MsgLength == "real" (not aligned) msg-length + sizeof(head)
                */
                if(DPR_OK != DPRLIB_channel_read_message(pCP, ChIndex, &head, sizeof(head))) {
                    continue;
                }
                user_id              = LE_TO_CPU(head.FctHandle);
                msg_with_head_length = LE_TO_CPU(head.MsgLength);

                DPR_ASSERT(msg_with_head_length >= sizeof(head));  /* */

                msg_length = msg_with_head_length - sizeof(head);  /* retieve length of the 'user' data */

                ++total_messages_found;

               /* PL:??? special handling of user_id for MGT-channel:
                */
                if(DPR_MGT_CHN == ChIndex) {
                    /* we peek user_id from the begin of the user-data */
                    user_id = DPR_READ_UINT32(GETADDR(DPR_READ_UINT32(&pRing->pRead)));
                }

                if(!pCP->ChannelDatas[ChIndex].readcbf)
                {
                    DPRLIBERRMSG("Chnl %d ReadCBF missing, skip message, app_id %u, length %lu\n",
                        ChIndex, user_id, msg_length);

                    DPRLIB_channel_read_message(pCP, ChIndex, NULL, msg_length);
                }
               /* PL: Note: readcbf set to cp16xx_base_read_cbf (for all channels) See cp16xx_base.c
                */
                else if(DPR_OK != pCP->ChannelDatas[ChIndex].readcbf(pCP, ChIndex, user_id, msg_length))
                {
                    DPRLIBERRMSG("Chnl %d User error, skip message, app_id %u, length %lu\n",
                        ChIndex, user_id, msg_length);

                    DPRLIB_channel_read_message(pCP, ChIndex, NULL, msg_length);
                }

                if(DPR_ATOMIC_READ(pCP->dprStopAllThreads))
                    return 0;

            }
           /* here is pWrite == pRead --> we have to check if the ring is full
            * If full, generate only a receive interrupt to the host.
            */
            else if(DPR_RING_FULL == DPR_READ_UINT32(&pRing->RingFull))
            {
                /* notify read complete */
                DPR_WRITE_UINT32(DPR_INT_MESSAGERC, &pCP->pDpramConfigArea->HPIMessageRC);
                dprlib_trigger_intr(pCP);
            }

           /* check if there are still receive messages in the ring
            */
            if(DPR_READ_UINT32(&pRing->pRead) != DPR_READ_UINT32(&pRing->pWrite))
            {
                more_messages = 1;
            }
        }
        /* repeat so long any packets are to preceed */
    } while(more_messages && (!max_packets_at_once || total_messages_found < max_packets_at_once));

    DPRLIBCHATMSG("exit: total_messages_found=%lu \n",total_messages_found);

    return (total_messages_found < max_packets_at_once)?0:1;
}

/*===========================================================================
* FUNCTION : dprlib_messagerc_proc
*----------------------------------------------------------------------------
* PURPOSE  : notify blocked writing threads
*----------------------------------------------------------------------------
* RETURNS  :
*----------------------------------------------------------------------------
* INPUTS   :
* OUTPUTS  :
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
void dprlib_messagerc_proc(void *arg)
{
    int i;
    CpData *pCP = (CpData*)arg;

    DPRLIBLOGMSG("start\n");

    for(i = 0; i < DPR_CFG_MAX_CHANNEL; i++) {
        /* wake up any waititng write thread */
        DPR_SEM_POST(pCP->ChannelDatas[i].bsemRC);
    }

    DPRLIBLOGMSG("exit\n");
}

/*===========================================================================
* FUNCTION : dprlib_defaultInterruptProc
*----------------------------------------------------------------------------
* PURPOSE  :  default Interrupt handler for unexpected interrupts
*----------------------------------------------------------------------------
* RETURNS  :
*----------------------------------------------------------------------------
* INPUTS   :
* OUTPUTS  :
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
DPR_VOID dprlib_defaultInterruptProc(CpData *pCP, DPR_UINT32 IntrInfo)
{
    DPRLIBERRMSG(" ERROR... unexpected config interrupt type 0x%x\n",
        IntrInfo);
}

/*===========================================================================
* FUNCTION : dprlib_config_proc
*----------------------------------------------------------------------------
* PURPOSE  : handle config Interrupts, that will be reported
             in pDpramConfigArea->SPIConfig
*----------------------------------------------------------------------------
* RETURNS  :
*----------------------------------------------------------------------------
* INPUTS   :
* OUTPUTS  :
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
void dprlib_config_proc(void *arg)
{
    DPR_UINT32 ConfigIntrInfo;
    CpData *pCP = (CpData*)arg;

    DPRLIBLOGMSG("start\n");

    /* save interrupt info field */
    ConfigIntrInfo = DPR_READ_UINT32(&pCP->pDpramConfigArea->SPIConfig);

    /* clear interrupt info field */
    DPR_WRITE_UINT32(DPR_INT_DPR_READY, &pCP->pDpramConfigArea->SPIConfig);
    switch(ConfigIntrInfo) {
        case DPR_INT_CONF_CHNL:
            /* init channnel call*/
#ifdef _DPR_FW
            dprlib_init_channels(pCP);
#else
            DPR_SEM_POST(pCP->sem_config_conf);
#endif
            break;

        case DPR_INT_CONF_DPRAM:
            /* Init DPRAM call*/
#ifdef _DPR_FW
            {
                DPR_UINT32 DpramMaxSize;
                DpramMaxSize = DPR_READ_UINT32(&pCP->pDpramConfigArea->DpramMaxSize);
                dprlib_init_dpram(pCP, &DpramMaxSize);
                DPR_WRITE_UINT32(DpramMaxSize, &pCP->pDpramConfigArea->DpramMaxSize);
            }
#else
            DPR_SEM_POST(pCP->sem_config_conf);
#endif
            break;
#ifdef _DPR_FW
        case DPR_INT_DPR_SHUTDOWN:
            /* host shutdown, dprlib exit */
            if(DPR_READ_UINT32(&pCP->pDpramConfigArea->SPIMessage) == DPR_INT_DPR_ERR) {
                DPRLIBERRMSG("critical error in Host\n");
                DPR_WRITE_UINT32(DPR_INT_DPR_READY, &pCP->pDpramConfigArea->SPIMessage);
            }

            DPRLIBLOGMSG("host shutdown, release DPRAM interface\n");
            dprlib_uninit_dpram(pCP, DPR_ERR_HOST_SHUTDOWN_REQUEST);
            break;
#else
        case DPR_INT_DPR_ERR:
            DPRLIB_stop(pCP, DPR_ERR_FW_SHUTDOWN_REQUEST);
            break;

        case DPR_INT_RESTART_REQ:
            DPRLIB_restart_req(pCP);
            break;
#endif
        default:
            /* default is Ring Message */
            dprlib_defaultInterruptProc(pCP, 0);
            break;
    }

    DPRLIBLOGMSG("exit\n");
}

/*===========================================================================
* FUNCTION : dprlib_proc
*----------------------------------------------------------------------------
* PURPOSE  : handle all events
*----------------------------------------------------------------------------
* RETURNS  :
*----------------------------------------------------------------------------
* INPUTS   :
* OUTPUTS  :
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
DPR_THREAD_RETURN_TYPE dprlib_proc(void *arg)
{
    CpData *pCP = (CpData*)arg;

#ifndef _DPR_FW
    DPR_THREAD_DAEMONIZE("cp16xx_proc");
    DPR_THREAD_ALLOW_SIGNALS;
    DPRLIBLOGMSG("start\n");

    while(1)
    {
        /* sleep till interrupt comes */
        if(DPR_SEM_WAIT_INTERRUPTIBLE(pCP->sem_proc))
            break;

        ++pCP->loop_counter;

        if(DPR_ATOMIC_READ(pCP->dprStopAllThreads)) {
            /* exit from loop */
            DPRLIBLOGMSG("dprStopAllThreads\n");
            break;
        }
#endif

        if(DPR_READ_UINT32(&pCP->pDpramConfigArea->SPIIOData) == DPR_INT_IO_DATA) {
            /* clear interrupt info field */
            DPR_WRITE_UINT32(DPR_INT_DPR_READY, &pCP->pDpramConfigArea->SPIIOData);
            /* nothing else to do */
        }

        /* Do not access DPR channels (especially L2) if DPR Configuration is going on */
        if(DPR_READ_UINT32(&pCP->pDpramConfigArea->SPIConfig) == DPR_INT_DPR_READY) {
            dprlib_message_proc(pCP, DPR_L2_SEND_CHN, DPR_L2_RECV_CHN, 0);
        }

        /* message loop is called on every interrupt because there is the possibility */
        /* for a loss of the configuration command */
        if(DPR_READ_UINT32(&pCP->pDpramConfigArea->SPIMessage) == DPR_INT_MESSAGE) {
            /* clear interrupt info field */
            DPR_WRITE_UINT32(DPR_INT_DPR_READY, &pCP->pDpramConfigArea->SPIMessage);
            dprlib_message_proc(pCP, DPR_PNIO_CMD_CHN, DPR_HIF_CHN, 0);
        }

        if(DPR_READ_UINT32(&pCP->pDpramConfigArea->SPIMessageRC) == DPR_INT_MESSAGERC) {
            /* clear interrupt info field */
            DPR_WRITE_UINT32(DPR_INT_DPR_READY, &pCP->pDpramConfigArea->SPIMessageRC);
            /* activate semaphore for messagerc task */
            dprlib_messagerc_proc(pCP);
        }

        if(DPR_READ_UINT32(&pCP->pDpramConfigArea->SPIConfig) != DPR_INT_DPR_READY) {
            /* configuration is processed in the same context due to performance reasons */
            dprlib_config_proc(pCP);
        }
#ifndef _DPR_FW
    }
    pCP->thndl_proc = 0;
    DPRLIBLOGMSG("exit\n");
    DPR_THREAD_END();
#endif
}

/*===========================================================================
* FUNCTION : dprlib_int_callback
*----------------------------------------------------------------------------
* PURPOSE  : Main Interrupt handler
*
*----------------------------------------------------------------------------
* RETURNS  :
*----------------------------------------------------------------------------
* INPUTS   :
* OUTPUTS  :
*----------------------------------------------------------------------------
* COMMENTS : Handling should be as short as possible
*==========================================================================*/
void dprlib_int_callback(void *arg)
{
#ifdef _DPR_FW
    CpData *pCP = &FWCpData;
#else
    CpData *pCP = (CpData*)arg;
#endif

    /* process Interrupt if the module is still running */
    if(0 == DPR_ATOMIC_READ(pCP->dprStopAllThreads) && DPR_ATOMIC_READ(pCP->semaphores_valid)) {
#ifdef _DPR_FW
        dprlib_proc(pCP);
#else
        DPR_SEM_POST(pCP->sem_proc);
#endif
    }
}

/*===========================================================================
* FUNCTION : dprlib_stop_all_tasks_and_semaphores
*----------------------------------------------------------------------------
* PURPOSE  : save stopping of the dpr threads
*----------------------------------------------------------------------------
* RETURNS  :
*----------------------------------------------------------------------------
* INPUTS   :
* OUTPUTS  :
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
DPR_VOID dprlib_stop_all_tasks_and_semaphores(CpData *pCP)
{
    int i;

    DPRLIBLOGMSG("->\n");

    /* set variables which indicate a shutdown */
    DPR_ATOMIC_SET(pCP->semaphores_valid, 0);
    DPR_ATOMIC_SET(pCP->dprStopAllThreads, 1);

    /* notify thread to quit */
    DPR_SEM_POST(pCP->sem_proc);
#ifndef _DPR_FW
    DPR_SEM_POST(pCP->sem_config_conf);
#endif
    /* give some time to stop all threads */
    DPR_TASK_DELAY(1000);

    /* give thread a shance to end itself */
    while(pCP->thndl_proc) {
        DPR_TASK_DELAY(10);
    }

    /* destroy all semaphores used in the dpr module */
    for(i = 0; i < DPR_CFG_MAX_CHANNEL; i++) {
        DPR_SEM_POST(pCP->ChannelDatas[i].bsemWriteLock);
        DPR_SEM_DESTROY(pCP->ChannelDatas[i].bsemRC);
        DPR_SEM_DESTROY(pCP->ChannelDatas[i].bsemWriteLock);
        DPR_SEM_DESTROY(pCP->ChannelDatas[i].bsemReadLock);
    }

    DPR_SEM_DESTROY(pCP->sem_proc);
#ifndef _DPR_FW
    DPR_SEM_DESTROY(pCP->sem_config_conf);
#endif

    DPRLIBLOGMSG("<- \n");
}

/*===========================================================================
* FUNCTION : dprlib_start_all_tasks_and_semaphores
*----------------------------------------------------------------------------
* PURPOSE  : save stopping of the dpr threads
*----------------------------------------------------------------------------
* RETURNS  :
*----------------------------------------------------------------------------
* INPUTS   :
* OUTPUTS  :
*----------------------------------------------------------------------------
* COMMENTS :
*==========================================================================*/
int dprlib_start_all_tasks_and_semaphores(CpData *pCP)
{
    int i;

    DPRLIBLOGMSG("->\n");

    if(DPR_SEM_CREATE(pCP->sem_proc)) {
        DPRLIBERRMSG("create sama ERROR\n");
        return DPR_ERROR;
    }

#ifndef _DPR_FW
    if(DPR_SEM_CREATE(pCP->sem_config_conf))
        return DPR_ERROR;
#endif
    for(i = 0; i < DPR_CFG_MAX_CHANNEL; i++) {
        if (DPR_SEM_CREATE(pCP->ChannelDatas[i].bsemRC)) {
            DPRLIBERRMSG("create sama 'bsemRC' ERROR\n");
            return DPR_ERROR;
        }
        if (DPR_SEM_CREATE(pCP->ChannelDatas[i].bsemWriteLock)) {
            DPRLIBERRMSG("create sama 'bsemWriteLock' ERROR\n");
            return DPR_ERROR;
        }
        if (DPR_SEM_CREATE(pCP->ChannelDatas[i].bsemReadLock)) {
            DPRLIBERRMSG("create sama 'bsemReadLock' ERROR\n");
            return DPR_ERROR;
        }
        DPR_SEM_POST(pCP->ChannelDatas[i].bsemWriteLock);
        DPR_SEM_POST(pCP->ChannelDatas[i].bsemReadLock);
    }

    DPR_ATOMIC_SET(pCP->semaphores_valid, 1);
    DPRLIBLOGMSG("Semaphores OK created, now create threads\n");


#ifndef _DPR_FW
    /* spawn all threads and wait for events */
    if (!DPR_THREAD_CREATE(&pCP->thndl_proc, "dprlib_proc", dprlib_proc, pCP))
        return DPR_ERROR;
    /* give some time to create all threads & get ready to receive interrupts */
    DPR_TASK_DELAY(1);
#endif

    DPRLIBLOGMSG("<- ret DPR_OK\n");
    return DPR_OK;
}

DPR_UINT32 dprlib_get_remote_version(CpData *pCP)
{
    if(0 == DPR_ATOMIC_READ(pCP->bDriverConnected))
        return DPR_ERROR;

    return DPR_READ_UINT32(&pCP->pDpramConfigArea->RemoteVersion);
}

void dprlib_post_local_version(CpData *pCP)
{
#if defined(_DPR_FW)
#if defined(USE_OS_POSIX)
    int major, minor1, minor2, buildno;
    DPR_UINT32 local_version;

    mgt_read_fw_version_parts(&major, &minor1, &minor2, &buildno);

    local_version = (24 << major) | (16 << minor1) | (8 << minor2) | buildno;
    pCP->pDpramConfigArea->LocalVersion = CPU_TO_LE(local_version);
#else
    DPR_UINT32 version = ((FW_VER_MAJOR << 24) | (FW_VER_MINOR1 << 16) | (FW_VER_MINOR2 << 8) | VERSION_BUILD_NUMBER);
    pCP->pDpramConfigArea->LocalVersion = CPU_TO_LE(version);
#endif
#else
    DPR_UINT32 version = ((FW_VER_MAJOR << 24) | (FW_VER_MINOR1 << 16) | (DRV_VER_MINOR << 8) | VERSION_BUILD_NUMBER);
    DPR_WRITE_UINT32(version, &pCP->pDpramConfigArea->LocalVersion);
#endif
}

DPR_UINT32  DPRLIB_get_remote_version(void * pCP)
{
    if(NULL == pCP) {
#ifdef _DPR_FW
        pCP = &FWCpData;
#else
        return DPR_ERROR;
#endif
    }

    return dprlib_get_remote_version(pCP);
}

const char *DPRLIB_get_remote_version_as_string(void * pCP)
{
    DPR_UINT32 version;
    DPR_CHAR major, minor1, minor2, buildnumber;
    static char buf[20];

    version = DPRLIB_get_remote_version(pCP);

    if(version == DPR_ERROR) {
        DPR_STRNCPY(buf, "unavailable", sizeof(buf));
        return buf;
    }

    major =       (DPR_CHAR)(version >> 24);
    minor1 =      (DPR_CHAR)(version >> 16);
    minor2 =      (DPR_CHAR)(version >> 8);
    buildnumber = (DPR_CHAR)(version);

    DPR_SNPRINTF(buf, sizeof(buf), "%d.%d.%d.%d", major, minor1, minor2, buildnumber);

    return buf;
}

DPR_UINT32  DPRLIB_get_remote_version_as_string_ext(void * pCP,
    char *buf, DPR_UINT32 buflen)
{
    DPR_UINT32 version;
    DPR_CHAR major, minor1, minor2, buildnumber;
    size_t ret;

    version = DPRLIB_get_remote_version(pCP);

    if(version == DPR_ERROR) {
        DPR_STRNCPY(buf, "unavailable", buflen);
        return DPR_STRLEN(buf, buflen, ret);
    }

    major =       (DPR_CHAR)(version >> 24);
    minor1 =      (DPR_CHAR)(version >> 16);
    minor2 =      (DPR_CHAR)(version >> 8);
    buildnumber = (DPR_CHAR)(version);

    return DPR_SNPRINTF(buf, buflen, "%d.%d.%d.%d", major, minor1, minor2, buildnumber);
}



