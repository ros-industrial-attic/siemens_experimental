/*****************************************************************************/
/*  Copyright (C) SIEMENS CORP., 2013 All rights reserved.*/
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

#include "kramiotlb_cfg.h"
#include "cns_e400.h"

#define CNS_IO_x32(offset) (* (volatile CNS_UINT32 *)((IRTE_SWI_BaseAdr) + (offset)) )

#define CNS_MODULE_ID           0x3344UL

#define CONS_BLOCK_REQ_RD       0x0000A000
#define CONS_BLOCK_ACK_RD       0x0000A004
#define CONS_BLOCK_REQ_WR       0x0000A008
#define CONS_BLOCK_ACK_WR       0x0000A00C

#define SER_CONS_BLOCK_CMD_REQUEST          0x2
#define SER_CONS_BLOCK_CMD_FREE             0x1

#define SER_CONS_BLOCK_USED_NO_TRANSITION   0x0
#define SER_CONS_BLOCK_USED_TRANSITION      0x1

#define SER_CONS_BLOCK_OK_NOT_READY         0x0
#define SER_CONS_BLOCK_OK_READY             0x1

#define SER_CONS_BLOCK_ERROR_NOT_OK         0x1

#define CNS_BITFIELD  CNS_UINT32

typedef struct _EDD_SER_CONS_BLOCK_REQ_BIT
{
#if __BYTE_ORDER == __LITTLE_ENDIAN
    CNS_BITFIELD    BlockStart :20; /* 20-Bit, Start-Addres */
    CNS_BITFIELD    BlockLen   :8;  /* 8-Bit, length of the block */
    CNS_BITFIELD    BlockCmd   :2;  /* 2-Bit, Default-Prio */
    CNS_BITFIELD    Reserved   :2;  /* 2-Bit, Not used */
#elif __BYTE_ORDER == __BIG_ENDIAN
    CNS_BITFIELD    Reserved   :2;  /* 2-Bit, Not used */
    CNS_BITFIELD    BlockCmd   :2;  /* 2-Bit, Default-Prio */
    CNS_BITFIELD    BlockLen   :8;  /* 8-Bit, length of the block */
    CNS_BITFIELD    BlockStart :20; /* 20-Bit, Start-Addres */
#endif
} EDD_SER_CONS_BLOCK_REQ_BIT;

typedef union _EDD_SER_CONS_BLOCK_REQ_TYPE
{
    CNS_UINT32                     Reg;
    EDD_SER_CONS_BLOCK_REQ_BIT     Bit;

} EDD_SER_CONS_BLOCK_REQ_TYPE;

typedef struct _EDD_SER_CONS_BLOCK_ACK_BIT
{
#if __BYTE_ORDER == __LITTLE_ENDIAN
    CNS_BITFIELD      BlockStart :20; /* 20-Bit, Start-Addres */
    CNS_BITFIELD      BlockLen   :8;  /* 8-Bit, length of the block  */
    CNS_BITFIELD      BlockUsed  :1;  /* 1-Bit, */
    CNS_BITFIELD      BlockOk    :1;  /* 1-Bit, Default-Prio */
    CNS_BITFIELD      BlockError :1;  /* 1-Bit, Default-Prio */
    CNS_BITFIELD      Reserved   :1;  /* 1-Bit, Not used */
#elif __BYTE_ORDER == __BIG_ENDIAN
    CNS_BITFIELD      Reserved   :1;  /* 1-Bit, Not used */
    CNS_BITFIELD      BlockError :1;  /* 1-Bit, Default-Prio */
    CNS_BITFIELD      BlockOk    :1;  /* 1-Bit, Default-Prio */
    CNS_BITFIELD      BlockUsed  :1;  /* 1-Bit, */
    CNS_BITFIELD      BlockLen   :8;  /* 8-Bit, length of the block */
    CNS_BITFIELD      BlockStart :20; /* 20-Bit, Start-Addres */
#endif
} EDD_SER_CONS_BLOCK_ACK_BIT;

typedef union _EDD_SER_CONS_BLOCK_ACK_TYPE
{
    CNS_UINT32                  Reg;
    EDD_SER_CONS_BLOCK_ACK_BIT  Bit;

} EDD_SER_CONS_BLOCK_ACK_TYPE;

static void CNS_FCT_ATTR  CNSFatalError(
    const CNS_UINT32   Line,
    CNS_UINT8          * const sFile,
    const CNS_UINT32   ModuleID,
    CNS_UINT8          * const sErr,
    const CNS_UINT32   Error,
    const CNS_UINT32   DW_0,
    const CNS_UINT32   DW_1   );

static CNS_EXCP_FCT   MD_excp_fct = (CNS_EXCP_FCT)CNSFatalError;

#define CNS_KRAM_BASE_OFFSET    0x00100000UL   /* 1MByte */
#define CNS_KRAM_SIZE           0x00030000UL   /* 192KByte */

void   CNS_FCT_ATTR CnsSetExcp(CNS_EXCP_FCT const excp_fct )
{

 if ( excp_fct == (void *)0)
 {
  return;
 }

 MD_excp_fct = excp_fct;
}

CNS_UINT32  CNS_FCT_ATTR CnsBlockRead(
    const CNS_UINT8 *       IRTE_SWI_BaseAdr,
    const CNS_UINT32        iBlockStart,
    const CNS_UINT32        iBlockLen,
    const CNS_UINT32        PollTimeout)
{
  EDD_SER_CONS_BLOCK_REQ_TYPE     ConsBlockReq;
  CNS_UINT32                      TimeOut;
  EDD_SER_CONS_BLOCK_ACK_TYPE     ConsBlockAck;



  if ( iBlockStart > CNS_KRAM_SIZE )
  {
    MD_excp_fct((CNS_UINT32)__LINE__, (CNS_UINT8 *)(void *)__FILE__, CNS_MODULE_ID,
                (CNS_UINT8 *)(void *)"CnsBlockRead 0", iBlockStart, 0UL, 0UL);
    return 0;
  }

  ConsBlockReq.Reg = 0;
  ConsBlockReq.Bit.BlockStart = iBlockStart | CNS_KRAM_BASE_OFFSET;
  ConsBlockReq.Bit.BlockLen   = iBlockLen;
  ConsBlockReq.Bit.BlockCmd   = SER_CONS_BLOCK_CMD_REQUEST;

  CNS_IO_x32(CONS_BLOCK_REQ_RD) = CNS_SWAP_32(ConsBlockReq.Reg);

  ConsBlockAck.Reg = 0;

  for (TimeOut = 0; TimeOut < PollTimeout; TimeOut++)
  {
    ConsBlockAck.Reg = CNS_SWAP_32(CNS_IO_x32(CONS_BLOCK_ACK_RD));

    if (ConsBlockAck.Bit.BlockError == SER_CONS_BLOCK_ERROR_NOT_OK)
    {
      /* Sequence-Error -> return at once !! */
      MD_excp_fct((CNS_UINT32)__LINE__, (CNS_UINT8 *)(void *)__FILE__, CNS_MODULE_ID,
                  (CNS_UINT8 *)(void *)"CnsBlockRead 1", 0UL, 0UL, 0UL);
      return TimeOut;
    }

    if ( (ConsBlockAck.Bit.BlockUsed == SER_CONS_BLOCK_USED_TRANSITION) &&
         (ConsBlockAck.Bit.BlockOk   == SER_CONS_BLOCK_OK_READY)               )
    {
      /* Command acknowledged -> return OK */
      return TimeOut;
    }

  }

  MD_excp_fct((CNS_UINT32)__LINE__, (CNS_UINT8 *)(void *)__FILE__, CNS_MODULE_ID,
              (CNS_UINT8 *)(void *)"CnsBlockRead 2", 0UL, 0UL, 0UL);

  return TimeOut;

}

/*===========================================================================*/


/*=============================================================================
* function name: SERConsBlockCmdLock
*
* function:  Command
*
* parameters:  CNS_UINT32          iRegCode
*                  CNS_UINT32          iBlockStart
*                  CNS_UINT16          iBlockLen
*
* return value: CNS_RESULT   SER_OK
*                                      SER_ERR_SEQUENCE
*                                      SER_ERR_TIMEOUT
*
*     All other RQB-parameters depend on kind of request.
*===========================================================================*/
CNS_UINT32  CNS_FCT_ATTR CnsBlockFreeRead(const CNS_UINT8 *    IRTE_SWI_BaseAdr,
                                          const CNS_UINT32     PollTimeout      )
{
  EDD_SER_CONS_BLOCK_REQ_TYPE     ConsBlockReq;
  CNS_UINT32                      TimeOut;
  EDD_SER_CONS_BLOCK_ACK_TYPE     ConsBlockAck;


  ConsBlockReq.Reg = 0;
  ConsBlockReq.Bit.BlockCmd   = SER_CONS_BLOCK_CMD_FREE;

  CNS_IO_x32(CONS_BLOCK_REQ_RD) = CNS_SWAP_32(ConsBlockReq.Reg);

  ConsBlockAck.Reg = 0;

  for (TimeOut = 0; TimeOut < PollTimeout; TimeOut++)
  {
    ConsBlockAck.Reg = CNS_SWAP_32(CNS_IO_x32(CONS_BLOCK_ACK_RD));

    if (ConsBlockAck.Bit.BlockError == SER_CONS_BLOCK_ERROR_NOT_OK)
    {
      /* Sequence-Error -> return at once !! */
      MD_excp_fct((CNS_UINT32)__LINE__, (CNS_UINT8 *)(void *)__FILE__, CNS_MODULE_ID,
                  (CNS_UINT8 *)(void *)"CnsBlockFreeRead 0", 0UL, 0UL, 0UL);
      return TimeOut;
    }

    if (   (ConsBlockAck.Bit.BlockUsed == SER_CONS_BLOCK_USED_NO_TRANSITION) &&
           (ConsBlockAck.Bit.BlockOk   == SER_CONS_BLOCK_OK_NOT_READY)             )
    {
      /* Command acknowledged -> return OK */
      return TimeOut;
    }

  }

  MD_excp_fct((CNS_UINT32)__LINE__, (CNS_UINT8 *)(void *)__FILE__, CNS_MODULE_ID,
              (CNS_UINT8 *)(void *)"CnsBlockFreeRead 1", 0UL, 0UL, 0UL);

  return TimeOut;
}
/*===========================================================================*/

/*=============================================================================
* function name: SERConsBlockWrite
*
* function:  Command
*
* parameters:  CNS_UINT32          iRegCode
*                  CNS_UINT32          iBlockStart
*                  CNS_UINT16          iBlockLen
*
* return value: CNS_RESULT   SER_OK
*                                      SER_ERR_SEQUENCE
*                                      SER_ERR_TIMEOUT
*
*     All other RQB-parameters depend on kind of request.
*===========================================================================*/
CNS_UINT32 CNS_FCT_ATTR CnsBlockWrite( const CNS_UINT8 *       IRTE_SWI_BaseAdr,
                                       const CNS_UINT32        iBlockStart,
                                       const CNS_UINT32        iBlockLen,
                                       const CNS_UINT32        PollTimeout   )
{
  EDD_SER_CONS_BLOCK_REQ_TYPE     ConsBlockReq;
  CNS_UINT32                      TimeOut;
  EDD_SER_CONS_BLOCK_ACK_TYPE     ConsBlockAck;


  if ( iBlockStart > CNS_KRAM_SIZE )
  {
    MD_excp_fct((CNS_UINT32)__LINE__, (CNS_UINT8 *)(void *)__FILE__, CNS_MODULE_ID,
                (CNS_UINT8 *)(void *)"CnsBlockWrite0", iBlockStart, 0UL, 0UL);
    return 0;
  }

  ConsBlockReq.Reg = 0;
  ConsBlockReq.Bit.BlockStart = iBlockStart | CNS_KRAM_BASE_OFFSET;
  ConsBlockReq.Bit.BlockLen   = iBlockLen;
  ConsBlockReq.Bit.BlockCmd   = SER_CONS_BLOCK_CMD_REQUEST;

  CNS_IO_x32(CONS_BLOCK_REQ_WR) = CNS_SWAP_32(ConsBlockReq.Reg);

  ConsBlockAck.Reg = 0;

  for (TimeOut = 0; TimeOut < PollTimeout; TimeOut++)
  {
    ConsBlockAck.Reg = CNS_SWAP_32(CNS_IO_x32(CONS_BLOCK_ACK_WR));

    if (ConsBlockAck.Bit.BlockError == SER_CONS_BLOCK_ERROR_NOT_OK)
    {
      /* Sequence-Error -> return at once !! */
      MD_excp_fct((CNS_UINT32)__LINE__, (CNS_UINT8 *)(void *)__FILE__, CNS_MODULE_ID,
                  (CNS_UINT8 *)(void *)"CnsBlockWrite 1", 0UL, 0UL, 0UL);
      return TimeOut;
    }

    if ( (ConsBlockAck.Bit.BlockUsed == SER_CONS_BLOCK_USED_TRANSITION) &&
         (ConsBlockAck.Bit.BlockOk   == SER_CONS_BLOCK_OK_READY)  )
    {
      /* Command acknowledged -> return OK */
      return TimeOut;
    }

  }

  MD_excp_fct((CNS_UINT32)__LINE__, (CNS_UINT8 *)(void *)__FILE__, CNS_MODULE_ID,
              (CNS_UINT8 *)(void *)"CnsBlockWrite 2", 0UL, 0UL, 0UL);

  return TimeOut;

}
/*===========================================================================*/

/*=============================================================================
* function name: SERConsBlockCmdLock
*
* function:  Command
*
* parameters:  CNS_UINT32          iRegCode
*                  CNS_UINT16          iBlockCmd
*                  CNS_UINT32          iBlockStart
*                  CNS_UINT16          iBlockLen
*
* return value: CNS_RESULT   SER_OK
*                                      SER_ERR_SEQUENCE
*                                      SER_ERR_TIMEOUT
*
*     All other RQB-parameters depend on kind of request.
*===========================================================================*/
CNS_UINT32  CNS_FCT_ATTR CnsBlockFreeWrite(const CNS_UINT8 *       IRTE_SWI_BaseAdr,
                                           const CNS_UINT32        PollTimeout    )
{
  EDD_SER_CONS_BLOCK_REQ_TYPE     ConsBlockReq;
  CNS_UINT32                      TimeOut;
  EDD_SER_CONS_BLOCK_ACK_TYPE     ConsBlockAck;


  ConsBlockReq.Reg = 0;
  ConsBlockReq.Bit.BlockCmd   = SER_CONS_BLOCK_CMD_FREE;

  CNS_IO_x32(CONS_BLOCK_REQ_WR) = CNS_SWAP_32(ConsBlockReq.Reg);

  ConsBlockAck.Reg = 0;

  for (TimeOut = 0; TimeOut < PollTimeout; TimeOut++)
  {
    ConsBlockAck.Reg = CNS_SWAP_32(CNS_IO_x32(CONS_BLOCK_ACK_WR));

    if (ConsBlockAck.Bit.BlockError == SER_CONS_BLOCK_ERROR_NOT_OK)
    {
      /* Sequence-Error -> return at once !! */
      MD_excp_fct((CNS_UINT32)__LINE__, (CNS_UINT8 *)(void *)__FILE__, CNS_MODULE_ID,
                  (CNS_UINT8 *)(void *)"CnsBlockFreeWrite 0", 0UL, 0UL, 0UL);
      return TimeOut;
    }

    if ( (ConsBlockAck.Bit.BlockUsed == SER_CONS_BLOCK_USED_NO_TRANSITION) &&
         (ConsBlockAck.Bit.BlockOk   == SER_CONS_BLOCK_OK_NOT_READY)   )
    {
      /* Command acknowledged -> return OK */
      return TimeOut;
    }

  }

  MD_excp_fct((CNS_UINT32)__LINE__, (CNS_UINT8 *)(void *)__FILE__, CNS_MODULE_ID,
              (CNS_UINT8 *)(void *)"CnsBlockFreeWrite", 0UL, 0UL, 0UL);

  return TimeOut;
}
/*===========================================================================*/


/*=============================================================================
* function name: SERCons_Reset
*
* function:  Special function to soft-reset the Consistency - Control
*            free all eventually used Buffers
*
* parameters:  EDD_LOCAL_DDB_PTR_TYPE  const pDDB
*
* return value: void
*
*===========================================================================*/
void       CNS_FCT_ATTR CnsReset(const CNS_UINT8 *         IRTE_SWI_BaseAdr )
{
  EDD_SER_CONS_BLOCK_REQ_TYPE     ConsBlockReq;
  EDD_SER_CONS_BLOCK_ACK_TYPE     ConsBlockAck;


  ConsBlockReq.Reg = CNS_SWAP_32(CNS_IO_x32(CONS_BLOCK_REQ_RD));
  ConsBlockAck.Reg = CNS_SWAP_32(CNS_IO_x32(CONS_BLOCK_ACK_RD));

  if (ConsBlockAck.Bit.BlockOk)
  {
    (void)CnsBlockFreeRead(IRTE_SWI_BaseAdr, CNS_MAX_POLL_TIMEOUT);
  }

  /* TODO: ConsBlock-Regs must be resetted manually */
  CNS_IO_x32(CONS_BLOCK_REQ_RD) = 0x00000000; /* Strange: these oparations have no effect on registers. */
  CNS_IO_x32(CONS_BLOCK_ACK_RD) = 0x00000000;

  /*////////////////////////////////////////////////////////////////////////
  // Check the WRITE-BLOCK:
  ////////////////////////////////////////////////////////////////////////*/

  ConsBlockReq.Reg = CNS_SWAP_32(CNS_IO_x32(CONS_BLOCK_REQ_WR));
  ConsBlockAck.Reg = CNS_SWAP_32(CNS_IO_x32(CONS_BLOCK_ACK_WR));

  if (ConsBlockAck.Bit.BlockOk) /* TODO: do we also need to Check "Bit.BlockUsed"  ? */
  { /* Block is still in use -> free it ! */
    (void)CnsBlockFreeWrite(IRTE_SWI_BaseAdr, CNS_MAX_POLL_TIMEOUT);
  }

  /* ConsBlock-Regs must be resetted manually */
  CNS_IO_x32(CONS_BLOCK_REQ_WR) = 0x00000000; /* Strange: these oparations have no effect on registers. */
  CNS_IO_x32(CONS_BLOCK_ACK_WR) = 0x00000000;

  if (ConsBlockReq.Reg)
  {
  }
}

/*lint -restore */

/***************************************************************************/
/* F u n c t i o n:                                                        */
/*                                                                         */
/* D e s c r i p t i o n :                                                 */
/*                                                                         */
/*                                                                         */
/* A r g u m e n t s:                                                      */
/*                                                                         */
/* Return Value:                                                           */
/*                                                                         */
/*                                                                         */
/***************************************************************************/
static void CNS_FCT_ATTR   CNSFatalError( const CNS_UINT32   Line,
                                                unsigned char      * const sFile,
                                                const CNS_UINT32   ModuleID,
                                                unsigned char      * const sErr,
                                                const CNS_UINT32   Error,
                                                const CNS_UINT32   DW_0,
                                                const CNS_UINT32   DW_1   )
{
 if ( DW_0 || DW_1 || Error || Line || ModuleID || sFile || sErr)
 {
 }

 for ( ;; )
 {
 }
}
/*---------------------- end [subroutine] ---------------------------------*/

#define SYNC_MODE           0x00013400
#define SYNC_MODE_IN_DONE   0x00000001   /* 1-Bit, Input-Data operating ready  */
#define SYNC_MODE_OUT_DONE  0x00000002   /* 1-Bit, Output-Data operating ready */
/* #define  SYNC_MODE_DMA_IN_ENABLE   0x00000004   1-Bit, ..... */

void   CNS_FCT_ATTR  CnsInOutDone(const CNS_UINT8 *IRTE_SWI_BaseAdr)
{
  CNS_UINT32    SyncMode;

  /* get current SyncMode-Register-Value  */
  SyncMode = CNS_SWAP_32(CNS_IO_x32(SYNC_MODE));

  /* prepare the new Value (activate IN_DONE-bit) */
  SyncMode = SyncMode | (SYNC_MODE_IN_DONE | SYNC_MODE_OUT_DONE);
  /* write back the new Value */
  CNS_IO_x32(SYNC_MODE) = CNS_SWAP_32(SyncMode);
}

/*****************************************************************************/
/*  end of file CNS_E400.C                                                   */
/*****************************************************************************/
