/*---------------------------------------------------------------------------
 *  Copyright (C) SIEMENS CORP., 2013 All rights reserved.
 *  Siemens AG. All Rights Reserved.
 *---------------------------------------------------------------------------
 *  Project     : PROFINET IO
 *  Package     : CP1616 DK Firmware
 *  Component   : Management Task
 *  File        : mgt_dpr_msg.h
 *  Date        : 14.04.2005
 *
 *---------------------------------------------------------------------------
 *
 *  D e s c r i p t i o n:
 *
 *  DPRAM Management Channel:
 *  Declarations of Messages from Host to Management Task
 *
 *---------------------------------------------------------------------------
 *
 *  H i s t o r y:
 *
 *
 *--------------------------------------------------------------------------- */

#ifndef _MGT_DPR_MSG_H
#define _MGT_DPR_MSG_H


#include "dpr_msg.h"

/* subsystems for CP16xx management task */
#define FW_SUBSYS_CP_MGT            0x02  /* PC management user ifc */

/* rb-declaration (dpram management channel: from host to mgt task) */
typedef struct tag_DPRMGTCHNL_TO_MGT_RB
{
    DPR_MSG_HDR   header;                     /* dpram channel header */
    uint16_t      mgt_opcode;                 /* opcode */

} DPRMGTCHNL_TO_MGT_RB, * DPRMGTCHNL_TO_MGT_RB_PTR;


/* rb-declaration (dpram management channel) set dma base address */            /*MH071119*/
typedef struct tag_DPRMGTCHNL_TO_MGT_SET_DMA                                    /*MH071119*/
{                                                                               /*MH071119*/
    DPRMGTCHNL_TO_MGT_RB  rqb;                                                  /*MH071119*/
    uint32_t              base;                                                 /*MH071119*/
                                                                                /*MH071119*/
} DPRMGTCHNL_TO_MGT_SET_DMA, * DPRMGTCHNL_TO_MGT_SET_DMA_PTR;                   /*MH071119*/


/* opcodes (dpram management channel: from host to mgt task) */
#define OPC_MGT_HOST_ABORT_REQ      0x0000
#define OPC_MGT_HOST_ABORT_CNF      0x0001
#define OPC_MGT_HOST_SET_DMA_REQ    0x0002                                      /*MH071119*/
#define OPC_MGT_HOST_SET_DMA_CNF    0x0003                                      /*MH071119*/

/* response codes (dpram management channel: from host to mgt task) */
#define RSP_MGT_HOST_OK                  DPR_MSG_HDR_RSP_OK
#define RSP_MGT_HOST_ILLEGAL_SUBSYSTEM   DPR_MSG_HDR_RSP_ILLEGAL_SUBSYSTEM
#define RSP_MGT_HOST_NO_RESOURCES        DPR_MSG_HDR_RSP_NO_RESOURCES
#define RSP_MGT_HOST_UNKNOWN_OPC         ((uint32_t)-1000)  /* unknown opcode */
#define RSP_MGT_HOST_OPERATION_FAILED    ((uint32_t)-1001)  /* operation failed */

#endif
