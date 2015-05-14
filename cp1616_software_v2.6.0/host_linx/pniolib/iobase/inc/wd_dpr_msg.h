/*---------------------------------------------------------------------------
 *  Copyright (C) SIEMENS CORP., 2013 All rights reserved.
 *  Siemens AG. All Rights Reserved.
 *---------------------------------------------------------------------------
 *  Project     : PROFINET IO
 *  Package     : CP1616 DK Firmware
 *  Component   : Management
 *  File        : mgt_channel_wd.h
 *  Date        : 11-Mar-2005
 *
 *---------------------------------------------------------------------------
 *
 *  D e s c r i p t i o n:
 *
 *  Main fuction
 *
 *                Request block declarations for DPRAM management channel - Watchdog requests
 *
 *---------------------------------------------------------------------------
 *
 *  H i s t o r y:
 *
 *                FB    2005-03-11      first implementation
 *--------------------------------------------------------------------------- */

#ifndef _MGT_CHANNEL_WD_H
#define _MGT_CHANNEL_WD_H

#include "dpr_msg.h"

/* some subsystems for CP16xx watchdog */
#define FW_SUBSYS_WATCHDOG     0x0C /* PC watchdog user ifc */
#define FW_SUBSYS_PNIO_CTRL    0x9B /* PROFInetIO controller instances */
#define FW_SUBSYS_PNIO_DEVICE  0x9C /* PROFInetIO device instances */

typedef struct tag_WD_SUBSYSTEM {

    uint8_t ss;
    uint8_t ss_instance;

} WD_SUBSYSTEM;

typedef struct tag_WD_DPRMGTCHNL_RB {

    DPR_MSG_HDR header;    /* dpram channel header */
    uint16_t    wd_opcode; /* opcode */

} WD_DPRMGTCHNL_RB, * WD_DPRMGTCHNL_RB_PTR;

/* rb-declaration watchdog init, shut */
#define WD_MAX_SS_COUNT 8

typedef struct tag_WD_INIT_SHUT_DPRMGTCHNL_RB {

	DPR_MSG_HDR header;		/* dpram channel header */
	uint16_t     wd_opcode;		/* opcode */
	uint16_t     wd_flags;		/* flags for watchdog */
	uint32_t     wd_user_ref;	/* user_ref for watchdog */
	uint32_t     wd_timeout;	/* timeout for watchdog (ms) */
	uint32_t     wd_ref;		/* ref for watchdog */
	uint16_t     wd_ss_count;	/* count subsystems for watchdog */
	WD_SUBSYSTEM wd_ss[WD_MAX_SS_COUNT];	/* list of subsystems for watchdog */

} WD_INIT_DPRMGTCHNL_RB, * WD_INIT_DPRMGTCHNL_RB_PTR, WD_SHUT_DPRMGTCHNL_RB, * WD_SHUT_DPRMGTCHNL_RB_PTR;

/* calculate length field of init, shut rb */

#define CalculateLengthRbInit(wd_ss_count)	(sizeof(WD_INIT_DPRMGTCHNL_RB) - sizeof(DPR_MSG_HDR) - (WD_MAX_SS_COUNT - wd_ss_count) * sizeof(WD_SUBSYSTEM))
#define CalculateLengthRbShut(wd_ss_count)	(sizeof(WD_INIT_DPRMGTCHNL_RB) - sizeof(DPR_MSG_HDR) - (WD_MAX_SS_COUNT - wd_ss_count) * sizeof(WD_SUBSYSTEM))

/* constants for field of init, shut rb */

#define WD_OPC_INIT_REQ						0x0000			/* register or change watchdog */
#define WD_OPC_INIT_CNF						0x0001
#define WD_OPC_SHUT_REQ						0x0002			/* unregister watchdog */
#define WD_OPC_SHUT_CNF						0x0003

#define WD_FLAG_FIRMWARE					0x0001			/* controlling process is firmware watchdog */
#define WD_FLAG_HOST						0x0002			/* controlled process is firmware watchdog */
#define WD_FLAG_HW_WD1						0x0010			/* trigger hardware watchdog 1 (IRT application watchdog) */
#define WD_FLAG_HW_WD2						0x0020			/* trigger hardware watchdog 2 (CPU watchdog) */
#define WD_FLAG_HW_INVALID_DATA				0x0100			/* fast disconnect in case of timeout */
#define WD_FLAG_NOTIFY_HOST					0x1000			/* notify host in case of timeout */
#define WD_FLAG_NOTIFY_SUBSYSTEM			0x2000			/* notify subsystems in case of timeout */

#define WD_REF_INVALID						0xFFFFFFFF

#define WD_USER_REF_HOSTDRIVER				0x00000000
#define WD_USER_REF_HOSTAPP_MIN				0x00000001
#define WD_USER_REF_HOSTAPP_MAX				0x7FFFFFFF
#define WD_USER_REF_FIRMWAREAPP_MIN			0x80000000
#define WD_USER_REF_FIRMWAREAPP_MAX			0xFFFFFFFF

/* rb-declaration watchdog timeout */

typedef struct tag_WD_TIMEOUT_DPRMGTCHNL_RB {

	DPR_MSG_HDR	header;			/* dpram channel header */
	uint16_t	wd_opcode;		/* opcode */
	uint8_t	wd_reserved[2];		/* fill bytes */
	uint32_t	wd_user_ref;		/* user_ref for watchdog */
	uint32_t	wd_timeout;		/* timeout for watchdog (ms) */
	uint32_t	wd_ref;			/* ref for watchdog */
	uint32_t	wd_originator;		/* originator task for watchdog timeout */

} WD_TIMEOUT_DPRMGTCHNL_RB, * WD_TIMEOUT_DPRMGTCHNL_RB_PTR;

/* calculate length field of timeout rb */

#define CalculateLengthRbTimeout()			(sizeof(WD_TIMEOUT_DPRMGTCHNL_RB) - sizeof(DPR_MSG_HDR))

/* constants for field of timeout rb */

#define WD_OPC_TIMEOUT_IND			0x0004			/* watchdog timeout event */
#define WD_OPC_TIMEOUT_RSP			0x0005

#define WD_ORIGINATOR_UNKNOWN			0xFFFF			/* originator task is unknown */
#define WD_ORIGINATOR_DRIVER			0x0000			/* originator task is driver */
#define WD_ORIGINATOR_PNIO_CONTROLLER		0x0001			/* originator task is host controller application */
#define WD_ORIGINATOR_PNIO_DEVICE		0x0002			/* originator task is host device application */

#define WD_RB_RSP_OK				DPR_MSG_HDR_RSP_OK
#define WD_RB_RSP_ILLEGAL_SUBSYSTEM		DPR_MSG_HDR_RSP_ILLEGAL_SUBSYSTEM
#define WD_RB_RSP_NO_RESOURCES			DPR_MSG_HDR_RSP_NO_RESOURCES
#define WD_RB_RSP_UNKNOWN_OPC			((uint32_t)-1000)	/* unknown opcode */
#define WD_RB_RSP_UNKNOWN_PARAM			((uint32_t)-1001)	/* unknonwn parameter */

/* Representation of a watchdog in DPRAM. Position and count depends on hardware. */

typedef struct tag_DPRAM_WD
{

	uint8_t wd_trigger;
	uint8_t wd_flag;

} DPRAM_WD;

#define WD_TRIGGER_RESET			0		/* controlling process resets watchdog */
#define WD_TRIGGER_SET				1		/* controlled process triggers watchdog */
#define WD_TRIGGER_SHUT				2		/* controlled process shuts watchdog */

#define WD_FLAG_RUNNING				0		/* watchdog is running */
#define WD_FLAG_TIMEOUT				1		/* controlling process detects watchdog timeout */

#endif /* _MGT_CHANNEL_WD_H */
