/*****************************************************************************/
/*  Copyright (C) SIEMENS CORP., 2013 All rights reserved.*/
/*****************************************************************************/
/*                                                                           */
/*  P r o j e c t         &P: SYSLTRC                           :P&          */
/*                                                                           */
/*  P a c k a g e         &W: LTRC-SYSADJ                       :W&          */
/*                                                                           */
/*  C o m p o n e n t     &C: LTRC-SYSADJ                       :C&          */
/*                                                                           */
/*  F i l e               &F: ltrc_sub.h                        :F&          */
/*                                                                           */
/*****************************************************************************/
/*                                                                           */
/*  D e s c r i p t i o n:                                                   */
/*                                                                           */
/*                       Frame for file "ltrc_sub.h".                        */
/*                       ============================                        */
/*                                                                           */
/*  Configuration for ltrc:                                                  */
/*  Defines subsystems for ltrc.                                             */
/*                                                                           */
/*  This file has to be overwritten during system integration, because       */
/*  some definitions depend on the different subsystems for lsa-trace.       */
/*                                                                           */
/*****************************************************************************/

#if !defined   LTRC_SUB_H_INCL
#     define   LTRC_SUB_H_INCL   1

/*=============================================================================
 * (1) ltrc_subsys
 *===========================================================================*/
enum ltrc_subsys
{
   TRACE_SUBSYS_TRACE, /* dont change the first entry, its necessary for ltrc */
   /*********************************************************************/
      TRACE_SUBSYS_ACL,
      TRACE_SUBSYS_BSD_SELECT,
      TRACE_SUBSYS_BSD_USER,
      TRACE_SUBSYS_BSD_VXADAPT,
      TRACE_SUBSYS_BUB,
      TRACE_SUBSYS_DICON,
      TRACE_SUBSYS_FWDL,
      TRACE_SUBSYS_IPCONFIG,
      TRACE_SUBSYS_MGT,
      TRACE_SUBSYS_OVS,
      TRACE_SUBSYS_OVS_1,
      TRACE_SUBSYS_OVS_2,
      TRACE_SUBSYS_OVS_3,
      TRACE_SUBSYS_PANEL,
      TRACE_SUBSYS_PGTASK,
      TRACE_SUBSYS_PGTASK_1,
      TRACE_SUBSYS_SNDRCV,
      TRACE_SUBSYS_TIMESYNC,
      TRACE_SUBSYS_VARSERV,
      TRACE_SUBSYS_TIS,
      TRACE_SUBSYS_TIS_1,
      TRACE_SUBSYS_TREL,
      TRACE_SUBSYS_SECURITY,

      /*************************************************************************/
      /* le_nbg2 -- ICON */
      TRACE_SUBSYS_ICON_CHANNEL,
      TRACE_SUBSYS_ICON_FLICT,
      TRACE_SUBSYS_ICON_MGT,
      TRACE_SUBSYS_ICON_PDEV,
      TRACE_SUBSYS_ICON_PSU,
      TRACE_SUBSYS_ICON_REMA,
      TRACE_SUBSYS_ICON_TIS,
    
      TRACE_SUBSYS_IPC,

      /*********************************************************************/
      /* PT2_LSA -- LSA-Komponenten */
      TRACE_SUBSYS_ACP_UPPER,
      TRACE_SUBSYS_ACP_LOWER,
      TRACE_SUBSYS_ACP_RTA,
      TRACE_SUBSYS_ACP_MEMORY,

      TRACE_SUBSYS_CLRPC_UPPER,
      TRACE_SUBSYS_CLRPC_CL,
      TRACE_SUBSYS_CLRPC_SV,
      TRACE_SUBSYS_CLRPC_EPM,
      TRACE_SUBSYS_CLRPC_LOWER,
      TRACE_SUBSYS_CLRPC_CL_PKT,
      TRACE_SUBSYS_CLRPC_SV_PKT,
      
      TRACE_SUBSYS_CM_ACP,
      TRACE_SUBSYS_CM_AR,
      TRACE_SUBSYS_CM_CL,
      TRACE_SUBSYS_CM_MC,
      TRACE_SUBSYS_CM_MEM,
      TRACE_SUBSYS_CM_NARE,
      TRACE_SUBSYS_CM_RPC,
      TRACE_SUBSYS_CM_SV,
      TRACE_SUBSYS_CM_UPPER,
      TRACE_SUBSYS_CM_EDD,
      TRACE_SUBSYS_CM_GSY,
      TRACE_SUBSYS_CM_OHA,
      TRACE_SUBSYS_CM_PD,
      TRACE_SUBSYS_CM_MRP, 
      TRACE_SUBSYS_CM_POF, 
      TRACE_SUBSYS_CM_REMA,

      TRACE_SUBSYS_DCP_UPPER,
      TRACE_SUBSYS_DCP_LOWER,
      TRACE_SUBSYS_DCP_SYSTEM,
      TRACE_SUBSYS_DCP_ERROR,
      TRACE_SUBSYS_DCP_FUNCTION,
      TRACE_SUBSYS_DCP_SERVER,
      TRACE_SUBSYS_DCP_SRVERR,
      TRACE_SUBSYS_DCP_SNDRCV,

      TRACE_SUBSYS_GSY_UPPER,
      TRACE_SUBSYS_GSY_LOWER,
      TRACE_SUBSYS_GSY_SYSTEM,
      TRACE_SUBSYS_GSY_ERROR,
      TRACE_SUBSYS_GSY_FUNCTION,
      TRACE_SUBSYS_GSY_DIAG,
      TRACE_SUBSYS_GSY_DEL,
      TRACE_SUBSYS_GSY_FWD,
      TRACE_SUBSYS_GSY_SNDRCV,
      TRACE_SUBSYS_GSY_PRM,
      TRACE_SUBSYS_GSY_SYNC,
      TRACE_SUBSYS_GSY_MASTER,
      
      TRACE_SUBSYS_NARE_UPPER,
      TRACE_SUBSYS_NARE_LOWER,
      TRACE_SUBSYS_NARE_SYSTEM,
      TRACE_SUBSYS_NARE_FUNCTION,
      TRACE_SUBSYS_NARE_PROGRAM,
      
      TRACE_SUBSYS_LSYS_GLOB,
      TRACE_SUBSYS_LSYS_CM,
      TRACE_SUBSYS_LSYS_PSU,
      TRACE_SUBSYS_LSYS_LTRC,
      
      TRACE_SUBSYS_LLDP_UPPER,
      TRACE_SUBSYS_LLDP_LOWER,
      TRACE_SUBSYS_LLDP_SYSTEM,
      TRACE_SUBSYS_LLDP_FUNCTION,
      TRACE_SUBSYS_LLDP_PROGRAM,

      TRACE_SUBSYS_MRP_UPPER,
      TRACE_SUBSYS_MRP_LOWER,
      TRACE_SUBSYS_MRP_SYSTEM,
      TRACE_SUBSYS_MRP_FUNCTION,
      TRACE_SUBSYS_MRP_PROGRAM,

      TRACE_SUBSYS_OHA_UPPER,
      TRACE_SUBSYS_OHA_LOWER,
      TRACE_SUBSYS_OHA_SYSTEM,
      TRACE_SUBSYS_OHA_FUNCTION,
      TRACE_SUBSYS_OHA_PROGRAM,

      TRACE_SUBSYS_POF_UPPER,
      TRACE_SUBSYS_POF_LOWER,
      TRACE_SUBSYS_POF_SYSTEM,
      TRACE_SUBSYS_POF_FUNCTION,
      TRACE_SUBSYS_POF_PROGRAM,

      TRACE_SUBSYS_SOCK_UPPER,
      TRACE_SUBSYS_SOCK_LOWER,
      TRACE_SUBSYS_SOCK_SYSTEM,
      TRACE_SUBSYS_SOCK_PROTOCOL,
      TRACE_SUBSYS_SOCK_STACK,

      TRACE_SUBSYS_EDDI_UPPER,
      TRACE_SUBSYS_EDDI_LOWER,
      TRACE_SUBSYS_EDDI_SYSTEM,
      TRACE_SUBSYS_EDDI_FUNCTION,
      TRACE_SUBSYS_EDDI_PROGRAM,
      TRACE_SUBSYS_EDDI_NRT,
      TRACE_SUBSYS_EDDI_PRM,
      TRACE_SUBSYS_EDDI_SWI,
      TRACE_SUBSYS_EDDI_CRT,
      TRACE_SUBSYS_EDDI_SYNC,

      TRACE_SUBSYS_EDDS_UPPER,
      TRACE_SUBSYS_EDDS_LOWER,
      TRACE_SUBSYS_EDDS_SYSTEM,
      TRACE_SUBSYS_EDDS_FUNCTION,
      TRACE_SUBSYS_EDDS_PROGRAM,
      TRACE_SUBSYS_EDDS_NRT,
      TRACE_SUBSYS_EDDS_PRM,
      TRACE_SUBSYS_EDDS_SWI,
      TRACE_SUBSYS_EDDS_CRT,
      TRACE_SUBSYS_EDDS_SYNC,

      TRACE_SUBSYS_EDDP_UPPER,
      TRACE_SUBSYS_EDDP_LOWER,
      TRACE_SUBSYS_EDDP_SYSTEM,
      TRACE_SUBSYS_EDDP_FUNCTION,
      TRACE_SUBSYS_EDDP_PROGRAM,
      TRACE_SUBSYS_EDDP_NRT,
      TRACE_SUBSYS_EDDP_PRM,
      TRACE_SUBSYS_EDDP_SWI,
      TRACE_SUBSYS_EDDP_CRT,
      TRACE_SUBSYS_EDDP_SYNC,

      /*********************************************************************/
      /* PNIO */
      TRACE_SUBSYS_PNIO_AGET300,
      TRACE_SUBSYS_PNIO_AGET400,
      TRACE_SUBSYS_PNIO_CTRL_GLOB,
      TRACE_SUBSYS_PNIO_CTRL_ALERT,
      TRACE_SUBSYS_PNIO_CTRL_DRRW,
      TRACE_SUBSYS_PNIO_CTRL_MEM,
      TRACE_SUBSYS_PNIO_CTRL_SDB,
      TRACE_SUBSYS_PNIO_CTRL_STATE,
      TRACE_SUBSYS_PNIO_ARM_UPDATE,
      TRACE_SUBSYS_PNIO_AGENTDEVK_CTRL,
      TRACE_SUBSYS_PNIO_AGENTDEVK_DEV,

      TRACE_SUBSYS_IOD,
      TRACE_SUBSYS_SDEV,
      TRACE_SUBSYS_APPL_IOD,
      TRACE_SUBSYS_IOD_TESTAPPL,

      TRACE_SUBSYS_IOR,
      TRACE_SUBSYS_L2_AGENT,

      /*********************************************************************/
      /* H1T */
      TRACE_SUBSYS_H1T,
      TRACE_SUBSYS_SDBV,            /* SDB-Verteiler */
      TRACE_SUBSYS_WD,              /* Watchdog */
      TRACE_SUBSYS_SNMP,            /* SNMP-Agent */
      TRACE_SUBSYS_WEBDIAG_AGENT,   /* Wegbdiag Agent */
      TRACE_SUBSYS_FWLDA,
      TRACE_SUBSYS_SYSA_MISC,
      TRACE_SUBSYS_SYSA_MRP,

      /******************************************************************/
      /* MINIWEB */
      TRACE_SUBSYS_MINIWEB,
      TRACE_SUBSYS_MINIWEB_DEFAPP,
      TRACE_SUBSYS_MINIWEB_HTTPCORE,
      TRACE_SUBSYS_MINIWEB_CORE,
      TRACE_SUBSYS_MINIWEB_FILESYS,
      TRACE_SUBSYS_MINIWEB_HTTPSRVC,
      TRACE_SUBSYS_MINIWEB_SECURITY,
      TRACE_SUBSYS_MINIWEB_SOCKSSL,
      TRACE_SUBSYS_MINIWEB_XML,
      TRACE_SUBSYS_MINIWEB_MOBILE,
      TRACE_SUBSYS_MINIWEB_CPVP,
      TRACE_SUBSYS_MINIWEB_MWSL,
      TRACE_SUBSYS_MINIWEB_MWSL2,
      TRACE_SUBSYS_MINIWEB_S7APP,
      TRACE_SUBSYS_MINIWEB_S7SRV,

      /*********************************************************************/
      /* IT-Subsysteme */
      TRACE_SUBSYS_FTP,
      TRACE_SUBSYS_FTPCLIENT,
      TRACE_SUBSYS_HTTP,
      TRACE_SUBSYS_S7_SERV,

      TRACE_SUBSYS_RTS_BASE,
      TRACE_SUBSYS_RTSP_BASE,
      TRACE_SUBSYS_RTSP_RFC,

      TRACE_SUBSYS_FM400,           /* FM400 Treiber */
      TRACE_SUBSYS_ETCRDRV,         /* FM400 Treiber */

      /*********************************************************************/
      /* TPAI + BACNET Subsysteme */
      TRACE_SUBSYS_TPAI,
      TRACE_SUBSYS_BACNET_GTW,
      TRACE_SUBSYS_BACNET_STAC,
      
      /*********************************************************************/
      /* COM_COMPS:  */
      TRACE_SUBSYS_PCPNIO_INIT,
      TRACE_SUBSYS_PCPNIO_STATE,
      TRACE_SUBSYS_PCPNIO_IO,
      TRACE_SUBSYS_PCPNIO_DS,
      TRACE_SUBSYS_PCPNIO_ALARM,
      TRACE_SUBSYS_PCPNIO_MGT,
      TRACE_SUBSYS_PCPNIO_CHNL,
      TRACE_SUBSYS_PCPNIO_PCOVS,
      TRACE_SUBSYS_PCPNIO_ENVLUSR,
      TRACE_SUBSYS_PCPNIO_ENVLP,
      TRACE_SUBSYS_PCPNIO_UTRANS,
      TRACE_SUBSYS_PCPNIO_AGENT,
    
      /*********************************************************************/
      /* DPS */
	   TRACE_SUBSYS_DPS_MGM, 
	   TRACE_SUBSYS_DPS_C0,
	   TRACE_SUBSYS_DPS_V1SL,
	   TRACE_SUBSYS_DPS_TIS,
	   TRACE_SUBSYS_DPS_SYSIF,
	   TRACE_SUBSYS_DPS_XBUS,

      /*********************************************************************/
      /* DP-Master */
      TRACE_SUBSYS_DP_SDBDP,

      /*********************************************************************/
      /* PBK */
      TRACE_SUBSYS_PBK,
      TRACE_SUBSYS_PBK_KBUSVERB,
      TRACE_SUBSYS_PBK_SERVER,
      TRACE_SUBSYS_PBK_OPMUX,
      TRACE_SUBSYS_PBK_PBUS,
      TRACE_SUBSYS_PBK_PBUS_01,
      TRACE_SUBSYS_PBK_PBUS_02,
      TRACE_SUBSYS_PBK_PBUS_03,
      TRACE_SUBSYS_PBK_PBUS_04,
      TRACE_SUBSYS_PBK_PBUS_05,
      TRACE_SUBSYS_PBK_PBUS_06,
      TRACE_SUBSYS_PBK_PBUS_07,
      TRACE_SUBSYS_PBK_PBUS_08,
      TRACE_SUBSYS_PBK_PBUS_09,
      TRACE_SUBSYS_PBK_PBUS_10,
      TRACE_SUBSYS_PBK_PBUS_11,
      TRACE_SUBSYS_PBK_PBUS_12,
      TRACE_SUBSYS_PBK_PBUS_13,
      TRACE_SUBSYS_PBK_PBUS_14,
      TRACE_SUBSYS_PBK_PBUS_15,
      TRACE_SUBSYS_PBK_CLIENT,
      TRACE_SUBSYS_PBK_CLIENT_01,
      TRACE_SUBSYS_PBK_CLIENT_02,
      TRACE_SUBSYS_PBK_CLIENT_03,
      TRACE_SUBSYS_PBK_CLIENT_04,
      TRACE_SUBSYS_PBK_CLIENT_05,
      TRACE_SUBSYS_PBK_CLIENT_06,
      TRACE_SUBSYS_PBK_CLIENT_07,
      TRACE_SUBSYS_PBK_CLIENT_08,
      TRACE_SUBSYS_PBK_CLIENT_09,
      TRACE_SUBSYS_PBK_CLIENT_10,
      TRACE_SUBSYS_PBK_CLIENT_11,
      TRACE_SUBSYS_PBK_CLIENT_12,
      TRACE_SUBSYS_PBK_CLIENT_13,
      TRACE_SUBSYS_PBK_CLIENT_14,
      TRACE_SUBSYS_PBK_CLIENT_15,
      TRACE_SUBSYS_PBK_VERB,
      TRACE_SUBSYS_PBK_VERB_01,
      TRACE_SUBSYS_PBK_VERB_02,
      TRACE_SUBSYS_PBK_VERB_03,
      TRACE_SUBSYS_PBK_VERB_04,
      TRACE_SUBSYS_PBK_VERB_05,
      TRACE_SUBSYS_PBK_VERB_06,
      TRACE_SUBSYS_PBK_VERB_07,
      TRACE_SUBSYS_PBK_VERB_08,
      TRACE_SUBSYS_PBK_VERB_09,
      TRACE_SUBSYS_PBK_VERB_10,
      TRACE_SUBSYS_PBK_VERB_11,
      TRACE_SUBSYS_PBK_VERB_12,
      TRACE_SUBSYS_PBK_VERB_13,
      TRACE_SUBSYS_PBK_VERB_14,
      TRACE_SUBSYS_PBK_VERB_15,

      /*************************************************************************/
      /* CBA */
      TRACE_SUBSYS_CBA_SYSA,
      TRACE_SUBSYS_CBA_RPC_SOCK,
      TRACE_SUBSYS_CBA_RPC,
      TRACE_SUBSYS_CBA_DCOM,
      TRACE_SUBSYS_CBA_APPL_AMARSHAL,
      TRACE_SUBSYS_CBA_APPL_S7,
      TRACE_SUBSYS_CBA_APPL_ACCO,
      TRACE_SUBSYS_CBA_APPL_DEVICE,
      TRACE_SUBSYS_CBA_GATE,
      TRACE_SUBSYS_CBA_TRACE,
      TRACE_SUBSYS_CBA_SRT,
	  TRACE_SUBSYS_CBA_KBUS,
      
      /* CBA-DIAG */
      TRACE_SUBSYS_CBA_DIAG,                    /* Base */
      TRACE_SUBSYS_CBA_DIAG_GSI_SOCK_CON,       /* wGsiSockCon; */
      TRACE_SUBSYS_CBA_DIAG_GSI_ADP,            /* wGsiAdapter; */
      TRACE_SUBSYS_CBA_DIAG_RPC_CCONTEXT,       /* wRpcClientContext; */
      TRACE_SUBSYS_CBA_DIAG_RPC_SCONTEXT,       /* wRPCServerContext; */
      TRACE_SUBSYS_CBA_DIAG_RPC_MOD,            /* wRPCMod; */
      TRACE_SUBSYS_CBA_DIAG_RPC_REGIF,          /* wRpcRegInterface; */
      TRACE_SUBSYS_CBA_DIAG_DCOM_STAT,          /* wDComStat; */
      TRACE_SUBSYS_CBA_DIAG_DCOM_RUNOBJ,        /* wDComRunningObj; */
      TRACE_SUBSYS_CBA_DIAG_DCOM_CIFC,          /* wDComClientIfc; */
      TRACE_SUBSYS_CBA_DIAG_DCOM_SIFC,          /* wDComServerIfc; */
      TRACE_SUBSYS_CBA_DIAG_PDEV,               /* wPDev; */
      TRACE_SUBSYS_CBA_DIAG_LDEV,               /* wLDev; */
      TRACE_SUBSYS_CBA_DIAG_RTAUTO,             /* wRtAuto; */
      TRACE_SUBSYS_CBA_DIAG_PUTPROP,            /* wPutProperty; */
      TRACE_SUBSYS_CBA_DIAG_GETPROP,            /* wGetProperty; */
      TRACE_SUBSYS_CBA_DIAG_ACCO,               /* wAcco; */
      TRACE_SUBSYS_CBA_DIAG_CONSCONN,           /* wConsumerConn; */
      TRACE_SUBSYS_CBA_DIAG_PING,               /* wPing; */
      TRACE_SUBSYS_CBA_DIAG_PROVCONN,           /* wProviderConn; */
      TRACE_SUBSYS_CBA_DIAG_ADVISE,             /* wAdvise; */
      TRACE_SUBSYS_CBA_DIAG_LDEVSTATEADVISE,    /* wLdevStateAdvise; */
      TRACE_SUBSYS_CBA_DIAG_LDEVGRERRADVISE,    /* wLdevGroupErr; */
      TRACE_SUBSYS_CBA_DIAG_ACCOGRERRADVISE,    /* wAccoGroupErr; */
      TRACE_SUBSYS_CBA_DIAG_AM_STAT,            /* wAmStatistics; */
      TRACE_SUBSYS_CBA_DIAG_GATECHANNEL,        /* wGateChannel */

      /*************************************************************************/
      /* CBA */
      /* CP1628 */
      TRACE_SUBSYS_ADMMAIN,
      TRACE_SUBSYS_ADMFLASH,
      TRACE_SUBSYS_CLOCK,
      TRACE_SUBSYS_ISO,
      TRACE_SUBSYS_MUXBIND,
      TRACE_SUBSYS_NDIR,
      TRACE_SUBSYS_S7ONTCP,
      TRACE_SUBSYS_SMDPRAM,
      TRACE_SUBSYS_SNMP1628,
      TRACE_SUBSYS_FUSION,
      TRACE_SUBSYS_TARGET,

      /*************************************************************************/
      /* PBUS Plus */

      /* subsystems of component PBUSP base package */
      TRACE_SUBSYS_PBUSP_ALARM,
      TRACE_SUBSYS_PBUSP_ALARM_RCV,
      TRACE_SUBSYS_PBUSP_ALARM_SND,
      TRACE_SUBSYS_PBUSP_AZK,
      TRACE_SUBSYS_PBUSP_AZK_RCV,
      TRACE_SUBSYS_PBUSP_AZK_SND,
      TRACE_SUBSYS_PBUSP_BUS,
      TRACE_SUBSYS_PBUSP_CLOCK,
      TRACE_SUBSYS_PBUSP_IO,

      /* PBUSP base package -> subsystems of Slave */
      TRACE_SUBSYS_PSP_ALARM,
      TRACE_SUBSYS_PSP_BUSOBSERVE,
      TRACE_SUBSYS_PSP_IO,
      TRACE_SUBSYS_MOD_INTEGR,

      /*************************************************************************/
      /* Device Proxy für CP1616 */
      TRACE_SUBSYS_PE_PROXY,
      TRACE_SUBSYS_DPRLIB_FW,
      TRACE_SUBSYS_DPRLIB_INTERN,
      TRACE_SUBSYS_DPR_SW,

      /*************************************************************************/
   TRACE_SUBSYS_NUM    /* dont change the last entry, its necessary for ltrc */
};
#define LTRC_SUBSYS_TYPE enum ltrc_subsys




/*=============================================================================
 * (2) ltrc_subsys_cfg_table
 *===========================================================================*/

/* const subsys info table for ltrc:                                                       */
/* - column 1: LSA_UINT32 - subsys id (see enum ltrc_subsys)                               */
/* - column 2: LSA_UINT32 - number of instance                                             */
/* - column 3: LSA_CHAR[LTRC_TEXT_STRING_LEN - ]subsys name (max length is 39)             */
/* - column 4: LTRC_ACTPAS_TYPE - default, if the subsystem should be activ oder passiv    */
/* - column 5: LTRC_GROUP_TYPE - to user controll function, you can call with group number */
/* - column 6: LTRC_BUFFER_TYPE - default, to this buffer the subsystem will be trace to   */

LSA_EXTERN
const  LTRC_ADVTEXT_INFO_TYPE    LSA_CONST_MEM_ATTR
ltrc_subsys_cfg_table [];

#if defined LTRC_SET_SUBSYS_CFG_TABLE

const  LTRC_ADVTEXT_INFO_TYPE    LSA_CONST_MEM_ATTR
ltrc_subsys_cfg_table [] =
{
   /*   LSA_UINT16,                     LSA_UINT16,  LSA_CHAR                    LTRC_ACTPAS_TYPE,     LTRC_GROUP_TYPE,      LTRC_BUFFER_TYPE,      LTRC_LEVEL_TYPE*/
   /*   subsysId,                         instance,  name[..]                    action,               controllGroup,        buffer,                level          */
   /* dont change the first entry, its necessary for ltrc */
   { TRACE_SUBSYS_TRACE,                      1,  "[TRACE]"},
   /*************************************************************************/
      { TRACE_SUBSYS_ACL,                        1,  "[ACL]",                    TRACE_SUBSYS_PASSIVE, TRACE_GROUP_BOARD,    TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*              CP3431                  CP4431           */
      { TRACE_SUBSYS_BSD_SELECT,                 1,  "[BSD][SELECT]",            TRACE_SUBSYS_PASSIVE, TRACE_GROUP_BOARD,    TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },
      { TRACE_SUBSYS_BSD_USER,                   1,  "[BSD][USER]",              TRACE_SUBSYS_PASSIVE, TRACE_GROUP_BOARD,    TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },
      { TRACE_SUBSYS_BSD_VXADAPT,                1,  "[BSD][VXADAPT]",           TRACE_SUBSYS_PASSIVE, TRACE_GROUP_BOARD,    TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },
      { TRACE_SUBSYS_BUB,                        1,  "[BUB]",                    TRACE_SUBSYS_PASSIVE, TRACE_GROUP_BOARD,    TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431                  CP4431           */
      { TRACE_SUBSYS_DICON,                      1,  "[DICON]",                  TRACE_SUBSYS_PASSIVE, TRACE_GROUP_BOARD,    TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*                             ertec400        edd_vxw   */
      { TRACE_SUBSYS_FWDL,                       1,  "[FWDL]",                   TRACE_SUBSYS_PASSIVE, TRACE_GROUP_BOARD,    TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },
      { TRACE_SUBSYS_IPCONFIG,                   1,  "[IPCONFIG]",               TRACE_SUBSYS_PASSIVE, TRACE_GROUP_BOARD,    TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616 CP3425 CP3431                  CP4431           */
      { TRACE_SUBSYS_MGT,                        1,  "[MGT]",                    TRACE_SUBSYS_PASSIVE, TRACE_GROUP_BOARD,    TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616 CP3425 CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_OVS,                        4,  "[OVS]",                    TRACE_SUBSYS_PASSIVE, TRACE_GROUP_BOARD,    TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616 CP3425 CP3431 Softnet          CP4431           */
      { TRACE_SUBSYS_PANEL,                      1,  "[PANEL]",                  TRACE_SUBSYS_PASSIVE, TRACE_GROUP_BOARD,    TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*                                      CP4431           */
      { TRACE_SUBSYS_PGTASK,                     2,  "[PGTASK]",                 TRACE_SUBSYS_PASSIVE, TRACE_GROUP_BOARD,    TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616 CP3425 CP3431                  CP4431           */
      { TRACE_SUBSYS_SNDRCV,                     1,  "[SNDRCV]",                 TRACE_SUBSYS_PASSIVE, TRACE_GROUP_BOARD,    TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*              CP3431                  CP4431           */
      { TRACE_SUBSYS_TIMESYNC,                   1,  "[TIMESYNC]",               TRACE_SUBSYS_PASSIVE, TRACE_GROUP_BOARD,    TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*              CP3431                  CP4431           */
      { TRACE_SUBSYS_VARSERV,                    1,  "[VARSERV]",                TRACE_SUBSYS_PASSIVE, TRACE_GROUP_BOARD,    TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*              CP3431                  CP4431           */
      { TRACE_SUBSYS_TIS,                        2,  "[TIS]",                    TRACE_SUBSYS_PASSIVE, TRACE_GROUP_BOARD,    TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616 CP3425 CP3431 Softnet          CP4431           */
      { TRACE_SUBSYS_TREL,                       1,  "[TREL]",                   TRACE_SUBSYS_PASSIVE, TRACE_GROUP_BOARD,    TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*       CP3425 CP3431                  CP4431           */
      { TRACE_SUBSYS_SECURITY,                   1,  "[SECURITY]",               TRACE_SUBSYS_PASSIVE, TRACE_GROUP_BOARD,    TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*                                      CP4431           */

      /*************************************************************************/
      /* le_nbg2 -- ICON */
      { TRACE_SUBSYS_ICON_CHANNEL,               1,  "[ICON][CHANNEL]",          TRACE_SUBSYS_PASSIVE, TRACE_GROUP_ICON,     TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },
      { TRACE_SUBSYS_ICON_FLICT,                 1,  "[ICON][CONFLICT]",         TRACE_SUBSYS_PASSIVE, TRACE_GROUP_ICON,     TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },
      { TRACE_SUBSYS_ICON_MGT,                   1,  "[ICON][MGT]",              TRACE_SUBSYS_PASSIVE, TRACE_GROUP_ICON,     TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },
      { TRACE_SUBSYS_ICON_PDEV,                  1,  "[ICON][PDEV]",             TRACE_SUBSYS_PASSIVE, TRACE_GROUP_ICON,     TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },
      { TRACE_SUBSYS_ICON_PSU,                   1,  "[ICON][PSU]",              TRACE_SUBSYS_PASSIVE, TRACE_GROUP_ICON,     TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },
      { TRACE_SUBSYS_ICON_REMA,                  1,  "[ICON][REMA]",             TRACE_SUBSYS_PASSIVE, TRACE_GROUP_ICON,     TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },
      { TRACE_SUBSYS_ICON_TIS,                   1,  "[ICON][TIS]",              TRACE_SUBSYS_PASSIVE, TRACE_GROUP_ICON,     TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },
      
      { TRACE_SUBSYS_IPC,                        1,  "[IPC]",                    TRACE_SUBSYS_PASSIVE, TRACE_GROUP_ICON,     TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      /*************************************************************************/
      /* PT2_LSA -- LSA-Komponenten */
      { TRACE_SUBSYS_ACP_UPPER,                  1,  "[ACP][UPPER]",             TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_ACP_LOWER,                  1,  "[ACP][LOWER]",             TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_ACP_RTA,                    1,  "[ACP][RTA]",               TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_ACP_MEMORY,                 1,  "[ACP][MEMORY]",            TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
  
      { TRACE_SUBSYS_CLRPC_UPPER,                1,  "[CLRPC][UPPER]",           TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_CLRPC_CL,                   1,  "[CLRPC][CL]",              TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_CLRPC_SV,                   1,  "[CLRPC][SV]",              TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_CLRPC_EPM,                  1,  "[CLRPC][EPM]",             TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_CLRPC_LOWER,                1,  "[CLRPC][LOWER]",           TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_CLRPC_CL_PKT,               1,  "[CLRPC][CL][PKT]",         TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_CLRPC_SV_PKT,               1,  "[CLRPC][SV][PKT]",         TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */

      { TRACE_SUBSYS_CM_ACP,                     1,  "[CM][ACP]",                TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_CM_AR,                      1,  "[CM][AR]",                 TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_CM_CL,                      1,  "[CM][CL]",                 TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_CM_MC,                      1,  "[CM][MC]",                 TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_CM_MEM,                     1,  "[CM][MEM]",                TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_CM_NARE,                    1,  "[CM][NARE]",               TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_CM_RPC,                     1,  "[CM][RPC]",                TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_CM_SV,                      1,  "[CM][SV]",                 TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_CM_UPPER,                   1,  "[CM][UPPER]",              TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_CM_EDD,                     1,  "[CM][EDD]",                TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_CM_GSY,                     1,  "[CM][GSY]",                TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_CM_OHA,                     1,  "[CM][OHA]",                TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_CM_PD,                      1,  "[CM][PD]",                 TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_CM_MRP,                     1,  "[CM][MRP]",                TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_CM_POF,                     1,  "[CM][POF]",                TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_CM_REMA,                    1,  "[CM][REMA]",               TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */

      { TRACE_SUBSYS_DCP_UPPER,                  1,  "[DCP][UPPER]",             TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_DCP_LOWER,                  1,  "[DCP][LOWER]",             TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_DCP_SYSTEM,                 1,  "[DCP][SYSTEM]",            TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_DCP_ERROR,                  1,  "[DCP][ERROR]",             TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_DCP_FUNCTION,               1,  "[DCP][FUNCTION]",          TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_DCP_SERVER,                 1,  "[DCP][SERVER]",            TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_DCP_SRVERR,                 1,  "[DCP][SRVERR]",            TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_DCP_SNDRCV,                 1,  "[DCP][SNDRCV]",            TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616 */
      
      { TRACE_SUBSYS_GSY_UPPER,                  1,  "[GSY][UPPER]",             TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_GSY_LOWER,                  1,  "[GSY][LOWER]",             TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_GSY_SYSTEM,                 1,  "[GSY][SYSTEM]",            TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_GSY_ERROR,                  1,  "[GSY][ERROR]",             TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_GSY_FUNCTION,               1,  "[GSY][FUNCTION]",          TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_GSY_DIAG,                   1,  "[GSY][DIAG]",              TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_GSY_DEL,                    1,  "[GSY][DEL]",               TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_GSY_FWD,                    1,  "[GSY][FWD]",               TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_GSY_SNDRCV,                 1,  "[GSY][SNDRCV]",            TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_GSY_PRM,                    1,  "[GSY][PRM]",               TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_GSY_SYNC,                   1,  "[GSY][SYNC]",              TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_GSY_MASTER,                 1,  "[GSY][MASTER]",            TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */

      { TRACE_SUBSYS_NARE_UPPER,                 1,  "[NARE][UPPER]",            TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_NARE_LOWER,                 1,  "[NARE][LOWER]",            TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_NARE_SYSTEM,                1,  "[NARE][SYSTEM]",           TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_NARE_FUNCTION,              1,  "[NARE][FUNCTION]",         TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_NARE_PROGRAM,               1,  "[NARE][PROGRAM]",          TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */

      { TRACE_SUBSYS_LSYS_GLOB,                  1,  "[LSYS][GLOB]",             TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616 CP3425 CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_LSYS_CM,                    1,  "[LSYS][CM]",               TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*                     Softnet ertec400        edd_vxw   */
      { TRACE_SUBSYS_LSYS_PSU,                   1,  "[LSYS][PSU]",              TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*                     Softnet ertec400        edd_vxw   */
      { TRACE_SUBSYS_LSYS_LTRC,                  1,  "[LSYS][LTRC]",             TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*       CP3425        Softnet ertec400        edd_vxw   */
      
      { TRACE_SUBSYS_LLDP_UPPER,                 1,  "[LLDP][UPPER]",            TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_LLDP_LOWER,                 1,  "[LLDP][LOWER]",            TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_LLDP_SYSTEM,                1,  "[LLDP][SYSTEM]",           TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_LLDP_FUNCTION,              1,  "[LLDP][FUNCTION]",         TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_LLDP_PROGRAM,               1,  "[LLDP][PROGRAM]",          TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */

      { TRACE_SUBSYS_MRP_UPPER,                  1,  "[MRP][UPPER]",             TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_MRP_LOWER,                  1,  "[MRP][LOWER]",             TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_MRP_SYSTEM,                 1,  "[MRP][SYSTEM]",            TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_MRP_FUNCTION,               1,  "[MRP][FUNCTION]",          TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*                     Softnet ertec400        edd_vxw   */
      { TRACE_SUBSYS_MRP_PROGRAM,                1,  "[MRP][PROGRAM]",           TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */

      { TRACE_SUBSYS_OHA_UPPER,                  1,  "[OHA][UPPER]",             TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_OHA_LOWER,                  1,  "[OHA][LOWER]",             TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_OHA_SYSTEM,                 1,  "[OHA][SYSTEM]",            TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_OHA_FUNCTION,               1,  "[OHA][FUNCTION]",          TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_OHA_PROGRAM,                1,  "[OHA][PROGRAM]",           TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */

      { TRACE_SUBSYS_POF_UPPER,                  1,  "[POF][UPPER]",             TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_POF_LOWER,                  1,  "[POF][LOWER]",             TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_POF_SYSTEM,                 1,  "[POF][SYSTEM]",            TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_POF_FUNCTION,               1,  "[POF][FUNCTION]",          TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_POF_PROGRAM,                1,  "[POF][PROGRAM]",           TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */

      { TRACE_SUBSYS_SOCK_UPPER,                 1,  "[SOCK][UPPER]",            TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_SOCK_LOWER,                 1,  "[SOCK][LOWER]",            TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_SOCK_SYSTEM,                1,  "[SOCK][SYSTEM]",           TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_SOCK_PROTOCOL,              1,  "[SOCK][PROTOCOL]",         TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */
      { TRACE_SUBSYS_SOCK_STACK,                 1,  "[SOCK][STACK]",            TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431 edd_vxw   */

      { TRACE_SUBSYS_EDDI_UPPER,                 1,  "[EDDI][UPPER]",            TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA_EDDI, TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },
      { TRACE_SUBSYS_EDDI_LOWER,                 1,  "[EDDI][LOWER]",            TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA_EDDI, TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },
      { TRACE_SUBSYS_EDDI_SYSTEM,                1,  "[EDDI][SYSTEM]",           TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA_EDDI, TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },
      { TRACE_SUBSYS_EDDI_FUNCTION,              1,  "[EDDI][FUNCTION]",         TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA_EDDI, TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },
      { TRACE_SUBSYS_EDDI_PROGRAM,               1,  "[EDDI][PROGRAM]",          TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA_EDDI, TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },
      { TRACE_SUBSYS_EDDI_NRT,                   1,  "[EDDI][NRT]",              TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA_EDDI, TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },
      { TRACE_SUBSYS_EDDI_PRM,                   1,  "[EDDI][PRM]",              TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA_EDDI, TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },
      { TRACE_SUBSYS_EDDI_SWI,                   1,  "[EDDI][SWI]",              TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA_EDDI, TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },
      { TRACE_SUBSYS_EDDI_CRT,                   1,  "[EDDI][CRT]",              TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA_EDDI, TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },
      { TRACE_SUBSYS_EDDI_SYNC,                  1,  "[EDDI][SYNC]",             TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA_EDDI, TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },

      { TRACE_SUBSYS_EDDS_UPPER,                 1,  "[EDDS][UPPER]",            TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA_EDDS, TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },
      { TRACE_SUBSYS_EDDS_LOWER,                 1,  "[EDDS][LOWER]",            TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA_EDDS, TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },
      { TRACE_SUBSYS_EDDS_SYSTEM,                1,  "[EDDS][SYSTEM]",           TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA_EDDS, TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },
      { TRACE_SUBSYS_EDDS_FUNCTION,              1,  "[EDDS][FUNCTION]",         TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA_EDDS, TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },
      { TRACE_SUBSYS_EDDS_PROGRAM,               1,  "[EDDS][PROGRAM]",          TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA_EDDS, TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },
      { TRACE_SUBSYS_EDDS_NRT,                   1,  "[EDDS][NRT]",              TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA_EDDS, TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },
      { TRACE_SUBSYS_EDDS_PRM,                   1,  "[EDDS][PRM]",              TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA_EDDS, TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },
      { TRACE_SUBSYS_EDDS_SWI,                   1,  "[EDDS][SWI]",              TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA_EDDS, TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },
      { TRACE_SUBSYS_EDDS_CRT,                   1,  "[EDDS][CRT]",              TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA_EDDS, TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },
      { TRACE_SUBSYS_EDDS_SYNC,                  1,  "[EDDS][SYNC]",             TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA_EDDS, TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },

      { TRACE_SUBSYS_EDDP_UPPER,                 1,  "[EDDP][UPPER]",            TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA_EDDP, TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },
      { TRACE_SUBSYS_EDDP_LOWER,                 1,  "[EDDP][LOWER]",            TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA_EDDP, TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },
      { TRACE_SUBSYS_EDDP_SYSTEM,                1,  "[EDDP][SYSTEM]",           TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA_EDDP, TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },
      { TRACE_SUBSYS_EDDP_FUNCTION,              1,  "[EDDP][FUNCTION]",         TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA_EDDP, TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },
      { TRACE_SUBSYS_EDDP_PROGRAM,               1,  "[EDDP][PROGRAM]",          TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA_EDDP, TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },
      { TRACE_SUBSYS_EDDP_NRT,                   1,  "[EDDP][NRT]",              TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA_EDDP, TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },
      { TRACE_SUBSYS_EDDP_PRM,                   1,  "[EDDP][PRM]",              TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA_EDDP, TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },
      { TRACE_SUBSYS_EDDP_SWI,                   1,  "[EDDP][SWI]",              TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA_EDDP, TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },
      { TRACE_SUBSYS_EDDP_CRT,                   1,  "[EDDP][CRT]",              TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA_EDDP, TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },
      { TRACE_SUBSYS_EDDP_SYNC,                  1,  "[EDDP][SYNC]",             TRACE_SUBSYS_PASSIVE, TRACE_GROUP_LSA_EDDP, TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },

      /*************************************************************************/
      /* PNIO */
      { TRACE_SUBSYS_PNIO_AGET300,               1,  "[PNIO][AGET300]",          TRACE_SUBSYS_PASSIVE, TRACE_GROUP_PNIO,     TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet                           */
      { TRACE_SUBSYS_PNIO_AGET400,               1,  "[PNIO][AGET400]",          TRACE_SUBSYS_PASSIVE, TRACE_GROUP_PNIO,     TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet          CP4431           */
      { TRACE_SUBSYS_PNIO_CTRL_GLOB,             1,  "[PNIO][CTRL][GLOB]",       TRACE_SUBSYS_PASSIVE, TRACE_GROUP_PNIO_CON, TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet          CP4431           */
      { TRACE_SUBSYS_PNIO_CTRL_ALERT,            1,  "[PNIO][CTRL][ALERT]",      TRACE_SUBSYS_PASSIVE, TRACE_GROUP_PNIO_CON, TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet                           */
      { TRACE_SUBSYS_PNIO_CTRL_DRRW,             1,  "[PNIO][CTRL][DRRW]",       TRACE_SUBSYS_PASSIVE, TRACE_GROUP_PNIO_CON, TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet                           */
      { TRACE_SUBSYS_PNIO_CTRL_MEM,              1,  "[PNIO][CTRL][MEM]",        TRACE_SUBSYS_PASSIVE, TRACE_GROUP_PNIO_CON, TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet                           */
      { TRACE_SUBSYS_PNIO_CTRL_SDB,              1,  "[PNIO][CTRL][SDB]",        TRACE_SUBSYS_PASSIVE, TRACE_GROUP_PNIO_CON, TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet          CP4431           */
      { TRACE_SUBSYS_PNIO_CTRL_STATE,            1,  "[PNIO][CTRL][STATE]",      TRACE_SUBSYS_PASSIVE, TRACE_GROUP_PNIO_CON, TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet                           */
      { TRACE_SUBSYS_PNIO_ARM_UPDATE,            1,  "[PNIO][ARM][UPDATE]",      TRACE_SUBSYS_PASSIVE, TRACE_GROUP_PNIO,     TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },
      { TRACE_SUBSYS_PNIO_AGENTDEVK_CTRL,        1,  "[PNIO][AGENTDEVK][CTRL]",  TRACE_SUBSYS_PASSIVE, TRACE_GROUP_PNIO,     TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616                       ertec400                  */
      { TRACE_SUBSYS_PNIO_AGENTDEVK_DEV,         1,  "[PNIO][AGENTDEVK][DEV]",   TRACE_SUBSYS_PASSIVE, TRACE_GROUP_PNIO,     TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616                       ertec400        edd_vxw   */
      
      { TRACE_SUBSYS_IOD,                        1,  "[PNIO][DEVICE][BASE]",     TRACE_SUBSYS_PASSIVE, TRACE_GROUP_PNIO,     TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431         ertec400        edd_vxw   */
      { TRACE_SUBSYS_SDEV,                       1,  "[PNIO][DEVICE][SDEV]",     TRACE_SUBSYS_PASSIVE, TRACE_GROUP_PNIO,     TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*                             ertec400        edd_vxw   */
      { TRACE_SUBSYS_APPL_IOD,                   1,  "[PNIO][DEVICE][BASE][APL]",TRACE_SUBSYS_PASSIVE, TRACE_GROUP_PNIO,     TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*                             ertec400        edd_vxw   */
      { TRACE_SUBSYS_IOD_TESTAPPL,               1,  "[PNIO][DEVICE][TEST]",     TRACE_SUBSYS_PASSIVE, TRACE_GROUP_PNIO,     TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*                             ertec400        edd_vxw   */
      
      { TRACE_SUBSYS_IOR,                        1,  "[PNIO][IO][ROUTER]",       TRACE_SUBSYS_PASSIVE, TRACE_GROUP_PNIO,     TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616                                                 */
      { TRACE_SUBSYS_L2_AGENT,                   1,  "[PNIO][L2][AGENT]",        TRACE_SUBSYS_PASSIVE, TRACE_GROUP_PNIO,     TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616                                                 */

      /*********************************************************************/
      /* H1T */
      { TRACE_SUBSYS_H1T,                        1,  "[H1T]",                    TRACE_SUBSYS_PASSIVE, TRACE_GROUP_H1T,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616                                                 */
      { TRACE_SUBSYS_SDBV,                       1,  "[SDBV]",                   TRACE_SUBSYS_PASSIVE, TRACE_GROUP_H1T,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431 Softnet ertec400 CP4431           */
      { TRACE_SUBSYS_WD,                         1,  "[WD]",                     TRACE_SUBSYS_PASSIVE, TRACE_GROUP_H1T,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616                       ertec400 CP4431           */
      { TRACE_SUBSYS_SNMP,                       1,  "[SNMP]",                   TRACE_SUBSYS_PASSIVE, TRACE_GROUP_H1T,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616        CP3431         ertec400 CP4431           */
      { TRACE_SUBSYS_WEBDIAG_AGENT,              1,  "[WEBDIAG][AGENT]",         TRACE_SUBSYS_PASSIVE, TRACE_GROUP_H1T,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616                       ertec400                  */
      { TRACE_SUBSYS_FWLDA,                      1,  "[FWLDA]",                  TRACE_SUBSYS_PASSIVE, TRACE_GROUP_H1T,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616                                                 */
      { TRACE_SUBSYS_SYSA_MISC,                  1,  "[SYSA][MISC]",             TRACE_SUBSYS_PASSIVE, TRACE_GROUP_H1T,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616                                                 */
      { TRACE_SUBSYS_SYSA_MRP,                   1,  "[SYSA][MRP]",              TRACE_SUBSYS_PASSIVE, TRACE_GROUP_H1T,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616                                                 */
      

      /*********************************************************************/
      /* MINIWEB */
      { TRACE_SUBSYS_MINIWEB,                    1,  "[MINIWEB]",                TRACE_SUBSYS_PASSIVE, TRACE_GROUP_MINIWEB,  TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616                                                 */
      { TRACE_SUBSYS_MINIWEB_DEFAPP,             1,  "[MINIWEB][DEFAPP]",        TRACE_SUBSYS_PASSIVE, TRACE_GROUP_MINIWEB,  TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616                                                 */
      { TRACE_SUBSYS_MINIWEB_HTTPCORE,           1,  "[MINIWEB][HTTPCORE]",      TRACE_SUBSYS_PASSIVE, TRACE_GROUP_MINIWEB,  TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616                                                 */
      { TRACE_SUBSYS_MINIWEB_CORE,               1,  "[MINIWEB][CORE]",          TRACE_SUBSYS_PASSIVE, TRACE_GROUP_MINIWEB,  TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616                                                 */
      { TRACE_SUBSYS_MINIWEB_FILESYS,            1,  "[MINIWEB][FILESYS]",       TRACE_SUBSYS_PASSIVE, TRACE_GROUP_MINIWEB,  TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616                                                 */
      { TRACE_SUBSYS_MINIWEB_HTTPSRVC,           1,  "[MINIWEB][HTTPSRVC]",      TRACE_SUBSYS_PASSIVE, TRACE_GROUP_MINIWEB,  TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616                                                 */
      { TRACE_SUBSYS_MINIWEB_SECURITY,           1,  "[MINIWEB][SECURITY]",      TRACE_SUBSYS_PASSIVE, TRACE_GROUP_MINIWEB,  TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616                                                 */
      { TRACE_SUBSYS_MINIWEB_SOCKSSL,            1,  "[MINIWEB][SOCKSSL]",       TRACE_SUBSYS_PASSIVE, TRACE_GROUP_MINIWEB,  TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616                                                 */
      { TRACE_SUBSYS_MINIWEB_XML,                1,  "[MINIWEB][XML]",           TRACE_SUBSYS_PASSIVE, TRACE_GROUP_MINIWEB,  TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616                                                 */
      { TRACE_SUBSYS_MINIWEB_MOBILE,             1,  "[MINIWEB][MOBILE]",        TRACE_SUBSYS_PASSIVE, TRACE_GROUP_MINIWEB,  TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616                                                 */
      { TRACE_SUBSYS_MINIWEB_CPVP,               1,  "[MINIWEB][CPVP]",          TRACE_SUBSYS_PASSIVE, TRACE_GROUP_MINIWEB,  TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616                                                 */
      { TRACE_SUBSYS_MINIWEB_MWSL,               1,  "[MINIWEB][MWSL]",          TRACE_SUBSYS_PASSIVE, TRACE_GROUP_MINIWEB,  TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616                                                 */
      { TRACE_SUBSYS_MINIWEB_MWSL2,              1,  "[MINIWEB][MWSL2]",         TRACE_SUBSYS_PASSIVE, TRACE_GROUP_MINIWEB,  TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616                                                 */
      { TRACE_SUBSYS_MINIWEB_S7APP,              1,  "[MINIWEB][S7APP]",         TRACE_SUBSYS_PASSIVE, TRACE_GROUP_MINIWEB,  TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616                                                 */
      { TRACE_SUBSYS_MINIWEB_S7SRV,              1,  "[MINIWEB][S7SRV]",         TRACE_SUBSYS_PASSIVE, TRACE_GROUP_MINIWEB,  TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*CP1616                                                 */

      /*********************************************************************/
      /* IT-Subsysteme */
      { TRACE_SUBSYS_FTP,                        1,  "[FTP]",                    TRACE_SUBSYS_PASSIVE, TRACE_GROUP_ETHERNET, TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*              CP3431                  CP4431           */
      { TRACE_SUBSYS_FTPCLIENT,                  1,  "[FTPCLIENT]",              TRACE_SUBSYS_PASSIVE, TRACE_GROUP_ETHERNET, TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*              CP3431                  CP4431           */
      { TRACE_SUBSYS_HTTP,                       1,  "[HTTP]",                   TRACE_SUBSYS_PASSIVE, TRACE_GROUP_ETHERNET, TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*              CP3431                  CP4431           */
      { TRACE_SUBSYS_S7_SERV,                    1,  "[S7_SERV]",                TRACE_SUBSYS_PASSIVE, TRACE_GROUP_ETHERNET, TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*              CP3431         ertec400 CP4431           */

      { TRACE_SUBSYS_RTS_BASE,                   1,  "[RTS][BASE]",              TRACE_SUBSYS_PASSIVE, TRACE_GROUP_ETHERNET, TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*              CP3431                                   */
      { TRACE_SUBSYS_RTSP_BASE,                  1,  "[RTSP][BASE]",             TRACE_SUBSYS_PASSIVE, TRACE_GROUP_ETHERNET, TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*              CP3431                  CP4431           */
      { TRACE_SUBSYS_RTSP_RFC,                   1,  "[RTSP][RFC]",              TRACE_SUBSYS_PASSIVE, TRACE_GROUP_ETHERNET, TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*              CP3431                  CP4431           */

      { TRACE_SUBSYS_FM400,                      1,  "[FM400]",                  TRACE_SUBSYS_PASSIVE, TRACE_GROUP_ETHERNET, TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*                                      CP4431           */
      { TRACE_SUBSYS_ETCRDRV,                    1,  "[ETCRDRV]",                TRACE_SUBSYS_PASSIVE, TRACE_GROUP_ETHERNET, TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*                                      CP4431           */
      
      /*********************************************************************/
      /* TPAI + BACNET Subsysteme */
      { TRACE_SUBSYS_TPAI,                       1,  "[TPAI]",                   TRACE_SUBSYS_PASSIVE, TRACE_GROUP_BACNET,   TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*              CP3431                                   */
      { TRACE_SUBSYS_BACNET_GTW,                 1,  "[BACNET][GTW]",            TRACE_SUBSYS_PASSIVE, TRACE_GROUP_BACNET,   TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*              CP3431                                   */
      { TRACE_SUBSYS_BACNET_STAC,                1,  "[BACNET][STAC]",           TRACE_SUBSYS_PASSIVE, TRACE_GROUP_BACNET,   TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*              CP3431                                   */
      
      /*********************************************************************/
      /* COM_COMPS:  */
      { TRACE_SUBSYS_PCPNIO_INIT,                1,  "[PCPNIO][INIT]",           TRACE_SUBSYS_PASSIVE, TRACE_GROUP_COMPS,    TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*                     Softnet                           */
      { TRACE_SUBSYS_PCPNIO_STATE,               1,  "[PCPNIO][STATE]",          TRACE_SUBSYS_PASSIVE, TRACE_GROUP_COMPS,    TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*                     Softnet                           */
      { TRACE_SUBSYS_PCPNIO_IO,                  1,  "[PCPNIO][IO]",             TRACE_SUBSYS_PASSIVE, TRACE_GROUP_COMPS,    TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*                     Softnet                           */
      { TRACE_SUBSYS_PCPNIO_DS,                  1,  "[PCPNIO][DS]",             TRACE_SUBSYS_PASSIVE, TRACE_GROUP_COMPS,    TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*                     Softnet                           */
      { TRACE_SUBSYS_PCPNIO_ALARM,               1,  "[PCPNIO][ALARM]",          TRACE_SUBSYS_PASSIVE, TRACE_GROUP_COMPS,    TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*                     Softnet                           */
      { TRACE_SUBSYS_PCPNIO_MGT,                 1,  "[PCPNIO][MGT]",            TRACE_SUBSYS_PASSIVE, TRACE_GROUP_COMPS,    TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*                     Softnet                           */
      { TRACE_SUBSYS_PCPNIO_CHNL,                1,  "[PCPNIO][CHNL]",           TRACE_SUBSYS_PASSIVE, TRACE_GROUP_COMPS,    TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*                     Softnet                           */
      { TRACE_SUBSYS_PCPNIO_PCOVS,               1,  "[PCPNIO][PCOVS]",          TRACE_SUBSYS_PASSIVE, TRACE_GROUP_COMPS,    TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*                     Softnet                           */
      { TRACE_SUBSYS_PCPNIO_ENVLUSR,             1,  "[PCPNIO][ENVLUSR]",        TRACE_SUBSYS_PASSIVE, TRACE_GROUP_COMPS,    TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*                     Softnet                           */
      { TRACE_SUBSYS_PCPNIO_ENVLP,               1,  "[PCPNIO][ENVLP]",          TRACE_SUBSYS_PASSIVE, TRACE_GROUP_COMPS,    TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*                     Softnet                           */
      { TRACE_SUBSYS_PCPNIO_UTRANS,              1,  "[PCPNIO][UTRANS]",         TRACE_SUBSYS_PASSIVE, TRACE_GROUP_COMPS,    TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*                     Softnet                           */
      { TRACE_SUBSYS_PCPNIO_AGENT,               1,  "[PCPNIO][AGENT]",          TRACE_SUBSYS_PASSIVE, TRACE_GROUP_COMPS,    TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*                     Softnet                           */
    
      /*********************************************************************/
      /* DPS */
      { TRACE_SUBSYS_DPS_MGM,                    1,  "[DPS][MGM]",               TRACE_SUBSYS_PASSIVE, TRACE_GROUP_DPS,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*       CP3425                                          */
      { TRACE_SUBSYS_DPS_C0,                     1,  "[DPS][C0]",                TRACE_SUBSYS_PASSIVE, TRACE_GROUP_DPS,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*       CP3425                                          */
      { TRACE_SUBSYS_DPS_V1SL,                   1,  "[DPS][V1SL]",              TRACE_SUBSYS_PASSIVE, TRACE_GROUP_DPS,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*       CP3425                                          */
      { TRACE_SUBSYS_DPS_TIS,                    1,  "[DPS][TIS]",               TRACE_SUBSYS_PASSIVE, TRACE_GROUP_DPS,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*       CP3425                                          */
      { TRACE_SUBSYS_DPS_SYSIF,                  1,  "[DPS][SYSIF]",             TRACE_SUBSYS_PASSIVE, TRACE_GROUP_DPS,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*       CP3425                                          */
      { TRACE_SUBSYS_DPS_XBUS,                   1,  "[DPS][XBUS]",              TRACE_SUBSYS_PASSIVE, TRACE_GROUP_DPS,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*       CP3425                                          */

      /*************************************************************************/
      /* DP-Master */
      { TRACE_SUBSYS_DP_SDBDP,                   1,  "[DP][SDBDP]",              TRACE_SUBSYS_PASSIVE, TRACE_GROUP_DPM,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },  /*                                               CP4435EXT*/

      /*********************************************************************/
      /* PBK */
      { TRACE_SUBSYS_PBK,                        1,  "[PBK][AGENT]",             TRACE_SUBSYS_PASSIVE, TRACE_GROUP_PBK,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*              CP3431                                   */
      { TRACE_SUBSYS_PBK_KBUSVERB,               1,  "[PBK][KBUSV]",             TRACE_SUBSYS_PASSIVE, TRACE_GROUP_PBK,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*              CP3431                                   */
      { TRACE_SUBSYS_PBK_SERVER,                 1,  "[PBK][SERVER]",            TRACE_SUBSYS_PASSIVE, TRACE_GROUP_PBK,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*              CP3431                                   */
      { TRACE_SUBSYS_PBK_OPMUX,                  1,  "[PBK][OPMUX]",             TRACE_SUBSYS_PASSIVE, TRACE_GROUP_PBK,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*              CP3431                                   */
      { TRACE_SUBSYS_PBK_PBUS,                  16,  "[PBK][PBUS]",              TRACE_SUBSYS_PASSIVE, TRACE_GROUP_PBK,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*              CP3431                                   */
      { TRACE_SUBSYS_PBK_CLIENT,                16,  "[PBK][CLIENT]",            TRACE_SUBSYS_PASSIVE, TRACE_GROUP_PBK,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*              CP3431                                   */
      { TRACE_SUBSYS_PBK_VERB,                  16,  "[PBK][VERB]",              TRACE_SUBSYS_PASSIVE, TRACE_GROUP_PBK,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*              CP3431                                   */

      /*************************************************************************/
      /* CBA */
      { TRACE_SUBSYS_CBA_SYSA,                   1,  "[CBA][SYSA]",              TRACE_SUBSYS_PASSIVE, TRACE_GROUP_CBA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*              CP3431                  CP4431           */
      { TRACE_SUBSYS_CBA_RPC_SOCK,               1,  "[CBA][RPC_SOCK]",          TRACE_SUBSYS_PASSIVE, TRACE_GROUP_CBA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*              CP3431                  CP4431           */
      { TRACE_SUBSYS_CBA_RPC,                    1,  "[CBA][RPC]",               TRACE_SUBSYS_PASSIVE, TRACE_GROUP_CBA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*              CP3431                  CP4431           */
      { TRACE_SUBSYS_CBA_DCOM,                   1,  "[CBA][DCOM]",              TRACE_SUBSYS_PASSIVE, TRACE_GROUP_CBA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*              CP3431                  CP4431           */
      { TRACE_SUBSYS_CBA_APPL_AMARSHAL,          1,  "[CBA][APPL][AMARSHAL]",    TRACE_SUBSYS_PASSIVE, TRACE_GROUP_CBA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*              CP3431                  CP4431           */
      { TRACE_SUBSYS_CBA_APPL_S7,                1,  "[CBA][APPL][S7]",          TRACE_SUBSYS_PASSIVE, TRACE_GROUP_CBA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*              CP3431                  CP4431           */
      { TRACE_SUBSYS_CBA_APPL_ACCO,              1,  "[CBA][APPL][ACCO]",        TRACE_SUBSYS_PASSIVE, TRACE_GROUP_CBA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*              CP3431                  CP4431           */
      { TRACE_SUBSYS_CBA_APPL_DEVICE,            1,  "[CBA][APPL][DEVICE]",      TRACE_SUBSYS_PASSIVE, TRACE_GROUP_CBA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*              CP3431                  CP4431           */
      { TRACE_SUBSYS_CBA_GATE,                   1,  "[CBA][GATE]",              TRACE_SUBSYS_PASSIVE, TRACE_GROUP_CBA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*              CP3431                  CP4431           */
      { TRACE_SUBSYS_CBA_TRACE,                  1,  "[CBA][BASE]",              TRACE_SUBSYS_PASSIVE, TRACE_GROUP_CBA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*              CP3431                  CP4431           */
      { TRACE_SUBSYS_CBA_SRT,                    1,  "[CBA][APSRT]",             TRACE_SUBSYS_PASSIVE, TRACE_GROUP_CBA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*              CP3431                  CP4431           */
      { TRACE_SUBSYS_CBA_KBUS,                   1,  "[CBA][KBUS]",              TRACE_SUBSYS_PASSIVE, TRACE_GROUP_CBA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*              CP3431                  CP4431           */
         
      /* CBA-DIAG */
      { TRACE_SUBSYS_CBA_DIAG,                   1,  "[CBA][DIAG][BASE]",               TRACE_SUBSYS_PASSIVE, TRACE_GROUP_CBA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*              CP3431           CP4431           */
      { TRACE_SUBSYS_CBA_DIAG_GSI_SOCK_CON,      1,  "[CBA][DIAG][GSI][SOCKETCONN]",    TRACE_SUBSYS_PASSIVE, TRACE_GROUP_CBA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*              CP3431           CP4431           */
      { TRACE_SUBSYS_CBA_DIAG_GSI_ADP,           1,  "[CBA][DIAG][GSI][SOCKETADAPTER]", TRACE_SUBSYS_PASSIVE, TRACE_GROUP_CBA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*              CP3431           CP4431           */
      { TRACE_SUBSYS_CBA_DIAG_RPC_CCONTEXT,      1,  "[CBA][DIAG][RPC][CLIENTCONTEXT]", TRACE_SUBSYS_PASSIVE, TRACE_GROUP_CBA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*              CP3431           CP4431           */
      { TRACE_SUBSYS_CBA_DIAG_RPC_SCONTEXT,      1,  "[CBA][DIAG][RPC][SERVERCONTEXT]", TRACE_SUBSYS_PASSIVE, TRACE_GROUP_CBA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*              CP3431           CP4431           */
      { TRACE_SUBSYS_CBA_DIAG_RPC_MOD,           1,  "[CBA][DIAG][RPC][MODULE]",        TRACE_SUBSYS_PASSIVE, TRACE_GROUP_CBA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*              CP3431           CP4431           */
      { TRACE_SUBSYS_CBA_DIAG_RPC_REGIF,         1,  "[CBA][DIAG][RPC][REG_IFC]",       TRACE_SUBSYS_PASSIVE, TRACE_GROUP_CBA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*              CP3431           CP4431           */
      { TRACE_SUBSYS_CBA_DIAG_DCOM_STAT,         1,  "[CBA][DIAG][DCOM][MODULE]",       TRACE_SUBSYS_PASSIVE, TRACE_GROUP_CBA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*              CP3431           CP4431           */
      { TRACE_SUBSYS_CBA_DIAG_DCOM_RUNOBJ,       1,  "[CBA][DIAG][DCOM][RUNNING_OBJ]",  TRACE_SUBSYS_PASSIVE, TRACE_GROUP_CBA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*              CP3431           CP4431           */
      { TRACE_SUBSYS_CBA_DIAG_DCOM_CIFC,         1,  "[CBA][DIAG][DCOM][CLIENT_IFC]",   TRACE_SUBSYS_PASSIVE, TRACE_GROUP_CBA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*              CP3431           CP4431           */
      { TRACE_SUBSYS_CBA_DIAG_DCOM_SIFC,         1,  "[CBA][DIAG][DCOM][SERVER_IFC]",   TRACE_SUBSYS_PASSIVE, TRACE_GROUP_CBA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*              CP3431           CP4431           */
      { TRACE_SUBSYS_CBA_DIAG_PDEV,              1,  "[CBA][DIAG][PDEV]",               TRACE_SUBSYS_PASSIVE, TRACE_GROUP_CBA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*              CP3431           CP4431           */
      { TRACE_SUBSYS_CBA_DIAG_LDEV,              1,  "[CBA][DIAG][LDEV]",               TRACE_SUBSYS_PASSIVE, TRACE_GROUP_CBA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*              CP3431           CP4431           */
      { TRACE_SUBSYS_CBA_DIAG_RTAUTO,            1,  "[CBA][DIAG][RTAUTO]",             TRACE_SUBSYS_PASSIVE, TRACE_GROUP_CBA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*              CP3431           CP4431           */
      { TRACE_SUBSYS_CBA_DIAG_PUTPROP,           1,  "[CBA][DIAG][RTAUTO][PUTPROP]",    TRACE_SUBSYS_PASSIVE, TRACE_GROUP_CBA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*              CP3431           CP4431           */
      { TRACE_SUBSYS_CBA_DIAG_GETPROP,           1,  "[CBA][DIAG][RTAUTO][GETPROP]",    TRACE_SUBSYS_PASSIVE, TRACE_GROUP_CBA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*              CP3431           CP4431           */
      { TRACE_SUBSYS_CBA_DIAG_ACCO,              1,  "[CBA][DIAG][ACCO]",               TRACE_SUBSYS_PASSIVE, TRACE_GROUP_CBA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*              CP3431           CP4431           */
      { TRACE_SUBSYS_CBA_DIAG_CONSCONN,          1,  "[CBA][DIAG][ACCO][CONSUMERCONN]", TRACE_SUBSYS_PASSIVE, TRACE_GROUP_CBA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*              CP3431           CP4431           */
      { TRACE_SUBSYS_CBA_DIAG_PING,              1,  "[CBA][DIAG][ACCO][PING]",         TRACE_SUBSYS_PASSIVE, TRACE_GROUP_CBA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*              CP3431           CP4431           */
      { TRACE_SUBSYS_CBA_DIAG_PROVCONN,          1,  "[CBA][DIAG][ACCO][PROVIDERCONN]", TRACE_SUBSYS_PASSIVE, TRACE_GROUP_CBA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*              CP3431           CP4431           */
      { TRACE_SUBSYS_CBA_DIAG_ADVISE,            1,  "[CBA][DIAG][ADVISE]",             TRACE_SUBSYS_PASSIVE, TRACE_GROUP_CBA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*              CP3431           CP4431           */
      { TRACE_SUBSYS_CBA_DIAG_LDEVSTATEADVISE,   1,  "[CBA][DIAG][LDEV][STATEADVISE]",  TRACE_SUBSYS_PASSIVE, TRACE_GROUP_CBA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*              CP3431           CP4431           */
      { TRACE_SUBSYS_CBA_DIAG_LDEVGRERRADVISE,   1,  "[CBA][DIAG][LDEV][GRERRADVISE]",  TRACE_SUBSYS_PASSIVE, TRACE_GROUP_CBA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*              CP3431           CP4431           */
      { TRACE_SUBSYS_CBA_DIAG_ACCOGRERRADVISE,   1,  "[CBA][DIAG][ACCO][GRERRADVISE]",  TRACE_SUBSYS_PASSIVE, TRACE_GROUP_CBA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*              CP3431           CP4431           */
      { TRACE_SUBSYS_CBA_DIAG_AM_STAT,           1,  "[CBA][DIAG][AM][MODULE]",         TRACE_SUBSYS_PASSIVE, TRACE_GROUP_CBA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*              CP3431           CP4431           */
      { TRACE_SUBSYS_CBA_DIAG_GATECHANNEL,       1,  "[CBA][DIAG][GATE]",               TRACE_SUBSYS_PASSIVE, TRACE_GROUP_CBA,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*              CP3431           CP4431           */

   /*************************************************************************/
      /* CP1628 */
      { TRACE_SUBSYS_ADMMAIN,                    1,  "[ADMMAIN]",                       TRACE_SUBSYS_PASSIVE, TRACE_GROUP_CP1628,   TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*                                       CP1628   */
      { TRACE_SUBSYS_ADMFLASH,                   1,  "[ADMFLASH]",                      TRACE_SUBSYS_PASSIVE, TRACE_GROUP_CP1628,   TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*                                       CP1628   */
      { TRACE_SUBSYS_CLOCK,                      1,  "[CLOCK]",                         TRACE_SUBSYS_PASSIVE, TRACE_GROUP_CP1628,   TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*                                       CP1628   */
      { TRACE_SUBSYS_ISO,                        1,  "[ISO]",                           TRACE_SUBSYS_PASSIVE, TRACE_GROUP_CP1628,   TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*                                       CP1628   */
      { TRACE_SUBSYS_MUXBIND,                    1,  "[MUXBIND]",                       TRACE_SUBSYS_PASSIVE, TRACE_GROUP_CP1628,   TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*                                       CP1628   */
      { TRACE_SUBSYS_NDIR,                       1,  "[NDIR]",                          TRACE_SUBSYS_PASSIVE, TRACE_GROUP_CP1628,   TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*                                       CP1628   */
      { TRACE_SUBSYS_S7ONTCP,                    1,  "[S7ONTCP]",                       TRACE_SUBSYS_PASSIVE, TRACE_GROUP_CP1628,   TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*                                       CP1628   */
      { TRACE_SUBSYS_SMDPRAM,                    1,  "[SMDPRAM]",                       TRACE_SUBSYS_PASSIVE, TRACE_GROUP_CP1628,   TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*                                       CP1628   */
      { TRACE_SUBSYS_SNMP1628,                   1,  "[SNMP1628]",                      TRACE_SUBSYS_PASSIVE, TRACE_GROUP_CP1628,   TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*                                       CP1628   */
      { TRACE_SUBSYS_FUSION,                     1,  "[FUSION]",                        TRACE_SUBSYS_PASSIVE, TRACE_GROUP_CP1628,   TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*                                       CP1628   */
      { TRACE_SUBSYS_TARGET,                     1,  "[TARGET]",                        TRACE_SUBSYS_PASSIVE, TRACE_GROUP_CP1628,   TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },   /*                                       CP1628   */

   /*************************************************************************/
      /* PBUS Plus */

      /* subsystems of component PBUSP base package */
      { TRACE_SUBSYS_PBUSP_ALARM,                 1,  "[PBUSP_ALARM]",                  TRACE_SUBSYS_PASSIVE, TRACE_GROUP_PBP,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },
      { TRACE_SUBSYS_PBUSP_ALARM_RCV,             1,  "[PBUSP_ALARM_RCV]",              TRACE_SUBSYS_PASSIVE, TRACE_GROUP_PBP,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },
      { TRACE_SUBSYS_PBUSP_ALARM_SND,             1,  "[PBUSP_ALARM_SND]",              TRACE_SUBSYS_PASSIVE, TRACE_GROUP_PBP,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },
      { TRACE_SUBSYS_PBUSP_AZK,                   1,  "[PBUSP_AZK]",                    TRACE_SUBSYS_PASSIVE, TRACE_GROUP_PBP,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },
      { TRACE_SUBSYS_PBUSP_AZK_RCV,               1,  "[PBUSP_AZK_RCV]",                TRACE_SUBSYS_PASSIVE, TRACE_GROUP_PBP,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },
      { TRACE_SUBSYS_PBUSP_AZK_SND,               1,  "[PBUSP_AZK_SND]",                TRACE_SUBSYS_PASSIVE, TRACE_GROUP_PBP,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },
      { TRACE_SUBSYS_PBUSP_BUS,                   1,  "[PBUSP_BUS]",                    TRACE_SUBSYS_PASSIVE, TRACE_GROUP_PBP,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },
      { TRACE_SUBSYS_PBUSP_CLOCK,                 1,  "[PBUSP_CLOCK]",                  TRACE_SUBSYS_PASSIVE, TRACE_GROUP_PBP,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },
      { TRACE_SUBSYS_PBUSP_IO,                    1,  "[PBUSP_IO]",                     TRACE_SUBSYS_PASSIVE, TRACE_GROUP_PBP,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },

      /* PBUSP base package -> subsystems of Slave */
      { TRACE_SUBSYS_PSP_ALARM,                   1,  "[PSP_ALARM]",                    TRACE_SUBSYS_PASSIVE, TRACE_GROUP_PBP,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },
      { TRACE_SUBSYS_PSP_BUSOBSERVE,              1,  "[PSP_BUSOBSERVE]",               TRACE_SUBSYS_PASSIVE, TRACE_GROUP_PBP,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },
      { TRACE_SUBSYS_PSP_IO,                      1,  "[PSP_IO]",                       TRACE_SUBSYS_PASSIVE, TRACE_GROUP_PBP,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },
      { TRACE_SUBSYS_MOD_INTEGR,                  1,  "[MOD_INTEGR]",                   TRACE_SUBSYS_PASSIVE, TRACE_GROUP_PBP,      TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },

      /*************************************************************************/
      /* Device Proxy für CP1616 */
      { TRACE_SUBSYS_PE_PROXY,                    1,  "[PE][PROXY]",                    TRACE_SUBSYS_PASSIVE, TRACE_GROUP_BOARD,    TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },
      { TRACE_SUBSYS_DPRLIB_FW,                   1,  "[DPRLIB][FW]",                   TRACE_SUBSYS_PASSIVE, TRACE_GROUP_BOARD,    TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },
      { TRACE_SUBSYS_DPRLIB_INTERN,               1,  "[DPRLIB][INTERN]",               TRACE_SUBSYS_PASSIVE, TRACE_GROUP_BOARD,    TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },
      { TRACE_SUBSYS_DPR_SW,                      1,  "[DPR][SW]",             			TRACE_SUBSYS_PASSIVE, TRACE_GROUP_BOARD,    TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL },

   /*************************************************************************/
   /* This entry must every time be here. Do not erase it!!! */
   { TRACE_SUBSYS_NUM,               0xffff,  "ENDE-ltrc_subsys_cfg_table-ENDE", TRACE_SUBSYS_PASSIVE, TRACE_GROUP_NOTHING,  TRACE_BUFFER_DEFAULT,  LTRC_DEFAULT_RUNTIME_LEVEL }
   /*************************************************************************/
};

#endif  /* if defined LTRC_SET_SUBSYS_CFG_TABLE */



/*=============================================================================
 * (3) ltrc_subsys_cfg_table
 *===========================================================================*/
/*|@@@START-LTRC-Scanner-Params@@@|

@>@ EnumString,                            Directory,                                  MacroPreName,  Ignoreparam,   FilesListDirectory @<@
   TRACE_SUBSYS_TRACE,                     trace;
@>@ ************************************ @<@

   TRACE_SUBSYS_MGT,                       d2_ge\pnioBase\mgt,                         MGT,           0,             d2_ge\pnioBase\mgt\inc\mgt_ltrc.h;
   TRACE_SUBSYS_MGT,                       vxw_pnio_khe\cp1616dk\firmware\mgt,         MGT,           0,             vxw_pnio_khe\cp1616dk\firmware\mgt\inc\mgt_ltrc.h;
   TRACE_SUBSYS_MGT,                       cp3431_v3\mgt,                              MGT            0,             cp3431_v3\mgt\inc\mgtltrc.h;
   TRACE_SUBSYS_MGT,                       cp3431_lean\mgt,                            MGT,           0,             cp3431_lean\mgt\inc\mgtltrc.h;
   TRACE_SUBSYS_MGT,                       cp4431_v3\mgt,                              MGT,           0,             cp4431_v3\mgt\inc\mgtltrc.h;
     
@>@ *** alt *** @<@
   TRACE_SUBSYS_BUB,                       le_nbg2\BuB,                                BUB_TSK,       0,             le_nbg2\BuB\inc\bub_ltrc.hpp;
   TRACE_SUBSYS_DICON,                     le_nbg2\diag\dicon,                         DICON,         0,             le_nbg2\diag\dicon\src\dicon_ltrcconfig.hpp;
   TRACE_SUBSYS_PGTASK,                    le_nbg\pgtask,                              PG,            1,             le_nbg\pgtask\inc\pgtrc.h;
@>@ *** neu *** @<@
   TRACE_SUBSYS_ACL,                       le_khe\acl,                                 ACL,           0,             le_khe\acl\inc\aclltrc.h;
   TRACE_SUBSYS_BUB,                       le_nbg2\BuB,                                BUB_TSK,       0,             le_nbg2\BuB\src\bub_ltrc.hpp;
   TRACE_SUBSYS_BSD_SELECT,                le_khe\bsdstack\src\bsdadapt,               BSD_SELECT,    0,             le_khe\bsdstack\src\include\bsdltrc.h;
   TRACE_SUBSYS_BSD_USER,                  le_khe\bsdstack\src\bsdadapt,               BSD_USER,      0,             le_khe\bsdstack\src\include\bsdltrc.h;
   TRACE_SUBSYS_BSD_VXADAPT,               le_khe\bsdstack\src\vxadapt,                BSD_VXADAPT,   0,             le_khe\bsdstack\src\vxadapt\src\bsdvxltrc.h;
   TRACE_SUBSYS_DICON,                     le_nbg2\diag\dicon,                         DICON,         0,             le_nbg2\diag\dicon\inc\dicon_ltrcconfig.hpp;
   TRACE_SUBSYS_IPCONFIG,                  le_nbg\ipconfig,                            IPC,           0,             le_nbg\ipconfig\inc\ipc_trc.h;
   TRACE_SUBSYS_IPCONFIG,                  le_nbg\ipconfig,                            IPC,           0,             le_nbg\ipconfig\incl\ipc_trc.h;
   TRACE_SUBSYS_OVS,                       le_nbg\ovs,                                 OVS,           2,             le_nbg\ovs\incl\ovsdiag.h;
   TRACE_SUBSYS_PANEL,                     le_nbg\panel,                               PANEL,         0,             le_nbg\panel\inc\panelltrc.h;
   TRACE_SUBSYS_PGTASK,                    le_nbg\pgtask,                              PG,            1,             le_nbg\pgtask\incl\pgtrc.h;
   TRACE_SUBSYS_SNDRCV,                    le_khe\sndrcv,                              SNDRCV,        0,             le_khe\sndrcv\inc\srext.h;
   TRACE_SUBSYS_TIMESYNC,                  le_khe\timesync,                            TSC,           1,             le_khe\timesync\inc\tsctrc.h;
   TRACE_SUBSYS_TIMESYNC,                  ge_nbg\timesync.pb,                         TSC,           1,             ge_nbg\timesync.pb\inc\ts_trace.h;
   TRACE_SUBSYS_VARSERV,                   le_khe\varserv,                             VARSERV;
   TRACE_SUBSYS_TIS,                       le_nbg2\diag\diagtis,                       TIS,           1,             le_nbg2\diag\diagtis\inc\diagtis_ltrcconfig.hpp;
   TRACE_SUBSYS_TREL,                      le_khe\trel,                                TREL,          0,             le_khe\trel\inc\trelltrc.h;
   TRACE_SUBSYS_SECURITY,                  le_khe\security,                            SEC,           0,             le_khe\security\secctrl\inc\sectrc.h;

@>@ ************************************ @<@

@>@ le_nbg2 -- ICON @<@
   TRACE_SUBSYS_ICON_CHANNEL,              le_nbg2\icon\iconchannel,                   ICON_CHANNEL,  0,             le_nbg2\icon\iconchannel\inc\iconChannel_ltrc.hpp;
   TRACE_SUBSYS_ICON_FLICT,                le_nbg2\icon\iconflict,                     ICON_FLICT,    0,             le_nbg2\icon\iconflict\inc\iconFlict_ltrc.hpp;
   TRACE_SUBSYS_ICON_MGT,                  le_nbg2\icon\iconmgt,                       ICON_MGT,      0,             le_nbg2\icon\iconmgt\inc\iconMgt_ltrc.hpp;
   TRACE_SUBSYS_ICON_PDEV,                 le_nbg2\icon\iconpdev,                      ICON_PDEV,     0,             le_nbg2\icon\iconpdev\inc\iconPdev_ltrc.hpp;
   TRACE_SUBSYS_ICON_PSU,                  le_nbg2\icon\iconpsu,                       IPSU,          0,             le_nbg2\icon\iconpsu\inc\iconPsu_ltrace.h;
   TRACE_SUBSYS_ICON_REMA,                 le_nbg2\icon\iconrema,                      ICON_REMA,     0,             le_nbg2\icon\iconrema\inc\iconRema_ltrc.hpp;
   TRACE_SUBSYS_ICON_TIS,                  le_nbg2\icon\icontis,                       ICON_TIS,      0,             le_nbg2\icon\icontis\inc\iconTis_ltrc.hpp;
   
@>@ *** alt *** @<@
   TRACE_SUBSYS_IPC,                       le_nbg2\ipc,                                IPC,           0,             le_nbg2\ipc\inc\ipcDev_ltrace.h;
@>@ *** neu *** @<@
   TRACE_SUBSYS_IPC,                       le_nbg2\ipc,                                IPC,           0,             le_nbg2\ipc\incl\ipcDev_ltrace.h;

@>@ ************************************ @<@
@>@ PT2_LSA -- LSA-Komponenten @<@
   TRACE_SUBSYS_ACP_UPPER,                 pt2_lsa\lsa\acp,                            ACP_UPPER;
   TRACE_SUBSYS_ACP_LOWER,                 pt2_lsa\lsa\acp,                            ACP_LL;
   TRACE_SUBSYS_ACP_RTA,                   pt2_lsa\lsa\acp,                            ACP_RTA;
   TRACE_SUBSYS_ACP_MEMORY,                pt2_lsa\lsa\acp,                            ACP_MEM;

   TRACE_SUBSYS_CLRPC_UPPER,               pt2_lsa\lsa\clrpc,                          CLRPC_UPPER;
   TRACE_SUBSYS_CLRPC_CL,                  pt2_lsa\lsa\clrpc,                          CLRPC_CL;
   TRACE_SUBSYS_CLRPC_SV,                  pt2_lsa\lsa\clrpc,                          CLRPC_SV;
   TRACE_SUBSYS_CLRPC_EPM,                 pt2_lsa\lsa\clrpc,                          CLRPC_EPM;
   TRACE_SUBSYS_CLRPC_LOWER,               pt2_lsa\lsa\clrpc,                          CLRPC_LOWER;
   TRACE_SUBSYS_CLRPC_CL_PKT,              pt2_lsa\lsa\clrpc,                          CLRPC_CL_PKT;
   TRACE_SUBSYS_CLRPC_SV_PKT,              pt2_lsa\lsa\clrpc,                          CLRPC_SV_PKT;

   TRACE_SUBSYS_CM_ACP,                    pt2_lsa\lsa\cm,                             CM_ACP;
   TRACE_SUBSYS_CM_AR,                     pt2_lsa\lsa\cm,                             CM_AR;
   TRACE_SUBSYS_CM_CL,                     pt2_lsa\lsa\cm,                             CM_CL;
   TRACE_SUBSYS_CM_MC,                     pt2_lsa\lsa\cm,                             CM_MC;
   TRACE_SUBSYS_CM_MEM,                    pt2_lsa\lsa\cm,                             CM_MEM;
   TRACE_SUBSYS_CM_NARE,                   pt2_lsa\lsa\cm,                             CM_NARE;
   TRACE_SUBSYS_CM_RPC,                    pt2_lsa\lsa\cm,                             CM_RPC;
   TRACE_SUBSYS_CM_SV,                     pt2_lsa\lsa\cm,                             CM_SV;
   TRACE_SUBSYS_CM_UPPER,                  pt2_lsa\lsa\cm,                             CM_UPPER;
   TRACE_SUBSYS_CM_EDD,                    pt2_lsa\lsa\cm,                             CM_EDD;
   TRACE_SUBSYS_CM_GSY,                    pt2_lsa\lsa\cm,                             CM_GSY;
   TRACE_SUBSYS_CM_OHA,                    pt2_lsa\lsa\cm,                             CM_OHA;
   TRACE_SUBSYS_CM_PD,                     pt2_lsa\lsa\cm,                             CM_PD;
   TRACE_SUBSYS_CM_MRP,                    pt2_lsa\lsa\cm,                             CM_MRP;
   TRACE_SUBSYS_CM_POF,                    pt2_lsa\lsa\cm,                             CM_POF;
   TRACE_SUBSYS_CM_REMA,                   pt2_lsa\lsa\cm,                             CM_REMA;

   TRACE_SUBSYS_DCP_UPPER,                 pt2_lsa\lsa\dcp,                            DCP_UPPER;
   TRACE_SUBSYS_DCP_LOWER,                 pt2_lsa\lsa\dcp,                            DCP_LOWER;
   TRACE_SUBSYS_DCP_SYSTEM,                pt2_lsa\lsa\dcp,                            DCP_SYSTEM;
   TRACE_SUBSYS_DCP_ERROR,                 pt2_lsa\lsa\dcp,                            DCP_ERROR;
   TRACE_SUBSYS_DCP_FUNCTION,              pt2_lsa\lsa\dcp,                            DCP_FUNCTION;
   TRACE_SUBSYS_DCP_SERVER,                pt2_lsa\lsa\dcp,                            DCP_SERVER;
   TRACE_SUBSYS_DCP_SRVERR,                pt2_lsa\lsa\dcp,                            DCP_SRVERR;

   TRACE_SUBSYS_GSY_UPPER,                 pt2_lsa\lsa\gsy,                            GSY_UPPER;
   TRACE_SUBSYS_GSY_LOWER,                 pt2_lsa\lsa\gsy,                            GSY_LOWER;
   TRACE_SUBSYS_GSY_SYSTEM,                pt2_lsa\lsa\gsy,                            GSY_SYSTEM;
   TRACE_SUBSYS_GSY_ERROR,                 pt2_lsa\lsa\gsy,                            GSY_ERROR;
   TRACE_SUBSYS_GSY_ERROR,                 pt2_lsa\lsa\gsy,                            CLP_ERROR;
   TRACE_SUBSYS_GSY_FUNCTION,              pt2_lsa\lsa\gsy,                            GSY_FUNCTION;
   TRACE_SUBSYS_GSY_DIAG,                  pt2_lsa\lsa\gsy,                            GSY_DIAG;
   TRACE_SUBSYS_GSY_DEL,                   pt2_lsa\lsa\gsy,                            GSY_DEL;
   TRACE_SUBSYS_GSY_FWD,                   pt2_lsa\lsa\gsy,                            GSY_FWD;
   TRACE_SUBSYS_GSY_SNDRCV,                pt2_lsa\lsa\gsy,                            GSY_SNDRCV;
   TRACE_SUBSYS_GSY_PRM,                   pt2_lsa\lsa\gsy,                            GSY_PRM;
   TRACE_SUBSYS_GSY_SYNC,                  pt2_lsa\lsa\gsy,                            GSY_SYNC;
   TRACE_SUBSYS_GSY_MASTER,                pt2_lsa\lsa\gsy,                            GSY_MASTER;

   TRACE_SUBSYS_NARE_UPPER,                pt2_lsa\lsa\nare,                           NARE_UPPER;
   TRACE_SUBSYS_NARE_LOWER,                pt2_lsa\lsa\nare,                           NARE_LOWER;
   TRACE_SUBSYS_NARE_SYSTEM,               pt2_lsa\lsa\nare,                           NARE_SYSTEM;
   TRACE_SUBSYS_NARE_FUNCTION,             pt2_lsa\lsa\nare,                           NARE_FUNCTION;
   TRACE_SUBSYS_NARE_PROGRAM,              pt2_lsa\lsa\nare,                           NARE_PROGRAM;

@>@ *** alt *** @<@
   TRACE_SUBSYS_LSYS_GLOB,                 pt2_lsa\lsa\sysall,                         LSYS_GLOB,     0,             pt2_lsa\lsa\sysall\inc\lsys_ltrc.h;
   TRACE_SUBSYS_LSYS_CM,                   pt2_lsa\lsa\sysall,                         LSYS_CM,       0,             pt2_lsa\lsa\sysall\inc\lsys_ltrc.h;
   TRACE_SUBSYS_LSYS_PSU,                  pt2_lsa\lsa\sysall,                         LSYS_PSU,      0,             pt2_lsa\lsa\sysall\inc\lsys_ltrc.h;
   TRACE_SUBSYS_LSYS_LTRC,                 pt2_lsa\lsa\sysall,                         LSYS_LTRC,     0,             pt2_lsa\lsa\sysall\inc\lsys_ltrc.h;
@>@ *** neu *** @<@  
   TRACE_SUBSYS_LSYS_GLOB,                 pt2_lsa\lsa\lsys,                           LSYS_GLOB,     0,             pt2_lsa\lsa\lsys\inc\lsys_ltrc.h;
   TRACE_SUBSYS_LSYS_CM,                   pt2_lsa\lsa\lsys,                           LSYS_CM,       0,             pt2_lsa\lsa\lsys\inc\lsys_ltrc.h;
   TRACE_SUBSYS_LSYS_PSU,                  pt2_lsa\lsa\lsys,                           LSYS_PSU,      0,             pt2_lsa\lsa\lsys\inc\lsys_ltrc.h;
   TRACE_SUBSYS_LSYS_LTRC,                 pt2_lsa\lsa\lsys,                           LSYS_LTRC,     0,             pt2_lsa\lsa\lsys\inc\lsys_ltrc.h;

   TRACE_SUBSYS_LLDP_UPPER,                pt2_lsa\lsa\lldp,                           LLDP_UPPER;
   TRACE_SUBSYS_LLDP_LOWER,                pt2_lsa\lsa\lldp,                           LLDP_LOWER;
   TRACE_SUBSYS_LLDP_SYSTEM,               pt2_lsa\lsa\lldp,                           LLDP_SYSTEM;
   TRACE_SUBSYS_LLDP_FUNCTION,             pt2_lsa\lsa\lldp,                           LLDP_FUNCTION;
   TRACE_SUBSYS_LLDP_PROGRAM,              pt2_lsa\lsa\lldp,                           LLDP_PROGRAM;

   TRACE_SUBSYS_MRP_UPPER,                 pt2_lsa\lsa\mrp,                            MRP_UPPER;
   TRACE_SUBSYS_MRP_LOWER,                 pt2_lsa\lsa\mrp,                            MRP_LOWER;
   TRACE_SUBSYS_MRP_SYSTEM,                pt2_lsa\lsa\mrp,                            MRP_SYSTEM;
   TRACE_SUBSYS_MRP_FUNCTION,              pt2_lsa\lsa\mrp,                            MRP_FUNCTION;
   TRACE_SUBSYS_MRP_PROGRAM,               pt2_lsa\lsa\mrp,                            MRP_PROGRAM;

   TRACE_SUBSYS_OHA_UPPER,                 pt2_lsa\lsa\oha,                            OHA_UPPER;
   TRACE_SUBSYS_OHA_LOWER,                 pt2_lsa\lsa\oha,                            OHA_LOWER;
   TRACE_SUBSYS_OHA_SYSTEM,                pt2_lsa\lsa\oha,                            OHA_SYSTEM;
   TRACE_SUBSYS_OHA_FUNCTION,              pt2_lsa\lsa\oha,                            OHA_FUNCTION;
   TRACE_SUBSYS_OHA_PROGRAM,               pt2_lsa\lsa\oha,                            OHA_PROGRAM;

   TRACE_SUBSYS_POF_UPPER,                 pt2_lsa\lsa\pof,                            POF_UPPER;
   TRACE_SUBSYS_POF_LOWER,                 pt2_lsa\lsa\pof,                            POF_LOWER;
   TRACE_SUBSYS_POF_SYSTEM,                pt2_lsa\lsa\pof,                            POF_SYSTEM;
   TRACE_SUBSYS_POF_FUNCTION,              pt2_lsa\lsa\pof,                            POF_FUNCTION;
   TRACE_SUBSYS_POF_PROGRAM,               pt2_lsa\lsa\pof,                            POF_PROGRAM;

   TRACE_SUBSYS_SOCK_UPPER,                pt2_lsa\lsa\sock,                           SOCK_UPPER;
   TRACE_SUBSYS_SOCK_LOWER,                pt2_lsa\lsa\sock,                           SOCK_LOWER;
   TRACE_SUBSYS_SOCK_SYSTEM,               pt2_lsa\lsa\sock,                           SOCK_SYSTEM;
   TRACE_SUBSYS_SOCK_PROTOCOL,             pt2_lsa\lsa\sock,                           SOCK_PROTOCOL;
   TRACE_SUBSYS_SOCK_STACK,                pt2_lsa\lsa\sock,                           SOCK_STACK;

   TRACE_SUBSYS_EDDI_UPPER,                pt2_lsa\lsa\eddi,                           EDDI_UPPER;
   TRACE_SUBSYS_EDDI_LOWER,                pt2_lsa\lsa\eddi,                           EDDI_LOWER;
   TRACE_SUBSYS_EDDI_SYSTEM,               pt2_lsa\lsa\eddi,                           EDDI_SYSTEM;
   TRACE_SUBSYS_EDDI_FUNCTION,             pt2_lsa\lsa\eddi,                           EDDI_FUNCTION;
   TRACE_SUBSYS_EDDI_PROGRAM,              pt2_lsa\lsa\eddi,                           EDDI_PROGRAM;
   TRACE_SUBSYS_EDDI_PROGRAM,              pt2_lsa\lsa\eddi\sys_vxw,                   SYS_VXW;
   TRACE_SUBSYS_EDDI_NRT,                  pt2_lsa\lsa\eddi,                           EDDI_NRT;
   TRACE_SUBSYS_EDDI_PRM,                  pt2_lsa\lsa\eddi,                           EDDI_PRM;
   TRACE_SUBSYS_EDDI_SWI,                  pt2_lsa\lsa\eddi,                           EDDI_SWI;
   TRACE_SUBSYS_EDDI_CRT,                  pt2_lsa\lsa\eddi,                           EDDI_CRT;
   TRACE_SUBSYS_EDDI_SYNC,                 pt2_lsa\lsa\eddi,                           EDDI_SYNC;

   TRACE_SUBSYS_EDDS_UPPER,                pt2_lsa\lsa\edds,                           EDDS_UPPER;
   TRACE_SUBSYS_EDDS_LOWER,                pt2_lsa\lsa\edds,                           EDDS_LOWER;
   TRACE_SUBSYS_EDDS_SYSTEM,               pt2_lsa\lsa\edds,                           EDDS_SYSTEM;
   TRACE_SUBSYS_EDDS_FUNCTION,             pt2_lsa\lsa\edds,                           EDDS_FUNCTION;
   TRACE_SUBSYS_EDDS_PROGRAM,              pt2_lsa\lsa\edds,                           EDDS_PROGRAM;
   TRACE_SUBSYS_EDDS_NRT,                  pt2_lsa\lsa\edds,                           EDDS_NRT;
   TRACE_SUBSYS_EDDS_PRM,                  pt2_lsa\lsa\edds,                           EDDS_PRM;
   TRACE_SUBSYS_EDDS_SWI,                  pt2_lsa\lsa\edds,                           EDDS_SWI;
   TRACE_SUBSYS_EDDS_CRT,                  pt2_lsa\lsa\edds,                           EDDS_CRT;
   TRACE_SUBSYS_EDDS_SYNC,                 pt2_lsa\lsa\edds,                           EDDS_SYNC;

   TRACE_SUBSYS_EDDP_UPPER,                pt2_lsa\lsa\eddp,                           EDDP_UPPER;
   TRACE_SUBSYS_EDDP_LOWER,                pt2_lsa\lsa\eddp,                           EDDP_LOWER;
   TRACE_SUBSYS_EDDP_SYSTEM,               pt2_lsa\lsa\eddp,                           EDDP_SYSTEM;
   TRACE_SUBSYS_EDDP_FUNCTION,             pt2_lsa\lsa\eddp,                           EDDP_FUNCTION;
   TRACE_SUBSYS_EDDP_PROGRAM,              pt2_lsa\lsa\eddp,                           EDDP_PROGRAM;
   TRACE_SUBSYS_EDDP_NRT,                  pt2_lsa\lsa\eddp,                           EDDP_NRT;
   TRACE_SUBSYS_EDDP_PRM,                  pt2_lsa\lsa\eddp,                           EDDP_PRM;
   TRACE_SUBSYS_EDDP_SWI,                  pt2_lsa\lsa\eddp,                           EDDP_SWI;
   TRACE_SUBSYS_EDDP_CRT,                  pt2_lsa\lsa\eddp,                           EDDP_CRT;
   TRACE_SUBSYS_EDDP_SYNC,                 pt2_lsa\lsa\eddp,                           EDDP_SYNC;

@>@ ************************************ @<@
@>@ PNIO @<@
   TRACE_SUBSYS_PNIO_AGET300,              le_khe\pnioagent300,                        ETCR,          0,             le_khe\pnioagent300\inc\pnio_agent_trc.h;
   TRACE_SUBSYS_PNIO_AGET400,              le_khe\pnioagent400,                        PNIO_AGET400,  0,             le_khe\pnioagent400\inc\pnio_aget400_trc.h;
   TRACE_SUBSYS_PNIO_CTRL_GLOB,            le_nbg\pnio\ctrl,                           PNIO_CTRL,           0,       le_nbg\pnio\ctrl\incl\pnio_ctrl_ltrc.h;
   TRACE_SUBSYS_PNIO_CTRL_ALERT,           le_nbg\pnio\ctrl,                           PNIO_CTRL_ALERT,     0,       le_nbg\pnio\ctrl\incl\pnio_ctrl_ltrc.h;
   TRACE_SUBSYS_PNIO_CTRL_DRRW,            le_nbg\pnio\ctrl,                           PNIO_CTRL_DRRW,      0,       le_nbg\pnio\ctrl\incl\pnio_ctrl_ltrc.h;
   TRACE_SUBSYS_PNIO_CTRL_MEM,             le_nbg\pnio\ctrl,                           PNIO_CTRL_MEM,       0,       le_nbg\pnio\ctrl\incl\pnio_ctrl_ltrc.h;
   TRACE_SUBSYS_PNIO_CTRL_SDB,             le_nbg\pnio\ctrl,                           PNIO_CTRL_SDB,       0,       le_nbg\pnio\ctrl\incl\pnio_ctrl_ltrc.h;
   TRACE_SUBSYS_PNIO_CTRL_STATE,           le_nbg\pnio\ctrl,                           PNIO_CTRL_STATE,     0,       le_nbg\pnio\ctrl\incl\pnio_ctrl_ltrc.h;
   TRACE_SUBSYS_PNIO_ARM_UPDATE            le_khe\armapp\,                             PNIO_ARM_UPDATE,     0,       le_khe\armapp\inc\armLtrc.h;
   TRACE_SUBSYS_PNIO_AGENTDEVK_CTRL,       vxw_pnio_khe\modules\pnioagent,             PNIO_AGENTDEVK_CTRL, 0,       vxw_pnio_khe\modules\pnioagent\inc\pnioag_cfg.h;
   TRACE_SUBSYS_PNIO_AGENTDEVK_DEV,        vxw_pnio_khe\modules\pniodagent,            PNIO_AGENTDEVK_DEV,  0,       vxw_pnio_khe\modules\pniodagent\inc\pniodag_cfg.h;
   
   TRACE_SUBSYS_IOD,                       le_nbg\pnio\device\iod,                     IOD_BASE,      0,             le_nbg\pnio\device\iod\incl\device_cfg.h;

   TRACE_SUBSYS_SDEV,                      d2_ge\modules\sdev,                         SDEV,          0,             d2_ge\modules\sdev\inc\sdevltrc.h;
   TRACE_SUBSYS_APPL_IOD,                  le_nbg\pnio\device\iod_appl,                APPLIOD,       0,             le_nbg\pnio\device\iod_appl\inc\appltrc.h;
   TRACE_SUBSYS_IOR,                       vxw_pnio_khe\cp1616dk\firmware\iorouter,    IOR,           0,             vxw_pnio_khe\cp1616dk\firmware\iorouter\inc\ior_ltrc.h;
   TRACE_SUBSYS_L2_AGENT,                  vxw_pnio_khe\modules\l2agent,               L2_AGENT;

@>@ ************************************ @<@
@>@ H1T @<@
   TRACE_SUBSYS_H1T,                       le_khe\rts_tcp,                             H1T,           0,             le_khe\rts_tcp\inc\h1t_ltrc.h;
   TRACE_SUBSYS_H1T,                       le_khe\rtsp,                                H1T,           0,             le_khe\rts_tcp\inc\h1t_ltrc.h;
   TRACE_SUBSYS_H1T,                       vxw_pnio_khe\cp1616dk\firmware\rts_tcp,     H1T,           0,             vxw_pnio_khe\cp1616dk\firmware\rts_tcp\inc\h1t_ltrc.h;

   TRACE_SUBSYS_SDBV,                      le_khe\sdb_verteiler,                       SDBV,          0,             le_khe\sdb_verteiler\inc\sdbv_cfg.h;
   TRACE_SUBSYS_WD,                        vxw_pnio_khe\modules\watchdog,              WD,            0,             vxw_pnio_khe\modules\watchdog\inc\wd_ltrc.h;
   TRACE_SUBSYS_SNMP,                      vxw_pnio_khe\modules\snmpagent,             SNMP,          0,             vxw_pnio_khe\modules\snmpagent\inc\snmp_ltrc.h;
   TRACE_SUBSYS_SNMP,                      d2_ge\modules\snmpagent_local,              SNMP,          0,             d2_ge\modules\snmpagent_local\inc\snmp_ltrc.h;
   TRACE_SUBSYS_SNMP,                      le_khe\snmp,                                SNMP,          0,             le_khe\snmp\inc\dbg_trace.h;
   TRACE_SUBSYS_WEBDIAG_AGENT,             vxw_pnio_khe\modules\webdiagagent,          WEB,           0,             vxw_pnio_khe\modules\webdiagagent\inc\webdiag_ltrc.h;
   
   TRACE_SUBSYS_FWLDA,                     d2_ge\pnioBase\sysadapt,                    FWLDA;
   TRACE_SUBSYS_FWLDA,                     vxw_pnio_khe\cp1616dk\firmware\sysadapt,    FWLDA;
   TRACE_SUBSYS_SYSA_MISC,                 vxw_pnio_khe\cp1616dk\firmware\sysadapt,    SYSA;
   TRACE_SUBSYS_SYSA_MRP,                  vxw_pnio_khe\cp1616dk\firmware\sysadapt,    SYSA_MRP;

@>@ ************************************ @<@
@>@ ge_khe -- MINIWEB @<@
@>@ *** alt *** @<@ 
   TRACE_SUBSYS_MINIWEB,                   le_nbg2\MiniWeb_after_V_3.0\MiniWeb,                     MINIWEB,            0,             le_nbg2\MiniWeb_after_V_3.0\MiniWeb\Src\Includes\MWEB_ErrorCodes.h;
   TRACE_SUBSYS_MINIWEB_MWSL,              le_nbg2\MiniWeb_after_V_3.0\MiniWeb,                     MINIWEB_MWSL,       0,             le_nbg2\MiniWeb_after_V_3.0\MiniWeb\Src\Packages\MWSL\Filter\MWSL\MWEB_MWSL_Ltrace.h;
   TRACE_SUBSYS_MINIWEB_HTTPCORE,          le_nbg2\MiniWeb_after_V_3.0\MiniWeb\Src\Core\HTTPCore,   MINIWEB_HTTPCORE,   0,             ge_khe\http\inc\inet_str.h;
@>@ *** neu *** @<@    
   TRACE_SUBSYS_MINIWEB,                   ge_khe\miniweb\src,                                      MWEB,               0,             ge_khe\miniweb\src\Includes\MWEB_LTrace.h;
   TRACE_SUBSYS_MINIWEB_DEFAPP,            ge_khe\miniweb\src\Core\App\DefApp,                      MWEB,               0,             ge_khe\miniweb\src\Core\App\DefApp\MWEB_DefaultApplication_Ltrace.h;
   TRACE_SUBSYS_MINIWEB_HTTPCORE,          ge_khe\miniweb\src\Core\HTTPCore,                        MWEB,               0,             ge_khe\miniweb\src\Core\HTTPCore\MWEB_HTTPCore_Ltrace.h;
   TRACE_SUBSYS_MINIWEB_CORE,              ge_khe\miniweb\src\Core\MiniWebCore,                     MWEB,               0,             ge_khe\miniweb\src\Core\MiniWebCore\MWEB_Core_Ltrace.h;
   TRACE_SUBSYS_MINIWEB_FILESYS,           ge_khe\miniweb\src\Core\Services\FileSys,                MWEB,               0,             ge_khe\miniweb\src\Core\Services\FileSys\MWEB_FileSystem_Ltrace.h;
   TRACE_SUBSYS_MINIWEB_HTTPSRVC,          ge_khe\miniweb\src\Core\Services\HTTPService,            MWEB,               0,             ge_khe\miniweb\src\Core\Services\HTTPService\MWEB_HTTPService_Ltrace.h;
   TRACE_SUBSYS_MINIWEB_SECURITY,          ge_khe\miniweb\src\Core\Services\Security\HTTPSecurity,  MWEB,               0,             ge_khe\miniweb\src\Core\Services\Security\HTTPSecurity\MWEB_Security_Ltrace.h;
   TRACE_SUBSYS_MINIWEB_SOCKSSL,           ge_khe\miniweb\src\Core\Services\SockSSL,                MWEB,               0,             ge_khe\miniweb\src\Core\Services\SockSSL\MWEB_SSL_Ltrace.h;
   TRACE_SUBSYS_MINIWEB_XML,               ge_khe\miniweb\src\Core\Services\XML,                    MWEB,               0,             ge_khe\miniweb\src\Core\Services\XML\MWEB_XML_Ltrace.h;
   TRACE_SUBSYS_MINIWEB_MOBILE,            ge_khe\miniweb\src\CP\CP4431web\App\Mobile,              MWEB,               0,             ge_khe\miniweb\src\CP\CP4431web\App\Mobile\MWEB_Mobile_Ltrace.h;
   TRACE_SUBSYS_MINIWEB_CPVP,              ge_khe\miniweb\src\CP\CP4431web\Services\CPVarProvider,  MWEB,               0,             ge_khe\miniweb\src\CP\CP4431web\Services\CPVarProvider\MWEB_CPVarProvider_Ltrace.h;
   TRACE_SUBSYS_MINIWEB_MWSL,              ge_khe\miniweb\src\Packages\MWSL\Filter\MWSL,            MWEB,               0,             ge_khe\miniweb\src\Packages\MWSL\Filter\MWSL\MWEB_MWSL_Ltrace.h;
   TRACE_SUBSYS_MINIWEB_MWSL2,             ge_khe\miniweb\src\Packages\MWSL2\Service,               MWEB,               0,             ge_khe\miniweb\src\Packages\MWSL2\Service\MWEB_MWSL2_Ltrace.h;
   TRACE_SUBSYS_MINIWEB_S7APP,             ge_khe\miniweb\src\CP,                                   MWEB,               0,             ge_khe\miniweb\src\Includes\MWEB_S7APP_LTrace.h;
   TRACE_SUBSYS_MINIWEB_S7SRV,             ge_khe\miniweb\src\CP,                                   MWEB,               0,             ge_khe\miniweb\src\Includes\MWEB_S7SRV_LTrace.h;

@>@ ************************************ @<@
@>@ IT-Systems @<@
   TRACE_SUBSYS_FTP,                       ge_khe\ftp,                                 FTP,           0,             ge_khe\ftp\inc\ftp.h;
   TRACE_SUBSYS_FTPCLIENT,                 ge_khe\ftpclient,                           FTPCLIENT,     0,             ge_khe\ftpclient\inc\ftpc.h;
   TRACE_SUBSYS_HTTP,                      ge_khe\http,                                HTTP,          0,             ge_khe\http\inc\inet_str.h;
   TRACE_SUBSYS_S7_SERV,                   ge_khe\s7_serv,                             S7S,           0,             ge_khe\s7_serv\inc\s7s_tools.h;

   TRACE_SUBSYS_RTS_BASE,                  le_khe\rts,                                 RTS,           0,             le_khe\rts\inc\rts_trc.h;
   TRACE_SUBSYS_RTSP_BASE,                 le_khe\rtsp,                                RTSP,          0,             le_khe\rtsp\inc\rtsp_trc.h;
   TRACE_SUBSYS_RTSP_RFC,                  le_khe\rts_tcp,                             RFC,           0,             le_khe\rtsp\inc\rtsp_trc.h;

   TRACE_SUBSYS_FM400,                     le_khe\fm400,                               FM400,         0,             le_khe\fm400\inc\fm400trc.h;
   TRACE_SUBSYS_ETCRDRV,                   le_nbg\etcr400.drv,                         ETCR,          0,             le_nbg\etcr400.drv\inc\etcrtrc.h;

@>@ ************************************ @<@
@>@ TPAI + BACNET Systems @<@
   TRACE_SUBSYS_TPAI,                      le_khe\tpai,                                TPAI,          0,             le_khe\tpai\inc\dbg_trace.h;
   TRACE_SUBSYS_BACNET_GTW,                le_khe\tpai,                                BACNET;
   TRACE_SUBSYS_BACNET_STAC,               le_khe\tapi,                                BACNET;

@>@ ************************************ @<@
@>@ DPS @<@
   TRACE_SUBSYS_DPS_MGM,                   ge_nbg\dps_300,                             DPS_MGM,       0,             ge_nbg\dps_300\inc\dps_ltrccfg.h;  
   TRACE_SUBSYS_DPS_C0,                    ge_nbg\dps_300,                             DPS_C0,        0,             ge_nbg\dps_300\inc\dps_ltrccfg.h;  
   TRACE_SUBSYS_DPS_V1SL,                  ge_nbg\dps_300,                             DPS_V1SL,      0,             ge_nbg\dps_300\inc\dps_ltrccfg.h; 
   TRACE_SUBSYS_DPS_TIS,                   ge_nbg\dps_300,                             DPS_TIS,       0,             ge_nbg\dps_300\inc\dps_ltrccfg.h; 
   TRACE_SUBSYS_DPS_SYSIF,                 ge_nbg\dps_300,                             DPS_SYSIF,     0,             ge_nbg\dps_300\inc\dps_ltrccfg.h; 
   TRACE_SUBSYS_DPS_XBUS,                  ge_nbg\dps_300,                             DPS_XBUS,      0,             ge_nbg\dps_300\inc\dps_ltrccfg.h; 

@>@ ************************************ @<@
@>@ DP-Master @<@
   TRACE_SUBSYS_DP_SDBDP,                  ge_nbg\dp_300,                              DP_SDBDP,      0,             ge_nbg\dp300\sdbdp\inc\dpsdbtrc.h;


@>@ ************************************ @<@
@>@ PBK @<@
   TRACE_SUBSYS_PBK,                       le_nbg\pbk,                                 PBK,           1,             le_nbg\pbk\inc\pbktrace.h;
   TRACE_SUBSYS_PBK_KBUSVERB,              le_nbg\pbk,                                 PBK,           1,             le_nbg\pbk\inc\pbktrace.h;
   TRACE_SUBSYS_PBK_SERVER,                le_nbg\pbk,                                 PBK,           1,             le_nbg\pbk\inc\pbktrace.h;
   TRACE_SUBSYS_PBK_OPMUX,                 le_nbg\pbk,                                 PBK,           1,             le_nbg\pbk\inc\pbktrace.h;
   TRACE_SUBSYS_PBK_PBUS,                  le_nbg\pbk,                                 PBK,           1,             le_nbg\pbk\inc\pbktrace.h;
   TRACE_SUBSYS_PBK_CLIENT,                le_nbg\pbk,                                 PBK,           1,             le_nbg\pbk\inc\pbktrace.h;
   TRACE_SUBSYS_PBK_VERB,                  le_nbg\pbk,                                 PBK,           1,             le_nbg\pbk\inc\pbktrace.h;

@>@ ************************************ @<@
@>@ CBA @<@
   TRACE_SUBSYS_CBA_SYSA,                  le_nbg\cba\sysa,                            SYSA,          0,             le_nbg\cba\sysa\inc\sysatrc.h;
   TRACE_SUBSYS_CBA_RPC_SOCK,              le_nbg\cba\rpc_ad,                          CBASOCK,       0,             le_nbg\cba\rpc_ad\inc\rpcsock_trc.h;
   TRACE_SUBSYS_CBA_RPC,                   le_nbg\cba\rpc\co_rpc,                      RPC,           0,             le_nbg\cba\rpc\co_rpc\rpctrc.h;
   TRACE_SUBSYS_CBA_RPC,                   le_nbg\cba\rpc_ad\src,                      CBA,           0,             le_nbg\cba\rpc\co_rpc\rpctrc.h;
   TRACE_SUBSYS_CBA_DCOM,                  le_nbg\cba\dcomrt,                          DCOM,          0,             le_nbg\cba\dcomrt\co\dcomtrc.h;
   TRACE_SUBSYS_CBA_DCOM,                  le_nbg\cba\dcom_ad,                         DCOM,          0,             le_nbg\cba\dcomrt\co\dcomtrc.h;
   TRACE_SUBSYS_CBA_APPL_AMARSHAL,         le_nbg\cba\amsh\amsh,                       AMSH,          0,             le_nbg\cba\amsh\amsh\amtrc.h;
   TRACE_SUBSYS_CBA_APPL_S7,               le_nbg\cba\appl\s7,                         CBA,           0,             le_nbg\cba\appl\s7\cbas7trc.h;
   TRACE_SUBSYS_CBA_APPL_S7,               le_nbg\cba\appl_ad,                         CBA,           0,             le_nbg\cba\appl\s7\cbas7trc.h;
   TRACE_SUBSYS_CBA_APPL_ACCO,             le_nbg\cba\appl\acco,                       CBA,           0,             le_nbg\cba\appl\acco\accotrc.h;
   TRACE_SUBSYS_CBA_APPL_DEVICE,           le_nbg\cba\appl\device,                     CBA,           0,             le_nbg\cba\appl\device\devtrc.h;
   TRACE_SUBSYS_CBA_GATE,                  le_nbg\cba\gate,                            GATE,          0,             le_nbg\cba\gate\inc\gate_trace.h;
   TRACE_SUBSYS_CBA_TRACE,                 le_nbg\cba\base_ad,                         BASE,          0,             le_nbg\cba\base_ad\inc\basetrc.h;
   TRACE_SUBSYS_CBA_TRACE,                 le_nbg\cba\base_ad,                         CBA,           0,             le_nbg\cba\base_ad\inc\basetrc.h;
   TRACE_SUBSYS_CBA_SRT,                   le_nbg\cba\apsrt_ad\src,                    CBA,           0,             le_nbg\cba\apsrt_ad\inc\cbacntrc.h;
   TRACE_SUBSYS_CBA_KBUS,                  le_nbg\cba\kbcba\src,                       KB,            0,             le_nbg\cba\kbcba\inc\kbtrace.h;

@>@ ************************************ @<@
@>@ CBA-DIAG @<@
   TRACE_SUBSYS_CBA_DIAG,                  le_nbg\cba\diag,                            CBA_DIAG,      1,             le_nbg\cba\diag\inc\cbadiagtrccfg.h;
   TRACE_SUBSYS_CBA_DIAG_GSI_SOCK_CON,     le_nbg\cba\diag,                            CBA_DIAG,      1,             le_nbg\cba\diag\inc\cbadiagtrccfg.h;
   TRACE_SUBSYS_CBA_DIAG_GSI_ADP,          le_nbg\cba\diag,                            CBA_DIAG,      1,             le_nbg\cba\diag\inc\cbadiagtrccfg.h;
   TRACE_SUBSYS_CBA_DIAG_RPC_CCONTEXT,     le_nbg\cba\diag,                            CBA_DIAG,      1,             le_nbg\cba\diag\inc\cbadiagtrccfg.h;
   TRACE_SUBSYS_CBA_DIAG_RPC_SCONTEXT,     le_nbg\cba\diag,                            CBA_DIAG,      1,             le_nbg\cba\diag\inc\cbadiagtrccfg.h;
   TRACE_SUBSYS_CBA_DIAG_RPC_MOD,          le_nbg\cba\diag,                            CBA_DIAG,      1,             le_nbg\cba\diag\inc\cbadiagtrccfg.h;
   TRACE_SUBSYS_CBA_DIAG_RPC_REGIF,        le_nbg\cba\diag,                            CBA_DIAG,      1,             le_nbg\cba\diag\inc\cbadiagtrccfg.h;
   TRACE_SUBSYS_CBA_DIAG_DCOM_STAT,        le_nbg\cba\diag,                            CBA_DIAG,      1,             le_nbg\cba\diag\inc\cbadiagtrccfg.h;
   TRACE_SUBSYS_CBA_DIAG_DCOM_RUNOBJ,      le_nbg\cba\diag,                            CBA_DIAG,      1,             le_nbg\cba\diag\inc\cbadiagtrccfg.h;
   TRACE_SUBSYS_CBA_DIAG_DCOM_CIFC,        le_nbg\cba\diag,                            CBA_DIAG,      1,             le_nbg\cba\diag\inc\cbadiagtrccfg.h;
   TRACE_SUBSYS_CBA_DIAG_DCOM_SIFC,        le_nbg\cba\diag,                            CBA_DIAG,      1,             le_nbg\cba\diag\inc\cbadiagtrccfg.h;
   TRACE_SUBSYS_CBA_DIAG_PDEV,             le_nbg\cba\diag,                            CBA_DIAG,      1,             le_nbg\cba\diag\inc\cbadiagtrccfg.h;
   TRACE_SUBSYS_CBA_DIAG_LDEV,             le_nbg\cba\diag,                            CBA_DIAG,      1,             le_nbg\cba\diag\inc\cbadiagtrccfg.h;
   TRACE_SUBSYS_CBA_DIAG_RTAUTO,           le_nbg\cba\diag,                            CBA_DIAG,      1,             le_nbg\cba\diag\inc\cbadiagtrccfg.h;
   TRACE_SUBSYS_CBA_DIAG_PUTPROP,          le_nbg\cba\diag,                            CBA_DIAG,      1,             le_nbg\cba\diag\inc\cbadiagtrccfg.h;
   TRACE_SUBSYS_CBA_DIAG_GETPROP,          le_nbg\cba\diag,                            CBA_DIAG,      1,             le_nbg\cba\diag\inc\cbadiagtrccfg.h;
   TRACE_SUBSYS_CBA_DIAG_ACCO,             le_nbg\cba\diag,                            CBA_DIAG,      1,             le_nbg\cba\diag\inc\cbadiagtrccfg.h;
   TRACE_SUBSYS_CBA_DIAG_CONSCONN,         le_nbg\cba\diag,                            CBA_DIAG,      1,             le_nbg\cba\diag\inc\cbadiagtrccfg.h;
   TRACE_SUBSYS_CBA_DIAG_PING,             le_nbg\cba\diag,                            CBA_DIAG,      1,             le_nbg\cba\diag\inc\cbadiagtrccfg.h;
   TRACE_SUBSYS_CBA_DIAG_PROVCONN,         le_nbg\cba\diag,                            CBA_DIAG,      1,             le_nbg\cba\diag\inc\cbadiagtrccfg.h;
   TRACE_SUBSYS_CBA_DIAG_ADVISE,           le_nbg\cba\diag,                            CBA_DIAG,      1,             le_nbg\cba\diag\inc\cbadiagtrccfg.h;
   TRACE_SUBSYS_CBA_DIAG_LDEVSTATEADVISE,  le_nbg\cba\diag,                            CBA_DIAG,      1,             le_nbg\cba\diag\inc\cbadiagtrccfg.h;
   TRACE_SUBSYS_CBA_DIAG_LDEVGRERRADVISE,  le_nbg\cba\diag,                            CBA_DIAG,      1,             le_nbg\cba\diag\inc\cbadiagtrccfg.h;
   TRACE_SUBSYS_CBA_DIAG_ACCOGRERRADVISE,  le_nbg\cba\diag,                            CBA_DIAG,      1,             le_nbg\cba\diag\inc\cbadiagtrccfg.h;
   TRACE_SUBSYS_CBA_DIAG_AM_STAT,          le_nbg\cba\diag,                            CBA_DIAG,      1,             le_nbg\cba\diag\inc\cbadiagtrccfg.h;
   TRACE_SUBSYS_CBA_DIAG_GATECHANNEL,      le_nbg\cba\diag,                            CBA_DIAG,      1,             le_nbg\cba\diag\inc\cbadiagtrccfg.h;

@>@ ************************************ @<@
@>@ CP1628 @<@
   TRACE_SUBSYS_ADMMAIN,                   cp1613\cp1628\firmware\AdmMain,             ADMMAIN,       0,             cp1613\cp1628\firmware\AdmMain\inc\AdmMain_trc.h;
   TRACE_SUBSYS_ADMFLASH,                  cp1613\cp1628\firmware\admflash,            ADMFLASH,      0,             cp1613\cp1628\firmware\admflash\inc\admflash_trc.h;
   TRACE_SUBSYS_CLOCK,                     cp1613\cp1628\firmware\clock,               CLOCK,         0,             cp1613\cp1628\firmware\clock\inc\clock_trc.h;
   TRACE_SUBSYS_ISO,                       cp1613\cp1628\firmware\iso,                 ISO,           0,             cp1613\cp1628\firmware\iso\inc\iso_trc.h;
   TRACE_SUBSYS_MUXBIND,                   cp1613\cp1628\firmware\muxbind,             MUXBIND,       0,             cp1613\cp1628\firmware\muxbind\inc\muxbind_trc.h;
   TRACE_SUBSYS_NDIR,                      cp1613\cp1628\firmware\Ndir,                NDIR,          0,             cp1613\cp1628\firmware\Ndir\inc\Ndir_trc.h;
   TRACE_SUBSYS_S7ONTCP,                   cp1613\cp1628\firmware\s7ontcp,             S7ONTCP,       0,             cp1613\cp1628\firmware\s7ontcp\inc\s7ontcp_trc.h;
   TRACE_SUBSYS_SMDPRAM,                   cp1613\cp1628\firmware\SmDpram,             SMDPRAM,       0,             cp1613\cp1628\firmware\SmDpram\inc\SmDpram_trc.h;
   TRACE_SUBSYS_SNMP1628,                  cp1613\cp1628\firmware\snmp,                SNMP,          0,             cp1613\cp1628\firmware\snmp\inc\snmp_trc.h;   
   TRACE_SUBSYS_FUSION,                    cp1613\cp1628\firmware\fusion,              FUSION,        0,             cp1613\cp1628\firmware\inc\fusion_trc.h;
   TRACE_SUBSYS_TARGET,                    cp1613\cp1628\firmware\target,              TARGET,        0,             cp1613\cp1628\firmware\target\config\ads834x\target_trc.h;

@>@ ************************************ @<@
@>@ PBUS PLUS @<@
   TRACE_SUBSYS_PBUSP_ALARM,               cp1500\components\pbus\pbusp\rhp_src,           PBUSP_ALARM,      0,      cp1500\components\pbus\pbusp\include\pbp_trc.h;
   TRACE_SUBSYS_PBUSP_ALARM_RCV,           cp1500\components\pbus\pbusp\rhp_src,           PBUSP_ALARM_RCV,  0,      cp1500\components\pbus\pbusp\include\pbp_trc.h;
   TRACE_SUBSYS_PBUSP_ALARM_SND,           cp1500\components\pbus\pbusp\rhp_src,           PBUSP_ALARM_SND,  0,      cp1500\components\pbus\pbusp\include\pbp_trc.h;
   TRACE_SUBSYS_PBUSP_AZK,                 cp1500\components\pbus\pbusp\rhp_src,           PBUSP_AZK,        0,      cp1500\components\pbus\pbusp\include\pbp_trc.h;
   TRACE_SUBSYS_PBUSP_AZK_RCV,             cp1500\components\pbus\pbusp\rhp_src,           PBUSP_AZK_RCV,    0,      cp1500\components\pbus\pbusp\include\pbp_trc.h;
   TRACE_SUBSYS_PBUSP_AZK_SND,             cp1500\components\pbus\pbusp\rhp_src,           PBUSP_AZK_SND,    0,      cp1500\components\pbus\pbusp\include\pbp_trc.h;
   TRACE_SUBSYS_PBUSP_BUS,                 cp1500\components\pbus\pbusp\rhp_src,           PBUSP_BUS,        0,      cp1500\components\pbus\pbusp\include\pbp_trc.h;
   TRACE_SUBSYS_PBUSP_CLOCK,               cp1500\components\pbus\pbusp\rhp_src,           PBUSP_CLOCK,      0,      cp1500\components\pbus\pbusp\include\pbp_trc.h;
   TRACE_SUBSYS_PBUSP_IO,                  cp1500\components\pbus\pbusp\rhp_src,           PBUSP_IO,         0,      cp1500\components\pbus\pbusp\include\pbp_trc.h;

@>@ PBUSP base package -> subsystems of Slave @<@
   TRACE_SUBSYS_PSP_ALARM,                 cp1500\components\pbus\psp\rhp_src,             PSP_ALARM,        0,      cp1500\components\pbus\psp\include\psp_trc.h;
   TRACE_SUBSYS_PSP_BUSOBSERVE,            cp1500\components\pbus\psp\rhp_src,             PSP_BUSOBSERVE,   0,      cp1500\components\pbus\psp\include\psp_trc.h;
   TRACE_SUBSYS_PSP_IO,                    cp1500\components\pbus\psp\rhp_src,             PSP_IO,           0,      cp1500\components\pbus\psp\include\psp_trc.h;
   TRACE_SUBSYS_MOD_INTEGR,                cp1500\components\pbus\pbp_CpIntegr\TfModule,   MOD_INTEGR,       0,      cp1500\components\pbus\pbp_CpIntegr\include\mod_Integr_trc.h;

@>@ ************************************ @<@
@>@ Device Proxy für CP1616 @<@
   TRACE_SUBSYS_PE_PROXY,                  vxw_pnio_khe\cp1616dk\firmware\pe,              PE_PROXY,          0,      vxw_pnio_khe\cp1616dk\firmware\pe\inc\peproxy_cfg.h;
   TRACE_SUBSYS_DPRLIB_FW,                 vxw_pnio_khe\modules\dpram\dprlib,              DPRLIB_FW,         0,      vxw_pnio_khe\modules\dpram\dprlib\inc\dprlib_ltrc.h;
   TRACE_SUBSYS_DPRLIB_INTERN,             vxw_pnio_khe\modules\dpram\dprlib,              DPRLIB_INTERN,	  0,      vxw_pnio_khe\modules\dpram\dprlib\inc\dprlib_ltrc.h;
   TRACE_SUBSYS_DPR_SW,					   vxw_pnio_khe\modules\dpramswitch,               DPR_SW,	 		  0,      vxw_pnio_khe\modules\dpramswitch\inc\dprsw_ltrc.h;

|@@@END-LTRC-Scanner-Params@@@|*/

/*****************************************************************************/
/*  end of file LTRC_SUB.H                                                   */
/*****************************************************************************/

/*------------------------------------------------------------------------*/
/*   Copyright (C) Siemens AG 2009,  All Rights Reserved. Confidential.   */
/*------------------------------------------------------------------------*/
#endif      /* reinclude guard */

