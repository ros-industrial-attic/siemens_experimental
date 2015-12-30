/****************************************************************************/
/*                                                                          */
/*  Copyright (C) SIEMENS CORP., 2015 All rights reserved.*/
/*                                                                          */
/*    file: PNIOERRX.H                                                      */
/*                                                                          */
/*    Description:                                                          */
/*    error codes for IO-Base user interface                                */
/*                                                                          */
/****************************************************************************/

#ifndef _PNIO_ERRX_H
#define _PNIO_ERRX_H


#define PNIO_OK                             0x00000000  /* success */
#define PNIO_OK_PE_WAKE_ONLINE				0x00000001
#define PNIO_OK_PE_WAKE_OFFLINE				0x00000002

/* warnings */
#define PNIO_WARN_IRT_INCONSISTENT          0x00000010  /* IRT Data may be inconsistent */
#define PNIO_WARN_NO_SUBMODULES             0x00000011  /* no submodules to be updated */
#define PNIO_WARN_LOCAL_STATE_BAD           0x00000012  /* data was written with local state PNIO_S_BAD, because not all components of splitted module have local state PNIO_S_GOOD */

/* wrong functions calls, parameter errors */
#define PNIO_ERR_PRM_HND                    0x00000101  /* parameter Handle is illegal */
#define PNIO_ERR_PRM_BUF                    0x00000102  /* parameter buffer is NULL-Ptr */
#define PNIO_ERR_PRM_LEN                    0x00000103  /* parameter length is wrong    */
#define PNIO_ERR_PRM_ADD                    0x00000104  /* parameter address is wrong   */
#define PNIO_ERR_PRM_RSTATE                 0x00000105  /* parameter remote state is NULL-Ptr */
#define PNIO_ERR_PRM_CALLBACK               0x00000106  /* parameter cbf is illegal           */
#define PNIO_ERR_PRM_TYPE                   0x00000107  /* parameter type has no valid value  */
#define PNIO_ERR_PRM_EXT_PAR                0x00000108  /* parameter ExtPar has no valid value */
#define PNIO_ERR_PRM_IO_TYPE                0x00000109  /* parameter PNIO_ADDR::IODataType is wrong */
#define PNIO_ERR_PRM_CP_ID                  0x0000010A  /* parameter CpIndex is wrong, propably driver is not loaded */
#define PNIO_ERR_PRM_LOC_STATE              0x0000010B  /* parameter IOlocState has no valid value  */
#define PNIO_ERR_PRM_REC_INDEX              0x0000010C  /* parameter RecordIndex has no valid value */
#define PNIO_ERR_PRM_TIMEOUT                0x0000010D  /* parameter timeout has no valid value */
#define PNIO_ERR_PRM_DEV_ANNOTATION         0x0000010E  /* parameter annotation has no valid value */
#define PNIO_ERR_PRM_DEV_STATE              0x0000010F  /* parameter state has no valid value */
#define PNIO_ERR_PRM_PCBF                   0x00000110  /* parameter pCbf has no valid value */
#define PNIO_ERR_PRM_MAX_AR_VALUE           0x00000111  /* parameter MaxAR has no valid value */
#define PNIO_ERR_PRM_ACCESS_TYPE            0x00000112  /* parameter AccessType has no valid value */
#define PNIO_ERR_PRM_POINTER                0x00000113  /* an invalid pointer was passed */
#define PNIO_ERR_PRM_INVALIDARG             0x00000114  /* an invalid argument was passed */
#define PNIO_ERR_PRM_MEASURE_NUMBER         0x00000115  /* wrong Measure No in cycle statistics, must be -1 (actual measure) up to 49 */
#define PNIO_ERR_PRM_CYCLE_OFFSET           0x00000116  /* wrong Offset for cycle info buffer (must be 0 to 19) */
#define PNIO_ERR_PRM_ROUTER_ADD             0x00000117  /* address used by io router */

/* instance errors */
#define PNIO_ERR_WRONG_HND                  0x00000201  /* unknown handle */
#define PNIO_ERR_MAX_REACHED                0x00000202  /* maximal number of opens reached; close unused applications */
#define PNIO_ERR_CREATE_INSTANCE            0x00000203  /* fatal error, reboot your system */
#define PNIO_ERR_MODE_VALUE                 0x00000204  /* parameter mode has no valid value */
#define PNIO_ERR_OPFAULT_NOT_REG            0x00000205  /* register OPFAULT callback befor register STARTOP callback */
#define PNIO_ERR_NEWCYCLE_SEQUENCE_REG      0x00000206  /* register NEWCYCLE callback befor register STARTOP callback */
#define PNIO_ERR_NETWORK_PROT_NOT_AVAILABLE 0x00000207  /* network protocol not available, check card configuration */

/* other errors */
#define PNIO_ERR_NO_CONNECTION              0x00000301  /* device data not available, because device is not connected to controller */
#define PNIO_ERR_OS_RES                     0x00000302  /* fatal error, no more operation system resources available */
#define PNIO_ERR_ALREADY_DONE               0x00000303  /* action was already performed */
/* spelling error, compatibility hack */
#define PNIO_ERR_ALLREADY_DONE              PNIO_ERR_ALREADY_DONE
#define PNIO_ERR_NO_CONFIG                  0x00000304  /* no configuration for this index available                                */
#define PNIO_ERR_SET_MODE_NOT_ALLOWED       0x00000305  /* PNIO_set_mode not allowed, use PNIO_CEP_MODE_CTRL by PNIO_controller_open */
#define PNIO_ERR_DEV_ACT_NOT_ALLOWED        0x00000306  /* PNIO_device_activate not allowed, use PNIO_CEP_MODE_CTRL by PNIO_controller_open */
#define PNIO_ERR_NO_LIC_SERVER              0x00000307  /* licence server not running, check your installation                              */
#define PNIO_ERR_VALUE_LEN                  0x00000308  /* wrong length value                                                               */
#define PNIO_ERR_SEQUENCE                   0x00000309  /* wrong calling sequence                                                           */
#define PNIO_ERR_INVALID_CONFIG             0x0000030A  /* invalid configuration, check your configuration                                  */
#define PNIO_ERR_UNKNOWN_ADDR               0x0000030B  /* address unknown in configuration, check your configuration                       */
#define PNIO_ERR_NO_RESOURCE                0x0000030C  /* no resouce too many requests been processed                                      */
#define PNIO_ERR_CONFIG_IN_UPDATE           0x0000030D  /* configuration update is in progress or CP is in STOP state, try again later */
#define PNIO_ERR_NO_FW_COMMUNICATION        0x0000030E  /* no communication with firmware, reset cp or try again later */
#define PNIO_ERR_STARTOP_NOT_REGISTERED     0x0000030F  /* no synchonous function allowed, use PNIO_CEP_SYNC_MODE by PNIO_controller_open or PNIO_device_open */
#define PNIO_ERR_OWNED                      0x00000310  /* interface-submodule cannot be removed because it is owned by an AR */
#define PNIO_ERR_START_THREAD_FAILED        0x00000311  /* failed to start thread, propably by lack of pthread resources */
#define PNIO_ERR_START_RT_THREAD_FAILED     0x00000312  /* failed to start realtime thread, propably you need root capability to do it */
#define PNIO_ERR_DRIVER_IOCTL_FAILED        0x00000313  /* failed to ioctl driver, propably API version mismatch */
#define PNIO_ERR_AFTER_EXCEPTION            0x00000314  /* exception ocurred, save exception info (see manual) and reset cp */
#define PNIO_ERR_NO_CYCLE_INFO_DATA         0x00000315  /* no cycle data available */
#define PNIO_ERR_SESSION                    0x00000316  /* request belongs to an old session */
#define PNIO_ERR_ALARM_DATA_FORMAT          0x00000317  /* wrong format of alarm data */
#define PNIO_ERR_ABORT                      0x00000318  /* operation was aborted */
#define PNIO_ERR_CORRUPTED_DATA             0x00000319  /* datas are corrupter or have wrong format */
#define PNIO_ERR_FLASH_ACCESS               0x0000031A  /* error by flash operations */
#define PNIO_ERR_WRONG_RQB_LEN              0x0000031B  /* wrong length of request block at firmware interface, firmware not compatible to host sw */
#define PNIO_ERR_NO_RESET_VERIFICATION      0x0000031C  /* reset request was sendet to firmware, but firmware rut up can't be verified */

/* internal error */
#define PNIO_ERR_INTERNAL                   0x000003FF  /* fatal error, contact SIEMENS hotline */

#endif  /* _PNIO_ERRX_H */
