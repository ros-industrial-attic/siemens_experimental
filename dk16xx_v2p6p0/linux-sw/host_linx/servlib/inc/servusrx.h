/****************************************************************************
*  Copyright (C) SIEMENS CORP., 2013 All rights reserved.
*****************************************************************************
* FILE NAME    : servusex.h
* ---------------------------------------------------------------------------
* DESCRIPTION  : local download user interface functions
*****************************************************************************
* Diese Software ist Freeware. Sie wird Ihnen unentgeltlich zur Verfuegung  *
* gestellt. Sie darf frei kopiert, modifiziert und benutzt sowie an Dritte  *
* weitergegeben werden. Die Software darf nur unter Beibehaltung aller      *
* Schutzrechtsvermerke sowie nur vollstaedig und unveraendert weitergegeben *
* werden. Die kommerzielle Weitergabe an Dritte (z.B. im Rahmen von         *
* Share-/Freeware-Distributionen) ist nur mit vorheriger schriftlicher      *
* Genehmigung der Siemens Aktiengesellschaft erlaubt.                       *
* DA DIE SOFTWARE IHNEN UNENTGELTLICH UEBERLASSEN WIRD, KOENNEN DIE AUTOREN *
* UND RECHTSINHABER DAFUER KEINE HAFTUNG UEBERNEHMEN. IHRE BENUTZUNG ERFOLGT*
* AUF EIGENE GEFAHR UND VERANTWORTUNG. DIE AUTOREN UND RECHTSINHABER HAFTEN *
* NUR FUER VORSATZ UND GROBE FAHRLAESSIGKEIT. WEITERGEHENDE ANSPRUECHE SIND *
* AUSGESCHLOSSEN. INSBESONDERE HAFTEN DIE AUTOREN UND RECHTSINHABER NICHT   *
* FUER ETWAIGE MAENGEL ODER FOLGESCHAEDEN.                                  *
* Falls Sie Fehler in der Software bemerken, teilen Sie es uns bitte mit.   *
*                                                                           *
* This Software is Freeware. It is distributed for free. You may copy,      *
* modify and use it for free as well as distribute it to others. You may    *
* only distribute it as a whole, unmodified and with all trademarks and     *
* copyright notices. You may not distribute it commercially (e.g. as a      *
* Share-/Freeware-Distributor) without the express written permission of    *
* Siemens Aktiengesellschaft. SINCE THIS SOFTWARE IS DISTRIBUTED FOR FREE,  *
* IT IS PROVIDED "AS IS" WITHOUT ANY REPRESENTATION OR WARRANTY OF ANY KIND *
* EITHER EXPRESSED OR IMPLIED INCLUDING BUT NOT LIMITED TO IMPLIED          *
* WARRANTIES FOR MERCHANTIBILITY OR FITNESS FOR USE. ANY USE OF THE         *
* APPLICATION EXAMPLE IS ON YOUR OWN RISK AND RESPONSIBILITY.               *
* If you happen to find any faults in it please tell us.                    *
*****************************************************************************/

#ifndef SERVUSER_H
#define SERVUSER_H

#include "pniobase.h"

#undef ATTR_PACKED
#if defined(_MSC_VER)
  #pragma pack( push, safe_old_packing, 4 )
  #define ATTR_PACKED
#elif defined(__GNUC__)
  #define ATTR_PACKED  __attribute__ ((aligned (4)))
#elif defined(BYTE_ATTR_PACKING)
  #include "pack.h"
  #define ATTR_PACKED PPC_BYTE_PACKED
#else
  #error please adapt pniousrx.h header for your compiler
#endif

#undef PNIO_CODE_ATTR
#ifdef PASCAL
  #define PNIO_CODE_ATTR __stdcall
#else
  #define PNIO_CODE_ATTR
#endif


typedef struct serv_cp_tm_s {
  PNIO_UINT32 tm_sec;     /* seconds after the minute - [0,59] */
  PNIO_UINT32 tm_min;     /* minutes after the hour - [0,59] */
  PNIO_UINT32 tm_hour;    /* hours since midnight - [0,23] */
  PNIO_UINT32 tm_mday;    /* day of the month - [1,31] */
  PNIO_UINT32 tm_mon;     /* months since January - [0,11] */
  PNIO_UINT32 tm_year;    /* years since 1900 */
  PNIO_UINT32 tm_wday;    /* days since Sunday - [0,6] */
  PNIO_UINT32 tm_yday;    /* days since January 1 - [0,365] */
  PNIO_UINT32 tm_isdst;   /* daylight savings time flag */
} ATTR_PACKED SERV_CP_TM_TYPE;


typedef struct pnio_cp_set_time_s {
  PNIO_UINT32      unix_utc_time;    /* UTC time unix  */
  PNIO_UINT32      reserved_t_hi;    /* reserved time_t 64 */
  PNIO_UINT32      systime_ms;       /* time millisecs */
  SERV_CP_TM_TYPE  local_tm;         /* local time     */
} ATTR_PACKED PNIO_CP_SET_TIME_TYPE;


typedef struct {
  PNIO_UINT32      blk_len;    /* out: =16: ip v4 vers */
  PNIO_UINT8       pnio_ip[4];         /* ip address   */
  PNIO_UINT8       pnio_net_mask[4];   /* network mask */
  PNIO_UINT8       pnio_gw_ip[4];      /* gateway addr */
} ATTR_PACKED PNIO_CP_ADDR_INFO_TYPE;

typedef enum
{
  PNIO_CP_DLD_UNKNOWN     = 0,
  PNIO_CP_DLD_FIRMWARE    = 1, /* firmware image, *.fwl file */
  PNIO_CP_DLD_CONFIG      = 2, /* configuration, *.xdb file */
} PNIO_CP_DLD_TYPE;

typedef enum
{
  PNIO_CP_RESET_SAFE      = 0, /* take into consideration the running application */
  PNIO_CP_RESET_FORCE     = 1
} PNIO_CP_RESET_TYPE;


/* SERV_FW_VERS_TYPE:
 *     Firmware version information. Each value is a unsigned integer.
 *     v1: major version number
 *     v2: minor1 version number
 *     v3: minor2 version number
 *     v4: build number
 *     v4: build increment
 *     v5: reserved, not used, binary zero
 */
typedef struct serv_fw_vers_s {
  PNIO_UINT16    v1;
  PNIO_UINT16    v2;
  PNIO_UINT16    v3;
  PNIO_UINT16    v4;
  PNIO_UINT16    v5;
  PNIO_UINT16    reserved;
} ATTR_PACKED SERV_FW_VERS_TYPE;


/* SERV_CP_FW_INFO_TYPE:
 *     Structure delivered by SERV_CP_get_fw_info() function call.
 *     Values received from the firmware are copied by the ServLib
 *     into this structure. Values in the structure are valid only
 *     if the function SERV_CP_get_fw_info() returns PNIO_OK.
 */
typedef struct serv_cp_fw_info_s {
    /* fw_version: five version nubers: major, minor,....
     */
    SERV_FW_VERS_TYPE fw_version;
    /* cp_ser_nr: zero terminated string, max length is 15+1.
     *            unique board id, set during the production.
     */
    PNIO_UINT8        cp_ser_nr[16];
    /* mlfb: zero terminated string, max length is 31+1.
     *       hardware catalog order number
     *       format for cp1604: 6GK1 160-4AA00, last four characters
     *       are not relevant, are internal version numbers
     */
    PNIO_UINT8        mlfb[32];
    /* hw_version: harware version number, unsigned integer
     */
    PNIO_UINT16       hw_version;
    /* mac: 1st MAC address of the board, each board has two internal
     *      board mac adresses and one additional mac addr. for each port
     *      The 1st internal mac addr. is used by the host ip stack,
     *      the second internal mac addr. is used by the PNIO stack.
     */
    PNIO_UINT8        mac[6];
    /* ip suite: ip v4 address, subnet mask, default gateway
     *           each element is in the network format.
     */
    PNIO_UINT32       ip_addr;
    PNIO_UINT32       ip_mask;
    PNIO_UINT32       default_router;
    /* name: StationName (davice name)
     *       zero terminated character string
     */
    PNIO_UINT8        name[256];
    /* TypeOfStation: PNIO TypeOfStation (davice type)
     *                zero terminated character string
     */
    PNIO_UINT8        TypeOfStation[256];

} ATTR_PACKED SERV_CP_FW_INFO_TYPE;


#ifdef __cplusplus
extern "C" {
#endif

  PNIO_UINT32 PNIO_CODE_ATTR SERV_CP_reset (
                                           PNIO_UINT32              CpIndex,
                                           PNIO_CP_RESET_TYPE       ResetType);

  PNIO_UINT32 PNIO_CODE_ATTR SERV_CP_download (
                                              PNIO_UINT32              CpIndex,
                                              PNIO_CP_DLD_TYPE         DataType,
                                              PNIO_UINT8             * pData,
                                              PNIO_UINT32              DataLen);

  PNIO_UINT32 PNIO_CODE_ATTR SERV_CP_set_time ( PNIO_UINT32 CpIndex, PNIO_CP_SET_TIME_TYPE *pCpTm );


  PNIO_UINT32 PNIO_CODE_ATTR SERV_CP_set_type_of_station (PNIO_UINT32 CpIndex, char* TypeName);

  PNIO_UINT32 PNIO_CODE_ATTR SERV_CP_get_fw_info (PNIO_UINT32 CpIndex, SERV_CP_FW_INFO_TYPE* p_info);

#ifdef __cplusplus
}
#endif

typedef enum
{
  SERV_CP_INFO_STOP_REQ = 1,    /* firmware ask for restart, close application */
  SERV_CP_INFO_PDEV_DATA,       /* port devices alarms */
  SERV_CP_INFO_PDEV_INIT_DATA,  /* initial port devices states */
  SERV_CP_INFO_LED_STATE        /* LED on/off notification */

} SERV_CP_INFO_TYPE;

typedef struct /* for details refer to IEC 61158-6 */
{
  PNIO_UINT16 Channel; /* chapter "Coding of the field ChannelNumber"
                                            0x0000-0x7FFF manufacturer specific
                                            0x8000 Submodule: Submodule
                                            0x8001 - 0xFFFF: reserved */
  PNIO_UINT16 Properties; /* chapter "Coding of the field ChannelProperties"
                                           Bit 0 - 7: ChannelProperties.Type
                                           Bit 8: ChannelProperties.Accumulative
                                                  Bit 9 - 10: ChannelProperties.Maintenance
                                                  Bit 11 - 12: ChannelProperties.Specifier
                                                  Bit 13 - 15: ChannelProperties.Direction */
  PNIO_UINT16 ErrType; /* chapter "Coding of the field ChannelErrorType" */

  PNIO_UINT16 ExtErrType; /* chapter "Coding of the field ExtChannelErrorType" */
  PNIO_UINT16 ExtAdvAlLo; /* chapter "Coding of the field ExtChannelAddValue" */
  PNIO_UINT16 ExtAdvAlHi; /* chapter "Coding of the field ExtChannelAddValue" */
} ATTR_PACKED SERV_CIB_PDEV_EXTCHNL_DIAG; /* coding of the ExtChannelDiagnosisData */


typedef struct /* SERV_CIB_PDEV_DATA::Version = 0 */
{
  PNIO_UINT32       Version;    /* version of SERV_CIB_PDEV_DATA structure */
  PNIO_IO_TYPE      IODataType; /* input, output */
  PNIO_UINT32       LogAddr;    /* logical address, configured by engineering tool */
  PNIO_UINT32       Slot;       /* slot number */
  PNIO_UINT32       Subslot;    /* subslot number */
  PNIO_ALARM_TYPE   AlarmType;  /* alarm type */
  PNIO_APRIO_TYPE   AlarmPriority; /* alarm priority */
  PNIO_UINT32       AlarmSequence; /* alarm sequence number */
  PNIO_UINT32       StationNr;     /* station number, configured by engineering tool */
  PNIO_UINT8         Reserved1[4];     /* reserved, don't use, silentalarm */
  PNIO_UINT8         Reserved2[48];    /* reserved, don't use, obstart header */
  PNIO_ALARM_TINFO   AlarmTinfo;       /* configuration details */
  SERV_CIB_PDEV_EXTCHNL_DIAG    Diagnostics;   /* external channel diagnosis data */
  PNIO_UINT8          DiagDs[4];               /* see STEP7 manual, Diagnosisalarm-OB (OB 82) */

  PNIO_ALARM_INFO   AlarmAinfo;                /* alarm data */

} ATTR_PACKED SERV_CIB_PDEV_DATA;

#define SERV_CIB_PS_DISTURBED                  0x0001
#define SERV_CIB_PS_EXISTS                     0x0002
#define SERV_CIB_PS_NOT_AVAILABLE              0x0004
#define SERV_CIB_PS_MAINTANANCE_REQUIRED       0x0100
#define SERV_CIB_PS_MAINTANANCE_DEMANDED       0x0200

typedef struct
{
  PNIO_UINT8     Reserved1;
  PNIO_UINT8     Reserved2;
  PNIO_UINT16    Reserved3;

  PNIO_IO_TYPE   IODataType; /* input, output */
  PNIO_UINT32    LogAddr;    /* logical address, configured by engineering tool */
  PNIO_UINT32    Slot;       /* slot number */
  PNIO_UINT32    Subslot;    /* subslot number */

  PNIO_UINT16    PortState;  /* bit field, SERV_CIB_PS_... */

  PNIO_UINT16    Reserved4;
} ATTR_PACKED SERV_CIB_PDEV_INIT_STATE;

typedef enum
{
  SERV_CIB_LED_TYPE_BF = 1,  /* bus fault led */
  SERV_CIB_LED_TYPE_SF = 2   /* group fault led */
} SERV_CIB_LED_TYPE;

typedef enum
{
  SERV_CIB_LED_STATE_OFF = 1, /* off */
  SERV_CIB_LED_STATE_ON  = 2  /* on */
} SERV_CIB_LED_STATE;

typedef struct
{
  SERV_CIB_LED_TYPE     LedType;  /* led type */
  SERV_CIB_LED_STATE    LedState; /* led state */
  PNIO_UINT8            Reserved1[4];
} ATTR_PACKED SERV_CIB_LED;

#define SERV_CIB_PDEV_MAX_PORT_ITEMS 8

typedef struct
{
  PNIO_UINT32               ItemNumber;
  SERV_CIB_PDEV_INIT_STATE  InitState[SERV_CIB_PDEV_MAX_PORT_ITEMS];
} ATTR_PACKED SERV_CIB_PDEV_INIT_DATA;

typedef struct
{
  PNIO_UINT32        Version;
  PNIO_UINT32        Handle;
  PNIO_UINT32        CpIndex;
  SERV_CP_INFO_TYPE  InfoType;
  union
  {
    SERV_CIB_PDEV_DATA        PdevData;       /* for SERV_CP_INFO_PDEV */
    SERV_CIB_PDEV_INIT_DATA   PdevInitData;   /* for SERV_CP_INFO_PDEV_INIT */
    SERV_CIB_LED              LedInfo;        /* for SERV_CP_INFO_LED_STATE */

  } u;
} ATTR_PACKED SERV_CP_INFO_PRM;

typedef void (*SERV_CP_INFO_CBF)(SERV_CP_INFO_PRM *pCbfPrm);

#ifdef __cplusplus
extern "C" {
#endif

  PNIO_UINT32 PNIO_CODE_ATTR SERV_CP_info_open(PNIO_UINT32 CpIndex, PNIO_UINT32  * Handle,
                                  PNIO_UINT32 Reserved);

  PNIO_UINT32 PNIO_CODE_ATTR SERV_CP_info_close(PNIO_UINT32 Handle);


  PNIO_UINT32 PNIO_CODE_ATTR SERV_CP_info_register_cbf(PNIO_UINT32 Handle,
                                          SERV_CP_INFO_TYPE InfoType, SERV_CP_INFO_CBF Cbf,
                                           PNIO_UINT32  Reserved);
#ifdef __cplusplus
}
#endif


#if defined(_MSC_VER)
  #pragma pack( pop, safe_old_packing )
#elif defined(BYTE_ATTR_PACKING)
  #include "unpack.h"
#endif

#endif /* SERVUSER_H */
