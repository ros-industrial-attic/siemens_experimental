#ifndef TRACEINFO_H                    /* ----- reinclude-protection ----- */
#define TRACEINFO_H
/****************************************************************************
* Copyright (C) SIEMENS CORP., 2013 All rights reserved.
* All Rights reserved                                                       *
*****************************************************************************
* FILE NAME    : traceinfo.h
* ---------------------------------------------------------------------------
* DESCRIPTION  : Subsystem definitions for firmware trace
*****************************************************************************/
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


#define EXCEPTION_BUFFER_START  0x504100UL
#define EXCEPTION_BUFFER_LENGTH 0x4000UL
#define TRACE_START             0x508100UL
#define TRACE_LENGTH            0xf7f00UL


typedef uint32_t TRACE_LINE[4];

typedef struct ltrc_text_info
{
    unsigned long   id;
    unsigned long   id_extension;
    char            name[32];
} LTRC_TEXT_INFO_TYPE;


typedef struct ltrc_buffer_admin
{
    uint32_t    buffer_id;
    uint32_t    size;
    uint32_t    line_number;
    uint32_t    line;
    uint32_t    wr_pos;
    uint32_t    wr_mask;
    uint8_t     is_fixed;
    uint8_t     is_freezed;
} LTRC_BUFFER_ADMIN_TYPE;

const unsigned long TRACE_ADMIN_OFFSET = ((TRACE_LENGTH - sizeof(LTRC_BUFFER_ADMIN_TYPE)) /
                                          sizeof(TRACE_LINE)) * sizeof(TRACE_LINE);


#endif/* of TRACEINFO_H */
