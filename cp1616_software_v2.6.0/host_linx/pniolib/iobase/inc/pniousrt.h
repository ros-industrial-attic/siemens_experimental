/****************************************************************************/
/* Copyright (C) SIEMENS CORP., 2013 All rights reserved.*/
/*                                                                          */
/* file: PNIOUSRT.H                                                         */
/*                                                                          */
/* Description:                                                             */
/*  data types and function declarations for IO-Base controller             */
/*  user interface                                                          */
/*                                                                          */
/****************************************************************************/
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

#ifndef PNIOUSRT_H
#define PNIOUSRT_H

#include "pniobase.h"

#undef ATTR_PACKED
#if defined(_MSC_VER)
#pragma pack( push, safe_old_packing, 4 )
#define  ATTR_PACKED
#elif defined(__GNUC__)
#define ATTR_PACKED  __attribute__ ((aligned (4)))
#elif defined(BYTE_ATTR_PACKING)
#include "pack.h"
#define ATTR_PACKED PPC_BYTE_PACKED
#else
#error please adapt pniousrt.h header for your compiler
#endif

#undef PNIO_CODE_ATTR
#ifdef PASCAL
    #define PNIO_CODE_ATTR __stdcall
#else
    #define PNIO_CODE_ATTR
#endif

#ifdef __cplusplus
extern "C" {
#endif
    PNIO_UINT32 PNIO_CODE_ATTR PNIO_data_test(PNIO_UINT32 Handle,      /* in */
        PNIO_UINT8 * pBuffer,   /* in */
        PNIO_UINT32 BufLen     /* in */
        );

    PNIO_UINT32 PNIO_CODE_ATTR PNIO_device_data_test(PNIO_UINT32 Handle,      /* in */
        PNIO_UINT8 * pBuffer,   /* in */
        PNIO_UINT32 BufLen     /* in */
        );
#ifdef __cplusplus
}
#endif
#if defined(_MSC_VER)
#pragma pack( pop, safe_old_packing )
#elif defined(BYTE_ATTR_PACKING)
#include "unpack.h"
#endif

#endif /* PNIOUSRT_H */
