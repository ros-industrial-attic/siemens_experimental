/*****************************************************************************/
/*   Copyright (C) SIEMENS CORP., 2013 All rights reserved.*/
/*****************************************************************************/
/*  F i l e               pniolib_vers.h                                     */
/*****************************************************************************/
/*  D e s c r i p t i o n:  Version header                                   */
/*                          Is used in the version resource file: *.rc       */
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
#ifndef _PNIOLIB_VERS_INCD_
#define _PNIOLIB_VERS_INCD_



// use global product number  (use CP16xx CD style)
#if 0
// SIMATIC NET CD vers:
//---------------------
#include "prod.h"
#include "build_nr.h"                    /* SINEC_BUILD_  ... */
#include "pniolib_vers_variant.h"

#define SINEC_DRV_MAJOR   CD_MAJOR
#define SINEC_DRV_MINOR   CD_MINOR_FILE
#define SINEC_DRV_VARIANT MY_PNIOLIB_VER_VARIANT
#define SINEC_DRV_BUILD   SINEC_BUILD
#else
// CP1616DK vers:
//---------------

#include "siemens.h"
#include "fw1616dk_vers.h"  /* fw version constant FW_VER_MAJOR, FW_VER_MINOR1... */
                               /* and fw_vers_bldnum.h: VERSION_BUILD_NUMBER, VERSION_BUILD_INCREMENT    */
#include "pniolib_vers_variant.h"
#define SINEC_DRV_MAJOR   FW_VER_MAJOR
#define SINEC_DRV_MINOR   FW_VER_MINOR1
#define SINEC_DRV_VARIANT FW_VER_MINOR2   /* MY_PNIOLIB_VER_VARIANT not used */
#define SINEC_DRV_BUILD   VERSION_BUILD_NUMBER

#endif // 0

// stringizing macros
#ifndef chSTR
#define chSTR(x)  #x
#define chSTR2(x) chSTR(x)
#endif // chSTR
#define chSTR3(x) "0" chSTR(x)


#ifdef _DEBUG
#define DEBREL "-DBG"
#else
#define DEBREL "-REL"
#endif

#ifdef _LOCAL_GEN
#define PROVER "L "
#define X_BLD  " "               /* SpecialBuild string extension (not used yet)*/
#else
#define PROVER "V "
#define X_BLD  " "               /* empty */
#endif

// Convert version numbers to zero prefixed version strings
#if FW_VER_MAJOR < 10
#define FW_VER_MAJOR_2D chSTR3(FW_VER_MAJOR)
#else
#define FW_VER_MAJOR_2D chSTR2(FW_VER_MAJOR)
#endif

#if FW_VER_MINOR1 < 10
#define FW_VER_MINOR1_2D chSTR3(FW_VER_MINOR1)
#else
#define FW_VER_MINOR1_2D chSTR2(FW_VER_MINOR1)
#endif

#if FW_VER_MINOR2 < 10
#define FW_VER_MINOR2_2D chSTR3(FW_VER_MINOR2)
#else
#define FW_VER_MINOR2_2D chSTR2(FW_VER_MINOR2)
#endif

#if FW_VER_MINOR3 < 10
#define FW_VER_MINOR3_2D chSTR3(FW_VER_MINOR3)
#else
#define FW_VER_MINOR3_2D chSTR2(FW_VER_MINOR3)
#endif

#define MY_FILE_VERSION      SINEC_DRV_MAJOR,SINEC_DRV_MINOR,SINEC_DRV_VARIANT,SINEC_DRV_BUILD
#define MY_FILE_VERSION_STR  "V " chSTR2(SINEC_DRV_MAJOR) "." chSTR2(SINEC_DRV_MINOR) \
                             "." chSTR2(SINEC_DRV_VARIANT) "." \
                             chSTR2(SINEC_DRV_BUILD) DEBREL

// substitution for prod.h:
// PEG 14.11.2007 decidsion: product version same as CD-Label == V1.2.3
#define VER_PRODUCTVERSION     SINEC_DRV_MAJOR,SINEC_DRV_MINOR,SINEC_DRV_VARIANT
#define VER_PRODUCTVERSION_STR "V " chSTR2(SINEC_DRV_MAJOR) "." chSTR2(SINEC_DRV_MINOR) \
                               "." chSTR2(SINEC_DRV_VARIANT)

#define MY_COMMENTS_STR           "PNIO Base \0"
#define MY_FILE_DESCRIPTION_STR   "PNIO Base \0"
#define MY_INTERNAL_NAME          "PNIO Base  \0"
#define MY_ORIGINAL_FILENAME      "pniolib.dll\0"
#define MY_PRIVATE_BUILD_STR      "PNIO Base " DEBREL " \0"
#define MY_SPECIAL_BUILD_STR      "PNIO Base " DEBREL X_BLD " \0"

#endif // _PNIOLIB_VERS_INCD_

