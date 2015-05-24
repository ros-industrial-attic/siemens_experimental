/*****************************************************************************/
/*   Copyright (C) SIEMENS CORP., 2013 All rights reserved.*/
/*****************************************************************************/
/*  F i l e               driver_vers.h                                      */
/*****************************************************************************/
/*  D e s c r i p t i o n:  Common header declarations                       */
/*                          This is common to Linux and vxWorks !!!          */
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
/*   Description  : Defines version number and describes the version history.*/
/*                                                                           */
/*   History      : All changes have to be documented below in this file.    */
/*   ========       Note: Do NOT use leading 0 in the version numbers.       */
/*                  E.g.: 08 is interpreted as octal and cause an error!!!   */
/*                                                                           */
/*****************************************************************************/

#ifndef _DRIVER_VERS_H_
#define _DRIVER_VERS_H_

/* driver version: The full driver version number is e.g.: '1.2.3.4'
 *                 First 2 digits are the CD firmware + host sw version,
 *                 third digit is a driver version defined here,
 *                 fourth digit is a build number defined by the build process.
 * IMPORTANT NOTE: Because of dependencies with the firmware, we build the driver
 *                 every time the FW is generated. Therefor we are using the same
 *                 minor version number. The build number is always incremented.
 *                 Increment DRV_VER_MINOR manualy only if driver must be changed
 *                 without a new firmware build.
 *                 See also: \firmware\inc\fw1616dk_vers.h, fw_vers_bldnum.h
 */
#define DRV_VER_MINOR    FW_VER_MINOR2

/* #define DRV_VER_MINOR    xxx        */


/* version description begin:  (current first, oldest last)

V1.1.4.x: PL: 04.08.2005 - Firmware version string update: Last digit is now a build number which is
                           'x' == Build number in the version string was added.
      0 : PL: 03.08.2005 - 1st Host-Linux + Firmware same CS version.

----- version description end -----
*/


#endif /* _DRIVER_VERS_H_ */

