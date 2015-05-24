/*****************************************************************************/
/*                                                                           */
/*  Copyright (C) SIEMENS CORP., 2013 All rights reserved.*/
/*                                                                           */
/*    file: l2eth_lib_cfg.h                                                  */
/*                                                                           */
/*    Description:                                                           */
/*    os abstraction for l2library                                           */
/*                                                                           */
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

#ifndef L2ETH_LIB_CFG_H
#define L2ETH_LIB_CFG_H

typedef DPR_DRV_HANDLE L2ETH_DRV_HANDLE;
#define L2ETH_DRV_OPEN DPR_DRV_OPEN
#define L2ETH_DRV_CLOSE DPR_DRV_CLOSE
#define L2ETH_DRV_WRITE DPR_DRV_WRITE
#define L2ETH_DRV_READ DPR_DRV_READ
#define L2ETH_DRV_ERROR DPR_DRV_ERROR
#define L2ETH_DRV_IOCTL DPR_DRV_IOCTL
#define L2ETH_DRV_MMAP DPR_DRV_MMAP
#define L2ETH_DRV_MUNMAP DPR_DRV_MUNMAP

#define L2ETH_STRERROR DPR_STRERROR


typedef DPR_MUTEX L2ETH_MUTEX;
#define L2ETH_MUTEX_CREATE_UNLOCKED(muxObj) DPR_MUTEX_CREATE_UNLOCKED(muxObj)
#define L2ETH_MUTEX_LOCK(muxObj) DPR_MUTEX_LOCK(muxObj)
#define L2ETH_MUTEX_UNLOCK(muxObj) DPR_MUTEX_UNLOCK(muxObj)
#define L2ETH_MUTEX_DESTROY(muxObj) DPR_MUTEX_DESTROY(muxObj)

typedef DPR_SEMAPHORE L2ETH_SEMAPHORE;
#define L2ETH_SEM_CREATE(semObj) DPR_SEM_CREATE(semObj)
#define L2ETH_SEM_WAIT_INTERRUPTIBLE(semObj) DPR_SEM_WAIT_INTERRUPTIBLE(semObj)
#define L2ETH_SEM_WAIT(semObj) DPR_SEM_WAIT(semObj)
#define L2ETH_SEM_POST(semObj) DPR_SEM_POST(semObj)
#define L2ETH_SEM_DESTROY(semObj) DPR_SEM_DESTROY(semObj)

typedef DPR_THREAD_HANDLE L2ETH_THREAD_HANDLE;
#define L2ETH_CALLBACK_DECL DPR_THREAD_DECL
#define L2ETH_THREAD_CREATE DPR_THREAD_CREATE
#define L2ETH_THREAD_JOIN(tHndl) DPR_THREAD_JOIN(tHndl)
#define L2ETH_THREAD_RETURN_TYPE DPR_THREAD_RETURN_TYPE
#define L2ETH_THREAD_END DPR_THREAD_END

#define L2ETH_VMALLOC DPR_VMALLOC
#define L2ETH_VFREE DPR_VFREE
#define L2ETH_ZALLOC DPR_ZALLOC
#define L2ETH_FREE DPR_FREE

#define L2ETH_DBG_ASSERT DPR_ASSERT

#endif /* #ifndef L2ETH_LIB_CFG_H */
