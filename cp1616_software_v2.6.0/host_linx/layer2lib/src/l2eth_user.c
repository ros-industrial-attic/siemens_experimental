/*****************************************************************************/
/*  Copyright (C) SIEMENS CORP., 2013 All rights reserved.*/
/*****************************************************************************/
/*  F i l e                l2eth_user.c                                      */
/*****************************************************************************/
/*  This module contains the common dual port RAM functions. There are       */
/*  no os dependent functions.                                               */
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

#include "os.h"
#include "cp16xx.h"

#include "l2eth_config.h"
#include "l2eth_errs.h"
#include "l2eth_user.h"
#include "l2eth_rqb.h"
#include "l2eth_base.h"

#ifndef __KERNEL__
#include "tracemcr.h"
#endif

/* exported functions */
#ifdef MANAGED_CP_EXCHANGE
extern "C" L2ETH_UINT32 L2ETH_CODE_ATTR ldah_check(L2ETH_UINT32 CpIndex);
#endif

/*===========================================================================
*    critical section
*==========================================================================*/
#ifdef __KERNEL__
    #define L2_MUTEX_CREATE_UNLOCKED
    #define L2_MUTEX_LOCK
    #define L2_MUTEX_UNLOCK
    #define L2_MUTEX_DESTROY
#else
    L2ETH_MUTEX L2MutexUser;
    #define L2_MUTEX_CREATE_UNLOCKED L2ETH_MUTEX_CREATE_UNLOCKED(L2MutexUser)
    #define L2_MUTEX_LOCK L2ETH_MUTEX_LOCK(L2MutexUser)
    #define L2_MUTEX_UNLOCK L2ETH_MUTEX_UNLOCK(L2MutexUser)
    #define L2_MUTEX_DESTROY L2ETH_MUTEX_DESTROY(L2MutexUser)
#endif

void InitL2CriticalSections(void)
{
    L2_MUTEX_CREATE_UNLOCKED;
}

void DestroyL2CriticalSections(void)
{
    L2_MUTEX_DESTROY;
}
/*===========================================================================
*    l2eth_open
*==========================================================================*/
L2ETH_UINT32 l2eth_open(L2ETH_UINT32 CpIndex,   /* in */
    L2ETH_CBF_RECEIVE_IND CbfReceiveInd,        /* in */
    L2ETH_CBF_SEND_COMPL CbfSendCompl,  /* in */
    L2ETH_CBF_STATUS_IND CbfStatusInd,  /* in */
    L2ETH_CBF_MODE_COMPL CbfModeCompl,  /* in */
    L2ETH_UINT32 * pHandle /* out: */ )
{
    L2ETH_UINT32 Ret = L2ETH_OK;

    L2_MUTEX_LOCK;

    TRC_OUT01(GR_INIT, LV_FCTPUB1, "->l2eth_open CpIndex %d", CpIndex);

    /* validate cp id */
    if(1 > CpIndex) {
        TRC_OUT(GR_INIT, LV_ERR, " invalid cpid parameter");
        TRC_OUT01(GR_INIT, LV_FCTPUB1, "<- l2eth_open CpIndex %d", CpIndex);
        L2_MUTEX_UNLOCK;
        return L2ETH_ERR_PRM_CP_ID;
    }

    /* call back functions - check for null */
    if(!CbfReceiveInd || !CbfSendCompl || !CbfStatusInd || !CbfModeCompl) {
        TRC_OUT(GR_INIT, LV_ERR, " invalid callback function");
        TRC_OUT01(GR_INIT, LV_FCTPUB1, "<- l2eth_open CpIndex %d", CpIndex);
        L2_MUTEX_UNLOCK;
        return L2ETH_ERR_PRM_PCBF;
    }

#ifdef MANAGED_CP_EXCHANGE
    (void)ldah_check(CpIndex);
#endif

    /* call library open function */
    Ret = l2_open(CpIndex, CbfReceiveInd, CbfSendCompl, CbfStatusInd, CbfModeCompl, pHandle);

    L2_MUTEX_UNLOCK;
    TRC_OUT01(GR_INIT, LV_FCTPUB1, "<- l2eth_open CpIndex %d", CpIndex);

    return Ret;
}

/*===========================================================================
*    l2eth_close
*==========================================================================*/
L2ETH_UINT32 l2eth_close(L2ETH_UINT32 Handle)
{
    l2eth_base *pl2BaseInst;
    L2ETH_UINT32 Ret = L2ETH_OK;

    L2_MUTEX_LOCK;

    TRC_OUT(GR_INIT, LV_FCTPUB1, "->l2eth_close");

    /* get application instance from pointer */
    pl2BaseInst = l2_get_instance(Handle);

    if(pl2BaseInst) {
        /* call library close function */
        Ret = l2_close(pl2BaseInst);
        if(Ret != L2ETH_OK) {
            TRC_OUT(GR_INIT, LV_ERR, "  * l2eth_base Close failed ");
        }
    } else {
        TRC_OUT(GR_INIT, LV_ERR, "  * ERROR getting l2eth_base instance ");
        Ret = L2ETH_ERR_PRM_HND;
    }

    L2_MUTEX_UNLOCK;
    TRC_OUT(GR_INIT, LV_FCTPUB1, "<- l2eth_close");

    return Ret;
}

/*===========================================================================
*    l2eth_allocate_packet
*==========================================================================*/
L2ETH_UINT32 l2eth_allocate_packet(L2ETH_UINT32 Handle, /* in */
    L2ETH_PACKET ** ppPacket) /* out */
{
    l2eth_base *pl2BaseInst;
    L2ETH_UINT32 Ret;

    TRC_OUT(GR_INIT, LV_FCTPUB1, "-> l2eth_allocate_packet");

    if(!ppPacket) {
        TRC_OUT(GR_INIT, LV_ERR, "invalid packet parameter");
        TRC_OUT(GR_INIT, LV_FCTPUB1, "<- l2eth_allocate_packet");

        return L2ETH_ERR_PRM_PKT;
    }

    /* get application instance from pointer */
    pl2BaseInst = l2_get_instance(Handle);

    if(pl2BaseInst) {
        /* call library allocate packet function */
#ifdef __KERNEL__
        L2_KERNEL_SPINLOCK_FLAGS flags;
        L2_KERNEL_WSPINLOCK_LOCK(Handle, flags);
        Ret = l2_get_allocate_packet(pl2BaseInst, ppPacket);
        L2_KERNEL_WSPINLOCK_UNLOCK(Handle, flags);
#else
        L2_MUTEX_LOCK;
        Ret = l2_get_allocate_packet(pl2BaseInst, ppPacket);
        L2_MUTEX_UNLOCK;
#endif
        if(Ret != L2ETH_OK) {
            TRC_OUT(GR_INIT, LV_ERR, "  * l2eth_allocate_packet failed ");
        }
    } else {
        TRC_OUT(GR_INIT, LV_ERR, "  * ERROR getting l2eth_base instance ");
        Ret = L2ETH_ERR_PRM_HND;
    }

    TRC_OUT(GR_INIT, LV_FCTPUB1, "<- l2eth_allocate_packet");

    return Ret;
}

/*===========================================================================
*    l2eth_send
*==========================================================================*/
L2ETH_UINT32 l2eth_send(L2ETH_UINT32 Handle, L2ETH_PACKET * pPacket)
{
    l2eth_base *pl2BaseInst;
    L2ETH_UINT32 Ret;
    /*
       locking is not required
       L2_MUTEX_LOCK;
     */

    //TRC_OUT(GR_INIT, LV_FCTPUB1, "->l2eth_send");

    if(!pPacket) {
        TRC_OUT(GR_INIT, LV_ERR, " invalid packet ");
        TRC_OUT(GR_INIT, LV_FCTPUB1, "<- l2eth_send");
        /* L2_MUTEX_UNLOCK; */
        return L2ETH_ERR_PRM_PKT;
    }

    if(!pPacket->pBuffer) {
        TRC_OUT(GR_INIT, LV_ERR, " invalid packet buffer ");
        TRC_OUT(GR_INIT, LV_FCTPUB1, "<- l2eth_send");
        /* L2_MUTEX_UNLOCK; */
        return L2ETH_ERR_PRM_BUF;
    }


    /* get application instance from pointer */
    pl2BaseInst = l2_get_instance(Handle);

    if(pl2BaseInst) {
        /* call library send packet function */
        Ret = l2_send_packet(pl2BaseInst, pPacket);
        if(Ret != L2ETH_OK) {
            TRC_OUT(GR_INIT, LV_ERR, "  send failed ");
        }
    } else {
        TRC_OUT(GR_INIT, LV_ERR, "  * ERROR getting l2eth_base instance ");
        Ret = L2ETH_ERR_PRM_HND;
    }

    /*
       locking is not required
       L2_MUTEX_UNLOCK;
     */
    //TRC_OUT(GR_INIT, LV_FCTPUB1, "<- l2eth_send");
    return Ret;
}

/*===========================================================================
*    l2eth_free_packet
*==========================================================================*/
L2ETH_UINT32 l2eth_free_packet(L2ETH_UINT32 Handle, L2ETH_PACKET * pPacket)
{
    l2eth_base *pl2BaseInst;
    L2ETH_UINT32 Ret;

    //TRC_OUT(GR_INIT, LV_FCTPUB1, "-> l2eth_free_packet");

    if(!pPacket) {
        TRC_OUT(GR_INIT, LV_ERR, "packet not set");
        TRC_OUT(GR_INIT, LV_FCTPUB1, "<- l2eth_free_packet");

        return L2ETH_ERR_PRM_PKT;
    }

    if(!pPacket->pBuffer) {
        TRC_OUT(GR_INIT, LV_ERR, " invalid packet buffer ");
        TRC_OUT(GR_INIT, LV_FCTPUB1, "<- l2eth_free_packet");

        return L2ETH_ERR_PRM_BUF;
    }

    /* get application instance from pointer */
    pl2BaseInst = l2_get_instance(Handle);

    if(pl2BaseInst) {
        /* call library free packet function */
#ifdef __KERNEL__
        L2_KERNEL_SPINLOCK_FLAGS flags;
        L2_KERNEL_WSPINLOCK_LOCK(Handle, flags);
        Ret = l2_free_packet(pl2BaseInst, pPacket);
        L2_KERNEL_WSPINLOCK_UNLOCK(Handle, flags);
#else
        L2_MUTEX_LOCK;
        Ret = l2_free_packet(pl2BaseInst, pPacket);
        L2_MUTEX_UNLOCK;
#endif
        if(Ret != L2ETH_OK) {
            TRC_OUT(GR_INIT, LV_ERR, "  *free packet failed ");
        }
    } else {
        TRC_OUT(GR_INIT, LV_ERR, "  * ERROR getting l2eth_base instance ");
        Ret = L2ETH_ERR_PRM_HND;
    }

    //TRC_OUT(GR_INIT, LV_FCTPUB1, "<- l2eth_free_packet");
    return Ret;
}


/*===========================================================================
*    l2eth_return_packet
*==========================================================================*/
L2ETH_UINT32 l2eth_return_packet(L2ETH_UINT32 Handle, L2ETH_PACKET * pPacket)
{
    l2eth_base *pl2BaseInst;
    L2ETH_UINT32 Ret;

    TRC_OUT(GR_INIT, LV_FCTPUB1, "-> l2eth_return_packet");

    if(!pPacket) {
        TRC_OUT(GR_INIT, LV_ERR, "packet not set");
        TRC_OUT(GR_INIT, LV_FCTPUB1, "<- l2eth_return_packet");

        return L2ETH_ERR_PRM_PKT;
    }

    if(!pPacket->pBuffer) {
        TRC_OUT(GR_INIT, LV_ERR, " invalid packet buffer ");
        TRC_OUT(GR_INIT, LV_FCTPUB1, "<- l2eth_return_packet");

        return L2ETH_ERR_PRM_BUF;
    }

    /* get application instance from pointer */
    pl2BaseInst = l2_get_instance(Handle);

    if(pl2BaseInst) {
        /* call library return packet function */
#ifdef __KERNEL__
        L2_KERNEL_SPINLOCK_FLAGS flags;
        L2_KERNEL_RSPINLOCK_LOCK(Handle, flags);
        Ret = l2_return_packet(pl2BaseInst, pPacket, L2ETH_PACKET_OWNER_USER);
        L2_KERNEL_RSPINLOCK_UNLOCK(Handle, flags);
#else
        L2_MUTEX_LOCK;
        Ret = l2_return_packet(pl2BaseInst, pPacket, L2ETH_PACKET_OWNER_USER);
        L2_MUTEX_UNLOCK;
#endif

        if(Ret != L2ETH_OK) {
            TRC_OUT(GR_INIT, LV_ERR, "  *return_packet failed ");
        }
    } else {
        TRC_OUT(GR_INIT, LV_ERR, "  * ERROR getting l2eth_base instance ");
        Ret = L2ETH_ERR_PRM_HND;
    }

    TRC_OUT(GR_INIT, LV_FCTPUB1, "<- l2eth_return_packet");
    return Ret;
}



/*===========================================================================
*    l2eth_set_mode
*==========================================================================*/
L2ETH_UINT32 l2eth_set_mode(L2ETH_UINT32 Handle, L2ETH_MODE Mode)
{
    l2eth_base *pl2BaseInst;
    L2ETH_UINT32 Ret;

    L2_MUTEX_LOCK;

    TRC_OUT(GR_INIT, LV_FCTPUB1, "-> l2eth_set_mode");

    if((L2ETH_ONLINE != Mode) && (L2ETH_OFFLINE != Mode)) {
        TRC_OUT(GR_INIT, LV_ERR, "invalid Mode");
        TRC_OUT(GR_INIT, LV_FCTPUB1, "<- l2eth_set_mode");

        L2_MUTEX_UNLOCK;
        return L2ETH_ERR_PRM_MODE;
    }

    /* get application instance from pointer */
    pl2BaseInst = l2_get_instance(Handle);

    if(pl2BaseInst) {
        Ret = l2_set_mode(pl2BaseInst, Mode);
        if(Ret != L2ETH_OK) {
            TRC_OUT(GR_INIT, LV_ERR, "BaseInst->setmode failed ");
        }
    } else {
        TRC_OUT(GR_INIT, LV_ERR, " ERROR getting l2eth_base instance ");
        Ret = L2ETH_ERR_PRM_HND;
    }

    L2_MUTEX_UNLOCK;
    TRC_OUT(GR_INIT, LV_FCTPUB1, "<- l2eth_set_mode");
    return Ret;
}


/*===========================================================================
*    l2eth_set_information
*==========================================================================*/
L2ETH_UINT32 l2eth_set_information(L2ETH_UINT32 Handle, /* in */
    L2ETH_QUERY * pQuery)
{                               /* in,out */
    l2eth_base *pl2BaseInst;
    L2ETH_UINT32 Ret;

    L2_MUTEX_LOCK;

    TRC_OUT(GR_INIT, LV_FCTPUB1, "-> l2eth_set_information");

    if(!pQuery) {
        TRC_OUT(GR_INIT, LV_ERR, "invalid pQuery parameter");
        TRC_OUT(GR_INIT, LV_FCTPUB1, "<- l2eth_set_information");

        L2_MUTEX_UNLOCK;

        /* return error */
        Ret = L2ETH_ERR_PRM_QUERY;
        TRC_OUT01(GR_INIT, LV_ERR, "NULL pQuery buffer = 0x%x", Ret);
        return Ret;
    }


    /* check query buffer is null or not */
    if(!pQuery->pBuffer) {
        TRC_OUT(GR_INIT, LV_ERR, "invalid pQuery parameter");
        TRC_OUT(GR_INIT, LV_FCTPUB1, "<- l2eth_set_information");

        L2_MUTEX_UNLOCK;

        /* return error */
        Ret = L2ETH_ERR_PRM_BUF;
        TRC_OUT01(GR_INIT, LV_ERR, "NULL pQuery buffer = 0x%x", Ret);
        return Ret;
    }

    /* get application instance from pointer */
    pl2BaseInst = l2_get_instance(Handle);

    if(pl2BaseInst) {
        /* call library set information function */
        Ret = l2_getset_information(pl2BaseInst, SET_INFO, pQuery);
        if(Ret != L2ETH_OK) {
            TRC_OUT(GR_INIT, LV_ERR, "BaseInst->set_information failed ");
        }
    } else {
        TRC_OUT(GR_INIT, LV_ERR, " ERROR getting l2eth_base instance ");
        Ret = L2ETH_ERR_PRM_HND;
    }

    L2_MUTEX_UNLOCK;
    TRC_OUT(GR_INIT, LV_FCTPUB1, "<- l2eth_set_information");

    return Ret;
}



/*===========================================================================
*    l2eth_get_information
*==========================================================================*/
L2ETH_UINT32 l2eth_get_information(L2ETH_UINT32 Handle, /* in */
    L2ETH_QUERY * pQuery)
{                               /* in,out */
    l2eth_base *pl2BaseInst;
    L2ETH_UINT32 Ret;

    L2_MUTEX_LOCK;

    TRC_OUT(GR_INIT, LV_FCTPUB1, "-> l2eth_get_information");

    if(!pQuery) {
        TRC_OUT(GR_INIT, LV_ERR, "invalid query parameter");
        TRC_OUT(GR_INIT, LV_FCTPUB1, "<- l2eth_get_information");

        L2_MUTEX_UNLOCK;

        /* return error */
        Ret = L2ETH_ERR_PRM_QUERY;
        TRC_OUT01(GR_INIT, LV_ERR, "NULL pQuery = 0x%x", Ret);
        return Ret;
    }

    /* check query buffer is null or not */
    if(!pQuery->pBuffer) {
        L2_MUTEX_UNLOCK;

        /* return error */
        Ret = L2ETH_ERR_PRM_BUF;
        TRC_OUT01(GR_INIT, LV_ERR, "NULL pQuery buffer = 0x%x", Ret);
        return Ret;
    }

    /* get application instance from pointer */
    pl2BaseInst = l2_get_instance(Handle);

    if(pl2BaseInst) {
        /* call library get information function */
        Ret = l2_getset_information(pl2BaseInst, GET_INFO, pQuery);
        if(Ret != L2ETH_OK) {
            TRC_OUT(GR_INIT, LV_ERR, "BaseInst->get_information failed ");
        }
    } else {
        TRC_OUT(GR_INIT, LV_ERR, " ERROR getting l2eth_base instance ");
        Ret = L2ETH_ERR_PRM_HND;
    }

    L2_MUTEX_UNLOCK;
    TRC_OUT(GR_INIT, LV_FCTPUB1, "<- l2eth_get_information");

    return Ret;
}
