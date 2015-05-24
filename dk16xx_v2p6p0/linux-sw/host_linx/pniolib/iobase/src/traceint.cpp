/****************************************************************************
*  Copyright (C) SIEMENS CORP., 2013 All rights reserved.
*****************************************************************************
* FILE NAME    : traceint.cpp
* ---------------------------------------------------------------------------
* DESCRIPTION  : pnio specific trace implementation
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

#include "os.h"

#include "pniousrx.h"
#include "pniousrd.h"
#include "traceint.h"

#if defined(PC_TRACE_ON) || defined(PRN_TRACE_ON) || defined(WDB_TRACE_ON)

void trcint_ShowAddr(OSTREAM &os, const PNIO_ADDR * add)
{
    if(!add) {
        os << "NULL";
        return;
    }

    os << " IODataType=" << (add->IODataType == PNIO_IO_IN ? "IO_IN" : "IO_OUT");
    os << ", AddrType=";

    if(add->AddrType == PNIO_ADDR_LOG) {
        os << "ADDR_LOG";
        os << ", Addr=" << add->u.Addr;
    }
    /*
     * else if(add->AddrType == PNIO_ADDR_GEO) { os<<"ADDR_GEO"; os<<",
     * addr.GeoAddr.SubnetID="<< add->GeoAddr.SubnetID; os<<",
     * addr.GeoAddr.StatNo="<< add->GeoAddr.StatNo; os<<",
     * addr.GeoAddr.Slot="<< add->GeoAddr.Slot; os<<",
     * addr.GeoAddr.Subslot="<< add->GeoAddr.Subslot; os<<",
     * addr.GeoAddr.Index="<< add->GeoAddr.Index; }
     */
    else {
        os << " (unknown address type !)";
    }
}

void trcint_ShowDAddr(OSTREAM &os, const PNIO_DEV_ADDR *add)
{
    if(!add) {
        os << "NULL";
        return;
    }

    os << " IODataType=" << (add->IODataType == PNIO_IO_IN ? "IO_IN" : "IO_OUT");
    os << ", AddrType=";

    if(add->AddrType == PNIO_ADDR_LOG) {
        os << "ADDR_LOG";
        os << ", Addr=" << add->u.reserved;
    } else if(add->AddrType == PNIO_ADDR_GEO) {
        os << "ADDR_GEO";
        os << ", addr.GeoAddr.Slot=" << add->u.Geo.Slot;
        os << ", addr.GeoAddr.Subslot=" << add->u.Geo.Subslot;
    } else {
        os << " (unknown address type !)";
    }
}

void trcint_ShowData(OSTREAM &os, PNIO_UINT32 Len, const PNIO_UINT8 * Buf)
{
    if(!Buf) {
        os << "NULL";
        return;
    }

    for(PNIO_UINT32 i = 0; i < Len; i++)
        os << (short)Buf[i] << ",";
}

void trcint_ShowDerror(OSTREAM &os, const PNIO_ERR_STAT * err)
{
    if(!err) {
        os << "NULL";
        return;
    }

    os << showbase << hex;
    os << " Err.ErrCode=" << (short)err->ErrCode;
    os << ", Err.ErrDecode=" << (short)err->ErrDecode;
    os << ", Err.ErrCode1=" << (short)err->ErrCode1;
    os << ", Err.ErrCode2=" << (short)err->ErrCode2;
    os << ", Err.AddValue1=" << (short)err->AddValue1;
    os << ", Err.AddValue2=" << (short)err->AddValue2;

}

void trcint_ShowDiagReq(OSTREAM &os, const PNIO_CTRL_DIAG * diag)
{
    if(!diag) {
        os << "NULL";
        return;
    }

    os << " DiagService=" << diag->DiagService;
    os << ", ReqRef=" << diag->ReqRef;
}

#endif /* PC_TRACE_ON || PRN_TRACE_ON */
