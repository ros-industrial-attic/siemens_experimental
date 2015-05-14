/*****************************************************************************/
/* Copyright (C) SIEMENS CORP., 2013 All rights reserved.*/
/*****************************************************************************/
/*                                                                           */
/* Diese Software ist Freeware. Sie wird Ihnen unentgeltlich zur Verfuegung  */
/* gestellt. Sie darf frei kopiert, modifiziert und benutzt sowie an Dritte  */
/* weitergegeben werden. Die Software darf nur unter Beibehaltung aller      */
/* Schutzrechtsvermerke sowie nur vollstaedig und unveraedert weitergegeben  */
/* werden. Die kommerzielle Weitergabe an Dritte (z.B. im Rahmen von         */
/* Share-/Freeware-Distributionen) ist nur mit vorheriger schriftlicher      */
/* Genehmigung der Siemens Aktiengesellschaft erlaubt.                       */
/* DA DIE SOFTWARE IHNEN UNENTGELTLICH UBERLASSEN WIRD, KOENNEN DIE AUTOREN  */
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

#ifndef _TRACE_MCRM_H_
#define _TRACE_MCRM_H_

#ifdef PC_TRACE_ON

#include "tracemcr.h"

/* use CTrcWrapper but not as singelton */
/* to provide instance oriented tracing */
class CTrmWrapper : public CTrcWrapper {
public:
  CTrmWrapper() : CTrcWrapper(){}
  ~CTrmWrapper(){}
};

#define TRM_INIT_FROM_FILE(pI, ConfigFilePath) (pI)->trc_init_from_file(ConfigFilePath);
#define TRM_DEINIT(pI)                         (pI)->trc_deinit(); 
#define TRM_IF_ON(pI, Group, Depth)            (((pI)->dest) && \
                                                     ((Group) & ((pI)->group)) && \
                                                     ((Depth) <= ((pI)->depth)))

#define TRM_IF_ON_EXPR(pI, Group, Depth, _EXPR_)    if(TRM_IF_ON((pI), Group, Depth)) { _EXPR_ }
#define TRM_OUT_OBJECT(pI, Group, Depth, Obj)       if(TRM_IF_ON((pI), Group, Depth)) { (pI)->trc_out((Group), Group##_NAME, (Depth), (Obj).str().data()); }

#define TRM_OUTD(pI, Group, Depth, Data, Len)       if(TRM_IF_ON(pI, Group, Depth)) { (pI)->trc_outd((Group), (Depth), (Data), (Len));}

#define TRM_OUT00(pI, Group, Depth, Msg)            if(TRM_IF_ON(pI, Group, Depth)) { (pI)->trc_outf_((Group), Group##_NAME, (Depth), (Msg));}
#define TRM_OUT01(pI, Group, Depth, Msg, p1)        if(TRM_IF_ON(pI, Group, Depth)) { (pI)->trc_outf_((Group), Group##_NAME, (Depth), (Msg), (p1));}
#define TRM_OUT02(pI, Group, Depth, Msg, p1, p2)    if(TRM_IF_ON(pI, Group, Depth)) { (pI)->trc_outf_((Group), Group##_NAME, (Depth), (Msg), (p1), (p2));}
#define TRM_OUT03(pI, Group, Depth, Msg, p1, p2, p3)    if(TRM_IF_ON(pI, Group, Depth)) { (pI)->trc_outf_((Group), Group##_NAME, (Depth), (Msg), (p1), (p2), (p3));}
#define TRM_OUT04(pI, Group, Depth, Msg, p1, p2, p3, p4)    if(TRM_IF_ON(pI, Group, Depth)) { (pI)->trc_outf_((Group), Group##_NAME, (Depth), (Msg), (p1), (p2), (p3), (p4));}
#define TRM_OUT05(pI, Group, Depth, Msg, p1, p2, p3, p4, p5)    if(TRM_IF_ON(pI, Group, Depth)) { (pI)->trc_outf_((Group), Group##_NAME, (Depth), (Msg), (p1), (p2), (p3), (p4), (p5));}

#else /* PC_TRACE_ON */

#define TRM_INIT_FROM_FILE(pI, ConfigFilePath)  /* empty */
#define TRM_DEINIT(pI)                          /* empty */
#define TRM_IF_ON(pI, Group, Depth)              false

#define TRM_IF_ON_EXPR(pI, Group, Depth, _EXPR_)    /* empty */
#define TRM_OUT(Group, Depth, Msg)      /* empty */
#define TRM_OUT_OBJECT(pI, Group, Depth, Obj)       /* empty */
#define TRM_OUTD(pI, Group, Depth, Data, Len)       /* empty */
#define TRM_OUT00(pI, Group, Depth, Msg)            
#define TRM_OUT01(pI, Group, Depth, Msg, p1)        /* empty */
#define TRM_OUT02(pI, Group, Depth, Msg, p1, p2)    /* empty */
#define TRM_OUT03(pI, Group, Depth, Msg, p1, p2, p3) /* empty */
#define TRM_OUT04(pI, Group, Depth, Msg, p1, p2, p3, p4) /* empty */
#define TRM_OUT05(pI, Group, Depth, Msg, p1, p2, p3, p4, p5) /* empty */
#endif /* PC_TRACE_ON */

#endif /* _TRACE_MCR_H_ */
