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

#ifndef _TRACE_MCR_H_
#define _TRACE_MCR_H_


#if defined(PC_TRACE_ON) || defined(PRN_TRACE_ON) || defined(WDB_TRACE_ON)

#include <sstream>

using namespace std;

#include "traceglobal.h"
#include "traceout.h"
#include "tracesub.h"

#define LV_ERR      TRACEDEPTH_ERRORS
#define LV_WARN     TRACEDEPTH_WARNINGS
#define LV_INFO     TRACEDEPTH_INFO
#define LV_FCTPUB1  TRACEDEPTH_LEVEL1   /* module public interface calls */
#define LV_FCTPUB2  TRACEDEPTH_LEVEL2   /* module public interface calls with parameters */
#define LV_FCTCLBF  TRACEDEPTH_LEVEL3   /* callback calls with parameters */
#define LV_DATA     TRACEDEPTH_LEVEL4   /* buffers and message data */
#define LV_FCTINT   TRACEDEPTH_LEVEL5   /* important internal function calls */
#define LV_TIMECRIT TRACEDEPTH_LEVEL6   /* TIME CRITICAL CALLS */
#define LV_CONDES   0xff                /* constructort / destructors */

#define OSTREAM ostringstream

#else  /* _TRC_ON */

#define LV_ERR      0
#define LV_WARN     0
#define LV_INFO     0
#define LV_FCTPUB1  0
#define LV_FCTPUB2  0
#define LV_FCTCLBF  0
#define LV_DATA     0
#define LV_FCTINT   0
#define LV_TIMECRIT 0
#define LV_CONDES   0

#define TRC_INIT()              /* empty */
#define TRC_INIT_FROM_FILE(ConfigFilePath)  /* empty */
#define TRC_DEINIT()            /* empty */
#define TRC_IF_ON(Group, Depth)              false

#define TRC_IF_ON_EXPR(Group, Depth, _EXPR_)    /* empty */
#define TRC_OUT(Group, Depth, Msg)      /* empty */
#define TRC_OUT_OBJECT(Group, Depth, Obj)       /* empty */
#define TRC_OUTD(Group, Depth, Data, Len)       /* empty */
#define TRC_OUT01(Group, Depth, Msg, p1)        /* empty */
#define TRC_OUT02(Group, Depth, Msg, p1, p2)    /* empty */
#define TRC_OUT03(Group, Depth, Msg, p1, p2, p3) /* empty */
#define TRC_OUT04(Group, Depth, Msg, p1, p2, p3, p4) /* empty */
#define TRC_OUT05(Group, Depth, Msg, p1, p2, p3, p4, p5) /* empty */

#endif /* _TRACE_ON */

/* PC_TRACE_ON: Full host trace. Configurable by the config file: pniotrace.conf
 *              Output gos into the trace file specified in the config file. See StaticTraceInit().
 */
#if defined(PC_TRACE_ON)

class CTrcWrapper : protected CTraceOut {
public:
        static CTrcWrapper* Instance();
        static void DestroyInstance();

protected:
        CTrcWrapper();

public:
    unsigned long trc_init_from_file(const char * configFile);
    void trc_deinit(void);
    void trc_out(unsigned long Group, const char *GrName, unsigned long Depth, const char *Msg);
    void trc_outf(const char *szFormat, ...);
    void trc_outf_(unsigned long Group, const char *GrName, unsigned long Depth,
        const char *szFormat, ...);
    void trc_outd(unsigned long Group, unsigned long Depth, unsigned char *Data, long Len);

    unsigned long dest;
    unsigned long group;
    unsigned long depth;

private:
        static CTrcWrapper* _instance;
};

  /*
   * #define TRC_INIT(RegPath) CTrcWrapper::Instance()->trc_init(RegPath);
   */

#define TRC_INIT_FROM_FILE(ConfigFilePath)      CTrcWrapper::Instance()->trc_init_from_file(ConfigFilePath);
#define TRC_DEINIT()                            { CTrcWrapper::Instance()->trc_deinit(); CTrcWrapper::DestroyInstance(); }
#define TRC_IF_ON(Group, Depth)                                 ((CTrcWrapper::Instance()->dest) && \
                                                                                                 ((Group) & (CTrcWrapper::Instance()->group)) && \
                                                                                                 ((Depth) <= (CTrcWrapper::Instance()->depth)))

#define TRC_IF_ON_EXPR(Group, Depth, _EXPR_)    if(TRC_IF_ON(Group, Depth)) { _EXPR_ }
#define TRC_OUT(Group, Depth, Msg)              if(TRC_IF_ON(Group, Depth)) { CTrcWrapper::Instance()->trc_out((Group), Group##_NAME, (Depth), (Msg));}
#define TRC_OUT_OBJECT(Group, Depth, Obj)       if(TRC_IF_ON(Group, Depth)) { CTrcWrapper::Instance()->trc_out((Group), Group##_NAME, (Depth), (Obj).str().data()); }

#define TRC_OUTD(Group, Depth, Data, Len)       if(TRC_IF_ON(Group, Depth)) { CTrcWrapper::Instance()->trc_outd((Group), (Depth), (Data), (Len));}

#define TRC_OUT01(Group, Depth, Msg, p1)        if(TRC_IF_ON(Group, Depth)) { CTrcWrapper::Instance()->trc_outf_((Group), Group##_NAME, (Depth), (Msg), (p1));}
#define TRC_OUT02(Group, Depth, Msg, p1, p2)    if(TRC_IF_ON(Group, Depth)) { CTrcWrapper::Instance()->trc_outf_((Group), Group##_NAME, (Depth), (Msg), (p1), (p2));}
#define TRC_OUT03(Group, Depth, Msg, p1, p2, p3)    if(TRC_IF_ON(Group, Depth)) { CTrcWrapper::Instance()->trc_outf_((Group), Group##_NAME, (Depth), (Msg), (p1), (p2), (p3));}
#define TRC_OUT04(Group, Depth, Msg, p1, p2, p3, p4)    if(TRC_IF_ON(Group, Depth)) { CTrcWrapper::Instance()->trc_outf_((Group), Group##_NAME, (Depth), (Msg), (p1), (p2), (p3), (p4));}
#define TRC_OUT05(Group, Depth, Msg, p1, p2, p3, p4, p5)    if(TRC_IF_ON(Group, Depth)) { CTrcWrapper::Instance()->trc_outf_((Group), Group##_NAME, (Depth), (Msg), (p1), (p2), (p3), (p4), (p5));}

//#endif /* PC_TRACE_ON */


/* PRN_TRACE_ON: Simple trace to console for fast integration. Output is accomplished by 'printf'
 *
 */
#elif defined(PRN_TRACE_ON)

/* Trace Control Variables: (global variables defined in fct_common.cpp) See TRC_INIT_FROM_FILE below.
 */
extern unsigned long g_pnio_trc_dest;    /* trace output destination. 0: no output, 1: stdout printf */
extern unsigned long g_pnio_trc_group;
extern unsigned long g_pnio_trc_depth;

/* TRC_INIT_FROM_FILE: Macro to init 'Trace Control Variables'. (called in StaticTraceInit Ctor)
 * For simple trace we don't use file initialisation - we have to set the 'Trace Control Variables'
 * here. The macro only sets the 'Trace Control Variables'.
 * Set init values to configure the trace as you need.
 */
#define TRC_INIT_FROM_FILE(ConfigFilePath)  { \
            g_pnio_trc_dest = 1; \
            g_pnio_trc_group= (GR_INIT | GR_PE); \
                                              g_pnio_trc_depth= LV_FCTINT; \
                                            }

//#define TRC_IF_ON(Group, Depth)   true /* true. trace is always on, false: trace is off */
#define TRC_IF_ON(Group, Depth) ((g_pnio_trc_dest) && ((Group) & (g_pnio_trc_group)) && ((Depth) <= (g_pnio_trc_depth)))
#define TRC_IF_ON_EXPR(Group, Depth, _EXPR_)  if(TRC_IF_ON(Group, Depth)) { _EXPR_ }
#define TRC_OUT_OBJECT(Group, Depth, Obj)     if(TRC_IF_ON(Group, Depth)) { printf("PNIO: %s %s \n", Group##_NAME, (Obj).str().data() ); }

#define TRC_INIT()                          /* not used empty */
#define TRC_DEINIT()                        /* not used empty */
#define TRC_OUTD(Group, Depth, Data, Len)   /* not used empty */

#define TRC_OUT(Group, Depth, Msg)                        if(TRC_IF_ON(Group, Depth)) { printf("PNIO: " Group##_NAME " %s \n", Msg); }
#define TRC_OUT01(Group, Depth, Msg, p1)                  if(TRC_IF_ON(Group, Depth)) { printf("PNIO: " Group##_NAME " " Msg "\n", p1); }
#define TRC_OUT02(Group, Depth, Msg, p1, p2)              if(TRC_IF_ON(Group, Depth)) { printf("PNIO: " Group##_NAME " " Msg "\n", p1, p2); }
#define TRC_OUT03(Group, Depth, Msg, p1, p2, p3)          if(TRC_IF_ON(Group, Depth)) { printf("PNIO: " Group##_NAME " " Msg "\n", p1, p2, p3); }
#define TRC_OUT04(Group, Depth, Msg, p1, p2, p3, p4)      if(TRC_IF_ON(Group, Depth)) { printf("PNIO: " Group##_NAME " " Msg "\n", p1, p2, p3, p4); }
#define TRC_OUT05(Group, Depth, Msg, p1, p2, p3, p4, p5)  if(TRC_IF_ON(Group, Depth)) { printf("PNIO: " Group##_NAME " " Msg "\n", p1, p2, p3, p4, p5); }

/* WDB_TRACE_ON: Windows only. Simple trace by OutputDebugString()
 */
#elif defined(WDB_TRACE_ON)

#ifdef __cplusplus
extern "C"
{
#endif

void __cdecl dbg_msg_out(char *szFormat, ... );


#ifdef __cplusplus
}
#endif

/* global control variables */
extern unsigned long g_pnio_trc_dest;    /* trace output destination. 0: no output, 1: stdout printf */
extern unsigned long g_pnio_trc_group;
extern unsigned long g_pnio_trc_depth;




/* init global vars, is called in fct_common.cpp, in StaticTraceInit */
#define TRC_INIT_FROM_FILE(ConfigFilePath)  { g_pnio_trc_dest = 1; \
                                              g_pnio_trc_group= (GR_INIT | GR_IOR); \
                                              g_pnio_trc_depth= LV_FCTINT; \
                                            }

//#define TRC_IF_ON(Group, Depth)   true /* true. trace is always on, false: trace is off */
#define TRC_IF_ON(Group, Depth) ((g_pnio_trc_dest) && ((Group) & (g_pnio_trc_group)) && ((Depth) <= (g_pnio_trc_depth)))
#define TRC_IF_ON_EXPR(Group, Depth, _EXPR_)  if(TRC_IF_ON(Group, Depth)) { _EXPR_ }
#define TRC_OUT_OBJECT(Group, Depth, Obj)     if(TRC_IF_ON(Group, Depth)) { dbg_msg_out("PNIO: %s %s \n", Group##_NAME, (Obj).str().data() ); }

#define TRC_INIT()                          /* not used empty */
#define TRC_DEINIT()                        /* not used empty */
#define TRC_OUTD(Group, Depth, Data, Len)   /* not used empty */

#define TRC_OUT(Group, Depth, Msg)                        if(TRC_IF_ON(Group, Depth)) { dbg_msg_out("%s \n", Msg); }
#define TRC_OUT01(Group, Depth, Msg, p1)                  if(TRC_IF_ON(Group, Depth)) { dbg_msg_out("PNIO: " Group##_NAME " " Msg "\n", p1); }
#define TRC_OUT02(Group, Depth, Msg, p1, p2)              if(TRC_IF_ON(Group, Depth)) { dbg_msg_out("PNIO: " Group##_NAME " " Msg "\n", p1, p2); }
#define TRC_OUT03(Group, Depth, Msg, p1, p2, p3)          if(TRC_IF_ON(Group, Depth)) { dbg_msg_out("PNIO: " Group##_NAME " " Msg "\n", p1, p2, p3); }
#define TRC_OUT04(Group, Depth, Msg, p1, p2, p3, p4)      if(TRC_IF_ON(Group, Depth)) { dbg_msg_out("PNIO: " Group##_NAME " " Msg "\n", p1, p2, p3, p4); }
#define TRC_OUT05(Group, Depth, Msg, p1, p2, p3, p4, p5)  if(TRC_IF_ON(Group, Depth)) { dbg_msg_out("PNIO: " Group##_NAME " " Msg "\n", p1, p2, p3, p4, p5); }

#endif  /* WDB_TRACE_ON */

#endif /* _TRACE_MCR_H_ */

