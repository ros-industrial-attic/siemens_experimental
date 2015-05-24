/****************************************************************************
*  Copyright (C) SIEMENS CORP., 2013 All rights reserved.
*****************************************************************************
* FILE NAME    : tracemcr.cpp
* ---------------------------------------------------------------------------
* DESCRIPTION  : trace module.
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
#define NOTRC
#include "traceout.h"
#include "tracemcr.h"

#ifdef PC_TRACE_ON

CTrcWrapper *CTrcWrapper::_instance = 0;

CTrcWrapper::CTrcWrapper() : CTraceOut()
{
}

CTrcWrapper *CTrcWrapper::Instance()
{
	if(_instance == 0)
		_instance = new CTrcWrapper;
	return _instance;
}

void CTrcWrapper::DestroyInstance()
{
	delete _instance;
	_instance = 0;
}

unsigned long CTrcWrapper::trc_init_from_file(const char * configFile)
{
    long hr = InitFromFile(configFile);

    GetDepth(depth);
    GetGroup(group);
    GetDest(dest);

    trc_outf("Trace init ret=0x%x, depth=0x%lx, group=0x%x", hr, depth, group);

    return hr;
}

void CTrcWrapper::trc_deinit(void)
{
    trc_out(GR_INIT, GET_GR_SNAME(GR_INIT), LV_FCTINT,
        "deinitialise the global trace object");
}

void CTrcWrapper::trc_out(unsigned long Group,
    const char *GrName, unsigned long Depth,
    const char *Msg)
{
    Lock();
    if(dest & 0x00000002)
        printf("D:x%02x %s %s\n", (unsigned int)Depth, GrName, Msg);

    char szBuffer[256];

    snprintf(szBuffer, sizeof (szBuffer), "%s %s%s", GrName,
        (Depth ? "" : "[ERROR] "), Msg);

    OutTraceNotSave(Group, Depth, szBuffer);
    Unlock();
}

void CTrcWrapper::trc_outf_(unsigned long Group,
    const char *GrName, unsigned long Depth,
    const char *szFormat, ...)
{
    char szBuffer[256];
    va_list pArg;

    Lock();

    snprintf(szBuffer, sizeof(szBuffer) - 1, "%s %s ", GrName, (Depth ? "" : "[ERROR]"));
    szBuffer[255] = 0;

    va_start(pArg, szFormat);
    vsnprintf(szBuffer + strlen(szBuffer),
        sizeof (szBuffer) - strlen(szBuffer), szFormat, pArg);
    va_end(pArg);

    OutTraceNotSave(Group, Depth, szBuffer);
    Unlock();
}

void CTrcWrapper::trc_outf(const char *szFormat, ...)
{
    char szBuffer[256];
    va_list pArg;

    Lock();

    va_start(pArg, szFormat);
    vsnprintf(szBuffer, sizeof (szBuffer), szFormat, pArg);
    va_end(pArg);

    OutTraceNotSave(group, depth, szBuffer);
    Unlock();
}

void CTrcWrapper::trc_outd(unsigned long Group,
    unsigned long Depth, unsigned char *Data, long Len)
{
    char Msg[200], Byte[8];
    long i = 0, ii = 1;

    Msg[0] = 0;

    Lock();


    while(i < Len) {

        snprintf(Byte, sizeof(Byte)-1, "%02x ", Data[i]);
        Byte[sizeof(Byte)-1] = 0;
        strncat(Msg, Byte, sizeof(Msg)-1);

        if(ii == 16 || i == (Len - 1)) {
            OutTraceNotSave(Group, Depth, Msg);
            ii = 0;
            Msg[0] = 0;
        }

        i++;
        ii++;
    }

    Unlock();
}

#endif /* PC_TRACE_ON */
