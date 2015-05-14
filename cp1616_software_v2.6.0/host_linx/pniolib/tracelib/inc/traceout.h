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

#ifndef _TRACE_OUT_H_
#define _TRACE_OUT_H_

#include "os.h"

class CTraceOut {
  public:
    CTraceOut(void);
    virtual ~ CTraceOut(void);

    inline void Lock(void) {
        DPR_MUTEX_LOCK(m_hMutex);
    };

    inline void Unlock(void) {
        DPR_MUTEX_UNLOCK(m_hMutex);
    };

    long GetMaxBackFiles(long &lMaxBackFiles);
    long SetMaxBackFiles(long lMaxBackFiles);
    long GetDest(unsigned  long &lDest);
    long SetDest(unsigned long lDest);
    long GetGroup(unsigned long &);
    long SetGroup(unsigned long);
    long GetFileFast(long &lFileFast);
    long SetFileFast(long lFileFast);
    long GetDepth(unsigned long &);
    long SetDepth(unsigned long);
    long GetFileName(char *szFilename);
    long SetFileName(char *szFilename);

    long InitFromFile(const char *szRegPath);
    long InitFromRegistry(const char *szTraceKey);

    long Rescue(void);

    inline void OutTraceNotSave(unsigned long TraceGroup,
        unsigned long TraceDepth, const char *szMessage) {
        OutTracePrivate(TraceGroup, TraceDepth, szMessage);
    }

  private:

    void OutTracePrivate(unsigned long TraceGroup,
        unsigned long TraceDepth, const char *szMessage);
    long Rename(long lIndex);


    FILE *m_pFile;
    DPR_MUTEX m_hMutex;
    long m_lFileCurrentEntries;
    long m_lCurrentBackFiles;

    unsigned long m_ulDepth;
    unsigned long m_ulGroup;
    long m_lDest;
    long m_lFileFast;
    long m_lFileEntries;
    long m_lMaxBackFiles;
    long m_lTime;
    char m_szFilename[255];
    long m_lPID;
};

const char *TRC_GetFormattedLocalTime(char *szTime, int size);
void TRC_OutputDebugString(const char *txt);

int TRC_ExtractBegin(const char * conffile, char * confpath, int confpathlen);
int TRC_ExtractEnd(void);
unsigned long TRC_ExtractKey(const char *key, char *buffer, int len);

#endif /* _TRACE_OUT_H_ */
