/****************************************************************************
*  Copyright (C) SIEMENS CORP., 2013 All rights reserved.
*****************************************************************************
* FILE NAME    : trace_os.c
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

/* We using here a whole mmaped file.
    It let us search in contiguos buffer, and dont
    bother about data fragmentation */

static int config_fd = -1;
static unsigned long config_size = 0;
static void *config_base = NULL;

const char *TRC_GetFormattedLocalTime(char *szTime, int size) {
    struct timeb curtime;
    struct tm tmnow;
    ftime(&curtime);
    DPR_GMTIME(&curtime.time, &tmnow);

    snprintf(szTime, size, "%02d.%02d.%02d %02d:%02d:%02d,%03d ",
        tmnow.tm_mday,
        tmnow.tm_mon + 1,
        ((tmnow.tm_year + 1900 > 2000) ? (tmnow.tm_year + 1900 - 2000) : (tmnow.tm_year + 1900)),
        tmnow.tm_hour,
        tmnow.tm_min,
        tmnow.tm_sec,
        curtime.millitm);
    return szTime;
}

#ifdef WDB_TRACE_ON

/* general debug output function.
 */
extern "C" void __cdecl dbg_msg_out(char *szFormat, ... );

void __cdecl dbg_msg_out(char *szFormat, ... )
{
    va_list pArg;
    char szMessage[1024];
    szMessage[0] = '\0';

    va_start(pArg, szFormat);
    int msgLen = _vsnprintf (szMessage, sizeof(szMessage) - 3, szFormat, pArg);
    va_end(pArg);

    char *p = szMessage;
    p += (msgLen < 0) ?  sizeof(szMessage) - 3 : msgLen;
    while ( p > szMessage && isspace(p[-1]) ) {
        *--p = '\0'; // clear space, lf, cr
    }
    *p++ = '\r';     // set uinified crlf ant the end
    *p++ = '\n';
    *p++ = '\0';

    OutputDebugString( szMessage );
    return;
} // end of dbg_out()

#endif /* WDB_TRACE_ON */


void TRC_OutputDebugString(const char *txt) {
    printf(txt);
}

int TRC_ExtractBegin(const char * conffile, char * confpath, int confpathlen)
{
    struct stat filestat;
    char *env;
#ifndef WIN32
    /* search in
      working directory/$(conffile)
      $(HOME)/.$(conffile)
      /etc/$(conffile)
     */

    if((env = getenv("PWD"))) {
        snprintf(confpath, confpathlen, "%s/%s", env, conffile);

        if(-1 != stat(confpath, &filestat)) {
            goto TRC_ExtractBegin_found;
        }
    }

    if((env = getenv("HOME"))) {
        snprintf(confpath, confpathlen, "%s/.%s", env, conffile);

        if(-1 != stat(confpath, &filestat)) {
            goto TRC_ExtractBegin_found;
        }
    }

    {
        snprintf(confpath, confpathlen, "/etc/%s", conffile);

        if(-1 != stat(confpath, &filestat)) {
            goto TRC_ExtractBegin_found;
        }
    }
#else
    env = 0;
    snprintf(confpath, confpathlen, "%s\\trace\\%s", DPR_DRV_CONFIG_PATH(), conffile);

    if(-1 != stat(confpath, &filestat)) {
            goto TRC_ExtractBegin_found;
    }
#endif

	printf("TraceLib: trace configuration file '%s' not found\n", confpath);
	perror("TraceLib");

  return errno;

TRC_ExtractBegin_found:

  config_size = filestat.st_size;

  if(-1 == (config_fd = open(confpath, O_RDONLY))) {
    perror("TraceLib: TRC_ExtractBegin, open failed");
    return errno;
  }

	config_base = malloc (config_size);
  if(!config_base) {
    perror("TraceLib: TRC_ExtractBegin, malloc failed");
    return -1;
  }

  read(config_fd, config_base, config_size);

  return 0;
}

static char* __findnonwhite(char *str, const char *set, size_t len)
{
    char *p = str;
    size_t i = 0;

    while(i < len) {
        if(!strchr(set, *p))
            return p;
        i++;
        p++;
    }

    return NULL;
}

static char* __findnonwhiteback(char *str, const char *set, size_t len)
{
    char *p = str+len-1;
    size_t i = len-1;

    while(i) {
        if(!strchr(set, *p))
            return p;
        i--;
        p--;
    }

    return NULL;
}

unsigned long TRC_ExtractKey(const char *key, char *buffer, int len)
{
    char *pendoffile = (char*)config_base + config_size,
        *pcurrent = (char*)config_base,
        *pstartofline, *pendofline,
        *pstartofcomment, *pstartofvalue,
        *pstartofkey, *pendofkey;
    int result, i;

    if(!config_base)
        return 0;

    /* fprintf(stderr, "debug: begin, base %p, end %p\n", pcurrent, pendoffile); */

    while(pcurrent < pendoffile) {

        /* search for an end of line */
        pstartofline = pcurrent;
        pendofline = (char*)memchr(pcurrent, '\n', pendoffile - pcurrent);
        if(!pendofline)
            pendofline = pendoffile-1;

        /* fprintf(stderr, "debug: line start %p, end %p\n", pstartofline, pendofline); */
        pcurrent = pendofline+1;

        /* cut comment */
        pstartofcomment = (char*)memchr(pstartofline, '#', pendofline - pstartofline);
        if(pstartofcomment)
            pendofline = pstartofcomment;

        /* search for a first non white space char */
        pstartofline = __findnonwhite(pstartofline, " \t\r", pendofline - pstartofline);

        /* skip empty lines */
        if(!pstartofline || pendofline <= pstartofline) {
            /* fprintf(stderr, "debug: empty line start %p, end %p\n", pstartofline, pendofline); */
            continue;
        }

        /* search for a value */
        pstartofvalue = (char*)memchr(pstartofline, '=', pendofline - pstartofline);

        /* alert on lines without key, or without value */
        if(!pstartofvalue || pstartofvalue == pstartofline) {
            fprintf(stderr, "invalid line in config file found\n");
            continue;
        }

        pstartofkey = pstartofline;
        pendofkey = (char*)memchr(pstartofkey, ' ', pstartofkey - pstartofvalue);
        if(!pendofkey)
            pendofkey = pstartofvalue;

        /* fprintf(stderr, "debug: key start %p, end %p\n", pstartofkey, pendofkey); */
        if(!strncmp(pstartofkey, key, pendofkey - pstartofkey)) {
            /* avoid = */
            pstartofvalue++;

            /* search for a first non white space char */
            pstartofvalue = __findnonwhite(pstartofvalue, " \t\r",
                pendofline - pstartofvalue);
            if(!pstartofvalue)
                pstartofvalue = pendofline;

            /* search for a last non white char */
            pendofline = 1 + __findnonwhiteback(pstartofvalue, " \t\r",
                pendofline - pstartofvalue);
            if(!pendofline)
                pendofline = pstartofvalue;

            result = (int)(pendofline - pstartofvalue);

            if(buffer && len > result) {
                /* copy string to output buffer */
                for(i = 0; i < result; i++) {
                    buffer[i] = pstartofvalue[i];
                }
                buffer[i] = '\0';

                /* fprintf(stderr, "TraceLib: key %s, value '%s'\n", key, buffer); */
            }
            return result;
        }
    }

    return 0;
}

int TRC_ExtractEnd(void)
{
	if(config_fd != -1) {
		free(config_base);
        close(config_fd);
	}
    config_fd = -1;

    return 0;
}
