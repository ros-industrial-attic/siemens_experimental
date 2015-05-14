/****************************************************************************
*  Copyright (C) SIEMENS CORP., 2013 All rights reserved.
*****************************************************************************
* FILE NAME    : pnopping.c
* ---------------------------------------------------------------------------
* DESCRIPTION  : Test application
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

#if defined(_DPR_WINDOWS)
	#include "windows.h"
	#include <stdio.h>
	#include <stdlib.h>
#elif defined(_DPR_LINUX)
    #include <stdio.h>
    #include <unistd.h>
    #include <stdlib.h>
#endif

#include "pnioerrx.h"
#include "pniousrx.h"
#include "pniousrt.h"

PNIO_UINT32 app_handle;
PNIO_UINT8 data[64];
PNIO_MODE_TYPE current_mode;

void mode_ind(PNIO_CBE_PRM * pCbfPrm)
{
    if(pCbfPrm->CbeType != PNIO_CBE_MODE_IND) {
        printf("invalid type of callback\n");
        return;
    }

    if(pCbfPrm->Handle != app_handle) {
        printf("invalid handle in callback\n");
        return;
    }

    current_mode = pCbfPrm->ModeInd.Mode;

}

void wait_for(PNIO_MODE_TYPE mode, unsigned long timeout_ms)
{
    unsigned long i;

    for(i = 0; i < timeout_ms; i++) {
        #if defined(_DPR_WINDOWS)
            Sleep(1);
        #else
            sleep(1);
        #endif
        if(mode == current_mode)
            return;
    }
}

void set_tile(PNIO_UINT8 *pBuffer, PNIO_UINT32 BufLen, PNIO_UINT8 start_value)
{
    PNIO_UINT32 i;
    for(i = 0; i < BufLen; i++, start_value++) {
        pBuffer[i] = start_value;
    }
}

int compare_tile(PNIO_UINT8 *pBuffer, PNIO_UINT32 BufLen, PNIO_UINT8 start_value)
{
    PNIO_UINT32 i;
    for(i = 0; i < BufLen; i++, start_value++) {
        if(pBuffer[i] != start_value)
            return 0;
    }
    return 1;
}

int main(int argc, char *argv[])
{
    PNIO_UINT32 ret;
    unsigned int loops, i, start_value;

    /* default */
    loops = 10;
    start_value = 0;

    if(argc > 1) {
        sscanf(argv[1], "%u", &loops);
    }
    if(argc > 2) {
        sscanf(argv[2], "%u", &start_value);
    }

    fprintf(stdout, "PNIO_controller_open\n");
    ret = PNIO_controller_open(1, PNIO_CEP_MODE_CTRL, mode_ind, mode_ind, NULL, &app_handle);
    if(PNIO_OK != ret) {
        fprintf(stderr, "PNIO_controller_open failed %x\n", ret);
        exit(-1);
    }

    for(i = 0; i < loops; i++, start_value++) {
        /* set tile */
        set_tile(data, sizeof(data), start_value);

        fprintf(stdout, "PNIO_data_test loop %u\n", i);
        ret = PNIO_data_test(app_handle, data, sizeof(data));
        if(PNIO_OK != ret) {
            fprintf(stderr, "PNIO_data_test failed %x\n", ret);
        }

        /* compare */
        if(!compare_tile(data, sizeof(data), start_value))
            fprintf(stderr, "ERROR inconsistenz\n");
    }

    fprintf(stdout, "PNIO_close\n");
    PNIO_close(app_handle);
    fprintf(stdout, "success\n");
    return 0;
}
