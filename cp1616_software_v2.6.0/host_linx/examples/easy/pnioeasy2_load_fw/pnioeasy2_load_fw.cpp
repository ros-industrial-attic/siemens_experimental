/*---------------------------------------------------------------------------*/
/* Copyright (C) SIEMENS CORP., 2013 All rights reserved.*/
/*---------------------------------------------------------------------------*/
/*                                                                           */
/*   Project           :                                                     */
/*   Filename          : pnioeasy2_load_fw.cpp                               */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*---------------------------------------------------------------------------*/
/*   Description:                                                            */
/*               This is a sample application to demonstrate the IO-Base     */
/*               user interface. This sample does following steps            */
/*                   1. Laden der Firmware in das Bauteil					 */
/*					 2. Initialize (with the use of change mode indication   */
/*                                  callback)                                */
/*                   3. Aktivieren und Deaktivieren des ersten Gerätes       */
/*					    wenn 'a' bzw 'd' gedrückt werden                     */
/*                   4. Printig alarm information                            */
/*                   5. Do some IO Transfer                                  */
/*                   6. Stop device                                          */
/*                   7. uninitialize                                         */
/*---------------------------------------------------------------------------*/
/* Attention : Callbacks are running concurrent in other threads so all      */
/*             printf statements should be synchronized. But this sample     */
/*             application doesn't synchronize for simplicity  !             */
/*                                                                           */
/*             For proper display performance use a console width of more    */
/*             than 130 characters                                           */
/*                                                                           */
/*---------------------------------------------------------------------------*/
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

#ifdef WIN32
#include <windows.h>
#include <stdio.h>
#include <conio.h>
#include <time.h>
#include <sys/timeb.h>
#define CRLF "\n"

#else
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <poll.h>
#include <termios.h>
#include <string.h>
#include <time.h>
#include <sys/timeb.h>
#define Sleep(x) usleep(x*1000)
#define CRLF "\r\n"
#endif

// Deklarationen der vorgegebenen Funktionen
#include "pniousrx.h"
// Informationen über die zurückgegebenen Errorcodes von dwErrorCode
#include "pnioerrx.h"
//Definition von DataType als enum
#include "servusrx.h"
//Zum Auslesen der Attribute einer Datei
#include <sys/stat.h>

/*------------------------------------------------------------------------*/
/* Global Data for Communication Processor                                */
/*------------------------------------------------------------------------*/

PNIO_UINT32 g_dwCpId = 1;

#define PNIO_DEVICE_COUNT 1			// Anzahl der IO-Geräte
int pnio_dev_return = 0;
int pnio_dev_failure = PNIO_DEVICE_COUNT;

/* Don't forget! Callbacks are running in concurrent threads !            */
/* 'volatile PNIO_MODE_TYPE g_currentMode' statement is                   */
/* required to force the compiler to generate unoptimized memory access   */
/* to 'g_currentMode'.                                                    */
/*                                                                        */
/* Example                                                                */
/*  while(g_currentMode == PNIO_MODE_OFFLINE){...};                       */
/* will not be optimized                                                  */


volatile PNIO_MODE_TYPE g_currentMode = PNIO_MODE_OFFLINE;
const char roll[]={'-','\\','|','/','-','\\','|','/'}; //roll ist eine optische Ausschmückung der Ausgabe

volatile bool cp_stop_req = false;

/*------------------------------------------------------------------------*/
/* Global Data for Device                                                 */
/*------------------------------------------------------------------------*/

volatile PNIO_IOXS g_localState = PNIO_S_GOOD;

#define DEVICE_INPUT_COUNT 2       // number of input modules

volatile PNIO_IOXS g_deviceInputState[DEVICE_INPUT_COUNT] = { PNIO_S_BAD, PNIO_S_BAD };

PNIO_ADDR g_deviceInputAddress[DEVICE_INPUT_COUNT] = {
    {PNIO_ADDR_LOG, PNIO_IO_IN, 0}, // input address of first  input module
    {PNIO_ADDR_LOG, PNIO_IO_IN, 1}  // input address of second input module
};

PNIO_UINT32 g_deviceInputLength[DEVICE_INPUT_COUNT] = {
    1,                          // length in bytes of first input module
    1                           // length in bytes of second input module
};
#define MAX_INPUT_LEN 1   //length of largest input module
PNIO_UINT8 g_deviceInputData[DEVICE_INPUT_COUNT][MAX_INPUT_LEN]={0};
//array with two dimensions in case of modules with more than 1 Byte data (not necessary in this example)

#define DEVICE_OUTPUT_COUNT  2      // number of output modules

volatile PNIO_IOXS g_deviceOutputState[DEVICE_OUTPUT_COUNT] = { PNIO_S_BAD, PNIO_S_BAD };

PNIO_ADDR g_deviceOutputAddress[DEVICE_OUTPUT_COUNT] = {
    {PNIO_ADDR_LOG, PNIO_IO_OUT, 0}, // output address of first  output module
    {PNIO_ADDR_LOG, PNIO_IO_OUT, 1}  // output address of second  output module
};

PNIO_UINT32 g_deviceOutputLength[DEVICE_OUTPUT_COUNT] = {
    1,                          // length in byte of first output module
    1                           // length in byte of second output module
};
#define MAX_OUTPUT_LEN 1   //length of largest output module
PNIO_UINT8 g_deviceOutputData[DEVICE_OUTPUT_COUNT][MAX_OUTPUT_LEN]={0};  
//array with two dimensions in case of modules with more than 1 Byte data (not necessary in this example)

/*-----------------------------------------------------------------------*/
/* mandatory callbacks                                                   */
/* but not used in this sample application                               */
/* da hier keine Datensätze gelesen bzw. geschrieben werden              */
/*-----------------------------------------------------------------------*/

void callback_for_ds_read_conf(PNIO_CBE_PRM * pCbfPrm);
void callback_for_ds_write_conf(PNIO_CBE_PRM * pCbfPrm);

/*-----------------------------------------------------------------------*/
/* useful but not mandatory callbacks                                    */
/*-----------------------------------------------------------------------*/

void callback_for_mode_change_indication(PNIO_CBE_PRM * pCbfPrm);
void callback_for_device_activation(PNIO_CBE_PRM * pCbfPrm);
void callback_for_alarm_indication(PNIO_CBE_PRM * pCbfPrm);
void callback_for_cp_stop_req(PNIO_CBE_PRM *pCbfPrm);


/*------------------------------------------------------------------------*/
/* forward declaration of helper functions                                */
/*------------------------------------------------------------------------*/

int load_filetoCP(char *s,PNIO_CP_DLD_TYPE DataType);
PNIO_UINT32 Initialize(PNIO_UINT32 CP_INDEX,PNIO_UINT32 *pdwHandle);
void UnInitialize(PNIO_UINT32 dwHandle);
void UpdateCyclicOutputData(PNIO_UINT32 dwHandle);
void UpdateCyclicInputData(PNIO_UINT32 dwHandle);
int getCharWithTimeout(void);
void GetSystemTimeString(char *tstring);





/*------------------------------------------------------------------------*/
/* Here starts the Application Code                                       */
/*------------------------------------------------------------------------*/

int main(void)
{
	PNIO_UINT32 dwHandle = 0;
    int key = 0, k = 0;
    int r = 0, j=0, pnio_Err_Code;
	char *s="..\\examples\\easy\\pnioeasy2_load_fw\\fw16xx-2.4.2.0.27.fwl";
	PNIO_CP_DLD_TYPE DataType = PNIO_CP_DLD_FIRMWARE;


	pnio_Err_Code = load_filetoCP(s,DataType);
	if (pnio_Err_Code != 0)
	{	printf("Error(loadfiletoCP())" CRLF);
		return 0;
	}

    printf("This sample application does following tasks" CRLF);
	printf("1.laden der Firmware in die Baugruppe" CRLF);
    printf("2.increases/decreases the value of the first/second output byte once per 100 milliseconds" CRLF);
    printf("3.reads input byte once per 100 milliseconds and displays it on screen" CRLF);
    printf("4.writes all called callbacks like alarm callback on screen" CRLF);
    printf("  (e.g. (un)plugging of modules)" CRLF);
    printf("5.if you press key 'd' first IO device will be deactivated" CRLF); //In diesem Beispiel gibt es nur ein IO-Gerät
    printf("6.if you press key 'a' first IO device will be activated" CRLF);

    printf(CRLF CRLF" Press 's' to start sample application" CRLF);
    printf("To stop sample application press 'q'." CRLF);

	//Einlesen des Zeichens
    do {
        key = getCharWithTimeout();
        if(key == (int)'q' || key == (int)'Q')
            return 0;
    } while(key != (int)'s' && key != (int)'S');

    
	//Initialisieren (Anmelden der Baugruppe als IO-Controller, wechseln in Betriebszustand OPERATE und Registrierung der Callbacks)
    if(PNIO_OK != Initialize(g_dwCpId, &dwHandle))
        return 0;

	printf(CRLF "Warten auf Geraete");
    do {
		j = 0;

		// Warten auf Aktivierung bzw. Reaktivierung der IO-Geräte
			while ((pnio_dev_return != PNIO_DEVICE_COUNT))
			{
				//Mindestens ein Gerät wurde per 'd' deaktiviert
				if ((pnio_dev_failure >=1) && (key == k)&&(key == (int)'d'))
				{
					printf(CRLF "Geraet wurde deaktiviert" CRLF);
					printf("Zum aktivieren bitte 'a' druecken" CRLF);
					printf("Oder 'q' zum Beenden" CRLF);
				}
				//Wartezeit auf die Rückmeldung der IO-Geräte nach der Aktivierung bzw. Reaktivierung
				Sleep(10);
				j++;
				key = getCharWithTimeout();
				
				if ((j == 200)&& (k != (int) 'd')) // maximales Warten auf Aktivierung der IO-Geräte (~ 23 sekunden)
					{
						//Keine Rückmeldung von IO-Geräten nach dem Start oder nach einer Reaktivierung
						printf(CRLF "Keine Rueckmeldung von %i Geraet(en)" CRLF, PNIO_DEVICE_COUNT - pnio_dev_return);
						printf("Zum Beenden bitte 'q' druecken" CRLF);
						do {
							key = getCharWithTimeout();
						} while((key != (int)'q') || (key != (int)'Q'));
						goto ende;
						 
					}
				if ((key == (int)'q')||(key == (int)'Q'))
					goto ende;

				//Aktivieren des IO-Gerätes
				if (key == (int) 'a')
				{
					PNIO_device_activate(dwHandle, &g_deviceInputAddress[0], PNIO_DA_TRUE);
					printf( CRLF "Geraet wird aktiviert");
					key = ' ';
					k=key;
				}
			}
			printf("in1: %02X   in2: %02X   out1: %02X   out2: %02X " CRLF,g_deviceInputData[0][0], g_deviceInputData[1][0], g_deviceOutputData[0][0], g_deviceOutputData[1][0]); 
        g_deviceOutputData[0][0]++;    /* Increase Output Data byte 0 of device #1 */
        g_deviceOutputData[1][0]--;    /* Decrease Output Data byte 0 of device #2 */
        
        UpdateCyclicOutputData(dwHandle);
        UpdateCyclicInputData(dwHandle);
    
		//Ausgabe der Moduldaten und -zustände
        printf("%c out0:0x%02X state: %s out1:0x%02X state: %s   in0:0x%02X state: %s in1:0x%02X state: %s \r" CRLF,
                roll[r++],
                g_deviceOutputData[0][0],
                ((g_deviceOutputState[0] == PNIO_S_GOOD) ? "good" : "bad "),
                g_deviceOutputData[1][0],
                ((g_deviceOutputState[1] == PNIO_S_GOOD) ? "good" : "bad "),
                g_deviceInputData[0][0],
                ((g_deviceInputState[0] == PNIO_S_GOOD) ? "good" : "bad "),
                g_deviceInputData[1][0],
                ((g_deviceInputState[1] == PNIO_S_GOOD) ? "good" : "bad "));

        fflush(stdout);	
        if(r==sizeof(roll))
            r=0;

		// Fehlermeldung wenn mindestens ein Modul fehlerhaft ist 
		if((PNIO_S_GOOD !=g_deviceOutputState[0])
				||(PNIO_S_GOOD!=g_deviceInputState[0])
				||(PNIO_S_GOOD!=g_deviceOutputState[1])
				||(PNIO_S_GOOD!=g_deviceInputState[1]))
			{
				printf("Mindestens ein Modul ist fehlerhaft" CRLF);
				printf("Beheben Sie den Fehler" CRLF);
				printf("Zum Beenden druecken Sie 'q'" CRLF);
				do {
					key = getCharWithTimeout();
				} while(key != (int)'q' && key != (int)'Q');
				goto ende;
			}

		key = getCharWithTimeout();

		//Deaktivieren des IO-Gerätes
			if (key == (int)'d')
			{
				PNIO_device_activate(dwHandle, &g_deviceInputAddress[0], PNIO_DA_FALSE);
				Sleep(1000);	//Warten auf ALARM_DEV_FAILURE - Callback 
				k=key;
			}
	
    } while(key != (int)'q' && key != (int)'Q' && !cp_stop_req );
	
	//Deinitialisieren (Wechsel in Betriebszustand OFFLINE und abmelden der Baugruppe als IO-Controller)
ende:
    UnInitialize(dwHandle);

    exit (0);
}

/*-------------------------------------------------------------*/
/*    Diese Funktion lädt eine Datei in das CP1616- Bauteil    */                                                          
/*                                                             */
/*-------------------------------------------------------------*/

int load_filetoCP(char *s,PNIO_CP_DLD_TYPE DataType)
{
	struct stat attribut;
	FILE *datei;
	PNIO_UINT32 dataLen;
	PNIO_UINT8 *pData;
	PNIO_UINT32 dwErrorCode;
	
	//Länge der Datei bestimmen
	stat(s, &attribut);
	dataLen = attribut.st_size; 
	
	// Datei öffnen
	datei = fopen(s, "rb");
	if (datei== NULL)
	{ 
		printf("Error fopen()" CRLF);
		return 1;
	}
	else printf("Datei geoeffnet" CRLF);

	// Speicher allokieren
	if ((pData = (PNIO_UINT8*)malloc(dataLen * sizeof(char))) == NULL)	
	{ 
		printf("Error malloc()" CRLF);
		return 1;	
	}
	
	//Datei in Speicherbereich lesen
	if (fread(pData,dataLen,1,datei) != 1)
	{
		printf("Error bei fread()" CRLF);
		return 1;
	}

	// Datei in Bauteil laden
	printf("Daten werden in Bauteil geladen..." CRLF);
	dwErrorCode = SERV_CP_download(g_dwCpId,DataType,pData,dataLen);
	if(dwErrorCode != PNIO_OK)
	{
		printf("Error in loadfiletoCP()" CRLF);
		printf("SERV_CP_download returned 0x%x" CRLF, dwErrorCode);
		return 1;
	}

	printf("Datei erfolgreich in Bauteil geladen" CRLF);
	
	// Speicherbereich freigeben
	free(pData);

	//Datei schließen
	if (fclose(datei)!= 0)
	{
		printf("Error fclose()" CRLF);
		return 1;
	}
	else printf("Datei erfolgreich geschlossen" CRLF CRLF);
	
	return 0;
}

/*-------------------------------------------------------------*/
/* this function sets operational mode                         */
/*                                                             */
/* do not call before Initialize was called successfully       */
/* because it needs PNIO_CBE_MODE_IND callback to be registered*/
/* returns error code                                          */
/*-------------------------------------------------------------*/

PNIO_UINT32 ChangeAndWaitForPnioMode(PNIO_UINT32 dwHandle, PNIO_MODE_TYPE mode)
{
    PNIO_UINT32 dwErrorCode;

    /*setting  mode asynchronously                                 */
    dwErrorCode = PNIO_set_mode(dwHandle, mode);

    if(dwErrorCode != PNIO_OK) {
        printf(CRLF" Error in ChangeAndWaitForPnioMode " CRLF);
        printf(CRLF" PNIO_set_mode returned 0x%x" CRLF, dwErrorCode);
        PNIO_close(dwHandle);
        return(dwErrorCode);                // return
    };

    /* wait for callback_for_mode_change_indication to be called. */
    /* callback_for_mode_change_indication sets g_currentMode     */
    printf(CRLF"waiting for changing operation mode" CRLF);
    while(g_currentMode != mode) {
        Sleep(100);
        printf(".");
    };

    printf("" CRLF);

    return(PNIO_OK);
}

/*----------------------------------------------------------------------*/
/* this function initializes the IO-BASE and returns a handle           */
/* by reference necessary for all subsequent calls to IO-BASE functions */
/* return value is PNIO error code                                      */
/*----------------------------------------------------------------------*/

PNIO_UINT32 Initialize(PNIO_UINT32 CP_INDEX,PNIO_UINT32* pdwHandle)
{
	PNIO_UINT32 dwErrorCode = PNIO_OK;

    /* Connect to Communication Processor and obtain a handle */

    dwErrorCode = PNIO_controller_open(
             /*in*/  CP_INDEX,                     /* index of communication processor      */
             /*in*/  PNIO_CEP_MODE_CTRL,           /* permission to change operation mode   */
             /*in*/  callback_for_ds_read_conf,    /* mandatory  callback                   */
             /*in*/  callback_for_ds_write_conf,   /* mandatory callback                    */
             /*in*/  callback_for_alarm_indication,/* alarm callback                        */
             /*out*/ pdwHandle                     /* handle                                */
        );
    printf(" Device Handle dwHandle = %d " CRLF, *pdwHandle); //Ausgabe von handle

    if(dwErrorCode != PNIO_OK) {
        printf(" Error in Initialize " CRLF);
        printf(" PNIO_controller_open returned 0x%x" CRLF, dwErrorCode);
        return(dwErrorCode);                // return
    };

    /* here we register the callback                         */
    /* PNIO_CBE_MODE_IND    for Mode changes  confirmation   */
    dwErrorCode = PNIO_register_cbf(
        /*in */ *pdwHandle,
        /*in */ PNIO_CBE_MODE_IND,
        /*in */ callback_for_mode_change_indication
        );

    if(dwErrorCode != PNIO_OK) {
        printf(" Error in Initialize " CRLF);
        printf(" PNIO_register_cbf (PNIO_CBE_MODE_IND,..)  returned 0x%x" CRLF,
            dwErrorCode);
        PNIO_close(*pdwHandle);
        return(dwErrorCode);                // return
    }

    /* here we register the callback                           */
    /* PNIO_CBE_DEV_ACT     for device activation confirmation */

    dwErrorCode = PNIO_register_cbf(
        /*in */ *pdwHandle,
        /*in */ PNIO_CBE_DEV_ACT_CONF,
        /*in */ callback_for_device_activation
        );

    if(dwErrorCode != PNIO_OK) {
        printf(" Error in Initialize " CRLF);
        printf(" PNIO_register_cbf (PNIO_CBE_DEV_ACT_CONF,..)  returned 0x%x" CRLF,
            dwErrorCode);
        PNIO_close(*pdwHandle);
        return(dwErrorCode);                // return
    }

    /* here we register the callback                               */
    /* PNIO_CBE_CP_STOP_REQ zur Bestätigung eines Stoppen der CP-Baugruppe */

    dwErrorCode = PNIO_register_cbf(
        /*in */ *pdwHandle,
        /*in */ PNIO_CBE_CP_STOP_REQ,
        /*in */ callback_for_cp_stop_req
        );

    if(dwErrorCode != PNIO_OK) {
        printf(" Error in Initialize " CRLF);
        printf(" PNIO_register_cbf (PNIO_CBE_CP_STOP_REQ,..)  returned 0x%x" CRLF,
            dwErrorCode);
        PNIO_close(*pdwHandle);
        return(dwErrorCode);                // return
    }

    /* here we change the mode to PNIO_MODE_OPERATE  */
    return(ChangeAndWaitForPnioMode(*pdwHandle, PNIO_MODE_OPERATE));

}


/*-------------------------------------------------------------*/
/* this function uninitializes the IO-BASE                     */
/*                                                             */
/* parameters                                                  */
/*    handle         : handle to the communication processor   */
/*-------------------------------------------------------------*/

void UnInitialize(PNIO_UINT32 dwHandle)
{
    PNIO_UINT32 dwErrorCode = PNIO_OK;

    /* here we change the mode to PNIO_MODE_OFFLINE  */
	dwErrorCode = ChangeAndWaitForPnioMode(dwHandle, PNIO_MODE_OFFLINE);
    if(dwErrorCode != PNIO_OK) {
        return;
    }
	//Schließen der CP_Baugruppe
    dwErrorCode = PNIO_close(dwHandle);

    if(dwErrorCode != PNIO_OK) {
        printf(" Error in UnInitialize " CRLF);
        printf(" PNIO_close returned 0x%x" CRLF, dwErrorCode);
    };
}


/*-------------------------------------------------------------*/
/* this function writes the cyclic output data to IO-BASE      */
/*                                                             */
/* parameters                                                  */
/*    handle         : handle to the communication processor   */
/*-------------------------------------------------------------*/

void UpdateCyclicOutputData(PNIO_UINT32 dwHandle)
{
    PNIO_UINT32 dwErrorCode;
    for(int i = 0; i < DEVICE_OUTPUT_COUNT; i++) {
        dwErrorCode = PNIO_data_write(
                       /*in*/ dwHandle,                             /*handle                            */
                       /*in*/ &(g_deviceOutputAddress[i]),          /* pointer to device output address */
                       /*in*/ g_deviceOutputLength[i],              /* length in bytes of output        */
                       /*in*/ &(g_deviceOutputData[i][0]),          /* pointer to output data           */
                       /*in*/ g_localState,                         /* local status                     */
                       /*out*/(PNIO_IOXS*)&(g_deviceOutputState[i]) /* remote status                    */
            );
    if(dwErrorCode != PNIO_OK)
		printf(CRLF"Error 0x%X in PNIO_data_write() Device %d "CRLF,dwErrorCode,i);
    }
}

/*-------------------------------------------------------------*/
/* this function reads the cyclic input data from IO-BASE      */
/*                                                             */
/* parameters                                                  */
/*    handle         : handle to the communication processor   */
/*-------------------------------------------------------------*/
void UpdateCyclicInputData(PNIO_UINT32 dwHandle)
{
    PNIO_UINT32 dwErrorCode;
    PNIO_UINT32 dwBytesReaded;
    for(int i = 0; i < DEVICE_INPUT_COUNT; i++) {
        dwErrorCode = PNIO_data_read(
                       /*in*/  dwHandle,                           /*handle                           */
                       /*in*/  &g_deviceInputAddress[i],           /* pointer to device input address */
                       /*in*/  g_deviceInputLength[i],             /* length in bytes of input        */
                       /*out*/ &dwBytesReaded,                     /* number of bytes read            */
                       /*in*/  &g_deviceInputData[i][0],           /* pointer to input data           */
                       /*in*/  g_localState,                       /* local status                    */
                       /*out*/(PNIO_IOXS*)&(g_deviceInputState[i]) /* remote status                   */
                       );
    if(dwErrorCode != PNIO_OK)
		printf(CRLF"Error 0x%X in PNIO_data_read() Device %d "CRLF,dwErrorCode,i);  
    }
}


/*--------------------------------------------------*/
/* mandatory callbacks but not used in this sample  */
/*--------------------------------------------------*/

void callback_for_ds_read_conf(PNIO_CBE_PRM * pCbfPrm)
{
    /**************************************************************/
    /* Attention :                                                */
    /* this is a callback and must be returned as soon as possible*/
    /* don't use any endless or time consuming functions          */
    /* e.g. exit() would be fatal                                 */
    /* defer all time consuming functionality to other threads    */
    /**************************************************************/

    printf( CRLF"callback_for_ds_read_conf " CRLF);
    printf("Fehler da dieser Callback nicht ausgeloest werden duerfte" CRLF);
}


void callback_for_ds_write_conf(PNIO_CBE_PRM * pCbfPrm)
{
    /**************************************************************/
    /* Attention :                                                */
    /* this is a callback and must be returned as soon as possible*/
    /* don't use any endless or time consuming functions          */
    /* e.g. exit() would be fatal                                 */
    /* defer all time consuming functionality to other threads    */
    /**************************************************************/
    printf( CRLF"callback_for_ds_write_conf " CRLF);
    printf("Fehler da dieser Callback nicht ausgeloest werden duerfte" CRLF);
}

/*--------------------------------------------------*/
/* useful callbacks                                 */
/*--------------------------------------------------*/


/*-------------------------------------------------------------*/
/* this function will be called from IO-BASE to signal a change*/
/* in the opreation mode                                       */
/*                                                             */
/* parameters                                                  */
/*    pCbfPrm         : Callback information                   */
/*-------------------------------------------------------------*/

void callback_for_mode_change_indication(PNIO_CBE_PRM * pCbfPrm)
{

    /**************************************************************/
    /* Attention :                                                */
    /* this is a callback and must be returned as soon as possible*/
    /* don't use any endless or time consuming functions          */
    /* e.g. exit() would be fatal                                 */
    /* defer all time consuming functionality to other threads    */
    /**************************************************************/

    /* Check if correct callback type */
    if(pCbfPrm->CbeType == PNIO_CBE_MODE_IND) {
        /* Callback has correct type so check mode change */
        /* and set global variable                        */
        switch (pCbfPrm->ModeInd.Mode) {
        case PNIO_MODE_OFFLINE:
            g_currentMode = PNIO_MODE_OFFLINE;
            break;
        case PNIO_MODE_CLEAR:
            g_currentMode = PNIO_MODE_CLEAR;
            break;
        case PNIO_MODE_OPERATE:
            g_currentMode = PNIO_MODE_OPERATE;
            break;
        default:
            printf("callback_for_mode_change_indication called with wrong mode" CRLF);
            break;
        };
    };

    printf("callback_for_mode_change_indication was called" CRLF);
}

/*-------------------------------------------------------------*/
/* this function will be called from IO-BASE to signal that    */
/* an alarm has been received                                   */
/*                                                             */
/* parameters                                                  */
/*    pCbfPrm         : Callback information                   */
/*-------------------------------------------------------------*/

void callback_for_alarm_indication(PNIO_CBE_PRM * pCbfPrm)
{
    char time_str[13];
    /**************************************************************/
    /* Attention :                                                */
    /* this is a callback and must be returned as soon as possible*/
    /* don't use any endless or time consuming functions          */
    /* e.g. exit() would be fatal                                 */
    /* defer all time consuming functionality to other threads    */
    /**************************************************************/

    /* Check if correct callback type */
    if(pCbfPrm->CbeType == PNIO_CBE_ALARM_IND) {
        GetSystemTimeString(time_str);
        switch (pCbfPrm->AlarmInd.pAlarmData->AlarmType) {
        case PNIO_ALARM_DIAGNOSTIC:
            printf(CRLF"PNIO_ALARM_DIAGNOSTIC at %s" CRLF, time_str);
            break;
        case PNIO_ALARM_PROCESS:
            printf(CRLF"PNIO_ALARM_PROCESS at %s" CRLF, time_str);
            break;
        case PNIO_ALARM_PULL:
            printf(CRLF"PNIO_ALARM_PULL at %s" CRLF, time_str);
            break;

        case PNIO_ALARM_PLUG:
            printf(CRLF"PNIO_ALARM_PLUG at %s" CRLF, time_str);
            break;

        case PNIO_ALARM_STATUS:
            printf(CRLF"PNIO_ALARM_STATUS at %s" CRLF, time_str);
            break;

        case PNIO_ALARM_UPDATE:
            printf(CRLF"PNIO_ALARM_UPDATE at %s" CRLF, time_str);
            break;

        case PNIO_ALARM_REDUNDANCY:
            printf(CRLF"PNIO_ALARM_REDUNDACY at %s" CRLF, time_str);
            break;

        case PNIO_ALARM_CONTROLLED_BY_SUPERVISOR:
            printf(CRLF"PNIO_ALARM_CONTROLLED_BY_SUPERVISOR at %s" CRLF, time_str);
            break;

        case PNIO_ALARM_RELEASED_BY_SUPERVISOR:
            printf(CRLF"PNIO_ALARM_RELEASED_BY_SUPERVISOR at %s" CRLF, time_str);
            break;

        case PNIO_ALARM_PLUG_WRONG:
            printf(CRLF"PNIO_ALARM_PLUG_WRONG at %s" CRLF, time_str);
            break;

        case PNIO_ALARM_RETURN_OF_SUBMODULE:
            printf(CRLF"PNIO_ALARM_RETURN_OF_SUBMODULE  at %s" CRLF, time_str);
            break;

        case PNIO_ALARM_DEV_FAILURE:
            printf(CRLF"PNIO_ALARM_DEV_FAILURE at %s" CRLF, time_str);
			pnio_dev_failure ++;
			pnio_dev_return --;
            break;

        case PNIO_ALARM_DEV_RETURN:
            printf(CRLF"PNIO_ALARM_DEV_RETURN at %s" CRLF, time_str);
			pnio_dev_return ++;
			pnio_dev_failure --;
            break;
        default:
            printf(CRLF"callback_for_alarm_indication called with wrong type at %s" CRLF, time_str);
            break;
        }
    }
}

/*-------------------------------------------------------------*/
/* this function will be called from IO-BASE to signal that    */
/* a device was activated or deactivated                       */
/*                                                             */
/* parameters                                                  */
/*    pCbfPrm         : Callback information                   */
/*-------------------------------------------------------------*/
void callback_for_device_activation(PNIO_CBE_PRM * pCbfPrm)
{
	char time_str[13];
    /**************************************************************/
    /* Attention :                                                */
    /* this is a callback and must be returned as soon as possible*/
    /* don't use any endless or time consuming functions          */
    /* e.g. exit() would be fatal                                 */
    /* defer all time consuming functionality to other threads    */
    /**************************************************************/
    
    GetSystemTimeString(time_str);
    switch (pCbfPrm->DevActConf.Mode) {
    case PNIO_DA_TRUE:
        printf(CRLF"device activation was send device with Address %d with result %x at %s" CRLF,
            pCbfPrm->DevActConf.pAddr->u.Addr, pCbfPrm->DevActConf.Result, time_str);
        break;
    case PNIO_DA_FALSE:
        printf(CRLF"device deactivation was send to device with Address %d with result %x at %s"
            CRLF, pCbfPrm->DevActConf.pAddr->u.Addr, pCbfPrm->DevActConf.Result, time_str);
        break;
    };
}

/*-------------------------------------------------------------*/
/* this function will be called from IO-BASE to signal that    */
/* die CP Baugruppe in den Stopmodus geht                      */
/*                                                             */
/* parameters                                                  */
/*    pCbfPrm         : Callback information                   */
/*-------------------------------------------------------------*/
void callback_for_cp_stop_req(PNIO_CBE_PRM *pCbfPrm)
{
    printf (CRLF"CP_STOP_REQUEST received, terminating Programm ..."CRLF);
    cp_stop_req = true;
}

/*****************************************************************/
/* This functions reads a character from console                 */
/*****************************************************************/

int getCharWithTimeout()
{
    int key = 0;
#ifndef WIN32
    struct pollfd pollfd[1];
    static int init = 0;
    static struct termios termiosOld;
    static struct termios termios;

    if(!init) {
        tcgetattr(fileno(stdin), &termios);
        memcpy(&termiosOld, &termios, sizeof (termios));
        cfmakeraw(&termios);
        init++;
    }
    /* put terminal into raw mode */
    tcsetattr(fileno(stdin), TCSANOW, &termios);
    pollfd->fd = fileno(stdin);
    pollfd->events = POLLIN;
    pollfd->revents = 0;
    poll(pollfd, 1, 100);
    if(pollfd->revents & POLLIN)
        key = getchar();
    tcsetattr(fileno(stdin), TCSANOW, &termiosOld);

#else

    Sleep(100);
    if(_kbhit()) {	//Gibt 0 zurück wenn keine Taste gedrückt wurde 
        key = _getch();
    }
#endif

    return key;
}


/***************************************************************************/
/* GetSystemTimeString needs a char array with length of at least 13 Bytes */
/* returns the local time hh:mm:ss.mmm                                     */
/***************************************************************************/

void GetSystemTimeString(char *tstring)
{

#ifdef WIN32
    #define my_timeb  _timeb
    #define my_ftime  _ftime
#else
    #define my_timeb  timeb
    #define my_ftime  ftime
#endif

    struct my_timeb timebuffer;
    char *timeline;

    my_ftime( &timebuffer );
    timeline = ctime( & ( timebuffer.time ) );  //Bsp. timeline = {Wed Jan 02 02:03:55 1980\n\0}

	// Ausgabe der Uhrzeit
    sprintf(tstring,"%.8s.%03u", &timeline[11], timebuffer.millitm);

    return;
}
