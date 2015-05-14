/*---------------------------------------------------------------------------*/
/* Copyright (C) SIEMENS CORP., 2013 All rights reserved.*/
/*---------------------------------------------------------------------------*/
/*                                                                           */
/*   Project           :                                                     */
/*   Filename          : pnioeasy1_load_proj.cpp                             */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*---------------------------------------------------------------------------*/
/*   Description:                                                            */
/*               This is a sample application to demonstrate the IO-Base     */
/*               user interface. This sample does following steps			 */
/*					 1. Laden der Projektierung in die CP1616 Baugruppe		 */
/*                   2. Initialize                                           */
/*                   3. do some IO Transfer                                  */
/*                   4. Stop device                                          */
/*                   5. uninitialize                                         */
/*---------------------------------------------------------------------------*/
/* Attention : Callbacks are running concurrent in other threads so all      */
/*             printf statements should be synchronized. But this sample     */
/*             application doesn't synchronize for simplicity  !             */
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
#define CRLF "\n"

#else
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <poll.h>
#include <termios.h>
#include <string.h>
#define CRLF "\r\n"
#define Sleep(x) usleep(x*1000)
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

/* Index of CP given by project */
PNIO_UINT32 g_dwCpId = 1;


/* Don't forget! Callbacks are running in concurrent threads !            */
/* 'volatile PNIO_MODE_TYPE g_currentMode' statement is                   */
/* required to force the compiler to generate unoptimized memory access   */
/* to 'g_currentMode'.                                                    */
/*                                                                        */
/* Example                                                                */
/*  while(g_currentMode == PNIO_MODE_OFFLINE){...};                       */
/* will not be optimized                                                  */


volatile PNIO_MODE_TYPE g_currentMode = PNIO_MODE_OFFLINE;



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
//array with two dimensions in case of modules with more than 1 Byte of data ( not necessary in this example)

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
//array with two dimensions in case of modules with more than 1 Byte of data ( not necessary in this example)

/*------------------------------------------------------------------------*/
/* mandatory callbacks                                                    */
/* but not used in this sample application                                */
/* da hier keine Datensätze gelesen bzw. geschrieben werden               */
/*------------------------------------------------------------------------*/

void callback_for_ds_read_conf(PNIO_CBE_PRM * pCbfPrm);
void callback_for_ds_write_conf(PNIO_CBE_PRM * pCbfPrm);

/*-----------------------------------------------------------------------*/
/* useful but not mandatory callbacks                                    */
/*-----------------------------------------------------------------------*/

void callback_for_mode_change_indication(PNIO_CBE_PRM * pCbfPrm);

/*------------------------------------------------------------------------*/
/* forward declaration of helper functions                                */
/*------------------------------------------------------------------------*/

int load_filetoCP(char *s,PNIO_CP_DLD_TYPE DataType);
PNIO_UINT32 Initialize(PNIO_UINT32 CP_INDEX,PNIO_UINT32 *pdwHandle);
void UnInitialize(PNIO_UINT32 dwHandle);
void UpdateCyclicOutputData(PNIO_UINT32 dwHandle);
void UpdateCyclicInputData(PNIO_UINT32 dwHandle);
int getCharWithTimeout(void);




/*------------------------------------------------------------------------*/
/* Here starts the Application Code                                       */
/*------------------------------------------------------------------------*/

int main(void)
{

    PNIO_UINT32 dwHandle = 0;
    int key=0, i=0, j=0, pnio_Err_Code=0;
	bool init;
	char *s="..\\..\\..\\..\\Examples\\Easy\\pnioeasy1_load_proj\\Testproj-1616\\XDBs\\pcst_1.xdb";
	PNIO_CP_DLD_TYPE DataType = PNIO_CP_DLD_CONFIG;

	pnio_Err_Code = load_filetoCP(s,DataType);
	if (pnio_Err_Code != 0)
	{	printf("Error(loadfiletoCP())" CRLF);
		return 0;
	}
	 
    printf("This sample application does following tasks" CRLF);
	printf("1. Laden der Projektierung in die CP1616 Baugruppe" CRLF);
    printf("2.increases/decreases the value of the first/second output byte once per 100 milliseconds" CRLF);
    printf("3.reads input byte once per 100 milliseconds and displays it on screen" CRLF);

    printf("\n\n Press 's' to start sample application" CRLF);
    printf("To stop sample application press 'q'." CRLF);

	// Einlesen des Zeichens
    do {
        key = getCharWithTimeout();
        if(key == (int)'q' || key == (int)'Q')
            return 0;

    } while(key != (int)'s' && key != (int)'S');

        
	// Initialisieren (Anmelden der Baugruppe als IO-Controller, wechseln in Betriebszustand OPERATE und Registrierung der Callbacks)
    if(PNIO_OK != Initialize(g_dwCpId, &dwHandle))
        return 0;

	init=true;

	printf("Warten auf Rueckmeldung der Geraete" CRLF);

	do {
        
		g_deviceOutputData[0][0]++;    /* Increase Output Data byte 0 of device #1 */
        g_deviceOutputData[1][0]--;    /* Decrease Output Data byte 0 of device #2 */
        	
		j=0;
		i=0;

		// Warten auf Rückmeldung der Geräte oder bei Ausfall eines oder mehrerer Module
		while (((g_deviceOutputState[0] == PNIO_S_BAD)
				||(g_deviceOutputState[1] == PNIO_S_BAD)
				||(g_deviceInputState[0] == PNIO_S_BAD)
				||(g_deviceInputState[1] == PNIO_S_BAD))
				&&(key != (int) 'q'))
		{
			// Wartezeit = ~ 23 sekunden
			Sleep(10);
			j++;
			key = getCharWithTimeout();
			if ((j==2)&&(i==0)&&(init==false))
			{
				printf("Mindestens ein Modul ist nicht verbunden" CRLF);
				printf("Druecken Sie 'q' zum Beenden oder beheben Sie das Problem" CRLF);
			}
			// Ausgabe nach Ablauf der Wartezeit
			if (j == 200)
			{
				i++;
				j=0;
				//Ausgabe der Daten und Zustände des ersten Ausgabe- und Eingabemoduls 
				printf("output1 device state:%s   input1 device state:%s"CRLF,
				((g_deviceOutputState[0] == PNIO_S_GOOD) ? "good" : "bad "),
				((g_deviceInputState[0] == PNIO_S_GOOD) ? "good" : "bad ")
				 );

				//Ausgabe der Daten und Zustände des zweiten Ausgabe- und Eingabemoduls 
				printf("output2 device state:%s   input2 device state:%s"CRLF,
				((g_deviceOutputState[1] == PNIO_S_GOOD) ? "good" : "bad "),
				((g_deviceInputState[1] == PNIO_S_GOOD) ? "good" : "bad ")
				);
				printf("Seit ca. %i sekunden gibt es keine Rueckmeldung von mindestens einem Modul" CRLF, 23*i);
				printf("Druecken Sie 'q' zum Beenden oder beheben Sie das Problem" CRLF);
			}

			// Überprüfung ob zu allen Modulen eine Verbindung besteht
			UpdateCyclicOutputData(dwHandle);
			UpdateCyclicInputData(dwHandle);
			
		} 

		init=false;

		UpdateCyclicOutputData(dwHandle);			//________________------- Mein Eintrag
		UpdateCyclicInputData(dwHandle);
		key = getCharWithTimeout();


        // Beenden der Schleife und des Programms bei Eingabe von 'q'
		if (key == (int)'q')
			break;

		//Ausgabe der Daten und Zustände des ersten Ausgabe- und Eingabemoduls 
        printf("output1:0x%02x  output device state:%s   input1: 0x%02x input device state:%s"CRLF,
            g_deviceOutputData[0][0],
            ((g_deviceOutputState[0] == PNIO_S_GOOD) ? "good" : "bad "),
            g_deviceInputData[0][0],
            ((g_deviceInputState[0] == PNIO_S_GOOD) ? "good" : "bad ")
            );

		//Ausgabe der Daten und Zustände des zweiten Ausgabe- und Eingabemoduls 
        printf("output2:0x%02x  output device state:%s   input2: 0x%02x input device state:%s"CRLF,
            g_deviceOutputData[1][0],
            ((g_deviceOutputState[1] == PNIO_S_GOOD) ? "good" : "bad "),
            g_deviceInputData[1][0],
            ((g_deviceInputState[1] == PNIO_S_GOOD) ? "good" : "bad ")
            );

    } while(key != (int)'q' && key != (int)'Q');

	//Deinitialisieren (Wechsel in Betriebszustand OFFLINE und abmelden der Baugruppe als IO-Controller)
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
/*-------------------------------------------------------------*/
/* this function initializes the IO-BASE and returns a handle  */
/* necessary for all subsequent calls to IO-BASE functions     */
/*-------------------------------------------------------------*/

PNIO_UINT32 Initialize(PNIO_UINT32 CP_INDEX,PNIO_UINT32* pdwHandle)
{

   
    PNIO_UINT32 dwErrorCode = PNIO_OK;

    /* Connect to Communication Processor and obtain a handle */

    dwErrorCode = PNIO_controller_open(
             /*in*/  CP_INDEX,                     /* index of communication processor      */
             /*in*/  PNIO_CEP_MODE_CTRL,           /* permission to change operation mode   */
             /*in*/  callback_for_ds_read_conf,    /* mandatory  callback                   */
             /*in*/  callback_for_ds_write_conf,   /* mandatory callback                    */
             /*in*/  0,                            /* alarm callback     - not used         */
             /*out*/ pdwHandle                     /* handle                                */
        );
    printf(" Device Handle dwHandle = %d " CRLF, *pdwHandle);
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

    dwErrorCode = ChangeAndWaitForPnioMode(dwHandle, PNIO_MODE_OFFLINE);
    if(dwErrorCode != PNIO_OK) {
        return;
    }
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
		if (dwErrorCode == 769) //dwErrorCode = 769 = Error 0x301 = Gerät nicht verbunden
		{ }
		else printf(CRLF"Error 0x%X in PNIO_data_write() Device %d "CRLF,dwErrorCode,i);
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
		{
			if (dwErrorCode == 769) //dwErrorCode = 769 = Error 0x301 = Gerät nicht verbunden
			{ }
			else printf(CRLF"Error 0x%X in PNIO_data_read() Device %d "CRLF,dwErrorCode,i);   
		}
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

    printf("callback_for_ds_read_conf " CRLF);
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
    printf("callback_for_ds_read_conf " CRLF);
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


/*************************************************************/
/* Get a character form console                              */
/*************************************************************/
int getCharWithTimeout()
{
    int key = 0;
#ifndef WIN32
    struct pollfd pollfd[1];
    static int init = 0;
    static struct termios termiosOld;
    static struct termios termios;

    if ( !init ) {
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
    if(_kbhit()) {
        key = _getch();
    }
#endif

    return key;
}

