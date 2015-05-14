/*------------------------------------------------------------------------*/
/* Copyright (C) SIEMENS CORP., 2013 All rights reserved.*/
/*------------------------------------------------------------------------*/
/*                                                                        */
/*   Project           :                                                  */
/*   Filename          : pnioeasy_irt1.cpp                                */
/*                                                                        */
/*                                                                        */
/*                                                                        */
/*------------------------------------------------------------------------*/
/*   Description:                                                         */
/*               This is a sample application to demonstrate the IO-Base  */
/*               user interface. This sample does following steps         */
/*                   1. Initialize                                        */
/*                   2. IRT IO read and write                             */
/*                   3. Stop device                                       */
/*                   4. uninitialize                                      */
/*------------------------------------------------------------------------*/
/* Attention : Callbacks are running concurrent in other threads so all   */
/*             printf statements should be synchronized. But this sample  */
/*             application doesn't synchronize for simplicity  !          */
/*------------------------------------------------------------------------*/
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

#include "pniousrx.h"
#include "pnioerrx.h"
#include "pniobase.h"

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
volatile PNIO_MODE_TYPE g_currentMode    = PNIO_MODE_OFFLINE;


/* Global variables to count callback events */
volatile PNIO_UINT32 g_OpFaultCount      = 0;
volatile PNIO_UINT32 g_StartOpCount      = 0;
volatile PNIO_UINT32 g_DataExchangeCount = 0;

/* Global variables to control the behavoir of the DataExchange function */
volatile int g_data                      = 0;
volatile bool g_set                      = true;
volatile bool g_mirror                   = false;
volatile bool g_increment                = false;
void (*g_DataExchange) (PNIO_UINT32)     = 0;

volatile PNIO_UINT32 g_readErrors        = 0;
volatile PNIO_UINT32 g_writeErrors       = 0;
volatile PNIO_UINT32 g_badRemoteStatus   = 0;

/*------------------------------------------------------------------------*/
/* Global Data for Device                                                 */
/*------------------------------------------------------------------------*/

volatile PNIO_IOXS g_localState = PNIO_S_GOOD;

typedef struct {
    PNIO_ADDR address;
    PNIO_UINT32 length;
    volatile PNIO_IOXS status;
    volatile PNIO_UINT8 *data;
} device;

device g_InputDevice[] = {
    /* address                        length rem status  data */
    {{PNIO_ADDR_LOG, PNIO_IO_IN, 0}, 1, PNIO_S_BAD, NULL},
    {{PNIO_ADDR_LOG, PNIO_IO_IN, 1}, 4, PNIO_S_BAD, NULL}
};

const PNIO_UINT32 g_deviceInputCount = sizeof (g_InputDevice) / sizeof (device);        // number of input modules

device g_OutputDevice[] = {
    /* address                         length rem status  data */
    {{PNIO_ADDR_LOG, PNIO_IO_OUT,    0},   1, PNIO_S_BAD, NULL},
    {{PNIO_ADDR_LOG, PNIO_IO_OUT,    1},   4, PNIO_S_BAD, NULL}
};

const PNIO_UINT32 g_deviceOutputCount = sizeof (g_OutputDevice) / sizeof (device);      // number of output modules


/*======================================================================================================*/


/*------------------------------------------------------------------------*/
/* mandatory callbacks                                                    */
/* but not used in this sample application                                */
/* because this example does no data set, read or write                   */
/*------------------------------------------------------------------------*/

void callback_for_ds_read_conf(PNIO_CBE_PRM * pCbfPrm);
void callback_for_ds_write_conf(PNIO_CBE_PRM * pCbfPrm);


/*------------------------------------------------------------------------*/
/* optional callbacks                                                     */
/*------------------------------------------------------------------------*/

void callback_for_mode_change_indication(PNIO_CBE_PRM * pCbfPrm);
void callback_for_alarm_indication(PNIO_CBE_PRM * pCbfPrm);


/*------------------------------------------------------------------------*/
/* callbacks for IRT function                                             */
/*------------------------------------------------------------------------*/

void callback_for_startop_indication(PNIO_CP_CBE_PRM * prm);
void callback_for_opfault_indication(PNIO_CP_CBE_PRM * prm);


/*------------------------------------------------------------------------*/
/* forward declaration of helper functions                                */
/*------------------------------------------------------------------------*/

PNIO_UINT32 Initialize(PNIO_UINT32 CP_INDEX);
void UnInitialize(PNIO_UINT32 dwHandle);
void DataExchange(PNIO_UINT32 dwHandle);
void UpdateCyclicOutputData(PNIO_UINT32 dwHandle);
void UpdateCyclicInputData(PNIO_UINT32 dwHandle);
int getCharWithTimeout(void);
void ChangeAndWaitForPnioMode(PNIO_UINT32 dwHandle, PNIO_MODE_TYPE mode);

/*------------------------------------------------------------------------*/


/*******************************************************************/
/* DataExchange                                                    */
/*******************************************************************/
/* This function is called in the StartOP handler                  */
/*******************************************************************/
void DataExchange(PNIO_UINT32 dwHandle)
{
    ++g_DataExchangeCount;

    UpdateCyclicInputData(dwHandle);

    if(g_set) {
        memset((void *)g_OutputDevice[0].data, g_data, g_OutputDevice[0].length);
        g_set = false;
    }

    if(g_mirror)
        if(g_OutputDevice[0].length == g_InputDevice[0].length)
            memcpy((void *)g_OutputDevice[0].data, (void *)g_InputDevice[0].data,
                g_InputDevice[0].length);

    if(g_increment)
        if(g_OutputDevice[1].length == sizeof (PNIO_UINT32))
            ++ * (PNIO_UINT32 *) g_OutputDevice[1].data;

    UpdateCyclicOutputData(dwHandle);
}


/*******************************************************************/
/* IRT Start OP                                                    */
/*******************************************************************/
/* This callback is called at the start of IRT cycle               */
/*******************************************************************/
void callback_for_startop_indication(PNIO_CP_CBE_PRM * prm)
{
    ++g_StartOpCount;

    if(g_DataExchange) {
        /* process data */
        g_DataExchange(prm->u.StartOp.AppHandle);
    }

    /* signal that data processing is done for this cycle */
    PNIO_CP_set_opdone(prm->u.StartOp.AppHandle, NULL);
}


/************************************************************************/
/* IRT OPFault_ind                                                      */
/************************************************************************/
/* This callback is called if OPdone is not called in time of IRT cycle */
/************************************************************************/
void callback_for_opfault_indication(PNIO_CP_CBE_PRM * prm)
{
    /* Opfault has come */
    ++g_OpFaultCount;
}


/*-------------------------------------------------------------*/
/* this function sets operational mode                         */
/*                                                             */
/* do not call before Initialize was called successfully       */
/* because it needs PNIO_CBE_MODE_IND callback to be registered*/
/*-------------------------------------------------------------*/
void ChangeAndWaitForPnioMode(PNIO_UINT32 dwHandle, PNIO_MODE_TYPE mode)
{
    PNIO_UINT32 dwErrorCode;

    /* setting  mode asynchronously */
    dwErrorCode = PNIO_set_mode(dwHandle, mode);

    if(dwErrorCode != PNIO_OK) {
        printf(" Error in ChangeAndWaitForPnioMode\n");
        printf(" PNIO_set_mode returned 0x%lx\n", dwErrorCode);
        PNIO_close(dwHandle);
        exit(1);                // exit
    }

    /* wait for callback_for_mode_change_indication to be called. */
    /* callback_for_mode_change_indication sets g_currentMode     */
    printf("waiting for changing operation mode\n");
    while(g_currentMode != mode) {
        Sleep(100);
        printf(".");
        fflush(stdout);
    }

    printf(CRLF);
}

void usage()
{
    printf("m - toggle mirroring of 1. IN data to 1. OUT data\n");
    printf("i - toggle increment of 2. OUT data (interpreted as unsigned long value)\n");
    printf("s - set value of 1. OUT data (1 byte)\n");
    printf("l - list all IN and OUT modules\n");
    printf("c - list counters for StartOp, DataExchange and OpFault\n");
    printf("h - print this help\n");
    printf("q - stop data exchange and quit application\n");
}

/*------------------------------------------------------------------------*/
/* Here starts the application code                                       */
/*------------------------------------------------------------------------*/
int main(int argc, char *argv[])
{
    PNIO_UINT32 dwHandle = 0;
    int key = 0;

    printf("This sample application does following tasks\n");
    printf("1. Initialize the controller\n");
    printf("2. Register for OpFault and StartOp callbacks\n");
    printf("3. Set mode to OPERATE\n");
    printf("4. Do IO read / write on StartOp indication\n\n\n");

    printf("Press 's' to start sample application\n");

    do {
        key = getCharWithTimeout();
        if(key == (int)'q' || key == (int)'Q')
            return 0;

    } while(key != (int)'s' && key != (int)'S');


    dwHandle = Initialize(g_dwCpId);

    // TODO wait until all devices are online

    g_DataExchange = DataExchange;
    usage();

    do {
        PNIO_UINT32 i = 0, j = 0;

        key = getCharWithTimeout();
        //printf("KEY = %c" CRLF, key);
        switch (key) {
        case 'm':              // toggle mirror
            g_mirror = !g_mirror;
            printf("mirror is %s\n", g_mirror ? "on" : "off");
            break;

        case 'i':              // toggle increment
            g_increment = !g_increment;
            printf("increment is %s\n", g_increment ? "on" : "off");
            break;

        case 'h':              // print help
            usage();
            break;

        case 's':              // set data
            printf("data is %d (0x%x) change to --> ", g_data, g_data);
            scanf("%i", &g_data);
            printf("new data is %d (0x%x)\n", g_data, g_data);
            g_set = true;
            break;

        case 'l':              // list device data
            printf("\nmodule  address length remote state data\n");
            for(i = 0; i < g_deviceInputCount; ++i) {
                printf("%2d. IN  %6d %6d %12s ",
                    i + 1, g_InputDevice[i].address.u.Addr, g_InputDevice[i].length,
                    g_InputDevice[i].status == PNIO_S_GOOD ? "good" : "bad");
                for(j = 0; j < g_InputDevice[i].length; ++j)
                    printf(" %02x", g_InputDevice[i].data[j]);
                printf("\n");
            }

            for(i = 0; i < g_deviceOutputCount; ++i) {
                printf("%2d. OUT %6d %6d %12s ",
                    i + 1, g_OutputDevice[i].address.u.Addr, g_OutputDevice[i].length,
                    g_OutputDevice[i].status == PNIO_S_GOOD ? "good" : "bad");
                for(j = 0; j < g_OutputDevice[i].length; ++j)
                    printf(" %02x", g_OutputDevice[i].data[j]);
                printf("\n");
            }
            break;

        case 'c':              // show stats
            printf("Statistics: %8lu StartOp callbacks, %8lu DataExchange calls, %8lu OpFault callbacks\n",
                   g_StartOpCount, g_DataExchangeCount, g_OpFaultCount);
            printf("            %8lu Data write errors, %8lu Data read errors,   %8lu Bad remote status\n",
                g_writeErrors, g_readErrors, g_badRemoteStatus);
            break;

        default:
            break;
        }
    } while(key != (int)'q' && key != (int)'Q');

    g_DataExchange = 0;

    UnInitialize(dwHandle);

    printf("Statistics: %8lu StartOp callbacks, %8lu DataExchange calls, %8lu OpFault callbacks\n",
           g_StartOpCount, g_DataExchangeCount, g_OpFaultCount);
    printf("            %8lu Data write errors, %8lu Data read errors,   %8lu Bad remote status\n",
        g_writeErrors, g_readErrors, g_badRemoteStatus);

    return 0;
}


/*-------------------------------------------------------------*/
/* this function initializes the IO-BASE and returns a handle  */
/* necessary for all subsequent calls to IO-BASE functions     */
/*-------------------------------------------------------------*/
PNIO_UINT32 Initialize(PNIO_UINT32 CP_INDEX)
{
    PNIO_UINT32 dwHandle = 0;   /* 0 is invalid handle */
    PNIO_UINT32 dwErrorCode = PNIO_OK;
    int i = 0;

    /* allocate memory for input data */
    for(i = 0; i < g_deviceInputCount; ++i) {
        g_InputDevice[i].data = new PNIO_UINT8[g_InputDevice[i].length];
        if(!g_InputDevice[i].data) {
            printf("Could not allocate memory (%d bytes) for g_InputData[%d]\n",
                g_InputDevice[i].length, i);
        } else {
            memset((void *)g_InputDevice[i].data, 0, g_InputDevice[i].length);
        }
    }

    /* allocate memory for output data */
    for(i = 0; i < g_deviceOutputCount; ++i) {
        g_OutputDevice[i].data = new PNIO_UINT8[g_OutputDevice[i].length];
        if(!g_OutputDevice[i].data) {
            printf("Could not allocate memory (%d bytes) for g_OutputData[%d]\n",
                g_OutputDevice[i].length, i);
        } else {
            memset((void *)g_OutputDevice[i].data, 0, g_OutputDevice[i].length);
        }
    }

    /* Connect to Communication Processor and obtain a handle */

    dwErrorCode = PNIO_controller_open(
            /*in*/  CP_INDEX,                     /* index of communication processor      */
            /*in*/  PNIO_CEP_MODE_CTRL,           /* permission to change operation mode   */
            /*in*/  callback_for_ds_read_conf,    /* mandatory  callback                   */
            /*in*/  callback_for_ds_write_conf,   /* mandatory callback                    */
            /*in*/  callback_for_alarm_indication,/* alarm callback                        */
            /*out*/ &dwHandle);                   /* handle                                */

    if(dwErrorCode != PNIO_OK) {
        printf("Error in Initialize\n");
        printf("PNIO_controller_open returned 0x%08x\n", dwErrorCode);
        exit(1);                // exit
    }

    /*Register the IRT callbacks */
    dwErrorCode = PNIO_CP_register_cbf(dwHandle, PNIO_CP_CBE_OPFAULT_IND, callback_for_opfault_indication);

    if(dwErrorCode != PNIO_OK) {
        /* Error */
        printf("\n\t Error while registering OpFault callback function , check if you are logged in as root\n");
        PNIO_close(dwHandle);
        exit(1);                // exit
    }

    dwErrorCode = PNIO_CP_register_cbf(dwHandle, PNIO_CP_CBE_STARTOP_IND, callback_for_startop_indication);

    if(dwErrorCode != PNIO_OK) {
        /* Error */
        printf("\n\t Error while registering StartOP callback function , check if you are logged in as root\n");
        PNIO_close(dwHandle);
        exit(1);                // exit
    }


    /* here we register the callback                         */
    /* PNIO_CBE_MODE_IND    for Mode changes  confirmation   */
    dwErrorCode = PNIO_register_cbf(
        /*in */ dwHandle,
        /*in */ PNIO_CBE_MODE_IND,
        /*in */ callback_for_mode_change_indication);

    if(dwErrorCode != PNIO_OK) {
        printf(" Error in Initialize\n");
        printf(" PNIO_register_cbf (PNIO_CBE_MODE_IND,..)  returned 0x%08x\n", dwErrorCode);
        PNIO_close(dwHandle);
        exit(1);                // exit
    }

    /* here we change the mode to PNIO_MODE_OPERATE  */
    ChangeAndWaitForPnioMode(dwHandle, PNIO_MODE_OPERATE);

    return dwHandle;
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
    int i = 0;

    /* here we change the mode to PNIO_MODE_OFFLINE  */
    ChangeAndWaitForPnioMode(dwHandle,PNIO_MODE_OFFLINE);

    dwErrorCode = PNIO_close(dwHandle);

    if(dwErrorCode != PNIO_OK) {
        printf("Error in UnInitialize\n");
        printf("PNIO_close returned 0x%08x\n", dwErrorCode);
        exit(1);                // exit
    }

    /* free memory for input data */
    for(i = 0; i < g_deviceInputCount; ++i) {
        if(g_InputDevice[i].data) {
            delete[]g_InputDevice[i].data;
            g_InputDevice[i].data = NULL;
        }
    }

    /* free memory for output data */
    for(i = 0; i < g_deviceOutputCount; ++i) {
        if(g_OutputDevice[i].data) {
            delete[]g_OutputDevice[i].data;
            g_OutputDevice[i].data = NULL;
        }
    }
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
    for(int i = 0; i < g_deviceOutputCount; i++) {
        dwErrorCode = PNIO_data_write(
            /*in*/ dwHandle,                   /*handle                            */
            /*in*/ &g_OutputDevice[i].address, /* pointer to device output address */
            /*in*/ g_OutputDevice[i].length,   /* length in bytes of output        */
            /*in*/ (PNIO_UINT8*)g_OutputDevice[i].data, /* pointer to output data  */
            /*in*/ g_localState,               /* local status                     */
            /*out*/(PNIO_IOXS*)&g_OutputDevice[i].status); /* remote status        */

        if(dwErrorCode != PNIO_OK)
            ++g_writeErrors;
        else if(g_OutputDevice[i].status == PNIO_S_BAD)
            ++g_badRemoteStatus;
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
    for(int i = 0; i < g_deviceInputCount; i++) {
        dwErrorCode = PNIO_data_read(
            /*in*/  dwHandle,                 /*handle                           */
            /*in*/  &g_InputDevice[i].address,/* pointer to device input address */
            /*in*/  g_InputDevice[i].length,  /* length in bytes of input        */
            /*out*/ &dwBytesReaded,           /* number of bytes read            */
            /*in*/  (PNIO_UINT8*)g_InputDevice[i].data, /* pointer to input data */
            /*in*/  g_localState,             /* local status                    */
            /*out*/ (PNIO_IOXS*)&g_InputDevice[i].status);/* remote status       */

        if(dwErrorCode != PNIO_OK)
            ++g_readErrors;
        else if(g_InputDevice[i].status == PNIO_S_BAD)
            ++g_badRemoteStatus;
    }
}

/*--------------------------------------------------*/
/* mandatory callbacks but not used in this sample  */
/*--------------------------------------------------*/
void callback_for_ds_read_conf(PNIO_CBE_PRM * pCbfPrm)
{
    /***************************************************************/
    /* Attention :                                                 */
    /* this is a callback and must be returned as soon as possible */
    /* don't use any endless or time consuming functions           */
    /* e.g. exit() would be fatal                                  */
    /* defer all time consuming functionality to other threads     */
    /***************************************************************/

    printf("callback_for_ds_read_conf\n");
    printf("this callback must not occur in this sample application\n");
}

void callback_for_ds_write_conf(PNIO_CBE_PRM * pCbfPrm)
{
    /***************************************************************/
    /* Attention :                                                 */
    /* this is a callback and must be returned as soon as possible */
    /* don't use any endless or time consuming functions           */
    /* e.g. exit() would be fatal                                  */
    /* defer all time consuming functionality to other threads     */
    /***************************************************************/
    printf("callback_for_ds_write_conf\n");
    printf("this callback must not occur in this sample application\n");
}

/*************************************************************/
/* Get a Character from console                              */
/*************************************************************/
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
        termios.c_lflag &= ~ICANON;
        termios.c_lflag &= ~ECHO;
        termios.c_lflag &= ~ISIG;
        termios.c_oflag |= ONLCR | OPOST;
        termios.c_cc[VMIN] = 0;
        termios.c_cc[VTIME] = 0;
        ++init;
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
    /* this is a callback and must be returned as soon as possible */
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
            printf("callback_for_mode_change_indication called with wrong mode\n");
            break;
        }
    }

    printf("callback_for_mode_change_indication was called (new mode is %d)\n", g_currentMode);
}

/*-------------------------------------------------------------*/
/* this function will be called from IO-BASE to signal that    */
/* a alarm has been received                                   */
/*                                                             */
/* parameters                                                  */
/*    pCbfPrm         : Callback information                   */
/*-------------------------------------------------------------*/
void callback_for_alarm_indication(PNIO_CBE_PRM * pCbfPrm)
{
    /**************************************************************/
    /* Attention :                                                */
    /* this is a callback and must be returned as soon as possible */
    /* don't use any endless or time consuming functions          */
    /* e.g. exit() would be fatal                                 */
    /* defer all time consuming functionality to other threads    */
    /**************************************************************/

    /* Check if correct callback type */
    if(pCbfPrm->CbeType == PNIO_CBE_ALARM_IND) {
        switch (pCbfPrm->AlarmInd.pAlarmData->AlarmType) {
        case PNIO_ALARM_DIAGNOSTIC:
            printf("PNIO_ALARM_DIAGNOSTIC\n");
            break;

        case PNIO_ALARM_PROCESS:
            printf("PNIO_ALARM_PROCESS\n");
            break;

        case PNIO_ALARM_PULL:
            printf("PNIO_ALARM_PULL\n");
            break;

        case PNIO_ALARM_PLUG:
            printf("PNIO_ALARM_PLUG\n");
            break;

        case PNIO_ALARM_STATUS:
            printf("PNIO_ALARM_STATUS\n");
            break;

        case PNIO_ALARM_UPDATE:
            printf("PNIO_ALARM_UPDATE\n");
            break;

        case PNIO_ALARM_REDUNDANCY:
            printf("PNIO_ALARM_REDUNDACY\n");
            break;

        case PNIO_ALARM_CONTROLLED_BY_SUPERVISOR:
            printf("PNIO_ALARM_CONTROLLED_BY_SUPERVISOR\n");
            break;

        case PNIO_ALARM_RELEASED_BY_SUPERVISOR:
            printf("PNIO_ALARM_RELEASED_BY_SUPERVISOR\n");
            break;

        case PNIO_ALARM_PLUG_WRONG:
            printf("PNIO_ALARM_PLUG_WRONG\n");
            break;

        case PNIO_ALARM_RETURN_OF_SUBMODULE:
            printf("PNIO_ALARM_RETURN_OF_SUBMODULE\n");
            break;

        case PNIO_ALARM_DEV_FAILURE:
            printf("PNIO_ALARM_DEV_FAILURE\n");
            break;

        case PNIO_ALARM_DEV_RETURN:
            printf("PNIO_ALARM_DEV_RETURN\n");
            break;
        default:
            printf("callback_for_alarm_indication called with unknown type\n");
            break;
        }
    }
}
