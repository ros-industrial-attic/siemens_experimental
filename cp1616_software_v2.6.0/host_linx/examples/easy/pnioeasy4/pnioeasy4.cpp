/*---------------------------------------------------------------------------*/
/* Copyright (C) SIEMENS CORP., 2013 All rights reserved.*/
/*---------------------------------------------------------------------------*/
/*                                                                           */
/*   Project           :                                                     */
/*   Filename          : pnioeasy4.cpp                                       */
/*                                                                           */
/*                       example of analog modules                           */
/*                                                                           */
/*---------------------------------------------------------------------------*/
/*   Description:                                                            */
/*               This is a sample application to demonstrate the IO-Base     */
/*               user interface. This sample does following steps            */
/*                   1. Initialize (with the use of change mode indication   */
/*                                  callback)                                */
/*                   2. Aktivieren bzw. Deaktivieren des ersten IO-Gerätes   */
/*                      by pressing 'a' and 'd'                              */
/*                   3. Printig alarm information                            */
/*                   4. Do some IO Transfer on analog modules                */
/*                   5. Stop device                                          */
/*                   6. uninitialize                                         */
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


#include "pniousrx.h"
#include "pnioerrx.h"

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
const char roll[]={'-','\\','|','/','-','\\','|','/'};

volatile bool cp_stop_req = false;
/*------------------------------------------------------------------------*/
/* Global Data for Device                                                 */
/*------------------------------------------------------------------------*/


volatile PNIO_IOXS g_localState = PNIO_S_GOOD;

#define DEVICE_INPUT_COUNT 2       // number of input modules

volatile PNIO_IOXS g_deviceInputState[DEVICE_INPUT_COUNT] = { PNIO_S_BAD, PNIO_S_BAD };

PNIO_ADDR g_deviceInputAddress[DEVICE_INPUT_COUNT] = {
    {PNIO_ADDR_LOG, PNIO_IO_IN, 512}, // input address of first  input module
    {PNIO_ADDR_LOG, PNIO_IO_IN, 516}  // input address of second input module - not used for data io at this code
};

PNIO_UINT32 g_deviceInputLength[DEVICE_INPUT_COUNT] = {
    4,                          // length in bytes of first input module
    4                           // length in bytes of second input module
};
#define MAX_INPUT_LEN 4   //length of largest input module
PNIO_UINT8 g_deviceInputData[DEVICE_INPUT_COUNT][MAX_INPUT_LEN]={0}; 

#define DEVICE_OUTPUT_COUNT  2      // number of output modules
volatile PNIO_IOXS g_deviceOutputState[DEVICE_OUTPUT_COUNT] = { PNIO_S_BAD, PNIO_S_BAD };

PNIO_ADDR g_deviceOutputAddress[DEVICE_OUTPUT_COUNT] = {
    {PNIO_ADDR_LOG, PNIO_IO_OUT, 512}, // output address of first  output module
    {PNIO_ADDR_LOG, PNIO_IO_OUT, 516}  // output address of second  output module  - not used for data io at this code
};

PNIO_UINT32 g_deviceOutputLength[DEVICE_OUTPUT_COUNT] = {
    4,                          // length in byte of first output module
    4                           // length in byte of second output module
};
#define MAX_OUTPUT_LEN 4   //length of largest output module
PNIO_UINT8 g_deviceOutputData[DEVICE_OUTPUT_COUNT][MAX_OUTPUT_LEN]={0};  

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

PNIO_UINT32 Initialize(PNIO_UINT32 CP_INDEX,PNIO_UINT32 *pdwHandle);
void UnInitialize(PNIO_UINT32 dwHandle);
void UpdateCyclicOutputData(PNIO_UINT32 dwHandle);
void UpdateCyclicInputData(PNIO_UINT32 dwHandle);
int getCharWithTimeout(void);
void GetSystemTimeString(char *tstring);

/*------------------------------------------------------------------------*/
/* all about analoge modules - defines and declarations                   */
/*                                                                        */
/*  Remark:                                                               */
/*  the analog values have to be converted to the SIMATIC S7 format       */
/*  before and after PNIO transfer.                                       */
/*  This format is pending on the used analog modules (MLFB - number) and */
/*  the configured Output/Input range.                                    */
/*  For more information about the implementation of the convert_xx()     */
/*  functions see the manual of the ET200S                                */
/*                                                                        */
/*                                                                        */
/*                                                                        */
/*------------------------------------------------------------------------*/

//analog value range
#define OVERFLOW 2
#define OVERCONTROL 1             //overshoot
#define NORMALRANGE 0             //rated range
#define UNDERCONTROL -1           //undershoot
#define UNDERFLOW -2


char* get_Range_Text(int bereich);
int get_Range_from_Value_m10V_p10V(short einheit);
int get_Range_from_Value_p1V_p5V(short einheit);
double convert_AI__m10V_p10V(short value,char* bereich); //convert analog value from SIMATIC S7 format to double
short convert_AO__m10V_p10V(double value,char* bereich); //convert analog value from double to SIMATIC S7 format 

double convert_AI__p1V_p5V(short value,char* bereich); //not used at this example
short convert_AO__p1V_p5V(double value,char* bereich); //not used at this example


/*------------------------------------------------------------------------*/
/* Here starts the Application Code                                       */
/*------------------------------------------------------------------------*/

int main(void)
{

    PNIO_UINT32 dwHandle = 0;
    int key = 0;
    int r;
    char time_strs[13];
    PNIO_IOXS tmpOutState = PNIO_S_BAD,tmpInState=PNIO_S_BAD;
    short sao[2],sai[2];    //analog input and output values in converted SIMATIC S7-Format
    double dao[2],dai[2];   //analog input and output values in double format
    char acao[2][100],acai[2][100];  //analog input and output range description in string format e.g. overflow

    printf("This sample application does following tasks" CRLF);
    printf("1.increases/decreases the two analoge values of the" CRLF);
    printf("  analoge output and input modules once per 100 milliseconds" CRLF);
    printf("2.writes/reads input byte once per 100 milliseconds and displays it on screen" CRLF);
    printf("3.displays all called callbacks like alarm callback on screen" CRLF);
    printf("  (e.g. (un)plugging of modules" CRLF);
    printf("4.if you press key 'd' first device will be deactivated" CRLF);
    printf("5.if you press key 'a' first device will be activated" CRLF);

    printf(CRLF CRLF" Press 's' to start sample application" CRLF);
    printf("To stop sample application press 'q'." CRLF);

    do {
        key = getCharWithTimeout();
        if(key == (int)'q' || key == (int)'Q')
            return 0;
    } while(key != (int)'s' && key != (int)'S');

    r=0;
    

    if(PNIO_OK != Initialize(g_dwCpId, &dwHandle))
        return 0;
    dao[0]=-15;
    dao[1]=15;
    do {

            
        //analoge module 
        dao[0]+=0.25;
        if(dao[0]>15)
            dao[0]=-15;

        dao[1]-=0.25;
        if(dao[1]<-15)
            dao[1]=15;
        
        //convert double to short in SIMATIC S7.Format
        sao[0]=convert_AO__m10V_p10V(dao[0],acao[0]);
        sao[1]=convert_AO__m10V_p10V(dao[1],acao[1]);

        //convert to PNIO data (SIMATIC S7-Format with big endian byte order)
        g_deviceOutputData[0][0]=(PNIO_UINT8)((sao[0] & 0xFF00)>>8);
        g_deviceOutputData[0][1]=(PNIO_UINT8)(sao[0] & 0x00FF);

        g_deviceOutputData[0][2]=(PNIO_UINT8)((sao[1] & 0xFF00)>>8);
        g_deviceOutputData[0][3]=(PNIO_UINT8)(sao[1] & 0x00FF);
            
        UpdateCyclicOutputData(dwHandle);
        UpdateCyclicInputData(dwHandle);
        
        //convert to SIMATIC S7-Format 
        sai[0]=(short)((g_deviceInputData[0][0]*0x100)+g_deviceInputData[0][1]);
        sai[1]=(short)((g_deviceInputData[0][2]*0x100)+g_deviceInputData[0][3]);

        //convert to double analog value
        dai[0]=convert_AI__m10V_p10V(sai[0],acai[0]);
        dai[1]=convert_AI__m10V_p10V(sai[1],acai[1]);

        if((tmpOutState!=g_deviceOutputState[0])||(tmpInState!=g_deviceInputState[0]))
        {
            GetSystemTimeString(time_strs);
            printf(CRLF"PNIO state change at %s"CRLF,time_strs);
             /* in case of any state change begin new line */
        }

        printf("%c *out0:%+07.3f %s *out1:%+07.3f %s state:%s -- *in0:%+07.3f %s  *in1:%+07.3f %s  state:%s \r",
			    roll[r++],
                dao[0],acao[0],dao[1],acao[1],
                ((g_deviceOutputState[0] == PNIO_S_GOOD) ? "good" : "bad "),
                dai[0],acai[0],dai[1],acai[1],
                ((g_deviceInputState[0] == PNIO_S_GOOD) ? "good" : "bad ")
                );

        fflush(stdout);
        if(r==sizeof(roll))
            r=0;
        tmpOutState=g_deviceOutputState[0];
        tmpInState=g_deviceInputState[0];

        key = getCharWithTimeout();

        switch (key) {
        case 'd':
            PNIO_device_activate(dwHandle, &g_deviceInputAddress[0], PNIO_DA_FALSE);
            key = ' ';
            break;
        case 'a':
            PNIO_device_activate(dwHandle, &g_deviceInputAddress[0], PNIO_DA_TRUE);
            key = ' ';
            break;

        }

    } while(key != (int)'q' && key != (int)'Q' && !cp_stop_req );


    UnInitialize(dwHandle);

    exit (0);
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
    /* PNIO_CBE_CP_STOP_REQ     for device activation confirmation */

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
    //if(dwErrorCode != PNIO_OK)
        //printf(CRLF"Error 0x%X in PNIO_data_write() Device %d "CRLF,dwErrorCode,i);
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
    //if(dwErrorCode != PNIO_OK)
        //printf(CRLF"Error 0x%X in PNIO_data_read() Device %d "CRLF,dwErrorCode,i);   
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
    printf("this callback must not occur in this sample application" CRLF);
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
    printf("this callback must not occur in this sample application" CRLF);
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
/* a alarm has been received                                   */
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
            break;

        case PNIO_ALARM_DEV_RETURN:
            printf(CRLF"PNIO_ALARM_DEV_RETURN at %s" CRLF, time_str);
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

    /**************************************************************/
    /* Attention :                                                */
    /* this is a callback and must be returned as soon as possible*/
    /* don't use any endless or time consuming functions          */
    /* e.g. exit() would be fatal                                 */
    /* defer all time consuming functionality to other threads    */
    /**************************************************************/
    char time_str[13];
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

void callback_for_cp_stop_req(PNIO_CBE_PRM *pCbfPrm)
{
    printf (CRLF"CP_STOP_REQUEST received, terminating Programm ..."CRLF);
    cp_stop_req = true;
};
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
    if(_kbhit()) {
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
    timeline = ctime( & ( timebuffer.time ) );


    sprintf(tstring,"%.8s.%03u", &timeline[11], timebuffer.millitm);


    return;
}

/***************************************************************************/
/* all about analoge modules - implementation                              */
/***************************************************************************/


char* get_Range_Text(int bereich)
{
    switch(bereich)
    {
    case OVERFLOW:
        return("OFlow ");
    case OVERCONTROL:
        return("OShoot");
    case NORMALRANGE:
        return("ratedR");
    case UNDERCONTROL:
        return("UShoot");
    case UNDERFLOW:
        return("UFlow ");
    default:
        return("!range");
    }
}

int get_Range_from_Value_m10V_p10V(short val)
{
    if(val>32511)
        return(OVERFLOW);
    if(val>27648)
        return(OVERCONTROL);
    if(val>-27649)
        return(NORMALRANGE);
    if(val>=-32512)
        return(UNDERCONTROL);
    return(UNDERFLOW);
}


int get_Range_from_Value_p1V_p5V(short val)
{
    if(val>32511)
        return(OVERFLOW);
    if(val>27648)
        return(OVERCONTROL);
    if(val>-1)
        return(NORMALRANGE);
    if(val>=-4864)
        return(UNDERCONTROL);
    return(UNDERFLOW);
}

/***************************************************************************/
/* Remark: the following convertion functions are adapted from the manual  */
/*         in an "easy to reconstruct" style                               */
/*         linear function y=f(x)=Const1*x+Const2                          */
/*         for better performance you should change the implementation like*/
/*         (y1-y0)/(x1-x0))*(x-x0)+y0    to    Const1*x+Const2             */
/***************************************************************************/




double convert_AI__m10V_p10V(short value,char* bereich)
{
    double x=value;
    const double y0=-10,x0=-27648,y1=10,x1=27648;

    if(bereich)
    {
        strcpy(bereich,get_Range_Text(get_Range_from_Value_m10V_p10V(value)));
    }
    return(((y1-y0)/(x1-x0))*(x-x0)+y0);
}
double convert_AI__p1V_p5V(short value,char* bereich)
{
    double x=value;  //from short to double
    const double y0=1,x0=0,y1=5,x1=27648;
    if(bereich)
    {
        strcpy(bereich,get_Range_Text(get_Range_from_Value_p1V_p5V(value)));
    }
    return(((y1-y0)/(x1-x0))*(x-x0)+y0);
}



short convert_AO__p1V_p5V(double value,char* bereich)
{
    double y;
    const double y0=0,x0=1,y1=27648,x1=5;
    short res;
    
    if(value<0)
    {
        //Unterlauf - underflow
        res=(unsigned short)0xE4FE;
        if(bereich)
            strcpy(bereich,get_Range_Text(get_Range_from_Value_p1V_p5V(res)));
    }
    else
    {
        if(value>5.7)
        {
            //Ueberlauf - overflow
            res=0x7F00;
            if(bereich)
                strcpy(bereich,get_Range_Text(get_Range_from_Value_p1V_p5V(res)));
        }
        else
        {
            y=((y1-y0)/(x1-x0))*(value-x0)+y0;
            res=(short int)y;
            //VZ+12bit presicion
            res=res & 0xFFF8;
            if(bereich)
                strcpy(bereich,get_Range_Text(get_Range_from_Value_p1V_p5V(res)));
        }
    }
    return(res);
}

short convert_AO__m10V_p10V(double value,char* bereich)
{
    double y;
    const double y0=-27648,x0=-10,y1=27648,x1=10;
    short res;
    // 

    if(value<-11.7589)
    {
        //Unterlauf - underflow
        res=(unsigned short)0x80FF;
        if(bereich)
            strcpy(bereich,get_Range_Text(get_Range_from_Value_m10V_p10V(res)));
    }
    else
    {
        if(value>11.7589)
        {
            //Ueberlauf - overflow
            res=0x7F00;
            if(bereich)
                strcpy(bereich,get_Range_Text(get_Range_from_Value_m10V_p10V(res)));
        }
        else
        {
            y=((y1-y0)/(x1-x0))*(value-x0)+y0;
            res=(short int)y;
            //VZ+13bit presicion
            res=res & 0xFFFC;
            if(bereich)
                strcpy(bereich,get_Range_Text(get_Range_from_Value_m10V_p10V(res)));
        }
    }
    return(res);
}


