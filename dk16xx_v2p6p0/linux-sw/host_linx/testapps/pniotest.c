/*------------------------------------------------------------------------*/
/*  Copyright (C) SIEMENS CORP., 2013 All rights reserved.*/
/*------------------------------------------------------------------------*/
/*                                                                        */
/*   Project           :                                                  */
/*   Filename          : PnioTest.c                                       */
/*                                                                        */
/*                                                                        */
/*                                                                        */
/*------------------------------------------------------------------------*/
/*   Description:                                                         */
/*               This is a sample application to demonstrate the IO-Base  */
/*               user interface. This sample does following steps         */
/*                   1. Initialize                                        */
/*                   2. Start device                                      */
/*                   3. do some IO Transfer                               */
/*                   4. Stop device                                       */
/*                   5. uninitialize                                      */
/*------------------------------------------------------------------------*/
/* Attention : Callbacks are running concurrent in other threads so all   */
/*             printf statements should be synchronized. But this sample  */
/*             application doesn't synchronize for simplicity  !          */
/*------------------------------------------------------------------------*/
/*****************************************************************************/
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

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>

#include "pniousrx.h"
#include "pnioerrx.h"

/*------------------------------------------------------------------------*/
/* Global Data for Communication Processor                                */
/*------------------------------------------------------------------------*/

int bDeviceReady = 0;

/* Index of CP given by project */
PNIO_UINT32 g_dwCpId             = 1;

PNIO_UINT32 g_dwHandle=0;

int semModChange = 0;


/* Don't forget! Callbacks are running in concurrent threads !             */
/* 'volatile PNIO_MODE_TYPE g_currentMode' statement is                   */
/* required to force the compiler to generate unoptimized memory access    */
/* to 'g_currentMode'.                                                    */
/*                                                                        */
/* Example                                                                */
/*  while(g_currentMode == PNIO_MODE_OFFLINE){...};                       */
/* will not be optimized                                                  */


volatile PNIO_MODE_TYPE g_currentMode   = PNIO_MODE_OFFLINE;


/*------------------------------------------------------------------------*/
/* Global Data for Device                                                 */
/*------------------------------------------------------------------------*/


volatile PNIO_IOXS  g_localState=PNIO_S_GOOD;

const PNIO_UINT32 g_deviceInputCount=3; /* number of input modules */

volatile PNIO_IOXS  g_deviceInputState[g_deviceInputCount]={ PNIO_S_BAD,PNIO_S_BAD,PNIO_S_BAD};


/*PNIO_ADDR  g_deviceInputAddress[] = */
/* input address of first  input module */
/* input address of second input module */
PNIO_ADDR  g_deviceInputAddress[g_deviceInputCount]=
{
        { PNIO_ADDR_LOG, PNIO_IO_IN, {0}},/* output address of first  output module */
        { PNIO_ADDR_LOG, PNIO_IO_IN, {1}}, /* output address of second  output module */
        { PNIO_ADDR_LOG, PNIO_IO_IN, {2}} /* output address of second  output module */
};


PNIO_UINT32 g_deviceInputLength[g_deviceInputCount]  =
  {
  1, /* length in bytes of first input module */
  1,  /* length in bytes of second input module */
  1
  };

PNIO_UINT8 g_deviceInputData[g_deviceInputCount];



const PNIO_UINT32 g_deviceOutputCount=3;/* number of output modules */
volatile PNIO_IOXS  g_deviceOutputState[g_deviceOutputCount]={ PNIO_S_BAD,PNIO_S_BAD,PNIO_S_BAD};

PNIO_ADDR  g_deviceOutputAddress[g_deviceOutputCount]=
  {
        { PNIO_ADDR_LOG, PNIO_IO_OUT, {0}},/* output address of first  output module */
        { PNIO_ADDR_LOG, PNIO_IO_OUT, {1}}, /* output address of second  output module */
        { PNIO_ADDR_LOG, PNIO_IO_OUT, {2}} /* output address of second  output module */
  };

PNIO_UINT32 g_deviceOutputLength[g_deviceOutputCount] =
  {
  1,/* length in byte of first output module */
  1, /* length in byte of second output module */
  1
  };

PNIO_UINT8 g_deviceOutputData[g_deviceOutputCount];

/*------------------------------------------------------------------------*/
/* mandatory callbacks                                                    */
/* but not used in this sample application                                */
/* because this example does no data set, read or write                    */
/*------------------------------------------------------------------------*/

void callback_for_ds_read_conf(PNIO_CBE_PRM *pCbfPrm);
void callback_for_ds_write_conf(PNIO_CBE_PRM *pCbfPrm);

/*-----------------------------------------------------------------------*/
/* useful but not mandatory callbacks                                   */
/*-----------------------------------------------------------------------*/

void callback_for_mode_change_indication(PNIO_CBE_PRM *pCbfPrm);
void callback_for_mode_change_indication2(PNIO_CBE_PRM *pCbfPrm);
void callback_for_device_activation(PNIO_CBE_PRM *pCbfPrm);
void callback_for_alarm_indication(PNIO_CBE_PRM *pCbfPrm);

/*-------------------------------------------------------------*/
/* this function sets operational mode               */
/*                                                             */
/* do not call before Initialize was called successfully       */
/* because it needs PNIO_CBE_MODE_IND callback to be registered*/
/*-------------------------------------------------------------*/

void ChangeAndWaitForPnioMode(PNIO_UINT32 dwHandle, PNIO_MODE_TYPE mode)
{
   PNIO_UINT32 dwErrorCode;

   /*setting  mode asynchronously                                 */
    dwErrorCode = PNIO_set_mode(dwHandle,mode);

   if(dwErrorCode != PNIO_OK){
    printf(" Error in ChangeAndWaitForPnioMode \n");
    printf(" PNIO_set_mode returned 0x%x\n",(int)dwErrorCode);
      PNIO_close(dwHandle);
    exit(1); /* exit */
   };

   /* wait for callback_for_mode_change_indication to be called. */
   /* callback_for_mode_change_indication sets g_currentMode     */
   printf("waiting for changing operation mode\n");
/*   while(g_currentMode != mode){
     sleep(1); // 1 sec
     printf(".");
   };
*/


    if(dwHandle == g_dwHandle) {
        //printf("waiting for changing operation mode g_dwHandle\n");

        /* wait for new msg in the list */
        while(!semModChange) {
            sleep(1);
        }
        semModChange = 0;

        if(g_currentMode != mode) {
            printf(" Error in ChangeAndWaitForPnioMode - g_dwHandle received another mode\n");
        }
    } else {
        printf(" Error Handle\n");
    }

   printf ("\n");
}

/*-------------------------------------------------------------*/
/* this function initializes the IO-BASE and returns a handle  */
/* necessary for all subsequent calls to IO-BASE functions     */
/*-------------------------------------------------------------*/

PNIO_UINT32 Initialize(PNIO_UINT32 CP_INDEX)
{

   PNIO_UINT32 dwHandle    = 0; /*0 is invalid handle*/
   PNIO_UINT32 dwErrorCode = PNIO_OK;

   /* Connect to Communication Processor and obtain a handle */
   /* char buff[200];*/

   dwErrorCode = PNIO_controller_open(
        /*in*/  CP_INDEX,                     /* index of communication processor      */
        /*in*/  PNIO_CEP_MODE_CTRL,           /* permission to change operation mode   */
        /*in*/  callback_for_ds_read_conf,    /* mandatory  callback                   */
        /*in*/  callback_for_ds_write_conf,   /* mandatory callback                    */
        /*in*/  callback_for_alarm_indication,/* alarm callback                        */
        /*out*/ &dwHandle                     /* handle                                */
    );


    printf( "-> 0: PNIO Test call PNIO_controller_open \n");
    if(dwErrorCode != PNIO_OK){
        printf(" Error in Initialize \n");
        printf(" PNIO_controller_open returned 0x%x\n",(int)dwErrorCode);
        exit(1); /* exit */
    }
    printf(" PNIO_controller_open -> PNIO_OK\n");


   /* here we register the callback                         */
   /* PNIO_CBE_MODE_IND    for Mode changes  confirmation   */
//     if(CP_INDEX == 0)
     {
            dwErrorCode = PNIO_register_cbf
            (
            /*in*/ dwHandle,
            /*in*/ PNIO_CBE_MODE_IND,
            /*in*/ callback_for_mode_change_indication
            );
        }

   if(dwErrorCode != PNIO_OK){
        printf(" Error in Initialize \n");
        printf(" PNIO_register_cbf (PNIO_CBE_MODE_IND,..)  returned 0x%x\n",(int)dwErrorCode);
        PNIO_close(dwHandle);
        exit(1); /* exit */
   }

   /* here we register the callback                         */
   /* PNIO_CBE_DEV_ACT     for device activation confirmation*/

       dwErrorCode = PNIO_register_cbf
     (
     /*in*/ dwHandle,
     /*in*/ PNIO_CBE_DEV_ACT_CONF,
     /*in*/ callback_for_device_activation
     );

   if(dwErrorCode != PNIO_OK){
        printf(" Error in Initialize \n");
        printf(" PNIO_register_cbf (PNIO_CBE_DEV_ACT_CONF,..)  returned 0x%x\n",(int)dwErrorCode);
        PNIO_close(dwHandle);
        exit(1); /* exit */
   }

    /* anil */
   /* here we change the mode to PNIO_MODE_OPERATE  */
   /*ChangeAndWaitForPnioMode(dwHandle,PNIO_MODE_OPERATE); */

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

//  ChangeAndWaitForPnioMode(dwHandle,PNIO_MODE_OFFLINE);
//  sleep(1);

  dwErrorCode = PNIO_close(dwHandle);

  if(dwErrorCode != PNIO_OK){
    printf(" Error in UnInitialize \n");
    printf(" PNIO_close returned 0x%x\n",(int) dwErrorCode);
    exit(1); /* exit */
    };
}


/*-------------------------------------------------------------*/
/* this function writes the cyclic output data to IO-BASE      */
/*                                                             */
/* parameters                                                  */
/*    handle         : handle to the communication processor   */
/*-------------------------------------------------------------*/

void  UpdateCyclicOutputData(PNIO_UINT32 dwHandle)
{
    PNIO_UINT32 dwErrorCode;
    for(unsigned int i=0;i<g_deviceOutputCount;i++) {
        dwErrorCode=PNIO_data_write(
            /*in*/ dwHandle,                   /*handle                            */
            /*in*/ &(g_deviceOutputAddress[i]),/* pointer to device output address */
            /*in*/ g_deviceOutputLength[i],    /* length in bytes of output        */
            /*in*/ &(g_deviceOutputData[i]),   /* pointer to output data            */
            /*in*/ g_localState,               /* local status                     */
            /*out*/(PNIO_IOXS*)&(g_deviceOutputState[i])  /* remote status                    */
            );
    }
}

/*-------------------------------------------------------------*/
/* this function reads the cyclic input data from IO-BASE      */
/*                                                             */
/* parameters                                                  */
/*    handle         : handle to the communication processor   */
/*-------------------------------------------------------------*/
void  UpdateCyclicInputData(PNIO_UINT32 dwHandle)
{
    PNIO_UINT32 dwErrorCode;
    PNIO_UINT32 dwBytesReaded;
    for(unsigned int i=0;i<g_deviceInputCount;i++) {
        dwErrorCode=PNIO_data_read(
            /*in*/  dwHandle,                /*handle                           */
            /*in*/  &g_deviceInputAddress[i],/* pointer to device input address */
            /*in*/  g_deviceInputLength[i],  /* length in bytes of input        */
            /*out*/ &dwBytesReaded,          /* number of bytes read            */
            /*in*/  &g_deviceInputData[i],   /* pointer to input data            */
            /*in*/  g_localState,      /* local status                    */
            /*out*/(PNIO_IOXS*)&(g_deviceInputState[i])/* remote status                   */
            );
    }
}

/*--------------------------------------------------*/
/* mandatory callbacks but not used in this sample  */
/*--------------------------------------------------*/

void callback_for_ds_read_conf(
                        PNIO_CBE_PRM *pCbfPrm
                        ){
  /**************************************************************/
  /* Attention :                                                */
  /* this is a callback and must be returned as soon as possible*/
  /* don't use any endless or time consuming functions          */
  /* e.g. exit() would be fatal                                 */
  /* defer all time consuming functionality to other threads    */
  /**************************************************************/

  printf("callback_for_ds_read_conf \n");
  printf("this callback must not occur in this sample application\n");
}


void callback_for_ds_write_conf(
                        PNIO_CBE_PRM* pCbfPrm
                        ){
  /**************************************************************/
  /* Attention :                                                */
  /* this is a callback and must be returned as soon as possible */
  /* don't use any endless or time consuming functions          */
  /* e.g. exit() would be fatal                                 */
  /* defer all time consuming functionality to other threads    */
  /**************************************************************/
  printf("callback_for_ds_read_conf \n");
  printf("this callback must not occur in this sample application\n");
}

/*--------------------------------------------------*/
/* useful callbacks                */
/*--------------------------------------------------*/

void callback_for_mode_change_indication(PNIO_CBE_PRM *pCbfPrm){
  /**************************************************************/
  /* Attention :                                                */
  /* this is a callback and must be returned as soon as possible*/
  /* don't use any endless or time consuming functions          */
  /* e.g. exit() would be fatal                                 */
  /* defer all time consuming functionality to other threads    */
  /**************************************************************/
    printf("callback_for_mode_change_indication was called\n");
  if(pCbfPrm->CbeType==PNIO_CBE_MODE_IND) /* Check callback type */
  {
    switch (pCbfPrm->ModeInd.Mode){
    case PNIO_MODE_OFFLINE:
        printf("callback_for_mode_change_indication called with PNIO_MODE_OFFLINE\n" );

      g_currentMode = PNIO_MODE_OFFLINE;
      break;
    case PNIO_MODE_CLEAR:
      g_currentMode = PNIO_MODE_CLEAR;
      break;
    case PNIO_MODE_OPERATE:
        printf("callback_for_mode_change_indication called with PNIO_MODE_OPERATE\n" );
      g_currentMode = PNIO_MODE_OPERATE;
      break;
    default:
      printf("callback_for_mode_change_indication called with wrong mode\n" );
    break;
    };

    printf("semModChange g_dwHandle\n");

    /*send notification */
    semModChange = 1;
  }
}

void callback_for_alarm_indication(PNIO_CBE_PRM *pCbfPrm){
  /**************************************************************/
  /* Attention :                                                */
  /* this is a callback and must be returned as soon as possible*/
  /* don't use any endless or time consuming functions          */
  /* e.g. exit() would be fatal                                 */
  /* defer all time consuming functionality to other threads    */
  /**************************************************************/

  if(pCbfPrm->CbeType==PNIO_CBE_ALARM_IND) /* Check callback type */
  {
    switch (pCbfPrm->AlarmInd.pAlarmData->AlarmType)
    {
      case PNIO_ALARM_DIAGNOSTIC:
        printf("PNIO_ALARM_DIAGNOSTIC\n");
          break;
      case PNIO_ALARM_PROCESS:
        printf("PNIO_ALARM_DIAGNOSTIC\n");
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
        printf("PNIO_ALARM_RETURN_OF_SUBMODULE \n");
        break;

      case PNIO_ALARM_DEV_FAILURE:
        printf("PNIO_ALARM_DEV_FAILURE\n");
        break;

      case PNIO_ALARM_DEV_RETURN:
        printf("PNIO_ALARM_DEV_RETURN\n");
        bDeviceReady = 1;
        break;
      default:
         printf("callback_for_alarm_indication called with wrong type\n" );
      break;
    }
  }
}


void callback_for_device_activation(PNIO_CBE_PRM *pCbfPrm)
{
  switch( pCbfPrm->DevActConf.Mode){
    case PNIO_DA_TRUE:
      printf ("device with Address %x was activated with result %x\n",
        (int)pCbfPrm->DevActConf.pAddr->u.Addr,
        (int)pCbfPrm->DevActConf.Result);
      break;
    case PNIO_DA_FALSE:
      printf ("device with Address %x was deactivated with result %x\n",
        (int)pCbfPrm->DevActConf.pAddr->u.Addr,
        (int)pCbfPrm->DevActConf.Result);
      break;
  };
}


int main(int argc, char* argv[])
{
  int i;
  int Schleife;
    /* create semaphore for notification */
  semModChange = 0;

  printf("\n----------------------------------------------------------\n");
  printf("This sample application   does following tasks\n");
  printf("1.increases the value of the first output byte\n once per second\n");
  printf("2.reads input byte once per second and displays it on screen\n");
  printf("3.writes all called callbacks like alarm callback on screen (e.g. (un)plugging of modules");


  for (Schleife=0;Schleife<3;Schleife++)
  {
    bDeviceReady=0;


  printf("PNIO Test : Initialize cp %d ...\n", (int)g_dwCpId);
  g_dwHandle = Initialize(g_dwCpId);

   /* here we change the mode to PNIO_MODE_OPERATE  */
  ChangeAndWaitForPnioMode(g_dwHandle,PNIO_MODE_OPERATE);


  i=0;
  do{

    g_deviceOutputData[0]++;

    UpdateCyclicOutputData(g_dwHandle);
    UpdateCyclicInputData(g_dwHandle);


    for (int j=0;j<3;j++)
    {
      g_deviceOutputState[j] = g_deviceInputState[j];
    }


    printf("output:0x%x  output device state:%s   input: 0x%x input device state:%s\n",
              g_deviceOutputData[0],
              ((g_deviceOutputState[0]==PNIO_S_GOOD)?"good":"bad"),
              g_deviceInputData[0],
              ((g_deviceInputState[0]==PNIO_S_GOOD)?"good":"bad")
              );
    if (!bDeviceReady)
      sleep(1);
    else
    {
      usleep(10000);
      i++;
    }
  } while (i<20);//( key!=(int)'q' && key!=(int)'Q');



  printf("PNIO Test : changemode OFFLINE - Handle 0x%x ...\n", (int)g_dwHandle);
  ChangeAndWaitForPnioMode(g_dwHandle,PNIO_MODE_OFFLINE);

  printf("PNIO Test : sleep 3 sec ....  \n");
  sleep(3);
  printf("UnInitialize cal cp %d ....\n", (int)g_dwCpId);
  UnInitialize(g_dwHandle);

} /* for */
  return 0;
}

