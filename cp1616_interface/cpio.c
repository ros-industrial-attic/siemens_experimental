/*********************************************************************************************//**
* @file cpio.c
* 
* cp1616_interface testing node
* 
* Copyright {2015} {Frantisek Durovsky}
* 
*  Licensed under the Apache License, Version 2.0 (the "License");
*  you may not use this file except in compliance with the License.
*  You may obtain a copy of the License at

*      http://www.apache.org/licenses/LICENSE-2.0

*  Unless required by applicable law or agreed to in writing, software
*  distributed under the License is distributed on an "AS IS" BASIS,
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*  See the License for the specific language governing permissions and
*  limitations under the License.
   
* *********************************************************************************************/
#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <stdlib.h>

#include "pniousrx.h"
#include "pnioerrx.h"

//---------------------------------
//Global data for CP
//---------------------------------
int bDeviceReady       = 0;
PNIO_UINT32 g_dwCpId   = 1;
PNIO_UINT32 g_dwHandle = 0;
int semModChange       = 0;
volatile PNIO_MODE_TYPE g_currentMode = PNIO_MODE_OFFLINE;


//--------------------------------
//Global Data fo device
//--------------------------------
//INPUT DATA
const PNIO_UINT32 g_deviceInputCount = 3;	//number of input modules
volatile PNIO_IOXS g_localState = PNIO_S_GOOD;
volatile PNIO_IOXS g_deviceInputState[g_deviceInputCount]={PNIO_S_BAD, PNIO_S_BAD, PNIO_S_BAD};

PNIO_ADDR g_deviceInputAddress[g_deviceInputCount] = 
{
  { PNIO_ADDR_LOG, PNIO_IO_IN, {0}},   //output address of first output module 
  { PNIO_ADDR_LOG, PNIO_IO_IN, {1}},   //output address of second output module
  { PNIO_ADDR_LOG, PNIO_IO_IN, {2}}    //output address of third output module
};

PNIO_UINT32 g_deviceInputLength[g_deviceInputCount] = 
{
  1, //length in bytes of first input module
  1, //length in bytes of second input module
  1  //length in bytes of third input module
};

PNIO_UINT8 g_deviceInputData[g_deviceInputCount];

//OUTPUT DATA
const PNIO_UINT32 g_deviceOutputCount = 3;
volatile PNIO_IOXS g_deviceOutputState[g_deviceOutputCount] = {PNIO_S_BAD, PNIO_S_BAD, PNIO_S_BAD};

PNIO_ADDR g_deviceOutputAddress[g_deviceOutputCount] = 
{
  {PNIO_ADDR_LOG, PNIO_IO_OUT, {0}},   //output address of first output module
  {PNIO_ADDR_LOG, PNIO_IO_OUT, {1}},   //output address of second output module
  {PNIO_ADDR_LOG, PNIO_IO_OUT, {2}}    //output address of third output module
};

PNIO_UINT32 g_deviceOutputLength[g_deviceOutputCount] = 
{
  1,	//length in bytes of first output module
  1,   //length in bytes of second output module
  1    //length in bytes of third output module
};

PNIO_UINT8 g_deviceOutputData[g_deviceOutputCount];

//-------------------------------
//Function declarations
//-------------------------------

void callback_for_ds_read_conf(PNIO_CBE_PRM *pCbfPrm);              //mandatory callback
void callback_for_ds_write_conf(PNIO_CBE_PRM *pCbfPrm);             //mandatory callback
void callback_for_mode_change_indication(PNIO_CBE_PRM *pCbfPrm);    //not mandatory callback
void callback_for_device_activation(PNIO_CBE_PRM *pCbfPrm);
void callback_for_alarm_indication(PNIO_CBE_PRM *pCbfPrm);


//-------------------------------
//Function definitions
//-------------------------------
PNIO_UINT32 Initialize(PNIO_UINT32 CP_INDEX)
{
  PNIO_UINT32 dwHandle = 0;   //0 is invalid handle
  PNIO_UINT32 dwErrorCode = PNIO_OK;
   
  printf("Open PNIO_controller: ");
  
  //Connect to CP and obtain handle
  dwErrorCode = PNIO_controller_open(
    /*in*/ CP_INDEX,			   //Communication Module index
    /*in*/ PNIO_CEP_MODE_CTRL,            //permission to change operation mode
    /*in*/ callback_for_ds_read_conf,     //mandatory callback
    /*in*/ callback_for_ds_write_conf,    //mandatory callback
    /*in*/ callback_for_alarm_indication, //alarm callback
    /*in*/ &dwHandle			   //handle
  );
  
  
  //Check errors
  if(dwErrorCode != PNIO_OK)
  {
    printf("ERROR:  0x%x\n", (int)dwErrorCode);
    exit(1);  /*exit*/
  }
  printf("SUCCESS\n");
    
  //------------------------------------------------------------------------------
  //register the callback PNIO_CBE_MODE_IND for Mode changes confirmation
  dwErrorCode = PNIO_register_cbf(
    /*in*/ dwHandle,
    /*in*/ PNIO_CBE_MODE_IND,
    /*in*/ callback_for_mode_change_indication
  );
  
  //Check errors
  if(dwErrorCode != PNIO_OK)
  {
      printf("Error in PNIO_register_cbf:  0x%x\n", (int)dwErrorCode);
      PNIO_close(dwHandle);
      exit(1);
  }
  
  //------------------------------------------------------------------------------
  //register the callback PNIO_CBE_DEV_ACT for device activation confirmation
  dwErrorCode = PNIO_register_cbf(
    /*in*/ dwHandle,
    /*in*/ PNIO_CBE_DEV_ACT_CONF,
    /*in*/ callback_for_device_activation
  );
  
  if(dwErrorCode != PNIO_OK){
    printf("Error in PNIO_register_cbf:  0x%x\n", (int)dwErrorCode);
    PNIO_close(dwHandle);
    exit(1);
  }
  
  return dwHandle;
   
}

void ChangeAndWaitForPnioMode(PNIO_UINT32 dwHandle, PNIO_MODE_TYPE mode)
{
  PNIO_UINT32 dwErrorCode;
  printf("Change PNIO mode ");
  
  //set asynchronous mode
  dwErrorCode = PNIO_set_mode(dwHandle, mode);
  
  if(dwErrorCode != PNIO_OK){
    printf("ERROR: 0x%x \n", (int)dwErrorCode);
    PNIO_close(dwHandle);
    exit(1);
  };
    
    
  if(dwHandle == g_dwHandle) {
    
    
    //wait for new message in the list
    while(!semModChange){
      sleep(1);
    }
    semModChange = 0;
  
    //check if the current mode is correct
    if(g_currentMode != mode){
      printf("ERROR : recieved another mode\n");
    }
    else {
      printf("SUCCESS\n");
    }
   
  }
}

void UpdateCyclicOutputData(PNIO_UINT32 dwHandle)
{
  PNIO_UINT32 dwErrorCode;
  for(unsigned int i=0;i<g_deviceOutputCount;i++) {
      dwErrorCode=PNIO_data_write(
            /*in*/ dwHandle,                             //handle                            
            /*in*/ &(g_deviceOutputAddress[i]),          // pointer to device output address 
            /*in*/ g_deviceOutputLength[i],              // length in bytes of output        
            /*in*/ &(g_deviceOutputData[i]),             // pointer to output data           
            /*in*/ g_localState,                         // local status                     
            /*out*/(PNIO_IOXS*)&(g_deviceOutputState[i]) // remote status                    
            );
    }
    if(dwErrorCode != PNIO_OK){
      printf("Error in UpdateCyclicOutputData \n");
      printf("PNIO_write_data (PNIO_CBE_DEV_ACT_CONF,..) returned 0x%x\n", (int)dwErrorCode);
    }
}

void UpdateCyclicInputData(PNIO_UINT32 dwHandle)
{
  PNIO_UINT32 dwErrorCode;
  PNIO_UINT32 dwBytesReaded;
  for(unsigned int i=0;i<g_deviceInputCount;i++) {
        dwErrorCode=PNIO_data_read(
            /*in*/  dwHandle,                             //handle                           
            /*in*/  &g_deviceInputAddress[i],             // pointer to device input address 
            /*in*/  g_deviceInputLength[i],               // length in bytes of input        
            /*out*/ &dwBytesReaded,                       // number of bytes read            
            /*in*/  &g_deviceInputData[i],                // pointer to input data            
            /*in*/  g_localState,                         // local status                    
            /*out*/(PNIO_IOXS*)&(g_deviceInputState[i])   // remote status                   
            );
    }
    if(dwErrorCode != PNIO_OK){
      printf("Error in UpdateCyclicInputData \n");
      printf("PNIO_read_data (PNIO_CBE_DEV_ACT_CONF,..) returned 0x%x\n", (int)dwErrorCode);
    }
}


void UnInitialize(PNIO_UINT32 dwHandle)
{
  printf("Close PNIO_controller: ");
  PNIO_UINT32 dwErrorCode = PNIO_OK;
  
  dwErrorCode = PNIO_close(dwHandle);
  
  if(dwErrorCode != PNIO_OK)
  {
    printf("Error 0x%x\n", (int) dwErrorCode);
    exit(1);
  }
  printf("SUCCESS\n");
}


void callback_for_ds_read_conf(PNIO_CBE_PRM *pCbfPrm)
{
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

void callback_for_ds_write_conf(PNIO_CBE_PRM* pCbfPrm)
{
  /**************************************************************/
  /* Attention :                                                */
  /* this is a callback and must be returned as soon as possible */
  /* don't use any endless or time consuming functions          */
  /* e.g. exit() would be fatal                                 */
  /* defer all time consuming functionality to other threads    */
  /**************************************************************/
  printf("callback_for_ds_write_conf \n");
  printf("this callback must not occur in this sample application\n");
}

void callback_for_mode_change_indication(PNIO_CBE_PRM *pCbfPrm)
{
  /**************************************************************/
  /* Attention :                                                */
  /* this is a callback and must be returned as soon as possible*/
  /* don't use any endless or time consuming functions          */
  /* e.g. exit() would be fatal                                 */
  /* defer all time consuming functionality to other threads    */
  /**************************************************************/
        
  if(pCbfPrm->CbeType==PNIO_CBE_MODE_IND) /* Check callback type */
  {
    switch (pCbfPrm->ModeInd.Mode)
    {
    case PNIO_MODE_OFFLINE:	
	  printf("request: OFFLINE: " );
          g_currentMode = PNIO_MODE_OFFLINE;
          break;
    case PNIO_MODE_CLEAR:
          printf("request: CLEAR: ");
          g_currentMode = PNIO_MODE_CLEAR;
          break;
    case PNIO_MODE_OPERATE:
	  printf("request: OPERATE: " );
	  g_currentMode = PNIO_MODE_OPERATE;
	  break;
    default:
	  printf("Wrong mode selected: " );
	  break;
    };

    //printf("semModChange g_dwHandle\n");

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

int main(int argc, char *argv[])
{
  
  printf("Application does following: \n");
  printf("1. Initialize CP\n");
  printf("2. Change mode to OPERATE\n");
  printf("3. Update input and output data in loop\n");
  printf("4. Change mode to OFFLINE\n");
  printf("5. Uninitialize CP\n");
  printf("---------------------------------\n\n");
      
  //Initialize
  bDeviceReady = 0;
  g_dwHandle = Initialize(g_dwCpId);
    
  //Change mode to PNIO_MODE_OPERATE
  ChangeAndWaitForPnioMode(g_dwHandle, PNIO_MODE_OPERATE);
    
  //Update data 
  for(unsigned int i = 0; i < 10; i++)
  {
    UpdateCyclicOutputData(g_dwHandle);
    UpdateCyclicInputData(g_dwHandle);
    
    printf("output:0x%x  output device state:%s \n",
	   g_deviceOutputData[0],
           ((g_deviceOutputState[0]==PNIO_S_GOOD)?"good":"bad")
	   );
    
    printf("input:0x%x  input device state:%s \n",
	   g_deviceInputData[0],
           ((g_deviceInputState[0]==PNIO_S_GOOD)?"good":"bad")
	   );
    
    sleep(1);
       
  }

  //change current mode to offline  
  ChangeAndWaitForPnioMode(g_dwHandle, PNIO_MODE_OFFLINE);
    
  //UnInitialize
  UnInitialize(g_dwHandle);
    
  return(EXIT_SUCCESS);
}