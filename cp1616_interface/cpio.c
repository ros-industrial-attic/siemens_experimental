/*********************************************************************************************//**
* @file cpio.c
* 
* cp1616_interface IO controller testing node
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

#define NUM_INPUT_MODULES 	4
#define NUM_OUTPUT_MODULES  	4

/************************************************/
/*Global data for CP                            */
/************************************************/
int bDeviceReady       = 0;
PNIO_UINT32 g_dwCpId   = 1;
PNIO_UINT32 g_dwHandle = 0;
int semModChange       = 0;
volatile PNIO_MODE_TYPE g_currentMode = PNIO_MODE_OFFLINE;
volatile PNIO_IOXS g_localState = PNIO_S_GOOD;


//INPUT Module memory allocation 
PNIO_UINT32 g_deviceInputCount = 0;	
PNIO_UINT32 *g_deviceInputLength = (PNIO_UINT32 *) malloc(NUM_INPUT_MODULES * sizeof(PNIO_UINT32));                          
PNIO_IOXS volatile * volatile g_deviceInputState = (PNIO_IOXS volatile *) malloc (NUM_INPUT_MODULES * sizeof(PNIO_IOXS));
PNIO_ADDR *g_deviceInputAddress = (PNIO_ADDR *) malloc(NUM_INPUT_MODULES * sizeof(PNIO_ADDR));

//OUTPUT Module - memory allocation 
PNIO_UINT32 g_deviceOutputCount = 0;
PNIO_UINT32 *g_deviceOutputLength = (PNIO_UINT32 *) malloc(NUM_OUTPUT_MODULES * sizeof(PNIO_UINT32));          
PNIO_IOXS volatile * volatile g_deviceOutputState = (PNIO_IOXS volatile *) malloc (NUM_OUTPUT_MODULES * sizeof(PNIO_IOXS));
PNIO_ADDR *g_deviceOutputAddress = (PNIO_ADDR *) malloc(NUM_OUTPUT_MODULES * sizeof(PNIO_ADDR));

//PNIO_UINT32 countData(PNIO_UINT32 *p, PNIO_UINT32 m){return(m)?p[--m]+countData(p,m):0;};	

//INPUT data variables
PNIO_UINT32 g_numOfInputData;
PNIO_UINT8 *g_deviceInputData;
PNIO_UINT8 **g_arrayOfInputData;

//OUTPUT data variables
PNIO_UINT32 g_numOfOutputData;
PNIO_UINT8 *g_deviceOutputData;
PNIO_UINT8 **g_arrayOfOutputData;

/************************************************/
/*Callbacks declarations                        */
/************************************************/

void callback_for_ds_read_conf(PNIO_CBE_PRM *pCbfPrm);              //mandatory callback
void callback_for_ds_write_conf(PNIO_CBE_PRM *pCbfPrm);             //mandatory callback
void callback_for_mode_change_indication(PNIO_CBE_PRM *pCbfPrm);    //not mandatory callback
void callback_for_device_activation(PNIO_CBE_PRM *pCbfPrm);
void callback_for_alarm_indication(PNIO_CBE_PRM *pCbfPrm);


/*********************************************************** */
/*                                                           */
/*Function:        Initialize()                              */
/*                                                           */
//************************************************************/
/* The function does the initialization of PNIO controller   */
/* registration of callbacks is part of initialization       */
/*************************************************************/

PNIO_UINT32 Initialize(PNIO_UINT32 CP_INDEX)
{
  PNIO_UINT32 dwHandle = 0;   //0 is invalid handle
  PNIO_UINT32 dwErrorCode = PNIO_OK;
   
  printf("Open PNIO_controller: ");
  
  //Connect to CP and obtain handle
  dwErrorCode = PNIO_controller_open(
    /*in*/ CP_INDEX,                      //Communication Module index
    /*in*/ PNIO_CEP_MODE_CTRL,            //permission to change operation mode
    /*in*/ callback_for_ds_read_conf,     //mandatory callback
    /*in*/ callback_for_ds_write_conf,    //mandatory callback
    /*in*/ callback_for_alarm_indication, //alarm callback
    /*in*/ &dwHandle);                    //handle
    
  
  //Check errors
  if(dwErrorCode != PNIO_OK)
  {
    printf("ERROR:  0x%x\n", (int)dwErrorCode);
    exit(1);  /*exit*/
  }
  else
    printf("SUCCESS\n");
    
  //------------------------------------------------------------------------------
  //register the callback PNIO_CBE_MODE_IND for Mode changes confirmation
  //------------------------------------------------------------------------------
  dwErrorCode = PNIO_register_cbf(
    /*in*/ dwHandle,
    /*in*/ PNIO_CBE_MODE_IND,
    /*in*/ callback_for_mode_change_indication);
  
  
  //Check errors
  if(dwErrorCode != PNIO_OK)
  {
      printf("Error in PNIO_register_cbf:  0x%x\n", (int)dwErrorCode);
      PNIO_close(dwHandle);
      exit(1);
  }
  
  //------------------------------------------------------------------------------
  //register the callback PNIO_CBE_DEV_ACT for device activation confirmation
  //------------------------------------------------------------------------------
  dwErrorCode = PNIO_register_cbf(
    /*in*/ dwHandle,
    /*in*/ PNIO_CBE_DEV_ACT_CONF,
    /*in*/ callback_for_device_activation
  );
  
  if(dwErrorCode != PNIO_OK)
  {
    printf("Error in PNIO_register_cbf:  0x%x\n", (int)dwErrorCode);
    PNIO_close(dwHandle);
    exit(1);
  }
  
  return dwHandle;
   
}

/*********************************************************** */
/*                                                           */
/*Function:        UnInitialize()                            */
/*                                                           */
//************************************************************/
/* The function uninitialize PNIO controller                 */
/*************************************************************/

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
  else
    printf("SUCCESS\n");
}

/*********************************************************** */
/*                                                           */
/*Function:      ChangeAndWaitForPnioMode()                  */
/*                                                           */
//************************************************************/
/* The function changes operational mode and waits for its   */
/* execution                                                 */
/*************************************************************/

void ChangeAndWaitForPnioMode(PNIO_UINT32 dwHandle, PNIO_MODE_TYPE mode)
{
  PNIO_UINT32 dwErrorCode;
  printf("Change PNIO mode ");
  
  //set asynchronous mode
  dwErrorCode = PNIO_set_mode(dwHandle, mode);
  
  if(dwErrorCode != PNIO_OK)
  {
    printf("ERROR: 0x%x \n", (int)dwErrorCode);
    PNIO_close(dwHandle);
    exit(1);
  };
    
    
  if(dwHandle == g_dwHandle) 
  {
    //wait for new message in the list
    while(!semModChange){
      sleep(1);
    }
    semModChange = 0;
  
    //check if the current mode is correct
    if(g_currentMode != mode)
      printf("ERROR : recieved another mode\n");
    else 
      printf("SUCCESS\n");
  }
}

/*********************************************************** */
/*                                                           */
/*Function:            AddIOModule()                         */
/*                                                           */
//************************************************************/
/* The function initialize input and output module variables */
/*************************************************************/
void AddIOModule(PNIO_UINT32 input_size, PNIO_UINT32 input_address, PNIO_UINT32 output_size, PNIO_UINT32 output_address)
{
  printf("Adding IO module: ");
  
  if(g_currentMode == PNIO_MODE_OPERATE)
  {
    printf("ERROR : not able to add IO module in OPERATE state\n");
  }
  if(g_deviceInputCount >= NUM_INPUT_MODULES)
  {
    printf("ERROR: Not able to add another module. Max count reached");
  }
  
  else
  {         
    g_deviceInputLength[g_deviceInputCount] = input_size;                    //number of telegram bytes
    g_deviceInputState[g_deviceInputCount] = PNIO_S_BAD;                     //initial Input state
    g_deviceInputAddress[g_deviceInputCount].AddrType = PNIO_ADDR_LOG;       //Address type
    g_deviceInputAddress[g_deviceInputCount].IODataType = PNIO_IO_IN;        //Data type
    g_deviceInputAddress[g_deviceInputCount].u.Addr = input_address;         //Module address
        
    g_deviceOutputLength[g_deviceOutputCount] = output_size;                 //number of telegram bytes
    g_deviceOutputState[g_deviceOutputCount] = PNIO_S_BAD;                   //initial Output state
    g_deviceOutputAddress[g_deviceOutputCount].AddrType = PNIO_ADDR_LOG;     //Address type
    g_deviceOutputAddress[g_deviceOutputCount].IODataType = PNIO_IO_OUT;     //Data type
    g_deviceOutputAddress[g_deviceOutputCount].u.Addr  = output_address;     //Module address
        
    printf("IN:  %d  %d \n", g_deviceInputLength[g_deviceInputCount], g_deviceInputAddress[g_deviceInputCount].u.Addr);
    printf("\t\t  OUT: %d  %d \n", g_deviceOutputLength[g_deviceOutputCount], g_deviceOutputAddress[g_deviceOutputCount].u.Addr);
    
    g_numOfInputData  += input_size;	//count total input size for AddIOModuleDataStructure() 
    g_numOfOutputData += output_size;  //count total output size for AddIOModuleDataStructure()
    
    g_deviceInputCount++;		//increment deviceInputCounter
    g_deviceOutputCount++;		//increment deviceOutputCounter
    }
}

/*********************************************************** */
/*                                                           */
/*Function:            RemoveIOModules()                     */
/*                                                           */
//************************************************************/
/* The function free allocated memory for all modules and    */
/* reset counters                                            */
/*************************************************************/
void RemoveIOModules()
{
  //free allocated memory
  free(g_deviceInputLength);
  free((PNIO_IOXS *)g_deviceInputState);
  free(g_deviceInputAddress);
  
  free(g_deviceOutputLength);
  free((PNIO_IOXS *)g_deviceOutputState);    
  free(g_deviceOutputAddress);
  
  //reset counters
  g_deviceInputCount = 0;
  g_deviceOutputCount = 0;

  
  
}

/*********************************************************** */
/*                                                           */
/*Function:          AddIOModuleDataStructure()              */
/*                                                           */
//************************************************************/
/* The function initialize data structures for input and     */
/* output variables of all IO modules. It should be called   */
/* right after AddIOModule() calls                           */
/*************************************************************/ 
void AddIOModuleDataStructure(void)
{
  //Memory allocation for 2D array of input data 
  g_deviceInputData = (PNIO_UINT8 *) calloc(g_numOfInputData, sizeof(PNIO_UINT8));
  g_arrayOfInputData = (PNIO_UINT8 **) malloc(g_numOfInputData*sizeof(PNIO_UINT8));
   
  unsigned int p1, p2;

  for(p1 = 0, p2 = 0 ; p2 < g_deviceInputCount;)
  { 
    g_arrayOfInputData[p2] = &(g_deviceInputData[p1]);
    p1 += g_deviceInputLength[p2++]; 
  }

  //Memory allocation for 2D array of output data 
  g_deviceOutputData = (PNIO_UINT8 *) calloc(g_numOfOutputData, sizeof(PNIO_UINT8));
  g_arrayOfOutputData = (PNIO_UINT8 **) malloc(g_numOfOutputData*sizeof(PNIO_UINT8));
  
  for(p1 = 0, p2 = 0 ; p2 < g_deviceOutputCount;)
  { 
    g_arrayOfOutputData[p2] = &(g_deviceOutputData[p1]);
    p1 += g_deviceOutputLength[p2++]; 
  }
}

/*********************************************************** */
/*                                                           */
/*Function:         RemoveModuleDataStructure()              */
/*                                                           */
//************************************************************/
/* The function removes input and output data variables      */
/*************************************************************/
void RemoveModuleDataStructure(void)
{
  free(g_deviceInputData);
  free(g_deviceOutputData);
  free(g_arrayOfInputData);
  free(g_arrayOfOutputData);

}

/*********************************************************** */
/*                                                           */
/*Function:      UpdateCyclicOutputData()                    */
/*                                                           */
//************************************************************/
/* The function writes output data to the process image      */
/* execution                                                 */
/*************************************************************/

void UpdateCyclicOutputData(PNIO_UINT32 dwHandle)
{
  PNIO_UINT32 dwErrorCode = PNIO_OK;
  for(unsigned int i=0;i<g_deviceOutputCount;i++)
  {
    dwErrorCode=PNIO_data_write(
            /*in*/ dwHandle,                               //handle                            
            /*in*/ &(g_deviceOutputAddress[i]),            // pointer to device output address 
            /*in*/ g_deviceOutputLength[i],                // length in bytes of output        
            /*in*/ &(g_deviceOutputData[i]),               // pointer to output data           
            /*in*/ g_localState,                           // local status                     
            /*out*/(PNIO_IOXS*)&(g_deviceOutputState[i])); // remote status                    
            
    if(dwErrorCode != PNIO_OK)
    {
      printf("Error in UpdateCyclicOutputData \n");
      printf("PNIO_write_data (PNIO_CBE_DEV_ACT_CONF,..) returned 0x%x\n", (int)dwErrorCode);
    }
  }
}


/*********************************************************** */
/*                                                           */
/*Function:      UpdateCyclicInputData()                     */
/*                                                           */
//************************************************************/
/* The function reads input data from the process image      */
/* execution                                                 */
/*************************************************************/

void UpdateCyclicInputData(PNIO_UINT32 dwHandle)
{
  PNIO_UINT32 dwErrorCode = PNIO_OK;
  PNIO_UINT32 dwBytesReaded;
  for(unsigned int i=0;i<g_deviceInputCount;i++) 
  {
    dwErrorCode=PNIO_data_read(
            /*in*/  dwHandle,                               //handle                           
            /*in*/  &g_deviceInputAddress[i],               // pointer to device input address 
            /*in*/  g_deviceInputLength[i],                 // length in bytes of input        
            /*out*/ &dwBytesReaded,                         // number of bytes read            
            /*in*/  &g_deviceInputData[i],                  // pointer to input data            
            /*in*/  g_localState,                           // local status                    
            /*out*/(PNIO_IOXS*)&(g_deviceInputState[i]));   // remote status                   
   
    if(dwErrorCode != PNIO_OK)
    {
      printf("Error in UpdateCyclicInputData \n");
      printf("PNIO_read_data (PNIO_CBE_DEV_ACT_CONF,..) returned 0x%x\n", (int)dwErrorCode);
    }
  }
}

/*********************************************************** */
/*                                                           */
/*Function:              PrintData()                         */
/*                                                           */
//************************************************************/
/* The function prints DataStructure to console               */
/*************************************************************/
void PrintData(void)
{
  
}





/*********************************************************** */
/*                                                           */
/*                   Callbacks definitions                   */
/*                                                           */
//************************************************************/
/* callacks must be returned as soon as possible             */
/* don't use any endless or time consuming functions         */
/* e.g. exit() would be fatal                                */
/* defer all time consuming functionality to other threads   */
/*************************************************************/

void callback_for_ds_read_conf(PNIO_CBE_PRM *pCbfPrm)
{
  printf("callback_for_ds_read_conf \n");
  printf("this callback must not occur in this sample application\n");
}

void callback_for_ds_write_conf(PNIO_CBE_PRM* pCbfPrm)
{
  printf("callback_for_ds_write_conf \n");
  printf("this callback must not occur in this sample application\n");
}

void callback_for_mode_change_indication(PNIO_CBE_PRM *pCbfPrm)
{
       
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

/*********************************************************** */
/*                                                           */
/*                           Main                            */
/*                                                           */
//************************************************************/

int main(int argc, char *argv[])
{
  printf("Application does following: \n");
  printf("1. Initialize CP\n");
  printf("2. Change mode to OPERATE\n");
  printf("3. Update input and output data in loop\n");
  printf("4. Change mode to OFFLINE\n");
  printf("5. Uninitialize CP\n");
  printf("---------------------------------\n\n");
      
  AddIOModule(16,512,16,512);
  AddIOModule(8,528,8,528);
 
  AddIOModuleDataStructure();
    
  
  g_arrayOfInputData[0][0]  = 0;	g_arrayOfInputData[0][1]  = 1;
  g_arrayOfInputData[0][2]  = 2;	g_arrayOfInputData[0][3]  = 3;
  g_arrayOfInputData[0][4]  = 4;	g_arrayOfInputData[0][5]  = 5;
  g_arrayOfInputData[0][6]  = 6;	g_arrayOfInputData[0][7]  = 7;
  g_arrayOfInputData[0][8]  = 8;	g_arrayOfInputData[0][9]  = 9;
  g_arrayOfInputData[0][10] = 10;	g_arrayOfInputData[0][11] = 11; 
  g_arrayOfInputData[0][12] = 12;	g_arrayOfInputData[0][13] = 13;
  g_arrayOfInputData[0][14] = 14;	g_arrayOfInputData[0][15] = 15;
   
  g_arrayOfInputData[1][0]  = 16;	g_arrayOfInputData[1][1]  = 17;
  g_arrayOfInputData[1][2]  = 18;	g_arrayOfInputData[1][3]  = 19;
  g_arrayOfInputData[1][4]  = 20;	g_arrayOfInputData[1][5]  = 21;
  g_arrayOfInputData[1][6]  = 22;	g_arrayOfInputData[1][7]  = 23;
  
  g_arrayOfOutputData[0][0]  = 0;	g_arrayOfOutputData[0][1]  = 1;
  g_arrayOfOutputData[0][2]  = 2;	g_arrayOfOutputData[0][3]  = 3;
  g_arrayOfOutputData[0][4]  = 4;	g_arrayOfOutputData[0][5]  = 5;
  g_arrayOfOutputData[0][6]  = 6;	g_arrayOfOutputData[0][7]  = 7;
  g_arrayOfOutputData[0][8]  = 8;	g_arrayOfOutputData[0][9]  = 9;
  g_arrayOfOutputData[0][10] = 10;	g_arrayOfOutputData[0][11] = 11; 
  g_arrayOfOutputData[0][12] = 12;	g_arrayOfOutputData[0][13] = 13;
  g_arrayOfOutputData[0][14] = 14;	g_arrayOfOutputData[0][15] = 15;
   
  g_arrayOfOutputData[1][0]  = 16;	g_arrayOfOutputData[1][1]  = 17;
  g_arrayOfOutputData[1][2]  = 18;	g_arrayOfOutputData[1][3]  = 19;
  g_arrayOfOutputData[1][4]  = 20;	g_arrayOfOutputData[1][5]  = 21;
  g_arrayOfOutputData[1][6]  = 22;	g_arrayOfOutputData[1][7]  = 23;
 
   
  RemoveIOModules();
  RemoveModuleDataStructure();
  
/*  //Initialize
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
*/
 
  return(EXIT_SUCCESS);
}