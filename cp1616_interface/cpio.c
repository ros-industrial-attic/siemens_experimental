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

#define NUM_INPUT_MODULES 		1
#define NUM_OUTPUT_MODULES  		1
#define MAX_NUM_OF_INIT_ATTEMPTS	1000

/******************************************************************************
 * Global data for CP                            
*******************************************************************************/
int g_DeviceReady      = 0;
int g_SemModChange     = 0;
PNIO_UINT32 g_dwCpId   = 1;
PNIO_UINT32 g_dwHandle = 0;

volatile PNIO_MODE_TYPE g_currentMode = PNIO_MODE_OFFLINE;
volatile PNIO_IOXS      g_localState  = PNIO_S_GOOD;
volatile bool           g_cpStopReq   = false;

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

/******************************************************************************
 * Callbacks declarations                        
 * ***************************************************************************/

void callback_for_ds_read_conf(PNIO_CBE_PRM *pCbfPrm);              //mandatory callback
void callback_for_ds_write_conf(PNIO_CBE_PRM *pCbfPrm);             //mandatory callback
void callback_for_mode_change_indication(PNIO_CBE_PRM *pCbfPrm);    //not mandatory callback
void callback_for_device_activation(PNIO_CBE_PRM *pCbfPrm);         //not mandatory callback	
void callback_for_alarm_indication(PNIO_CBE_PRM *pCbfPrm);          //not mandatory callback
void callback_for_cp_stop_req(PNIO_CBE_PRM *pCbfPrm);               //not mandatory callback

/******************************************************************************
 *                                                            
 * Function:        Initialize()                              
 *                                                           
 ******************************************************************************
 * The function does the initialization of PNIO controller   
 * registration of callbacks is part of initialization       
 *****************************************************************************/

PNIO_UINT32 Initialize(PNIO_UINT32 CP_INDEX)
{
  PNIO_UINT32 dwHandle = 0;               //0 is invalid handle
  PNIO_UINT32 dwErrorCode = PNIO_OK;      
   
  printf("Openning PNIO_controller: ");
  
  //Connect to CP and obtain handle
  dwErrorCode = PNIO_controller_open(
    /*in*/ CP_INDEX,                      //Communication Module index
    /*in*/ PNIO_CEP_MODE_CTRL,            //permission to change operation mode
    /*in*/ callback_for_ds_read_conf,     //mandatory callback
    /*in*/ callback_for_ds_write_conf,    //mandatory callback
    /*in*/ callback_for_alarm_indication, //alarm callback
   /*out*/ &dwHandle);                    //handle
    
  
  //Check errors
  if(dwErrorCode != PNIO_OK)
  {
    printf("ERROR:  0x%x\n", (int)dwErrorCode);
    exit(1);  /*exit*/
  }
  else
    printf("SUCCESS\n");
    
  //---------------------------------------------------------------------------
  //register the callback PNIO_CBE_MODE_IND for Mode changes confirmation
  //---------------------------------------------------------------------------
  dwErrorCode = PNIO_register_cbf(
    /*in*/ dwHandle,
    /*in*/ PNIO_CBE_MODE_IND,
    /*in*/ callback_for_mode_change_indication);
  
  
  //Check errors
  if(dwErrorCode != PNIO_OK)
  {
      printf("Error in PNIO_register_cbf: callback_for_mode_change_indication: 0x%x\n", (int)dwErrorCode);
      PNIO_close(dwHandle);
      exit(1);
  }
  
  //---------------------------------------------------------------------------
  //register the callback PNIO_CBE_DEV_ACT for device activation confirmation
  //---------------------------------------------------------------------------
  dwErrorCode = PNIO_register_cbf(
    /*in*/ dwHandle,
    /*in*/ PNIO_CBE_DEV_ACT_CONF,
    /*in*/ callback_for_device_activation);
  
  if(dwErrorCode != PNIO_OK)
  {
    printf("Error in PNIO_register_cbf: callback_for_device_activation: 0x%x\n", (int)dwErrorCode);
    PNIO_close(dwHandle);
    exit(1);
  }
  
  //---------------------------------------------------------------------------
  //register the callback PNIO_CBE_CP_STOP_REQ to stop the device 
  //---------------------------------------------------------------------------
  dwErrorCode = PNIO_register_cbf(
        /*in */ dwHandle,
        /*in */ PNIO_CBE_CP_STOP_REQ,
        /*in */ callback_for_cp_stop_req);

    if(dwErrorCode != PNIO_OK) 
    {
      printf("Error in PNIO_register_cbf: callback_for_cp_stop_req: 0x%x\n", (int)dwErrorCode);
      PNIO_close(dwHandle);
      exit(1);
    }
  
  return dwHandle;
   
}

/******************************************************************************
 *                                                           
 *Function:        UnInitialize()                            
 *                                                           
 ******************************************************************************
 * The function uninitialize PNIO controller                 
 *****************************************************************************/

void UnInitialize(PNIO_UINT32 dwHandle)
{
  printf("Closing PNIO_controller: ");
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

/****************************************************************************** 
 *                                                           
 * Function:      ChangeAndWaitForPnioMode()                  
 *                                                           
 ******************************************************************************
 * The function changes operational mode of CP and waits for its execution                                                 
 ******************************************************************************/

void ChangeAndWaitForPnioMode(PNIO_UINT32 dwHandle, PNIO_MODE_TYPE mode)
{
  PNIO_UINT32 dwErrorCode;
  printf("Changing PNIO mode: ");
  
  //set required mode
  dwErrorCode = PNIO_set_mode(dwHandle, mode);
  
  if(dwErrorCode != PNIO_OK)
  {
    printf("ERROR: 0x%x \n", (int)dwErrorCode);
    PNIO_close(dwHandle);
    exit(1);
  };
    
    
  if(dwHandle == g_dwHandle) 
  {
    //wait for a callback_for_mode_change_indication
    while(!g_SemModChange){
      sleep(1);
    }
    g_SemModChange = 0;
  
    //check if the current mode is correct
    if(g_currentMode != mode)
    {
      printf("ERROR : recieved another mode\n");
      exit(1);
    }
    else 
      printf("SUCCESS\n");
  }
}

/******************************************************************************                                                           
 * 
 * Function:            AddOutputModule()                     
 *                                                           
 ******************************************************************************
 * The function initialize output module variables.          
 ******************************************************************************/
void AddOutputModule(PNIO_UINT32 output_size, PNIO_UINT32 output_address)
{
  printf("Adding Output module: ");
  
  if(g_currentMode == PNIO_MODE_OPERATE)
  {
    printf("ERROR : not able to add Output module in OPERATE state\n");
    exit(1);
  }
  if(g_deviceInputCount >= NUM_INPUT_MODULES)
  {
    printf("ERROR: Not able to add another module. Max count reached");
    exit(1);
  }  
  else
  {         
    g_deviceOutputLength[g_deviceOutputCount] = output_size;                 //number of transfer bytes
    g_deviceOutputState[g_deviceOutputCount] = PNIO_S_BAD;                   //initial Output state
    g_deviceOutputAddress[g_deviceOutputCount].AddrType = PNIO_ADDR_LOG;     //Address type
    g_deviceOutputAddress[g_deviceOutputCount].IODataType = PNIO_IO_OUT;     //Data type
    g_deviceOutputAddress[g_deviceOutputCount].u.Addr  = output_address;     //Module address

    printf("OUT: Size: %02d  Q: %d - %d \n",
           g_deviceOutputLength[g_deviceOutputCount], 
           g_deviceOutputAddress[g_deviceOutputCount].u.Addr, 
           g_deviceOutputAddress[g_deviceOutputCount].u.Addr + g_deviceOutputLength[g_deviceOutputCount] - 1);
   
    g_numOfOutputData += output_size;  //total output size for AddModuleDataStructure()
    g_deviceOutputCount++;		//increment deviceOutputCounter
  }
}

/****************************************************************************** 
 *                                                           
 * Function:            AddInputModule()                      
 *                                                           
 ******************************************************************************
 * The function initialize input module variables            
 ******************************************************************************/
void AddInputModule(PNIO_UINT32 input_size, PNIO_UINT32 input_address)
{
  printf("Adding IO module: ");
  
  if(g_currentMode == PNIO_MODE_OPERATE)
  {
    printf("ERROR : not able to add Input module in OPERATE state\n");
  }
  if(g_deviceInputCount >= NUM_INPUT_MODULES)
  {
    printf("ERROR: Not able to add another module. Max count reached");
  }  
  else
  {         
    g_deviceInputLength[g_deviceInputCount] = input_size;                    //number of transfer bytes
    g_deviceInputState[g_deviceInputCount] = PNIO_S_BAD;                     //initial Input state
    g_deviceInputAddress[g_deviceInputCount].AddrType = PNIO_ADDR_LOG;       //Address type
    g_deviceInputAddress[g_deviceInputCount].IODataType = PNIO_IO_IN;        //Data type
    g_deviceInputAddress[g_deviceInputCount].u.Addr = input_address;         //Module address
         
   printf("IN:  Size: %02d  I: %d - %d \n",
          g_deviceInputLength[g_deviceInputCount], 
          g_deviceInputAddress[g_deviceInputCount].u.Addr,
          g_deviceInputAddress[g_deviceInputCount].u.Addr + g_deviceInputLength[g_deviceInputCount] - 1);
   
   g_numOfInputData  += input_size;	//total input size for AddIOModuleDataStructure() 
   g_deviceInputCount++;		//increment deviceInputCounter
  }
}

/******************************************************************************
 *                                                           
 * Function:            RemoveIOModules()                     
 *                                                           
 ******************************************************************************
 * The function is called to free allocated memory for all IO modules      
 ******************************************************************************/
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

/******************************************************************************
 *                                                           
 * Function:        AddInputModuleDataStructure()            
 *                                                           
 ******************************************************************************
 * The function initialize data structures for input variables for all input 
 * modules. It should be called after all Input modules are added by means of 
 * AddInputModule() function                             
 *****************************************************************************/ 
void AddInputModuleDataStructure(void)
{
  printf("Input memory:  I %d - %d  [Total size: %d bytes]\n\n", 
	 g_deviceInputAddress[0].u.Addr,
	 g_deviceInputAddress[g_deviceInputCount-1].u.Addr + g_deviceInputLength[g_deviceInputCount-1] - 1,
	 g_numOfInputData);
  
  //Memory allocation for 2D array of input data 
  g_deviceInputData = (PNIO_UINT8 *) calloc(g_numOfInputData, sizeof(PNIO_UINT8));
  g_arrayOfInputData = (PNIO_UINT8 **) malloc(g_numOfInputData*sizeof(PNIO_UINT8));
   
  unsigned int p1, p2;

  for(p1 = 0, p2 = 0 ; p2 < g_deviceInputCount;)
  { 
    g_arrayOfInputData[p2] = &(g_deviceInputData[p1]);
    p1 += g_deviceInputLength[p2++]; 
  }
}
/****************************************************************************** 
 *                                                           
 * Function:          AddOutputModuleDataStructure()          
 *                                                           
 ******************************************************************************
 * The function initialize data structures for output variables of all output
 * modules. It should be called after all Output modules are added by menas of
 * AddOutputModule() function                             
 *****************************************************************************/ 
void AddOutputModuleDataStructure(void)
{
  printf("Output memory: Q %d - %d  [Total size: %d bytes]\n\n", 
	 g_deviceOutputAddress[0].u.Addr,
	 g_deviceOutputAddress[g_deviceOutputCount-1].u.Addr + g_deviceOutputLength[g_deviceOutputCount-1] - 1,
	 g_numOfOutputData);
  
  //Memory allocation for 2D array of output data 
  g_deviceOutputData = (PNIO_UINT8 *) calloc(g_numOfOutputData, sizeof(PNIO_UINT8));
  g_arrayOfOutputData = (PNIO_UINT8 **) malloc(g_numOfOutputData*sizeof(PNIO_UINT8));
  
  unsigned int p1, p2;
  
  for(p1 = 0, p2 = 0 ; p2 < g_deviceOutputCount;)
  { 
    g_arrayOfOutputData[p2] = &(g_deviceOutputData[p1]);
    p1 += g_deviceOutputLength[p2++]; 
  }
}

/****************************************************************************** 
 *                                                           
 * Function:         RemoveModuleDataStructure()              
 * 
 ******************************************************************************
 * The function removes all input and output data structures      
 ******************************************************************************/
void RemoveModuleDataStructure(void)
{
  printf("Removing IO Module Data structures: ");
  free(g_deviceInputData);
  free(g_deviceOutputData);
  free(g_arrayOfInputData);
  free(g_arrayOfOutputData);
  printf("SUCCESS\n");
}

/******************************************************************************
 *                                                           
 * Function:      UpdateCyclicOutputData()                    
 *                                                           
*******************************************************************************
 * The function writes cyclic output data to the IO Base                                                 
 *****************************************************************************/

PNIO_UINT32 UpdateCyclicOutputData(PNIO_UINT32 dwHandle)
{
  PNIO_UINT32 dwErrorCode = PNIO_OK;
  unsigned int i;
  
  for(i=0;i<g_deviceOutputCount;i++)
  {
    dwErrorCode=PNIO_data_write(
            /*in*/ dwHandle,                               // handle                            
            /*in*/ &(g_deviceOutputAddress[i]),            // pointer to device output address 
            /*in*/ g_deviceOutputLength[i],                // length in bytes of output        
            /*in*/ g_arrayOfOutputData[i],                 // pointer to output data           
            /*in*/ g_localState,                           // local status                     
            /*out*/(PNIO_IOXS*)&(g_deviceOutputState[i])); // remote status                    
            
#ifdef DEBUG
    if(dwErrorCode != PNIO_OK)
    {
      printf("Error in UpdateCyclicOutputData \n");
      printf("PNIO_write_data (PNIO_CBE_DEV_ACT_CONF,..) returned 0x%x\n", (int)dwErrorCode);
    }
#endif
  }
  return(dwErrorCode);
}

/****************************************************************************** 
 *                                                           
 * Function:      UpdateCyclicInputData()                     
 *                                                           
 ******************************************************************************
 * The function reads cyclic input data from from IO Base                                                 
 *****************************************************************************/

PNIO_UINT32 UpdateCyclicInputData(PNIO_UINT32 dwHandle)
{
  PNIO_UINT32 dwErrorCode = PNIO_OK;
  PNIO_UINT32 dwBytesReaded;
  
  unsigned int i;
  
  for(i=0;i<g_deviceInputCount;i++) 
  {
    dwErrorCode=PNIO_data_read(
            /*in*/  dwHandle,                               // handle                           
            /*in*/  &g_deviceInputAddress[i],               // pointer to device input address 
            /*in*/  g_deviceInputLength[i],                 // length in bytes of input        
            /*out*/ &dwBytesReaded,                         // number of bytes read            
            /*in*/  g_arrayOfInputData[i],                  // pointer to input data            
            /*in*/  g_localState,                           // local status                    
            /*out*/(PNIO_IOXS*)&(g_deviceInputState[i]));   // remote status                   
 
#ifdef DEBUG
    if(dwErrorCode != PNIO_OK)
    {
      printf("Error in UpdateCyclicInputData \n");
      printf("PNIO_read_data (PNIO_CBE_DEV_ACT_CONF,..) returned 0x%x\n", (int)dwErrorCode);
    }
#endif
  }
  return(dwErrorCode);
}

/******************************************************************************
 *                                                           
 *Function:          PrintInputData()                        
 *                                                           
 ******************************************************************************
 * The function prints arrayOfInputData to the terminal       
 *****************************************************************************/
void PrintInputData(void)
{
  PNIO_UINT32 i;
  PNIO_UINT32 j;
  
  for(i = 0; i < g_deviceInputCount; i++)
  {
    printf("Input module  %u [I: %u - %u]",
	   i,
	   g_deviceInputAddress[i].u.Addr, 
	   g_deviceInputAddress[i].u.Addr + g_deviceInputLength[i]-1);
    
    for(j = 0; j < g_deviceInputLength[i]; j++)
    {
      printf(" %02x", g_arrayOfInputData[i][j]);
    }
    printf("\n");
  }
}

/****************************************************************************** 
 *
 *Function:          PrintOutputData()                       
 *                                                           
 *****************************************************************************
 * The function prints arrayOfOutputData to the terminal        
 ****************************************************************************/
void PrintOutputData(void)
{
  PNIO_UINT32 i;
  PNIO_UINT32 j;
  
  for(i = 0; i < g_deviceOutputCount; i++)
  {
    printf("Output module %u [Q: %u - %u]", 
	   i,
	   g_deviceOutputAddress[i].u.Addr,
	   g_deviceOutputAddress[i].u.Addr + g_deviceOutputLength[i]-1);
    
    for(j = 0; j < g_deviceOutputLength[i]; j++)
    {
      printf(" %02x", g_arrayOfOutputData[i][j]);
    }
    printf("\n");
  }
}

/****************************************************************************** 
 *                                                           
 *                   Callbacks definitions                   
 *                                                           
*******************************************************************************
 * callacks must be returned as soon as possible             
 * don't use any endless or time consuming functions         
 * e.g. exit() would be fatal                                
 * defer all time consuming functionality to other threads   
******************************************************************************/

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
       
  if(pCbfPrm->CbeType == PNIO_CBE_MODE_IND) /* Check callback type */
  {
    switch (pCbfPrm->ModeInd.Mode)
    {
      case PNIO_MODE_OFFLINE:	
           printf("request-> OFFLINE: " );
           g_currentMode = PNIO_MODE_OFFLINE;
           break;
      case PNIO_MODE_CLEAR:
           printf("request-> CLEAR: ");
           g_currentMode = PNIO_MODE_CLEAR;
           break;
      case PNIO_MODE_OPERATE:
           printf("request-> OPERATE: " );
           g_currentMode = PNIO_MODE_OPERATE;
           break;
      default:
           printf("Wrong mode selected: " );
           break;
    };

    /*send notification */
    g_SemModChange = 1;
  }
}

void callback_for_alarm_indication(PNIO_CBE_PRM *pCbfPrm)
{
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
#ifdef DEBUG
	  printf("PNIO_ALARM_DEV_FAILURE\n");
#endif
        break;

      case PNIO_ALARM_DEV_RETURN:
#ifdef DEBUG
	printf("PNIO_ALARM_DEV_RETURN\n");
#endif
        g_DeviceReady = 1;	//Set g_DeviceReady flag to notify application that CP is ready for communication
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

void callback_for_cp_stop_req(PNIO_CBE_PRM *pCbfPrm)
{
  printf("\nCP_STOP_REQUEST recieved, terminating program ...\n");
  g_cpStopReq = true;
}
/****************************************************************************** 
 *                                                           
 *                           Main                            
 *                                                           
******************************************************************************/

int main(int argc, char *argv[])
{
  PNIO_UINT32 dwErrorCode = PNIO_OK;
  unsigned int i,j,k;
    
  printf("--------------------------------------------------------------------------------\n\n");
  printf("Application does following: \n");
  printf("1. Initialize CP\n");
  printf("2. Change mode to OPERATE\n");
  printf("3. Update output data in loop\n");
  printf("4. Change mode to OFFLINE\n");
  printf("5. Uninitialize CP\n\n");
  printf("--------------------------------------------------------------------------------\n\n");
   
  //Prepare module variables and data structures
  AddOutputModule(16,4116);
  AddOutputModuleDataStructure();
  
  AddInputModule(1,4120);
  AddInputModuleDataStructure();
  
  //Fill out Output data
  g_arrayOfOutputData[0][0]   = 1;	g_arrayOfOutputData[0][1]   = 2;	
  g_arrayOfOutputData[0][2]   = 3;	g_arrayOfOutputData[0][3]   = 4;
  g_arrayOfOutputData[0][4]   = 5;	g_arrayOfOutputData[0][5]   = 6;	
  g_arrayOfOutputData[0][6]   = 7;	g_arrayOfOutputData[0][7]   = 8;	
  g_arrayOfOutputData[0][8]   = 9;	g_arrayOfOutputData[0][9]   = 10;	
  g_arrayOfOutputData[0][10]  = 11;	g_arrayOfOutputData[0][11]  = 12;	
  g_arrayOfOutputData[0][12]  = 13;	g_arrayOfOutputData[0][13]  = 14;	
  g_arrayOfOutputData[0][14]  = 15;	g_arrayOfOutputData[0][15]  = 16;	
  
  //Initialize
  g_DeviceReady = 0;
  g_dwHandle = Initialize(g_dwCpId);
    
  //Change mode to PNIO_MODE_OPERATE
  ChangeAndWaitForPnioMode(g_dwHandle, PNIO_MODE_OPERATE);
    
  printf("Starting communication:");
  i = 0;
  while(g_DeviceReady == 0)
  {
    if((i % 5) == 0) printf(".");
    i++;
    usleep(100000);
    if(i == MAX_NUM_OF_INIT_ATTEMPTS)
    {
      printf("Not able to start communication, Uninitializing...");
      break;
    }
  }
  printf("SUCCESS\n");
  printf("--------------------------------------------------------------------------------\n\n");
   
  if(g_DeviceReady != 0)
  {
    for(i = 0; ((i < 5) && (g_cpStopReq == 0)); i++)
    {
      for(j = 0; ((j < 7) && (g_cpStopReq == 0)); j++)
      {
	//Write Output data
	g_arrayOfOutputData[0][0] = g_arrayOfOutputData[0][0] << 1;
	PrintOutputData();
	dwErrorCode = UpdateCyclicOutputData(g_dwHandle);
	if(dwErrorCode != PNIO_OK)
	{
	  printf("Error 0x%x\n", (int)dwErrorCode);
	  printf("output:0x%x  output device state:%s \n",
	         g_deviceOutputData[0],
	         ((g_deviceOutputState[0]==PNIO_S_GOOD)?"good":"bad"));
	}
	
	//Read Input data
	dwErrorCode = UpdateCyclicInputData(g_dwHandle);
	PrintInputData();
	if(dwErrorCode != PNIO_OK)
	{
	  printf("Error 0x%x\n", (int)dwErrorCode);
	  printf("input:0x%x  input device state:%s \n",
	         g_deviceOutputData[0],
	         ((g_deviceOutputState[0]==PNIO_S_GOOD)?"good":"bad"));
	}
	
	usleep(50000);
      }
    
      for(k = 0; ((k < 7) && (g_cpStopReq == 0)); k++)
      {
	g_arrayOfOutputData[0][0] = g_arrayOfOutputData[0][0] >> 1;
	PrintOutputData();
	dwErrorCode = UpdateCyclicOutputData(g_dwHandle);
	if(dwErrorCode != PNIO_OK)
	{
	  printf("Error 0x%x\n", (int)dwErrorCode);
	  printf("output:0x%x  output device state:%s \n",
	         g_deviceOutputData[0],
	         ((g_deviceOutputState[0]==PNIO_S_GOOD)?"good":"bad"));
	}
	
	dwErrorCode = UpdateCyclicInputData(g_dwHandle);
	PrintInputData();
	if(dwErrorCode != PNIO_OK)
	{
	  printf("Error 0x%x\n", (int)dwErrorCode);
	  printf("input:0x%x  input device state:%s \n",
	         g_deviceOutputData[0],
	         ((g_deviceOutputState[0]==PNIO_S_GOOD)?"good":"bad"));
	}
	
	usleep(50000);
      }
    }
  }
  //change current mode to offline  
  ChangeAndWaitForPnioMode(g_dwHandle, PNIO_MODE_OFFLINE);
 
  //UnInitialize
  UnInitialize(g_dwHandle);
  
  RemoveIOModules();
  RemoveModuleDataStructure();
 
  return(EXIT_SUCCESS);
  
}