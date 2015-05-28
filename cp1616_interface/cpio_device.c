/*********************************************************************************************//**
* @file cpio_device.c
* 
* cp1616_interface IO_Device testing node 
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

/***************************************************/
/* CP1616 annotations                              */
/***************************************************/

#define ANNOT_NAME       "StarterKit"      /* device type (String 25) */
#define ANNOT_ORDERID    "6GK1 161-6AA00"  /* Order Id    (String 20) */
#define ANNOT_HW_REV     0                 /* HwRevision  (short)     */
#define ANNOT_SW_PREFIX  'V'               /* SwRevisionPrefix (char) */
#define ANNOT_SW_REV_1   2                 /* SwRevision1 (short)     */
#define ANNOT_SW_REV_2   6                 /* SwRevision2 (short)     */
#define ANNOT_SW_REV_3   0                 /* SwRevision3 (short)     */

/***************************************************/
/* Values for PNIO_device_open                     */
/***************************************************/

#define NUM_OF_SLOTS 9
#define VENDOR_ID    0x002a
#define DEVICE_ID    0x0003
#define INSTANCE_ID  0x0001
	
/*  mod    submod        modId   subId   Api */
#define DEVICE_DATA \
   { 1,      1,         0x001b, 0x010001, 0x00, 0, 0, 0, 0 }, \
   { 2,      1,         0x0020, 0x0001, 0x00, 0, 0, 0, 0 }, \
   { 3,      1,         0x0032, 0x0001, 0x00, 0, 0, 0, 0 }, \
   { 4,      1,         0x0029, 0x0001, 0x00, 0, 0, 0, 0 }, \
   { 5,      1,         0x0027, 0x0001, 0x00, 0, 0, 0, 0 }, \
   { 6,      1,         0x0028, 0x0001, 0x00, 0, 0, 0, 0 }, \
   { 7,      1,         0x0033, 0x0001, 0x00, 0, 0, 0, 0 }, \
   { 8,      1,         0x0034, 0x0001, 0x00, 0, 0, 0, 0 }, \
   { 9,      1,         0x0023, 0x0001, 0x00, 0, 0, 0, 0 }

#define DEVICE_DATA_ENTRIES 9 /* The total number of members of DEVICE_DATA structure */

/**************************************************/
/* Known Diagnose alarms for this device          */
/**************************************************/

#define CH_ERR_INVAL_LINKUP     0x0100
#define CH_ERR_INVAL_LINKDOWN   0x0101
#define CH_ERR_NO_REDUND_PS     0x0200
#define CH_ERR_NO_CPLUG         0x0201
#define CH_ERR_CPLUG_ERROR      0x0202

/**************************************************/
/*Headers                                         */
/**************************************************/

#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <stdlib.h>
#include <string.h>

#include "pniobase.h"
#include "pniousrd.h"
#include "pnioerrx.h"

/************************************************/
/*Typedefs                                      */
/************************************************/

typedef struct device_data_s
{
    int slot;
    int subslot;
    PNIO_UINT32 modId;
    PNIO_UINT32 subslotId;
    PNIO_UINT32 api;
    PNIO_UINT16 maxSubslots;      /* for internal use. set this to 0 */
    int modState;                 /* for internal use. set this to 0 */
    int subState;                 /* for internal use. set this to 0 */
    int dir;                      /* for internal use, set this to 0 */
} device_data_t;


/************************************************/
/*Global data for CP                            */
/************************************************/
PNIO_UINT32 g_dwCpId   = 1;                           //CP INDEX
PNIO_UINT32 g_dwHandle = 0;                           //Device Handle				 

static device_data_t device_data[] = { DEVICE_DATA }; //Data Structure

device_data_t *g_device_data = NULL;
static int g_dwArraySize = DEVICE_DATA_ENTRIES;       //Total number of slots as configured in the STEP7 project



/*********************************************************** */
/*                                                           */
/*Function:        Initialize()                              */
/*                                                           */
//************************************************************/
/* The function does the initialization of the PNIO device   */
/* registration of callbacks is part of initialization       */
/*************************************************************/


PNIO_UINT32 Initialize(PNIO_UINT32 CP_INDEX)
{
  
  PNIO_CFB_FUNCTIONS  structCBFunctions;
  PNIO_UINT32 dwHandle = 0;               //0 is invalid handle
  PNIO_UINT32 uiMaxAR=1;                  //maximum application relationships supported
  PNIO_UINT32 dwErrorCode = PNIO_OK;
  
  //Initialize the annotation structure
  PNIO_ANNOTATION structPNIOAnnotation = {
    ANNOT_NAME,
    ANNOT_ORDERID,
    ANNOT_HW_REV,
    ANNOT_SW_PREFIX,
    ANNOT_SW_REV_1,
    ANNOT_SW_REV_2,
    ANNOT_SW_REV_3};
  
  //Initialize the callback structure
  //Set the callback function pointers
  memset(&structCBFunctions, 0, sizeof(PNIO_CFB_FUNCTIONS)); 
  structCBFunctions.size                  = sizeof(PNIO_CFB_FUNCTIONS);
  structCBFunctions.cbf_data_write        = PNIO_cbf_data_write;
  structCBFunctions.cbf_data_read         = PNIO_cbf_data_read;
  structCBFunctions.cbf_rec_read          = PNIO_cbf_rec_read;
  structCBFunctions.cbf_rec_write         = PNIO_cbf_rec_write;
  structCBFunctions.cbf_alarm_done        = PNIO_cbf_req_done;
  structCBFunctions.cbf_check_ind         = PNIO_cbf_check_ind;
  structCBFunctions.cbf_ar_check_ind      = PNIO_cbf_ar_check_ind;
  structCBFunctions.cbf_ar_info_ind       = PNIO_cbf_ar_info_ind;
  structCBFunctions.cbf_ar_indata_ind     = PNIO_cbf_ar_indata_ind;
  structCBFunctions.cbf_ar_abort_ind      = PNIO_cbf_ar_abort_ind;
  structCBFunctions.cbf_ar_offline_ind    = PNIO_cbf_ar_offline_ind;
  structCBFunctions.cbf_apdu_status_ind   = PNIO_cbf_apdu_status_ind;
  structCBFunctions.cbf_prm_end_ind       = PNIO_cbf_prm_end_ind;
  structCBFunctions.cbf_cp_stop_req       = PNIO_cbf_cp_stop_req;
  structCBFunctions.cbf_device_stopped    = PNIO_cbf_device_stopped;
  structCBFunctions.cbf_start_led_flash   = NULL;
  structCBFunctions.cbf_stop_led_flash    = NULL;  
         
  printf("Open PNIO_device: ");
  
  //Connect to CP and obtain handle
  dwErrorCode = PNIO_device_open(
    /*in*/ CP_INDEX,			   //Communication Module index
    /*in*/ PNIO_CEP_MODE_CTRL,            //permission to change operation mode
    /*in*/ VENDOR_ID,                     //vendor ID
    /*in*/ DEVICE_ID,                     //device ID
    /*in*/ INSTANCE_ID,                   //instance ID
    /*in*/ uiMaxAR,                       //max AR count
    /*in*/ &structPNIOAnnotation,         //annotation
    /*in*/ &structCBFunctions,            //callback functions information
    /*out*/&dwHandle                      //device handle    
  );
  
  //Check errors
  if(dwErrorCode != PNIO_OK)
  {
    printf("ERROR:  0x%x\n", (int)dwErrorCode);
    exit(1);  /*exit*/
  }
  printf("SUCCESS\n");
 
  return dwHandle;
}

/*********************************************************** */
/*                                                           */
/*Function:        UnInitialize()                            */
/*                                                           */
//************************************************************/
/* The function does the deinitialization of the PNIO device */
/*************************************************************/

void UnInitialize(PNIO_UINT32 dwHandle)
{
  printf("Close PNIO_device: ");
  PNIO_UINT32 dwErrorCode = PNIO_OK;
  
  dwErrorCode = PNIO_device_close(dwHandle);
  
  if(dwErrorCode != PNIO_OK)
  {
    printf("Error 0x%x\n", (int) dwErrorCode);
    exit(1);
  }
  printf("SUCCESS\n");
}

/*********************************************************** */
/*                                                           */
/*Function:        AddApi()                                  */
/*                                                           */
//************************************************************/
/* The function adds all the api stated in configuration     */
/* structure for each module/submodule                       */
/*************************************************************/

void AddApi(void)
{
  int i;
  int j;
  int highestSlotsNumber;
  int highestSubslotNumber = 0;
  PNIO_UINT32 api;
  PNIO_UINT32 dwErrorCode = PNIO_OK;
  
  //for each slot
  for(i = j = 0; i < g_dwArraySize; i++)
  {
    //read api from configuration data
    api = g_device_data[i].api;
    
    //look if api added at a prior position
    for(j = 0; j < i; j++)
    {
      if(api == g_device_data[j].api){
	//api was added 
	break;
      }
    }    
  }
  
  if(i == j) { /* not added, add a new api */
  /* calculate highest slot and subslot number for this api */
  highestSlotsNumber   = g_device_data[j].slot;
  highestSubslotNumber = g_device_data[j].subslot;

  /*
  check if the api exists in the slots ahead,
  if yes, then update highest slot/subslot number accordingly
  */
  
  for(j = i+1; j <  g_dwArraySize; j++) 
  {
    if(api == g_device_data[j].api) 
    {
      if(g_device_data[j].slot > highestSlotsNumber) highestSlotsNumber = g_device_data[j].slot;
      if(g_device_data[j].subslot > highestSubslotNumber) highestSubslotNumber = g_device_data[j].subslot;
    }
  }
  
  printf("Adding profile: ");
  
  dwErrorCode = PNIO_api_add(
    /*in*/ g_dwHandle,
    /*in*/ api,
    /*in*/ (PNIO_UINT16) highestSlotsNumber,
    /*in*/ (PNIO_UINT16) highestSubslotNumber
  );
  if(dwErrorCode != PNIO_OK)
    printf("Error 0x%x\n", (int) dwErrorCode);
  else
    printf("SUCCESS\n");
  
}

/*********************************************************** */
/*                                                           */
/*Function:        RemoveApi()                               */
/*                                                           */
//************************************************************/
/* The function remove all the api stated in configuration   */
/* structure                                                 */
/*************************************************************/
void RemoveApi(void)
{
  int i;
  int j;
  PNIO_UINT32 api;
  PNIO_UINT32 dwErrorCode = PNIO_OK;
  
  printf("Removing Api: ")
  //for each slot
  for(i = j = 0; i < g_dwArraySize && status == PNIO_OK; i++)
  {
    //read api from configuration data
    api = g_device_data[i].api;
    
    //look if the api has been added at a prior position in our g_device_data structure
    for(j = 0; j < i; j++)
    {
      if(api == g_device_data[j].api) break; // api added at a prior position, hence it has already been removed
    }
  
    if(i == j) //api not removed yet
    {
      dwErrorCode = PNIO_api_remove(g_dwHandle, api);
      if(dwErrorCode != PNIO_OK)
        printf("Error\n");
      else     
        printf("SUCCESS")
  }
}

/*********************************************************** */
/*                                                           */
/*Function:        AddModSubMod()                            */
/*                                                           */
//************************************************************/
/* The function adds all the modules and submodules of the   */
/* device in serial order                                    */
/*************************************************************/

void AddModSubMod(void)
{
  PNIO_UINT32 dwErrorCode = PNIO_OK;
  PNIO_DEV_ADDR addr;     //location (module/submodule)
  int slot = 0;
  int i;
  
  addr.AddrType = PNIO_ADDR_GEO;    //must be PNIO_ADDR_GEO
  
  //---------------------------------------------------
  //Add module 0 
  //---------------------------------------------------
  printf("Pluging module 0 ... \n");
  addr.u.Geo.Slot    = g_device_data[0].slot;    //plug module 0
  addr.u.Geo.Subslot = g_device_data[0].subslot  //get the corresponding sub-slot
  
  dwErrorCode = PNIO_mod_plug(
    /*in*/ g_dwHandle,   		//device handle
    /*in*/ g_device_data[0].api,       //api number
    /*in*/ &addr,                      //location(slot, subslot)
    /*in*/ g_device_data[0].modId);    //module 0 identifier
          
  
  if(dwErrorCode != PNIO_OK)
  {
    printf("Error 0x%x\n", (int) dwErrorCode);
    g_device_data[0].modState = 0;
  }
  else
  {
    printf("SUCCESS\n");
    g_device_data[0].modState = 1;
  }
  
  printf(" api=%u, slot=%d, subslot=%d, max_slots=%d, mod_id=%u\n",
        g_device_data[0].api,
        g_device_data[0].slot,
        g_device_data[0].subslot,
        g_device_data[0].maxSubslots,
        g_device_data[0].modId);
        
  
  if(!g_device_data[0].modState) {
        printf("ERROR: Failure in plugging module 0 -> no other module / submodule will be plugged...\n");
	exit(1);    
  }
  
  //-----------------------------------------------------
  //Add submodule corresponding to module 0
  //-----------------------------------------------------
  printf("Pluging submodule 0 to module 0...\n");
  dwErrorCode = PNIO_sub_plug (
     /*in*/ g_hDevice,                    /* device handle */
     /*in*/ g_device_data[0].api,         /* api number */
     /*in*/ &addr,                        /* location (slot, subslot) */
     /*in*/ g_device_data[0].subslotId);  /* submodule 0 identifier */
                
  
  if(dwErrorCode != PNIO_OK)
  {
    printf("Error 0x%x\n", (int) dwErrorCode);
    g_device_data[0].subState = 0;
  }
  else
  {
    printf("SUCCESS\n");
    g_device_data[0].subState = 1;
  }
  
   printf(" api=%u, slot=%d, subslot=%d, max_slots=%d, mod_id=%u\n",
        g_device_data[0].api,
        g_device_data[0].slot,
        g_device_data[0].subslot,
        g_device_data[0].maxSubslots,
        g_device_data[0].modId);

    if(!g_device_data[0].subState) {
        printf("ERROR: Failure in plugging the submodule corresponding to module 0\n");
        printf(" -> no other module / submodule will be plugged...\n");
	exit(1);
    }
  
  printf("Pluging another modules, submodules...\n");
  
  //----------------------------------------------------
  //Add all modules
  //----------------------------------------------------
  
  for(i = 1; i < g_dwArraySize)
  {
    addr.u.Geo.Slot    = g_device_data[i].slot;	//plug module at correct slot 
    addr.u.Geo.Subslot = g_device_data[i].subslot;    //get the corresponding sub-slot
    
    printf("Pluging module %d: " ,i);
    dwErrorCode = PNIO_mod_plug(
       /*in*/ g_dwHandle,   		   //device handle
       /*in*/ g_device_data[i].api,       //api number
       /*in*/ &addr,                      //location(slot, subslot)
       /*in*/ g_device_data[i].modId);    //module identifier
                  
    if(dwErrorCode != PNIO_OK)
    {
      printf("Error 0x%x\n", (int) dwErrorCode);
      g_device_data[i].modState = 0;
    }
    else
    {
      printf("SUCCESS\n");
      g_device_data[i].modState = 1;
    }
    
    printf(" api=%u, slot=%d, subslot=%d, max_slots=%d, mod_id=%u\n",
          g_device_data[i].api,
          g_device_data[i].slot,
          g_device_data[i].subslot,
          g_device_data[i].maxSubslots,
          g_device_data[i].modId);
   
    if(dwErrorCode == PNIO_OK)
    {
       //advance in the g_device_data structure jumping over all the submodule entries
       //to reach the next module entry in the structure
       i += g_device_data[i].maxSubslots;
    }
    else
    {
      //go to the next entry in g_device_data table
      i++;
    }
        
  } //end for
 
  //----------------------------------------------------
  //Add all submodules
  //----------------------------------------------------
  for(i = 1; i < g_dwArraySize; i++)
  {
    if(g_device_data[i].maxSubslots > 0){
      //beginning of a new slot
      slot = i;   //index of corresponding slot for a given subslot
      
      g_device_data[slot].subState = 1; 
    }
    
    if(g_device_data[slot].modState)
    {
      //add submodule only if the module is added
      addr.u.Geo.Slot     = g_device_data[i].slot;
      addr.u.Geo.Subslot  = g_device_data[i].subslot;
      
      printf("Pluging submodule to module %d:", i);
      dwErrorCode = PNIO_sub_plug (
       /*in*/ g_dwHandle,                   //device handle
       /*in*/ g_device_data[i].api,         //api number
       /*in*/ &addr,                        //location(slot, subslot)
       /*in*/ g_device_data[i].subslotId);  //submodule identifier
	
      if(dwErrorCode != PNIO_OK)
      {
	printf("Error 0x%x\n", (int) dwErrorCode);
	g_device_data[i].subState = 0;
	g_device_data[slot].subState = 0;
      }
      else
      {
	printf("SUCCESS\n");
	g_device_data[i].subState = 1;
      }
      
      printf(" api=%u, slot=%d, subslot=%d, max_slots=%d, mod_id=%u\n",
          g_device_data[i].api,
          g_device_data[i].slot,
          g_device_data[i].subslot,
          g_device_data[i].maxSubslots,
          g_device_data[i].modId);
    }
  }  //end for
 
  //if not all the modules/submodules are plugger correctyl, print warning
  for(i = 0; i < g_dwArraySize; i++)
  {
    if(g_device_data[i].subState == 0)
    {
      printf("WARNING: Not all modules or submodules were plugged correctly!!\n");
      break;
    }
  } 
}

/*********************************************************** */
/*                                                           */
/*Function:        RemoveModSubMod()                         */
/*                                                           */
//************************************************************/
/* The function first removes the submodules and then the    */
/* modules from PNIO device in reverse order                 */
/*************************************************************/
void RemodeModSubMod(void)
{
  int i;
  PNIO_DEV_ADDR addr;   //location module/submodule
  PNIO_UINT32 dwErrorCode 
  
  //Remove modules/submodules in reverse order
  for(i = g_dwArraySize -1; i >= 0 && dwErrorCode == PNIO_OK; i--)
  {
    if(g_device_data[i].subState == 1)
    {
      addr.AddrType      = PNIO_ADDR_GEO;          //must be PNIO_ADDR_GEO
      addr.u.Geo.Slot    = g_device_data[i].slot;  //slot number
      addr.u.Geo.Subslot = g_device_data[i].subslot;
      
      //----------------------------------------------------
      //Remove submodules
      //----------------------------------------------------
      printf("Removing submodule:")
      dwErrorCode = PNIO_sub_pull(
	/*in*/ g_dwHandle,
	/*in*/ g_device_data[i].api, &addr);
      
      if(dwErrorCode == PNIO_OK)
      {
	printf("SUCCESS\n");
	g_device_data[i].subState = 0;
      }
      else
	printf("Error 0x%x\n", (int) dwErrorCode);
      
        printf(" api=%u, slot=%d, subslot=%d, max_slots=%d, mod_id=%u\n",
                g_device_data[i].api,
                g_device_data[i].slot,
                g_device_data[i].subslot,
                g_device_data[i].maxSubslots,
                g_device_data[i].modId);
      
       //Notify the controller that the device state is NOT-OK every time after removing a submodule
      dwErrorCode = PNIO_set_dev_state(g_dwHandle, PNIO_DEVSTAT_STATION_PROBLEM);
    }
    
    if(dwErrorCode == PNIO_OK && g_device_data[i].modState == 1)
    {
      addr.AddrType      = PNIO_ADDR_GEO;		//must be PNIO_ADDR_GEO
      addr.u.Geo.Slot    = g_device_data[i].slot;     //slot number
      addr.u.Geo.Subslot = 1;				//doesnt matter
      
      //----------------------------------------------------
      //Remove modules
      //----------------------------------------------------
      printf("Removing module: ")
      dwErrorCode = PNIO_mod_pull(g_dwHandle, g_device_data[i].api, &addr);
      
       if(dwErrorCode == PNIO_OK)
      {
	printf("SUCCESS\n");
	g_device_data[i].subState = 0;
      }
      else
	printf("Error 0x%x\n", (int) dwErrorCode);
      
      printf(" api=%u, slot=%d, subslot=%d, max_slots=%d, mod_id=%u\n",
                g_device_data[i].api,
                g_device_data[i].slot,
                g_device_data[i].subslot,
                g_device_data[i].maxSubslots,
                g_device_data[i].modId);
      
      dwErrorCode = PNIO_set_dev_state(g_dwHandle, PNIO_DEVSTAT_STATION_PROBLEM);     
    }      
  }  
}



int main(void)
{
  PNIO_UINT32 ErrorCode = PNIO_OK;
  
  printf("Application does following: \n");
  printf("Initialize PNIO device\n");
  printf("Read/Write IO data\n");
  printf("Process the data record read/write request of IO controller\n");
  printf("Uninitialize device\n");
  
  //Initialize
  g_dwHandle = Initialize(g_dwCpId);
  
  //Add profile
  AddApi();
  
  //Add modules and submodules to the device
  AddModSubMod();
  
  //Remove the modules and submodules
  RemodeModSubMod();
  
  //Remove the API
  RemoveApi();
  
  //Uninitialize
  UnInitialize(g_dwHandle);
  
  //set handle to invalid values
  g_dwHandle = 0; 
  
  
  
}