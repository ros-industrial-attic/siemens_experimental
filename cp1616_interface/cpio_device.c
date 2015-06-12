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

/******************************************************************************
 * CP1616 annotations                              
 *****************************************************************************/

#define ANNOT_NAME       "StarterKit"      /* device type (String 25) */
#define ANNOT_ORDERID    "6GK1 161-6AA00"  /* Order Id    (String 20) */
#define ANNOT_HW_REV     0                 /* HwRevision  (short)     */
#define ANNOT_SW_PREFIX  'V'               /* SwRevisionPrefix (char) */
#define ANNOT_SW_REV_1   2                 /* SwRevision1 (short)     */
#define ANNOT_SW_REV_2   6                 /* SwRevision2 (short)     */
#define ANNOT_SW_REV_3   0                 /* SwRevision3 (short)     */

/******************************************************************************
 * Values for PNIO_device_open                     
 ******************************************************************************/

#define NUMOF_SLOTS 1			   /* slot 1*/ 
#define NUMOF_SUBSLOTS          2         /* Every slot has 1 subslot */
#define NUMOF_BYTES_PER_SUBSLOT 1         /* Maximum data length as configured in the sample */
#define VENDOR_ID    0x002a
#define DEVICE_ID    0x0003
#define INSTANCE_ID  0x0001
	
/*  slot   subslot     modId  subId Api */
#define DEVICE_DATA \
   { 1,      1,         0x19, 0x10001, 0x00, 0, 0, 0, 0 }, \
   
   
#define DEVICE_DATA_ENTRIES 1 /* The total number of members of DEVICE_DATA structure  */
#define MAX_COUNT 500	       /* Max counter value for PNIO_cbf_prm_end_ind() callback */

/******************************************************************************
 * Known Diagnose alarms for this device          
 *****************************************************************************/

#define CH_ERR_INVAL_LINKUP     0x0100
#define CH_ERR_INVAL_LINKDOWN   0x0101
#define CH_ERR_NO_REDUND_PS     0x0200
#define CH_ERR_NO_CPLUG         0x0201
#define CH_ERR_CPLUG_ERROR      0x0202

/******************************************************************************
 * Headers                                         
******************************************************************************/

#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <stdlib.h>
#include <string.h>

#include "pniobase.h"
#include "pniousrd.h"
#include "pnioerrx.h"

/******************************************************************************
 * Typedefs                                      
 *****************************************************************************/

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

/******************************************************************************
 * Function prototypes                           
 *****************************************************************************/
int GetSubmodNum(PNIO_UINT32 mod, PNIO_UINT32 sub);


/******************************************************************************
 * Global data for CP                            
 *****************************************************************************/
PNIO_UINT32 g_dwCpId   = 1;     //CP INDEX
PNIO_UINT32 g_dwHandle = 0;     //Device Handle				 

PNIO_UINT16 g_SessionKey = 0;   //Session identifier obtained in PNIO_cbf_ar_info_ind callback
PNIO_UINT16 g_arNumber = 0;     //Application relation number

static device_data_t device_data[] = { DEVICE_DATA }; //Data Structure

device_data_t *g_device_data = NULL;
static int g_dwArraySize = DEVICE_DATA_ENTRIES;       //Total number of slots as configured in the STEP7 project
int idxTbl[DEVICE_DATA_ENTRIES];                      //An array of slot ids, sub-slot entries will contain -1 

static int AR_INFO_IND_flag = 0;   // Callback: Application relation with IO controller is set 
static int PRM_END_IND_flag = 0;   // Callback: End of Parametrization 
static int INDATA_IND_flag = 0;    // Callback: First data transmission from IO-Controller
static int OFFLINE_IND_flag = 0;   // Callback: Device has recieved stop request from user application

/**** Output Data  (IO Controller ==> IO Device) */
PNIO_UINT8     OutData        [NUMOF_SLOTS][NUMOF_SUBSLOTS][NUMOF_BYTES_PER_SUBSLOT];
PNIO_UINT32    OutDatLen      [NUMOF_SLOTS][NUMOF_SUBSLOTS];
PNIO_IOXS      OutDatIocs     [NUMOF_SLOTS][NUMOF_SUBSLOTS];
PNIO_UINT8     OutDatIops     [NUMOF_SLOTS][NUMOF_SUBSLOTS];

/**** Input Data  (IO Device ==> IO Controller) */
PNIO_UINT8     InData         [NUMOF_SLOTS][NUMOF_SUBSLOTS][NUMOF_BYTES_PER_SUBSLOT];
PNIO_UINT32    InDatLen       [NUMOF_SLOTS][NUMOF_SUBSLOTS];
PNIO_IOXS      InDatIops      [NUMOF_SLOTS][NUMOF_SUBSLOTS];
PNIO_UINT8     InDatIocs      [NUMOF_SLOTS][NUMOF_SUBSLOTS];


/****************************************************************************** 
 *                                                           
 *                             CALLBACKS                                
 *                                                           
 ******************************************************************************/ 


/******************************************************************************* 
 *                                                           
 * Callback:               PNIO_cbf_data_write()                     
 *                                                           
 *******************************************************************************
 * Passes the input data from the application to the stack. The application 
 * reads the data from the specified input module and handles it to the stack                        
 *******************************************************************************/
PNIO_IOXS PNIO_cbf_data_write(
  /*in*/  PNIO_UINT32      DevHndl,   //device handle
  /*in*/  PNIO_DEV_ADDR*   pAddr,     //geographical address
  /*in*/  PNIO_UINT32      BufLen,    //length of the submodule input data
  /*out*/ PNIO_UINT8*      pBuffer,   //Ptr to write data buffer
  /*in*/  PNIO_IOXS        Iocs)      //remote IO controller consumer status

{
  unsigned int i;
  
  PNIO_UINT32 slot_num    = pAddr->u.Geo.Slot;
  PNIO_UINT32 subslot_num = pAddr->u.Geo.Subslot;
  
  printf("PNIO_cbf_data_write(len = %u, Iocs = %u devHandle = %u, slot = %u, subslot = %u\n",
        BufLen, Iocs, DevHndl, slot_num, subslot_num);
  
  InDatLen  [slot_num][subslot_num] = BufLen;		//save data length (only for debugging)
  InDatIocs [slot_num][subslot_num] = Iocs;		//consumer status (of remote IO Controller)
  
  if(BufLen == 0)
  {
    printf("BuLen = 0, nothing to write to...\n");
    InDatIops [slot_num][subslot_num] = PNIO_S_GOOD;
  }
  else if (BufLen <= (PNIO_UINT32)NUMOF_BYTES_PER_SUBSLOT)
  {
    memcpy(pBuffer, &InData[slot_num][subslot_num][0], BufLen);	//copy the application data to the stack
    InDatIops[slot_num][subslot_num] = PNIO_S_GOOD;			//assume everything is ok
    
    printf("IO Data buffer: \n");
    for(i = 0; i < BufLen; i++) 
    {
      if(i%16 == 0 && i!=0)
	printf("\n");

      printf(" 0x%02x", pBuffer[i]);
    }
    printf("\n");
  }
  else
  {
    printf("!!! PNIO_cbf_data_write: Buflen=%lu > allowed size (%u)!!! Abort writing..\n",
            (unsigned long)BufLen, NUMOF_BYTES_PER_SUBSLOT);
    
    InDatIops [slot_num][subslot_num] = PNIO_S_BAD; /* set local status to bad */
  }
  
  return (InDatIops[slot_num][subslot_num]);   //return local provider status
  
}

/****************************************************************************** 
 *                                                           
 * Callback:                   PNIO_cbf_data_read()                      
 *                                                           
 ******************************************************************************
 * Passes the output data from the stack to the application. The application 
 * takes the data and writes it to the specified output module                                   
 *****************************************************************************/
PNIO_IOXS PNIO_cbf_data_read(
  /*in*/  PNIO_UINT32     DevHndl,        //device handle
  /*in*/  PNIO_DEV_ADDR*  pAddr,          //geographical address
  /*in*/  PNIO_UINT32     BufLen,         //length of submodule input data
  /*in*/  PNIO_UINT8*     pBuffer,        //Ptr to data buffer to read from
  /*in*/  PNIO_IOXS       Iops)           //IO controller provider status
{
  unsigned int i;
  
  PNIO_UINT32 slot_num     = pAddr->u.Geo.Slot;
  PNIO_UINT32 subslot_num  = pAddr->u.Geo.Subslot;
  
  printf("## PNIO_cbf_data_read(..., len=%u, Iops=%u) for devHandle 0x%x, slot %u, subslot %u\n",
        BufLen, Iops, DevHndl, slot_num, subslot_num);
  
  OutDatLen  [slot_num][subslot_num] = BufLen;		//save data length (only for debugging)
  OutDatIops [slot_num][subslot_num] = Iops;		//provider status (of remote IO controller)
  
  if(BufLen == 0)
  {
    printf(" BufLen = 0, nothing to read..\n");
    OutDatIocs [slot_num][subslot_num] = PNIO_S_GOOD;
  }
  else if(BufLen <= (PNIO_UINT32)NUMOF_BYTES_PER_SUBSLOT) 
  {
    memcpy (&OutData[slot_num][subslot_num][0], pBuffer, BufLen); //Copy the data from the stack to the application buffer
    OutDatIocs [slot_num][subslot_num] = PNIO_S_GOOD;             // assume everything is ok 
    
    printf(" IO Data buffer:\n");
    for(i = 0; i < BufLen; i++) 
    {
      if(i % 16 == 0 && i!=0)
	printf("\n");

      printf(" 0x%02x", OutData[slot_num][subslot_num][i]);
    }
    printf("\n");
  }
  else
  {
     printf("!!! PNIO_cbf_data_read: Buflen=%lu > allowed size (%u)!!! Abort reading...\n",
            (unsigned long)BufLen, NUMOF_BYTES_PER_SUBSLOT);
     OutDatIocs [slot_num][subslot_num] = PNIO_S_BAD; // set local status to bad 
  }
  
  return(OutDatIocs[slot_num][subslot_num]);		//consumer state (of local IO device)
}

/****************************************************************************** 
 *                                                           
 * Callback:                  PNIO_cbf_rec_read()                       
 *                                                           
 ******************************************************************************
 * This callback is called to notify tha a read record request has been 
 * recieved from IO Controller. This function has to provide the record
 * data and copy it to the specified buffer address. The maximum pBufLen 
 * can not be exceeded! This callback returns the real copied data      
 * length and the success state (PNIO_OK)                    
 *****************************************************************************/
void PNIO_cbf_rec_read  (
    /*in*/      PNIO_UINT32          DevHndl,         //device handle 
    /*in*/      PNIO_UINT32          Api,             //api number
    /*in*/      PNIO_UINT16          ArNumber,        //Application-relation number
    /*in*/      PNIO_UINT16          Sessi,           //Manual synchronization - The main function calls device-stop functiononKey
    /*in*/      PNIO_UINT32          SequenceNum,     //Sequence number
    /*in*/      PNIO_DEV_ADDR      * pAddr,           //geographical address 
    /*in*/      PNIO_UINT32          RecordIndex,     //record index
    /*in/out*/  PNIO_UINT32        * pBufLen,         //in: length to read, out: length, read by user 
    /*out*/     PNIO_UINT8         * pBuffer,         // [out] buffer pointer */
    /*out*/     PNIO_ERR_STAT      * pPnioState)      // 4 byte PNIOStatus (ErrCode, ErrDecode, ErrCode1, ErrCode2), see IEC61158-6 */
{
    PNIO_UINT32 i;
    PNIO_UINT32 dwErrorCode = PNIO_OK;
    
    //fill dummy buffer
    PNIO_UINT8 ReadRecDummyData[] = {"Test Record"};
    
    if(*pBufLen > sizeof(ReadRecDummyData))
      *pBufLen = sizeof(ReadRecDummyData);
    
    printf("\nREAD_RECORD Request, Api=%lu Slot=%lu Subslot=%lu Index=%lu, Length=%lu, Sequence_nr=%lu\n",
	   (unsigned long)Api, 
	   (unsigned long)pAddr->u.Geo.Slot,
	   (unsigned long)pAddr->u.Geo.Subslot,
	   (unsigned long)RecordIndex,
	   (unsigned long)*pBufLen, 
	   (unsigned long)SequenceNum);
    
    //copy the data to specified buffer
    if(*pBufLen < sizeof(ReadRecDummyData))
      printf("WARNING: Can not transmit all data, buffer too small...\n");
    
    memcpy(pBuffer,			//destination pointer for write data
	   ReadRecDummyData,		//source pointer for write data
	   *pBufLen);			//length of transmitted data
    
    printf("RECORD DATA transmitted:");
    for(i = 0; i < *pBufLen; i++)
    {
      if(i%16 == 0)
	printf("\n");
      
      printf("0x%02lx", (long)pBuffer[i]);
    }
    printf("\n");
    
    if(dwErrorCode == PNIO_OK)
    {
      memset(pPnioState, 0, sizeof(*pPnioState));
      return;
    } 
    else
    {
      *pBufLen=0;			
      pPnioState->ErrCode   = 0xde;	//IODReadRes with ErrorDecode = PNIORW 
      pPnioState->ErrDecode = 0x80;	//PNIORW
      pPnioState->ErrCode1  = 9;	//example: Error Class 10 = application, ErrorNr 9 = "feature not supported"
      pPnioState->ErrCode2  = 0;	//not used in this case	
      pPnioState->AddValue1 = 0;	//not used in this case
      pPnioState->AddValue2 = 0;	//not used in this case
      return;
  }
}

/****************************************************************************** 
 *                                                           
 * Callback:                     PNIO_cbf_rec_write()                      
 *                                                           
 ******************************************************************************
 * This callback is called to notify that a write record request has been 
 * recieved from IO controller. The user has to read the record data from 
 * the specified source buffer. The length of provided data are specified
 * in function parameter *pBufLen. After serving this function, the user 
 * returns the success state                                 
 *****************************************************************************/
void PNIO_cbf_rec_write  (   
    /*in*/      PNIO_UINT32          DevHndl,         //device handle 
    /*in*/      PNIO_UINT32          Api,             //api number
    /*in*/      PNIO_UINT16          ArNumber,        //Application-relation number
    /*in*/      PNIO_UINT16          Sessi,           //Manual synchronization - The main function calls device-stop functiononKey
    /*in*/      PNIO_UINT32          SequenceNum,     //Sequence number
    /*in*/      PNIO_DEV_ADDR      * pAddr,           //geographical address 
    /*in*/      PNIO_UINT32          RecordIndex,     //record index
    /*in/out*/  PNIO_UINT32        * pBufLen,         //in: length to read, out: length, read by user 
    /*out*/     PNIO_UINT8         * pBuffer,         // [out] buffer pointer */
    /*out*/     PNIO_ERR_STAT      * pPnioState)      // 4 byte PNIOStatus (ErrCode, ErrDecode, ErrCode1, ErrCode2), see IEC61158-6 */

{
    PNIO_UINT8  WriteRecDummyData[50];
    PNIO_UINT32 i;
    PNIO_UINT32 dwErrorCode = PNIO_OK;
    
    printf("\nWRITE RECORD Request, Api=%lu Slot=%lu Subslot=%lu Index=%lu, Length=%lu, Sequence_nr=%lu\n",
	(unsigned long)Api, 
	(unsigned long)pAddr->u.Geo.Slot,
	(unsigned long)pAddr->u.Geo.Subslot,
	(unsigned long)RecordIndex, 
	(unsigned long)*pBufLen, 
	(unsigned long)SequenceNum);
    
    //check data size (accepted_data < provided_data)
    if(*pBufLen > sizeof(WriteRecDummyData))
    {
      *pBufLen = sizeof(WriteRecDummyData);
      printf("WARNING: Can not write all data, not enough space..\n");
    }
    
    //copy the record data into a buffer for further use
    memcpy(WriteRecDummyData,		//destination pointer for record data
           pBuffer,			//source pointer for record data
	   *pBufLen);			//length of the accepted data
    
    printf("RECORD DATA written: ");
    for(i = 0; i < *pBufLen; i++)
    {
      if(i % 16 == 0)
	printf("\n");
      
      printf("0x%02lx", (long)WriteRecDummyData[i]);
    }
    printf("\n");
    
    if(dwErrorCode == PNIO_OK) 
    {
      memset(pPnioState, 0, sizeof(*pPnioState));
      return;
    }
    else  //if an eror occured, it must be specified according to IEC 61158-6
    {
      *pBufLen=0;
                                                            
      pPnioState->ErrCode   = 0xdf;	//IODWrites with ErrorDecode = PNIORW
      pPnioState->ErrDecode = 0x80;	//PNIORW
      pPnioState->ErrCode1  = 9;	//example: Error Class 10 = application, ErrorNr 9 = "feature not supported"
      pPnioState->ErrCode2  = 0;	//not used in this case
      pPnioState->AddValue1 = 0;	//not used in this case
      pPnioState->AddValue2  =0;	//not used in this case
      return;
    }
}

/*****************************************************************************
 *                                                           
 * Callback:                   PNIO_cbf_check_ind()                      
 *                                                           
 *****************************************************************************
 * This callback is called for every submodule of the IO Base Device interface,
 * for which the configuration does not match that of the IO controller. The 
 * user program is consequently given the option of changing the submodule   
 * layout or marking the submodule as compatible or incorrect. In case of 
 * agreement of the configuration, this callback is not called                               
 *****************************************************************************/

void PNIO_cbf_check_ind(
    /*in*/  PNIO_UINT32     DevHndl,       //device handle 
    /*in*/  PNIO_UINT32     Api,           //Api number 
    /*in*/  PNIO_UINT16     ArNumber,      //Application-relation number 
    /*in*/  PNIO_UINT16     SessionKey,    //session key
    /*in*/  PNIO_DEV_ADDR*  pAddr,         //geographical address 
    /*out*/ PNIO_UINT32*    pModIdent,     //Ptr to module identifier 
    /*out*/ PNIO_UINT16*    pModState,     //Ptr to module state 
    /*out*/ PNIO_UINT32*    pSubIdent,     //Ptr to submodule identifier 
    /*out*/ PNIO_UINT16*    pSubState)     //Ptr to submodule state 
{
    int idx;

    printf ("## CHECK_IND slot=%u, subslot=%u, ModId=0x%x, State(%u), SubId=%u, State (%u)\n",
        pAddr->u.Geo.Slot, pAddr->u.Geo.Subslot, *pModIdent, *pModState, *pSubIdent, *pSubState);

    /* get the index int of our configuration */
    idx = GetSubmodNum(pAddr->u.Geo.Slot, pAddr->u.Geo.Subslot);

    /*
    Check the configuration sent by controller against the configuration_data structure.
    If there is any mismatch, return error.
    */
    if((idx != -1) && ((unsigned int)g_device_data[idx].subslot == pAddr->u.Geo.Subslot) &&
        (g_device_data[idx].modId == *pModIdent) && (g_device_data[idx].subslotId == *pSubIdent)) {
        *pModState = PNIO_MOD_STATE_PROPER_MODULE;
        *pSubState = PNIO_SUB_STATE_IDENT_OK;
    } else {
        printf ("## the configuration of plugged modules is inconsistent to HWCONFIG\n");
        printf ("## please check your configuration first!\n");
        *pModState = PNIO_MOD_STATE_WRONG_MODULE;
        *pSubState = PNIO_SUB_STATE_IDENT_WRONG;
    }
}

/****************************************************************************** 
 *                                                           
 * Callback:                   PNIO_cbf_ar_check_ind()                  
 *                                                           
 ******************************************************************************
 * This callback is called by the IO Base Device interface as soon as controller
 * establishes a connection with the IO Base device user program. As a result of
 * this callback, application-relation global parameters are transferred to the
 * IO Base user program for insspection. In case of errors in the application-
 * relation layout, the IO Base Device user program can terminate the application-        
 * relational global parameters                              
 *****************************************************************************/

void PNIO_cbf_ar_check_ind(
    /*in*/  PNIO_UINT32     DevHndl,              //device handle
    /*in*/  PNIO_UINT32     HostIp,               //ip adderss of the host controller 
    /*in*/  PNIO_UINT16     ArType,               //Application-relation type 
    /*in*/  PNIO_UUID_TYPE  ArUUID,               //Application-relation UUID 
    /*in*/  PNIO_UINT32     ArProperties,         //Application-relation properties 
    /*in*/  PNIO_UUID_TYPE  CmiObjUUID,           //UUID of application-relation initiator e.g. IO-controller 
    /*in*/  PNIO_UINT16     CmiStationNameLength, //Length of the param station-name (next param) 
    /*in*/  PNIO_UINT8*     pCmiStationName,      //Station name 
    /*in*/  PNIO_AR_TYPE*   pAr)                  //pointer to application-relation structure PNIO_AR_TYPE 
{
    union {
        unsigned long l;
        unsigned char c[4];
    } lc;
    char stname[256];
    int  len = CmiStationNameLength < 256 ? CmiStationNameLength : 255;
    lc.l = HostIp;
    strncpy(stname, (const char *)pCmiStationName, len);	//copy StationName to stname
    stname[len] = '\0';
    printf("## PNIO_cbf_ar_check_ind (Station %s, IP %d.%d.%d.%d)\n",
        stname,lc.c[0], lc.c[1], lc.c[2], lc.c[3]);
}

/******************************************************************************
 *                                                           
 * Callback:                    PNIO_cbf_ar_info_ind()                    
 *                                                           
 ******************************************************************************
 * This callback is called by the IO Base Device interface as soon as the 
 * application-relation for the IO controller is laid out. Consequently, 
 * the IO Base Device user program is informed about the modules and submodules
 * that wiil be operated in this application relation. In this callback 
 * function user can initialize IO data in submodule to initial value 0 a can
 * also set local consumer and provider status                              
 ******************************************************************************/

void PNIO_cbf_ar_info_ind(
    /*in*/  PNIO_UINT32     DevHndl,    //device handle
    /*in*/  PNIO_UINT16     ArNumber,   //Application-relation number 
    /*in*/  PNIO_UINT16     SessionKey, //session key 
    /*in*/  PNIO_AR_TYPE*   pAr)        //pointer to application-relation structure PNIO_AR_TYPE 
{
    int i,j;
  
    g_arNumber = ArNumber;     // Store the AR number 
    g_SessionKey = SessionKey; // Store the session key 

    printf ("## AR-INFO_IND new AR from PNIO controller established, SessionKey %x\n",
        SessionKey);
    
    //---------------------------------------------------------------------
    //   set all IO data in submodules to inital value = 0              
    //---------------------------------------------------------------------
    memset (&InData,        0, sizeof (InData));        // IO data (input) 
    memset (&InDatLen,      0, sizeof (InDatLen));      // length of input data 
    memset (&InDatIops,     0, sizeof (InDatIops));     // local provider status 
    memset (&InDatIocs,     0, sizeof (InDatIocs));     // remote consumer status 

    memset (&OutData,       0, sizeof (OutData));       // IO data (output) 
    memset (&OutDatLen,     0, sizeof (OutDatLen));     // length of output data 
    memset (&OutDatIocs,    0, sizeof (OutDatIocs));    // local consumer status 
    memset (&OutDatIops,    0, sizeof (OutDatIops));    // remote provider status 

    //----------------------------------------------------------------------
    // set local provider status preset values for all input/output slots   
    // set local consumer status for all output slots
    //----------------------------------------------------------------------
    for(i = 0; i < DEVICE_DATA_ENTRIES ; i++) {
        for(j = 0; j < 1 /*g_device_data[i].maxSubslots*/; j++) {
            /*** set local provider state = GOOD for input data***/
            if(i == 0) {
                if(g_device_data[i].modState == 1) { /* plugged */
                    InDatIops [i][j] = PNIO_S_GOOD;
					OutDatIocs [i][j] = PNIO_S_GOOD;
                } else {
                    InDatIops [i][j] = PNIO_S_BAD;
					OutDatIocs [i][j] = PNIO_S_BAD;
                }
            } else {
                if(g_device_data[i].modState == 1 &&
                    g_device_data[i+j].subState == 1) { /* plugged */
                    InDatIops [i][j] = PNIO_S_GOOD;
					OutDatIocs [i][j] = PNIO_S_GOOD;
                } else {
                    InDatIops [i][j] = PNIO_S_BAD;
					OutDatIocs [i][j] = PNIO_S_BAD;
                }
            }
        }
    }

    AR_INFO_IND_flag = 1;
}

/****************************************************************************** 
 *                                                           
 * Callback:                  PNIO_cbf_ar_indata_ind()                  
 *                                                           
 ******************************************************************************
 * This callback is called by the IO Base Device interface as soon as an IO 
 * controller has transmitted the IO data for the first time. It signals the
 * beginning of cyclical data exchange                                             
 ******************************************************************************/

void PNIO_cbf_ar_indata_ind(
    /*in*/ PNIO_UINT32     DevHndl,        //device handle
    /*in*/ PNIO_UINT16     ArNumber,       //Application-relation number 
    /*in*/ PNIO_UINT16     SessionKey)     //session key 
{
    printf ("## AR IN-Data event indication has been received, ArNumber = %x\n\n",
        ArNumber);

    INDATA_IND_flag = 1;
}

/*****************************************************************************  
 *                                                           
 * Callback:                  PNIO_cbf_ar_abort_ind()                   
 *                                                           
 *****************************************************************************
 * This callback is called by the IO Base Device interface as soon as the
 * connection is terminated after a data exchange with IO began                                    
 *****************************************************************************/
void PNIO_cbf_ar_abort_ind(
    /*in*/  PNIO_UINT32     DevHndl,     //device handle
    /*in*/  PNIO_UINT16     ArNumber,    //Application-relation number 
    /*in*/  PNIO_UINT16     SessionKey,  //session key 
    /*in*/  PNIO_AR_REASON  ReasonCode)  //reason code 
{
    // AR abort after ArInData-indication 
    printf ("## AR ABORT indication, ArNumber = %x, Reason = %x\n\n",
        ArNumber, ReasonCode);
}

/****************************************************************************** 
 *                                                           
 * Callback:                   PNIO_cbf_ar_offline_ind()                 
 *                                                           
 ******************************************************************************
 * This callback is called by the IO Base Device interface as soon as the 
 * connection is terminated before a data exchange with IO began                                    
 *****************************************************************************/
void PNIO_cbf_ar_offline_ind(
    /*in*/  PNIO_UINT32     DevHndl,       //device handle 
    /*in*/  PNIO_UINT16     ArNumber,      //Application-relation number 
    /*in*/  PNIO_UINT16     SessionKey,    //session key 
    /*in*/  PNIO_AR_REASON  ReasonCode)    //reason code 
{
    printf ("## AR Offline indication, ArNumber = %x, Reason = %x\n",
        ArNumber, ReasonCode);
    
    OFFLINE_IND_flag = 1;
}

/******************************************************************************
 *                                                           
 * Callback:                   PNIO_cbf_prm_end_ind()                    
 *                                                           
 ******************************************************************************
 * This callback is called by the IO Base Device interface as soon as an IO 
 * controller signals the end of the parametrizing phase                                       
 *****************************************************************************/
void PNIO_cbf_prm_end_ind(
    /*in*/  PNIO_UINT32 DevHndl,            //device handle
    /*in*/  PNIO_UINT16 ArNumber,           //application-relation number 
    /*in*/  PNIO_UINT16 SessionKey,         //session key 
    /*in*/  PNIO_UINT32 Api,                //Associated API 
    /*in*/  PNIO_UINT16 SlotNum,            //slot number 
    /*in*/  PNIO_UINT16 SubslotNum)         //sub-slot number 
{
  int i=0;
  
  // Wait (MAX_COUNT x 0.1s) for PNIO_cbf_ar_info_ind() Callbacks
  while (AR_INFO_IND_flag == 0)
  {
    if (i==MAX_COUNT)
    {
      printf("No PNIO_cbf_ar_info_ind() event recived\n\n");
      return;
    }
    i++;
    usleep(100000);
  }
    printf ("## End of parametrizing phase - Application ready - received ArNumber = %x\n", ArNumber);
    printf("\n--------------------------------------------------------------------------------\n\n");
    
    PRM_END_IND_flag = 1;
}

/****************************************************************************** 
 *                                                           
 * Callback:                PNIO_cbf_cp_stop_req()                    
 *                                                           
 ******************************************************************************
 * This callback is called by the IO Base Device interface   
 * as soon as an IO controller stop request is recieved      
 *****************************************************************************/
void PNIO_cbf_cp_stop_req(PNIO_UINT32 DevHndl)
{
  printf("## PNIO_cbf_cp_stop_req\n");    
}

/****************************************************************************** 
 *                                                           
 * Callback:               PNIO_cbf_device_stopped()                 
 *                                                           
 ******************************************************************************
 * This callback is called by the IO Base Device interface as soon as an IO 
 * device stop requests is recieved         
 *****************************************************************************/
void PNIO_cbf_device_stopped(
    /*in*/  PNIO_UINT32 DevHndl,    //Handle for Multidevice 
    /*in*/  PNIO_UINT32 Reserved)   //Reserved for future use
{
  printf("## PNIO_cbf_device_stopped\n");
}

/****************************************************************************** 
 *                                                           
 * Callback:                PNIO_cbf_req_done()                       
 *                                                           
 ******************************************************************************
 * This callback is called if the IO Base device interface as soon as an alarm
 * is sent                               
 *****************************************************************************/
void PNIO_cbf_req_done (
    /*in*/ PNIO_UINT32          DevHndl,        //device handle
    /*in*/ PNIO_UINT32          UserHndl,       //user handle
    /*in*/ PNIO_UINT32          Status,         //status
    /*in*/ PNIO_ERR_STAT      * pPnioState)	 //PNIO state	
{
    printf("req_done not supported\n");
}

/****************************************************************************** 
 *                                                           
 * Callback:                PNIO_cbf_apdu_status_ind()                
 *                                                           
 ******************************************************************************
 * This callback is called after the state of the controller 
 * has changed                                               
 *****************************************************************************/
void PNIO_cbf_apdu_status_ind(
    /*in*/  PNIO_UINT32          DevHndl,	//device handle
    /*in*/  PNIO_UINT16          ArNumber,    //Application-relation number
    /*in*/  PNIO_UINT16          SessionKey,  //session key
    /*in*/  PNIO_APDU_STATUS_IND ApduStatus)
{
    printf("APDU Status Change not supported");
}



/****************************************************************************** 
 *                                                           
 *                             FUNCTIONS                               
 *                                                           
 ******************************************************************************/ 




/****************************************************************************** 
 *                                                           
 * Function:            do_after_prm_end_ind_cbf()                
 *                                                           
 ******************************************************************************
 * This function is called after the PNIO_cbf_prm_end_ind callback has been 
 * called, it calls PNIO_initiate_dat_write PNIO_initiate_data_read and 
 * PNIO_set_appl_state_ready             
 *****************************************************************************/

void do_after_prm_end_ind_cbf(void)
{
  PNIO_UINT32 dwErrorCode;
  PNIO_APPL_READY_LIST_TYPE readyListType;
  
  /* Here we need to call "PNIO_initiate_data_write" so that the IO Base Device user       
  * program can initialize the  incoming data (from the perspective of the IO controller) 
  * for the functional submodules and set the local status to "GOOD". For all             
  * non-functional submodules, the local status should be set to "BAD"                    
  * We have already initialized the local buffer in "PNIO_cbf_ar_info_ind" callback      */
  
  dwErrorCode = PNIO_initiate_data_write(g_dwHandle);
  if(dwErrorCode != PNIO_OK)
    printf("Error: PNIO_init_data_write. 0x%x\n", dwErrorCode);
    
  /* We also have to call "PNIO_initiate_data_read" so that the IO Base Device user program  
   * can set the local status for functional submodules of all the outgoing data             
   * (from the perspective of the IO controller) to GOOD. For all non-functional submodules, 
   * the local status has to be set to "BAD".                                                */
  
  dwErrorCode = PNIO_initiate_data_read(g_dwHandle);
   if(dwErrorCode != PNIO_OK)
    printf("Error: PNIO_init_data_read. 0x%x\n", dwErrorCode);
   
  /* Here we need to call PNIO_set_appl_state_ready so that                                  
   * the IO Base Device user program registers a list of the non-functional submodules       
   * and the extent of readiness to get into a data exchange at the IO controller            */ 
  
  memset(&readyListType, 0, sizeof(readyListType));
  readyListType.ap_list.Flink = NULL;
  readyListType.ap_list.Blink = NULL;
  
  dwErrorCode = PNIO_set_appl_state_ready(
      g_dwHandle,
      g_arNumber,
      g_SessionKey,
      &readyListType);

  if(dwErrorCode == PNIO_OK)
        printf("\nDevice is ready...\n\n");
    else
        printf("\nError in setting appl state ready\n");
}

/******************************************************************************
 *                                                           
 * Function:             do_after_prm_end_ind_cbf()                
 *                                                           
 ******************************************************************************
 * This function is called after the PNIO_cbf_ar_indata_ind callback has been
 * called, it calls PNIO_initiate_data_write and PNIO_initiate_data_read()    
 *****************************************************************************/
void do_after_indata_ind_cbf(void)
{
  PNIO_UINT32 dwErrorCode = PNIO_OK;
  /* Here we need to call "PNIO_initiate_data_write" so that the IO Base Device user       
   * program can initialize the  incoming data (from the perspective of the IO controller) 
   * for the functional submodules and set the local status to "GOOD". For all             
   * non-functional submodules, the local status should be set to "BAD"                    
   * We have already initialized the local buffer in "PNIO_cbf_ar_info_ind" callback       */
  
  dwErrorCode = PNIO_initiate_data_write(g_dwHandle);
    if(dwErrorCode != PNIO_OK) {
        printf("## Error - PNIO_init_data_write. 0x%x\n", dwErrorCode);
    }
  /* We also have to call "PNIO_initiate_data_read" so that the IO Base Device user program   
   * can set the local status for functional submodules of all the outgoing data              
   * (from the perspective of the IO controller) to GOOD. For all non-functional submodules,  
   * the local status has to be set to "BAD".                                                 */
  
  dwErrorCode = PNIO_initiate_data_read(g_dwHandle);
    if(dwErrorCode != PNIO_OK) {
        printf("## Error - PNIO_init_data_read. 0x%x\n", dwErrorCode);
    }
}

/****************************************************************************** 
 *                                                           
 * Function:                GetSubmodNum()                            
 *                                                           
 ******************************************************************************
 * The function returns the index in the IO data array for the given module                                          
 *****************************************************************************/
int GetSubmodNum(PNIO_UINT32 mod, PNIO_UINT32 sub)
{
  int i;
  int j;
  
  for(i = 0; i < g_dwArraySize; i++)		//Look for module index
  {
    if((int)mod == idxTbl[i])
      break;
  }
  
  if(i == g_dwArraySize)                       //no module means also no submodule
    return -1;
  
  for(j = 0; j < g_device_data[i].maxSubslots; j++)
  {
    if(g_device_data[i+j].subslot == (int)sub)  //find submodule index
      return j;
  }
  
  return -1;
}

/******************************************************************************
 *                                                           
 * Function:                     Initialize()                              
 *                                                           
 ******************************************************************************
 * The function does the initialization of the PNIO device registration of 
 * callbacks is part of initialization       
 *****************************************************************************/
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
         
  printf("--------------------------------------------------------------------------------\n\n");
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

/****************************************************************************** 
 *                                                           
 * Function:                   UnInitialize()                            
 *                                                           
 ******************************************************************************
 * The function does the deinitialization of the PNIO device 
 *****************************************************************************/
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

/****************************************************************************** 
 *                                                           
 * Function:                      AddApi()                                  
 *                                                           
 ******************************************************************************
 * The function adds all the api stated in configuration structure for each 
 * module/submodule                       
 *****************************************************************************/
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
  
    if(i == j)   /* not added, add a new api */
    { 
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
  
      printf("Adding API profile: ");
  
      dwErrorCode = PNIO_api_add(
	/*in*/ g_dwHandle,
	/*in*/ api,
	/*in*/ (PNIO_UINT16) highestSlotsNumber,
	/*in*/ (PNIO_UINT16) highestSubslotNumber
	      );
  
      if(dwErrorCode != PNIO_OK)
      {
	printf("Error 0x%x\n", (int) dwErrorCode);
	exit(1);	
      }
      else
	printf("SUCCESS\n");
    }
  }
  printf("\n--------------------------------------------------------------------------------\n\n");
}

/****************************************************************************** 
 *                                                            
 * Function:                    RemoveApi()                               
 *                                                           
 ******************************************************************************
 * The function remove all the api stated in configuration structure                                                 
 *****************************************************************************/
void RemoveApi(void)
{
  int i;
  int j;
  PNIO_UINT32 api;
  PNIO_UINT32 dwErrorCode = PNIO_OK;
  
  printf("Removing Api: ");
  //for each slot
  for(i = j = 0; i < g_dwArraySize && dwErrorCode == PNIO_OK; i++)
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
      {
        printf("Error\n");
	exit(1);
      }
      else     
        printf("SUCCESS\n");
    }
  }
}

/****************************************************************************** 
 *                                                           
 * Function:        AddModSubMod()                            
 *                                                           
 ******************************************************************************
 * The function adds all the modules and submodules in serial order                                    
 *****************************************************************************/
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
  printf("Pluging module 0: ");
  addr.u.Geo.Slot    = g_device_data[0].slot;    //plug module 0
  addr.u.Geo.Subslot = g_device_data[0].subslot; //get the corresponding sub-slot
  
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
  
  printf("Module 0 info: api = %u, slot = %d, subslot = %d, max_slots = %d, mod_id = 0x%x\n",
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
  printf("Pluging submodule 0 to module 0...");
  dwErrorCode = PNIO_sub_plug (
     /*in*/ g_dwHandle,                   /* device handle */
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
  
    if(!g_device_data[0].subState) {
        printf("ERROR: Failure in plugging the submodule corresponding to module 0\n");
        printf(" -> no other module / submodule will be plugged...\n");
	exit(1);
    }
  
  
  
  //----------------------------------------------------
  //Add all modules
  //----------------------------------------------------
  if(NUMOF_SLOTS > 1)
  {
    printf("Pluging another modules, submodules...\n"); 
  
    for(i = 1; i < g_dwArraySize;)
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
      
      printf(" api = %u, slot = %u, subslot = %d, max_slots=%d, mod_id=0x%x\n",
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
  }
  
  //if not all the modules/submodules are plugger correctyl, print warning
  for(i = 0; i < g_dwArraySize; i++)
  {
    if(g_device_data[i].subState == 0)
    {
      printf("WARNING: Not all modules or submodules were plugged correctly!!\n");
      break;
    }
  }
  printf("\n--------------------------------------------------------------------------------\n\n");
}

/****************************************************************************** 
 *                                                           
 * Function:                    RemoveModSubMod()                         
 *                                                           
 ******************************************************************************
 * The function first removes the submodules and then the modules from PNIO 
 * device in reverse order                 
 *****************************************************************************/
void RemoveModSubMod(void)
{
  int i;
  PNIO_DEV_ADDR addr;    //location module/submodule
  PNIO_UINT32 dwErrorCode = PNIO_OK; 
  
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
      printf("Removing submodule: ");
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
      
        printf("Submodule info: api=%u, slot=%d, subslot=%d, max_slots=%d, mod_id=0x%x\n",
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
      printf("Removing module: ");
      dwErrorCode = PNIO_mod_pull(g_dwHandle, g_device_data[i].api, &addr);
      
       if(dwErrorCode == PNIO_OK)
      {
	printf("SUCCESS\n");
	g_device_data[i].subState = 0;
      }
      else
	printf("Error 0x%x\n", (int) dwErrorCode);
      
      printf("Module info: api=%u, slot=%d, subslot=%d, max_slots=%d, mod_id=0x%x\n",
                g_device_data[i].api,
                g_device_data[i].slot,
                g_device_data[i].subslot,
                g_device_data[i].maxSubslots,
                g_device_data[i].modId);
      
      dwErrorCode = PNIO_set_dev_state(g_dwHandle, PNIO_DEVSTAT_STATION_PROBLEM);     
    }      
  }  
}

/****************************************************************************** 
 *                                                           
 * Function:                  PrintIOData()                        
 *                                                           
 ******************************************************************************
 * The function displays the buffer contents to the terminal       
 ******************************************************************************/
void PrintIOData()
{
  int SlotNum;
  int SubNum;
  PNIO_UINT32 IoInd;
  
  int i;
  
  printf("\nOutput data...\n");
  //loop corresponding to slots
  for(i = 0; i < DEVICE_DATA_ENTRIES; i++)
  {
    SlotNum = g_device_data[i].slot;
    SubNum  = g_device_data[i].subslot;
    
    if(OutDatLen[SlotNum][SubNum] != 0)
    {
      printf("\nRead Module: Slot = %u, Subslot = %u, DataLen = %u, Local (consumer) status = %u\n",
                SlotNum, SubNum, OutDatLen[SlotNum][SubNum], OutDatIocs[SlotNum][SubNum]);
      
      for(IoInd = 0; IoInd < OutDatLen[SlotNum][SubNum]; IoInd++)
      {
	if(IoInd % 16 == 0 && IoInd != 0)
	  printf("\n");
	printf(" 0x%02x", (unsigned int)OutData[SlotNum][SubNum][IoInd]);
      }
      printf("\n");

    }
  }
  
  printf("\nInput data..\n");
  //loop corresponding to the slots
  for(i = 0; i < DEVICE_DATA_ENTRIES;i++)
  {
    SlotNum = g_device_data[i].slot;
    SubNum  = g_device_data[i].subslot;
    
    if(InDatLen[SlotNum][SubNum] != 0)
    {
      printf("\nWrite Module: Slot = %u, Subslot = %u, DataLen = %u, Local (provider) status = %u\n",
                SlotNum, SubNum, InDatLen[SlotNum][SubNum], InDatIops[SlotNum][SubNum]);
      
      for(IoInd=0; IoInd < InDatLen[SlotNum][SubNum]; IoInd++) 
      {
        if(IoInd % 16 == 0 && IoInd != 0)
          printf("\n");

	printf(" 0x%02x", (unsigned int)InData[SlotNum][SubNum][IoInd]);
      }
      printf("\n");
    }
  }
}

/****************************************************************************** 
 *                                                           
 * Function:                 ConfigureDeviceData()                     
 *                                                           
 ******************************************************************************
 * This function creates a structure out of the config data and fills the 
 * unfilled members whereever necesary.        
 *****************************************************************************/
void ConfigureDeviceData(void)
{
  unsigned int i = 0;
  unsigned int beginNewSlot = 0;
  unsigned int idx = 0;
  
  //copy predeffined structure to g_device_data as it is
  g_device_data = device_data;
  for(i = 0; i < DEVICE_DATA_ENTRIES; i++)
  {  
    printf("Data structure.slot[%d]: slot = %u  subslot = %u  mod_id = 0x%x  subslot_id = %u \n", 
	   i,
	   g_device_data[i].slot,
	   g_device_data[i].subslot,
	   (unsigned int) g_device_data[i].modId,
	   (unsigned int) g_device_data[i].subslotId
	  );
  }
  
  //fill idxTbl with -1
  memset(idxTbl, -1, DEVICE_DATA_ENTRIES * sizeof(int));
  idxTbl[idx++] = g_device_data[0].slot;
    
  //browsing through the device data structure
  for(i = 0; i < DEVICE_DATA_ENTRIES; i++)
  {
    if(g_device_data[i].slot == g_device_data[beginNewSlot].slot)
    {
      g_device_data[beginNewSlot].maxSubslots++;
      g_device_data[i].modId = g_device_data[beginNewSlot].modId;
    }
    else
    {
      beginNewSlot = i;					//index corresponding to the beginning of the new slots
      g_device_data[beginNewSlot].maxSubslots = 1;	//every new module/slot has min one sub-slot
      idxTbl[idx++] = g_device_data[i].slot;		//store the entry of the new slot in idxTbl
    }
  }
 
}

/****************************************************************************** 
 *                                                           
 * 				 MAIN                   
 *                                                           
 ******************************************************************************/


int main(void)
{
  PNIO_UINT32 ErrorCode = PNIO_OK; 
  int callback_counter = 0;
  int ch = 0;
   
  int i, slot, subslot;  /* i corresponds to every element in g_device_data */
  char data[50] = {0};

  
  printf("--------------------------------------------------------------------------------\n\n");
  printf("Application does following: \n");
  printf("Initialize PNIO device\n");
  printf("Waits for user command to Uninitialize\n");
  printf("Uninitialize CP\n\n");
  printf("--------------------------------------------------------------------------------\n\n");
  
  //----------------------------------------------------
  //Initialize
  //----------------------------------------------------
  ConfigureDeviceData();	
  g_dwHandle = Initialize(g_dwCpId);
  AddApi();
  AddModSubMod();
  
  //----------------------------------------------------
  //Start the CP
  //----------------------------------------------------
  printf("Starting PNIO device: ");
  ErrorCode = PNIO_device_start(g_dwHandle);
  if(ErrorCode != PNIO_OK)
  {   printf("Error: 0x%x\n", ErrorCode);
     goto CLEAN_UP;
  }
  else
  {
    printf("SUCCESS\n");
  }
 
  printf("Setting device state OK: ");
  ErrorCode = PNIO_set_dev_state(g_dwHandle, PNIO_DEVSTAT_OK);
  if(ErrorCode != PNIO_OK)
  {
    printf("Error: 0x%x\n", ErrorCode);
    goto CLEAN_UP;
  }
  else
  {
    printf("SUCCESS\n");
  }
    
  //----------------------------------------------------
  //Wait for callbacks
  //----------------------------------------------------  
  printf("Waiting for callbacks...\n");
  
  //Wait for necessary callbacks
  while(callback_counter != MAX_COUNT)
  {
    //PNIO_cbf_prm_end_ind() callback already called?
    if(PRM_END_IND_flag == 1)
    {
      do_after_prm_end_ind_cbf();
      break;
    }
    usleep(100000);
    callback_counter++;
  }
  
  if(callback_counter == MAX_COUNT) 
    goto CLEAN_UP;
  
  callback_counter = 0;  
  while(callback_counter != MAX_COUNT)
  {
    //PNIO_cbf_indata_ind() callback already called?
    if(INDATA_IND_flag == 1)
    {
      do_after_indata_ind_cbf();
      break;
    }
    
    usleep(100000);
    callback_counter++;
  }
    
  if(callback_counter == MAX_COUNT)
     goto CLEAN_UP;
    
  printf("All necessary callbacks called, press 'q' to close the device...\n");
    
  //----------------------------------------------------
  //Initialize read/write buffers 
  //----------------------------------------------------
  for(i = 0; i < g_dwArraySize; i++)
  {
    //set initial values for input data
    memset(data, 0 , 50);
    
    slot = g_device_data[i].slot;
    subslot = g_device_data[i].subslot;
      
    sprintf(data,"SLOT_ID=%d, SUBSLOT_ID=%d, DATA=%d.%d",
                    slot, subslot, slot, subslot);
    memcpy(InData[slot][subslot], data, 50);
    
  }
  PrintIOData();
  
  
  
  
  //loop to wait for the user input
  do{
    usleep(100000);
    ch = getchar();
  } while(((int)'q' != ch) && ((int)'Q' != ch));

  
  //----------------------------------------------------
  //CLEAN UP
  //----------------------------------------------------
  
CLEAN_UP:
  printf("\n--------------------------------------------------------------------------------\n\n");
  printf("Stopping PNIO device...\n");
  
  ErrorCode = PNIO_device_stop(g_dwHandle);
  if(ErrorCode != PNIO_OK)
  {
    printf("Error in stopping the device. Error# 0x%x\n", ErrorCode);
    exit(1);
  }
  
  //wait for OFFLINE_IND_flag to be set by PNIO_cbf_ar_offline_ind() callback
  do{
    usleep(100000);
  }while(OFFLINE_IND_flag == 0);
  
  //Remove the modules and submodules
  RemoveModSubMod();
  
  //Remove the API
  RemoveApi();
  
  //Uninitialize
  UnInitialize(g_dwHandle);
  
  //set handle to invalid values
  g_dwHandle = 0; 
  

  
}