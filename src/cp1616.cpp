/*********************************************************************************************//**
* @file cp1616.cpp
* 
* Main cp1616 node
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

#include <cp1616/cp1616.h>

//=============================================================================
//Constructor
//=============================================================================
cp1616_io_controller::cp1616_io_controller() :
    CpReady(0),
    SemModChange(0),
    CpId(1),
    CpHandle(0),
    CpCurrentMode(PNIO_MODE_OFFLINE),
    CpLocalState(PNIO_S_GOOD),
    CpStopRequest(false),
    deviceInputCount(0),
    deviceOutputCount(0),
    totalInputSize(0),
    totalOutputSize(0)
{
    //Allocate memory for Input module
    deviceInputLength  = new PNIO_UINT32 [NUM_OF_INPUT_MODULES * sizeof(PNIO_UINT32)];
    deviceInputState   = new PNIO_IOXS volatile [NUM_OF_INPUT_MODULES * sizeof(PNIO_IOXS)];
    deviceInputAddress = new PNIO_ADDR [NUM_OF_INPUT_MODULES * sizeof(PNIO_ADDR)];

    //Allocate memory for Output module
    deviceOutputLength  = new PNIO_UINT32 [NUM_OF_OUTPUT_MODULES * sizeof(PNIO_UINT32)];
    deviceOutputState   = new PNIO_IOXS volatile [NUM_OF_OUTPUT_MODULES * sizeof(PNIO_IOXS)];
    deviceOutputAddress = new PNIO_ADDR [NUM_OF_OUTPUT_MODULES * sizeof(PNIO_ADDR)];
}

//=============================================================================
//Destructor
//=============================================================================
cp1616_io_controller::~cp1616_io_controller()
{

    if(deviceInputCount > 0)
    {
        delete InModuleData;
        delete InData;
    }

    if(deviceOutputCount > 0)
    {
        delete OutModuleData;
        delete OutData;
    }
    delete deviceInputLength;
    delete deviceInputState;
    delete deviceInputAddress;

    delete deviceOutputLength;
    delete deviceOutputState;
    delete deviceOutputAddress;
}

//=============================================================================
//addIntputModule
//=============================================================================
int
cp1616_io_controller::addInputModule(unsigned int I_size, unsigned int I_address)
{
    std::cout << "Adding IO module: ";

    if(CpCurrentMode == PNIO_MODE_OPERATE)
        std::cout << "Error: not able to add Input module in Operate state!" << std::endl;
    else if(deviceInputCount >= NUM_OF_INPUT_MODULES)
        std::cout << "Error: Not able to add antoher module. Max count reached!" << std::endl;
    else
    {
        //Set variables required for PNIO_data_read function
        deviceInputLength[deviceInputCount] = I_size;                   //number of transferred bytes
        deviceInputState[deviceInputCount]  = PNIO_S_BAD;               //initial Input State
        deviceInputAddress[deviceInputCount].AddrType = PNIO_ADDR_LOG;  //Address type
        deviceInputAddress[deviceInputCount].IODataType = PNIO_IO_IN;   //Data type
        deviceInputAddress[deviceInputCount].u.Addr = I_address;        //Module memory address


        std::cout << "IN:  Size: " << std::setfill(' ') << std::setw(3)
                  << deviceInputLength[deviceInputCount]
                  << " I: " << deviceInputAddress[deviceInputCount].u.Addr
                  << " - "  << deviceInputAddress[deviceInputCount].u.Addr + deviceInputLength[deviceInputCount] - 1;


    deviceInputCount++;             //increment deviceInputCount
    totalInputSize += I_size;       //save overall Input Size

    std::cout << " ... done" << std::endl;
    }
}

//=============================================================================
//addOutputModule
//=============================================================================
int
cp1616_io_controller::addOutputModule(unsigned int Q_size, unsigned int Q_address)
{
    std::cout << "Adding IO module: ";

    if(CpCurrentMode == PNIO_MODE_OPERATE)
        std::cout << "Error: not able to add Output module in Operate state!" << std::endl;
    else if(deviceOutputCount >= NUM_OF_INPUT_MODULES)
        std::cout << "Error: Not able to add antoher module. Max count reached!" << std::endl;
    else
    {
        //Set variables required for PNIO_data_write function
        deviceOutputLength[deviceOutputCount] = Q_size;                   //number of transferred bytes
        deviceOutputState[deviceOutputCount]  = PNIO_S_BAD;               //initial Input State
        deviceOutputAddress[deviceOutputCount].AddrType = PNIO_ADDR_LOG;  //Address type
        deviceOutputAddress[deviceOutputCount].IODataType = PNIO_IO_OUT;  //Data type
        deviceOutputAddress[deviceOutputCount].u.Addr = Q_address;        //Module memory address


        std::cout << "Out: Size: " << std::setfill(' ') << std::setw(3)
                  << deviceOutputLength[deviceOutputCount]
                  << " Q: " << deviceOutputAddress[deviceOutputCount].u.Addr
                  << " - " << deviceOutputAddress[deviceOutputCount].u.Addr + deviceOutputLength[deviceOutputCount] - 1;


        deviceOutputCount++;             //increment deviceOutputCount
        totalOutputSize += Q_size;       //save overall Output Size

        std::cout << " ... done" << std::endl;
    }
}

//=============================================================================
//init - Initialization of PNIO controller,
//=============================================================================
int
cp1616_io_controller::init()
{
    PNIO_UINT32 ErrorCode = PNIO_OK;

    //-------------------------------------------------------------------------
    //Prepare 2D array for registered input modules data
    //-------------------------------------------------------------------------
    if(deviceInputCount > 0)    //if any input module available
    {
        //Memory allocation
        InModuleData = new PNIO_UINT8 [totalInputSize * sizeof(PNIO_UINT8)];
        InData       = new PNIO_UINT8* [deviceInputCount * sizeof(PNIO_UINT8)];

        //Set initial Input values to zero
        memset(InModuleData, 0, totalInputSize);

        unsigned int p1, p2;

        //Assign array pointers
        for(p1 = 0, p2 = 0; p2 < deviceInputCount; )
        {
            InData[p2] = &(InModuleData[p1]);
            p1 += deviceInputLength[p2++];
        }

        //Print Input data structure
        std::cout << std::endl << "Input Data Array: [Total size: " << totalInputSize << " bytes]" << std::endl;;
        for(int i = 0; i < deviceInputCount; i++)
        {
            std::cout << "I: " << deviceInputAddress[i].u.Addr
                      << " - " << deviceInputAddress[i].u.Addr + deviceInputLength[i] - 1
                      << ": InData "  << "[" << i << "]" << "[0]" ;
            if((deviceInputLength[i]-1) > 0)
                std::cout << "... [" << i << "]" << "[" << deviceInputLength[i]-1 << "]" << std::endl;
        }
    }

    //-------------------------------------------------------------------------
    //Prepare 2D array for for registered output modules data
    //-------------------------------------------------------------------------
    if(deviceOutputCount > 0)    //if any output module available
    {
        //Memory allocation
        OutModuleData = new PNIO_UINT8 [totalOutputSize * sizeof(PNIO_UINT8)];
        OutData       = new PNIO_UINT8* [deviceOutputCount * sizeof(PNIO_UINT8)];

        //Set initial Output values to zero
        memset(OutModuleData, 0, totalOutputSize);

        unsigned int p1, p2;

        //Assign array pointers
        for(p1 = 0, p2 = 0; p2 < deviceOutputCount; )
        {
            OutData[p2] = &(OutModuleData[p1]);
            p1 += deviceOutputLength[p2++];
        }

        //Print Output data structure
        std::cout << std::endl << "Output Data Array: [Total size: " << totalOutputSize << " bytes]" << std::endl;;
        for(int i = 0; i < deviceOutputCount; i++)
        {
            std::cout << "I: " << deviceOutputAddress[i].u.Addr
                      << " - " << deviceOutputAddress[i].u.Addr + deviceOutputLength[i] - 1
                      << ": OutData "  << "[" << i << "]" << "[0]" ;
            if((deviceOutputLength[i]-1) > 0)
                std::cout << "... [" << i << "]" << "[" << deviceOutputLength[i]-1 << "]" << std::endl;
        }
    }

    //-------------------------------------------------------------------------
    //Open PNIO_Controller
    //-------------------------------------------------------------------------
    std::cout << std::endl << "Openning CP1616 in PNIO_controller mode: ";

    //Connect to CP and obtain handle
    ErrorCode = PNIO_controller_open(
                CpId,
                PNIO_CEP_MODE_CTRL,
                &cp1616_io_controller::callback_for_ds_read_conf,
                &cp1616_io_controller::callback_for_ds_write_conf,
                &cp1616_io_controller::callback_for_alarm_indication,
                &CpHandle);

    //Check errors
    if(ErrorCode != PNIO_OK)
    {
        std::cout << "Error: 0x" << std::hex << (int)ErrorCode << std::endl;
        exit(EXIT_FAILURE);
    }
    else
        std::cout << "done" << std::endl;

    //---------------------------------------------------------------------------
    //register PNIO_CBE_MODE_IND callback for mode changes confirmation
    //---------------------------------------------------------------------------
    ErrorCode = PNIO_register_cbf(
                CpHandle,
                PNIO_CBE_MODE_IND,
                &cp1616_io_controller::callback_for_mode_change_indication);

    //Check errors
    if(ErrorCode != PNIO_OK)
    {
        std::cout << "Error in PNIO_register_cbf: callback_for_mode_change_indication: 0x" << (int)ErrorCode << std::endl;
        PNIO_close(CpHandle);
        exit(EXIT_FAILURE);
    }

   //---------------------------------------------------------------------------
   //register the callback PNIO_CBE_DEV_ACT for device activation confirmation
   //---------------------------------------------------------------------------
   ErrorCode = PNIO_register_cbf(
        /*in*/ CpHandle,
        /*in*/ PNIO_CBE_DEV_ACT_CONF,
        /*in*/ &cp1616_io_controller::callback_for_device_activation);

    if(ErrorCode != PNIO_OK)
    {
        std::cout << "Error in PNIO_register_cbf: callback_for_device_activation: 0x" << (int)ErrorCode << std::endl;
        PNIO_close(CpHandle);
        exit(EXIT_FAILURE);
    }

    //---------------------------------------------------------------------------
    //register the callback PNIO_CBE_CP_STOP_REQ to stop the device
    //---------------------------------------------------------------------------
    ErrorCode = PNIO_register_cbf(
            /*in */ CpHandle,
            /*in */ PNIO_CBE_CP_STOP_REQ,
            /*in */ &cp1616_io_controller::callback_for_cp_stop_req);

    if(ErrorCode != PNIO_OK)
    {
        std::cout << "Error in PNIO_register_cbf: callback_for_cp_stop_req: 0x" << (int)ErrorCode << std::endl;
        PNIO_close(CpHandle);
        exit(EXIT_FAILURE);
    }

    //---------------------------------------------------------------------------
    //Change CP mode to Operate
    //---------------------------------------------------------------------------
    changeAndWaitForPnioMode(PNIO_MODE_OPERATE);

    //---------------------------------------------------------------------------
    //Start communication
    //---------------------------------------------------------------------------
    std::cout << "Starting communication: ";
    int cnt = 0;

    while(CpReady == 0) //Wait for CpReady flag to be set by callback_for_alarm_indication
    {
        cnt++;
        usleep(100000);
        if(cnt == MAX_NUM_OF_INIT_ATTEMPTS)
        {
            std::cout << "Not able to start communication, Uninitializing..." << std::endl;
            break;
        }
    }
    std::cout << "done" << std::endl;
    std::cout << "--------------------------------------------------------------------------------" << std::endl << std::endl;


    //if everything ok return 0;
    return 0;
}

//=============================================================================
//uinit - UnInitialize PNIO controller
//=============================================================================
int
cp1616_io_controller::uinit()
{
    PNIO_UINT32 ErrorCode = PNIO_OK;

    //Change CP mode to OFFLINE
    changeAndWaitForPnioMode(PNIO_MODE_OFFLINE);

    std::cout << "Closing PNIO_controller: ";
    ErrorCode = PNIO_close(CpHandle);

    if(ErrorCode != PNIO_OK)
    {
        std::cout << "Error 0x" << std::hex << (int)ErrorCode << std::endl;
        exit(EXIT_FAILURE);
    }
    else
    {
        std::cout << "done" << std::endl;
        return 0;
    }
}

//=============================================================================
//ChangeAndWaitForPnioMode - change operational mode of CP
//=============================================================================
void
cp1616_io_controller::changeAndWaitForPnioMode(PNIO_MODE_TYPE mode)
{
    PNIO_UINT32 ErrorCode;
    PNIO_UINT32 InputCpHandle = CpHandle;

    std::cout << "Changing PNIO mode: ";

    //set required mode
    ErrorCode = PNIO_set_mode(CpHandle, mode);

    if(ErrorCode != PNIO_OK)
    {
        std::cout << "ERROR: 0x" << std::hex << (int)ErrorCode << std::endl;
        PNIO_close(CpHandle);
        exit(EXIT_FAILURE);
    }


    if(CpHandle == InputCpHandle)  //check if CpHandle still valid
    {
        //wait for a callback_for_mode_change_indication
        while(!SemModChange){
        usleep(100000);
    }
    SemModChange = 0;

    //check if the current mode is correct
    if(CpCurrentMode != mode)
    {
      std::cout << "ERROR : recieved another mode" << std::endl;
      PNIO_close(CpHandle);
      exit(EXIT_FAILURE);
    }
    else
      std::cout << "done" << std::endl;
  }
}

//=============================================================================
//updateCyclicInputData
//=============================================================================
PNIO_UINT32
cp1616_io_controller::updateCyclicInputData()
{
    PNIO_UINT32 ErrorCode;
    PNIO_UINT32 BytesRead;

    unsigned int i;

    for(i = 0;i < deviceInputCount; i++)
    {
        ErrorCode=PNIO_data_read(
                /*in*/  CpHandle,                               // handle
                /*in*/  &deviceInputAddress[i],                 // pointer to device input address
                /*in*/  deviceInputLength[i],                   // length in bytes of input
                /*out*/ &BytesRead,                              // number of bytes read
                /*in*/  InData[i],                              // pointer to input data
                /*in*/  CpLocalState,                           // local status
                /*out*/(PNIO_IOXS*)&(deviceInputState[i]));     // remote status

    #ifdef DEBUG
        if(ErrorCode != PNIO_OK)
        {
          printf("Error in UpdateCyclicInputData \n");
          printf("PNIO_read_data (PNIO_CBE_DEV_ACT_CONF,..) returned 0x%x\n", (int)dwErrorCode);
        }
    #endif
    }
    return(ErrorCode);
}

//=============================================================================
//updateCyclicOutputData
//=============================================================================
PNIO_UINT32
cp1616_io_controller::updateCyclicOutputData()
{
    PNIO_UINT32 ErrorCode;

    unsigned int i;

    for(i = 0;i < deviceOutputCount; i++)
    {
        ErrorCode=PNIO_data_write(
                /*in*/  CpHandle,                               // handle
                /*in*/  &deviceOutputAddress[i],                // pointer to device output address
                /*in*/  deviceOutputLength[i],                  // length in bytes of output
                /*in*/  OutData[i],                             // pointer to output data
                /*in*/  CpLocalState,                           // local status
                /*out*/(PNIO_IOXS*)&(deviceOutputState[i]));    // remote status

    #ifdef DEBUG
        if(ErrorCode != PNIO_OK)
        {
          printf("Error in UpdateCyclicInputData \n");
          printf("PNIO_read_data (PNIO_CBE_DEV_ACT_CONF,..) returned 0x%x\n", (int)dwErrorCode);
        }
    #endif
    }
    return(ErrorCode);
}

//=============================================================================
//printOutputData
//=============================================================================
void
cp1616_io_controller::printOutputData(unsigned int module)
{
    std::cout << "Output m." << module << std::dec
              << " [Q: " << deviceOutputAddress[module].u.Addr
              << " - "  << deviceOutputAddress[module].u.Addr + deviceOutputLength[module]-1
              << "]: ";

    for(int i = 0; i < deviceOutputLength[module]; i++)
        std::cout <<  std::setfill(' ') << std::setw(2) << std::hex << (int)OutData[module][i] << " ";

    std::cout << std::endl;
}

//=============================================================================
//printInputData
//=============================================================================
void
cp1616_io_controller::printInputData(unsigned int module)
{
    std::cout << "Input  m." << module << std::dec
               << " [I: " << deviceInputAddress[module].u.Addr
               << " - "  << deviceInputAddress[module].u.Addr + deviceInputLength[module]-1
               << "]: ";

    for(int i = 0; i < deviceInputLength[module]; i++)
        std::cout <<  std::setfill(' ') << std::setw(2) << std::hex << (int)InData[module][i] << " ";

    std::cout << std::endl;
}


//=============================================================================
//setOutputData
//=============================================================================
void
cp1616_io_controller::setOutData(unsigned int module, unsigned int data_index, PNIO_UINT8 value)
{
    //Check if index argumnets correspond with OutData Array
    if(module >= deviceOutputCount)
        std::cout << "#Error: setOutData: Non exisitng item in OutData array, check module='"<< module <<"'!" << std::endl;
    else if (data_index >= deviceOutputLength[module])
        std::cout << "#Error: setOutData: Non existing item in OutData array, check data_index='" << data_index << "'!" << std::endl;
    else
        OutData[module][data_index] = value;
}

//=============================================================================
//Callbacks
//=============================================================================

void
cp1616_io_controller::callback_for_ds_read_conf(PNIO_CBE_PRM *pCbfPrm)
{
    std::cout << "callback_for_ds_read_conf" << std::endl;
    std::cout << "this callback must not occur in this sample application" << std::endl;
}

void
cp1616_io_controller::callback_for_ds_write_conf(PNIO_CBE_PRM* pCbfPrm)
{
    std::cout << "callback_for_ds_write_conf" << std::endl;
    std::cout << "this callback must not occur in this sample application" << std::endl;
}

void
cp1616_io_controller::callback_for_mode_change_indication(PNIO_CBE_PRM *pCbfPrm)
{
    //Create CallbackHandler object to access cp1616_io_controller variables from static member function
    cp1616_io_controller* CallbackHandler = (cp1616_io_controller*)cp1616_object;

    if(pCbfPrm->CbeType == PNIO_CBE_MODE_IND) /* Check callback type */
    {
        switch (pCbfPrm->ModeInd.Mode)
        {
            case PNIO_MODE_OFFLINE:
                std::cout << "request-> OFFLINE: ";
                CallbackHandler->CpCurrentMode = PNIO_MODE_OFFLINE;
                break;
            case PNIO_MODE_CLEAR:
                std::cout << "request-> CLEAR: ";
                CallbackHandler->CpCurrentMode = PNIO_MODE_CLEAR;
                break;
            case PNIO_MODE_OPERATE:
                std::cout << "request-> OPERATE: ";
                CallbackHandler->CpCurrentMode = PNIO_MODE_OPERATE;
                break;
            default:
                std::cout << "Wrong mode selected: ";
                break;
    };

    //send notification
    CallbackHandler->SemModChange = 1;
  }
}

void
cp1616_io_controller::callback_for_alarm_indication(PNIO_CBE_PRM *pCbfPrm)
{
    //Create CallbackHandler object to access cp1616_io_controller variables from static member function
    cp1616_io_controller* CallbackHandler = (cp1616_io_controller*)cp1616_object;

    if(pCbfPrm->CbeType==PNIO_CBE_ALARM_IND) /* Check callback type */
    {
        switch (pCbfPrm->AlarmInd.pAlarmData->AlarmType)
        {
            case PNIO_ALARM_DIAGNOSTIC:
                std::cout << "PNIO_ALARM_DIAGNOSTIC" << std::endl;
                break;
            case PNIO_ALARM_PROCESS:
                std::cout << "PNIO_ALARM_DIAGNOSTIC" << std::endl;
                break;
            case PNIO_ALARM_PULL:
                std::cout << "PNIO_ALARM_PULL" << std::endl;
                break;
            case PNIO_ALARM_PLUG:
                std::cout << "PNIO_ALARM_PLUG" << std::endl;
                break;
            case PNIO_ALARM_STATUS:
                std::cout << "PNIO_ALARM_STATUS" << std::endl;
                break;
            case PNIO_ALARM_UPDATE:
                std::cout << "PNIO_ALARM_UPDATE" << std::endl;
                break;
            case PNIO_ALARM_REDUNDANCY:
                std::cout << "PNIO_ALARM_REDUNDACY" << std::endl;
                break;
            case PNIO_ALARM_CONTROLLED_BY_SUPERVISOR:
                std::cout << "PNIO_ALARM_CONTROLLED_BY_SUPERVISOR" << std::endl;
                break;
            case PNIO_ALARM_RELEASED_BY_SUPERVISOR:
                std::cout << "PNIO_ALARM_RELEASED_BY_SUPERVISOR" << std::endl;
                break;
            case PNIO_ALARM_PLUG_WRONG:
                std::cout << "PNIO_ALARM_PLUG_WRONG" << std::endl;
                break;
            case PNIO_ALARM_RETURN_OF_SUBMODULE:
                std::cout << "PNIO_ALARM_RETURN_OF_SUBMODULE" << std::endl;
                break;
            case PNIO_ALARM_DEV_FAILURE:
#ifdef DEBUG
                 std::cout << "PNIO_ALARM_DEV_FAILURE" << std::endl;
#endif
                break;

            case PNIO_ALARM_DEV_RETURN:
#ifdef DEBUG
                std::cout << "PNIO_ALARM_DEV_RETURN" << std::endl;
#endif
                CallbackHandler->CpReady = 1;	//Set CpReady flag to notify application that CP is ready for communication
                break;
            default:
                std::cout << "callback_for_alarm_indication called with wrong type" << std::endl;
                break;
        }
    }
}

void
cp1616_io_controller::callback_for_device_activation(PNIO_CBE_PRM *pCbfPrm)
{
    switch( pCbfPrm->DevActConf.Mode)
    {
        case PNIO_DA_TRUE:
            std::cout << "device with Address " << std::hex << (int)pCbfPrm->DevActConf.pAddr->u.Addr
                      << " was activated with result " << (int)pCbfPrm->DevActConf.Result << std::endl;
            break;
        case PNIO_DA_FALSE:
            std::cout << "device with Address " << std::hex << (int)pCbfPrm->DevActConf.pAddr->u.Addr
                      << " was activated with result " << (int)pCbfPrm->DevActConf.Result << std::endl;
            break;
    };
}

void
cp1616_io_controller::callback_for_cp_stop_req(PNIO_CBE_PRM *pCbfPrm)
{
    //Create CallbackHandler object to access cp1616_io_controller variables from static member function
    cp1616_io_controller* CallbackHandler = (cp1616_io_controller*)cp1616_object;

    std::cout << "CP_STOP_REQUEST recieved, terminating program ..." << std::endl;
    CallbackHandler->CpStopRequest = true;
}


//=============================================================================
//MAIN
//=============================================================================

int main(int argc, char **argv)
{
    ros::init(argc,argv, "ros_profinet_experimental");
    ros::NodeHandle nh;
    
    std::cout << "--------------------------------------------------------------------------------" << std::endl;

    cp1616_io_controller cp1616;
    cp1616_object = (void *) &cp1616;  //assign global variable to handle callbacks

    //Add IO modules
    cp1616.addOutputModule(4,4116);
    cp1616.addOutputModule(4,4120);
    cp1616.addOutputModule(4,4124);
    cp1616.addOutputModule(16,4128);

    cp1616.addInputModule(4,4132);
    cp1616.addInputModule(4,4136);
    cp1616.addInputModule(4,4140);
    cp1616.addInputModule(16,4144);

    //Initialize CP
    cp1616.init();

    PNIO_UINT8 DO_value = 1;
    PNIO_UINT32 ErrorCode = PNIO_OK;

    //Shift values
    for(int i = 0; i < 20; i++)
    {
        for(int j = 0; j < 7; j++)
        {
            //Write Output data
            DO_value = DO_value << 1;
            cp1616.setOutData(0,0,DO_value);
            cp1616.printOutputData(0);
            cp1616.printOutputData(1);
            cp1616.printOutputData(2);
            cp1616.printOutputData(3);
            cp1616.printInputData(0);
            cp1616.printInputData(1);
            cp1616.printInputData(2);
            cp1616.printInputData(3);
            ErrorCode = cp1616.updateCyclicOutputData();
            if(ErrorCode != PNIO_OK)
                printf("Error 0x%x\n", (int)ErrorCode);
            usleep(100000);
        }

        for(int j = 0; j < 7; j++)
        {
            //Write Output data
            DO_value = DO_value >> 1;
            cp1616.setOutData(0,0,DO_value);
            cp1616.printOutputData(0);
            cp1616.printOutputData(1);
            cp1616.printOutputData(2);
            cp1616.printOutputData(3);
            cp1616.printInputData(0);
            cp1616.printInputData(1);
            cp1616.printInputData(2);
            cp1616.printInputData(3);
            ErrorCode = cp1616.updateCyclicOutputData();
            if(ErrorCode != PNIO_OK)
                printf("Error 0x%x\n", (int)ErrorCode);
            usleep(100000);
        }
    }

    cp1616.uinit();

    return(EXIT_SUCCESS);
}
