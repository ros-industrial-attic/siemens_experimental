/*********************************************************************************************//**
* @file ros_profinet_experimental_lib.h
* 
* ros_profinet_experimental header 
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
#ifndef CP1616_H
#define CP1616_H

#include <ros/ros.h>

//IO base headers
#include "pniousrx.h"
#include "pnioerrx.h"

#define NUM_OF_INPUT_MODULES    4       //Fixed definitions for development purposes
#define NUM_OF_OUTPUT_MODULES   4

#define MAX_NUM_OF_INIT_ATTEMPTS 1000   //Number of attempts to initialize communication

void* cp1616_object;   //global variable for handling Callbacks

//Abstract IO Controller class
class io_controller
{
public:

   virtual int addInputModule(unsigned int I_size, unsigned int I_address) = 0;
   virtual int addOutputModule(unsigned int Q_size, unsigned int Q_address) = 0;
   virtual int init() = 0;
   virtual int uinit() = 0;

private:


};

//CP1616 implementation
class cp1616_io_controller: public io_controller
{
public:
    explicit cp1616_io_controller();
    ~cp1616_io_controller();
    int init();
    int uinit();
    int addInputModule(unsigned int I_size, unsigned int I_address);
    int addOutputModule(unsigned int Q_size, unsigned int Q_address);

    PNIO_UINT32 updateCyclicInputData();
    PNIO_UINT32 updateCyclicOutputData();

    void changeAndWaitForPnioMode(PNIO_MODE_TYPE mode);
    void setOutData(unsigned int module, unsigned int data_index, PNIO_UINT8 value);

    void printOutputData(unsigned int module);
    void printInputData(unsigned int module);

private:

    //Callback functions required by IO Base library
    static void callback_for_ds_read_conf(PNIO_CBE_PRM *pCbfPrm);
    static void callback_for_ds_write_conf(PNIO_CBE_PRM *pCbfPrm);
    static void callback_for_alarm_indication(PNIO_CBE_PRM *pCbfPrm);

    static void callback_for_mode_change_indication(PNIO_CBE_PRM *pCbfPrm);
    static void callback_for_device_activation(PNIO_CBE_PRM *pCbfPrm);
    static void callback_for_cp_stop_req(PNIO_CBE_PRM *pCbfPrm);

    //CP variables
    int CpReady;
    int SemModChange;
    PNIO_UINT32 CpId;
    PNIO_UINT32 CpHandle;

    volatile PNIO_MODE_TYPE CpCurrentMode;
    volatile PNIO_IOXS      CpLocalState;
    volatile bool           CpStopRequest;

    //Input module variables
    PNIO_UINT32 deviceInputCount;
    PNIO_UINT32 *deviceInputLength;
    PNIO_IOXS volatile* volatile deviceInputState;
    PNIO_ADDR *deviceInputAddress;

    //Output modules variables
    PNIO_UINT32 deviceOutputCount;
    PNIO_UINT32 *deviceOutputLength;
    PNIO_IOXS volatile* volatile deviceOutputState;
    PNIO_ADDR *deviceOutputAddress;

    //Input Data variables
    PNIO_UINT8 *InModuleData;
    PNIO_UINT8 **InData;
    unsigned int totalInputSize;

    //Output Data variables
    PNIO_UINT8 *OutModuleData;
    PNIO_UINT8 **OutData;
    unsigned int totalOutputSize;
};



#endif //CP1616_H