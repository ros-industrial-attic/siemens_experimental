/*********************************************************************************************//**
* @file cp1616_main.cpp
*
* Testing code for both IO Controller and IO Device modes
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
#include <cp1616/cp1616_io_controller.h>
#include <cp1616/cp1616_io_controller_callbacks.h>
#include <cp1616/cp1616_io_device.h>
#include <cp1616/cp1616_io_device_callbacks.h>

#define IO_CONTROLLER_MODE
//#define IO_DEVICE_MODE

int main(int argc, char **argv)
{
  ros::init(argc,argv, "cp1616");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh_;

#ifdef IO_CONTROLLER_MODE

  cp1616::Cp1616IOController *cp1616_object;
  cp1616_object = cp1616::Cp1616IOController::getControllerInstance();

  //Add IO modules
  cp1616_object->addOutputModule(4,4116);
  cp1616_object->addOutputModule(4,4120);
  cp1616_object->addOutputModule(4,4124);
  cp1616_object->addOutputModule(16,4128);

  cp1616_object->addInputModule(4,4132);
  cp1616_object->addInputModule(4,4136);
  cp1616_object->addInputModule(4,4140);
  cp1616_object->addInputModule(16,4144);

  //Initialize CP
  cp1616_object->init();

  if(cp1616_object->getCpReady() != 0)
  {
    PNIO_UINT8 do_value = 1;
    PNIO_UINT32 error_code = PNIO_OK;

    //Shift values
    for(int i = 0; i < 20; i++)
    {
      for(int j = 0; j < 7; j++)
      {
        //Shift Output data
        do_value = do_value << 1;
        cp1616_object->output_module_data_[0][0] = do_value;

        //Print OutData, InData
        cp1616_object->printOutputData(0);
        cp1616_object->printOutputData(1);
        cp1616_object->printOutputData(2);
        cp1616_object->printOutputData(3);
        cp1616_object->printInputData(0);
        cp1616_object->printInputData(1);
        cp1616_object->printInputData(2);
        cp1616_object->printInputData(3);

        error_code = cp1616_object->updateCyclicOutputData();
        if(error_code != PNIO_OK)
        {
          ROS_INFO("Not able to update output data: Error 0x%x\n", (int)error_code);
        }
        usleep(100000);
      }
      for(int j = 0; j < 7; j++)
      {
        //Shift Output data
        do_value = do_value >> 1;
        cp1616_object->output_module_data_[0][0] = do_value;

        //Print OutData, InData
        cp1616_object->printOutputData(0);
        cp1616_object->printOutputData(1);
        cp1616_object->printOutputData(2);
        cp1616_object->printOutputData(3);
        cp1616_object->printInputData(0);
        cp1616_object->printInputData(1);
        cp1616_object->printInputData(2);
        cp1616_object->printInputData(3);

        error_code = cp1616_object->updateCyclicOutputData();
        if(error_code != PNIO_OK)
        {
          ROS_INFO("Not able to update input data: Error 0x%x\n", (int)error_code);
        }
        usleep(100000);
      }
    }
 }
  //Uninitialize CP
  cp1616_object->uinit();

#endif

#ifdef IO_DEVICE_MODE

  cp1616::Cp1616IODevice *cp1616_object;
  cp1616_object = cp1616::Cp1616IODevice::getDeviceInstance();

  int error_code;

  //Initialize CP
  cp1616_object->configureDeviceData();
  error_code = cp1616_object->init();
  error_code = cp1616_object->addApi();
  error_code = cp1616_object->addModSubMod();
  error_code = cp1616_object->startOperation();

  if(error_code == PNIO_OK) //if CP ready for communication
  {
    int slot = 2;
    int subslot = 1;
    int triangle = 0;
    bool raise_flag = true;

    for(int i = 0; i < 30; i++)
    {
      //Output from IO Controller (PLC) perspective - input CP data
      cp1616_object->updateCyclicOutputData();

      //increment/decrement triangle signal
      if((raise_flag == true) && (triangle < 100))
        triangle++;
      if((raise_flag == false) && (triangle > 0))
        triangle--;

      if(triangle == 100) raise_flag = false;
      if(triangle == 0)   raise_flag = true;

      //Write triangle value to in_data_
      cp1616_object->input_data_[slot][subslot][0] = triangle;

      //Input from IO Controller (PLC) perspective - output CP data
      cp1616_object->updateCyclicInputData();

      usleep(100000);
    }
  }
  else
      ROS_ERROR("CP not initialized properly: Error 0x%x", (int)error_code);

  //Uninitialize CP
  cp1616_object->stopOperation();
  cp1616_object->removeModSubMod();
  cp1616_object->removeApi();
  cp1616_object->uinit();

#endif

    return(EXIT_SUCCESS);
}
