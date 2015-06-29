#include <cp1616/cp1616_io_controller.h>
#include <cp1616/cp1616_io_controller_callbacks.h>
#include <cp1616/cp1616_io_device.h>
#include <cp1616/cp1616_io_device_callbacks.h>

//#define IO_CONTROLLER_MODE
#define IO_DEVICE_MODE

int main(int argc, char **argv)
{
  ros::init(argc,argv, "ros_profinet_experimental");
  ros::NodeHandle nh;

#ifdef IO_CONTROLLER_MODE

  Cp1616IOController *cp1616;
  cp1616 = Cp1616IOController::getControllerInstance();

  //Add IO modules
  cp1616->addOutputModule(4,4116);
  cp1616->addOutputModule(4,4120);
  cp1616->addOutputModule(4,4124);
  cp1616->addOutputModule(16,4128);

  cp1616->addInputModule(4,4132);
  cp1616->addInputModule(4,4136);
  cp1616->addInputModule(4,4140);
  cp1616->addInputModule(16,4144);

  //Initialize CP
  cp1616->init();

  if(cp1616->getCpReady() != 0)
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
        cp1616->setOutData(0,0,do_value);

        //Print OutData, InData
        cp1616->printOutputData(0);
        cp1616->printOutputData(1);
        cp1616->printOutputData(2);
        cp1616->printOutputData(3);
        cp1616->printInputData(0);
        cp1616->printInputData(1);
        cp1616->printInputData(2);
        cp1616->printInputData(3);

        error_code = cp1616->updateCyclicOutputData();
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
        cp1616->setOutData(0,0,do_value);

        //Print OutData, InData
        cp1616->printOutputData(0);
        cp1616->printOutputData(1);
        cp1616->printOutputData(2);
        cp1616->printOutputData(3);
        cp1616->printInputData(0);
        cp1616->printInputData(1);
        cp1616->printInputData(2);
        cp1616->printInputData(3);

        error_code = cp1616->updateCyclicOutputData();
        if(error_code != PNIO_OK)
        {
          ROS_INFO("Not able to update input data: Error 0x%x\n", (int)error_code);
        }P
        usleep(100000);
      }
    }
 }
  //Uninitialize CP
  cp1616->uinit();

#endif

#ifdef IO_DEVICE_MODE

  Cp1616IODevice *cp1616;
  cp1616 = Cp1616IODevice::getDeviceInstance();

  int error_code;

  //Initialize CP
  cp1616->configureDeviceData();
  error_code = cp1616->init();
  error_code = cp1616->addApi();
  error_code = cp1616->addModSubMod();
  error_code = cp1616->startOperation();

  if(error_code == PNIO_OK) //if CP ready for communication
  {
    int slot = 2;
    int subslot = 1;
    int triangle = 0;
    bool raise_flag = true;

    for(int i = 0; i < 30; i++)
    {
      //Output from IO Controller (PLC) perspective - input CP data
      cp1616->updateCyclicOutputData();

      //write data
      if((raise_flag == true) && (triangle < 100))
        triangle++;
      if((raise_flag == false) && (triangle > 0))
        triangle--;

      if(triangle == 100) raise_flag = false;
      if(triangle == 0)   raise_flag = true;

      cp1616->in_data_[slot][subslot][0] = triangle;

      //Input from IO Controller (PLC) perspective - output CP data
      cp1616->updateCyclicInputData();

      usleep(100000);
    }
  }
  else
      ROS_ERROR("CP not initialized properly: Error 0x%x", (int)error_code);

  //Uninitialize CP
  cp1616->stopOperation();
  cp1616->removeModSubMod();
  cp1616->removeApi();
  cp1616->uinit();

#endif

    return(EXIT_SUCCESS);
}
