/*********************************************************************************************//**
* @file siemens_cp1616_io_controller_callbacks.cpp
*
* Callbacks required by IO Base library
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
#ifndef SIEMENS_CP1616_IO_CONTROLLER_CALLBACKS_CPP
#define SIEMENS_CP1616_IO_CONTROLLER_CALLBACKS_CPP

#include <siemens_cp1616/siemens_cp1616_io_controller.h>

namespace siemens_cp1616
{
namespace pnio_controller_callbacks
{
  void modeChangeIndication(PNIO_CBE_PRM *p_cbf_prm)
  {
    //Create callback_handler object to access cp1616_io_controller variables
    Cp1616IOController *callback_handler = Cp1616IOController::getControllerInstance();

    if(p_cbf_prm->CbeType == PNIO_CBE_MODE_IND) /* Check callback type */
    {
      switch (p_cbf_prm->ModeInd.Mode)
      {
	case PNIO_MODE_OFFLINE:
	  ROS_INFO("IO_controller mode change request-> OFFLINE: ");
	  callback_handler->setCpCurrentModeFlag(PNIO_MODE_OFFLINE);
	  break;
	case PNIO_MODE_CLEAR:
	  ROS_INFO("IO_controller mode change request-> CLEAR: ");
	  callback_handler->setCpCurrentModeFlag(PNIO_MODE_CLEAR);
	  break;
	case PNIO_MODE_OPERATE:
	  ROS_INFO("IO_controller mode change request-> OPERATE: ");
	  callback_handler->setCpCurrentModeFlag(PNIO_MODE_OPERATE);
	  break;
	default:
	  ROS_INFO("Not able to change IO_controller mode: Wrong mode selected!");
	  break;
      };

      //send notification
      callback_handler->setSemModChange(1);
    }
  }

  void alarmIndication(PNIO_CBE_PRM *p_cbf_prm)
  {
    //Create callback_handler object to access cp1616_io_controller variables
    Cp1616IOController *callback_handler = Cp1616IOController::getControllerInstance();

    if(p_cbf_prm->CbeType==PNIO_CBE_ALARM_IND) /* Check callback type */
    {
      switch (p_cbf_prm->AlarmInd.pAlarmData->AlarmType)
      {
	case PNIO_ALARM_DIAGNOSTIC:
	  ROS_WARN("PNIO_ALARM_DIAGNOSTIC");
	  break;
	case PNIO_ALARM_PROCESS:
	  ROS_WARN("PNIO_ALARM_DIAGNOSTIC");
	  break;
	case PNIO_ALARM_PULL:
	  ROS_WARN("PNIO_ALARM_PULL");
	  break;
	case PNIO_ALARM_PLUG:
	  ROS_WARN("PNIO_ALARM_PLUG");
	  break;
	case PNIO_ALARM_STATUS:
	  ROS_WARN("PNIO_ALARM_STATUS");
	  break;
	case PNIO_ALARM_UPDATE:
	  ROS_WARN("PNIO_ALARM_UPDATE");
	  break;
	case PNIO_ALARM_REDUNDANCY:
	  ROS_WARN("PNIO_ALARM_REDUNDANCY");
	  break;
	case PNIO_ALARM_CONTROLLED_BY_SUPERVISOR:
	  ROS_WARN("PNIO_ALARM_CONTROLLED_BY_SUPERVISOR");
	  break;
	case PNIO_ALARM_RELEASED_BY_SUPERVISOR:
	  ROS_WARN("PNIO_ALARM_RELEASED_BY_SUPERVISOR");
	  break;
	case PNIO_ALARM_PLUG_WRONG:
	  ROS_WARN("PNIO_ALARM_PLUG_WRONG");
	  break;
	case PNIO_ALARM_RETURN_OF_SUBMODULE:
	  ROS_WARN("PNIO_ALARM_RETURN_OF_SUBMODULE");
	  break;
	case PNIO_ALARM_DEV_FAILURE:
	  #if DEBUG
	    ROS_WARN("PNIO_ALARM_DEV_FAILURE");
	  #endif
	  break;
	case PNIO_ALARM_DEV_RETURN:
	  #ifdef DEBUG
	    ROS_WARN("PNIO_ALARM_DEV_RETURN");
	  #endif
	  callback_handler->setCpReady(1);	//Set cp_ready_ flag to notify application that CP is ready for communication
	  break;
	default:
	  ROS_WARN("callback_for_alarm_indication called with wrong type");
	  break;
      };
    }
  }

  void deviceActivation(PNIO_CBE_PRM *p_cbf_prm)
  {
    switch( p_cbf_prm->DevActConf.Mode)
    {
      case PNIO_DA_TRUE:
	ROS_INFO("IO_Controller - address %x was activated with result %x",
	  (int)p_cbf_prm->DevActConf.pAddr->u.Addr,
	  (int)p_cbf_prm->DevActConf.Result);
	  break;
	case PNIO_DA_FALSE:
	  ROS_INFO("IO_Controller - address %x was deactivated with result %x",
	  (int)p_cbf_prm->DevActConf.pAddr->u.Addr,
	  (int)p_cbf_prm->DevActConf.Result);
	break;
      };
  }

  void dsReadConf(PNIO_CBE_PRM *p_cbf_prm)
  {
    ROS_WARN("CallbackForDsReadConf should not occur within this implementation");
  }

  void dsWriteConf(PNIO_CBE_PRM *p_cbf_prm)
  {
    ROS_WARN("CallbackForDsWriteConf should not occur within this implementation");
  }
} //siemens_cp1616_io_controller_callbacks
} //siemens_cp1616

#endif //SIEMENS_CP1616_IO_CONTROLLER_CALLBACKS_CPP
