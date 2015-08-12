/*********************************************************************************************//**
* @file siemens_cp1616_io_device_wrapper.cpp
* 
* ROS wrapper for cp1616 IO device mode
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
#include <siemens_cp1616/siemens_cp1616_io_device.h>
#include <siemens_cp1616/siemens_cp1616_io_device_callbacks.h>

#include <siemens_cp1616/set_alarm.h>
#include <siemens_cp1616/reset_alarm.h>

#include <std_srvs/Empty.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>

static const double TOPIC_UPDATE_PERIOD = 0.01;

void subCallback(const std_msgs::UInt8MultiArray::ConstPtr &msg);
void timerCallback(const ros::TimerEvent &event);

bool setAlarmCallback(siemens_cp1616::set_alarm::Request &request,
                      siemens_cp1616::set_alarm::Response &response);

bool resetAlarmCallback(siemens_cp1616::reset_alarm::Request &request,
                        siemens_cp1616::reset_alarm::Response &response);

std::vector<ros::Publisher>  cp_publishers;
std::vector<ros::Subscriber> cp_subscribers;

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "cp1616_io_device_wrapper");
  ros::NodeHandle nh;
  
  std::string filepath;
  nh.getParam("filepath", filepath);
  
  //Create CP IO Device object
  siemens_cp1616::Cp1616IODevice *cp;
  cp = siemens_cp1616::Cp1616IODevice::createDeviceInstance(filepath);
  
  //Initialize cp1616
  int error_code = cp->init();
  
  //If initialized, create timer, publishers, and subscribers
  if(error_code == PNIO_OK)
  {
    //Create timer object to update input/output data in defined intervals
    ros::Timer time = nh.createTimer(ros::Duration(TOPIC_UPDATE_PERIOD), &timerCallback);
    
    //Create service servers for setAlarm and resetAlarm
    ros::ServiceServer set_alarm_service   = nh.advertiseService("set_alarm", &setAlarmCallback);
    ros::ServiceServer reset_alarm_service = nh.advertiseService("reset_alarm", &resetAlarmCallback);
   
    //Resize containers for publishers and subscribers
    cp_publishers.resize(cp->modules_.size());
    cp_subscribers.resize(cp->modules_.size());
    
    //modules slots might be configured as input, output or bidirectional
    for(unsigned int i = 0; i < cp->modules_.size(); i++)
    {
      //Create subscribers for all input modules/slots
      if((cp->modules_[i].type == "input") || (cp->modules_[i].type == "Input"))
      {
        ros::Subscriber temp_sub = nh.subscribe(cp->modules_[i].topic,1, &subCallback);
        cp_subscribers[i] = temp_sub;
      }     
      
      //Create publishers for all output modules/slots
      if((cp->modules_[i].type == "output") || (cp->modules_[i].type == "Output"))
      {
        ros::Publisher temp_pub = nh.advertise<std_msgs::UInt8MultiArray>(cp->modules_[i].topic,1);
        cp_publishers[i] = temp_pub;
      } 
      
      //Create publishers and subscribers for bidirectional modules/slots
      if((cp->modules_[i].type == "bidirect") || (cp->modules_[i].type == "Bidirect"))
      {
        std::string sub_topic_str = cp->modules_[i].topic + "_input_topic";
        ros::Subscriber temp_sub = nh.subscribe(sub_topic_str,1, &subCallback);
        cp_subscribers[i] = temp_sub;

        std::string pub_topic_str = cp->modules_[i].topic + "_output_topic";
        ros::Publisher temp_pub = nh.advertise<std_msgs::UInt8MultiArray>(pub_topic_str,1);
        cp_publishers[i] = temp_pub;
      }      
    }
    
    ros::spin();
  }   
  return(EXIT_SUCCESS);
}

//Common callback function for all subscibers, messages are distinguished by labels 
void subCallback(const std_msgs::UInt8MultiArray::ConstPtr &msg)
{
  //Create callback_handler object to access cp1616_io_device class methods
  siemens_cp1616::Cp1616IODevice *callback_handler = siemens_cp1616::Cp1616IODevice::getDeviceInstance();
      
  //Identify incoming data by MultiArray label
  unsigned int current_index;
  std::string msg_label = msg->layout.dim[0].label;
  for(unsigned int i = 0; i < callback_handler->modules_.size(); i++)
  {
    //Find corresponding msg_label in modules_ data (yaml config) and get current_index
    if(msg_label == callback_handler->modules_[i].label.c_str())
      current_index = i;
  }
  
  //Check if current_index is valid
  if(current_index < callback_handler->modules_.size())
  {
    //in case of input or bidirect current module outgoing data 
    if((callback_handler->modules_[current_index].type == "input") 
      || (callback_handler->modules_[current_index].type == "Input")
      || (callback_handler->modules_[current_index].type == "bidirect") 
      || (callback_handler->modules_[current_index].type == "Bidirect"))
    {
      //Copy topic data to input_data_ for cyclic update (current_index + 1 because of DAP module)
      for(unsigned int j = 0; j < callback_handler->modules_[current_index].size; j++)
        callback_handler->input_data_[current_index + 1][j] = msg->data.at(j);
    }
  }  
  else
    ROS_WARN_STREAM("CP1616 IO Device Wrapper: Not able to identify topic data, check assigned message labels");
}

void timerCallback(const ros::TimerEvent &event)
{
  //Create callback_handler object to access cp1616_io_device class methods
  siemens_cp1616::Cp1616IODevice *callback_handler = siemens_cp1616::Cp1616IODevice::getDeviceInstance();
  
  //Update CP input/output buffers
  callback_handler->updateCyclicInputData();
  callback_handler->updateCyclicOutputData();
  
  //Publish actual incoming data to ROS topic
  for(unsigned int i = 0; i < callback_handler->modules_.size(); i++)
  {
    //in case of input or bidirect current module send incoming to topics 
    if((callback_handler->modules_[i].type == "output") 
      || (callback_handler->modules_[i].type == "Output")
      || (callback_handler->modules_[i].type == "bidirect")
      || (callback_handler->modules_[i].type == "Bidirect"))
    {
      //Prepare msg data for i-th input module
      std_msgs::UInt8MultiArray msg;
      std_msgs::MultiArrayDimension msg_dim;
      msg.data.clear(); 
        
      msg_dim.label = callback_handler->modules_[i].label.c_str();
      msg_dim.size  = callback_handler->modules_[i].size;
      msg.layout.dim.push_back(msg_dim);

      //Copy input data to msd.data for publishing
      for(unsigned char j = 0; j < callback_handler->modules_[i].size; j++)
         msg.data.push_back(callback_handler->output_data_[i+1][j]);
    
      //Publish data to ROS topic
      cp_publishers[i].publish(msg);
    }
  } 
}

bool setAlarmCallback(siemens_cp1616::set_alarm::Request &request,
                      siemens_cp1616::set_alarm::Response &response)
{
  //Create callback_handler object to access cp1616_io_device class methods
  siemens_cp1616::Cp1616IODevice *callback_handler = siemens_cp1616::Cp1616IODevice::getDeviceInstance();
    
  PNIO_UINT32 error_code = PNIO_OK;
  error_code = callback_handler->sendDiagnosticAlarm(request.slot_num);
  
  if(error_code != PNIO_OK)
  {
    response.feedback = false;
    return false;
  }
  else 
  {
    response.feedback = true;
    return true; 
  }
}

bool resetAlarmCallback(siemens_cp1616::reset_alarm::Request &request,
                        siemens_cp1616::reset_alarm::Response &response)
{
  //Create callback_handler object to access cp1616_io_device class methods
  siemens_cp1616::Cp1616IODevice *callback_handler = siemens_cp1616::Cp1616IODevice::getDeviceInstance();
  
  PNIO_UINT32 error_code = PNIO_OK;
  error_code = callback_handler->resetDiagnosticAlarm(request.slot_num);
  
  if(error_code != PNIO_OK)
  {
    response.feedback = false;
    return false;
  }
  else 
    response.feedback = true;
    return true;  
}