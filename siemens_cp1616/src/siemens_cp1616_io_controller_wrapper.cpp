/*********************************************************************************************//**
* @file siemens_cp1616_io_controller_wrapper.cpp
* 
* ROS wrapper for cp1616 IO Controller mode
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
#include <siemens_cp1616/siemens_cp1616_io_controller.h>
#include <siemens_cp1616/siemens_cp1616_io_controller_callbacks.h>

#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>

static const double TOPIC_UPDATE_PERIOD = 0.01;

void subCallback(const std_msgs::UInt8MultiArray::ConstPtr &msg);
void timerCallback(const ros::TimerEvent &event);

std::vector<ros::Publisher>  cp_publishers;
std::vector<ros::Subscriber> cp_subscribers;

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "cp1616_io_controller_wrapper");
  ros::NodeHandle nh;
  
  std::string filepath;
  nh.getParam("filepath", filepath);
  
  //Create cp object
  siemens_cp1616::Cp1616IOController *cp;
  cp = siemens_cp1616::Cp1616IOController::createControllerInstance(filepath);
  
  //Initialize cp1616
  int error_code = cp->init();
  
  //If initialized, create timer, publishers and subscribers
  if((error_code == PNIO_OK) && (cp->getCpReady() != 0))
  {
    //Create timer object to update input/output data in defined intervals
    ros::Timer timer = nh.createTimer(ros::Duration(TOPIC_UPDATE_PERIOD), &timerCallback);
    
    //Resize containers for publishers and subscribers
    cp_publishers.resize(cp->input_modules_.size());
    cp_subscribers.resize(cp->output_modules_.size());
    
    //Create publishers for all defined input modules 
    for(unsigned int i = 0; i < cp->input_modules_.size(); i++)
    {
      ros::Publisher temp_pub = nh.advertise<std_msgs::UInt8MultiArray>(cp->input_modules_[i].topic,1);
      cp_publishers[i] = temp_pub;
    }
    
    //Create subscribers for all defined output modules
    for(unsigned int i = 0; i < cp->output_modules_.size(); i++)
    {
      ros::Subscriber temp_sub = nh.subscribe(cp->output_modules_[i].topic, 1, &subCallback);
      cp_subscribers[i] = temp_sub;
    }
     
    ros::spin();
  }    
  return(EXIT_SUCCESS);
}

//Common callback function for all subscibers, messages are distinguished by labels 
void subCallback(const std_msgs::UInt8MultiArray::ConstPtr &msg)
{
  //Create callback_handler object to access cp1616_io_controller variables
  siemens_cp1616::Cp1616IOController *callback_handler = siemens_cp1616::Cp1616IOController::getControllerInstance();
  
  //Identify incoming data by MultiArray label
  unsigned int current_index;
  std::string msg_label = msg->layout.dim[0].label;
  for(unsigned int i = 0; i < callback_handler->output_modules_.size(); i++)
  {
    //Find corresponding msg_label in output_modules_ data (yaml config) and get current_index
    if(msg_label == callback_handler->output_modules_[i].label.c_str())
      current_index = i;
  }
  
  //Check if current_index is valid
  if(current_index < callback_handler->output_modules_.size())
  {
    //Copy topic data to output_data_ for cyclic update
    for(unsigned int j = 0; j < callback_handler->output_modules_[current_index].size; j++)
      callback_handler->output_data_[current_index][j] = msg->data.at(j);
  }
  else
    ROS_WARN_STREAM("CP1616 IO Controller Wrapper: Not able to identify topic data, check assigned message labels");
}

void timerCallback(const ros::TimerEvent &event)
{
  //Create callback_handler object to access cp1616_io_controller class methods
  siemens_cp1616::Cp1616IOController *callback_handler = siemens_cp1616::Cp1616IOController::getControllerInstance();
  
  //Update CP input/output buffers
  callback_handler->updateCyclicOutputData();
  callback_handler->updateCyclicInputData();
        
  //Publish actual incoming data to ROS topic
  for(unsigned int i = 0; i < callback_handler->input_modules_.size(); i++)
  {
    //Prepare msg data for i-th input module
    std_msgs::UInt8MultiArray msg;
    std_msgs::MultiArrayDimension msg_dim;
    msg.data.clear(); 
        
    msg_dim.label = callback_handler->input_modules_[i].label.c_str();
    msg_dim.size  = callback_handler->input_modules_[i].size;
    msg.layout.dim.push_back(msg_dim);

    //Copy input data to msd.data for publishing
    for(unsigned char j = 0; j < callback_handler->input_modules_[i].size; j++)
      msg.data.push_back(callback_handler->input_data_[i][j]);
    
    //Publish data to ROS topic
    cp_publishers[i].publish(msg);
  }
}
