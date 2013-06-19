/*
 * dec_communication.cpp
 *
 *  Created on: Jun 16, 2013
 *      Author: pastor
 */

#include <string>

#include <dec_utilities/assert.h>
#include <dec_utilities/param_server.h>

#include <dec_visualization/dec_communication.h>
#include <dec_visualization/dec_simulation.h>

namespace dec_visualization
{

DECCommunication::DECCommunication()
{
  ROS_INFO("Creating DEC communication.");
}

bool DECCommunication::initialize(ros::NodeHandle node_handle)
{
  ROS_VERIFY(DECProcessor::init(node_handle));

  std::string rs485_device_name;
  ROS_VERIFY(dec_utilities::read(node_handle, "rs485_device_name", rs485_device_name));

  ROS_INFO("Reading data from >%s< of >%i< arduinos with max >%i< lights and max >%i< sensors.", rs485_device_name.c_str(),
           number_of_arduinos_, max_number_of_lights_per_arduino_, max_number_of_sensors_per_arduino_);

  return true;
}

bool DECCommunication::process()
{
  return DECProcessor::update();
}

}
