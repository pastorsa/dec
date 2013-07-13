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
  return true;
}

bool DECCommunication::process()
{
  return DECProcessor::update();
}

}
