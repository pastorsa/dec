/*
 * dec_light_show_manager_node.cpp
 *
 *  Created on: Jun 16, 2013
 *      Author: pastor
 */

#include <ros/ros.h>
#include <dec_light_show_manager/dec_light_show_manager.h>
#include <dec_utilities/assert.h>
#include <dec_utilities/param_server.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "");
  dec_light_show_manager::DecLightShowManager dec_light_show_manager;
  ROS_VERIFY(dec_light_show_manager.initialize());
  ros::AsyncSpinner spinner(0); // number of processor cores
  spinner.start();
  dec_light_show_manager.run();
  spinner.stop();
}
