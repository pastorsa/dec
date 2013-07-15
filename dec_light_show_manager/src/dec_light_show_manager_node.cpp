/*
 * dec_light_show_manager_node.cpp
 *
 *  Created on: Jun 16, 2013
 *      Author: pastor
 */

#include <dec_light_show_manager/dec_light_show_manager.h>
#include <dec_utilities/assert.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "");

  dec_light_show_manager::DecLightShowManager dec_light_show_manager;
  ROS_VERIFY(dec_light_show_manager.initialize());

  ros::AsyncSpinner spinner(0); // number of processor cores
  spinner.start();

  ros::Rate rate(100); // 100 Hz
  bool manager_is_running = true;
  while (ros::ok() && manager_is_running)
  {
    manager_is_running = dec_light_show_manager.update();
    rate.sleep();
  }

  spinner.stop();
}
