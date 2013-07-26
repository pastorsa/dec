/*
 * test_udp_dec.cpp
 *
 *  Created on: Jul 11, 2013
 *      Author: pastor
 */

// #include <ros/ros.h>

#include <dec_udp/dec_interface.h>

namespace dec_udp
{


}
int main(int argc, char** argv)
{
  // ros::init(argc, argv, "");
  // ros::NodeHandle node_handle("~");

  const int NUM_NODES = 1;
  int node_id = 1;

  dec_udp::DecInterface dec_interface(NUM_NODES);

  // if(!dec_interface.sendSetupData(node_id))
  {
    return -1;
  }

  // ros::MultiThreadedSpinner mts;
  // mts.spin();
  return 0;
}
