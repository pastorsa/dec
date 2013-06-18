/*
 * dec_visualization_node.cpp
 *
 *  Created on: Jun 16, 2013
 *      Author: pastor
 */


#include <dec_visualization/dec_visualization.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "");
  ros::NodeHandle node_handle("~");
  dec_visualization::DECVisualization dec_visualization(node_handle);
  dec_visualization.run();

  ros::MultiThreadedSpinner mts;
  mts.spin();

  // ros::spin();
}
