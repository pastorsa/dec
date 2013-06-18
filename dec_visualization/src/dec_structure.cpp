/*
 * dec_structure.cpp
 *
 *  Created on: Jun 17, 2013
 *      Author: pastor
 */

#include <tf/transform_datatypes.h>
#include <conversions/ros_to_tf.h>
#include <conversions/tf_to_ros.h>
#include <dec_utilities/assert.h>
#include <dec_utilities/param_server.h>

#include <dec_visualization/dec_structure.h>

namespace dec_visualization
{

bool DECStructure::initialize(ros::NodeHandle node_handle)
{
  ROS_VERIFY(dec_utilities::read(node_handle, "number_of_arduinos", number_of_arduinos_));
  ROS_VERIFY(dec_utilities::read(node_handle, "max_number_of_sensors_per_arduino", max_number_of_sensors_per_arduino_));
  ROS_VERIFY(dec_utilities::read(node_handle, "max_number_of_lights_per_arduino", max_number_of_lights_per_arduino_));

  ROS_VERIFY(read(node_handle, "node_positions", node_positions_));
  ROS_VERIFY(read(node_handle, "light_node_positions", light_node_positions_));
  ROS_INFO("Read >%i< nodes and >%i< light node positions.",
           (int)node_positions_.size(), (int)light_node_positions_.size());
  ROS_VERIFY(read(node_handle, "beams", beams_));
  ROS_VERIFY(read(node_handle, "light_beams", light_beams_));
  ROS_VERIFY(read(node_handle, "sensors", sensors_));

  ROS_VERIFY(read(node_handle, "light_beam_connections", light_beam_connections_, light_beams_));
  ROS_VERIFY(read(node_handle, "sensor_connections", sensor_connections_, sensors_));

  return true;
}



}
