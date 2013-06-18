/*
 * dec_simulation.cpp
 *
 *  Created on: Jun 17, 2013
 *      Author: pastor
 */

#include <dec_utilities/assert.h>
#include <dec_utilities/param_server.h>
#include <tf/transform_datatypes.h>
#include <conversions/ros_to_tf.h>
#include <conversions/tf_to_ros.h>

#include <dec_visualization/dec_simulation.h>

using namespace std;
using namespace conversions;

namespace dec_visualization
{

DECSimulation::DECSimulation()
{
  ROS_INFO("Creating DEC simulation.");
}

bool DECSimulation::initialize(ros::NodeHandle node_handle)
{
  ROS_VERIFY(DECProcessor::init(node_handle));

  ROS_VERIFY(dec_utilities::read(node_handle, "number_of_simulated_objects", number_of_simulated_objects_));
  ROS_VERIFY(dec_utilities::read(node_handle, "simulated_object_name", simulated_object_name_));
  ROS_VERIFY(dec_utilities::read(node_handle, "simualtion_sensor_object_threshold", simualtion_sensor_object_threshold_));

  ROS_DEBUG("Simulation up to >%i< objects.", number_of_simulated_objects_);
  return true;
}


double DECSimulation::getDistance(tf::Point point, std::pair<int, int> beam)
{
  tf::Vector3 p0;
  convert(node_positions_[beam.first], p0);
  tf::Vector3 p1;
  convert(node_positions_[beam.second], p1);
  tf::Vector3 v = p1 - p0;
  tf::Vector3 w = point - p0;

  if (fabs(v.length()) < 0.001)
  {
    ROS_WARN("Invalid distance computation of beam between node >%i< and >%i<.", beam.first, beam.second);
    return fabs((point - p0).length());
  }

  double c1 = w.dot(v);
  if (c1 <= 0)
    return fabs((point - p0).length());

  double c2 = v.dot(v);
  if (c2 <= c1)
    return fabs((point - p1).length());

  double b = c1 / c2;
  tf::Vector3 pb = p0 + b * v;

  return fabs((point - pb).length());
}

bool DECSimulation::process()
{
  ROS_DEBUG_STREAM(std::endl << std::endl);

  for (unsigned int i = 0; i < sensors_.size(); ++i)
  {
    setSensorValue(sensor_connections_[i], sensor_index_counter_[i], 0);
  }

  for (int object_index = 0; object_index < number_of_simulated_objects_; ++object_index)
  {
    tf::Pose object_pose;
    std::string object_name = simulated_object_name_ + "_" + boost::lexical_cast<std::string>(object_index);
    if (world_state_.getObjectPose(object_name, object_pose, false))
    {
      for (unsigned int i = 0; i < sensors_.size(); ++i)
      {
        double distance = getDistance(object_pose.getOrigin(), sensors_[i]);
        if(distance < simualtion_sensor_object_threshold_)
        {
          ROS_DEBUG("Distance between object >%s< and sensor beam >%i< between node >%i< and >%i< is >%.2f< m.",
                    object_name.c_str(), i, sensors_[i].first, sensors_[i].second, distance);
          // scale distance to interval [0..1]
          double sensor_value = 1.0 - (distance / simualtion_sensor_object_threshold_);
          sensor_value *= SENSOR_RESOLUTION;
          addSensorValue(sensor_connections_[i], sensor_index_counter_[i], (int)sensor_value);
        }
      }
    }
  }

  ROS_DEBUG_STREAM("data: " << std::endl << data_);

  return DECProcessor::update();
}



}


