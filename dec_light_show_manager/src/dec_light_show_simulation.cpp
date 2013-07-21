/*
 * dec_light_show_simulation.cpp
 *
 *  Created on: Jun 17, 2013
 *      Author: pastor
 */

#include <dec_utilities/assert.h>
#include <dec_utilities/param_server.h>
#include <tf/transform_datatypes.h>
#include <conversions/ros_to_tf.h>
#include <conversions/tf_to_ros.h>

#include <dec_light_show_manager/dec_light_show_simulation.h>

using namespace std;
using namespace conversions;

namespace dec_light_show_manager
{

DecLightShowSimulation::DecLightShowSimulation(boost::shared_ptr<DecLightShowData> light_show_data)
  : number_of_simulated_objects_(0),
    simulated_object_name_(""),
    simulation_sensor_object_threshold_(0.0),
    light_show_data_(light_show_data)
{
}

bool DecLightShowSimulation::initialize(ros::NodeHandle node_handle)
{
  ROS_VERIFY(dec_utilities::read(node_handle, "number_of_simulated_objects", number_of_simulated_objects_));
  ROS_VERIFY(dec_utilities::read(node_handle, "simulated_object_name", simulated_object_name_));
  ROS_VERIFY(dec_utilities::read(node_handle, "simulation_sensor_object_threshold", simulation_sensor_object_threshold_));

  ROS_DEBUG("Simulation up to >%i< objects.", number_of_simulated_objects_);
  return true;
}

double DecLightShowSimulation::getMinimumDistance(tf::Point point, std::vector<std::pair<unsigned int, unsigned int> > beams)
{
  double min_distance = std::numeric_limits<double>::max();
  for (unsigned int i = 0; i < beams.size(); ++i)
  {
    double distance = getDistance(point, beams[i]);
    if (distance < min_distance)
      min_distance = distance;
  }
  return min_distance;
}

double DecLightShowSimulation::getDistance(tf::Point point, std::pair<unsigned int, unsigned int> beam)
{
  tf::Vector3 p0;
  convert(light_show_data_->getNodePosition(beam.first), p0);
  tf::Vector3 p1;
  convert(light_show_data_->getNodePosition(beam.second), p1);
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

bool DecLightShowSimulation::copySimulatedSensorInformation()
{
  light_show_data_->sensor_values_.setZero();
  for (int object_index = 0; object_index < number_of_simulated_objects_; ++object_index)
  {
    tf::Pose object_pose;
    std::string object_name = simulated_object_name_ + "_" + boost::lexical_cast<std::string>(object_index);
    if (world_state_.getObjectPose(object_name, object_pose, false))
    {
      for (unsigned int i = 0; i < light_show_data_->capactive_sensors_.size(); ++i)
      {
        double distance = getMinimumDistance(object_pose.getOrigin(), light_show_data_->capactive_sensors_[i].nodes_);
        if (distance < simulation_sensor_object_threshold_)
        {
          // ROS_DEBUG("Distance between object >%s< and sensor beam >%i< between node >%i< and >%i< is >%.2f< m.",
          //           object_name.c_str(), i, light_show_data_->sensors_[i].first, light_show_data_->sensors_[i].second, distance);
          // scale distance to interval [0..1]
          // double sensor_value = 1.0 - (distance / simulation_sensor_object_threshold_);
          // sensor_value *= 255;
          // set sensor value
          light_show_data_->sensor_values_(i) = (sensor_channel_t)1.0;
        }
      }
    }
  }

  return true;
}

}


