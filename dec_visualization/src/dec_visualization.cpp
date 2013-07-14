/*
 * dec_visualization.cpp
 *
 *  Created on: Jun 16, 2013
 *      Author: pastor
 */

#include <vector>
#include <boost/thread/mutex.hpp>

#include <tf/transform_datatypes.h>
#include <conversions/ros_to_tf.h>
#include <conversions/tf_to_ros.h>

#include <dec_utilities/assert.h>
#include <dec_utilities/param_server.h>

#include <dec_visualization/dec_simulation.h>
// #include <dec_visualization/dec_communication.h>

#include <dec_visualization/dec_visualization.h>

using namespace std;
using namespace conversions;

namespace dec_visualization
{

DECVisualization::DECVisualization(ros::NodeHandle node_handle)
  : simulation_mode_(false)
{
  ROS_VERIFY(initialize(node_handle));
}

bool DECVisualization::initialize(ros::NodeHandle node_handle)
{
  ROS_INFO("Initializing DEC visualization.");

  ROS_VERIFY(dec_utilities::read(node_handle, "simulation_mode", simulation_mode_));
//  if(simulation_mode_)
//  {
    dec_processor_.reset(new DECSimulation());
//  }
//  else
//  {
//    dec_processor_.reset(new DECCommunication());
//  }
  ROS_ASSERT(dec_processor_->initialize(node_handle));

  // Eigen::DenseIndex size = (Eigen::DenseIndex)dec_processor_->light_nodes_.size();
  // graph_ = Eigen::MatrixXf::Zero(size, size);

  return true;
}

void DECVisualization::run()
{
  // setGraphDistances();

  ROS_INFO_COND(simulation_mode_, "Starting simulation mode.");
  ROS_INFO_COND(!simulation_mode_, "Starting real mode.");

  while (ros::ok())
  {
    ROS_VERIFY(dec_processor_->process());
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }
}

// bool DECVisualization::setGraphDistances()
// {
//  Eigen::DenseIndex size = (Eigen::DenseIndex)dec_processor_->light_nodes_.size();
//  graph_.setZero(size,size);
//
//  for (unsigned int i = 0; i < dec_processor_->light_nodes_.size(); ++i)
//  {
//    for (unsigned int j = 0; j < i; ++j)
//    {
//      tf::Vector3 point_i, point_j;
//      convert(dec_processor_->light_node_positions_[i], point_i);
//      convert(dec_processor_->light_node_positions_[j], point_j);
//
//      double distance = fabs(tf::Vector3(point_i - point_j).length());
//      graph_(i,j) = distance;
//      graph_(j,i) = distance;
//    }
//  }
//  // ROS_INFO_STREAM(graph_);
//
//  return true;
// }

} // namespace

