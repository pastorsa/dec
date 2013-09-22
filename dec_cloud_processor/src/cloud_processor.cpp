/*
 * cloud_processor.cpp
 *
 *  Created on: Sep 15, 2013
 *      Author: pastor
 */

#include <string>

#include <dec_utilities/assert.h>
#include <dec_utilities/param_server.h>
#include <pcl/point_types.h>
#include <conversions/ros_to_tf.h>
#include <conversions/tf_to_ros.h>

#include <iostream>
#include <pcl/common/io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <tf/transform_datatypes.h>
#include <pcl/filters/extract_indices.h>
#include <boost/lexical_cast.hpp>

#include <dec_cloud_processor/cloud_processor.h>

using namespace dec_utilities;

namespace dec_cloud_processor
{

CloudProcessor::CloudProcessor(ros::NodeHandle node_handle)
	: node_handle_(node_handle)
{
  std::string point_cloud_topic;
  ROS_VERIFY(read(node_handle_, "point_cloud_topic", point_cloud_topic));



  // do this last
  point_cloud_sub_ = node_handle_.subscribe(point_cloud_topic, 10, &CloudProcessor::pointCloudCallback, this);
  ros::Duration(1.0).sleep();
}

void CloudProcessor::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr point_cloud)
{

}

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "DecCloudProcessor");
  ros::NodeHandle node_handle("~");
  dec_cloud_processor::CloudProcessor cloud_processor(node_handle);
  ros::MultiThreadedSpinner mts;
  mts.spin();
  return 0;
}
