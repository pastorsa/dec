/*
 * cloud_processor.h
 *
 *  Created on: Sep 15, 2013
 *      Author: pastor
 */

#ifndef CLOUD_PROCESSOR_H_
#define CLOUD_PROCESSOR_H_

#include <ros/ros.h>

#include <visualization_msgs/Marker.h>
#include <boost/shared_ptr.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/crop_box.h>

namespace dec_cloud_processor
{

class CloudProcessor
{

public:
  CloudProcessor(ros::NodeHandle node_handle);
  virtual ~CloudProcessor() {};

private:

  ros::Publisher rviz_pub_;
  ros::NodeHandle node_handle_;

  ros::Subscriber point_cloud_sub_;
  ros::Publisher processed_frame_pub_;

  visualization_msgs::Marker square_markers_;

  void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr point_cloud);

};

}

#endif /* CLOUD_PROCESSOR_H_ */
