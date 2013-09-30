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

#include <dec_msgs/LightFrame.h>

namespace dec_cloud_processor
{

static const std::string BASE_FRAME_ID = "/BASE";

class CloudProcessor
{

public:
  CloudProcessor(ros::NodeHandle node_handle);
  virtual ~CloudProcessor() {};

  bool initialize();

  void run();

private:

  ros::NodeHandle node_handle_;

  ros::Publisher rviz_pub_;
  ros::Publisher cloud_all_pub_;
  ros::Publisher cloud_cropped_pub_;

  ros::Publisher light_frame_pub_;

  ros::Subscriber point_cloud_sub_;
  ros::Publisher processed_frame_pub_;

  visualization_msgs::Marker square_markers_;

  void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr point_cloud);

  void readParams();

  tf::Transform point_cloud_to_base_transform_;

  pcl::CropBox<pcl::PointXYZ> crop_box_;

  dec_msgs::LightFrame light_frame_;

  // std::vector<std::pair<double, double> > x_range_;
  // std::vector<std::pair<double, double> > y_range_;

  tf::TransformListener tf_listener_;

  std::vector<tf::Vector3> node_positions_;

  double detection_radius_;
  double downsampling_leaf_size_;

};

}

#endif /* CLOUD_PROCESSOR_H_ */
