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

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <dec_cloud_processor/cloud_processor.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

using namespace dec_utilities;

namespace dec_cloud_processor
{

CloudProcessor::CloudProcessor(ros::NodeHandle node_handle)
: node_handle_(node_handle), detection_radius_(-1.0), downsampling_leaf_size_(0.01), transform_using_tf_(false), id_counter_(0)
{
  const unsigned int BUFFER_SIZE = 1;
  rviz_pub_ = node_handle_.advertise<visualization_msgs::Marker>("visualization_marker", BUFFER_SIZE);

  // cloud_all_pub_ = node_handle_.advertise<pcl::PointCloud<pcl::PointXYZ> >("cloud_all", 100);
  cloud_cropped_pub_ = node_handle_.advertise<pcl::PointCloud<pcl::PointXYZ> >("cloud_cropped", BUFFER_SIZE);

  std::string light_frame_topic;
  ROS_VERIFY(read(node_handle_, "light_frame_topic", light_frame_topic));
  light_frame_pub_ = node_handle_.advertise<dec_msgs::LightFrame>(light_frame_topic, BUFFER_SIZE);

  ros::Duration(1.0).sleep();
}

bool CloudProcessor::initialize()
{
  readParams();

  // do this last
  std::string point_cloud_topic;
  ROS_VERIFY(read(node_handle_, "point_cloud_topic", point_cloud_topic));
  const unsigned int BUFFER_SIZE = 1;
  point_cloud_sub_ = node_handle_.subscribe(point_cloud_topic, BUFFER_SIZE, &CloudProcessor::pointCloudCallback, this);
  // ros::Duration(1.0).sleep();

  return true;
}

void CloudProcessor::run()
{
  ROS_INFO("%s is up and running.", node_handle_.getNamespace().c_str());
  // ros::MultiThreadedSpinner mts;
  ros::spin();
}

void CloudProcessor::readParams()
{
  node_positions_.clear();

  ros::NodeHandle node_handle("/DecLightShowManager");
  XmlRpc::XmlRpcValue rpc_values;
  const std::string key = "node_positions";
  ROS_VERIFY(node_handle.hasParam(key));
  ROS_VERIFY(node_handle.getParam(key, rpc_values));
  ROS_ASSERT(rpc_values.getType() == XmlRpc::XmlRpcValue::TypeArray);

  for (int i = 0; i < rpc_values.size(); ++i)
  {
    ROS_ASSERT(rpc_values[i].hasMember("position"));
    XmlRpc::XmlRpcValue rpc_value = rpc_values[i]["position"];
    ROS_ASSERT(rpc_value.size() == 3);
    std::vector<double> values;
    for (int j = 0; j < 3; ++j)
    {
      ROS_ASSERT(rpc_value[j].getType() == XmlRpc::XmlRpcValue::TypeDouble
                 || rpc_value[j].getType() == XmlRpc::XmlRpcValue::TypeInt);
      double value = rpc_value[j];
      const double NODE_UNIT_SCALE = 1000.0;
      value /= NODE_UNIT_SCALE;
      values.push_back(static_cast<double> (value));
    }
    node_positions_.push_back(tf::Vector3(values[0], values[1], 0.0));
  }

  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double max_x = -std::numeric_limits<double>::max();
  double max_y = -std::numeric_limits<double>::max();
  for (unsigned int i = 0; i < node_positions_.size(); ++i)
  {
    if (node_positions_[i].getX() > max_x)
      max_x = node_positions_[i].getX();
    if (node_positions_[i].getY() > max_y)
      max_y = node_positions_[i].getY();
    if (node_positions_[i].getX() < min_x)
      min_x = node_positions_[i].getX();
    if (node_positions_[i].getY() < min_y)
      min_y = node_positions_[i].getY();
  }
  double avg_x = (max_x - min_x) / 2.0;
  ROS_ASSERT(avg_x > 0.0);
  double avg_y = (max_y - min_y) / 2.0;
  ROS_ASSERT(avg_y > 0.0);
  // ROS_INFO("Min X: %.2f", min_x);
  // ROS_INFO("Min Y: %.2f", min_y);
  // ROS_INFO("Max X: %.2f", max_x);
  // ROS_INFO("Max Y: %.2f", max_y);
  geometry_msgs::Point offset;
  offset.x = avg_x;
  offset.y = avg_y;
  offset.z = 0.0;
  offsetNodePositions(node_positions_, offset);

  ROS_VERIFY(read(node_handle_, "detection_radius", detection_radius_));
  detection_radius_ = detection_radius_ * detection_radius_;
  ROS_ASSERT(detection_radius_ > 0.0);

  ROS_VERIFY(read(node_handle_, "downsampling_leaf_size", downsampling_leaf_size_));
  ROS_ASSERT(downsampling_leaf_size_ > 0.0);

  double sensor_area_width = 0.0;
  double sensor_area_length = 0.0;
  double sensor_area_min_height = 0.0;
  double sensor_area_max_height = 0.0;
  ROS_VERIFY(read(node_handle_, "sensor_area_width", sensor_area_width));
  ROS_VERIFY(read(node_handle_, "sensor_area_length", sensor_area_length));
  ROS_VERIFY(read(node_handle_, "sensor_area_min_height", sensor_area_min_height));
  ROS_VERIFY(read(node_handle_, "sensor_area_max_height", sensor_area_max_height));
  ROS_ASSERT(sensor_area_max_height - sensor_area_min_height > 0.0);

  std::vector<double> kinect_to_base_transform;
  ROS_VERIFY(read(node_handle_, "kinect_to_base_transform", kinect_to_base_transform));
  ROS_ASSERT(kinect_to_base_transform.size() == 7);
  point_cloud_to_base_transform_.setOrigin(tf::Vector3(kinect_to_base_transform[0],
                                                       kinect_to_base_transform[1],
                                                       kinect_to_base_transform[2]));
  point_cloud_to_base_transform_.setRotation(tf::Quaternion(kinect_to_base_transform[3],
                                                            kinect_to_base_transform[4],
                                                            kinect_to_base_transform[5],
                                                            kinect_to_base_transform[6]));

  ROS_VERIFY(read(node_handle_, "transform_using_tf", transform_using_tf_));

  Eigen::Vector4f sensor_area_min;
  Eigen::Vector4f sensor_area_max;

  std::vector<double> base_to_sensor_area_transform;
  ROS_VERIFY(read(node_handle_, "base_to_sensor_area_transform", base_to_sensor_area_transform));
  ROS_ASSERT(base_to_sensor_area_transform.size() == 6);

  sensor_area_min(0) = base_to_sensor_area_transform[0] - (sensor_area_width / 2.0);
  sensor_area_min(1) = base_to_sensor_area_transform[1] - (sensor_area_length / 2.0);
  sensor_area_min(2) = sensor_area_min_height;

  sensor_area_max(0) = base_to_sensor_area_transform[0] + (sensor_area_width / 2.0);
  sensor_area_max(1) = base_to_sensor_area_transform[1] + (sensor_area_length / 2.0);
  sensor_area_max(2) = sensor_area_max_height;

  crop_box_.setMin(sensor_area_min);
  crop_box_.setMax(sensor_area_max);

  double sensor_area_height = (sensor_area_max_height - sensor_area_min_height);
  double crop_box_height = sensor_area_min_height + (sensor_area_height / 2.0);

  Eigen::Vector3f eigen_translation(base_to_sensor_area_transform[0],
                                    base_to_sensor_area_transform[1],
                                    crop_box_height);
  crop_box_.setTranslation(eigen_translation);
  crop_box_.setRotation(Eigen::Vector3f(0,0,0));

  visualization_msgs::Marker crop_box_marker;
  crop_box_marker.action = visualization_msgs::Marker::ADD;
  crop_box_marker.header.frame_id.assign(BASE_FRAME_ID);
  crop_box_marker.header.stamp = ros::Time::now();
  crop_box_marker.ns = node_handle_.getNamespace() + "_CropBox";
  crop_box_marker.type = visualization_msgs::Marker::CUBE;
  crop_box_marker.lifetime = ros::Duration();
  crop_box_marker.pose.position.x = eigen_translation(0);
  crop_box_marker.pose.position.y = eigen_translation(1);
  crop_box_marker.pose.position.z = eigen_translation(2);
  tf::Quaternion idendity_quat = tf::Quaternion::getIdentity();
  conversions::convert(idendity_quat, crop_box_marker.pose.orientation);
  crop_box_marker.scale.x = sensor_area_width;
  crop_box_marker.scale.y = sensor_area_length;
  crop_box_marker.scale.z = sensor_area_height;
  crop_box_marker.color.a = 0.18;
  crop_box_marker.color.r = 0.0;
  crop_box_marker.color.g = 1.0;
  crop_box_marker.color.b = 0.0;
  rviz_pub_.publish(crop_box_marker);

  light_frame_.binned_points.resize(node_positions_.size(), 0);
  light_frame_.header.frame_id = BASE_FRAME_ID;

//  double x_length = sensor_area_width / (double)num_x_sections;
//  double y_length = sensor_area_length / (double)num_y_sections;
//  std::pair<double, double> zero_pair(0.0, 0.0);
//  x_range_.resize(num_x_sections, zero_pair);
//  for (int i = 0; i < num_x_sections; ++i)
//  {
//    x_range_[i].first = sensor_area_min(0) + (double)i * x_length;
//    x_range_[i].second = sensor_area_min(0) + (double)(i + 1) * x_length;
//    ROS_INFO("X %i: < %.2f | %.2f >", (int)i, x_range_[i].first, x_range_[i].second);
//  }
//  y_range_.resize(num_y_sections, zero_pair);
//  for (int i = 0; i < num_y_sections; ++i)
//  {
//    y_range_[i].first = sensor_area_min(1) + (double)i * y_length;
//    y_range_[i].second = sensor_area_min(1) + (double)(i + 1) * y_length;
//    ROS_INFO("Y %i: < %.2f | %.2f >", (int)i, y_range_[i].first, y_range_[i].second);
//  }
}

void CloudProcessor::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr point_cloud)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*point_cloud, *input_cloud);

  // Create the filtering object
  pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
  voxelgrid.setInputCloud (input_cloud);
  const float Z_LEAF_SIZE = 100.0f;
  voxelgrid.setLeafSize (downsampling_leaf_size_, downsampling_leaf_size_, Z_LEAF_SIZE);
  voxelgrid.filter (*voxel_cloud);


  // transform into BASE frame
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_base_frame (new pcl::PointCloud<pcl::PointXYZ>);
  if(transform_using_tf_)
  {
    tf::StampedTransform point_cloud_to_base_transform;
    try
    {
      if (tf_listener_.waitForTransform(BASE_FRAME_ID,
                                        point_cloud->header.frame_id,
                                        point_cloud->header.stamp,
                                        ros::Duration(0.5)))
      {
        tf_listener_.lookupTransform(BASE_FRAME_ID,
                                     point_cloud->header.frame_id,
                                     point_cloud->header.stamp,
                                     point_cloud_to_base_transform);
      }
      else
      {
        ROS_ERROR("Did not receive any transformations yet.");
        return;
      }
    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR("Could not get transform from >%s< to >%s<...", BASE_FRAME_ID.c_str(), point_cloud->header.frame_id.c_str());
      return;
    }
        //    ROS_WARN("%s: kinect_to_base_transform: [%f, %f, %f, %f, %f, %f, %f]",
        //             node_handle_.getNamespace().c_str(),
        //             point_cloud_to_base_transform.getOrigin().getX(),
        //             point_cloud_to_base_transform.getOrigin().getY(),
        //             point_cloud_to_base_transform.getOrigin().getZ(),
        //             point_cloud_to_base_transform.getRotation().getW(),
        //             point_cloud_to_base_transform.getRotation().getX(),
        //             point_cloud_to_base_transform.getRotation().getY(),
        //             point_cloud_to_base_transform.getRotation().getZ());
    pcl_ros::transformPointCloud<pcl::PointXYZ>(*voxel_cloud, *cloud_in_base_frame, point_cloud_to_base_transform);
  }
  else
  {
    pcl_ros::transformPointCloud<pcl::PointXYZ>(*voxel_cloud, *cloud_in_base_frame, point_cloud_to_base_transform_);
  }

  cloud_in_base_frame->header.frame_id = BASE_FRAME_ID;
  cloud_in_base_frame->header.stamp = point_cloud->header.stamp;

  // cloud_all_pub_.publish(*cloud_in_base_frame);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  // crop points
  //  crop_box_.setInputCloud(cloud_in_base_frame);
  //  pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud_in_base_frame (new pcl::PointCloud<pcl::PointXYZ>);
  //  crop_box_.filter(*cropped_cloud_in_base_frame);

  Eigen::Vector4f vmin = crop_box_.getMin();
  Eigen::Vector4f vmax = crop_box_.getMax();

  // ROS_INFO("Pass Z. >%i< points left.", (int)cloud_in_base_frame->size());
  pcl::PassThrough<pcl::PointXYZ> pass_z;
  pass_z.setInputCloud (cloud_in_base_frame);
  pass_z.setFilterFieldName("z");
  pass_z.setFilterLimits (vmin(2), vmax(2));
  pass_z.filter (*cloud_filtered);
  *cloud_in_base_frame = *cloud_filtered;

  // ROS_INFO("Pass X. >%i< points left.", (int)cloud_in_base_frame->size());
  pcl::PassThrough<pcl::PointXYZ> pass_x;
  pass_x.setInputCloud (cloud_in_base_frame);
  pass_x.setFilterFieldName("x");
  pass_x.setFilterLimits (vmin(0), vmax(0));
  pass_x.filter (*cloud_filtered);
  *cloud_in_base_frame = *cloud_filtered;

  // ROS_INFO("Pass Y. >%i< points left.", (int)cloud_in_base_frame->size());
  pcl::PassThrough<pcl::PointXYZ> pass_y;
  pass_y.setInputCloud (cloud_in_base_frame);
  pass_y.setFilterFieldName("y");
  pass_y.setFilterLimits (vmin(1), vmax(1));
  pass_y.filter (*cloud_filtered);
  *cloud_in_base_frame = *cloud_filtered;

  cloud_cropped_pub_.publish(*cloud_in_base_frame);

  for (unsigned int i = 0; i < light_frame_.binned_points.size(); ++i)
    light_frame_.binned_points[i] = 0;

  for (unsigned int i = 0; i < cloud_in_base_frame->size(); ++i)
  {
    for (unsigned int j = 0; j < node_positions_.size(); ++j)
    {
      tf::Vector3 vec(cloud_in_base_frame->points[i].x, cloud_in_base_frame->points[i].y, 0.0);
      if ((vec - node_positions_[j]).length2() < detection_radius_)
      {
        light_frame_.binned_points[j]++;
      }
    }
  }

  light_frame_.header.stamp = point_cloud->header.stamp;
  light_frame_.header.seq = id_counter_++;

  light_frame_pub_.publish(light_frame_);

}

void CloudProcessor::offsetNodePositions(std::vector<tf::Vector3>& node_positions,
                                         const geometry_msgs::Point& offset_node)
{
  ROS_ASSERT_MSG(!node_positions.empty(), "Empty list of nodes provided, cannot offset.");
  for (unsigned int i = 0; i < node_positions.size(); ++i)
  {
    node_positions[i].setX(node_positions[i].getX() - offset_node.x);
    node_positions[i].setY(node_positions[i].getY() - offset_node.y);
    node_positions[i].setZ(node_positions[i].getZ() - offset_node.z);
  }
}

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "~");
  ros::NodeHandle node_handle("~");

  ROS_INFO("Starting >%s<.", node_handle.getNamespace().c_str());
  dec_cloud_processor::CloudProcessor cloud_processor(node_handle);

  if(!cloud_processor.initialize())
    return -1;

  ros::spin();
  // cloud_processor.run();
  return 0;
}
