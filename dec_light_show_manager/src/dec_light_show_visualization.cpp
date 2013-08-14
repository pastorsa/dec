/*
 * dec_light_show_visualization.cpp
 *
 *  Created on: Jun 17, 2013
 *      Author: pastor
 */

#include <tf/transform_datatypes.h>
#include <conversions/ros_to_tf.h>
#include <conversions/tf_to_ros.h>

#include <dec_utilities/assert.h>
#include <dec_utilities/param_server.h>

#include <dec_light_show_manager/dec_light_show_visualization.h>

using namespace std;
using namespace conversions;

namespace dec_light_show_manager
{

DecLightShowVisualization::DecLightShowVisualization()
  : static_publish_counter_(0), static_publish_rate_(20), initial_avg_sensor_alpha_(0), initial_sensor_alpha_(0)
{
}

bool DecLightShowVisualization::initialize(boost::shared_ptr<DecLightShowData> light_show_data, ros::NodeHandle node_handle)
{
  light_show_data_ = light_show_data;
  node_handle_ = node_handle;
  static_publish_counter_ = 0;

  const int PUBLISHER_BUFFER_SIZE = 10;
  rviz_pub_ = node_handle_.advertise<visualization_msgs::MarkerArray>("visualization_marker", PUBLISHER_BUFFER_SIZE, true);

  setupNodeMarkers(node_handle_, "nodes", node_markers_, light_show_data_->node_positions_, true);
  setupBeamMarkers(node_handle_, "beams", beam_markers_, light_show_data_->beam_poses_, true);
  setupBeamMarkers(node_handle_, "sensors", sensor_markers_, light_show_data_->sensor_poses_, true);
  if (!sensor_markers_.markers.empty())
  {
    initial_sensor_alpha_ = sensor_markers_.markers[0].color.a;
  }

  setupNodeMarkers(node_handle_, "block_nodes", block_light_node_markers_, light_show_data_->block_light_node_positions_);
  setupBlockBeamMarkers(node_handle_, "block_beams", block_light_beam_markers_, light_show_data_->block_light_beams_, light_show_data_->block_light_beam_poses_);
  setupPixelBeamMarkers(node_handle_, "pixel_beams", pixel_light_beam_markers_, light_show_data_->pixel_light_beams_, light_show_data_->pixel_light_beam_led_poses_);

  setupTextMarkers(node_handle_, "node", node_markers_, node_text_markers_, true);
  setupTextMarkers(node_handle_, "beam", beam_markers_, beam_text_markers_, true);
  setupTextMarkers(node_handle_, "sensor", sensor_markers_, sensor_text_markers_, true);

  setupTextMarkers(node_handle_, "block_node", block_light_node_markers_, block_light_node_text_markers_, true);
  setupTextMarkers(node_handle_, "block_beam", block_light_beam_markers_, block_light_beam_text_markers_, true);
  setupTextMarkers(node_handle_, "pixel_beam", pixel_light_beam_markers_, pixel_light_beam_text_markers_, true);

  setupPanelMarkers(node_handle_, light_show_data_->node_positions_);

  std::vector<geometry_msgs::Point> avg_sensor_positions;
  for (unsigned int i = 0; i < light_show_data_->sensors_.size(); ++i)
  {
    avg_sensor_positions.push_back(light_show_data_->sensors_[i].getAvgPosition());
  }
  ROS_ASSERT(light_show_data_->total_num_sensors_ == avg_sensor_positions.size());
  setupNodeMarkers(node_handle_, "avg_sensors", avg_sensor_markers_, avg_sensor_positions);
  if (!avg_sensor_markers_.markers.empty())
  {
    initial_avg_sensor_alpha_ = avg_sensor_markers_.markers[0].color.a;
  }
  setupTextMarkers(node_handle_, "avg_sensors", avg_sensor_markers_, avg_sensor_text_markers_, true);

  return true;
}

bool DecLightShowVisualization::update()
{
  for (unsigned int i = 0; i < light_show_data_->total_num_sensors_; ++i)
  {
    ROS_ASSERT_MSG(!(light_show_data_->sensor_levels_(i) < 0.0) && !(light_show_data_->sensor_levels_(i) > 1.0),
                   "Invalid sensor value >%f< in sensor >%i<.", light_show_data_->sensor_levels_(i), i);
    avg_sensor_markers_.markers[i].color.r = 1.0 - light_show_data_->sensor_levels_(i);
    avg_sensor_markers_.markers[i].color.g = light_show_data_->sensor_levels_(i);
    avg_sensor_markers_.markers[i].color.b = 1.0 - light_show_data_->sensor_levels_(i);
    if (light_show_data_->sensor_levels_(i) > 0.5)
      avg_sensor_markers_.markers[i].color.a = light_show_data_->sensor_levels_(i);
    else
      avg_sensor_markers_.markers[i].color.a = initial_avg_sensor_alpha_;
  }

  unsigned int sensor_index = 0;
  for (unsigned int i = 0; i < light_show_data_->sensors_.size(); ++i)
  {
    for (unsigned int j = 0; j < light_show_data_->sensors_[i].getNumComponents(); ++j)
    {
      sensor_markers_.markers[sensor_index].color.r = 1.0 - light_show_data_->sensor_levels_(i);
      sensor_markers_.markers[sensor_index].color.g = light_show_data_->sensor_levels_(i);
      sensor_markers_.markers[sensor_index].color.b = 1.0 - light_show_data_->sensor_levels_(i);
      if (light_show_data_->sensor_levels_(i) > 0.5)
        sensor_markers_.markers[sensor_index].color.a = light_show_data_->sensor_levels_(i);
      else
        sensor_markers_.markers[sensor_index].color.a = initial_sensor_alpha_;
      sensor_index++;
    }
  }

  ROS_ASSERT(light_show_data_->total_num_node_leds_ == light_show_data_->node_led_values_.cols());
  ROS_ASSERT(4 == (int)light_show_data_->node_led_values_.rows());
  for (unsigned int i = 0; i < light_show_data_->total_num_node_leds_; ++i)
  {
    block_light_node_markers_.markers[i].color.r = (float)light_show_data_->node_led_values_(RED_OFFSET, i) / 255.0f;
    block_light_node_markers_.markers[i].color.g = (float)light_show_data_->node_led_values_(GREEN_OFFSET, i) / 255.0f;
    block_light_node_markers_.markers[i].color.b = (float)light_show_data_->node_led_values_(BLUE_OFFSET, i) / 255.0f;
    block_light_node_markers_.markers[i].color.a = (float)light_show_data_->node_led_values_(BRIGHTNESS_OFFSET, i) / 255.0f;
  }


  ROS_ASSERT(light_show_data_->total_num_block_beam_leds_ == light_show_data_->block_beam_led_values_.cols());
  ROS_ASSERT(4 == (int)light_show_data_->block_beam_led_values_.rows());
  for (unsigned int i = 0; i < light_show_data_->total_num_block_beam_leds_; ++i)
  {
    block_light_beam_markers_.markers[i].color.r = static_cast<float>(light_show_data_->block_beam_led_values_(RED_OFFSET, i)) / 255.0f;
    block_light_beam_markers_.markers[i].color.g = static_cast<float>(light_show_data_->block_beam_led_values_(GREEN_OFFSET, i)) / 255.0f;
    block_light_beam_markers_.markers[i].color.b = static_cast<float>(light_show_data_->block_beam_led_values_(BLUE_OFFSET, i)) / 255.0f;
    block_light_beam_markers_.markers[i].color.a = static_cast<float>(light_show_data_->block_beam_led_values_(BRIGHTNESS_OFFSET, i)) / 255.0f;
  }

  ROS_ASSERT(light_show_data_->total_num_pixel_beam_leds_ == light_show_data_->pixel_beam_led_values_.cols());
  ROS_ASSERT(4 == (int)light_show_data_->pixel_beam_led_values_.rows());
  for (unsigned int i = 0; i < light_show_data_->total_num_pixel_beam_leds_; ++i)
  {
    pixel_light_beam_markers_.markers[i].color.r = static_cast<float>(light_show_data_->pixel_beam_led_values_(RED_OFFSET, i)) / 255.0f;
    pixel_light_beam_markers_.markers[i].color.g = static_cast<float>(light_show_data_->pixel_beam_led_values_(GREEN_OFFSET, i)) / 255.0f;
    pixel_light_beam_markers_.markers[i].color.b = static_cast<float>(light_show_data_->pixel_beam_led_values_(BLUE_OFFSET, i)) / 255.0f;
    pixel_light_beam_markers_.markers[i].color.a = static_cast<float>(light_show_data_->pixel_beam_led_values_(BRIGHTNESS_OFFSET, i)) / 255.0f;
  }

  rviz_pub_.publish(sensor_markers_);
  rviz_pub_.publish(block_light_node_markers_);
  rviz_pub_.publish(block_light_beam_markers_);
  rviz_pub_.publish(pixel_light_beam_markers_);
  rviz_pub_.publish(avg_sensor_markers_);

  static_publish_counter_++;
  if (static_publish_counter_ > static_publish_rate_)
  {
    static_publish_counter_ = 0;
    rviz_pub_.publish(node_markers_);
    rviz_pub_.publish(beam_markers_);
    rviz_pub_.publish(sensor_markers_);
    rviz_pub_.publish(node_text_markers_);
    rviz_pub_.publish(beam_text_markers_);
    rviz_pub_.publish(sensor_text_markers_);
    rviz_pub_.publish(avg_sensor_text_markers_);
    rviz_pub_.publish(block_light_node_text_markers_);
    rviz_pub_.publish(block_light_beam_text_markers_);
    rviz_pub_.publish(pixel_light_beam_text_markers_);
    rviz_pub_.publish(panel_markers_);
  }

  return true;
}

void DecLightShowVisualization::setupTextMarkers(ros::NodeHandle& node_handle,
                                                 const std::string& namespace_name,
                                                 const visualization_msgs::MarkerArray& markers,
                                                 visualization_msgs::MarkerArray& text_markers,
                                                 const bool forever)
{
  if(markers.markers.empty())
  {
    ROS_WARN("No >%s< markers specified. Not text marker added.", namespace_name.c_str());
    return;
  }

  text_markers.markers.clear();
  visualization_msgs::Marker marker;
  marker.header.frame_id.assign("/BASE");
  marker.ns = namespace_name + "s_text";
  marker.header.stamp = ros::Time::now();
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::Marker::ADD;
  if (forever)
  {
    marker.lifetime = ros::Duration();
  }
  else
  {
    marker.lifetime = ros::Duration(1.0);
  }
  convert(tf::Quaternion::getIdentity(), marker.pose.orientation);

  for (unsigned int i = 0; i < markers.markers.size(); ++i)
  {
    // add text
    marker.id = i;
    marker.text = namespace_name + "_" + boost::lexical_cast<std::string>(markers.markers[i].id);

    geometry_msgs::Vector3 scale;
    scale.x = 0.3;
    scale.y = 0.3;
    scale.z = 0.07;

    std_msgs::ColorRGBA color;
    color.r = 1.0;
    color.g = 1.0;
    color.b = 1.0;
    color.a = 1.0;

    float x_offset = marker.text.length() * 0.026f;
    float y_offset = 0.0f;
    float z_offset = 0.0f;

    if (namespace_name.compare("node") == 0)
    {
      scale.x = 0.6;
      scale.y = 0.6;
      scale.z = 0.12;
      color.r = 0.2;
      color.g = 1.0;
      color.b = 0.2;
      color.a = 1.0;
      x_offset = 0.0f;
      z_offset = 0.1f;
    }
    else if (namespace_name.compare("beam") == 0)
    {
      scale.x = 0.6;
      scale.y = 0.6;
      scale.z = 0.12;
      color.r = 1.0;
      color.g = 0.2;
      color.b = 0.2;
      color.a = 1.0;
      x_offset = 0.0f;
      y_offset = 0.0f;
      z_offset = 0.0f;
    }
    else if (namespace_name.compare("block_node") == 0)
    {
      scale.x = 0.6;
      scale.y = 0.6;
      scale.z = 0.12;
      color.r = 0.2;
      color.g = 0.2;
      color.b = 1.0;
      color.a = 1.0;
      x_offset = 0.0f;
      y_offset = 0.0f;
      z_offset = -0.17f;
    }
    else if (namespace_name.compare("block_beam") == 0)
    {
      scale.x = 0.6;
      scale.y = 0.6;
      scale.z = 0.12;
      color.r = 0.9;
      color.g = 0.9;
      color.b = 0.2;
      color.a = 1.0;
      x_offset = 0.0f;
      y_offset = 0.0f;
      z_offset = 0.0f;
    }
    else if (namespace_name.compare("sensor") == 0)
    {
      scale.x = 0.6;
      scale.y = 0.6;
      scale.z = 0.12;
      color.r = 0.0;
      color.g = 1.0;
      color.b = 1.0;
      color.a = 1.0;
      x_offset = 0.0f;
      y_offset = 0.0f;
      z_offset = 0.0f;
    }
    else if (namespace_name.compare("avg_sensors") == 0)
    {
      scale.x = 0.6;
      scale.y = 0.6;
      scale.z = 0.12;
      color.r = 0.0;
      color.g = 1.0;
      color.b = 1.0;
      color.a = 1.0;
      x_offset = 0.0f;
      y_offset = 0.0f;
      z_offset = 0.0f;
    }

    marker.scale = scale;
    marker.color = color;

    marker.pose.position = markers.markers[i].pose.position;
    marker.pose.position.x += x_offset;
    marker.pose.position.y += y_offset;
    marker.pose.position.z += z_offset;

    text_markers.markers.push_back(marker);
  }
}

void DecLightShowVisualization::setupNodeMarkers(ros::NodeHandle& node_handle,
                                                 const std::string& namespace_name,
                                                 visualization_msgs::MarkerArray& node_markers,
                                                 const std::vector<geometry_msgs::Point>& node_positions,
                                                 const bool forever)
{
  if(node_positions.empty())
  {
    ROS_WARN("No >%s< setup.", namespace_name.c_str());
    return;
  }

  node_markers.markers.clear();
  visualization_msgs::Marker marker;
  marker.header.frame_id.assign("/BASE");
  marker.ns = namespace_name;
  marker.header.stamp = ros::Time::now();
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  if (forever)
  {
    marker.lifetime = ros::Duration();
  }
  else
  {
    marker.lifetime = ros::Duration(1.0);
  }
  geometry_msgs::Vector3 size;
  ROS_VERIFY(dec_utilities::read(node_handle, namespace_name + "_size", size));
  marker.scale = size;

  std::vector<double> color;
  ROS_VERIFY(dec_utilities::read(node_handle, namespace_name + "_color", color));
  ROS_ASSERT(color.size() == 4);
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = color[3];

  for (unsigned int i = 0; i < node_positions.size(); ++i)
  {
    marker.id = i;
    marker.pose.position = node_positions[i];
    marker.pose.orientation.w = 1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    node_markers.markers.push_back(marker);
  }
  ROS_DEBUG("Created >%i< >%s< nodes.", (int)node_markers.markers.size(), namespace_name.c_str());
}

void DecLightShowVisualization::setupBeamMarkers(ros::NodeHandle& node_handle,
                                                 const std::string& namespace_name,
                                                 visualization_msgs::MarkerArray& beam_markers,
                                                 const std::vector<geometry_msgs::Pose>& poses,
                                                 const bool forever)
{
  beam_markers.markers.clear();
  visualization_msgs::Marker marker;
  marker.header.frame_id.assign("/BASE");
  marker.ns = namespace_name;
  marker.header.stamp = ros::Time::now();
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;
  if (forever)
  {
    marker.lifetime = ros::Duration();
  }
  else
  {
    marker.lifetime = ros::Duration(1.0);
  }
  geometry_msgs::Vector3 size;
  ROS_VERIFY(dec_utilities::read(node_handle, namespace_name + "_size", size));
  // ROS_ASSERT(size.z > 0.0 && size.z <= 1.0);
  marker.scale = size;
  // marker.scale.x *= 0.8;
  // marker.scale.y *= 0.8;

  std::vector<double> color;
  ROS_VERIFY(dec_utilities::read(node_handle, namespace_name + "_color", color));
  ROS_ASSERT(color.size() == 4);
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = color[3];

  for (unsigned int i = 0; i < poses.size(); ++i)
  {
    marker.id = i;
    marker.pose = poses[i];
    beam_markers.markers.push_back(marker);
  }
  ROS_DEBUG("Created >%i< >%s< beams.", (int)beam_markers.markers.size(), namespace_name.c_str());
}

void DecLightShowVisualization::setupBlockBeamMarkers(ros::NodeHandle& node_handle,
                                                 const std::string& namespace_name,
                                                 visualization_msgs::MarkerArray& beam_markers,
                                                 const std::vector<BlockLightBeam>& block_light_beams,
                                                 const std::vector<geometry_msgs::Pose>& poses,
                                                 const bool forever)
{
  setupBeamMarkers(node_handle, namespace_name, beam_markers, poses, forever);

  std::vector<float> lengths;
  for (unsigned int i = 0; i < block_light_beams.size(); ++i)
  {
    for (unsigned int j = 0; j < block_light_beams[i].getNumComponents(); ++j)
    {
      // float length = 1.0f / 60.0f;
      // lengths.push_back(length);
      lengths.push_back(block_light_beams[i].length_[j]);
    }
  }

  for (unsigned int i = 0; i < poses.size(); ++i)
  {
    beam_markers.markers[i].scale.z = lengths[i];
  }
}


void DecLightShowVisualization::setupPixelBeamMarkers(ros::NodeHandle& node_handle,
                                                      const std::string& namespace_name,
                                                      visualization_msgs::MarkerArray& beam_markers,
                                                      const std::vector<PixelLightBeam>& pixel_light_beams,
                                                      const std::vector<geometry_msgs::Pose>& poses)
{
  beam_markers.markers.clear();
  visualization_msgs::Marker marker;
  marker.header.frame_id.assign("/BASE");
  marker.ns = namespace_name;
  marker.header.stamp = ros::Time::now();
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration(1.0);

  std::vector<float> lengths;
  for (unsigned int i = 0; i < pixel_light_beams.size(); ++i)
  {
    for (unsigned int j = 0; j < pixel_light_beams[i].getNumComponents(); ++j)
    {
      for (unsigned int n = 0; n < pixel_light_beams[i].getNumLeds(j); ++n)
      {
        // float length = 1.0f / 60.0f;
        // lengths.push_back(length);
        lengths.push_back(pixel_light_beams[i].length_[j]);
      }
    }
  }

  ROS_ASSERT_MSG(lengths.size() == poses.size(), "Number of lengths >%i< does not correspond to number of poses provided >%i<.",
                 (int)lengths.size(), (int)poses.size());

  geometry_msgs::Vector3 size;
  ROS_VERIFY(dec_utilities::read(node_handle, namespace_name + "_size", size));
  // ROS_ASSERT(size.z > 0.0 && size.z <= 1.0);
  marker.scale = size;

  std::vector<double> color;
  ROS_VERIFY(dec_utilities::read(node_handle, namespace_name + "_color", color));
  ROS_ASSERT(color.size() == 4);
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = color[3];

  for (unsigned int i = 0; i < poses.size(); ++i)
  {
    marker.id = i;
    marker.pose = poses[i];
    marker.scale.z = lengths[i];
    beam_markers.markers.push_back(marker);
  }
  ROS_DEBUG("Created %i >%s< beams.", (int)beam_markers.markers.size(), namespace_name.c_str());
}

void DecLightShowVisualization::setupPanelMarkers(ros::NodeHandle& node_handle,
                                                  const std::vector<geometry_msgs::Point>& node_positions)
{
  panel_markers_.markers.clear();
  std::vector<unsigned int> panel_indices;
  const std::string KEY = "panels";
  if (node_handle.hasParam(KEY))
  {
    ROS_VERIFY(dec_utilities::read(node_handle, KEY, panel_indices));
    ROS_ASSERT(panel_indices.size() > 0 && ((panel_indices.size() % 3) == 0));

    visualization_msgs::Marker panel_marker;
    panel_marker.header.frame_id.assign("/BASE");
    panel_marker.ns = KEY;
    panel_marker.header.stamp = ros::Time::now();
    panel_marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    panel_marker.action = visualization_msgs::Marker::ADD;
    panel_marker.lifetime = ros::Duration();

    std::vector<double> color;
    ROS_VERIFY(dec_utilities::read(node_handle, KEY + "_color", color));
    ROS_ASSERT(color.size() == 4);
    panel_marker.color.r = color[0];
    panel_marker.color.g = color[1];
    panel_marker.color.b = color[2];
    panel_marker.color.a = color[3];

    panel_marker.scale.x = 1.0;
    panel_marker.scale.y = 1.0;
    panel_marker.scale.z = 1.0;

    panel_marker.id = 0;
    panel_marker.points.resize(panel_indices.size());
    for (unsigned int i = 0; i < panel_indices.size(); ++i)
    {
      ROS_ASSERT(panel_indices[i] < node_positions.size());
      panel_marker.points[i] = node_positions[panel_indices[i]];
    }
    panel_markers_.markers.push_back(panel_marker);
    ROS_DEBUG("Created >%i< panels.",(int)(panel_indices.size() / 3));
  }
  else
  {
    ROS_WARN("No panels read from configuration. Not displaying.");
  }
}

//void DecLightShowVisualization::setupLightBeamMarkers()
//{
//  pixel_light_beam_markers_.markers.clear();
//  visualization_msgs::Marker marker;
//  marker.header.frame_id.assign("/BASE");
//  marker.ns = "light_beam_leds";
//  marker.header.stamp = ros::Time::now();
//  marker.type = visualization_msgs::Marker::CYLINDER;
//  marker.action = visualization_msgs::Marker::ADD;
//  marker.lifetime = ros::Duration(1.0);
//  marker.scale = light_show_data_->light_beams_size_;
//
//  std::vector<double> color;
//  ROS_VERIFY(dec_utilities::read(node_handle_, "light_beams_color", color));
//  ROS_ASSERT(color.size() == 4);
//  marker.color.r = color[0];
//  marker.color.g = color[1];
//  marker.color.b = color[2];
//  marker.color.a = color[3];
//
//  /*
//  int num_led_index = 0;
//  for (unsigned int i = 0; i < light_show_data_->num_leds_of_each_light_beam_.size(); ++i)
//  {
//    for (int led = 0; led < light_show_data_->num_leds_of_each_light_beam_[i]; ++led)
//    {
//      geometry_msgs::Pose led_pose = light_show_data_->getBeamLedPose(i, led, light_show_data_->num_leds_of_each_light_beam_[i]);
//      marker.id = num_led_index++;
//      marker.scale.z = light_show_data_->light_beams_size_.z / (float)light_show_data_->num_leds_of_each_light_beam_[i];
//      marker.pose = led_pose;
//      light_beam_led_markers_.markers.push_back(marker);
//    }
//  }
//  */
//}

}
