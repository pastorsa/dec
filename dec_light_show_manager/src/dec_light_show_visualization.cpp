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
{
}

bool DecLightShowVisualization::initialize(boost::shared_ptr<DecLightShowData> light_show_data, ros::NodeHandle node_handle)
{
  light_show_data_ = light_show_data;
  node_handle_ = node_handle;

  const int PUBLISHER_BUFFER_SIZE = 10;
  rviz_pub_ = node_handle_.advertise<visualization_msgs::MarkerArray>("visualization_marker", PUBLISHER_BUFFER_SIZE, true);

  /*
  setupNodeMarkers(node_handle_, "nodes", node_markers_, light_show_data_->node_positions_);
  setupNodeMarkers(node_handle_, "light_nodes", light_node_markers_, light_show_data_->light_node_positions_);

  setupBeamMarkers(node_handle_, "beams", beam_markers_, light_show_data_->node_positions_, light_show_data_->beams_);
  setupBeamMarkers(node_handle_, "sensors", sensor_markers_, light_show_data_->node_positions_, light_show_data_->sensors_);

  setupLightBeamMarkers();
  setupBeamMarkers(node_handle_, "light_beams", light_beam_markers_, light_show_data_->node_positions_, light_show_data_->light_beams_);

  setupTextMarkers(node_handle_, "node", node_markers_, node_text_markers_);
  setupTextMarkers(node_handle_, "light_node", light_node_markers_, light_node_text_markers_);
  setupTextMarkers(node_handle_, "beam", beam_markers_, beam_text_markers_);
  setupTextMarkers(node_handle_, "light_beam", light_beam_markers_, light_beam_text_markers_);
  setupTextMarkers(node_handle_, "sensor", sensor_markers_, sensor_text_markers_);
*/
  return true;
}

bool DecLightShowVisualization::update()
{
  /*
  for (unsigned int i = 0; i < light_show_data_->total_num_sensors_; ++i)
  {
    ROS_ASSERT_MSG(!(light_show_data_->sensor_levels_(i) < 0.0) && !(light_show_data_->sensor_levels_(i) > 1.0),
                   "Invalid sensor value >%f< in sensor >%i<.", light_show_data_->sensor_levels_(i), i);
    sensor_markers_.markers[i].color.g = light_show_data_->sensor_levels_(i);
    sensor_markers_.markers[i].color.b = 1.0 - light_show_data_->sensor_levels_(i);
  }

  ROS_ASSERT(light_show_data_->total_num_node_leds_ == light_show_data_->node_led_values_.cols());
  ROS_ASSERT(4 == (int)light_show_data_->node_led_values_.rows());
  for (unsigned int i = 0; i < light_show_data_->total_num_node_leds_; ++i)
  {
    light_node_markers_.markers[i].color.r = (float)light_show_data_->node_led_values_(RED_OFFSET, i) / 255.0f;
    light_node_markers_.markers[i].color.g = (float)light_show_data_->node_led_values_(GREEN_OFFSET, i) / 255.0f;
    light_node_markers_.markers[i].color.b = (float)light_show_data_->node_led_values_(BLUE_OFFSET, i) / 255.0f;
    light_node_markers_.markers[i].color.a = (float)light_show_data_->node_led_values_(ALPHA_OFFSET, i) / 255.0f;
  }

  ROS_ASSERT(light_show_data_->total_num_beam_leds_ == light_show_data_->beam_led_values_.cols());
  ROS_ASSERT(4 == (int)light_show_data_->beam_led_values_.rows());
  for (unsigned int i = 0; i < light_show_data_->total_num_beam_leds_; ++i)
  {
    light_beam_markers_.markers[i].color.r = static_cast<float>(light_show_data_->beam_led_values_(RED_OFFSET, i)) / 255.0f;
    light_beam_markers_.markers[i].color.g = static_cast<float>(light_show_data_->beam_led_values_(GREEN_OFFSET, i)) / 255.0f;
    light_beam_markers_.markers[i].color.b = static_cast<float>(light_show_data_->beam_led_values_(BLUE_OFFSET, i)) / 255.0f;
    light_beam_markers_.markers[i].color.a = static_cast<float>(light_show_data_->beam_led_values_(ALPHA_OFFSET, i)) / 255.0f;
  }

  rviz_pub_.publish(node_markers_);
  rviz_pub_.publish(beam_markers_);
  rviz_pub_.publish(sensor_markers_);
  rviz_pub_.publish(light_node_markers_);
  rviz_pub_.publish(light_beam_markers_);
  rviz_pub_.publish(light_beam_led_markers_);
  rviz_pub_.publish(node_text_markers_);
  rviz_pub_.publish(beam_text_markers_);
  rviz_pub_.publish(sensor_text_markers_);
  rviz_pub_.publish(light_beam_text_markers_);
  rviz_pub_.publish(light_node_text_markers_);
*/
  return true;
}

void DecLightShowVisualization::setupTextMarkers(ros::NodeHandle& node_handle,
                                                 const std::string& namespace_name,
                                                 const visualization_msgs::MarkerArray& markers,
                                                 visualization_msgs::MarkerArray& text_markers)
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
  marker.lifetime = ros::Duration(1.0);
  convert(tf::Quaternion::getIdentity(), marker.pose.orientation);

  for (unsigned int i = 0; i < markers.markers.size(); ++i)
  {
    // add text
    marker.id = i;
    marker.text = namespace_name + "_" + boost::lexical_cast<std::string>(markers.markers[i].id);

    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.07;

    marker.pose.position = markers.markers[i].pose.position;
    marker.pose.position.x += (double)(marker.text.length() * 0.026);

    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    text_markers.markers.push_back(marker);
  }
}

void DecLightShowVisualization::setupNodeMarkers(ros::NodeHandle& node_handle,
                                                 const std::string& namespace_name,
                                                 visualization_msgs::MarkerArray& node_markers,
                                                 const std::vector<geometry_msgs::Point>& node_positions)
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
  marker.lifetime = ros::Duration(1.0);

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
}

void DecLightShowVisualization::setupBeamMarkers(ros::NodeHandle& node_handle,
                                                 const std::string& namespace_name,
                                                 visualization_msgs::MarkerArray& beam_markers,
                                                 const std::vector<geometry_msgs::Point>& node_positions,
                                                 const std::vector<std::pair<int, int> >& beams)
{
  beam_markers.markers.clear();
  visualization_msgs::Marker marker;
  marker.header.frame_id.assign("/BASE");
  marker.ns = namespace_name;
  marker.header.stamp = ros::Time::now();
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration(1.0);

  geometry_msgs::Vector3 size;
  ROS_VERIFY(dec_utilities::read(node_handle, namespace_name + "_size", size));
  ROS_ASSERT(size.z > 0.0 && size.z <= 1.0);
  marker.scale = size;
  marker.scale.x *= 0.8;
  marker.scale.y *= 0.8;

  std::vector<double> color;
  ROS_VERIFY(dec_utilities::read(node_handle, namespace_name + "_color", color));
  ROS_ASSERT(color.size() == 4);
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = color[3];

  for (unsigned int i = 0; i < beams.size(); ++i)
  {
    marker.id = i;
    tf::Vector3 p1, p2;
    convert(node_positions[beams[i].first], p1);
    convert(node_positions[beams[i].second], p2);
    tf::Vector3 center = (p1 + p2) / 2.0f;
    convert(center, marker.pose.position);

    tf::Vector3 p12 = p2 - p1;
    marker.scale.z = p12.length() * size.z;
    p12.normalize();
    tf::Vector3 z_world = tf::Vector3(0.0, 0.0, 1.0);
    tf::Vector3 z_cross_p12 = z_world.cross(p12);
    double angle = acos(p12.dot(z_world));
    tf::Quaternion q;
    q.setRotation(z_cross_p12, angle);
    convert(q, marker.pose.orientation);
    beam_markers.markers.push_back(marker);
  }
}

void DecLightShowVisualization::setupLightBeamMarkers()
{
  light_beam_led_markers_.markers.clear();
  visualization_msgs::Marker marker;
  marker.header.frame_id.assign("/BASE");
  marker.ns = "light_beam_leds";
  marker.header.stamp = ros::Time::now();
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration(1.0);
  marker.scale = light_show_data_->light_beams_size_;

  std::vector<double> color;
  ROS_VERIFY(dec_utilities::read(node_handle_, "light_beams_color", color));
  ROS_ASSERT(color.size() == 4);
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = color[3];

  /*
  int num_led_index = 0;
  for (unsigned int i = 0; i < light_show_data_->num_leds_of_each_light_beam_.size(); ++i)
  {
    for (int led = 0; led < light_show_data_->num_leds_of_each_light_beam_[i]; ++led)
    {
      geometry_msgs::Pose led_pose = light_show_data_->getBeamLedPose(i, led, light_show_data_->num_leds_of_each_light_beam_[i]);
      marker.id = num_led_index++;
      marker.scale.z = light_show_data_->light_beams_size_.z / (float)light_show_data_->num_leds_of_each_light_beam_[i];
      marker.pose = led_pose;
      light_beam_led_markers_.markers.push_back(marker);
    }
  }
  */
}

}
