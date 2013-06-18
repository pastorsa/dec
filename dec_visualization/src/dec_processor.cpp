/*
 * dec_process.cpp
 *
 *  Created on: Jun 17, 2013
 *      Author: pastor
 */

#include <tf/transform_datatypes.h>
#include <conversions/ros_to_tf.h>
#include <conversions/tf_to_ros.h>

#include <dec_utilities/assert.h>
#include <dec_utilities/param_server.h>

#include <dec_visualization/dec_processor.h>

using namespace std;
using namespace conversions;

namespace dec_visualization
{

DECProcessor::DECProcessor()
{
}

bool DECProcessor::init(ros::NodeHandle node_handle)
{
  node_handle_ = node_handle;

  const int PUBLISHER_BUFFER_SIZE = 10;
  rviz_pub_ = node_handle_.advertise<visualization_msgs::MarkerArray>("visualization_marker", PUBLISHER_BUFFER_SIZE, true);

  ROS_ASSERT(DECData::init(node_handle_));

  setupNodeMarkers(node_handle_, "nodes", node_markers_, node_text_markers_, node_positions_);
  setupNodeMarkers(node_handle_, "light_nodes", light_node_markers_, light_node_text_markers_, light_node_positions_);

  setupTextMarkers(node_handle_, "node", node_markers_, node_text_markers_);
  setupTextMarkers(node_handle_, "light_node", light_node_markers_, light_node_text_markers_);

  setupBeamMarkers(node_handle_, "beams", beam_markers_, node_positions_, beams_);
  setupBeamMarkers(node_handle_, "light_beams", light_beam_markers_, node_positions_, light_beams_);
  setupBeamMarkers(node_handle_, "sensors", sensor_markers_, node_positions_, sensors_);

  setupTextMarkers(node_handle_, "beam", beam_markers_, beam_text_markers_);
  setupTextMarkers(node_handle_, "light_beam", light_beam_markers_, light_beam_text_markers_);
  setupTextMarkers(node_handle_, "sensor", sensor_markers_, sensor_text_markers_);

  for (unsigned int i = 0; i < arduino_to_light_beam_map_.size(); ++i)
  {
    for (unsigned int j = 0; j < arduino_to_light_beam_map_[i].size(); ++j)
    {
      const int INDEX = arduino_to_light_beam_map_[i][j];
      const int ENTRY = NUM_ENTRIES_FOR_ARDUINO_LEVEL + max_number_of_sensors_per_arduino_ + (j * 4);
      data_(i, ENTRY + RED_OFFSET) = static_cast<int>(light_node_markers_.markers[INDEX].color.r * COLOR_RESOLUTION);
      data_(i, ENTRY + GREEN_OFFSET) = static_cast<int>(light_node_markers_.markers[INDEX].color.g * COLOR_RESOLUTION);
      data_(i, ENTRY + BLUE_OFFSET) = static_cast<int>(light_node_markers_.markers[INDEX].color.b * COLOR_RESOLUTION);
      data_(i, ENTRY + ALPHA_OFFSET) = static_cast<int>(light_node_markers_.markers[INDEX].color.a * COLOR_RESOLUTION);
    }
  }
  for (unsigned int i = 0; i < arduino_to_light_node_map_.size(); ++i)
  {
    for (unsigned int j = 0; j < arduino_to_light_node_map_[i].size(); ++j)
    {
      const int INDEX = arduino_to_light_beam_map_[i][j];
      const int ENTRY = NUM_ENTRIES_FOR_ARDUINO_LEVEL + max_number_of_sensors_per_arduino_ + (arduino_to_light_beam_map_[i].size() * 4) + (j * 4);
      data_(i, ENTRY + RED_OFFSET) = static_cast<int>(light_node_markers_.markers[INDEX].color.r * COLOR_RESOLUTION);
      data_(i, ENTRY + GREEN_OFFSET) = static_cast<int>(light_node_markers_.markers[INDEX].color.g * COLOR_RESOLUTION);
      data_(i, ENTRY + BLUE_OFFSET) = static_cast<int>(light_node_markers_.markers[INDEX].color.b * COLOR_RESOLUTION);
      data_(i, ENTRY + ALPHA_OFFSET) = static_cast<int>(light_node_markers_.markers[INDEX].color.a * COLOR_RESOLUTION);
    }
  }

  return true;
}

void DECProcessor::setupMode()
{

}

void DECProcessor::localMode()
{
  for (unsigned int i = 0; i < arduino_to_sensor_map_.size(); ++i)
  {
    int sensor_value = 0;
    for (unsigned int j = 0; j < arduino_to_sensor_map_[i].size(); ++j)
    {
      int value = data_(i, NUM_ENTRIES_FOR_ARDUINO_LEVEL + j);
      ROS_ASSERT(value >= 0);
      sensor_value += value;
    }

    if (sensor_value > SENSOR_RESOLUTION)
      sensor_value = SENSOR_RESOLUTION;

    // scale sensor value to range 0..1
    double sensor = static_cast<double>(sensor_value) / static_cast<double>(SENSOR_RESOLUTION);

    int red = static_cast<int>(sensor * COLOR_RESOLUTION);
    int green = static_cast<int>((1-sensor) * COLOR_RESOLUTION);
    int blue = static_cast<int>(COLOR_RESOLUTION / 2.0);
    int alpha = static_cast<int>(COLOR_RESOLUTION);

    // ROS_INFO("Arduino >%i< has >%i< light beams attached.", i, (int)arduino_to_light_beam_map_[i].size());
    for (unsigned int j = 0; j < arduino_to_light_beam_map_[i].size(); ++j)
    {
      const int ENTRY = NUM_ENTRIES_FOR_ARDUINO_LEVEL + max_number_of_sensors_per_arduino_ + (j * 4);
      data_(i, ENTRY + RED_OFFSET) = red;
      data_(i, ENTRY + GREEN_OFFSET) = green;
      data_(i, ENTRY + BLUE_OFFSET) = blue;
      data_(i, ENTRY + ALPHA_OFFSET) = alpha;
    }
    for (unsigned int j = 0; j < arduino_to_light_node_map_[i].size(); ++j)
    {
      const int ENTRY = NUM_ENTRIES_FOR_ARDUINO_LEVEL + max_number_of_sensors_per_arduino_ + (arduino_to_light_beam_map_[i].size() * 4) + (j * 4);
      data_(i, ENTRY + RED_OFFSET) = red;
      data_(i, ENTRY + GREEN_OFFSET) = green;
      data_(i, ENTRY + BLUE_OFFSET) = blue;
      data_(i, ENTRY + ALPHA_OFFSET) = alpha;
    }
  }
}

bool DECProcessor::update()
{
  ROS_ASSERT(interaction_mode_ >= 0 && interaction_mode_ < (int)eNUM_MODES);

  if (interaction_mode_ == (int)eSETUP)
  {
    setupMode();
  }
  else if (interaction_mode_ == (int)eLOCAL)
  {
    localMode();
  }
  else
  {
    ROS_ERROR("Invalid interaction mode >%i<.", interaction_mode_);
    return false;
  }

  publish();
  return true;
}

bool DECProcessor::setLightMarkers()
{
  for (unsigned int i = 0; i < light_beam_connections_.size(); ++i)
  {
    const int ARDUINO_INDEX = light_beam_connections_[i];
    const int ENTRY_INDEX = NUM_ENTRIES_FOR_ARDUINO_LEVEL + max_number_of_sensors_per_arduino_ + (light_beam_index_counter_[i] * 4);
    light_beam_markers_.markers[i].color.r = static_cast<double>(data_(ARDUINO_INDEX, ENTRY_INDEX + RED_OFFSET)) / COLOR_RESOLUTION;
    light_beam_markers_.markers[i].color.g = static_cast<double>(data_(ARDUINO_INDEX, ENTRY_INDEX + GREEN_OFFSET)) / COLOR_RESOLUTION;
    light_beam_markers_.markers[i].color.b = static_cast<double>(data_(ARDUINO_INDEX, ENTRY_INDEX + BLUE_OFFSET)) / COLOR_RESOLUTION;
    light_beam_markers_.markers[i].color.a = static_cast<double>(data_(ARDUINO_INDEX, ENTRY_INDEX + ALPHA_OFFSET)) / COLOR_RESOLUTION;
  }

  for (unsigned int i = 0; i < light_node_connections_.size(); ++i)
  {
    const int ARDUINO_INDEX = light_node_connections_[i];
    const int ENTRY_INDEX = NUM_ENTRIES_FOR_ARDUINO_LEVEL + max_number_of_sensors_per_arduino_ + (arduino_to_light_beam_map_[i].size() * 4) + (light_node_index_counter_[i] * 4);
    light_node_markers_.markers[i].color.r = static_cast<double>(data_(ARDUINO_INDEX, ENTRY_INDEX + RED_OFFSET)) / COLOR_RESOLUTION;
    light_node_markers_.markers[i].color.g = static_cast<double>(data_(ARDUINO_INDEX, ENTRY_INDEX + GREEN_OFFSET)) / COLOR_RESOLUTION;
    light_node_markers_.markers[i].color.b = static_cast<double>(data_(ARDUINO_INDEX, ENTRY_INDEX + BLUE_OFFSET)) / COLOR_RESOLUTION;
    light_node_markers_.markers[i].color.a = static_cast<double>(data_(ARDUINO_INDEX, ENTRY_INDEX + ALPHA_OFFSET)) / COLOR_RESOLUTION;
  }

  return true;
}

void DECProcessor::publish()
{
  ROS_ASSERT(setLightMarkers());

  rviz_pub_.publish(node_markers_);
  rviz_pub_.publish(node_text_markers_);
  rviz_pub_.publish(beam_markers_);
  rviz_pub_.publish(beam_text_markers_);
  rviz_pub_.publish(light_beam_markers_);
  rviz_pub_.publish(light_beam_text_markers_);
  rviz_pub_.publish(light_node_markers_);
  rviz_pub_.publish(light_node_text_markers_);
  rviz_pub_.publish(sensor_markers_);
  rviz_pub_.publish(sensor_text_markers_);
}

void DECProcessor::setupTextMarkers(ros::NodeHandle& node_handle,
                                        const std::string& namespace_name,
                                        const visualization_msgs::MarkerArray& markers,
                                        visualization_msgs::MarkerArray& text_markers)
{
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

    marker.pose.position = markers.markers[i].pose.position;
    marker.pose.position.x += (double)(marker.text.length() * 0.025);

    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.07;

    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    text_markers.markers.push_back(marker);
  }
}

void DECProcessor::setupNodeMarkers(ros::NodeHandle& node_handle,
                                        const std::string& namespace_name,
                                        visualization_msgs::MarkerArray& node_markers,
                                        visualization_msgs::MarkerArray& node_text_markers,
                                        const std::vector<geometry_msgs::Point>& node_positions)
{
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

void DECProcessor::setupBeamMarkers(ros::NodeHandle& node_handle,
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
    tf::Vector3 center = (p2 + p1) / 2.0f;
    convert(center, marker.pose.position);

    tf::Vector3 p12 = p2 - p1;
    tf::Vector3 z_world = tf::Vector3(0.0, 0.0, 1.0);
    tf::Quaternion q;
    tf::Vector3 p1_cross_p2 = p12.cross(z_world);
    q.setX(p1_cross_p2.getX());
    q.setY(p1_cross_p2.getY());
    q.setZ(p1_cross_p2.getZ());
    q.setW(sqrt((p12.length2()) * (z_world.length2())) + p12.dot(z_world));
    q.normalize();
    convert(q, marker.pose.orientation);

    marker.scale.z = p12.length() * size.z;
    beam_markers.markers.push_back(marker);
  }
}

}
