/*
 * dec_components.cpp
 *
 *  Created on: Jul 18, 2013
 *      Author: pastor
 */

#include <dec_utilities/param_server.h>
#include <dec_utilities/assert.h>
#include <conversions/ros_to_tf.h>
#include <conversions/tf_to_ros.h>

#include <dec_light_show_manager/dec_components.h>

using namespace dec_utilities;
using namespace conversions;

namespace dec_light_show_manager
{

// ===================================================================
// Component
// ===================================================================

void Component::setIds(const unsigned int id)
{
  ROS_ASSERT(!poses_.empty());
  for (unsigned int i = 0; i < poses_.size(); ++i)
  {
    if (ids_.empty())
    {
      ids_.push_back(id);
    }
    else
    {
      ids_.push_back(ids_.back() + 1);
    }
  }
  ROS_ASSERT(ids_.size() == poses_.size());
}

bool Component::read(XmlRpc::XmlRpcValue& config,
                     const std::string& key,
                     std::pair<unsigned int, unsigned int>& node_pair)
{
  if (config.size() != 2)
  {
    ROS_ERROR("Number of >%s< >%i< is wrong. It should be >2<.", key.c_str(), config.size());
    return false;
  }
  if ((config[0].getType() != XmlRpc::XmlRpcValue::TypeInt)
      || (config[1].getType() != XmlRpc::XmlRpcValue::TypeInt))
  {
    ROS_ERROR("Values must either be of type integer.");
    return false;
  }
  node_pair.first = (int)config[0];
  node_pair.second = (int)config[1];
  ROS_ASSERT_MSG(node_pair.first != node_pair.second,
                 "Invalid >%s< configuration. Node indices must differ.", key.c_str());
  return true;
}

bool Component::read(XmlRpc::XmlRpcValue& rpc_values,
                     const std::string& key,
                     std::vector<std::pair<unsigned int, unsigned int> >& nodes)
{
  if (rpc_values.getType() != XmlRpc::XmlRpcValue::TypeStruct)
  {
    ROS_ERROR("Configuration >%s< must be a struct.", key.c_str());
    return false;
  }

  if (!rpc_values.hasMember(key))
  {
    ROS_ERROR("Each configuration must contain a field >%s<.", key.c_str());
    return false;
  }
  XmlRpc::XmlRpcValue rpc_value = rpc_values[key];
  if (rpc_value.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("Configuration >%s< must be an array.", key.c_str());
    return false;
  }

  nodes.clear();
  for (int i = 0; i < rpc_values.size(); ++i)
  {
    if (rpc_value.size() == 0)
    {
      ROS_ERROR("No values specified for >%s<.", key.c_str());
      return false;
    }
    if (rpc_value[0].getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
      std::pair<unsigned int, unsigned int> node_pair;
      ROS_VERIFY(read(rpc_value, key, node_pair));
      nodes.push_back(node_pair);
      return true;
    }
    else if (rpc_value[0].getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      for (int i = 0; i < rpc_value.size(); ++i)
      {
        std::pair<unsigned int, unsigned int> node_pair;
        ROS_VERIFY(read(rpc_value[i], key, node_pair));
        nodes.push_back(node_pair);
      }
      return true;
    }
    else
    {
      ROS_ERROR("Invalid configuration in >%s<.", key.c_str());
      return false;
    }
  }
  return true;
}

// ===================================================================
// Connector
// ===================================================================

bool Connector::initialize(XmlRpc::XmlRpcValue& config)
{
  ROS_VERIFY(getParam(config, "teensy", teensy_));
  ROS_ASSERT(teensy_ <= 255);

  return true;
}

// ===================================================================
// Node
// ===================================================================

bool Node::initialize(XmlRpc::XmlRpcValue& config, const unsigned int id)
{
  if (!config.hasMember("position"))
  {
    ROS_ERROR("Each configuration must contain a field >position<.");
    return false;
  }
  XmlRpc::XmlRpcValue rpc_value = config["position"];
  if (rpc_value.size() != 3)
  {
    ROS_ERROR("Number of node positions >%i< is wrong. It should be >3<.", config.size());
    return false;
  }
  std::vector<double> values;
  for (int j = 0; j < 3; ++j)
  {
    if (rpc_value[j].getType() != XmlRpc::XmlRpcValue::TypeDouble
        && rpc_value[j].getType() != XmlRpc::XmlRpcValue::TypeInt)
    {
      ROS_ERROR("Value must either be of type integer or double.");
      return false;
    }
    double value = rpc_value[j];
    const double NODE_UNIT_SCALE = 1000.0;
    value /= NODE_UNIT_SCALE;
    values.push_back(static_cast<double> (value));
  }
  geometry_msgs::Point point;
  point.x = values[0];
  point.y = values[1];
  point.z = values[2];

  geometry_msgs::Pose pose;
  pose.position = point;
  convert(tf::Quaternion::getIdentity(), pose.orientation);
  poses_.push_back(pose);

  Component::setIds(id);
  ROS_DEBUG("Node: Read >%i< node positions.", (int)poses_.size());
  return true;
}

// ===================================================================
// Beam
// ===================================================================

bool Beam::initialize(XmlRpc::XmlRpcValue& config, const unsigned int id,
                      const std::vector<geometry_msgs::Point>& node_positions)
{
  ROS_VERIFY(read(config, "nodes", nodes_));
  ROS_ASSERT(nodes_.size() > 0);
  ROS_DEBUG("Beam: Read >%i< beams.", (int)nodes_.size());
  for (unsigned int i = 0; i < nodes_.size(); ++i)
  {
    ROS_ASSERT_MSG((nodes_[i].first < node_positions.size()) && (nodes_[i].second < node_positions.size()),
        "Invalid nodes index read from >%i< to >%i<. It must be within [0, %i].",
        (int)nodes_[i].first, (int)nodes_[i].second, (int)(node_positions.size() - 1));
    ROS_ASSERT_MSG(nodes_[i].first != nodes_[i].second, "Invalid beam read. Node indices must differ.");
  }

  // this will be updated by light beams and friends
  poses_.resize(nodes_.size());
  for (unsigned int i = 0; i < nodes_.size(); ++i)
  {
    setPoses(i, node_positions, 0.5);
  }

  Component::setIds(id);
  return true;
}

void Beam::setPoses(const unsigned int local_index,
                    const std::vector<geometry_msgs::Point>& node_positions,
                    const float center)
{
  geometry_msgs::Pose led_pose;
  tf::Vector3 p1, p2;
  convert(node_positions[nodes_[local_index].first], p1);
  convert(node_positions[nodes_[local_index].second], p2);

  tf::Vector3 p1p2 = p2 - p1;
  tf::Vector3 beam_center_position = (p1 + (center * p1p2));
  convert(beam_center_position, led_pose.position);

  tf::Vector3 p12 = p2 - p1;
  p12.normalize();
  tf::Vector3 z_world = tf::Vector3(0.0, 0.0, 1.0);
  tf::Vector3 z_cross_p12 = z_world.cross(p12);
  float angle = acos(p12.dot(z_world));
  tf::Quaternion q;
  q.setRotation(z_cross_p12, angle);
  convert(q, led_pose.orientation);

  poses_[local_index] = led_pose;
}

// ===================================================================
// Sensor
// ===================================================================

bool Sensor::initialize(XmlRpc::XmlRpcValue& config, const unsigned int id,
                        const std::vector<geometry_msgs::Point>& node_positions)
{
  ROS_VERIFY(Beam::initialize(config, id, node_positions));
  ROS_VERIFY(Connector::initialize(config));

  setAvgPose();
  return true;
}

void Sensor::setAvgPose()
{
  tf::Vector3 avg_position(0.0, 0.0, 0.0);
  for (unsigned int i = 0; i < poses_.size(); ++i)
  {
    tf::Vector3 position;
    convert(poses_[i].position, position);
    avg_position += position;
  }
  avg_position /= static_cast<float>(poses_.size());
  convert(avg_position, avg_position_);
}

// ===================================================================
// Light
// ===================================================================

bool Light::initialize(XmlRpc::XmlRpcValue& config)
{
  ROS_VERIFY(Connector::initialize(config));

  ROS_VERIFY(getParam(config, "num_leds", num_leds_));
  for (unsigned int i = 0; i < num_leds_.size(); ++i)
  {
    ROS_ASSERT(num_leds_[i] > 0 && num_leds_[i] < 256);
  }
  ROS_VERIFY(getParam(config, "num_leds_per_meter", num_leds_per_meter_));
  ROS_ASSERT(num_leds_per_meter_ > 0);

  return true;
}

unsigned int Light::getTotalNumLeds() const
{
  unsigned int total_num_leds = 0;
  for (unsigned int i = 0; i < num_leds_.size(); ++i)
  {
    total_num_leds += num_leds_[i];
  }
  return total_num_leds;
}

// ===================================================================
// LightNode
// ===================================================================

bool LightNode::initialize(XmlRpc::XmlRpcValue& config, const unsigned int id,
                           const std::vector<geometry_msgs::Point>& node_positions)
{
  // Node does not need to be initialized (only setting of ids required)
  ROS_VERIFY(Light::initialize(config));

  ROS_VERIFY(getParam(config, "nodes", nodes_));
  ROS_ASSERT(nodes_.size() == num_leds_.size());
  ROS_WARN_COND(nodes_.empty(), "No light nodes read from config.");

  ROS_DEBUG("LightNode: Read >%i< light nodes.", (int)nodes_.size());
  for (unsigned int i = 0; i < nodes_.size(); ++i)
  {
    ROS_ASSERT(nodes_[i] < node_positions.size());
    geometry_msgs::Pose ros_pose;
    ros_pose.position = node_positions[nodes_[i]];
    convert(tf::Quaternion::getIdentity(), ros_pose.orientation);
    poses_.push_back(ros_pose);
  }

  setIds(id);
  for (unsigned int i = 0; i < nodes_.size(); ++i)
  {
    ROS_DEBUG(" Node >%i< has id >%i< and has >%i< LEDs and is at (%.2f %.2f %.2f).",
             (int)ids_[i], (int)nodes_[i], (int)num_leds_[i], poses_[i].position.x, poses_[i].position.y, poses_[i].position.z);
  }

  ROS_ASSERT(ids_.size() == poses_.size());
  ROS_ASSERT(nodes_.size() == poses_.size());

  return true;
}

// ===================================================================
// LightBeam
// ===================================================================

bool LightBeam::initialize(XmlRpc::XmlRpcValue& config, const unsigned int id,
                           const std::vector<geometry_msgs::Point>& node_positions)
{
  ROS_VERIFY(Beam::initialize(config, id, node_positions));
  ROS_VERIFY(Light::initialize(config));

  ROS_ASSERT_MSG(nodes_.size() == num_leds_.size(), "Number of nodes >%i< must correspond to number of LEDs >%i< in light beam.",
                 (int)nodes_.size(), (int)num_leds_.size());
  ROS_DEBUG("LightBeam: Read >%i< light beams.", (int)nodes_.size());
  ROS_VERIFY(getParam(config, "centers", centers_));
  ROS_ASSERT(nodes_.size() == centers_.size());
  for (unsigned int i = 0; i < num_leds_.size(); ++i)
  {
    ROS_ASSERT(!(centers_[i] < 0.0) && !(centers_[i] > 1.0));
  }
  ROS_ASSERT(nodes_.size() == centers_.size());

  length_.clear();
  for (unsigned int i = 0; i < num_leds_.size(); ++i)
  {
    length_.push_back(static_cast<float>(num_leds_[i]) / static_cast<float>(num_leds_per_meter_));
  }

  poses_.resize(nodes_.size());
  for (unsigned int i = 0; i < nodes_.size(); ++i)
  {
    ROS_ASSERT(!(centers_[i] < 0.0));
    ROS_ASSERT(!(centers_[i] > 1.0));
    setPoses(i, node_positions, centers_[i]);
  }

  for (unsigned int i = 0; i < num_leds_.size(); ++i)
  {
    ROS_DEBUG(" Beam >%i< has >%i< LEDs and is at (%.2f %.2f %.2f).",
             (int)ids_[i], (int)num_leds_[i], poses_[i].position.x, poses_[i].position.y, poses_[i].position.z);
  }

  return true;
}

// ===================================================================
// PixelLightBeam
// ===================================================================

bool PixelLightBeam::initialize(XmlRpc::XmlRpcValue& config, const unsigned int id,
                                const std::vector<geometry_msgs::Point>& node_positions)
{
  ROS_VERIFY(LightBeam::initialize(config, id, node_positions));
  pixel_poses_.resize(nodes_.size());
  for (unsigned int i = 0; i < nodes_.size(); ++i)
  {
    setPixelPoses(i, node_positions);
  }

  ROS_DEBUG("PixelLightBeam: Read >%i< pixel light beams.", (int)nodes_.size());
  return true;
}

void PixelLightBeam::setPixelPoses(const unsigned int local_index,
                     const std::vector<geometry_msgs::Point>& node_positions)
{
  std::vector<geometry_msgs::Pose> led_poses;
  for (unsigned int i = 0; i < getNumLeds(local_index); ++i)
  {
    float led_fragment_length = length_[local_index] / static_cast<float>(getNumLeds(local_index));
    float percent_distance_from_first_node = static_cast<float>(i) * led_fragment_length;

    geometry_msgs::Pose led_pose;
    tf::Vector3 p1, p2;
    convert(node_positions[nodes_[local_index].first], p1);
    convert(node_positions[nodes_[local_index].second], p2);

    tf::Vector3 p1p2 = p2 - p1;
    tf::Vector3 center = (p1 + (percent_distance_from_first_node * p1p2));// / 2.0f;
    convert(center, led_pose.position);

    tf::Vector3 p12 = p2 - p1;
    p12.normalize();
    tf::Vector3 z_world = tf::Vector3(0.0, 0.0, 1.0);
    tf::Vector3 z_cross_p12 = z_world.cross(p12);
    float angle = acos(p12.dot(z_world));
    tf::Quaternion q;
    q.setRotation(z_cross_p12, angle);
    convert(q, led_pose.orientation);

    led_poses.push_back(led_pose);
  }
  pixel_poses_.push_back(led_poses);
}

// ===================================================================
// BlockLightBeam
// ===================================================================

bool BlockLightBeam::initialize(XmlRpc::XmlRpcValue& config, const unsigned int id,
                                const std::vector<geometry_msgs::Point>& node_positions)
{
  ROS_VERIFY(LightBeam::initialize(config, id, node_positions));

  ROS_DEBUG("BlockLightBeam: Read >%i< block light beams.", (int)nodes_.size());
  return true;
}

}

