/*
 * dec_light_show_plane_creator.cpp
 *
 *  Created on: Jul 14, 2013
 *      Author: pastor
 */

#include <dec_utilities/assert.h>
#include <dec_utilities/param_server.h>
#include <conversions/tf_to_ros.h>
#include <conversions/ros_to_tf.h>
#include <dec_light_show_manager/dec_light_show_utilities.h>

#include <angles/angles.h>
#include <dec_msgs/LightShow.h>
#include <dec_light_shows/dec_light_show_plane_creator.h>

PLUGINLIB_DECLARE_CLASS(dec_light_shows, DecLightShowPlaneCreator, dec_light_shows::DecLightShowPlaneCreator,
                        dec_light_show_manager::DecLightShow)

using namespace dec_light_show_manager;
using namespace conversions;

namespace dec_light_shows
{

DecLightShowPlaneCreator::DecLightShowPlaneCreator() :
    node_handle_("/DecLightShowManager"), visualization_mode_(false),
    visualization_rate_(0), visualization_counter_(0),
    min_distance_(0.0), max_distance_(0.0), distance_range_(0.0),
    up_down_(false), profile_type_(MathUtilities::eLINEAR)
{
  ROS_VERIFY(dec_utilities::read(node_handle_, "visualization_mode", visualization_mode_));
  if (visualization_mode_)
  {
    const int PUBLISHER_BUFFER_SIZE = 10;
    rviz_pub_ = node_handle_.advertise<visualization_msgs::MarkerArray>("visualization_marker", PUBLISHER_BUFFER_SIZE, true);
  }
}

bool DecLightShowPlaneCreator::initialize(XmlRpc::XmlRpcValue& config)
{
  block_node_buffer_ = Eigen::VectorXf::Zero(data_->total_num_node_leds_);
  if (data_->total_num_block_beam_leds_ > 0)
  {
    block_beam_buffer_ = Eigen::VectorXf::Zero(data_->total_num_block_beam_leds_);
  }
  if (data_->total_num_pixel_beam_leds_ > 0)
  {
    pixel_beam_buffer_ = Eigen::VectorXf::Zero(data_->total_num_pixel_beam_leds_);
  }

  block_light_node_positions_.resize(data_->block_light_node_positions_.size());
  for (unsigned int i = 0; i < data_->block_light_node_positions_.size(); ++i)
  {
    convert(data_->block_light_node_positions_[i], block_light_node_positions_[i]);
  }
  block_light_beam_positions_.resize(data_->block_light_beam_poses_.size());
  for (unsigned int i = 0; i < data_->block_light_beam_poses_.size(); ++i)
  {
    convert(data_->block_light_beam_poses_[i].position, block_light_beam_positions_[i]);
  }
  pixel_light_beam_positions_.resize(data_->pixel_light_beam_led_poses_.size());
  for (unsigned int i = 0; i < data_->pixel_light_beam_led_poses_.size(); ++i)
  {
    convert(data_->pixel_light_beam_led_poses_[i].position, pixel_light_beam_positions_[i]);
  }

  readParameters(config);
  setupSpace(config);
  setupSensorMarkers(config);

  return true;
}

bool DecLightShowPlaneCreator::start()
{
  visualization_counter_ = 0;
  return true;
}

bool DecLightShowPlaneCreator::update()
{
  integrate();
  computeDistance();

  if(visualization_mode_)
    publish();
  return true;
}

bool DecLightShowPlaneCreator::stop()
{

  return true;
}

float DecLightShowPlaneCreator::computeDistance(const tf::Vector3& point,
                                                const tf::Vector3& plane_vector,
                                                const tf::Vector3& plane_normal)
{
  return fabs(-plane_normal.dot(point - plane_vector));
}

void DecLightShowPlaneCreator::computeDistance()
{
  for (unsigned int i = 0; i < positions_.size(); ++i)
  {
    // =====================================================
    // Block Nodes
    // =====================================================
    for (unsigned int j = 0; j < block_light_node_positions_.size(); ++j)
    {
      float distance = computeDistance(block_light_node_positions_[j], positions_[i], normals_[i]);
      if (distance < max_distance_)
      {
        // data_->node_led_levels_(j) -= DecData::BASE_LIGHT_LEVEL * static_cast<float>((max_distance_ - distance) / max_distance_);
        float scaled_distance = DecData::MIN_LIGHT_LEVEL + (distance / max_distance_) * DecData::BASE_LIGHT_LEVEL;
        data_->node_led_levels_(j) -= MathUtilities::ramp(DecData::MIN_LIGHT_LEVEL, DecData::MIN_LIGHT_LEVEL + DecData::BASE_LIGHT_LEVEL, scaled_distance, profile_type_);
      }
      if (data_->node_led_levels_(j) < DecData::MIN_LIGHT_LEVEL)
      {
        data_->node_led_levels_(j) = DecData::MIN_LIGHT_LEVEL;
      }
    }

    // =====================================================
    // Block Beams
    // =====================================================
    for (unsigned int j = 0; j < block_light_beam_positions_.size(); ++j)
    {
      float distance = computeDistance(block_light_beam_positions_[j], positions_[i], normals_[i]);
      if (distance < max_distance_)
      {
        // data_->block_beam_led_levels_(j) -= DecData::BASE_LIGHT_LEVEL * static_cast<float>((max_distance_ - distance) / max_distance_);
        float scaled_distance = DecData::MIN_LIGHT_LEVEL + (distance / max_distance_) * DecData::BASE_LIGHT_LEVEL;
        data_->block_beam_led_levels_(j) -= MathUtilities::ramp(DecData::MIN_LIGHT_LEVEL, DecData::MIN_LIGHT_LEVEL + DecData::BASE_LIGHT_LEVEL, scaled_distance, profile_type_);
      }
      if (data_->block_beam_led_levels_(j) < DecData::MIN_LIGHT_LEVEL)
      {
        data_->block_beam_led_levels_(j) = DecData::MIN_LIGHT_LEVEL;
      }
    }

    // =====================================================
    // Pixel Beams
    // =====================================================
    for (unsigned int j = 0; j < pixel_light_beam_positions_.size(); ++j)
    {
      float distance = computeDistance(pixel_light_beam_positions_[j], positions_[i], normals_[i]);
      if (distance < max_distance_)
      {
        // data_->pixel_beam_led_levels_(j) -= DecData::BASE_LIGHT_LEVEL * static_cast<float>((max_distance_ - distance) / max_distance_);
        float scaled_distance = DecData::MIN_LIGHT_LEVEL + (distance / max_distance_) * DecData::BASE_LIGHT_LEVEL;
        data_->pixel_beam_led_levels_(j) -= MathUtilities::ramp(DecData::MIN_LIGHT_LEVEL, DecData::MIN_LIGHT_LEVEL + DecData::BASE_LIGHT_LEVEL, scaled_distance, profile_type_);
      }
      if (data_->pixel_beam_led_levels_(j) < DecData::MIN_LIGHT_LEVEL)
      {
        data_->pixel_beam_led_levels_(j) = DecData::MIN_LIGHT_LEVEL;
      }
    }
  }
}

void DecLightShowPlaneCreator::integrate()
{
  for (unsigned int i = 0; i < positions_.size(); ++i)
  {
    positions_[i] += linear_velocities_[i] * data_->control_dt_;
    // velocities_[i] += simulated_accelerations_[i] * data_->control_dt_;

    tf::Matrix3x3 matrix;
    matrix.setRPY(angular_velocities_[i].getY(),
                  angular_velocities_[i].getX(),
                  angular_velocities_[i].getZ());
    normals_[i] = matrix * normals_[i];

    bool max_x = positions_[i].getX() > max_space_.getX();
    if (max_x || positions_[i].getX() < min_space_.getX())
    {
      if (up_down_)
      {
        linear_velocities_[i].setX(linear_velocities_[i].getX() * -1.0f);
        linear_accelerations_[i].setX(linear_accelerations_[i].getX() * -1.0f);
      }
      else
      {
        if (max_x)
        {
          positions_[i].setX(min_space_.getX());
        }
        else
        {
          positions_[i].setX(max_space_.getX());
        }
      }
    }
    bool max_y = positions_[i].getY() > max_space_.getY();
    if (max_y || positions_[i].getY() < min_space_.getY())
    {
      if (up_down_)
      {
        linear_velocities_[i].setY(linear_velocities_[i].getY() * -1.0f);
        linear_accelerations_[i].setY(linear_accelerations_[i].getY() * -1.0f);
      }
      else
      {
        if (max_y)
        {
          positions_[i].setY(min_space_.getY());
        }
        else
        {
          positions_[i].setY(max_space_.getY());
        }
      }
    }
    bool max_z = positions_[i].getZ() > max_space_.getZ();
    if (max_z || positions_[i].getZ() < min_space_.getZ())
    {
      if (up_down_)
      {
        linear_velocities_[i].setZ(linear_velocities_[i].getZ() * -1.0f);
        linear_accelerations_[i].setZ(linear_accelerations_[i].getZ() * -1.0f);
      }
      else
      {
        if (max_z)
        {
          positions_[i].setZ(min_space_.getZ());
        }
        else
        {
          positions_[i].setZ(max_space_.getZ());
        }
      }
    }

    // simulated_accelerations_[i] = accelerations_[i] * sin(data_->ros_time_sec_);
  }

  // ROS_WARN("Position: %.2f %.2f %.2f", positions_[0].getX(), positions_[0].getY(), positions_[0].getZ());
  // ROS_WARN("Velocity: %.2f %.2f %.2f", velocities_[0].getX(), velocities_[0].getY(), velocities_[0].getZ());
  // ROS_WARN("Acceleration: %.2f %.2f %.2f", simulated_accelerations_[0].getX(), simulated_accelerations_[0].getY(), simulated_accelerations_[0].getZ());

  convert(positions_[0], virtual_sensors_.markers[0].pose.position);
  convert(positions_[0], virtual_sensors_.markers[1].pose.position);
  virtual_sensors_.markers[0].pose.orientation = getOrientation(normals_[0]);
  virtual_sensors_.markers[1].pose.orientation = getOrientation(normals_[0]);
}

geometry_msgs::Quaternion DecLightShowPlaneCreator::getOrientation(tf::Vector3& normal)
{
  normal.normalize();
  tf::Vector3 z_world = tf::Vector3(0.0, 0.0, 1.0);
  tf::Vector3 z_cross_p12 = z_world.cross(normal);
  float angle = acos(normal.dot(z_world));
  tf::Quaternion quat = tf::Quaternion::getIdentity();
  if (fabs(angle) > 1e-4 && z_cross_p12.length() > 1e-4)
  {
    quat.setRotation(z_cross_p12, angle);
  }
  geometry_msgs::Quaternion q;
  convert(quat, q);
  return q;
}

void DecLightShowPlaneCreator::publish()
{
  ros::Time now = ros::Time::now();
  if(visualization_counter_ > visualization_rate_)
  {
    for (unsigned int i = 0; i < virtual_sensors_.markers.size(); ++i)
    {
      virtual_sensors_.markers[i].header.stamp = now;
    }
    visualization_counter_ = 0;
    rviz_pub_.publish(virtual_sensors_);
  }
  visualization_counter_++;
}

void DecLightShowPlaneCreator::setupSpace(XmlRpc::XmlRpcValue& config)
{
  Eigen::VectorXf x_values = Eigen::VectorXf::Zero(data_->getNumNodes());
  Eigen::VectorXf y_values = Eigen::VectorXf::Zero(data_->getNumNodes());
  Eigen::VectorXf z_values = Eigen::VectorXf::Zero(data_->getNumNodes());

  for (unsigned int i = 0; i < data_->getNumNodes(); ++i)
  {
    x_values(i) = data_->getNodePosition(i).x;
    y_values(i) = data_->getNodePosition(i).y;
    z_values(i) = data_->getNodePosition(i).z;
  }

  min_space_.setX(x_values.minCoeff());
  min_space_.setY(y_values.minCoeff());
  min_space_.setZ(z_values.minCoeff());

  max_space_.setX(x_values.maxCoeff());
  max_space_.setY(y_values.maxCoeff());
  max_space_.setZ(z_values.maxCoeff());

  min_space_.setZ(min_space_.getZ() + max_distance_);
  max_space_.setZ(max_space_.getZ() - max_distance_);

  // max_space_ /= 2.0f;
  // min_space_ /= 2.0f;
  // tf::Vector3 scale = (max_space_ - min_space_).absolute() / 2.0f;
  // max_space_.setZ(max_space_.getZ() + scale.getZ());
  // min_space_.setZ(min_space_.getZ() + scale.getZ());
}

void DecLightShowPlaneCreator::readParameters(XmlRpc::XmlRpcValue& config)
{
  ROS_VERIFY(DecLightShowUtilities::getParam(config, "visualization_rate", visualization_rate_));

  std::vector<float> initial_position;
  ROS_VERIFY(DecLightShowUtilities::getParam(config, "initial_position", initial_position));
  ROS_ASSERT(initial_position.size() == 3);
  tf::Vector3 position(initial_position[0], initial_position[1], initial_position[2]);
  positions_.push_back(position);

  std::vector<float> initial_normal;
  ROS_VERIFY(DecLightShowUtilities::getParam(config, "initial_normal", initial_normal));
  ROS_ASSERT(initial_normal.size() == 3);
  tf::Vector3 normal(initial_normal[0], initial_normal[1], initial_normal[2]);
  normal.normalize();
  normals_.resize(positions_.size(), normal);

  std::vector<float> initial_linear_velocity;
  ROS_VERIFY(DecLightShowUtilities::getParam(config, "initial_linear_velocity", initial_linear_velocity));
  ROS_ASSERT(initial_linear_velocity.size() == 3);
  tf::Vector3 linear_velocity(initial_linear_velocity[0], initial_linear_velocity[1], initial_linear_velocity[2]);
  linear_velocities_.push_back(linear_velocity);

  std::vector<float> initial_angular_velocity;
  ROS_VERIFY(DecLightShowUtilities::getParam(config, "initial_angular_velocity", initial_angular_velocity));
  ROS_ASSERT(initial_angular_velocity.size() == 3);
  tf::Vector3 angular_velocity(initial_angular_velocity[0], initial_angular_velocity[1], initial_angular_velocity[2]);
  angular_velocities_.push_back(angular_velocity);

  std::vector<float> initial_linear_acceleration;
  ROS_VERIFY(DecLightShowUtilities::getParam(config, "initial_linear_acceleration", initial_linear_acceleration));
  ROS_ASSERT(initial_linear_acceleration.size() == 3);
  tf::Vector3 linear_acceleration(initial_linear_acceleration[0], initial_linear_acceleration[1], initial_linear_acceleration[2]);
  linear_accelerations_.push_back(linear_acceleration);
  simulated_linear_accelerations_ = linear_accelerations_;

  std::vector<float> initial_angular_acceleration;
  ROS_VERIFY(DecLightShowUtilities::getParam(config, "initial_angular_acceleration", initial_angular_acceleration));
  ROS_ASSERT(initial_angular_acceleration.size() == 3);
  tf::Vector3 angular_acceleration(initial_angular_acceleration[0], initial_angular_acceleration[1], initial_angular_acceleration[2]);
  angular_accelerations_.push_back(angular_acceleration);
  simulated_angular_accelerations_ = angular_accelerations_;

  ROS_ASSERT(positions_.size() == linear_velocities_.size());
  ROS_ASSERT(positions_.size() == angular_velocities_.size());
  ROS_ASSERT(positions_.size() == linear_accelerations_.size());
  ROS_ASSERT(positions_.size() == angular_accelerations_.size());
  ROS_ASSERT(positions_.size() == normals_.size());

  DecLightShowUtilities::param(config, "up_down", up_down_, up_down_);

  int profile_type = MathUtilities::eLINEAR;
  DecLightShowUtilities::param(config, "profile_type", profile_type, profile_type);
  ROS_ASSERT_MSG(profile_type >= 0 && profile_type < MathUtilities::eNUM_PROFILES,
                 "Unknown filter type >%i< in >%s<.", profile_type, name_.c_str());
  profile_type_ = static_cast<MathUtilities::Profile>(profile_type);
}

void DecLightShowPlaneCreator::setupSensorMarkers(XmlRpc::XmlRpcValue& config)
{
  ROS_VERIFY(DecLightShowUtilities::getParam(config, "min_distance", min_distance_));
  std::vector<float> min_color;
  ROS_VERIFY(DecLightShowUtilities::getParam(config, "min_color", min_color));
  ROS_ASSERT(min_color.size() == 4);

  ROS_VERIFY(DecLightShowUtilities::getParam(config, "max_distance", max_distance_));
  std::vector<float> max_color;
  ROS_VERIFY(DecLightShowUtilities::getParam(config, "max_color", max_color));
  ROS_ASSERT(max_color.size() == 4);

  ROS_ASSERT(min_distance_ >= 0.0);
  ROS_ASSERT(max_distance_ > min_distance_);
  distance_range_ = max_distance_ - min_distance_;

  std::vector<float> distances;
  distances.push_back(min_distance_);
  distances.push_back(max_distance_);
  std::vector<std::vector<float> > colors;
  colors.push_back(min_color);
  colors.push_back(max_color);
  ROS_ASSERT(distances.size() == colors.size());

  visualization_msgs::Marker marker;
  marker.header.frame_id.assign("/BASE");
  marker.ns = "virtual_cube";
  marker.header.stamp = ros::Time::now();
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration(1.0);
  ROS_ASSERT(!positions_.empty());
  convert(positions_[0], marker.pose.position);
  ROS_ASSERT(!normals_.empty());
  marker.pose.orientation = getOrientation(normals_[0]);

  virtual_sensors_.markers.clear();
  for (unsigned int i = 0; i < distances.size(); ++i)
  {
    marker.id = (int)i;
    marker.scale.x = 10.0f;
    marker.scale.y = 10.0f;
    marker.scale.z = distances[i];
    marker.color.r = colors[i][0];
    marker.color.g = colors[i][1];
    marker.color.b = colors[i][2];
    marker.color.a = colors[i][3];
    virtual_sensors_.markers.push_back(marker);
  }

  marker.ns = "virtual_space";
  marker.header.stamp = ros::Time::now();
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  tf::Vector3 scale = (max_space_ - min_space_).absolute();
  marker.scale.x = scale.getX();
  marker.scale.y = scale.getY();
  marker.scale.z = scale.getZ();
  tf::Vector3 center = (max_space_ + min_space_) / 2.0f;
  convert(center, marker.pose.position);
  convert(tf::Quaternion::getIdentity(), marker.pose.orientation);
  marker.id = 0;
  std::vector<float> virtual_cube_color;
  ROS_VERIFY(DecLightShowUtilities::getParam(config, "virtual_cube_color", virtual_cube_color));
  ROS_ASSERT(virtual_cube_color.size() == 4);
  marker.color.r = virtual_cube_color[0];
  marker.color.g = virtual_cube_color[1];
  marker.color.b = virtual_cube_color[2];
  marker.color.a = virtual_cube_color[3];
  virtual_sensors_.markers.push_back(marker);
}

}
