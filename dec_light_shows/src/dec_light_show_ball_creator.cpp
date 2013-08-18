/*
 * dec_light_show_ball_creator.cpp
 *
 *  Created on: Jul 14, 2013
 *      Author: pastor
 */

#include <dec_utilities/assert.h>
#include <conversions/tf_to_ros.h>
#include <conversions/ros_to_tf.h>
#include <dec_light_show_manager/dec_light_show_utilities.h>

#include <dec_msgs/LightShow.h>
#include <dec_light_shows/dec_light_show_ball_creator.h>

PLUGINLIB_DECLARE_CLASS(dec_light_shows, DecLightShowBallCreator, dec_light_shows::DecLightShowBallCreator,
                        dec_light_show_manager::DecLightShow)

using namespace dec_light_show_manager;
using namespace conversions;

namespace dec_light_shows
{

DecLightShowBallCreator::DecLightShowBallCreator() :
    node_handle_("/DecLightShowManager"),
    visualization_rate_(0), visualization_counter_(0), integrate_(true),
    min_expansion_radius_(0.0), max_expansion_radius_(0.0), expansion_speed_(0.0), current_raduis_(0.0),
    min_distance_(0.0), max_distance_(0.0), profile_type_(MathUtilities::eLINEAR)
{
  const int PUBLISHER_BUFFER_SIZE = 10;
  rviz_pub_ = node_handle_.advertise<visualization_msgs::MarkerArray>("visualization_marker", PUBLISHER_BUFFER_SIZE, true);

}

bool DecLightShowBallCreator::initialize(XmlRpc::XmlRpcValue& config)
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

  ROS_ASSERT(positions_.size() == 1);
  for (unsigned int i = 0; i < positions_.size(); ++i)
  {
    block_node_distances_.resize(block_light_node_positions_.size(), 0.0);
    for (unsigned int j = 0; j < block_light_node_positions_.size(); ++j)
    {
      block_node_distances_[j] = (block_light_node_positions_[j] - positions_[i]).length();
      if(block_node_distances_[j] < 0.0)
       block_node_distances_[j] = 0.0;
    }
    block_beam_distances_.resize(block_light_beam_positions_.size(), 0.0);
    for (unsigned int j = 0; j < block_light_beam_positions_.size(); ++j)
    {
      block_beam_distances_[j] = (block_light_beam_positions_[j] - positions_[i]).length();
      if(block_beam_distances_[j] < 0.0)
        block_beam_distances_[j] = 0.0;
    }
    pixel_beam_distances_.resize(pixel_light_beam_positions_.size(), 0.0);
    for (unsigned int j = 0; j < pixel_light_beam_positions_.size(); ++j)
    {
      pixel_beam_distances_[j] = (pixel_light_beam_positions_[j] - positions_[i]).length();
      if(pixel_beam_distances_[j] < 0.0)
        pixel_beam_distances_[j] = 0.0;
    }
  }

  return true;
}

bool DecLightShowBallCreator::start()
{
  visualization_counter_ = 0;
  return true;
}

bool DecLightShowBallCreator::update()
{
  if (integrate_)
  {
    integrate();
    computeIntegratedDistance();
  }
  else
  {
    expand();
    computeExpandedDistance();
  }

  if(data_->visualization_mode_)
  {
    publish();
  }
  return true;
}

bool DecLightShowBallCreator::stop()
{

  return true;
}

void DecLightShowBallCreator::computeExpandedDistance()
{
  for (unsigned int i = 0; i < positions_.size(); ++i)
  {
    for (unsigned int j = 0; j < block_light_node_positions_.size(); ++j)
    {
      if (block_node_distances_[j] < current_raduis_)
      {
        if (data_->node_led_levels_(j) <= DecData::BASE_LIGHT_LEVEL)
        {
          float scaled_distance = DecData::MIN_LIGHT_LEVEL + (block_node_distances_[j] / current_raduis_) * DecData::BASE_LIGHT_LEVEL;
          data_->node_led_levels_(j) -= MathUtilities::ramp(DecData::MIN_LIGHT_LEVEL, DecData::MIN_LIGHT_LEVEL + DecData::BASE_LIGHT_LEVEL, scaled_distance, profile_type_);
        }
      }
      if (data_->node_led_levels_(j) < DecData::MIN_LIGHT_LEVEL)
      {
        data_->node_led_levels_(j) = DecData::MIN_LIGHT_LEVEL;
      }
    }
    for (unsigned int j = 0; j < block_light_beam_positions_.size(); ++j)
    {

      if (block_beam_distances_[j] < current_raduis_)
      {
        if (data_->block_beam_led_levels_(j) <= DecData::BASE_LIGHT_LEVEL)
        {
          float scaled_distance = DecData::MIN_LIGHT_LEVEL + (block_beam_distances_[j] / current_raduis_) * DecData::BASE_LIGHT_LEVEL;
          data_->block_beam_led_levels_(j) -= MathUtilities::ramp(DecData::MIN_LIGHT_LEVEL, DecData::MIN_LIGHT_LEVEL + DecData::BASE_LIGHT_LEVEL, scaled_distance, profile_type_);
        }
      }
      if (data_->block_beam_led_levels_(j) < DecData::MIN_LIGHT_LEVEL)
      {
        data_->block_beam_led_levels_(j) = DecData::MIN_LIGHT_LEVEL;
      }
    }
    for (unsigned int j = 0; j < pixel_light_beam_positions_.size(); ++j)
    {

      if (pixel_beam_distances_[j] < current_raduis_)
      {
        if (data_->pixel_beam_led_levels_(j) <= DecData::BASE_LIGHT_LEVEL)
        {
          float scaled_distance = DecData::MIN_LIGHT_LEVEL + (pixel_beam_distances_[j] / current_raduis_) * DecData::BASE_LIGHT_LEVEL;
          data_->pixel_beam_led_levels_(j) -= MathUtilities::ramp(DecData::MIN_LIGHT_LEVEL, DecData::MIN_LIGHT_LEVEL + DecData::BASE_LIGHT_LEVEL, scaled_distance, profile_type_);
        }
      }
      if (data_->pixel_beam_led_levels_(j) < DecData::MIN_LIGHT_LEVEL)
      {
        data_->pixel_beam_led_levels_(j) = DecData::MIN_LIGHT_LEVEL;
      }
    }
  }
}

void DecLightShowBallCreator::computeIntegratedDistance()
{
  for (unsigned int i = 0; i < positions_.size(); ++i)
  {
    // =====================================================
    // Block Nodes
    // =====================================================
    for (unsigned int j = 0; j < block_light_node_positions_.size(); ++j)
    {
      float distance = (block_light_node_positions_[j] - positions_[i]).length();
      if (distance < max_distance_)
      {
        data_->node_led_levels_(j) -= static_cast<float>((max_distance_ - distance) / max_distance_);
        if (data_->node_led_levels_(j) < DecData::MIN_LIGHT_LEVEL)
        {
          data_->node_led_levels_(j) = DecData::MIN_LIGHT_LEVEL;
        }
      }
    }

    // =====================================================
    // Block Beams
    // =====================================================
    for (unsigned int j = 0; j < block_light_beam_positions_.size(); ++j)
    {
      float distance = (block_light_beam_positions_[j] - positions_[i]).length();
      if (distance < max_distance_)
      {
        data_->block_beam_led_levels_(j) -= static_cast<float>((max_distance_ - distance) / max_distance_);
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
      float distance = (pixel_light_beam_positions_[j] - positions_[i]).length();
      if (distance < max_distance_)
      {
        data_->pixel_beam_led_levels_(j) -= static_cast<float>((max_distance_ - distance) / max_distance_);
      }
      if (data_->pixel_beam_led_levels_(j) < DecData::MIN_LIGHT_LEVEL)
      {
        data_->pixel_beam_led_levels_(j) = DecData::MIN_LIGHT_LEVEL;
      }
    }
  }
}

void DecLightShowBallCreator::expand()
{
  for (unsigned int i = 0; i < positions_.size(); ++i)
  {
    current_raduis_ += expansion_speed_ * data_->control_dt_;
    if (current_raduis_ > max_expansion_radius_)
    {
      expansion_speed_ *= -1.0f;
    }
    else if (current_raduis_ < min_expansion_radius_)
    {
      expansion_speed_ *= -1.0f;
    }
  }
}

void DecLightShowBallCreator::integrate()
{
  for (unsigned int i = 0; i < positions_.size(); ++i)
  {
    positions_[i] += velocities_[i] * data_->control_dt_;
    velocities_[i] += simulated_accelerations_[i] * data_->control_dt_;

    if (positions_[i].getX() > max_space_.getX()
        || positions_[i].getX() < min_space_.getX())
    {
      velocities_[i].setX(velocities_[i].getX() * -1.0f);
      accelerations_[i].setX(accelerations_[i].getX() * -1.0f);
    }
    if (positions_[i].getY() > max_space_.getY()
        || positions_[i].getY() < min_space_.getY())
    {
      velocities_[i].setY(velocities_[i].getY() * -1.0f);
      accelerations_[i].setY(accelerations_[i].getY() * -1.0f);
    }
    if (positions_[i].getZ() > max_space_.getZ()
        || positions_[i].getZ() < min_space_.getZ())
    {
      velocities_[i].setZ(velocities_[i].getZ() * -1.0f);
      accelerations_[i].setZ(accelerations_[i].getZ() * -1.0f);
    }

    simulated_accelerations_[i] = accelerations_[i] * sin(data_->ros_time_sec_);
  }

  // ROS_WARN("Position: %.2f %.2f %.2f", positions_[0].getX(), positions_[0].getY(), positions_[0].getZ());
  // ROS_WARN("Velocity: %.2f %.2f %.2f", velocities_[0].getX(), velocities_[0].getY(), velocities_[0].getZ());
  // ROS_WARN("Acceleration: %.2f %.2f %.2f", simulated_accelerations_[0].getX(), simulated_accelerations_[0].getY(), simulated_accelerations_[0].getZ());

  convert(positions_[0], virtual_sensors_.markers[0].pose.position);
  convert(positions_[0], virtual_sensors_.markers[1].pose.position);
}

void DecLightShowBallCreator::publish()
{
  if (!integrate_)
  {
    for (unsigned int i = 0; i < virtual_sensors_.markers.size(); ++i)
    {
      virtual_sensors_.markers[i].scale.x = current_raduis_;
      virtual_sensors_.markers[i].scale.y = current_raduis_;
      virtual_sensors_.markers[i].scale.z = current_raduis_;
    }
  }

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

void DecLightShowBallCreator::setupSpace(XmlRpc::XmlRpcValue& config)
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

  max_space_ /= 2.0f;
  min_space_ /= 2.0f;
  tf::Vector3 scale = (max_space_ - min_space_).absolute() / 2.0f;

  max_space_.setZ(max_space_.getZ() + scale.getZ());
  min_space_.setZ(min_space_.getZ() + scale.getZ());
}

void DecLightShowBallCreator::readParameters(XmlRpc::XmlRpcValue& config)
{
  ROS_VERIFY(DecLightShowUtilities::getParam(config, "visualization_rate", visualization_rate_));

  std::vector<float> initial_position;
  ROS_VERIFY(DecLightShowUtilities::getParam(config, "initial_position", initial_position));
  ROS_ASSERT(initial_position.size() == 3);
  tf::Vector3 position(initial_position[0], initial_position[1], initial_position[2]);
  positions_.push_back(position);

  std::vector<float> initial_velocity;
  ROS_VERIFY(DecLightShowUtilities::getParam(config, "initial_velocity", initial_velocity));
  ROS_ASSERT(initial_velocity.size() == 3);
  tf::Vector3 velocity(initial_velocity[0], initial_velocity[1], initial_velocity[2]);
  velocities_.push_back(velocity);

  std::vector<float> initial_acceleration;
  ROS_VERIFY(DecLightShowUtilities::getParam(config, "initial_acceleration", initial_acceleration));
  ROS_ASSERT(initial_acceleration.size() == 3);
  tf::Vector3 acceleration(initial_acceleration[0], initial_acceleration[1], initial_acceleration[2]);
  accelerations_.push_back(acceleration);
  simulated_accelerations_ = accelerations_;

  integrate_ = true;
  if (velocity[0] < 0.001 && velocity[1] < 0.001 && velocity[2] < 0.001 && acceleration[0] < 0.001 && acceleration[1] < 0.001 && acceleration[2] < 0.001)
  {
    integrate_ = false;
  }

  ROS_VERIFY(DecLightShowUtilities::getParam(config, "min_expansion_radius", min_expansion_radius_));
  ROS_VERIFY(DecLightShowUtilities::getParam(config, "max_expansion_radius", max_expansion_radius_));
  ROS_VERIFY(DecLightShowUtilities::getParam(config, "expansion_speed", expansion_speed_));
  current_raduis_ = min_expansion_radius_;

  ROS_ASSERT(positions_.size() == velocities_.size());
  ROS_ASSERT(positions_.size() == accelerations_.size());

  int profile_type = MathUtilities::eLINEAR;
  DecLightShowUtilities::param(config, "profile_type", profile_type, profile_type);
  ROS_ASSERT_MSG(profile_type >= 0 && profile_type < MathUtilities::eNUM_PROFILES,
                 "Unknown filter type >%i< in >%s<.", profile_type, name_.c_str());
  profile_type_ = static_cast<MathUtilities::Profile>(profile_type);
}

void DecLightShowBallCreator::setupSensorMarkers(XmlRpc::XmlRpcValue& config)
{
  ROS_VERIFY(DecLightShowUtilities::getParam(config, "min_distance", min_distance_));
  std::vector<float> min_color;
  ROS_VERIFY(DecLightShowUtilities::getParam(config, "min_color", min_color));
  ROS_ASSERT(min_color.size() == 4);

  ROS_VERIFY(DecLightShowUtilities::getParam(config, "max_distance", max_distance_));
  std::vector<float> max_color;
  ROS_VERIFY(DecLightShowUtilities::getParam(config, "max_color", max_color));
  ROS_ASSERT(max_color.size() == 4);

  ROS_ASSERT(max_distance_ > min_distance_);

  std::vector<float> distances;
  distances.push_back(min_distance_);
  distances.push_back(max_distance_);
  std::vector<std::vector<float> > colors;
  colors.push_back(min_color);
  colors.push_back(max_color);
  ROS_ASSERT(distances.size() == colors.size());

  visualization_msgs::Marker marker;
  marker.header.frame_id.assign("/BASE");
  marker.ns = "virtual_ball";
  marker.header.stamp = ros::Time::now();
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration(1.0);
  ROS_ASSERT(!positions_.empty());
  convert(positions_[0], marker.pose.position);
  convert(tf::Quaternion::getIdentity(), marker.pose.orientation);

  virtual_sensors_.markers.clear();
  for (unsigned int i = 0; i < distances.size(); ++i)
  {
    marker.id = (int)i;
    marker.scale.x = distances[i];
    marker.scale.y = distances[i];
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
