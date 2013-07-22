/*
 * dec_light_processor.cpp
 *
 *  Created on: Jul 14, 2013
 *      Author: pastor
 */

#include <dec_utilities/assert.h>
#include <dec_light_show_manager/dec_light_show_utilities.h>

#include <dec_light_shows/dec_light_processor.h>

PLUGINLIB_DECLARE_CLASS(dec_light_shows, DecLightProcessor, dec_light_shows::DecLightProcessor, dec_light_show_manager::DecLightShow)

using namespace dec_light_show_manager;

namespace dec_light_shows
{

DecLightProcessor::DecLightProcessor()
  : filter_ring_index_(0),
    filter_size_(0),
    max_distance_(0.0),
    wave_travel_speed_(0.0),
    wave_length_(0.0),
    wave_activation_size_(0),
    half_wave_activation_size_(0)
{
}

bool DecLightProcessor::initialize(XmlRpc::XmlRpcValue& config)
{
  Eigen::Vector3f max_distance = Eigen::Vector3f::Zero();
  max_distance(0) = data_->node_led_distances_to_sensor_.maxCoeff();
  if (data_->total_num_block_beam_leds_ > 0)
  {
    max_distance(1) = data_->block_beam_led_distances_to_sensor_.maxCoeff();
  }
  if (data_->total_num_pixel_beam_leds_ > 0)
  {
    max_distance(2) = data_->pixel_beam_led_distances_to_sensor_.maxCoeff();
  }
  max_distance_ = static_cast<float>(max_distance.maxCoeff());
  ROS_ASSERT(max_distance_ > 0.0);

  normalized_node_led_distances_to_sensor_ = data_->node_led_distances_to_sensor_;
  normalized_node_led_distances_to_sensor_.array() /= max_distance_;
  normalized_node_led_distances_to_sensor_.array() = normalized_node_led_distances_to_sensor_.array() - 1.0;
  normalized_node_led_distances_to_sensor_.array() /= -1.0;

  if (data_->total_num_block_beam_leds_ > 0)
  {
    normalized_block_beam_led_distances_to_sensor_ = data_->block_beam_led_distances_to_sensor_;
    normalized_block_beam_led_distances_to_sensor_.array() /= max_distance_;
    normalized_block_beam_led_distances_to_sensor_.array() = normalized_block_beam_led_distances_to_sensor_.array() - 1.0;
    normalized_block_beam_led_distances_to_sensor_.array() /= -1.0;
  }

  if (data_->total_num_pixel_beam_leds_ > 0)
  {
    normalized_pixel_beam_led_distances_to_sensor_ = data_->pixel_beam_led_distances_to_sensor_;
    normalized_pixel_beam_led_distances_to_sensor_.array() /= max_distance_;
    normalized_pixel_beam_led_distances_to_sensor_.array() = normalized_pixel_beam_led_distances_to_sensor_.array() - 1.0;
    normalized_pixel_beam_led_distances_to_sensor_.array() /= -1.0;
  }

  ROS_ASSERT(normalized_node_led_distances_to_sensor_.maxCoeff() <= 1.0);
  ROS_ASSERT(normalized_node_led_distances_to_sensor_.minCoeff() >= 0.0);

  ROS_INFO("%s : Control frequency is >%.1f< Hz and delta_t is >%.2f< seconds.", name_.c_str(), data_->control_frequency_, data_->control_dt_);
  ROS_INFO("%s : Maximum distance is >%.2f< meters.", name_.c_str(), max_distance_);

  ROS_VERIFY(DecLightShowUtilities::getParam(config, "wave_travel_speed", wave_travel_speed_));
  ROS_ASSERT(wave_travel_speed_ > 0.0);
  filter_size_ = static_cast<unsigned int>((max_distance_ / wave_travel_speed_) * data_->control_frequency_);
  ROS_INFO("%s : Wave travel speed is >%.2f< meters/second and filter size is >%i<.", name_.c_str(), wave_travel_speed_, (int)filter_size_);

  ROS_VERIFY(DecLightShowUtilities::getParam(config, "wave_length", wave_length_));
  ROS_ASSERT(wave_length_ > 0.0);
  wave_activation_size_ = static_cast<unsigned int>((static_cast<float>(filter_size_) * wave_length_) / max_distance_);
  if (wave_activation_size_ % 2 == 0)
  {
    wave_activation_size_++;
  }
  half_wave_activation_size_ = wave_activation_size_ / 2;
  ROS_INFO("%s : Wave length is >%.2f< meters and wave activation size is >%i<.", name_.c_str(), wave_length_, (int)wave_activation_size_);

  block_node_buffer_ = Eigen::MatrixXf::Zero(data_->total_num_node_leds_, filter_size_);
  if (data_->total_num_block_beam_leds_ > 0)
  {
    block_beam_buffer_ = Eigen::MatrixXf::Zero(data_->total_num_block_beam_leds_, filter_size_);
  }
  if (data_->total_num_pixel_beam_leds_ > 0)
  {
    pixel_beam_buffer_ = Eigen::MatrixXf::Zero(data_->total_num_pixel_beam_leds_, filter_size_);
  }

  filter_ = Eigen::MatrixXf::Zero(filter_size_, filter_size_);
  for (unsigned int i = 0; i < filter_size_; ++i)
  {
    for (unsigned int j = 0; j < filter_size_; ++j)
    {
      float value = 0.0;
      unsigned int aux = abs(i - j);
      if (aux < half_wave_activation_size_)
      {
        value = static_cast<float>(half_wave_activation_size_ - aux) / static_cast<float>(half_wave_activation_size_);
      }
      filter_(i,j) = value;
    }
  }
  // ROS_INFO_STREAM("filter\n" << filter_);
  return true;
}

bool DecLightProcessor::start()
{
  block_node_buffer_.setZero(data_->total_num_node_leds_, filter_size_);
  if (data_->total_num_block_beam_leds_ > 0)
  {
    block_beam_buffer_.setZero(data_->total_num_block_beam_leds_, filter_size_);
  }
  if (data_->total_num_pixel_beam_leds_ > 0)
  {
    pixel_beam_buffer_.setZero(data_->total_num_pixel_beam_leds_, filter_size_);
  }
  return true;
}

bool DecLightProcessor::update()
{
  // take sensor_levels_ and convert them into node_led_level_ and beam_led_level_

  // =====================================================
  // Block Nodes
  // =====================================================

  data_->node_led_levels_.setZero();
  for (unsigned int i = 0; i < data_->sensor_levels_.size(); ++i)
  {
    for (unsigned int j = 0; j < data_->node_led_levels_.size(); ++j)
    {
      unsigned int current_filter_index = filter_ring_index_;
      for (unsigned int n = 0; n < filter_size_; ++n)
      {
        block_node_buffer_(j, current_filter_index) += data_->sensor_levels_(i) * getFilter(static_cast<float>(normalized_node_led_distances_to_sensor_(j, i)), n);
        current_filter_index++;
        if (current_filter_index >= filter_size_)
        {
          current_filter_index = 0;
        }
      }
    }
  }
  // set data
  data_->node_led_levels_ = block_node_buffer_.col(filter_ring_index_);
  // truncate
  for (unsigned int i = 0; i < data_->node_led_levels_.size(); ++i)
  {
    if(data_->node_led_levels_(i) > 1.0f)
    {
      data_->node_led_levels_(i) = 1.0f;
    }
  }
  // zero data
  for (unsigned int i = 0; i < data_->node_led_levels_.size(); ++i)
  {
    block_node_buffer_(i, filter_ring_index_) = 0.0;
  }

  // =====================================================
  // Block Beams
  // =====================================================

  if (data_->total_num_block_beam_leds_ > 0)
  {
    data_->block_beam_led_levels_.setZero();
    for (unsigned int i = 0; i < data_->sensor_levels_.size(); ++i)
    {
      for (unsigned int j = 0; j < data_->block_beam_led_levels_.size(); ++j)
      {
        unsigned int current_filter_index = filter_ring_index_;
        for (unsigned int n = 0; n < filter_size_; ++n)
        {
          block_beam_buffer_(j, current_filter_index) += data_->sensor_levels_(i) * getFilter(static_cast<float>(normalized_block_beam_led_distances_to_sensor_(j, i)), n);
          current_filter_index++;
          if (current_filter_index >= filter_size_)
          {
            current_filter_index = 0;
          }
        }
      }
    }
    // set data
    data_->block_beam_led_levels_ = block_beam_buffer_.col(filter_ring_index_);
    // truncate
    for (unsigned int i = 0; i < data_->block_beam_led_levels_.size(); ++i)
    {
      if(data_->block_beam_led_levels_(i) > 1.0f)
      {
        data_->block_beam_led_levels_(i) = 1.0f;
      }
    }
    // zero data
    for (unsigned int i = 0; i < data_->block_beam_led_levels_.size(); ++i)
    {
      block_beam_buffer_(i, filter_ring_index_) = 0.0;
    }
  }

  // =====================================================
  // Pixel Beams
  // =====================================================

  if (data_->total_num_pixel_beam_leds_ > 0)
  {
    data_->pixel_beam_led_levels_.setZero();
    for (unsigned int i = 0; i < data_->sensor_levels_.size(); ++i)
    {
      for (unsigned int j = 0; j < data_->pixel_beam_led_levels_.size(); ++j)
      {
        unsigned int current_filter_index = filter_ring_index_;
        for (unsigned int n = 0; n < filter_size_; ++n)
        {
          pixel_beam_buffer_(j, current_filter_index) += data_->sensor_levels_(i) * getFilter(static_cast<float>(normalized_pixel_beam_led_distances_to_sensor_(j, i)), n);
          current_filter_index++;
          if (current_filter_index >= filter_size_)
          {
            current_filter_index = 0;
          }
        }
      }
    }
    // set data
    data_->pixel_beam_led_levels_ = pixel_beam_buffer_.col(filter_ring_index_);
    // truncate
    for (unsigned int i = 0; i < data_->pixel_beam_led_levels_.size(); ++i)
    {
      if(data_->pixel_beam_led_levels_(i) > 1.0f)
      {
        data_->pixel_beam_led_levels_(i) = 1.0f;
      }
    }
    // zero data
    for (unsigned int i = 0; i < data_->pixel_beam_led_levels_.size(); ++i)
    {
      pixel_beam_buffer_(i, filter_ring_index_) = 0.0;
    }
  }

  // =====================================================

  filter_ring_index_++;
  if (filter_ring_index_ >= filter_size_)
  {
    filter_ring_index_ = 0;
  }
  return true;
}

bool DecLightProcessor::stop()
{

  return true;
}

}
