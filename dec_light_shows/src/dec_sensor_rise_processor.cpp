/*
 * dec_sensor_rise_processor.cpp
 *
 *  Created on: Jul 14, 2013
 *      Author: pastor
 */

#include <dec_utilities/assert.h>
#include <dec_light_show_manager/dec_light_show_utilities.h>

#include <dec_light_shows/dec_sensor_rise_processor.h>

PLUGINLIB_DECLARE_CLASS(dec_light_shows, DecSensorRiseProcessor, dec_light_shows::DecSensorRiseProcessor,
                        dec_light_show_manager::DecLightShow)

using namespace dec_light_show_manager;

namespace dec_light_shows
{

bool DecSensorRiseProcessor::initialize(XmlRpc::XmlRpcValue& config)
{
  float rise_timeout_in_seconds = 0.0;
  ROS_VERIFY(DecLightShowUtilities::getParam(config, "rise_timeout_in_seconds", rise_timeout_in_seconds));
  ROS_ASSERT_MSG(rise_timeout_in_seconds > data_->control_dt_, "Rise timeout in seconds >%.2f< must be greater than >%.2f< seconds.",
                 rise_timeout_in_seconds, data_->control_dt_);
  timeout_counts_ = static_cast<unsigned int>(rise_timeout_in_seconds / data_->control_dt_);
  ROS_ASSERT(timeout_counts_ > 0);
  timeout_counts_++;

  float rise_delay_in_seconds = 0.0;
  ROS_VERIFY(DecLightShowUtilities::getParam(config, "rise_delay_in_seconds", rise_delay_in_seconds));
  ROS_ASSERT_MSG(rise_delay_in_seconds > data_->control_dt_, "Rise delay in seconds >%.2f< must be greater than >%.2f< seconds.",
                 rise_delay_in_seconds, data_->control_dt_);
  window_size_ = static_cast<unsigned int>(rise_delay_in_seconds / data_->control_dt_);
  ROS_ASSERT(window_size_ > 0);
  window_size_++;


  int rise_type = 0;
  ROS_VERIFY(DecLightShowUtilities::getParam(config, "rise_type", rise_type));
  ROS_ASSERT(rise_type >= LINEAR && rise_type < NUM_RISE_TYPES);
  rise_type_ = static_cast<RiseType>(rise_type);

  timeout_counter_.resize(data_->total_num_sensors_, window_size_ - 1);
  indices_.resize(data_->total_num_sensors_, window_size_ - 1);
  values_.resize(window_size_, 0.0);

  switch (rise_type)
  {
    case LINEAR:
    {
      values_[0] = 0.0;
      float rise = 1.0f/static_cast<float>(window_size_ - 1);
      for (unsigned int i = 1; i < window_size_; ++i)
      {
        values_[i] = values_[i - 1] + rise;
      }
      break;
    }
    default:
    {
      ROS_ERROR("%s : Unknown rise type >%i<.", name_.c_str(), (int)rise_type_);
      return false;
    }
  }
  values_[window_size_ - 1] = 1.0;

  return true;
}

bool DecSensorRiseProcessor::start()
{
  // once in the beginning
  for (int i = 0; i < (int)data_->sensor_values_.size(); ++i)
  {
    data_->sensor_levels_[i] = 0.0f;
  }

  for (int i = 0; i < (int)data_->sensor_values_.size(); ++i)
  {
    indices_[i] = 0;
    timeout_counter_[i] = 0;
  }
  return true;
}

bool DecSensorRiseProcessor::update()
{
  for (unsigned int i = 0; i < data_->sensor_values_.size(); ++i)
  {
    ROS_ASSERT(data_->sensor_values_[i] != 0 || data_->sensor_values_[i] != 1);
    if (data_->sensor_values_[i] == DecData::SENSOR_HIGH)
    {
      // assign level
      if (timeout_counter_[i] < timeout_counts_)
      {
        data_->sensor_levels_[i] = values_[indices_[i]];
        // increment indices
        if(indices_[i] < window_size_ - 1)
        {
          indices_[i]++;
        }
      }
      else
      {
        // decrement indices
        if(indices_[i] > 0)
        {
          indices_[i]--;
        }
        data_->sensor_levels_[i] = values_[indices_[i]];
      }

      timeout_counter_[i]++;
    }
    else
    {
      timeout_counter_[i] = 0;
      indices_[i] = 0;
    }
  }
  return true;
}

bool DecSensorRiseProcessor::stop()
{

  return true;
}

}
