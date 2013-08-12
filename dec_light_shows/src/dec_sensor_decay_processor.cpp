/*
 * dec_sensor_decay_processor.cpp
 *
 *  Created on: Jul 14, 2013
 *      Author: pastor
 */

#include <dec_utilities/assert.h>
#include <dec_light_show_manager/dec_light_show_utilities.h>

#include <dec_light_shows/dec_sensor_decay_processor.h>

PLUGINLIB_DECLARE_CLASS(dec_light_shows, DecSensorDecayProcessor, dec_light_shows::DecSensorDecayProcessor,
                        dec_light_show_manager::DecLightShow)

using namespace dec_light_show_manager;

namespace dec_light_shows
{

bool DecSensorDecayProcessor::initialize(XmlRpc::XmlRpcValue& config)
{
  float decay_delay_in_seconds = 0.0;
  ROS_VERIFY(DecLightShowUtilities::getParam(config, "decay_delay_in_seconds", decay_delay_in_seconds));
  ROS_ASSERT_MSG(decay_delay_in_seconds > data_->control_dt_, "Decay delay in seconds >%.2f< must be greater than >%.2f< seconds.",
                 decay_delay_in_seconds, data_->control_dt_);
  window_size_ = static_cast<unsigned int>(decay_delay_in_seconds / data_->control_dt_);
  ROS_ASSERT(window_size_ > 0);
  window_size_++;

  int decay_type = 0;
  ROS_VERIFY(DecLightShowUtilities::getParam(config, "decay_type", decay_type));
  ROS_ASSERT(decay_type >= LINEAR && decay_type < NUM_DECAY_TYPES);
  decay_type_ = static_cast<DecayType>(decay_type);

  indices_.resize(data_->total_num_sensors_, window_size_ - 1);
  values_.resize(window_size_, 0.0);

  switch (decay_type)
  {
    case LINEAR:
    {
      values_[0] = 1.0;
      float decay = 1.0f/static_cast<float>(window_size_ - 1);
      for (unsigned int i = 1; i < window_size_; ++i)
      {
        values_[i] = values_[i - 1] - decay;
      }
      break;
    }
    default:
    {
      ROS_ERROR("%s : Unknown decay type >%i<.", name_.c_str(), (int)decay_type_);
      return false;
    }
  }
  values_[window_size_ - 1] = 0.0;

  return true;
}

bool DecSensorDecayProcessor::start()
{
  for (int i = 0; i < (int)data_->sensor_values_.size(); ++i)
  {
    indices_[i] = window_size_ - 1;
  }
  return true;
}

bool DecSensorDecayProcessor::update()
{

  for (unsigned int i = 0; i < data_->sensor_values_.size(); ++i)
  {
    ROS_ASSERT(data_->sensor_values_[i] != 0 || data_->sensor_values_[i] != 1);
    if (data_->sensor_values_[i] == DecData::SENSOR_LOW)
    {
      // assign level
      data_->sensor_levels_[i] = values_[indices_[i]];
      // increment indices
      if (indices_[i] < window_size_ - 1)
      {
        indices_[i]++;
      }
    }
    else
    {
      indices_[i] = 0;
    }
  }
  return true;
}

bool DecSensorDecayProcessor::stop()
{

  return true;
}

}
