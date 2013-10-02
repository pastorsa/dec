/*
 * dec_sensor_rise_processor.cpp
 *
 *  Created on: Jul 14, 2013
 *      Author: pastor
 */

#include <dec_utilities/assert.h>
#include <dec_light_show_manager/dec_light_show_utilities.h>

#include <dec_light_shows/dec_sensor_windowing.h>

PLUGINLIB_DECLARE_CLASS(dec_light_shows, DecSensorWindowing, dec_light_shows::DecSensorWindowing,
                        dec_light_show_manager::DecLightShow)

using namespace dec_light_show_manager;

namespace dec_light_shows
{

bool DecSensorWindowing::initialize(XmlRpc::XmlRpcValue& config)
{
  unsigned int up_window_size = 0;
  ROS_VERIFY(DecLightShowUtilities::getParam(config, "up_window_size", up_window_size));
  ROS_ASSERT(up_window_size > 0);
  up_fraction_ = 1.0f/static_cast<float>(up_window_size);

  unsigned int down_window_size = 0;
  ROS_VERIFY(DecLightShowUtilities::getParam(config, "down_window_size", down_window_size));
  ROS_ASSERT(down_window_size > 0);
  down_fraction_ = 1.0f/static_cast<float>(down_window_size);

  return true;
}

bool DecSensorWindowing::start()
{
  // once in the beginning
  for (int i = 0; i < (int)data_->sensor_levels_.size(); ++i)
  {
    data_->sensor_levels_[i] = 0.0f;
  }
  return true;
}

bool DecSensorWindowing::update()
{
  for (unsigned int i = 0; i < data_->sensor_values_.size(); ++i)
  {
    ROS_ASSERT(data_->sensor_values_[i] != 0 || data_->sensor_values_[i] != 1);
    if (data_->sensor_values_[i] == DecData::SENSOR_HIGH)
    {
      data_->sensor_levels_[i] += up_fraction_;
    }
    else
    {
      data_->sensor_levels_[i] -= down_fraction_;
    }

    if (data_->sensor_levels_[i] > 1.0f)
      data_->sensor_levels_[i] = 1.0f;
    if (data_->sensor_levels_[i] < 0.0f)
      data_->sensor_levels_[i] = 0.0f;
  }
  return true;
}

bool DecSensorWindowing::stop()
{

  return true;
}

}
