/*
 * dec_sensor_butterworth_processor.cpp
 *
 *  Created on: Jul 14, 2013
 *      Author: pastor
 */

#include <dec_utilities/assert.h>

#include <dec_light_show_manager/dec_light_show_utilities.h>
#include <dec_light_shows/dec_sensor_butterworth_processor.h>

PLUGINLIB_DECLARE_CLASS(dec_light_shows, DecSensorButterworthProcessor, dec_light_shows::DecSensorButterworthProcessor, dec_light_show_manager::DecLightShow)

using namespace dec_light_show_manager;

namespace dec_light_shows
{

DecSensorButterworthProcessor::DecSensorButterworthProcessor()
{
}

bool DecSensorButterworthProcessor::initialize(XmlRpc::XmlRpcValue& config)
{
  unfiltered_data_.resize(data_->total_num_sensors_, 0.0);
  filtered_data_.resize(data_->total_num_sensors_, 0.0);
  XmlRpc::XmlRpcValue sensor_filter_config = config["sensor_filter"];
  ROS_VERIFY(((filters::MultiChannelFilterBase<float>&)filter_).configure(data_->sensor_values_.size(), sensor_filter_config));

  ROS_INFO("%s : Configured for >%i< Teensys++ 2.0 with:\n   >%i< sensors\n   >%i< light nodes\n   >%i< block light beams\n   >%i< pixel light beams.",
           name_.c_str(), data_->getNumTeensys(), (int)data_->total_num_sensors_, (int)data_->total_num_node_leds_, (int)data_->total_num_block_beam_leds_, (int)data_->total_num_pixel_beam_leds_);
  return true;
}

bool DecSensorButterworthProcessor::start()
{
  for (int i = 0; i < (int)data_->sensor_values_.size(); ++i)
  {
    unfiltered_data_[i] = 0.0;
  }
  // warm start
  for (unsigned n = 0; n < 100; ++n)
  {
    ROS_VERIFY(filter_.update(unfiltered_data_, filtered_data_));
  }

  // data_->prev_sensor_values_ = data_->sensor_values_;
  return true;
}

bool DecSensorButterworthProcessor::update()
{
  for (int i = 0; i < (int)data_->sensor_values_.size(); ++i)
  {
    unfiltered_data_[i] = static_cast<float>(data_->sensor_values_(i));
  }
  ROS_VERIFY(filter_.update(unfiltered_data_, filtered_data_));
  for (int i = 0; i < (int)data_->sensor_values_.size(); ++i)
  {
    if (filtered_data_[i] < 0.0f)
    {
      filtered_data_[i] = 0.0f;
    }
    if (filtered_data_[i] > 1.0f)
    {
      filtered_data_[i] = 1.0f;
    }
    data_->sensor_levels_(i) = filtered_data_[i];
  }
  return true;
}

bool DecSensorButterworthProcessor::stop()
{

  return true;
}

}
