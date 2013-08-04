/*
 * dec_sensor_processor.cpp
 *
 *  Created on: Jul 14, 2013
 *      Author: pastor
 */

#include <dec_utilities/assert.h>

#include <dec_light_show_manager/dec_light_show_utilities.h>
#include <dec_light_shows/dec_sensor_processor.h>

PLUGINLIB_DECLARE_CLASS(dec_light_shows, DecSensorProcessor, dec_light_shows::DecSensorProcessor, dec_light_show_manager::DecLightShow)

using namespace dec_light_show_manager;

namespace dec_light_shows
{

DecSensorProcessor::DecSensorProcessor()
// : num_cycles_to_load_(0), num_cycles_to_unload_(0)
{
}

bool DecSensorProcessor::initialize(XmlRpc::XmlRpcValue& config)
{
  unfiltered_data_.resize(data_->total_num_sensors_, 0.0);
  filtered_data_.resize(data_->total_num_sensors_, 0.0);
  XmlRpc::XmlRpcValue sensor_filter_config = config["sensor_filter"];
  ROS_VERIFY(((filters::MultiChannelFilterBase<float>&)filter_).configure(data_->sensor_values_.size(), sensor_filter_config));

  // read parameters
  //  ROS_VERIFY(DecLightShowUtilities::getParam(config, "num_cycles_to_load", num_cycles_to_load_));
  //  ROS_ASSERT_MSG(num_cycles_to_load_ > 0, "Number of cycles to load >%i< must be greater then zero.", num_cycles_to_load_);
  //  ROS_VERIFY(DecLightShowUtilities::getParam(config, "num_cycles_to_unload", num_cycles_to_unload_));
  //  ROS_ASSERT_MSG(num_cycles_to_unload_ > 0, "Number of cycles to unload >%i< must be greater then zero.", num_cycles_to_unload_);
  //
  //  for (int i = 0; i < (int)data_->sensor_values_.size(); ++i)
  //  {
  //    boost::shared_ptr<DecCircularBuffer<sensor_channel_t> > load_circular_buffer(new DecCircularBuffer<sensor_channel_t>(num_cycles_to_load_, (sensor_channel_t)0));
  //    load_circular_buffers_.push_back(load_circular_buffer);
  //
  //    boost::shared_ptr<DecCircularBuffer<sensor_channel_t> > unload_circular_buffer(new DecCircularBuffer<sensor_channel_t>(num_cycles_to_unload_, (sensor_channel_t)0));
  //    unload_circular_buffers_.push_back(unload_circular_buffer);
  //  }

  ROS_INFO("%s : Configured for >%i< Teensys++ 2.0 with:\n   >%i< sensors\n   >%i< light nodes\n   >%i< block light beams\n   >%i< pixel light beams.",
           name_.c_str(), data_->getNumTeensys(), (int)data_->total_num_sensors_, (int)data_->total_num_node_leds_, (int)data_->total_num_block_beam_leds_, (int)data_->total_num_pixel_beam_leds_);
  return true;
}

bool DecSensorProcessor::start()
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

bool DecSensorProcessor::update()
{
  //  for (int i = 0; i < (int)data_->sensor_values_.size(); ++i)
  //  {
  //    data_->sensor_levels_(i) = static_cast<float>(data_->sensor_values_(i));
  //  }
  //  return true;

  for (int i = 0; i < (int)data_->sensor_values_.size(); ++i)
  {
   data_->sensor_levels_(i) = 0.0;
  }
  return true;

  for (int i = 0; i < (int)data_->sensor_values_.size(); ++i)
  {
    unfiltered_data_[i] = static_cast<float>(data_->sensor_values_(i));
  }
  ROS_VERIFY(filter_.update(unfiltered_data_, filtered_data_));
  for (int i = 0; i < (int)data_->sensor_values_.size(); ++i)
  {
    if (filtered_data_[i] < 0.0f)
      filtered_data_[i] = 0.0f;
    if (filtered_data_[i] > 1.0f)
      filtered_data_[i] = 1.0f;
    data_->sensor_levels_(i) = filtered_data_[i];
  }

  return true;

//  // process sensor values into sensor levels
//  for (int i = 0; i < (int)data_->sensor_values_.size(); ++i)
//  {
//    load_circular_buffers_[i]->push_front(data_->sensor_values_(i));
//
//    data_->sensor_levels_(i) = 0.0;
//    for (unsigned int j = 0; j < load_circular_buffers_[i]->size(); ++j)
//    {
//      data_->sensor_levels_(i) += static_cast<float>(load_circular_buffers_[i]->at(j));
//    }
//    data_->sensor_levels_(i) /= static_cast<float>(load_circular_buffers_[i]->size());
//    if(data_->sensor_levels_(i) > 1.0f)
//      data_->sensor_levels_(i) = 1.0f;
//  }
//
//  // ROS_DEBUG_STREAM("sensor levels: " << data_->sensor_levels_.transpose());
//
//  // update previous sensor values
//  data_->prev_sensor_values_ = data_->sensor_values_;
//
//  return true;
}

bool DecSensorProcessor::stop()
{

  return true;
}

}
