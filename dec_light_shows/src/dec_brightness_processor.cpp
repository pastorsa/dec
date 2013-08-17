/*
 * dec_brightness_processor.cpp
 *
 *  Created on: Jul 14, 2013
 *      Author: pastor
 */

#include <dec_utilities/assert.h>
#include <dec_light_show_manager/dec_light_show_utilities.h>

#include <dec_light_shows/dec_brightness_processor.h>

PLUGINLIB_DECLARE_CLASS(dec_light_shows, DecBrightnessProcessor, dec_light_shows::DecBrightnessProcessor,
                        dec_light_show_manager::DecLightShow)

using namespace dec_light_show_manager;

namespace dec_light_shows
{

bool DecBrightnessProcessor::initialize(XmlRpc::XmlRpcValue& config)
{
  ROS_VERIFY(DecLightShowUtilities::getParam(config, "min_brightness", min_brightness_));
  max_brightness_ = data_->getMaxBrightness();
  ROS_ASSERT(min_brightness_ < max_brightness_);
  ROS_VERIFY(DecLightShowUtilities::getParam(config, "initial_node_brightness", node_brightness_));
  ROS_ASSERT(min_brightness_ <= node_brightness_);
  ROS_ASSERT(max_brightness_ >= node_brightness_);
  ROS_VERIFY(DecLightShowUtilities::getParam(config, "initial_beam_brightness", beam_brightness_));
  ROS_ASSERT(min_brightness_ <= beam_brightness_);
  ROS_ASSERT(max_brightness_ >= beam_brightness_);

  // do this last
  const int SUBSCRIBER_BUFFER_SIZE = 1;
  brightness_sub_ = data_->node_handle_.subscribe(name_ + "/command", SUBSCRIBER_BUFFER_SIZE, &DecBrightnessProcessor::brightnessCB, this);

  return true;
}

bool DecBrightnessProcessor::start()
{
  return update();
}

bool DecBrightnessProcessor::update()
{
  boost::mutex::scoped_try_lock trylock(mutex_);
  for (int i = 0; i < (int)data_->node_led_levels_.size(); ++i)
  {
    data_->node_led_values_(BRIGHTNESS_OFFSET, i) = static_cast<led_channel_t>(node_brightness_);
  }

  if (data_->total_num_block_beam_leds_ > 0)
  {
    for (int i = 0; i < (int)data_->block_beam_led_levels_.size(); ++i)
    {
      data_->block_beam_led_values_(BRIGHTNESS_OFFSET, i) = static_cast<led_channel_t>(beam_brightness_);
    }
  }

  if (data_->total_num_pixel_beam_leds_ > 0)
  {
    for (int i = 0; i < (int)data_->pixel_beam_led_levels_.size(); ++i)
    {
      data_->pixel_beam_led_values_(BRIGHTNESS_OFFSET, i) = static_cast<led_channel_t>(beam_brightness_);
    }
  }

  return true;
}

bool DecBrightnessProcessor::stop()
{

  return true;
}

void DecBrightnessProcessor::brightnessCB(dec_msgs::BrightnessConstPtr brightness)
{

  unsigned int node_brightness = 0;
  if(brightness->node_brightness < (int)min_brightness_)
  {
    node_brightness = static_cast<unsigned int>(min_brightness_);
  }
  else if(brightness->node_brightness > (int)max_brightness_)
  {
    node_brightness = static_cast<unsigned int>(max_brightness_);
  }
  else
  {
    node_brightness = static_cast<unsigned int>(brightness->node_brightness);
  }

  unsigned int beam_brightness = 0;
  if(brightness->beam_brightness < (int)min_brightness_)
  {
    beam_brightness = static_cast<unsigned int>(min_brightness_);
  }
  else if(brightness->beam_brightness > (int)max_brightness_)
  {
    beam_brightness = static_cast<unsigned int>(max_brightness_);
  }
  else
  {
    beam_brightness = static_cast<unsigned int>(brightness->beam_brightness);
  }

  boost::mutex::scoped_lock lock(mutex_);
  node_brightness_ = node_brightness;
  beam_brightness_ = beam_brightness;
}

}
