/*
 * dec_color_processor.cpp
 *
 *  Created on: Jul 14, 2013
 *      Author: pastor
 */

#include <dec_utilities/assert.h>
#include <dec_light_show_manager/dec_light_show_utilities.h>

#include <dec_light_shows/dec_color_processor.h>

PLUGINLIB_DECLARE_CLASS(dec_light_shows, DecColorProcessor, dec_light_shows::DecColorProcessor, dec_light_show_manager::DecLightShow)

using namespace dec_light_show_manager;

namespace dec_light_shows
{

bool DecColorProcessor::initialize(XmlRpc::XmlRpcValue& config)
{
  ROS_VERIFY(DecLightShowUtilities::getParam(config, "low_level_color", low_level_color_));
  ROS_VERIFY(DecLightShowUtilities::getParam(config, "high_level_color", high_level_color_));
  colors_.resize(4, 0.0);
  color_multiplier_.resize(4, 0.0);
  ROS_ASSERT(low_level_color_.size() == 4);
  ROS_ASSERT(high_level_color_.size() == low_level_color_.size() );
  for (unsigned int i = 0; i < low_level_color_.size(); ++i)
  {
    ROS_ASSERT(!(low_level_color_[i] < 0.0) && !(low_level_color_[i] > 1.0));
    ROS_ASSERT(!(high_level_color_[i] < 0.0) && !(high_level_color_[i] > 1.0));
    color_multiplier_[i] = high_level_color_[i] - low_level_color_[i];
  }
  return true;
}

bool DecColorProcessor::start()
{

  return true;
}

bool DecColorProcessor::update()
{
  // take node_led_levels_ and beam_led_levels_ and process them into node_led_values_ and beam_led_values_
  colors_ = low_level_color_;
  for (unsigned int i = 0; i < data_->node_led_levels_.size(); ++i)
  {
    ROS_ASSERT(!(data_->node_led_levels_[i] < 0.0) && !(data_->node_led_levels_[i] > 1.0));
    for (unsigned int j = 0; j < NUM_COLOR_VALUES; ++j)
    {
      float value = (low_level_color_[j] + (color_multiplier_[j] * data_->node_led_levels_[i])) * 255.0f;
      // ROS_ASSERT_MSG(!(value < 0.0) && !(value > 255.0), "Invalid value >%.2f< computed.", value);
      // ROS_INFO("Node %i color %i value >%.2f< computed.", i, j, value);
      data_->node_led_values_(j, i) = static_cast<led_channel_t>(value);
    }
  }

  if (data_->total_num_block_beam_leds_ > 0)
  {
    for (unsigned int i = 0; i < data_->block_beam_led_levels_.size(); ++i)
    {
      ROS_ASSERT(!(data_->block_beam_led_levels_[i] < 0.0) && !(data_->block_beam_led_levels_[i] > 1.0));
      for (unsigned int j = 0; j < NUM_COLOR_VALUES; ++j)
      {
        float value = (low_level_color_[j] + (color_multiplier_[j] * data_->block_beam_led_levels_[i])) * 255.0f;
        // ROS_ASSERT_MSG(!(value < 0.0) && !(value > 255.0), "Invalid value >%.2f< computed.", value);
        // ROS_INFO("Node %i color %i value >%.2f< computed.", i, j, value);
        data_->block_beam_led_values_(j, i) = static_cast<led_channel_t>(value);
      }
    }
  }
  if (data_->total_num_pixel_beam_leds_ > 0)
  {
    for (unsigned int i = 0; i < data_->pixel_beam_led_levels_.size(); ++i)
    {
      ROS_ASSERT(!(data_->pixel_beam_led_levels_[i] < 0.0) && !(data_->pixel_beam_led_levels_[i] > 1.0));
      for (unsigned int j = 0; j < NUM_COLOR_VALUES; ++j)
      {
        float value = (low_level_color_[j] + (color_multiplier_[j] * data_->pixel_beam_led_levels_[i])) * 255.0f;
        // ROS_ASSERT_MSG(!(value < 0.0) && !(value > 255.0), "Invalid value >%.2f< computed.", value);
        // ROS_INFO("Node %i color %i value >%.2f< computed.", i, j, value);
        data_->pixel_beam_led_values_(j, i) = static_cast<led_channel_t>(value);
      }
    }
  }

  return true;
}

bool DecColorProcessor::stop()
{

  return true;
}

}
