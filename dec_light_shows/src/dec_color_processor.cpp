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
  readParameters(config);

  ROS_VERIFY(DecLightShowUtilities::getParam(config, "low_level_color", low_level_color_));
  ROS_VERIFY(DecLightShowUtilities::getParam(config, "high_level_color", high_level_color_));
  // colors_.resize(4, 0.0);
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
  // colors_ = low_level_color_;
  for (unsigned int i = 0; i < data_->node_led_levels_.size(); ++i)
  {
    ROS_ASSERT(!(data_->node_led_levels_[i] < 0.0) && !(data_->node_led_levels_[i] > 1.0));
    unsigned int color_index = 0;
    bool found = false;
    for (unsigned int n = 1; !found && n < colors_.size(); ++n)
    {
      if (data_->node_led_levels_[i] < starts_[n])
      {
        color_index = n - 1;
        found = true;
      }
    }
    for (unsigned int j = 0; j < NUM_COLOR_VALUES; ++j)
    {
      float value = (colors_[color_index][j] + (color_multipliers_[color_index][j] * data_->node_led_levels_[i])) * 255.0f;
      data_->node_led_values_(j, i) = static_cast<led_channel_t>(value);
    }

//    for (unsigned int j = 0; j < NUM_COLOR_VALUES; ++j)
//    {
//      float value = (low_level_color_[j] + (color_multiplier_[j] * data_->node_led_levels_[i])) * 255.0f;
//      // ROS_ASSERT_MSG(!(value < 0.0) && !(value > 255.0), "Invalid value >%.2f< computed.", value);
//      // ROS_INFO("Node %i color %i value >%.2f< computed.", i, j, value);
//      data_->node_led_values_(j, i) = static_cast<led_channel_t>(value);
//    }
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


void DecColorProcessor::readParameters(XmlRpc::XmlRpcValue& config)
{
  ROS_ASSERT(config.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  const std::string KEY = "color_levels";
  ROS_VERIFY(config.hasMember(KEY));
  XmlRpc::XmlRpcValue color_levels = config[KEY];
  ROS_ASSERT(color_levels.getType() == XmlRpc::XmlRpcValue::TypeArray);
  colors_.clear();
  color_multipliers_.clear();
  starts_.clear();
  for (int i = 0; i < color_levels.size(); ++i)
  {
    ROS_ASSERT(color_levels[i].hasMember("start"));
    float start = 0.0;
    ROS_VERIFY(DecLightShowUtilities::getParam(color_levels[i], "start", start));
    ROS_ASSERT(!(start < 0.0) && !(start > 1.0));
    starts_.push_back(start);

    ROS_ASSERT(color_levels[i].hasMember("color"));
    std::vector<float> color;
    ROS_VERIFY(DecLightShowUtilities::getParam(color_levels[i], "color", color));
    ROS_ASSERT(color.size() == 4);
    for (unsigned int j = 0; j < color.size(); ++j)
    {
      ROS_ASSERT(!(color[j] < 0.0) && !(color[j] > 1.0));
    }
    colors_.push_back(color);
    ROS_DEBUG("Read: %.2f -> %.2f %.2f %.2f %.2f", start, color[0], color[1], color[2], color[3]);
  }

  ROS_ASSERT(colors_.size() > 0);
  for (unsigned int i = 0; i < colors_.size() - 1; ++i)
  {
    std::vector<float> color_multiplier(NUM_COLOR_VALUES, 0.0);
    for (unsigned int j = 0; j < NUM_COLOR_VALUES; ++j)
    {
      color_multiplier[j] = colors_[i + 1][j] - colors_[i][j];
    }
    color_multipliers_.push_back(color_multiplier);
  }
}

}
