/*
 * dec_color_processor.cpp
 *
 *  Created on: Jul 14, 2013
 *      Author: pastor
 */

#include <dec_utilities/assert.h>
#include <dec_light_show_manager/dec_light_show_utilities.h>

#include <dec_light_shows/dec_color_processor.h>

PLUGINLIB_DECLARE_CLASS(dec_light_shows, DecColorProcessor, dec_light_shows::DecColorProcessor,
                        dec_light_show_manager::DecLightShow)

using namespace dec_light_show_manager;

namespace dec_light_shows
{

bool DecColorProcessor::initialize(XmlRpc::XmlRpcValue& config)
{
  readParameters(config);
  return true;
}

bool DecColorProcessor::start()
{

  return true;
}

bool DecColorProcessor::update()
{
  double x = 0.0;
  double xd = 0.0;
  double xdd = 0.0;
  // take node_led_levels_ and beam_led_levels_ and process them into node_led_values_ and beam_led_values_
  // colors_ = low_level_color_;
  for (int i = 0; i < (int)data_->node_led_levels_.size(); ++i)
  {
    ROS_ASSERT(!(data_->node_led_levels_[i] < 0.0) && !(data_->node_led_levels_[i] > 1.0));
    unsigned int color_index = 0;
    bool found = false;
    for (unsigned int n = 1; !found && n < colors_.size(); ++n)
    {
      if (data_->node_led_levels_[i] - 1e-3 < starts_[n])
      {
        color_index = n - 1;
        found = true;
      }
    }
    for (unsigned int j = 0; j < NUM_COLOR_VALUES; ++j)
    {
      splines_[color_index][j]->sample((double)(data_->node_led_levels_[i] - starts_[color_index]), x, xd, xdd);
      // float value = (colors_[color_index][j] + (color_multipliers_[color_index][j] * data_->node_led_levels_[i])) * 255.0f;
      data_->node_led_values_(j, i) = static_cast<led_channel_t>(x * 255.0f);
    }
  }

  if (data_->total_num_block_beam_leds_ > 0)
  {
    for (int i = 0; i < (int)data_->block_beam_led_levels_.size(); ++i)
    {
      ROS_ASSERT(!(data_->block_beam_led_levels_[i] < 0.0) && !(data_->block_beam_led_levels_[i] > 1.0));
      unsigned int color_index = 0;
      bool found = false;
      for (unsigned int n = 1; !found && n < colors_.size(); ++n)
      {
        if (data_->block_beam_led_levels_[i] - 1e-3 < starts_[n])
        {
          color_index = n - 1;
          found = true;
        }
      }
      for (unsigned int j = 0; j < NUM_COLOR_VALUES; ++j)
      {
        splines_[color_index][j]->sample((double)(data_->block_beam_led_levels_[i] - starts_[color_index]), x, xd, xdd);
        // float value = (colors_[color_index][j] + (color_multipliers_[color_index][j] * data_->block_beam_led_levels_[i])) * 255.0f;
        data_->block_beam_led_values_(j, i) = static_cast<led_channel_t>(x * 255.0f);
      }
    }
  }
  if (data_->total_num_pixel_beam_leds_ > 0)
  {
    for (int i = 0; i < (int)data_->pixel_beam_led_levels_.size(); ++i)
    {
      ROS_ASSERT(!(data_->pixel_beam_led_levels_[i] < 0.0) && !(data_->pixel_beam_led_levels_[i] > 1.0));
      unsigned int color_index = 0;
      bool found = false;
      for (unsigned int n = 0; !found && n < colors_.size() - 1; ++n)
      {
        if (data_->pixel_beam_led_levels_[i] - 1e-3 < starts_[n + 1])
        {
          color_index = n;
          found = true;
        }
      }
      ROS_ASSERT(found);
      for (unsigned int j = 0; j < NUM_COLOR_VALUES; ++j)
      {
        splines_[color_index][j]->sample((double)(data_->pixel_beam_led_levels_[i] - starts_[color_index]), x, xd, xdd);
        // float value = (colors_[color_index][j] + (color_multipliers_[color_index][j] * data_->pixel_beam_led_levels_[i])) * 255.0f;
        data_->pixel_beam_led_values_(j, i) = static_cast<led_channel_t>(x * 255.0f);
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
  ROS_ASSERT_MSG(color_levels.size() > 1, "Number of colors specified in >%s< is invalid >%i<. It must be greater than two.",
                 name_.c_str(), color_levels.size());
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
    ROS_DEBUG("Read (%s): %.2f -> %.2f %.2f %.2f %.2f", name_.c_str(), start, color[0], color[1], color[2], color[3]);
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

    std::vector<boost::shared_ptr<splines::QuinticSpline> > splines;
    for (unsigned int j = 0; j < NUM_COLOR_VALUES; ++j)
    {
      boost::shared_ptr<splines::QuinticSpline> spline(new splines::QuinticSpline());
      spline->setCoefficients(colors_[i][j], 0.0, 0.0, colors_[i + 1][j], 0.0, 0.0, starts_[i + 1] - starts_[i]);
      splines.push_back(spline);
    }
    splines_.push_back(splines);
  }
}

}
