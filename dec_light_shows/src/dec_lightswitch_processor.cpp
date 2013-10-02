/*
 * dec_light_processor.cpp
 *
 *  Created on: Jul 14, 2013
 *      Author: pastor
 */

#include <dec_utilities/assert.h>
#include <dec_light_show_manager/dec_light_show_utilities.h>

#include <dec_light_shows/dec_light_show_math_utilities.h>
#include <dec_light_shows/dec_lightswitch_processor.h>

PLUGINLIB_DECLARE_CLASS(dec_light_shows, DecLightSwitchProcessor, dec_light_shows::DecLightSwitchProcessor,
                        dec_light_show_manager::DecLightShow)

using namespace dec_light_show_manager;

namespace dec_light_shows
{


DecLightSwitchProcessor::DecLightSwitchProcessor()
{
}

bool DecLightSwitchProcessor::initialize(XmlRpc::XmlRpcValue& config)
{
  return true;
}

bool DecLightSwitchProcessor::start()
{
  data_->node_led_levels_.setConstant(DecData::BASE_LIGHT_LEVEL);
  return true;
}

bool DecLightSwitchProcessor::update()
{
  data_->node_led_levels_.setConstant(DecData::BASE_LIGHT_LEVEL);
  for (unsigned int i = 0; i < data_->sensor_levels_.size(); ++i)
  {
    data_->node_led_levels_(i) = data_->sensor_levels_(i);
  }
  for (unsigned int i = 0; i < data_->node_led_levels_.size(); ++i)
  {
    if(data_->node_led_levels_(i) > DecData::MAX_LIGHT_LEVEL)
    {
      data_->node_led_levels_(i) = DecData::MAX_LIGHT_LEVEL;
    }
  }
  return true;
}

bool DecLightSwitchProcessor::stop()
{
  return true;
}

}
