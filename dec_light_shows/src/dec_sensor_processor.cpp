/*
 * dec_sensor_processor.cpp
 *
 *  Created on: Jul 14, 2013
 *      Author: pastor
 */

#include <dec_utilities/assert.h>

#include <dec_light_show_manager/dec_light_show_utilities.h>
#include <dec_light_shows/dec_sensor_processor.h>

PLUGINLIB_DECLARE_CLASS(dec_light_shows, DecSensorProcessor, dec_light_shows::DecSensorProcessor,
                        dec_light_show_manager::DecLightShow)

using namespace dec_light_show_manager;

namespace dec_light_shows
{

DecSensorProcessor::DecSensorProcessor()
 : build_up_window_size_(0)
{
}

bool DecSensorProcessor::initialize(XmlRpc::XmlRpcValue& config)
{
  ROS_VERIFY(DecLightShowUtilities::getParam(config, "build_up_window_size", build_up_window_size_));
  return true;
}

bool DecSensorProcessor::start()
{
  ROS_INFO("Starting >%s<.", name_.c_str());
  return true;
}

bool DecSensorProcessor::update()
{

  return true;
}

bool DecSensorProcessor::stop()
{

  return true;
}

}
