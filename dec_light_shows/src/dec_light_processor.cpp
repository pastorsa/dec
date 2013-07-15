/*
 * dec_light_processor.cpp
 *
 *  Created on: Jul 14, 2013
 *      Author: pastor
 */

#include <dec_light_shows/dec_light_processor.h>

PLUGINLIB_DECLARE_CLASS(dec_light_shows, DecLightProcessor, dec_light_shows::DecLightProcessor,
                        dec_light_show_manager::DecLightShow)

namespace dec_light_shows
{

bool DecLightProcessor::initialize(XmlRpc::XmlRpcValue& config)
{

  return true;
}

bool DecLightProcessor::start()
{

  return true;
}

bool DecLightProcessor::update()
{

  return true;
}

bool DecLightProcessor::stop()
{

  return true;
}

}
