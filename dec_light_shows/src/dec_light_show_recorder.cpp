/*
 * dec_light_show_recorder.cpp
 *
 *  Created on: Jul 14, 2013
 *      Author: pastor
 */

#include <dec_light_shows/dec_light_show_recorder.h>

PLUGINLIB_DECLARE_CLASS(dec_light_shows, DecLightShowRecorder, dec_light_shows::DecLightShowRecorder,
                        dec_light_show_manager::DecLightShow)

namespace dec_light_shows
{

bool DecLightShowRecorder::initialize(XmlRpc::XmlRpcValue& config)
{

  return true;
}

bool DecLightShowRecorder::start()
{

  return true;
}

bool DecLightShowRecorder::update()
{

  return true;
}

bool DecLightShowRecorder::stop()
{

  return true;
}

}
