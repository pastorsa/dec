/*
 * dec_audio_modulation.cpp
 *
 *  Created on: Jul 14, 2013
 *      Author: pastor
 */

#include <dec_light_shows/dec_audio_modulation.h>

PLUGINLIB_DECLARE_CLASS(dec_light_shows, DecAudioModulation, dec_light_shows::DecAudioModulation,
                        dec_light_show_manager::DecLightShow)

namespace dec_light_shows
{

bool DecAudioModulation::initialize(XmlRpc::XmlRpcValue& config)
{

  return true;
}

bool DecAudioModulation::start()
{

  return true;
}

bool DecAudioModulation::update()
{

  return true;
}

bool DecAudioModulation::stop()
{

  return true;
}

}
