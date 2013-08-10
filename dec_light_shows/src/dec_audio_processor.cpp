/*
 * dec_audio_processor.cpp
 *
 *  Created on: Jul 14, 2013
 *      Author: pastor
 */

#include <dec_light_shows/dec_audio_processor.h>

PLUGINLIB_DECLARE_CLASS(dec_light_shows, DecAudioProcessor, dec_light_shows::DecAudioProcessor,
                        dec_light_show_manager::DecLightShow)

namespace dec_light_shows
{

bool DecAudioProcessor::initialize(XmlRpc::XmlRpcValue& config)
{

  return true;
}

bool DecAudioProcessor::start()
{

  return true;
}

bool DecAudioProcessor::update()
{

  return true;
}

bool DecAudioProcessor::stop()
{

  return true;
}

}
