/*
 * dec_light_show_player.cpp
 *
 *  Created on: Jul 14, 2013
 *      Author: pastor
 */

#include <dec_light_shows/dec_light_show_player.h>

PLUGINLIB_DECLARE_CLASS(dec_light_shows, DecLightShowPlayer, dec_light_shows::DecLightShowPlayer,
                        dec_light_show_manager::DecLightShow)

namespace dec_light_shows
{

bool DecLightShowPlayer::initialize(XmlRpc::XmlRpcValue& config)
{

  return true;
}

bool DecLightShowPlayer::start()
{

  return true;
}

bool DecLightShowPlayer::update()
{

  return true;
}

bool DecLightShowPlayer::stop()
{

  return true;
}

}
