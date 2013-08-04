/*
 * dec_idle.cpp
 *
 *  Created on: Jul 14, 2013
 *      Author: pastor
 */

#include <dec_light_shows/dec_idle.h>

PLUGINLIB_DECLARE_CLASS(dec_light_shows, DecIdle, dec_light_shows::DecIdle,
                        dec_light_show_manager::DecLightShow)

namespace dec_light_shows
{

bool DecIdle::initialize(XmlRpc::XmlRpcValue& config)
{

  return true;
}

bool DecIdle::start()
{

  return true;
}

bool DecIdle::update()
{

  return true;
}

bool DecIdle::stop()
{

  return true;
}

}
