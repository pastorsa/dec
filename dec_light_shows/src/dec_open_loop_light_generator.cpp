/*
 * dec_open_loop_light_generator.cpp
 *
 *  Created on: Jul 14, 2013
 *      Author: pastor
 */

#include <dec_light_shows/dec_open_loop_light_generator.h>

PLUGINLIB_DECLARE_CLASS(dec_light_shows, DecOpenLoopLightGenerator, dec_light_shows::DecOpenLoopLightGenerator,
                        dec_light_show_manager::DecLightShow)

namespace dec_light_shows
{

bool DecOpenLoopLightGenerator::initialize(XmlRpc::XmlRpcValue& config)
{

  return true;
}

bool DecOpenLoopLightGenerator::start()
{

  return true;
}

bool DecOpenLoopLightGenerator::update()
{

  return true;
}

bool DecOpenLoopLightGenerator::stop()
{

  return true;
}

}
