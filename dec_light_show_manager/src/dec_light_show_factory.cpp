/*
 * dec_light_show_factory.cpp
 *
 *  Created on: Jul 14, 2013
 *      Author: pastor
 */


#include <dec_light_show_manager/dec_light_show_factory.h>

namespace dec_light_show_manager
{

DecLightShowFactory::DecLightShowFactory()
{

}

DecLightShowFactory::~DecLightShowFactory()
{

}

bool DecLightShowFactory::createLightShowByName(const std::string& class_name,
                                                 const std::string& name,
                                                 const int id,
                                                 boost::shared_ptr<DecLightShowData> data,
                                                 boost::shared_ptr<DecLightShow>& light_show)
{
  bool success = false;
  try
  {
    ROS_INFO("Creating light show instance >%s<.", class_name.c_str());
    light_show = light_show_loader_->createInstance(class_name);
    light_show->initializeBase(name, data, id);
    success = true;
  }
  catch (pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("Light show plugin >%s< failed to load: %s", class_name.c_str(), ex.what());
  }
  return success;
}

void DecLightShowFactory::reset()
{
  light_show_loader_.reset(new pluginlib::ClassLoader<DecLightShow>("dec_light_show_manager", "dec_light_show_manager::DecLightShow"));
}

}
