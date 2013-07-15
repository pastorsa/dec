/*
 * dec_light_show_stack.cpp
 *
 *  Created on: Jul 14, 2013
 *      Author: pastor
 */

#include <dec_light_show_manager/dec_light_show_stack.h>

namespace dec_light_show_manager
{

DecLightShowStack::DecLightShowStack(const std::string& name, const int id, const std::vector<int>& groups)
  : name_(name), id_(id), groups_(groups)
{
}

DecLightShowStack::~DecLightShowStack()
{
}

bool DecLightShowStack::start()
{
  return true;
}

bool DecLightShowStack::update()
{
  return true;
}

bool DecLightShowStack::stop()
{
  return true;
}

void DecLightShowStack::addLightShow(boost::shared_ptr<DecLightShow> light_show)
{
  light_shows_.push_back(light_show);
  light_show_map_.insert(LightShowMap::value_type(light_show->getName(), light_show));
}

bool DecLightShowStack::hasLightShow(const std::string& name, boost::shared_ptr<DecLightShow>& light_show) const
{
  LightShowMap::const_iterator light_show_it = light_show_map_.find(name);
  if (light_show_it == light_show_map_.end())
    return false;

  light_show = light_show_it->second;
  return true;
}

bool DecLightShowStack::hasLightShow(const std::string& name) const
{
  LightShowMap::const_iterator light_show_it = light_show_map_.find(name);
  if (light_show_it == light_show_map_.end())
    return false;

  return true;
}

bool DecLightShowStack::conflicts(boost::shared_ptr<DecLightShowStack> other) const
{
  for (unsigned int i=0; i<groups_.size(); ++i)
  {
    if (groups_[i]==1 && other->groups_[i]==1)
      return true;
  }
  return false;
}

bool DecLightShowStack::belongsToGroup(int group_id) const
{
  return groups_[group_id] == 1;
}

std::vector<boost::shared_ptr<DecLightShow> > const& DecLightShowStack::getLightShows() const
{
  return light_shows_;
}

void DecLightShowStack::getLightShows(std::vector<int>& light_shows) const
{
  for (unsigned int i = 0; i < light_shows_.size(); ++i)
  {
    light_shows[light_shows_[i]->getId()] = 1;
  }
}

std::string DecLightShowStack::getName() const
{
  return name_;
}

int DecLightShowStack::getId() const
{
  return id_;
}



}
