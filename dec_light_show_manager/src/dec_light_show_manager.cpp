/*
 * dec_light_show_manager.cpp
 *
 *  Created on: Jul 14, 2013
 *      Author: pastor
 */

#include <dec_utilities/param_server.h>
#include <dec_light_show_manager/dec_light_show_manager.h>

#define LIGHT_SHOW_NAMESPACE "light_shows"
#define LIGHT_SHOW_STACKS_NAMESPACE "light_show_stacks"

namespace dec_light_show_manager
{

static const int ROS_TIME_OFFSET = 1340100000;

DecLightShowManager::DecLightShowManager() :
    num_light_show_stacks_(0), num_light_shows_(0), light_shows_running_(false),
    switching_light_shows_(false), switching_light_shows_success_(false),
    local_node_handle_("~"),
    start_ros_time_in_sec_(0.0)
{
  dec_mutex_init(&switching_light_shows_mutex_);
  dec_cond_init(&switching_light_shows_cond_);
}

DecLightShowManager::~DecLightShowManager()
{
  dec_mutex_destroy(&switching_light_shows_mutex_);
  dec_cond_destroy(&switching_light_shows_cond_);
}

bool DecLightShowManager::initialize()
{
  light_show_factory_.reset(new DecLightShowFactory());
  light_show_data_.reset(new DecLightShowData());
  if(!light_show_data_->initialize(local_node_handle_))
  {
    return false;
  }

  if (!loadLightShows())
  {
    return false;
  }

  // advertise service:
  switch_light_show_stack_service_ = local_node_handle_.advertiseService(
      "switchLightShowStack", &DecLightShowManager::switchLightShowStackService, this);

  light_shows_running_ = true;
  ROS_INFO_COND(light_shows_running_, "DEC light show manager is running.");
  start_ros_time_ = ros::Time::now();
  start_ros_time_in_sec_ = start_ros_time_.toSec();
  return true;
}

bool DecLightShowManager::loadLightShows()
{
  // make sure that all light_shows are destroyed
  // light_shows_.clear();
  // light_show_list_.clear();
  light_show_stacks_.clear();
  light_show_stack_list_.clear();
  light_show_stack_names_.clear();
  active_light_show_stacks_.clear();
  next_light_show_stacks_.clear();

  if (!readLightShows())
  {
    ROS_ERROR("Couldn't read light show configurations.");
    return false;
  }

  if (!readLightShowStacks())
  {
    ROS_ERROR("Couldn't read light show stacks.");
    return false;
  }

  // memory pre-allocation
  active_light_show_stacks_.reserve(num_light_show_stacks_);
  next_light_show_stacks_.reserve(num_light_show_stacks_);
  stack_active_list_.reserve(num_light_show_stacks_);
  stack_next_active_list_.reserve(num_light_show_stacks_);
  stack_insert_list_.reserve(num_light_show_stacks_);
  stack_insert_list_tmp_.reserve(num_light_show_stacks_);
  light_show_stop_list_.reserve(num_light_shows_);
  light_show_start_list_.reserve(num_light_shows_);
  light_show_active_list_.reserve(num_light_shows_);
  light_show_next_active_list_.reserve(num_light_shows_);
  active_light_show_stacks_.clear();
  next_light_show_stacks_.clear();
  stack_active_list_.clear();
  stack_active_list_.resize(light_show_stack_list_.size(), 0);
  stack_insert_list_.clear();
  stack_insert_list_tmp_.clear();
  light_show_stop_list_.clear();
  light_show_start_list_.clear();
  light_show_active_list_.clear();
  light_show_next_active_list_.clear();

  // get the default light_show stack running:
  std::vector<std::string> default_light_show_stacks;
  if (!dec_utilities::read(local_node_handle_, "default_light_show_stacks", default_light_show_stacks))
  {
    ROS_ERROR("Couldn't read default_light_show_stacks.");
    return false;
  }

//  if (!switchLightShowStack(default_light_show_stacks))
//  {
//    ROS_ERROR("Error loading default light_show stacks.");
//    return false;
//  }

  return true;
}

bool DecLightShowManager::switchLightShowStackService(dec_light_show_msgs::SwitchLightShowStack::Request& request,
                                                      dec_light_show_msgs::SwitchLightShowStack::Response& response)
{

  // only return list of stacks
  if(request.light_show_stacks.empty())
  {
    response.light_show_stacks = light_show_stack_names_;
    response.result = dec_light_show_msgs::SwitchLightShowStack::Response::SUCCEEDED;
    return true;
  }

  // switch stacks
  bool success = switchLightShowStack(request.light_show_stacks);
  if (success)
  {
    response.result = dec_light_show_msgs::SwitchLightShowStack::Response::SUCCEEDED;
  }
  else
  {
    response.result = dec_light_show_msgs::SwitchLightShowStack::Response::FAILED;
  }
  return true;
}

bool DecLightShowManager::getLightShowStackByName(const std::string& name,
                                                  boost::shared_ptr<DecLightShowStack>& light_show_stack)
{
  LightShowStackMap::iterator light_show_stack_it = light_show_stacks_.find(name);
  if (light_show_stack_it == light_show_stacks_.end())
  {
    return false;
  }
  light_show_stack = light_show_stack_it->second;

  return true;
}

bool DecLightShowManager::switchLightShowStack(const std::vector<std::string>& light_show_stack_names)
{
  ROS_DEBUG("Switching to stacks:");
  stack_insert_list_tmp_.clear();
  stack_insert_list_tmp_.resize(num_light_show_stacks_, 0);
  for (unsigned int i = 0; i < light_show_stack_names.size(); ++i)
  {
    boost::shared_ptr<DecLightShowStack> light_show_stack;
    if (!getLightShowStackByName(light_show_stack_names[i], light_show_stack))
    {
      ROS_ERROR("LightShow stack >%s< not found!", light_show_stack_names[i].c_str());
      return false;
    }
    int id = light_show_stack->getId();
    stack_insert_list_tmp_[id] = 1;
    ROS_DEBUG("  %s", light_show_stack_names[i].c_str());
  }

  dec_mutex_lock(&switching_light_shows_mutex_);

  stack_insert_list_ = stack_insert_list_tmp_;
  switching_light_shows_success_ = false;
  switching_light_shows_ = true;

  bool ret = false;
  if (abs(dec_cond_timedwait_relative(&switching_light_shows_cond_, &switching_light_shows_mutex_, 3000000000))==ETIMEDOUT)
  {
    ret = false;
    ROS_ERROR("Time out reached when switching light show stack.");
  }
  else
  {
    ret = switching_light_shows_success_;
  }
  ROS_ERROR_COND(!ret, "Failed to switch light show stack.");

  dec_mutex_unlock(&switching_light_shows_mutex_);

  return ret;
}

void DecLightShowManager::debugLightShowList(const std::vector<int>& light_shows)
{
  for (int i = 0; i < num_light_shows_; ++i)
  {
    if (light_shows[i])
    {
      ROS_INFO("%d) %s (id=%d)", i + 1, light_show_list_[i]->getName().c_str(), light_show_list_[i]->getId());
    }
  }
}

void DecLightShowManager::debugLightShowStackList(const std::vector<int>& light_shows)
{
  for (int i = 0; i < num_light_show_stacks_; ++i)
  {
    if (light_shows[i])
    {
      ROS_INFO("%d) %s (id=%d)", i + 1, light_show_stack_list_[i]->getName().c_str(),
               light_show_stack_list_[i]->getId());
    }
  }
}

bool DecLightShowManager::processLightShowSwitch()
{
  // for each stack newly inserted, remove all conflicting current stacks
  stack_next_active_list_ = stack_active_list_;

  for (int i = 0; i < num_light_show_stacks_; ++i)
  {
    if (!stack_insert_list_[i])
      continue;
    // stack i needs to be inserted, find all conflicts and remove them
    for (int j = 0; j < num_light_show_stacks_; ++j)
    {
      if (!stack_next_active_list_[j])
        continue;
      if (light_show_stack_list_[i]->conflicts(light_show_stack_list_[j]))
      {
        // ROS_INFO("%s conflicts with %s", light_show_stack_list_[i]->getName().c_str(), light_show_stack_list_[j]->getName().c_str());
        // j is conflicting, delete it
        stack_next_active_list_[j] = 0;
      }
    }
    // add i
    stack_next_active_list_[i] = 1;
  }

  // ROS_INFO("stack_active_list:");
  // debugLightShowStackList(stack_active_list_);
  // ROS_INFO("stack next active list:");
  // debugLightShowStackList(stack_next_active_list_);

  // get the full list of light_shows, before and after
  light_show_active_list_.clear();
  light_show_active_list_.resize(num_light_shows_, 0);
  light_show_next_active_list_ = light_show_active_list_;
  for (int i = 0; i < num_light_show_stacks_; ++i)
  {
    if (stack_active_list_[i])
      light_show_stack_list_[i]->getLightShows(light_show_active_list_);
    if (stack_next_active_list_[i])
      light_show_stack_list_[i]->getLightShows(light_show_next_active_list_);
  }

  // ROS_INFO("light_show_active_list:");
  // debugLightShowList(light_show_active_list_);
  // ROS_INFO("light_show next active list:");
  // debugLightShowList(light_show_next_active_list_);

  // now decide which light_shows to start and stop
  light_show_stop_list_.clear();
  light_show_stop_list_.resize(num_light_shows_, 0);
  light_show_start_list_ = light_show_stop_list_;
  for (int i = 0; i < num_light_shows_; ++i)
  {
    if (light_show_active_list_[i] && !light_show_next_active_list_[i])
      light_show_stop_list_[i] = 1;
    if (!light_show_active_list_[i] && light_show_next_active_list_[i])
      light_show_start_list_[i] = 1;
  }

  // ROS_INFO("light_show_stop_list:");
  // debugLightShowList(light_show_stop_list_);
  // ROS_INFO("light_show start list:");
  // debugLightShowList(light_show_start_list_);

  // decide the execution order of stack_next_active_list:
  next_light_show_stacks_.clear();
  stack_active_list_ = stack_next_active_list_;
  for (unsigned int i = 0; i < stack_groups_.size(); ++i)
  {
    for (int j = 0; j < num_light_show_stacks_; ++j)
    {
      if (stack_next_active_list_[j] && light_show_stack_list_[j]->belongsToGroup(i))
      {
        next_light_show_stacks_.push_back(light_show_stack_list_[j]);
        stack_next_active_list_[j] = 0;
      }
    }
  }

  // debug execution order:
  // ROS_INFO("Now executing stacks in this order:");
  // for (unsigned int i=0; i<next_light_show_stacks_.size(); ++i)
  // {
  //   ROS_INFO("%d) %s", i+1, next_light_show_stacks_[i]->getName().c_str());
  // }

  return true;
}

bool DecLightShowManager::update()
{
  bool ret = true;

  // set the ROS time:
  light_show_data_->ros_time_ = ros::Time::now();
  light_show_data_->ros_time_sec_ = (light_show_data_->ros_time_ - ros::Duration(ROS_TIME_OFFSET)).toSec();

  // fill sensor information
  light_show_data_->copySensorInformationFromStructure();

  if (light_shows_running_)
  {
    if (dec_mutex_trylock(&switching_light_shows_mutex_) == 0)
    {
      if (switching_light_shows_)
      {
        // switching_light_shows_attempted = true;
        processLightShowSwitch();

        int num_started = 0;
        int num_stopped = 0;

        // stop light_shows that are not needed any more:
        for (int i = 0; i < num_light_shows_; ++i)
        {
          if (light_show_stop_list_[i])
          {
            ROS_DEBUG("Stopping light show >%s<.", light_show_list_[i]->getName().c_str());
            bool this_ret = light_show_list_[i]->stop();
            ++num_stopped;
            if (!this_ret)
            {
              ROS_ERROR("Error stopping light show >%s<.", light_show_list_[i]->getName().c_str());
            }
            ret = ret && this_ret;
          }
        }

        // start light_shows that aren't running:
        for (int i = 0; i < num_light_shows_; ++i)
        {
          if (light_show_start_list_[i])
          {
            ROS_DEBUG("Starting light show >%s<.", light_show_list_[i]->getName().c_str());
            bool this_ret = light_show_list_[i]->start();
            ++num_started;
            if (!this_ret)
            {
              ROS_ERROR("Error starting light_show >%s<.", light_show_list_[i]->getName().c_str());
            }
            ret = ret && this_ret;
          }
        }

        switching_light_shows_ = false;
        switching_light_shows_success_ = ret;
        active_light_show_stacks_ = next_light_show_stacks_;
        next_light_show_stacks_.clear();

        // signal that we are done
        dec_cond_signal(&switching_light_shows_cond_);
        dec_mutex_unlock(&switching_light_shows_mutex_);
      }
      else
      {
        dec_mutex_unlock(&switching_light_shows_mutex_);
      }
    }

    // if something isn't right so far, don't run the light_shows:
    if (!ret)
    {
      return false;
    }

    // run the light_show stacks in order
    for (unsigned int i = 0; i < active_light_show_stacks_.size(); ++i)
    {
      ret = active_light_show_stacks_[i]->update();
      if (!ret)
      {
        ROS_ERROR("Failed to run light show stack >%s<, freezing.", active_light_show_stacks_[i]->getName().c_str());
        return ret;
      }
    }
  }

  // send light data
  if (!light_show_data_->copyLightDataToStructure())
  {
    return false;
  }

  return ret;
}

bool DecLightShowManager::readLightShows()
{
  light_shows_.clear();
  light_show_list_.clear();
  light_show_factory_->reset();

  // read the list of light_shows from the param server
  XmlRpc::XmlRpcValue light_shows;
  if (!local_node_handle_.getParam(LIGHT_SHOW_NAMESPACE, light_shows))
  {
    ROS_ERROR("Couldn't find parameter %s/%s", local_node_handle_.getNamespace().c_str(), LIGHT_SHOW_NAMESPACE);
    return false;
  }

  if (light_shows.getType() != XmlRpc::XmlRpcValue::TypeStruct)
  {
    ROS_ERROR("%s/%s must be a struct", local_node_handle_.getNamespace().c_str(), LIGHT_SHOW_NAMESPACE);
    return false;
  }

  // for each light_show defined:
  XmlRpc::XmlRpcValue::iterator light_show_it;
  int id = 0;
  for (light_show_it = light_shows.begin(); light_show_it != light_shows.end(); ++light_show_it)
  {
    std::string name = light_show_it->first;
    XmlRpc::XmlRpcValue light_show_config = light_show_it->second;

    if (!light_show_config.hasMember("type"))
    {
      ROS_ERROR("LightShow %s must have a type", name.c_str());
      return false;
    }

    std::string type = light_show_config["type"];

    // create the constructor class
    boost::shared_ptr<DecLightShow> light_show;
    if (!light_show_factory_->createLightShowByName(type, name, id, light_show_data_, light_show))
    {
      ROS_ERROR("Couldn't create instance of light_show type %s", type.c_str());
      return false;
    }

    // add it to our list of light_shows
    if (!light_show->initialize(light_show_config))
    {
      ROS_ERROR("Couldn't initialize light_show %s of type %s", name.c_str(), type.c_str());
      return false;
    }
    light_shows_.insert(LightShowMap::value_type(name, light_show));
    light_show_list_.push_back(light_show);
    ++id;
  }

  num_light_shows_ = light_show_list_.size();
  return true;
}

bool DecLightShowManager::readLightShowStacks()
{
  light_show_stacks_.clear();
  light_show_stack_names_.clear();
  light_show_stack_list_.clear();
  stack_groups_.clear();

  // first read in stack group names:
  if (!dec_utilities::read(local_node_handle_, "stack_groups", stack_groups_))
  {
    ROS_ERROR("Couldn't read %s/stack_groups array", local_node_handle_.getNamespace().c_str());
    return false;
  }

  // now read light_show stacks:
  XmlRpc::XmlRpcValue light_show_stacks;
  if (!local_node_handle_.getParam(LIGHT_SHOW_STACKS_NAMESPACE, light_show_stacks)
      || light_show_stacks.getType() != XmlRpc::XmlRpcValue::TypeArray || light_show_stacks.size() == 0)
  {
    ROS_ERROR("Parameter %s/%s must exist and contain at least 1 light_show stack definition.",
              local_node_handle_.getNamespace().c_str(), LIGHT_SHOW_STACKS_NAMESPACE);
    return false;
  }

  // for each light_show stack defined:
  for (int i = 0; i < light_show_stacks.size(); ++i)
  {
    if (!light_show_stacks[i].hasMember("name") || !light_show_stacks[i].hasMember("light_shows"))
    {
      ROS_ERROR("Each light_show stack must have a name and list of light_shows,");
      return false;
    }

    // get the name and light_show list
    std::string stack_name = light_show_stacks[i]["name"];
    XmlRpc::XmlRpcValue light_show_list = light_show_stacks[i]["light_shows"];
    if (light_show_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("%s/[%d]/light_shows must be an array", LIGHT_SHOW_STACKS_NAMESPACE, i);
      return false;
    }

    std::vector<std::string> group_names;
    if (!light_show_stacks[i].hasMember("groups")
        || !light_show_stacks[i]["groups"].getType() == XmlRpc::XmlRpcValue::TypeArray
        || !dec_utilities::getParam(light_show_stacks[i], "groups", group_names))
    {
      // use a default set:
      group_names.push_back("SENSOR");
      group_names.push_back("LIGHT");
    }

    // convert group names to vector of bool
    std::vector<int> group_indices(stack_groups_.size(), 0);
    for (unsigned int g = 0; g < group_names.size(); ++g)
    {
      bool found = false;
      for (unsigned int g2 = 0; g2 < stack_groups_.size(); ++g2)
      {
        if (group_names[g] == stack_groups_[g2])
        {
          found = true;
          group_indices[g2] = 1;
          break;
        }
      }
      if (!found)
      {
        ROS_ERROR("Stack %s has group %s which doesn't exist!", stack_name.c_str(), group_names[g].c_str());
      }
    }

    boost::shared_ptr<DecLightShowStack> light_show_stack(new DecLightShowStack(stack_name, i, group_indices));
    for (int j = 0; j < light_show_list.size(); ++j)
    {
      std::string light_show_name = light_show_list[j];
      LightShowMap::iterator light_show_it = light_shows_.find(light_show_name);
      if (light_show_it == light_shows_.end())
      {
        ROS_ERROR("LightShow %s in stack %s not defined", light_show_name.c_str(), stack_name.c_str());
      }
      else
      {
        light_show_stack->addLightShow(light_show_it->second);
      }
    }
    light_show_stacks_.insert(LightShowStackMap::value_type(stack_name, light_show_stack));
    light_show_stack_names_.push_back(stack_name);
    light_show_stack_list_.push_back(light_show_stack);

  }

  num_light_show_stacks_ = light_show_stack_list_.size();
  return true;
}

void DecLightShowManager::getLightShowStackNames(std::vector<std::string>& names) const
{
  names = light_show_stack_names_;
}

bool DecLightShowManager::getLightShowByName(const std::string& name, boost::shared_ptr<DecLightShow>& light_show)
{
  LightShowMap::iterator light_show_it = light_shows_.find(name);
  if (light_show_it == light_shows_.end())
    return false;

  light_show = light_show_it->second;
  return true;
}

}
