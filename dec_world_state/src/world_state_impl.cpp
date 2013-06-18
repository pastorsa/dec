/*
 * world_state_impl.cpp
 *
 *  Created on: Jan 14, 2013
 *      Author: kalakris
 */

#include <dec_world_state/world_state_impl.h>
#include <conversions/ros_to_tf.h>
#include <conversions/tf_to_ros.h>
#include <dec_utilities/singleton_factory.h>

using namespace conversions;

namespace dec_utilities
{
BOOST_SINGLETON_TEMPLATE_PLACEMENT(SingletonFactory<dec_world_state::WorldStateImpl>)
}

namespace dec_world_state
{

WorldStateImpl::WorldStateImpl()
{
  object_sub_ = node_handle_.subscribe("/world_state/objects", 1000, &WorldStateImpl::objectCallbackEvent, this);
  object_pub_ = node_handle_.advertise<ObjectState>("/world_state/objects", 1000, false);
}

WorldStateImpl::~WorldStateImpl()
{
}


void WorldStateImpl::objectCallback(const ObjectState& object_state)
{
  const std::string& name = object_state.object.name;
  if (object_state.operation == object_state.REMOVE)
  {
    ObjectMap::iterator item = object_map_.find(name);
    if (item != object_map_.end())
    {
      object_map_.erase(item);
    }
  }
  else if (object_state.operation == object_state.ADD)
  {
    object_map_[name] = object_state.object;
  }

  for (size_t i=0; i<callbacks_.size(); ++i)
  {
    callbacks_[i](object_state);
  }
}

void WorldStateImpl::objectCallbackEvent(const ros::MessageEvent<ObjectState const>& object_state_event)
{
  const ObjectState& object_state = *(object_state_event.getMessage());
  if (object_state_event.getPublisherName() != ros::this_node::getName())
  {
    objectCallback(object_state);
  }
}

bool WorldStateImpl::getObject(const std::string& name,
                               dec_msgs::Object& object,
                               const bool verbose)
{
  if(object_map_.empty())
  {
    ROS_ERROR_COND(verbose, "No objects received yet.");
    return false;
  }

  ObjectMap::iterator item = object_map_.find(name);
  if (item == object_map_.end())
  {
    ROS_ERROR_COND(verbose, "Couldn't find object %s in world state", name.c_str());
    if(verbose)
    {
      ROS_WARN("Only know about...");
      for (item = object_map_.begin(); item != object_map_.end(); ++item)
        ROS_INFO("  %s", item->first.c_str());
    }
    return false;
  }
  object = item->second;
  return true;
}

bool WorldStateImpl::getObjectPose(const std::string& name, tf::Pose& pose, const bool verbose)
{
  dec_msgs::Object object;
  if (!getObject(name, object, verbose))
    return false;
  convert(object.pose.pose.pose, pose);
  return true;
}

void WorldStateImpl::addObjects(const std::vector<dec_msgs::Object>& objects)
{
  for (unsigned int i=0; i<objects.size(); ++i)
  {
    addObject(objects[i]);
  }
}

void WorldStateImpl::addObjects(const dec_msgs::Objects& objects)
{
  addObjects(objects.objects);
}

void WorldStateImpl::removeObjects(const dec_msgs::Objects& objects)
{
  for (unsigned int i=0; i<objects.objects.size(); ++i)
  {
    removeObject(objects.objects[i]);
  }
}

void WorldStateImpl::addObject(const dec_msgs::Object& object)
{
  publishObject(object, ObjectState::ADD);
}

void WorldStateImpl::addObject(const std::string& name, const tf::Pose& pose)
{
  dec_msgs::Object object;
  object.name = name;
  object.pose.header.frame_id = "/BASE";
  object.pose.header.stamp = ros::Time::now();
  convert(pose, object.pose.pose.pose);
  addObject(object);
}

void WorldStateImpl::removeObject(const dec_msgs::Object& object)
{
  publishObject(object, ObjectState::REMOVE);
}

void WorldStateImpl::removeObject(const std::string& name)
{
  dec_msgs::Object object;
  object.name = name;
  removeObject(object);
}

void WorldStateImpl::removeAllObjects()
{
  std::vector<ObjectMap::value_type> v(object_map_.begin(), object_map_.end());
  for (unsigned int i=0; i<v.size(); ++i)
  {
    removeObject(v[i].second);
  }
}

void WorldStateImpl::publishObject(const dec_msgs::Object& object, int operation)
{
  ObjectState state;
  state.object = object;
  state.operation = operation;
  objectCallback(state);
  object_pub_.publish(state);
}

void WorldStateImpl::getObjects(std::vector<dec_msgs::Object>& objects)
{
  objects.clear();
  for (ObjectMap::iterator it = object_map_.begin(); it != object_map_.end(); ++it)
  {
    objects.push_back(it->second);
  }
}

void WorldStateImpl::addCallback(boost::function<void (const ObjectState& state)> callback)
{
  callbacks_.push_back(callback);
}

int WorldStateImpl::getNumSubscribers()
{
  return object_pub_.getNumSubscribers();
}

} /* namespace dec_world_state */
