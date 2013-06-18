/*
 * world_state.cpp
 *
 *  Created on: Jan 8, 2013
 *      Author: kalakris
 */

#include <dec_world_state/world_state.h>
#include <dec_world_state/world_state_impl.h>
#include <conversions/ros_to_tf.h>
#include <dec_utilities/singleton_factory.h>

using namespace conversions;

namespace dec_world_state
{

WorldState::WorldState()
{
  impl_ = dec_utilities::SingletonFactory<WorldStateImpl>::instance->get("/world_state");
}

WorldState::~WorldState()
{
}

bool WorldState::getObject(const std::string& name, dec_msgs::Object& object, const bool verbose)
{
  return impl_->getObject(name, object, verbose);
}

bool WorldState::getObjectPose(const std::string& name, tf::Pose& pose, const bool verbose)
{
  return impl_->getObjectPose(name, pose, verbose);
}

void WorldState::addObjects(const dec_msgs::Objects& objects)
{
  return impl_->addObjects(objects);
}

void WorldState::addObjects(const std::vector<dec_msgs::Object>& objects)
{
  return impl_->addObjects(objects);
}

void WorldState::addObject(const dec_msgs::Object& object)
{
  return impl_->addObject(object);
}

void WorldState::removeObjects(const dec_msgs::Objects& objects)
{
  return impl_->removeObjects(objects);
}

void WorldState::removeObject(const dec_msgs::Object& object)
{
  return impl_->removeObject(object);
}

void WorldState::removeAllObjects()
{
  return impl_->removeAllObjects();
}

void WorldState::getObjects(std::vector<dec_msgs::Object>& objects)
{
  return impl_->getObjects(objects);
}

void WorldState::addObject(const std::string& name, const tf::Pose& pose)
{
  return impl_->addObject(name, pose);
}

void WorldState::removeObject(const std::string& name)
{
  return impl_->removeObject(name);
}

void WorldState::addCallback(boost::function<void (const ObjectState& state)> callback)
{
  impl_->addCallback(callback);
}

int WorldState::getNumSubscribers()
{
  return impl_->getNumSubscribers();
}

} /* namespace dec_world_state */
