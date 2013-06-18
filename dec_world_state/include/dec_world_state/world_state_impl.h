/*
 * world_state_impl.h
 *
 *  Created on: Jan 14, 2013
 *      Author: kalakris
 */

#ifndef WORLD_STATE_IMPL_H_
#define WORLD_STATE_IMPL_H_

#include <ros/ros.h>
#include <boost/function.hpp>
#include <dec_world_state/ObjectState.h>
#include <dec_msgs/Objects.h>
#include <tf/transform_datatypes.h>
#include <map>

namespace dec_utilities
{
  template<class T> class SingletonFactory;
}

namespace dec_world_state
{

class WorldStateImpl
{

  friend class dec_utilities::SingletonFactory<WorldStateImpl>;

public:
  WorldStateImpl();
  virtual ~WorldStateImpl();

  /**
   * Get the latest state of an object. Returns false if object doesn't exist.
   */
  bool getObject(const std::string& name, dec_msgs::Object& object, const bool verbose = false);
  bool getObjectPose(const std::string& name, tf::Pose& pose, const bool verbose = false);

  void addObjects(const std::vector<dec_msgs::Object>& objects);
  void addObjects(const dec_msgs::Objects& objects);
  void addObject(const dec_msgs::Object& object);
  void addObject(const std::string& name, const tf::Pose& pose);
  void removeObjects(const dec_msgs::Objects& objects);
  void removeObject(const dec_msgs::Object& object);
  void removeObject(const std::string& name);
  void removeAllObjects();
  void getObjects(std::vector<dec_msgs::Object>& objects);

  void addCallback(boost::function<void (const ObjectState& state)> callback);
  int getNumSubscribers();

private:
  ros::NodeHandle node_handle_;
  ros::Subscriber object_sub_;
  ros::Publisher object_pub_;

  void objectCallbackEvent(const ros::MessageEvent<ObjectState const>& object_state_event);
  void objectCallback(const ObjectState& object_state);
  void publishObject(const dec_msgs::Object& object, int operation);

  typedef std::map<std::string, dec_msgs::Object> ObjectMap;
  ObjectMap object_map_;

  std::vector<boost::function<void (const ObjectState& state)> > callbacks_;

};

} /* namespace dec_world_state */
#endif /* WORLD_STATE_IMPL_H_ */
