/*
 * world_state.h
 *
 *  Created on: Jan 8, 2013
 *      Author: kalakris
 */

#ifndef WORLD_STATE_H_
#define WORLD_STATE_H_

#include <boost/function.hpp>
#include <dec_msgs/Objects.h>
#include <dec_world_state/ObjectState.h>
#include <tf/transform_datatypes.h>

namespace dec_world_state
{

class WorldStateImpl;
typedef boost::shared_ptr<WorldStateImpl> WorldStateImplPtr;

class WorldState
{
public:
  WorldState();
  virtual ~WorldState();

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

  /**
   * Returns the number of subscribers on the objects topic
   */
  int getNumSubscribers();

  void addCallback(boost::function<void (const ObjectState& state)> callback);

private:
  WorldStateImplPtr impl_;
};

} /* namespace dec_world_state */
#endif /* WORLD_STATE_H_ */
