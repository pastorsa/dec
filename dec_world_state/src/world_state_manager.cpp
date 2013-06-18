/*
 * world_state_manager.cpp
 *
 *  Created on: Jan 7, 2013
 *      Author: kalakris
 */

// #include <dec_utilities/rviz_marker_manager.h>
#include <dec_world_state/world_state.h>
#include <ros/package.h>
#include <boost/filesystem.hpp>
#include <conversions/ros_to_tf.h>
#include <conversions/tf_to_ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <dec_object_models/object_parameters.h>
#include <tf/transform_listener.h>
#include <dec_msgs/Shape.h>

using namespace conversions;

namespace dec_world_state
{

struct ObjectMapEntry
{
  std::string name;
  tf::Pose pose;
  bool is_being_edited;
  bool is_attached;
  std::string attached_to;
  tf::Pose relative_pose; // pose relative to attachment point
};

class WorldStateManager
{
public:
  WorldStateManager();
  virtual ~WorldStateManager();

  int run();

private:
  ros::NodeHandle node_handle_;
  ros::WallTimer timer_;

  WorldState world_state_;
  std::string dec_object_models_path_;
  dec_object_models::ObjectParameters object_parameters_;

  interactive_markers::InteractiveMarkerServer interactive_marker_server_;

  tf::TransformListener tf_listener_;

  typedef std::map<std::string, ObjectMapEntry> ObjectMap;
  ObjectMap object_map_;
  int prev_num_subscribers_;

  void objectCallback(const ObjectState& object_state);
  void timerCallback(const ros::WallTimerEvent& event);

  void removeObject(const ObjectState& object_state);
  void addObject(const ObjectState& object_state);
  void attachObject(const ObjectState& object_state);
  void detachObject(const ObjectState& object_state);

  void publishMarkers(const std::string& name, const std::string& frame_id);

  void interactiveMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  void getObjectMarkers(const std::string& name, std::vector<visualization_msgs::Marker>& markers);
  void collisionShapeToVizMarker(const dec_msgs::Shape &obj,
                                                    const std::vector<double> offsets,
                                                    visualization_msgs::Marker &mk);
  void updatePose(const std::string& name);

};

WorldStateManager::WorldStateManager():
    interactive_marker_server_("/world_state/interactive_markers")
{
  //object_sub_ = node_handle_.subscribe("/world_state/objects", 100, &WorldStateManager::objectCallback, this);
  dec_object_models_path_ = ros::package::getPath("dec_object_models");
  prev_num_subscribers_ = 0;
  world_state_.addCallback(boost::bind(&WorldStateManager::objectCallback, this, _1));
  timer_ = node_handle_.createWallTimer(ros::WallDuration(0.1), boost::bind(&WorldStateManager::timerCallback, this, _1));
}

WorldStateManager::~WorldStateManager()
{
}

void WorldStateManager::timerCallback(const ros::WallTimerEvent& event)
{
  bool publish_all = false;
  int num_subscribers = world_state_.getNumSubscribers();
  if (num_subscribers > prev_num_subscribers_)
  {
    ROS_INFO("WorldStateManager: Re-publishing all objects.");
    publish_all = true;
  }
  prev_num_subscribers_ = num_subscribers;

  for (ObjectMap::iterator it=object_map_.begin(); it!=object_map_.end(); ++it)
  {
    const std::string& name = it->first;
    const ObjectMapEntry& entry = it->second;

    if (publish_all)
    {
      dec_msgs::Object object;
      if (!world_state_.getObject(name, object))
      {
        ROS_ERROR("WorldStateManager: weird situation, WorldState doesn't have the object that I have: %s", name.c_str());
        continue;
      }
      convert(entry.pose, object.pose.pose.pose);
      world_state_.addObject(object);
      geometry_msgs::Pose pose_msg = object.pose.pose.pose;
      interactive_marker_server_.setPose(name, pose_msg);
      interactive_marker_server_.applyChanges();
    }
  }
}

void WorldStateManager::updatePose(const std::string& name)
{
  ObjectMap::iterator it = object_map_.find(name);
  if (it == object_map_.end())
    return;
  ObjectMapEntry& entry = it->second;
  if (!entry.is_attached)
    return;

  try
  {
    tf::StampedTransform stamped_tf;
    tf_listener_.lookupTransform("/BASE", entry.attached_to, ros::Time(0), stamped_tf);
    entry.pose = stamped_tf * entry.relative_pose;
  }
  catch (tf::TransformException& ex)
  {
    ROS_WARN("Couldn't lookup latest transform for attachment point: %s", entry.attached_to.c_str());
  }
}

void WorldStateManager::objectCallback(const ObjectState& object_state)
{
  if (object_state.operation == object_state.REMOVE)
  {
    removeObject(object_state);
  }
  else if (object_state.operation == object_state.ADD)
  {
    addObject(object_state);
  }
  else if (object_state.operation == object_state.ATTACH)
  {
    attachObject(object_state);
  }
  else if (object_state.operation == object_state.DETACH)
  {
    detachObject(object_state);
  }
}

void WorldStateManager::attachObject(const ObjectState& object_state)
{
  const std::string& name = object_state.object.name;
  ObjectMap::iterator it = object_map_.find(name);
  if (it == object_map_.end())
  {
    ROS_WARN("Tried to attach object %s which doesn't exist!", name.c_str());
    return;
  }
  ObjectMapEntry& entry = it->second;
  entry.is_attached = true;
  entry.attached_to = object_state.attached_to;

  // get pose of attachment point
  try
  {
    tf::StampedTransform stamped_tf;
    tf_listener_.lookupTransform("/BASE", entry.attached_to, ros::Time(0), stamped_tf);
    entry.relative_pose = stamped_tf.inverseTimes(entry.pose);
  }
  catch (tf::TransformException& ex)
  {
    ROS_WARN("Couldn't lookup latest transform for attachment point: %s", entry.attached_to.c_str());
  }

}

void WorldStateManager::detachObject(const ObjectState& object_state)
{
  const std::string& name = object_state.object.name;
  ObjectMap::iterator it = object_map_.find(name);
  if (it == object_map_.end())
  {
    ROS_WARN("Tried to detach object %s which doesn't exist!", name.c_str());
    return;
  }

  ObjectMapEntry& entry = it->second;
  if (!entry.is_attached)
  {
    ROS_WARN("Tried to detach object %s which isn't attached!", name.c_str());
  }

  // update pose before detaching
  updatePose(name);

  entry.is_attached = false;
  entry.attached_to = "";

  // publish
  geometry_msgs::Pose pose_msg;
  convert(entry.pose, pose_msg);
  interactive_marker_server_.setPose(name, pose_msg);
  interactive_marker_server_.applyChanges();
  world_state_.addObject(name, entry.pose);

}

void WorldStateManager::removeObject(const ObjectState& object_state)
{
  const std::string& name = object_state.object.name;
  ObjectMap::iterator it = object_map_.find(name);
  if (it != object_map_.end())
  {
    object_map_.erase(name);
  }

  interactive_marker_server_.erase(name);
  interactive_marker_server_.applyChanges();
}

void WorldStateManager::addObject(const ObjectState& object_state)
{
  const std::string& name = object_state.object.name;
  const std::string& frame_id = object_state.object.pose.header.frame_id;

  // first update the internal state
  ObjectMap::iterator it = object_map_.find(name);
  if (it == object_map_.end())
  {
    ObjectMapEntry entry;
    entry.name = name;
    convert(object_state.object.pose.pose.pose, entry.pose);
    entry.is_being_edited = false;
    entry.is_attached = false;
    entry.attached_to = "";
    object_map_[name] = entry;
  }
  else
  {
    // update the pose only
    convert(object_state.object.pose.pose.pose, it->second.pose);
  }

  // publish marker

  // if we're not already editing, create the markers
  visualization_msgs::InteractiveMarker int_marker;
  if (!interactive_marker_server_.get(name, int_marker))
  {
    publishMarkers(name, frame_id);
  }
  else // otherwise just update the pose
  {
    interactive_marker_server_.setPose(name, object_state.object.pose.pose.pose);
    interactive_marker_server_.applyChanges();
  }

}

void WorldStateManager::publishMarkers(const std::string& name, const std::string& frame_id)
{
  ObjectMap::iterator it = object_map_.find(name);
  const ObjectMapEntry& entry = it->second;
  ROS_ASSERT(it != object_map_.end());

  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = frame_id;
  convert(entry.pose, int_marker.pose);
  int_marker.scale = 0.5;
  int_marker.name = name;
  int_marker.description = "";

  visualization_msgs::InteractiveMarkerControl display_marker;
  display_marker.always_visible = true;
  getObjectMarkers(name, display_marker.markers);
  int_marker.controls.push_back(display_marker);

  visualization_msgs::InteractiveMarkerControl control;
  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "rotate_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  interactive_marker_server_.insert(int_marker);
  interactive_marker_server_.setCallback(int_marker.name,
                                         boost::bind(&WorldStateManager::interactiveMarkerFeedback, this, _1));
  interactive_marker_server_.applyChanges();

}

void WorldStateManager::interactiveMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  std::string name = feedback->marker_name;
  tf::Pose pose;
  convert(feedback->pose, pose);
  world_state_.addObject(name, pose);
}

void WorldStateManager::getObjectMarkers(const std::string& name, std::vector<visualization_msgs::Marker>& markers)
{
  // search dec_object_models for mesh file
  std::string post_fix = "/objects/" + name + "/" + name +".obj";
  std::string file_name = dec_object_models_path_ + post_fix;
  dec_msgs::Shape collision_shape;
  std::vector<double> collision_shape_offsets;
  //ROS_INFO("Searching for %s", file_name.c_str());
  if (boost::filesystem::exists(boost::filesystem::path(file_name)))
  {
    std::string package_rel_file_name = "package://dec_object_models" + post_fix;
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.mesh_resource = package_rel_file_name;
    convert(tf::Pose::getIdentity(), marker.pose);
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.5;
    // above is the default color
    bool mesh_disabled = false;
    object_parameters_.getMeshDisplayColor(name, marker.color, mesh_disabled);
    if (!mesh_disabled)
      markers.push_back(marker);
  }
  if (object_parameters_.getCollisionModel(name, collision_shape, collision_shape_offsets))
  { // add collision object if it exists
    visualization_msgs::Marker marker;
    collisionShapeToVizMarker(collision_shape, collision_shape_offsets, marker);
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 0.5;
    // above is the default color
    bool collision_disabled = false;
    object_parameters_.getCollisionDisplayColor(name, marker.color, collision_disabled);
    if (!collision_disabled)
      markers.push_back(marker);
  }

  visualization_msgs::Marker marker;
  /*
  // add pose arrows anyway
  const double pose_size = 0.2;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.points.resize(2);
  marker.points[0].x = 0.0;
  marker.points[0].y = 0.0;
  marker.points[0].z = 0.0;
  marker.scale.x = pose_size/10.0;
  marker.scale.y = pose_size/5.0;
  marker.scale.z = pose_size/10.0;
  marker.color.a = 0.9;
  convert(tf::Pose::getIdentity(), marker.pose);

  tf::Matrix3x3 identity;
  identity.setIdentity();
  for (int i=0; i<3; ++i)
  {
    tf::Vector3 tip = (identity.getColumn(i) * pose_size);
    marker.points[1].x = tip.getX();
    marker.points[1].y = tip.getY();
    marker.points[1].z = tip.getZ();
    marker.color.r = (i==0) ? 1.0 : 0.0;
    marker.color.g = (i==1) ? 1.0 : 0.0;
    marker.color.b = (i==2) ? 1.0 : 0.0;
    markers.push_back(marker);
  }
  */

  // add text
  //visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.text = name;
  convert(tf::Pose::getIdentity(), marker.pose);
  marker.pose.position.z -= 0.05;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 0.05;
  marker.color.r = 0.8;
  marker.color.g = 0.8;
  marker.color.b = 0.8;
  marker.color.a = 0.8;
  markers.push_back(marker);

}

void WorldStateManager::collisionShapeToVizMarker(const dec_msgs::Shape &obj,
                                                  const std::vector<double> offsets,
                                                  visualization_msgs::Marker &mk)
{
  switch (obj.type)
  {
    case dec_msgs::Shape::SPHERE:
      mk.type = visualization_msgs::Marker::SPHERE;
      mk.scale.x = mk.scale.y = mk.scale.z = obj.dimensions[0] * 2.0;
      break;

    case dec_msgs::Shape::BOX:
      mk.type = visualization_msgs::Marker::CUBE;
      mk.scale.x = obj.dimensions[0];
      mk.scale.y = obj.dimensions[1];
      mk.scale.z = obj.dimensions[2];
      break;

    case dec_msgs::Shape::CYLINDER:
      mk.type = visualization_msgs::Marker::CYLINDER;
      mk.scale.x = obj.dimensions[0] * 2.0;
      mk.scale.y = obj.dimensions[0] * 2.0;
      mk.scale.z = obj.dimensions[1];
      break;

    default:
      ROS_ERROR("Unknown object type: %d", (int)obj.type);
      break;
  }
  convert(tf::Pose::getIdentity(), mk.pose);
  if (offsets.size()==3)
  {
    mk.pose.position.x = offsets[0];
    mk.pose.position.y = offsets[1];
    mk.pose.position.z = offsets[2];
  }
}


int WorldStateManager::run()
{
  ros::spin();
  return 0;
}

} /* namespace dec_world_state */

int main(int argc, char** argv)
{
  ros::init(argc, argv, "WorldStateManager");

  dec_world_state::WorldStateManager manager;
  return manager.run();
}
