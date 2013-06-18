/*
 * object_parameters.h
 *
 *  Created on: Jan 24, 2011
 *      Author: kalakris
 */

#ifndef OBJECT_PARAMETERS_H_
#define OBJECT_PARAMETERS_H_

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>
#include <std_msgs/ColorRGBA.h>
#include <dec_msgs/Shape.h>
#include <vector>

namespace dec_object_models
{

class ObjectParameters
{
public:
  ObjectParameters();
  virtual ~ObjectParameters();

  bool getCollisionModel(const std::string& object_name, dec_msgs::Shape& model, std::vector<double>& offsets);
  bool getCollisionDisplayColor(const std::string& object_name, std_msgs::ColorRGBA& color, bool& disabled);
  bool getMeshDisplayColor(const std::string& object_name, std_msgs::ColorRGBA& color, bool& disabled);
  bool useObjectOrientation(const std::string& object_name);

private:
  ros::NodeHandle node_handle_;
  bool getDisplayColor(const std::string& param_name, const std::string& object_name, std_msgs::ColorRGBA& color, bool& disabled);

};

}

#endif /* OBJECT_PARAMETERS_H_ */
