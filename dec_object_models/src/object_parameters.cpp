/*
 * object_parameters.cpp
 *
 *  Created on: Jan 24, 2011
 *      Author: kalakris
 */

#include <dec_object_models/object_parameters.h>
#include <dec_utilities/param_server.h>
#include <tf/transform_datatypes.h>

namespace dec_object_models
{

ObjectParameters::ObjectParameters()
{
}

ObjectParameters::~ObjectParameters()
{
}

bool ObjectParameters::getCollisionModel(const std::string& object_name, dec_msgs::Shape& model, std::vector<double>& offsets)
{
  std::string type;
  if (!dec_utilities::read(node_handle_, "/objects/"+object_name+"/collision_model", type))
    return false;
  if (type=="BOX")
    model.type=model.BOX;
  else if (type=="CYLINDER")
    model.type=model.CYLINDER;
  else
    return false;
  if (!dec_utilities::read(node_handle_, "/objects/"+object_name+"/collision_model_dimensions", model.dimensions))
    return false;

  if (!dec_utilities::read(node_handle_, "/objects/"+object_name+"/collision_model_offsets", offsets, false))
  {
    offsets.clear();
    offsets.resize(3,0.0);
  }
  return true;
}


bool ObjectParameters::useObjectOrientation(const std::string& object_name)
{
  bool result = false;
  node_handle_.param("/objects/"+object_name+"/use_object_orientation", result, false);
  return result;
}

bool ObjectParameters::getDisplayColor(const std::string& param_name, const std::string& object_name, std_msgs::ColorRGBA& color, bool& disabled)
{
  std::vector<double> color_array;
  disabled = false;
  if (!dec_utilities::read(node_handle_, "/objects/"+object_name+"/"+param_name, color_array, false))
    return false;
  if (color_array.size() != 4)
  {
    ROS_WARN("Object %s display_color size must be 4!", object_name.c_str());
    return false;
  }
  color.r = color_array[0];
  color.g = color_array[1];
  color.b = color_array[2];
  color.a = color_array[3];
  if (color.a < 0.0001)
  {
    disabled = true;
  }
  return true;
}

bool ObjectParameters::getMeshDisplayColor(const std::string& object_name, std_msgs::ColorRGBA& color, bool& disabled)
{
  return getDisplayColor("mesh_display_color", object_name, color, disabled);
}

bool ObjectParameters::getCollisionDisplayColor(const std::string& object_name, std_msgs::ColorRGBA& color, bool& disabled)
{
  return getDisplayColor("collision_display_color", object_name, color, disabled);
}

}
