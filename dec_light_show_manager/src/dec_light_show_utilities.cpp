/*
 * dec_light_show_utilities.cpp
 *
 *  Created on: Nov 7, 2012
 *      Author: pastor
 */

#include <dec_light_show_manager/dec_light_show_utilities.h>

namespace dec_light_show_manager
{

bool DecLightShowUtilities::getParam(XmlRpc::XmlRpcValue& config, const std::string& key, int& i)
{
  if (!config.hasMember(key))
    return false;
  XmlRpc::XmlRpcValue param = config[key];
  if (param.getType() != XmlRpc::XmlRpcValue::TypeInt)
    return false;
  i = param;
  return true;
}

bool DecLightShowUtilities::getParam(XmlRpc::XmlRpcValue& config, const std::string& key, unsigned int& ui)
{
  if (!config.hasMember(key))
    return false;
  XmlRpc::XmlRpcValue param = config[key];
  if (param.getType() != XmlRpc::XmlRpcValue::TypeInt)
    return false;
  int i = param;
  if (i < 0)
  {
    ROS_ERROR("Invalid parameter >%i< read from configuration. Must be positive.", key.c_str());
    return false;
  }
  ui = static_cast<unsigned int>(i);
  return true;
}

bool DecLightShowUtilities::getParam(XmlRpc::XmlRpcValue& config,
                                   const std::string& key,
                                   double& d)
{
  if (!config.hasMember(key))
    return false;
  XmlRpc::XmlRpcValue param = config[key];
  if (param.getType() != XmlRpc::XmlRpcValue::TypeDouble)
    return false;
  d = param;
  return true;
}

bool DecLightShowUtilities::getParam(XmlRpc::XmlRpcValue& config,
                                   const std::string& key,
                                   float& f)
{
  if (!config.hasMember(key))
    return false;
  XmlRpc::XmlRpcValue param = config[key];
  if (param.getType() != XmlRpc::XmlRpcValue::TypeDouble)
    return false;
  double d = param;
  f = static_cast<float>(d);
  return true;
}

bool DecLightShowUtilities::getParam(XmlRpc::XmlRpcValue& config,
                                   const std::string& key,
                                   std::string& str)
{
  if (!config.hasMember(key))
    return false;
  XmlRpc::XmlRpcValue param = config[key];
  if (param.getType() != XmlRpc::XmlRpcValue::TypeString)
    return false;
  str = std::string(param);
  return true;
}

bool DecLightShowUtilities::getParam(XmlRpc::XmlRpcValue& config,
                                   const std::string& key,
                                   std::vector<int>& i_array)
{
  if (!config.hasMember(key))
    return false;

  XmlRpc::XmlRpcValue i_array_xml = config[key];

  if (i_array_xml.getType() != XmlRpc::XmlRpcValue::TypeArray)
    return false;

  i_array.clear();
  for (int i = 0; i < i_array_xml.size(); ++i)
    i_array.push_back(int(i_array_xml[i]));

  return true;
}

bool DecLightShowUtilities::getParam(XmlRpc::XmlRpcValue& config,
                                   const std::string& key,
                                   std::vector<float>& f_array)
{
  if (!config.hasMember(key))
    return false;

  XmlRpc::XmlRpcValue f_array_xml = config[key];

  if (f_array_xml.getType() != XmlRpc::XmlRpcValue::TypeArray)
    return false;

  f_array.clear();
  for (int i = 0; i < f_array_xml.size(); ++i)
  {
    if (f_array_xml[i].getType() != XmlRpc::XmlRpcValue::TypeDouble)
      return false;
    double value = static_cast<double>(f_array_xml[i]);
    f_array.push_back(static_cast<float>(value));
  }

  return true;
}

bool DecLightShowUtilities::getParam(XmlRpc::XmlRpcValue& config,
                                   const std::string& key,
                                   std::vector<double>& d_array)
{
  if (!config.hasMember(key))
    return false;

  XmlRpc::XmlRpcValue d_array_xml = config[key];

  if (d_array_xml.getType() != XmlRpc::XmlRpcValue::TypeArray)
    return false;

  d_array.clear();
  for (int i = 0; i < d_array_xml.size(); ++i)
  {
    if (d_array_xml[i].getType() != XmlRpc::XmlRpcValue::TypeDouble)
      return false;
    d_array.push_back(double(d_array_xml[i]));
  }

  return true;
}

bool DecLightShowUtilities::getParam(XmlRpc::XmlRpcValue& config,
                                   const std::string& key,
                                   std::vector<std::string>& str_array)
{
  if (!config.hasMember(key))
    return false;

  XmlRpc::XmlRpcValue str_array_xml = config[key];

  if (str_array_xml.getType() != XmlRpc::XmlRpcValue::TypeArray)
    return false;

  str_array.clear();
  for (int i = 0; i < str_array_xml.size(); ++i)
    str_array.push_back(std::string(str_array_xml[i]));

  return true;
}

bool DecLightShowUtilities::getParam(XmlRpc::XmlRpcValue& config,
                                   const std::string& key,
                                   bool& b)
{
  if (!config.hasMember(key))
    return false;
  XmlRpc::XmlRpcValue param = config[key];
  if (param.getType() != XmlRpc::XmlRpcValue::TypeBoolean)
    return false;
  b = param;

  return true;
}

bool DecLightShowUtilities::loadList(XmlRpc::XmlRpcValue& config,
                                   std::vector<std::string>& names,
                                   const std::string& key,
                                   const bool verbose)
{
  if (!config.hasMember(key))
  {
    ROS_ERROR_COND(verbose, "DecLightShow didn't have '%s' parameter.", key.c_str());
    return false;
  }
  XmlRpc::XmlRpcValue array = config[key];
  if (array.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR_COND(verbose, "DecLightShow parameter '%s' must be an array.", key.c_str());
    return false;
  }
  names.clear();
  for (int i = 0; i < array.size(); ++i)
  {
    names.push_back(std::string(array[i]));
  }
  return true;
}

}
