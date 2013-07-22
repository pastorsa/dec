/*
 * dec_light_show_utilities.h
 *
 *  Created on: Nov 7, 2012
 *      Author: pastor
 */

#ifndef DEC_LIGHT_SHOW_UTILITIES_H_
#define DEC_LIGHT_SHOW_UTILITIES_H_

#include <vector>
#include <string>
#include <ros/ros.h>

namespace dec_light_show_manager
{

/**
 * Provides static helper functions for light shows
 */
class DecLightShowUtilities
{
public:
  static bool getParam(XmlRpc::XmlRpcValue& config, const std::string& key, int& i);
  static bool getParam(XmlRpc::XmlRpcValue& config, const std::string& key, bool& i);
  static bool getParam(XmlRpc::XmlRpcValue& config, const std::string& key, double& d);
  static bool getParam(XmlRpc::XmlRpcValue& config, const std::string& key, float& f);
  static bool getParam(XmlRpc::XmlRpcValue& config, const std::string& key, std::string& str);
  static bool getParam(XmlRpc::XmlRpcValue& config, const std::string& key, std::vector<int>& i_array);
  static bool getParam(XmlRpc::XmlRpcValue& config, const std::string& key, std::vector<float>& d_array);
  static bool getParam(XmlRpc::XmlRpcValue& config, const std::string& key, std::vector<double>& d_array);
  static bool getParam(XmlRpc::XmlRpcValue& config, const std::string& key, std::vector<std::string>& str_array);

  /**
   * Loads a parameter from the given config element. Uses default value if not available.
   *
   * @param config (in) The XmlRpcValue to load the parameter from
   * @param key (in) The key for the parameter
   * @param param_val (out) The variable where the parameter value should be stored
   * @param default_val (in) The default value of the parameter
   * @return true if the parameter was found, false if the default value was used
   */
  template<typename T>
  static bool param(XmlRpc::XmlRpcValue& config, const std::string& key, T& param_val, const T& default_val)
  {
    if (getParam(config, key, param_val))
      return true;
    param_val = default_val;
    return false;
  }

  /*!
   * @param config
   * @param names
   * @param key
   * @param verbose
   * @return True on success, False otherwise
   */
  static bool loadList(XmlRpc::XmlRpcValue& config,
                       std::vector<std::string>& names,
                       const std::string& key,
                       const bool verbose = true);

};

}

#endif /* DEC_LIGHT_SHOW_UTILITIES_H_ */
