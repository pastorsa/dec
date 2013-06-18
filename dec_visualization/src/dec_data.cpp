/*
 * dec_data.cpp
 *
 *  Created on: Jun 17, 2013
 *      Author: pastor
 */

#include <tf/transform_datatypes.h>
#include <conversions/ros_to_tf.h>
#include <conversions/tf_to_ros.h>
#include <dec_utilities/assert.h>
#include <dec_utilities/param_server.h>

#include <dec_visualization/dec_data.h>

namespace dec_visualization
{

bool DECData::init(ros::NodeHandle node_handle)
{
  ROS_VERIFY(dec_utilities::read(node_handle, "number_of_arduinos", number_of_arduinos_));
  ROS_ASSERT(number_of_arduinos_ > 0);
  ROS_VERIFY(dec_utilities::read(node_handle, "max_number_of_sensors_per_arduino", max_number_of_sensors_per_arduino_));
  ROS_ASSERT(max_number_of_sensors_per_arduino_ > 0);
  ROS_VERIFY(dec_utilities::read(node_handle, "max_number_of_lights_per_arduino", max_number_of_lights_per_arduino_));
  ROS_ASSERT(max_number_of_lights_per_arduino_ > 0);

  ROS_VERIFY(dec_utilities::read(node_handle, "interaction_mode", interaction_mode_));

  ROS_VERIFY(read(node_handle, "node_positions", node_positions_));
  ROS_VERIFY(dec_utilities::read(node_handle, "light_nodes", light_nodes_));
  light_node_positions_.clear();
  for (unsigned int i = 0; i < light_nodes_.size(); ++i)
  {
    ROS_ASSERT_MSG(light_nodes_[i] >= 0 && light_nodes_[i] < (int)node_positions_.size(),
                   "Invalid light node specified >%i<. Must be within [0..%i].", light_nodes_[i], (int)node_positions_.size());
    light_node_positions_.push_back(node_positions_[light_nodes_[i]]);
  }

  ROS_VERIFY(read(node_handle, "beams", beams_));
  ROS_VERIFY(read(node_handle, "light_beams", light_beams_));
  ROS_VERIFY(read(node_handle, "sensors", sensors_));

  ROS_VERIFY(read(node_handle, "light_beam_connections", light_beam_connections_, light_beam_index_counter_, arduino_to_light_beam_map_, light_beams_.size()));
  ROS_VERIFY(read(node_handle, "light_node_connections", light_node_connections_, light_node_index_counter_, arduino_to_light_node_map_, light_nodes_.size()));
  ROS_ASSERT(arduino_to_light_beam_map_.size() == arduino_to_light_node_map_.size() &&
             (int)arduino_to_light_beam_map_.size() == number_of_arduinos_);
  for (int i = 0; i < number_of_arduinos_; ++i)
  {
    ROS_ASSERT_MSG(static_cast<int>(arduino_to_light_beam_map_[i].size()) + static_cast<int>(arduino_to_light_node_map_[i].size())
               < max_number_of_lights_per_arduino_, "Number of lights (beams >%i< and nodes >%i<) exceed specified limit >%i<.",
               (int)arduino_to_light_beam_map_[i].size(), (int)arduino_to_light_node_map_[i].size(),
               max_number_of_lights_per_arduino_);
  }

  ROS_VERIFY(read(node_handle, "sensor_connections", sensor_connections_, sensor_index_counter_, arduino_to_sensor_map_, sensors_.size()));
  for (unsigned int i = 0; i < arduino_to_light_beam_map_.size(); ++i)
  {
    ROS_ASSERT(static_cast<int>(arduino_to_sensor_map_[i].size()) < max_number_of_sensors_per_arduino_);
  }


  const int NUM_ENTRIES_PER_ARDUINO =
      + NUM_ENTRIES_FOR_ARDUINO_LEVEL
      + max_number_of_sensors_per_arduino_
      + 4 * max_number_of_lights_per_arduino_; // for RGBA

  data_ = Eigen::MatrixXi::Zero(number_of_arduinos_, NUM_ENTRIES_PER_ARDUINO);
  return (initialized_ = true);
}

bool DECData::read(ros::NodeHandle node_handle,
          const::std::string& array_name,
          std::vector<int>& values,
          std::vector<int>& index_values,
          std::vector<std::vector<int> >& map,
          const unsigned int& num)
{
  ROS_VERIFY(dec_utilities::read(node_handle, array_name, values));
  ROS_ASSERT_MSG(values.size() == num, "Number of >%s< >%i< must equal >%i<.", array_name.c_str(), (int)values.size(), (int)num);

  for (unsigned int i = 0; i < values.size(); ++i)
  {
    ROS_ASSERT_MSG(values[i] >= 0 && values[i] < number_of_arduinos_,
                   ">%s< >%i< is >%i< and therefore invalid. Must be within [0, %i]",
                   array_name.c_str(), (int)i, values[i], number_of_arduinos_);
  }

  index_values.resize(values.size(), -1);
  map.resize(number_of_arduinos_);
  std::vector<int> counts(number_of_arduinos_, 0);
  for (int i = 0; i < number_of_arduinos_; ++i)
  {
    std::vector<int> v;
    for (unsigned int j = 0; j < values.size(); ++j)
    {
      if (i == values[j])
      {
        v.push_back(values[j]);
        index_values[j] = counts[i];
        counts[i]++;
      }
    }
    map[i] = v;
  }

  for (unsigned int i = 0; i < index_values.size(); ++i)
  {
    ROS_ASSERT_MSG(index_values[i] >= 0, "Invalid index value >%i<.", index_values[i]);
  }

  return true;
}

bool DECData::read(ros::NodeHandle node_handle,
                        const std::string& array_name,
                            std::vector<geometry_msgs::Point>& array)
{
  XmlRpc::XmlRpcValue rpc_values;
  if (!node_handle.getParam(array_name, rpc_values))
  {
    ROS_ERROR("Could not read from >%s/%s<.", node_handle.getNamespace().c_str(), array_name.c_str());
    return false;
  }

  array.clear();
  for (int i = 0; i < rpc_values.size(); ++i)
  {
    if (!rpc_values[i].hasMember("position"))
    {
      ROS_ERROR("Each arm configuration must contain a field >position<.");
      return false;
    }
    XmlRpc::XmlRpcValue rpc_value = rpc_values[i]["position"];
    if (rpc_value.size() != 3)
    {
      ROS_ERROR("Number of joint_configuration >%i< is wrong. It should be >3<.", rpc_value.size());
      return false;
    }
    std::vector<double> values;
    for (int j = 0; j < 3; ++j)
    {
      if (rpc_value[j].getType() != XmlRpc::XmlRpcValue::TypeDouble && rpc_value[j].getType() != XmlRpc::XmlRpcValue::TypeInt)
      {
        ROS_ERROR("Value must either be of type integer or double.");
        return false;
      }
      values.push_back(static_cast<double> (rpc_value[j]));
    }
    geometry_msgs::Point point;
    point.x = values[0];
    point.y = values[1];
    point.z = values[2];
    array.push_back(point);
  }
  return true;
}

bool DECData::read(ros::NodeHandle node_handle,
                        const std::string& array_name,
                            std::vector<std::pair<int, int> >& nodes)
{
  XmlRpc::XmlRpcValue rpc_values;
  if (!node_handle.getParam(array_name, rpc_values))
  {
    ROS_ERROR("Could not read from >%s/%s<.", node_handle.getNamespace().c_str(), array_name.c_str());
    return false;
  }

  nodes.clear();
  for (int i = 0; i < rpc_values.size(); ++i)
  {
    if (!rpc_values[i].hasMember("nodes"))
    {
      ROS_ERROR("Each arm configuration must contain a field >nodes<.");
      return false;
    }
    XmlRpc::XmlRpcValue rpc_value = rpc_values[i]["nodes"];
    if (rpc_value.size() != 2)
    {
      ROS_ERROR("Number of joint_configuration >%i< is wrong. It should be >2<.", rpc_value.size());
      return false;
    }
    if ((rpc_value[0].getType() != XmlRpc::XmlRpcValue::TypeInt)
        || (rpc_value[1].getType() != XmlRpc::XmlRpcValue::TypeInt))
    {
      ROS_ERROR("Values must either be of type integer.");
      return false;
    }
    std::pair<int, int> values(rpc_value[0], rpc_value[1]);
    if ( (values.first < 0) || (values.first >= (int)node_positions_.size()) ||
        (values.second < 0) || (values.second >= (int)node_positions_.size()) )
    {
      ROS_ERROR("Invalid >%s< read >%i< >%i<. It must be within [0, %i].",
                array_name.c_str(), values.first, values.second, (int)node_positions_.size());
      return false;
    }
    nodes.push_back(values);
  }
  return true;
}

int DECData::getNumArduinos()
{
  ROS_ASSERT(initialized_);
  return number_of_arduinos_;
}

int DECData::getSensorValue(const int arduino_index,
                    const int sensor_index)
{
  return data_(arduino_index, NUM_ENTRIES_FOR_ARDUINO_LEVEL + sensor_index);
}

void DECData::addSensorValue(const int arduino_index,
                             const int sensor_index,
                             const int sensor_value)
{
  data_(arduino_index, NUM_ENTRIES_FOR_ARDUINO_LEVEL + sensor_index) += sensor_value;
}

void DECData::setSensorValue(const int arduino_index,
                                const int sensor_index,
                                const int sensor_value)
{
  data_(arduino_index, NUM_ENTRIES_FOR_ARDUINO_LEVEL + sensor_index) = sensor_value;
}

}
