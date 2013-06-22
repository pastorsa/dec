/*
 * dec_data.cpp
 *
 *  Created on: Jun 17, 2013
 *      Author: pastor
 */

#include <iostream>
#include <sstream>
#include <fstream>

#include <ros/package.h>
#include <tf/transform_datatypes.h>
#include <conversions/ros_to_tf.h>
#include <conversions/tf_to_ros.h>
#include <dec_utilities/assert.h>
#include <dec_utilities/param_server.h>

#include <dec_visualization/dec_data.h>

using namespace std;
using namespace conversions;

namespace dec_visualization
{

DECData::DECData() : interaction_mode_(0), number_of_arduinos_(0),
    max_number_of_light_strips_per_arduino_(0), max_number_of_leds_per_light_strip_(0),
    max_number_of_sensors_per_arduino_(0), initialized_(false)
{
};


bool DECData::init(ros::NodeHandle node_handle)
{
  ROS_VERIFY(dec_utilities::read(node_handle, "number_of_arduinos", number_of_arduinos_));
  ROS_ASSERT_MSG(number_of_arduinos_ > 0 && number_of_arduinos_ <= 30,
                 "Number of arduinos specified >%i< is invalid. It need to be within [1..30].", number_of_arduinos_);
  ROS_VERIFY(dec_utilities::read(node_handle, "max_number_of_sensors_per_arduino", max_number_of_sensors_per_arduino_));
  ROS_ASSERT_MSG(max_number_of_sensors_per_arduino_ > 0 && max_number_of_sensors_per_arduino_ <= 255,
                 "Maximum number of sensors specified >%i< is invalid.", max_number_of_sensors_per_arduino_);
  ROS_VERIFY(dec_utilities::read(node_handle, "max_number_of_light_strips_per_arduino", max_number_of_light_strips_per_arduino_));
  ROS_ASSERT_MSG(max_number_of_light_strips_per_arduino_ > 0 && max_number_of_light_strips_per_arduino_ <= 255,
                 "Maximum number of light strips specified >%i< is invalid.", max_number_of_light_strips_per_arduino_);
  ROS_VERIFY(dec_utilities::read(node_handle, "max_number_of_leds_per_light_strip", max_number_of_leds_per_light_strip_));
  ROS_ASSERT_MSG(max_number_of_leds_per_light_strip_ > 0 && max_number_of_leds_per_light_strip_ <= 255,
                 "Maximum number of leds per light strip specified >%i< is invalid.", max_number_of_leds_per_light_strip_);

  ROS_VERIFY(dec_utilities::read(node_handle, "interaction_mode", interaction_mode_));

  ROS_VERIFY(read(node_handle, "node_positions", node_positions_));
  ROS_ASSERT_MSG(!node_positions_.empty(), "No node positions specified. Structure must contain at least one node.");

  int offset_node_index = 0;
  ROS_VERIFY(dec_utilities::read(node_handle, "offset_node_index", offset_node_index));
  ROS_ASSERT_MSG(offset_node_index >= 0 && offset_node_index < (int)node_positions_.size(),
             "Offset node index >%i< needs to be within [0..%i].", offset_node_index, (int)node_positions_.size()-1);
  offsetNodePositions(node_positions_, offset_node_index);

  ROS_VERIFY(dec_utilities::read(node_handle, "light_nodes", light_nodes_));
  light_node_positions_.clear();
  for (unsigned int i = 0; i < light_nodes_.size(); ++i)
  {
    ROS_ASSERT_MSG(light_nodes_[i] >= 0 && light_nodes_[i] < (int)node_positions_.size(),
                   "Invalid light node specified >%i<. Must be within [0..%i].", light_nodes_[i], (int)node_positions_.size()-1);
    light_node_positions_.push_back(node_positions_[light_nodes_[i]]);
  }

  ROS_VERIFY(read(node_handle, "beams", beams_));
  ROS_VERIFY(read(node_handle, "light_beams", light_beams_));
  ROS_VERIFY(read(node_handle, "sensors", sensors_));

  ROS_VERIFY(read(node_handle, "light_beam_connections", light_beam_connections_, light_beam_index_counter_,
                  arduino_to_light_beam_map_, light_beams_.size()));
  ROS_VERIFY(read(node_handle, "light_node_connections", light_node_connections_, light_node_index_counter_,
                  arduino_to_light_node_map_, light_nodes_.size()));
  ROS_ASSERT(arduino_to_light_beam_map_.size() == arduino_to_light_node_map_.size() &&
             (int)arduino_to_light_beam_map_.size() == number_of_arduinos_);
  for (int i = 0; i < number_of_arduinos_; ++i)
  {
    ROS_ASSERT_MSG(static_cast<int>(arduino_to_light_beam_map_[i].size()) + static_cast<int>(arduino_to_light_node_map_[i].size())
               < max_number_of_light_strips_per_arduino_, "Number of lights (beams >%i< and nodes >%i<) exceed specified limit >%i<.",
               (int)arduino_to_light_beam_map_[i].size(), (int)arduino_to_light_node_map_[i].size(),
               max_number_of_light_strips_per_arduino_);
  }

  ROS_VERIFY(read(node_handle, "sensor_connections", sensor_connections_, sensor_index_counter_, arduino_to_sensor_map_, sensors_.size()));
  for (unsigned int i = 0; i < arduino_to_light_beam_map_.size(); ++i)
  {
    ROS_ASSERT(static_cast<int>(arduino_to_sensor_map_[i].size()) < max_number_of_sensors_per_arduino_);
  }

  ROS_VERIFY(dec_utilities::read(node_handle, "num_leds_of_each_light_beam", num_leds_of_each_light_beam_));
  ROS_ASSERT_MSG(num_leds_of_each_light_beam_.size() == light_beams_.size(),
                 "Number of LEDs for each light beam is incorrect >%i<. It should equal the number of light beams >%i<.",
                 (int)num_leds_of_each_light_beam_.size(), (int)light_beams_.size());
  ROS_VERIFY(dec_utilities::read(node_handle, "num_leds_of_each_light_node", num_leds_of_each_light_node_));
  ROS_ASSERT_MSG(num_leds_of_each_light_node_.size() == light_nodes_.size(),
                 "Number of LEDs for each light node is incorrect >%i<. It should equal the number of light nodes >%i<.",
                 (int)num_leds_of_each_light_node_.size(), (int)light_nodes_.size());

  bool generate_configuration_file = false;
  ROS_VERIFY(dec_utilities::read(node_handle, "generate_configuration_file", generate_configuration_file));
  if (generate_configuration_file)
  {
    std::string path = ros::package::getPath("DEC");
    dec_utilities::appendTrailingSlash(path);
    generateConfigurationFile(path + "dec_config.h");
  }
  bool generate_structure_file = false;
  ROS_VERIFY(dec_utilities::read(node_handle, "generate_structure_file", generate_structure_file));
  if (generate_structure_file)
  {
    std::string path = ros::package::getPath("DEC");
    dec_utilities::appendTrailingSlash(path);
    generateStructureFile(path + "../../dec_controller/dec_structure.h");
  }

  const int NUM_ENTRIES_PER_ARDUINO =
      + NUM_ENTRIES_FOR_ARDUINO_LEVEL
      + max_number_of_sensors_per_arduino_
      + 4 * max_number_of_light_strips_per_arduino_; // for RGBA

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
                   array_name.c_str(), (int)i, values[i], number_of_arduinos_-1);
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
    ROS_ASSERT_MSG((values.first >= 0) && (values.first < (int)node_positions_.size())
        && (values.second >= 0) && (values.second < (int)node_positions_.size()),
        "Invalid >%s< index read at row %i: from >%i< to >%i<. It must be within [0, %i].",
                array_name.c_str(), i, values.first, values.second, (int)(node_positions_.size()-1));
    ROS_ASSERT_MSG(values.first != values.second, "Invalid beam read row %i. Node indices must differ.", i);

    nodes.push_back(values);
  }
  return true;
}

void DECData::offsetNodePositions(std::vector<geometry_msgs::Point>& node_positions,
                                  const int node_index)
{
  ROS_ASSERT_MSG(!node_positions.empty(), "Empty list of nodes provided, cannot offset.");
  ROS_ASSERT_MSG(node_index >= 0 && node_index < node_positions.size(),
                 "Provided node index >%i< is invalid. Need to be within [0..%i]. Cannot offset node positions.",
                 node_index, node_positions.size()-1);

  double offset_x = node_positions[node_index].x;
  double offset_y = node_positions[node_index].y;
  double offset_z = node_positions[node_index].z;
  for (unsigned int i = 0; i < node_positions.size(); ++i)
  {
    node_positions[i].x -= offset_x;
    node_positions[i].y -= offset_y;
    node_positions[i].z -= offset_z;
  }
}

int DECData::getNumArduinos() const
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

bool DECData::generateConfigurationFile(const std::string& abs_file_name)
{
  std::ofstream header_file;
  header_file.open(abs_file_name.c_str(), std::ios::out);
  ROS_ASSERT_MSG(header_file.is_open(), "Problem when opening header file >%s<.", abs_file_name.c_str());

  header_file << "// This configuration file is generated from dec_visualization/config/config.yaml.\n";
  header_file << "// Please do not edit.\n";
  header_file << "// Instead edit dec_visualization/config/structure.yaml and regenerate this file.\n\n";

  header_file << "#ifndef _DEC_CONFIG_H" << endl;
  header_file << "#define _DEC_CONFIG_H" << endl;

  header_file << "static const uint8_t DEC_NUM_NODES  = " << number_of_arduinos_ << ";\n";
  header_file << "static const uint8_t DEC_MAX_NUMBER_OF_SENSORS_PER_NODE = " << max_number_of_sensors_per_arduino_ << ";\n";
  header_file << "static const uint8_t DEC_MAX_NUMBER_OF_LED_STRIPS_PER_NODE = " << max_number_of_light_strips_per_arduino_ << ";\n";
  header_file << "static const uint8_t DEC_MAX_NUMBER_OF_LEDS_PER_LIGHT_STRIP = " << max_number_of_leds_per_light_strip_ << ";\n";

  header_file << "#endif // _DEC_CONFIG_H" << endl;
  header_file.close();

  return true;
}

bool DECData::generateStructureFile(const std::string& abs_file_name)
{
  std::ofstream header_file;
  header_file.open(abs_file_name.c_str(), std::ios::out);
  ROS_ASSERT_MSG(header_file.is_open(), "Problem when opening header file >%s<.", abs_file_name.c_str());

  header_file << "// This configuration file is generated from dec_visualization/config/structure.yaml.\n";
  header_file << "// Please do not edit.\n";
  header_file << "// Instead edit dec_visualization/config/structure.yaml and regenerate this file.\n\n";

  header_file << "#ifndef _DEC_STRUCTURE_H" << endl;
  header_file << "#define _DEC_STRUCTURE_H" << endl;
  header_file << endl;

  header_file << "uint8_t num_led_strips_per_arduino[" << number_of_arduinos_ << "];\n";
  for (int i = 0; i < number_of_arduinos_; ++i)
  {
    int num_light_strips = 0;
    num_light_strips += (int)arduino_to_light_beam_map_[i].size();
    num_light_strips += (int)arduino_to_light_node_map_[i].size();
    header_file << "num_led_strips_per_arduino[" << i << "] = " << num_light_strips << ";\n";
  }
  header_file << endl;



  header_file << "#endif // _DEC_STRUCTURE_H" << endl;
  header_file.close();

  return true;
}

}
