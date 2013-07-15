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

#include <dec_light_show_manager/dec_data.h>

using namespace std;
using namespace conversions;

namespace dec_light_show_manager
{

DecData::DecData() : interaction_mode_(0), number_of_teensys_(0),
    max_number_of_light_strips_per_teensy_(0), max_number_of_leds_per_light_strip_(0),
    max_number_of_sensors_per_teensy_(0), enable_communication_(false), ros_time_sec_(0.0),
    initialized_(false)
{
};

bool DecData::initialize(ros::NodeHandle node_handle)
{
  ROS_VERIFY(dec_utilities::read(node_handle, "number_of_teensys", number_of_teensys_));
  ROS_ASSERT_MSG(number_of_teensys_ > 0 && number_of_teensys_ <= 255,
                 "Number of teensys specified >%i< is invalid. It need to be within [1..255].", number_of_teensys_);

  ROS_VERIFY(dec_utilities::read(node_handle, "sensor_pins", sensor_pins_));
  for (unsigned int i = 0; i < sensor_pins_.size(); ++i)
  {
    const int SENSOR_PINS[29] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 17, 18, 19, 20, 21, 22, 23,
                                 28, 29, 30, 31, 32, 33, 34, 35, 36, 37}; // Teensy++ 2.0 digital pins that are not PWM capable
    bool found = false;
    for (unsigned int j = 0; !found && j < 9; ++j)
    {
      if (SENSOR_PINS[j] == sensor_pins_[i])
      {
        found = true;
      }
    }
    ROS_ASSERT_MSG(found, "Invalid sensor pin >%i< specified. This pin is not a valid digital pin (excluding PWM pins) on a Teensy++ 2.0 !", sensor_pins_[i]);
  }
  max_number_of_sensors_per_teensy_ = (int)sensor_pins_.size();
  ROS_ASSERT_MSG(max_number_of_sensors_per_teensy_ > 0 && max_number_of_sensors_per_teensy_ <= 255,
                 "Maximum number of sensors specified >%i< is invalid.", max_number_of_sensors_per_teensy_);

  ROS_VERIFY(dec_utilities::read(node_handle, "light_pins", light_pins_));
  for (unsigned int i = 0; i < light_pins_.size(); ++i)
  {
    const int PWM_PINS[9] = {0, 1, 14, 15, 16, 24, 25, 26, 27}; // Teensy++ 2.0 PWM pins
    bool found = false;
    for (unsigned int j = 0; !found && j < 9; ++j)
    {
      if (PWM_PINS[j] == light_pins_[i])
      {
        found = true;
      }
    }
    ROS_ASSERT_MSG(found, "Invalid light pin >%i< specified. This pin is not PWM capable on a Teensy++ 2.0 !", light_pins_[i]);
  }
  max_number_of_light_strips_per_teensy_ = (int)light_pins_.size();
  ROS_ASSERT_MSG(max_number_of_light_strips_per_teensy_ > 0 && max_number_of_light_strips_per_teensy_ <= 255,
                 "Maximum number of light strips specified >%i< is invalid.", max_number_of_light_strips_per_teensy_);

  ROS_VERIFY(dec_utilities::read(node_handle, "max_number_of_leds_per_light_strip", max_number_of_leds_per_light_strip_));
  ROS_ASSERT_MSG(max_number_of_leds_per_light_strip_ > 0 && max_number_of_leds_per_light_strip_ <= 255,
                 "Maximum number of LEDs per light strip specified >%i< is invalid.", max_number_of_leds_per_light_strip_);

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

  ROS_VERIFY(read(node_handle, "light_beam_connections", light_beam_connections_, light_beam_index_counter_, teensy_to_light_beam_map_, light_beams_.size()));
  ROS_VERIFY(read(node_handle, "light_node_connections", light_node_connections_, light_node_index_counter_, teensy_to_light_node_map_, light_nodes_.size()));
  ROS_VERIFY(read(node_handle, "sensor_connections", sensor_connections_, sensor_index_counter_, teensy_to_sensor_map_, sensors_.size()));
  for (int i = 0; i < number_of_teensys_; ++i)
  {
    ROS_ASSERT_MSG(static_cast<int>(teensy_to_light_beam_map_[i].size()) + static_cast<int>(teensy_to_light_node_map_[i].size())
               <= max_number_of_light_strips_per_teensy_, "Number of lights (beams >%i< and nodes >%i<) exceed specified limit >%i<.",
               (int)teensy_to_light_beam_map_[i].size(), (int)teensy_to_light_node_map_[i].size(), max_number_of_light_strips_per_teensy_);
    ROS_ASSERT_MSG(static_cast<int>(teensy_to_sensor_map_[i].size()) <= max_number_of_sensors_per_teensy_,
                   "Number of sensors per teensy >%i< exceeds the limit >%i< of allowed sensors per teensy.",
                   (int)teensy_to_sensor_map_[i].size(), max_number_of_sensors_per_teensy_);
  }

  ROS_VERIFY(dec_utilities::read(node_handle, "num_leds_of_each_light_beam", num_leds_of_each_light_beam_));
  ROS_ASSERT_MSG(num_leds_of_each_light_beam_.size() == light_beams_.size(),
                 "Number of LEDs for each light beam is incorrect >%i<. It should equal the number of light beams >%i<.",
                 (int)num_leds_of_each_light_beam_.size(), (int)light_beams_.size());
  ROS_VERIFY(dec_utilities::read(node_handle, "num_leds_of_each_light_node", num_leds_of_each_light_node_));
  ROS_ASSERT_MSG(num_leds_of_each_light_node_.size() == light_nodes_.size(),
                 "Number of LEDs for each light node is incorrect >%i<. It should equal the number of light nodes >%i<.",
                 (int)num_leds_of_each_light_node_.size(), (int)light_nodes_.size());

  light_data_.resize((size_t)number_of_teensys_);

  const int NUM_ENTRIES_PER_ARDUINO =
      + NUM_ENTRIES_FOR_TEENSY_LEVEL
      + max_number_of_sensors_per_teensy_
      + 4 * max_number_of_light_strips_per_teensy_; // for RGBA

  data_ = Eigen::MatrixXi::Zero(number_of_teensys_, NUM_ENTRIES_PER_ARDUINO);
  ROS_INFO("Created >%i< by >%i< data matrix.", (int)data_.rows(), (int)data_.cols());

  // Lastly, generate the configuration file
  bool generate_configuration_file = false;
  ROS_VERIFY(dec_utilities::read(node_handle, "generate_configuration_file", generate_configuration_file));
  if (generate_configuration_file)
  {
    std::string path = ros::package::getPath("dec_communication");
    dec_utilities::appendTrailingSlash(path);
    generateConfigurationFile(path + "include/dec_communication/DEC_config.h");
  }
  bool generate_structure_file = false;
  ROS_VERIFY(dec_utilities::read(node_handle, "generate_structure_file", generate_structure_file));
  if (generate_structure_file)
  {
    std::string path = ros::package::getPath("dec_communication");
    dec_utilities::appendTrailingSlash(path);
    // generateStructureFile(path + "DEC_structure_teensy.h", "PROGMEM ", "prog_");
    generateStructureFile(path + "include/dec_communication/DEC_structure.h");
  }

  ROS_VERIFY(dec_utilities::read(node_handle, "enable_communication", enable_communication_));

  return (initialized_ = true);
}

bool DecData::read(ros::NodeHandle node_handle,
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
    ROS_ASSERT_MSG(values[i] >= 0 && values[i] < number_of_teensys_,
                   ">%s< >%i< is >%i< and therefore invalid. Must be within [0, %i]",
                   array_name.c_str(), (int)i, values[i], number_of_teensys_-1);
  }

  index_values.resize(values.size(), -1);
  map.resize(number_of_teensys_);
  std::vector<int> counts(number_of_teensys_, 0);
  for (int i = 0; i < number_of_teensys_; ++i)
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

bool DecData::read(ros::NodeHandle node_handle,
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

bool DecData::read(ros::NodeHandle node_handle,
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

void DecData::offsetNodePositions(std::vector<geometry_msgs::Point>& node_positions,
                                  const int node_index)
{
  ROS_ASSERT_MSG(!node_positions.empty(), "Empty list of nodes provided, cannot offset.");
  ROS_ASSERT_MSG(node_index >= 0 && node_index < (int)node_positions.size(),
                 "Provided node index >%i< is invalid. Need to be within [0..%i]. Cannot offset node positions.",
                 node_index, int(node_positions.size())-1);

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

int DecData::getNumTeensys() const
{
  ROS_ASSERT(initialized_);
  return number_of_teensys_;
}

int DecData::getSensorValue(const int teensy_index,
                    const int sensor_index)
{
  return data_(teensy_index, NUM_ENTRIES_FOR_TEENSY_LEVEL + sensor_index);
}

void DecData::addSensorValue(const int teensy_index,
                             const int sensor_index,
                             const int sensor_value)
{
  data_(teensy_index, NUM_ENTRIES_FOR_TEENSY_LEVEL + sensor_index) += sensor_value;
}

void DecData::setSensorValue(const int teensy_index,
                                const int sensor_index,
                                const int sensor_value)
{
  data_(teensy_index, NUM_ENTRIES_FOR_TEENSY_LEVEL + sensor_index) = sensor_value;
}

bool DecData::generateConfigurationFile(const std::string& abs_file_name)
{
  std::ofstream header_file;
  header_file.open(abs_file_name.c_str(), std::ios::out);
  ROS_ASSERT_MSG(header_file.is_open(), "Problem when opening header file >%s<.", abs_file_name.c_str());

  header_file << "// This configuration file is generated from dec_visualization/config/config.yaml.\n";
  header_file << "// Please do not edit.\n";
  header_file << "// Instead edit dec_visualization/config/structure.yaml and regenerate this file.\n\n";

  header_file << "#ifndef _DEC_CONFIG_H" << endl;
  header_file << "#define _DEC_CONFIG_H\n" << endl;

  header_file << "// Number of teensys in the structure.\n";
  header_file << "#define DEC_NUMBER_OF_ARDUINOS (uint8_t)" << number_of_teensys_ << "\n\n";
  header_file << "// Maximum number of sensors, light strips, and LEDs per light strip. (To statically allocate memory)\n";
  header_file << "#define DEC_MAX_NUMBER_OF_SENSORS_PER_NODE (uint8_t)" << max_number_of_sensors_per_teensy_ << "\n";
  header_file << "#define DEC_MAX_NUMBER_OF_LED_STRIPS_PER_NODE (uint8_t)" << max_number_of_light_strips_per_teensy_ << "\n";
  header_file << "#define DEC_MAX_NUMBER_OF_LEDS_PER_LIGHT_STRIP (uint8_t)" << max_number_of_leds_per_light_strip_ << "\n";

  header_file << "\n#endif // _DEC_CONFIG_H" << endl;
  header_file.close();

  return true;
}

std::vector<std::vector<int> > DecData::getTeensyToSensorsDistances(const int& teensy_id) const
{
  ROS_ASSERT(teensy_id >= 0 && teensy_id < number_of_teensys_);

  std::vector<std::vector<double> > teensy_to_sensor_distances;
  for (int i = 0; i < number_of_teensys_; ++i)
  {
    std::vector<double> distances;
    for (unsigned int j = 0; j < teensy_to_sensor_map_[i].size(); ++j)
    {
      tf::Vector3 point_1, point_2;
      convert(node_positions_[sensors_[j].first], point_1);
      convert(node_positions_[sensors_[j].second], point_2);
      // tf::Vector3 sensor_position = (point_1 + point_2) / static_cast<double>(2.0);

      double distance = fabs(tf::Vector3(point_1 - point_2).length());
      distances.push_back(distance);
    }
    teensy_to_sensor_distances.push_back(distances);
  }

  // compute maximum and scale accordingly
  double max = -1.0;
  for (unsigned int i = 0; i < teensy_to_sensor_distances.size(); ++i)
  {
    for (unsigned int j = 0; j < teensy_to_sensor_distances[i].size(); ++j)
    {
      if (teensy_to_sensor_distances[i][j] > max)
      {
        max = teensy_to_sensor_distances[i][j];
      }
    }
  }
  ROS_INFO("Maximum distance is >%.2f< meters.", max);
  ROS_ASSERT(max > 0.0);
  const double RATIO = static_cast<double>(1.0)/max;

  std::vector<std::vector<int> > teensy_to_sensor_metric;
  for (unsigned int i = 0; i < teensy_to_sensor_distances.size(); ++i)
  {
    std::vector<int> metric;
    for (unsigned int j = 0; j < teensy_to_sensor_distances[i].size(); ++j)
    {
      metric.push_back(static_cast<int>(teensy_to_sensor_distances[i][j] * RATIO));
    }
    teensy_to_sensor_metric.push_back(metric);
  }

  return teensy_to_sensor_metric;
}

bool DecData::generateStructureFile(const std::string& abs_file_name,
                                    const std::string progmem_prefix,
                                    const std::string unit_prefix)
{
  std::ofstream header_file;
  header_file.open(abs_file_name.c_str(), std::ios::out);
  ROS_ASSERT_MSG(header_file.is_open(), "Problem when opening header file >%s<.", abs_file_name.c_str());

  header_file << "// This configuration file is generated from dec_visualization/config/structure.yaml.\n";
  header_file << "// Please do not edit.\n";
  header_file << "// Instead edit dec_visualization/config/structure.yaml and regenerate this file.\n\n";
  header_file << "#ifndef _DEC_STRUCTURE_H\n#define _DEC_STRUCTURE_H\n\n";

  if (!progmem_prefix.empty())
    header_file << "#include <avr/pgmspace.h>\n\n";

  header_file << progmem_prefix << "static const " << unit_prefix << "uint8_t LIGHT_PIN_ORDERING[" << light_pins_.size() << "] = {";
  // first lights (nodes, then beams)
  for (unsigned int i = 0; i < light_pins_.size(); ++i)
  {
    header_file << light_pins_[i];
    if (i + 1 < light_pins_.size())
      header_file << ", ";
  }
  header_file << "};" << endl;
  header_file << progmem_prefix << "static const " << unit_prefix << "uint8_t SENSOR_PIN_ORDERING[" << sensor_pins_.size() << "] = {";
  for (unsigned int i = 0; i < sensor_pins_.size(); ++i)
  {
    header_file << sensor_pins_[i];
    if (i + 1 < sensor_pins_.size())
      header_file << ", ";
  }
  header_file << "};" << endl << endl;

  header_file << progmem_prefix << "static const " << unit_prefix << "uint8_t NUM_LED_NODES_PER_ARDUINO[" << number_of_teensys_ << "] = {";
  for (int i = 0; i < number_of_teensys_; ++i)
  {
    header_file << (int)teensy_to_light_node_map_[i].size();
    if (i + 1 < number_of_teensys_)
      header_file << ", ";
  }
  header_file << "};" << endl;

  header_file << progmem_prefix << "static const " << unit_prefix << "uint8_t NUM_LED_BEAMS_PER_ARDUINO[" << number_of_teensys_ << "] = {";
  for (int i = 0; i < number_of_teensys_; ++i)
  {
    header_file << (int)teensy_to_light_beam_map_[i].size();
    if (i + 1 < number_of_teensys_)
      header_file << ", ";
  }
  header_file << "};" << endl;

//  header_file << progmem_prefix << "static const " << unit_prefix << "uint8_t NUM_LED_STRIPS_PER_ARDUINO[" << number_of_teensys_ << "] = {";
//  for (int i = 0; i < number_of_teensys_; ++i)
//  {
//    int num_light_strips = 0;
//    num_light_strips += (int)teensy_to_light_beam_map_[i].size();
//    num_light_strips += (int)teensy_to_light_node_map_[i].size();
//    header_file << num_light_strips;
//    if (i + 1 < number_of_teensys_)
//      header_file << ", ";
//  }
//  header_file << "};" << endl;

  unsigned int num_leds_for_each_light = num_leds_of_each_light_node_.size() + num_leds_of_each_light_beam_.size();
  header_file << progmem_prefix << "static const " << unit_prefix << "uint8_t NUM_LEDS_OF_EACH_LIGHT[" << num_leds_for_each_light << "] = {";
  for (unsigned int i = 0; i < num_leds_for_each_light; ++i)
  {
    if (i < num_leds_of_each_light_node_.size())
    {
      header_file << num_leds_of_each_light_node_[i];
      if (!num_leds_of_each_light_beam_.empty())
        header_file << ", ";
    }
    else
    {
      header_file << num_leds_of_each_light_beam_[i - num_leds_of_each_light_node_.size()];
      if (i + 1 < num_leds_of_each_light_node_.size() + num_leds_of_each_light_beam_.size())
        header_file << ", ";

    }
  }
  header_file << "};" << endl;
  header_file << progmem_prefix << "static const " << unit_prefix << "uint8_t NUM_SENSORS_PER_ARDUINO[" << number_of_teensys_ << "] = {";
  for (int i = 0; i < number_of_teensys_; ++i)
  {
    header_file << teensy_to_sensor_map_[i].size();
    if (i + 1 < number_of_teensys_)
      header_file << ", ";
  }
  header_file << "};" << endl << endl;

  std::vector<std::string> things;
  things.push_back("LIGHT_BEAM");
  things.push_back("LIGHT_NODE");
  things.push_back("SENSOR");
  std::vector<std::vector<int> > connections;
  connections.push_back(light_beam_connections_);
  connections.push_back(light_node_connections_);
  connections.push_back(sensor_connections_);
  std::vector<std::vector<int> > counters;
  counters.push_back(light_beam_index_counter_);
  counters.push_back(light_node_index_counter_);
  counters.push_back(sensor_index_counter_);
  std::vector<std::vector<std::vector<int> > > teensy_to_thing_maps;
  teensy_to_thing_maps.push_back(teensy_to_light_beam_map_);
  teensy_to_thing_maps.push_back(teensy_to_light_node_map_);
  teensy_to_thing_maps.push_back(teensy_to_sensor_map_);

  for (unsigned int i = 0; i < things.size(); ++i)
  {
    header_file << progmem_prefix << "static const " << unit_prefix << "uint8_t " << things[i] << "_CONNECTIONS[" << connections[i].size() << "] = {";
    for (unsigned int j = 0; j < connections[i].size(); ++j)
    {
      header_file << connections[i][j];
      if (j + 1 < connections[i].size())
        header_file << ", ";
    }
    header_file << "};" << endl;
  }

  for (unsigned int i = 0; i < things.size(); ++i)
  {
    header_file << progmem_prefix << "static const " << unit_prefix << "uint8_t " << things[i] << "_INDEX_COUNTER[" << counters[i].size() << "] = {";
    for (unsigned int j = 0; j < counters[i].size(); ++j)
    {
      header_file << counters[i][j];
      if (j + 1 < counters[i].size())
        header_file << ", ";
    }
    header_file << "};" << endl;
  }

  for (unsigned int i = 0; i < things.size(); ++i)
  {
    header_file << "// A value of 255 is assigned to invalidate the entry.\n";
    const std::string NAME = "ARDUINO_TO_" + things[i] + "_MAP";
    header_file << progmem_prefix << "static const " << unit_prefix << "uint8_t " << NAME << "[" << number_of_teensys_ << "][" << teensy_to_thing_maps[i].size() << "] = {";
    for (int j = 0; j < number_of_teensys_; ++j)
    {
      header_file << "{";
      for (int k = 0; k < number_of_teensys_; ++k)
      {
        if (k < (int)teensy_to_thing_maps[i][j].size())
        {
          header_file << teensy_to_thing_maps[i][j][k];
        }
        else
        {
          header_file << "255";
        }
        if (k + 1 < number_of_teensys_)
          header_file << ", ";
      }
      header_file << "}";
      if (j + 1 < number_of_teensys_)
        header_file << ", ";
    }
    header_file << "};" << endl;
  }

  header_file << "\n#endif // _DEC_STRUCTURE_H" << endl;
  header_file.close();

  return true;
}

}
