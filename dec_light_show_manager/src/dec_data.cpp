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

DecData::DecData() :
    control_frequency_(0.0),
    control_dt_(0.0),
    ros_time_sec_(0.0),
    initialized_(false)
{
}

bool DecData::initialize(ros::NodeHandle node_handle)
{
  ROS_VERIFY(DecStructure::initialize(node_handle));

  sensor_levels_ = Eigen::VectorXf::Zero(total_num_sensors_);
  sensor_values_ = VectorX_sensor_channel_t::Zero(total_num_sensors_);
  prev_sensor_values_ = VectorX_sensor_channel_t::Zero(total_num_sensors_);

  node_led_levels_ = Eigen::VectorXf::Zero(total_num_node_leds_);
  node_led_values_ = MatrixX_led_channel_t::Zero(4, total_num_node_leds_);

  if (total_num_block_beam_leds_ > 0)
  {
    block_beam_led_levels_ = Eigen::VectorXf::Zero(total_num_block_beam_leds_);
    block_beam_led_values_ = MatrixX_led_channel_t::Zero(4, total_num_block_beam_leds_);
  }
  if (total_num_pixel_beam_leds_ > 0)
  {
    pixel_beam_led_levels_ = Eigen::VectorXf::Zero(total_num_pixel_beam_leds_);
    pixel_beam_led_values_ = MatrixX_led_channel_t::Zero(4, total_num_pixel_beam_leds_);
  }

  std::vector<double> block_light_nodes_color;
  ROS_VERIFY(dec_utilities::read(node_handle, "block_nodes_color", block_light_nodes_color));
  std::vector<double> block_light_beams_color;
  ROS_VERIFY(dec_utilities::read(node_handle, "block_beams_color", block_light_beams_color));
  std::vector<double> pixel_light_beams_color;
  ROS_VERIFY(dec_utilities::read(node_handle, "pixel_beams_color", pixel_light_beams_color));
  std::vector<double> sensors_color;
  ROS_VERIFY(dec_utilities::read(node_handle, "sensors_color", sensors_color));

  for (unsigned int i = 0; i < total_num_node_leds_; ++i)
  {
    node_led_values_(ALPHA_OFFSET, i) = static_cast<led_channel_t>(block_light_nodes_color[ALPHA_OFFSET] * 255);
  }
  for (unsigned int i = 0; i < total_num_block_beam_leds_; ++i)
  {
    block_beam_led_values_(ALPHA_OFFSET, i) = static_cast<led_channel_t>(block_light_beams_color[ALPHA_OFFSET] * 255);
  }
  for (unsigned int i = 0; i < total_num_pixel_beam_leds_; ++i)
  {
    pixel_beam_led_values_(ALPHA_OFFSET, i) = static_cast<led_channel_t>(pixel_light_beams_color[ALPHA_OFFSET] * 255);
  }
  for (unsigned int i = 0; i < total_num_sensors_; ++i)
  {
    sensor_values_(i) = static_cast<sensor_channel_t>(0.0);
  }

  Eigen::MatrixXf sensor_to_node_led_distances;
  Eigen::MatrixXf sensor_to_block_beam_led_distances;
  Eigen::MatrixXf sensor_to_pixel_beam_led_distances;
  unsigned int num_led_index = 0;

  sensor_to_node_led_distances = Eigen::MatrixXf::Zero(total_num_sensors_, total_num_node_leds_);
  for (unsigned int i = 0; i < total_num_sensors_; ++i)
  {
    num_led_index = 0;
    for (unsigned int j = 0; j < block_light_nodes_.size(); ++j)
    {
      for (unsigned int k = 0; k < block_light_nodes_[j].getNumComponents(); ++k)
      {
        sensor_to_node_led_distances(i, num_led_index) = computeDistance(sensors_[i].getAvgPosition(), block_light_nodes_[j].getPosition(k));
        num_led_index++;
      }
    }
  }

  if (total_num_block_beam_leds_)
  {
    sensor_to_block_beam_led_distances = Eigen::MatrixXf::Zero(total_num_sensors_, total_num_block_beam_leds_);
    for (unsigned int i = 0; i < total_num_sensors_; ++i)
    {
      num_led_index = 0;
      for (unsigned int j = 0; j < block_light_beams_.size(); ++j)
      {
        for (unsigned int k = 0; k < block_light_beams_[j].getNumComponents(); ++k)
        {
          sensor_to_block_beam_led_distances(i, num_led_index) = computeDistance(sensors_[i].getAvgPosition(), block_light_beams_[j].getPosition(k));
          num_led_index++;
        }
      }
    }
  }

  if (total_num_pixel_beam_leds_)
  {
    sensor_to_pixel_beam_led_distances = Eigen::MatrixXf::Zero(total_num_sensors_, total_num_pixel_beam_leds_);
    for (unsigned int i = 0; i < total_num_sensors_; ++i)
    {
      num_led_index = 0;
      for (unsigned int j = 0; j < pixel_light_beams_.size(); ++j)
      {
        for (unsigned int n = 0; n < pixel_light_beams_[j].pixel_poses_.size(); ++n)
        {
          for (unsigned int m = 0; m < pixel_light_beams_[j].pixel_poses_[n].size(); ++m)
          {
            sensor_to_pixel_beam_led_distances(i, num_led_index) = computeDistance(sensors_[i].getAvgPosition(), pixel_light_beams_[j].pixel_poses_[n][m].position);
            num_led_index++;
          }
        }
      }
      ROS_ASSERT(num_led_index == total_num_pixel_beam_leds_);
    }
  }

  // ROS_DEBUG_STREAM("sensor_to_node_led_distances: (" << total_num_sensors_ << " x " << total_num_node_leds_ << ")\n" << sensor_to_node_led_distances);
  // ROS_DEBUG_STREAM("sensor_to_block_beam_led_distances: (" << total_num_sensors_ << " x " << total_num_block_beam_leds_ << ")\n" << sensor_to_block_beam_led_distances);
  // ROS_DEBUG_STREAM("sensor_to_pixel_beam_led_distances: (" << total_num_sensors_ << " x " << total_num_pixel_beam_leds_ << ")\n" << sensor_to_pixel_beam_led_distances);

  Eigen::VectorXd distances = Eigen::VectorXd::Zero(3);
  node_led_distances_to_sensor_ = Eigen::MatrixXf::Zero(total_num_node_leds_, total_num_sensors_);
  node_led_distances_to_sensor_ = sensor_to_node_led_distances.transpose();
  ROS_ASSERT(!(node_led_distances_to_sensor_.minCoeff() < 0.0));
  distances(0) = node_led_distances_to_sensor_.maxCoeff();

  if (total_num_block_beam_leds_ > 0)
  {
    block_beam_led_distances_to_sensor_ = Eigen::MatrixXf::Zero(total_num_block_beam_leds_, total_num_sensors_);
    block_beam_led_distances_to_sensor_ = sensor_to_block_beam_led_distances.transpose();
    ROS_ASSERT(!(block_beam_led_distances_to_sensor_.minCoeff() < 0.0));
    distances(1) = block_beam_led_distances_to_sensor_.maxCoeff();
  }

  if (total_num_pixel_beam_leds_ > 0)
  {
    pixel_beam_led_distances_to_sensor_ = Eigen::MatrixXf::Zero(total_num_pixel_beam_leds_, total_num_sensors_);
    pixel_beam_led_distances_to_sensor_ = sensor_to_pixel_beam_led_distances.transpose();
    ROS_ASSERT(!(pixel_beam_led_distances_to_sensor_.minCoeff() < 0.0));
    distances(2) = pixel_beam_led_distances_to_sensor_.maxCoeff();
  }
  double max_distance = distances.maxCoeff();

  // scale distances such that furthest away becomes 0.0
  node_led_distances_to_sensor_.array() = (node_led_distances_to_sensor_.array() - max_distance) * -1.0;
  // ROS_INFO_STREAM("\nnode\n" << node_led_distances_to_sensor_);
  if (total_num_block_beam_leds_ > 0)
  {
    block_beam_led_distances_to_sensor_.array() = (block_beam_led_distances_to_sensor_.array() - max_distance) * -1.0;
    // ROS_INFO_STREAM("\nblock beam\n" << block_beam_led_distances_to_sensor_);
  }
  if (total_num_pixel_beam_leds_ > 0)
  {
    pixel_beam_led_distances_to_sensor_.array() = (pixel_beam_led_distances_to_sensor_.array() - max_distance) * -1.0;
    // ROS_INFO_STREAM("\npixel beam\n" << pixel_beam_led_distances_to_sensor_);
  }

  // Lastly, generate the configuration file
  bool generate_configuration_file = false;
  if(dec_utilities::read(node_handle, "generate_configuration_file", generate_configuration_file, false))
  {
    if (generate_configuration_file)
    {
      std::string path = ros::package::getPath("dec_communication");
      dec_utilities::appendTrailingSlash(path);
      generateConfigurationFile(path + "include/dec_communication/DEC_config.h");
    }
  }
//  bool generate_structure_file = false;
//  if(dec_utilities::read(node_handle, "generate_structure_file", generate_structure_file, false))
//  {
//    if (generate_structure_file)
//    {
//      std::string path = ros::package::getPath("dec_communication");
//      dec_utilities::appendTrailingSlash(path);
//      // generateStructureFile(path + "DEC_structure_teensy.h", "PROGMEM ", "prog_");
//      generateStructureFile(path + "include/dec_communication/DEC_structure.h");
//    }
//  }

  // if the task was generating the files... we are done
  if (generate_configuration_file/* || generate_structure_file*/)
  {
    return false;
  }

  list_of_teensys_to_exclude_from_communication_.clear();
  ROS_VERIFY(dec_utilities::read(node_handle, "list_of_teensys_to_exclude_from_communication", list_of_teensys_to_exclude_from_communication_));
  ROS_INFO_COND(list_of_teensys_to_exclude_from_communication_.empty() && number_of_teensys_ > 1, "Communicating to all >%i< Teensys.", (int)number_of_teensys_);
  ROS_INFO_COND(list_of_teensys_to_exclude_from_communication_.empty() && number_of_teensys_ == 1, "Communicating to the single Teensys.");
  ROS_ASSERT(number_of_teensys_ > 0);
  for (unsigned int i = 0; i < list_of_teensys_to_exclude_from_communication_.size(); ++i)
  {
    ROS_ASSERT_MSG(list_of_teensys_to_exclude_from_communication_[i] < number_of_teensys_,
                   "Invalid teensy id >%i< specified to exclude from communication. There are only >%i< Teensys specified.",
                   (int)list_of_teensys_to_exclude_from_communication_[i], (int)number_of_teensys_);
    ROS_WARN("Not communicating to teensy with id >%i<.", (int)list_of_teensys_to_exclude_from_communication_[i]);
  }

  return (initialized_ = true);
}

//geometry_msgs::Pose DecData::getBeamLedPose(const int beam_id, const int led, const int num_leds)
//{
//  float offset = ((float)1.0 - light_beams_size_.z) / (float)2.0;
//  float led_fragment_length = light_beams_size_.z / (float)num_leds;
//  float percent_distance_from_first_node = offset + (led_fragment_length/(float)2.0) + ((float)led * led_fragment_length);
//
//  ROS_ASSERT(!(percent_distance_from_first_node < 0.0));
//  ROS_ASSERT(!(percent_distance_from_first_node > 1.0));
//
//  geometry_msgs::Pose led_pose;
//  tf::Vector3 p1, p2;
//  convert(node_positions_[light_beams_[beam_id].first], p1);
//  convert(node_positions_[light_beams_[beam_id].second], p2);
//
//  tf::Vector3 p1p2 = p2 - p1;
//  tf::Vector3 center = (p1 + (percent_distance_from_first_node * p1p2));// / 2.0f;
//  convert(center, led_pose.position);
//
//  tf::Vector3 p12 = p2 - p1;
//  p12.normalize();
//  tf::Vector3 z_world = tf::Vector3(0.0, 0.0, 1.0);
//  tf::Vector3 z_cross_p12 = z_world.cross(p12);
//  double angle = acos(p12.dot(z_world));
//  tf::Quaternion q;
//  q.setRotation(z_cross_p12, angle);
//  convert(q, led_pose.orientation);
//
//  return led_pose;
//}

float DecData::computeDistance(const geometry_msgs::Point& sensor, const geometry_msgs::Point& thing)
{
  tf::Vector3 center_sensor(sensor.x, sensor.y, sensor.z);
  tf::Vector3 center_thing(thing.x, thing.y, thing.z);
  return (center_sensor - center_thing).length();
}

//bool DecData::read(ros::NodeHandle node_handle,
//          const::std::string& array_name,
//          std::vector<int>& values,
//          std::vector<int>& index_values,
//          std::vector<std::vector<int> >& map,
//          const unsigned int& num)
//{
//  ROS_VERIFY(dec_utilities::read(node_handle, array_name, values));
//  ROS_ASSERT_MSG(values.size() == num, "Number of >%s< >%i< must equal >%i<.", array_name.c_str(), (int)values.size(), (int)num);
//
//  for (unsigned int i = 0; i < values.size(); ++i)
//  {
//    ROS_ASSERT_MSG(values[i] >= 0 && values[i] < (int)number_of_teensys_,
//                   ">%s< >%i< is >%i< and therefore invalid. Must be within [0, %i]",
//                   array_name.c_str(), (int)i, values[i], number_of_teensys_-1);
//  }
//
//  index_values.resize(values.size(), -1);
//  map.resize(number_of_teensys_);
//  std::vector<unsigned int> counts(number_of_teensys_, 0);
//  for (unsigned int i = 0; i < number_of_teensys_; ++i)
//  {
//    std::vector<int> v;
//    for (unsigned int j = 0; j < values.size(); ++j)
//    {
//      if ((int)i == values[j])
//      {
//        v.push_back(values[j]);
//        index_values[j] = counts[i];
//        counts[i]++;
//      }
//    }
//    map[i] = v;
//  }
//
//  for (unsigned int i = 0; i < index_values.size(); ++i)
//  {
//    ROS_ASSERT_MSG(index_values[i] >= 0, "Invalid index value >%i<.", index_values[i]);
//  }
//  return true;
//}

//bool DecData::read(ros::NodeHandle node_handle,
//                   const std::string& array_name,
//                   std::vector<std::pair<int, int> >& nodes)
//{
//  XmlRpc::XmlRpcValue rpc_values;
//  if (!node_handle.getParam(array_name, rpc_values))
//  {
//    ROS_ERROR("Could not read from >%s/%s<.", node_handle.getNamespace().c_str(), array_name.c_str());
//    return false;
//  }
//
//  nodes.clear();
//  for (int i = 0; i < rpc_values.size(); ++i)
//  {
//    if (!rpc_values[i].hasMember("nodes"))
//    {
//      ROS_ERROR("Each arm configuration must contain a field >nodes<.");
//      return false;
//    }
//    XmlRpc::XmlRpcValue rpc_value = rpc_values[i]["nodes"];
//    if (rpc_value.size() != 2)
//    {
//      ROS_ERROR("Number of joint_configuration >%i< is wrong. It should be >2<.", rpc_value.size());
//      return false;
//    }
//    if ((rpc_value[0].getType() != XmlRpc::XmlRpcValue::TypeInt)
//        || (rpc_value[1].getType() != XmlRpc::XmlRpcValue::TypeInt))
//    {
//      ROS_ERROR("Values must either be of type integer.");
//      return false;
//    }
//    std::pair<int, int> values(rpc_value[0], rpc_value[1]);
//    ROS_ASSERT_MSG((values.first >= 0) && (values.first < (int)node_positions_.size())
//        && (values.second >= 0) && (values.second < (int)node_positions_.size()),
//        "Invalid >%s< index read at row %i: from >%i< to >%i<. It must be within [0, %i].",
//                array_name.c_str(), i, values.first, values.second, (int)(node_positions_.size()-1));
//    ROS_ASSERT_MSG(values.first != values.second, "Invalid beam read row %i. Node indices must differ.", i);
//
//    nodes.push_back(values);
//  }
//  return true;
//}

int DecData::getNumTeensys() const
{
  ROS_ASSERT(initialized_);
  return number_of_teensys_;
}

//std::vector<std::vector<int> > DecData::getTeensyToSensorsDistances(const unsigned int& teensy_id) const
//{
//  ROS_ASSERT(teensy_id < number_of_teensys_);
//
//  std::vector<std::vector<double> > teensy_to_sensor_distances;
//  for (unsigned int i = 0; i < number_of_teensys_; ++i)
//  {
//    std::vector<double> distances;
//    for (unsigned int j = 0; j < teensy_to_sensor_map_[i].size(); ++j)
//    {
//      tf::Vector3 point_1, point_2;
//      convert(node_positions_[sensors_[j].first], point_1);
//      convert(node_positions_[sensors_[j].second], point_2);
//      // tf::Vector3 sensor_position = (point_1 + point_2) / static_cast<double>(2.0);
//
//      double distance = fabs(tf::Vector3(point_1 - point_2).length());
//      distances.push_back(distance);
//    }
//    teensy_to_sensor_distances.push_back(distances);
//  }
//
//  // compute maximum and scale accordingly
//  double max = -1.0;
//  for (unsigned int i = 0; i < teensy_to_sensor_distances.size(); ++i)
//  {
//    for (unsigned int j = 0; j < teensy_to_sensor_distances[i].size(); ++j)
//    {
//      if (teensy_to_sensor_distances[i][j] > max)
//      {
//        max = teensy_to_sensor_distances[i][j];
//      }
//    }
//  }
//  ROS_INFO("Maximum distance is >%.2f< meters.", max);
//  ROS_ASSERT(max > 0.0);
//  const double RATIO = static_cast<double>(1.0)/max;
//
//  std::vector<std::vector<int> > teensy_to_sensor_metric;
//  for (unsigned int i = 0; i < teensy_to_sensor_distances.size(); ++i)
//  {
//    std::vector<int> metric;
//    for (unsigned int j = 0; j < teensy_to_sensor_distances[i].size(); ++j)
//    {
//      metric.push_back(static_cast<int>(teensy_to_sensor_distances[i][j] * RATIO));
//    }
//    teensy_to_sensor_metric.push_back(metric);
//  }
//
//  return teensy_to_sensor_metric;
//}

bool DecData::generateConfigurationFile(const std::string& abs_file_name)
{
  ROS_INFO("Writing >%s<.", abs_file_name.c_str());
  std::ofstream header_file;
  header_file.open(abs_file_name.c_str(), std::ios::out);
  ROS_ASSERT_MSG(header_file.is_open(), "Problem when opening header file >%s<.", abs_file_name.c_str());

  header_file << "// This configuration file is generated from dec_visualization/config/config.yaml.\n";
  header_file << "// Please do not edit.\n";
  header_file << "// Instead edit dec_visualization/config/structure.yaml and regenerate this file.\n\n";

  header_file << "#ifndef _DEC_CONFIG_H" << endl;
  header_file << "#define _DEC_CONFIG_H\n" << endl;

  header_file << "// Number of teensys in the structure.\n";
  header_file << "#define DEC_NUMBER_OF_TEENSYS (uint8_t)" << number_of_teensys_ << "\n\n";
  header_file << "// Maximum number of sensors, light strips, and LEDs per light strip. (To statically allocate memory)\n";
  header_file << endl;
  header_file << "#define DEC_MAX_NUMBER_OF_SENSORS_PER_NODE (uint8_t)" << max_number_of_sensors_per_teensy_ << "\n";
  header_file << "#define DEC_MAX_NUMBER_OF_LED_STRIPS_PER_NODE (uint8_t)" << max_number_of_light_strips_per_teensy_ << "\n";
  header_file << endl;
  header_file << "#define DEC_MAX_NUMBER_OF_BLOCKS_PER_TEENSY (uint8_t)" << MAX_NUMBER_OF_BLOCKS_PER_TEENSY << endl;
  header_file << "#define DEC_MAX_NUMBER_OF_PIXELS_PER_TEENSY (uint8_t)" << MAX_NUMBER_OF_PIXELS_PER_TEENSY << endl;
  header_file << endl;
  // header_file << "#define DEC_MAX_NUMBER_OF_BLOCKS_PER_LIGHT_STRIP (uint8_t)" << MAX_NUMBER_OF_BLOCKS_PER_LIGHT_STRIP << endl;
  header_file << "#define DEC_MAX_NUMBER_OF_PIXELS_PER_LIGHT_STRIP (uint8_t)" << MAX_NUMBER_OF_PIXELS_PER_LIGHT_STRIP << endl;

  header_file << "static const " << "uint8_t LIGHT_PIN_ORDERING[" << light_pins_.size() << "] = {";
  // first lights (nodes, then beams)
  for (unsigned int i = 0; i < light_pins_.size(); ++i)
  {
    header_file << light_pins_[i];
    if (i + 1 < light_pins_.size())
      header_file << ", ";
  }
  header_file << "};" << endl;
  header_file << "static const " << "uint8_t SENSOR_PIN_ORDERING[" << sensor_pins_.size() << "] = {";
  for (unsigned int i = 0; i < sensor_pins_.size(); ++i)
  {
    header_file << sensor_pins_[i];
    if (i + 1 < sensor_pins_.size())
      header_file << ", ";
  }
  header_file << "};" << endl;

  header_file << "\n#endif // _DEC_CONFIG_H" << endl;
  header_file.close();

  ROS_INFO("Done.");
  return true;
}

/*
bool DecData::generateStructureFile(const std::string& abs_file_name,
                                    const std::string progmem_prefix,
                                    const std::string unit_prefix)
{
  ROS_INFO("Writing >%s<.", abs_file_name.c_str());
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
  header_file << "};" << endl;
*/
//  header_file << endl << progmem_prefix << "static const " << unit_prefix << "uint8_t NUM_SENSORS_PER_TEENSY[" << number_of_teensys_ << "] = {";
//  for (unsigned int i = 0; i < dec_interface_setup_data_.size(); ++i)
//  {
//    header_file << (unsigned int)dec_interface_setup_data_[i].num_sensors;
//    if (i + 1 < number_of_teensys_)
//      header_file << ", ";
//  }
//  header_file << "};" << endl << endl;

//  header_file << progmem_prefix << "static const " << unit_prefix << "uint8_t NUM_BLOCK_LEDS_PER_TEENSY[" << number_of_teensys_ << "] = {";
//  for (unsigned int i = 0; i < dec_interface_setup_data_.size(); ++i)
//  {
//    header_file << (unsigned int)dec_interface_setup_data_[i].num_block_leds;
//    if (i + 1 < number_of_teensys_)
//      header_file << ", ";
//  }
//  header_file << "};" << endl;

//  header_file << progmem_prefix << "static const " << unit_prefix << "uint8_t BLOCK_LEDS_START_INDEX[][" << MAX_NUMBER_OF_BLOCKS_PER_LIGHT_STRIP << "] = {";
//  for (unsigned int j = 0; j < number_of_teensys_; ++j)
//  {
//    header_file << "{";
//    for (unsigned int k = 0; k < dec_interface_setup_data_[j].num_block_leds; ++k)
//    {
//      header_file << (unsigned int)dec_interface_setup_data_[j].block_leds[k].index;
//      if (k + 1 < dec_interface_setup_data_[j].num_block_leds)
//        header_file << ", ";
//    }
//    header_file << "}";
//    if (j + 1 < number_of_teensys_)
//      header_file << ", ";
//  }
//  header_file << "};" << endl;
//
//  header_file << progmem_prefix << "static const " << unit_prefix << "uint8_t BLOCK_LEDS_NUMBER[][" << MAX_NUMBER_OF_BLOCKS_PER_LIGHT_STRIP << "] = {";
//  for (unsigned int j = 0; j < number_of_teensys_; ++j)
//  {
//    header_file << "{";
//    for (unsigned int k = 0; k < dec_interface_setup_data_[j].num_block_leds; ++k)
//    {
//      header_file << (unsigned int)dec_interface_setup_data_[j].block_leds[k].num_blocks;
//      if (k + 1 < dec_interface_setup_data_[j].num_block_leds)
//        header_file << ", ";
//    }
//    header_file << "}";
//    if (j + 1 < number_of_teensys_)
//      header_file << ", ";
//  }
//  header_file << "};" << endl;
//
//  header_file << progmem_prefix << "static const " << unit_prefix << "uint8_t BLOCK_LEDS_PINS[][" << MAX_NUMBER_OF_BLOCKS_PER_LIGHT_STRIP << "] = {";
//  for (unsigned int j = 0; j < number_of_teensys_; ++j)
//  {
//    header_file << "{";
//    for (unsigned int k = 0; k < dec_interface_setup_data_[j].num_block_leds; ++k)
//    {
//      header_file << (unsigned int)dec_interface_setup_data_[j].block_leds[k].pin;
//      if (k + 1 < dec_interface_setup_data_[j].num_block_leds)
//        header_file << ", ";
//    }
//    header_file << "}";
//    if (j + 1 < number_of_teensys_)
//      header_file << ", ";
//  }
//  header_file << "};" << endl << endl;

//  header_file << progmem_prefix << "static const " << unit_prefix << "uint8_t NUM_PIXEL_LEDS_PER_TEENSY[" << number_of_teensys_ << "] = {";
//  for (unsigned int i = 0; i < number_of_teensys_; ++i)
//  {
//    header_file << (unsigned int)dec_interface_setup_data_[i].num_pixel_leds;
//    if (i + 1 < number_of_teensys_)
//      header_file << ", ";
//  }
//  header_file << "};" << endl;
//
//  header_file << progmem_prefix << "static const " << unit_prefix << "uint8_t PIXEL_LEDS_START_INDEX[][" << MAX_NUMBER_OF_PIXELS_PER_LIGHT_STRIP << "] = {";
//  for (unsigned int j = 0; j < number_of_teensys_; ++j)
//  {
//    header_file << "{";
//    for (unsigned int k = 0; k < dec_interface_setup_data_[j].num_pixel_leds; ++k)
//    {
//      header_file << (unsigned int)dec_interface_setup_data_[j].pixel_leds[k].index;
//      if (k + 1 < dec_interface_setup_data_[j].num_pixel_leds)
//        header_file << ", ";
//    }
//    header_file << "}";
//    if (j + 1 < number_of_teensys_)
//      header_file << ", ";
//  }
//  header_file << "};" << endl;
//
//  header_file << progmem_prefix << "static const " << unit_prefix << "uint8_t PIXEL_LEDS_NUMBER[][" << MAX_NUMBER_OF_PIXELS_PER_LIGHT_STRIP << "] = {";
//  for (unsigned int j = 0; j < number_of_teensys_; ++j)
//  {
//    header_file << "{";
//    for (unsigned int k = 0; k < dec_interface_setup_data_[j].num_pixel_leds; ++k)
//    {
//      header_file << (unsigned int)dec_interface_setup_data_[j].pixel_leds[k].num_pixels;
//      if (k + 1 < dec_interface_setup_data_[j].num_pixel_leds)
//        header_file << ", ";
//    }
//    header_file << "}";
//    if (j + 1 < number_of_teensys_)
//      header_file << ", ";
//  }
//  header_file << "};" << endl;
//
//  header_file << progmem_prefix << "static const " << unit_prefix << "uint8_t PIXEL_LEDS_PINS[][" << MAX_NUMBER_OF_PIXELS_PER_LIGHT_STRIP << "] = {";
//  for (unsigned int j = 0; j < number_of_teensys_; ++j)
//  {
//    header_file << "{";
//    for (unsigned int k = 0; k < dec_interface_setup_data_[j].num_pixel_leds; ++k)
//    {
//      header_file << (unsigned int)dec_interface_setup_data_[j].pixel_leds[k].pin;
//      if (k + 1 < dec_interface_setup_data_[j].num_pixel_leds)
//        header_file << ", ";
//    }
//    header_file << "}";
//    if (j + 1 < number_of_teensys_)
//      header_file << ", ";
//  }
//  header_file << "};" << endl << endl;

//  // <number_of_strips - total_number_of_leds>
//  std::vector<std::pair<unsigned int, unsigned int> > total_number_of_strips(number_of_teensys_);
//  header_file << progmem_prefix << "static const " << unit_prefix << "uint8_t NUM_STRIPS_USED[" << number_of_teensys_ << "] = {";
//  for (unsigned int i = 0; i < number_of_teensys_; ++i)
//  {
//    for (unsigned int j = 0; j < num_leds_at_each_strip_[i].size(); ++j)
//    {
//      if (num_leds_at_each_strip_[i][j] > 0)
//      {
//        total_number_of_strips[i].first++;
//        total_number_of_strips[i].second = num_leds_at_each_strip_[i][j];
//      }
//    }
//    header_file << total_number_of_strips[i].first;
//    if (i + 1 < number_of_teensys_)
//      header_file << ", ";
//  }
//  header_file << "};" << endl;
//
//  header_file << progmem_prefix << "static const " << unit_prefix << "uint8_t TOTAL_NUM_LEDS_AT_STRIP[][" << max_number_of_light_strips_per_teensy_ << "] = {";
//  ROS_ASSERT(number_of_teensys_ == num_leds_at_each_strip_.size());
//  for (unsigned int i = 0; i < number_of_teensys_; ++i)
//  {
//    header_file << "{";
//    for (unsigned int j = 0; j < num_leds_at_each_strip_[i].size(); ++j)
//    {
//      header_file << (unsigned int) num_leds_at_each_strip_[i][j];
//      if (j + 1 < num_leds_at_each_strip_[i].size())
//        header_file << ", ";
//    }
//    header_file << "}";
//    if (i + 1 < number_of_teensys_)
//      header_file << ", ";
//  }
//  header_file << "};" << endl;

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

  //  unsigned int num_leds_for_each_light = num_leds_of_each_light_node_.size() + num_leds_of_each_light_beam_.size();
  //  header_file << progmem_prefix << "static const " << unit_prefix << "uint8_t NUM_LEDS_OF_EACH_LIGHT[" << num_leds_for_each_light << "] = {";
  //  for (unsigned int i = 0; i < num_leds_for_each_light; ++i)
  //  {
  //    if (i < num_leds_of_each_light_node_.size())
  //    {
  //      header_file << num_leds_of_each_light_node_[i];
  //      if (!num_leds_of_each_light_beam_.empty())
  //        header_file << ", ";
  //    }
  //    else
  //    {
  //      header_file << num_leds_of_each_light_beam_[i - num_leds_of_each_light_node_.size()];
  //      if (i + 1 < num_leds_of_each_light_node_.size() + num_leds_of_each_light_beam_.size())
  //        header_file << ", ";
  //    }
  //  }
  //  header_file << "};" << endl;


//  std::vector<std::string> things;
//  things.push_back("LIGHT_BEAM");
//  things.push_back("LIGHT_NODE");
//  things.push_back("SENSOR");
//  std::vector<std::vector<int> > connections;
//  connections.push_back(light_block_beam_connections_);
//  connections.push_back(light_node_connections_);
//  connections.push_back(sensor_connections_);
//  std::vector<std::vector<int> > counters;
//  counters.push_back(light_beam_index_counter_);
//  counters.push_back(light_node_index_counter_);
//  counters.push_back(sensor_index_counter_);
//  std::vector<std::vector<std::vector<int> > > teensy_to_thing_maps;
//  teensy_to_thing_maps.push_back(teensy_to_light_beam_map_);
//  teensy_to_thing_maps.push_back(teensy_to_light_node_map_);
//  teensy_to_thing_maps.push_back(teensy_to_sensor_map_);
//
//  for (unsigned int i = 0; i < things.size(); ++i)
//  {
//    header_file << progmem_prefix << "static const " << unit_prefix << "uint8_t " << things[i] << "_CONNECTIONS[" << connections[i].size() << "] = {";
//    for (unsigned int j = 0; j < connections[i].size(); ++j)
//    {
//      header_file << connections[i][j];
//      if (j + 1 < connections[i].size())
//        header_file << ", ";
//    }
//    header_file << "};" << endl;
//  }
//
//  for (unsigned int i = 0; i < things.size(); ++i)
//  {
//    header_file << progmem_prefix << "static const " << unit_prefix << "uint8_t " << things[i] << "_INDEX_COUNTER[" << counters[i].size() << "] = {";
//    for (unsigned int j = 0; j < counters[i].size(); ++j)
//    {
//      header_file << counters[i][j];
//      if (j + 1 < counters[i].size())
//        header_file << ", ";
//    }
//    header_file << "};" << endl;
//  }
//
//  for (unsigned int i = 0; i < things.size(); ++i)
//  {
//    header_file << "// A value of 255 is assigned to invalidate the entry.\n";
//    const std::string NAME = "ARDUINO_TO_" + things[i] + "_MAP";
//    header_file << progmem_prefix << "static const " << unit_prefix << "uint8_t " << NAME << "[" << number_of_teensys_ << "][" << teensy_to_thing_maps[i].size() << "] = {";
//    for (unsigned int j = 0; j < number_of_teensys_; ++j)
//    {
//      header_file << "{";
//      for (unsigned int k = 0; k < number_of_teensys_; ++k)
//      {
//        if (k < teensy_to_thing_maps[i][j].size())
//        {
//          header_file << teensy_to_thing_maps[i][j][k];
//        }
//        else
//        {
//          header_file << "255";
//        }
//        if (k + 1 < number_of_teensys_)
//          header_file << ", ";
//      }
//      header_file << "}";
//      if (j + 1 < number_of_teensys_)
//        header_file << ", ";
//    }
//    header_file << "};" << endl;
//  }

//  header_file << "\n#endif // _DEC_STRUCTURE_H" << endl;
//  header_file.close();
//
//  ROS_INFO("Done.");
//  return true;
//}


bool DecData::sendSetupData()
{
  unsigned int setup_count = 0;
  std::vector<bool> is_setup(number_of_teensys_, false);
  for (unsigned int i = 0; i < number_of_teensys_; ++i)
  {
    for (unsigned int j = 0; j < list_of_teensys_to_exclude_from_communication_.size(); ++j)
    {
      if (i == list_of_teensys_to_exclude_from_communication_[j])
      {
        is_setup[i] = true;
        setup_count++;
        ROS_WARN("Excluding setup of node >%i<...", i);
      }
    }
    if (!is_setup[i] && dec_interface_->sendSetupData(i, dec_interface_setup_data_[i]))
    {
      ROS_INFO("Setup of node >%i< complete.", i);
      is_setup[i] = true;
      setup_count++;
    }
    ROS_WARN_COND(!is_setup[i], "Failed to setup node >%i<... retrying...", i);
    ROS_INFO_COND(is_setup[i], "Setup of node >%i< complete.", i);
    if (i + 1 >= number_of_teensys_ && setup_count < number_of_teensys_)
    {
      i = -1;
    }
  }
  return true;
}

bool DecData::copySensorInformationToStructure()
{
  return true;
}

bool DecData::copySensorInformationFromStructure()
{
  for (unsigned int i = 0; i < sensor_to_teensy_map_.size(); ++i)
  {
    sensor_values_(i) = (int)dec_interface_->received_sensor_data_[sensor_to_teensy_map_[i].first].sensor_value[sensor_to_teensy_map_[i].second];
  }
  return true;
}

bool DecData::copyLightDataToStructure()
{
  for (unsigned int i = 0; i < light_node_leds_to_teensy_map_.size(); ++i)
  {
    const uint8_t TEENSY_ID = static_cast<uint8_t>(light_node_leds_to_teensy_map_[i].first);
    // const uint8_t BLOCK_INDEX = static_cast<uint8_t>(light_node_leds_to_teensy_map_[i].second.first);
    const uint8_t BLOCK_INDEX = static_cast<uint8_t>(i);
    const uint8_t STRIP_INDEX =  static_cast<uint8_t>(light_node_leds_to_teensy_map_[i].second.second);
    ROS_ASSERT_MSG(dec_interface_setup_data_[TEENSY_ID].num_block_leds < DEC_MAX_NUMBER_OF_BLOCKS_PER_TEENSY,
               "dec_interface_setup_data_[%i].num_block_leds is >%i< and DEC_MAX_NUMBER_OF_BLOCKS_PER_TEENSY is >%i<.",
               (int)TEENSY_ID, (int)dec_interface_setup_data_[TEENSY_ID].num_block_leds, (int)DEC_MAX_NUMBER_OF_BLOCKS_PER_TEENSY);
    ROS_ASSERT_MSG(BLOCK_INDEX < dec_interface_setup_data_[TEENSY_ID].num_block_leds,
                   "Number of node blocks is >%i< and BLOCK_INDEX is >%i<.",
                   (int)dec_interface_setup_data_[TEENSY_ID].num_block_leds, (int)BLOCK_INDEX);
    ROS_INFO("NODE >%i<: TEENSY_ID %i | BLOCK_INDEX %i | STRIP_INDEX %i |", (int)i, (int)TEENSY_ID, (int)BLOCK_INDEX, (int)STRIP_INDEX);
    // ROS_INFO("NODE >%i<: NUM BLOCKS %i |", (int)i, (int)dec_interface_setup_data_[TEENSY_ID].num_block_leds);
    ROS_INFO(">%i< - >%i< >%i< >%i< >%i<", (int)i,
             node_led_values_(RED_OFFSET, i),
             node_led_values_(GREEN_OFFSET, i),
             node_led_values_(BLUE_OFFSET, i),
             node_led_values_(ALPHA_OFFSET, i));
    dec_interface_light_data_[TEENSY_ID].block_leds[BLOCK_INDEX].red = (uint8_t) node_led_values_(RED_OFFSET, i);
    dec_interface_light_data_[TEENSY_ID].block_leds[BLOCK_INDEX].green = (uint8_t) node_led_values_(GREEN_OFFSET, i);
    dec_interface_light_data_[TEENSY_ID].block_leds[BLOCK_INDEX].blue = (uint8_t) node_led_values_(BLUE_OFFSET, i);
    dec_interface_light_data_[TEENSY_ID].block_leds[BLOCK_INDEX].brightness = (uint8_t) node_led_values_(ALPHA_OFFSET, i);
  }

  for (unsigned int i = 0; i < block_light_beam_leds_to_teensy_map_.size(); ++i)
  {
    const uint8_t TEENSY_ID = static_cast<uint8_t>(block_light_beam_leds_to_teensy_map_[i].first);
   // const uint8_t BLOCK_INDEX = static_cast<uint8_t>(light_node_leds_to_teensy_map_.back().second.first)
   //     + static_cast<uint8_t>(1) + static_cast<uint8_t>(block_light_beam_leds_to_teensy_map_[i].second.first);
    const uint8_t BLOCK_INDEX = static_cast<uint8_t>(light_node_leds_to_teensy_map_.size() + i);
    const uint8_t STRIP_INDEX = static_cast<uint8_t>(block_light_beam_leds_to_teensy_map_[i].second.second);
    ROS_INFO("BLOCK BEAM >%i<: TEENSY_ID %i | BLOCK_STRIP %i | STRIP_INDEX %i |", (int)i, (int)TEENSY_ID, (int)BLOCK_INDEX, (int)STRIP_INDEX);
    ROS_ASSERT_MSG(dec_interface_setup_data_[TEENSY_ID].num_block_leds < DEC_MAX_NUMBER_OF_BLOCKS_PER_TEENSY,
               "dec_interface_setup_data_[%i].num_block_leds is >%i< and DEC_MAX_NUMBER_OF_BLOCKS_PER_TEENSY is >%i<.",
               (int)TEENSY_ID, (int)dec_interface_setup_data_[TEENSY_ID].num_block_leds, (int)DEC_MAX_NUMBER_OF_BLOCKS_PER_TEENSY);
    ROS_ASSERT_MSG(BLOCK_INDEX < dec_interface_setup_data_[TEENSY_ID].num_block_leds,
                   "Number of beam blocks is >%i< and BLOCK_INDEX is >%i<.",
                   (int)dec_interface_setup_data_[TEENSY_ID].num_block_leds, (int)BLOCK_INDEX);
    ROS_INFO(">%i< - >%i< >%i< >%i< >%i<", (int)i,
             block_beam_led_values_(RED_OFFSET, i),
             block_beam_led_values_(GREEN_OFFSET, i),
             block_beam_led_values_(BLUE_OFFSET, i),
             block_beam_led_values_(ALPHA_OFFSET, i));
    dec_interface_light_data_[TEENSY_ID].block_leds[BLOCK_INDEX].red = (uint8_t) block_beam_led_values_(RED_OFFSET, i);
    dec_interface_light_data_[TEENSY_ID].block_leds[BLOCK_INDEX].green = (uint8_t) block_beam_led_values_(GREEN_OFFSET, i);
    dec_interface_light_data_[TEENSY_ID].block_leds[BLOCK_INDEX].blue = (uint8_t) block_beam_led_values_(BLUE_OFFSET, i);
    dec_interface_light_data_[TEENSY_ID].block_leds[BLOCK_INDEX].brightness = (uint8_t) block_beam_led_values_(ALPHA_OFFSET, i);
  }

  for (unsigned int i = 0; i < pixel_light_beam_leds_to_teensy_map_.size(); ++i)
  {
    const uint8_t TEENSY_ID = static_cast<uint8_t>(pixel_light_beam_leds_to_teensy_map_[i].first);
    const uint8_t PIXEL_INDEX = static_cast<uint8_t>(pixel_light_beam_leds_to_teensy_map_[i].second.first);
    const uint8_t STRIP_INDEX = static_cast<uint8_t>(pixel_light_beam_leds_to_teensy_map_[i].second.second);
    ROS_ASSERT_MSG(dec_interface_setup_data_[TEENSY_ID].num_pixel_leds < DEC_MAX_NUMBER_OF_PIXELS_PER_TEENSY,
               "dec_interface_setup_data_[%i].num_pixel_leds is >%i< and DEC_MAX_NUMBER_OF_PIXELS_PER_TEENSY is >%i<.",
               (int)TEENSY_ID, (int)dec_interface_setup_data_[TEENSY_ID].num_pixel_leds, (int)DEC_MAX_NUMBER_OF_PIXELS_PER_TEENSY);
    ROS_ASSERT_MSG(PIXEL_INDEX < dec_interface_setup_data_[TEENSY_ID].num_pixel_leds,
                   "Number of pixels is >%i< and PIXEL_INDEX is >%i<.",
                   (int)dec_interface_setup_data_[TEENSY_ID].num_pixel_leds, (int)PIXEL_INDEX);
    ROS_ASSERT_MSG(STRIP_INDEX < DEC_MAX_NUMBER_OF_PIXELS_PER_LIGHT_STRIP,
                   "Number of beam strips is >%i< and STRIP_INDEX is >%i<.",
                   (int)DEC_MAX_NUMBER_OF_PIXELS_PER_LIGHT_STRIP, (int)STRIP_INDEX);
    ROS_INFO("PIXEL BEAM >%i<: TEENSY_ID %i | PIXEL_STRIP %i | STRIP_INDEX %i |", (int)i, (int)TEENSY_ID, (int)PIXEL_INDEX, (int)STRIP_INDEX);
    ROS_INFO(">%i< - >%i< >%i< >%i< >%i<", (int)i,
             pixel_beam_led_values_(RED_OFFSET, i),
             pixel_beam_led_values_(GREEN_OFFSET, i),
             pixel_beam_led_values_(BLUE_OFFSET, i),
             pixel_beam_led_values_(ALPHA_OFFSET, i));
    dec_interface_light_data_[TEENSY_ID].pixel_leds[PIXEL_INDEX].red[STRIP_INDEX] = (uint8_t) pixel_beam_led_values_(RED_OFFSET, i);
    dec_interface_light_data_[TEENSY_ID].pixel_leds[PIXEL_INDEX].green[STRIP_INDEX] = (uint8_t) pixel_beam_led_values_(GREEN_OFFSET, i);
    dec_interface_light_data_[TEENSY_ID].pixel_leds[PIXEL_INDEX].blue[STRIP_INDEX] = (uint8_t) pixel_beam_led_values_(BLUE_OFFSET, i);
    dec_interface_light_data_[TEENSY_ID].pixel_leds[PIXEL_INDEX].brightness[STRIP_INDEX] = (uint8_t) pixel_beam_led_values_(ALPHA_OFFSET, i);
  }

  // send/receive light/sensor data
  for (unsigned int i = 0; i < number_of_teensys_; ++i)
  {
    bool send = true;
    for (unsigned int j = 0; send && j < list_of_teensys_to_exclude_from_communication_.size(); ++j)
    {
      if (i == list_of_teensys_to_exclude_from_communication_[j])
      {
        send = false;
      }
    }
    if (send)
    {
      if(!dec_interface_->sendLightData(i, dec_interface_light_data_[i]))
      {
        ROS_WARN("Missed send/receive cycle for node >%i<.", i);
      }
    }
  }
  return true;
}

bool DecData::copyLightDataFromStructure()
{
  return true;
}

}

