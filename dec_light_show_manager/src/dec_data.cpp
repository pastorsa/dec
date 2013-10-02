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
    recording_(true),
    visualization_mode_(false),
    initialized_(false),
    max_brightness_(0)
{
}

bool DecData::initialize(ros::NodeHandle node_handle)
{
  node_handle_ = node_handle;
  ROS_VERIFY(DecStructure::initialize(node_handle));

  ROS_VERIFY(dec_utilities::read(node_handle_, "max_brightness", max_brightness_));

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

  //  for (unsigned int i = 0; i < total_num_node_leds_; ++i)
  //  {
  //    node_led_values_(ALPHA_OFFSET, i) = static_cast<led_channel_t>(block_light_nodes_color[ALPHA_OFFSET] * 255);
  //  }
  //  for (unsigned int i = 0; i < total_num_block_beam_leds_; ++i)
  //  {
  //    block_beam_led_values_(ALPHA_OFFSET, i) = static_cast<led_channel_t>(block_light_beams_color[ALPHA_OFFSET] * 255);
  //  }
  //  for (unsigned int i = 0; i < total_num_pixel_beam_leds_; ++i)
  //  {
  //    pixel_beam_led_values_(ALPHA_OFFSET, i) = static_cast<led_channel_t>(pixel_light_beams_color[ALPHA_OFFSET] * 255);
  //  }
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

  if (total_num_block_beam_leds_ > 0)
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

  if (total_num_pixel_beam_leds_ > 0)
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

  send_flags_.resize(number_of_teensys_, true);
  for (unsigned int i = 0; i < number_of_teensys_; ++i)
  {
    for (unsigned int j = 0; send_flags_[i] && j < list_of_teensys_to_exclude_from_communication_.size(); ++j)
    {
      if (i == list_of_teensys_to_exclude_from_communication_[j])
      {
        send_flags_[i] = false;
      }
    }
  }

  return (initialized_ = true);
}

float DecData::computeDistance(const geometry_msgs::Point& sensor, const geometry_msgs::Point& thing)
{
  tf::Vector3 center_sensor(sensor.x, sensor.y, sensor.z);
  tf::Vector3 center_thing(thing.x, thing.y, thing.z);
  return (center_sensor - center_thing).length();
}

int DecData::getNumTeensys() const
{
  ROS_ASSERT(initialized_);
  return number_of_teensys_;
}

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
  header_file << "#define DEC_MAX_NUMBER_OF_PIXELS_PER_LIGHT_STRIP (uint8_t)" << MAX_NUMBER_OF_PIXELS_PER_LIGHT_STRIP << endl << endl;

  header_file << "static const " << "uint8_t LIGHT_PIN_ORDERING[" << light_pins_.size() << "] = {";
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
  //  for (unsigned int i = 0; i < sensor_to_teensy_map_.size(); ++i)
  //  {
  //    sensor_values_(i) = (int)dec_interface_->received_sensor_data_[sensor_to_teensy_map_[i].first].sensor_value[sensor_to_teensy_map_[i].second];
  //  }
  return true;
}

bool DecData::copyLightDataToStructure()
{

  for (unsigned int i = 0; i < light_node_leds_to_teensy_map_.size(); ++i)
  {
    const uint8_t TEENSY_ID = static_cast<uint8_t>(light_node_leds_to_teensy_map_[i].first);
    const uint8_t BLOCK_INDEX = static_cast<uint8_t>(light_node_leds_to_teensy_map_[i].second);
    ROS_ASSERT_MSG(dec_interface_setup_data_[TEENSY_ID].num_block_leds < DEC_MAX_NUMBER_OF_BLOCKS_PER_TEENSY,
               "dec_interface_setup_data_[%i].num_block_leds is >%i< and DEC_MAX_NUMBER_OF_BLOCKS_PER_TEENSY is >%i<.",
               (int)TEENSY_ID, (int)dec_interface_setup_data_[TEENSY_ID].num_block_leds, (int)DEC_MAX_NUMBER_OF_BLOCKS_PER_TEENSY);
    ROS_ASSERT_MSG(BLOCK_INDEX < dec_interface_setup_data_[TEENSY_ID].num_block_leds,
                   "Number of node blocks is >%i< and BLOCK_INDEX is >%i<.",
                   (int)dec_interface_setup_data_[TEENSY_ID].num_block_leds, (int)BLOCK_INDEX);
    // ROS_DEBUG("Node : >%i< - >%i< >%i< >%i< >%i<", (int)i, node_led_values_(RED_OFFSET, i), node_led_values_(GREEN_OFFSET, i), node_led_values_(BLUE_OFFSET, i), node_led_values_(ALPHA_OFFSET, i));
    float alpha = node_led_values_(BRIGHTNESS_OFFSET, i) / 255.0f;
    ROS_ASSERT(!(alpha < 0) && !(alpha > 1.0f));
    dec_interface_light_data_[TEENSY_ID].block_leds[BLOCK_INDEX].red = (uint8_t) static_cast<float>(node_led_values_(RED_OFFSET, i)) * alpha;
    dec_interface_light_data_[TEENSY_ID].block_leds[BLOCK_INDEX].green = (uint8_t) static_cast<float>(node_led_values_(GREEN_OFFSET, i)) * alpha;
    dec_interface_light_data_[TEENSY_ID].block_leds[BLOCK_INDEX].blue = (uint8_t) static_cast<float>(node_led_values_(BLUE_OFFSET, i)) * alpha;
  }

  for (unsigned int i = 0; i < block_light_beam_leds_to_teensy_map_.size(); ++i)
  {
    const uint8_t TEENSY_ID = static_cast<uint8_t>(block_light_beam_leds_to_teensy_map_[i].first);
    const uint8_t BLOCK_INDEX = static_cast<uint8_t>(block_light_beam_leds_to_teensy_map_[i].second);
    ROS_ASSERT_MSG(dec_interface_setup_data_[TEENSY_ID].num_block_leds < DEC_MAX_NUMBER_OF_BLOCKS_PER_TEENSY,
               "dec_interface_setup_data_[%i].num_block_leds is >%i< and DEC_MAX_NUMBER_OF_BLOCKS_PER_TEENSY is >%i<.",
               (int)TEENSY_ID, (int)dec_interface_setup_data_[TEENSY_ID].num_block_leds, (int)DEC_MAX_NUMBER_OF_BLOCKS_PER_TEENSY);
    ROS_ASSERT_MSG(BLOCK_INDEX < dec_interface_setup_data_[TEENSY_ID].num_block_leds, "Number of beam blocks is >%i< and BLOCK_INDEX is >%i<.", (int)dec_interface_setup_data_[TEENSY_ID].num_block_leds, (int)BLOCK_INDEX);
    // ROS_DEBUG("Beam >%i< - >%i< >%i< >%i< >%i<", (int)i, block_beam_led_values_(RED_OFFSET, i), block_beam_led_values_(GREEN_OFFSET, i), block_beam_led_values_(BLUE_OFFSET, i), block_beam_led_values_(ALPHA_OFFSET, i));
    float alpha = block_beam_led_values_(BRIGHTNESS_OFFSET, i) / 255.0f;
    ROS_ASSERT(!(alpha < 0) && !(alpha > 1.0f));
    dec_interface_light_data_[TEENSY_ID].block_leds[BLOCK_INDEX].red = (uint8_t) static_cast<float>(block_beam_led_values_(RED_OFFSET, i)) * alpha;
    dec_interface_light_data_[TEENSY_ID].block_leds[BLOCK_INDEX].green = (uint8_t) static_cast<float>(block_beam_led_values_(GREEN_OFFSET, i)) * alpha;
    dec_interface_light_data_[TEENSY_ID].block_leds[BLOCK_INDEX].blue = (uint8_t) static_cast<float>(block_beam_led_values_(BLUE_OFFSET, i)) * alpha;
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
    // ROS_DEBUG("PIXEL BEAM >%i<: TEENSY_ID %i | PIXEL_STRIP %i | STRIP_INDEX %i |", (int)i, (int)TEENSY_ID, (int)PIXEL_INDEX, (int)STRIP_INDEX);
    float alpha = pixel_beam_led_values_(BRIGHTNESS_OFFSET, i) / 255.0f;
    ROS_ASSERT(!(alpha < 0) && !(alpha > 1.0f));
    dec_interface_light_data_[TEENSY_ID].pixel_leds[PIXEL_INDEX].red[STRIP_INDEX] = (uint8_t) static_cast<float>(pixel_beam_led_values_(RED_OFFSET, i)) * alpha;
    dec_interface_light_data_[TEENSY_ID].pixel_leds[PIXEL_INDEX].green[STRIP_INDEX] = (uint8_t) static_cast<float>(pixel_beam_led_values_(GREEN_OFFSET, i)) * alpha;
    dec_interface_light_data_[TEENSY_ID].pixel_leds[PIXEL_INDEX].blue[STRIP_INDEX] = (uint8_t) static_cast<float>(pixel_beam_led_values_(BLUE_OFFSET, i)) * alpha;
  }

  // send/receive light/sensor data
  //  for (unsigned int i = 0; i < number_of_teensys_; ++i)
  //  {
  //    bool send = true;
  //    for (unsigned int j = 0; send && j < list_of_teensys_to_exclude_from_communication_.size(); ++j)
  //    {
  //      if (i == list_of_teensys_to_exclude_from_communication_[j])
  //      {
  //        send = false;
  //      }
  //    }
  //    if (send)
  //    {
  //      if(!dec_interface_->sendLightData(i, dec_interface_light_data_[i], dec_interface_setup_data_[i]))
  //      {
  //        ROS_WARN("Missed send/receive cycle for node >%i<.", i);
  //      }
  //    }
  //  }

  if(!dec_interface_->sendLightData(send_flags_, dec_interface_light_data_, dec_interface_setup_data_))
  {
    ROS_WARN("Missed send/receive cycle");
  }

  return true;
}

bool DecData::copyLightDataFromStructure()
{
  return true;
}

}
