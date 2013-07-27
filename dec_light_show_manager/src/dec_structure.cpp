/*
 * dec_structure.cpp
 *
 *  Created on: Jul 18, 2013
 *      Author: pastor
 */

#include <dec_utilities/param_server.h>
#include <dec_utilities/assert.h>

#include <dec_light_show_manager/dec_structure.h>

using namespace dec_utilities;

namespace dec_light_show_manager
{

bool DecStructure::initialize(ros::NodeHandle& node_handle)
{
  // read structure configuration (nodes, beams, panels, ...)
  ROS_DEBUG("Reading node configurations.");
  ROS_VERIFY(read(node_handle, nodes_));
  ROS_DEBUG("Reading beam configurations.");
  ROS_VERIFY(read(node_handle, beams_));

  // error checking
  for (unsigned int i = 0; i < nodes_.size(); ++i)
  {
    ROS_ASSERT(nodes_[i].poses_.size() == 1);
  }
  for (unsigned int i = 0; i < beams_.size(); ++i)
  {
    ROS_ASSERT(beams_[i].poses_.size() == 1);
  }

  ROS_VERIFY(dec_utilities::read(node_handle, "nodes_size", nodes_size_));
  ROS_VERIFY(dec_utilities::read(node_handle, "nodes_color", nodes_color_));
  ROS_VERIFY(dec_utilities::read(node_handle, "beams_size", beams_size_));
  ROS_VERIFY(dec_utilities::read(node_handle, "beams_color", beams_color_));

  ROS_VERIFY(dec_utilities::read(node_handle, "block_nodes_size", block_light_nodes_size_));
  ROS_VERIFY(dec_utilities::read(node_handle, "block_beams_size", block_light_beams_size_));
  ROS_VERIFY(dec_utilities::read(node_handle, "pixel_beams_size", pixel_light_beams_size_));
  ROS_VERIFY(dec_utilities::read(node_handle, "sensors_size", sensors_size_));

  ROS_DEBUG("Reading sensor configurations.");
  ROS_VERIFY(read(node_handle, sensors_));
  ROS_DEBUG("Reading light node configurations.");
  ROS_VERIFY(read(node_handle, block_light_nodes_));
  ROS_DEBUG("Reading block light beam configurations.");
  ROS_VERIFY(read(node_handle, block_light_beams_));
  ROS_DEBUG("Reading pixel light beam configurations.");
  ROS_VERIFY(read(node_handle, pixel_light_beams_));

  setNumberOfTeensys();
  setPins(node_handle);
  setupTeensyMap();
  ROS_VERIFY(isUnique());

  dec_interface_.reset(new dec_udp::DecInterface((uint8_t)number_of_teensys_));
  dec_interface_light_data_.resize((size_t)number_of_teensys_);
  for (unsigned int i = 0; i < dec_interface_setup_data_.size(); ++i)
  {
    allocatePixelData(&(dec_interface_setup_data_[i]), &(dec_interface_light_data_[i]));
    ROS_ASSERT(dec_interface_light_data_[i].pixel_memory_allocated == (uint8_t)1);
  }

  // ROS_WARN("Allocated >%i< bytes for setup data.", (int)sizeof(_setup_data));
  // ROS_WARN("Allocated >%i< bytes for sensor data.", (int)sizeof(_sensor_data));
  // ROS_WARN("Allocated >%i< bytes for light data.", (int)sizeof(_light_data));

  return true;
}

bool DecStructure::isUnique()
{

  std::map<unsigned int, int> uniqueness_test_map;
  for (unsigned int i = 0; i < block_light_nodes_.size(); ++i)
  {
    for (unsigned int j = 0; j < block_light_nodes_[i].getNumComponents(); ++j)
    {
      unsigned int node_id = block_light_nodes_[i].nodes_[j];
      if( uniqueness_test_map.find(node_id) != uniqueness_test_map.end() )
      {
        ROS_ERROR("Multiple block light nodes with id >%i< contained.", node_id);
        return false;
      }
      uniqueness_test_map.insert(std::pair<unsigned int, int>(node_id, 0));
    }
  }

  std::vector<std::pair<unsigned int, unsigned int> > uniqueness_test_vector;
  for (unsigned int i = 0; i < block_light_beams_.size(); ++i)
  {
    for (unsigned int j = 0; j < block_light_beams_[i].getNumComponents(); ++j)
    {
      for (unsigned int n = 0; n < uniqueness_test_vector.size(); ++n)
      {
        if (((block_light_beams_[i].nodes_[j].first == uniqueness_test_vector[n].first)
            && (block_light_beams_[i].nodes_[j].second == uniqueness_test_vector[n].second))
            || ((block_light_beams_[i].nodes_[j].first == uniqueness_test_vector[n].second)
                && (block_light_beams_[i].nodes_[j].second == uniqueness_test_vector[n].first)))
        {
          ROS_ERROR("Multiple block beams from >%i< to >%i< contained.",
                    block_light_beams_[i].nodes_[j].first, block_light_beams_[i].nodes_[j].second);
          return false;
        }
      }
      uniqueness_test_vector.push_back(block_light_beams_[i].nodes_[j]);
    }
  }

  uniqueness_test_vector.clear();
  for (unsigned int i = 0; i < pixel_light_beams_.size(); ++i)
  {
    for (unsigned int j = 0; j < pixel_light_beams_[i].getNumComponents(); ++j)
    {
      for (unsigned int n = 0; n < uniqueness_test_vector.size(); ++n)
      {
        if (((pixel_light_beams_[i].nodes_[j].first == uniqueness_test_vector[n].first)
            && (pixel_light_beams_[i].nodes_[j].second == uniqueness_test_vector[n].second))
            || ((pixel_light_beams_[i].nodes_[j].first == uniqueness_test_vector[n].second)
                && (pixel_light_beams_[i].nodes_[j].second == uniqueness_test_vector[n].first)))
        {
          ROS_ERROR("Multiple pixel beams from >%i< to >%i< contained.",
                    pixel_light_beams_[i].nodes_[j].first, pixel_light_beams_[i].nodes_[j].second);
          return false;
        }
      }
      uniqueness_test_vector.push_back(pixel_light_beams_[i].nodes_[j]);
    }
  }

  return true;
}

void DecStructure::setupTeensyMap()
{
  ROS_DEBUG("Setting up teensy map.");
  dec_interface_setup_data_.resize(number_of_teensys_);
  for (unsigned int i = 0; i < dec_interface_setup_data_.size(); ++i)
  {
    resetSetupData(&(dec_interface_setup_data_[i]));
  }

  std::vector<unsigned int> sensor_pin_counter(number_of_teensys_, 0);

  ROS_DEBUG("Setting up sensor map.");
  sensor_to_teensy_map_.clear();
  for (unsigned int i = 0; i < sensors_.size(); ++i)
  {
    std::pair<unsigned int, unsigned int> teensy_id_and_sensor_id_pair;
    const uint8_t TEENSY_ID = sensors_[i].getTeeynsyId();
    sensors_[i].setPin(sensor_pin_counter[TEENSY_ID]);
    teensy_id_and_sensor_id_pair.first = TEENSY_ID;
    teensy_id_and_sensor_id_pair.second = dec_interface_setup_data_[TEENSY_ID].num_sensors;
    dec_interface_setup_data_[TEENSY_ID].sensors[dec_interface_setup_data_[TEENSY_ID].num_sensors].pin =
        (uint8_t)sensor_pins_[sensor_pin_counter[TEENSY_ID]];
    dec_interface_setup_data_[TEENSY_ID].num_sensors++;
    sensor_to_teensy_map_.push_back(teensy_id_and_sensor_id_pair);
    ROS_DEBUG("Sensor >%i< is connected to teensy >%u< at pin >%u<.", (int)i,
             (int)sensor_to_teensy_map_[i].first, (int)sensor_to_teensy_map_[i].second);
    sensor_pin_counter[TEENSY_ID]++;
  }
  for(unsigned int i = 0; i < sensor_pin_counter.size(); ++i)
  {
    ROS_ASSERT_MSG(sensor_pin_counter[i] < max_number_of_sensors_per_teensy_,
                   "Number of sensors connected to teensy >%i< is >%i< and exceeds limit >%i<.",
                   (int)i, (int)sensor_pin_counter[i], (int)max_number_of_sensors_per_teensy_);
  }

  std::vector<unsigned int> total_num_leds_at_each_strip(DEC_MAX_NUMBER_OF_LED_STRIPS_PER_NODE, 0);
  std::vector<std::vector<unsigned int> > num_leds_at_each_strip;
  num_leds_at_each_strip.resize(number_of_teensys_, total_num_leds_at_each_strip);

  std::vector<unsigned int> light_pin_counter(number_of_teensys_, 0);
  std::vector<unsigned int> block_counter(number_of_teensys_, 0);

  num_block_node_leds_per_teensy_.resize(number_of_teensys_, 0.0);

  ROS_DEBUG("Setting up light node map.");
  light_node_leds_to_teensy_map_.clear();
  for (unsigned int i = 0; i < block_light_nodes_.size(); ++i)
  {
    const unsigned int TEENSY_ID = block_light_nodes_[i].getTeeynsyId();
    block_light_nodes_[i].setPin(light_pin_counter[TEENSY_ID]);
    num_leds_at_each_strip[TEENSY_ID][light_pin_counter[TEENSY_ID]] += block_light_nodes_[i].getTotalNumLeds();
    bool new_strip = true;
    for (unsigned int j = 0; j < block_light_nodes_[i].getNumComponents(); ++j)
    {
      std::pair<unsigned int, unsigned int> teensy_id_to_led_info_pair;
      teensy_id_to_led_info_pair.first = TEENSY_ID;
      teensy_id_to_led_info_pair.second = block_counter[TEENSY_ID];
      light_node_leds_to_teensy_map_.push_back(teensy_id_to_led_info_pair);

      if(new_strip)
      {
        new_strip = false;
        dec_interface_setup_data_[TEENSY_ID].block_leds[dec_interface_setup_data_[TEENSY_ID].num_block_leds].index = (uint8_t)0;
      }
      else
      {
        dec_interface_setup_data_[TEENSY_ID].block_leds[dec_interface_setup_data_[TEENSY_ID].num_block_leds].index =
            dec_interface_setup_data_[TEENSY_ID].block_leds[dec_interface_setup_data_[TEENSY_ID].num_block_leds - 1].index +
            dec_interface_setup_data_[TEENSY_ID].block_leds[dec_interface_setup_data_[TEENSY_ID].num_block_leds - 1].num_blocks;
      }
      dec_interface_setup_data_[TEENSY_ID].block_leds[dec_interface_setup_data_[TEENSY_ID].num_block_leds].num_blocks = (uint8_t)block_light_nodes_[i].getNumLeds(j);
      dec_interface_setup_data_[TEENSY_ID].block_leds[dec_interface_setup_data_[TEENSY_ID].num_block_leds].pin = (uint8_t)light_pin_counter[TEENSY_ID];
      ROS_INFO("Block light node >%i< at pin >%i< connected to teensy >%i<. LED index >%i< num_leds >%i<.",
               (int)dec_interface_setup_data_[TEENSY_ID].num_block_leds,
               (int)dec_interface_setup_data_[TEENSY_ID].block_leds[dec_interface_setup_data_[TEENSY_ID].num_block_leds].pin,
               (int)TEENSY_ID,
               (int)dec_interface_setup_data_[TEENSY_ID].block_leds[dec_interface_setup_data_[TEENSY_ID].num_block_leds].index,
               (int)dec_interface_setup_data_[TEENSY_ID].block_leds[dec_interface_setup_data_[TEENSY_ID].num_block_leds].num_blocks);
      dec_interface_setup_data_[TEENSY_ID].num_block_leds++;
      block_counter[TEENSY_ID]++;
    }
    num_block_node_leds_per_teensy_[TEENSY_ID] = dec_interface_setup_data_[TEENSY_ID].num_block_leds;
    light_pin_counter[TEENSY_ID]++;
  }
  ROS_ASSERT(light_node_leds_to_teensy_map_.size() == total_num_node_leds_);

  num_block_beam_leds_per_teensy_.resize(number_of_teensys_, 0.0);

  ROS_DEBUG("Setting up block light beam map.");
  block_light_beam_leds_to_teensy_map_.clear();
  for (unsigned int i = 0; i < block_light_beams_.size(); ++i)
  {
    const unsigned int TEENSY_ID = block_light_beams_[i].getTeeynsyId();
    block_light_beams_[i].setPin(light_pin_counter[TEENSY_ID]);
    num_leds_at_each_strip[TEENSY_ID][light_pin_counter[TEENSY_ID]] += block_light_beams_[i].getTotalNumLeds();
    bool new_strip = true;
    for (unsigned int j = 0; j < block_light_beams_[i].getNumComponents(); ++j)
    {
      std::pair<unsigned int, unsigned int> teensy_id_to_led_info_pair;
      teensy_id_to_led_info_pair.first = TEENSY_ID;
      teensy_id_to_led_info_pair.second = block_counter[TEENSY_ID];
      block_light_beam_leds_to_teensy_map_.push_back(teensy_id_to_led_info_pair);

      if(new_strip)
      {
        new_strip = false;
        dec_interface_setup_data_[TEENSY_ID].block_leds[dec_interface_setup_data_[TEENSY_ID].num_block_leds].index = (uint8_t)0;
      }
      else
      {
        dec_interface_setup_data_[TEENSY_ID].block_leds[dec_interface_setup_data_[TEENSY_ID].num_block_leds].index =
            dec_interface_setup_data_[TEENSY_ID].block_leds[dec_interface_setup_data_[TEENSY_ID].num_block_leds - 1].index +
            dec_interface_setup_data_[TEENSY_ID].block_leds[dec_interface_setup_data_[TEENSY_ID].num_block_leds - 1].num_blocks;
      }
      dec_interface_setup_data_[TEENSY_ID].block_leds[dec_interface_setup_data_[TEENSY_ID].num_block_leds].num_blocks = (uint8_t)block_light_beams_[i].getNumLeds(j);
      dec_interface_setup_data_[TEENSY_ID].block_leds[dec_interface_setup_data_[TEENSY_ID].num_block_leds].pin = (uint8_t)light_pin_counter[TEENSY_ID];
      ROS_INFO("Block light beam at pin >%i< connected to teensy >%i<. LED index >%i< num_leds >%i<.",
               (int)dec_interface_setup_data_[TEENSY_ID].block_leds[dec_interface_setup_data_[TEENSY_ID].num_block_leds].pin,
               (int)TEENSY_ID,
               (int)dec_interface_setup_data_[TEENSY_ID].block_leds[dec_interface_setup_data_[TEENSY_ID].num_block_leds].index,
               (int)dec_interface_setup_data_[TEENSY_ID].block_leds[dec_interface_setup_data_[TEENSY_ID].num_block_leds].num_blocks);
      dec_interface_setup_data_[TEENSY_ID].num_block_leds++;
      block_counter[TEENSY_ID]++;
    }
    num_block_beam_leds_per_teensy_[TEENSY_ID] = (dec_interface_setup_data_[TEENSY_ID].num_block_leds - num_block_node_leds_per_teensy_[TEENSY_ID]);
    light_pin_counter[TEENSY_ID]++;
  }
  for (unsigned int i = 0; i < number_of_teensys_; ++i)
  {
    ROS_ASSERT_MSG((unsigned int)dec_interface_setup_data_[i].num_block_leds < MAX_NUMBER_OF_BLOCKS_PER_TEENSY,
                   "Max number of allowed blocks is >%i<, however, >%i< are specified.",
                   (int)MAX_NUMBER_OF_BLOCKS_PER_TEENSY, (int)dec_interface_setup_data_[i].num_block_leds);
  }
  ROS_ASSERT(block_light_beam_leds_to_teensy_map_.size() == total_num_block_beam_leds_);

  ROS_DEBUG("Setting up pixel light beam map.");
  unsigned int led_index = 0;
  pixel_light_beam_leds_to_teensy_map_.clear();
  for (unsigned int i = 0; i < pixel_light_beams_.size(); ++i)
  {
    const unsigned int TEENSY_ID = pixel_light_beams_[i].getTeeynsyId();
    pixel_light_beams_[i].setPin(light_pin_counter[TEENSY_ID]);
    num_leds_at_each_strip[TEENSY_ID][light_pin_counter[TEENSY_ID]] += pixel_light_beams_[i].getTotalNumLeds();
    bool new_strip = true;
    for (unsigned int j = 0; j < pixel_light_beams_[i].getNumComponents(); ++j)
    {
      for (unsigned int k = 0; k < pixel_light_beams_[i].getNumLeds(j); ++k)
      {
        std::pair<unsigned int, std::pair<unsigned int, unsigned int> > teensy_id_to_led_info_pair;
        teensy_id_to_led_info_pair.first = TEENSY_ID;
        std::pair<unsigned int, unsigned int> strip_id_and_led_id_pair;
        strip_id_and_led_id_pair.first = j;
        strip_id_and_led_id_pair.second = k;
        teensy_id_to_led_info_pair.second = strip_id_and_led_id_pair;
        pixel_light_beam_leds_to_teensy_map_.push_back(teensy_id_to_led_info_pair);
        led_index++;
      }
      ROS_ASSERT(j < MAX_NUMBER_OF_PIXELS_PER_TEENSY);
      if(new_strip)
      {
        new_strip = false;
        dec_interface_setup_data_[TEENSY_ID].pixel_leds[j].index = (uint8_t)0;
      }
      else
      {
        dec_interface_setup_data_[TEENSY_ID].pixel_leds[j].index =
            dec_interface_setup_data_[TEENSY_ID].pixel_leds[j - 1].index +
            dec_interface_setup_data_[TEENSY_ID].pixel_leds[j - 1].num_pixels;
      }
      dec_interface_setup_data_[TEENSY_ID].pixel_leds[j].num_pixels = (uint8_t)pixel_light_beams_[i].getNumLeds(j);
      dec_interface_setup_data_[TEENSY_ID].pixel_leds[j].pin = light_pin_counter[TEENSY_ID];
      ROS_DEBUG("Pixel light beam at pin >%i< connected to teensy >%i< with index >%i< num_leds >%i<.",
               (int)dec_interface_setup_data_[TEENSY_ID].pixel_leds[j].pin,
               (int)pixel_light_beam_leds_to_teensy_map_[led_index].first,
               (int)dec_interface_setup_data_[TEENSY_ID].pixel_leds[j].index,
               (int)dec_interface_setup_data_[TEENSY_ID].pixel_leds[j].num_pixels);
      dec_interface_setup_data_[TEENSY_ID].num_pixel_leds++;
    }
    light_pin_counter[TEENSY_ID]++;
  }
  for (unsigned int i = 0; i < number_of_teensys_; ++i)
  {
    ROS_ASSERT_MSG((unsigned int)dec_interface_setup_data_[i].num_pixel_leds < MAX_NUMBER_OF_PIXELS_PER_LIGHT_STRIP,
                   "Max number of allowed pixels is >%i<, however, >%i< are specified.",
                   (int)MAX_NUMBER_OF_PIXELS_PER_LIGHT_STRIP, (int)dec_interface_setup_data_[i].num_pixel_leds);
  }
  ROS_ASSERT_MSG(led_index == total_num_pixel_beam_leds_, "LED index is >%i< and total number of pixel beam leds is >%i<.",
                 (int)led_index, (int)total_num_pixel_beam_leds_);

  ROS_ASSERT(pixel_light_beam_leds_to_teensy_map_.size() == total_num_pixel_beam_leds_);

  std::vector<unsigned int> total_number_of_strips(number_of_teensys_);

  for (unsigned int i = 0; i < number_of_teensys_; ++i)
  {
    for (unsigned int j = 0; j < num_leds_at_each_strip[i].size(); ++j)
    {
      if (num_leds_at_each_strip[i][j] > 0)
      {
        total_number_of_strips[i]++;
      }
    }
    ROS_ASSERT_MSG(total_number_of_strips[i] <= DEC_MAX_NUMBER_OF_LED_STRIPS_PER_NODE,
                   "Maximum number of light strips is >%i<, however >%i< have been specified for teensy >%i<.",
                   (int)DEC_MAX_NUMBER_OF_LED_STRIPS_PER_NODE, (int)total_number_of_strips[i], (int)i);
    dec_interface_setup_data_[i].num_strips_used = (uint8_t)total_number_of_strips[i];
  }
  for (unsigned int i = 0; i < number_of_teensys_; ++i)
  {
    for (unsigned int j = 0; j < num_leds_at_each_strip[i].size(); ++j)
    {
      dec_interface_setup_data_[i].strip_setup[j].total_num_leds_at_strip = (uint8_t)num_leds_at_each_strip[i][j];
    }
  }
}

void DecStructure::offsetNodePositions(std::vector<geometry_msgs::Point>& node_positions,
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

void DecStructure::setNumberOfTeensys()
{
  std::map<unsigned int, int> used_ids;
  for (unsigned int i = 0; i < block_light_nodes_.size(); ++i)
  {
    used_ids.insert(std::pair<unsigned int, int>(block_light_nodes_[i].getTeeynsyId(), 0));
  }
  for (unsigned int i = 0; i < block_light_beams_.size(); ++i)
  {
    used_ids.insert(std::pair<unsigned int, int>(block_light_beams_[i].getTeeynsyId(), 0));
  }
  for (unsigned int i = 0; i < pixel_light_beams_.size(); ++i)
  {
    used_ids.insert(std::pair<unsigned int, int>(pixel_light_beams_[i].getTeeynsyId(), 0));
  }
  for (unsigned int i = 0; i < sensors_.size(); ++i)
  {
    used_ids.insert(std::pair<unsigned int, int>(sensors_[i].getTeeynsyId(), 0));
  }

  number_of_teensys_ = used_ids.size();
  // error checking
  ROS_ASSERT_MSG(number_of_teensys_ > 0 && number_of_teensys_ <= 255,
                 "Number of teensys specified >%i< is invalid. It need to be within [1..255].", number_of_teensys_);
  for (std::map<unsigned int, int>::const_iterator ci = used_ids.begin(); ci != used_ids.end(); ++ci)
  {
    ROS_ASSERT_MSG(number_of_teensys_ > ci->first, "Teensy id >%i< is invalud. Only >%i< teensys present in the structure.",
                   (int)ci->first, (int)number_of_teensys_);
  }
}

void DecStructure::setPins(ros::NodeHandle node_handle)
{
  ROS_VERIFY(dec_utilities::read(node_handle, "sensor_pins", sensor_pins_));
  for (unsigned int i = 0; i < sensor_pins_.size(); ++i)
  {
    const unsigned int SENSOR_PINS[29] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 17, 18, 19, 20, 21, 22, 23,
                                 28, 29, 30, 31, 32, 33, 34, 35, 36, 37}; // Teensy++ 2.0 digital pins that are not PWM capable
    bool found = false;
    for (unsigned int j = 0; !found && j < 29; ++j)
    {
      if (SENSOR_PINS[j] == sensor_pins_[i])
      {
        found = true;
      }
    }
    ROS_ASSERT_MSG(found, "Invalid sensor pin >%i< specified. This pin is not a valid digital pin (excluding PWM pins) on a Teensy++ 2.0 !", sensor_pins_[i]);
  }
  max_number_of_sensors_per_teensy_ = sensor_pins_.size();
  ROS_ASSERT_MSG(max_number_of_sensors_per_teensy_ <= 255, "Maximum number of sensors specified >%i< is invalid.",
                 max_number_of_sensors_per_teensy_);

  ROS_VERIFY(dec_utilities::read(node_handle, "light_pins", light_pins_));
  for (unsigned int i = 0; i < light_pins_.size(); ++i)
  {
    const unsigned int PWM_PINS[9] = {0, 1, 14, 15, 16, 24, 25, 26, 27}; // Teensy++ 2.0 PWM pins
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
  max_number_of_light_strips_per_teensy_ = light_pins_.size();
  ROS_ASSERT_MSG(max_number_of_light_strips_per_teensy_ <= 255,
                 "Maximum number of light strips specified >%i< is invalid.", max_number_of_light_strips_per_teensy_);

  total_num_node_leds_ = 0;
  for (unsigned int i = 0; i < block_light_nodes_.size(); ++i)
  {
    ROS_ASSERT(block_light_nodes_[i].getTotalNumLeds() <= MAX_NUMBER_OF_PIXELS_PER_LIGHT_STRIP);
    total_num_node_leds_ += block_light_nodes_[i].getNumComponents();
  }
  ROS_ASSERT_MSG(total_num_node_leds_ > 0, "No block light node read from configuration.");

  total_num_block_beam_leds_ = 0;
  for (unsigned int i = 0; i < block_light_beams_.size(); ++i)
  {
    ROS_ASSERT(block_light_beams_[i].getTotalNumLeds() <= MAX_NUMBER_OF_PIXELS_PER_LIGHT_STRIP);
    total_num_block_beam_leds_ += block_light_beams_[i].getNumComponents();
  }
  ROS_WARN_COND(total_num_block_beam_leds_ == 0, "No block light beam read from configuration.");

  total_num_pixel_beam_leds_ = 0;
  for (unsigned int i = 0; i < pixel_light_beams_.size(); ++i)
  {
    ROS_ASSERT(pixel_light_beams_[i].getTotalNumLeds() <= MAX_NUMBER_OF_PIXELS_PER_LIGHT_STRIP);
    total_num_pixel_beam_leds_ += pixel_light_beams_[i].getTotalNumLeds();
  }
  ROS_WARN_COND(total_num_pixel_beam_leds_ == 0, "No pixel light beam read from configuration.");

  // each pin is a separate sensor, not each component
  total_num_sensors_ = sensors_.size();
  ROS_ASSERT_MSG(total_num_sensors_ > 0, "No sensors read from configuration.");

  ROS_INFO("Structure contains >%i< teensys, >%i< node lights, >%i< beam lights, >%i< pixel lights, and >%i< sensors.",
           (int)number_of_teensys_, (int)total_num_node_leds_, (int)total_num_block_beam_leds_, (int)total_num_pixel_beam_leds_, (int)total_num_sensors_);
}

XmlRpc::XmlRpcValue DecStructure::get(ros::NodeHandle& node_handle, const std::string& key)
{
  XmlRpc::XmlRpcValue rpc_values;
  ROS_VERIFY(node_handle.hasParam(key));
  ROS_VERIFY(node_handle.getParam(key, rpc_values));
  ROS_ASSERT(rpc_values.getType() == XmlRpc::XmlRpcValue::TypeArray);
  return rpc_values;
}

bool DecStructure::read(ros::NodeHandle& node_handle, std::vector<Node>& nodes)
{
  XmlRpc::XmlRpcValue rpc_values = get(node_handle, "node_positions");
  int id = 0;
  for (int i = 0; i < rpc_values.size(); ++i)
  {
    Node node;
    ROS_VERIFY(node.initialize(rpc_values[i], id));
    id += node.getNumComponents();
    ROS_ASSERT(node.poses_.size() == 1);
    node_positions_.push_back(node.getPose(0).position);
    nodes.push_back(node);
  }

  ROS_ASSERT_MSG(!node_positions_.empty(), "No node positions specified. Structure must contain at least one node.");
  int offset_node_index = 0;
  ROS_VERIFY(dec_utilities::read(node_handle, "offset_node_index", offset_node_index));
  ROS_ASSERT_MSG(offset_node_index >= 0 && offset_node_index < (int)node_positions_.size(),
             "Offset node index >%i< needs to be within [0..%i].", offset_node_index, (int)node_positions_.size()-1);
  offsetNodePositions(node_positions_, offset_node_index);
  for(unsigned int i = 0; i < nodes_.size(); ++i)
  {
    nodes_[i].update(node_positions_[i]);
  }

  return true;
}

bool DecStructure::read(ros::NodeHandle& node_handle, std::vector<Beam>& beams)
{
  XmlRpc::XmlRpcValue rpc_values = get(node_handle, "beams");
  int id = 0;
  for (int i = 0; i < rpc_values.size(); ++i)
  {
    Beam beam;
    ROS_VERIFY(beam.initialize(rpc_values[i], id, node_positions_));
    id += beam.getNumComponents();
    beams.push_back(beam);
  }
  beam_poses_.clear();
  for (unsigned int i = 0; i < beams.size(); ++i)
  {
    for (unsigned int j = 0; j < beams[i].getNumComponents(); ++j)
    {
      beam_poses_.push_back(beams[i].getPose(j));
    }
  }
  return true;
}

bool DecStructure::read(ros::NodeHandle& node_handle, std::vector<Sensor>& sensors)
{
  XmlRpc::XmlRpcValue rpc_values = get(node_handle, "sensors");
  int id = 0;
  for (int i = 0; i < rpc_values.size(); ++i)
  {
    Sensor sensor;
    ROS_VERIFY(sensor.initialize(rpc_values[i], id, node_positions_));
    id += sensor.getNumComponents();
    sensors.push_back(sensor);
  }
  sensor_poses_.clear();
  for (unsigned int i = 0; i < sensors.size(); ++i)
  {
    for (unsigned int j = 0; j < sensors[i].getNumComponents(); ++j)
    {
      sensor_poses_.push_back(sensors[i].getPose(j));
    }
  }
  return true;
}

bool DecStructure::read(ros::NodeHandle& node_handle, std::vector<LightNode>& light_nodes)
{
  XmlRpc::XmlRpcValue rpc_values = get(node_handle, "light_nodes");
  int id = 0;
  for (int i = 0; i < rpc_values.size(); ++i)
  {
    LightNode light_node;
    ROS_VERIFY(light_node.initialize(rpc_values[i], id, node_positions_));
    id += light_node.getNumComponents();
    light_nodes.push_back(light_node);
  }
  block_light_node_positions_.clear();
  for (unsigned int i = 0; i < light_nodes.size(); ++i)
  {
    for (unsigned int j = 0; j < light_nodes[i].getNumComponents(); ++j)
    {
      block_light_node_positions_.push_back(light_nodes[i].getPosition(j));
    }
  }
  return true;
}

bool DecStructure::read(ros::NodeHandle& node_handle, std::vector<BlockLightBeam>& block_light_beams)
{
  const std::string KEY = "block_light_beams";
  if (!node_handle.hasParam(KEY))
  {
    ROS_WARN("No >%s< read from configuration.", KEY.c_str());
    return true;
  }
  XmlRpc::XmlRpcValue rpc_values = get(node_handle, KEY);
  int id = 0;
  for (int i = 0; i < rpc_values.size(); ++i)
  {
    BlockLightBeam block_light_beam;
    ROS_VERIFY(block_light_beam.initialize(rpc_values[i], id, node_positions_));
    id += block_light_beam.getNumComponents();
    block_light_beams.push_back(block_light_beam);
  }
  block_light_beam_poses_.clear();
  for (unsigned int i = 0; i < block_light_beams.size(); ++i)
  {
    for (unsigned int j = 0; j < block_light_beams[i].getNumComponents(); ++j)
    {
      block_light_beam_poses_.push_back(block_light_beams[i].getPose(j));
    }
  }
  return true;
}

bool DecStructure::read(ros::NodeHandle& node_handle, std::vector<PixelLightBeam>& pixel_light_beams)
{
  const std::string KEY = "pixel_light_beams";
  if (!node_handle.hasParam(KEY))
  {
    ROS_WARN("No >%s< read from configuration.", KEY.c_str());
    return true;
  }
  XmlRpc::XmlRpcValue rpc_values = get(node_handle, KEY);
  int id = 0;
  for (int i = 0; i < rpc_values.size(); ++i)
  {
    PixelLightBeam pixel_light_beam;
    ROS_VERIFY(pixel_light_beam.initialize(rpc_values[i], id, node_positions_));
    id += pixel_light_beam.getNumComponents();
    pixel_light_beams.push_back(pixel_light_beam);
  }
  pixel_light_beam_poses_.clear();
  pixel_light_beam_led_poses_.clear();
  for (unsigned int i = 0; i < pixel_light_beams.size(); ++i)
  {
    for (unsigned int j = 0; j < pixel_light_beams[i].getNumComponents(); ++j)
    {
      pixel_light_beam_poses_.push_back(pixel_light_beams[i].getPose(j));
    }
    for (unsigned int n = 0; n < pixel_light_beams[i].pixel_poses_.size(); ++n)
    {
      pixel_light_beam_led_poses_.insert(pixel_light_beam_led_poses_.end(),
                                         pixel_light_beams_[i].pixel_poses_[n].begin(),
                                         pixel_light_beams_[i].pixel_poses_[n].end());
    }
  }
  return true;
}

}
