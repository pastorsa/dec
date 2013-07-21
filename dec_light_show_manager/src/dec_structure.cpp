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
  ROS_INFO("Reading node configurations.");
  ROS_VERIFY(read(node_handle, nodes_));
  ROS_INFO("Reading beam configurations.");
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

  ROS_VERIFY(dec_utilities::read(node_handle, "light_nodes_size", light_nodes_size_));
  ROS_VERIFY(dec_utilities::read(node_handle, "light_beams_size", light_beams_size_));
  ROS_ASSERT(light_beams_size_.z > 0.0 && !(light_beams_size_.z > 1.0));
  ROS_VERIFY(dec_utilities::read(node_handle, "sensors_size", sensors_size_));
  ROS_ASSERT(sensors_size_.z > 0.0 && !(sensors_size_.z > 1.0));

  ROS_INFO("Reading sensor configurations.");
  ROS_VERIFY(read(node_handle, capactive_sensors_));
  ROS_INFO("Reading light node configurations.");
  ROS_VERIFY(read(node_handle, block_light_nodes_));
  std::map<unsigned int, int> uniqueness_test_map;
  for (unsigned int i = 0; i < block_light_nodes_.size(); ++i)
  {
    // uniqueness_test_map.find(block_light_nodes_)
  }

  ROS_INFO("Reading block light beam configurations.");
  ROS_VERIFY(read(node_handle, block_light_beams_));
  ROS_INFO("Reading pixel light beam configurations.");
  ROS_VERIFY(read(node_handle, pixel_light_beams_));

  setNumberOfTeensys();
  setPins(node_handle);
  setupTeensyMap();

  return true;
}

void DecStructure::setupTeensyMap()
{
  ROS_INFO("Setting up teensy map.");
  setup_data_.resize(number_of_teensys_);
  for (unsigned int i = 0; i < setup_data_.size(); ++i)
  {
    setup_data_[i].num_sensors = (uint8_t)0;
    setup_data_[i].num_block_leds = (uint8_t)0;
    setup_data_[i].num_pixel_leds = (uint8_t)0;
    for (unsigned int j = 0; j < DEC_MAX_NUMBER_OF_SENSORS_PER_NODE; ++j)
    {
      setup_data_[i].sensors[j].pin = (uint8_t)0;
    }
    for (unsigned int j = 0; j < DEC_MAX_NUMBER_OF_BLOCKS_PER_LIGHT_STRIP; ++j)
    {
      setup_data_[i].block_leds[j].index = (uint8_t)0;
      setup_data_[i].block_leds[j].num_leds = (uint8_t)0;
      setup_data_[i].block_leds[j].pin = (uint8_t)0;
    }
    for (unsigned int j = 0; j < DEC_MAX_NUMBER_OF_PIXELS_PER_LIGHT_STRIP; ++j)
    {
      setup_data_[i].pixel_leds[j].index = (uint8_t)0;
      setup_data_[i].pixel_leds[j].num_leds = (uint8_t)0;
      setup_data_[i].pixel_leds[j].pin = (uint8_t)0;
    }
  }

  ROS_INFO("Setting up sensor map.");
  sensor_to_teensy_map_.clear();
  for (unsigned int i = 0; i < capactive_sensors_.size(); ++i)
  {
    std::pair<unsigned int, unsigned int> teensy_id_and_sensor_id_pair;
    const uint8_t TEENSY_ID = capactive_sensors_[i].getTeeynsyId();
    teensy_id_and_sensor_id_pair.first = TEENSY_ID;
    teensy_id_and_sensor_id_pair.second = setup_data_[TEENSY_ID].num_sensors;
    setup_data_[TEENSY_ID].num_sensors++;
    sensor_to_teensy_map_.push_back(teensy_id_and_sensor_id_pair);
    ROS_INFO("Sensor >%i< is connected to teensy >%u< at pin >%u<.", (int)i,
             (int)sensor_to_teensy_map_[i].first, (int)sensor_to_teensy_map_[i].second);
  }

  ROS_INFO("Setting up light node map.");
  light_node_leds_to_teensy_map_.clear();
  for (unsigned int i = 0; i < block_light_nodes_.size(); ++i)
  {
    const unsigned int TEENSY_ID = block_light_nodes_[i].getTeeynsyId();
    bool new_strip = true;
    for (unsigned int j = 0; j < block_light_nodes_[i].getNumComponents(); ++j)
    {
      std::pair<unsigned int, unsigned int> teensy_id_and_strip_id_pair;
      teensy_id_and_strip_id_pair.first = TEENSY_ID;
      teensy_id_and_strip_id_pair.second = i;
      if(new_strip)
      {
        new_strip = false;
        setup_data_[TEENSY_ID].block_leds[setup_data_[TEENSY_ID].num_block_leds].index = (uint8_t)0;
      }
      else
      {
        setup_data_[TEENSY_ID].block_leds[setup_data_[TEENSY_ID].num_block_leds].index =
            setup_data_[TEENSY_ID].block_leds[setup_data_[TEENSY_ID].num_block_leds - 1].index +
            setup_data_[TEENSY_ID].block_leds[setup_data_[TEENSY_ID].num_block_leds - 1].num_leds;
      }
      setup_data_[TEENSY_ID].block_leds[setup_data_[TEENSY_ID].num_block_leds].num_leds = (uint8_t)block_light_nodes_[i].getNumLeds(j);
      setup_data_[TEENSY_ID].block_leds[setup_data_[TEENSY_ID].num_block_leds].pin = (uint8_t)teensy_id_and_strip_id_pair.second;
      light_node_leds_to_teensy_map_.push_back(teensy_id_and_strip_id_pair);
      ROS_INFO("Block light node at pin >%i< connected to teensy >%i<. LED index >%i< num_leds >%i<.",
               (int)setup_data_[TEENSY_ID].block_leds[setup_data_[TEENSY_ID].num_block_leds].pin,
               (int)light_node_leds_to_teensy_map_[i].first,
               (int)setup_data_[TEENSY_ID].block_leds[setup_data_[TEENSY_ID].num_block_leds].index,
               (int)setup_data_[TEENSY_ID].block_leds[setup_data_[TEENSY_ID].num_block_leds].num_leds);
      setup_data_[TEENSY_ID].num_block_leds++;
    }
  }

  ROS_INFO("Setting up block light beam map.");
  block_light_beam_leds_to_teensy_map_.clear();
  for (unsigned int i = 0; i < block_light_beams_.size(); ++i)
  {
    const unsigned int TEENSY_ID = block_light_beams_[i].getTeeynsyId();
    bool new_strip = true;
    for (unsigned int j = 0; j < block_light_beams_[i].getNumComponents(); ++j)
    {
      std::pair<unsigned int, unsigned int> teensy_id_and_strip_id_pair;
      teensy_id_and_strip_id_pair.first = TEENSY_ID;
      teensy_id_and_strip_id_pair.second = i + block_light_nodes_.size();
      if(new_strip)
      {
        new_strip = false;
        setup_data_[TEENSY_ID].block_leds[setup_data_[TEENSY_ID].num_block_leds].index = (uint8_t)0;
      }
      else
      {
        setup_data_[TEENSY_ID].block_leds[setup_data_[TEENSY_ID].num_block_leds].index =
            setup_data_[TEENSY_ID].block_leds[setup_data_[TEENSY_ID].num_block_leds - 1].index +
            setup_data_[TEENSY_ID].block_leds[setup_data_[TEENSY_ID].num_block_leds - 1].num_leds;
      }
      setup_data_[TEENSY_ID].block_leds[setup_data_[TEENSY_ID].num_block_leds].num_leds = (uint8_t)block_light_beams_[i].getNumLeds(j);
      setup_data_[TEENSY_ID].block_leds[setup_data_[TEENSY_ID].num_block_leds].pin = (uint8_t)(teensy_id_and_strip_id_pair.second);

      block_light_beam_leds_to_teensy_map_.push_back(teensy_id_and_strip_id_pair);
      ROS_INFO("Block light beam at pin >%i< connected to teensy >%i<. LED index >%i< num_leds >%i<.",
               (int)setup_data_[TEENSY_ID].block_leds[setup_data_[TEENSY_ID].num_block_leds].pin,
               (int)block_light_beam_leds_to_teensy_map_[i].first,
               (int)setup_data_[TEENSY_ID].block_leds[setup_data_[TEENSY_ID].num_block_leds].index,
               (int)setup_data_[TEENSY_ID].block_leds[setup_data_[TEENSY_ID].num_block_leds].num_leds);

      setup_data_[TEENSY_ID].num_block_leds++;
    }
  }

  ROS_INFO("Setting up pixel light beam map.");
  unsigned int led_index = 0;
  pixel_light_beam_leds_to_teensy_map_.clear();
  for (unsigned int i = 0; i < pixel_light_beams_.size(); ++i)
  {
    const unsigned int TEENSY_ID = pixel_light_beams_[i].getTeeynsyId();
    bool new_strip = true;
    for (unsigned int j = 0; j < pixel_light_beams_[i].getNumComponents(); ++j)
    {
      for (unsigned int k = 0; k < pixel_light_beams_[i].getNumLeds(j); ++k)
      {
        std::pair<unsigned int, std::pair<unsigned int, unsigned int> > teensy_id_to_led_info_pair;
        teensy_id_to_led_info_pair.first = TEENSY_ID;
        std::pair<unsigned int, unsigned int> strip_id_and_led_id_pair;
        strip_id_and_led_id_pair.first = i + block_light_nodes_.size() + block_light_beams_.size();
        strip_id_and_led_id_pair.second = k;
        if(new_strip)
        {
          new_strip = false;
          setup_data_[TEENSY_ID].pixel_leds[setup_data_[TEENSY_ID].num_pixel_leds].index = (uint8_t)0;
        }
        else
        {
          setup_data_[TEENSY_ID].pixel_leds[setup_data_[TEENSY_ID].num_pixel_leds].index =
              setup_data_[TEENSY_ID].pixel_leds[setup_data_[TEENSY_ID].num_pixel_leds - 1].index +
              setup_data_[TEENSY_ID].pixel_leds[setup_data_[TEENSY_ID].num_pixel_leds - 1].num_leds;
        }
        setup_data_[TEENSY_ID].pixel_leds[setup_data_[TEENSY_ID].num_pixel_leds].num_leds = (uint8_t)1;
        setup_data_[TEENSY_ID].pixel_leds[setup_data_[TEENSY_ID].num_pixel_leds].pin = (uint8_t)strip_id_and_led_id_pair.first;
        teensy_id_to_led_info_pair.second = strip_id_and_led_id_pair;
        pixel_light_beam_leds_to_teensy_map_.push_back(teensy_id_to_led_info_pair);
        ROS_INFO("Pixel light beam at pin >%i< connected to teensy >%i< with index >%i< num_leds >%i<.",
                 (int)setup_data_[TEENSY_ID].pixel_leds[setup_data_[TEENSY_ID].num_pixel_leds].pin,
                 (int)pixel_light_beam_leds_to_teensy_map_[led_index].first,
                 (int)setup_data_[TEENSY_ID].pixel_leds[setup_data_[TEENSY_ID].num_pixel_leds].index,
                 (int)setup_data_[TEENSY_ID].pixel_leds[setup_data_[TEENSY_ID].num_pixel_leds].num_leds);
        led_index++;
        setup_data_[TEENSY_ID].num_pixel_leds++;
      }
    }
  }
  ROS_ASSERT_MSG(led_index == total_num_pixel_beam_leds_, "LED index is >%i< and total number of pixel beam leds is >%i<.",
                 (int)led_index, (int)total_num_pixel_beam_leds_);
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
  for (unsigned int i = 0; i < capactive_sensors_.size(); ++i)
  {
    used_ids.insert(std::pair<unsigned int, int>(capactive_sensors_[i].getTeeynsyId(), 0));
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
  ROS_ASSERT_MSG(max_number_of_sensors_per_teensy_ <= 255, "Maximum number of sensors specified >%i< is invalid.", max_number_of_sensors_per_teensy_);

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
  ROS_ASSERT_MSG(max_number_of_light_strips_per_teensy_ <= 255, "Maximum number of light strips specified >%i< is invalid.", max_number_of_light_strips_per_teensy_);

  total_num_node_leds_ = 0;
  std::vector<unsigned int> light_pins(number_of_teensys_, 0);
  for (unsigned int i = 0; i < block_light_nodes_.size(); ++i)
  {
    ROS_ASSERT(block_light_nodes_[i].getTotalNumLeds() <= MAX_NUMBER_OF_LEDS_PER_LIGHT_STRIP);
    block_light_nodes_[i].setPin(light_pins[block_light_nodes_[i].getTeeynsyId()]);
    light_pins[block_light_nodes_[i].getTeeynsyId()]++;
    // only 1 LED for each block node is modeled
    total_num_node_leds_ += block_light_nodes_[i].getNumComponents();
  }
  //  ROS_ASSERT_MSG(total_num_node_leds_ == block_light_nodes_.size(),
  //                 "Total number of light node LEDs >%i< does not match number of block light nodes >%i<.",
  //                 (int)total_num_node_leds_, (int)block_light_nodes_.size());

  total_num_block_beam_leds_ = 0;
  for (unsigned int i = 0; i < block_light_beams_.size(); ++i)
  {
    ROS_ASSERT(block_light_beams_[i].getTotalNumLeds() <= MAX_NUMBER_OF_LEDS_PER_LIGHT_STRIP);
    block_light_beams_[i].setPin(light_pins[block_light_beams_[i].getTeeynsyId()]);
    light_pins[block_light_beams_[i].getTeeynsyId()]++;
    // only 1 LED for each block beam is modeled
    total_num_block_beam_leds_ += block_light_beams_[i].getNumComponents();
  }

  total_num_pixel_beam_leds_ = 0;
  for (unsigned int i = 0; i < pixel_light_beams_.size(); ++i)
  {
    ROS_ASSERT(pixel_light_beams_[i].getTotalNumLeds() <= MAX_NUMBER_OF_LEDS_PER_LIGHT_STRIP);
    pixel_light_beams_[i].setPin(light_pins[pixel_light_beams_[i].getTeeynsyId()]);
    light_pins[pixel_light_beams_[i].getTeeynsyId()]++;
    // each LED is modeled
    total_num_pixel_beam_leds_ += pixel_light_beams_[i].getTotalNumLeds();
  }
  for(unsigned int i = 0; i < light_pins.size(); ++i)
  {
    ROS_ASSERT_MSG(light_pins[i] < max_number_of_light_strips_per_teensy_,
                   "Number of lights connected to teensy >%i< exceeds limit >%i<.",
                   (int)i, (int)max_number_of_light_strips_per_teensy_);
  }

  total_num_sensors_ = capactive_sensors_.size();
  std::vector<unsigned int> sensor_pins(number_of_teensys_, 0);
  for (unsigned int i = 0; i < capactive_sensors_.size(); ++i)
  {
    capactive_sensors_[i].setPin(sensor_pins[capactive_sensors_[i].getTeeynsyId()]);
    sensor_pins[capactive_sensors_[i].getTeeynsyId()]++;
  }
  for(unsigned int i = 0; i < sensor_pins.size(); ++i)
  {
    ROS_ASSERT_MSG(sensor_pins[i] < max_number_of_sensors_per_teensy_,
                   "Number of sensors connected to teensy >%i< exceeds limit >%i<.",
                   (int)i, (int)max_number_of_sensors_per_teensy_);
  }

  ROS_INFO("Setup complete:");
  ROS_INFO("Structure contains >%i< node lights, >%i< beam lights, >%i< pixel lights, and >%i< sensors.",
           total_num_node_leds_, total_num_block_beam_leds_, total_num_pixel_beam_leds_, total_num_sensors_);
}

/*
bool DecStructure::read(ros::NodeHandle& node_handle,
                   std::vector<geometry_msgs::Point>& positions)
{
  const std::string KEY = "node_positions";
  XmlRpc::XmlRpcValue rpc_values;
  if (!node_handle.getParam(KEY, rpc_values))
  {
    ROS_ERROR("Could not read from >%s/%s<.", node_handle.getNamespace().c_str(), KEY.c_str());
    return false;
  }

  positions.clear();
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
    positions.push_back(point);
  }

  return true;
}
*/

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
  return true;
}

bool DecStructure::read(ros::NodeHandle& node_handle, std::vector<BlockLightBeam>& block_light_beams)
{
  XmlRpc::XmlRpcValue rpc_values = get(node_handle, "block_light_beams");
  int id = 0;
  for (int i = 0; i < rpc_values.size(); ++i)
  {
    BlockLightBeam block_light_beam;
    ROS_VERIFY(block_light_beam.initialize(rpc_values[i], id, node_positions_));
    id += block_light_beam.getNumComponents();
    block_light_beams.push_back(block_light_beam);
  }
  return true;
}

bool DecStructure::read(ros::NodeHandle& node_handle, std::vector<PixelLightBeam>& pixel_light_beams)
{
  XmlRpc::XmlRpcValue rpc_values = get(node_handle, "pixel_light_beams");
  int id = 0;
  for (int i = 0; i < rpc_values.size(); ++i)
  {
    PixelLightBeam pixel_light_beam;
    ROS_VERIFY(pixel_light_beam.initialize(rpc_values[i], id, node_positions_));
    id += pixel_light_beam.getNumComponents();
    pixel_light_beams.push_back(pixel_light_beam);
  }
  return true;
}

}
