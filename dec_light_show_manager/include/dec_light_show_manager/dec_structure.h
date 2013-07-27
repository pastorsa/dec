/*
 * dec_structure.h
 *
 *  Created on: Jul 18, 2013
 *      Author: pastor
 */

#ifndef DEC_STRUCTURE_H_
#define DEC_STRUCTURE_H_

#include <string>

#include <dec_udp/dec_interface.h>

#include <dec_light_show_manager/dec_components.h>

namespace dec_light_show_manager
{

const unsigned int MAX_NUMBER_OF_BLOCKS_PER_TEENSY = 9 * 10;
const unsigned int MAX_NUMBER_OF_PIXELS_PER_TEENSY = 5;

const unsigned int MAX_NUMBER_OF_PIXELS_PER_LIGHT_STRIP = 150;

class DecStructure
{

#define DEC_EXTRA 1
#ifdef DEC_EXTRA
  friend class DecLightShowVisualization;
  friend class DecLightShowSimulation;
#endif

public:
  DecStructure() :
    total_num_node_leds_(0),
    total_num_block_beam_leds_(0),
    total_num_pixel_beam_leds_(0),
    total_num_sensors_(0),
    number_of_teensys_(0),
    max_number_of_light_strips_per_teensy_(0),
    max_number_of_sensors_per_teensy_(0) {};
  virtual ~DecStructure() {};

  /*!
   * @param node_handle
   * @return True on success, otherwise False
   */
  bool initialize(ros::NodeHandle& node_handle);

  /*!
   * @return
   */
  inline unsigned int getNumNodes() const
  {
    return nodes_.size();
  }

  /*!
   * @param id
   * @return Position of node
   */
  geometry_msgs::Point getNodePosition(const unsigned int id) const
  {
    ROS_ASSERT(id < (int)getNumNodes());
    ROS_ASSERT(nodes_[id].poses_.size() == 1);
    return nodes_[id].getPose(0).position;
  }

  /*!
   */
  unsigned int total_num_node_leds_;
  unsigned int total_num_block_beam_leds_;
  unsigned int total_num_pixel_beam_leds_;
  unsigned int total_num_sensors_;

protected:

  /*!
   */
  std::vector<Node> nodes_;
  std::vector<Beam> beams_;

  /*!
   */
  std::vector<Sensor> sensors_;
  std::vector<LightNode> block_light_nodes_;
  std::vector<BlockLightBeam> block_light_beams_;
  std::vector<PixelLightBeam> pixel_light_beams_;

  /*!
   */
  geometry_msgs::Vector3 nodes_size_;
  std::vector<double> nodes_color_;
  geometry_msgs::Vector3 beams_size_;
  std::vector<double> beams_color_;

  /*!
   */
  geometry_msgs::Vector3 block_light_nodes_size_;
  geometry_msgs::Vector3 block_light_beams_size_;
  geometry_msgs::Vector3 pixel_light_beams_size_;
  geometry_msgs::Vector3 sensors_size_;

  /*! Number of teensys in the structure and related info
   */
  unsigned int number_of_teensys_;
  unsigned int max_number_of_light_strips_per_teensy_;
  unsigned int max_number_of_sensors_per_teensy_;

  /*! This light pins vector contains the order of PWM capable output pins at which the
   * lights are hooked up to the teensy
   */
  std::vector<unsigned int> light_pins_;
  /*! This sensor pins vector contains the order of input pins at which the
   * sensors are hooked up to the teensy
   */
  std::vector<unsigned int> sensor_pins_;

  /*! This vector is of size total_num_sensors_
   * Each entry (index by the sensor id) contains a pair where
   * the first entry is the teensy_id and the second is the index into sensor
   * values at that teensy
   */
  std::vector<std::pair<unsigned int, unsigned int> > sensor_to_teensy_map_;
  /*! This vector is of size total_num_node_leds_
   * Each entry (index by the node led id) contains a pair where
   * the first entry is the teensy_id and the second is the pair block and strip id
   */
  std::vector<std::pair<unsigned int, unsigned int> > light_node_leds_to_teensy_map_;
  /*! This vector is of size total_num_beam_leds_
   * Each entry (index by the beam led id) contains a pair where
   * the first element is the teensy_id and the second is the pair block and strip id
   */
  std::vector<std::pair<unsigned int, unsigned int> > block_light_beam_leds_to_teensy_map_;
  /*! This vector is of size total_num_beam_leds_
   * Each entry (index by the beam led id) contains a pair where
   * the first element is the teensy_id and the second is the pair
   * strip_id and led_id
   */
  std::vector<std::pair<unsigned int, std::pair<unsigned int, unsigned int> > > pixel_light_beam_leds_to_teensy_map_;

  /*! To send/receive data over udp
   */
  boost::shared_ptr<dec_udp::DecInterface> dec_interface_;
  std::vector<light_data_t> dec_interface_light_data_;
  std::vector<setup_data_t> dec_interface_setup_data_;

private:

  /*!
   */
  std::vector<geometry_msgs::Point> node_positions_;
  std::vector<geometry_msgs::Pose> beam_poses_;
  std::vector<geometry_msgs::Pose> sensor_poses_;
  std::vector<geometry_msgs::Point> block_light_node_positions_;
  std::vector<geometry_msgs::Pose> block_light_beam_poses_;
  std::vector<geometry_msgs::Pose> pixel_light_beam_poses_;
  std::vector<geometry_msgs::Pose> pixel_light_beam_led_poses_;

  std::vector<unsigned int> num_block_node_leds_per_teensy_;
  std::vector<unsigned int> num_block_beam_leds_per_teensy_;

  void setNumberOfTeensys();
  void setPins(ros::NodeHandle node_handle);
  void setupTeensyMap();
  bool isUnique();

  bool read(ros::NodeHandle& node_handle, std::vector<Node>& nodes);
  bool read(ros::NodeHandle& node_handle, std::vector<Beam>& beams);
  bool read(ros::NodeHandle& node_handle, std::vector<Sensor>& sensors);
  bool read(ros::NodeHandle& node_handle, std::vector<LightNode>& light_nodes);
  bool read(ros::NodeHandle& node_handle, std::vector<BlockLightBeam>& block_light_beams);
  bool read(ros::NodeHandle& node_handle, std::vector<PixelLightBeam>& pixel_light_beams);

  XmlRpc::XmlRpcValue get(ros::NodeHandle& node_handle, const std::string& key);

  /*! Offset all nodes such that the node node_index is at (0.0, 0.0, 0.0)
   * @param node_positions
   * @param node_index
   */
  void offsetNodePositions(std::vector<geometry_msgs::Point>& node_positions, const int node_index);

};

}


#endif /* DEC_STRUCTURE_H_ */
