/*
 * dec_data.h
 *
 *  Created on: Jun 17, 2013
 *      Author: pastor
 */

#ifndef DEC_DATA_H_
#define DEC_DATA_H_

#include <dec_utilities/assert.h>
#include <dec_utilities/param_server.h>

#include <Eigen/Eigen>

#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Point.h>

#include <dec_udp/dec_interface.h>

namespace dec_visualization
{

class DECData
{

public:
  DECData();
  virtual ~DECData() {};

  /*!
   * @param node_handle
   * @return True on success, otherwise False
   */
  bool init(ros::NodeHandle node_handle);

  /*!
   * @return number of teensys
   */
  int getNumTeensys() const;

  /*!
   * @param teensy_index
   * @param sensor_index
   * @return sensor_value
   */
  int getSensorValue(const int teensy_index,
                      const int sensor_index);

  /*!
   * @param teensy_index
   * @param sensor_index
   * @param sensor_value
   */
  void addSensorValue(const int teensy_index,
                        const int sensor_index,
                        const int sensor_value);

  /*!
   * @param teensy_index
   * @param sensor_index
   * @param sensor_value
   */
  void setSensorValue(const int teensy_index,
                         const int sensor_index,
                         const int sensor_value);

protected:

  int interaction_mode_;
  enum {
    eSETUP = 0,
    eLOCAL = 1,
    eNUM_MODES = 2,
  };

  static const int NUM_ENTRIES_FOR_TEENSY_LEVEL = 1;
  static const int RED_OFFSET = 0;
  static const int GREEN_OFFSET = 1;
  static const int BLUE_OFFSET = 2;
  static const int ALPHA_OFFSET = 3;

  static const int SENSOR_RESOLUTION = 255;

  /*! Matrix of size number_of_teensys x num_entries_per_teensy
   * Each entry contains
   * level | 4 * max_lights | max_sensors
   */
  Eigen::MatrixXi data_;

  int number_of_teensys_;
  int max_number_of_light_strips_per_teensy_;
  int max_number_of_leds_per_light_strip_;
  int max_number_of_sensors_per_teensy_;

  std::vector<geometry_msgs::Point> node_positions_;

  /*!
   */
  std::vector<int> light_nodes_;
  std::vector<geometry_msgs::Point> light_node_positions_;;

  /*! Pairs of nodes in node_positions_ (indexed from 0)
   */
  std::vector<std::pair<int, int> > beams_;
  std::vector<std::pair<int, int> > sensors_;
  std::vector<std::pair<int, int> > light_beams_;

  /*! Number of LEDs for each light beam and light node.
   * The size corresponds to the number of light beams and light nodes in the structure.
   */
  std::vector<int> num_leds_of_each_light_beam_;
  std::vector<int> num_leds_of_each_light_node_;

  /*! Which light_beam/light_node/sensor is connected to which teensy
   * The size of each vector is equal to the size of
   * light_beams/light_nodes/sensors. The stored number is the
   * id of the teensy to which this thing is connected.
   */
  std::vector<int> light_beam_connections_;
  std::vector<int> light_node_connections_;
  std::vector<int> sensor_connections_;

  /*! These vectors are of size number_of_teensys
   * Each entry (index by the teensy id) contains the index
   * into teensy_to_*_map_ (see below) to obtain the "local" index of this
   * thing at this teensy.
   */
  std::vector<int> light_beam_index_counter_;
  std::vector<int> light_node_index_counter_;
  std::vector<int> sensor_index_counter_;

  /*! Number of light beams/light nodes/sensor for each teensy
   * The size of these vectors is equal to the number of teensys in the network
   * each entry contains a list of indices into light beams/light nodes/sensors
   * to which this teensy is connected. Obviously, it can be empty, meaning that
   * this teensy does not have a light beams/light nodes/sensors attached.
   */
  std::vector<std::vector<int> > teensy_to_light_beam_map_;
  std::vector<std::vector<int> > teensy_to_light_node_map_;
  std::vector<std::vector<int> > teensy_to_sensor_map_;

  /*! This light pins vector contains the order of PWM capable output pins at which the
   * lights are hooked up to the teensy
   */
  std::vector<int> light_pins_;
  /*! This sensor pins vector contains the order of input pins at which the
   * sensors are hooked up to the teensy
   */
  std::vector<int> sensor_pins_;

  /*!
   */
  bool enable_communication_;
  /*! Data to be send to the teensys over udp
   */
  std::vector<light_data_t> light_data_;


private:
  bool initialized_;

  bool read(ros::NodeHandle node_handle,
            const std::string& array_name,
            std::vector<geometry_msgs::Point>& array);
  bool read(ros::NodeHandle node_handle,
            const std::string& array_name,
            std::vector<std::pair<int, int> >& nodes);

  bool read(ros::NodeHandle node_handle,
            const::std::string& array_name,
            std::vector<int>& values,
            std::vector<int>& index_values,
            std::vector<std::vector<int> >& map,
            const unsigned int& num);

  /*! Offset all nodes such that the node node_index is at (0.0, 0.0, 0.0)
   * @param node_positions
   * @param node_index
   */
  void offsetNodePositions(std::vector<geometry_msgs::Point>& node_positions,
                           const int node_index);

  /*!
   * @param teensy_id
   * @return distance of
   */
  std::vector<std::vector<int> > getTeensyToSensorsDistances(const int& teensy_id) const;

  /*!
   */
  bool generateConfigurationFile(const std::string& abs_file_name);
  bool generateStructureFile(const std::string& abs_file_name,
                             const std::string progmem_prefix = "",
                             const std::string unit_prefix = "");
};

}


#endif /* DEC_DATA_H_ */
