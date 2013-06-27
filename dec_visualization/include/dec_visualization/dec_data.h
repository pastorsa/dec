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
   * @return number of arduinos
   */
  int getNumArduinos() const;

  /*!
   * @param arduino_index
   * @param sensor_index
   * @return sensor_value
   */
  int getSensorValue(const int arduino_index,
                      const int sensor_index);

  /*!
   * @param arduino_index
   * @param sensor_index
   * @param sensor_value
   */
  void addSensorValue(const int arduino_index,
                        const int sensor_index,
                        const int sensor_value);

  /*!
   * @param arduino_index
   * @param sensor_index
   * @param sensor_value
   */
  void setSensorValue(const int arduino_index,
                         const int sensor_index,
                         const int sensor_value);

protected:

  int interaction_mode_;
  enum {
    eSETUP = 0,
    eLOCAL = 1,
    eNUM_MODES = 2,
  };

  static const int NUM_ENTRIES_FOR_ARDUINO_LEVEL = 1;
  static const int RED_OFFSET = 0;
  static const int GREEN_OFFSET = 1;
  static const int BLUE_OFFSET = 2;
  static const int ALPHA_OFFSET = 3;

  static const int SENSOR_RESOLUTION = 1000;

  /*! Matrix of size number_of_arduinos x num_entries_per_arduino
   * Each entry contains
   * level | 4 * max_lights | max_sensors
   */
  Eigen::MatrixXi data_;

  int number_of_arduinos_;
  int max_number_of_light_strips_per_arduino_;
  int max_number_of_leds_per_light_strip_;
  int max_number_of_sensors_per_arduino_;

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

  /*! Which light_beam/light_node/sensor is connected to which arduino
   * The size of each vector is equal to the size of
   * light_beams/light_nodes/sensors. The stored number is the
   * id of the arduino to which this thing is connected.
   */
  std::vector<int> light_beam_connections_;
  std::vector<int> light_node_connections_;
  std::vector<int> sensor_connections_;

  /*! These vectors are of size number_of_arduinos
   * Each entry (index by the arduino id) contains the index
   * into arduino_to_*_map_ (see below) to obtain the "local" index of this
   * thing at this arduino.
   */
  std::vector<int> light_beam_index_counter_;
  std::vector<int> light_node_index_counter_;
  std::vector<int> sensor_index_counter_;

  /*! Number of light beams/light nodes/sensor for each arduino
   * The size of these vectors is equal to the number of arduinos in the network
   * each entry contains a list of indices into light beams/light nodes/sensors
   * to which this arduino is connected. Obviously, it can be empty, meaning that
   * this arduino does not have a light beams/light nodes/sensors attached.
   */
  std::vector<std::vector<int> > arduino_to_light_beam_map_;
  std::vector<std::vector<int> > arduino_to_light_node_map_;
  std::vector<std::vector<int> > arduino_to_sensor_map_;

  /*! This vector contains the pin settings of the DE pin of the RS-485 interace
   * of each arduino. Thus, the size of this vector must be the same as number_of_arduino
   */
  // std::vector<int> node_icsc_de_pins_;
  // int controller_icsc_de_pin_;

  /*! The pin ordering describes the order of pins at which "first" the light beams led strips
   * are connected then "second" the light node led strips are connected and "third" the
   * sensors are connected.
   * For example if a arduino has 3 light beams, 2 light nodes, and 5 sensors attached then
   * the light node pins are the first three numbers in io_pin_ordering, the light beam pins are the
   * next 2 numbers, and the sensor pins are the next 5 numbers.
   * Obviously, the numbers are limited to the amount of io pins on the arduino
   */
  std::vector<int> io_pin_ordering_;

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
   */
  bool generateConfigurationFile(const std::string& abs_file_name);
  bool generateStructureFile(const std::string& abs_file_name);
};

}


#endif /* DEC_DATA_H_ */
