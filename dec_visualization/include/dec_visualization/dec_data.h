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
  DECData() : interaction_mode_(0), initialized_(false) {};
  virtual ~DECData() {};

  /*!
   * @param node_handle
   * @return True on success, otherwise False
   */
  bool init(ros::NodeHandle node_handle);

  /*!
   * @return number of arduinos
   */
  int getNumArduinos();

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
  int max_number_of_lights_per_arduino_;
  int max_number_of_sensors_per_arduino_;

  std::vector<geometry_msgs::Point> node_positions_;

  /*!
   */
  std::vector<int> light_nodes_;
  std::vector<geometry_msgs::Point> light_node_positions_;;

  /*! pairs of nodes in node_positions_ (indexed from 0)
   */
  std::vector<std::pair<int, int> > beams_;
  std::vector<std::pair<int, int> > sensors_;
  std::vector<std::pair<int, int> > light_beams_;

  /*! which light_beam or sensor is connected to which arduino
   */
  std::vector<int> light_beam_connections_;
  std::vector<int> light_node_connections_;
  std::vector<int> sensor_connections_;

  std::vector<int> light_beam_index_counter_;
  std::vector<int> light_node_index_counter_;
  std::vector<int> sensor_index_counter_;

  std::vector<std::vector<int> > arduino_to_light_beam_map_;
  std::vector<std::vector<int> > arduino_to_light_node_map_;
  std::vector<std::vector<int> > arduino_to_sensor_map_;

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

};

}


#endif /* DEC_DATA_H_ */
