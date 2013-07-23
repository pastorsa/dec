/*
 * dec_data.h
 *
 *  Created on: Jul 14, 2013
 *      Author: pastor
 */

#ifndef DEC_DATA_H_
#define DEC_DATA_H_

#include <vector>
#include <ros/ros.h>

#include <dec_utilities/assert.h>
#include <dec_utilities/param_server.h>

#include <Eigen/Eigen>
#include <geometry_msgs/Point.h>

#include <dec_light_show_manager/dec_structure.h>

#define RED_OFFSET   0
#define GREEN_OFFSET 1
#define BLUE_OFFSET  2
#define ALPHA_OFFSET 3
// static const double COLOR_RESOLUTION = 255.0;
// static const int SENSOR_RESOLUTION = 255;

namespace dec_light_show_manager
{

//typedef uint8_t led_channel_t;
//typedef uint8_t sensor_channel_t;
typedef int led_channel_t;
typedef int sensor_channel_t;

/**
 * This class contains public data to be shared across light shows
 */
class DecData : public DecStructure
{

public:

#ifdef DEC_EXTRA
  friend class DecLightShowVisualization;
  friend class DecLightShowSimulation;
#endif

  DecData();
  virtual ~DecData() {};

  /*!
   * @param node_handle
   * @return True on success, otherwise False
   */
  bool initialize(ros::NodeHandle node_handle);

  /*!
   * @return number of teensys
   */
  int getNumTeensys() const;

  /*! Sends setup data to each teensy and keeps doing that
   * until all teensys have responded.
   * @return True on success, otherwise False
   */
  bool sendSetupData();

  /*!
   * @return
   */
  bool copySensorInformationToStructure();
  bool copySensorInformationFromStructure();
  bool copyLightDataToStructure();
  bool copyLightDataFromStructure();

  double control_frequency_;
  double control_dt_;

  /*! Sensor values as received by the teensys or simulation
   */
  typedef Eigen::Matrix<sensor_channel_t, 1, Eigen::Dynamic> VectorX_sensor_channel_t;
  VectorX_sensor_channel_t sensor_values_;
  VectorX_sensor_channel_t prev_sensor_values_;

  /*! Light values as send to the teensys or simulation
   * The size of each matrix corresponds 4 (r,g,b,a) times the total number of node/beam leds
   */
  typedef Eigen::Matrix<led_channel_t, 4, Eigen::Dynamic> MatrixX_led_channel_t;
  MatrixX_led_channel_t node_led_values_;
  MatrixX_led_channel_t block_beam_led_values_;
  MatrixX_led_channel_t pixel_beam_led_values_;

  /*! The vector is of size total_num_node_leds_ and total_num_beam_leds_
   * The value range is [0..1] and will be converted to a color
   */
  Eigen::VectorXf node_led_levels_;
  Eigen::VectorXf block_beam_led_levels_;
  Eigen::VectorXf pixel_beam_led_levels_;
  /*! The vector is of size total_num_sensors_;
   * The value is set by the sensor processor light show
   */
  Eigen::VectorXf sensor_levels_;

  /*! These matrices are of size {num_node_leds,num_block_beam_leds,num_pixel_beam_leds} x num_sensor
   * Each entry corresponds to the distance of that sensor-led pair in meters
   */
  Eigen::MatrixXf node_led_distances_to_sensor_;
  Eigen::MatrixXf pixel_beam_led_distances_to_sensor_;
  Eigen::MatrixXf block_beam_led_distances_to_sensor_;


//  //std::vector<std::vector<Eigen::VectorXi> > light_beam_data_;
//  // #include<Eigen/StdVector>
//  // \/* ... *\/
//  // std::vector<Eigen::Vector4f,Eigen::aligned_allocator<Eigen::Vector4f> >
//
//  /*! Number of LEDs for each light beam and light node.
//   * The size corresponds to the number of light beams and light nodes in the structure.
//   */
//   std::vector<int> num_leds_of_each_light_beam_;
//   std::vector<int> num_leds_of_each_light_node_;
//
//  /*! Which light_beam/light_node/sensor is connected to which teensy
//   * The size of each vector is equal to the size of
//   * light_beams/light_nodes/sensors. The stored number is the
//   * id of the teensy to which this thing is connected.
//   */
//   std::vector<int> light_beam_connections_;
//   std::vector<int> light_node_connections_;
//   std::vector<int> sensor_connections_;
//
//  /*! These vectors are of size number_of_teensys
//   * Each entry (index by the teensy id) contains the index
//   * into teensy_to_*_map_ (see below) to obtain the "local" index of this
//   * thing at this teensy.
//   */
//   std::vector<int> light_beam_index_counter_;
//   std::vector<int> light_node_index_counter_;
//   std::vector<int> sensor_index_counter_;
//
//  /*! Number of light beams/light nodes/sensor for each teensy
//   * The size of these vectors is equal to the number of teensys in the network
//   * each entry contains a list of indices into light beams/light nodes/sensors
//   * to which this teensy is connected. Obviously, it can be empty, meaning that
//   * this teensy does not have a light beams/light nodes/sensors attached.
//   */
//  std::vector<std::vector<int> > teensy_to_light_beam_map_;
//  std::vector<std::vector<int> > teensy_to_light_node_map_;
//  std::vector<std::vector<int> > teensy_to_sensor_map_;


  /*!
   */
  ros::Time ros_time_;
  double ros_time_sec_;

private:
  bool initialized_;

//  bool read(ros::NodeHandle node_handle,
//            const std::string& array_name,
//            std::vector<std::pair<int, int> >& nodes);
//
//  bool read(ros::NodeHandle node_handle,
//            const::std::string& array_name,
//            std::vector<int>& values,
//            std::vector<int>& index_values,
//            std::vector<std::vector<int> >& map,
//            const unsigned int& num);
//
  /*!
   * @param teensy_id
   * @return distance of
   */
  std::vector<std::vector<int> > getTeensyToSensorsDistances(const unsigned int& teensy_id) const;

  /*!
   */
  bool generateConfigurationFile(const std::string& abs_file_name);
  bool generateStructureFile(const std::string& abs_file_name,
                             const std::string progmem_prefix = "",
                             const std::string unit_prefix = "");


  /*! To send/receive data over udp
   */
  boost::shared_ptr<dec_udp::DecInterface> dec_interface_;
  std::vector<sensor_data_t> dec_interface_sensor_data_;
  std::vector<light_data_t> dec_interface_light_data_;
  std::vector<setup_data_t> dec_interface_setup_data_;

  /*!
   */
  float computeDistance(const geometry_msgs::Point& sensor, const geometry_msgs::Point& thing);

  std::vector<unsigned int> list_of_teensys_to_exclude_from_communication_;

};

}

#endif /* DEC_DATA_H_ */
