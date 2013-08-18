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
#define BRIGHTNESS_OFFSET 3

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

  static const float BASE_LIGHT_LEVEL = 0.5f;
  static const float MAX_LIGHT_LEVEL = 1.0f;
  static const float MIN_LIGHT_LEVEL = 0.0f;

  static const uint8_t SENSOR_HIGH = 1;
  static const uint8_t SENSOR_LOW = 0;

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

  /*!
   */
  ros::Time ros_time_;
  double ros_time_sec_;

  /*!
   */
  bool recording_;

  /*!
   */
  ros::NodeHandle node_handle_;

  /*!
   * @return
   */
  unsigned int getMaxBrightness() const
  {
    return max_brightness_;
  }

  bool visualization_mode_;

private:
  bool initialized_;
  std::vector<bool> send_flags_;

  /*!
   */
  unsigned int max_brightness_;

  /*!
   * @param teensy_id
   * @return distance of
   */
  std::vector<std::vector<int> > getTeensyToSensorsDistances(const unsigned int& teensy_id) const;

  /*!
   */
  bool generateConfigurationFile(const std::string& abs_file_name);

  /*!
   */
  float computeDistance(const geometry_msgs::Point& sensor, const geometry_msgs::Point& thing);

  /*!
   */
  std::vector<unsigned int> list_of_teensys_to_exclude_from_communication_;

};

}

#endif /* DEC_DATA_H_ */
