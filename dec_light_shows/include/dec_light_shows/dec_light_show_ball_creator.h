/*
 * dec_light_show_ball_creator.h
 *
 *  Created on: Jul 14, 2013
 *      Author: pastor
 */

#ifndef DEC_LIGHT_SHOW_BALL_CREATOR_H_
#define DEC_LIGHT_SHOW_BALL_CREATOR_H_

#include <vector>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <dec_light_show_manager/dec_light_show.h>

namespace dec_light_shows
{

class DecLightShowBallCreator : public dec_light_show_manager::DecLightShow
{
public:
  DecLightShowBallCreator();
  virtual ~DecLightShowBallCreator() {};

  virtual bool initialize(XmlRpc::XmlRpcValue& config);
  virtual bool update();
  virtual bool start();
  virtual bool stop();

private:

  ros::NodeHandle node_handle_;
  ros::Publisher rviz_pub_;

  visualization_msgs::MarkerArray virtual_sensors_;
  void setupSensorMarkers(XmlRpc::XmlRpcValue& config);
  void readParameters(XmlRpc::XmlRpcValue& config);
  void setupSpace(XmlRpc::XmlRpcValue& config);
  void publish();

  unsigned int visualization_rate_;
  unsigned int visualization_counter_;

  void integrate();
  std::vector<tf::Vector3> positions_;
  std::vector<tf::Vector3> velocities_;
  std::vector<tf::Vector3> accelerations_;
  std::vector<tf::Vector3> simulated_accelerations_;

  void computeDistance();
  /*! This buffer is of size >total_num_node_leds_<
   */
  Eigen::VectorXf block_node_buffer_;
  /*! This buffer is of size >total_num_block_beam_leds_<
   */
  Eigen::VectorXf block_beam_buffer_;
  /*! This buffer is of size >total_num_pixel_beam_leds_<
   */
  Eigen::VectorXf pixel_beam_buffer_;

  float min_distance_;
  float max_distance_;

  tf::Vector3 min_space_;
  tf::Vector3 max_space_;

  std::vector<tf::Vector3> block_light_node_positions_;
  std::vector<tf::Vector3> block_light_beam_positions_;
  std::vector<tf::Vector3> pixel_light_beam_positions_;

};

}

#endif /* DEC_LIGHT_SHOW_BALL_CREATOR_H_ */
