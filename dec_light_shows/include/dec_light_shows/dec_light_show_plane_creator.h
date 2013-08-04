/*
 * dec_light_show_plane_creator.h
 *
 *  Created on: Jul 14, 2013
 *      Author: pastor
 */

#ifndef DEC_LIGHT_SHOW_PLANE_CREATOR_H_
#define DEC_LIGHT_SHOW_PLANE_CREATOR_H_

#include <vector>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <dec_light_show_manager/dec_light_show.h>
#include <dec_light_shows/dec_light_show_math_utilities.h>

namespace dec_light_shows
{

class DecLightShowPlaneCreator : public dec_light_show_manager::DecLightShow
{
public:
  DecLightShowPlaneCreator();
  virtual ~DecLightShowPlaneCreator() {};

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
  std::vector<tf::Vector3> normals_;
  std::vector<tf::Vector3> positions_;

  std::vector<tf::Vector3> linear_velocities_;
  std::vector<tf::Vector3> angular_velocities_;

  std::vector<tf::Vector3> linear_accelerations_;
  std::vector<tf::Vector3> angular_accelerations_;

  std::vector<tf::Vector3> simulated_linear_accelerations_;
  std::vector<tf::Vector3> simulated_angular_accelerations_;

  geometry_msgs::Quaternion getOrientation(tf::Vector3& normal);

  float computeDistance(const tf::Vector3& point,
                        const tf::Vector3& plane_vector,
                        const tf::Vector3& plane_normal);

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
  float distance_range_;

  bool up_down_;
  MathUtilities::Profile profile_type_;

  tf::Vector3 min_space_;
  tf::Vector3 max_space_;

  std::vector<tf::Vector3> block_light_node_positions_;
  std::vector<tf::Vector3> block_light_beam_positions_;
  std::vector<tf::Vector3> pixel_light_beam_positions_;

};

}

#endif /* DEC_LIGHT_SHOW_PLANE_CREATOR_H_ */
