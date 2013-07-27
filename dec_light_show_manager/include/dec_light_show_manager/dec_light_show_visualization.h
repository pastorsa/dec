/*
 * dec_light_show_visualization.h
 *
 *  Created on: Jun 17, 2013
 *      Author: pastor
 */

#ifndef DEC_LIGHT_SHOW_VISUALIZATION_H_
#define DEC_LIGHT_SHOW_VISUALIZATION_H_

#include <boost/shared_ptr.hpp>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <dec_light_show_manager/dec_light_show_data.h>

namespace dec_light_show_manager
{

class DecLightShowVisualization
{

public:

  DecLightShowVisualization();
  virtual ~DecLightShowVisualization() {};

  /*!
   * @param light_node_data
   * @param node_handle
   * @return True on success, otherwise False
   */
  bool initialize(boost::shared_ptr<DecLightShowData> light_show_data, ros::NodeHandle node_handle);

  /*! This function does the visualization
   * @return True on success, otherwise False
   */
  bool update();

private:

  bool readParams(ros::NodeHandle node_handle);

  ros::NodeHandle node_handle_;
  ros::Publisher rviz_pub_;

  unsigned int static_publish_counter_;
  unsigned int static_publish_rate_;

  visualization_msgs::MarkerArray node_markers_;
  visualization_msgs::MarkerArray beam_markers_;
  visualization_msgs::MarkerArray sensor_markers_;
  visualization_msgs::MarkerArray avg_sensor_markers_;

  visualization_msgs::MarkerArray block_light_node_markers_;
  visualization_msgs::MarkerArray block_light_beam_markers_;
  visualization_msgs::MarkerArray pixel_light_beam_markers_;

  visualization_msgs::MarkerArray node_text_markers_;
  visualization_msgs::MarkerArray beam_text_markers_;
  visualization_msgs::MarkerArray sensor_text_markers_;
  visualization_msgs::MarkerArray avg_sensor_text_markers_;

  float initial_avg_sensor_alpha_;
  float initial_sensor_alpha_;

  visualization_msgs::MarkerArray block_light_node_text_markers_;
  visualization_msgs::MarkerArray block_light_beam_text_markers_;
  visualization_msgs::MarkerArray pixel_light_beam_text_markers_;

  visualization_msgs::MarkerArray panel_markers_;

  static void setupNodeMarkers(ros::NodeHandle& node_handle,
                               const std::string& namespace_name,
                               visualization_msgs::MarkerArray& node_markers,
                               const std::vector<geometry_msgs::Point>& node_positions,
                               const bool forever = false);
  static void setupBeamMarkers(ros::NodeHandle& node_handle,
                               const std::string& namespace_name,
                               visualization_msgs::MarkerArray& beam_markers,
                               const std::vector<geometry_msgs::Pose>& poses,
                               const bool forever = false);
  static void setupTextMarkers(ros::NodeHandle& node_handle,
                               const std::string& namespace_name,
                               const visualization_msgs::MarkerArray& markers,
                               visualization_msgs::MarkerArray& text_markers,
                               const bool forever = false);

  void setupPixelBeamMarkers(ros::NodeHandle& node_handle,
                             const std::string& namespace_name,
                             visualization_msgs::MarkerArray& beam_markers,
                             const std::vector<PixelLightBeam>& pixel_light_beams,
                             const std::vector<geometry_msgs::Pose>& poses);

  void setupPanelMarkers(ros::NodeHandle& node_handle,
                         const std::vector<geometry_msgs::Point>& node_positions);

  boost::shared_ptr<DecLightShowData> light_show_data_;
};

}


#endif /* DEC_LIGHT_SHOW_VISUALIZATION_H_ */
