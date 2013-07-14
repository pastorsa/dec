/*
 * dec_processor.h
 *
 *  Created on: Jun 17, 2013
 *      Author: pastor
 */

#ifndef DEC_PROCESSOR_H_
#define DEC_PROCESSOR_H_

#include <boost/shared_ptr.hpp>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <dec_visualization/dec_data.h>

namespace dec_visualization
{

class DECProcessor : public DECData
{

  friend class DECVisualization;

public:

  DECProcessor();
  virtual ~DECProcessor() {};

  /*!
   * @param node_handle
   * @return
   */
  virtual bool initialize(ros::NodeHandle node_handle) = 0;

  /*! This function does the visualization depending of the interaction mode
   * @return True on success, otherwise False
   */
  virtual bool process() = 0;

protected:

  bool update();
  bool init(ros::NodeHandle node_handle);

private:

  static const double COLOR_RESOLUTION = 255.0;

  bool readParams(ros::NodeHandle node_handle);

  ros::NodeHandle node_handle_;
  ros::Publisher rviz_pub_;

  visualization_msgs::MarkerArray node_markers_;
  visualization_msgs::MarkerArray light_node_markers_;

  visualization_msgs::MarkerArray beam_markers_;
  visualization_msgs::MarkerArray sensor_markers_;
  visualization_msgs::MarkerArray light_beam_markers_;

  visualization_msgs::MarkerArray node_text_markers_;
  visualization_msgs::MarkerArray light_node_text_markers_;

  visualization_msgs::MarkerArray beam_text_markers_;
  visualization_msgs::MarkerArray sensor_text_markers_;
  visualization_msgs::MarkerArray light_beam_text_markers_;

  static void setupNodeMarkers(ros::NodeHandle& node_handle,
                               const std::string& namespace_name,
                               visualization_msgs::MarkerArray& node_markers,
                               visualization_msgs::MarkerArray& node_text_markers,
                               const std::vector<geometry_msgs::Point>& node_positions);
  static void setupBeamMarkers(ros::NodeHandle& node_handle,
                               const std::string& namespace_name,
                               visualization_msgs::MarkerArray& beam_markers,
                               const std::vector<geometry_msgs::Point>& node_positions,
                               const std::vector<std::pair<int, int> >& beams);

  static void setupTextMarkers(ros::NodeHandle& node_handle,
                               const std::string& namespace_name,
                               const visualization_msgs::MarkerArray& markers,
                               visualization_msgs::MarkerArray& text_markers);

  void publish();
  bool setLightMarkers();

  // void sendLightData();

  void setupMode();
  void localMode();

  boost::shared_ptr<dec_udp::DecInterface> dec_interface_;

};

}


#endif /* DEC_PROCESSOR_H_ */
