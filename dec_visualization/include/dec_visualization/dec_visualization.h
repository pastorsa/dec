/*
 * dec_visualization.h
 *
 *  Created on: Jun 16, 2013
 *      Author: pastor
 */

#ifndef DEC_VISUALIZATION_H_
#define DEC_VISUALIZATION_H_

#include <vector>
#include <map>

#include <ros/ros.h>
#include <Eigen/Eigen>

#include <geometry_msgs/Point.h>

#include <dec_visualization/dec_processor.h>

namespace dec_visualization
{

class DECVisualization
{

public:

  DECVisualization(ros::NodeHandle node_handle);
  virtual ~DECVisualization() {};

  bool initialize(ros::NodeHandle node_handle);
  void run();

private:
  bool simulation_mode_;

  boost::shared_ptr<DECProcessor> dec_processor_;

  bool setGraphDistances();
  Eigen::MatrixXf graph_;

};

}

#endif /* DEC_VISUALIZATION_H_ */
