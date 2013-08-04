/*
 * color_trajectory_controller.h
 *
 *  Created on: Nov 14, 2010
 *      Author: kalakris
 */

#ifndef COLOR_TRAJECTORY_CONTROLLER_H_
#define COLOR_TRAJECTORY_CONTROLLER_H_

#include <splines/splines.h>
#include <dec_light_shows/trajectory_generator.h>

namespace dec_light_shows
{

class ColorTrajectoryGenerator: public TrajectoryGenerator
{
public:
  ColorTrajectoryGenerator();
  virtual ~ColorTrajectoryGenerator();

  virtual bool initialize(XmlRpc::XmlRpcValue& config);
  virtual bool update();
  virtual bool start();
  virtual bool stop();

protected:
  void updateCurSplineFromPoints(TrajectoryPoint& p1, TrajectoryPoint& p2);
  void getCurrentTrajectoryPoint(TrajectoryPoint& point);

private:

  bool readEndeffNamesFromConfig(XmlRpc::XmlRpcValue& config,
                                 const std::string& param_name,
                                 std::vector<std::string>& dimension_names,
                                 bool& param_exists,
                                 std::vector<int>& endeff_indices,
                                 const std::vector<std::string>& variable_suffixes);

  bool readJointNamesFromConfig(XmlRpc::XmlRpcValue& config,
                                const std::string& param_name,
                                std::vector<std::string>& dimension_names,
                                bool& param_exists,
                                std::vector<int>& joint_indices,
                                const std::vector<std::string>& variable_suffixes);

};

}

#endif /* COLOR_TRAJECTORY_CONTROLLER_H_ */
