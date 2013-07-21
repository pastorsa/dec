/*
 * dec_light_show_simulation.h
 *
 *  Created on: Jun 17, 2013
 *      Author: pastor
 */

#ifndef DEC_LIGHT_SHOW_SIMULATION_H_
#define DEC_LIGHT_SHOW_SIMULATION_H_

#include <ros/ros.h>

#include <boost/shared_ptr.hpp>
#include <dec_world_state/world_state.h>

#include <dec_light_show_manager/dec_light_show_data.h>

namespace dec_light_show_manager
{

class DecLightShowSimulation
{

public:

  DecLightShowSimulation(boost::shared_ptr<DecLightShowData> light_show_data);
  virtual ~DecLightShowSimulation() {};

  /*!
   * @param node_handle
   * @return True on success, otherwise False
   */
  bool initialize(ros::NodeHandle node_handle);

  /*!
   * @return True on success, otherwise False
   */
  bool copySimulatedSensorInformation();

private:

  double getMinimumDistance(tf::Point point, std::vector<std::pair<int, int> > beams);
  double getDistance(tf::Point point, std::pair<int, int> beam);

  dec_world_state::WorldState world_state_;
  int number_of_simulated_objects_;
  std::string simulated_object_name_;

  double simulation_sensor_object_threshold_;

  boost::shared_ptr<DecLightShowData> light_show_data_;
};

}


#endif /* DEC_LIGHT_SHOW_SIMULATION_H_ */
