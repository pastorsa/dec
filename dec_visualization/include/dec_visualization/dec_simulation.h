/*
 * dec_simulation.h
 *
 *  Created on: Jun 17, 2013
 *      Author: pastor
 */

#ifndef DEC_SIMULATION_H_
#define DEC_SIMULATION_H_

#include <boost/shared_ptr.hpp>
#include <ros/ros.h>

#include <dec_world_state/world_state.h>

#include <dec_visualization/dec_processor.h>

namespace dec_visualization
{

class DECSimulation : public DECProcessor
{

public:

  DECSimulation();
  virtual ~DECSimulation() {};

  /*!
   * @param node_handle
   * @return True on success, otherwise False
   */
  bool initialize(ros::NodeHandle node_handle);

  /*!
   * @return True on success, otherwise False
   */
  bool process();

private:

  double getDistance(tf::Point point, std::pair<int, int> beam);

  dec_world_state::WorldState world_state_;
  int number_of_simulated_objects_;
  std::string simulated_object_name_;

  double simulation_sensor_object_threshold_;

};

}



#endif /* DEC_SIMULATION_H_ */
