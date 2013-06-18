/*
 * dec_structure.h
 *
 *  Created on: Jun 17, 2013
 *      Author: pastor
 */

#ifndef DEC_STRUCTURE_H_
#define DEC_STRUCTURE_H_

#include <ros/ros.h>
#include <vector>

#include <geometry_msgs/Point.h>

namespace dec_visualization
{

class DECStructure
{

  friend class DECVisualization;
  friend class DECSimulation;
  friend class DECCommunication;
public:

  DECStructure() {};
  virtual ~DECStructure() {};

  bool initialize(ros::NodeHandle node_handle);

private:

  bool read(ros::NodeHandle node_handle,
            const std::string& array_name,
            std::vector<geometry_msgs::Point>& array);
  bool read(ros::NodeHandle node_handle,
            const std::string& array_name,
            std::vector<std::pair<int, int> >& nodes);

  bool read(ros::NodeHandle node_handle,
            const::std::string& array_name,
            std::vector<int>& values,
            const std::vector<std::pair<int, int> >& beams);

  std::vector<geometry_msgs::Point> node_positions_;
  std::vector<geometry_msgs::Point> light_node_positions_;

  /*! pairs of nodes in node_positions_ (indexed from 0)
   */
  std::vector<std::pair<int, int> > beams_;
  std::vector<std::pair<int, int> > sensors_;
  std::vector<std::pair<int, int> > light_beams_;

  /*! which light_beam or sensor is connected to which arduino
   */
  std::vector<int> light_beam_connections_;
  std::vector<int> sensor_connections_;

  int number_of_arduinos_;
    int max_number_of_sensors_per_arduino_;
  int max_number_of_lights_per_arduino_;


};

}


#endif /* DEC_STRUCTURE_H_ */
