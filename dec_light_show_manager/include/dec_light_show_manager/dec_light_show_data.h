/*
 * dec_light_show_data.h
 *
 *  Created on: Jul 14, 2013
 *      Author: pastor
 */

#ifndef DEC_LIGHT_SHOW_DATA_H_
#define DEC_LIGHT_SHOW_DATA_H_

#include <vector>
#include <ros/ros.h>

#include <dec_utilities/assert.h>
#include <dec_utilities/param_server.h>

#include <Eigen/Eigen>
#include <geometry_msgs/Point.h>

#include <dec_udp/dec_interface.h>

#include <dec_light_show_manager/dec_data.h>

namespace dec_light_show_manager
{

/**
 * This class contains public data to be shared across light shows
 */
class DecLightShowData : public DecData
{
  friend class DecLightShowManager;

public:
  DecLightShowData();
  virtual ~DecLightShowData();

private:

};

}

#endif /* DEC_LIGHT_SHOW_DATA_H_ */
