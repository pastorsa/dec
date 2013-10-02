/*
 * dec_sensor_rise_processor.h
 *
 *  Created on: Jul 14, 2013
 *      Author: pastor
 */

#ifndef DEC_SENSOR_WINDOWING_H_
#define DEC_SENSOR_WINDOWING_H_

#include <dec_light_show_manager/dec_light_show.h>

namespace dec_light_shows
{

class DecSensorWindowing : public dec_light_show_manager::DecLightShow
{
public:
  DecSensorWindowing() : up_fraction_(0.0), down_fraction_(0.0) {};
  virtual ~DecSensorWindowing() {};

  virtual bool initialize(XmlRpc::XmlRpcValue& config);
  virtual bool update();
  virtual bool start();
  virtual bool stop();

private:

  float up_fraction_;
  float down_fraction_;

};

}

#endif /* DEC_SENSOR_WINDOWING_H_ */
