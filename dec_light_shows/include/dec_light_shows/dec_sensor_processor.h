/*
 * dec_sensor_processor.h
 *
 *  Created on: Jul 14, 2013
 *      Author: pastor
 */

#ifndef DEC_SENSOR_PROCESSOR_H_
#define DEC_SENSOR_PROCESSOR_H_

#include <dec_light_show_manager/dec_light_show.h>

namespace dec_light_shows
{

class DecSensorProcessor : public dec_light_show_manager::DecLightShow
{
public:
  DecSensorProcessor();
  virtual ~DecSensorProcessor() {};

  virtual bool initialize(XmlRpc::XmlRpcValue& config);
  virtual bool update();
  virtual bool start();
  virtual bool stop();

private:

  int build_up_window_size_;

};

}

#endif /* DEC_SENSOR_PROCESSOR_H_ */
