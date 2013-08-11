/*
 * dec_sensor_butterworth_processor.h
 *
 *  Created on: Jul 14, 2013
 *      Author: pastor
 */

#ifndef DEC_SENSOR_BUTTERWORTH_PROCESSOR_H_
#define DEC_SENSOR_BUTTERWORTH_PROCESSOR_H_

#include <vector>
#include <boost/shared_ptr.hpp>
#include <filters/transfer_function.h>

#include <dec_light_show_manager/dec_light_show.h>
#include <dec_light_show_manager/dec_data.h>

// local include
#include <dec_light_shows/dec_circular_buffer.h>

namespace dec_light_shows
{

class DecSensorButterworthProcessor : public dec_light_show_manager::DecLightShow
{
public:
  DecSensorButterworthProcessor();
  virtual ~DecSensorButterworthProcessor() {};

  virtual bool initialize(XmlRpc::XmlRpcValue& config);
  virtual bool update();
  virtual bool start();
  virtual bool stop();

private:

  filters::MultiChannelTransferFunctionFilter<float> filter_;
  std::vector<float> unfiltered_data_;
  std::vector<float> filtered_data_;

};

}

#endif /* DEC_SENSOR_BUTTERWORTH_PROCESSOR_H_ */
