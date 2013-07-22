/*
 * dec_sensor_processor.h
 *
 *  Created on: Jul 14, 2013
 *      Author: pastor
 */

#ifndef DEC_SENSOR_PROCESSOR_H_
#define DEC_SENSOR_PROCESSOR_H_

#include <vector>
#include <boost/shared_ptr.hpp>
#include <filters/transfer_function.h>

#include <dec_light_show_manager/dec_light_show.h>
#include <dec_light_show_manager/dec_data.h>

// local include
#include <dec_light_shows/dec_circular_buffer.h>

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

  filters::MultiChannelTransferFunctionFilter<float> filter_;
  std::vector<float> unfiltered_data_;
  std::vector<float> filtered_data_;

//   int num_cycles_to_load_;
//   int num_cycles_to_unload_;
//   std::vector<boost::shared_ptr<DecCircularBuffer<dec_light_show_manager::sensor_channel_t> > > load_circular_buffers_;
//   std::vector<boost::shared_ptr<DecCircularBuffer<dec_light_show_manager::sensor_channel_t> > > unload_circular_buffers_;

};

}

#endif /* DEC_SENSOR_PROCESSOR_H_ */
