/*
 * dec_light_processor.h
 *
 *  Created on: Jul 14, 2013
 *      Author: pastor
 */

#ifndef DEC_LIGHT_PROCESSOR_H_
#define DEC_LIGHT_PROCESSOR_H_

// system includes
#include <dec_light_show_manager/dec_light_show.h>

// local includes
#include <boost/circular_buffer.hpp>
// #include <dec_light_shows/dec_circular_buffer.h>

namespace dec_light_shows
{

class DecLightProcessor : public dec_light_show_manager::DecLightShow
{
public:
  DecLightProcessor();
  virtual ~DecLightProcessor() {};

  virtual bool initialize(XmlRpc::XmlRpcValue& config);
  virtual bool update();
  virtual bool start();
  virtual bool stop();

private:

//  boost::shared_ptr<boost::circular_buffer<Eigen::VectorXf> > circular_sensor_buffer_;
//  Eigen::MatrixXf node_led_distances_to_sensor_weight_;
//
//  Eigen::VectorXf activations_;
//
//  int num_cycles_per_buffer_shifts_;
//  int cycles_counter_;

};

}

#endif /* DEC_LIGHT_PROCESSOR_H_ */
