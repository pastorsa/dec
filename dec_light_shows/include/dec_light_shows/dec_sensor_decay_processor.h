/*
 * dec_sensor_decay_processor.h
 *
 *  Created on: Jul 14, 2013
 *      Author: pastor
 */

#ifndef DEC_SENSOR_DECAY_PROCESSOR_H_
#define DEC_SENSOR_DECAY_PROCESSOR_H_

#include <dec_light_show_manager/dec_light_show.h>

namespace dec_light_shows
{

class DecSensorDecayProcessor : public dec_light_show_manager::DecLightShow
{
public:
  DecSensorDecayProcessor()
  : decay_type_(LINEAR), window_size_(0) {};
  virtual ~DecSensorDecayProcessor() {};

  virtual bool initialize(XmlRpc::XmlRpcValue& config);
  virtual bool update();
  virtual bool start();
  virtual bool stop();

private:

  enum DecayType
  {
    LINEAR,
    NUM_DECAY_TYPES
  };
  DecayType decay_type_;

  unsigned int window_size_;
  std::vector<unsigned int> indices_;
  std::vector<float> values_;



};

}

#endif /* DEC_SENSOR_DECAY_PROCESSOR_H_ */
