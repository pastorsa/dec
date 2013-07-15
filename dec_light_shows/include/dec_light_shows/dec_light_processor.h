/*
 * dec_light_processor.h
 *
 *  Created on: Jul 14, 2013
 *      Author: pastor
 */

#ifndef DEC_LIGHT_PROCESSOR_H_
#define DEC_LIGHT_PROCESSOR_H_

#include <dec_light_show_manager/dec_light_show.h>

namespace dec_light_shows
{

class DecLightProcessor : public dec_light_show_manager::DecLightShow
{
public:
  DecLightProcessor() {};
  virtual ~DecLightProcessor() {};

  virtual bool initialize(XmlRpc::XmlRpcValue& config);
  virtual bool update();
  virtual bool start();
  virtual bool stop();

private:

};

}

#endif /* DEC_LIGHT_PROCESSOR_H_ */
