/*
 * dec_light_processor.h
 *
 *  Created on: Jul 14, 2013
 *      Author: pastor
 */

#ifndef DEC_LIGHTSWITCH_PROCESSOR_H_
#define DEC_LIGHTSWITCH_PROCESSOR_H_

// system includes
#include <dec_light_show_manager/dec_light_show.h>

// local includes
#include <boost/circular_buffer.hpp>
// #include <dec_light_shows/dec_circular_buffer.h>

namespace dec_light_shows
{

class DecLightSwitchProcessor : public dec_light_show_manager::DecLightShow
{
public:
  DecLightSwitchProcessor();
  virtual ~DecLightSwitchProcessor() {};

  virtual bool initialize(XmlRpc::XmlRpcValue& config);
  virtual bool update();
  virtual bool start();
  virtual bool stop();

private:

};

}

#endif /* DEC_LIGHTSWITCH_PROCESSOR_H_ */
