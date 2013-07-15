/*
 * dec_open_loop_light_generator.h
 *
 *  Created on: Jul 14, 2013
 *      Author: pastor
 */

#ifndef DEC_OPEN_LOOP_LIGHT_GENERATOR_H_
#define DEC_OPEN_LOOP_LIGHT_GENERATOR_H_

#include <dec_light_show_manager/dec_light_show.h>

namespace dec_light_shows
{

class DecOpenLoopLightGenerator : public dec_light_show_manager::DecLightShow
{
public:
  DecOpenLoopLightGenerator() {};
  virtual ~DecOpenLoopLightGenerator() {};

  virtual bool initialize(XmlRpc::XmlRpcValue& config);
  virtual bool update();
  virtual bool start();
  virtual bool stop();

private:

};

}

#endif /* DEC_OPEN_LOOP_LIGHT_GENERATOR_H_ */
