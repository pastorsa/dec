/*
 * dec_idle.h
 *
 *  Created on: Jul 14, 2013
 *      Author: pastor
 */

#ifndef DEC_IDLE_H_
#define DEC_IDLE_H_

#include <dec_light_show_manager/dec_light_show.h>

namespace dec_light_shows
{

class DecIdle : public dec_light_show_manager::DecLightShow
{
public:
  DecIdle() {};
  virtual ~DecIdle() {};

  virtual bool initialize(XmlRpc::XmlRpcValue& config);
  virtual bool update();
  virtual bool start();
  virtual bool stop();

private:

};

}

#endif /* DEC_IDLE_H_ */
