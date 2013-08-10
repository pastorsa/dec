/*
 * dec_audio_modulation.h
 *
 *  Created on: Jul 14, 2013
 *      Author: pastor
 */

#ifndef DEC_AUDIO_MODULATION_H_
#define DEC_AUDIO_MODULATION_H_

#include <dec_light_show_manager/dec_light_show.h>

namespace dec_light_shows
{

class DecAudioModulation : public dec_light_show_manager::DecLightShow
{
public:
  DecAudioModulation() {};
  virtual ~DecAudioModulation() {};

  virtual bool initialize(XmlRpc::XmlRpcValue& config);
  virtual bool update();
  virtual bool start();
  virtual bool stop();

private:

};

}

#endif /* DEC_AUDIO_MODULATION_H_ */
