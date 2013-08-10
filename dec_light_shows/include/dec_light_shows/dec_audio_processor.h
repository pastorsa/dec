/*
 * dec_audio_processor.h
 *
 *  Created on: Jul 14, 2013
 *      Author: pastor
 */

#ifndef DEC_AUDIO_PROCESSOR_H_
#define DEC_AUDIO_PROCESSOR_H_

#include <dec_light_show_manager/dec_light_show.h>

namespace dec_light_shows
{

class DecAudioProcessor : public dec_light_show_manager::DecLightShow
{
public:
  DecAudioProcessor() {};
  virtual ~DecAudioProcessor() {};

  virtual bool initialize(XmlRpc::XmlRpcValue& config);
  virtual bool update();
  virtual bool start();
  virtual bool stop();

private:

};

}

#endif /* DEC_AUDIO_PROCESSOR_H_ */
