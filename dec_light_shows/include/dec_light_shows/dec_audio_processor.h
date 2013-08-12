/*
 * dec_audio_processor.h
 *
 *  Created on: Jul 14, 2013
 *      Author: pastor
 */

#ifndef DEC_AUDIO_PROCESSOR_H_
#define DEC_AUDIO_PROCESSOR_H_

#include <boost/shared_ptr.hpp>
#include <dec_audio/audio_processor.h>
#include <dec_audio/AudioSample.h>

#include <boost/thread/mutex.hpp>

#include <dec_light_show_manager/dec_light_show.h>

namespace dec_light_shows
{

class DecAudioProcessor : public dec_light_show_manager::DecLightShow
{
public:

  // TODO : change this
  const static unsigned int NUM_AUDIO_SIGNALS = 8;

  DecAudioProcessor() :
  calibration_counts_(0),
  audio_receveived_counter_(0),
  audio_consumed_counter_(0),
  volume_(0.0),
  avg_volume_(0.0) {};
  virtual ~DecAudioProcessor() {};

  virtual bool initialize(XmlRpc::XmlRpcValue& config);
  virtual bool update();
  virtual bool start();
  virtual bool stop();

private:

  ros::Subscriber audio_sub_;

  void audioCB(dec_audio::AudioSampleConstPtr audio_sample);

  boost::mutex audio_sample_mutex_;
  dec_audio::AudioSample audio_sample_;

  unsigned int calibration_counts_;
  unsigned int audio_receveived_counter_;
  unsigned int audio_consumed_counter_;

  float volume_;
  float avg_volume_;

};

}

#endif /* DEC_AUDIO_PROCESSOR_H_ */
