/*
 * dec_audio_processor.cpp
 *
 *  Created on: Jul 14, 2013
 *      Author: pastor
 */

#include <dec_utilities/assert.h>
#include <dec_light_show_manager/dec_light_show_utilities.h>

#include <dec_light_shows/dec_audio_processor.h>

PLUGINLIB_DECLARE_CLASS(dec_light_shows, DecAudioProcessor, dec_light_shows::DecAudioProcessor,
                        dec_light_show_manager::DecLightShow)

using namespace dec_light_show_manager;

namespace dec_light_shows
{

bool DecAudioProcessor::initialize(XmlRpc::XmlRpcValue& config)
{
  float calibration_time_in_seconds = 0.0;
  ROS_VERIFY(DecLightShowUtilities::getParam(config, "calibration_time_in_seconds", calibration_time_in_seconds));
  ROS_ASSERT_MSG(calibration_time_in_seconds > data_->control_dt_, "Calibration time in seconds >%.2f< must be greater than >%.2f< seconds.",
                 calibration_time_in_seconds, data_->control_dt_);
  calibration_counts_ = static_cast<unsigned int>(calibration_time_in_seconds / data_->control_dt_);
  ROS_ASSERT(calibration_counts_ > 0);
  calibration_counts_++;

  audio_sample_.data.resize(NUM_AUDIO_SIGNALS, 0.0);

  // do this last
  const int SUBSCRIBER_BUFFER_SIZE = 1;
  audio_sub_ = data_->node_handle_.subscribe("/AudioProcessor/audio_samples", SUBSCRIBER_BUFFER_SIZE, &DecAudioProcessor::audioCB, this);

  return true;
}

bool DecAudioProcessor::start()
{
  avg_volume_ = 0.0;
  boost::mutex::scoped_lock lock(audio_sample_mutex_);
  audio_receveived_counter_ = 0;
  audio_consumed_counter_ = 0;
  return true;
}

void DecAudioProcessor::audioCB(dec_audio::AudioSampleConstPtr audio_sample)
{
  ROS_ASSERT_MSG(audio_sample->data.size() == NUM_AUDIO_SIGNALS,
                 "Number of hardcoded audio signals >%i< does not match number of received audio signals >%i<.",
                 (int)NUM_AUDIO_SIGNALS, (int)audio_sample->data.size());
  boost::mutex::scoped_lock lock(audio_sample_mutex_);
  audio_receveived_counter_++;
  audio_sample_ = *audio_sample;
}

bool DecAudioProcessor::update()
{
  if(audio_consumed_counter_ < audio_receveived_counter_)
  {
    if (audio_consumed_counter_ < calibration_counts_)
    {
      float avg_volume = 0.0;
      for (unsigned int i = 0; i < audio_sample_.data.size(); ++i)
      {
        avg_volume += audio_sample_.data[i];
      }
      avg_volume /= static_cast<float>(audio_sample_.data.size());
      avg_volume_ += avg_volume;
      if (audio_consumed_counter_ + 1 == calibration_counts_)
      {
        avg_volume_ /= static_cast<float>(calibration_counts_);
      }
    }
    else
    {
      volume_ = 0.0;
      for (unsigned int i = 0; i < audio_sample_.data.size(); ++i)
      {
        volume_ += audio_sample_.data[i];
      }
      volume_ /= static_cast<float>(audio_sample_.data.size());
      volume_ -= avg_volume_;

      if (volume_ > 1.0f)
      {
        volume_ = 1.0f;
      }
      const led_channel_t BASE_BRIGHTNESS = 70;
      led_channel_t brightness = BASE_BRIGHTNESS + static_cast<led_channel_t>(volume_ * 400);
      if (brightness > 255)
        brightness = 255;
      // ROS_INFO("volume: %.2f -> brightness %i", volume_, brightness);
      for (int i = 0; i < (int)data_->node_led_levels_.size(); ++i)
      {
        data_->node_led_values_(BRIGHTNESS_OFFSET, i) = static_cast<led_channel_t>(brightness);
      }

      if (data_->total_num_block_beam_leds_ > 0)
      {
        for (int i = 0; i < (int)data_->block_beam_led_levels_.size(); ++i)
        {
          data_->block_beam_led_values_(BRIGHTNESS_OFFSET, i) = static_cast<led_channel_t>(brightness);
        }
      }

      if (data_->total_num_pixel_beam_leds_ > 0)
      {
        for (int i = 0; i < (int)data_->pixel_beam_led_levels_.size(); ++i)
        {
          data_->pixel_beam_led_values_(BRIGHTNESS_OFFSET, i) = static_cast<led_channel_t>(brightness);
        }
      }

    }
    audio_consumed_counter_++;
  }
  return true;
}

bool DecAudioProcessor::stop()
{

  return true;
}

}
