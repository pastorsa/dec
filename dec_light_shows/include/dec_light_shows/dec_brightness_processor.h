/*
 * dec_brightness_processor.h
 *
 *  Created on: Jul 14, 2013
 *      Author: pastor
 */

#ifndef DEC_BRIGHTNESS_PROCESSOR_H_
#define DEC_BRIGHTNESS_PROCESSOR_H_

#include <ros/ros.h>
#include <boost/thread/mutex.hpp>

#include <dec_msgs/Brightness.h>

#include <dec_light_show_manager/dec_light_show.h>

namespace dec_light_shows
{

class DecBrightnessProcessor : public dec_light_show_manager::DecLightShow
{
public:
  DecBrightnessProcessor()
  : node_brightness_(0),
    beam_brightness_(0),
    min_brightness_(0),
    max_brightness_(0) {};
  virtual ~DecBrightnessProcessor() {};

  virtual bool initialize(XmlRpc::XmlRpcValue& config);
  virtual bool update();
  virtual bool start();
  virtual bool stop();

private:
  ros::Subscriber brightness_sub_;

  boost::mutex mutex_;
  void brightnessCB(dec_msgs::BrightnessConstPtr brightness);

  unsigned int node_brightness_;
  unsigned int beam_brightness_;
  unsigned int min_brightness_;
  unsigned int max_brightness_;

};

}

#endif /* DEC_BRIGHTNESS_PROCESSOR_H_ */
