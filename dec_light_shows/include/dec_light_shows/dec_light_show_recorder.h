/*
 * dec_light_show_recorder.h
 *
 *  Created on: Jul 14, 2013
 *      Author: pastor
 */

#ifndef DEC_LIGHT_SHOW_RECORDER_H_
#define DEC_LIGHT_SHOW_RECORDER_H_

#include <dec_msgs/LightShow.h>
#include <dec_msgs/LightShowFrame.h>

#include <dec_light_show_manager/dec_light_show.h>

namespace dec_light_shows
{

class DecLightShowRecorder : public dec_light_show_manager::DecLightShow
{
public:
  DecLightShowRecorder() {};
  virtual ~DecLightShowRecorder() {};

  virtual bool initialize(XmlRpc::XmlRpcValue& config);
  virtual bool update();
  virtual bool start();
  virtual bool stop();

private:

  bool writeToDisc();

  std::string abs_bag_file_name_;
  dec_msgs::LightShow light_show_;
  dec_msgs::LightShowFrame frame_;
  void addFrame();

};

}

#endif /* DEC_LIGHT_SHOW_RECORDER_H_ */
