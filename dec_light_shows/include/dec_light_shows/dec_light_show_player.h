/*
 * dec_light_show_player.h
 *
 *  Created on: Jul 14, 2013
 *      Author: pastor
 */

#ifndef DEC_LIGHT_SHOW_PLAYER_H_
#define DEC_LIGHT_SHOW_PLAYER_H_

#include <dec_msgs/LightShow.h>
#include <dec_light_show_manager/dec_light_show.h>

namespace dec_light_shows
{

class DecLightShowPlayer : public dec_light_show_manager::DecLightShow
{
public:
  DecLightShowPlayer() : index_(0) {};
  virtual ~DecLightShowPlayer() {};

  virtual bool initialize(XmlRpc::XmlRpcValue& config);
  virtual bool update();
  virtual bool start();
  virtual bool stop();

private:

  bool readFromDisc();
  void setFrame();
  unsigned int index_;

  std::string abs_bag_file_name_;
  dec_msgs::LightShow light_show_;

};

}

#endif /* DEC_LIGHT_SHOW_PLAYER_H_ */
