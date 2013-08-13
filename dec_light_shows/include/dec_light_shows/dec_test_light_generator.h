/*
 * dec_open_loop_light_generator.h
 *
 *  Created on: Jul 14, 2013
 *      Author: pastor
 */

#ifndef DEC_OPEN_LOOP_LIGHT_GENERATOR_H_
#define DEC_OPEN_LOOP_LIGHT_GENERATOR_H_

#include <boost/thread/mutex.hpp>

#include <dec_msgs/LightShowFrame.h>
#include <dec_msgs/Test.h>

#include <dec_light_show_manager/dec_light_show.h>

namespace dec_light_shows
{

class DecTestLightGenerator : public dec_light_show_manager::DecLightShow
{
public:
  DecTestLightGenerator() :
    update_(false) {};
  virtual ~DecTestLightGenerator() {};

  virtual bool initialize(XmlRpc::XmlRpcValue& config);
  virtual bool update();
  virtual bool start();
  virtual bool stop();

private:

  bool testService(dec_msgs::Test::Request& request, dec_msgs::Test::Response& response);

  bool update_;
  ros::ServiceServer test_service_server_;
  boost::mutex mutex_;

  dec_msgs::LightShowFrame light_show_frame_;

};

}

#endif /* DEC_OPEN_LOOP_LIGHT_GENERATOR_H_ */
