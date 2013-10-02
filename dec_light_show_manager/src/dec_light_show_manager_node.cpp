/*
 * dec_light_show_manager_node.cpp
 *
 *  Created on: Jun 16, 2013
 *      Author: pastor
 */

#include <ros/ros.h>
#include <dec_light_show_manager/dec_light_show_manager.h>
#include <dec_utilities/assert.h>
#include <dec_utilities/param_server.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "~");
  ros::NodeHandle node_handle;
  dec_light_show_manager::DecLightShowManager dec_light_show_manager;
  if(!dec_light_show_manager.initialize())
  {
    return 0;
  }

  message_filters::Subscriber<dec_msgs::LightFrame> light_frame1_sub(node_handle, "/CloudProcessor1/light_frame", 1);
  message_filters::Subscriber<dec_msgs::LightFrame> light_frame2_sub(node_handle, "/CloudProcessor2/light_frame", 1);

  // message_filters::TimeSynchronizer<dec_msgs::LightFrame, dec_msgs::LightFrame> sync(light_frame1_sub, light_frame2_sub, 10);

  typedef message_filters::sync_policies::ApproximateTime<dec_msgs::LightFrame, dec_msgs::LightFrame> SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(3), light_frame1_sub, light_frame2_sub);
  sync.registerCallback(boost::bind(&dec_light_show_manager::DecLightShowManager::lightFrame, &dec_light_show_manager, _1, _2));

  ros::MultiThreadedSpinner mts;
  mts.spin();

  // ros::spin();

  // ros::AsyncSpinner spinner(0); // number of processor cores
  // spinner.start();
  // dec_light_show_manager.run();
  // spinner.stop();

  return 0;
}
