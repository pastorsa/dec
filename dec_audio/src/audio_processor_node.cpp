/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		audio_processor_node.cpp

  \author	Peter Pastor
  \date		Jun 10, 2011

 *********************************************************************/

// system includes
#include <ros/ros.h>
#include <dec_utilities/assert.h>

// local includes
#include <dec_audio/audio_processor.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "");
  ros::NodeHandle node_handle("~");
  dec_audio::AudioProcessor audio_processor(node_handle);
  ROS_VERIFY(audio_processor.initialize());
  ros::spin();
  return 0;
}
