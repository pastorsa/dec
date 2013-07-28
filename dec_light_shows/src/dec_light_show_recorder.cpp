/*
 * dec_light_show_recorder.cpp
 *
 *  Created on: Jul 14, 2013
 *      Author: pastor
 */

#include <ros/package.h>

#include <dec_utilities/assert.h>
#include <dec_light_show_manager/dec_light_show_utilities.h>
#include <dec_light_shows/dec_light_show_recorder.h>
#include <dec_utilities/param_server.h>
#include <dec_utilities/file_io.h>
#include <dec_light_shows/light_show_constants.h>

PLUGINLIB_DECLARE_CLASS(dec_light_shows, DecLightShowRecorder, dec_light_shows::DecLightShowRecorder,
                        dec_light_show_manager::DecLightShow)

using namespace dec_light_show_manager;
using namespace dec_utilities;
using namespace dec_msgs;

namespace dec_light_shows
{

bool DecLightShowRecorder::initialize(XmlRpc::XmlRpcValue& config)
{
  abs_bag_file_name_ = ros::package::getPath(DATA_PACKAGE_NAME);
  dec_utilities::appendTrailingSlash(abs_bag_file_name_);
  abs_bag_file_name_.append(DATA_DIRECTORY_name);
  dec_utilities::appendTrailingSlash(abs_bag_file_name_);
  std::string bag_file_name;
  ROS_VERIFY(DecLightShowUtilities::getParam(config, "bag_file_name", bag_file_name));
  abs_bag_file_name_.append(bag_file_name + BAG_FILE_PREFIX);
  ROS_INFO("Writing to >%s<.", abs_bag_file_name_.c_str());

  frame_.block_node_levels.resize(data_->total_num_node_leds_, 0.0);
  frame_.block_beam_levels.resize(data_->total_num_block_beam_leds_, 0.0);
  frame_.pixel_beam_levels.resize(data_->total_num_pixel_beam_leds_, 0.0);

  return true;
}

bool DecLightShowRecorder::start()
{
  light_show_.frames.clear();
  return true;
}

bool DecLightShowRecorder::update()
{
  addFrame();
  return true;
}

bool DecLightShowRecorder::stop()
{
  return writeToDisc();
}

void DecLightShowRecorder::addFrame()
{
  frame_.header.stamp = data_->ros_time_;
  for (unsigned int i = 0; i < data_->node_led_levels_.size(); ++i)
  {
    frame_.block_node_levels[i] = data_->node_led_levels_(i);
  }

  for (unsigned int i = 0; i < data_->block_beam_led_levels_.size(); ++i)
  {
    frame_.block_beam_levels[i] = data_->block_beam_led_levels_(i);
  }

  for (unsigned int i = 0; i < data_->pixel_beam_led_levels_.size(); ++i)
  {
    frame_.pixel_beam_levels[i] = data_->pixel_beam_led_levels_(i);
  }
  light_show_.frames.push_back(frame_);
}

bool DecLightShowRecorder::writeToDisc()
{
  ROS_INFO("Writing >%i< frames into >%s<.", (int)light_show_.frames.size(), abs_bag_file_name_.c_str());
  if (!FileIO<dec_msgs::LightShow>::writeToBagFile(light_show_, LIGHT_SHOW_TOPIC_NAME, abs_bag_file_name_))
  {
    ROS_ERROR("Problems writing >%s<.", abs_bag_file_name_.c_str());
    return false;
  }
  return true;
}

}
