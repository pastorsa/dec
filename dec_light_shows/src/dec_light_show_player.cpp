/*
 * dec_light_show_player.cpp
 *
 *  Created on: Jul 14, 2013
 *      Author: pastor
 */

#include <ros/package.h>

#include <dec_utilities/param_server.h>
#include <dec_utilities/file_io.h>
#include <dec_utilities/assert.h>
#include <dec_light_shows/light_show_constants.h>
#include <dec_light_show_manager/dec_light_show_utilities.h>

#include <dec_light_shows/dec_light_show_player.h>

PLUGINLIB_DECLARE_CLASS(dec_light_shows, DecLightShowPlayer, dec_light_shows::DecLightShowPlayer,
                        dec_light_show_manager::DecLightShow)

using namespace dec_light_show_manager;
using namespace dec_utilities;

namespace dec_light_shows
{

bool DecLightShowPlayer::initialize(XmlRpc::XmlRpcValue& config)
{
  abs_bag_file_name_ = ros::package::getPath(DATA_PACKAGE_NAME);
  dec_utilities::appendTrailingSlash(abs_bag_file_name_);
  abs_bag_file_name_.append(DATA_DIRECTORY_name);
  dec_utilities::appendTrailingSlash(abs_bag_file_name_);
  std::string bag_file_name;
  ROS_VERIFY(DecLightShowUtilities::getParam(config, "bag_file_name", bag_file_name));
  abs_bag_file_name_.append(bag_file_name + BAG_FILE_PREFIX);
  ROS_INFO("Reading from >%s<.", abs_bag_file_name_.c_str());

  return true;
}

bool DecLightShowPlayer::start()
{
  index_ = 0;
  return readFromDisc();
}

bool DecLightShowPlayer::update()
{
  setFrame();
  return true;
}

bool DecLightShowPlayer::stop()
{

  return true;
}

void DecLightShowPlayer::setFrame()
{
  if (index_ >= light_show_.frames.size())
  {
    index_ = 0;
  }

  for (unsigned int i = 0; i < data_->node_led_levels_.size(); ++i)
  {
    data_->node_led_levels_(i) += light_show_.frames[index_].block_node_levels[i];
    if (data_->node_led_levels_(i) > 1.0f)
      data_->node_led_levels_(i) = 1.0f;
  }
  for (unsigned int i = 0; i < data_->block_beam_led_levels_.size(); ++i)
  {
    data_->block_beam_led_levels_(i) += light_show_.frames[index_].block_beam_levels[i];
    if (data_->block_beam_led_levels_(i) > 1.0f)
      data_->block_beam_led_levels_(i) = 1.0f;
  }
  for (unsigned int i = 0; i < data_->pixel_beam_led_levels_.size(); ++i)
  {
    data_->pixel_beam_led_levels_(i) += light_show_.frames[index_].pixel_beam_levels[i];
    if (data_->pixel_beam_led_levels_(i) > 1.0f)
      data_->pixel_beam_led_levels_(i) = 1.0f;
  }

  index_++;
}

bool DecLightShowPlayer::readFromDisc()
{
  ROS_INFO("Reading from file >%s<.", abs_bag_file_name_.c_str());
  if (!FileIO<dec_msgs::LightShow>::readFromBagFile(light_show_, LIGHT_SHOW_TOPIC_NAME, abs_bag_file_name_))
  {
    ROS_ERROR("Problems reading >%s<.", abs_bag_file_name_.c_str());
    return false;
  }
  ROS_INFO("Read >%i< frames from file >%s<.", (int)light_show_.frames.size(), abs_bag_file_name_.c_str());
  return true;
}

}
