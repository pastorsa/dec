/*
 * dec_test_light_generator.cpp
 *
 *  Created on: Jul 14, 2013
 *      Author: pastor
 */

#include <dec_light_shows/dec_test_light_generator.h>

PLUGINLIB_DECLARE_CLASS(dec_light_shows, DecTestLightGenerator, dec_light_shows::DecTestLightGenerator,
                        dec_light_show_manager::DecLightShow)

namespace dec_light_shows
{

bool DecTestLightGenerator::initialize(XmlRpc::XmlRpcValue& config)
{
  light_show_frame_.block_node_levels.resize(data_->total_num_node_leds_, 0.0);
  light_show_frame_.block_beam_levels.resize(data_->total_num_block_beam_leds_, 0.0);
  light_show_frame_.pixel_beam_levels.resize(data_->total_num_pixel_beam_leds_, 0.0);

  // advertise service, do this last
  test_service_server_ = data_->node_handle_.advertiseService("test", &DecTestLightGenerator::testService, this);

  return true;
}

bool DecTestLightGenerator::testService(dec_msgs::Test::Request& request, dec_msgs::Test::Response& response)
{
  if (request.frame.block_node_levels.empty()
      && request.frame.block_beam_levels.empty()
      && request.frame.pixel_beam_levels.empty())
  {
    response.frame.block_node_levels.resize(data_->total_num_node_leds_, 0.0);
    ROS_ASSERT(data_->total_num_node_leds_ == data_->node_led_levels_.size());
    for (unsigned int i = 0; i < data_->total_num_node_leds_; ++i)
    {
      response.frame.block_node_levels[i] = data_->node_led_levels_[i];
    }
    response.frame.block_beam_levels.resize(data_->total_num_block_beam_leds_, 0.0);
    ROS_ASSERT(data_->total_num_block_beam_leds_ == data_->block_beam_led_levels_.size());
    for (unsigned int i = 0; i < data_->total_num_block_beam_leds_; ++i)
    {
      response.frame.block_beam_levels[i] = data_->block_beam_led_levels_[i];
    }
    response.frame.pixel_beam_levels.resize(data_->total_num_pixel_beam_leds_, 0.0);
    ROS_ASSERT(data_->total_num_pixel_beam_leds_ == data_->pixel_beam_led_levels_.size());
    for (unsigned int i = 0; i < data_->total_num_pixel_beam_leds_; ++i)
    {
      response.frame.pixel_beam_levels[i] = data_->pixel_beam_led_levels_[i];
    }
    response.result = response.SUCCEEDED;
    return true;
  }

  if (request.frame.block_node_levels.size() != data_->total_num_node_leds_)
  {
    ROS_ERROR("Invalid number of block nodes >%i<.", (int)request.frame.block_node_levels.size());
    response.result = response.FAILED;
    return true;
  }
  if (request.frame.block_beam_levels.size() != data_->total_num_block_beam_leds_)
  {
    ROS_ERROR("Invalid number of block beams >%i<.", (int)request.frame.block_beam_levels.size());
    response.result = response.FAILED;
    return true;
  }
  if (request.frame.pixel_beam_levels.size() != data_->total_num_pixel_beam_leds_)
  {
    ROS_ERROR("Invalid number of pixel beams >%i<.", (int)request.frame.pixel_beam_levels.size());
    response.result = response.FAILED;
    return true;
  }

  response.result = response.SUCCEEDED;
  boost::mutex::scoped_lock lock(mutex_);
  update_ = true;
  light_show_frame_ = request.frame;
  return true;
}

bool DecTestLightGenerator::start()
{
  update_ = false;
  return true;
}

bool DecTestLightGenerator::update()
{
  boost::mutex::scoped_lock lock(mutex_);
  if (update_)
  {
    update_ = false;
    for (unsigned int i = 0; i < data_->total_num_node_leds_; ++i)
    {
      data_->node_led_levels_[i] = light_show_frame_.block_node_levels[i];
    }
    for (unsigned int i = 0; i < data_->total_num_block_beam_leds_; ++i)
    {
      data_->block_beam_led_levels_[i] = light_show_frame_.block_beam_levels[i];
    }
    for (unsigned int i = 0; i < data_->total_num_pixel_beam_leds_; ++i)
    {
      data_->pixel_beam_led_levels_[i] = light_show_frame_.pixel_beam_levels[i];
    }
  }
  return true;
}

bool DecTestLightGenerator::stop()
{

  return true;
}

}
