/*
 * dec_light_processor.cpp
 *
 *  Created on: Jul 14, 2013
 *      Author: pastor
 */

#include <dec_utilities/assert.h>
#include <dec_light_show_manager/dec_light_show_utilities.h>

#include <dec_light_shows/dec_light_processor.h>

PLUGINLIB_DECLARE_CLASS(dec_light_shows, DecLightProcessor, dec_light_shows::DecLightProcessor, dec_light_show_manager::DecLightShow)

using namespace dec_light_show_manager;

namespace dec_light_shows
{

DecLightProcessor::DecLightProcessor()
 // : num_cycles_per_buffer_shifts_(0), cycles_counter_(0)
{
}

bool DecLightProcessor::initialize(XmlRpc::XmlRpcValue& config)
{

//  double max_distance = data_->node_led_distances_to_sensor_.maxCoeff();
//  ROS_INFO("Maximum distance is %f", max_distance);
//
//  // read parameters
//  int size_circular_sensor_buffer = 0;
//  ROS_VERIFY(DecLightShowUtilities::getParam(config, "size_circular_sensor_buffer", size_circular_sensor_buffer));
//
//  circular_sensor_buffer_.reset(new boost::circular_buffer<Eigen::VectorXf>(size_circular_sensor_buffer));
//  Eigen::VectorXf zeros = Eigen::VectorXf::Zero(data_->sensor_levels_.rows());
//  for (int i = 0; i < size_circular_sensor_buffer; ++i)
//  {
//    circular_sensor_buffer_->push_front(zeros);
//  }
//
//  node_led_distances_to_sensor_weight_ = data_->node_led_distances_to_sensor_;
//  double distance_dividor;
//  ROS_VERIFY(DecLightShowUtilities::getParam(config, "distance_dividor", distance_dividor));
//  ROS_ASSERT(distance_dividor > 0.0);
//  node_led_distances_to_sensor_weight_.array() /= distance_dividor;
//
//  activations_ = Eigen::VectorXf::Zero(data_->sensor_levels_.rows());
  return true;
}

bool DecLightProcessor::start()
{

//  // fill buffer with current sensor values
//  Eigen::VectorXf zeros = Eigen::VectorXf::Zero(data_->sensor_levels_.rows());
//  for (unsigned int i = 0; i < circular_sensor_buffer_->size(); ++i)
//  {
//    circular_sensor_buffer_->push_front(zeros);
//  }
//
//  cycles_counter_ = 0;
  return true;
}

bool DecLightProcessor::update()
{

//  // take sensor_levels_ and convert them into node_led_level_ and beam_led_level_
//  data_->node_led_levels_.setZero();
//
//  if(++cycles_counter_ > num_cycles_per_buffer_shifts_)
//  {
//    ROS_INFO("Shifting...");
//    circular_sensor_buffer_->push_front(data_->sensor_levels_);
//    cycles_counter_ = 0;
//  }
//
//  boost::circular_buffer<Eigen::VectorXf>::const_iterator ci;
//  for (ci = circular_sensor_buffer_->begin(); ci != circular_sensor_buffer_->end(); ++ci)
//  {
//    data_->node_led_levels_ += data_->node_led_distances_to_sensor_ * (*ci);
//  }
//
//  // data_->node_led_levels_ = data_->node_led_distances_to_sensor_ * data_->sensor_levels_;
//  for (int i = 0; i < (int)data_->node_led_levels_.size(); ++i)
//  {
//    if(data_->node_led_levels_(i) > 1.0f)
//      data_->node_led_levels_(i) = 1.0f;
//  }
//
//  ROS_INFO("Done");
  return true;
}

bool DecLightProcessor::stop()
{

  return true;
}

}
