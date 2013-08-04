/*
 * dec_light_processor.h
 *
 *  Created on: Jul 14, 2013
 *      Author: pastor
 */

#ifndef DEC_LIGHT_PROCESSOR_H_
#define DEC_LIGHT_PROCESSOR_H_

// system includes
#include <dec_light_show_manager/dec_light_show.h>

// local includes
#include <boost/circular_buffer.hpp>
// #include <dec_light_shows/dec_circular_buffer.h>

namespace dec_light_shows
{

class DecLightProcessor : public dec_light_show_manager::DecLightShow
{
public:
  DecLightProcessor();
  virtual ~DecLightProcessor() {};

  virtual bool initialize(XmlRpc::XmlRpcValue& config);
  virtual bool update();
  virtual bool start();
  virtual bool stop();

private:

  /*!
   */
  Eigen::MatrixXf normalized_node_led_distances_to_sensor_;
  Eigen::MatrixXf normalized_block_beam_led_distances_to_sensor_;
  Eigen::MatrixXf normalized_pixel_beam_led_distances_to_sensor_;

  /*!
   */
  unsigned int filter_ring_index_;

  /*!
   */
  unsigned int filter_size_;

  /*! This buffer is of size >total_num_node_leds_ x filter_size_<
   */
  Eigen::MatrixXf block_node_buffer_;
  /*! This buffer is of size >total_num_block_beam_leds_ x filter_size_<
   */
  Eigen::MatrixXf block_beam_buffer_;
  /*! This buffer is of size >total_num_pixel_beam_leds_ x filter_size_<
   */
  Eigen::MatrixXf pixel_beam_buffer_;

  /*! This filter is of size >filter_size_ x fitler_size_<
   */
  Eigen::MatrixXf filter_;

  float max_distance_;
  float wave_travel_speed_;
  float wave_length_;

  float half_wave_activation_size_;

  float getFilter(const float normalized_distance, const unsigned int index);

  float wave_travel_distance_;
  float level_range_;
  MathUtilities::Profile profile_type_;

  Eigen::MatrixXf block_node_filter_buffer_;
  Eigen::MatrixXf block_beam_filter_buffer_;
  Eigen::MatrixXf pixel_beam_filter_buffer_;

  void processBlockNodes();
  void processBlockBeams();
  void processPixelBeams();

};

}

#endif /* DEC_LIGHT_PROCESSOR_H_ */
