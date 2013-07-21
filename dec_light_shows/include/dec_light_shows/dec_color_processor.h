/*
 * dec_color_processor.h
 *
 *  Created on: Jul 14, 2013
 *      Author: pastor
 */

#ifndef DEC_COLOR_PROCESSOR_H_
#define DEC_COLOR_PROCESSOR_H_

#include <vector>
#include <dec_light_show_manager/dec_light_show.h>

namespace dec_light_shows
{

class DecColorProcessor : public dec_light_show_manager::DecLightShow
{
public:
  DecColorProcessor() {};
  virtual ~DecColorProcessor() {};

  virtual bool initialize(XmlRpc::XmlRpcValue& config);
  virtual bool update();
  virtual bool start();
  virtual bool stop();

private:

//  static const unsigned int NUM_COLOR_VALUES = 4;
//
//  std::vector<float> low_level_color_;
//  std::vector<float> high_level_color_;
//  std::vector<float> colors_;
//  std::vector<float> color_multiplier_;

};

}

#endif /* DEC_COLOR_PROCESSOR_H_ */
