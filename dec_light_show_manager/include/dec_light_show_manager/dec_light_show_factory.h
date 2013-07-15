/*
 * dec_light_show_factory.h
 *
 *  Created on: Jul 14, 2013
 *      Author: pastor
 */

#ifndef DEC_LIGHT_SHOW_FACTORY_H_
#define DEC_LIGHT_SHOW_FACTORY_H_

#include <string>
#include <boost/shared_ptr.hpp>
#include <pluginlib/class_loader.h>
#include <dec_light_show_manager/dec_light_show.h>
#include <dec_light_show_manager/dec_light_show_data.h>

namespace dec_light_show_manager
{

class DecLightShowFactory
{

public:
  DecLightShowFactory();
  virtual ~DecLightShowFactory();

  /*!
   * @param class_name
   * @param name
   * @param id
   * @param data
   * @param light_show
   * @return True on success, otherwise False
   */
  bool createLightShowByName(const std::string& class_name,
                             const std::string& name,
                             const int id,
                             boost::shared_ptr<DecLightShowData> data,
                             boost::shared_ptr<DecLightShow>& light_show);

  /*!
   */
  void reset();

private:
  boost::shared_ptr<pluginlib::ClassLoader<DecLightShow> > light_show_loader_;

};

}


#endif /* DEC_LIGHT_SHOW_FACTORY_H_ */
