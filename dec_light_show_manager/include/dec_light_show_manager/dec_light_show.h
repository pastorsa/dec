/*
 * dec_light_show.h
 *
 *  Created on: Jul 14, 2013
 *      Author: pastor
 */

#ifndef DEC_LIGHT_SHOW_H_
#define DEC_LIGHT_SHOW_H_

#include <string>
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <dec_light_show_manager/dec_light_show_data.h>

namespace dec_light_show_manager
{

class DecLightShow
{

public:
  DecLightShow() :
    name_(""), id_(-1) {};
  virtual ~DecLightShow() {};

  /**
   * Run one control cycle of the light show
   * @return True on success, False on failure
   */
  virtual bool initialize(XmlRpc::XmlRpcValue& config) = 0;

  /**
   * Prepare this light show for running
   *
   * This is needed when switching in a new light show makes
   * use of current sensor/light state
   * @return True on success, False on failure
   */
  virtual bool start()=0;

  /**
   * Run one control cycle of the light show
   * @return True on success, False on failure
   */
  virtual bool update()=0;

  /**
   * Prepare to stop this light show
   * @return True on success, False on failure
   */
  virtual bool stop()=0;

  /**
   * Initialization of the base class
   */
  void initializeBase(const std::string& name, boost::shared_ptr<DecLightShowData> data, const int id);

  /**
   * Get the name of the light show
   * @return
   */
  std::string getName() const;

  /**
   * Get the light show id
   * @return
   */
  int getId() const;

protected:
  std::string name_;
  boost::shared_ptr<DecLightShowData> data_;
  int id_;
};

inline void DecLightShow::initializeBase(const std::string& name, boost::shared_ptr<DecLightShowData> data, const int id)
{
  name_ = name;
  data_ = data;
  id_ = id;
}

inline std::string DecLightShow::getName() const
{
  return name_;
}

inline int DecLightShow::getId() const
{
  return id_;
}

} // namespace

#endif /* DEC_LIGHT_SHOW_H_ */
