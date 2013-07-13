/*
 * dec_communication.h
 *
 *  Created on: Jun 16, 2013
 *      Author: pastor
 */

#ifndef DEC_VISUALIZATION_COMMUNICATION_H_
#define DEC_VISUALIZATION_COMMUNICATION_H_

#include <Eigen/Eigen>
#include <boost/shared_ptr.hpp>

#include <dec_visualization/dec_processor.h>

namespace dec_visualization
{

class DECCommunication : public DECProcessor
{

  friend class DECVisualization;
public:

  DECCommunication();
  virtual ~DECCommunication() {};

  /*!
   * @param node_handle
   * @return True on success, otherwise False
   */
  bool initialize(ros::NodeHandle node_handle);

  /*!
   * @return True on success, otherwise False
   */
  bool process();

private:

};

}

#endif /* DEC_VISUALIZATION_COMMUNICATION_H_ */
