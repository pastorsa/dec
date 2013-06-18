/*
 * rviz_publisher.cpp
 *
 *  Created on: Jan 7, 2011
 *      Author: kalakris
 */

#include <dec_utilities/rviz_publisher.h>

namespace dec_utilities
{

RvizPublisher::RvizPublisher(const std::string& topic_name)
{
  pub_ = node_handle_.advertise<visualization_msgs::Marker>(topic_name, 100, true);
}

RvizPublisher::~RvizPublisher()
{
}

void RvizPublisher::publish(const visualization_msgs::Marker& marker)
{
  pub_.publish(marker);
}


}
