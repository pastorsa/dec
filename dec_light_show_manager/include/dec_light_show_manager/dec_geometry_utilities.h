/*
 * dec_geometry_utilities.h
 *
 *  Created on: Jul 18, 2013
 *      Author: pastor
 */

#ifndef DEC_GEOMETRY_UTILITIES_H_
#define DEC_GEOMETRY_UTILITIES_H_

namespace dec_light_show_manager
{

geometry_msgs::Pose getBeamLedPose(std::pair<int, int> nodes,
                                   const int num_leds_per_meter,
                                   const int num_leds,
                                   const int led)
{
  float offset = ((float)1.0 - light_beams_size_.z) / (float)2.0;
  float led_fragment_length = light_beams_size_.z / (float)num_leds;
  float percent_distance_from_first_node = offset + (led_fragment_length/(float)2.0) + ((float)led * led_fragment_length);

  ROS_ASSERT(!(percent_distance_from_first_node < 0.0));
  ROS_ASSERT(!(percent_distance_from_first_node > 1.0));

  geometry_msgs::Pose led_pose;
  tf::Vector3 p1, p2;
  convert(node_positions_[light_beams_[beam_id].first], p1);
  convert(node_positions_[light_beams_[beam_id].second], p2);

  tf::Vector3 p1p2 = p2 - p1;
  tf::Vector3 center = (p1 + (percent_distance_from_first_node * p1p2));// / 2.0f;
  convert(center, led_pose.position);

  tf::Vector3 p12 = p2 - p1;
  p12.normalize();
  tf::Vector3 z_world = tf::Vector3(0.0, 0.0, 1.0);
  tf::Vector3 z_cross_p12 = z_world.cross(p12);
  double angle = acos(p12.dot(z_world));
  tf::Quaternion q;
  q.setRotation(z_cross_p12, angle);
  convert(q, led_pose.orientation);

  return led_pose;
}

}



#endif /* DEC_GEOMETRY_UTILITIES_H_ */
