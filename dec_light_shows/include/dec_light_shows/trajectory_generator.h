/*
 * trajectory_generator.h
 *
 *  Created on: Nov 22, 2010
 *      Author: kalakris
 */

#ifndef DEC_LIGHT_SHOWS_TRAJECTORY_GENERATOR_H_
#define DEC_LIGHT_SHOWS_TRAJECTORY_GENERATOR_H_

#include <vector>
#include <ros/ros.h>
#include <splines/splines.h>
#include <dec_msgs/Trajectory.h>
#include <dec_msgs/TrajectoryStatus.h>
#include <dec_light_show_manager/dec_light_show.h>

namespace dec_light_shows
{

struct TrajectoryPoint
{
  dec_msgs::TrajectoryPoint point_;  /**< The trajectory point */
  int id_;                                      /**< ID of the trajectory point */
  double percent_complete_;                     /**< Percentage completed of this trajectory id */
};

class TrajectoryGenerator: public dec_light_show_manager::DecLightShow
{
public:
  TrajectoryGenerator();
  virtual ~TrajectoryGenerator();

  /**
   * Initializes the trajectory generator
   * @param config
   * @param num_dimensions
   * @param dimension_names
   * @return
   */
  bool initialize(XmlRpc::XmlRpcValue& config, const std::vector<std::string>& dimension_names);

  /**
   * Poll for ROS message, and update our internal trajectory buffer
   * @return
   */
  bool processMessages(dec_msgs::TrajectoryConstPtr trajectory);

  /**
   * Advance the target, call this every control cycle. Updated target is available in target_.
   * @return
   */
  virtual bool updateTarget();

protected:
  double task_servo_dt_;
  std::vector<splines::QuinticSpline> cur_spline_;
  TrajectoryPoint target_; /**< The current target */

  virtual void updateCurSplineFromPoints(TrajectoryPoint& p1, TrajectoryPoint& p2);
  virtual void getCurrentTrajectoryPoint(TrajectoryPoint& point)=0;
  void holdCurrentPositions();

  int max_buffer_size_;
  int buffer_size_;
  int cur_target_index_;
  int num_dimensions_;
  double cur_trajectory_start_time_;

  double status_publishing_interval_;
  double last_status_publish_time_;

  std::vector<std::string> dimension_names_;

  std::vector<TrajectoryPoint> buffer_;

  dec_msgs::TrajectoryStatus trajectory_status_;

  ros::Subscriber trajectory_msg_subscriber_;
  ros::Publisher trajectory_status_publisher_;

  boost::shared_ptr<dec_msgs::Trajectory const> trajectory_msg_;

  void resetState();
  void publishStatus();
  bool checkMessageForNaN(dec_msgs::TrajectoryConstPtr trajectory);
};

}

#endif /* DEC_LIGHT_SHOWS_TRAJECTORY_GENERATOR_H_ */
