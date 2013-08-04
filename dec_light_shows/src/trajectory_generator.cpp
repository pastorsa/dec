/*
 * trajectory_generator.cpp
 *
 *  Created on: Nov 22, 2010
 *      Author: kalakris
 */

#include <dec_light_shows/trajectory_generator.h>
#include <dec_light_show_manager/dec_light_show_utilities.h>

using namespace dec_light_show_manager;

namespace dec_light_shows
{

TrajectoryGenerator::TrajectoryGenerator()
: task_servo_dt_(0.0), max_buffer_size_(0), buffer_size_(0), cur_target_index_(0), num_dimensions_(0), cur_trajectory_start_time_(0.0),
  status_publishing_interval_(0.0), status_publishing_interval_(0.0), last_status_publish_time_(0.0)
{
}

TrajectoryGenerator::~TrajectoryGenerator()
{
}

bool TrajectoryGenerator::initialize(XmlRpc::XmlRpcValue& config, const std::vector<std::string>& dimension_names)
{
  // read parameters:
  DecLightShowUtilities::param(config, "max_buffer_size", max_buffer_size_, 1000);
  DecLightShowUtilities::param(config, "status_publishing_interval", status_publishing_interval_, 0.01);
  num_dimensions_ = dimension_names.size();
  dimension_names_ = dimension_names;

  // initialize the buffer
  TrajectoryPoint default_element;
  default_element.point_.positions.resize(num_dimensions_);
  default_element.point_.velocities.resize(num_dimensions_);
  default_element.point_.accelerations.resize(num_dimensions_);
  buffer_.resize(max_buffer_size_, default_element);
  target_ = default_element;

  // allocate other memory
  cur_spline_.resize(num_dimensions_, splines::QuinticSpline());

  resetState();

  const int PUBLISHER_BUFFER_SIZE = 10;
  trajectory_status_publisher_ = data_->node_handle_.advertise<dec_msgs::TrajectoryStatus>("status", PUBLISHER_BUFFER_SIZE, true);

  const int SUBSCRIBER_BUFFER_SIZE = 10;
  // initialize the subscribers (do this last):
  trajectory_msg_subscriber_ = data_->node_handle_.subscribe("command", SUBSCRIBER_BUFFER_SIZE, &TrajectoryGenerator::processMessages, this);

  return true;
}

void TrajectoryGenerator::resetState()
{
  buffer_size_ = 0;
  cur_target_index_ = -1;
  trajectory_status_.id = -1;
}

void TrajectoryGenerator::holdCurrentPositions()
{
  for (int i=0; i<num_dimensions_; ++i)
  {
    target_.point_.velocities[i] = 0.0;
    target_.point_.accelerations[i] = 0.0;
  }
}

bool TrajectoryGenerator::processMessages(dec_msgs::TrajectoryConstPtr trajectory)
{

  if (trajectory->preempt)
  {
    if (trajectory_status_.id > 0)
    {
      trajectory_status_.status = trajectory_status_.PREEMPTED;
      trajectory_status_.end_time = data_->ros_time_;
      publishStatus();
      holdCurrentPositions();
    }
    else
    {
      ROS_WARN("%s Received preemption request when no trajectory is active!", name_.c_str());
    }
    resetState();
    return true;
  }

  const std::vector<dec_msgs::TrajectoryPoint>& points = trajectory->points;

  // correct the order of joints:
  int local_to_msg_joint_map[num_dimensions_];
  for (int i=0; i<num_dimensions_; ++i)
  {
    local_to_msg_joint_map[i]=-1;
  }
  for (unsigned int i=0; i<trajectory->dimension_names.size(); ++i)
  {
    // search in the local dimension name list
    int local_id = -1;
    for (unsigned int j=0; j<dimension_names_.size(); ++j)
    {
      if (trajectory->dimension_names[i] == dimension_names_[j])
      {
        local_id = j;
        break;
      }
    }
    if (local_id != -1)
    {
      local_to_msg_joint_map[local_id] = i;
    }
    else
    {
      ROS_WARN("%s: joint name %s is not handled by me", name_.c_str(), trajectory->dimension_names[i].c_str());
      //return false;
    }
  }

  // check whether we have enough space
  if (int(points.size()) + buffer_size_ >= max_buffer_size_)
  {
    ROS_ERROR("%s: buffer of size %d full!", name_.c_str(), max_buffer_size_);
    return false;
  }

  int msg_dimensions = trajectory->dimension_names.size();

  // ensure that all positions, velocities and accelerations exist:
  for (unsigned int i=0; i<points.size(); ++i)
  {
    if ((int)points[i].positions.size() != msg_dimensions
        || (int)points[i].velocities.size() != msg_dimensions
        || (int)points[i].accelerations.size() != msg_dimensions)
    {
      ROS_ERROR("%s: positions, velocities and accelerations must be specified for each dimension", name_.c_str());
      return false;
    }
  }

  if (!checkMessageForNaN(trajectory))
    return false;

  // get the total trajectory time
  double total_time = points[points.size()-1].time_from_start.toSec();
  if (total_time < data_->control_dt_)
    total_time = data_->control_dt_;

  // figure out a time offset to add to the incoming points:
  double time_offset=0.0;
  if (cur_target_index_==-1)
  {
    cur_trajectory_start_time_ = data_->ros_time_sec_ - task_servo_dt_;
    //cur_trajectory_start_ros_time_ = data_->ros_time_ - ros::Duration(task_servo_dt_);
    //cur_segment_start_time_ = task_servo_time - task_servo_dt_;

    // add a dummy start point with the current position
    getCurrentTrajectoryPoint(buffer_[0]);
    //slToRosCState(data_->cart_des_state_[sl_endeff_id_], data_->cart_des_orient_[sl_endeff_id_], buffer_[0]);
    buffer_[0].point_.time_from_start = ros::Duration(0.0);
    buffer_[0].id_ = -1;
    buffer_[0].percent_complete_=0.0;
    buffer_size_++;
    cur_target_index_=0;
  }
  else
  {
    time_offset = buffer_[buffer_size_-1].point_.time_from_start.toSec();
  }

  // copy the points into our buffer
  for (unsigned int i=0; i<points.size(); ++i)
  {
    const dec_msgs::TrajectoryPoint& in_point = points[i];
    dec_msgs::TrajectoryPoint& out_point = buffer_[buffer_size_].point_;

    // if the time difference from the previous point is less than dt, skip this point:
    ros::Duration new_point_time = ros::Duration(time_offset) + in_point.time_from_start;
    if (new_point_time - buffer_[buffer_size_-1].point_.time_from_start < ros::Duration(task_servo_dt_))
    {
      new_point_time = buffer_[buffer_size_-1].point_.time_from_start + ros::Duration(task_servo_dt_);

    // if (in_point.time_from_start.toSec()>1e-6)
    // {
    //   ROS_WARN("%s: Time difference between trajectory points was too small, skipping points!", name_.c_str());
    // }
    // continue;
    }

    // set the id
    buffer_[buffer_size_].id_ = trajectory->id;
    buffer_[buffer_size_].percent_complete_ = 100.0 * ((new_point_time.toSec() - time_offset) / total_time);

    out_point.time_from_start = new_point_time;

    for (int local_id=0; local_id<num_dimensions_; ++local_id)
    {
      int msg_id = local_to_msg_joint_map[local_id];
      if (msg_id == -1)
      {
        // this joint doesn't exist in the msg: hold prev position
        out_point.positions[local_id] = buffer_[buffer_size_-1].point_.positions[local_id];
        out_point.velocities[local_id] = buffer_[buffer_size_-1].point_.velocities[local_id];
        out_point.accelerations[local_id] = buffer_[buffer_size_-1].point_.velocities[local_id];
      }
      else
      {
        // copy from the message
        out_point.positions[local_id] = in_point.positions[msg_id];
        out_point.velocities[local_id] = in_point.velocities[msg_id];
        out_point.accelerations[local_id] = in_point.accelerations[msg_id];
      }
    }

    ++buffer_size_;
  }

  return true;
}

bool TrajectoryGenerator::updateTarget()
{
  // if we have a target
  if (cur_target_index_ >= 0)
  {
    // check if we have already crossed our target:
    double cur_time = data_->ros_time_sec_ - cur_trajectory_start_time_;
    if (cur_time > buffer_[cur_target_index_].point_.time_from_start.toSec() + 1e-6)
    {
      // publish status
      if (buffer_[cur_target_index_].id_ != -1)
      {
        trajectory_status_.id = buffer_[cur_target_index_].id_;
        trajectory_status_.percent_complete = buffer_[cur_target_index_].percent_complete_;
        if (trajectory_status_.percent_complete > 99.9999) // kind of hacky
        {
          trajectory_status_.status = trajectory_status_.FINISHED;
          trajectory_status_.end_time = data_->ros_time_;
        }
        else
          trajectory_status_.status = trajectory_status_.IN_PROGRESS;
        publishStatus();
      }

      if (cur_target_index_==buffer_size_-1)
      {
        // we're out of trajectory to execute
        holdCurrentPositions();
        resetState();
      }
      else
      {
        cur_target_index_++;
        // construct the current spline from the two waypoints
        updateCurSplineFromPoints(buffer_[cur_target_index_-1],
                                  buffer_[cur_target_index_]);

        // publish status if we're starting a new trajectory
        if (buffer_[cur_target_index_].id_ != trajectory_status_.id)
        {
          trajectory_status_.id = buffer_[cur_target_index_].id_;
          trajectory_status_.percent_complete = 0.0; // we're just starting this trajectory
          trajectory_status_.status = trajectory_status_.IN_PROGRESS;
          trajectory_status_.start_time = data_->ros_time_;
          publishStatus();
        }

      }
    }

    // if we haven't blown past the end..
    if (cur_target_index_ >=0)
    {
      double time_in_cur_segment = cur_time - buffer_[cur_target_index_-1].point_.time_from_start.toSec();

      // sample each spline
      double x, xd, xdd;

      for (int local_joint=0; local_joint<num_dimensions_; ++local_joint)
      {
        cur_spline_[local_joint].sample(time_in_cur_segment, x, xd, xdd);
        target_.point_.positions[local_joint] = x;
        target_.point_.velocities[local_joint] = xd;
        target_.point_.accelerations[local_joint] = xdd;
      }
    }
  }

  // if there is nothing in the buffer, hold position, and zero velocities and accelerations
  if (cur_target_index_==-1 && buffer_size_==0)
  {
    trajectory_status_.id = -1;
    holdCurrentPositions();
  }
  return true;
}

void TrajectoryGenerator::updateCurSplineFromPoints(TrajectoryPoint& p1, TrajectoryPoint& p2)
{
  double time = (p2.point_.time_from_start - p1.point_.time_from_start).toSec();

  for (int i=0; i<num_dimensions_; ++i)
  {
    cur_spline_[i].setCoefficients(
        p1.point_.positions[i],
        p1.point_.velocities[i],
        p1.point_.accelerations[i],
        p2.point_.positions[i],
        p2.point_.velocities[i],
        p2.point_.accelerations[i],
        time);
  }
}

void TrajectoryGenerator::publishStatus()
{
//  boost::shared_ptr<dec_msgs::TrajectoryStatus> message = trajectory_status_publisher_.allocate();
//  if (message)
//  {
//    *message = trajectory_status_;
//    trajectory_status_publisher_.publish(message);
//    last_status_publish_time_ = task_servo_time;
//  }
  trajectory_status_publisher_.publish(trajectory_status_);
  last_status_publish_time_ = data_->ros_time_sec_;
}

bool TrajectoryGenerator::checkMessageForNaN(dec_msgs::TrajectoryConstPtr trajectory)
{
  for (unsigned int i=0; i<trajectory->points.size(); ++i)
  {
    for (unsigned int j=0; j<trajectory->points[i].positions.size(); ++j)
    {
      if (!std::isfinite(trajectory->points[i].positions[j]) ||
          !std::isfinite(trajectory->points[i].velocities[j]) ||
          !std::isfinite(trajectory->points[i].accelerations[j]))
      {
        ROS_ERROR("%s: Nan found!", name_.c_str());
        ROS_ERROR_STREAM(*trajectory);
        return false;
      }
    }
  }
  return true;
}

}
