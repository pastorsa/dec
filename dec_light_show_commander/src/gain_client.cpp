/*
 * gain_client.cpp
 *
 *  Created on: Jun 7, 2011
 *      Author: kalakris
 */

#include <arm_controller_interface/gain_client.h>
#include <usc_utilities/param_server.h>
#include <usc_utilities/assert.h>

namespace arm_controller_interface
{

GainClient::GainClient(const int endeffector_id,
                       const std::string& controller_name)
{
  ROS_VERIFY(initialize(endeffector_id, controller_name));
}

bool GainClient::initialize(const int endeffector_id,
                            const std::string& controller_name)
{
  ROS_VERIFY(TrajectoryClient::initialize(controller_name));
  std::string prefix = ArmRobotInfo::getEndeffectorName(endeffector_id);

  arm_force_control_names_.push_back(prefix + "_FORCE_X_PGAIN");
  arm_force_control_names_.push_back(prefix + "_FORCE_Y_PGAIN");
  arm_force_control_names_.push_back(prefix + "_FORCE_Z_PGAIN");
  arm_force_control_names_.push_back(prefix + "_TORQUE_X_PGAIN");
  arm_force_control_names_.push_back(prefix + "_TORQUE_Y_PGAIN");
  arm_force_control_names_.push_back(prefix + "_TORQUE_Z_PGAIN");
  arm_force_control_names_.push_back(prefix + "_FORCE_X_IGAIN");
  arm_force_control_names_.push_back(prefix + "_FORCE_Y_IGAIN");
  arm_force_control_names_.push_back(prefix + "_FORCE_Z_IGAIN");
  arm_force_control_names_.push_back(prefix + "_TORQUE_X_IGAIN");
  arm_force_control_names_.push_back(prefix + "_TORQUE_Y_IGAIN");
  arm_force_control_names_.push_back(prefix + "_TORQUE_Z_IGAIN");

  arm_position_control_names_.push_back(prefix + "_POS_X_PGAIN");
  arm_position_control_names_.push_back(prefix + "_POS_Y_PGAIN");
  arm_position_control_names_.push_back(prefix + "_POS_Z_PGAIN");
  arm_position_control_names_.push_back(prefix + "_ORIENT_X_PGAIN");
  arm_position_control_names_.push_back(prefix + "_ORIENT_Y_PGAIN");
  arm_position_control_names_.push_back(prefix + "_ORIENT_Z_PGAIN");
  arm_position_control_names_.push_back(prefix + "_POS_X_IGAIN");
  arm_position_control_names_.push_back(prefix + "_POS_Y_IGAIN");
  arm_position_control_names_.push_back(prefix + "_POS_Z_IGAIN");
  arm_position_control_names_.push_back(prefix + "_ORIENT_X_IGAIN");
  arm_position_control_names_.push_back(prefix + "_ORIENT_Y_IGAIN");
  arm_position_control_names_.push_back(prefix + "_ORIENT_Z_IGAIN");
  arm_position_control_names_.push_back(prefix + "_POS_X_DGAIN");
  arm_position_control_names_.push_back(prefix + "_POS_Y_DGAIN");
  arm_position_control_names_.push_back(prefix + "_POS_Z_DGAIN");
  arm_position_control_names_.push_back(prefix + "_ORIENT_X_DGAIN");
  arm_position_control_names_.push_back(prefix + "_ORIENT_Y_DGAIN");
  arm_position_control_names_.push_back(prefix + "_ORIENT_Z_DGAIN");

  std::string short_prefix;
  if (ArmRobotInfo::isRightHand(endeffector_id))
  {
    short_prefix = "R";
  }
  else if(ArmRobotInfo::isLeftHand(endeffector_id))
  {
    short_prefix = "L";
  }
  else
  {
    ROS_ERROR("GainClient for %s: Invalid endeffector id >%i< provided.", controller_name.c_str(), endeffector_id);
    return false;
  }
  finger_force_control_names_.push_back(short_prefix + "_RF_FORCE_PGAIN");
  finger_force_control_names_.push_back(short_prefix + "_MF_FORCE_PGAIN");
  finger_force_control_names_.push_back(short_prefix + "_LF_FORCE_PGAIN");
  finger_force_control_names_.push_back(short_prefix + "_RF_FORCE_IGAIN");
  finger_force_control_names_.push_back(short_prefix + "_MF_FORCE_IGAIN");
  finger_force_control_names_.push_back(short_prefix + "_LF_FORCE_IGAIN");

  return true;
}

bool GainClient::setArmForceControlGains(const sl_controller_msgs::CartesianGains& arm_gains, double transition_duration, bool wait_for_success)
{
  sl_controller_msgs::Trajectory traj;
  traj.preempt = false;
  traj.dimension_names = arm_force_control_names_;
  int num_dim = traj.dimension_names.size();

  sl_controller_msgs::TrajectoryPoint point;
  point.time_from_start = ros::Duration(transition_duration);
  point.positions.push_back(arm_gains.pos_x.p_gain);
  point.positions.push_back(arm_gains.pos_y.p_gain);
  point.positions.push_back(arm_gains.pos_z.p_gain);
  point.positions.push_back(arm_gains.rot_x.p_gain);
  point.positions.push_back(arm_gains.rot_y.p_gain);
  point.positions.push_back(arm_gains.rot_z.p_gain);
  point.positions.push_back(arm_gains.pos_x.i_gain);
  point.positions.push_back(arm_gains.pos_y.i_gain);
  point.positions.push_back(arm_gains.pos_z.i_gain);
  point.positions.push_back(arm_gains.rot_x.i_gain);
  point.positions.push_back(arm_gains.rot_y.i_gain);
  point.positions.push_back(arm_gains.rot_z.i_gain);
  point.velocities.resize(num_dim, 0.0);
  point.accelerations.resize(num_dim, 0.0);
  traj.points.push_back(point);

  return sendCommand(traj, wait_for_success);
}

bool GainClient::setArmForceControlGains(const std::vector<double>& p_gains, const std::vector<double>& i_gains, double transition_duration, bool wait_for_success)
{
  ROS_ASSERT(p_gains.size() == 6);
  ROS_ASSERT(i_gains.size() == 6);

  sl_controller_msgs::Trajectory traj;
  traj.preempt = false;
  traj.dimension_names = arm_force_control_names_;
  int num_dim = traj.dimension_names.size();

  sl_controller_msgs::TrajectoryPoint point;
  point.time_from_start = ros::Duration(transition_duration);

  point.positions.push_back(p_gains[0]);
  point.positions.push_back(p_gains[1]);
  point.positions.push_back(p_gains[2]);
  point.positions.push_back(p_gains[3]);
  point.positions.push_back(p_gains[4]);
  point.positions.push_back(p_gains[5]);
  point.positions.push_back(i_gains[0]);
  point.positions.push_back(i_gains[1]);
  point.positions.push_back(i_gains[2]);
  point.positions.push_back(i_gains[3]);
  point.positions.push_back(i_gains[4]);
  point.positions.push_back(i_gains[5]);
  point.velocities.resize(num_dim, 0.0);
  point.accelerations.resize(num_dim, 0.0);
  traj.points.push_back(point);

  return sendCommand(traj, wait_for_success);
}

bool GainClient::setArmPositionControlGains(const sl_controller_msgs::CartesianGains& arm_gains, double transition_duration, bool wait_for_success)
{
  sl_controller_msgs::Trajectory traj;
  traj.preempt = false;
  traj.dimension_names = arm_position_control_names_;
  int num_dim = traj.dimension_names.size();

  sl_controller_msgs::TrajectoryPoint point;
  point.time_from_start = ros::Duration(transition_duration);

  point.positions.push_back(arm_gains.pos_x.p_gain);
  point.positions.push_back(arm_gains.pos_y.p_gain);
  point.positions.push_back(arm_gains.pos_z.p_gain);
  point.positions.push_back(arm_gains.rot_x.p_gain);
  point.positions.push_back(arm_gains.rot_y.p_gain);
  point.positions.push_back(arm_gains.rot_z.p_gain);
  point.positions.push_back(arm_gains.pos_x.i_gain);
  point.positions.push_back(arm_gains.pos_y.i_gain);
  point.positions.push_back(arm_gains.pos_z.i_gain);
  point.positions.push_back(arm_gains.rot_x.i_gain);
  point.positions.push_back(arm_gains.rot_y.i_gain);
  point.positions.push_back(arm_gains.rot_z.i_gain);
  point.positions.push_back(arm_gains.pos_x.d_gain);
  point.positions.push_back(arm_gains.pos_y.d_gain);
  point.positions.push_back(arm_gains.pos_z.d_gain);
  point.positions.push_back(arm_gains.rot_x.d_gain);
  point.positions.push_back(arm_gains.rot_y.d_gain);
  point.positions.push_back(arm_gains.rot_z.d_gain);

  point.velocities.resize(num_dim, 0.0);
  point.accelerations.resize(num_dim, 0.0);
  traj.points.push_back(point);

  return sendCommand(traj, wait_for_success);
}

bool GainClient::setArmPositionControlGains(const std::vector<double>& p_gains, const std::vector<double>& i_gains, double transition_duration, bool wait_for_success)
{
  ROS_ASSERT(p_gains.size() == 6);
  ROS_ASSERT(i_gains.size() == 6);

  sl_controller_msgs::Trajectory traj;
  traj.preempt = false;
  traj.dimension_names = arm_position_control_names_;
  int num_dim = traj.dimension_names.size();

  sl_controller_msgs::TrajectoryPoint point;
  point.time_from_start = ros::Duration(transition_duration);

  point.positions.push_back(p_gains[0]);
  point.positions.push_back(p_gains[1]);
  point.positions.push_back(p_gains[2]);
  point.positions.push_back(p_gains[3]);
  point.positions.push_back(p_gains[4]);
  point.positions.push_back(p_gains[5]);
  point.positions.push_back(i_gains[0]);
  point.positions.push_back(i_gains[1]);
  point.positions.push_back(i_gains[2]);
  point.positions.push_back(i_gains[3]);
  point.positions.push_back(i_gains[4]);
  point.positions.push_back(i_gains[5]);
  // d gains = p gains for task position controller
  point.positions.push_back(p_gains[0]);
  point.positions.push_back(p_gains[1]);
  point.positions.push_back(p_gains[2]);
  point.positions.push_back(p_gains[3]);
  point.positions.push_back(p_gains[4]);
  point.positions.push_back(p_gains[5]);
  point.velocities.resize(num_dim, 0.0);
  point.accelerations.resize(num_dim, 0.0);
  traj.points.push_back(point);

  return sendCommand(traj, wait_for_success);
}

bool GainClient::setArmPositionForceControlGains(const sl_controller_msgs::CartesianGains& arm_position_gains,
                                                 const sl_controller_msgs::CartesianGains& arm_force_gains,
                                                 double transition_duration, bool wait_for_success)
{
  sl_controller_msgs::Trajectory traj;
  traj.preempt = false;
  traj.dimension_names = arm_position_control_names_;
  traj.dimension_names.insert(traj.dimension_names.end(), arm_force_control_names_.begin(), arm_force_control_names_.end());
  int num_dim = traj.dimension_names.size();

  sl_controller_msgs::TrajectoryPoint point;
  point.time_from_start = ros::Duration(transition_duration);

  point.positions.push_back(arm_position_gains.pos_x.p_gain);
  point.positions.push_back(arm_position_gains.pos_y.p_gain);
  point.positions.push_back(arm_position_gains.pos_z.p_gain);
  point.positions.push_back(arm_position_gains.rot_x.p_gain);
  point.positions.push_back(arm_position_gains.rot_y.p_gain);
  point.positions.push_back(arm_position_gains.rot_z.p_gain);
  point.positions.push_back(arm_position_gains.pos_x.i_gain);
  point.positions.push_back(arm_position_gains.pos_y.i_gain);
  point.positions.push_back(arm_position_gains.pos_z.i_gain);
  point.positions.push_back(arm_position_gains.rot_x.i_gain);
  point.positions.push_back(arm_position_gains.rot_y.i_gain);
  point.positions.push_back(arm_position_gains.rot_z.i_gain);
  point.positions.push_back(arm_position_gains.pos_x.d_gain);
  point.positions.push_back(arm_position_gains.pos_y.d_gain);
  point.positions.push_back(arm_position_gains.pos_z.d_gain);
  point.positions.push_back(arm_position_gains.rot_x.d_gain);
  point.positions.push_back(arm_position_gains.rot_y.d_gain);
  point.positions.push_back(arm_position_gains.rot_z.d_gain);
  point.positions.push_back(arm_force_gains.pos_x.p_gain);
  point.positions.push_back(arm_force_gains.pos_y.p_gain);
  point.positions.push_back(arm_force_gains.pos_z.p_gain);
  point.positions.push_back(arm_force_gains.rot_x.p_gain);
  point.positions.push_back(arm_force_gains.rot_y.p_gain);
  point.positions.push_back(arm_force_gains.rot_z.p_gain);
  point.positions.push_back(arm_force_gains.pos_x.i_gain);
  point.positions.push_back(arm_force_gains.pos_y.i_gain);
  point.positions.push_back(arm_force_gains.pos_z.i_gain);
  point.positions.push_back(arm_force_gains.rot_x.i_gain);
  point.positions.push_back(arm_force_gains.rot_y.i_gain);
  point.positions.push_back(arm_force_gains.rot_z.i_gain);

  point.velocities.resize(num_dim, 0.0);
  point.accelerations.resize(num_dim, 0.0);
  traj.points.push_back(point);

  return sendCommand(traj, wait_for_success);
}

bool GainClient::setArmPositionForceControlGains(const std::vector<double>& position_p_gains, std::vector<double>& position_i_gains,
                                                 const std::vector<double>& force_p_gains, std::vector<double>& force_i_gains,
                                                 double transition_duration, bool wait_for_success)
{
  ROS_ASSERT(position_p_gains.size() == 6);
  ROS_ASSERT(position_i_gains.size() == 6);
  ROS_ASSERT(force_p_gains.size() == 6);
  ROS_ASSERT(force_i_gains.size() == 6);

  sl_controller_msgs::Trajectory traj;
  traj.preempt = false;
  traj.dimension_names = arm_position_control_names_;
  traj.dimension_names.insert(traj.dimension_names.end(), arm_force_control_names_.begin(), arm_force_control_names_.end());
  int num_dim = traj.dimension_names.size();

  sl_controller_msgs::TrajectoryPoint point;
  point.time_from_start = ros::Duration(transition_duration);

  point.positions.push_back(position_p_gains[0]);
  point.positions.push_back(position_p_gains[1]);
  point.positions.push_back(position_p_gains[2]);
  point.positions.push_back(position_p_gains[3]);
  point.positions.push_back(position_p_gains[4]);
  point.positions.push_back(position_p_gains[5]);
  point.positions.push_back(position_i_gains[0]);
  point.positions.push_back(position_i_gains[1]);
  point.positions.push_back(position_i_gains[2]);
  point.positions.push_back(position_i_gains[3]);
  point.positions.push_back(position_i_gains[4]);
  point.positions.push_back(position_i_gains[5]);
  // d gains = p gains for task position controller
  point.positions.push_back(position_p_gains[0]);
  point.positions.push_back(position_p_gains[1]);
  point.positions.push_back(position_p_gains[2]);
  point.positions.push_back(position_p_gains[3]);
  point.positions.push_back(position_p_gains[4]);
  point.positions.push_back(position_p_gains[5]);
  point.positions.push_back(force_p_gains[0]);
  point.positions.push_back(force_p_gains[1]);
  point.positions.push_back(force_p_gains[2]);
  point.positions.push_back(force_p_gains[3]);
  point.positions.push_back(force_p_gains[4]);
  point.positions.push_back(force_p_gains[5]);
  point.positions.push_back(force_i_gains[0]);
  point.positions.push_back(force_i_gains[1]);
  point.positions.push_back(force_i_gains[2]);
  point.positions.push_back(force_i_gains[3]);
  point.positions.push_back(force_i_gains[4]);
  point.positions.push_back(force_i_gains[5]);

  point.velocities.resize(num_dim, 0.0);
  point.accelerations.resize(num_dim, 0.0);
  traj.points.push_back(point);

  return sendCommand(traj, wait_for_success);
}

bool GainClient::setFingerForceControlGains(const arm_controller_msgs::FingerForceControlGains& finger_gains, double transition_duration, bool wait_for_success)
{
  sl_controller_msgs::Trajectory traj;
  traj.preempt = false;
  traj.dimension_names = finger_force_control_names_;
  int num_dim = traj.dimension_names.size();

  sl_controller_msgs::TrajectoryPoint point;
  point.time_from_start = ros::Duration(transition_duration);

  point.positions.push_back(finger_gains.p_gains[0]);
  point.positions.push_back(finger_gains.p_gains[1]);
  point.positions.push_back(finger_gains.p_gains[2]);
  point.positions.push_back(finger_gains.i_gains[0]);
  point.positions.push_back(finger_gains.i_gains[1]);
  point.positions.push_back(finger_gains.i_gains[2]);
  point.velocities.resize(num_dim, 0.0);
  point.accelerations.resize(num_dim, 0.0);
  traj.points.push_back(point);

  return sendCommand(traj, wait_for_success);
}

bool GainClient::disableForceControlGains(double transition_duration, bool wait_for_success)
{
  sl_controller_msgs::CartesianGains force_gains;
  setZero(force_gains);
  return setArmForceControlGains(force_gains, transition_duration, wait_for_success);
}

bool GainClient::disablePositionControlGains(double transition_duration, bool wait_for_success)
{
  sl_controller_msgs::CartesianGains position_gains;
  setZero(position_gains);
  return setArmPositionControlGains(position_gains, transition_duration, wait_for_success);
}

bool GainClient::disablePositionForceControlGains(double transition_duration, bool wait_for_success)
{
  sl_controller_msgs::CartesianGains position_gains;
  setZero(position_gains);
  sl_controller_msgs::CartesianGains force_gains;
  setZero(force_gains);
  return setArmPositionForceControlGains(position_gains, force_gains, transition_duration, wait_for_success);
}

bool GainClient::disableFingerForceControlGains(double transition_duration, bool wait_for_success)
{
  arm_controller_msgs::FingerForceControlGains finger_gains;
  setZero(finger_gains);
  return setFingerForceControlGains(finger_gains, transition_duration, wait_for_success);
}

bool GainClient::enableForceControlGains(double transition_duration, bool wait_for_success)
{
  sl_controller_msgs::CartesianGains force_gains;
  setOne(force_gains);
  return setArmForceControlGains(force_gains, transition_duration, wait_for_success);
}

bool GainClient::enablePositionControlGains(double transition_duration, bool wait_for_success)
{
  sl_controller_msgs::CartesianGains position_gains;
  setOne(position_gains);
  return setArmPositionControlGains(position_gains, transition_duration, wait_for_success);
}

bool GainClient::enablePositionForceControlGains(double transition_duration, bool wait_for_success)
{
  sl_controller_msgs::CartesianGains position_gains;
  setOne(position_gains);
  sl_controller_msgs::CartesianGains force_gains;
  setOne(force_gains);
  return setArmPositionForceControlGains(position_gains, force_gains, transition_duration, wait_for_success);
}

bool GainClient::enableFingerForceControlGains(double transition_duration, bool wait_for_success)
{
  arm_controller_msgs::FingerForceControlGains finger_gains;
  setOne(finger_gains);
  return setFingerForceControlGains(finger_gains, transition_duration, wait_for_success);
}

bool GainClient::setArmPositionForceControlGains(const std::string& gain_string, double transition_duration,
                                     bool wait_for_success)
{
  if (gain_string.length() != 6)
  {
    ROS_ERROR("gain_string must be of length 6");
    return false;
  }
  std::vector<double> position_gains(6);
  std::vector<double> force_gains(6);
  for (int i=0; i<6; ++i)
  {
    setGainElement(gain_string[i], position_gains[i], force_gains[i]);
  }
  return setArmPositionForceControlGains(position_gains, position_gains,
                                         force_gains, force_gains,
                                         transition_duration, wait_for_success);
}

bool GainClient::setGainElement(char gain_char, double& position_gain, double& force_gain)
{
  if (gain_char == 'f')
  {
    force_gain = 1.0;
    position_gain = 0.0;
  }
  else if (gain_char == 'p')
  {
    force_gain = 0.0;
    position_gain = 1.0;
  }
  else if (gain_char == '0')
  {
    force_gain = 0.0;
    position_gain = 0.0;
  }
  else
    return false;
  return true;
}

bool GainClient::readGains(XmlRpc::XmlRpcValue& config,
                           sl_controller_msgs::CartesianGains& position_gains,
                           sl_controller_msgs::CartesianGains& force_gains)
{
  if (config.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("Gains must be specified as an array");
    return false;
  }

  if (config.size() != 6)
  {
    ROS_ERROR("Gains must be be an array of six characters");
    return false;
  }

  char v[6];
  for (int i=0; i<6; ++i)
  {
    if (config[i].getType() != XmlRpc::XmlRpcValue::TypeString)
    {
      ROS_ERROR("Gains must be be an array of six characters");
      return false;
    }
    std::string s = config[i];
    v[i] = s.at(0);
    if (v[i] != 'f' && v[i] !='p' && v[i]!='0')
    {
      ROS_ERROR("Gains must be either 'p', 'f', or '0'");
      return false;
    }
  }

  setGainElement(v[0], position_gains.pos_x.p_gain, force_gains.pos_x.p_gain);
  setGainElement(v[1], position_gains.pos_y.p_gain, force_gains.pos_y.p_gain);
  setGainElement(v[2], position_gains.pos_z.p_gain, force_gains.pos_z.p_gain);
  setGainElement(v[3], position_gains.rot_x.p_gain, force_gains.rot_x.p_gain);
  setGainElement(v[4], position_gains.rot_y.p_gain, force_gains.rot_y.p_gain);
  setGainElement(v[5], position_gains.rot_z.p_gain, force_gains.rot_z.p_gain);

  setIGainsFromPGains(position_gains);
  setDGainsFromPGains(position_gains);
  setIGainsFromPGains(force_gains);
  setDGainsFromPGains(force_gains);
  return true;
}

void GainClient::setIGainsFromPGains(sl_controller_msgs::CartesianGains& gains)
{
  gains.pos_x.i_gain = gains.pos_x.p_gain;
  gains.pos_y.i_gain = gains.pos_y.p_gain;
  gains.pos_z.i_gain = gains.pos_z.p_gain;
  gains.rot_x.i_gain = gains.rot_x.p_gain;
  gains.rot_y.i_gain = gains.rot_y.p_gain;
  gains.rot_z.i_gain = gains.rot_z.p_gain;
}

void GainClient::setDGainsFromPGains(sl_controller_msgs::CartesianGains& gains)
{
  gains.pos_x.d_gain = gains.pos_x.p_gain;
  gains.pos_y.d_gain = gains.pos_y.p_gain;
  gains.pos_z.d_gain = gains.pos_z.p_gain;
  gains.rot_x.d_gain = gains.rot_x.p_gain;
  gains.rot_y.d_gain = gains.rot_y.p_gain;
  gains.rot_z.d_gain = gains.rot_z.p_gain;
}

void GainClient::set(sl_controller_msgs::CartesianGains& gains, const double& value)
{
  gains.pos_x.p_gain = value;
  gains.pos_y.p_gain = value;
  gains.pos_z.p_gain = value;
  gains.rot_x.p_gain = value;
  gains.rot_y.p_gain = value;
  gains.rot_z.p_gain = value;
  gains.pos_x.i_gain = value;
  gains.pos_y.i_gain = value;
  gains.pos_z.i_gain = value;
  gains.rot_x.i_gain = value;
  gains.rot_y.i_gain = value;
  gains.rot_z.i_gain = value;
  gains.pos_x.d_gain = value;
  gains.pos_y.d_gain = value;
  gains.pos_z.d_gain = value;
  gains.rot_x.d_gain = value;
  gains.rot_y.d_gain = value;
  gains.rot_z.d_gain = value;
}

void GainClient::setZero(sl_controller_msgs::CartesianGains& gains)
{
  set(gains, 0.0);
}

void GainClient::setOne(sl_controller_msgs::CartesianGains& gains)
{
  set(gains, 1.0);
}

void GainClient::set(arm_controller_msgs::FingerForceControlGains& gains, const double& value)
{
  gains.p_gains[arm_controller_msgs::FingerForceControlGains::R_RF] = value;
  gains.p_gains[arm_controller_msgs::FingerForceControlGains::R_MF] = value;
  gains.p_gains[arm_controller_msgs::FingerForceControlGains::R_LF] = value;
  gains.i_gains[arm_controller_msgs::FingerForceControlGains::R_RF] = value;
  gains.i_gains[arm_controller_msgs::FingerForceControlGains::R_MF] = value;
  gains.i_gains[arm_controller_msgs::FingerForceControlGains::R_LF] = value;
}

void GainClient::setZero(arm_controller_msgs::FingerForceControlGains& gains)
{
  set(gains, 0.0);
}

void GainClient::setOne(arm_controller_msgs::FingerForceControlGains& gains)
{
  set(gains, 1.0);
}

}
