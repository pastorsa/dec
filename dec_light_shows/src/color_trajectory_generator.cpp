/*
 * color_trajectory_controller.cpp
 *
 *  Created on: Nov 14, 2010
 *      Author: kalakris
 */

#include <dec_light_shows/color_trajectory_generator.h>
#include <dec_light_show_manager/dec_light_show_utilities.h>

PLUGINLIB_DECLARE_CLASS(dec_light_shows, ColorTrajectoryGenerator, dec_light_shows::ColorTrajectoryGenerator, dec_light_show_manager::DecLightShow)

using namespace dec_light_show_manager;
using namespace dec_utilities;

namespace dec_light_shows
{

ColorTrajectoryGenerator::ColorTrajectoryGenerator()
{
}

ColorTrajectoryGenerator::~ColorTrajectoryGenerator()
{
}

bool ColorTrajectoryGenerator::readEndeffNamesFromConfig(XmlRpc::XmlRpcValue& config,
                               const std::string& param_name,
                               std::vector<std::string>& dimension_names,
                               bool& param_exists,
                               std::vector<int>& endeff_indices,
                               const std::vector<std::string>& variable_suffixes)
{
  if (config.hasMember(param_name))
  {
    std::vector<std::string> control_cartesian_names;
    if (!DecLightShowUtilities::getParam(config, param_name, control_cartesian_names))
    {
      ROS_ERROR("%s: Error reading names from %s.", name_.c_str(), param_name.c_str());
      return false;
    }
    for (unsigned int i=0; i<control_cartesian_names.size(); ++i)
    {
      int ind = DecLightShowUtilities::getEndeffIndex(control_cartesian_names[i]);
      if (ind <= 0)
      {
        ROS_ERROR("%s: Endeff %s not found!", name_.c_str(), control_cartesian_names[i].c_str());
        return false;
      }
      endeff_indices.push_back(ind);
      for (unsigned int j=0; j<variable_suffixes.size(); ++j)
      {
        dimension_names.push_back(control_cartesian_names[i]+variable_suffixes[j]);
      }
    }
    param_exists = true;
  }
  return true;
}

bool ColorTrajectoryGenerator::readJointNamesFromConfig(XmlRpc::XmlRpcValue& config,
                               const std::string& param_name,
                               std::vector<std::string>& dimension_names,
                               bool& param_exists,
                               std::vector<int>& joint_indices,
                               const std::vector<std::string>& variable_suffixes)
{
  if (config.hasMember(param_name))
  {
    std::vector<std::string> control_joint_names;
    if (!DecLightShowUtilities::getParam(config, param_name, control_joint_names))
    {
      ROS_ERROR("%s: Error reading names from %s.", name_.c_str(), param_name.c_str());
      return false;
    }
    for (unsigned int i=0; i<control_joint_names.size(); ++i)
    {
      int ind = DecLightShowUtilities::getJointIndex(control_joint_names[i]);
      if (ind <= 0)
      {
        ROS_ERROR("%s: Joint %s not found!", name_.c_str(), control_joint_names[i].c_str());
        return false;
      }
      joint_indices.push_back(ind);
      for (unsigned int j=0; j<variable_suffixes.size(); ++j)
      {
        dimension_names.push_back(control_joint_names[i]+variable_suffixes[j]);
      }
    }
    param_exists = true;
  }
  return true;
}

bool ColorTrajectoryGenerator::initialize(XmlRpc::XmlRpcValue& config)
{
  //sl_endeff_id_ = RIGHT_HAND;

  std::vector<std::string> dim_names;
  //num_dimensions_ = 0;

  std::vector<std::string> cart_var_suffixes;
  cart_var_suffixes.push_back("_POS_X");
  cart_var_suffixes.push_back("_POS_Y");
  cart_var_suffixes.push_back("_POS_Z");
  cart_var_suffixes.push_back("_ORIENT_QW");
  cart_var_suffixes.push_back("_ORIENT_QX");
  cart_var_suffixes.push_back("_ORIENT_QY");
  cart_var_suffixes.push_back("_ORIENT_QZ");

  std::vector<std::string> cart_tool_offset_var_suffixes;
  cart_tool_offset_var_suffixes.push_back("_TOOL_POS_X");
  cart_tool_offset_var_suffixes.push_back("_TOOL_POS_Y");
  cart_tool_offset_var_suffixes.push_back("_TOOL_POS_Z");
  cart_tool_offset_var_suffixes.push_back("_TOOL_ORIENT_QW");
  cart_tool_offset_var_suffixes.push_back("_TOOL_ORIENT_QX");
  cart_tool_offset_var_suffixes.push_back("_TOOL_ORIENT_QY");
  cart_tool_offset_var_suffixes.push_back("_TOOL_ORIENT_QZ");

  std::vector<std::string> cart_force_var_suffixes;
  cart_force_var_suffixes.push_back("_FORCE_X");
  cart_force_var_suffixes.push_back("_FORCE_Y");
  cart_force_var_suffixes.push_back("_FORCE_Z");
  cart_force_var_suffixes.push_back("_TORQUE_X");
  cart_force_var_suffixes.push_back("_TORQUE_Y");
  cart_force_var_suffixes.push_back("_TORQUE_Z");

  std::vector<std::string> cart_gain_var_suffixes;
  cart_gain_var_suffixes.push_back("_POS_X_PGAIN");
  cart_gain_var_suffixes.push_back("_POS_X_IGAIN");
  cart_gain_var_suffixes.push_back("_POS_X_DGAIN");
  cart_gain_var_suffixes.push_back("_POS_Y_PGAIN");
  cart_gain_var_suffixes.push_back("_POS_Y_IGAIN");
  cart_gain_var_suffixes.push_back("_POS_Y_DGAIN");
  cart_gain_var_suffixes.push_back("_POS_Z_PGAIN");
  cart_gain_var_suffixes.push_back("_POS_Z_IGAIN");
  cart_gain_var_suffixes.push_back("_POS_Z_DGAIN");
  cart_gain_var_suffixes.push_back("_ORIENT_X_PGAIN");
  cart_gain_var_suffixes.push_back("_ORIENT_X_IGAIN");
  cart_gain_var_suffixes.push_back("_ORIENT_X_DGAIN");
  cart_gain_var_suffixes.push_back("_ORIENT_Y_PGAIN");
  cart_gain_var_suffixes.push_back("_ORIENT_Y_IGAIN");
  cart_gain_var_suffixes.push_back("_ORIENT_Y_DGAIN");
  cart_gain_var_suffixes.push_back("_ORIENT_Z_PGAIN");
  cart_gain_var_suffixes.push_back("_ORIENT_Z_IGAIN");
  cart_gain_var_suffixes.push_back("_ORIENT_Z_DGAIN");

  std::vector<std::string> cart_force_gain_var_suffixes;
  cart_force_gain_var_suffixes.push_back("_FORCE_X_PGAIN");
  cart_force_gain_var_suffixes.push_back("_FORCE_X_IGAIN");
  cart_force_gain_var_suffixes.push_back("_FORCE_X_DGAIN");
  cart_force_gain_var_suffixes.push_back("_FORCE_Y_PGAIN");
  cart_force_gain_var_suffixes.push_back("_FORCE_Y_IGAIN");
  cart_force_gain_var_suffixes.push_back("_FORCE_Y_DGAIN");
  cart_force_gain_var_suffixes.push_back("_FORCE_Z_PGAIN");
  cart_force_gain_var_suffixes.push_back("_FORCE_Z_IGAIN");
  cart_force_gain_var_suffixes.push_back("_FORCE_Z_DGAIN");
  cart_force_gain_var_suffixes.push_back("_TORQUE_X_PGAIN");
  cart_force_gain_var_suffixes.push_back("_TORQUE_X_IGAIN");
  cart_force_gain_var_suffixes.push_back("_TORQUE_X_DGAIN");
  cart_force_gain_var_suffixes.push_back("_TORQUE_Y_PGAIN");
  cart_force_gain_var_suffixes.push_back("_TORQUE_Y_IGAIN");
  cart_force_gain_var_suffixes.push_back("_TORQUE_Y_DGAIN");
  cart_force_gain_var_suffixes.push_back("_TORQUE_Z_PGAIN");
  cart_force_gain_var_suffixes.push_back("_TORQUE_Z_IGAIN");
  cart_force_gain_var_suffixes.push_back("_TORQUE_Z_DGAIN");

  std::vector<std::string> joint_var_suffixes;
  joint_var_suffixes.push_back("");

  std::vector<std::string> cart_null_joint_var_suffixes;
  cart_null_joint_var_suffixes.push_back("_POSTURE");

  std::vector<std::string> joint_force_var_suffixes;
  joint_force_var_suffixes.push_back("_FORCE");

  std::vector<std::string> joint_gain_var_suffixes;
  joint_gain_var_suffixes.push_back("_PGAIN");
  joint_gain_var_suffixes.push_back("_IGAIN");
  joint_gain_var_suffixes.push_back("_DGAIN");

  std::vector<std::string> joint_force_gain_var_suffixes;
  joint_force_gain_var_suffixes.push_back("_FORCE_PGAIN");
  joint_force_gain_var_suffixes.push_back("_FORCE_IGAIN");
  joint_force_gain_var_suffixes.push_back("_FORCE_DGAIN");

  // set all control modes to false first to prevent valgrind from complaining
  control_cartesian_ = false;
  control_cartesian_tool_frames_ = false;
  control_cartesian_forces_ = false;
  control_cartesian_gains_ = false;
  control_cartesian_force_gains_ = false;
  control_joints_ = false;
  control_cartesian_null_space_ = false;
  control_joint_forces_ = false;
  control_joint_gains_ = false;
  control_joint_force_gains_ = false;

  start_index_cartesian_ = 0;
  if (!readEndeffNamesFromConfig(config, "control_cartesian", dim_names,
                                 control_cartesian_, control_cartesian_endeffs_, cart_var_suffixes))
    return false;
  start_index_cartesian_tool_frames_ = dim_names.size();
  if (!readEndeffNamesFromConfig(config, "control_cartesian_tool_frames", dim_names,
                                 control_cartesian_tool_frames_, control_cartesian_tool_frames_endeffs_, cart_tool_offset_var_suffixes))
    return false;
  start_index_cartesian_forces_ = dim_names.size();
  if (!readEndeffNamesFromConfig(config, "control_cartesian_forces", dim_names,
                                 control_cartesian_forces_, control_cartesian_forces_endeffs_, cart_force_var_suffixes))
    return false;
  start_index_cartesian_gains_ = dim_names.size();
  if (!readEndeffNamesFromConfig(config, "control_cartesian_gains", dim_names,
                                 control_cartesian_gains_, control_cartesian_gains_endeffs_, cart_gain_var_suffixes))
    return false;
  start_index_cartesian_force_gains_ = dim_names.size();
  if (!readEndeffNamesFromConfig(config, "control_cartesian_force_gains", dim_names,
                                 control_cartesian_force_gains_, control_cartesian_force_gains_endeffs_, cart_force_gain_var_suffixes))
    return false;
  start_index_joints_ = dim_names.size();
  if (!readJointNamesFromConfig(config, "control_joints", dim_names,
                                control_joints_, control_joints_joints_, joint_var_suffixes))
    return false;
  start_index_cartesian_null_space_ = dim_names.size();
  if (!readJointNamesFromConfig(config, "control_cartesian_null_space", dim_names,
                                control_cartesian_null_space_, control_cartesian_null_space_joints_, cart_null_joint_var_suffixes))
    return false;
  start_index_joint_forces_ = dim_names.size();
  if (!readJointNamesFromConfig(config, "control_joint_forces", dim_names,
                                control_joint_forces_, control_joint_forces_joints_, joint_force_var_suffixes))
    return false;
  start_index_joint_gains_ = dim_names.size();
  if (!readJointNamesFromConfig(config, "control_joint_gains", dim_names,
                                control_joint_gains_, control_joint_gains_joints_, joint_gain_var_suffixes))
    return false;
  start_index_joint_force_gains_ = dim_names.size();
  if (!readJointNamesFromConfig(config, "control_joint_force_gains", dim_names,
                                control_joint_force_gains_, control_joint_force_gains_joints_, joint_force_gain_var_suffixes))

  num_dimensions_ = dim_names.size();

  // initialize the trajectory generator:
  if (!TrajectoryGenerator::initialize(config, dim_names))
  {
    ROS_ERROR("%s: couldn't initialize TrajectoryGenerator", name_.c_str());
    return false;
  }

  return true;
}

bool ColorTrajectoryGenerator::update()
{
  //stop if we cannot process the message
  if(!processMessages())
  {
    ROS_ERROR("cannot process message.");
    return false;
  }
  updateTarget();

  if (control_cartesian_tool_frames_)
  {
    for (unsigned int e=0; e<control_cartesian_tool_frames_endeffs_.size(); ++e)
    {
      int endeff = control_cartesian_tool_frames_endeffs_[e];
      for (int i=POS_X; i<=POS_Z; i++)
      {
        data_->cart_tool_offset_[endeff].x[i+1] = target_.point_.positions[start_index_cartesian_tool_frames_ + e*NUM_CARTESIAN + i];
        data_->cart_tool_offset_[endeff].xd[i+1] = target_.point_.velocities[start_index_cartesian_tool_frames_ + e*NUM_CARTESIAN + i];
        data_->cart_tool_offset_[endeff].xdd[i+1] = target_.point_.accelerations[start_index_cartesian_tool_frames_ + e*NUM_CARTESIAN + i];
      }
      for (int i=ORIENT_QW; i<=ORIENT_QZ; i++)
      {
        data_->cart_tool_offset_orient_[endeff].q[i-ORIENT_QW+_QW_] = target_.point_.positions[start_index_cartesian_tool_frames_ + e*NUM_CARTESIAN + i];
        data_->cart_tool_offset_orient_[endeff].qd[i-ORIENT_QW+_QW_] = target_.point_.velocities[start_index_cartesian_tool_frames_ + e*NUM_CARTESIAN + i];
        data_->cart_tool_offset_orient_[endeff].qdd[i-ORIENT_QW+_QW_] = target_.point_.accelerations[start_index_cartesian_tool_frames_ + e*NUM_CARTESIAN + i];
      }

      // normalize the quaternion:
      if(!normalizeQuaternion(data_->cart_tool_offset_orient_[endeff]))
      {
        ROS_ERROR("cannot normalize quaternion.");
        return false;
      }

      // assign angular velocity and acceleration
      quatToAngularVelAcc(data_->cart_tool_offset_orient_[endeff]);
    }

  }

  if (control_cartesian_)
  {
    for (unsigned int e=0; e<control_cartesian_endeffs_.size(); ++e)
    {
      int endeff = control_cartesian_endeffs_[e];
      for (int i=POS_X; i<=POS_Z; i++)
      {
        data_->cart_tool_des_state_[endeff].x[i+1] = target_.point_.positions[start_index_cartesian_ + e*NUM_CARTESIAN + i];
        data_->cart_tool_des_state_[endeff].xd[i+1] = target_.point_.velocities[start_index_cartesian_ + e*NUM_CARTESIAN + i];
        data_->cart_tool_des_state_[endeff].xdd[i+1] = target_.point_.accelerations[start_index_cartesian_ + e*NUM_CARTESIAN + i];
      }
      for (int i=ORIENT_QW; i<=ORIENT_QZ; i++)
      {
        data_->cart_tool_des_orient_[endeff].q[i-ORIENT_QW+_QW_] = target_.point_.positions[start_index_cartesian_ + e*NUM_CARTESIAN + i];
        data_->cart_tool_des_orient_[endeff].qd[i-ORIENT_QW+_QW_] = target_.point_.velocities[start_index_cartesian_ + e*NUM_CARTESIAN + i];
        data_->cart_tool_des_orient_[endeff].qdd[i-ORIENT_QW+_QW_] = target_.point_.accelerations[start_index_cartesian_ + e*NUM_CARTESIAN + i];
      }

      // normalize the quaternion:
      if(!normalizeQuaternion(data_->cart_tool_des_orient_[endeff]))
      {
        ROS_ERROR("cannot normalize quaternion.");
        return false;
      }

      // assign angular velocity and acceleration
      quatToAngularVelAcc(data_->cart_tool_des_orient_[endeff]);

      // update hand des state
      sl_utilities::getCartHandState(data_->cart_tool_des_state_[endeff], data_->cart_tool_des_orient_[endeff],
                                     data_->cart_tool_offset_[endeff], data_->cart_tool_offset_orient_[endeff],
                                     data_->cart_hand_des_state_[endeff], data_->cart_hand_des_orient_[endeff]);
    }
  }

  if (control_cartesian_forces_)
  {
    for (unsigned int e=0; e<control_cartesian_forces_endeffs_.size(); ++e)
    {
      int endeff = control_cartesian_forces_endeffs_[e];
      data_->cart_des_force_[endeff].force.x = target_.point_.positions[start_index_cartesian_forces_ + e*NUM_CARTESIAN_FORCES + FORCE_X];
      data_->cart_des_force_[endeff].force.y = target_.point_.positions[start_index_cartesian_forces_ + e*NUM_CARTESIAN_FORCES + FORCE_Y];
      data_->cart_des_force_[endeff].force.z = target_.point_.positions[start_index_cartesian_forces_ + e*NUM_CARTESIAN_FORCES + FORCE_Z];
      data_->cart_des_force_[endeff].torque.x = target_.point_.positions[start_index_cartesian_forces_ + e*NUM_CARTESIAN_FORCES + TORQUE_X];
      data_->cart_des_force_[endeff].torque.y = target_.point_.positions[start_index_cartesian_forces_ + e*NUM_CARTESIAN_FORCES + TORQUE_Y];
      data_->cart_des_force_[endeff].torque.z = target_.point_.positions[start_index_cartesian_forces_ + e*NUM_CARTESIAN_FORCES + TORQUE_Z];
    }
  }

  if (control_cartesian_gains_)
  {
    for (unsigned int e=0; e<control_cartesian_gains_endeffs_.size(); ++e)
    {
      int endeff = control_cartesian_gains_endeffs_[e];
      data_->cart_gains_[endeff].pos_x.p_gain = target_.point_.positions[start_index_cartesian_gains_ + e*NUM_CARTESIAN_GAINS + POS_X_PGAIN];
      data_->cart_gains_[endeff].pos_x.i_gain = target_.point_.positions[start_index_cartesian_gains_ + e*NUM_CARTESIAN_GAINS + POS_X_IGAIN];
      data_->cart_gains_[endeff].pos_x.d_gain = target_.point_.positions[start_index_cartesian_gains_ + e*NUM_CARTESIAN_GAINS + POS_X_DGAIN];
      data_->cart_gains_[endeff].pos_y.p_gain = target_.point_.positions[start_index_cartesian_gains_ + e*NUM_CARTESIAN_GAINS + POS_Y_PGAIN];
      data_->cart_gains_[endeff].pos_y.i_gain = target_.point_.positions[start_index_cartesian_gains_ + e*NUM_CARTESIAN_GAINS + POS_Y_IGAIN];
      data_->cart_gains_[endeff].pos_y.d_gain = target_.point_.positions[start_index_cartesian_gains_ + e*NUM_CARTESIAN_GAINS + POS_Y_DGAIN];
      data_->cart_gains_[endeff].pos_z.p_gain = target_.point_.positions[start_index_cartesian_gains_ + e*NUM_CARTESIAN_GAINS + POS_Z_PGAIN];
      data_->cart_gains_[endeff].pos_z.i_gain = target_.point_.positions[start_index_cartesian_gains_ + e*NUM_CARTESIAN_GAINS + POS_Z_IGAIN];
      data_->cart_gains_[endeff].pos_z.d_gain = target_.point_.positions[start_index_cartesian_gains_ + e*NUM_CARTESIAN_GAINS + POS_Z_DGAIN];
      data_->cart_gains_[endeff].rot_x.p_gain = target_.point_.positions[start_index_cartesian_gains_ + e*NUM_CARTESIAN_GAINS + ORIENT_X_PGAIN];
      data_->cart_gains_[endeff].rot_x.i_gain = target_.point_.positions[start_index_cartesian_gains_ + e*NUM_CARTESIAN_GAINS + ORIENT_X_IGAIN];
      data_->cart_gains_[endeff].rot_x.d_gain = target_.point_.positions[start_index_cartesian_gains_ + e*NUM_CARTESIAN_GAINS + ORIENT_X_DGAIN];
      data_->cart_gains_[endeff].rot_y.p_gain = target_.point_.positions[start_index_cartesian_gains_ + e*NUM_CARTESIAN_GAINS + ORIENT_Y_PGAIN];
      data_->cart_gains_[endeff].rot_y.i_gain = target_.point_.positions[start_index_cartesian_gains_ + e*NUM_CARTESIAN_GAINS + ORIENT_Y_IGAIN];
      data_->cart_gains_[endeff].rot_y.d_gain = target_.point_.positions[start_index_cartesian_gains_ + e*NUM_CARTESIAN_GAINS + ORIENT_Y_DGAIN];
      data_->cart_gains_[endeff].rot_z.p_gain = target_.point_.positions[start_index_cartesian_gains_ + e*NUM_CARTESIAN_GAINS + ORIENT_Z_PGAIN];
      data_->cart_gains_[endeff].rot_z.i_gain = target_.point_.positions[start_index_cartesian_gains_ + e*NUM_CARTESIAN_GAINS + ORIENT_Z_IGAIN];
      data_->cart_gains_[endeff].rot_z.d_gain = target_.point_.positions[start_index_cartesian_gains_ + e*NUM_CARTESIAN_GAINS + ORIENT_Z_DGAIN];
    }
  }

  if (control_cartesian_force_gains_)
  {
    for (unsigned int e=0; e<control_cartesian_force_gains_endeffs_.size(); ++e)
    {
      int endeff = control_cartesian_force_gains_endeffs_[e];
      data_->cart_force_gains_[endeff].pos_x.p_gain = target_.point_.positions[start_index_cartesian_force_gains_ + e*NUM_CARTESIAN_FORCE_GAINS + FORCE_X_PGAIN];
      data_->cart_force_gains_[endeff].pos_x.i_gain = target_.point_.positions[start_index_cartesian_force_gains_ + e*NUM_CARTESIAN_FORCE_GAINS + FORCE_X_IGAIN];
      data_->cart_force_gains_[endeff].pos_x.d_gain = target_.point_.positions[start_index_cartesian_force_gains_ + e*NUM_CARTESIAN_FORCE_GAINS + FORCE_X_DGAIN];
      data_->cart_force_gains_[endeff].pos_y.p_gain = target_.point_.positions[start_index_cartesian_force_gains_ + e*NUM_CARTESIAN_FORCE_GAINS + FORCE_Y_PGAIN];
      data_->cart_force_gains_[endeff].pos_y.i_gain = target_.point_.positions[start_index_cartesian_force_gains_ + e*NUM_CARTESIAN_FORCE_GAINS + FORCE_Y_IGAIN];
      data_->cart_force_gains_[endeff].pos_y.d_gain = target_.point_.positions[start_index_cartesian_force_gains_ + e*NUM_CARTESIAN_FORCE_GAINS + FORCE_Y_DGAIN];
      data_->cart_force_gains_[endeff].pos_z.p_gain = target_.point_.positions[start_index_cartesian_force_gains_ + e*NUM_CARTESIAN_FORCE_GAINS + FORCE_Z_PGAIN];
      data_->cart_force_gains_[endeff].pos_z.i_gain = target_.point_.positions[start_index_cartesian_force_gains_ + e*NUM_CARTESIAN_FORCE_GAINS + FORCE_Z_IGAIN];
      data_->cart_force_gains_[endeff].pos_z.d_gain = target_.point_.positions[start_index_cartesian_force_gains_ + e*NUM_CARTESIAN_FORCE_GAINS + FORCE_Z_DGAIN];
      data_->cart_force_gains_[endeff].rot_x.p_gain = target_.point_.positions[start_index_cartesian_force_gains_ + e*NUM_CARTESIAN_FORCE_GAINS + TORQUE_X_PGAIN];
      data_->cart_force_gains_[endeff].rot_x.i_gain = target_.point_.positions[start_index_cartesian_force_gains_ + e*NUM_CARTESIAN_FORCE_GAINS + TORQUE_X_IGAIN];
      data_->cart_force_gains_[endeff].rot_x.d_gain = target_.point_.positions[start_index_cartesian_force_gains_ + e*NUM_CARTESIAN_FORCE_GAINS + TORQUE_X_DGAIN];
      data_->cart_force_gains_[endeff].rot_y.p_gain = target_.point_.positions[start_index_cartesian_force_gains_ + e*NUM_CARTESIAN_FORCE_GAINS + TORQUE_Y_PGAIN];
      data_->cart_force_gains_[endeff].rot_y.i_gain = target_.point_.positions[start_index_cartesian_force_gains_ + e*NUM_CARTESIAN_FORCE_GAINS + TORQUE_Y_IGAIN];
      data_->cart_force_gains_[endeff].rot_y.d_gain = target_.point_.positions[start_index_cartesian_force_gains_ + e*NUM_CARTESIAN_FORCE_GAINS + TORQUE_Y_DGAIN];
      data_->cart_force_gains_[endeff].rot_z.p_gain = target_.point_.positions[start_index_cartesian_force_gains_ + e*NUM_CARTESIAN_FORCE_GAINS + TORQUE_Z_PGAIN];
      data_->cart_force_gains_[endeff].rot_z.i_gain = target_.point_.positions[start_index_cartesian_force_gains_ + e*NUM_CARTESIAN_FORCE_GAINS + TORQUE_Z_IGAIN];
      data_->cart_force_gains_[endeff].rot_z.d_gain = target_.point_.positions[start_index_cartesian_force_gains_ + e*NUM_CARTESIAN_FORCE_GAINS + TORQUE_Z_DGAIN];
    }
  }

  if (control_cartesian_null_space_)
  {
    for (unsigned int i=0; i<control_cartesian_null_space_joints_.size(); ++i)
    {
      int sl_joint = control_cartesian_null_space_joints_[i];
      int local_joint = start_index_cartesian_null_space_ + i;
      data_->joint_des_posture_[sl_joint].th = target_.point_.positions[local_joint];
      data_->joint_des_posture_[sl_joint].thd = target_.point_.velocities[local_joint];
      data_->joint_des_posture_[sl_joint].thdd = target_.point_.accelerations[local_joint];
    }
  }

  if (control_joints_)
  {
    for (unsigned int i=0; i<control_joints_joints_.size(); ++i)
    {
      int sl_joint = control_joints_joints_[i];
      int local_joint = start_index_joints_ + i;
      data_->joint_des_state_[sl_joint].th = target_.point_.positions[local_joint];
      data_->joint_des_state_[sl_joint].thd = target_.point_.velocities[local_joint];
      data_->joint_des_state_[sl_joint].thdd = target_.point_.accelerations[local_joint];
    }
  }

  if (control_joint_forces_)
  {
    for (unsigned int i=0; i<control_joint_forces_joints_.size(); ++i)
    {
      int sl_joint = control_joint_forces_joints_[i];
      int local_joint = start_index_joint_forces_ + i;
      data_->joint_des_force_[sl_joint] = target_.point_.positions[local_joint];
    }
  }

  if (control_joint_gains_)
  {
    for (unsigned int i=0; i<control_joint_gains_joints_.size(); ++i)
    {
      int sl_joint = control_joint_gains_joints_[i];
      int local_joint = start_index_joint_gains_ + i*NUM_JOINT_GAINS;
      data_->joint_gains_[sl_joint].p_gain = target_.point_.positions[local_joint + PGAIN];
      data_->joint_gains_[sl_joint].i_gain = target_.point_.positions[local_joint + IGAIN];
      data_->joint_gains_[sl_joint].d_gain = target_.point_.positions[local_joint + DGAIN];
    }
  }

  if (control_joint_force_gains_)
  {
    for (unsigned int i=0; i<control_joint_force_gains_joints_.size(); ++i)
    {
      int sl_joint = control_joint_force_gains_joints_[i];
      int local_joint = start_index_joint_force_gains_ + i*NUM_JOINT_GAINS;
      data_->joint_force_gains_[sl_joint].p_gain = target_.point_.positions[local_joint + PGAIN];
      data_->joint_force_gains_[sl_joint].i_gain = target_.point_.positions[local_joint + IGAIN];
      data_->joint_force_gains_[sl_joint].d_gain = target_.point_.positions[local_joint + DGAIN];
    }
  }

  return true;
}

bool UniversalTrajectoryGenerator::start()
{
  if (control_cartesian_tool_frames_)
  {
    for (unsigned int e=0; e<control_cartesian_endeffs_.size(); ++e)
    {
      int endeff = control_cartesian_tool_frames_endeffs_[e];
      data_->cart_tool_offset_[endeff].x[_X_] = 0.0;
      data_->cart_tool_offset_[endeff].x[_Y_] = 0.0;
      data_->cart_tool_offset_[endeff].x[_Z_] = 0.0;
      data_->cart_tool_offset_orient_[endeff].q[_QW_] = 1.0;
      data_->cart_tool_offset_orient_[endeff].q[_QX_] = 0.0;
      data_->cart_tool_offset_orient_[endeff].q[_QY_] = 0.0;
      data_->cart_tool_offset_orient_[endeff].q[_QZ_] = 0.0;
      ControllerUtilities::zeroVelAcc(data_->cart_tool_offset_[endeff]);
      ControllerUtilities::zeroVelAcc(data_->cart_tool_offset_orient_[endeff]);
    }
  }

  if (control_cartesian_)
  {
    for (unsigned int e=0; e<control_cartesian_endeffs_.size(); ++e)
    {
      int endeff = control_cartesian_endeffs_[e];
      if (data_->use_visual_tracking_[endeff])
      {
        data_->cart_tool_des_orient_[endeff] = data_->cart_hand_visual_orient_[endeff];
        data_->cart_tool_des_state_[endeff] = data_->cart_hand_visual_state_[endeff];
      }
      else
      {
        data_->cart_tool_des_orient_[endeff] = data_->cart_hand_orient_[endeff];
        data_->cart_tool_des_state_[endeff] = data_->cart_hand_state_[endeff];
      }
      ControllerUtilities::zeroVelAcc(data_->cart_tool_des_orient_[endeff]);
      ControllerUtilities::zeroVelAcc(data_->cart_tool_des_state_[endeff]);
    }
  }


  if (control_joints_)
  {
    for (unsigned int i=0; i<control_joints_joints_.size(); ++i)
    {
      int sl_joint = control_joints_joints_[i];
      data_->joint_des_state_[sl_joint] = joint_des_state[sl_joint];
    }
  }

  if (control_cartesian_null_space_)
  {
    for (unsigned int i=0; i<control_cartesian_null_space_joints_.size(); ++i)
    {
      int sl_joint = control_cartesian_null_space_joints_[i];
      data_->joint_des_posture_[sl_joint] = joint_des_state[sl_joint];
    }
  }

  getCurrentTrajectoryPoint(target_);

  return true;
}

bool UniversalTrajectoryGenerator::stop()
{
  holdCurrentPositions();

  // zero the tool frames
//  for (unsigned int e=0; e<control_cartesian_endeffs_.size(); ++e)
//  {
//    int endeff = control_cartesian_tool_frames_endeffs_[e];
//    data_->cart_tool_offset_[endeff].x[_X_] = 0.0;
//    data_->cart_tool_offset_[endeff].x[_Y_] = 0.0;
//    data_->cart_tool_offset_[endeff].x[_Z_] = 0.0;
//    data_->cart_tool_offset_orient_[endeff].q[_QW_] = 1.0;
//    data_->cart_tool_offset_orient_[endeff].q[_QX_] = 0.0;
//    data_->cart_tool_offset_orient_[endeff].q[_QY_] = 0.0;
//    data_->cart_tool_offset_orient_[endeff].q[_QZ_] = 0.0;
//    ControllerUtilities::zeroVelAcc(data_->cart_tool_offset_[endeff]);
//    ControllerUtilities::zeroVelAcc(data_->cart_tool_offset_orient_[endeff]);
//  }
  return true;
}

void ColorTrajectoryGenerator::updateCurSplineFromPoints(TrajectoryPoint& p1, TrajectoryPoint& p2)
{
  // modify the p2 quaternion to go the shorter way around:
  if (control_cartesian_)
  {
    for (unsigned int e=0; e<control_cartesian_endeffs_.size(); ++e)
    {
      double dot = 0.0;
      for (int i=ORIENT_QW; i<=ORIENT_QZ; i++)
      {
        dot += p1.point_.positions[start_index_cartesian_ + e*NUM_CARTESIAN + i] *
            p2.point_.positions[start_index_cartesian_ + e*NUM_CARTESIAN + i];
      }
      if (dot < 0)
      {
        for (int i=ORIENT_QW; i<=ORIENT_QZ; i++)
        {
          int index = start_index_cartesian_ + e*NUM_CARTESIAN + i;
          p2.point_.positions[index] = -p2.point_.positions[index];
          p2.point_.velocities[index] = -p2.point_.velocities[index];
          p2.point_.accelerations[index] = -p2.point_.accelerations[index];
        }
      }
    }
  }

  TrajectoryGenerator::updateCurSplineFromPoints(p1, p2);
}

//void UniversalTrajectoryGenerator::slToRosCState(const SL_Cstate& sl_c_state,
//                                                  const SL_quat& sl_quat,
//                                                  TrajectoryPoint& ros_c_state)
//{
//  for (int i=X; i<=Z; i++)
//  {
//    ros_c_state.point_.positions[i+start_index_cartesian_] = sl_c_state.x[i+1];
//    ros_c_state.point_.velocities[i+start_index_cartesian_] = sl_c_state.xd[i+1];
//    ros_c_state.point_.accelerations[i+start_index_cartesian_] = sl_c_state.xdd[i+1];
//  }
//  for (int i=QW; i<=QZ; i++)
//  {
//    ros_c_state.point_.positions[i+start_index_cartesian_] = sl_quat.q[i-QW+_QW_];
//    ros_c_state.point_.velocities[i+start_index_cartesian_] = sl_quat.qd[i-QW+_QW_];
//    ros_c_state.point_.accelerations[i+start_index_cartesian_] = sl_quat.qdd[i-QW+_QW_];
//  }
//}

void ColorTrajectoryGenerator::getCurrentTrajectoryPoint(TrajectoryPoint& point)
{
  if (control_cartesian_)
  {
    for (unsigned int e=0; e<control_cartesian_endeffs_.size(); ++e)
    {
      int endeff = control_cartesian_endeffs_[e];
      for (int i=POS_X; i<=POS_Z; i++)
      {
        point.point_.positions[start_index_cartesian_ + e*NUM_CARTESIAN + i]     = data_->cart_tool_des_state_[endeff].x[i+1];
        point.point_.velocities[start_index_cartesian_ + e*NUM_CARTESIAN + i]    = data_->cart_tool_des_state_[endeff].xd[i+1];
        point.point_.accelerations[start_index_cartesian_ + e*NUM_CARTESIAN + i] = data_->cart_tool_des_state_[endeff].xdd[i+1];
      }
      for (int i=ORIENT_QW; i<=ORIENT_QZ; i++)
      {
        point.point_.positions[start_index_cartesian_ + e*NUM_CARTESIAN + i]     = data_->cart_tool_des_orient_[endeff].q[i-ORIENT_QW+_QW_];
        point.point_.velocities[start_index_cartesian_ + e*NUM_CARTESIAN + i]    = data_->cart_tool_des_orient_[endeff].qd[i-ORIENT_QW+_QW_];
        point.point_.accelerations[start_index_cartesian_ + e*NUM_CARTESIAN + i] = data_->cart_tool_des_orient_[endeff].qdd[i-ORIENT_QW+_QW_];
      }
    }
  }

  if (control_cartesian_tool_frames_)
  {
    for (unsigned int e=0; e<control_cartesian_tool_frames_endeffs_.size(); ++e)
    {
      int endeff = control_cartesian_tool_frames_endeffs_[e];
      for (int i=POS_X; i<=POS_Z; i++)
      {
        point.point_.positions[start_index_cartesian_tool_frames_ + e*NUM_CARTESIAN + i]     = data_->cart_tool_offset_[endeff].x[i+1];
        point.point_.velocities[start_index_cartesian_tool_frames_ + e*NUM_CARTESIAN + i]    = data_->cart_tool_offset_[endeff].xd[i+1];
        point.point_.accelerations[start_index_cartesian_tool_frames_ + e*NUM_CARTESIAN + i] = data_->cart_tool_offset_[endeff].xdd[i+1];
      }
      for (int i=ORIENT_QW; i<=ORIENT_QZ; i++)
      {
        point.point_.positions[start_index_cartesian_tool_frames_ + e*NUM_CARTESIAN + i]     = data_->cart_tool_offset_orient_[endeff].q[i-ORIENT_QW+_QW_];
        point.point_.velocities[start_index_cartesian_tool_frames_ + e*NUM_CARTESIAN + i]    = data_->cart_tool_offset_orient_[endeff].qd[i-ORIENT_QW+_QW_];
        point.point_.accelerations[start_index_cartesian_tool_frames_ + e*NUM_CARTESIAN + i] = data_->cart_tool_offset_orient_[endeff].qdd[i-ORIENT_QW+_QW_];
      }
    }
  }

  if (control_cartesian_forces_)
  {
    for (unsigned int e=0; e<control_cartesian_forces_endeffs_.size(); ++e)
    {
      int endeff = control_cartesian_forces_endeffs_[e];
      point.point_.positions[start_index_cartesian_forces_ + e*NUM_CARTESIAN_FORCES + FORCE_X]  = data_->cart_des_force_[endeff].force.x;
      point.point_.positions[start_index_cartesian_forces_ + e*NUM_CARTESIAN_FORCES + FORCE_Y]  = data_->cart_des_force_[endeff].force.y;
      point.point_.positions[start_index_cartesian_forces_ + e*NUM_CARTESIAN_FORCES + FORCE_Z]  = data_->cart_des_force_[endeff].force.z;
      point.point_.positions[start_index_cartesian_forces_ + e*NUM_CARTESIAN_FORCES + TORQUE_X] = data_->cart_des_force_[endeff].torque.x;
      point.point_.positions[start_index_cartesian_forces_ + e*NUM_CARTESIAN_FORCES + TORQUE_Y] = data_->cart_des_force_[endeff].torque.y;
      point.point_.positions[start_index_cartesian_forces_ + e*NUM_CARTESIAN_FORCES + TORQUE_Z] = data_->cart_des_force_[endeff].torque.z;
    }
  }

  if (control_cartesian_gains_)
  {
    for (unsigned int e=0; e<control_cartesian_gains_endeffs_.size(); ++e)
    {
      int endeff = control_cartesian_gains_endeffs_[e];
      point.point_.positions[start_index_cartesian_gains_ + e*NUM_CARTESIAN_GAINS + POS_X_PGAIN]    = data_->cart_gains_[endeff].pos_x.p_gain;
      point.point_.positions[start_index_cartesian_gains_ + e*NUM_CARTESIAN_GAINS + POS_X_IGAIN]    = data_->cart_gains_[endeff].pos_x.i_gain;
      point.point_.positions[start_index_cartesian_gains_ + e*NUM_CARTESIAN_GAINS + POS_X_DGAIN]    = data_->cart_gains_[endeff].pos_x.d_gain;
      point.point_.positions[start_index_cartesian_gains_ + e*NUM_CARTESIAN_GAINS + POS_Y_PGAIN]    = data_->cart_gains_[endeff].pos_y.p_gain;
      point.point_.positions[start_index_cartesian_gains_ + e*NUM_CARTESIAN_GAINS + POS_Y_IGAIN]    = data_->cart_gains_[endeff].pos_y.i_gain;
      point.point_.positions[start_index_cartesian_gains_ + e*NUM_CARTESIAN_GAINS + POS_Y_DGAIN]    = data_->cart_gains_[endeff].pos_y.d_gain;
      point.point_.positions[start_index_cartesian_gains_ + e*NUM_CARTESIAN_GAINS + POS_Z_PGAIN]    = data_->cart_gains_[endeff].pos_z.p_gain;
      point.point_.positions[start_index_cartesian_gains_ + e*NUM_CARTESIAN_GAINS + POS_Z_IGAIN]    = data_->cart_gains_[endeff].pos_z.i_gain;
      point.point_.positions[start_index_cartesian_gains_ + e*NUM_CARTESIAN_GAINS + POS_Z_DGAIN]    = data_->cart_gains_[endeff].pos_z.d_gain;
      point.point_.positions[start_index_cartesian_gains_ + e*NUM_CARTESIAN_GAINS + ORIENT_X_PGAIN] = data_->cart_gains_[endeff].rot_x.p_gain;
      point.point_.positions[start_index_cartesian_gains_ + e*NUM_CARTESIAN_GAINS + ORIENT_X_IGAIN] = data_->cart_gains_[endeff].rot_x.i_gain;
      point.point_.positions[start_index_cartesian_gains_ + e*NUM_CARTESIAN_GAINS + ORIENT_X_DGAIN] = data_->cart_gains_[endeff].rot_x.d_gain;
      point.point_.positions[start_index_cartesian_gains_ + e*NUM_CARTESIAN_GAINS + ORIENT_Y_PGAIN] = data_->cart_gains_[endeff].rot_y.p_gain;
      point.point_.positions[start_index_cartesian_gains_ + e*NUM_CARTESIAN_GAINS + ORIENT_Y_IGAIN] = data_->cart_gains_[endeff].rot_y.i_gain;
      point.point_.positions[start_index_cartesian_gains_ + e*NUM_CARTESIAN_GAINS + ORIENT_Y_DGAIN] = data_->cart_gains_[endeff].rot_y.d_gain;
      point.point_.positions[start_index_cartesian_gains_ + e*NUM_CARTESIAN_GAINS + ORIENT_Z_PGAIN] = data_->cart_gains_[endeff].rot_z.p_gain;
      point.point_.positions[start_index_cartesian_gains_ + e*NUM_CARTESIAN_GAINS + ORIENT_Z_IGAIN] = data_->cart_gains_[endeff].rot_z.i_gain;
      point.point_.positions[start_index_cartesian_gains_ + e*NUM_CARTESIAN_GAINS + ORIENT_Z_DGAIN] = data_->cart_gains_[endeff].rot_z.d_gain;
    }
  }

  if (control_cartesian_force_gains_)
  {
    for (unsigned int e=0; e<control_cartesian_force_gains_endeffs_.size(); ++e)
    {
      int endeff = control_cartesian_force_gains_endeffs_[e];
      point.point_.positions[start_index_cartesian_force_gains_ + e*NUM_CARTESIAN_FORCE_GAINS + FORCE_X_PGAIN]  = data_->cart_force_gains_[endeff].pos_x.p_gain;
      point.point_.positions[start_index_cartesian_force_gains_ + e*NUM_CARTESIAN_FORCE_GAINS + FORCE_X_IGAIN]  = data_->cart_force_gains_[endeff].pos_x.i_gain;
      point.point_.positions[start_index_cartesian_force_gains_ + e*NUM_CARTESIAN_FORCE_GAINS + FORCE_X_DGAIN]  = data_->cart_force_gains_[endeff].pos_x.d_gain;
      point.point_.positions[start_index_cartesian_force_gains_ + e*NUM_CARTESIAN_FORCE_GAINS + FORCE_Y_PGAIN]  = data_->cart_force_gains_[endeff].pos_y.p_gain;
      point.point_.positions[start_index_cartesian_force_gains_ + e*NUM_CARTESIAN_FORCE_GAINS + FORCE_Y_IGAIN]  = data_->cart_force_gains_[endeff].pos_y.i_gain;
      point.point_.positions[start_index_cartesian_force_gains_ + e*NUM_CARTESIAN_FORCE_GAINS + FORCE_Y_DGAIN]  = data_->cart_force_gains_[endeff].pos_y.d_gain;
      point.point_.positions[start_index_cartesian_force_gains_ + e*NUM_CARTESIAN_FORCE_GAINS + FORCE_Z_PGAIN]  = data_->cart_force_gains_[endeff].pos_z.p_gain;
      point.point_.positions[start_index_cartesian_force_gains_ + e*NUM_CARTESIAN_FORCE_GAINS + FORCE_Z_IGAIN]  = data_->cart_force_gains_[endeff].pos_z.i_gain;
      point.point_.positions[start_index_cartesian_force_gains_ + e*NUM_CARTESIAN_FORCE_GAINS + FORCE_Z_DGAIN]  = data_->cart_force_gains_[endeff].pos_z.d_gain;
      point.point_.positions[start_index_cartesian_force_gains_ + e*NUM_CARTESIAN_FORCE_GAINS + TORQUE_X_PGAIN] = data_->cart_force_gains_[endeff].rot_x.p_gain;
      point.point_.positions[start_index_cartesian_force_gains_ + e*NUM_CARTESIAN_FORCE_GAINS + TORQUE_X_IGAIN] = data_->cart_force_gains_[endeff].rot_x.i_gain;
      point.point_.positions[start_index_cartesian_force_gains_ + e*NUM_CARTESIAN_FORCE_GAINS + TORQUE_X_DGAIN] = data_->cart_force_gains_[endeff].rot_x.d_gain;
      point.point_.positions[start_index_cartesian_force_gains_ + e*NUM_CARTESIAN_FORCE_GAINS + TORQUE_Y_PGAIN] = data_->cart_force_gains_[endeff].rot_y.p_gain;
      point.point_.positions[start_index_cartesian_force_gains_ + e*NUM_CARTESIAN_FORCE_GAINS + TORQUE_Y_IGAIN] = data_->cart_force_gains_[endeff].rot_y.i_gain;
      point.point_.positions[start_index_cartesian_force_gains_ + e*NUM_CARTESIAN_FORCE_GAINS + TORQUE_Y_DGAIN] = data_->cart_force_gains_[endeff].rot_y.d_gain;
      point.point_.positions[start_index_cartesian_force_gains_ + e*NUM_CARTESIAN_FORCE_GAINS + TORQUE_Z_PGAIN] = data_->cart_force_gains_[endeff].rot_z.p_gain;
      point.point_.positions[start_index_cartesian_force_gains_ + e*NUM_CARTESIAN_FORCE_GAINS + TORQUE_Z_IGAIN] = data_->cart_force_gains_[endeff].rot_z.i_gain;
      point.point_.positions[start_index_cartesian_force_gains_ + e*NUM_CARTESIAN_FORCE_GAINS + TORQUE_Z_DGAIN] = data_->cart_force_gains_[endeff].rot_z.d_gain;
    }
  }

  if (control_cartesian_null_space_)
  {
    for (unsigned int i=0; i<control_cartesian_null_space_joints_.size(); ++i)
    {
      int sl_joint = control_cartesian_null_space_joints_[i];
      int local_joint = start_index_cartesian_null_space_ + i;
      point.point_.positions[local_joint]     = data_->joint_des_posture_[sl_joint].th;
      point.point_.velocities[local_joint]    = data_->joint_des_posture_[sl_joint].thd;
      point.point_.accelerations[local_joint] = data_->joint_des_posture_[sl_joint].thdd;
    }
  }

  if (control_joints_)
  {
    for (unsigned int i=0; i<control_joints_joints_.size(); ++i)
    {
      int sl_joint = control_joints_joints_[i];
      int local_joint = start_index_joints_ + i;
      point.point_.positions[local_joint]     = data_->joint_des_state_[sl_joint].th;
      point.point_.velocities[local_joint]    = data_->joint_des_state_[sl_joint].thd;
      point.point_.accelerations[local_joint] = data_->joint_des_state_[sl_joint].thdd;
    }
  }

  if (control_joint_forces_)
  {
    for (unsigned int i=0; i<control_joint_forces_joints_.size(); ++i)
    {
      int sl_joint = control_joint_forces_joints_[i];
      int local_joint = start_index_joint_forces_ + i;
      point.point_.positions[local_joint] = data_->joint_des_force_[sl_joint];
    }
  }

  if (control_joint_gains_)
  {
    for (unsigned int i=0; i<control_joint_gains_joints_.size(); ++i)
    {
      int sl_joint = control_joint_gains_joints_[i];
      int local_joint = start_index_joint_gains_ + i*NUM_JOINT_GAINS;
      point.point_.positions[local_joint + PGAIN] = data_->joint_gains_[sl_joint].p_gain;
      point.point_.positions[local_joint + IGAIN] = data_->joint_gains_[sl_joint].i_gain;
      point.point_.positions[local_joint + DGAIN] = data_->joint_gains_[sl_joint].d_gain;
    }
  }

  if (control_joint_force_gains_)
  {
    for (unsigned int i=0; i<control_joint_force_gains_joints_.size(); ++i)
    {
      int sl_joint = control_joint_force_gains_joints_[i];
      int local_joint = start_index_joint_force_gains_ + i*NUM_JOINT_GAINS;
      point.point_.positions[local_joint + PGAIN] = data_->joint_force_gains_[sl_joint].p_gain;
      point.point_.positions[local_joint + IGAIN] = data_->joint_force_gains_[sl_joint].i_gain;
      point.point_.positions[local_joint + DGAIN] = data_->joint_force_gains_[sl_joint].d_gain;
    }
  }
}

}
