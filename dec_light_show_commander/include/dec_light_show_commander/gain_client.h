/*
 * gain_client.h
 *
 *  Created on: Jun 7, 2011
 *      Author: kalakris
 */

#ifndef GAIN_CLIENT_H_
#define GAIN_CLIENT_H_

#include <arm_controller_interface/trajectory_client.h>
#include <sl_controller_msgs/CartesianGains.h>
#include <arm_controller_msgs/FingerForceControlGains.h>

// TODO: remove this again...
#include <arm_controller_interface/arm_robot_info.h>

namespace arm_controller_interface
{

class GainClient: public TrajectoryClient
{
public:

  /*! Empty constructor, need initialize before use.
   */
  GainClient() {};

  /*!
   * @param endeffector_id
   * @param controller_name
   */
  GainClient(const int endeffector_id, const std::string& controller_name = "GainGenerator");
  virtual ~GainClient() {};

  /*!
   * @param endeffector_id
   * @param controller_name
   * @return True on success, otherwise False
   */
  bool initialize(const int endeffector_id, const std::string& controller_name = "GainGenerator");

  /*!
   * @param arm_gains
   * @param transition_duration
   * @param wait_for_success
   * @return True on success, otherwise False
   */
  bool setArmForceControlGains(const sl_controller_msgs::CartesianGains& arm_gains, double transition_duration = 1.0,
                               bool wait_for_success = true);
  bool setArmPositionControlGains(const sl_controller_msgs::CartesianGains& arm_gains, double transition_duration = 1.0,
                                  bool wait_for_success = true);
  bool setArmPositionForceControlGains(const sl_controller_msgs::CartesianGains& arm_position_gains,
                                       const sl_controller_msgs::CartesianGains& arm_force_gains,
                                       double transition_duration = 1.0, bool wait_for_success = true);
  bool setFingerForceControlGains(const arm_controller_msgs::FingerForceControlGains& finger_gains,
                                  double transition_duration = 1.0, bool wait_for_success = true);

  bool setArmForceControlGains(const std::vector<double>& p_gains, const std::vector<double>& i_gains,
                               double transition_duration = 1.0, bool wait_for_success = true);
  bool setArmPositionControlGains(const std::vector<double>& p_gains, const std::vector<double>& i_gains,
                                  double transition_duration = 1.0, bool wait_for_success = true);
  bool setArmPositionForceControlGains(const std::vector<double>& position_p_gains,
                                       std::vector<double>& position_i_gains, const std::vector<double>& force_p_gains,
                                       std::vector<double>& force_i_gains, double transition_duration = 1.0,
                                       bool wait_for_success = true);
  bool setArmPositionForceControlGains(const std::string& gain_string, double transition_duration = 1.0,
                                       bool wait_for_success = true);

  bool disableForceControlGains(double transition_duration=1.0, bool wait_for_success=true);
  bool disablePositionControlGains(double transition_duration=1.0, bool wait_for_success=true);
  bool disablePositionForceControlGains(double transition_duration=1.0, bool wait_for_success=true);
  bool disableFingerForceControlGains(double transition_duration=1.0, bool wait_for_success=true);

  bool enableForceControlGains(double transition_duration=1.0, bool wait_for_success=true);
  bool enablePositionControlGains(double transition_duration=1.0, bool wait_for_success=true);
  bool enablePositionForceControlGains(double transition_duration=1.0, bool wait_for_success=true);
  bool enableFingerForceControlGains(double transition_duration=1.0, bool wait_for_success=true);

  static bool readGains(XmlRpc::XmlRpcValue& config,
                        sl_controller_msgs::CartesianGains& position_gains,
                        sl_controller_msgs::CartesianGains& force_gains);

  static bool setGainElement(char gain_char, double& position_gain, double& force_gain);

private:
  std::vector<std::string> arm_force_control_names_;
  std::vector<std::string> arm_position_control_names_;
  std::vector<std::string> finger_force_control_names_;

  void set(sl_controller_msgs::CartesianGains& gains, const double& value);
  void setZero(sl_controller_msgs::CartesianGains& gains);
  void setOne(sl_controller_msgs::CartesianGains& gains);

  void set(arm_controller_msgs::FingerForceControlGains& gains, const double& value);
  void setZero(arm_controller_msgs::FingerForceControlGains& gains);
  void setOne(arm_controller_msgs::FingerForceControlGains& gains);

  static void setIGainsFromPGains(sl_controller_msgs::CartesianGains& gains);
  static void setDGainsFromPGains(sl_controller_msgs::CartesianGains& gains);

};

class RightHandGainClient : public GainClient
{
public:
  RightHandGainClient(const int sl_endeffector_id = ArmRobotInfo::getRightHandEndeffectorId(),
                      const std::string& controller_name = "GainGenerator") :
      GainClient(sl_endeffector_id, controller_name) {};
  virtual ~RightHandGainClient() {};
};

class LeftHandGainClient : public GainClient
{
public:
  LeftHandGainClient(const int sl_endeffector_id = ArmRobotInfo::getLeftHandEndeffectorId(),
                     const std::string& controller_name = "GainGenerator") :
      GainClient(sl_endeffector_id, controller_name) {};
  virtual ~LeftHandGainClient() {};
};

}

#endif /* GAIN_CLIENT_H_ */
