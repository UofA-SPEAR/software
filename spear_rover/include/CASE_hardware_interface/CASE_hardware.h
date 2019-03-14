// File to define hardware interfaces for CASE
// Only defines the available interfaces + variables for joint initialization
// Essentially defines what we need for our specific hardware interface

#ifndef ROS_CONTROL__CASE_HARDWARE_H
#define ROS_CONTROL__CASE_HARDWARE_H

#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <ros/ros.h>
#include <boost/scoped_ptr.hpp>

namespace CASE_hardware_interface {
class CASEHardware : public hardware_interface::RobotHW {
 protected:
  // Interfaces
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::VelocityJointInterface velocity_joint_interface_;
  hardware_interface::PositionJointInterface position_joint_interface_;
  hardware_interface::EffortJointInterface effort_joint_interface_;

  joint_limits_interface::VelocityJointSaturationInterface
      velocity_joint_saturation_interface_;
  joint_limits_interface::VelocityJointSoftLimitsInterface
      velocity_joint_limits_interface_;

  // Custom or available transmissions
  //

  // Shared memory
  int num_joints_;
  int joint_mode_;  // position, velocity, or effort
  std::vector<std::string> joint_names_;
  std::vector<int> joint_types_;
  std::vector<double> joint_velocity_;
  std::vector<double> joint_velocity_command_;
  std::vector<double> joint_lower_limits_;
  std::vector<double> joint_upper_limits_;
  std::vector<double> joint_effort_limits_;
  // Required for joint state handle
  std::vector<double> joint_position_;
  std::vector<double> joint_position_command_;
  std::vector<double> joint_effort_;
  std::vector<double> joint_effort_command_;

  // Publisher for sending CAN commands
  ros::Publisher wheel_pub;
};  // class

};  // namespace

#endif
