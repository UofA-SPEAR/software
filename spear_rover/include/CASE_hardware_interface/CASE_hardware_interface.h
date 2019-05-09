// Defines available variables and class members
// Essentially this file is just the descriptor for what we need to re-implement
// for the "basic" functionality

#ifndef ROS_CONTROL__CASE_HARDWARE_INTERFACE_H
#define ROS_CONTROL__CASE_HARDWARE_INTERFACE_H
#include <CASE_hardware_interface/CASE_hardware.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <ros/ros.h>
#include <boost/scoped_ptr.hpp>

using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::SoftJointLimits;
using joint_limits_interface::VelocityJointSoftLimitsHandle;
using joint_limits_interface::VelocityJointSoftLimitsInterface;

namespace CASE_hardware_interface {
static const double POSITION_STEP_FACTOR = 10;
static const double VELOCITY_STEP_FACTOR = 10;

class CASEHardwareInterface : public CASE_hardware_interface::CASEHardware {
 public:
  CASEHardwareInterface(ros::NodeHandle &nh);
  ~CASEHardwareInterface();
  bool init();
  void update(const ros::TimerEvent& e);
  void read();
  void write(ros::Duration elapsed_time);

 protected:
  ros::NodeHandle nh_;
  ros::Duration elapsed_time_;
  ros::Time last_updated_;
  VelocityJointInterface velocityJointInterface;
  VelocityJointSoftLimitsInterface velocityJointSoftLimitsInterface;
  double loop_hz_;
  boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
  double p_error_, v_error_, e_error_;
};
}

#endif
