// Actually implement hardware nclude <pluginlib/class_list_macros.hpp>
// interface
//

#include <CASE_hardware_interface/CASE_hardware.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <pluginlib/class_list_macros.hpp>
#include <sstream>

// message includes
#include <canros/uavcan__equipment__actuator__ArrayCommand.h>
using namespace canros;

using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::SoftJointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;

CASEHardware::CASEHardware() {
  wheel_pub = nh_.advertise<uavcan__equipment__actuator__ArrayCommand>(
      "/canros/msg/uavcan/equipment/actuator/ArrayCommand", 10);
}

CASEHardware::~CASEHardware() {}

// Returns#include <pluginlib/class_list_macros.hpp>
// 1 if initialization success
bool CASEHardware::init() {
  // Get joint names
  nh_.getParam("/CASE/hardware_interface/joints", joint_names_);
  num_joints_ = joint_names_.size();

  // Resize vectors
  joint_velocity_.resize(num_joints_);
  joint_velocity_command_.resize(num_joints_);

  // These are only here for the joint state handle
  joint_position_.resize(num_joints_);
  joint_position_command_.resize(num_joints_);
  joint_effort_.resize(num_joints_);
  joint_effort_command_.resize(num_joints_);

  // Initialize Controller
  // Need to
  for (int i = 0; i < num_joints_; ++i) {
      ROS_INFO("Loading joint %d", i);
    // Create joint state interface
    JointStateHandle jointStateHandle(joint_names_[i], &joint_position_[i],
                                      &joint_velocity_[i], &joint_effort_[i]);
    joint_state_interface_.registerHandle(jointStateHandle);

    // Create joint velocity interface
    JointHandle jointVelocityHandle(jointStateHandle,
                                    &joint_velocity_command_[i]);
    velocity_joint_interface_.registerHandle(jointVelocityHandle);
  }

  registerInterface(&joint_state_interface_);
  registerInterface(&velocity_joint_interface_);

  // We also need to do any hardware init here (if we have any)
}


void CASEHardware::read() {
  for (int i = 0; i < num_joints_; i++) {
    // Read in all values from hardware
  }
}

void CASEHardware::write(ros::Duration elapsed_time) {

  // Write ArrayCommands to wheels
  uavcan__equipment__actuator__ArrayCommand out_msg;
  for (int i = 0; i < num_joints_; i++) {
    uavcan__equipment__actuator__Command out_cmd;

    out_cmd.actuator_id = i;
    out_cmd.command_type =
        uavcan__equipment__actuator__Command::COMMAND_TYPE_SPEED;
    // mapping a double to a float here but eh
    out_cmd.command_value =
        velocity_joint_interface_.getHandle(joint_names_[i]).getCommand();

    out_msg.commands.push_back(out_cmd);
  }

  //wheel_pub.publish(out_msg);
}

PLUGINLIB_EXPORT_CLASS(CASEHardware, hardware_interface::RobotHW)
