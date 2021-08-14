#include <controller_manager/controller_manager.h>
#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <spear_msgs/DriveCommand.h>
#include <spear_msgs/DriveOdometry.h>
#include <spear_msgs/JointCommand.h>

#include <algorithm>
#include <iterator>
#include <memory>
#include <unordered_map>
#include <vector>

#include "canros_client.hpp"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/subscriber.h"

using namespace hardware_interface;

struct WheelInfo {
  WheelInfo(const std::string &link) : link(link), pos(0), vel(0), eff(0) {}
  std::string link;
  double cmd_vel;
  double pos;
  double vel;
  double eff;
};

struct ArmJointInfo {
  ArmJointInfo(const std::string &link) : link(link), pos(0), vel(0), eff(0) {}
  std::string link;
  double cmd_pos;
  double pos;
  double vel;
  double eff;
};

class RobotHWChild {
 public:
  virtual void write(const ros::Time &time, const ros::Duration &period) = 0;
  virtual void read(const ros::Time &time, const ros::Duration &period) = 0;
};

class CASEDriveHardware : public RobotHWChild {
 public:
  CASEDriveHardware(RobotHW &parent, ros::NodeHandle &nh,
                    CanrosClient &canros_client)
      : canros_client(canros_client) {
    wheel_infos = {{44, WheelInfo("joint_wheel_backleft")},
                   {45, WheelInfo("joint_wheel_frontleft")},
                   {43, WheelInfo("joint_wheel_backright")},
                   {41, WheelInfo("joint_wheel_frontright")}};
    for (auto &pair : wheel_infos) {
      auto &wheel_info = pair.second;
      const auto state_handle = JointStateHandle(
          wheel_info.link, &wheel_info.pos, &wheel_info.vel, &wheel_info.eff);
      const auto velocity_handle =
          JointHandle(state_handle, &wheel_info.cmd_vel);
      wheel_state_interface.registerHandle(state_handle);
      wheel_velocity_interface.registerHandle(velocity_handle);
    }
    parent.registerInterface(&wheel_state_interface);
    parent.registerInterface(&wheel_velocity_interface);

    canros_client.observe_actuator_status(position_observer(
        [&](const actuator_id_t id, const command_value_t angle) {
          auto it = wheel_infos.find(id);
          if (it != wheel_infos.end()) {
            it->second.pos = angle;
          }
        }));
  }
  CASEDriveHardware(CASEDriveHardware&) = delete;
  CASEDriveHardware(CASEDriveHardware&&) = delete;

  void write(const ros::Time &time, const ros::Duration &period) override {
    auto command = ActuatorArrayCommand();
    for (const auto &pair : wheel_infos) {
      const auto &actuator_id = pair.first;
      const auto &wheel_info = pair.second;
      command.add_speed(actuator_id, wheel_info.cmd_vel);
    }
    canros_client.send_actuator_commands(command.build());
  }

  void read(const ros::Time &time, const ros::Duration &period) override {}

 private:
  JointStateInterface wheel_state_interface;
  VelocityJointInterface wheel_velocity_interface;

  std::unordered_map<actuator_id_t, WheelInfo> wheel_infos;

  CanrosClient &canros_client;
};

class CASEArmHardware : public RobotHWChild {
 public:
  CASEArmHardware(RobotHW &parent, ros::NodeHandle &nh,
                  CanrosClient &canros_client)
      : canros_client(canros_client) {
    arm_joint_infos = {/*{10, ArmJointInfo("shoulder_yaw")},*/
                       {11, ArmJointInfo("shoulder_pitch")},
                       {12, ArmJointInfo("elbow_pitch")},
                       /*{13, ArmJointInfo("wrist_pitch")},*/
                       /*{14, ArmJointInfo("wrist_roll")},*/
                       /*{15, ArmJointInfo("grab")}*/};

    for (auto &pair : arm_joint_infos) {
      auto &joint_info = pair.second;
      const auto state_handle = JointStateHandle(
          joint_info.link, &joint_info.pos, &joint_info.vel, &joint_info.eff);
      const auto position_handle =
          JointHandle(state_handle, &joint_info.cmd_pos);
      arm_state_interface.registerHandle(state_handle);
      arm_position_interface.registerHandle(position_handle);
    }
    parent.registerInterface(&arm_state_interface);
    parent.registerInterface(&arm_position_interface);

    // canros_client.observe_actuator_status(position_observer(
    //     [&](const actuator_id_t id, const command_value_t angle) {
    //       auto it = arm_joint_infos.find(id);
    //       if (it != arm_joint_infos.end()) {
    //         it->second.pos = angle;
    //       }
    //     }));
  }

  void write(const ros::Time &time, const ros::Duration &period) override {
    auto command = ActuatorArrayCommand();
    for (const auto &pair : arm_joint_infos) {
      const auto &actuator_id = pair.first;
      const auto &joint_info = pair.second;
      command.add_position(actuator_id, joint_info.cmd_pos);
    }
    canros_client.send_actuator_commands(command.build());
  }

  void read(const ros::Time &time, const ros::Duration &period) override {}

 private:
  JointStateInterface arm_state_interface;
  PositionJointInterface arm_position_interface;

  std::unordered_map<actuator_id_t, ArmJointInfo> arm_joint_infos;

  CanrosClient &canros_client;
};

class CASEHardware : public RobotHW {
 public:
  CASEHardware(ros::NodeHandle &nh, CanrosClient &canros_client) {
    children = {std::make_shared<CASEDriveHardware>(*this, nh, canros_client),
                std::make_shared<CASEArmHardware>(*this, nh, canros_client)};
  }
  void write(const ros::Time &time, const ros::Duration &period) override {
    for (auto child : children) {
      child->write(time, period);
    }
  }
  void read(const ros::Time &time, const ros::Duration &period) override {
    for (auto child : children) {
      child->read(time, period);
    }
  }

 private:
  std::vector<std::shared_ptr<RobotHWChild>> children;
};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "hardware_node");
  ros::NodeHandle nh;

  if (ros::console::set_logger_level("ros.controller_manager",
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  ROS_ERROR("Create canros client");
  auto canros_client = CanrosClient(nh);

  ROS_ERROR("Create robot");
  auto robot = CASEHardware(nh, canros_client);
  ROS_ERROR("Create controller manager");
  controller_manager::ControllerManager cm(&robot, nh);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Rate rate(10);

  while (ros::ok()) {
    // ROS_ERROR("top of loop");
    const auto time = ros::Time::now();
    const auto duration = ros::Duration(0.1);
    robot.read(time, duration);
    cm.update(time, duration);
    robot.write(time, duration);
    rate.sleep();
  }
  spinner.stop();
}
