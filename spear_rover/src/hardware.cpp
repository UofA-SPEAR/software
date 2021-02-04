#include <spear_msgs/DriveCommand.h>
#include <spear_msgs/JointCommand.h>
#include <spear_msgs/DriveOdometry.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>

#include <boost/functional.hpp>
#include <memory>
#include <vector>

using namespace hardware_interface;

struct WheelInfo {
  WheelInfo(const std::string &link, int actuator_id)
      : link(link), actuator_id(actuator_id), pos(0), vel(0), eff(0) {}
  std::string link;
  int actuator_id;
  double cmd_vel;
  double pos;
  double vel;
  double eff;
};

struct ArmJointInfo {
  ArmJointInfo(const std::string &link, int actuator_id)
      : link(link), actuator_id(actuator_id), pos(0), vel(0), eff(0) {}
  std::string link;
  int actuator_id;
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
  CASEDriveHardware(RobotHW &parent, ros::NodeHandle &nh) {
    wheel_infos = {WheelInfo("link_wheel_backleft", 44),
                   WheelInfo("link_wheel_frontleft", 45),
                   WheelInfo("link_wheel_backright", 43),
                   WheelInfo("link_wheel_frontright", 41)};
    for (auto &wheel_info : wheel_infos) {
      const auto state_handle = JointStateHandle(
          wheel_info.link, &wheel_info.pos, &wheel_info.vel, &wheel_info.eff);
      const auto velocity_handle =
          JointHandle(state_handle, &wheel_info.cmd_vel);
      wheel_state_interface.registerHandle(state_handle);
      wheel_velocity_interface.registerHandle(velocity_handle);
    }
    parent.registerInterface(&wheel_state_interface);
    parent.registerInterface(&wheel_velocity_interface);

    wheel_commands_publisher = nh.advertise<spear_msgs::DriveCommand>(
        "/can/spear/actuators/drive_command", 1);
    boost::function<void(const spear_msgs::DriveOdometry::ConstPtr &)> odom_callback =
        [&](const spear_msgs::DriveOdometry::ConstPtr &message) { on_wheel_odom_message(message); };
    wheel_odom_subscriber =
        nh.subscribe("/can/spear/actuators/drive_odometry", 1000, odom_callback);
  }

  void write(const ros::Time &time, const ros::Duration &period) override {
    for (const auto &wheel_info : wheel_infos) {
      auto command = spear_msgs::DriveCommand();
      command.id = wheel_info.actuator_id;
      command.speed = wheel_info.cmd_vel;
      wheel_commands_publisher.publish(command);
      ROS_INFO("Command = %f", wheel_info.cmd_vel);
    }
  }

  void read(const ros::Time &time, const ros::Duration &period) override {}

 private:
  void on_wheel_odom_message(const spear_msgs::DriveOdometry::ConstPtr &message) {
    // Index is in the same order as wheel_infos are initialized
    const auto wheel_index = message->id;
    wheel_infos.at(wheel_index).pos += message->delta;
  }

  JointStateInterface wheel_state_interface;
  VelocityJointInterface wheel_velocity_interface;

  std::vector<WheelInfo> wheel_infos;

  ros::Publisher wheel_commands_publisher;
  ros::Subscriber wheel_odom_subscriber;
};

class CASEArmHardware : public RobotHWChild {
 public:
  CASEArmHardware(RobotHW &parent, ros::NodeHandle &nh) {
    arm_joint_infos = {
        ArmJointInfo("shoulder_yaw", 10), ArmJointInfo("shoulder_pitch", 11),
        ArmJointInfo("elbow_pitch", 12), ArmJointInfo("wrist_pitch", 13),
        ArmJointInfo("wrist_roll", 14)};

    for (auto &joint_info : arm_joint_infos) {
      const auto state_handle = JointStateHandle(
          joint_info.link, &joint_info.pos, &joint_info.vel, &joint_info.eff);
      const auto position_handle =
          JointHandle(state_handle, &joint_info.cmd_pos);
      arm_state_interface.registerHandle(state_handle);
      arm_position_interface.registerHandle(position_handle);
    }
    parent.registerInterface(&arm_state_interface);
    parent.registerInterface(&arm_position_interface);

    arm_commands_publisher = nh.advertise<spear_msgs::JointCommand>(
        "/can/spear/actuators/joint_command", 1);
  }

  void write(const ros::Time &time, const ros::Duration &period) override {
    for (const auto &joint_info : arm_joint_infos) {
      auto command = spear_msgs::JointCommand();
      command.id = joint_info.actuator_id;
      command.angle = joint_info.cmd_pos;
      arm_commands_publisher.publish(command);
      ROS_INFO("Command = %f", joint_info.cmd_pos);
    }
  }

  void read(const ros::Time &time, const ros::Duration &period) override {}

 private:
  std::vector<ArmJointInfo> arm_joint_infos;

  JointStateInterface arm_state_interface;
  PositionJointInterface arm_position_interface;

  ros::Publisher arm_commands_publisher;
};

class CASEHardware : public RobotHW {
 public:
  CASEHardware(ros::NodeHandle &nh) {
    children = {std::make_shared<CASEDriveHardware>(*this, nh),
                std::make_shared<CASEArmHardware>(*this, nh)};
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

  auto robot = CASEHardware(nh);
  controller_manager::ControllerManager cm(&robot, nh);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Rate rate(10);

  while (ros::ok()) {
    const auto time = ros::Time::now();
    const auto duration = ros::Duration(0.1);
    robot.read(time, duration);
    cm.update(time, duration);
    robot.write(time, duration);
    rate.sleep();
  }
  spinner.stop();
}
