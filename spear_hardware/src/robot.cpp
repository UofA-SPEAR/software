
#include <canros/uavcan__equipment__actuator__ArrayCommand.h>
#include <canros/uavcan__equipment__actuator__Command.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>

#include <vector>

using ActuatorArrayCommand = canros::uavcan__equipment__actuator__ArrayCommand;
using ActuatorCommand = canros::uavcan__equipment__actuator__Command;

using namespace hardware_interface;

struct WheelInfo {
  WheelInfo(const std::string &link, int actuator_id)
      : link(link), actuator_id(actuator_id), pos(0), vel(0), eff(0) {}
  std::string link;
  int actuator_id;
  double cmd;
  double pos;
  double vel;
  double eff;
};

class CASEHardware : public RobotHW {
 public:
  CASEHardware(ros::NodeHandle &nh) {
    wheel_infos = {WheelInfo("link_wheel_backleft", 44),
                   WheelInfo("link_wheel_frontleft", 45),
                   WheelInfo("link_wheel_backright", 43),
                   WheelInfo("link_wheel_frontright", 41)};
    for (auto &wheel_info : wheel_infos) {
      const auto state_handle = JointStateHandle(
          wheel_info.link, &wheel_info.pos, &wheel_info.vel, &wheel_info.eff);
      const auto velocity_handle = JointHandle(state_handle, &wheel_info.cmd);
      wheel_state_interface.registerHandle(state_handle);
      wheel_velocity_interface.registerHandle(velocity_handle);
    }
    registerInterface(&wheel_state_interface);
    registerInterface(&wheel_velocity_interface);

    wheel_commands_publisher =
        nh.advertise<ActuatorArrayCommand>("/drive/cmds", 1);
  }

  void write(const ros::Time &time, const ros::Duration &period) override {
    auto wheel_commands = ActuatorArrayCommand();
    for (const auto &wheel_info : wheel_infos) {
      auto command = ActuatorCommand();
      command.actuator_id = wheel_info.actuator_id;
      command.command_type = ActuatorCommand::COMMAND_TYPE_SPEED;
      command.command_value = wheel_info.cmd;
      wheel_commands.commands.push_back(command);
      ROS_INFO("Command = %f", wheel_info.cmd);
    }
    wheel_commands_publisher.publish(wheel_commands);
  }

  void read(const ros::Time &time, const ros::Duration &period) override {}

 private:
  JointStateInterface wheel_state_interface;
  VelocityJointInterface wheel_velocity_interface;

  std::vector<WheelInfo> wheel_infos;

  ros::Publisher wheel_commands_publisher;
};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "hardware");
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