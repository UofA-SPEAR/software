/**
 * Subscribes to geometry_msgs/Twist messages and publishes ArrayCommand
 * messages to canros.
 */

#include <canros/uavcan__equipment__actuator__ArrayCommand.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

#include "skidsteer.hpp"

ros::Publisher pub;
ros::Subscriber sub;

void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& twistMsg) {
  skidsteer_t skidsteer = twist_to_skidsteer(*twistMsg);

  canros::uavcan__equipment__actuator__ArrayCommand outMsg;
  canros::uavcan__equipment__actuator__Command cmd;
  cmd.command_type = 3;

  // left back
  cmd.actuator_id = 2;
  cmd.command_value = skidsteer.left.back;
  outMsg.commands.push_back(cmd);

  // left front
  cmd.actuator_id = 3;
  cmd.command_value = skidsteer.left.front;
  outMsg.commands.push_back(cmd);

  // right back
  cmd.actuator_id = 0;
  cmd.command_value = skidsteer.right.back;
  outMsg.commands.push_back(cmd);

  // right front
  cmd.actuator_id = 1;
  cmd.command_value = skidsteer.right.front;
  outMsg.commands.push_back(cmd);

  pub.publish(outMsg);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "twist_to_skidsteer");

  ros::NodeHandle node;

  sub = node.subscribe("/rover_diff_drive_controller/cmd_vel", 1000,
                       cmd_vel_callback);
  pub = node.advertise<canros::uavcan__equipment__actuator__ArrayCommand>(
      "/drive/cmds", 1000);

  ros::spin();

  return 0;
}
