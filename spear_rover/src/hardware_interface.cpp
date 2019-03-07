#include "ros/ros.h"

#include "spear_msgs/drive_cmd.h"
#include "spear_msgs/wheel_cmd.h"
#include "spear_msgs/WheelCmdArray.h"

#include "hardware_interface/mappings.h"

#include <sstream>

ros::NodeHandle* node;
ros::Subscriber drive_sub;
ros::Publisher wheel_pub;

/** @brief Callback for receiving messages from drive controller.
 *
 * As of right now, simply splits left to both wheels and same for right.
 */
void driveControlsCallback(const spear_msgs::drive_cmd::ConstPtr& msg)
{
  ROS_INFO("Left: %d -- Right: %d", msg->left, msg->right);

  spear_msgs::WheelCmdArray wheelCommands;
  spear_msgs::WheelCmdArray wheelCommands_part2;
  spear_msgs::wheel_cmd wheelCommand;

  // Deal with left side
  wheelCommand.velocity = msg->left;

  wheelCommand.wheel = HW_IF_WHEEL_LEFT_1;
  wheelCommands.wheel_cmds.push_back(wheelCommand);
  ROS_INFO("Left wheel %d: %f", wheelCommand.wheel, wheelCommand.velocity);

  wheelCommand.wheel = HW_IF_WHEEL_LEFT_2;
  wheelCommands.wheel_cmds.push_back(wheelCommand);
  ROS_INFO("Left wheel %d: %f", wheelCommand.wheel, wheelCommand.velocity);

  // Deal with right side
  // Hacky fix
  wheelCommand.velocity = msg->right;

  wheelCommand.wheel = HW_IF_WHEEL_RIGHT_1;
  wheelCommands_part2.wheel_cmds.push_back(wheelCommand);
  ROS_INFO("Right wheel %d: %f", wheelCommand.wheel, wheelCommand.velocity);

  wheelCommand.wheel = HW_IF_WHEEL_RIGHT_2;
  wheelCommands_part2.wheel_cmds.push_back(wheelCommand);
  ROS_INFO("Right wheel %d: %f", wheelCommand.wheel, wheelCommand.velocity);

  // Currently there is a firmware issue on the drive board
  // It will ignore/drop/whatever the third command
  // so we have to split four commands into two arrays
  // Temporary fix until the firmware is debugged
  wheel_pub.publish(wheelCommands);
  wheel_pub.publish(wheelCommands_part2);
}

int main(int argc, char** argv)
{
  // Initialise ROS node
  ros::init(argc, argv, "hardware_interface");
  node = new ros::NodeHandle;

  // Initialise publishers and subscribers.
  drive_sub = node->subscribe("/drive", 100, driveControlsCallback);
  wheel_pub = node->advertise<spear_msgs::WheelCmdArray>("/hw_interface/drive", 100);

  // Infinite Loop
  ros::spin();

  return 0;
}
