#include "ros/ros.h"

#include "drive_controls/drive_cmd.h"
#include "hardware_interface/wheel_cmd.h"
#include "hardware_interface/WheelCmdArray.h"

#include "hardware_interface/mappings.h"

#include <sstream>

ros::NodeHandle* node;
ros::Subscriber drive_sub;
ros::Publisher wheel_pub;

/** @brief Callback for receiving messages from drive controller.
 *
 * As of right now, simply splits left to both wheels and same for right.
 */
void driveControlsCallback(const drive_controls::drive_cmd::ConstPtr& msg)
{
  ROS_INFO("Left: %d -- Right: %d", msg->left, msg->right);

  hardware_interface::WheelCmdArray wheelCommands;
  hardware_interface::wheel_cmd wheelCommand;

  // Deal with left side
  wheelCommand.velocity = msg->left;

  wheelCommand.wheel = HW_IF_WHEEL_LEFT_1;
  wheelCommands.wheel_cmds.push_back(wheelCommand);
  ROS_INFO("Left wheel %d: %f", wheelCommand.wheel, wheelCommand.velocity);

  wheelCommand.wheel = HW_IF_WHEEL_LEFT_2;
  wheelCommands.wheel_cmds.push_back(wheelCommand);
  ROS_INFO("Left wheel %d: %f", wheelCommand.wheel, wheelCommand.velocity);

  // Deal with right side
  wheelCommand.velocity = msg->right;

  wheelCommand.wheel = HW_IF_WHEEL_RIGHT_1;
  wheelCommands.wheel_cmds.push_back(wheelCommand);
  ROS_INFO("Right wheel %d: %f", wheelCommand.wheel, wheelCommand.velocity);

  wheelCommand.wheel = HW_IF_WHEEL_RIGHT_2;
  wheelCommands.wheel_cmds.push_back(wheelCommand);
  ROS_INFO("Right wheel %d: %f", wheelCommand.wheel, wheelCommand.velocity);

  wheel_pub.publish(wheelCommands);
}

int main(int argc, char** argv)
{
  // Initialise ROS node
  ros::init(argc, argv, "hardware_interface");
  node = new ros::NodeHandle;

  // Initialise publishers and subscribers.
  drive_sub = node->subscribe("/drive", 100, driveControlsCallback);
  wheel_pub = node->advertise<hardware_interface::WheelCmdArray>("/hw_interface/drive", 100);

  // Infinite Loop
  ros::spin();

  return 0;
}
