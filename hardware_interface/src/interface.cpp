#include "ros/ros.h"

#include "drive_controls/drive_cmd.h"
#include "hardware_interface/wheel_cmd.h"

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

  hardware_interface::wheel_cmd out;

  // Deal with left side
  out.velocity = msg->left;

  out.wheel = HW_IF_WHEEL_LEFT_1;
  wheel_pub.publish(out);
  ROS_INFO("Left wheel %d: %f", out.wheel, out.velocity);

  out.wheel = HW_IF_WHEEL_LEFT_2;
  wheel_pub.publish(out);
  ROS_INFO("Left wheel %d: %f", out.wheel, out.velocity);

  // Deal with right side
  out.velocity = msg->right;

  out.wheel = HW_IF_WHEEL_RIGHT_1;
  wheel_pub.publish(out);
  ROS_INFO("Right wheel %d: %f", out.wheel, out.velocity);

  out.wheel = HW_IF_WHEEL_RIGHT_2;
  wheel_pub.publish(out);
  ROS_INFO("Right wheel %d: %f", out.wheel, out.velocity);
}

int main(int argc, char** argv)
{
  // Initialise ROS node
  ros::init(argc, argv, "hardware_interface");
  node = new ros::NodeHandle;

  // Initialise publishers and subscribers.
  drive_sub = node->subscribe("/drive", 100, driveControlsCallback);
  wheel_pub = node->advertise<hardware_interface::wheel_cmd>("/hw_interface/drive", 100);

  // Infinite Loop
  ros::spin();

  return 0;
}
