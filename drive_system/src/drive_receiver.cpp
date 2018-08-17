#include "ros/ros.h"
#include "drive_system/drive_cmd.h"

void driveCallback(const drive_system::drive_cmd::ConstPtr& msg)
{
  ROS_INFO("[Driving]: (L: %d R: %d)", msg->left, msg->right);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/receive_drive", 10, driveCallback);

  ros::spin();

  return 0;
}
