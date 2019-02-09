#include "ros/ros.h"
#include "drive_controls/drive_cmd.h"

void driveCallback(const drive_controls::drive_cmd::ConstPtr& msg)
{
  ROS_INFO("[Driving]: (L: %d R: %d)", msg->left, msg->right);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "drive_receiver");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/drive", 10, driveCallback);

  ros::spin();

  return 0;
}
