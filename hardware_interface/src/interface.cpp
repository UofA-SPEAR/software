#include "ros/ros.h"
#include "drive_controls/drive_cmd.h"

#include <sstream>o
void driveCallback(const drive_controls::drive_cmd::ConstPtr& msg) {
    ROS_INFO("Left: %d -- Right: %d", msg->left, msg->right);
}


int main(int argc, char **argv) {

	ros::init(argc, argv, "hardware_interface");

    ros::NodeHandle node;

    ros::Subscriber drive = node.subscribe("/drive", 100, driveCallback);

    ros::spin();

    return 0;
}
