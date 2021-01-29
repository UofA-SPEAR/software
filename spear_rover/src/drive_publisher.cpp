#include <ros/ros.h>
#include <spear_msgs/DriveCommand.h>

void callback(spear_msgs::DriveCommand::ConstPtr& msg) {
    ROS_INFO("[Drive Command] id: %d, speed: %d", msg->id, msg->speed);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "drive_publisher");

    ros::NodeHandle n;

    auto publisher = n.advertise<spear_msgs::DriveCommand>("/core/drive", 100);
    ros::Rate rate(1); // 1Hz

    while (1) {
        spear_msgs::DriveCommand cmd;
        cmd.id = 1;
        cmd.speed = 42.0;
        publisher.publish(cmd);

        ros::spinOnce();
        rate.sleep();
    }
}
