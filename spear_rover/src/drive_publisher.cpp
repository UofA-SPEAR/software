#include <ros/ros.h>
#include <spear_msgs/drive_command.h>

void callback(spear_msgs::drive_command::ConstPtr& msg) {
    ROS_INFO("[Drive Command] id: %d, speed: %d", msg->id, msg->speed);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "drive_publisher");

    ros::NodeHandle n;

    auto publisher = n.advertise<spear_msgs::drive_command>("/core/drive", 100);
    ros::Rate rate(1); // 1Hz

    while (1) {
        spear_msgs::drive_command cmd;
        cmd.id = 1;
        cmd.speed = 42.0;
        publisher.publish(cmd);

        ros::spinOnce();
        rate.sleep();
    }
}
