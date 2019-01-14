#include "ros/ros.h"
#include "drive_controls/drive_cmd.h"

#include <sstream>

auto wheel_out;

ros::NodeHandle node;
ros::Subscriber drive_sub;
ros::Publisher wheel_pub;

/** @brief Callback for receiving messages from drive controller.
 *
 */
void driveControlsCallback(const drive_controls::drive_cmd::ConstPtr& msg) {
    ROS_INFO("Left: %d -- Right: %d", msg->left, msg->right);


}


int main(int argc, char **argv) {

    // Initialise ROS node
	ros::init(argc, argv, "hardware_interface");

    // Initialise publishers and subscribers.
    drive = node.subscribe("/drive", 100, driveControlsCallback);
    wheel_pub = node.advertise<drive_controls::drive_cmd>("/hw_interface/wheels", 100);


    // Infinite Loop
    ros::spin();

    return 0;
}
