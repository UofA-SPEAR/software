#include <CASE_hardware_interface/CASE_hardware_interface.h>

CASE_hardware_interface::CASEHardwareInterface* CASE;

void timerCallback(const ros::TimerEvent& e) {
    CASE->update(e);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "CASE_hardware_interface");
    ros::NodeHandle nh;
    CASE = new CASE_hardware_interface::CASEHardwareInterface(nh);

    ros::Timer timer = nh.createTimer(ros::Duration(0.1), timerCallback);

    ros::spin();

    return 0;
}
