#include <CASE_hardware_interface/CASE_hardware_interface.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "CASE_hardware_interface");
    ros::NodeHandle nh;
    CASE_hardware_interface::CASEHardwareInterface CASE(nh);
    controller_manager::ControllerManager cm(&CASE);

    while (true) {
        CASE.update();
        ros::Duration sleep(0.1);
    }
    return 0;
}
