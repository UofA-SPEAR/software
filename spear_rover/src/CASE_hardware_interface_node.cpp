#include <CASE_hardware_interface/CASE_hardware.h>
#include <controller_manager/controller_manager.h>

void timerCallback(const ros::TimerEvent& e) {
    //CASE->update(e);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "CASE_hardware_interface");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle nh;
    CASEHardware CASE;
    CASE.init();

    controller_manager::ControllerManager cm(&CASE, nh);

    ros::Duration period (1.0/50); // 50Hz refresh

    while (ros::ok()) {
	CASE.read();
	cm.update(ros::Time::now(), period);
	CASE.write(period);
	period.sleep();
    }

    spinner.stop();

    return 0;
}
