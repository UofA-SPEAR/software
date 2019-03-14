#include <CASE_hardware_interface/CASE_hardware_interface.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "CASE_hardware_interface");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  CASE_hardware_interface::CASEHardwareInterface CASE(nh);
  ros::spin();
  return 0;
}
