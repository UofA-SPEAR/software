#include "mapper.h"

// ROS message types
#include <std_msgs/String.h>
#include <spear_msgs/DriveCommand.h>
#include <spear_msgs/JointCommand.h>
#include <spear_msgs/DriveOdometry.h>

// UAVCAN message types
#include "spear/actuators/DriveCommand_1_0.h"
#include "spear/actuators/JointCommand_1_0.h"
#include "spear/actuators/DriveOdometry_1_0.h"

// Needs to be usable by all callbacks
// I really dislike this solution.
std::unique_ptr<ros::NodeHandle> nh;
std::unique_ptr<UavcanMapper> m;

/* -------- ROS to CAN Callbacks -------- */
void ros2can_drive_cb(const spear_msgs::DriveCommand::ConstPtr& msg) {
  // Transfer ID must monotonically increase
  static uint8_t transfer_id = 0;

  // Business conversion logic, in this case it's a 1:1 mapping so very simple.
  spear_actuators_DriveCommand_1_0 cmd;
  cmd.speed = msg->speed;
  cmd.id.value = msg->id;

  // Simplest I could make a macro.
  TX_TRANSFER(spear_actuators_DriveCommand_1_0, cmd, transfer_id, 0, CanardPriorityNominal);
  transfer_id++;
}

void ros2can_joint_cb(const spear_msgs::JointCommand::ConstPtr& msg) {
  // Transfer ID must monotonically increase
  static uint8_t transfer_id = 0;

  // Business conversion logic, in this case it's a 1:1 mapping so very simple.
  spear_actuators_JointCommand_1_0 cmd;
  cmd.angle = msg->angle;
  cmd.id.value = msg->id;

  // Simplest I could make a macro.
  // TODO decide on port IDs
  TX_TRANSFER(spear_actuators_JointCommand_1_0, cmd, transfer_id, 0, CanardPriorityNominal);
  transfer_id++;
}

/* -------- CAN to ROS Conversions -------- */
ros::Publisher* can2ros_odom_pub;
void can2ros_odom_cb(CanardTransfer* xfer) {
  // Need to deserialize UAVCAN message and deal with the transfer
  spear_actuators_DriveOdometry_1_0 cmd;
  RX_TRANSFER(spear_actuators_DriveOdometry_1_0, xfer, cmd);

  // Here we can do whatever we actually want with the message.
  spear_msgs::DriveOdometry out_msg;
  out_msg.id = cmd.id.value;
  out_msg.delta = cmd.delta;
  can2ros_odom_pub->publish(out_msg);
}

ros::Publisher* can2ros_arm_pub;
void can2ros_arm_cb(CanardTransfer* xfer) {
  // Need to deserialize UAVCAN message and deal with the transfer
  spear_actuators_JointCommand_1_0 cmd;
  RX_TRANSFER(spear_actuators_JointCommand_1_0, xfer, cmd);

  // Here we can do whatever we actually want with the message.
  spear_msgs::JointCommand out_msg;
  out_msg.id = cmd.id.value;
  out_msg.angle = cmd.angle;
  can2ros_arm_pub->publish(out_msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "uavcan_mapper");
  nh = std::make_unique<ros::NodeHandle>();

  /* -------- Subscribe to relevant ROS topics here -------- */
  ros::Subscriber _ros2can_drive_sub = nh->subscribe("/can/spear/actuators/drive_command", 100, ros2can_drive_cb);
  ros::Subscriber _ros2can_arm_sub = nh->subscribe("/can/spear/actuators/joint_command", 100, ros2can_joint_cb);

  // Start spinning for incoming ROS messages
  ros::AsyncSpinner spinner(2);
  spinner.start();

  /* -------- Create any ROS advertisers you need here -------- */
  // I don't like this, there's probably a better way.
  ros::Publisher _can2ros_odom_pub = nh->advertise<spear_msgs::DriveCommand>("/can/spear/actuators/drive_odometry", 100);
  can2ros_odom_pub = &_can2ros_odom_pub;
  ros::Publisher _can2ros_arm_pub = nh->advertise<spear_msgs::JointCommand>("/can/spear/actuators/joint_command", 100);
  can2ros_arm_pub = &_can2ros_arm_pub;

  // Create mapper object
  CanardNodeID can_id = nh->param("node_id", 100);
  std::string can_iface = nh->param("can_interface", std::string("vcan0"));
  m = std::make_unique<UavcanMapper>(can_id, can_iface);

  /* -------- Subscribe to relevant UAVCAN broadcasts here -------- */
  Can2Ros sub{};
  // TODO decide on port IDs
  sub.subject_id = 0;
  sub.callback = can2ros_odom_cb;
  m->map_can2ros(sub);
  // TODO decide on port IDs
  sub.subject_id = 1;
  sub.callback = can2ros_arm_cb;
  m->map_can2ros(sub);

  // Spin CAN forever
  m->spin();

  ros::waitForShutdown();
  return -1;
}
