#include "mapper.h"

#include <boost/function.hpp>

// ROS message types
#include <std_msgs/String.h>
#include <spear_msgs/DriveCommand.h>
#include <spear_msgs/JointCommand.h>
#include <spear_msgs/DriveOdometry.h>

// UAVCAN message types
#include "spear/actuators/DriveCommand_1_0.h"
#include "spear/actuators/JointCommand_1_0.h"
#include "spear/actuators/DriveOdometry_1_0.h"

void ros2can_drive_cb(std::shared_ptr<UavcanMapper>& m, const spear_msgs::DriveCommand::ConstPtr& msg) {
  // Transfer ID must monotonically increase
  static uint8_t transfer_id = 0;

  // Business conversion logic, in this case it's a 1:1 mapping so very simple.
  spear_actuators_DriveCommand_1_0 cmd;
  cmd.speed = msg->speed;
  cmd.id.value = msg->id;

  // Simplest I could make a macro.
  TX_TRANSFER(m, spear_actuators_DriveCommand_1_0, cmd, transfer_id, 0, CanardPriorityNominal);
  transfer_id++;
}

void ros2can_joint_cb(std::shared_ptr<UavcanMapper>& m, const spear_msgs::JointCommand::ConstPtr& msg) {
  // Transfer ID must monotonically increase
  static uint8_t transfer_id = 0;

  // Business conversion logic, in this case it's a 1:1 mapping so very simple.
  spear_actuators_JointCommand_1_0 cmd;
  cmd.angle = msg->angle;
  cmd.id.value = msg->id;

  // Simplest I could make a macro.
  // TODO decide on port IDs
  TX_TRANSFER(m, spear_actuators_JointCommand_1_0, cmd, transfer_id, 1, CanardPriorityNominal);
  transfer_id++;
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "uavcan_mapper");
  auto nh = std::make_unique<ros::NodeHandle>();

  // Create mapper object
  CanardNodeID can_id = nh->param("node_id", 100);
  std::string can_iface = nh->param("can_interface", std::string("vcan0"));
  auto m = std::make_shared<UavcanMapper>(can_id, can_iface);

  /* -------- Subscribe to relevant ROS topics here -------- */
  //boost::function<void(const spear_msgs::DriveCommand::ConstPtr&)> ros2can_drive_cb =
  //  [&](const spear_msgs::DriveCommand::ConstPtr& msg) {
  //};
  boost::function<void(const spear_msgs::DriveCommand::ConstPtr&)> _ros2can_drive_cb = [&m](const auto msg) {
    ros2can_drive_cb(m, msg);
  };
  auto _ros2can_drive_sub = nh->subscribe("/can/spear/actuators/drive_command", 100, _ros2can_drive_cb);

  boost::function<void(const spear_msgs::JointCommand::ConstPtr&)> _ros2can_joint_cb = [&](const auto msg) {
    ros2can_joint_cb(m, msg);
  };
  auto _ros2can_arm_sub = nh->subscribe("/can/spear/actuators/joint_command", 100, _ros2can_joint_cb);

  /* -------- UAVCAN mappings -------- */
  // Map drive odometry messages from CAN into ROS.
  ros::Publisher can2ros_odom_pub = nh->advertise<spear_msgs::DriveOdometry>("/can/spear/actuators/drive_odometry", 100);
  m->map_can2ros(3, [&can2ros_odom_pub](CanardTransfer* xfer) {
    // Need to deserialize UAVCAN message and deal with the transfer
    spear_actuators_DriveOdometry_1_0 cmd;
    RX_TRANSFER(spear_actuators_DriveOdometry_1_0, xfer, cmd);

    // Here we can do whatever we actually want with the message.
    spear_msgs::DriveOdometry out_msg;
    out_msg.id = cmd.id.value;
    out_msg.delta = cmd.delta;
    can2ros_odom_pub.publish(out_msg);
  });

  // Map Arm messages from CAN into ROS.
  ros::Publisher can2ros_arm_pub = nh->advertise<spear_msgs::JointCommand>("/can/spear/actuators/joint_command", 100);
  m->map_can2ros(1, [&can2ros_arm_pub](CanardTransfer* xfer) {
    // Need to deserialize UAVCAN message and deal with the transfer
    spear_actuators_JointCommand_1_0 cmd;
    RX_TRANSFER(spear_actuators_JointCommand_1_0, xfer, cmd);

    // Here we can do whatever we actually want with the message.
    spear_msgs::JointCommand out_msg;
    out_msg.id = cmd.id.value;
    out_msg.angle = cmd.angle;
    can2ros_arm_pub.publish(out_msg);
  });

  // Start spinning for incoming ROS messages
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Spin CAN forever
  m->spin();

  ros::waitForShutdown();
  return -1;
}
