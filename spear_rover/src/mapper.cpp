#include "mapper.h"

// Needs to be usable by all callbacks
// I really dislike this solution.
std::unique_ptr<ros::NodeHandle> nh;
std::unique_ptr<UavcanMapper> m;

/* -------- ROS to CAN Callbacks -------- */
/// Example callback
void ros2can_drive_cb(const spear_msgs::drive_command::ConstPtr& msg) {
  // Transfer ID must monotonically increase
  static uint8_t transfer_id = 0;

  ROS_INFO("<Drive Command> id: %d, speed: %f", msg->id, msg->speed);

  // Business conversion logic, in this case it's a 1:1 mapping so very simple.
  spear_drive_DriveCommand_1_0 cmd;
  cmd.speed = msg->speed;
  cmd.id.value = msg->id;

  // Simplest I could make a macro.
  TX_TRANSFER(spear_drive_DriveCommand_1_0, cmd, transfer_id, 0, CanardPriorityNominal);
  transfer_id++;
}

/* -------- CAN to ROS Conversions -------- */
ros::Publisher* can2ros_drive_pub;
void can2ros_drive_cb(CanardTransfer* xfer) {
  // Need to deserialize UAVCAN message and deal with the transfer
  spear_drive_DriveCommand_1_0 cmd;
  RX_TRANSFER(spear_drive_DriveCommand_1_0, xfer, cmd);

  // Here we can do whatever we actually want with the message.
  spear_msgs::drive_command out_msg;
  out_msg.id = cmd.id.value;
  out_msg.speed = cmd.speed;
  can2ros_drive_pub->publish(out_msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "uavcan_mapper");
  nh = std::make_unique<ros::NodeHandle>();

  /* -------- Subscribe to relevant ROS topics here -------- */
  ros::Subscriber _ros2can_drive_sub = nh->subscribe("/core/drive", 100, ros2can_drive_cb);

  // Start spinning for incoming ROS messages
  ros::AsyncSpinner spinner(2);
  spinner.start();

  /* -------- Create any ROS advertisers you need here -------- */
  // I don't like this, there's probably a better way.
  ros::Publisher _can2ros_drive_pub = nh->advertise<spear_msgs::drive_command>("/core/drive", 100);
  can2ros_drive_pub = &_can2ros_drive_pub;

  // Create mapper object
  CanardNodeID can_id = nh->param("node_id", 100);
  std::string can_iface = nh->param("can_interface", std::string("vcan0"));
  m = std::make_unique<UavcanMapper>(can_id, can_iface);

  /* -------- Subscribe to relevant UAVCAN broadcasts here -------- */
  Can2Ros sub{};
  sub.subject_id = 0;
  sub.callback = can2ros_drive_cb;
  m->map_can2ros(sub);

  // Spin CAN forever
  m->spin();

  ros::waitForShutdown();
  return -1;
}
