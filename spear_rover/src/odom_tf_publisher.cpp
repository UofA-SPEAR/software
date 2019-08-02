/**
 * Broadcasts odom based on incoming data
 */

#include <string>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include "skidsteer.hpp"

#include <canros/spear__drive__WheelOdom.h>

ros::Publisher pub;
nav_msgs::Odometry oldOdom;

/**
 * Broadcasts a transform.
 * @param x X-coordinate of transform
 * @param y Y-coordinate of transform
 * @param z Z-coordinate of transform
 * @param roll Roll of transform
 * @param pitch Pitch of transform
 * @param yaw Yaw of transform
 * @param from Name of parent tf frame
 * @param to Name of child tf frame
 * @return The transform that was broadcast
 */
nav_msgs::Odometry broadcastOdom(double x, double y, double z, double roll,
                                 double pitch, double yaw, std::string from,
                                 std::string to) {
  nav_msgs::Odometry msg;

  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "odom";
  msg.child_frame_id = "base_link";

  msg.pose.pose.position.x = x;
  msg.pose.pose.position.y = y;
  msg.pose.pose.position.z = z;

  msg.pose.pose.orientation =
      tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

  // Broadcast msg
  pub.publish(msg);

  return msg;
}

/**
 * Adds x, y, z, roll, pitch, yaw onto an old odom message and publishes the
 * result.
 *
 * Note: modifies global variable "oldOdom"
 *
 * @param x X-coordinate to add
 * @param y Y-coordinate to add
 * @param z Z-coordinate to add
 * @param roll Roll to add
 * @param pitch Pitch to add
 * @param yaw Yaw to add
 * @param from Name of parent tf frame
 * @param to Name of child tf frame
 */
void odomDelta(double x, double y, double z, double roll, double pitch,
               double yaw, std::string from, std::string to) {
  // Calculate new origin by adding x, y, z coordinates
  double oldX = oldOdom.pose.pose.position.x;
  double oldY = oldOdom.pose.pose.position.y;
  double oldZ = oldOdom.pose.pose.position.z;
  double newX = oldX + x;
  double newY = oldY + y;
  double newZ = oldZ + z;

  // Calculate new rotation by multiplying quaternions
  // We must convert to a tf::Quaternion temporarily since a
  // geometry_msgs::Quaternion doesn not have the "*" operator.
  tf::Quaternion oldRotation;
  tf::quaternionMsgToTF(oldOdom.pose.pose.orientation, oldRotation);
  tf::Quaternion rotationDelta = tf::createQuaternionFromRPY(roll, pitch, yaw);
  tf::Quaternion newRotation = oldRotation * rotationDelta;
  // Convert back to a geometry_msgs/Quaternion
  geometry_msgs::Quaternion newRotationMsg;
  tf::quaternionTFToMsg(newRotation, newRotationMsg);

  // Broadcast new odom msg
  nav_msgs::Odometry msg;

  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "odom";
  msg.child_frame_id = "base_link";

  msg.pose.pose.position.x = newX;
  msg.pose.pose.position.y = newY;
  msg.pose.pose.position.z = newZ;

  msg.pose.pose.orientation = newRotationMsg;

  // Broadcast msg
  pub.publish(msg);

  // Update old odom message
  oldOdom = msg;
}

/*
 * WHeels are:
 * 0 - left back
 * 1 - left front
 * 2 - right back
 * 3 - right front
 */

void odom_callback(const canros::spear__drive__WheelOdom::ConstPtr& msg) {
  static canros::spear__drive__WheelOdom wheels[4];

  // We are going to assume that there are fewer than 255 wheels.
  static bool wheel_data[4] = {0};

  wheel_data[msg->wheel_id] = true;
  wheels[msg->wheel_id] = *msg;

  // Return if we aren't done with all the wheels
  for (int i = 0; i < 4; i++) {
    if (!wheel_data[i]) {
      return;
    }
  }

  skidsteer_t skid;
  skid.left.back = wheels[0].delta;
  skid.left.front = wheels[1].delta;
  skid.right.back = wheels[2].delta;
  skid.right.front = wheels[3].delta;

  // If all the data is in
  odom_delta_t odom_delta = wheel_increments_to_odom_delta(skid);

  odomDelta(odom_delta.x, odom_delta.y, 0, 0, 0, odom_delta.yaw, "odom",
            "base_link");
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "odom_tf_publisher");

  ros::NodeHandle node;
  pub = node.advertise<nav_msgs::Odometry>("/rover_diff_drive_controller/odom",
                                           1000);

  // Publish an initial odom message
  oldOdom = broadcastOdom(0, 0, 0, 0, 0, 0, "odom", "base_link");
  ros::Duration(1).sleep();

  ros::Subscriber sub =
      node.subscribe("/canros/spear/drive/WheelOdom", 1000, odom_callback);

  ros::spin();

  return 0;
}
