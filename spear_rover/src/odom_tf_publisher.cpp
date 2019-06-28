/**
 * Outline for a node that updates the odometry transform based on incoming data
 */

#include <string>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "skidsteer.hpp"

#include <canros/spear__drive__WheelOdom.h>

tf::Transform old_transform;

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
tf::Transform broadcastTransform(double x, double y, double z,
                  double roll, double pitch, double yaw,
                  std::string from, std::string to) {
  // This must be static
  static tf::TransformBroadcaster br;

  tf::Transform transform;
  tf::Quaternion rotation;
  transform.setOrigin(tf::Vector3(x, y, z));
  rotation.setRPY(roll, pitch, yaw);
  transform.setRotation(rotation);
  // Broadcast transform
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), from, to));
  return transform;
}


/**
 * Adds x, y, z, roll, pitch, yaw onto an old transform and publishes the result.
 * @param x X-coordinate to add
 * @param y Y-coordinate to add
 * @param z Z-coordinate to add
 * @param roll Roll to add
 * @param pitch Pitch to add
 * @param yaw Yaw to add
 * @param from Name of parent tf frame
 * @param to Name of child tf frame
 * @param oldTransform The old transform. THIS IS MODIFIED.
 */
void transformDelta(double x, double y, double z,
                    double roll, double pitch, double yaw,
                    std::string from, std::string to,
                    tf::Transform &oldTransform) {
  // This must be static
  static tf::TransformBroadcaster br;

  // Calculate new origin by adding x, y, z coordinates
  double oldX = oldTransform.getOrigin().x();
  double oldY = oldTransform.getOrigin().y();
  double oldZ = oldTransform.getOrigin().z();
  double newX = oldX + x;
  double newY = oldY + y;
  double newZ = oldZ + z;

  // Calculate new rotation by multiplying quaternions
  tf::Quaternion oldRotation = oldTransform.getRotation();
  tf::Quaternion rotationDelta;
  rotationDelta.setRPY(roll, pitch, yaw);
  tf::Quaternion newRotation = oldRotation * rotationDelta;

  // Broadcast new transform
  tf::Transform newTransform;
  newTransform.setOrigin(tf::Vector3(newX, newY, newZ));
  newTransform.setRotation(newRotation);
  br.sendTransform(tf::StampedTransform(newTransform, ros::Time::now(), from, to));

  // Update old transform
  oldTransform = newTransform;
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
    static bool wheel_data[4] = { 0 };

    wheel_data[msg->wheel_id] = true;
    wheels[msg->wheel_id] = *msg;

    // Return if we aren't done with all the wheels
    for (int i = 0; i < 4; i++) {
        if (!wheel_data[i]) {
            return;
        }
    }

    skidsteer_t skid;
    skid.left.back   = wheels[0].delta;
    skid.left.front  = wheels[1].delta;
    skid.right.back  = wheels[2].delta;
    skid.right.front = wheels[3].delta;

    // If all the data is in
    tf_delta_t tf_delta = odom_to_tf_delta(skid);

    transformDelta(tf_delta.x, tf_delta.y, 0, tf_delta.yaw, 0, 0,
            "odom", "base_link", old_transform);

}


int main(int argc, char** argv) {
  ros::init(argc, argv, "odom_tf_publisher");

  ros::NodeHandle node;

  old_transform = broadcastTransform(0, 0, 0, 0, 0, 0, "odom", "base_link");
  ros::Duration(1).sleep();

  ros::Subscriber sub =
      node.subscribe("/canros/spear/drive/WheelOdom", 1000, odom_callback);

  ros::spin();

  return 0;
}
