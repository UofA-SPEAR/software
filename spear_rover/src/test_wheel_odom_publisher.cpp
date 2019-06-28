/**
 * Simple node to publish WheelOdom messages to test odom_tf_publisher node
 */

#include <ros/ros.h>

#include <canros/spear__drive__WheelOdom.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_wheel_odom_publisher");

  ros::NodeHandle node;

  ros::Publisher pub = node.advertise<canros::spear__drive__WheelOdom>(
      "/canros/spear/drive/WheelOdom", 1000);

  ros::Rate loopRate(10);
  while (ros::ok()) {
    canros::spear__drive__WheelOdom msg;

    // left back
    msg.wheel_id = 0;
    msg.delta = 0.2;
    pub.publish(msg);

    // left front
    msg.wheel_id = 1;
    msg.delta = 0.2;
    pub.publish(msg);

    // right back
    msg.wheel_id = 2;
    msg.delta = 0.1;
    pub.publish(msg);

    // right front
    msg.wheel_id = 3;
    msg.delta = 0.1;
    pub.publish(msg);

    ros::spinOnce();
    loopRate.sleep();
  }


  return 0;
}
