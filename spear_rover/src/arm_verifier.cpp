/**
 * A simple node that does forward kinematics to verify the inverse kinematics
 * Just prints x,y,z coords to ROS_INFO
 * NOT FINISHED - Ryan
 */

#include <math.h>

#include <ros/ros.h>
#include <spear_msgs/ArmAngles.h>
#include <spear_msgs/JointAngle.h>

ros::NodeHandle* node;
ros::Subscriber arm_angles_sub;

//Placeholder values, although actual values shouldn't matter as long as the ratios are maintained (currently they are not)
const float BICEP_LENGTH = 1.0;
const float FOREARM_LENGTH = 1.0;
const float WRIST_LENGTH = 1.0;

void armAnglesCallback(const spear_msgs::ArmAngles::ConstPtr& msg) {
  float shoulderYaw = msg->joints[0].angle;
  float shoulderPitch = msg->joints[1].angle;
  float elbowPitch = msg->joints[2].angle;
  float wristRoll = msg->joints[3].angle;
  float wristPitch = msg->joints[4].angle;
  float grab = msg->joints[5].angle;

  ROS_INFO_STREAM("z: " << (float)(sin(shoulderPitch) * BICEP_LENGTH + WRIST_LENGTH * sin(wristPitch)));
}

int main (int argc, char** argv) {
  ros::init(argc, argv, "arm_verifier");
  node = new ros::NodeHandle;
  arm_angles_sub = node->subscribe("/arm/angles", 1000, armAnglesCallback);
  ros::spin();
  return 0;
}
