/**
 * A simple node to send a goal in x, y, w (rotation) coordinates to the
 * move_base server. Target coordinates are specified as ros params.
 *
 * Example usage:
 * $ rosrun spear_rover simple_navigation_goals_node _x:=5.1 _y:=4.2 _w:=1.0
 */

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
    MoveBaseClient;

int main(int argc, char** argv) {
  ros::init(argc, argv, "simple_navigation_goals");

  ros::NodeHandle nh("~");
  double xParam, yParam, wParam;
  if (nh.getParam("x", xParam)) {
    ROS_INFO_STREAM("Got x: " << xParam);
  } else {
    ROS_FATAL("x parameter not specified.");
    exit(1);
  }
  if (nh.getParam("y", yParam)) {
    ROS_INFO_STREAM("Got y: " << yParam);
  } else {
    ROS_FATAL("y parameter not specified.");
    exit(1);
  }
  if (nh.getParam("w", wParam)) {
    ROS_INFO_STREAM("Got w: " << wParam);
  } else {
    ROS_FATAL("w parameter not specified.");
    exit(1);
  }

  // tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // wait for the action server to come up
  while (!ac.waitForServer(ros::Duration(1.0))) {
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  ROS_INFO("move_base action server is up");

  move_base_msgs::MoveBaseGoal goal;

  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = xParam;
  goal.target_pose.pose.position.y = yParam;
  goal.target_pose.pose.orientation.w = wParam;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();
  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base reached the goal!");
  else
    ROS_INFO("The base failed to reach the goal for some reason");

  return 0;
}
