cmake_minimum_required(VERSION 2.8.3)
project(spear_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  std_msgs
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

add_message_files(
  FILES
  BallCoord.msg
  BallCoords.msg
  arm_position.msg
  DriveCommand.msg
  JointCommand.msg
  DriveOdometry.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
 CATKIN_DEPENDS
  message_generation
  message_runtime
  std_msgs
)

