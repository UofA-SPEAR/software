cmake_minimum_required(VERSION 2.8.3)
project(spear_rover)

## Compile as C++14
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    actionlib
    canros
    controller_manager
    cv_bridge
    geometry_msgs
    hardware_interface
    image_transport
    message_generation
    message_runtime
    move_base
    robot_localization
    roscpp
    rospy
    sensor_msgs
    spear_msgs
    std_msgs
    tf
)

catkin_python_setup()


add_service_files(FILES GpsToFrame.srv)
generate_messages(DEPENDENCIES
  geometry_msgs
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rover2
  CATKIN_DEPENDS
  actionlib
  canros
  controller_manager
  cv_bridge
  geometry_msgs
  hardware_interface
  image_transport
  message_generation
  message_runtime
  move_base
  robot_localization
  roscpp
  rospy
  sensor_msgs
  spear_msgs
  std_msgs
  tf
#  DEPENDS system_lib
)

##########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/rover2.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(ball_detector_node src/ball_detector.cpp)
add_dependencies(ball_detector_node
   ${${PROJECT_NAME}_EXPORTED_TARGETS}
   ${catkin_EXPORTED_TARGETS}
)
## Specify libraries to link a library or executable target against
target_link_libraries(ball_detector_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)


add_executable(ball_drawer_node src/ball_drawer.cpp)
add_dependencies(ball_drawer_node
   ${${PROJECT_NAME}_EXPORTED_TARGETS}
   ${catkin_EXPORTED_TARGETS}
)
## Specify libraries to link a library or executable target against
target_link_libraries(ball_drawer_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# Add simple_navigation_goals node
add_executable(simple_navigation_goals_node src/simple_navigation_goals.cpp)
add_dependencies(simple_navigation_goals_node
   ${${PROJECT_NAME}_EXPORTED_TARGETS}
   ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(simple_navigation_goals_node
  ${catkin_LIBRARIES}
)

add_executable(hardware_node src/hardware.cpp)
add_dependencies(hardware_node
   ${${PROJECT_NAME}_EXPORTED_TARGETS}
   ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(hardware_node
  ${catkin_LIBRARIES}
)


install(PROGRAMS
    nodes/move_with_twist.py
    nodes/drive_publisher.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(gps_to_frame_server src/gps_to_frame_server.cpp)
target_link_libraries(gps_to_frame_server ${catkin_LIBRARIES})
add_dependencies(gps_to_frame_server ${PROJECT_NAME}_gencpp
   ${${PROJECT_NAME}_EXPORTED_TARGETS}
   ${catkin_EXPORTED_TARGETS}
)

#### UAVCAN Mapper Node
add_executable(mapper_node
    src/mapper.cpp
    src/mapper_helpers.cpp
    libcanard/libcanard/canard.c
)
target_include_directories(mapper_node
    PUBLIC
    libcanard/libcanard
    dsdl/
    dsdl/nunavut
)
target_link_libraries(mapper_node
    ${catkin_LIBRARIES}
)


############ Temp Testing stuff
add_executable(drive_publisher
    src/drive_publisher.cpp
)
target_link_libraries(drive_publisher ${catkin_LIBRARIES})


add_executable(can_sender
    src/can_sender.cpp
    libcanard/libcanard/canard.c
)
# TODO how the hell do I add the include directories
target_include_directories(can_sender
    PUBLIC
    libcanard/libcanard/
    dsdl/
    dsdl/nunavut
)
target_link_libraries(can_sender
    ${catkin_LIBRARIES}
)
