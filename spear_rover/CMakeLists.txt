cmake_minimum_required(VERSION 2.8.3)
project(spear_rover)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    cv_bridge
    image_transport
    canros
    std_msgs
    spear_msgs
)

catkin_python_setup()



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
  CATKIN_DEPENDS message_runtime std_msgs spear_msgs message_generation
#  DEPENDS system_lib
)

##########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
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

add_executable(arm_ik_node src/arm_ik.cpp)
add_dependencies(arm_ik_node
   ${${PROJECT_NAME}_EXPORTED_TARGETS}
   ${catkin_EXPORTED_TARGETS}
)
## Specify libraries to link a library or executable target against
target_link_libraries(arm_ik_node
  ${catkin_LIBRARIES}
)

add_executable(arm_verifier_node src/arm_verifier.cpp)
add_dependencies(arm_verifier_node
   ${${PROJECT_NAME}_EXPORTED_TARGETS}
   ${catkin_EXPORTED_TARGETS}
)
## Specify libraries to link a library or executable target against
target_link_libraries(arm_verifier_node
  ${catkin_LIBRARIES}
)

# Add hardware interface node
add_executable(hardware_interface_node src/hardware_interface.cpp)
add_dependencies(arm_verifier_node
   ${${PROJECT_NAME}_EXPORTED_TARGETS}
   ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(hardware_interface_node
  ${catkin_LIBRARIES}
)

install(PROGRAMS
    nodes/mapper.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
