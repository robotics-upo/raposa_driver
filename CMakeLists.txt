cmake_minimum_required(VERSION 2.8.3)
project(raposa_driver)



set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
  robot_state_publisher
  arduimu_v3
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)


# Generate .world files from .world.xacro files
find_package(arduimu_v3 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
# LIBRARIES raposa_driver
 CATKIN_DEPENDS nav_msgs roscpp sensor_msgs std_msgs tf arduimu_v3
 DEPENDS Eigen3
)

###########
## Build ##
###########

# executables
include_directories( include/  ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${ARDUIMU_V3_INCLUDE_DIRS})

add_executable(raposa_node src/raposa_node.cpp)
add_executable(raposa_teleop_joy src/raposa_teleop_joy.cpp)
add_executable(raposa_calibration src/raposa_calibration_node.cpp)

target_link_libraries(raposa_node
   ${catkin_LIBRARIES}
)

target_link_libraries(raposa_teleop_joy
   ${catkin_LIBRARIES}
)

target_link_libraries(raposa_calibration
   ${catkin_LIBRARIES}
)
