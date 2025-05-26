cmake_minimum_required(VERSION 3.8)
project(arm_ctrl)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(humanoid_interfaces REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(gazebo_ros2_control REQUIRED)
find_package(gazebo_ros REQUIRED)

# Message generation
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArmCtrlCmd.msg"
  "msg/ArmCtrlFlag.msg"
  "msg/ArmJointAngle.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs
)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

include_directories(
  include/
  ${humanoid_interfaces_INCLUDE_DIRS}
)

# IK and trajectory planner libraries
add_library(ik_module src/ik_module.cpp)
ament_target_dependencies(ik_module rclcpp Eigen3)

add_library(trajectory_planner src/trajectory_planner.cpp)
ament_target_dependencies(trajectory_planner rclcpp Eigen3)

# Main executable
add_executable(main_node src/main_node.cpp)
target_link_libraries(main_node ik_module trajectory_planner)
ament_target_dependencies(main_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  humanoid_interfaces
  controller_manager
  hardware_interface
  pluginlib
  gazebo_ros2_control
  gazebo_ros
)

# ROS2 interface linking
rosidl_target_interfaces(main_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install executables and libraries
install(TARGETS
  main_node
  ik_module
  trajectory_planner
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

# Install shared resources
install(DIRECTORY include/
  DESTINATION include/
)

# Install message definitions
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
