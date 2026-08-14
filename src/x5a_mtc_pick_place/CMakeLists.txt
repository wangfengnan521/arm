cmake_minimum_required(VERSION 3.8)
project(x5a_mtc_pick_place)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_task_constructor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(x5a_task_interfaces REQUIRED)

add_executable(x5a_mtc_task_server src/x5a_mtc_task_server.cpp)
target_compile_features(x5a_mtc_task_server PUBLIC cxx_std_17)
ament_target_dependencies(x5a_mtc_task_server
  control_msgs geometry_msgs moveit_ros_planning_interface
  moveit_task_constructor_core moveit_task_constructor_msgs
  rclcpp rclcpp_action sensor_msgs shape_msgs std_msgs tf2
  tf2_geometry_msgs x5a_task_interfaces
)

install(TARGETS x5a_mtc_task_server DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
ament_package()
