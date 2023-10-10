cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

project(gazebo_ros_indep_steer_drive)

find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)


add_library(gazebo_ros_indep_steer_drive SHARED src/gazebo_ros_indep_steer_drive.cc)

target_include_directories(gazebo_ros_indep_steer_drive PUBLIC include)

ament_target_dependencies(gazebo_ros_indep_steer_drive geometry_msgs gazebo_ros)

install(TARGETS gazebo_ros_indep_steer_drive DESTINATION lib/${PROJECT_NAME})

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

ament_package()
