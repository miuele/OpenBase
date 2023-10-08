cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

project(gazebo_ros_wheel_odometry)

find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(gazebo_ros_wheel_odometry SHARED src/gazebo_ros_wheel_odometry.cc)

ament_target_dependencies(gazebo_ros_wheel_odometry
	nav_msgs
	geometry_msgs
	gazebo_ros
	tf2
	tf2_geometry_msgs
	tf2_ros
	)

install(TARGETS gazebo_ros_wheel_odometry DESTINATION lib/${PROJECT_NAME})

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

ament_package()
