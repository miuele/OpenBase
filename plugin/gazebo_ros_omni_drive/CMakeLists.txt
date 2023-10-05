cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

project(gazebo_ros_omni_drive)

find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)

#add_library(gazebo_plugin INTERFACE)
#target_include_directories(gazebo_plugin INTERFACE ${GAZEBO_INCLUDE_DIRS})
#target_link_directories(gazebo_plugin INTERFACE ${GAZEBO_LIBRARY_DIRS})
#
#target_compile_options(gazebo_plugin INTERFACE "${GAZEBO_CXX_FLAGS}")
#target_link_options(gazebo_plugin INTERFACE "${GAZEBO_CXX_FLAGS}")

add_library(gazebo_ros_omni_drive SHARED src/gazebo_ros_omni_drive.cc)
#target_link_libraries(gazebo_ros_omni_drive gazebo_plugin)
ament_target_dependencies(gazebo_ros_omni_drive geometry_msgs gazebo_ros)

install(TARGETS gazebo_ros_omni_drive DESTINATION lib/${PROJECT_NAME})

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

ament_package()
