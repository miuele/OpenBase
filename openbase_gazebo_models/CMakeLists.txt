cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

project(openbase_gazebo_models)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

add_custom_command(OUTPUT openbase_omni3.sdf
	COMMAND xacro ${CMAKE_CURRENT_SOURCE_DIR}/models/openbase_omni3/model.sdf.xacro
			> openbase_omni3.sdf
	DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/models/openbase_omni3/model.sdf.xacro
	)

add_custom_target(openbase_omni3 ALL DEPENDS openbase_omni3.sdf)

add_custom_command(OUTPUT simplified_omni3.sdf
	COMMAND xacro ${CMAKE_CURRENT_SOURCE_DIR}/models/simplified_omni3/model.sdf.xacro
			> simplified_omni3.sdf
	DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/models/simplified_omni3/model.sdf.xacro
	)

add_custom_target(simplified_omni3 ALL DEPENDS simplified_omni3.sdf)

add_custom_command(OUTPUT omni_wheel.sdf
	COMMAND xacro ${CMAKE_CURRENT_SOURCE_DIR}/models/omni_wheel/model.sdf.xacro
			> omni_wheel.sdf
	DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/models/omni_wheel/model.sdf.xacro
	)

add_custom_target(omni_wheel ALL DEPENDS omni_wheel.sdf)

add_custom_command(OUTPUT simple_omni_wheel.sdf
	COMMAND xacro ${CMAKE_CURRENT_SOURCE_DIR}/models/simple_omni_wheel/model.sdf.xacro
			> simple_omni_wheel.sdf
	DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/models/simple_omni_wheel/model.sdf.xacro
	)

add_custom_target(simple_omni_wheel ALL DEPENDS simple_omni_wheel.sdf)

install(
	DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/models
	DESTINATION share/${PROJECT_NAME}
	)

install(
	FILES ${CMAKE_CURRENT_BINARY_DIR}/openbase_omni3.sdf
	DESTINATION share/${PROJECT_NAME}/models/openbase_omni3
	RENAME model.sdf
	)

install(
	FILES ${CMAKE_CURRENT_BINARY_DIR}/omni_wheel.sdf
	DESTINATION share/${PROJECT_NAME}/models/omni_wheel
	RENAME model.sdf
	)

install(
	FILES ${CMAKE_CURRENT_BINARY_DIR}/simple_omni_wheel.sdf
	DESTINATION share/${PROJECT_NAME}/models/simple_omni_wheel
	RENAME model.sdf
	)

install(
	FILES ${CMAKE_CURRENT_BINARY_DIR}/simplified_omni3.sdf
	DESTINATION share/${PROJECT_NAME}/models/simplified_omni3
	RENAME model.sdf
	)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

ament_package()
