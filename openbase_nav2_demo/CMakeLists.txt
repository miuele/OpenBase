cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

project(openbase_nav2_demo)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

function(xacro_generator_target TARGET_NAME SOURCE_FILE DEST_FILE)
	add_custom_command(OUTPUT "${DEST_FILE}"
		COMMAND xacro "${SOURCE_FILE}" -o "${DEST_FILE}"
		DEPENDS "${SOURCE_FILE}"
		VERBATIM
		)
	add_custom_target(${TARGET_NAME} ALL DEPENDS "${DEST_FILE}")
endfunction()

function(add_xacro_urdf_file MODEL_NAME)
	xacro_generator_target(
		${MODEL_NAME}
		"${CMAKE_CURRENT_SOURCE_DIR}/urdf/${MODEL_NAME}.urdf.xacro"
		"${CMAKE_CURRENT_BINARY_DIR}/${MODEL_NAME}.urdf"
		)

	install(
		FILES "${CMAKE_CURRENT_BINARY_DIR}/${MODEL_NAME}.urdf"
		DESTINATION "share/${PROJECT_NAME}/urdf"
		)

endfunction()

add_xacro_urdf_file(nav_omni4)

install(
	DIRECTORY urdf launch params rviz worlds
	DESTINATION share/${PROJECT_NAME}
	)

ament_package()
