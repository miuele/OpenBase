cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

project(openbase_gazebo_models)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

function(xacro_generator_target TARGET_NAME SOURCE_FILE DEST_FILE DEPENDS_FILE)
	add_custom_command(OUTPUT "${DEST_FILE}"
		COMMAND xacro "${SOURCE_FILE}" > "${DEST_FILE}"
		DEPENDS "${SOURCE_FILE}" ${DEPENDS_FILE}
		VERBATIM
		)
	add_custom_target(${TARGET_NAME} ALL DEPENDS "${DEST_FILE}")
endfunction()

function(add_xacro_model_file MODEL_NAME)
	if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/models/${MODEL_NAME}/parts.sdf.xacro")
		xacro_generator_target(
			${MODEL_NAME}
			"${CMAKE_CURRENT_SOURCE_DIR}/models/${MODEL_NAME}/model.sdf.xacro"
			"${CMAKE_CURRENT_BINARY_DIR}/${MODEL_NAME}.sdf"
			"${CMAKE_CURRENT_SOURCE_DIR}/models/${MODEL_NAME}/parts.sdf.xacro"
			)
	else()
		xacro_generator_target(
			${MODEL_NAME}
			"${CMAKE_CURRENT_SOURCE_DIR}/models/${MODEL_NAME}/model.sdf.xacro"
			"${CMAKE_CURRENT_BINARY_DIR}/${MODEL_NAME}.sdf"
			""
			)
	endif()

	install(
		FILES "${CMAKE_CURRENT_BINARY_DIR}/${MODEL_NAME}.sdf"
		DESTINATION "share/${PROJECT_NAME}/models/${MODEL_NAME}"
		RENAME model.sdf
		)

endfunction()

add_xacro_model_file(openbase_omni3)
add_xacro_model_file(omni_wheel)
add_xacro_model_file(simple_omni_wheel)
add_xacro_model_file(simplified_omni3)
add_xacro_model_file(damped_omni3)
add_xacro_model_file(damped_omni4)
add_xacro_model_file(nav_omni4)
add_xacro_model_file(unicycle_wheel)
add_xacro_model_file(indep_steer)

add_xacro_model_file(primitive_omni_wheel)
add_xacro_model_file(omni4)
add_xacro_model_file(multi_laser)

install(
	DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/models
	DESTINATION share/${PROJECT_NAME}
	)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

ament_package()
