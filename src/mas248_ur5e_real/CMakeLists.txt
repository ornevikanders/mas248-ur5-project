cmake_minimum_required(VERSION 3.8)
project(mas248_ur5e_real)
find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(CANONICAL_XACRO "${CMAKE_CURRENT_SOURCE_DIR}/../mas248_ur5e/urdf/ur5e_workcell.urdf.xacro")
set(DERIVED_XACRO "${CMAKE_CURRENT_BINARY_DIR}/urdf/ur5e_workcell_real.urdf.xacro")
set(DERIVED_CONFIG "${CMAKE_CURRENT_BINARY_DIR}/config/ur5e")
if(NOT EXISTS "${CANONICAL_XACRO}")
  message(FATAL_ERROR "Expected MAS248 Lecture 8 workcell: ${CANONICAL_XACRO}")
endif()
# Re-run CMake (and the generator below) automatically when the workcell,
# the generator or the hardware defaults change. Without this an edited
# workcell is silently ignored until a forced reconfigure.
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  "${CANONICAL_XACRO}"
  "${CMAKE_CURRENT_SOURCE_DIR}/scripts/prepare_description.py"
  "${CMAKE_CURRENT_SOURCE_DIR}/config/real_robot_defaults.yaml")
# This generator writes ONLY into build/, never to any Lecture 8/9 source.
execute_process(
  COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/scripts/prepare_description.py"
          --source "${CANONICAL_XACRO}" --output "${DERIVED_XACRO}"
          --config-output "${DERIVED_CONFIG}"
  RESULT_VARIABLE GENERATION_CODE
  OUTPUT_VARIABLE GENERATION_OUT ERROR_VARIABLE GENERATION_ERR)
if(NOT GENERATION_CODE EQUAL 0)
  message(FATAL_ERROR "Cannot generate Lecture 10 workcell: ${GENERATION_ERR}")
endif()
message(STATUS "${GENERATION_OUT}")
# Source configuration is visible inside the ZIP; generated configs are vendor files.
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/config/" DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/config/" DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/urdf" "${CMAKE_CURRENT_SOURCE_DIR}/launch"
        DESTINATION share/${PROJECT_NAME})
# The handed-out ZIP had an empty scripts/ folder. validate_description.py,
# validate_stack.py, run_picture_real.py and preflight.py do not exist yet;
# add them here when they do. The launch files already validate IP,
# calibration, workcell links and the real ros2_control plugin.
ament_package()
