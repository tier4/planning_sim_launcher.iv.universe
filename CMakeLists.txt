cmake_minimum_required(VERSION 2.8.3)

project(planning_simulator_launcher)

find_package(catkin REQUIRED COMPONENTS rospy)

catkin_package(CATKIN_DEPENDS rospy)

# ==== Install Python Scripts ==================================================

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/${PROJECT_NAME}/scenario_loader.py.cmake
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/${PROJECT_NAME}/scenario_loader.py)

file(
  GLOB ${PROJECT_NAME}_SCRIPTS
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/${PROJECT_NAME}/*.py)

catkin_install_python(
  PROGRAMS ${${PROJECT_NAME}_SCRIPTS}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# ==== Generate Default scenario_database.json =================================

file(
  GLOB ${PROJECT_NAME}_DEFAULT_SCENARIOS_GLOB
  ${CMAKE_CURRENT_SOURCE_DIR}/scenario/debug/*.yaml)

function(convert reference delimiter)
  list(GET ARGV 2 RESULT)
  math(EXPR LENGTH "${ARGC}-1")
  foreach(INDEX RANGE 3 ${LENGTH})
    list(GET ARGV ${INDEX} EACH)
    set(RESULT "${RESULT}\",\n        \"${EACH}")
  endforeach()
  set(${reference} "${RESULT}" PARENT_SCOPE)
endfunction()

convert(${PROJECT_NAME}_DEFAULT_SCENARIOS ${${PROJECT_NAME}_DEFAULT_SCENARIOS_GLOB})

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/scenario_database_template.json
  ${CMAKE_CURRENT_SOURCE_DIR}/scenario_database_autoconf.json)

# install(
#   FILES ${CMAKE_CURRENT_SOURCE_DIR}/scenario_database.json
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# ==== Miscallenous ============================================================

install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# install(
#   DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/rviz
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#   )
