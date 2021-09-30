cmake_minimum_required(VERSION 2.8.3)
project(kimm_franka_controllers)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

LIST( APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}" )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure
  franka_hw
  franka_gripper
  geometry_msgs
  hardware_interface
  pluginlib
  realtime_tools
  roscpp
  rospy
  roslib
)

find_package(Eigen3 REQUIRED)
find_package(Franka 0.5.0 REQUIRED)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    controller_interface
    dynamic_reconfigure
    franka_hw
    geometry_msgs
    hardware_interface
    pluginlib
    realtime_tools
    roscpp
  DEPENDS Franka
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Franka_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
src/basic_franka_controller.cpp
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  ${PROJECT_NAME}_gencpp
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
    ${RBDL_LIBRARY}

)
#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES ${PROJECT_NAME}_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
catkin_install_python(
  PROGRAMS scripts/interactive_marker.py scripts/move_to_start.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_dyros_mobile_manipulator_controller.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
