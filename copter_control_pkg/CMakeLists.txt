cmake_minimum_required(VERSION 3.0.2)
project(copter_control_pkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)


catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES copter_control_pkg
 CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(copter_control src/copter_control.cpp)

add_executable(${PROJECT_NAME}_node src/copter_control_pkg_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  copter_control
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
  copter_control
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

