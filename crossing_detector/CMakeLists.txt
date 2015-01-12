cmake_minimum_required(VERSION 2.8.3)
project(crossing_detector)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  angles
  geometry_msgs
  lama_common
  lama_msgs
  nav_msgs
  rosbag
  roscpp
  sensor_msgs
  )

## System dependencies are found with CMake's conventions
find_package(CGAL REQUIRED COMPONENTS Core)
set(CGAL_DONT_OVERRIDE_CMAKE_FLAGS TRUE CACHE BOOL "Don't override flags")
include(${CGAL_USE_FILE})

find_package(Boost REQUIRED COMPONENTS python)
find_package(PythonLibs 2.7 REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# We need this to export __init__.py to ${CATKIN_PACKAGE_PYTHON_DESTINATION}.
catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  DetectCrossing.srv
  LaserDetectCrossing.srv
  )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  lama_msgs
  sensor_msgs
  )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
  include

  LIBRARIES
  crossing_detector
  laser_crossing_detector
  costmap_crossing_detector
  crossing_detector_wrapper_cpp
  laser_crossing_detector_wrapper_cpp

  CATKIN_DEPENDS
  angles
  geometry_msgs
  lama_common
  lama_msgs
  nav_msgs
  roscpp
  sensor_msgs

  DEPENDS
  cgal
  )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CGAL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  )

## Declare a cpp library
add_library(crossing_detector src/crossing_detector.cpp)
add_library(laser_crossing_detector src/laser_crossing_detector.cpp)
add_library(costmap_crossing_detector src/costmap_crossing_detector.cpp)
add_library(crossing_detector_wrapper_cpp src/crossing_detector_wrapper.cpp)
add_library(laser_crossing_detector_wrapper_cpp src/laser_crossing_detector_wrapper.cpp)

## Declare a cpp executable
add_executable(crossing_detector_server src/crossing_detector_server.cpp)
add_executable(laser_crossing_detector_server src/laser_crossing_detector_server.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(crossing_detector_server crossing_detector_generate_messages_cpp)
add_dependencies(laser_crossing_detector_server crossing_detector_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(crossing_detector
  ${catkin_LIBRARIES}
  ${CGAL_LIBRARIES}
  ${CGAL_3RD_PARTY_LIBRARIES}
  )
target_link_libraries(laser_crossing_detector crossing_detector ${catkin_LIBRARIES})
target_link_libraries(costmap_crossing_detector crossing_detector ${catkin_LIBRARIES})
target_link_libraries(crossing_detector_server crossing_detector ${catkin_LIBRARIES})
target_link_libraries(laser_crossing_detector_server laser_crossing_detector crossing_detector ${catkin_LIBRARIES})
target_link_libraries(crossing_detector_wrapper_cpp crossing_detector ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(laser_crossing_detector_wrapper_cpp crossing_detector laser_crossing_detector ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# Don't prepend wrapper library name with lib and add to Python libs.
set_target_properties(crossing_detector_wrapper_cpp PROPERTIES
  PREFIX ""
  LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION})

set_target_properties(laser_crossing_detector_wrapper_cpp PROPERTIES
  PREFIX ""
  LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS
  laser_crossing_detector
  crossing_detector_server
  crossing_detector
  costmap_crossing_detector
  laser_crossing_detector
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS
  crossing_detector_wrapper_cpp
  laser_crossing_detector_wrapper_cpp
  LIBRARY DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(${PROJECT_NAME}-test test/utest.test test/utest.cpp)
  target_link_libraries(${PROJECT_NAME}-test crossing_detector laser_crossing_detector costmap_crossing_detector)
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
