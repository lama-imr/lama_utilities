cmake_minimum_required(VERSION 2.8.3)
project(lama_interfaces)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
	COMPONENTS
	roscpp
	rospy
	message_generation
	actionlib
	actionlib_msgs
	geometry_msgs
	sensor_msgs
	nav_msgs
	)

## System dependencies are found with CMake's conventions
find_package(PkgConfig)
pkg_check_modules(SQLITE REQUIRED sqlite3)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
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
add_message_files(
	FILES
	LamaDescriptorIdentifier.msg
	LamaMapAction.msg
	LamaObject.msg
	complex.msg
)

## Generate services in the 'srv' folder
add_service_files(
	FILES
	AddInterface.srv
	ActOnMap.srv
	GetLamaObject.srv
	SetLamaObject.srv
	GetVectorLaserScan.srv
	SetVectorLaserScan.srv
	lmi_polygon_get.srv
	lmi_polygon_set.srv
	lmi_vector_double_get.srv
	lmi_vector_double_set.srv
	lmi_zernike_moments_get.srv
	lmi_zernike_moments_set.srv
	lmi_vector_pose_get.srv
	lmi_vector_pose_set.srv
	lmi_vector_odometry_get.srv
	lmi_vector_odometry_set.srv
	test.srv
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  Localize.action
  Navigate.action
  Learn.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
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
	INCLUDE_DIRS include ${catkin_INCLUDE_DIRS} ${SQLITE_INCLUDE}
	LIBRARIES jockey learning_jockey localizing_jockey navigating_jockey
	CATKIN_DEPENDS roscpp rospy actionlib_msgs geometry_msgs sensor_msgs nav_msgs
	DEPENDS SQLITE
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
  ${SQLITE_INCLUDE}
)

## Declare a cpp library
#add_library(lama_core src/lama_core.cpp)
add_library(jockey src/jockey.cpp)
add_library(learning_jockey src/learning_jockey.cpp)
add_library(localizing_jockey src/localizing_jockey.cpp)
add_library(navigating_jockey src/navigating_jockey.cpp)

## Declare a cpp executable
# set(SRC ${SRC} src/CLMIIntegralInvariant.cpp)
# set(SRC ${SRC} src/CLMIPolygon.cpp)
# set(SRC ${SRC} src/CLMIZernikeMoments.cpp)
# set(SRC ${SRC} src/lama_utils.cpp)
# set(SRC ${SRC} src/lmi_defaults.cpp)
# add_executable(lama_interfaces ${SRC})

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(lama_core lama_interfaces_generate_messages_cpp)
# add_dependencies(lama_interfaces lama_interfaces_generate_messages_cpp)
add_dependencies(jockey lama_interfaces_generate_messages_cpp)
add_dependencies(learning_jockey lama_interfaces_generate_messages_cpp)
add_dependencies(localizing_jockey lama_interfaces_generate_messages_cpp)
add_dependencies(navigating_jockey lama_interfaces_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(lama_interfaces
# ${catkin_LIBRARIES}
# ${Boost_LIBRARIES}
# ${SQLITE_LIBRARIES}
# )

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
# install(TARGETS lama_interfaces lama_interfaces_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_lama_interfaces.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
catkin_add_nosetests(test)
