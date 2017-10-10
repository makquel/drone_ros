cmake_minimum_required(VERSION 2.8.3)
project(yocto)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  genmsg
)

find_package(PkgConfig REQUIRED)
#find_package(libusb-1.0 REQUIRED)
include_directories (${LIBUSB_1_INCLUDE_DIRS})
pkg_search_module(LIBUSB_1 REQUIRED libusb-1.0)
include_directories(SYSTEM ${LIBUSB_1_INCLUDE_DIRS})

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

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

# Generate messages in the 'msg' folder
 add_message_files(
   DIRECTORY
   msg
   FILES
   PWM_info.msg
	 voltage_info.msg
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
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
#  INCLUDE_DIRS include
#  LIBRARIES yocto
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

link_directories( /usr/local/lib )
link_directories( /usr/local/include )

#add_library(libm)
#add_library(libpthread)
#add_library(libusb1.0)
#add_library(libstdc++)
## Declare a cpp library

add_library(yoctoapi
	src/yocto/yapi/yapi.c
	src/yocto/yapi/yapi.def
	src/yocto/yapi/ydef.h
	src/yocto/yapi/yfifo.c
	src/yocto/yapi/yhash.c
	src/yocto/yapi/yjson.c
	src/yocto/yapi/ykey.c
	src/yocto/yapi/ymemory.c
	src/yocto/yapi/ypkt_lin.c
	src/yocto/yapi/ypkt_osx.c
	src/yocto/yapi/ypkt_win.c
	src/yocto/yapi/yprog.c
	src/yocto/yapi/yproto.h
	src/yocto/yapi/ystream.c
	src/yocto/yapi/ytcp.c
	src/yocto/yapi/ythread.c
	src/yocto/yapi/yversion.h
)
 add_library(yocto
   src/yocto/yocto_api.cpp
   src/yocto/yocto_pwminput.cpp
	 src/yocto/yocto_voltage.cpp
)

## Declare a cpp executable
# add_executable(yocto_node src/yocto_node.cpp)



## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(yocto_node yocto_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(yocto_node
#   ${catkin_LIBRARIES}
# )

target_link_libraries(yocto
   ${catkin_LIBRARIES}
   ${LIBUSB_1_LIBRARIES}
   yoctoapi
 )

target_link_libraries(yoctoapi
   ${catkin_LIBRARIES}
   ${LIBUSB_1_LIBRARIES}
 )


add_executable(yocto_node src/yocto_PWM_node.cpp)
add_dependencies(yocto_node yocto_generate_messages_cpp yocto yoctoapi ${catkin_EXPORTED_TARGETS})
target_link_libraries(yocto_node yocto yoctoapi ${catkin_LIBRARIES} ${LIBUSB_1_LIBRARIES})


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
# install(TARGETS yocto yocto_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_yocto.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
