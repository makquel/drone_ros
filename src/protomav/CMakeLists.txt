cmake_minimum_required(VERSION 2.8.3)
project(protomav)

find_package(catkin REQUIRED 
  std_msgs 
  roscpp 
  mavros 
  sensor_msgs
  tf
  message_generation
  move_base_msgs
  actionlib
  geographic_msgs
  geodesy
  )

add_message_files(
  FILES
  WaypointMsg.msg
 )

add_service_files(
   FILES
   AddWaypoint.srv
   GetWPListSize.srv
   GetWPVector.srv
   ClearWaypoints.srv
   SetListCompleted.srv
 )

generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
   geometry_msgs
 )

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES protomav
   CATKIN_DEPENDS roscpp std_msgs mavros sensor_msgs tf message_runtime move_base_msgs geographic_msgs geodesy actionlib
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS}
  include/${PROJECT_NAME}
  )
  
## Declare a cpp library
add_library(protomav
  src/MavMessenger.cpp
  src/wp_manager_client.cpp 
  src/broadcast.cpp
 )

target_link_libraries(protomav ${catkin_LIBRARIES})

## Declare a cpp executable
add_executable(heartbeat src/heartbeat.cpp src/MavMessenger.cpp)
add_executable(status src/status.cpp src/MavMessenger.cpp)
#add_executable(camera src/camera.cpp src/MavMessenger.cpp)
add_executable(wp_manager src/wp_manager.cpp)
add_executable(control src/control.cpp src/MavMessenger.cpp )
#add_executable(mav_messenger src/MavMessenger.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(protomav protomav_generate_messages_cpp)
add_dependencies(wp_manager protomav_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(heartbeat
   protomav
   ${catkin_LIBRARIES}
 )
target_link_libraries(wp_manager
   protomav
   ${catkin_LIBRARIES}
 )
 
target_link_libraries(control
   protomav
   ${catkin_LIBRARIES}
 )	
 
target_link_libraries(status
   protomav
   ${catkin_LIBRARIES}
 )
 


#target_link_libraries(mav_messenger
#   ${catkin_LIBRARIES}
#   mavros
#  )
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

# Mark executables and/or libraries for installation
install(TARGETS protomav 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  # myfile1
  # myfile2
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_protomav.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
