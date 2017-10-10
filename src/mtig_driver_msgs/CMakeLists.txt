cmake_minimum_required(VERSION 2.8.3)
project(mtig_driver_msgs)

find_package(catkin REQUIRED COMPONENTS 
	std_msgs 
	message_generation
)

add_message_files(
	DIRECTORY msg
	FILES
	GpsInfo.msg
)

generate_messages(
	DEPENDENCIES
	std_msgs
)

catkin_package(
	DEPENDS message_runtime
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)