cmake_minimum_required(VERSION 2.8.3)
project(interface)

find_package(catkin REQUIRED COMPONENTS
	rospy std_msgs genmsg message_generation
)
catkin_python_setup()

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS std_msgs)

include_directories(
	${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
	bin/interface_node
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

