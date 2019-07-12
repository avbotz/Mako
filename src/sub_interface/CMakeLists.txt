cmake_minimum_required(VERSION 2.8.3)
project(interface)

find_package(catkin REQUIRED COMPONENTS
	rospy genmsg message_generation
)
catkin_python_setup()

catkin_package()

include_directories(
	${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
	bin/interface_node
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

