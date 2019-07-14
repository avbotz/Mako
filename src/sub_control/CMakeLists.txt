cmake_minimum_required(VERSION 2.8.3)
project(control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS vision roscpp std_msgs genmsg
	message_generation)

add_service_files(FILES ControlAlive.srv ControlState.srv ControlWrite.srv 
	ControlWriteState.srv)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS std_msgs
               DEPENDS Boost
               INCLUDE_DIRS include)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(control_node src/control.cpp src/service.cpp src/atmega.cpp)
target_link_libraries(control_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
    flycapture)
add_dependencies(control_node control_gencpp)

add_library(control src/atmega.cpp)

catkin_package(INCLUDE_DIRS include LIBRARIES control)
