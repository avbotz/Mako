cmake_minimum_required(VERSION 2.8.3)
project(remote_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation control)
find_package(OpenCV)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(remote_control_node src/receiver.cpp)
target_link_libraries(remote_control_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(remote_control_node remote_control_gencpp)
