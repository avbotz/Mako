cmake_minimum_required(VERSION 2.8.3)
project(mission)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation
    vision control)
find_package(OpenCV)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(mission_node src/mission.cpp)
target_link_libraries(mission_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(mission_node mission_gencpp)
