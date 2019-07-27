cmake_minimum_required(VERSION 2.8.3)
project(mission)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation
    vision control)
find_package(OpenCV)

catkin_package(CATKIN_DEPENDS message_runtime INCLUDE_DIRS include)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(mission_node src/mission.cpp src/functions.cpp src/commands.cpp
	src/client.cpp src/euler.cpp)
target_link_libraries(mission_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(mission_node mission_gencpp)

add_executable(restart_node src/restart.cpp)
target_link_libraries(restart_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(restart_node mission_gencpp)

add_executable(prelim_node src/prelim.cpp src/functions.cpp src/commands.cpp
	src/client.cpp src/euler.cpp)
target_link_libraries(prelim_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(prelim_node mission_gencpp)
