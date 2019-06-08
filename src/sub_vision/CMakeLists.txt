cmake_minimum_required(VERSION 2.8.3)
project(vision)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg cv_bridge
    image_transport)
find_package(OpenCV)
find_library(FLYCAPTURE2 flycapture)
find_library(SPINNAKER spinnaker)

add_service_files(FILES Perception.srv)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS std_msgs
               DEPENDS Boost
               INCLUDE_DIRS include)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

# Executables
add_executable(vision_node src/vision.cpp src/camera.cpp src/capture.cpp
    src/detect.cpp src/filters.cpp src/log.cpp src/gate.cpp)
target_link_libraries(vision_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
    flycapture)
add_dependencies(vision_node vision_gencpp)

add_executable(mock_camera src/mock_camera.cpp)
target_link_libraries(mock_camera ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(mock_camera vision_gencpp)

add_executable(mock_detect src/mock_detect.cpp)
target_link_libraries(mock_detect ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(mock_detect vision_gencpp)

