cmake_minimum_required(VERSION 2.8.3)
project(vision)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg cv_bridge
    image_transport)
find_package(OpenCV)
find_library(FLYCAPTURE2 flycapture)

add_service_files(FILES Observation.srv)

generate_messages(DEPENDENCIES std_msgs)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

# Executables
add_executable(vision_node src/vision.cpp src/camera.cpp src/capture.cpp
    src/detect.cpp src/filters.cpp src/log.cpp)
target_link_libraries(vision_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
    flycapture)
add_dependencies(vision_node vision_gencpp)

add_executable(camera src/mock_camera.cpp)
target_link_libraries(camera ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(camera vision_gencpp)

add_executable(detect src/mock_detect.cpp)
target_link_libraries(detect ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(detect vision_gencpp)
