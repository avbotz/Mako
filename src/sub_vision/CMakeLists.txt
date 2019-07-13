cmake_minimum_required(VERSION 2.8.3)
project(vision)

set(SPINNAKER_INCLUDE_DIR "/usr/include/spinnaker" CACHE PATH "Spinnaker Include Directory")
set(SPINNAKER_LIB_DIR "/usr/lib" CACHE PATH "Spinnaker Libs Directory")

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg cv_bridge
    image_transport message_generation geometry_msgs)
find_package(OpenCV)
find_library(FLYCAPTURE2 flycapture)

add_service_files(FILES Vision.srv)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS std_msgs
               DEPENDS Boost
               INCLUDE_DIRS include)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} 
    ${SPINNAKER_INCLUDE_DIR})
link_directories(${SPINNAKER_LIB_DIR})

# Executables
add_executable(vision_node src/vision.cpp src/camera.cpp src/capture.cpp
    src/detect.cpp src/filters.cpp src/log.cpp src/gate.cpp)
target_link_libraries(vision_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
    flycapture)
add_dependencies(vision_node vision_gencpp)

add_executable(mock_camera_node src/mock_camera.cpp)
target_link_libraries(mock_camera_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(mock_camera_node vision_gencpp)

add_executable(mock_detect_node src/mock_detect.cpp)
target_link_libraries(mock_detect_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(mock_detect_node vision_gencpp)

add_executable(acquisition_node src/acquisition.cpp)
target_link_libraries(acquisition_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
    Spinnaker)
add_dependencies(acquisition_node vision_gencpp)

#catkin_install_python(PROGRAMS bin/inference_node 
#                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
