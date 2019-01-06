cmake_minimum_required(VERSION 2.8.3)
project(control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED)
find_package(OpenCV)

catkin_package(INCLUDE_DIRS include)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.hpp"
    PATTERN ".svn" EXCLUDE)
