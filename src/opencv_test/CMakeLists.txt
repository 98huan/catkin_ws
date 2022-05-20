cmake_minimum_required(VERSION 2.8)
Project(opencv_test)

find_package(OpenCV REQUIRED)

##下面是输出信息        我也不知道输出到哪里
# message(STATUS "Opnecv ;ibrary status: ")
# message(STATUS "> version: ${OpenCV_VERSION} ")
# message(STATUS "libraries: ${OpenCV_LIBS} ")
# message(STATUS "> include: ${OpenCV_INCLUDE_DIRS}  ")
include_directories(${OpenCV_INCLUDE_DIRS}) 

add_executable(test test.cpp)
target_link_libraries(test ${OpenCV_LIBS})