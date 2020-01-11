cmake_minimum_required(VERSION 3.1)

project(apriltag)

find_package(OpenCV QUIET)

include_directories(.)
aux_source_directory(common COMMON_SRC)
set(APRILTAG_SRCS apriltag.c apriltag_pose.c apriltag_quad_thresh.c)

# Library
file(GLOB TAG_FILES ${CMAKE_SOURCE_DIR}/tag*.c)
set_source_files_properties(SOURCE ${TAG_FILES} PROPERTIES COMPILE_FLAGS -O0)
add_library(${PROJECT_NAME} SHARED ${APRILTAG_SRCS} ${COMMON_SRC} ${TAG_FILES})

# opencv_demo
add_executable(opencv_demo example/opencv_demo.cc)
target_link_libraries(opencv_demo apriltag ${OpenCV_LIBRARIES})
