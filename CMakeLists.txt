cmake_minimum_required(VERSION 3.1)

project(apriltag)

set( CMAKE_CXX_COMPILER "g++" )
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3" )

find_package(OpenCV QUIET)
find_package(Sophus REQUIRED)
Find_Package(Eigen3 REQUIRED)
find_package(Pangolin)

include_directories(${EIGEN3_INCLUDE_DIR} ${Sophus_INCLUDE_DIRS} ${Pangolin_INCLUDE_DIRS}) 

include_directories(.)
aux_source_directory(common COMMON_SRC)
set(APRILTAG_SRCS apriltag.c apriltag_pose.c apriltag_quad_thresh.c)

# Library
file(GLOB TAG_FILES ${CMAKE_SOURCE_DIR}/tag*.c)
set_source_files_properties(SOURCE ${TAG_FILES} PROPERTIES COMPILE_FLAGS -O0)
add_library(${PROJECT_NAME} SHARED ${APRILTAG_SRCS} ${COMMON_SRC} ${TAG_FILES})

# opencv_demo
add_executable(opencv_demo example/opencv_demo.cc)
target_link_libraries(opencv_demo apriltag ${OpenCV_LIBRARIES} ${Sophus_LIBRARIES} ${Pangolin_LIBRARIES})
