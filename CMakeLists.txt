cmake_minimum_required(VERSION 3.1)

project(apriltag)

set( CMAKE_CXX_COMPILER "g++" )
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3" )

#LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
#Find_Package(G2O REQUIRED)
#SET(G2O_LIBS g2o_cli g2o_ext_freeglut_minimal g2o_simulator g2o_solver_slam2d_linear 
#    g2o_types_icp g2o_types_slam2d g2o_types_sba g2o_types_slam3d g2o_core g2o_interface 
#    g2o_solver_csparse g2o_solver_structure_only g2o_csparse_extension g2o_opengl_helper g2o_solver_dense 
#    g2o_stuff g2o_types_sclam2d g2o_parser g2o_solver_pcg g2o_types_data g2o_types_sim3 cxsparse )

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
#target_link_libraries(opencv_demo apriltag ${OpenCV_LIBRARIES} ${Sophus_LIBRARIES} ${Pangolin_LIBRARIES} ${G2O_LIBS})
target_link_libraries(opencv_demo apriltag ${OpenCV_LIBRARIES} ${Sophus_LIBRARIES} ${Pangolin_LIBRARIES})