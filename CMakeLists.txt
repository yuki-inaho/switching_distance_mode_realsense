cmake_minimum_required(VERSION 3.5)
project(aspara_picker_realsense)

set(CMAKE_CXX_COMPILER "/usr/bin/g++")
set(CMAKE_CXX_FLAGS "-O0 -std=gnu++11 -fopenmp")


add_executable(RealSenseAspara src/main.cpp)
include_directories(
  include
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(realsense2 realsense2 REQUIRED)
include_directories(${realsense2_CFLAGS})
target_link_libraries(RealSenseAspara ${realsense2_LDFLAGS})

pkg_check_modules(OpenCV opencv REQUIRED)
include_directories(${OpenCV_CFLAGS})
target_link_libraries(RealSenseAspara ${OpenCV_LDFLAGS})

find_package(PkgConfig REQUIRED)
pkg_check_modules(eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(VTK REQUIRED)

include_directories(${eigen3_INCLUDE_DIRS})
target_link_libraries(RealSenseAspara ${eigen3_LIBRARIES})
include_directories(${PCL_INCLUDE_DIRS})
target_link_libraries(RealSenseAspara ${PCL_LIBRARIES})
include_directories(${VTK_INCLUDE_DIRS})
target_link_libraries(RealSenseAspara ${VTK_LIBRARIES})

pkg_check_modules(mlpack mlpack REQUIRED)
include_directories(${mlpack_INCLUDE_DIRS})
target_link_libraries(RealSenseAspara ${mlpack_LIBRARIES})
