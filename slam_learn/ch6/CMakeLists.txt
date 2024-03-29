cmake_minimum_required( VERSION 2.8 )
project(gauss_newton)

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++14 -O3" )

# OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_DIRS} )

add_executable(gaussnewton gauss_newton.cpp)
target_link_libraries(gaussnewton ${OpenCV_LIBRARIES})