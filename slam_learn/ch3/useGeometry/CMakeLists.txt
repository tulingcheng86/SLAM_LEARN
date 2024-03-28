cmake_minimum_required(VERSION 2.8)
project(useGeometry)
add_compile_options(-std=c++14)
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-O3" )

#添加头文件
include_directories("/usr/include/eigen3")

add_executable(useGeometry useGeometry.cpp)