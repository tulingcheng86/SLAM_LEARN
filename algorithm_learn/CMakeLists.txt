cmake_minimum_required( VERSION 2.8 )
project(learn_algorthm)

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++14 -O3" )

# OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_DIRS} )

# 添加librealsense2库
find_package(realsense2 REQUIRED)
include_directories(${realsense2_INCLUDE_DIR})

add_executable(KLT KLT.cpp)
target_link_libraries(KLT ${OpenCV_LIBRARIES} ${realsense2_LIBRARY})

add_executable(ORB_featrue ORB_featrue.cpp)
target_link_libraries(ORB_featrue ${OpenCV_LIBRARIES} ${realsense2_LIBRARY})

add_executable(orb orb.cpp)
target_link_libraries(orb ${OpenCV_LIBRARIES})

add_executable(vo vo.cpp)
target_link_libraries(vo ${OpenCV_LIBRARIES})

add_executable(image_stitching image_stitching.cpp)
target_link_libraries(image_stitching ${OpenCV_LIBRARIES})

