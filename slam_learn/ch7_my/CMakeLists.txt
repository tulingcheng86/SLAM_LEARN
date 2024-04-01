cmake_minimum_required( VERSION 2.8 )
project( vo1 )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++17 -O3" )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package( OpenCV  REQUIRED )
# find_package( OpenCV REQUIRED ) # use this if in OpenCV2 

include_directories( 
    ${OpenCV_INCLUDE_DIRS} 
    "/usr/include/eigen3/"
)


add_executable(feature_extr feature_extr.cpp)
target_link_libraries(feature_extr ${OpenCV_LIBRARIES})

add_executable(pose_estimation_2d2d pose_estimation_2d2d.cpp)
target_link_libraries(pose_estimation_2d2d ${OpenCV_LIBRARIES})

add_executable(triangulation triangulation.cpp)
target_link_libraries(triangulation ${OpenCV_LIBRARIES})

#add_executable(pose_estimation_3d2d pose_estimation_3d2d.cpp)
#target_link_libraries(pose_estimation_3d2d ${OpenCV_LIBRARIES})