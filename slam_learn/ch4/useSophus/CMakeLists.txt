cmake_minimum_required(VERSION 3.15)
project(useSophus)

set(CMAKE_CXX_STANDARD 11)

# 使用Eigen
include_directories(/usr/local/include/eigen3)
# 使用Sophus
find_package(Sophus REQUIRED)
add_executable(useSophus useSophus.cpp)
target_link_libraries(useSophus ${Sophus_LIBRARIES})

