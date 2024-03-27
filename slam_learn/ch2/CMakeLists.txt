# 声明要求的 cmake 最低版本
cmake_minimum_required(VERSION 2.8)

# 声明一个 cmake 工程
project(HelloSLAM)

# 添加一个可执行程序
# 语法：add_executable( 程序名 源代码文件 ）
add_executable(helloslam helloslam.cpp)

add_executable(useslam usehelloslam.cpp)
target_link_libraries(useslam hello_shared)

#add_executable( ku名 源代码文件 ）
#对于库的使用者，只要拿到了头文件和库文件，就可以调用这个库了。
add_library(hello libhelloslam.cpp)
#得到的文件是 libhello_shared.so
add_library( hello_shared SHARED libhelloslam.cpp )

