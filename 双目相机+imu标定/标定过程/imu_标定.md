# 安装
https://github.com/ethz-asl/kalibr/wiki/installation
用18.04docker镜像
启动
docker run -it -v /tmp/.X11-unix:/tmp/.X11-unix:rw --privileged --gpus all -e DISPLAY=:1 -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all -e PYTHONUNBUFFERED=1 -e QT_X11_NO_MITSHM=1 kalibr:latest /bin/bash

# imu标定过程

## 依赖安装
### google-glog + gflags
sudo apt-get install libgoogle-glog-dev libgflags-dev
### BLAS & LAPACK
sudo apt-get install libatlas-base-dev
### SuiteSparse and CXSparse (optional)
sudo apt-get install libsuitesparse-dev

## 安装ceres
wget http://ceres-solver.org/ceres-solver-2.0.0.tar.gz
tar zxf ceres-solver-2.0.0.tar.gz
cd ceres-solver
mkdir build && cd build
cmake ..
make
sudo make install
### 这里如果报eigen3版本错误，就源码安装eigen3.3
源码安装eigen3.3

## 手动安装Eigen3.3.3
https://gitlab.com/libeigen/eigen/-/releases
cd 
mkdir build
cd build
cmake ..
sudo make install

## code_utils安装
### 安装依赖
sudo apt-get install libdw-dev

### 安装 code_utils
cd kalibr_workspace/src
git clone https://github.com/gaowenliang/code_utils.git
cd ..
catkin_make

### 报错
backward.hpp No such file

将报错文件的#include "backward.hpp"改成#include "code_utils/backward.hpp"

## imu_utils安装

cd kalibr_workspace/src
git clone https://github.com/gaowenliang/imu_utils.git
### 将CMakeLists.txt中的 `set(CMAKE_CXX_FLAGS "-std=c++11")` 改成 `set(CMAKE_CXX_STANDARD 14)`

cd ..
catkin_make


注意一定要先编译code_utils再编译imu_utils

------------------------------------

# 标定imu
这里将IMU静置，录取bag包，官方建议是两个小时

在imu_utils包中新建一个launch文件，格式如下

<launch>
    <node pkg="imu_utils" type="imu_an" name="imu_an" output="screen">
        <param name="imu_topic" type="string" value= "/imu"/>    # imu topic的名字
        <param name="imu_name" type="string" value= "my_imu"/>   	# imu名字，和生成的标定结果文件名有关
        <param name="data_save_path" type="string" value= "$(find imu_utils)/data/"/>
        <param name="max_time_min" type="int" value= "１２０"/>   #标定的时长，bag包时长
        <param name="max_cluster" type="int" value= "100"/>
    </node>
</launch>

开始标定
-------------------------
## 启动launch文件
roslaunch imu_utils imu.launch

## 以200倍数播放bag包
rosbag play -r 200 imu.bag

-------------------
出现Wait for imu data时候：
参考https://github.com/gaowenliang/imu_utils/issues/15
我认为参数(“max_time_min”)的意思是bag文件要播放xxx分钟，所以我把它改成1分钟，很快就播放完了，但它会提醒我“时间太短”。最后我运行imu 40 m ，参数设置如下
<param name="max_time_min" type="int" value= "30"/>


会在data文件夹下生成个yaml文件
### 例子
%YAML:1.0

type: IMU
name: my_imu
Gyr:
   unit: " rad/s"
   avg-axis:
      gyr_n: 2.0241157583860731e-03
      gyr_w: 1.6755331439024962e-04
   x-axis:
      gyr_n: 2.0115721398540937e-03
      gyr_w: 1.9766444409295751e-04
   y-axis:
      gyr_n: 1.9144995423045869e-03
      gyr_w: 1.1930782025151282e-04
   z-axis:
      gyr_n: 2.1462755929995379e-03
      gyr_w: 1.8568767882627850e-04
Acc:
   unit: " m/s^2"
   avg-axis:
      acc_n: 7.5605406743003934e-03
      acc_w: 9.6403789014655721e-04
   x-axis:
      acc_n: 6.8109563537370179e-03
      acc_w: 9.8003893167469022e-04
   y-axis:
      acc_n: 7.5030423699921503e-03
      acc_w: 1.0967886059304567e-03
   z-axis:
      acc_n: 8.3676232991720129e-03
      acc_w: 8.1528613283452455e-04


# 保存结果
为了后面的相机-IMU标定，我们需要gyr_n,gyr_w,acc_n,acc_w四个量，新建imu.yaml，保存下来

rostopic: /sbpilot/imu
update_rate: 200.0 #Hz

accelerometer_noise_density: 7.5605406743003934e-03
accelerometer_random_walk: 9.6403789014655721e-04
gyroscope_noise_density: 2.0241157583860731e-03
gyroscope_random_walk: 1.6755331439024962e-04

--------------------------------------------------------
IMU标定完成



