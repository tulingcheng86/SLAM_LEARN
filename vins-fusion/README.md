**【学习VINS-MONO】环境配置、测试**

 

[Ubuntu18.04安装教程（很详细）_ubuntu18安装-CSDN博客](https://blog.csdn.net/weixin_43233550/article/details/115417176?ops_request_misc=%7B%22request%5Fid%22%3A%22170375722216800186584716%22%2C%22scm%22%3A%2220140713.130102334..%22%7D&request_id=170375722216800186584716&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~blog~top_positive~default-1-115417176-null-null.nonecase&utm_term=ubuntu18.04安装教程&spm=1018.2226.3001.4450)

 

[unbuntu18.04安装ROS（自测成功安装）-CSDN博客](https://blog.csdn.net/weixin_45634390/article/details/135058500)

 

优化终端[csdn - 安全中心](https://link.csdn.net/?target=https%3A%2F%2Fzhuanlan.zhihu.com%2Fp%2F346665734)

 

[电脑本机连了VPN 在虚拟机中没有连接 如何虚拟机共享主机VPN连接-CSDN博客](https://blog.csdn.net/qq_27462573/article/details/130484723)

 

前置依赖

[ceres-solver/ceres-solver: A large scale non-linear optimization library (github.com)](https://github.com/ceres-solver/ceres-solver)

 

得先

装高版本的cmake 

下3.27的好点（没必要这么高）建议3.15

[linux/ubuntu 安装 cmake 3.15.3_ubuntu之cmake 3.15.3下载、安装、使用-CSDN博客](https://blog.csdn.net/moumshi/article/details/101231502#:~:text=安装步骤 第1步：下载 下载cmake 3.15.3 压缩包 cmake-3.15.3.tar.gz %3B,地址是 https%3A%2F%2Fcmake.org%2Fdownload%2F ； 第2步：新建文件夹 在合适的地方（如桌面）新建一文件夹（如 InstallCMake），将 cmake-3.15.3.tar.gz 拷贝至此。)

不要随意卸载原先的cmake

[CMake - Upgrade Your Software Build System](https://cmake.org/)

 

开始

catkin_make 失败的话

 

​                               

Pull ceres from github https://github.com/ceres-solver/ceres-solver, checkout branch 1.14.x and install as normal.

 

***\**\**\**\**\**\*参考这个做的\**\**\**\**\**\**\***

[【学习VINS-MONO】环境配置、测试_vins-mono gpu-CSDN博客](https://blog.csdn.net/qq_45306739/article/details/126589852?ops_request_misc=&request_id=&biz_id=102&utm_term=VINS-Mono&utm_medium=distribute.pc_search_result.none-task-blog-2~blog~sobaiduweb~default-1-126589852.nonecase&spm=1018.2226.3001.4450)

**这个是VINS-MONO的！！！！**

**运行**

roscore

source ~/catkin_ws/devel/setup.bash #每个终端运行前都要加上这一句

roslaunch vins_estimator euroc.launch 

roslaunch vins_estimator vins_rviz.launch

roslaunch benchmark_publisher publish.launch sequence_name:=MH_01_easy.bag

rosbag play ~/catkin_ws/Dates/MH_01_easy.bag

 

 

**【学习VINS-Fusion】环境配置、测试**

 

 

[HKUST-Aerial-Robotics/VINS-Fusion: An optimization-based multi-sensor state estimator (github.com)](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion?tab=readme-ov-file#3-euroc-example)

**Ros 配置同 mono**

**Build VINS-Fusion**

 

cd ~/catkin_ws/src

  git clone https://github.com/HKUST-Aerial-Robotics/VINS-Fusion.git

  cd ../

  catkin_make

  source ~/catkin_ws/devel/setup.bash

 

测试

[kmavvisualinertialdatasets – ASL Datasets (ethz.ch)](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)

下载

```
MH_01_easy.bag
```

1、 **Monocualr camera + IMU单摄像头**

source ~/catkin_ws/devel/setup.bash

 

```
roslaunch vins vins_rviz.launch
    rosrun vins vins_node ~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_mono_imu_config.yaml 
    (optional) rosrun loop_fusion loop_fusion_node ~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_mono_imu_config.yaml 
    rosbag play ~/catkin_ws1/Dates/MH_01_easy.bag
```

 

 

2、 **Stereo cameras + IMU 立体摄像头**

 

source ~/catkin_ws/devel/setup.bash

 

```
   roslaunch vins vins_rviz.launch
    rosrun vins vins_node ~/catkin_ws1/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml 
    (optional) rosrun loop_fusion loop_fusion_node ~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml 
    rosbag play ~/catkin_ws1/Dates/MH_01_easy.bag
```

 

 

3、 **立体相机Stereo cameras**

source ~/catkin_ws/devel/setup.bash

 

```
roslaunch vins vins_rviz.launch
    rosrun vins vins_node ~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_config.yaml 
    (optional) rosrun loop_fusion loop_fusion_node ~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_config.yaml 
    rosbag play YOUR_DATASET_FOLDER/MH_01_easy.bag
```

 

 

 

4、 **KITTI 数据集测试**

Download [KITTI Odometry dataset](http://www.cvlibs.net/datasets/kitti/eval_odometry.php) to YOUR_DATASET_FOLDER. Take sequences 00 for example, Open two terminals, run vins and rviz respectively. (We evaluated odometry on KITTI benchmark without loop closure funtion)

 

source ~/catkin_ws1/devel/setup.bash

 

```
roslaunch vins vins_rviz.launch
```



```
(optional) rosrun loop_fusion loop_fusion_node ~/catkin_ws/src/VINS-Fusion/config/kitti_odom/kitti_config00-02.yaml
 
rosrun vins kitti_odom_test ~/catkin_ws/src/VINS-Fusion/config/kitti_odom/kitti_config00-02.yaml YOUR_DATASET_FOLDER/sequences/00/ 
```
 

rosrun vins kitti_odom_test ~/catkin_ws1/src/VINS-Fusion/config/kitti_odom/kitti_config00-02.yaml ~/Downloads/00/
 

**ROS2 版本** （实现）

**搜这个的时候搜出来的**

https://github.com/zinuok/VINS-Fusion-ROS2

**参考这个issue**
https://github.com/zinuok/VINS-Fusion-ROS2/issues/8

“@cvirxsc Try this one: https://github.com/bonabai/VINS-Fusion-ROS2.git, and checkout no_cuda branch.”

**用这个作者的**
https://github.com/bonabai/VINS-Fusion-ROS2

**依赖**
环境是ubuntu 22.04   roshumble
opencv 应该是自带的

Eigen-3.3.9
Ceres 1.14?

**build**
cd $(PATH_TO_YOUR_ROS2_WS)/src
git clone https://github.com/zinuok/VINS-Fusion-ROS2
cd ..
colcon build --symlink-install && source ./install/setup.bash && source ./install/local_setup.bash


cmake出错 可能是cmake 版本太低

 

 

 

 

$ ./bootstrap && make && sudo make install

 

 

```
cp sensing/gimbal/gazebo_gimbal_controller_plugin.cpp /home/tlc/PX4-Autopliot/Tools/sitl_gazebo/src/
 
cp sitl_config/init.d-posix/rcS /home/tlc/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix
 
cp sitl_config/worlds/* /home/tlc/PX4-Autopilot/Tools/sitl_gazebo/worlds
 
cp -r sitl_config/models/* /home/tlc/PX4-Autopilot/Tools/sitl_gazebo/models
 
cp -r sitl_config/launch/* /home/tlc/PX4-Autopilot/launch
 

```



 

```
Xtdrone VINS-fusion仿真
参考
Ubuntu18.04 XTDrone 仿真环境配置 简记-接PX4速配 - 知乎 (zhihu.com)
 
Ubuntu18.04从零开搭PX4&Mavros&Gazebo环境并测试(极速版) - 知乎 (zhihu.com)
 
仿真平台基础配置 (yuque.com)
 
 
VINS-fusion依赖
参考
【学习VINS-MONO】环境配置、测试_vins-mono gpu-CSDN博客
 
```
