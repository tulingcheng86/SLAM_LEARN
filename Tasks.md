# TASKS

- [x] jetson nano本地录个包 ，发给另一台电脑跑一下，能很好运行
- [x] 录一个场景大一点的包，开回环不开回环都测一下，能很好运行
- [x] 建立一个完整的docker
- [x] ros1 的包给ros2跑试下，能很好运行
- [ ] 路径估计，定位，轨迹规划
- [x] xtdrone 仿真环境搭建，部署egoplanner
- [x] 能用640*480 参数跑VIO-RTABMAP
- [x] d435i录制包 **离线**跑vio-rtabmap 看要怎么编写相机启动函数和录制哪些话题才能启动
- [x] 用evo分析了VINS-RTAB和RTAB跑MH01数据集的轨迹，VINS-RTABMAP的效果还差些。。应该是参数没设好
- [ ] 能录制跑 一开始效果还好，但是播放到一段时间后数据没那么全，还是录制的时候量太大了，可以分包来录制试试
- [ ] 建图保存加到仿真环境里面
- [ ] 探索路径规划
- [ ] slam14讲
- [ ] linux 高级编程
- [ ] webserver
- [ ] 探索毕设内容 语义salm
- [ ] 



# daily task

3.29

- [x] slam 第六讲 了解 最小二乘法 高斯牛顿法
- [x] 写了orb特征提取
- [x] slam第七讲 实践 （看不完 还有一半）--剩下的花大概一个星期看完
- [x] linux高级编程看下 为后续webserver做准备 --花大概一个星期看完
- [ ] 刷算法题2-3道 
- [x] 每天看看八股 2-3节



3.30-3.31

- [ ] slam第七\八讲 实践 （看不完 还有一半）--剩下的花大概一个星期看完
- [ ] linux高级编程看下 为后续webserver做准备 --花大概一个星期看完
- [x] 刷算法题2-3道 
- [x] 每天看看八股 2-3节



4.14.1

- [x] slam第七\八讲 实践 （看不完 还有一半）--剩下的花大概一个星期看完
- [x] linux高级编程看下 为后续webserver做准备 --花大概一个星期看完
- [ ] 刷算法题2-3道 
- [ ] 每天看看八股 2-3节



4.2

- [x] slam第十讲 
- [ ] linux高级编程看下 为后续webserver做准备 --花大概一个星期看完
- [x] 刷算法题2-3道 /晚上看看有没有空再刷点
- [ ] 每天看看八股 2-3节
- [x] 读文献 4篇 关于语义分割 slam的





# 1去除结构光

```
  <arg name="allow_no_texture_points"   default="false"/>
  <arg name="emitter_enable"   		      default="false"/>
	
  <!-- rosparam set /camera/stereo_module/emitter_enabled false -->
  <rosparam>
  /camera/stereo_module/emitter_enabled: 0
  </rosparam>

  <rosparam if="$(arg emitter_enable)">
  /camera/stereo_module/emitter_enabled: 1
  </rosparam>
```

# 2录制bag

```
 rosbag record -O d435i-test.bag /camera/imu /camera/infra1/image_rect_raw /camera/infra2/image_rect_raw /camera/infra1/camera_info /camera/infra2/camera_info


  colcon build --packages-select realsense2_camera
```

# 3查看QOS

tlc@192.168.100.236
ros2 topic info --verbose /camera/camera/imu

export ROS_DOMAIN_ID=30

# 4 ip问题 GPT4问问题 

这个jetson nano连接了一个D435i相机，并且配置好了realsense固件和realsense-ros，通过roslaunch realsense2_camera rs_fusion_camera_stereo.launch 启动相机ros话题节点，可以查看到相机的各个话题信息。现在我要将 jetson nano发布的ros话题信息发送给同一局域网下的电脑，使得这台电脑能rostopic list查看到话题，也能订阅这个话题。有什么方法吗

**jetson nano ip为**
192.168.100.207
export ROS_MASTER_URI=http://192.168.100.207:11311
export ROS_IP=192.168.100.207

忘记说了，另一台电脑上是用docker运行的ros镜像，这个docker里的ros能订阅到jetson nano发布的话题吗
**ros镜像 ip为** 172.17.0.2
export ROS_MASTER_URI=http://192.168.100.207:11311
export ROS_IP=172.17.0.2

export ROS_MASTER_URI=http://192.168.100.207:11311
export ROS_IP=172.17.0.2

# 5 SCP复制
```shell
scp jetson@192.168.100.207:/home/jetson/realsense_ws/data/d435ii.bag /你的本地地址/
```

# 6 .sh
```shell
roslaunch vins vins_rviz.launch

rosrun vins vins_node src/VINS-Fusion/config/realsense_d435i/realsense_stereo_imu_config.yaml

rosrun loop_fusion loop_fusion_node  src/VINSFusion/config/realsense_d435i/realsense_stereo_imu_config.yaml 

roslaunch octomap_server octomap_mapping.launch

rosbag play d435ii.bag 

rosrun octomap_server octomap_saver name.bt

roslaunch realsense2_camera rs_fusion_camera_stereo.launch 

roslaunch rtabmap_ros rtabmap.launch \
   args:="-d --Odom/Strategy 9 --OdomVINS/ConfigPath src/VINS-Fusion/config/realsense_d435i/realsense_stereo_imu_config.yaml" \
   left_image_topic:=/camera/infra1/image_rect_raw \
   right_image_topic:=/camera/infra2/image_rect_raw \
   stereo:=true \
   frame_id:=camera_imu_optical_frame \
   imu_topic:="/camera/imu" \
   wait_imu_to_init:="true"
```



# 7 容器启动 /构建 命令
```shell
xhost +
docker run -it -v /tmp/.X11-unix:/tmp/.X11-unix:rw --privileged --gpus all -e DISPLAY=:1 -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all -e PYTHONUNBUFFERED=1 -e QT_X11_NO_MITSHM=1 wdczz/xtdrone:2.3 /bin/bash

docker build -t myapp:v1 .
```



# 8 rtabmap

```shell
roslaunch realsense2_camera rs_fusion_camera_stereo.launch

rosrun imu_filter_madgwick imu_filter_node _use_mag:=false _publish_tf:=false _world_frame:="enu" /imu/data_raw:=/camera/imu /imu/data:=/rtabmap/imu

roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start --Optimizer/GravitySigma 0.3" depth_topic:=/camera/aligned_depth_to_color/image_raw rgb_topic:=/camera/color/image_raw camera_info_topic:=/camera/color/camera_info approx_sync:=false wait_imu_to_init:=true imu_topic:=/rtabmap/imu

roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start" depth_topic:=/camera/aligned_depth_to_color/image_raw rgb_topic:=/camera/color/image_raw camera_info_topic:=/camera/color/camera_info
```

## 安装rtabmap

https://blog.csdn.net/Starry_Sheep/article/details/124725504

```shell
sudo apt-get install ros-melodic-rtabmap ros-melodic-rtabmap-ros 

sudo apt-get remove ros-melodic-rtabmap ros-melodic-rtabmap-ros
```

这个不需要安装在工作目录下

```shell
cd 
git clone https://github.com/introlab/rtabmap.git rtabmap
cd rtabmap/build
cmake -DCMAKE_INSTALL_PREFIX=~/catkin_ws/devel ..
make -j4
make install
```

安装RTABMAP_ROS
这个安装在工作目录下

```shell
mkdir rtabmap_ws
cd rtabmap_ws
git clone https://github.com/introlab/rtabmap_ros.git rtabmap_ros
catkin build
```





## 保存

```c++
<launch>
  <arg name="serial_no"           default=""/>
  <arg name="usb_port_id"         default=""/>
  <arg name="device_type"         default=""/>
  <arg name="json_file_path"      default=""/>
  <arg name="camera"              default="camera"/>
  <arg name="tf_prefix"           default="$(arg camera)"/>
  <arg name="external_manager"    default="false"/>
  <arg name="manager"             default="realsense2_camera_manager"/>
  <arg name="output"              default="screen"/>
  <arg name="respawn"              default="false"/>

  <arg name="fisheye_width"       default="-1"/>
  <arg name="fisheye_height"      default="-1"/>
  <arg name="enable_fisheye"      default="false"/>

  <arg name="depth_width"         default="-1"/>
  <arg name="depth_height"        default="-1"/>
  <arg name="enable_depth"        default="true"/>

  <arg name="confidence_width"    default="-1"/>
  <arg name="confidence_height"   default="-1"/>
  <arg name="enable_confidence"   default="true"/>
  <arg name="confidence_fps"      default="-1"/>

  <arg name="infra_width"         default="640"/>
  <arg name="infra_height"        default="480"/>
  <arg name="enable_infra"        default="true"/>
  <arg name="enable_infra1"       default="true"/>
  <arg name="enable_infra2"       default="true"/>
  <arg name="infra_rgb"           default="true"/>

  <arg name="color_width"         default="640"/>
  <arg name="color_height"        default="480"/>
  <arg name="enable_color"        default="true"/>

  <arg name="fisheye_fps"         default="-1"/>
  <arg name="depth_fps"           default="-1"/>
  <arg name="infra_fps"           default="30"/>
  <arg name="color_fps"           default="-1"/>
  <arg name="gyro_fps"            default="-1"/>
  <arg name="accel_fps"           default="-1"/>
  <arg name="enable_gyro"         default="true"/>
  <arg name="enable_accel"        default="true"/>

  <arg name="enable_pointcloud"         default="false"/>
  <arg name="pointcloud_texture_stream" default="RS2_STREAM_COLOR"/>
  <arg name="pointcloud_texture_index"  default="0"/>
  <arg name="allow_no_texture_points"   default="false"/>
  <arg name="emitter_enable"   		      default="false"/>
	
  <!-- rosparam set /camera/stereo_module/emitter_enabled false -->
  <rosparam>
  /camera/stereo_module/emitter_enabled: 0
  </rosparam>

  <rosparam if="$(arg emitter_enable)">
  /camera/stereo_module/emitter_enabled: 1
  </rosparam>
  
  <arg name="ordered_pc"                default="false"/>

  <arg name="enable_sync"               default="true"/>
  <arg name="align_depth"               default="true"/>

  <arg name="publish_tf"                default="true"/>
  <arg name="tf_publish_rate"           default="0"/>

  <arg name="filters"                   default=""/>
  <arg name="clip_distance"             default="-2"/>
  <arg name="linear_accel_cov"          default="0.01"/>
  <arg name="initial_reset"             default="false"/>
  <arg name="reconnect_timeout"         default="6.0"/>
  <arg name="wait_for_device_timeout"   default="-1.0"/>
  <arg name="unite_imu_method"          default="linear_interpolation"/>
  <arg name="topic_odom_in"             default="odom_in"/>
  <arg name="calib_odom_file"           default=""/>
  <arg name="publish_odom_tf"           default="true"/>

  <arg name="stereo_module/exposure/1"  default="7500"/>
  <arg name="stereo_module/gain/1"      default="16"/>
  <arg name="stereo_module/exposure/2"  default="1"/>
  <arg name="stereo_module/gain/2"      default="16"/>
  
  

  <group ns="$(arg camera)">
    <include file="$(find realsense2_camera)/launch/includes/nodelet.launch.xml">
      <arg name="tf_prefix"                value="$(arg tf_prefix)"/>
      <arg name="external_manager"         value="$(arg external_manager)"/>
      <arg name="manager"                  value="$(arg manager)"/>
      <arg name="output"                   value="$(arg output)"/>
      <arg name="respawn"                  value="$(arg respawn)"/>
      <arg name="serial_no"                value="$(arg serial_no)"/>
      <arg name="usb_port_id"              value="$(arg usb_port_id)"/>
      <arg name="device_type"              value="$(arg device_type)"/>
      <arg name="json_file_path"           value="$(arg json_file_path)"/>

      <arg name="enable_pointcloud"        value="$(arg enable_pointcloud)"/>
      <arg name="pointcloud_texture_stream" value="$(arg pointcloud_texture_stream)"/>
      <arg name="pointcloud_texture_index"  value="$(arg pointcloud_texture_index)"/>
      <arg name="enable_sync"              value="$(arg enable_sync)"/>
      <arg name="align_depth"              value="$(arg align_depth)"/>

      <arg name="fisheye_width"            value="$(arg fisheye_width)"/>
      <arg name="fisheye_height"           value="$(arg fisheye_height)"/>
      <arg name="enable_fisheye"           value="$(arg enable_fisheye)"/>

      <arg name="depth_width"              value="$(arg depth_width)"/>
      <arg name="depth_height"             value="$(arg depth_height)"/>
      <arg name="enable_depth"             value="$(arg enable_depth)"/>

      <arg name="confidence_width"         value="$(arg confidence_width)"/>
      <arg name="confidence_height"        value="$(arg confidence_height)"/>
      <arg name="enable_confidence"        value="$(arg enable_confidence)"/>
      <arg name="confidence_fps"           value="$(arg confidence_fps)"/>

      <arg name="color_width"              value="$(arg color_width)"/>
      <arg name="color_height"             value="$(arg color_height)"/>
      <arg name="enable_color"             value="$(arg enable_color)"/>

      <arg name="infra_width"              value="$(arg infra_width)"/>
      <arg name="infra_height"             value="$(arg infra_height)"/>
      <arg name="enable_infra"             value="$(arg enable_infra)"/>
      <arg name="enable_infra1"            value="$(arg enable_infra1)"/>
      <arg name="enable_infra2"            value="$(arg enable_infra2)"/>
      <arg name="infra_rgb"                value="$(arg infra_rgb)"/>

      <arg name="fisheye_fps"              value="$(arg fisheye_fps)"/>
      <arg name="depth_fps"                value="$(arg depth_fps)"/>
      <arg name="infra_fps"                value="$(arg infra_fps)"/>
      <arg name="color_fps"                value="$(arg color_fps)"/>
      <arg name="gyro_fps"                 value="$(arg gyro_fps)"/>
      <arg name="accel_fps"                value="$(arg accel_fps)"/>
      <arg name="enable_gyro"              value="$(arg enable_gyro)"/>
      <arg name="enable_accel"             value="$(arg enable_accel)"/>

      <arg name="publish_tf"               value="$(arg publish_tf)"/>
      <arg name="tf_publish_rate"          value="$(arg tf_publish_rate)"/>

      <arg name="filters"                  value="$(arg filters)"/>
      <arg name="clip_distance"            value="$(arg clip_distance)"/>
      <arg name="linear_accel_cov"         value="$(arg linear_accel_cov)"/>
      <arg name="initial_reset"            value="$(arg initial_reset)"/>
      <arg name="reconnect_timeout"        value="$(arg reconnect_timeout)"/>
      <arg name="wait_for_device_timeout"  value="$(arg wait_for_device_timeout)"/>
      <arg name="unite_imu_method"         value="$(arg unite_imu_method)"/>
      <arg name="topic_odom_in"            value="$(arg topic_odom_in)"/>
      <arg name="calib_odom_file"          value="$(arg calib_odom_file)"/>
      <arg name="publish_odom_tf"          value="$(arg publish_odom_tf)"/>
      <arg name="stereo_module/exposure/1" value="$(arg stereo_module/exposure/1)"/>
      <arg name="stereo_module/gain/1"     value="$(arg stereo_module/gain/1)"/>
      <arg name="stereo_module/exposure/2" value="$(arg stereo_module/exposure/2)"/>
      <arg name="stereo_module/gain/2"     value="$(arg stereo_module/gain/2)"/>

      <arg name="allow_no_texture_points"  value="$(arg allow_no_texture_points)"/>
      <arg name="ordered_pc"               value="$(arg ordered_pc)"/>
      
    </include>
  </group>
</launch>

      
```

# 9 rtabmap 跑 euroc  noVINS/VINS

```
编译成功 跑数据集的时候 出现odometry: waiting imu to initialize orientation (wait_imu_to_init=true)
在launch文件里面把参数imu_topic的值改成/imu0
```

```shell
//NOVINS
roslaunch rtabmap_examples euroc_datasets.launch MH_seq:=true

rosbag play --clock MH_01_easy.bag
//WITH-VINS
roslaunch rtabmap_examples euroc_datasets.launch args:="--Odom/Strategy 9 OdomVINS/ConfigPath ~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml" MH_seq:=true raw_images_for_odom:=true

rosbag play --clock MH_01_easy.bag
```





**查看是否链接到VINS**

```
root@e088e9462c1a:~/catkin_ws# rosrun rtabmap_odom rgbd_odometry --version
```

![](/home/sbim/Videos/5.jpg)



## 教程

```shell
git clone https://github.com/HKUST-Aerial-Robotics/VINS-Fusion.git 
cd VINS-Fusion 
```

**下载补丁**

https://gist.github.com/matlabbe/795ab37067367dca58bbadd8201d986c

**下载第一个**[**vins-fusion_be55a93_pull136.patch**](https://gist.github.com/matlabbe/795ab37067367dca58bbadd8201d986c#file-vins-fusion_be55a93_pull136-patch)到VINS-FUSION文件夹中

```shell
git apply vins-fusion_be55a93_pull136.patch
cd ~/catkin_ws
catkin build
```

**然后到rtabmap下**

```
cd ~/rtabmap/build
cmake -DCMAKE_INSTALL_PREFIX=~/catkin_ws/devel -DWITH_VINS=ON ..
make -j4
make install
```

**最后检查是否链接成功**

```shell
cd ~/catkin_ws
catkin build
source devel/setup.bash
rosrun rtabmap_odom rgbd_odometry --version
```

**测试euroc数据集**

```shell
//WITH-VINS
roslaunch rtabmap_examples euroc_datasets.launch args:="--Odom/Strategy 9 OdomVINS/ConfigPath ~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml" MH_seq:=true raw_images_for_odom:=true

rosbag play --clock MH_01_easy.bag
```

![image-20240312175012684](/home/sbim/.config/Typora/typora-user-images/image-20240312175012684.png)



## d435i 跑rtabmap

```shell
roslaunch rtabmap_examples test_d435i_vio.launch

roslaunch rtabmap_examples test_d435i_vio.launch args:="--Odom/Strategy 9 OdomVINS/ConfigPath ~/catkin_ws/src/VINS-Fusion/config/realsense_d435i/realsense_stereo_imu_config.yaml"

```

```
rosbag record -b 1024 -O my_bagfile_1.bag /camera/aligned_depth_to_color/camera_info  camera/aligned_depth_to_color/image_raw /camera/color/camera_info /camera/color/image_raw /camera/imu  /tf_static /camera/infra1/image_rect_raw /camera/infra2/image_rect_raw /camera/infra2/camera_info /camera/infra1/camera_info 
```



### 支线

```shell
roslaunch realsense2_camera opensourctracking.launch

rosbag record -O my_bagfile_2.bag /camera/aligned_depth_to_color/camera_info  camera/aligned_depth_to_color/image_raw /camera/color/camera_info /camera/color/image_raw /camera/imu /camera/imu_info /tf_static

```



realsense_d435i/realsense_stereo_imu_config.yaml

# 10 evo

```shell
 evo_traj tum MH_01-GT.tum -p plot_mode=xyz

evo_traj tum MH_01-GT.tum -p

```

### 1. 准备轨迹数据

首先，确保你的轨迹数据以 `evo` 支持的格式之一存储。如果你的数据是以ROS bag文件形式存在，并且包含了 `/odom` 话题，你可以直接使用 `evo` 从中提取轨迹数据。否则，你可能需要先将数据转换为 `evo` 支持的格式之一。

```shell
rosbag record -O odom_novins /rtabmap/odom

docker cp 3a2:/root/catkin_ws/data/odom_vins.bag /home/sbim/traj_ana/

```

### 2. 从ROS bag文件提取轨迹数据

如果你的轨迹数据存储在ROS bag文件中，使用以下命令从 `/odom` 话题提取轨迹数据：

```bash
evo_traj bag odom_vins.bag /rtabmap/odom --save_as_tum
```

### 3. 分析轨迹

```shell
evo_ape tum MH01_ground.txt  rtabmap_MH01.tum rtabmap_MH01_VINS.tum -va --plot --plot_mode xy

evo_traj tum  MH01_ground.txt rtabmap_MH01.tum rtabmap_MH01_VINS.tum --ref MH01_ground.txt -va -p
```

在误差图中，通常会显示几个不同的指标来评估估计轨迹与真实轨迹之间的误差。以下是常见的几个指标及其含义：

1. **APE（Absolute Pose Error）：绝对姿态误差**，即估计轨迹中每个位姿与真实轨迹对应位姿之间的误差。**APE越小表示估计轨迹与真实轨迹越接近。**
2. **RMSE（Root Mean Squared Error）：均方根误差**，表示所有误差的平方的均值的平方根。**RMSE越小表示估计轨迹整体与真实轨迹的误差越小。**
3. **Median Error：中位误差**，即所有误差按大小排序后的中间值。它可以反映估计轨迹的整体准确性。
4. **Mean Error：平均误差**，即所有误差的平均值。它表示估计轨迹的平均误差水平。
5. **Standard Deviation（STD）：标准差**，表示误差值的离散程度。**标准差越小表示误差值集中在均值附近**，反之则表示误差值分布较为分散。



**结论**

带了VINS的还更差（笑哭）



## 4.分析issac和vinsfusion 自录制包

```
evo_traj tum vinsfusion.txt issac.txt --ref vinsfusion.txt -va -p
```

![image-20240322172332108](/home/sbim/.config/Typora/typora-user-images/image-20240322172332108.png)



# 11 egoplanner

照着教程安装就行

https://github.com/ZJU-FAST-Lab/ego-planner

报错

## /usr/bin/ld: 找不到 -lpose_utils报错

Errors     << odom_visualization:make /home/labh/xtdrone_ws/logs/odom_visualization/build.make.002.log
/usr/bin/ld: 找不到 -lpose_utils
collect2: error: ld returned 1 exit status
make[2]: *** [/home/labh/xtdrone_ws/devel/.private/odom_visualization/lib/odom_visualization/odom_visualization] Error 1
make[1]: *** [CMakeFiles/odom_visualization.dir/all] Error 2
make: *** [all] Error 2

解决方法：

用locate命令定位XXX库文件

```
locate libpose_utils.so
```

得到结果为：

```
/home/labh/.local/share/Trash/files/Fast-Perching/devel/lib/libpose_utils.so
/home/labh/.local/share/Trash/files/devel.2/.private/pose_utils/lib/libpose_utils.so
/home/labh/.local/share/Trash/files/devel.2/lib/libpose_utils.so
```

再用软链接将上述中的任意一个与其链接起来

```
sudo ln -s /home/labh/.local/share/Trash/files/devel.2/lib/libpose_utils.so /usr/lib/libpose_utils.so
```





# 12 **查看当前摄像头可配置参数**

```
rs-enumerate-devices   **查看当前摄像头可配置参数**
```



root@3a24ef7ed95e:~/catkin_ws# roslaunch realsense2_camera rs_fusion_camera_stereo.launch
... logging to /root/.ros/log/8201d0c4-e778-11ee-83f5-0242ac110003/roslaunch-3a24ef7ed95e-1135889.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://3a24ef7ed95e:43527/





# 13 路径规划

![image-20240323110718878](/home/sbim/.config/Typora/typora-user-images/image-20240323110718878.png)

![image-20240323110728156](/home/sbim/.config/Typora/typora-user-images/image-20240323110728156.png)

![image-20240323120331319](/home/sbim/.config/Typora/typora-user-images/image-20240323120331319.png)



# 14 SLAM 

这几个都是基于ORB-SLAM2的 https://github.com/raulmur/ORB_SLAM2

**DS-SLAM         Semantic_slam**    **cube_slam**

https://github.com/floatlazer/semantic_slam?tab=readme-ov-file

https://github.com/shichaoy/cube_slam

https://github.com/ivipsourcecode/DS-SLAM?tab=readme-ov-file



pkg-config --modversion opencv

pkg-config --modversion eigen3

pkg-config --modversion pangolin

```
g++ -o output ORB_featrue.cpp `pkg-config --cflags --libs opencv4` -lrealsense2

```



# 15 仿真

## 配置PX4

```
sudo apt-get update 

sudo apt-get install unzip


sudo apt-get install python3-pip
pip3 install kconfiglib
pip3 install --user jinja2
pip3 install --user pyyaml
pip3 install --user jsonschema
pip3 install --user empy
pip3 install --user toml

pip3 install --user numpy
pip3 install --user packaging

sudo apt-get install libxml2-dev libxslt-dev -y
pip3 install lxml

sudo apt install ros-melodic-gazebo-ros-pkgs

打开~/.bashrc文件，添加以下内容：

source ~/PX4-Autopilot/Tools/setup_gazebo.bash ~/PX4-Autopilot/ ~/PX4-Autopilot/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot/Tools/sitl_gazebo

source ~/.bashrc 
```







## Mavros配置

**二进制安装**

建一个工作空间

然后catkin build

```
sudo apt install ros-melodic-mavros ros-melodic-mavros-extras

wget https://gitee.com/robin_shaun/XTDrone/raw/master/sitl_config/mavros/install_geographiclib_datasets.sh

sudo chmod a+x ./install_geographiclib_datasets.sh
# 下面这步需要装⼀段时间,可以耐⼼等待
sudo ./install_geographiclib_datasets.sh
# 如果没问题，终端输出的内容如下：
```

![image-20240323174756640](/home/sbim/.config/Typora/typora-user-images/image-20240323174756640.png)



## QGC

**安装QGC地面站，可以在下面的网址下载**

https://github.com/mavlink/qgroundcontrol/releases/tag/v4.1.5

wget https://github.com/mavlink/qgroundcontrol/releases/download/v4.1.5/QGroundControl.AppImage

sudo apt-get install espeak libespeak-dev libudev-dev libsdl1.2-dev

**选择 Appimage格式下载**

下载后在地面站目录下执行：

```
sudo apt install libpulse0 -y
sudo apt-get install libpulse-mainloop-glib0 -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libfuse2 -y
sudo apt install libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor0 -y

sudo apt-get dist-upgrade -y 
```



```
chmod +x QGroundControl.AppImage
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
./QGroundControl.AppImage
```





XTDRONE

```
git clone https://gitee.com/robin_shaun/XTDrone.git
cd XTDrone
git submodule update --init --recursive
cp sensing/gimbal/gazebo_gimbal_controller_plugin.cpp ~/PX4-Autopilot/Tools/sitl_gazebo/src/
cp sitl_config/init.d-posix/rcS ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/
cp sitl_config/worlds/* ~/PX4-Autopilot/Tools/sitl_gazebo/worlds/
cp -r sitl_config/models/* ~/PX4-Autopilot/Tools/sitl_gazebo/models/ 
cp -r sitl_config/launch/* ~/PX4-Autopilot/launch/
cd ~/.gazebo/models/
rm -r stereo_camera/ 3d_lidar/ 3d_gpu_lidar/ hokuyo_lidar/
```

#  16 ORB-slam3 

```
xhost +
docker run -it -v /tmp/.X11-unix:/tmp/.X11-unix:rw --privileged --gpus all -e DISPLAY=:1 -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all -e PYTHONUNBUFFERED=1 -e QT_X11_NO_MITSHM=1 orbslam3:latest /bin/bash

docker build -t myapp:v1 .
```



```
docker exec -it b14 /bin/bash


./Examples/Stereo/stereo_euroc ./Vocabulary/ORBvoc.txt ./Examples/Stereo/EuRoC.yaml ../Datasets/EuRoC/MH01 ./Examples/Stereo/EuRoC_TimeStamps/MH01.txt

./Monocular/mono_euroc ../Vocabulary/ORBvoc.txt ./Monocular/EuRoC.yaml ../Datasets/EuRoC/MH01 ./Monocular/EuRoC_TimeStamps/MH01.txt
```





# 17 语义slam 语义通信

过程是这样的



前端：

无人机采集数据，并通过语义slam 语义分割出动态信息和静态信息或者特殊信息



通过边缘设备服务器，传输这些分割的点云信息



后端：

接收这些信息并且三维建图，能够在三维图像中将分割的信息表示出来



在持续的建图过程中，由无人机传输的信息不一定是全部的点云信息，

只是将新加进来的信息传输，减少运算量。避免在后端重复建图。

静态不变，动态识别加入，



将语义SLAM（Simultaneous Localization and Mapping，同时定位与地图构建）和语义传输结合起来的想法是非常前沿且具有挑战性的，这种方法有潜力极大提高无人机在未知环境中的导航、识别和地图构建能力。在你的场景中，无人机采集的数据不仅用于建图和导航，还通过边缘计算设备进行处理和传输，减少了数据量并提高了处理速度。下面我会概述一个可能的系统框架、实验设计和一些可能的评估指标。

## 系统框架

**前端（数据采集与初步处理）**：

**无人机采集数据**：无人机配备传感器（如RGB相机、激光雷达、深度相机）收集周围环境的数据。这些传感器采集包括颜色信息、距离信息和3D结构信息。

**语义SLAM处理**：采集的数据实时进行语义分割，区分动态对象（如行人、车辆）和静态对象（如建筑、道路）。



实时进行语义分割的SLAM系统需要基于深度学习模型进行。这些模型能够识别和区分图像中的不同对象，如动态对象（行人、车辆）和静态对象（建筑、道路）。

- 采用现有的深度学习框架（如TensorFlow或PyTorch）和预训练的语义分割模型（如DeepLab、Mask R-CNN）可以实现这一步骤。关键是将这些模型整合进SLAM算法中，实现实时性能。



**数据传输：**

**边缘设备服务器**：作为数据传输的中介，优化和转发无人机发送的数据，可能包括数据压缩、格式转换等。

- 边缘计算设备可以在数据传输前对数据进行预处理，包括数据压缩、格式转换等。这有助于减少传输的数据量和提高传输效率。
- 利用现有的边缘计算技术和协议（如5G网络技术），可以实现高效的数据预处理和快速传输。



**数据预处理**：对分割后的点云数据进行压缩或优化处理，准备传输。

- 对分割后的点云数据进行压缩或优化处理，例如通过点云压缩算法减少数据大小，同时保留重要的几何和语义信息。

- 现有的点云处理库（如PCL—Point Cloud Library）可以用于数据预处理。



**后端（数据接收与深度处理）**：

**数据接收与三维建模**：后端系统接收传输的数据，并基于这些数据进行三维重建。

**动态更新与优化**：随着新数据的持续接收，后端系统动态更新地图，避免重复建模，并实施算法优化以提高效率和准确性。



## 实验设计

不同实验环境：选择或构建具有不同动静态对象的环境，确保环境多样化以验证系统的鲁棒性和准确性。



1. **数据采集**：使用无人机在上述环境中飞行，收集必要的数据。
2. **系统实现**：根据上述框架开发完整的系统，包括数据采集、处理、传输和接收三维建模。
3. **性能评估**：在不同条件下测试系统，评估其性能。



## 评估指标

- **准确性**：三维建图的准确度，可以通过比较生成的模型与真实环境的差异来评估。
- **效率**：系统处理数据和更新地图的速度。
- **带宽占用**：数据传输过程中的带宽占用，考虑到使用边缘计算设备的目的之一就是减少数据量。
- **系统鲁棒性**：系统在不同环境、不同动态对象出现频率下的表现。





大多数系统采用的典型策略是仅跟踪相机相对于静态
背景的运动，并将移动物体视为离群值，其3D几何形状和
运动不随时间建模。然而，在机器人应用中，机器人最感
兴趣的往往是前景中移动的物体。



许多传统SLAM和密集重建系统背后的核心潜在假
设是，场景在很大程度上是静态的。如何在不影响实
时性能的情况下，将这些密集系统扩展到跟踪和重建
多个模型



另一方面，如果一个标签消失了，并且在一定数量
的帧内没有重新出现，则假定相应的模型离开了场景。
在这种情况下，模型将被添加到非活动列表，如果它包含足够的高置信度的
冲浪，否则被删除。

# 18 一文四问





![image-20240402175026587](/home/sbim/.config/Typora/typora-user-images/image-20240402175026587.png)



![image-20240402175035757](/home/sbim/.config/Typora/typora-user-images/image-20240402175035757.png)

![image-20240402175048779](/home/sbim/.config/Typora/typora-user-images/image-20240402175048779.png)
