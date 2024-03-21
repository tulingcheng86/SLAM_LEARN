# TASKS

- [x] jetson nano本地录个包 ，发给另一台电脑跑一下，能很好运行
- [x] 录一个场景大一点的包，开回环不开回环都测一下，能很好运行
- [x] 建立一个完整的docker
- [x] ros1 的包给ros2跑试下，能很好运行
- [ ] 有路径估计，定位，轨迹规划
- [ ] xtdrone 仿真环境搭建，部署egoplanner
- [ ] d435i录制包 离线跑vio-rtabmap 看要怎么编写相机启动函数和录制哪些话题才能启动
- [ ] 



# 去除结构光

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

# 录制bag

```
  rosbag record -O d435i-test.bag /camera/imu /camera/infra1/image_rect_raw /camera/infra2/image_rect_raw /camera/infra1/camera_info /camera/infra2/camera_info


  colcon build --packages-select realsense2_camera
```

# 查看QOS

tlc@192.168.100.236
ros2 topic info --verbose /camera/camera/imu

export ROS_DOMAIN_ID=30

# ip问题 GPT4问问题 

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

# SCP复制
```shell
scp jetson@192.168.100.207:/home/jetson/realsense_ws/data/d435ii.bag /你的本地地址/
```

# .sh
```shell
roslaunch vins vins_rviz.launch
rosrun vins vins_node src/VINS-Fusion/config/realsense_d435i/realsense_stereo_imu_config.yaml

rosrun loop_fusion loop_fusion_node src/VINS-Fusion/config/realsense_d435i/realsense_stereo_imu_config.yaml 

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



# 容器启动 /构建 命令
```shell
xhost +
docker run -it -v /tmp/.X11-unix:/tmp/.X11-unix:rw --privileged --gpus all -e DISPLAY=:1 -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all -e PYTHONUNBUFFERED=1 -e QT_X11_NO_MITSHM=1 wdczz/xtdrone:2.3 /bin/bash

docker build -t myapp:v1 .
```



# rtabmap

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





# 保存

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

# rtabmap 跑 euroc  noVINS/VINS

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

# evo

```shell
 evo_traj tum MH_01-GT.tum -p plot_mode=xyz

evo_traj tum MH_01-GT.tum -p

```



# egoplanner

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





# 命

rs-enumerate-devices



root@3a24ef7ed95e:~/catkin_ws# roslaunch realsense2_camera rs_fusion_camera_stereo.launch
... logging to /root/.ros/log/8201d0c4-e778-11ee-83f5-0242ac110003/roslaunch-3a24ef7ed95e-1135889.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://3a24ef7ed95e:43527/

SUMMARY
========

PARAMETERS
 * /camera/realsense2_camera/accel_fps: 250
 * /camera/realsense2_camera/accel_frame_id: camera_accel_frame
 * /camera/realsense2_camera/accel_optical_frame_id: camera_accel_opti...
 * /camera/realsense2_camera/align_depth: True
 * /camera/realsense2_camera/aligned_depth_to_color_frame_id: camera_aligned_de...
 * /camera/realsense2_camera/aligned_depth_to_fisheye1_frame_id: camera_aligned_de...
 * /camera/realsense2_camera/aligned_depth_to_fisheye2_frame_id: camera_aligned_de...
 * /camera/realsense2_camera/aligned_depth_to_fisheye_frame_id: camera_aligned_de...
 * /camera/realsense2_camera/aligned_depth_to_infra1_frame_id: camera_aligned_de...
 * /camera/realsense2_camera/aligned_depth_to_infra2_frame_id: camera_aligned_de...
 * /camera/realsense2_camera/allow_no_texture_points: False
 * /camera/realsense2_camera/base_frame_id: camera_link
 * /camera/realsense2_camera/calib_odom_file: 
 * /camera/realsense2_camera/clip_distance: -2.0
 * /camera/realsense2_camera/color_fps: 30
 * /camera/realsense2_camera/color_frame_id: camera_color_frame
 * /camera/realsense2_camera/color_height: 480
 * /camera/realsense2_camera/color_optical_frame_id: camera_color_opti...
 * /camera/realsense2_camera/color_width: 640
 * /camera/realsense2_camera/confidence_fps: -1
 * /camera/realsense2_camera/confidence_height: -1
 * /camera/realsense2_camera/confidence_width: -1
 * /camera/realsense2_camera/depth_fps: 30
 * /camera/realsense2_camera/depth_frame_id: camera_depth_frame
 * /camera/realsense2_camera/depth_height: 480
 * /camera/realsense2_camera/depth_optical_frame_id: camera_depth_opti...
 * /camera/realsense2_camera/depth_width: 640
 * /camera/realsense2_camera/device_type: 
 * /camera/realsense2_camera/enable_accel: True
 * /camera/realsense2_camera/enable_color: True
 * /camera/realsense2_camera/enable_confidence: True
 * /camera/realsense2_camera/enable_depth: True
 * /camera/realsense2_camera/enable_fisheye1: False
 * /camera/realsense2_camera/enable_fisheye2: False
 * /camera/realsense2_camera/enable_fisheye: False
 * /camera/realsense2_camera/enable_gyro: True
 * /camera/realsense2_camera/enable_infra1: True
 * /camera/realsense2_camera/enable_infra2: True
 * /camera/realsense2_camera/enable_infra: True
 * /camera/realsense2_camera/enable_pointcloud: False
 * /camera/realsense2_camera/enable_pose: False
 * /camera/realsense2_camera/enable_sync: True
 * /camera/realsense2_camera/filters: 
 * /camera/realsense2_camera/fisheye1_frame_id: camera_fisheye1_f...
 * /camera/realsense2_camera/fisheye1_optical_frame_id: camera_fisheye1_o...
 * /camera/realsense2_camera/fisheye2_frame_id: camera_fisheye2_f...
 * /camera/realsense2_camera/fisheye2_optical_frame_id: camera_fisheye2_o...
 * /camera/realsense2_camera/fisheye_fps: -1
 * /camera/realsense2_camera/fisheye_frame_id: camera_fisheye_frame
 * /camera/realsense2_camera/fisheye_height: -1
 * /camera/realsense2_camera/fisheye_optical_frame_id: camera_fisheye_op...
 * /camera/realsense2_camera/fisheye_width: -1
 * /camera/realsense2_camera/gyro_fps: 200
 * /camera/realsense2_camera/gyro_frame_id: camera_gyro_frame
 * /camera/realsense2_camera/gyro_optical_frame_id: camera_gyro_optic...
 * /camera/realsense2_camera/imu_optical_frame_id: camera_imu_optica...
 * /camera/realsense2_camera/infra1_frame_id: camera_infra1_frame
 * /camera/realsense2_camera/infra1_optical_frame_id: camera_infra1_opt...
 * /camera/realsense2_camera/infra2_frame_id: camera_infra2_frame
 * /camera/realsense2_camera/infra2_optical_frame_id: camera_infra2_opt...
 * /camera/realsense2_camera/infra_fps: 30
 * /camera/realsense2_camera/infra_height: 480
 * /camera/realsense2_camera/infra_rgb: False
 * /camera/realsense2_camera/infra_width: 640
 * /camera/realsense2_camera/initial_reset: False
 * /camera/realsense2_camera/json_file_path: 
 * /camera/realsense2_camera/linear_accel_cov: 0.01
 * /camera/realsense2_camera/odom_frame_id: camera_odom_frame
 * /camera/realsense2_camera/ordered_pc: False
 * /camera/realsense2_camera/pointcloud_texture_index: 0
 * /camera/realsense2_camera/pointcloud_texture_stream: RS2_STREAM_COLOR
 * /camera/realsense2_camera/pose_frame_id: camera_pose_frame
 * /camera/realsense2_camera/pose_optical_frame_id: camera_pose_optic...
 * /camera/realsense2_camera/publish_odom_tf: True
 * /camera/realsense2_camera/publish_tf: True
 * /camera/realsense2_camera/reconnect_timeout: 6.0
 * /camera/realsense2_camera/rosbag_filename: 
 * /camera/realsense2_camera/serial_no: 
 * /camera/realsense2_camera/stereo_module/exposure/1: 7500
 * /camera/realsense2_camera/stereo_module/exposure/2: 1
 * /camera/realsense2_camera/stereo_module/gain/1: 16
 * /camera/realsense2_camera/stereo_module/gain/2: 16
 * /camera/realsense2_camera/tf_publish_rate: 0.0
 * /camera/realsense2_camera/topic_odom_in: odom_in
 * /camera/realsense2_camera/unite_imu_method: linear_interpolation
 * /camera/realsense2_camera/usb_port_id: 
 * /camera/realsense2_camera/wait_for_device_timeout: -1.0
 * /camera/stereo_module/emitter_enabled: 0
 * /rosdistro: melodic
 * /rosversion: 1.14.13

NODES
  /camera/
    realsense2_camera (nodelet/nodelet)
    realsense2_camera_manager (nodelet/nodelet)

auto-starting new master
process[master]: started with pid [1135913]
ROS_MASTER_URI=http://localhost:11311

setting /run_id to 8201d0c4-e778-11ee-83f5-0242ac110003
process[rosout-1]: started with pid [1135924]
started core service [/rosout]
process[camera/realsense2_camera_manager-2]: started with pid [1135931]
[ INFO] [1711021526.440742210]: Initializing nodelet with 20 worker threads.
process[camera/realsense2_camera-3]: started with pid [1135932]
[ INFO] [1711021526.759582509]: RealSense ROS v2.3.2
[ INFO] [1711021526.759600377]: Built with LibRealSense v2.54.2
[ INFO] [1711021526.759606778]: Running with LibRealSense v2.54.2
[ INFO] [1711021526.775284771]:  
[ INFO] [1711021526.789775419]: Device with serial number 207522071003 was found.

[ INFO] [1711021526.789792154]: Device with physical ID /sys/devices/pci0000:00/0000:00:14.0/usb2/2-1/2-1.4/2-1.4:1.0/video4linux/video0 was found.
[ INFO] [1711021526.789797067]: Device with name Intel RealSense D435I was found.
[ INFO] [1711021526.789942395]: Device with port number 2-1.4 was found.
[ INFO] [1711021526.789951302]: Device USB type: 3.2
[ INFO] [1711021526.790677910]: getParameters...
[ INFO] [1711021526.805155905]: setupDevice...
[ INFO] [1711021526.805168321]: JSON file is not provided
[ INFO] [1711021526.805173244]: ROS Node Namespace: camera
[ INFO] [1711021526.805178843]: Device Name: Intel RealSense D435I
[ INFO] [1711021526.805189273]: Device Serial No: 207522071003
[ INFO] [1711021526.805195309]: Device physical port: /sys/devices/pci0000:00/0000:00:14.0/usb2/2-1/2-1.4/2-1.4:1.0/video4linux/video0
[ INFO] [1711021526.805202546]: Device FW version: 5.13.0.50
[ INFO] [1711021526.805208172]: Device Product ID: 0x0B3A
[ INFO] [1711021526.805215732]: Enable PointCloud: Off
[ INFO] [1711021526.805221561]: Align Depth: On
[ INFO] [1711021526.805229016]: Sync Mode: On
[ INFO] [1711021526.805250554]: Device Sensors: 
[ INFO] [1711021526.806424657]: Stereo Module was found.
[ INFO] [1711021526.809699682]: RGB Camera was found.
[ INFO] [1711021526.809784740]: Motion Module was found.
[ INFO] [1711021526.809798491]: (Infrared, 0) sensor isn't supported by current device! -- Skipping...
[ INFO] [1711021526.809806368]: (Confidence, 0) sensor isn't supported by current device! -- Skipping...
[ INFO] [1711021526.810050005]: num_filters: 1
[ INFO] [1711021526.810057951]: Setting Dynamic reconfig parameters.
[ INFO] [1711021526.827482913]: Done Setting Dynamic reconfig parameters.
[ INFO] [1711021526.827755054]: depth stream is enabled - width: 640, height: 480, fps: 30, Format: Z16
[ INFO] [1711021526.827866660]: infra1 stream is enabled - width: 640, height: 480, fps: 30, Format: Y8
[ INFO] [1711021526.827971856]: infra2 stream is enabled - width: 640, height: 480, fps: 30, Format: Y8
[ INFO] [1711021526.828248937]: color stream is enabled - width: 640, height: 480, fps: 30, Format: RGB8
[ INFO] [1711021526.829527096]: gyro stream is enabled - fps: 200
[ WARN] [1711021526.829536405]: No mathcing profile found for accel with fps=250
[ WARN] [1711021526.829543684]: Using default profile instead.
[ INFO] [1711021526.829551444]: accel stream is enabled - fps: 100
[ INFO] [1711021526.829561380]: setupPublishers...
[ INFO] [1711021526.830500145]: Expected frequency for depth = 30.00000
[ INFO] [1711021526.840611752]: Expected frequency for infra1 = 30.00000
[ INFO] [1711021526.845801057]: Expected frequency for infra2 = 30.00000
[ INFO] [1711021526.851474101]: Expected frequency for color = 30.00000
[ INFO] [1711021526.856751386]: Expected frequency for aligned_depth_to_color = 30.00000
[ INFO] [1711021526.862299998]: Start publisher IMU
[ INFO] [1711021526.862947300]: setupStreams...
 21/03 11:45:26,874 WARNING [135906503403264] (ds-calib-parsers.cpp:41) IMU Calibration is not available, default intrinsic and extrinsic will be used.
[ INFO] [1711021526.913375251]: SELECTED BASE:Depth, 0
[ INFO] [1711021526.916654010]: RealSense Node Is Up!
[ WARN] [1711021526.983944839]: 
[ WARN] [1711021527.827781332]: Hardware Notification:IR stream start failure,1.71102e+12,Error,Hardware Error
^C[camera/realsense2_camera-3] killing on exit
[camera/realsense2_camera_manager-2] killing on exit
[rosout-1] killing on exit
