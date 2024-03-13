# TASKS

- jetson nano本地录个包 ，发给另一台电脑跑一下，能很好运行
- 录一个场景大一点的包，开回环不开回环都测一下，能很好运行
- 建立一个完整的docker
- ros1 的包给ros2跑试下，能很好运行
- 有路径估计，定位，轨迹规划
- xtdrone 仿真环境搭建，部署egoplanner
- 

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

# GPT4问问题 ip问题

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



# 保存

```
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





# evo

```shell
evo_traj tum MH_01-GT.tum -p plot_mode=xyz

evo_traj tum MH_01-GT.tum -p

```

