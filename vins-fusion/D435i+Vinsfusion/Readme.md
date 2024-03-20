

# 1.【基于Ubuntu18.04+Melodic的realsense D435i安装】

https://developer.aliyun.com/article/1303849#:~:text=%E5%9F%BA%E4%BA%8EUbuntu18.04%2BMelodic%E7%9A%84realsense,D435%E5%AE%89%E8%A3%85%EF%BC%8C%E9%A6%96%E5%85%88SDK%E5%AE%89%E8%A3%85%EF%BC%8C%E5%90%8E%E9%9D%A2%E7%BB%99%E5%87%BA%E4%B8%A4%E4%B8%AA%E5%AE%89%E8%A3%85realsense-ros%EF%BC%8C%E6%9C%9F%E5%BE%85%E4%BD%A0%E7%9A%84%E5%85%B3%E6%B3%A8%EF%BC%8C%E5%90%8E%E6%9C%9F%E6%88%91%E4%BC%9A%E6%8C%81%E7%BB%AD%E6%9B%B4%E6%96%B0ROS%E4%B8%ADD435i%E7%9A%84%E5%AE%89%E8%A3%85%E4%BD%BF%E7%94%A8





# 2.使用Realsense D435i运行VINS-Fusion并建图

**参考：**

https://blog.csdn.net/qq_38364548/article/details/124955452#t8

https://blog.csdn.net/sinat_16643223/article/details/115276730

https://blog.csdn.net/zsy00100/article/details/126228841?spm=1001.2101.3001.6650.1&utm_medium=distribute.pc_relevant.none-task-blog-2%7Edefault%7ECTRLIST%7ERate-1-126228841-blog-123612571.235%5Ev43%5Epc_blog_bottom_relevance_base9&depth_1-utm_source=distribute.pc_relevant.none-task-blog-2%7Edefault%7ECTRLIST%7ERate-1-126228841-blog-123612571.235%5Ev43%5Epc_blog_bottom_relevance_base9&utm_relevant_index=2

**Realsense D435i关闭IR结构光**

https://blog.csdn.net/Hanghang_/article/details/103612300



# docekr 运行 Realsense D435i运行VINS-Fusion并建图

## 1.先决条件

### 1.1 安装docker

https://linux.cn/article-14871-1.html

### 1.2 拉取vins-fusion的image

```
docker pull jianchong/vins-fusion
```

### 1.3 运行vins-fusion

```
xhost +

docker run -it -v /tmp/.X11-unix:/tmp/.X11-unix:rw --privileged --gpus all -e DISPLAY=:1 -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all -e PYTHONUNBUFFERED=1 -e QT_X11_NO_MITSHM=1 shangyeyuancainai/18.04-vins-fusion:v1 /bin/bash
```

**或**

```
docker run -it -e DISPLAY=$DISPLAY --volume="/tmp/.X11-unix:/tmp/.X11-unix" jianchong/vins-fusion /bin/bash
```

### 1.4 系统硬件环境

```
系统：Ubuntu 18.04+  ROS melodic 
设备：Realsense D435i+USB3.2 TypeC数据线
```

备注：由于相机数据量较大，保证数据传输的稳定性，务必采用USB3.0以上的TypeC数据线，否则普通数据线会卡帧。

### 1.5 RealSense SDK安装

#### 克隆SDK

```
git clone https://github.com/IntelRealSense/librealsense
cd librealsense
```

#### 安装相关依赖

```
sudo apt-get install libudev-dev pkg-config libgtk-3-dev
sudo apt-get install libusb-1.0-0-dev pkg-config
sudo apt-get install libglfw3-dev
sudo apt-get install libssl-dev
```

#### 安装权限脚本

```
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && udevadm trigger
```

#### 进行编译与安装

```
mkdir build
cd build
cmake ../ -DBUILD_EXAMPLES=true
make
sudo make install
```

#### 测试安装是否成功

```
realsense-viewer
```

![](/home/sbim/SLAM_LEARN/vins-fusion/D435i+Vinsfusion/picture/screenshot-20240308-144819.png)



### 1.6 D435i 安装ROS接口

先安装一下

```
sudo apt-get install ros-melodic-rgbd-launch
```

#### realsense—ros源码安装

这里新建进入在~/realsense_ws/src目录下(放在catkin_ws下就行)

```
mkdir realsense_ws/src
cd realsense_ws/src
```

然后直接进入官网仓库地址：[https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy](https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy?spm=a2c6h.13046898.publish-article.4.37266ffaZn7N1w)

**需要下载分支ros1-legacy**

从github下载相关的包，然后进行编译：

```
git clone -b ros1-legacy https://github.com/IntelRealSense/realsense-ros.git
git clone https://github.com/pal-robotics/ddynamic_reconfigure.git
cd ~/realsense_ws && catkin_make
```

**编译完成后**，**使用如下命令测试**：

编译、然后更新一下source ~/.bashrc

```
echo "source ~/realsense_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**检查摄像头是否正常连接，命令测试**

```
roslaunch realsense2_camera demo_pointcloud.launch 
#点云deemo
rqt_image_view 
#rqt查看图像
```

即可启动RealSense的ROS节点。



## 2.Intel RealSense D435i实时运行VINS-fusion

#### 修改配置文件yaml

在config文件夹里面

#### 修改相机启动文件

在config文件夹里面

放在~/catkin_ws/src/realsense/realsense2_camera/launch里面

#### 运行

**编译** 

```
catkin build
source devel/setup.bash
```

**运行**

```
roslaunch vins vins_rviz.launch
//加载配置文件
rosrun vins vins_node src/VINS-Fusion/config/realsense_d435i/realsense_stereo_imu_config.yaml
//回环
rosrun loop_fusion loop_fusion_node src/VINS-Fusion/config/realsense_d435i/realsense_stereo_imu_config.yaml
//启动相机
roslaunch realsense2_camera rs_fusion_camera_stereo.launch 
```

![](/home/sbim/SLAM_LEARN/vins-fusion/D435i+Vinsfusion/picture/screenshot-20240308-150736.png)



## 使用Vins-fusion建立Octomap

#### 编译安装 OctomapServer 建图包

```cobol
cd src/
git clone https://github.com/OctoMap/octomap_mapping.git
```

 返回你的工作空间主目录，安装下依赖，然后开始编译：

```bash
cd ../
rosdep install octomap_mapping
sudo apt-get install ros-melodic-octomap-ros
catkin build
```

**ROS 中提供了一个 Rviz 可视化 Octomap 的插件，如果没有安装使用下面的命令即可：**

```csharp
sudo apt-get install ros-melodic-octomap-rviz-plugins
```

其中的OccupancyGrid是显示三维概率地图，也就是octomap地图。OccupancyMap是显示二维占据栅格地图。



rostopic list看到vins和相机发布的话题，**其中的/vins_estimator/point_cloud即是我们需要的点云话题**。但是由于我们使用的点云转octomap需要的是pointcloud2，所以我们需要先把pointcloud转换为pointcloud2。

```
cd ~/catkin_ws/src
git clone https://github.com/1332927388/pcl2octomap.git
cd ~/catkin_ws && catkin_make
```

编辑:

~/catkin_ws/src/octomap_mapping/octomap_server/launch中的octomap_mapping.launch，将两个launch文件合二为一

```c++
<!-- 
  Example launch file for octomap_server mapping: 
  Listens to incoming PointCloud2 data and incrementally builds an octomap. 
  The data is sent out in different representations. 
  Copy this file into your workspace and adjust as needed, see
  www.ros.org/wiki/octomap_server for details  
-->
<launch>
	<node pkg="point_cloud_converter" name="point_cloud_converter" type="point_cloud_converter_node" >
		<remap from="points_in" to="/vins_estimator/point_cloud"/>
		<remap from="points2_out" to="/points" />
	</node>	
 
	<node pkg="octomap_server" type="octomap_server_node" name="octomap_server">
		<param name="resolution" value="0.05" />
		
		<!-- fixed map frame (set to 'map' if SLAM or localization running!) -->
		<param name="frame_id" type="string" value="world" />
		
		<!-- maximum range to integrate (speedup!) -->
		<param name="sensor_model/max_range" value="5.0" />
		
		<!-- data source to integrate (PointCloud2) -->
		<remap from="cloud_in" to="points" />
	
	</node>
</launch>
```

启动octomap_server

```undefined
roslaunch octomap_server octomap_mapping.launch
```

![](/home/sbim/SLAM_LEARN/vins-fusion/D435i+Vinsfusion/picture/screenshot-20240308-151702.png)

启动 octomap_server 节点后，可以使用它提供的地图保存服务
保存压缩的二进制存储格式地图：

```cobol
rosrun octomap_server octomap_saver 2022.bt
```

安装Octovis可视化工具可以查看八叉树模型效果

```csharp
sudo apt-get install octovis
```

查看ocotmap.bt八叉树地图文件

```cobol
octovis 2022.bt
```



# 3.使用D435i运行Rtabmap

参考：
https://blog.csdn.net/Starry_Sheep/article/details/124725504

https://blog.csdn.net/rosfreshman/article/details/116404077

https://github.com/IntelRealSense/realsense-ros/wiki/SLAM-with-D435i#personalize-rviz



## 3.1安装rtabmap

```shell
sudo apt-get install ros-melodic-rtabmap ros-melodic-rtabmap-ros 

sudo apt-get remove ros-melodic-rtabmap ros-melodic-rtabmap-ros
```

**这个不需要安装在工作目录下**

```shell
cd 
git clone https://github.com/introlab/rtabmap.git rtabmap
cd rtabmap/build
cmake -DCMAKE_INSTALL_PREFIX=~/catkin_ws/devel ..
make -j4
make install
```

**安装RTABMAP_ROS**
**这个安装在工作目录下**

```shell
cd catkin_ws
git clone https://github.com/introlab/rtabmap_ros.git rtabmap_ros
catkin build
```

这样就安装好了没有链接vins的rtabmap



## 3.2 测试安装成功没

**rtabmap 跑 euroc**

```shell
//NOVINS
roslaunch rtabmap_examples euroc_datasets.launch MH_seq:=true

rosbag play --clock MH_01_easy.bag
```

```shell
编译成功 跑数据集的时候 出现odometry: waiting imu to initialize orientation (wait_imu_to_init=true)
在launch文件里面把参数imu_topic的值改成/imu0
```

## 3.3 链接 VINS

**查看是否链接到VINS**

```shell
root@e088e9462c1a:~/catkin_ws# rosrun rtabmap_odom rgbd_odometry --version
```

![](/home/sbim/Videos/5.jpg)



**教程**

下载补丁

https://gist.github.com/matlabbe/795ab37067367dca58bbadd8201d986c

**下载第一个**[**vins-fusion_be55a93_pull136.patch**](https://gist.github.com/matlabbe/795ab37067367dca58bbadd8201d986c#file-vins-fusion_be55a93_pull136-patch)到VINS-FUSION文件夹中

```shell
git apply vins-fusion_be55a93_pull136.patch
cd ~/catkin_ws
catkin build
```

**然后到rtabmap下**

```shell
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

```shell
//支线：https://github.com/IntelRealSense/realsense-ros/wiki/SLAM-with-D435i#personalize-rviz

sudo apt-get install ros-kinetic-imu-filter-madgwick

sudo apt-get install ros-kinetic-rtabmap-ros

sudo apt-get install ros-kinetic-robot-localization

roslaunch realsense2_camera opensource_tracking.launch
```



## 3.4 用D435I运行vin-rtabmap

**要运行有画面，要将rs_fusion_camera_stereo.launch文件中的改成这个(不知道为什么640不行，还没解决)**

```c++
  <arg name="infra_width"         default="848"/>
```

**运行**

```shell
roslaunch rtabmap_examples test_d435i_vio.launch

roslaunch rtabmap_examples test_d435i_vio.launch args:="--Odom/Strategy 9 OdomVINS/ConfigPath ~/catkin_ws/src/VINS-Fusion/config/realsense_d435i/realsense_stereo_imu_config.yaml"
```

**而且运行也不是很稳定**





















