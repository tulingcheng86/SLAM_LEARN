首先安装ROS2

# 1、ROS2安装（换源后装的快点 不然太慢了 这东西还是看运气）

[ROS2安装方法 - ROS2入门教程 (guyuehome.com)](https://book.guyuehome.com/ROS2/1.系统架构/1.3_ROS2安装方法/)

 

如果以下命令

sudo curl -sSL [https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg](https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg)
 出错的话

[彻底解决【“curl: (7) Failed to connect to raw.githubusercontent.com port 443: Connection refused”】错误_curl: (7) failed to connect to nodejs.org port 443-CSDN博客](https://blog.csdn.net/donaldsy/article/details/107482368)

 

Gitclone老是失败

 

[解决Linux系统git clone失败或超时问题-CSDN博客](https://blog.csdn.net/weixin_42771853/article/details/133135301#:~:text=文章浏览阅读1k次。 首先使用 sudo vim %2Fetc%2Fhosts 进入hosts，此时是查看模式。,使用git clone常常不成功，以下是解决办法，亲测有效。 按下 i 进入编辑模式，此时需要插入两个IP地址。 2. 重启网络和服务器。)**（这个测试过后有用！）用这个就行了** **非常有用！！！**

 

法二：看运气 一般般有用

sudo vim /etc/hosts

添加

192.30.253.113  github.com

192.30.252.131  github.com

185.31.16.185  github.global.ssl.fastly.net

74.125.237.1  dl-ssl.google.com

173.194.127.200  groups.google.com

74.125.128.95  ajax.googleapis.com

 

 

# 2、安装pangolin

https://github.com/stevenlovegrove/Pangolin

安装pangolin git clone --recursive https://github.com/stevenlovegrove/Pangolin.git 

![计算机生成了可选文字: cdPangolin-e.5/ mkdirbuild&&cdbuild sudomakeinstall](file:///C:/Users/BT7274/AppData/Local/Temp/msohtmlclip1/01/clip_image002.jpg)

![计算机生成了可选文字: 4然0就完成了，可以下面代码进行验证 1Pangolin/bui1d/examp1es/He110Pang01in 2．/He110Pangolin](file:///C:/Users/BT7274/AppData/Local/Temp/msohtmlclip1/01/clip_image004.jpg)



**gtsam****你得在orb****和rtab****之前装和编译好**

# 3、更换GTSAM版本

（非常关键 镜像源好像没ab92779b25b04b..版本 。最好或者说必须在linux里成功gitclone下来才能切换到ab92779b25b04b..版本（没试过host复制过来行不行）这个还可以探索）

sudo apt remove ros-humble-gtsam

sudo cp -rf /usr/include/eigen3/Eigen /usr/include

sudo cp -rf /usr/include/eigen3 /usr/local/include

sudo cp -rf /usr/include/Eigen /usr/local/include

git clone https://github.com/borglab/gtsam.git

cd gtsam

git checkout ab92779b25b04b376fbbd1846bbbd21904c50e7a

 cmakelists文件显示该版本为4.1.0

mkdir build

cd build

![计算机生成了可选文字: t河题也是因为踬事不配，忘了跟你说了，因为懌的配指明了不甲TBS所I瑟关跹它就可以，我 是直在CMa迮Lists．以0吧一行注跹了 include(cmake/HandleTBB.cmake)](file:///C:/Users/BT7274/AppData/Local/Temp/msohtmlclip1/01/clip_image006.jpg)

 

cmake GTSAM_WITH_TBB=OFF -DGTSAM_Bcmake -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DUILD_EXAMPLES_ ALWAYS=OFF DGTSAM_BUILD_UNSTABLE=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON ..

 

上面的好像语法错误，要用下面这个

cmake -DGTSAM_WITH_TBB=OFF -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_BUILD_UNSTABLE=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON ..

 

sudo make install -j4

 

 

 

# 4、安装配置ORB_SLAM3 v0.4-beta版本

cd ~

git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git ORB_SLAM3_v0.4

镜像源（https://gitee.com/zjyfzu/ORB_SLAM3.git）需要切换好像

git checkout v0.4-beta

cd ORB_SLAM3_v0.4_RGBL/

git checkout v0.4-beta

 

下载补丁

wget https://gist.githubusercontent.com/matlabbe/f5cb281304a1305b2824a6ce19792e13/raw/f9faa15c5d35084d123639578ac4ce2ca88bf006/orbslam3_v4_rtabmap_fix.patch

补丁下载不了的话，window直接打开链接，右键页面另存为后缀为.patch的文件，再复制给linux

 

git apply orbslam3_v4_rtabmap_fix.patch

chmod +x build.sh

将build.sh文件的make -j 全部改为 make

./build.sh

sudo gedit ~/.bashrc

export ORB_SLAM_ROOT_DIR=~/ORB_SLAM3_v0.4

source ~/.bashrc

 

# 5、安装rtabmap，不能安装在ROS工作空间下

cd ~

git clone https://github.com/introlab/rtabmap.git rtabmap

镜像源https://gitee.com/xhb2016888/rtabmap.git

cd rtabmap

git checkout 0.21.1-humble

cd build

cmake -DWITH_G2O=OFF -DWITH_ORB_SLAM=ON ..

![计算机生成了可选文字: ywx@ywx：-Itestl/src/rtabmap 0 With ZED With ZEDOC With RealSense WithRealSense22，54·1 WithmyntEyeS With DepthAI Odomet「yApp「oaches： With loamvelodyne With floam With Itbfovis With Iibvts02 With dVOCore With okvis Withmsckfvio WithVINS．Fusion With OpenVINS With 0R8SLAM3 NO(ZEDsdkand/orCudanotfound) NO(ZEDOpenCapturenotfound） NO(Iibreatsensenotfound） YES〈License：Apache．2） NO NO NO NO NO NO NO NO NO NO NO (mynteyeSSdknotfound) (WITH DEPTHAI=OFF) (WITH LOA卜I—OFF） (WITH FLOAMOFF) (WITH FOVIS=OFF) (WITH VIS02=OFF) (WITH DVO=OFF） (WITH OKVIS=OFF) (WITHMSCKF_VIO=OFF) (HITH VINS—OFF） (HITH OPENVINS=OFF) Showalloptionswith． Configu「ingdone Generatingdone Buildfileshavebeen ywx@ywx written YES（License：GPLv3） -LA]g「epWITH to：/home/ywx/testl/src/rtabmap](file:///C:/Users/BT7274/AppData/Local/Temp/msohtmlclip1/01/clip_image008.jpg)

make -j4（不需要install）

![计算机生成了可选文字: 94％] [94％] [94％] [94％] 94％] [94％] [94％] [94％] 94％] [95％] [95％] 95％] [96％] [96％] [96％] 96％] [97％] [97％] [97％] 98％] [98％] [98％] [9] [98％] [98％] [98到 98％] [99％] [99％] [99到 99％] [16謂] [1％] YWX@YWX：-/testl/src/rtabmap/build Builttargetrgbdcamera BuildingCXX0习e匚ttools/Ca1tbratton/CMakeFttes/catibratton.dtr,/matn.cpp.O LinkingCXXexecutable一/．．/bin/rtabmap-epipotar_geometry LinkingCXXexecutable。．/．．/btn/rtabmap-odometryViewer Builttargeteptpolar_geometry Building（××objecttoots/matcher/CMakeFites/matcher Builtta「getodomet「yviewe「 BuildingCXX0e匚texamples/BONMapping/CMakeFtIes/bowmapping.di「/main.cpp.o LinkingCXXexecutable．．/．./btn/rtabmap-dataRecorder LinkingCXXexecutable一/．．'bin/rtabmap-calibration Builtta「getdataReco「de「 Genc「moc_mapBLJiIde「.cpp BuildingCXXobjectexamples/RCBDMapptng/CMakeFtles/rgbd_mapptng、「/matn.cpp.o Builttargetcalibration Generatingmoc_l'iap8utLderÅi_fi.cpp Gene「moc_mapBuiIdc「·Cpp Building（××objectexamples/WtftMapptng/CMakeFttes/wtft_mapptng、dir/matn.cpp.o LinkingCXXexecutable ．/··/bin/rtabmap-bow—mapping Builtta「getbo城napping Gene「otingmoc_MopBuLIde「.cpp Building（××objectexamples/NoEventsExampte/C卜lakeFttes/noEventsExamp1e.dtr/matn.cpp.o LinkingCXXexecutable一/．./bin/rtabmap-matcher Builtta「getmatche「 Building〔XXobjectexamples/NoEventsExample/CMakeFiIes/noEventsExampIe.dir/moc_MapButIde「，匚pp0 Building（××object。Cpp·0 BuildingCXXobjectexamples/RGBDMapptng/CMakeFtles/「gbd_mapping.di.「/moc_MapButlde「·cpp@0 BuildingCXXobjectexamples/WifiMapping/CMakeFiles/wtfi_mapping.di「/moc_MapBuiIde「Nifi.cpp.o LinkingCXXexecutable ．／．。[bin/rtabmap BuilttargetnoEventsExampIe LinkingCXXexecutable Builttargetrgbdmapping LinkingCXXexecutable Builtta「getwifi_mapping -noEventsExampLe ，寻·，/btn/rtabmap-rgbd_mapping ·．/·。-wift_mapptng ywx@ywx．。/et1/「匚「tabma卩'](file:///C:/Users/BT7274/AppData/Local/Temp/msohtmlclip1/01/clip_image010.jpg)

OptimizerG2O.cpp中77行三个头文件改成orbslam3中的绝对路径（如果报错需要改的话）

 

# 6、安装rtabmap_ros

cd ~/test1/src

git clone https://github.com/introlab/rtabmap_ros.git rtabmap_ros

镜像源https://gitee.com/congcongyanyi/rtabmap_ros.git

cd rtabmap_ros/

git checkout 0.21.1-humble

回到test1/目录下

rosdep update && rosdep install --from-paths src --ignore-src -r -y

这一步出错 连接不上的话（看运气）

[rosdep安装与使用_rosdep install --from-paths src --ignore-src --ros-CSDN博客](https://blog.csdn.net/weixin_45378779/article/details/103617471)

sudo apt remove ros-humble-rtabmap

export MAKEFLAGS="-j2" # Can be ignored if you have a lot of RAM (>16GB)

colcon build --symlink-install --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release

**爆红色错** **检查不到路径** **就重新启动个终端（先加这句）**

export RTABMap_DIR=~/rtabmap/build **在~/.bashrc****添加rtabmap****的路径**

![img](file:///C:/Users/BT7274/AppData/Local/Temp/msohtmlclip1/01/clip_image012.jpg)

可能要自己安装下colcon

sudo apt install python3-colcon-common-extensions

检查是否安装完成

Colcon –help

 

ros2 run rtabmap_slam rtabmap --verison检查是否成功接入orbslam3

上面那个输出不了跟下面那个一样用下面那个指令

ros2 run rtabmap_odom rgbd_odometry --version

 

![计算机生成了可选文字: YWX@YWX:～ 一5Source． ywx@ywx： 5「0s2「un「tabmap_slam「tabmap ywx@ywx· ;[INFO][1695788352．699119179] 0·00e000 [INFO][1695788352．70e065053] 30．000000 [INFO][1695788352．7ee112131] .[INFO][1695788352．70e159528] false [INFO][1695788352．706192552] 。[INFO][1695788352．7ee219282] t「t_Je .[INFO][1695788352．70e244924] false [INFO][1695788352．73e271157] minneighbors 2 ,[INFO][1695788352．7ee348273] 16 rtabmap(maps)： rtabmap(maps)： rtabmap(maps)： rtabmap(maps)： rtabmap(maps)： rtabmap(maps): rtabmap(maps)： 「tabmap(maps)： rtabmap(maps)： Q map_fttter「adius napfilterangle map_cteanup map_always_update mapempty_rayt「acing cloudoutput_voxettzed cloudsubt「actfilte「ing CLOUdsubtractfiltering octomap_tree_depth [INFO] [INFO] [INFO] [INFO] [INFO] [INFO] [INFO] [INFO] [INFO] [INFO] [1695788352·72213e584] [1695788352．722196719] [1695788352“722206214] [1695788352。722234S29] [1695788352．722259933} [1695788352．722291513] [1695788352．722322173] [1695788352·722331857] [1695788352“722343953] [16957g8352722355743] [rtabmap]： [rtabmap]， [rtabmap]： [「tabmap]： [「tabmap]· [rtabmap]: [rtabmap]： [rtabmap]： [rtabmap]： [rtabmap]： [rtabmap]： [rtabmap]， [rtabmap]： [rtabmap]： [rtabmap]： [「tab卩旧习： [rtabmap]： [「tabmap]， [rtabmap]， rtabmap• rtabmap. 「tabmap. rtabmap. rtabmap. 「tabmap. 「tabmap• rtabmap. 「tabmap. rtabmap: id mapframeid baselink map logto「osoutlevel tntttalpose useactionfor_goat false tfdelay ．050300 tftole「ance 0·100000 0d05en50「一Syn（ false false gen_scan gendepth false RTAB-Map: PCL： WithVTK： OpenCV： With With With With With With With With OpenCVxfeatures2d： OpenCVnonfree： ORBOcTree： SuperpointTo「ch： Python3: FastCV： OpenGV: Madgwtck： 1．12。1 false false false false false false t「](file:///C:/Users/BT7274/AppData/Local/Temp/msohtmlclip1/01/clip_image014.jpg)

 

![计算机生成了可选文字: Q With Python3: With FastCV： With OpenGV： With Madgwtck： With PDAL： With TORO： With g20： With GTSAM： With Vertigo: With CVSBA： With Ceres： With OpenN12： With With F「eenect2· With K4W2： With K4A： With DC1394： With FlyCapture2： With ZED： With ZEDOpenCaptu「e: With RealSense: With RealSenseSLAM： With RealSense2： With MYNTEYES： With DepthAI： With tibpotntmatcher： With CCCoreLtb： With octonap： With Cpu-tsdf: With openchisel。 WithAliceVision： With LOA卜1： With FLOAM： With FOVIS： With vt502： With 0： With ORBSLAM3： With OKVIS： With MSCKFVIO： With VINS-Fuston: With OpenVINS： Y'以@尹以5 YWX@YWX:～ false false false false false false false trtJe false false false t「刂e false false false false false true false false false false false false false false false false false false false false false](file:///C:/Users/BT7274/AppData/Local/Temp/msohtmlclip1/01/clip_image016.jpg)

 

 

  

 

# 7、数据集测试

 

**一、euroc_datasets.launch.py** **测试**

[kmavvisualinertialdatasets – ASL Datasets (ethz.ch)](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)

下载

![img](file:///C:/Users/BT7274/AppData/Local/Temp/msohtmlclip1/01/clip_image018.jpg)

 

**前置安装** **不知道哪些是必备的** **但是都装了**

sudo apt install python3-rosbag

 

sudo apt-get install ros-humble-image-proc

sudo apt-get install ros-humble-imu-complementary-filter

 

sudo apt install python3-pip

sudo apt update

 

sudo apt install ros-humble-rosbag2* 

 

**第一步**

euroc_datasets.launch.py 在

/home/tlc/test1/install/rtabmap_examples/share/rtabmap_examples/launch

底下 所以用下面的命令

 

执行

ros2 launch rtabmap_examples euroc_datasets.launch.py args:="Odom/Strategy 5 OdomORBSLAM/VocPath /home/rosadmin/test1/src/ORB_SLAM3/Vocabulary/ORBvoc.txt" MH_seq:=true raw_images_for_odom:=true

 

 

**第二步**

sudo pip install rosbags   

rosbags-convert V1_01_easy.bag

cd V1_01_easy

执行

ros2 bag play V1_01_easy.db3 --clock

 

 

 

参考资料：

\# Example to run euroc datasets:

\#  $ sudo pip install rosbags   # See https://docs.openvins.com/dev-ros1-to-ros2.html

\#  $ rosbags-convert V1_01_easy.bag

\#  $ rosbags-convert MH_01_easy.bag

\#

\#  $ ros2 launch rtabmap_examples euroc_datasets.launch.py gt:=true

\#  $ cd V1_01_easy

\#  $ ros2 bag play V1_01_easy.db3 --clock

\#

\#  $ ros2 launch rtabmap_examples euroc_datasets.launch.py gt:=false

\#  $ cd MH_01_easy

\#  $ ros2 bag play MH_01_easy.db3 --clock

 

sudo apt install python3-rosbag

 

sudo apt-get install ros-humble-image-proc

sudo apt-get install ros-humble-imu-complementary-filter

 

sudo apt install python3-pip

sudo apt update

 

sudo apt install ros-humble-rosbag2* 

 

 

二、rgbdslam_datasets.launch.py 测试（没实现）

 

\# Example to run rgbd datasets:

\# [ROS1] Prepare ROS1 rosbag for conversion to ROS2

\#wget http://vision.in.tum.de/rgbd/dataset/freiburg3/rgbd_dataset_freiburg3_long_office_household.bag

\#  $ rosbag decompress rgbd_dataset_freiburg3_long_office_household.bag

\#wget https://raw.githubusercontent.com/srv/srv_tools/kinetic/bag_tools/scripts/change_frame_id.py

\#  Edit change_frame_id.py, remove/comment lines beginning with "PKG" and "import roslib", change line "Exception, e" to "Exception"

\#  $ roscore

\#python3 change_frame_id.py -o rgbd_dataset_freiburg3_long_office_household_frameid_fixed.bag -i rgbd_dataset_freiburg3_long_office_household.bag -f openni_rgb_optical_frame -t /camera/rgb/image_color

 

\# [ROS2]

\# sudo pip install rosbags   # See https://docs.openvins.com/dev-ros1-to-ros2.html

\#rosbags-convert rgbd_dataset_freiburg3_long_office_household_frameid_fixed.bag

 

\# ros2 launch rtabmap_examples rgbdslam_datasets.launch.py

\#cd rgbd_dataset_freiburg3_long_office_household_frameid_fixed

\#ros2 bag play rgbd_dataset_freiburg3_long_office_household_frameid_fixed.db3 –clock

 

 

# 8、构建无人机

参考

[NovoG93/sjtu_drone：ROS/ROS 2  gazebo四轴飞行器模拟器。 (github.com)](https://github.com/NovoG93/sjtu_drone)

**要求**

此软件包已使用 ROS 2 Humble 版本 （Ubuntu 22.04） 和 Gazebo 11 进行了测试。

 

**下载和构建**

cd ~/git && git clone git@github.com:NovoG93/sjtu_drone.git -b ros2

**(****下不下来就win****下下来再复制过去)**

cd ~/test1/src && ln -s ~/git/sjtu_drone

cd .. && rosdep install -r -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO && colcon build --packages-select-regex sjtu*

 

 

**启动**

**ROS 2 Source Installation**

1. Start gazebo, spawn drone, open teleop in xterm window, and     open rviz:
        ros2 launch sjtu_drone_bringup sjtu_drone_bringup.launch.py
2. Takeoff:
        ros2 topic pub /drone/takeoff     std_msgs/msg/Empty {} --once
3. Move drone: (use teleop window)
4. Land:
        ros2 topic pub /drone/land std_msgs/msg/Empty     {} --once

 

 

使用gazebo仿真时遇到的问题

**仿真时严重卡顿问题:**

尝试下打开gazebo在Camera栏中先选择Orthographic，此时会发现拖拉栅格十分丝滑，仿真也变得丝滑了很多，再根据需要考虑是否需要再切换回Perspective。