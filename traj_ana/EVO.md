# evo

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



**RTABMAP估计的轨迹**

**rtabmap_MH01_ape**![rtabmap_MH01_ape](/home/sbim/SLAM_LEARN/traj_ana/ana_MH01_RTAB/rtabmap_MH01_ape.png)

![rtabmap_MH01](/home/sbim/SLAM_LEARN/traj_ana/ana_MH01_RTAB/rtabmap_MH01.png)

**VINS-RTABMAP估计的轨迹**

**vins-rtabmap_MH01_ape**![vins-rtabmap_MH01_ape](/home/sbim/SLAM_LEARN/traj_ana/ana_MH01_RTAB/vins-rtabmap_MH01_ape.png)

![vins-rtabmap_MH01_xyz](/home/sbim/SLAM_LEARN/traj_ana/ana_MH01_RTAB/vins-rtabmap_MH01_xyz.png)
