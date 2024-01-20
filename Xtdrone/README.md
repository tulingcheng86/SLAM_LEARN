

# 基于xtdrone的仿真无人机学习：定点飞行 （用ego_planner

  参考[三维运动规划 · 语雀](https://www.yuque.com/xtdrone/manual_cn/3d_motion_planning)

## 零·基础知识



MAVROS（MAVLink to ROS）是一个ROS（Robot Operating System）节点，用于在ROS系统中与PX4飞控系统进行通信。
 它通过将MAVLink消息转换为ROS消息，实现了ROS系统与无人机之间的数据交换和控制。

在PX4 SITL（Software-in-the-Loop）仿真环境中，无人机模型通过PX4飞控软件进行仿真。
 通过在仿真环境中运行PX4 SITL，可以在计算机上模拟无人机的飞行和姿态控制。


 在这个launch文件中，MAVROS节点和PX4 SITL仿真环境中的无人机扮演着以下角色：
 1、MAVROS节点：负责接收来自无人机的传感器数据（如GPS、IMU等）和状态信息，并将其转换为ROS消息。同时，它也负责接收来自ROS系统的控制指令，并将其转发给无人机。
 2、PX4 SITL仿真环境中的无人机：由PX4飞控软件仿真的无人机模型。它接收来自MAVROS节点的控制指令，并根据指令模拟无人机的飞行和姿态控制。同时，它还将传感器数据和状态信息发送给MAVROS节点，以便在ROS系统中进行处理和显示。


 "/xtdrone/iris_0/cmd" 这个话题是用来向 ROS 中连接的无人机发送控制指令的。在 ROS 中，所有的通信都是通过发布者（publisher）和订阅者（subscriber）之间的消息传递来完成的。

一个发布者节点会将无人机的控制指令发布到 "/xtdrone/iris_0/cmd" 这个话题上，并且无人机上运行的订阅者节点将监听该话题以接收并执行这些指令。这通常涉及到无人机的移动、悬停、转向等操作，具体取决于发布到该话题上的指令。

![img](https://img-blog.csdnimg.cn/direct/e7ea4908f614474ca0b4fd1210ed44b9.png)![点击并拖拽以移动](data:image/gif;base64,R0lGODlhAQABAPABAP///wAAACH5BAEKAAAALAAAAAABAAEAAAICRAEAOw==)编辑

**键盘控制**
 该节点通过接收键盘输入来控制飞行器的前进、后退、左移、右移、上升、下降以及旋转等动作
 按下"w"键会增加飞行器的前进速度，按下"x"键会减小飞行器的前进速度。其他按键也有类似的功能。


 查看该话题  无人机的姿态（速度和角度）
 rostopic echo /xtdrone/iris_cmd_vel_flu 

**ego_planner**
 ego_planner需要输入深度图+相机位姿或是点云，这里以深度图+相机位姿的组合为例进行仿真
 相机位姿由VINS-Fusion计算得到。

这是一些ROS话题，用于机器人的控制和状态估计。

> /xtdrone/iris_0/cmd
>  /xtdrone/iris_0/cmd_accel_enu
>  /xtdrone/iris_0/cmd_accel_flu
>  /xtdrone/iris_0/cmd_pose_enu
>  /xtdrone/iris_0/cmd_pose_flu
>  /xtdrone/iris_0/cmd_vel_enu
>  /xtdrone/iris_0/cmd_vel_flu
>  /xtdrone/iris_0/planning/bspline
>  /xtdrone/iris_0/planning/data_display
>  /xtdrone/iris_0/planning/pos_cmd
>  /xtdrone/iris_0/vins_estimator/odometry
>  /xtdrone/leader/cmd
>  /xtdrone/leader/cmd_vel_flu

**具体说明如下：**

> /xtdrone/iris_0/planning/bspline：该话题用于发布来自路径规划器的B样条曲线路径信息，用于控制机器人的运动。
>
> /xtdrone/iris_0/planning/data_display：该话题用于在RViz中显示机器人的路径规划器数据信息，以便进行调试和验证。
>
> /xtdrone/iris_0/planning/pos_cmd：该话题用于发布机器人的位置控制指令，通过订阅该话题，机器人可以按照指定的位置移动到目标点。
>
> /xtdrone/iris_0/vins_estimator/odometry：该话题用于发布机器人的运动状态信息，包括机器人的位置、速度和姿态等数据，通过订阅该话题，用户可以获取机器人的实时状态信息。



**ego_planner 可以读取几个全局目标点然后规划路径，目的是无人机每到达一个目标点后旋转九十度，然后执行完所有目标点后回家（家里坐标（0，0，0））**

## **一、定点飞行**

**思路：将ego_planner一条轨迹拆成多个轨迹，编写脚本分段执行任务**

![img](https://img-blog.csdnimg.cn/direct/0d45bba9318042d69f05b6514e019c44.png)![点击并拖拽以移动](data:image/gif;base64,R0lGODlhAQABAPABAP///wAAACH5BAEKAAAALAAAAAABAAEAAAICRAEAOw==)编辑拆分成三段



## 二、旋转

**用键盘控制无人机飞行
 在一个终端运行（启动gazebo，出现了场景和飞机）**

```
cd ~/PX4_Firmware
roslaunch px4 indoor1.launch
```

![点击并拖拽以移动](data:image/gif;base64,R0lGODlhAQABAPABAP///wAAACH5BAEKAAAALAAAAAABAAEAAAICRAEAOw==)


 
 Gazebo启动后，在另一个终端运行（启动通信脚本，iris代表子机型，0代表飞机的编号，与0号iris建立通信）

```
cd ~/XTDrone/communication/
python multirotor_communication.py iris 0
```

![点击并拖拽以移动](data:image/gif;base64,R0lGODlhAQABAPABAP///wAAACH5BAEKAAAALAAAAAABAAEAAAICRAEAOw==)


 
 与0号iris建立通信后，在另一个终端运行（启动键盘控制脚本，iris代表机型，1代表飞机的个数，vel代表速度控制）

```
cd ~/XTDrone/control/keyboard
python multirotor_keyboard_control.py iris 1 vel
```

![点击并拖拽以移动](data:image/gif;base64,R0lGODlhAQABAPABAP///wAAACH5BAEKAAAALAAAAAABAAEAAAICRAEAOw==)

## 通过查看键盘控制无人机的话题消息  可知

> root@d10fba288ba3:/# rostopic info /xtdrone/iris_0/cmd_vel_flu
>  Type: geometry_msgs/Twist
>
> Publishers: 
>  \* /iris_multirotor_keyboard_control (http://d10fba288ba3:35643/)
>
> Subscribers: 
>  \* /iris_0_communication (http://d10fba288ba3:38861/)


 **因此
 /xtdrone/iris_0/cmd_vel_flu 话题使用的消息类型是 geometry_msgs/Twist。**

> **在终端中执行以下命令，将指定的消息内容发布到该话题：**
>
> rostopic pub /xtdrone/iris_0/cmd_vel_flu geometry_msgs/Twist "linear:
>   x: 0.0
>   y: 0.0
>   z: 0.0
>  angular:
>   x: 0.0
>   y: 0.0
>   z: 0.33"

**但是该消息内容只会执行一次，所以要编写个py脚本循环发布旋转消息**
 

xuanzhuan.py

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist

def main():
    rospy.init_node('command_publisher')

    # 创建一个发布者，指定要发布的话题和消息类型
    pub = rospy.Publisher('/xtdrone/iris_0/cmd_vel_flu', Twist, queue_size=10)

    # 创建一个 Twist 对象，用于填充指令消息的内容
    cmd_vel = Twist()
    cmd_vel.angular.z = 1.57  # 设置角速度

    rate = rospy.Rate(220)  # 设置发布频率为 250Hz
    count = 0  # 计数器

    while not rospy.is_shutdown()and count<240:
        pub.publish(cmd_vel)  # 发布指令消息
        rate.sleep()
        count+=1

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
```

![点击并拖拽以移动](data:image/gif;base64,R0lGODlhAQABAPABAP///wAAACH5BAEKAAAALAAAAAABAAEAAAICRAEAOw==)



**三、执行任务**

**现在只需要将这些任务用脚本依次执行就行了**

**先依照\**[三维运动规划 · 语雀](https://www.yuque.com/xtdrone/manual_cn/3d_motion_planning)\****

**启动仿真**

然后执行脚本

```bash
echo "Start a Tmux session!"

# 创建一个名为 "ego_planner" 的 Tmux 会话
tmux new -d -s ego_planner

echo "Start ego_planner_1!"

# 向 Tmux 会话发送命令，并输入一个回车以启动 "roslaunch" 命令
tmux send-keys -t ego_planner "roslaunch ego_planner single_uav.launch" ENTER 

sleep 22

echo "Start Rotating!"
tmux send-keys -t ego_planner "C-c" ENTER
sleep 2
tmux send-keys -t ego_planner "python xuanzhuan.py" ENTER 
sleep 6

echo "Start ego_planner_2!"
# 向 Tmux 会话发送命令，并输入一个回车以启动 "roslaunch" 命令
tmux send-keys -t ego_planner "roslaunch ego_planner single_uav_2.launch" ENTER 
sleep 15

echo "Start Rotating!"
tmux send-keys -t ego_planner "C-c" ENTER
sleep 2
tmux send-keys -t ego_planner "python xuanzhuan.py" ENTER 
sleep 6

echo "Start GOhome!!"
# 向 Tmux 会话发送命令，并输入一个回车以启动 "roslaunch" 命令
tmux send-keys -t ego_planner "roslaunch ego_planner single_uav_3.launch" ENTER 
```

![点击并拖拽以移动](data:image/gif;base64,R0lGODlhAQABAPABAP///wAAACH5BAEKAAAALAAAAAABAAEAAAICRAEAOw==)



**完成任务**

![img](https://img-blog.csdnimg.cn/direct/a2a06d8074e946fe9407162314473fb7.png)![点击并拖拽以移动](data:image/gif;base64,R0lGODlhAQABAPABAP///wAAACH5BAEKAAAALAAAAAABAAEAAAICRAEAOw==)编辑