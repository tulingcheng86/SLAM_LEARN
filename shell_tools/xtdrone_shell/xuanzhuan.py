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