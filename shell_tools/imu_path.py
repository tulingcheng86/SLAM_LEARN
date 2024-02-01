#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class IMUPathNode(Node):
    def __init__(self):
        super().__init__('imu_path_node')
        
        # 订阅IMU数据
        self.imu_sub = self.create_subscription(Imu, '/imu0', self.imu_callback, 10)
        
        # 发布Path数据
        self.path_pub = self.create_publisher(Path, '/imu/path', 10)
        
        # 初始化Path消息
        self.path = Path()
        self.path.header.frame_id = "world"  # 或者你的参考坐标系
        
        # 假设初始位置和速度为0
        self.position = [0.0, 0.0, 0.0]
        self.velocity = [0.0, 0.0, 0.0]
        self.last_time = self.get_clock().now()

    def imu_callback(self, msg):
        # 计算时间间隔
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # 简化的加速度处理（实际应用中需要更复杂的处理）
        acc = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]

        # 更新速度和位置
        for i in range(3):
            self.velocity[i] += acc[i] * dt
            self.position[i] += self.velocity[i] * dt

        # 创建一个新的PoseStamped消息，将当前位置添加到路径中
        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self.position[0]
        pose.pose.position.y = self.position[1]
        pose.pose.position.z = self.position[2]
        self.path.poses.append(pose)

        # 更新Path消息的头部时间戳
        self.path.header.stamp = pose.header.stamp

        # 发布Path消息
        self.path_pub.publish(self.path)

def main(args=None):
    rclpy.init(args=args)
    node = IMUPathNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
