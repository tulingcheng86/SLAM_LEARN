#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import message_filters
from sensor_msgs.msg import Image

class StereoImageSync(Node):
    def __init__(self):
        super().__init__('stereo_image_sync')

        # 创建左右图像的订阅者
        left_image_sub = message_filters.Subscriber(self, Image, '/left/image_raw_fix')
        right_image_sub = message_filters.Subscriber(self, Image, '/right/image_raw_fix')

        # 创建一个时间同步器，当左右图像的时间戳接近时调用回调函数
        ts = message_filters.ApproximateTimeSynchronizer([left_image_sub, right_image_sub], 10, 0.05, allow_headerless=True)
        ts.registerCallback(self.sync_callback)

        # 创建发布者
        self.left_image_pub = self.create_publisher(Image, '/stereo/left/image_raw_sync', 17)
        self.right_image_pub = self.create_publisher(Image, '/stereo/right/image_raw_sync', 17)

    def sync_callback(self, left_image, right_image):
        # 当左右图像时间同步时，发布图像
        self.left_image_pub.publish(left_image)
        self.right_image_pub.publish(right_image)

def main(args=None):
    rclpy.init(args=args)
    stereo_image_sync = StereoImageSync()
    rclpy.spin(stereo_image_sync)
    stereo_image_sync.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
