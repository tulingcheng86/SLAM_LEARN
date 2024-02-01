import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        self.right_subscription = self.create_subscription(
            Image,
            '/right/image_raw',
            self.right_image_callback,
            10)
        self.right_publisher = self.create_publisher(
            Image,
            '/right/image_raw_fix',
            10)
        
        self.left_subscription = self.create_subscription(
            Image,
            '/left/image_raw',
            self.left_image_callback,
            10)
        self.left_publisher = self.create_publisher(
            Image,
            '/left/image_raw_fix',
            10)
        
    def right_image_callback(self, msg):
        expected_size = msg.height * msg.width
        msg.step = msg.width
        current_size = len(msg.data)
        # 检查是否需要填充
        if current_size < expected_size:
            padding_size = expected_size - current_size
            msg.data = bytearray(msg.data)
            msg.data.extend(bytearray(padding_size))
        # 发布修复后的图像
        self.right_publisher.publish(msg)
        self.get_logger().info('Published fixed image to /right/image_raw_fix')
    
    def left_image_callback(self, msg):
        expected_size = msg.height * msg.width
        msg.step = msg.width
        current_size = len(msg.data)
        # 检查是否需要填充
        if current_size < expected_size:
            padding_size = expected_size - current_size
            msg.data = bytearray(msg.data)
            msg.data.extend(bytearray(padding_size))
        # 发布修复后的图像
        self.left_publisher.publish(msg)
        self.get_logger().info('Published fixed image to /left/image_raw_fix')

def main(args=None):
    rclpy.init(args=args)
    image_processor = ImageProcessor()
    rclpy.spin(image_processor)
    image_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()