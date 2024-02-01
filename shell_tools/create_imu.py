import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped, QuaternionStamped

class ImuFusionNode(Node):
    def __init__(self):
        super().__init__('imu_fusion_node')

        # 订阅话题
        self.accel_subscription = self.create_subscription(Vector3Stamped, '/sbpilot/acceleration_raw', self.accel_callback, 10)
        self.gyro_subscription = self.create_subscription(Vector3Stamped, '/sbpilot/angular_rate_raw', self.gyro_callback, 10)
        self.quat_subscription = self.create_subscription(QuaternionStamped, '/sbpilot/quaternion', self.quat_callback, 10)

        # 发布IMU数据
        self.imu_pub = self.create_publisher(Imu, '/sbpilot/imu', 10)
        
        # 待融合数据
        self.last_accel = None
        self.last_gyro = None
        self.last_quat = None

        # 使用定时器控制发布频率，这里设置为30Hz
        timer_period = 1.0 / 30  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def accel_callback(self, msg):
        self.last_accel = msg

    def gyro_callback(self, msg):
        self.last_gyro = msg

    def quat_callback(self, msg):
        self.last_quat = msg

    def timer_callback(self):
        # 检查是否收到所有类型的消息
        if self.last_accel is not None and self.last_gyro is not None and self.last_quat is not None:
            # 确保数据是同步的，这里简化为检查时间戳是否足够接近
            if abs(self.last_accel.header.stamp.sec - self.last_quat.header.stamp.sec) < 1 and \
               abs(self.last_gyro.header.stamp.sec - self.last_quat.header.stamp.sec) < 1:
                self.publish_imu()

    def publish_imu(self):
        imu_msg = Imu()
        # 创建IMU消息并填充数据
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'world'  # 根据需要更改frame_id

        # 填充加速度数据
        imu_msg.linear_acceleration.x = self.last_accel.vector.x
        imu_msg.linear_acceleration.y = self.last_accel.vector.y
        imu_msg.linear_acceleration.z = self.last_accel.vector.z

        # 填充角速度数据
        imu_msg.angular_velocity.x = self.last_gyro.vector.x
        imu_msg.angular_velocity.y = self.last_gyro.vector.y
        imu_msg.angular_velocity.z = self.last_gyro.vector.z

        # 填充姿态（四元数）
        imu_msg.orientation = self.last_quat.quaternion

        # 发布IMU消息
        self.imu_pub.publish(imu_msg)
        self.get_logger().info(f"Published IMU data at {imu_msg.header.stamp.sec}.{imu_msg.header.stamp.nanosec} (sec.nanosec)")

        # 发布后重置记录的数据
        self.last_accel = None
        self.last_gyro = None
        self.last_quat = None

def main(args=None):
    rclpy.init(args=args)
    imu_fusion_node = ImuFusionNode()
    rclpy.spin(imu_fusion_node)
    imu_fusion_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
