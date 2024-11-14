import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class ScanCropperNode(Node):
    def __init__(self):
        super().__init__('scan_cropper_node')

        self.subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.publisher_ = self.create_publisher(LaserScan, 'scan_cropped', 10)

        # 左右の角度範囲を設定
        self.left_min_angle = 60.0
        self.left_max_angle = 120.0
        self.right_min_angle = -120.0
        self.right_max_angle = -60.0
    
    def lidar_callback(self, msg):
        # LaserScanの各角度を計算
        angle_increment = msg.angle_increment
        angle_min = msg.angle_min
        # print(f"angle_min:{msg.angle_min}, angle_max;{msg.angle_max}, angle_increment;{msg.angle_increment}")

        # 左と右の範囲のインデックスを計算
        left_indices = self.get_indices_for_angle_range(angle_min, angle_increment, self.left_min_angle, self.left_max_angle, msg.ranges)
        right_indices = self.get_indices_for_angle_range(angle_min, angle_increment, self.right_min_angle, self.right_max_angle, msg.ranges)

        # 指定した角度範囲のデータを抽出
        cropped_ranges = [float('inf')] * len(msg.ranges)  # 他の角度は無限大に設定
        for i in left_indices + right_indices:
            cropped_ranges[i] = msg.ranges[i]

        # 新しいLaserScanメッセージの作成
        cropped_msg = LaserScan()
        cropped_msg.header = msg.header
        cropped_msg.angle_min = msg.angle_min
        cropped_msg.angle_max = msg.angle_max
        cropped_msg.angle_increment = msg.angle_increment
        cropped_msg.time_increment = msg.time_increment
        cropped_msg.scan_time = msg.scan_time
        cropped_msg.range_min = msg.range_min
        cropped_msg.range_max = msg.range_max
        cropped_msg.ranges = cropped_ranges

        # 新しいトピックにパブリッシュ
        self.publisher_.publish(cropped_msg)
        # self.get_logger().info('Published cropped scan data.')

    def get_indices_for_angle_range(self, angle_min, angle_increment, min_angle, max_angle, ranges):
        """指定された角度範囲内のインデックスを取得するヘルパー関数"""
        min_index = int((np.radians(min_angle) - angle_min) / angle_increment)
        max_index = int((np.radians(max_angle) - angle_min) / angle_increment)
        return list(range(max(min_index, 0), min(max_index + 1, len(ranges))))

def main(args=None):
    rclpy.init(args=args)
    scan_cropper_node = ScanCropperNode()
    rclpy.spin(scan_cropper_node)
    scan_cropper_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
