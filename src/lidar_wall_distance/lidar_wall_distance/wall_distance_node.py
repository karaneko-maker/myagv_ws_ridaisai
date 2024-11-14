import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
import numpy as np

class ScanAndDistanceNode(Node):
    def __init__(self):
        super().__init__('scan_and_distance_node')

        # LIDARのデータを購読
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',  
            self.lidar_callback,
            10)
        self.subscription  # prevent unused variable warning

        # クロップしたデータをパブリッシュする
        self.publisher_ = self.create_publisher(LaserScan, 'scan_cropped', 1)
        # 壁との距離をパブリッシュする
        self.publisher = self.create_publisher(Float32MultiArray, 'distance_topic', 1)
        self.distance = Float32MultiArray()

        # 左右の角度範囲を設定
        self.left_min_angle = 60.0
        self.left_max_angle = 120.0
        self.right_min_angle = -120.0
        self.right_max_angle = -60.0

        

    def lidar_callback(self, msg):
        # LIDARデータの処理
        ranges = np.array(msg.ranges)

        # 左右の範囲を定義 (指定された角度範囲)
        left_indices = self.get_indices_for_angle_range(msg.angle_min, msg.angle_increment, self.left_min_angle, self.left_max_angle, ranges)
        right_indices = self.get_indices_for_angle_range(msg.angle_min, msg.angle_increment, self.right_min_angle, self.right_max_angle, ranges)

        # 左側の距離計算（補正）
        left_distances = ranges[left_indices]
        left_distances = left_distances[~np.isnan(left_distances)]  # NaNを除去

        # infを無視して有効な距離のみを使用
        left_distances = left_distances[np.isfinite(left_distances)]  # infを除去

        angles_left = np.linspace(self.left_min_angle, self.left_max_angle, len(left_distances))  # 角度を計算
        left_vertical_distances = [
            dist * np.cos(np.radians(angle - 90)) for dist, angle in zip(left_distances, angles_left)
        ]

        # 有効な値のみで平均を計算
        if left_vertical_distances:
            left_distance = np.mean(left_vertical_distances)  # NaNやinfを無視して平均を計算
        else:
            left_distance = float('nan')  # 有効なデータがない場合はNaNに設定

        # 右側の距離計算（補正）
        right_distances = ranges[right_indices]
        right_distances = right_distances[~np.isnan(right_distances)]  # NaNを除去

        # infを無視して有効な距離のみを使用
        right_distances = right_distances[np.isfinite(right_distances)]  # infを除去

        angles_right = np.linspace(self.right_min_angle, self.right_max_angle, len(right_distances))  # 角度を計算
        right_vertical_distances = [
            dist * np.cos(np.radians(angle + 90)) for dist, angle in zip(right_distances, angles_right)
        ]

        # 有効な値のみで平均を計算
        if right_vertical_distances:
            right_distance = np.mean(right_vertical_distances)  # NaNやinfを無視して平均を計算
        else:
            right_distance = float('nan')  # 有効なデータがない場合はNaNに設定
        # 距離をログ出力
        self.get_logger().info(f'Left Wall Distance (corrected): {left_distance:.2f} m')
        self.get_logger().info(f'Right Wall Distance (corrected): {right_distance:.2f} m')

        # 距離をパブリッシュ
        array = [left_distance, right_distance]
        self.distance = Float32MultiArray(data=array)
        self.publisher.publish(self.distance)

        # LaserScanの各角度を計算
        angle_increment = msg.angle_increment
        angle_min = msg.angle_min

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

    def get_indices_for_angle_range(self, angle_min, angle_increment, min_angle, max_angle, ranges):
        """指定された角度範囲内のインデックスを取得するヘルパー関数"""
        min_index = int((np.radians(min_angle) - angle_min) / angle_increment)
        max_index = int((np.radians(max_angle) - angle_min) / angle_increment)
        return list(range(max(min_index, 0), min(max_index + 1, len(ranges))))

def main(args=None):
    rclpy.init(args=args)
    scan_and_distance_node = ScanAndDistanceNode()
    rclpy.spin(scan_and_distance_node)
    scan_and_distance_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
