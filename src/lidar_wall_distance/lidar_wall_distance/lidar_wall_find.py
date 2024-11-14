import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class ScanAndDistanceNode(Node):
    def __init__(self):
        super().__init__('scan_and_distance_node')

        # LIDARのデータを購読
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',  
            self.lidar_callback,
            1)

    def lidar_callback(self, msg):
        # LaserScanメッセージからデータを取得
        ranges = msg.ranges
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        # 2次元点群を格納するリスト
        points = []

        # 各点を(x, y)座標に変換
        for i, r in enumerate(ranges):
            if r > msg.range_min and r < msg.range_max:  # 有効範囲内の距離データのみ使用
                angle = angle_min + i * angle_increment
                x = r * np.cos(angle)
                y = r * np.sin(angle)
                points.append((x, y))
        
        # numpy配列に変換（処理や可視化を行いやすくするため）
        points = np.array(points)
        
        # デバッグ用の出力
        self.get_logger().info(f'Point cloud: {points}')

        # RANSACを使って壁を検出する関数へ渡す
        self.detect_wall_with_ransac(points)

    def detect_wall_with_ransac(self, points):
        # RANSAC処理を行う（先ほどの例と同様）
        pass  # ここにRANSAC処理を実装する

def main(args=None):
    rclpy.init(args=args)
    node = ScanAndDistanceNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
