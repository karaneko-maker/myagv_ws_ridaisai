import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import cv2
import matplotlib.pyplot as plt
from sklearn.linear_model import RANSACRegressor

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
        print(f"Points shape: {points.shape}")

        plt.scatter(points[:, 0], points[:, 1], color='blue', label='Points')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.legend()
        plt.title("raw point data")
        plt.axis('equal')
        plt.show()

        # デバッグ用の出力
        self.get_logger().info(f'Point cloud: {points}')
        
        left_points = points[(points[:, 0] < 0)] # x < 0の点群 (左側)
        right_points = points[(points[:, 0] > 0)] # x > 0の点群 (右側)

        # RANSACを使って壁を検出する関数へ渡す
        left_slope, left_intercept = self.detect_wall_with_ransac(left_points)
        right_slope, right_intercept = self.detect_wall_with_ransac(right_points)

        # プロットを呼び出し
        self.plot_points_and_line(left_points, left_slope, left_intercept, color_points='blue', color_line='purple')
        self.plot_points_and_line(right_points, right_slope, right_intercept, color_points='orange', color_line='red')
        plt.title('Detected Line and Points')
        plt.axis('equal')
        plt.ylim(-2, 2)
        plt.show()

    def detect_wall_with_ransac(self, points):
        # RANSAC処理
        points = np.array(points, dtype=np.float32)
        
        # RANSAC回帰器の作成
        ransac = RANSACRegressor(residual_threshold=0.05, random_state=1000)

        # X, Yから直線を予測
        X = points[:, 0].reshape(-1, 1)  # X (特徴量), 1列に変換
        y = points[:, 1]   # Y (ターゲット)

        # RANSAC回帰をフィッティング
        ransac.fit(X, y)

        # フィッティングした回帰モデルの係数と切片を表示
        slope = ransac.estimator_.coef_[0]  # 傾き
        intercept = ransac.estimator_.intercept_  # 切片
        
        return slope, intercept
    
    # グラフに表示
    def plot_points_and_line(self, points, slope, intercept, color_points='blue', color_line='red'):
        # 点のプロット
        plt.scatter(points[:, 0], points[:, 1], color=color_points, label='Points')

        # xの範囲を定義
        x_vals = np.linspace(points[:, 0].min(), points[:, 0].max(), 100)
        # 直線の式 y = slope * x + intercept を使ってy値を計算
        y_vals = slope * x_vals + intercept

        # 直線をプロット
        plt.plot(x_vals, y_vals, color=color_line, label='Detected Line (RANSAC)')
        
        # ラベルとタイトル
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.legend()
        # plt.title('Detected Line and Points')
        # plt.axis('equal')
        # plt.ylim(-2, 2)
        # plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = ScanAndDistanceNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
