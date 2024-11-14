import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import cv2
import matplotlib.pyplot as plt

class ScanAndDistanceNode(Node):
    def __init__(self):
        super().__init__('lidar_wall_find')

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

        # 分ける前の点群をプロット
        self.plot_points(points, "All Points Before Splitting")

        
        # 点群を左右に分割
        left_points = points[(points[:, 0] < 0) & (points[:, 1] > -2) & (points[:, 1] < 2)] # x < 0 , -2< y 2<の点群 (左側)
        self.get_logger().info(f'left_points shape: {left_points.shape}')
        right_points = points[(points[:, 0] > 0) & (points[:, 1] > -2) & (points[:, 1] < 2)] # x > 0 , -2< y 2<の点群 (右側)
        self.get_logger().info(f'right_points shape: {right_points.shape}')

        # 左右の壁を検出する
        left_lines = self.detect_multiple_lines(left_points)
        right_lines = self.detect_multiple_lines(right_points)

        # 前方の点群を追跡して壁を検出
        front_points = points[(points[:, 1] > 0) & (points[:, 0] > -0.2) & (points[:, 0] < 0.2)]
        if len(front_points) < 2:
            self.get_logger().warn("Not enough points to fit a line.")
        else:
            front_lines = self.detect_multiple_lines(front_points)
        self.get_logger().info(f'front_points shape: {front_points.shape}, points: {front_points}')

        # 結果をプロット
        self.plot_lines_and_points(left_points, left_lines, "Left Wall")
        self.plot_lines_and_points(right_points, right_lines, "Right Wall")
        self.plot_lines_and_points(front_points, front_lines, "Front Wall")
        # plt.title('Detected Lines and Points')
        # plt.show()
        # プロット後、待機するように修正
        plt.pause(0.1)  # 一時停止してプロットが表示されるようにする

    def detect_multiple_lines(self, points, max_iter=1000, threshold=0.05, min_inlier_ratio=0.1, max_lines=5):
        lines = []
        points = np.array(points, dtype=np.float32)
        if len(points) < 2:
            self.get_logger().warn("Not enough points to fit a line.")
            return lines  # 点が少なすぎる場合は空のリストを返す

        all_points = points.copy()  # 初期点群を保存しておく
        for _ in range(max_lines):
            # RANSACで直線をフィット
            [vx, vy, x0, y0] = cv2.fitLine(all_points, cv2.DIST_L2, 0, threshold, threshold)
            
            # 直線の傾きと切片を計算
            slope = vy / vx
            intercept = y0 - slope * x0
            
            # この直線に近い点を探す
            inliers = []
            for point in all_points:
                x, y = point
                # 点が直線に近いかどうかを判断する
                if abs(y - (slope * x + intercept)) < threshold:
                    inliers.append(point)
            
            print(f"Inliers found: {len(inliers)} out of {len(all_points)}")

            # Inliersが一定割合（min_inlier_ratio）以上あれば、その直線を採用
            if len(inliers) / len(all_points) > min_inlier_ratio:
                lines.append((slope, intercept))
                inliers = np.array(inliers)
                all_points = np.array([point for point in all_points if point not in inliers])  # 残りの点群からinliersを除去
            else:
                print(f"no_line!!!!!!!!!!!!!!")
                break  # Inliersが少なすぎた場合は終了

        return lines
    
    def plot_points(self, points, title):
        """点群を描画するための補助関数"""
        plt.scatter(points[:, 0], points[:, 1], color='blue', label='Points')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.legend()
        plt.title(title)
        plt.show()

    # 結果をプロット
    def plot_lines_and_points(self, points, lines, wall_side):
        plt.scatter(points[:, 0], points[:, 1], color='blue', label=f'{wall_side} Points')
        
        # 各直線をプロット
        for slope, intercept in lines:
            slope = float(slope)  # slope を float 型に変換
            intercept = float(intercept)  # intercept を float 型に変換
            x_vals = np.linspace(points[:, 0].min(), points[:, 0].max(), 100)
            y_vals = slope * x_vals + intercept
            plt.plot(x_vals, y_vals, label=f'{wall_side} Line: y = {slope:.2f}x + {intercept:.2f}')
        
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.legend()
        plt.title(f'{wall_side} Detected Line')
        plt.show()
    

def main(args=None):
    rclpy.init(args=args)
    node = ScanAndDistanceNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
