import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import cv2
import matplotlib.pyplot as plt
from sklearn.linear_model import RANSACRegressor
import matplotlib.patches as patches
from geometry_msgs.msg import Twist

class ScanAndDistanceNode(Node):
    def __init__(self):
        super().__init__('lidar_wall_find_2')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',  
            self.lidar_callback,
            1)
            # パブリッシャーを作成（ロボットの速度を制御するため）
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 1)

    def lidar_callback(self, msg):
        ranges = msg.ranges
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        points = []
        for i, r in enumerate(ranges):
            if r > msg.range_min and r < msg.range_max:
                angle = angle_min + i * angle_increment
                x = r * np.cos(angle)
                y = r * np.sin(angle)
                points.append((x, y))

        points = np.array(points)
        # self.get_logger().info(f'Point cloud: {points.shape}')

        left_points = points[(points[:, 0] < 0)]  # 左側
        right_points = points[(points[:, 0] > 0)]  # 右側

        # 最初のRANSACを適用
        left_slope, left_intercept, left_inliers = self.detect_wall_with_ransac(left_points)
        right_slope, right_intercept, right_inliers = self.detect_wall_with_ransac(right_points)

        # 左右のアウトライアを抽出
        left_outliers = left_points[~left_inliers]
        right_outliers = right_points[~right_inliers]

        # 左右のアウトライアを統合
        positive_slope, positive_intercept = None, None
        combined_outliers = np.vstack((left_outliers, right_outliers))
        positive_outliers = combined_outliers[combined_outliers[:, 1] > 0]

        if len(positive_outliers) > 0:
            # 統合したアウトライアにRANSACを再適用
            new_positive_slope, new_positive_intercept, _ = self.detect_wall_with_ransac(positive_outliers)
            # 新しい値が計算できた場合のみ、値を代入
            if new_positive_slope is not None:
                positive_slope = new_positive_slope
            if new_positive_intercept is not None:
                positive_intercept = new_positive_intercept
        
            # 新たに直線をプロット
            self.plot_points_and_line(positive_outliers, positive_slope, positive_intercept, color_points='green', color_line='black')
        
        # 点と直線との距離を計算
        point_left = (-0.14, 0.31)  # 左側の直線に対する点
        point_right = (0.27, 0.31)  # 右側の直線に対する点

        distance_left = self.point_to_line_distance(left_slope, left_intercept, point_left)
        distance_right = self.point_to_line_distance(right_slope, right_intercept, point_right)
        distance_positive = self.point_to_line_distance(positive_slope, positive_intercept, point_right)
        self.get_logger().info(f'distance_left: {distance_left}, distance_right: {distance_right}, distance_positive: {distance_positive}')

        # 距離に基づいて旋回命令を発行
        if positive_slope is not None and positive_intercept is not None and self.are_lines_perpendicular(right_slope, positive_slope) and distance_positive < 0.48:
            self.turn_left_larger()
        elif distance_right < 0.1:
            self.turn_left()
        elif distance_left < 0.05:
            self.turn_right()
        else:
            self.go_straight()
        
        self.get_logger().info(f'perpendicular : {self.are_lines_perpendicular(right_slope, positive_slope)}')


        # プロット
        self.plot_points_and_line(left_points, left_slope, left_intercept, color_points='blue', color_line='purple')
        self.plot_points_and_line(right_points, right_slope, right_intercept, color_points='orange', color_line='red')
        # 四角形を描画
        rect = patches.Rectangle(
            (-0.14, -0.18),    # 四角形の左下の座標 (x, y)
            0.41,         # 幅
            0.49,         # 高さ
            linewidth=2,
            edgecolor='black',
            facecolor='none'  # 塗りつぶしなし
        )
        plt.gca().add_patch(rect)  # 現在のグラフに四角形を追加

        plt.title('Detected Line and Points')
        plt.axis('equal')
        plt.ylim(-2, 2)
        # plt.show()

    def detect_wall_with_ransac(self, points):
        points = np.array(points, dtype=np.float32)

        ransac = RANSACRegressor(residual_threshold=0.03, random_state=1000)
        X = points[:, 0].reshape(-1, 1)
        y = points[:, 1]

        ransac.fit(X, y)
        
        inlier_mask = ransac.inlier_mask_  # インライアのマスク
        slope = ransac.estimator_.coef_[0]
        intercept = ransac.estimator_.intercept_
        
        return slope, intercept, inlier_mask

    def point_to_line_distance(self, slope, intercept, point):
        """
        点と直線の距離を計算する関数
        点: (x0, y0)
        直線: y = mx + b (m = slope, b = intercept)
        """
        x0, y0 = point
        # 直線の方程式 Ax + By + C = 0 に変換
        A = slope
        B = -1
        C = intercept

        # 距離の計算
        distance = abs(A * x0 + B * y0 + C) / np.sqrt(A**2 + B**2)
        return distance
    
    # 直線の垂直条件をチェックする関数
    def are_lines_perpendicular(self, slope1, slope2):
        print(f"abs(slope1 * slope2 + 1) = {abs(slope1 * slope2 + 1)}")
        return abs(slope1 * slope2 + 1) < 2.0  # 1e-2 は許容誤差

    def turn_right(self):
        # 右旋回を行うメソッド
        twist = Twist()
        twist.linear.x = 0.33
        twist.angular.z = -0.2  # 右回転の速度を設定
        self.publisher.publish(twist)
        self.get_logger().info("Turning Right")

    def turn_left(self):
        # 左旋回を行うメソッド
        twist = Twist()
        twist.linear.x = 0.33
        twist.angular.z = 0.2  # 左回転の速度を設定
        self.publisher.publish(twist)
        self.get_logger().info("Turning Left")
    
    def turn_left_larger(self):
        # 左旋回を行うメソッド
        twist = Twist()
        twist.linear.x = 0.23
        twist.angular.z = 0.8  # 左回転の速度を設定
        self.publisher.publish(twist)
        self.get_logger().info("Turning Left Larger")

    def go_straight(self):
        # 直進を行うメソッド
        twist = Twist()
        twist.linear.x = 0.3
        twist.angular.z = 0.0
        self.publisher.publish(twist)
        self.get_logger().info("Go Straight")

    def plot_points_and_line(self, points, slope, intercept, color_points='blue', color_line='red'):
        plt.scatter(points[:, 0], points[:, 1], color=color_points, label='Points')

        x_vals = np.linspace(points[:, 0].min(), points[:, 0].max(), 100)
        y_vals = slope * x_vals + intercept

        plt.plot(x_vals, y_vals, color=color_line, label='Detected Line (RANSAC)')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.legend()

def main(args=None):
    rclpy.init(args=args)
    node = ScanAndDistanceNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
