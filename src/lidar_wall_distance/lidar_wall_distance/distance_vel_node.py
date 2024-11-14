import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import numpy as np

class DistanceAndVelNode(Node):
    def __init__(self):
        super().__init__('distance_vel_node')
        self.subscription = self.create_subscription(Float32MultiArray, '/distance_topic', self.listener_callback, 1)
        self.control_publisher = self.create_publisher(Twist, 'cmd_vel', 1)

        # 目標壁距離（道の真ん中を目指す）
        self.target_distance = 1.0  # 目標となる壁までの距離（例: 1.0メートル）

        # PD制御のためのパラメータ
        self.kp = 1.0  # 比例ゲイン（旋回制御の強さ）
        self.kd = 0.1  # 微分ゲイン（速度の変化を抑える）

        # 前回のエラーを保持
        self.previous_left_error = 0.0
        self.previous_right_error = 0.0

        # 前進速度の設定
        self.linear_x = 0.2  # 定速前進

    def listener_callback(self, msg):
        # 受け取ったメッセージのデータを取得
        distances = msg.data
        self.left_distance = distances[0]
        self.right_distance = distances[1]

        # PD制御による調整
        self.control_robot()

        # 受け取った左と右の距離データを表示
        self.get_logger().info(f'Left distance: {self.left_distance}, Right distance: {self.right_distance}')

    def control_robot(self):
        # 目標距離との差分（目標距離は1.0メートル）
        left_error = self.target_distance - self.left_distance
        right_error = self.target_distance - self.right_distance

        # PD制御の計算
        left_derivative = left_error - self.previous_left_error
        right_derivative = right_error - self.previous_right_error

        # 左右のエラーをPD制御で補正
        left_angular_z = self.kp * left_error + self.kd * left_derivative
        right_angular_z = self.kp * right_error + self.kd * right_derivative

        # 旋回方向を決定
        angular_z = 0.0
        if right_error > 0:  # 右側の壁が目標距離より遠い
            angular_z = -right_angular_z  # 右に回転（負の値）
        elif left_error > 0:  # 左側の壁が目標距離より遠い
            angular_z = left_angular_z  # 左に回転（正の値）

        # `Twist`メッセージを使ってロボットに制御信号を送る
        control_msg = Twist()
        control_msg.linear.x = self.linear_x  # 前進速度
        control_msg.angular.z = angular_z  # 旋回速度（壁との距離に基づいて調整）

        self.get_logger().info(f'linear x: {self.linear_x}, angular z: {angular_z}')

        # ロボットの制御メッセージをパブリッシュ
        self.control_publisher.publish(control_msg)

        # 前回のエラーを更新
        self.previous_left_error = left_error
        self.previous_right_error = right_error

def main(args=None):
    rclpy.init(args=args)
    node = DistanceAndVelNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
