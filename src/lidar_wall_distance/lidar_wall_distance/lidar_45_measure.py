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

        # 目標距離と最小安全距離
        self.target_distance = 1.0  # 壁からの中央目標距離
        self.min_distance = 0.1  # ぶつからないようにする最小距離（角）

        # PD制御のためのパラメータ
        self.kp = 1.0  # 比例ゲイン
        self.kd = 0.1  # 微分ゲイン

        # 前回のエラーを保存
        self.previous_left_error = 0.0
        self.previous_right_error = 0.0

        # 定速前進
        self.linear_x = 0.2  # 前進速度

    def listener_callback(self, msg):
        # データを取得（左・前方左・前方右・右の順で距離が得られると仮定）
        distances = msg.data
        self.left_distance = distances[0]
        self.front_left_distance = distances[1]
        self.front_right_distance = distances[2]
        self.right_distance = distances[3]

        # 安全距離を保つための制御
        self.control_robot()

        # 距離データのログを表示
        self.get_logger().info(f'Left: {self.left_distance}, Front Left: {self.front_left_distance}, Front Right: {self.front_right_distance}, Right: {self.right_distance}')

    def control_robot(self):
        # 壁からの距離誤差
        left_error = self.target_distance - self.left_distance
        right_error = self.target_distance - self.right_distance

        # 前方角度方向の距離が最小距離以下の場合の処理
        if self.front_left_distance < self.min_distance or self.front_right_distance < self.min_distance:
            # 壁に近づきすぎた場合、旋回して回避
            angular_z = 0.5 if self.front_right_distance < self.front_left_distance else -0.5
            control_msg = Twist()
            control_msg.linear.x = -0.1  # 少し後退して回避
            control_msg.angular.z = angular_z  # 壁を避ける方向に旋回

            self.get_logger().info(f'Obstacle detected! Backing and turning with angular z: {angular_z}')
            self.control_publisher.publish(control_msg)
            return  # 回避動作中のため、通常の制御はスキップ

        # 通常のPD制御による調整
        left_derivative = left_error - self.previous_left_error
        right_derivative = right_error - self.previous_right_error

        # 左右のエラーをPD制御で補正
        left_angular_z = self.kp * left_error + self.kd * left_derivative
        right_angular_z = self.kp * right_error + self.kd * right_derivative

        # 旋回方向の決定
        angular_z = 0.0
        if right_error > 0:
            angular_z = -right_angular_z  # 右回転
        elif left_error > 0:
            angular_z = left_angular_z  # 左回転

        # 通常の前進および旋回の制御メッセージを作成
        control_msg = Twist()
        control_msg.linear.x = self.linear_x
        control_msg.angular.z = angular_z

        self.get_logger().info(f'linear x: {self.linear_x}, angular z: {angular_z}')

        # 制御メッセージを送信
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
