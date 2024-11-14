import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist

class ImageMonitorNode(Node):
    def __init__(self):
        super().__init__('color_control')
        self.subscription = self.create_subscription(
            Image, '/image_raw', self.image_callback, 1)
        self.bridge = CvBridge()

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 1)
        self.vel = Twist()
        self.vel_x = 0.3

        self.get_logger().info('Image Monitor Node has started.')
        
        # PD制御用のパラメータ
        self.Kp = 0.1  # 比例ゲイン
        self.Kd = 0.01  # 微分ゲイン
        self.prev_error = 0  # 前回の誤差
        self.prev_time = None  # 前回の時間（微分項の計算のため）
        self.arg_z = 0

    def image_callback(self, msg):
        # ROS2の画像メッセージをOpenCV形式に変換
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 赤色の閾値を設定
        red_threshold = 80
        green_threshold = 50
        blue_threshold = 50

        height, width, _ = frame.shape
        target_y = 320
        # print(f"height = {height}")
        left_region = frame[:, :width // 240]

        # 赤色を判定する条件
        red_pixels = np.where(
            (left_region[:, :, 2] >= red_threshold) &  # 赤チャネルが閾値を超えている
            (left_region[:, :, 1] <= green_threshold) &  # 緑チャネルが閾値を下回っている
            (left_region[:, :, 0] <= blue_threshold)    # 青チャネルが閾値を下回っている
        )

        # 赤色のピクセルが見つかった場合
        if red_pixels[0].size > 0:
            # 赤色ピクセルの位置（Y座標）の平均値を取る
            red_center_y = np.mean(red_pixels[0])

            # 誤差（画像中心との差）
            error = red_center_y - target_y
            self.arg_z = 0.01 * error
            # print(f"error = {error}")
            # print(f"arg_z = {arg_z}")

            # PD制御
            current_time = self.get_clock().now().to_msg().sec
            if self.prev_time is None:
                delta_time = 1  # 初回の場合
            else:
                delta_time = current_time - self.prev_time

            # 微分項
            delta_error = error - self.prev_error
            control_signal = self.Kp * error 
            # control_signal = self.Kp * error + self.Kd * (delta_error / delta_time)

            # ロボットへの制御信号を出力
            self.control_robot(control_signal)

            # 前回の誤差と時間を更新
            self.prev_error = error
            self.prev_time = current_time

            # self.get_logger().info(f'Control signal (Y方向): {control_signal}')

    def control_robot(self, control_signal):
        # 制御信号に基づいてロボットの動作を制御するコードを追加
        # ここでは簡単に進行方向を制御するロジックを入れることができます
        if control_signal > 0:
            self.get_logger().info('Move up')
            self.vel.linear.x = self.vel_x
            self.vel.angular.z = self.arg_z
            self.publisher.publish(self.vel)

        elif control_signal < 0:
            self.get_logger().info('Move down')
            self.vel.linear.x = self.vel_x
            self.vel.angular.z = self.arg_z
            self.publisher.publish(self.vel)
        else:
            self.get_logger().info('Move forward')
            self.vel.linear.x = self.vel_x
            self.vel.angular.z = 0.0
            self.publisher.publish(self.vel)

        self.get_logger().info("Velocity: Linear=%f" % (self.vel.linear.x))
        self.get_logger().info("Velocity: Angular=%f" % (self.vel.angular.z))

def main(args=None):
    rclpy.init(args=args)
    node = ImageMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
