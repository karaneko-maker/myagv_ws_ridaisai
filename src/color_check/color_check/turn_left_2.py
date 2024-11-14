import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ParallelWallController(Node):
    def __init__(self):
        super().__init__('turn_left_2')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.image_sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()

        # 色検出用のしきい値
        self.red_threshold = 80
        self.green_threshold = 50
        self.blue_threshold = 80

        # 状態の初期化
        self.mode = "normal"  # "normal", "turn_left_larger"
        self.seen_blue = False  # 青を見たかどうかのフラグ
        
    def image_callback(self, msg):
        # ROSのImageメッセージをOpenCVの画像に変換
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 画像の高さと幅を取得
        height, width, _ = cv_image.shape

        # 画像の上半分を切り出し
        upper_half = cv_image[:height // 2 , :]
        # 画像を表示（デバッグ用）
        cv2.imshow("Received Image", upper_half)
        cv2.waitKey(1)  # 画像ウィンドウを更新


        # 青色、赤色領域を検出 (指定されたしきい値に基づいて)
        blue_pixels = np.where(
            (upper_half[:, :, 0] >= self.blue_threshold) & 
            (upper_half[:, :, 1] <= self.green_threshold) & 
            (upper_half[:, :, 2] <= self.red_threshold)
        )

        red_pixels = np.where(
            (upper_half[:, :, 2] >= self.red_threshold) & 
            (upper_half[:, :, 1] <= self.green_threshold) & 
            (upper_half[:, :, 0] <= self.blue_threshold)
        )

        # 青色領域が見つかった場合、"turn left larger"モードに入る
        if blue_pixels[0].size > 0:
            if not self.seen_blue:
                self.seen_blue = True  # 青を見たことにする
                self.get_logger().info('Blue detected, switching to "turn_left_larger" mode')
            if self.mode != "turn_left_larger":
                self.mode = "turn_left_larger"
                self.turn_left_larger()

        # 青色が見えなくなった場合、"normal"モードに戻す
        elif blue_pixels[0].size == 0 and self.seen_blue:
            self.seen_blue = False
            if self.mode != "normal":
                self.mode = "normal"
                self.get_logger().info('Blue no longer detected, switching back to "normal" mode')
                print("1!!!!!!!!!!!!!!!")
                self.turn_right()  # 右旋回で元の動作に戻す

        # 赤色領域が見つかった場合の処理
        elif self.mode == "normal" and red_pixels[0].size > 0:
            # 赤色領域がある場合、y座標に応じて制御
            y_coords = red_pixels[0]
            if min(y_coords) < 60:
                # 左旋回を行う
                print(f"Red detected, y = {y_coords} < 60")
                self.turn_left()
            else:
                # 右旋回を行う
                print(f"Red detected, y = {y_coords} >= 60")
                self.turn_right()

        else:
            print("error!!!!!!!!!!!!!!!!!!!!!!")

    def turn_left(self):
        # 左旋回のコマンドを送信
        twist = Twist()
        twist.linear.x = 0.3
        twist.angular.z = 0.4  # 左旋回
        self.cmd_pub.publish(twist)
        self.get_logger().info('Turning Left')

    def turn_right(self):
        # 右旋回のコマンドを送信
        twist = Twist()
        twist.linear.x = 0.3
        twist.angular.z = -0.4  # 右旋回
        self.cmd_pub.publish(twist)
        self.get_logger().info('Turning Right')

    def turn_left_larger(self):
        # 「turn left larger」モードのコマンドを送信
        twist = Twist()
        twist.linear.x = 0.2
        twist.angular.z = 0.8  # 左旋回を強化
        self.cmd_pub.publish(twist)
        self.get_logger().info('Turning Left Larger')


def main(args=None):
    rclpy.init(args=args)
    node = ParallelWallController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
