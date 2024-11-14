import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ParallelWallController(Node):
    def __init__(self):
        super().__init__('angle_position_control')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.image_sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()

        # 赤色検出用のしきい値
        self.red_threshold = 80
        self.green_threshold = 50
        self.blue_threshold = 50

    def image_callback(self, msg):
        # ROSのImageメッセージをOpenCVの画像に変換
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 画像の高さと幅を取得
        height, width, _ = cv_image.shape

        # 画像の上半分を切り出し
        upper_half = cv_image[:height // 2, :]

        # 赤色領域を検出 (指定されたしきい値に基づいて)
        red_pixels = np.where(
            (upper_half[:, :, 2] >= self.red_threshold) & 
            (upper_half[:, :, 1] <= self.green_threshold) & 
            (upper_half[:, :, 0] <= self.blue_threshold)
        )

        # 赤色領域が存在する場合
        if red_pixels[0].size > 0:
            # 赤色領域のy座標を取得
            y_coords = red_pixels[0]
            
            # 赤色領域のy座標が60未満か確認
            if min(y_coords) < 60:
                # 左旋回を行う
                print(f"y = {y_coords} < 55")
                self.turn_left()
            else:
                # 右旋回を行う
                print(f"y = {y_coords} > 55")
                self.turn_right()
        else:
            # 赤色領域が検出されない場合は右旋回
            self.turn_right()

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


def main(args=None):
    rclpy.init(args=args)
    node = ParallelWallController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
