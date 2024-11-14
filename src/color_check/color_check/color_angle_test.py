import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np

class ParallelWallController(Node):
    def __init__(self):
        super().__init__('color_angle_test')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.image_sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        
        self.bridge = CvBridge()
        
        # 車体の状態
        self.aligned_to_wall = False
        self.wall_distance = 0.5  # 目標とする壁までの距離

    def image_callback(self, msg):
        # 画像データを取得
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # 画像から赤い線の角度を計算
        angle = self.detect_red_line_angle(cv_image)
        
        # 角度が許容範囲内であれば並進開始
        if abs(angle) < 5:  # 5度以内の許容誤差
            self.aligned_to_wall = True
        else:
            self.aligned_to_wall = False
        
        # 並進と回転を分離した制御
        if not self.aligned_to_wall:
            self.rotate_to_align(angle)
        else:
            self.move_straight()

    def detect_red_line_angle(self, image):
        # 赤色の閾値を設定
        red_threshold = 80
        green_threshold = 50
        blue_threshold = 50

        # 赤色のマスクを生成
        red_pixels = np.where(
            (image[:, :, 2] >= red_threshold) &  # 赤チャネルが閾値を超える
            (image[:, :, 1] <= green_threshold) &  # 緑チャネルが閾値を下回る
            (image[:, :, 0] <= blue_threshold)    # 青チャネルが閾値を下回る
        )

        # 赤色のピクセル位置で真っ白（255）、それ以外は黒（0）のマスク画像を作成
        mask = np.zeros(image.shape[:2], dtype=np.uint8)
        mask[red_pixels] = 255

        # マスクから輪郭を抽出
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 赤色の線が検出できなかった場合
        if not contours:
            return 0  # 角度を0とする

        # 最大の輪郭を選択
        largest_contour = max(contours, key=cv2.contourArea)

        # 最小二乗直線フィッティングを行い、傾きを計算
        [vx, vy, x, y] = cv2.fitLine(largest_contour, cv2.DIST_L2, 0, 0.01, 0.01)

        # 傾き（vy/vx）から角度を計算
        angle = np.degrees(np.arctan2(vy, vx))
        print(f"angle = {angle}")
        return angle

    def rotate_to_align(self, angle):
        # 車体の角度を調整
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.3  # 回転方向を設定
        self.cmd_pub.publish(twist)
        # self.get_logger().info(f'Rotating to align: angle={angle}')

    def move_straight(self):
        # 車体の並進を制御
        twist = Twist()
        twist.linear.x = 0.3  # 前進速度
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        self.get_logger().info(f'Moving straight, aligned with wall')

def main(args=None):
    rclpy.init(args=args)
    controller = ParallelWallController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
