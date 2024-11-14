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
        
        # 状態とパラメータ
        self.aligned_to_wall = False
        self.threshold_angle = 1.0  # 角度の許容範囲（度）
        self.threshold_near = 1.0   # 赤のY位置の近い範囲
        self.threshold_far = 2.0    # 赤のY位置の遠い範囲
        self.target_y = 51          # 赤色の基準Y位置 90

    def image_callback(self, msg):
        # 画像データを取得
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # 赤線の傾きと位置をチェック
        angle = self.detect_red_line_angle(cv_image)
        
        # 傾きのチェック
        if abs(angle) < self.threshold_angle:  # 傾きがしきい値以内
            self.aligned_to_wall = True
            print("Iam straight*************************")
            red_center_y = self.detect_red_line_position(cv_image)
            
            # 赤線の位置に応じた制御
            if red_center_y is not None:
                error = self.target_y - red_center_y
                if abs(error) <= self.threshold_near:
                    self.control_robot("forward")  # 近い範囲なので前進
                elif error > self.threshold_far:
                    self.control_robot("left")  # 赤が近いので左旋回
                elif error < -self.threshold_far:
                    self.control_robot("right")  # 赤が遠いので右旋回
                else:
                    self.control_robot("forward")  # 動作不要な場合
            else:
                self.control_robot("stop")  # 赤線が見つからない場合
        else:
            self.aligned_to_wall = False
            self.rotate_to_align(angle)  # 傾きが大きい場合回転調整

    def detect_red_line_angle(self, image):
        red_threshold = 80
        green_threshold = 50
        blue_threshold = 50

        red_pixels = np.where(
            (image[:, :, 2] >= red_threshold) & 
            (image[:, :, 1] <= green_threshold) & 
            (image[:, :, 0] <= blue_threshold)
        )

        mask = np.zeros(image.shape[:2], dtype=np.uint8)
        mask[red_pixels] = 255
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return 0  # 赤線が見つからない場合は角度を0とする

        largest_contour = max(contours, key=cv2.contourArea)
        [vx, vy, x, y] = cv2.fitLine(largest_contour, cv2.DIST_L2, 0, 0.01, 0.01)
        angle = np.degrees(np.arctan2(vy, vx))
        print(f"angle = {angle}")
        return angle

    def detect_red_line_position(self, image):
        red_threshold = 80
        green_threshold = 50
        blue_threshold = 50
        height, width, _ = image.shape
        left_region = image[:, (width // 2) - 1:(width // 2) + 1]

        red_pixels = np.where(
            (left_region[:, :, 2] >= red_threshold) &
            (left_region[:, :, 1] <= green_threshold) &
            (left_region[:, :, 0] <= blue_threshold)
        )

        if red_pixels[0].size > 0:
            return np.mean(red_pixels[0])  # 赤色の平均値を返す
        return None

    def rotate_to_align(self, angle):
        print("I am rotating!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        twist = Twist()
        twist.linear.x = 0.3
        twist.angular.z = -0.3 if angle > 0 else 0.3  # 角度に応じて回転方向を設定
        self.cmd_pub.publish(twist)
        print(f"cmd_vel = {twist.angular.z}")

    def control_robot(self, action):
        twist = Twist()
        if action == "forward":
            twist.linear.x = 0.3
            twist.angular.z = 0.0
        elif action == "left":
            twist.linear.x = 0.3
            twist.angular.z = 0.2
        elif action == "right":
            twist.linear.x = 0.3
            twist.angular.z = -0.2
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        self.get_logger().info(f'Action: {action}, Linear={twist.linear.x}, Angular={twist.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    controller = ParallelWallController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
