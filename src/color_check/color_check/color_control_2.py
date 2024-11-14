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

        self.get_logger().info('Image Monitor Node has started.')
        
        # 距離の範囲パラメータ
        self.threshold_near = 1    # 近いと判定する範囲
        self.threshold_far = 2     # 遠いと判定する範囲


    def image_callback(self, msg):
        # ROS2の画像メッセージをOpenCV形式に変換
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 赤色の閾値を設定
        red_threshold = 80
        green_threshold = 50
        blue_threshold = 50

        height, width, _ = frame.shape
        target_y = 51   #90
        # print(f"height = {height}")
        # left_region = frame[:, :width // 1]
        # 中央2列の範囲を指定
        left_region = frame[:, (width // 2) - 1:(width // 2) + 1]

        # 赤色を判定する条件
        red_pixels = np.where(
            (left_region[:, :, 2] >= red_threshold) &  # 赤チャネルが閾値を超えている
            (left_region[:, :, 1] <= green_threshold) &  # 緑チャネルが閾値を下回っている
            (left_region[:, :, 0] <= blue_threshold)    # 青チャネルが閾値を下回っている
        )

        # 赤色のピクセルが見つかった場合
        if red_pixels[0].size > 0:
            # 赤色ピクセルの位置（Y座標）の平均値を取る
            red_center_y = np.min(red_pixels[0])
            print(red_center_y)
            error = target_y - red_center_y

            # 距離に応じた制御
            if abs(error) <= self.threshold_near:
                self.control_robot("forward")  # 近い範囲内なので前進
            elif error > self.threshold_far:    # 近い（大きい）
                self.control_robot("left")  # 近いので左旋回
            elif error < -self.threshold_far:   # 遠い(小さい)
                self.control_robot("right")  # 遠いので右旋回
            else:
                self.control_robot("foward")  # 範囲外だが動く必要がない場合
            self.get_logger().info(f'Error: {error}, Action: {self.action}')
        else:
            self.control_robot("stop")
            self.get_logger().info("No red pixels found, Action: stop")

            
    
    def control_robot(self, action):
        # 制御信号に基づいてロボットの動作を制御するコード
        self.action = action
        if action == "forward":
            self.get_logger().info('Move forward')
            self.vel.linear.x = 0.3
            self.vel.angular.z = 0.0
            self.publisher.publish(self.vel)
        elif action == "left":
            self.get_logger().info('Turn left')
            self.vel.linear.x = 0.3
            self.vel.angular.z = 0.2
            self.publisher.publish(self.vel)
        elif action == "right":
            self.get_logger().info('Turn right')
            self.vel.linear.x = 0.3
            self.vel.angular.z = -0.2
            self.publisher.publish(self.vel)
        # else:
        #     self.get_logger().info('Stop')
        #     self.vel.linear.x = 0.0
        #     self.vel.angular.z = 0.0
        #     self.publisher.publish(self.vel)

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
