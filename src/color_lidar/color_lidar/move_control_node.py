import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_msgs.msg import Bool, String
import numpy as np

class MoveControlNode(Node):

    def __init__(self, namespace=''):
        super().__init__('move_control_node', namespace=namespace)
        self.publisher_ = self.create_publisher(String, f'{namespace}/agv_reach', 10)
        self.subscription = self.create_subscription(String, f'{namespace}/command', self.listener_callback, 10)  

        self.move_publisher = self.create_publisher(Bool, '/move_msg', 1)
        self.right_side_D_publisher = self.create_publisher(Bool, '/rihgt_side_D_msg', 1)
        self.right_side_C_publisher = self.create_publisher(Bool, '/rihgt_side_C_msg', 1)
        self.move_msg = False
        self.is_right_side_D = False
        self.is_right_side_C = False

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/image_raw', self.image_callback, 1)

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 1)

        self.blue_detected = False  # 青検出フラグ
        self.previous_blue_detected = False
        self.blue_count = 0
        self.green_detected = False  # 緑検出フラグ
        self.red_detected = False   # 赤検出フラグ

        self.current_destination = None

    def listener_callback(self, msg):
        self.get_logger().info(f'Received message on {self.get_namespace()}/command: {msg.data}')
        command = msg.data.strip().lower()
        
        # 目標地の選択と移動開始
        if command.startswith("go to "):
            destination_key = command[6:].upper()
            self.get_logger().info(f'current destination key: {destination_key}')
            if destination_key in ["A", "B", "C", "D", "F"]:
                self.current_destination = destination_key
                self.activate_move()
            elif destination_key == "E":
                self.current_destination = destination_key
                self.activate_move_E()
            else:
                self.get_logger().info(f'Unknown destination: {destination_key}')
        
    def activate_move(self):
        # 出発するためのコマンドを送信
        self.move_msg = Bool()
        self.move_msg.data = True
        self.move_publisher.publish(self.move_msg)
        self.get_logger().info('Moving to destination...')
    
    def activate_move_E(self):
        # 直進
        twist = Twist()
        twist.linear.x = 0.4
        twist.angular.z = 0.0
        self.publisher.publish(twist)
        self.get_logger().info('Go Straight...')


    def image_callback(self, msg):
        # ROS2の画像メッセージをOpenCV形式に変換
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 画像の左端を切り出す（例えば、画像の幅の半分）
        height, width, _ = frame.shape
        left_region = frame[:, :width // 1]  # 左端の1/4幅を切り出し

        # 色の閾値を設定
        red_threshold = 80  # 赤の閾値（強さ）
        green_threshold = 50  # 緑の閾値（低ければ低いほど赤と認識しやすい）
        blue_threshold = 50  # 青の閾値（低ければ低いほど赤と認識しやすい）

        # print(f"move : {self.move_msg}")

        if self.current_destination == "D":
            blue_pixels = np.where(
                (left_region[:, :, 0] >= blue_threshold) &  # 青チャネルが閾値を超えている
                (left_region[:, :, 1] <= green_threshold) &  # 緑チャネルが閾値を下回っている
                (left_region[:, :, 2] <= red_threshold)      # 赤チャネルが閾値を下回っている
            )
            green_pixels = np.where(
                (left_region[:, :, 0] <= blue_threshold) &  # 青チャネルが閾値を超えている
                (left_region[:, :, 1] >= green_threshold) &  # 緑チャネルが閾値を下回っている
                (left_region[:, :, 2] <= red_threshold)      # 赤チャネルが閾値を下回っている
            )

            current_blue_detected = blue_pixels[0].size > 0

            if green_pixels[0].size > 0:
                self.get_logger().info(f'Green pixels found at {list(zip(green_pixels[0], green_pixels[1]))}')
                self.stop_move()
                self.publish_goal_reached(self.current_destination)
                self.is_right_side_D = False
                self.blue_count = 0
                print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            elif current_blue_detected:
                # 青が検出された場合
                if not self.previous_blue_detected:
                    self.blue_count += 1
                    self.get_logger().info(f'blue_count = {self.blue_count}')
                    if self.blue_count == 1:
                        self.is_right_side_D = True
                        self.get_logger().info('right side D mode start')
                    else:
                        self.is_right_side_D = False
                        self.get_logger().info('right side D mode finish')
                # `previous_blue_detected` を更新
                self.previous_blue_detected = True
            else:
                # 青が検出されていない場合は `previous_blue_detected` をリセット
                self.previous_blue_detected = False

            msg = Bool()
            msg.data = self.is_right_side_D
            self.right_side_D_publisher.publish(msg)
            

        
        elif self.current_destination == "C":
            blue_pixels = np.where(
                (left_region[:, :, 0] >= blue_threshold) &  # 青チャネルが閾値を超えている
                (left_region[:, :, 1] <= green_threshold) &  # 緑チャネルが閾値を下回っている
                (left_region[:, :, 2] <= red_threshold)      # 赤チャネルが閾値を下回っている
            )
            red_pixels = np.where(
                (left_region[:, :, 2] >= red_threshold) &  # 赤チャネルが閾値を超えている
                (left_region[:, :, 1] <= green_threshold) &  # 緑チャネルが閾値を下回っている
                (left_region[:, :, 0] <= blue_threshold)    # 青チャネルが閾値を下回っている
            )

            current_blue_detected = blue_pixels[0].size > 0

            if red_pixels[0].size > 0:
                # self.get_logger().info(f'Red pixels found at {list(zip(red_pixels[0], red_pixels[1]))}')
                self.red_detected = True
                self.stop_move()
                self.publish_goal_reached(self.current_destination)
                self.blue_detected = False
                self.red_detected = False
                self.is_right_side_C = False
                self.blue_count = 0
            elif current_blue_detected:
                # 青が検出された場合
                if not self.previous_blue_detected:
                    self.blue_count += 1
                    self.get_logger().info(f'blue_count = {self.blue_count}')
                    if self.blue_count == 1:
                        self.is_right_side_D = True
                        self.get_logger().info('right side D mode start')
                    else:
                        self.is_right_side_D = False
                        self.get_logger().info('right side D mode finish')
                # `previous_blue_detected` を更新
                self.previous_blue_detected = True
            else:
                # 青が検出されていない場合は `previous_blue_detected` をリセット
                self.previous_blue_detected = False

            msg = Bool()
            msg.data = self.is_right_side_C
            self.right_side_D_publisher.publish(msg)

        elif self.current_destination in ["A", "E"]:
            # 赤色を判定する条件を追加
            red_pixels = np.where(
                (left_region[:, :, 2] >= red_threshold) &  # 赤チャネルが閾値を超えている
                (left_region[:, :, 1] <= green_threshold) &  # 緑チャネルが閾値を下回っている
                (left_region[:, :, 0] <= blue_threshold)    # 青チャネルが閾値を下回っている
            )
            if red_pixels[0].size > 0:
                # self.get_logger().info(f'Red pixels found at {list(zip(red_pixels[0], red_pixels[1]))}')
                self.stop_move()
                self.publish_goal_reached(self.current_destination)
                # self.get_logger().info('Red pixels detected and stop!!')

        elif self.current_destination in ["F", "B"]:
            # 青色を判定する条件を追加
            # blue_pixels = np.where(
            #     (left_region[:, :, 0] >= red_threshold) &  # 赤チャネルが閾値を超えている
            #     (left_region[:, :, 1] <= green_threshold) &  # 緑チャネルが閾値を下回っている
            #     (left_region[:, :, 2] <= blue_threshold)    # 青チャネルが閾値を下回っている
            # )
            blue_pixels = np.where(
                (left_region[:, :, 0] >= blue_threshold) &  # 青チャネルが閾値を超えている
                (left_region[:, :, 1] <= green_threshold) &  # 緑チャネルが閾値を下回っている
                (left_region[:, :, 2] <= red_threshold)      # 赤チャネルが閾値を下回っている
            )

            if blue_pixels[0].size > 0:
                # self.get_logger().info(f'Blue pixels found at {list(zip(blue_pixels[0], blue_pixels[1]))}')
                self.stop_move()
                self.publish_goal_reached(self.current_destination)
                # self.get_logger().info('Blue pixels detected and stop!!')

    def stop_move(self):
        # lidar node を停止するためのコマンドを送信
        self.move_msg = Bool()
        self.move_msg.data = False
        self.move_publisher.publish(self.move_msg)
        # self.get_logger().info('Stop lidar node')

        # 速度0を出す
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)
    
    def publish_goal_reached(self, destination_key):
        agv_id = "agv1"  # ここでAGVのIDを設定する
        message = String()
        message.data = f"{agv_id} arrived {destination_key}"
        self.publisher_.publish(message)
        self.get_logger().info(f'Published: {agv_id} arrived {destination_key}!')
    
def main(args=None):
    rclpy.init(args=args)
    namespace = ''  # 設定するnamespace
    node = MoveControlNode(namespace=namespace)

    # Keep the node spinning and accepting messages
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()