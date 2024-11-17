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

        self.red_lower_detected = False
        self.previous_red_lower_detected = False
        self.red_lower_count = 0

        self.red_upper_detected = False
        self.previous_red_upper_detected = False
        self.red_upper_count = 0

        self.red_lower_no_detection_count = 0
        self.red_upper_no_detection_count = 0

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
        left_region = frame[:, :width // 4]  # 左端の1/4幅を切り出し
        upper_region = frame[:height//2, :width // 320] # 実際のテープは下側
        lower_region = frame[height//2:, :width // 320] # 実際のテープは上側

        # 色の閾値を設定
        red_threshold = 80  # 赤の閾値（強さ）
        green_threshold = 50  # 緑の閾値（低ければ低いほど赤と認識しやすい）
        blue_threshold = 50  # 青の閾値（低ければ低いほど赤と認識しやすい）

        # print(f"move : {self.move_msg}")

        if self.current_destination == "D":
            red_lower_pixels = np.where(
                (lower_region[:, :, 2] >= red_threshold) &  # 赤チャネルが閾値を超えている
                (lower_region[:, :, 1] <= green_threshold) &  # 緑チャネルが閾値を下回っている
                (lower_region[:, :, 0] <= blue_threshold)    # 青チャネルが閾値を下回っている
            )
            
            red_upper_pixels = np.where(
                (upper_region[:, :, 2] >= red_threshold) &  # 赤チャネルが閾値を超えている
                (upper_region[:, :, 1] <= green_threshold) &  # 緑チャネルが閾値を下回っている
                (upper_region[:, :, 0] <= blue_threshold)    # 青チャネルが閾値を下回っている
            )
            current_red_lower_detected = red_lower_pixels[0].size > 0

            if red_upper_pixels[0].size > 0:
                self.get_logger().info(f'Red upper pixels found at {list(zip(red_upper_pixels[0], red_upper_pixels[1]))}')
                self.stop_move()
                self.publish_goal_reached(self.current_destination)
                self.is_right_side_D = False
                self.red_lower_count = 0
                self.red_lower_no_detection_count = 0
                print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            elif current_red_lower_detected:
                # 検出された場合
                if self.red_lower_no_detection_count > 10:
                    if not self.previous_red_lower_detected:
                        self.red_lower_count += 1
                        self.red_lower_no_detection_count = 0
                        self.get_logger().info(f'red_lower_count = {self.red_lower_count}')
                        if self.red_lower_count == 2:
                            self.is_right_side_D = True
                            self.get_logger().info('right side D mode start')
                        elif self.red_lower_count == 3:
                            self.is_right_side_D = False
                            self.get_logger().info('right side D mode finish')
                        else:
                            self.get_logger().info('no happen')
                    # `previous_red_lower_detected` を更新
                    self.previous_red_lower_detected = True
                else:
                    self.get_logger().info(f'previous_red_lower_detected = {self.previous_red_lower_detected}')
            else:
                # 検出されていない場合は `previous_red_lower_detected` をリセット
                self.previous_red_lower_detected = False
                self.red_lower_no_detection_count += 1

            msg = Bool()
            msg.data = self.is_right_side_D
            self.right_side_D_publisher.publish(msg)
            
        
        elif self.current_destination == "C":
            red_upper_pixels = np.where(
                (upper_region[:, :, 2] >= red_threshold) &  # 赤チャネルが閾値を超えている
                (upper_region[:, :, 1] <= green_threshold) &  # 緑チャネルが閾値を下回っている
                (upper_region[:, :, 0] <= blue_threshold)    # 青チャネルが閾値を下回っている
            )
            red_lower_pixels = np.where(
                (lower_region[:, :, 2] >= red_threshold) &  # 赤チャネルが閾値を超えている
                (lower_region[:, :, 1] <= green_threshold) &  # 緑チャネルが閾値を下回っている
                (lower_region[:, :, 0] <= blue_threshold)    # 青チャネルが閾値を下回っている
            )

            current_red_upper_detected = red_upper_pixels[0].size > 0
            self.is_right_side_C = True

            if red_lower_pixels[0].size > 0:
                # self.get_logger().info(f'Red pixels found at {list(zip(red_lower_pixels[0], red_lower_pixels[1]))}')
                self.stop_move()
                self.publish_goal_reached(self.current_destination)
                self.is_right_side_C = False
                self.red_upper_count = 0

        elif self.current_destination in ["A", "E"]:
            # 赤色を判定する条件を追加
            red_pixels = np.where(
                (lower_region[:, :, 2] >= red_threshold) &  # 赤チャネルが閾値を超えている
                (lower_region[:, :, 1] <= green_threshold) &  # 緑チャネルが閾値を下回っている
                (lower_region[:, :, 0] <= blue_threshold)    # 青チャネルが閾値を下回っている
            )
            if red_pixels[0].size > 0:
                # self.get_logger().info(f'Red pixels found at {list(zip(red_pixels[0], red_pixels[1]))}')
                self.stop_move()
                self.publish_goal_reached(self.current_destination)
                # self.get_logger().info('Red pixels detected and stop!!')

        elif self.current_destination in ["F", "B"]:
            # 赤色を判定する条件を追加
            red_pixels = np.where(
                (upper_region[:, :, 2] >= red_threshold) &  # 赤チャネルが閾値を超えている
                (upper_region[:, :, 1] <= green_threshold) &  # 緑チャネルが閾値を下回っている
                (upper_region[:, :, 0] <= blue_threshold)    # 青チャネルが閾値を下回っている
            )

            if red_pixels[0].size > 0:
                self.get_logger().info(f'Blue pixels found at {list(zip(red_pixels[0], red_pixels[1]))}')
                self.stop_move()
                self.publish_goal_reached(self.current_destination)
                # self.get_logger().info('Red pixels detected and stop!!')

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