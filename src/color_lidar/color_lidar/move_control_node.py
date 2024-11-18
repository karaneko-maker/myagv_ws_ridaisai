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

        self.move_publisher = self.create_publisher(Bool, '/move_msg', 10)
        self.right_side_D_publisher = self.create_publisher(Bool, '/right_side_D_msg', 10)
        self.right_side_C_publisher = self.create_publisher(Bool, '/right_side_C_msg', 10)
        self.right_side_publisher = self.create_publisher(Bool, '/right_side_msg', 10)
        self.move_msg = False
        self.is_right_side_D = False
        self.is_right_side_C = False
        self.is_right_side = False

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/image_raw', self.image_callback, 10)

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

        self.camera_move_subscription = self.create_subscription(Bool, '/camera_move_msg',  self.camera_move_callback, 10)
        self.camera_move = True
    
    def camera_move_callback(self, msg):
        self.camera_move = msg.data
        
        # if self.camera_move:
        #     self.get_logger().info("Camera move!")
        # else:
        #     self.get_logger().info("Camera stop")

    def listener_callback(self, msg):
        # self.get_logger().info(f'Received message on {self.get_namespace()}/command: {msg.data}')
        command = msg.data.strip().lower()
        
        # 目標地の選択と移動開始
        if command.startswith("go to "):
            destination_key = command[6:].upper()
            # self.get_logger().info(f'current destination key: {destination_key}')
            if destination_key in ["A", "B", "C", "F"]:
                self.current_destination = destination_key
                self.activate_move()
            elif destination_key == "E":
                self.current_destination = destination_key
                self.activate_move_E()
            elif destination_key == "D":
                self.current_destination = destination_key
                self.activate_move_D()
            elif destination_key == "C":
                self.current_destination = destination_key
                self.activate_move_C()
            # else:
            #     # self.get_logger().info(f'Unknown destination: {destination_key}')
        
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
    
    def activate_move_C(self):
        # 右側走行（左旋回）モード
        self.is_right_side = True
        msg = Bool()
        msg.data = self.is_right_side
        self.right_side_publisher.publish(msg)
        self.move_msg = Bool()
        self.move_msg.data = True
        self.move_publisher.publish(self.move_msg)
    
    def activate_move_D(self):
        # 右側走行（左旋回）モード
        self.is_right_side = True
        msg = Bool()
        msg.data = self.is_right_side
        self.right_side_publisher.publish(msg)
        self.move_msg = Bool()
        self.move_msg.data = True
        self.move_publisher.publish(self.move_msg)

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
            if self.camera_move:
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

                # if red_upper_pixels[0].size > 0:
                #     self.get_logger().info(f'Red upper pixels found at {list(zip(red_upper_pixels[0], red_upper_pixels[1]))}')
                #     self.stop_move()
                #     self.publish_goal_reached(self.current_destination)
                #     self.is_right_side_D = False
                #     self.is_right_side = False
                #     self.red_lower_count = 0
                #     self.red_lower_no_detection_count = 0
                #     print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                if current_red_lower_detected:
                    if not self.previous_red_lower_detected:
                        self.red_lower_count += 1
                        print(f"red_lower_count = {self.red_lower_count}")

                        if self.red_lower_count == 2:
                            self.is_right_side_D = True
                            self.get_logger().info('right side D mode start')
                        elif self.red_lower_count == 3:
                            self.is_right_side_D = False
                            self.is_right_side = True
                            self.get_logger().info('right side mode start')
                        elif self.red_lower_count == 4:
                            self.get_logger().info(f'Red upper pixels found at {list(zip(red_upper_pixels[0], red_upper_pixels[1]))}')
                            self.stop_move()
                            self.publish_goal_reached(self.current_destination)
                            self.is_right_side_D = False
                            self.is_right_side = False
                            self.red_lower_count = 0
                            self.red_lower_no_detection_count = 0
                        else:
                            self.get_logger().info('no happen')
                        # `previous_red_lower_detected` を更新
                        self.previous_red_lower_detected = True
                        # else:
                        #     self.get_logger().info(f'previous_red_lower_detected = {self.previous_red_lower_detected}')
                else:
                    # 検出されていない場合は `previous_red_upper_detected` をリセット
                    self.previous_red_lower_detected = False

                msg = Bool()
                msg.data = self.is_right_side_D
                self.right_side_D_publisher.publish(msg)
                # if self.is_right_side_D:
                #     self.get_logger().info("Right Side D!")
                msg2 = Bool()
                msg2.data = self.is_right_side
                self.right_side_publisher.publish(msg2)
            
        
        elif self.current_destination == "C":
            if self.camera_move:
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

                if red_lower_pixels[0].size > 0:
                    # self.get_logger().info(f'Red pixels found at {list(zip(red_lower_pixels[0], red_lower_pixels[1]))}')
                    self.stop_move()
                    self.publish_goal_reached(self.current_destination)
                    self.is_right_side_C = False
                    self.red_upper_count = 0
                elif current_red_upper_detected:
                    # 検出された場合
                    if self.red_upper_no_detection_count > 10:
                        if not self.previous_red_upper_detected:
                            self.red_upper_count += 1
                            self.red_upper_no_detection_count = 0
                            self.get_logger().info(f'red_upper_count = {self.red_upper_count}')
                            
                            if self.red_upper_count == 1:
                                self.is_right_side_C = False
                                self.is_right_side = True
                                self.get_logger().info('right side mode start')
                            if self.red_upper_count == 2:
                                self.is_right_side_C = True
                                self.is_right_side = False
                                self.get_logger().info('right side D mode start')
                            else:
                                self.get_logger().info('no happen')
                        # `previous_red_lower_detected` を更新
                        self.previous_red_upper_detected = True
                    else:
                        self.get_logger().info(f'previous_red_upper_detected = {self.previous_red_upper_detected}')
                else:
                    # 検出されていない場合は `previous_red_upper_detected` をリセット
                    self.previous_red_upper_detected = False
                    self.red_upper_no_detection_count += 1

                msg1 = Bool()
                msg1.data = self.is_right_side_C
                self.right_side_C_publisher.publish(msg1)
                msg2 = Bool()
                msg2.data = self.is_right_side
                self.right_side_publisher.publish(msg2)
        
        elif self.current_destination == "B":
            if self.camera_move:
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
        
        elif self.current_destination == "A":
            if self.camera_move:
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

                if red_upper_pixels[0].size > 0 and red_lower_pixels[0].size == 0:
                    self.stop_move()
                    self.publish_goal_reached(self.current_destination)
                    # self.get_logger().info('Red pixels detected and stop!!')

        elif self.current_destination == "E":
            if self.camera_move:
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

        elif self.current_destination == "F":
            if self.camera_move:
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
        message.data = f"{agv_id} arrived {destination_key}O"
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