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
        self.right_side_B_publisher = self.create_publisher(Bool, '/right_side_B_msg', 10)
        self.right_side_A_publisher = self.create_publisher(Bool, '/right_side_A_msg', 10)
        self.right_side_F_publisher = self.create_publisher(Bool, '/right_side_F_msg', 10)
        self.right_side_publisher = self.create_publisher(Bool, '/right_side_msg', 10)
        self.move_msg = False
        self.is_right_side_D = False
        self.is_right_side_C = False
        self.is_right_side_B = False
        self.is_right_side_A = False
        self.is_right_side_F = False
        self.is_right_side = False

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/image_raw', self.image_callback, 10)

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.red_lower_detected = False
        self.previous_red_lower_detected = False
        self.red_lower_count = 0

        self.red_upper_detected = False
        self.previous_red_upper_detected = False
        self.red_upper_count = 0

        self.red_detected = False
        self.previous_red_detected = False
        self.red_count = 0

        self.red_lower_no_detection_count = 0
        self.red_upper_no_detection_count = 0

        self.current_destination = None

        self.camera_move_subscription = self.create_subscription(Bool, '/camera_move_msg',  self.camera_move_callback, 10)
        self.D_stop_subscription = self.create_subscription(Bool, '/D_stop_msg',  self.D_stop_callback, 10)
        self.camera_move = True
        self.D_stop = False

        self.nb = 0
        self.na = 0
        self.nf = 0
        self.ne = 0
        self.nd = 0
    
    def camera_move_callback(self, msg):
        self.camera_move = msg.data
        
        # if self.camera_move:
        #     self.get_logger().info("Camera move!")
        # else:
        #     self.get_logger().info("Camera stop")

    def D_stop_callback(self, msg):
        self.D_stop = msg.data

    def listener_callback(self, msg):
        # self.get_logger().info(f'Received message on {self.get_namespace()}/command: {msg.data}')
        command = msg.data.strip().lower()
        
        # 目標地の選択と移動開始
        if command.startswith("go to "):
            destination_key = command[6:].upper()
            # self.get_logger().info(f'current destination key: {destination_key}')
            if destination_key == "E":
                self.current_destination = destination_key
                self.activate_move_E()
            elif destination_key == "D":
                self.current_destination = destination_key
                self.nd = 0
                self.activate_move_D()
            elif destination_key == "C":
                self.current_destination = destination_key
                self.activate_move_C()
            elif destination_key == "B":
                self.current_destination = destination_key
                self.nb = 0
                self.activate_move_B()
            elif destination_key == "A":
                self.current_destination = destination_key
                self.na = 0
                self.activate_move_A()
            elif destination_key == "F":
                self.current_destination = destination_key
                self.nf = 0
                self.activate_move_F()
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
        self.get_logger().info('Mode D Start...')
    
    def activate_move_B(self):
        # 右側走行モード
        self.is_right_side_B = True
        msg = Bool()
        msg.data = self.is_right_side_B
        self.right_side_B_publisher.publish(msg)
        self.move_msg = Bool()
        self.move_msg.data = True
        self.move_publisher.publish(self.move_msg)
    
    def activate_move_A(self):
        self.is_right_side_A = True
        msg = Bool()
        msg.data = self.is_right_side_A
        self.right_side_A_publisher.publish(msg)
        self.move_msg = Bool()
        self.move_msg.data = True
        self.move_publisher.publish(self.move_msg)
    
    def activate_move_F(self):
        self.is_right_side_F = True
        msg = Bool()
        msg.data = self.is_right_side_F
        self.right_side_F_publisher.publish(msg)
        self.move_msg = Bool()
        self.move_msg.data = True
        self.move_publisher.publish(self.move_msg)
        self.get_logger().info('Go to F')

    def image_callback(self, msg):
        # ROS2の画像メッセージをOpenCV形式に変換
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 画像の左端を切り出す（例えば、画像の幅の半分）
        height, width, _ = frame.shape
        left_region = frame[:, :width // 160]  # 左端の1/4幅を切り出し
        upper_region = frame[:height//2, :width // 320] # 実際のテープは下側
        lower_region = frame[height//2:, :width // 320] # 実際のテープは上側

        # 色の閾値を設定
        red_threshold = 80  # 赤の閾値（強さ）
        green_threshold = 50  # 緑の閾値（低ければ低いほど赤と認識しやすい）
        blue_threshold = 50  # 青の閾値（低ければ低いほど赤と認識しやすい）

        # print(f"move : {self.move_msg}")

        if self.current_destination == "D":
            if self.camera_move:
                # 赤色を判定する条件を追加
                red_pixels = np.where(
                    (left_region[:, :, 2] >= red_threshold) &  # 赤チャネルが閾値を超えている
                    (left_region[:, :, 1] <= green_threshold) &  # 緑チャネルが閾値を下回っている
                    (left_region[:, :, 0] <= blue_threshold)    # 青チャネルが閾値を下回っている
                )
                if red_pixels[0].size == 0:
                    self.na += 1
                
                if self.na > 30:
                    if red_pixels[0].size > 0:
                        self.is_right_side_D = True
                        self.is_right_side = False

            if self.D_stop:
                self.stop_move()
                self.publish_goal_reached(self.current_destination)
                self.is_right_side_D = False
                self.is_right_side = False

            if self.is_right_side_D:
                self.get_logger().info(f'WAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA')
            
            msg = Bool()
            msg.data = self.is_right_side_D
            self.right_side_D_publisher.publish(msg)
            msg2 = Bool()
            msg2.data = self.is_right_side
            self.right_side_publisher.publish(msg2)
            
        
        elif self.current_destination == "C":
            # if self.camera_move:
            red_pixels = np.where(
                (left_region[:, :, 2] >= red_threshold) &  # 赤チャネルが閾値を超えている
                (left_region[:, :, 1] <= green_threshold) &  # 緑チャネルが閾値を下回っている
                (left_region[:, :, 0] <= blue_threshold)    # 青チャネルが閾値を下回っている
            )

            self.is_right_side_C = True
            if red_pixels[0].size > 0:
                # self.get_logger().info(f'Red pixels found at {list(zip(red_lower_pixels[0], red_lower_pixels[1]))}')
                self.stop_move()
                self.publish_goal_reached(self.current_destination)
                self.is_right_side_C = False
    
            msg1 = Bool()
            msg1.data = self.is_right_side_C
            self.right_side_C_publisher.publish(msg1)
        
        elif self.current_destination == "B":
            self.is_right_side_B = True

            # 赤色を判定する条件を追加
            red_pixels = np.where(
                (left_region[:, :, 2] >= red_threshold) &  # 赤チャネルが閾値を超えている
                (left_region[:, :, 1] <= green_threshold) &  # 緑チャネルが閾値を下回っている
                (left_region[:, :, 0] <= blue_threshold)    # 青チャネルが閾値を下回っている
            )
            if red_pixels[0].size == 0:
                self.nb += 1
            
            if self.nb > 30:
                if red_pixels[0].size > 0:
                    # self.get_logger().info(f'Red pixels found at {list(zip(red_lower_pixels[0], red_lower_pixels[1]))}')
                    self.stop_move()
                    self.publish_goal_reached(self.current_destination)
                    self.is_right_side_B = False

            msg1 = Bool()
            msg1.data = self.is_right_side_B
            self.right_side_B_publisher.publish(msg1)

        
        elif self.current_destination == "A":
            self.is_right_side_A = True

            # 赤色を判定する条件を追加
            red_pixels = np.where(
                (left_region[:, :, 2] >= red_threshold) &  # 赤チャネルが閾値を超えている
                (left_region[:, :, 1] <= green_threshold) &  # 緑チャネルが閾値を下回っている
                (left_region[:, :, 0] <= blue_threshold)    # 青チャネルが閾値を下回っている
            )
            if red_pixels[0].size == 0:
                self.na += 1
            
            if self.na > 30:
                if red_pixels[0].size > 0:
                    # self.get_logger().info(f'Red pixels found at {list(zip(red_lower_pixels[0], red_lower_pixels[1]))}')
                    self.stop_move()
                    self.publish_goal_reached(self.current_destination)
                    self.is_right_side_A = False
            
            msg = Bool()
            msg.data = self.is_right_side_A
            self.right_side_A_publisher.publish(msg)
        
        elif self.current_destination == "F":
            self.is_right_side_F = True

            # 赤色を判定する条件を追加
            red_pixels = np.where(
                (left_region[:, :, 2] >= red_threshold) &  # 赤チャネルが閾値を超えている
                (left_region[:, :, 1] <= green_threshold) &  # 緑チャネルが閾値を下回っている
                (left_region[:, :, 0] <= blue_threshold)    # 青チャネルが閾値を下回っている
            )
            if red_pixels[0].size == 0:
                self.nf += 1
            
            if self.nf > 30:
                if red_pixels[0].size > 0:
                    # self.get_logger().info(f'Red pixels found at {list(zip(red_lower_pixels[0], red_lower_pixels[1]))}')
                    self.stop_move()
                    self.publish_goal_reached(self.current_destination)
                    self.is_right_side_F = False
                    self.nf = 0
            
            msg = Bool()
            msg.data = self.is_right_side_F
            self.right_side_F_publisher.publish(msg)

        elif self.current_destination == "E":
            # 赤色を判定する条件を追加
            red_pixels = np.where(
                (left_region[:, :, 2] >= red_threshold) &  # 赤チャネルが閾値を超えている
                (left_region[:, :, 1] <= green_threshold) &  # 緑チャネルが閾値を下回っている
                (left_region[:, :, 0] <= blue_threshold)    # 青チャネルが閾値を下回っている
            )
            if red_pixels[0].size == 0:
                self.ne += 1
        
            if self.ne > 30:
                if red_pixels[0].size > 0:
                    # self.get_logger().info(f'Red pixels found at {list(zip(red_lower_pixels[0], red_lower_pixels[1]))}')
                    self.stop_move()
                    self.publish_goal_reached(self.current_destination)
                    self.ne = 0


    def stop_move(self):
        # lidar node を停止するためのコマンドを送信
        self.move_msg = Bool()
        self.move_msg.data = False
        self.move_publisher.publish(self.move_msg)
        self.get_logger().info('Stop lidar node')

        # 速度0を出す
        n = 0
        while n < 1000:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher.publish(twist)
            n += 1

    
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