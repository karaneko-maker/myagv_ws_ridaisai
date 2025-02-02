import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from collections import deque
import threading
class AGVCommander(Node):
    def __init__(self):
        super().__init__('agv_commander')
        self.publisher_command = self.create_publisher(String, 'agv_command', 10)
        self.positions = ['F', 'E', 'D', 'C', 'B', 'A']
        self.agv_positions = {'agv1': 'A', 'agv2': 'B'}
        self.agv_moving = False
        self.order_queue = deque()
        self.processing_order = False
        self.create_subscription(Int32, 'ramen_order_count', self.order_callback, 10)
        self.create_subscription(String, 'agv_arrival', self.agv_arrival_callback, 10)
        self.timer = self.create_timer(1.0, self.check_and_move_agvs)
        self.input_thread = threading.Thread(target=self.listen_for_enter)
        self.input_thread.daemon = True
        self.input_thread.start()
    def listen_for_enter(self):
        while True:
            input("AGVの次の動作をするにはEnterキーを押して")
            self.process_next_order()
    def order_callback(self, msg):
        self.get_logger().info(f'受信した注文数メッセージ: {msg.data}')
        self.order_queue.append(msg.data)
        self.get_logger().info(f'現在の注文キュー: {list(self.order_queue)}')
    def agv_arrival_callback(self, msg):
        agv_id, position = msg.data.split(' ')
        self.agv_positions[agv_id] = position
        self.agv_moving = False  # AGVが到着したら移動中フラグをリセット
        self.get_logger().info(f'到着確認を受信: {agv_id} が {position} に到着')
    def process_next_order(self):
        if not self.processing_order and self.order_queue:
            self.processing_order = True
            self.current_order_count = self.order_queue.popleft()
            self.get_logger().info(f'注文を処理中: {self.current_order_count}')
            self.move_agv_to_next_position()
            self.processing_order = False
    def check_and_move_agvs(self):
        if not self.agv_moving:
            for agv_id in self.agv_positions:
                current_pos = self.agv_positions[agv_id]
                next_pos_index = (self.positions.index(current_pos) + 1) % len(self.positions)
                next_pos = self.positions[next_pos_index]
                if next_pos not in self.agv_positions.values():
                    self.move_agv(agv_id, next_pos)
    def move_agv_to_next_position(self):
        for agv_id in self.agv_positions:
            current_pos = self.agv_positions[agv_id]
            next_pos_index = (self.positions.index(current_pos) + 1) % len(self.positions)
            next_pos = self.positions[next_pos_index]
            if next_pos not in self.agv_positions.values():
                self.move_agv(agv_id, next_pos)
    def move_agv(self, agv_id, next_pos):
        if self.agv_positions[agv_id] != next_pos:
            command = f"{agv_id} move to {next_pos}"
            msg = String()
            msg.data = command
            self.publisher_command.publish(msg)
            self.get_logger().info(f'コマンドを送信: {command}')
            self.agv_moving = True
def main(args=None):
    rclpy.init(args=args)
    agv_commander = AGVCommander()
    rclpy.spin(agv_commander)
    agv_commander.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()