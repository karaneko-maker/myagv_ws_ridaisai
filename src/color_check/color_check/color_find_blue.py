import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import matplotlib.pyplot as plt

class ImageMonitorNode(Node):
    def __init__(self):
        super().__init__('color_find_blue')
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',  # カメラトピック名を指定
            self.image_callback,
            10)
        self.bridge = CvBridge()
        self.get_logger().info('Image Monitor Node has started.')

    def image_callback(self, msg):
        # ROS2の画像メッセージをOpenCV形式に変換
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 画像の左端を切り出す（例えば、画像の幅の半分）
        height, width, _ = frame.shape
        left_region = frame[:, :width // 1]  # 左端の1/4幅を切り出し

        # 赤色の閾値を設定
        red_threshold = 80  # 赤の閾値（強さ）80
        green_threshold = 50  # 緑の閾値（低ければ低いほど赤と認識しやすい）50
        blue_threshold = 50  # 青の閾値（低ければ低いほど赤と認識しやすい）50

        # 赤色を判定する条件を追加
        red_pixels = np.where(
            (left_region[:, :, 0] >= red_threshold) &  # 赤チャネルが閾値を超えている
            (left_region[:, :, 1] <= green_threshold) &  # 緑チャネルが閾値を下回っている
            (left_region[:, :, 2] <= blue_threshold)    # 青チャネルが閾値を下回っている
        )

        # 赤色のピクセルが見つかった場合、その位置に四角を描画
        if red_pixels[0].size > 0:
            self.get_logger().info(f'Red pixels found at {list(zip(red_pixels[0], red_pixels[1]))}')

            # 四角形を描く
            for y, x in zip(red_pixels[0], red_pixels[1]):
                # 赤色ピクセルの周囲に四角を描く
                cv2.rectangle(frame, (x-10, y-10), (x+10, y+10), (0, 255, 0), 2)  # 緑色の四角

            # 画像を表示して赤色の検出範囲を視覚的に確認
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            plt.imshow(frame_rgb)
            plt.axis('off')  # 軸を非表示
            plt.show()  # 画像を表示

            self.get_logger().info('Red pixels detected and rectangle drawn.')

            # ノードを停止
            rclpy.shutdown()  # ノードを停止

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