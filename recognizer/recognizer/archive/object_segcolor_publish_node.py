import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np
from ultralytics import YOLO

class YoloSegColorPublisher(Node):
    def __init__(self):
        super().__init__('yolo_seg_color_publisher')
        self.publisher_ = self.create_publisher(Image, 'yolo/segmentation/label_image', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz
        self.bridge = CvBridge()

        # YOLOv8 セグメンテーションモデル（必要に応じて変えてください）
        self.model = YOLO("yolov8n-seg.pt")
        #self.model = YOLO("yolov8n.pt")

        # カメラ or 動画ファイル（ここではカメラ）
        self.cap = cv2.VideoCapture(0)

        # クラスごとの色マップ
        self.class_colors = self.generate_class_colors()

    def generate_class_colors(self):
        # クラス数に応じた色を生成（再現性のためseed固定）
        np.random.seed(42)
        class_names = self.model.names
        return {i: np.random.randint(0, 255, (3,), dtype=np.uint8) for i in class_names}

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("カメラフレームの取得に失敗しました")
            return

        # YOLO推論
        results = self.model(frame)[0]

        # ラベルカラー表示用マスク（黒背景）
        label_mask = np.zeros_like(frame, dtype=np.uint8)

        if results.masks is not None:
            masks = results.masks.data.cpu().numpy()   # (N, H, W)
            classes = results.boxes.cls.cpu().numpy().astype(int)  # (N,)

            for mask, cls in zip(masks, classes):
                color = self.class_colors.get(cls, (255, 255, 255))  # 未定義なら白
                binary_mask = (mask > 0.5).astype(np.uint8)

                # 輪郭抽出して塗りつぶし
                contours, _ = cv2.findContours(binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(label_mask, contours, -1, color.tolist(), thickness=cv2.FILLED)

        # ROS Imageとしてパブリッシュ
        ros_image = self.bridge.cv2_to_imgmsg(label_mask, encoding='bgr8')
        self.publisher_.publish(ros_image)
        self.get_logger().info("ラベルカラー画像をパブリッシュしました")

def main(args=None):
    rclpy.init(args=args)
    node = YoloSegColorPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
