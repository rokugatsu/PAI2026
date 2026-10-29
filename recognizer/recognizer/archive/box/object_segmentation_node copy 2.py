
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile,QoSHistoryPolicy,QoSReliabilityPolicy

from sensor_msgs.msg import Image
from robo_msgs.msg import Waypoint

import cv2
from cv_bridge import CvBridge, CvBridgeError

from ultralytics import YOLO



class ObjectSegmentation(Node):
    
    def __init__(self, **args):
        super().__init__('object_segmentation')

        self.WPT_NORMAL = 0
        self.WPT_TRAFFICLIGHT = 4

        # カメラ or 動画ファイル（ここではカメラ）
        #self.cap = cv2.VideoCapture(0)

        #self.model = YOLO("yolov8m-seg.pt")
        self.model1 = YOLO("/home/june/ros2_ws/yolov8n.pt")
        self.model2 = YOLO("/home/june/ros2_ws/trafficlight.pt")

        self.model = self.model1

        self.bridge = CvBridge()
        #self.type = self.WPT_NORMAL
        self.type = self.WPT_TRAFFICLIGHT

        #self.subscription = self.create_subscription(
        #    Image,
        #    '/image_raw',
        #    self.image_callback,
        #    qos_profile_sensor_data)
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        self.subscription1 = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            qos_profile)
        
        self.subscription2 = self.create_subscription(
            Waypoint,
            '/waypoint',
            self.waypoint_callback,
            qos_profile)
        
        #self.img_pub = self.create_publisher(Image, "/image_detection", 1) #パブリッシャーの定義
        #self.publisher_ = self.create_publisher(Image, 'yolo/image', 1)
        self.publisher_ = self.create_publisher(Image, 'yolo/image', 1)

        #self.timer = self.create_timer(0.01, self.timer_callback)  # 10Hz

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("カメラフレームの取得に失敗しました")
            return

        # YOLO推論
        if self.type == self.WPT_TRAFFICLIGHT:
            self.model = self.model2
            results = self.model(frame)[0]
            gx_red = 9999 
            gy_red = 9999
            gx_green = 9999 
            gy_green = 9999
            x1_red, y1_red, x2_red, y2_red = 0, 0, 0, 0
            x1_green, y1_green, x2_green, y2_green = 0, 0, 0, 0
            
        else:
            self.model = self.model1
            results = self.model(frame)[0]

        # 検出結果を描画     
        for box in results.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf = float(box.conf[0])
            cls = int(box.cls[0])
            label = f'{self.model.names[cls]} {conf:.2f}'
            
            if self.type == self.WPT_TRAFFICLIGHT:
    
                if label == 'Red':
                    tmp_gx_red = (x1 + x2)/2        
                    tmp_gy_red = (y1 + y2)/2        
                    
                    if gx_red > tmp_gx_red:
                        gx_red = tmp_gx_red
                        gy_red = tmp_gy_red
                        x1_red=x1, y1_red=y1, x2_red=x2, y2_red=y2

                elif label == 'Green':
                    tmp_gx_green = (x1 + x2)/2        
                    tmp_gy_green = (y1 + y2)/2        

                    if gx_green > tmp_gx_green:
                        gx_green = tmp_gx_green
                        gy_green = tmp_gy_green
                        x1_green=x1, y1_green=y1, x2_green=x2, y2_green=y2

                if gx_red != 9999:
                    print("Red light detected, stopping the vehicle.")   
                    cv2.rectangle(frame, (x1_red, y1_red), (x2_red, y2_red), (255, 0, 0), 2)
                    cv2.putText(frame, label, (x1_red, y1_red - 10),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            
                elif gx_green != 9999:
                    print("Green light detected, proceeding.")   
                    cv2.rectangle(frame, (x1_green, y1_green), (x2_green, y2_green), (0, 255, 0), 2)
                    cv2.putText(frame, label, (x1_green, y1_green - 10),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)


        # OpenCV画像をROS Imageに変換してPublish
        ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        #self.publisher_.publish(ros_image)
        #self.get_logger().info('YOLO画像をパブリッシュしました')
        cv2.imshow("YOLO", frame)
        cv2.waitKey(1)

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().warn(str(e))
            return
    
        #results = self.model(frame)[0]
        
        # YOLO推論
        if self.type == self.WPT_TRAFFICLIGHT:
            self.model = self.model2
            results = self.model(frame)[0]
        else:
            self.model = self.model1
            results = self.model(frame)[0]

        # 検出結果を描画
        for box in results.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf = float(box.conf[0])
            cls = int(box.cls[0])
            label = f'{self.model.names[cls]} {conf:.2f}'
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        # OpenCV画像をROS Imageに変換してPublish
        ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(ros_image)
        #self.get_logger().info('YOLO画像をパブリッシュしました')
        cv2.imshow("YOLO", frame)
        cv2.waitKey(1)

    def waypoint_callback(self, msg):
        try:
            self.type = msg.type
            print(f"Received waypoint: type={self.type}")    
        except CvBridgeError as e:
            self.get_logger().warn(str(e))
            return
    


def main():
    rclpy.init()
    object_segmentation = ObjectSegmentation()
    try:
        rclpy.spin(object_segmentation)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
