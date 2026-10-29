
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile,QoSHistoryPolicy,QoSReliabilityPolicy

from sensor_msgs.msg import Image
from robo_msgs.msg import Waypoint
from robo_msgs.msg import TrafficLight

import cv2
from cv_bridge import CvBridge, CvBridgeError

from ultralytics import YOLO

import datetime
from collections import defaultdict
import numpy as np


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
        self.type = self.WPT_NORMAL
        #self.type = self.WPT_TRAFFICLIGHT

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
        
        self.track_history = defaultdict(lambda: [])

        #self.img_pub = self.create_publisher(Image, "/image_detection", 1) #パブリッシャーの定義
        #self.publisher_ = self.create_publisher(Image, 'yolo/image', 1)
        self.publisher_YOLO = self.create_publisher(Image, 'yolo/image', 1)
        self.publisher_TrafficLight = self.create_publisher(TrafficLight, 'trafficlight', 1)

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().warn(str(e))
            return
    
        #results = self.model(frame)[0]
        
        # YOLO推論
        #if self.type == self.WPT_TRAFFICLIGHT:
        #    self.model = self.model2
        #    results = self.model(frame)[0]
        #else:
        #    self.model = self.model1
        #    results = self.model(frame)[0]

        #信号検知
        if self.type == self.WPT_TRAFFICLIGHT:

            results = self.model(frame)[0]
            
            gx_red , gy_red = 9999, 9999
            gx_green , gy_green = 9999,9999

            x1_red, y1_red, x2_red, y2_red = 0, 0, 0, 0
            x1_green, y1_green, x2_green, y2_green = 0, 0, 0, 0

            trafficlight = TrafficLight()
            trafficlight.red = 0
            trafficlight.green = 0

            # 検出結果を描画     
            for box in results.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = float(box.conf[0])
                cls = int(box.cls[0])
                #label = f'{self.model.names[cls]} {conf:.2f}'
                label = f'{self.model.names[cls]}'

                cv2.rectangle(frame, (x1, y1), (x2, y2), (125, 125, 125), 2)
                cv2.putText(frame, label, (x1, y1 - 10),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (125, 125, 125), 2)

                if label == 'Trafficlight_red':
                    tmp_gx_red = (x1 + x2)/2        
                    tmp_gy_red = (y1 + y2)/2        
                    
                    if gy_red > tmp_gy_red:
                        gx_red = tmp_gx_red
                        gy_red = tmp_gy_red
                        x1_red,y1_red,x2_red,y2_red = x1, y1, x2, y2

                elif label == 'Trafficlight_blue':
                    tmp_gx_green = (x1 + x2)/2        
                    tmp_gy_green = (y1 + y2)/2        

                    if gy_green > tmp_gy_green:
                        gx_green = tmp_gx_green
                        gy_green = tmp_gy_green
                        x1_green,y1_green,x2_green,y2_green = x1, y1, x2, y2

            #print(f"{gx_red=}", f"{gx_green=}")
            
            #信号検知結果の評価
            trafficlight.red =0
            trafficlight.green =0
            if gy_green < 9999 and gy_red < 9999: #両方検知された場合            
                aspect_ratio_red = (x2_red - x1_red) / (y2_red - y1_red + 1e-5)   
                aspect_ratio_green = (x2_green - x1_green) / (y2_green - y1_green + 1e-5)
                
                #Green は常にRedの下にあるはず.#そうでなければエラー扱い
                if aspect_ratio_red < 0.7 and aspect_ratio_green < 0.7:
                    if gy_green > gy_red:  
                        trafficlight.red +=1
                #Green が形状不適合
                elif aspect_ratio_red < 0.7 and aspect_ratio_green >= 0.7:
                        trafficlight.red +=1
                #Red が形状不適合
                elif aspect_ratio_red >= 0.7 and aspect_ratio_green < 0.7:
                        trafficlight.green +=1

            elif gy_green == 9999 and gy_red < 9999: #Redが検知された場合 
                aspect_ratio_red = (x2_red - x1_red) / (y2_red - y1_red + 1e-5)   
                if aspect_ratio_red < 0.7: #形状適合
                    trafficlight.red +=1

            elif gy_green < 9999 and gy_red == 9999: #Greenが検知された場合            
                aspect_ratio_green = (x2_green - x1_green) / (y2_green - y1_green + 1e-5)
                if aspect_ratio_green < 0.7: #形状適合
                    trafficlight.green +=1

            #結果の表示
            if trafficlight.red > 0:
                print("Red light detected, stopping the vehicle.")   
                print(f"{aspect_ratio_red=}")
                cv2.rectangle(frame, (x1_red, y1_red), (x2_red, y2_red), (0, 0, 255), 2)
                cv2.putText(frame, 'Red', (x1_red, y1_red - 10),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            
            if trafficlight.green > 0:
                print("Green light detected, proceeding.")
                print(f"{aspect_ratio_green=}")
                cv2.rectangle(frame, (x1_green, y1_green), (x2_green, y2_green), (0, 255, 0), 2)
                cv2.putText(frame, 'Green', (x1_green, y1_green - 10),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            self.publisher_TrafficLight.publish(trafficlight)

            cv2.imshow("YOLO", frame)
            cv2.waitKey(1)

        else: #信号以外

            results = self.model(frame)[0]
            # YOLOv8でトラッキング
            #results = self.model.track(frame, persist=True)[0]
            #result = self.model.track(frame, persist=True)[0]
        '''
        # Get the boxes and track IDs
        if result.boxes and result.boxes.is_track:
            #boxes = result.boxes.xywh.cpu()
            boxes = result.boxes
            track_ids = result.boxes.id.int().cpu().tolist()

            # Visualize the result on the frame
            frame = result.plot()

            # 検出結果を描画
            for box, track_id in zip(boxes, track_ids):
                x, y, w, h = map(int, box.xywh[0])
                conf = float(box.conf[0])
                cls = int(box.cls[0])
                #label = f'{self.model.names[cls]} {conf:.2f}'
                track = self.track_history[track_id]
                track.append((float(x), float(y)))  # x, y center point
                #if len(track) > 30:  # retain 30 tracks for 30 frames
                #    track.pop(0)

                if len(track) > 10:  # retain 10 tracks for 10 frames
                    track.pop(0)

                points = np.hstack(track).astype(np.int32).reshape((-1, 1, 2))
                if cls == 0 or cls == 1 or cls == 15 or cls == 16: #person or bicycle or cat or dog    
                    cv2.polylines(frame, [points], isClosed=False, color=(255, 0, 0), thickness=5)
                else:
                    cv2.polylines(frame, [points], isClosed=False, color=(230, 230, 230), thickness=5)


            # Plot the tracks
            #for box, track_id in zip(boxes, track_ids):
            #    x, y, w, h = box
            #    track = self.track_history[track_id]
            #    track.append((float(x), float(y)))  # x, y center point
            #    if len(track) > 30:  # retain 30 tracks for 30 frames
            #        track.pop(0)

                # Draw the tracking lines
            #    points = np.hstack(track).astype(np.int32).reshape((-1, 1, 2))
                #cls = box.cls[0]
                #if cls == 0 or cls == 1 or cls == 15 or cls == 16: #person or bicycle or cat or dog    
                #    cv2.polylines(frame, [points], isClosed=False, color=(0, 0, 255), thickness=5)
                #else:
                #    cv2.polylines(frame, [points], isClosed=False, color=(230, 230, 230), thickness=5)

            #    cv2.polylines(frame, [points], isClosed=False, color=(230, 230, 230), thickness=5)

            '''
        
            '''
            # 検出結果を描画  bgr   
            for box in results.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = float(box.conf[0])
                cls = int(box.cls[0])
                label = f'{self.model.names[cls]} {conf:.2f}'
                if cls ==0 or cls ==1 or cls == 15 or cls == 16: #person or bicycle or cat or dog    
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    cv2.putText(frame, label, (x1, y1 - 10),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                elif cls ==2 or cls ==3 or cls ==5 or cls ==7: #car, motorcycle, bus or truck
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
                    cv2.putText(frame, label, (x1, y1 - 10),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                else:
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (125, 125, 125), 2)
                    cv2.putText(frame, label, (x1, y1 - 10),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (125, 125, 125), 2)
            
                    # 検出結果を描画
            for box in results.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = float(box.conf[0])
                cls = int(box.cls[0])
                label = f'{self.model.names[cls]} {conf:.2f}'
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            '''
        # OpenCV画像をROS Imageに変換してPublish
        #ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        #self.publisher_YOLO.publish(ros_image)
        #self.get_logger().info('YOLO画像をパブリッシュしました')
        cv2.imshow("YOLO", frame)
        cv2.waitKey(1)

    def waypoint_callback(self, msg):
        try:
            self.type = msg.type
            print(f"Received waypoint: type={self.type}")

            # YOLO model switch
            if self.type == self.WPT_TRAFFICLIGHT:
                self.model = self.model2
            else:
                self.model = self.model1

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
