
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile,QoSHistoryPolicy,QoSReliabilityPolicy

from sensor_msgs.msg import Image
from robo_msgs.msg import Waypoint
from robo_msgs.msg import TrafficLight
from robo_msgs.msg import Scene

import cv2
from cv_bridge import CvBridge, CvBridgeError

from ultralytics import YOLO

import datetime
from collections import defaultdict
import numpy as np
import math


class ObjectSegmentation(Node):
    
    def __init__(self, **args):
        super().__init__('object_segmentation')

        self.WPT_NORMAL = 0
        self.WPT_TRAFFICLIGHT = 4

        # カメラ or 動画ファイル（ここではカメラ）
        #self.cap = cv2.VideoCapture(0)

        #self.model = YOLO("yolov8m-seg.pt")
        self.model1 = YOLO("/home/june/ros2_ws/yolov8n.pt")
        #self.model2 = YOLO("/home/june/ros2_ws/trafficlight.pt")
        #self.model2 = YOLO("/home/june/ros2_ws/trafficlight_best2.pt")
        self.model2 = YOLO("/home/june/ros2_ws/trafficlight_best3.pt")

        # Normal ---
        self.model = self.model1
        self.mode = 'Normal'
        # test ---
        #self.mode = 'TrafficLight'
        #self.model = self.model2
        #---

        self.bridge = CvBridge()
        
        self.type = self.WPT_NORMAL
        self.last_type = self.WPT_NORMAL
        
        self.WPT_num = 0
        self.WPT_last_num = 0
     
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
        self.rect_history = defaultdict(lambda: [])

        #self.img_pub = self.create_publisher(Image, "/image_detection", 1) #パブリッシャーの定義
        #self.publisher_ = self.create_publisher(Image, 'yolo/image', 1)
        self.publisher_YOLO = self.create_publisher(Image, 'yolo/image', 1)
        self.publisher_TrafficLight = self.create_publisher(TrafficLight, 'trafficlight', 1)
        self.publisher_Scene = self.create_publisher(Scene, 'scene', 1)

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
        if self.mode == 'TrafficLight':

            results = self.model(frame)[0]
            
            gx_red , gy_red = 9999, 9999
            gx_green , gy_green = 9999,9999

            x1_red, y1_red, x2_red, y2_red = 0, 0, 0, 0
            x1_green, y1_green, x2_green, y2_green = 0, 0, 0, 0

            trafficlight = TrafficLight()
            trafficlight.red = 0
            trafficlight.green = 0

            #ガイドラインの描画
            cv2.line(frame, (0, 240), (640, 240), (255, 255, 0), 1)
            
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

            threshold_aspect_ratio = 0.6
            #threshold_width = 18
            #threshold_height = 36
            threshold_width = 20
            threshold_height = 36
            
            #if gy_green < 9999 and gy_red < 9999: #両方検知された場合            
            if gy_green < 360 and gy_red < 360 : #両方検知された場合            
                
                aspect_ratio_red = (x2_red - x1_red) / (y2_red - y1_red + 1e-5)   
                aspect_ratio_green = (x2_green - x1_green) / (y2_green - y1_green + 1e-5)
                
                width_red = x2_red - x1_red
                height_red = y2_red - y1_red

                width_green = x2_green - x1_green
                height_green = y2_green - y1_green

                #Green は常にRedの下にあるはず.#そうでなければエラー扱い
                #if aspect_ratio_red < threshold_aspect_ratio and aspect_ratio_green < threshold_aspect_ratio:
                #    if gy_green > gy_red and height_red < threshold_height and width_green < threshold_width:  
                #        trafficlight.red +=1
                #Green が形状不適合
                #elif aspect_ratio_red < threshold_aspect_ratio and aspect_ratio_green >= threshold_aspect_ratio:
                
                #Green が形状不適合
                if aspect_ratio_red < threshold_aspect_ratio and aspect_ratio_green >= threshold_aspect_ratio:
                    if height_red < threshold_height and width_red < threshold_width:
                        
                        trafficlight.red +=1
                
                #Red が形状不適合
                elif aspect_ratio_red >= threshold_aspect_ratio and aspect_ratio_green < threshold_aspect_ratio:
                    if height_green < threshold_height and width_green < threshold_width:
                        trafficlight.green +=1
                
                #print(f"{aspect_ratio_red=}",f"{aspect_ratio_green=}")      

            #elif gy_green == 9999 and gy_red < 9999: #Redが検知された場合 
            elif gy_green == 9999 and gy_red < 360: #Redが検知された場合 

                width_red = x2_red - x1_red
                height_red = y2_red - y1_red

                aspect_ratio_red = (x2_red - x1_red) / (y2_red - y1_red + 1e-5)   
                if aspect_ratio_red < threshold_aspect_ratio: #形状適合
                    if height_red < threshold_height and width_red < threshold_width:
                        trafficlight.red +=1

            #elif gy_green < 9999 and gy_red == 9999: #Greenが検知された場合            
            elif gy_green < 360 and gy_red == 9999: #Greenが検知された場合            

                width_green = x2_green - x1_green
                height_green = y2_green - y1_green

                aspect_ratio_green = (x2_green - x1_green) / (y2_green - y1_green + 1e-5)
                if aspect_ratio_green < threshold_aspect_ratio: #形状適合
                    if height_green < threshold_height and width_green < threshold_width:
                        trafficlight.green +=1

            #結果の表示
            if trafficlight.red > 0:
                print("Red light detected, stopping the vehicle.")   
                
                #width_red = x2_red - x1_red
                #height_red = y2_red - y1_red
                
                print(f"{gx_red=}",f"{gy_red=}")
                print(f"{width_red=}")
                print(f"{height_red=}")
                print(f"{aspect_ratio_red=}")
                cv2.rectangle(frame, (x1_red, y1_red), (x2_red, y2_red), (0, 0, 255), 2)
                cv2.putText(frame, 'Red', (x1_red, y1_red - 10),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                
                text = f'Trafficlight:Red'
                cv2.putText(frame, text, (10,30), cv2.FONT_HERSHEY_SIMPLEX, 1, color=(0, 0, 255), thickness=2)

            if trafficlight.green > 0:
                print("Green light detected, proceeding.")
                
                #width_green = x2_green - x1_green
                #height_green = y2_green - y1_green

                print(f"{gx_green=}",f"{gy_green=}")
                print(f"{width_green=}")
                print(f"{height_green=}")
                print(f"{aspect_ratio_green=}")
                cv2.rectangle(frame, (x1_green, y1_green), (x2_green, y2_green), (0, 255, 0), 2)
                cv2.putText(frame, 'Green', (x1_green, y1_green - 10),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                text = f'Trafficlight:Green'
                cv2.putText(frame, text, (10,30), cv2.FONT_HERSHEY_SIMPLEX, 1, color=(0, 255, 0), thickness=2)

            self.publisher_TrafficLight.publish(trafficlight)
            
            cv2.imshow("YOLO", frame)
            cv2.waitKey(1)

        else: #信号以外 YOLOv8でトラッキング
            result = self.model.track(frame, persist=True)[0]

            scene = Scene()
            scene.scene = 0
            tmp_scene = 99

            #ガイドラインの描画
            cv2.line(frame, (210, 0), (210, 480), (255, 255, 0), 1)
            cv2.line(frame, (430, 0), (430, 480), (255, 255, 0), 1)

            # Get the boxes and track IDs
            if result.boxes and result.boxes.is_track:

                boxes = result.boxes
                track_ids = result.boxes.id.int().cpu().tolist()

                # Visualize the result on the frame
                frame = result.plot()

                #scene = Scene()
                #scene.scene = 0
                #tmp_scene = 99

                # 検出結果を描画
                for box, track_id in zip(boxes, track_ids):
                    x, y, w, h = map(int, box.xywh[0])
                    conf = float(box.conf[0])
                    cls = int(box.cls[0])

                    #label = f'{self.model.names[cls]} {conf:.2f}'
                    track = self.track_history[track_id]
                    track.append((float(x), float(y)))  # x, y center point
                    
                    rect = self.rect_history[track_id]
                    rect.append((x, y, w, h))  # x, y, w
                   
                    if len(track) > 10:  # retain 10 tracks for 10 frames
                        track.pop(0)

                    if len(rect) > 30:  # retain 10 tracks for 5 frames
                        rect.pop(0)

                    #points = np.hstack(track).astype(np.int32).reshape((-1, 1, 2))
                    points = np.array(track, dtype=np.int32).reshape((-1, 1, 2))
                    xywh = np.array(rect, dtype=np.float32).reshape((-1, 4, 1))

                    #print(f'track_id={track_id}',points)
                    #print(f'track_id={track_id}',len(xywh),xywh,xywh[0,3])
                      
                    print(f'head={points[0]},x={points[0, 0, 0]},y={points[0, 0, 1]}')
                    print(f'tail={points[-1]},x={points[-1, 0, 0]},y={points[-1, 0, 1]}')

                    x0=points[0, 0, 0]
                    y0=points[0, 0, 1]

                    x1=points[-1, 0, 0]
                    y1=points[-1, 0, 1]
                                           
                    #print(f'points={points}')
                      
                    expantion_rate = 1.0
                    if len(xywh) >= 3:
                        n = len(xywh)-1
                        expantion_rate = (xywh[n,3])/(xywh[0,3])
                        print(f'expantion_rate={expantion_rate}')
                    
                    text = f'{expantion_rate},{w/640.0:.2g},{h/480.0:.2g}'
                    print(text)
                    hh = int(h/2)
                    ww = int(w/2)

                    if cls == 0 or cls == 1 or cls == 15 or cls == 16: #person or bicycle or cat or dog    
                        if h/480.0 > 0.75 :#0.8
                            print("Pedestrian_Very_Close!")
                            cv2.rectangle(frame, ((x-ww), (y-hh)), ((x+ww), (y+hh)), (0, 0, 255), 4,lineType=cv2.LINE_AA, shift=0)
                            cv2.polylines(frame, [points], isClosed=False, color=(0, 0, 255), thickness=4)
                            #cv2.putText(frame, text, (x,y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color=(0, 0, 255), thickness=4)
                            
                            if tmp_scene >= 1: 
                                tmp_scene = 1

                        elif (expantion_rate > 1.1 and h/480.0 > 0.35): #近づいてきている
                            if ((x1 < 210 and x0 > x1 ) or (x1 > 430 and x0 < x1 )):
                                print("Pedestrian_Passing_Through!")
                                cv2.rectangle(frame, ((x-ww), (y-hh)), ((x+ww), (y+hh)), (255, 0, 0), 2,lineType=cv2.LINE_AA, shift=0)
                                cv2.polylines(frame, [points], isClosed=False, color=(255, 0, 0), thickness=2)
                                #cv2.putText(frame, text, (x,y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color=(255, 0, 0), thickness=2)                           
                                
                                if tmp_scene >= 3:
                                    tmp_scene = 3

                            else:
                                print("Peestrian_Oncomming!")
                                cv2.rectangle(frame, ((x-ww), (y-hh)), ((x+ww), (y+hh)), (0, 0, 255), 2,lineType=cv2.LINE_AA, shift=0)
                                cv2.polylines(frame, [points], isClosed=False, color=(0, 0, 255), thickness=2)
                                #cv2.putText(frame, text, (x,y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color=(0, 0, 255), thickness=2)
                                
                                if tmp_scene >= 2:
                                    tmp_scene = 2

                        else: 
                            print("Peestrian_All_Green!")
                            cv2.rectangle(frame, ((x-ww), (y-hh)), ((x+ww),(y+hh)), (0, 255, 0), 2,lineType=cv2.LINE_AA, shift=0)
                            cv2.polylines(frame, [points], isClosed=False, color=(0, 255, 0), thickness=2)
                            #cv2.putText(frame, text, (x,y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color=(0, 255, 0), thickness=2)
                            
                            if tmp_scene >= 4:
                                tmp_scene = 4
                    else:
                        cv2.polylines(frame, [points], isClosed=False, color=(230, 230, 230), thickness=5)
                        #cv2.putText(frame, text, (x,y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color=(230, 230, 230), thickness=2)

            scene.scene = tmp_scene    
            self.publisher_Scene.publish(scene)  
        
            text = f'Scene:{scene.scene}'
            cv2.putText(frame, text, (10,30), cv2.FONT_HERSHEY_SIMPLEX, 1, color=(255, 255, 0), thickness=2)

        # OpenCV画像をROS Imageに変換してPublish
        #ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        #self.publisher_YOLO.publish(ros_image)
        #self.get_logger().info('YOLO画像をパブリッシュしました')
        cv2.imshow("YOLO", frame)
        cv2.waitKey(1)

    def waypoint_callback(self, msg):
        try:
            #self.last_type = self.type
            self.WPT_last_num = self.WPT_num
            self.WPT_num = msg.num
            if self.WPT_num > self.WPT_last_num:
                self.last_type = self.type
                self.type = msg.type
                
            # YOLO model switch
            if self.type == self.WPT_TRAFFICLIGHT or  self.last_type  == self.WPT_TRAFFICLIGHT:
                self.model = self.model2
                self.mode = 'TrafficLight'
            else:
                self.model = self.model1
                self.mode = 'Normal'
                

            print(f"Received waypoint: type={self.type}, last_type={self.last_type}")
 
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
