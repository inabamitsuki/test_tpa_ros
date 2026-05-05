#!/usr/bin/env python3
# encoding: utf-8

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String
import mediapipe as mp
from yahboomcar_msgs.msg import PointArray
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
import cv2 as cv
import numpy as np
import time
import apriltag # สำหรับ AprilTag

class PoseDetector:
    def __init__(self, mode=False, smooth=True, detectionCon=0.5, trackCon=0.5):
        self.mpPose = mp.solutions.pose
        self.mpDraw = mp.solutions.drawing_utils
        self.pose = self.mpPose.Pose(
            static_image_mode=mode,
            smooth_landmarks=smooth,
            min_detection_confidence=detectionCon,
            min_tracking_confidence=trackCon )
        self.lmDrawSpec = self.mpDraw.DrawingSpec(color=(0, 0, 255), thickness=-1, circle_radius=6)
        self.drawSpec = self.mpDraw.DrawingSpec(color=(0, 255, 0), thickness=2, circle_radius=2)

    def process_pose(self, frame, draw=True):
        pointArray = PointArray()
        img_RGB = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        results = self.pose.process(img_RGB)
        
        if results.pose_landmarks:
            if draw:
                self.mpDraw.draw_landmarks(frame, results.pose_landmarks, self.mpPose.POSE_CONNECTIONS, self.lmDrawSpec, self.drawSpec)
            for lm in results.pose_landmarks.landmark:
                point = Point()
                point.x, point.y, point.z = lm.x, lm.y, lm.z
                pointArray.points.append(point)
        return frame, pointArray, (results.pose_landmarks is not None)

class MY_Picture(Node):
    def __init__(self, name):
        super().__init__(name)
        self.bridge = CvBridge()
        self.sub_img = self.create_subscription(
            CompressedImage, '/espRos/esp32camera', self.handleTopic, 1)
        
        self.pub_point = self.create_publisher(PointArray, '/mediapipe/points', 1000)
        
        # ตั้งค่า AprilTag (ตระกูล tag36h11)
        options = apriltag.DetectorOptions(families="tag36h11")
        self.at_detector = apriltag.Detector(options)
        self.pub_at_id = self.create_publisher(String, '/vision/latest_at_id', 10)

        # ตัวตรวจจับ Pose
        self.pose_detector = PoseDetector()
        
        # ตัวแปรสถานะและข้อมูลล่าสุด (ค้างค่าไว้)
        self.latest_at_id = "Waiting for Tag..."
        self.pose_status = "Not Found"
        
        self.last_time = time.time()
        self.fps = 0

    def handleTopic(self, msg):
        # 1. รับภาพและปรับขนาด
        frame = self.bridge.compressed_imgmsg_to_cv2(msg)
        frame = cv.resize(frame, (640, 480))
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        	
        # 3. ตรวจจับ AprilTag (อัปเดตค่าล่าสุด)
        tags = self.at_detector.detect(gray)
        for tag in tags:
            for i in range(4):
                cv.line(frame, tuple(tag.corners[i].astype(int)), 
                        tuple(tag.corners[(i+1)%4].astype(int)), (255, 0, 255), 2)
            self.latest_at_id = f"ID: {tag.tag_id}"
            at_msg = String()
            at_msg.data = str(tag.tag_id) # ส่งเฉพาะตัวเลข ID หรือ self.latest_at_id ก็ได้
            self.pub_at_id.publish(at_msg)

        # 4. ตรวจจับ Pose (ส่งข้อมูลออก Topic และวาดบนภาพซ้าย)
        frame, point_msg, detected = self.pose_detector.process_pose(frame, draw=True)
        self.pub_point.publish(point_msg)
        self.pose_status = "Detected" if detected else "Not Found"

        # 5. สร้าง Dashboard ด้านขวา (ไม่มีโครงร่างมนุษย์)
        right_panel = np.zeros((480, 400, 3), np.uint8) # กำหนดความกว้าง 400
        
        cv.putText(right_panel, "VISION DASHBOARD", (20, 50), cv.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
        cv.line(right_panel, (20, 70), (350, 70), (255, 255, 255), 1)
        
        # แสดงผล AprilTag ค้างไว้
        cv.putText(right_panel, "Latest AprilTag:", (20, 120), cv.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
        cv.putText(right_panel, self.latest_at_id, (20, 155), cv.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 255), 2)
        
        # แสดงสถานะ Pose
        color = (0, 255, 0) if detected else (0, 0, 255)
        cv.putText(right_panel, f"Human Pose: {self.pose_status}", (20, 220), cv.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

        # 6. คำนวณ FPS
        curr_time = time.time()
        self.fps = 1 / (curr_time - self.last_time)
        self.last_time = curr_time
        cv.putText(frame, f"FPS: {int(self.fps)}", (20, 40), cv.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

        # 7. รวมภาพ (ซ้าย+ขวา) และแสดงผล
        combined = np.hstack((frame, right_panel))
        cv.imshow('Yahboom Multi-Sensor Vision', combined)
        cv.waitKey(1)

def main():
    print("Initializing Multi-Vision System...")
    rclpy.init()
    node = MY_Picture("Yahboom_Vision_Node")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
