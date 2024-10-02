#!/usr/bin/env python
# -- coding: utf-8 --

import rospy
from std_msgs.msg import String, Int16
from macstouch.msg import vision_info

import os, sys
sys.path.append(os.path.dirname(__file__))
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))


import cv2
import numpy as np
from ultralytics import YOLO
import pyrealsense2

# from macstouch_config import MaterialList
from realsense.realsense_camera import DepthCamera

MaterialList = ["bread", "meat", "cheeze", "pickle", "onion", "sauce", "tomato", "lettuce"]
resolution_width, resolution_height = (848, 480)

class Vision:
    def __init__(self) -> None:
        self.vision_sub = rospy.Subscriber('/vision_req', Int16, self.vision_callback, queue_size=1)
        self.vision_pub = rospy.Publisher('/pick_coord', vision_info, queue_size=1)

        self.model = YOLO('/home/choiyj/catkin_ws/src/macstouch/src/vision/pt/best.pt')
        self.coord_limit = [[-20, 20],[-20, 20], [50, 60]]

        self.rs = DepthCamera(resolution_width, resolution_height)
        ret, depth_raw_frame, color_raw_frame = self.rs.get_raw_frame()
        self.color_frame = np.asanyarray(color_raw_frame.get_data())
        self.depth_scale = self.rs.get_depth_scale()
        self.coord = None

    def test(self):
        ret, depth_raw_frame, color_raw_frame = self.rs.get_raw_frame()
        color_frame = np.asanyarray(color_raw_frame.get_data())
        depth_frame = depth_raw_frame.as_depth_frame()

        available = False

        while available is False:
            available = self.coord_check(self.coord)

        return self.coord

    def vision_callback(self, msg):
        target = MaterialList[msg.data]
        print(target)

        # ret, depth_raw_frame, color_raw_frame = self.rs.get_raw_frame()
        # self.color_frame = np.asanyarray(color_raw_frame.get_data())
        # depth_frame = depth_raw_frame.as_depth_frame()

        available = False

        while available is False:
            # coord = self.yolo_detection(self.color_frame, depth_frame)
            available = self.coord_check(self.coord)
            print(available)
            
        mode, grip_pos, size = self.grip_detection(self.coord)

        self.pub(msg.data, mode, grip_pos, size)

    def coord_check(self, coord):
        x_check = self.coord_limit[0][0] < coord[0] < self.coord_limit[0][1]
        y_check = self.coord_limit[1][0] < coord[1] < self.coord_limit[1][1]
        z_check = self.coord_limit[2][0] < coord[2] < self.coord_limit[2][1]
        return x_check * y_check * z_check

    def yolo_detection(self, color_frame, depth_frame):
        results = self.model(color_frame)

        annotated_frame = color_frame.copy()

        min_dis = 99999999999
        center_xy = [424,240]

        _bbox = None

        for result in results:
            boxes = result.boxes
            for box in boxes:
                confidence = box.conf
                if confidence > 0.8:
                    xyxy = box.xyxy.tolist()[0]
                    cx = int((xyxy[2]+xyxy[0])//2)
                    cy = int((xyxy[3]+xyxy[1])//2)
                    dis = (cx-resolution_width/2)**2+(cy-resolution_height/2)**2
                    if dis < min_dis:
                        min_dis = dis
                        center_xy = [cx, cy]
                        _bbox = xyxy

        if _bbox is None:
            return [0,0,0]

        color = [255, 0, 0]
        bbox = list(map(int, _bbox)) 
        x, y, x2, y2 = bbox

        depth = round((depth_frame.get_distance(center_xy[0], center_xy[1]) * 100), 2)
        wx, wy, wz = pyrealsense2.rs2_deproject_pixel_to_point(self.rs.depth_intrinsics, [center_xy[0], center_xy[1]], depth)
        wx = round(wx, 3)
        wy = round(wy, 3)
        wz = round(wz, 3)
        
        cv2.putText(annotated_frame, "{}, {}, {}".format(wx, wy, wz), (x + 5, y + 60), 0, 1.0, color, 2)

        cv2.rectangle(annotated_frame, (x, y), (x2, y2), color, 2)
        print(wx, wy, wz)

        resized_frame = cv2.resize(annotated_frame, (848, 480))
        self.color_frame = resized_frame
        # cv2.imshow('Detecting pickle and tomato', resized_frame)

        return [wx, wy, wz]

    def grip_detection(self, coord):
        return 'pnp', [coord[0], coord[1], coord[2], 0, 0, 0], 0
    
    def pub(self, target, mode, grip_pos, size):
        data = vision_info()
        data.material = target
        data.grip_mode = 0
        data.coord = grip_pos
        data.size = size
        self.vision_pub.publish(data)


def main():
    rospy.init_node("vision_node")
    vision = Vision()

    while not rospy.is_shutdown():
        # coord = vision.test()
        ret, depth_raw_frame, color_raw_frame = vision.rs.get_raw_frame()
        vision.color_frame = np.asanyarray(color_raw_frame.get_data())
        depth_frame = depth_raw_frame.as_depth_frame()
        vision.coord = vision.yolo_detection(vision.color_frame, depth_frame) 
        cv2.imshow('tomato', vision.color_frame)
        # print("detected!!  ", coord)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    Vision.rs.close()

if __name__ == "__main__":
    main()