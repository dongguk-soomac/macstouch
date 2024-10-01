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

from macstouch_config import MaterialList
from realsense.realsense_camera import DepthCamera

resolution_width, resolution_height = (640, 480)

class Vision:
    def __init__(self) -> None:
        self.vision_sub = rospy.Subscriber('/vision_req', Int16, self.vision_callback, queue_size=1)
        self.vision_pub = rospy.Publisher('/pick_coord', vision_info)

        self.model = YOLO('./pt/best.pt')
        self.coord_limit = []

        self.rs = DepthCamera(resolution_width, resolution_height)
        self.depth_scale = self.rs.get_depth_scale()

    def vision_callback(self, msg):
        target = MaterialList[msg.data]

        ret, depth_raw_frame, color_raw_frame = self.rs.get_raw_frame()
        color_frame = np.asanyarray(color_raw_frame.get_data())
        depth_frame = depth_raw_frame.as_depth_frame()

        available = False

        while available is False:
            coord = self.yolo_detection(color_frame, depth_frame, target)
            available = self.coord_check(coord)
            
        mode, grip_pos, size = self.grip_detection(color_frame, depth_frame, coord)

        self.pub(msg.data, mode, grip_pos, size)

    def coord_check(self, coord):
        x_check = self.coord_limit[0][0] < coord[0] < self.coord_limit[0][1]
        y_check = self.coord_limit[1][0] < coord[1] < self.coord_limit[1][1]
        z_check = self.coord_limit[2][0] < coord[2] < self.coord_limit[2][1]
        return x_check * y_check * z_check

    def yolo_detection(self, color_frame, depth_frame, target):
        results = self.model(color_frame)

        annotated_frame = color_frame.copy()

        min_dis = 99999999999
        center_xy = [320,240]
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
                        _bbox = box

        color = [255, 0, 0]
        bbox = list(map(int, _bbox)) 
        x, y, x2, y2 = bbox

        depth = round((depth_frame.get_distance(center_xy[0], center_xy[1]) * 100), 2)
        wx, wy, wz = pyrealsense2.rs2_deproject_pixel_to_point(self.rs.depth_intrinsics, [cx, cy], depth)

        cv2.putText(annotated_frame, "{}, {}, {}".format(wx, wy, wz), (x + 5, y + 60), 0, 1.0, color, 2)

        cv2.rectangle(annotated_frame, (x, y), (x2, y2), color, 2)
        print(wx, wy, wz)

        return [wx, wy, wz]

    def grip_detection(self, color_frame, depth_frame, coord):
        return 'pnp', coord.expand([0,0,0,0]), 0
    
    def pub(self, target, mode, grip_pos, size):
        data = vision_info()
        data.material = target
        data.grip_mode = mode
        data.coord = grip_pos
        data.size = size
        self.vision_pub.publish(data)

def main():
    rospy.init_node("main_node")
    vision = Vision()

    rospy.spin()
    Vision.rs.close()

if __name__ == "__main__":
    main()