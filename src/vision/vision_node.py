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

        self.rs = DepthCamera(resolution_width, resolution_height)
        self.depth_scale = self.rs.get_depth_scale()

    def vision_callback(self, msg):
        target = MaterialList[msg.data]

        ret, depth_raw_frame, color_raw_frame = self.rs.get_raw_frame()
        color_frame = np.asanyarray(color_raw_frame.get_data())
        depth_frame = depth_raw_frame.as_depth_frame()

        coord = self.yolo_detection(color_frame, depth_frame, target)
        mode, grip_pos, size = self.grip_detection(color_frame, depth_frame, coord)

        self.pub(msg.data, mode, grip_pos, size)

    def yolo_detection(self, color_frame, depth_frame, target):
        results = self.model(color_frame)

        annotated_frame = color_frame.copy()

        min_dis = 99999999999
        center_xy = [320,240]

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

        depth = round((depth_frame.get_distance(center_xy[0], center_xy[1]) * 100), 2)
        wx, wy, wz = pyrealsense2.rs2_deproject_pixel_to_point(self.rs.depth_intrinsics, [cx, cy], depth)

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

if __name__ == "__main__":
    main()