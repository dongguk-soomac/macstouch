#!/usr/bin/env python3

import rospy
import numpy as np
import math
import time
import json
import os, sys

from copy import deepcopy

# msg
from std_msgs.msg import Float32MultiArray as fl
from std_msgs.msg import Bool, Float32, String
from macstouch.msg import control_info
from macstouch.msg import action_info

# json 파일 경로 설정
current_dir = os.path.dirname(os.path.abspath(__file__))
json_file_path = os.path.join(current_dir, 'coordinates.json')

# json 파일 읽기
with open(json_file_path, 'r', encoding='utf-8') as file:
    coordinates_data = json.load(file)

# MaterialList = ["bread", "meat", "cheeze", "pickle", "onion", "sauce", "tomato", "cabage"]
def transformation_camera(vision_coord, materail_coord):
    pick_coord = deepcopy(vision_coord)
    camera_thick = 0
    camera_z_offset = camera_thick + 50 
    tool = 150
    camera_x_offset = tool - 6.18
    
    for i in range(6):
        materail_coord[i] = materail_coord[i]*10

    #x
    if vision_coord[0] > 0: # 좌측 재료
        pick_coord[0] = vision_coord[0] - camera_x_offset - materail_coord[1] 
    else: # 우측 재료
        pick_coord[0] = vision_coord[0] + camera_x_offset + materail_coord[1]
    # y
    pick_coord[1] = vision_coord[1] + materail_coord[0]
    # z
    pick_coord[2] = vision_coord[2] - camera_z_offset - materail_coord[2]
    #rz
    pick_coord[3] = -90 
    #ry
    pick_coord[4] = 0
    #rx
    pick_coord[5] = -180

    return pick_coord

class Control:
    def __init__(self) -> None:
        # json 파일의 고정 좌표값 저장
        global coordinates_data
        self.init_pos = coordinates_data["init_pos"]
        self.place_coord = coordinates_data["place_coord"]
        self.tool_coord = coordinates_data["tool_coord"] # 2차원 배열이며, 재료에 따른 도구별 저장 순서는 MaterialList를 따름
        self.vision_coord = coordinates_data["vision_coord"]
        # [635, 100, 400, -90, 0, -90]

        # ros setting
        rospy.Subscriber('/control_req', control_info, self.control_cb)
        rospy.Subscriber('/action_done', Bool, self.action_done_cb)

        self.action_req = rospy.Publisher('/action_req', action_info, queue_size=10)


        self.done = rospy.Publisher('/done', Bool, queue_size=10)

        self.action_state = 0
        rospy.spin()

    def control_cb(self, data):
        print('control_req topic')
        print(data)
        self.mode = data.mode
        self.material = data.material
        self.grip_mode = data.grip_mode
        self.coord = data.coord
        self.grip_size = data.grip_size
        self.action_state = 1
        self.action()

    def action(self):
        if self.mode == 'init_pos':
            if self.action_state == 1:
                print('##### [Mode : init_pos] step_1 : init_pos action')
                self.control_action_pub('init_pos', None, None, self.init_pos, None)     
                self.action_state += 1

            elif self.action_state == 2:
                print('##### [Mode : init_pos] step_2 : done')
                msg = Bool()
                msg.data = True
                self.done.publish(msg)
                self.action_state += 1
            else:
                pass

        elif self.mode == 'vision':
            if self.action_state == 1:
                print('##### [Mode : vision] step_1 : vision action')
                self.control_action_pub('vision', None, None, self.vision_coord[self.material], None) # material -> None / grip_size -< None            
                self.action_state += 1

            elif self.action_state == 2:
                print('##### [Mode : vision] step_2 : done')
                msg = Bool()
                msg.data = True
                self.done.publish(msg)
                self.action_state += 1
            else:
                pass                  

        elif self.mode == 'pnp':
            if self.action_state == 1:
                pick_coord = transformation_camera(self.vision_coord[self.material], list(self.coord))
                print('##### [Mode : pnp] step_1 : pick action')
                self.control_action_pub('pick', None, self.grip_mode, pick_coord, self.grip_size) # client에서는 grip_mode를 바탕으로 pick 동작 구분            
                self.action_state += 1

            elif self.action_state == 2:  
                print('##### [Mode : pnp] step_2 : place action') 
                self.control_action_pub('place', self.material, None, self.place_coord, None) # client에서는 place action 시 material index를 바탕으로 place 동작 구분    
                self.action_state += 1

            elif self.action_state == 3:
                print('##### [Mode : pnp] step_3 : done')
                msg = Bool()
                msg.data = True
                self.done.publish(msg)
                self.action_state += 1  
            else:
                pass                

        elif self.mode == 'tool_return':
            if self.action_state == 1:
                print('##### [Mode : tool_return] step_1 : tool_return')
                self.control_action_pub('tool_return', None, None, self.tool_coord[self.material], None)            
                self.action_state += 1

            elif self.action_state == 2:
                print('##### [Mode : tool_return] step_2 : done')
                msg = Bool()
                msg.data = True
                self.done.publish(msg)
                self.action_state += 1
            else:
                pass          
    
        elif self.mode == 'tool_get':
            if self.action_state == 1:
                print('##### [Mode : tool_get] step_1 : tool_get')
                self.control_action_pub('tool_get', self.material, None, self.tool_coord[self.material], None)            
                self.action_state += 1

            elif self.action_state == 2:
                print('##### [Mode : tool_get] step_2 : done')
                msg = Bool()
                msg.data = True
                self.done.publish(msg)
                self.action_state += 1
            else:
                pass          

    def action_done_cb(self, data):
        self.action()
        
    def control_action_pub(self, action, material, grip_mode, coord, grip_size):
        action_msg = action_info() # [action, material, grip_mode, coord, grip_size]

        # 보내려는 값이 None일 시 해당 값은 디폴트 값으로 설정 후 publish
        if material == None:
            material = -1
        if grip_mode == None:
            grip_mode = "x"
        if coord == None:
            coord = self.init_pos
        if grip_size == None:
            grip_size = 0

        action_msg.action = action
        action_msg.material = material
        action_msg.grip_mode = grip_mode
        action_msg.coord = coord
        action_msg.grip_size = grip_size
        
        self.action_req.publish(action_msg)

if __name__ == '__main__':
    try:
        rospy.init_node('control', anonymous=True)
        rospy.loginfo("Control_node is on")
        Control()

    except rospy.ROSInterruptException:
        rospy.loginfo('Program is shut down')