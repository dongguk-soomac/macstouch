#!/usr/bin/env python3

# 현재 코드 따로 복사해두고, 나중에 main에서 pull 받아서 새로 브랜치 판 다음에 control 바뀐거 수정해서 pr하기

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
def transformation_camera(_material_index, _pick_coord, _materail_coord, _mode):
    ############ Memo ############
    # 기본 tool 길이 : 158.2
    ##############################
    ### 변수 설정 ###
    global coordinates_data
    tool_length = coordinates_data["tool_length"]

    tool = tool_length[material_index]
    material_index = deepcopy(_material_index)
    pick_coord = deepcopy(_pick_coord)
    materail_coord = deepcopy(_materail_coord)
    mode = deepcopy(_mode)

    camera_thick = 25.2
    camera_z_offset = camera_thick + 50 # 50 : 6축의 회전축과 카메라 체결부까지의 거리 
    camera_stand_to_center = 6.18 # 카메라 거치대부터 카메라 센터까지의 거리(x축 거리)
    camera_origin_translation_x = 9 # 카메라 원점 x 방향 조절
    camera_origin_translation_y = 25 # 카메라 원점 y 방향 조절
    min_z = 50 # 그리퍼가 바닥과 충돌하지 않는 최소 높이

    z_offset_for_each_index = None

    ### 좌표 변환(about x, y) ### // z는 이후 재료별로 보정
    # cm to mm 변환
    for i in range(3):
        materail_coord[i] = materail_coord[i]*10

    # x방향 pick좌표 설정
    if pick_coord[0] > 0: # 우측 재료
        camera_x_offset = tool + camera_stand_to_center
        pick_coord[0] = pick_coord[0] - camera_x_offset - materail_coord[1] + camera_origin_translation_x
    
    else: # 좌측 재료
        camera_x_offset = tool - camera_stand_to_center
        pick_coord[0] = pick_coord[0] + camera_x_offset + materail_coord[1] - camera_origin_translation_x

    # y방향 pick좌표 설정
    pick_coord[1] = pick_coord[1] - materail_coord[0] + camera_origin_translation_y

    ### 재료별 튜닝 ###
    # tool: 카메라 마운트 ~ 엔드이펙터 길이
    # z_offset_for_each_index: z 높이 조절 offset (+ : 높게, - : 낮게)
    # rx, ry, rz: 
    # [-90, 0, -180] : 카메라 포즈에서 그대로 내려간 것, 
    # [-45, 0, 180] : 카메라 포즈에서 45도 돌아서 내려간 것

    if material_index == 2: # cheeze # 진행 필요
        z_offset_for_each_index = 0
        rz, ry, rx = -90, 0, -180

    if material_index == 3: # pickle ### okay ###
        z_offset_for_each_index = -10
        rz, ry, rx = materail_coord[3:6]

    if material_index == 4: # onion ### okay ###
        mode = int(mode)
        # x
        pick_coord[0] = 694.5 - 40 - 42 * mode     
        # y
        pick_coord[1] = 102.647
        # z
        z_offset_for_each_index = -20
        # rpy
        rz, ry, rx = -90, 0, -180

    if material_index == 5: # sauce # 진행 필요
        z_offset_for_each_index = 0
        rz, ry, rx = -90, 0, -180

    if material_index == 6: # tomato
        z_offset_for_each_index = -5
        rz, ry, rx = materail_coord[3:6] # from vision data
          
    if material_index == 7: # lettuce ### okay ###
        z_offset_for_each_index = -10
        pick_coord[1] = -68.408
        mode = int(mode)
        pick_coord[0] = 694.5 - 40 - 42 * mode
        rz, ry, rx = 90, 0, 180

    # z방향 pick좌표 설정
    pick_coord[2] = np.max((pick_coord[2] - camera_z_offset - materail_coord[2] + z_offset_for_each_index, min_z))

    #rz
    pick_coord[3] = rz

    #ry
    pick_coord[4] = ry

    #rx
    pick_coord[5] = rx

    return pick_coord

class ManageCoord:
    def __init__(self):
        global coordinates_data
        self.place_coord = coordinates_data["place_coord"] # 초기 place 좌표        
        self.bread_coord = coordinates_data["bread_coord"] # 초기 빵 좌표
        self.meat_coord = coordinates_data["meat_coord"] # 초기 빵 좌표

        self.material_height = coordinates_data["material_height"] # place 높이 조절을 위한 재료 높이, 튜닝 필요
        self.bread_offset = coordinates_data["bread_offset"] # 빵 간격, 튜닝 필요
        self.meat_offset = coordinates_data["meat_offset"] # 패티 간격, 튜닝 필요
        self.bread_lid_coord = coordinates_data["bread_lid_coord"]        

        # 빵, 패티의 초기 index 설정
        self.bread_index = 0
        self.meat_index = 0        
        
    def change_bread_pick_coord(self):
        self.bread_index += 1
        if self.bread_index <= 3: # 0, 1, 2, 3번 빵은 x축 방향으로 나열 됨
            self.bread_coord[0] += self.bread_offset[0]

        elif self.bread_index == 4: # 4번 빵은 3번빵으로부터 y축 방향으로 한 칸 떨어진 빵
            self.bread_coord[1] += self.bread_offset[1]

        elif self.bread_index >= 5: # 4, 5, 6, 7번 빵은 x축 방향으로 나열 됨
            self.bread_coord[0] -= self.bread_offset[2]

    def change_meat_pick_coord(self):
        self.bread_index += 1
        if self.meat_index <= 3: # 0, 1, 2, 3번 패티는 x축 방향으로 나열 됨
            self.meat_coord[0] += self.meat_offset[0]

        elif self.meat_index == 4: # 4번 패티는 3번빵으로부터 y축 방향으로 한 칸 떨어진 빵
            self.meat_coord[1] += self.meat_offset[1]

        elif self.meat_index >= 5: # 4, 5, 6, 7번 패티는 x축 방향으로 나열 됨
            self.meat_coord[2] -= self.meat_offset[2]

    def change_place_coord(self, material_index):
        self.bread_index += self.material_height[material_index]

    def reset_place_coord(self):
        self.place_coord = coordinates_data["place_coord"] # 초기 place 좌표

    def reset_bread_coord(self):
        self.bread_index = 0
        self.bread_coord = coordinates_data["bread_coord"] # 초기 빵 좌표

    def reset_meat_coord(self):
        self.meat_index = 0        
        self.meat_coord = coordinates_data["meat_coord"] # 초기 빵 좌표


class Control:
    def __init__(self) -> None:
        # ManageCoord class
        self.managecoord = ManageCoord()

        # json 파일의 고정 좌표값 저장
        global coordinates_data
        self.init_pos = coordinates_data["init_pos"]
        self.tool_coord = coordinates_data["tool_coord"] # 2차원 배열이며, 재료에 따른 도구별 저장 순서는 MaterialList를 따름
        self.vision_coord = coordinates_data["vision_coord"]
        tool_list = coordinates_data["tool_length"]

        # tool에 따른 vision 좌표 보정
        for i in range(8):
            self.vision_coord[i][0] += tool_list[i]
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
                # 좌표 설정
                if self.material == 0: # bread : 고정 좌표
                    pick_coord = self.managecoord.bread_coord
                    self.managecoord.change_bread_pick_coord()
                
                if self.material == 1: # meat : 고정 좌표
                    pick_coord = self.managecoord.meat_coord
                    self.managecoord.change_meat_pick_coord()

                else: # 변동 좌표에 대한 pick_coord 설정
                    pick_coord = transformation_camera(self.material, self.vision_coord[self.material], list(self.coord), self.grip_mode)

                print('##### [Mode : pnp] step_1 : pick action')
                print(pick_coord)
                self.control_action_pub('pick', None, self.grip_mode, pick_coord, None) # client에서는 grip_mode를 바탕으로 pick 동작 구분            
                self.action_state += 1

            elif self.action_state == 2:  
                if self.material == 0: # bread일 때 place 동작 구분
                    print('##### [Mode : pnp] step_2 : bread_place action') 
                    self.control_action_pub('bread_place', self.material, None, self.managecoord.place_coord, self.managecoord.bread_lid_coord) # client에서는 place action 시 material index를 바탕으로 place 동작 구분    
                    self.action_state += 1
                # 토마토의 경우, 어디를 집었는지에 따라 위치 보정하는 코드 추가할 것.
                else:
                    print('##### [Mode : pnp] step_2 : place action') 
                    self.control_action_pub('place', self.material, None, self.managecoord.place_coord, None) # client에서는 place action 시 material index를 바탕으로 place 동작 구분    
                    self.action_state += 1
                    self.managecoord.change_place_coord(self.material) # place 위치 상승

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

        elif self.mode == 'finish':
            if self.action_state == 1:
                print('##### [Mode : finish] step_1 : bread_close')
                self.control_action_pub('bread_close', self.material, None, self.managecoord.bread_lid_coord, self.managecoord.place_coord)        
                self.action_state += 1

            # elif self.action_state == 2:
            #     print('##### [Mode : finish] step_2 : lid_close')
            #     self.control_action_pub('lid_close', self.material, None, self.tool_coord[self.material], None)            
            #     self.action_state += 1

            # elif self.action_state == 2:
            #     print('##### [Mode : finish] step_3 : push')
            #     self.control_action_pub('push', self.material, None, self.tool_coord[self.material], None)            
            #     self.action_state += 1

            elif self.action_state == 2:
                print('##### [Mode : finish] step_4 : done')
                msg = Bool()
                msg.data = True
                self.done.publish(msg)
                self.action_state += 1                


            else:
                pass     
        
    
    def action_done_cb(self, data):
        self.action()
        
    def control_action_pub(self, action, material, grip_mode, coord, coord_2):
        action_msg = action_info() # [action, material, grip_mode, coord, grip_size]

        # 보내려는 값이 None일 시 해당 값은 디폴트 값으로 설정 후 publish
        if material == None:
            material = -1
        if grip_mode == None:
            grip_mode = "x"
        if coord == None:
            coord = self.init_pos
        if coord_2 == None:
            coord_2 = self.init_pos

        action_msg.action = action
        action_msg.material = material
        action_msg.grip_mode = grip_mode
        action_msg.coord = coord
        action_msg.coord_2 = coord_2
        
        self.action_req.publish(action_msg)

if __name__ == '__main__':
    try:
        rospy.init_node('control', anonymous=True)
        rospy.loginfo("Control_node is on")
        Control()

    except rospy.ROSInterruptException:
        rospy.loginfo('Program is shut down')



# "init_pos" : [98.26, 39.8, 1069.9, -87.3, -2.18, 0],