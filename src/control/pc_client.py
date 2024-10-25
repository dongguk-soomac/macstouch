#!/usr/bin/python
# -*- coding: utf-8 -*-
import socket 
import time
import rospy
import numpy as np
import json
import os, sys

# msg
from std_msgs.msg import Float32MultiArray as fl
from std_msgs.msg import Bool, Float32, String
from macstouch.msg import action_info
from copy import deepcopy

sys.path.append(os.path.dirname(__file__))
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

sock = None

# json 파일 경로 설정
current_dir = os.path.dirname(os.path.abspath(__file__))
json_file_path = os.path.join(current_dir, 'coordinates.json')

# json 파일 읽기
with open(json_file_path, 'r', encoding='utf-8') as file:
    coordinates_data = json.load(file)

# MaterialList = ["bread", "meat", "cheeze", "pickle", "onion", "sauce", "tomato", "cabage"]

## 1. Action ########################################################

class Action:
    def __init__(self) -> None:
        # fixed pos
        self.init_pos = coordinates_data["init_pos"]
        # self.vision_pos = coordinates_data["vision_coord"]
        self.tool_pos = coordinates_data["tool_coord"]
        # for push
        self.push_length = coordinates_data["push_length"]

        ## offset
        # pick and place offset
        self.pnp_offset_up = [0, 0, 100, 0, 0, 0]
        self.pnp_offset_down = [0, 0, -100, 0, 0, 0]
        self.pnp_offset_back = [-100, 0, 0, 0, 0, 0]
        self.pnp_offset_after_up = [0, 0, 130, 0, 0, 0]
        self.tool_offset_forward = [150, 0, 0, 0, 0, 0]
        self.tool_offset_backward = [-150, 0, 0, 0, 0, 0]

        self.pnp_offset_tomato = [-40, 0, -100, 0, 0, 0]

        self.pnp_case_offset_up = [0, 0, 200, 0, 0, 0]
        self.tool_offset_slow_forward = [50, 0, 0, 0, 0, 0]
        self.tool_offset_slow_backward = [-50, 0, 0, 0, 0, 0]
                
        self.tool_offset_upward = [0, 0, 160, 0, 0, 0]
        self.tool_offset_downward = [0, 0, -160, 0, 0, 0]
        
        self.tool_offset_45 = 100
        # for 빵 뚜껑 집기
        self.bread_offset_up = [0, 0, 0, 0, 0, 0]
        self.bread_offset_down = [0, 0, 0, 0, 0, 0]

        # for 소스
        self.sauce_offset_forward = [100, 0, 0, 0, 0, 0]
        self.sauce_offset_backward = [-100, 0, 0, 0, 0, 0]
        self.sauce_offset_upward = [0, 0, 100, 0, 0, 0]
        self.sauce_offset_downward = [0, 0, -100, 0, 0, 0]

        self.sauce_pnp_offset_upward = [0, 0, 150, 0, 0, 0]
        self.sauce_pnp_offset_downward = [0, 0, -150, 0, 0, 0]
        
        # for lid close
        self.lid_close_forward = [0, 25, 0, 0, 0, 0]
        self.lid_close_backward = [0, -25, 0, 0, 0, 0]
        self.lid_close_upward = [0, 0, 25, 0, 0, 0]
        self.lid_close_downward = [0, 0, -25, 0, 0, 0]        

        # for grill_open
        self.grill_offset_forward = [-150, 0, 0, 0, 0, 0]
        self.grill_offset_backward = [150, 0, 0, 0, 0, 0]
        self.grill_offset_upward = [0, 0, 250, 0, 0, 0]
        self.grill_offset_downward = [0, 0, -250, 0, 0, 0]

    # 소켓 통신으로 보내는 문자열 생성
    def make_str(self, *args):
        # 전달받은 모든 인자들을 문자열로 변환하고 +로 결합
        setdata = '+'.join(map(str, args))
        return setdata

    # 문자 배열을 0,0,0,0,0,0 형태로 바꿔주는 함수
    def format_array(self, data):
        result = ','.join(map(str, data))
        # print(result)
        
        return result

    
    ## 동작 구분 #################################################

    # modes = ["MovePoint",    # ("MovePoint", "end_effector", str(index))
    #          "MoveOffset",   # ("MoveOffset", self.format_array(self.tool_offset_forward))
    #          "MoveGrip",     # ("MoveGrip", "pick", self.format_array(material_coord), self.format_array(-self.pnp_offset_up), False)
    #          "MoveSmooth",
    #          "Gripper",      # ("Gripper", True)
    #          "ChangeTool", 
    #          "ChangeParam", 
    #          "ReadCoord", 
    #          "ReadJoint", 
    #          "ReadState"]

    def action_init_pos(self, target_position):
        # setdata1 = self.make_str("ToolBase",  self.format_array([0]))
        # setdata = self.make_str("MovePoint", self.format_array(target_position))
        setdata = self.make_str("MoveJoint", self.format_array(target_position))
        #setdata = '='.join([setdata1, setdata2])
        socke.send_data(setdata)

    def action_vision(self, vision_coord, posture):
        # 1. MV(Material_vision)
        vision_coord =list(vision_coord)
        vision_coord.append(posture)
        setdata1 = self.make_str("MovePoint", self.format_array(vision_coord))
        setdata = '='.join([setdata1])
        socke.send_data(setdata)

    # def action_tool_get(self, tool_index, tool_position): # for test
    #     setdata = self.make_str("ToolNum",  self.format_array([tool_index]))
    #     print(tool_index)
    #     socke.send_data(setdata)


    # def tool_return(self, tool_coord):
    #     # 1. change tool
    #     setdata = self.make_str("ToolBase",  self.format_array([0]))
    #     socke.send_data(setdata)

    def action_tool_get(self, tool_index, tool_position, posture):
        if tool_position[0] <= 0: # 빵, 패티, 소스 x방향 보정
            self.tool_offset_backward = self.direction_change(self.tool_offset_backward)
            self.tool_offset_forward = self.direction_change(self.tool_offset_forward)        
            self.tool_offset_slow_backward = self.direction_change(self.tool_offset_slow_backward)
            self.tool_offset_slow_forward = self.direction_change(self.tool_offset_slow_forward)                      

        # 기본툴로 변경
        setdata1 = self.make_str("ToolBase",  tool_index)
        print(tool_index)   

        # 2. tool 장착 + 오프셋으로 이동
        offset_point = self.list_add(self.list_add(tool_position, self.tool_offset_backward), self.tool_offset_slow_backward)
        offset_point.append(posture)
        setdata2 = self.make_str("MovePoint", self.format_array(offset_point))
            
        # 3. 그리퍼 열기
        setdata3 = self.make_str("Gripper", True)
        
        # 5. 장착 직전까지 이동
        setdata4 = self.make_str("MoveOffset", self.format_array(self.tool_offset_forward))
        
        # 6. 속도 매우느림으로 변경
        setdata5 = self.make_str("ChangeParam",  1)

        # 7. 장착
        setdata6 = self.make_str("MoveOffset", self.format_array(self.tool_offset_slow_forward))

        # 8. 속도 보통으로 변경
        setdata7 = self.make_str("ChangeParam", 0)

        # 9. 위로
        setdata8 = self.make_str("MoveOffset",  self.format_array(self.tool_offset_upward))
        
        # 10. tool 변경
        setdata9 = self.make_str("ToolNum",  tool_index)
        
        # 11. 뒤로 이동
        offset_point_back = self.list_add(self.tool_offset_slow_backward, self.tool_offset_backward)
        setdata10 = self.make_str("MoveOffset",  self.format_array(offset_point_back))
        print(tool_index)

        if tool_position[0] <= 0: # 방향 원위치
            self.tool_offset_backward = self.direction_change(self.tool_offset_backward)
            self.tool_offset_forward = self.direction_change(self.tool_offset_forward)   
            self.tool_offset_slow_backward = self.direction_change(self.tool_offset_slow_backward)
            self.tool_offset_slow_forward = self.direction_change(self.tool_offset_slow_forward)                 

        setdata = '='.join([setdata1, setdata2, setdata3, setdata4, setdata5, setdata6, setdata7, setdata8, setdata9, setdata10])
        socke.send_data(setdata)

    def action_tool_get_45(self, tool_index, tool_position, posture):                     
        setdata = [''] * 10
        
        # 기본툴로 변경
        print("45###############")
        setdata.append(self.make_str("ToolBase",  tool_index))

        # 3. 그리퍼 열기
        setdata.append(self.make_str("Gripper", True))
        
        
        tool_offset_back = [0]*6
        tool_offset_back[0] = -self.tool_offset_45 * np.cos((90+tool_position[3]) * np.pi/180)
        tool_offset_back[1] = -self.tool_offset_45 * np.sin((90+tool_position[3]) * np.pi/180)

        tool_offset_front = [0]*6
        tool_offset_front[0] = 0.5 * self.tool_offset_45 * np.cos((90+tool_position[3]) * np.pi/180)
        tool_offset_front[1] = 0.5 * self.tool_offset_45 * np.sin((90+tool_position[3]) * np.pi/180)

        # 45도 방향 오프셋으로 이동
        offset_point = self.list_add(tool_position, tool_offset_back)
        offset_point.append(posture)
        setdata.append(self.make_str("MovePoint", self.format_array(offset_point)))
        print("tool_coord : ", tool_position)
        print("offset_point : ", offset_point)

        # 5. 장착 직전까지 이동
        setdata.append(self.make_str("MoveOffset", self.format_array(tool_offset_front)))
        print("tool_offset_front : ", tool_offset_front)
        
        # 6. 속도 매우느림으로 변경
        setdata.append(self.make_str("ChangeParam",  1))

        # 7. 장착
        setdata.append(self.make_str("MoveOffset", self.format_array(tool_offset_front)))
        print("tool_offset_front : ", tool_offset_front)

        # 8. 속도 보통으로 변경
        setdata.append(self.make_str("ChangeParam", 0))

        # 9. 위로
        setdata.append(self.make_str("MoveOffset",  self.format_array(self.tool_offset_upward)))
        print("tool_offset_upward : ", self.tool_offset_upward)
        
        # 10. tool 변경
        setdata.append(self.make_str("ToolNum",  tool_index))
        
        # 11. 뒤로 이동
        setdata.append(self.make_str("MoveOffset",  self.format_array(tool_offset_back)))
        print(tool_index)
        print("tool_offset_back : ", tool_offset_back)

        setdata = '='.join(setdata)        
        socke.send_data(setdata) 

    def tool_return(self, tool_coord, posture):
        if tool_coord[0] <= 0: # 방향 반대로
            self.tool_offset_backward = self.direction_change(self.tool_offset_backward)
            self.tool_offset_forward = self.direction_change(self.tool_offset_forward)   
            self.tool_offset_slow_backward = self.direction_change(self.tool_offset_slow_backward)
            self.tool_offset_slow_forward = self.direction_change(self.tool_offset_slow_forward)     

        # 1. change tool
        setdata1 = self.make_str("ToolBase",  0)

        # 2. tool 장착 + 뒤 + 위
        tool_back = self.list_add(self.tool_offset_backward, self.tool_offset_slow_backward)
        offset_point = self.list_add(self.list_add(tool_coord, tool_back),self.tool_offset_upward)
        offset_point.append(posture)
        setdata2 = self.make_str("MovePoint", self.format_array(offset_point))
            
        # 3. 그리퍼 열기
        setdata3 = self.make_str("Gripper", True)

        # 4. 오프셋 앞으로
        setdata4 = self.make_str("MoveOffset", self.format_array(self.list_add(self.tool_offset_forward, self.tool_offset_slow_forward)))

        # 6. 오프셋 아래로
        setdata5 = self.make_str("MoveOffset", self.format_array(self.tool_offset_downward))

        # 5. 속도 감소
        setdata6 = self.make_str("ChangeParam", 1)

        # 7. 툴 빼기
        setdata7 = self.make_str("MoveOffset", self.format_array(self.tool_offset_slow_backward))

        # 8. 속도 원위치
        setdata8 = self.make_str("ChangeParam",  0)

        # 9. 뒤로 오프셋
        setdata9 = self.make_str("MoveOffset", self.format_array(self.tool_offset_backward))        

        if tool_coord[0] <= 0: # 방향 원위치
            self.tool_offset_backward = self.direction_change(self.tool_offset_backward)
            self.tool_offset_forward = self.direction_change(self.tool_offset_forward)   
            self.tool_offset_slow_backward = self.direction_change(self.tool_offset_slow_backward)
            self.tool_offset_slow_forward = self.direction_change(self.tool_offset_slow_forward)    

        setdata = '='.join([setdata1, setdata2, setdata3, setdata4, setdata5, setdata6, setdata7, setdata8, setdata9])
        socke.send_data(setdata)

    def action_tool_return_45(self, tool_position, posture):                     
        setdata = [''] * 8
        
        # 기본툴로 변경
        setdata.append(self.make_str("ToolBase",  0))

        # 3. 그리퍼 열기
        setdata.append(self.make_str("Gripper", True))
        
        
        tool_offset_back = [0]*6
        tool_offset_back[0] = -self.tool_offset_45 * np.cos((90+tool_position[3]) * np.pi/180)
        tool_offset_back[1] = -self.tool_offset_45 * np.sin((90+tool_position[3]) * np.pi/180)

        tool_offset_front = [0]*6
        tool_offset_front[0] = self.tool_offset_45 * np.cos((90+tool_position[3]) * np.pi/180)
        tool_offset_front[1] = self.tool_offset_45 * np.sin((90+tool_position[3]) * np.pi/180)

        # 위, 뒤
        offset_point = self.list_add(self.list_add(tool_position, self.tool_offset_upward),tool_offset_back)
        offset_point.append(posture)
        setdata.append(self.make_str("MovePoint", self.format_array(offset_point)))

        # 위
        setdata.append(self.make_str("MoveOffset", self.format_array(tool_offset_front)))
        


        # 7. 탈착
        setdata.append(self.make_str("MoveOffset", self.format_array(self.tool_offset_downward)))

        # 8. 속도 매우느림으로 변경
        setdata.append(self.make_str("ChangeParam", 1))

        # 9. 뒤로
        setdata.append(self.make_str("MoveOffset",  self.format_array(tool_offset_back)))

        # 6. 속도 보통으로 변경
        setdata.append(self.make_str("ChangeParam",  0))


        setdata = '='.join(setdata)        
        socke.send_data(setdata) 

    def action_pick_lettuce(self, grip_mode, material_coord, posture):
        after_pick = list(deepcopy(material_coord))
        # after_pick[3:6] = -90, 0, 180 # 양상추 잡는 각도가 반대라 일단 보류 -> 추후 case 나누거나 중간 점 찾는 등으로 해결
        if material_coord[0] <= 0: # 방향 반대로
            self.pnp_offset_back = self.direction_change(self.pnp_offset_back)
        
        setdata0 = self.make_str("MoveOffset", self.format_array(self.pnp_offset_back))

        # 2-1. 그리퍼
        setdata1 = self.make_str("Gripper", True)
        # 2-2. "재료 위치 + 오프셋"으로 이동
        offset_point = self.list_add(material_coord, self.pnp_offset_up)
        offset_point.append(posture)
        setdata2 = self.make_str("MovePoint", self.format_array(offset_point))
            
        # 2-3. 재료 위치로 이동 (오프셋만큼 이동)
        setdata3 = self.make_str("MoveOffset", self.format_array(self.pnp_offset_down))

        # 3. gripper
        setdata4 = self.make_str("Gripper", False)

        # 4. 오프셋
        offset_point_2 = self.list_add(after_pick, self.pnp_offset_after_up)
        offset_point_2 = self.list_add(offset_point_2, self.pnp_offset_back)
        offset_point_2.append(posture)
        setdata5 = self.make_str("MovePoint", self.format_array(offset_point_2))

        if material_coord[0] <= 0: # 방향 반대로
            self.pnp_offset_back = self.direction_change(self.pnp_offset_back)

        setdata = '='.join([setdata0, setdata1, setdata2, setdata3, setdata4, setdata5])
        socke.send_data(setdata)

    def action_pick(self, grip_mode, material_coord, posture):
        after_pick = list(deepcopy(material_coord))
        # after_pick[3:6] = -90, 0, 180 # 양상추 잡는 각도가 반대라 일단 보류 -> 추후 case 나누거나 중간 점 찾는 등으로 해결
        if material_coord[0] <= 0: # 방향 반대로
            self.pnp_offset_back = self.direction_change(self.pnp_offset_back)
        
        setdata0 = self.make_str("MoveOffset", self.format_array(self.pnp_offset_back))

        # 2-1. 그리퍼
        setdata1 = self.make_str("Gripper", True)
        # 2-2. "재료 위치 + 오프셋"으로 이동
        offset_point = self.list_add(material_coord, self.pnp_offset_up)
        offset_point.append(posture)
        setdata2 = self.make_str("MovePoint", self.format_array(offset_point))
        setdata21 = self.make_str("ChangeParam",  4)
        
        # 2-3. 재료 위치로 이동 (오프셋만큼 이동)
        setdata3 = self.make_str("MoveOffset", self.format_array(self.pnp_offset_down))

        # 3. gripper
        setdata4 = self.make_str("Gripper", False)

        # 4. 오프셋
        setdata5 = self.make_str("MoveOffset", self.format_array(self.pnp_offset_after_up))        
        # offset_point_2 = self.list_add(after_pick, self.pnp_offset_after_up)

        # offset_point_2.append(posture)
        # setdata5 = self.make_str("MovePoint", self.format_array(offset_point_2))
        setdata6 = self.make_str("ChangeParam",  0)

        if material_coord[0] <= 0: # 방향 반대로
            self.pnp_offset_back = self.direction_change(self.pnp_offset_back)

        setdata = '='.join([setdata0, setdata1, setdata2, setdata3, setdata4, setdata5, setdata6])
        socke.send_data(setdata)

    def action_pick_case(self, index, material_coord, posture):
        after_pick = list(deepcopy(material_coord))
        # after_pick[3:6] = -90, 0, 180 # 양상추 잡는 각도가 반대라 일단 보류 -> 추후 case 나누거나 중간 점 찾는 등으로 해결
        if material_coord[0] <= 0: # 방향 반대로
            self.pnp_offset_back = self.direction_change(self.pnp_offset_back)
        
        setdata0 = self.make_str("MoveOffset", self.format_array(self.pnp_offset_back))

        # 2-1. 그리퍼
        setdata1 = self.make_str("Gripper", True)
        # 2-2. "재료 위치 + 오프셋"으로 이동
        offset_point = self.list_add(material_coord, self.pnp_offset_up)
        offset_point.append(posture)
        setdata2 = self.make_str("MovePoint", self.format_array(offset_point))
            
        # 2-3. 재료 위치로 이동 (오프셋만큼 이동)
        setdata3 = self.make_str("MoveOffset", self.format_array(self.pnp_offset_down))

        # 3. gripper
        setdata4 = self.make_str("Gripper", False)

        # 4. 오프셋
        setdata5 = self.make_str("MoveOffset", self.format_array(self.pnp_case_offset_up))

        if material_coord[0] <= 0: # 방향 반대로
            self.pnp_offset_back = self.direction_change(self.pnp_offset_back)

        setdata = '='.join([setdata0, setdata1, setdata2, setdata3, setdata4, setdata5])
        socke.send_data(setdata)

    def action_place(self, place_coord, posture):

        # 1. "오프셋"으로 이동
        offset_point = self.list_add(place_coord, self.pnp_offset_up)
        offset_point.append(posture)
        setdata1 = self.make_str("MovePoint", self.format_array(offset_point))
        # 2. 버거 위치로 이동
        setdata2 = self.make_str("MoveOffset", self.format_array(self.pnp_offset_down))
        # 3. 그리퍼
        setdata3 = self.make_str("Gripper", True)
        # 4. 오프셋 위로 이동
        setdata4 = self.make_str("MoveOffset", self.format_array(self.pnp_offset_up))

        setdata = '='.join([setdata1, setdata2, setdata3, setdata4])
        socke.send_data(setdata)

    def action_place_tomato(self, place_coord, posture):

        # 1. "오프셋"으로 이동
        offset_point = self.list_add(place_coord, self.pnp_offset_up)
        offset_point.append(posture)
        setdata1 = self.make_str("MovePoint", self.format_array(offset_point))
        # 2. 버거 위치로 이동
        setdata2 = self.make_str("MoveOffset", self.format_array(self.pnp_offset_tomato))
        # 3. 그리퍼
        setdata3 = self.make_str("Gripper", True)
        # 4. 오프셋 위로 이동
        setdata4 = self.make_str("MoveOffset", self.format_array(self.pnp_offset_up))


        setdata = '='.join([setdata1, setdata2, setdata3, setdata4])
        socke.send_data(setdata)

    def action_bread_place(self, place_coord, bread_lib_coord, posture):
        # 1. "오프셋"으로 이동
        offset_point = self.list_add(place_coord, self.pnp_offset_up)
        offset_point.append(posture)
        setdata1 = self.make_str("MovePoint", self.format_array(offset_point))

        # 2. 버거 위치로 이동
        setdata2 = self.make_str("MoveOffset", self.format_array(self.pnp_offset_down))

        # 3. 그리퍼 open
        setdata3 = self.make_str("Gripper", True)

        # 4. 위 빵 집기
        setdata4 = self.make_str("MoveOffset", self.format_array(self.bread_offset_up))

        # 5. 그리퍼 close
        setdata5 = self.make_str("Gripper", False)       
         
        # 6. 오프셋 위로 이동
        setdata6 = self.make_str("MoveOffset", self.format_array(self.list_add(self.pnp_offset_up, self.bread_offset_down)))
        
        # 7. 버거 뚜껑이 위치할 좌표 위로 이동
        offset_point_2 = self.list_add(bread_lib_coord, self.pnp_offset_up)
        offset_point_2.append(posture)
        setdata7 = self.make_str("MovePoint", self.format_array(offset_point_2))

        # 8. bread_lib 위치로 이동
        setdata8 = self.make_str("MoveOffset", self.format_array(self.pnp_offset_down))
        
        # 9. 그리퍼 open
        setdata9 = self.make_str("Gripper", True)

        # 10. 오프셋 위로 이동
        setdata10 = self.make_str("MoveOffset", self.format_array(self.pnp_offset_up))                

        setdata = '='.join([setdata1, setdata2, setdata3, setdata4, setdata5, setdata6, setdata7, setdata8, setdata9, setdata10])
        socke.send_data(setdata)

    def action_bread_close(self, bread_coord, place_coord, posture):
        # 1. 그리퍼 열기
        setdata1 = self.make_str("Gripper", True)
        
        # 2. "재료 위치 + 오프셋"으로 이동
        setdata2 = self.make_str("MovePoint", self.format_array(self.list_add(bread_coord, self.pnp_offset_up)))
            
        # 3. 재료 위치로 이동 (오프셋만큼 이동)
        setdata3 = self.make_str("MoveOffset", self.format_array(self.pnp_offset_down))

        # 4. 그리퍼 닫기
        setdata4 = self.make_str("Gripper", False)

        # 5. 오프셋 위로 이동
        setdata5 = self.make_str("MoveOffset", self.format_array(self.pnp_offset_up))

        # 6. place위치 + offset 위로 이동
        setdata6 = self.make_str("MovePoint", self.format_array(self.list_add(place_coord, self.pnp_offset_up)))
        
        # 7. 버거 위치로 이동
        setdata7 = self.make_str("MoveOffset", self.format_array(self.pnp_offset_down))

        # 8. 그리퍼 열기
        setdata8 = self.make_str("Gripper", True)

        # 9. 오프셋 위로 이동
        setdata9 = self.make_str("MoveOffset", self.format_array(self.pnp_offset_up))

        setdata = '='.join([setdata1, setdata2, setdata3, setdata4, setdata5, setdata6, setdata7, setdata8, setdata9])
        socke.send_data(setdata)    

    def action_sauce_place(self, place_coord, posture):
        setdata = [''] * 15
        
        # 소스 위치 + 오프셋 이동
        offet_1 = self.list_add(place_coord, self.sauce_pnp_offset_upward)
        offet_1.append(posture)
        setdata.append(self.make_str("MovePoint", self.format_array(offet_1)))

        # 소스 도포 위치로 이동
        setdata.append(self.make_str("MoveOffset", self.format_array(self.list_add(self.sauce_pnp_offset_downward, self.sauce_offset_upward))))

        n_value = 8
        sauce_coord = self.list_add(place_coord, self.sauce_offset_upward)
        sauce_circle = self.sauce_traj(sauce_coord, r = 15, n = n_value, posture = posture)

        # 소스 도포 점의 첫 점으로 이동
        setdata.append(self.make_str("MovePoint", self.format_array(sauce_circle[0])))

        # 8. 속도 느리게
        setdata.append(self.make_str("ChangeParam",  2))

        # 그리퍼 닫기
        setdata4 = self.make_str("Gripper", False)        
        
        # 원 그리며 도포
        setdata.append(self.make_str("MovePoint", self.format_array(sauce_circle[1]))) 
        # setdata4 = self.make_str("Gripper", True)        
        setdata.append(self.make_str("MovePoint", self.format_array(sauce_circle[2]))) 
        # setdata4 = self.make_str("Gripper", False)        
        setdata.append(self.make_str("MovePoint", self.format_array(sauce_circle[3])))
        # setdata4 = self.make_str("Gripper", True)        
        setdata.append(self.make_str("MovePoint", self.format_array(sauce_circle[4])))
        # setdata4 = self.make_str("Gripper", False)        
        setdata.append(self.make_str("MovePoint", self.format_array(sauce_circle[5])))
        # setdata4 = self.make_str("Gripper", True)        
        setdata.append(self.make_str("MovePoint", self.format_array(sauce_circle[6])))
        # setdata4 = self.make_str("Gripper", False)        
        setdata.append(self.make_str("MovePoint", self.format_array(sauce_circle[7])))                                       
        setdata.append(self.make_str("MovePoint", self.format_array(sauce_circle[0])))
        setdata4 = self.make_str("Gripper", True)        

        # 8. 속도 느리게
        setdata.append(self.make_str("ChangeParam",  0))

        setdata.append(self.make_str("MovePoint", self.format_array(offet_1)))

        setdata = '='.join(setdata)        
        socke.send_data(setdata) 

    def action_grill_open(self, traj, posture):
        traj_num = len(traj)       
        data_num = traj_num + 6
        setdata = [''] * data_num
 
      

        # 그릴 손잡이 위치 + 오프셋 이동
        offet_1 = self.list_add(self.list_add(traj[0], self.grill_offset_backward),self.grill_offset_upward)
        offet_1.append(posture)
        setdata.append(self.make_str("MovePoint", self.format_array(offet_1)))

        # 그릴 손잡이 위치로 이동
        setdata.append(self.make_str("MoveOffset", self.format_array(self.grill_offset_downward)))
        setdata.append(self.make_str("MoveOffset", self.format_array(self.grill_offset_forward)))

        # 8. 속도 느리게
        setdata.append(self.make_str("ChangeParam",  3))

        for pos in traj[1:]:
            pos.append(posture)
            # setdata.append(self.make_str("MovePoint", self.format_array(pos))) 
            setdata.append(self.make_str("MovePoint", self.format_array(pos))) 



        setdata.append(self.make_str("MoveOffset", self.format_array(self.grill_offset_backward)))

        # 8. 속도 보통
        setdata.append(self.make_str("ChangeParam",  0))


        setdata = '='.join(setdata)        
        socke.send_data(setdata) 

    def action_grill_close(self, traj, posture):
        pass

    def action_lid_close(self, traj, posture):
        traj_num = len(traj)       
        data_num = traj_num + 6
        setdata = [''] * data_num
      
        # 그릴 손잡이 위치 + 오프셋 이동
        offet_1 = self.list_add(self.list_add(traj[0], self.lid_close_forward),self.lid_close_upward)
        offet_1.append(posture)
        setdata.append(self.make_str("MovePoint", self.format_array(offet_1)))

        # 그릴 손잡이 위치로 이동
        setdata.append(self.make_str("MoveOffset", self.format_array(self.lid_close_downward)))
        setdata.append(self.make_str("MoveOffset", self.format_array(self.lid_close_backward)))

        # 8. 속도 느리게 + overlap
        setdata.append(self.make_str("ChangeParam",  3))

        for pos in traj[1:]:
            pos.append(posture)
            # setdata.append(self.make_str("MovePoint", self.format_array(pos))) 
            setdata.append(self.make_str("MovePoint", self.format_array(pos))) 

        setdata.append(self.make_str("Moveoffset", self.format_array(self.lid_close_upward)))

        # 8. 속도 보통
        setdata.append(self.make_str("ChangeParam",  0))


        setdata = '='.join(setdata)        
        socke.send_data(setdata) 

    def action_push(self, start_point, posture):
        setdata = [''] * 6
        setdata.append(self.make_str("Gripper", True))
        # push 위치 + 상단, 후방 offset 이동
        offet_1 = self.list_add(self.list_add(start_point, self.lid_close_upward), self.lid_close_backward)
        offet_1.append(posture)
        setdata.append(self.make_str("MovePoint", self.format_array(offet_1)))

        # 하단으로 offset
        setdata.append(self.make_str("MoveOffset", self.format_array(self.lid_close_downward)))
        
        # 전방으로 offset
        setdata.append(self.make_str("MoveOffset", self.format_array(self.lid_close_forward)))

        # push 진행
        setdata.append(self.make_str("MoveOffset", self.format_array(self.push_length)))

        # 다시 올라오기(뒤로 + 위로 offset)
        push_back = self.direction_change(self.push_length)
        setdata.append(self.make_str("MoveOffset", self.format_array(self.list_add(push_back, self.lid_close_upward))))
        push_back = self.direction_change(self.push_length)

        setdata = '='.join(setdata)        
        socke.send_data(setdata) 

    def action_test_pos(self, target_pos, posture):
        # 1. 그리퍼 열기
        setdata1 = self.make_str("Gripper", True)
        # 8. 속도 보통
        setdata2 = self.make_str("ChangeParam",  1)
        target_pos = list(target_pos)
        target_pos.append(posture)
        setdata3 = self.make_str("MovePoint", self.format_array(target_pos))
        # 8. 속도 보통
        setdata4 = self.make_str("ChangeParam",  0)        
        setdata = '='.join([setdata1, setdata2, setdata3, setdata4])
        socke.send_data(setdata)  
          
    def sauce_traj(self, center_coord, r, n, posture):
        # 각도를 n 개로 분할
        angles = np.linspace(0, 2 * np.pi, n)

        circle_points = []
        
        for angle in angles:
            x = center_coord[0] + r * np.cos(angle)
            y = center_coord[1] + r * np.sin(angle)
            z = center_coord[2]
            rz = center_coord[3]
            ry = center_coord[4]
            rx = center_coord[5]
            circle_points.append((x, y, z, rz, ry, rx, posture))
    
        return circle_points


    def list_add(self, list_1, list_2):
        return [list(list_1)[i] + list_2[i] for i in range(len(list_2))]
    
    def direction_change(self, list):
        for i in range(len(list)):
            list[i] = -list[i]
        return list

## 2. ROS ########################################################

class Ros():
# ROS 초기화
    def __init__(self):
        rospy.init_node("client_node")
        self.sub_action_req = rospy.Subscriber('/action_req', action_info, self.action_req_callback)
        self.pub_action_done = rospy.Publisher('/action_done', Bool, queue_size=10)

    def action_req_callback(self, msg):
        
        # 받은 action_info에 따라 작업 처리
        action_name = msg.action        # String
        index = msg.material            # int
        grip_mode = msg.grip_mode       # String
        target_position = msg.coord     # FloatArray
        target_position_2 = msg.coord_2        # Float32
        posture = msg.posture
        if len(msg.traj) >= 6:
            traj = list(map(list, list(np.array(list(msg.traj)).reshape((-1,6)))))
            print(traj)
        else:
            traj = msg.traj

        print(f"Action: {action_name}, Index: {index}, Gr)ip Mode: {grip_mode}, "
            f"Target Position: {target_position}, Target Position_2: {target_position_2}")
        

        # current_tool = coordinates_data['MaterialList'][index] # json
        # current_tool_coord = coordinates_data['tool_coord'][index] # json
        # print("current tool is", current_tool, "and coordinate is", current_tool_coord)

        # actions
        if action_name == "init_pos":
            action.action_init_pos(target_position)
        elif action_name == "tool_get":
            action.action_tool_get(index, target_position, posture)
        elif action_name == "tool_get_45":
            action.action_tool_get_45(index, target_position, posture)         
        elif action_name == "tool_return_45":
            action.action_tool_return_45(target_position, posture)                 

        elif action_name == "vision":
            action.action_vision(target_position, posture)
        elif action_name == "pick":
            if index == 9:
                action.action_pick_case(index, target_position, posture)
            elif index == 8 :
                action.action_pick_lettuce(index, target_position, posture)
            else:
                action.action_pick(index, target_position, posture)            
        elif action_name == "place":
            action.action_place(target_position, posture)
        elif action_name == "place_tomato":
            action.action_place_tomato(target_position, posture)
        elif action_name == "tool_return":
            action.tool_return(target_position, posture)
        elif action_name == "bread_place":
            action.action_bread_place(target_position, target_position_2, posture)            
        elif action_name == "bread_close":
            action.action_bread_close(target_position, target_position_2, posture)                            
        elif action_name == "sauce_place":
            action.action_sauce_place(target_position, posture) 



        elif action_name == "grill_open":
            action.action_grill_open(traj, posture)     
        elif action_name == "grill_close":
            action.action_grill_close(traj, posture)  

        elif action_name == "lid_close":
            action.action_lid_close(traj, posture)       

        elif action_name == "push":
            action.action_push(target_position, posture)

        elif action_name == "test_pos":
            action.action_test_pos(target_position, posture)                                                    
                                
                                

    def publish_action_done(self, done):
        self.pub_action_done.publish(done)
        print("Published action done.")
        
    


## 3. Socket #####################################################

class Socket():
        
    # 소켓 서버 설정
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM) #INET은 주소패밀리의 기본값, SOCK_STREAM은 소켓 유형의 기본값
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1) #(level, optname, value: int) 주어진 소켓 옵션의 값을 설정
        self.sock.connect(('192.168.1.23',5000)) #address에 있는 원격 소켓에 연결

        Socket_data = 'start' #data 인스턴스 생성
        Socket_data = Socket_data.encode() # 유니코드를 utf-8, euc-kr, ascii 형식의 byte코드로 변환
        self.sock.send(Socket_data) #data 전송 


    # 소켓 통신에서 행동 완료 신호를 처리하는 함수
    def check_action_done(self, data):
        # 소켓에서 수신한 데이터를 분석하여 완료 신호가 있을 때만 action_done을 발행
        if "done" in data.lower():  # 'done' 문자열을 포함하는지 체크 (대소문자 무시)
            print("Action done, publishing /action_done to Control node")
            ros.pub_action_done.publish(True)


    # 데이터 송신 함수
    def send_data(self, data):
        try:
            setdata = data.encode()  # 문자열 -> byte code 변환
            self.sock.send(setdata)  # 클라이언트 소켓으로 데이터 송신
        except Exception as e:
            print(f"Error sending data: {e}")

    
    def close(self):
        self.sock.close()

# [320, 150, 0, 90, 0, -180],  [98.26, 39.8, 1226.9, -87.3, -2.18, 0]

## 4. Main #######################################################

# 메인 함수
def main():
    
    try:
        global action, ros, socke, sock
        # 클래스 초기화
        action = Action()
        ros = Ros()
        socke = Socket()
        sock = socke.sock

        # 소켓통신 확인 목적 (나중에 삭제)
        # message = input("insert any msg for start client: ")
        # sock.send(message.encode())  # 데이터 전송

        while not rospy.is_shutdown():
            
            # 소켓에서 데이터 수신
            Socket_data = sock.recv(65535)  # 서버로부터 데이터 수신
            if not Socket_data:
                break
            Socket_data = Socket_data.decode()  # 문자열로 변환
            print(Socket_data, "-- received")  # 출력

            # 행동 완료 여부를 확인하는 함수 호출
            socke.check_action_done(Socket_data)

    except Exception as e:
        print("Connection error", str(e))

    except KeyboardInterrupt:           # "ctrl" + "c" 버튼 입력
        print("KeyboardInterrupt")
    
    
    socke.close()  # 소켓 종료

if __name__ == "__main__":
    main()