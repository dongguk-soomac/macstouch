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
        self.vision_pos = coordinates_data["vision_coord"]
        self.tool_pos = coordinates_data["tool_coord"]

        ## offset
        # pick and place offset
        self.pnp_offset = [0, 0, 150, 0, 0, 0]
        self.pnp_offset_back = [0, 0, -150, 0, 0, 0]
        self.pnp_offset_x = [-100, 0, 0, 0, 0, 0]

        # tool_get(or return) x offset 
        self.tool_offset_mount = [150, 0, 0, 0, 0, 0]
        self.tool_offset_mount_back = [-150, 0, 0, 0, 0, 0]

        self.tool_offset_forward = [50, 0, 0, 0, 0, 0]
        self.tool_offset_backward = [-50, 0, 0, 0, 0, 0]

        # tool_get(or return) z offset
        self.tool_offset_up = [0, 0, 60, 0, 0, 0]
        self.tool_offset_down = [0, 0, -60, 0, 0, 0]

        # for 빵 뚜껑 집기
        self.bread_offset_up = [0, 0, 0, 0, 0, 0]
        self.bread_offset_down = [0, 0, 0, 0, 0, 0]

        # for 소스
        self.sauce_offset_forward = [100, 0, 0, 0, 0, 0]
        self.sauce_offset_backward = [-100, 0, 0, 0, 0, 0]

        self.sauce_offset_upward = [0, 0, 100, 0, 0, 0]
        self.sauce_offset_downward = [0, 0, -100, 0, 0, 0]

        
        
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
    #          "MoveOffset",   # ("MoveOffset", self.format_array(self.tool_offset_mount))
    #          "MoveGrip",     # ("MoveGrip", "pick", self.format_array(material_coord), self.format_array(-self.pnp_offset), False)
    #          "MoveSmooth",
    #          "Gripper",      # ("Gripper", True)
    #          "ChangeTool", 
    #          "ChangeParam", 
    #          "ReadCoord", 
    #          "ReadJoint", 
    #          "ReadState"]

    def action_init_pos(self, target_position):
        # setdata1 = self.make_str("ToolBase",  self.format_array([0]))
        setdata = self.make_str("MovePoint", self.format_array(target_position))
        #setdata = '='.join([setdata1, setdata2])
        socke.send_data(setdata)

    def action_vision(self, vision_coord):

        # 1. MV(Material_vision)
        setdata1 = self.make_str("MovePoint", self.format_array(vision_coord))

        setdata = '='.join([setdata1])
        socke.send_data(setdata)

    def action_tool_get(self, tool_index, tool_position): # for test
        setdata = self.make_str("ToolNum",  self.format_array([tool_index]))
        print(tool_index)
        socke.send_data(setdata)

    # def action_tool_get(self, tool_index, tool_position):
    #     # 1. change tool
    #     setdata1 = self.make_str("ToolBase",  self.format_array([tool_index]))
    #     print(tool_index)        
    #     # 2. ME(Material_end)
    #     setdata2 = self.make_str("MovePoint", self.format_array([list(tool_position)[i] + self.tool_offset_mount_back[i] for i in range(len(self.tool_offset_mount_back))]))        
    #     # 3. 그리퍼 열기
    #     setdata3 = self.make_str("Gripper", True)
    #     # 4. 장착하는 방향
    #     setdata4 = self.make_str("MoveOffset", self.format_array(self.tool_offset_mount))
    #     # 5. 위로
    #     setdata5 = self.make_str("MoveOffset",  self.format_array(self.tool_offset_up))
    #     # 6. 뒤로
    #     setdata6 = self.make_str("MoveOffset",  self.format_array(self.tool_offset_mount_back))
    #     # 7. change tool
    #     setdata7 = self.make_str("ToolNum",  self.format_array([tool_index]))
    #     print(tool_index)
        
    #     setdata = '='.join([setdata1, setdata2, setdata3, setdata4, setdata5, setdata6, setdata7])
    #     socke.send_data(setdata)

    # def tool_return(self, tool_coord):
    #     # 1. change tool
    #     setdata1 = self.make_str("ToolBase",  self.format_array([0]))
    #     # 2. ME(Material_end)
    #     setdata2 = self.make_str("MovePoint", self.format_array(list(tool_coord) + self.tool_offset_mount_back + self.tool_offset_up))
    #     # 3. 그리퍼 열기
    #     setdata3 = self.make_str("Gripper", True)
    #     # 4. 오프셋 위로
    #     setdata4 = self.make_str("MoveOffset", self.format_array(self.tool_offset_mount))
    #     # 5. 오프셋 아래로
    #     setdata5 = self.make_str("MoveOffset", self.format_array(self.tool_offset_down))
    #     # 6. 오프셋 뒤로
    #     setdata6 = self.make_str("MoveOffset", self.format_array(self.tool_offset_mount_back))

    #     final_data = '='.join([setdata1, setdata2, setdata3, setdata4, setdata5, setdata6])
    #     socke.send_data(final_data)

    def tool_return(self, tool_coord):
        # 1. change tool
        setdata = self.make_str("ToolBase",  self.format_array([0]))
        socke.send_data(setdata)

    def action_pick(self, grip_mode, material_coord, posture):
        after_pick = list(deepcopy(material_coord))
        # after_pick[3:6] = -90, 0, 180 # 양상추 잡는 각도가 반대라 일단 보류 -> 추후 case 나누거나 중간 점 찾는 등으로 해결
        # 2-1. 그리퍼
        setdata2 = self.make_str("Gripper", True)
        # 2-2. "재료 위치 + 오프셋"으로 이동
        add_list = self.list_add(material_coord, self.pnp_offset)
        add_list.append(posture)
        setdata3 = self.make_str("MovePoint", self.format_array(add_list))
            
            #[list(material_coord)[i] + self.pnp_offset[i] for i in range(len(self.pnp_offset))]))
        # 2-3. 재료 위치로 이동 (오프셋만큼 이동)
        setdata4 = self.make_str("MoveOffset", self.format_array(self.pnp_offset_back))
        # 3. gripper
        setdata5 = self.make_str("Gripper", False)
        # 4. 오프셋
        setdata6 = self.make_str("MovePoint", self.format_array(self.list_add(after_pick, self.pnp_offset)))

        setdata = '='.join([setdata2, setdata3, setdata4, setdata5, setdata6])
        socke.send_data(setdata)
    
    def action_back(self):
        setdata = self.make_str("MoveOffset", self.format_array(self.pnp_offset_x))
        socke.send_data(setdata)

    def action_place(self, place_coord):
        
        # 1. place (MM = 실제 물체 위치 - offset)
        # setdata1 = self.make_str("MoveGrip", "place", self.format_array(material_coord), self.format_array(self.pnp_offset), True)
        ## 1번 임시동작
        # 1-1. "오프셋"으로 이동
        setdata1 = self.make_str("MovePoint", self.format_array([list(place_coord)[i] + self.pnp_offset[i] for i in range(len(self.pnp_offset))]))
        # 1-2. 버거 위치로 이동
        setdata2 = self.make_str("MoveOffset", self.format_array(self.pnp_offset_back))
        # setdata2 = self.make_str("MovePoint", "over_burger") # 버거 위치# control로 부터 받은 place 위치
        # 1-3. 그리퍼
        setdata3 = self.make_str("Gripper", True)
        # 1-4. 오프셋 위로 이동
        setdata4 = self.make_str("MoveOffset", self.format_array(self.pnp_offset))

        setdata = '='.join([setdata1, setdata2, setdata3, setdata4])
        socke.send_data(setdata)

    def action_bread_place(self, place_coord, bread_lib_coord):
        # 1. "오프셋"으로 이동
        setdata1 = self.make_str("MovePoint", self.format_array(self.list_add(place_coord, self.pnp_offset)))

        # 2. 버거 위치로 이동
        setdata2 = self.make_str("MoveOffset", self.format_array(self.pnp_offset_back))

        # 3. 그리퍼 open
        setdata3 = self.make_str("Gripper", True)

        # 4. 위 빵 집기
        setdata4 = self.make_str("MoveOffset", self.format_array(self.bread_offset_up))

        # 5. 그리퍼 close
        setdata5 = self.make_str("Gripper", False)       
         
        # 6. 오프셋 위로 이동
        setdata6 = self.make_str("MoveOffset", self.format_array(self.list_add(self.pnp_offset, self.bread_offset_down)))
        
        # 7. 버거 뚜껑이 위치할 좌표 위로 이동
        setdata7 = self.make_str("MovePoint", self.format_array(self.list_add(bread_lib_coord, self.pnp_offset)))

        # 8. bread_lib 위치로 이동
        setdata8 = self.make_str("MoveOffset", self.format_array(self.pnp_offset_back))
        
        # 9. 그리퍼 open
        setdata9 = self.make_str("Gripper", True)

        # 10. 오프셋 위로 이동
        setdata10 = self.make_str("MoveOffset", self.format_array(self.pnp_offset))                

        setdata = '='.join([setdata1, setdata2, setdata3, setdata4, setdata5, setdata6, setdata7, setdata8, setdata9, setdata10])
        socke.send_data(setdata)

    def action_bread_close(self, bread_coord, place_coord):
        # 1. 그리퍼 열기
        setdata1 = self.make_str("Gripper", True)
        
        # 2. "재료 위치 + 오프셋"으로 이동
        setdata2 = self.make_str("MovePoint", self.format_array(self.list_add(bread_coord, self.pnp_offset)))
            
        # 3. 재료 위치로 이동 (오프셋만큼 이동)
        setdata3 = self.make_str("MoveOffset", self.format_array(self.pnp_offset_back))

        # 4. 그리퍼 닫기
        setdata4 = self.make_str("Gripper", False)

        # 5. 오프셋 위로 이동
        setdata5 = self.make_str("MoveOffset", self.format_array(self.pnp_offset))

        # 6. place위치 + offset 위로 이동
        setdata6 = self.make_str("MovePoint", self.format_array(self.list_add(place_coord, self.pnp_offset)))
        
        # 7. 버거 위치로 이동
        setdata7 = self.make_str("MoveOffset", self.format_array(self.pnp_offset_back))

        # 8. 그리퍼 열기
        setdata8 = self.make_str("Gripper", True)

        # 9. 오프셋 위로 이동
        setdata9 = self.make_str("MoveOffset", self.format_array(self.pnp_offset))

        setdata = '='.join([setdata1, setdata2, setdata3, setdata4, setdata5, setdata6, setdata7, setdata8, setdata9])
        socke.send_data(setdata)    

    def action_sauce_get(self, sauce_coord):
        # 1. 그리퍼 열기
        setdata1 = self.make_str("Gripper", True)

        # 소스 위치 + 오프셋 이동
        setdata2 = self.make_str("MovePoint", self.format_array(self.list_add(sauce_coord+self.sauce_offset_backward)))

        # 소스 위치로 이동
        setdata3 = self.make_str("MoveOffset", self.format_array(self.sauce_offset_forward))
        
        # 위로 이동
        setdata4 = self.make_str("MoveOffset", self.format_array(self.sauce_offset_upward))
        
        # 뒤로 이동
        setdata5 = self.make_str("MoveOffset", self.format_array(self.sauce_offset_backward))

    def action_sauce_pick(self, sauce_coord):
        # 1. 그리퍼 열기
        setdata1 = self.make_str("Gripper", True)

        # 소스 위치 + 오프셋 이동
        setdata2 = self.make_str("MovePoint", self.format_array(self.list_add(sauce_coord+self.sauce_offset_backward)))

        # 소스 위치로 이동
        setdata3 = self.make_str("MoveOffset", self.format_array(self.sauce_offset_forward))
        
        # 위로 이동
        setdata4 = self.make_str("MoveOffset", self.format_array(self.sauce_offset_upward))
        
        # 뒤로 이동
        setdata5 = self.make_str("MoveOffset", self.format_array(self.sauce_offset_backward))





    def list_add(self, list_1, list_2):
        return [list(list_1)[i] + list_2[i] for i in range(len(list_2))]
        
        

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
        print(f"Action: {action_name}, Index: {index}, Grip Mode: {grip_mode}, "
            f"Target Position: {target_position}, Target Position_2: {target_position_2}")
        

        # current_tool = coordinates_data['MaterialList'][index] # json
        # current_tool_coord = coordinates_data['tool_coord'][index] # json
        # print("current tool is", current_tool, "and coordinate is", current_tool_coord)

        # actions
        if action_name == "init_pos":
            action.action_init_pos(target_position)
        elif action_name == "tool_get":
            action.action_tool_get(index, target_position)
        elif action_name == "vision":
            action.action_vision(target_position)
        elif action_name == "pick":
            action.action_pick(index, target_position, posture)
        elif action_name == "place":
            action.action_place(target_position)
        elif action_name == "tool_return":
            action.tool_return(target_position)
        elif action_name == "bread_place":
            action.action_bread_place(target_position, target_position_2)            
        elif action_name == "bread_close":
            action.action_bread_close(target_position, target_position_2)           
        elif action_name == "back":
            action.action_back()                       
                                

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