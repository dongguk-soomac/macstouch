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

        # offset
        self.pnp_offset = [0, 0, 10, 0, 0, 0]
        self.pnp_offset_back = [0, 0, -10, 0, 0, 0]
        self.tool_offset_mount = [20, 0, 0, 0, 0, 0]
        self.tool_offset_mount_back = [-20, 0, 0, 0, 0, 0]
        self.tool_offset_up = [0, 0, 30, 0, 0, 0]
        self.tool_offset_down = [0, 0, -30, 0, 0, 0]
        self.tool_offset_forward = [50, 0, 0, 0, 0, 0]
        self.tool_offset_backward = [-50, 0, 0, 0, 0, 0]
        
    # 소켓 통신으로 보내는 문자열 생성
    def make_str(self, *args):
        # 전달받은 모든 인자들을 문자열로 변환하고 +로 결합
        setdata = '+'.join(map(str, args))
        return setdata

    # 문자 배열을 0,0,0,0,0,0 형태로 바꿔주는 함수
    def format_array(self, data):
        result = ','.join(map(str, data))
        print(result)
        
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

    def action_init_pos(self):

        setdata = self.make_str("MovePoint", self.format_array(self.init_pos))
        socke.send_data(setdata)

    def action_vision(self, vision_coord):

        # 1. MV(Material_vision)
        setdata1 = self.make_str("MovePoint", self.format_array(vision_coord))

        setdata = '='.join([setdata1])
        socke.send_data(setdata)


    def action_tool_get(self, tool_index, tool_position):
        
        # 1. ME(Material_end)
        setdata1 = self.make_str("MovePoint", self.format_array(list(tool_position)+self.tool_offset_mount))
        # 2. 장착하는 방향
        setdata2 = self.make_str("MoveOffset", self.format_array(self.tool_offset_mount_back))
        # 3. 위로
        setdata3 = self.make_str("MoveOffset",  self.format_array(self.tool_offset_up))
        # 4. 뒤로
        setdata4 = self.make_str("MoveOffset",  self.format_array(self.tool_offset_forward))
        # 5. change tool
        setdata5 = self.make_str("ToolNum",  self.format_array([tool_index]))

        setdata = '='.join([setdata1, setdata2, setdata3, setdata4, setdata5])
        socke.send_data(setdata)


    def action_pick(self, grip_mode, material_coord):
        
        # 1. 뒤로
        # setdata1 = self.make_str("MoveOffset", self.format_array(self.tool_offset_backward))
        # 2. pick (MM = 실제 물체 위치)
        #setdata2 = self.make_str("MoveGrip", "pick", self.format_array(material_coord), self.format_array(-self.pnp_offset), False)
        ## 2번 임시동작
        
        # 2-1. 그리퍼
        setdata2 = self.make_str("Gripper", True)
        # 2-2. "재료 위치 + 오프셋"으로 이동
        setdata3 = self.make_str("MovePoint", self.format_array(list(material_coord) + self.pnp_offset))
        # 2-3. 재료 위치로 이동 (오프셋만큼 이동)
        setdata4 = self.make_str("MoveOffset", self.format_array(self.pnp_offset_back))
        # 3. gripper
        setdata5 = self.make_str("Gripper", False)
        # 4. 오프셋
        setdata6 = self.make_str("MoveOffset", self.format_array(self.pnp_offset))

        setdata = '='.join([setdata2, setdata3, setdata4, setdata5, setdata6])
        socke.send_data(setdata)
    

    def action_place(self, place_coord):
        
        # 1. place (MM = 실제 물체 위치 - offset)
        # setdata1 = self.make_str("MoveGrip", "place", self.format_array(material_coord), self.format_array(self.pnp_offset), True)
        ## 1번 임시동작
        # 1-1. "오프셋"으로 이동
        setdata1 = self.make_str("MovePoint", self.format_array(list(place_coord) + self.pnp_offset)) 
        # 1-2. 버거 위치로 이동
        setdata2 = self.make_str("MoveOffset", self.format_array(self.pnp_offset_back))
        # setdata2 = self.make_str("MovePoint", "over_burger") # 버거 위치# control로 부터 받은 place 위치
        # 1-3. 그리퍼
        setdata3 = self.make_str("Gripper", True)
        # 1-4. 오프셋 위로 이동
        setdata4 = self.make_str("MoveOffset", self.format_array(self.pnp_offset))

        setdata = '='.join([setdata1, setdata2, setdata3, setdata4])
        socke.send_data(setdata)

    def tool_return(self, tool_coord):

        # 1. change tool
        setdata1 = self.make_str("ToolBase",  self.format_array([0]))
        # 2. ME(Material_end)
        setdata2 = self.make_str("MovePoint", self.format_array(list(tool_coord) + self.tool_offset_forward + self.tool_offset_up))
        # 3. 오프셋 위로
        setdata3 = self.make_str("MoveOffset", self.format_array(self.tool_offset_backward))
        # 4. 오프셋 아래로
        setdata4 = self.make_str("MoveOffset", self.format_array(self.tool_offset_down))
        # 5. 오프셋 뒤로
        setdata5 = self.make_str("MoveOffset", self.format_array(self.tool_offset_forward))

        final_data = '='.join([setdata1, setdata2, setdata3, setdata4, setdata5])
        socke.send_data(final_data)
        

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
        grip_sep = msg.grip_size        # Float32

        print(f"Action: {action_name}, Index: {index}, Grip Mode: {grip_mode}, "
            f"Target Position: {target_position}, Grip Sep: {grip_sep}")
        

        current_tool = coordinates_data['MaterialList'][index] # python list
        current_tool_coord = coordinates_data['tool_coord'][index] # json

        print("current tool is", current_tool, "and coordinate is", current_tool_coord)

        # actions
        if action_name == "init_pos":
            action.action_init_pos()
        elif action_name == "tool_get":
            action.action_tool_get(index, target_position)
        elif action_name == "vision":
            action.action_vision(target_position)
        elif action_name == "pick":
            action.action_pick(index, target_position)
        elif action_name == "place":
            action.action_place(target_position)
        elif action_name == "tool_return":
            action.tool_return(target_position)

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