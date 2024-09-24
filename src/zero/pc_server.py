#!/usr/bin/python
# -*- coding: utf-8 -*-
import socket 
import time
import rospy
import numpy as np

# msg
from std_msgs.msg import Float32MultiArray as fl
from std_msgs.msg import Bool, Float32, String
from macstouch.msg import action_info

client_socket = None
action = None

## 0. Action ########################################################

class Action:
    def __init__(self) -> None:

        # fixed pos
        self.init_pos = (95, -280, 425, -120, 84, -28)

        # offset
        self.pnp_offset = (0, 0, 10, 0, 0, 0)
        self.tool_offset_mount = (20, 0, 0, 0, 0, 0)
        self.tool_offset_updown = (0, 0, 30, 0, 0, 0)
        self.tool_offset_forwardbackward = (50, 0, 0, 0, 0, 0)

        # mode 참고용
        # modes = ["MovePoint", "MoveOffset", "MoveGrip", "MoveSmooth", "Gripper",
        #        "ChangeTool", "ChangeParam", "ReadCoord", "ReadJoint", "ReadState"]
        
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

    def action_init_pos(self):

        setdata = self.make_str("MovePoint", "init_pos")
        send_data(setdata) 

    def action_vision(self, index, target_position, grip_sep):

        # 1. ME(Material_end)
        setdata = self.make_str("MovePoint", "end_effector", index)
        send_data(setdata) 
        # 2. 장착하는 방향
        setdata = self.make_str("MoveOffset", self.format_array(self.tool_offset_mount))
        send_data(setdata) 
        # 3. 위로
        setdata = self.make_str("MoveOffset",  self.format_array(self.tool_offset_updown))
        send_data(setdata) 
        # 4. 뒤로
        setdata = self.make_str("MoveOffset",  self.format_array(-self.tool_offset_forwardbackward))
        send_data(setdata) 
        # 5. MV(Material_vision)
        setdata = self.make_str("MovePoint", "vision", self.format_array(index))
        send_data(setdata) 

    def action_pnp(self, index, material_coord):
        
        # 1. 뒤로
        setdata = self.make_str("MoveOffset", self.format_array(-self.tool_offset_forwardbackward))
        send_data(setdata)

        # 2. pick (MM = 실제 물체 위치)
        #setdata = self.make_str("MoveGrip", self.format_array(material_coord), self.format_array(-self.pnp_offset), False)
        #send_data(setdata) 
        ## 임시 action
        # 2-1. 그리퍼
        setdata = self.make_str("Gripper", False)
        send_data(setdata) 
        # 2-2. "재료 위치 + 오프셋"으로 이동
        setdata = self.make_str("MovePoint", self.format_array(material_coord+self.pnp_offset))
        send_data(setdata)
        # 2-3. 재료 위치로 이동 (오프셋만큼 이동)
        setdata = self.make_str("MoveOffset", self.format_array(-self.pnp_offset))
        send_data(setdata)

        # 3. gripper
        setdata = self.make_str("Gripper", True)
        send_data(setdata) 
        # 4. 오프셋
        setdata = self.make_str("MoveOffset", self.format_array((self.pnp_offset)))
        send_data(setdata) 

        # 5. place (MM = 실제 물체 위치 - offset)
        #setdata = self.make_str("MoveGrip", self.format_array(material_coord), self.format_array(self.pnp_offset), True)
        #send_data(setdata) 
        ## 임시 action
        # 5-1. "오프셋"으로 이동
        setdata = self.make_str("MoveOffset", self.format_array(self.pnp_offset))
        send_data(setdata)
        # 5-2. 버거 위치로 이동
        setdata = self.make_str("MovePoint", "over_burger")
        send_data(setdata)
        # 5-3. 그리퍼
        setdata = self.make_str("Gripper", False)
        send_data(setdata) 

    def tool_return(self, index):
        
        # 1. ME
        setdata = self.make_str("MovePoint", "end_effector", index)
        send_data(setdata)
        # 2. 오프셋 위로
        setdata = self.make_str("MoveOffset", self.format_array((self.tool_offset_updown)))
        send_data(setdata)
        # 2. 오프셋 앞으로
        setdata = self.make_str("MoveOffset", self.format_array((self.tool_offset_forwardbackward)))
        send_data(setdata)
        # 3. 오프셋 아래로
        setdata = self.make_str("MoveOffset", self.format_array((-self.tool_offset_updown)))
        send_data(setdata)
        # 4. 오프셋 뒤로
        setdata = self.make_str("MoveOffset", self.format_array((-self.tool_offset_forwardbackward)))
        send_data(setdata)

        

## 1. ROS ########################################################

# ROS 초기화
def setup_ros():
    rospy.init_node("client_node")
    sub_action_req = rospy.Subscriber('/action_req', action_info, action_req_callback)
    pub_action_done = rospy.Publisher('/action_done', Bool, queue_size=10)
    return pub_action_done

def action_req_callback(msg):
    
    # 받은 action_info에 따라 작업 처리
    action_name = msg.action        # String
    index = msg.material            # int
    grip_mode = msg.grip_mode       # String
    target_position = msg.coord     # FloatArray
    grip_sep = msg.grip_size        # Float32

    print(f"Action: {action_name}, Index: {index}, Grip Mode: {grip_mode}, "
          f"Target Position: {target_position}, Grip Sep: {grip_sep}")
    
    
    # 구현 중
    if action_name == "init_pos":
        action.action_init_pos()
    elif action_name == "vision":
        action.action_vision(index, target_position, grip_sep)
    elif action_name == "pnp":
        action.action_pnp(index, target_position)
    elif action_name == "tool_return":
        action.tool_return(index)
    


## 2. Socket #####################################################

# 소켓 서버 설정
def setup_socket():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(('192.168.1.23', 5000))
    print("1. 서버 소켓이 지정된 IP 주소와 포트에 바인딩 성공")
    server_socket.listen(5)
    print("2. 서버가 클라이언트의 연결을 대기 중")
    client_socket, addr = server_socket.accept()
    print(f"3. 클라이언트 {addr} 연결 성공")
    return server_socket, client_socket


# 소켓 통신에서 행동 완료 신호를 처리하는 함수
def check_action_done(data, pub_action_done):
    # 소켓에서 수신한 데이터를 분석하여 완료 신호가 있을 때만 action_done을 발행
    if "done" in data.lower():  # 'done' 문자열을 포함하는지 체크 (대소문자 무시)
        print("Action done, publishing /action_done to Control node")
        pub_action_done.publish(True)


# 데이터 송신 함수
def send_data(data):
    try:
        setdata = data.encode()  # 문자열 -> byte code 변환
        client_socket.send(setdata)  # 클라이언트 소켓으로 데이터 송신
    except Exception as e:
        print(f"Error sending data: {e}")



## 3. Main #######################################################

# 메인 함수
def main():
    global client_socket, action
    server_socket, client_socket = setup_socket()
    pub_action_done = setup_ros()

    # 액션 클래스 초기화
    action = Action()

    while not rospy.is_shutdown():
        try:
            # 소켓에서 데이터 수신
            data = client_socket.recv(65535)
            if not data:
                break  # 연결이 종료되면 루프 종료
            data = data.decode()
            print(data)  # 수신된 데이터 출력

            
            # 소켓에서 데이터 송신하는 함수 호출
            setdata = input("input data: ")
            send_data(setdata)

            # 행동 완료 여부를 확인하는 함수 호출
            check_action_done(data, pub_action_done)

        except Exception as e:
            print(f"Error: {e}")
            break

    # 소켓 통신 종료
    client_socket.close()
    server_socket.close()

if __name__ == "__main__":
    main()
