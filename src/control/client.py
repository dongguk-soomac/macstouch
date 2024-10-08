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
parameters = None

        

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

    # 구현 중
    if action_name == "init_pos":
        parameters.action_init_pos()
    elif action_name == "vision":
        parameters.action_vision()

    print(f"Action: {action_name}, Index: {index}, Grip Mode: {grip_mode}, "
          f"Target Position: {target_position}, Grip Sep: {grip_sep}")
    


## 2. Socket #####################################################

# 소켓 서버 설정
def setup_socket():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(('192.168.0.35', 5000))
    print("1. 서버 소켓이 지정된 IP 주소와 포트에 바인딩 성공")
    server_socket.listen(0)
    print("2. 서버가 클라이언트의 연결을 대기 중")
    client_socket, addr = server_socket.accept()
    print(f"3. 클라이언트 {addr} 연결 성공")
    return server_socket, client_socket


# 소켓 통신에서 행동 완료 신호를 처리하는 함수
def check_action_done(data):
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
    global client_socket, pub_action_done, parameters
    server_socket, client_socket = setup_socket()
    pub_action_done = setup_ros()

    # 파라미터 클래스 초기화 (작성 예정)
    # parameters = Parameters()

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
            check_action_done(data)

        except Exception as e:
            print(f"Error: {e}")
            break

    # 소켓 통신 종료
    client_socket.close()
    server_socket.close()

if __name__ == "__main__":
    main()