#!/usr/bin/python
# -*- coding: utf-8 -*-

## 1．초기 설정 ①　모듈 가져오기 ######################
from i611_MCS import *
from teachdata import *
from i611_extend import *
from rbsys import *
from i611_common import *
from i611_io import *
from i611shm import * 

from threading import Event # event 모듈
import os # 운영체제 모듈
import time # 시간관련 모듈
import math # 계산관련 모듈
import socket

MaterialList = ["bread", "meat", "cheeze", "pickle", "onion", "sauce", "tomato", "lettuce"]

def th_stop(event):   # thread 함수
    while True:
        if event.is_set():
            return
        if din(9) == '1': # din(9) = 1일 때 프로그램 종료
            print("User Program Stop!")
            pid = os.getpid()
            os.kill(pid, signal.SIGTERM)    #os signal.SIGKILL
        time.sleep(0.1)

def main():

    


    ## 2. 초기 설정 ② ####################################
    # ZERO 로봇 생성자 
    rb = i611Robot()
    # 좌표계의 정의
    _BASE = Base()
    # 로봇과 연결 시작 초기화
    rb.open()
    # I/O 입출력 기능의 초기화 
    IOinit( rb )

    # gripper pin reset
    dout(48, '000')
    
    # 교시 데이터 파일 읽기
    data = Teachdata("teach_data")
    ## 1. 교시 포인트 설정 ######################
    p1 = Position( -418.30, -398.86, 287.00, 0, 0, -180 )
    p2 = Position( -158.54, -395.10, 287.00, 0, 0, -180 )
    ## 2. 동작 조건 설정 ######################## 
    m = MotionParam( jnt_speed=10, lin_speed=10, pose_speed=10, overlap = 30 )
    #MotionParam 형으로 동작 조건 설정
    rb.motionparam( m )


    # initial pose definition
    init_pos = Position(100, 0, 1070, -90, 0, 0)

    # 재료별 엔드이펙터
    end_effector = [

                    Position(0, 0, 0, 0, 0, 0),
                    Position(0, 0, 0, 0, 0, 0),
                    Position(0, 0, 0, 0, 0, 0),
                    Position(0, 0, 0, 0, 0, 0),
                    Position(0, 0, 0, 0, 0, 0),
                    Position(0, 0, 0, 0, 0, 0),
                    Position(0, 0, 0, 0, 0, 0),
                    Position(0, 0, 0, 0, 0, 0),

                ]

    # 재료별 카메라
    vision = [

                Position(0, 0, 0, 0, 0, 0),
                Position(0, 0, 0, 0, 0, 0),
                Position(0, 0, 0, 0, 0, 0),
                Position(0, 0, 0, 0, 0, 0),
                Position(0, 0, 0, 0, 0, 0),
                Position(0, 0, 0, 0, 0, 0),
                Position(0, 0, 0, 0, 0, 0),
                Position(0, 0, 0, 0, 0, 0),
                
            ]

    # 햄버거 위, place 위치
    over_burger = Position(0, 0, 0, 0, 0, 0)


    # Set tool offset
    rb.settool(id=1, offx=0, offy=0, offz=200, offrz=0, offry=0, offrx=0)
    rb.settool(id=2, offx=0, offy=0, offz=200, offrz=0, offry=0, offrx=0)
    rb.settool(id=3, offx=0, offy=0, offz=200, offrz=0, offry=0, offrx=0)
    rb.settool(id=4, offx=0, offy=0, offz=200, offrz=0, offry=0, offrx=0)
    rb.settool(id=5, offx=0, offy=0, offz=200, offrz=0, offry=0, offrx=0)
    rb.settool(id=6, offx=0, offy=0, offz=200, offrz=0, offry=0, offrx=0)
    rb.settool(id=7, offx=0, offy=0, offz=200, offrz=0, offry=0, offrx=0)
    rb.settool(id=8, offx=0, offy=0, offz=200, offrz=0, offry=0, offrx=0)

    # Set motion params
    param = [

        MotionParam(jnt_speed=10, lin_speed=10, pose_speed=10, overlap = 30),
        MotionParam(jnt_speed=10, lin_speed=10, pose_speed=20, overlap = 30)

    ]

    event = Event()
    th = threading.Thread(target=th_stop, args=(event,))  # def된 함수를 thread 생성
    th.setDaemon(True)  # main 함수와 같이 시작하고 끝나도록 daemon 함수로 설정 (병렬동작이 가능하도록 하는 기능)
    th.start()  # thread 동작

    try:

        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # socket() 소켓서버 생성
        sock.bind(('192.168.1.23',5000)) #서버가 사용할 IP주소와 포트번호를 생성한 소켓에 결합
        print("1")
        sock.listen(1) #소켓 서버의 클라이언트의 접속을 기다린다.
        print("2")
        client_socket, addr = sock.accept() #요청 수신되면 요청을 받아들여 데이터 통신을 위한 소켓 생성
        print("3")

        ## 3. 로봇 동작을 정의 ##############################
        # 작업 시작
        Socket_data = 'start' #data 인스턴스 생성
        Socket_data = Socket_data.encode() # 유니코드를 utf-8, euc-kr, ascii 형식의 byte코드로 변환
        sock.send(Socket_data) #data 전송 


        while True:
                
            Socket_data = sock.recv(65535) # server socket으로부터 data 수신
            Socket_data = Socket_data.decode() # byte code -> 문자열 변환
            Socket_data = Socket_data.split('+')


            setdata =input("input data")
            setdata = setdata.encode() #문자열 -> byte code 변환 
            client_socket.send(setdata) #client socket으로 data 송신

            mode = Socket_data[0]

            rb.asyncm(1)

            if mode =="MovePoint":
                
                # 1. MovePoint+init_pos
                if Socket_data[1] == "init_pos":
                    rb.move(init_pos)

                # 2. MovePoint+end_effector+num
                elif Socket_data[1] == "end_effector":
                    rb.move(end_effector[int(Socket_data[2])])

                # 3. MovePoint+vision+num
                elif Socket_data[1] == "vision":
                    rb.move(vision[int(Socket_data[2])])

                # 4. MovePoint+over_burger
                elif Socket_data[1] == "over_burger":
                    rb.move(over_burger)

                # 5. MovePoint+x,y,z,rx,ry,rz
                else:
                    pos = [int(i) for i in Socket_data[1].split(',')]  # 좌표로 명령 시 반드시 6개 정수
                    rb.move(Position(pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]))

                rb.join()

                sock.send("Done")

            elif mode =="MoveOffset":
                offset = [int(i) for i in Socket_data[1].split(',')]  # 좌표로 명령 시 반드시 6개 정수
                rb.relline(dx=offset[0], dy=offset[1], dz=offset[2], drx=offset[3], dry=offset[4], drz=offset[5])

                rb.join()

                sock.send("Done")

            elif mode =="MoveGrip":

                sock.send("Done")

            elif mode =="MoveSmooth":

                sock.send("Done")

            elif mode =="Gripper":
                if Socket_data[1] == "True":
                    dout(48, '1**') # 그리퍼 열기

                elif Socket_data[1] == "False":
                    dout(48, '**1') # 그리퍼 닫기

                rb.sleep(1)
                dout(48, '000')

                sock.send("Done")

            elif mode =="ChangeTool":
                id = int(Socket_data[1])+1
                rb.changetool(tid=id)

                sock.send("Done")

            elif mode =="ChangeParam":
                rb.motionparam(param[int(Socket_data[1])])

                sock.send("Done")

            elif mode =="ReadCoord":
                position_list = shm_read( 0x3000, 6).split(',')

                position_list[0] = round(float(position_list[0])*1000,3) # XY 좌표계 X 위치
                position_list[1] = round(float(position_list[1])*1000,3) # XY 좌표계 Y 위치
                position_list[2] = round(float(position_list[2])*1000,3) # XY 좌표계 Z 위치
                position_list[3] = round(math.degrees(float(position_list[3])),3) # XY 좌표계 Rz 위치
                position_list[4] = round(math.degrees(float(position_list[4])),3) # XY 좌표계 Ry 위치
                position_list[5] = round(math.degrees(float(position_list[5])),3) # XY 좌표계 Rx 위치

                print("x={0}, y={1}, z={2}, rz={3}, ry={4}, rx={5}".format(\
                    position_list[0], position_list[1], position_list[2], position_list[3], position_list[4], position_list[5]))
                
                cur_pos = ','.join(str(p) for p in position_list)
                sock.send(cur_pos)

            elif mode =="ReadJoint":
                joint_list = shm_read( 0x3050, 6 ).split( ',' )

                joint_list[0] = round(math.degrees(float(joint_list[0])),3) # Joint 좌표계 J1 위치
                joint_list[1] = round(math.degrees(float(joint_list[1])),3) # Joint 좌표계 J2 위치
                joint_list[2] = round(math.degrees(float(joint_list[2])),3) # Joint 좌표계 J3 위치
                joint_list[3] = round(math.degrees(float(joint_list[3])),3) # Joint 좌표계 J4 위치
                joint_list[4] = round(math.degrees(float(joint_list[4])),3) # Joint 좌표계 J5 위치
                joint_list[5] = round(math.degrees(float(joint_list[5])),3) # Joint 좌표계 J6 위치

                print("j1={0}, j2={1}, j3={2}, j4={3}, j5={4}, j6={5}".format(\
                    joint_list[0], joint_list[1], joint_list[2], joint_list[3], joint_list[4], joint_list[5]))
                
                cur_jnt = ','.join(str(j) for j in joint_list)
                sock.send(cur_jnt)

            elif mode =="ReadState":

                sock.send("Done")

            rb.asyncm(2)

    except KeyboardInterrupt:           # "ctrl" + "c" 버튼 입력
        print("KeyboardInterrupt")
    except Robot_emo:
        print("Robot_emo")
    except Exception as e:
        print("Error name is : {}".format(e))
    finally:
        print("finally")
        dout(48, '000')

    event.set()
    rb.close()
    sock.close() # 소켓통신 종료
    client_socket.close()

if __name__ == '__main__':
    main()