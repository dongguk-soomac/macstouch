#!/usr/bin/python
# -*- coding: utf-8 -*-
import socket 
import time 

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM) #INET은 주소패밀리의 기본값, SOCK_STREAM은 소켓 유형의 기본값
sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1) #(level, optname, value: int) 주어진 소켓 옵션의 값을 설정
sock.connect(('192.168.1.23',5000)) #address에 있는 원격 소켓에 연결

Socket_data = 'start' #data 인스턴스 생성
Socket_data = Socket_data.encode() # 유니코드를 utf-8, euc-kr, ascii 형식의 byte코드로 변환
sock.send(Socket_data) #data 전송 


while True:

    message = input("insert msg: ")
    if message.lower() == 'exit':
        break
    sock.send(message.encode())  # 데이터 전송



    Socket_data = sock.recv(65535)  # 서버로부터 데이터 수신
    Socket_data = Socket_data.decode()  # 문자열로 변환
    print(Socket_data)  # 출력

    if Socket_data == "1":  # input data가 1일 경우
        sock.send("p1".encode())  # 데이터 전송
    elif Socket_data == "2":  # input data가 2일 경우
        sock.send("p2".encode())  # 데이터 전송
    # 사용자 입력 추가: 종료 조건 추가
    elif Socket_data == "exit":
        break

sock.close()  # 소켓 종료