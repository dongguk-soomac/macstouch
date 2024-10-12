#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from tkinter import Tk, Button, Label
from selected_menu import menu2pub  

# ROS 초기화 및 설정
rospy.init_node('tkinter_ros_node', anonymous=True)
pub = rospy.Publisher('/menu_list', String, queue_size=10)

# def publish_menu():
#     message = f"{menu2pub}"
#     rospy.loginfo(f"Publishing: {message}")
#     pub.publish(message)

def callback(msg):
    window = Tk()
    window.title("Menu Publish & Error")
    window.geometry("400x200")  # 창 크기 설정

    # "Error" 텍스트를 출력할 라벨 생성 (버튼 위에 위치)
    error_label = Label(window, text=msg.data, fg="black", font=("Arial", 16))
    error_label.pack(padx=20, pady=80)  # 텍스트를 위쪽에 배치

    # 버튼 생성 (텍스트 아래에 위치)
    button = Button(
        window, 
        text="확인", 
        command=window.destroy,  # 버튼 클릭 시 메뉴를 publish
        width=10,
        height=2
    )
    button.pack(pady=20)  # 버튼을 텍스트 아래에 배치

    # Tkinter 창 실행
    window.mainloop()

# ROS 구독자 설정
sub = rospy.Subscriber('/error', String, callback)

# publish_menu()

# ROS 스핀을 계속해서 돌려 메시지 처리
rospy.spin()
