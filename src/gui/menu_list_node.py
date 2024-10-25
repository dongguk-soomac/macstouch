#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from macstouch.msg import order  # order.msg를 생성 후 컴파일하면 자동 생성

i = 0

def menu_callback(data):
    global i
    rospy.loginfo(f"Received data: {data.data}")

    menu_index = data.data

    i += 1

    # order 메시지 생성
    order_msg = order()
    order_msg.id = i

    # 문자열에서 숫자만 추출하여 리스트로 변환
    menu_list = [int(char) for char in menu_index if char.isdigit()]
    
    rospy.loginfo(f"Extracted menu list: {menu_list}")

    # 조건에 맞게 데이터 처리
    if len(menu_list) > 3:
        order_msg.menu = menu_list[:]
        
    elif len(menu_list) <= 3:
        if menu_index == "[0]":
            order_msg.menu = [2, 1, 1, 3, 1, 1, 0, 1, 1]  # 제로 버거

        elif menu_index == "[1]":
            order_msg.menu = [2, 1, 1, 3, 1, 1, 0, 0, 1]  # 치즈 버거

        elif menu_index == '[2]':
            order_msg.menu = [2, 1, 0, 3, 1, 1, 0, 0, 1]  # 불고기 버거

        elif menu_index == '[3]':
            order_msg.menu = [2, 2, 0, 3, 1, 0, 1, 1, 1]  # 더블 패티 버거

        else:
            rospy.logwarn("Invalid menu index received!")
            return

    # 퍼블리시
    order_pub.publish(order_msg)
    rospy.loginfo(f"Published: order_id={order_msg.id}, menu_list={order_msg.menu}")

def listener_and_publisher():
    rospy.init_node('menu_order_node', anonymous=True)

    # 퍼블리셔 생성: /order 토픽에 order 메시지 퍼블리시
    global order_pub
    order_pub = rospy.Publisher('/id_order', order, queue_size=10)

    # 서브스크라이버 생성: /menu_index 토픽을 구독
    rospy.Subscriber('/menu_index', String, menu_callback)

    rospy.loginfo("Listening on /menu_index and publishing to /id_order...")
    rospy.spin()

if __name__ == '__main__':
    try:
        listener_and_publisher()
    except rospy.ROSInterruptException:
        pass
