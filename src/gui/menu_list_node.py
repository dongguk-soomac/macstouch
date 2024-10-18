#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from macstouch.msg import order  # order.msg를 생성 후 컴파일하면 자동 생성

# 불고기 버거 = [2, 1, 0, 3, 8, 1, 1, 3]
# 제로 버거 = [2, 0, 0, 3, 8, 1, 1, 3]
# 치즈 버거 = [2, 1, 1, 3, 8, 1, 1, 3]
# 더블 치즈 버거 = [2, 1, 2, 3, 8, 1, 1, 3]
i = 0

def menu_callback(data):
    global i
    rospy.loginfo(f"Received data: {data.data}")

    # # 데이터가 숫자인지 확인
    # if data.data.isdigit():
    #     menu_index = int(data.data)
    # else:
    #     rospy.logwarn("Received invalid data. Not an integer.")
    #     return

    menu_index = data.data

    i += 1
    
    # order 메시지 생성
    order_msg = order()
    order_msg.id = i

    # menu_index 값에 따라 menu 리스트 할당
    if menu_index == "[0]":
        order_msg.menu = [2, 1, 0, 3, 1, 1, 1, 3]  # 불고기 버거
    elif menu_index == "[1]":
        order_msg.menu = [2, 0, 0, 3, 1, 1, 1, 3]  # 제로 버거
    elif menu_index == "[2]":
        order_msg.menu = [2, 1, 1, 3, 1, 1, 1, 3]  # 치즈 버거
    elif menu_index == "[3]":
        order_msg.menu = [2, 1, 2, 3, 1, 1, 1, 3]  # 더블 치즈 버거
    elif menu_index == "[4]":
        order_msg.menu = [2, 1, 0, 3, 1, 1, 1, 3]  # 불고기 버거
    elif menu_index == "[5]":
        order_msg.menu = [2, 0, 0, 3, 1, 1, 1, 3]  # 제로 버거
    elif menu_index == "[6]":
        order_msg.menu = [2, 1, 1, 3, 1, 1, 1, 3]  # 치즈 버거
    elif menu_index == "[7]":
        order_msg.menu = [2, 1, 2, 3, 1, 1, 1, 3]  # 더블 치즈 버거
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

    rospy.loginfo("Listening on /menu_index and publishing to /order...")
    rospy.spin()

if __name__ == '__main__':
    try:
        listener_and_publisher()
    except rospy.ROSInterruptException:
        pass
