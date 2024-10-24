#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from macstouch.msg import order  # order.msg를 생성 후 컴파일하면 자동 생성
from selected_menu import menu_count, page, menu_index

i = 0
# print("publish message : ", menu_count)

def menu_callback(data):
    global i
    rospy.loginfo(f"Received data: {data.data}")

    # print("publish message : ", data.data)
    # print(type(data.data))

    # a = list(data.data)
    # print(type(a))

    # print(order_msg.menu)

    # for_custom_list = [0, 0, 0, 0, 0, 0, 0, 0, 0]

    # menu_index = data.data

    i += 1

    order_msg = order()
    order_msg.id = i

    print(len(menu_count))

    if (len(menu_count) <= 3):
        print("page 2")
        # print("hello")
        if menu_index == "[0]":
            order_msg.menu = [2, 1, 0, 3, 1, 1, 0, 0, 1]  # 불고기 버거

        elif menu_index == "[1]":
            order_msg.menu = [2, 1, 1, 3, 1, 0, 1, 1, 1]  # 제로 버거

        elif menu_index == '[2]':
            order_msg.menu = [2, 1, 1, 3, 1, 0, 1, 0, 1]  # 치즈 버거

        elif menu_index == '[3]':
            order_msg.menu = [2, 2, 0, 3, 1, 0, 1, 1, 1]  # 더블 패티 버거

        print(order_msg.menu)
            
    elif (len(menu_count) > 1):
        print("page 3")
        order_msg.menu = data.data[:]

    # print("test: {}", order_msg.menu[:])

    # # # 퍼블리시
    order_pub.publish(order_msg)
    rospy.loginfo(f"Published: order_id={order_msg.id}, menu_list={order_msg.menu}")
    rospy.spin()


def listener_and_publisher():
    rospy.init_node('menu_order_node', anonymous=True)

    # 퍼블리셔 생성: /order 토픽에 order 메시지 퍼블리시
    global order_pub

    rospy.Subscriber('/menu_index', String, menu_callback)

    order_pub = rospy.Publisher('/id_order', order, queue_size=1)

    # 서브스크라이버 생성: /menu_index 토픽을 구독
    # rospy.Subscriber('/menu_index', String, menu_callback)

    rospy.loginfo("Listening on /menu_index and publishing to /id_order...")
    rospy.spin()

if __name__ == '__main__':
    try:
        listener_and_publisher()
    except rospy.ROSInterruptException:
        pass
